/*
 * Control_REMAIN.c
 *
 *  控制总成（在 5ms 的 PIT 中断里运行）
 *
 *  1. 上电延时 START_DELAY 内：电机停转、舵机回中
 *  2. 从 midline_info.error 取中线偏差（丢线时沿用上一次，连续丢太久就停车）
 *  3. 舵机：Servo_Set_Turn(error)          —— 比例 / PD 打角
 *  4. 后轮差速：后左轮 = 基础速度 * (1 + ratio)
 *               后右轮 = 基础速度 * (1 - ratio)     ratio = K * 偏差（比例模型，随车速等比放大）
 *               —— 偏差为正（赛道偏右）时车右转，后左轮(外侧)快、后右轮(内侧)慢
 *  5. 速度环：前馈占空比 + PID 修正，输出给后轮电机
 *
 *  Created on: 2026年10月21日
 *      Author: Freed
 */
#include "Control_REMAIN.h"
#include "MIDLine_REMAIN.h"                                 // midline_info
#include "Encoder_REMAIN.h"
#include "Servo_REMAIN.h"
#include "Motor_REMAIN.h"
#include "Pursuit_REMAIN.h"                                 // 纯追踪（逆透视）+ 十字状态机

/* ================================ 纯追踪桥接 ================================
 * 图像在主循环里处理（50Hz），控制在本中断里跑（200Hz）。
 * 中间隔着一个 pursuit_struct 快照，用双缓冲发布，中断里永远读到完整的一份。 */
static pursuit_struct control_ps;                           // 本拍取到的纯追踪快照
static uint8          control_use_pursuit = 0;              // 1 = 本拍用纯追踪

/* ================================ 分档打角桥接 ================================ */
static float control_steer_demand = 0.0f;                   // 本拍打角需求（度，正 = 向右）
static uint8 control_force_center = 0;                      // 1 = 强制回中位（路口专用）

/* 纯追踪算前瞻距离要用的"规划速度"。写在 Control_Tick 里，读在 Pursuit_Process 里。
 * 用规划值而不是实测值：实测值参与会形成"减速->前瞻变近->打角变大->更减速"的正反馈。 */
volatile float control_base_speed_ref = CONTROL_BASE_SPEED;

/* ================================ PID 结构 ================================ */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;                                     // 积分累加
    float integral_max;                                 // 积分限幅
    float last_error;                                   // 上一次误差（微分用）
    float out_max;                                      // 输出限幅
} pid_struct;

static pid_struct pid_l;                                // 左轮速度环
static pid_struct pid_r;                                // 右轮速度环

static float  control_last_error = 0.0f;                // 上一次的中线偏差
static uint16 control_lost_tick  = 0;                   // 连续丢线计数
static uint32 control_tick_cnt   = 0;                   // 节拍计数
static float  control_speed_l    = 0.0f;                // 左轮实测速度（显示用）
static float  control_speed_r    = 0.0f;                // 右轮实测速度（显示用）
static float  control_ff_gain_l   = 0.0f;   // 左轮前馈斜率：占空比 / 速度
static float  control_ff_gain_r   = 0.0f;   // 右轮前馈斜率
static float  control_ff_offset_l = 0.0f;   // 左轮死区补偿（静摩擦）
static float  control_ff_offset_r = 0.0f;   // 右轮死区补偿
static uint8  control_ff_ready  = 0;       // 0-前馈还在标定中（不开闭环）
static int32  control_learn_dead = 0;       // 标定时给了却不转的最后一档占空比
static int32  control_learn_d1   = 0;       // 第一测量点的占空比
static float  control_learn_s1_l = 0.0f;    // 第一测量点左轮平均速度
static float  control_learn_s1_r = 0.0f;    // 第一测量点右轮平均速度
static float  control_learn_s2_l = 0.0f;    // 第二测量点左轮平均速度
static float  control_learn_s2_r = 0.0f;    // 第二测量点右轮平均速度
static uint16 control_learn_cnt  = 0;       // 标定计时（拍）
static uint16 control_learn_n    = 0;       // 实际参与平均的样本数（丢掉启动瞬态后）
static uint8  control_learn_phase = 0;      // 0-爬占空比 1-量第一点 2-量第二点 3-算直线
static int32  control_learn_duty  = 0;      // 当前标定用的占空比
static float  control_learn_sum_l = 0.0f;   // 标定期间左轮速度累加
static float  control_learn_sum_r = 0.0f;   // 标定期间右轮速度累加
static uint8  control_enc_fault  = 0;
static uint16 control_enc_cut    = 0;                   // 故障停车剩余拍数（减到 0 自动恢复）                   // 编码器掉线标志
static uint16 control_enc_fault_cnt = 0;                // 掉线持续拍数
static uint16 control_retry_cnt   = 0;                  // 标定失败重试次数
static uint16 control_over_cnt   = 0;                  // 超速持续拍数（看门狗用）
static uint16 control_calm_cnt   = 0;                  // 停车后等轮子停稳的拍数
static uint8  control_over_retry  = 0;                // 看门狗已经触发过几次
static uint8  control_ff_fallback = 0;                // 1-放弃自动标定，用硬编码前馈
static float  control_noise_l      = 0.0f;             // 电机断电、轮子静止时测到的噪声底（脉冲/5ms）
static float  control_noise_r      = 0.0f;
static float  control_noise_peak_l = 0.0f;
static float  control_noise_peak_r = 0.0f;
static uint8  control_noise_ready  = 0;
static uint8  control_loop_off     = 0;                // 1-编码器不可信，放弃闭环，用固定占空比开环

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID 初始化
// 参数说明     pid             结构体指针
// 参数说明     kp/ki/kd        参数
// 参数说明     out_max         输出限幅
// 返回参数     void
// 备注信息     积分限幅取 out_max/ki，防止积分饱和
//-------------------------------------------------------------------------------------------------------------------
static void pid_init(pid_struct *pid, float kp, float ki, float kd, float out_max)
{
    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->integral     = 0.0f;
    pid->last_error   = 0.0f;
    pid->out_max      = out_max;
    pid->integral_max = (ki > 0.001f) ? (out_max / ki) : (0.0f);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     位置式 PID 计算
// 参数说明     pid             结构体指针
// 参数说明     target          目标值
// 参数说明     actual          实际值
// 返回参数     float           输出（已限幅）
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static float pid_calc(pid_struct *pid, float target, float actual)
{
    float error = target - actual;
    float out;

    pid->integral += error;
    if(pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if(pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;

    out = (pid->kp * error) +
          (pid->ki * pid->integral) +
          (pid->kd * (error - pid->last_error));

    pid->last_error = error;

    if(out >  pid->out_max) out =  pid->out_max;
    if(out < -pid->out_max) out = -pid->out_max;

    return (out);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     控制模块初始化
// 参数说明     void
// 返回参数     void
// 使用示例     Control_Init();
// 备注信息     必须在 PIT_Init() 之前调用
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     两点定直线：duty = k * speed + b（左右轮各算一条）
// 参数说明     d1      第一标定点的占空比
// 参数说明     s1      第一标定点的实测速度
// 参数说明     d2      第二标定点的占空比
// 参数说明     s2      第二标定点的实测速度
// 参数说明     b       输出：截距（死区补偿）
// 参数说明     dead    单点模式用的死区估计
// 参数说明     force_single  1 = 强制单点模式
// 返回参数     k       斜率；返回 <= 0 表示这根线不可信（轮子没转/编码器没信号）
// 使用示例     k = control_line_fit(d1, s1, d2, s2, &b, dead, 0);
// 备注信息     两点速度拉不开就退回单点公式，宁可粗糙也别把系数算飞
//-------------------------------------------------------------------------------------------------------------------
static float control_line_fit(float d1, float s1, float d2, float s2, float *b, float dead, uint8 force_single)
{
    float k = 0.0f;

    if((0 == force_single) && (s2 > (s1 + 1.0f)) && (d2 > d1))
    {
        k  = (d2 - d1) / (s2 - s1);         // 正常情况：两点定斜率
        *b = d1 - k * s1;
    }
    else if(s1 > 0.5f)
    {
        k  = (d1 - dead) / s1;              // 第二点没量到有效速度：退回单点 + 死区
        *b = dead;
    }
    else
    {
        k  = 0.0f;                          // 两点都没速度：这根线作废
        *b = 0.0f;
    }

    if(k < 0.0f)
    {
        k  = 0.0f;
        *b = 0.0f;
    }

    return (k);
}

void Control_Init(void)
{
    pid_init(&pid_l, CONTROL_PID_KP, CONTROL_PID_KI, CONTROL_PID_KD, CONTROL_PID_OUT_MAX);
    pid_init(&pid_r, CONTROL_PID_KP, CONTROL_PID_KI, CONTROL_PID_KD, CONTROL_PID_OUT_MAX);

    control_last_error  = 0.0f;
    control_lost_tick   = 0;
    control_tick_cnt    = 0;
    /* 上电直接用实测系数，不标定：轮到就跑到目标速度，省掉 1.5 s 标定过程 */
    control_ff_gain_l   = CONTROL_FF_GAIN_L_DEFAULT;
    control_ff_gain_r   = CONTROL_FF_GAIN_R_DEFAULT;
    control_ff_offset_l = CONTROL_FF_OFFSET_L_DEFAULT;
    control_ff_offset_r = CONTROL_FF_OFFSET_R_DEFAULT;
    control_ff_ready    = 1;                        /* 默认已标定好，看门狗和输出段直接用 */
    control_learn_dead  = 0;
    control_learn_d1    = 0;
    control_learn_s1_l  = 0.0f;
    control_learn_s1_r  = 0.0f;
    control_learn_s2_l  = 0.0f;
    control_learn_s2_r  = 0.0f;
    control_learn_cnt   = 0;
    control_learn_n     = 0;
    control_learn_phase = 0;
    control_learn_duty  = CONTROL_FF_LEARN_DUTY_MIN;
    control_learn_sum_l = 0.0f;
    control_learn_sum_r = 0.0f;
    control_enc_fault   = 0;
    control_enc_fault_cnt = 0;
    control_enc_cut     = 0;
    control_retry_cnt   = 0;
    control_over_cnt    = 0;
    control_calm_cnt    = 0;
    control_over_retry  = 0;
    control_ff_fallback = 0;
    control_noise_l      = 0.0f;
    control_noise_r      = 0.0f;
    control_noise_peak_l = 0.0f;
    control_noise_peak_r = 0.0f;
    control_noise_ready  = 0;
    control_loop_off     = 0;
    control_speed_l     = 0.0f;
    control_speed_r     = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     一次完整控制（5ms 周期中断里调用）
// 参数说明     void
// 返回参数     void
// 使用示例     Control_Tick();       // 在 cc60_pit_ch0_isr 里
// 备注信息     1. 中断里只做控制，不做图像处理
//              2. 浮点运算在 TC264 上有 FPU，200Hz 完全没有压力
//-------------------------------------------------------------------------------------------------------------------
void Control_Tick(void)
{
    float error    = 0.0f;
    float speed_l  = 0.0f;
    float speed_r  = 0.0f;
    float target_l = 0.0f;
    float target_r = 0.0f;
    float duty_l   = 0.0f;
    float duty_r   = 0.0f;
    float ratio      = 0.0f;
    float abs_err    = 0.0f;
    float slow       = 1.0f;
    float base_speed = CONTROL_BASE_SPEED;
    int16 raw_l = 0;
    int16 raw_r = 0;

    control_tick_cnt ++;

    // 1. 发车条件：上电先等 CONTROL_START_DELAY_MS（顺便测编码器噪声底），之后由斑马线决定走不走
#if (1 == ZEBRA_EN)
    if((0 == zebra_info.state) || (2 == zebra_info.state) ||
       ((control_tick_cnt * CONTROL_PERIOD_MS) < CONTROL_START_DELAY_MS))
#else
    if((control_tick_cnt * CONTROL_PERIOD_MS) < CONTROL_START_DELAY_MS)
#endif
    {
        /* 这 3 s 电机是断电的、轮子静止，任何脉冲都是电气干扰，正好拿来测噪声底 */
        raw_l = Encoder_Get_Count(0);
        raw_r = Encoder_Get_Count(1);
        if(((raw_l < 0) ? (-(float)raw_l) : ((float)raw_l)) > control_noise_peak_l)
        {
            control_noise_peak_l = (raw_l < 0) ? (-(float)raw_l) : ((float)raw_l);
        }
        if(((raw_r < 0) ? (-(float)raw_r) : ((float)raw_r)) > control_noise_peak_r)
        {
            control_noise_peak_r = (raw_r < 0) ? (-(float)raw_r) : ((float)raw_r);
        }
        Motor_Stop();
        Servo_Set_Duty(Servo_MID);
        return;
    }

    if(0 == control_noise_ready)
    {
        control_noise_ready = 1;
        control_noise_l = control_noise_peak_l + 1.0f;
        control_noise_r = control_noise_peak_r + 1.0f;
        /* 封顶 3：目标速度本身只有 10，门限开太大就会把真实低速也清零（表现为 R 时不时跳 0） */
        if(control_noise_l > 3.0f)
        {
            control_noise_l = 3.0f;
        }
        if(control_noise_r > 3.0f)
        {
            control_noise_r = 3.0f;
        }

        if((control_noise_l > CONTROL_NOISE_MAX_OK) || (control_noise_r > CONTROL_NOISE_MAX_OK))
        {
            /* 静止时就有几十个脉冲：编码器线被电机 PWM 干扰，闭环读数不可信 -> 退回开环 */
            control_loop_off = 1;
        }
    }

#if (1 == CONTROL_ENCODER_TEST_EN)
    /* 编码器静态自检：电机断电，直接显示每 5ms 的原始脉冲数。
       用手慢慢转动后轮，V/R 会跟着变正/变负 -> 编码器和方向都正常。 */
    control_speed_l = (float)Encoder_Get_Count(0);
    control_speed_r = (float)Encoder_Get_Count(1);
    Motor_Stop();
    Servo_Set_Duty(Servo_MID);
    return;
#endif

    // 2. 取中线偏差
    if(midline_info.valid)
    {
        error             = midline_info.error;
        control_last_error = error;
        control_lost_tick  = 0;
    }
    else                                                // 丢线：沿用上一次的偏差
    {
        error = control_last_error;
        if(control_lost_tick < 60000)
        {
            control_lost_tick ++;
        }
    }

    if((CONTROL_LOST_STOP_TICKS > 0) && (control_lost_tick > CONTROL_LOST_STOP_TICKS))
    {
        Motor_Stop();                                   // 长时间看不到赛道，停车保护
        Servo_Set_Duty(Servo_MID);
        return;
    }

    // 3. 舵机：纯追踪优先，算不出来再退回"偏差 -> 分段 PD"
#if (1 == CONTROL_PURSUIT_EN)
    control_use_pursuit = Pursuit_Get(&control_ps);      // 0 = 纯追踪这一帧不可信（标定表没好/视野太差）
#endif

#if (1 == CONTROL_STEP_STEER_EN)
    /* 分档打角：舵机只认 5 个档位（460 / 440 / 474 / 418 / 500）。
     * 1) 打角需求：优先用纯追踪算出来的角度；没有纯追踪就把中线偏差按比例换成角度。
     * 2) 路口：force_center = 1 → Servo_Set_Step 立刻回中位 460，无论需求是多少。
     *    这是"十字路口一定直行"的硬保险：纯追踪说路口、或老算法说路口，任意一个成立就回中。 */
    control_steer_demand = 0.0f;
    control_force_center = 0;

    if(1 == control_use_pursuit)
    {
        control_steer_demand = control_ps.delta_deg;                 // 正 = 向右
        if(0 != control_ps.cross)
        {
            control_force_center = 1;
        }
    }
    else
    {
        control_steer_demand = error * CONTROL_ERR_PX_TO_DEG;        // 正偏差 = 赛道偏右 = 往右打
        if(midline_info.cross)
        {
            control_force_center = 1;
        }
    }

    if(0 != control_force_center)
    {
        Servo_Clear_Hold();                                          // 路口不要残留出弯保持
    }

    Servo_Set_Step(control_steer_demand, control_force_center);
#else
    if(1 == control_use_pursuit)
    {
        /* 纯追踪输出的已经是"该打多少"，不需要再叠一层增益；内部只做限幅和斜率限制 */
        Servo_Set_Offset(control_ps.steer_out);
    }
    else
    {
        if(midline_info.cross)                          // 路口：先取消出弯保持，别带着旧打角冲出去
        {
            Servo_Clear_Hold();
        }
        Servo_Set_Turn(error);
    }
#endif

    // 4. 读编码器：每 5ms 读一次并清零，读到的脉冲数就是"速度"
    speed_l = Encoder_Get_Speed(0);
    speed_r = Encoder_Get_Speed(1);
    control_speed_l = speed_l;
    control_speed_r = speed_r;
#if (1 == CONTROL_SPEED_NOISE_GATE)
    if((speed_l < control_noise_l) && (speed_l > -control_noise_l))
    {
        speed_l = 0.0f;
    }
    if((speed_r < control_noise_r) && (speed_r > -control_noise_r))
    {
        speed_r = 0.0f;
    }
    control_speed_l = speed_l;
    control_speed_r = speed_r;
#endif

#if (1 == CONTROL_OVERSPEED_EN)
    /* 超速看门狗：闭环在跑，速度却远超目标 -> 前馈系数一定是错的（标定期间轮子没转）。
       不停车的话车会一直全速冲，所以这里停 0.5 s 并把标定结果作废重来。 */
    if(control_ff_ready && ((speed_l > CONTROL_OVERSPEED_SPEED) || (speed_r > CONTROL_OVERSPEED_SPEED)))
    {
        if(control_over_cnt < 60000)
        {
            control_over_cnt ++;
        }
        if(control_over_cnt >= CONTROL_OVERSPEED_TICKS)
        {
            control_over_cnt  = 0;
            control_over_retry ++;
            if(control_over_retry > 2)          /* 连标两次都飞：不再相信自动标定，退回 v1.0 的硬编码前馈 */
            {
                control_ff_fallback = 1;
                control_ff_gain_l   = (float)CONTROL_DUTY_BASE / CONTROL_BASE_SPEED;
                control_ff_gain_r   = control_ff_gain_l;
                control_ff_offset_l = 0.0f;
                control_ff_offset_r = 0.0f;
                control_ff_ready    = 1;
                pid_l.integral      = 0.0f;
                pid_r.integral      = 0.0f;
                Motor_Stop();
                return;
            }
            control_enc_cut   = CONTROL_ENC_CUT_TICKS;
            control_ff_ready  = 0;
            control_learn_phase = 0;
            control_learn_duty  = CONTROL_FF_LEARN_DUTY_MIN;
            control_learn_cnt   = 0;
            control_calm_cnt    = 200;              /* 1 s 等轮子停稳再重标 */
            pid_l.integral = 0.0f;
            pid_r.integral = 0.0f;
            Motor_Stop();
            return;
        }
    }
    else
    {
        control_over_cnt = 0;
    }
#endif

    // 5. 前馈系数自动标定：上电第一次发车先开环跑一小段，量出"占空比 -> 速度"
#if (1 == CONTROL_FF_LEARN_EN)
    /* 上电第一次发车前馈自标定，左右轮分开标：
         duty_l = gain_l * speed_l + offset_l
         duty_r = gain_r * speed_r + offset_r
       为什么必须分开：两个电机的减速箱、静摩擦、编码器线数都可能不一样。
       实测这辆车同一个占空比下右轮读数能是左轮的 7 倍（V11 / R80），
       用同一个系数的话一边飞一边爬，差速和直道都会跑偏。
       流程：
         0 爬占空比找到"刚好能转"的 d1，同时把上一档记成死区
         1 保持 d1 量出 s1_l / s1_r
         2 抬到 d2 = d1 + BOOST 再量出 s2_l / s2_r
         3 左右各用两点定一条直线 */
    if((0 == control_ff_ready) && (0 == control_loop_off))
    {                                                   /* 标定没完成才进这一大块 */
        /* 刚被看门狗停车时轮子还在惯性转，这时候量"能不能转"会被骗（明明会转却记成死区），
           所以先空转 1 s 等它停稳再重新标。 */
        if(control_calm_cnt > 0)
        {
            control_calm_cnt --;
            Motor_Stop();
            return;
        }

    {
        if(0 == control_learn_phase)                    /* ---- 0 爬占空比 ---- */
        {
            Motor_Set_Duty(control_learn_duty, control_learn_duty);

            if((speed_l > CONTROL_FF_MIN_SPEED) || (speed_r > CONTROL_FF_MIN_SPEED)      /* 任一轮转起来就行，单路编码器坏了也能标 */)
            {
                control_learn_d1    = control_learn_duty;   // 两轮都能转的最小占空比
                control_learn_phase = 1;
                control_learn_cnt   = 0;
                control_learn_n     = 0;
                control_learn_sum_l = 0.0f;
                control_learn_sum_r = 0.0f;
                return;
            }

            control_learn_cnt ++;
            if(control_learn_cnt >= CONTROL_FF_LEARN_RAMP_TICKS)   // 这一档给了 RAMP_TICKS 拍还拖不动
            {
                control_learn_cnt   = 0;
                control_learn_dead  = control_learn_duty;   // 记成死区估计
                control_learn_duty += CONTROL_FF_LEARN_STEP;
                if(control_learn_duty > CONTROL_FF_LEARN_DUTY_MAX)
                {
                    control_enc_fault = 1;                  // 加到上限还不转：编码器没信号 / 电机没供电
                    control_enc_cut   = CONTROL_ENC_CUT_TICKS;
                    control_ff_gain_l = (float)CONTROL_DUTY_BASE / CONTROL_BASE_SPEED;
                    control_ff_gain_r = control_ff_gain_l;
                    control_ff_offset_l = 0.0f;
                    control_ff_offset_r = 0.0f;
                    control_ff_ready  = 1;
                }
            }
            return;
        }

        if((1 == control_learn_phase) || (2 == control_learn_phase))    /* ---- 1/2 量速度 ---- */
        {
            Motor_Set_Duty(control_learn_duty, control_learn_duty);

            if(control_learn_cnt < 60000)
            {
                control_learn_cnt ++;
            }
            if(control_learn_cnt > CONTROL_FF_LEARN_SKIP)
            {
                control_learn_sum_l += speed_l;
                control_learn_sum_r += speed_r;
                if(control_learn_n < 60000)
                {
                    control_learn_n ++;
                }
            }

            if(control_learn_cnt >= CONTROL_FF_LEARN_TICKS)
            {
                float nl = (control_learn_n > 0) ? (1.0f / (float)control_learn_n) : (0.0f);
                float avg_l = control_learn_sum_l * nl;
                float avg_r = control_learn_sum_r * nl;

                if(1 == control_learn_phase)
                {
                    control_learn_s1_l   = avg_l;           // 第一点速度
                    control_learn_s1_r   = avg_r;
                    control_learn_duty  += CONTROL_FF_LEARN_BOOST;
                    if(control_learn_duty > CONTROL_FF_LEARN_DUTY_MAX) control_learn_duty = CONTROL_FF_LEARN_DUTY_MAX;
                    control_learn_phase  = 2;
                    control_learn_cnt    = 0;
                    control_learn_n      = 0;
                    control_learn_sum_l  = 0.0f;
                    control_learn_sum_r  = 0.0f;
                }
                else
                {
                    control_learn_s2_l  = avg_l;            // 第二点速度
                    control_learn_s2_r  = avg_r;
                    control_learn_phase = 3;
                }
            }
            return;
        }

        {                                               /* ---- 3 左右各算一条直线 ---- */
            float d1 = (float)control_learn_d1;
            float d2 = (float)control_learn_duty;
            float kl = 0.0f, bl = 0.0f, kr = 0.0f, br = 0.0f;

            kl = control_line_fit(d1, control_learn_s1_l, d2, control_learn_s2_l, &bl,
                                  (float)control_learn_dead, 0);
            kr = control_line_fit(d1, control_learn_s1_r, d2, control_learn_s2_r, &br,
                                  (float)control_learn_dead, 0);

            /* 某一轮全程没转（编码器没接 / 轮子卡住）：借用另一轮的系数，至少能跑 */
            if((kl <= 0.0f) && (kr > 0.0f)) { kl = kr; bl = br; }
            if((kr <= 0.0f) && (kl > 0.0f)) { kr = kl; br = bl; }
            if(kl <= 0.0f)
            {
                kl = (float)CONTROL_DUTY_BASE / CONTROL_BASE_SPEED;
                kr = kl;
                bl = 0.0f;
                br = 0.0f;
                control_enc_fault = 1;                  // 两轮都读不到速度
                control_enc_cut   = CONTROL_ENC_CUT_TICKS;
            }

            if(bl < 0.0f) bl = 0.0f;
            if(br < 0.0f) br = 0.0f;
            if(bl > CONTROL_FF_OFFSET_MAX) bl = CONTROL_FF_OFFSET_MAX;
            if(br > CONTROL_FF_OFFSET_MAX) br = CONTROL_FF_OFFSET_MAX;
            if(kl < CONTROL_FF_GAIN_MIN) kl = CONTROL_FF_GAIN_MIN;
            if(kr < CONTROL_FF_GAIN_MIN) kr = CONTROL_FF_GAIN_MIN;
            if(kl > CONTROL_FF_GAIN_MAX) kl = CONTROL_FF_GAIN_MAX;
            if(kr > CONTROL_FF_GAIN_MAX) kr = CONTROL_FF_GAIN_MAX;

            /* 合理性校验：用标出来的系数反算 base 速度要多少占空比。
               正常是 900~2000；算出 4000 以上说明标定期间轮子根本没转起来，
               这组系数会把占空比一直顶满（车会窜出去），宁可判失败重标。 */
            if((kl * CONTROL_BASE_SPEED + bl) > CONTROL_FF_DUTY_AT_BASE_MAX)
            {
                kl = 0.0f;
            }
            if((kr * CONTROL_BASE_SPEED + br) > CONTROL_FF_DUTY_AT_BASE_MAX)
            {
                kr = 0.0f;
            }
            if((kl <= 0.0f) && (kr > 0.0f)) { kl = kr; bl = br; }
            if((kr <= 0.0f) && (kl > 0.0f)) { kr = kl; br = bl; }
            if((kl <= 0.0f) && (kr <= 0.0f))
            {
                control_enc_fault = 1;
                control_enc_cut   = CONTROL_ENC_CUT_TICKS;
                control_ff_ready  = 1;
                control_ff_gain_l = (float)CONTROL_DUTY_BASE / CONTROL_BASE_SPEED;
                control_ff_gain_r = control_ff_gain_l;
                control_ff_offset_l = 0.0f;
                control_ff_offset_r = 0.0f;
                return;
            }

            control_ff_gain_l   = kl;
            control_ff_gain_r   = kr;
            control_ff_offset_l = bl;
            control_ff_offset_r = br;

            control_retry_cnt = 0;
            control_ff_ready  = 1;
        }
        return;                                         // 标定期间不开闭环
    }
    }
#endif
    if(control_ff_gain_l <= 0.0f)
    {
        control_ff_gain_l = (float)CONTROL_DUTY_BASE / CONTROL_BASE_SPEED;
        control_ff_gain_r = control_ff_gain_l;
    }

    // 6. 弯道减速 + 后轮差速
    //    纯追踪接管时：slow 由前轮转角直接给出，ratio 由曲率乘后轮距给出，都是几何物理量
    //    退回老算法时：偏差越大 -> 弯越急 -> 基础速度先降下来，避免高速冲弯打不动方向
    if(1 == control_use_pursuit)
    {
        slow  = control_ps.slow;
        ratio = control_ps.ratio;
        if(control_ps.cross)                            // 路口横臂补出来的线不可信，慢一点
        {
            slow = slow * CONTROL_CROSS_SPEED_SCALE;
        }
        if(control_ps.off_track)                        // 车已经压到赛道边缘，先保命
        {
            slow = slow * CONTROL_OFFTRACK_SPEED_SCALE;
        }
        if(slow < 0.25f)
        {
            slow = 0.25f;
        }
        base_speed = CONTROL_BASE_SPEED * slow;
    }
    else
    {
        abs_err = (error > 0.0f) ? (error) : (-error);
        slow    = 1.0f - (CONTROL_CURVE_SLOW_K * abs_err);
        if(slow < CONTROL_CURVE_SLOW_MIN) slow = CONTROL_CURVE_SLOW_MIN;
        base_speed = CONTROL_BASE_SPEED * slow;

        //    后轮差速：偏差为正（赛道偏右）-> 车的右转弯 -> 后左轮(外侧)快、后右轮(内侧)慢
        ratio = CONTROL_DIFF_K * error;
        if(ratio >  CONTROL_DIFF_K_MAX) ratio =  CONTROL_DIFF_K_MAX;
        if(ratio < -CONTROL_DIFF_K_MAX) ratio = -CONTROL_DIFF_K_MAX;
    }
#if (1 == CONTROL_DIFF_REVERSE)
    ratio = -ratio;                                     // 差速方向反了的时候用
#endif

    control_base_speed_ref = base_speed;                // 下一帧纯追踪用它算前瞻距离

    target_l = base_speed * (1.0f + ratio);             // 后左轮
    target_r = base_speed * (1.0f - ratio);             // 后右轮

    if(target_l > CONTROL_SPEED_MAX) target_l = CONTROL_SPEED_MAX;
    if(target_r > CONTROL_SPEED_MAX) target_r = CONTROL_SPEED_MAX;
    if(target_l < CONTROL_SPEED_MIN) target_l = CONTROL_SPEED_MIN;   // 内侧轮不要停转，否则等于拖刹
    if(target_r < CONTROL_SPEED_MIN) target_r = CONTROL_SPEED_MIN;

    // 7. 速度环：前馈（标定出来的系数）+ PID 修正
    if(control_loop_off)                                /* 编码器不可信：按 v1.0 的固定占空比开环跑 */
    {
        duty_l = (float)CONTROL_DUTY_BASE * (1.0f + ratio) * slow;
        duty_r = (float)CONTROL_DUTY_BASE * (1.0f - ratio) * slow;
    }
    else
    {
        duty_l = control_ff_gain_l * target_l + control_ff_offset_l;   // 左轮：斜率 x 目标 + 死区补偿
        duty_r = control_ff_gain_r * target_r + control_ff_offset_r;   // 右轮：左右轮分开标
    }

#if (1 == CONTROL_SPEED_LOOP_EN)
    if(0 == control_loop_off)
    {
        duty_l += pid_calc(&pid_l, target_l, speed_l);      // PID 只做修正，主出力靠前馈
        duty_r += pid_calc(&pid_r, target_r, speed_r);
    }
#endif

    if(duty_l < 0.0f) duty_l = 0.0f;
    if(duty_r < 0.0f) duty_r = 0.0f;

    // 8. 编码器掉线保护：给了占空比、轮子却一直没速度 -> 停车 + 屏幕显示 E
#if (1 == CONTROL_ENC_FAULT_EN)
    /* 检测：占空比够大、两轮速度却都接近 0，连续 CONTROL_ENC_FAULT_TICKS 拍 -> 判掉线 */
    if((0 == control_enc_fault) && (((duty_l + duty_r) * 0.5f) > CONTROL_ENC_FAULT_DUTY))
    {
        if((speed_l < 1.0f) && (speed_l > -1.0f) && (speed_r < 1.0f) && (speed_r > -1.0f))
        {
            if(control_enc_fault_cnt < 60000)
            {
                control_enc_fault_cnt ++;
            }
            if(control_enc_fault_cnt > CONTROL_ENC_FAULT_TICKS)
            {
                control_enc_fault     = 1;
                control_enc_cut       = CONTROL_ENC_CUT_TICKS;   // 停车 0.5 s 后自动重试
                control_enc_fault_cnt = 0;
            }
        }
        else
        {
            control_enc_fault_cnt = 0;
        }
    }
    else
    {
        control_enc_fault_cnt = 0;
    }

    /* 停车期：清积分 + 断电；时间到自动恢复。
       为什么要自动恢复而不是锁死：如果目标速度本身低于电机死区，轮子转不动是正常现象，
       锁死会让车永远停在原地，还得重新上电。 */
    if(control_enc_cut > 0)
    {
        control_enc_cut --;
        pid_l.integral = 0.0f;                          // 别让积分继续憋着，否则一恢复就窜
        pid_r.integral = 0.0f;
        Motor_Stop();
        if(0 == control_enc_cut)
        {
            /* 停车结束：清故障并重新标定一次。
               为什么重新标定而不是沿用上次结果：上电瞬间可能还没读到速度就误判了，
               重试往往一次就过；只有连续失败 CONTROL_ENC_RETRY_MAX 次才彻底放弃。 */
            if(control_retry_cnt < CONTROL_ENC_RETRY_MAX)
            {
                control_retry_cnt ++;
                control_enc_fault   = 0;
                control_ff_ready    = 0;
                control_learn_phase = 0;
                control_learn_duty  = CONTROL_FF_LEARN_DUTY_MIN;
                control_learn_cnt   = 0;
            }
            else
            {
                control_enc_fault = 1;                  // 放弃重试：E 常亮当告警，但不再停车
                control_ff_offset_l = (float)CONTROL_FF_LEARN_DUTY_MIN;   // 给个能转的最低垫量，别让车趴窝
                control_ff_offset_r = (float)CONTROL_FF_LEARN_DUTY_MIN;
            }
        }
        return;
    }
#endif

    Motor_Set_Duty((int32)duty_l, (int32)duty_r);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在图像左上角显示实测轮速（6x8 小字体，只占 8 像素高）
// 参数说明     void
// 返回参数     void
// 使用示例     Control_Show_On_TFT180();     // 放在 SATA_Frame_Process() 的最后
// 备注信息     左/右轮各 3 位数字，单位是"脉冲/5ms"；
//              如果车在动而这里一直是 000，说明编码器没接对/定时器选错，
//              先把 CONTROL_SPEED_LOOP_EN 改成 0 用开环跑，再排编码器
//-------------------------------------------------------------------------------------------------------------------
void Control_Show_On_TFT180(void)
{
    /* 画在图像下方的第 5 行（y = CAMERA_INFO_Y5 = 148），6x8 小字占 8 像素高，
       不会压到上面的二值化图像。以前画在 (0,0) 会把图像左上角盖掉。 */
    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(RGB565_YELLOW, RGB565_BLACK);
    tft180_show_string(0,  CAMERA_INFO_Y5, "V");
    tft180_show_int(8,     CAMERA_INFO_Y5, (int32)control_speed_l, 3);
    tft180_show_string(30, CAMERA_INFO_Y5, "R");
    tft180_show_int(38,    CAMERA_INFO_Y5, (int32)control_speed_r, 3);
    tft180_show_string(66, CAMERA_INFO_Y5, "G");                   // G = 当前前馈系数（占空比/速度）
    tft180_show_int(74,    CAMERA_INFO_Y5, (int32)((control_ff_gain_l + control_ff_gain_r) * 0.5f), 3);

#if (0 == CAMERA_DIS_IMAGE)        /* 第 6 行只有关掉图像显示时才画，否则会超出 160 行触发断言 */
    /* 第 6 行：标定出来的 4 个数，g 是斜率（占空比/速度），b 是死区补偿。
       看这一行就知道标定靠不靠谱：b 应该在 700~1100，g 在两轮之间可以差好几倍。 */
    tft180_set_color(RGB565_YELLOW, RGB565_BLACK);
    tft180_show_string(0,  CAMERA_INFO_Y6, "g");
    tft180_show_int(8,     CAMERA_INFO_Y6, (int32)control_ff_gain_l, 3);
    tft180_show_string(26, CAMERA_INFO_Y6, "/");
    tft180_show_int(30,    CAMERA_INFO_Y6, (int32)control_ff_gain_r, 3);
    tft180_show_string(48, CAMERA_INFO_Y6, "b");
    tft180_show_int(56,    CAMERA_INFO_Y6, (int32)control_ff_offset_l, 4);
    tft180_show_string(80, CAMERA_INFO_Y6, "/");
    tft180_show_int(84,    CAMERA_INFO_Y6, (int32)control_ff_offset_r, 4);
#endif

    /* 状态位：L=正在自动标定前馈  E=编码器没信号已停车（空=正常） */
    if(control_enc_fault)
    {
        tft180_set_color(RGB565_RED, RGB565_BLACK);
        tft180_show_string(96, CAMERA_INFO_Y5, "E");
    }
    else if(0 == control_ff_ready)
    {
        tft180_set_color(RGB565_GREEN, RGB565_BLACK);
        tft180_show_string(96, CAMERA_INFO_Y5, "L");
    }
    else
    {
        tft180_show_string(96, CAMERA_INFO_Y5, " ");
        if(control_ff_fallback)                 /* F = 自动标定放弃，正在用硬编码前馈 600/10 */
        {
            tft180_set_color(RGB565_RED, RGB565_BLACK);
            tft180_show_string(104, CAMERA_INFO_Y5, "F");
        }
        if(control_loop_off)                    /* n = 编码器不可信，正在开环跑 */
        {
            tft180_set_color(RGB565_YELLOW, RGB565_BLACK);
            tft180_show_string(112, CAMERA_INFO_Y5, "n");
        }
    }
    tft180_set_color(RGB565_YELLOW, RGB565_BLACK);
    tft180_set_font(TFT180_8X16_FONT);
}