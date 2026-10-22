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
void Control_Init(void)
{
    pid_init(&pid_l, CONTROL_PID_KP, CONTROL_PID_KI, CONTROL_PID_KD, CONTROL_PID_OUT_MAX);
    pid_init(&pid_r, CONTROL_PID_KP, CONTROL_PID_KI, CONTROL_PID_KD, CONTROL_PID_OUT_MAX);

    control_last_error = 0.0f;
    control_lost_tick  = 0;
    control_tick_cnt   = 0;
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

    control_tick_cnt ++;

    // 1. 发车延时：上电后先别动，方便架起来调试
    if((control_tick_cnt * CONTROL_PERIOD_MS) < CONTROL_START_DELAY_MS)
    {
        Motor_Stop();
        Servo_Set_Duty(Servo_MID);
        return;
    }

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

    // 3. 舵机：偏差 -> 打角
    Servo_Set_Turn(error);

    // 4. 弯道减速 + 后轮差速
    //    弯道减速：偏差越大 -> 弯越急 -> 基础速度先降下来，避免高速冲弯打不动方向
    abs_err = (error > 0.0f) ? (error) : (-error);
    slow    = 1.0f - (CONTROL_CURVE_SLOW_K * abs_err);
    if(slow < CONTROL_CURVE_SLOW_MIN) slow = CONTROL_CURVE_SLOW_MIN;
    base_speed = CONTROL_BASE_SPEED * slow;

    //    后轮差速：偏差为正（赛道偏右）-> 车的右转弯 -> 后左轮(外侧)快、后右轮(内侧)慢
    //    用“速度比例”而不是固定差值：ratio 乘以基础速度，提速时差速自动等比放大
    ratio = CONTROL_DIFF_K * error;
    if(ratio >  CONTROL_DIFF_K_MAX) ratio =  CONTROL_DIFF_K_MAX;
    if(ratio < -CONTROL_DIFF_K_MAX) ratio = -CONTROL_DIFF_K_MAX;
#if (1 == CONTROL_DIFF_REVERSE)
    ratio = -ratio;                                     // 差速方向反了的时候用
#endif

    target_l = base_speed * (1.0f + ratio);            // 后左轮
    target_r = base_speed * (1.0f - ratio);            // 后右轮

    if(target_l > CONTROL_SPEED_MAX) target_l = CONTROL_SPEED_MAX;
    if(target_r > CONTROL_SPEED_MAX) target_r = CONTROL_SPEED_MAX;
    if(target_l < CONTROL_SPEED_MIN) target_l = CONTROL_SPEED_MIN;   // 内侧轮不要停转，否则等于拖刹
    if(target_r < CONTROL_SPEED_MIN) target_r = CONTROL_SPEED_MIN;

    // 5. 速度环：前馈 + PID 修正
    speed_l = Encoder_Get_Speed(0);
    speed_r = Encoder_Get_Speed(1);
    control_speed_l = speed_l;                          // 给屏幕显示用
    control_speed_r = speed_r;

    if(CONTROL_BASE_SPEED > 1.0f)                       // 前馈：占空比按目标速度等比缩放
    {
        duty_l = (float)CONTROL_DUTY_BASE * (target_l / CONTROL_BASE_SPEED);
        duty_r = (float)CONTROL_DUTY_BASE * (target_r / CONTROL_BASE_SPEED);
    }
    else                                                // 基础速度没标定好时不开前馈，避免除零
    {
        duty_l = 0.0f;
        duty_r = 0.0f;
    }

#if (1 == CONTROL_SPEED_LOOP_EN)
    duty_l += pid_calc(&pid_l, target_l, speed_l);      // PID 只做修正，主出力靠前馈
    duty_r += pid_calc(&pid_r, target_r, speed_r);
#endif

    if(duty_l < 0.0f) duty_l = 0.0f;
    if(duty_r < 0.0f) duty_r = 0.0f;

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
    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(RGB565_YELLOW, RGB565_BLACK);
    tft180_show_string(0, 0, "V");
    tft180_show_int(8,  0, (int32)control_speed_l, 3);
    tft180_show_string(32, 0, "R");
    tft180_show_int(40, 0, (int32)control_speed_r, 3);
    tft180_set_font(TFT180_8X16_FONT);
}