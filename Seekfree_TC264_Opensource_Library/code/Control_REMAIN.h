/*
 * Control_REMAIN.h
 *
 *  控制总成：5ms 控制节拍里完成
 *      编码器测速 -> 速度环 PID -> 差速 -> 左右电机 PWM
 *      中线偏差  -> 舵机 PD   -> 舵机 PWM
 *  注：左右电机是**后轮驱动轮**，差速只作用在后左/后右之间（前轮由舵机转向）。
 *
 *  Created on: 2026年10月21日
 *      Author: Freed
 */

#ifndef CODE_CONTROL_REMAIN_H_
#define CODE_CONTROL_REMAIN_H_
#include "zf_common_headfile.h"

/* ================================ 时序 ================================ */
#define CONTROL_PERIOD_MS           (5)                     // 控制周期 5ms（PIT 中断周期）
#define CONTROL_START_DELAY_MS       1000                                              // 上电延时 3s 再发车（0=不延时）

/* ================================ 速度环 ================================ */
#define CONTROL_BASE_SPEED           75.0f                                                                                                                                                       // 基础速度（编码器脉冲/周期）：20 -> 10，整体降速 50%
#define CONTROL_SPEED_MAX            112.5f                                                                                                                                                    // 差速后的速度上限：35 -> 18
#define CONTROL_SPEED_MIN            22.5f                                                                                                                                                         // 内侧轮最低速度：5 -> 3
#define CONTROL_DUTY_BASE           (600)                   // 达到基础速度大致的占空比（前馈量）：1200 -> 600
#define CONTROL_PID_KP              (20.0f)                 // 速度环 Kp
#define CONTROL_PID_KI              (0.5f)                  // 速度环 Ki
#define CONTROL_PID_KD              (0.0f)                  // 速度环 Kd
#define CONTROL_PID_OUT_MAX         (500.0f)                // 速度环输出限幅：1000 -> 500（与目标同步减半）

/* ============================ 速度闭环开关 ============================ */
/* 1-正常闭环；0-只用前馈开环跑（编码器还没接好、或想先确认车能走直线时用） */
#define CONTROL_SPEED_LOOP_EN       (1)

/* ============================ 前馈自动标定（强烈建议保持 1） ============================
 * 换了编码器以后，"多少占空比跑出多少脉冲/周期"是未知的，直接硬填 CONTROL_BASE_SPEED
 * 很容易差好几倍（车要么爬要么窜）。所以上电第一次发车时先开环跑 CONTROL_FF_LEARN_TICKS 拍，
 * 量出实际速度，自动算出前馈系数 = 占空比 / 速度；标定完再进闭环。
 * 标完之后 CONTROL_BASE_SPEED 就可以老老实实当"脉冲/周期"的目标速度用。*/
#define CONTROL_ENCODER_TEST_EN   0            /* 1 = 编码器静态自检：电机不转，屏幕 V/R 显示每 5ms 的原始脉冲 */
#define CONTROL_SPEED_NOISE_GATE  1            /* 1 = 运行中扣掉静止时测到的噪声底 */
#define CONTROL_NOISE_MAX_OK      20.0f        /* 噪声底超过这个数（脉冲/5ms）说明编码器线受干扰，闭环不可信 */

/* 上次实测结果（2026-10-22，架空自标定）：上电直接用，不再每次标定 */
#define CONTROL_FF_GAIN_L_DEFAULT   7.0f
#define CONTROL_FF_GAIN_R_DEFAULT   8.0f
#define CONTROL_FF_OFFSET_L_DEFAULT 529.0f
#define CONTROL_FF_OFFSET_R_DEFAULT 484.0f

#define CONTROL_FF_LEARN_EN          0                                                    // 1-上电自动标定前馈  0-直接用手填的 DUTY_BASE/BASE_SPEED
#define CONTROL_FF_LEARN_TICKS      (140)                   // 每个测量点 140 拍 = 0.7 s（前 30 拍丢瞬态，后 110 拍取平均）
#define CONTROL_FF_LEARN_SKIP       (30)                    // 前 30 拍(0.15s)是电机启动瞬态，不参与平均
#define CONTROL_FF_LEARN_DUTY_MIN   (800)                   // 标定起始占空比：6% 拖不动带减速箱的小车，从 8% 起
#define CONTROL_FF_LEARN_DUTY_MAX   (4000)                  // 标定占空比上限，爬到还不转就判编码器故障
#define CONTROL_FF_LEARN_STEP       (300)                   // 每 CONTROL_FF_LEARN_RAMP_TICKS 拍加多少占空比
#define CONTROL_FF_LEARN_RAMP_TICKS (15)                    // 每次加占空比的间隔（拍）
#define CONTROL_FF_LEARN_BOOST       (800)                   // 第二测量点比启动点再高多少（离开死区，斜率才准）
#define CONTROL_FF_MIN_SPEED        (1.5f)                  // 认"轮子真的转起来了"的速度门限（脉冲/周期）
#define CONTROL_FF_GAIN_MIN         (2.0f)                  // 前馈系数下限（占空比 / 速度）
#define CONTROL_FF_GAIN_MAX         (400.0f)                // 前馈系数上限
#define CONTROL_FF_OFFSET_MAX        (1200.0f)               // 死区补偿上限：实测死区 800~1000，给到 1200 就够，再大等于一直踩着油门
#define CONTROL_FF_DUTY_AT_BASE_MAX  (4000.0f)               // 标定完用手算：base 速度对应的占空比超过它就不可信
#define CONTROL_OVERSPEED_EN         (1)                     // 1-超速看门狗（实测速度远超目标就停车重新标定）
#define CONTROL_OVERSPEED_SPEED      180.0f                                                                                                                                                    // 超速门限（脉冲/周期），= 目标 10 的 3 倍
#define CONTROL_OVERSPEED_TICKS      (100)                   // 连续 100 拍（0.5 s）超速就动作

/* ============================ 编码器掉线保护 ============================ */
/* 给了占空比、轮子却一直读不到速度 -> 编码器没插好/定时器选错/供电没接。
 * 这时如果继续闭环保留积分，车会一脚油门冲出去，所以直接停车并在屏幕上显示 E。*/
#define CONTROL_ENC_FAULT_EN        (1)                     // 1-开保护  0-不检查（先用 0 排查接线）
#define CONTROL_ENC_FAULT_DUTY      (1200.0f)               // 平均占空比超过这个值（15%，正常慢速跑不会触发）
#define CONTROL_ENC_FAULT_TICKS     (200)                   // 且连续 200 拍（1 s）速度接近 0，就判故障
#define CONTROL_ENC_CUT_TICKS       (100)                   // 判故障后停车 0.5 s 再自动重试（不锁死）
#define CONTROL_ENC_RETRY_MAX       (5)                     // 连续失败几次后放弃重试（E 常亮当告警，但不再停车）

/* ============================ 后轮差速（驱动轮） ============================ */
/* 车是"舵机转前轮 + 双电机驱动后轮"，差速作用在后左/后右两个驱动轮上。
 * 内外轮速度比由转弯半径决定： v_外 / v_内 = (R + W/2) / (R - W/2)，W 为后轮距
 * 小角度下近似成线性的“速度比例差”，比固定的绝对差值更合理：
 *      ratio = K * error
 *      v_后左 = 基础速度 * (1 + ratio)
 *      v_后右 = 基础速度 * (1 - ratio)
 * 用比例而不是绝对差值的好处：基础速度一提，差速量自动跟着变大，不用重调参数。 */
#define CONTROL_DIFF_K              (0.0040f)               // 偏差 1 像素对应的速度比例（0.4%）
#define CONTROL_DIFF_K_MAX          (0.5000f)               // 最大差速比例（内侧轮最多降 50%）
#define CONTROL_DIFF_REVERSE        (0)                     // 1-差速方向反了（车往弯外甩）时改 1

/* ============================ 弯道减速 ============================ */
/* 偏差越大说明弯越急，先把基础速度降下来，避免高速冲进弯道打不动方向 */
#define CONTROL_CURVE_SLOW_K        (0.0080f)
#define CONTROL_CURVE_SLOW_MIN      (0.4500f)

/* ============================ 纯追踪接管（逆透视 + pure pursuit） ============================
 * 1-用 Pursuit_REMAIN 算出来的曲率打角和减速（推荐）
 * 0-退回老算法（中线加权偏差 -> 分段 PD），出问题时把这里改成 0 就能立刻对比 */
#define CONTROL_PURSUIT_EN          (1)
#define CONTROL_CROSS_SPEED_SCALE   (0.80f)                 // 过路口整体降速到 80%（横臂补线不可信，慢一点更稳）
#define CONTROL_OFFTRACK_SPEED_SCALE (0.45f)                // 车压到赛道边缘时降到 45%，先保命再纠偏

/* ============================ 分档打角（实车要求） ============================
 * 1-舵机只输出 5 个固定档位：460（中位/路口）、440、474（直道调姿）、418、500（弯道转向）
 *   档位由 Servo_Set_Step() 内部判定（带滞环 + 停留时间），路口强制 460、绝不转弯。
 * 0-退回连续打角（纯追踪 steer_out / 偏差分段 PD），出问题时改这里立刻对比 */
#define CONTROL_STEP_STEER_EN       (1)
#define CONTROL_ERR_PX_TO_DEG       (0.60f)                 // 偏差(像素) -> 打角需求(度)，没有纯追踪时用

/* ================================ 保护 ================================ */
#define CONTROL_LOST_STOP_TICKS     (0)                     // 连续丢线后停车（0=不停车，弯道短暂丢线不会卡死）

/* 纯追踪算前瞻距离时要知道"现在跑多快"。这个变量每拍由 Control_Tick 写入，
 * 是**规划值**（含弯道减速后），不是实测值——用实测值会形成正反馈：
 * 减速 -> 前瞻变近 -> 打角变大 -> 更减速。 */
extern volatile float control_base_speed_ref;

/* ================================ 接口 ================================ */
void Control_Init (void);                                   // 复位 PID 与状态
void Control_Tick (void);                                   // 5ms 中断里调用：一次完整控制
void Control_Show_On_TFT180 (void);                         // 图像左上角显示实测轮速

#endif /* CODE_CONTROL_REMAIN_H_ */
