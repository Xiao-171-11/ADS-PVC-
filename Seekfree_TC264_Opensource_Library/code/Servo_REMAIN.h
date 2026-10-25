/*
 * Servo_REMAIN.h
 *
 *  舵机：偏差 -> 打角（分段增益 + 微分阻尼 + 死区）
 *
 *  为什么分段：
 *    直道上偏差只有几像素，如果增益统一给很大，车就会在直道上来回摆（转向过度）；
 *    弯道里偏差几十像素，又需要足够的打角量，不然打不动方向、直接冲出赛道。
 *    所以小偏差用小增益（柔），大偏差用大增益（有力），中间一个拐点 SERO_KNEE 衔接。
 *
 *  Created on: 2026年10月19日
 *      Author: Freed
 */

#ifndef CODE_SERVO_REMAIN_H_
#define CODE_SERVO_REMAIN_H_
#include "zf_common_headfile.h"

#define Servo_MID               (464)                   // 舵机中值（50Hz，实测）
#define SERVO_TRIM              (0)                     // 中值微调：直道上车往哪边偏，就往反方向调几个数
#define SERVO_PWM_PIN           ATOM0_CH1_P33_9         // 舵机 PWM 引脚
#define SERVO_PWM_FREQ          (50)                    // 舵机固定 50Hz

/* ======================== 机械行程（实测，不许越过） ========================
 * 占空比可用区间 395 ~ 534，中值 464：
 *   向下最多 464 - 395 = 69 个码，向上最多 534 - 464 = 70 个码。
 * 所有限幅都从这两个数推出来，任何通路（偏差 PD / 纯追踪 / 出弯保持）都不会越界；
 * 一旦顶到死点就是堵转发烫，所以边界只写在这里一次。 */
#define SERVO_DUTY_MIN          (395)                   // 实测机械下限
#define SERVO_DUTY_MAX          (534)                   // 实测机械上限

/* ======================== 打角极性（单开关） ========================
 * 约定：out（偏移量）> 0 表示"逻辑向左打"，由下面这个符号翻译成占空比。
 *   SERVO_DIR = -1 ：out 正 -> 占空比减小（占空比越小越向左）——仓库原有方向
 *   SERVO_DIR = +1 ：out 正 -> 占空比增大（占空比越小越向左）——若实测是"号小往左"就改这里
 * 偏差 PD 与纯追踪两条通路共用这一个符号，改一次两边一起反向，不会互相打架。
 * 标定方法：上电后把 Servo_Set_Duty(400) / Servo_Set_Duty(530) 各打一次，
 *           看前轮往哪边偏，再决定这个符号（不要凭记忆改）。 */
#define SERVO_DIR               (-1)

#if (SERVO_DIR < 0)
#define SERVO_DUTY_RANGE_L      ((Servo_MID + SERVO_TRIM) - SERVO_DUTY_MIN)   // out 正（左）可用 69 个码
#define SERVO_DUTY_RANGE_R      (SERVO_DUTY_MAX - (Servo_MID + SERVO_TRIM))   // out 负（右）可用 70 个码
#else
#define SERVO_DUTY_RANGE_L      (SERVO_DUTY_MAX - (Servo_MID + SERVO_TRIM))   // 70 个码
#define SERVO_DUTY_RANGE_R      ((Servo_MID + SERVO_TRIM) - SERVO_DUTY_MIN)   // 69 个码
#endif
#define SERVO_DUTY_RANGE        (SERVO_DUTY_RANGE_L)    // 兼容旧名字（取较大一侧）
#define SERVO_DUTY_FROM_OUT(off)    ((int32)((float)(Servo_MID + SERVO_TRIM) + ((float)(SERVO_DIR) * (float)(off))))
#define SERVO_DEAD_ZONE         (3.0f)
#define SERVO_KNEE              (8.0f)
#define SERVO_KP_NEAR           (1.2f)
#define SERVO_KP_FAR            (4.0f)
#define SERVO_ERR_FILTER        (0.65f)                 // 偏差低通：0.65*本次 + 0.35*上次，压掉直道上的逐帧抖动
#define SERVO_HOLD_MIN_PEAK     (30.0f)                 // 出弯保持的最小峰值：没真进过弯就不许锁
#define SERVO_KD                (2.0f)                  // 微分阻尼：抑制来回摆，0 = 关掉
/* ---- exit hold ---------------- */
#define SERVO_EXIT_HOLD_EN      (1)                     // 1-open exit hold
#define SERVO_HOLD_TICKS        (40)                    // hold time: 40 x 5ms = 0.2 s
#define SERVO_HOLD_DECAY        (1)                     // 1-keep then return to center gradually over the hold window / 0-hold constant
#define SERVO_ARM_ERROR         (30.0f)
#define SERVO_ARM_MIN_TICKS     (10)                    // need 10 ticks(50ms) to arm
#define SERVO_RELEASE_ERROR     (15.0f)                 // |error| below this = just left corner

/* 偏差通路的 error 取反开关。
 * 注意：打角极性已经统一到上面的 SERVO_DIR 一处（两条通路共用），
 * 这里再开 1 只会把偏差通路单独翻过去，纯追踪那条不会跟着翻，两条通路就打架了。
 * 除非你确定只想翻偏差通路，否则保持 0。 */
#define SERVO_REVERSE           (0)

/* ---- 连续打角（纯追踪用）----
 * 纯追踪每帧算出来的是"应该打多少度"，是连续量，不需要分段增益，但必须限制变化率：
 * 舵机是机械件，一帧之内从 +69 跳到 -70 会听到"咔"一声并且大电流，长期这么干会烧齿轮。 */
#define SERVO_SLEW_MAX          (14.0f)                 // 每个 5ms 控制周期最多变化多少占空比（69/14 ≈ 5 拍打满）

void Servo_Init     (void);                             // 初始化舵机 PWM
void Servo_Set_Duty (int32 duty);                       // 直接输出占空比（内部限幅）
void Servo_Set_Turn (float error);                      // 按中线偏差打角
void Servo_Set_Offset (float offset);                   // 直接按占空比偏移打角（正=向左），带斜率限制
void Servo_Clear_Hold (void);                           // 立刻取消出弯保持（路口用）

#endif /* CODE_SERVO_REMAIN_H_ */
