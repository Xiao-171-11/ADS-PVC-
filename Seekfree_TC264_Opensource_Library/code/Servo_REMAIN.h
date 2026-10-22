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

#define Servo_MID               (464)                   // 舵机中值（实测螺杆有公差，465 整体偏右，定为 464）；偏大向左转，偏小向右转
#define SERVO_TRIM              (0)                     // 中值微调：直道上车往哪边偏，就往反方向调几个数
#define SERVO_PWM_PIN           ATOM0_CH1_P33_9         // 舵机 PWM 引脚
#define SERVO_PWM_FREQ          (50)                    // 舵机固定 50Hz

/* ======================== 分段打角参数 ======================== */
/* ---- 机械行程（实测：占空比只有 395 ~ 534，超出就顶死点堵转、发烫）----
 * 中值 464：向左最多 +70（=534），向右最多 -69（=395）。左右不对称，所以拆成两个宏。
 * 原来是统一的 180（464±180 = 284~644），指令能推到机械死点外面去，这是"舵机卡死"的根源。 */
#define SERVO_DUTY_RANGE_L      (70)                    // 向左最大偏移：464+70 = 534
#define SERVO_DUTY_RANGE_R      (69)                    // 向右最大偏移：464-69 = 395
#define SERVO_DUTY_RANGE        (SERVO_DUTY_RANGE_L)    // 兼容旧名字（取较大一侧）
#define SERVO_DEAD_ZONE         (2.0f)                  // 死区：偏差小于 2 像素不动，专门治直道抖动
#define SERVO_KNEE              (15.0f)                 // 拐点：偏差 15 像素以内用小增益，以外用大增益
#define SERVO_KP_NEAR           (1.0f)                  // 小偏差增益（直道柔和，原来统一 3.0 太猛）
#define SERVO_KP_FAR            (2.5f)                  // 大偏差增益（弯道有力）
#define SERVO_KD                (2.0f)                  // 微分阻尼：抑制来回摆，0 = 关掉
#define SERVO_REVERSE           (0)                     // 1-左右打角反了就把这里改成 1

void Servo_Init     (void);                             // 初始化舵机 PWM
void Servo_Set_Duty (int32 duty);                       // 直接输出占空比（内部限幅）
void Servo_Set_Turn (float error);                      // 按中线偏差打角

#endif /* CODE_SERVO_REMAIN_H_ */
