/*
 * Servo_REMAIN.c
 *
 *  舵机控制：分段增益 + 微分阻尼 + 死区
 *
 *  打角曲线（out 是相对中值的占空比偏移，正数向左、负数向右）：
 *      |error| <= SERVO_DEAD_ZONE :  out = 0
 *      |error| <= SERVO_KNEE      :  out = ± KP_NEAR * |error|
 *      |error| >  SERVO_KNEE      :  out = ± (KP_NEAR*KNEE + KP_FAR*(|error|-KNEE))
 *      最后叠加微分项 KD*(error - last_error) 做阻尼
 *
 *  段与段之间在拐点处连续，不会跳变。
 *
 *  Created on: 2026年10月19日
 *      Author: Freed
 */
#include "Servo_REMAIN.h"

static float servo_last_error = 0.0f;                   // 上一次的偏差（微分项用）

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机初始化
// 参数说明     void
// 返回参数     void
// 使用示例     Servo_Init();
// 备注信息     50Hz 舵机，上电先回中值，避免上电瞬间乱打角
//-------------------------------------------------------------------------------------------------------------------
void Servo_Init(void)
{
    pwm_init(SERVO_PWM_PIN, SERVO_PWM_FREQ, Servo_MID + SERVO_TRIM);
    servo_last_error = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     直接输出舵机占空比（带限幅）
// 参数说明     duty            占空比，超出 Servo_MID-SERVO_DUTY_RANGE_R ~ Servo_MID+SERVO_DUTY_RANGE_L 会被夹住
// 返回参数     void
// 使用示例     Servo_Set_Duty(Servo_MID);
// 备注信息     限幅是为了保护舵机，不要让它长期顶到机械死点
//-------------------------------------------------------------------------------------------------------------------
void Servo_Set_Duty(int32 duty)
{
    /* 左右不对称硬限幅：不管指令从哪来（打角、标定、以后新加的函数），都在这里被卡死，
     * 保证占空比永远落在 395 ~ 534 之间，不会把舵机推到机械死点上堵转。 */
    if(duty > (int32)(Servo_MID + SERVO_TRIM + SERVO_DUTY_RANGE_L))
    {
        duty = (int32)(Servo_MID + SERVO_TRIM + SERVO_DUTY_RANGE_L);
    }
    if(duty < (int32)(Servo_MID + SERVO_TRIM - SERVO_DUTY_RANGE_R))
    {
        duty = (int32)(Servo_MID + SERVO_TRIM - SERVO_DUTY_RANGE_R);
    }
    pwm_set_duty(SERVO_PWM_PIN, (uint32)duty);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按中线偏差打角（分段增益 + 微分 + 死区）
// 参数说明     error           中线偏差（像素），正数=赛道偏右
// 返回参数     void
// 使用示例     Servo_Set_Turn(midline_info.error);
// 备注信息     1. 直道小偏差只给小打角，弯道大偏差给大打角，避免“直道摆、弯道打不动”
//              2. KD 用相邻两帧偏差的差做阻尼，能明显压住来回摆
//-------------------------------------------------------------------------------------------------------------------
void Servo_Set_Turn(float error)
{
    float abs_error = 0.0f;
    float out       = 0.0f;

#if (1 == SERVO_REVERSE)
    error = -error;
#endif

    abs_error = (error > 0.0f) ? (error) : (-error);

    if(abs_error <= SERVO_DEAD_ZONE)                    // 死区：直道上的小抖动不动作
    {
        out = 0.0f;
    }
    else if(abs_error <= SERVO_KNEE)                    // 小偏差：柔和
    {
        out = SERVO_KP_NEAR * error;
    }
    else                                                // 大偏差：有力
    {
        float sign = (error > 0.0f) ? (1.0f) : (-1.0f);
        out = sign * ((SERVO_KP_NEAR * SERVO_KNEE) + (SERVO_KP_FAR * (abs_error - SERVO_KNEE)));
    }

    out += SERVO_KD * (error - servo_last_error);       // 微分阻尼，抑制过冲和来回摆
    servo_last_error = error;

    Servo_Set_Duty((int32)((float)(Servo_MID + SERVO_TRIM) - out));   // 赛道偏右 -> 占空比变小 -> 向右转
}
