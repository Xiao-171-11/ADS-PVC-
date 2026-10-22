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
static float  servo_hold_out = 0.0f;
static uint16 servo_hold_cnt = 0;
static uint16 servo_big_cnt  = 0;
static uint8  servo_armed    = 0;
static float  servo_peak_abs = 0.0f;
static float  servo_peak_out = 0.0f;
static float  servo_err_f    = 0.0f;                   // 低通后的偏差
static uint8  servo_deep     = 0;                      // 本弯峰值够大（真进过弯）

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
void Servo_Clear_Hold(void)
{
    servo_hold_cnt = 0;
    servo_armed    = 0;
    servo_big_cnt  = 0;
    servo_deep     = 0;
}

void Servo_Set_Turn(float error)
{
    float abs_error = 0.0f;
    float p_out     = 0.0f;
    float out       = 0.0f;

#if (1 == SERVO_REVERSE)
    error = -error;
#endif

    /* 偏差低通：直道上边线逐帧抖动 1~3 像素，不过滤就会被高增益放大成来回打角 */
    error = (SERVO_ERR_FILTER * error) + ((1.0f - SERVO_ERR_FILTER) * servo_err_f);
    servo_err_f = error;

    abs_error = (error > 0.0f) ? (error) : (-error);

    if(abs_error <= SERVO_DEAD_ZONE)
    {
        p_out = 0.0f;
    }
    else if(abs_error <= SERVO_KNEE)
    {
        p_out = SERVO_KP_NEAR * error;
    }
    else
    {
        float sign = (error > 0.0f) ? (1.0f) : (-1.0f);
        p_out = sign * ((SERVO_KP_NEAR * SERVO_KNEE) + (SERVO_KP_FAR * (abs_error - SERVO_KNEE)));
    }

    out = p_out + (SERVO_KD * (error - servo_last_error));
    servo_last_error = error;

#if (1 == SERVO_EXIT_HOLD_EN)
    if(abs_error >= SERVO_ARM_ERROR)
    {
        if(0 == servo_big_cnt)                          /* new run of large error: restart peak */
        {
            servo_peak_abs = abs_error;
            servo_peak_out = p_out;
        }
        if(servo_big_cnt < 60000)
        {
            servo_big_cnt ++;
        }
        if(servo_big_cnt >= SERVO_ARM_MIN_TICKS)
        {
            servo_armed = 1;
        }
        if(abs_error > servo_peak_abs)
        {
            servo_peak_abs = abs_error;
            servo_peak_out = p_out;
        }
        if(servo_peak_abs >= SERVO_HOLD_MIN_PEAK)
        {
            servo_deep = 1;
        }
        servo_hold_cnt = 0;                             /* back into a corner: drop the hold */
    }
    else
    {
        servo_big_cnt = 0;
        if(1 == servo_armed)
        {
            if(abs_error > SERVO_RELEASE_ERROR)         /* still in the corner: track the peak */
            {
                if(abs_error > servo_peak_abs)
                {
                    servo_peak_abs = abs_error;
                    servo_peak_out = p_out;
                }
            }
            else if((0 == servo_hold_cnt) && (1 == servo_deep))   /* just left a real corner: latch the peak */
            {
                servo_hold_out = servo_peak_out;
                if(servo_hold_out >  (float)SERVO_DUTY_RANGE_L)
                {
                    servo_hold_out =  (float)SERVO_DUTY_RANGE_L;
                }
                if(servo_hold_out < -(float)SERVO_DUTY_RANGE_R)
                {
                    servo_hold_out = -(float)SERVO_DUTY_RANGE_R;
                }
                servo_hold_cnt = SERVO_HOLD_TICKS;
                servo_peak_abs = 0.0f;
                servo_deep     = 0;
            }
        }
    }

    if(servo_hold_cnt > 0)
    {
        /* 误差已经反过来了：车被往另一边推，继续锁着旧打角只会推它出去，立即释放 */
        if(((error * servo_hold_out) < 0.0f) && (abs_error > SERVO_RELEASE_ERROR))
        {
            servo_hold_cnt = 0;
            servo_armed    = 0;
            servo_big_cnt  = 0;
            servo_deep     = 0;
        }
    }

    if(servo_hold_cnt > 0)
    {
        out = servo_hold_out;
#if (1 == SERVO_HOLD_DECAY)
        out = servo_hold_out * ((float)servo_hold_cnt / (float)SERVO_HOLD_TICKS);
#endif
        servo_hold_cnt --;
        if(0 == servo_hold_cnt)
        {
            servo_armed = 0;
        }
    }
#endif

    Servo_Set_Duty((int32)((float)(Servo_MID + SERVO_TRIM) - out));
}

