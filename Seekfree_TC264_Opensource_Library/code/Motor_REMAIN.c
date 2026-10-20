/*
 * Drive_EV.c
 *
 *  Created on: 2026Äê10ÔÂ19ÈÕ
 *      Author: Freed
 */
#include<Motor_REMAIN.h>
void Motor_R_init()
{
    gpio_init(Motor_R_turn,GPO,0,GPO_PUSH_PULL);
    pwm_init(Motor_R_pwm,5000,0);
}
void Motor_L_init()
{
    gpio_init(Motor_L_turn,GPO,0,GPO_PUSH_PULL);
    pwm_init(Motor_L_pwm,5000,0);
}
void Motor_R_SetRemain(int pwm_R)
{
    if(pwm_R>=0)
    {
        gpio_set_level(Motor_R_turn,0);
        pwm_set_duty(Motor_R_pwm,pwm_R);
    }
    else
    {
        gpio_set_level(Motor_R_turn,1);
        pwm_set_duty(Motor_R_pwm,-pwm_R);
    }
}
void Motor_L_SetRemain(int pwm_L)
{
    if(pwm_L>=0)
        {
            gpio_set_level(Motor_L_turn,0);
            pwm_set_duty(Motor_L_pwm,pwm_L);
        }
        else
        {
            gpio_set_level(Motor_L_turn,1);
            pwm_set_duty(Motor_L_pwm,-pwm_L);
        }
}
void HDMI()
{

}



