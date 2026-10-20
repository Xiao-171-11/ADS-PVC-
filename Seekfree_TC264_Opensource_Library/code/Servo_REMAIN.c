/*
 * Servo_REMAIN.c
 *
 *  Created on: 2026Äê10ÔÂ19ÈÕ
 *      Author: Freed
 */
#include<Servo_REMAIN.h>
void Servo_Init()
{
    pwm_init(ATOM0_CH1_P33_9,50,Servo_MID);
}
