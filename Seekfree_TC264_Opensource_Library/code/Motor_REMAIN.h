/*
 * Motor_REMAIN.h
 *
 *  Created on: 2026Äê10ÔÂ19ÈÕ
 *      Author: Freed
 */

#ifndef CODE_MOTOR_REMAIN_H_
#define CODE_MOTOR_REMAIN_H_
#include "zf_common_headfile.h"
#define Motor_R_pwm ATOM0_CH6_P02_6
#define Motor_R_turn P02_7
#define Motor_L_pwm ATOM0_CH4_P02_4
#define Motor_L_turn P02_5
void Motor_R_init();
void Motor_L_init();
void Motor_R_SetRemain(int pwm_R);
void Motor_L_SetRemain(int pwm_L);
void HDMI();


#endif /* CODE_MOTOR_REMAIN_H_ */
