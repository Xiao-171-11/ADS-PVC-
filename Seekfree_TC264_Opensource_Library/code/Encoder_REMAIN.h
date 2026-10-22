/*
 * Encoder_REMAIN.h
 *
 *  编码器测速：每 5ms 控制周期读一次脉冲数，作为速度环的反馈量
 *
 *  Created on: 2026年10月21日
 *      Author: Freed
 */

#ifndef CODE_ENCODER_REMAIN_H_
#define CODE_ENCODER_REMAIN_H_
#include "zf_common_headfile.h"

/* ================================ 接线（按实际接线改这几行） ================================ */
/* 注意：P00_0~P00_7 被摄像头数据口占用，P02_0~P02_3 被摄像头 PCLK/VSYNC/IIC 占用，
 *       P02_4~P02_7 被左右电机的 PWM/方向占用，P15_x 被 TFT180 占用，
 *       这里选的是剩下的空闲脚，接线不一致时只改下面 4 行宏即可                        */
#define ENCODER_L_INDEX         TIM2_ENCODER                    // 左轮用 GPT1 的 T2
#define ENCODER_L_CH1           TIM2_ENCODER_CH1_P33_7         // 左轮 A 相（计数）
#define ENCODER_L_CH2           TIM2_ENCODER_CH2_P33_6         // 左轮 B 相（方向）

#define ENCODER_R_INDEX         TIM4_ENCODER                    // 右轮用 GPT1 的 T4
#define ENCODER_R_CH1           TIM4_ENCODER_CH1_P02_8         // 右轮 A 相（计数）
#define ENCODER_R_CH2           TIM4_ENCODER_CH2_P00_9         // 右轮 B 相（方向）

/* 方向反了就把对应的宏改成 1（现象：正转速度读出来是负数） */
#define ENCODER_L_REVERSE       (0)
#define ENCODER_R_REVERSE       (0)

/* 速度一阶低通系数：0-不用滤波  越小越平滑但越滞后 */
#define ENCODER_SPEED_FILTER    (0.70f)

/* ================================ 接口 ================================ */
void    Encoder_Init        (void);                             // 初始化两路编码器
int16   Encoder_Get_Count   (uint8 is_right);                   // 读并清零，返回本周期的脉冲数
float   Encoder_Get_Speed   (uint8 is_right);                   // 读并清零，返回滤波后的速度（脉冲/周期）

#endif /* CODE_ENCODER_REMAIN_H_ */
