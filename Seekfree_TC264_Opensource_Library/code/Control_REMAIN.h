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
#define CONTROL_START_DELAY_MS      (3000)                  // 上电延时 3s 再发车（0=不延时）

/* ================================ 速度环 ================================ */
#define CONTROL_BASE_SPEED          (10.0f)                 // 基础速度（编码器脉冲/周期）：20 -> 10，整体降速 50%
#define CONTROL_SPEED_MAX           (18.0f)                 // 差速后的速度上限：35 -> 18
#define CONTROL_SPEED_MIN           (3.0f)                  // 内侧轮最低速度：5 -> 3
#define CONTROL_DUTY_BASE           (600)                   // 达到基础速度大致的占空比（前馈量）：1200 -> 600
#define CONTROL_PID_KP              (20.0f)                 // 速度环 Kp
#define CONTROL_PID_KI              (0.5f)                  // 速度环 Ki
#define CONTROL_PID_KD              (0.0f)                  // 速度环 Kd
#define CONTROL_PID_OUT_MAX         (500.0f)                // 速度环输出限幅：1000 -> 500（与目标同步减半）

/* ============================ 速度闭环开关 ============================ */
/* 1-正常闭环；0-只用前馈开环跑（编码器还没接好、或想先确认车能走直线时用） */
#define CONTROL_SPEED_LOOP_EN       (1)

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
#define CONTROL_CURVE_SLOW_K        (0.0040f)               // 偏差 1 像素降低多少比例的速度
#define CONTROL_CURVE_SLOW_MIN      (0.6000f)               // 最多降到基础速度的 60%

/* ================================ 保护 ================================ */
#define CONTROL_LOST_STOP_TICKS     (0)                     // 连续丢线后停车（0=不停车，弯道短暂丢线不会卡死）

/* ================================ 接口 ================================ */
void Control_Init (void);                                   // 复位 PID 与状态
void Control_Tick (void);                                   // 5ms 中断里调用：一次完整控制
void Control_Show_On_TFT180 (void);                         // 图像左上角显示实测轮速

#endif /* CODE_CONTROL_REMAIN_H_ */
