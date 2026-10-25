/*
 * Pursuit_REMAIN.h
 *
 *  纯追踪（pure pursuit）：在逆透视俯视图上找前瞻点 -> 算曲率 -> 算前轮转角 -> 舵机偏移
 *
 *  和原来"中线加权偏差 -> 分段 PD 打角"的区别：
 *    原来：偏差是整幅图像素误差的加权平均，单位是"像素"，同一辆车同一赛道，
 *          近处 1 像素和远处 1 像素代表的横向距离差几十倍，增益永远调不到两头都好。
 *    现在：先把图像变成地面俯视图（每个格点 6.7 x 11.8 mm，恒定比例），
 *          在车前方 Ld 处取一个目标点，用几何公式直接算出"车要沿多大的圆弧过去"，
 *          再换算成前轮转角。曲率是**物理量**（1/mm），与安装高度、镜头、分辨率全部解耦。
 *
 *  几何公式（后轴中心为参考点，前视距离 Ld，目标点横向偏移 Xt）：
 *      kappa = 2 * Xt / (Xt^2 + Yt^2)          圆弧曲率，右转为正
 *      delta = atan(kappa * L)                 前轮转角，L 为轴距
 *      ratio = kappa * W / 2                   后轮差速比例，W 为后轮距
 *
 *  三个"防冲出赛道"的机制（针对用户反馈的路口/弯道冲出）：
 *    1. 十字路口：左右边界同时消失 -> 判定横臂开口 -> 保持进路口前的航向并逐拍衰减，
 *       同时按 open_rows / one_side_rows 区分"路口"和"弯道外侧丢线"，两者绝不混淆。
 *    2. 弯道：曲率直接参与减速，转角越大速度越低；并且单侧丢线时用学习到的半赛道宽
 *       推算另一侧，弯道里外道边界跑出画面也不会把中线拽到沟里。
 *    3. 冲出赛道：最近几行的中线偏离超过半赛道宽 -> off_track 置位，
 *       控制层会强制降速，并且直接朝赛道中心打角。
 *
 *  Created on: 2026年10月28日
 *      Author: Freed
 */

#ifndef CODE_PURSUIT_REMAIN_H_
#define CODE_PURSUIT_REMAIN_H_
#include "zf_common_headfile.h"
#include "IPM_REMAIN.h"

/* ============================ 边界提取 ============================ */
#define PURSUIT_SEARCH_COLS     (42)                    // 从预测中线向左右各搜多少列
#define PURSUIT_LINE_W_MAX      (8)                     // 一条边线最多几格宽（再宽是横臂，不是边线）
#define PURSUIT_TRACK_W_MIN     (22)                    // 有效赛道宽度下限（格点，约 148mm）
#define PURSUIT_TRACK_W_MAX     (92)                    // 有效赛道宽度上限（格点，约 620mm）
#define PURSUIT_WIDTH_TOL       (0.50f)                 // 实测宽度和学习的宽度差超过 50% 就当不可信
#define PURSUIT_SINGLE_TOL      (10.0f)                 // 单侧推算结果与预测中线差多少格以内才敢用

/* ============================ 纯追踪几何参数 ============================ */
#define PURSUIT_WHEELBASE_MM    (130.0f)                // 轴距（前后轮中心距），量一下填这里
#define PURSUIT_REAR_TRACK_MM   (120.0f)                // 后轮距（两个驱动轮中心距）
#define PURSUIT_DELTA_MAX_DEG   (32.0f)                 // 前轮最大转角（机械极限 30~35 度）

/* 前视距离随速度线性变化：慢的时候看近一点（不会切内道），快的时候看远一点（不会摆） */
#define PURSUIT_LD_MIN_MM       (250.0f)
#define PURSUIT_LD_MAX_MM       (800.0f)                // 不要超过 IPM_Y_FAR_MM，否则前瞻点在俯视图外
#define PURSUIT_SPEED_NORM_MIN  (0.30f)                 // 速度归一化下限（发车前也至少看 415mm）

/* ============================ 弯道减速 ============================ */
#define PURSUIT_CURV_SLOW_K     (0.0140f)                // 前轮每 1 度转角降多少速度
#define PURSUIT_CURV_SLOW_MIN   (0.4200f)               // 最慢降到 42%

/* ============================ 十字路口 ============================ */
/* 判据是"左右两边同时丢"：十字的横臂把两条边线一起切断；
 * 弯道只会让外侧那条边线跑出画面（单侧丢），两者用 open/one_side 的比例区分。 */
#define PURSUIT_CROSS_MIN_OPEN  (6)                     // 至少 6 行左右同时丢才算横臂开口
#define PURSUIT_CROSS_MIN_VALID (8)                     // 至少有 8 行看清楚才算可信（否则摄像头有问题）
#define PURSUIT_CROSS_TICKS     (20)                    // 路口最多保持 20 帧（50fps 下 0.4 s）
#define PURSUIT_CROSS_CLEAN     (4)                     // 连续 4 帧没有开口 -> 提前退出
#define PURSUIT_CROSS_REJECT    (20)                    // 出路口后 20 帧内不再判路口（防抖）
#define PURSUIT_CROSS_MAX_KAPPA (0.0025f)               // 进路口前曲率超过这个值 -> 是弯道不是路口

/* ============================ 冲出赛道 ============================ */
#define PURSUIT_OFFTRACK_MM     (190.0f)                // 最近 6 行中线偏离超过这个距离 = 车已经在赛道边缘
#define PURSUIT_RECOVER_GAIN    (0.2200f)               // 救车增益：每 mm 偏离给多少舵机偏移

/* ============================ 视野丢失保持 ============================ */
#define PURSUIT_HOLD_FRAMES     (5)                     // 看得清的行数不够时，沿用上一份快照最多 5 帧（100 ms）

/* ============================ 屏幕显示 ============================ */
/* TFT180 默认 TFT180_PORTAIT：宽 128、高 160，字体 TFT180_8X16（一个字符 8x16 像素）。
 * 屏幕上半部分 Y1~Y6 六行已经被相机/基点/基线/中线/控制占满（y = 2/18/34/50/68/84），
 * 所以纯追踪的信息画在第 7、8 行（y = 100 / 116），并且运行时再判一次屏幕够不够高。
 * 注意：zf 库里 show_float 的字符串长度是 num + pointnum + 2，show_int 是 num + 1，
 * 都是固定长度（不足补空格）。算 x 的时候必须按这个长度算，否则最后一格会画到 x = 128，
 * 触发 tft180.c:460 的 zf_assert(x < tft180_width_max)。 */
#define PURSUIT_DIS_EN          (1)                     // 1 = 在屏幕上显示纯追踪状态

/* ============================ 快照 ============================ */
typedef struct
{
    uint8  valid;                                       // 1 = 本次快照可用于控制
    uint8  cross;                                       // 1 = 正在过路口
    uint8  off_track;                                   // 1 = 车压到赛道边缘了
    uint8  valid_rows;                                  // 看得清的行数
    uint8  open_rows;                                   // 左右同时丢的行数
    uint8  one_side_rows;                               // 只有一侧丢的行数
    int16  target_row;                                  // 前瞻点所在行
    int16  target_col;                                  // 前瞻点所在列
    float  kappa;                                       // 曲率（1/mm），右转为正
    float  delta_deg;                                   // 前轮转角（度）
    float  steer_out;                                   // 给舵机的偏移（正=向左，与 Servo_Set_Turn 同号）
    float  slow;                                        // 速度系数（0~1）
    float  ratio;                                       // 后轮差速比例
    float  lookahead_mm;                                // 本次前视距离
    float  target_x_mm;                                 // 前瞻点横向（相对车轴中心，右为正）
    float  target_y_mm;                                 // 前瞻点纵向
    float  near_off_mm;                                 // 最近 6 行的中线横向偏移
} pursuit_struct;

extern pursuit_struct pursuit_info;                     // 主循环用（显示、调试）
extern int16 pursuit_center_row[];                       // 每行中线列号（显示用），长度 = IPM_H

void  Pursuit_Init(void);                               // 复位状态机
void  Pursuit_Process(void);                            // 每帧调用：提取边界 -> 十字判定 -> 纯追踪
uint8 Pursuit_Get(pursuit_struct *out);                 // 控制中断里取快照（双缓冲，不会读到半截数据）
void  Pursuit_Show_On_TFT180(void);                     // 屏幕显示 K/D/C/S

#endif /* CODE_PURSUIT_REMAIN_H_ */
