/*
 * MIDLine_REMAIN.h
 *
 *  加权求中线值（含互补滤波）
 *
 *  思路：
 *    1) 每一行由左右两条边缘基线各得到一个边界列，两者取中点 = 该行的中线点，存进 mid_line_list
 *    2) 每一行有一个权重 mid_weight_list[row]（可自行修改），按权重加权平均得到本次中线 mid_line
 *    3) 与上一帧结果做互补滤波（一阶低通）：mid_line_value = last * 0.2 + new * 0.8
 *       输出 mid_line_value 与偏差 error，可直接送舵机 PID
 *
 *  Created on: 2026年10月21日
 *      Author: Freed
 */

#ifndef CODE_MIDLINE_REMAIN_H_
#define CODE_MIDLINE_REMAIN_H_
#include "zf_common_headfile.h"
#include "Bedline_REMAIN.h"                             // 需要左右边缘基线

/* ================================ 可调参数 ================================ */
#define MIDLINE_DEFAULT_CENTER  (MT9V03X_W / 2)         // 中线默认值（图像中心列）
#define MIDLINE_INVALID_COL     (0xFF)                  // mid_line_list[] 中表示该行无效

#define MIDLINE_WEIGHT_MIN      (1)                     // 权重数组的起始值（最远处那一行）
#define MIDLINE_WEIGHT_GAIN     (0.20f)                 // 每往下一行的权重增量（越靠近车头越大）

#define MIDLINE_FILTER_NEW      (0.8f)                  // 互补滤波：本次值的系数
#define MIDLINE_FILTER_OLD      (0.2f)                  // 互补滤波：上次值的系数（两者和为 1）

#define MIDLINE_MIN_ROWS        (5)                     // 参与加权计算的最少行数
#define MIDLINE_ONE_SIDE_SCALE  (0.50f)                 // 只有单侧边线时用赛道半宽推算，权重打 5 折
#define MIDLINE_DEFAULT_WIDTH   (110.0f)                // 尚未统计出赛道宽度时使用的默认宽度（像素）

/* ================================ 数据结构 ================================ */
typedef struct
{
    uint8  valid;                                       // 1-本帧中线计算成功
    uint8  num;                                         // 参与加权的行数
    uint8  row_start, row_end;                          // 计算区间（row_start 靠车头）
    uint32 weight_sum;                                  // 权重累加值 Σw
    uint8  mid_line;                                    // 本次加权中线值（滤波前）
    uint8  mid_line_value;                              // 互补滤波后的中线输出（给舵机用这个）
    float  last_mid_line;                               // 上一帧的中线输出（滤波状态）
    float  center;                                      // 滤波后的中线（浮点，画图/调试用）
    float  error;                                       // 偏差 = center - MT9V03X_W/2（正=偏右）
    float  width;                                       // 本帧加权平均赛道宽度（像素）
    uint8  line[MT9V03X_H];                             // mid_line_list：每一行的中线列
} midline_struct;

extern midline_struct midline_info;

/* 权重数组：mid_weight_list[row]，越靠近车头权重越大，可自行按行修改（例如丢线区域置 0） */
extern uint8 mid_weight_list[MT9V03X_H];

/* ================================ 接口声明 ================================ */
void    MIDLine_Init            (void);                 // 清空中线信息 + 生成权重数组
uint8   Find_MIDLine            (void);                 // 加权求中线 + 互补滤波
void    MIDLine_Show_On_TFT180  (void);                 // 画出中线并显示中线值/偏差

#endif /* CODE_MIDLINE_REMAIN_H_ */
