/*
 * MIDLine_REMAIN.c
 *
 *  加权求中线值（含互补滤波）
 *
 *  每一行：
 *      left_col  = k_left  * row + b_left         （左侧边缘基线）
 *      right_col = k_right * row + b_right        （右侧边缘基线）
 *      mid       = (left_col + right_col) / 2     两侧都有效
 *      mid       = left_col  + width / 2          只有左侧有效（用上一帧统计的赛道宽度推算）
 *      mid       = right_col - width / 2          只有右侧有效
 *  存进 mid_line_list[row]，该行的权重取 mid_weight_list[row]（单侧推算时权重减半）
 *
 *  加权：  mid_line      = Σ(mid * w) / Σw
 *  滤波：  mid_line_value = last_mid_line * 0.2 + mid_line * 0.8      互补滤波（一阶低通）
 *  输出：  error         = mid_line_value - MT9V03X_W / 2
 *
 *  Created on: 2026年10月21日
 *      Author: Freed
 */
#include "MIDLine_REMAIN.h"

midline_struct midline_info;                            // 中线计算结果
uint8          mid_weight_list[MT9V03X_H];              // mid_weight_list[row]：每行的中线权重

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     图像坐标 -> TFT180 显示坐标（与 tft180_show_gray_image 的缩放一致）
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint16 midline_to_dis_x(uint8 col)
{
    return ((uint16)((uint32)col * CAMERA_DIS_W / MT9V03X_W));
}

static uint16 midline_to_dis_y(uint8 row)
{
    return ((uint16)((uint32)row * CAMERA_DIS_H / MT9V03X_H));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空中线信息，并按行生成权重数组
// 参数说明     void
// 返回参数     void
// 使用示例     MIDLine_Init();
// 备注信息     1. 权重规律：mid_weight_list[row] = MIDLINE_WEIGHT_MIN + row * MIDLINE_WEIGHT_GAIN
//                 即行号越大（越靠近车头）权重越大
//              2. 想自己调权重，初始化之后直接改 mid_weight_list[] 即可，例如
//                 mid_weight_list[0] = 0;    // 最远一行不参与
//-------------------------------------------------------------------------------------------------------------------
void MIDLine_Init(void)
{
    memset(&midline_info, 0, sizeof(midline_info));

    midline_info.mid_line       = MIDLINE_DEFAULT_CENTER;
    midline_info.mid_line_value = MIDLINE_DEFAULT_CENTER;
    midline_info.last_mid_line  = (float)MIDLINE_DEFAULT_CENTER;
    midline_info.center         = (float)MIDLINE_DEFAULT_CENTER;

    for(uint16 i = 0; i < MT9V03X_H; i ++)
    {
        midline_info.line[i] = MIDLINE_INVALID_COL;     // mid_line_list 全部置无效
        mid_weight_list[i]   = (uint8)(MIDLINE_WEIGHT_MIN + (uint8)((float)i * MIDLINE_WEIGHT_GAIN));
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     加权求中线值 + 互补滤波
// 参数说明     void
// 返回参数     uint8           0-成功  1-失败（边线无效或有效行数不足）
// 使用示例     Find_MIDLine();
// 备注信息     1. 依赖 Bedline_REMAIN 的 bedline_left / bedline_right，必须先调用 Find_Bedline()
//              2. 行区间取左右两条基线的公共部分；只有一侧有效时用那一侧的行区间
//              3. 单侧推算用的赛道宽度取上一帧的加权宽度，首帧用 MIDLINE_DEFAULT_WIDTH
//              4. 结果看 midline_info.mid_line_value（滤波后）和 midline_info.error
//-------------------------------------------------------------------------------------------------------------------
uint8 Find_MIDLine(void)
{
    float width_ref = (midline_info.width > 1.0f) ? (midline_info.width) : (MIDLINE_DEFAULT_WIDTH);
    int   row_start = -1;
    int   row_end   = -1;

    midline_info.valid      = 0;
    midline_info.num        = 0;
    midline_info.weight_sum = 0;

    for(uint16 i = 0; i < MT9V03X_H; i ++)
    {
        midline_info.line[i] = MIDLINE_INVALID_COL;     // mid_line_list 全部置无效
    }

    if(!bedline_left.valid && !bedline_right.valid)     // 两侧边线都无效，无法求中线
    {
        return (1);
    }

    // 1. 计算行区间：取两条基线的公共区间（row 大的一端靠车头）
    if(bedline_left.valid)
    {
        row_start = (int)bedline_left.row_start;
        row_end   = (int)bedline_left.row_end;
    }
    if(bedline_right.valid)
    {
        if(row_start < 0)
        {
            row_start = (int)bedline_right.row_start;
            row_end   = (int)bedline_right.row_end;
        }
        else
        {
            if((int)bedline_right.row_start > row_start)
            {
                row_start = (int)bedline_right.row_start;
            }
            if((int)bedline_right.row_end < row_end)
            {
                row_end = (int)bedline_right.row_end;
            }
        }
    }

    if((row_start < 0) || ((row_start - row_end + 1) < MIDLINE_MIN_ROWS))
    {
        return (1);
    }

    // 2. 逐行求中线，并按 mid_weight_list[row] 加权累加
    uint32 weight_midline_sum = 0;                      // 加权中线累加值 Σ(mid * w)
    uint32 weight_sum         = 0;                      // 权重累加值     Σw
    float  sum_w_width        = 0.0f;                   // Σ(w * 赛道宽度)
    float  sum_w_both         = 0.0f;                   // 两侧都有效的行权重和
    uint8  num                = 0;

    for(int row = row_start; row >= row_end; row --)
    {
        float left_col  = 0.0f;
        float right_col = 0.0f;
        uint8 ok_left   = 0;
        uint8 ok_right  = 0;

        if(bedline_left.valid)                          // 左边界：基线在该行的列号
        {
            float c = (bedline_left.k * (float)row) + bedline_left.b;
            if((c >= 0.0f) && (c <= (float)(MT9V03X_W - 1)))
            {
                left_col = c;
                ok_left  = 1;
            }
        }
        if(bedline_right.valid)                         // 右边界
        {
            float c = (bedline_right.k * (float)row) + bedline_right.b;
            if((c >= 0.0f) && (c <= (float)(MT9V03X_W - 1)))
            {
                right_col = c;
                ok_right  = 1;
            }
        }

        uint32 w_row = (uint32)mid_weight_list[row];    // 该行权重（来自可调权重数组）
        float  mid   = 0.0f;

        if(ok_left && ok_right && (right_col > left_col))                   // 两侧都有效：取中点
        {
            mid = (left_col + right_col) * 0.5f;

            sum_w_width += ((float)w_row * (right_col - left_col));         // 顺便统计赛道宽度
            sum_w_both  += (float)w_row;
        }
        else if(ok_left || ok_right)                                        // 只有单侧：用半宽推算
        {
            float half_width = width_ref * 0.5f;
            mid   = ok_left ? (left_col + half_width) : (right_col - half_width);
            w_row = (uint32)((float)w_row * MIDLINE_ONE_SIDE_SCALE);        // 可信度打折
        }
        else
        {
            continue;                                                       // 本行两侧都无效
        }

        if(mid < 0.0f)                                  // 限幅到图像范围
        {
            mid = 0.0f;
        }
        if(mid > (float)(MT9V03X_W - 1))
        {
            mid = (float)(MT9V03X_W - 1);
        }

        midline_info.line[row] = (uint8)mid;            // mid_line_list[row]
        weight_midline_sum    += (uint32)((uint8)mid) * w_row;
        weight_sum            += w_row;
        num ++;
    }

    // 3. 加权求和得到本次中线
    if((num < MIDLINE_MIN_ROWS) || (0 == weight_sum))
    {
        return (1);
    }

    uint8 mid_line = (uint8)(weight_midline_sum / weight_sum);

    // 4. 互补滤波（一阶低通）：本次值占 MIDLINE_FILTER_NEW，上次占 MIDLINE_FILTER_OLD
    midline_info.num            = num;
    midline_info.weight_sum     = weight_sum;
    midline_info.mid_line       = mid_line;                                 // 滤波前
    midline_info.center         = (midline_info.last_mid_line * MIDLINE_FILTER_OLD) +
                                  ((float)mid_line * MIDLINE_FILTER_NEW);   // 滤波后
    midline_info.last_mid_line  = midline_info.center;                      // 更新滤波状态
    midline_info.mid_line_value = (uint8)(midline_info.center + 0.5f);      // 四舍五入成整数输出
    midline_info.error          = midline_info.center - ((float)MT9V03X_W * 0.5f);

    midline_info.row_start = (uint8)row_start;
    midline_info.row_end   = (uint8)row_end;

    if(sum_w_both > 0.0f)                           // 只用“两侧都有效”的行更新赛道宽度
    {
        midline_info.width = sum_w_width / sum_w_both;
    }
    else
    {
        midline_info.width = width_ref;             // 本帧全是单侧，沿用上一帧宽度
    }

    midline_info.valid = 1;
    return (0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在屏幕上画出中线，并显示滤波后的中线值与偏差
// 参数说明     void
// 返回参数     void
// 使用示例     MIDLine_Show_On_TFT180();
// 备注信息     中线用绿色点绘制；必须排在 tft180_show_gray_image 之后才会覆盖在图像上
//-------------------------------------------------------------------------------------------------------------------
void MIDLine_Show_On_TFT180(void)
{
    if(midline_info.valid)
    {
        for(int row = (int)midline_info.row_start; row >= (int)midline_info.row_end; row --)
        {
            if(midline_info.line[row] == MIDLINE_INVALID_COL)
            {
                continue;
            }

            uint16 x = midline_to_dis_x(midline_info.line[row]);
            uint16 y = midline_to_dis_y((uint8)row);
            if((x < tft180_width_max) && (y < tft180_height_max))
            {
                tft180_draw_point(x, y, RGB565_GREEN);
            }
        }
    }

    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(0, CAMERA_INFO_Y4, "C");                                     // 滤波后的中线值
    tft180_show_int(14, CAMERA_INFO_Y4, (int32)midline_info.mid_line_value, 3);
    tft180_show_string(64, CAMERA_INFO_Y4, "E");                                    // 偏差
    tft180_show_float(78, CAMERA_INFO_Y4, (double)midline_info.error, 2, 1);
}
