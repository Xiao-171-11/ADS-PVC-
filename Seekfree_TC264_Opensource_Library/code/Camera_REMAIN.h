/*
 * Camera_REMAIN.h
 *
 *  Created on: 2026年10月20日
 *      Author: Freed
 */

#ifndef CODE_CAMERA_REMAIN_H_
#define CODE_CAMERA_REMAIN_H_

#include "zf_common_headfile.h"

/* ================================ 显示参数 ================================ */
/* 总钻风原图 188*120，TFT180 竖屏可用区域 128*160                       */
/* 显示宽度不能超过 tft180_width_max，显示高度不能超过 tft180_height_max   */
#define CAMERA_DIS_W            (126)                   // 屏幕显示宽度  126/188 等比缩放
#define CAMERA_DIS_H            (80)                    // 屏幕显示高度  80/120  等比缩放
#define CAMERA_DIS_INFO         (1)                     // 1-在图像下方显示当前阈值  0-不显示

/* 屏幕下方信息区行坐标（8x16 字体，最后一行 y+15 必须小于 tft180_height_max=160） */
#define CAMERA_INFO_Y1          (CAMERA_DIS_H + 4)      // 84  阈值 TH
#define CAMERA_INFO_Y2          (CAMERA_DIS_H + 22)     // 102 基点 BP
#define CAMERA_INFO_Y3          (CAMERA_DIS_H + 40)     // 120 斜率 K
#define CAMERA_INFO_Y4          (CAMERA_DIS_H + 58)     // 138 中线 C / 偏差 E
/* base_image：二值化结果 0-黑(赛道黑线)  255-白(背景)                     */
extern uint8 base_image[MT9V03X_H][MT9V03X_W];

/* ================================ 接口声明 ================================ */
void    Camera_LCD_Init          (void);                                            // 摄像头 + TFT180 初始化

/* 二值化阈值一律由大津法（OTSU，最大类间方差法）计算，不使用固定阈值 */
uint8   Camera_Otsu_Threshold    (const uint8 *image, uint32 width, uint32 height);  // 大津法求阈值 T*
void    Camera_Binarize          (uint8 threshold);                                 // 按阈值二值化到 base_image

void    Camera_LCD_Dis_Show      (void);                                            // 显示 base_image 二值化图像
void    Camera_LCD_Dis_Raw       (void);                                            // 显示原始灰度图（排查用）
void    Camera_Process_Once      (void);                                            // 大津法阈值 + 二值化 + 显示（一帧完整流程）

#endif /* CODE_CAMERA_REMAIN_H_ */
