/*
 * IPM_REMAIN.h
 *
 *  逆透视变换（Inverse Perspective Mapping）：把摄像头看到的斜视图重采样成地面俯视图
 *
 *  为什么需要：
 *    1. 摄像头是斜着看赛道的：画面最下边 1 像素 = 2 mm 地面，画面上边 1 像素 = 100 mm，
 *       直接在图像坐标上算偏差，等于"近处放大、远处缩小"，权重全是乱的。
 *    2. 纯追踪（pure pursuit）必须在地面坐标系里选前瞻点、算曲率，所以先要有俯视图。
 *
 *  实现方式：查表，不做运行时矩阵运算。
 *    上电时 IPM_Init() 用针孔模型把 96 x 64 个地面格点反算回摄像头像素，
 *    把像素下标存进 ipm_lut[][]，之后每帧 IPM_Process() 只是 6144 次查表 + 赋值。
 *    RUBO-UDIPM 和 losehu/smartcar-C 两套独立开源实现都是这个套路，
 *    区别是他们把标定好的 3x3 矩阵写死；这里是按安装参数现场算，
 *    换车/换安装高度只需要改下面三个宏，不用重新标定。
 *
 *  坐标系（车辆坐标系）：X 向右为正，Y 向前为正，Z 向上为正；
 *    原点是两个后轮中点正下方的地面，相机光心在 (0, 0, IPM_CAM_HEIGHT_MM)。
 *
 *  标定方法（没有尺子也能做）：
 *    1. 把车放在赛道上，正对前方，量一下镜头离地高度 -> IPM_CAM_HEIGHT_MM
 *    2. 量一下镜头光轴与水平面的夹角 -> IPM_CAM_PITCH_DEG（俯角，向下为正）
 *    3. IPM_CAM_FOV_H_DEG 是横向视场角，总钻风 MT9V03X 配 2.8mm 镜头约 100 度
 *    4. 打开 IPM_DIS_EN，屏幕上直接看俯视图：赛道两条边线应该是平行的竖线；
 *       不平行就微调 FOV（大了边线向内收，小了向外张），不在画面中间就调 PITCH
 *
 *  Created on: 2026年10月28日
 *      Author: Freed
 */

#ifndef CODE_IPM_REMAIN_H_
#define CODE_IPM_REMAIN_H_
#include "zf_common_headfile.h"

/* ============================ 俯视图尺寸 ============================ */
/* 96 列 x 6.7mm 横向覆盖 +-320mm（够 450mm 赛道左右各留 95mm 余量）。
 * 64 行 x 19.8mm 从 150mm 看到 1400mm：路口上方必须能看到"路面又回来了"，
 * 所以远端不能只做到 800mm（800mm 处只剩源图第 52 行，路口一挡就什么都看不见）。 */
#define IPM_H                   (64)
#define IPM_W                   (96)

/* ============================ 地面覆盖范围（mm） ============================ */
/* 最近 150mm：总钻风装 240mm 高、俯角 22 度时，画面最下沿大约对应车头前 144mm，
 *             所以 150mm 是最靠近车的有效行，再近就是画面外了。 */
#define IPM_Y_NEAR_MM           (150.0f)
/* 最远 1400mm：22 度俯角下对应源图第 42 行，再往后源图只剩 10 行且地面分辨率
 *             已经退化到 90mm/像素以上（模糊但没有坏处，判"有没有路面"够用）。 */
#define IPM_Y_FAR_MM            (1400.0f)
#define IPM_X_HALF_MM           (320.0f)

/* ============================ 相机安装参数（换车就改这三个） ============================ */
#define IPM_CAM_HEIGHT_MM       (240.0f)                // 镜头光心离地高度
#define IPM_CAM_PITCH_DEG       (22.0f)                 // 光轴俯角（向下为正）
#define IPM_CAM_FOV_H_DEG       (100.0f)                // 横向视场角（2.8mm 镜头约 100 度）

/* 查表哨兵：这个格点在摄像头画面外（或在地平线以上），填白色 */
#define IPM_LUT_INVALID         (0xFFFFu)
#define IPM_BLACK               (0)
#define IPM_WHITE               (255)

/* 1 = 屏幕上直接显示俯视图（调试标定用，会盖住上面几行信息） */
#define IPM_DIS_EN              (0)

typedef struct
{
    uint8  ok;                                          // 1 = LUT 有效（参数算出来在画面内）
    uint8  horizon_row;                                 // 地平线所在行（越小说明俯角越大）
    uint16 invalid_cnt;                                 // 画面外的格点数（>20% 说明参数配错了）
    float  focal_px;                                    // 等效焦距（像素）
    float  dx_mm;                                       // 每个格点横向代表多少 mm
    float  dy_mm;                                       // 每个格点纵向代表多少 mm
} ipm_struct;

extern ipm_struct ipm_info;
extern uint8 ipm_image[IPM_H][IPM_W];                   // 俯视二值图：0=黑线 255=白

void  IPM_Init(void);                                   // 算 LUT（上电调一次）
uint8 IPM_Process(const uint8 *image);                  // 每帧调用；返回 0-成功 1-参数无效
void  IPM_Show_On_TFT180(void);                         // IPM_DIS_EN=1 时显示俯视图

#endif /* CODE_IPM_REMAIN_H_ */
