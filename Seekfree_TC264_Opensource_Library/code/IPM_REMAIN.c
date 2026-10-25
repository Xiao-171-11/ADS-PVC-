/*
 * IPM_REMAIN.c
 *
 *  逆透视变换：摄像头图像 -> 地面俯视图（查表实现）
 *
 *  核心思想只有一句话：地面上每个格点，上电时算一次"它落在摄像头第几行第几列"，
 *  以后每帧只做一次数组取值。所以运行时没有三角函数、没有矩阵乘法、没有除法。
 *
 *  数学推导（针孔模型，车辆坐标系 X右 Y前 Z上）：
 *      相机光心 (0, 0, h)，俯角 theta（向下为正）
 *      相机三轴：x_c = (1, 0, 0)               向右
 *                y_c = (0, -sin, -cos)        图像向下
 *                z_c = (0,  cos, -sin)        光轴向前下
 *      地面点 (X, Y, 0) 在相机系下：
 *              x_c = X
 *              y_c = h*cos - Y*sin
 *              z_c = Y*cos + h*sin
 *      投到像平面（f 为焦距，单位像素）：
 *              u = cx + f * x_c / z_c
 *              v = cy + f * y_c / z_c
 *      z_c 就是"该点有多远"，只要 z_c 很小（接近地平线）就判为无效点。
 *
 *  Created on: 2026年10月28日
 *      Author: Freed
 */
#include "IPM_REMAIN.h"

#define IPM_PI_F                (3.14159265f)

static uint16 ipm_lut[IPM_H][IPM_W];                    // 每个格点对应的摄像头像素下标（行 * 宽 + 列）

ipm_struct ipm_info;
uint8 ipm_image[IPM_H][IPM_W];

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逆透视查表初始化（上电调一次）
// 参数说明     void
// 返回参数     void
// 使用示例     IPM_Init();
// 备注信息     1. 只需要 IPM_CAM_HEIGHT_MM / IPM_CAM_PITCH_DEG / IPM_CAM_FOV_H_DEG 三个参数，
//                 换镜头、改安装高度都只改宏，不重新标定
//              2. 算完会检查"出界格点比例"，超过 1/4 就认为参数配错，ipm_info.ok=0，
//                 此时 IPM_Process() 直接返回失败，纯追踪会自动退回老的中线 PD 打角
//-------------------------------------------------------------------------------------------------------------------
void IPM_Init(void)
{
    float  pitch_rad = IPM_CAM_PITCH_DEG * IPM_PI_F / 180.0f;
    float  fov_rad   = IPM_CAM_FOV_H_DEG * IPM_PI_F / 180.0f;
    float  sin_p     = sinf(pitch_rad);
    float  cos_p     = cosf(pitch_rad);
    float  focal     = (((float)MT9V03X_W) * 0.5f) / tanf(fov_rad * 0.5f);
    float  cx        = (((float)MT9V03X_W) * 0.5f) - 0.5f;
    float  cy        = (((float)MT9V03X_H) * 0.5f) - 0.5f;
    float  cam_h     = IPM_CAM_HEIGHT_MM;
    float  horizon   = 0.0f;
    uint16 invalid   = 0;
    uint8  r         = 0;
    uint8  c         = 0;

    ipm_info.focal_px = focal;
    ipm_info.dx_mm    = (2.0f * IPM_X_HALF_MM) / ((float)(IPM_W - 1));
    ipm_info.dy_mm    = (IPM_Y_FAR_MM - IPM_Y_NEAR_MM) / ((float)(IPM_H - 1));

    /* 地平线所在行：Y 趋于无穷时 v = cy - f*tan(theta)。
       这一行以上没有地面，行号越小说明相机压得越平（俯角越小）。 */
    horizon = cy - (focal * tanf(pitch_rad));
    if(horizon < 0.0f)
    {
        ipm_info.horizon_row = 0;
    }
    else if(horizon > ((float)MT9V03X_H))
    {
        ipm_info.horizon_row = (uint8)MT9V03X_H;
    }
    else
    {
        ipm_info.horizon_row = (uint8)(horizon + 0.5f);
    }

    for(r = 0; r < IPM_H; r++)
    {
        float y_ground = IPM_Y_NEAR_MM + (((float)r) * ipm_info.dy_mm);
        float z_c      = (y_ground * cos_p) + (cam_h * sin_p);
        float y_c      = (cam_h * cos_p) - (y_ground * sin_p);

        for(c = 0; c < IPM_W; c++)
        {
            float x_ground = (((float)c) - (((float)(IPM_W - 1)) * 0.5f)) * ipm_info.dx_mm;
            float u_px     = 0.0f;
            float v_px     = 0.0f;

            if(z_c <= 1.0f)                             /* 该点在地平线上，除不得 */
            {
                ipm_lut[r][c] = IPM_LUT_INVALID;
                invalid ++;
                continue;
            }

            u_px = cx + ((focal * x_ground) / z_c);
            v_px = cy + ((focal * y_c) / z_c);

            if((u_px < -0.5f) || (u_px > (((float)MT9V03X_W) - 0.5f)) ||
               (v_px < -0.5f) || (v_px > (((float)MT9V03X_H) - 0.5f)))
            {
                ipm_lut[r][c] = IPM_LUT_INVALID;        /* 该点落在摄像头画面外 */
                invalid ++;
                continue;
            }

            {
                int16 u_i = (int16)(u_px + 0.5f);
                int16 v_i = (int16)(v_px + 0.5f);

                if(u_i < 0)                u_i = 0;
                if(u_i > (MT9V03X_W - 1))  u_i = (MT9V03X_W - 1);
                if(v_i < 0)                v_i = 0;
                if(v_i > (MT9V03X_H - 1))  v_i = (MT9V03X_H - 1);

                ipm_lut[r][c] = (uint16)((v_i * MT9V03X_W) + u_i);
            }
        }
    }

    ipm_info.invalid_cnt = invalid;

    /* 参数自检：
       1. 出界格点超过 1/4 -> 俯视图大部分是空白，参数肯定配错了
       2. 地平线跑到画面里太靠上（<5 行）或者跑到画面外（>115 行）-> 俯角明显不对 */
    ipm_info.ok = 1;
    if((invalid * 4) >= (IPM_H * IPM_W))
    {
        ipm_info.ok = 0;
    }
    if(((float)ipm_info.horizon_row < 5.0f) || ((float)ipm_info.horizon_row > (((float)MT9V03X_H) - 5.0f)))
    {
        ipm_info.ok = 0;
    }

    /* 俯视图先全填白色，第一帧来之前就是一张白纸 */
    for(r = 0; r < IPM_H; r++)
    {
        for(c = 0; c < IPM_W; c++)
        {
            ipm_image[r][c] = (uint8)IPM_WHITE;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     一帧逆透视重采样
// 参数说明     image           二值化图像首地址（base_image[0]，0=黑 255=白）
// 返回参数     uint8           0-成功 1-参数无效/空指针
// 使用示例     IPM_Process(base_image[0]);
// 备注信息     6144 次查表 + 赋值，TC264 上大约 200 us，随便调
//-------------------------------------------------------------------------------------------------------------------
uint8 IPM_Process(const uint8 *image)
{
    uint8 r = 0;
    uint8 c = 0;

    if((NULL == image) || (0 == ipm_info.ok))
    {
        return (1);
    }

    for(r = 0; r < IPM_H; r++)
    {
        const uint16 *lut = ipm_lut[r];
        uint8        *dst = ipm_image[r];

        for(c = 0; c < IPM_W; c++)
        {
            uint16 off = lut[c];

            if(IPM_LUT_INVALID == off)
            {
                dst[c] = (uint8)IPM_WHITE;              /* 画面外当作白色空白 */
            }
            else
            {
                dst[c] = image[off];
            }
        }
    }

    return (0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示俯视图（调试用）
// 参数说明     void
// 返回参数     void
// 使用示例     IPM_Show_On_TFT180();
// 备注信息     1. 只有 IPM_DIS_EN=1 才真正画图，平时是空函数
//              2. 标定的时候打开：赛道两条边线在俯视图里应该是平行的竖线，
//                 上宽下窄说明 FOV 给小了、下宽上窄说明 FOV 给大了
//              3. TFT180 默认竖屏 128x160，俯视图 96x56 画在屏幕最下面（y = 160-56 = 104），
//                 不盖住上面六行文字；高度不够就退回 (0,0)。
//-------------------------------------------------------------------------------------------------------------------
void IPM_Show_On_TFT180(void)
{
#if (1 == IPM_DIS_EN)
    uint16 y = 0;

    if(tft180_height_max > IPM_H)
    {
        y = (uint16)(tft180_height_max - IPM_H);
    }
    tft180_show_gray_image(0, y, ipm_image[0], IPM_W, IPM_H, IPM_W, IPM_H, 128);
#endif
}
