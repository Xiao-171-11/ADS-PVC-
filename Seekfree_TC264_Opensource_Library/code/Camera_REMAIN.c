/*
 * Camera_REMAIN.c
 *
 *  Created on: 2026年10月20日
 *      Author: Freed
 */
#include "Camera_REMAIN.h"

uint8 base_image[MT9V03X_H][MT9V03X_W];                 // 二值化结果：0-黑(黑线)  255-白(背景)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     摄像头 + TFT180 初始化
// 参数说明     void
// 返回参数     void
// 使用示例     Camera_LCD_Init();
// 备注信息     先初始化屏幕，摄像头初始化失败时可以直接在屏幕上报错
//-------------------------------------------------------------------------------------------------------------------
void Camera_LCD_Init(void)
{
    tft180_init();                                      // 1.8 寸 TFT 初始化（硬件 SPI2，P15 口）
    tft180_full(0x0000);                                // 上电清屏，避免残影被误认为一片白
    tft180_set_color(0xFFFF, 0x0000);                   // 白字黑底

    if(mt9v03x_init())                                  // 0-成功  1-失败
    {
        tft180_show_string(0, 0, "CAMERA ERR");         // 摄像头没接好/配置失败时的提示
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     大津法(OTSU)计算二值化阈值
// 参数说明     image           8bit 灰度图像数组指针
// 参数说明     width           图像实际宽度
// 参数说明     height          图像实际高度
// 返回参数     uint8           二值化阈值 1~254
// 使用示例     threshold = Camera_SET_Mode(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
// 备注信息     1. 直方图放 static 区，不占用只有 2K 的 CPU0 用户栈
//              2. 输入全 0 / 全 255 这类退化图像时返回 128，绝不返回 0（返回 0 会导致整屏全白）
//-------------------------------------------------------------------------------------------------------------------
uint8 Camera_SET_Mode(const uint8 *image, uint32 width, uint32 height)
{
    static uint32 histogram[256];                       // 256*4 = 1KB，必须放静态区

    uint32 total_pixels = width * height;
    if((NULL == image) || (0 == total_pixels))
    {
        return (128);
    }

    memset(histogram, 0, sizeof(histogram));
    for(uint32 i = 0; i < total_pixels; i ++)
    {
        histogram[image[i]] ++;
    }

    float sum = 0;                                      // 整幅图像的灰度总和
    for(int i = 0; i < 256; i ++)
    {
        sum += (float)i * (float)histogram[i];
    }

    float   sum_b     = 0;                              // 背景灰度和
    float   var_max   = -1.0f;                          // 最大类间方差
    uint32  w_b       = 0;                              // 背景像素个数
    uint8   threshold = 128;                            // 默认阈值（退化保护）

    for(int t = 0; t < 256; t ++)
    {
        w_b += histogram[t];
        if(0 == w_b)
        {
            continue;                                   // 前面全是空桶，跳过
        }

        uint32 w_f = total_pixels - w_b;
        if(0 == w_f)
        {
            break;                                      // 后面没有前景了，结束（原代码这里 break 且没给默认值 → 返回 0）
        }

        sum_b += (float)t * (float)histogram[t];

        float m_b = sum_b / (float)w_b;
        float m_f = (sum - sum_b) / (float)w_f;
        float var_between = (float)w_b * (float)w_f * (m_b - m_f) * (m_b - m_f);

        if(var_between > var_max)
        {
            var_max   = var_between;
            threshold = (uint8)t;
        }
    }

    if((0 == threshold) || (255 == threshold))           // 阈值贴到两端会把图像判成全黑 / 全白
    {
        threshold = 128;
    }

    return (threshold);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把 mt9v03x_image 拷到 base_image 并按阈值二值化成 0/255
// 参数说明     sata            二值化阈值
// 返回参数     void
// 使用示例     Camera_SET_Mode_RETURN(threshold);
// 备注信息     注意行列顺序：base_image 是 [MT9V03X_H][MT9V03X_W]，行循环用 H，列循环用 W
//              原来写反（i<W 当行、j<H 当列）会越界写 12716 字节，还会留下 8160 个没处理的像素
//-------------------------------------------------------------------------------------------------------------------
void Camera_SET_Mode_RETURN(uint8 sata)
{
    if(0 == sata)                                       // 阈值 0 会把所有像素判成白色，兜底
    {
        sata = 128;
    }

    memcpy(base_image[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);

    for(int j = 0; j < MT9V03X_H; j ++)                 // 行：MT9V03X_H = 120
    {
        for(int i = 0; i < MT9V03X_W; i ++)             // 列：MT9V03X_W = 188
        {
            base_image[j][i] = (base_image[j][i] <= sata) ? (0) : (255);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把二值化后的 base_image 显示到 TFT180
// 参数说明     void
// 返回参数     void
// 使用示例     Camera_LCD_Dis_Show();
// 备注信息     最后一个参数 128 表示由驱动按阈值二值化：< 128 画黑，>= 128 画白
//              传 0 表示不二值化、原样输出灰度，排查时用得上
//-------------------------------------------------------------------------------------------------------------------
void Camera_LCD_Dis_Show(void)
{
    tft180_show_gray_image(0, 0, base_image[0],
                           MT9V03X_W, MT9V03X_H,
                           CAMERA_DIS_W, CAMERA_DIS_H,
                           128);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     直接显示摄像头原始灰度图（不做二值化），排查时用
// 参数说明     void
// 返回参数     void
// 使用示例     把 SATA_Loop() 里的 Camera_Process_Once() 临时换成 Camera_LCD_Dis_Raw();
// 备注信息     这里 threshold 传 0 = 不二值化，原样输出 8bit 灰度
//-------------------------------------------------------------------------------------------------------------------
void Camera_LCD_Dis_Raw(void)
{
    tft180_show_gray_image(0, 0, mt9v03x_image[0],
                           MT9V03X_W, MT9V03X_H,
                           CAMERA_DIS_W, CAMERA_DIS_H,
                           0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     一帧完整处理：求阈值 → 二值化 → 显示
// 参数说明     void
// 返回参数     void
// 使用示例     Camera_Process_Once();
// 备注信息     调用前必须保证 mt9v03x_finish_flag 已经置 1（一帧采集完成）
//-------------------------------------------------------------------------------------------------------------------
void Camera_Process_Once(void)
{
    uint8 threshold = Camera_SET_Mode(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);  // 用真实图像算阈值

    Camera_SET_Mode_RETURN(threshold);                  // 二值化到 base_image
    Camera_LCD_Dis_Show();                              // 显示二值化图像

#if (1 == CAMERA_DIS_INFO)
    tft180_set_color(0xFFFF, 0x0000);
    tft180_show_string(0, CAMERA_DIS_H + 6, "TH");
    tft180_show_int(24, CAMERA_DIS_H + 6, (int32)threshold, 3);
#endif
}
