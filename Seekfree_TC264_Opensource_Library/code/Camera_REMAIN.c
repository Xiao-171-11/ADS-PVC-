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
// 函数简介     大津法（OTSU / 最大类间方差法）自动计算二值化阈值
// 参数说明     image           8bit 灰度图像数组指针
// 参数说明     width           图像实际宽度
// 参数说明     height          图像实际高度
// 返回参数     uint8           阈值 T* ∈ [0,255]，正常赛道图像落在 1~254
// 使用示例     threshold = Camera_Otsu_Threshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
//
// 【算法原理】大津法 Otsu (1979)：遍历 256 个候选阈值，取“类间方差最大”的那一个
//     ① 统计直方图   n[i] 为灰度 i 的像素个数（i = 0..255），总像素数 N = width * height
//     ② 对候选阈值 t，把图像分成两类：
//            C0 = [0, t]      （灰度低 —— 赛道黑线）
//            C1 = [t+1, 255]  （灰度高 —— 白色背景）
//     ③ 计算两类的像素占比与灰度均值：
//            w_b = Σ(i<=t) n[i]                    C0 像素个数
//            w_f = N - w_b                          C1 像素个数
//            μ0  = Σ(i<=t) i*n[i] / w_b             C0 灰度均值
//            μ1  = Σ(i>t)  i*n[i] / w_f             C1 灰度均值
//     ④ 类间方差（越大说明两类分得越开）：
//            g(t) = w_b * w_f * (μ0 - μ1)^2
//     ⑤ T* = argmax_t g(t)，即让 g(t) 取最大值的 t
//
// 【实现说明】用累加量避免每次重算：
//            w_b  边遍历边累加直方图；
//            sum_b 边遍历边累加 t*n[t]；
//            sum   为全图灰度和 Σ i*n[i]，于是 μ1 里的 Σ(i>t) i*n[i] = sum - sum_b
//     复杂度：直方图 O(N) + 阈值遍历 O(256) + 二值化 O(N)；188*120 图像为微秒级
//     相比固定阈值的优势：每帧按当前光照、反光、赛道材质自适应，换场地不用重调参数
//
// 备注信息     1. 直方图放 static 区（1KB），不占用只有 2K 的 CPU0 用户栈
//              2. 本函数只做纯大津法计算，不做任何修正；退化情况（整幅图灰度单一）由调用者兜底
//-------------------------------------------------------------------------------------------------------------------
uint8 Camera_Otsu_Threshold(const uint8 *image, uint32 width, uint32 height)
{
    static uint32 histogram[256];                       // 256*4 = 1KB，必须放静态区

    uint32 total_pixels = width * height;
    if((NULL == image) || (0 == total_pixels))
    {
        return (0);                                     // 参数非法，交给调用者兜底
    }

    memset(histogram, 0, sizeof(histogram));            // ① 统计直方图
    for(uint32 i = 0; i < total_pixels; i ++)
    {
        histogram[image[i]] ++;
    }

    float sum = 0;                                      // 全图灰度和 Σ i*n[i]
    for(int i = 0; i < 256; i ++)
    {
        sum += (float)i * (float)histogram[i];
    }

    float   sum_b     = 0;                              // Σ(i<=t) i*n[i]
    float   var_max   = -1.0f;                          // 最大类间方差 g(t)
    uint32  w_b       = 0;                              // C0 像素个数
    uint8   threshold = 0;                              // 当前最优阈值 T*

    for(int t = 0; t < 256; t ++)                       // ②~⑤ 遍历 256 个候选阈值
    {
        w_b += histogram[t];
        if(0 == w_b)
        {
            continue;                                   // C0 里还没有像素，跳过
        }

        uint32 w_f = total_pixels - w_b;                // C1 像素个数
        if(0 == w_f)
        {
            break;                                      // C1 里没有像素了，后面不可能更大
        }

        sum_b += (float)t * (float)histogram[t];

        float m_b = sum_b / (float)w_b;                 // μ0
        float m_f = (sum - sum_b) / (float)w_f;         // μ1
        float var_between = (float)w_b * (float)w_f * (m_b - m_f) * (m_b - m_f);   // g(t) = w0*w1*(μ0-μ1)2

        if(var_between > var_max)                       // ⑤ 记录最大类间方差对应的阈值
        {
            var_max   = var_between;
            threshold = (uint8)t;
        }
    }

    return (threshold);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按给定阈值把 mt9v03x_image 二值化到 base_image（0 / 255）
// 参数说明     threshold       二值化阈值，由 Camera_Otsu_Threshold() 给出
// 返回参数     void
// 使用示例     Camera_Binarize(Camera_Otsu_Threshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H));
// 备注信息     1. 灰度 > threshold 判为白(255)，否则判为黑(0)——与屏幕驱动 threshold 分支一致
//              2. 行列顺序：base_image 是 [MT9V03X_H][MT9V03X_W]，行循环用 H，列循环用 W
//                 写反（i<W 当行、j<H 当列）会越界写 12716 字节，还会漏掉 8160 个像素
//-------------------------------------------------------------------------------------------------------------------
void Camera_Binarize(uint8 threshold)
{
    memcpy(base_image[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);

    for(int j = 0; j < MT9V03X_H; j ++)                 // 行：MT9V03X_H = 120
    {
        for(int i = 0; i < MT9V03X_W; i ++)             // 列：MT9V03X_W = 188
        {
            base_image[j][i] = (base_image[j][i] > threshold) ? (255) : (0);
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
// 使用示例     把 SATA_MAIN.c 里 SATA_Frame_Process() 的 Camera_Process_Once() 临时换成 Camera_LCD_Dis_Raw();
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
// 函数简介     一帧完整处理：大津法求阈值 → 二值化 → 显示
// 参数说明     void
// 返回参数     void
// 使用示例     Camera_Process_Once();
// 备注信息     调用前必须保证 mt9v03x_finish_flag 已经置 1（一帧采集完成）
//-------------------------------------------------------------------------------------------------------------------
void Camera_Process_Once(void)
{
    // 1. 大津法：对当前这一帧真实图像计算自适应阈值
    uint8 threshold = Camera_Otsu_Threshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);

    // 2. 退化兜底：整幅图灰度完全一致（镜头被遮挡、摄像头未出图）时，
    //    任何候选阈值的类间方差都是 0，大津法给出的 0 / 255 会把整屏刷成纯白或纯黑。
    //    正常赛道图像不会走到这里，不影响大津法的计算结果。
    if((0 == threshold) || (255 == threshold))
    {
        threshold = 128;
    }

    // 3. 按大津法阈值二值化
    Camera_Binarize(threshold);

#if (1 == CAMERA_DIS_IMAGE)
    Camera_LCD_Dis_Show();                              // 图像显示；CAMERA_DIS_IMAGE=0 时整段跳过
#endif

#if (1 == CAMERA_DIS_INFO)
    tft180_set_color(0xFFFF, 0x0000);
    tft180_show_string(0, CAMERA_INFO_Y1, "TH");
    tft180_show_int(24, CAMERA_INFO_Y1, (int32)threshold, 3);
#endif
}
