# SATA · 智能车摄像头组工程（TC264 + 总钻风 MT9V03X + TFT180）

> ⚠️ **本仓库中有一部分代码由 AI 辅助生成 / 修改**（大津法二值化、TFT180 二值图显示、帧同步流程等）。
> 具体范围与人工负责部分见文末 [「AI 生成代码声明」](#ai-生成代码声明)。

基于 **逐飞 TC264 开源库（Seekfree TC264 Opensource Library）** 的摄像头组工程：
采集总钻风灰度图像 → 大津法（OTSU）自适应阈值 → 二值化 → 在 1.8 寸 TFT180 上实时显示。

---

## 1. 硬件平台

| 项目 | 型号 / 参数 |
|---|---|
| 主控 | Infineon **TC264D**（TriCore，三核，AURIX） |
| 摄像头 | 逐飞 **总钻风 MT9V03X**，188×120 灰度，50 FPS |
| 显示屏 | 逐飞 **1.8 寸 TFT180**（ST7735，硬件 SPI2，60 MHz） |
| 开发环境 | **AURIX Development Studio (ADS) v1.10.x** + Tasking 编译器 |
| 调试串口 | `debug_init()` 默认调试串口，115200-8-N-1 |

## 2. 编译与烧录（ADS）

1. 安装 **AURIX Development Studio**（含 Tasking TriCore 编译器）。
2. 打开 ADS → `File ▸ Switch Workspace ▸ Other…` → 选择**本仓库根目录**。
3. `File ▸ Import ▸ General ▸ Existing Projects into Workspace`
   → `Select root directory` 选择本仓库根目录 → 勾选 **`Seekfree_TC264_Opensource_Library`** → Finish。
4. `Project ▸ Build Project`（Ctrl+B），编译配置选 **Debug**。
5. 生成的固件：`Seekfree_TC264_Opensource_Library/Debug/Seekfree_TC264_Opensource_Library.hex`。
6. 用 **AURIX Flasher** 或逐飞下载器烧录。

> **首次编译前请先执行一次 `Project ▸ Clean…`**：`Debug/` 编译产物不入库，克隆后需要全量重编。

### 编码说明

工程文件为 **GBK 编码**（`.settings/org.eclipse.core.resources.prefs` 中 `encoding/<project>=GBK`），
`README.md` / `.gitignore` 为 UTF-8。**请勿把 `.settings` 排除在版本控制之外**，否则中文注释在别人机器上会乱码。

## 3. 目录结构

```
ADS_workspace_AI/
├── .settings/                              # Eclipse/ADS 工程设置（含 GBK 编码）
├── .gitignore
├── README.md
└── Seekfree_TC264_Opensource_Library/      # ADS 工程（可直接导入）
    ├── .project / .cproject                # ADS 工程文件
    ├── Lcf_Tasking_Tricore_Tc.lsl          # 链接脚本（CPU0/CPU1 栈、RAM 分配）
    ├── code/                               # 用户功能模块
    │   ├── Camera_REMAIN.c/.h              # ★ 摄像头采集 + OTSU 二值化 + TFT180 显示
    │   ├── SATA.c/.h                       # ★ 主业务：初始化 + 单帧处理入口
    │   ├── LCD_REMAIN.c/.h                 # 屏幕辅助函数
    │   ├── LED_REMAIN.c/.h                 # 指示灯
    │   ├── Motor_REMAIN.c/.h               # 电机
    │   ├── Servo_REMAIN.c/.h               # 舵机
    │   ├── PIT_REMAIN.c/.h                 # 定时器中断
    │   └── Badpoint_REMAIN.c/.h            # 坏点处理（预留）
    ├── libraries/                          # 逐飞 TC264 开源库 + Infineon iLLD
    │   ├── zf_common/  zf_driver/  zf_device/  zf_components/
    │   ├── infineon_libraries/             # 官方 SDK
    │   └── doc/                            # 版本说明 / GPL3 许可
    ├── user/                               # 主入口与中断
    │   ├── cpu0_main.c                     # ★ core0_main：初始化 + SATA_Loop()
    │   ├── cpu1_main.c                     # core1_main
    │   ├── isr.c / isr.h / isr_config.h    # 中断服务函数与配置
    │   └── cpu0_main.h
    └── Debug/                              # 编译产物（.gitignore 已排除）
```

## 4. 接线

### TFT180（硬件 SPI2）

| TFT 引脚 | TC264 引脚 | 说明 |
|---|---|---|
| SCL | `P15_3` | SPI2_SCLK |
| SDA | `P15_5` | SPI2_MOSI |
| RES | `P15_1` | 复位 |
| DC | `P15_0` | 命令/数据 |
| CS | `P15_2` | 片选 |
| BL | `P15_4` | 背光 |
| VCC / GND | 3.3 V / GND | — |

### 总钻风 MT9V03X

| 摄像头引脚 | TC264 引脚 | 说明 |
|---|---|---|
| D0 ~ D7 | `P00_0` ~ `P00_7` | 8 位并行数据（必须是连续 8 个 GPIO，从 x0 或 x8 起） |
| PCLK | `P02_1` | 像素时钟，ERU_CH2_REQ14 |
| VSYNC | `P02_0` | 场中断，ERU_CH3_REQ6 |
| SCL / SDA | `P02_3` / `P02_2` | SCCB 配置（失败时退化到 UART1） |
| VCC / GND | 3.3 V / GND | — |

引脚定义全部集中在 `libraries/zf_device/zf_device_tft180.h` 与 `zf_device_mt9v03x.h`，
换板子时**只改这两处宏**即可。

## 5. 程序流程

```
core0_main()                       user/cpu0_main.c
   ├─ clock_init() / debug_init()
   ├─ SATA_MAIN()                  code/SATA.c       → Camera_LCD_Init()
   │                                                   ├─ tft180_init()
   │                                                   └─ mt9v03x_init()
   ├─ cpu_wait_event_ready()                          （CPU1 同步点）
   └─ while(1) SATA_Loop()
          └─ if(mt9v03x_finish_flag)                 （DMA 完成一帧）
                 ├─ Camera_SET_Mode(mt9v03x_image)   → OTSU 阈值 1~254
                 ├─ Camera_SET_Mode_RETURN(th)       → base_image[H][W] = 0/255
                 ├─ Camera_LCD_Dis_Show()            → tft180_show_gray_image(...,128)
                 └─ 屏幕下方显示当前阈值 TH xxx
```

### 核心接口

| 函数 | 作用 |
|---|---|
| `Camera_LCD_Init(void)` | TFT180 + 摄像头初始化，失败时屏幕提示 `CAMERA ERR` |
| `Camera_SET_Mode(image, w, h)` | 大津法求阈值，返回 1~254（退化输入返回 128，**不会返回 0**） |
| `Camera_SET_Mode_RETURN(th)` | `memcpy` 一帧到 `base_image` 并按阈值二值化成 0(黑)/255(白) |
| `Camera_LCD_Dis_Show(void)` | 显示二值化图像（驱动内按 `threshold=128` 画黑/白） |
| `Camera_LCD_Dis_Raw(void)` | 显示原始灰度图，用于区分「摄像头/屏幕问题」与「二值化问题」 |
| `Camera_Process_Once(void)` | 一帧完整流程：求阈值 → 二值化 → 显示 |
| `SATA_MAIN()` / `SATA_Loop()` | 初始化 / 主循环单帧处理 |

### 显示参数（`code/Camera_REMAIN.h`）

```c
#define CAMERA_DIS_W      (126)   // 屏幕显示宽度，<= tft180_width_max(128，竖屏)
#define CAMERA_DIS_H      (80)    // 屏幕显示高度，<= tft180_height_max(160，竖屏)
#define CAMERA_DIS_INFO   (1)     // 1-图像下方显示当前 OTSU 阈值
```

## 6. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| 屏幕**一片白** | 二值化阈值为 0，`像素 <= 0` 永不成立 → 全部输出 255 | 阈值必须用**真实图像**计算；`Camera_SET_Mode` 已做退化保护 |
| 屏幕**一片黑** | 阈值 255 / 镜头被遮挡 / 曝光过低 | 看屏幕上的 `TH` 数值是否合理（室外一般 50~140） |
| 图像右侧有竖直灰度条 | 二值化循环行列写反（`base_image[i][j]` 用 W 当行） | 行循环用 `MT9V03X_H`，列循环用 `MT9V03X_W` |
| 屏幕显示 `CAMERA ERR` | 摄像头 SCCB/UART 配置失败 | 检查排线、`P00_0~P00_7` 是否连续、供电 |
| 程序停住不刷新 | 初始化函数里写了 `while(1)`，`cpu_wait_event_ready()` 执行不到 | 初始化与主循环分离（见 `SATA.c`） |

## 7. 版本记录

| 日期 | 版本 | 内容 |
|---|---|---|
| 2026-10-18 | v0.0 | 逐飞 TC264 开源库空工程 |
| 2026-10-20 | v0.1 | 接入 MT9V03X + TFT180，OTSU 二值化并实时显示；修复阈值恒 0 导致白屏、二值化行列越界写 12.7 KB、初始化 `while(1)` 阻塞 CPU1 三个问题 |

## AI 生成代码声明

> 依据透明开源原则，特此声明：**本仓库中以下代码由 AI（大语言模型）辅助生成或修改**，
> 已由作者在真实硬件上编译、烧录与实测确认。

| 文件 | AI 参与的代码 | 说明 |
|---|---|---|
| `code/Camera_REMAIN.c` | `Camera_SET_Mode()` 大津法实现、`Camera_SET_Mode_RETURN()` 二值化循环、`Camera_LCD_Dis_Show()` / `Camera_LCD_Dis_Raw()` / `Camera_Process_Once()` | 阈值退化保护、行列顺序修正、直方图移入 `static` 均由 AI 给出 |
| `code/Camera_REMAIN.h` | 接口原型、`CAMERA_DIS_W/H`、`CAMERA_DIS_INFO` 宏 | — |
| `code/SATA.c` / `SATA.h` | `SATA_MAIN()` 拆分、`SATA_Loop()` 帧同步（`mt9v03x_finish_flag`） | 原实现把 `while(1)` 写在初始化流程里 |
| `user/cpu0_main.c` | `core0_main()` 主循环调用 `SATA_Loop()` | 仅调整调用顺序，未改库文件 |

**由作者人工完成**：硬件选型与接线、摄像头/屏幕外设验证、整车机械与电气、
阈值整定与赛道实测、代码在 ADS 中的编译烧录与最终验收。

**未经 AI 修改**：`libraries/` 下全部逐飞开源库与 Infineon SDK 文件保持原样，可直接与官方版本比对。

## 许可证

`libraries/` 目录为**成都逐飞科技有限公司**的 TC264 开源库，遵循 **GPL 3.0**（见 `libraries/doc/`）。
用户代码（`code/`、`user/`）在上述许可框架下随仓库一同发布。
