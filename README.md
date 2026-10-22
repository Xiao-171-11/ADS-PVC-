# SATA-01 · TC264 智能车完整程序（摄像头循迹 + 差速 + 速度闭环 + 舵机闭环）

按 B 站合集 **「智能车入门基础教学」（UP：WwuSama）第 13～25 集** 的流程写成的**可直接烧录运行**的完整程序。
**实车验证：已能稳定跑完整条赛道。**

- 引脚 ↔ 功能对照：见 [第 3 章](#3-引脚表引脚--功能)
- 哪些代码由 AI 生成：见 [第 7 章](#7-ai-生成代码声明)

- 主控：Infineon **TC264D**，开发环境 **AURIX Development Studio**
- 硬件：总钻风 MT9V03X（188×120）+ TFT180 + 双电机 + 舵机 + 双编码器
- 运行结构：
  - **主循环（≈50 fps）**：大津法二值化 → 边缘基点 → 边缘基线 → 加权中线（含互补滤波）
  - **5 ms 中断（200 Hz）**：编码器测速 → 速度环 PID → 差速 → 左右电机 PWM；中线偏差 → 舵机 PD → 舵机 PWM
  - 两者通过 `midline_info.error` 交换数据

---

## 1. 编译与烧录

1. 打开 **AURIX Development Studio** → `File ▸ Switch Workspace ▸ Other…` → 选 **`D:\ADS_workspace_01`**。
2. `File ▸ Import ▸ General ▸ Existing Projects into Workspace` → `Select root directory` 选 `D:\ADS_workspace_01` → 勾选 **`Seekfree_TC264_Opensource_Library`** → Finish。
3. `Project ▸ Build Project`（Ctrl+B），配置选 **Debug**。
4. 生成固件：`Seekfree_TC264_Opensource_Library/Debug/Seekfree_TC264_Opensource_Library.hex`。
5. 用 **AURIX Flasher** 或逐飞下载器烧录。

> 首次导入后如果工程里有红叉，先 `Project ▸ Clean…` 再 Build；新增文件没进编译就按 **F5** 刷新工程。
> 工程编码是 **GBK**（`.settings/org.eclipse.core.resources.prefs`），不要改。

## 2. 目录与职责

```
Seekfree_TC264_Opensource_Library/
├── code/
│   ├── SATA_MAIN.c/.h      ★ 主程序：初始化 + 主循环 + 单帧图像处理
│   ├── Camera_REMAIN.c/.h   采集 / 大津法阈值 / 二值化 / TFT180 显示
│   ├── Badpoint_REMAIN.c/.h 边缘基点
│   ├── Bedline_REMAIN.c/.h  边缘基线（向上跟踪 + 最小二乘拟合）
│   ├── MIDLine_REMAIN.c/.h  加权中线 + 互补滤波
│   ├── Encoder_REMAIN.c/.h  ★ 编码器测速
│   ├── Servo_REMAIN.c/.h    ★ 舵机（比例 / PD 打角）
│   ├── Motor_REMAIN.c/.h    ★ 电机（PWM + 方向）
│   ├── Control_REMAIN.c/.h  ★ 控制总成（速度环 PID + 差速 + 舵机环），5ms 中断里跑
│   ├── PIT_REMAIN.c/.h      ★ 5ms 控制节拍
│   ├── LCD_REMAIN.c/.h  LED_REMAIN.c/.h   辅助
│   └── 其他 *REMAIN           预留模块
├── user/
│   ├── cpu0_main.c          只做 clock_init + debug_init + SATA_MAIN()
│   ├── isr.c                cc60_pit_ch0_isr 里调用 Control_Tick()
│   └── isr_config.h         中断优先级
└── libraries/               逐飞 TC264 开源库（未改动）
```

## 3. 引脚表（引脚 ↔ 功能）

> 所有引脚都写成 `code/*_REMAIN.h` 里的宏，**改接线只需要改宏**，逻辑代码一行都不用动。

### 3.1 执行器

| 引脚 | 对应部分 | 宏 / 文件 | 备注 |
|---|---|---|---|
| `P33_9` | **舵机** PWM（ATOM0_CH1，50 Hz） | `SERVO_PWM_PIN` · `code/Servo_REMAIN.h` | 中值 465；偏大向左、偏小向右 |
| `P02_4` | **左后电机** PWM（ATOM0_CH4，5 kHz） | `Motor_L_pwm` · `code/Motor_REMAIN.h` | 占空比 0~10000 |
| `P02_5` | 左后电机 **方向 DIR** | `Motor_L_turn` · 同上 | 方向反了改 `MOTOR_DIR_REVERSE_L` |
| `P02_6` | **右后电机** PWM（ATOM0_CH6，5 kHz） | `Motor_R_pwm` · 同上 | |
| `P02_7` | 右后电机 **方向 DIR** | `Motor_R_turn` · 同上 | 方向反了改 `MOTOR_DIR_REVERSE_R` |

### 3.2 编码器

| 引脚 | 对应部分 | 宏 / 文件 |
|---|---|---|
| `P33_7` | **左后编码器** A 相（TIM2 计数） | `ENCODER_L_CH1` · `code/Encoder_REMAIN.h` |
| `P33_6` | **左后编码器** B 相（TIM2 方向） | `ENCODER_L_CH2` · 同上 |
| `P02_8` | **右后编码器** A 相（TIM4 计数） | `ENCODER_R_CH1` · 同上 |
| `P00_9` | **右后编码器** B 相（TIM4 方向） | `ENCODER_R_CH2` · 同上 |

### 3.3 摄像头 / 屏幕 / 指示灯

| 引脚 | 对应部分 | 说明 |
|---|---|---|
| `P02_0` | 总钻风 **VSYNC** | 逐飞库 `zf_device_mt9v03x.c` 内固定，不要在别处复用 |
| `P02_1` | 总钻风 **PCLK** | 同上 |
| `P02_2` / `P02_3` | 总钻风 **SCCB SCL / SDA** | 同上 |
| `P00_0 ~ P00_7` | 总钻风 **8 位数据口 D0~D7** | 同上 |
| `P15_3` / `P15_5` | TFT180 **SCL / SDA(MOSI)** | 硬件 SPI2 |
| `P15_1` / `P15_0` | TFT180 **RES / DC** | |
| `P15_2` / `P15_4` | TFT180 **CS / BL(背光)** | |
| `P21_5` `P21_4` `P20_8` `P20_9` | **指示灯 LED** | `code/LED_REMAIN.c`，本程序未使用 |

### 3.4 引脚占用一览（改接线时避开这些）

```
P00_0~P00_7   摄像头数据口 D0~D7        P02_0~P02_3   摄像头 VSYNC / PCLK / SCCB
P02_4~P02_7   左/右后电机 PWM + DIR     P02_8         右编码器 A
P00_9         右编码器 B                P33_6 / P33_7 左编码器 B / A
P33_9         舵机                      P15_0~P15_5   TFT180
P21_4 P21_5 P20_8 P20_9                指示灯 LED
```

> 编码器这四根选的是**没被摄像头、电机、屏幕、LED 占用**的空闲脚。实物接线不同的话，改 `Encoder_REMAIN.h` 里 4 行宏即可，
> 可选组合见 `libraries/zf_driver/zf_driver_encoder.h`（TIM2/TIM3/TIM4 支持正交，全部支持方向模式）。

## 4. 上车前必须做的 6 项标定

> 代码能跑，但**参数是通用初值**，按顺序做下面 6 步，车才会跑得稳。

| # | 标定项 | 怎么做 | 改哪里 |
|---|---|---|---|
| 1 | **方向** | 架空车，手动给正占空比看轮子转向；正转但读到的速度是负 → 改 `MOTOR_DIR_REVERSE_x` / `ENCODER_x_REVERSE` | `Motor_REMAIN.h`、`Encoder_REMAIN.h` |
| 2 | **舵机中值与方向** | 上电 3 秒内舵机应回中；给 `Servo_Set_Turn(20)` 看是否向右打角，反了改 `SERVO_REVERSE` | `Servo_REMAIN.h` |
| 3 | **基础速度** | 架空，观察 `encoder` 速度值；把 `CONTROL_BASE_SPEED` 调到车轮转起来合适、`CONTROL_DUTY_BASE` 调到刚好能维持这个速度（前馈） | `Control_REMAIN.h` |
| 4 | **后轮差速方向** | 架空，临时把 `error` 赋成 +50，看**后左轮**是否比**后右轮**快；反了改 `CONTROL_DIFF_REVERSE` | `Control_REMAIN.h` |
| 5 | **舵机比例** | 先 `SERVO_KD = 0`，把车放在赛道中间，逐步加大 `SERVO_KP` 直到能跟住弯道又不抖 | `Servo_REMAIN.h` |
| 6 | **速度环 PID** | 先 `KI=KD=0` 加 `KP` 到速度能跟上目标，再少量加 `KI` 消除静差 | `Control_REMAIN.h` |

## 5. 参数一览（`Control_REMAIN.h`）

| 宏 | 默认 | 含义 |
|---|---|---|
| `CONTROL_PERIOD_MS` | 5 | 控制周期，与 `PIT_Init()` 一致 |
| `CONTROL_START_DELAY_MS` | 3000 | 上电延时，期间电机 0、舵机回中 |
| `CONTROL_BASE_SPEED` | **10** | 基础速度（编码器脉冲/周期），**闭环时车速由它决定** |
| `CONTROL_SPEED_MAX` | **18** | 差速后的速度上限 |
| `CONTROL_SPEED_MIN` | **3** | 内侧轮最低速度，防止差速后停转 |
| `CONTROL_DUTY_BASE` | **600** | 维持基础速度的前馈占空比（6%），**开环时车速由它决定** |
| `CONTROL_PID_KP/KI/KD` | **20 / 0.5 / 0** | 速度环参数 |
| `CONTROL_PID_OUT_MAX` | **500** | 速度环修正量限幅 |
| `CONTROL_SPEED_LOOP_EN` | **1** | 1-速度闭环；0-只按前馈开环跑（编码器没接好时用） |
| `CONTROL_DIFF_K` | 0.0040 | 偏差 1 像素对应的**速度比例**（0.4%） |
| `CONTROL_DIFF_K_MAX` | 0.50 | 最大差速比例（内侧轮最多降 50%） |
| `CONTROL_DIFF_REVERSE` | 0 | 差速方向反了改 1 |
| `CONTROL_CURVE_SLOW_K` | **0.0040** | 弯道减速：偏差 1 像素降多少比例速度 |
| `CONTROL_CURVE_SLOW_MIN` | **0.60** | 最多降到基础速度的 60% |
| `CONTROL_LOST_STOP_TICKS` | **0** | 连续丢线后停车（0 = 不停车，弯道短暂丢线不会卡死） |

### 舵机打角参数（`Servo_REMAIN.h`）

分段增益：直道用小增益（不摆），弯道用大增益（打得动）。

| 宏 | 默认 | 含义 |
|---|---|---|
| `Servo_MID` / `SERVO_TRIM` | 465 / 0 | 中值 + 中值微调（车直道往哪边偏就往反方向调几个数） |
| `SERVO_DUTY_RANGE` | **180** | 最大打角偏移（465±180 ≈ 0.57~1.29 ms） |
| `SERVO_DEAD_ZONE` | **2.0** | 死区：偏差 < 2 像素不动作，治直道抖动 |
| `SERVO_KNEE` | **15.0** | 拐点：15 像素以内小增益，以外大增益 |
| `SERVO_KP_NEAR` | **1.0** | 小偏差增益（原来统一 3.0 太猛） |
| `SERVO_KP_FAR` | **2.5** | 大偏差增益 |
| `SERVO_KD` | **2.0** | 微分阻尼，压住来回摆（0 = 关） |
| `SERVO_REVERSE` | 0 | 左右打角反了改 1 |

打角曲线实测（error → 占空比 → 占满比例）：

```
error    0    5   15   30   50   80   94
占空比  465  460  450  412  362  288  285
占满      0%   3%   8%  29%  57%  99% 100%
旧版(统一KP=3)  0%  10%  30%  60% 100% 100% 100%
```

直道小偏差异常有 5~15 像素，旧版一上来就打 10~30% 的量 → 车左右摆；新版只给 3~8%。
弯道 50 像素以上，旧版早早就打满（150/150），新版把量程放到 180 并在 80 像素附近才打满，弯里有更多余量。

> **改车速改哪个**：`CONTROL_SPEED_LOOP_EN = 1`（闭环）时改 `CONTROL_BASE_SPEED`，PID 会把实际速度拉到目标；
> `= 0`（开环）时改 `CONTROL_DUTY_BASE`。两个一起按同一比例改最稳（前馈和目标匹配，PID 不用使劲）。
> **当前这一版 = 上一轮再降 50%**：20→10、1200→600、35→18、1000→500、5→3。

### 屏幕上能看到什么

| 位置 | 内容 |
|---|---|
| 图像左上角（黄色 6x8 小字） | `V### R###` = 后左轮 / 后右轮**实测速度**（脉冲/5ms），车在动而一直 000 = 编码器没通 |
| 图像下方第 1 行 | `TH ###` 大津法阈值 |
| 第 2 行 | `BP ### ###` 左 / 右基点列 |
| 第 3 行 | `K ±#.## ±#.##` 左 / 右基线斜率 |
| 第 4 行 | `C ###  E +#.#` 互补滤波后的中线值 / 偏差 |

### 后轮差速模型

车是「**舵机转前轮 + 双电机驱动后轮**」，所以差速只作用在**后左 / 后右**两个驱动轮之间：

```
ratio  = CONTROL_DIFF_K × error           // 偏差 -> 速度比例，±CONTROL_DIFF_K_MAX 限幅
v_后左 = 基础速度 × (1 + ratio)            // 外侧
v_后右 = 基础速度 × (1 - ratio)            // 内侧
```

方向：`error > 0` 表示赛道中心在图像右侧 -> 车要**右转** -> 后左轮是**外侧**（走的路更长）-> 左快右慢。
用「比例」而不是「固定差值」：真实的内外轮比由转弯半径决定，`v_外 / v_内 = (R + W/2) / (R − W/2)`，
车速一提，同样的打角就需要更大的轮速差，比例模型自己就跟着放大了，不用重调参数。

| error | ratio | 后左 / 后右（基础速度 10，当前设置） |
|---|---|---|
| −94（赛道偏左） | −0.376 | 6.2 / 13.8 |
| −30 | −0.120 | 8.8 / 11.2 |
| 0 | 0 | 10.0 / 10.0 |
| +30 | +0.120 | 11.2 / 8.8 |
| +94（赛道偏右） | +0.376 | 13.8 / 6.2 |

**两个必须注意的点**：

1. **机械差速**：后桥如果还装着机械差速器，它本身已经按转弯半径把速度分给两轮，电子差速是**叠加**在它上面的，实际轮速差会偏大、车会往弯里扎。处理办法二选一：把 `CONTROL_DIFF_K` 减小 30%~50%，或者干脆锁死机械差速（直轴）只用电子的。
2. **外侧轮被限速**：`CONTROL_SPEED_MAX` 必须 ≥ `CONTROL_BASE_SPEED × (1 + CONTROL_DIFF_K_MAX)`。当前 10 × 1.5 = 15 < 18，有余量；以后提速时这两个数要一起改，否则外侧轮被削平、车往弯外推。

## 6. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| **直道上左右摆 / 转向过度** | 小偏差增益太大、没有阻尼 | 减小 `SERVO_KP_NEAR`（1.0→0.6）、加大 `SERVO_KD`（2.0→3.0）、必要时把 `SERVO_DEAD_ZONE` 加到 3~4 |
| **过弯时卡住 / 停在弯里** | 弯道丢线 1 秒触发停车保护；或弯里速度太高打不动方向 | `CONTROL_LOST_STOP_TICKS` 已改 **0**（不再停车）；再不行把 `CONTROL_CURVE_SLOW_MIN` 从 0.6 降到 0.5，让弯道更慢 |
| **原地转圈 / 两个后轮转向相反** | 左右电机方向脚极性不一致（实测：右后轮正转、左后轮倒转） | 把转反的那一侧 `MOTOR_DIR_REVERSE_x` 改成 1，架空复测两轮同向 |
| 车跑起来了但屏幕左上角 `V` 一直是 `000` | 编码器没接对 / 定时器选错 / `ENCODER_x_REVERSE` 极性 | 先把 `CONTROL_SPEED_LOOP_EN` 改 0 用开环跑，再逐路查编码器 |
| 上电 3 秒后车不动 | 没有图像 / `midline_info.valid` 一直是 0 | 看屏幕：有 `TH` 数值说明图像在跑；`CAMERA ERR` 说明摄像头没配好 |
| 车往反方向打角 | 舵机方向 | `SERVO_REVERSE` 改 1 |
| 车原地打转 | 左右电机方向有一个反了 | `MOTOR_DIR_REVERSE_L/R` |
| 速度环一加就抖 | `KP` 太大 / 编码器极性反 | 先确认速度为正值，再减小 `KP` |
| 弯道跟不上 / 甩出去 | `SERVO_KP_FAR` 偏小 / `CONTROL_DIFF_K` 偏小 | 逐步加大，注意不要抖 |
| 弯道内侧轮"拖刹"、车头扎进弯里 | 差速太大或 `CONTROL_SPEED_MIN` 太低 | 减小 `CONTROL_DIFF_K`，或抬高 `CONTROL_SPEED_MIN` |
| 提速后弯道明显不如低速 | `CONTROL_SPEED_MAX` 卡住了外侧轮 | `CONTROL_SPEED_MAX` 要 ≥ `CONTROL_BASE_SPEED × (1 + CONTROL_DIFF_K_MAX)` |
| 屏幕信息行重叠 | `CAMERA_INFO_Y1..Y4` 被改过 | 恢复 84/102/120/138（最后一行不能超过 144） |

## 7. AI 生成代码声明

> 依据透明开源原则，特此说明：**本仓库 `code/` 与 `user/` 下的部分代码由 AI（大语言模型）辅助生成或改写**。
> 全部代码已由作者在 TC264 实车上编译、烧录、标定，并跑完整条赛道验证。
> `libraries/` 是逐飞 TC264 开源库原版（GPL 3.0），**未做任何修改**，可与官方版本逐字节比对。

### 7.1 逐文件说明

| 文件 | 来源 | 说明 |
|---|---|---|
| `code/Camera_REMAIN.c/.h` | **AI 生成** | 大津法阈值、二值化、TFT180 显示；阈值算法与教科书实现做过 408 组对拍验证 |
| `code/Badpoint_REMAIN.c/.h` | **AI 生成** | 边缘基点；文件骨架由作者预先建立，实现由 AI 补全 |
| `code/Bedline_REMAIN.c/.h` | **AI 生成** | 边线跟踪 + 最小二乘拟合基线 |
| `code/MIDLine_REMAIN.c/.h` | **AI 生成** | 加权中线 + 互补滤波（按作者提供的参考代码实现） |
| `code/Encoder_REMAIN.c/.h` | **AI 生成** | 双编码器测速 |
| `code/Control_REMAIN.c/.h` | **AI 生成** | 速度环 PID、后轮差速、弯道减速、丢线保护 |
| `code/Servo_REMAIN.c/.h` | **AI 改写** | 分段增益 + 微分 + 死区打角；原作者只写了 `pwm_init` 一行 |
| `code/Motor_REMAIN.c/.h` | **AI 改写** | 原作者已有引脚与基础 set 函数，AI 重构为 `Motor_Set_Duty()` + 方向翻转宏 |
| `code/PIT_REMAIN.c/.h` | **AI 改写** | 改为 5 ms 控制节拍 |
| `code/SATA_MAIN.c` | **AI 生成** | 主程序：外设初始化顺序 + 主循环 + 单帧处理编排 |
| `user/cpu0_main.c` | **AI 精简** | 只保留 `clock_init` + `debug_init` + `SATA_MAIN()` |
| `user/isr.c` | **AI 加一行** | `cc60_pit_ch0_isr` 中调用 `Control_Tick()` |
| `code/LED_REMAIN.c/.h`、`code/LCD_REMAIN.c/.h`，以及 `Motor_REMAIN.h` 的引脚宏、`Servo_REMAIN.h` 的舵机中值 | **作者原创** | 与 AI 无关 |
| `libraries/**` | 逐飞 TC264 开源库 | GPL 3.0，原样未改 |

### 7.2 作者亲自完成的部分

- 硬件选型与接线，电机 / 舵机 / 编码器的机械安装
- 引脚分配（`P02_4~P02_7` 电机、`P33_9` 舵机、编码器四线、`P15_x` 屏幕）
- 实车标定：电机与编码器方向、舵机中值与方向、基础速度、后轮差速、PID 与打角参数
- 多轮上车调试（原地转圈、直道左右摆、弯道卡住等现象的定位与参数调整），最终跑完全程
