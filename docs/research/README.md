# 开源智能车项目研读报告（室内 PVC 赛道 · TC264 + MT9V03X）

本目录是围绕 `D:\ADS_Workspace_spakade`（Sata 工程）做的开源资料调研与改造设计。
**结论可用，代码未改动**——所有文档都是"读别人的代码得出的、针对本工程的具体改法"，不含对本工程的任何修改。

---

## 文档索引

| 文档 | 内容 | 什么时候看 |
|:--|:--|:--|
| [`00-本工程基线剖析.md`](00-本工程基线剖析.md) | 本工程现状：硬件表 / 图像链 / 控制链 / 参数一览 / **12 条缺陷**（全部带 `文件:行号`） | 任何时候先看这个，建立基准 |
| [`01-开源仓库地图.md`](01-开源仓库地图.md) | 12 个参考条目（11 个车项目 + 1 个上位机工具仓）的档案卡 + 6 张横向对比矩阵 + **空白清单** + 选型结论 | 想知道"别人怎么做的""该抄谁" |
| [`02-关键算法剖析.md`](02-关键算法剖析.md) | 9 章算法对照：PID 三形态 / 四象限限幅 / 速度斜坡 / 方向环 / 前瞻加权 / 边线提取与补线 / 元素判据 / 双核分工 / **按投入产出比排序的 9 条** | 想理解"为什么这么改" |
| [`03-改造清单.md`](03-改造清单.md) | **11 项改造**（A1/A2/A3/B1/B2/B3/B4/C1/C2/D1/D2，P0~P3），每项含 **现状行号 → 目标 → 可直接编译的代码 → 验证方法 → 风险** | 准备动手改代码 |

**最短路径**：`00`（知道自己在哪）→ `03` 的"总览"表 + A1（知道下一步做什么）→ 需要理由时回查 `02`。

---

## 核心结论（五句话）

1. **本工程的最大缺陷不是缺元素识别，是"对整条边线做一次最小二乘直线拟合"**。透视投影下弯道本来就是曲线，直线拟合的残差随曲率单调增大——这不是调参能救的。所以 P0 是接逆透视，不是补环岛。
2. **逆透视在 MCU 上不是矩阵运算，是查表**。`RUBO-UDIPM` 与 `losehu/smartcar-C` 两个互不相关的作者给出了结构完全一致的实现：初始化时算一次 100×114 的 `uint8_t*` 指针表（≈45.6 KB），越界像素统一指向一个白色常量，运行时只做一次解引用。`RUBO-UDIPM` 附带的 3 份下位机模板可以**直接编译**。
3. **同平台参考首选 `TC264-ZongZuanFeng-Camera-Car`**（TC264D + 逐飞库 + MT9V03X 188×120，几乎逐引脚对应），它的中断分配表、指针表零拷贝降采样、CPU0 感知 / CPU1 控制双核切分可以直接搬。
4. **性价比最高的一项不是算法，是接陀螺仪**：`user/isr.c:87-95` 的中断和 `libraries\zf_device\zf_device_imu660rc.c:392-396` 的回调**都已经写好了**，全工程只是从来没调用 `imu660rc_init()`。加一行初始化 + 一个阻尼项，就能消掉图像 50 fps 带来的方向抖动。
5. **元素判据抄"结构"不抄"代码"**：`17th-SmartCar`/`HammerTa` 的四层结构（路线先验 → 标志互斥门 → 几何否决 → 前摇分诊）比一堆 `if` 可维护，但里面的阈值全是别人车模的数字。
---

## 调研规模

| 项 | 数量 |
|:--|:--|
| 下载的参考仓库 | **12 个**（11 个车项目 + 1 个上位机工具仓，约 15,000 源文件 / 490 MB） |
| 逐行研读并写成笔记的仓库 | 11 个（笔记在 `.research_refs\_notes\`，合计 **7,751 行 / 464 KB**） |
| 可直接移植的下位机代码模板 | 3 份（`RUBO-UDIPM` 的去畸变 / 透视 / 去畸变+逆透视） |


### 参考仓库清单

| 仓库 | 平台 | 定位 | 笔记 |
|:--|:--|:--|:--|
| USTB_Swan | RT1064 | 控制层教科书 | `_notes/USTB_Swan.md` |
| TC264-ZongZuanFeng-Camera-Car | **TC264D** | 同平台首选 | `_notes/TC264-ZongZuanFeng.md` |
| 17th-SmartCar | TC264 | 判据工程范本 | `_notes/RT1064_17th.md` |
| HammerTa-17thTC264 | TC264 | 唯一开箱即用的总钻风四轮 | `_notes/HammerTa-17thTC264.md` |
| losehu/smartcar-C | CH32V307 | 48 个快照的元素状态机演进 | `_notes/losehu-smartcar-C.md` |
| Smart-car (wuyou33) | TC264D | "补线画回二值图再重爬" | `_notes/wuyou33-Smart-car.md` |
| SmartCarBaseCameraByXiayu | CH32V307 | 八邻域唯一完整实现 | `_notes/xiayu-8neighbor.md` |
| RT1064_Smartcar | RT1064 | 分层 + LADRC 1kHz + 遥测协议 | `_notes/RT1064_17th.md` |
| smartcar2022 | RT1064 | Flash 参数块 + GUI | `_notes/smartcar2022.md` |
| SmartCar (MaahirG) | Jetson | 架构读物，不可迁移 | `_notes/MaahirG-SmartCar.md` |
| Awesome-IntelligentCarRace | — | 索引（`table.csv` 仅 16 条） | `_notes/Awesome-index.md` |
| RUBO-UDIPM | — | **上位机 + 3 份可直接编译的逆透视模板** | 无（直接读模板） |

---

## 目录结构

```
D:\ADS_Workspace_spakade\
├─ docs\research\            ← 本目录（4 篇报告 + 本 README）
├─ .research_refs\           ← 参考仓库（已在 .git/info/exclude 中排除，不进 git status）
│   ├─ <仓库名>\             ← 12 个解压后的仓库
│   ├─ _zips\                ← 原始 zip（可重解压）
│   ├─ _notes\               ← 子代理研读笔记（含全部行号）
│   └─ _utf8\                ← GBK→UTF-8 镜像（便于 grep）
└─ Seekfree_TC264_Opensource_Library\
    ├─ code\                 ← 本工程算法模块（SATA_MAIN / Camera / Bedline / MIDLine / Zebra / Control / Servo / Motor / Encoder / PIT / LCD / Badpoint，全部 *_REMAIN.c/h）
    ├─ user\                 ← cpu0_main.c / cpu1_main.c（CPU1 空转）/ isr.c / isr_config.h
    └─ libraries\            ← 逐飞库
```

---

## 两个环境坑（复现调研时需要）

**1. 本工程与多数参考仓库的 C/H 是 GBK(936) 编码**，`read` 类工具会报 `invalid UTF-8 text`。用：

```powershell
$enc=[System.Text.Encoding]::GetEncoding(936)
$enc.GetString([System.IO.File]::ReadAllBytes("路径"))
```

注意：`USTB_Swan` 是 UTF-8（少数 GBK）；`smartcar2022` 的 CRLF 与孤立 CR 混排，行号必须用 `[System.IO.File]::ReadAllLines()` 取。

**2. GitHub 直连与 git 代理都不稳定**，`gh-proxy.com` 前缀最可靠：

```powershell
curl.exe -sL --max-time 120 "https://gh-proxy.com/https://github.com/OWNER/REPO/archive/refs/heads/main.zip" -o REPO.zip
Expand-Archive -Path REPO.zip -DestinationPath .research_refs\REPO -Force
```

（`main` / `master` 两个分支名都要试；`cdn.jsdelivr.net` / `ghproxy.net` / `kkgithub.com` 在本机不通。）
