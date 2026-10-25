/*
 * Pursuit_REMAIN.c
 *
 *  纯追踪 + 逆透视巡线
 *
 *  处理流程（全部在主循环里跑，帧率 50fps）：
 *      1. IPM_Process() 已经把 base_image 重采样成俯视图 ipm_image[56][96]
 *      2. pursuit_extract_track()：从最近行往上，逐行向左右找边线，得到每行中线
 *      3. 十字状态机：用"左右同时丢"的行数区分路口和弯道，路口保持原航向
 *      4. 纯追踪：在车前方 Ld 处取前瞻点，算曲率 -> 前轮转角 -> 舵机偏移
 *      5. 发布快照（双缓冲），5ms 控制中断里 Pursuit_Get() 取用
 *
 *  Created on: 2026年10月28日
 *      Author: Freed
 */

#include "Pursuit_REMAIN.h"

#define PURSUIT_CENTER_COL  (((float)(IPM_W - 1)) * 0.5f)   /* 96 列 -> 47.5 */
#define PURSUIT_RAD2DEG     (57.2957795f)

/* ---- 每行中线与可信度（只在本文件内用） ---- */
static float trk_center[IPM_H];
static uint8 trk_ok[IPM_H];
static uint8 trk_lost_l[IPM_H];
static uint8 trk_lost_r[IPM_H];
static uint8 trk_valid;
static uint8 trk_open;
static uint8 trk_one;

/* ---- 十字路口状态机 ---- */
static uint8 cross_state;                                   /* 0=不在路口 1=正在过路口 */
static uint8 cross_hits;                                    /* 连续多少帧可疑 */
static uint8 cross_ticks;                                   /* 路口内剩余帧数 */
static uint8 cross_clean;                                   /* 连续多少帧干净 */
static uint8 cross_reject;                                  /* 出路口后的禁止判定帧数 */
static float cross_kappa;                                   /* 进路口那一刻的曲率 */

/* ---- 视野丢失保持 ---- */
static uint8 pursuit_hold;                                  /* 已经沿用旧快照多少帧 */

/* ---- 双缓冲发布 ---- */
static pursuit_struct ps_buf[2];
static volatile uint8 ps_pub;

pursuit_struct pursuit_info;
int16 pursuit_center_row[IPM_H];

/* ============================================================================
 *  从 ipred 沿 dir 方向找一条边线的游程中点
 *    dir = -1 向左、+1 向右
 *    *thick 返回游程宽度（格点）
 *    返回中点列号（float，半格精度），找不到返回 -1
 *
 *  注意：单格黑就算命中。俯视图近处的边线只有 1 格宽（源图横向 1 像素
 *        代表 8~12mm，而格点只有 6.7mm），如果要求"连续两个黑点"，
 *        最近 20 多行会被整片判成丢线——这是踩过的坑。
 * ==========================================================================*/
static float pursuit_find_run(uint8 row, int16 ipred, int8 dir, uint8 *thick)
{
    int16 x;
    int16 a;
    int16 b;
    int16 step;
    int16 lim;
    int16 dist;

    *thick = 0;

    if(ipred < 0)
    {
        ipred = 0;
    }
    if(ipred > (IPM_W - 1))
    {
        ipred = (IPM_W - 1);
    }

    step = (int16)dir;
    lim  = (int16)(PURSUIT_SEARCH_COLS + PURSUIT_LINE_W_MAX);
    x    = ipred + step;

    while((x >= 0) && (x < IPM_W) && (((x - ipred) * step) <= PURSUIT_SEARCH_COLS))
    {
        if(ipm_image[row][x] < 128)
        {
            a = x;                                          /* 向 -1 方向扩展起点 */
            while((a - 1) >= 0)
            {
                dist = ((a - 1) > ipred) ? ((a - 1) - ipred) : (ipred - (a - 1));
                if((dist > lim) || (ipm_image[row][a - 1] >= 128))
                {
                    break;
                }
                a --;
            }

            b = x;                                          /* 向 +1 方向扩展终点 */
            while((b + 1) < IPM_W)
            {
                dist = ((b + 1) > ipred) ? ((b + 1) - ipred) : (ipred - (b + 1));
                if((dist > lim) || (ipm_image[row][b + 1] >= 128))
                {
                    break;
                }
                b ++;
            }

            *thick = (uint8)((b - a) + 1);

            return (((float)(a + b)) * 0.5f);
        }
        x += step;
    }

    return (-1.0f);
}

/* ============================================================================
 *  逐行提取赛道中线
 *    有左右两侧   -> 取中点，可信
 *    只有一侧     -> 用学习到的半赛道宽推算，且必须和上一行的预测值吻合
 *    两侧都没有   -> 直接用上一行预测值外推（十字横臂、全黑区域、赛道外）
 * ==========================================================================*/
static void pursuit_extract_track(void)
{
    float pc;
    float slope;
    float hw;
    float pred;
    float lp;
    float rp;
    float w;
    float c2;
    int16 ip;
    uint8 tl;
    uint8 tr;
    uint8 lost_l;
    uint8 lost_r;
    uint8 r;

    pc    = PURSUIT_CENTER_COL;
    slope = 0.0f;
    hw    = (PURSUIT_TRACK_W_MIN * 1.6f);

    trk_valid = 0;
    trk_open  = 0;
    trk_one   = 0;

    for(r = 0; r < IPM_H; r ++)
    {
        pred = pc + slope;
        ip   = (int16)pred;

        tl   = 0;
        tr   = 0;
        lp   = pursuit_find_run(r, ip, -1, &tl);
        rp   = pursuit_find_run(r, ip,  1, &tr);

        if((lp < 0.0f) || (tl > PURSUIT_LINE_W_MAX))        /* 太宽的是横臂，不是边线 */
        {
            lp = -1.0f;
        }
        if((rp < 0.0f) || (tr > PURSUIT_LINE_W_MAX))
        {
            rp = -1.0f;
        }

        lost_l = 0;
        lost_r = 0;

        if((lp >= 0.0f) && (rp >= 0.0f))
        {
            w = rp - lp;
            if((w >= (float)PURSUIT_TRACK_W_MIN) &&
               (w <= (float)PURSUIT_TRACK_W_MAX) &&
               (fabsf(w - (2.0f * hw)) <= (PURSUIT_WIDTH_TOL * 2.0f * hw)))
            {
                trk_center[r] = (lp + rp) * 0.5f;
                trk_ok[r]     = 1;
                trk_lost_l[r] = 0;
                trk_lost_r[r] = 0;
                trk_valid ++;

                hw = (0.85f * hw) + (0.15f * (w * 0.5f));   /* 缓慢学习赛道半宽 */

                if(r > 0)
                {
                    slope = (0.5f * slope) + (0.5f * (trk_center[r] - pc));
                }
                pc = trk_center[r];
                continue;
            }
        }

        if((lp >= 0.0f) && (rp < 0.0f))                     /* 只看到左边 */
        {
            c2 = lp + hw;
            if(fabsf(c2 - pred) <= PURSUIT_SINGLE_TOL)
            {
                trk_center[r] = c2;
                lost_r        = 1;
            }
            else
            {
                trk_center[r] = pred;
                lost_l        = 1;
                lost_r        = 1;
            }
        }
        else if((rp >= 0.0f) && (lp < 0.0f))                /* 只看到右边 */
        {
            c2 = rp - hw;
            if(fabsf(c2 - pred) <= PURSUIT_SINGLE_TOL)
            {
                trk_center[r] = c2;
                lost_l        = 1;
            }
            else
            {
                trk_center[r] = pred;
                lost_l        = 1;
                lost_r        = 1;
            }
        }
        else                                                /* 两侧都没有 */
        {
            trk_center[r] = pred;
            lost_l        = 1;
            lost_r        = 1;
        }

        trk_ok[r]     = 0;
        trk_lost_l[r] = lost_l;
        trk_lost_r[r] = lost_r;

        if(0 != lost_l)
        {
            if(0 != lost_r)
            {
                trk_open ++;
            }
            else
            {
                trk_one ++;
            }
        }
        else if(0 != lost_r)
        {
            trk_one ++;
        }

        pc = trk_center[r];
    }
}

/* ============================================================================
 *  十字路口状态机
 *    路口特征：横臂把左右两条边线同时切断 -> open 行数很大，one_side 很小
 *    弯道特征：只有外侧那条边线跑出画面 -> one_side 很大，open 很小
 *    进出都用"连续帧数"去抖，出路口后再禁判一段，防止连续路口抖动。
 * ==========================================================================*/
static void pursuit_cross_update(float kappa_prev)
{
    uint8 suspect;

    suspect = 0;

    if(trk_open >= PURSUIT_CROSS_MIN_OPEN)
    {
        suspect = 1;
    }
    if(trk_open <= ((trk_one * 3) / 2))                     /* 单侧丢线占多数 -> 弯道 */
    {
        suspect = 0;
    }
    if(trk_valid < PURSUIT_CROSS_MIN_VALID)                 /* 看得太少的帧不算 */
    {
        suspect = 0;
    }
    if(trk_open > ((IPM_H * 3) / 5))                        /* 大半个画面都没了 -> 摄像头问题 */
    {
        suspect = 0;
    }
    if(fabsf(kappa_prev) > PURSUIT_CROSS_MAX_KAPPA)         /* 进路口前正在大转弯 -> 是弯道 */
    {
        suspect = 0;
    }

    if(cross_reject > 0)
    {
        cross_reject --;
        suspect = 0;
    }

    if(1 == cross_state)
    {
        if(cross_ticks > 0)
        {
            cross_ticks --;
        }
        if(0 != suspect)
        {
            cross_clean = 0;
        }
        else if(cross_clean < 200)
        {
            cross_clean ++;
        }

        if((0 == cross_ticks) || (cross_clean >= PURSUIT_CROSS_CLEAN))
        {
            cross_state  = 0;
            cross_ticks  = 0;
            cross_clean  = 0;
            cross_reject = PURSUIT_CROSS_REJECT;
        }
    }
    else
    {
        if(0 != suspect)
        {
            if(cross_hits < 200)
            {
                cross_hits ++;
            }
        }
        else
        {
            cross_hits = 0;
        }

        if(cross_hits >= 2)
        {
            cross_state = 1;
            cross_ticks = PURSUIT_CROSS_TICKS;
            cross_clean = 0;
            cross_hits  = 0;
            cross_kappa = kappa_prev;                       /* 记住进路口前车头的走向 */
        }
    }
}

/* ============================================================================
 *  主处理：边界提取 -> 十字判定 -> 纯追踪 -> 发布快照
 * ==========================================================================*/
void Pursuit_Process(void)
{
    pursuit_struct snap;
    float kappa_prev;
    float kappa;
    float kappa_use;
    float speed_ref;
    float norm;
    float ld;
    float cc;
    float xt;
    float yt;
    float delta;
    float decay;
    float steer;
    float ratio;
    float slow;
    float near_sum;
    float near_off;
    int16 r0;
    uint8 off_track;
    uint8 r;

    kappa_prev = pursuit_info.kappa;

    memset(&snap, 0, sizeof(snap));
    snap.valid = 0;
    snap.slow  = 1.0f;

    if(0 == ipm_info.ok)                                    /* 标定表没建好，直接交给老算法 */
    {
        ps_buf[ps_pub ^ 1] = snap;
        ps_pub = (uint8)(ps_pub ^ 1);
        pursuit_info = snap;
        return;
    }

    pursuit_extract_track();
    pursuit_cross_update(kappa_prev);

    for(r = 0; r < IPM_H; r ++)
    {
        pursuit_center_row[r] = (int16)trk_center[r];
    }

    /* 看得清的行太少：先沿用上一份快照，最多 PURSUIT_HOLD_FRAMES 帧。
       目的是别让一帧反光/抖动就把车瞬间交给另一套控制器（弯中抖动就是这么来的）。 */
    if(trk_valid < PURSUIT_CROSS_MIN_VALID)
    {
        if(pursuit_hold < PURSUIT_HOLD_FRAMES)
        {
            pursuit_hold ++;
            snap               = ps_buf[ps_pub];
            snap.valid_rows    = trk_valid;
            snap.open_rows     = trk_open;
            snap.one_side_rows = trk_one;
            ps_buf[ps_pub ^ 1] = snap;
            ps_pub             = (uint8)(ps_pub ^ 1);
            pursuit_info       = snap;
            return;
        }
    }
    else
    {
        pursuit_hold = 0;
    }

    /* ---------------- 纯追踪 ---------------- */
    speed_ref = control_base_speed_ref;
    norm      = speed_ref / CONTROL_SPEED_MAX;
    if(norm < PURSUIT_SPEED_NORM_MIN)
    {
        norm = PURSUIT_SPEED_NORM_MIN;
    }
    if(norm > 1.0f)
    {
        norm = 1.0f;
    }

    ld = PURSUIT_LD_MIN_MM + ((PURSUIT_LD_MAX_MM - PURSUIT_LD_MIN_MM) * norm);
    r0 = (int16)(((ld - IPM_Y_NEAR_MM) / ipm_info.dy_mm) + 0.5f);
    if(r0 < 2)
    {
        r0 = 2;
    }
    if(r0 > (IPM_H - 2))
    {
        r0 = (IPM_H - 2);
    }

    cc = (trk_center[r0 - 1] + trk_center[r0] + trk_center[r0 + 1]) / 3.0f;
    xt = (cc - PURSUIT_CENTER_COL) * ipm_info.dx_mm;
    yt = IPM_Y_NEAR_MM + (((float)r0) * ipm_info.dy_mm);
    kappa = (2.0f * xt) / ((xt * xt) + (yt * yt));

    if(1 == cross_state)                                    /* 路口：沿进路口前的航向直行并衰减 */
    {
        decay = ((float)cross_ticks) / ((float)PURSUIT_CROSS_TICKS);
        kappa = cross_kappa * decay;
    }

    delta = atanf(kappa * PURSUIT_WHEELBASE_MM) * PURSUIT_RAD2DEG;
    if(delta > PURSUIT_DELTA_MAX_DEG)
    {
        delta = PURSUIT_DELTA_MAX_DEG;
    }
    if(delta < (-PURSUIT_DELTA_MAX_DEG))
    {
        delta = (-PURSUIT_DELTA_MAX_DEG);
    }

    kappa_use = tanf(delta / PURSUIT_RAD2DEG) / PURSUIT_WHEELBASE_MM;

    /* out 正 = 向左；delta 正 = 右转，所以取负号 */
    steer = (-(delta / PURSUIT_DELTA_MAX_DEG)) * ((float)SERVO_DUTY_RANGE_L);
    if(steer > ((float)SERVO_DUTY_RANGE_L))
    {
        steer = (float)SERVO_DUTY_RANGE_L;
    }
    if(steer < (-(float)SERVO_DUTY_RANGE_R))
    {
        steer = -(float)SERVO_DUTY_RANGE_R;
    }

    ratio = kappa_use * PURSUIT_REAR_TRACK_MM * 0.5f;
    if(ratio > CONTROL_DIFF_K_MAX)
    {
        ratio = CONTROL_DIFF_K_MAX;
    }
    if(ratio < (-CONTROL_DIFF_K_MAX))
    {
        ratio = (-CONTROL_DIFF_K_MAX);
    }

    slow = 1.0f - (PURSUIT_CURV_SLOW_K * fabsf(delta));
    if(slow < PURSUIT_CURV_SLOW_MIN)
    {
        slow = PURSUIT_CURV_SLOW_MIN;
    }
    if(slow > 1.0f)
    {
        slow = 1.0f;
    }

    /* ---------------- 冲出赛道保护 ---------------- */
    near_sum = 0.0f;
    for(r = 0; r < 6; r ++)
    {
        near_sum += trk_center[r];
    }
    near_off  = ((near_sum / 6.0f) - PURSUIT_CENTER_COL) * ipm_info.dx_mm;
    off_track = 0;
    if((fabsf(near_off) > PURSUIT_OFFTRACK_MM) && (fabsf(near_off) < (3.0f * PURSUIT_OFFTRACK_MM)))
    {
        off_track = 1;
        steer     = (-near_off) * PURSUIT_RECOVER_GAIN;     /* 直接朝赛道中心打 */
        if(steer > ((float)SERVO_DUTY_RANGE_L))
        {
            steer = (float)SERVO_DUTY_RANGE_L;
        }
        if(steer < (-(float)SERVO_DUTY_RANGE_R))
        {
            steer = -(float)SERVO_DUTY_RANGE_R;
        }
    }

    /* ---------------- 组包发布 ---------------- */
    snap.valid         = (trk_valid >= PURSUIT_CROSS_MIN_VALID) ? 1 : 0;
    snap.cross         = cross_state;
    snap.off_track     = off_track;
    snap.valid_rows    = trk_valid;
    snap.open_rows     = trk_open;
    snap.one_side_rows = trk_one;
    snap.target_row    = r0;
    snap.target_col    = (int16)cc;
    snap.kappa         = kappa;
    snap.delta_deg     = delta;
    snap.steer_out     = steer;
    snap.slow          = slow;
    snap.ratio         = ratio;
    snap.lookahead_mm  = ld;
    snap.target_x_mm   = xt;
    snap.target_y_mm   = yt;
    snap.near_off_mm   = near_off;

    ps_buf[ps_pub ^ 1] = snap;
    ps_pub             = (uint8)(ps_pub ^ 1);

    pursuit_info = snap;
}

/* ============================================================================
 *  控制中断里取快照（双缓冲：读的时候读一个完整的旧包，不会读到写了一半的）
 * ==========================================================================*/
uint8 Pursuit_Get(pursuit_struct *out)
{
    uint8 idx;

    if(0 == out)
    {
        return (0);
    }

    idx  = ps_pub;
    *out = ps_buf[idx];

    return (out->valid);
}

void Pursuit_Init(void)
{
    uint8 r;

    memset(trk_center, 0, sizeof(trk_center));
    memset(trk_ok, 0, sizeof(trk_ok));
    memset(trk_lost_l, 0, sizeof(trk_lost_l));
    memset(trk_lost_r, 0, sizeof(trk_lost_r));
    memset(pursuit_center_row, 0, sizeof(pursuit_center_row));
    memset(&pursuit_info, 0, sizeof(pursuit_info));
    memset(ps_buf, 0, sizeof(ps_buf));

    for(r = 0; r < IPM_H; r ++)
    {
        trk_center[r]       = PURSUIT_CENTER_COL;
        pursuit_center_row[r] = (int16)PURSUIT_CENTER_COL;
    }

    trk_valid    = 0;
    trk_open     = 0;
    trk_one      = 0;
    cross_state  = 0;
    cross_hits   = 0;
    cross_ticks  = 0;
    cross_clean  = 0;
    cross_reject = 0;
    cross_kappa  = 0.0f;
    pursuit_hold = 0;
    ps_pub       = 0;
}

/* ============================================================================
 *  屏幕显示：TFT180 默认竖屏 128x160、字体 8x16（一个字符 8x16 像素）
 *    Y1~Y6（y = 2/18/34/50/68/84）已被相机/基点/基线/中线/控制五块占满，
 *    所以这里补第 7、8 行（y = 100 / 116）：
 *      第 7 行：P 看得清的行数 | c 路口标志 | d 前轮转角 | o 冲出赛道标志
 *      第 8 行：k 曲率 x1000 | L 前瞻距离 mm
 *    关键：zf 库的 show_int 字符串长度是 num+1，show_float 是 num+pointnum+2，
 *          不足补空格（定长），所以 x 必须按这个长度留格子，最后一格一旦到 x=128
 *          就会在 tft180.c:460 触发 zf_assert(x < tft180_width_max) 死机。
 * ==========================================================================*/
void Pursuit_Show_On_TFT180(void)
{
#if ((1 == PURSUIT_DIS_EN) && (0 == IPM_DIS_EN))
    uint16 y1 = 0;
    uint16 y2 = 0;

    y1 = (uint16)(CAMERA_INFO_TOP + 100);
    y2 = (uint16)(y1 + 16);
    if((y1 + 16) > tft180_height_max)
    {
        return;                                         /* 屏幕放不下（比如开了 CAMERA_DIS_IMAGE 整体下移）就不画 */
    }

    tft180_show_string(0,   y1, "P");
    tft180_show_int   (8,   y1, (int32)pursuit_info.valid_rows, 2);     /* 8  .. 31 */
    tft180_show_string(32,  y1, "c");
    tft180_show_int   (40,  y1, (int32)pursuit_info.cross, 1);          /* 40 .. 55 */
    tft180_show_string(56,  y1, "d");
    tft180_show_float (64,  y1, (double)pursuit_info.delta_deg, 2, 1);  /* 64 .. 103 */
    tft180_show_string(104, y1, "o");
    tft180_show_int   (112, y1, (int32)pursuit_info.off_track, 1);      /* 112 .. 127 */

    if((y2 + 16) > tft180_height_max)
    {
        return;
    }
    tft180_show_string(0,  y2, "k");
    tft180_show_float (8,  y2, (double)(pursuit_info.kappa * 1000.0f), 1, 2);   /* 8  .. 47 */
    tft180_show_string(48, y2, "L");
    tft180_show_int   (56, y2, (int32)pursuit_info.lookahead_mm, 3);            /* 56 .. 87 */
#endif
}
