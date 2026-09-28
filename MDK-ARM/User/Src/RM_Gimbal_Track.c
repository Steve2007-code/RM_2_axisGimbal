#include "RM_Gimbal_Track.h"
#include "DM_BLDC_Motor.h"
#include "UART_Data.h"
#include "can.h"

/*============================= 内部状态 =============================*/

GimbalTrack_TypeDef g_GimbalTrack;

/* 行缓冲，解析文本协议用 */
static char   s_line[48];
static uint8_t s_line_len = 0;

/* 转发暂存区，必须 static：HAL_UART_Transmit_IT 是异步的，只存指针不拷数据 */
static uint8_t s_fwd_buf[64];

/* DWT 时间戳 */
static uint32_t s_last_frame_cyc = 0;   /* 最近一帧视觉数据的时刻 */
static uint32_t s_last_ctrl_cyc  = 0;   /* 上次控制周期的时刻 */

/*============================= 小工具 =============================*/

static float Track_Abs(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float Track_Clamp(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* 解析首个带符号十进制整数，遇非数字停止（手写是为了不引 stdio，省 flash） */
static int Track_Atoi(const char *s)
{
    int val = 0;
    int neg = 0;

    if (*s == '-')      { neg = 1; s++; }
    else if (*s == '+') { s++; }

    while (*s >= '0' && *s <= '9')
    {
        val = val * 10 + (*s - '0');
        s++;
    }
    return neg ? -val : val;
}

/*============================= 达妙 MIT 打包 =============================*/

/*
 * 这两个打包函数和你 DM_BLDC_Motor.c 里的公式完全一样，
 * 之所以复制一份而不是直接调用，是因为你那两个是 static（外部看不见），
 * 而且我不想去改你的文件。
 * 公式：u = (x - xmin) / (xmax - xmin) * (2^N - 1)
 *       xmin 是负数这一步已经完成了零点平移，不要再额外减 2048
 */
static uint16_t Track_Mit16(float x, float xmin, float xmax)
{
    float v;
    x = Track_Clamp(x, xmin, xmax);
    v = (x - xmin) / (xmax - xmin) * 65535.0f + 0.5f;
    if (v < 0.0f)      v = 0.0f;
    if (v > 65535.0f)  v = 65535.0f;
    return (uint16_t)v;
}

static uint16_t Track_Mit12(float x, float xmin, float xmax)
{
    float v;
    x = Track_Clamp(x, xmin, xmax);
    v = (x - xmin) / (xmax - xmin) * 4095.0f + 0.5f;
    if (v < 0.0f)     v = 0.0f;
    if (v > 4095.0f)  v = 4095.0f;
    return (uint16_t)v;
}

/*
 * @brief  组一帧 MIT 控制帧并发出去
 * @param  ax  电机结构体（只用来取 hcan / ID / 力矩量程，不修改它的成员）
 * @note   位域布局: D0D1=pos16 | D2+D3高4=vel12 | D3低4+D4=kp12 | D5+D6高4=kd12 | D6低4+D7=tor12
 */
static HAL_StatusTypeDef Track_Send(DM_Motor_Data *ax,
                                    float pos, float vel, float kp, float kd, float tor,
                                    float pmax, float vmax)
{
    uint8_t  Data[8];
    uint16_t pos_raw;
    uint16_t vel_raw;
    uint16_t kp_raw;
    uint16_t kd_raw;
    uint16_t tor_raw;

    pos_raw = Track_Mit16(pos, -pmax, pmax);
    vel_raw = Track_Mit12(vel, -vmax, vmax);
    kp_raw  = Track_Mit12(kp,  0.0f, 500.0f);
    kd_raw  = Track_Mit12(kd,  0.0f, 5.0f);
    tor_raw = Track_Mit12(tor, ax->Torque_min, ax->Torque_max);

    Data[0] = (uint8_t)(pos_raw >> 8);
    Data[1] = (uint8_t)(pos_raw & 0xFF);
    Data[2] = (uint8_t)(vel_raw >> 4);
    Data[3] = (uint8_t)(((vel_raw & 0x0F) << 4) | (kp_raw >> 8));
    Data[4] = (uint8_t)(kp_raw & 0xFF);
    Data[5] = (uint8_t)(kd_raw >> 4);
    Data[6] = (uint8_t)(((kd_raw & 0x0F) << 4) | (tor_raw >> 8));
    Data[7] = (uint8_t)(tor_raw & 0xFF);

    return DM_CAN_Motor_Send_Data(ax->hcan, ax->ID, Data, 8);
}

/*============================= 协议解析 =============================*/

/* 收到一帧有效坐标 */
static void Track_OnFrame(int dx, int dy)
{
    uint32_t now;
    float dt_vis;
    float d_yaw_raw;
    float d_pit_raw;

    /* 视觉帧间隔用 DWT 实测：HAL_GetTick 只有 1ms 分辨率，30Hz 下量化误差 3% */
    now    = DWT->CYCCNT;
    dt_vis = (float)(uint32_t)(now - s_last_frame_cyc) / (float)SystemCoreClock;
    s_last_frame_cyc = now;
    if (dt_vis < 0.001f) dt_vis = 0.001f;
    if (dt_vis > 0.5f)   dt_vis = 0.5f;

    /*
     * D 项只在"有新视觉帧"的时候刷新一次，帧间沿用旧值。
     * 原因：视觉 30Hz，控制 500Hz，中间 16 个控制周期误差根本没变，
     * 若每周期都算 (err-err_last)/dt，帧间恒为 0、帧到达时又突然炸一下，
     * 输出会变成台阶状，电机一路抖。
     */
    d_yaw_raw = 0.0f;
    d_pit_raw = 0.0f;
    if (g_GimbalTrack.tracked)
    {
        d_yaw_raw = ((float)dx - g_GimbalTrack.err_yaw)   / dt_vis;
        d_pit_raw = ((float)dy - g_GimbalTrack.err_pitch) / dt_vis;
    }

    /* 一阶低通，find_blobs 的中心点本身有抖动，直接微分噪声很大 */
    g_GimbalTrack.d_err_yaw   = g_GimbalTrack.d_err_yaw   * 0.7f + d_yaw_raw * 0.3f;
    g_GimbalTrack.d_err_pitch = g_GimbalTrack.d_err_pitch * 0.7f + d_pit_raw * 0.3f;

    g_GimbalTrack.err_yaw   = (float)dx;
    g_GimbalTrack.err_pitch = (float)dy;
    g_GimbalTrack.tracked   = 1;
    g_GimbalTrack.lost_cnt  = 0;
    g_GimbalTrack.last_frame_ms = HAL_GetTick();
    g_GimbalTrack.frame_cnt++;
}

/* 收到"找不到" */
static void Track_OnLost(void)
{
    g_GimbalTrack.lost_cnt++;
    if (g_GimbalTrack.lost_cnt >= TRACK_LOST_CONFIRM)
    {
        g_GimbalTrack.tracked      = 0;
        g_GimbalTrack.err_yaw      = 0.0f;
        g_GimbalTrack.err_pitch    = 0.0f;
        g_GimbalTrack.d_err_yaw    = 0.0f;
        g_GimbalTrack.d_err_pitch  = 0.0f;
    }
}

/* 解析一行完整文本 */
/*
 * 协议格式：
 *   新：YSC,has,dx,dy,rst\n     has=1 有目标 / 0 无目标，rst=1 俯仰复位
 *   旧：YSC,dx,dy\n  或  YC,dx,dy\n  或  YSL\n / YL\n
 * 新旧靠逗号个数区分：新协议 4 个字段(3 个逗号)，旧协议 2 个字段(1 个逗号)
 */
static void Track_ParseLine(const char *s)
{
    const char *p;
    const char *q;
    int dx;
    int dy;
    int has;
    int rst;
    int nfield;

    if ((s[0] == 'Y') && (s[1] == 'S') && (s[2] == 'C') && (s[3] == ',')) p = s + 4;
    else if ((s[0] == 'Y') && (s[1] == 'C') && (s[2] == ','))             p = s + 3;
    else
    {
        /* YSL / YL：无目标 */
        if ((s[0] == 'Y') && ((s[1] == 'L') || ((s[1] == 'S') && (s[2] == 'L'))))
        {
            Track_OnLost();
        }
        return;
    }

    /* 数逗号 */
    nfield = 1;
    q = p;
    while (*q != '\0')
    {
        if (*q == ',') nfield++;
        q++;
    }

    has = 1;
    rst = 0;

    /* 新协议多出来的第一个字段：是否有目标 */
    if (nfield >= 4)
    {
        has = Track_Atoi(p);
        while ((*p != '\0') && (*p != ',')) p++;
        if (*p == ',') p++;
    }

    dx = Track_Atoi(p);
    while ((*p != '\0') && (*p != ',')) p++;
    if (*p == ',') p++;
    dy = Track_Atoi(p);

    /* 新协议最后多出来的字段：俯仰复位 */
    if (nfield >= 4)
    {
        while ((*p != '\0') && (*p != ',')) p++;
        if (*p == ',') p++;
        rst = Track_Atoi(p);
    }

    /* 复位字段独立于"是否有目标"：看不到目标时也能让俯仰回零 */
    if (rst) GimbalTrack_ResetPitch();
    else     g_GimbalTrack.reset_pitch = 0;

    if (has) Track_OnFrame(dx, dy);
    else     Track_OnLost();
}

void GimbalTrack_FeedByte(uint8_t c)
{
    if ((c == '\n') || (c == '\r'))
    {
        if (s_line_len > 0)
        {
            s_line[s_line_len] = '\0';
            s_line_len = 0;
            Track_ParseLine(s_line);
        }
        return;
    }

    /* 行超长说明收到的是乱码/粘包，直接丢掉重来，比解析出半个数安全 */
    if (s_line_len >= (uint8_t)(sizeof(s_line) - 1))
    {
        s_line_len = 0;
        return;
    }

    s_line[s_line_len++] = (char)c;
}

/*============================= 对外接口 =============================*/

void GimbalTrack_Init(void)
{
    /* 打开 DWT 周期计数器（Cortex-M4 自带，HCLK 分辨率，免费的高精度时基） */
    CoreDebug->DEMCR |= 0x01000000u;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= 0x00000001u;

    g_GimbalTrack.err_yaw        = 0.0f;
    g_GimbalTrack.err_pitch      = 0.0f;
    g_GimbalTrack.d_err_yaw      = 0.0f;
    g_GimbalTrack.d_err_pitch    = 0.0f;
    g_GimbalTrack.out_yaw        = 0.0f;
    g_GimbalTrack.out_pitch      = 0.0f;
    g_GimbalTrack.target_yaw     = 0.0f;
    g_GimbalTrack.target_pitch   = 0.0f;
    g_GimbalTrack.kp             = TRACK_KP;
    g_GimbalTrack.kd             = TRACK_KD;
    g_GimbalTrack.tracked        = 0;
    g_GimbalTrack.reset_pitch    = 0;
    g_GimbalTrack.lost_cnt       = TRACK_LOST_CONFIRM;
    g_GimbalTrack.frame_cnt      = 0;
    g_GimbalTrack.last_frame_ms  = HAL_GetTick();

    s_line_len       = 0;
    s_last_frame_cyc = DWT->CYCCNT;
    s_last_ctrl_cyc  = 0;
}

void GimbalTrack_Enable(void)
{
    /* 目标角从 0 起算，也就是电机自己的零点 */
    g_GimbalTrack.target_yaw   = 0.0f;
    g_GimbalTrack.target_pitch = 0.0f;

    DM_CAN_Motor_SetEnable(DM_Motor_4310_ID, DM_Mototr_Enable);
    DM_CAN_Motor_SetEnable(DM_Motor_4340_ID, DM_Mototr_Enable);
}

void GimbalTrack_Disable(void)
{
    DM_CAN_Motor_SetEnable(DM_Motor_4310_ID, DM_Mototr_Disable);
    DM_CAN_Motor_SetEnable(DM_Motor_4340_ID, DM_Mototr_Disable);
}

void GimbalTrack_ResetPitch(void)
{
    g_GimbalTrack.reset_pitch  = 1;
    g_GimbalTrack.target_pitch = 0.0f;
    g_GimbalTrack.out_pitch    = 0.0f;
    g_GimbalTrack.err_pitch    = 0.0f;
    g_GimbalTrack.d_err_pitch  = 0.0f;
}

void GimbalTrack_SetGain(float kp, float kd)
{
    g_GimbalTrack.kp = kp;
    g_GimbalTrack.kd = kd;
}

void GimbalTrack_Pump(void)
{
    uint16_t n = 0;
    uint8_t  c;

    /* 上一次还没发完就不要动缓冲，否则会把正在发的数据覆盖掉 */
    if (UART1_TX_Busy) return;

    while ((UART6_RX_idx != UART6_RX_Rd) && (n < (uint16_t)sizeof(s_fwd_buf)))
    {
        c = UART6_RX_Fifo[UART6_RX_Rd];
        UART6_RX_Rd = (UART6_RX_Rd + 1) % 128;

        GimbalTrack_FeedByte(c);    /* 顺手解析 */
        s_fwd_buf[n++] = c;         /* 再原样转发给串口1，方便电脑上看原始数据 */
    }

    if (n == 0) return;
    UART_Send_NonBlock(&huart1, s_fwd_buf, n);
}

void GimbalTrack_Control(void)
{
    uint32_t now;
    uint32_t period_cyc;
    float dt;
    float err_yaw;
    float err_pit;
    float out_yaw;
    float out_pit;
    float alpha;

    /* ---------- 限频：主循环裸奔也能保证固定控制周期 ---------- */
    now        = DWT->CYCCNT;
    period_cyc = (uint32_t)TRACK_CTRL_PERIOD_MS * (SystemCoreClock / 1000u);
    if (s_last_ctrl_cyc != 0)
    {
        if ((uint32_t)(now - s_last_ctrl_cyc) < period_cyc) return;
        dt = (float)(uint32_t)(now - s_last_ctrl_cyc) / (float)SystemCoreClock;
    }
    else
    {
        dt = (float)TRACK_CTRL_PERIOD_MS / 1000.0f;
    }
    s_last_ctrl_cyc = now;
    if (dt > 0.1f) dt = 0.1f;   /* 被高优先级任务卡住过，别让这一拍积分爆掉 */

    /* ---------- 视觉掉线保护：太久没新帧也判丢 ---------- */
    if (g_GimbalTrack.tracked &&
        ((uint32_t)(HAL_GetTick() - g_GimbalTrack.last_frame_ms) > (uint32_t)TRACK_VIS_TIMEOUT_MS))
    {
        g_GimbalTrack.tracked     = 0;
        g_GimbalTrack.err_yaw     = 0.0f;
        g_GimbalTrack.err_pitch   = 0.0f;
        g_GimbalTrack.d_err_yaw   = 0.0f;
        g_GimbalTrack.d_err_pitch = 0.0f;
    }

    /* ---------- 取误差，没看到目标就是 0（输出 0 -> 电机保持不动） ---------- */
    err_yaw = g_GimbalTrack.tracked ? g_GimbalTrack.err_yaw   : 0.0f;
    err_pit = g_GimbalTrack.tracked ? g_GimbalTrack.err_pitch : 0.0f;

    if (Track_Abs(err_yaw) < (float)TRACK_DEADZONE_PX) err_yaw = 0.0f;
    if (Track_Abs(err_pit) < (float)TRACK_DEADZONE_PX) err_pit = 0.0f;

    /* ---------- PD ---------- */
    out_yaw = g_GimbalTrack.kp * err_yaw   + g_GimbalTrack.kd * g_GimbalTrack.d_err_yaw;
    out_pit = g_GimbalTrack.kp * err_pit   + g_GimbalTrack.kd * g_GimbalTrack.d_err_pitch;

    out_yaw = Track_Clamp(out_yaw, -TRACK_OUT_MAX, TRACK_OUT_MAX);
    out_pit = Track_Clamp(out_pit, -TRACK_OUT_MAX, TRACK_OUT_MAX);

    /* 一阶低通：视觉 30Hz 而控制 500Hz，不抹平的话每个视觉帧到达时速度都会跳一下 */
    alpha = dt / (dt + TRACK_OUT_TAU);
    g_GimbalTrack.out_yaw   += (out_yaw - g_GimbalTrack.out_yaw)   * alpha;
    g_GimbalTrack.out_pitch += (out_pit - g_GimbalTrack.out_pitch) * alpha;

    /* ---------- 输出到电机 ---------- */
    g_GimbalTrack.target_yaw  = Track_Clamp(g_GimbalTrack.target_yaw,
                                            -TRACK_YAW_LIMIT_DEG * PI / 180.0f,
                                             TRACK_YAW_LIMIT_DEG * PI / 180.0f);
    g_GimbalTrack.target_pitch  = Track_Clamp(g_GimbalTrack.target_pitch,
                                              -TRACK_PITCH_LIMIT_DEG * PI / 180.0f,
                                               TRACK_PITCH_LIMIT_DEG * PI / 180.0f);

#if TRACK_YAW_USE_VEL
    /* 速度模式：kp=0 关掉电机位置环，vel 直接给角速度。
       静止时 vel=0，T = kd*(0 - v) 会把速度拉回 0，等效刹车保持，不会漂 */
    Track_Send(&DM_Motor_4340_StructData,
               g_GimbalTrack.target_yaw,
               (float)TRACK_YAW_SIGN * g_GimbalTrack.out_yaw,
               0.0f, TRACK_VEL_KD, 0.0f,
               TRACK_YAW_PMAX, TRACK_YAW_VMAX);
#else
    /* 位置模式：把角速度积分成目标角，再交给电机内部位置环。
       前提：电机真实 PMAX 必须等于 TRACK_YAW_PMAX，见 .h 顶部警告 */
    g_GimbalTrack.target_yaw += (float)TRACK_YAW_SIGN * g_GimbalTrack.out_yaw * dt;
    Track_Send(&DM_Motor_4340_StructData,
               g_GimbalTrack.target_yaw, 0.0f,
               TRACK_POS_KP, TRACK_POS_KD, 0.0f,
               TRACK_YAW_PMAX, TRACK_YAW_VMAX);
#endif

    /* 复位期间：俯仰不积分、目标角锁 0，电机内部位置环把它拉回零点 */
    if (g_GimbalTrack.reset_pitch)
    {
        g_GimbalTrack.target_pitch = 0.0f;
        g_GimbalTrack.out_pitch    = 0.0f;
#if TRACK_PITCH_USE_VEL
        Track_Send(&DM_Motor_4310_StructData,
                   0.0f, 0.0f,
                   0.0f, TRACK_VEL_KD, 0.0f,
                   TRACK_PITCH_PMAX, TRACK_PITCH_VMAX);
#else
        Track_Send(&DM_Motor_4310_StructData,
                   0.0f, 0.0f,
                   TRACK_POS_KP, TRACK_POS_KD, 0.0f,
                   TRACK_PITCH_PMAX, TRACK_PITCH_VMAX);
#endif
        return;
    }

#if TRACK_PITCH_USE_VEL
    Track_Send(&DM_Motor_4310_StructData,
               g_GimbalTrack.target_pitch,
               (float)TRACK_PITCH_SIGN * g_GimbalTrack.out_pitch,
               0.0f, TRACK_VEL_KD, 0.0f,
               TRACK_PITCH_PMAX, TRACK_PITCH_VMAX);
#else
    g_GimbalTrack.target_pitch += (float)TRACK_PITCH_SIGN * g_GimbalTrack.out_pitch * dt;
    Track_Send(&DM_Motor_4310_StructData,
               g_GimbalTrack.target_pitch, 0.0f,
               TRACK_POS_KP, TRACK_POS_KD, 0.0f,
               TRACK_PITCH_PMAX, TRACK_PITCH_VMAX);
#endif
}
