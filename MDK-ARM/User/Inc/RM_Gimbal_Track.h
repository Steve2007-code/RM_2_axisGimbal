#ifndef __RM_GIMBAL_TRACK_H__
#define __RM_GIMBAL_TRACK_H__

#include "main.h"

/*==============================================================================
 * RM 二轴云台 视觉追踪算法（K230 串口6 -> PD -> 达妙电机）
 *
 * 数据链路：
 *   K230(UART6) -- "YSC,dx,dy\n" / "YSL\n" --> 本模块解析 --> PD --> MIT帧 --> CAN
 *
 * 轴分配：
 *   4310 = 俯仰 pitch
 *   4340 = 偏航 yaw
 *
 * 丢失目标：
 *   收到 YSL 或 超过 TRACK_VIS_TIMEOUT_MS 没新帧 --> 误差清零 --> 输出归零
 *   --> 目标角度不再累加 --> 电机停在当前位置不动
 *
 *----------------------------------------------------------------------------
 * 【PMAX 警告 · 上电前必看】
 * 达妙 MIT 的位置字段是 16bit 线性映射到 [-PMAX, +PMAX]，所以位置分辨率 = 2*PMAX/65535。
 *
 *   4310：PMAX=12.5rad  -> 分辨率 0.00038rad = 0.022度/LSB  -> 很细，可以直接位置控制
 *   4340：你现在结构体里填的是 1000
 *         -> 分辨率 2000/65535 = 0.0305rad = 1.75度/LSB
 *         -> 转 60 度只有 68 个计数，位置控制会一格一格地抖，根本没法追踪
 *
 * 所以 yaw 默认走【速度模式】：kp 设 0，只用 vel 字段发角速度，
 *   vel 是 12bit 映射到 ±VMAX(10rad/s)，分辨率 0.0049rad/s，不受 PMAX 影响。
 *
 * 想让 yaw 也走位置模式，必须两步都做：
 *   1) 达妙上位机把 4340 的 RID21(PMAX) 改回 12.5
 *   2) 把本文件 TRACK_YAW_USE_VEL 改成 0、TRACK_YAW_PMAX 改成 12.5
 *
 * 怎么确认电机真实的 PMAX：发一帧 pos=1.0rad 的位置指令，看它实际转多少度。
 *   实际转角 = 指令 / 80   -->  说明电机 PMAX 还是 12.5，而你按 1000 编码了
 *   （增益差 80 倍，表现为"发了指令但电机几乎不动"）
 *============================================================================*/

/*============================= 可调参数 =============================*/

/* 控制周期(ms)。主循环是裸奔的，靠这个限频。
   2ms 发 2 帧 CAN，1Mbps 下约 111us/帧，总线占用约 11%，同时满足达妙超时保护(需 >100Hz) */
#define TRACK_CTRL_PERIOD_MS    2

/* PD 增益。误差单位直接用像素，输出单位 rad/s，所以：
   TRACK_KP = 0.0025 表示 偏差 100px 时给出 0.25 rad/s(约 14度/s) 的角速度
   TRACK_KD 单位是 (rad/s)/(px/s)，量级比 Kp 小一个数量级，典型值约 Kp 的 1/10

   [这组值是 PC 闭环仿真扫出来的，不是拍脑袋]
   模型: FOV 60度(1833 px/rad) + 视觉延迟 66ms + 30Hz + 控制 2ms + 电机一阶滞后
   结果: 14度阶跃 -> 超调 0%、稳态误差 0.06度、纹波 0.03度、到位 0.38s

   [坑] Kd 千万不要按"和 Kp 差不多"来给。D 项信号本身带着视觉延迟+低通，
        Kd=0.0015 时仿真纹波直接炸到 11度（持续振荡，云台一直抖）。
        实测绝对上限：30fps -> 0.0002，95fps -> 0.0003，超过 0.001 必炸。
        Kd 不随 Kp 线性放大（D 项乘的是 px/s，量级由噪声和帧率决定），
        别按 Kp 的 1/10 去给大 Kp 配 Kd，会超标。宁可先用 Kd=0 纯 P 跑。

   [镜头 FOV 不是 60度时] 按这个缩放:
        px_per_rad = 画面宽(px) / FOV(弧度)
        Kp_new = TRACK_KP * 1833 / px_per_rad
        FOV 越大 -> 画面越"广" -> 同样像素偏差对应的角度越小 -> Kp 可以越大 */
#define TRACK_KP                0.0080f
#define TRACK_KD                0.0003f

/* 输出限幅(rad/s)，防止一帧大误差直接把云台甩飞 */
#define TRACK_OUT_MAX           3.0f

/* 输出一阶低通时间常数(s)。视觉帧只有 30Hz，帧间输出会跳变，靠这个抹平。
   调小 = 更灵敏更抖；调大 = 更平滑更迟钝 */
#define TRACK_OUT_TAU           0.03f

/* 死区(px)。偏差小于它就认为已经对准，输出 0，防止到位后电机来回抖 */
#define TRACK_DEADZONE_PX       10

/* 连续收到几帧"找不到"才真的判丢。防单帧误检（光照抖动、遮挡一瞬） */
#define TRACK_LOST_CONFIRM      3

/* 视觉超时(ms)。这么久没收到任何新帧也判丢，防 K230 掉线/卡死后云台一直转 */
#define TRACK_VIS_TIMEOUT_MS    500

/* 方向符号。装机后如果某个轴是反向追（越追越偏），把对应的改成 -1 就行，
   不用改算法也不用改接线 */
#define TRACK_YAW_SIGN          -1      /* 4340 偏航：实测方向反了，取反 */
#define TRACK_PITCH_SIGN        1       /* 4310 俯仰 */

/* 目标角度限幅(度)。防止一直积分转出机械限位或绕线 */
#define TRACK_YAW_LIMIT_DEG     60.0f
#define TRACK_PITCH_LIMIT_DEG   35.0f

/* 各轴控制模式：1=速度模式(发 vel)，0=位置模式(发 pos)
   pitch 用位置模式是因为电机内部位置环能顶住重力，松手不下垂
   yaw 用速度模式是为了绕开 PMAX=1000 的分辨率坑（见顶部警告） */
#define TRACK_PITCH_USE_VEL     0
#define TRACK_YAW_USE_VEL       1

/* 电机内部实际的量程，必须和达妙上位机里读出来的一致 */
#define TRACK_PITCH_PMAX        12.5f   /* 4310 位置量程 rad */
#define TRACK_PITCH_VMAX        30.0f   /* 4310 速度量程 rad/s */
#define TRACK_YAW_PMAX          1000.0f /* 4340 位置量程 rad（仅位置模式用到） */
#define TRACK_YAW_VMAX          10.0f   /* 4340 速度量程 rad/s */

/* 电机内部伺服参数（MIT 帧里的 kp/kd，不是我们这个 PD 的 Kp/Kd，别搞混）
   速度模式：kp=0 关掉位置环，kd 是速度阻尼，单位 Nm/(rad/s)，调小会软、调大会抖
   位置模式：kp 是位置刚度，kd 是阻尼，这两个值沿用你 DM_CAN_Motor_Init 里的默认 */
#define TRACK_VEL_KD            5.0f
#define TRACK_POS_KP            20.0f
#define TRACK_POS_KD            2.0f

/*============================= 数据结构 =============================*/

typedef struct
{
    /* 视觉输入 */
    float err_yaw;              /* 当前偏航偏差 px   = dx = 目标中心x - 画面中心x */
    float err_pitch;            /* 当前俯仰偏差 px   = dy = 目标中心y - 画面中心y */
    float d_err_yaw;            /* 偏航偏差变化率 px/s（每收到新帧刷新一次，帧间沿用） */
    float d_err_pitch;          /* 俯仰偏差变化率 px/s */
    uint8_t  tracked;           /* 1=当前看到目标  0=丢失 */
    uint8_t  lost_cnt;          /* 连续丢失帧计数 */
    uint8_t  reset_pitch;       /* 1=俯仰复位中：PD 暂停、目标角锁 0（由 rst 字段置位） */
    uint32_t frame_cnt;         /* 收到的有效帧计数（调试用，看它涨说明收到数据了） */
    uint32_t last_frame_ms;     /* 最近一帧时刻 HAL_GetTick() */

    /* 控制输出 */
    float kp;                   /* 在线可调的比例增益 */
    float kd;                   /* 在线可调的微分增益 */
    float out_yaw;              /* 偏航 PD 输出 rad/s（低通后） */
    float out_pitch;            /* 俯仰 PD 输出 rad/s（低通后） */
    float target_yaw;           /* 偏航目标角 rad（位置模式下积分得到） */
    float target_pitch;         /* 俯仰目标角 rad */
} GimbalTrack_TypeDef;

/* 全局句柄，方便在调试窗口里直接观察/修改 */
extern GimbalTrack_TypeDef g_GimbalTrack;

/*============================= 函数声明 =============================*/

/**
  * @brief  追踪算法初始化
  * @param  无
  * @note   会打开 DWT 周期计数器用于测 dt；必须在 GimbalTrack_Enable 之前调用
  * @retval 无
  */
void GimbalTrack_Init(void);

/**
  * @brief  使能两颗电机并把目标角清零
  * @param  无
  * @note   使能瞬间电机会转到 target=0，也就是它自己的零点。
  *         建议先让云台摆到机械中位、再上电（或用上位机存零点），否则使能会跳一下
  * @retval 无
  */
void GimbalTrack_Enable(void);

/**
  * @brief  失能两颗电机
  * @param  无
  * @note   无
  * @retval 无
  */
void GimbalTrack_Disable(void);

/**
  * @brief  喂一个字节给协议解析器
  * @param  c 从串口6 收到的一个字节
  * @note   内部按行(\n)缓冲解析。
  *         新协议 YSC,has,dx,dy,rst ：has=1 有目标/0 无目标，rst=1 俯仰复位
  *         旧协议 YSC,dx,dy 和 YC,dx,dy / YSL / YL 仍然兼容（按逗号个数自动判断）
  *         二进制协议也不要它，改协议时只需要改这一个函数
  * @retval 无
  */
void GimbalTrack_FeedByte(uint8_t c);

/**
  * @brief  取串口6 数据 + 解析 + 转发到串口1
  * @param  无
  * @note   这个函数是 UART_Send_UART6_To_UART1 的替代品，多了"边转发边解析"这一步。
  *         用法：把 main.c while 里的 UART_Send_UART6_To_UART1(); 换成 GimbalTrack_Pump();
  *         不要两个都调用，它们会抢同一份 FIFO 数据
  * @retval 无
  */
void GimbalTrack_Pump(void);

/**
  * @brief  PD 控制 + 发 CAN 帧（定频执行）
  * @param  无
  * @note   内部按 TRACK_CTRL_PERIOD_MS 限频，所以放在裸奔的 while 里直接调就行。
  *         没看到目标时输出 0、目标角保持不变，电机停在原地
  * @retval 无
  */
void GimbalTrack_Control(void);

/**
  * @brief  在线改 PD 增益
  * @param  kp 比例增益 (rad/s)/px - kd 微分增益 (rad/s)/(px/s)
  * @note   调参用，不用每次重新编译
  * @retval 无
  */
void GimbalTrack_SetGain(float kp, float kd);

/**
  * @brief  俯仰轴复位（目标角归零并锁住）
  * @param  无
  * @note   一般由串口 rst 字段触发，也可以手动调用。
  *         复位期间俯仰 PD 暂停积分、目标角锁 0；rst 撤销后自动恢复跟踪
  * @retval 无
  */
void GimbalTrack_ResetPitch(void);

#endif
