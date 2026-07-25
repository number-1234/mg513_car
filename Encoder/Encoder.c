/**
 * @file    Encoder.c
 * @brief   双路编码器 — GPIO 中断计数 + 100ms 速度换算
 *
 * ┌──────────────┬──────────┬──────────────┐
 * │ 编码器       │ 引脚     │ 对应电机     │
 * ├──────────────┼──────────┼──────────────┤
 * │ A (AA/AB)    │ PA17/16  │ 左电机 A     │
 * │ B (BA/BB)    │ PA15/14  │ 右电机 B     │
 * └──────────────┴──────────┴──────────────┘
 *
 * 硬件参数：电机轴 260PPR，减速比 28:1，轮径 65mm
 * 轮轴一圈 = 260×28 = 7280 脉冲
 * 轮周长 = π×65 ≈ 204.2mm
 *
 * 速度换算（每 100ms）：
 *   V(mm/s) = (pulse / 7280) × π×65 × 10
 *
 * 注意：右电机对称安装，前进时 EncoderB 计数为负，
 *       Read_Encoder 取绝对值后 EncoderB_VEL 始终 ≥ 0。
 */
#include "Encoder.h"
#include "ti_msp_dl_config.h"

/* ── 物理常量 ── */
#define ENCODER_PPR_MOTOR      260.0f     /* 电机轴每圈脉冲 */
#define GEAR_RATIO              28.0f     /* 减速比 */
#define PULSES_PER_WHEEL       (ENCODER_PPR_MOTOR * GEAR_RATIO)  /* 7280 */
#define WHEEL_DIAMETER_MM       65.0f     /* 轮径 */
#define SAMPLE_RATE_HZ          10.0f     /* 100ms → 10Hz */

/* 换算系数 = π×D×f / PPR，预计算避免浮点重复运算 */
#define VEL_SCALE  ((3.14159f * WHEEL_DIAMETER_MM * SAMPLE_RATE_HZ) / PULSES_PER_WHEEL)

/* ── 全局变量 ── */
volatile int32_t EncoderA_CNT;
volatile int32_t EncoderB_CNT;
volatile float   EncoderA_VEL;
volatile float   EncoderB_VEL;
volatile int32_t EncoderA_Last_Raw;
volatile int32_t EncoderB_Last_Raw;

/* ================================================================
 *  初始化
 * ================================================================ */
void Encoder_Init(void)
{
    EncoderA_CNT  = 0;
    EncoderB_CNT  = 0;
    EncoderA_VEL  = 0.0f;
    EncoderB_VEL  = 0.0f;
    EncoderA_Last_Raw = 0;
    EncoderB_Last_Raw = 0;

    /* 使能电机驱动 */
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_SBYT_PIN);

    /* 4 路 PWM 初始归零，电机停转 */
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0U, GPIO_PWM_0_C0_IDX);  /* PA21 */
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0U, GPIO_PWM_0_C1_IDX);  /* PA22 */
    DL_Timer_setCaptureCompareValue(PWM_1_INST, 0U, GPIO_PWM_1_C1_IDX);  /* PA24 */
    DL_Timer_setCaptureCompareValue(PWM_2_INST, 0U, GPIO_PWM_2_C1_IDX);  /* PA25 */

    /* 启动 3 个 PWM 定时器 */
    DL_Timer_startCounter(PWM_0_INST);
    DL_Timer_startCounter(PWM_1_INST);
    DL_Timer_startCounter(PWM_2_INST);

    /* 编码器 GPIO 中断 — 双边沿计数 */
    NVIC_ClearPendingIRQ(encoder_INT_IRQN);
    NVIC_EnableIRQ(encoder_INT_IRQN);

    /* 100ms 速度采样定时器 */
    DL_Timer_startCounter(TIMER_0_INST);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

/* ================================================================
 *  复位
 * ================================================================ */
void Encoder_Reset(void)
{
    NVIC_DisableIRQ(encoder_INT_IRQN);
    EncoderA_CNT = 0;
    EncoderB_CNT = 0;
    EncoderA_VEL = 0.0f;
    EncoderB_VEL = 0.0f;
    NVIC_EnableIRQ(encoder_INT_IRQN);
}

/* ================================================================
 *  GPIO 中断：AB 相正交解码
 *
 *  由 main.c → GROUP1_IRQHandler 调用。
 *  上升沿触发 AA(PA17) 或 BA(PA15)，
 *  读对应 B 相电平判断方向。
 * ================================================================ */
void Encodering(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOA)) {

    case encoder_AA_IIDX:    /* 编码器 A — 左电机 */
        if (DL_GPIO_readPins(GPIOA, encoder_AB_PIN) != 0U) {
            EncoderA_CNT++;
        } else {
            EncoderA_CNT--;
        }
        break;

    case encoder_BA_IIDX:    /* 编码器 B — 右电机 */
        if (DL_GPIO_readPins(GPIOA, encoder_BB_PIN) != 0U) {
            EncoderB_CNT++;
        } else {
            EncoderB_CNT--;
        }
        break;

    default:
        break;
    }
}

/* ================================================================
 *  100ms 定时回调：清零计数 → 换算速度
 *
 *  由 main.c → TIMA0_IRQHandler 调用。
 *  临界区关中断，原子读取 + 清零。
 *  速度取绝对值——右电机前进时编码器为负是正常的（对称安装）。
 * ================================================================ */
void Read_Encoder(void)
{
    int32_t raw_a, raw_b;

    /* 原子读取 + 清零 */
    NVIC_DisableIRQ(encoder_INT_IRQN);
    raw_a = EncoderA_CNT;
    raw_b = EncoderB_CNT;
    EncoderA_CNT = 0;
    EncoderB_CNT = 0;
    NVIC_EnableIRQ(encoder_INT_IRQN);

    /* 保存原始值（调试用） */
    EncoderA_Last_Raw = raw_a;
    EncoderB_Last_Raw = raw_b;

    /* 取绝对值后换算线速度 mm/s */
    if (raw_a < 0) raw_a = -raw_a;
    if (raw_b < 0) raw_b = -raw_b;

    EncoderA_VEL = (float)raw_a * VEL_SCALE;
    EncoderB_VEL = (float)raw_b * VEL_SCALE;
}
