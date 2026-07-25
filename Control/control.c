/**
 * @file    control.c
 * @brief   小车运动控制核心模块
 *
 * 整体架构：由 100ms 定时器周期性调用 Control() 函数，根据外部设置的
 * flag 决定当前工作模式（循迹/停车/转向/直行），计算出左右电机的目标速度，
 * 再通过两个独立的 PI 速度闭环控制电机 PWM 输出。
 *
 * ┌──────────┐    ┌──────────────┐    ┌──────────┐    ┌──────────┐
 * │ 传感器/  │───▶│ Control()    │───▶│ PID_A/B  │───▶│ Set_Pwm()│
 * │ 陀螺仪   │    │ 模式判断+解算 │    │ 速度闭环  │    │ 电机驱动  │
 * └──────────┘    └──────────────┘    └──────────┘    └──────────┘
 *
 * 四种运行模式（由全局变量 flag 决定）：
 *   flag=1  循迹模式   — 传感器 PD + 陀螺仪 Yaw 阻尼 → 差速转向
 *   flag=2  停车等待   — 两轮速度归零，清空角度记录
 *   flag=3  单轮转向   — 陀螺仪 Yaw 到位判断，原地转向
 *   flag=4  陀螺仪直行 — 保持当前航向角直线前进，P 修正航向偏差
 */

#include "control.h"

#include "Encoder/Encoder.h"
#include "GYRO/GYRO_JY901.h"
#include "Sensor/Sensor.h"
#include "sys/sys.h"

/* ================================================================
 *  全局控制参数（所有可调参数集中在此，方便整定）
 * ================================================================ */

/* ---------- PWM 限幅 ---------- */
#define PWM_MAX                     1000.0f

/* ---------- 循迹模式参数 ---------- */
#define LINE_SPEED_MM_S              100.0f   /* 循迹基础线速度 (mm/s) */
#define LINE_MAX_SPEED_MM_S          150.0f   /* 循迹差速上限 */
#define LINE_WEIGHT_TO_SPEED          7.0f   /* 传感器偏差 → P 系数 */
#define LINE_KD                       5.5f   /* 传感器偏差变化 → D 系数 */

/* ---------- 陀螺仪直行模式参数 ---------- */
#define STRAIGHT_SPEED_MM_S          100.0f
#define STRAIGHT_KP                    0.2f
#define STRAIGHT_BIAS_MAX             150.0f

/* ---------- 单轮转向模式参数 ---------- */
#define TURN_ANGLE_DEG                55.0f
#define TURN_SPEED_MM_S              100.0f
#define TURN_TOLERANCE_DEG             2.0f
#define TURN_PWM                       150

/* ---------- 速度 PI 控制器参数 ---------- */
#define SPEED_PWM_FEEDFORWARD          0.58f
#define SPEED_KP                       0.25f
#define SPEED_KI                       0.50f
#define SPEED_DT_S                     0.10f
#define SPEED_INTEGRAL_MAX              400.0f

/* ================================================================
 *  全局变量
 * ================================================================ */

extern volatile int flag;

volatile int Motor_Left;
volatile int Motor_Right;
volatile float Turn_Zero_Yaw;
volatile float Straight_Zero_Yaw;

/* PID 分量（PID 调参用） */
volatile float PID_A_Target;
volatile float PID_B_Target;
volatile float PID_A_Error;
volatile float PID_B_Error;
volatile float PID_A_P;
volatile float PID_A_I;
volatile float PID_B_P;
volatile float PID_B_I;

/* 循迹 PD 分量（调参用） */
volatile float Line_Deviation;
volatile float Line_Bias_P;
volatile float Line_Bias_D;
volatile float Line_Bias_Total;

/* ---------- 模块内部静态变量 ---------- */
static float s_integral_a;
static float s_integral_b;
static float s_control_angle;
static float s_control_bias;
static float s_last_deviation;    /* 上一周期传感器偏差（D 项用） */

/* ================================================================
 *  初始化
 * ================================================================ */

void Control_Init(void)
{
    Motor_Left = 0;
    Motor_Right = 0;
    Turn_Zero_Yaw = 0.0f;
    Straight_Zero_Yaw = 0.0f;
    s_integral_a = 0.0f;
    s_integral_b = 0.0f;
    s_control_angle = 0.0f;
    s_control_bias = 0.0f;
    s_last_deviation = 0.0f;
    Set_Pwm(0, 0);
}

/* ================================================================
 *  主控制函数（每 100ms 由定时器中断调用一次）
 * ================================================================ */

void Control(void)
{
    float TargetA = 0.0f;
    float TargetB = 0.0f;
    float bias = 0.0f;

    /* ============================================================
     *  flag=1 — 循迹模式：传感器 PD + 陀螺仪 Yaw 阻尼
     *
     *  bias = 传感器P + 传感器D + 陀螺阻尼
     *  车身转得越快阻尼越大，抑制直线晃动和甩尾
     * ============================================================ */
    if (flag == 1) {
        float deviation = (float)Incremental_Quantity();

        /* 传感器 PD */
        bias = -deviation * LINE_WEIGHT_TO_SPEED;
        bias += -(deviation - s_last_deviation) * LINE_KD;


        /* 存储循迹 PD 分量供上位机调参 */
        Line_Deviation = deviation;
        Line_Bias_P    = -deviation * LINE_WEIGHT_TO_SPEED;
        Line_Bias_D    = -(deviation - s_last_deviation) * LINE_KD;
        Line_Bias_Total = bias;
        s_last_deviation = deviation;

        /* 存储循迹 PD 分量供上位机调参 */

        /* 差速分配：左+右- */
        TargetA = LINE_SPEED_MM_S + bias;
        TargetB = LINE_SPEED_MM_S - bias;

        /* 限幅 [0, LINE_MAX] */
        TargetA = PWM_Limit(TargetA, LINE_MAX_SPEED_MM_S, 0.0f);
        TargetB = PWM_Limit(TargetB, LINE_MAX_SPEED_MM_S, 0.0f);

        s_control_angle = 0.0f;
        s_control_bias = bias;
    }

    /* ============================================================
     *  flag=2 — 停车等待
     * ============================================================ */
    else if (flag == 2) {
        TargetA = 0.0f;
        TargetB = 0.0f;
        s_control_angle = 0.0f;
        s_control_bias = 0.0f;
    }

    /* ============================================================
     *  flag=3 — 简单原地转向（Yaw 43~47 到位）
     * ============================================================ */
    else if (flag == 3) {
        if (!GYRO_Is_Ready()) {
            Motor_Left  = 0;
            Motor_Right = 0;
        } else if (Yaw > 47.0f) {
            Motor_Left  = TURN_PWM;
            Motor_Right =  0;
        } else if (Yaw < 43.0f) {
            Motor_Left  =  0;
            Motor_Right = TURN_PWM;
        } else {
            Motor_Left  = 0;
            Motor_Right = 0;
            Straight_Zero_Yaw = Yaw;
            flag = 4;
        }
        s_control_angle = Yaw;
        s_control_bias   = 0.0f;
    }

    /* ============================================================
     *  flag=4 — 陀螺仪直行模式
     * ============================================================ */
    else if (flag == 4) {
        if (GYRO_Is_Ready()) {
            s_control_angle = normalize_angle(Yaw - Straight_Zero_Yaw);
            bias = GYRO_Control(s_control_angle, 0.0f);
        } else {
            s_control_angle = 0.0f;
            bias = 0.0f;
        }
        s_control_bias = bias;
        TargetA = STRAIGHT_SPEED_MM_S + bias;
        TargetB = STRAIGHT_SPEED_MM_S - bias;
    }

    /* ============================================================
    /* ============================================================
     *  flag=5 — PID 调参模式（固定目标速度）
     * ============================================================ */
    else if (flag == 5) {
        TargetA = 100.0f;
        TargetB = 100.0f;
        s_control_angle = 0.0f;
        s_control_bias = 0.0f;
    }

   
    else {
        TargetA = 0.0f;
        TargetB = 0.0f;
    }

    /* ============================================================
     *  flag=3 直接 PWM 控制，不走 PID
     * ============================================================ */
    if (flag == 3) {
        Set_Pwm(Motor_Left, Motor_Right);
        return;
    }

    /* ============================================================
     *  速度闭环 + PWM 输出（flag 1/2/4 公共出口）
     * ============================================================ */
    Motor_Left  = (int)PWM_Limit(PID_A(EncoderA_VEL, TargetA), PWM_MAX, 0.0f);
    Motor_Right = (int)PWM_Limit(PID_B(EncoderB_VEL, TargetB), PWM_MAX, 0.0f);
    Set_Pwm(Motor_Left, Motor_Right);
}

/* ================================================================
 *  紧急停止
 * ================================================================ */

void Control_Stop(void)
{
    s_integral_a = 0.0f;
    s_integral_b = 0.0f;
    Motor_Left = 0;
    Motor_Right = 0;
    Set_Pwm(0, 0);
}

/* ================================================================
 *  底层电机驱动
 * ================================================================ */

/**
 * @brief  4 路 PWM 输出（3 定时器）
 *
 *  PWM_0 (TIMG6)  → C0=PA21, C1=PA22
 *  PWM_1 (TIMA1)  → C1=PA24
 *  PWM_2 (TIMG12) → C1=PA25
 *
 *  左电机 A：PA25(前进) + PA21(后退)
 *  右电机 B：PA22(前进) + PA24(后退)
 */
void Set_Pwm(int Left, int Right)
{
    Left  = (int)PWM_Limit((float)Left,  PWM_MAX, -PWM_MAX);
    Right = (int)PWM_Limit((float)Right, PWM_MAX, -PWM_MAX);

    /* ---- 左电机 A ---- */
    if (Left > 0) {
        DL_Timer_setCaptureCompareValue(PWM_2_INST, (uint32_t)Left,
                                        GPIO_PWM_2_C1_IDX);       /* PA25 前进 */
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0,
                                        GPIO_PWM_0_C0_IDX);       /* PA21 =0 */
    } else if (Left < 0) {
        DL_Timer_setCaptureCompareValue(PWM_2_INST, 0,
                                        GPIO_PWM_2_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)myabs(Left),
                                        GPIO_PWM_0_C0_IDX);       /* PA21 后退 */
    } else {
        DL_Timer_setCaptureCompareValue(PWM_2_INST, (uint32_t)PWM_MAX,
                                        GPIO_PWM_2_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)PWM_MAX,
                                        GPIO_PWM_0_C0_IDX);
    }

    /* ---- 右电机 B ---- */
    if (Right > 0) {
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)Right,
                                        GPIO_PWM_0_C1_IDX);       /* PA22 前进 */
        DL_Timer_setCaptureCompareValue(PWM_1_INST, 0,
                                        GPIO_PWM_1_C1_IDX);       /* PA24 =0 */
    } else if (Right < 0) {
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0,
                                        GPIO_PWM_0_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_1_INST, (uint32_t)myabs(Right),
                                        GPIO_PWM_1_C1_IDX);       /* PA24 后退 */
    } else {
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)PWM_MAX,
                                        GPIO_PWM_0_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_1_INST, (uint32_t)PWM_MAX,
                                        GPIO_PWM_1_C1_IDX);
    }
}

/* ================================================================
 *  速度 PI 控制器（左右轮各一个独立实例）
 * ================================================================ */

float PID_A(float Encoder, float Target)
{
    float error, pwm;

    if (Target <= 0.0f) {
        s_integral_a = 0.0f;
        return 0.0f;
    }

    error = Target - Encoder;
    s_integral_a += error * SPEED_DT_S;
    s_integral_a = PWM_Limit(s_integral_a, SPEED_INTEGRAL_MAX, -SPEED_INTEGRAL_MAX);

    pwm  = SPEED_PWM_FEEDFORWARD * Target;

    /* 存储 PID 分量供上位机调参 */
    PID_A_Target = Target;
    PID_A_Error  = error;
    PID_A_P      = SPEED_KP * error;
    PID_A_I      = SPEED_KI * s_integral_a;
    pwm += SPEED_KP * error;
    pwm += SPEED_KI * s_integral_a;
    return pwm;
}

float PID_B(float Encoder, float Target)
{
    float error, pwm;

    if (Target <= 0.0f) {
        s_integral_b = 0.0f;
        return 0.0f;
    }

    error = Target - Encoder;
    s_integral_b += error * SPEED_DT_S;
    s_integral_b = PWM_Limit(s_integral_b, SPEED_INTEGRAL_MAX, -SPEED_INTEGRAL_MAX);

    pwm  = SPEED_PWM_FEEDFORWARD * Target;
    pwm += SPEED_KP * error;

    /* 存储 PID 分量供上位机调参 */
    PID_B_Target = Target;
    PID_B_Error  = error;
    PID_B_P      = SPEED_KP * error;
    PID_B_I      = SPEED_KI * s_integral_b;
    pwm += SPEED_KI * s_integral_b;
    return pwm;
}

/* ================================================================
 *  陀螺仪航向 P 控制器（flag=4 直行模式用）
 * ================================================================ */

float GYRO_Control(float now, float target)
{
    float bias = STRAIGHT_KP * normalize_angle(target - now);
    return PWM_Limit(bias, STRAIGHT_BIAS_MAX, -STRAIGHT_BIAS_MAX);
}

/* ================================================================
 *  工具函数
 * ================================================================ */

float PWM_Limit(float value, float maximum, float minimum)
{
    return limit_float(value, maximum, minimum);
}

float Control_Get_Angle(void)
{
    return s_control_angle;
}

float Control_Get_Bias(void)
{
    return s_control_bias;
}
