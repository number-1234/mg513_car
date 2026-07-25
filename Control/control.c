/**
 * @file    control.c
 * @brief   小车运动控制核心模块
 *
 * flag=1 循迹 / flag=2 停车 / flag=3 角度环转45° / flag=4 保持角度直行
 */
#include "control.h"
#include "angle_hold.h"
#include "Encoder/Encoder.h"
#include "GYRO/GYRO_JY901.h"
#include "Sensor/Sensor.h"
#include "sys/sys.h"

/* ── PWM ── */
#define PWM_MAX              1000.0f

/* ── 循迹 ── */
#define LINE_SPEED_MM_S       80.0f
#define LINE_MAX_SPEED_MM_S  150.0f
#define LINE_WEIGHT_TO_SPEED   3.0f   /* P */
#define LINE_KD                2.5f   /* D */

/* ── 角度环直行 (flag=4) ── */
#define STRAIGHT_SPEED_MM_S  100.0f
#define ANGLE_KP              10.0f   /* 角度 P (同 angle_hold) */
#define ANGLE_KD              10.0f   /* 角度 D (同 angle_hold) */

/* ── 速度 PI ── */
#define SPEED_PWM_FEEDFORWARD  0.58f
#define SPEED_KP               0.25f
#define SPEED_KI               0.50f
#define SPEED_DT_S             0.10f
#define SPEED_INTEGRAL_MAX      400.0f

/* ── 全局 ── */
extern volatile int flag;
volatile int   Motor_Left, Motor_Right;
volatile float Turn_Zero_Yaw, Straight_Zero_Yaw;

/* ── 静态 ── */
static float s_integral_a, s_integral_b;
static float s_control_angle, s_control_bias;
static float s_last_deviation;
static float s_last_yaw;           /* angle hold 用 */
static float s_angle_target;       /* 角度环目标 */
static bool  s_angle_inited;       /* 角度环是否已设目标 */

/* ================================================================
 *  初始化
 * ================================================================ */
void Control_Init(void)
{
    Motor_Left = 0;  Motor_Right = 0;
    Turn_Zero_Yaw = 0.0f;  Straight_Zero_Yaw = 0.0f;
    s_integral_a = 0.0f;  s_integral_b = 0.0f;
    s_control_angle = 0.0f;  s_control_bias = 0.0f;
    s_last_deviation = 0.0f;  s_last_yaw = 0.0f;
    s_angle_inited = false;
    AngleHold_Init();
    Set_Pwm(0, 0);
}
uint8_t beep_flag=0;
/* ================================================================
 *  主控制
 * ================================================================ */
void Control(void)
{
    float TargetA = 0.0f, TargetB = 0.0f, bias = 0.0f;

    /* ── flag=1 循迹 ── */
    if (flag == 1) {
        s_angle_inited = false;
        AngleHold_Init();
        float dev = (float)Incremental_Quantity();
        bias  = -dev * LINE_WEIGHT_TO_SPEED;
        bias += -(dev - s_last_deviation) * LINE_KD;
        s_last_deviation = dev;

        TargetA = LINE_SPEED_MM_S + bias;
        TargetB = LINE_SPEED_MM_S - bias;
        TargetA = PWM_Limit(TargetA, LINE_MAX_SPEED_MM_S, 0.0f);
        TargetB = PWM_Limit(TargetB, LINE_MAX_SPEED_MM_S, 0.0f);
        s_control_angle = 0.0f;
        s_control_bias  = bias;
    }

    /* ── flag=2 停车 ── */
    else if (flag == 2) {

    }

    /* ── flag=3 角度环原地转 45° ── */
    else if (flag == 3) {
        if (!s_angle_inited) {
            s_angle_target = Yaw+45  ;   /* 目标 = 当前 + 45° */
            s_last_yaw = Yaw;
            s_angle_inited = true;
        }
        AngleHold_SetTarget(s_angle_target);
        AngleHold_Control();                 /* 原地旋转 */
        // if(beep_flag==0)
        // {
        //     beep_10ms();
        //     beep_flag++;
        // }
        s_control_angle = Yaw;
       
        return;
    }

    /* ── flag=4 保持角度 + 直行 ── */
    else if (flag == 4) {
        /* 角度 PD：算出差速偏置 */
        float a_err   = normalize_angle(s_angle_target - Yaw);
        float a_delta = Yaw - s_last_yaw;
        s_last_yaw = Yaw;
        float steer = a_err * ANGLE_KP - a_delta * ANGLE_KD;
        steer = PWM_Limit(steer, STRAIGHT_SPEED_MM_S, -STRAIGHT_SPEED_MM_S);

        s_control_angle = a_err;
        s_control_bias  = steer;

        TargetA = STRAIGHT_SPEED_MM_S - steer;
        TargetB = STRAIGHT_SPEED_MM_S + steer;
        TargetA = PWM_Limit(TargetA, LINE_MAX_SPEED_MM_S, 30.0f);
        TargetB = PWM_Limit(TargetB, LINE_MAX_SPEED_MM_S, 30.0f);
    }

    /* ── 未定义 ── */
    else {
        s_angle_inited = false;
    AngleHold_Init();
        TargetA = 0.0f;  TargetB = 0.0f;
    }

    /* ── 速度闭环 ── */
    Motor_Left  = (int)PWM_Limit(PID_A(EncoderA_VEL, TargetA), PWM_MAX, 0.0f);
    Motor_Right = (int)PWM_Limit(PID_B(EncoderB_VEL, TargetB), PWM_MAX, 0.0f);
    Set_Pwm(Motor_Left, Motor_Right);
}

/* ================================================================
 *  急停
 * ================================================================ */
void Control_Stop(void)
{
    s_integral_a = 0.0f;  s_integral_b = 0.0f;
    Motor_Left = 0;  Motor_Right = 0;
    Set_Pwm(0, 0);
}

/* ================================================================
 *  4 路 PWM 输出
 * ================================================================ */
void Set_Pwm(int Left, int Right)
{
    Left  = (int)PWM_Limit((float)Left,  PWM_MAX, -PWM_MAX);
    Right = (int)PWM_Limit((float)Right, PWM_MAX, -PWM_MAX);

    if (Left > 0) {
        DL_Timer_setCaptureCompareValue(PWM_2_INST, (uint32_t)Left,  GPIO_PWM_2_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0,               GPIO_PWM_0_C0_IDX);
    } else if (Left < 0) {
        DL_Timer_setCaptureCompareValue(PWM_2_INST, 0,               GPIO_PWM_2_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)myabs(Left), GPIO_PWM_0_C0_IDX);
    } else {
        DL_Timer_setCaptureCompareValue(PWM_2_INST, (uint32_t)PWM_MAX, GPIO_PWM_2_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)PWM_MAX, GPIO_PWM_0_C0_IDX);
    }

    if (Right > 0) {
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)Right, GPIO_PWM_0_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_1_INST, 0,               GPIO_PWM_1_C1_IDX);
    } else if (Right < 0) {
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0,               GPIO_PWM_0_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_1_INST, (uint32_t)myabs(Right), GPIO_PWM_1_C1_IDX);
    } else {
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)PWM_MAX, GPIO_PWM_0_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_1_INST, (uint32_t)PWM_MAX, GPIO_PWM_1_C1_IDX);
    }
}

/* ================================================================
 *  速度 PI
 * ================================================================ */
float PID_A(float Encoder, float Target)
{
    if (Target <= 0.0f) { s_integral_a = 0.0f; return 0.0f; }
    float error = Target - Encoder;
    s_integral_a += error * SPEED_DT_S;
    s_integral_a = PWM_Limit(s_integral_a, SPEED_INTEGRAL_MAX, -SPEED_INTEGRAL_MAX);
    return SPEED_PWM_FEEDFORWARD * Target + SPEED_KP * error + SPEED_KI * s_integral_a;
}

float PID_B(float Encoder, float Target)
{
    if (Target <= 0.0f) { s_integral_b = 0.0f; return 0.0f; }
    float error = Target - Encoder;
    s_integral_b += error * SPEED_DT_S;
    s_integral_b = PWM_Limit(s_integral_b, SPEED_INTEGRAL_MAX, -SPEED_INTEGRAL_MAX);
    return SPEED_PWM_FEEDFORWARD * Target + SPEED_KP * error + SPEED_KI * s_integral_b;
}

/* ================================================================
 *  工具
 * ================================================================ */
float GYRO_Control(float now, float target)
{
    float b = STRAIGHT_SPEED_MM_S * 0.002f;  /* fallback，实际不再使用 */
    return PWM_Limit(b, 150.0f, -150.0f);
}
void beep_10ms()
{
    DL_GPIO_clearPins(GPIO_GRP_0_PORT,GPIO_GRP_0_BEEP_PIN);
    delay_ms(10);
    DL_GPIO_setPins(GPIO_GRP_0_PORT, GPIO_GRP_0_BEEP_PIN);
}
float PWM_Limit(float v, float max, float min) { return limit_float(v, max, min); }
float Control_Get_Angle(void) { return s_control_angle; }
float Control_Get_Bias(void)  { return s_control_bias; }
