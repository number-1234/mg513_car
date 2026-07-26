/**
 * @file    control.c
 * @brief   小车运动控制 — flag=1循迹 / 2停车 / 3转向 / 4直行
 *
 * flag=3: Turn_Run(要转的角度) — 每次传相同目标，到位返回true
 * flag=4: Straight_Run(目标角度, 前进速度) — 直接输出左右轮目标
 */
#include "control.h"
#include "angle_hold.h"
#include "Encoder/Encoder.h"
#include "GYRO/GYRO_JY901.h"
#include "Sensor/Sensor.h"
#include "sys/sys.h"

/* ── 参数 ── */
#define PWM_MAX              1000.0f
#define LINE_SPEED_MM_S       150.0f
#define LINE_MAX_SPEED_MM_S  250.0f
#define LINE_P                 10.0f   /* 循迹 P */
#define LINE_D                 20.0f   /* 循迹 D */
#define STRAIGHT_SPEED_MM_S  150.0f   /* 直行速度 */
#define STRAIGHT_P            10.0f   /* 直行角度 P */
#define STRAIGHT_D            10.0f   /* 直行角度 D */
#define SPEED_KP               0.25f
#define SPEED_KI               0.50f
#define SPEED_FEEDFORWARD      0.58f
#define SPEED_DT               0.10f
#define SPEED_INTEGRAL_MAX      400.0f

/* ── 全局 ── */
extern volatile int flag;
volatile int   Motor_Left, Motor_Right;
volatile float Turn_Zero_Yaw, Straight_Zero_Yaw;

/* ── 静态 ── */
static float s_i_l, s_i_r;
static float s_last_dev;

/* ================================================================
 *  Turn_Run(target_deg)
 *    传入目标角度（°），重复调用。到位返回 true。
 *    例: Turn_Run(Yaw + 90)  — 原地右转90°
 * ================================================================ */
static bool Turn_Run(float target_deg)
{
    static bool inited;
    if (!inited) { AngleHold_SetTarget(target_deg); inited = true; }

    AngleHold_Control();
    if (AngleHold_IsArrived(2.0f)) { inited = false; return true; }
    return false;
}

/* ================================================================
 *  Straight_Run(target_deg, speed, *ta, *tb)
 *    传入目标角度和前进速度，输出左右轮目标。
 *    例: Straight_Run(90, 100, &ta, &tb) — 保持90°方向、100mm/s直行
 * ================================================================ */
static void Straight_Run(float target_deg, float base_speed,
                         float *ta, float *tb)
{
    static float prev_yaw;
    float err   = normalize_angle(target_deg - Yaw);
    float delta = Yaw - prev_yaw;
    prev_yaw = Yaw;

    float steer = err * STRAIGHT_P - delta * STRAIGHT_D;
    steer = PWM_Limit(steer, base_speed, -base_speed);

    *ta = base_speed - steer;
    *tb = base_speed + steer;
}

/* ================================================================
 *  SpeedPI — 速度闭环
 * ================================================================ */
static float SpeedPI(float *integral, float target, float actual)
{
    if (target <= 0.0f) { *integral = 0.0f; return 0.0f; }
    float err = target - actual;
    *integral += err * SPEED_DT;
    *integral  = PWM_Limit(*integral, SPEED_INTEGRAL_MAX, -SPEED_INTEGRAL_MAX);
    return SPEED_FEEDFORWARD * target + SPEED_KP * err + SPEED_KI * (*integral);
}

/* ================================================================
 *  初始化
 * ================================================================ */
void Control_Init(void)
{
    Motor_Left = Motor_Right = 0;
    Turn_Zero_Yaw = Straight_Zero_Yaw = 0.0f;
    s_i_l = s_i_r = 0.0f;
    s_last_dev = 0.0f;
    Set_Pwm(0, 0);
}

/* ================================================================
 *  主控制 (每100ms)
 * ================================================================ */
void Control(void)
{
    float ta = 0, tb = 0;

    if (flag == 1) {
        float dev = (float)Incremental_Quantity();
        float bias = -dev * LINE_P - (dev - s_last_dev) * LINE_D;
        s_last_dev = dev;
        ta = PWM_Limit(LINE_SPEED_MM_S + bias, LINE_MAX_SPEED_MM_S, 0);
        tb = PWM_Limit(LINE_SPEED_MM_S - bias, LINE_MAX_SPEED_MM_S, 0);
    }
    else if (flag == 2) {
        /* 停车 */
    }
    else if (flag == 3) {
        if (Turn_Run(Yaw + 90)) flag = 4;   /* 转到位切直行 */
        return;
    }
    else if (flag == 4) {
        Straight_Run(0, STRAIGHT_SPEED_MM_S, &ta, &tb);
    }

    Motor_Left  = (int)PWM_Limit(SpeedPI(&s_i_l, ta, EncoderA_VEL), PWM_MAX, 0);
    Motor_Right = (int)PWM_Limit(SpeedPI(&s_i_r, tb, EncoderB_VEL), PWM_MAX, 0);
    Set_Pwm(Motor_Left, Motor_Right);
}

/* ================================================================
 *  工具 / 兼容旧接口
 * ================================================================ */
void Control_Stop(void) { s_i_l = s_i_r = 0; Set_Pwm(0, 0); }
float PID_A(float e, float t) { return SpeedPI(&s_i_l, t, e); }
float PID_B(float e, float t) { return SpeedPI(&s_i_r, t, e); }
float GYRO_Control(float n, float t) { return 0; }
float PWM_Limit(float v, float max, float min) { return limit_float(v, max, min); }
float Control_Get_Angle(void) { return 0; }
float Control_Get_Bias(void)  { return 0; }

/* ================================================================
 *  4路PWM
 * ================================================================ */
void Set_Pwm(int Left, int Right)
{
    Left  = (int)PWM_Limit(Left,  PWM_MAX, -PWM_MAX);
    Right = (int)PWM_Limit(Right, PWM_MAX, -PWM_MAX);

    if (Left > 0) {
        DL_Timer_setCaptureCompareValue(PWM_2_INST, (uint32_t)Left,  GPIO_PWM_2_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0,              GPIO_PWM_0_C0_IDX);
    } else if (Left < 0) {
        DL_Timer_setCaptureCompareValue(PWM_2_INST, 0,              GPIO_PWM_2_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)(-Left), GPIO_PWM_0_C0_IDX);
    } else {
        DL_Timer_setCaptureCompareValue(PWM_2_INST, (uint32_t)PWM_MAX, GPIO_PWM_2_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)PWM_MAX, GPIO_PWM_0_C0_IDX);
    }

    if (Right > 0) {
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)Right, GPIO_PWM_0_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_1_INST, 0,               GPIO_PWM_1_C1_IDX);
    } else if (Right < 0) {
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0,               GPIO_PWM_0_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_1_INST, (uint32_t)(-Right), GPIO_PWM_1_C1_IDX);
    } else {
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)PWM_MAX, GPIO_PWM_0_C1_IDX);
        DL_Timer_setCaptureCompareValue(PWM_1_INST, (uint32_t)PWM_MAX, GPIO_PWM_1_C1_IDX);
    }
}
