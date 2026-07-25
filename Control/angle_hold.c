/**
 * @file    angle_hold.c
 * @brief   Yaw 角度保持 — PD 差速转向
 *
 * 控制律：PWM = error × KP  -  yaw_delta × KD
 *   error     = normalize_angle(target - Yaw)
 *   yaw_delta = Yaw - 上次Yaw（角速度）
 *
 * 差速输出：左轮 +PWM，右轮 -PWM，产生旋转力矩
 *
 * 可调参数：
 *   ANGLE_KP  — 回正力度（越大越快，太大振荡）
 *   ANGLE_KD  — 阻尼（越大越稳，太大迟钝）
 *   PWM_DEADBAND — 死区（低于此值不转，避免抖动）
 *   PWM_MAX_ANGLE — 上限
 */
#include "angle_hold.h"

#include "control.h"          /* Set_Pwm, PWM_Limit */
#include "GYRO/GYRO_JY901.h"  /* Yaw */
#include "sys/sys.h"          /* normalize_angle */

/* ── 可调参数 ── */
#define ANGLE_KP       15.0f
#define ANGLE_KD       7.0f
#define PWM_DEADBAND   40
#define PWM_MAX_ANGLE  500

/* ── 内部状态 ── */
static float s_target_yaw;
static float s_last_yaw;
static bool  s_inited;

/* ================================================================
 *  AngleHold_Init — 记录当前 Yaw 为目标角度
 * ================================================================ */
void AngleHold_Init(void)
{
    s_target_yaw = Yaw;
    s_last_yaw   = Yaw;
    s_inited     = true;
}

/* ================================================================
 *  AngleHold_SetTarget — 修改目标角度
 * ================================================================ */
void AngleHold_SetTarget(float target_deg)
{
    s_target_yaw = target_deg;
}

/* ================================================================
 *  AngleHold_Control — 角度闭环（每 100ms 调用一次）
 * ================================================================ */
void AngleHold_Control(void)
{
    float error, yaw_delta;
    int   pwm_out;

    if (!GYRO_Is_Ready()) {
        Set_Pwm(0, 0);
        return;
    }

    error     = normalize_angle(s_target_yaw - Yaw);
    yaw_delta = Yaw - s_last_yaw;
    s_last_yaw = Yaw;

    /* PD 控制 */
    pwm_out = (int)(error * ANGLE_KP - yaw_delta * ANGLE_KD);

    /* 死区 */
    if (pwm_out > 0 && pwm_out < PWM_DEADBAND)   pwm_out = PWM_DEADBAND;
    if (pwm_out < 0 && pwm_out > -PWM_DEADBAND)  pwm_out = -PWM_DEADBAND;

    /* 限幅 */
    pwm_out = (int)PWM_Limit((float)pwm_out, PWM_MAX_ANGLE, -PWM_MAX_ANGLE);

    /* 差速输出：左-, 右+ → 右转  (电机接线验证) */
    Set_Pwm(-pwm_out, pwm_out);
}

/* ================================================================
 *  AngleHold_IsArrived — 是否已到达目标（容差内且角速度小）
 * ================================================================ */
bool AngleHold_IsArrived(float tolerance)
{
    float error = normalize_angle(s_target_yaw - Yaw);
    return (error < tolerance) && (error > -tolerance);
}
