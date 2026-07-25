/**
 * @file    angle_hold.h
 * @brief   Yaw 角度保持控制器 — 差速转向锁定车身朝向
 *
 * 调用 AngleHold_Init() 记录目标角度，
 * 每 100ms 调用一次 AngleHold_Control() 输出差速 PWM。
 */
#ifndef CONTROL_ANGLE_HOLD_H
#define CONTROL_ANGLE_HOLD_H

#include <stdbool.h>

void AngleHold_Init(void);                     /* 记录当前 Yaw 为目标角度 */
void AngleHold_Control(void);                  /* 角度闭环控制，调用 Set_Pwm */

void AngleHold_SetTarget(float target_deg);    /* 设定目标角度（默认0） */
bool AngleHold_IsArrived(float tolerance);     /* 是否到达目标（tolerance°内） */

#endif
