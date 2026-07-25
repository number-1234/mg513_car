#ifndef SYS_SYS_H
#define SYS_SYS_H

#include <stdint.h>
#include <stdio.h>

#include "ti_msp_dl_config.h"

/*
 * 4-PWM 电机控制方案（3 个定时器，4 个 PWM 通道）：
 *   PWM_0 (TIMG6)  → PA21(C0=A_IN1 左前进), PA22(C1=B_IN1 右前进)
 *   PWM_1 (TIMA1)  → PA24(C1=B_IN2 右后退)
 *   PWM_2 (TIMG12) → PA25(C1=A_IN2 左后退)
 *
 * 电机 TB6612 / L298N 等 H 桥驱动：
 *   前进：Forward 通道 PWM，Reverse 通道 0
 *   后退：Forward 通道 0，Reverse 通道 PWM
 *   刹车：两通道均 100% duty → IN1=H, IN2=H（短路制动）
 */

void delay_ms(uint32_t milliseconds);
void delay_us(uint32_t microseconds);
uint32_t system_millis(void);

float limit_float(float value, float maximum, float minimum);
int myabs(int value);
float normalize_angle(float angle);

#endif
