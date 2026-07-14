/**
 * @file    motor.h
 * @brief   双电机驱动模块 — 基于 SysConfig 配置
 *          芯片: MSPM0G3507  主频: 32MHz
 *
 *          左电机A: AIN1=PA26 AIN2=PA25 PWM=PA21 (TIMA0 CCP0)
 *          右电机B: BIN1=PA24 BIN2=PA23 PWM=PA22 (TIMA0 CCP1)
 *          使能:    SBYT=PA2
 *
 *          ⚠ GPIO 和 PWM 已由 SysConfig (SYSCFG_DL_init) 完成初始化，
 *          本模块仅负责运行控制，不再重复初始化外设。
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "ti_msp_dl_config.h"

/* ========================= 电机选择 ========================= */
#define MOTOR_A     0       /* 左电机 */
#define MOTOR_B     1       /* 右电机 */

/* ========================= 电机方向 ========================= */
#define MOTOR_FORWARD    0  /* 正转 (前进) */
#define MOTOR_BACKWARD   1  /* 反转 (后退) */
#define MOTOR_STOP       2  /* 停止 (刹车) */

/* ========================= PWM 参数 (来自 SysConfig) ========================= */
#define MOTOR_PWM_PERIOD        1000U   /* PWM 周期 (SysConfig: period=1000) */
#define MOTOR_PWM_FREQ          32000U  /* PWM 频率 = 32MHz / 1000 = 32KHz */

/* ========================= 函数声明 ========================= */

void Motor_Init(void);

void Motor_SetSpeed(uint8_t motor, uint8_t speed);
void Motor_SetDirection(uint8_t motor, uint8_t direction);
void Motor_Run(uint8_t motor, uint8_t direction, uint8_t speed);

void Motor_Stop(uint8_t motor);
void Motor_StopAll(void);

/* 小车高级控制 */
void Car_Forward(uint8_t speed);
void Car_Backward(uint8_t speed);
void Car_TurnLeft(uint8_t speed);
void Car_TurnRight(uint8_t speed);
void Car_Stop(void);

#endif /* MOTOR_H */
