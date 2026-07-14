/**
 * @file    motor.c
 * @brief   双电机驱动模块实现
 *          芯片: MSPM0G3507  主频: 32MHz
 *
 *          左电机A: AIN1=PA26 AIN2=PA25 PWM=PA21 (TIMA0 CCP0)
 *          右电机B: BIN1=PA24 BIN2=PA23 PWM=PA22 (TIMA0 CCP1)
 *          使能:    SBYT=PA2
 *
 *          PWM 参数 (SysConfig: PWM_MOTOR / TIMA0):
 *            - 模式: 边沿对齐向上计数 (EDGE_ALIGN_UP)
 *            - 时钟: BUSCLK 32MHz, 不分频
 *            - 周期: 1000 → 频率 32KHz
 *            - CC0 (PA21): 左电机A 速度控制
 *            - CC1 (PA22): 右电机B 速度控制
 */

#include "motor.h"

/* ========================= 内部变量 ========================= */
static uint8_t g_motorASpeed = 0;   /* 左电机当前速度 (%) */
static uint8_t g_motorBSpeed = 0;   /* 右电机当前速度 (%) */

/* ========================= 公共函数 ========================= */

/**
 * @brief 电机模块初始化
 * @note  GPIO 方向引脚和 PWM 已由 SYSCFG_DL_GPIO_init() /
 *        SYSCFG_DL_PWM_MOTOR_init() 完成配置，这里只需使能驱动芯片。
 *        必须确保 SYSCFG_DL_init() 先于本函数调用。
 */
void Motor_Init(void)
{
    /* 拉高 SBYT, 使能电机驱动芯片 (TB6612 / L298N 等) */
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_SBYT_PIN);

    /* 确保所有电机初始为停止状态 */
    Motor_StopAll();
}

/* ========================= 单电机控制 ========================= */

/**
 * @brief 设置单个电机速度 (PWM 占空比)
 * @param motor MOTOR_A (左) / MOTOR_B (右)
 * @param speed 速度百分比 0~100
 */
void Motor_SetSpeed(uint8_t motor, uint8_t speed)
{
    uint32_t duty;
    DL_TIMER_CC_INDEX cc_index;

    /* 限幅 */
    if (speed > 100) {
        speed = 100;
    }

    /* 百分比 → 占空比计数值 (period=1000) */
    duty = (uint32_t)speed * MOTOR_PWM_PERIOD / 100;

    /* 选择 PWM 通道 */
    if (motor == MOTOR_A) {
        cc_index = GPIO_PWM_MOTOR_C0_IDX;   /* CC0 → PA21 → 左A */
        g_motorASpeed = speed;
    } else {
        cc_index = GPIO_PWM_MOTOR_C1_IDX;   /* CC1 → PA22 → 右B */
        g_motorBSpeed = speed;
    }

    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, duty, cc_index);
}

/**
 * @brief 设置单个电机方向
 * @param motor     MOTOR_A (左) / MOTOR_B (右)
 * @param direction MOTOR_FORWARD / MOTOR_BACKWARD / MOTOR_STOP
 *
 *        方向逻辑:
 *          FORWARD  → IN1=H, IN2=L
 *          BACKWARD → IN1=L, IN2=H
 *          STOP     → IN1=L, IN2=L (刹车)
 */
void Motor_SetDirection(uint8_t motor, uint8_t direction)
{
    uint32_t in1_pin, in2_pin;

    if (motor == MOTOR_A) {
        in1_pin = MOTOR_AIN1_PIN;   /* PA26 */
        in2_pin = MOTOR_AIN2_PIN;   /* PA25 */
    } else {
        in1_pin = MOTOR_BIN1_PIN;   /* PA24 */
        in2_pin = MOTOR_BIN2_PIN;   /* PA23 */
        /* B电机接线极性相反, 交换正反转 */
        direction = (direction == MOTOR_FORWARD)  ? MOTOR_BACKWARD :
                    (direction == MOTOR_BACKWARD) ? MOTOR_FORWARD :
                                                    MOTOR_STOP;
    }

    switch (direction) {
    case MOTOR_FORWARD:
        DL_GPIO_setPins(MOTOR_PORT, in1_pin);
        DL_GPIO_clearPins(MOTOR_PORT, in2_pin);
        break;

    case MOTOR_BACKWARD:
        DL_GPIO_clearPins(MOTOR_PORT, in1_pin);
        DL_GPIO_setPins(MOTOR_PORT, in2_pin);
        break;

    case MOTOR_STOP:
    default:
        DL_GPIO_clearPins(MOTOR_PORT, in1_pin | in2_pin);
        break;
    }
}

/**
 * @brief 控制单个电机 (方向 + 速度)
 */
void Motor_Run(uint8_t motor, uint8_t direction, uint8_t speed)
{
    Motor_SetDirection(motor, direction);
    Motor_SetSpeed(motor, speed);
}

/**
 * @brief 停止单个电机 (速度归零 + 刹车)
 */
void Motor_Stop(uint8_t motor)
{
    Motor_SetSpeed(motor, 0);
    Motor_SetDirection(motor, MOTOR_STOP);
}

/**
 * @brief 停止所有电机
 */
void Motor_StopAll(void)
{
    Motor_Stop(MOTOR_A);
    Motor_Stop(MOTOR_B);
}

/* ========================= 小车高级控制 ========================= */

/**
 * @brief 小车前进
 */
void Car_Forward(uint8_t speed)
{
    Motor_Run(MOTOR_A, MOTOR_FORWARD, speed);
    Motor_Run(MOTOR_B, MOTOR_FORWARD, speed);
}

/**
 * @brief 小车后退
 */
void Car_Backward(uint8_t speed)
{
    Motor_Run(MOTOR_A, MOTOR_BACKWARD, speed);
    Motor_Run(MOTOR_B, MOTOR_BACKWARD, speed);
}

/**
 * @brief 小车原地左转 (左后退 + 右前进)
 */
void Car_TurnLeft(uint8_t speed)
{
    Motor_Run(MOTOR_A, MOTOR_BACKWARD, speed);
    Motor_Run(MOTOR_B, MOTOR_FORWARD,  speed);
}

/**
 * @brief 小车原地右转 (左前进 + 右后退)
 */
void Car_TurnRight(uint8_t speed)
{
    Motor_Run(MOTOR_A, MOTOR_FORWARD,  speed);
    Motor_Run(MOTOR_B, MOTOR_BACKWARD, speed);
}

/**
 * @brief 小车停止
 */
void Car_Stop(void)
{
    Motor_StopAll();
}
