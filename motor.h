#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum {
    MOTOR_LEFT = 0,  /* 左轮 */
    MOTOR_RIGHT,    /* 右轮 */
    MOTOR_COUNT     /* 电机数量，仅用于数组长度和遍历 */
} motor_id_t;

typedef enum {
    MOTOR_STOP = 0, /* 刹车停止 */
    MOTOR_FORWARD,  /* 前进 */
    MOTOR_BACKWARD  /* 后退 */
} motor_direction_t;

/* 单个电机的只读运行数据，供主程序串口输出。 */
typedef struct {
    int32_t sampled_pulses;     /* 最近 100 ms 内的编码器脉冲数 */
    float target_speed_mm_s;    /* 目标速度，单位 mm/s */
    float measured_speed_mm_s;  /* 实测速度，单位 mm/s */
    uint16_t pwm;               /* 当前 PWM，范围 0～1000 */
} motor_telemetry_t;

/* 初始化电机 GPIO、PWM、编码器中断和速度闭环定时器。 */
void motor_init(void);

/* 设置指定电机的方向。 */
void motor_set_direction(motor_id_t motor, motor_direction_t direction);

/* 直接设置指定电机的 PWM，输入会被限制在 0～1000。 */
void motor_set_pwm(motor_id_t motor, uint16_t pwm);

/* 设置速度闭环目标；小于等于 0 时控制器输出 0。 */
void motor_set_speed_target(motor_id_t motor, float speed_mm_s);

/* 清空两侧控制器状态并立即刹车。 */
void motor_stop_all(void);

/* 复制指定电机的运行快照，不暴露模块内部状态。 */
void motor_get_telemetry(motor_id_t motor, motor_telemetry_t *telemetry);

#endif
