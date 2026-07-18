#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* 电机编号，用于指定左轮或右轮。 */
typedef enum {
    MOTOR_LEFT = 0, /* 左轮，下标为0 */
    MOTOR_RIGHT,    /* 右轮，下标为1 */
    MOTOR_COUNT     /* 电机数量，只用于内部数组长度和循环 */
} motor_id_t;

/* 电机运动方向。 */
typedef enum {
    MOTOR_STOP = 0, /* 刹车停止 */
    MOTOR_FORWARD,  /* 前进 */
    MOTOR_BACKWARD  /* 后退 */
} motor_direction_t;

/* 初始化电机GPIO、PWM、编码器中断和100ms速度环定时器。 */
void motor_init(void);

/* 以下是底层单电机控制接口。 */
void motor_set_direction(motor_id_t motor, motor_direction_t direction);
void motor_set_pwm(motor_id_t motor, uint16_t pwm);            /* PWM范围0~1000 */
void motor_set_speed_target(motor_id_t motor, float speed_mm_s);/* 目标速度mm/s */

/* 最常用接口：让左右轮同时前进，并分别指定目标速度。 */
void motor_drive_forward(float left_speed_mm_s, float right_speed_mm_s);

/* 立即清空速度环状态、PWM和目标速度，并让两侧电机刹车。 */
void motor_stop_all(void);

/* 直接读取左右轮数据，不需要创建或传递结构体。 */
float motor_get_left_speed(void);          /* 左轮实测速度，mm/s */
float motor_get_right_speed(void);         /* 右轮实测速度，mm/s */
float motor_get_left_target_speed(void);   /* 左轮目标速度，mm/s */
float motor_get_right_target_speed(void);  /* 右轮目标速度，mm/s */
uint16_t motor_get_left_pwm(void);         /* 左轮当前PWM，0~1000 */
uint16_t motor_get_right_pwm(void);        /* 右轮当前PWM，0~1000 */

#endif
