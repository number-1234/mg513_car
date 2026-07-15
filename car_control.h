#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdint.h>

#include "motor.h"

/* 小车对外遥测数据，由左右两个电机快照组成。 */
typedef struct {
    motor_telemetry_t left_motor;
    motor_telemetry_t right_motor;
    uint8_t sensor_bits;
    uint8_t left_command_percent;
    uint8_t right_command_percent;
} car_telemetry_t;

/* 根据 app_config.h 中的模式初始化小车。 */
void car_control_init(void);

/* 主循环周期调用；循迹模式下读取传感器并更新速度目标。 */
void car_control_update(void);

/* 获取当前左右轮运行快照。 */
void car_control_get_telemetry(car_telemetry_t *telemetry);

#endif
