#ifndef __IMU_DRIVER_H__
#define __IMU_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

// 解析后的 IMU 数据结构体
typedef struct {
    float acc[3];    // Ax, Ay, Az (单位: g)
    float gyro[3];   // Wx, Wy, Wz (单位: °/s)
    float angle[3];  // Roll, Pitch, Yaw (单位: °)
    float quat[4];   // q0, q1, q2, q3
} IMU_Data_t;

extern volatile IMU_Data_t g_imu_data;

// 函数声明
void IMU_Init(void);
void IMU_SendCmd(uint8_t addr, uint16_t data);
void IMU_SetYawZero(void);
void IMU_SetBaudrate(uint16_t baud_code);
void IMU_SetRate(uint16_t rate_code);

#endif /* __IMU_DRIVER_H__ */
