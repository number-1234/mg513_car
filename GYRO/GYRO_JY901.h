/**
 * @file    GYRO_JY901.h
 * @brief   JY901S 陀螺仪模块 — 替代 GYRO.h（MPU6050 版本）
 *
 * API 与 GYRO.h 完全一致，可直接替换使用。
 * 底层通过 WIT SDK + 硬件 I2C（复用 bsp_mpu6050）与 JY901S 通信。
 */
#ifndef GYRO_GYRO_JY901_H
#define GYRO_GYRO_JY901_H

#include <stdbool.h>
#include <stdint.h>

/* 姿态角，只读全局量，供控制模块使用 */
extern float Pitch;
extern float Roll;
extern float Yaw;

uint8_t GYRO_Init(void);      /* 初始化 JY901S + WIT SDK，返回 0=成功 */
bool    GYRO_Update(void);    /* 读取最新姿态角，返回 true=有新数据 */
bool    GYRO_Is_Ready(void);  /* 传感器是否就绪 */

#endif
