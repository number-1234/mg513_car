/**
 * @file    jy901_i2c.h
 * @brief   JY901S I2C 读写封装 — 复用 MPU6050 的硬件 I2C 驱动
 *
 * 将 bsp_mpu6050.c 的 MPU6050_WriteReg / MPU6050_ReadData
 * 封装为 WIT SDK 所需的 WitI2cWrite / WitI2cRead 回调。
 */
#ifndef GYRO_JY901_I2C_H
#define GYRO_JY901_I2C_H

#include <stdint.h>

/**
 * @brief  I2C 写：start → addr(W) → reg → data[0..len-1] → stop
 * @return 1=成功, 0=失败（适配 WIT SDK 返回值约定）
 */
int32_t JY901_I2cWrite(uint8_t ucAddr, uint8_t ucReg,
                       uint8_t *p_ucVal, uint32_t uiLen);

/**
 * @brief  I2C 读：start → addr(W) → reg → restart → addr(R) → data[0..len-1] → stop
 * @return 1=成功, 0=失败
 */
int32_t JY901_I2cRead(uint8_t ucAddr, uint8_t ucReg,
                      uint8_t *p_ucVal, uint32_t uiLen);

#endif
