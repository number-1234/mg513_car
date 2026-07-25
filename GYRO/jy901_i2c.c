/**
 * @file    jy901_i2c.c
 * @brief   JY901S I2C 读写 — 桥接 MPU6050 硬件 I2C 驱动到 WIT SDK 回调
 *
 * 复用 Sensor/mpu6050/src/bsp_mpu6050.c 的硬件 I2C（I2C0, PA0/PA1, 400kHz）。
 * MPU6050_WriteReg 返回 0=成功，WIT SDK 要求 1=成功，此处做适配。
 */
#include "jy901_i2c.h"

/* bsp_mpu6050 提供的硬件 I2C 读写函数 */
char MPU6050_WriteReg(uint8_t addr, uint8_t regaddr,
                      uint8_t num, uint8_t *regdata);
char MPU6050_ReadData(uint8_t addr, uint8_t regaddr,
                      uint8_t num, uint8_t *Read);

/* ── WIT SDK I2C 写回调 ── */
int32_t JY901_I2cWrite(uint8_t ucAddr, uint8_t ucReg,
                       uint8_t *p_ucVal, uint32_t uiLen)
{
    /* WIT SDK 传入 ucAddr 已左移 1 位（8-bit），
     * MPU6050_WriteReg 期望 7-bit 地址，右移还原 */
    uint8_t dev_7bit = ucAddr >> 1;

    /* MPU6050_WriteReg: 0=成功, 非0=失败 → WIT SDK: 1=成功, 0=失败 */
    if (MPU6050_WriteReg(dev_7bit, ucReg, (uint8_t)uiLen, p_ucVal) == 0) {
        return 1;
    }
    return 0;
}

/* ── WIT SDK I2C 读回调 ── */
int32_t JY901_I2cRead(uint8_t ucAddr, uint8_t ucReg,
                      uint8_t *p_ucVal, uint32_t uiLen)
{
    uint8_t dev_7bit = ucAddr >> 1;

    if (MPU6050_ReadData(dev_7bit, ucReg, (uint8_t)uiLen, p_ucVal) == 0) {
        return 1;
    }
    return 0;
}
