/**
 * @file    calibrate.h
 * @brief   灰度传感器校准 — 通过UART输出黑白校准值
 *
 *          用法:
 *            1. 将传感器放在白色地面上 → 记录 WHITE 值
 *            2. 将传感器放在黑色线上   → 记录 BLACK 值
 *            3. 更新 main.c 中 white[] 和 black[] 数组
 *            4. 将 DO_CALIBRATION 改回 0, 重新编译
 */

#ifndef CALIBRATE_H
#define CALIBRATE_H

#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"

/* 是否进入校准模式: 1=校准, 0=正常循迹 */
#define DO_CALIBRATION  0

void Calibrate_Run(No_MCU_Sensor *sensor);

#endif /* CALIBRATE_H */
