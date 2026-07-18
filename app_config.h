#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 设为1时进入灰度传感器校准程序，正常运行请保持为0。 */
#define APP_SENSOR_CALIBRATION_ENABLED      0

/* 上电后等待外设电源和信号稳定的时间。 */
#define APP_STARTUP_DELAY_MS              100U

/* 主控制循环周期：每5ms执行一次循迹或直行控制。 */
#define APP_CONTROL_PERIOD_MS               5U

/* 串口数据打印周期，过快打印会影响主循环实时性。 */
#define APP_PRINT_PERIOD_MS               100U

/* 循迹输出百分比换算为电机速度时的满量程，单位mm/s。 */
#define APP_LINE_SPEED_FULL_SCALE_MM_S   1000.0f

/* 丢线后使用MPU6050保持直行的基础速度，单位mm/s。 */
#define APP_STRAIGHT_SPEED_MM_S         250.0f

#endif
