#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 应用运行模式 ------------------------------------------------------------ */

/* 设为 1：进入灰度传感器黑白校准；设为 0：正常运行。 */
#define APP_SENSOR_CALIBRATION_ENABLED  0

/* 设为 1：关闭循迹，只测试左右轮速度闭环。 */
#define APP_MOTOR_TEST_MODE             0

/* 电机闭环测试参数。 */
#define APP_MOTOR_TEST_TARGET_MM_S      500.0f
#define APP_MOTOR_TEST_INITIAL_PWM      300U

/* 主程序运行参数。 */
#define APP_STARTUP_DELAY_MS            100U
#define APP_CONTROL_PERIOD_MS              5U
#define APP_TELEMETRY_PERIOD_MS         100U

/* 循迹模块输出的百分比，按照此满量程换算为 mm/s。 */
#define APP_LINE_SPEED_FULL_SCALE_MM_S  1000.0f

#endif
