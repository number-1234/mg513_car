#include <stdbool.h>
#include <stdio.h>

#include "app_config.h"
#include "calibrate.h"
#include "car_control.h"
#include "delay.h"
#include "ti_msp_dl_config.h"
#include "Uart.h"

/* 读取车辆快照并发送一行结构化串口遥测。 */
static void send_telemetry(void)
{
    char buffer[192];
    car_telemetry_t telemetry;

    car_control_get_telemetry(&telemetry);
    (void)snprintf(
        buffer,
        sizeof(buffer),
        "Sensor:%02X Cmd:%u,%u Pulse:%ld,%ld Target:%.0f,%.0f "
        "Speed:%.0f,%.0f PWM:%u,%u\r\n",
        (unsigned int)telemetry.sensor_bits,
        (unsigned int)telemetry.left_command_percent,
        (unsigned int)telemetry.right_command_percent,
        (long)telemetry.left_motor.sampled_pulses,
        (long)telemetry.right_motor.sampled_pulses,
        telemetry.left_motor.target_speed_mm_s,
        telemetry.right_motor.target_speed_mm_s,
        telemetry.left_motor.measured_speed_mm_s,
        telemetry.right_motor.measured_speed_mm_s,
        (unsigned int)telemetry.left_motor.pwm,
        (unsigned int)telemetry.right_motor.pwm);
    uart_write_string(buffer);
}

int main(void)
{
    uint32_t last_telemetry_ms;

    /* 初始化 SysConfig 中配置的时钟、GPIO、ADC、PWM、UART 和定时器。 */
    SYSCFG_DL_init();

#if APP_SENSOR_CALIBRATION_ENABLED
    /* 校准函数完成后不会返回。 */
    sensor_calibration_run();
#endif

    /* 根据 app_config.h 进入电机测试模式或循迹模式。 */
    car_control_init();
    last_telemetry_ms = system_millis();

    while (true) {
        /* 车辆逻辑和串口遥测均在主循环中执行；速度 PI 在中断中运行。 */
        car_control_update();
        if ((uint32_t)(system_millis() - last_telemetry_ms) >=
            APP_TELEMETRY_PERIOD_MS) {
            last_telemetry_ms = system_millis();
            send_telemetry();
        }
        delay_ms(APP_CONTROL_PERIOD_MS);
    }
}
