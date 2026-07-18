/*
 * 主程序只负责组合各功能，不在这里实现循迹算法或直行PID。
 *
 * 程序流程：
 * 1. 初始化系统、电机、灰度循迹模块和MPU6050直行模块；
 * 2. 检测到黑线时执行循迹；
 * 3. 丢线时把当前航向角作为0度，然后保持这个方向直行；
 * 4. 再次找到黑线后恢复循迹，并等待下一次丢线重新定零。
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_config.h"
#include "calibrate.h"
#include "delay.h"
#include "line_tracking.h"
#include "motor.h"
#include "straight_drive.h"
#include "ti_msp_dl_config.h"
#include "Uart.h"
static bool turn_active = false;
static float turn_zero_yaw = 0.0f;
int angle=-55;
/* 通过串口打印MPU6050/DMP初始化结果。 */
static void print_mpu_init_result(uint8_t status)
{
    char buffer[48];

    /* mpu_dmp_init()返回0表示所有初始化步骤都成功。 */
    if (status == 0U) {
        uart_write_string("MPU6050 init OK\r\n");
        return;
    }

    /* 非0错误码可用于判断DMP初始化失败在哪个步骤。 */
    (void)snprintf(buffer, sizeof(buffer),
                   "MPU6050 init failed: %u\r\n",
                   (unsigned int)status);
    uart_write_string(buffer);
}

/* 丢线期间打印当前角度和左右轮实测速度。 */
static void print_no_line_data(void)
{
    char buffer[160];

    (void)snprintf(
        buffer,
        sizeof(buffer),
        "NoLine Sensor:%02X Angle:%.2f L:%.1f R:%.1f\r\n",
        (unsigned int)line_tracking_get_sensor_bits(), /* 8路数字量 */
        (double)straight_drive_get_angle(),            /* 相对本次参考零点的角度 */
        (double)motor_get_left_speed(),                /* 左轮mm/s */
        (double)motor_get_right_speed());               /* 右轮mm/s */
    uart_write_string(buffer);
}
int flat=0;
int main(void)
{
    uint8_t mpu_status;       /* MPU6050/DMP初始化返回值 */
    uint32_t last_print_ms;   /* 上一次打印串口数据的时间 */
    bool line_found;          /* true=检测到线，false=丢线 */

    /* 初始化SysConfig生成的时钟、GPIO、定时器、ADC、串口和硬件I2C。 */
    SYSCFG_DL_init();

    /* 给外设上电后留出稳定时间。 */
    delay_ms(APP_STARTUP_DELAY_MS);

#if 0
    /* 校准模式会一直停留在校准程序中，不会进入下面的正常控制。 */
    sensor_calibration_run();
#endif

    /* 各个功能独立初始化，调用关系清晰且互不嵌套。 */
    motor_init();
    line_tracking_init();
    mpu_status = straight_drive_init();
    print_mpu_init_result(mpu_status);

    last_print_ms = system_millis();

    while (true) {
        /*
         * line_tracking_run()会读取灰度传感器：
         * - 找到线：函数内部已经根据循迹结果设置左右轮速度；
         * - 没找到线：函数返回false，不再设置电机速度。
         */
        line_found = line_tracking_run();
  
        if (line_found) {
            /* Re-arm the one-shot stop for the next line-loss event. */
            flat = 1;
            /*
             * 正在循迹时清除上一次直行状态，使下一次丢线重新记0度。
             * 同时持续读取DMP，避免长时间循迹造成FIFO溢出，并保存最新yaw。
             */
            straight_drive_reset();
            straight_drive_update_sensor();
            
        } else {
            if (flat) {
            motor_stop_all();
            delay_ms(1000);
            straight_drive_update_sensor();
            turn_zero_yaw = straight_drive_get_yaw();
            turn_active = true;
            flat = 0;
            }
           if (turn_active) 
           {
            /* 相对参考零点转45°，转速150mm/s。 */
           if (straight_drive_turn_to_angle(turn_zero_yaw, -angle, 150.0f))
            {
             turn_active = false; /* 已到达目标角度 */
             }
           }
            else 
            {
             /* 转向完成后执行后续直行。 */
             straight_drive_run(APP_STRAIGHT_SPEED_MM_S);
            }
        }

        /* 没有检测到黑线时，每APP_PRINT_PERIOD_MS打印一次角度和轮速。 */
        if ((uint32_t)(system_millis() - last_print_ms) >=
            APP_PRINT_PERIOD_MS) {
            last_print_ms = system_millis();
            if (!line_tracking_has_line()) {
                print_no_line_data();
            }
        }
        //print_running_data();

        /* 控制主循环周期，目前配置为5ms。 */
        delay_ms(APP_CONTROL_PERIOD_MS);
    }
}
