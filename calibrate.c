#include "calibrate.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "delay.h"
#include "ti_msp_dl_config.h"
#include "Uart.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"

#define SENSOR_CHANNEL_COUNT       8U
#define CALIBRATION_SAMPLE_COUNT   32U
#define CALIBRATION_SETTLE_MS      3000U
#define CALIBRATION_SAMPLE_GAP_MS  2U

/* 连续采样多次并计算每个通道的平均值。 */
static void capture_average(No_MCU_Sensor *sensor,
                            uint16_t values[SENSOR_CHANNEL_COUNT])
{
    uint32_t sums[SENSOR_CHANNEL_COUNT] = {0U};
    uint32_t sample;
    uint32_t channel;

    for (sample = 0U; sample < CALIBRATION_SAMPLE_COUNT; sample++) {
        No_Mcu_Ganv_Sensor_Task_Without_tick(sensor);
        for (channel = 0U; channel < SENSOR_CHANNEL_COUNT; channel++) {
            sums[channel] += sensor->Analog_value[channel];
        }
        delay_ms(CALIBRATION_SAMPLE_GAP_MS);
    }

    for (channel = 0U; channel < SENSOR_CHANNEL_COUNT; channel++) {
        values[channel] =
            (uint16_t)(sums[channel] / CALIBRATION_SAMPLE_COUNT);
    }
}

/* 将 8 路数据格式化后通过串口输出。 */
static void print_values(const char *label,
                         const uint16_t values[SENSOR_CHANNEL_COUNT])
{
    char buffer[160];

    (void)snprintf(buffer, sizeof(buffer),
                   "%s: %u,%u,%u,%u,%u,%u,%u,%u\r\n",
                   label,
                   (unsigned int)values[0], (unsigned int)values[1],
                   (unsigned int)values[2], (unsigned int)values[3],
                   (unsigned int)values[4], (unsigned int)values[5],
                   (unsigned int)values[6], (unsigned int)values[7]);
    uart_write_string(buffer);
}

void sensor_calibration_run(void)
{
    No_MCU_Sensor sensor;
    uint16_t white[SENSOR_CHANNEL_COUNT];
    uint16_t black[SENSOR_CHANNEL_COUNT];

    memset(&sensor, 0, sizeof(sensor));
    No_MCU_Ganv_Sensor_Init_Frist(&sensor);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    /* 第一阶段：采集白色表面的基准值。 */
    uart_write_string("\r\n=== 白色校准 ===\r\n");
    uart_write_string("请将传感器放在白色表面，3 秒后开始采样...\r\n");
    delay_ms(CALIBRATION_SETTLE_MS);
    capture_average(&sensor, white);
    print_values("WHITE", white);

    /* 第二阶段：采集黑色表面的基准值。 */
    uart_write_string("\r\n=== 黑色校准 ===\r\n");
    uart_write_string("请将传感器放在黑色表面，3 秒后开始采样...\r\n");
    delay_ms(CALIBRATION_SETTLE_MS);
    capture_average(&sensor, black);
    print_values("BLACK", black);

    /* 打印可复制到车辆控制模块中的最终结果。 */
    uart_write_string("\r\n请将以下数值复制到 car_control.c：\r\n");
    print_values("white[8]", white);
    print_values("black[8]", black);
    uart_write_string("校准完成。请关闭校准模式并重新构建。\r\n");

    /* 校准模式不再进入正常车辆控制流程。 */
    while (true) {
        __WFI();
    }
}
