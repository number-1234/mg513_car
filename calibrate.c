/**
 * @file    calibrate.c
 * @brief   灰度传感器黑白校准
 *
 *          流程:
 *            Phase 1 → 传感器放白色地面, 等待3秒, 采样32次取平均, 打印WHITE值
 *            Phase 2 → 传感器放黑色线上, 等待3秒, 采样32次取平均, 打印BLACK值
 *            Phase 3 → 打印可直接复制的数组代码, 进入死循环
 *
 *          用户拿到值后:
 *            1. 复制打印的数组, 粘贴到 main.c 的 white[] / black[]
 *            2. 将 calibrate.h 中的 DO_CALIBRATION 改为 0
 *            3. 重新编译下载
 */

#include "calibrate.h"
#include "Uart.h"
#include "delay.h"
#include "stdio.h"
#include "string.h"

#define CALIB_SAMPLES   32    /* 每个相位采样次数 */

/* ========================= 校准主函数 ========================= */

void Calibrate_Run(No_MCU_Sensor *sensor)
{
    char            buf[256];
    unsigned long   sum[8];
    unsigned short  white[8];
    unsigned short  black[8];
    int             i, s;

    /* ---- 传感器基础初始化 (不需要黑白值) ---- */
    No_MCU_Ganv_Sensor_Init_Frist(sensor);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    /* ================================================================
     *  Phase 1: 白色校准
     * ================================================================ */
    uart0_send_string("\r\n========================================\r\n");
    uart0_send_string("  SENSOR CALIBRATION\r\n");
    uart0_send_string("========================================\r\n\r\n");

    uart0_send_string(">>> Phase 1: WHITE <<<\r\n");
    uart0_send_string("Place sensor on WHITE surface...\r\n");
    uart0_send_string("Starting in 3 seconds...\r\n");

    Tick_delay(3000);

    uart0_send_string("Sampling white...\r\n");
    memset(sum, 0, sizeof(sum));

    for (s = 0; s < CALIB_SAMPLES; s++) {
        No_Mcu_Ganv_Sensor_Task_Without_tick(sensor);
        for (i = 0; i < 8; i++) {
            sum[i] += sensor->Analog_value[i];
        }
        Tick_delay(2);
    }

    for (i = 0; i < 8; i++) {
        white[i] = (unsigned short)(sum[i] / CALIB_SAMPLES);
    }

    sprintf(buf, "WHITE: %d, %d, %d, %d, %d, %d, %d, %d\r\n\r\n",
            white[0], white[1], white[2], white[3],
            white[4], white[5], white[6], white[7]);
    uart0_send_string(buf);

    /* ================================================================
     *  Phase 2: 黑色校准
     * ================================================================ */
    uart0_send_string(">>> Phase 2: BLACK <<<\r\n");
    uart0_send_string("Place sensor on BLACK line...\r\n");
    uart0_send_string("Starting in 3 seconds...\r\n");

    Tick_delay(3000);

    uart0_send_string("Sampling black...\r\n");
    memset(sum, 0, sizeof(sum));

    for (s = 0; s < CALIB_SAMPLES; s++) {
        No_Mcu_Ganv_Sensor_Task_Without_tick(sensor);
        for (i = 0; i < 8; i++) {
            sum[i] += sensor->Analog_value[i];
        }
        Tick_delay(2);
    }

    for (i = 0; i < 8; i++) {
        black[i] = (unsigned short)(sum[i] / CALIB_SAMPLES);
    }

    sprintf(buf, "BLACK: %d, %d, %d, %d, %d, %d, %d, %d\r\n\r\n",
            black[0], black[1], black[2], black[3],
            black[4], black[5], black[6], black[7]);
    uart0_send_string(buf);

    /* ================================================================
     *  Phase 3: 打印可直接复制的代码
     * ================================================================ */
    uart0_send_string("========================================\r\n");
    uart0_send_string("  COPY THE LINES BELOW INTO main.c:\r\n");
    uart0_send_string("========================================\r\n\r\n");

    sprintf(buf,
        "unsigned short white[8] = {%d, %d, %d, %d, %d, %d, %d, %d};\r\n",
        white[0], white[1], white[2], white[3],
        white[4], white[5], white[6], white[7]);
    uart0_send_string(buf);

    sprintf(buf,
        "unsigned short black[8] = {%d, %d, %d, %d, %d, %d, %d, %d};\r\n",
        black[0], black[1], black[2], black[3],
        black[4], black[5], black[6], black[7]);
    uart0_send_string(buf);

    uart0_send_string("\r\n========================================\r\n");
    uart0_send_string("  Calibration DONE.\r\n");
    uart0_send_string("  1. Copy the 2 lines above into main.c\r\n");
    uart0_send_string("  2. Set DO_CALIBRATION to 0 in calibrate.h\r\n");
    uart0_send_string("  3. Rebuild & flash\r\n");
    uart0_send_string("========================================\r\n");

    /* 校准完成后死循环, 等待用户重新编译 */
    while (1) {
        __WFI();
    }
}
