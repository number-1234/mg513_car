/**
 * @file    main.c
 * @brief   循迹小车主程序 — MSPM0G3507, 32MHz
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "stdio.h"
#include "string.h"
#include "Uart.h"
#include "motor.h"
#include "line_follow.h"
#include "calibrate.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"

/* ========================= 传感器校准值 (校准后更新) ========================= */
unsigned short white[8] = {1053, 1225, 1277, 1216, 1033, 861, 1174, 1060};
unsigned short black[8] = {73,   74,   75,   72,   75,   77,  78,   75};

unsigned short Anolog[8] = {0};
unsigned short Normal[8];
unsigned char  rx_buff[256] = {0};

No_MCU_Sensor sensor;
unsigned char Digtal;

int main(void)
{
    SYSCFG_DL_init();

#if DO_CALIBRATION
    /* ========== 校准模式 ========== */
    Calibrate_Run(&sensor);
    /* 不会到达这里 */
#else
    /* ========== 正常循迹模式 ========== */
    Motor_Init();

    /* 传感器初始化 */
    No_MCU_Ganv_Sensor_Init(&sensor, white, black);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    /* 循迹控制器初始化 */
    LineFollow_Init();

    Tick_delay(100);

    while (1) {
        uint8_t lf_digital;
        uint8_t left_speed, right_speed;

        /* 1. 读取传感器 */
        No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
        Digtal = Get_Digtal_For_User(&sensor);

        /* 2. 位序转换 (Direction=1 → bit0=最右, 翻转使bit0=最左) */
        lf_digital = 0;
        for (int i = 0; i < 8; i++) {
            if (Digtal & (1 << i)) {
                lf_digital |= (1 << (7 - i));
            }
        }

        /* 3. PD循迹 → 左右轮速度 */
        LineFollow_Run(lf_digital, &left_speed, &right_speed);

        /* 4. 驱动电机 */
        Motor_Run(MOTOR_A, MOTOR_FORWARD, left_speed);
        Motor_Run(MOTOR_B, MOTOR_FORWARD, right_speed);

        /* 5. 串口调试: 传感器位图 + 速度 */
        sprintf((char *)rx_buff, "D:%d%d%d%d%d%d%d%d L:%d R:%d\r\n",
                (Digtal>>7)&1, (Digtal>>6)&1, (Digtal>>5)&1, (Digtal>>4)&1,
                (Digtal>>3)&1, (Digtal>>2)&1, (Digtal>>1)&1, (Digtal>>0)&1,
                left_speed, right_speed);
        uart0_send_string((char *)rx_buff);

        Tick_delay(5);
    }
#endif
}
