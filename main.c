 #include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "Control/control.h"
#include "Encoder/Encoder.h"
#include "GYRO/GYRO_JY901.h"
#include "Sensor/Sensor.h"
#include "sys/sys.h"

volatile int flag = 2;  /* 初始停车 */

static uint32_t s_last_print_ms;

static void Print_Running_Data(void)
{
    printf("Flag:%d Sensor:%02X Yaw:%.1f "
           "L:%.1f R:%.1f PWM:%d,%d\r\n",
           flag,
           (unsigned int)Sensor_Bits,
           (double)Yaw,
           (double)EncoderA_VEL,
           (double)EncoderB_VEL,
           Motor_Left,
           Motor_Right);
}

int main(void)
{
    uint8_t gyro_status;

    SYSCFG_DL_init();
    delay_ms(100U);

    Sensor_Init();
    Control_Init();

    gyro_status = GYRO_Init();
    Encoder_Init();

    if (gyro_status != 0U) {
        printf("JY901S fail: %u\r\n", (unsigned int)gyro_status);
    }

    flag = 3;
    s_last_print_ms = system_millis();

    while (1) {
        Sensor_Read();
        (void)GYRO_Update();
        Follow_Route();

        if ((uint32_t)(system_millis() - s_last_print_ms) >= 100U) {
            s_last_print_ms = system_millis();
            Print_Running_Data();
        }
    }
}

void TIMA0_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_LOAD) {
        Read_Encoder();
        Control();
    }
}

void GROUP1_IRQHandler(void)
{
    Encodering();
}
