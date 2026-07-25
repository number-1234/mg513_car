#include "Sensor.h"

#include <stdio.h>
#include <string.h>

#include "Control/control.h"
#include "GYRO/GYRO_JY901.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "sys/sys.h"
#include "ti_msp_dl_config.h"

/* 改为 1 后执行黑白标定，标定结束后不会启动小车。 */
#define SENSOR_CALIBRATION_ENABLED      0

/* 丢线后先停车，等待期间继续读取 MPU6050。 */
#define STOP_WAIT_MS                 1000U

#define SENSOR_CHANNEL_COUNT            8U
#define CALIBRATION_SAMPLE_COUNT        32U
#define CALIBRATION_SETTLE_MS         3000U
#define CALIBRATION_SAMPLE_GAP_MS        2U

extern volatile int flag;

static No_MCU_Sensor s_sensor;
static uint32_t s_stop_start_ms;

/* 现有 ADC 灰度模块的黑白标定值。 */
static unsigned short s_white[8] = {
    443,345,408,571,391,466,493,500
};

static unsigned short s_black[8] = {
  100,102,103,102,104,105,106,104
};

volatile uint8_t Sensor_Bits = 0xFFU;

static uint8_t reverse_bits(uint8_t value)
{
    uint8_t result = 0U;
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++) {
        if ((value & (uint8_t)(1U << bit)) != 0U) {
            result |= (uint8_t)(1U << (7U - bit));
        }
    }
    return result;
}

void Sensor_Init(void)
{
#if 0
    Sensor_Calibration();
#endif

    No_MCU_Ganv_Sensor_Init(&s_sensor, s_white, s_black);
    s_sensor.Digtal = 0xFFU;
    Sensor_Bits = 0xFFU;
    s_stop_start_ms = system_millis();

    NVIC_ClearPendingIRQ(ADC12_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
}

/* ADC 和 8 路模拟开关采集过程保持原样，这里只读取最终的数字结果。 */
void Sensor_Read(void)
{
    No_Mcu_Ganv_Sensor_Task_Without_tick(&s_sensor);
    Sensor_Bits = reverse_bits(Get_Digtal_For_User(&s_sensor));
}

bool Sensor_Has_Line(void)
{
    if (Sensor_Bits == 0xFFU) {
        return false;
    }
    return true;
}

/*
 * 状态机：
 *   flag=1 循迹 → 丢线等1秒 → flag=3 转向
 *   flag=3 转向 → Yaw 到位(43~47°) → flag=4 直行
 *   flag=4 直行 → 找回线 → flag=1 循迹
 */
void Follow_Route(void)
{
    // if (flag == 1) {
    //     if (!Sensor_Has_Line()) {
    //         if ((uint32_t)(system_millis() - s_stop_start_ms) >= STOP_WAIT_MS) {
    //             flag = 3;
    //         }
    //     } else {
    //         s_stop_start_ms = system_millis();
    //     }
    // }
    // else if (flag == 4) {
    //     if (Sensor_Has_Line()) {
    //         flag = 1;
    //     }
    // }
}

/* 8 路权重直接用 if 写出，修改循迹力度时只改这里。 */
int Incremental_Quantity(void)
{
    int value = 0;

    if ((Sensor_Bits & 0x01U) == 0U) value -= 12;
    if ((Sensor_Bits & 0x02U) == 0U) value -= 9;
    if ((Sensor_Bits & 0x04U) == 0U) value -= 7;
    if ((Sensor_Bits & 0x08U) == 0U) value -= 3;
    if ((Sensor_Bits & 0x10U) == 0U) value += 3;
    if ((Sensor_Bits & 0x20U) == 0U) value += 7;
    if ((Sensor_Bits & 0x40U) == 0U) value += 9;
    if ((Sensor_Bits & 0x80U) == 0U) value += 12;

    return value;
}

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

static void print_values(const char *name,
                         const uint16_t values[SENSOR_CHANNEL_COUNT])
{
    printf("%s: %u,%u,%u,%u,%u,%u,%u,%u\r\n",
           name,
           (unsigned int)values[0], (unsigned int)values[1],
           (unsigned int)values[2], (unsigned int)values[3],
           (unsigned int)values[4], (unsigned int)values[5],
           (unsigned int)values[6], (unsigned int)values[7]);
}

/* 原来的 calibrate.c 已合并到 Sensor 模块。 */
void Sensor_Calibration(void)
{
    No_MCU_Sensor sensor;
    uint16_t white[SENSOR_CHANNEL_COUNT];
    uint16_t black[SENSOR_CHANNEL_COUNT];

    memset(&sensor, 0, sizeof(sensor));
    No_MCU_Ganv_Sensor_Init_Frist(&sensor);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    printf("\r\n=== 白色校准 ===\r\n");
    printf("请将传感器放在白色表面，3 秒后开始采样...\r\n");
    delay_ms(CALIBRATION_SETTLE_MS);
    capture_average(&sensor, white);
    print_values("WHITE", white);

    printf("\r\n=== 黑色校准 ===\r\n");
    printf("请将传感器放在黑色表面，3 秒后开始采样...\r\n");
    delay_ms(CALIBRATION_SETTLE_MS);
    capture_average(&sensor, black);
    print_values("BLACK", black);

    printf("\r\n请将以下数值复制到 Sensor/Sensor.c：\r\n");
    print_values("white[8]", white);
    print_values("black[8]", black);
    printf("校准完成，请将 SENSOR_CALIBRATION_ENABLED 改回 0。\r\n");

    while (true) {
        __WFI();
    }
}
