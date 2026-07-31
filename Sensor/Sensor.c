#include "Sensor.h"

#include <stdio.h>
#include <string.h>

#include "Control/control.h"
#include "GYRO/GYRO_JY901.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "sys/sys.h"
#include "ti_msp_dl_config.h"
#include "LCD/lcd.h"

/* 改为 1 后执行黑白标定，标定结束后不会启动小车。 */
#define SENSOR_CALIBRATION_ENABLED      0

/* 丢线后先停车，等待期间继续读取 MPU6050。 */
#define STOP_WAIT_MS                 1000U
#define STOP_LINE_CONSECUTIVE_BLACK_COUNT 4U
#define STOP_LINE_CONFIRM_MS           30U

#define SENSOR_CHANNEL_COUNT            8U
#define CALIBRATION_SAMPLE_COUNT        32U
#define CALIBRATION_SETTLE_MS         3000U
#define CALIBRATION_SAMPLE_GAP_MS        2U

/* 新 8 路红外模块：7-bit I2C 地址 0x12，状态寄存器 0x30。 */
#define IR8_I2C_ADDRESS                0x12U
#define IR8_STATUS_REGISTER            0x30U

extern volatile int flag;

static No_MCU_Sensor s_sensor;
static uint32_t s_stop_start_ms;
static bool s_ir8_online;
static uint32_t s_stop_line_enable_ms;
static bool s_stop_line_armed;
static uint32_t s_stop_line_black_start_ms;
static bool s_stop_line_black_active;

/* 复用 PA0/PA1 上已经初始化的硬件 I2C0。
 * 返回 0 表示传输成功；addr 使用 7-bit I2C 地址。 */
char MPU6050_ReadData(uint8_t addr, uint8_t regaddr,
                      uint8_t num, uint8_t *read_data);

/* 现有 ADC 灰度模块的黑白标定值。 */
static unsigned short s_white[8] = {
2000,2000,2000,2000,2000,2000,2000,2000,
};

static unsigned short s_black[8] = {
 300,300,300,300,300,300,300,300
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
#if 1
    /* 旧 8 路 ADC 灰度循迹：红外模块测试期间不初始化。 */
#if SENSOR_CALIBRATION_ENABLED
    Sensor_Calibration();
#endif

    No_MCU_Ganv_Sensor_Init(&s_sensor, s_white, s_black);
    s_sensor.Digtal = 0xFFU;
    Sensor_Bits = 0x00U;
    s_stop_start_ms = system_millis();

    NVIC_ClearPendingIRQ(ADC12_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
#endif

    /* 新 I2C 八路红外模块无需额外初始化；FF 表示 8 路均为白底。 */
    Sensor_Bits = 0xFFU;
    s_ir8_online = false;
    s_stop_line_armed = false;
    s_stop_line_black_active = false;
    s_stop_start_ms = system_millis();
}

/* 从 I2C 八路红外模块读取 X1..X8 状态。 */
void Sensor_Read(void)
{
#if 1
    /* 旧 8 路 ADC 灰度循迹读取，保留以便日后恢复。 */
    No_Mcu_Ganv_Sensor_Task_Without_tick(&s_sensor);
    Sensor_Bits = reverse_bits(Get_Digtal_For_User(&s_sensor));
    s_ir8_online = true;
#endif

#if 0
    uint8_t raw_bits = 0U;

    /*
     * 模块：bit7..bit0 = X1..X8，0=黑线，1=白底。
     * 直接保留模块原始状态，便于串口数据和模块说明书一一对应。
     */
    if (MPU6050_ReadData(IR8_I2C_ADDRESS, IR8_STATUS_REGISTER,
                         1U, &raw_bits) == 0) {
        Sensor_Bits = raw_bits;
        s_ir8_online = true;
    } else {
        /* 通信失败不可沿用旧值，避免小车按陈旧循迹信息继续运行。 */
        Sensor_Bits = 0xFFU;
        s_ir8_online = false;
    }
#endif
}

bool Sensor_Has_Line(void)
{
    return s_ir8_online && (Sensor_Bits != 0xFFU);
}

bool Sensor_Is_Online(void)
{
    return s_ir8_online;
}

uint8_t Sensor_Black_Count(void)
{
    uint8_t value = Sensor_Bits;
    uint8_t count = 0U;

    /* 当前模块定义：每个为 0 的位表示对应探头检测到黑线。 */
    for (uint8_t bit = 0U; bit < 8U; bit++) {
        if ((value & 1U) == 0U) {
            count++;
        }
        value >>= 1U;
    }
    return count;
}

void Sensor_Enable_Stop_Line_After(uint32_t delay_ms)
{
    s_stop_line_enable_ms = system_millis() + delay_ms;
    s_stop_line_armed = true;
    s_stop_line_black_active = false;
}

void Sensor_Disable_Stop_Line(void)
{
    s_stop_line_armed = false;
    s_stop_line_black_active = false;
}

bool Sensor_Stop_Line_Enabled(void)
{
    return s_stop_line_armed &&
           ((int32_t)(system_millis() - s_stop_line_enable_ms) >= 0);
}

/* 连续黑线必须相邻；例如 0x4B 虽有 4 个 0，但它们分散，不是停车线。 */
static bool Sensor_HasConsecutiveBlack(uint8_t required_count)
{
    uint8_t bit;
    uint8_t consecutive_count = 0U;

    for (bit = 0U; bit < 8U; bit++) {
        if ((Sensor_Bits & (uint8_t)(1U << bit)) == 0U) {
            consecutive_count++;
            if (consecutive_count >= required_count) {
                return true;
            }
        } else {
            consecutive_count = 0U;
        }
    }

    return false;
}

/* 仅在主循环调用：相邻 4 路黑线持续 30ms 才判定停车线。 */
bool Sensor_Stop_Line_Detected(void)
{
    uint32_t now_ms = system_millis();

    if (!Sensor_Stop_Line_Enabled() ||
        !Sensor_HasConsecutiveBlack(STOP_LINE_CONSECUTIVE_BLACK_COUNT)) {
        s_stop_line_black_active = false;
        return false;
    }

    if (!s_stop_line_black_active) {
        s_stop_line_black_active = true;
        s_stop_line_black_start_ms = now_ms;
        return false;
    }

    return (uint32_t)(now_ms - s_stop_line_black_start_ms) >=
           STOP_LINE_CONFIRM_MS;
}

/*
 * 状态机：
 *   flag=1 循迹 → 丢线等1秒 → flag=3 转向
 *   flag=3 转向 → Yaw 到位(43~47°) → flag=4 直行
 *   flag=4 直行 → 找回线 → flag=1 循迹
 */
void Follow_Route(void)
{
    if (flag == 1) {
        if (!Sensor_Has_Line()) {
            if ((uint32_t)(system_millis() - s_stop_start_ms) >= STOP_WAIT_MS) {
                 flag = 3;
            }
        } else {
            s_stop_start_ms = system_millis();
        }
    }
     else if (flag == 4) {
         if (Sensor_Has_Line()) {
             flag = 1;
         }
     }
}

/* 8 路权重直接用 if 写出，修改循迹力度时只改这里。 */
int Incremental_Quantity(void)
{
    int value = 0;

    /* 模块原始位序：X1=bit7（左），X8=bit0（右），0=压线。 */
    if ((Sensor_Bits & 0x80U) == 0U) value -= 12;
    if ((Sensor_Bits & 0x40U) == 0U) value -= 9;
    if ((Sensor_Bits & 0x20U) == 0U) value -= 7;
    if ((Sensor_Bits & 0x10U) == 0U) value -= 3;
    if ((Sensor_Bits & 0x08U) == 0U) value += 3;
    if ((Sensor_Bits & 0x04U) == 0U) value += 7;
    if ((Sensor_Bits & 0x02U) == 0U) value += 9;
    if ((Sensor_Bits & 0x01U) == 0U) value += 12;

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
    char buf[24];

    memset(&sensor, 0, sizeof(sensor));
    No_MCU_Ganv_Sensor_Init_Frist(&sensor);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    /* ---- 白色校准 ---- */
    LCD_Fill(0, 0, 169, 79, BLACK);
    LCD_ShowString(10, 5,  "WHITE calibrating...", WHITE, BLACK, 16, 0);
    LCD_ShowString(10, 25, "Put on WHITE surface", WHITE, BLACK, 16, 0);
    printf("\r\n=== 白色校准 ===\r\n");
    printf("请将传感器放在白色表面，3 秒后开始采样...\r\n");
    delay_ms(CALIBRATION_SETTLE_MS);
    capture_average(&sensor, white);
    print_values("WHITE", white);

    /* 显示白底8路值 */
    LCD_Fill(0, 0, 169, 79, BLACK);
    LCD_ShowString(10, 5, "WHITE values:", GREEN, BLACK, 16, 0);
    for (int i = 0; i < SENSOR_CHANNEL_COUNT; i++) {
        int row = i / 4;
        int col = i % 4;
        int x = 5 + col * 42;
        int y = 25 + row * 20;
        snprintf(buf, sizeof(buf), "%4u", white[i]);
        LCD_ShowString(x, y, (const u8 *)buf, WHITE, BLACK, 16, 0);
    }
    delay_ms(2000);

    /* ---- 黑色校准 ---- */
    LCD_Fill(0, 0, 169, 79, BLACK);
    LCD_ShowString(10, 5,  "BLACK calibrating...", WHITE, BLACK, 16, 0);
    LCD_ShowString(10, 25, "Put on BLACK surface", WHITE, BLACK, 16, 0);
    printf("\r\n=== 黑色校准 ===\r\n");
    printf("请将传感器放在黑色表面，3 秒后开始采样...\r\n");
    delay_ms(CALIBRATION_SETTLE_MS);
    capture_average(&sensor, black);
    print_values("BLACK", black);

    /* 显示黑底8路值 */
    LCD_Fill(0, 0, 169, 79, BLACK);
    LCD_ShowString(10, 5, "BLACK values:", RED, BLACK, 16, 0);
    for (int i = 0; i < SENSOR_CHANNEL_COUNT; i++) {
        int row = i / 4;
        int col = i % 4;
        int x = 5 + col * 42;
        int y = 25 + row * 20;
        snprintf(buf, sizeof(buf), "%4u", black[i]);
        LCD_ShowString(x, y, (const u8 *)buf, WHITE, BLACK, 16, 0);
    }

    printf("\r\n请将以下数值复制到 Sensor/Sensor.c：\r\n");
    print_values("white[8]", white);
    print_values("black[8]", black);
    printf("校准完成，请将 SENSOR_CALIBRATION_ENABLED 改回 0。\r\n");

    /* 底部提示完成 */
    LCD_ShowString(10, 70, "Done! Copy to code", YELLOW, BLACK, 16, 0);

    while (true) {
        __WFI();
    }
}
