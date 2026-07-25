#include "sys.h"

#include <string.h>

/* SysTick 由 empty.syscfg 配置为每 1ms 进入一次中断。 */
static volatile uint32_t s_tick_ms;

void delay_us(uint32_t microseconds)
{
    /* 保持原灰度采集程序使用的 32MHz 软件延时写法。 */
    volatile uint32_t cycles = microseconds * 32U;

    while (cycles > 0U) {
        cycles--;
    }
}

void delay_ms(uint32_t milliseconds)
{
    uint32_t start = s_tick_ms;

    while ((uint32_t)(s_tick_ms - start) < milliseconds) {
    }
}

uint32_t system_millis(void)
{
    return s_tick_ms;
}

void SysTick_Handler(void)
{
    s_tick_ms++;
}

/* printf 最终通过 UART0 逐字节发送。 */
int fputc(int character, FILE *stream)
{
    (void)stream;
    while (DL_UART_isBusy(UART_0_INST)) {
    }
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)character);
    return character;
}

int fputs(const char *restrict text, FILE *restrict stream)
{
    int count = 0;

    (void)stream;
    if (text == NULL) {
        return 0;
    }

    while (*text != '\0') {
        (void)fputc(*text, stream);
        text++;
        count++;
    }
    return count;
}

int puts(const char *text)
{
    int count = fputs(text, stdout);
    count += fputs("\r\n", stdout);
    return count;
}

float limit_float(float value, float maximum, float minimum)
{
    if (value > maximum) {
        value = maximum;
    }
    if (value < minimum) {
        value = minimum;
    }
    return value;
}

int myabs(int value)
{
    if (value < 0) {
        return -value;
    }
    return value;
}

float normalize_angle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}
