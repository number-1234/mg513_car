#include "delay.h"

/* SysTick 每 1 ms 自增一次。 */
static volatile uint32_t s_tick_ms;

void delay_us(uint32_t microseconds)
{
    /* 32 MHz 下的近似软件延时，仅供传感器短时序使用。 */
    volatile uint32_t cycles = microseconds * 32U;

    while (cycles > 0U) {
        cycles--;
    }
}

void delay_ms(uint32_t milliseconds)
{
    const uint32_t start = s_tick_ms;

    while ((uint32_t)(s_tick_ms - start) < milliseconds) {
        /* 使用无符号减法，计数器回绕时仍能正确判断经过时间。 */
    }
}

uint32_t system_millis(void)
{
    return s_tick_ms;
}

void SysTick_Handler(void)
{
    /* SysConfig 已将 SysTick 周期配置为 1 ms。 */
    s_tick_ms++;
}
