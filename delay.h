#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>

/* 微秒级忙等待；第三方灰度传感器驱动依赖此接口。 */
void delay_us(uint32_t microseconds);

/* 基于 SysTick 的毫秒级阻塞延时。 */
void delay_ms(uint32_t milliseconds);

/* 返回开机以来的毫秒数，约 49.7 天回绕一次。 */
uint32_t system_millis(void);

#endif
