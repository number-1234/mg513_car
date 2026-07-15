#ifndef ADC_H
#define ADC_H

#include <stdint.h>

/* 灰度传感器第三方驱动使用的兼容接口。 */
unsigned int adc_getValue(void);

/* 自有代码优先使用的 ADC 读取接口。 */
uint16_t adc_read(void);

#endif
