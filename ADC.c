#include "ADC.h"

#include <stdbool.h>

#include "ti_msp_dl_config.h"

/* ADC 转换完成标志，由中断置位。 */
static volatile bool s_conversion_complete;

uint16_t adc_read(void)
{
    uint16_t result;

    /* 先清标志再启动转换，避免误用上一次转换结果。 */
    s_conversion_complete = false;
    DL_ADC12_startConversion(ADC12_0_INST);

    /* 等待 ADC 中断；WFE 可以减少忙等期间的功耗。 */
    while (!s_conversion_complete) {
        __WFE();
    }

    result = (uint16_t)DL_ADC12_getMemResult(
        ADC12_0_INST, ADC12_0_ADCMEM_ADC_CH0);
    return result;
}

unsigned int adc_getValue(void)
{
    /* 保留第三方灰度驱动要求的函数名和返回类型。 */
    return (unsigned int)adc_read();
}

void ADC12_0_INST_IRQHandler(void)
{
    /* 只处理 MEM0 转换完成事件。 */
    if (DL_ADC12_getPendingInterrupt(ADC12_0_INST) ==
        DL_ADC12_IIDX_MEM0_RESULT_LOADED) {
        s_conversion_complete = true;
    }
}
