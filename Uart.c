#include "Uart.h"

#include <stddef.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

void uart_write_char(char character)
{
    while (DL_UART_isBusy(UART_0_INST)) {
        /* 等待上一个字节发送完成。 */
    }
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)character);
}

void uart_write_string(const char *text)
{
    /* 空指针直接返回，避免访问非法地址。 */
    if (text == NULL) {
        return;
    }

    /* 逐字节发送，直到字符串结束。 */
    while (*text != '\0') {
        uart_write_char(*text);
        text++;
    }
}
