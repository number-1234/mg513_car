#ifndef UART_H
#define UART_H

/* 阻塞发送一个字符。 */
void uart_write_char(char character);

/* 阻塞发送以 '\0' 结尾的字符串，允许传入空指针。 */
void uart_write_string(const char *text);

#endif
