#include "uart.h"
#include <string.h>

/* --------------------------- uart相关变量 --------------------------- */
volatile uint8_t uart_tx_buff[128]; // uart发送缓冲区

/* ---------------------------------------------------------------- */
/*                            uart相关函数定义                            */
/* ---------------------------------------------------------------- */

void UART_print_char(UART_Regs *uart, const uint8_t chr)
{
    DL_UART_transmitDataBlocking(uart, chr);
}

/**
 * @brief   打印字符串
 *
 * @param uart   UART寄存器结构体指针
 * @param str   要打印的字符串，必须以'\0'结尾
 */
void UART_print_string(UART_Regs *uart, const char *str)
{
    while (*str)
    {
        UART_print_char(uart, (uint8_t)*str);
        str++;
    }
}

/**
 * @brief   发送数据
 * @param uart  UART寄存器结构体指针
 * @param buff      要发送的数据缓冲区
 * @param length    要发送的数据长度
 */
void UART_send_data(UART_Regs *uart, const uint8_t *buff, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        DL_UART_transmitDataBlocking(uart, buff[i]);
    }
}
