#ifndef UART_H
#define UART_H

#include "ti_msp_dl_config.h"

extern volatile uint8_t uart_tx_buff[128]; // uart发送缓冲区

// --------------------------- uart相关函数声明 --------------------------- //

void UART_print_char(UART_Regs *uart, const uint8_t chr);
void UART_print_string(UART_Regs *uart, const char *str);
void UART_send_data(UART_Regs *uart, const uint8_t *buff, uint16_t length);

#endif /* UART_H */
