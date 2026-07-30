#ifndef UART_H
#define UART_H

#include "ti_msp_dl_config.h"

extern volatile uint8_t uart_tx_buff[128]; // uart发送缓冲区
extern volatile uint8_t uart_rx_buff[128]; // uart接收缓冲区
extern volatile uint8_t uart_rx_flag;               // uart接收完成标志
extern volatile uint8_t uart_maixcam_rx_done;       // MAIXCAM UART一帧数据接收完成标志

// --------------------------- uart相关函数声明 --------------------------- //

void UART_print_char(UART_Regs *uart, const uint8_t chr);
void UART_print_string(UART_Regs *uart, const char *str);
void UART_send_data(UART_Regs *uart, const uint8_t *buff, uint16_t length);

#endif /* UART_H */
