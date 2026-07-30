#include "uart.h"
#include <string.h>

/* --------------------------- uart相关变量 --------------------------- */
volatile uint8_t uart_tx_buff[128]; // uart发送缓冲区
volatile uint8_t uart_rx_buff[128]; // uart接收缓冲区
volatile uint8_t uart_rx_flag = 0;  // uart接收完成标志

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

// void UART_TRANS_INST_IRQHandler(void)
// {
//     // switch (DL_UART_Main_getPendingInterrupt(UART_TRANS_INST))
//     // {
//         //数据解析案例
//     //     static uint8_t RxState = 0;   // 接收状态机状态
//     //     static uint8_t pRxBuffer = 0; // 接收缓冲区指针

//     // case DL_UART_MAIN_IIDX_RX: {
//     //     // 处理接收中断

//     //     uint8_t RxData = DL_UART_receiveDataBlocking(UART_TRANS_INST);
//     //     // UART_print_char(DEBUG_INST, RxData); // 将接收到的数据打印到调试串口
//     //     if (RxState == 0 && uart_rx_flag == 0)
//     //     {
//     //         if (RxData == '@')
//     //         {
//     //             RxState = 1;   // 进入接收数据状态
//     //             pRxBuffer = 0; // 重置接收缓冲区指针
//     //         }
//     //     }
//     //     else if (RxState == 1)
//     //     {
//     //         if (RxData == '#')
//     //         {
//     //             RxState = 2;
//     //         }
//     //         else
//     //         {
//     //             uart_rx_buff[pRxBuffer++] = RxData; // 存储接收到的数据
//     //         }
//     //     }
//     //     else if (RxState == 2)
//     //     {
//     //         if (RxData == '$')
//     //         {
//     //             RxState = 0;                    // 重置状态机
//     //             uart_rx_buff[pRxBuffer] = '\0'; // 添加字符串结束符
//     //             uart_rx_flag = 1;               // 设置接收完成标志
//     //         }
//     //     }
//     // }
//     // break;
//     // default:
//     //     // 处理其他中断
//     //     break;
//     // }
// }