#include "uart.h"
#include <string.h>

/* --------------------------- uart相关变量 --------------------------- */
volatile uint8_t uart_tx_buff[128];        // uart发送缓冲区
volatile uint8_t uart_rx_buff[128];        // uart接收缓冲区
volatile uint8_t uart_rx_flag = 0;         // uart接收完成标志
volatile uint8_t uart_maixcam_rx_done = 0; // MAIXCAM UART一帧数据接收完成标志

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

/**
 * @brief   MAIXCAM UART中断服务函数
 *
 * 使用状态机接收固定10字节帧数据，帧格式：
 *   字节0-1: 帧头 0xFF 0xFE
 *   字节2:   第一个数据的正负标志 (0x00=正, 0x01=负)
 *   字节3-4: 第一个数据 (低字节在前, 高字节在后)
 *   字节5:   第二个数据的正负标志 (0x00=正, 0x01=负)
 *   字节6-7: 第二个数据 (低字节在前, 高字节在后)
 *   字节8-9: 帧尾 0xFE 0xFF
 *
 * @param   void
 * @return  void
 */
void UART_MAIXCAM_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_MAIXCAM_INST))
    {
    case DL_UART_MAIN_IIDX_RX: {
        static uint8_t RxState = 0;  // 接收状态机状态
        static uint8_t pRxIndex = 0; // 接收缓冲区写入索引

        uint8_t RxData = DL_UART_receiveDataBlocking(UART_MAIXCAM_INST);

        // 上一帧数据尚未被主循环取走，丢弃新收到的字节，防止覆盖缓冲区
        if (uart_maixcam_rx_done)
        {
            break;
        }

        switch (RxState)
        {
        /* ---- 状态0: 等待帧头第一个字节 0xFF ---- */
        case 0:
            if (RxData == 0xFF)
            {
                uart_rx_buff[pRxIndex++] = RxData;
                RxState = 1;
            }
            else
            {
                pRxIndex = 0; // 无效字节，重置指针
            }
            break;

        /* ---- 状态1: 等待帧头第二个字节 0xFE ---- */
        case 1:
            if (RxData == 0xFE)
            {
                uart_rx_buff[pRxIndex++] = RxData;
                RxState = 2;
            }
            else
            {
                // 帧头不完整，回退。若收到 0xFF 则可能是新帧头
                if (RxData == 0xFF)
                {
                    pRxIndex = 0;
                    uart_rx_buff[pRxIndex++] = RxData;
                    RxState = 1; // 保持在帧头检测状态
                }
                else
                {
                    pRxIndex = 0;
                    RxState = 0;
                }
            }
            break;

        /* ---- 状态2: 接收正负标志 ---- */
        case 2:
            uart_rx_buff[pRxIndex++] = RxData;
            RxState = 3;
            break;

        /* ---- 状态3: 接收数据低字节 ---- */
        case 3:
            uart_rx_buff[pRxIndex++] = RxData;
            RxState = 4;
            break;

        /* ---- 状态4: 接收第一个数据高字节 ---- */
        case 4:
            uart_rx_buff[pRxIndex++] = RxData;
            RxState = 5;
            break;

        /* ---- 状态5: 接收第二个数据的正负标志 ---- */
        case 5:
            uart_rx_buff[pRxIndex++] = RxData;
            RxState = 6;
            break;

        /* ---- 状态6: 接收第二个数据低字节 ---- */
        case 6:
            uart_rx_buff[pRxIndex++] = RxData;
            RxState = 7;
            break;

        /* ---- 状态7: 接收第二个数据高字节 ---- */
        case 7:
            uart_rx_buff[pRxIndex++] = RxData;
            RxState = 8;
            break;

        /* ---- 状态8: 等待帧尾第一个字节 0xFE ---- */
        case 8:
            if (RxData == 0xFE)
            {
                uart_rx_buff[pRxIndex++] = RxData;
                RxState = 9;
            }
            else
            {
                pRxIndex = 0;
                RxState  = 0;
            }
            break;

        /* ---- 状态9: 等待帧尾第二个字节 0xFF ---- */
        case 9:
            if (RxData == 0xFF)
            {
                uart_rx_buff[pRxIndex++] = RxData;
                uart_rx_flag          = 1;  // 完整帧接收完成
                uart_maixcam_rx_done  = 1;  // MAIXCAM帧接收完成
                pRxIndex = 0;
                RxState  = 0;
            }
            else
            {
                pRxIndex = 0;
                RxState  = 0;
            }
            break;

        default:
            pRxIndex = 0;
            RxState = 0;
            break;
        }
    }
    break;
    default:
        break;
    }
}