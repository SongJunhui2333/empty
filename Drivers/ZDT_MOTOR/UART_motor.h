#ifndef UART_MOTOR_H
#define UART_MOTOR_H

#include "UART/uart.h"
#include "ti_msp_dl_config.h"

#include "fifo.h"

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

extern __IO uint8_t rxCmd[FIFO_SIZE];
extern __IO uint8_t rxCount;

void UART_motor_Init(void);
void usart_getCmd(void);
void usart_SendCmd(__IO uint8_t *cmd, uint8_t len);
void usart_SendByte(uint16_t data);

#endif // UART_MOTOR_H