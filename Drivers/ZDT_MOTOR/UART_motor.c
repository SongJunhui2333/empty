#include "UART_motor.h"

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

__IO uint8_t rxCmd[FIFO_SIZE] = {0};
__IO uint8_t rxCount = 0;

void UART_motor_Init(void)
{
    // 清除串口中断标志
    NVIC_ClearPendingIRQ(UART_MOTOR_INST_INT_IRQN);
    // 使能串口中断
    NVIC_EnableIRQ(UART_MOTOR_INST_INT_IRQN);
}

/**
 * @brief   UART_MOTOR_INST中断函数
 * @param   无
 * @retval  无
 */
void UART_MOTOR_INST_IRQHandler(void)
{
    /**********************************************************
    ***	串口接收中断
    **********************************************************/
    if (DL_UART_getPendingInterrupt(UART_MOTOR_INST) == DL_UART_IIDX_RX)
    {
        // 未完成一帧数据接收，数据进入缓冲队列
        fifo_enQueue((uint8_t)DL_UART_Main_receiveData(UART_MOTOR_INST));
    }

    // 清除串口接收中断
    DL_UART_clearInterruptStatus(UART_MOTOR_INST, DL_UART_IIDX_RX);
}

/**
 * @brief   获取串口数据
 * @param   无
 * @retval  无
 */
void usart_getCmd(void)
{
    __IO uint16_t i = 0;

    // 提取一帧数据命令
    rxCount = fifo_queueLength();
    for (i = 0; i < rxCount; i++)
    {
        rxCmd[i] = fifo_deQueue();
    }
}

/**
 * @brief   USART发送多个字节
 * @param   无
 * @retval  无
 */
void usart_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    __IO uint8_t i = 0;

    for (i = 0; i < len; i++)
    {
        usart_SendByte(cmd[i]);
    }
}

/**
 * @brief   USART发送一个字节
 * @param   无
 * @retval  无
 */
void usart_SendByte(uint16_t data)
{
    __IO uint16_t t0 = 0;

    // 当串口0忙的时候等待，不忙的时候再发送传进来的字符
    while (DL_UART_isBusy(UART_MOTOR_INST) == true)
    {
        ++t0;
        if (t0 > 8000)
        {
            return;
        } // 超时退出，防止卡死
    }

    // 发送单个字符
    DL_UART_Main_transmitData(UART_MOTOR_INST, data);
}
