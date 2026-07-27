#include "gw_gray_serial.h"

unsigned char Digtal;

/**
 * @brief 读取灰度传感器的值
 * @return 读取到的灰度值
 */
uint8_t gw_gray_serial_read()
{
    uint8_t ret = 0;
    uint8_t i;

    for (i = 0; i < 8; ++i)
    {
        /* 输出时钟下降沿 */
        DL_GPIO_clearPins(GW_Serial_PORT, GW_Serial_CLK_PIN);
        delay_us(2);
        // 避免GPIO翻转过快导致反应不及时
        ret |= (DL_GPIO_readPins(GW_Serial_PORT, GW_Serial_DAT_PIN) == 0 ? 0 : 1) << i;

        /* 输出时钟上升沿,让传感器更新数据*/
        DL_GPIO_setPins(GW_Serial_PORT, GW_Serial_CLK_PIN);

        /* 延迟需要在5us左右 */
        delay_us(5);
    }

    return ret;
}