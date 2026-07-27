#include "key.h"

uint8_t key_state_flag = 0; // 按键状态标志位
uint8_t key_start_flag = 0; // 启动/停止标志位，0表示停止，1表示启动

uint8_t get_gpio_state(GPIO_Regs *gpio_regs, uint32_t key)
{
    uint32_t high_bits = DL_GPIO_readPins(gpio_regs, key); // 0x00000040 0b01000000 PB6 0~31
    if ((high_bits & key) != 0)
        return 1;
    else
        return 0;
}