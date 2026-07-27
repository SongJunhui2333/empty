#ifndef KEY_H
#define KEY_H

#include "ti_msp_dl_config.h"

extern uint8_t key_state_flag; // 按键状态标志位
extern uint8_t key_start_flag; // 启动/停止标志位，0表示停止，1表示启动

uint8_t get_gpio_state(GPIO_Regs *gpio_regs, uint32_t key);

#endif