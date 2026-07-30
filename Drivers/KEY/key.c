#include "key.h"

uint8_t key_state_flag = 0; // 按键状态标志位
uint8_t key_start_flag = 0; // 启动/停止标志位，0表示停止，1表示启动

uint8_t key_num = 0;

uint8_t get_gpio_state(GPIO_Regs *gpio_regs, uint32_t key)
{
    uint32_t high_bits = DL_GPIO_readPins(gpio_regs, key); // 0x00000040 0b01000000 PB6 0~31
    if ((high_bits & key) != 0)
        return 1;
    else
        return 0;
}

uint8_t Key_GetState()
{
    uint8_t key_num = 0;
    if (!get_gpio_state(KEY_K_1_PORT, KEY_K_1_PIN)) // 检测到按键按下
    {
        key_num = 1; // 按键1被按下
    }
    else if (!get_gpio_state(KEY_K_2_PORT, KEY_K_2_PIN)) // 检测到按键按下
    {
        key_num = 2; // 按键2被按下
    }
    else if (!get_gpio_state(KEY_K_3_PORT, KEY_K_3_PIN)) // 检测到按键按下
    {
        key_num = 3; // 按键3被按下
    }
    return key_num;
}

uint8_t Key_GetNum()
{
    uint8_t temp;
    if (key_num != 0) // 检测到按键按下
    {
        temp = key_num; // 保存按键编号
        key_num = 0;    // 清除按键编号
        return temp;    // 返回按键编号
    }
    else
        return 0; // 没有按键按下，返回0
}

void Key_Tick(void)
{
    static uint8_t Count = 0;
    static uint8_t CurrentKeyState = 0; // 当前按键状态
    static uint8_t LastKeyState = 0;    // 上一次按键状态
    Count++;
    if (Count >= 25) // 每25ms检测一次按键状态
    {
        Count = 0;
        LastKeyState = CurrentKeyState;   // 保存上一次按键状态
        CurrentKeyState = Key_GetState(); // 获取当前按键状态

        if (CurrentKeyState == 0 && LastKeyState != 0) // 检测到按键释放
        {
            key_num = LastKeyState; // 保存按键编号
        }
    }
}