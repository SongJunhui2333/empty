/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "main.h"
#include "ti_msp_dl_config.h"

int main(void)
{
    SYSCFG_DL_init();

    /* Don't remove this! */
    Interrupt_Init();

    // 初始化OLED显示屏
    OLED_Init();
    // 初始化陀螺仪
    WIT_Init();

    motor_init(1); // 初始化电机1
    motor_init(2); // 初始化电机2
    // motor_set_direction(1, 1);                                                      // 设置电机1为正转
    // motor_set_direction(2, 1);                                                      // 设置电机2为正转
    pid_init(&pid_motor_l, PID_INCREMENTAL, MOTOR_KP, MOTOR_KI, MOTOR_KD, 4000, 0); // 初始化左轮PID
    pid_init(&pid_motor_r, PID_INCREMENTAL, MOTOR_KP, MOTOR_KI, MOTOR_KD, 4000, 0); // 初始化右轮PID

    pid_set_setpoint(&pid_motor_l, 0); // 初始化左轮速度PID目标值
    pid_set_setpoint(&pid_motor_r, 0); // 初始化右轮速度PID目标值

    NVIC_EnableIRQ(GPIOA_INT_IRQn); // 使能GPIOA中断
    NVIC_EnableIRQ(GPIOB_INT_IRQn); // 使能GPIOB中断

    NVIC_EnableIRQ(MOTOR_CONTROL_INST_INT_IRQN); // 使能电机控制中断

    NVIC_EnableIRQ(TIMER_BASE_INST_INT_IRQN); // 使能基础定时器中断

    my_delay_ms(500); // 等待0.5秒使系统上电完成

    while (1)
    {
        // // 主循环

        // 灰度传感器读取测试代码
        // sprintf((char *)uart_tx_buff, "Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n", gw_gray_sensor[0], gw_gray_sensor[1],
        //         gw_gray_sensor[2], gw_gray_sensor[3], gw_gray_sensor[4], gw_gray_sensor[5], gw_gray_sensor[6],
        //         gw_gray_sensor[7]);
        // UART_print_string(DEBUG_INST, (char *)uart_tx_buff);
        // memset((void *)uart_tx_buff, 0, 128);

        // sprintf((char *)oled_buff, "L: %d, R: %d", filt_velocity_l, filt_velocity_r);
        // OLED_ShowString(0, 0, (uint8_t *)oled_buff, 16);

        // float_t time = (float)tick_ms / 1000.0;
        // sprintf((char *)oled_buff, "time: %.2f", time);
        // OLED_ShowString(0, 2, (uint8_t *)oled_buff, 16);

        // uint8_t key_num = Key_GetNum(); // 获取按键编号
        // if (key_num == 1)               // 按键1被按下
        // {
        //     key_state_flag = (key_state_flag + 1) % 7;
        // }
        // else if (key_num == 2) // 按键2被按下
        // {
        //     key_state_flag = (key_state_flag - 1 + 7) % 7;
        // }
        // else if (key_num == 3) // 按键3被按下
        // {
        //     key_start_flag = !key_start_flag; // 切换启动/停止状态
        // }
        // sprintf((char *)oled_buff, "key_state: %d", key_state_flag);
        // OLED_ShowString(0, 4, (uint8_t *)oled_buff, 16);

        // sprintf((char *)oled_buff, "key_start: %d", key_start_flag);
        // OLED_ShowString(0, 6, (uint8_t *)oled_buff, 16);

        if (CurrentMode == NextMode) // 如果当前模式与下一个模式相同，则继续执行当前模式
        {
            switch (CurrentMode)
            {
            case 1:
                Mode1_Loop();
                break;
            case 2:
                Mode2_Loop();
                break;
            case 3:
                Mode3_Loop();
                break;
            case 4:
                Mode4_Loop();
                break;
            case 5:
                Mode5_Loop();
                break;
            case 6:
                Mode6_Loop();
                break;
            default:
                break;
            }
        }
        else // 如果当前模式与下一个模式不同，则切换到下一个模式
        {
            switch (CurrentMode)
            {
            case 1:
                Mode1_Exit();
                break;
            case 2:
                Mode2_Exit();
                break;
            case 3:
                Mode3_Exit();
                break;
            case 4:
                Mode4_Exit();
                break;
            case 5:
                Mode5_Exit();
                break;
            case 6:
                Mode6_Exit();
            default:
                break;
            }

            switch (NextMode)
            {
            case 1:
                Mode1_Init();
                break;
            case 2:
                Mode2_Init();
                break;
            case 3:
                Mode3_Init();
                break;
            case 4:
                Mode4_Init();
                break;
            case 5:
                Mode5_Init();
                break;
            case 6:
                Mode6_Init();
                break;
            default:
                break;
            }

            CurrentMode = NextMode; // 更新当前模式为下一个模式
        }
    }
}
