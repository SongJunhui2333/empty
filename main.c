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
    // WIT_Init();

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

    NVIC_EnableIRQ(UART_MAIXCAM_INST_INT_IRQN); // 使能MAIXCAM UART中断

    // 初始化电机参数
    Emm_V5_En_Control(1, true, false);

    Emm_V5_Origin_Trigger_Return(1, 0, false);

    my_delay_ms(500); // 等待0.5秒使系统上电完成

    // Emm_V5_Pos_Control(1, 0, 100, 0, 120, 1, false);
    // ZDT_MOTOR_Pos_Control(1, ZDT_MOTOR_UP, ZDT_MOTOR_Default_Vel, ZDT_MOTOR_Default_Acc, 120, 1);

    while (1)
    {
        // // 主循环

        // UART_print_string(UART_MAIXCAM_INST, "Main Loop Running...\r\n");

        // Emm_V5_Pos_Control(1, 0, 100, 0, 120, 1, false);

        // delay_ms(1000); // 延时1秒
        // Emm_V5_Pos_Control(1, 1, 100, 0, 120, 1, false);

        // delay_ms(50); // 延时500毫秒


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
            case 7:
                Mode7_Loop();
                break;
            case 8:
                Mode8_Loop();
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
                break;
            case 7:
                Mode7_Exit();
                break;
            case 8:
                Mode8_Exit();
                break;
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
            case 7:
                Mode7_Init();
                break;
            case 8:
                Mode8_Init();
                break;
            default:
                break;
            }

            CurrentMode = NextMode; // 更新当前模式为下一个模式
        }
    }
}
