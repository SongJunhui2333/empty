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

uint8_t oled_buff[36];

int main(void)
{
    SYSCFG_DL_init();

    /* Don't remove this! */
    Interrupt_Init();
    NVIC_EnableIRQ(GPIOA_INT_IRQn); // 使能GPIOA中断
    NVIC_EnableIRQ(GPIOB_INT_IRQn); // 使能GPIOB中断
    // 初始化OLED显示屏
    OLED_Init();

    motor_init(1);                                                                  // 初始化电机1
    motor_init(2);                                                                  // 初始化电机2
    motor_set_direction(1, 0);                                                      // 设置电机1为正转
    motor_set_direction(2, 0);                                                      // 设置电机2为正转
    pid_init(&pid_motor_l, PID_INCREMENTAL, MOTOR_KP, MOTOR_KI, MOTOR_KD, 4000, 0); // 初始化左轮PID
    pid_init(&pid_motor_r, PID_INCREMENTAL, MOTOR_KP, MOTOR_KI, MOTOR_KD, 4000, 0); // 初始化右轮PID

    pid_set_setpoint(&pid_motor_l, 15); // 设置左轮目标速度
    pid_set_setpoint(&pid_motor_r, 15); // 设置右轮目标速度

    NVIC_EnableIRQ(MOTOR_CONTROL_INST_INT_IRQN); // 使能电机控制中断

    my_delay_ms(500); // 等待0.5秒使系统上电完成

    while (1)
    {
        // // 主循环
        // motor_set_duty(1, 1000); // 设置电机1占空比为2000
        // motor_set_duty(2, 1000); // 设置电机2占空比为2000
        my_delay_ms(1000);       // 延时1秒
    }
}
