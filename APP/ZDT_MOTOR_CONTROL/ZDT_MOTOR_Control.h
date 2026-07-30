#ifndef ZDT_MOTOR_CONTROL_H
#define ZDT_MOTOR_CONTROL_H

#define ZDT_MOTOR_Limit_Pos 360  // 电机位置限制，单位为脉冲
#define ZDT_MOTOR_Default_Vel 10 // 电机默认速度，单位为RPM
#define ZDT_MOTOR_Default_Acc 10 // 电机默认加速度，单位为RPM/s

#include "Emm_V5.h"
#include "clock.h"
#include "key.h"
#include "motor.h"
#include "oled_hardware_i2c.h"
#include "stdio.h"
#include "ti_msp_dl_config.h"

typedef enum
{
    ZDT_MOTOR_UP = 0,   // 顺时针方向
    ZDT_MOTOR_DOWN = 1, // 逆时针方向
} ZDT_MOTOR_Direction;

void ZDT_MOTOR_Pos_Control(uint8_t addr, ZDT_MOTOR_Direction dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF);

#endif // ZDT_MOTOR_CONTROL_H