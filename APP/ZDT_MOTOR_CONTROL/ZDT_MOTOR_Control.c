#include "ZDT_MOTOR_Control.h"

void ZDT_MOTOR_Pos_Control(uint8_t addr, ZDT_MOTOR_Direction dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF)
{
    // 限制目标位置在允许范围内
    if (clk > ZDT_MOTOR_Limit_Pos)
    {
        clk = ZDT_MOTOR_Limit_Pos;
    }
    else if (clk < 0)
    {
        clk = 0;
    }

    Emm_V5_Pos_Control(addr, dir, vel, acc, clk, 1, false);
}