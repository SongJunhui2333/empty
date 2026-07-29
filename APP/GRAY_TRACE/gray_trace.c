#include "gray_trace.h"

uint8_t get_gray_num(uint8_t *sensorValues)
{
    uint8_t num = 0;
    for (int i = 0; i < 8; i++)
    {
        if (sensorValues[i] == 1)
        {
            num++;
        }
    }
    return num;
}

void gray_trace(uint8_t *sensorValues)
{
    const short weights[8] = {-7, -6, -4, -1, 1, 4, 6, 7};
    int16_t weighted_sum = 0; // 加权和

    for (int i = 0; i < 8; i++)
    {
        weighted_sum += sensorValues[i] * weights[i];
    }
    weighted_sum *= 2; // 将权重乘以2，增加对偏离的敏感度

    pid_set_setpoint(&pid_motor_l, motor_l_base_speed + weighted_sum);
    pid_set_setpoint(&pid_motor_r, motor_r_base_speed - weighted_sum);
}

void gray_trace_tick()
{
    static uint8_t Count;
    static uint8_t stop_flag = 0;

    Count++;
    if (Count >= 20) // 每20ms进行一次循迹计算
    {
        Count = 0;

        uint8_t gray_sum = get_gray_num(gw_gray_sensor);
        if (stop_flag == 0)
        {
            if (gray_sum >= 3 && tick_ms > 500) // 如果检测到3个以上的传感器为黑色，且已经运行超过0.5秒，则停止循迹
            {
                stop_flag = 1;
                pid_set_setpoint(&pid_motor_l, 0);
                pid_set_setpoint(&pid_motor_r, 0);
            }
            else
            {
                gray_trace(gw_gray_sensor);
            }
        }
    }
}