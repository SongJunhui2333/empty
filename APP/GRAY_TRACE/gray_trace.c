#include "gray_trace.h"

/**
 * @brief 计算灰度传感器中检测到的黑线数量。
 * @param sensorValues 灰度传感器的值数组，长度为8。
 * @return 检测到的黑线数量。
 */
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

/**
 * @brief 进行循迹控制，根据灰度传感器的值调整电机速度。
 * @param sensorValues 灰度传感器的值数组，长度为8。
 * @param motor_l_base_speed 左轮电机的基础速度。
 * @param motor_r_base_speed 右轮电机的基础速度。
 */
void gray_trace(uint8_t *sensorValues, const short *weights, pid_t *pid_controller, uint16_t motor_l_base_speed,
                uint16_t motor_r_base_speed)
{

    uint8_t gray_sum = get_gray_num(sensorValues);

    int16_t weighted_sum = 0; // 加权和

    for (int i = 0; i < 8; i++)
    {
        weighted_sum += sensorValues[i] * weights[i];
    }

    if (gray_sum == 0)
    {
        gray_sum = 1; // 避免除以零
    }

    float calcu = (float)weighted_sum / (float)gray_sum; // 计算偏差值

    float steering = pid_calculate(pid_controller, calcu); // 使用PID计算转向调整值

    sprintf((char *)uart_tx_buff, "checkingData: %.2f,%.2f\r\n", calcu, steering);
    UART_print_string(DEBUG_INST, (char *)uart_tx_buff);
    memset((void *)uart_tx_buff, 0, 128);

    pid_set_setpoint(&pid_motor_l, motor_l_base_speed - steering);
    pid_set_setpoint(&pid_motor_r, motor_r_base_speed + steering);
}

void gray_trace_tick()
{
    static uint8_t Count;

    Count++;
    if (Count >= 20) // 每20ms进行一次循迹计算
    {
        Count = 0;

        if (question2_flag == 1) // 如果模式2未开始，则不进行循迹
        {
            uint8_t gray_sum = get_gray_num(gw_gray_sensor);
            gray_trace(gw_gray_sensor, question2_trace_weights, &question2_pid_heading, QUESTION2_MOTOR_BASE_SPEED,
                       QUESTION2_MOTOR_BASE_SPEED);

            if (gray_sum >= 4) // 如果检测到3个或以上的黑线，则认为问题2完成，切换到模式1
            {
                NextMode = 1; // 切换到模式1，重新选择问题
            }
        }
        else if (question4_flag == 1) // 如果模式4未开始，则不进行循迹
        {
            // uint8_t gray_sum = get_gray_num(gw_gray_sensor);

            // 缓启动：每隔20ms线性插值，使电机速度从0平滑上升到最大速度
            uint32_t elapsed = tick_ms - question4_start_time;
            if (elapsed < QUESTION4_RAMP_TIME_MS)
            {
                question4_current_speed =
                    (uint16_t)((uint32_t)QUESTION4_MOTOR_MAX_SPEED * elapsed / QUESTION4_RAMP_TIME_MS);
            }
            else
            {
                question4_current_speed = QUESTION4_MOTOR_MAX_SPEED;
            }

            // // 更新左右电机PID目标速度为当前的缓启动速度
            // pid_set_setpoint(&pid_motor_l, (float)question4_current_speed);
            // pid_set_setpoint(&pid_motor_r, (float)question4_current_speed);
            gray_trace(gw_gray_sensor, question4_trace_weights, &question4_pid_heading, question4_current_speed,
                       question4_current_speed);
        }
    }
}