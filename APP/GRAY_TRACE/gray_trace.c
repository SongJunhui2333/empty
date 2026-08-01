#include "gray_trace.h"
// #include "../Drivers/VOFA/vofa.h"

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
void gray_trace(uint8_t *sensorValues, const float *weights, pid_t *pid_controller, uint16_t motor_l_base_speed,
                uint16_t motor_r_base_speed)
{

    uint8_t gray_sum = get_gray_num(sensorValues);

    float weighted_sum = 0; // 加权和

    for (int i = 0; i < 8; i++)
    {
        weighted_sum += (float)sensorValues[i] * weights[i];
    }

    if (gray_sum == 0)
    {
        gray_sum = 1; // 避免除以零
    }

    float calcu = (float)weighted_sum / (float)gray_sum; // 计算偏差值

    /* ---- Mode 5/6 一阶低通滤波: 平滑灰度偏差值, 减少转向抖动 ---- */
    {
        static float last_filtered_calcu = 0.0f;
        static uint8_t was_active = 0;

        if (question5_flag == 1 || question6_flag == 1)
        {
            /* 刚进入滤波模式时用当前偏差值初始化滤波器, 避免从旧值跳变 */
            if (!was_active)
            {
                last_filtered_calcu = calcu;
                was_active = 1;
            }

            float a = 0.15f; /* 滤波系数: 0~1, 越小越平滑 (0.15 ≈ 140ms 时间常数) */
            calcu = a * calcu + (1.0f - a) * last_filtered_calcu;
            last_filtered_calcu = calcu;
        }
        else
        {
            was_active = 0; /* 退出滤波模式, 下次进入重新初始化 */
        }
    }

    /* ---- Mode 2 一阶低通滤波: 滤波系数 0.3, 响应更快 ---- */
    {
        static float last_filtered_calcu = 0.0f;
        static uint8_t was_q2 = 0;

        if (question2_flag == 1)
        {
            if (!was_q2)
            {
                last_filtered_calcu = calcu;
                was_q2 = 1;
            }

            float a = 0.3f; /* 滤波系数: 0.3 ≈ 60ms 时间常数 */
            calcu = a * calcu + (1.0f - a) * last_filtered_calcu;
            last_filtered_calcu = calcu;
        }
        else
        {
            was_q2 = 0;
        }
    }

    float steering = pid_calculate(pid_controller, calcu); // 使用PID计算转向调整值

    // /* ---- VOFA+ 实时数据发送: Mode 5 小车转向PID (5通道 JustFloat) ---- */
    // /* ch0:目标值  ch1:灰度偏差  ch2:steering  ch3:PID误差  ch4:黑线数 */
    // if (question5_flag == 1 && !vofa_tx_busy)
    // {
    //     float vofa_data[5];
    //     vofa_data[0] = pid_controller->setpoint;
    //     vofa_data[1] = calcu;
    //     vofa_data[2] = steering;
    //     vofa_data[3] = pid_controller->error;
    //     vofa_data[4] = (float)gray_sum;
    //     vofa_send_frame(vofa_data, 5);
    // }

    // /* ---- VOFA+ 实时数据发送: Mode 2 小车转向PID (5通道 JustFloat) ---- */
    // /* ch0:目标值  ch1:灰度偏差  ch2:steering  ch3:PID误差  ch4:黑线数 */
    // if (question2_flag == 1)
    // {
    //     float vofa_data[5];
    //     vofa_data[0] = pid_controller->setpoint;
    //     vofa_data[1] = calcu;
    //     vofa_data[2] = steering;
    //     vofa_data[3] = pid_controller->error;
    //     vofa_data[4] = (float)gray_sum;
    //     vofa_send_frame(vofa_data, 5);
    // }

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

            if ((gray_sum >= QUESTION2_BLACK_LINE_THRESHOLD) &&
                tick_ms - question2_start_time > 10000) /* 检测到阈值以上黑线则认为完成 */
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
        else if (question5_flag == 1)
        {
            // 缓启动：每隔20ms线性插值，使电机速度从0平滑上升到最大速度
            uint32_t elapsed = tick_ms - question5_start_time;
            if (elapsed < QUESTION5_RAMP_TIME_MS)
            {
                question5_current_speed =
                    (uint16_t)((uint32_t)QUESTION5_MOTOR_MAX_SPEED * elapsed / QUESTION5_RAMP_TIME_MS);
            }
            else
            {
                question5_current_speed = QUESTION5_MOTOR_MAX_SPEED;
            }

            gray_trace(gw_gray_sensor, question5_trace_weights, &question5_pid_heading, question5_current_speed,
                       question5_current_speed);
        }
        else if (question6_flag == 1)
        {
            // 缓启动：每隔20ms线性插值，使电机速度从0平滑上升到最大速度
            uint32_t elapsed = tick_ms - question6_start_time;
            if (elapsed < QUESTION6_RAMP_TIME_MS)
            {
                question6_current_speed =
                    (uint16_t)((uint32_t)QUESTION6_MOTOR_MAX_SPEED * elapsed / QUESTION6_RAMP_TIME_MS);
            }
            else
            {
                question6_current_speed = QUESTION6_MOTOR_MAX_SPEED;
            }

            gray_trace(gw_gray_sensor, question6_trace_weights, &question6_pid_heading, question6_current_speed,
                       question6_current_speed);
        }
    }
}