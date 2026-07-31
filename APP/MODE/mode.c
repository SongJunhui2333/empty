#include "mode.h"
#include <stdlib.h>

uint8_t CurrentMode = 0; // 当前模式编号，0表示未选择模式
uint8_t NextMode = 1;    // 下一个模式编号，0表示未选择模式

// uint8_t KeyNum;

/* ---------------------------------------------------------------- */
/*                             模式1：初始选择界面                          */
/* ---------------------------------------------------------------- */

uint8_t Q_Select_Num = 0;       // 问题选择编号，0表示未选择问题
static float Time_Record = 0.0; // 记录模式2的运行时间

void Mode1_Init(void)
{
    // 初始选择界面初始化代码
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Select Mode:", 16);
}

void Mode1_Loop(void)
{
    // 在这里添加模式1的循环代码
    uint8_t KeyNum = Key_GetNum();
    if (KeyNum == 1) // 按键1被按下
    {
        Q_Select_Num = (Q_Select_Num + 1) % 8; // 循环选择问题编号
    }
    else if (KeyNum == 2) // 按键2被按下
    {
        Q_Select_Num = (Q_Select_Num + 7) % 8; // 循环选择问题编号
    }

    if (KeyNum == 3) // 按键3被按下
    {
        NextMode = Q_Select_Num; // 切换到选择的问题模式
    }
    OLED_ShowString(0, 2, (uint8_t *)"Question Num: ", 16);

    sprintf((char *)oled_show_buff, "%d", Q_Select_Num);
    OLED_ShowString(0, 4, (uint8_t *)oled_show_buff, 16);

    sprintf((char *)oled_show_buff, "Time: %.2f s", Time_Record);
    OLED_ShowString(0, 6, (uint8_t *)oled_show_buff, 16);
}

void Mode1_Exit(void)
{
    // 在这里添加模式1的退出代码
    Q_Select_Num = 0; // 重置问题选择编号
    Time_Record = 0;  // 重置模式2的运行时间
}

/* ---------------------------------------------------------------- */
/*                             模式2：第二问代码                            */
/* ---------------------------------------------------------------- */
uint16_t QUESTION2_MOTOR_BASE_SPEED = 60;                              // 模式2的电机基础速度
static uint32_t question2_start_time = 0;                              // 记录模式2开始的时间
uint8_t question2_flag = 0;                                            // 模式2的标志位，0表示未开始，1表示已开始
const float question2_trace_weights[8] = {-9, -8, -2, -1, 1, 2, 8, 9}; // 模式2的循迹权重数组

pid_t question2_pid_heading;                          // 模式2的PID控制器实例，用于调整小车的转向
static float QUESTION2_HEADING_KP = (4.5f * 0.6f);    // 模式2的PID控制器比例系数
static float QUESTION2_HEADING_KI = (0.005f);         // 模式2的PID控制器积分系数
static float QUESTION2_HEADING_KD = (18.0f * 0.3f);   // 模式2的PID控制器微分系数
static float QUESTION2_HEADING_OUTPUT_MAX = (40.0f);  // 模式2的PID控制器输出最大值
static float QUESTION2_HEADING_OUTPUT_MIN = (-40.0f); // 模式2的PID控制器输出最小值

void Mode2_Init(void)
{
    // 在这里添加模式2的初始化代码
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Question 2", 16);
    question2_start_time = tick_ms;

    pid_reset(&question2_pid_heading); // 重置模式2的PID控制器
    // 初始化模式2的PID控制器
    pid_init(&question2_pid_heading, PID_POSITION, QUESTION2_HEADING_KP, QUESTION2_HEADING_KI, QUESTION2_HEADING_KD,
             QUESTION2_HEADING_OUTPUT_MAX, QUESTION2_HEADING_OUTPUT_MIN);
    pid_set_integral_limit(&question2_pid_heading, QUESTION2_HEADING_OUTPUT_MAX * 0.1f,
                           QUESTION2_HEADING_OUTPUT_MIN * 0.1f);
    pid_set_setpoint(&question2_pid_heading, QUESTION2_HEADING_SETPOINT); // 设置目标航向

    question2_flag = 1; // 设置模式2的标志位为已开始

    motor_set_direction(1, 1); // 设置左轮电机方向为正转
    motor_set_direction(2, 1); // 设置右轮电机方向为正转

    buzzer_on(); // 打开蜂鸣器，提示模式2开始
}

void Mode2_Loop(void)
{
    if (tick_ms - question2_start_time > 500) // 500毫秒后关闭蜂鸣器
    {
        buzzer_off(); // 关闭蜂鸣器
    }
    // 在这里添加模式2的循环代码
    float_t time = (float)(tick_ms - question2_start_time) / 1000.0;
    sprintf((char *)oled_show_buff, "time: %.2f", time);
    OLED_ShowString(0, 2, (uint8_t *)oled_show_buff, 16);
}

void Mode2_Exit(void)
{
    // 在这里添加模式2的退出代码
    Time_Record = (float)(tick_ms - question2_start_time) / 1000.0; // 记录模式2的运行时间
    question2_start_time = 0;                                       // 重置模式2开始的时间
    question2_flag = 0;                                             // 重置模式2的标志位

    pid_set_setpoint(&pid_motor_l, 0); // 停止左轮电机
    pid_set_setpoint(&pid_motor_r, 0); // 停止右轮电机
    // delay_ms(300);                     // 等待电机停止
    // motor_set_direction(1, 0);         // 设置左轮电机方向为停止
    // motor_set_direction(2, 0);         // 设置右轮电机方向为停止
    pid_reset(&question2_pid_heading); // 重置模式2的PID控制器
}

/* ---------------------------------------------------------------- */
/*                             模式3：第三问代码                            */
/*                  阶段1:管道下降 → 阶段2:管道上升 → 阶段3:球控              */
/* ---------------------------------------------------------------- */

/* ---------- 可调参数 ---------- */
#define QUESTION3_PHASE1_TIME_MS 800  // 阶段1持续时间（毫秒）
#define QUESTION3_PHASE1_PULSES  110   // 阶段1管道下降脉冲数
#define QUESTION3_PHASE2_TIME_MS 1000  // 阶段2持续时间（毫秒）
#define QUESTION3_PHASE2_PULSES  110   // 阶段2管道上升脉冲数
#define QUESTION3_MOTOR_MAX_PULSES 290 // 电机正反转最大脉冲限幅

/* ---------- 球控PID ---------- */
static pid_t question3_pid_motor;
static float QUESTION3_MOTOR_KP = 2.8f;
static float QUESTION3_MOTOR_KI = 0.01f;
static float QUESTION3_MOTOR_KD = 15.0f;
static float QUESTION3_MOTOR_OUTPUT_MAX = 290.0f;
static float QUESTION3_MOTOR_OUTPUT_MIN = -290.0f;

/* ---------- 状态定义 ---------- */
#define Q3_STATE_IDLE         0  // 等待按键
#define Q3_STATE_PHASE1_DOWN  1  // 管道下降
#define Q3_STATE_PHASE2_UP    2  // 管道上升
#define Q3_STATE_BALL_CONTROL 3  // 小球平衡控制

void Mode3_Init(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"QUESTION 3", 16);
    OLED_ShowString(0, 3, (uint8_t *)"Key3: Start", 8);
    OLED_ShowString(0, 5, (uint8_t *)"Key4: Exit", 8);

    /* ---- 初始化 PID ---- */
    pid_init(&question3_pid_motor, PID_POSITION,
             QUESTION3_MOTOR_KP, QUESTION3_MOTOR_KI, QUESTION3_MOTOR_KD,
             QUESTION3_MOTOR_OUTPUT_MAX, QUESTION3_MOTOR_OUTPUT_MIN);
    pid_set_setpoint(&question3_pid_motor, 0.0f);

    Emm_V5_Origin_Trigger_Return(1, 0, false);
}

void Mode3_Loop(void)
{
    static uint32_t test_start_time = 0;
    static uint8_t  test_flag  = 0;
    static uint8_t  state      = Q3_STATE_IDLE;

    uint32_t elapsed = tick_ms - test_start_time;

    /* ---- 蜂鸣器: 500ms后关闭 ---- */
    if (elapsed > 500 && test_flag == 1) { buzzer_off(); }

    /* ==== 阶段1: 管道下降 ==== */
    if (elapsed <= QUESTION3_PHASE1_TIME_MS && state == Q3_STATE_IDLE && test_flag == 1)
    {
        state = Q3_STATE_PHASE1_DOWN;
        uint32_t pulses = (QUESTION3_PHASE1_PULSES > QUESTION3_MOTOR_MAX_PULSES)
                            ? QUESTION3_MOTOR_MAX_PULSES : QUESTION3_PHASE1_PULSES;
        Emm_V5_Pos_Control(1, 0, 500, 50, pulses, 1, false);
    }
    /* ==== 阶段2: 管道上升 ==== */
    else if (elapsed > QUESTION3_PHASE1_TIME_MS &&
             elapsed <= (QUESTION3_PHASE1_TIME_MS + QUESTION3_PHASE2_TIME_MS) &&
             state == Q3_STATE_PHASE1_DOWN && test_flag == 1)
    {
        state = Q3_STATE_PHASE2_UP;
        uint32_t pulses = (QUESTION3_PHASE2_PULSES > QUESTION3_MOTOR_MAX_PULSES)
                            ? QUESTION3_MOTOR_MAX_PULSES : QUESTION3_PHASE2_PULSES;
        Emm_V5_Pos_Control(1, 1, 500, 50, pulses, 1, false);
    }
    /* ==== 阶段3: 小球平衡控制 ==== */
    else if (state == Q3_STATE_PHASE2_UP &&
             elapsed > (QUESTION3_PHASE1_TIME_MS + QUESTION3_PHASE2_TIME_MS) && test_flag == 1)
    {
        state = Q3_STATE_BALL_CONTROL;
    }

    /* ---- 阶段3: 小球平衡控制循环 ---- */
    if (state == Q3_STATE_BALL_CONTROL)
    {
        static int16_t nudge = 0;

        if (uart_maixcam_rx_done)
        {
            uart_maixcam_rx_done = 0;

            uint8_t  sign1 = uart_rx_buff[2];
            uint16_t raw1  = uart_rx_buff[3] | (uart_rx_buff[4] << 8);
            int16_t  ball_error = (sign1 == 0x01) ? -(int16_t)raw1 : (int16_t)raw1;

            uint8_t  sign2 = uart_rx_buff[5];
            uint16_t raw2  = uart_rx_buff[6] | (uart_rx_buff[7] << 8);
            int16_t  ball_vel = (sign2 == 0x01) ? -(int16_t)raw2 : (int16_t)raw2;

            float motor_pos = pid_calculate(&question3_pid_motor, (float)ball_error);

            // 微扰逻辑
            if (abs(ball_error) > 15 && abs(ball_vel) < 20)
            {
                nudge += (ball_error > 0) ? -5 : 5;
                if (nudge > 80)  nudge = 80;
                if (nudge < -80) nudge = -80;
            }
            else if (abs(ball_error) <= 15)
            {
                nudge = 0;
            }

            motor_pos += (float)nudge;

            if (motor_pos > (float)QUESTION3_MOTOR_MAX_PULSES)
                motor_pos = (float)QUESTION3_MOTOR_MAX_PULSES;
            if (motor_pos < -(float)QUESTION3_MOTOR_MAX_PULSES)
                motor_pos = -(float)QUESTION3_MOTOR_MAX_PULSES;

            if (motor_pos >= 0)
                Emm_V5_Pos_Control(1, 0, 500, 50, (uint32_t)motor_pos, 1, false);
            else
                Emm_V5_Pos_Control(1, 1, 500, 50, (uint32_t)(-motor_pos), 1, false);
        }
    }

    /* ---- 按键处理 ---- */
    uint8_t KeyNum = Key_GetNum();
    if (KeyNum == 4)
    {
        NextMode = 1;
    }
    if (KeyNum == 3)
    {
        test_flag       = 1;
        test_start_time = tick_ms;
        state           = Q3_STATE_IDLE;
        pid_reset(&question3_pid_motor);
        pid_set_setpoint(&question3_pid_motor, 0.0f);
        buzzer_on();
    }
}

void Mode3_Exit(void)
{
    Emm_V5_Origin_Trigger_Return(1, 0, false);
}

/* ---------------------------------------------------------------- */
/*                             模式4：第四问代码                            */
/* ---------------------------------------------------------------- */
uint32_t question4_start_time = 0;                                     // 记录模式4开始的时间
uint8_t question4_flag = 0;                                            // 模式4的标志位，0表示未开始，1表示已开始
const float question4_trace_weights[8] = {-8, -6, -2, -1, 1, 2, 6, 8}; // 模式4的循迹权重数组
uint16_t question4_current_speed = 0;                                  // 模式4当前的电机速度目标值

pid_t question4_pid_heading;                          // 模式4的PID控制器实例，用于调整小车的转向
static float QUESTION4_HEADING_KP = (4.5f * 0.6f);    // 模式4的PID控制器比例系数
static float QUESTION4_HEADING_KI = (0.0f);           // 模式4的PID控制器积分系数
static float QUESTION4_HEADING_KD = (18.0f);          // 模式4的PID控制器微分系数
static float QUESTION4_HEADING_OUTPUT_MAX = (40.0f);  // 模式4的PID控制器输出最大值
static float QUESTION4_HEADING_OUTPUT_MIN = (-40.0f); // 模式4的PID控制器输出最小值

void Mode4_Init(void)
{
    // 在这里添加模式4的初始化代码
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Question 4", 16);

    question4_start_time = tick_ms;    // 记录模式4开始的时间
    question4_current_speed = 0;       // 重置缓启动速度
    pid_reset(&question4_pid_heading); // 重置模式4的PID控制器
    // 初始化模式4的PID控制器
    pid_init(&question4_pid_heading, PID_POSITION, QUESTION4_HEADING_KP, QUESTION4_HEADING_KI, QUESTION4_HEADING_KD,
             QUESTION4_HEADING_OUTPUT_MAX, QUESTION4_HEADING_OUTPUT_MIN);
    pid_set_setpoint(&question4_pid_heading, 0.0f); // 设置目标航向

    question4_flag = 1; // 设置模式4的标志位为已开始

    motor_set_direction(1, 1); // 设置左轮电机方向为正转
    motor_set_direction(2, 1); // 设置右轮电机方向为正转

    buzzer_on();
}

void Mode4_Loop(void)
{
    // 在这里添加模式4的循环代码
    // uint32_t time = (float)(tick_ms - question4_start_time);
    if (tick_ms - question4_start_time > 500) // 500毫秒后关闭蜂鸣器
    {
        buzzer_off();
    }
}

void Mode4_Exit(void)
{
    // 在这里添加模式4的退出代码
    question4_flag = 0;                // 重置模式4的标志位
    pid_set_setpoint(&pid_motor_l, 0); // 停止左轮电机
    pid_set_setpoint(&pid_motor_r, 0); // 停止右轮电机
    delay_ms(300);                     // 等待电机停止
    motor_set_direction(1, 0);         // 设置左轮电机方向为停止
    motor_set_direction(2, 0);         // 设置右轮电机方向为停止
    pid_reset(&question4_pid_heading); // 重置模式4的PID控制器
}

/* ---------------------------------------------------------------- */
/*                             模式5：第五问代码                            */
/* ---------------------------------------------------------------- */

pid_t question5_pid_motor;                         // 球杆系统PID控制器
static float QUESTION5_MOTOR_KP = 4.5f;            // 比例系数：球偏差 → 电机脉冲
static float QUESTION5_MOTOR_KI = 0.005f;           // 积分系数：缓慢消除稳态误差（抗饱和已内置）
static float QUESTION5_MOTOR_KD = 22.0f;           // 微分系数：利用球速提供阻尼
static float QUESTION5_MOTOR_OUTPUT_MAX = 290.0f;  // 电机正方向最大脉冲
static float QUESTION5_MOTOR_OUTPUT_MIN = -290.0f; // 电机负方向最大脉冲

uint32_t question5_start_time = 0;                                     // 记录模式5开始的时间
uint8_t question5_flag = 0;                                            // 模式5的标志位，0表示未开始，1表示已开始
const float question5_trace_weights[8] = {-8, -6, -3, -1, 1, 3, 6, 8}; // 模式5的循迹权重数组
uint16_t question5_current_speed = 0;                                  // 模式5当前的电机速度目标值

pid_t question5_pid_heading;                          // 模式5的PID控制器实例，用于调整小车的转向
static float QUESTION5_HEADING_KP = 3.5f;             // 比例系数: 提高弯道响应
static float QUESTION5_HEADING_KI = 0.01f;            // 积分系数: 加速消除稳态偏置
static float QUESTION5_HEADING_KD = 10.0f;            // 微分系数: 增强阻尼防振荡
static float QUESTION5_HEADING_OUTPUT_MAX = (40.0f);  // 输出最大值
static float QUESTION5_HEADING_OUTPUT_MIN = (-40.0f); // 输出最小值

void Mode5_Init(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Question 5", 16);

    // 初始化球杆系统PID控制器
    pid_init(&question5_pid_motor, PID_POSITION, QUESTION5_MOTOR_KP, QUESTION5_MOTOR_KI, QUESTION5_MOTOR_KD,
             QUESTION5_MOTOR_OUTPUT_MAX, QUESTION5_MOTOR_OUTPUT_MIN);
    pid_set_setpoint(&question5_pid_motor, 0.0f); // 目标：小球在管中心（偏差=0）

    // 初始化模式5的PID控制器
    pid_init(&question5_pid_heading, PID_POSITION, QUESTION5_HEADING_KP, QUESTION5_HEADING_KI, QUESTION5_HEADING_KD,
             QUESTION5_HEADING_OUTPUT_MAX, QUESTION5_HEADING_OUTPUT_MIN);
    pid_set_setpoint(&question5_pid_heading, 0.0f); // 设置目标航向

    question5_start_time = tick_ms; // 记录模式5开始的时间
    question5_flag = 1;             // 设置模式5的标志位为已开始

    motor_set_direction(1, 1); // 设置左轮电机方向为正转
    motor_set_direction(2, 1); // 设置右轮电机方向为正转

    buzzer_on();
}

void Mode5_Loop(void)
{
    if (tick_ms - question5_start_time > 500) // 500毫秒后关闭蜂鸣器
    {
        buzzer_off();
    }

    static int16_t nudge = 0; // 微扰累积量，小球静止且偏差过大时逐步累加

    if (uart_maixcam_rx_done)
    {
        uart_maixcam_rx_done = 0;

        // ---- 解析第一个数据：小球位置偏差 ----
        uint8_t sign1 = uart_rx_buff[2];
        uint16_t raw1 = uart_rx_buff[3] | (uart_rx_buff[4] << 8);
        int16_t ball_error = (sign1 == 0x01) ? -(int16_t)raw1 : (int16_t)raw1;

        // ---- 解析第二个数据：小球当前速度 ----
        uint8_t sign2 = uart_rx_buff[5];
        uint16_t raw2 = uart_rx_buff[6] | (uart_rx_buff[7] << 8);
        int16_t ball_vel = (sign2 == 0x01) ? -(int16_t)raw2 : (int16_t)raw2;

        // ---- PID 计算电机目标位置 ----
        float motor_pos = pid_calculate(&question5_pid_motor, (float)ball_error);

// ---- 微扰逻辑：小球静止且偏差 >15 时，逐步叠加微扰推动小球 ----
#define QUESTION5_VEL_STILL 20 // 判定小球静止的速度阈值
#define QUESTION5_NUDGE_STEP 5 // 每次微扰增量（脉冲）
#define QUESTION5_NUDGE_MAX 80 // 微扰累积上限

        if (abs(ball_error) > 15 && abs(ball_vel) < QUESTION5_VEL_STILL)
        {
            // 小球卡住了：按偏差反方向叠加微扰
            nudge += (ball_error > 0) ? -QUESTION5_NUDGE_STEP : QUESTION5_NUDGE_STEP;

            if (nudge > QUESTION5_NUDGE_MAX)
                nudge = QUESTION5_NUDGE_MAX;
            if (nudge < -QUESTION5_NUDGE_MAX)
                nudge = -QUESTION5_NUDGE_MAX;
        }
        else if (abs(ball_error) <= 15)
        {
            nudge = 0; // 偏差达标，清零微扰
        }
        // else: 球在运动中，保持当前微扰不变，让其继续作用

        motor_pos += (float)nudge;

        // 限幅保护（电机绝对位置）
        if (motor_pos > 330.0f)
            motor_pos = 330.0f;
        if (motor_pos < -330.0f)
            motor_pos = -330.0f;

        // // ---- 调试输出 ----
        // sprintf((char *)uart_tx_buff, "e:%d v:%d m:%.0f n:%d\r\n", ball_error, ball_vel, motor_pos, nudge);
        // UART_print_string(DEBUG_INST, (char *)uart_tx_buff);
        // memset((char *)uart_tx_buff, 0, sizeof(uart_tx_buff));

        // ---- 绝对位置模式驱动电机 ----
        if (motor_pos >= 0)
        {
            Emm_V5_Pos_Control(1, 0, 500, 50, (uint32_t)motor_pos, 1, false);
        }
        else
        {
            Emm_V5_Pos_Control(1, 1, 500, 50, (uint32_t)(-motor_pos), 1, false);
        }
    }

    uint8_t KeyNum = Key_GetNum();
    if (KeyNum == 4)
    {
        NextMode = 1;
    }
}

void Mode5_Exit(void)
{
    question5_flag = 0;          // 重置模式5的标志位
    question5_current_speed = 0; // 重置缓启动速度

    Emm_V5_Origin_Trigger_Return(1, 0, false);
}

/* ---------------------------------------------------------------- */
/*                             模式6：第六问代码                            */
/* ---------------------------------------------------------------- */

void Mode6_Init(void)
{
    // 在这里添加模式6的初始化代码
}

void Mode6_Loop(void)
{
    // 在这里添加模式6的循环代码
}

void Mode6_Exit(void)
{
    // 在这里添加模式6的退出代码
}

/* ---------------------------------------------------------------- */
/*                            模式7：调节参数模式                            */
/* ---------------------------------------------------------------- */

uint8_t Param_Select_Num = 0; // 参数选择编号，0表示未选择参数

void Mode7_Init(void)
{
    // 在这里添加模式7的初始化代码
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Mode 7: Adjust MODE", 16);
    OLED_ShowString(0, 2, (uint8_t *)"Choose Parameter:", 8);
}

void Mode7_Loop(void)
{
    // 在这里添加模式7的循环代码

    // 在这里添加模式1的循环代码
    uint8_t KeyNum = Key_GetNum();
    if (KeyNum == 1) // 按键1被按下
    {
        Param_Select_Num = (Param_Select_Num + 1) % 7; // 循环选择问题编号
    }
    else if (KeyNum == 2) // 按键2被按下
    {
        Param_Select_Num = (Param_Select_Num + 6) % 7; // 循环选择问题编号
    }

    if (KeyNum == 3) // 按键3被按下
    {
        // NextMode = Param_Select_Num; // 切换到选择的问题模式
        if (Param_Select_Num == 3)
        {
            NextMode = 8; // 切换到模式8，调节第三问电机参数
        }
    }
    if (KeyNum == 4) // 按键4被按下
    {
        NextMode = 1; // 切换到选择的问题模式
    }

    sprintf((char *)oled_show_buff, "Param: %d", Param_Select_Num);
    OLED_ShowString(0, 4, (uint8_t *)oled_show_buff, 16);
}

void Mode7_Exit(void)
{
    // 在这里添加模式7的退出代码
    Param_Select_Num = 0; // 重置参数选择编号
}

/* ---------------------------------------------------------------- */
/*                           模式8：第三问调节电机参数                          */
/* ---------------------------------------------------------------- */

void Mode8_Init(void)
{
    // 在这里添加模式8的初始化代码
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Adjust: 3", 16);
    OLED_ShowString(0, 3, (uint8_t *)"K_1: motor_up", 8);
    OLED_ShowString(0, 4, (uint8_t *)"K_2: motor_down", 8);
    OLED_ShowString(0, 5, (uint8_t *)"K_3: SET_ZERO", 8);
    OLED_ShowString(0, 6, (uint8_t *)"K_4: RETURN", 8);
    OLED_ShowString(0, 7, (uint8_t *)"K_5: BACK_ZERO", 8);

    // 初始化电机参数
    Emm_V5_En_Control(1, true, false);

    Emm_V5_Origin_Trigger_Return(1, 0, false);
}

void Mode8_Loop(void)
{
    // 在这里添加模式8的循环代码
    uint8_t KeyNum = Key_GetNum();
    if (KeyNum == 1) // 按键1被按下
    {
        Emm_V5_Pos_Control(1, 0, 500, 10, 10, 0, false);
    }
    else if (KeyNum == 2) // 按键2被按下
    {
        Emm_V5_Pos_Control(1, 1, 500, 10, 10, 0, false);
    }
    else if (KeyNum == 3) // 按键3被按下
    {
        // Emm_V5_Origin_Modify_Params(1, true, 0, 0, 30, 10000, 300, 800, 60, true);
        Emm_V5_Origin_Set_O(1, true);
    }
    if (KeyNum == 4) // 按键4被按下
    {
        NextMode = 7; // 切换回模式7，参数调节入口
    }
    if (KeyNum == 5) // 触发电机回零
    {
        Emm_V5_Origin_Trigger_Return(1, 0, false);
    }
}

void Mode8_Exit(void)
{
    // 在这里添加模式8的退出代码
    // Emm_V5_Stop_Now(1, false);
}