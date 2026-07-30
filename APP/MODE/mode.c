#include "mode.h"

uint8_t CurrentMode = 0; // 当前模式编号，0表示未选择模式
uint8_t NextMode = 1;    // 下一个模式编号，0表示未选择模式

// uint8_t KeyNum;

uint8_t Q_Select_Num = 0; // 问题选择编号，0表示未选择问题

/* ---------------------------------------------------------------- */
/*                             模式1：初始选择界面                          */
/* ---------------------------------------------------------------- */

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
        Q_Select_Num = (Q_Select_Num + 1) % 6; // 循环选择问题编号
    }
    else if (KeyNum == 2) // 按键2被按下
    {
        Q_Select_Num = (Q_Select_Num + 5) % 6; // 循环选择问题编号
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

static uint32_t question2_start_time = 0;                              // 记录模式2开始的时间
uint8_t question2_flag = 0;                                            // 模式2的标志位，0表示未开始，1表示已开始
const short question2_trace_weights[8] = {-8, -6, -2, -1, 1, 2, 6, 8}; // 模式2的循迹权重数组

pid_t question2_pid_heading;                  // 模式2的PID控制器实例，用于调整小车的转向
#define QUESTION2_HEADING_KP (4.5f * 0.6f)    // 模式2的PID控制器比例系数
#define QUESTION2_HEADING_KI (0.035f)         // 模式2的PID控制器积分系数
#define QUESTION2_HEADING_KD (18.0f * 0.3f)   // 模式2的PID控制器微分系数
#define QUESTION2_HEADING_OUTPUT_MAX (40.0f)  // 模式2的PID控制器输出最大值
#define QUESTION2_HEADING_OUTPUT_MIN (-40.0f) // 模式2的PID控制器输出最小值

void Mode2_Init(void)
{
    // 在这里添加模式2的初始化代码
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Question 2", 16);
    question2_start_time = tick_ms;

    // 初始化模式2的PID控制器
    pid_init(&question2_pid_heading, PID_POSITION, QUESTION2_HEADING_KP, QUESTION2_HEADING_KI, QUESTION2_HEADING_KD,
             QUESTION2_HEADING_OUTPUT_MAX, QUESTION2_HEADING_OUTPUT_MIN);
    pid_set_setpoint(&question2_pid_heading, QUESTION2_HEADING_SETPOINT); // 设置目标航向

    question2_flag = 1; // 设置模式2的标志位为已开始

    motor_set_direction(1, 1); // 设置左轮电机方向为正转
    motor_set_direction(2, 1); // 设置右轮电机方向为正转
}

void Mode2_Loop(void)
{
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
    pid_reset(&question2_pid_heading); // 重置模式2的PID控制器
}

/* ---------------------------------------------------------------- */
/*                             模式3：第三问代码                            */
/* ---------------------------------------------------------------- */

void Mode3_Init(void)
{
    // 在这里添加模式3的初始化代码
}

void Mode3_Loop(void)
{
    // 在这里添加模式3的循环代码
}

void Mode3_Exit(void)
{
    // 在这里添加模式3的退出代码
}

/* ---------------------------------------------------------------- */
/*                             模式4：第四问代码                            */
/* ---------------------------------------------------------------- */

void Mode4_Init(void)
{
    // 在这里添加模式4的初始化代码
}

void Mode4_Loop(void)
{
    // 在这里添加模式4的循环代码
}

void Mode4_Exit(void)
{
    // 在这里添加模式4的退出代码
}

/* ---------------------------------------------------------------- */
/*                             模式5：第五问代码                            */
/* ---------------------------------------------------------------- */

void Mode5_Init(void)
{
    // 在这里添加模式5的初始化代码
}

void Mode5_Loop(void)
{
    // 在这里添加模式5的循环代码
}

void Mode5_Exit(void)
{
    // 在这里添加模式5的退出代码
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

void Mode7_Init(void)
{
    // 在这里添加模式7的初始化代码
}

void Mode7_Loop(void)
{
    // 在这里添加模式7的循环代码
}

void Mode7_Exit(void)
{
    // 在这里添加模式7的退出代码
}