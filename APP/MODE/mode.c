#include "mode.h"

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

static uint32_t question2_start_time = 0;                              // 记录模式2开始的时间
uint8_t question2_flag = 0;                                            // 模式2的标志位，0表示未开始，1表示已开始
const short question2_trace_weights[8] = {-8, -6, -2, -1, 1, 2, 6, 8}; // 模式2的循迹权重数组

pid_t question2_pid_heading;                          // 模式2的PID控制器实例，用于调整小车的转向
static float QUESTION2_HEADING_KP = (4.5f * 0.6f);    // 模式2的PID控制器比例系数
static float QUESTION2_HEADING_KI = (0.035f);         // 模式2的PID控制器积分系数
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
    delay_ms(300);                     // 等待电机停止
    motor_set_direction(1, 0);         // 设置左轮电机方向为停止
    motor_set_direction(2, 0);         // 设置右轮电机方向为停止
    pid_reset(&question2_pid_heading); // 重置模式2的PID控制器
}

/* ---------------------------------------------------------------- */
/*                             模式3：第三问代码                            */
/* ---------------------------------------------------------------- */
static uint32_t question3_start_time = 0; // 记录模式3开始的时间

#define QUESTION3_MOTOR_RiseTime 900  // 模式3的电机上升时间
#define QUESTION3_MOTOR_DiseCLK 120   // 模式3的电机上升脉冲数
#define QUESTION3_MOTOR_FallTime 1000 // 模式3的电机下降时间
#define QUESTION3_MOTOR_FallCLK 120   // 模式3的电机下降脉冲数

void Mode3_Init(void)
{
    OLED_Clear();
    // 在这里添加模式3的初始化代码
    OLED_ShowString(0, 0, (uint8_t *)"QUESTION 3", 16);

    question3_start_time = tick_ms; // 记录模式3开始的时间
}

void Mode3_Loop(void)
{
    // 在这里添加模式3的循环代码
    uint16_t elapsed_time = (tick_ms - question3_start_time); // 计算模式3运行的时间（毫秒）

    static uint8_t State = 0; // 电机状态，0表示未开始，1表示上升，2表示下降

    if (elapsed_time <= QUESTION3_MOTOR_RiseTime && State == 0)
    {
        State = 1; // 设置电机状态为下降
        // 在电机上升时间内，设置电机上升位置
        ZDT_MOTOR_Pos_Control(1, ZDT_MOTOR_UP, ZDT_MOTOR_Default_Vel, ZDT_MOTOR_Default_Acc, QUESTION3_MOTOR_DiseCLK,
                              false);
    }
    else if (elapsed_time >= QUESTION3_MOTOR_RiseTime &&
             elapsed_time <= (QUESTION3_MOTOR_RiseTime + QUESTION3_MOTOR_FallTime) && State == 1)
    {
        State = 2; // 设置电机状态为上升
        // 在电机下降时间内，设置电机下降位置
        ZDT_MOTOR_Pos_Control(1, ZDT_MOTOR_DOWN, ZDT_MOTOR_Default_Vel, ZDT_MOTOR_Default_Acc, QUESTION3_MOTOR_FallCLK,
                              false);
    }
    else if (State == 2 && elapsed_time > (QUESTION3_MOTOR_RiseTime + QUESTION3_MOTOR_FallTime))
    {
        State = 0; // 重置电机状态为未开始
        // 电机回0
        ZDT_MOTOR_Pos_Control(1, ZDT_MOTOR_DOWN, ZDT_MOTOR_Default_Vel, ZDT_MOTOR_Default_Acc, 10, false);
    }

    uint8_t KeyNum = Key_GetNum();
    if (KeyNum == 4) // 按键4被按下
    {
        NextMode = 1; // 切换回模式1，选择问题模式
    }
}
void Mode3_Exit(void)
{
    // 在这里添加模式3的退出代码
    // Emm_V5_Stop_Now(1, false); // 停止步进电机电机
    Emm_V5_Origin_Trigger_Return(1, 0, false);
}

/* ---------------------------------------------------------------- */
/*                             模式4：第四问代码                            */
/* ---------------------------------------------------------------- */
uint32_t question4_start_time = 0;                                     // 记录模式4开始的时间
uint8_t question4_flag = 0;                                            // 模式4的标志位，0表示未开始，1表示已开始
const short question4_trace_weights[8] = {-8, -6, -2, -1, 1, 2, 6, 8}; // 模式4的循迹权重数组

pid_t question4_pid_heading;                          // 模式4的PID控制器实例，用于调整小车的转向
static float QUESTION4_HEADING_KP = (4.5f * 0.6f);    // 模式4的PID控制器比例系数
static float QUESTION4_HEADING_KI = (0.035f);         // 模式4的PID控制器积分系数
static float QUESTION4_HEADING_KD = (18.0f * 0.3f);   // 模式4的PID控制器微分系数
static float QUESTION4_HEADING_OUTPUT_MAX = (40.0f);  // 模式4的PID控制器输出最大值
static float QUESTION4_HEADING_OUTPUT_MIN = (-40.0f); // 模式4的PID控制器输出最小值

void Mode4_Init(void)
{
    // 在这里添加模式4的初始化代码
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Question 4", 16);

    question4_start_time = tick_ms;    // 记录模式4开始的时间
    pid_reset(&question4_pid_heading); // 重置模式4的PID控制器
    // 初始化模式4的PID控制器
    pid_init(&question4_pid_heading, PID_POSITION, QUESTION4_HEADING_KP, QUESTION4_HEADING_KI, QUESTION4_HEADING_KD,
             QUESTION4_HEADING_OUTPUT_MAX, QUESTION4_HEADING_OUTPUT_MIN);
    pid_set_setpoint(&question4_pid_heading, 0.0f); // 设置目标航向

    question4_flag = 1; // 设置模式4的标志位为已开始

    motor_set_direction(1, 1); // 设置左轮电机方向为正转
    motor_set_direction(2, 1); // 设置右轮电机方向为正转
}

void Mode4_Loop(void)
{
    // 在这里添加模式4的循环代码
    uint32_t time = (float)(tick_ms - question4_start_time);
}

void Mode4_Exit(void)
{
    // 在这里添加模式4的退出代码
}

/* ---------------------------------------------------------------- */
/*                             模式5：第五问代码                            */
/* ---------------------------------------------------------------- */

pid_t question5_pid_motor;                         // 模式5的PID控制器实例，用于调节电机参数
static float QUESTION5_MOTOR_KP = 1.0f;            // 模式5的PID控制器比例系数
static float QUESTION5_MOTOR_KI = 0.0f;            // 模式5的PID控制器积分系数
static float QUESTION5_MOTOR_KD = 0.0f;            // 模式5的PID控制器微分系数
static float QUESTION5_MOTOR_OUTPUT_MAX = 360.0f;  // 模式5的PID控制器输出最大值
static float QUESTION5_MOTOR_OUTPUT_MIN = -360.0f; // 模式5的PID控制器输出最小值

void Mode5_Init(void)
{
    // 在这里添加模式5的初始化代码
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Question 5", 16);

    pid_init(&question5_pid_motor, PID_POSITION, QUESTION5_MOTOR_KP, QUESTION5_MOTOR_KI, QUESTION5_MOTOR_KD,
             QUESTION5_MOTOR_OUTPUT_MAX, QUESTION5_MOTOR_OUTPUT_MIN); // 初始化模式5的PID控制器
    pid_set_setpoint(&question5_pid_motor, 0);                        // 设置模式5的PID控制器目标值为0
}

void Mode5_Loop(void)
{
    // 在这里添加模式5的循环代码
    // 轮询检查是否有新帧到达
    if (uart_maixcam_rx_done)
    {

        // uart_rx_buff[0..1]: 帧头 0xFF 0xFE
        uint8_t sign = uart_rx_buff[2];           // 0x00=正, 0x01=负
        uint16_t data = uart_rx_buff[3]           // 低字节
                        | (uart_rx_buff[4] << 8); // 高字节
        // uart_rx_buff[5..6]: 帧尾 0xFE 0xFF

        // 处理数据...

        int pross_data = (sign == 0) ? data : -data; // 根据符号位处理数据
        sprintf((char *)uart_tx_buff, "processed:%d\r\n", pross_data);
        UART_print_string(DEBUG_INST, (char *)uart_tx_buff);
        memset((char *)uart_tx_buff, 0, sizeof(uart_tx_buff)); // 清空发送缓冲区

        float steering = pid_calculate(&question5_pid_motor, (float)pross_data); // 使用PID计算转向调整值

        sprintf((char *)uart_tx_buff, "steering:%.2f\r\n", steering);
        UART_print_string(DEBUG_INST, (char *)uart_tx_buff);
        memset((char *)uart_tx_buff, 0, sizeof(uart_tx_buff)); // 清空发送缓冲区

        if (steering > 0)
        {
            if (steering > 360)
            {
                steering = 360; // 限制最大值
            }
            Emm_V5_Pos_Control(1, 0, 10, 10, steering, 1, false);
        }
        else
        {

            if (steering < -360)
            {
                steering = -360; // 限制最小值
            }
            Emm_V5_Pos_Control(1, 1, 10, 10, -steering, 1, false);
        }

        uart_maixcam_rx_done = 0; // 清除标志，准备接收下一帧
    }

    uint8_t KeyNum = Key_GetNum();
    if (KeyNum == 4) // 按键4被按下
    {
        NextMode = 1; // 切换回模式1，选择问题模式
    }
}

void Mode5_Exit(void)
{
    // 在这里添加模式5的退出代码
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