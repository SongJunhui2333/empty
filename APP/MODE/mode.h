#ifndef MODE_H
#define MODE_H

#include "Emm_V5.h"
#include "ZDT_MOTOR_Control.h"
#include "buzzer.h"
#include "clock.h"
#include "key.h"
#include "motor.h"
#include "oled_hardware_i2c.h"
#include "stdio.h"
#include "string.h"
#include "ti_msp_dl_config.h"

extern uint8_t CurrentMode; // 当前模式编号，0表示未选择模式
extern uint8_t NextMode;    // 下一个模式编号，0表示未选择模式

/* -------------------------- 模式1：初始选择界面 -------------------------- */
void Mode1_Init(void);
void Mode1_Loop(void);
void Mode1_Exit(void);

/* --------------------------- 模式2：第二问代码 -------------------------- */
extern uint8_t  QUESTION2_BLACK_LINE_THRESHOLD; /* 模式2黑线检测阈值 (默认4) */
extern uint16_t QUESTION2_MOTOR_BASE_SPEED;    // 模式2的电机基础速度
extern const float question2_trace_weights[8]; // 模式2的循迹权重数组
extern pid_t question2_pid_heading;            // 模式2的PID控制器实例，用于调整小车的转向
#define QUESTION2_HEADING_SETPOINT (0.0f)      // 模式2的PID控制器目标航向值
extern uint32_t question2_start_time;          // 记录模式2开始的时间

void Mode2_Init(void);
void Mode2_Loop(void);
void Mode2_Exit(void);
extern uint8_t question2_flag; // 问题2的标志位，1表示已开始，0表示未开始

/* --------------------------- 模式3：第三问代码 -------------------------- */
void Mode3_Init(void);
void Mode3_Loop(void);
void Mode3_Exit(void);

/* --------------------------- 模式4：第四问代码 -------------------------- */
extern pid_t question4_pid_heading;            // 模式4的PID控制器实例，用于调整小车的转向
extern uint8_t question4_flag;                 // 模式4的标志位，0表示未开始，1表示已开始
extern const float question4_trace_weights[8]; // 模式4的循迹权重数组
extern uint32_t question4_start_time;          // 记录模式4开始的时间
#define QUESTION4_MOTOR_MAX_SPEED 70           // 模式4的电机最大速度（PID目标值）
#define QUESTION4_RAMP_TIME_MS 7000            // 模式4的缓启动时间（毫秒），可自行修改
extern uint16_t question4_current_speed;       // 模式4当前的电机速度目标值（缓启动过程中实时变化）
void Mode4_Init(void);
void Mode4_Loop(void);
void Mode4_Exit(void);

/* --------------------------- 模式5：第五问代码 -------------------------- */
extern pid_t question5_pid_motor;              // 球杆系统PID控制器
extern pid_t question5_pid_heading;            // 模式5的PID控制器实例，用于调整小车的转向
extern uint8_t question5_flag;                 // 问题5的标志位，1表示已开始，0表示未开始
extern const float question5_trace_weights[8]; // 模式5的循迹权重数组
extern uint32_t question5_start_time;          // 记录模式5开始的时间
#define QUESTION5_MOTOR_MAX_SPEED 45           // 模式5的电机最大速度（PID目标值）
#define QUESTION5_RAMP_TIME_MS 8000            // 模式5的缓启动时间（毫秒），可自行修改
extern uint16_t question5_current_speed;       // 模式5当前的电机速度目标值（缓启动过程中实时变化）
void Mode5_Init(void);
void Mode5_Loop(void);
void Mode5_Exit(void);

/* --------------------------- 模式6：第六问代码 -------------------------- */
extern pid_t question6_pid_motor;              // 球杆系统PID控制器
extern pid_t question6_pid_heading;            // 模式6的PID控制器实例，用于调整小车的转向
extern uint8_t question6_flag;                 // 模式6的标志位，0表示未开始，1表示已开始
extern const float question6_trace_weights[8]; // 模式6的循迹权重数组
extern uint32_t question6_start_time;          // 记录模式6开始的时间
extern uint16_t question6_current_speed;       // 模式6当前的电机速度目标值（缓启动过程中实时变化）
#define QUESTION6_MOTOR_MAX_SPEED 45           // 模式6的电机最大速度（PID目标值）
#define QUESTION6_RAMP_TIME_MS 8000            // 模式6的缓启动时间（毫秒），可自行修改
void Mode6_Init(void);
void Mode6_Loop(void);
void Mode6_Exit(void);

/* -------------------------- 模式7：调参入口代码 -------------------------- */
void Mode7_Init(void);
void Mode7_Loop(void);
void Mode7_Exit(void);

/* ------------------------- 模式8：调节第三问电机参数 ------------------------ */
void Mode8_Init(void);
void Mode8_Loop(void);
void Mode8_Exit(void);

/* ----------------------- 模式9：调节第三问目标位置 ----------------------- */
void Mode9_Init(void);
void Mode9_Loop(void);
void Mode9_Exit(void);

/* ------------------- 模式10：切换第二问速度任务方案 -------------------- */
void Mode10_Init(void);
void Mode10_Loop(void);
void Mode10_Exit(void);

#endif