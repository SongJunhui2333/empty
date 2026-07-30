#ifndef MODE_H
#define MODE_H

#include "clock.h"
#include "key.h"
#include "motor.h"
#include "oled_hardware_i2c.h"
#include "stdio.h"
#include "ti_msp_dl_config.h"

extern uint8_t CurrentMode; // 当前模式编号，0表示未选择模式
extern uint8_t NextMode;    // 下一个模式编号，0表示未选择模式

/* -------------------------- 模式1：初始选择界面 -------------------------- */
void Mode1_Init(void);
void Mode1_Loop(void);
void Mode1_Exit(void);

/* --------------------------- 模式2：第二问代码 -------------------------- */
#define QUESTION2_MOTOR_BASE_SPEED 70          // 模式2的电机基础速度
extern const short question2_trace_weights[8]; // 模式2的循迹权重数组
extern pid_t question2_pid_heading;            // 记录模式2开始的时间
#define QUESTION2_HEADING_SETPOINT (0.0f)      // 模式2的PID控制器目标航向值

void Mode2_Init(void);
void Mode2_Loop(void);
void Mode2_Exit(void);
extern uint8_t question2_flag; // 问题2的标志位，1表示已开始，0表示未开始

/* --------------------------- 模式3：第三问代码 -------------------------- */
void Mode3_Init(void);
void Mode3_Loop(void);
void Mode3_Exit(void);

/* --------------------------- 模式4：第四问代码 -------------------------- */
void Mode4_Init(void);
void Mode4_Loop(void);
void Mode4_Exit(void);

/* --------------------------- 模式5：第五问代码 -------------------------- */
void Mode5_Init(void);
void Mode5_Loop(void);
void Mode5_Exit(void);

// 模式6：第六问代码
void Mode6_Init(void);
void Mode6_Loop(void);
void Mode6_Exit(void);

#endif