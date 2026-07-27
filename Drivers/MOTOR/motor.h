#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"
#include "ti_msp_dl_config.h"

/* --------------------------- 电机闭环PID参数 -------------------------- */
#define MOTOR_KP (0.35f)
#define MOTOR_KI (0.07f)
#define MOTOR_KD (0.0f)

//--------------------------- 电机编码器变量 -------------------------- */
extern uint16_t encoder_l_count;
extern uint16_t encoder_r_count;
extern uint32_t encoder_l_total;
extern uint32_t encoder_r_total;

//---------------------------- 电机目标速度 ---------------------------- */
extern uint16_t motor_l_target_speed;
extern uint16_t motor_r_target_speed;

//---------------------------- 测量得的电机速度 ---------------------------- */
extern int filt_velocity_r; // 滤波后的速度
extern int last_filt_velocitya_r;
extern int filt_velocity_l; // 滤波后的速度
extern int last_filt_velocitya_l;

//------------------------ 创建左右电机PID控制器实例 ------------------------ */
extern pid_t pid_motor_l;
extern pid_t pid_motor_r;

//---------------------------- 电机相关函数 ---------------------------- */

void motor_init(uint8_t motor_id);
void motor_set_duty(uint8_t motor_id, uint16_t duty);
void motor_set_direction(uint8_t motor_id, uint8_t direction);

#endif // MOTOR_H