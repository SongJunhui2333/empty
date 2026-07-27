#include "motor.h"

/* ---------------------------------------------------------------- */
/*                              电机编码器变量                             */
/* ---------------------------------------------------------------- */

uint16_t encoder_l_count = 0;
uint16_t encoder_r_count = 0;
uint32_t encoder_l_total = 0; /* 左轮编码器累计计数值（永不清零） */
uint32_t encoder_r_total = 0; /* 右轮编码器累计计数值（永不清零） */

/* ---------------------------- 电机目标速度 ---------------------------- */
uint16_t motor_l_target_speed = 15;
uint16_t motor_r_target_speed = 15;

/* ---------------------------------------------------------------- */
/*                             测量得的电机速度                             */
/* ---------------------------------------------------------------- */
int filt_velocity_r = 0; // 滤波后的速度
int last_filt_velocitya_r = 0;

int filt_velocity_l = 0; // 滤波后的速度
int last_filt_velocitya_l = 0;

/* ------------------------ 创建左右电机PID控制器实例 ------------------------ */
pid_t pid_motor_l;
pid_t pid_motor_r;

/* ---------------------------------------------------------------- */
/*                              电机相关函数                              */
/* ---------------------------------------------------------------- */

/**
 * @brief Initializes the motor driver and sets the initial state of the motor.
 *
 * This function configures the GPIO pins and timer for the specified motor.
 * It enables the motor driver and sets the initial PWM duty cycle to 0.
 *
 * @param motor_id The ID of the motor to initialize (1 or 2).
 */
void motor_init(uint8_t motor_id)
{

    DL_Timer_startCounter(PWM_MOTOR_INST); // 启动PWM计数器

    if (motor_id == 1)
    {
        DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_MOTOR_INST, 0, GPIO_PWM_MOTOR_C0_IDX);
    }
    else if (motor_id == 2)
    {
        DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_MOTOR_INST, 0, GPIO_PWM_MOTOR_C1_IDX);
    }
}

void motor_set_duty(uint8_t motor_id, uint16_t duty)
{

    if (duty > 4000)
    {
        duty = 4000; // 限制占空比最大值为4000
    }
    if (duty < 0)
    {
        duty = 0; // 限制占空比最小值为0
    }

    if (motor_id == 1)
    {
        DL_Timer_setCaptureCompareValue(PWM_MOTOR_INST, duty, GPIO_PWM_MOTOR_C0_IDX);
    }
    else if (motor_id == 2)
    {
        DL_Timer_setCaptureCompareValue(PWM_MOTOR_INST, duty, GPIO_PWM_MOTOR_C1_IDX);
    }
}

/**
 * @brief Sets the direction of the specified motor.
 *
 * This function controls the direction of the motor by setting the appropriate GPIO pins.
 *
 * @param motor_id The ID of the motor to control (1:左轮电机 or 2:右轮电机).
 * @param direction The desired direction (0: stop, 1: forward, 2: reverse).
 */
void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if (motor_id == 1)
    {
        if (direction == 0)
        {
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if (direction == 1)
        {
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if (direction == 2)
        {
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    }
    else if (motor_id == 2)
    {
        if (direction == 0)
        {
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if (direction == 1)
        {
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if (direction == 2)
        {
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
    }
}