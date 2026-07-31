#include "interrupt.h"

// #include "mpu6050.h"
// #include "bno08x_uart_rvc.h"
// #include "wit.h"
// #include "vl53l0x.h"
// #include "lsm6dsv16x.h"
// #include "imu660rb.h"

uint8_t enable_group1_irq = 0;

void Interrupt_Init(void)
{
    if (enable_group1_irq)
    {
        NVIC_EnableIRQ(1);
    }
}

void SysTick_Handler(void)
{
    tick_ms++;
}

#if defined UART_BNO08X_INST_IRQHandler
void UART_BNO08X_INST_IRQHandler(void)
{
    uint8_t checkSum = 0;
    extern uint8_t bno08x_dmaBuffer[19];

    DL_DMA_disableChannel(DMA, DMA_BNO08X_CHAN_ID);
    uint8_t rxSize = 18 - DL_DMA_getTransferSize(DMA, DMA_BNO08X_CHAN_ID);

    if (DL_UART_isRXFIFOEmpty(UART_BNO08X_INST) == false)
        bno08x_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_BNO08X_INST);

    for (int i = 2; i <= 14; i++)
        checkSum += bno08x_dmaBuffer[i];

    if ((rxSize == 19) && (bno08x_dmaBuffer[0] == 0xAA) && (bno08x_dmaBuffer[1] == 0xAA) &&
        (checkSum == bno08x_dmaBuffer[18]))
    {
        bno08x_data.index = bno08x_dmaBuffer[2];
        bno08x_data.yaw = (int16_t)((bno08x_dmaBuffer[4] << 8) | bno08x_dmaBuffer[3]) / 100.0;
        bno08x_data.pitch = (int16_t)((bno08x_dmaBuffer[6] << 8) | bno08x_dmaBuffer[5]) / 100.0;
        bno08x_data.roll = (int16_t)((bno08x_dmaBuffer[8] << 8) | bno08x_dmaBuffer[7]) / 100.0;
        bno08x_data.ax = (bno08x_dmaBuffer[10] << 8) | bno08x_dmaBuffer[9];
        bno08x_data.ay = (bno08x_dmaBuffer[12] << 8) | bno08x_dmaBuffer[11];
        bno08x_data.az = (bno08x_dmaBuffer[14] << 8) | bno08x_dmaBuffer[13];
    }

    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_BNO08X_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_BNO08X_CHAN_ID, (uint32_t)&bno08x_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_BNO08X_CHAN_ID, 18);
    DL_DMA_enableChannel(DMA, DMA_BNO08X_CHAN_ID);
}
#endif

#if defined UART_WIT_INST_IRQHandler
void UART_WIT_INST_IRQHandler(void)
{
    uint8_t checkSum, packCnt = 0;
    extern uint8_t wit_dmaBuffer[33];

    DL_DMA_disableChannel(DMA, DMA_WIT_CHAN_ID);
    uint8_t rxSize = 32 - DL_DMA_getTransferSize(DMA, DMA_WIT_CHAN_ID);

    if (DL_UART_isRXFIFOEmpty(UART_WIT_INST) == false)
        wit_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_WIT_INST);

    while (rxSize >= 11)
    {
        checkSum = 0;
        for (int i = packCnt * 11; i < (packCnt + 1) * 11 - 1; i++)
            checkSum += wit_dmaBuffer[i];

        if ((wit_dmaBuffer[packCnt * 11] == 0x55) && (checkSum == wit_dmaBuffer[packCnt * 11 + 10]))
        {
            if (wit_dmaBuffer[packCnt * 11 + 1] == 0x51)
            {
                wit_data.ax =
                    (int16_t)((wit_dmaBuffer[packCnt * 11 + 3] << 8) | wit_dmaBuffer[packCnt * 11 + 2]) / 2.048; // mg
                wit_data.ay =
                    (int16_t)((wit_dmaBuffer[packCnt * 11 + 5] << 8) | wit_dmaBuffer[packCnt * 11 + 4]) / 2.048; // mg
                wit_data.az =
                    (int16_t)((wit_dmaBuffer[packCnt * 11 + 7] << 8) | wit_dmaBuffer[packCnt * 11 + 6]) / 2.048; // mg
                wit_data.temperature =
                    (int16_t)((wit_dmaBuffer[packCnt * 11 + 9] << 8) | wit_dmaBuffer[packCnt * 11 + 8]) / 100.0; // °C
            }
            else if (wit_dmaBuffer[packCnt * 11 + 1] == 0x52)
            {
                wit_data.gx =
                    (int16_t)((wit_dmaBuffer[packCnt * 11 + 3] << 8) | wit_dmaBuffer[packCnt * 11 + 2]) / 16.384; // °/S
                wit_data.gy =
                    (int16_t)((wit_dmaBuffer[packCnt * 11 + 5] << 8) | wit_dmaBuffer[packCnt * 11 + 4]) / 16.384; // °/S
                wit_data.gz =
                    (int16_t)((wit_dmaBuffer[packCnt * 11 + 7] << 8) | wit_dmaBuffer[packCnt * 11 + 6]) / 16.384; // °/S
            }
            else if (wit_dmaBuffer[packCnt * 11 + 1] == 0x53)
            {
                wit_data.roll = (int16_t)((wit_dmaBuffer[packCnt * 11 + 3] << 8) | wit_dmaBuffer[packCnt * 11 + 2]) /
                                32768.0 * 180.0; // °
                wit_data.pitch = (int16_t)((wit_dmaBuffer[packCnt * 11 + 5] << 8) | wit_dmaBuffer[packCnt * 11 + 4]) /
                                 32768.0 * 180.0; // °
                wit_data.yaw = (int16_t)((wit_dmaBuffer[packCnt * 11 + 7] << 8) | wit_dmaBuffer[packCnt * 11 + 6]) /
                               32768.0 * 180.0; // °
                wit_data.version = (int16_t)((wit_dmaBuffer[packCnt * 11 + 9] << 8) | wit_dmaBuffer[packCnt * 11 + 8]);
            }
        }

        rxSize -= 11;
        packCnt++;
    }

    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_WIT_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t)&wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, 32);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);
}
#endif

void GROUP1_IRQHandler(void)
{
    //     switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
    //     {
    // #if defined GPIO_MULTIPLE_GPIOA_INT_IIDX
    //     case GPIO_MULTIPLE_GPIOA_INT_IIDX:
    //         switch (DL_GPIO_getPendingInterrupt(GPIOA))
    //         {
    // #if (defined GPIO_MPU6050_PORT) && (GPIO_MPU6050_PORT == GPIOA)
    //         case GPIO_MPU6050_PIN_MPU6050_INT_IIDX:
    //             Read_Quad();
    //             break;
    // #endif

    // #if (defined GPIO_LSM6DSV16X_PORT) && (GPIO_LSM6DSV16X_PORT == GPIOA)
    //         case GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_IIDX:
    //             Read_LSM6DSV16X();
    //             break;
    // #endif

    // #if (defined GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT) && (GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT == GPIOA)
    //         case GPIO_VL53L0X_PIN_VL53L0X_GPIO1_IIDX:
    //             Read_VL53L0X();
    //             break;
    // #endif

    // #if (defined GPIO_IMU660RB_PIN_IMU660RB_INT1_PORT) && (GPIO_IMU660RB_PIN_IMU660RB_INT1_PORT == GPIOA)
    //         case GPIO_IMU660RB_PIN_IMU660RB_INT1_IIDX:
    //             Read_IMU660RB();
    //             break;
    // #endif

    //         default:
    //             break;
    //         }
    // #endif

    // #if defined GPIO_MULTIPLE_GPIOB_INT_IIDX
    //     case GPIO_MULTIPLE_GPIOB_INT_IIDX:
    //         switch (DL_GPIO_getPendingInterrupt(GPIOB))
    //         {
    // #if (defined GPIO_MPU6050_PORT) && (GPIO_MPU6050_PORT == GPIOB)
    //         case GPIO_MPU6050_PIN_MPU6050_INT_IIDX:
    //             Read_Quad();
    //             break;
    // #endif

    // #if (defined GPIO_LSM6DSV16X_PORT) && (GPIO_LSM6DSV16X_PORT == GPIOB)
    //         case GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_IIDX:
    //             Read_LSM6DSV16X();
    //             break;
    // #endif

    // #if (defined GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT) && (GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT == GPIOB)
    //         case GPIO_VL53L0X_PIN_VL53L0X_GPIO1_IIDX:
    //             Read_VL53L0X();
    //             break;
    // #endif

    // #if (defined GPIO_IMU660RB_PIN_IMU660RB_INT1_PORT) && (GPIO_IMU660RB_PIN_IMU660RB_INT1_PORT == GPIOB)
    //         case GPIO_IMU660RB_PIN_IMU660RB_INT1_IIDX:
    //             Read_IMU660RB();
    //             break;
    // #endif

    //         default:
    //             break;
    //         }
    // #endif

    // #if defined GPIO_MPU6050_INT_IIDX
    //     case GPIO_MPU6050_INT_IIDX:
    //         Read_Quad();
    //         break;
    // #endif

    // #if defined GPIO_LSM6DSV16X_INT_IIDX
    //     case GPIO_LSM6DSV16X_INT_IIDX:
    //         Read_LSM6DSV16X();
    //         break;
    // #endif

    // #if defined GPIO_VL53L0X_INT_IIDX
    //     case GPIO_VL53L0X_INT_IIDX:
    //         Read_VL53L0X();
    //         break;
    // #endif

    // #if defined GPIO_IMU660RB_INT_IIDX
    //     case GPIO_IMU660RB_INT_IIDX:
    //         Read_IMU660RB();
    //         break;
    // #endif
    //     }

    switch (DL_GPIO_getPendingInterrupt(GPIOA))
    {
    // 编码器记录
    case DC_MOTOR_ENCODER1_A_IIDX: // 编码器2（电机2=左轮）
        encoder_r_count++;
        encoder_r_total++;
        break;
    case DC_MOTOR_ENCODER1_B_IIDX: // 编码器2（电机2=左轮）
        encoder_r_count++;
        encoder_r_total++;
        break;

    case DC_MOTOR_ENCODER2_A_IIDX: // 编码器1（电机1=右轮）
        encoder_l_count++;
        encoder_l_total++;
        break;
    case DC_MOTOR_ENCODER2_B_IIDX: // 编码器1（电机1=右轮）
        encoder_l_count++;
        encoder_l_total++;
        break;

    default:
        break;
    }

    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {

    default:
        break;
    }
}

void MOTOR_CONTROL_INST_IRQHandler(void)
{

    switch (DL_Timer_getPendingInterrupt(MOTOR_CONTROL_INST))
    {

    case DL_TIMER_IIDX_LOAD: {

        float a = 0.3; // 滤波系数，取值范围为0~1，越接近1，滤波效果越明显
        filt_velocity_l =
            a * encoder_l_count +
            (1 - a) * last_filt_velocitya_l;     // 简单算法滤波，此次速度取30%的权重，过往速度取70%的权重，让速度更平滑
        last_filt_velocitya_l = filt_velocity_l; // 此次速度记录为“上次速度”

        filt_velocity_r =
            a * encoder_r_count +
            (1 - a) * last_filt_velocitya_r;     // 简单算法滤波，此次速度取30%的权重，过往速度取70%的权重，让速度更平滑
        last_filt_velocitya_r = filt_velocity_r; // 此次速度记录为“上次速度”

        encoder_l_count = 0;
        encoder_r_count = 0;

        // sprintf((char *)uart_tx_buff, "L: %d, R: %d\r\n", filt_velocity_l, filt_velocity_r);
        // UART_print_string(DEBUG_INST, (char *)uart_tx_buff);
        // memset((void *)uart_tx_buff, 0, 128);

        /* 使用编码器计数值作为速度反馈进行PID计算 */
        float ctrl_l = pid_calculate(&pid_motor_l, (float)filt_velocity_l);
        float ctrl_r = pid_calculate(&pid_motor_r, (float)filt_velocity_r);
        // if (ctrl_l > 20)
        //     ctrl_l = 20;
        // if (ctrl_r > 20)
        //     ctrl_r = 20;

        /* 将PID输出转换为电机占空比并施加到电机 */
        motor_set_duty(2, (uint16_t)(100 * ctrl_r));
        motor_set_duty(1, (uint16_t)(100 * ctrl_l));

        break;
    }

    default:
        break;
    }
}

void TIMER_BASE_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(TIMER_BASE_INST))
    {
    case DL_TIMER_IIDX_LOAD: {
        // 处理定时器中断
        gw_gray_serial_tick(); // 处理灰度传感器串口数据
        gray_trace_tick();
        Key_Tick();
        break;
    }

    default:
        break;
    }
}