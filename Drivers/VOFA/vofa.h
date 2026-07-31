#ifndef VOFA_H
#define VOFA_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ========================================================================== */
/*                          VOFA+ 串口可视化驱动                                  */
/*                                                                             */
/*  协议说明:                                                                    */
/*   - 发送 (MCU → VOFA+): JustFloat 二进制协议                                   */
/*     每通道 4 字节 float (小端序), 帧尾 0x00 0x00 0x80 0x7F                     */
/*   - 接收 (VOFA+ → MCU): FireWater ASCII 文本协议                               */
/*     命令格式: CMD=VALUE\r\n                                                   */
/* ========================================================================== */

/* ---- JustFloat 协议宏 ---- */
#define VOFA_MAX_CHANNELS     10   /* 最大通道数 */
#define VOFA_FRAME_TAIL_BYTES 4    /* 帧尾字节数: 0x00 0x00 0x80 0x7F */

/* ---- FireWater 命令接收 ---- */
#define VOFA_CMD_BUF_SIZE     64   /* 命令接收缓冲区大小 */

/* ---- 命令类型 ---- */
typedef enum {
    VOFA_CMD_NONE = 0,      /* 无命令 */
    VOFA_CMD_SET_KP,        /* 设置 Kp: KP=1.5 */
    VOFA_CMD_SET_KI,        /* 设置 Ki: KI=0.01 */
    VOFA_CMD_SET_KD,        /* 设置 Kd: KD=10.0 */
    VOFA_CMD_SET_SETPOINT,  /* 设置目标值: SET=100 */
    VOFA_CMD_SET_SPEED,     /* 设置电机基础速度: SPD=60 */
    VOFA_CMD_RESET,         /* 复位PID: RESET */
    VOFA_CMD_PRINT_PARAMS,  /* 打印当前参数: PRINT */

    /* ---- 控球 PID 专用命令 ---- */
    VOFA_CMD_SET_BALL_KP,       /* 设置球控 Kp: BKP=1.5 */
    VOFA_CMD_SET_BALL_KI,       /* 设置球控 Ki: BKI=0.01 */
    VOFA_CMD_SET_BALL_KD,       /* 设置球控 Kd: BKD=10.0 */
    VOFA_CMD_SET_BALL_FF,       /* 设置前馈增益: BFF=0.2 */
    VOFA_CMD_RESET_BALL,        /* 复位球控PID: BRESET */
    VOFA_CMD_PRINT_BALL_PARAMS, /* 打印球控参数: BPRINT */
} vofa_cmd_type_t;

/* ---- TX 忙标志 (防止 ISR 与主循环并发发送导致数据错乱) ---- */
extern volatile uint8_t vofa_tx_busy;

/* ---- 命令结构体 ---- */
typedef struct {
    vofa_cmd_type_t type;   /* 命令类型 */
    float           value;  /* 命令值 (对 RESET/PRINT/BRESET/BPRINT 无效) */
} vofa_cmd_t;

/* ========================================================================== */
/*                             API 函数声明                                     */
/* ========================================================================== */

/**
 * @brief  初始化 VOFA 驱动
 * @note   必须在 SYSCFG_DL_init() 之后调用
 *         已在 main.c 中通过 NVIC_EnableIRQ(DEBUG_INST_INT_IRQN) 使能中断
 */
void vofa_init(void);

/**
 * @brief  通过 JustFloat 协议发送一帧 float 数据到 VOFA+
 * @param  data         float 数据数组指针
 * @param  num_channels 通道数 (1 ~ VOFA_MAX_CHANNELS)
 * @note   帧格式: [float0][float1]...[floatN-1][0x00 0x00 0x80 0x7F]
 *         建议调用频率: 10~200 Hz
 */
void vofa_send_frame(const float *data, uint8_t num_channels);

/**
 * @brief  检查是否有来自 VOFA+ 的命令可用
 * @return 1 = 有命令待处理, 0 = 无命令
 */
uint8_t vofa_cmd_available(void);

/**
 * @brief  获取来自 VOFA+ 的最新命令 (取出后自动清除)
 * @return vofa_cmd_t 命令结构体
 */
vofa_cmd_t vofa_get_cmd(void);

/**
 * @brief  VOFA 接收字节处理函数 (由 DEBUG_INST_IRQHandler 调用)
 * @param  byte 接收到的字节
 * @note   此函数在中断上下文中执行，仅做缓冲和解析
 */
void vofa_rx_isr(uint8_t byte);

#endif /* VOFA_H */
