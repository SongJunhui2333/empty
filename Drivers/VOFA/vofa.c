#include "vofa.h"
#include "uart.h"
#include <stdlib.h>
#include <string.h>

/* ========================================================================== */
/*                           内部变量                                          */
/* ========================================================================== */

/* ---- FireWater 命令接收缓冲区 ---- */
static char    vofa_rx_buf[VOFA_CMD_BUF_SIZE]; /* 接收缓冲区 */
static uint8_t vofa_rx_idx = 0;               /* 缓冲区写入索引 */

/* ---- 命令队列 ---- */
static volatile uint8_t vofa_cmd_ready = 0;   /* 命令就绪标志 */
static vofa_cmd_t       vofa_cmd_curr;        /* 当前待处理命令 */

/* ========================================================================== */
/*                           初始化                                            */
/* ========================================================================== */

void vofa_init(void)
{
    vofa_rx_idx    = 0;
    vofa_cmd_ready = 0;
    memset((void *)vofa_rx_buf, 0, sizeof(vofa_rx_buf));
    memset((void *)&vofa_cmd_curr, 0, sizeof(vofa_cmd_curr));
}

/* ========================================================================== */
/*                     JustFloat 协议发送 (MCU → VOFA+)                         */
/* ========================================================================== */

/**
 * @brief  通过 JustFloat 协议发送一帧 float 数据
 * @note   帧格式: [float0(4B)][float1(4B)]...[0x00 0x00 0x80 0x7F]
 *         帧尾 0x00 0x00 0x80 0x7F = float +Inf (IEEE754 小端序)
 */
void vofa_send_frame(const float *data, uint8_t num_channels)
{
    if (num_channels > VOFA_MAX_CHANNELS)
        num_channels = VOFA_MAX_CHANNELS;

    /* 本地缓冲区: 最大通道数 * 4 + 帧尾4字节 */
    uint8_t  buf[VOFA_MAX_CHANNELS * 4 + VOFA_FRAME_TAIL_BYTES];
    uint16_t idx = 0;

    /* ---- 打包 float 通道数据 (小端序) ---- */
    for (uint8_t i = 0; i < num_channels; i++)
    {
        /* 通过 union 或指针将 float 转为 4 字节 */
        uint32_t raw;
        memcpy(&raw, &data[i], sizeof(float));

        buf[idx++] = (uint8_t)(raw & 0xFF);         /* byte[0] LSB */
        buf[idx++] = (uint8_t)((raw >> 8) & 0xFF);  /* byte[1] */
        buf[idx++] = (uint8_t)((raw >> 16) & 0xFF); /* byte[2] */
        buf[idx++] = (uint8_t)((raw >> 24) & 0xFF); /* byte[3] MSB */
    }

    /* ---- 帧尾: float +Inf (0x7F800000 小端序 → 00 00 80 7F) ---- */
    buf[idx++] = 0x00;
    buf[idx++] = 0x00;
    buf[idx++] = 0x80;
    buf[idx++] = 0x7F;

    /* 通过已有的 UART_send_data 发送 */
    UART_send_data(DEBUG_INST, buf, idx);
}

/* ========================================================================== */
/*                  FireWater 命令解析 (VOFA+ → MCU)                            */
/* ========================================================================== */

/**
 * @brief  解析一行命令文本
 * @param  line 以 '\0' 结尾的命令行 (不含 \r \n)
 * @note   支持格式:
 *         - KP=1.5     → 设置 Kp
 *         - KI=0.01    → 设置 Ki
 *         - KD=10.0    → 设置 Kd
 *         - SET=100    → 设置目标值
 *         - RESET      → 复位 PID
 *         - PRINT      → 打印当前 PID 参数
 */
static void vofa_parse_command(const char *line)
{
    if (line[0] == '\0')
        return;

    /* ---- RESET 命令 ---- */
    if (strncmp(line, "RESET", 5) == 0)
    {
        vofa_cmd_curr.type  = VOFA_CMD_RESET;
        vofa_cmd_curr.value = 0.0f;
        vofa_cmd_ready      = 1;
        return;
    }

    /* ---- PRINT 命令 ---- */
    if (strncmp(line, "PRINT", 5) == 0)
    {
        vofa_cmd_curr.type  = VOFA_CMD_PRINT_PARAMS;
        vofa_cmd_curr.value = 0.0f;
        vofa_cmd_ready      = 1;
        return;
    }

    /* ---- KP=xxx 命令 ---- */
    if (strncmp(line, "KP=", 3) == 0)
    {
        vofa_cmd_curr.type  = VOFA_CMD_SET_KP;
        vofa_cmd_curr.value = (float)atof(line + 3);
        vofa_cmd_ready      = 1;
        return;
    }

    /* ---- KI=xxx 命令 ---- */
    if (strncmp(line, "KI=", 3) == 0)
    {
        vofa_cmd_curr.type  = VOFA_CMD_SET_KI;
        vofa_cmd_curr.value = (float)atof(line + 3);
        vofa_cmd_ready      = 1;
        return;
    }

    /* ---- KD=xxx 命令 ---- */
    if (strncmp(line, "KD=", 3) == 0)
    {
        vofa_cmd_curr.type  = VOFA_CMD_SET_KD;
        vofa_cmd_curr.value = (float)atof(line + 3);
        vofa_cmd_ready      = 1;
        return;
    }

    /* ---- SET=xxx 命令 ---- */
    if (strncmp(line, "SET=", 4) == 0)
    {
        vofa_cmd_curr.type  = VOFA_CMD_SET_SETPOINT;
        vofa_cmd_curr.value = (float)atof(line + 4);
        vofa_cmd_ready      = 1;
        return;
    }

    /* 无法识别的命令，静默忽略 */
}

/* ========================================================================== */
/*                     RX 中断服务 (由 uart.c 调用)                              */
/* ========================================================================== */

void vofa_rx_isr(uint8_t byte)
{
    /* 忽略控制字符以外的不可见字符 (保留退格以便调试) */
    if (byte < 0x20 && byte != '\r' && byte != '\n' && byte != '\b')
        return;

    /* ---- 收到换行符: 解析完整命令 ---- */
    if (byte == '\n')
    {
        if (vofa_rx_idx > 0)
        {
            vofa_rx_buf[vofa_rx_idx] = '\0';
            vofa_parse_command(vofa_rx_buf);
            vofa_rx_idx = 0;
        }
        return;
    }

    /* ---- 收到回车符: 忽略 (等待 \n) ---- */
    if (byte == '\r')
        return;

    /* ---- 退格处理 ---- */
    if (byte == '\b')
    {
        if (vofa_rx_idx > 0)
            vofa_rx_idx--;
        return;
    }

    /* ---- 普通字符: 存入缓冲区 ---- */
    if (vofa_rx_idx < VOFA_CMD_BUF_SIZE - 1)
    {
        vofa_rx_buf[vofa_rx_idx++] = byte;
    }
    else
    {
        /* 缓冲区溢出: 重置，丢弃当前帧 */
        vofa_rx_idx = 0;
    }
}

/* ========================================================================== */
/*                         命令查询接口                                        */
/* ========================================================================== */

uint8_t vofa_cmd_available(void)
{
    return vofa_cmd_ready;
}

vofa_cmd_t vofa_get_cmd(void)
{
    vofa_cmd_t cmd = vofa_cmd_curr; /* 拷贝返回值 */
    vofa_cmd_ready = 0;              /* 清除标志 */
    memset((void *)&vofa_cmd_curr, 0, sizeof(vofa_cmd_curr));
    return cmd;
}
