#include "vofa.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define VOFA_MAX_FLOATS 12U
#define VOFA_TX_BUF_SIZE 256U

/**
 * @brief JustFloat 协议发送浮点数组 (4字节小端 float + 4字节帧尾 0x00 0x00 0x80 0x7F)
 */
void Vofa_SendFloat(const float *values, uint8_t count)
{
    static const uint8_t frame_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
    uint8_t buffer[VOFA_MAX_FLOATS * sizeof(float) + sizeof(frame_tail)];
    uint16_t length;

    if (values == NULL || count == 0U || count > VOFA_MAX_FLOATS) {
        return;
    }

    length = (uint16_t)count * sizeof(float);
    memcpy(buffer, values, length);
    memcpy(&buffer[length], frame_tail, sizeof(frame_tail));
    (void)HAL_UART_Transmit(&huart6, buffer, length + (uint16_t)sizeof(frame_tail), 100U);
}

/**
 * @brief FireWater 协议发送浮点数组 (ASCII文本格式: "val1,val2,val3...\n")
 */
void Vofa_SendFirewater(const float *values, uint8_t count)
{
    char tx_buf[VOFA_TX_BUF_SIZE];
    uint16_t len = 0U;

    if (values == NULL || count == 0U) {
        return;
    }

    for (uint8_t i = 0U; i < count; i++) {
        if (i > 0U) {
            len += (uint16_t)snprintf(&tx_buf[len], sizeof(tx_buf) - len, ",");
        }
        len += (uint16_t)snprintf(&tx_buf[len], sizeof(tx_buf) - len, "%.3f", values[i]);
    }
    len += (uint16_t)snprintf(&tx_buf[len], sizeof(tx_buf) - len, "\n");

    if (len > 0U && len < sizeof(tx_buf)) {
        (void)HAL_UART_Transmit(&huart6, (const uint8_t *)tx_buf, len, 100U);
    }
}


/**
 * @brief 发送纯文本字符串到 VOFA
 */
void Vofa_SendString(const char *str)
{
    if (str == NULL) {
        return;
    }
    uint16_t len = (uint16_t)strlen(str);
    if (len > 0U) {
        (void)HAL_UART_Transmit(&huart6, (const uint8_t *)str, len, 100U);
    }
}

/**
 * @brief 格式化打印字符串到 VOFA
 */
void Vofa_Printf(const char *format, ...)
{
    char tx_buf[VOFA_TX_BUF_SIZE];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(tx_buf, sizeof(tx_buf), format, args);
    va_end(args);

    if (len > 0) {
        (void)HAL_UART_Transmit(&huart6, (const uint8_t *)tx_buf, (uint16_t)len, 100U);
    }
}

/**
 * @brief 打印 UpperCP 上位机接收到的命令/数据到 VOFA
 */
void Vofa_PrintUpperCPData(const char *cmd_buf)
{
    if (cmd_buf != NULL && cmd_buf[0] != '\0') {
        Vofa_Printf("[UpperCP_Rx]: %s\n", cmd_buf);
    }
}

