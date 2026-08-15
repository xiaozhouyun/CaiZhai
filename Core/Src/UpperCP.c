#include "UpperCP.h"
#include "cmsis_os.h"
#include "vofa.h"
#include "app.h"
#include "arms.h"
#include "navigation.h"
#include "pca9685.h"
#include "usart.h"
#include "voice.h"
#include "tof200f.h"
#include "bujin.h"
#include "FreeRTOS.h"
#include "task.h"
#include "action_scheduler.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UPPERCP_RX_BUF_LEN 128U
#define UPPERCP_DMA_RX_BUF_LEN 128U
#define UPPERCP_CMD_MOVE_GBK "\xD2\xC6\xB6\xAF"

static volatile uint8_t uppercp_rx_buf[UPPERCP_RX_BUF_LEN];
static volatile uint16_t uppercp_rx_head;
static volatile uint16_t uppercp_rx_tail;
static volatile uint32_t uppercp_rx_count;
static volatile uint8_t uppercp_last_byte;
static uint8_t uppercp_dma_rx_buf[UPPERCP_DMA_RX_BUF_LEN];
static uint16_t uppercp_dma_rx_pos;
static volatile uint32_t uppercp_rx_overflow_count;
static volatile uint32_t uppercp_uart_error_count;
static char uppercp_cmd_buf[UPPERCP_RX_BUF_LEN];
static char uppercp_last_cmd[UPPERCP_RX_BUF_LEN];
static uint16_t uppercp_cmd_len;

static void Serial5_Printf(const char *fmt, ...)
{
    char tx_buf[96];
    va_list args;
    int len;

    va_start(args, fmt);
    len = vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);
    va_end(args);

    if (len <= 0) {
        return;
    }

    if ((uint32_t)len >= sizeof(tx_buf)) {
        len = (int)sizeof(tx_buf) - 1;
    }

    (void)HAL_UART_Transmit(&huart5, (uint8_t *)tx_buf, (uint16_t)len, 100U);
}

void UpperCP_UartRxByte(uint8_t data)
{
    uint16_t next_head = (uint16_t)((uppercp_rx_head + 1U) % UPPERCP_RX_BUF_LEN);

    uppercp_last_byte = data;
    uppercp_rx_count++;

    if (next_head == uppercp_rx_tail) {
        uppercp_rx_overflow_count++;
        return;
    }

    uppercp_rx_buf[uppercp_rx_head] = data;
    uppercp_rx_head = next_head;
}

void UpperCP_UartDmaStart(void)
{
    uppercp_dma_rx_pos = 0U;

    if (HAL_UART_Receive_DMA(&huart5, uppercp_dma_rx_buf, UPPERCP_DMA_RX_BUF_LEN) != HAL_OK) {
        uppercp_uart_error_count++;
        return;
    }

    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
}

void UpperCP_UartDmaRxProcess(void)
{
    uint16_t dma_rx_pos;

    dma_rx_pos = (uint16_t)(UPPERCP_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(huart5.hdmarx));
    if (dma_rx_pos >= UPPERCP_DMA_RX_BUF_LEN) {
        dma_rx_pos = 0U;
    }

    while (uppercp_dma_rx_pos != dma_rx_pos) {
        UpperCP_UartRxByte(uppercp_dma_rx_buf[uppercp_dma_rx_pos]);
        uppercp_dma_rx_pos++;
        if (uppercp_dma_rx_pos >= UPPERCP_DMA_RX_BUF_LEN) {
            uppercp_dma_rx_pos = 0U;
        }
    }
}

uint32_t UpperCP_GetRxOverflowCount(void)
{
    return uppercp_rx_overflow_count;
}

uint32_t UpperCP_GetUartErrorCount(void)
{
    return uppercp_uart_error_count;
}

static void UpperCP_UartDmaRestart(void)
{
    (void)HAL_UART_DMAStop(&huart5);
    UpperCP_UartDmaStart();
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart5)
    {
        UpperCP_UartDmaRxProcess();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart5)
    {
        UpperCP_UartDmaRxProcess();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart5)
    {
        uppercp_uart_error_count++;
        UpperCP_UartDmaRestart();
    }

    Emm_UartErrorCallback(huart);
}

void UpperCP_SendTask(const char *task)
{
    static const char line_end[] = "\r\n";
    uint8_t valid_task = 0U;

    if (task == NULL) {
        return;
    }

    if ((strcmp(task, "send") == 0) ||
        (strcmp(task, "pour") == 0) ||
        (strcmp(task, "scan") == 0)) {
        valid_task = 1U;
    } else if (strncmp(task, "send:", 5U) == 0) {
        const char *position_text = task + 5;
        char *end;
        long position = strtol(position_text, &end, 10);

        if ((position_text[0] >= '1') && (position_text[0] <= '9') &&
            (*end == '\0') &&
            (position >= 1L) && (position <= 12L)) {
            valid_task = 1U;
        }
    }

    if (valid_task == 0U) {
        return;
    }

    (void)HAL_UART_Transmit(&huart5, (uint8_t *)task, (uint16_t)strlen(task), 100U);
    (void)HAL_UART_Transmit(&huart5, (uint8_t *)line_end, (uint16_t)(sizeof(line_end) - 1U), 100U);
}

const char *UpperCP_GetLastCommand(void)
{
    return uppercp_last_cmd;
}

uint32_t UpperCP_GetRxCount(void)
{
    return uppercp_rx_count;
}

uint8_t UpperCP_GetLastByte(void)
{
    return uppercp_last_byte;
}

static char *ret = NULL;
uint8_t PosFlag = 1;
float angle_dif1 = 0.0f;         /**< 旋转角度微调步进增量全局变量 */
uint8_t upordownFlag = 0;        /**< 上下抓取目标状态标志位 (0：抓地上，1：抓树上) */
uint8_t CameraFlag = 0;


static const uint8_t k_default_fruits[8] = {4, 3, 1, 10, 8, 9, 2, 11};
/* uint8_t fruits[8] = {3,5,7,1,6,10,12,9}; */
uint8_t fruits[8] = { 4, 3, 1, 10, 8, 9, 2, 11};
/* uint8_t fruits[8] = {12,2,9,4,5,11,1,7}; */

uint8_t fruits_count = 0;

void UpperCP_ResetQrResult(void)
{
    memcpy(fruits, k_default_fruits, sizeof(fruits));
    fruits_count = 0U;
    CameraFlag = 0U;
}

void UpperCP_RX(void)
{
    uint8_t command_ready = 0U;

    while (uppercp_rx_tail != uppercp_rx_head) {
        uint8_t ch = uppercp_rx_buf[uppercp_rx_tail];
        uppercp_rx_tail = (uint16_t)((uppercp_rx_tail + 1U) % UPPERCP_RX_BUF_LEN);

        if (ch == ';' || ch == '\r' || ch == '\n') {
            if (uppercp_cmd_len == 0U) {
                continue;
            }
            uppercp_cmd_buf[uppercp_cmd_len] = '\0';
            uppercp_cmd_len = 0U;
            command_ready = 1U;
            break;
        }

        if (uppercp_cmd_len < (UPPERCP_RX_BUF_LEN - 1U)) {
            uppercp_cmd_buf[uppercp_cmd_len++] = (char)ch;
        } else {
            uppercp_cmd_len = 0U;
        }
    }

    if (command_ready == 0U) {
        return;
    }

    // /* 将上位机 UpperCP 接收到的原始命令数据通过 VOFA+ 打印输出 */
    // Vofa_PrintUpperCPData(uppercp_cmd_buf);

    ret = strtok(uppercp_cmd_buf, ":");
    if (ret != NULL) {
        strncpy(uppercp_last_cmd, ret, sizeof(uppercp_last_cmd) - 1U);
        uppercp_last_cmd[sizeof(uppercp_last_cmd) - 1U] = '\0';

        /* 按首字符分发，只调用匹配的函数 */
        switch (ret[0]) {
            case 'a': Arm_func();   break;  /* arm:X */
            case 'c': cmd_func();   break;  /* cmd:X */
            case 'v': voice_func(); break;  /* voice:X */
            case 'Q': ErWeiMa_func(); break; /* QR:X */
            case 'm': Move_func();  break;  /* move:X */
            default:
                if ((uint8_t)ret[0] == 0xD2) { Move_func(); }  /* 移动(GBK) */
                break;
        }
        ret = NULL;
    }
}

void cmd_func(void)
{
    if (strncmp(ret, "cmd", 3) == 0) {
        float temp_num = 0.0f;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%f", &temp_num);
            Serial5_Printf("num = %f\r\n", temp_num);
        }

    }
}

void speed_func(void)
{
    if (strncmp(ret, "speed", 5) == 0) {
        int temp_num = 0;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%d", &temp_num);
        }

        speed.tar = (float)temp_num;
        speed.diff = (speed.tar - speed.real) / 8.0f;

    }
}

void angle_func(void)
{
    if (strncmp(ret, "angle", 5) == 0) {
        int temp_num = 0;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%d", &temp_num);
        }

        angle_speed.tar = (float)temp_num;
        angle_speed.diff = (angle_speed.tar - angle_speed.real) / 8.0f;

    }
}

void face_func(void)
{
    if (strncmp(ret, "face", 4) == 0) {
        float temp_num = 0.0f;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%f", &temp_num);
        }

        TarAngle = (int)temp_num;

    }
}

void voice_func(void)
{
    if (strncmp(ret, "voice", 5) == 0) {
        int temp_num = 0;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%d", &temp_num);
        }

        Voice_Num(temp_num);

    }
}

void Arm_func(void)
{
    if (strncmp(ret, "arm", 3) == 0) {
        int temp_num = 0;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%d", &temp_num);
        }
        /* 串口任务只负责解析；耗时动作统一由 StartTask07 的状态机执行。 */
        if ((temp_num >= 0) && (temp_num <= 6)) {
            Vofa_Printf("[ARM_RX] arm=%d -> ActionScheduler\r\n", temp_num);
            App_NotifyVisionCommandReceived((uint8_t)temp_num);
            ActionScheduler_RequestVisionArm((uint8_t)temp_num);
        } else {
            Vofa_Printf("[ARM_RX] invalid arm=%d\r\n", temp_num);
        }
    }
}

void ErWeiMa_func(void)
{
    if (strncmp(ret, "QR", 2) == 0) {
        uint8_t parsed[8];
        uint8_t seen[13] = {0U};
        char *p_num;
        uint8_t i = 0U;
        uint8_t valid = 1U;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            char *end;
            long value;

            if (i >= 8U) {
                valid = 0U;
                break;
            }

            value = strtol(p_num, &end, 10);
            if ((end == p_num) || (*end != '\0') ||
                (value < 1L) || (value > 12L) || seen[value] != 0U) {
                valid = 0U;
                break;
            }

            parsed[i++] = (uint8_t)value;
            seen[value] = 1U;
        }

        if ((valid != 0U) && (i == 8U)) {
            memcpy(fruits, parsed, sizeof(fruits));
            fruits_count = 8U;
            CameraFlag = 1U;
        }
    }
}

void Move_func(void)
{
    if ((strncmp(ret, UPPERCP_CMD_MOVE_GBK, 4) == 0) || (strncmp(ret, "move", 4) == 0)) {
        float temp_num = 0.0f;
        char *p_num;

        for (p_num = strtok(NULL, ","); p_num != NULL; p_num = strtok(NULL, ",")) {
            sscanf(p_num, "%f", &temp_num);
            Serial5_Printf("move %.2f\r\n", temp_num);
        }

        TarPos = temp_num;
    }
}
