#include "UpperCP.h"
#include "main.h"
#include "navigation.h"

#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart5;
struct move speed;
struct move angle_speed;
int TarAngle;
float TarPos;

static char uart_output[256];
static size_t uart_output_length;

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                    uint8_t *data,
                                    uint16_t length,
                                    uint32_t timeout)
{
    (void)huart;
    (void)timeout;
    if ((uart_output_length + length) > sizeof(uart_output)) {
        return 1;
    }
    memcpy(&uart_output[uart_output_length], data, length);
    uart_output_length += length;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart,
                                      uint8_t *data,
                                      uint16_t length)
{
    (void)huart;
    (void)data;
    (void)length;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart)
{
    (void)huart;
    return HAL_OK;
}

void Emm_UartErrorCallback(UART_HandleTypeDef *huart) { (void)huart; }
void Voice_Num(int number) { (void)number; }
void Vofa_Printf(const char *format, ...) { (void)format; }
void Vofa_PrintUpperCPData(const char *cmd_buf) { (void)cmd_buf; }
void ActionScheduler_RequestVisionArm(uint8_t command) { (void)command; }

static int expect_output(const char *expected)
{
    const size_t expected_length = strlen(expected);
    if ((uart_output_length != expected_length) ||
        (memcmp(uart_output, expected, expected_length) != 0)) {
        fprintf(stderr, "unexpected UART output length=%zu expected=%zu\n",
                uart_output_length, expected_length);
        return 0;
    }
    return 1;
}

int main(void)
{
    const char *invalid_tasks[] = {
        "send:", "send:0", "send:01", "send:+1", "send: 1",
        "send:13", "send:abc", "send:1xxx", "other"
    };
    size_t accepted_length;
    size_t i;

    UpperCP_SendTask("send");
    UpperCP_SendTask("send:1");
    UpperCP_SendTask("send:12");
    UpperCP_SendTask("scan");
    UpperCP_SendTask("pour");

    if (!expect_output("send\r\nsend:1\r\nsend:12\r\nscan\r\npour\r\n")) {
        return 1;
    }

    accepted_length = uart_output_length;
    UpperCP_SendTask(NULL);
    for (i = 0U; i < (sizeof(invalid_tasks) / sizeof(invalid_tasks[0])); ++i) {
        UpperCP_SendTask(invalid_tasks[i]);
    }
    if (uart_output_length != accepted_length) {
        fprintf(stderr, "invalid task produced UART output\n");
        return 1;
    }

    puts("uppercp_send_task_test: PASS");
    return 0;
}
