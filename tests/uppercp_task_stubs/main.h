#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stdint.h>

typedef int HAL_StatusTypeDef;
typedef struct {
    void *hdmarx;
} UART_HandleTypeDef;

#define HAL_OK 0
#define UART_IT_IDLE 0
#define __HAL_DMA_GET_COUNTER(handle) (0U)
#define __HAL_UART_ENABLE_IT(huart, interrupt) ((void)(huart), (void)(interrupt))

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                    uint8_t *data,
                                    uint16_t length,
                                    uint32_t timeout);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart,
                                      uint8_t *data,
                                      uint16_t length);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart);

#endif
