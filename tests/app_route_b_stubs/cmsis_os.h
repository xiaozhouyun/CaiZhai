#ifndef TEST_CMSIS_OS_H
#define TEST_CMSIS_OS_H

#include <stdint.h>

#define pdMS_TO_TICKS(ms) (ms)
void vTaskDelay(uint32_t ticks);

#endif
