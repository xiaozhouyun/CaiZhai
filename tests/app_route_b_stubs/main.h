#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stdbool.h>
#include <stdint.h>

/* app.c 单元测试不引入芯片 HAL，只提供它实际读取的外部量。 */
#define __HWT101_HAL_H
#define __TOF200F_H
extern volatile float g_hwt101_yaw;
extern volatile float TofData;
extern volatile uint32_t TofFrameSeq;

uint32_t HAL_GetTick(void);

#endif
