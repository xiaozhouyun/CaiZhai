#ifndef __TOF200F_H
#define __TOF200F_H

#include "main.h"

/** TOF200F 激光测距传感器原始测量距离值 (毫米 mm，除以 10.0 转换为厘米 cm) */
extern volatile float TofData;

void TOF200F_Init(void);
void TOF200F_UartRxByte(uint8_t data);
float TOF200F_GetDistanceCm(void);
void get_dis(void);

#endif
