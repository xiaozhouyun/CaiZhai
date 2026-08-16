#ifndef __TOF200F_H
#define __TOF200F_H

#include "main.h"

/** TOF200F 激光测距传感器原始测量距离值 (毫米 mm，除以 10.0 转换为厘米 cm) */
extern volatile float TofData;
/** 每收到一帧 CRC 正确且距离有效的 TOF 数据后递增，供任务判断 TofData 是否为新测量值 */
extern volatile uint32_t TofFrameSeq;

void TOF200F_Init(void);
void TOF200F_UartRxByte(uint8_t data);
float TOF200F_GetDistanceCm(void);
void get_dis(void);

#endif
