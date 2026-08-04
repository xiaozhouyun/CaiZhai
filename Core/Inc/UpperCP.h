#ifndef UPPERCP_H
#define UPPERCP_H

#include <stdint.h>

extern uint8_t PosFlag;
extern uint8_t fruits[8];
extern uint8_t fruits_count;

extern uint8_t CameraFlag;

extern float angle_dif1;         /**< 旋转角度微调步进增量全局变量 */
extern uint8_t upordownFlag;     /**< 上下抓取目标状态标志位 (0：抓地上，1：抓树上) */

void UpperCP_RX(void);
void UpperCP_UartRxByte(uint8_t data);
void UpperCP_UartDmaStart(void);
void UpperCP_UartDmaRxProcess(void);
void UpperCP_SendTask(const char *task);
const char *UpperCP_GetLastCommand(void);
uint32_t UpperCP_GetRxCount(void);
uint8_t UpperCP_GetLastByte(void);

void cmd_func(void);
void speed_func(void);
void angle_func(void);
void face_func(void);
void voice_func(void);
void Arm_func(void);
void ErWeiMa_func(void);
void Move_func(void);

#endif
