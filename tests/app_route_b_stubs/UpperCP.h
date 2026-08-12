#ifndef TEST_UPPERCP_H
#define TEST_UPPERCP_H

#include <stdint.h>

extern uint8_t CameraFlag;
extern uint8_t fruits[8];
extern uint8_t fruits_count;

void UpperCP_ResetQrResult(void);
void UpperCP_SendTask(const char *task);

#endif
