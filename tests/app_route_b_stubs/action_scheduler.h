#ifndef TEST_ACTION_SCHEDULER_H
#define TEST_ACTION_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

void ActionScheduler_Cancel(void);
bool ActionScheduler_IsGimbalBusy(void);
void ActionScheduler_StartGimbalMove(float target_angle, uint32_t duration_ms);

#endif
