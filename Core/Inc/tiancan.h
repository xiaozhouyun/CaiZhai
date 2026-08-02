#ifndef __TIANCAN_H
#define __TIANCAN_H

#include <stdbool.h>
#include <stdint.h>

/** 注册 PID 控制器到天蚕调参系统 */
bool Tiancan_RegisterPid(const char *name, float *kp, float *ki, float *kd);
void Tiancan_RxByte(uint8_t data);
void Tiancan_Process(void);

#endif
