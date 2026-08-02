#ifndef __TIANCAN_H
#define __TIANCAN_H

#include <stdbool.h>
#include <stdint.h>

/** 天蚕调参 PID 控制器结构体 */
typedef struct {
    const char *name;  /**< PID 控制器名称 */
    float *kp;         /**< 指向 Kp 参数的指针 */
    float *ki;         /**< 指向 Ki 参数的指针 */
    float *kd;         /**< 指向 Kd 参数的指针 */
    float *target;     /**< 指向目标期望值 (Target/Setpoint) 的指针 */
} TiancanPid_t;

/** 注册 PID 控制器到天蚕调参系统 */
bool Tiancan_RegisterPid(const char *name, float *kp, float *ki, float *kd, float *target);
void Tiancan_RxByte(uint8_t data);
void Tiancan_Process(void);

#endif
