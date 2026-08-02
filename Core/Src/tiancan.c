#include "tiancan.h"
#include <stdio.h>
#include <string.h>

#define TIANCAN_MAX_PID_COUNT 8U
#define TIANCAN_LINE_SIZE     64U
#define TIANCAN_NAME_SIZE     12U

static TiancanPid_t s_pids[TIANCAN_MAX_PID_COUNT];
static uint8_t s_pid_count;
static char s_line[TIANCAN_LINE_SIZE];
static volatile uint8_t s_line_length;
static volatile bool s_line_ready;

/**
 * @brief  注册 PID 控制器到天蚕调参系统
 * @param  name PID 控制器标识名称（如 "shoot", "align"）
 * @param  kp   指向 Kp 增益变量的指针（若不调节可传 NULL）
 * @param  ki   指向 Ki 增益变量的指针（若不调节可传 NULL）
 * @param  kp     指向 Kp 增益变量的指针（若不调节可传 NULL）
 * @param  ki     指向 Ki 增益变量的指针（若不调节可传 NULL）
 * @param  kd     指向 Kd 增益变量的指针（若不调节可传 NULL）
 * @param  target 指向目标期望值变量的指针（若不调节可传 NULL）
 * @return true 注册成功，false 注册失败（参数非法/重复注册/超出数组上限）
 */
bool Tiancan_RegisterPid(const char *name, float *kp, float *ki, float *kd, float *target)
{
    uint8_t i;

    /* 参数有效性校验：名称不能为空，且参数指针不能全为 NULL */
    if (name == NULL || (kp == NULL && ki == NULL && kd == NULL && target == NULL)) {
        return false;
    }

    /* 校验是否存在重复注册的同名 PID */
    for (i = 0U; i < s_pid_count; ++i) {
        if (strcmp(s_pids[i].name, name) == 0) {
            return false;
        }
    }

    /* 校验注册数量是否已达到系统最大容量上限 */
    if (s_pid_count >= TIANCAN_MAX_PID_COUNT) {
        return false;
    }

    /* 将 PID 参数及目标值指针信息写入注册表 */
    s_pids[s_pid_count].name = name;
    s_pids[s_pid_count].kp = kp;
    s_pids[s_pid_count].ki = ki;
    s_pids[s_pid_count].kd = kd;
    s_pids[s_pid_count].target = target;
    ++s_pid_count;

    return true;
}

void Tiancan_RxByte(uint8_t data)
{
    if (s_line_ready) {
        return;
    }

    if (data == '\r' || data == '\n') {
        if (s_line_length != 0U) {
            s_line[s_line_length] = '\0';
            s_line_ready = true;
        }
        return;
    }

    if (s_line_length < (TIANCAN_LINE_SIZE - 1U)) {
        s_line[s_line_length++] = (char)data;
    } else {
        s_line_length = 0U;
    }
}

static void Tiancan_SetGain(TiancanPid_t *pid, const char *gain, float value)
{
    if (strcmp(gain, "kp") == 0 && pid->kp != NULL) {
        *pid->kp = value;
    } else if (strcmp(gain, "ki") == 0 && pid->ki != NULL) {
        *pid->ki = value;
    } else if (strcmp(gain, "kd") == 0 && pid->kd != NULL) {
        *pid->kd = value;
    } else if ((strcmp(gain, "tar") == 0 || strcmp(gain, "target") == 0) && pid->target != NULL) {
        *pid->target = value;
    }
}

void Tiancan_Process(void)
{
    char name[TIANCAN_NAME_SIZE];
    char gain[7];
    float value;
    uint8_t i;

    if (!s_line_ready) {
        return;
    }

    s_line_ready = false;
    s_line_length = 0U;
    if (sscanf(s_line, "set %11s %6s %f", name, gain, &value) != 3) {
        return;
    }

    if (strcmp(name, "all") == 0) {
        for (i = 0U; i < s_pid_count; ++i) {
            Tiancan_SetGain(&s_pids[i], gain, value);
        }
        return;
    }

    for (i = 0U; i < s_pid_count; ++i) {
        if (strcmp(s_pids[i].name, name) == 0) {
            Tiancan_SetGain(&s_pids[i], gain, value);
            return;
        }
    }
}
