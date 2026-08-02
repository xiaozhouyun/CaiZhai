#ifndef __PCA9685_H
#define __PCA9685_H

#include <stdint.h>

#define PCA9685_MIN_PULSE_US     500U
#define PCA9685_MAX_PULSE_US     2500U
#define PCA9685_PERIOD_US        20000U
#define PCA9685_CHANNEL_COUNT    16U

static inline uint16_t PCA9685_PulseUsToTicks(uint16_t pulse_us)
{
    if (pulse_us < PCA9685_MIN_PULSE_US)
    {
        pulse_us = PCA9685_MIN_PULSE_US;
    }
    if (pulse_us > PCA9685_MAX_PULSE_US)
    {
        pulse_us = PCA9685_MAX_PULSE_US;
    }

    return (uint16_t)(((uint32_t)pulse_us * 4096U + (PCA9685_PERIOD_US / 2U)) /
                      PCA9685_PERIOD_US);
}

int32_t PCA9685_Init(void);
int32_t PCA9685_Set270Angle(float angle_deg);
/**
 * @brief  获取指定通道 180° 舵机的当前记录角度
 * @param  channel: PCA9685 通道 (0~15)
 * @return 当前角度（度）
 */
float PCA9685_Get180Angle(uint8_t channel);

/**
 * @brief  直接设置指定通道 180° 舵机的目标角度（瞬间到位）
 * @param  channel: PCA9685 通道 (0~15)
 * @param  angle_deg: 目标角度 (-90.0f 至 90.0f)
 * @return 0 成功，-1 失败
 */
int32_t PCA9685_Set180Angle(uint8_t channel, float angle_deg);

/**
 * @brief  广播设置全部 0~15 通道舵机到同一角度（一次性硬件广播，非逐个遍历）
 * @param  angle_deg: 目标角度 (-90.0f 至 90.0f)
 * @return 0 成功，-1 失败
 */
int32_t PCA9685_SetAll180Angle(float angle_deg);

/**
 * @brief  上电初始化：所有舵机通道归零
 *         - 通道 0（270° 舵机）：走 Set270Angle(0)，物理中心位
 *         - 通道 1~15（180° 舵机）：走 Set180Angle(0)
 * @return 0 成功
 */
int32_t PCA9685_ResetAllToZero(void);

/**
 * @brief  平滑设置指定通道 180° 舵机的目标角度（分步平滑、非硬延迟）
 * @param  channel: PCA9685 通道 (0~15)
 * @param  target_angle_deg: 目标角度 (-90.0f 至 90.0f)
 * @param  steps: 拆分的细微步数（例如 100）
 * @param  step_delay_ms: 每微步的时间间隔（单位 ms，RTOS 下自动休眠让出 CPU）
 * @return 0 成功，-1 失败
 */
int32_t PCA9685_Set180AngleSmooth(uint8_t channel, float target_angle_deg, uint16_t steps, uint32_t step_delay_ms);

#endif
