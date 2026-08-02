#include "pca9685.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"

#define PCA9685_ADDRESS          (0x40U << 1)
#define PCA9685_MODE1            0x00U
#define PCA9685_PRESCALE         0xFEU
#define PCA9685_LED0_ON_L        0x06U
#define PCA9685_PRESCALE_50HZ    121U
#define PCA9685_270_CENTER_DEG   65.0f

/* 全局静态数组：保存通道 1~4 舵机的当前角度（初始默认为 0.0 度） */
static float s_pca9685_180_angles[5] = {0.0f};

static int32_t PCA9685_Write(uint8_t reg, uint8_t *data, uint16_t len)
{
    if (HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDRESS, reg,
                          I2C_MEMADD_SIZE_8BIT, data, len,
                          HAL_MAX_DELAY) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

static int32_t PCA9685_SetTicks(uint8_t channel, uint16_t off_ticks)
{
    uint8_t data[4];

    if (channel > 15U)
    {
        return -1;
    }

    data[0] = 0U;
    data[1] = 0U;
    data[2] = (uint8_t)(off_ticks & 0xFFU);
    data[3] = (uint8_t)(off_ticks >> 8);

    return PCA9685_Write((uint8_t)(PCA9685_LED0_ON_L + 4U * channel), data, sizeof(data));
}

static uint16_t PCA9685_AngleToPulseUs(float angle_deg, float max_angle_deg)
{
    float pulse_us;

    if (angle_deg < -max_angle_deg)
    {
        angle_deg = -max_angle_deg;
    }
    if (angle_deg > max_angle_deg)
    {
        angle_deg = max_angle_deg;
    }

    pulse_us = 1500.0f + angle_deg * 1000.0f / max_angle_deg;
    return (uint16_t)(pulse_us + 0.5f);
}

int32_t PCA9685_Init(void)
{
    uint8_t value;

    value = 0x10U;
    if (PCA9685_Write(PCA9685_MODE1, &value, 1U) != 0)
    {
        return -1;
    }

    value = PCA9685_PRESCALE_50HZ;
    if (PCA9685_Write(PCA9685_PRESCALE, &value, 1U) != 0)
    {
        return -1;
    }

    value = 0x00U;
    if (PCA9685_Write(PCA9685_MODE1, &value, 1U) != 0)
    {
        return -1;
    }

    HAL_Delay(1U);
    value = 0xA1U;
    return PCA9685_Write(PCA9685_MODE1, &value, 1U);
}
//-80到+80
int32_t PCA9685_Set270Angle(float angle_deg)
{
    return PCA9685_SetTicks(0U, PCA9685_PulseUsToTicks(
        PCA9685_AngleToPulseUs(angle_deg + PCA9685_270_CENTER_DEG, 135.0f)));
}

float PCA9685_Get180Angle(uint8_t channel)
{
    if (channel < 1U || channel > 4U)
    {
        return 0.0f;
    }

    return s_pca9685_180_angles[channel];
}
//1U给到机械爪 -80到
int32_t PCA9685_Set180Angle(uint8_t channel, float angle_deg)
{
    if (channel < 1U || channel > 4U)
    {
        return -1;
    }

    /* 记录并更新当前通道的角度状态 */
    s_pca9685_180_angles[channel] = angle_deg;

    return PCA9685_SetTicks(channel, PCA9685_PulseUsToTicks(
        PCA9685_AngleToPulseUs(angle_deg, 90.0f)));
}

int32_t PCA9685_Set180AngleSmooth(uint8_t channel, float target_angle_deg, uint16_t steps, uint32_t step_delay_ms)
{
    if (channel < 1U || channel > 4U || steps == 0U)
    {
        return -1;
    }

    /* 1. 读取当前角度，计算目标偏差 err */
    float current_angle = s_pca9685_180_angles[channel];
    float err = target_angle_deg - current_angle;

    /* 2. 计算每一步的角度递增量 */
    float step_angle = err / (float)steps;

    /* 3. 循环将动作分为 steps 次执行，确保运动丝滑 */
    for (uint16_t i = 0U; i < steps; i++)
    {
        current_angle += step_angle;
        (void)PCA9685_Set180Angle(channel, current_angle);

        if (step_delay_ms > 0U)
        {
            /* 若 RTOS 调度器已运行，使用 vTaskDelay 让出 CPU；否则使用 HAL_Delay */
            if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
            {
                vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
            }
            else
            {
                HAL_Delay(step_delay_ms);
            }
        }
    }

    /* 4. 确保最终精准到达目标角度 */
    return PCA9685_Set180Angle(channel, target_angle_deg);
}
