#include "pca9685.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"

#define PCA9685_ADDRESS          (0x40U << 1)
#define PCA9685_MODE1            0x00U
#define PCA9685_PRESCALE         0xFEU
#define PCA9685_LED0_ON_L        0x06U
#define PCA9685_ALL_LED_ON_L     0xFAU
#define PCA9685_ALL_LED_OFF_L    0xFCU
#define PCA9685_PRESCALE_50HZ    121U
#define PCA9685_270_CENTER_DEG   65.0f

/* 保存 PCA9685 全部 0~15 通道的当前记录角度（初始均为 0.0 度）。 */
static float s_pca9685_180_angles[PCA9685_CHANNEL_COUNT] = {0.0f};

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

    if (channel >= PCA9685_CHANNEL_COUNT)
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
    if (channel >= PCA9685_CHANNEL_COUNT)
    {
        return 0.0f;
    }

    return s_pca9685_180_angles[channel];
}

int32_t PCA9685_Set180Angle(uint8_t channel, float angle_deg)
{
    if (channel >= PCA9685_CHANNEL_COUNT)
    {
        return -1;
    }

    /* 记录并更新当前通道的角度状态 */
//    s_pca9685_180_angles[channel] = angle_deg;

    return PCA9685_SetTicks(channel, PCA9685_PulseUsToTicks(
        PCA9685_AngleToPulseUs(angle_deg, 90.0f)));
}

int32_t PCA9685_SetAll180Angle(float angle_deg)
{
    uint8_t data[4];
    uint16_t off_ticks = PCA9685_PulseUsToTicks(
        PCA9685_AngleToPulseUs(angle_deg, 90.0f));

    data[0] = 0U;
    data[1] = 0U;
    data[2] = (uint8_t)(off_ticks & 0xFFU);
    data[3] = (uint8_t)(off_ticks >> 8);

    /* 广播写入 ALL_LED 寄存器，0~15 通道同时动作 */
    if (PCA9685_Write(PCA9685_ALL_LED_ON_L, data, sizeof(data)) != 0)
    {
        return -1;
    }

    /* 同步软件角度记录：全部通道归位到目标角度 */
    for (uint8_t i = 0U; i < PCA9685_CHANNEL_COUNT; i++)
    {
        s_pca9685_180_angles[i] = angle_deg;
    }

    return 0;
}

int32_t PCA9685_ResetAllToZero(void)
{
    /* 通道 0：270° 舵机，走独立角度计算（中心偏移 +65°） */
    PCA9685_Set270Angle(0.0f);
    s_pca9685_180_angles[0] = 0.0f;

    /* 通道 1~15：180° 舵机，逐个归零 */
    for (uint8_t i = 1U; i < PCA9685_CHANNEL_COUNT; i++)
    {
        PCA9685_Set180Angle(i, 0.0f);
        s_pca9685_180_angles[i] = 0.0f;
    }

    return 0;
}

/**
 * @brief  平滑驱动指定通道 180° 舵机旋转至目标角度（插值平滑插帧控制）
 * @param  channel          舵机通道号 (0 ~ 15)
 *                          - 1U: 云台舵机 (-90° 至 +90°)
 *                          - 3U: 伸缩机构舵机 (-80° 至 +40°)
 *                          - 4U: 机械爪夹紧舵机 (-30° 张开, 10° 闭合)
 * @param  target_angle_deg 目标角度（单位：度）
 * @param  steps            平滑细化步数（分割出的微小插值步骤总数，steps > 0）
 * @param  step_delay_ms    每一步微小动作之间的延迟时间（单位：毫秒 ms）
 * @return int32_t 0 成功到达目标角度，-1 参数非法（通道溢出或 steps 为 0）
 */
int32_t PCA9685_Set180AngleSmooth(uint8_t channel, float target_angle_deg, uint16_t steps, uint32_t step_delay_ms)
{
    /* 入参合法性校验：确保通道号不超出范围且平滑细化步数大于零 */
    if (channel >= PCA9685_CHANNEL_COUNT || steps == 0U)
    {
        return -1;
    }

    /* 1. 读取该通道当前记录的角度值，计算与目标角度之间的偏置残差 err */
    float current_angle = s_pca9685_180_angles[channel];
    float err = target_angle_deg - current_angle;

    /* 2. 计算平均分配到每一步的角度递增量 */
    float step_angle = err / (float)steps;

    /* 3. 分步循环累加角度并输出，将突变大动作离散平滑化，确保机械臂/舵机运动丝滑 */
    for (uint16_t i = 0U; i < steps; i++)
    {
        current_angle += step_angle;
        PCA9685_Set180Angle(channel, current_angle);

        if (step_delay_ms > 0U)
        {
            /* 兼容 RTOS 与裸机：若 FreeRTOS 调度器处于运行状态，使用 vTaskDelay 让出 CPU；否则降级使用 HAL_Delay 阻塞等待 */
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

    /* 4. 最终精确校准并刷新记录该通道的目标角度值 */
    return PCA9685_Set180Angle(channel, target_angle_deg);
}
