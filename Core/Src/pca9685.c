#include "pca9685.h"
#include "i2c.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

extern osMutexId_t duojiI2cHandle;

#define PCA9685_ADDRESS          (0x40U << 1)
#define PCA9685_MODE1            0x00U
#define PCA9685_PRESCALE         0xFEU
#define PCA9685_LED0_ON_L        0x06U
#define PCA9685_ALL_LED_ON_L     0xFAU
#define PCA9685_ALL_LED_OFF_L    0xFCU
#define PCA9685_PRESCALE_50HZ    121U
#define PCA9685_I2C_TIMEOUT_MS   20U
#define PCA9685_270_CENTER_DEG   0.0f   /* 零点平移：0° 即为舵机物理正中位 */
#define PCA9685_270_MIN_DEG     -135.0f /* 270°舵机限位下限 (-135°) */
#define PCA9685_270_MAX_DEG      135.0f /* 270°舵机限位上限 (+135°) */

/* 保存 PCA9685 全部 0~15 通道的当前记录角度（初始均为 0.0 度）。 */
float s_pca9685_180_angles[PCA9685_CHANNEL_COUNT] = {0.0f};
static float s_pca9685_270_angle = 0.0f;

static int32_t PCA9685_Write(uint8_t reg, uint8_t *data, uint16_t len)
{
    osStatus_t mutex_status = osOK;
    uint8_t use_mutex = 0U;
    HAL_StatusTypeDef status;

    if ((duojiI2cHandle != NULL) && (osKernelGetState() == osKernelRunning))
    {
        use_mutex = 1U;
        mutex_status = osMutexAcquire(duojiI2cHandle, PCA9685_I2C_TIMEOUT_MS);
    }

    if (mutex_status != osOK)
    {
        return -1;
    }

    status = HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDRESS, reg,
                               I2C_MEMADD_SIZE_8BIT, data, len,
                               PCA9685_I2C_TIMEOUT_MS);

    if (use_mutex != 0U)
    {
        (void)osMutexRelease(duojiI2cHandle);
    }

    if (status != HAL_OK)
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
float PCA9685_Get270Angle(void)
{
    return s_pca9685_270_angle;
}

/* 270° 舵机角度控制 (软限位范围：-135.0° 至 +135.0°) */
int32_t PCA9685_Set270Angle(float angle_deg)
{
    if (angle_deg < PCA9685_270_MIN_DEG)
    {
        angle_deg = PCA9685_270_MIN_DEG;
    }
    else if (angle_deg > PCA9685_270_MAX_DEG)
    {
        angle_deg = PCA9685_270_MAX_DEG;
    }

    s_pca9685_270_angle = angle_deg;
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
    s_pca9685_180_angles[channel] = angle_deg;

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
    PCA9685_Set270Angle(30.0f);
    s_pca9685_180_angles[0] = 30.0f;

    /* 通道 1~15：180° 舵机，逐个归零 */
    for (uint8_t i = 1U; i < PCA9685_CHANNEL_COUNT; i++)
    {
        float zero_angle = 0.0f;

        /* 通道 6：伸缩机构，零位 = -50°（完全收缩） */
        if (i == 6U)
        {
            zero_angle = -80.0f;
        }

        PCA9685_Set180Angle(i, zero_angle);
        s_pca9685_180_angles[i] = zero_angle;
    }

    return 0;
}

/**
 * @brief  平滑驱动指定通道 180° 舵机旋转至目标角度（插值平滑插帧控制）
 * @param  channel          舵机通道号 (0 ~ 15)
 *                          - 7U: 云台舵机 (-90° 至 +90°)
 *                          - 6U: 伸缩机构舵机 (-80° 至 +25°)
 *                          - 5U: 机械爪夹紧舵机 (-30° 张开, 10° 闭合)
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

    /* 3. 匀速平滑驱动：使用绝对时间锚定（vTaskDelayUntil）替代相对延迟（vTaskDelay），
     *    确保每一步的时间间隔绝对恒定，自动补偿 I2C 写入耗时波动，实现真正的匀速运动。
     *    裸机环境下使用 HAL_GetTick 做补偿式延迟，减少抖动。 */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        /* --- FreeRTOS：vTaskDelayUntil 绝对时间锚定，匀速输出 --- */
        TickType_t xLastWakeTime = xTaskGetTickCount();
        TickType_t xPeriod = pdMS_TO_TICKS(step_delay_ms);

        /* 防止 step_delay_ms 过小导致周期为 0 tick（vTaskDelayUntil 用 0 tick 是未定义行为） */
        if (xPeriod == 0U)
        {
            xPeriod = 1U;
        }

        for (uint16_t i = 0U; i < steps; i++)
        {
            current_angle += step_angle;
            PCA9685_Set180Angle(channel, current_angle);
            vTaskDelayUntil(&xLastWakeTime, xPeriod);
        }
    }
    else
    {
        /* --- 裸机：HAL_GetTick 补偿式延迟，扣除 I2C 耗时 --- */
        for (uint16_t i = 0U; i < steps; i++)
        {
            uint32_t t_start = HAL_GetTick();
            current_angle += step_angle;
            PCA9685_Set180Angle(channel, current_angle);

            if (step_delay_ms > 0U)
            {
                uint32_t elapsed = HAL_GetTick() - t_start;
                if (elapsed < step_delay_ms)
                {
                    HAL_Delay(step_delay_ms - elapsed);
                }
            }
        }
    }

    /* 4. 最终精确校准并刷新记录该通道的目标角度值 */
    return PCA9685_Set180Angle(channel, target_angle_deg);
}

/**
 * @brief  平滑驱动 270° 舵机（通道 0）旋转至目标角度（插值平滑插帧控制）
 * @param  target_angle_deg 目标角度（单位：度）
 * @param  steps            平滑细化步数（分割出的微小插值步骤总数，steps > 0）
 * @param  step_delay_ms    每一步微小动作之间的延迟时间（单位：毫秒 ms）
 * @return int32_t 0 成功到达目标角度，-1 参数非法（steps 为 0）
 */
int32_t PCA9685_Set270AngleSmooth(float target_angle_deg, uint16_t steps, uint32_t step_delay_ms)
{
    if (steps == 0U)
    {
        return -1;
    }

    /* 1. 读取 270° 舵机当前记录的角度值，计算残差 */
    float current_angle = s_pca9685_270_angle;
    float err = target_angle_deg - current_angle;

    /* 2. 计算平均分配到每一步的角度递增量 */
    float step_angle = err / (float)steps;

    /* 3. 匀速平滑驱动：绝对时间锚定，确保步间间隔恒定 */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        /* --- FreeRTOS：vTaskDelayUntil 绝对时间锚定，匀速输出 --- */
        TickType_t xLastWakeTime = xTaskGetTickCount();
        TickType_t xPeriod = pdMS_TO_TICKS(step_delay_ms);

        if (xPeriod == 0U)
        {
            xPeriod = 1U;
        }

        for (uint16_t i = 0U; i < steps; i++)
        {
            current_angle += step_angle;
            PCA9685_Set270Angle(current_angle);
            vTaskDelayUntil(&xLastWakeTime, xPeriod);
        }
    }
    else
    {
        /* --- 裸机：HAL_GetTick 补偿式延迟 --- */
        for (uint16_t i = 0U; i < steps; i++)
        {
            uint32_t t_start = HAL_GetTick();
            current_angle += step_angle;
            PCA9685_Set270Angle(current_angle);

            if (step_delay_ms > 0U)
            {
                uint32_t elapsed = HAL_GetTick() - t_start;
                if (elapsed < step_delay_ms)
                {
                    HAL_Delay(step_delay_ms - elapsed);
                }
            }
        }
    }

    /* 4. 最终精确到位 */
    return PCA9685_Set270Angle(target_angle_deg);
}
