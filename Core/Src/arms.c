#include "arms.h"
#include "pca9685.h"
#include "bujin.h"
#include "UpperCP.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#define ARM_EXTEND_MIN_ANGLE_DEG      (-80.0f)
#define ARM_EXTEND_MAX_ANGLE_DEG      (40.0f)
#define ARM_EXTEND_SEARCH_STEP_DEG    (0.05f)
#define ARM_EXTEND_DEG_TO_RAD         (0.01745329252f)

/**
 * @brief  根据机构连杆几何精确公式计算连杆长度
 * @param  delta_deg 相对完全收缩位的角度变化量，单位：度
 * @param  length_mm 计算得到的长度，单位：mm
 * @return 0 成功；-1 表示该角度不在公式的有效定义域内
 */
static int32_t Arm_ExtendLengthFromDelta(float delta_deg, float *length_mm)
{
    const float a = 15845.0f / 164.0f;
    const float b = 5.0f * sqrtf(21049215.0f) / 164.0f;
    const float c = 205.0f / 2.0f;
    float delta_rad;
    float inner;
    float radicand;

    delta_rad = delta_deg * ARM_EXTEND_DEG_TO_RAD;
    inner = b * cosf(delta_rad) - a * sinf(delta_rad) - c;
    radicand = 19600.0f - inner * inner;
    if (radicand < 0.0f)
    {
        return -1;
    }

    *length_mm = a * cosf(delta_rad) + b * sinf(delta_rad) + sqrtf(radicand);
    return 0;
}

/**
 * @brief  将公式的绝对几何长度换算为从 -80° 收缩端开始的相对伸出量
 * @param  delta_deg 角度变化量，单位：度
 * @param  distance_mm 伸出位移量，单位：mm
 * @return 0 成功；-1 失败
 */
static int32_t Arm_ExtendDistanceFromDelta(float delta_deg, float *distance_mm)
{
    float base_length_mm;
    float current_length_mm;

    if (Arm_ExtendLengthFromDelta(0.0f, &base_length_mm) != 0 ||
        Arm_ExtendLengthFromDelta(delta_deg, &current_length_mm) != 0)
    {
        return -1;
    }

    *distance_mm = current_length_mm - base_length_mm;
    return 0;
}

/* 当前升降位置，单位 cm；Move_up/Move_down/Move_Pos 会维护这个值。 */
float now_pos = 0.0f;

/**
  * @brief  升降机构上升指定距离
  * @param  Data_cm 上升距离，单位 cm
  */
void Move_up(float Data_cm)
{
    Emm_V5_PosUP_Control(5, 0, 50, 50, Data_cm * 10.0f, false, true);
    Emm_V5_Synchronous_motion(0);
    now_pos += Data_cm;
}

/**
  * @brief  升降机构下降指定距离
  * @param  Data_cm 下降距离，单位 cm
  */
void Move_down(float Data_cm)
{
    Emm_V5_PosUP_Control(5, 1, 500, 50, Data_cm * 10.0f, false, true);
    Emm_V5_Synchronous_motion(0);
    now_pos -= Data_cm;
}

/**
  * @brief  移动到目标高度位置
  * @param  Tar_pos 目标位置，单位 cm
  */
void Move_Pos(float Tar_pos)
{
    float move_pos = Tar_pos - now_pos;

    if (move_pos > 0.0f)
    {
        Move_up(move_pos);
    }
    else
    {
        Move_down(-move_pos);
    }

    now_pos = Tar_pos;
}

/**
  * @brief  机械臂伸缩控制函数（使用 PCA9685 通道 6U 统一维护舵机角度）
  * @param  dist_cm 本次相对伸缩距离，单位 cm；正值伸出，负值缩回
  * @note   已删除孤立变量 s_extend_delta_deg，改为直接从 PCA9685 角度记录数组 
  *         s_pca9685_180_angles[6] (通过 PCA9685_Get180Angle(6U)) 读取通道 6 当前实际角度。
  */
void extend_cm(float dist_cm)
{
    float current_angle_deg;
    float current_delta_deg;
    float current_length_mm;
    float target_length_mm;
    float next_delta_deg;
    float next_length_mm;
    float last_length_mm;
    float angle_deg;

    /* 直接从 s_pca9685_180_angles[6] 获取伸缩通道 6U 当前实际角度 */
    current_angle_deg = PCA9685_Get180Angle(6U);
    current_delta_deg = current_angle_deg - ARM_EXTEND_MIN_ANGLE_DEG;
    if (current_delta_deg < 0.0f)
    {
        current_delta_deg = 0.0f;
    }

    if (Arm_ExtendDistanceFromDelta(current_delta_deg, &current_length_mm) != 0)
    {
        return;
    }

    target_length_mm = current_length_mm + dist_cm * 10.0f;
    next_delta_deg = current_delta_deg;

    /* 逐步反解 L(Δθ)：正距离向较大角度搜索，负距离向较小角度搜索 */
    if (dist_cm > 0.0f)
    {
        last_length_mm = current_length_mm;
        while (next_delta_deg < (ARM_EXTEND_MAX_ANGLE_DEG - ARM_EXTEND_MIN_ANGLE_DEG))
        {
            next_delta_deg += ARM_EXTEND_SEARCH_STEP_DEG;
            if (Arm_ExtendDistanceFromDelta(next_delta_deg, &next_length_mm) != 0)
            {
                next_delta_deg -= ARM_EXTEND_SEARCH_STEP_DEG;
                break;
            }
            if (next_length_mm < last_length_mm)
            {
                next_delta_deg -= ARM_EXTEND_SEARCH_STEP_DEG;
                break;
            }
            if (next_length_mm >= target_length_mm)
            {
                break;
            }
            last_length_mm = next_length_mm;
        }
    }
    else if (dist_cm < 0.0f)
    {
        while (next_delta_deg > 0.0f)
        {
            next_delta_deg -= ARM_EXTEND_SEARCH_STEP_DEG;
            if (Arm_ExtendDistanceFromDelta(next_delta_deg, &next_length_mm) != 0 ||
                next_length_mm <= target_length_mm)
            {
                break;
            }
        }
    }

    angle_deg = ARM_EXTEND_MIN_ANGLE_DEG + next_delta_deg;
    if (angle_deg > ARM_EXTEND_MAX_ANGLE_DEG)
    {
        angle_deg = ARM_EXTEND_MAX_ANGLE_DEG;
    }
    else if (angle_deg < ARM_EXTEND_MIN_ANGLE_DEG)
    {
        angle_deg = ARM_EXTEND_MIN_ANGLE_DEG;
    }

    /* 驱动通道 6U 伸缩舵机平滑旋转，PCA9685 内部会自动更新 s_pca9685_180_angles[6] = angle_deg */
    (void)PCA9685_Set180AngleSmooth(6U, angle_deg, 100U, 15U);
}

/**
  * @brief  机械臂伸缩机构归零/复位到完全收缩位置 (-80°)
  * @note   驱动通道 6U 伸缩舵机平滑旋转至 ARM_EXTEND_MIN_ANGLE_DEG (-80.0f)
  */
void Arm_ExtendZero(void)
{
    (void)PCA9685_Set180AngleSmooth(6U, ARM_EXTEND_MIN_ANGLE_DEG, 100U, 15U);
}

/**
  * @brief  爪子闭合/抓取控制函数（通道 5U）
  * @note   驱动通道 5 夹爪舵机闭合 (+10°)
  */
void ZhuaZi_close(void)
{
    (void)PCA9685_Set180AngleSmooth(5U, 10.0f, 100U, 10U);
}

/**
  * @brief  爪子打开/松开控制函数（通道 5U）
  * @note   驱动通道 5 夹爪舵机张开 (-30°)
  */
void ZhuaZi_open(void)
{
    (void)PCA9685_Set180AngleSmooth(5U, -30.0f, 100U, 10U);
}

/**
  * @brief  机械臂放置果子控制函数
  * @note   用于驱动机械臂与夹爪完成果子的松开与放置动作
  */
void Arm_put(void)
{
    /* 缩回抬升后旋转 */
    Move_up(8.5f);
    vTaskDelay(pdMS_TO_TICKS(100U));
    PCA9685_Set180AngleSmooth(7U, 0, 100U, 10U); // 7U 云台回中
    vTaskDelay(pdMS_TO_TICKS(1000U));
    Arm_ExtendZero(); // 6U 伸缩机构归零/缩回到最小点 (-80°)
    vTaskDelay(pdMS_TO_TICKS(200U));
    /* 开爪 */
    ZhuaZi_open();
    vTaskDelay(pdMS_TO_TICKS(800U));
}

/**
  * @brief  机械臂旋转角度控制（云台旋转舵机，通道 7U）
  * @param  angle_deg 目标角度，单位：度
  */
void Arm_SetRotateAngle(float angle_deg)
{
    (void)PCA9685_Set180AngleSmooth(7U, angle_deg, 100U, 10U);
}
