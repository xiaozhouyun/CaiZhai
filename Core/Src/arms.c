ove_down#include "arms.h"
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

/* 从完全收缩位开始累计的舵机角度变化量，单位：度。 */
static float s_extend_delta_deg = -80.0f;

/*
 * @brief  根据题图中的精确公式计算连杆长度
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

/* 将公式的绝对几何长度换算为从 -80° 收缩端开始的相对伸出量。 */
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
  * @note   5 是旧工程使用的升降步进电机地址；
  *         dir=0 表示上升方向；
  *         snF=true 表示先缓存运动命令，随后用同步命令触发。
  */
void Move_up(float Data_cm)
{
    Emm_V5_PosUP_Control(5, 0, 200, 200, Data_cm * 10.0f, false, true);
    Emm_V5_Synchronous_motion(0);
    now_pos += Data_cm;
}

/**
  * @brief  升降机构下降指定距离
  * @param  Data_cm 下降距离，单位 cm
  * @note   dir=1 表示下降方向；运动完成后同步更新 now_pos。
  */
void Move_down(float Data_cm)
{
    Emm_V5_PosUP_Control(5, 1, 200, 200, Data_cm * 10.0f, false, true);
    Emm_V5_Synchronous_motion(0);
    now_pos -= Data_cm;
}

/**
  * @brief  移动到目标高度位置
  * @param  Tar_pos 目标位置，单位 cm
  * @note   函数会用 Tar_pos - now_pos 算出相对移动距离：
  *         结果为正就上升，结果为负就下降。
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
  * @brief  机械臂伸缩控制函数
  * @param  dist_cm 本次相对伸缩距离，单位 cm；正值伸出，负值缩回
  * @note   本函数保存累计长度，使用精确公式反解累计角度变化量后，
  *         再叠加到 -80° 初始角度，并在内部拆成 100 步平滑执行。
  */
void extend_cm(float dist_cm)
{
    float current_length_mm;
    float target_length_mm;
    float next_delta_deg;
    float next_length_mm;
    float last_length_mm;
    float angle_deg;

    if (Arm_ExtendDistanceFromDelta(s_extend_delta_deg, &current_length_mm) != 0)
    {
        return;
    }

    target_length_mm = current_length_mm + dist_cm * 10.0f;
    next_delta_deg = s_extend_delta_deg;

    /*
     * 逐步反解 L(Δθ)：正距离向较大角度搜索，负距离向较小角度搜索。
     * 这样 100 次小增量调用时，每次只前进一个很小的舵机角度。
     */
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
            /* 公式长度开始回落时，已到可伸出的极限，不能继续寻找另一分支。 */
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

    /* 一次完整伸缩动作在此处分成 100 步，每步间隔 15 ms。 */
    if (PCA9685_Set180AngleSmooth(6U, angle_deg, 100U, 15U) == 0)
    {
        s_extend_delta_deg = angle_deg - ARM_EXTEND_MIN_ANGLE_DEG;
    }
}

/**
  * @brief  爪子闭合/抓取控制函数
  * @note   用于驱动末端夹爪执行闭合动作以抓取目标对象
  */
void ZhuaZi_close(void)
{
    /* 爪子闭合/抓取控制逻辑实现预留 */
       PCA9685_Set180AngleSmooth(5U, 10, 100U, 10U);
}

/**
  * @brief  机械臂放置果子控制函数
  * @note   用于驱动机械臂与夹爪完成果子的松开与放置动作
  */
void Arm_put(void)
{
    /* 机械臂放置果子控制逻辑实现预留 */
    //缩回抬升后旋转
	Move_up(10.0f);
	vTaskDelay(pdMS_TO_TICKS(200U));
	PCA9685_Set180AngleSmooth(7U, 0, 100U, 10U);//回中
	vTaskDelay(pdMS_TO_TICKS(1000U));
	PCA9685_Set180AngleSmooth(6U, ARM_EXTEND_MIN_ANGLE_DEG, 100U, 10U);//缩回到最小点
	s_extend_delta_deg = 0.0f;
	//开爪
	ZhuaZi_open();
	vTaskDelay(pdMS_TO_TICKS(800U));
}

/**
  * @brief  爪子打开/松开控制函数
  * @note   用于驱动末端夹爪张开以释放目标对象
  */
void ZhuaZi_open(void)
{
    /* 爪子张开/释放控制逻辑实现预留 */
      PCA9685_Set180AngleSmooth(5U, -30, 100U, 10U);
}

/**
  * @brief  机械臂旋转角度控制（云台旋转舵机，通道3）
  * @param  angle_deg 目标角度，单位：度
  */
void Arm_SetRotateAngle(float angle_deg)
{
    PCA9685_Set180AngleSmooth(3U, angle_deg, 100U, 10U);
}
