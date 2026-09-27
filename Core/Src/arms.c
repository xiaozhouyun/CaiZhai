#include "arms.h"
#include "pca9685.h"
#include "bujin.h"
#include "UpperCP.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "action_scheduler.h"
#include "vofa.h"
#include "odometer_pause.h"

#define LIFT_ODOMETER_PAUSE_MS        20U
#define LIFT_TX_IDLE_TIMEOUT_MS        5U
#define LIFT_RX_GUARD_MS               1U
#define ARM_EXTEND_CAL_POINT_COUNT     7U

/*
 * 实测伸缩标定（机构总长 -> 以 -70° 为零点的相对伸出量）：
 * -70°: 250mm ->   0mm；-50°: 270mm ->  20mm；-30°: 310mm ->  60mm；
 * -20°: 335mm ->  85mm；-10°: 350mm -> 100mm；  0°: 380mm -> 130mm；
 * +10°: 400mm -> 150mm。+10° 是当前已实测的最大位置，不向更大角度外推。
 *
 * 六段实测斜率依次为 1.0、2.0、2.5、1.5、3.0、2.0 mm/°，不能使用统一比例。
 * 相邻标定点之间使用分段线性插值，避免少量数据拟合高阶曲线后产生过冲。
 */
static const float s_arm_extend_angle_deg[ARM_EXTEND_CAL_POINT_COUNT] = {
    -70.0f, -50.0f, -30.0f, -20.0f, -10.0f, 0.0f, 10.0f
};
static const float s_arm_extend_distance_mm[ARM_EXTEND_CAL_POINT_COUNT] = {
      0.0f,  20.0f,  60.0f,  85.0f, 100.0f, 130.0f, 150.0f
};

/* 当前升降位置，单位 cm；Move_up/Move_down/Move_Pos 会维护这个值。 */
float volatile now_pos = 2.0f;

static bool Arms_PrepareLiftTx(void)
{
    Odometer_PausePolling(LIFT_ODOMETER_PAUSE_MS);
    Emm_ClearPendingTxQueue();
    if (!Emm_WaitTxIdle(LIFT_TX_IDLE_TIMEOUT_MS))
    {
        Vofa_Printf("[LIFT_DBG] USART2 busy timeout\r\n");
        return false;
    }

    osDelay(LIFT_RX_GUARD_MS);
    return true;
}

/**
  * @brief  升降机构上升指定距离
  * @param  Data_cm 上升距离，单位 cm
  */
void Move_down(float Data_cm)
{
    uint32_t drops_before;

    if (!Arms_PrepareLiftTx())
    {
        return;
    }

    drops_before = Emm_GetTxDropCount();
    Emm_V5_PosUP_Control(5, 0, 100, 50, Data_cm * 10.0f, false, 0);
    // Vofa_Printf("[LIFT_DBG] target=%.2f old=%.2f delta=%.2f drops=%lu\r\n",
    //             now_pos + Data_cm, now_pos, Data_cm,
    //             (unsigned long)Emm_GetTxDropCount());
    /* 仅当未发生丢帧时才更新位置跟踪，防止 now_pos 与实际物理位置脱节 */
    if (Emm_GetTxDropCount() == drops_before) {
        now_pos += Data_cm;
    }
}

/**
  * @brief  升降机构下降指定距离
  * @param  Data_cm 下降距离，单位 cm
  */
void Move_up(float Data_cm)
{
    uint32_t drops_before;

    if (!Arms_PrepareLiftTx())
    {
        return;
    }

    drops_before = Emm_GetTxDropCount();
    Emm_V5_PosUP_Control(5, 1, 100, 50, Data_cm * 10.0f, false, 0);
    // Vofa_Printf("[LIFT_DBG] target=%.2f old=%.2f delta=%.2f drops=%lu\r\n",
    //             now_pos - Data_cm, now_pos, Data_cm,
    //             (unsigned long)Emm_GetTxDropCount());
    /* 仅当未发生丢帧时才更新位置跟踪，防止 now_pos 与实际物理位置脱节 */
    if (Emm_GetTxDropCount() == drops_before) {
        now_pos -= Data_cm;
    }
}

/**
  * @brief  移动到目标高度位置
  * @param  Tar_pos 目标位置，单位 cm
  */
void Move_Pos(float Tar_pos)
{
    float move_pos = Tar_pos - now_pos;
    float prev_pos = now_pos;

    if (move_pos > 0.0f)
    {
        Move_up(move_pos);
    }
    else if (move_pos < 0.0f)
    {
        Move_down(-move_pos);
    }

    /* 仅当 Move_up/Move_down 确认更新了 now_pos 时，才同步；
     * 若丢帧导致 now_pos 未变，保留原值等待下次重试，避免位置跟踪漂移。 */
    if (now_pos == prev_pos && move_pos != 0.0f) {
        /* 升降指令未能发送，now_pos 未更新，本次不覆盖 */
        return;
    }
    now_pos = Tar_pos;
}

/* 正向模型：舵机角度 θ -> 相对伸出量 x(mm)，超出实测范围时钳位到端点。 */
static float Arm_ExtendDistanceMmForAngle(float angle_deg)
{
    uint8_t i;

    if (angle_deg <= s_arm_extend_angle_deg[0]) {
        return s_arm_extend_distance_mm[0];
    }
    if (angle_deg >= s_arm_extend_angle_deg[ARM_EXTEND_CAL_POINT_COUNT - 1U]) {
        return s_arm_extend_distance_mm[ARM_EXTEND_CAL_POINT_COUNT - 1U];
    }

    for (i = 0U; i < (ARM_EXTEND_CAL_POINT_COUNT - 1U); i++) {
        if (angle_deg <= s_arm_extend_angle_deg[i + 1U]) {
            /* x = x0 + (θ-θ0)/(θ1-θ0) * (x1-x0) */
            float ratio = (angle_deg - s_arm_extend_angle_deg[i])
                        / (s_arm_extend_angle_deg[i + 1U] - s_arm_extend_angle_deg[i]);
            return s_arm_extend_distance_mm[i]
                 + ratio * (s_arm_extend_distance_mm[i + 1U] - s_arm_extend_distance_mm[i]);
        }
    }

    return s_arm_extend_distance_mm[ARM_EXTEND_CAL_POINT_COUNT - 1U];
}

/* 反向模型：目标伸出量 x(mm) -> 舵机角度 θ，供厘米控制和 TOF 抓取共同使用。 */
static float Arm_ExtendAngleForDistanceMm(float distance_mm)
{
    uint8_t i;

    if (distance_mm <= s_arm_extend_distance_mm[0]) {
        return s_arm_extend_angle_deg[0];
    }
    if (distance_mm >= s_arm_extend_distance_mm[ARM_EXTEND_CAL_POINT_COUNT - 1U]) {
        return s_arm_extend_angle_deg[ARM_EXTEND_CAL_POINT_COUNT - 1U];
    }

    for (i = 0U; i < (ARM_EXTEND_CAL_POINT_COUNT - 1U); i++) {
        if (distance_mm <= s_arm_extend_distance_mm[i + 1U]) {
            /* θ = θ0 + (x-x0)/(x1-x0) * (θ1-θ0) */
            float ratio = (distance_mm - s_arm_extend_distance_mm[i])
                        / (s_arm_extend_distance_mm[i + 1U] - s_arm_extend_distance_mm[i]);
            return s_arm_extend_angle_deg[i]
                 + ratio * (s_arm_extend_angle_deg[i + 1U] - s_arm_extend_angle_deg[i]);
        }
    }

    return s_arm_extend_angle_deg[ARM_EXTEND_CAL_POINT_COUNT - 1U];
}

/**
  * @brief  根据实测分段模型换算相对伸缩距离
  * @param  dist_cm 本次相对伸缩距离，单位 cm；正值伸出，负值缩回
  */
float Arm_ExtendAngleForDeltaCm(float current_angle_deg, float dist_cm)
{
    /* 先由当前指令角度估算位置，再叠加相对位移，最后通过反向模型求目标角度。 */
    float current_distance_mm = Arm_ExtendDistanceMmForAngle(current_angle_deg);
    float target_distance_mm = current_distance_mm + dist_cm * 10.0f;

    return Arm_ExtendAngleForDistanceMm(target_distance_mm);
}

void extend_cm(float dist_cm)
{
    /* Get180Angle 返回指令记录，并非舵机实测反馈。 */
    float target_angle_deg = Arm_ExtendAngleForDeltaCm(PCA9685_Get180Angle(6U), dist_cm);

    (void)PCA9685_Set180AngleSmooth(6U, target_angle_deg, 100U, 15U);
}

/**
  * @brief  机械臂伸缩机构归零/复位到实测完全收缩位置 (-70°)
  * @note   驱动通道 6U 伸缩舵机平滑旋转至共用收缩限位
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
    (void)PCA9685_Set180AngleSmooth(5U, 5.0f, 100U, 10U);
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
    /*
     * 遗留同步接口：当前状态机不调用本函数。
     * 若被其他模块调用，也保持“先收臂 -> 升至10cm并等待 -> 再转云台”的顺序。
     */
    PCA9685_Set180AngleSmooth(6U, ARM_EXTEND_MIN_ANGLE_DEG, 100U, 10U);
    Move_Pos(10.0f);
    vTaskDelay(pdMS_TO_TICKS(3000U));
    ActionScheduler_StartGimbalMove(0.0f, 1000U);
    vTaskDelay(pdMS_TO_TICKS(1000U));
    ZhuaZi_open();
    vTaskDelay(pdMS_TO_TICKS(1000U));
}

/**
  * @brief  机械臂旋转角度控制（云台旋转舵机，通道 7U）
  * @param  angle_deg 目标角度，单位：度
  */
void Arm_SetRotateAngle(float angle_deg)
{
    /* 所有外部云台调用都经此非阻塞接口，接口内部会先升到10cm。 */
    ActionScheduler_StartGimbalMove(angle_deg, 1000U);
}
