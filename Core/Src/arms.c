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

#define ARM_EXTEND_MIN_ANGLE_DEG      (-80.0f)
#define ARM_EXTEND_MAX_ANGLE_DEG      (25.0f)
#define ARM_EXTEND_TOTAL_RANGE_DEG    (105.0f)  /* -80°→+25° 总行程 */
#define ARM_EXTEND_TOTAL_RANGE_MM     (300.0f)  /* 对应最大伸出 30cm */
#define LIFT_ODOMETER_PAUSE_MS        20U
#define LIFT_TX_IDLE_TIMEOUT_MS        5U
#define LIFT_RX_GUARD_MS               1U

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

/**
  * @brief  机械臂伸缩控制函数（线性映射：105° ↔ 300mm）
  * @param  dist_cm 本次相对伸缩距离，单位 cm；正值伸出，负值缩回
  */
void extend_cm(float dist_cm)
{
    float current_angle_deg;
    float current_dist_mm;
    float target_dist_mm;
    float target_angle_deg;

    /* 读取通道 6U 当前实际角度 */
    current_angle_deg = PCA9685_Get180Angle(6U);

    /* 当前角度 → 当前伸出距离: dist = (angle + 80) / 105 * 300 */
    current_dist_mm = (current_angle_deg - ARM_EXTEND_MIN_ANGLE_DEG)
                      / ARM_EXTEND_TOTAL_RANGE_DEG * ARM_EXTEND_TOTAL_RANGE_MM;
    if (current_dist_mm < 0.0f)
    {
        current_dist_mm = 0.0f;
    }

    /* 目标伸出距离 = 当前 + 增量(cm→mm) */
    target_dist_mm = current_dist_mm + dist_cm * 10.0f;
    if (target_dist_mm < 0.0f)
    {
        target_dist_mm = 0.0f;
    }

    /* 目标距离 → 目标角度: angle = -80 + dist / 300 * 105 */
    target_angle_deg = ARM_EXTEND_MIN_ANGLE_DEG
                       + target_dist_mm / ARM_EXTEND_TOTAL_RANGE_MM * ARM_EXTEND_TOTAL_RANGE_DEG;

    /* 限位钳位 [-80°, +25°] */
    if (target_angle_deg > ARM_EXTEND_MAX_ANGLE_DEG)
    {
        target_angle_deg = ARM_EXTEND_MAX_ANGLE_DEG;
    }
    else if (target_angle_deg < ARM_EXTEND_MIN_ANGLE_DEG)
    {
        target_angle_deg = ARM_EXTEND_MIN_ANGLE_DEG;
    }

    /* 驱动通道 6U 伸缩舵机平滑旋转 */
    (void)PCA9685_Set180AngleSmooth(6U, target_angle_deg, 100U, 15U);
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
    /*
     * 遗留同步接口：当前状态机不调用本函数。
     * 若被其他模块调用，也保持“先收臂 -> 升至10cm并等待 -> 再转云台”的顺序。
     */
    PCA9685_Set180AngleSmooth(6U, -80.0f, 100U, 10U);
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
