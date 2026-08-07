#ifndef ACTION_SCHEDULER_H
#define ACTION_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

/** @brief 初始化抓取/云台非阻塞状态机；在创建任务前调用一次。 */
void ActionScheduler_Init(void);
/** @brief 投递视觉 arm:0~6 命令；忙碌时缓存最新 arm:0/5/6，丢弃对准微调命令。 */
void ActionScheduler_RequestVisionArm(uint8_t command);
/** @brief 推进一次状态机；由 StartTask07 每 20ms 调用，函数内禁止延时。 */
void ActionScheduler_Tick(void);
/** @brief 启动通道7云台的非阻塞匀速转动；实际插补由 ActionScheduler_Tick 完成。 */
void ActionScheduler_StartGimbalMove(float target_angle_deg, uint32_t duration_ms);
/** @brief 查询云台是否仍在“抬升至10cm”或“匀速转动”阶段。 */
bool ActionScheduler_IsGimbalBusy(void);
/** @brief 取消尚未完成的动作序列；不主动改变当前已下发的舵机位置。 */
void ActionScheduler_Cancel(void);
/** @brief 查询是否有等待中的抓取、复位或坏果清理序列。 */
bool ActionScheduler_IsBusy(void);
/** @brief 直接设置伸缩臂到指定厘米距离（测试用）；不经过状态机。 */
void ActionScheduler_SetExtendCm(float distance_cm);

#endif
