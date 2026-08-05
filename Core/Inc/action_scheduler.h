#ifndef ACTION_SCHEDULER_H
#define ACTION_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

/** @brief 初始化抓取/云台非阻塞状态机；在创建任务前调用一次。 */
void ActionScheduler_Init(void);
/** @brief 投递视觉 arm:0~6 命令；忙碌时丢弃新抓取命令，避免动作重叠。 */
void ActionScheduler_RequestVisionArm(uint8_t command);
/** @brief 推进一次状态机；由 StartTask07 每 20ms 调用，函数内禁止延时。 */
void ActionScheduler_Tick(void);
/** @brief 取消尚未完成的动作序列；不主动改变当前已下发的舵机位置。 */
void ActionScheduler_Cancel(void);
/** @brief 查询是否有等待中的抓取、复位或坏果清理序列。 */
bool ActionScheduler_IsBusy(void);

#endif
