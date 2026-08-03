#ifndef __VOFA_H
#define __VOFA_H

#include <stdint.h>

/* JustFloat 协议发送 */
void Vofa_SendFloat(const float *values, uint8_t count);

/* FireWater 协议发送浮点数组 */
void Vofa_SendFirewater(const float *values, uint8_t count);

/* FireWater 格式发送机器人状态数据 */
void Vofa_SendRobotStateFirewater(float x, float y, float yaw, float tof, float nav_state);

/* 字符串与格式化输出 */
void Vofa_SendString(const char *str);
void Vofa_Printf(const char *format, ...);

/* 打印 UpperCP 上位机数据 */
void Vofa_PrintUpperCPData(const char *cmd_buf);

#endif

