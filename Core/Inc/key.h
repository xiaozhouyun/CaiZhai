#ifndef __KEY_H
#define __KEY_H

#include "main.h"
#include "gpio.h"

/**
 * @brief  按键轮询扫描模块头文件（非中断方式）
 *
 * @note   引脚映射：
 *          - led1 (PB9), 内部上拉，按下为低电平
 *          - led3 (PE0), 内部上拉，按下为低电平
 *          - led2 (PC2), 内部上拉，按下为低电平
 */

/**
 * @brief  按键轮询扫描函数，需周期性调用（建议每 10~20ms 调用一次）
 * @note   内部实现软件消抖（约 30ms），检测下降沿触发按键动作
 *          - led1: 翻转 LED + Chassis_SetSpeed(0, 0.2) 原地旋转
 *          - led2: 预留
 *          - led3: 预留
 */
void Key_Scan(void);

#endif /* __KEY_H */
