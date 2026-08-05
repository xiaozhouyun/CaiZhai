#ifndef __ARMS_H
#define __ARMS_H

#include "main.h"

/**
 * @brief  机械臂升降机构控制模块头文件
 */

/* 全局导出变量 */
extern float volatile now_pos;  /**< 当前升降机构位置，单位：cm */

/* 函数声明 */

/**
 * @brief  升降机构上升指定距离
 * @param  Data_cm 上升距离，单位：cm
 */
void Move_up(float Data_cm);

/**
 * @brief  升降机构下降指定距离
 * @param  Data_cm 下降距离，单位：cm
 */
void Move_down(float Data_cm);

/**
 * @brief  移动到指定的目标高度位置
 * @param  Tar_pos 目标位置，单位：cm
 */
void Move_Pos(float Tar_pos);

/**
 * @brief  机械臂伸缩控制函数
 * @param  dist_cm 机械臂伸缩距离，单位：cm
 */
void extend_cm(float dist_cm);

/**
 * @brief  机械臂伸缩机构归零/复位到完全收缩位置 (-80°)
 */
void Arm_ExtendZero(void);

/**
 * @brief  爪子闭合/抓取控制函数
 */
void ZhuaZi_close(void);

/**
 * @brief  爪子打开/松开控制函数
 */
void ZhuaZi_open(void);

/**
 * @brief  机械臂放置果子控制函数
 */
void Arm_put(void);

/**
 * @brief  机械臂旋转角度控制（云台旋转）
 * @param  angle_deg 目标角度，单位：度
 */
void Arm_SetRotateAngle(float angle_deg);

#endif /* __ARMS_H */
