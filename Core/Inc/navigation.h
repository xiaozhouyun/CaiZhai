#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "main.h"
#include "tiancan.h"
#include <stdbool.h>

/**
 * @brief 导航运动状态机枚举
 */
typedef enum {
    NAVIGATION_STATE_IDLE,          /**< 空闲状态，底盘静止 */
    NAVIGATION_STATE_TARGET_ALIGN,  /**< 朝向目标点对齐状态（原地旋转） */
    NAVIGATION_STATE_MOVING,        /**< 向目标点直线行进状态（带航向纠偏） */
    NAVIGATION_STATE_ARRIVED        /**< 到达目标点，调整最终朝向角度状态 */
} Navigation_State_t;

/**
 * @brief 二维空间坐标与朝向角结构体
 */
typedef struct {
    float x;    /**< X 坐标，单位：毫米 (mm) */
    float y;    /**< Y 坐标，单位：毫米 (mm) */
    float yaw;  /**< 朝向角，单位：度 (deg) */
} position_t;

/**
 * @brief 辅助运动状态结构体（目前保留，主要兼容旧工程的调试结构）
 */
struct move {
    float tar;  /**< 目标值 */
    float real; /**< 实际测量值 */
    float diff; /**< 偏差 */
};

/* 导航与地图相关参数定义 */
#define NAV_MAP_WIDTH_MM          5200.0f       /**< 地图宽度，单位：mm */
#define NAV_MAP_HEIGHT_MM         2400.0f       /**< 地图高度，单位：mm */
#define NAV_START_CENTER_X_MM     0.0f          /**< 起始中心点 X 坐标 */
#define NAV_START_CENTER_Y_MM     0.0f          /**< 起始中心点 Y 坐标 */
#define NAV_A_B_LINE_X_MM         1000.0f       /**< 区域分割界限 X（A线） */
#define NAV_B_C_LINE_X_MM         2000.0f       /**< 区域分割界限 X（B线） */
#define NAV_C_D_LINE_X_MM         3600.0f       /**< 区域分割界限 X（C线） */
#define NAV_C_ROW_SPACING_MM      500.0f        /**< C行间距 */
#define NAV_D_ROW_SPACING_MM      600.0f        /**< D行间距 */
#define NAV_COLUMN_SPACING_MM     700.0f        /**< 列间距 */

/* 底盘动力学参数 */
#define WHEEL_RADIUS_MM           42.5f         /**< 驱动轮半径，单位：mm */
#define HALF_TRACK_MM             100.0f        /**< 轮距的一半（小车中心到左右轮的侧向距离），单位：mm */
#define PI                        3.14159265f
#define TWO_PI                    6.2831853f

/* 四驱电机对应的 CAN/RS485 站号/串口通信站号 */
#define left_head                 3             /**< 左前轮电机地址 */
#define left_tail                 2             /**< 左后轮电机地址 */
#define right_head                4             /**< 右前轮电机地址 */
#define right_tail                1             /**< 右后轮电机地址 */

/* 全局导出变量 */
extern volatile position_t g_robot_pos;         /**< 机器人当前位置（实时里程计累积） */
extern Navigation_State_t navigation_state;     /**< 当前导航状态 */
extern TiancanPid_t anglepid;                   /**< 旋转对齐角度 PID 全局变量（对应 Navigation_HandleTargetAlign 中的角度控制） */
extern struct move speed;                       /**< 速度控制结构体 */
extern struct move angle_speed;                 /**< 角速度控制结构体 */
extern float nav_yaw_zero_deg;                  /**< 零点偏差 */
extern int TarAngle;
extern float TarPos;
extern bool is_moving;
extern float angle_fix;
extern float v[2];                              /**< 左右轮计算出的实际控制速度，单位：mm/s */

/* 导航 API 声明 */

/**
 * @brief 规范化角度到 [-180, 180] 度区间
 */
float Navigation_NormalizeDeg(float angle);

/**
 * @brief 复位导航里程计位置到指定的起始坐标与偏航角
 * @param start_x_mm 起始 X 坐标
 * @param start_y_mm 起始 Y 坐标
 * @param yaw_zero_deg 起始参考零偏角度
 */
void Navigation_Reset(float start_x_mm, float start_y_mm, float yaw_zero_deg);

/**
 * @brief 根据轮式里程计测得的单步位移量和陀螺仪测得的偏航角，更新导航位置
 * @param delta_mm 单步内的中心位移量，单位：mm
 * @param yaw_deg 陀螺仪或里程计融合后的绝对偏航角，单位：度
 */
void Navigation_UpdateByDelta(float delta_mm, float yaw_deg);

/**
 * @brief 导航状态机 tick 调度，周期性调用，用于实现闭环控制与状态机跳转
 */
void Navigation_TaskTick(void);

/**
 * @brief 请求导航到一个新的二维路径点
 * @param target_x_mm 目标 X 坐标 (mm)
 * @param target_y_mm 目标 Y 坐标 (mm)
 * @param target_yaw_rad 目标最终朝向角 (弧度)
 * @return 0 成功启动导航请求，-1 当前导航正忙
 */
int8_t Navigation_Request(float target_x_mm, float target_y_mm, float target_yaw_rad);

/**
 * @brief 检查导航模块是否处于空闲状态
 */
bool Navigation_IsIdle(void);

/**
 * @brief 强制停止导航运动并让底盘电机停转
 */
void Navigation_Stop(void);

/**
 * @brief 底盘控制接口：设置期望的线速度和角速度（执行差速动力学变换，下发串口命令并触发同步）
 * @param linear_vel_mm_s 前进线速度，单位：mm/s
 * @param angular_vel_rad_s 旋转角速度，单位：rad/s
 */
void Chassis_SetSpeed(float linear_vel_mm_s, float angular_vel_rad_s);

/**
 * @brief 获取当前偏航角（单位：度）
 */
float Navigation_GetYawDeg(void);



#endif

