#ifndef __BUJIN_H
#define __BUJIN_H

#include "main.h"
#include <stdbool.h>

/** Emm_V5 闭环驱动器系统参数枚举 */
typedef enum {
    S_VER   = 0,  /**< 固件版本号 */
    S_RL    = 1,  /**< 相电阻/相电感 */
    S_PID   = 2,  /**< PID 参数 */
    S_VBUS  = 3,  /**< 母线电压 */
    S_CPHA  = 5,  /**< 相电流/相位 */
    S_ENCL  = 7,  /**< 编码器线数/位置 */
    S_TPOS  = 8,  /**< 目标位置 */
    S_VEL   = 9,  /**< 实时速度 */
    S_CPOS  = 10, /**< 实时/当前位置 */
    S_PERR  = 11, /**< 位置误差 */
    S_FLAG  = 13, /**< 状态标志位/到位标志 */
    S_Conf  = 14, /**< 系统配置参数 */
    S_State = 15, /**< 系统运行状态 */
    S_ORG   = 16, /**< 零点/原点位置及状态 */
} SysParams_t;

/** 回零模式枚举 */
typedef enum {
    ZERO_MODE_NEAR_ORIGIN_HOME  = 0,  /**< 近原点回零：朝原点方向运动，遇原点开关后反向找 Z 信号 */
    ZERO_MODE_ORIGIN_REV_HOME   = 1,  /**< 原点反方向回零：先反向离开原点，再正向找 Z 信号 */
    ZERO_MODE_ORIGIN_ONLY       = 2,  /**< 仅原点回零：直接找原点开关信号即停 */
    ZERO_MODE_FORCE_HOME        = 3,  /**< 强行回零：以当前位置为零点，不依赖外部传感器 */
} Emm_V5_Zero_Mode_t;

/** 回零参数结构体 */
typedef struct {
    uint8_t  mode;            /**< 回零模式，取值见 Emm_V5_Zero_Mode_t */
    uint8_t  direction;       /**< 回零方向，0=CW 顺时针，1=CCW 逆时针 */
    uint16_t speed_rpm;       /**< 回零速度，单位 RPM */
    uint32_t timeout_ms;      /**< 回零超时时间，单位 ms，0 表示不限时 */
    uint16_t collision_rpm;   /**< 堵转保护速度阈值，单位 RPM */
    uint16_t collision_ma;    /**< 堵转保护电流阈值，单位 mA */
    uint16_t collision_ms;    /**< 堵转保护时间窗口，单位 ms */
    uint8_t  auto_trigger;    /**< 上电自动触发回零，0=关闭，1=开启 */
} Emm_V5_Zero_Params_t;

void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, float mm, bool raF, bool snF);
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode);
void Emm_V5_Reset_Clog_Pro(uint8_t addr);
void Emm_V5_Synchronous_motion(uint8_t addr);
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);
void motor_to_angle_control(uint8_t addr, float angle, uint16_t vel, uint8_t acc);

/* 零点相关 */
void Emm_V5_Set_Zero(uint8_t addr, bool save);
void Emm_V5_Trigger_Zero(uint8_t addr, Emm_V5_Zero_Mode_t mode, bool snF);
void Emm_V5_Read_Zero_Params(uint8_t addr);
void Emm_V5_Modify_Zero_Params(uint8_t addr, bool save, const Emm_V5_Zero_Params_t *params);
void Emm_V5_Read_Zero_Status(uint8_t addr);
void Emm_V5_PosUP_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, float mm, bool raF, bool snF);
void Emm_V5_Chassis_Pos_Control(uint8_t dir, uint16_t vel, uint8_t acc, float mm);
void Emm_UartTxCpltCallback(UART_HandleTypeDef *huart);
void Emm_UartErrorCallback(UART_HandleTypeDef *huart);
#endif
