#ifndef __APP_H
#define __APP_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 应用程序运行模式枚举
 */
typedef enum {
    APP_MODE_IDLE,      /**< 空闲模式：机器人处于静止状态，不执行任何航线 */
    APP_MODE_TEST,      /**< 测试模式：用于调试和测试功能，可能包含自定义的测试逻辑 */
    APP_MODE_ROUTE_A,   /**< 航线 A 模式：执行第一阶段的路径导航（例如向深处前行） */
    APP_MODE_ROUTE_B,   /**< 航线 B 模式：预留模式 */
    APP_MODE_ROUTE_C    /**< 航线 C 模式：执行第二阶段的路径导航（例如折返或区域内作业） */
} AppMode_t;

/**
 * @brief 路径点（航点）结构体
 */
typedef struct {
    float x_mm;         /**< 目标点 X 坐标，单位：毫米 */
    float y_mm;         /**< 目标点 Y 坐标，单位：毫米 */
    float yaw_rad;      /**< 目标点朝向角，单位：弧度 */
    bool has_action;    /**< 是否在到达目标点后执行舵机动作 (true/false) */
} AppWaypoint_t;

/* 全局应用模式变量，由导航任务或串口控制修改 */
extern volatile AppMode_t g_app_mode;

/**
 * @brief 初始化应用状态，复位模式和控制变量
 */
void App_Init(void);

/**
 * @brief 推进当前应用模式一次；由 StartTask07 每 20ms 调用，不阻塞。
 */
void App_RunCurrentMode(void);

/**
 * @brief 设置应用程序的目标运行模式
 * @param mode 目标模式
 */
void App_SetMode(AppMode_t mode);

/**
 * @brief 查询当前应用是否正在运行路径导航
 * @return true 正在运行，false 处于空闲或停止状态
 */
bool App_IsRunning(void);

/**
 * @brief 通知当前航线任务：上位机已完成一次抓取。
 */
void App_NotifyGrabDone(void);

/**
 * @brief 串口指令接收处理函数，用于解析外部启动/停止指令
 * @param data 接收到的串口数据字节 ('a'/'A' 启动, 't'/'T' 停止)
 */
void vofaRxbyte(uint8_t data);

/**
 * @brief  C区环形拓扑多目标点最短路径规划与导航执行函数
 * @details C区环形轨道拓扑结构示意图：
 * 
 *               (y = 2450)
 *      [6] <------------------ [5]
 *       |                       |
 *      [7]                     [4]
 *       |                       |
 *      [8]                     [3]
 *       |                       |
 *      [9]                     [2]
 *       |                       |
 *     [10]                     [1]
 *       |                       |
 *      [11] -----------------> [0]
 *               (y = 0)
 *   (x = -2700)             (x = -1900)
 * 
 *          根据当前节点与待访问目标节点列表，自动对比顺时针/逆时针巡航的总路程，
 *          选取最短路径生成过渡航点与作业动作，并驱动小车完成自动化巡航。
 * @param  start_node_idx 起始节点编号 (0 ~ 11)
 * @param  target_nodes   待访问的目标节点编号数组
 * @param  num_targets    目标节点数量
 * @param  next_mode      完成后跳转的下一个模式
 * @return 0 成功启动，-1 参数错误
 */
int32_t App_RouteC_PlanAndRun(uint8_t start_node_idx, const uint8_t *target_nodes,
                              uint8_t num_targets, AppMode_t next_mode);

#endif
