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
    APP_MODE_ROUTE_B,   /**< 航线 B 入口模式：从 C 区底部进入 B 区扫码点 */
    APP_MODE_SCAN_B,    /**< B 区扫码占位模式：当前直接进入 B 区抓取路线 */
    APP_MODE_SCAN_C,    /**< C 区扫码模式：发送 scan 后等待二维码或 10 秒超时 */
    APP_MODE_ROUTE_C,   /**< 航线 C 模式：执行第二阶段的路径导航（例如折返或区域内作业） */
    APP_MODE_BACK       /**< 返回模式：执行返回动作，例如回到起始点 */
} AppMode_t;

#define APP_ACTION_NONE      0x00U
#define APP_ACTION_POSITIVE  0x01U
#define APP_ACTION_NEGATIVE  0x02U

/**
 * @brief 路径点（航点）结构体
 */
typedef struct {
    float x_mm;         /**< 目标点 X 坐标，单位：毫米 */
    float y_mm;         /**< 目标点 Y 坐标，单位：毫米 */
    float yaw_rad;      /**< 目标点朝向角，单位：弧度 */
    bool has_action;    /**< 是否在到达目标点后执行舵机动作 (true/false) */
    uint8_t action_mask;/**< 云台作业方向：APP_ACTION_POSITIVE/NEGATIVE 位组合 */
    uint8_t positive_position; /**< +90 度视野对应的 C 区位置，0 表示普通视觉任务 */
    uint8_t negative_position; /**< -90 度视野对应的 C 区位置，0 表示普通视觉任务 */
} AppWaypoint_t;

/* 全局应用模式变量，由导航任务或串口控制修改 */
extern volatile AppMode_t g_app_mode;

/* 全局抓取使能开关：true 开启抓取（默认），false 则只跑点不抓取 */
extern volatile bool g_enable_grasp_logic;

/* 内部状态标志：路径导航运行中 / 停止请求 */
extern volatile bool s_app_running;
extern volatile bool s_stop_requested;

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
 * @brief 通知当前视觉等待：已收到一条有效 arm 指令。
 * @param command arm 指令值，范围 0~6。
 */
void App_NotifyVisionCommandReceived(uint8_t command);

/**
 * @brief 串口指令接收处理函数，用于解析外部启动/停止指令
 * @param data 接收到的串口数据字节 ('a'/'A' 启动, 't'/'T' 停止)
 */
void vofaRxbyte(uint8_t data);

/**
 * @brief  C区环形拓扑多目标点最短路径规划与导航执行函数
 * @details C区环形轨道拓扑结构示意图：
 * 
 *               (y = 2350)
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
 *   (x = -2600)             (x = -1900)
 * 
 *          从固定入口节点 11 出发，根据水果位置自动对比顺时针/逆时针巡航的总路程，
 *          选取最短路径生成过渡航点与作业动作，并驱动小车完成自动化巡航。
 * @param  fruit_positions 8 个水果位置编号数组，每项范围为 1 ~ 12
 * @param  next_mode      完成后跳转的下一个模式
 * @return 0 成功启动，-1 参数错误
 */
int32_t App_RouteC_PlanAndRun(const uint8_t *fruit_positions,
                              AppMode_t next_mode);

#endif
