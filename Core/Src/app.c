#include "app.h"
#include "cmsis_os.h"
#include "arms.h"
#include "tiancan.h"
#include "navigation.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "bujin.h"
#include "voice.h"
#include "pca9685.h"
#include "UpperCP.h"
/* 定义 PI 常量，避免未定义标识符 */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* 获取航线数组的元素个数 */
#define APP_ROUTE_LEN(route) ((uint8_t)(sizeof(route) / sizeof((route)[0])))
/* 方便定义路径点（X_mm, Y_mm, Yaw_rad, has_action）的辅助宏 */
#define WAYPOINT(x, y, yaw, act)    {(x), (y), (yaw), (act)}
#define WAYPOINT_NO_ACT(x, y, yaw)  {(x), (y), (yaw), false}
#define WAYPOINT_ACT(x, y, yaw)     {(x), (y), (yaw), true}

/* 当前系统的全局应用模式 */
volatile AppMode_t g_app_mode = APP_MODE_IDLE;

/* 内部状态变量：路径导航是否运行中，是否收到停止请求 */
static volatile bool s_app_running;
static volatile bool s_stop_requested;
static osThreadId_t s_caizhai_tast_id;

#define APP_EVENT_GRAB_DONE    (1U << 0)

/* 航线 A 的目标路径点序列 */
static const AppWaypoint_t k_route_a[] = {
    WAYPOINT(0.0f, 700.0f, 0.0f, 1),
    WAYPOINT(0.0f, 1200.0f, 0.0f, 1),
    WAYPOINT(0.0f, 1700.0f, 0.0f, 1),
    WAYPOINT(0.0f, 2200.0f, 0.0f, 1),
    WAYPOINT(0.0f, 0.0f, 0.0f, 0),
    WAYPOINT(-1950.0f, 0.0f, 0.0f, false),
};

/* 航线 C 的目标路径点序列 */
static const AppWaypoint_t k_route_c[] = {
    WAYPOINT(-1950.0f, 0.0f, 0.0f, false),
    WAYPOINT(-1950.0f, 500.0f, 0.0f, false),
    WAYPOINT(-1950.0f, 1000.0f, 0.0f, false),
    WAYPOINT(-1950.0f, 1500.0f, 0.0f, false),
    WAYPOINT(-1950.0f, 2000.0f, 0.0f, false),
    WAYPOINT(-1950.0f, 2500.0f, 0.0f, false),
    WAYPOINT(-2700.0f, 2500.0f, -PI, false),
    WAYPOINT(-2700.0f, 2000.0f, -PI, false),
    WAYPOINT(-2700.0f, 1500.0f, -PI, false),
    WAYPOINT(-2700.0f, 1000.0f, -PI, false),
    WAYPOINT(-2700.0f, 500.0f, -PI, false),
    WAYPOINT(-2700.0f, 0.0f, -PI, false),
};

/* 内部静态函数：执行特定的一组航线点，并跳转到指定的下一个模式 */
static void App_RunRoute(const AppWaypoint_t *route, uint8_t route_len,
                         AppMode_t next_mode);
static bool App_WaitGrabDone(void);

/**
 * @brief 初始化应用层状态
 */
void App_Init(void)
{
    g_app_mode = APP_MODE_IDLE;
    s_app_running = false;
    s_stop_requested = false;
}

/**
 * @brief 设置目标应用模式
 */
void App_SetMode(AppMode_t mode)
{
    g_app_mode = mode;
}

/**
 * @brief 查询导航状态
 */
bool App_IsRunning(void)
{
    return s_app_running;
}

/**
 * @brief 接收串口命令，用于启动或停止导航任务
 * @param data 接收到的字符。'a'/'A' 代表启动航线 A，'t'/'T' 代表立即紧急停机
 */
void vofaRxbyte(uint8_t data)
{
    Tiancan_RxByte(data);

    if (data == 'a' || data == 'A') {
        s_app_running = true;
        s_stop_requested = false;
        App_SetMode(APP_MODE_ROUTE_A);
    } else if (data == 't' || data == 'T') {
        s_app_running = false;
        s_stop_requested = true;
        App_SetMode(APP_MODE_IDLE);
    }
    else if (data == 's' || data == 'S') {
        s_app_running = 1;
        s_stop_requested = 0;
        App_SetMode(APP_MODE_TEST);
    }
}
/**
 * @brief 应用主循环/任务中调用的模式执行函数
 *        根据当前所处的 g_app_mode 决定执行哪条航线，或处理停止请求
 */
void App_RunCurrentMode(void)
{
        if (s_stop_requested) {
            s_stop_requested = false;
            Navigation_Stop();
            return;
        }

  
     switch (g_app_mode) {
        case APP_MODE_TEST:
            /* Test mode: currently does nothing, but can be used for debugging or custom tests. */
//            osDelay(500U);
//            PCA9685_Set180AngleSmooth(7U, 20.0f, 100U, 10U);
//             osDelay(1000U);
//         PCA9685_Set180AngleSmooth(7U, 0.0f, 100U, 10U);
//         PCA9685_Set180AngleSmooth(1U, 20.0f, 500U, 10U);
        //     PCA9685_Set180AngleSmooth(7U,0.0f, 100U, 10U);

        //      osDelay(1000U);
//           PCA9685_Set180AngleSmooth(6U,25.0f, 100U, 10U);
//         extend_cm(20.0f);
//        Chassis_SetSpeed(500.0f, 0.0f);
//            Move_down(5.0f);
//          PCA9685_Set180AngleSmooth(7U, 80, 200U, 10U);
//          PCA9685_Set270AngleSmooth(0.0f, 100U, 20U);
//          (void)PCA9685_Set180AngleSmooth(5U, -80.0f, 100U, 10U);
//        ZhuaZi_open();		//爪子张开
  PCA9685_Set180AngleSmooth(7U, -90, 150U, 10U);
        // ZhuaZi_close();
           osDelay(2000U);
//            Chassis_SetSpeed(0.0f, 0.0f);
            App_SetMode(APP_MODE_IDLE);

            break;

        case APP_MODE_IDLE:
            /* Idle mode: ensure the robot is stopped and not executing any navigation tasks. */
           
            break;

        case APP_MODE_ROUTE_A:
            Voice_Num(17);
            vTaskDelay(pdMS_TO_TICKS(100));
            App_RunRoute(k_route_a, APP_ROUTE_LEN(k_route_a), APP_MODE_ROUTE_C);
            break;

        case APP_MODE_ROUTE_B:
            /* Route B is intentionally retained as a no-op placeholder. */
            break;

        case APP_MODE_ROUTE_C:
            /*
             * Preserve the current behavior: route C leaves the app in route C
             * after completion, so the scheduler can enter it again.
             */
            // App_RunRoute(k_route_c, APP_ROUTE_LEN(k_route_c), APP_MODE_IDLE);
            App_RouteC_PlanAndRun(0, (const uint8_t[]){2,4,7,9,10}, 5, APP_MODE_IDLE);
            break;

        default:
            App_SetMode(APP_MODE_IDLE);
            break;
    }
}

/**
 * @brief 顺序执行一个航线数组中的所有点
 * @param route 航点数组指针
 * @param route_len 航点数组长度
 * @param next_mode 执行完成后的下一个应用模式
 */
static void App_RunRoute(const AppWaypoint_t *route, uint8_t route_len,
                         AppMode_t next_mode)
{
    uint8_t i;

    /* 逐个航点遍历，确保系统仍处于运行状态 */
    for (i = 0U; i < route_len && App_IsRunning(); i++) {
        /* 请求导航到当前航点坐标 */
        (void)Navigation_Request(route[i].x_mm, route[i].y_mm, route[i].yaw_rad);

        /* 1. 等待导航模块到达目标点（底盘运动中，舵机保持不动） */
        while (!Navigation_IsIdle() && App_IsRunning()) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        /* 2. 到达目标点且底盘停稳后，只有当该航点配置了 has_action == true 时才执行舵机动作 */
        if (App_IsRunning() && route[i].has_action) {
               Move_Pos(8.0f);
                 osDelay(500U);
            PCA9685_Set180AngleSmooth(7U, 90, 150U, 10U);
             Move_Pos(0.0f);
              
            osDelay(500U);
          
            s_caizhai_tast_id = osThreadGetId();
            (void)osThreadFlagsClear(APP_EVENT_GRAB_DONE);
            UpperCP_SendTask("send");
              osDelay(500U);
            if (!App_WaitGrabDone()) {
                Navigation_Stop();
                return;
            }
           
            //回零
             Move_Pos(8.0f);
                 osDelay(500U);
             PCA9685_Set180AngleSmooth(7U, -90, 150U, 10U);
                  Move_Pos(0.0f);
     
            osDelay(500U);
            (void)osThreadFlagsClear(APP_EVENT_GRAB_DONE);
            UpperCP_SendTask("send");
              osDelay(500U);
            if (!App_WaitGrabDone()) {
                Navigation_Stop();
                return;
            }
        }
    }

    /* 如果中途被停止，直接退出 */
    if (!App_IsRunning()) {
        return;
    }

    /* 如果下一步是空闲模式，重置运行状态并停止底盘 */
    if (next_mode == APP_MODE_IDLE) {
        s_app_running = false;
        Navigation_Stop();
    }

    /* 切换到下一个运行模式 */
    App_SetMode(next_mode);
}

static bool App_WaitGrabDone(void)
{
    uint32_t flags;

    while (App_IsRunning() && !s_stop_requested) {
        flags = osThreadFlagsWait(APP_EVENT_GRAB_DONE, osFlagsWaitAny, 50U);
        if ((flags & APP_EVENT_GRAB_DONE) != 0U) {
            return true;
        }
    }

    return false;
}

void App_NotifyGrabDone(void)
{
    if (s_caizhai_tast_id != NULL) {
        (void)osThreadFlagsSet(s_caizhai_tast_id, APP_EVENT_GRAB_DONE);
    }
}

/**
 * @brief  C区环形拓扑多目标点最短路径规划与导航执行函数
 * @details C区 12 个节点的环形轨道拓扑结构示意图：
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
 *          算法原理：
 *          1. C区拥有 12 个离散顶点 (0~11)，闭合成一个矩形环形轨道赛道。
 *          2. 传入起点 node 编号与待访问目标点列表 `target_nodes`。
 *          3. 评估顺时针 (Clockwise) 与逆时针 (Counter-Clockwise) 遍历完所有目标节点所需的跨越步数。
 *          4. 自动选取总步数最少的绕行方向（路程最短）。
 *          5. 沿途生成航点队列，目标节点置 `has_action = true`（到点停稳后触发舵机作业），
 *             过路节点置 `has_action = false`（只经过不停留/不动舵机）。
 *          6. 调用 `App_RunRoute` 驱动小车高效完成多目标任务。
 * 
 * @param  start_node_idx 起始节点编号 (0 ~ 11)
 * @param  target_nodes   待访问的目标节点编号数组
 * @param  num_targets    目标节点数量
 * @param  next_mode      完成后跳转的下一个模式
 * @return 0 成功启动，-1 参数错误
 */
int32_t App_RouteC_PlanAndRun(uint8_t start_node_idx, const uint8_t *target_nodes,
                              uint8_t num_targets, AppMode_t next_mode)
{
    uint8_t i;
    uint8_t step;
    uint8_t curr;
    uint8_t cw_steps = 0U;
    uint8_t ccw_steps = 0U;
    bool is_target[12] = {false};

    /* 参数有效性校验：节点必须在 0~11 范围内 */
    if (start_node_idx >= 12U || target_nodes == NULL || num_targets == 0U) {
        return -1;
    }

    /* 标记待访问的目标节点索引，便于快速查询 */
    for (i = 0U; i < num_targets; i++) {
        if (target_nodes[i] < 12U) {
            is_target[target_nodes[i]] = true;
        }
    }

    /* --- 步骤 1: 评估【顺时针方向】扫完所有目标点所需的跨越步数 --- */
    curr = start_node_idx;
    for (step = 1U; step <= 12U; step++) {
        curr = (curr + 1U) % 12U; /* 顺时针递增索引，超出 11 取模归零 */
        if (is_target[curr]) {
            cw_steps = step;     /* 更新覆盖全部目标所需的最后步数 */
        }
    }

    /* --- 步骤 2: 评估【逆时针方向】扫完所有目标点所需的跨越步数 --- */
    curr = start_node_idx;
    for (step = 1U; step <= 12U; step++) {
        curr = (curr == 0U) ? 11U : (curr - 1U); /* 逆时针递减索引，小于 0 回到 11 */
        if (is_target[curr]) {
            ccw_steps = step;    /* 更新覆盖全部目标所需的最后步数 */
        }
    }

    /* --- 步骤 3: 比较顺时针与逆时针路径长度，选取最省时间的偏好方向 --- */
    bool choose_cw = (cw_steps <= ccw_steps);
    uint8_t best_steps = choose_cw ? cw_steps : ccw_steps;

    /* 若未命中任何有效目标节点，直接返回 */
    if (best_steps == 0U) {
        return 0;
    }

    /* --- 步骤 4: 提取沿途原始基础节点序列 --- */
    AppWaypoint_t temp_route[12];
    curr = start_node_idx;

    for (i = 0U; i < best_steps; i++) {
        /* 根据决定的最优方向走下一步 */
        if (choose_cw) {
            curr = (curr + 1U) % 12U;
        } else {
            curr = (curr == 0U) ? 11U : (curr - 1U);
        }

        /* 从 C 区基础赛道数据中拷贝坐标与姿态角 */
        temp_route[i] = k_route_c[curr];

        /* 若该点属于目标节点，设置到点后执行舵机作业 (true)；若是中途过路点则不触发 (false) */
        temp_route[i].has_action = is_target[curr];
    }

    /* --- 步骤 5: 优化路径 —— 剔除直线上无动作的冗余中间点，严格保留作业点与四大拐角枢纽点 (0, 5, 6, 11) --- */
    AppWaypoint_t dynamic_route[12];
    uint8_t out_len = 0U;

    for (i = 0U; i < best_steps; i++) {
        /* 计算当前点对应的 C 区节点原始索引编号 (0~11) */
        uint8_t node_idx;
        if (choose_cw) {
            node_idx = (start_node_idx + i + 1U) % 12U;
        } else {
            node_idx = (start_node_idx + 12U - ((i + 1U) % 12U)) % 12U;
        }

        AppWaypoint_t curr_pt = temp_route[i];

        /*
         * 节点保留规则（满足任一条件即保留）：
         * 1. 目标作业点 (has_action == true)；
         * 2. 本次路线的最后一个终点 (i == best_steps - 1)；
         * 3. 赛道四大转弯拐角枢纽节点 (0, 5, 6, 11)，必须保留，绝不斜切撞树！
         */
        bool is_corner_node = (node_idx == 0U || node_idx == 5U || node_idx == 6U || node_idx == 11U);

        if (curr_pt.has_action || (i == best_steps - 1U) || is_corner_node) {
            dynamic_route[out_len++] = curr_pt;
        }
        /* 属于直线上无动作的冗余中间节点（如 1,3,8 等），直接剔除，大直线高速通畅行驶 */
    }

    /* --- 步骤 6: 下发给底层导航，沿赛道外围一路畅通执行 --- */
    App_RunRoute(dynamic_route, out_len, next_mode);

    return 0;
}
