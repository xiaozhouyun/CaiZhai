#include "app.h"
#include "main.h"
#include "arms.h"
#include "tiancan.h"
#include "navigation.h"
#include "usart.h"
#include "bujin.h"
#include "voice.h"
#include "pca9685.h"
#include "UpperCP.h"
#include "action_scheduler.h"
#include "cmsis_os.h"
#include "vofa.h"
/* 定义 PI 常量，避免未定义标识符 */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* 获取航线数组的元素个数 */
#define APP_ROUTE_LEN(route) ((uint8_t)(sizeof(route) / sizeof((route)[0])))
#define APP_ROUTE_LIFT_SETTLE_MS      1000U  /* 升至 10cm 后等待升降台实际到位，再转云台 */
#define APP_ROUTE_LOWER_SETTLE_MS     1500U  /* 降至 1cm 后等待机构稳定，再请求视觉抓取 */
/* 方便定义路径点（X_mm, Y_mm, Yaw_rad, has_action）的辅助宏 */
#define WAYPOINT(x, y, yaw, act)    {(x), (y), (yaw), (act)}
#define WAYPOINT_NO_ACT(x, y, yaw)  {(x), (y), (yaw), false}
#define WAYPOINT_ACT(x, y, yaw)     {(x), (y), (yaw), true}

/* 当前系统的全局应用模式 */
volatile AppMode_t g_app_mode = APP_MODE_IDLE;

/* 全局抓取使能开关：true 开启抓取（默认），false 则只跑点不抓取 */
volatile bool g_enable_grasp_logic = 1;

/* 内部状态变量：路径导航是否运行中，是否收到停止请求 */
volatile bool s_app_running;
volatile bool s_stop_requested;
static volatile bool s_grab_done;      /* 视觉动作机完成一次处理后置位 */
static const AppWaypoint_t *s_route;   /* 当前执行路线；A 指向常量，C 指向下方静态副本 */
static AppWaypoint_t s_dynamic_route[12]; /* C 区规划结果，不能使用函数栈数组 */
static uint8_t s_route_len;
static uint8_t s_route_index;
static AppMode_t s_route_next_mode;
static uint32_t s_route_deadline;

/* 路线状态机：每次 Tick 最多下发一个阶段动作，绝不等待导航或视觉结果。 */
typedef enum {
    APP_ROUTE_IDLE,
    /* 当前没有已启动路线；App_RunCurrentMode 会根据 g_app_mode 启动 A 或 C。 */

    APP_ROUTE_WAIT_NAVIGATION,
    /* 已向 Navigation_Request 下发当前航点，等待 Navigation_IsIdle() 到点。 */

    APP_ROUTE_FIRST_WAIT_LIFT,
    /* 作业点第一视野：已抬升到 10cm，等待 3000ms 后向 +90度转云台。 */

    APP_ROUTE_FIRST_WAIT_GIMBAL,
    /* 云台已转至 +90度，等待 1500ms 给舵机完整转动时间。 */

    APP_ROUTE_FIRST_WAIT_LOWER,
    /* 已降至 1cm，等待 3000ms 后向上位机发送第一次 send 任务。 */

    APP_ROUTE_WAIT_GRAB_FIRST,
    /* 第一次 send 已发出；等待 ActionScheduler 调用 App_NotifyGrabDone()。 */

    APP_ROUTE_SECOND_WAIT_LIFT,
    /* 第一次视野完成：已再次抬升到 10cm，等待 3000ms 后转向 -90度。 */

    APP_ROUTE_SECOND_WAIT_GIMBAL,
    /* 云台已转至 -90度，等待 1500ms 给舵机完整转动时间。 */

    APP_ROUTE_SECOND_WAIT_LOWER,
    /* 已降至 1cm，等待 3000ms 后发送第二次 send 任务。 */

    APP_ROUTE_WAIT_GRAB_SECOND
    /* 第二次 send 已发出；完成后航点索引加一并请求下一个航点。 */
} AppRouteState_t;

static AppRouteState_t s_route_state;

static bool App_RouteDelayExpired(void)
{
    /* 使用有符号差值，避免毫秒计数器回绕时误判。 */
    return ((int32_t)(HAL_GetTick() - s_route_deadline) >= 0);
}

static void App_RouteSetDelay(uint32_t delay_ms)
{
    /* 此函数只记录“下一步最早执行时刻”，不会阻塞当前任务。 */
    s_route_deadline = HAL_GetTick() + delay_ms;
}

/**
 * @brief  调试用：打印云台及升降机构底层 TX 队列的健康状态
 * @note   当前处于注释屏蔽状态，以节省串口和 CPU 资源。
 *         排查发送卡死或丢包时可恢复 Vofa_Printf 打印。
 */
static void App_LogLiftTxStatus(void)
{
    Emm_TxStatus_t tx_status;

    Emm_GetTxStatus(&tx_status);
    // Vofa_Printf("[LIFT_WAIT] tx_start=%lu tx_done=%lu busy=%u pending=%u drops=%lu\r\n",
    //             (unsigned long)tx_status.start_count,
    //             (unsigned long)tx_status.complete_count,
    //             (unsigned int)tx_status.busy,
    //             (unsigned int)tx_status.pending,
    //             (unsigned long)Emm_GetTxDropCount());
}

/* 航线 A 的目标路径点序列 */
static const AppWaypoint_t k_route_a[] = {
    WAYPOINT(0.0f, 700.0f, 0.0f, 1),
    WAYPOINT(0.0f, 1200.0f, 0.0f, 1),
    WAYPOINT(0.0f, 1700.0f, 0.0f, 1),
    WAYPOINT(0.0f, 2200.0f, 0.0f, 1),
    WAYPOINT(0.0f, 0.0f, 0.0f, 0),
    WAYPOINT(-1900.0f, 0.0f, 0.0f, false),
};

/* 航线 C 的目标路径点序列 */
static const AppWaypoint_t k_route_c[] = {
    // WAYPOINT(-1900.0f, 0.0f, 0.0f, false),
    WAYPOINT(-1900.0f, 400.0f, 0.0f, 0),
    WAYPOINT(-1900.0f, 900.0f, 0.0f, 0),
    WAYPOINT(-1900.0f, 1400.0f, 0.0f, 0),
    WAYPOINT(-1900.0f, 1900.0f, 0.0f, 0),
    WAYPOINT(-1900.0f, 2400.0f, 0.0f, false),
    WAYPOINT(-2600.0f, 2400.0f, -PI, 0),
    WAYPOINT(-2600.0f, 1900.0f, -PI, 0),
    WAYPOINT(-2600.0f, 1400.0f, -PI, 0),
    WAYPOINT(-2600.0f, 900.0f, -PI, 0),
    WAYPOINT(-2600.0f, 400.0f, -PI, 0),
    WAYPOINT(-2600.0f, 0.0f, -PI, false),
};

/* 内部静态函数：执行特定的一组航线点，并跳转到指定的下一个模式 */
static void App_StartRoute(const AppWaypoint_t *route, uint8_t route_len,
                           AppMode_t next_mode);
static void App_RouteTick(void);

/**
 * @brief 初始化应用层状态
 */
void App_Init(void)
{
    /* 所有路线/抓取状态均从空闲开始，防止上次运行残留的完成标志误触发。 */
    g_app_mode = APP_MODE_IDLE;
    s_app_running = false;
    s_stop_requested = false;
    s_grab_done = false;
    s_route = NULL;
    s_route_state = APP_ROUTE_IDLE;
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
    /* 天蚕协议始终先处理；本函数额外识别本项目的单字符启停命令。 */
    Tiancan_RxByte(data);

    if (data == 'a' || data == 'A') {
        /* A 区路线从 Task07 的下一次 Tick 开始，不在 USART6 中断中执行。 */
        s_app_running = true;
        s_stop_requested = false;
        App_SetMode(APP_MODE_ROUTE_A);
    } else if (data == 't' || data == 'T') {
        /* 这里只置急停请求；真正停止动作由 Task07 串行执行，避免中断内操作外设。 */
        s_app_running = false;
        s_stop_requested = true;
        App_SetMode(APP_MODE_IDLE);
    }
    else if (data == 's' || data == 'S') {
        /* 保留原测试入口：下一次 Task07 Tick 将云台置为测试角度后回到空闲。 */
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
        /* 急停优先：停止路线推进并取消未完成抓取序列。 */
        s_stop_requested = false;
        s_route_state = APP_ROUTE_IDLE;
        s_route = NULL;
        ActionScheduler_Cancel();
        Navigation_Stop();
        return;
    }

    if (s_route_state != APP_ROUTE_IDLE) {
        /* 已启动路线后不重复进入模式分支，只推进当前路线一步。 */
        App_RouteTick();
        return;
    }

    switch (g_app_mode) {
        case APP_MODE_TEST:
            /* 单次测试动作，不使用原先的平滑阻塞接口。 */
            // ActionScheduler_StartGimbalMove(-90.0f, 1500U);
        //    Move_Pos(10.0f);
        //    vTaskDelay(pdMS_TO_TICKS(2000U));
        //        Move_Pos(2.0f);
        //    vTaskDelay(pdMS_TO_TICKS(2000U));
        //         Move_Pos(15.0f);
        //    vTaskDelay(pdMS_TO_TICKS(2000U));
            // Move_Pos(25.0f);
            //   vTaskDelay(pdMS_TO_TICKS(1500U));
                    // 
        Chassis_SetSpeed(0.0f,0.6f);
           App_SetMode(APP_MODE_IDLE);
            break;

        case APP_MODE_IDLE:
            /* Idle mode: ensure the robot is stopped and not executing any navigation tasks. */
           
            break;

        case APP_MODE_ROUTE_A:
            /* 语音提示只在 A 路线刚启动时调用一次；后续 Tick 转入路线状态机。 */
                Move_Pos(25.0f);
              vTaskDelay(pdMS_TO_TICKS(3000U));
            Voice_Num(17);
            App_StartRoute(k_route_a, APP_ROUTE_LEN(k_route_a), APP_MODE_ROUTE_C);
            break;

        case APP_MODE_ROUTE_B:
            /* Route B is intentionally retained as a no-op placeholder. */
            break;

        case APP_MODE_ROUTE_C:
            /* C 区规划函数只负责生成静态路线并启动状态机，不再同步跑完整条路线。 */
            App_RouteC_PlanAndRun(0, (const uint8_t[]){2,4,7,9,10}, 5, APP_MODE_BACK);
            //   App_StartRoute(k_route_c, APP_ROUTE_LEN(k_route_c), APP_MODE_BACK);
            break;
        case APP_MODE_BACK:
            /* 两步返回原点(0,0)：先Y轴归零，再X轴归零，避免斜线碰撞风险 */
            s_dynamic_route[0].x_mm = g_robot_pos.x;
            s_dynamic_route[0].y_mm = 0.0f;
            s_dynamic_route[0].yaw_rad = 0.0f;
            s_dynamic_route[0].has_action = false;
            s_dynamic_route[1].x_mm = 0.0f;
            s_dynamic_route[1].y_mm = 0.0f;
            s_dynamic_route[1].yaw_rad = PI/2.0f;
            s_dynamic_route[1].has_action = false;
            App_StartRoute(s_dynamic_route, 2, APP_MODE_IDLE);
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
static void App_StartRoute(const AppWaypoint_t *route, uint8_t route_len,
                           AppMode_t next_mode)
{
    if (route == NULL || route_len == 0U) {
        return;
    }
    /* 仅记录路线并请求首航点；到点检测交给后续 App_RouteTick。 */
    s_route = route;
    s_route_len = route_len;
    s_route_index = 0U;
    s_route_next_mode = next_mode;
    s_route_state = APP_ROUTE_WAIT_NAVIGATION;
    (void)Navigation_Request(s_route[0].x_mm, s_route[0].y_mm, s_route[0].yaw_rad);
}

static void App_RouteTick(void)
{
    if (!App_IsRunning() || s_route == NULL) {
        /* 外部停止或非法路线指针时，停止导航并退出路线状态机。 */
        s_route_state = APP_ROUTE_IDLE;
        Navigation_Stop();
        return;
    }

    if (s_route_state == APP_ROUTE_WAIT_NAVIGATION) {
        /* 底盘未到点时本函数立即返回，导航任务仍以 10ms 独立运行。 */
        if (!Navigation_IsIdle()) {
            return;
        }
        if (!s_route[s_route_index].has_action || !g_enable_grasp_logic) {
            /* 普通航点或者未开启抓取逻辑时，不需要视觉作业：到点后直接请求下一个航点。 */
            s_route_index++;
        } else {
            /* 作业点固定执行“抬升至10cm→转向→下降→两次视觉任务”。 */
            Move_Pos(25.0f);
            // App_LogLiftTxStatus();
            s_route_state = APP_ROUTE_FIRST_WAIT_LIFT;
            App_RouteSetDelay(APP_ROUTE_LIFT_SETTLE_MS);
            return;
        }
    } else if (s_route_state == APP_ROUTE_FIRST_WAIT_LIFT) {
        /* 
         * 前置拦截（Guard Clause）检查：
         * 1. !App_RouteDelayExpired()：路径规划延时（如到位等待）未结束
         * 2. ActionScheduler_IsGimbalBusy()：云台当前正忙于执行转动或复位动作
         * 只要满足任意一条，即放弃当前周期的执行，等待下轮 Tick。防止指令冲突打断当前动作。
         */
        if (!App_RouteDelayExpired() || ActionScheduler_IsGimbalBusy()) {
            return;
        }
        /* 由 Task07 线性插补到正向视野，不能直接跳到 +90度。 */
        ActionScheduler_StartGimbalMove(90.0f, 1000U);
        s_route_state = APP_ROUTE_FIRST_WAIT_GIMBAL;
        App_RouteSetDelay(400U);
        return;
    } else if (s_route_state == APP_ROUTE_FIRST_WAIT_GIMBAL) {
        if (!App_RouteDelayExpired() || ActionScheduler_IsGimbalBusy()) {
            return;
        }
        Move_Pos(2.0f);
        s_route_state = APP_ROUTE_FIRST_WAIT_LOWER;
        App_RouteSetDelay(APP_ROUTE_LOWER_SETTLE_MS);
        return;
    } else if (s_route_state == APP_ROUTE_FIRST_WAIT_LOWER) {
        if (!App_RouteDelayExpired()) {
            return;
        }
        s_grab_done = false;
        UpperCP_SendTask("send"); /* 请求相机完成正向云台视野内的果实处理 */
        s_route_state = APP_ROUTE_WAIT_GRAB_FIRST;
        return;
    } else if (s_route_state == APP_ROUTE_WAIT_GRAB_FIRST) {
         if (!s_grab_done) {
            return;
        }
        if (!App_RouteDelayExpired() || ActionScheduler_IsGimbalBusy()) {
            return;
        }

        /* 由 Task07 线性插补到反向视野，不能直接跳到 -90度。 */
        ActionScheduler_StartGimbalMove(-90.0f, 1200U);
        s_route_state = APP_ROUTE_SECOND_WAIT_GIMBAL;
        App_RouteSetDelay(400U);
        return;
    } else if (s_route_state == APP_ROUTE_SECOND_WAIT_GIMBAL) {
        if (!App_RouteDelayExpired() || ActionScheduler_IsGimbalBusy()) {
            return;
        }
        Move_Pos(2.0f);
        s_route_state = APP_ROUTE_SECOND_WAIT_LOWER;
        App_RouteSetDelay(APP_ROUTE_LOWER_SETTLE_MS);
        return;
    } else if (s_route_state == APP_ROUTE_SECOND_WAIT_LOWER) {
        if (!App_RouteDelayExpired()) {
            return;
        }
        s_grab_done = false;
        UpperCP_SendTask("send"); /* 请求相机完成反向云台视野内的果实处理 */
        s_route_state = APP_ROUTE_WAIT_GRAB_SECOND;
        return;
    } else if (s_route_state == APP_ROUTE_WAIT_GRAB_SECOND) {
        if (!s_grab_done) {
            return;
        }
        s_route_index++;
        s_route_state = APP_ROUTE_WAIT_NAVIGATION;
    }

    if (s_route_index < s_route_len) {
        /* 本航点已完成，向导航任务请求下一航点；到点结果由下一轮 Tick 检查。 */
        (void)Navigation_Request(s_route[s_route_index].x_mm,
                                 s_route[s_route_index].y_mm,
                                 s_route[s_route_index].yaw_rad);
        return;
    }

    s_route_state = APP_ROUTE_IDLE;
    s_route = NULL;
    if (s_route_next_mode == APP_MODE_IDLE) {
        /* 全部路线结束且不再切换下一段时，明确关闭运行标志并停止底盘。 */
        s_app_running = false;
        Navigation_Stop();
    }
    App_SetMode(s_route_next_mode);
}

void App_NotifyGrabDone(void)
{
    /* 由 ActionScheduler 在动作结束时调用；这里只置位，不能做阻塞操作。 */
    s_grab_done = true;
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
 *   (x = -2700)             (x = -1950)
 * 
 *          算法原理：
 *          1. C区拥有 12 个离散顶点 (0~11)，闭合成一个矩形环形轨道赛道。
 *          2. 传入起点 node 编号与待访问目标点列表 `target_nodes`。
 *          3. 评估顺时针 (Clockwise) 与逆时针 (Counter-Clockwise) 遍历完所有目标节点所需的跨越步数。
 *          4. 自动选取总步数最少的绕行方向（路程最短）。
 *          5. 沿途生成航点队列，目标节点置 `has_action = true`（到点停稳后触发舵机作业），
 *             过路节点置 `has_action = false`（只经过不停留/不动舵机）。
 *          6. 启动非阻塞航线调度器完成多目标任务。
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
            s_dynamic_route[out_len++] = curr_pt;
        }
        /* 属于直线上无动作的冗余中间节点（如 1,3,8 等），直接剔除，大直线高速通畅行驶 */
    }

    /* --- 步骤 6: 下发给底层导航，沿赛道外围一路畅通执行 --- */
    App_StartRoute(s_dynamic_route, out_len, next_mode);

    return 0;
}
