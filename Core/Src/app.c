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
#include <math.h>
/* 定义 PI 常量，避免未定义标识符 */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* 获取航线数组的元素个数 */
#define APP_ROUTE_LEN(route) ((uint8_t)(sizeof(route) / sizeof((route)[0])))
#define APP_ROUTE_LIFT_SETTLE_MS      1800U  /* 升至 25cm 后等待升降台实际到位，再转云台 */
#define APP_ROUTE_LOWER_SETTLE_MS     1500U  /* 降至 1cm 后等待机构稳定，再请求视觉抓取 */
#define APP_QR_SCAN_TIMEOUT_MS       10000U  /* C 区二维码最长等待时间，超时使用默认位置 */
/* 方便定义路径点（X_mm, Y_mm, Yaw_rad, has_action）的辅助宏 */
#define WAYPOINT(x, y, yaw, act)    {(x), (y), (yaw), (act), \
                                     ((act) ? (APP_ACTION_POSITIVE | APP_ACTION_NEGATIVE) : APP_ACTION_NONE)}
#define WAYPOINT_NO_ACT(x, y, yaw)  WAYPOINT((x), (y), (yaw), false)
#define WAYPOINT_ACT(x, y, yaw)     WAYPOINT((x), (y), (yaw), true)
#define WAYPOINT_SIDE(x, y, yaw, action) \
    {(x), (y), (yaw), true, (action)}

/* 当前系统的全局应用模式 */
volatile AppMode_t g_app_mode = APP_MODE_IDLE;

/* 全局抓取使能开关：true 开启抓取（默认），false 则只跑点不抓取 */
volatile bool g_enable_grasp_logic =1;

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
static bool s_qr_scan_started;
static uint32_t s_qr_scan_deadline;

/* 路线状态机：每次 Tick 最多下发一个阶段动作，绝不等待导航或视觉结果。 */
typedef enum {
    APP_ROUTE_IDLE,
    /* 当前没有已启动路线；App_RunCurrentMode 会根据 g_app_mode 启动 A 或 C。 */

    APP_ROUTE_WAIT_NAVIGATION,
    /* 已向 Navigation_Request 下发当前航点，等待 Navigation_IsIdle() 到点。 */

    APP_ROUTE_FIRST_WAIT_LIFT,
    /* 作业点第一视野：已抬升到 25cm，等待到位后向 +90度转云台。 */

    APP_ROUTE_FIRST_WAIT_GIMBAL,
    /* 云台已转至 +90度，等待 1500ms 给舵机完整转动时间。 */

    APP_ROUTE_FIRST_WAIT_LOWER,
    /* 已降至 1cm，等待 3000ms 后向上位机发送第一次 send 任务。 */

    APP_ROUTE_WAIT_GRAB_FIRST,
    /* 第一次 send 已发出；等待 ActionScheduler 调用 App_NotifyGrabDone()。 */

    APP_ROUTE_SECOND_WAIT_LIFT,
    /* 第一次视野完成：已再次抬升到 25cm，等待到位后转向 -90度。 */

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
}

/* 航线 A 的目标路径点序列 */
static const AppWaypoint_t k_route_a[] = {
    WAYPOINT(0.0f, 700.0f, 0.0f, 1),
    WAYPOINT(0.0f, 1200.0f, 0.0f, 1),
    WAYPOINT(0.0f, 1700.0f, 0.0f, 1),
    WAYPOINT(0.0f, 2150.0f, 0.0f, 1),
    WAYPOINT(0.0f, 0.0f, 0.0f, 0),
    WAYPOINT(-2600.0f, 10.0f, PI/2, false),
};

/* B 区沿同一竖直通道向下，左右错位果树按单侧云台动作依次处理。 */
static const AppWaypoint_t k_route_b[] = {
    WAYPOINT_SIDE(-1500.0f, 2150.0f, PI, APP_ACTION_POSITIVE),
    WAYPOINT_SIDE(-1500.0f, 1950.0f, PI, APP_ACTION_NEGATIVE),
    WAYPOINT_SIDE(-1500.0f, 1700.0f, PI, APP_ACTION_POSITIVE),
    WAYPOINT_SIDE(-1500.0f, 1500.0f, PI, APP_ACTION_NEGATIVE),
    WAYPOINT_SIDE(-1500.0f, 1200.0f, PI, APP_ACTION_POSITIVE),
    WAYPOINT_SIDE(-1500.0f, 1000.0f, PI, APP_ACTION_NEGATIVE),
    WAYPOINT_SIDE(-1500.0f,  700.0f, PI, APP_ACTION_POSITIVE),
    WAYPOINT_SIDE(-1500.0f,  500.0f, PI, APP_ACTION_NEGATIVE),
};

/* 航线 C 的目标路径点序列 */
static const AppWaypoint_t k_route_c[] = {
    WAYPOINT(-1900.0f, 10.0f, PI, false),
    WAYPOINT(-1900.0f, 400.0f, PI, 0),
    WAYPOINT(-1900.0f, 900.0f, PI, 0),
    WAYPOINT(-1900.0f, 1400.0f, PI, 0),
    WAYPOINT(-1900.0f, 1900.0f, PI, 0),
    WAYPOINT(-1900.0f, 2350.0f, PI, false),
    WAYPOINT(-2600.0f, 2350.0f, 0, 0),
    WAYPOINT(-2600.0f, 1850.0f, 0, 0),
    WAYPOINT(-2600.0f, 1350.0f, 0, 0),
    WAYPOINT(-2600.0f, 850.0f, 0, 0),
    WAYPOINT(-2600.0f, 350.0f, 0, 0),
    WAYPOINT(-2600.0f, 10.0f, 0, false),
};

/* 内部静态函数：执行特定的一组航线点，并跳转到指定的下一个模式 */
static void App_StartRoute(const AppWaypoint_t *route, uint8_t route_len,
                           AppMode_t next_mode);
static void App_RouteTick(void);
static void App_SendVisionTask(void);

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
    s_qr_scan_started = false;
}

/**
 * @brief 设置目标应用模式
 */
void App_SetMode(AppMode_t mode)
{
    if (mode == APP_MODE_SCAN_C) {
        s_qr_scan_started = false;
    }
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
        // Chassis_SetSpeed(0.0f,0.6f);
      
         ActionScheduler_StartGimbalMove(90.0f, 1000U);
           App_SetMode(APP_MODE_IDLE);
            break;

        case APP_MODE_IDLE:
            /* Idle mode: ensure the robot is stopped and not executing any navigation tasks. */
           
            break;

        case APP_MODE_ROUTE_A:
            /* 语音提示只在 A 路线刚启动时调用一次；后续 Tick 转入路线状态机。 */
            Voice_Num(17);
            App_StartRoute(k_route_a, APP_ROUTE_LEN(k_route_a), APP_MODE_SCAN_C);
            break;

        case APP_MODE_ROUTE_B:
            /* C 区结束后先下到底部，再沿 B 区中线上行到扫码点，禁止斜穿顶部区域。 */
            s_dynamic_route[0] = (AppWaypoint_t){g_robot_pos.x, 10.0f, PI / 2.0f,
                                                  false, APP_ACTION_NONE};
            s_dynamic_route[1] = (AppWaypoint_t){-1500.0f, 10.0f, 0.0f,
                                                  false, APP_ACTION_NONE};
            s_dynamic_route[2] = (AppWaypoint_t){-1500.0f, 2350.0f, PI,
                                                  false, APP_ACTION_NONE};
            App_StartRoute(s_dynamic_route, 3U, APP_MODE_SCAN_B);
            break;

        case APP_MODE_SCAN_B:
            /* 预留：以后在此发送扫码请求并等待完成信号，最长 8 秒。 */
            App_StartRoute(k_route_b, APP_ROUTE_LEN(k_route_b), APP_MODE_BACK);
            break;

        case APP_MODE_SCAN_C:
            if (!s_qr_scan_started) {
                UpperCP_ResetQrResult();
                PCA9685_Set270Angle(60.0f); /* 二维码相机转向正前方 */
                UpperCP_SendTask("scan");
                s_qr_scan_deadline = HAL_GetTick() + APP_QR_SCAN_TIMEOUT_MS;
                s_qr_scan_started = true;
                break;
            }

            if (((CameraFlag != 0U) && (fruits_count == 8U)) ||
                ((int32_t)(HAL_GetTick() - s_qr_scan_deadline) >= 0)) {
                  PCA9685_Set270Angle(30.0f);
                App_SetMode(APP_MODE_ROUTE_C);
            }
            break;

        case APP_MODE_ROUTE_C:
            /* 扫码完成或超时后，使用当前 fruits 数组启动 C 区规划。 */
            App_RouteC_PlanAndRun(fruits, APP_MODE_ROUTE_B);
            break;
        case APP_MODE_BACK:
            /* 两步返回原点(0,0)：先Y轴归零，再X轴归零，避免斜线碰撞风险 */
            s_dynamic_route[0].x_mm = g_robot_pos.x;
            s_dynamic_route[0].y_mm = 10.0f;
            s_dynamic_route[0].yaw_rad = PI / 2.0f;    /* 拐角点姿态设为+X方向(+90°)，到点只需顺势旋转90°指引直行 */
            s_dynamic_route[0].has_action = false;
            s_dynamic_route[0].action_mask = APP_ACTION_NONE;
            s_dynamic_route[1].x_mm = 0.0f;
            s_dynamic_route[1].y_mm = 0.0f;
            s_dynamic_route[1].yaw_rad = PI / 2.0f;    /* 到达起点原点后保持+X方向，不再恢复初始朝向 */
            s_dynamic_route[1].has_action = false;
            s_dynamic_route[1].action_mask = APP_ACTION_NONE;
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
        if (!s_route[s_route_index].has_action ||
            s_route[s_route_index].action_mask == APP_ACTION_NONE ||
            !g_enable_grasp_logic) {
            /* 普通航点或者未开启抓取逻辑时，不需要视觉作业：到点后直接请求下一个航点。 */
            s_route_index++;
        } else {
            /* 作业点先抬升到安全高度，再按 action_mask 处理目标侧。 */
            if (now_pos < 26.9f || now_pos > 27.1f) {
                Move_Pos(27.0f);
                s_route_state = APP_ROUTE_FIRST_WAIT_LIFT;
                App_RouteSetDelay(APP_ROUTE_LIFT_SETTLE_MS);
            } else {
                s_route_state = APP_ROUTE_FIRST_WAIT_LIFT;
                App_RouteSetDelay(0U);
            }
            // App_LogLiftTxStatus();
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
        if ((s_route[s_route_index].action_mask & APP_ACTION_POSITIVE) != 0U) {
            /* 当前点需要正向视野：先处理 +90 度侧。 */
            ActionScheduler_StartGimbalMove(90.0f, 1000U);
            s_route_state = APP_ROUTE_FIRST_WAIT_GIMBAL;
        } else {
            /* 当前点只有反向目标：跳过正向视野，直接转到 -90 度侧。 */
            ActionScheduler_StartGimbalMove(-90.0f, 1200U);
            s_route_state = APP_ROUTE_SECOND_WAIT_GIMBAL;
        }
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

        if ((s_route[s_route_index].action_mask & APP_ACTION_NEGATIVE) == 0U) {
            /* 当前点只有 +90 度目标，第一视野完成后直接进入下一航点。 */
            s_route_index++;
            s_route_state = APP_ROUTE_WAIT_NAVIGATION;
        } else {
            /* 由 Task07 线性插补到反向视野，不能直接跳到 -90度。
             * 但如果之前跳过逻辑已经把云台转到了 -90°，直接下降升降台即可。 */
            if (PCA9685_Get180Angle(7U) < -75.0f) {
                Move_Pos(2.0f);
                s_route_state = APP_ROUTE_SECOND_WAIT_LOWER;
                App_RouteSetDelay(APP_ROUTE_LOWER_SETTLE_MS);
            } else {
                ActionScheduler_StartGimbalMove(-90.0f, 1200U);
                s_route_state = APP_ROUTE_SECOND_WAIT_GIMBAL;
                App_RouteSetDelay(400U);
            }
            return;
        }

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
        /* 仅在 A 区最后一个抓果点完成、向 (0,0) 起点倒车时(索引为 4)开启自动倒车；其他航点保持关闭正向前进 */
        if (s_route == k_route_a && s_route_index == 4U) {
            g_enable_auto_reverse = true;
        } else {
            g_enable_auto_reverse = false;
        }

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
 * @brief  向 K230/上位机发送一次视觉处理请求
 */
static void App_SendVisionTask(void)
{
    /* 请求 K230/上位机处理当前云台视野内的果实，并回传 arm:0~6。 */
    UpperCP_SendTask("send");
}

/**
 * @brief  C区环形拓扑多目标点最短路径规划与导航执行函数
 * @details C区 12 个节点的环形轨道拓扑结构示意图：
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
 *          算法原理：
 *          1. C区拥有 12 个离散顶点 (0~11)，闭合成一个矩形环形轨道赛道。
 *          2. 把二维码位置 1~12 映射为环路节点和 +90/-90 度云台动作位。
 *          3. 评估顺时针 (Clockwise) 与逆时针 (Counter-Clockwise) 覆盖全部目标的实际毫米路程。
 *          4. 自动选取总路程较短的绕行方向；路程相同时选择顺时针。
 *          5. 沿途生成航点队列，同一底盘停车点的两侧动作合并到 action_mask。
 *          6. 启动非阻塞航线调度器完成多目标任务。
 * 
 * @param  fruit_positions 8 个水果位置编号数组，每项范围为 1 ~ 12
 * @param  next_mode      完成后跳转的下一个模式
 * @return 0 成功启动，-1 参数错误
 */
int32_t App_RouteC_PlanAndRun(const uint8_t *fruit_positions,
                              AppMode_t next_mode)
{
    const uint8_t start_node_idx = 11U;
    uint8_t i;
    uint8_t step;
    uint8_t curr;
    uint8_t next;
    uint8_t node_idx;
    uint8_t cw_steps = 0U;
    uint8_t ccw_steps = 0U;
    uint8_t best_steps;
    uint8_t out_len = 0U;
    uint8_t target_actions[12] = {APP_ACTION_NONE};
    float traveled;
    float cw_distance = 0.0f;
    float ccw_distance = 0.0f;
    bool choose_cw;
    AppWaypoint_t temp_route[12];

    if (fruit_positions == NULL) {
        return -1;
    }

    /* 二维码位置映射为底盘停车节点，并合并同一点的左右云台动作。 */
    for (i = 0U; i < 8U; i++) {
        uint8_t position = fruit_positions[i];
        uint8_t action;

        if (position >= 1U && position <= 4U) {
            node_idx = (uint8_t)(5U - position);
            action = APP_ACTION_POSITIVE;
        } else if (position >= 5U && position <= 8U) {
            node_idx = (uint8_t)(9U - position);
            action = APP_ACTION_NEGATIVE;
        } else if (position >= 9U && position <= 12U) {
            node_idx = (uint8_t)(position - 2U);
            action = APP_ACTION_POSITIVE;
        } else {
            return -1;
        }

        target_actions[node_idx] |= action;
    }

    /* 顺时针累计相邻节点的实际毫米距离，并记录覆盖最后一个目标时的路程。 */
    curr = start_node_idx;
    traveled = 0.0f;
    for (step = 1U; step <= 12U; step++) {
        next = (uint8_t)((curr + 1U) % 12U);
        traveled += fabsf(k_route_c[next].x_mm - k_route_c[curr].x_mm) +
                    fabsf(k_route_c[next].y_mm - k_route_c[curr].y_mm);
        curr = next;
        if (target_actions[curr] != APP_ACTION_NONE) {
            cw_steps = step;
            cw_distance = traveled;
        }
    }

    /* 逆时针执行相同计算，不能用节点个数代替实际路程。 */
    curr = start_node_idx;
    traveled = 0.0f;
    for (step = 1U; step <= 12U; step++) {
        next = (curr == 0U) ? 11U : (uint8_t)(curr - 1U);
        traveled += fabsf(k_route_c[next].x_mm - k_route_c[curr].x_mm) +
                    fabsf(k_route_c[next].y_mm - k_route_c[curr].y_mm);
        curr = next;
        if (target_actions[curr] != APP_ACTION_NONE) {
            ccw_steps = step;
            ccw_distance = traveled;
        }
    }

    if (cw_steps == 0U || ccw_steps == 0U) {
        return -1;
    }

    choose_cw = (cw_distance <= ccw_distance);
    best_steps = choose_cw ? cw_steps : ccw_steps;
    curr = start_node_idx;

    /* 提取选定方向上的基础节点，并附加该停车点所需的云台方向。 */
    for (i = 0U; i < best_steps; i++) {
        if (choose_cw) {
            curr = (uint8_t)((curr + 1U) % 12U);
        } else {
            curr = (curr == 0U) ? 11U : (uint8_t)(curr - 1U);
        }

        temp_route[i] = k_route_c[curr];
        temp_route[i].action_mask = target_actions[curr];
        temp_route[i].has_action = (target_actions[curr] != APP_ACTION_NONE);
    }

    /* 剔除直线无动作点；目标、终点和四个防斜切拐角必须保留。 */
    for (i = 0U; i < best_steps; i++) {
        if (choose_cw) {
            node_idx = (uint8_t)((start_node_idx + i + 1U) % 12U);
        } else {
            node_idx = (uint8_t)((start_node_idx + 12U - ((i + 1U) % 12U)) % 12U);
        }

        if (temp_route[i].has_action || (i == best_steps - 1U) ||
            node_idx == 0U || node_idx == 5U || node_idx == 6U || node_idx == 11U) {
            if (out_len >= APP_ROUTE_LEN(s_dynamic_route)) {
                return -1;
            }
            s_dynamic_route[out_len++] = temp_route[i];
        }
    }

    App_StartRoute(s_dynamic_route, out_len, next_mode);

    return 0;
}
