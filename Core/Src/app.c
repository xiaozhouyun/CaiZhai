#include "app.h"
#include "main.h"
#include "arms.h"
#include "tiancan.h"
#include "navigation.h"
#include "hwt101_hal.h"
#include "usart.h"
#include "bujin.h"
#include "voice.h"
#include "pca9685.h"
#include "UpperCP.h"
#include "action_scheduler.h"
#include "tof200f.h"
#include "cmsis_os.h"
#include "vofa.h"
#include <math.h>
#include <stdio.h>
/* 定义 PI 常量，避免未定义标识符 */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* 获取航线数组的元素个数 */
#define APP_ROUTE_LEN(route) ((uint8_t)(sizeof(route) / sizeof((route)[0])))
#define APP_ROUTE_LIFT_SETTLE_MS      2200U  /* 升至 25cm 后等待升降台实际到位，再转云台 */
#define APP_ROUTE_LOWER_SETTLE_MS     1500U  /* 降至 1cm 后等待机构稳定，再请求视觉抓取 */
#define APP_ROUTE_POUR_DELAY_MS       5000U  /* C 区到达倒料点后，等待上位机执行 pour */
#define APP_QR_SCAN_TIMEOUT_MS       30000U  /* C 区二维码最长等待时间，超时使用默认位置 */
#define APP_QR_VOICE_INTERVAL_MS      1500U  /* 相邻二维码位置语音的播放间隔 */
#define APP_ROUTE_C_MAX_WAYPOINTS       24U  /* 8 个目标按 QR 顺序运行时所需的目标点和环路拐角上限 */
#define APP_VISION_NO_RX_SEARCH_DELAY_MS 2000U /* A/C 区 send 后无上位机消息时，延时后启动摄像头搜索 */
#define APP_VISION_CAMERA_SWEEP_DEG   10.0f  /* A/C 区等待视觉响应时，摄像头相对基准上摆角度 */
#define APP_VISION_CAMERA_SWEEP_STEP_DEG 1.0f /* 摄像头搜索每次相对基准角的步进角度 */
#define APP_VISION_CAMERA_SWEEP_STEP_MS  500U /* 摄像头搜索每个角度停留时间 */
#define APP_QR_CAMERA_SCAN_START_DEG  60.0f  /* 二维码相机第一档俯仰角 */
#define APP_QR_CAMERA_SCAN_MIDDLE_DEG 70.0f  /* 二维码相机第二档俯仰角 */
#define APP_QR_CAMERA_SCAN_END_DEG    80.0f  /* 二维码相机第三档俯仰角 */
#define APP_QR_GIMBAL_LEFT_DEG       -20.0f  /* 二维码搜索左侧最大角度 */
#define APP_QR_GIMBAL_RIGHT_DEG       20.0f  /* 二维码搜索右侧最大角度 */
#define APP_QR_GIMBAL_STEP_DEG         5.0f  /* 二维码搜索水平云台每次步进 */
#define APP_QR_SCAN_ANGLE_DWELL_MS   1500U    /* 二维码扫描舵机每个角度停留时间，非阻塞等待 */
#define APP_QR_SCAN_STEP_MS         (APP_QR_SCAN_ANGLE_DWELL_MS * 3U)
#define APP_C_TOF_TARGET_MM          200.0f   /* 车尾 TOF 到 C 区底部挡板的目标距离，单位：mm */
#define APP_C_TOF_TOLERANCE_MM        10.0f   /* 校准允许误差：190~210mm 均视为进入目标范围 */
#define APP_C_TOF_SPEED_MM_S          30.0f   /* C 区校准时前后移动的低速线速度，单位：mm/s */
#define APP_C_TOF_TIMEOUT_MS        10000U     /* 整个校准流程最长时间，超时停车并中止路线 */
#define APP_C_TOF_FRAME_STALE_MS     300U     /* 持续移动时超过 300ms 无新帧，立即停车等待数据 */
#define APP_C_TOF_STABLE_FRAMES        2U     /* 连续两帧达标后才置零，避免单帧抖动误触发 */
#define APP_B_TOF_SPEED_MM_S          50.0f   /* B 区校准速度保持原值，避免 C 区降速影响 B 区 */
#define APP_B_TOF_Y_TARGET_MM         200.0f   /* 车头朝 0° 时，车尾 TOF 到后方挡板的目标距离；达标后将 Y 标定为 0 */
#define APP_B_TOF_X_TARGET_MM        1300.0f   /* 车头朝 -90° 时，车尾 TOF 到侧后方基准面的目标距离；达标后将 X 标定为 -950 */
#define APP_ROUTE_A_REVERSE_HEADING_BIAS_RAD (0.6f * PI / 180.0f) /* A 区末段长距离倒车向 +90° 方向补偿 0.6° */
#define APP_QR_SCAN_LEFT              0U     /* 扫码云台阶段：从中位左转到 -20 度 */
#define APP_QR_SCAN_CENTER_FROM_LEFT  1U     /* 扫码云台阶段：从左侧回中 */
#define APP_QR_SCAN_RIGHT             2U     /* 扫码云台阶段：从中位右转到 +20 度 */
#define APP_QR_SCAN_CENTER_FROM_RIGHT 3U     /* 扫码云台阶段：从右侧回中 */
/* 方便定义路径点（X_mm, Y_mm, Yaw_rad, has_action）的辅助宏 */
#define WAYPOINT(x, y, yaw, act)    {(x), (y), (yaw), (act), \
                                     ((act) ? (APP_ACTION_POSITIVE | APP_ACTION_NEGATIVE) : APP_ACTION_NONE), \
                                     0U, 0U}
#define WAYPOINT_NO_ACT(x, y, yaw)  WAYPOINT((x), (y), (yaw), false)
#define WAYPOINT_ACT(x, y, yaw)     WAYPOINT((x), (y), (yaw), true)
#define WAYPOINT_SIDE(x, y, yaw, action) \
    {(x), (y), (yaw), true, (action), 0U, 0U}

/* 当前系统的全局应用模式 */
volatile AppMode_t g_app_mode = APP_MODE_IDLE;

/* 全局抓取使能开关：true 开启抓取（默认），false 则只跑点不抓取 */
volatile bool g_enable_grasp_logic =1;

/* 内部状态变量：路径导航、视觉动作、二维码扫描和倒料流程的运行现场 */
volatile bool s_app_running;           /* 应用路线运行标志：true 表示当前有路线任务在执行，停止或路线结束后清零 */
volatile bool s_stop_requested;        /* 外部停止请求标志：按键/指令请求停车后置位，App_RunCurrentMode() 统一执行停止并清零 */
static volatile bool s_grab_done;      /* 视觉抓取完成标志：ActionScheduler 完成一次 send/arm 处理后置位，路线状态机等待它继续下一步 */
static const AppWaypoint_t *s_route;   /* 当前执行路线指针：A/B 指向常量路线，C/返程可指向 s_dynamic_route 动态路线副本 */
static AppWaypoint_t s_dynamic_route[APP_ROUTE_C_MAX_WAYPOINTS]; /* C 区二维码顺序或返程临时路线缓存，避免使用函数栈数组 */
static uint8_t s_route_len;            /* 当前路线的航点总数，用于判断 s_route_index 是否已经跑完 */
static uint8_t s_route_index;          /* 当前正在执行的航点下标，到点并完成附加动作后递增 */
static AppMode_t s_route_next_mode;    /* 当前路线完成后要切换到的下一个应用模式，通常用于 A/B/C/返程衔接 */
static uint32_t s_route_deadline;      /* 非阻塞等待截止时刻，升降台、云台、倒料等延时状态共用该时间戳 */
static bool s_qr_scan_started;         /* C 区二维码扫描是否已启动，防止在等待二维码期间重复发送扫描请求 */
static uint32_t s_qr_scan_deadline;    /* C 区二维码扫描超时时刻，超过后使用默认路线继续执行 */
static uint8_t s_qr_voice_index;       /* 二维码结果语音播报下标，按 fruits[] 顺序逐个播报位置编号 */
static uint32_t s_qr_voice_deadline;   /* 下一次二维码位置语音允许播放的时刻，用于控制播报间隔 */
static bool s_route_pour_sent;         /* C 区倒料点 pour 指令发送标志，确保同一个倒料等待阶段只发送一次 pour */
static uint8_t s_qr_scan_phase;        /* 二维码水平搜索阶段：左转、左回中、右转、右回中循环 */
static float s_qr_gimbal_angle;        /* 二维码扫描当前水平云台角，控制 PCA9685 通道 7 */
static uint32_t s_qr_scan_step_start_tick; /* 当前二维码扫描停留阶段的起始时刻，用于俯仰慢速扫动 */
static uint32_t s_qr_scan_step_deadline; /* 下一次二维码扫描舵机步进允许执行的时刻 */
static bool s_vision_camera_search_active; /* A/C 区 send 后无上位机消息时，是否启用摄像头上下搜索 */
static uint32_t s_vision_send_tick;     /* 最近一次 A/C 区 send 下发时刻 */
static uint32_t s_vision_send_rx_count; /* send 下发时记录的上位机 UART5 接收字节计数 */
static float s_vision_camera_base_angle; /* send 下发时摄像头俯仰基准角 */
static uint8_t s_route_c_start_node_idx; /* 本次 C 区路线入口节点：0=左下入口，11=右下入口 */
static bool s_c_tof_cal_started;         /* 是否已建立本轮 TOF 校准的时间和帧序号基准 */
static uint8_t s_c_tof_stable_frames;    /* 连续落入 190~210mm 范围的有效帧计数 */
static uint32_t s_c_tof_last_seq;        /* 校准状态机最近一次处理的 TofFrameSeq */
static uint32_t s_c_tof_start_tick;      /* 本轮校准起始时刻，用于 5s 总超时保护 */
static uint32_t s_c_tof_last_frame_tick; /* 最近一次有效新帧时刻，用于运动中的断帧停车保护 */

typedef enum {
    APP_B_CAL_ROTATE_Y_START,    /* 下发原地转向 0° 的导航请求 */
    APP_B_CAL_WAIT_YAW_0,       /* 等待导航完成 0° 转向，确保车尾 TOF 正对 Y 轴基准面 */
    APP_B_CAL_ADJUST_Y,         /* 闭环调整到 200mm，达标后把 Y 坐标置 0 */
    APP_B_CAL_ROTATE_X_START,   /* 下发原地转向 -90° 的导航请求 */
    APP_B_CAL_WAIT_YAW_NEG_90,  /* 等待导航完成 -90° 转向，确保车尾 TOF 正对 X 轴基准面 */
    APP_B_CAL_ADJUST_X          /* 闭环调整到 1250mm，达标后把 X 坐标置 -950 */
} AppBCalibrationState_t;

typedef enum {
    APP_TOF_CAL_IN_PROGRESS,    /* 未达标或正在等待新 TOF 帧，下一个 Tick 继续 */
    APP_TOF_CAL_DONE,           /* 连续稳定帧均落入容差范围，允许写入坐标基准 */
    APP_TOF_CAL_FAILED          /* 校准超时，必须停车退出，禁止使用未校准坐标继续路线 */
} AppTofCalibrationResult_t;

static AppBCalibrationState_t s_b_cal_state; /* B 区入口双轴校准当前阶段 */
static bool s_b_tof_cal_started;             /* 当前轴是否已建立超时与新帧基准 */
static uint8_t s_b_tof_stable_frames;        /* 当前轴连续落入目标容差的 TOF 帧数 */
static uint32_t s_b_tof_last_seq;            /* 已处理的最新 TofFrameSeq，防止重复使用旧数据 */
static uint32_t s_b_tof_start_tick;          /* 当前轴校准起始时刻，用于总超时保护 */
static uint32_t s_b_tof_last_frame_tick;     /* 最近有效新帧时刻，运动中断帧时用于及时停车 */

/* 路线状态机：每次 Tick 最多下发一个阶段动作，绝不等待导航或视觉结果。 */
typedef enum {
    APP_ROUTE_IDLE,
    /* 当前没有已启动路线；App_RunCurrentMode 会根据 g_app_mode 启动 A 或 C。 */

    APP_ROUTE_WAIT_NAVIGATION,
    /* 已向 Navigation_Request 下发当前航点，等待 Navigation_IsIdle() 到点。 */

    APP_ROUTE_WAIT_POUR,
    /* C 区倒料点已发送 pour，等待 2 秒后继续下一航点。 */

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
    WAYPOINT(0.0f, 0.0f, PI/2, 0),
    WAYPOINT(-2600.0f, 0.0f, PI/2, false),
};

/* B 区沿同一竖直通道向下，左右错位果树按单侧云台动作依次处理。 */
static const AppWaypoint_t k_route_b[] = {
    WAYPOINT_SIDE(-950.0f, 2150.0f, PI, APP_ACTION_POSITIVE),
    WAYPOINT_SIDE(-950.0f, 1950.0f, PI, APP_ACTION_NEGATIVE),
    WAYPOINT_SIDE(-950.0f, 1700.0f, PI, APP_ACTION_POSITIVE),
    WAYPOINT_SIDE(-950.0f, 1500.0f, PI, APP_ACTION_NEGATIVE),
    WAYPOINT_SIDE(-950.0f, 1200.0f, PI, APP_ACTION_POSITIVE),
    WAYPOINT_SIDE(-950.0f, 1000.0f, PI, APP_ACTION_NEGATIVE),
    WAYPOINT_SIDE(-950.0f,  700.0f, PI, APP_ACTION_POSITIVE),
    WAYPOINT_SIDE(-950.0f,  500.0f, PI, APP_ACTION_NEGATIVE),
};

/* 航线 C 的目标路径点序列 */
static const AppWaypoint_t k_route_c[] = {
    WAYPOINT(-1965.0f, 0.0f, 0, false),//左起点
    WAYPOINT(-1965.0f, 370.0f, 0, 0),
    WAYPOINT(-1965.0f, 860.0f, 0, 0),
    WAYPOINT(-1965.0f, 1360.0f, 0, 0),
    WAYPOINT(-1965.0f, 1870.0f, 0, 0),
    WAYPOINT(-1965.0f, 2300.0f, 0, 0),//左拐点
    WAYPOINT(-2615.0f, 2300.0f, 0, 0),//右拐点
    WAYPOINT(-2615.0f, 1850.0f, 0, 0),
    WAYPOINT(-2615.0f, 1350.0f, 0, 0),
    WAYPOINT(-2615.0f, 860.0f, 0, 0),
    WAYPOINT(-2615.0f, 370.0f, 0, 0),
    WAYPOINT(-2615.0f, 0.0f, 0, false),//右起点
};

static float App_CurrentMoveHeadingBiasRad(void)
{
    /* A 区第 6 个航点对应从 (0,0) 倒车到 C 区扫码点的长直线。 */
    if (s_route == k_route_a && s_route_index == 5U) {
        return APP_ROUTE_A_REVERSE_HEADING_BIAS_RAD;
    }
    return 0.0f;
}

/* 内部静态函数：执行特定的一组航线点，并跳转到指定的下一个模式 */
static void App_StartRoute(const AppWaypoint_t *route, uint8_t route_len,
                           AppMode_t next_mode);
static void App_RouteTick(void);
static void App_SendVisionTask(void);
static void App_VisionCameraSearchTick(void);
static void App_VisionCameraSearchReset(void);
static void App_QrScanSweepTick(void);
static float App_RouteC_GetShortestRingDistance(uint8_t from_node, uint8_t to_node);
static uint8_t App_RouteC_SelectEntryNode(uint8_t first_position);
static void App_RouteC_TofCalibrationTick(void);
static void App_RouteB_TofCalibrationTick(void);
static AppTofCalibrationResult_t App_RouteB_AdjustTof(float target_mm);

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
    s_qr_voice_index = 0U;
    s_route_pour_sent = false;
    s_qr_scan_phase = APP_QR_SCAN_LEFT;
    s_qr_gimbal_angle = 0.0f;
    s_qr_scan_step_start_tick = 0U;
    s_qr_scan_step_deadline = 0U;
    s_route_c_start_node_idx = 11U;
    s_c_tof_cal_started = false;
    s_c_tof_stable_frames = 0U;
    s_c_tof_last_seq = 0U;
    s_c_tof_start_tick = 0U;
    s_c_tof_last_frame_tick = 0U;
    s_b_cal_state = APP_B_CAL_ROTATE_Y_START;
    s_b_tof_cal_started = false;
    s_b_tof_stable_frames = 0U;
    s_b_tof_last_seq = 0U;
    s_b_tof_start_tick = 0U;
    s_b_tof_last_frame_tick = 0U;
    App_VisionCameraSearchReset();
}

/**
 * @brief 设置目标应用模式
 */
void App_SetMode(AppMode_t mode)
{
    if (mode == APP_MODE_SCAN_C) {
        s_qr_scan_started = false;
        s_qr_voice_index = 0U;
        s_qr_scan_phase = APP_QR_SCAN_LEFT;
        s_qr_gimbal_angle = 0.0f;
        s_qr_scan_step_start_tick = 0U;
        s_qr_scan_step_deadline = 0U;
    }
    if (mode == APP_MODE_CALIBRATE_C) {
        s_c_tof_cal_started = false;
        s_c_tof_stable_frames = 0U;
    }
    if (mode == APP_MODE_CALIBRATE_B) {
        s_b_cal_state = APP_B_CAL_ROTATE_Y_START;
        s_b_tof_cal_started = false;
        s_b_tof_stable_frames = 0U;
    }
    if (mode != APP_MODE_ROUTE_A && mode != APP_MODE_ROUTE_C) {
        App_VisionCameraSearchReset();
    }
    g_app_mode = mode;
}

static void App_VisionCameraSearchReset(void)
{
    s_vision_camera_search_active = false;
    s_vision_send_tick = 0U;
    s_vision_send_rx_count = 0U;
    s_vision_camera_base_angle = 0.0f;
}

static void App_SendVisionTask(void)
{
    s_grab_done = false;
    UpperCP_SendTask("send");

    if (g_app_mode == APP_MODE_ROUTE_A || g_app_mode == APP_MODE_ROUTE_C) {
        s_vision_camera_search_active = true;
        s_vision_send_tick = HAL_GetTick();
        s_vision_send_rx_count = UpperCP_GetRxCount();
        s_vision_camera_base_angle = PCA9685_Get270Angle();
    } else {
        App_VisionCameraSearchReset();
    }
}

static void App_VisionCameraSearchTick(void)
{
    uint32_t now;
    uint32_t elapsed;
    uint32_t phase;
    uint32_t max_step = (uint32_t)(APP_VISION_CAMERA_SWEEP_DEG / APP_VISION_CAMERA_SWEEP_STEP_DEG);
    uint32_t cycle_steps = max_step * 2U;
    float offset_deg;

    if (!s_vision_camera_search_active) {
        return;
    }

    if (UpperCP_GetRxCount() != s_vision_send_rx_count) {
        s_vision_camera_search_active = false;
        return;
    }

    now = HAL_GetTick();
    if ((int32_t)(now - (s_vision_send_tick + APP_VISION_NO_RX_SEARCH_DELAY_MS)) < 0) {
        return;
    }

    elapsed = now - s_vision_send_tick - APP_VISION_NO_RX_SEARCH_DELAY_MS;
    phase = (cycle_steps == 0U) ? 0U : ((elapsed / APP_VISION_CAMERA_SWEEP_STEP_MS) % cycle_steps);

    if (phase <= max_step) {
        offset_deg = APP_VISION_CAMERA_SWEEP_STEP_DEG * (float)phase;
    } else {
        offset_deg = APP_VISION_CAMERA_SWEEP_STEP_DEG * (float)(cycle_steps - phase);
    }

    (void)PCA9685_Set270Angle(s_vision_camera_base_angle + offset_deg);
}

static void App_QrScanSweepTick(void)
{
    uint32_t now = HAL_GetTick();

    if ((int32_t)(now - s_qr_scan_step_deadline) < 0) {
        uint32_t elapsed = now - s_qr_scan_step_start_tick;

        if (elapsed < APP_QR_SCAN_ANGLE_DWELL_MS) {
            (void)PCA9685_Set270Angle(APP_QR_CAMERA_SCAN_START_DEG);
        } else if (elapsed < (APP_QR_SCAN_ANGLE_DWELL_MS * 2U)) {
            (void)PCA9685_Set270Angle(APP_QR_CAMERA_SCAN_MIDDLE_DEG);
        } else {
            (void)PCA9685_Set270Angle(APP_QR_CAMERA_SCAN_END_DEG);
        }
        return;
    }

    s_qr_scan_step_start_tick = now;
    s_qr_scan_step_deadline = now + APP_QR_SCAN_STEP_MS;
    (void)PCA9685_Set270Angle(APP_QR_CAMERA_SCAN_START_DEG);

    switch (s_qr_scan_phase) {
    case APP_QR_SCAN_LEFT:
        s_qr_gimbal_angle -= APP_QR_GIMBAL_STEP_DEG;
        if (s_qr_gimbal_angle <= APP_QR_GIMBAL_LEFT_DEG) {
            s_qr_gimbal_angle = APP_QR_GIMBAL_LEFT_DEG;
            s_qr_scan_phase = APP_QR_SCAN_CENTER_FROM_LEFT;
        }
        break;

    case APP_QR_SCAN_CENTER_FROM_LEFT:
        s_qr_gimbal_angle += APP_QR_GIMBAL_STEP_DEG;
        if (s_qr_gimbal_angle >= 0.0f) {
            s_qr_gimbal_angle = 0.0f;
            s_qr_scan_phase = APP_QR_SCAN_RIGHT;
        }
        break;

    case APP_QR_SCAN_RIGHT:
        s_qr_gimbal_angle += APP_QR_GIMBAL_STEP_DEG;
        if (s_qr_gimbal_angle >= APP_QR_GIMBAL_RIGHT_DEG) {
            s_qr_gimbal_angle = APP_QR_GIMBAL_RIGHT_DEG;
            s_qr_scan_phase = APP_QR_SCAN_CENTER_FROM_RIGHT;
        }
        break;

    default:
        s_qr_gimbal_angle -= APP_QR_GIMBAL_STEP_DEG;
        if (s_qr_gimbal_angle <= 0.0f) {
            s_qr_gimbal_angle = 0.0f;
            s_qr_scan_phase = APP_QR_SCAN_LEFT;
        }
        break;
    }

    (void)PCA9685_Set180Angle(7U, s_qr_gimbal_angle);
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
        // App_SetMode(APP_MODE_TEST);
         App_SetMode(APP_MODE_SCAN_C);
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
        //  vTaskDelay(pdMS_TO_TICKS(2000U));
       Chassis_SetSpeed(0.0f,0.9f);
        //  ActionScheduler_StartGimbalMove(90.0f, 1000U);
        // Move_down(5.0f);
        // ZhuaZi_open();
        // Move_Pos(27.0f);
        vTaskDelay(pdMS_TO_TICKS(5000U));
               Chassis_SetSpeed(0.0f,0.0f);
        //   Move_Pos(2.0f);
        //   ZhuaZi_close();
           App_SetMode(APP_MODE_IDLE);
            break;

        case APP_MODE_IDLE:
            /* Idle mode: ensure the robot is stopped and not executing any navigation tasks. */
           
            break;

        case APP_MODE_ROUTE_A:
            /* 语音提示只在 A 路线刚启动时调用一次；后续 Tick 转入路线状态机。 */
            Navigation_Reset(NAV_START_CENTER_X_MM, NAV_START_CENTER_Y_MM, g_hwt101_yaw);
            Voice_Num(17);
            App_StartRoute(k_route_a, APP_ROUTE_LEN(k_route_a), APP_MODE_SCAN_C);
            break;

        case APP_MODE_ROUTE_B:
            /*
             * C 区结束后先沿 Y 轴回到底部，再横移到 B 区入口 (-950, 0)。
             * 第 2 个航点保持 +90°，不在路线中提前转到 0°；到点后再由独立校准
             * 状
             * 态机按“0° 校 Y、-90° 校 X”的顺序执行，避免测距期间与路线转向交叉。
             */
            s_dynamic_route[0] = (AppWaypoint_t){g_robot_pos.x, 0.0f, PI / 2.0f,
                                                  false, APP_ACTION_NONE, 0U, 0U};
            s_dynamic_route[1] = (AppWaypoint_t){-950.0f, 0.0f, PI / 2.0f,
                                                  false, APP_ACTION_NONE, 0U, 0U};
            s_dynamic_route[2] = (AppWaypoint_t){-950.0f, 2300.0f, PI,
                                                  false, APP_ACTION_NONE, 0U, 0U};
            App_StartRoute(s_dynamic_route, 2U, APP_MODE_CALIBRATE_B);
            break;

        case APP_MODE_CALIBRATE_B:
            /* 每次任务 Tick 只推进一步；两轴都校准成功后才继续 B 区剩余路线。 */
            App_RouteB_TofCalibrationTick();
            break;

        case APP_MODE_SCAN_B:
            /* 预留：以后在此发送扫码请求并等待完成信号，最长 8 秒。 */
            App_StartRoute(k_route_b, APP_ROUTE_LEN(k_route_b), APP_MODE_BACK);
            break;

        case APP_MODE_SCAN_C:
            if (!s_qr_scan_started) {
                UpperCP_ResetQrResult();
                PCA9685_Set270Angle(APP_QR_CAMERA_SCAN_START_DEG); /* 二维码相机从扫码起始俯仰角开始慢速扫动 */
                PCA9685_Set180Angle(7U, 0.0f); /* 扫码前水平云台回中，避免沿用上一次偏角 */
                s_qr_scan_phase = APP_QR_SCAN_LEFT;
                s_qr_gimbal_angle = 0.0f;
                s_qr_scan_step_start_tick = HAL_GetTick();
                s_qr_scan_step_deadline = s_qr_scan_step_start_tick + APP_QR_SCAN_STEP_MS;
                UpperCP_SendTask("scan");
                s_qr_scan_deadline = HAL_GetTick() + APP_QR_SCAN_TIMEOUT_MS;
                s_qr_voice_deadline = HAL_GetTick();
                s_qr_scan_started = true;
                break;
            }

            if ((CameraFlag != 0U) && (fruits_count == 8U)) {
                if (s_qr_voice_index < fruits_count) {
                    if ((int32_t)(HAL_GetTick() - s_qr_voice_deadline) >= 0) {
                        Voice_Num(30 + fruits[s_qr_voice_index]);
                        s_qr_voice_index++;
                        s_qr_voice_deadline = HAL_GetTick() + APP_QR_VOICE_INTERVAL_MS;
                    }
                    break;
                }

                PCA9685_Set270Angle(APP_CAMERA_CENTER_DEG);
                PCA9685_Set180Angle(7U, 0.0f);
                App_SetMode(APP_MODE_ROUTE_C_ENTRY);
            } else if ((int32_t)(HAL_GetTick() - s_qr_scan_deadline) >= 0) {
                if (s_qr_voice_index < 8U) {
                    if ((int32_t)(HAL_GetTick() - s_qr_voice_deadline) >= 0) {
                        Voice_Num(30 + fruits[s_qr_voice_index]);
                        s_qr_voice_index++;
                        s_qr_voice_deadline = HAL_GetTick() + APP_QR_VOICE_INTERVAL_MS;
                    }
                    break;
                }

                PCA9685_Set270Angle(APP_CAMERA_CENTER_DEG);
                PCA9685_Set180Angle(7U, 0.0f);
                App_SetMode(APP_MODE_ROUTE_C_ENTRY);
                // App_SetMode(APP_MODE_IDLE);
            } else {
                App_QrScanSweepTick();
            }
            break;

        case APP_MODE_ROUTE_C_ENTRY:
            /*
             * 二维码播报结束后先选择 C 区入口：节点 0 为左下入口，节点 11 为右下入口。
             * 入口航点来自 k_route_c，因此到点后导航会把车头调整到 0°，使车尾 TOF 正对底部挡板。
             * 此航点不绑定抓取动作；到点后切换到独立的 TOF 校准模式。
             */
            s_route_c_start_node_idx = App_RouteC_SelectEntryNode(fruits[0]);
            s_dynamic_route[0] = k_route_c[s_route_c_start_node_idx];
            s_dynamic_route[0].has_action = false;
            s_dynamic_route[0].action_mask = APP_ACTION_NONE;
            s_dynamic_route[0].positive_position = 0U;
            s_dynamic_route[0].negative_position = 0U;
            App_StartRoute(s_dynamic_route, 1U, APP_MODE_CALIBRATE_C);
            break;

        case APP_MODE_CALIBRATE_C:
            /* 非阻塞推进 TOF 校准；成功后进入 ROUTE_C，失败则停车回到 IDLE。 */
            App_RouteC_TofCalibrationTick();
            break;

        case APP_MODE_ROUTE_C:
            /* 扫码完成或超时后，使用当前 fruits 数组启动 C 区规划。 */
            App_RouteC_PlanAndRun(fruits, APP_MODE_BACK);
            break;
        case APP_MODE_BACK:
            /* 两步返回原点(0,0)：先Y轴归零，再X轴归零，避免斜线碰撞风险 */
            s_dynamic_route[0].x_mm = g_robot_pos.x;
            s_dynamic_route[0].y_mm = 0.0f;
            s_dynamic_route[0].yaw_rad = PI / 2.0f;    /* 拐角点姿态设为+X方向(+90°)，到点只需顺势旋转90°指引直行 */
            s_dynamic_route[0].has_action = false;
            s_dynamic_route[0].action_mask = APP_ACTION_NONE;
            s_dynamic_route[0].positive_position = 0U;
            s_dynamic_route[0].negative_position = 0U;
            s_dynamic_route[1].x_mm = 0.0f;
            s_dynamic_route[1].y_mm = 0.0f;
            s_dynamic_route[1].yaw_rad = PI / 2.0f;    /* 到达起点原点后保持+X方向，不再恢复初始朝向 */
            s_dynamic_route[1].has_action = false;
            s_dynamic_route[1].action_mask = APP_ACTION_NONE;
            s_dynamic_route[1].positive_position = 0U;
            s_dynamic_route[1].negative_position = 0U;
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
    (void)Navigation_Request(s_route[0].x_mm, s_route[0].y_mm, s_route[0].yaw_rad,
                             App_CurrentMoveHeadingBiasRad());
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
        if (g_app_mode == APP_MODE_ROUTE_C &&
            !s_route_pour_sent &&
            s_route[s_route_index].x_mm == -2615.0f &&
            s_route[s_route_index].y_mm == 2300.0f) {
            UpperCP_SendTask("pour");
            s_route_pour_sent = true;
            s_route_state = APP_ROUTE_WAIT_POUR;
            App_RouteSetDelay(APP_ROUTE_POUR_DELAY_MS);
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
            return;
        }
    } else if (s_route_state == APP_ROUTE_WAIT_POUR) {
        if (!App_RouteDelayExpired()) {
            return;
        }
        s_route_index++;
        s_route_state = APP_ROUTE_WAIT_NAVIGATION;
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
        if (g_app_mode == APP_MODE_SCAN_B) {
            /* B 区抓取树上果子：云台到位后保持当前高度，直接请求视觉抓取。 */
            App_SendVisionTask();
            s_route_state = APP_ROUTE_WAIT_GRAB_FIRST;
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
        App_SendVisionTask();
        s_route_state = APP_ROUTE_WAIT_GRAB_FIRST;
        return;
    } else if (s_route_state == APP_ROUTE_WAIT_GRAB_FIRST) {
         if (!s_grab_done) {
            App_VisionCameraSearchTick();
            return;
        }
        App_VisionCameraSearchReset();
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
        if (g_app_mode == APP_MODE_SCAN_B) {
            /* B 区反向视野同样不降低升降台，直接识别并抓取。 */
            App_SendVisionTask();
            s_route_state = APP_ROUTE_WAIT_GRAB_SECOND;
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
        App_SendVisionTask();
        s_route_state = APP_ROUTE_WAIT_GRAB_SECOND;
        return;
    } else if (s_route_state == APP_ROUTE_WAIT_GRAB_SECOND) {
        if (!s_grab_done) {
            App_VisionCameraSearchTick();
            return;
        }
        App_VisionCameraSearchReset();
        s_route_index++;
        s_route_state = APP_ROUTE_WAIT_NAVIGATION;
    }

    if (s_route_index < s_route_len) {
        /* 自动倒车全局开关已打开：所有航点均允许自动倒车 */
        g_enable_auto_reverse = true;

        /* 本航点已完成，向导航任务请求下一航点；到点结果由下一轮 Tick 检查。 */
        (void)Navigation_Request(s_route[s_route_index].x_mm,
                                 s_route[s_route_index].y_mm,
                                 s_route[s_route_index].yaw_rad,
                                 App_CurrentMoveHeadingBiasRad());
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
 * @brief  按二维码下发顺序规划并执行 C 区环形路线
 * @details C区 12 个节点的环形轨道拓扑结构示意图：
 * 
 *               (y = 2300)
 *      [5] <------------------ [6]
 *       |                       |
 *      [4]                     [7]
 *       |                       |
 *      [3]                     [8]
 *       |                       |
 *      [2]                     [9]
 *       |                       |
 *     [1]                     [10]
 *       |                       |
 *      [0] -----------------> [11]
 *               (y = 0)
 *   (x = -1900)             (x = -2605)
 * 
 *          算法原理：
 *          1. C区拥有 12 个离散顶点 (0~11)，闭合成一个矩形环形轨道赛道。
 *          2. 依次把二维码位置 1~12 映射为环路节点和云台动作位，不重排、不合并。
 *          3. 位置 5~8 位于中间列，可从左右两条通道作业，按当前位置选择更近的停靠侧。
 *          4. 每两个相邻目标之间比较顺/逆时针实际毫米路程，选择较短的一段环路。
 *          5. 保留目标点和防斜切拐角，启动非阻塞航线调度器完成多目标任务。
 * 
 * @param  fruit_positions 8 个水果位置编号数组，每项范围为 1 ~ 12
 * @param  next_mode      完成后跳转的下一个模式
 * @return 0 成功启动，-1 参数错误
 */
static float App_RouteC_GetShortestRingDistance(uint8_t from_node, uint8_t to_node)
{
    uint8_t next;
    float cw_distance = 0.0f;
    float ccw_distance = 0.0f;

    next = from_node;
    while (next != to_node) {
        uint8_t following = (uint8_t)((next + 1U) % 12U);
        cw_distance += fabsf(k_route_c[following].x_mm - k_route_c[next].x_mm) +
                       fabsf(k_route_c[following].y_mm - k_route_c[next].y_mm);
        next = following;
    }

    next = from_node;
    while (next != to_node) {
        uint8_t following = (next == 0U) ? 11U : (uint8_t)(next - 1U);
        ccw_distance += fabsf(k_route_c[following].x_mm - k_route_c[next].x_mm) +
                        fabsf(k_route_c[following].y_mm - k_route_c[next].y_mm);
        next = following;
    }

    return (cw_distance <= ccw_distance) ? cw_distance : ccw_distance;
}

/**
 * @brief 根据二维码中的第一个作业位置选择 C 区左/右入口
 * @details
 * - 位置 1~4 只有左通道作业节点，通常选择节点 0；
 * - 位置 9~12 只有右通道作业节点，通常选择节点 11；
 * - 位置 5~8 可从两侧作业，分别计算两个入口到两侧候选节点的最短环路距离；
 * - 两侧距离相同时选择右入口，减少扫码点附近的横向移动。
 * @param first_position 二维码数组中的第一个水果位置，合法范围 1~12
 * @return 0 表示左下入口，11 表示右下入口；非法位置安全回退到右入口
 */
static uint8_t App_RouteC_SelectEntryNode(uint8_t first_position)
{
    uint8_t first_node;
    float left_distance;
    float right_distance;

    if (first_position >= 1U && first_position <= 4U) {
        first_node = (uint8_t)(5U - first_position);
        left_distance = App_RouteC_GetShortestRingDistance(0U, first_node);
        right_distance = App_RouteC_GetShortestRingDistance(11U, first_node);
    } else if (first_position >= 5U && first_position <= 8U) {
        /* 中间两列水果可由任一通道抓取，因此每个入口都要比较两个候选作业节点。 */
        uint8_t left_lane_node = (uint8_t)(9U - first_position);
        uint8_t right_lane_node = (uint8_t)(first_position + 2U);
        float left_to_left = App_RouteC_GetShortestRingDistance(0U, left_lane_node);
        float left_to_right = App_RouteC_GetShortestRingDistance(0U, right_lane_node);
        float right_to_left = App_RouteC_GetShortestRingDistance(11U, left_lane_node);
        float right_to_right = App_RouteC_GetShortestRingDistance(11U, right_lane_node);

        left_distance = (left_to_left <= left_to_right) ? left_to_left : left_to_right;
        right_distance = (right_to_left <= right_to_right) ? right_to_left : right_to_right;
    } else if (first_position >= 9U && first_position <= 12U) {
        first_node = (uint8_t)(first_position - 2U);
        left_distance = App_RouteC_GetShortestRingDistance(0U, first_node);
        right_distance = App_RouteC_GetShortestRingDistance(11U, first_node);
    } else {
        return 11U;
    }

    /* 距离相同时优先右入口，减少从 C 区扫码点横移的距离。 */
    return (left_distance < right_distance) ? 0U : 11U;
}

/**
 * @brief 使用车尾 TOF，按单个目标距离非阻塞调整底盘前后位置
 * @param target_mm 车尾 TOF 到当前基准面的目标距离，单位：mm
 * @details 首次调用只记录帧序号和超时起点，丢弃转向前的旧测距值。之后每个 Tick
 * 只处理一帧新数据：距离过大时倒车靠近基准面，距离过小时前进远离。
 * 连续两帧进入目标±10mm 才返回完成；断帧超过 300ms 先停车，单轴超过 10s 返回失败。
 * @return APP_TOF_CAL_IN_PROGRESS 调整中；APP_TOF_CAL_DONE 稳定达标；
 *         APP_TOF_CAL_FAILED 超时失败
 */
static AppTofCalibrationResult_t App_RouteB_AdjustTof(float target_mm)
{
    uint32_t now = HAL_GetTick();
    uint32_t frame_seq = TofFrameSeq;
    float distance_mm;

    if (!s_b_tof_cal_started) {
        /* 此时车身刚停止转向：仅建立新帧基准，不用旧距离立即驱动底盘。 */
        Navigation_Stop();
        s_b_tof_last_seq = frame_seq;
        s_b_tof_start_tick = now;
        s_b_tof_last_frame_tick = now;
        s_b_tof_stable_frames = 0U;
        s_b_tof_cal_started = true;
        return APP_TOF_CAL_IN_PROGRESS;
    }

    if ((uint32_t)(now - s_b_tof_start_tick) >= APP_C_TOF_TIMEOUT_MS) {
        Chassis_SetSpeed(0.0f, 0.0f);
        return APP_TOF_CAL_FAILED;
    }

    if (frame_seq == s_b_tof_last_seq) {
        /* 无新帧时不重复判定旧距离；若底盘原先在移动，断帧后必须停车。 */
        if ((uint32_t)(now - s_b_tof_last_frame_tick) >= APP_C_TOF_FRAME_STALE_MS) {
            Chassis_SetSpeed(0.0f, 0.0f);
        }
        return APP_TOF_CAL_IN_PROGRESS;
    }

    distance_mm = TofData;
    if (frame_seq != TofFrameSeq) {
        /* 中断可能正在分别更新距离和帧序号，不使用前后不一致的快照。 */
        return APP_TOF_CAL_IN_PROGRESS;
    }
    s_b_tof_last_seq = frame_seq;
    s_b_tof_last_frame_tick = now;

    if (distance_mm >= (target_mm - APP_C_TOF_TOLERANCE_MM) &&
        distance_mm <= (target_mm + APP_C_TOF_TOLERANCE_MM)) {
        /* 进入容差带后先停车，再累计稳定帧，防止惯性和单帧噪声造成误标定。 */
        Chassis_SetSpeed(0.0f, 0.0f);
        s_b_tof_stable_frames++;
        if (s_b_tof_stable_frames >= APP_C_TOF_STABLE_FRAMES) {
            return APP_TOF_CAL_DONE;
        }
        return APP_TOF_CAL_IN_PROGRESS;
    }

    s_b_tof_stable_frames = 0U;
    if (distance_mm > target_mm) {
        /* TOF 位于车尾：距离过大时倒车靠近标定面。 */
        Chassis_SetSpeed(-APP_B_TOF_SPEED_MM_S, 0.0f);
    } else {
        Chassis_SetSpeed(APP_B_TOF_SPEED_MM_S, 0.0f);
    }
    return APP_TOF_CAL_IN_PROGRESS;
}

/**
 * @brief B 区入口双轴校准：0° 校准 Y，-90° 校准 X
 * @details 入口航点到达时车头保持 +90°。本状态机先原地转到 0°，等待导航完全
 * 结束后把车尾距离调到 200mm，仅将 Y 置 0；再原地转到 -90°，把车尾距离
 * 调到 1250mm，仅将 X 置 -950。两次都成功后，才重新启动原路线的第 3 个航点。
 * @note 航点朝向参数使用弧度；g_robot_pos.yaw 由导航层维护，单位为度。
 */
static void App_RouteB_TofCalibrationTick(void)
{
    AppTofCalibrationResult_t result;

    switch (s_b_cal_state) {
    case APP_B_CAL_ROTATE_Y_START:
        /* 目标坐标取当前值，该请求只用于原地改变最终朝向。 */
        if (Navigation_Request(g_robot_pos.x, g_robot_pos.y, 0.0f, 0.0f) == 0) {
            s_b_cal_state = APP_B_CAL_WAIT_YAW_0;
        }
        break;

    case APP_B_CAL_WAIT_YAW_0:
        /* Navigation_IsIdle() 表示终点角度闭环已结束，此前不允许启动 TOF 直线调整。 */
        if (Navigation_IsIdle()) {
            s_b_tof_cal_started = false;
            s_b_cal_state = APP_B_CAL_ADJUST_Y;
        }
        break;

    case APP_B_CAL_ADJUST_Y:
        result = App_RouteB_AdjustTof(APP_B_TOF_Y_TARGET_MM);
        if (result == APP_TOF_CAL_DONE) {
            /* 200mm 只对应 Y 轴外部基准，保留当前 X 和航向零偏。 */
            Navigation_SetY(0.0f);
            s_b_cal_state = APP_B_CAL_ROTATE_X_START;
        } else if (result == APP_TOF_CAL_FAILED) {
            s_app_running = false;
            App_SetMode(APP_MODE_IDLE);
        }
        break;

    case APP_B_CAL_ROTATE_X_START:
        /* Y 已标定为 0；保持当前位置，原地转到 -90° 准备标定 X。 */
        if (Navigation_Request(g_robot_pos.x, g_robot_pos.y, -PI / 2.0f, 0.0f) == 0) {
            s_b_cal_state = APP_B_CAL_WAIT_YAW_NEG_90;
        }
        break;

    case APP_B_CAL_WAIT_YAW_NEG_90:
        /* 转向完成后重新建立 TOF 帧基准，避免沿用 0° 时的测距数据。 */
        if (Navigation_IsIdle()) {
            s_b_tof_cal_started = false;
            s_b_cal_state = APP_B_CAL_ADJUST_X;
        }
        break;

    default:
        /* 默认分支即 APP_B_CAL_ADJUST_X：根据 1250mm 目标调整，只在稳定达标后写入 X。 */
        result = App_RouteB_AdjustTof(APP_B_TOF_X_TARGET_MM);
        if (result == APP_TOF_CAL_DONE) {
            Navigation_SetX(-950.0f);
            /* 恢复原动态路线的第 3 点 (-950, 2300, 180°)，到点后转入 B 区作业路线。 */
            App_StartRoute(&s_dynamic_route[2], 1U, APP_MODE_SCAN_B);
        } else if (result == APP_TOF_CAL_FAILED) {
            s_app_running = false;
            App_SetMode(APP_MODE_IDLE);
        }
        break;
    }
}

/**
 * @brief 在 C 区入口利用车尾 TOF 将底盘调整到距挡板 200mm，并把导航 Y 标定为 0
 * @details 本函数由 15ms 应用任务周期调用，不包含阻塞等待：
 * 1. 首次进入时停止导航并记录超时、新帧基准；
 * 2. 只处理 TofFrameSeq 变化后的有效新数据，连续断帧时先停车；
 * 3. 距离大于 210mm 时倒车靠近挡板，小于 190mm 时前进远离挡板；
 * 4. 连续两帧落入 190~210mm 后停车，仅将 Y 设置为 0，再启动原 C 区作业路线；
 * 5. 5s 内未完成则停车并中止路线，禁止使用未校准坐标继续运行。
 */
static void App_RouteC_TofCalibrationTick(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t frame_seq = TofFrameSeq;
    float distance_mm;

    if (!s_c_tof_cal_started) {
        /* 丢弃进入状态前的旧测量值，必须等下一帧连续上报数据再开始移动。 */
        Navigation_Stop();
        s_c_tof_last_seq = frame_seq;
        s_c_tof_start_tick = now;
        s_c_tof_last_frame_tick = now;
        s_c_tof_stable_frames = 0U;
        s_c_tof_cal_started = true;
        return;
    }

    if ((uint32_t)(now - s_c_tof_start_tick) >= APP_C_TOF_TIMEOUT_MS) {
        /* 传感器异常或机械运动未到位时采取停车退出，不写入 Y=0。 */
        Chassis_SetSpeed(0.0f, 0.0f);
        s_app_running = false;
        App_SetMode(APP_MODE_IDLE);
        return;
    }

    if (frame_seq == s_c_tof_last_seq) {
        /* 没有新测量值时不得重复使用旧距离；运动中断帧超过阈值必须停车。 */
        if ((uint32_t)(now - s_c_tof_last_frame_tick) >= APP_C_TOF_FRAME_STALE_MS) {
            Chassis_SetSpeed(0.0f, 0.0f);
        }
        return;
    }

    distance_mm = TofData;
    if (frame_seq != TofFrameSeq) {
        /* 中断正在更新距离与帧序号，本周期不使用可能不一致的一组数据。 */
        return;
    }
    s_c_tof_last_seq = frame_seq;
    s_c_tof_last_frame_tick = now;

    if (distance_mm >= (APP_C_TOF_TARGET_MM - APP_C_TOF_TOLERANCE_MM) &&
        distance_mm <= (APP_C_TOF_TARGET_MM + APP_C_TOF_TOLERANCE_MM)) {
        /* 达标帧先停车，再累计稳定次数，防止惯性和单帧噪声导致错误置零。 */
        Chassis_SetSpeed(0.0f, 0.0f);
        s_c_tof_stable_frames++;
        if (s_c_tof_stable_frames >= APP_C_TOF_STABLE_FRAMES) {
            Navigation_SetY(0.0f);
            App_SetMode(APP_MODE_ROUTE_C);
        }
        return;
    }

    s_c_tof_stable_frames = 0U;
    if (distance_mm > APP_C_TOF_TARGET_MM) {
        /* TOF 位于车尾：距离过大时倒车靠近挡板。 */
        Chassis_SetSpeed(-APP_C_TOF_SPEED_MM_S, 0.0f);
    } else {
        /* 距离过小时前进远离挡板。 */
        Chassis_SetSpeed(APP_C_TOF_SPEED_MM_S, 0.0f);
    }
}

int32_t App_RouteC_PlanAndRun(const uint8_t *fruit_positions,
                              AppMode_t next_mode)
{
    uint8_t i;
    uint8_t curr;
    uint8_t next;
    uint8_t node_idx;
    uint8_t out_len = 0U;
    uint8_t action;
    float cw_distance;
    float ccw_distance;
    bool choose_cw;

    if (fruit_positions == NULL) {
        return -1;
    }

    /* 从刚完成 TOF 校准的实际入口开始规划，不能再固定假设从右入口节点 11 起步。 */
    curr = s_route_c_start_node_idx;

    /* 严格按二维码数组顺序逐个生成目标，不能按环路位置重新排序。 */
    for (i = 0U; i < 8U; i++) {
        uint8_t position = fruit_positions[i];

        if (position >= 1U && position <= 4U) {
            node_idx = (uint8_t)(5U - position);
            action = APP_ACTION_NEGATIVE;
        } else if (position >= 5U && position <= 8U) {
            uint8_t right_lane_node = (uint8_t)(9U - position);  /* 右通道 0~5 的左侧作业点 */
            uint8_t left_lane_node = (uint8_t)(position + 2U);   /* 左通道 6~11 的右侧作业点 */
            float right_lane_dist = App_RouteC_GetShortestRingDistance(curr, right_lane_node);
            float left_lane_dist = App_RouteC_GetShortestRingDistance(curr, left_lane_node);

            if (left_lane_dist < right_lane_dist) {
                node_idx = left_lane_node;
                action = APP_ACTION_NEGATIVE;
            } else {
                node_idx = right_lane_node;
                action = APP_ACTION_POSITIVE;
            }
        } else if (position >= 9U && position <= 12U) {
            node_idx = (uint8_t)(position - 2U);
            action = APP_ACTION_POSITIVE;
        } else {
            return -1;
        }

        cw_distance = 0.0f;
        next = curr;
        while (next != node_idx) {
            uint8_t following = (uint8_t)((next + 1U) % 12U);
            cw_distance += fabsf(k_route_c[following].x_mm - k_route_c[next].x_mm) +
                           fabsf(k_route_c[following].y_mm - k_route_c[next].y_mm);
            next = following;
        }

        ccw_distance = 0.0f;
        next = curr;
        while (next != node_idx) {
            uint8_t following = (next == 0U) ? 11U : (uint8_t)(next - 1U);
            ccw_distance += fabsf(k_route_c[following].x_mm - k_route_c[next].x_mm) +
                            fabsf(k_route_c[following].y_mm - k_route_c[next].y_mm);
            next = following;
        }

        choose_cw = (cw_distance <= ccw_distance);
        while (curr != node_idx) {
            next = choose_cw ? (uint8_t)((curr + 1U) % 12U)
                             : ((curr == 0U) ? 11U : (uint8_t)(curr - 1U));
            curr = next;

            /* 中途只保留矩形拐角；目标点在循环后单独加入并绑定本次动作。 */
            if (curr != node_idx &&
                (curr == 0U || curr == 5U || curr == 6U || curr == 11U)) {
                if (out_len >= APP_ROUTE_LEN(s_dynamic_route)) {
                    return -1;
                }
                s_dynamic_route[out_len] = k_route_c[curr];
                s_dynamic_route[out_len].has_action = false;
                s_dynamic_route[out_len].action_mask = APP_ACTION_NONE;
                s_dynamic_route[out_len].positive_position = 0U;
                s_dynamic_route[out_len].negative_position = 0U;
                out_len++;
            }
        }

        if (out_len >= APP_ROUTE_LEN(s_dynamic_route)) {
            return -1;
        }
        s_dynamic_route[out_len] = k_route_c[node_idx];
        s_dynamic_route[out_len].has_action = true;
        s_dynamic_route[out_len].action_mask = action;
        s_dynamic_route[out_len].positive_position =
            (action == APP_ACTION_POSITIVE) ? position : 0U;
        s_dynamic_route[out_len].negative_position =
            (action == APP_ACTION_NEGATIVE) ? position : 0U;
        out_len++;
    }

    s_route_pour_sent = false;
    App_StartRoute(s_dynamic_route, out_len, next_mode);

    return 0;
}
