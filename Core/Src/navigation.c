#include "navigation.h"
#include "bujin.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tiancan.h"
#include "vofa.h"
#include <math.h>

#define NAV_PI                    3.1415926f

/* 旋转对齐控制 PID 及前馈参数 */
static float align_kp = 2.8f;
static float align_ki = 0.0f;
static float align_kd = 0.3f;
static float target_yaw = 0.0f;  /**< 旋转对齐目标朝向角度 (rad) */

/** 
 * @brief 旋转对齐角度 PID 全局变量（通道 1）
 * 包含了 Navigation_HandleTargetAlign 函数中使用的 PID 控制参数与目标值
 */
TiancanPid_t anglepid = {
    .name   = "anglepid",
    .kp     = &align_kp,
    .ki     = &align_ki,
    .kd     = &align_kd,
    .target = &target_yaw
};

#define ALIGN_FF_BASE             0.6f          /**< 旋转对齐静态摩擦前馈 (rad/s) */
#define ALIGN_MAX_ANGULAR         1.0f          /**< 旋转对齐最大角速度 (rad/s) */
#define ALIGN_ERR_THRESH          0.05f         /**< 旋转对齐精度阈值 (rad)，约 2.86 度，防止死锁卡住 */

/* 直线行进控制参数 */
#define MOVE_LINEAR_SPEED         300.0f        /**< 直线行进最大期望线速度 (mm/s) */
#define MOVE_LINEAR_RAMP          15.0f         /**< 线速度斜坡步长：每个控制周期最大增量 (mm/s)，用于软启动 */
static float move_kp = 1.5f;
static float move_ki = 0.0f;
static float move_kd = 0.1f;
static float move_target = 0.0f;  /**< 直线行进航偏纠偏目标朝向 (rad) */

/** 
 * @brief 直线行进航偏纠偏 PID 全局变量（通道 2）
 * 包含了 Navigation_HandleMoving 函数中使用的 PID 控制参数与目标值
 */
TiancanPid_t movepid = {
    .name   = "movepid",
    .kp     = &move_kp,
    .ki     = &move_ki,
    .kd     = &move_kd,
    .target = &move_target
};

#define MOVE_ARRIVE_DIST          35.0f        /**< 目标点判定范围半径 (mm)，35mm 判定到达，放宽到达死区防止卡点 */
#define MOVE_MIN_LINEAR           20.0f         /**< 减速时最小保证线速度 (mm/s) */
#define MOVE_MIN_SPEED            30.0f         /**< 最终线速度最小限制 (mm/s)，平滑减速低速到位 */
#define MOVE_MAX_ANGULAR          1.5f          /**< 直线纠偏中最大角速度限制 (rad/s) */
#define MOVE_FF_BASE              100.0f         /**< 直线行进静摩擦力前馈 (mm/s)，叠加到最终线速度克服启动死区 */

/* 到达最终角度调整控制参数 */
static float arrived_kp = 3.0f;
static float arrived_ki = 0.0f;
static float arrived_kd = 0.1f;
static float arrived_target = 0.0f;  /**< 终点角度调整目标朝向 (rad) */

/** 
 * @brief 终点角度微调 PID 全局变量（通道 3）
 * 包含了 Navigation_HandleArrived 函数中使用的 PID 控制参数与目标值
 */
TiancanPid_t arrivedpid = {
    .name   = "arrivedpid",
    .kp     = &arrived_kp,
    .ki     = &arrived_ki,
    .kd     = &arrived_kd,
    .target = &arrived_target
};

#define ARRIVED_SETTLE_MS         800U          /**< 到达后停稳等待时间 (ms)，让机身惯性消除后再转圈 */
#define ARRIVED_MAX_ANGULAR       1.0f          /**< 终点最大角速度限制 (rad/s)，降低防轮胎打滑 */
#define ARRIVED_MIN_ANGULAR       0.6f          /**< 终点最小角速度限制 (rad/s)，防止转速过低电机不转 */
#define ARRIVED_FF_BASE           0.6f         /**< 终点旋转静摩擦前馈 (rad/s)，突破起步死区 */
#define ARRIVED_ERR_THRESH        0.05f         /**< 最终角度对齐允许最大误差 (rad)，约 2.86 度，防止死锁死等 */

/* 状态机全局变量 */
Navigation_State_t navigation_state = NAVIGATION_STATE_IDLE;
volatile position_t g_robot_pos = {0.0f, 0.0f, 0.0f};           /* 机器人在世界坐标系下的绝对位姿 */
struct move speed = {0.0f, 0.0f, 0.0f};                         /* 保留以防其他文件 extern */
struct move angle_speed = {0.0f, 0.0f, 0.0f};
float nav_yaw_zero_deg=0.0f;
int TarAngle;
float TarPos = 360.0f;
float v[2];                                                     /**< 左右轮线速度，单位：mm/s */

static position_t target;                                       /* 当前的导航目标位姿 */
static position_t start;                                        /* 启动本次导航时的机器人位姿 */
static bool s_is_reverse_mode = false;                           /* 自动倒车模式标志 */

/* 静态控制函数声明 */
static void Navigation_HandleIdle(void);
static void Navigation_HandleTargetAlign(void);
static void Navigation_HandleMoving(void);
static void Navigation_HandleArrived(void);
static float Navigation_NormalizeRad(float angle);

static void Navigation_RegisterTiancanPids(void)
{
    static bool registered;

    if (!registered) {
        /* 通道 1：注册旋转对齐角度 PID (anglepid，包含 target_yaw) */
        (void)Tiancan_RegisterPid(anglepid.name, anglepid.kp, anglepid.ki, anglepid.kd, anglepid.target);
        /* 通道 2：注册直线行进纠偏 PID (movepid，包含 move_target) */
        (void)Tiancan_RegisterPid(movepid.name, movepid.kp, movepid.ki, movepid.kd, movepid.target);
        /* 通道 3：注册到达最终调整 PID (arrivedpid，包含 arrived_target) */
        (void)Tiancan_RegisterPid(arrivedpid.name, arrivedpid.kp, arrivedpid.ki, arrivedpid.kd, arrivedpid.target);
        registered = true;
    }
}

/**
 * @brief 规范化角度到 [-180, 180] 度范围内
 */
float Navigation_NormalizeDeg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief 复位里程计与朝向角零偏
 */
void Navigation_Reset(float start_x_mm, float start_y_mm, float yaw_zero_deg)
{
    Navigation_RegisterTiancanPids();
    g_robot_pos.x = start_x_mm;
    g_robot_pos.y = start_y_mm;
    g_robot_pos.yaw = 0.0f;
    nav_yaw_zero_deg = yaw_zero_deg;
    navigation_state = NAVIGATION_STATE_IDLE;
    /*
     * 此处仍处于 main() 的调度器启动前阶段，只初始化导航状态。
     * 电机使能与停止帧由 StartDefaultTask() 在 FreeRTOS 运行后统一下发，
     * 避免在启动阶段进入包含 osDelay() 的 USART2 电机通信链。
     */
}

/**
 * @brief 根据位移差更新机器人二维坐标
 * @note  由于机器人在大地坐标系下的航向偏角定义，采用 sin(yaw) 更新 x，cos(yaw) 更新 y
 */
void Navigation_UpdateByDelta(float delta_mm, float yaw_deg)
{
    float yaw_rad = yaw_deg * NAV_PI / 180.0f;

    g_robot_pos.yaw = yaw_deg;
    g_robot_pos.x += delta_mm * sinf(yaw_rad);
    g_robot_pos.y += delta_mm * cosf(yaw_rad);
}

/**
 * @brief 获取当前的偏航角度
 */
float Navigation_GetYawDeg(void)
{
    return g_robot_pos.yaw;
}

/**
 * @brief 周期调度主函数，根据当前状态机执行不同的动作
 */
void Navigation_TaskTick(void)
{
    switch (navigation_state) {
    case NAVIGATION_STATE_IDLE:
        Navigation_HandleIdle();
        break;
    case NAVIGATION_STATE_TARGET_ALIGN:
        Navigation_HandleTargetAlign();
        break;
    case NAVIGATION_STATE_MOVING:
        Navigation_HandleMoving();
        break;
    case NAVIGATION_STATE_ARRIVED:
        Navigation_HandleArrived();
        break;
    default:
        navigation_state = NAVIGATION_STATE_IDLE;
        break;
    }
}

/**
 * @brief 发送导航任务请求
 */
int8_t Navigation_Request(float target_x_mm, float target_y_mm, float target_yaw_rad)
{
    /* 如果当前正在执行其他导航任务，直接拒绝 */
    if (navigation_state != NAVIGATION_STATE_IDLE) {
        return -1;
    }

    target.x = target_x_mm;
    target.y = target_y_mm;
    target.yaw = target_yaw_rad;
    start = g_robot_pos;

    /* 1. 计算从起始点指向目标点的绝对方位角 */
    float heading_angle = atan2f(target.x - start.x, target.y - start.y);

    /* 2. 计算如果按正常前进，车头需要旋转的角度偏差 */
    float fwd_err = Navigation_NormalizeRad(heading_angle - g_robot_pos.yaw * NAV_PI / 180.0f);

    /* 3. 自动倒车判定：当目标点位于车后方（角度偏差绝对值 > 100° ≈ 1.745 rad）时，开启自动倒车模式 */
    if (fabsf(fwd_err) > 1.745f) {
        s_is_reverse_mode = true;
    } else {
        s_is_reverse_mode = false;
    }

    /* 触发状态机，第一步进行朝向目标点的对齐旋转 */
    navigation_state = NAVIGATION_STATE_TARGET_ALIGN;
    return 0;
}

/**
 * @brief 查询导航状态是否空闲
 */
bool Navigation_IsIdle(void)
{
    return navigation_state == NAVIGATION_STATE_IDLE;
}

/**
 * @brief 强制关闭状态机并将底盘轮子停转
 */
void Navigation_Stop(void)
{
    navigation_state = NAVIGATION_STATE_IDLE;
    Chassis_SetSpeed(0.0f, 0.0f);
}

/**
 * @brief 闲置状态处理：重置期望速度
 */
static void Navigation_HandleIdle(void)
{
    speed.real = 0.0f;
    angle_speed.real = 0.0f;
}

/**
 * @brief 原地旋转调整朝向，使机器人转向目标点方向（前进时车头对准，倒车时车尾对准）
 */ 
static void Navigation_HandleTargetAlign(void)
{
    static Navigation_State_t last_state = NAVIGATION_STATE_IDLE;
    static float last_err;
    static TickType_t last_time;
    float err;
    float dt;
    float angular_speed;
    TickType_t now;

    /* 首次进入该状态时，计算两点之间的绝对方位角作为期望旋转朝向 */
    if (last_state != NAVIGATION_STATE_TARGET_ALIGN) {
        float heading_angle = atan2f(target.x - start.x, target.y - start.y);
        if (s_is_reverse_mode) {
            /* 倒车模式：车尾正对目标点 */
            *anglepid.target = Navigation_NormalizeRad(heading_angle + NAV_PI);
        } else {
            /* 前进模式：车头正对目标点 */
            *anglepid.target = heading_angle;
        }
        last_err = 0.0f;
        last_time = xTaskGetTickCount();
    }
    last_state = NAVIGATION_STATE_TARGET_ALIGN;

    /* 偏差角度 = 期望角(rad) - 当前角度(rad) */
    err = Navigation_NormalizeRad(*anglepid.target - g_robot_pos.yaw * NAV_PI / 180.0f);
    
    /* 偏差角度小于判定门限，说明朝向已对准目标点，进入直线行进状态 */
    if (fabsf(err) < ALIGN_ERR_THRESH) {
        navigation_state = NAVIGATION_STATE_MOVING;
        last_state = NAVIGATION_STATE_IDLE;
        return;
    }

    now = xTaskGetTickCount();
    dt = (float)(now - last_time) / (float)configTICK_RATE_HZ;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;
    }

    /* PD闭环反馈控制旋转（使用 anglepid 全局变量中的 PID 参数） */
    angular_speed = -((*anglepid.kp) * err + (*anglepid.kd) * (err - last_err) / dt);
    
    /* 比例前馈：误差越大前馈越强，平滑归零无突变，从根本上避免超调 */
    {
        float ff_ratio = fabsf(err) / NAV_PI;
        if (ff_ratio > 1.0f) ff_ratio = 1.0f;
        angular_speed -= (err > 0.0f) ? (ALIGN_FF_BASE * ff_ratio) : -(ALIGN_FF_BASE * ff_ratio);
    }
    
    /* 饱和度限幅保护 */
    if (angular_speed > ALIGN_MAX_ANGULAR) {
        angular_speed = ALIGN_MAX_ANGULAR;
    } else if (angular_speed < -ALIGN_MAX_ANGULAR) {
        angular_speed = -ALIGN_MAX_ANGULAR;
    }

    /* 旋转状态：线速度为0，输出期望角速度 */
    Chassis_SetSpeed(0.0f, angular_speed);

    last_err = err;
    last_time = now;
}

/**
 * @brief 直线行进处理函数：直线行进的同时进行航偏纠偏控制
 */
static void Navigation_HandleMoving(void)
{
    static Navigation_State_t last_state = NAVIGATION_STATE_IDLE;
    static float last_err;
    static float current_linear_speed;           /**< 当前实际线速度，斜坡软启动用 */
    static TickType_t last_time;
    float dx = target.x - g_robot_pos.x;
    float dy = target.y - g_robot_pos.y;
    float distance = sqrtf(dx * dx + dy * dy);   /* 计算距离目标点的欧氏距离 */
    float err;
    float dt;
    float angular_speed;
    float target_linear_speed;
    float angular_ratio;
    TickType_t now;

    /* 到达目标点判定半径内，说明行进完成，进入终点角度微调状态 */
    if (distance < MOVE_ARRIVE_DIST) {
        navigation_state = NAVIGATION_STATE_ARRIVED;
        Chassis_SetSpeed(0.0f, 0.0f);
        current_linear_speed = 0.0f;
        last_state = NAVIGATION_STATE_IDLE;
        return;
    }

    if (last_state != NAVIGATION_STATE_MOVING) {
        last_err = 0.0f;
        current_linear_speed = 0.0f;             /* 首次进入从零开始斜坡起步 */
        last_time = xTaskGetTickCount();
    }
    last_state = NAVIGATION_STATE_MOVING;

    /* 纠偏误差：根据当前位置和目标点连线的方位角，对比当前机器人的朝向角度 */
    float heading_angle = atan2f(dx, dy);
    *movepid.target = s_is_reverse_mode ? Navigation_NormalizeRad(heading_angle + NAV_PI) : heading_angle;
    err = Navigation_NormalizeRad(*movepid.target - g_robot_pos.yaw * NAV_PI / 180.0f);

    now = xTaskGetTickCount();
    dt = (float)(now - last_time) / (float)configTICK_RATE_HZ;
    /* dt 上下限保护，防止微分项在极端调度间隔下爆炸 */
    if (dt < 0.01f || dt > 0.1f) {
        dt = 0.01f;
    }

    /* PD计算纠偏输出的角速度（使用 movepid 全局变量中的 PID 参数） */
    angular_speed = -((*movepid.kp) * err + (*movepid.kd) * (err - last_err) / dt);
    if (angular_speed > MOVE_MAX_ANGULAR) {
        angular_speed = MOVE_MAX_ANGULAR;
    } else if (angular_speed < -MOVE_MAX_ANGULAR) {
        angular_speed = -MOVE_MAX_ANGULAR;
    }

    /* 临近终点减速逻辑，距离小于 200mm 时，线速度呈线性比例减小 */
    if (distance < 200.0f) {
        target_linear_speed = MOVE_LINEAR_SPEED * distance / 200.0f;
        if (target_linear_speed < MOVE_MIN_LINEAR) {
            target_linear_speed = MOVE_MIN_LINEAR; /* 限制最小速度，防止死区卡住 */
        }
    } else {
        target_linear_speed = MOVE_LINEAR_SPEED;
    }

    /* 角速度越大说明偏差越大，按比例压制线速度，防止两轮速差超限打滑 */
    angular_ratio = fabsf(angular_speed) / MOVE_MAX_ANGULAR; /* 归一化角速度占比 [0, 1] */
    target_linear_speed *= (1.0f - 0.5f * angular_ratio);    /* 角速度满幅时线速度最多降低 50% */
    if (target_linear_speed < MOVE_MIN_LINEAR) {
        target_linear_speed = MOVE_MIN_LINEAR;
    }

    /* 斜坡滤波：加速与减速均进行平滑控制，防止速度剧变造成急停或甩尾 */
    if (current_linear_speed < target_linear_speed) {
        current_linear_speed += MOVE_LINEAR_RAMP;
        if (current_linear_speed > target_linear_speed) {
            current_linear_speed = target_linear_speed;
        }
    } else if (current_linear_speed > target_linear_speed) {
        current_linear_speed -= MOVE_LINEAR_RAMP;
        if (current_linear_speed < target_linear_speed) {
            current_linear_speed = target_linear_speed;
        }
    }

    /* 根据倒车模式选择给底盘发送的线速度正负号，并叠加静摩擦前馈 */
    float ff_sign = s_is_reverse_mode ? -1.0f : 1.0f;
    float final_linear_speed = (s_is_reverse_mode ? -current_linear_speed : current_linear_speed)
                             + MOVE_FF_BASE * ff_sign;

    /* 最小线速度限制：低于阈值上提到 ±MIN，防止输出过低电机堵转 */
    if (final_linear_speed > 0.0f && final_linear_speed < MOVE_MIN_SPEED) {
        final_linear_speed = MOVE_MIN_SPEED;
    } else if (final_linear_speed < 0.0f && final_linear_speed > -MOVE_MIN_SPEED) {
        final_linear_speed = -MOVE_MIN_SPEED;
    }

    Chassis_SetSpeed(final_linear_speed, angular_speed);
    last_err = err;
    last_time = now;
}

/**
 * @brief 终点调整状态处理：在目标点原地旋转至最终期望的偏航角
 * @note  进入后先停稳等待 ARRIVED_SETTLE_MS，消除惯性，再开始转圈调角度。
 */
static void Navigation_HandleArrived(void)
{
    static Navigation_State_t last_state = NAVIGATION_STATE_IDLE;
    static float last_err;
    static TickType_t last_time;
    static TickType_t settle_start;  /**< 进入停稳阶段的时刻 */
    bool settling;                   /**< 当前是否处于停稳等待中 */
    float err;
    float dt;
    float angular_speed;
    TickType_t now;

    /* 首次进入 ARRIVED → 记录停稳起始时刻 */
    if (last_state != NAVIGATION_STATE_ARRIVED) {
        settle_start = xTaskGetTickCount();
        last_state = NAVIGATION_STATE_ARRIVED;
        last_err = 0.0f;
        last_time = settle_start;
    }

    /* 1. 优先停稳等待阶段：先保持零速，等待惯性彻底消除让车子完全停稳 */
    settling = ((xTaskGetTickCount() - settle_start) < pdMS_TO_TICKS(ARRIVED_SETTLE_MS));
    if (settling) {
        Chassis_SetSpeed(0.0f, 0.0f);
        /* 停稳期间持续刷新 last_time，避免后续转圈阶段 dt 异常放大 */
        last_time = xTaskGetTickCount();
        return;
    }

    /* 2. 车子停稳后再计算目标角度与当前角度的偏差 */
    *arrivedpid.target = target.yaw * PI / 180.0f;
    err = Navigation_NormalizeRad(*arrivedpid.target - g_robot_pos.yaw * PI / 180.0f);

    /* 朝向角误差小于允许误差 → 导航完成，停车进入空闲 */
    if (fabsf(err) < ARRIVED_ERR_THRESH) {
        Navigation_Stop();
        last_state = NAVIGATION_STATE_IDLE;
        return;
    }

    /* --- 以下为转圈调角度阶段 --- */

    now = xTaskGetTickCount();
    dt = (float)(now - last_time) / (float)configTICK_RATE_HZ;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;
    }

    /* PD计算旋转调整的角速度（使用 arrivedpid 全局变量中的 PID 参数） */
    angular_speed = -((*arrivedpid.kp) * err + (*arrivedpid.kd) * (err - last_err) / dt);

    /* 静摩擦前馈：与 TargetAlign 同理，误差比例缩放，突破起步死区 */
    {
        float ff_ratio = fabsf(err) / (float)PI;
        if (ff_ratio > 1.0f) ff_ratio = 1.0f;
        angular_speed -= (err > 0.0f) ? (ARRIVED_FF_BASE * ff_ratio) : -(ARRIVED_FF_BASE * ff_ratio);
    }

    if (angular_speed > ARRIVED_MAX_ANGULAR) {
        angular_speed = ARRIVED_MAX_ANGULAR;
    } else if (angular_speed < -ARRIVED_MAX_ANGULAR) {
        angular_speed = -ARRIVED_MAX_ANGULAR;
    }

    /* 最小角速度限制：低于阈值的非零输出上提到 ±MIN，防止电机堵转或单边不转 */
    if (angular_speed > 0.0f && angular_speed < ARRIVED_MIN_ANGULAR) {
        angular_speed = ARRIVED_MIN_ANGULAR;
    } else if (angular_speed < 0.0f && angular_speed > -ARRIVED_MIN_ANGULAR) {
        angular_speed = -ARRIVED_MIN_ANGULAR;
    }

    Chassis_SetSpeed(0.0f, angular_speed);
    last_err = err;
    last_time = now;
}

/**
 * @brief 设置底盘的全局速度（线速度与角速度）
 * @note  根据差速机器人运动学公式：
 *        v_left  = v_linear + w * half_track
 *        v_right = v_linear - w * half_track
 *        然后根据轮子半径转换为对应电机的物理转速(RPM)。
 */
void Chassis_SetSpeed(float linear_vel_mm_s, float angular_vel_rad_s)
{
    float left_vel = linear_vel_mm_s + angular_vel_rad_s * HALF_TRACK_MM;
    float right_vel = linear_vel_mm_s - angular_vel_rad_s * HALF_TRACK_MM;
    
    /* 物理RPM计算式：rpm = v / (2 * pi * r) * 60 */
    float left_rpm = fabsf(left_vel) / (TWO_PI * WHEEL_RADIUS_MM) * 60.0f;
    float right_rpm = fabsf(right_vel) / (TWO_PI * WHEEL_RADIUS_MM) * 60.0f;

    v[0] = left_vel;
    v[1] = right_vel;
    
    uint16_t send_left_rpm = (uint16_t)(left_rpm + 0.5f);
    uint16_t send_right_rpm = (uint16_t)(right_rpm + 0.5f);
    uint8_t left_dir = (left_vel >= 0.0f) ? 1U : 0U;
    uint8_t right_dir = (right_vel >= 0.0f) ? 1U : 0U;

    /* 低速死区过滤：低于 2 RPM (约 10mm/s) 时强行归零并固定方向，消除极低速电机乱转与颠簸震荡 */
    if (send_left_rpm < 2U) {
        send_left_rpm = 0U;
        left_dir = 1U;
    }
    if (send_right_rpm < 2U) {
        send_right_rpm = 0U;
        right_dir = 1U;
    }

    /* 加速度 acc 设为 50，实现电机的平滑加减速，避免 255 造成的急停顿挫 */
    uint8_t acc = 50U;

    /* 控制下发：低速死区过滤后发送，同时下发并设置同步标志 */
    Emm_V5_Vel_Control(left_head, left_dir, send_left_rpm, acc, true);
    Emm_V5_Vel_Control(left_tail, left_dir, send_left_rpm, acc, true);
    Emm_V5_Vel_Control(right_head, right_dir, send_right_rpm, acc, true);
    Emm_V5_Vel_Control(right_tail, right_dir, send_right_rpm, acc, true);
    
    /* 广播/通知，触发多电机硬件同步对齐运动 */
    Emm_V5_Synchronous_motion(0);
}

/**
 * @brief 规范化角度到 [-PI, PI] 弧度区间
 */
static float Navigation_NormalizeRad(float angle)
{
    while (angle > PI) {
        angle -= TWO_PI;
    }
    while (angle < -PI) {
        angle += TWO_PI;
    }
    return angle;
}

