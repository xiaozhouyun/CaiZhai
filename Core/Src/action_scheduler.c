#include "action_scheduler.h"
#include "UpperCP.h"
#include "app.h"
#include "arms.h"
#include "bujin.h"
#include "pca9685.h"
#include "tof200f.h"
#include "vofa.h"
#include "cmsis_os.h"
#include <math.h>

#define ARM_EXTEND_MIN_ANGLE_DEG      (-80.0f)
#define ARM_EXTEND_MAX_ANGLE_DEG      (25.0f)
#define ARM_EXTEND_TOTAL_RANGE_DEG    (105.0f)
#define ARM_EXTEND_TOTAL_RANGE_MM     (300.0f)

/*
 * 机械动作反应时间（单位：ms）。
 * 这些时间从“命令已下发”开始计时，期间状态机不下发下一机械动作，
 * 但 StartTask07、导航和串口任务仍可继续运行。实机调慢/调快只改这里。
 */
#define ARM_TOF_SETTLE_MS             200U   /* 触发测距后，等待 TOF 刷新 */
#define ARM_CLAW_OPEN_MS              400U   /* 开爪到完全张开 */
#define ARM_EXTEND_SETTLE_MS          1200U   /* 伸缩臂移动到测距目标 */
#define ARM_CLAW_CLOSE_MS             500U   /* 闭爪后等待夹紧果实 */
#define ARM_PUT_LIFT_SETTLE_MS        1800U   /* 抓取后升到 10cm：无到位反馈，保守等待避免与收臂重叠 */
#define ARM_RETRACT_SETTLE_MS         600U   /* 伸缩臂完全收回 */
#define ARM_GIMBAL_SETTLE_MS          200U   /* 云台停稳后再开爪，防惯性摆动 */
#define ARM_CLAW_RELEASE_MS           600U   /* 开爪后等待果实脱离 */
#define ARM_SKIP_LIFT_SETTLE_MS       1500U   /* 跳过目标时升到 5cm：无到位反馈，保守等待避免与收臂重叠 */
#define ARM_BAD_TOF_SETTLE_MS         500U   /* 坏果流程保留较长测距等待 */
#define ARM_BAD_RELEASE_SETTLE_MS     500U   /* 坏果开爪后的机构反应时间 */

/*
 * 抓取动作状态机说明：
 * 1. 上位机收到 arm 命令后只调用 ActionScheduler_RequestVisionArm() 投递动作；
 * 2. StartTask07 每 20ms 调用一次 ActionScheduler_Tick()；
 * 3. 每个 WAIT 状态只在截止时间到达时执行一次“下一个动作”，然后切换状态；
 * 4. 因此等待期间 Task07 会立即返回，不会占住串口、导航和 OLED 等其他任务。
 */
typedef enum {
    ACTION_IDLE,
    /* 无正在执行的抓取/复位动作；允许接收下一条 arm:0~6 命令。 */

    ACTION_GRAB_WAIT_DISTANCE,
    /* arm:0 已调用 get_dis()；等待 100ms 后读取最新 TofData，并开始开爪。 */

    ACTION_GRAB_WAIT_OPEN,
    /* 正常抓取已下发通道5=-30度开爪；等待 800ms，避免伸臂时碰撞夹爪。 */

    ACTION_GRAB_WAIT_CLOSE,
    /* 已按 TofData/10-1cm 下发通道6伸臂角度；等待 100ms 后执行闭爪。 */

    ACTION_GRAB_WAIT_GRIP,
    /* 已下发通道5=3度闭爪；等待 800ms，确保果实夹紧后才允许转运。 */

    ACTION_PUT_WAIT_LIFT,
    /* 已升到 10cm 安全高度；等待 3000ms 后把通道7云台转回 0度中位。 */

    ACTION_PUT_WAIT_EXTEND,
    /* 伸缩臂已收回；下发升到 10cm 的命令。 */

    ACTION_PUT_WAIT_ROTATE,
    /* 云台已回中；等待 500ms 后开爪，避免在转动过程中释放果实。 */

    ACTION_PUT_WAIT_OPEN,
    /* 已下发开爪；等待 1000ms 让果实离爪，然后通知路线执行下一步。 */

    ACTION_SKIP_WAIT_LIFT,
    /* arm:5 或对准失败：已升到 5cm 安全高度；等待 2000ms 后收臂。 */

    ACTION_SKIP_WAIT_EXTEND,
    /* 跳过目标时伸缩臂已收回；等待 1000ms 后将云台转回 0度。 */

    ACTION_SKIP_WAIT_ROTATE,
    /* 跳过目标时云台已回中；等待 200ms 后通知路线跳至下一个任务。 */

    ACTION_BAD_WAIT_DISTANCE,
    /* arm:6 已触发测距；等待 500ms 后按 TofData/10-2cm 伸臂并闭爪。 */

    ACTION_BAD_WAIT_CLOSE,
    /* 坏果已夹紧；等待 800ms 后开爪，维持原坏果处理的动作节拍。 */

    ACTION_BAD_WAIT_OPEN
    /* 坏果夹爪已打开；等待 800ms 后进入通用放置/复位流程。 */
} ActionState_t;

static ActionState_t s_state;       /* 当前动作阶段 */
static uint32_t s_deadline;         /* 当前阶段最早允许推进的 HAL 时基 */
static uint8_t s_retry_count;       /* 云台到左右极限后的前移次数 */
static bool s_gimbal_moving;        /* 通道7是否正在执行非阻塞匀速轨迹 */
static bool s_gimbal_lift_pending;  /* 云台转动前，是否仍在等待升降台到 10cm */
static float s_gimbal_start_angle;  /* 本次轨迹起始角度 */
static float s_gimbal_target_angle; /* 本次轨迹目标角度 */
static uint32_t s_gimbal_start_tick;/* 本次轨迹起始时刻 */
static uint32_t s_gimbal_duration;  /* 本次轨迹总时长 */
static uint32_t s_gimbal_lift_deadline; /* 升至10cm后允许开始转云台的时刻 */
static bool s_pending_command_valid; /* 忙碌期间缓存的一条关键视觉命令 */
static uint8_t s_pending_command;    /* 仅缓存 arm:0 / arm:5 / arm:6 */

/* 仅在收到命令或切换动作阶段时输出，避免在 20ms Tick 中连续刷屏。 */
static const char *ActionScheduler_StateName(ActionState_t state)
{
    switch (state) {
    case ACTION_IDLE:                return "IDLE";
    case ACTION_GRAB_WAIT_DISTANCE:  return "GRAB_DISTANCE";
    case ACTION_GRAB_WAIT_OPEN:      return "GRAB_OPEN";
    case ACTION_GRAB_WAIT_CLOSE:     return "GRAB_CLOSE";
    case ACTION_GRAB_WAIT_GRIP:      return "GRAB_GRIP";
    case ACTION_PUT_WAIT_LIFT:       return "PUT_LIFT";
    case ACTION_PUT_WAIT_EXTEND:     return "PUT_EXTEND";
    case ACTION_PUT_WAIT_ROTATE:     return "PUT_ROTATE";
    case ACTION_PUT_WAIT_OPEN:       return "PUT_OPEN";
    case ACTION_SKIP_WAIT_LIFT:      return "SKIP_LIFT";
    case ACTION_SKIP_WAIT_EXTEND:    return "SKIP_EXTEND";
    case ACTION_SKIP_WAIT_ROTATE:    return "SKIP_ROTATE";
    case ACTION_BAD_WAIT_DISTANCE:   return "BAD_DISTANCE";
    case ACTION_BAD_WAIT_CLOSE:      return "BAD_CLOSE";
    case ACTION_BAD_WAIT_OPEN:       return "BAD_OPEN";
    default:                         return "UNKNOWN";
    }
}

static void ActionScheduler_Debug(const char *event, uint8_t command)
{
    Vofa_Printf("[ARM_DBG] %s cmd=%u state=%s busy=%u gimbal=%.1f retry=%u updown=%u tof=%.1f\r\n",
                event,
                command,
                ActionScheduler_StateName(s_state),
                ActionScheduler_IsBusy() ? 1U : 0U,
                PCA9685_Get180Angle(7U),
                s_retry_count,
                upordownFlag,
                TofData);
}

/*
 * 通道7每个 Task07 Tick 只更新一次插补目标。
 * 它使用非阻塞的 PCA9685_Set180Angle()，不能使用内部带 vTaskDelay 的 Smooth 接口。
 */
static void ActionScheduler_GimbalTick(void)
{
    uint32_t elapsed;
    float progress;
    float angle;

    if (s_gimbal_lift_pending) {
        if ((int32_t)(HAL_GetTick() - s_gimbal_lift_deadline) < 0) {
            return;
        }
        s_gimbal_lift_pending = false;
        s_gimbal_start_angle = PCA9685_Get180Angle(7U);
        s_gimbal_start_tick = HAL_GetTick();
        s_gimbal_moving = true;
        ActionScheduler_Debug("GIMBAL_LIFT_DONE", 0U);
    }

    if (!s_gimbal_moving) {
        return;
    }

    elapsed = HAL_GetTick() - s_gimbal_start_tick;
    if (elapsed >= s_gimbal_duration) {
        (void)PCA9685_Set180Angle(7U, s_gimbal_target_angle);
        s_gimbal_moving = false;
        ActionScheduler_Debug("GIMBAL_DONE", 0U);
        return;
    }

    progress = (float)elapsed / (float)s_gimbal_duration;
    angle = s_gimbal_start_angle
          + (s_gimbal_target_angle - s_gimbal_start_angle) * progress;
    (void)PCA9685_Set180Angle(7U, angle);
}

static bool ActionScheduler_Expired(void)
{
    /* 有符号相减可正确处理 HAL_GetTick() 32 位回绕。 */
    return ((int32_t)(HAL_GetTick() - s_deadline) >= 0);
}

static void ActionScheduler_SetDeadline(uint32_t delay_ms)
{
    /* 所有动作间隔统一用 HAL 时基描述，绝不能在此调用 osDelay/vTaskDelay。 */
    s_deadline = HAL_GetTick() + delay_ms;
}

void ActionScheduler_SetExtendCm(float distance_cm)
{
    /* 通道6角度与伸出距离线性对应：-80度为 0mm，+25度为 300mm。 */
    float current = PCA9685_Get180Angle(6U);
    float dist_mm = (current - ARM_EXTEND_MIN_ANGLE_DEG)
                  / ARM_EXTEND_TOTAL_RANGE_DEG * ARM_EXTEND_TOTAL_RANGE_MM;
    float target;

    if (dist_mm < 0.0f) {
        /* 读取值受初始化/浮点误差影响时，不能允许计算出负行程。 */
        dist_mm = 0.0f;
    }
    target = ARM_EXTEND_MIN_ANGLE_DEG
           + (dist_mm + distance_cm * 10.0f) / ARM_EXTEND_TOTAL_RANGE_MM
           * ARM_EXTEND_TOTAL_RANGE_DEG;
    if (target > ARM_EXTEND_MAX_ANGLE_DEG) {
        /* 目标距离过远时卡在机械最大伸出角，保护机构。 */
        target = ARM_EXTEND_MAX_ANGLE_DEG;
    } else if (target < ARM_EXTEND_MIN_ANGLE_DEG) {
        /* 目标距离为负或过小时卡在完全收回角。 */
        target = ARM_EXTEND_MIN_ANGLE_DEG;
    }
    /* 此处只下发一个目标角，不调用带 osDelay 的 Smooth 接口。 */
    (void)PCA9685_Set180Angle(6U, target);
}

static void ActionScheduler_StartPut(ActionState_t first_state)
{
    /*
     * 放置顺序必须是：先收缩臂 -> 再抬升10cm -> 最后转云台。
     * 收缩臂命令先下发，等待其完成后才允许升降台动作。
     */
    (void)PCA9685_Set180Angle(6U, -80.0f);
    s_state = first_state;
    ActionScheduler_SetDeadline(ARM_RETRACT_SETTLE_MS);
}

static void ActionScheduler_StartSkip(void)
{
    /* 跳过目标也遵循“先收臂、再抬升、后转云台”的机械安全顺序。 */
    (void)PCA9685_Set180Angle(6U, -80.0f);
    s_state = ACTION_SKIP_WAIT_EXTEND;
    ActionScheduler_SetDeadline(ARM_RETRACT_SETTLE_MS);
}

void ActionScheduler_Init(void)
{
    /* 初始化不操作硬件，仅清除上一次动作的软件状态。 */
    s_state = ACTION_IDLE;
    s_deadline = 0U;
    s_retry_count = 0U;
    s_gimbal_moving = false;
    s_gimbal_lift_pending = false;
    s_gimbal_start_angle = 0.0f;
    s_gimbal_target_angle = 0.0f;
    s_gimbal_start_tick = 0U;
    s_gimbal_duration = 0U;
    s_gimbal_lift_deadline = 0U;
    s_pending_command_valid = false;
    s_pending_command = 0U;
}

bool ActionScheduler_IsBusy(void)
{
    /* IDLE 以外均表示存在等待时间或后续机械动作。 */
    return (s_state != ACTION_IDLE) || s_gimbal_moving || s_gimbal_lift_pending;
}

bool ActionScheduler_IsGimbalBusy(void)
{
    return s_gimbal_moving || s_gimbal_lift_pending;
}

void ActionScheduler_Cancel(void)
{
    /* 急停只阻止后续状态推进，已下发到舵机的最后位置保持不变。 */
    s_state = ACTION_IDLE;
    s_gimbal_moving = false;
    s_gimbal_lift_pending = false;
    s_pending_command_valid = false;
}

/**
 * @brief  启动云台运动及关联的防碰撞安全抬升逻辑
 * @param  target_angle_deg 目标角度
 * @param  duration_ms      转动耗时，0 表示瞬发
 * @param  lift_before_move 是否需要安全抬升（大范围转场设 true，视觉微调设 false）
 * 
 * @note   关键状态机标志位说明：
 *         - s_gimbal_lift_pending: 等待升降机到达安全高度（云台死锁禁止转动）
 *         - s_gimbal_moving: 升降完成/无需升降，云台正在平滑插补转动中
 */
static void ActionScheduler_StartGimbalMoveInternal(float target_angle_deg, uint32_t duration_ms, bool lift_before_move)
{
    s_gimbal_target_angle = target_angle_deg;
    s_gimbal_duration = duration_ms;

    /* 第一步：防碰撞安全抬升。仅大范围转场需要先升至 10cm，视觉微调保持原高度。 */
    if (lift_before_move && (now_pos < 9.9f || now_pos > 10.1f)) {
        Move_Pos(25.0f);
        osDelay(500U);
        s_gimbal_lift_deadline = HAL_GetTick() + ARM_PUT_LIFT_SETTLE_MS;
        s_gimbal_lift_pending = true;
        s_gimbal_moving = false;
        ActionScheduler_Debug("GIMBAL_LIFT_START", 0U);
        return;
    }

    /* 调用方已经完成升至10cm的等待，可立即开始本次云台插补。 */
    s_gimbal_start_angle = PCA9685_Get180Angle(7U);
    s_gimbal_start_tick = HAL_GetTick();
    if (duration_ms == 0U || s_gimbal_start_angle == s_gimbal_target_angle) {
        (void)PCA9685_Set180Angle(7U, target_angle_deg);
        s_gimbal_moving = false;
        s_gimbal_lift_pending = false;
    } else {
        s_gimbal_lift_pending = false;
        s_gimbal_moving = true;
        // ActionScheduler_Debug("GIMBAL_START", 0U);
    }
}

void ActionScheduler_StartGimbalMove(float target_angle_deg, uint32_t duration_ms)
{
    /* 对外接口维持原安全约束：转云台前先升至 10cm。 */
    ActionScheduler_StartGimbalMoveInternal(target_angle_deg, duration_ms, true);
}

void ActionScheduler_RequestVisionArm(uint8_t command)
{
    float gimbal_angle;

    ActionScheduler_Debug("RX", command);
    if (ActionScheduler_IsBusy()) {
        /*
         * 对准微调命令可由相机下一帧重发；抓取/跳过/坏果命令不能丢失。
         * 缓存只保留最新一条关键命令，当前动作结束后自动执行。
         */
        if (command == 0U || command == 5U || command == 6U) {
            s_pending_command = command;
            s_pending_command_valid = true;
            ActionScheduler_Debug("QUEUE_BUSY", command);
        } else {
            ActionScheduler_Debug("DROP_BUSY", command);
        }
        return;
    }

    gimbal_angle = PCA9685_Get180Angle(7U);
    if (command == 1U || command == 2U) {
        /* 1/2 仅做视觉对准；到极限后以底盘前移重新获得视野。 */
        if ((command == 1U && gimbal_angle <= -90.0f) ||
            (command == 2U && gimbal_angle >= 90.0f)) {
            s_retry_count++;
            ActionScheduler_Debug("GIMBAL_LIMIT", command);
            if (s_retry_count >= 1000U) {
                /* 连续五次撞到云台极限仍未对准，按 arm:5 流程放弃当前果实。 */
                if (upordownFlag == 0U) {
                    ActionScheduler_StartSkip();
                    ActionScheduler_Debug("GIVEUP_SKIP", command);
                } else {
                    ActionScheduler_Debug("GIVEUP_TREE", command);
                    App_NotifyGrabDone();
                }
            } else {
                /* 云台已到机械极限时，底盘前移 100mm 后等待相机重新反馈。 */
                Emm_V5_Chassis_Pos_Control(1, 50, 20, 10.0f);
                ActionScheduler_Debug("CHASSIS_FORWARD", command);
            }
        } else {
            /* 未到极限时每次只微调 1度，避免单次转动造成目标丢失。 */
            /* 视觉对准阶段固定在抓取高度，不能伪造“已升到10cm”的 now_pos。 */
            ActionScheduler_StartGimbalMoveInternal(gimbal_angle + ((command == 1U) ? -1.0f : 1.0f), 300U, false);
            ActionScheduler_Debug("GIMBAL_STEP", command);
        }
    } else if (command == 3U) {
        /* 目标偏上：升降机构上移 1cm；本命令不进入长动作序列。 */
        Move_up(1.0f);
    } else if (command == 4U) {
        /* 目标偏下：升降机构下移 1cm；本命令不进入长动作序列。 */
        Move_down(1.0f);
    } else if (command == 0U) {
        /* 正常抓取：先触发测距，100ms 后读取 TofData 计算伸臂量。 */
        s_retry_count = 0U;
        if (upordownFlag != 0U) {
            /* 树上果当前不执行地面抓取动作，直接让路线继续。 */
            App_NotifyGrabDone();
            return;
        }
        get_dis();
        s_state = ACTION_GRAB_WAIT_DISTANCE;
        ActionScheduler_SetDeadline(ARM_TOF_SETTLE_MS);
        ActionScheduler_Debug("GRAB_START", command);
    } else if (command == 5U) {
        /* 跳过目标：升至安全高度后收臂、云台回中，再通知路线继续。 */
        s_retry_count = 0U;
        if (upordownFlag == 0U) {
            ActionScheduler_StartSkip();
            ActionScheduler_Debug("SKIP_START", command);
        } else {
            /* 树上果跳过不需要移动地面升降/伸缩机构。 */
            App_NotifyGrabDone();
        }
    } else if (command == 6U) {
        /* 坏果清理沿用放置流程，但伸臂距离比正常抓取少 1cm。 */
        if (upordownFlag != 0U) {
            /* 树上坏果同样不进入本地地面抓取机构流程。 */
            App_NotifyGrabDone();
            return;
        }
        get_dis();
        s_state = ACTION_BAD_WAIT_DISTANCE;
        ActionScheduler_SetDeadline(ARM_BAD_TOF_SETTLE_MS);
        ActionScheduler_Debug("BAD_START", command);
    }
}

void ActionScheduler_Tick(void)
{
    /* 云台匀速轨迹与抓取状态机并行推进，二者均不会阻塞当前任务。 */
    ActionScheduler_GimbalTick();

    /* 云台/抓取空闲后优先执行缓存的 arm:0/5/6，不再依赖 K230 重发。 */
    if (s_pending_command_valid && !ActionScheduler_IsBusy()) {
        uint8_t command = s_pending_command;
        s_pending_command_valid = false;
        ActionScheduler_Debug("DEQUEUE", command);
        ActionScheduler_RequestVisionArm(command);
        return;
    }

    /* 未到本阶段截止时间时不执行 I2C/电机命令，保持 Task07 响应性。 */
    if (!ActionScheduler_Expired()) {
        return;
    }

    switch (s_state) {
    case ACTION_GRAB_WAIT_DISTANCE:
        /* 测距稳定后开爪，等待爪子张开。 */
        (void)PCA9685_Set180Angle(5U, -30.0f);
        s_state = ACTION_GRAB_WAIT_OPEN;
        ActionScheduler_SetDeadline(ARM_CLAW_OPEN_MS);
        ActionScheduler_Debug("GRAB_OPEN", 0U);
        break;
    case ACTION_GRAB_WAIT_OPEN:
        /* 以当前测距值计算伸臂目标，随后短暂等待机构开始运动。 */
        ActionScheduler_SetExtendCm(TofData / 10.0f + 5.0f);
        s_state = ACTION_GRAB_WAIT_CLOSE;
        ActionScheduler_SetDeadline(ARM_EXTEND_SETTLE_MS);
        ActionScheduler_Debug("GRAB_EXTEND", 0U);
        break;
    case ACTION_GRAB_WAIT_CLOSE:
        /* 到达目标距离后夹紧，保留原流程的抓稳时间。 */
        (void)PCA9685_Set180Angle(5U, 3.0f);
        s_state = ACTION_GRAB_WAIT_GRIP;
        ActionScheduler_SetDeadline(ARM_CLAW_CLOSE_MS);
        ActionScheduler_Debug("GRAB_CLOSE", 0U);
        break;
    case ACTION_GRAB_WAIT_GRIP:
        /* 果实夹紧并抓稳后，先收回伸缩臂。 */
        ActionScheduler_StartPut(ACTION_PUT_WAIT_EXTEND);
        ActionScheduler_Debug("PUT_START", 0U);
        break;
    case ACTION_PUT_WAIT_EXTEND:
        /* 伸缩臂完全收回后，升降台抬升到10cm。 */
        Move_Pos(25.0f);
        s_state = ACTION_PUT_WAIT_LIFT;
        ActionScheduler_SetDeadline(ARM_PUT_LIFT_SETTLE_MS);
        ActionScheduler_Debug("PUT_LIFT", 0U);
        break;
    case ACTION_PUT_WAIT_LIFT:
        /* 升降台到达10cm后，云台回到中位。时长按当前角度比例计算，对齐航线摆动速度。 */
        {
            float delta = fabsf(PCA9685_Get180Angle(7U));
            uint32_t dur = (uint32_t)(delta * 13.0f);
            if (dur < 300U) dur = 300U;
            ActionScheduler_StartGimbalMoveInternal(0.0f, dur, false);
        }
        s_state = ACTION_PUT_WAIT_ROTATE;
        ActionScheduler_SetDeadline(0U);
        ActionScheduler_Debug("PUT_CENTER", 0U);
        break;
    case ACTION_PUT_WAIT_ROTATE:
        /* 云台插补中则等待；插补结束后额外等 ARM_GIMBAL_SETTLE_MS 停稳再开爪 */
        if (ActionScheduler_IsGimbalBusy()) {
            return;
        }
        /* 首次不忙时设停稳截止，下个 Tick 再放行 */
        if (s_deadline == 0U) {
            ActionScheduler_SetDeadline(ARM_GIMBAL_SETTLE_MS);
            return;
        }
        (void)PCA9685_Set180Angle(5U, -30.0f);
        s_state = ACTION_PUT_WAIT_OPEN;
        ActionScheduler_SetDeadline(ARM_CLAW_RELEASE_MS);
        ActionScheduler_Debug("PUT_RELEASE", 0U);
        break;
    case ACTION_PUT_WAIT_OPEN:
        /* 完整抓取/放置结束，通知路线状态机发送下一次任务。 */
        s_state = ACTION_IDLE;
        ActionScheduler_Debug("DONE", 0U);
        App_NotifyGrabDone();
        break;
    case ACTION_SKIP_WAIT_LIFT:
        /* 已升到10cm，云台现在才允许回中。时长按当前角度比例计算。 */
        {
            float delta = fabsf(PCA9685_Get180Angle(7U));
            uint32_t dur = (uint32_t)(delta * 13.0f);
            if (dur < 300U) dur = 300U;
            ActionScheduler_StartGimbalMove(0.0f, dur);
        }
        s_state = ACTION_SKIP_WAIT_ROTATE;
        ActionScheduler_SetDeadline(0U);
        ActionScheduler_Debug("SKIP_CENTER", 5U);
        break;
    case ACTION_SKIP_WAIT_EXTEND:
        /* 跳过目标时先收臂完成，再升到10cm，最后才转云台。 */
        Move_Pos(25.0f);
        s_state = ACTION_SKIP_WAIT_LIFT;
        ActionScheduler_SetDeadline(ARM_PUT_LIFT_SETTLE_MS);
        ActionScheduler_Debug("SKIP_LIFT", 5U);
        break;
    case ACTION_SKIP_WAIT_ROTATE:
        /* 跳过动作完整结束；路线状态机的 s_grab_done 将被置位。 */
        if (ActionScheduler_IsGimbalBusy()) {
            return;
        }
        if (s_deadline == 0U) {
            ActionScheduler_SetDeadline(ARM_GIMBAL_SETTLE_MS);
            return;
        }
        s_state = ACTION_IDLE;
        ActionScheduler_Debug("SKIP_DONE", 5U);
        App_NotifyGrabDone();
        break;
    case ACTION_BAD_WAIT_DISTANCE:
        /* 坏果与正常果的差别是目标伸臂量少 1cm，后续均复用放置流程。 */
        ActionScheduler_SetExtendCm(TofData / 10.0f + 5.0f);
        (void)PCA9685_Set180Angle(5U, 3.0f);
        s_state = ACTION_BAD_WAIT_CLOSE;
        ActionScheduler_SetDeadline(ARM_CLAW_CLOSE_MS);
        ActionScheduler_Debug("BAD_GRIP", 6U);
        break;
    case ACTION_BAD_WAIT_CLOSE:
        /* 坏果夹紧保持一段时间后开爪，执行原有清理节拍。 */
        (void)PCA9685_Set180Angle(5U, -30.0f);
        s_state = ACTION_BAD_WAIT_OPEN;
        ActionScheduler_SetDeadline(ARM_BAD_RELEASE_SETTLE_MS);
        ActionScheduler_Debug("BAD_RELEASE", 6U);
        break;
    case ACTION_BAD_WAIT_OPEN:
        /* 开爪等待结束后抬升，并转入通用收臂、回中、开爪流程。 */
        ActionScheduler_StartPut(ACTION_PUT_WAIT_EXTEND);
        ActionScheduler_Debug("BAD_PUT", 6U);
        break;
    default:
        break;
    }
}
