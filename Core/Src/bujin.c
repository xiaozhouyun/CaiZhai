#include "bujin.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart2;
extern osMutexId_t usart2TXHandle;

#define BUJIN_PI                  3.1415926f
#define BUJIN_PULSE_PER_REV       3200.0f

/* 位置模式的毫米换算半径，单位 mm；当前按升降机构旧参数 1.02cm 换成 10.2mm。 */
#define BUJIN_WHEEL_RADIUS_MM     85.0f

/* Emm V5 步进驱动器接在 USART2，所有指令最终都从这里发出。 */
static void Emm_Send(const uint8_t *data, uint16_t len)
{
   osStatus_t mutex_status = osOK;
   uint8_t use_mutex = 0;

   if ((usart2TXHandle != NULL) && (osKernelGetState() == osKernelRunning))
   {
       use_mutex = 1;
       mutex_status = osMutexAcquire(usart2TXHandle, osWaitForever);
   }

   if (mutex_status == osOK)
   {
       (void)HAL_UART_Transmit(&huart2, (uint8_t *)data, len, HAL_MAX_DELAY);

       if (use_mutex != 0)
       {
           (void)osMutexRelease(usart2TXHandle);
       }
   }
}

static uint32_t Emm_MmToPulse(float mm)
{
   float pulse;

   if (mm <= 0.0f)
   {
       return 0;
   }

   pulse = mm / (2.0f * BUJIN_PI * BUJIN_WHEEL_RADIUS_MM) * BUJIN_PULSE_PER_REV;
   return (uint32_t)(pulse + 0.5f);
}

static void Emm_V5_Pos_Control_ByPulse(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t pulse, bool raF, bool snF)
{
   /* 帧格式：地址 + 0xFD + 方向 + 速度 + 加速度 + 4 字节脉冲数 + 相对/绝对 + 同步 + 0x6B */
   uint8_t cmd[13] = {
       addr,
       0xFD,
       dir,
       (uint8_t)(vel >> 8),
       (uint8_t)(vel >> 0),
       acc,
       (uint8_t)(pulse >> 24),
       (uint8_t)(pulse >> 16),
       (uint8_t)(pulse >> 8),
       (uint8_t)(pulse >> 0),
       (uint8_t)raF,
       (uint8_t)snF,
       0x6B,
   };

   Emm_Send(cmd, sizeof(cmd));
   osDelay(5);
}

/**
 * @brief  电机使能控制
 * @param  addr  电机地址，旧工程里升降电机使用 5，广播地址常用 0
 * @param  state true 使能电机，false 关闭电机
 * @param  snF   多机同步标志，true 表示等待同步运动命令，false 表示立即执行
 */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
   /* 帧格式：地址 + 0xF3 + 0xAB + 使能状态 + 同步标志 + 校验字节 0x6B */
   uint8_t cmd[6] = {addr, 0xF3, 0xAB, (uint8_t)state, (uint8_t)snF, 0x6B};
   Emm_Send(cmd, sizeof(cmd));
}

/**
 * @brief  立即停止电机
 * @param  addr 电机地址，0 可作为广播地址
 * @param  snF  多机同步标志
 */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
   /* 帧格式：地址 + 0xFE + 0x98 + 同步标志 + 校验字节 0x6B */
   uint8_t cmd[5] = {addr, 0xFE, 0x98, (uint8_t)snF, 0x6B};
   Emm_Send(cmd, sizeof(cmd));
}

/**
 * @brief  速度模式控制
 * @param  addr 电机地址
 * @param  dir  方向，0 为 CW，1 为 CCW
 * @param  vel  速度，单位 RPM
 * @param  acc  加速度，0 表示直接启动，数值越大加减速越快
 * @param  snF  多机同步标志
 */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
   /* 帧格式：地址 + 0xF6 + 方向 + 速度高低字节 + 加速度 + 同步标志 + 0x6B */
   uint8_t cmd[8] = {
       addr,
       0xF6,
       dir,
       (uint8_t)(vel >> 8),
       (uint8_t)(vel >> 0),
       acc,
       (uint8_t)snF,
       0x6B,
   };

   Emm_Send(cmd, sizeof(cmd));
  osDelay(1);
}

/**
 * @brief  位置模式控制
 * @param  addr 电机地址
 * @param  dir  方向，0 为 CW，1 为 CCW
 * @param  vel  速度，单位 RPM
 * @param  acc  加速度，0 表示直接启动
 * @param  mm   移动距离，单位 mm；函数内部换算成脉冲数
 * @param  raF  false 相对运动，true 绝对位置运动
 * @param  snF  多机同步标志，true 时需再发送 Emm_V5_Synchronous_motion()
 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, float mm, bool raF, bool snF)
{
   Emm_V5_Pos_Control_ByPulse(addr, dir, vel, acc, Emm_MmToPulse(mm), raF, snF);
}

/**
 * @brief  修改驱动器控制模式
 * @param  addr      电机地址
 * @param  svF       true 保存到驱动器，false 只临时生效
 * @param  ctrl_mode 控制模式，具体含义看 Emm V5 驱动器参数说明
 */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
   /* 帧格式：地址 + 0x46 + 0x69 + 保存标志 + 控制模式 + 0x6B */
   uint8_t cmd[6] = {addr, 0x46, 0x69, (uint8_t)svF, ctrl_mode, 0x6B};
   Emm_Send(cmd, sizeof(cmd));
}

/**
 * @brief  清除堵转保护状态
 * @param  addr 电机地址
 */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
   /* 帧格式：地址 + 0x0E + 0x52 + 0x6B */
   uint8_t cmd[4] = {addr, 0x0E, 0x52, 0x6B};
   Emm_Send(cmd, sizeof(cmd));
}

/**
 * @brief  触发同步运动
 * @param  addr 电机地址，0 通常用于广播触发所有等待同步的电机
 */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
   /* 先用 snF=true 下发动作，再用本函数发送同步触发命令。 */
   uint8_t cmd[4] = {addr, 0xFF, 0x66, 0x6B};
   Emm_Send(cmd, sizeof(cmd));
   osDelay(1);
}

/**
 * @brief  读取驱动器系统参数
 * @param  addr 电机地址
 * @param  s    要读取的参数类型，对应 bujin.h 里的 SysParams_t
 */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
   uint8_t cmd[4] = {0};
   uint8_t i = 0;

   /* 不同参数对应不同命令码，有些参数需要两个命令字节。 */
   cmd[i++] = addr;
   switch (s)
   {
       case S_VER:   cmd[i++] = 0x1F; break;
       case S_RL:    cmd[i++] = 0x20; break;
       case S_PID:   cmd[i++] = 0x21; break;
       case S_VBUS:  cmd[i++] = 0x24; break;
       case S_CPHA:  cmd[i++] = 0x27; break;
       case S_ENCL:  cmd[i++] = 0x31; break;
       case S_TPOS:  cmd[i++] = 0x33; break;
       case S_VEL:   cmd[i++] = 0x35; break;
       case S_CPOS:  cmd[i++] = 0x36; break;
       case S_PERR:  cmd[i++] = 0x37; break;
       case S_FLAG:  cmd[i++] = 0x3A; break;
       case S_ORG:   cmd[i++] = 0x3B; break;
       case S_Conf:  cmd[i++] = 0x42; cmd[i++] = 0x6C; break;
       case S_State: cmd[i++] = 0x43; cmd[i++] = 0x7A; break;
       default: return;
   }

   cmd[i++] = 0x6B;
   Emm_Send(cmd, i);
}

/**
 * @brief  按角度控制电机相对转动
 * @param  addr  电机地址
 * @param  angle 相对角度，正数按 dir=0，负数按 dir=1
 * @param  vel   速度，单位 RPM
 * @param  acc   加速度
 * @note   这里按 3200 脉冲一圈换算：pulse = 3200 * angle / 360
 */
void motor_to_angle_control(uint8_t addr, float angle, uint16_t vel, uint8_t acc)
{
   uint8_t dir = 0;

   if (angle < 0.0f)
   {
       angle = -angle;
       dir = 1;
   }

   Emm_V5_Pos_Control_ByPulse(addr, dir, vel, acc, (uint32_t)(BUJIN_PULSE_PER_REV * angle / 360.0f), false, false);
}

/* ====== 回零 / 零点管理 ====== */
/**
 * @brief  将当前单圈位置设为回零零点
 * @param  addr 电机地址
 * @param  save true 保存到驱动器，false 仅临时设置
 */
void Emm_V5_Set_Zero(uint8_t addr, bool save)
{
   /* 帧格式：地址 + 0x93 + 0x88 + 存储标志 + 0x6B */
   uint8_t cmd[5] = {addr, 0x93, 0x88, (uint8_t)save, 0x6B};
   Emm_Send(cmd, sizeof(cmd));
}
/**
 * @brief  触发回零
 * @param  addr 电机地址
 * @param  mode 回零模式，见 Emm_V5_Zero_Mode_t
 * @param  snF  多机同步标志，true 表示等待同步触发，false 表示立即执行
 */
void Emm_V5_Trigger_Zero(uint8_t addr, Emm_V5_Zero_Mode_t mode, bool snF)
{
   /* 帧格式：地址 + 0x9A + 回零模式 + 同步标志 + 0x6B */
   uint8_t cmd[5] = {addr, 0x9A, (uint8_t)mode, (uint8_t)snF, 0x6B};
   Emm_Send(cmd, sizeof(cmd));
}

/**
 * @brief  读取原点回零参数
 * @param  addr 电机地址
 * @note   返回帧由 UART 接收处理逻辑解析
 */
void Emm_V5_Read_Zero_Params(uint8_t addr)
{
   uint8_t cmd[3] = {addr, 0x22, 0x6B};
   Emm_Send(cmd, sizeof(cmd));
}

/**
 * @brief  修改原点回零参数
 * @param  addr   电机地址
 * @param  save   true 保存到驱动器，false 仅临时设置
 * @param  params 回零参数，NULL 时不发送命令
 */
void Emm_V5_Modify_Zero_Params(uint8_t addr, bool save, const Emm_V5_Zero_Params_t *params)
{
   uint8_t cmd[20];

   if (params == NULL)
   {
       return;
   }

   /* 帧格式：地址 + 0x4C + 0xAE + 保存标志 + 回零参数 + 0x6B */
   cmd[0] = addr;
   cmd[1] = 0x4C;
   cmd[2] = 0xAE;
   cmd[3] = (uint8_t)save;
   cmd[4] = (uint8_t)params->mode;
   cmd[5] = params->direction;
   cmd[6] = (uint8_t)(params->speed_rpm >> 8);
   cmd[7] = (uint8_t)params->speed_rpm;
   cmd[8] = (uint8_t)(params->timeout_ms >> 24);
   cmd[9] = (uint8_t)(params->timeout_ms >> 16);
   cmd[10] = (uint8_t)(params->timeout_ms >> 8);
   cmd[11] = (uint8_t)params->timeout_ms;
   cmd[12] = (uint8_t)(params->collision_rpm >> 8);
   cmd[13] = (uint8_t)params->collision_rpm;
   cmd[14] = (uint8_t)(params->collision_ma >> 8);
   cmd[15] = (uint8_t)params->collision_ma;
   cmd[16] = (uint8_t)(params->collision_ms >> 8);
   cmd[17] = (uint8_t)params->collision_ms;
   cmd[18] = (uint8_t)params->auto_trigger;
   cmd[19] = 0x6B;
   Emm_Send(cmd, sizeof(cmd));
}

/**
 * @brief  读取回零状态标志位
 * @param  addr 电机地址
 * @note   返回帧由 UART 接收处理逻辑解析
 */
void Emm_V5_Read_Zero_Status(uint8_t addr)
{
   uint8_t cmd[3] = {addr, 0x3B, 0x6B};
   Emm_Send(cmd, sizeof(cmd));
}

