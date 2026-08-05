#include "odometer.h"
#include "bujin.h"
#include "navigation.h"

extern UART_HandleTypeDef huart2;

/* 里程计轮子站号地址与参数宏定义 */
#define ODOMETER_LEFT_FRONT_ADDR         3U             /**< 左前轮电机地址 */
#define ODOMETER_RIGHT_FRONT_ADDR        4U             /**< 右前轮电机地址 */
#define ODOMETER_WHEEL_RADIUS_MM         47.5f          /**< 轮子半径：47.5 mm */
#define ODOMETER_PI                      3.1415926f
/* 电机角度（度）到物理位移（毫米）的转换系数：2 * pi * r / 360 */
#define ODOMETER_DEG_TO_MM               (2.0f * ODOMETER_PI * ODOMETER_WHEEL_RADIUS_MM / 360.0f)

/* Emm V5 串口应答帧格式相关常数 */
#define ODOMETER_FRAME_MAX_LEN           8U             /**< 正常回帧长度：8字节 */
#define ODOMETER_POS_CMD                 0x36U          /**< 查询当前角度的命令字 */
#define ODOMETER_FRAME_END               0x6BU          /**< 回帧校验结束字 */
#define ODOMETER_ANGLE_SCALE             (360.0f / 65536.0f) /**< 65536 个数值代表一圈（360度） */


/**
 * @brief 单轮里程计状态结构体
 */
typedef struct {
    uint8_t has_last_angle;    /**< 是否已保存上一次读取的有效角度（初始化标志） */
    uint8_t has_delta;         /**< 是否已成功计算出当前周期的位移偏差量 */
    float last_angle_deg;      /**< 上一次电机的绝对角度值（度） */
    float delta_mm;            /**< 当前计算出来的物理位移偏差量（毫米） */
} OdometerWheelState;

uint8_t odom_rx_buf[ODOMETER_FRAME_MAX_LEN];            /* 串口数据接收缓存 */
float motor_half_delta_mm;                              /* 左右轮中心平均单步位移量 */

static uint8_t odom_rx_len;                             /* 当前缓存内接收字节的计数 */
static OdometerWheelState odom_left;                    /* 左前轮里程计状态 */
static OdometerWheelState odom_right;                   /* 右前轮里程计状态 */

/**
 * @brief 计算单轮在一个采样周期内的相对旋转角度，并转换为毫米位移量
 * @param angle_deg 当前采样的绝对角度值
 * @param wheel 指向该轮里程计状态结构体的指针
 * @return 转换后的位移增量（单位：mm），带正负号（正数前进，负数后退）
 */
static float Odometer_CalcDeltaMm(float angle_deg, OdometerWheelState *wheel)
{
    float delta_deg = angle_deg - wheel->last_angle_deg;

    wheel->last_angle_deg = angle_deg;
    
    /* 解决电机旋转跨越 ±180 度时的跳变问题 */
    if (delta_deg > 180.0f) {
        delta_deg -= 360.0f;
    } else if (delta_deg < -180.0f) {
        delta_deg += 360.0f;
    }
    return delta_deg * ODOMETER_DEG_TO_MM;
}

/**
 * @brief 核心融合更新函数：当左右前轮的一对数据都计算好后，计算底盘的中心位移量并更新导航定位
 */
static void Odometer_UpdateNavigationIfPairReady(void)
{
    /* 只有当左右两轮在当前周期内都有了新数据，才进行融合 */
    if (odom_left.has_delta == 0U || odom_right.has_delta == 0U) {
        return;
    }

    /* 由于左右侧轮的机械安装朝向相反，一侧正转时另一侧呈反转，所以两者的物理前进增量差为：
       (左轮位移 - 右轮位移) / 2 */
    motor_half_delta_mm = (odom_left.delta_mm - odom_right.delta_mm) * 0.5f;
    
    /* 融合完毕后，清除数据就绪标志，等待下一采样周期 */
    odom_left.has_delta = 0U;
    odom_right.has_delta = 0U;
    
    /* 调用导航定位的累加函数更新当前机器人的二维绝对世界坐标（结合陀螺仪读出的 g_robot_pos.yaw） */
    Navigation_UpdateByDelta(motor_half_delta_mm, g_robot_pos.yaw);
}

/**
 * @brief 接收某个站号对应的轮子绝对角度，触发计算和更新
 * @param addr 电机地址
 * @param angle_deg 解析出的角度（单位：度）
 */
static void Odometer_UpdateByAngle(uint8_t addr, float angle_deg)
{
    OdometerWheelState *wheel = 0;

    /* 分类定位到左前或右前轮状态结构体 */
    if (addr == ODOMETER_LEFT_FRONT_ADDR) {
        wheel = &odom_left;
    } else if (addr == ODOMETER_RIGHT_FRONT_ADDR) {
        wheel = &odom_right;
    }
    if (wheel == 0) {
        return;
    }
    
    /* 如果是首次接收该轮的数据，仅初始化 last_angle 并不计算位移，防止启动跳变 */
    if (wheel->has_last_angle == 0U) {
        wheel->last_angle_deg = angle_deg;
        wheel->has_last_angle = 1U;
        return;
    }

    /* 计算此周期内的位移差，置位数据就绪标志，尝试触发导航融合 */
    wheel->delta_mm = Odometer_CalcDeltaMm(angle_deg, wheel);
    wheel->has_delta = 1U;
    Odometer_UpdateNavigationIfPairReady();
}

/**
 * @brief 解析完整的 8 字节串口参数回帧
 *        帧格式：[地址] + 0x36 + [符号(1负0正)] + [4字节大端绝对角度值] + 0x6B
 */
static void Odometer_ParseFrame(void)
{
    uint32_t raw_angle;
    float motor_angle_deg;

    if (odom_rx_len != ODOMETER_FRAME_MAX_LEN) {
        return;
    }

    /* 拼接 4 字节的原始角度累积脉冲值 */
    raw_angle = ((uint32_t)odom_rx_buf[3] << 24) |
                ((uint32_t)odom_rx_buf[4] << 16) |
                ((uint32_t)odom_rx_buf[5] << 8) |
                (uint32_t)odom_rx_buf[6];
                
    /* 将脉冲值缩放到 0~360 度 */
    motor_angle_deg = (float)raw_angle * ODOMETER_ANGLE_SCALE;
    
    /* 处理正负号字节 */
    if (odom_rx_buf[2] != 0U) {
        motor_angle_deg = -motor_angle_deg;
    }

    /* 将解析到的角度提交给该电机的里程计状态更新函数 */
    Odometer_UpdateByAngle(odom_rx_buf[0], motor_angle_deg);
}

/**
 * @brief 里程计初始化
 */
void Odometer_Init(void)
{
    odom_left = (OdometerWheelState){0};
    odom_right = (OdometerWheelState){0};
    odom_rx_len = 0U;
    motor_half_delta_mm = 0.0f;
    
    /* 清除过载错误并开启串口中断 */
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/**
 * @brief 周期性轮询两个前轮的绝对角度数据（通过 RS485 总线轮流请求）
 */
void Odometer_Update(void)
{
    static uint8_t toggle;
    static uint32_t last_poll_time;
    uint32_t now = HAL_GetTick();

    /* 限制最小轮询采样间隔时间 */
    if ((uint32_t)(now - last_poll_time) < 50U) {
        return;
    }
    last_poll_time = now;

    /* 轮询策略：每个周期仅查询一个轮子，交替进行，减少总线拥堵，接收端进行数据匹配对齐 */
    if (toggle == 0U) {
        Emm_V5_Read_Sys_Params(ODOMETER_LEFT_FRONT_ADDR, S_CPOS);
        toggle = 1U;
    } else {
        Emm_V5_Read_Sys_Params(ODOMETER_RIGHT_FRONT_ADDR, S_CPOS);
        toggle = 0U;
    }
}

/**
 * @brief 串口中断字符输入处理状态机，组装完整的反馈数据包并调用解析
 */
void Odometer_UartRxByte(uint8_t data)
{
    /* 接收第 0 字节：判定是否为指定的两个有效电机地址站号 */
    if (odom_rx_len == 0U) {
        if (data == ODOMETER_LEFT_FRONT_ADDR || data == ODOMETER_RIGHT_FRONT_ADDR) {
            odom_rx_buf[odom_rx_len++] = data;
        }
        return;
    }

    /* 接收第 1 字节：判定是否为查询当前角度的命令响应字 */
    if (odom_rx_len == 1U && data != ODOMETER_POS_CMD) {
        odom_rx_len = 0U; /* 命令字不符，直接复位 */
        return;
    }

    /* 接收后续的字节并放入缓存 */
    odom_rx_buf[odom_rx_len++] = data;
    
    /* 达到应答包的最大长度，对包尾的结束字进行核对 */
    if (odom_rx_len >= ODOMETER_FRAME_MAX_LEN) {
        if (odom_rx_buf[ODOMETER_FRAME_MAX_LEN - 1U] == ODOMETER_FRAME_END) {
            Odometer_ParseFrame(); /* 解析并处理整帧数据 */
        }
        odom_rx_len = 0U; /* 完成或出错后均复位计数器 */
    }
}

