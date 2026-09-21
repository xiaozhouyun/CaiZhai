#include "tof200f.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

/*
 * 后置 TOF200F（USART1、地址 0x01）单次测距 Modbus-RTU 命令：
 * 01 03 00 10 00 01 85 CF
 */
static uint8_t tof200f_start_single[] = {0x01, 0x03, 0x00, 0x10, 0x00, 0x01, 0x85, 0xCF};

/* MODBUS 回包格式：01 03 02 [距离高8位] [距离低8位] [CRC低8位] [CRC高8位]，共 7 字节 */
#define TOF200F_RX_FRAME_LEN  7
#define TOF200F_REAR_ADDRESS  0x01U
#define TOF200F_FRONT_ADDRESS 0x02U

/* 后置 TOF200F：USART1，Modbus 地址 0x01。 */
volatile float TofData = 0.0f;
/* 有效测距帧序号：只在 TofData 成功更新后递增，不能把错误帧当成新数据。 */
volatile uint32_t TofFrameSeq = 0U;

/* 前置 TOF200F：USART6，Modbus 地址 0x02。 */
volatile float FrontTofData = 0.0f;
volatile uint32_t FrontTofFrameSeq = 0U;

static uint8_t tof200f_rear_rx_buf[TOF200F_RX_FRAME_LEN];
static uint8_t tof200f_rear_rx_index = 0U;
static uint8_t tof200f_front_rx_buf[TOF200F_RX_FRAME_LEN];
static uint8_t tof200f_front_rx_index = 0U;

/**
 * @brief  简易 MODBUS CRC16 校验计算
 */
static uint16_t TOF200F_CalcCRC16(const uint8_t *buffer, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t pos = 0U; pos < len; pos++)
    {
        crc ^= (uint16_t)buffer[pos];
        for (uint8_t i = 8U; i != 0U; i--)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc >>= 1U;
                crc ^= 0xA001U;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

/**
 * @brief  解析一路 TOF200F 主动输出的 Modbus-RTU 距离帧
 * @param  data             当前收到的字节
 * @param  expected_address 本路传感器的 Modbus 地址
 * @param  rx_buf           本路独立的 7 字节接收缓冲区
 * @param  rx_index         本路独立的接收位置
 * @param  distance         本路最新有效距离，单位 mm
 * @param  frame_seq        本路有效帧序号
 */
static void TOF200F_ParseByte(uint8_t data,
                              uint8_t expected_address,
                              uint8_t *rx_buf,
                              uint8_t *rx_index,
                              volatile float *distance,
                              volatile uint32_t *frame_seq)
{
    /* 字节 0：等待本路传感器的 Modbus 地址。 */
    if (*rx_index == 0U)
    {
        if (data == expected_address)
        {
            rx_buf[(*rx_index)++] = data;
        }
        return;
    }

    /* 字节 1：TOF200F 距离帧功能码固定为 0x03。 */
    if ((*rx_index == 1U) && (data != 0x03U))
    {
        *rx_index = (data == expected_address) ? 1U : 0U;
        if (*rx_index == 1U)
        {
            rx_buf[0] = data;
        }
        return;
    }

    /* 字节 2：距离数据长度固定为 2 字节。 */
    if ((*rx_index == 2U) && (data != 0x02U))
    {
        *rx_index = (data == expected_address) ? 1U : 0U;
        if (*rx_index == 1U)
        {
            rx_buf[0] = data;
        }
        return;
    }

    rx_buf[(*rx_index)++] = data;

    if (*rx_index >= TOF200F_RX_FRAME_LEN)
    {
        uint16_t calc_crc = TOF200F_CalcCRC16(rx_buf, 5U);
        uint16_t recv_crc = (uint16_t)rx_buf[5] | ((uint16_t)rx_buf[6] << 8U);

        if (calc_crc == recv_crc)
        {
            uint16_t raw_dist = ((uint16_t)rx_buf[3] << 8U) | (uint16_t)rx_buf[4];

            if ((raw_dist > 0U) && (raw_dist < 4000U))
            {
                *distance = (float)raw_dist;
                (*frame_seq)++;
            }
        }

        *rx_index = 0U;
    }
}

/**
 * @brief  初始化前、后两个 TOF200F 的串口接收中断
 */
void TOF200F_Init(void)
{
    tof200f_rear_rx_index = 0U;
    tof200f_front_rx_index = 0U;
    TofData = 0.0f;
    TofFrameSeq = 0U;
    FrontTofData = 0.0f;
    FrontTofFrameSeq = 0U;

    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
}

/**
 * @brief  通过 USART1 触发一次后置 TOF200F 测距请求（50ms 超时）
 */
void get_dis(void)
{
    (void)HAL_UART_Transmit(&huart1, tof200f_start_single, sizeof(tof200f_start_single), 50U);
}

/**
 * @brief  USART1 后置 TOF200F（地址 0x01）逐字节接收入口
 */
void TOF200F_UartRxByte(uint8_t data)
{
    TOF200F_ParseByte(data, TOF200F_REAR_ADDRESS,
                      tof200f_rear_rx_buf, &tof200f_rear_rx_index,
                      &TofData, &TofFrameSeq);
}

/**
 * @brief  USART6 前置 TOF200F（地址 0x02）逐字节接收入口
 */
void TOF200F_FrontUartRxByte(uint8_t data)
{
    TOF200F_ParseByte(data, TOF200F_FRONT_ADDRESS,
                      tof200f_front_rx_buf, &tof200f_front_rx_index,
                      &FrontTofData, &FrontTofFrameSeq);
}

/**
 * @brief  获取当前传感器测距值（单位：厘米 cm）
 */
float TOF200F_GetDistanceCm(void)
{
    return TofData / 10.0f;
}
