#include "tof200f.h"

extern UART_HandleTypeDef huart1;

/*
 * TOF200F 单次测距 Modbus-RTU 命令：
 * 01 03 00 10 00 01 85 CF
 */
static uint8_t tof200f_start_single[] = {0x01, 0x03, 0x00, 0x10, 0x00, 0x01, 0x85, 0xCF};

/* MODBUS 回包格式：01 03 02 [距离高8位] [距离低8位] [CRC低8位] [CRC高8位]，共 7 字节 */
#define TOF200F_RX_FRAME_LEN 7

volatile float TofData = 0.0f;

static uint8_t tof200f_rx_buf[TOF200F_RX_FRAME_LEN];
static uint8_t tof200f_rx_index = 0;

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
 * @brief  初始化 TOF200F 传感器串口中断
 */
void TOF200F_Init(void)
{
    tof200f_rx_index = 0U;
    TofData = 0.0f;

    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

/**
 * @brief  触发一次 TOF200F 测距请求 (增加 50ms 安全超时)
 */
void get_dis(void)
{
    (void)HAL_UART_Transmit(&huart1, tof200f_start_single, sizeof(tof200f_start_single), 50U);
}

/**
 * @brief  串口 1 逐字节中断接收与 MODBUS-RTU 协议状态机解析
 */
void TOF200F_UartRxByte(uint8_t data)
{
    /* 字节 0: 校验地址码 0x01 */
    if (tof200f_rx_index == 0U)
    {
        if (data == 0x01U)
        {
            tof200f_rx_buf[tof200f_rx_index++] = data;
        }
        return;
    }

    /* 字节 1: 校验功能码 0x03 */
    if (tof200f_rx_index == 1U)
    {
        if (data != 0x03U)
        {
            /* 校验失败复位；若当前字节正好是 0x01，则作为新帧头处理 */
            tof200f_rx_index = (data == 0x01U) ? 1U : 0U;
            if (tof200f_rx_index == 1U)
            {
                tof200f_rx_buf[0] = data;
            }
            return;
        }
    }

    /* 字节 2: 校验数据长度 0x02 */
    if (tof200f_rx_index == 2U)
    {
        if (data != 0x02U)
        {
            tof200f_rx_index = (data == 0x01U) ? 1U : 0U;
            if (tof200f_rx_index == 1U)
            {
                tof200f_rx_buf[0] = data;
            }
            return;
        }
    }

    /* 存入数据 */
    tof200f_rx_buf[tof200f_rx_index++] = data;

    /* 接收满 7 字节完备帧 */
    if (tof200f_rx_index >= TOF200F_RX_FRAME_LEN)
    {
        /* 计算 CRC16 校验 */
        uint16_t calc_crc = TOF200F_CalcCRC16(tof200f_rx_buf, 5U);
        uint16_t recv_crc = (uint16_t)tof200f_rx_buf[5] | ((uint16_t)tof200f_rx_buf[6] << 8U);

        if (calc_crc == recv_crc)
        {
            uint16_t raw_dist = ((uint16_t)tof200f_rx_buf[3] << 8U) | (uint16_t)tof200f_rx_buf[4];
            /* 合法测量值判断（0 ~ 2500mm 之间） */
            if (raw_dist > 0U && raw_dist < 2500U)
            {
                TofData = (float)raw_dist;
            }
        }

        tof200f_rx_index = 0U;
    }
}

/**
 * @brief  获取当前传感器测距值（单位：厘米 cm）
 */
float TOF200F_GetDistanceCm(void)
{
    return TofData / 10.0f;
}
