#include "voice.h"

/* JQ8x00 语音模块接在 USART3 上，这里直接复用 CubeMX 生成的串口句柄。 */
extern UART_HandleTypeDef huart3;

/*
 * FLASH 内音频文件路径编号。
 * 例如 "/00001" 表示模块 FLASH 中编号为 00001 的音频文件。
 */
char num1[] = "/00001";
char num2[] = "/00002";
char num3[] = "/00003";
char num4[] = "/00004";
char num5[] = "/00005";
char num6[] = "/00006";
char num7[] = "/00007";
char num8[] = "/00008";
char num9[] = "/00009";
char num10[] = "/00010";
char Apple[] = "/00011";
char Pear[] = "/00012";
char Pumpkin[] = "/00013";
char Pepper[] = "/00014";
char Tomato[] = "/00015";
char Eggplant[] = "/00016";
char Teaminfor[] = "/00017";
char Mature[] = "/00018";
char Inmature[] = "/00019";
char Bad[] = "/00020";
char Music[] = "/00099";

/* 二维码位置 1~12 对应语音文件 00031~00042。 */
static char position_voice_paths[12][7] = {
    "/00031", "/00032", "/00033", "/00034",
    "/00035", "/00036", "/00037", "/00038",
    "/00039", "/00040", "/00041", "/00042"
};

/* 数字音频索引表，下标 1~10 分别对应 num1~num10；下标 0 不播放数字。 */
static char *numArray[] = {0, num1, num2, num3, num4, num5, num6, num7, num8, num9, num10};

/**
  * @brief  通过 USART3 向 JQ8x00 语音模块发送一帧数据
  * @param  data 待发送的数据缓冲区
  * @param  len  数据长度，单位字节
  */
static void JQ8x00_UART(uint8_t *data, uint16_t len)
{
    (void)HAL_UART_Transmit(&huart3, data, len, HAL_MAX_DELAY);
}

/**
  * @brief  播放“水果名称 + 数量”的组合语音
  * @param  quantity  数量，只支持 1~10
  * @param  Fruitname 水果名称音频路径，例如 Apple、Pear
  * @note   超出 1~10 的数量会被忽略，避免访问 numArray 越界。
  */
void playFruitSound(int quantity, char *Fruitname)
{
    if (quantity >= 1 && quantity <= 10)
    {
        JQ8x00_RandomPathPlay(JQ8X00_FLASH, Fruitname);
        HAL_Delay(1000);
        JQ8x00_RandomPathPlay(JQ8X00_FLASH, numArray[quantity]);
        HAL_Delay(1500);
    }
}

/**
  * @brief  发送不带数据参数的 JQ8x00 控制命令
  * @param  Command 命令字，取值见 UartCommand
  * @note   帧格式为 AA + 命令 + 数据长度 00 + 校验和。
  */
void JQ8x00_Command(UartCommand Command)
{
    uint8_t Buffer[4] = {0xAA, 0, 0, 0};

    Buffer[1] = (uint8_t)Command;
    Buffer[2] = 0x00;
    Buffer[3] = Buffer[0] + Buffer[1] + Buffer[2];

    JQ8x00_UART(Buffer, sizeof(Buffer));
}

/**
  * @brief  发送带数据参数的 JQ8x00 控制命令
  * @param  Command 命令字，取值见 UartCommandData
  * @param  DATA    命令参数
  * @note   大多数命令使用 1 字节参数，指定曲目/次数/快进快退类命令使用 2 字节参数。
  */
void JQ8x00_Command_Data(UartCommandData Command, uint8_t DATA)
{
    uint8_t Buffer[6] = {0xAA, 0, 0, 0, 0, 0};
    uint8_t DataLen = 0;

    Buffer[1] = (uint8_t)Command;
    if ((Command != AppointTrack) && (Command != SetCycleCount) &&
        (Command != SelectTrackNoPlay) && (Command != AppointTimeBack) &&
        (Command != AppointTimeFast))
    {
        Buffer[2] = 1;
        Buffer[3] = DATA;
        Buffer[4] = Buffer[0] + Buffer[1] + Buffer[2] + Buffer[3];
        DataLen = 5;
    }
    else
    {
        Buffer[2] = 2;
        Buffer[3] = DATA / 256;
        Buffer[4] = DATA % 256;
        Buffer[5] = Buffer[0] + Buffer[1] + Buffer[2] + Buffer[3] + Buffer[4];
        DataLen = 6;
    }

    JQ8x00_UART(Buffer, DataLen);
}

/**
  * @brief  按路径播放指定存储介质中的音频文件
  * @param  command  路径播放命令，0x08 为随机路径播放，0x17 为插播
  * @param  symbol   存储介质，当前主要使用 JQ8X00_FLASH
  * @param  DATA     6 字节路径字符串，例如 "/00011"
  * @note   发送内容会补上 "*.???"，让模块按路径匹配音频文件。
  */
static void JQ8x00_PathPlay(uint8_t command, JQ8X00_Symbol symbol, char *DATA)
{
    uint8_t Buffer[52] = {0xAA, 0};
    uint8_t i = 4;
    uint8_t j;
    uint8_t k;

    Buffer[1] = command;
    Buffer[2] = 11;
    Buffer[3] = (uint8_t)symbol;

    for (k = 0; k < 6; k++)
    {
        Buffer[i++] = (uint8_t)DATA[k];
    }

    Buffer[i++] = '*';
    Buffer[i++] = '?';
    Buffer[i++] = '?';
    Buffer[i++] = '?';

    for (j = 0; j < i; j++)
    {
        Buffer[i] = Buffer[i] + Buffer[j];
    }

    i++;
    JQ8x00_UART(Buffer, i);
}

/**
  * @brief  随机路径播放指定音频
  * @param  symbol 存储介质
  * @param  DATA   6 字节路径字符串，例如 "/00011"
  */
void JQ8x00_RandomPathPlay(JQ8X00_Symbol symbol, char *DATA)
{
    JQ8x00_PathPlay(0x08, symbol, DATA);
}

/**
  * @brief  插播指定路径音频
  * @param  symbol 存储介质
  * @param  DATA   6 字节路径字符串，例如 "/00011"
  */
void JQ8x00_RandomPathPlay_Inserch(JQ8X00_Symbol symbol, char *DATA)
{
    JQ8x00_PathPlay(0x17, symbol, DATA);
    HAL_Delay(10);
}

/**
  * @brief  设置语音模块音量
  * @param  vol 音量值，范围由 JQ8x00 模块固件定义
  */
void JQ8x00_Volumn(uint8_t vol)
{
    JQ8x00_Command_Data(SetVolume, vol);
}

/**
  * @brief  结束当前播放
  */
void JQ8x00_Over(void)
{
    uint8_t Buffer[4] = {0xAA, 0x10, 0x00, 0xBA};
    JQ8x00_UART(Buffer, sizeof(Buffer));
}

/**
  * @brief  发送 0x03 控制命令
  */
void JQ8x00_Stop(void)
{
    uint8_t Buffer[4] = {0xAA, 0x03, 0x00, 0xAD};
    JQ8x00_UART(Buffer, sizeof(Buffer));
}

/**
  * @brief  发送 0x04 控制命令
  */
void JQ8x00_Hold(void)
{
    uint8_t Buffer[4] = {0xAA, 0x04, 0x00, 0xAE};
    JQ8x00_UART(Buffer, sizeof(Buffer));
}

/**
  * @brief  设置循环模式
  * @note   当前固定发送 0x18 命令和 0x02 参数，按模块协议表示对应循环模式。
  */
void JQ8x00_Loop(void)
{
    uint8_t Buffer[5] = {0xAA, 0x18, 0x01, 0x02, 0xC5};
    JQ8x00_UART(Buffer, sizeof(Buffer));
}

/**
  * @brief  设置循环次数为 2 次
  */
void JQ8x00_Loop_times2(void)
{
    uint8_t Buffer[6] = {0xAA, 0x19, 0x02, 0x00, 0x02, 0xC7};
    JQ8x00_UART(Buffer, sizeof(Buffer));
}

/**
  * @brief  按编号播放预设语音
  * @param  Num 语音编号：1~10 为数字，11~20 为水果/状态，31~42 为二维码位置，99 为音乐
  * @note   Num 为 0 时结束播放；其他未定义编号会被忽略。
  */
void Voice_Num(int Num)
{
    if ((Num >= 31) && (Num <= 42))
    {
        JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH,
                                      position_voice_paths[Num - 31]);
        return;
    }

    switch (Num)
    {
        case 0:  JQ8x00_Over(); break;
        case 1:  JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num1); break;
        case 2:  JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num2); break;
        case 3:  JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num3); break;
        case 4:  JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num4); break;
        case 5:  JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num5); break;
        case 6:  JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num6); break;
        case 7:  JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num7); break;
        case 8:  JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num8); break;
        case 9:  JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num9); break;
        case 10: JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num10); break;
        case 11: JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Apple); break;
        case 12: JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Pear); break;
        case 13: JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Pumpkin); break;
        case 14: JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Pepper); break;
        case 15: JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Tomato); break;
        case 16: JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Eggplant); break;
        case 17: JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Teaminfor); break;
        case 18: JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Mature); break;
        case 19: JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Inmature); break;
        case 20: JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Bad); break;
        case 99:
            JQ8x00_Loop();
            JQ8x00_Loop_times2();
            JQ8x00_RandomPathPlay(JQ8X00_FLASH, Music);
            break;
        default:
            break;
    }
}
