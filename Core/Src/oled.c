#include "oled.h"
#include "oled_font.h"

static uint8_t OLED_GRAM[128][8];
static uint8_t oled_ready;

#define OLED_I2C_DELAY_US 5U

static void OLED_I2C_DelayUs(uint32_t us)
{
    uint32_t start;
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < (cycles_per_us * us))
    {
    }
}

static void OLED_W_SCL(uint8_t value)
{
    HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void OLED_W_SDA(uint8_t value)
{
    HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void OLED_I2C_Init(void)
{
    OLED_W_SCL(1);
    OLED_W_SDA(1);
}

static uint8_t OLED_I2C_Start(void)
{
    OLED_W_SDA(1);
    OLED_W_SCL(1);
    OLED_I2C_DelayUs(OLED_I2C_DELAY_US);
    if ((HAL_GPIO_ReadPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin) != GPIO_PIN_SET) ||
        (HAL_GPIO_ReadPin(OLED_SCL_GPIO_Port, OLED_SCL_Pin) != GPIO_PIN_SET))
    {
        return OLED_ERROR;
    }
    OLED_W_SDA(0);
    OLED_I2C_DelayUs(OLED_I2C_DELAY_US);
    OLED_W_SCL(0);
    OLED_I2C_DelayUs(OLED_I2C_DELAY_US);

    return OLED_OK;
}

static void OLED_I2C_Stop(void)
{
    OLED_W_SDA(0);
    OLED_I2C_DelayUs(OLED_I2C_DELAY_US);
    OLED_W_SCL(1);
    OLED_I2C_DelayUs(OLED_I2C_DELAY_US);
    OLED_W_SDA(1);
    OLED_I2C_DelayUs(OLED_I2C_DELAY_US);
}

static uint8_t OLED_I2C_WaitAck(void)
{
    uint8_t result;

    OLED_W_SDA(1);
    OLED_I2C_DelayUs(OLED_I2C_DELAY_US);
    OLED_W_SCL(1);
    OLED_I2C_DelayUs(OLED_I2C_DELAY_US);
    result = (HAL_GPIO_ReadPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin) == GPIO_PIN_RESET) ? OLED_OK : OLED_ERROR;
    OLED_W_SCL(0);
    OLED_I2C_DelayUs(OLED_I2C_DELAY_US);

    return result;
}

static uint8_t OLED_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        OLED_W_SDA((uint8_t)!!(Byte & (0x80 >> i)));
        OLED_I2C_DelayUs(OLED_I2C_DELAY_US);
        OLED_W_SCL(1);
        OLED_I2C_DelayUs(OLED_I2C_DELAY_US);
        OLED_W_SCL(0);
        OLED_I2C_DelayUs(OLED_I2C_DELAY_US);
    }

    return OLED_I2C_WaitAck();
}

static uint8_t OLED_I2C_Write(uint8_t control, uint8_t value)
{
    if (OLED_I2C_Start() != OLED_OK)
    {
        return OLED_ERROR;
    }

    if ((OLED_I2C_SendByte(OLED_Address) != OLED_OK) ||
        (OLED_I2C_SendByte(control) != OLED_OK) ||
        (OLED_I2C_SendByte(value) != OLED_OK))
    {
        OLED_I2C_Stop();
        return OLED_ERROR;
    }

    OLED_I2C_Stop();
    return OLED_OK;
}

void OLED_Write_Command(uint8_t IIC_Command)
{
    (void)OLED_I2C_Write(OLED_Cmd_Address, IIC_Command);
}

void OLED_Write_Data(uint8_t IIC_Data)
{
    (void)OLED_I2C_Write(OLED_Data_Address, IIC_Data);
}

void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
    if (cmd)
    {
        OLED_Write_Data(dat);
    }
    else
    {
        OLED_Write_Command(dat);
    }
}

void fill_picture(uint8_t fill_Data)
{
    uint8_t m;
    uint8_t n;

    for (m = 0; m < 8; m++)
    {
        OLED_WR_Byte((uint8_t)(0xB0 + m), 0);
        OLED_WR_Byte(0x00, 0);
        OLED_WR_Byte(0x10, 0);
        for (n = 0; n < 128; n++)
        {
            OLED_WR_Byte(fill_Data, 1);
        }
    }
}

void OLED_Set_Pos(uint8_t x, uint8_t y)
{
    OLED_WR_Byte((uint8_t)(0xB0 + y), 0);
    OLED_WR_Byte((uint8_t)(((x & 0xF0) >> 4) | 0x10), 0);
    OLED_WR_Byte((uint8_t)(x & 0x0F), 0);
}

void OLED_Display_On(void)
{
    OLED_WR_Byte(0x8D, 0);
    OLED_WR_Byte(0x14, 0);
    OLED_WR_Byte(0xAF, 0);
}

void OLED_Display_Off(void)
{
    OLED_WR_Byte(0x8D, 0);
    OLED_WR_Byte(0x10, 0);
    OLED_WR_Byte(0xAE, 0);
}

void OLED_Clear(void)
{
    uint8_t i;
    uint8_t n;

    for (n = 0; n < 8; n++)
    {
        for (i = 0; i < 128; i++)
        {
            OLED_GRAM[i][n] = 0;
        }
    }

    OLED_Refresh_Gram();
}

void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    uint8_t pos;
    uint8_t bx;
    uint8_t temp;

    if (x > 127 || y > 63)
    {
        return;
    }

    pos = 7 - y / 8;
    bx = y % 8;
    temp = (uint8_t)(1 << (7 - bx));

    if (t)
    {
        OLED_GRAM[x][pos] |= temp;
    }
    else
    {
        OLED_GRAM[x][pos] &= (uint8_t)~temp;
    }

    OLED_Refresh_Gram();
}

void OLED_Fill(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot)
{
    uint8_t x;
    uint8_t y;

    for (x = x1; x <= x2; x++)
    {
        for (y = y1; y <= y2; y++)
        {
            OLED_DrawPoint(x, y, dot);
        }
    }

    OLED_Refresh_Gram();
}

void OLED_On(void)
{
    uint8_t i;
    uint8_t n;

    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte((uint8_t)(0xB0 + i), 0);
        OLED_WR_Byte(0x00, 0);
        OLED_WR_Byte(0x10, 0);
        for (n = 0; n < 128; n++)
        {
            OLED_WR_Byte(1, 1);
        }
    }
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size)
{
    uint8_t c;
    uint8_t i;

    c = chr - ' ';
    if (x > 127)
    {
        x = 0;
        y = (uint8_t)(y + 2);
    }

    if (Char_Size == 16)
    {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 8; i++)
        {
            OLED_WR_Byte(F8X16[c * 16 + i], 1);
        }
        OLED_Set_Pos(x, (uint8_t)(y + 1));
        for (i = 0; i < 8; i++)
        {
            OLED_WR_Byte(F8X16[c * 16 + i + 8], 1);
        }
    }
    else
    {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 6; i++)
        {
            OLED_WR_Byte(F6x8[c][i], 1);
        }
    }
}

static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;

    while (n--)
    {
        result *= m;
    }

    return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size2, uint8_t point)
{
    uint8_t t;
    uint8_t temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++)
    {
        temp = (uint8_t)((num / oled_pow(10, (uint8_t)(len - t - 1))) % 10);
        if (enshow == 0 && t < len)
        {
            if (temp == 0 && point)
            {
                if (t != len - 1)
                {
                    OLED_ShowChar((uint8_t)(x + (size2 / 2) * t), y, ' ', size2);
                    continue;
                }
            }
            else
            {
                enshow = 1;
            }
        }
        OLED_ShowChar((uint8_t)(x + (size2 / 2) * t), y, (uint8_t)(temp + '0'), size2);
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, char *chr, uint8_t Char_Size)
{
    uint8_t j = 0;

    while (chr[j] != '\0')
    {
        OLED_ShowChar(x, y, (uint8_t)chr[j], Char_Size);
        x = (uint8_t)(x + 8);
        if (x > 120)
        {
            x = 0;
            y = (uint8_t)(y + 2);
        }
        j++;
    }
}

void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no)
{
    uint8_t t;

    OLED_Set_Pos(x, y);
    for (t = 0; t < 16; t++)
    {
        OLED_WR_Byte(Hzk[2 * no][t], 1);
    }

    OLED_Set_Pos(x, (uint8_t)(y + 1));
    for (t = 0; t < 16; t++)
    {
        OLED_WR_Byte(Hzk[2 * no + 1][t], 1);
    }
}

void OLED_Refresh_Gram(void)
{
    uint8_t i;
    uint8_t n;

    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte((uint8_t)(0xB0 + i), 0);
        OLED_WR_Byte(0x00, 0);
        OLED_WR_Byte(0x10, 0);
        for (n = 0; n < 128; n++)
        {
            OLED_WR_Byte(OLED_GRAM[n][i], 1);
        }
    }
}

void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t BMP[])
{
    uint16_t j = 0;
    uint8_t x;
    uint8_t y;

    for (y = y0; y < y1; y++)
    {
        OLED_Set_Pos(x0, y);
        for (x = x0; x < x1; x++)
        {
            OLED_WR_Byte(BMP[j++], 1);
        }
    }
}

uint8_t OLED_Init(void)
{
    static const uint8_t init_commands[] = {
        0xAE, 0x00, 0x10, 0x40, 0xB0, 0x81, 0xFF, 0xA1,
        0xA6, 0xA8, 0x3F, 0xC8, 0xD3, 0x00, 0xD5, 0x80,
        0xD8, 0x05, 0xD9, 0xF1, 0xDA, 0x12, 0xDB, 0x30,
        0x8D, 0x14, 0xAF
    };
    uint8_t i;

    oled_ready = 0U;
    OLED_I2C_Init();
    HAL_Delay(1);

    for (i = 0U; i < (uint8_t)sizeof(init_commands); i++)
    {
        if (OLED_I2C_Write(OLED_Cmd_Address, init_commands[i]) != OLED_OK)
        {
            return OLED_ERROR;
        }
    }

    OLED_Clear();
    oled_ready = 1U;
    return OLED_OK;
}

uint8_t OLED_IsReady(void)
{
    return oled_ready;
}

void Boot_Animation(void)
{
    uint8_t x;
    uint8_t y;

    for (x = 63; x >= 18; x--)
    {
        OLED_DrawPoint((uint8_t)(108 - 0.7f * x), x, 1);
        OLED_DrawPoint((uint8_t)(17 + 0.7f * x), x, 1);
        y = (uint8_t)(64 - x);
        OLED_DrawPoint((uint8_t)(64 - 0.7f * y), y, 1);
        OLED_DrawPoint((uint8_t)(64 + 0.7f * y), y, 1);
        if (x == 18)
        {
            break;
        }
    }

    for (x = 30; x <= 94; x++)
    {
        OLED_DrawPoint((uint8_t)(125 - x), 47, 1);
        OLED_DrawPoint(x, 18, 1);
    }
}

void OLED_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint16_t t;
    int xerr = 0;
    int yerr = 0;
    int delta_x = (int)x2 - (int)x1;
    int delta_y = (int)y2 - (int)y1;
    int distance;
    int incx;
    int incy;
    int uRow = x1;
    int uCol = y1;

    if (delta_x > 0) incx = 1;
    else if (delta_x == 0) incx = 0;
    else { incx = -1; delta_x = -delta_x; }

    if (delta_y > 0) incy = 1;
    else if (delta_y == 0) incy = 0;
    else { incy = -1; delta_y = -delta_y; }

    distance = (delta_x > delta_y) ? delta_x : delta_y;
    for (t = 0; t <= (uint16_t)(distance + 1); t++)
    {
        OLED_DrawPoint((uint8_t)uRow, (uint8_t)uCol, 1);
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}

void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t len, uint8_t size2)
{
    int integer_part = (int)num;
    uint8_t count = 0;
    uint32_t abs_integer;
    uint32_t fraction;

    if (num < 0.0f)
    {
        OLED_ShowString((uint8_t)(x - size2 / 2), y, "-", size2);
        num = -num;
        integer_part = (int)num;
    }

    abs_integer = (uint32_t)integer_part;
    do
    {
        count++;
        abs_integer /= 10;
    } while (abs_integer > 0);

    if (len > count + 1)
    {
        fraction = (uint32_t)((num - (float)integer_part) * (float)oled_pow(10, (uint8_t)(len - count - 1)));
        OLED_ShowNum(x, y, (uint32_t)integer_part, count, size2, 1);
        OLED_ShowString((uint8_t)(x + count * size2 / 2), y, ".", size2);
        OLED_ShowNum((uint8_t)(x + (count + 1) * size2 / 2), y, fraction, (uint8_t)(len - count - 1), size2, 0);
    }
    else
    {
        OLED_ShowNum(x, y, (uint32_t)integer_part, count, size2, 1);
    }
}
