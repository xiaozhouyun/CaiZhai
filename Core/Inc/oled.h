#ifndef __OLED_H
#define __OLED_H

#include "main.h"

#define OLED_Address      0x78
#define OLED_Cmd_Address  0x00
#define OLED_Data_Address 0x40
#define OLED_OK           0U
#define OLED_ERROR        1U

void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
uint8_t OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
void OLED_Fill(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size2, uint8_t point);
void OLED_ShowString(uint8_t x, uint8_t y, char *p, uint8_t Char_Size);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no);
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t BMP[]);
void fill_picture(uint8_t fill_Data);
void OLED_Refresh_Gram(void);
void Boot_Animation(void);
void OLED_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t len, uint8_t size2);
void OLED_Write_Command(uint8_t IIC_Command);
void OLED_Write_Data(uint8_t IIC_Data);

#endif
