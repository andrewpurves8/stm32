/*
 * ssd1306.h
 *
 *  Created on: Feb 9, 2024
 *      Author: andrew.purves
 */

#ifndef INC_SSD1306_H_
#define INC_SSD1306_H_

#include <stdint.h>

#define SCREEN_WIDTH                            128
#define SCREEN_HEIGHT                           32

#define SSD1306_BLACK                           0
#define SSD1306_WHITE                           1
#define SSD1306_INVERSE                         2

#define SSD1306_NORMAL_DISPLAY                  0xA6
#define SSD1306_INVERSE_DISPLAY                 0xA7

/******************************************************************************************
 *								APIs supported by this driver
 ******************************************************************************************/

void SSD1306_Init(uint8_t displayMode);
void SSD1306_ClearDisplay(void);
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void SSD1306_Display();
void SSD1306_ReadBuffer(uint8_t* buf);
void SSD1306_Print(const char* text, uint32_t length, uint8_t fontSize);

#endif
