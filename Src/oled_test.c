/*
 * oled_test.c
 *
 *  Created on: Feb 12, 2024
 *      Author: andrew.purves
 */

#include "stm32l07xx.h"
#include "ssd1306.h"

int main(void)
{
	RCC_SetSysClk(SYS_CLK_HSI);
	
	SSD1306_Init();
	SSD1306_Display();
	delay(10);
	uint8_t buffer[128 * ((32 + 7) / 8)];
	SSD1306_ReadBuffer(&buffer);
	SSD1306_DrawPixel(0, 0, SSD1306_WHITE);
	SSD1306_DrawPixel(0, 10, SSD1306_WHITE);
	SSD1306_DrawPixel(20, 0, SSD1306_WHITE);
	SSD1306_DrawPixel(30, 30, SSD1306_WHITE);
	SSD1306_Display();
	delay(10);
	SSD1306_ReadBuffer(&buffer);

	while (1);
}
