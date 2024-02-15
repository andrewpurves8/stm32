/*
 * ssd1306_test.c
 *
 *  Created on: Feb 12, 2024
 *      Author: andrew.purves
 */

#include "stm32l07xx.h"
#include "ssd1306.h"

int main(void)
{
	RCC_SetSysClk(SYS_CLK_HSI);
	
	SSD1306_Init(SSD1306_NORMAL_DISPLAY);
	// SSD1306_DrawPixel(0, 0, SSD1306_WHITE);
	// SSD1306_DrawPixel(0, 10, SSD1306_WHITE);
	// SSD1306_DrawPixel(20, 0, SSD1306_WHITE);
	// SSD1306_DrawPixel(30, 30, SSD1306_WHITE);
	const char* text = "ABCD EFG HIJKLM NOPQ RSTU VWXYZabc defgh ijklmnop qrs tu vwxy z";
	SSD1306_Print(text, strlen(text), 2);
	SSD1306_Display();

	while (1);
}
