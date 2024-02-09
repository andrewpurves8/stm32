/*
 * stm32f07xx.c
 *
 *  Created on: Feb 8, 2024
 *      Author: andrew.purves
 */

#include <stdint.h>

void delay(uint32_t ms)
{
	for (uint32_t i = 0; i < ms * 1333; i++);
}
