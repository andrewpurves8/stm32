/*
 * stm32l07xx_rcc_driver.c
 *
 *  Created on: Feb 6, 2024
 *      Author: andrew.purves
 */

#include "stm32l07xx_rcc_driver.h"

static uint32_t RCC_GetPCLKValue(uint8_t rccCfgrBit);

static uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
static uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};

uint32_t RCC_GetPCLK1Value(void)
{
	return RCC_GetPCLKValue(8);
}

uint32_t RCC_GetPCLK2Value(void)
{
	return RCC_GetPCLKValue(11);
}

static uint32_t RCC_GetPCLKValue(uint8_t rccCfgrBit)
{
	uint32_t systemClk;

	uint8_t clkSrc, temp, ahbp, apbp;

	clkSrc = ((RCC->CFGR >> 2) & 0x3);

	if (clkSrc == 0)
	{
		systemClk = 16000000;
	}
	else if (clkSrc == 1)
	{
		systemClk = 8000000;
	}
	else if (clkSrc == 2)
	{
		// systemClk = RCC_GetPLLOutputClock();
		// TODO: calc PLL clock
		systemClk = 0;
	}

	// ahb
	temp = ((RCC->CFGR >> 4) & 0xF);
	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}

	// apb1
	temp = ((RCC->CFGR >> 8) & 0x7);
	if (temp < 4)
	{
		apbp = 1;
	}
	else
	{
		apbp = APB1_PreScaler[temp - 4];
	}

	return (systemClk / ahbp) / apbp;
}