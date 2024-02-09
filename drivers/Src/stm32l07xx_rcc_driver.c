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

void RCC_SetSysClk(uint8_t clk)
{
	if (clk > SYS_CLK_PLL)
	{
		return;
	}

	if (clk == SYS_CLK_MSI)
	{
		RCC->CR |= (1 << RCC_CR_MSION);
	}
	else if (clk == SYS_CLK_HSI)
	{
		RCC->CR |= (1 << RCC_CR_HSI16ON);
	}
	else if (clk == SYS_CLK_HSE)
	{
		RCC->CR |= (1 << RCC_CR_HSEON);
	}
	else
	{
		RCC->CR |= (1 << RCC_CR_PLLON);
	}
	RCC->CFGR |= clk;
}

uint32_t RCC_GetPCLK1Value(void)
{
	return RCC_GetPCLKValue(RCC_CFGR_PPRE1);
}

uint32_t RCC_GetPCLK2Value(void)
{
	return RCC_GetPCLKValue(RCC_CFGR_PPRE2);
}

static uint32_t RCC_GetPCLKValue(uint8_t rccCfgrBit)
{
	uint32_t systemClk;

	uint8_t clkSrc, temp, ahbp, apbp;

	clkSrc = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3);

	if (clkSrc == SYS_CLK_HSI)
	{
		systemClk = 16000000;
	}
	else if (clkSrc == SYS_CLK_HSE)
	{
		systemClk = 8000000;
	}
	else
	{
		// no support for MSI or PLL
		systemClk = 0;
	}

	// ahb
	temp = ((RCC->CFGR >> RCC_CFGR_HPRE) & 0xF);
	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}

	// apb1
	temp = ((RCC->CFGR >> rccCfgrBit) & 0x7);
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