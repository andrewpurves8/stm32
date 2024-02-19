/*
 * stm32l07xx_adc_driver.c
 *
 *  Created on: Feb 19, 2024
 *      Author: andrew.purves
 */

#include "stm32l07xx_adc_driver.h"

void ADC_PeripheralControl(uint8_t en)
{
	if (en == ENABLE)
	{
		REG_SET_BIT(ADC1->CR, ADC_CR_ADEN);
	}
	else
	{
		REG_SET_BIT(ADC1->CR, ADC_CR_ADDIS);
	}
}

void ADC_PeriClockControl(uint8_t en)
{
	if (en == ENABLE)
	{
		ADC1_PCLK_EN();
	}
	else
	{
		ADC1_PCLK_DI();
	}
}

// void ADC_Init(ADC_Handle_t *pADCHandle)
void ADC_Init(uint8_t pin)
{
	// enable the clock for the ADCx peripheral
	ADC_PeriClockControl(ENABLE);
	ADC_PeripheralControl(ENABLE);
	while (!REG_TEST_BIT(ADC1->ISR, ADC_ISR_ADRDY));

	REG_SET_BIT(ADC1->CHSELR, pin);
}

void ADC_DeInit()
{
	ADC_PeriClockControl(DISABLE);
}

float ADC_DataRead()
{
	static const float divisor = (float) 0xFFF; // assuming default 12 bit resolution

	REG_SET_BIT(ADC1->CR, ADC_CR_ADSTART);
	while (!REG_TEST_BIT(ADC1->ISR, ADC_ISR_EOC));
	REG_SET_BIT(ADC1->ISR, ADC_ISR_EOC);

	uint32_t data = ADC1->DR & 0xFFF;
	return data / divisor;
}
