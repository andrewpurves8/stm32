/*
 * adc_test.c
 *
 *  Created on: Feb 19, 2024
 *      Author: andrew.purves
 */

#include <stdio.h>
#include <string.h>
#include "stm32l07xx.h"

void ADC_GPIOInits(void)
{
	GPIO_Handle_t i2cPins;

	i2cPins.pGPIOx = GPIOA;
	i2cPins.pinConfig.pinMode = GPIO_MODE_ANALOG;
	i2cPins.pinConfig.pinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&i2cPins);
}

int main(void)
{
	float adcValue = 0.f;
	RCC_SetSysClk(SYS_CLK_HSI);

	// ADC pin inits
	ADC_GPIOInits();

	ADC_Init(GPIO_PIN_NO_4);

	while (1)
	{
		delay(250);
		
		adcValue = ADC_DataRead();
	}
}
