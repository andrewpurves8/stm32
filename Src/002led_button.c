/*
 * 002led_button.c
 *
 *  Created on: Jan 19, 2024
 *      Author: andrew.purves
 */


#include "stm32l07xx.h"
#include "stm32l07xx_gpio_driver.h"

#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW

void delay(void)
{
	for (uint32_t i = 0; i < 250000 ; i++);
}


int main(void)
{
	GPIO_Handle_t gpioLed, gpioBtn;

	gpioLed.pGPIOx = GPIOA;
	gpioLed.pinConfig.pinNumber = GPIO_PIN_NO_5;
	gpioLed.pinConfig.pinMode = GPIO_MODE_OUT;
	gpioLed.pinConfig.pinSpeed = GPIO_SPEED_FAST;
	gpioLed.pinConfig.pinOPType = GPIO_OP_TYPE_PP;
	gpioLed.pinConfig.pinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioLed);

	gpioBtn.pGPIOx = GPIOC;
	gpioBtn.pinConfig.pinNumber = GPIO_PIN_NO_13;
	gpioBtn.pinConfig.pinMode = GPIO_MODE_IN;
	gpioBtn.pinConfig.pinSpeed = GPIO_SPEED_FAST;
	gpioBtn.pinConfig.pinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&gpioBtn);

	while(1)
	{
		if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED)
		{
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
			delay();
		}
	}
	return 0;
}
