/*
 * 005button_interrupt.c
 *
 *  Created on: Jan 24, 2024
 *      Author: andrew.purves
 */

#include <string.h>
#include "stm32l07xx.h"

#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW

void delay(void)
{
	// this will introduce ~200ms delay when system clock is 16MHz
	for(uint32_t i = 0; i < 250000; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLed, gpioBtn;

	memset(&gpioLed, 0, sizeof(gpioLed));
	memset(&gpioBtn, 0, sizeof(gpioBtn));

	gpioLed.pGPIOx = GPIOA;
	gpioLed.pinConfig.pinNumber = GPIO_PIN_NO_5;
	gpioLed.pinConfig.pinMode = GPIO_MODE_OUT;
	gpioLed.pinConfig.pinSpeed = GPIO_SPEED_LOW;
	gpioLed.pinConfig.pinOPType = GPIO_OP_TYPE_PP;
	gpioLed.pinConfig.pinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&gpioLed);

	gpioBtn.pGPIOx = GPIOC;
	gpioBtn.pinConfig.pinNumber = GPIO_PIN_NO_13;
	gpioBtn.pinConfig.pinMode = GPIO_MODE_IT_FT;
	gpioBtn.pinConfig.pinSpeed = GPIO_SPEED_FAST;
	gpioBtn.pinConfig.pinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&gpioBtn);

	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, RESET);
	//IRQ configurations
	GPIO_IrqPriorityConfig(IRQ_NO_EXTI15_4, NVIC_IRQ_PRI0);
	GPIO_IrqInterruptConfig(IRQ_NO_EXTI15_4, ENABLE);

    while(1);
}


void EXTI4_15_IRQHandler(void)
{
	GPIO_IrqHandling(GPIO_PIN_NO_13); // clear the pending event from exti line
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
   	delay(); // debouncing
}
