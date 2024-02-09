/*
 * 015uart_tx.c
 *
 *  Created on: Feb 7, 2024
 *      Author: andrew.purves
 */

#include <stdio.h>
#include <string.h>
#include "stm32l07xx.h"

char msg[1024] = "UART tx testing...\n\r";

USART_Handle_t usart1Handle;

void USART1_Init(void)
{
	usart1Handle.pUSARTx = USART1;
	usart1Handle.config.baud = USART_STD_BAUD_115200;
	usart1Handle.config.hwFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart1Handle.config.mode = USART_MODE_ONLY_TX;
	usart1Handle.config.noOfStopBits = USART_STOPBITS_1;
	usart1Handle.config.wordLength = USART_WORDLEN_8BITS;
	usart1Handle.config.parityControl = USART_PARITY_DISABLE;
	USART_Init(&usart1Handle);
}

void USART1_GPIOInit(void)
{
	GPIO_Handle_t usartGpios;

	usartGpios.pGPIOx = GPIOA;
	usartGpios.pinConfig.pinMode = GPIO_MODE_ALTFN;
	usartGpios.pinConfig.pinOPType = GPIO_OP_TYPE_PP;
	usartGpios.pinConfig.pinPuPdControl = GPIO_PIN_PU;
	usartGpios.pinConfig.pinSpeed = GPIO_SPEED_FAST;
	usartGpios.pinConfig.pinAltFunMode = 4;

	// USART1 TX
	usartGpios.pinConfig.pinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&usartGpios);

	// USART1 RX
	usartGpios.pinConfig.pinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&usartGpios);
}

int main(void)
{
	RCC_SetSysClk(SYS_CLK_HSI);

	USART1_GPIOInit();

    USART1_Init();

    USART_PeripheralControl(USART1, ENABLE);

    while (1)
    {
		delay(250);
		USART_SendData(&usart1Handle, (uint8_t*) msg, strlen(msg));
    }

	return 0;
}
