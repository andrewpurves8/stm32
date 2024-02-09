/*
 * 016uart_case.c
 *
 *  Created on: Feb 9, 2024
 *      Author: andrew.purves
 */

#include <stdio.h>
#include <string.h>
#include "stm32l07xx.h"

char *msg[2] = {"abcd123", "AbCdEfG"};

// reply from arduino will be stored here
char rxBuf[1024];

USART_Handle_t usart1Handle;

// this flag indicates reception completion
uint8_t rxCmplt = RESET;

void USART1_Init(void)
{
	usart1Handle.pUSARTx = USART1;
	usart1Handle.config.baud = USART_STD_BAUD_115200;
	usart1Handle.config.hwFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart1Handle.config.mode = USART_MODE_TXRX;
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
	uint32_t count = 0;
	
	RCC_SetSysClk(SYS_CLK_HSI);

	USART1_GPIOInit();
    USART1_Init();

    USART_IrqInterruptConfig(IRQ_NO_USART1, ENABLE);

    USART_PeripheralControl(USART1, ENABLE);

    while (1)
    {
		delay(250);

		// next message index
		count = count % 2;

		// enable the reception in interrupt mode
		while (USART_ReceiveDataIT(&usart1Handle, rxBuf, strlen(msg[count])) != USART_READY);

		// send the msg indexed by count in interrupt mode
		while (USART_SendDataIT(&usart1Handle, (uint8_t*) msg[count], strlen(msg[count])) != USART_READY);

    	// wait until all the bytes are received from the arduino
    	// when all the bytes are received rxCmplt will be SET in application callback
    	while (rxCmplt != SET);

    	// just make sure that last byte should be null otherwise %s fails while printing
    	rxBuf[strlen(msg[count]) + 1] = '\0';

    	// invalidate the flag
    	rxCmplt = RESET;

    	count++;
    }

	return 0;
}

void USART1_IRQHandler(void)
{
	USART_IrqHandling(&usart1Handle);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t appEv)
{
	if (appEv == USART_EVENT_RX_CMPLT)
	{
		rxCmplt = SET;
	}
	else if (appEv == USART_EVENT_TX_CMPLT)
	{
		;
	}
}
