/*
 * spi_tx_rx_long_string.c
 *
 *  Created on: Feb 20, 2024
 *      Author: andrew.purves
 */

#include <stdio.h>
#include <string.h>
#include "stm32l07xx.h"

static const char* longString = "abcdefghijklmnopqrstuvqxyzABCDEFGHIJKLMNOPQRSTUVQXYZabcdefghijklmnopqrstuvqxyzABCDEFGHIJKLMNOPQRSTUVQXYZabcdefghijklmnopqrstuvqxyzABCDEFGHIJKLMNOPQRSTUVQXYZabcdefghijklmnopqrstuvqxyzABCDEFGHIJKLMNOPQRSTUVQXYZabcdefghijklmnopqrstuvqxyzABCDEFGHIJKLMNOPQRSTUVQXYZabcdefghijklmnopqrstuvqxyzABCDEFGHIJKLMNOPQRSTUVQXYZabcdefghijklmnopqrstuvqxyzABCDEFGHIJKLMNOPQRSTUVQXYZabcdefghijklmnopqrstuvqxyzABCDEFGHIJKLMNOPQRSTUVQXYZabcdefghijklmnopqrstuvqxyzABCDEFGHIJKLMNOPQRSTUVQXYZabcdefghijklmnopqrstuvqxyzABCDEFGHIJKLMNOPQRSTUVQXYZ";
static uint8_t slaveRcvBuf[1000];
static uint8_t masterRcvBuf[1000];
static uint8_t slaveRcvComplete = FALSE;

static SPI_Handle_t spi1Handle;
static SPI_Handle_t spi2Handle;

/*
 * PA6 --> SPI1_MISO
 * PA7 --> SPI1_MOSI
 * PA5 --> SPI1_SCLK
 * PA4 --> SPI1_NSS
 * ALT function mode: 0
 */
void SPI1_GPIOInits(void)
{
	GPIO_Handle_t spiPins;

	spiPins.pGPIOx = GPIOA;
	spiPins.pinConfig.pinMode = GPIO_MODE_ALTFN;
	spiPins.pinConfig.pinAltFunMode = 0;
	spiPins.pinConfig.pinOPType = GPIO_OP_TYPE_PP;
	spiPins.pinConfig.pinPuPdControl = GPIO_NO_PUPD;
	spiPins.pinConfig.pinSpeed = GPIO_SPEED_FAST;

	// SCLK
	spiPins.pinConfig.pinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&spiPins);

	// MOSI
	spiPins.pinConfig.pinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&spiPins);

	// MISO
	spiPins.pinConfig.pinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&spiPins);

	// NSS
	spiPins.pinConfig.pinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&spiPins);
}

void SPI1_Inits(void)
{
	spi1Handle.pSPIx = SPI1;
	spi1Handle.spiConfig.busConfig = SPI_BUS_CONFIG_FD;
	spi1Handle.spiConfig.deviceMode = SPI_DEVICE_MODE_MASTER;
	spi1Handle.spiConfig.sclkSpeed = SPI_SCLK_SPEED_DIV32;
	// spi1Handle.spiConfig.sclkSpeed = SPI_SCLK_SPEED_DIV64;
	spi1Handle.spiConfig.cpol = SPI_CPOL_LOW;
	spi1Handle.spiConfig.cpha = SPI_CPHA_LOW;
	spi1Handle.spiConfig.ssm = SPI_SSM_DI; // hardware slave management enabled for NSS pin
	// spi1Handle.spiConfig.dff = SPI_DFF_8BITS;
	spi1Handle.spiConfig.dff = SPI_DFF_16BITS;

	SPI_Init(&spi1Handle);
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode: 0
 */
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t spiPins;

	spiPins.pGPIOx = GPIOB;
	spiPins.pinConfig.pinMode = GPIO_MODE_ALTFN;
	spiPins.pinConfig.pinAltFunMode = 0;
	spiPins.pinConfig.pinOPType = GPIO_OP_TYPE_PP;
	spiPins.pinConfig.pinPuPdControl = GPIO_NO_PUPD;
	spiPins.pinConfig.pinSpeed = GPIO_SPEED_FAST;

	// SCLK
	spiPins.pinConfig.pinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&spiPins);

	// MOSI
	spiPins.pinConfig.pinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&spiPins);

	// MISO
	spiPins.pinConfig.pinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&spiPins);

	// NSS
	spiPins.pinConfig.pinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&spiPins);
}

void SPI2_Inits(void)
{
	spi2Handle.pSPIx = SPI2;
	spi2Handle.spiConfig.busConfig = SPI_BUS_CONFIG_FD;
	spi2Handle.spiConfig.deviceMode = SPI_DEVICE_MODE_SLAVE;
	spi2Handle.spiConfig.sclkSpeed = SPI_SCLK_SPEED_DIV32;
	// spi2Handle.spiConfig.sclkSpeed = SPI_SCLK_SPEED_DIV64;
	spi2Handle.spiConfig.cpol = SPI_CPOL_LOW;
	spi2Handle.spiConfig.cpha = SPI_CPHA_LOW;
	spi2Handle.spiConfig.ssm = SPI_SSM_DI; // hardware slave management enabled for NSS pin
	// spi2Handle.spiConfig.dff = SPI_DFF_8BITS;
	spi2Handle.spiConfig.dff = SPI_DFF_16BITS;

	SPI_Init(&spi2Handle);
}

int main(void)
{
	uint32_t dataLen = strlen(longString);

	RCC_SetSysClk(SYS_CLK_HSI);

	SPI1_GPIOInits();
	SPI2_GPIOInits();

	SPI1_Inits();
	SPI2_Inits();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1, NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SsoeConfig(SPI1, ENABLE);

	SPI_IrqInterruptConfig(IRQ_NO_SPI1, ENABLE);
	SPI_IrqInterruptConfig(IRQ_NO_SPI2, ENABLE);

	SPI_PeripheralControl(SPI1, ENABLE);
	SPI_PeripheralControl(SPI2, ENABLE);

	while (1)
	{
		delay(250);

		while (SPI_SendDataIT(&spi2Handle, longString, dataLen) == SPI_BUSY_IN_TX);
		// while (SPI_ReceiveDataIT(&spi1Handle, &masterRcvBuf, dataLen) == SPI_BUSY_IN_RX);
		SPI_ReceiveData(&spi1Handle, &masterRcvBuf, dataLen);

		// while (SPI_ReceiveDataIT(&spi2Handle, &slaveRcvBuf, dataLen) == SPI_BUSY_IN_RX);
		// // while (SPI_SendDataIT(&spi1Handle, longString, dataLen) == SPI_BUSY_IN_TX);
		// SPI_SendData(SPI1, longString, dataLen);

		// while (!slaveRcvComplete);
		// slaveRcvComplete = FALSE;

		// for (uint32_t i = 0; i < dataLen; i++)
		// {
		// 	char c = (char) slaveRcvBuf[i];
		// 	if (c >= 'A' && c <= 'Z')
		// 	{
		// 		slaveRcvBuf[i] = c + 32;
		// 	}
		// 	else if (c >= 'a' && c <= 'z')
		// 	{
		// 		slaveRcvBuf[i] = c - 32;
		// 	}
		// }

		// while (SPI_SendDataIT(&spi2Handle, &slaveRcvBuf, dataLen) == SPI_BUSY_IN_TX);
		// SPI_ReceiveData(&spi1Handle, &masterRcvBuf, dataLen);
	}

	return 0;
}

void SPI1_IRQHandler(void)
{
	SPI_IrqHandling(&spi1Handle);
}

void SPI2_IRQHandler(void)
{
	SPI_IrqHandling(&spi2Handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEv)
{
	if (appEv == SPI_EVENT_RX_CMPLT && pSPIHandle == &spi2Handle)
	{
		slaveRcvComplete = TRUE;
	}
}
