/*
 * spi_tx_rx_long_string.c
 *
 *  Created on: Feb 20, 2024
 *      Author: andrew.purves
 */

#include <stdio.h>
#include <string.h>
#include "stm32l07xx_dma_driver.h"
#include "stm32l07xx_spi_driver.h"
#include "stm32l07xx_gpio_driver.h"
#include "stm32l07xx_rcc_driver.h"

static const char* longString = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
static uint8_t slaveRcvBuf[1000];
static uint8_t masterRcvBuf[1000];
static uint8_t masterRcvComplete = FALSE;
static volatile uint8_t slaveRcvComplete = FALSE;

static SPI_Handle_t spi1Handle;
static SPI_Handle_t spi2Handle;
static DMA_Handle_t spi1TxDmaHandle;
static DMA_Handle_t spi2RxDmaHandle;

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
	spi1Handle.spiConfig.sclkSpeed = SPI_SCLK_SPEED_DIV2;
	spi1Handle.spiConfig.cpol = SPI_CPOL_LOW;
	spi1Handle.spiConfig.cpha = SPI_CPHA_LOW;
	spi1Handle.spiConfig.ssm = SPI_SSM_DI; // hardware slave management enabled for NSS pin
	spi1Handle.spiConfig.dff = SPI_DFF_8BITS;
	spi1Handle.spiConfig.dma = SPI_DMA_EN;

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
	spi2Handle.spiConfig.sclkSpeed = SPI_SCLK_SPEED_DIV2;
	spi2Handle.spiConfig.cpol = SPI_CPOL_LOW;
	spi2Handle.spiConfig.cpha = SPI_CPHA_LOW;
	spi2Handle.spiConfig.ssm = SPI_SSM_DI; // hardware slave management enabled for NSS pin
	spi2Handle.spiConfig.dff = SPI_DFF_8BITS;
	spi2Handle.spiConfig.dma = SPI_DMA_EN;

	SPI_Init(&spi2Handle);
}

/*
 * Relevant DMA requests per channel:
 * SPI1_RX: Channel 2, CxS = 0001
 * SPI1_TX: Channel 3, CxS = 0001
 * SPI2_RX: Channel 4/6, CxS = 0010
 * SPI2_TX: Channel 5/7, CxS = 0010
 */
void SPI1_DMA_Inits(void)
{
	spi1TxDmaHandle.channel = 3;
	spi1TxDmaHandle.channelSelection = 0b0001;
	spi1TxDmaHandle.direction = DMA_DIR_READ_FROM_MEMORY;
	spi1TxDmaHandle.memoryIncrement = DMA_MINC_EN;
	spi1TxDmaHandle.memorySize = DMA_MSIZE_8BITS;
	spi1TxDmaHandle.mode = DMA_MODE_NORMAL;
	spi1TxDmaHandle.peripheralIncrement = DMA_PINC_DI;
	spi1TxDmaHandle.peripheralSize = DMA_PSIZE_8BITS;
	spi1TxDmaHandle.priority = DMA_PRIO_VERY_HIGH;
	DMA_Init(&spi1TxDmaHandle);
}

void SPI2_DMA_Inits(void)
{
	spi2RxDmaHandle.channel = 4;
	spi2RxDmaHandle.channelSelection = 0b0010;
	spi2RxDmaHandle.direction = DMA_DIR_READ_FROM_PERIPHERAL;
	spi2RxDmaHandle.memoryIncrement = DMA_MINC_EN;
	spi2RxDmaHandle.memorySize = DMA_MSIZE_8BITS;
	spi2RxDmaHandle.mode = DMA_MODE_NORMAL;
	spi2RxDmaHandle.peripheralIncrement = DMA_PINC_DI;
	spi2RxDmaHandle.peripheralSize = DMA_PSIZE_8BITS;
	spi2RxDmaHandle.priority = DMA_PRIO_VERY_HIGH;
	DMA_Init(&spi2RxDmaHandle);
}

int main(void)
{
	uint32_t dataLen = strlen(longString);

	RCC_SetSysClk(SYS_CLK_HSI);

	SPI1_GPIOInits();
	SPI2_GPIOInits();

	SPI1_Inits();
	SPI2_Inits();

	SPI1_DMA_Inits();
	SPI2_DMA_Inits();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1, NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SsoeConfig(SPI1, ENABLE);

	DMA_IrqInterruptConfig(IRQ_NO_DMA1_CHANNEL3_2, ENABLE);
	DMA_IrqInterruptConfig(IRQ_NO_DMA1_CHANNEL7_4, ENABLE);

	SPI_PeripheralControl(SPI1, ENABLE);
	SPI_PeripheralControl(SPI2, ENABLE);

	while (1)
	{
		delay(250);

		SPI_ReceiveDMA(SPI2, &spi2RxDmaHandle, &slaveRcvBuf, dataLen);
		SPI_SendDMA(SPI1, &spi1TxDmaHandle, longString, dataLen);

		while (!slaveRcvComplete);
	}

	return 0;
}

void DMA1_Channel2_3_IRQHandler(void)
{
	DMA_IrqHandling(&spi1TxDmaHandle);
}

void DMA1_Channel4_7_IRQHandler(void)
{
	DMA_IrqHandling(&spi2RxDmaHandle);
}

void DMA_ApplicationEventCallback(DMA_Handle_t *pDMAHandle, uint8_t appEv)
{
	if (pDMAHandle == &spi1RxDmaHandle && appEv == DMA_EVENT_CMPLT)
	{
		masterRcvComplete = TRUE;
	}
	else if (pDMAHandle == &spi2RxDmaHandle && appEv == DMA_EVENT_CMPLT)
	{
		slaveRcvComplete = TRUE;
	}
}
