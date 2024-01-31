
/*
 * This application receives and prints the user message received from the Arduino peripheral in SPI interrupt mode
 * User sends the message through Arduino IDE's serial monitor tool
 * Monitor the message received in the SWV itm data console
 */

/*
 * Note : Follow the instructions to test this code
 * 1. Download this code on to STM32 board, acts as Master
 * 2. Download Slave code (003SPISlaveUartReadOverSPI.ino) on to Arduino board (Slave)
 * 3. Reset both the boards
 * 4. Enable SWV ITM data console to see the message
 * 5. Open Arduino IDE serial monitor tool
 * 6. Type anything and send the message (Make sure that in the serial monitor tool line ending set to carriage return)
 */
#include <stdio.h>
#include <string.h>
#include "stm32l07xx.h"

SPI_Handle_t spi2Handle;

#define MAX_LEN 500

char rcvBuff[MAX_LEN];

volatile char readByte;
volatile uint8_t rcvStop = 0;

/* this flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;

void delay(void)
{
	for(uint32_t i = 0; i < 250000; i++);
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t spiPins;

	spiPins.pGPIOx = GPIOB;
	spiPins.pinConfig.pinMode = GPIO_MODE_ALTFN;
	spiPins.pinConfig.pinAltFunMode = 5;
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
	spi2Handle.spiConfig.deviceMode = SPI_DEVICE_MODE_MASTER;
	spi2Handle.spiConfig.sclkSpeed = SPI_SCLK_SPEED_DIV32;
	spi2Handle.spiConfig.dff = SPI_DFF_8BITS;
	spi2Handle.spiConfig.cpol = SPI_CPOL_LOW;
	spi2Handle.spiConfig.cpha = SPI_CPHA_LOW;
	spi2Handle.spiConfig.ssm = SPI_SSM_DI; // hardware slave management enabled for NSS pin

	SPI_Init(&spi2Handle);
}

/* configures the gpio pin over which SPI peripheral issues data available interrupt */
void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t spiIntPin;
	memset(&spiIntPin, 0, sizeof(spiIntPin));

	// this is led gpio configuration
	spiIntPin.pGPIOx = GPIOD;
	spiIntPin.pinConfig.pinNumber = GPIO_PIN_NO_6;
	spiIntPin.pinConfig.pinMode = GPIO_MODE_IT_FT;
	spiIntPin.pinConfig.pinSpeed = GPIO_SPEED_LOW;
	spiIntPin.pinConfig.pinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&spiIntPin);

	GPIO_IrqPriorityConfig(IRQ_NO_EXTI15_4, NVIC_IRQ_PRI3);
	GPIO_IrqInterruptConfig(IRQ_NO_EXTI15_4, ENABLE);
}

int main(void)
{
	uint8_t dummy = 0xFF;

	Slave_GPIO_InterruptPinInit();

	// this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// this function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1, NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SsoeConfig(SPI2, ENABLE);

	SPI_IrqInterruptConfig(IRQ_NO_SPI2, ENABLE);

	while (1)
	{
		rcvStop = 0;

		while (!dataAvailable); // wait till data available interrupt from transmitter device(slave)

		GPIO_IrqInterruptConfig(IRQ_NO_EXTI15_4, DISABLE);

		// enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		while (!rcvStop)
		{
			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
			while (SPI_SendDataIT(&spi2Handle, &dummy, 1) == SPI_BUSY_IN_TX);
			while (SPI_ReceiveDataIT(&spi2Handle, &readByte, 1) == SPI_BUSY_IN_RX);
		}

		// confirm SPI is not busy
		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("Rcvd data = %s\n", rcvBuff);

		dataAvailable = 0;

		GPIO_IrqInterruptConfig(IRQ_NO_EXTI15_4, ENABLE);
	}

	return 0;
}

/* runs when a data byte is received from the peripheral over SPI */
void SPI2_IRQHandler(void)
{
	SPI_IrqHandling(&spi2Handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEv)
{
	static uint32_t i = 0;
	/* in the RX complete event, copy data in to rcv buffer. '\0' indicates end of message(rcvStop = 1) */
	if (appEv == SPI_EVENT_RX_CMPLT)
	{
		rcvBuff[i++] = readByte;
		if (readByte == '\0' || (i == MAX_LEN))
		{
			rcvStop = 1;
			rcvBuff[i - 1] = '\0';
			i = 0;
		}
	}

}

/* slave data available interrupt handler */
void EXTI15_4_IRQHandler(void)
{
	GPIO_IrqHandling(GPIO_PIN_NO_6);
	dataAvailable = 1;
}
