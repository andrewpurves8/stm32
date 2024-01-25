/*
 * 008spi_cmd_handling.c
 *
 *  Created on: Jan 24, 2024
 *      Author: andrew.purves
 */

#include <stdio.h>
#include <string.h>
#include "stm32l07xx.h"

// extern void initialise_monitor_handles();

// command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     					1
#define LED_OFF    					0

// arduino analog pins
#define ANALOG_PIN0 				0
#define ANALOG_PIN1 				1
#define ANALOG_PIN2 				2
#define ANALOG_PIN3 				3
#define ANALOG_PIN4 				4

// arduino led
#define LED_PIN  					9

void delay(void)
{
	for (uint32_t i = 0; i < 250000; i++);
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
	SPI_Handle_t spi2Handle;

	spi2Handle.pSPIx = SPI2;
	spi2Handle.spiConfig.busConfig = SPI_BUS_CONFIG_FD;
	spi2Handle.spiConfig.deviceMode = SPI_DEVICE_MODE_MASTER;
	spi2Handle.spiConfig.sclkSpeed = SPI_SCLK_SPEED_DIV32;
	spi2Handle.spiConfig.cpol = SPI_CPOL_LOW;
	spi2Handle.spiConfig.cpha = SPI_CPHA_LOW;
	spi2Handle.spiConfig.ssm = SPI_SSM_DI; // hardware slave management enabled for NSS pin

	SPI_Init(&spi2Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t gpioBtn, gpioLed;

	gpioBtn.pGPIOx = GPIOA;
	gpioBtn.pinConfig.pinNumber = GPIO_PIN_NO_0;
	gpioBtn.pinConfig.pinMode = GPIO_MODE_IN;
	gpioBtn.pinConfig.pinSpeed = GPIO_SPEED_FAST;
	gpioBtn.pinConfig.pinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&gpioBtn);

	gpioLed.pGPIOx = GPIOD;
	gpioLed.pinConfig.pinNumber = GPIO_PIN_NO_12;
	gpioLed.pinConfig.pinMode = GPIO_MODE_OUT;
	gpioLed.pinConfig.pinSpeed = GPIO_SPEED_FAST;
	gpioLed.pinConfig.pinOPType = GPIO_OP_TYPE_OD;
	gpioLed.pinConfig.pinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&gpioLed);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{

	if (ackbyte == (uint8_t) 0xF5)
	{
		// ack
		return 1;
	}

	return 0;
}

int main(void)
{
	uint8_t dummyWrite = 0xff;
	uint8_t dummyRead;

	// initialise_monitor_handles();

	printf("Application is running\n");

	GPIO_ButtonInit();

	// this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// this function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	printf("SPI Init. done\n");

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1, NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2, ENABLE);

	while (1)
	{
		// wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// to avoid button de-bouncing related issues 200ms of delay
		delay();

		// enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

	    // CMD_LED_CTRL  	<pin no(1)>     <value(1)>
		uint8_t commandCode = COMMAND_LED_CTRL;
		uint8_t ackByte;
		uint8_t args[2];

		// send command
		SPI_SendData(SPI2, &commandCode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		// send some dummy bits (1 byte) fetch the response from the slave
		SPI_SendData(SPI2, &dummyWrite, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if (SPI_VerifyResponse(ackByte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;

			// send arguments
			SPI_SendData(SPI2, args, 2);
			// dummy read
			SPI_ReceiveData(SPI2, args, 2);
			printf("COMMAND_LED_CTRL Executed\n");
		}
		// end of COMMAND_LED_CTRL


		// CMD_SENOSR_READ   <analog pin number(1)>

		// wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandCode = COMMAND_SENSOR_READ;

		// send command
		SPI_SendData(SPI2, &commandCode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyRead, 1);


		// send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummyWrite, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if (SPI_VerifyResponse(ackByte))
		{
			args[0] = ANALOG_PIN0;

			// send arguments
			SPI_SendData(SPI2, args, 1); // sending one byte of

			// do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummyRead, 1);

			// insert some delay so that slave can ready with the data
			delay();

			// send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2, &dummyWrite, 1);

			uint8_t analogRead;
			SPI_ReceiveData(SPI2, &analogRead, 1);
			printf("COMMAND_SENSOR_READ %d\n", analogRead);
		}

		// CMD_LED_READ 	 <pin no(1) >

		// wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandCode = COMMAND_LED_READ;

		// send command
		SPI_SendData(SPI2, &commandCode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		// send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummyWrite, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if (SPI_VerifyResponse(ackByte))
		{
			args[0] = LED_PIN;

			// send arguments
			SPI_SendData(SPI2, args, 1); // sending one byte of

			// do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummyRead, 1);

			// insert some delay so that slave can ready with the data
			delay();

			// send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2, &dummyWrite, 1);

			uint8_t ledStatus;
			SPI_ReceiveData(SPI2, &ledStatus, 1);
			printf("COMMAND_READ_LED %d\n", ledStatus);
		}

		// CMD_PRINT 		<len(2)>  <message(len) >

		// wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandCode = COMMAND_PRINT;

		// send command
		SPI_SendData(SPI2, &commandCode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		// send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummyWrite, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		uint8_t message[] = "Hello! How are you?";
		if (SPI_VerifyResponse(ackByte))
		{
			args[0] = strlen((char*) message);

			// send arguments
			SPI_SendData(SPI2, args, 1); // sending length

			// do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummyRead, 1);

			delay();

			// send message
			for (int i = 0; i < args[0]; i++) {
				SPI_SendData(SPI2, &message[i], 1);
				SPI_ReceiveData(SPI2, &dummyRead, 1);
			}

			printf("COMMAND_PRINT Executed \n");
		}

		// CMD_ID_READ
		// wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandCode = COMMAND_ID_READ;

		// send command
		SPI_SendData(SPI2, &commandCode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		// send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummyWrite, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		uint8_t id[11];
		uint32_t i = 0;
		if (SPI_VerifyResponse(ackByte))
		{
			// read 10 bytes id from the slave
			for (i = 0; i < 10; i++)
			{
				// send dummy byte to fetch data from slave
				SPI_SendData(SPI2, &dummyWrite, 1);
				SPI_ReceiveData(SPI2, &id[i], 1);
			}

			id[10] = '\0';

			printf("COMMAND_ID : %s \n", id);
		}

		// lets confirm SPI is not busy
		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("SPI Communication Closed\n");
	}

	return 0;
}
