/*
 * am2320_test.c
 *
 *  Created on: Jan 31, 2024
 *      Author: andrew.purves
 */

#include <stdio.h>
#include <string.h>
#include "stm32l07xx.h"

// datasheet specifies that 0xB8 should be sent for write, but address is expected to be the 7 MSBs
#define SLAVE_ADDR		  						0x5C
#define MY_ADDR 								0x61
#define READ_COMMAND							0x03
#define TEMPERATURE_ADDR						0x02
#define TEMPERATURE_LEN							2
#define HUMIDITY_ADDR							0x00
#define HUMIDITY_LEN							2

// request to sensor contains function code + starting address + number of registers
#define DATA_LEN_SEND							3
// sensor sends function code + number of bytes + requested data (TEMPERATURE_LEN + HUMIDITY_LEN) + CRC code
#define DATA_LEN_RECEIVE						(3 + TEMPERATURE_LEN + HUMIDITY_LEN)

static I2C_Handle_t i2c3Handle;

// rcv buffer
static uint8_t rcvBuf[DATA_LEN_RECEIVE];
static uint8_t sendData[DATA_LEN_SEND] = {READ_COMMAND, HUMIDITY_ADDR, TEMPERATURE_LEN + HUMIDITY_LEN};

static float temperature = 0.f;
static float humidity = 0.f;

/*
 * PC0-> SCL
 * PC1 -> SDA
 */
void I2C3_GPIOInits(void)
{
	GPIO_Handle_t i2cPins;

	i2cPins.pGPIOx = GPIOC;
	i2cPins.pinConfig.pinMode = GPIO_MODE_ALTFN;
	i2cPins.pinConfig.pinOPType = GPIO_OP_TYPE_OD;
	i2cPins.pinConfig.pinPuPdControl = GPIO_PIN_PU;
	i2cPins.pinConfig.pinAltFunMode = 7;
	i2cPins.pinConfig.pinSpeed = GPIO_SPEED_HIGH;

	// sda
	i2cPins.pinConfig.pinNumber = GPIO_PIN_NO_1;
	GPIO_Init(&i2cPins);

	// scl
	i2cPins.pinConfig.pinNumber = GPIO_PIN_NO_0;
	GPIO_Init(&i2cPins);
}

void I2C3_Inits(void)
{
	i2c3Handle.pI2Cx = I2C3;
	i2c3Handle.config.deviceAddress = MY_ADDR;
	i2c3Handle.config.sclSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&i2c3Handle);
}

int main(void)
{
	RCC_SetSysClk(SYS_CLK_HSI);

	I2C3_GPIOInits();

	// i2c peripheral configuration
	I2C3_Inits();

	// i2C IRQ configurations
	I2C_IrqInterruptConfig(IRQ_NO_I2C3, ENABLE);

	// enable the i2c peripheral
	I2C_PeripheralControl(I2C3, ENABLE);

	uint8_t dummyWrite = 0;

	while (1)
	{
		while (I2C_MasterSendDataIT(&i2c3Handle, &dummyWrite, 1, SLAVE_ADDR) != I2C_READY);
		delay(10);
		while (I2C_MasterSendDataIT(&i2c3Handle, &sendData, DATA_LEN_SEND, SLAVE_ADDR) != I2C_READY);
		delay(10);
		while (I2C_MasterReceiveDataIT(&i2c3Handle, &rcvBuf, DATA_LEN_RECEIVE, SLAVE_ADDR) != I2C_READY);
		delay(1000);
	}
}

void I2C3_IRQHandler(void)
{
	I2C_IrqHandling(&i2c3Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEv)
{
	static uint8_t nackReceived = RESET;
	static uint8_t humidityHighByte = 0;
	static uint8_t humidityLowByte = 0;
	static uint8_t temperatureHighByte = 0;
	static uint8_t temperatureLowByte = 0;
	if (appEv == I2C_EV_RX_CMPLT)
	{
		humidityHighByte = rcvBuf[2];
		humidityLowByte = rcvBuf[3];
		humidity = (humidityHighByte << 8 | humidityLowByte) / 10.f;

		temperatureHighByte = rcvBuf[4];
		temperatureLowByte = rcvBuf[5];
		temperature = (temperatureHighByte << 8 | temperatureLowByte) / 10.f;
	}
	else if (appEv == I2C_EV_STOP)
	{
		// the stop after the NACK caused by sensor wakeup must be ignored
		if (nackReceived)
		{
			nackReceived = RESET;
		}
		else
		{
			I2C_CloseSendData(pI2CHandle);
		}
	}
	else if (appEv == I2C_ERROR_NACK)
	{
		nackReceived = SET;
		// REG_SET_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_TXE);
		I2C_CloseSendData(pI2CHandle);
		I2C_GenerateStopCondition(I2C3);
	}
}
