/*
 * i2c_read_temperature.c
 *
 *  Created on: Jan 31, 2024
 *      Author: andrew.purves
 */

#include <stdio.h>
#include <string.h>
#include "stm32l07xx.h"

#define MY_ADDR 								0x61

//#define SLAVE_ADDR  							0xB8
#define SLAVE_ADDR  							0x5C
#define READ_COMMAND							0x03
#define TEMPERATURE_ADDR						0x02
#define TEMPERATURE_LEN							2

// request to sensor contains function code + starting address + number of registers
#define DATA_LEN_SEND							3
// sensor sends function code + number of bytes + requested data (TEMPERATURE_LEN) + CRC code
#define DATA_LEN_RECEIVE						(3 + TEMPERATURE_LEN)

void delay(void)
{
	for (uint32_t i = 0; i < 250000; i++);
}

I2C_Handle_t i2c3Handle;

// rcv buffer
uint8_t rcvBuf[DATA_LEN_RECEIVE];
uint8_t sendData[DATA_LEN_SEND] = {READ_COMMAND, TEMPERATURE_ADDR, TEMPERATURE_LEN};

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
	i2cPins.pinConfig.pinSpeed = GPOI_SPEED_HIGH;

	// scl
	i2cPins.pinConfig.pinNumber = GPIO_PIN_NO_0;
	GPIO_Init(&i2cPins);

	// sda
	i2cPins.pinConfig.pinNumber = GPIO_PIN_NO_1;
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
	I2C3_GPIOInits();

	// i2c peripheral configuration
	I2C3_Inits();

	// enable the i2c peripheral
	I2C_PeripheralControl(I2C3, ENABLE);

	while (1)
	{
		delay();

		I2C_MasterSendData(&i2c3Handle, &sendData, DATA_LEN_SEND, SLAVE_ADDR);
		I2C_MasterReceiveData(&i2c3Handle, &rcvBuf, DATA_LEN_RECEIVE, SLAVE_ADDR);
	}
}
