/*
 * 013i2c_slave_tx_string.c
 *
 *  Created on: Feb 5, 2024
 *      Author: andrew.purves
 */


#include <stdio.h>
#include <string.h>
#include "stm32l07xx.h"

#define SLAVE_ADDR  							0x69
#define MY_ADDR 								SLAVE_ADDR

uint8_t txBuf[32] = "ABCD1234";
I2C_Handle_t i2c3Handle;

void delay(void)
{
	for (uint32_t i = 0; i < 250000; i++);
}

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
	i2c3Handle.masterSlave = I2C_SLAVE;

	I2C_Init(&i2c3Handle);
}

int main(void)
{
	// i2c pin inits
	I2C3_GPIOInits();

	// i2c peripheral configuration
	I2C3_Inits();

	// i2c IRQ configurations
	I2C_IrqInterruptConfig(IRQ_NO_I2C3, ENABLE);
	I2C_EnableInterrupts(I2C3);

	// enable the i2c peripheral
	I2C_PeripheralControl(I2C3, ENABLE);

	while (1);
}

void I2C3_IRQHandler(void)
{
	I2C_IrqHandling(&i2c3Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEv)
{
	static uint8_t commandCode = 0;
	static uint8_t count = 0;

	if (appEv == I2C_EV_DATA_REQ)
	{
		// master wants some data. slave has to send it
		if (commandCode == 0x51)
		{
			// send the length information to the master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*) txBuf));
		}
		else if (commandCode == 0x52)
		{
			// send the contents of txBuf
			I2C_SlaveSendData(pI2CHandle->pI2Cx, txBuf[count++]);
		}
	}
	else if (appEv == I2C_EV_DATA_RCV)
	{
		// data is waiting for the slave to read
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}
	else if (appEv == I2C_ERROR_NACK)
	{
		// this happens only during slave txing
		// master has sent the NACK so slave should understand that master doesnt need
		// more data
		commandCode = 0xff;
		count = 0;
	}
	else if (appEv == I2C_EV_STOP)
	{
		// this happens only during slave reception
		// master has ended the I2C communication with the slave
	}
}
