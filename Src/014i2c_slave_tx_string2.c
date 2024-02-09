/*
 * 014i2c_slave_tx_string2.c
 *
 *  Created on: Feb 5, 2024
 *      Author: andrew.purves
 */

#include <stdio.h>
#include <string.h>
#include "stm32l07xx.h"

#define SLAVE_ADDR  							0x68
#define MY_ADDR 								SLAVE_ADDR

// very large message
uint8_t txBuf[] = "vbhytrdcfvbhgtfcvbhnygtfrdfvbgshnvtdkrhhdrnkgvdklxdhutlxdinulkxdbhnkxdkjgldxkbthkgjxdhnkgjbxdnkjghxkdbjhtgkubxdtkhgxkbduthgbxkhtkgxdnhkxgduhngubxdnughxbjdthgxdubtnxdhnxhdnthgbxduhbxdukhrnxdbuhgkxduhrgxdnugbxduhrgxbdnugxdgxbdidxghbxdgigdkiudhxkbxidukhrgiuxdkrughkxduhgxbudknhigxdgbuxdgrhdixndrgbiuhkgixdhrugibkhkbnuidkhzgixdidukrhgiukhdkxdiurkhgnbxdhrgbxudriukigbxidhuxrighkibuxdrkhigbgxidurhgiuxdrugr...123";
uint32_t dataLen = 0;
uint8_t commandCode;
uint8_t rcvBuf[32];
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
	dataLen = strlen((char*) txBuf);

	RCC_SetSysClk(SYS_CLK_HSI);

	// i2c pin inits
	I2C3_GPIOInits();

	// i2c peripheral configuration
	I2C3_Inits();

	// enable the i2c peripheral
	I2C_PeripheralControl(I2C3, ENABLE);

	I2C_IrqInterruptConfig(IRQ_NO_I2C3, ENABLE);

	I2C_EnableInterrupts(I2C3);

	while (1);
}

void I2C3_IRQHandler(void)
{
	I2C_IrqHandling(&i2c3Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEv)
{
	static uint32_t count = 0;
	static uint32_t wPtr = 0;

	if (appEv == I2C_ERROR_NACK)
	{
		// this will happen during slave transmitting data to master
		// master needs no more data, so slave concludes end of tx


		// if the current active code is 0x52 then dont invalidate
		if (commandCode != 0x52)
		{
			commandCode = 0xFF;
		}

		// reset the count variable because its end of transmission
		count = 0;

		// slave concludes it sent all the bytes when wPtr reaches dataLen
		if (wPtr >= dataLen)
		{
			wPtr = 0;
			commandCode = 0xff;
		}
	}
	else if (appEv == I2C_EV_STOP)
	{
		// this will happen during end slave reception
		// slave concludes end of rx
		count = 0;
	}
	else if (appEv == I2C_EV_DATA_REQ)
	{
		// master is requesting for the data
		if (commandCode == 0x51)
		{
			// here we are sending 4 bytes of length information
			I2C_SlaveSendData(I2C3, (dataLen >> ((count % 4) * 8)) & 0xFF);
		    count++;
		}
		else if (commandCode == 0x52)
		{
			// sending txBuf contents indexed by wPtr variable
			I2C_SlaveSendData(I2C3, txBuf[wPtr++]);
		}
	}
	else if (appEv == I2C_EV_DATA_RCV)
	{
		// master has sent command code, read it
		commandCode = I2C_SlaveReceiveData(I2C3);
	}
}