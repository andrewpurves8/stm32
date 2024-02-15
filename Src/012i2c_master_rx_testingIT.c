/*
 * 012i2c_master_rx_testingIT.c
 *
 *  Created on: Feb 2, 2024
 *      Author: andrew.purves
 */

#include <stdio.h>
#include <string.h>
#include "stm32l07xx.h"

// flag variable
uint8_t rxComplt = RESET;

#define MY_ADDR 0x61
#define SLAVE_ADDR  0x68

I2C_Handle_t i2c3Handle;
uint8_t rcvBuf[32];

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
	uint8_t commandCode, len;

	RCC_SetSysClk(SYS_CLK_HSI);

	// i2c pin inits
	I2C3_GPIOInits();

	// i2c peripheral configuration
	I2C3_Inits();

	// i2C IRQ configurations
	I2C_IrqInterruptConfig(IRQ_NO_I2C3, ENABLE);

	// enable the i2c peripheral
	I2C_PeripheralControl(I2C3, ENABLE);

	while (1)
	{
		delay(250);

		commandCode = 0x51;
		while (I2C_MasterSendDataIT(&i2c3Handle, &commandCode, 1, SLAVE_ADDR) != I2C_READY);
		while (I2C_MasterReceiveDataIT(&i2c3Handle, &len, 1, SLAVE_ADDR) != I2C_READY);

		commandCode = 0x52;
		while (I2C_MasterSendDataIT(&i2c3Handle, &commandCode, 1, SLAVE_ADDR) != I2C_READY);
		while (I2C_MasterReceiveDataIT(&i2c3Handle, rcvBuf, len, SLAVE_ADDR) != I2C_READY);

		rxComplt = RESET;

		// wait till rx completes
        while (rxComplt != SET);

		rcvBuf[len + 1] = '\0';
		rxComplt = RESET;
	}
}

void I2C3_IRQHandler(void)
{
	I2C_IrqHandling(&i2c3Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEv)
{
	if (appEv == I2C_EV_RX_CMPLT)
	{
		rxComplt = SET;
	}
	else if (appEv == I2C_ERROR_NACK)
	{
		// in master ack failure happens when slave fails to send ack for the byte
		// sent from the master.
		I2C_CloseSendData(pI2CHandle);

		// generate the stop condition to release the bus
		I2C_GenerateStopCondition(I2C3);

		// hang in infinite loop
		while (1);
	}
}
