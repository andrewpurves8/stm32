/*
 * 011i2c_master_rx_testing.c
 *
 *  Created on: Jan 31, 2024
 *      Author: andrew.purves
 */


#include <stdio.h>
#include <string.h>
#include "stm32l07xx.h"

#define MY_ADDR 								0x61
#define SLAVE_ADDR  							0x68

I2C_Handle_t i2c3Handle;

// rcv buffer
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

	I2C_Init(&i2c3Handle);
}

int main(void)
{
	uint8_t len;
	
	RCC_SetSysClk(SYS_CLK_HSI);

	I2C3_GPIOInits();

	// i2c peripheral configuration
	I2C3_Inits();

	// enable the i2c peripheral
	I2C_PeripheralControl(I2C3, ENABLE);

	while (1)
	{
		delay(250);

		uint8_t data = 0x51;
		I2C_MasterSendData(&i2c3Handle, &data, 1, SLAVE_ADDR);
		I2C_MasterReceiveData(&i2c3Handle, &len, 1, SLAVE_ADDR);

		data = 0x52;
		I2C_MasterSendData(&i2c3Handle, &data, 1, SLAVE_ADDR);
		I2C_MasterReceiveData(&i2c3Handle, &rcvBuf, len, SLAVE_ADDR);
	}
}