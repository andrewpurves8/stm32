/*
 * am2320.h
 *
 *  Created on: Feb 16, 2024
 *      Author: andrew.purves
 */

#ifndef INC_AM2320_H_
#define INC_AM2320_H_

#include "stm32l07xx_i2c_driver.h"
#include <stdint.h>

#define AM2320_TEMPERATURE                      0
#define AM2320_HUMIDITY                         1
#define AM2320_TEMPERATURE_AND_HUMIDITY         2

#define AM2320_DATA_NOT_READY                   0
#define AM2320_DATA_READY                       1

/******************************************************************************************
 *								APIs supported by this driver
 ******************************************************************************************/

void AM2320_Init(uint8_t requestedData);
void AM2320_RequestData();
float AM2320_GetTemperature();
float AM2320_GetHumidity();

// this must be called from the application's I2C1_IRQHandler
void AM2320_IrqHandling();

// this must be called from the application's I2C_ApplicationEventCallback
// returns whether requested data is ready to be retrieved
uint8_t AM2320_HandleInterruptEvent(I2C_Handle_t *pI2CHandle, uint8_t appEv);

#endif
