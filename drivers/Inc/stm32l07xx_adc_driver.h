/*
 * stm32l07xx_adc_driver.h
 *
 *  Created on: Feb 19, 2024
 *      Author: andrew.purves
 */

#ifndef INC_STM32L07XX_ADC_DRIVER_H_
#define INC_STM32L07XX_ADC_DRIVER_H_

#include "stm32l07xx.h"

// /*
//  * Configuration structure for ADCx peripheral
//  */
// typedef struct
// {
// 	uint8_t pin;
// } ADC_Config_t;

// /*
//  * Handle structure for ADCx peripheral
//  */
// typedef struct
// {
// 	ADC_RegDef_t *pADCx;
// 	ADC_Config_t config;
// } ADC_Handle_t;

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void ADC_PeriClockControl(uint8_t en);

/*
 * Init and De-init
 */
// void ADC_Init(ADC_Handle_t *pADCHandle);
void ADC_Init(uint8_t pin);
void ADC_DeInit();

/*
 * Data read
 */
float ADC_DataRead();

#endif
