/*
 * stm32l07xx_rcc_driver.h
 *
 *  Created on: Feb 6, 2024
 *      Author: andrew.purves
 */

#ifndef INC_STM32L07XX_RCC_DRIVER_H_
#define INC_STM32L07XX_RCC_DRIVER_H_

#include "stm32l07xx.h"

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32L07XX_RCC_DRIVER_H_ */