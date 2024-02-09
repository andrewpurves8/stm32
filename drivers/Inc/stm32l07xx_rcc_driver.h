/*
 * stm32l07xx_rcc_driver.h
 *
 *  Created on: Feb 6, 2024
 *      Author: andrew.purves
 */

#ifndef INC_STM32L07XX_RCC_DRIVER_H_
#define INC_STM32L07XX_RCC_DRIVER_H_

#include "stm32l07xx.h"

#define SYS_CLK_MSI                             0
#define SYS_CLK_HSI                             1
#define SYS_CLK_HSE                             2
#define SYS_CLK_PLL                             3

void RCC_SetSysClk(uint8_t clk);
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32L07XX_RCC_DRIVER_H_ */