/*
 * stm32l07xx_gpio_driver.c
 *
 *  Created on: Jan 19, 2024
 *      Author: andrew.purves
 */

#include "stm32l07xx_gpio_driver.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t en)
{
	if (en == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		//TODO
	}
}


void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;

	// enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// configure the mode of gpio pin
	if (pGPIOHandle->pinConfig.pinMode <= GPIO_MODE_ANALOG)
	{
		// non interrupt mode
		temp = (pGPIOHandle->pinConfig.pinMode << (2 * pGPIOHandle->pinConfig.pinNumber ));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->pinConfig.pinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		// if (pGPIOHandle->pinConfig.pinMode == GPIO_MODE_IT_FT)
		// {
		// 	//1. configure the FTSR
		// 	EXTI->FTSR |= (1 << pGPIOHandle->pinConfig.pinNumber);
		// 	//Clear the corresponding RTSR bit
		// 	EXTI->RTSR &= ~(1 << pGPIOHandle->pinConfig.pinNumber);

		// }
		// else if (pGPIOHandle->pinConfig.pinMode ==GPIO_MODE_IT_RT)
		// {
		// 	//1 . configure the RTSR
		// 	EXTI->RTSR |= (1 << pGPIOHandle->pinConfig.pinNumber);
		// 	//Clear the corresponding RTSR bit
		// 	EXTI->FTSR &= ~(1 << pGPIOHandle->pinConfig.pinNumber);

		// }
		// else if (pGPIOHandle->pinConfig.pinMode == GPIO_MODE_IT_RFT)
		// {
		// 	//1. configure both FTSR and RTSR
		// 	EXTI->RTSR |= (1 << pGPIOHandle->pinConfig.pinNumber);
		// 	//Clear the corresponding RTSR bit
		// 	EXTI->FTSR |= (1 << pGPIOHandle->pinConfig.pinNumber);
		// }

		// //2. configure the GPIO port selection in SYSCFG_EXTICR
		// uint8_t temp1 = pGPIOHandle->pinConfig.pinNumber / 4 ;
		// uint8_t temp2 = pGPIOHandle->pinConfig.pinNumber % 4;
		// uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		// SYSCFG_PCLK_EN();
		// SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		// //3 . enable the exti interrupt delivery using IMR
		// EXTI->IMR |= 1 << pGPIOHandle->pinConfig.pinNumber;
	}

	//2. configure the speed
	temp = (pGPIOHandle->pinConfig.pinSpeed << (2 * pGPIOHandle->pinConfig.pinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->pinConfig.pinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3. configure the pupd settings
	temp = (pGPIOHandle->pinConfig.pinPuPdControl << (2 * pGPIOHandle->pinConfig.pinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->pinConfig.pinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	//4. configure the optype
	temp = (pGPIOHandle->pinConfig.pinOPType << pGPIOHandle->pinConfig.pinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->pinConfig.pinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. configure the alt functionality
	if (pGPIOHandle->pinConfig.pinMode == GPIO_MODE_ALTFN)
	{
		uint8_t afrReg, regSection;

		afrReg = pGPIOHandle->pinConfig.pinNumber / 8;
		regSection = pGPIOHandle->pinConfig.pinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[afrReg] &= ~(0xF << (4 * regSection));
		pGPIOHandle->pGPIOx->AFR[afrReg] |= (pGPIOHandle->pinConfig.pinAltFunMode << (4 * regSection));
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
   return (uint8_t) ((pGPIOx->IDR >> pinNumber) & 0x00000001);
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t) pGPIOx->IDR;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{
	if (value == SET)
	{
		pGPIOx->ODR |= (1 << pinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << pinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR  ^= (1 << pinNumber);
}

// void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
// {

// 	if (EnorDi == ENABLE)
// 	{
// 		if (IRQNumber <= 31)
// 		{
// 			//program ISER0 register
// 			*NVIC_ISER0 |= (1 << IRQNumber );

// 		}
// 		else if (IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
// 		{
// 			//program ISER1 register
// 			*NVIC_ISER1 |= (1 << (IRQNumber % 32) );
// 		}
// 		else if (IRQNumber >= 64 && IRQNumber < 96 )
// 		{
// 			//program ISER2 register //64 to 95
// 			*NVIC_ISER2 |= (1 << (IRQNumber % 64) );
// 		}
// 	}
// 	else
// 	{
// 		if (IRQNumber <= 31)
// 		{
// 			//program ICER0 register
// 			*NVIC_ICER0 |= (1 << IRQNumber );
// 		}
// 		else if (IRQNumber > 31 && IRQNumber < 64 )
// 		{
// 			//program ICER1 register
// 			*NVIC_ICER1 |= (1 << (IRQNumber % 32) );
// 		}
// 		else if (IRQNumber >= 64 && IRQNumber < 96 )
// 		{
// 			//program ICER2 register
// 			*NVIC_ICER2 |= (1 << (IRQNumber % 64) );
// 		}
// 	}

// }

// void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
// {
// 	//1. first lets find out the ipr register
// 	uint8_t iprx = IRQNumber / 4;
// 	uint8_t iprx_section  = IRQNumber %4 ;

// 	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED) ;

// 	*( NVIC_PR_BASE_ADDR + iprx ) |=  (IRQPriority << shift_amount );
// }

// void GPIO_IRQHandling(uint8_t pinNumber)
// {
// 	//clear the exti pr register corresponding to the pin number
// 	if (EXTI->PR & (1 << pinNumber))
// 	{
// 		//clear
// 		EXTI->PR |= (1 << pinNumber);
// 	}
// }
