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
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}


void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	// enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// configure the mode of gpio pin
	if (pGPIOHandle->pinConfig.pinMode <= GPIO_MODE_ANALOG)
	{
		// non interrupt mode
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->pinConfig.pinNumber));
		pGPIOHandle->pGPIOx->MODER |= pGPIOHandle->pinConfig.pinMode << (2 * pGPIOHandle->pinConfig.pinNumber);
	}
	else
	{
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->pinConfig.pinNumber));
		pGPIOHandle->pGPIOx->MODER |= GPIO_MODE_IN << (2 * pGPIOHandle->pinConfig.pinNumber);

		if (pGPIOHandle->pinConfig.pinMode == GPIO_MODE_IT_FT)
		{
			// configure the FTSR
			REG_SET_BIT(EXTI->FTSR, pGPIOHandle->pinConfig.pinNumber);
			// clear the corresponding RTSR bit
			REG_CLEAR_BIT(EXTI->RTSR, pGPIOHandle->pinConfig.pinNumber);
		}
		else if (pGPIOHandle->pinConfig.pinMode == GPIO_MODE_IT_RT)
		{
			// configure the RTSR
			REG_SET_BIT(EXTI->RTSR, pGPIOHandle->pinConfig.pinNumber);
			// clear the corresponding FTSR bit
			REG_CLEAR_BIT(EXTI->FTSR, pGPIOHandle->pinConfig.pinNumber);
		}
		else if (pGPIOHandle->pinConfig.pinMode == GPIO_MODE_IT_RFT)
		{
			// configure both FTSR and RTSR
			REG_SET_BIT(EXTI->RTSR, pGPIOHandle->pinConfig.pinNumber);
			REG_SET_BIT(EXTI->FTSR, pGPIOHandle->pinConfig.pinNumber);
		}

		// configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t extix = pGPIOHandle->pinConfig.pinNumber / 4 ;
		uint8_t extixSecion = pGPIOHandle->pinConfig.pinNumber % 4;
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[extix] = portCode << (extixSecion * 4);

		// enable the exti interrupt delivery using IMR
		REG_SET_BIT(EXTI->IMR, pGPIOHandle->pinConfig.pinNumber);
	}

	// configure the speed
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->pinConfig.pinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= pGPIOHandle->pinConfig.pinSpeed << (2 * pGPIOHandle->pinConfig.pinNumber);

	// configure the pupd settings
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->pinConfig.pinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= pGPIOHandle->pinConfig.pinPuPdControl << (2 * pGPIOHandle->pinConfig.pinNumber);

	// configure the optype
	REG_CLEAR_BIT(pGPIOHandle->pGPIOx->OTYPER, pGPIOHandle->pinConfig.pinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= pGPIOHandle->pinConfig.pinOPType << pGPIOHandle->pinConfig.pinNumber;

	// configure the alt functionality
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
		REG_SET_BIT(pGPIOx->ODR, pinNumber);
	}
	else
	{
		REG_CLEAR_BIT(pGPIOx->ODR, pinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR ^= (1 << pinNumber);
}

void GPIO_IrqInterruptConfig(uint8_t irqNumber, uint8_t en)
{

	if (en == ENABLE)
	{
		REG_SET_BIT(*NVIC_ISER, irqNumber);
	}
	else
	{
		REG_SET_BIT(*NVIC_ICER, irqNumber);
	}

}

void GPIO_IrqPriorityConfig(uint8_t irqNumber, uint32_t irqPriority)
{
	// find out the ipr register
	uint8_t iprx = irqNumber / 4;
	uint8_t iprxSection = irqNumber % 4;

	uint8_t shiftAmount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR + iprx) |= (irqPriority << shiftAmount);
}

void GPIO_IrqHandling(uint8_t pinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if (EXTI->PR & (1 << pinNumber))
	{
		REG_SET_BIT(EXTI->PR, pinNumber);
	}
}
