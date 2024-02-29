/*
 * stm32l07xx_dma_driver.c
 *
 *  Created on: Feb 23, 2024
 *      Author: andrew.purves
 */

#include "stm32l07xx_dma_driver.h"

static void enableInterrupts(uint8_t channel)
{
	REG_SET_BIT(DMA->CHANNELS[channel - 1].CCR, DMA_CCR_TCIE);
	REG_SET_BIT(DMA->CHANNELS[channel - 1].CCR, DMA_CCR_TEIE);
}

static void disableInterrupts(uint8_t channel)
{
	REG_CLEAR_BIT(DMA->CHANNELS[channel - 1].CCR, DMA_CCR_TCIE);
	REG_CLEAR_BIT(DMA->CHANNELS[channel - 1].CCR, DMA_CCR_TEIE);
}

void DMA_PeriClockControl(uint8_t en)
{
	if (en == ENABLE)
	{
		DMA_PCLK_EN();
	}
	else
	{
		DMA_PCLK_DI();
	}
}

void DMA_Init(DMA_Handle_t *pDMAHandle)
{
	DMA_PeriClockControl(ENABLE);

	uint32_t ccrReg = 0;
	ccrReg |= pDMAHandle->direction << DMA_CCR_DIR;
	ccrReg |= pDMAHandle->mode << DMA_CCR_CIRC;
	ccrReg |= pDMAHandle->peripheralIncrement << DMA_CCR_PINC;
	ccrReg |= pDMAHandle->memoryIncrement << DMA_CCR_MINC;
	ccrReg |= pDMAHandle->peripheralSize << DMA_CCR_PSIZE;
	ccrReg |= pDMAHandle->memorySize << DMA_CCR_MSIZE;
	ccrReg |= pDMAHandle->priority << DMA_CCR_PL;
	DMA->CHANNELS[pDMAHandle->channel - 1].CCR = ccrReg;

	DMA->CSELR |= pDMAHandle->channelSelection << (pDMAHandle->channel - 1) * 4;
}

void DMA_DeInit()
{
	DMA_PeriClockControl(DISABLE);
}

void DMA_EnableInterrupts(uint8_t channel)
{
	REG_SET_BIT(DMA->CHANNELS[channel - 1].CCR, DMA_CCR_TCIE);
	REG_SET_BIT(DMA->CHANNELS[channel - 1].CCR, DMA_CCR_TEIE);
}

void DMA_DisableInterrupts(uint8_t channel)
{
	REG_CLEAR_BIT(DMA->CHANNELS[channel - 1].CCR, DMA_CCR_TCIE);
	REG_CLEAR_BIT(DMA->CHANNELS[channel - 1].CCR, DMA_CCR_TEIE);
}

void DMA_IrqInterruptConfig(uint8_t irqNumber, uint8_t en)
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

void DMA_IrqPriorityConfig(uint8_t irqNumber, uint32_t irqPriority)
{
	// find out the ipr register
	uint8_t iprx = irqNumber / 4;
	uint8_t iprxSection = irqNumber % 4;

	uint8_t shiftAmount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR + iprx) |= irqPriority << shiftAmount;
}

void DMA_IrqHandling(DMA_Handle_t *pDMAHandle)
{
	uint8_t channelIndex = pDMAHandle->channel - 1;
	uint32_t isr = DMA->ISR;
	uint32_t ccr = DMA->CHANNELS[channelIndex].CCR;
	uint8_t bitShift = 4 * channelIndex;

	if (REG_TEST_BIT(isr, DMA_ISR_TCIF + bitShift) && REG_TEST_BIT(ccr, DMA_CCR_TCIE))
	{
		REG_SET_BIT(DMA->IFCR, DMA_IFCR_CTCIF + bitShift);
		pDMAHandle->state = DMA_READY;
		DMA_ApplicationEventCallback(pDMAHandle, DMA_EVENT_CMPLT);
	}
	else if (REG_TEST_BIT(isr, DMA_ISR_TEIF + bitShift) && REG_TEST_BIT(ccr, DMA_CCR_TEIE))
	{
		REG_SET_BIT(DMA->IFCR, DMA_IFCR_CTEIF + bitShift);
		pDMAHandle->state = DMA_READY;
		DMA_ApplicationEventCallback(pDMAHandle, DMA_EVENT_ERR);
	}
}

uint8_t DMA_StartIT(DMA_Handle_t *pDMAHandle, uint32_t memoryAddress, uint32_t peripheralAddress, uint32_t len)
{
	uint8_t state = pDMAHandle->state;

	if (state == DMA_READY)
	{
		pDMAHandle->state = DMA_BUSY;
		uint8_t channelNo = pDMAHandle->channel;
		DMA_PeripheralControl(channelNo, DISABLE);

		volatile DMAChannel_RegDef_t* dmaChannel = &DMA->CHANNELS[channelNo - 1];
		dmaChannel->CMAR = memoryAddress;
		dmaChannel->CPAR = peripheralAddress;
		dmaChannel->CNDTR = len;
		
		enableInterrupts(channelNo);

		DMA_PeripheralControl(pDMAHandle->channel, ENABLE);
	}

	return state;
}

void DMA_PeripheralControl(uint8_t channel, uint8_t en)
{
	if (en == ENABLE)
	{
		REG_SET_BIT(DMA->CHANNELS[channel - 1].CCR, DMA_CCR_EN);
	}
	else
	{
		REG_CLEAR_BIT(DMA->CHANNELS[channel - 1].CCR, DMA_CCR_EN);
	}
}

__weak void DMA_ApplicationEventCallback(DMA_Handle_t *pDMAHandle, uint8_t appEv)
{

}

