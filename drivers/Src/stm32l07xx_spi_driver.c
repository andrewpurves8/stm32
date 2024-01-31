/*
 * stm32l07xx_spi_driver.c
 *
 *  Created on: Jan 24, 2024
 *      Author: andrew.purves
 */

#include "stm32l07xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t en)
{

	if (en == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
	}
}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// configure the SPI_CR1 register
	uint32_t cr1Value = 0;

	// configure the device mode
	cr1Value |= pSPIHandle->spiConfig.deviceMode << SPI_CR1_MSTR;

	// configure the bus config
	if (pSPIHandle->spiConfig.busConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDI mode should be cleared
		REG_CLEAR_BIT(cr1Value, SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->spiConfig.busConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDI mode should be set
		REG_SET_BIT(cr1Value, SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->spiConfig.busConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI mode should be cleared
		REG_CLEAR_BIT(cr1Value, SPI_CR1_BIDIMODE);
		// RXONLY bit must be set
		REG_SET_BIT(cr1Value, SPI_CR1_RXONLY);
	}

	// configure the spi serial clock speed (baud rate)
	cr1Value |= pSPIHandle->spiConfig.sclkSpeed << SPI_CR1_BR;

	// configure the DFF
	cr1Value |= pSPIHandle->spiConfig.dff << SPI_CR1_DFF;

	// configure the CPOL
	cr1Value |= pSPIHandle->spiConfig.cpol << SPI_CR1_CPOL;

	// configure the CPHA
	cr1Value |= pSPIHandle->spiConfig.cpha << SPI_CR1_CPHA;

	cr1Value |= pSPIHandle->spiConfig.ssm << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = cr1Value;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	// todo
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName)
{
	if (pSPIx->SR & flagName)
	{
		return SET;
	}
	return RESET;
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while (len > 0)
	{
		// wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == RESET);

		// check the DFF bit in CR1
		if ((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			// 16 bit DFF
			// load the data in to the DR
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			len--;
			len--;
			(uint16_t*) pTxBuffer++;
		}
		else
		{
			// 8 bit DFF
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while(len > 0)
	{
		// wait until RXNE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t) RESET);

		// check the DFF bit in CR1
		if ((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			// 16 bit DFF
			// load the data from DR to Rxbuffer address
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			len--;
			len--;
			(uint16_t*) pRxBuffer++;
		}
		else
		{
			// 8 bit DFF
			*(pRxBuffer) = pSPIx->DR ;
			len--;
			pRxBuffer++;
		}
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t en)
{
	if (en == ENABLE)
	{
		REG_SET_BIT(pSPIx->CR1, SPI_CR1_SPE);
	}
	else
	{
		REG_CLEAR_BIT(pSPIx->CR1, SPI_CR1_SPE);
	}
}

void  SPI_SsiConfig(SPI_RegDef_t *pSPIx, uint8_t en)
{
	if (en == ENABLE)
	{
		REG_SET_BIT(pSPIx->CR1, SPI_CR1_SSI);
	}
	else
	{
		REG_CLEAR_BIT(pSPIx->CR1, SPI_CR1_SSI);
	}
}

void  SPI_SsoeConfig(SPI_RegDef_t *pSPIx, uint8_t en)
{
	if (en == ENABLE)
	{
		REG_SET_BIT(pSPIx->CR2, SPI_CR2_SSOE);
	}
	else
	{
		REG_CLEAR_BIT(pSPIx->CR2, SPI_CR2_SSOE);
	}
}

void SPI_IrqInterruptConfig(uint8_t irqNumber, uint8_t en)
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

void SPI_IrqPriorityConfig(uint8_t irqNumber, uint32_t irqPriority)
{
	// first lets find out the ipr register
	uint8_t iprx = irqNumber / 4;
	uint8_t iprxSection = irqNumber % 4;

	uint8_t shift_amount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR + iprx) |= (irqPriority << shift_amount);
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->txState;

	if (state != SPI_BUSY_IN_TX)
	{
		// save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->txLen = len;
		// mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->txState = SPI_BUSY_IN_TX;

		// enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		REG_SET_BIT(pSPIHandle->pSPIx->CR2, SPI_CR2_TXEIE);
	}

	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->rxState;

	if (state != SPI_BUSY_IN_RX)
	{
		// save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->rxLen = len;
		// mark the SPI state as busy in reception so that
		// no other code can take over same SPI peripheral until reception is over
		pSPIHandle->rxState = SPI_BUSY_IN_RX;

		// enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		REG_SET_BIT(pSPIHandle->pSPIx->CR2, SPI_CR2_RXNEIE);
	}

	return state;
}

void SPI_IrqHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	// first lets check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		// handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		// handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		// handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

// some helper function implementations

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if ((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{
		// 16 bit DFF
		// load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
		pSPIHandle->txLen--;
		pSPIHandle->txLen--;
		(uint16_t*) pSPIHandle->pTxBuffer++;
	}
	else
	{
		// 8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->txLen--;
		pSPIHandle->pTxBuffer++;
	}

	if (!pSPIHandle->txLen)
	{
		// txLen is zero, so close the spi transmission and inform the application that
		// TX is over

		// this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// do rxing as per the dff
	if (pSPIHandle->pSPIx->CR1 & (1 << 11))
	{
		// 16 bit
		*((uint16_t*) pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->rxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;
	}
	else
	{
		// 8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->rxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if (!pSPIHandle->rxLen)
	{
		// reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	// clear the ovr flag
	if (pSPIHandle->txState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void) temp;
	// inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	REG_CLEAR_BIT(pSPIHandle->pSPIx->CR2, SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->txLen = 0;
	pSPIHandle->txState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	REG_CLEAR_BIT(pSPIHandle->pSPIx->CR2, SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->rxLen = 0;
	pSPIHandle->rxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void) temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEv)
{
	// this is a weak implementation - the user application may override this function
}