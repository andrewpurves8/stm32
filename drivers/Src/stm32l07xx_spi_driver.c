/*
 * stm32l07xx_spi_driver.c
 *
 *  Created on: Jan 24, 2024
 *      Author: andrew.purves
 */

#include "stm32l07xx_spi_driver.h"

static void spiTxeInterruptHandle(SPI_Handle_t *pSPIHandle);
static void spiRxneInterruptHandle(SPI_Handle_t *pSPIHandle);
static void spiOvrErrInterruptHandle(SPI_Handle_t *pSPIHandle);
static void sendDummyByteIfMaster(SPI_Handle_t *pSPIHandle);

static void spiTxeInterruptHandle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if (REG_TEST_BIT(pSPIHandle->pSPIx->CR1, SPI_CR1_DFF))
	{
		// 16 bit DFF
		// load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
		pSPIHandle->txLen -= 2;
		// (uint16_t*) pSPIHandle->pTxBuffer++;
		pSPIHandle->pTxBuffer += 2;
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

static void spiRxneInterruptHandle(SPI_Handle_t *pSPIHandle)
{
	// do rxing as per the dff
	if (REG_TEST_BIT(pSPIHandle->pSPIx->CR1, SPI_CR1_DFF))
	{
		// 16 bit
		*((uint16_t*) pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->rxLen -= 2;
		pSPIHandle->pRxBuffer += 2;
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
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spiOvrErrInterruptHandle(SPI_Handle_t *pSPIHandle)
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

static void sendDummyByteIfMaster(SPI_Handle_t *pSPIHandle)
{
	if (pSPIHandle->spiConfig.deviceMode == SPI_DEVICE_MODE_MASTER)
	{
		// wait until TXE is set
		while (!REG_TEST_BIT(pSPIHandle->pSPIx->SR, SPI_SR_TXE));

		// check the DFF bit in CR1
		if (REG_TEST_BIT(pSPIHandle->pSPIx->CR1, SPI_CR1_DFF))
		{
			// 16 bit DFF
			// load the data in to the DR
			pSPIHandle->pSPIx->DR = (uint16_t) 0;
		}
		else
		{
			// 8 bit DFF
			pSPIHandle->pSPIx->DR = (uint8_t) 0;
		}
	}
}

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
		REG_CLEAR_BIT(pSPIHandle->pSPIx->CR2, SPI_CR2_TXDMAEN);
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

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while (len > 0)
	{
		// wait until TXE is set
		while (!REG_TEST_BIT(pSPIx->SR, SPI_SR_TXE));

		// check the DFF bit in CR1
		if (REG_TEST_BIT(pSPIx->CR1, SPI_CR1_DFF))
		{
			// 16 bit DFF
			// load the data in to the DR
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			len -= 2;
			// (uint16_t*) pTxBuffer++;
			pTxBuffer += 2;
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

void SPI_ReceiveData(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	// if master, clear off RXNE for any data received before the first dummy byte is sent
	if (pSPIHandle->spiConfig.deviceMode == SPI_DEVICE_MODE_MASTER)
	{
		(void) pSPIHandle->pSPIx->DR;
	}

	while (len > 0)
	{
		// if master, send dummy byte to force slave to send
		sendDummyByteIfMaster(pSPIHandle);

		// wait until RXNE is set
		while (!REG_TEST_BIT(pSPIHandle->pSPIx->SR, SPI_SR_RXNE));

		// check the DFF bit in CR1
		if ((REG_TEST_BIT(pSPIHandle->pSPIx->CR1, SPI_CR1_DFF)))
		{
			// 16 bit DFF
			// load the data from DR to Rxbuffer address
			*((uint16_t*) pRxBuffer) = pSPIHandle->pSPIx->DR;
			len -= 2;
			// (uint16_t*) pRxBuffer++;
			pRxBuffer += 2;
		}
		else
		{
			// 8 bit DFF
			*(pRxBuffer) = pSPIHandle->pSPIx->DR;
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

void SPI_SendDMA(SPI_RegDef_t *pSPIx, DMA_Handle_t *pDMAHandle, uint8_t *pBuffer, uint32_t len)
{
	while (DMA_StartIT(pDMAHandle, (uint32_t) pBuffer, (uint32_t) &pSPIx->DR, len) != DMA_READY);
	REG_SET_BIT(pSPIx->CR2, SPI_CR2_TXEIE);
	REG_SET_BIT(pSPIx->CR2, SPI_CR2_TXDMAEN);
	SPI_PeripheralControl(pSPIx, ENABLE);
}

void SPI_ReceiveDMA(SPI_RegDef_t *pSPIx, DMA_Handle_t *pDMAHandle, uint8_t *pBuffer, uint32_t len)
{
	while (DMA_StartIT(pDMAHandle, (uint32_t) pBuffer, (uint32_t) &pSPIx->DR, len) != DMA_READY);
	REG_SET_BIT(pSPIx->CR2, SPI_CR2_RXNEIE);
	REG_SET_BIT(pSPIx->CR2, SPI_CR2_RXDMAEN);
	SPI_PeripheralControl(pSPIx, ENABLE);
}

void SPI_IrqHandling(SPI_Handle_t *pHandle)
{
	// first lets check for TXE
	if (REG_TEST_BIT(pHandle->pSPIx->SR, SPI_SR_TXE) && REG_TEST_BIT(pHandle->pSPIx->CR2, SPI_CR2_TXEIE))
	{
		// handle TXE
		spiTxeInterruptHandle(pHandle);
	}

	// check for RXNE
	if (REG_TEST_BIT(pHandle->pSPIx->SR, SPI_SR_RXNE) && REG_TEST_BIT(pHandle->pSPIx->CR2, SPI_CR2_RXNEIE))
	{
		// handle RXNE
		spiRxneInterruptHandle(pHandle);
	}

	// check for ovr flag
	if (REG_TEST_BIT(pHandle->pSPIx->SR, SPI_SR_OVR) && REG_TEST_BIT(pHandle->pSPIx->CR2, SPI_CR2_ERRIE))
	{
		// handle ovr error
		spiOvrErrInterruptHandle(pHandle);
	}
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