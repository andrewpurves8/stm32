/*
 * stm32l07xx_usart_driver.c
 *
 *  Created on: Feb 6, 2024
 *      Author: andrew.purves
 */

#include "stm32l07xx_usart_driver.h"

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t baudRate)
{
	uint32_t pClkx, usartDiv;
  	uint32_t tempReg = 0;

	// get the value of APB bus clock in to the variable pClkx
	if (pUSARTx == USART1)
	{
		// USART1 is hanging on APB2 bus
		pClkx = RCC_GetPCLK2Value();
	}
	else
	{
		pClkx = RCC_GetPCLK1Value();
	}

	// check for OVER8 configuration bit
	if (REG_TEST_BIT(pUSARTx->CR1, USART_CR1_OVER8))
	{
		// OVER8 = 1, over sampling by 8
		usartDiv = (2 * pClkx + baudRate / 2) / baudRate;
		usartDiv &= ~(0xF);
		uint32_t brr3_0 = (usartDiv & 0xF) >> 1;
		usartDiv |= brr3_0;
	}
	else
	{
		// over sampling by 16
		usartDiv = (pClkx + baudRate / 2) / baudRate;
		tempReg = usartDiv;
	}

	pUSARTx->BRR = tempReg;
}

void USART_Init(USART_Handle_t *pUSARTHandle)
{
	// temporary variable
	uint32_t tempReg = 0;

	/******************************** Configuration of CR1 ******************************************/
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	// enable USART Tx and Rx engines according to the USART_Mode configuration item
	if (pUSARTHandle->config.mode == USART_MODE_ONLY_RX)
	{
		// enable the Receiver bit field
		REG_SET_BIT(tempReg, USART_CR1_RE);
	}
	else if (pUSARTHandle->config.mode == USART_MODE_ONLY_TX)
	{
		// enable the Transmitter bit field
		REG_SET_BIT(tempReg, USART_CR1_TE);
	}
	else if (pUSARTHandle->config.mode == USART_MODE_TXRX)
	{
		// enable the both Transmitter and Receiver bit fields
		REG_SET_BIT(tempReg, USART_CR1_RE);
		REG_SET_BIT(tempReg, USART_CR1_TE);
	}

    // configure the Word length configuration item
	tempReg |= ((pUSARTHandle->config.wordLength >> 1) & 0x1) << USART_CR1_M1;
	tempReg |= (pUSARTHandle->config.wordLength & 0x1) << USART_CR1_M0;

    // configuration of parity control bit fields
	if (pUSARTHandle->config.parityControl == USART_PARITY_EN_EVEN)
	{
		REG_SET_BIT(tempReg, USART_CR1_PCE);
	}
	else if (pUSARTHandle->config.parityControl == USART_PARITY_EN_ODD)
	{
	    REG_SET_BIT(tempReg, USART_CR1_PCE);
	    REG_SET_BIT(tempReg, USART_CR1_PS);
	}

   	// program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempReg;

	/******************************** Configuration of CR2 ******************************************/
	tempReg = 0;

	// configure the number of stop bits inserted during USART frame transmission
	tempReg |= pUSARTHandle->config.noOfStopBits << USART_CR2_STOP;
	pUSARTHandle->pUSARTx->CR2 = tempReg;

	/******************************** Configuration of CR3 ******************************************/
	tempReg = 0;

	// configuration of USART hardware flow control
	if (pUSARTHandle->config.hwFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		// enable CTS flow control
		REG_SET_BIT(tempReg, USART_CR3_CTSE);
	}
	else if (pUSARTHandle->config.hwFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		// enable RTS flow control
		REG_SET_BIT(tempReg, USART_CR3_RTSE);
	}
	else if (pUSARTHandle->config.hwFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		// enable both CTS and RTS Flow control
		REG_SET_BIT(tempReg, USART_CR3_CTSE);
		REG_SET_BIT(tempReg, USART_CR3_RTSE);
	}

	pUSARTHandle->pUSARTx->CR3 = tempReg;

	/******************************** Configuration of BRR (Baudrate register) ******************************************/

	// configure the baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->config.baud);
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t en)
{
	if (en == ENABLE)
	{
		REG_SET_BIT(pUSARTx->CR1, USART_CR1_UE);
	}
	else
	{
		REG_CLEAR_BIT(pUSARTx->CR1, USART_CR1_UE);
	}
}

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t en)
{
	if (en == ENABLE)
	{
		if (pUSARTx == USART1)
		{
			USART1_PCCK_EN();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCCK_EN();
		}
		else if (pUSARTx == USART4)
		{
			USART4_PCCK_EN();
		}
		else if (pUSARTx == USART5)
		{
			USART5_PCCK_EN();
		}
	}
	else
	{
		if (pUSARTx == USART1)
		{
			USART1_PCCK_DI();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCCK_DI();
		}
		else if (pUSARTx == USART4)
		{
			USART4_PCCK_DI();
		}
		else if (pUSARTx == USART5)
		{
			USART5_PCCK_DI();
		}
	}
}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint16_t *pData;

   	// loop over until "len" number of bytes are transferred
	for (uint32_t i = 0; i < len; i++)
	{
		// wait until TXE flag is set in the SR
		while (!REG_TEST_BIT(pUSARTHandle->pUSARTx->ISR, USART_ISR_TXE));

		// check the USART_WordLength item for 9 BIT or 8 BIT in a frame
		if (pUSARTHandle->config.wordLength == USART_WORDLEN_9BITS)
		{
			// if 9 BIT load the TDR with 2 bytes masking the bits other than first 9 bits
			pData = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->TDR = (*pData & (uint16_t) 0x01FF);

			// check for USART_ParityControl
			if (pUSARTHandle->config.parityControl == USART_PARITY_DISABLE)
			{
				// no parity is used in this transfer, so 9 bits of user data will be sent
				// increment pTxBuffer an extra time
				pTxBuffer++;
			}
		}
		else if (pUSARTHandle->config.wordLength == USART_WORDLEN_8BITS)
		{
			// this is 8 bit data transfer
			pUSARTHandle->pUSARTx->TDR = (*pTxBuffer & (uint8_t) 0xFF);
		}
		else
		{
			// this is 7 bit data transfer
			pUSARTHandle->pUSARTx->TDR = (*pTxBuffer & (uint8_t) 0x7F);
		}
		pTxBuffer++;
	}

	// wait till TC flag is set in the SR
	while (!REG_TEST_BIT(pUSARTHandle->pUSARTx->ISR, USART_ISR_TC));
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len)
{
   	// loop over until "len" number of bytes are transferred
	for (uint32_t i = 0; i < len; i++)
	{
		// wait until RXNE flag is set in the SR
		while (!REG_TEST_BIT(pUSARTHandle->pUSARTx->ISR, USART_ISR_RXNE));

		// check the USART_WordLength to decide whether we are going to receive 9 bit of data in a frame or 8 bit
		if (pUSARTHandle->config.wordLength == USART_WORDLEN_9BITS)
		{
			// we are going to receive 9 bit data in a frame

			// now, check are we using USART_ParityControl control or not
			if (pUSARTHandle->config.parityControl == USART_PARITY_DISABLE)
			{
				// no parity is used, so all 9 bits will be of user data

				// read only first 9 bits so mask the RDR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint16_t) 0x01FF);

				// now increment the pRxBuffer an extra time
				pRxBuffer++;
			}
			else
			{
				// parity is used, so 8 bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->RDR & (uint8_t) 0xFF);
			}
		}
		else if (pUSARTHandle->config.wordLength == USART_WORDLEN_8BITS)
		{
			// we are going to receive 8 bit data in a frame

			// now, check are we using USART_ParityControl control or not
			if (pUSARTHandle->config.parityControl == USART_PARITY_DISABLE)
			{
				// no parity is used, so all 8 bits will be of user data

				// read 8 bits from RDR
				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR & (uint8_t) 0xFF);
			}
			else
			{
				// parity is used, so 7 bits will be of user data and 1 bit is parity

				// read only 7 bits, hence mask the RDR with 0X7F
				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR & (uint8_t) 0x7F);
			}
		}
		else
		{
			// we are going to receive 7 bit data in a frame

			// now, check are we using USART_ParityControl control or not
			if (pUSARTHandle->config.parityControl == USART_PARITY_DISABLE)
			{
				// no parity is used, so all 7 bits will be of user data

				// read 7 bits from RDR
				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR & (uint8_t) 0x7F);
			}
			else
			{
				// parity is used, so 6 bits will be of user data and 1 bit is parity

				// read only 6 bits, hence mask the RDR with 0X3F
				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR & (uint8_t) 0x3F);
			}
		}
		pRxBuffer++;
	}
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t txState = pUSARTHandle->txBusyState;

	if (txState != USART_BUSY_IN_TX)
	{
		pUSARTHandle->txLen = len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->txBusyState = USART_BUSY_IN_TX;
		
		// enable interrupt for TXE
		REG_SET_BIT(pUSARTHandle->pUSARTx->CR1, USART_CR1_TXEIE);

		// enable interrupt for TC
		REG_SET_BIT(pUSARTHandle->pUSARTx->CR1, USART_CR1_TCIE);
	}

	return txState;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t rxState = pUSARTHandle->rxBusyState;

	if (rxState != USART_BUSY_IN_RX)
	{
		pUSARTHandle->rxLen = len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->rxBusyState = USART_BUSY_IN_RX;

		// enable interrupt for RXNE
		REG_SET_BIT(pUSARTHandle->pUSARTx->CR1, USART_CR1_RXNEIE);
	}

	return rxState;
}

void USART_IrqInterruptConfig(uint8_t irqNumber, uint8_t en)
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

void USART_IrqPriorityConfig(uint8_t irqNumber, uint32_t irqPriority)
{
	uint8_t iprx = irqNumber / 4;
	uint8_t iprxSection = irqNumber % 4;

	uint8_t shiftAmount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR + iprx) |= irqPriority << shiftAmount;
}

void USART_IrqHandling(USART_Handle_t *pUSARTHandle)
{
	uint16_t *pData;

	/************************* Check for TC flag ********************************************/
	if (REG_TEST_BIT(pUSARTHandle->pUSARTx->ISR, USART_ISR_TC) && REG_TEST_BIT(pUSARTHandle->pUSARTx->CR1, USART_CR1_TCIE))
	{
		// this interrupt is because of TC

		// close transmission and call application callback if txLen is zero
		if (pUSARTHandle->txBusyState == USART_BUSY_IN_TX)
		{
			// check the txLen. If it is zero then close the data transmission
			if (!pUSARTHandle->txLen)
			{
				// clear the TC flag
				REG_SET_BIT(pUSARTHandle->pUSARTx->ICR, USART_ICR_TCCF);

				// reset the application state
				pUSARTHandle->txBusyState = USART_READY;

				// reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				// reset the length to zero
				pUSARTHandle->txLen = 0;

				// call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	/************************* Check for TXE flag ********************************************/
	if (REG_TEST_BIT(pUSARTHandle->pUSARTx->ISR, USART_ISR_TXE) && REG_TEST_BIT(pUSARTHandle->pUSARTx->CR1, USART_CR1_TXEIE))
	{
		// this interrupt is because of TXE
		if (pUSARTHandle->txBusyState == USART_BUSY_IN_TX)
		{
			// keep sending data until txLen reaches to zero
			if (pUSARTHandle->txLen > 0)
			{
				if (pUSARTHandle->config.wordLength == USART_WORDLEN_9BITS)
				{
					// if 9 BIT load the TDR with 2bytes masking the bits other than first 9 bits
					pData = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->TDR = (*pData & (uint16_t) 0x01FF);

					// check for USART_ParityControl
					if (pUSARTHandle->config.parityControl == USART_PARITY_DISABLE)
					{
						// no parity is used in this transfer, so 9 bits of user data will be sent
						// increment pTxBuffer an extra time
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->txLen--;
					}
				}
				else if (pUSARTHandle->config.wordLength == USART_WORDLEN_8BITS)
				{
					// this is 8 bit data transfer
					pUSARTHandle->pUSARTx->TDR = (*pUSARTHandle->pTxBuffer & (uint8_t) 0xFF);
				}
				else
				{
					// this is 7 bit data transfer
					pUSARTHandle->pUSARTx->TDR = (*pUSARTHandle->pTxBuffer & (uint8_t) 0x7F);
				}
				pUSARTHandle->pTxBuffer++;
				pUSARTHandle->txLen--;
			}

			if (pUSARTHandle->txLen == 0)
			{
				// txLen is zero
				// clear the TXEIE bit (disable interrupt for TXE flag)
				REG_CLEAR_BIT(pUSARTHandle->pUSARTx->CR1, USART_CR1_TXEIE);
			}
		}
	}

	/************************* Check for RXNE flag ********************************************/
	if (REG_TEST_BIT(pUSARTHandle->pUSARTx->ISR, USART_ISR_RXNE) && REG_TEST_BIT(pUSARTHandle->pUSARTx->CR1, USART_CR1_RXNEIE))
	{
		// this interrupt is because of rxne
		if (pUSARTHandle->rxBusyState == USART_BUSY_IN_RX)
		{
			if (pUSARTHandle->rxLen > 0)
			{
				if (pUSARTHandle->config.wordLength == USART_WORDLEN_9BITS)
				{
					// we are going to receive 9 bit data in a frame
					if (pUSARTHandle->config.parityControl == USART_PARITY_DISABLE)
					{
						// no parity is used, so all 9 bits will be of user data
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint16_t) 0x01FF);

						// increment the pRxBuffer an extra time
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->rxLen--;
					}
					else
					{
						// parity is used, so 8 bits will be of user data and 1 bit is parity
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->RDR & (uint8_t) 0xFF);
					}
				}
				else if (pUSARTHandle->config.wordLength == USART_WORDLEN_9BITS)
				{
					// we are going to receive 8 bit data in a frame
					if (pUSARTHandle->config.parityControl == USART_PARITY_DISABLE)
					{
						// no parity is used, so all 8 bits will be of user data
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR & (uint8_t) 0xFF);
					}
					else
					{
						// parity is used, so, 7 bits will be of user data and 1 bit is parity
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR & (uint8_t) 0x7F);
					}
				}
				else
				{
					// we are going to receive 7 bit data in a frame

					// now, check are we using USART_ParityControl control or not
					if (pUSARTHandle->config.parityControl == USART_PARITY_DISABLE)
					{
						// no parity is used, so all 7 bits will be of user data
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR & (uint8_t) 0x7F);
					}
					else
					{
						// parity is used, so, 6 bits will be of user data and 1 bit is parity
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR & (uint8_t) 0x3F);
					}
				}
				pUSARTHandle->pRxBuffer++;
				pUSARTHandle->rxLen--;
			}

			if (!pUSARTHandle->rxLen)
			{
				// disable the rxne
				REG_CLEAR_BIT(pUSARTHandle->pUSARTx->CR1, USART_CR1_RXNEIE);
				pUSARTHandle->rxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

	/************************* Check for CTS flag ********************************************/
	if (REG_TEST_BIT(pUSARTHandle->pUSARTx->ISR, USART_ISR_CTS) && REG_TEST_BIT(pUSARTHandle->pUSARTx->CR3, USART_CR3_CTSE))
	{
		// clear the CTS flag
		REG_SET_BIT(pUSARTHandle->pUSARTx->ICR, USART_ICR_CTSCF);

		// this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}

	/************************* Check for IDLE detection flag ********************************************/
	if (REG_TEST_BIT(pUSARTHandle->pUSARTx->ISR, USART_ISR_IDLE) && REG_TEST_BIT(pUSARTHandle->pUSARTx->CR1, USART_CR1_IDLEIE))
	{
		// clear the IDLE flag
		REG_SET_BIT(pUSARTHandle->pUSARTx->ICR, USART_ICR_IDLECF);

		// this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

	/************************* Check for Overrun detection flag ********************************************/
	if (REG_TEST_BIT(pUSARTHandle->pUSARTx->ISR, USART_ISR_ORE) && REG_TEST_BIT(pUSARTHandle->pUSARTx->CR1, USART_CR1_RXNEIE))
	{
		// need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag - TODO check this
		// clear the ORE flag
		REG_SET_BIT(pUSARTHandle->pUSARTx->ICR, USART_ICR_ORECF);

		// this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
	}

	/************************* Check for Error Flag ********************************************/
	// noise flag, Overrun error and Framing Error in multibuffer communication
	if (REG_TEST_BIT(pUSARTHandle->pUSARTx->CR3, USART_CR3_EIE))
	{
		if (REG_TEST_BIT(pUSARTHandle->pUSARTx->ISR, USART_ISR_FE))
		{
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
		}
		
		if (REG_TEST_BIT(pUSARTHandle->pUSARTx->ISR, USART_ISR_NF))
		{
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NE);
		}

		if (REG_TEST_BIT(pUSARTHandle->pUSARTx->ISR, USART_ISR_ORE))
		{
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
		}
	}
}

__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t appEv)
{

}