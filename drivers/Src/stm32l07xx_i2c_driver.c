/*
 * stm32l07xx_i2c_driver.c
 *
 *  Created on: Jan 25, 2024
 *      Author: andrew.purves
 */

#include "stm32l07xx_i2c_driver.h"

static uint32_t RCC_GetPCLK1Value(void);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_SetAddressWrite(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void I2C_SetAddressRead(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx);
static void I2C_SetNBytes(I2C_RegDef_t *pI2Cx, uint8_t nBytes);

static void I2C_HandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_HandleTXISInterrupt(I2C_Handle_t *pI2CHandle);

static void I2C_HandleIrqEvents(I2C_Handle_t *pI2CHandle);
static void I2C_HandleIrqErrors(I2C_Handle_t *pI2CHandle);

static uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
static uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};

static uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, systemClk;

	uint8_t clkSrc, temp, ahbp, apb1p;

	clkSrc = ((RCC->CFGR >> 2) & 0x3);

	if (clkSrc == 0)
	{
		systemClk = 16000000;
	}
	else if (clkSrc == 1)
	{
		systemClk = 8000000;
	}
	else if (clkSrc == 2)
	{
		// systemClk = RCC_GetPLLOutputClock();
		// TODO: calc PLL clock
		systemClk = 0;
	}

	// ahb
	temp = ((RCC->CFGR >> 4) & 0xF);
	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}

	// apb1
	temp = ((RCC->CFGR >> 10 ) & 0x7);
	if (temp < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = (systemClk / ahbp) / apb1p;

	return pclk1;
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	REG_SET_BIT(pI2Cx->CR2, I2C_CR2_START);
}

static void I2C_SetAddressWrite(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
	REG_CLEAR_BIT(pI2Cx->CR2, I2C_CR2_ADD10);
	slaveAddr = slaveAddr << I2C_CR2_ADD71;
	pI2Cx->CR2 |= slaveAddr;
	REG_CLEAR_BIT(pI2Cx->CR2, I2C_CR2_RDWRN);
}

static void I2C_SetAddressRead(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
	REG_CLEAR_BIT(pI2Cx->CR2, I2C_CR2_ADD10);
	slaveAddr = slaveAddr << I2C_CR2_ADD71;
	pI2Cx->CR2 |= slaveAddr;
	REG_SET_BIT(pI2Cx->CR2, I2C_CR2_RDWRN);
}

static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx)
{
	REG_SET_BIT(pI2Cx->ICR, I2C_ICR_ADDRCF);
}

static void I2C_SetNBytes(I2C_RegDef_t *pI2Cx, uint8_t nBytes)
{
	pI2Cx->CR2 |= nBytes << I2C_CR2_NBYTES;
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	REG_SET_BIT(pI2Cx->CR2, I2C_CR2_STOP);
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t en)
{
	if (en == ENABLE)
	{
		REG_SET_BIT(pI2Cx->CR1, I2C_CR1_PE);
	}
	else
	{
		REG_CLEAR_BIT(pI2Cx->CR1, I2C_CR1_PE);
	}
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t en)
{
	if (en == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t regValue = 0;

	// enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// TIMINGR setup as per table 130 of reference manual RM0367
	uint32_t pclkSpeed = RCC_GetPCLK1Value();
	if (pI2CHandle->config.sclSpeed == I2C_SCL_SPEED_FM)
	{
		if (pclkSpeed == 8000000)
		{
			regValue |= 0x0 << I2C_TIMINGR_PRESC;
		}
		else
		{
			// assume 16MHz
			regValue |= 0x1 << I2C_TIMINGR_PRESC;
		}
		regValue |= 0x9 << I2C_TIMINGR_SCLL;
		regValue |= 0x3 << I2C_TIMINGR_SCLH;
		regValue |= 0x2 << I2C_TIMINGR_SDADEL;
		regValue |= 0x3 << I2C_TIMINGR_SCLDEL;
	}
	else
	{
		if (pclkSpeed == 8000000)
		{
			regValue |= 0x1 << I2C_TIMINGR_PRESC;
		}
		else
		{
			// assume 16MHz
			regValue |= 0x3 << I2C_TIMINGR_PRESC;
		}
		regValue |= 0x13 << I2C_TIMINGR_SCLL;
		regValue |= 0xF << I2C_TIMINGR_SCLH;
		regValue |= 0x2 << I2C_TIMINGR_SDADEL;
		regValue |= 0x4 << I2C_TIMINGR_SCLDEL;
	}
	pI2CHandle->pI2Cx->TIMINGR = regValue;

	// hardcoded TIMINGR value for 100kHz, 900ns rise time and 250ns fall time from STMCube generated code
	// pI2CHandle->pI2Cx->TIMINGR = (uint32_t)0x00200607;

    // program the device own address
	regValue = 0;
	regValue |= pI2CHandle->config.deviceAddress << I2C_OAR1_ADD71;
	pI2CHandle->pI2Cx->OAR1 = regValue;
	REG_SET_BIT(pI2CHandle->pI2Cx->OAR1, I2C_OAR1_EN);
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	I2C_PeriClockControl(pI2Cx, DISABLE);
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint8_t len, uint8_t slaveAddr)
{
	// confirm that bus is not busy
	while (REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_BUSY));

	// send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_SetAddressWrite(pI2CHandle->pI2Cx, slaveAddr);

	I2C_SetNBytes(pI2CHandle->pI2Cx, len);

	// generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// confirm that start generation is completed by checking the BUSY flag in the ISR
	while (!REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_BUSY));

	// confirm that address phase is completed by checking the START flag in CR2
	while (REG_TEST_BIT(pI2CHandle->pI2Cx->CR2, I2C_CR2_START));

	// send the data until len becomes 0
	while (len > 0)
	{
		while (!REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_TXE)); // wait till TXE is setup
		pI2CHandle->pI2Cx->TXDR = *pTxbuffer;
		pTxbuffer++;
		len--;
	}

	// when len becomes zero wait for TXE=1 and TC=1 before generating the STOP condition
	// Note: TXE=1, TC=1, means that both SR and DR are empty and next transmission should begin
	// when TC=1 SCL will be stretched (pulled to LOW)

	while (!REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_TXE));
	while (!REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_TC));

	// generate STOP condition and master need not wait for the completion of stop condition
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr)
{
	// confirm that bus is not busy
	while (REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_BUSY));

	// send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_SetAddressRead(pI2CHandle->pI2Cx, slaveAddr);

	I2C_SetNBytes(pI2CHandle->pI2Cx, len);
	
	// generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// confirm that start generation is completed by checking the BUSY flag in the SR1
	while (!REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_BUSY));

	// confirm that address phase is completed by checking the START flag in CR2
	while (REG_TEST_BIT(pI2CHandle->pI2Cx->CR2, I2C_CR2_START));

	// procedure to read only 1 byte from slave
	if (len == 1)
	{
		// wait until RXNE becomes 1
		while (!REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_RXNE));

		// generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->RXDR;
	}
	else if (len > 1)
	{
		// read the data until Len becomes zero
		for (uint32_t i = len; i > 0; i--)
		{
			// wait until RXNE becomes 1
			while (!REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_RXNE));

			if (i == 2) // if last 2 bytes are remaining
			{
				// generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			// read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->RXDR;

			// increment the buffer address
			pRxBuffer++;
		}
	}
}

void I2C_EnableInterrupts(I2C_RegDef_t *pI2Cx)
{
	REG_SET_BIT(pI2Cx->CR1, I2C_CR1_TXIE);
	REG_SET_BIT(pI2Cx->CR1, I2C_CR1_RXIE);
	REG_SET_BIT(pI2Cx->CR1, I2C_CR1_ADDRIE);
	REG_SET_BIT(pI2Cx->CR1, I2C_CR1_NACKIE);
	REG_SET_BIT(pI2Cx->CR1, I2C_CR1_STOPIE);
	REG_SET_BIT(pI2Cx->CR1, I2C_CR1_TCIE);
	REG_SET_BIT(pI2Cx->CR1, I2C_CR1_ERRIE);
}

void I2C_DisableInterrupts(I2C_RegDef_t *pI2Cx)
{
	REG_CLEAR_BIT(pI2Cx->CR1, I2C_CR1_TXIE);
	REG_CLEAR_BIT(pI2Cx->CR1, I2C_CR1_RXIE);
	REG_CLEAR_BIT(pI2Cx->CR1, I2C_CR1_ADDRIE);
	REG_CLEAR_BIT(pI2Cx->CR1, I2C_CR1_NACKIE);
	REG_CLEAR_BIT(pI2Cx->CR1, I2C_CR1_STOPIE);
	REG_CLEAR_BIT(pI2Cx->CR1, I2C_CR1_TCIE);
	REG_CLEAR_BIT(pI2Cx->CR1, I2C_CR1_ERRIE);
}

void I2C_IrqInterruptConfig(uint8_t irqNumber, uint8_t en)
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

void I2C_IrqPriorityConfig(uint8_t irqNumber, uint32_t irqPriority)
{
	// find out the ipr register
	uint8_t iprx = irqNumber / 4;
	uint8_t iprxSection = irqNumber % 4;

	uint8_t shiftAmount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR + iprx) |= irqPriority << shiftAmount;
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr)
{
	uint8_t busyState = pI2CHandle->txRxState;

	if ((busyState != I2C_BUSY_IN_TX) && (busyState != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->txLen = len;
		pI2CHandle->txRxState = I2C_BUSY_IN_TX;
		pI2CHandle->devAddr = slaveAddr;
		pI2CHandle->masterSlave = I2C_MASTER;

		I2C_EnableInterrupts(pI2CHandle->pI2Cx);
		I2C_SetAddressWrite(pI2CHandle->pI2Cx, slaveAddr);
		I2C_SetNBytes(pI2CHandle->pI2Cx, len);
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	}

	return busyState;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr)
{
	uint8_t busyState = pI2CHandle->txRxState;

	if ((busyState != I2C_BUSY_IN_TX) && (busyState != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->rxLen = len;
		pI2CHandle->txRxState = I2C_BUSY_IN_RX;
		pI2CHandle->rxSize = len;
		pI2CHandle->devAddr = slaveAddr;
		pI2CHandle->masterSlave = I2C_MASTER;

		I2C_EnableInterrupts(pI2CHandle->pI2Cx);
		I2C_SetAddressRead(pI2CHandle->pI2Cx, slaveAddr);
		I2C_SetNBytes(pI2CHandle->pI2Cx, len);
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	}

	return busyState;
}

static void I2C_HandleTXISInterrupt(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->txLen > 0)
	{
		pI2CHandle->pI2Cx->TXDR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->txLen--;
		pI2CHandle->pTxBuffer++;
	}
}

static void I2C_HandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	// we have to do the data reception
	if (pI2CHandle->rxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->RXDR;
		pI2CHandle->rxLen--;
	}
	else if (pI2CHandle->rxSize > 1)
	{
		// read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->RXDR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->rxLen--;
	}

	if (pI2CHandle->rxLen == 0)
	{
		// close the I2C data reception and notify the application

		// generate the stop condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		// notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	I2C_DisableInterrupts(pI2CHandle->pI2Cx);

	pI2CHandle->txRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->rxLen = 0;
	pI2CHandle->rxSize = 0;
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	I2C_DisableInterrupts(pI2CHandle->pI2Cx);

	pI2CHandle->txRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->txLen = 0;
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->TXDR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
    return (uint8_t) pI2C->RXDR;
}

void I2C_IrqHandling(I2C_Handle_t *pI2CHandle)
{
	I2C_HandleIrqEvents(pI2CHandle);
	I2C_HandleIrqErrors(pI2CHandle);
}

static void I2C_HandleIrqEvents(I2C_Handle_t *pI2CHandle)
{
	// address matched with own address (slave mode)
	if (REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_ADDR))
	{
		// interrupt is generated because of ADDR event
		I2C_ClearAddrFlag(pI2CHandle->pI2Cx);
	}

	if (REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_TC))
	{
		// TC flag is set
		if (pI2CHandle->txRxState == I2C_BUSY_IN_TX)
		{
			// make sure that TXE is also set
			if (REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_TXE))
			{
				// TC, TXE = 1
				if (pI2CHandle->txLen == 0)
				{
					// generate the STOP condition
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					// reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					// notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if (pI2CHandle->txRxState == I2C_BUSY_IN_RX)
		{
			;
		}
	}

	// handle For interrupt generated by STOPF event
	if (REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_STOPF))
	{
		// STOF flag is set
		// clear the STOPF
		REG_SET_BIT(pI2CHandle->pI2Cx->ICR, I2C_ICR_STOPCF);
		REG_SET_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_TXE);

		// notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	// handle For interrupt generated by TXIS event
	if (REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_TXIS))
	{
		if (pI2CHandle->masterSlave == I2C_MASTER)
		{
			I2C_HandleTXISInterrupt(pI2CHandle);
		}
		else
		{
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
		}
	}

	// handle For interrupt generated by RXNE event
	if (REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_RXNE))
	{
		if (pI2CHandle->masterSlave == I2C_MASTER)
		{
			I2C_HandleRXNEInterrupt(pI2CHandle);
		}
		else
		{
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
		}
	}
}

static void I2C_HandleIrqErrors(I2C_Handle_t *pI2CHandle)
{
	// bus error
	if (REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_BERR))
	{
		// clear the BERR flag
		REG_SET_BIT(pI2CHandle->pI2Cx->ICR, I2C_ICR_BERRCF);

		// notify the application about the error
	   	I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	// arbitration loss error
	if (REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_ARLO))
	{
		// clear the ARLO flag
		REG_SET_BIT(pI2CHandle->pI2Cx->ICR, I2C_ICR_BERRCF);

		// notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	// NACK received
	if (REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_NACKF))
	{
		// clear the NACKF flag
		REG_SET_BIT(pI2CHandle->pI2Cx->ICR, I2C_ICR_NACKCF);

		// notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_NACK);
	}

	// overrun/underrun error
	if (REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_OVR))
	{
		// clear the OVR flag
		REG_SET_BIT(pI2CHandle->pI2Cx->ICR, I2C_ICR_OVRCF);

		// notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	// timeout error
	if (REG_TEST_BIT(pI2CHandle->pI2Cx->ISR, I2C_ISR_TIMEOUT))
	{
		// clear the OVR flag
		REG_SET_BIT(pI2CHandle->pI2Cx->ICR, I2C_ICR_TIMEOUTCF);

		// notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}
