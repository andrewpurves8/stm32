/*
 * stm32l07xx_spi_driver.h
 *
 *  Created on: Jan 24, 2024
 *      Author: andrew.purves
 */

#ifndef INC_STM32L07XX_SPI_DRIVER_H_
#define INC_STM32L07XX_SPI_DRIVER_H_

#include "stm32l07xx.h"
#include "stm32l07xx_dma_driver.h"

/*
 *  Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t deviceMode;
	uint8_t busConfig;
	uint8_t sclkSpeed;
	uint8_t dff;
	uint8_t cpol;
	uint8_t cpha;
	uint8_t ssm;
	uint8_t dma;
} SPI_Config_t;

/*
 *Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t spiConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t txLen;
	uint32_t rxLen;
	uint8_t txState;
	uint8_t rxState;
} SPI_Handle_t;

/*
 * SPI application states
 */
#define SPI_READY 								0
#define SPI_BUSY_IN_RX 							1
#define SPI_BUSY_IN_TX 							2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT 						1
#define SPI_EVENT_RX_CMPLT 						2
#define SPI_EVENT_OVR_ERR 						3
#define SPI_EVENT_CRC_ERR 						4

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_SLAVE					0
#define SPI_DEVICE_MODE_MASTER 					1

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD 						1
#define SPI_BUS_CONFIG_HD 						2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 			3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2 					0
#define SPI_SCLK_SPEED_DIV4 					1
#define SPI_SCLK_SPEED_DIV8 					2
#define SPI_SCLK_SPEED_DIV16 					3
#define SPI_SCLK_SPEED_DIV32 					4
#define SPI_SCLK_SPEED_DIV64 					5
#define SPI_SCLK_SPEED_DIV128 					6
#define SPI_SCLK_SPEED_DIV256 					7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS 							0
#define SPI_DFF_16BITS 							1

/*
 * @CPOL
 */
#define SPI_CPOL_LOW 							0
#define SPI_CPOL_HIGH 							1

/*
 * @CPHA
 */
#define SPI_CPHA_LOW 							0
#define SPI_CPHA_HIGH 							1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI 								0
#define SPI_SSM_EN 								1

/*
 * @SPI_DMA
 */
#define SPI_DMA_DI 								0
#define SPI_DMA_EN 								1

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t en);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

// whether this sends/receives depends on the configuration of pDMAHandle
// typedef struct DMA_Handle_t DMA_Handle_t;
void SPI_SendDMA(SPI_RegDef_t *pSPIx, DMA_Handle_t *pDMAHandle, uint8_t *pBuffer, uint32_t len);
void SPI_ReceiveDMA(SPI_RegDef_t *pSPIx, DMA_Handle_t *pDMAHandle, uint8_t *pBuffer, uint32_t len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IrqInterruptConfig(uint8_t irqNumber, uint8_t en);
void SPI_IrqPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void SPI_IrqHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t en);
void SPI_SsiConfig(SPI_RegDef_t *pSPIx, uint8_t en);
void SPI_SsoeConfig(SPI_RegDef_t *pSPIx, uint8_t en);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
uint8_t I2C_DeviceMode(I2C_RegDef_t *i2cx);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEv);

#endif /* INC_STM32L07XX_SPI_DRIVER_H_ */
