/*
 * stm32l07xx_dma_driver.h
 *
 *  Created on: Feb 23, 2024
 *      Author: andrew.purves
 */

#ifndef INC_STM32L07XX_DMA_DRIVER_H_
#define INC_STM32L07XX_DMA_DRIVER_H_

#include "stm32l07xx.h"

// /*
//  * Configuration structure for DMAx peripheral
//  */
// typedef struct
// {
// 	uint32_t sclSpeed;
// 	uint8_t ownAddress;
// } DMA_Config_t;

/*
 * Handle structure for DMAx peripheral
 */
typedef struct
{
	uint8_t channel;
	uint8_t channelSelection;
	uint8_t direction;
	uint8_t memoryIncrement;
	uint8_t memorySize;
	uint8_t mode;
	uint8_t peripheralIncrement;
	uint8_t peripheralSize;
	uint8_t priority;
	uint8_t state;
} DMA_Handle_t;

// direction
#define DMA_DIR_READ_FROM_PERIPHERAL 			0
#define DMA_DIR_READ_FROM_MEMORY 				1

// memory increment
#define DMA_MINC_DI 							0
#define DMA_MINC_EN 							1

// memory size
#define DMA_MSIZE_8BITS 						0
#define DMA_MSIZE_16BITS 						1
#define DMA_MSIZE_32BITS 						2

// mode
#define DMA_MODE_NORMAL 						0
#define DMA_MODE_CIRCULAR 						1

// peripheral increment
#define DMA_PINC_DI 							0
#define DMA_PINC_EN 							1

// peripheral size
#define DMA_PSIZE_8BITS 						0
#define DMA_PSIZE_16BITS 						1
#define DMA_PSIZE_32BITS 						2

// priority
#define DMA_PRIO_LOW 							0
#define DMA_PRIO_MEDIUM 						1
#define DMA_PRIO_HIGH 							2
#define DMA_PRIO_VERY_HIGH 						3

// states
#define DMA_READY 								0
#define DMA_BUSY	 							1

// events
#define DMA_EVENT_CMPLT 						0
#define DMA_EVENT_ERR	 						1

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void DMA_PeriClockControl(uint8_t en);

/*
 * Init and De-init
 */
void DMA_Init(DMA_Handle_t *pDMAHandle);
void DMA_DeInit();

/*
 * IRQ Configuration and ISR handling
 */
void DMA_EnableInterrupts(uint8_t channel);
void DMA_DisableInterrupts(uint8_t channel);
void DMA_IrqInterruptConfig(uint8_t irqNumber, uint8_t en);
void DMA_IrqPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void DMA_IrqHandling(DMA_Handle_t *pDMAHandle);

/*
 * Start
 */
uint8_t DMA_StartIT(DMA_Handle_t *pDMAHandle, uint32_t memoryAddress, uint32_t peripheralAddress, uint32_t len);

/*
 * Other Peripheral Control APIs
 */
void DMA_PeripheralControl(uint8_t channel, uint8_t en);

/*
 * Application callback
 */
void DMA_ApplicationEventCallback(DMA_Handle_t *pDMAHandle, uint8_t appEv);

#endif
