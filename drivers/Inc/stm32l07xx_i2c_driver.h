/*
 * stm32l07xx_i2c_driver.h
 *
 *  Created on: Jan 25, 2024
 *      Author: andrew.purves
 */

#ifndef INC_STM32L07XX_I2C_DRIVER_H_
#define INC_STM32L07XX_I2C_DRIVER_H_

#include "stm32l07xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t sclSpeed;
	uint8_t deviceAddress;
} I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t txLen;
	uint32_t rxLen;
	uint8_t txRxState;		/* !< communication state > */
	uint8_t devAddr;		/* !< device address > */
	uint32_t rxSize;
	uint8_t masterSlave;	/* !< whether the device is master or slave  > */
} I2C_Handle_t;

/*
 * I2C application states
 */
#define I2C_READY 								0
#define I2C_BUSY_IN_RX 							1
#define I2C_BUSY_IN_TX 							2

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 						100000
#define I2C_SCL_SPEED_FM 						400000

/*
 * I2C application events macros
 */
#define I2C_EV_TX_CMPLT 						0
#define I2C_EV_RX_CMPLT 						1
#define I2C_EV_STOP 							2
#define I2C_ERROR_BERR 							3
#define I2C_ERROR_ARLO 							4
#define I2C_ERROR_NACK 							5
#define I2C_ERROR_OVR 							6
#define I2C_ERROR_TIMEOUT 						7
#define I2C_EV_DATA_REQ 						8
#define I2C_EV_DATA_RCV 						9

#define I2C_SLAVE								0
#define I2C_MASTER								1

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t en);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint8_t len, uint8_t slaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slaveAddr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IrqInterruptConfig(uint8_t irqNumber, uint8_t en);
void I2C_IrqPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void I2C_IrqHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t en);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t en);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t en);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEv);

#endif
