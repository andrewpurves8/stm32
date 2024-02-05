/*
 * stm32f07xx.h
 *
 *  Created on: Jan 19, 2024
 *      Author: andrew.purves
 */

#ifndef STM32L07XX_H_
#define STM32L07XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex M0+ Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER          						((__vo uint32_t*) 0xE000E100)


/*
 * ARM Cortex M0+ Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER 								((__vo uint32_t*) 0xE000E180)


/*
 * ARM Cortex M0+ Processor Priority Register Address Calculation
 */
#define NVIC_PR			 						((__vo uint32_t*) 0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  				2

/*
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR							0x08000000U
#define SRAM1_BASEADDR							0x20000000U
#define ROM_BASEADDR							0x1FFF0000U
#define SRAM 									SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR 						0x40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR						0x40010000U
#define AHBPERIPH_BASEADDR						0x40020000U
#define IOPORT_BASEADDR							0x50000000U

/*
 * Base addresses of peripherals which are hanging on IOPORT bus
 */
#define GPIOA_BASEADDR                   		(IOPORT_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                   		(IOPORT_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 					 		(IOPORT_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 					 		(IOPORT_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 					 		(IOPORT_BASEADDR + 0x1000)
#define GPIOH_BASEADDR 					 		(IOPORT_BASEADDR + 0x1C00)

/*
 * Base addresses of peripherals which are hanging on AHB bus
 */
#define RCC_BASEADDR                     		(AHBPERIPH_BASEADDR + 0x1000)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR							(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR							(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR							(APB1PERIPH_BASEADDR + 0x7800)

#define SPI2_BASEADDR							(APB1PERIPH_BASEADDR + 0x3800)

#define USART2_BASEADDR							(APB1PERIPH_BASEADDR + 0x4400)
#define USART4_BASEADDR							(APB1PERIPH_BASEADDR + 0x4C00)
#define USART5_BASEADDR							(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define EXTI_BASEADDR							(APB2PERIPH_BASEADDR + 0x0400)
#define SPI1_BASEADDR							(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR        					(APB2PERIPH_BASEADDR + 0x0000)
#define USART1_BASEADDR							(APB2PERIPH_BASEADDR + 0x3800)

///**********************************peripheral register definition structures **********************************/

/*
 * peripheral register definition structure for GPIO
 */
typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
} GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t ICSCR;
	__vo uint32_t CRRCR;
	__vo uint32_t CFGR;
	__vo uint32_t CIER;
	__vo uint32_t CIFR;
	__vo uint32_t CICR;
	__vo uint32_t IOPRSTR;
	__vo uint32_t AHBRSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t APB1RSTR;
	__vo uint32_t IOPENR;
	__vo uint32_t AHBENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t APB1ENR;
	__vo uint32_t IOPSMENR;
	__vo uint32_t AHBSMENR;
	__vo uint32_t APB2SMENR;
	__vo uint32_t APB1SMENR;
	__vo uint32_t CCIPR;
	__vo uint32_t CSR;
} RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
} EXTI_RegDef_t;

/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
} SPI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t CFGR1;
	__vo uint32_t CFGR2;
	__vo uint32_t EXTICR[4];
	__vo uint32_t COMP1_CTRL;
	__vo uint32_t COMP2_CTRL;
	__vo uint32_t CFGR3;
} SYSCFG_RegDef_t;

/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
 	__vo uint32_t CR1;
 	__vo uint32_t CR2;
 	__vo uint32_t OAR1;
 	__vo uint32_t OAR2;
 	__vo uint32_t TIMINGR;
 	__vo uint32_t TIMEOUTR;
 	__vo uint32_t ISR;
 	__vo uint32_t ICR;
 	__vo uint32_t PECR;
 	__vo uint32_t RXDR;
 	__vo uint32_t TXDR;
} I2C_RegDef_t;

/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t BRR;
	__vo uint32_t GTPR;
	__vo uint32_t RTOR;
	__vo uint32_t RQR;
	__vo uint32_t ISR;
	__vo uint32_t ICR;
	__vo uint32_t RDR;
	__vo uint32_t TDR;
} USART_RegDef_t;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA  									((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB  									((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC  									((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD  									((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE  									((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH  									((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC 									((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI									((EXTI_RegDef_t*) EXTI_BASEADDR)
#define SYSCFG									((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1  									((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2  									((SPI_RegDef_t*) SPI2_BASEADDR)

#define I2C1  									((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2  									((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3  									((I2C_RegDef_t*) I2C3_BASEADDR)

#define USART1  								((USART_RegDef_t*) USART1_BASEADDR)
#define USART2  								((USART_RegDef_t*) USART2_BASEADDR)

#define REG_SET_BIT(REG, BIT)					(REG |= (1 << (BIT)))
#define REG_CLEAR_BIT(REG, BIT)					(REG &= ~(1 << (BIT)))
#define REG_TEST_BIT(REG, BIT)					(REG & (1 << (BIT)))

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()    						REG_SET_BIT(RCC->IOPENR, 0)
#define GPIOB_PCLK_EN()							REG_SET_BIT(RCC->IOPENR, 1)
#define GPIOC_PCLK_EN()							REG_SET_BIT(RCC->IOPENR, 2)
#define GPIOD_PCLK_EN()							REG_SET_BIT(RCC->IOPENR, 3)
#define GPIOE_PCLK_EN()							REG_SET_BIT(RCC->IOPENR, 4)
#define GPIOH_PCLK_EN()							REG_SET_BIT(RCC->IOPENR, 7)

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() 							REG_SET_BIT(RCC->APB1ENR, 21)
#define I2C2_PCLK_EN() 							REG_SET_BIT(RCC->APB1ENR, 22)
#define I2C3_PCLK_EN() 							REG_SET_BIT(RCC->APB1ENR, 30)

/*
 * Clock Enable Macros for SPIx peripheralsbu
 */
#define SPI1_PCLK_EN() 							REG_SET_BIT(RCC->APB2ENR, 12)
#define SPI2_PCLK_EN() 							REG_SET_BIT(RCC->APB1ENR, 14)

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCCK_EN() 						REG_SET_BIT(RCC->APB2ENR, 14)
#define USART2_PCCK_EN() 						REG_SET_BIT(RCC->APB1ENR, 17)

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() 						REG_SET_BIT(RCC->APB2ENR, 0)

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()    						REG_CLEAR_BIT(RCC->IOPENR, 0)
#define GPIOB_PCLK_DI()							REG_CLEAR_BIT(RCC->IOPENR, 1)
#define GPIOC_PCLK_DI()							REG_CLEAR_BIT(RCC->IOPENR, 2)
#define GPIOD_PCLK_DI()							REG_CLEAR_BIT(RCC->IOPENR, 3)
#define GPIOE_PCLK_DI()							REG_CLEAR_BIT(RCC->IOPENR, 4)
#define GPIOH_PCLK_DI()							REG_CLEAR_BIT(RCC->IOPENR, 7)

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI() 							REG_CLEAR_BIT(RCC->APB1ENR, 21)
#define I2C2_PCLK_DI() 							REG_CLEAR_BIT(RCC->APB1ENR, 22)
#define I2C3_PCLK_DI() 							REG_CLEAR_BIT(RCC->APB1ENR, 30)

/*
 * Clock Disable Macros for SPIx peripheralsbu
 */
#define SPI1_PCLK_DI() 							REG_CLEAR_BIT(RCC->APB2ENR, 12)
#define SPI2_PCLK_DI() 							REG_CLEAR_BIT(RCC->APB1ENR, 14)

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCCK_DI() 						REG_CLEAR_BIT(RCC->APB2ENR, 14)
#define USART2_PCCK_DI() 						REG_CLEAR_BIT(RCC->APB1ENR, 17)

/*
 * Clock Disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI() 						REG_CLEAR_BIT(RCC->APB2ENR, 0)

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               		do { REG_SET_BIT(RCC->IOPRSTR, 0); REG_CLEAR_BIT(RCC->IOPRSTR, 0); } while (0)
#define GPIOB_REG_RESET()               		do { REG_SET_BIT(RCC->IOPRSTR, 1); REG_CLEAR_BIT(RCC->IOPRSTR, 1); } while (0)
#define GPIOC_REG_RESET()               		do { REG_SET_BIT(RCC->IOPRSTR, 2); REG_CLEAR_BIT(RCC->IOPRSTR, 2); } while (0)
#define GPIOD_REG_RESET()               		do { REG_SET_BIT(RCC->IOPRSTR, 3); REG_CLEAR_BIT(RCC->IOPRSTR, 3); } while (0)
#define GPIOE_REG_RESET()               		do { REG_SET_BIT(RCC->IOPRSTR, 4); REG_CLEAR_BIT(RCC->IOPRSTR, 4); } while (0)
#define GPIOH_REG_RESET()               		do { REG_SET_BIT(RCC->IOPRSTR, 7); REG_CLEAR_BIT(RCC->IOPRSTR, 7); } while (0)

/*
 * This macro returns a code (between 0 and 5) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      			((x == GPIOA) ? 0 : \
												(x == GPIOB) ? 1 : \
												(x == GPIOC) ? 2 : \
												(x == GPIOD) ? 3 : \
								        		(x == GPIOE) ? 4 : \
								        		(x == GPIOH) ? 5 : 0)

/*
 * IRQ Numbers of STM32L07xx MCU
 */
#define IRQ_NO_EXTI1_0 							5
#define IRQ_NO_EXTI3_2 							6
#define IRQ_NO_EXTI15_4 						7
#define IRQ_NO_I2C3     						21
#define IRQ_NO_I2C1     						23
#define IRQ_NO_I2C2     						24
#define IRQ_NO_SPI1								25
#define IRQ_NO_SPI2         					26
#define IRQ_NO_USART1	    					27
#define IRQ_NO_USART2	    					28

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    						0
#define NVIC_IRQ_PRI1    						1
#define NVIC_IRQ_PRI2    						2
#define NVIC_IRQ_PRI3    						3

//some generic macros

#define TRUE 									1
#define FALSE	 								0
#define ENABLE 									TRUE
#define DISABLE 								FALSE
#define SET 									TRUE
#define RESET 									FALSE

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 		0
#define SPI_CR1_CPOL      				 		1
#define SPI_CR1_MSTR     				 		2
#define SPI_CR1_BR   					 		3
#define SPI_CR1_SPE     				 		6
#define SPI_CR1_LSBFIRST   			 	 		7
#define SPI_CR1_SSI     				 		8
#define SPI_CR1_SSM      				 		9
#define SPI_CR1_RXONLY      		 			10
#define SPI_CR1_DFF     			 			11
#define SPI_CR1_CRCNEXT   			 			12
#define SPI_CR1_CRCEN   			 			13
#define SPI_CR1_BIDIOE     			 			14
#define SPI_CR1_BIDIMODE      					15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 					0
#define SPI_CR2_TXDMAEN				 			1
#define SPI_CR2_SSOE				 			2
#define SPI_CR2_FRF								4
#define SPI_CR2_ERRIE							5
#define SPI_CR2_RXNEIE				 			6
#define SPI_CR2_TXEIE							7

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE								0
#define SPI_SR_TXE					 			1
#define SPI_SR_CHSIDE					 		2
#define SPI_SR_UDR						 		3
#define SPI_SR_CRCERR					 		4
#define SPI_SR_MODF						 		5
#define SPI_SR_OVR						 		6
#define SPI_SR_BSY						 		7
#define SPI_SR_FRE						 		8

/******************************************************************************************
 * Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE								0
#define I2C_CR1_TXIE							1
#define I2C_CR1_RXIE							2
#define I2C_CR1_ADDRIE							3
#define I2C_CR1_NACKIE							4
#define I2C_CR1_STOPIE							5
#define I2C_CR1_TCIE							6
#define I2C_CR1_ERRIE							7
#define I2C_CR1_DNF								8

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_ADD0    				 		0
#define I2C_CR2_ADD71 				 	 		1
#define I2C_CR2_ADD98  			 	 			8
#define I2C_CR2_RDWRN 							10
#define I2C_CR2_ADD10  			 	 			11
#define I2C_CR2_START 							13
#define I2C_CR2_STOP  				 			14
#define I2C_CR2_NBYTES 							16

/*
 * Bit position definitions I2C_TIMINGR
 */
#define I2C_TIMINGR_SCLL						0
#define I2C_TIMINGR_SCLH						8
#define I2C_TIMINGR_SDADEL						16
#define I2C_TIMINGR_SCLDEL						20
#define I2C_TIMINGR_PRESC						28

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 		0
#define I2C_OAR1_ADD71 				 	 		1
#define I2C_OAR1_ADD98  			 	 		8
#define I2C_OAR1_EN		   			 			15

/*
 * Bit position definitions I2C_ISR
 */
#define I2C_ISR_TXE   			 				0
#define I2C_ISR_TXIS   			 				1
#define I2C_ISR_RXNE   			 				2
#define I2C_ISR_ADDR   			 				3
#define I2C_ISR_NACKF  			 				4
#define I2C_ISR_STOPF  			 				5
#define I2C_ISR_TC   			 				6
#define I2C_ISR_BERR   			 				8
#define I2C_ISR_ARLO   			 				9
#define I2C_ISR_OVR   			 				10
#define I2C_ISR_TIMEOUT			 				12
#define I2C_ISR_BUSY   			 				15

/*
 * Bit position definitions I2C_ICR
 */
#define I2C_ICR_ADDRCF 			 				3
#define I2C_ICR_NACKCF 			 				4
#define I2C_ICR_STOPCF 			 				5
#define I2C_ICR_BERRCF 			 				8
#define I2C_ICR_ARLOCF 			 				9
#define I2C_ICR_OVRCF 			 				10
#define I2C_ICR_TIMEOUTCF		 				12

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
//#define USART_CR1_SBK							0
//#define USART_CR1_RWU 						1
//#define USART_CR1_RE  						2
//#define USART_CR1_TE 							3
//#define USART_CR1_IDLEIE 						4
//#define USART_CR1_RXNEIE  					5
//#define USART_CR1_TCIE						6
//#define USART_CR1_TXEIE						7
//#define USART_CR1_PEIE 						8
//#define USART_CR1_PS 							9
//#define USART_CR1_PCE 						10
//#define USART_CR1_WAKE  						11
//#define USART_CR1_M 							12
//#define USART_CR1_UE 							13
//#define USART_CR1_OVER8  						15

/*
 * Bit position definitions USART_CR2
 */
//#define USART_CR2_ADD   						0
//#define USART_CR2_LBDL   						5
//#define USART_CR2_LBDIE  						6
//#define USART_CR2_LBCL   						8
//#define USART_CR2_CPHA   						9
//#define USART_CR2_CPOL   						10
//#define USART_CR2_STOP   						12
//#define USART_CR2_LINEN   					14

/*
 * Bit position definitions USART_CR3
 */
//#define USART_CR3_EIE   						0
//#define USART_CR3_IREN   						1
//#define USART_CR3_IRLP  						2
//#define USART_CR3_HDSEL   					3
//#define USART_CR3_NACK   						4
//#define USART_CR3_SCEN   						5
//#define USART_CR3_DMAR  						6
//#define USART_CR3_DMAT   						7
//#define USART_CR3_RTSE   						8
//#define USART_CR3_CTSE   						9
//#define USART_CR3_CTSIE   					10
//#define USART_CR3_ONEBIT   					11

/*
 * Bit position definitions USART_SR
 */
//#define USART_SR_PE        					0
//#define USART_SR_FE        					1
//#define USART_SR_NE        					2
//#define USART_SR_ORE       					3
//#define USART_SR_IDLE       					4
//#define USART_SR_RXNE        					5
//#define USART_SR_TC        					6
//#define USART_SR_TXE        					7
//#define USART_SR_LBD        					8
//#define USART_SR_CTS        					9

#include "stm32l07xx_gpio_driver.h"
#include "stm32l07xx_spi_driver.h"
#include "stm32l07xx_i2c_driver.h"

#endif /* STM32L07XX_H_ */
