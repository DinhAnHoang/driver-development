/*
 * stm32f407xx.h
 *
 *  Created on: Sep 24, 2023
 *      Author: An Hoang Dinh
 */
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_


#define __vio volatile
/*
 * Base addresses of Flash and SRAM memories
 *
 */
#define FLASH_BASEADDR  	0x08000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM2_BASEADDR		0x2001C000U     /*| SRAM1 are 112Kb -> SRAM2_BASEADDR = SRAM1_BASEADDR + 112Kb 				*/
#define ROM_BASEADDR		0x1FFF0000U		/*| ROM memories are system memories, it next to with main memories on FLASH*/
#define SRAM 				SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral Base Address
 *
 */
#define PERIPH_BASE		  	0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE 	0x50000000U

/*
 * BASE Addresses of peripherals which are hanging on AHB1 bus
 *  TODO: Complete for all other peripherals
 */
#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR 		(AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASE + 0x2000U)
#define GPIOJ_BASEADDR 		(AHB1PERIPH_BASE + 0x2400U)
#define GPIOK_BASEADDR 		(AHB1PERIPH_BASE + 0x2800U)

#define RCC_BASEADDR 		(AHB1PERIPH_BASE + 0x3800U)

/*
 * BASE Addresses of peripherals which are hanging on APB1 bus
 *  TODO: Complete for all other peripherals
 */

#define I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASEADDR		(APB1PERIPH_BASE + 0x5C00U)

#define SPI1_BASEADDR 		(APB2PERIPH_BASE + 0x3000U)
#define SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASEADDR 		(APB1PERIPH_BASE + 0x3C00U)

#define USART2_BASEADDR 	(APB1PERIPH_BASE + 0x4400U)
#define USART3_BASEADDR 	(APB1PERIPH_BASE + 0x4800U)
#define UART4_BASEADDR 		(APB1PERIPH_BASE + 0x4C00U)
#define UART5_BASEADDR 		(APB1PERIPH_BASE + 0x5000U)

/*
 * BASE Addresses of peripherals which are hanging on APB2 bus
 *  TODO: Complete for all other peripherals
 */


#define USART1_BASEADDR 	(APB2PERIPH_BASE + 0x1000U)
#define USART6_BASEADDR 	(APB2PERIPH_BASE + 0x1400U)

#define EXTI_BASEADDR 		(APB2PERIPH_BASE + 0x3C00U)

#define SYSCFG_BASEADDR 	(APB2PERIPH_BASE + 0x3800U)





/************************************* Peripheral register definition structures ******************************************/
/*
 * NOTE:  Register of a peripheral are specific to MCU
 *  Please check your RM
 */

/*----------------- GPIO -----------------*/
typedef struct
{
	__vio uint32_t MODER;						/*| Address offset: 0x00 	*/
	__vio uint32_t OTYPER;						/*| Address offset: 0x04 	*/
	__vio uint32_t OSPEEDR;						/*| Address offset: 0x08 	*/
	__vio uint32_t PUPDR;						/*| Address offset: 0x0C 	*/
	__vio uint32_t IDR;							/*| Address offset: 0x10 	*/
	__vio uint32_t ODR;							/*| Address offset: 0x14 	*/
	__vio uint32_t BSRR;						/*| Address offset: 0x18 	*/
	__vio uint32_t LCKR;						/*| Address offset: 0x1C 	*/
	__vio uint32_t AFR[2];						/*| Address offset: 0x20 and 0x24 	*/

}GPIO_RegDef_t;
/*----------------- SPI -----------------*/
typedef struct
{
	__vio uint32_t CR1;							/*| Address offset: 0x00 	*/
	__vio uint32_t CR2;							/*| Address offset: 0x04 	*/
	__vio uint32_t SR;							/*| Address offset: 0x08 	*/
	__vio uint32_t DR;							/*| Address offset: 0x0C 	*/
	__vio uint32_t CRCPR;						/*| Address offset: 0x10 	*/
	__vio uint32_t RXCRCR;						/*| Address offset: 0x14 	*/
	__vio uint32_t TXCRCR;						/*| Address offset: 0x18 	*/
	__vio uint32_t I2SCFGR;						/*| Address offset: 0x1C 	*/
	__vio uint32_t I2SPR;						/*| Address offset: 0x20 	*/
}SPI_RegDef_t;

/*----------------- I2C -----------------*/
typedef struct
{
	__vio uint32_t CR1;							/*| Address offset: 0x00 	*/
	__vio uint32_t CR2;							/*| Address offset: 0x04 	*/
	__vio uint32_t OAR1;						/*| Address offset: 0x08 	*/
	__vio uint32_t OAR2;						/*| Address offset: 0x0C 	*/
	__vio uint32_t DR;							/*| Address offset: 0x10 	*/
	__vio uint32_t SR1;							/*| Address offset: 0x14 	*/
	__vio uint32_t SR2;							/*| Address offset: 0x18 	*/
	__vio uint32_t CCR;							/*| Address offset: 0x1C 	*/
	__vio uint32_t TRISE;						/*| Address offset: 0x20 	*/
	__vio uint32_t FLTR;						/*| Address offset: 0x24 	*/
}I2C_RegDef_t;

/*----------------- USART -----------------*/
typedef struct
{
	__vio uint32_t SR;							/*| Address offset: 0x00 	*/
	__vio uint32_t DR;							/*| Address offset: 0x04 	*/
	__vio uint32_t BRR;							/*| Address offset: 0x08 	*/
	__vio uint32_t CR1;							/*| Address offset: 0x0C 	*/
	__vio uint32_t CR2;							/*| Address offset: 0x10 	*/
	__vio uint32_t CR3;							/*| Address offset: 0x14 	*/
	__vio uint32_t GTPR;						/*| Address offset: 0x18 	*/
}USART_RegDef_t;

/*----------------- RCC -----------------*/
typedef struct
{
	__vio uint32_t CR;							/*| Address offset: 0x00 	*/
	__vio uint32_t PLLCFGR;						/*| Address offset: 0x04 	*/
	__vio uint32_t CFGR;						/*| Address offset: 0x08 	*/
	__vio uint32_t CIR;							/*| Address offset: 0x0C 	*/
	__vio uint32_t AHB1RSTR;					/*| Address offset: 0x10 	*/
	__vio uint32_t AHB2RSTR;					/*| Address offset: 0x14 	*/
	__vio uint32_t AHB3RSTR;					/*| Address offset: 0x18 	*/
	 uint32_t RESERVED0;						/*| Address offset: 0x1C 	*/
	__vio uint32_t APB1RSTR;					/*| Address offset: 0x20 	*/
	__vio uint32_t APB2RSTR;					/*| Address offset: 0x24 	*/
	 uint32_t RESERVED1;						/*| Address offset: 0x1C 	*/
	 uint32_t RESERVED2;						/*| Address offset: 0x1C 	*/
	 __vio uint32_t AHB1ENR;					/*| Address offset: 0x30 	*/
	 __vio uint32_t AHB2ENR;					/*| Address offset: 0x34 	*/
	 __vio uint32_t AHB3ENR;					/*| Address offset: 0x38 	*/
	 uint32_t RESERVED3;						/*| Address offset: 0x3C 	*/
	 __vio uint32_t APB1ENR;					/*| Address offset: 0x40 	*/
	 __vio uint32_t APB2ENR;					/*| Address offset: 0x44 	*/
	 uint32_t RESERVED4;						/*| Address offset: 0x48 	*/
	 uint32_t RESERVED5;						/*| Address offset: 0x4C 	*/
	 __vio uint32_t AHB1LPENR;					/*| Address offset: 0x50 	*/
	 __vio uint32_t AHB2LPENR;					/*| Address offset: 0x54 	*/
	 __vio uint32_t AHB3LPENR;					/*| Address offset: 0x58 	*/
	 uint32_t RESERVED6;						/*| Address offset: 0x5C 	*/
	 __vio uint32_t APB1LPENR;					/*| Address offset: 0x60 	*/
	 __vio uint32_t APB2LPENR;					/*| Address offset: 0x64 	*/
	 uint32_t RESERVED7;						/*| Address offset: 0x68 	*/
	 uint32_t RESERVED8;						/*| Address offset: 0x6C 	*/
	 __vio uint32_t BDCR;						/*| Address offset: 0x70 	*/
	 __vio uint32_t CSR;						/*| Address offset: 0x74 	*/
	 uint32_t RESERVED9;						/*| Address offset: 0x78 	*/
	 uint32_t RESERVED10;						/*| Address offset: 0x7C 	*/
	 __vio uint32_t SSCGR;						/*| Address offset: 0x80 	*/
	 __vio uint32_t PLLI2SCFGR;					/*| Address offset: 0x84 	*/

}RCC_RegDef_t;

/*----------------- EXTI -----------------*/
typedef struct
{
	__vio uint32_t IMR;							/*| Address offset: 0x00 	*/
	__vio uint32_t EMR;							/*| Address offset: 0x04 	*/
	__vio uint32_t RTSR;						/*| Address offset: 0x08 	*/
	__vio uint32_t FTSR;						/*| Address offset: 0x0C 	*/
	__vio uint32_t SWIER;						/*| Address offset: 0x10 	*/
	__vio uint32_t PR;							/*| Address offset: 0x14 	*/

}EXTI_RegDef_t;

/*----------------- SYSCFG -----------------*/
typedef struct
{
	__vio uint32_t SYSCFG_MEMRMP;				/*| Address offset: 0x00 	*/
	__vio uint32_t SYSCFG_PMC;					/*| Address offset: 0x04 	*/
	__vio uint32_t SYSCFG_EXTICR[4];			/*| Address offset: 0x08,0x0C,0x10,0x14 	*/
	 uint32_t RESERVED[2];						/*| Address RESERVED  0x18 and 0x1C	*/
	__vio uint32_t APB1RSTR;					/*| Address offset: 0x20 	*/
}SYSCFG_RegDef_t;
/************************************* Peripheral base address definition  ******************************************/
/*
 * NOTE:  Register of a peripheral are specific to MCU
 *  Please check your RM
 */

#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)

/*
 * Clock enable macro for GPIOx peripheral
 */
#define GPIOA_PCLK_EN() 	(RCC->AHB1ENR |=(1<<0))
#define GPIOB_PCLK_EN() 	(RCC->AHB1ENR |=(1<<1))
#define GPIOC_PCLK_EN() 	(RCC->AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN() 	(RCC->AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN() 	(RCC->AHB1ENR |=(1<<4))
#define GPIOF_PCLK_EN() 	(RCC->AHB1ENR |=(1<<5))
#define GPIOG_PCLK_EN() 	(RCC->AHB1ENR |=(1<<6))
#define GPIOH_PCLK_EN() 	(RCC->AHB1ENR |=(1<<7))

/*
 * Clock disable macro for GPIOx peripheral
 */
#define GPIOA_PCLK_DI() 	(RCC->AHB1RSTR |=(1<<0))
#define GPIOB_PCLK_DI() 	(RCC->AHB1RSTR |=(1<<1))
#define GPIOC_PCLK_DI() 	(RCC->AHB1RSTR |=(1<<2))
#define GPIOD_PCLK_DI() 	(RCC->AHB1RSTR |=(1<<3))
#define GPIOE_PCLK_DI() 	(RCC->AHB1RSTR |=(1<<4))
#define GPIOF_PCLK_DI() 	(RCC->AHB1RSTR |=(1<<5))
#define GPIOG_PCLK_DI() 	(RCC->AHB1RSTR |=(1<<6))
#define GPIOH_PCLK_DI() 	(RCC->AHB1RSTR |=(1<<7))

/*
 * Clock enable macro for I2Cx peripheral
 */
#define I2C1_PCLK_EN() 		(RCC->APB1ENR |=(1<<21))
#define I2C2_PCLK_EN() 		(RCC->APB1ENR |=(1<<22))
#define I2C3_PCLK_EN() 		(RCC->APB1ENR |=(1<<23))

/*
 * Clock enable macro for SPIx peripheral
 */
#define SPI1_PCLK_EN() 		(RCC->APB2ENR |=(1<<12))
#define SPI2_PCLK_EN() 		(RCC->APB1ENR |=(1<<14))
#define SPI3_PCLK_EN() 		(RCC->APB1ENR |=(1<<15))

/*
 * Clock disable macro for SPIx peripheral
 */
#define SPI1_PCLK_DI() 	(RCC->APB2RSTR |=(1<<12))
#define SPI2_PCLK_DI() 	(RCC->APB1RSTR |=(1<<14))
#define SPI3_PCLK_DI() 	(RCC->APB1RSTR |=(1<<15))

/*
 * Clock enable macro for USARTx peripheral
 */
#define USART1_PCLK_EN() 	(RCC->APB2ENR |=(1<<4))
#define USART2_PCLK_EN() 	(RCC->APB1ENR |=(1<<17))
#define USART3_PCLK_EN() 	(RCC->APB1ENR |=(1<<18))
#define UART4_PCLK_EN() 	(RCC->APB1ENR |=(1<<19))
#define UART5_PCLK_EN() 	(RCC->APB1ENR |=(1<<20))
#define USART6_PCLK_EN() 	(RCC->APB2ENR |=(1<<5))

/*
 * Clock unable macro for USARTx peripheral
 */
#define USART1_PCLK_EN() 	(RCC->APB2ENR |=(1<<4))
#define USART2_PCLK_EN() 	(RCC->APB1ENR |=(1<<17))
#define USART3_PCLK_EN() 	(RCC->APB1ENR |=(1<<18))
#define UART4_PCLK_EN() 	(RCC->APB1ENR |=(1<<19))
#define UART5_PCLK_EN() 	(RCC->APB1ENR |=(1<<20))
#define USART6_PCLK_EN() 	(RCC->APB2ENR |=(1<<5))


/*
 * Clock disable macro for GPIOx peripheral
 */
#define GPIOA_REG_RESET() 	do{(RCC->AHB1RSTR |=(1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET() 	do{(RCC->AHB1RSTR |=(1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET() 	do{(RCC->AHB1RSTR |=(1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET() 	do{(RCC->AHB1RSTR |=(1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET() 	do{(RCC->AHB1RSTR |=(1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET() 	do{(RCC->AHB1RSTR |=(1<<5)); (RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET() 	do{(RCC->AHB1RSTR |=(1<<6)); (RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET() 	do{(RCC->AHB1RSTR |=(1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)

/*
 * Clock disable macro for SPIx peripheral
 */
#define SPI1_REG_RESET() 	do{(RCC->APB2RSTR |=(1<<12)); (RCC->AHB1RSTR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET() 	do{(RCC->APB1RSTR |=(1<<14)); (RCC->AHB1RSTR &= ~(1<<14));}while(0)
#define SPI3_REG_RESET() 	do{(RCC->AHB1RSTR |=(1<<15)); (RCC->AHB1RSTR &= ~(1<<15));}while(0)


/*
 * Clock enable macro for SYSCONFIG peripheral
 */
#define SYSCFG_PCLK_EN() 		(RCC->APB2ENR |=(1<<14))

#define GPIO_BASEADDR_TO_CODE(x) 	  (	(x==GPIOA)?0:\
										(x==GPIOB)?1:\
										(x==GPIOC)?2:\
										(x==GPIOD)?3:\
										(x==GPIOE)?4:\
										(x==GPIOF)?5:\
										(x==GPIOG)?6:\
										(x==GPIOH)?7:0)

/*
 * IRQ(Interrupt request) Number of STM32F407x MCU
 */
#define IRQ_NO_EXTI0 					 6
#define IRQ_NO_EXTI1 					 7
#define IRQ_NO_EXTI2 					 8
#define IRQ_NO_EXTI3 					 9
#define IRQ_NO_EXTI4 					 10
#define IRQ_NO_EXTI9_5 					 23
#define IRQ_NO_EXTI15_10 				 40

/***********************************************************
 * SPI register macro
 **********************************************************/

/*CR1 Register*/
#define SPI_REG_CR1_BIDIMODE 				15
#define SPI_REG_CR1_BIDIOE 					14
#define SPI_REG_CR1_CRCEN 					13
#define SPI_REG_CR1_CRCNEXT 				12
#define SPI_REG_CR1_DFF						11
#define SPI_REG_CR1_RXONLY					10
#define SPI_REG_CR1_SSM 					9
#define SPI_REG_CR1_SSI						8
#define SPI_REG_CR1_LSBFIRST 				7
#define SPI_REG_CR1_SPE						6
#define SPI_REG_CR1_BR						3
#define SPI_REG_CR1_MSTR					2
#define SPI_REG_CR1_CPOL					1
#define SPI_REG_CR1_CPHA					0


/*CR2 Register*/

#define SPI_REG_CR2_TXEIE 					7
#define SPI_REG_CR2_RXNEIE					6
#define SPI_REG_CR2_ERRIE 					5
#define SPI_REG_CR2_FRF						4
#define SPI_REG_CR2_SSOE					2
#define SPI_REG_CR2_TXDMAEN					1
#define SPI_REG_CR2_RXDMAEN					0

/*SR Register*/
#define SPI_REG_SR_FRE 						8
#define SPI_REG_SR_BSY 						7
#define SPI_REG_SR_OVR						6
#define SPI_REG_SR_MODF 					5
#define SPI_REG_SR_CRCERR					4
#define SPI_REG_SR_UDR						3
#define SPI_REG_SR_CHSIDE					3
#define SPI_REG_SR_TXE						1
#define SPI_REG_SR_RXNE						0
/*
 * Some generic macro
 */

#define ENABLE 						1
#define DISABLE 					0

#define SET 						ENABLE
#define RESET 						DISABLE

#define GPIO_PIN_SET 				SET
#define GPIO_PIN_RESET 				RESET













#endif /* INC_STM32F407XX_H_ */
