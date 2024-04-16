/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jan 10, 2024
 *      Author: This PC
 */

#include "stm32f407xx_spi_driver.h"
/**
  * @brief 		- This function enable or disnable peripheral clock for the given GPIO port
  *
  * @param[in]  - Base address of GPIO port peripheral
  * @param[in]  - ENABLE or DISABLE macro
  *
  * @retval 	- none
  *
  * @note		-
  */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else
		{
			SPI3_PCLK_EN();
		}

	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else
		{
			SPI3_PCLK_DI();
		}
	}
}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	/*Configuration SPI_CR1 register*/
	uint32_t tempReg = 0;
	/*1. configure the device mode*/

	tempReg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_REG_CR1_MSTR;

	/*2. configure the bus configure*/
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		/*bidi mode should be clear*/
		tempReg &= ~(1<<SPI_REG_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		/*bidi mode should be set*/
		tempReg |= (SET << SPI_REG_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		/*bidi mode should be clear*/
		tempReg &= ~(SET << SPI_REG_CR1_BIDIMODE);
		/*RXONLY must be set*/
		tempReg |= (SET << SPI_REG_CR1_RXONLY);
	}
	else
	{
		/*ERROR*/
	}
	/*3. configure the SPI Serial clock speed (baud rate)*/
	tempReg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_REG_CR1_BR;

	/*4. configure the DFF*/
	tempReg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_REG_CR1_DFF;

	/*5. configure the DFF*/
	tempReg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_REG_CR1_CPOL;

	/*6. configure the DFF*/
	tempReg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_REG_CR1_CPHA;

	/*Save all data to register CR1*/
	pSPIHandle->pSPIx->CR1 = tempReg;
}

//void SPI_DeInit(SPI_Handle_t *pSPIx)
//{
//	if(pSPIx == SPI1)
//	{
//		SPI1_REG_RESET();
//	}else if(pSPIx == SPI2)
//	{
//		SPI2_REG_RESET();
//	}else
//	{
//		SPI3_REG_RESET();
//	}
//}



