/*
 * stm32f407xx_flash_driver.h
 *
 *  Created on: Apr 9, 2024
 *      Author: This PC
 */

#ifndef INC_STM32F407XX_FLASH_DRIVER_H_
#define INC_STM32F407XX_FLASH_DRIVER_H_
#include "stm32f407xx.h"

/*KEY to unlock flash configuration */
#define KEY1  0x45670123U
#define KEY2  0xCDEF89ABU

#define SECTOR_0 				0x08000000U
#define SECTOR_1 				(SECTOR_0 + 4000U)

#define FLASH_INTERFACE_BASEADDR  	(AHB1PERIPH_BASE + 0x3C00U)
#define FLASH 						((FLASH_RegDef_t*) FLASH_INTERFACE_BASEADDR)

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}FLASH_Config_t;

typedef struct
{
	FLASH_RegDef_t *pFLASHx; 					/*| this hold the base address of SPIx(x:0,1,2) peripheral 	*/
	FLASH_Config_t FLASH_Config;				/*| this hold the GPIO pin configuration setting 	*/

}FLASH_Handle_t;


void FLASH_Lock ();
void FLASH_UnLock ();
void FLASH_EraseSector(__vio uint32_t Sector);
void FLASH_EraseMass();
void FLASH_Read();
void FLASH_Write1Word(__vio uint32_t Sector,uint32_t data_write );





#endif /* INC_STM32F407XX_FLASH_DRIVER_H_ */
