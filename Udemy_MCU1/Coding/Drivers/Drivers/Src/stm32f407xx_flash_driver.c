/*
 * stm32f407xx_flash_driver.c
 *
 *  Created on: Apr 9, 2024
 *      Author: This PC
 */


#include "stm32f407xx_flash_driver.h"


void FLASH_UnLock (){
	if((FLASH->CR & (1<<31)) != 0)
	{
		FLASH->KEYR = KEY1;
		FLASH->KEYR = KEY2;
	}
	else
	{
		/**/
	}
}

void FLASH_Lock (){
	FLASH->CR |= (1<<31);
}

void FLASH_EraseMass(){

	/*1. Waiting for bit BSY to 0*/
	while ((FLASH->SR & (1 << 16)) != 0x00){
		/*Have another action with flash memory and waiting it by check bit BSY */
	}
	/*2. Set bit MER - Mass Erase, của thanh ghi FLASH_CR để xóa toàn bộ. */
	FLASH->CR |= 1<<2;

	/*4. set bit STRT to start erase */
	FLASH->CR |= 1<<16;

	/*5. Waiting for bit BSY to 0*/
	while ((FLASH->SR & (1 << 16)) != 0x00){
		/*Have another action with flash memory and waiting it by check bit BSY */
	}
}

void FLASH_EraseSector( __vio uint32_t Sector){

	/*1. Waiting for bit BSY to 0*/
	while ((FLASH->SR & (1 << 16)) != 0x00){
		/*Have another action with flash memory and waiting it by check bit BSY */
	}
	/*2. Set bit SER - Sector Erase, của thanh ghi FLASH_CR. */
	FLASH->CR |= 1<<1;

	/*3. Choice sector to Erase. */
	FLASH->CR |= Sector<<3;

	/*4. set bit STRT to start erase */
	FLASH->CR |= 1<<16;

	/*5. Waiting for bit BSY to 0*/
	while ((FLASH->SR & (1 << 16)) != 0x00){
		/*Have another action with flash memory and waiting it by check bit BSY */
	}
}

void FLASH_Write1Word(__vio uint32_t Sector,uint32_t data_write ){
	/*1. Waiting for bit BSY to 0*/
	/*Have another action with flash memory and waiting it by check bit BSY */
	while ((FLASH->SR & (1 << 16)) != 0x00){}

	/*2. Set bit PRG, của thanh ghi FLASH_CR. */
		FLASH->CR |= 1;

	/*3. These bits select the program parallelism.
			00 program x8
			01 program x16
			10 program x32
			11 program x64
	 	 	*/
		FLASH->CR |= (0x02 << 8);

	/*4. Write data*/
		*(uint32_t*) Sector = data_write;
}
