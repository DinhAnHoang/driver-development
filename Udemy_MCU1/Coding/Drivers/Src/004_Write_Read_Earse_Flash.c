/*
 * Write_Read_Earse_Flash.c
 *
 *  Created on: Apr 11, 2024
 *      Author: This PC
 */

#include "stm32f407xx.h"

int main(){
	FLASH_UnLock ();
	FLASH_EraseSector(1);
	FLASH_Write1Word(0x08004000U, 0xAABBCCDD);

}
