/*
 * 002_Button_LED.c
 *
 *  Created on: Nov 11, 2023
 *      Author: An.DinhHoang
 */


#include "stm32f407xx.h"
#define BUTTON_PRESS 1
void delay_us(int32_t number);
uint8_t Led_pin[4]  = {GPIO_PIN_NO_12,GPIO_PIN_NO_13,GPIO_PIN_NO_14,GPIO_PIN_NO_15};


int main (void){
	uint8_t a = 0;
	GPIO_Handle_t gpioLed;
	GPIO_Handle_t GPIOBtn;
	gpioLed.pGPIOx = GPIOD;
	GPIO_PeriClockControl(GPIOD, ENABLE);
	for(int i = 0; i<4;i++)
	{
		gpioLed.GPIO_PinConfig.GPIO_PinNumber = Led_pin[i];
		gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
		gpioLed.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE_PP;
		gpioLed.GPIO_PinConfig.GIPO_PinPuPdControl = GPIO_NO_PUPD;
		GPIO_Init(&gpioLed);
	}

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIOBtn.pGPIOx = GPIOA;
	/*configuration Button*/
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.GPIO_PinConfig.GIPO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	while(1)
	{
		GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0);
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BUTTON_PRESS)
		{
			delay_us(25);
			if(a ==1){
				GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
			}
		}
	}
	return 0;
}

void delay_us(int32_t number)
{
	for (int i = 0; i<number; i++);
}
