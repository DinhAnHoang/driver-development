/*
 * 001_Toggle.c
 *
 *  Created on: Oct 29, 2023
 *      Author: An.HoangDinh
 */
#include "stm32f407xx.h"
void delay_us(int32_t number);
void blinkLed_1(uint32_t delay_time);
void blinkLed_2(uint32_t delay_time);
uint8_t Led_pin[4]  = {GPIO_PIN_NO_12,GPIO_PIN_NO_13,GPIO_PIN_NO_14,GPIO_PIN_NO_15};


int main (void){
	GPIO_Handle_t gpioLed;
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
	while(1)
	{
		blinkLed_1(200000);
		blinkLed_1(200000);
		blinkLed_2(1000000);
	}
	return 0;
}

void delay_us(int32_t number)
{
	for (int i = 0; i<number; i++);
}

void blinkLed_1(uint32_t delay_time)
{
	for(int i = 0; i<4;i++)
	{
		GPIO_ToggleOutputPin(GPIOD,Led_pin[i]);
		delay_us(delay_time);
	}
}

void blinkLed_2(uint32_t delay_time)
{
	for(int i = 0; i<4;i++)
	{
		GPIO_ToggleOutputPin(GPIOD,Led_pin[i]);
	}
	delay_us(delay_time);
}
