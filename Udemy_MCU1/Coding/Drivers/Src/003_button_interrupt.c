/*
 * 003_button_interrupt.c
 *
 *  Created on: Apr 7, 2024
 *      Author: This PC
 */

#include "stm32f407xx.h"
#define BUTTON_PRESS 0
void delay_us(int32_t number);
uint8_t Led_pin[4]  = {GPIO_PIN_NO_12,GPIO_PIN_NO_13,GPIO_PIN_NO_14,GPIO_PIN_NO_15};


int main (void){
	GPIO_Handle_t gpioLed,GPIOBtn;
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
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.GPIO_PinConfig.GIPO_PinPuPdControl = GPIO_PIN_PD;

	GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_NO_12,1);

	GPIO_Init(&GPIOBtn);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0,15U);

	while(1)
	{

	}
	return 0;
}
void EXTI0_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
}
void delay_us(int32_t number)
{
	for (int i = 0; i<number; i++);
}
