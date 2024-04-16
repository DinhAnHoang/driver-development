/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Oct 1, 2023
 *      Author: This PC
 */

#include "stm32f407xx_gpio_driver.h"


/**
  * @brief 		-
  * @param[in]  -
  * @param[in]  -
  * @param[in]  -
  *
  * @retval 	- none
  *
  * @note		-
  */

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

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else
		{
			GPIOH_PCLK_EN();
		}

	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else
		{
			GPIOH_PCLK_DI();
		}
	}
}

/**
  * @brief 		- This function setting value to Init GPIO peripheral
  * @param[in]  - Base address of GPIO peripheral and pin configuration setting
  *
  * @retval 	- none
  *
  * @note		- We define some attribute on struct GPIO_PinConfig_t
  */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	uint8_t temp1 = 0;
	uint8_t temp2 = 0;
	/*1. Configure the mode of GPIO pin*/
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <=GPIO_MODE_ANALOG)
	{
		temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else
	{
		/* 1. This part using on interrupt mode*/
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_FT)
		{
			/*This implement code for Interrupt raising edge*/
			EXTI->FTSR |= (0x01<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/*Clear the responding of RTSR prevent for accident*/
			EXTI->RTSR &= ~(0x01<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RT)
		{
			/*This implement code for Interrupt rising edge*/
			EXTI->RTSR |= (0x01<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/*Clear the responding of FTSR prevent for accident*/
			EXTI->FTSR &= ~(0x01<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RFT)
		{
			/*This implement code for both Interrupt rising edge and Interrupt falling edge */
			EXTI->RTSR |= (0x01<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/*Clear the responding of FTSR prevent for accident*/
			EXTI->FTSR |= (0x01<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else
		{
			/*Error input mode*/
		}

		/*2.Configure the GPIO port selection on SYSCFG_EXT1CR*/
		uint8_t ExtiCrReg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /4; /*Get EXTI_CR[ExtiCrReg] 1,2,3 or 4*/
		uint8_t ExticrVal = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %4; /*Get EXTI_CR[ExtiCrReg] 1,2,3 or 4*/
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->SYSCFG_EXTICR[ExtiCrReg] |= portcode  << (ExticrVal*4);

		/*3.Enable the EXTI delivery using IMR */
		EXTI->IMR |= (0x01<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	/*2. Configure speed */
	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed) << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	/*3. Configure PU-PD settings */
	temp = ((pGPIOHandle->GPIO_PinConfig.GIPO_PinPuPdControl) << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	/*4. Configure Output type */
	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_OPType) << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x01U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	/*5. Configure the alt functionality */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUNC)
	{
		/*configure the alt function registers*/
		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) /8; /*Get register AFR [0] or AFR [1]*/
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) %8; /*Get position of bit form 0 - 32*/
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0x0FU << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode) << (4*temp2);
	}

}

/**
  * @brief 		- This function deinit value are setting before
  *
  * @param[in]  - Base address of GPIO peripheral
  * @retval 	- none
  *
  * @note		- none
  */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else
	{
		GPIOH_REG_RESET();
	}
}

/**
  * @brief 		- This function read value from input pin already know
  *
  * @param[in]  - Base address of GPIO peripheral
  * @param[in]  - Position of Pin number
  * @retval 	- 0 or 1
  *
  * @note		- none
  */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >>  PinNumber) & 0x00000001);
	return value;
}

/**
  * @brief 		- This function read value from input pin already know
  *
  * @param[in]  - Base address of GPIO peripheral
  * @param[in]  - Position of Pin number
  * @retval 	-
  *
  * @note		- none
  */
uint8_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint8_t value;
	value = (uint8_t)(pGPIOx->IDR);
	return value;
}

/**
  * @brief 		- This function write value to pin
  *
  * @param[in]  - Base address of GPIO peripheral
  * @param[in]  - Position of Pin number
  * @param[in]  - SET or RESET
  * @retval 	- none
  *
  * @note		- none
  */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		/*Write 1 to output data register at bit field corresponding to the pin number*/
		pGPIOx->ODR |= (0x01<<PinNumber);
	}
	else
	{
		/*Write 0 to output data register at bit field corresponding to the pin number*/
		pGPIOx->ODR &= ~(0x01<<PinNumber);
	}
}

/**
  * @brief 		- This function write value to port
  *
  * @param[in]  - Base address of GPIO peripheral
  * @param[in]  - SET or RESET
  * @retval 	- none
  *
  * @note		- none
  */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/**
  * @brief 		- This function toggle output of pin
  *
  * @param[in]  - Base address of GPIO peripheral
  * @param[in]  - Position of Pin number
  * @retval 	- none
  *
  * @note		- none
  */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR = pGPIOx->ODR ^ (1<< PinNumber); 		/* ^ == XOR  */
	/* Other write:  pGPIOx->ODR ^= (1<< PinNumber) 				 */
}

/*
 * IRQ Configuration and ISR handling
 */
/*****************************************************************
 * @fn          - GPIO_IRQInterruptConfig
 *
 * @brief       - This function configures interrupt
 *
 * @param[in]   - IRQ Interrupt number
 * @param[in]   - Macro: Enable/Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			/*Program ISER0(Interrupt set-enable register) Register  */
			*NVIC_ISER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <=64){
			/*Program ISER1 Register */
			*NVIC_ISER1 |= (1<<(IRQNumber % 32));
		}
		else if(IRQNumber > 64 && IRQNumber <96){
			/*Program ISER2 Register */
			*NVIC_ISER2 |= (1<<(IRQNumber % 64));
		}
	}
	else{
		if(IRQNumber <= 31){
			/*Program ICER0(Interrupt clear-enable register) Register  */
			*NVIC_ICER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <=64){
			*NVIC_ICER1 |= (1<<(IRQNumber % 32));
		}
		else if(IRQNumber > 64 && IRQNumber <96){
			*NVIC_ICER2 |= (1<<(IRQNumber % 64));
		}
	}
}


/*****************************************************************
 * @fn          - GPIO_IRQPriorityConfig
 *
 * @brief       - This function configures interrupt priority
 *
 * @param[in]   - IRQ Interrupt number
 * @param[in]   - IRQ interrupt priority
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


/*****************************************************************
 * @fn          - GPIO_IRQHandling
 *
 * @brief       - This function handle interrupts
 *
 * @param[in]   - Pin number
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
    /* Clear the PR register corresponding to pin number */
    if(EXTI->PR & (1 << PinNumber))
    {
        /* Clear pin */
        EXTI->PR |= (1 << PinNumber);
    }
}
