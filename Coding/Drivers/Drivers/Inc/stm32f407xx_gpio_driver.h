/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Oct 1, 2023
 *      Author: This PC
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include <stdint.h>
#include "stm32f407xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode; 					/*| Possible values from @GPIO_PIN_MODES 	*/
	uint8_t GPIO_PinSpeed;					/*| Possible values from @GPIO_SPEED_TYPE 	*/
	uint8_t GIPO_PinPuPdControl;			/*| Possible values from @GPIO_PU&PD	 	*/
	uint8_t GPIO_OPType;					/*| Possible values from @GPIO_OUTPUT_TYPE 	*/
	uint8_t GPIO_PinAltFunMode;				/*| Possible values from @GPIO_PIN_MODES 	*/

}GPIO_PinConfig_t;



typedef struct
{
	GPIO_RegDef_t *pGPIOx; 					/*| this hold the base address of GPIO port to which the pin belongs 	*/
	GPIO_PinConfig_t GPIO_PinConfig;		/*| this hold the GPIO pin configuration setting 	*/

}GPIO_Handle_t;




/* *****************************************************************************
 * API supported by this driver
 * For more information about the APIs check the functions definition
 *
 ******************************************************************************/

/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
void GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*
 * Interrupt configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPiority, uint8_t EnorDi);
void GPIO_IRQHandling(void);




#define GPIO_PIN_NO_0 				0
#define GPIO_PIN_NO_1 				1
#define GPIO_PIN_NO_2 				2
#define GPIO_PIN_NO_3 				3
#define GPIO_PIN_NO_4 				4
#define GPIO_PIN_NO_5 				5
#define GPIO_PIN_NO_6 				6
#define GPIO_PIN_NO_7 				7
#define GPIO_PIN_NO_8 				8
#define GPIO_PIN_NO_9 				9
#define GPIO_PIN_NO_10 				10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12 				12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15






/*
 * @GPIO_PIN_MODES
 * GPIO Pin possible modes
 */

#define GPIO_MODE_IN 				0
#define GPIO_MODE_OUT 				1
#define GPIO_MODE_ALTFUNC 			2
#define GPIO_MODE_ANALOG 			3
#define GPIO_MODE_IT_FT 			4   	/*Input falling edge*/
#define GPIO_MODE_IT_RT 			5		/*Input rising edge*/
#define GPIO_MODE_IT_RFT 			6		/*Input rising-falling edge*/

/*
 * @GPIO_OUTPUT_TYPE
 * GPIO Pin possible output type
 */

#define GPIO_OP_TYPE_PP				0     	/*Output type Push-pull*/
#define GPIO_OP_TYPE_OD 			1		/*Output type open drain*/

/*
 * @GPIO_SPEED_TYPE
 * GPIO Pin output speed
 */

#define GPIO_SPEED_LOW				0
#define GPIO_SPEED_MEDIUM			1
#define GPIO_SPEED_HIGH				2
#define GPIO_SPEED_VERYHIGH 		3

/*
 * @GPIO_PU&PD
 * GPIO Pin pull up and pull down configurations macros
 */

#define GPIO_NO_PUPD				0
#define GPIO_PIN_PU					1
#define GPIO_PIN_PD					2
#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
