/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jul 10, 2023
 *  Author: EMRE PEKGUZEL
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/* configuration structure for a GPIO pin */
typedef struct
{
	uint8_t GPIO_PinNumber;			/* possible pin numbers from @GPIOPN    */
	uint8_t GPIO_PinMode;			/* possible modes from @GPIOM 			*/
	uint8_t GPIO_PinSpeed;			/* possible pin speeds from  @GPIOOS	*/
	uint8_t GPIO_PinPuPdControl;	/* possible pull options from @GPIOPUPD */
	uint8_t GPIO_PinOPType;			/* possible output modes from @GPIOOM 	*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/* handle structure for a GPIO pin */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/* @GPIOPN GPIO pin possible numbers */
#define GPIO_PIN_NO_0	 0
#define GPIO_PIN_NO_1	 1
#define GPIO_PIN_NO_2	 2
#define GPIO_PIN_NO_3	 3
#define GPIO_PIN_NO_4	 4
#define GPIO_PIN_NO_5	 5
#define GPIO_PIN_NO_6	 6
#define GPIO_PIN_NO_7	 7
#define GPIO_PIN_NO_8	 8
#define GPIO_PIN_NO_9	 9
#define GPIO_PIN_NO_10	 10
#define GPIO_PIN_NO_11	 11
#define GPIO_PIN_NO_12	 12
#define GPIO_PIN_NO_13	 13
#define GPIO_PIN_NO_14	 14
#define GPIO_PIN_NO_15	 15

/* @GPIOM GPIO pin possible modes */
#define GPIO_MODE_IN 	 0
#define GPIO_MODE_OUT 	 1
#define GPIO_MODE_ALTFN  2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT  4
#define GPIO_MODE_IT_RT  5
#define GPIO_MODE_IT_RFT 6

/* @GPIOOM GPIO pin possible output modes */
#define GPIO_OP_TYPE_PP  0
#define GPIO_OP_TYPE_OD  1

/* @GPIOOS GPIO pin possible output speeds */
#define GPIO_SPEED_LS	 0
#define GPIO_SPEED_MS	 1
#define GPIO_SPEED_HS	 2
#define GPIO_SPEED_VHS	 3

/* @GPIOPUPD GPIO pin possible pull up/down configs */
#define GPIO_PULL_NONE	 0
#define GPIO_PULL_UP	 1
#define GPIO_PULL_DOWN	 2


/*********************************APIs SUPPORTED BY THIS DRIVER*********************************/

/* peripheral clock setup */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/* init and de-init */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* data read and write */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t EnorDi);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* IRQ configuration and ISR handling */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */


