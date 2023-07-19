/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jul 10, 2023
 *  Author: EMRE PEKGUZEL
 */

#include "stm32f407xx_gpio_driver.h"


/* peripheral clock setup */
/**********************************************************************
 *
 *		@func	:	GPIO_PeriClockControl
 *		@brief	:	Enables or disables peripheral clock for given GPIO
 *					ports
 *
 * 		@param	:	Base address of the GPIO peripheral
 * 		@param 	:	Enable or disable macros
 * 		@param 	:	none
 *		@return :	none
 *
 * 		@note	:	none
 * 		@date	:	07.19.23
 *
 **********************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{

}

/* init and de-init */
/**********************************************************************
 *
 *		@func	:	GPIO_Init
 *		@brief	:	Initializes given GPIO pin
 *
 * 		@param	:	Handle structure for a GPIO pin
 * 		@param 	:	none
 * 		@param 	:	none
 *		@return :	none
 *
 * 		@note	:	none
 * 		@date	:	07.19.23
 *
 **********************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

}

/**********************************************************************
 *
 *		@func	:	GPIO_DeInit
 *		@brief	:	Deinitializes given GPIO pin
 *
 * 		@param	:	Base address of the GPIO peripheral
 * 		@param 	:	none
 * 		@param 	:	none
 *		@return :	none
 *
 * 		@note	:	none
 * 		@date	:	07.19.23
 *
 **********************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}

/* data read and write */
/**********************************************************************
 *
 *		@func	:	GPIO_ReadFromInputPin
 *		@brief	:	Reads the value of given GPIO pin
 *
 * 		@param	:	Base address of the GPIO peripheral
 * 		@param 	:	Pin number of given GPIO port
 * 		@param 	:	none
 *		@return :	Read value of given GPIO pin
 *
 * 		@note	:	none
 * 		@date	:	07.19.23
 *
 **********************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/**********************************************************************
 *
 *		@func	:	GPIO_ReadFromInputPort
 *		@brief	:	Reads the value of given GPIO port
 *
 * 		@param	:	Base address of the GPIO peripheral
 * 		@param 	:	none
 * 		@param 	:	none
 *		@return :	Read value of given GPIO port
 *
 * 		@note	:	none
 * 		@date	:	07.19.23
 *
 **********************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

}

/**********************************************************************
 *
 *		@func	:	GPIO_WriteToOutputPin
 *		@brief	:	Writes the desired value to given GPIO pin
 *
 * 		@param	:	Base address of the GPIO peripheral
 * 		@param 	:	Pin number of given GPIO port
 * 		@param 	:	Enable or disable macros
 *		@return :	none
 *
 * 		@note	:	none
 * 		@date	:	07.19.23
 *
 **********************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t EnorDi)
{

}

/**********************************************************************
 *
 *		@func	:	GPIO_WriteToOutputPort
 *		@brief	:	Writes the desired value to given GPIO pin
 *
 * 		@param	:	Base address of the GPIO peripheral
 * 		@param 	:	The value to write
 * 		@param 	:	none
 *		@return :	none
 *
 * 		@note	:	none
 * 		@date	:	07.19.23
 *
 **********************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

}

/**********************************************************************
 *
 *		@func	:	GPIO_ToggleOutputPin
 *		@brief	:	Toggles given GPIO pin
 *
 * 		@param	:	Base address of the GPIO peripheral
 * 		@param 	:	Pin number of given GPIO port
 * 		@param 	:	none
 *		@return :	none
 *
 * 		@note	:	none
 * 		@date	:	07.19.23
 *
 **********************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/* IRQ configuration and ISR handling */
/**********************************************************************
 *
 *		@func	:	GPIO_IRQConfig
 *		@brief	:	Writes the desired value to the GPIO pin
 *
 * 		@param	:	IRQ number for desired interrupt
 * 		@param 	:	Priority value for given interrupt
 * 		@param 	:	Enable or disable macros
 *		@return :	none
 *
 * 		@note	:	none
 * 		@date	:	07.19.23
 *
 **********************************************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

/**********************************************************************
 *
 *		@func	:	GPIO_IRQHandling
 *		@brief	:	Handler function for IRQ
 *
 * 		@param	:	Pin number that triggers the interrupt
 * 		@param 	:	none
 * 		@param 	:	none
 *		@return :	none
 *
 * 		@note	:	none
 * 		@date	:	07.19.23
 *
 **********************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
