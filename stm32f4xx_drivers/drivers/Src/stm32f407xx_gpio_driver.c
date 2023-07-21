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
	if(EnorDi)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
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
	uint32_t temp = 0;
	/* 1. configure the mode of GPIO pin */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		/* non interrupt mode */
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		/* interrupt mode */
		//TODO
	}
	temp = 0;

	/* 2. configure the speed of GPIO pin */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	/* 3. configure the pull up/down state */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	/* 4. configure the output type */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	/* 5. configure the alternate functionality */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		if((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) < GPIO_PIN_NO_8)
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFRL &= ~(0xF<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->AFRL |= temp;
		}
		else
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % GPIO_PIN_NO_8)));
			pGPIOHandle->pGPIOx->AFRH &= ~(0xF << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % GPIO_PIN_NO_8)));
			pGPIOHandle->pGPIOx->AFRH |= temp;
		}
		temp = 0;
	}
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
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
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
 *		@return :	Read value of given GPIO pin (0 or 1)
 *
 * 		@note	:	none
 * 		@date	:	07.19.23
 *
 **********************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	/* correction may be required */
	value = (uint8_t)(pGPIOx->IDR & (1 << PinNumber));
	return value;
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
	uint16_t value;
	/* correction may be required */
	value = (uint16_t)(pGPIOx->IDR);
	return value;
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
	if(EnorDi)
	{
		/* write 1 to the output data register */
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		/* clear the output data register */
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
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
	/* move value to the output register */
	pGPIOx->ODR = Value;
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
	/* exclusive or */
	pGPIOx->ODR ^= (1 << PinNumber);
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
