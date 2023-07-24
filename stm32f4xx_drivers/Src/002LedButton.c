/*
 *  LedButton.c
 *
 *  Created on: Jul 21, 2023
 *  Author: EMRE PEKGUZEL
 */

#include "stm32f407xx.h"

int main(void)
{
	GPIO_Handle_t gpioLed, gpioButton;

	gpioLed.pGPIOx = GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MS;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULL_NONE;

	gpioButton.pGPIOx = GPIOA;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MS;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULL_NONE;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioLed);
	GPIO_Init(&gpioButton);

	while(ENABLE)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
		{
			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_13, ENABLE);
		}
		else
		{
			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_13, DISABLE);
		}
	}

}
