/*
 *  001LedToggle.c
 *
 *  Created on: Jul 21, 2023
 *  Author: EMRE PEKGUZEL
 */

#include "stm32f407xx.h"

void delay()
{
	for(int i = 0; i < 50000; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLed;

	gpioLed.pGPIOx = GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MS;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULL_NONE;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&gpioLed);

	while(ENABLE)
	{
		delay();

		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
	}

}
