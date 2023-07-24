/*
 *  LedButton.c
 *
 *  Created on: Jul 22, 2023
 *  Author: EMRE PEKGUZEL
 */

#include <string.h>
#include "stm32f407xx.h"



void delay()
{
	for(int i = 0; i < 250000; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLed;
	GPIO_Handle_t gpioItButton;

	memset(&gpioLed, 0, sizeof(gpioLed));
	memset(&gpioItButton, 0, sizeof(gpioItButton));

	/* configure output gpio pin */
	gpioLed.pGPIOx = GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULL_NONE;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MS;

	/* configure interrupt button pin */
	gpioItButton.pGPIOx = GPIOA;
	gpioItButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioItButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	gpioItButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULL_NONE;
	gpioItButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MS;

	/* enable peripheral clocks */
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);

	/* initialize gpios */
	GPIO_Init(&gpioLed);
	GPIO_Init(&gpioItButton);

	/* configure IRQs */
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, IRQ_NVIC_PRIO15);
	GPIO_IRQConfig(IRQ_NO_EXTI0, ENABLE);

	for(;;);

	return 0;
}

/* toggle the gpio pin when interrupt is triggered */
void EXTI0_IRQHandler(void)
{
	/* debouncing protection */
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
