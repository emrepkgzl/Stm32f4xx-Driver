/*
 * 005SpiTxonButton.c
 *
 * Created on: Agu 11, 2023
 * Author: PEKGUZEL
 */

/*
 * PB15 MOSI
 * PB14 MISO
 * PB13 SCLK
 * PB12 NSS
 * SPI2 ALT F 15
 */

#include "stm32f407xx.h"
#include <String.h>

char message[] = "Hello World!";

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULL_NONE;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HS;

	/* SCLK init */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	/* MOSI init */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	/* MISO init */
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	/* NSS init */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}



void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CFG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_PCLK_DIV_8;
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_SSI = SPI_SSI_DI;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_DI;
	SPI2Handle.SPI_Config.SPI_SSOE = SPI_SSOE_EN;


	SPI_Init(&SPI2Handle);
}

void ItButton_GPIOInits(void)
{
		GPIO_Handle_t gpioItButton;

		/* configure interrupt button pin */
		gpioItButton.pGPIOx = GPIOA;
		gpioItButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
		gpioItButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
		gpioItButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULL_NONE;
		gpioItButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MS;

		/* initialize gpios */
		GPIO_Init(&gpioItButton);

		/* configure IRQs */
		GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, IRQ_NVIC_PRIO15);
		GPIO_IRQConfig(IRQ_NO_EXTI0, ENABLE);
}

void delay()
{
	for(int i = 0; i < 250000; i++);
}

int main(void)
{
	/* this function will be used to initialize GPIO pins to behave as SPI2 pins */
	SPI2_GPIOInits();

	/* this function will be used to initialize SPI2 peripheral parameters */
	SPI2_Inits();

	/* this function will be used to initialize user button as an interrupt */
	ItButton_GPIOInits();

	while(1);

	return 0;
}


/* toggle the gpio pin when interrupt is triggered */
void EXTI0_IRQHandler(void)
{
	/* debouncing protection */
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);

	/* enable the SPI2 peripheral after configurations */
	SPI_PeripheralControl(SPI2, ENABLE);

	/* get the length of the data */
	uint8_t dataLen = strlen(message);

	/* send data length over SPI2 */
	SPI_SendData(SPI2, &dataLen, 1);

	/* send data over SPI2 */
	SPI_SendData(SPI2, (uint8_t*)message, strlen(message));

	/* wait till data has sent completely */
	while(SPI_GetFlagStatus(SPI2, SPI_SR_BSY));

	/* disable the SPI2 peripheral after configurations */
	SPI_PeripheralControl(SPI2, DISABLE);
}

