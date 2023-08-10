/*
 *  004SpiTxTest.c
 *
 *  Created on: Jul 24, 2023
 *  Author: EMRE PEKGUZEL
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
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CFG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_PCLK_DIV_256;
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_SSI = SPI_SSI_EN;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2Handle);
}

int main(void)
{
	char message[] = "Hello World!";

	/* this function will be used to initialize GPIO pins to behave as SPI2 pins */
	SPI2_GPIOInits();

	/* this function will be used to initialize SPI2 peripheral parameters */
	SPI2_Inits();

	/* enable the SPI2 peripheral after configurations */
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*)message, strlen(message));

	while(1)
	{
		SPI_SendData(SPI2, (uint8_t*)message, strlen(message));

	}

	return 0;
}

