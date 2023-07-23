/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 23 Tem 2023
 *  Author: EMRE PEKGÜZEL
 */

#include "stm32f407xx.h"

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

/* configuration structure for an SPI peripheral */
typedef struct
{
	uint8_t SPI_DeviceMode;			/* possible device modes from @SPIDM	*/
	uint8_t SPI_BusConfig;			/* possible bus configs from  @SPIBC	*/
	uint8_t SPI_SclkSpeed;			/* possible sclk speeds from  @SPISS	*/
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_PinConfig_t;

/* handle structure for an SPI peripheral */
typedef struct
{
	SPI_RegDef_t   *pSPIx;
	SPI_PinConfig_t SPI_PinConfig;
}SPI_Handle_t;

/* @SPIDM possible device modes */
/* @SPIBC possible bus configs */
/* @SPISS possible sclk speeds */

/*********************************APIs SUPPORTED BY THIS DRIVER*********************************/

/* peripheral clock setup */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/* init and de-init */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/* data send and receive */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/* IRQ configuration and ISR handling */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/* other peripheral control APIs */

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
