/*
 *  stm32f407xx_spi_driver.h
 *
 *  Created on: Jul 23, 2023
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
	uint8_t SPI_DFF;				/* possible dff options from  @SPIDFF	*/
	uint8_t SPI_CPOL;				/* possible cpol options from @SPICPOL	*/
	uint8_t SPI_CPHA;				/* possible cpha options from @SPICPHA	*/
	uint8_t SPI_SSM;				/* possible ssm options from  @SPISSM	*/
	uint8_t SPI_SSI;				/* possible ssm options from  @SPISSI	*/
	uint8_t SPI_SSOE;				/* possible ssm options from  @SPISSOE	*/

}SPI_Config_t;

/* handle structure for an SPI peripheral */
typedef struct
{
	SPI_RegDef_t   *pSPIx;
	SPI_Config_t SPI_Config;
}SPI_Handle_t;

/* @SPIDM possible device modes */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/* @SPIBC possible bus configs */
#define SPI_BUS_CFG_FD				1
#define SPI_BUS_CFG_HD				2
#define SPI_BUS_CFG_SIMPLEX_RXONLY	3

/* @SPISS possible sclk speeds */
#define SPI_SCLK_SPEED_PCLK_DIV_2 	0
#define SPI_SCLK_SPEED_PCLK_DIV_4 	1
#define SPI_SCLK_SPEED_PCLK_DIV_8 	2
#define SPI_SCLK_SPEED_PCLK_DIV_16 	3
#define SPI_SCLK_SPEED_PCLK_DIV_32	4
#define SPI_SCLK_SPEED_PCLK_DIV_64	5
#define SPI_SCLK_SPEED_PCLK_DIV_128	6
#define SPI_SCLK_SPEED_PCLK_DIV_256 7

/* @SPIDFF possible DFF options */
#define SPI_DFF_8BITS				0
#define SPI_DFF_16BITS				1

/* @SPICPOL possible CPOL options */
#define SPI_CPOL_LOW				0
#define SPI_CPOL_HIGH				1

/* @SPICPHA possible CPHA options */
#define SPI_CPHA_LOW				0
#define SPI_CPHA_HIGH				1

/* @SPISSM possible SSM options */
#define SPI_SSM_DI  				0
#define SPI_SSM_EN					1

/* @SPISSI possible SSM options */
#define SPI_SSI_DI  				0
#define SPI_SSI_EN					1

/* @SPISSI possible SSOE options */
#define SPI_SSOE_DI  				0
#define SPI_SSOE_EN					1
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
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
