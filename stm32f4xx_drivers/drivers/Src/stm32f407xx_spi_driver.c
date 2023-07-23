/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 23 Tem 2023
 *  Author: EMRE PEKGÜZEL
 */

#include "stm32f407xx_spi_driver.h"


/* peripheral clock setup */
/**********************************************************************
 *
 *		@func	:	SPI_PeriClockControl
 *		@brief	:	Enables or disables peripheral clock for given SPI
 *					peripheral
 *
 * 		@param	:	Base address of the SPI peripheral
 * 		@param 	:	Enable or disable macros
 * 		@param 	:	none
 *		@return :	none
 *
 * 		@note	:	none
 * 		@date	:	07/23/23
 *
 **********************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

}

/* init and de-init */
/**********************************************************************
 *
 *		@func	:	SPI_Init
 *		@brief	:	Initializes given SPI peripheral
 *
 * 		@param	:	Base address of the SPI peripheral
 * 		@param 	:	none
 * 		@param 	:	none
 *		@return :	none
 *
 * 		@note	:	none
 * 		@date	:	07/23/23
 *
 **********************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

}

/**********************************************************************
 *
 *		@func	:	SPI_DeInit
 *		@brief	:	Deinitializes given SPI peripheral
 *
 * 		@param	:	Base address of the SPI peripheral
 * 		@param 	:	none
 * 		@param 	:	none
 *		@return :	none
 *
 * 		@note	:	none
 * 		@date	:	07/23/23
 *
 **********************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}

/* data send and receive */
/**********************************************************************
 *
 *		@func	:	SPI_SendData
 *		@brief	:	Sends desired data over SPI
 *
 * 		@param	:	Base address of the SPI peripheral
 * 		@param 	:	Address of the data to send
 * 		@param 	:	Length of the data to send
 *		@return :	none
 *
 * 		@note	:	none
 * 		@date	:	07/23/23
 *
 **********************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{

}

/**********************************************************************
 *
 *		@func	:	SPI_ReceiveData
 *		@brief	:	Receives data over SPI
 *
 * 		@param	:	Base address of the SPI peripheral
 * 		@param 	:	Address of the variable to write over
 * 		@param 	:	Length of the data to receive
 *		@return :	none
 *
 * 		@note	:	none
 * 		@date	:	07/23/23
 *
 **********************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}

/* IRQ configuration and ISR handling */
/**********************************************************************
 *
 *		@func	:	SPI_IRQConfig
 *		@brief	:	Enables or disables given IRQ
 *
 * 		@param	:	IRQ number for desired interrupt
 * 		@param 	:	Enable or disable macros
 * 		@param 	:	none
 *		@return :	none
 *
 * 		@note	:	none
 * 		@date	:	07/23/23
 *
 **********************************************************************/
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

/**********************************************************************
 *
 *		@func	:	SPI_IRQPriorityConfig
 *		@brief	:	Configurates the priority desired IRQ
 *
 * 		@param	:	IRQ number for desired interrupt
 * 		@param 	:	Priority value for given interrupt
 * 		@param 	:	none
 *		@return :	none
 *
 * 		@note	:	none
 * 		@date	:	07/23/23
 *
 **********************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

/**********************************************************************
 *
 *		@func	:	SPI_IRQHandling
 *		@brief	:	Handler function for IRQ
 *
 * 		@param	:	SPI peripheral that triggers the interrupt
 * 		@param 	:	none
 * 		@param 	:	none
 *		@return :	none
 *
 * 		@note	:	none
 * 		@date	:	07/23/23
 *
 **********************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}

