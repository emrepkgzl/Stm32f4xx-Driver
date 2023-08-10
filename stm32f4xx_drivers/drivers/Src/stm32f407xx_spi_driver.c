/*
 *  stm32f407xx_spi_driver.c
 *
 *  Created on: Jul 23, 2023
 *  Author: EMRE PEKGÜZEL
 */

#include "stm32f407xx_spi_driver.h"


/* peripheral clock setup */
/**********************************************************************
 *	@func	:	SPI_PeriClockControl
 *	@brief	:	Enables or disables peripheral clock for given SPI
 *				peripheral
 *
 * 	@param	:	Base address of the SPI peripheral
 * 	@param 	:	Enable or disable macros
 * 	@param 	:	none
 *	@return :	none
 *
 * 	@note	:	none
 * 	@date	:	07/23/23
 **********************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

/* init and de-init */
/**********************************************************************
 *	@func	:	SPI_Init
 *	@brief	:	Initializes given SPI peripheral
 *
 * 	@param	:	Base address of the SPI peripheral
 * 	@param 	:	none
 * 	@param 	:	none
 *	@return :	none
 *
 * 	@note	:	none
 * 	@date	:	07/24/23
 **********************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t temp = 0;

	/* Enable peripheral clock */
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	/* 1. configure the SPI_CR1 register */


	temp |= (pSPIHandle->SPI_Config.SPI_BusConfig << SPI_CR1_MSTR);

	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CFG_FD)
	{
		/* bidi mode should be cleared */
		temp &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CFG_HD)
	{
		/* bidi mode should be set */
		temp |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CFG_SIMPLEX_RXONLY)
	{
		/* bidi mode should be cleared */
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		/* rxonly bit should be set */
		temp |= (1 << SPI_CR1_RXONLY);
	}

	temp |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR);

	temp |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	temp |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	temp |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	temp |= (pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

	temp |= (pSPIHandle->SPI_Config.SPI_SSI << SPI_CR1_SSI);

	pSPIHandle->pSPIx->CR1 = temp;
}

/**********************************************************************
 *	@func	:	SPI_DeInit
 *	@brief	:	Deinitializes given SPI peripheral
 *
 *	@param	:	Base address of the SPI peripheral
 *	@param 	:	none
 *	@param 	:	none
 *	@return :	none
 *
 *	@note	:	none
 *	@date	:	07/24/23
 **********************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

/* data send and receive */
/**********************************************************************
 *	@func	:	SPI_SendData
 *	@brief	:	Sends desired data over SPI
 *
 * 	@param	:	Base address of the SPI peripheral
 * 	@param 	:	Address of the data to send
 * 	@param 	:	Length of the data to send
 *	@return :	none
 *
 * 	@note	:	This is a blocking call
 * 	@date	:	07/24/23
 **********************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		/* wait(block) untill tx buffer become empty */
		while(!(pSPIx->SR & (1 << SPI_SR_TXE)));

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			/* 16 bit frame format */
			pSPIx->DR = *(uint16_t*)pTxBuffer;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			/* 8 bit frame format */
			pSPIx->DR = *pTxBuffer;
			pTxBuffer++;
		}
		Len--;
	}
}

/**********************************************************************
 *	@func	:	SPI_ReceiveData
 *	@brief	:	Receives data over SPI
 *
 * 	@param	:	Base address of the SPI peripheral
 * 	@param 	:	Address of the variable to write over
 * 	@param 	:	Length of the data to receive
 *	@return :	none
 *
 * 	@note	:	none
 * 	@date	:	07/24/23
 **********************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}

/* IRQ configuration and ISR handling */
/**********************************************************************
 *	@func	:	SPI_IRQConfig
 *	@brief	:	Enables or disables given IRQ
 *
 * 	@param	:	IRQ number for desired interrupt
 * 	@param 	:	Enable or disable macros
 * 	@param 	:	none
 *	@return :	none
 *
 * 	@note	:	none
 * 	@date	:	07/24/23
 **********************************************************************/
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

/**********************************************************************
 *	@func	:	SPI_IRQPriorityConfig
 *	@brief	:	Configurates the priority desired IRQ
 *
 * 	@param	:	IRQ number for desired interrupt
 * 	@param 	:	Priority value for given interrupt
 * 	@param 	:	none
 *	@return :	none
 *
 * 	@note	:	none
 * 	@date	:	07/24/23
 **********************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

/**********************************************************************
 *	@func	:	SPI_IRQHandling
 *	@brief	:	Handler function for IRQ
 *
 * 	@param	:	SPI peripheral that triggers the interrupt
 * 	@param 	:	none
 * 	@param 	:	none
 *	@return :	none
 *
 * 	@note	:	none
 * 	@date	:	07/24/23
 **********************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}

/**********************************************************************
 *	@func	:	SPI_PeripheralControl
 *	@brief	:	Controller function to enable or disable spi peripheral
 *
 * 	@param	:	Base address of the SPI peripheral
 * 	@param 	:	Enable or disable macros
 * 	@param 	:	none
 *	@return :	none
 *
 * 	@note	:	none
 * 	@date	:	08/10/23
 **********************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


