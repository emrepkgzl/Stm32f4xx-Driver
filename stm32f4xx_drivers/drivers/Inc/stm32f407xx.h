/*
 * stm32f407xx.h
 *
 *  Created on: Jul 9, 2023
 *  Author: EMRE PEKGUZEL
 */

#include <stdint.h>

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#define __vo volatile

/*
 * These adresses below can be found on memory
 * map which is a port of reference manual
 */

/******************************PERIPHERAL BASE ADDRESS DEFINITIONS******************************/

/* base adresses of Flash and SRAM memories */
#define FLASHB_BASEADDR			0x08000000U		/* U stands for unsigned 	 */
#define SRAM1_BASEADDR			0x20000000U		/* SIZE-> 112KB 		 	 */
#define SRAM2_BASEADDR			0x20001C00U		/* SRAM1 BADDR + 112KB   	 */
#define ROM_BASEADDR			0x1FFF0000U		/* SYSTEM MEMORY ADDR    	 */
#define SRAM 					SRAM1_BASEADDR	/* ASSUME SRAM1 AS MAIN SRAM */

/* base adresses of Peripheral Buses */
#define PRPH_BASEADDR			0x40000000U
#define APB1_BASEADDR			PRPH_BASEADDR
#define APB2_BASEADDR			0x40010000U
#define AHB1_BASEADDR			0x40020000U
#define AHB2_BASEADDR			0x50000000U

/* base adresses of peripherals which are hanging on AHB1 bus */
#define GPIOA_BASEADDR 			(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR 			(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 			(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 			(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 			(AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 			(AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 			(AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 			(AHB1_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 			(AHB1_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR 			(AHB1_BASEADDR + 0x2400)
#define GPIOK_BASEADDR 			(AHB1_BASEADDR + 0x2800)

#define RCC_BASEADDR			(AHB1_BASEADDR + 0x3800)

/* base adresses of peripherals which are hanging on APB1 bus */
#define I2C1_BASEADDR			(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1_BASEADDR + 0x5000)

/* base adresses of peripherals which are hanging on APB2 bus */
#define SPI1_BASEADDR			(APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2_BASEADDR + 0x3400)

#define USART1_BASEADDR			(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2_BASEADDR + 0x1400)

#define EXTI_BASEADDR			(APB2_BASEADDR + 0x3C00)

#define SYSCFG_BASEADDR			(APB2_BASEADDR + 0x3800)


/***************************PERIPHERAL REGISTER DEFINITION STRUCTURES***************************/

typedef struct
{
	__vo uint32_t MODER;   		/* GPIO port mode register						Address offset: 0x00 */
	__vo uint32_t OTYPER;  		/* GPIO port output type register				Address offset: 0x04 */
	__vo uint32_t OSPEEDR; 		/* GPIO port output speed register				Address offset: 0x08 */
	__vo uint32_t PUPDR;   		/* GPIO port pull-up/pull-down register			Address offset: 0x0C */
	__vo uint32_t IDR;     		/* GPIO port input data register				Address offset: 0x10 */
	__vo uint32_t ODR;     		/* GPIO port output data register				Address offset: 0x14 */
	__vo uint32_t BSRR;    		/* GPIO port bit set/reset register				Address offset: 0x18 */
	__vo uint32_t LCKR;    		/* GPIO port configuration lock register		Address offset: 0x1C */
	__vo uint32_t AFRL;    		/* GPIO alternate function low register			Address offset: 0x20 */
	__vo uint32_t AFRH;    		/* GPIO alternate function high register		Address offset: 0x24 */
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;   		/* RCC clock control register					Address offset: 0x00 */
	__vo uint32_t PLLCFGR;  	/* RCC PLL configuration register				Address offset: 0x04 */
	__vo uint32_t CFGR;	 		/* RCC clock configuration register				Address offset: 0x08 */
	__vo uint32_t CIR;   		/* RCC clock interrupt register					Address offset: 0x0C */
	__vo uint32_t AHB1RSTR; 	/* RCC AHB1 peripheral reset register			Address offset: 0x10 */
	__vo uint32_t AHB2RSTR; 	/* RCC AHB2 peripheral reset register			Address offset: 0x14 */
	__vo uint32_t AHB3RSTR; 	/* RCC AHB3 peripheral reset register			Address offset: 0x18 */
		 uint32_t RESERVED0;	/* RESERVED										Address offset: 0x1C */
	__vo uint32_t APB1RSTR; 	/* RCC APB1 peripheral reset register			Address offset: 0x20 */
	__vo uint32_t APB2RSTR; 	/* RCC APB2 peripheral reset register			Address offset: 0x24 */
		 uint32_t RESERVED1;	/* RESERVED										Address offset: 0x28 */
		 uint32_t RESERVED2;	/* RESERVED										Address offset: 0x2C */
	__vo uint32_t AHB1ENR;		/* RCC AHB1 peripheral clock enable register	Address offset: 0x30 */
	__vo uint32_t AHB2ENR;		/* RCC AHB2 peripheral clock enable register	Address offset: 0x34 */
	__vo uint32_t AHB3ENR;		/* RCC AHB3 peripheral clock enable register	Address offset: 0x38 */
		 uint32_t RESERVED3;	/* RESERVED										Address offset: 0x3C */
	__vo uint32_t APB1ENR;		/* RCC APB1 peripheral clock enable register	Address offset: 0x40 */
	__vo uint32_t APB2ENR;		/* RCC APB2 peripheral clock enable register	Address offset: 0x44 */
		 uint32_t RESERVED4;	/* RESERVED										Address offset: 0x48 */
		 uint32_t RESERVED5;	/* RESERVED										Address offset: 0x4C */
	__vo uint32_t AHB1LPENR;	/* RCC AHB1 p clock enable in low power reg		Address offset: 0x50 */
	__vo uint32_t AHB2LPENR;	/* RCC AHB2 p clock enable in low power reg		Address offset: 0x54 */
	__vo uint32_t AHB3LPENR;	/* RCC AHB3 p clock enable in low power reg		Address offset: 0x58 */
		 uint32_t RESERVED6;	/* RESERVED										Address offset: 0x5C */
	__vo uint32_t APB1LPENR;	/* RCC APB1 p clock enable in low power reg		Address offset: 0x60 */
	__vo uint32_t APB2LPENR;	/* RCC APB2 p clock enable in low power reg		Address offset: 0x64 */
		 uint32_t RESERVED7;	/* RESERVED										Address offset: 0x68 */
		 uint32_t RESERVED8;	/* RESERVED										Address offset: 0x6C */
	__vo uint32_t BDCR;			/* RCC Backup domain control register			Address offset: 0x70 */
	__vo uint32_t CSR;			/* RCC clock control & status register			Address offset: 0x74 */
		 uint32_t RESERVED9;	/* RESERVED										Address offset: 0x78 */
		 uint32_t RESERVED10;	/* RESERVED										Address offset: 0x7C */
	__vo uint32_t SSCGR;		/* RCC spread spectrum clk generation reg		Address offset: 0x80 */
	__vo uint32_t PLLI2SCFGR;	/* RCC PLLI2S configuration register			Address offset: 0x84 */
;
}RCC_RegDef_t;


/* peripheral definitions (peripheral base adresses typecasted to xxx_RegDef_t) */
#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF	((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG	((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH	((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI	((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ	((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK	((GPIO_RegDef_t*)GPIOK_BASEADDR)

#define RCC 	((RCC_RegDef_t*)RCC_BASEADDR)

/************************************PERIPHERAL CLOCK MACROS************************************/


/* clock enable macros for GPIOx peripherals */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))

/* clock enable macros for I2Cx peripherals */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/* clock enable macros for U(S)ARTx peripherals */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

/* clock enable macros for SPIx peripherals */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))

/* clock enable macro for SYSCFG peripheral */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

/* clock disable macros for GPIOx peripherals */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))

/* clock disable macros for I2Cx peripherals */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/* clock disable macros for U(S)ARTx peripherals */
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

/* clock disable macros for SPIx peripherals */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))

/* clock disable macro for SYSCFG peripheral */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))

/* GPIOx peripheral reset macros */
#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0);
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0);
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0);
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0);
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0);
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(0);
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0);
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0);
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));} while(0);

/* some generic macros */
#define ENABLE	1
#define DISABLE 0
#define SET		ENABLE
#define RESET	DISABLE
#define TRUE 	ENABLE
#define FALSE 	!= TRUE

#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
