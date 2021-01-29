

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include <stdio.h>

									// base addresses of FLASH and SRAM memories
#define FLASH_BASE_ADDR		0x08000000U
#define SRAM_BASE_ADDR 		0x20000000U


									// AHBx and APBx BUS Peripheral base addresses
#define PERIPH_BASE					0x40000000U
#define APB1_PERIPH_BASE_ADDR		PERIPH_BASE
#define APB2_PERIPH_BASE_ADDR		0x40010000U
#define AHB_PERIPH_BASE_ADDR		0x40018000U

									// base addresses of peripheral which are hanging on APB1 bus
#define I2C1_BASE_ADDR		(APB1_PERIPH_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR		(APB1_PERIPH_BASE_ADDR + 0x5800)

#define USART2_BASE_ADDR	(APB1_PERIPH_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR	(APB1_PERIPH_BASE_ADDR + 0x4800)

#define SPI2_BASE_ADDR		(APB1_PERIPH_BASE_ADDR + 0x3800)


									// base addresses of peripheral which are hanging on APB2 bus
#define GPIOA_BASE_ADDR		(APB2_PERIPH_BASE_ADDR + 0x0800)
#define GPIOB_BASE_ADDR		(APB2_PERIPH_BASE_ADDR + 0x0C00)
#define GPIOC_BASE_ADDR		(APB2_PERIPH_BASE_ADDR + 0x1000)
#define GPIOD_BASE_ADDR		(APB2_PERIPH_BASE_ADDR + 0x1400)

#define EXTI_BASE_ADDR		(APB2_PERIPH_BASE_ADDR + 0x0400)

#define USART1_BASE_ADDR	(APB2_PERIPH_BASE_ADDR + 0x3800)

#define SPI1_BASE_ADDR		(APB2_PERIPH_BASE_ADDR + 0x3000)

#define AFIO_BASE_ADDR		APB2_PERIPH_BASE_ADDR




									// base addresses of peripheral which are hanging on AHB bus
#define RCC_BASE_ADDR		(0x40021000)																	// check <<<<<<<<<<<<<<<<<<<<<<<<<





//*************************************************** peripheral register definition structures***************************************************

typedef struct{
	volatile uint32_t CRL;			 /*!< Port configuration register low, (GPIOx_CRL) 0-7 */
	volatile uint32_t CRH;			 /* Port configuration register high (GPIOx_CRH) 8-15 */
	volatile uint32_t IDR;			 /* Port input data register (GPIOx_IDR) */
	volatile uint32_t ODR;			 /* Port output data register (GPIOx_ODR) */
	volatile uint32_t BSRR;			 /* Port bit set/reset register (GPIOx_BSRR) */
	volatile uint32_t BRR;			 /* Port bit reset register (GPIOx_BRR) */
	volatile uint32_t LCKR;			 /* Port configuration lock register (GPIOx_LCKR) */
}GPIO_RegDef_t;

typedef struct{
	volatile uint32_t CR;			/* Clock control register (RCC_CR) */
	volatile uint32_t CFGR;			/* Clock configuration register (RCC_CFGR) */
	volatile uint32_t CIR;			/* Clock interrupt register (RCC_CIR) */
 	volatile uint32_t APB2RSTR;		/* APB2 peripheral reset register (RCC_APB2RSTR) */
	volatile uint32_t APB1RSTR;		/* APB1 peripheral reset register (RCC_APB1RSTR) */
	volatile uint32_t AHBENR;		/* AHB peripheral clock enable register (RCC_AHBENR) */
	volatile uint32_t APB2ENR;		/* APB2 peripheral clock enable register (RCC_APB2ENR) */
	volatile uint32_t APB1ENR;		/* APB1 peripheral clock enable register (RCC_APB1ENR) */
	volatile uint32_t BDCR;			/* Backup domain control register (RCC_BDCR) */
	volatile uint32_t CSR;			/* Control/status register (RCC_CSR) */
}RCC_RegDef_t;

typedef struct{
	volatile uint32_t IMR;			 /*!< Interrupt mask register */
	volatile uint32_t EMR;			 /* Event mask register */
	volatile uint32_t RTSR;			 /* Rising trigger selection register */
	volatile uint32_t FTSR;			 /* Falling trigger selection register */
	volatile uint32_t SWIER;		 /* Software interrupt event register */
	volatile uint32_t PR;			 /* Pending register */
}EXTI_RegDef_t;

typedef struct{
	volatile uint32_t EVCR;			 /*!< Event control register */
	volatile uint32_t MAPR;			 /*!< AF remap and debug I/O configuration register */
	volatile uint32_t EXTICR1;		 /*!< External interrupt configuration register 1 */
	volatile uint32_t EXTICR2;		 /*!< External interrupt configuration register 2 */
	volatile uint32_t EXTICR3;		 /*!< External interrupt configuration register 3 */
	volatile uint32_t EXTICR4;		 /*!< External interrupt configuration register 4 */
	volatile uint32_t MAPR2;		 /*!< AF remap and debug I/O configuration register2 */
}AFIO_RegDef_t;

//************************************************ peripheral definitions *******************************************************

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASE_ADDR)

#define RCC ((RCC_RegDef_t*)RCC_BASE_ADDR)

#define EXTI ((EXTI_RegDef_t*)EXTI_BASE_ADDR)

#define AFIO ((AFIO_RegDef_t*)AFIO_BASE_ADDR)
//************************************************** CLOCK  *************************************************

//ENable Clock GPIO

#define GPIOA_PCLK_EN() (RCC -> APB2ENR |= (1<<2))
#define GPIOB_PCLK_EN() (RCC -> APB2ENR |= (1<<3))
#define GPIOC_PCLK_EN() (RCC -> APB2ENR |= (1<<4))
#define GPIOD_PCLK_EN() (RCC -> APB2ENR |= (1<<5))
#define AFIO_PCLK_EN()  (RCC -> APB2ENR |= (1<<0))


//Disable Clock GPIO

#define GPIOA_PCLK_DI() (RCC -> APB2ENR &= ~(1<<2))
#define GPIOB_PCLK_DI() (RCC -> APB2ENR &= ~(1<<3))
#define GPIOC_PCLK_DI() (RCC -> APB2ENR &= ~(1<<4))
#define GPIOD_PCLK_DI() (RCC -> APB2ENR &= ~(1<<5))
#define AFIO_PCLK_DI()  (RCC -> APB2ENR &= ~(1<<0))

//******************************************* RESET GPIO REGISTERS******************************************

#define GPIOA_REG_RESET()	do{(RCC -> APB2RSTR |= (1<<2)); (RCC -> APB2RSTR &=~ (1<<2));}while(0)
#define GPIOB_REG_RESET()	do{(RCC -> APB2RSTR |= (1<<3)); (RCC -> APB2RSTR &=~ (1<<3));}while(0)
#define GPIOC_REG_RESET()	do{(RCC -> APB2RSTR |= (1<<4)); (RCC -> APB2RSTR &=~ (1<<4));}while(0)
#define GPIOD_REG_RESET()	do{(RCC -> APB2RSTR |= (1<<5)); (RCC -> APB2RSTR &=~ (1<<5));}while(0)

//****************************************** other macro ****************************************************
#define ENABLE 				 1
#define DISABLE				 0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#endif /* INC_STM32F103XX_H_ */


