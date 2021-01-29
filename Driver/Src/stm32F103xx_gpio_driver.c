#include "stm32F103xx_gpio_driver.h"
#include <stdio.h>



/*
 * @fn 				- GPIO_PeriClockControl
 *
 * @brief			- This function enable or disable peripheral clock for given GPIO port
 *
 * @param[in]		- base adress of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return 			- none
 *
 * @Note 			- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnoDi){

	if(EnoDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){


	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < GPIO_PIN_8){
	//INPUT
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INPUT_ANALOG){

			pGPIOHandle ->pGPIOx->CRL &= ~ (4 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INPUT_FLOATING){
				// RESET STATE

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INPUT_PU){
			pGPIOHandle ->pGPIOx->CRL &= ~ (4<< (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle ->pGPIOx->CRL |=   (8 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle ->pGPIOx->ODR |=   (1 << (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INPUT_PD){
			pGPIOHandle ->pGPIOx->CRL &= ~ (4<< (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle ->pGPIOx->CRL |=   (8 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));



	//OUTPUT
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUTPUT_PP){
			pGPIOHandle ->pGPIOx->CRL &= ~ (4 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

			switch (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed) {
				case GPIO_MAX_SPEED_10Mhz:
					pGPIOHandle ->pGPIOx->CRL |= (GPIO_MAX_SPEED_10Mhz << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
					break;
				case GPIO_MAX_SPEED_2Mhz:
					pGPIOHandle ->pGPIOx->CRL |= (GPIO_MAX_SPEED_2Mhz << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
					break;
				case GPIO_MAX_SPEED_50Mhz:
					pGPIOHandle ->pGPIOx->CRL |= (GPIO_MAX_SPEED_50Mhz << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
					break;
			}

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUTPUT_OD) {
			// RESET STATE
			switch (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed) {
				case GPIO_MAX_SPEED_10Mhz:
					pGPIOHandle->pGPIOx->CRL |= (GPIO_MAX_SPEED_10Mhz << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
					break;
				case GPIO_MAX_SPEED_2Mhz:
					pGPIOHandle->pGPIOx->CRL |= (GPIO_MAX_SPEED_2Mhz << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
					break;
				case GPIO_MAX_SPEED_50Mhz:
					pGPIOHandle->pGPIOx->CRL |= (GPIO_MAX_SPEED_50Mhz << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
					break;
			}
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF_OUTPUT_PP){
			pGPIOHandle ->pGPIOx->CRL &= ~ (4<< (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle ->pGPIOx->CRL |=   (GPIO_MODE_AF_OUTPUT_PP << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

			switch (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed) {
				case GPIO_MAX_SPEED_10Mhz:
					pGPIOHandle->pGPIOx->CRL |= (GPIO_MAX_SPEED_10Mhz << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
					break;
				case GPIO_MAX_SPEED_2Mhz:
					pGPIOHandle->pGPIOx->CRL |= (GPIO_MAX_SPEED_2Mhz << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
					break;
				case GPIO_MAX_SPEED_50Mhz:
					pGPIOHandle->pGPIOx->CRL |= (GPIO_MAX_SPEED_50Mhz << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
					break;
			}

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF_OUTPUT_OD){
			pGPIOHandle ->pGPIOx->CRL |=   (8 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			// RESET STATE
			switch (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed) {
				case GPIO_MAX_SPEED_10Mhz:
					pGPIOHandle->pGPIOx->CRL |= (GPIO_MAX_SPEED_10Mhz<< (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
					break;
				case GPIO_MAX_SPEED_2Mhz:
					pGPIOHandle->pGPIOx->CRL |= (GPIO_MAX_SPEED_2Mhz<< (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
					break;
				case GPIO_MAX_SPEED_50Mhz:
					pGPIOHandle->pGPIOx->CRL |= (GPIO_MAX_SPEED_50Mhz<< (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
					break;
			}
		}


	}else{

	}

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1);
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value =(uint16_t) pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == GPIO_PIN_SET){
		pGPIOx ->ODR |= (1<< PinNumber);
	}else{
		pGPIOx ->ODR &= ~ (1<< PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx ->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx ->ODR ^= (1<<PinNumber);
}

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENoDi);
void GPIO_IRQHandling(uint8_t PinNumber);

