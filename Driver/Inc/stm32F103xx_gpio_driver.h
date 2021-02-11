

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_


#include "stm32f103xx.h"


typedef struct{
	uint8_t GPIO_PinNumber;	/*!< Specifies the GPIO pins to be configured.
                           This parameter can be any value of @ref GPIO_pins_define */

	uint8_t GPIO_PinMode;	/*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode_define */

	uint8_t GPIO_PinSpeed;	/* Pin Speed mode */



}GPIO_PinConfig_t;


typedef struct{
	EXTI_RegDef_t		*pEXTI;
	GPIO_RegDef_t		*pGPIOx;
	GPIO_PinConfig_t	 GPIO_PinConfig;
	AFIO_RegDef_t 		*pAFIFO;


}GPIO_Handle_t;


//*************** API FUNCTION DEFINITIONS*******************

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnoDi);

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t ENoDi);
void GPIO_IRQHandling(uint8_t PinNumber);




/** @defgroup GPIO_pins_define GPIO pins define
  * @{
  */
#define GPIO_PIN_0			0
#define GPIO_PIN_1          1
#define GPIO_PIN_2          2
#define GPIO_PIN_3          3
#define GPIO_PIN_4          4
#define GPIO_PIN_5          5
#define GPIO_PIN_6          6
#define GPIO_PIN_7          7
#define GPIO_PIN_8          8
#define GPIO_PIN_9          9
#define GPIO_PIN_10        10
#define GPIO_PIN_11        11
#define GPIO_PIN_12        12
#define GPIO_PIN_13        13
#define GPIO_PIN_14        14
#define GPIO_PIN_15	       15

/** @defgroup GPIO_mode_define GPIO mode define
  */
//INPUT
#define  GPIO_MODE_INPUT_ANALOG					1				/*!< Input Analog Mode               		  */
#define  GPIO_MODE_INPUT_FLOATING   			2				/*!< Input Floating Mode -RESET STATE         */
#define  GPIO_MODE_INPUT_PU                     3				 /*!< Input with pull-up 			          */
#define  GPIO_MODE_INPUT_PD						4				/*!< Input with pull-down						*/
//OUTPUT
#define	 GPIO_MODE_OUTPUT_PP					5				/* General purpose output push-pull			  */
#define	 GPIO_MODE_OUTPUT_OD					6				/* General purpose output Open-drain - RESET STATE */
#define  GPIO_MODE_AF_OUTPUT_PP                 7  				 /*!< Alternate Function Push Pull Mode       */
#define  GPIO_MODE_AF_OUTPUT_OD                 8   			/*!< Alternate Function Open Drain Mode       */

// AFIFO INNTERRUPT
#define GPIO_MODE_IT_FT			9
#define GPIO_MODE_IT_RT			10
#define GPIO_MODE_IT_RFT		11


/** @ref GPIO_speed_define
 */
#define GPIO_MAX_SPEED_10Mhz		1
#define GPIO_MAX_SPEED_2Mhz			2
#define GPIO_MAX_SPEED_50Mhz		3






#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
