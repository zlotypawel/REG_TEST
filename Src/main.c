#include <stdint.h>
#include "stm32f103xx.h"
#include "stm32F103xx_gpio_driver.h"


void delay(void);

int main(void) {
	GPIO_Handle_t GpioLed, GpioSW;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOB, ENABLE);
	AFIO_PCLK_EN();


	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_MAX_SPEED_2Mhz;

	GpioSW.pGPIOx = GPIOB;
	GpioSW.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	GpioSW.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;


	GPIO_IRQConfig(IRQ_NO_EXTI1, ENABLE);

	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioSW);





	while (1) {

		/*if(!(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_7)  == GPIO_PIN_SET))
		{
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_0);
			delay();
		}*/

	}
}

void EXTI1_IRQHandler(void){
	EXTI -> PR |= (1 << 1);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_0);
}
