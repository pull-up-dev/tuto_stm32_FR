#include "main.h"
#include "interrupt_callbacks.h"

volatile uint8_t button_rising = 0U;
volatile uint8_t button_falling = 0U;

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin) {
		case BUTTON_Pin:
			button_rising = 1U;
			break;
	}
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin) {
		case BUTTON_Pin:
			button_falling = 1U;
			break;
	}
}
