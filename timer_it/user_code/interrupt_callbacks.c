#include "main.h"
#include "interrupt_callbacks.h"

extern TIM_HandleTypeDef htim2;
volatile uint8_t button_rising = 0U;
volatile uint8_t button_falling = 0U;
volatile uint8_t timer_rollover = 0U;

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2) {
		timer_rollover = 1U;
	}
}
