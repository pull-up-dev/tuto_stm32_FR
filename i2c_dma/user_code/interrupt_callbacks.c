#include "main.h"
#include "interrupt_callbacks.h"

extern I2C_HandleTypeDef hi2c1;

volatile uint8_t button_rising = 0U;
volatile uint8_t button_falling = 0U;
volatile uint8_t i2c_tx_cplt = 0U;
volatile uint8_t i2c_rx_cplt = 0U;

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

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c == &hi2c1) {
		i2c_tx_cplt = 1U;
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c == &hi2c1) {
		i2c_rx_cplt = 1U;
	}
}
