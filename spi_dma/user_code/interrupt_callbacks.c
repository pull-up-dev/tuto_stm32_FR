#include "main.h"
#include "interrupt_callbacks.h"

extern SPI_HandleTypeDef hspi1;
volatile uint8_t button_rising = 0U;
volatile uint8_t button_falling = 0U;
volatile uint8_t spi_tx_cplt = 0U;
volatile uint8_t spi_txrx_cplt = 0U;

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

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi1) {
		spi_tx_cplt = 1U;
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi1) {
		spi_txrx_cplt = 1U;
	}
}
