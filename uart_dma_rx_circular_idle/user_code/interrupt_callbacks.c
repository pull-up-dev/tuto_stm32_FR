#include "main.h"
#include "interrupt_callbacks.h"

extern UART_HandleTypeDef huart1;

volatile uint8_t button_rising = 0U;
volatile uint8_t button_falling = 0U;
volatile uint8_t uart_rx_event = 0U;
volatile uint16_t uart_rx_buf_ptr = 0U;
volatile uint8_t uart_tx_event = 0U;

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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Pos)
{
	if (huart == &huart1) {
		uart_rx_buf_ptr = Pos;
		uart_rx_event = 1U;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1) {
		uart_tx_event = 1U;
	}
}
