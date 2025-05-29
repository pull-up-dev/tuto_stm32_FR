#ifndef INTERRUPT_CALLBACKS_H_
#define INTERRUPT_CALLBACKS_H_

#include <stdint.h>

extern volatile uint8_t button_rising;
extern volatile uint8_t button_falling;
extern volatile uint8_t uart_rx_event;
extern volatile uint8_t uart_tx_event;

#endif /* INTERRUPT_CALLBACKS_H_ */
