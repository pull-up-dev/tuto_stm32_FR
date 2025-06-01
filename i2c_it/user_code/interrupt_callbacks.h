#ifndef INTERRUPT_CALLBACKS_H_
#define INTERRUPT_CALLBACKS_H_

#include <stdint.h>

extern volatile uint8_t button_rising;
extern volatile uint8_t button_falling;
extern volatile uint8_t i2c_tx_cplt;
extern volatile uint8_t i2c_rx_cplt;

#endif /* INTERRUPT_CALLBACKS_H_ */
