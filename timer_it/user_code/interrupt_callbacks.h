#ifndef INTERRUPT_CALLBACKS_H_
#define INTERRUPT_CALLBACKS_H_

#include <stdint.h>

extern volatile uint8_t button_rising;
extern volatile uint8_t button_falling;
extern volatile uint8_t timer_rollover;

#endif /* INTERRUPT_CALLBACKS_H_ */
