#ifndef __LED_ALERT_H_
#define __LED_ALERT_H_
#include <board.h>

#define LED0_PIN	GET_PIN(F, 9)
#define LED1_PIN	GET_PIN(F, 10)

void led_init(uint32_t pin);
void led_on(uint32_t pin);
void led_off(uint32_t pin);
void led_blink(uint32_t pin);


#endif


