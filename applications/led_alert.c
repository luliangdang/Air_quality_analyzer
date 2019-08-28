#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "led_alert.h"

/* led引脚初始化 */
void led_init(uint32_t pin)
{
		rt_pin_mode(pin, PIN_MODE_OUTPUT);
}
/* 开启led */
void led_on(uint32_t pin)
{
		rt_pin_write(pin, PIN_LOW);
}
/* 关闭led */
void led_off(uint32_t pin)
{
		rt_pin_write(pin, PIN_HIGH);
}
/* le闪烁 */
void led_blink(uint32_t pin)
{
			led_on(pin);
			rt_thread_delay(100);
			led_off(pin);
			rt_thread_delay(100);
}
//MSH_CMD_EXPORT(led_blink, led blink sample);
