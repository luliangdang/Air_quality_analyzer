#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "key.h"
#include "led_alert.h"

#define KEY1_PIN	GET_PIN(A,0)

/* 按键中断回调函数 */
void key_callback(void *args)
{
		if(rt_pin_read(KEY1_PIN)==1)
		{
				rt_kprintf("press\nmsh >");
				rt_pin_write(LED1_PIN, PIN_LOW);
		}
		else if(rt_pin_read(KEY1_PIN)==0)
		{
				rt_kprintf("up\nmsh >");
				rt_pin_write(LED1_PIN, PIN_HIGH);
		}
}
/* 按键初始化 */
void key_init(void)
{
		/* 按键引脚为输入模式 */
		rt_pin_mode(KEY1_PIN, PIN_MODE_INPUT_PULLDOWN);
}
/* 按键测试函数 */
static int key_sample(void)
{
		/* 按键引脚初始化 */
		key_init();
		/* 绑定中断，边沿触发模式，回调函数名为key_callback */
		rt_pin_attach_irq(KEY1_PIN, PIN_IRQ_MODE_RISING_FALLING, key_callback, RT_NULL);
		/* 使能中断 */
		rt_pin_irq_enable(KEY1_PIN, PIN_IRQ_ENABLE);
}
MSH_CMD_EXPORT(key_sample, key press sample);


