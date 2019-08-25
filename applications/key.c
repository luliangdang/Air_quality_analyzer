#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define KEY1_PIN	GET_PIN(A,0)

unsigned int count = 0;
void key_callback(void *args)
{
//		rt_thread_delay(1);
		if(rt_pin_read(KEY1_PIN)==1)
		{
				rt_kprintf("press\nmsh >");
				rt_pin_write(LED1_PIN, 1);
		}
		else if(rt_pin_read(KEY1_PIN)==0)
		{
				rt_kprintf("up\nmsh >");
				rt_pin_write(LED1_PIN, 0);
		}
}

void key_sample(void)
{
		count = 0;
	
		rt_pin_mode(KEY1_PIN, PIN_MODE_INPUT_PULLDOWN);
		
		rt_pin_attach_irq(KEY1_PIN, PIN_IRQ_MODE_RISING_FALLING, key_callback, RT_NULL);
		
		rt_pin_irq_enable(KEY1_PIN, PIN_IRQ_ENABLE);
}
MSH_CMD_EXPORT(key_sample, key press sample);

