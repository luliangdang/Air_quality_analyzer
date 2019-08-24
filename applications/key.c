#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define KEY1_PIN	GET_PIN(A,0)

unsigned int count = 0;
void key_callback(void *args)
{
		count++;
		rt_kprintf("%d   ",count);
		rt_kprintf("ok\n");
		if(count>100)
		{
				rt_pin_irq_enable(KEY1_PIN, PIN_IRQ_DISABLE);
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

