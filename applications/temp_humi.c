#include "temp_humi.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "sensor.h"
#include "sensor_dallas_dht11.h"
#include "drv_gpio.h"
#include "led_alert.h"


/* 添加DHT11数据引脚 */
#define DHT11_DATA_PIN    GET_PIN(B, 12)

/* 温湿度线程入口函数 */
static void temp_humi_read_entry(void *parameter)
{
		/* 创建一个新的设备对象句柄 */
		rt_device_t dev = RT_NULL;
		/* 创建一个传感器数据对象 */
    struct rt_sensor_data sensor_data;
    rt_size_t res;
		/* 读取数据频率 */
    rt_uint8_t get_data_freq = 1; /* 1Hz */
		/* 查找设备 */
    dev = rt_device_find("temp_dht11");
    if (dev == RT_NULL)
    {
				rt_kprintf(" find device failed\n");
        return;
    }
		/* 开启设备 */
		if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("open device failed!\n");
        return;
    }
		/* 设置读取数据频率 */
		rt_device_control(dev, RT_SENSOR_CTRL_SET_ODR, (void *)(&get_data_freq));
		
		while (1)
    {
				/* 读取设备数据 */
        res = rt_device_read(dev, 0, &sensor_data, 1);

        if (res != 1)
        {
            rt_kprintf("read data failed! result is %d\n", res);
            rt_device_close(dev);
            return;
        }
        else
        {
            if (sensor_data.data.temp >= 0)
            {
                uint8_t temp = (sensor_data.data.temp & 0xffff) >> 0;      // get temp
                uint8_t humi = (sensor_data.data.temp & 0xffff0000) >> 16; // get humi
                rt_kprintf("temp:%d, humi:%d\nmsh />" ,temp, humi);
            }
        }
				led_blink(LED0_PIN);
        rt_thread_delay(800);
    }
		/* 关闭设备 */
		rt_device_close(dev);
}

/* 创建DHT11线程 */
static int dht11_sample_thread(void)
{
		/* 创建dht11线程对象 */
		rt_thread_t dht11_thread;
		/* 创建并配置线程参数 */
		dht11_thread = rt_thread_create("dht11_tem",
                                     temp_humi_read_entry,
                                     "dht11",
                                     1024,
                                     RT_THREAD_PRIORITY_MAX / 2,
                                     20);
		/* 开启线程 */
		if(dht11_thread != RT_NULL)
		{
				rt_thread_startup(dht11_thread);
		}
		return RT_EOK;
}
MSH_CMD_EXPORT(dht11_sample_thread, dht11 test sample);
//INIT_APP_EXPORT(dht11_sample_thread);
/* 初始化DHT11数据引脚 */
static int rt_hw_dht11_port(void)
{
    struct rt_sensor_config cfg;
    
    cfg.intf.user_data = (void *)DHT11_DATA_PIN;
    rt_hw_dht11_init("dht11", &cfg);

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_dht11_port);
