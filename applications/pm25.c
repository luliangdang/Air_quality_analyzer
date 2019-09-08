
#include "pm25.h"
#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>

static int rt_hw_pms5003_init(void);
static int pms5003_thread_entry(void *parameter);

/* 串口设备句柄 */
static rt_device_t serial = RT_NULL;

#define PM25_UART_NAME "uart3"

struct pms5003_data_type pms5003_data;

/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;

    result = rt_mq_send(&rx_mq, &msg, sizeof(msg));
    if ( result == -RT_EFULL)
    {
        /* 消息队列满 */
        rt_kprintf("message queue full！\n");
    }
    return result;
}

void pms5003ReciveDataAnl(uint8_t *data_buffer)
{
		uint8_t i;
		uint32_t sum = 0;
		
		/* 校验和计算 */
		for(i = 0; i < 30; i++)
		{
				sum += data_buffer[i];
		}
		
		/* 接收数据校验和判断 */
		if(sum != ((data_buffer[30]<<8) | data_buffer[31]))
		{
				return;
		}
		
		/* 数据校验成功，数据解析 */
		pms5003_data.pm1_val = (data_buffer[10]<<8) | data_buffer[11];
		pms5003_data.pm25_val = (data_buffer[12]<<8) | data_buffer[13];
		pms5003_data.pm10_val = (data_buffer[14]<<8) | data_buffer[15];
}

void pms5003s_read_bit(uint8_t data)
{
		static uint8_t state = 0;
		static uint8_t data_len = 30, data_cnt;		/* 数据帧长度 */
		static uint8_t rxBuffer[33];		/* 数据接收缓存区 */
	
		if(state == 0 && data == 0x42)		/* 起始位1判断 */
		{
				state = 1;
				rxBuffer[0] = data;
		}
		else if(state ==1 && data == 0x4d)/* 起始位2判断 */
		{
				state = 2;
		}
		else if(state ==2 && data_len >0)	/* 数据接收 */
		{
				data_len--;
				rxBuffer[2 + data_cnt++] = data;
				if(data_len ==0)	state = 3;
		}
		else if(state == 3)								/* 数据接收完成，进行数据处理 */
		{
				state = 0;
				data_len = 30;
				data_cnt = 0;
				pms5003ReciveDataAnl(rxBuffer);
		}
}

/* read pms5003 device data */
static int pms5003_read_pm25(void)
{
		rt_kprintf("pm1_val = %d\n", pms5003_data.pm1_val);
		rt_kprintf("pm10_val = %d\n", pms5003_data.pm10_val);
		rt_kprintf("pm25_val = %d\n", pms5003_data.pm25_val);
}

/* 线程入口函数 */
static int pms5003_thread_entry(void *parameter)
{
		struct rx_msg msg;
    rt_err_t result;
    rt_uint32_t rx_length;
    static char rx_buffer[RT_SERIAL_RB_BUFSZ + 1];
	
		while(1)
		{
				rt_memset(&msg, 0, 33);
        /* 从消息队列中读取消息*/
        result = rt_mq_recv(&rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            /* 从串口读取数据*/
            rx_length = rt_device_read(msg.dev, 0, rx_buffer, msg.size);
						for (int i = 0;i < 33; i++)
						{
								pms5003s_read_bit(rx_buffer[i]);
						}
            /* 打印数据 */
            rt_kprintf("%s\n",rx_buffer);
        }
		}
}

static int pms5003_init(void)
{
		rt_err_t ret = RT_EOK;
		rt_hw_pms5003_init();
		
		rt_thread_t pms_thread = rt_thread_create("pms5003",							/* 线程名称 */
																							pms5003_thread_entry,	/* 线程入口函数 */
																							RT_NULL,								/* 线程传递参数 */
																							1024,									/* 线程栈大小 */
																							25,										/* 线程优先级 */
																							100);									/* 线程轮询时间 */
		if(pms_thread != RT_NULL)
		{
				rt_thread_startup(pms_thread);
		}
		else
    {
        ret = RT_ERROR;
    }
		
		return ret;
		
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(pms5003_init, uart device dma sample);


/* init the pm2.5 device */
static int rt_hw_pms5003_init(void)
{
		rt_err_t ret = RT_EOK;
		struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; /* 配置参数 */
    char uart_name[RT_NAME_MAX];
		static char msg_pool[256];
		
		/* 初始化消息队列 */
    rt_mq_init(&rx_mq, "rx_mq",
               msg_pool,                 /* 存放消息的缓冲区 */
               sizeof(struct rx_msg),    /* 一条消息的最大长度 */
               sizeof(msg_pool),         /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);        /* 如果有多个线程等待，按照先来先得到的方法分配消息 */
	
		rt_strncpy(uart_name, PM25_UART_NAME, RT_NAME_MAX);
	
		/* 查找系统中的串口设备 */
    serial = rt_device_find(uart_name);
		
		if (!serial)
    {
        rt_kprintf("find %s failed!\n", uart_name);
        return RT_ERROR;
    }
		
		/* 以 DMA 接收及轮询发送方式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_DMA_RX);
							 
		config.baud_rate = BAUD_RATE_9600;
		config.data_bits = DATA_BITS_8;
		config.stop_bits = STOP_BITS_1;
		config.parity = PARITY_NONE;
		
		/* 打开设备后才可修改串口配置参数 */
		rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
		
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_input);
}


