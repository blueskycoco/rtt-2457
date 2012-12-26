/*
 * File      : app.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://openlab.rt-thread.com/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-10-08     Bernard      the first version
 */

/**
 * @addtogroup wh44b0
 */
/** @{ */
#include <rtthread.h>
#ifdef RT_USING_LWIP
#include <netif/ethernetif.h>
#include "rtl8019.h"
#endif
void rt_led_thread_entry(void *parameter)
{
		/* LwIP Initialization */
#ifdef RT_USING_LWIP
		{
			extern void lwip_sys_init(void);
			eth_system_device_init();
	
			/* register ethernetif device */
			rt_hw_rtl8019_init();
	
			/* re-init device driver */
			rt_device_init_all();
			/* init lwip system */
			lwip_sys_init();
			rt_kprintf("TCP/IP initialized!\n");
			netio_init();
		}
#endif
#if defined(RT_USING_DFS) && defined(RT_USING_LWIP) && defined(RT_USING_DFS_NFS)
			/* NFSv3 Initialization */
			nfs_init();
	
			if (dfs_mount(RT_NULL, "/nfs", "nfs", 0, RT_NFS_HOST_EXPORT) == 0)
				rt_kprintf("NFSv3 File System initialized!\n");
			else
			{
				if(mkdir("/nfs",0777)==RT_EOK)
				{
					if (dfs_mount(RT_NULL, "/nfs", "nfs", 0, RT_NFS_HOST_EXPORT) == 0)
					{
						rt_kprintf("nfs mount on /nfs ok\n");
					}
					else
						rt_kprintf("nfs mount on /nfs failed!\n");
				}
				else
					rt_kprintf("nfs mount on /nfs failed!\n");
			}

#endif
	while (1)
	{
		/* light on leds for one second */
		//rt_kprintf("Led on rom\n");
		rt_hw_led_set(0xff);
		rt_thread_delay(1000);
		//rt_kprintf("Led off rom\n");
		/* light off leds for one second */
		rt_hw_led_set(0x00);
		rt_thread_delay(1000);
	}
}

/* application start function */
int rt_application_init()
{
	rt_thread_t led_thread;
	led_thread = rt_thread_create("led",
								rt_led_thread_entry, RT_NULL,
								512, 20, 20);
	if (led_thread != RT_NULL)
		rt_thread_startup(led_thread);
	return 0;	/* empty */
}

/** @} */
