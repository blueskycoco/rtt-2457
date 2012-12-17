/*
 * File      : startup.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://openlab.rt-thread.com/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-09-23     Bernard      first version
 * 2006-10-05     Bernard      add .nobbs attribute for _svc_stack_start
 */

#include <rtthread.h>
#include <rthw.h>

#include <s3c44b0.h>
#include <board.h>
#ifdef RT_USING_DFS
#include <dfs_init.h>
#include <dfs_fs.h>
#endif
#ifdef RT_USING_DFS_UFFS
#include <dfs_uffs.h>
#endif
#ifdef RT_USING_DFS_ROMFS
#include <dfs_romfs.h>
#endif
#ifdef RT_USING_LWIP
#include <netif/ethernetif.h>
#include "rtl8019.h"
#endif

#if defined(__CC_ARM)
	extern int Image$$ER_ZI$$ZI$$Base;
	extern int Image$$ER_ZI$$ZI$$Length;
	extern int Image$$ER_ZI$$ZI$$Limit;
#elif (defined (__GNUC__))
	rt_uint8_t _undefined_stack_start[128];
	rt_uint8_t _abort_stack_start[128];
	rt_uint8_t _fiq_stack_start[1024];
	rt_uint8_t _irq_stack_start[1024];	
	rt_uint8_t _svc_stack_start[4096] SECTION(".nobss");
#endif
extern struct serial_device uart0;
extern struct rt_device uart0_device;
extern struct serial_device uart1;
extern struct rt_device uart1_device;

extern void rt_hw_interrupt_init(void);
extern void rt_serial_init(void);
extern void rt_hw_cpu_icache_enable(void);

/**
 * @addtogroup wh44b0
 */
/*@{*/

#ifdef __CC_ARM
extern int Image$$RW_RAM1$$ZI$$Limit;
#else
extern int __bss_end;
#endif

#ifdef RT_USING_FINSH
extern void finsh_system_init(void);
#endif
extern int  rt_application_init(void);

/**
 * This function will startup RT-Thread RTOS.
 */
void rtthread_startup(void)
{
	int flag=0;
	/* enable cpu cache */
	rt_hw_cpu_icache_enable();

	/* init hardware interrupt */
	rt_hw_interrupt_init();

	/* init board */
	rt_hw_board_init();

	/* init hardware serial */
	//rt_serial_init();
	
	rt_show_version();

	/* init tick */
	rt_system_tick_init();

	/* init kernel object */
	rt_system_object_init();

	/* init timer system */
	rt_system_timer_init();
	/* init memory system */
#ifdef RT_USING_HEAP
#ifdef __CC_ARM
	rt_system_heap_init((void*)&Image$$RW_RAM1$$ZI$$Limit, (void*)0xC800000);
#else
	rt_system_heap_init((void*)&__bss_end, (void*)0xC800000);
#endif
#endif

	/* init scheduler system */
	rt_system_scheduler_init();

#ifdef RT_USING_HOOK
	/* set idle thread hook */
	//rt_thread_idle_sethook(rt_hw_led_flash);
#endif
#ifdef RT_USING_DFS
	dfs_init();
#ifdef RT_USING_MTD_NOR
    sst39vfxx_mtd_init("nor", 10, 30);
	dfs_jffs2_init();
#endif
#ifdef RT_USING_MTD_NAND
	nand_mtd_init();
	dfs_uffs_init();
#endif 
#ifdef RT_USING_DFS_ROMFS
	dfs_romfs_init();
#endif
#ifdef RT_USING_DFS_DEVFS
	devfs_init();	
#endif
#endif
#ifdef RT_USING_DEVICE
	/* register uart0 */
	rt_hw_serial_register(&uart0_device, "uart0",
		RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
		&uart0);

	/* register uart2, used for RTI debug */
	rt_hw_serial_register(&uart1_device, "uart1",
		RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
		&uart1);

    rt_device_init_all();
#endif

	/* init application */
	rt_application_init();

#ifdef RT_USING_FINSH
	/* init the finsh input */
	
	/* init finsh */
	finsh_system_init();
	#ifdef RT_USING_DEVICE
	finsh_set_device("uart0");
	#endif
#endif

	/* init idle thread */
	rt_thread_idle_init();

	/* unmask interrupt */
	rt_hw_interrupt_umask(INT_GLOBAL);
	
#ifdef RT_USING_DFS
#ifdef RT_USING_MTD_NOR
	if (dfs_mount("nor", "/", "jffs2", 0, 0) == 0)
	{
		rt_kprintf("jffs2 initialized!\n");
		flag=1;
	}
    else
      	rt_kprintf("jffs2 initialzation failed!\n");
#endif
#ifndef RT_USING_MTD_NOR

	if (dfs_mount("nand0", "/", "uffs", 0, 0) == 0)
	{
		rt_kprintf("uffs mount / partion ok\n");
		flag=1;
	}
	else
		rt_kprintf("uffs mount / partion failed!\n");
	if (dfs_mount("nor", "/nor", "uffs", 0, 0) == 0)
	{
		rt_kprintf("uffs mount on nor ok\n");
	}
	else
	{
		if(flag==1 && mkdir("/nor",0777)==RT_EOK)
		{
			if (dfs_mount("nor", "/nor", "uffs", 0, 0) == 0)
			{
				rt_kprintf("uffs mount on nor ok\n");
			}
			else
				rt_kprintf("uffs mount on nor failed!\n");
		}
		else
			rt_kprintf("uffs mount on nor failed!\n");
	}
#else
	if (dfs_mount("nand0", "/nand0", "uffs", 0, 0) == 0)
	{
		rt_kprintf("uffs mount /nand0 partion ok\n");
	}
	else
	{
		if(flag==1 && mkdir("/nand0",0777)==RT_EOK)
		{
			if (dfs_mount("nand0", "/nand0", "uffs", 0, 0) == 0)
			{
				rt_kprintf("uffs mount on /nand0 ok\n");
			}
			else
				rt_kprintf("uffs mount on /nand0 failed!\n");
		}
		else
			rt_kprintf("uffs mount on /nand0 failed!\n");
	}

#endif
#ifdef RT_USING_DFS_DEVFS
	if (dfs_mount(RT_NULL, "/dev", "devfs", 0, 0) == 0)
	{
		rt_kprintf("devfs initialized!\n");
	}
	else
		rt_kprintf("devfs initialzation failed!\n");
	#ifdef RT_USING_NEWLIB
	libc_system_init("uart0");
	#endif
#endif
#ifdef RT_USING_DFS_ROMFS
	if (dfs_mount(RT_NULL, "/rom", "rom", 0, &romfs_root) == 0)
	{
		rt_kprintf("romfs initialized!\n");
	}
	else
	{
		if(flag==1 && mkdir("/rom",0777)==RT_EOK)
		{
			if (dfs_mount(RT_NULL, "/rom", "rom", 0, &romfs_root) == 0)
			{
				rt_kprintf("romfs mount on /rom ok\n");
			}
			else
				rt_kprintf("romfs mount on /rom failed!\n");
		}
		else
			rt_kprintf("romfs mount on /rom failed!\n");
	}
#endif
#ifdef RT_USING_MTD_NAND
	if (dfs_mount("nand1", "/nand1", "uffs", 0, 0) == 0)
	{
		rt_kprintf("uffs mount nand1 partion ok\n");
	}
	else
	{
		if(flag==1 && mkdir("/nand1",0777)==RT_EOK)
		{
			if (dfs_mount("nand1", "/nand1", "uffs", 0, 0) == 0)
			{
				rt_kprintf("uffs mount on /nand1 ok\n");
			}
			else
				rt_kprintf("uffs mount on /nand1 failed!\n");
		}
		else
			rt_kprintf("uffs mount on /nand1 failed!\n");
	}
	if (dfs_mount("nand2", "/nand2", "uffs", 0, 0) == 0)
	{
		rt_kprintf("uffs mount nand2 partion ok\n");
	}
	else
	{
		if(flag==1 && mkdir("/nand2",0777)==RT_EOK)
		{
			if (dfs_mount("nand2", "/nand2", "uffs", 0, 0) == 0)
			{
				rt_kprintf("uffs mount on /nand2 ok\n");
			}
			else
				rt_kprintf("uffs mount on /nand2 failed!\n");
		}
		else
			rt_kprintf("uffs mount on /nand2 failed!\n");
	}
	if (dfs_mount("nand3", "/nand3", "uffs", 0, 0) == 0)
	{
		rt_kprintf("uffs mount nand3 partion ok\n");
	}
	else
	{
		if(flag==1 && mkdir("/nand3",0777)==RT_EOK)
		{
			if (dfs_mount("nand3", "/nand3", "uffs", 0, 0) == 0)
			{
				rt_kprintf("uffs mount on /nand3 ok\n");
			}
			else
				rt_kprintf("uffs mount on /nand3 failed!\n");
		}
		else
			rt_kprintf("uffs mount on /nand3 failed!\n");
	}
#endif
#endif

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
		}
#endif
#if defined(RT_USING_DFS) && defined(RT_USING_LWIP) && defined(RT_USING_DFS_NFS)
			/* NFSv3 Initialization */
			nfs_init();
	
			if (dfs_mount(RT_NULL, "/nfs", "nfs", 0, RT_NFS_HOST_EXPORT) == 0)
				rt_kprintf("NFSv3 File System initialized!\n");
			else
			{
				if(flag==1 && mkdir("/nfs",0777)==RT_EOK)
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

	rt_kprintf("init finish\n");
	/* start scheduler */
	rt_system_scheduler_start();

	/* never reach here */
	return ;
}

int main(void)
{
	/* invoke rtthread startup */
	rtthread_startup();
}

/*@}*/
