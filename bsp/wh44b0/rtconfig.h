/* RT-Thread config file */
#ifndef __RTTHREAD_CFG_H__
#define __RTTHREAD_CFG_H__
//#define RT_DEBUG
//#define RT_DEBUG_SCHEDULER 1

/* RT_NAME_MAX*/
#define RT_NAME_MAX	8

/* RT_ALIGN_SIZE*/
#define RT_ALIGN_SIZE	4

/* PRIORITY_MAX*/
#define RT_THREAD_PRIORITY_MAX	256

/* Tick per Second*/
#define RT_TICK_PER_SECOND	1000

/* SECTION: RT_DEBUG */
/* Thread Debug*/
/* #define RT_THREAD_DEBUG */
//#define DFS_DEBUG 1
#define RT_USING_DFS
#define RT_USING_DFS_DEVFS
#define RT_USING_DFS_ROMFS
//#define RT_USING_MTD_NOR
//#define RT_USING_DFS_JFFS2
#define RT_USING_MTD_NAND
#define RT_USING_DFS_UFFS
#define RT_USING_DFS_NFS

/* the max number of mounted filesystem */
#define DFS_FILESYSTEMS_MAX			10
/* the max number of opened files 		*/
#define DFS_FD_MAX					4

/* Using Hook*/
#define RT_USING_HOOK

/* SECTION: IPC */
/* Using Semaphore*/
#define RT_USING_SEMAPHORE

/* Using Mutex*/
#define RT_USING_MUTEX

/* Using Event*/
#define RT_USING_EVENT

/* Using Faset Event*/
/* #define RT_USING_FASTEVENT */

/* Using MailBox*/
#define RT_USING_MAILBOX

/* Using Message Queue*/
#define RT_USING_MESSAGEQUEUE

/* SECTION: Memory Management */
/* Using Memory Pool Management*/
#define RT_USING_MEMPOOL

/* Using Dynamic Heap Management*/
#define RT_USING_HEAP

/* Using Small MM*/
/*#define RT_USING_SMALL_MEM*/

/* Using SLAB Allocator*/
#define RT_USING_SLAB

/* SECTION: Device System */
/* Using Device System*/
#define RT_USING_DEVICE

/* SECTION: Console options */
/* the buffer size of console*/
#define RT_CONSOLEBUF_SIZE	128
#define RT_USING_CONSOLE
/* SECTION: FinSH shell options */
/* Using FinSH as Shell*/
#define RT_USING_FINSH
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define FINSH_THREAD_STACK_SIZE	4096

/* SECTION: a mini libc */
/* Using mini libc library*/
/*#define RT_USING_MINILIBC*/
#define RT_USING_NEWLIB
#define RT_USING_PTHREADS
/* SECTION: C++ support */
/* Using C++ support*/
/* #define RT_USING_CPLUSPLUS */

/* SECTION: lwip, a lighwight TCP/IP protocol stack */
/* Using lighweight TCP/IP protocol stack*/
#define RT_USING_LWIP
#define RT_LWIP_USING_RT_MEM
#define RT_LWIP_TCP_PCB_NUM	5
#define RT_LWIP_TCP_SND_BUF	8192
#define RT_LWIP_TCP_WND	8192
#define RT_LWIP_TCP_SEG_NUM	40
#define RT_LWIP_TCPTHREAD_PRIORITY	12
#define RT_LWIP_TCPTHREAD_MBOX_SIZE	32
#define RT_LWIP_TCPTHREAD_STACKSIZE	4096
#define RT_LWIP_ETHTHREAD_PRIORITY	144
#define RT_LWIP_ETHTHREAD_MBOX_SIZE	32
#define RT_LWIP_ETHTHREAD_STACKSIZE	512
#define RT_NFS_HOST_EXPORT	"192.168.0.5:/"


/* Trace LwIP protocol*/
/* #define RT_LWIP_DEBUG */
#define RT_LWIP_DNS

/* Enable ICMP protocol*/
#define RT_LWIP_ICMP

/* Enable IGMP protocol*/
#define RT_LWIP_IGMP

/* Enable UDP protocol*/
#define RT_LWIP_UDP

/* Enable TCP protocol*/
#define RT_LWIP_TCP

/* Enable SNMP protocol*/
/* #define RT_LWIP_SNMP */

/* Using DHCP*/
/* #define RT_LWIP_DHCP */

/* ip address of target*/
#define RT_LWIP_IPADDR0	192
#define RT_LWIP_IPADDR1	168
#define RT_LWIP_IPADDR2	0
#define RT_LWIP_IPADDR3	30

/* gateway address of target*/
#define RT_LWIP_GWADDR0	192
#define RT_LWIP_GWADDR1	168
#define RT_LWIP_GWADDR2	0
#define RT_LWIP_GWADDR3	1

/* mask address of target*/
#define RT_LWIP_MSKADDR0	255
#define RT_LWIP_MSKADDR1	255
#define RT_LWIP_MSKADDR2	255
#define RT_LWIP_MSKADDR3	0

#endif
