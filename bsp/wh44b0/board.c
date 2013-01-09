/*
 * File      : board.c
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
 * 2006-09-24     Bernard      add rt_hw_finsh_init implementation
 */

#include <rtthread.h>
#include <rthw.h>

#include <s3c44b0.h>
#include "board.h"
#define UART0	((struct uartport *)&U0BASE)
struct serial_int_rx uart0_int_rx;
struct serial_device uart0 =
{
	UART0,
	&uart0_int_rx,
	RT_NULL
};
struct rt_device uart0_device;

#define UART1	((struct uartport *)&U1BASE)
struct serial_int_rx uart1_int_rx;
struct serial_device uart1 =
{
	UART1,
	&uart1_int_rx,
	RT_NULL
};
struct rt_device uart1_device;

//#define BOARD_DEBUG
extern void rt_serial_putc(const char ch);

#define DATA_COUNT 0xfff

/**
 * This function will handle serial
 */
static void rt_serial0_handler(int vector)
{
	rt_hw_serial_isr(&uart0_device);
}

/**
 * This function will handle serial
 */
static void rt_serial1_handler(int vector)
{
	rt_hw_serial_isr(&uart1_device);
}

/**
 * This function will handle init uart
 */
static void rt_hw_uart_init(void)
{
	int i;
	rt_uint32_t divisor = 0;	

	/* FIFO enable, Tx/Rx FIFO clear */
	uart0.uart_device->ufcon = 0x0;
	/* disable the flow control */
	uart0.uart_device->umcon = 0x0;
	/* Normal,No parity,1 stop,8 bit */
	uart0.uart_device->ulcon = 0x3;
	/*
	 * tx=level,rx=edge,disable timeout int.,enable rx error int.,
	 * normal,interrupt or polling
	 */
	uart0.uart_device->ucon = 0x245;
	/* Set uart0 bps */	
	#if defined(__FLASH_BUILD__)
	divisor = ((int)(66000000/16./115200 + 0.5) -1);//0x20;
	#else
	divisor = ((int)(40000000/16./115200 + 0.5) -1);//0x20;
	#endif
	uart0.uart_device->ubrd = divisor;
	/* output PCLK to UART0/1, PWMTIMER */
	CLKCON |= 0x0D00;

	/* FIFO enable, Tx/Rx FIFO clear */
	uart1.uart_device->ufcon = 0x0;
	/* disable the flow control */
	uart1.uart_device->umcon = 0x0;
	/* Normal,No parity,1 stop,8 bit */
	uart1.uart_device->ulcon = 0x3;
	/*
	 * tx=level,rx=edge,disable timeout int.,enable rx error int.,
	 * normal,interrupt or polling
	 */
	uart1.uart_device->ucon = 0x245;
	/* Set uart0 bps */
	uart1.uart_device->ubrd = divisor;
	
	for (i = 0; i < 100; i++);

	
	rt_hw_interrupt_install(INT_URXD0, rt_serial0_handler, RT_NULL);
	rt_hw_interrupt_umask(INT_URXD0);

	rt_hw_interrupt_install(INT_URXD1, rt_serial1_handler, RT_NULL);
	rt_hw_interrupt_umask(INT_URXD1);	
}

void rt_timer_handler(int vector)
{
#ifdef BOARD_DEBUG
	rt_kprintf("timer handler, increase a tick\n");
#endif

	rt_tick_increase();
}

void rt_hw_port_init(void)
{
	/* PORT A GROUP */
	/* BIT 9	8	7	6	5	4	3	2	1	0	*/
	/* A24	  A23	A22	A21	A20	A19	A18	A17	A16	A0	*/	      
	/* 1		1	1	1	1	1	1	1	1	1	*/
	PCONA = 0x3ff;
	PDATA = 0x2db;

	/* PORT B GROUP */
	/* BIT 10   9    8       7      6        5     4     3     2     1     0    */
	/* /CS5 /CS4 /CS3    /CS2   /CS1     nWBE3 nWBE2 /SRAS /SCAS SCLS  SCKE	*/
	/* NC   NC   RTL8019 USBD12 NV_Flash NC    NC    Sdram Sdram Sdram Sdram*/
	/* 0,   0,   1,      1,     1,       0,    0,    1,    1,    1,    1    */
	PDATB = 0x4f;
	PCONB = 0x7cf;

	/* PORT C GROUP */
	/* BUSWIDTH=16													*/
	/* PC15		14		13		12		11		10		9		8	*/
	/* o		o		RXD1	TXD1	o		o		o		o	*/
	/* NC		NC		Uart1	Uart1	NC		NC		NC		NC	*/
	/* 01		01		11		11		01		01		01		00	*/

	/* PC7		6		5		4		3		2		1		0	*/
	/* o		o		o		o		o		o		o		o	*/
	/* NC		NC		NC		NC		NFALE 	NFCLE 	NFCE 	NFRB*/
	/* 01		01		01		01		01		01		01		00	*/
	PDATC = 0x0100;	/* All IO is low */
	PCONC = 0xfff4ff54;
	PUPC  = 0x0000;	/* PULL UP RESISTOR should be enabled to I/O */

	/* PORT D GROUP */
	/* PORT D GROUP(I/O OR LCD)										*/
	/* BIT7		6		5		4		3		2		1		0	*/
	/* VF		VM		VLINE	VCLK	VD3		VD2		VD1		VD0	*/
	/* 01		01		01		01		01		01		01		01	*/
	PDATD= 0x0;
	PCOND= 0xaaaa;	
	PUPD = 0x00; /* These pins must be set only after CPU's internal LCD controller is enable */
	
	/* PORT E GROUP  */
	/* Bit 8		7		6		5		4		3		2		1		0		*/
	/* ENDLAN	LED3	LED2	LED1	LED0	BEEP	RXD0	TXD0	CLKOUT	*/ 
	/* 00		01		01		01		01		01		10		10		11		*/
	PCONE	= 0x20428;	/*0->input, 1 2->TXD0 RXD0, 3 4->input, 5->led, 6->buzzer, 7->led, 8->CODECLK */
	PDATE	= 0x57;
	PUPE	= 0x000;	/* disable all pull-up */
	
	/* PORT F GROUP */
	/* Bit8		7		6		5		 4		3		2		1		0		*/   
	/* IISCLK	IISDI	IISDO	IISLRCK	Input	Input	Input	IICSDA	IICSCL	*/
	/* 100		010		010		001		00		01		01		10		10		*/
	PDATF = 0x2f;
	PCONF = 0x24900a;
	PUPF  = 0x000;

	/* PORT G GROUP */
	/* BIT7		6		5		4		3		2		1		0	 */
	/* INT7		INT6		INT5		INT4		INT3		INT2		INT1		INT0	*/
	/* S3		S4		S5		S6		NIC		EXT		IDE		USB	*/
	/* 11      11      11      11      11      11      11      11       */
	PDATG = 0xfc;
	PCONG = 0xff3c; /* eint1 is eth interrupt in WH44B0 */
	PUPG  = 0x00;	/* should be enabled   */

	SPUCR=0x6;  /* D15-D0 pull-up disable */

	/* all external interrupts are triggered by low level */
	EXTINT=(4<<16)|(4<<8)|(4<<4);
}
/**
 * This function will init timer0 for system ticks
 */
static  void rt_hw_timer_init()
{

	/* set timer0 register */
	/* stop timer */
	TCON 	&= ~(0x00000001);

	/* dead zone = 0, pre = 150 */
	TCFG0 = 0x00000095;
	/* all are interrupt mode */
	TCFG1 = 0x00000003;

	TCNTB0 = (rt_int32_t)(1000000 / (4 *16* RT_TICK_PER_SECOND)) - 1;//DATA_COUNT;
	TCMPB0 = 0;

	/* manual update */
	TCON	|= 0x00000002;

	/* auto reload on,output inverter off */
	TCON 	&= ~(0x0000000f);
	TCON 	|= (0x00000008);

	/* install timer handler */
	rt_hw_interrupt_install(INT_TIMER0, rt_timer_handler, RT_NULL);
	rt_hw_interrupt_umask(INT_TIMER0);

	/* start timer */
	TCON 	|=(0x00000001);

}
void INTEINT4567_handler(int irqno)
{
    rt_uint32_t eint_pend;
	rt_uint32_t extint_pend;
    eint_pend = INTPND;
	extint_pend = EXTINTPND;
    /* EINT4 : SL811HS */
    if( eint_pend & (1<<INT_EINT4567) )
    {
    	if(extint_pend & (1<<0))
        	INTEINT4_handler(0);
		if(extint_pend & (1<<1))
        	INTEINT5_handler(1);
		if(extint_pend & (1<<2))
        	INTEINT6_handler(2);
		if(extint_pend & (1<<3))
        	INTEINT7_handler(3);
    }

	/* clear EINT pending bit */
	INTPND = eint_pend;
	EXTINTPND =	extint_pend;
}

/**
 * This function will init lumit4510 board
 */
void rt_hw_board_init()
{
	/* init port setting */
	rt_hw_port_init();	

	rt_hw_uart_init();

	rt_hw_timer_init();
	
	rt_hw_interrupt_install(INT_EINT4567, INTEINT4567_handler, RT_NULL);
	rt_hw_interrupt_umask(INT_EINT4567);

}

void rt_hw_led_set(rt_uint32_t led)
{
	if((led & 0x01)==0x01)		/* D1 */
		PDATC = PDATC | (1<<1) ;
	else 
		PDATC = PDATC & (~(1<<1)) ;
	
	if((led & 0x02)==0x02)		/* D2 */
		PDATC = PDATC | (1<<2) ;
	else
		PDATC = PDATC & (~(1<<2)) ;

	if((led & 0x04)==0x04)		/* D3 */
		PDATC = PDATC | (1<<3) ;
	else
		PDATC = PDATC & (~(1<<3)) ;

	if((led & 0x08)==0x08)		/* D4 */
		PDATE = PDATE | (1<<5) ;
	else
		PDATE = PDATE & (~(1<<5)) ;

}

/* led loop */
void rt_hw_led_flash(void)
{
	register int i;
	//rt_kprintf("in timer hook\n");
	rt_hw_led_set(0x01);
	for ( i = 0; i < 2000000; i++);
	
	rt_hw_led_set(0x02);
	for ( i = 0; i < 2000000; i++);
	
	rt_hw_led_set(0x04);
	for ( i = 0; i < 2000000; i++);
}

#ifdef RT_USING_FINSH
extern void finsh_notify(void);
void rt_serial_isr(int vector)
{
	//finsh_notify();
}

void rt_hw_finsh_init()
{
	/* install UART isr */
	rt_hw_interrupt_install(INT_URXD0, rt_serial_isr, RT_NULL);
	rt_hw_interrupt_umask(INT_URXD0);
}
#endif

/*void rt_hw_console_output(const char* string)
{
	while (*string)
	{
		rt_serial_putc(*string++);
	}
}*/

/*@}*/
