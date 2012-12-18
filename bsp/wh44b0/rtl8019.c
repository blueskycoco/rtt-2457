#include <rtthread.h>
#include <netif/ethernetif.h>

#include "rtl8019.h"
#include <s3c44b0.h>

/*
 * Davicom RTL8019 driver
 *
 * IRQ_LAN connects to EINT1
 * nLAN_CS connects to nGCS3
 */

#define RTL8019_DEBUG		1 
#if RTL8019_DEBUG
#define RTL8019_TRACE	rt_kprintf
#else
#define RTL8019_TRACE(...)
#endif
#define inportw(r) 		(*(volatile rt_uint16_t *)(r))
#define outportw(r, d) 	(*(volatile rt_uint16_t *)(d) = (r))
#define inportb(r) 		(*(volatile rt_uint8_t *)(r))
#define outportb(r, d) 	(*(volatile rt_uint8_t *)(d) = (r))

#define MAX_ADDR_LEN 6
static rt_uint8_t SrcMacID[MAX_ADDR_LEN] = {0x00,0x80,0x48,0x12,0x34,0x56};
struct pbuf *g_pbuf=RT_NULL;
struct e8390_pkt_hdr {
  rt_uint8_t status; /* status */
  rt_uint8_t next;   /* pointer to next packet. */
  rt_uint16_t count; /* header + packet length in bytes */
};
struct rt_rtl8019_eth
{
	/* inherit from ethernet device */
	struct eth_device parent;
	rt_uint8_t txing:1;		/* Transmit Active */
	rt_uint8_t irqlock:1;		/* 8390's intrs disabled when '1'. */
	rt_uint8_t dmaing:1;		/* Remote DMA Active */
	rt_uint8_t tx_start_page, rx_start_page, stop_page;
	rt_uint8_t current_page;	/* Read pointer in buffer  */
	rt_uint8_t interface_num;	/* Net port (AUI, 10bT.) to use. */
	rt_uint8_t txqueue;		/* Tx Packet buffer queue length. */
	rt_int32_t tx1, tx2;			/* Packet lengths for ping-pong tx. */
	rt_int32_t lasttx;			/* Alpha version consistency check. */
	rt_uint8_t startp;
	rt_uint8_t mcfilter[8];
	/* interface address info. */
	rt_uint8_t  dev_addr[MAX_ADDR_LEN];		/* hw address	*/
};
static struct rt_rtl8019_eth rtl8019_device;
static struct rt_semaphore sem_tx_done, sem_lock;

void rt_rtl8019_isr(int irqno);

static void delay_ms(rt_uint32_t ms)
{
	rt_uint32_t len;
	for (;ms > 0; ms --)
		for (len = 0; len < 100; len++ );
}
static void NS8390_trigger_send(rt_uint32_t length,rt_int32_t start_page)
{

	outportb(E8390_NODMA+E8390_PAGE0, e8390_base+E8390_CMD);

	if (inportb(e8390_base + E8390_CMD) & E8390_TRANS)
	{
		RTL8019_TRACE(" trigger_send() called with the transmitter busy.\n");
		return;
	}
	outportb(length & 0xff, e8390_base + EN0_TCNTLO);
	outportb(length >> 8, e8390_base + EN0_TCNTHI);
	outportb(start_page, e8390_base + EN0_TPSR);
	outportb(E8390_NODMA+E8390_TRANS+E8390_START, e8390_base+E8390_CMD);
}

void ei_get_8390_hdr(struct e8390_pkt_hdr *hdr, int ring_page)
{
	/* This *shouldn't* happen. If it does, it's the last thing you'll see */

	if (rtl8019_device.dmaing)
	{
		RTL8019_TRACE("<ei_get_8390_hdr> DMAing conflict in ne_get_8390_hdr [DMAstat:%d][irqlock:%d].\n",rtl8019_device.dmaing, rtl8019_device.irqlock);
		return;
	}

	rtl8019_device.dmaing |= 0x01;
	outportb(E8390_NODMA+E8390_PAGE0+E8390_START, e8390_base+ E8390_CMD);
	outportb(sizeof(struct e8390_pkt_hdr), e8390_base + EN0_RCNTLO);
	outportb(0, e8390_base + EN0_RCNTHI);
	outportb(0, e8390_base + EN0_RSARLO);		/* On page boundary */
	outportb(ring_page, e8390_base + EN0_RSARHI);
	outportb(E8390_RREAD+E8390_START, e8390_base + E8390_CMD);

	//insw(NE_BASE + NE_DATAPORT, hdr, sizeof(struct e8390_pkt_hdr)>>1);
	//rt_memcpy(hdr,(rt_uint16_t *)(e8390_base+EN0_DATAPORT),sizeof(struct e8390_pkt_hdr)>>1);
	rt_uint32_t i;
	for(i=0;i<sizeof(struct e8390_pkt_hdr)>>1;i++)
		((rt_uint16_t *)hdr)[i]=inportw(e8390_base+EN0_DATAPORT);
	outportb(ENISR_RDC, e8390_base + EN0_ISR);	/* Ack intr. */
	rtl8019_device.dmaing &= ~0x01;

	//le16_to_cpus(&hdr->count);
	//RTL8019_TRACE("<ei_get_8390_hdr> hdr->count = %d\n",hdr->count);
}
struct pbuf *ne_block_input(rt_uint32_t count , int ring_offset)
{

	struct pbuf *p,*q;
	/* This *shouldn't* happen. If it does, it's the last thing you'll see */
	if (rtl8019_device.dmaing)
	{
		RTL8019_TRACE("<ne_block_input> DMAing conflict in ne_block_input [DMAstat:%d][irqlock:%d].\n",rtl8019_device.dmaing, rtl8019_device.irqlock);
		return;
	}
	rtl8019_device.dmaing |= 0x01;
	
	/* allocate buffer */
	p = pbuf_alloc(PBUF_LINK, count, PBUF_RAM);
	RTL8019_TRACE("pkt_len %d , g_pbuf->tot_len %d , g_pbuf->len %d\n",count,g_pbuf->tot_len,g_pbuf->len);
	outportb(E8390_NODMA+E8390_PAGE0+E8390_START, e8390_base+ E8390_CMD);
	outportb(count& 0xff, e8390_base + EN0_RCNTLO);
	outportb(count>> 8, e8390_base + EN0_RCNTHI);
	outportb(ring_offset & 0xff, e8390_base + EN0_RSARLO);
	outportb(ring_offset >> 8, e8390_base + EN0_RSARHI);
	outportb(E8390_RREAD+E8390_START, e8390_base + E8390_CMD);
	//insw(NE_BASE + NE_DATAPORT,buf,count>>1);
	rt_uint32_t index=e8390_base+EN0_DATAPORT;
	
	if(p == RT_NULL)
	{
		rt_uint16_t dummy;
		while(count)
		{
			dummy=inportw(e8390_base+EN0_DATAPORT);
			count=count-2;
		}
		if (count & 0x01)
		{
			dummy = inportb((rt_uint8_t *)(index+1));
		}

	}
	else
	{
		rt_uint32_t i;
		rt_uint16_t *data;
		for(q=p;q!=RT_NULL;q=q->next)
		{
			data=(rt_uint16_t *)q->payload;
			for(i=0;i<q->len>>1;i++)
				data[i] = inportw(e8390_base+EN0_DATAPORT);
		}
	}
	outportb(ENISR_RDC, e8390_base + EN0_ISR);	/* Ack intr. */
	rtl8019_device.dmaing &= ~0x01;
	return p;
}

struct pbuf *ei_receive()
{
	rt_uint8_t rxing_page, this_frame, next_frame;
	rt_uint16_t current_offset;
	rt_uint32_t rx_pkt_count = 0;
	struct pbuf *p;
	struct e8390_pkt_hdr rx_frame;
	RTL8019_TRACE("ei_receive==>\n");
	int num_rx_pages = rtl8019_device.stop_page-rtl8019_device.rx_start_page;
	while (++rx_pkt_count < 10)
	{
		int pkt_len, pkt_stat;

		/* Get the rx page (incoming packet pointer). */
		outportb(E8390_NODMA+E8390_PAGE1, e8390_base + E8390_CMD);
		rxing_page = inportb(e8390_base + EN1_CURPAG);
		outportb(E8390_NODMA+E8390_PAGE0, e8390_base + E8390_CMD);

		/* Remove one frame from the ring.  Boundary is always a page behind. */
		this_frame = inportb(e8390_base + EN0_BOUNDARY) + 1;
		if (this_frame >= rtl8019_device.stop_page)
			this_frame = rtl8019_device.rx_start_page;

		/* Someday we'll omit the previous, iff we never get this message.
		   (There is at least one clone claimed to have a problem.)

		   Keep quiet if it looks like a card removal. One problem here
		   is that some clones crash in roughly the same way.
		 */
		if (this_frame != rtl8019_device.current_page && (this_frame!=0x0 || rxing_page!=0xFF))
			RTL8019_TRACE("<ei_receive> mismatched read page pointers %2x vs %2x.\n",this_frame, rtl8019_device.current_page);

		if (this_frame == rxing_page)	/* Read all the frames? */
			break;				/* Done for now */

		current_offset = this_frame << 8;
		ei_get_8390_hdr(&rx_frame, this_frame);

		pkt_len = rx_frame.count - sizeof(struct e8390_pkt_hdr);
		pkt_stat = rx_frame.status;

		next_frame = this_frame + 1 + ((pkt_len+4)>>8);

		/* Check for bogosity warned by 3c503 book: the status byte is never
		   written.  This happened a lot during testing! This code should be
		   cleaned up someday. */
		if (rx_frame.next != next_frame &&
		    rx_frame.next != next_frame + 1 &&
		    rx_frame.next != next_frame - num_rx_pages &&
		    rx_frame.next != next_frame + 1 - num_rx_pages) {
			rtl8019_device.current_page = rxing_page;
			outportb(rtl8019_device.current_page-1, e8390_base+EN0_BOUNDARY);
			continue;
		}

		if (pkt_len < 60  ||  pkt_len > 1518)
		{
			RTL8019_TRACE("<ei_receive> bogus packet size: %d, status=%#2x nxpg=%#2x.\n",rx_frame.count, rx_frame.status,rx_frame.next);
		}
		 else if ((pkt_stat & 0x0F) == ENRSR_RXOK)
		{
			p =	ne_block_input(pkt_len,current_offset + sizeof(rx_frame));			

		}
		else
		{
			RTL8019_TRACE("<ei_receive> bogus packet: status=%#2x nxpg=%#2x size=%d\n",rx_frame.status, rx_frame.next,rx_frame.count);
		}
		next_frame = rx_frame.next;

		/* This _should_ never happen: it's here for avoiding bad clones. */
		if (next_frame >= rtl8019_device.stop_page) {
			RTL8019_TRACE("<ei_receive> next frame inconsistency, %#2x\n", next_frame);
			next_frame = rtl8019_device.rx_start_page;
		}
		rtl8019_device.current_page = next_frame;
		outportb(next_frame-1, e8390_base+EN0_BOUNDARY);
	}

	/* We used to also ack ENISR_OVER here, but that would sometimes mask
	   a real overrun, leaving the 8390 in a stopped state with rec'vr off. */
	outportb(ENISR_RX+ENISR_RX_ERR, e8390_base+EN0_ISR);
	RTL8019_TRACE("ei_receive<==\n");
	return p;
}

/* reception packet. */
struct pbuf *rt_rtl8019_rx(rt_device_t dev)
{

	struct pbuf *p = RT_NULL,*q;
	
	rt_sem_take(&sem_lock, RT_WAITING_FOREVER);
	RTL8019_TRACE("rtl8019 rx ==>\n");

	//wait for uplayer to read back
	p=ei_receive();
	if(p!=RT_NULL)
	{
		rt_uint32_t i;	
		//p=pbuf_alloc(PBUF_LINK, p->tot_len, PBUF_RAM);
		//rt_memcpy(p,g_pbuf,g_pbuf->tot_len);
		q=p;
		RTL8019_TRACE("rx tot_len %d \nbuf :\n",q->tot_len);
		while(q!=RT_NULL)
		{
			for(i=0;i<q->len;i++)
			{
				RTL8019_TRACE("%2d ",((rt_uint8_t *)q->payload)[i]);
				if(i%5==0 && i!=0)
					RTL8019_TRACE("\n");	
			}
	
			q=q->next;
		}
		//pbuf_free(g_pbuf);
		rt_sem_release(&sem_lock);
		RTL8019_TRACE("rtl8019 rx <==\n");
		return p;
	}
	rt_sem_release(&sem_lock);
	RTL8019_TRACE("rtl8019 rx <== error\n");
	return RT_NULL;
}

/* ethernet device interface */
/* transmit packet. */
rt_err_t rt_rtl8019_tx( rt_device_t dev, struct pbuf* p)
{	
	rt_uint32_t send_length = p->tot_len, output_page;
	rt_uint8_t *data;	
	rt_uint32_t i=0,j=0;
	struct pbuf* q;
	RTL8019_TRACE("rtl8019 tx ==>\n");
	/* lock RTL8019 device */
	rt_sem_take(&sem_lock, RT_WAITING_FOREVER);
	q=p;
	if(q->tot_len < ETH_ZLEN)
	{

		data=(rt_uint8_t *)rt_malloc(ETH_ZLEN);
		rt_memset(data, 0, ETH_ZLEN);	// more efficient than doing just the needed bits 
		send_length=ETH_ZLEN;
	}
	else
	{
		if(q->tot_len&1)
			send_length=q->tot_len+1;
		else
			send_length=q->tot_len;
		data=(rt_uint8_t *)rt_malloc(send_length);
		rt_memset(data, 0, send_length);	// more efficient than doing just the needed bits 
	}
	
	RTL8019_TRACE("tx total length %d tot_len %d \nbuf :\n",send_length,q->tot_len);
	while(q != RT_NULL)
	{
		j=i+q->len;
		rt_memcpy((rt_uint8_t *)data, (rt_uint8_t *)q->payload, q->len);
		data=data+q->len;
		for(;i<j;i++)
		{
			RTL8019_TRACE(" %2d ",data[i]);
			if(i%5==0 && i!=0)
				RTL8019_TRACE("\n");
		}		
		i=i+q->len;
		q=q->next;
		
	}
	
	outportb(0x00, e8390_base + EN0_IMR);
	rtl8019_device.irqlock = 1;
	/*
	 * We have two Tx slots available for use. Find the first free
	 * slot, and then perform some sanity checks. With two Tx bufs,
	 * you get very close to transmitting back-to-back packets. With
	 * only one Tx buf, the transmitter sits idle while you reload the
	 * card, leaving a substantial gap between each transmitted packet.
	 */

	if (rtl8019_device.tx1 == 0)
	{
		output_page = rtl8019_device.tx_start_page;
		rtl8019_device.tx1 = send_length;
		if (rtl8019_device.tx2 > 0)
			RTL8019_TRACE(" idle transmitter tx2=%d, lasttx=%d, txing=%d.\n",rtl8019_device.tx2, rtl8019_device.lasttx, rtl8019_device.txing);
	}
	else if (rtl8019_device.tx2 == 0)
	{
		output_page = rtl8019_device.tx_start_page + TX_PAGES/2;
		rtl8019_device.tx2 = send_length;
		if (rtl8019_device.tx1 > 0)
			RTL8019_TRACE("idle transmitter, tx1=%d, lasttx=%d, txing=%d.\n",rtl8019_device.tx1, rtl8019_device.lasttx, rtl8019_device.txing);
	}
	else
	{	/* We should never get here. */
		
		RTL8019_TRACE(" No Tx buffers free! tx1=%d tx2=%d last=%d\n",rtl8019_device.tx1, rtl8019_device.tx2, rtl8019_device.lasttx);
		rtl8019_device.irqlock = 0;
		outportb(ENISR_ALL, e8390_base + EN0_IMR);
		rt_free(data);
		/* unlock RTL8019 device */
		rt_sem_release(&sem_lock);
		return RT_ERROR;
	}

	/*
	 * Okay, now upload the packet and trigger a send if the transmitter
	 * isn't already sending. If it is busy, the interrupt handler will
	 * trigger the send later, upon receiving a Tx done interrupt.
	 */

	//ne_block_output(send_length, p, output_page);

	/* This *shouldn't* happen. If it does, it's the last thing you'll see */
	if (rtl8019_device.dmaing)
	{
		RTL8019_TRACE(" DMAing conflict in ne_block_output. [DMAstat:%d][irqlock:%d]\n",rtl8019_device.dmaing, rtl8019_device.irqlock);
		rtl8019_device.irqlock = 0;
		outportb(ENISR_ALL, e8390_base + EN0_IMR);
		rt_free(data);
		/* unlock RTL8019 device */
		rt_sem_release(&sem_lock);
		return RT_ERROR;
	}
	rtl8019_device.dmaing |= 0x01;
	/* We should already be in page 0, but to be safe... */
	outportb(E8390_PAGE0+E8390_START+E8390_NODMA, e8390_base + E8390_CMD);

	outportb(ENISR_RDC, e8390_base + EN0_ISR);

	/* Now the normal output. */
	outportb(send_length& 0xff, e8390_base + EN0_RCNTLO);
	outportb(send_length>> 8,   e8390_base + EN0_RCNTHI);
	outportb(0x00, e8390_base + EN0_RSARLO);
	outportb(output_page, e8390_base + EN0_RSARHI);

	outportb(E8390_RWRITE+E8390_START, e8390_base + E8390_CMD);
	for(i=0;i<send_length/2;i++)
		outportw(((rt_uint16_t *)data)[i],e8390_base+EN0_DATAPORT);
	//rt_memcpy((volatile rt_uint16_t *)(e8390_base+EN0_DATAPORT),(rt_uint16_t *)data,send_length>>1);
	while ((inportb(e8390_base + EN0_ISR) & ENISR_RDC) == 0)
	{
		//RTL8019_TRACE("Waiting for ENISR_RDC int\n");
		delay_ms(20);
	}

	//RTL8019_TRACE("rtl8019 tx jjj\n");
	outportb(ENISR_RDC, e8390_base + EN0_ISR);	/* Ack intr. */
	rtl8019_device.dmaing &= ~0x01;

	if (!rtl8019_device.txing)
	{
		rtl8019_device.txing = 1;
		NS8390_trigger_send(send_length, output_page);
		if (output_page == rtl8019_device.tx_start_page)
		{
			rtl8019_device.tx1 = -1;
			rtl8019_device.lasttx = -1;
		}
		else
		{
			rtl8019_device.tx2 = -1;
			rtl8019_device.lasttx = -2;
		}
	}
	else rtl8019_device.txqueue++;
	
	//wait for tx int occur
	rt_sem_take(&sem_tx_done, RT_WAITING_FOREVER);

	/* Turn 8390 interrupts back on. */
	rtl8019_device.irqlock = 0;
	outportb(ENISR_ALL, e8390_base + EN0_IMR);
	rt_free(data);

	/* unlock RTL8019 device */
	rt_sem_release(&sem_lock);

	RTL8019_TRACE("rt_rtl8019_tx<==\n");

	return RT_EOK;
}

static void ei_rx_overrun()
{
	rt_uint8_t was_txing, must_resend = 0;

	/*
	 * Record whether a Tx was in progress and then issue the
	 * stop command.
	 */
	was_txing = inportb(e8390_base+E8390_CMD) & E8390_TRANS;
	outportb(E8390_NODMA+E8390_PAGE0+E8390_STOP, e8390_base+E8390_CMD);

	RTL8019_TRACE(" Receiver overrun.\n");

	/*
	 * Wait a full Tx time (1.2ms) + some guard time, NS says 1.6ms total.
	 * Early datasheets said to poll the reset bit, but now they say that
	 * it "is not a reliable indicator and subsequently should be ignored."
	 * We wait at least 10ms.
	 */

	delay_ms(10);

	/*
	 * Reset RBCR[01] back to zero as per magic incantation.
	 */
	outportb(0x00, e8390_base+EN0_RCNTLO);
	outportb(0x00, e8390_base+EN0_RCNTHI);

	/*
	 * See if any Tx was interrupted or not. According to NS, this
	 * step is vital, and skipping it will cause no end of havoc.
	 */

	if (was_txing)
	{
		rt_uint8_t tx_completed = inportb(e8390_base+EN0_ISR) & (ENISR_TX+ENISR_TX_ERR);
		if (!tx_completed)
			must_resend = 1;
	}

	/*
	 * Have to enter loopback mode and then restart the NIC before
	 * you are allowed to slurp packets up off the ring.
	 */
	outportb(E8390_TXOFF, e8390_base + EN0_TXCR);
	outportb(E8390_NODMA + E8390_PAGE0 + E8390_START, e8390_base + E8390_CMD);

	/*
	 * Clear the Rx ring of all the debris, and ack the interrupt.
	 */
	//ei_receive();
	eth_device_ready(&(rtl8019_device.parent));
	outportb(ENISR_OVER, e8390_base+EN0_ISR);

	/*
	 * Leave loopback mode, and resend any packet that got stopped.
	 */
	outportb(E8390_TXCONFIG, e8390_base + EN0_TXCR);
	if (must_resend)
    		outportb(E8390_NODMA + E8390_PAGE0 + E8390_START + E8390_TRANS, e8390_base + E8390_CMD);
}
static void ei_tx_intr()
{
	//unsigned long e8390_base = dev->base_addr;
	//struct ei_device *ei_local = (struct ei_device *) netdev_priv(dev);
	outportb(ENISR_TX, e8390_base + EN0_ISR); /* Ack intr. */

	/*
	 * There are two Tx buffers, see which one finished, and trigger
	 * the send of another one if it exists.
	 */
	rtl8019_device.txqueue--;

	if (rtl8019_device.tx1 < 0)
	{
		if (rtl8019_device.lasttx != 1 && rtl8019_device.lasttx != -1)
			RTL8019_TRACE(" bogus last_tx_buffer %d, tx1=%d.\n",rtl8019_device.lasttx, rtl8019_device.tx1);
		rtl8019_device.tx1 = 0;
		if (rtl8019_device.tx2 > 0)
		{
			rtl8019_device.txing = 1;
			NS8390_trigger_send(rtl8019_device.tx2, rtl8019_device.tx_start_page + 6);
			rtl8019_device.tx2 = -1,
			rtl8019_device.lasttx = 2;
		}
		else rtl8019_device.lasttx = 20, rtl8019_device.txing = 0;
	}
	else if (rtl8019_device.tx2 < 0)
	{
		if (rtl8019_device.lasttx != 2  &&  rtl8019_device.lasttx != -2)
			RTL8019_TRACE("bogus last_tx_buffer %d, tx2=%d.\n",rtl8019_device.lasttx, rtl8019_device.tx2);
		rtl8019_device.tx2 = 0;
		if (rtl8019_device.tx1 > 0)
		{
			rtl8019_device.txing = 1;
			NS8390_trigger_send(rtl8019_device.tx1, rtl8019_device.tx_start_page);
			rtl8019_device.tx1 = -1;
			rtl8019_device.lasttx = 1;
		}
		else
			rtl8019_device.lasttx = 10, rtl8019_device.txing = 0;
	}
//	else RTL8019_TRACE(KERN_WARNING "%s: unexpected TX-done interrupt, lasttx=%d.\n",
//			 rtl8019_device.lasttx);


	rt_sem_release(&sem_tx_done);
	RTL8019_TRACE("we release the sem_tx_done\n");
}

static void ei_tx_err()
{
	
	unsigned char txsr = inportb(e8390_base+EN0_TSR);
	unsigned char tx_was_aborted = txsr & (ENTSR_ABT+ENTSR_FU);


	RTL8019_TRACE(" transmitter error (%#2x): ",  txsr);
	if (txsr & ENTSR_ABT)
		RTL8019_TRACE("excess-collisions ");
	if (txsr & ENTSR_ND)
		RTL8019_TRACE("non-deferral ");
	if (txsr & ENTSR_CRS)
		RTL8019_TRACE("lost-carrier ");
	if (txsr & ENTSR_FU)
		RTL8019_TRACE("FIFO-underrun ");
	if (txsr & ENTSR_CDH)
		RTL8019_TRACE("lost-heartbeat ");
	RTL8019_TRACE("\n");


	outportb(ENISR_TX_ERR, e8390_base + EN0_ISR); /* Ack intr. */

	if (tx_was_aborted)
		ei_tx_intr();
	
}

/* interrupt service routine */
void rt_rtl8019_isr(int irqno)
{
	int interrupts, nr_serviced = 0;

	/*
	 *	Protect the irq test too.
	 */
	if (rtl8019_device.irqlock)
	{
		#if 1 /* This might just be an interrupt for a PCI device sharing this line */
		/* The "irqlock" check is only for testing. */
		RTL8019_TRACE("<rt_rtl8019_isr> Interrupted while interrupts are masked! isr=%#2x imr=%#2x.\n",inportb(e8390_base + EN0_ISR),inportb(e8390_base + EN0_IMR));
		#endif
			return ;
	}

	/* Change to page 0 and read the intr status reg. */
	outportb(E8390_NODMA+E8390_PAGE0, e8390_base + E8390_CMD);
	RTL8019_TRACE("<rt_rtl8019_isr> interrupt(isr=%#2.2x).\n", inportb(e8390_base + EN0_ISR));

	/* !!Assumption!! -- we stay in page 0.	 Don't break this. */
	while ((interrupts = inportb(e8390_base + EN0_ISR)) != 0 &&
	       ++nr_serviced < MAX_SERVICE)
	{
		
		if (interrupts & ENISR_OVER)
			ei_rx_overrun();
		else if (interrupts & (ENISR_RX+ENISR_RX_ERR))
		{
			/* Got a good (?) packet. */
		    //ei_receive();
			//ask lwip to receive data
			eth_device_ready(&(rtl8019_device.parent));
		}
		/* Push the next to-transmit packet through. */
		if (interrupts & ENISR_TX)
			ei_tx_intr();
		else if (interrupts & ENISR_TX_ERR)
			ei_tx_err();

		if (interrupts & ENISR_COUNTERS)
		{
			outportb(ENISR_COUNTERS, e8390_base + EN0_ISR); /* Ack intr. */
		}

		/* Ignore any RDC interrupts that make it back to here. */
		if (interrupts & ENISR_RDC)
		{
			outportb(ENISR_RDC, e8390_base + EN0_ISR);
		}

		outportb(E8390_NODMA+E8390_PAGE0+E8390_START, e8390_base + E8390_CMD);
	}

	outportb(E8390_NODMA+E8390_PAGE0+E8390_START, e8390_base + E8390_CMD);
	if (nr_serviced >= MAX_SERVICE)
	{
		/* 0xFF is valid for a card removal */
		if(interrupts!=0xFF)
			RTL8019_TRACE("<rt_rtl8019_isr> Too much work at interrupt, status %#2.2x\n",interrupts);
		outportb(ENISR_ALL, e8390_base + EN0_ISR); /* Ack. most intrs. */
	} 
	else 
	{
		RTL8019_TRACE("<rt_rtl8019_isr> unknown interrupt %#2x\n",  interrupts);
		outportb(0xff, e8390_base + EN0_ISR); /* Ack. all intrs. */
	}

}

/* RT-Thread Device Interface */
/* initialize the interface */
static rt_err_t rt_rtl8019_init(rt_device_t dev)
{
	int i;
	RTL8019_TRACE("rtl8019 init:\n");
	rtl8019_device.tx_start_page = 0x40;
	rtl8019_device.stop_page=0x80;
	rtl8019_device.rx_start_page = 0x4c;
	
	//unsigned long reset_start_time = jiffies;

	/* DON'T change these to inb_p/outb_p or reset will fail on clones. */
	outportb(inportb(e8390_base + NE_RESET), e8390_base + NE_RESET);

	while ((inportb(e8390_base + EN0_ISR) & ENISR_RESET) == 0)
	{
			RTL8019_TRACE("reseting rtl8019 ...\n");
			delay_ms(500);
	}

	outportb(0xff, e8390_base + EN0_ISR);		/* Ack all intr. */
	outportb(0x49, e8390_base + EN0_DCFG);
	
	/* Follow National Semi's recommendations for initing the DP83902. */
	outportb(E8390_NODMA+E8390_PAGE0+E8390_STOP, e8390_base+E8390_CMD); /* 0x21 */
	outportb(0x49, e8390_base + EN0_DCFG);	/* 0x48 or 0x49 */
	/* Clear the remote byte count registers. */
	outportb(0x00,  e8390_base + EN0_RCNTLO);
	outportb(0x00,  e8390_base + EN0_RCNTHI);
	/* Set to monitor and loopback mode -- this is vital!. */
	outportb(E8390_RXOFF, e8390_base + EN0_RXCR); /* 0x20 */
	outportb(E8390_TXOFF, e8390_base + EN0_TXCR); /* 0x02 */
	/* Set the transmit page and receive ring. */
	outportb(rtl8019_device.tx_start_page, e8390_base + EN0_TPSR);
	rtl8019_device.tx1 = rtl8019_device.tx2 = 0;
	outportb(rtl8019_device.rx_start_page, e8390_base + EN0_STARTPG);
	outportb(rtl8019_device.stop_page-1, e8390_base + EN0_BOUNDARY);	/* 3c503 says 0x3f,NS0x26*/
	rtl8019_device.current_page = rtl8019_device.rx_start_page;		/* assert boundary+1 */
	outportb(rtl8019_device.stop_page, e8390_base + EN0_STOPPG);
	/* Clear the pending interrupts and mask. */
	outportb(0xFF, e8390_base + EN0_ISR);
	outportb(0x00,  e8390_base + EN0_IMR);

	/* Copy the station address into the DS8390 registers. */

	outportb(E8390_NODMA + E8390_PAGE1 + E8390_STOP, e8390_base+E8390_CMD); /* 0x61 */
	for(i = 0; i < 6; i++)
	{
		outportb(rtl8019_device.dev_addr[i], e8390_base + EN1_PHYS_SHIFT(i));
		if (inportb(e8390_base + EN1_PHYS_SHIFT(i))!=rtl8019_device.dev_addr[i])
		RTL8019_TRACE("Hw. address read/write mismap %d\n",i);
	}

	outportb(rtl8019_device.rx_start_page, e8390_base + EN1_CURPAG);
	outportb(E8390_NODMA+E8390_PAGE0+E8390_STOP, e8390_base+E8390_CMD);

	rtl8019_device.tx1 = rtl8019_device.tx2 = 0;
	rtl8019_device.txing = 0;
	
	if(rtl8019_device.startp==1)
	{
		RTL8019_TRACE("rtl8019 tx startp\n");
		outportb(0xff,  e8390_base + EN0_ISR);
		outportb(ENISR_ALL,  e8390_base + EN0_IMR);
		outportb(E8390_NODMA+E8390_PAGE0+E8390_START, e8390_base+E8390_CMD);
		outportb(E8390_TXCONFIG, e8390_base + EN0_TXCR); /* xmit on. */
		/* 3c503 TechMan says rxconfig only after the NIC is started. */
		outportb(E8390_RXCONFIG, e8390_base + EN0_RXCR); /* rx on,  */
		rt_memset(rtl8019_device.mcfilter, 0xFF, 8);
		outportb(E8390_NODMA + E8390_PAGE1, e8390_base + E8390_CMD);
		for(i = 0; i < 8; i++)
		{
			outportb(rtl8019_device.mcfilter[i], e8390_base + EN1_MULT_SHIFT(i));
			if(inportb(e8390_base + EN1_MULT_SHIFT(i))!=rtl8019_device.mcfilter[i])
			RTL8019_TRACE("Multicast filter read/write mismap %d\n",i);
		}
		outportb(E8390_NODMA + E8390_PAGE0, e8390_base + E8390_CMD);
		outportb(E8390_RXCONFIG, e8390_base + EN0_RXCR);
	}
	RTL8019_TRACE("rtl8019 init done:\n");

	return RT_EOK;
}

static rt_err_t rt_rtl8019_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}

static rt_err_t rt_rtl8019_close(rt_device_t dev)
{
	return RT_EOK;
}

static rt_size_t rt_rtl8019_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
	rt_set_errno(-RT_ENOSYS);
	return 0;
}

static rt_size_t rt_rtl8019_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
	rt_set_errno(-RT_ENOSYS);
	return 0;
}

static rt_err_t rt_rtl8019_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	switch (cmd)
	{
	case NIOCTL_GADDR:
		/* get mac address */
		if (args) rt_memcpy(args, rtl8019_device.dev_addr, 6);
		else return -RT_ERROR;
		break;

	default :
		break;
	}

	return RT_EOK;
}

void INTEINT1_handler(int irqno)
{
    rt_uint32_t eint_pend;

    eint_pend = INTPND;

    /* EINT7 : RTL8019AEP */
    if( eint_pend & (1<<INT_EINT1) )
    {
        rt_rtl8019_isr(0);
    }

	/* clear EINT pending bit */
	INTPND = eint_pend;
}

void rt_hw_rtl8019_init()
{

	rt_sem_init(&sem_tx_done, "tx_ack", 1, RT_IPC_FLAG_FIFO);
	rt_sem_init(&sem_lock, "eth_lock", 1, RT_IPC_FLAG_FIFO);

	/*
	 * SRAM Tx/Rx pointer automatically return to start address,
	 * Packet Transmitted, Packet Received
	 */

	rtl8019_device.dev_addr[0] = SrcMacID[0];
	rtl8019_device.dev_addr[1] = SrcMacID[1];
	rtl8019_device.dev_addr[2] = SrcMacID[2];
	rtl8019_device.dev_addr[3] = SrcMacID[3];
	rtl8019_device.dev_addr[4] = SrcMacID[4];
	rtl8019_device.dev_addr[5] = SrcMacID[5];

	rtl8019_device.parent.parent.init	   = rt_rtl8019_init;
	rtl8019_device.parent.parent.open	   = rt_rtl8019_open;
	rtl8019_device.parent.parent.close	   = rt_rtl8019_close;
	rtl8019_device.parent.parent.read	   = rt_rtl8019_read;
	rtl8019_device.parent.parent.write	   = rt_rtl8019_write;
	rtl8019_device.parent.parent.control	= rt_rtl8019_control;
	rtl8019_device.parent.parent.user_data  = RT_NULL;

	rtl8019_device.parent.eth_rx	 = rt_rtl8019_rx;
	rtl8019_device.parent.eth_tx	 = rt_rtl8019_tx;

	eth_device_init(&(rtl8019_device.parent), "e0");
	rtl8019_device.startp=1;
	/* instal interrupt */
	rt_hw_interrupt_install(INT_EINT1, INTEINT1_handler, RT_NULL);
	rt_hw_interrupt_umask(INT_EINT1);
}


