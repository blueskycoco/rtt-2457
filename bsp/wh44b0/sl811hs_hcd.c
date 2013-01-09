/*
 * File      : sl811hs_hcd.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2011, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-01-08     bbstr      first version
 */

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <s3c44b0.h>
#include "board.h"
#include "sl811hs.h"
#ifdef RT_USING_USB_HOST

#define SL811_DEBUG		0
#if SL811_DEBUG
#define SL811_TRACE	rt_kprintf
#else
#define SL811_TRACE(...)
#endif
#define ADDR_BASE   0x0a000000 
#define DATA_BASE   0x0a000001
#define inportb(r) 		(*(volatile rt_uint8_t *)(r))
#define outportb(r, d) 	(*(volatile rt_uint8_t *)(d) = (r))
struct sl811
{
    struct uhcd sl811_hcd;
    struct uhubinst root_hub;
    rt_bool_t ignore_disconnect = RT_FALSE;
    rt_uint32_t port1;
    rt_uint8_t ctrl1,ctrl2,irq_enable;
};
static struct sl811 sl811_device;
static inline rt_uint8_t sl811_read(rt_uint8_t reg)
{
    outportb(reg, ADDR_BASE);
    return inportb(DATA_BASE);
}

static inline void sl811_write(rt_uint8_t reg, rt_uint8_t val)
{
    outportb(reg, ADDR_BASE);
    outportb(val, DATA_BASE);
}

static inline void
sl811_write_buf(rt_uint8_t addr, const void *buf, rt_uint16_t count)
{
    const rt_uint8_t    *data;

    if (!count)
       return;
    outportb(addr,ADDR_BASE);
    data = buf;
    do {
          outportb(*data++,DATA_BASE);                
       } while (--count);
}

static inline void
sl811_read_buf(rt_uint8_t addr, void *buf, rt_uint16_t count)
{
    rt_uint8_t      *data;
    if (!count)
       return;
    outportb(addr,ADDR_BASE);
    data = buf;
    do {
         *data++ = inportb(DATA_BASE);
       } while (--count);
}
static struct rt_semaphore sem_lock;

/**
  * @brief  USBH_Connect
  *         USB Connect callback function from the Interrupt. 
  * @param  selected device
  * @retval none
  */
rt_uint8_t sl811_connect ()
{
    struct uhost_msg msg;
#if 0	
    pdev->host.ConnSts = 1;

    rt_kprintf("sl811_connect\n");
    
    if(root_hub.port_status[0] & PORT_CCS) return 0;
    if(ignore_disconnect == RT_TRUE) return 0;

    USB_Host.Control.hc_num_out = USBH_Alloc_Channel(&USB_OTG_Core, 0x00);
    USB_Host.Control.hc_num_in = USBH_Alloc_Channel(&USB_OTG_Core, 0x80);  

    /* Open Control pipes */
    USBH_Open_Channel(&USB_OTG_Core, USB_Host.Control.hc_num_in,
        USB_Host.device_prop.address,USB_Host.device_prop.speed, EP_TYPE_CTRL,
        USB_Host.Control.ep0size); 

    /* Open Control pipes */
    USBH_Open_Channel(&USB_OTG_Core, USB_Host.Control.hc_num_out,
        USB_Host.device_prop.address, USB_Host.device_prop.speed,
        EP_TYPE_CTRL, USB_Host.Control.ep0size);   
#endif       
    root_hub.port_status[0] |= (PORT_CCS | PORT_CCSC);
    msg.type = USB_MSG_CONNECT_CHANGE;
    msg.content.uhub = &root_hub;
    rt_usb_post_event(&msg, sizeof(struct uhost_msg));    

    return 0;
}

/**
  * @brief  USBH_Disconnect
  *         USB Disconnect callback function from the Interrupt. 
  * @param  selected device
  * @retval none
  */
rt_uint8_t sl811_disconnect ()
{    
    struct uhost_msg msg;
#if 0
    pdev->host.ConnSts = 0;

    rt_kprintf("sl811_disconnect\n");

    USBH_DeInit(&USB_OTG_Core , &USB_Host);
    USBH_DeAllocate_AllChannel(&USB_OTG_Core);  
    USB_Host.gState = HOST_IDLE;
#endif
    root_hub.port_status[0] |= PORT_CCSC;
    root_hub.port_status[0] &= ~PORT_CCS;
    msg.type = USB_MSG_CONNECT_CHANGE;
    msg.content.uhub = &root_hub;
    rt_usb_post_event(&msg, sizeof(struct uhost_msg));    

    return 0;
}

rt_uint8_t sl811_sof ()
{
  /* This callback could be used to implement a scheduler process */
  return 0;  
}

/**
 * This function will do control transfer in lowlevel, it will send request to the host controller
 *
 * @param uinst the usb device instance. 
 * @param setup the buffer to save sending request packet.
 * @param buffer the data buffer to save requested data
 * @param nbytes the size of buffer
 * 
 * @return the error code, RT_EOK on successfully.
 */
static int sl811_control_xfer(uinst_t uinst, ureq_t setup, void* buffer, 
    int nbytes, int timeout)
{
    rt_uint32_t speed;

    RT_ASSERT(uinst != RT_NULL);
    RT_ASSERT(setup != RT_NULL);

    if(!(root_hub.port_status[0] & PORT_CCS) || 
        (root_hub.port_status[0] & PORT_CCSC)) return -1;

    rt_sem_take(&sem_lock, RT_WAITING_FOREVER);
#if 0
    /* Save Global State */
    USB_Host.gStateBkp = USB_Host.gState; 
    
    /* Prepare the Transactions */
    USB_Host.gState = HOST_CTRL_XFER;
    USB_Host.Control.buff = (rt_uint8_t*)buffer; 
    USB_Host.Control.length = nbytes;
    USB_Host.Control.state = CTRL_SETUP;    
    speed = HCD_GetCurrentSpeed(&USB_OTG_Core);

    rt_memcpy((void*)USB_Host.Control.setup.d8, (void*)setup, 8);
    
    USBH_Modify_Channel (&USB_OTG_Core, USB_Host.Control.hc_num_out,
        uinst->address, speed, EP_TYPE_CTRL, uinst->max_packet_size);
    USBH_Modify_Channel (&USB_OTG_Core, USB_Host.Control.hc_num_in,
        uinst->address, speed, EP_TYPE_CTRL, uinst->max_packet_size);  

    while(1)
    {
        USBH_HandleControl(&USB_OTG_Core, &USB_Host);    
        if(USB_Host.Control.state == CTRL_COMPLETE) break;
    }
#endif    
    rt_sem_release(&sem_lock);        

    return nbytes;
}

/**
 * This function will do int transfer in lowlevel, it will send request to the host controller
 *
 * @param pipe the int transfer pipe. 
 * @param buffer the data buffer to save requested data
 * @param nbytes the size of buffer
 * 
 * @return the error code, RT_EOK on successfully.
 *
 */
static int sl811_int_xfer(upipe_t pipe, void* buffer, int nbytes, int timeout)
{
    int size;
    
    RT_ASSERT(pipe != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);

    if(!(root_hub.port_status[0] & PORT_CCS) || 
        (root_hub.port_status[0] & PORT_CCSC)) return -1;

    rt_sem_take(&sem_lock, RT_WAITING_FOREVER);

    rt_kprintf("sl811_int_xfer\n");
    
    rt_sem_release(&sem_lock);        

    return size;
}

/**
 * This function will do bulk transfer in lowlevel, it will send request to the host controller
 *
 * @param pipe the bulk transfer pipe. 
 * @param buffer the data buffer to save requested data
 * @param nbytes the size of buffer
 * 
 * @return the error code, RT_EOK on successfully.
 */
static int sl811_bulk_xfer(upipe_t pipe, void* buffer, int nbytes, int timeout)
{    
    rt_uint8_t channel;
    int left = nbytes;
    rt_uint8_t *ptr;
//    URB_STATE state;

    RT_ASSERT(pipe != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);

    if(!(root_hub.port_status[0] & PORT_CCS) || 
        (root_hub.port_status[0] & PORT_CCSC)) return -1;

    ptr = (rt_uint8_t*)buffer;
    channel = (rt_uint32_t)pipe->user_data & 0xFF;
    
    rt_sem_take(&sem_lock, RT_WAITING_FOREVER);
#if 0    
    if(pipe->ep.bEndpointAddress & USB_DIR_IN)
    {    
        while(left > pipe->ep.wMaxPacketSize)
        {
            USBH_BulkReceiveData(&USB_OTG_Core, ptr, pipe->ep.wMaxPacketSize, 
                channel);
            while(1)
            {
                state = HCD_GetURB_State(&USB_OTG_Core , channel);
                if(state == URB_DONE) break;
                else if(state == URB_NOTREADY) rt_kprintf("not ready\n");
                else if(state == URB_STALL) rt_kprintf("stall\n");
                //else if(state == URB_IDLE) rt_kprintf("idle\n");
            }
            
            ptr += pipe->ep.wMaxPacketSize;
            left -= pipe->ep.wMaxPacketSize;
        }

        USBH_BulkReceiveData(&USB_OTG_Core, ptr, left, channel);
        while(1)
        {
            state = HCD_GetURB_State(&USB_OTG_Core , channel);
            if(state == URB_DONE) break;
            else if(state == URB_NOTREADY) rt_kprintf("not ready\n");
            else if(state == URB_STALL) rt_kprintf("stall\n");
            //else if(state == URB_IDLE) rt_kprintf("idle\n");            
        }
    }    
    else
    {    
send_data:
        while(left > pipe->ep.wMaxPacketSize)
        {
            USBH_BulkSendData(&USB_OTG_Core, ptr, pipe->ep.wMaxPacketSize, 
                channel);

            while(1)
            {
                state = HCD_GetURB_State(&USB_OTG_Core, channel);
                if(state == URB_DONE) break;
                if(state == URB_NOTREADY) goto send_data;
            }

            ptr += pipe->ep.wMaxPacketSize;
            left -= pipe->ep.wMaxPacketSize;        
        }        

        USBH_BulkSendData(&USB_OTG_Core, ptr, left, channel);    
        while(1)
        {
            state = HCD_GetURB_State(&USB_OTG_Core , channel);
            if(state == URB_DONE) break;
            if(state == URB_NOTREADY) goto send_data;
        }        
    }
#endif
    rt_sem_release(&sem_lock);
    return nbytes;
}

/**
 * This function will do isochronous transfer in lowlevel, it will send request to the host controller
 *
 * @param pipe the isochronous transfer pipe.  
 * @param buffer the data buffer to save requested data
 * @param nbytes the size of buffer
 * 
 * @return the error code, RT_EOK on successfully.
 *
 * @note unimplement yet
 */
static int sl811_iso_xfer(upipe_t pipe, void* buffer, int nbytes, int timeout)
{
    /* no implement */
    RT_ASSERT(0);
    
    return 0;
}

/**
 * This function will allocate a pipe for specified endpoint, it will be used to do transfer.
 *
 * @param pipe the pointer of pipe handle to be allocated.
 * @param ifinst the usb interface instance.
 * @param ep the endpoint descriptor.
 * @param func_callback callback function to be registed 
 * 
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t sl811_alloc_pipe(upipe_t* pipe, uifinst_t ifinst, uep_desc_t ep, 
    func_callback callback)
{
    rt_uint32_t channel, speed;
    rt_uint8_t ep_type;
    upipe_t p;

    RT_ASSERT(ep != RT_NULL);

    p = (upipe_t)rt_malloc(sizeof(struct upipe));
    p->ifinst = ifinst;
    p->callback = callback;
    p->status = UPIPE_STATUS_OK;
    rt_memcpy(&p->ep, ep, ep->bLength);
#if 0
    speed = HCD_GetCurrentSpeed(&USB_OTG_Core);
    channel = USBH_Alloc_Channel(&USB_OTG_Core, p->ep.bEndpointAddress);

    if((ep->bmAttributes & USB_EP_ATTR_TYPE_MASK) == USB_EP_ATTR_BULK)
    ep_type = EP_TYPE_BULK;
    else if((ep->bmAttributes & USB_EP_ATTR_TYPE_MASK) == USB_EP_ATTR_INT)
    ep_type = EP_TYPE_INTR;
    else rt_kprintf("unsupported endpoint type\n");
        
    /* Open the new channels */
    USBH_Open_Channel(&USB_OTG_Core, channel, ifinst->uinst->address, 
        speed, ep_type, p->ep.wMaxPacketSize);
#endif
    RT_DEBUG_LOG(1, ("sl811_alloc_pipe : %d, chanel %d, max packet size %d\n", 
        p->ep.bEndpointAddress, channel, p->ep.wMaxPacketSize));
    
    p->user_data = (void*)channel;
    *pipe = p;
    
     return RT_EOK;
}

/**
 * This function will free a pipe, it will release all resouces of the pipe.
 *
 * @param pipe the pipe handler to be free.
 * 
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t sl811_free_pipe(upipe_t pipe)
{
    rt_uint8_t channel;

    RT_ASSERT(pipe != RT_NULL);
    
    RT_DEBUG_LOG(RT_DEBUG_USB, ("sl811_free_pipe:%d\n", 
        pipe->ep.bEndpointAddress));

    channel = (rt_uint32_t)pipe->user_data & 0xFF;    
    //USBH_Free_Channel(&USB_OTG_Core, channel);

    rt_free(pipe);

    return RT_EOK;
}

/**
 * This function will control the roothub of susb host controller.
 *
 * @param port the port to be reset.
 * 
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t sl811_hub_control(rt_uint16_t port, rt_uint8_t cmd, void* args)
{
    RT_ASSERT(port == 1);
    
    switch(cmd)
    {
    case RH_GET_PORT_STATUS:
        *(rt_uint32_t*)args = root_hub.port_status[port - 1];
        break;
    case RH_SET_PORT_STATUS:
        break;
    case RH_CLEAR_PORT_FEATURE:
        switch((rt_uint32_t)args & 0xFF)
        {
        case PORT_FEAT_C_RESET:
            root_hub.port_status[port - 1] &= ~PORT_PRSC;    
            ignore_disconnect = RT_FALSE;            
            break;
        case PORT_FEAT_C_CONNECTION:
            root_hub.port_status[port - 1] &= ~PORT_CCSC;            
            break;
        case PORT_FEAT_C_ENABLE:
            root_hub.port_status[port - 1] &= ~PORT_PESC;            
            break;
        default:
            break;
        }
        break;
    case RH_SET_PORT_FEATURE:
        switch((rt_uint32_t)args & 0xFF)
        {        
        case PORT_FEAT_POWER:
            root_hub.port_status[port - 1] |= PORT_PPS;
            break;
        case PORT_FEAT_RESET:            
            ignore_disconnect = RT_TRUE;            
            root_hub.port_status[port - 1] |= PORT_PRS;    
      //      USB_OTG_ResetPort(&USB_OTG_Core);             
            root_hub.port_status[port - 1] &= ~PORT_PRS;  
            break;
        case PORT_FEAT_ENABLE:
            root_hub.port_status[port - 1] |= PORT_PES;            
            break;
        }    
        break;
    default:
        break;
    }    

    return RT_EOK;
}

static struct uhcd_ops sl811_ops = 
{
    sl811_control_xfer,
    sl811_bulk_xfer,
    sl811_int_xfer,
    sl811_iso_xfer,
    sl811_alloc_pipe,
    sl811_free_pipe,
    sl811_hub_control,    
};
void sl811_isr(int arg)
{
	//struct sl811	*sl811 = hcd_to_sl811(hcd);
	rt_uint8_t		irqstat;
	irqreturn_t ret = IRQ_NONE;
	unsigned	retries = 5;

	//spin_lock(&sl811->lock);

retry:
	irqstat = sl811_read(SL11H_IRQ_STATUS) & ~SL11H_INTMASK_DP;
	if (irqstat) {
		sl811_write(SL11H_IRQ_STATUS, irqstat);
		irqstat &= sl811_device.irq_enable;
	}

	/* USB packets, not necessarily handled in the order they're
	 * issued ... that's fine if they're different endpoints.
	 */
	if (irqstat & SL11H_INTMASK_DONE_A) {
		done(sl811, sl811->active_a, SL811_EP_A(SL811_HOST_BUF));
		sl811->active_a = NULL;
		sl811->stat_a++;
	}
	if (irqstat & SL11H_INTMASK_SOFINTR) {
		unsigned index;

		index = sl811->frame++ % (PERIODIC_SIZE - 1);
		sl811->stat_sof++;

		/* be graceful about almost-inevitable periodic schedule
		 * overruns:  continue the previous frame's transfers iff
		 * this one has nothing scheduled.
		 */
		if (sl811->next_periodic) {
			// ERR("overrun to slot %d\n", index);
			sl811->stat_overrun++;
		}
		if (sl811->periodic[index])
			sl811->next_periodic = sl811->periodic[index];
	}

	/* khubd manages debouncing and wakeup */
	if (irqstat & SL11H_INTMASK_INSRMV) {
		sl811->stat_insrmv++;

		/* most stats are reset for each VBUS session */
		sl811->stat_wake = 0;
		sl811->stat_sof = 0;
		sl811->stat_a = 0;
		sl811->stat_b = 0;
		sl811->stat_lost = 0;

		sl811->ctrl1 = 0;
		sl811_write(sl811, SL11H_CTLREG1, sl811->ctrl1);

		sl811->irq_enable = SL11H_INTMASK_INSRMV;
		sl811_write(sl811, SL11H_IRQ_ENABLE, sl811->irq_enable);

		/* usbcore nukes other pending transactions on disconnect */
		if (sl811->active_a) {
			sl811_write(sl811, SL811_EP_A(SL11H_HOSTCTLREG), 0);
			finish_request(sl811, sl811->active_a,
				container_of(sl811->active_a
						->hep->urb_list.next,
					struct urb, urb_list),
				-ESHUTDOWN);
			sl811->active_a = NULL;
		}

		/* port status seems weird until after reset, so
		 * force the reset and make khubd clean up later.
		 */
		if (irqstat & SL11H_INTMASK_RD)
			sl811->port1 &= ~USB_PORT_STAT_CONNECTION;
		else
			sl811->port1 |= USB_PORT_STAT_CONNECTION;

		sl811->port1 |= USB_PORT_STAT_C_CONNECTION << 16;

	} else if (irqstat & SL11H_INTMASK_RD) {
		if (sl811->port1 & USB_PORT_STAT_SUSPEND) {
			DBG("wakeup\n");
			sl811->port1 |= USB_PORT_STAT_C_SUSPEND << 16;
			sl811->stat_wake++;
		} else
			irqstat &= ~SL11H_INTMASK_RD;
	}

	if (irqstat) {
		if (sl811->port1 & USB_PORT_STAT_ENABLE)
			start_transfer(sl811);
		ret = IRQ_HANDLED;
		if (retries--)
			goto retry;
	}

	if (sl811->periodic_count == 0 && list_empty(&sl811->async))
		sofirq_off(sl811);
	sl811_write(sl811, SL11H_IRQ_ENABLE, sl811->irq_enable);

	spin_unlock(&sl811->lock);

	return ret;
}
void INTEINT4_handler(int irqno)
{
    sl811_isr(0);
}
/**
 * This function will initialize susb host controller device.
 *
 * @param dev the host controller device to be initalize.
 * 
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t sl811_init(rt_device_t dev)
{    
    rt_sem_init(&sem_lock, "s_lock", 1, RT_IPC_FLAG_FIFO);    

    /* roothub initilizition */
    root_hub.num_ports = 1;
    root_hub.is_roothub = RT_TRUE;
    root_hub.self = RT_NULL;
    root_hub.hcd = &(sl811_device.sl811_hcd);
#if 0
    /* Hardware Init */
    USB_OTG_HS_Init(&USB_OTG_Core);  
    
    /* configure GPIO pin used for switching VBUS power */
    USB_OTG_BSP_ConfigVBUS(0);    
    
    /* Host de-initializations */
    USBH_DeInit(&USB_OTG_Core, &USB_Host);
    
    /* Start the USB OTG core */     
    HCD_Init(&USB_OTG_Core , USB_OTG_HS_CORE_ID);

    USBH_DeAllocate_AllChannel(&USB_OTG_Core);  
          
    /* Enable Interrupts */
    USB_OTG_HS_EnableInterrupt(&USB_OTG_Core);    
#endif
    return RT_EOK;
}

/**
 * This function will define the susb host controller device, it will be register to the device
 * system.
 * 
 * @return the error code, RT_EOK on successfully.
 */
void rt_hw_sl811_init(void)
{
    sl811_device.sl811_hcd.parent.type = RT_Device_Class_USBHost;
    sl811_device.sl811_hcd.parent.init = sl811_init;
    
    sl811_device.sl811_hcd.ops = &sl811_ops;
    
    rt_device_register(&(sl811_device.sl811_hcd.parent), "sl811", 0);    
}

#endif

