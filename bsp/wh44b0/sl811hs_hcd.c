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

#ifdef RT_USING_USB_HOST

static struct uhcd susb_hcd;
static struct uhubinst root_hub;
static rt_bool_t ignore_disconnect = RT_FALSE;

static struct rt_semaphore sem_lock;

/**
  * @brief  USBH_Connect
  *         USB Connect callback function from the Interrupt. 
  * @param  selected device
  * @retval none
  */
rt_uint8_t susb_connect ()
{
    struct uhost_msg msg;
#if 0	
    pdev->host.ConnSts = 1;

    rt_kprintf("susb_connect\n");
    
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
rt_uint8_t susb_disconnect ()
{    
    struct uhost_msg msg;
#if 0
    pdev->host.ConnSts = 0;

    rt_kprintf("susb_disconnect\n");

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

rt_uint8_t susb_sof ()
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
static int susb_control_xfer(uinst_t uinst, ureq_t setup, void* buffer, 
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
static int susb_int_xfer(upipe_t pipe, void* buffer, int nbytes, int timeout)
{
    int size;
    
    RT_ASSERT(pipe != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);

    if(!(root_hub.port_status[0] & PORT_CCS) || 
        (root_hub.port_status[0] & PORT_CCSC)) return -1;

    rt_sem_take(&sem_lock, RT_WAITING_FOREVER);

    rt_kprintf("susb_int_xfer\n");
    
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
static int susb_bulk_xfer(upipe_t pipe, void* buffer, int nbytes, int timeout)
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
static int susb_iso_xfer(upipe_t pipe, void* buffer, int nbytes, int timeout)
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
static rt_err_t susb_alloc_pipe(upipe_t* pipe, uifinst_t ifinst, uep_desc_t ep, 
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
    RT_DEBUG_LOG(1, ("susb_alloc_pipe : %d, chanel %d, max packet size %d\n", 
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
static rt_err_t susb_free_pipe(upipe_t pipe)
{
    rt_uint8_t channel;

    RT_ASSERT(pipe != RT_NULL);
    
    RT_DEBUG_LOG(RT_DEBUG_USB, ("susb_free_pipe:%d\n", 
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
static rt_err_t susb_hub_control(rt_uint16_t port, rt_uint8_t cmd, void* args)
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

static struct uhcd_ops susb_ops = 
{
    susb_control_xfer,
    susb_bulk_xfer,
    susb_int_xfer,
    susb_iso_xfer,
    susb_alloc_pipe,
    susb_free_pipe,
    susb_hub_control,    
};

/**
 * This function will initialize susb host controller device.
 *
 * @param dev the host controller device to be initalize.
 * 
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t susb_init(rt_device_t dev)
{    
    rt_sem_init(&sem_lock, "s_lock", 1, RT_IPC_FLAG_FIFO);    

    /* roothub initilizition */
    root_hub.num_ports = 1;
    root_hub.is_roothub = RT_TRUE;
    root_hub.self = RT_NULL;
    root_hub.hcd = &susb_hcd;
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
void rt_hw_susb_init(void)
{
    susb_hcd.parent.type = RT_Device_Class_USBHost;
    susb_hcd.parent.init = susb_init;
    
    susb_hcd.ops = &susb_ops;
    
    rt_device_register(&susb_hcd.parent, "susb", 0);    
}

#endif

