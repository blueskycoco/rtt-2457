/*
 * File      : cdc.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2012, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-10-03     Yi Qiu      first version
 */

#define USB_CDC_PRODUCT_ID              0x5740   /* Product ID */
#define USB_CDC_BUFSIZE                 0x40

#define USB_CDC_CLASS_COMM              0x02
#define USB_CDC_CLASS_DATA              0x0A

#define USB_CDC_SUBCLASS_DLCM           0x01
#define USB_CDC_SUBCLASS_ACM            0x02
#define USB_CDC_SUBCLASS_TCM            0x03
#define USB_CDC_SUBCLASS_MCCM           0x04
#define USB_CDC_SUBCLASS_CCM            0x05
#define USB_CDC_SUBCLASS_ETH            0x06
#define USB_CDC_SUBCLASS_ATM            0x07

#define USB_CDC_PROTOCOL_V25TER         0x01

#define USB_CDC_PROTOCOL_I430           0x30
#define USB_CDC_PROTOCOL_HDLC           0x31
#define USB_CDC_PROTOCOL_TRANS          0x32
#define USB_CDC_PROTOCOL_Q921M          0x50
#define USB_CDC_PROTOCOL_Q921           0x51
#define USB_CDC_PROTOCOL_Q921TM         0x52
#define USB_CDC_PROTOCOL_V42BIS         0x90
#define USB_CDC_PROTOCOL_Q931           0x91
#define USB_CDC_PROTOCOL_V120           0x92
#define USB_CDC_PROTOCOL_CAPI20         0x93
#define USB_CDC_PROTOCOL_HOST           0xFD
#define USB_CDC_PROTOCOL_PUFD           0xFE

#define USB_CDC_CS_INTERFACE            0x24
#define USB_CDC_CS_ENDPOINT             0x25

#define USB_CDC_SCS_HEADER              0x00
#define USB_CDC_SCS_CALL_MGMT           0x01
#define USB_CDC_SCS_ACM                 0x02
#define USB_CDC_SCS_UNION               0x06

#define CDC_SEND_ENCAPSULATED_COMMAND   0x00
#define CDC_GET_ENCAPSULATED_RESPONSE   0x01
#define CDC_SET_COMM_FEATURE            0x02
#define CDC_GET_COMM_FEATURE            0x03
#define CDC_CLEAR_COMM_FEATURE          0x04
#define CDC_SET_AUX_LINE_STATE          0x10
#define CDC_SET_HOOK_STATE              0x11
#define CDC_PULSE_SETUP                 0x12
#define CDC_SEND_PULSE                  0x13
#define CDC_SET_PULSE_TIME              0x14
#define CDC_RING_AUX_JACK               0x15
#define CDC_SET_LINE_CODING             0x20
#define CDC_GET_LINE_CODING             0x21
#define CDC_SET_CONTROL_LINE_STATE      0x22
#define CDC_SEND_BREAK                  0x23
#define CDC_SET_RINGER_PARMS            0x30
#define CDC_GET_RINGER_PARMS            0x31
#define CDC_SET_OPERATION_PARMS         0x32
#define CDC_GET_OPERATION_PARMS         0x33
#define CDC_SET_LINE_PARMS              0x34
#define CDC_GET_LINE_PARMS              0x35
#define CDC_DIAL_DIGITS                 0x36
#define CDC_SET_UNIT_PARAMETER          0x37
#define CDC_GET_UNIT_PARAMETER          0x38
#define CDC_CLEAR_UNIT_PARAMETER        0x39
#define CDC_GET_PROFILE                 0x3A
#define CDC_SET_ETH_MULTICAST_FILTERS   0x40
#define CDC_SET_ETH_POWER_MGMT_FILT     0x41
#define CDC_GET_ETH_POWER_MGMT_FILT     0x42
#define CDC_SET_ETH_PACKET_FILTER       0x43
#define CDC_GET_ETH_STATISTIC           0x44
#define CDC_SET_ATM_DATA_FORMAT         0x50
#define CDC_GET_ATM_DEVICE_STATISTICS   0x51
#define CDC_SET_ATM_DEFAULT_VC          0x52
#define CDC_GET_ATM_VC_STATISTICS       0x53

#pragma pack(1)

struct ucdc_header_descriptor
{
    rt_uint8_t length;
    rt_uint8_t type;
    rt_uint8_t subtype;
    rt_uint16_t bcd;        
};
typedef struct ucdc_header_descriptor* ucdc_hdr_desc_t;

struct ucdc_acm_descriptor
{
    rt_uint8_t length;
    rt_uint8_t type;
    rt_uint8_t subtype;
    rt_uint8_t capabilties;
};
typedef struct ucdc_acm_descriptor* ucdc_acm_desc_t;

struct ucdc_call_mgmt_descriptor
{
    rt_uint8_t length;
    rt_uint8_t type;
    rt_uint8_t subtype;
    rt_uint8_t capabilties;    
    rt_uint8_t data_interface;
};
typedef struct ucdc_call_mgmt_descriptor* ucdc_call_mgmt_desc_t;

struct ucdc_union_descriptor
{
    rt_uint8_t length;
    rt_uint8_t type;
    rt_uint8_t subtype;
    rt_uint8_t master_interface;
    rt_uint8_t slave_interface0;
};
typedef struct ucdc_union_descriptor* ucdc_union_desc_t;

struct ucdc_comm_descriptor
{
    struct uinterface_descriptor intf_desc;
    struct ucdc_header_descriptor hdr_desc;
    struct ucdc_call_mgmt_descriptor call_mgmt_desc;    
    struct ucdc_acm_descriptor acm_desc;    
    struct ucdc_union_descriptor union_desc;    
    struct uendpoint_descriptor ep_desc;
};
typedef struct ucdc_comm_descriptor* ucdc_comm_desc_t;

struct ucdc_data_descriptor
{
    struct uinterface_descriptor intf_desc;
    struct uendpoint_descriptor ep_out_desc;        
    struct uendpoint_descriptor ep_in_desc;
};
typedef struct ucdc_data_descriptor* ucdc_data_desc_t;

struct ucdc_line_coding
{
    rt_uint32_t dwDTERate;
    rt_uint8_t bCharFormat;
    rt_uint8_t bParityType;
    rt_uint8_t bDataBits;
};
typedef struct ucdc_line_coding* ucdc_line_coding_t;

#pragma pack()

