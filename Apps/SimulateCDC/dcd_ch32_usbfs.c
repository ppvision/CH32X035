/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Greg Davill
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED && ((CFG_TUSB_MCU == OPT_MCU_CH32V307) || (CFG_TUSB_MCU == OPT_MCU_CH32X035))
#include "device/dcd.h"
#include "ch32x035_usb.h"
#include "ch32_usbfs_regs.h"


// Max number of bi-directional endpoints including EP0

// typedef struct {
//     uint8_t *buffer;
//     // tu_fifo_t * ff; // TODO support dcd_edpt_xfer_fifo API
//     uint16_t total_len;
//     uint16_t queued_len;
//     uint16_t max_size;
//     bool short_packet;
// } ch32_usbfs_ep_state;

// #define XFER_CTL_BASE(_ep, _dir) &xfer_status[_ep][_dir]
// static ch32_usbfs_ep_state xfer_status[EP_MAX][2];

#define EP_TX_LEN(ep)  *(volatile uint16_t *)((volatile uint16_t *)&(USBHSD->UEP0_TX_LEN) + (ep)*2)
#define EP_TX_CTRL(ep) *(volatile uint8_t *)((volatile uint8_t *)&(USBHSD->UEP0_TX_CTRL) + (ep)*4)
#define EP_RX_CTRL(ep) *(volatile uint8_t *)((volatile uint8_t *)&(USBHSD->UEP0_RX_CTRL) + (ep)*4)
#define EP_RX_MAX_LEN(ep) *(volatile uint16_t *)((volatile uint16_t *)&(USBHSD->UEP0_MAX_LEN) + (ep)*2)

#define EP_TX_DMA_ADDR(ep) *(volatile uint32_t *)((volatile uint32_t *)&(USBHSD->UEP1_TX_DMA) + (ep - 1))
#define EP_RX_DMA_ADDR(ep) *(volatile uint32_t *)((volatile uint32_t *)&(USBHSD->UEP1_RX_DMA) + (ep - 1))

/* Endpoint Buffer */
#ifndef   MIN
#define MIN(a, b)                                       (((a) < (b)) ? (a) : (b))
#endif

TU_ATTR_ALIGNED(4) uint8_t EP0_DatabufHD[64];  // ep0(64)

volatile uint8_t USBHS_Dev_Endp0_Tog = 0x01;







// ********************************************************************************************

#ifndef USBD_IRQHandler
#define USBD_IRQHandler OTG_FS_IRQHandler //use actual usb irq name instead
#endif

#ifndef USB_NUM_BIDIR_ENDPOINTS
#define USB_NUM_BIDIR_ENDPOINTS 8
#endif



#define USB_IOEN                    0x00000080
#define USB_PHY_V33                 0x00000040
#define UDP_PUE_MASK                0x0000000C
#define UDP_PUE_DISABLE             0x00000000
#define UDP_PUE_35UA                0x00000004
#define UDP_PUE_10K                 0x00000008
#define UDP_PUE_1K5                 0x0000000C

#define UDM_PUE_MASK                0x00000003
#define UDM_PUE_DISABLE             0x00000000
#define UDM_PUE_35UA                0x00000001
#define UDM_PUE_10K                 0x00000002
#define UDM_PUE_1K5                 0x00000003


#define USBFSD_UEP_MOD_BASE         0x4002340C
#define USBFSD_UEP_DMA_BASE         0x40023410
#define USBFSD_UEP_LEN_BASE         0x40023420
#define USBFSD_UEP_CTL_BASE         0x40023422
#define USBFSD_UEP_RX_EN            0x08
#define USBFSD_UEP_TX_EN            0x04
#define USBFSD_UEP_BUF_MOD          0x01
#define DEF_UEP_DMA_LOAD            0 /* Direct the DMA address to the data to be processed */
#define DEF_UEP_CPY_LOAD            1 /* Use memcpy to move data to a buffer */

#define USBFSD_UEP_MOD( N )         (*((volatile uint8_t *)( USBFSD_UEP_MOD_BASE + N )))
#define USBFSD_UEP_TX_CTRL( N )     (*((volatile uint8_t *)( USBFSD_UEP_CTL_BASE + N * 0x04 )))
#define USBFSD_UEP_RX_CTRL( N )     (*((volatile uint8_t *)( USBFSD_UEP_CTL_BASE + N * 0x04 )))
#define USBFSD_UEP_DMA( N )         (*((volatile uint32_t *)( USBFSD_UEP_DMA_BASE + N * 0x04 )))
#define USBFSD_UEP_BUF( N )         ((uint8_t *)(*((volatile uint32_t *)( USBFSD_UEP_DMA_BASE + N * 0x04 ))) + 0x20000000)
#define USBFSD_UEP_TLEN( N )        (*((volatile uint16_t *)( USBFSD_UEP_LEN_BASE + N * 0x04 )))


#define USB_SET_DMA(ep_idx, addr)    (*(volatile uint32_t *)((uint32_t)(&USBFS_DEVICE->UEP0_DMA)  + 4 * ep_idx) = addr)


/* Endpoint state */
typedef struct  {
    uint16_t ep_mps;    /* Endpoint max packet size */
    uint8_t ep_type;    /* Endpoint type */
    uint8_t ep_stalled; /* Endpoint stall flag */
    uint8_t ep_enable;  /* Endpoint enable */
    uint8_t *xfer_buf;
    uint32_t xfer_len;
    uint32_t actual_xfer_len;
    bool short_packet;
}ch32_usbfs_ep_state ;





uint8_t usb_get_ep_ctr(int ep_idx,int off) {
    if(ep_idx >EP_MAX) return 0;
    volatile uint8_t * pCtrl = NULL;
    switch (ep_idx) {
        case 0:pCtrl = (volatile uint8_t *)&USBFS_DEVICE->UEP0_CTRL_H;break;
        case 1:pCtrl = (volatile uint8_t *)&USBFS_DEVICE->UEP1_CTRL_H;break;
        case 2:pCtrl = (volatile uint8_t *)&USBFS_DEVICE->UEP2_CTRL_H;break;
        case 3:pCtrl = (volatile uint8_t *)&USBFS_DEVICE->UEP3_CTRL_H;break;
        case 4:pCtrl = (volatile uint8_t *)&USBFS_DEVICE->UEP4_CTRL_H;break;
        case 5:pCtrl = (volatile uint8_t *)&USBFS_DEVICE->UEP5_CTRL_H;break;
        case 6:pCtrl = (volatile uint8_t *)&USBFS_DEVICE->UEP6_CTRL_H;break;
        case 7:pCtrl = (volatile uint8_t *)&USBFS_DEVICE->UEP7_CTRL_H;break;
    }
    if(pCtrl) return pCtrl[off?1:0];
    return 0;
}

void usb_set_ep_ctr(int ep_idx,uint8_t val,int off) {
    if(ep_idx >EP_MAX) return ;
    volatile uint8_t * pCtrl = NULL;
    switch (ep_idx) {
        case 0:pCtrl = (volatile uint8_t *)&USBFS_DEVICE->UEP0_CTRL_H;break;
        case 1:pCtrl = (volatile uint8_t *)&USBFS_DEVICE->UEP1_CTRL_H;break;
        case 2:pCtrl = (volatile uint8_t *)&USBFS_DEVICE->UEP2_CTRL_H;break;
        case 3:pCtrl = (volatile uint8_t *)&USBFS_DEVICE->UEP3_CTRL_H;break;
        case 4:pCtrl = (volatile uint8_t *)&USBFS_DEVICE->UEP4_CTRL_H;break;
        case 5:pCtrl = (volatile uint8_t *)&USBFS_DEVICE->UEP5_CTRL_H;break;
        case 6:pCtrl = (volatile uint8_t *)&USBFS_DEVICE->UEP6_CTRL_H;break;
        case 7:pCtrl = (volatile uint8_t *)&USBFS_DEVICE->UEP7_CTRL_H;break;
    }
    if(pCtrl)  pCtrl[off?1:0] = val;
}

void usb_set_tx_ctr(int ep_idx,uint8_t val) {
    usb_set_ep_ctr(ep_idx,val,0);
}

void usb_set_rx_ctr(int ep_idx,uint8_t val) {
    usb_set_ep_ctr(ep_idx,val,0);
}

uint8_t usb_get_tx_ctr(int ep_idx) {
    return usb_get_ep_ctr(ep_idx,0);
}
uint8_t usb_get_rx_ctr(int ep_idx) {
    return usb_get_ep_ctr(ep_idx,0);
}


uint16_t usb_oper_tx_len(int ep_idx,int oper ,uint16_t val) {
    if(ep_idx >EP_MAX) return 0;
    volatile uint16_t * pCtrl = NULL;
    switch (ep_idx) {
        case 0:pCtrl = (volatile uint16_t *)&USBFS_DEVICE->UEP0_TX_LEN;break;
        case 1:pCtrl = (volatile uint16_t *)&USBFS_DEVICE->UEP1_TX_LEN;break;
        case 2:pCtrl = (volatile uint16_t *)&USBFS_DEVICE->UEP2_TX_LEN;break;
        case 3:pCtrl = (volatile uint16_t *)&USBFS_DEVICE->UEP3_TX_LEN;break;
        case 4:pCtrl = (volatile uint16_t *)&USBFS_DEVICE->UEP4_TX_LEN;break;
        case 5:pCtrl = (volatile uint16_t *)&USBFS_DEVICE->UEP5_TX_LEN;break;
        case 6:pCtrl = (volatile uint16_t *)&USBFS_DEVICE->UEP6_TX_LEN;break;
        case 7:pCtrl = (volatile uint16_t *)&USBFS_DEVICE->UEP7_TX_LEN;break;
    }
    if(!pCtrl) return 0;
    if(oper == 0){
         return (* pCtrl);
    }else{
        *pCtrl = val;return (* pCtrl);
    }
    return 0;
}

uint16_t usb_get_tx_len(int ep_idx){
    return usb_oper_tx_len(ep_idx,0,0);
}
void usb_set_tx_len(int ep_idx,uint16_t val){
    usb_oper_tx_len(ep_idx,1,val);
}

#define USB_GET_TX_LEN(ep_idx)     usb_get_tx_len(ep_idx)
#define USB_SET_TX_LEN(ep_idx,len) usb_set_tx_len(ep_idx,len)

#define USB_SET_TX_CTRL(ep_idx, val) usb_set_tx_ctr(ep_idx, val)
#define USB_GET_TX_CTRL(ep_idx)      usb_get_tx_ctr(ep_idx)

#define USB_SET_RX_CTRL(ep_idx, val) usb_set_rx_ctr(ep_idx, val)
#define USB_GET_RX_CTRL(ep_idx)      usb_get_rx_ctr(ep_idx)

/* Setup packet definition used to read raw data from USB line */
struct usb_setup_packet {
    /** Request type. Bits 0:4 determine recipient, see
	 * \ref usb_request_recipient. Bits 5:6 determine type, see
	 * \ref usb_request_type. Bit 7 determines data transfer direction, see
	 * \ref usb_endpoint_direction.
	 */
    uint8_t bmRequestType;

    /** Request. If the type bits of bmRequestType are equal to
	 * \ref usb_request_type::LIBUSB_REQUEST_TYPE_STANDARD
	 * "USB_REQUEST_TYPE_STANDARD" then this field refers to
	 * \ref usb_standard_request. For other cases, use of this field is
	 * application-specific. */
    uint8_t bRequest;

    /** Value. Varies according to request */
    uint16_t wValue;

    /** Index. Varies according to request, typically used to pass an index
	 * or offset */
    uint16_t wIndex;

    /** Number of bytes to transfer */
    uint16_t wLength;
}  __attribute__((packed));


/* Driver state */
struct ch32_usbfs_udc {
    __attribute__((aligned(4))) struct usb_setup_packet setup;
    volatile uint8_t dev_addr;
    ch32_usbfs_ep_state in_ep[USB_NUM_BIDIR_ENDPOINTS];                            /*!< IN endpoint parameters*/
    ch32_usbfs_ep_state out_ep[USB_NUM_BIDIR_ENDPOINTS];                           /*!< OUT endpoint parameters */
    __attribute__((aligned(4))) uint8_t ep_databuf[USB_NUM_BIDIR_ENDPOINTS - 1][64 + 64]; //epx_out(64)+epx_in(64)
} g_ch32_usbfs_udc;

//TUSB_DIR_OUT == 0
#define XFER_CTL_BASE(_ep, _dir) (_dir)?&g_ch32_usbfs_udc.in_ep[_ep]:&g_ch32_usbfs_udc.out_ep[_ep]



volatile bool ep0_rx_data_toggle;
volatile bool ep0_tx_data_toggle;

void USBD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USBFS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));


// ********************************************************************************************













void GPIO_USB_INIT(void);
void USBFS_Device_InitEndpoints( void ){
    USBFS_DEVICE->BASE_CTRL = 0x00;
    USBFS_DEVICE->UEP4_1_MOD = USBFS_UEP4_RX_EN | USBFS_UEP4_TX_EN | USBFS_UEP1_RX_EN | USBFS_UEP1_TX_EN;
    USBFS_DEVICE->UEP2_3_MOD = USBFS_UEP2_RX_EN | USBFS_UEP2_TX_EN | USBFS_UEP3_RX_EN | USBFS_UEP3_TX_EN;
    USBFS_DEVICE->UEP567_MOD = USBFS_UEP5_RX_EN | USBFS_UEP5_TX_EN | USBFS_UEP6_RX_EN | USBFS_UEP6_TX_EN|USBFS_UEP7_RX_EN|USBFS_UEP7_TX_EN;

    USBFS_DEVICE->UEP0_DMA = (uint32_t)g_ch32_usbfs_udc.ep_databuf[0];
    USBFS_DEVICE->UEP1_DMA = (uint32_t)g_ch32_usbfs_udc.ep_databuf[1];
    USBFS_DEVICE->UEP2_DMA = (uint32_t)g_ch32_usbfs_udc.ep_databuf[2];
    USBFS_DEVICE->UEP3_DMA = (uint32_t)g_ch32_usbfs_udc.ep_databuf[3];

    USBFS_DEVICE->UEP5_DMA = (uint32_t)g_ch32_usbfs_udc.ep_databuf[5];
    USBFS_DEVICE->UEP6_DMA = (uint32_t)g_ch32_usbfs_udc.ep_databuf[6];
    USBFS_DEVICE->UEP7_DMA = (uint32_t)g_ch32_usbfs_udc.ep_databuf[7];

    USBFS_DEVICE->INT_FG = 0xFF;
    USBFS_DEVICE->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
    USBFS_DEVICE->DEV_ADDR = 0x00;

    USBFS_DEVICE->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
    USBFS_DEVICE->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
}


void dcd_init(uint8_t rhport) {
    (void)rhport;
    memset(&g_ch32_usbfs_udc, 0, sizeof(g_ch32_usbfs_udc));

    GPIO_USB_INIT();
    AFIO->CTLR = (AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK )) | USB_PHY_V33 | UDP_PUE_1K5 | USB_IOEN;

    USBFS_Device_InitEndpoints();

}

void dcd_int_enable(uint8_t rhport) {
    (void)rhport;
    NVIC_EnableIRQ(USBFS_IRQn);
}

void dcd_int_disable(uint8_t rhport) {
    (void)rhport;
    NVIC_DisableIRQ(USBFS_IRQn);
}

void dcd_edpt_close_all(uint8_t rhport) {
    (void)rhport;
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr) {
    (void)dev_addr;

    if (dev_addr == 0) {
        USBFS_DEVICE->DEV_ADDR = (USBFS_DEVICE->DEV_ADDR & USBFS_UDA_GP_BIT) | 0;
    }
    g_ch32_usbfs_udc.dev_addr = dev_addr;
    // Response with zlp status
    dcd_edpt_xfer(rhport, 0x80, NULL, 0);
}

void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;
}


void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const *request) {
    (void)rhport;

    if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE &&
        request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
        request->bRequest == TUSB_REQ_SET_ADDRESS) {
        USBFS_DEVICE->DEV_ADDR = (uint8_t)request->wValue;
    }
    USB_SET_TX_CTRL(0,USBFS_UEP_T_TOG|USBFS_UEP_T_RES_NAK);
    USB_SET_RX_CTRL(0,USBFS_UEP_R_TOG|USBFS_UEP_R_RES_ACK);
}

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const *desc_edpt) {
    (void)rhport;

    uint8_t const epnum = tu_edpt_number(desc_edpt->bEndpointAddress);
    uint8_t const dir = tu_edpt_dir(desc_edpt->bEndpointAddress);

    TU_ASSERT(epnum < EP_MAX);

    uint16_t max_size = tu_edpt_packet_size(desc_edpt);
    // ch32_usbfs_ep_state *xfer = XFER_CTL_BASE(epnum, dir);

    if (epnum != 0) {
        if (dir == TUSB_DIR_OUT) {
            g_ch32_usbfs_udc.out_ep[epnum].ep_mps  = max_size;
            g_ch32_usbfs_udc.out_ep[epnum].ep_enable = true;
            USB_SET_RX_CTRL(epnum,USBFS_UEP_R_RES_NAK | USBFS_UEP_R_AUTO_TOG );
        } else {
            g_ch32_usbfs_udc.in_ep[epnum].ep_mps  = max_size;
            g_ch32_usbfs_udc.in_ep[epnum].ep_enable = true;
            USB_SET_TX_CTRL(epnum, USBFS_UEP_T_RES_NAK | USBFS_UEP_T_AUTO_TOG);
        }
    }
    return true;
}

int usbd_ep_close(const uint8_t ep) {
    (void)ep;
    return 0;
}
void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
    (void)rhport;

    uint8_t const ep_idx = tu_edpt_number(ep_addr);
    uint8_t const dir   = tu_edpt_dir(ep_addr);

    if (dir == TUSB_DIR_OUT) {
        if (ep_idx == 0) {
            USB_SET_RX_CTRL(0, USBFS_UEP_R_TOG | USBFS_UEP_R_RES_STALL) ;
        } else {
            USB_SET_RX_CTRL(ep_idx ,(USB_GET_RX_CTRL(0) & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_STALL);
        }
    } else {
        if (ep_idx == 0) {
            USBFSD_UEP_TX_CTRL(0) = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_STALL ;
            USBFS_DEVICE->UEP0_TX_LEN = 0;
        } else {
            USBFSD_UEP_TX_CTRL(0) = (USBFSD_UEP_TX_CTRL(0) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_STALL;
            USB_SET_TX_CTRL(ep_idx ,(USB_GET_TX_CTRL(0) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_R_RES_STALL);
        }
    }
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr) {
    (void)rhport;

    uint8_t const ep_idx = tu_edpt_number(ep_addr);
    uint8_t const dir = tu_edpt_dir(ep_addr);
    if (ep_idx == 0) {
        USBFSD_UEP_TX_CTRL(0) = USBFS_UEP_R_RES_ACK;
        return ;
    }
    if (dir == TUSB_DIR_OUT) {
        USB_SET_RX_CTRL(ep_idx ,(USB_GET_RX_CTRL(ep_idx) & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK)) | USBFS_UEP_R_RES_ACK);
    } else {
        USB_SET_TX_CTRL(ep_idx, (USB_GET_TX_CTRL(ep_idx)& ~(USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK)) | USBFS_UEP_T_RES_ACK);
    }
    return ;
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *data, uint16_t data_len) {
    (void)rhport;
    uint8_t const ep_idx = tu_edpt_number(ep_addr);
    if (!data && data_len) {
        return false;
    }

    if ((uint32_t)data & 0x03) {
        printf("data do not align4\r\n");
        return false;
    }

    g_ch32_usbfs_udc.in_ep[ep_idx].xfer_buf = (uint8_t *)data;
    g_ch32_usbfs_udc.in_ep[ep_idx].xfer_len = data_len;
    g_ch32_usbfs_udc.in_ep[ep_idx].actual_xfer_len = 0;

    if (ep_idx == 0) {
        if (data_len == 0) {
            USB_SET_TX_LEN(ep_idx, 0);
        } else {
            data_len = MIN(data_len, g_ch32_usbfs_udc.in_ep[ep_idx].ep_mps);
            USB_SET_TX_LEN(ep_idx, data_len);
            USB_SET_DMA(ep_idx, (uint32_t)data);
        }
        if (ep0_tx_data_toggle) {
            USB_SET_TX_CTRL(ep_idx, USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK);
        } else {
            USB_SET_TX_CTRL(ep_idx, USBFS_UEP_T_RES_ACK);
        }

    } else {
        if (data_len == 0) {
            USB_SET_TX_LEN(ep_idx, 0);
        } else {
            data_len = MIN(data_len, g_ch32_usbfs_udc.in_ep[ep_idx].ep_mps);
            USB_SET_TX_LEN(ep_idx, data_len);
            memcpy(&g_ch32_usbfs_udc.ep_databuf[ep_idx - 1][64], data, data_len);
        }
        USB_SET_TX_CTRL(ep_idx, (USB_GET_TX_CTRL(ep_idx) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK);
    }
    return true;
}


static void receive_packet(ch32_usbfs_ep_state *xfer, uint16_t xfer_size) {
    // xfer->queued_len = xfer->total_len - remaining;
#if 0
    uint16_t ep_mps;    /* Endpoint max packet size */
    uint8_t ep_type;    /* Endpoint type */
    uint8_t ep_stalled; /* Endpoint stall flag */
    uint8_t ep_enable;  /* Endpoint enable */
    uint8_t *xfer_buf;
    uint32_t xfer_len;
    uint32_t actual_xfer_len;
#endif
    uint16_t remaining = xfer->xfer_len - xfer->actual_xfer_len;
    uint16_t to_recv_size;

    if (remaining <= xfer->ep_mps) {
        // Avoid buffer overflow.
        to_recv_size = (xfer_size > remaining) ? remaining : xfer_size;
    } else {
        // Room for full packet, choose recv_size based on what the microcontroller
        // claims.
        to_recv_size = (xfer_size > xfer->ep_mps) ? xfer->ep_mps : xfer_size;
    }

    if (to_recv_size) {
    }

    xfer->xfer_len += xfer_size;

    // Per USB spec, a short OUT packet (including length 0) is always
    // indicative of the end of a transfer (at least for ctl, bulk, int).
}

extern int use_old;

void USBFS_IRQHandler(void)
{
    if(use_old) return USBFS_IRQHandler2();
    uint32_t ep_idx = 0, token, write_count, read_count;
    uint8_t intflag = 0;

    intflag = USBFS_DEVICE->INT_FG;

    if (intflag & USBFS_UIF_TRANSFER) {
        token  = USBFS_DEVICE->INT_ST & USBFS_UIS_TOKEN_MASK;
        ep_idx = USBFS_DEVICE->INT_ST & USBFS_UIS_ENDP_MASK;
        uint8_t endp = ep_idx | (token == USBFS_UIS_TOKEN_IN ? TUSB_DIR_IN_MASK : 0);

        switch (token) {
            case USBFS_UIS_TOKEN_SETUP:
                //USB_SET_RX_CTRL(ep_idx, USBFS_UEP_R_RES_NAK);
                USBFSD->UEP0_CTRL_H = USBFS_UEP_T_TOG|USBFS_UEP_T_RES_NAK|USBFS_UEP_R_TOG|USBFS_UEP_R_RES_NAK;
//                dcd_event_setup_received(0, (uint8_t *)&g_ch32_usbfs_udc.setup, true);
                dcd_event_setup_received(0, (uint8_t *)&g_ch32_usbfs_udc.ep_databuf[0], true);
                // usbd_event_ep0_setup_complete_handler((uint8_t *)&g_ch32_usbfs_udc.setup);
                break;

            case USBFS_UIS_TOKEN_IN:
                if (ep_idx == 0x00) {
                    if (g_ch32_usbfs_udc.in_ep[ep_idx].xfer_len > g_ch32_usbfs_udc.in_ep[ep_idx].ep_mps) {
                        g_ch32_usbfs_udc.in_ep[ep_idx].xfer_len -= g_ch32_usbfs_udc.in_ep[ep_idx].ep_mps;
                        g_ch32_usbfs_udc.in_ep[ep_idx].actual_xfer_len += g_ch32_usbfs_udc.in_ep[ep_idx].ep_mps;
                        ep0_tx_data_toggle ^= 1;
                    } else {
                        g_ch32_usbfs_udc.in_ep[ep_idx].actual_xfer_len += g_ch32_usbfs_udc.in_ep[ep_idx].xfer_len;
                        g_ch32_usbfs_udc.in_ep[ep_idx].xfer_len = 0;
                        ep0_tx_data_toggle = true;
                    }

                    dcd_event_xfer_complete(0, endp,  g_ch32_usbfs_udc.in_ep[ep_idx].actual_xfer_len, XFER_RESULT_SUCCESS, true);
                    // usbd_event_ep_in_complete_handler(ep_idx | 0x80, g_ch32_usbfs_udc.in_ep[ep_idx].actual_xfer_len);

                    if (g_ch32_usbfs_udc.dev_addr > 0) {
                        USBFS_DEVICE->DEV_ADDR = (USBFS_DEVICE->DEV_ADDR & USBFS_UDA_GP_BIT) | g_ch32_usbfs_udc.dev_addr;
                        g_ch32_usbfs_udc.dev_addr = 0;
                    }

                    if (g_ch32_usbfs_udc.setup.wLength && ((g_ch32_usbfs_udc.setup.bmRequestType & USBFS_UIS_TOKEN_MASK) == USBFS_UIS_TOKEN_OUT)) {
                        /* In status, start reading setup */
                        USB_SET_DMA(ep_idx, (uint32_t)&g_ch32_usbfs_udc.setup);
                        USB_SET_RX_CTRL(ep_idx, USBFS_UEP_R_RES_ACK);
                        ep0_tx_data_toggle = true;

                    } else if (g_ch32_usbfs_udc.setup.wLength == 0) {
                        /* In status, start reading setup */
                        USB_SET_DMA(ep_idx, (uint32_t)&g_ch32_usbfs_udc.setup);
                        USB_SET_RX_CTRL(ep_idx, USBFS_UEP_R_RES_ACK);
                        ep0_tx_data_toggle = true;
                    }
                } else {
                    USB_SET_TX_CTRL(ep_idx, (USB_GET_TX_CTRL(ep_idx) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK);

                    if (g_ch32_usbfs_udc.in_ep[ep_idx].xfer_len > g_ch32_usbfs_udc.in_ep[ep_idx].ep_mps) {
                        g_ch32_usbfs_udc.in_ep[ep_idx].xfer_buf += g_ch32_usbfs_udc.in_ep[ep_idx].ep_mps;
                        g_ch32_usbfs_udc.in_ep[ep_idx].xfer_len -= g_ch32_usbfs_udc.in_ep[ep_idx].ep_mps;
                        g_ch32_usbfs_udc.in_ep[ep_idx].actual_xfer_len += g_ch32_usbfs_udc.in_ep[ep_idx].ep_mps;

                        write_count = MIN(g_ch32_usbfs_udc.in_ep[ep_idx].xfer_len, g_ch32_usbfs_udc.in_ep[ep_idx].ep_mps);
                        USB_SET_TX_LEN(ep_idx, write_count);
                        memcpy(&g_ch32_usbfs_udc.ep_databuf[ep_idx - 1][64], g_ch32_usbfs_udc.in_ep[ep_idx].xfer_buf, write_count);

                        USB_SET_TX_CTRL(ep_idx, (USB_GET_TX_CTRL(ep_idx) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK);
                    } else {
                        g_ch32_usbfs_udc.in_ep[ep_idx].actual_xfer_len += g_ch32_usbfs_udc.in_ep[ep_idx].xfer_len;
                        g_ch32_usbfs_udc.in_ep[ep_idx].xfer_len = 0;
                        // usbd_event_ep_in_complete_handler(ep_idx | 0x80, g_ch32_usbfs_udc.in_ep[ep_idx].actual_xfer_len);
                        dcd_event_xfer_complete(0, endp,g_ch32_usbfs_udc.in_ep[ep_idx].actual_xfer_len, XFER_RESULT_SUCCESS, true);

                    }
                }
                break;
            case USBFS_UIS_TOKEN_OUT:
                if (ep_idx == 0x00) {
                    USB_SET_RX_CTRL(ep_idx, USBFS_UEP_R_RES_NAK);

                    read_count = USBFS_DEVICE->RX_LEN;

                    g_ch32_usbfs_udc.out_ep[ep_idx].actual_xfer_len += read_count;
                    g_ch32_usbfs_udc.out_ep[ep_idx].xfer_len -= read_count;

                    // usbd_event_ep_out_complete_handler(0x00, g_ch32_usbfs_udc.out_ep[ep_idx].actual_xfer_len);
                    dcd_event_xfer_complete(0, endp, g_ch32_usbfs_udc.out_ep[ep_idx].actual_xfer_len, XFER_RESULT_SUCCESS, true);
                    if (read_count == 0) {
                        /* Out status, start reading setup */
                        USB_SET_DMA(ep_idx, (uint32_t)&g_ch32_usbfs_udc.setup);
                        USB_SET_RX_CTRL(ep_idx, USBFS_UEP_R_RES_ACK);
                        ep0_rx_data_toggle = true;
                        ep0_tx_data_toggle = true;
                    } else {
                        ep0_rx_data_toggle ^= 1;
                    }
                } else {
                    if (USBFS_DEVICE->INT_ST & USBFS_UIS_TOG_OK) {
                        USB_SET_RX_CTRL(ep_idx, (USB_GET_RX_CTRL(ep_idx) & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK);
                        read_count = USBFS_DEVICE->RX_LEN;

                        memcpy(g_ch32_usbfs_udc.out_ep[ep_idx].xfer_buf, &g_ch32_usbfs_udc.ep_databuf[ep_idx - 1][0], read_count);

                        g_ch32_usbfs_udc.out_ep[ep_idx].xfer_buf += read_count;
                        g_ch32_usbfs_udc.out_ep[ep_idx].actual_xfer_len += read_count;
                        g_ch32_usbfs_udc.out_ep[ep_idx].xfer_len -= read_count;

                        if ((read_count < g_ch32_usbfs_udc.out_ep[ep_idx].ep_mps) || (g_ch32_usbfs_udc.out_ep[ep_idx].xfer_len == 0)) {
                            dcd_event_xfer_complete(0, endp, g_ch32_usbfs_udc.out_ep[ep_idx].actual_xfer_len, XFER_RESULT_SUCCESS, true);

                        } else {
                            USB_SET_RX_CTRL(ep_idx, (USB_GET_RX_CTRL(ep_idx) & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK);
                        }
                    }
                }
                break;

            case USBFS_UIS_TOKEN_SOF:

                break;

            default:
                break;
        }

        USBFS_DEVICE->INT_FG = USBFS_UIF_TRANSFER;
    } else if (intflag & USBFS_UIF_BUS_RST) {

        usb_set_tx_ctr(0,USBFS_UEP_T_RES_NAK);
        usb_set_rx_ctr(0,USBFS_UEP_R_RES_NAK);
        usb_set_tx_len(0,0);
        USBFSD->DEV_ADDR = 0;
        USBFS_Device_InitEndpoints( );
        USBFSD->INT_FG = USBFS_UIF_BUS_RST;
    } else if (intflag & USBFS_UIF_SUSPEND) {
        dcd_event_t event = { .rhport = 0, .event_id = DCD_EVENT_SUSPEND };
        dcd_event_handler(&event, true);
        USBFS_DEVICE->INT_FG = USBFS_UIF_SUSPEND;
    } else {
        USBFS_DEVICE->INT_FG = intflag;
    }
}

#endif
