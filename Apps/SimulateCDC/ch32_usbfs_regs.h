#ifndef _USB_CH32_USBHS_REG_H
#define _USB_CH32_USBHS_REG_H

#if (CFG_TUSB_MCU == OPT_MCU_CH32V307)
#include <ch32v30x.h>
#elif (CFG_TUSB_MCU == OPT_MCU_CH32X035)
#include <ch32x035.h>
#include <ch32x035_usb.h>
#endif

/******************* GLOBAL ******************/
#define EP_MAX 8


#endif
