/* ============================================================================
 * PROJECT: TC9560
 * Copyright (C) 2018  Toshiba Electronic Devices & Storage Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * ========================================================================= */

/*! History:   
 *		18 July 2016 :	
 */
 
#ifndef __NEU_OS_H__
#define __NEU_OS_H__

#include "includes.h"
#include "dwc_list.h"

#ifndef LM_INTERFACE
#define LM_INTERFACE
#endif

/* KERNEL ERROR CODES*/
#define ERESTARTSYS		512
/* SYSTEM ERROR CODES*/
#define ENOMEM			12		/* Out of memory */

#ifdef	__cplusplus
extern "C" {
#endif
extern uint32_t REG_RD(uint32_t volatile * reg);
extern void REG_WR(uint32_t volatile * reg, uint32_t value);
int32_t neu_init(void);
#define GFP_KERNEL		1
/**
 * Modify bit values in a register.	 Using the
 * algorithm: (reg_contents & ~clear_mask) | set_mask.
 */
extern void DWC_MODIFY_REG32(uint32_t volatile * reg, uint32_t clear_mask, uint32_t set_mask);

#define container_of(ptr, type, member) ({ \
	const typeof( ((type *)0)->member ) \
	*__mptr = (ptr); \
	(type *)( (char *)__mptr - offsetof(type,member) );}) 

		
 struct tasklet_struct
 {
		 struct tasklet_struct * next;
		 unsigned long state;
		 void (* func)(unsigned long);
		 unsigned long data;
};
struct lm_device; 

struct device { 
				void * driver_data;
				struct device_driver * driver;


};
 
struct lm_device {
		struct device		 dev;
		uint32_t			 irq;
		uint32_t			 id;
};

/* Refered from Linux/include/linux/irqreturn.h */
/**
 * enum irqreturn
 * @IRQ_NONE			interrupt was not from this device
 * @IRQ_HANDLED			interrupt was handled by this device
 * @IRQ_WAKE_THREAD		handler requests to wake the handler thread
 */
  enum irqreturn {
		  IRQ_NONE				  = (0 << 0),
		  IRQ_HANDLED			  = (1 << 0),
		  IRQ_WAKE_THREAD		  = (1 << 1),
  };
  
  typedef enum irqreturn irqreturn_t;
  #define IRQ_RETVAL(x)	  ((x != 0) ? IRQ_HANDLED : IRQ_NONE)
 
struct list_head {
		 struct list_head * next, * prev;
};
 
enum usb_device_speed {
		 USB_SPEED_UNKNOWN = 0,					 /* enumerating */
		 USB_SPEED_LOW,							 /* usb 1.1 */
		 USB_SPEED_FULL,						 /* usb 1.1 */
		 USB_SPEED_HIGH,						 /* usb 2.0 */
		 USB_SPEED_WIRELESS,					 /* wireless (usb 2.5) */
		 USB_SPEED_SUPER						 /* usb 3.0 */
 };

#ifdef	__cplusplus
}
#endif

#endif	/* NEU_OS_H */
