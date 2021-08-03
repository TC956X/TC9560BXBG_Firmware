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
 *	  18 July 2016 : 
 */

/* ==========================================================================
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_pcd_linux.c $
 * $Revision: 1.11 $
 * $Date: 2016/07/18 23:50:47 $
 * $Change: 2224063 $
 *
 * Synopsys HS OTG Linux Software Driver and documentation (hereinafter,
 * "Software") is an Unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto. You are permitted to use and
 * redistribute this Software in source and binary forms, with or without
 * modification, provided that redistributions of source code must retain this
 * notice. You may not view, use, disclose, copy or distribute this file or
 * any information contained herein except pursuant to this license grant from
 * Synopsys. If you do not agree with this notice, including the disclaimer
 * below, then you are not authorized to use the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * ========================================================================== */

/** @file
 * This file implements the Peripheral Controller Driver.
 *
 * The Peripheral Controller Driver (PCD) is responsible for
 * translating requests from the Function Driver into the appropriate
 * actions on the DWC_otg controller. It isolates the Function Driver
 * from the specifics of the controller by providing an API to the
 * Function Driver.
 *
 * The Peripheral Controller Driver for Linux will implement the
 * Gadget API, so that the existing Gadget drivers can be used.
 * (Gadget Driver is the Linux terminology for a Function Driver.)
 *
 * The Linux Gadget API is defined in the header file
 * <code><linux/usb_gadget.h></code>.  The USB EP operations API is
 * defined in the structure <code>usb_ep_ops</code> and the USB
 * Controller API is defined in the structure
 * <code>usb_gadget_ops</code>.
 *
 */

#include "dwc_otg_os_dep.h"
#include "dwc_otg_pcd_if.h"
#include "dwc_otg_pcd.h"
#include "dwc_otg_driver.h"
#include "dwc_otg_dbg.h"
#include "neu_os.h"
#include "tc9560_uart.h"
#include "dwc_otg_pcd_linux.h"

#ifndef LM_INTERFACE
#define LM_INTERFACE
#endif

static struct usb_endpoint_descriptor neu_hs_bulk_out1_desc;
static struct usb_endpoint_descriptor neu_hs_bulk_out2_desc;
static struct usb_endpoint_descriptor neu_hs_bulk_out3_desc;
static struct usb_endpoint_descriptor neu_hs_bulk_out4_desc;
static struct usb_endpoint_descriptor neu_hs_bulk_in5_desc;
static struct usb_endpoint_descriptor neu_hs_bulk_in6_desc;
static struct usb_endpoint_descriptor neu_hs_bulk_in7_desc;
static struct usb_endpoint_descriptor neu_hs_bulk_in8_desc;
static struct usb_ep_ops dwc_otg_pcd_ep_ops = {NULL};

#if 1
static struct usb_endpoint_descriptor neu_hs_bulk_out1_desc;
void neu_hs_bulk_out1_desc_defn(void) { 	
			neu_hs_bulk_out1_desc.bLength = USB_DT_ENDPOINT_SIZE; 
			neu_hs_bulk_out1_desc.bDescriptorType = USB_DT_ENDPOINT;
			neu_hs_bulk_out1_desc.bEndpointAddress = 0x01; 
			/* bEndpointAddress copied from fs_bulk_out_desc during fsg_bind() y */
			neu_hs_bulk_out1_desc.bmAttributes = USB_ENDPOINT_XFER_BULK; 
			neu_hs_bulk_out1_desc.wMaxPacketSize = 512; 
			neu_hs_bulk_out1_desc.bInterval = 0x05; /* NAK every 1 uframe*/ 
}
#endif
#if 1
void neu_hs_bulk_out2_desc_defn(void) { 	
			neu_hs_bulk_out2_desc.bLength = USB_DT_ENDPOINT_SIZE; 
			neu_hs_bulk_out2_desc.bDescriptorType = USB_DT_ENDPOINT;
			neu_hs_bulk_out2_desc.bEndpointAddress = 0x02;
			/* bEndpointAddress copied from fs_bulk_out_desc during fsg_bind() */
			neu_hs_bulk_out2_desc.bmAttributes = USB_ENDPOINT_XFER_BULK; 
			neu_hs_bulk_out2_desc.wMaxPacketSize = 512; 
			neu_hs_bulk_out2_desc.bInterval = 0x05; /* NAK every 1 uframe */
}

void neu_hs_bulk_out3_desc_defn(void)  { 	
			neu_hs_bulk_out3_desc.bLength = USB_DT_ENDPOINT_SIZE; 
			neu_hs_bulk_out3_desc.bDescriptorType = USB_DT_ENDPOINT;
			neu_hs_bulk_out3_desc.bEndpointAddress = 0x03;
			/* bEndpointAddress copied from fs_bulk_out_desc during fsg_bind() */
			neu_hs_bulk_out3_desc.bmAttributes = USB_ENDPOINT_XFER_BULK; 
			neu_hs_bulk_out3_desc.wMaxPacketSize = 512; 
			neu_hs_bulk_out3_desc.bInterval = 0x05; /* NAK every 1 uframe */
}

void neu_hs_bulk_out4_desc_defn(void) { 	
			neu_hs_bulk_out4_desc.bLength = USB_DT_ENDPOINT_SIZE; 
			neu_hs_bulk_out4_desc.bDescriptorType = USB_DT_ENDPOINT;
			neu_hs_bulk_out4_desc.bEndpointAddress = 0x04;
			/* bEndpointAddress copied from fs_bulk_out_desc during fsg_bind() */
			neu_hs_bulk_out4_desc.bmAttributes = USB_ENDPOINT_XFER_BULK; 
			neu_hs_bulk_out4_desc.wMaxPacketSize = 512; 
			neu_hs_bulk_out4_desc.bInterval = 0x05; /* NAK every 1 uframe */
}

void neu_hs_bulk_in5_desc_defn(void) { 
			neu_hs_bulk_in5_desc.bLength = USB_DT_ENDPOINT_SIZE; 
			neu_hs_bulk_in5_desc.bDescriptorType = USB_DT_ENDPOINT;
			neu_hs_bulk_in5_desc.bEndpointAddress = 0x85;
			/* bEndpointAddress copied from fs_bulk_in_desc during fsg_bind() */
			neu_hs_bulk_in5_desc.bmAttributes = USB_ENDPOINT_XFER_BULK; 
			neu_hs_bulk_in5_desc.wMaxPacketSize = 512; 
			neu_hs_bulk_in5_desc.bInterval = 0x05;
}

void neu_hs_bulk_in6_desc_defn(void)  { 
			neu_hs_bulk_in6_desc.bLength = USB_DT_ENDPOINT_SIZE; 
			neu_hs_bulk_in6_desc.bDescriptorType = USB_DT_ENDPOINT;
			neu_hs_bulk_in6_desc.bEndpointAddress = 0x86;
			/* bEndpointAddress copied from fs_bulk_in_desc during fsg_bind() */
			neu_hs_bulk_in6_desc.bmAttributes = USB_ENDPOINT_XFER_BULK; 
			neu_hs_bulk_in6_desc.wMaxPacketSize = 512; 
			neu_hs_bulk_in6_desc.bInterval = 0x05;
}

void neu_hs_bulk_in7_desc_defn(void)  { 
			neu_hs_bulk_in7_desc.bLength = USB_DT_ENDPOINT_SIZE; 
			neu_hs_bulk_in7_desc.bDescriptorType = USB_DT_ENDPOINT;
			neu_hs_bulk_in7_desc.bEndpointAddress = 0x87;
			/* bEndpointAddress copied from fs_bulk_in_desc during fsg_bind() */
			neu_hs_bulk_in7_desc.bmAttributes = USB_ENDPOINT_XFER_BULK;
			neu_hs_bulk_in7_desc.wMaxPacketSize = 512; 
			neu_hs_bulk_in7_desc.bInterval = 0x05;
}

void neu_hs_bulk_in8_desc_defn(void)  { 
			neu_hs_bulk_in8_desc.bLength = USB_DT_ENDPOINT_SIZE; 
			neu_hs_bulk_in8_desc.bDescriptorType = USB_DT_ENDPOINT;
			neu_hs_bulk_in8_desc.bEndpointAddress = 0x88;
			/* bEndpointAddress copied from fs_bulk_in_desc during fsg_bind() */
			neu_hs_bulk_in8_desc.bmAttributes = USB_ENDPOINT_XFER_BULK; 
			neu_hs_bulk_in8_desc.wMaxPacketSize = 512; 
			neu_hs_bulk_in8_desc.bInterval = 0x05;
}

 struct usb_descriptor_header *neu_hs_function[] = {
		(struct usb_descriptor_header *) &neu_intf_desc,
		(struct usb_descriptor_header *) &neu_hs_bulk_in5_desc,
		(struct usb_descriptor_header *) &neu_hs_bulk_in6_desc,	
		(struct usb_descriptor_header *) &neu_hs_bulk_in7_desc,	
		(struct usb_descriptor_header *) &neu_hs_bulk_in8_desc,	
		(struct usb_descriptor_header *) &neu_hs_bulk_out1_desc, 
		(struct usb_descriptor_header *) &neu_hs_bulk_out2_desc,
		(struct usb_descriptor_header *) &neu_hs_bulk_out3_desc,
		(struct usb_descriptor_header *) &neu_hs_bulk_out4_desc, NULL, };

#endif

static int32_t neu_setup(struct usb_gadget const * gadget, const struct usb_ctrlrequest *ctrl);

/*** Function Declararions ***/

/* Display the contents of the buffer */

static void neu_unbind(struct usb_gadget *gadget);
struct neu_dev *the_neu;

static inline uint32_t usb_endpoint_maxp(const struct usb_endpoint_descriptor *epd) {
	return epd->wMaxPacketSize;
}

static void bulk_in_complete(struct usb_ep const *ep, struct usb_request const *req) {
	struct neu_buffhd *bh = req->context;
	
	bh->inreq_busy = 0;
	bh->state = BUF_STATE_EMPTY;
	(void)ep;
}

static void bulk_in6_complete(struct usb_ep const *ep, struct usb_request const *req) {
	struct neu_buffhd *bh = req->context;
	
	bh->inreq_busy = 0;
	bh->state = BUF_STATE_EMPTY;
	(void)ep;
}

static void bulk_in7_complete(struct usb_ep const *ep, struct usb_request const *req) {
	struct neu_buffhd *bh = req->context;
	
	bh->inreq_busy = 0;
	bh->state = BUF_STATE_EMPTY;
	(void)ep;
}

static void bulk_in8_complete(struct usb_ep const *ep, struct usb_request const *req) {
	struct neu_buffhd *bh = req->context;
	
	bh->inreq_busy = 0;
	bh->state = BUF_STATE_EMPTY;
	(void)ep;
}

/* Callback function for Legacy traffic */
static void bulk_out0_complete(struct usb_ep const *ep, struct usb_request  *req) {
	struct neu_buffhd *bh = req->context;
	usb_start_transmit(bh->zoutreq[req->req_num]->buf, req, 0);
	(void)ep;
}

/* Callback function for AVB Class A traffic */
static void bulk_out1_complete(struct usb_ep const *ep, struct usb_request  *req) {
	struct neu_buffhd *bh = req->context;

	usb_start_transmit(bh->zoutreq[req->req_num]->buf, req, 1);
	(void)ep;
}

/* Callback function for AVB Class B traffic */
static void bulk_out2_complete(struct usb_ep const *ep, struct usb_request  *req) {
	struct neu_buffhd *bh = req->context;
	
	usb_start_transmit(bh->zoutreq[req->req_num]->buf, req, 2);
	(void)ep;
}

/* Callback function for GPTP traffic */
static void bulk_out3_complete(struct usb_ep const *ep, struct usb_request  *req) {
	struct neu_buffhd *bh = req->context;

	usb_start_transmit(bh->zoutreq[req->req_num]->buf, req, 3);
	(void)ep;
}


/* USB Endpoint Operations */
/*
 * The following sections briefly describe the behavior of the Gadget
 * API endpoint operations implemented in the DWC_otg driver
 * software. Detailed descriptions of the generic behavior of each of
 * these functions can be found in the Linux header file
 * include/linux/usb_gadget.h.
 *
 * The Gadget API provides wrapper functions for each of the function
 * pointers defined in usb_ep_ops. The Gadget Driver calls the wrapper
 * function, which then calls the underlying PCD function. The
 * following sections are named according to the wrapper
 * functions. Within each section, the corresponding DWC_otg PCD
 * function name is specified.
 *
 */

/**
 * This function is called by the Gadget Driver for each EP to be
 * configured for the current configuration (SET_CONFIGURATION).
 *
 * This function initializes the dwc_otg_ep_t data structure, and then
 * calls dwc_otg_ep_activate.
 */
static int32_t ep_enable(struct usb_ep *usb_ep,
		const struct usb_endpoint_descriptor *ep_desc) {
	int32_t retval = 0;

	DBG_USB_Print(DBG_PCDV, "%s(%p,%p)\n", __func__, usb_ep, ep_desc);

	if (((!(usb_ep)) || (!(ep_desc))) || ((ep_desc->bDescriptorType) != (USB_DT_ENDPOINT))) {
		DBG_Warn_Print("%s, bad ep or descriptor\n", __func__);
		retval = -EINVAL;
	}else if (usb_ep == &gadget_wrapper->ep0) {
		DBG_Warn_Print("%s, bad ep(0)\n", __func__);
		retval = -EINVAL;
	}else if (!ep_desc->wMaxPacketSize) {/* Check FIFO size? */
		DBG_Warn_Print("%s, bad %s maxpacket\n", __func__, usb_ep->name);
		retval = -ERANGE;
	}else if ((!(gadget_wrapper->driver))
			|| ((gadget_wrapper->gadget.speed) == (USB_SPEED_UNKNOWN))) {
		DBG_Warn_Print("%s, bogus device state\n", __func__);
		retval = -ESHUTDOWN;
	}else {
		retval = dwc_otg_pcd_ep_enable(gadget_wrapper->pcd,
				(const uint8_t *) ep_desc, (void *) usb_ep);
		if (retval != 0) {
			DBG_Warn_Print("dwc_otg_pcd_ep_enable failed\n");
			retval = -EINVAL;
		}else {
			usb_ep->maxpacket = ep_desc->wMaxPacketSize; /* setting the max packet size used by this ep */
			retval = 0;
		}
	}
	return retval;
}

/**
 * This function is called when an EP is disabled due to disconnect or
 * change in configuration. Any pending requests will terminate with a
 * status of -ESHUTDOWN.
 *
 * This function modifies the dwc_otg_ep_t data structure for this EP,
 * and then calls dwc_otg_ep_deactivate.
 */
static int32_t ep_disable(struct usb_ep const *usb_ep) {
	int32_t retval;
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	DBG_USB_Print(DBG_PCDV, "%s(%p)\n", __func__, usb_ep);
	if (!usb_ep) {
		DBG_USB_Print(DBG_PCD,
				"%s, %s not enabled\n", __func__, usb_ep ? usb_ep->name : NULL);
		retval = -EINVAL;
	}else {
		retval = dwc_otg_pcd_ep_disable(gadget_wrapper->pcd, usb_ep);
		if (retval != 0) {
			retval = -EINVAL;
		}
	}
	return retval;
}

/**
 * This function allocates a request object to use with the specified
 * endpoint.
 *
 * @param ep The endpoint to be used with with the request
 * @param gfp_flags the GFP_* flags to use.
 */
static struct usb_request *dwc_otg_pcd_alloc_request(struct usb_ep const *ep,
		int32_t gfp_flags) /* TODO: gfp_flags is unused here */
{
	struct usb_request *usb_req = NULL;

	DBG_USB_Print(DBG_PCDV, "%s(%p,%d)\n", __func__, ep, gfp_flags);
	if (0 == ep) {
		DBG_Warn_Print("%s() %s\n", __func__, "Invalid EP!\n");
	}else {
		usb_req = DWC_ALLOC(sizeof(struct usb_request));
		
		if (0 == usb_req) {
			DBG_Warn_Print("%s() %s\n", __func__, "request allocation failed!\n");
		}else {
			memset(usb_req, 0, sizeof(struct usb_request));
			usb_req->dma = DWC_DMA_ADDR_INVALID; 
			/* TODO: May have to give the support for this(see usb_ep_alloc_request() in gadget.h) */ 
		}
	}
	(void)gfp_flags;
	return usb_req;
}

/**
 * This function frees a request object.
 *
 * @param ep The endpoint associated with the request
 * @param req The request being freed
 */
static void dwc_otg_pcd_free_request(struct usb_ep const *ep, struct usb_request *req) {
	DBG_USB_Print(DBG_PCDV, "%s(%p,%p)\n", __func__, ep, req);
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	if ((0 == ep) || (0 == req)) {
		DBG_Warn_Print("%s() %s\n", __func__, "Invalid ep or req argument!\n");		
	}else {
		DWC_FREE(req);
	}
	return;
}

/**
 * This function is used to submit an I/O Request to an EP.
 *
 *	- When the request completes the request's completion callback
 *	  is called to return the request to the driver.
 *	- An EP, except control EPs, may have multiple requests
 *	  pending.
 *	- Once submitted the request cannot be examined or modified.
 *	- Each request is turned into one or more packets.
 *	- A BULK EP can queue any amount of data; the transfer is
 *	  packetized.
 *	- Zero length Packets are specified with the request 'zero'
 *	  flag.
 */
 int32_t ep_queue(struct usb_ep const *usb_ep, struct usb_request *usb_req,	gfp_t gfp_flags) {
	dwc_otg_pcd_t *str_ptr_pcd;
	/* struct dwc_otg_pcd_ep *ep; */
	int32_t retval = 0;
	dma_addr_t str_dma_addr;
	DBG_USB_Print(DBG_PCDV,
			"%s(%p,%p,%d)\n", __func__, usb_ep, usb_req, gfp_flags);
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	if ((!(usb_req)) || (!(usb_req->buf))) {
		DBG_Warn_Print("bad params\n");
		retval = -EINVAL;
	}else if (!usb_ep) {
		DBG_Warn_Print("bad ep\n");
		retval = -EINVAL;
	}else {
		str_ptr_pcd = gadget_wrapper->pcd;
		if ((!(gadget_wrapper->driver))
				|| (gadget_wrapper->gadget.speed == USB_SPEED_UNKNOWN)) {
			DBG_USB_Print(DBG_PCDV,
					"gadget.speed=%d\n", gadget_wrapper->gadget.speed);
			DBG_Warn_Print("bogus device state\n");
			retval = -ESHUTDOWN;
		}else {

			DBG_USB_Print(DBG_PCD,
					"%s queue req %p, len %d buf %p\n", usb_ep->name, usb_req, usb_req->length, usb_req->buf);
			usb_req->status = -EINPROGRESS;
			usb_req->actual = 0;
			str_dma_addr = (dwc_dma_t)usb_req->buf;
			retval = dwc_otg_pcd_ep_queue(str_ptr_pcd, usb_ep, usb_req->buf, str_dma_addr,
					usb_req->length, usb_req->zero, usb_req);
			if (retval != 0) {
				retval = -EINVAL;
			}
		}
	}
	(void)gfp_flags;
	return retval;
}


/* Function for data travelling from USB to Eth */
void neu_ep_queue_out(uint32_t req_num, const char_t* ep_name, uint8_t out_ep)
{
	struct neu_buffhd *bh; 
	struct neu_dev *neu = the_neu;
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	if(out_ep == 1)
	{	
		bh = &neu->buffhds[4];
		ep_queue(neu->bulk_out1, bh->zoutreq[req_num], 0);
	}
	
	if(out_ep == 2)
	{	
		bh = &neu->buffhds[5];
		ep_queue(neu->bulk_out2, bh->zoutreq[req_num], 0);
	}
	if(out_ep == 3)
	{	
		bh = &neu->buffhds[6];
		ep_queue(neu->bulk_out3, bh->zoutreq[req_num], 0);
	}
	if(out_ep == 4)
	{	
		bh = &neu->buffhds[7];
		ep_queue(neu->bulk_out4, bh->zoutreq[req_num], 0);
	}
	(void)ep_name;
}

/* Function for data travelling from TC9560 to host */
int32_t neu_ep_queue_in(uint8_t *buf_addr, uint8_t bulk_in_num, uint32_t size)
{
	struct neu_buffhd *bh;
	struct neu_dev *neu = the_neu;
	uint8_t uint_req_num;
	int32_t retval;
	
	bh = &neu->buffhds[bulk_in_num];
	uint_req_num = bh->bi_req_num;
	bh->bi_req_num = (bh->bi_req_num + 1) % IN_REQ_NUMBER;
	bh->buf = buf_addr;
	bh->zinreq[uint_req_num]->buf  = buf_addr;
	bh->zinreq[uint_req_num]->length = size;
	DBG_Test_Print(DBG_TEST_IN_EP, "%s \n", __func__);
	switch (bulk_in_num) {
		case 1:
			retval = ep_queue(neu->bulk_in6, bh->zinreq[uint_req_num], 0);		
		break;
		
		case 2:
			retval = ep_queue(neu->bulk_in7, bh->zinreq[uint_req_num], 0);		
		break;
		
		case 3:
			retval = ep_queue(neu->bulk_in8, bh->zinreq[uint_req_num], 0);		
		break;
		
		case 0:
		default:
			retval = ep_queue(neu->bulk_in5, bh->zinreq[uint_req_num], 0);	
		break;
	}
	
	return retval;
}

#if 1
void dwc_otg_pcd_ep_ops_defn(void)  { 
	dwc_otg_pcd_ep_ops.enable =  ep_enable; 
	dwc_otg_pcd_ep_ops.disable = ep_disable;
	dwc_otg_pcd_ep_ops.alloc_request = dwc_otg_pcd_alloc_request;
	dwc_otg_pcd_ep_ops.free_request =  dwc_otg_pcd_free_request;
	dwc_otg_pcd_ep_ops.queue = ep_queue;
	dwc_otg_pcd_ep_ops.dequeue = NULL;
	dwc_otg_pcd_ep_ops.set_halt = NULL;
	dwc_otg_pcd_ep_ops.set_wedge = NULL;
	dwc_otg_pcd_ep_ops.fifo_status = 0; 
	dwc_otg_pcd_ep_ops.fifo_flush = 0;
}
#endif

/*	Gadget Operations */
/**
 * The following gadget operations will be implemented in the DWC_otg
 * PCD. Functions in the API that are not described below are not
 * implemented.
 *
 * The Gadget API provides wrapper functions for each of the function
 * pointers defined in usb_gadget_ops. The Gadget Driver calls the
 * wrapper function, which then calls the underlying PCD function. The
 * following sections are named according to the wrapper functions
 * (except for ioctl, which doesn't have a wrapper function). Within
 * each section, the corresponding DWC_otg PCD function name is
 * specified.
 *
 */
#ifdef CONFIG_USB_DWC_OTG_LPM
static int32_t test_lpm_enabled(struct usb_gadget *gadget)
{
	struct gadget_wrapper *d;

	d = container_of(gadget, struct gadget_wrapper, gadget);

	return dwc_otg_pcd_is_lpm_enabled(d->pcd);
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
static int32_t test_besl_enabled(struct usb_gadget *gadget)
{
	struct gadget_wrapper *d;

	d = container_of(gadget, struct gadget_wrapper, gadget);

	return dwc_otg_pcd_is_besl_enabled(d->pcd);
}

static int32_t get_param_baseline_besl(struct usb_gadget *gadget)
{
	struct gadget_wrapper *d;

	d = container_of(gadget, struct gadget_wrapper, gadget);

	return dwc_otg_pcd_get_param_baseline_besl(d->pcd);
}


static int32_t get_param_deep_besl(struct usb_gadget *gadget)
{
	struct gadget_wrapper *d;

	d = container_of(gadget, struct gadget_wrapper, gadget);

	return dwc_otg_pcd_get_param_deep_besl(d->pcd);
}
#endif
#endif
static int32_t setup(dwc_otg_pcd_t const * pcd, uint8_t const * bytes) {
	
	int32_t retval = -DWC_E_NOT_SUPPORTED;
	if ((gadget_wrapper->driver) && (gadget_wrapper->driver->setup)) {
		neu_setup(&gadget_wrapper->gadget, (struct usb_ctrlrequest*) bytes);
	}
	retval = 1;

	(void)pcd;
	return retval;
}

static int32_t complete(dwc_otg_pcd_t const *pcd, void const *ep_handle, void *req_handle,
		int32_t status, uint32_t actual) {
	  struct usb_request *ptr_usb_req = (struct usb_request *) req_handle;

		if ((ptr_usb_req) && (ptr_usb_req->complete)) {
		switch (status) {
		case -DWC_E_SHUTDOWN:
			ptr_usb_req->status = -ESHUTDOWN;
			break;
		case -DWC_E_INVALID:
			ptr_usb_req->status = -EINVAL;
			break;
		default:
			ptr_usb_req->status = status;

		}

		ptr_usb_req->actual = actual;
		ptr_usb_req->complete(ep_handle, ptr_usb_req);
	}
	(void)pcd;
	return 0;
}

static int32_t connect(dwc_otg_pcd_t const * pcd, int32_t speed) {
	gadget_wrapper->gadget.speed = (enum usb_device_speed)speed;
	(void)pcd;
	return 0;
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
static int32_t disconnect(dwc_otg_pcd_t const * pcd) {
	if ((gadget_wrapper->driver) && (gadget_wrapper->driver->disconnect)) {
		gadget_wrapper->driver->disconnect(&gadget_wrapper->gadget);
	}
	(void)pcd;
	return 0;
}

static int32_t resume(dwc_otg_pcd_t const * pcd) {
	if ((gadget_wrapper->driver) && (gadget_wrapper->driver->resume)) {
		gadget_wrapper->driver->resume(&gadget_wrapper->gadget);
	}
	(void)pcd;
	return 0;
}
#endif

static int32_t suspend(dwc_otg_pcd_t const * pcd) {
	if ((gadget_wrapper->driver) && (gadget_wrapper->driver->suspend)) {
		gadget_wrapper->driver->suspend(&gadget_wrapper->gadget);
	}
	(void)pcd;
	return 0;
}

/**
 * This function updates the otg values in the gadget structure.
 */
static int32_t int_hnp_changed(dwc_otg_pcd_t const * pcd) {

	if (!gadget_wrapper->gadget.is_otg) {
		;
	}else {
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		gadget_wrapper->gadget.b_hnp_enable = get_b_hnp_enable(pcd);
		gadget_wrapper->gadget.a_hnp_support = get_a_hnp_support(pcd);
		gadget_wrapper->gadget.a_alt_hnp_support = get_a_alt_hnp_support(pcd);
#endif
	}
	return 0;
}

static int32_t reset(dwc_otg_pcd_t const * pcd) {
	(void)pcd;
	return 0;
}

/**
 * This function is the top level PCD interrupt handler.
 */
void dwc_otg_pcd_irq(void *dev) {
	dwc_otg_pcd_t *str_ptr_pcd = dev;
	int32_t retval = IRQ_NONE;

	retval = dwc_otg_pcd_handle_intr(str_ptr_pcd);
	if (retval != 0) {
		S3C2410X_CLEAR_EINTPEND();
	}

}

/**
 * This function initialized the usb_ep structures to their default
 * state.
 *
 * @param d Pointer on gadget_wrapper.
 */
void gadget_add_eps(struct gadget_wrapper *d) {
	static const char_t *names[] = {

	"ep0", "ep1in", "ep2in", "ep3in", "ep4in", "ep5in", "ep6in", "ep7in",
			"ep8in", "ep9in", "ep10in", "ep11in", "ep12in", "ep13in", "ep14in",
			"ep15in", "ep1out", "ep2out", "ep3out", "ep4out", "ep5out",
			"ep6out", "ep7out", "ep8out", "ep9out", "ep10out", "ep11out",
			"ep12out", "ep13out", "ep14out", "ep15out" };

	uint32_t i;
	struct usb_ep *str_usb_ep;
	uint8_t uchar_dev_endpoints;

	d->gadget.ep0 = &d->ep0;
	d->gadget.speed = USB_SPEED_UNKNOWN;
	d->gadget.max_speed = USB_SPEED_HIGH;

	/**
	 * Initialize the EP0 structure.
	 */
	str_usb_ep = &d->ep0;

	/* Init the usb_ep structure. */
	str_usb_ep->name = names[0];
	str_usb_ep->ops = (struct usb_ep_ops *) &dwc_otg_pcd_ep_ops;

	/**
	 * @todo NGS: Set the max packet size for EP
	 */
	str_usb_ep->maxpacket = MAX_PACKET_SIZE;
	dwc_otg_pcd_ep_enable(d->pcd, NULL, str_usb_ep);

	/**
	 * Initialize the EP structures.
	 */
	uchar_dev_endpoints = d->pcd->core_if->dev_if->num_in_eps;

	for (i = 0; i < uchar_dev_endpoints; i++) {
		str_usb_ep = &d->in_ep[i];

		/* Init the usb_ep structure. */
		str_usb_ep->name = names[d->pcd->in_ep[i].dwc_ep.num];
		str_usb_ep->ops = (struct usb_ep_ops *) &dwc_otg_pcd_ep_ops;

		/**
		 * Set the max packet size for the EP
		 */
		str_usb_ep->maxpacket = MAX_PACKET_SIZE;
	}

	uchar_dev_endpoints = d->pcd->core_if->dev_if->num_out_eps;

	for (i = 0; i < uchar_dev_endpoints; i++) {
		str_usb_ep = &d->out_ep[i];

		/* Init the usb_ep structure. */
		str_usb_ep->name = names[15 + d->pcd->out_ep[i].dwc_ep.num];
		str_usb_ep->ops = (struct usb_ep_ops *) &dwc_otg_pcd_ep_ops;

		/**
		 * Set the max packet size for the EP
		 */
		str_usb_ep->maxpacket = MAX_PACKET_SIZE;

	}

	/* Allocate Packet size for EP0. */
	d->ep0.maxpacket = MAX_EP0_SIZE;
}

/**
 * This function allocated memory to gadget wrapper structure.
 * state.
 *
 */
static struct gadget_wrapper *alloc_wrapper(struct lm_device const *_dev)
{
	static char_t pcd_name[] = "dwc_otg_pcd";
	dwc_otg_device_t *ptr_otg_dev = lm_get_drvdata(_dev);
	static struct usb_gadget_ops dwc_otg_pcd_ops;
#ifdef CONFIG_USB_DWC_OTG_LPM
	dwc_otg_pcd_ops.lpm_support = test_lpm_enabled;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)	
	dwc_otg_pcd_ops.besl_support = test_besl_enabled;
	dwc_otg_pcd_ops.get_baseline_besl = get_param_baseline_besl;
	dwc_otg_pcd_ops.get_deep_besl = get_param_deep_besl;
#endif	
#endif

	struct gadget_wrapper *d;

	d = DWC_ALLOC(sizeof(*d));
	if (d != NULL) {
		memset(d, 0, sizeof(*d));
		d->gadget.name = pcd_name;
		d->pcd = ptr_otg_dev->pcd;
		d->gadget.ops = &dwc_otg_pcd_ops;
	}
	return d;
}

/**
 * This function initialized the PCD portion of the driver.
 *
 */
int32_t pcd_init(struct lm_device const *_dev) 
{
	dwc_otg_device_t *ptr_otg_dev = lm_get_drvdata(_dev);
	int32_t retval = 0;
#if 1
	static struct dwc_otg_pcd_function_ops str_fops;
	str_fops.complete = complete;
	str_fops.setup =  setup; 
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	str_fops.disconnect =  disconnect;
    str_fops.resume =  resume;
#endif
	str_fops.connect = connect;
	str_fops.suspend = suspend; 
	str_fops.hnp_changed = int_hnp_changed;
	str_fops.reset =  reset;
	str_fops.sleep = NULL;
	str_fops.cfi_setup = NULL;
#endif


	ptr_otg_dev->pcd = dwc_otg_pcd_init(ptr_otg_dev->core_if);

	if (!ptr_otg_dev->pcd) {
		retval = -ENOMEM;
	}else {
		ptr_otg_dev->pcd->otg_dev = ptr_otg_dev;
		/* Memory allocation to endpoing structure */
		gadget_wrapper = alloc_wrapper(_dev);
		/*
		 * Initialize EP structures
		 */
		gadget_add_eps(gadget_wrapper);
		/*
		 * Setup interupt handler
		 */
		cb_array[1].handler = dwc_otg_pcd_irq;
		cb_array[1].dev = ptr_otg_dev->pcd;

		dwc_otg_pcd_start(gadget_wrapper->pcd, &str_fops); 
		/*dwc_otg_pcd_start(gadget_wrapper->pcd, &fops);  */
	}
	return retval;
}




/**
 * This function registers a gadget driver with the PCD.
 *
 * When a driver is successfully registered, it will receive control
 * requests including set_configuration(), which enables non-control
 * requests.  then usb traffic follows until a disconnect is reported.
 * then a host may connect again, or the driver might get unbound.
 *
 * @param driver The driver being registered
 * @param bind The bind function of gadget driver
 */
int32_t usb_gadget_probe_driver(struct usb_gadget_driver *driver,
		int32_t (*bind)(struct usb_gadget *)) {
	int32_t retval;
	int32_t return_value = 0;
	
	if (gadget_wrapper == 0) {
		DBG_USB_Print(DBG_PCDV, "ENODEV\n");
		return_value = -ENODEV;
	}else if (gadget_wrapper->driver != 0) {
		DBG_USB_Print(DBG_PCDV, "EBUSY (%p)\n", gadget_wrapper->driver);
		return_value = -EBUSY;
	}else {
		/* hook up the driver */
		gadget_wrapper->driver = driver;
		retval = bind(&gadget_wrapper->gadget);

		if (retval != 0) {
			gadget_wrapper->driver = 0;
			gadget_wrapper->gadget.dev.driver = 0;
			return_value = retval;
		}
	}
	return return_value;
}

/**
 * This function unregisters a gadget driver
 *
 * @param driver The driver being unregistered
 */
int32_t usb_gadget_unregister_driver(struct usb_gadget_driver const *driver) {
	int32_t retval = 0;

	if (gadget_wrapper == 0) {
		DBG_USB_Print(DBG_USB, "%s Return(%d): s_pcd==0\n", __func__, -ENODEV);
		retval = -ENODEV;
	}else if ((driver == 0) || (driver != gadget_wrapper->driver)) {
		DBG_USB_Print(DBG_USB, "%s Return(%d): driver?\n", __func__, -EINVAL);
		retval = -EINVAL;
	}else {
		driver->unbind(&gadget_wrapper->gadget);
		gadget_wrapper->driver = 0;
	}
	return retval;
}
#if 1
struct usb_gadget_driver neu_driver;
void neu_driver_call (void) { 
		neu_driver.max_speed = USB_SPEED_HIGH;
		neu_driver.bind = neu_bind; 	   
		neu_driver.unbind = neu_unbind;
		neu_driver.setup = neu_setup; /* neu_setup should be called from gadget driver setup() */
		neu_driver.disconnect = NULL; 
		neu_driver.suspend = NULL; 
		neu_driver.resume = NULL; 
		neu_driver.reset = NULL; 
		}
#endif


int32_t do_set_interface(struct neu_dev *neu, int32_t altsetting) {	
	int32_t rc = 0;
	uint32_t i;
	const struct usb_endpoint_descriptor *d = NULL;
	struct neu_buffhd *bh; 

	if (neu->running != 0)
		DBG_USB_Print(DBG_USB,"reset interface\n");

	
	reset_intr:
	/* Disable the endpoints */
	if (neu->bulk_in5_enabled != 0) {
		ep_disable(neu->bulk_in5);
		neu->bulk_in5_enabled = 0;
	}
	if (neu->bulk_in6_enabled != 0) {
		ep_disable(neu->bulk_in6);
		neu->bulk_in6_enabled = 0;
	}

	if (neu->bulk_in7_enabled != 0) {
		ep_disable(neu->bulk_in7);
		neu->bulk_in7_enabled = 0;
	}

	if (neu->bulk_in8_enabled != 0) {
		ep_disable(neu->bulk_in8);
		neu->bulk_in8_enabled = 0;
	}
	
	if (neu->bulk_out1_enabled != 0) {
		ep_disable(neu->bulk_out1);
		neu->bulk_out1_enabled = 0;
	}

	if (neu->bulk_out2_enabled != 0) {
		ep_disable(neu->bulk_out2);
		neu->bulk_out2_enabled = 0;
	}

	if (neu->bulk_out3_enabled != 0) {
		ep_disable(neu->bulk_out3);
		neu->bulk_out3_enabled = 0;
	}

	if (neu->bulk_out4_enabled != 0) {
		ep_disable(neu->bulk_out4);
		neu->bulk_out4_enabled = 0;
	}
	
	neu->running = 0;
	if ((altsetting < 0) || (rc != 0)) { 
		;
	}else {

		if (neu->gadget->speed == USB_SPEED_HIGH) {
			d = &neu_hs_bulk_in5_desc;
		}
		rc = ep_enable(neu->bulk_in5, d);
		if (rc != 0) {
			goto reset_intr;
		}
		neu->bulk_in5_enabled = 1;

		
		if (neu->gadget->speed == USB_SPEED_HIGH) {
			d = &neu_hs_bulk_in6_desc;
		}
		rc = ep_enable(neu->bulk_in6, d);
		if (rc != 0) {
			goto reset_intr;
		}
		neu->bulk_in6_enabled = 1;

		if (neu->gadget->speed == USB_SPEED_HIGH) {
			d = &neu_hs_bulk_in7_desc;
		}
		rc = ep_enable(neu->bulk_in7, d);
		if (rc != 0) {
			goto reset_intr;
		}
		neu->bulk_in7_enabled = 1;

		if (neu->gadget->speed == USB_SPEED_HIGH) {
			d = &neu_hs_bulk_in8_desc;
		}
		rc = ep_enable(neu->bulk_in8, d);
		if (rc != 0) {
			goto reset_intr;
		}
		neu->bulk_in8_enabled = 1;
		
		if (neu->gadget->speed == USB_SPEED_HIGH) {
			d = &neu_hs_bulk_out1_desc;
		}
		rc = ep_enable(neu->bulk_out1, d);
		if (rc != 0) {
			goto reset_intr;
		}
		neu->bulk_out1_enabled = 1;

		
		if (neu->gadget->speed == USB_SPEED_HIGH) {
			d = &neu_hs_bulk_out2_desc;
		}
		rc = ep_enable(neu->bulk_out2, d);
		if (rc != 0) {
			goto reset_intr;
		}
		neu->bulk_out2_enabled = 1;

		if (neu->gadget->speed == USB_SPEED_HIGH) {
			d = &neu_hs_bulk_out3_desc;
		}
		rc = ep_enable(neu->bulk_out3, d);
		if (rc != 0) {
			goto reset_intr;
		}
		neu->bulk_out3_enabled = 1;

		if (neu->gadget->speed == USB_SPEED_HIGH) {
			d = &neu_hs_bulk_out4_desc;
		}
		rc = ep_enable(neu->bulk_out4, d);
		if (rc != 0) {
			goto reset_intr;
		}
		neu->bulk_out4_enabled = 1;
		
		neu->bulk_out_maxpacket = usb_endpoint_maxp(d);

		bh = &neu->buffhds[0];
		bh->bi_req_num = 0;
		for (i=0; i < IN_REQ_NUMBER; i++) 
		{
			bh->zinreq[i] = dwc_otg_pcd_alloc_request(neu->bulk_in5, GFP_KERNEL);
			bh->zinreq[i]->req_num = i;
			bh->zinreq[i]->buf  = bh->buf;
			bh->zinreq[i]->context = bh;
			neu->running = 1;
			bh->zinreq[i]->complete = (void (*)(const struct usb_ep *, struct usb_request *))bulk_in_complete;
			bh->zinreq[i]->length = BH_BUF_SIZE;
			DBG_USB_Print(DBG_USB,"zz: in1 req=%p\n",i, bh->zinreq[i]);
		}
		
		bh = &neu->buffhds[1];
		bh->bi_req_num = 0;
		for (i=0; i < IN_REQ_NUMBER; i++) 
		{
			bh->zinreq[i] = dwc_otg_pcd_alloc_request(neu->bulk_in6, GFP_KERNEL);
			bh->zinreq[i]->req_num = i;
			bh->zinreq[i]->buf  = bh->buf;
			bh->zinreq[i]->context = bh;
			neu->running = 1;
			bh->zinreq[i]->complete =(void (*)(const struct usb_ep *, struct usb_request *)) bulk_in6_complete;
			bh->zinreq[i]->length = BH_BUF_SIZE;
			DBG_USB_Print(DBG_USB,"zz: in2 req=%p\n",i, bh->zinreq[i]);
		}
		
		bh = &neu->buffhds[2];
		bh->bi_req_num = 0;
		for (i=0; i < IN_REQ_NUMBER; i++) 
		{
			bh->zinreq[i] = dwc_otg_pcd_alloc_request(neu->bulk_in7, GFP_KERNEL);
			bh->zinreq[i]->req_num = i;
			bh->zinreq[i]->buf  = bh->buf;
			bh->zinreq[i]->context = bh;
			neu->running = 1;
			bh->zinreq[i]->complete = (void (*)(const struct usb_ep *, struct usb_request *))bulk_in7_complete;
			bh->zinreq[i]->length = BH_BUF_SIZE;
			DBG_USB_Print(DBG_USB,"zz: in3 req=%p\n",i, bh->zinreq[i]);
		}
		

		bh = &neu->buffhds[3];  /* GPTP */
		bh->bi_req_num = 0;
		for (i=0; i < IN_REQ_NUMBER; i++) 
		{
			bh->zinreq[i] = dwc_otg_pcd_alloc_request(neu->bulk_in8, GFP_KERNEL);
			bh->zinreq[i]->req_num = i;
			/* bh->inreq->buf  = bh->buf;	 */	
			bh->zinreq[i]->context = bh;
			neu->running = 1;
			bh->zinreq[i]->complete = (void (*)(const struct usb_ep *, struct usb_request *))bulk_in8_complete;
			bh->zinreq[i]->length = BH_BUF_PTP_SIZE;
			DBG_USB_Print(DBG_USB,"zz: in4 req=%p\n",i, bh->zinreq[i]);
		}

		bh = &neu->buffhds[4];
		for (i=0; i < OUT_REQ_NUMBER; i++) 
		{
			bh->buf = DWC_ALLOC(BH_BUF_SIZE); 
			
			bh->zoutreq[i] = dwc_otg_pcd_alloc_request(neu->bulk_out1, GFP_KERNEL);
			bh->zoutreq[i]->context = bh;
			bh->zoutreq[i]->complete = bulk_out0_complete;
			bh->zoutreq[i]->length = BH_BUF_SIZE;
			bh->zoutreq[i]->req_num = i;
			bh->zoutreq[i]->buf  = bh->buf;
			
			DBG_USB_Print(DBG_USB,"zz: out1 buf%d=%p\n",i, bh->buf);
			ep_queue(neu->bulk_out1, bh->zoutreq[i], 0);
		}
		neu->running = 1;

		bh = &neu->buffhds[5];
		for (i=0; i < OUT_REQ_NUMBER; i++) 
		{
			bh->buf = DWC_ALLOC(BH_BUF_SIZE); 
			
			bh->zoutreq[i] = dwc_otg_pcd_alloc_request(neu->bulk_out2, GFP_KERNEL);
			bh->zoutreq[i]->context = bh;
			bh->zoutreq[i]->complete = bulk_out1_complete;
			bh->zoutreq[i]->length = BH_BUF_SIZE;
			bh->zoutreq[i]->req_num = i;
			bh->zoutreq[i]->buf  = bh->buf;
			
			DBG_USB_Print(DBG_USB,"zz: out2 buf%d=%p\n",i, bh->buf);
			ep_queue(neu->bulk_out2, bh->zoutreq[i], 0);
		}
		neu->running = 1;
		

		bh = &neu->buffhds[6];
		for (i=0; i < OUT_REQ_NUMBER; i++) 
		{
			bh->buf = DWC_ALLOC(BH_BUF_SIZE); 
			
			bh->zoutreq[i] = dwc_otg_pcd_alloc_request(neu->bulk_out3, GFP_KERNEL);
			bh->zoutreq[i]->context = bh;
			bh->zoutreq[i]->complete = bulk_out2_complete;
			bh->zoutreq[i]->length = BH_BUF_SIZE;
			bh->zoutreq[i]->req_num = i;
			bh->zoutreq[i]->buf  = bh->buf;
			
			DBG_USB_Print(DBG_USB,"zz: out3 buf%d=%p\n",i, bh->buf);
			ep_queue(neu->bulk_out3, bh->zoutreq[i], 0);
		}
		neu->running = 1;

		bh = &neu->buffhds[7];
		for (i=0; i < OUT_REQ_NUMBER; i++) 
		{
			bh->buf = DWC_ALLOC(BH_BUF_PTP_SIZE); 
			
			bh->zoutreq[i] = dwc_otg_pcd_alloc_request(neu->bulk_out4, GFP_KERNEL);
			bh->zoutreq[i]->context = bh;
			bh->zoutreq[i]->complete = bulk_out3_complete;
			bh->zoutreq[i]->length = BH_BUF_PTP_SIZE;
			bh->zoutreq[i]->req_num = i;
			bh->zoutreq[i]->buf  = bh->buf;
			
			DBG_USB_Print(DBG_USB,"zz: out4 buf%d=%p\n",i, bh->buf);
			ep_queue(neu->bulk_out4, bh->zoutreq[i], 0);
		}
		neu->running = 1;
	}
	return rc;
}

static int32_t do_set_config(struct neu_dev *neu, uint8_t new_config) {
	int32_t rc = 0;

	rc = do_set_interface(neu, 0);
	(void)new_config;
	return rc;
}

static int32_t neu_setup(struct usb_gadget const * gadget, const struct usb_ctrlrequest *ctrl) {
	struct usb_request *ptr_req = the_neu->ep0req; /* TODO: how to get the ref struct usb_request */
		
	struct neu_dev *neu = (struct neu_dev *) dev_get_drvdata(&gadget->dev);

	ptr_req->length = 512;
	do_set_config(neu, neu->new_config);
	(void)ctrl;
	return 0;
}

static void neu_unbind(struct usb_gadget *gadget) {
	struct neu_dev *neu = (struct neu_dev *) dev_get_drvdata(&gadget->dev);
	

	struct usb_request *ptr_req = neu->ep0req;
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	DBG_USB_Print(DBG_USB,"unbind\n");

	/* If the thread isn't already dead, tell it to exit now */
	if (neu->state != NEU_STATE_TERMINATED) {
		do_set_config(neu, 0);
		neu->state = NEU_STATE_TERMINATED;
	}

	/* Free the request and buffer for endpoint 0 */
	if (ptr_req != 0) {
		DWC_FREE(ptr_req->buf);
		dwc_otg_pcd_free_request(neu->ep0, ptr_req);
	}

	dev_set_drvdata(&gadget->dev, NULL);
}

int32_t neu_bind(struct usb_gadget *gadget) {
	
	struct neu_dev *neu = the_neu;
	int32_t  rc = 0;
	struct usb_request *str_usb_req;
	struct usb_ep *str_usb_ep;
	int32_t retval = 0;
	int32_t flag = 0;
	
	neu->gadget = gadget;
	dev_set_drvdata(&gadget->dev, neu);
	neu->ep0 = gadget->ep0;
	neu->ep0->driver_data = neu;

	str_usb_ep = &gadget_wrapper->in_ep[0];  
	if(!str_usb_ep) {
	   flag = AUTOCONF_FAIL;
	}else {
		str_usb_ep->driver_data = neu; /* claim the endpoint */
		neu->bulk_in5 = str_usb_ep;

		str_usb_ep = &gadget_wrapper->in_ep[1]; 
		if(!str_usb_ep) {
		   flag = AUTOCONF_FAIL;
		}else {
			str_usb_ep->driver_data = neu; /* claim the endpoint */
			neu->bulk_in6 = str_usb_ep;

			str_usb_ep = &gadget_wrapper->in_ep[2];  
			if(!str_usb_ep) {
			   flag = AUTOCONF_FAIL;
			}else {
			
				str_usb_ep->driver_data = neu; /* claim the endpoint */
				neu->bulk_in7 = str_usb_ep;

				str_usb_ep = &gadget_wrapper->in_ep[3];  
				if(!str_usb_ep) {
				   flag = AUTOCONF_FAIL;
				}else {
					str_usb_ep->driver_data = neu; /* claim the endpoint */
					neu->bulk_in8 = str_usb_ep;

					str_usb_ep = &gadget_wrapper->out_ep[0]; 
					if(!str_usb_ep) {
						flag = AUTOCONF_FAIL;
					}else {

						str_usb_ep->driver_data = neu; /* claim the endpoint */
						neu->bulk_out1 = str_usb_ep;

						str_usb_ep = &gadget_wrapper->out_ep[1]; 
						if(!str_usb_ep){
							flag = AUTOCONF_FAIL;
						}else {
							str_usb_ep->driver_data = neu; /* claim the endpoint */
							neu->bulk_out2 = str_usb_ep;

							str_usb_ep = &gadget_wrapper->out_ep[2]; 
							if(!str_usb_ep) {
								flag = AUTOCONF_FAIL;
							}else {
								str_usb_ep->driver_data = neu; /* claim the endpoint */
								neu->bulk_out3 = str_usb_ep;

								str_usb_ep = &gadget_wrapper->out_ep[3]; 
								if(!str_usb_ep) {
									flag = AUTOCONF_FAIL;
								}else {
									str_usb_ep->driver_data = neu; /* claim the endpoint */
									neu->bulk_out4 = str_usb_ep;


									/* Allocate the request and buffer for endpoint 0 */
									neu->ep0req = str_usb_req = dwc_otg_pcd_alloc_request(neu->ep0, GFP_KERNEL); 
									if (!str_usb_req) {
										flag = OUT_NEU_BIND;
									}else {
										str_usb_req->buf = DWC_ALLOC(EP0_BUFSIZE);
										if (!str_usb_req->buf) {
											flag = OUT_NEU_BIND;
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}

	switch (flag){
		case AUTOCONF_FAIL:
			DBG_Error_Print("unable to autoconfigure all endpoints\n");
			rc = -ENOTSUPP;
			neu->state = NEU_STATE_TERMINATED; 
			neu_unbind(gadget);
			retval = rc;
			break;
		case OUT_NEU_BIND:
			neu->state = NEU_STATE_TERMINATED; 
			neu_unbind(gadget);
			retval = rc;
			break;
		default :
			retval = 0;
	}
	return retval;
}

int32_t neu_init(void) {

	int32_t rc;
	int32_t retval = 0;
	struct neu_dev *neu = NULL;

	/*Allocate neu device structure*/
	rc = sizeof *neu;
	neu = DWC_ALLOC((uint32_t)rc);
	if (!neu) {
		retval = -ENOMEM;
	}
	else {
		the_neu = neu;
		rc = usb_gadget_probe_driver(&neu_driver, neu_driver.bind);
		if (rc != 0) {
			DBG_Error_Print("usb_gadget_probe_driver EINVAL\n");
		}
		retval = rc;
	}
	return retval;
}
