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

/* ==========================================================================
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_pcd.h $
 * $Revision: 1.7 $
 * $Date: 2016/07/18 23:50:47 $
 * $Change: 2231774 $
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
#ifndef __DWC_OTG_PCD_H__
#define __DWC_OTG_PCD_H__

#include "dwc_otg_os_dep.h"
#include "usb.h"
#include "dwc_otg_cil.h"
#include "dwc_otg_pcd_if.h"
#include "neu_os.h"
#include "tc9560_common.h"
struct cfiobject;

/**
 * @file
 *
 * This file contains the structures, constants, and interfaces for
 * the Perpherial Contoller Driver (PCD).
 *
 * The Peripheral Controller Driver (PCD) for Linux will implement the
 * Gadget API, so that the existing Gadget drivers can be used. For
 * the Mass Storage Function driver the File-backed USB Storage Gadget
 * (FBS) driver will be used.  The FBS driver supports the
 * Control-Bulk (CB), Control-Bulk-Interrupt (CBI), and Bulk-Only
 * transports.
 *
 */

/** Invalid DMA Address */
#define DWC_DMA_ADDR_INVALID	(~ (dwc_dma_t)0)

/** Max Transfer size for any EP */
#define DDMA_MAX_TRANSFER_SIZE	65535

/**
 * Get the pointer to the core_if from the pcd pointer.
 */
#define GET_CORE_IF( _pcd)		(_pcd->core_if)
#define EINPROGRESS				150	/* Operation now in progress. Taken from linux open source*/
#define EINVAL					22	/* Invalid argument */
#define ESHUTDOWN				143 /* Cannot send after transport endpoint shutdown */
#define USB_DT_ENDPOINT			0x05 

struct scatterlist {
	uint64_t	page_link;
	uint32_t	offset;
	uint32_t	length;
	dma_addr_t	dma_address;
};

//#define neu_num_buffers 8
#define NEU_NUMBER_BUFFHD		8

enum neu_buffer_state {
	BUF_STATE_EMPTY = 0, BUF_STATE_FULL, BUF_STATE_BUSY
};

#define OUT_REQ_NUMBER			8
#define IN_REQ_NUMBER			8
#define BH_BUF_SIZE				1600
#define BH_BUF_PTP_SIZE			0xA0

struct neu_buffhd {
	void * buf;
	enum neu_buffer_state state;
	//struct neu_buffhd *next;
	uint32_t  bulk_out_intended_length;
	//struct usb_request *inreq;
	struct usb_request * zinreq[IN_REQ_NUMBER];
	int32_t inreq_busy;
	int32_t outreqnum;
	//struct usb_request *outreq;
	struct usb_request * zoutreq[OUT_REQ_NUMBER];
	int32_t outreq_busy;
	uint8_t bi_req_num;
};
 
/**
 * struct usb_ep - device side representation of USB endpoint
 * @name:identifier for the endpoint, such as "ep-a" or "ep9in-bulk"
 * @ops: Function pointers used to access hardware-specific operations.
 * @ep_list:the gadget's ep_list holds all of its endpoints
 * @maxpacket:The maximum packet size used on this endpoint.  The initial
 *		value can sometimes be reduced (hardware allowing), according to
 *		the endpoint descriptor used to configure the endpoint.
 * @maxpacket_limit:The maximum packet size value which can be handled by this
 *		endpoint. It's set once by UDC driver when endpoint is initialized, and
 *		should not be changed. Should not be confused with maxpacket.
 * @max_streams: The maximum number of streams supported
 *		by this EP (0 - 16, actual number is 2^n)
 * @mult: multiplier, 'mult' value for SS Isoc EPs
 * @maxburst: the maximum number of bursts supported by this EP (for usb3)
 * @driver_data:for use by the gadget driver.
 * @address: used to identify the endpoint when finding descriptor that
 *		matches connection speed
 * @desc: endpoint descriptor.	This pointer is set before the endpoint is
 *		enabled and remains valid until the endpoint is disabled.
 * @comp_desc: In case of SuperSpeed support, this is the endpoint companion
 *		descriptor that is used to configure the endpoint
 *
 * the bus controller driver lists all the general purpose endpoints in
 * gadget->ep_list.	 the control endpoint (gadget->ep0) is not in that list,
 * and is accessed only in response to a driver setup() callback.
 */
struct usb_ep {
	void * driver_data;
	const char_t * name;
	const struct usb_ep_ops * ops;
	struct list_head ep_list;
	USIGN maxpacket :16;
	USIGN maxpacket_limit :16;
	USIGN max_streams :16;
	USIGN mult :2;
	USIGN maxburst :5;
	uint8_t address;
	const struct usb_endpoint_descriptor * desc;
	const struct usb_ss_ep_comp_descriptor * comp_desc;
};



 struct usb_request {
	void				 * buf;
	USIGN				 length;
	dma_addr_t			 dma;
	struct scatterlist	 * sg;
	USIGN				 num_sgs;
	USIGN				 num_mapped_sgs;
	USIGN				 stream_id:16;
	USIGN				 no_interrupt:1;
	USIGN				 zero:1;
	USIGN				 short_not_ok:1;
	void (* complete)	 (struct usb_ep const * ep,struct usb_request  * req);
	void				 * context;
	struct list_head	 list;
	int32_t				 status;
	uint32_t			 actual;
	uint32_t			 req_num;
	uint8_t				 bulk_out_num;
};


/* endpoint-specific parts of the api to the usb controller hardware.
* unlike the urb model, (de)multiplexing layers are not required.
* (so this api could slash overhead if used on the host side...)
*
* note that device side usb controllers commonly differ in how many
* endpoints they support, as well as their capabilities.
*/
struct usb_ep_ops {
	int32_t (* enable) (struct usb_ep * usb_ep, const struct usb_endpoint_descriptor * ep_desc);
	int32_t (* disable) (struct usb_ep const * usb_ep);
	struct 	usb_request * (* alloc_request) (struct usb_ep const * ep, gfp_t gfp_flags);
	void 	(* free_request) (struct usb_ep const * ep, struct usb_request * req);
	int32_t (* queue) (struct usb_ep const * usb_ep, struct usb_request * usb_req, gfp_t gfp_flags);
	int32_t (* dequeue) (struct usb_ep const * usb_ep, struct usb_request const * usb_req);
	int32_t (* set_halt) (struct usb_ep const * usb_ep, int32_t value);
	int32_t (* set_wedge) (struct usb_ep const * usb_ep);
	int32_t (* fifo_status) (struct usb_ep * ep);
	void 	(* fifo_flush) (struct usb_ep * ep);
};



/**
 * States of EP0.
 */
typedef enum ep0_state {
	EP0_DISCONNECT,		/* no host */
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_IN_STATUS_PHASE,
	EP0_OUT_STATUS_PHASE,
	EP0_STALL,
} ep0state_e;

/** Fordward declaration.*/
struct dwc_otg_pcd;

/** DWC_otg request structure.
 * This structure is a list of requests.
 */
typedef struct dwc_otg_pcd_request {
	void * priv;
	void * buf;
	dwc_dma_t dma;
	uint32_t length;
	uint32_t actual;
	USIGN sent_zlp:1;
	/**
	 * Used instead of original buffer if
	 * it(physical address) is not dword-aligned.
	 **/
	uint8_t * dw_align_buf;
	dwc_dma_t dw_align_buf_dma;

	 DWC_CIRCLEQ_ENTRY(dwc_otg_pcd_request) queue_entry;
} dwc_otg_pcd_request_t;

DWC_CIRCLEQ_HEAD(req_list, dwc_otg_pcd_request);

/**	  PCD EP structure.
 * This structure describes an EP, there is an array of EPs in the PCD
 * structure.
 */
typedef struct dwc_otg_pcd_ep {
	/** USB EP Descriptor */
	const usb_endpoint_descriptor_t *desc;

	/** queue of dwc_otg_pcd_requests. */
	struct req_list queue;
	USIGN stopped:1;
	USIGN disabling:1;
	USIGN dma:1;
	USIGN queue_sof:1;

	/** DWC_otg ep data. */
	dwc_ep_t dwc_ep;

	/** Pointer to PCD */
	struct dwc_otg_pcd * pcd;

	void * priv;
} dwc_otg_pcd_ep_t;

/** DWC_otg PCD Structure.
 * This structure encapsulates the data for the dwc_otg PCD.
 */
struct dwc_otg_pcd {
	const struct dwc_otg_pcd_function_ops * fops;
	/** The DWC otg device pointer */
	struct dwc_otg_device * otg_dev;
	/** Core Interface */
	dwc_otg_core_if_t * core_if;
	/** State of EP0 */
	ep0state_e ep0state;
	/** EP0 Request is pending */
	USIGN ep0_pending:1;
	/** Indicates when SET CONFIGURATION Request is in process */
	USIGN request_config:1;
	/** The state of the Remote Wakeup Enable. */
	USIGN remote_wakeup_enable:1;
	/** The state of the B-Device HNP Enable. */
	USIGN b_hnp_enable:1;
	/** The state of A-Device HNP Support. */
	USIGN a_hnp_support:1;
	/** The state of the A-Device Alt HNP support. */
	USIGN a_alt_hnp_support:1;
	/** Count of pending Requests */
	USIGN request_pending;

	/** SETUP packet for EP0
	 * This structure is allocated as a DMA buffer on PCD initialization
	 * with enough space for up to 3 setup packets.
	 */
	union {
		usb_device_request_t req;
		uint32_t d32[2];
	} * setup_pkt;

	dwc_dma_t setup_pkt_dma_handle;

	/* Additional buffer and flag for CTRL_WR premature case */
	uint8_t * backup_buf;
	USIGN data_terminated;

	/** 2-byte dma buffer used to return status from GET_STATUS */
	uint16_t * status_buf;
	dwc_dma_t status_buf_dma_handle;

	/** EP0 */
	dwc_otg_pcd_ep_t ep0;

	/** Array of IN EPs. */
	dwc_otg_pcd_ep_t in_ep[MAX_EPS_CHANNELS - 1];
	/** Array of OUT EPs. */
	dwc_otg_pcd_ep_t out_ep[MAX_EPS_CHANNELS - 1];

	/** The test mode to enter when the tasklet is executed. */
	USIGN test_mode;

};

//FIXME this functions should be static, and this prototypes should be removed
extern void dwc_otg_request_nuke(dwc_otg_pcd_ep_t * ep);
extern void dwc_otg_request_done(dwc_otg_pcd_ep_t * ep, dwc_otg_pcd_request_t * req, int32_t status);

/**********************function declaration******************/
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
static int32_t dwc_otg_pcd_start_cb(void * p);
static int32_t dwc_otg_pcd_resume_cb(void * p);
#endif
static int32_t dwc_otg_pcd_suspend_cb(void * p);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
static int32_t dwc_otg_pcd_stop_cb(void * p);
#endif
static int32_t assign_tx_fifo(dwc_otg_core_if_t * core_if);
static int32_t assign_tx_fifo(dwc_otg_core_if_t * core_if);
static void release_perio_tx_fifo(dwc_otg_core_if_t * core_if, uint32_t fifo_num);
static void release_tx_fifo(dwc_otg_core_if_t * core_if, uint32_t fifo_num);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
static uint32_t get_ep_of_last_in_token(dwc_otg_core_if_t const * core_if);
#endif
static int32_t get_device_speed(dwc_otg_core_if_t const * core_if);
static dwc_otg_pcd_ep_t * get_ep_from_handle(dwc_otg_pcd_t * pcd, void const * handle);
void dwc_otg_ep_free_desc_chain(dwc_otg_dev_dma_desc_t * desc_addr, uint32_t dma_desc_addr, uint32_t count);
static void dwc_otg_pcd_reinit(dwc_otg_pcd_t * pcd);
static void dwc_otg_pcd_init_ep(dwc_otg_pcd_t * pcd, dwc_otg_pcd_ep_t * pcd_ep, uint32_t is_in, uint32_t ep_num);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
extern void dwc_otg_pcd_stop(dwc_otg_pcd_t * pcd);
extern void dwc_otg_pcd_rem_wkup_from_suspend(dwc_otg_pcd_t const * pcd, int32_t set);
#endif
static void dwc_otg_pcd_update_otg(dwc_otg_pcd_t * pcd, const USIGN reset);
static dwc_otg_pcd_ep_t * get_in_ep(dwc_otg_pcd_t * pcd, uint32_t ep_num);
static inline dwc_otg_pcd_ep_t * get_out_ep(dwc_otg_pcd_t * pcd, uint32_t ep_num);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
static int32_t write_empty_tx_fifo(dwc_otg_pcd_t * pcd, uint32_t epnum);
#endif
static inline void ep0_out_start(dwc_otg_core_if_t const * core_if, dwc_otg_pcd_t const * pcd);
static inline void ep0_do_stall(dwc_otg_pcd_t * pcd, const int32_t err_val);
static inline void do_gadget_setup(dwc_otg_pcd_t * pcd, usb_device_request_t const * ctrl);
static inline void do_setup_in_status_phase(dwc_otg_pcd_t * pcd);
static inline void do_setup_out_status_phase(dwc_otg_pcd_t * pcd);
static inline void pcd_clear_halt(dwc_otg_pcd_t * pcd, dwc_otg_pcd_ep_t * ep);
static inline void do_get_status(dwc_otg_pcd_t * pcd);
static inline void do_get_descriptor(dwc_otg_pcd_t *pcd);
static inline void do_set_feature(dwc_otg_pcd_t * pcd);
static inline void do_clear_feature(dwc_otg_pcd_t * pcd);
static inline void do_set_address(dwc_otg_pcd_t * pcd);
static inline void pcd_setup(dwc_otg_pcd_t * pcd);
static int32_t ep0_complete_request(dwc_otg_pcd_ep_t * ep);
static void complete_ep(dwc_otg_pcd_ep_t * ep);
static void handle_ep0(dwc_otg_pcd_t * pcd);
static inline void handle_in_ep_disable_intr(dwc_otg_pcd_t * pcd, const uint32_t epnum);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
static void dwc_otg_pcd_handle_noniso_bna(dwc_otg_pcd_ep_t * ep);
static void restart_transfer(dwc_otg_pcd_t * pcd, const uint32_t epnum);
static inline void handle_in_ep_timeout_intr(dwc_otg_pcd_t * pcd, const uint32_t epnum);
static inline int32_t handle_out_ep_babble_intr(dwc_otg_pcd_t const * pcd, const uint32_t epnum);
static inline int32_t handle_out_ep_nak_intr(dwc_otg_pcd_t const * pcd, const uint32_t epnum);
static inline int32_t handle_out_ep_nyet_intr(dwc_otg_pcd_t const * pcd, const uint32_t epnum);
#endif
static int32_t dwc_otg_pcd_handle_in_ep_intr(dwc_otg_pcd_t * pcd);
static int32_t dwc_otg_pcd_handle_out_ep_intr(dwc_otg_pcd_t * pcd);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
extern int32_t dwc_otg_pcd_handle_sof_intr(dwc_otg_pcd_t const * pcd);
extern int32_t dwc_otg_pcd_handle_ep_mismatch_intr(dwc_otg_pcd_t const * pcd);
extern int32_t dwc_otg_pcd_handle_ep_fetsusp_intr(dwc_otg_pcd_t const * pcd);
#endif

#endif
