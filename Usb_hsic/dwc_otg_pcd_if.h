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
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_pcd_if.h $
 * $Revision: 1.6 $
 * $Date: 2016/07/18 23:50:47 $
 * $Change: 2125019 $
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
#ifndef __DWC_OTG_PCD_IF_H__
#define __DWC_OTG_PCD_IF_H__

#include "dwc_os.h"
#include "dwc_otg_core_if.h"

/** @file
 * This file defines DWC_OTG PCD Core API.
 */

struct dwc_otg_pcd;
typedef struct dwc_otg_pcd dwc_otg_pcd_t;

/** Maxpacket size for EP0 */
#define MAX_EP0_SIZE	64
/** Maxpacket size for any EP */
#define MAX_PACKET_SIZE	1024

/** @name Function Driver Callbacks */
/** @{ */

/** This function will be called whenever a previously queued request has
 * completed.  The status value will be set to -DWC_E_SHUTDOWN to indicated a
 * failed or aborted transfer, or -DWC_E_RESTART to indicate the device was reset,
 * or -DWC_E_TIMEOUT to indicate it timed out, or -DWC_E_INVALID to indicate invalid
 * parameters. */
typedef int32_t (*dwc_completion_cb_t) (dwc_otg_pcd_t const * pcd, void const *ep_handle,
				void *req_handle, int32_t status, uint32_t actual);
/** This function should handle any SETUP request that cannot be handled by the
 * PCD Core.  This includes most GET_DESCRIPTORs, SET_CONFIGS, Any
 * class-specific requests, etc.  The function must non-blocking.
 *
 * Returns 0 on success.
 * Returns -DWC_E_NOT_SUPPORTED if the request is not supported.
 * Returns -DWC_E_INVALID if the setup request had invalid parameters or bytes.
 * Returns -DWC_E_SHUTDOWN on any other error. */
typedef int32_t (* dwc_setup_cb_t) (dwc_otg_pcd_t const * pcd, uint8_t const * bytes);
/** This is called whenever the device has been disconnected.  The function
 * driver should take appropriate action to clean up all pending requests in the
 * PCD Core, remove all endpoints (except ep0), and initialize back to reset
 * state. */
typedef int32_t (* dwc_disconnect_cb_t) (dwc_otg_pcd_t const * pcd);
/** This function is called when device has been connected. */
typedef int32_t (* dwc_connect_cb_t) (dwc_otg_pcd_t const * pcd, int32_t speed);
/** This function is called when device has been suspended */
typedef int32_t (* dwc_suspend_cb_t) (dwc_otg_pcd_t const * pcd);
/** This function is called when device has received LPM tokens, i.e.
 * device has been sent to sleep state. */
typedef int32_t (* dwc_sleep_cb_t) (dwc_otg_pcd_t const * pcd);
/** This function is called when device has been resumed
 * from suspend(L2) or L1 sleep state. */
typedef int32_t (* dwc_resume_cb_t) (dwc_otg_pcd_t const * pcd);
/** This function is called whenever hnp params has been changed.
 * User can call get_b_hnp_enable, get_a_hnp_support, get_a_alt_hnp_support functions
 * to get hnp parameters. */
typedef int32_t (* dwc_hnp_params_changed_cb_t) (dwc_otg_pcd_t const * pcd);
/** This function is called whenever USB RESET is detected. */
typedef int32_t (* dwc_reset_cb_t) (dwc_otg_pcd_t const * pcd);

typedef int32_t (* cfi_setup_cb_t) (dwc_otg_pcd_t const * pcd, void * ctrl_req_bytes);

/**
 *
 * @param ep_handle	Void pointer to the usb_ep structure
 * @param ereq_port Pointer to the extended request structure created in the
 *	portable part.
 */
typedef int32_t (* xiso_completion_cb_t) (dwc_otg_pcd_t * pcd, void * ep_handle,
				 void * req_handle, int32_t status, void * ereq_port);
/** Function Driver Ops Data Structure */
struct dwc_otg_pcd_function_ops {
	dwc_connect_cb_t connect;
	dwc_disconnect_cb_t disconnect;
	dwc_setup_cb_t setup;
	dwc_completion_cb_t complete;
	dwc_suspend_cb_t suspend;
	dwc_sleep_cb_t sleep;
	dwc_resume_cb_t resume;
	dwc_reset_cb_t reset;
	dwc_hnp_params_changed_cb_t hnp_changed;
	cfi_setup_cb_t cfi_setup;
};
/** @} */

/** @name Function Driver Functions */
/** @{ */

/** Call this function to get pointer on dwc_otg_pcd_t,
 * this pointer will be used for all PCD API functions.
 *
 * @param core_if The DWC_OTG Core
 */
extern dwc_otg_pcd_t *dwc_otg_pcd_init(dwc_otg_core_if_t * core_if);


/** Call this to bind the function driver to the PCD Core.
 *
 * @param pcd Pointer on dwc_otg_pcd_t returned by dwc_otg_pcd_init function.
 * @param fops The Function Driver Ops data structure containing pointers to all callbacks.
 */
extern void dwc_otg_pcd_start(dwc_otg_pcd_t * pcd,
				 const struct dwc_otg_pcd_function_ops * fops);

/** Enables an endpoint for use.  This function enables an endpoint in
 * the PCD.	 The endpoint is described by the ep_desc which has the
 * same format as a USB ep descriptor.	The ep_handle parameter is used to refer
 * to the endpoint from other API functions and in callbacks.  Normally this
 * should be called after a SET_CONFIGURATION/SET_INTERFACE to configure the
 * core for that interface.
 *
 * Returns -DWC_E_INVALID if invalid parameters were passed.
 * Returns -DWC_E_SHUTDOWN if any other error ocurred.
 * Returns 0 on success.
 *
 * @param pcd The PCD
 * @param ep_desc Endpoint descriptor
 * @param ep_handle Handle on endpoint, that will be used to identify endpoint.
 */
extern int32_t dwc_otg_pcd_ep_enable(dwc_otg_pcd_t * pcd,
				 const uint8_t * ep_desc, void * usb_ep);

/** Disable the endpoint referenced by ep_handle.
 *
 * Returns -DWC_E_INVALID if invalid parameters were passed.
 * Returns -DWC_E_SHUTDOWN if any other error occurred.
 * Returns 0 on success. */
extern int32_t dwc_otg_pcd_ep_disable(dwc_otg_pcd_t * pcd, void const * ep_handle);

/** Queue a data transfer request on the endpoint referenced by ep_handle.
 * After the transfer is completes, the complete callback will be called with
 * the request status.
 *
 * @param pcd The PCD
 * @param ep_handle The handle of the endpoint
 * @param buf The buffer for the data
 * @param dma_buf The DMA buffer for the data
 * @param buflen The length of the data transfer
 * @param zero Specifies whether to send zero length last packet.
 * @param req_handle Set this handle to any value to use to reference this
 * request in the ep_dequeue function or from the complete callback
 *
 * Returns -DWC_E_INVALID if invalid parameters were passed.
 * Returns -DWC_E_SHUTDOWN if any other error ocurred.
 * Returns 0 on success. */
extern int32_t dwc_otg_pcd_ep_queue(dwc_otg_pcd_t * pcd, void const * ep_handle,
				uint8_t * buf, dwc_dma_t dma_buf,
				uint32_t buflen, int32_t zero, void * req_handle);


/** This function should be called on every hardware interrupt */
extern int32_t dwc_otg_pcd_handle_intr(dwc_otg_pcd_t * pcd);

/** This function returns current frame number */
extern uint32_t dwc_otg_pcd_get_frame_number(dwc_otg_pcd_t const * pcd);

/** This function returns 1 if LPM support is enabled, and 0 otherwise. */
extern int32_t dwc_otg_pcd_is_lpm_enabled(dwc_otg_pcd_t const * pcd);

/** This function returns 1 if LPM Errata support is enabled, and 0 otherwise. */
extern int32_t dwc_otg_pcd_is_besl_enabled(dwc_otg_pcd_t const * pcd);

/** This function returns baseline_besl module parametr. */
extern int32_t dwc_otg_pcd_get_param_baseline_besl(dwc_otg_pcd_t const * pcd);

/** This function returns deep_besl module parametr. */
extern int32_t dwc_otg_pcd_get_param_deep_besl(dwc_otg_pcd_t const * pcd);

/** This function returns 1 if remote wakeup is allowed and 0, otherwise. */
extern int32_t dwc_otg_pcd_get_rmwkup_enable(dwc_otg_pcd_t const * pcd);

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
/** This function starts the SRP Protocol if no session is in progress. If
 * a session is already in progress, but the device is suspended,
 * remote wakeup signaling is started.
 */
extern int32_t dwc_otg_pcd_wakeup(dwc_otg_pcd_t const * pcd);

/** Initiate SRP */
extern void dwc_otg_pcd_initiate_srp(dwc_otg_pcd_t const * pcd);

/** Starts remote wakeup signaling. */
extern void dwc_otg_pcd_remote_wakeup(dwc_otg_pcd_t const * pcd, int32_t set);

/** Starts micorsecond soft disconnect. */
extern void dwc_otg_pcd_disconnect_us(dwc_otg_pcd_t const * pcd, uint32_t no_of_usecs);

/** These functions allow to get hnp parameters */
extern uint32_t get_b_hnp_enable(dwc_otg_pcd_t const * pcd);
extern uint32_t get_a_hnp_support(dwc_otg_pcd_t const * pcd);
extern uint32_t get_a_alt_hnp_support(dwc_otg_pcd_t const * pcd);
#endif
/******************************************************************************/

/** @} */

#endif 

