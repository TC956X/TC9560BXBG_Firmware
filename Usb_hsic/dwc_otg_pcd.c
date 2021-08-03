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
 *      18 July 2016 : 
 */

/* ==========================================================================
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_pcd.c $
 * $Revision: 1.12 $
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

/** @file
 * This file implements PCD Core. All code in this file is portable and doesn't
 * use any OS specific functions.
 * PCD Core provides Interface, defined in <code><dwc_otg_pcd_if.h></code>
 * header file, which can be used to implement OS specific PCD interface.
 *
 * An important function of the PCD is managing interrupts generated
 * by the DWC_otg controller. The implementation of the DWC_otg device
 * mode interrupt service routines is in dwc_otg_pcd_intr.c.
 *
 * @todo Add Device Mode test modes (Test J mode, Test K mode, etc).
 * @todo Does it work when the request size is greater than DEPTSIZ
 * transfer size
 *
 */
#include "dwc_os.h"
#include "dwc_otg_pcd.h"
#include "dwc_otg_driver.h"
#include "dwc_otg_dbg.h"

/**
 * Choose endpoint from ep arrays using usb_ep structure.
 */
static dwc_otg_pcd_ep_t *get_ep_from_handle(dwc_otg_pcd_t * pcd, void const *handle)
{
	dwc_otg_pcd_ep_t *retval = NULL;
	int32_t flag = 0;
	int32_t i;
	if (pcd->ep0.priv == handle) {
		retval = &pcd->ep0;
	} else {
		for (i = 0; i < (MAX_EPS_CHANNELS - 1); i++) {
			if (pcd->in_ep[i].priv == handle) {
				retval = &pcd->in_ep[i];
				flag = 1;
			} else {
				if (pcd->out_ep[i].priv == handle) {
					retval = &pcd->out_ep[i];
					flag = 1;
				}
			}
			if (flag == 1){
				break;
			}
		}
	}
	return retval;
}

/**
 * This function completes a request.  It call's the request call back.
 */
void dwc_otg_request_done(dwc_otg_pcd_ep_t * ep, dwc_otg_pcd_request_t * req, int32_t status)
{
	USIGN uint_stopped = ep->stopped;

	DBG_USB_Print(DBG_PCDV, "%s(ep %p req %p)\n", __func__, ep, req);
	DWC_CIRCLEQ_REMOVE_INIT(&ep->queue, req, queue_entry);

	/* don't modify queue heads during completion callback */
	ep->stopped = 1;
	/* spin_unlock/spin_lock now done in fops->complete() */
	ep->pcd->fops->complete(ep->pcd, ep->priv, req->priv, status,
				req->actual);

	if (ep->pcd->request_pending > 0) {
		--ep->pcd->request_pending;
	}

	ep->stopped = uint_stopped;
	DWC_FREE(req);
}

/**
 * This function terminates all the requsts in the EP request queue.
 */
void dwc_otg_request_nuke(dwc_otg_pcd_ep_t * ep)
{
	dwc_otg_pcd_request_t *ptr_req;

	ep->stopped = 1;
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	/* called with irqs blocked?? */
	while (!DWC_CIRCLEQ_EMPTY(&ep->queue)) {
		ptr_req = DWC_CIRCLEQ_FIRST(&ep->queue);
		dwc_otg_request_done(ep, ptr_req, -DWC_E_SHUTDOWN);
	}
}

void dwc_otg_pcd_start(dwc_otg_pcd_t * pcd,
		       const struct dwc_otg_pcd_function_ops *fops)
{
	pcd->fops = fops;
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
/**
 * PCD Callback function for initializing the PCD when switching to
 * device mode.
 *
 * @param p void pointer to the <code>dwc_otg_pcd_t</code>
 */
static int32_t dwc_otg_pcd_start_cb(void *p)
{
	dwc_otg_pcd_t *ptr_pcd = (dwc_otg_pcd_t *) p;
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(ptr_pcd);

	/*
	 * Initialized the Core for Device mode.
	 */
		dwc_otg_core_dev_init(ptr_core_if);
		/* Set core_if's lock pointer to the pcd->lock */
		/* core_if->lock = pcd->lock; */
	return 1;
}

/**
 * PCD Callback function for notifying the PCD when resuming from
 * suspend.
 *
 * @param p void pointer to the <code>dwc_otg_pcd_t</code>
 */
static int32_t dwc_otg_pcd_resume_cb(void *p)
{
	dwc_otg_pcd_t *ptr_pcd = (dwc_otg_pcd_t *) p;

	if (ptr_pcd->fops->resume != 0) {
		ptr_pcd->fops->resume(ptr_pcd);
	}

	/* Stop the SRP timeout timer. */
	if ((GET_CORE_IF(ptr_pcd)->core_params->phy_type != DWC_PHY_TYPE_PARAM_FS)
	    || (!GET_CORE_IF(ptr_pcd)->core_params->i2c_enable)) {
		if (GET_CORE_IF(ptr_pcd)->srp_timer_started != 0) {
			GET_CORE_IF(ptr_pcd)->srp_timer_started = 0;
		}
	}
	return 1;
}

#endif
/**
 * PCD Callback function for notifying the PCD device is suspended.
 *
 * @param p void pointer to the <code>dwc_otg_pcd_t</code>
 */
static int32_t dwc_otg_pcd_suspend_cb(void *p)
{
	dwc_otg_pcd_t *ptr_pcd = (dwc_otg_pcd_t *) p;

	if (ptr_pcd->fops->suspend != 0) {
		ptr_pcd->fops->suspend(ptr_pcd);
	}

	return 1;
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
/**
 * PCD Callback function for stopping the PCD when switching to Host
 * mode.
 *
 * @param p void pointer to the <code>dwc_otg_pcd_t</code>
 */
static int32_t dwc_otg_pcd_stop_cb(void *p)
{
	dwc_otg_pcd_t *ptr_pcd = (dwc_otg_pcd_t *) p;
	dwc_otg_pcd_stop(ptr_pcd);
	return 1;
}

#endif
/**
 * This function allocates a DMA Descriptor chain for the Endpoint
 * buffer to be used for a transfer to/from the specified endpoint.
 */
dwc_otg_dev_dma_desc_t *dwc_otg_ep_alloc_desc_chain(dwc_dma_t * dma_desc_addr,
						    uint32_t count)
{
	*dma_desc_addr = (dwc_dma_t)DWC_ALLOC(count * sizeof(dwc_otg_dev_dma_desc_t));
	return (dwc_otg_dev_dma_desc_t*)*dma_desc_addr;
}

/**
 * This function frees a DMA Descriptor chain that was allocated by ep_alloc_desc.
 */
void dwc_otg_ep_free_desc_chain(dwc_otg_dev_dma_desc_t * desc_addr,
				uint32_t dma_desc_addr, uint32_t count)
{
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	DWC_FREE(desc_addr);
	(void)count;
	/*(void)desc_addr;*/
	(void)dma_desc_addr;
}

static void dwc_otg_pcd_init_ep(dwc_otg_pcd_t * pcd, dwc_otg_pcd_ep_t * pcd_ep,
				uint32_t is_in, uint32_t ep_num)
{
	/* Init EP structure */
	pcd_ep->desc = 0;
	pcd_ep->pcd = pcd;
	pcd_ep->stopped = 1;
	pcd_ep->queue_sof = 0;

	/* Init DWC ep structure */
	pcd_ep->dwc_ep.is_in = is_in;
	pcd_ep->dwc_ep.num = (uint8_t)ep_num;
	pcd_ep->dwc_ep.active = 0;
	pcd_ep->dwc_ep.tx_fifo_num = 0;
	/* Control until ep is actvated */
	pcd_ep->dwc_ep.type = DWC_OTG_EP_TYPE_CONTROL;
	pcd_ep->dwc_ep.maxpacket = MAX_PACKET_SIZE;
	pcd_ep->dwc_ep.dma_addr = 0;
	pcd_ep->dwc_ep.start_xfer_buff = 0;
	pcd_ep->dwc_ep.xfer_buff = 0;
	pcd_ep->dwc_ep.xfer_len = 0;
	pcd_ep->dwc_ep.xfer_count = 0;
	pcd_ep->dwc_ep.sent_zlp = 0;
	pcd_ep->dwc_ep.total_len = 0;
	pcd_ep->dwc_ep.desc_addr = 0;
	pcd_ep->dwc_ep.dma_desc_addr = 0;
	DWC_CIRCLEQ_INIT(&pcd_ep->queue);
}

/**
 * Initialize ep's
 */
static void dwc_otg_pcd_reinit(dwc_otg_pcd_t * pcd)
{
	uint32_t i;
	uint32_t hw_cfg1;
	dwc_otg_pcd_ep_t *ptr_ep;
	uint32_t in_ep_cntr, out_ep_cntr;
	uint32_t uint_num_in_eps = (GET_CORE_IF(pcd))->dev_if->num_in_eps;
	uint32_t uint_num_out_eps = (GET_CORE_IF(pcd))->dev_if->num_out_eps;

	/**
	 * Initialize the EP0 structure.
	 */
	ptr_ep = &pcd->ep0;
	dwc_otg_pcd_init_ep(pcd, ptr_ep, 0, 0);

	in_ep_cntr = 0;
	hw_cfg1 = (GET_CORE_IF(pcd))->hwcfg1.d32 >> 3;
	for (i = 1; in_ep_cntr < uint_num_in_eps; i++) {
		if ((hw_cfg1 & 0x1) == 0) {
			dwc_otg_pcd_ep_t *str_ptr_ep = &pcd->in_ep[in_ep_cntr];
			in_ep_cntr++;
			/**
			 * @todo NGS: Add direction to EP, based on contents
			 * of HWCFG1.  Need a copy of HWCFG1 in pcd structure?
			 * sprintf(";r
			 */
			dwc_otg_pcd_init_ep(pcd, str_ptr_ep, 1 /* IN */ , i);

			DWC_CIRCLEQ_INIT(&str_ptr_ep->queue);
		}
		hw_cfg1 >>= 2;
	}

	out_ep_cntr = 0;
	hw_cfg1 = (GET_CORE_IF(pcd))->hwcfg1.d32 >> 2;
	for (i = 1; out_ep_cntr < uint_num_out_eps; i++) {
		if ((hw_cfg1 & 0x1) == 0) {
			dwc_otg_pcd_ep_t *str_ptr_ep = &pcd->out_ep[out_ep_cntr];
			out_ep_cntr++;
			/**
			 * @todo NGS: Add direction to EP, based on contents
			 * of HWCFG1.  Need a copy of HWCFG1 in pcd structure?
			 * sprintf(";r
			 */
			dwc_otg_pcd_init_ep(pcd, str_ptr_ep, 0 /* OUT */ , i);
			DWC_CIRCLEQ_INIT(&str_ptr_ep->queue);
		}
		hw_cfg1 >>= 2;
	}

	pcd->ep0state = EP0_DISCONNECT;
	pcd->ep0.dwc_ep.maxpacket = MAX_EP0_SIZE;
	pcd->ep0.dwc_ep.type = DWC_OTG_EP_TYPE_CONTROL;
}


/**
 * This function initialized the PCD portion of the driver.
 *
 */
dwc_otg_pcd_t *dwc_otg_pcd_init(dwc_otg_core_if_t * core_if)
{
	dwc_otg_pcd_t *ptr_pcd = NULL;
	dwc_otg_dev_if_t *ptr_dev_if;
	dwc_otg_pcd_t *retval;
	uint32_t do_pcd_reinit_flag = 1;
#if 1
	/**** Uses only in this function*********/
	static dwc_otg_cil_callbacks_t pcd_callbacks;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	pcd_callbacks.start = dwc_otg_pcd_start_cb;
	pcd_callbacks.stop = dwc_otg_pcd_stop_cb;
#endif
	pcd_callbacks.suspend = dwc_otg_pcd_suspend_cb;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	pcd_callbacks.resume_wakeup = dwc_otg_pcd_resume_cb;
#endif
	pcd_callbacks.p = NULL;			/* Set at registration */
	pcd_callbacks.disconnect = NULL;
#ifdef CONFIG_USB_DWC_OTG_LPM
	pcd_callbacks.sleep = NULL;
#endif
	pcd_callbacks.session_start = NULL;
	/*************************************************/
#endif
	/*
	 * Allocate PCD structure
	 */
	ptr_pcd = DWC_ALLOC(sizeof(dwc_otg_pcd_t));
	if (ptr_pcd == NULL) {
		retval = NULL;
	}else {
		ptr_pcd->core_if = core_if;
		ptr_dev_if = core_if->dev_if;
		ptr_pcd->otg_dev = DWC_ALLOC(sizeof(dwc_otg_device_t));
		
		/*
		 * Initialized the Core for Device mode here if there is nod ADP support. 
		 * Otherwise it will be done later in dwc_otg_adp_start routine.
		 */
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		if (!core_if->adp_enable) {
			dwc_otg_core_dev_init(core_if);   /* verify that all the core params match neu */
		}
#endif
		/*
		 * Register the PCD Callbacks.
		 */
		dwc_otg_cil_register_pcd_callbacks(core_if, &pcd_callbacks, ptr_pcd);

		/*
		 * Initialize the DMA buffer for SETUP packets
		 */
		if (GET_CORE_IF(ptr_pcd)->dma_enable  != 0) {
			ptr_pcd->setup_pkt = DWC_ALLOC(sizeof(*ptr_pcd->setup_pkt) * 5);	
			ptr_pcd->setup_pkt_dma_handle = (dwc_dma_t)ptr_pcd->setup_pkt;
			if (ptr_pcd->setup_pkt == NULL) {
				DWC_FREE(ptr_pcd);
				retval = NULL;
				do_pcd_reinit_flag = 0;
			}else {
				ptr_pcd->status_buf = (uint16_t *)  DWC_ALLOC(sizeof(uint16_t));
				ptr_pcd->status_buf_dma_handle = (dwc_dma_t)ptr_pcd->status_buf;
				if (ptr_pcd->status_buf == NULL) {
					DWC_FREE(ptr_pcd->setup_pkt);
					DWC_FREE(ptr_pcd);
					retval = NULL;
					do_pcd_reinit_flag = 0;
				}else {
					if (GET_CORE_IF(ptr_pcd)->dma_desc_enable != 0) {
						ptr_dev_if->setup_desc_addr[0] =
							dwc_otg_ep_alloc_desc_chain
							(&ptr_dev_if->dma_setup_desc_addr[0], 1);
						ptr_dev_if->setup_desc_addr[1] =
							dwc_otg_ep_alloc_desc_chain
							(&ptr_dev_if->dma_setup_desc_addr[1], 1);
						ptr_dev_if->in_desc_addr =
							dwc_otg_ep_alloc_desc_chain
							(&ptr_dev_if->dma_in_desc_addr, 1);
						ptr_dev_if->out_desc_addr =
							dwc_otg_ep_alloc_desc_chain
							(&ptr_dev_if->dma_out_desc_addr, 1);
						ptr_pcd->data_terminated = 0;
						if ((ptr_dev_if->setup_desc_addr[0] == 0)
							|| (ptr_dev_if->setup_desc_addr[1] == 0)
							|| (ptr_dev_if->in_desc_addr == 0)
							|| (ptr_dev_if->out_desc_addr == 0)) {

							if (ptr_dev_if->out_desc_addr != 0) {
								dwc_otg_ep_free_desc_chain
									(ptr_dev_if->out_desc_addr,
									 ptr_dev_if->dma_out_desc_addr, 1);
							}
							if (ptr_dev_if->in_desc_addr != 0) {
								dwc_otg_ep_free_desc_chain
									(ptr_dev_if->in_desc_addr,
									 ptr_dev_if->dma_in_desc_addr, 1);
							}
							if (ptr_dev_if->setup_desc_addr[1] != 0) {
								dwc_otg_ep_free_desc_chain
									(ptr_dev_if->setup_desc_addr[1],
									 ptr_dev_if->dma_setup_desc_addr[1], 1);
							}
							if (ptr_dev_if->setup_desc_addr[0] != 0) {
								dwc_otg_ep_free_desc_chain
									(ptr_dev_if->setup_desc_addr[0],
									 ptr_dev_if->dma_setup_desc_addr[0], 1);
							}
							DWC_FREE(ptr_pcd->setup_pkt);
							DWC_FREE(ptr_pcd->status_buf);

							DWC_FREE(ptr_pcd);

							retval = NULL;
							do_pcd_reinit_flag = 0;
						}
					}
				}
			}
		}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
        else {
			ptr_pcd->setup_pkt = DWC_ALLOC(sizeof(*ptr_pcd->setup_pkt) * 5);
			if (ptr_pcd->setup_pkt == NULL) {
				DWC_FREE(ptr_pcd);
				retval = NULL;
				do_pcd_reinit_flag = 0;
			}else {
				ptr_pcd->status_buf = DWC_ALLOC(sizeof(uint16_t));
				if (ptr_pcd->status_buf == NULL) {
					DWC_FREE(ptr_pcd->setup_pkt);
					DWC_FREE(ptr_pcd);
					retval = NULL;
					do_pcd_reinit_flag = 0;
				}
			}
		}
#endif
		
		if(do_pcd_reinit_flag == 1)
		{
			dwc_otg_pcd_reinit(ptr_pcd);
			retval = ptr_pcd;
		}
	}
	return retval;
}


/**
 * This function assigns periodic Tx FIFO to an periodic EP
 * in shared Tx FIFO mode
 */
static int32_t assign_tx_fifo(dwc_otg_core_if_t * core_if)
{
	uint32_t TxMsk = 1;
	int32_t i;
	int32_t retval = 0;

	for (i = 0; i < core_if->hwcfg4.b.num_in_eps; ++i) {
		if ((TxMsk & core_if->tx_msk) == 0) {
			core_if->tx_msk |= TxMsk;
			retval = i + 1;
			break;
		}else {
			TxMsk <<= 1;
		}
	}
	return retval;
}

/**
 * This function releases periodic Tx FIFO
 * in shared Tx FIFO mode
 */
static void release_perio_tx_fifo(dwc_otg_core_if_t * core_if,
				  uint32_t fifo_num)
{
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	core_if->p_tx_msk =
	    (core_if->p_tx_msk & (1U << (fifo_num - 1))) ^ core_if->p_tx_msk;
}

/**
 * This function releases periodic Tx FIFO
 * in shared Tx FIFO mode
 */
static void release_tx_fifo(dwc_otg_core_if_t * core_if, uint32_t fifo_num)
{
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	core_if->tx_msk =
	    (core_if->tx_msk & (1U << (fifo_num - 1))) ^ core_if->tx_msk;
}

/**
 * This function is being called from gadget 
 * to enable PCD endpoint.
 */
int32_t dwc_otg_pcd_ep_enable(dwc_otg_pcd_t * pcd,
			  const uint8_t * ep_desc, void *usb_ep)
{
	int32_t  dir;
	uint32_t uint_num;
	dwc_otg_pcd_ep_t *ptr_ep = NULL;
	const usb_endpoint_descriptor_t *ptr_desc;
	gdfifocfg_data_t gbl_dfifo_cfg;
	gdfifocfg_data_t gdfifocfgbase ;
	int32_t retval = 0;
	uint32_t i, epcount;
	fifosize_data_t dptxfsiz  ;
	gbl_dfifo_cfg.d32 = 0 ;
	gdfifocfgbase.d32 = 0 ;
	dptxfsiz.d32 = 0 ;
	
	ptr_desc = (const usb_endpoint_descriptor_t *)ep_desc;

	if (!ptr_desc) {
		pcd->ep0.priv = usb_ep;
		ptr_ep = &pcd->ep0;
		retval = -DWC_E_INVALID;
		goto out_ep_en;
	}

	uint_num = UE_GET_ADDR(ptr_desc->bEndpointAddress);
	dir = UE_GET_DIR(ptr_desc->bEndpointAddress);

	if (!ptr_desc->wMaxPacketSize) {
		DBG_Warn_Print("bad maxpacketsize\n");
		retval = -DWC_E_INVALID;
		goto out_ep_en;
	}

	if (dir == UE_DIR_IN) {
		epcount = pcd->core_if->dev_if->num_in_eps;
		for (i = 0; i < epcount; i++) {
			if (uint_num == pcd->in_ep[i].dwc_ep.num) {
				ptr_ep = &pcd->in_ep[i];
				break;
			}
		}
	} else {
		epcount = pcd->core_if->dev_if->num_out_eps;
		for (i = 0; i < epcount; i++) {
			if (uint_num == pcd->out_ep[i].dwc_ep.num) {
				ptr_ep = &pcd->out_ep[i];
				break;
			}
		}
	}

	if (!ptr_ep) {
		DBG_Warn_Print("bad address\n");
		retval = -DWC_E_INVALID;
		goto out_ep_en;
	}


	ptr_ep->desc = ptr_desc;
	ptr_ep->priv = usb_ep;

	/*
	 * Activate the EP
	 */
	ptr_ep->stopped = 0;

	ptr_ep->dwc_ep.is_in = (dir == UE_DIR_IN);
	ptr_ep->dwc_ep.maxpacket = UGETW(ptr_desc->wMaxPacketSize);

	ptr_ep->dwc_ep.type = ptr_desc->bmAttributes & UE_XFERTYPE;

	if (ptr_ep->dwc_ep.is_in != 0) {
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		if (!GET_CORE_IF(pcd)->en_multiple_tx_fifo) {
			ptr_ep->dwc_ep.tx_fifo_num = 0;
		} else {
#endif
			/*
			 * if Dedicated FIFOs mode is on then assign a Tx FIFO.
			 */
			ptr_ep->dwc_ep.tx_fifo_num =
			    assign_tx_fifo(GET_CORE_IF(pcd));
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		}
#endif
		/* Calculating EP info controller base address */
		if (((ptr_ep->dwc_ep).tx_fifo_num)
		    && (GET_CORE_IF(pcd)->en_multiple_tx_fifo)) {
			gbl_dfifo_cfg.d32 =
			    REG_RD(&GET_CORE_IF(pcd)->
					   core_global_regs->gdfifocfg);
			gdfifocfgbase.d32 = gbl_dfifo_cfg.d32 >> 16;
			dptxfsiz.d32 =
			    (REG_RD
			     (&GET_CORE_IF(pcd)->core_global_regs->
			      dtxfsiz[ptr_ep->dwc_ep.tx_fifo_num - 1]) >> 16);
			gbl_dfifo_cfg.b.epinfobase =
			    gdfifocfgbase.d32 + dptxfsiz.d32;
			if (GET_CORE_IF(pcd)->snpsid <= OTG_CORE_REV_2_94a) {
				REG_WR(&GET_CORE_IF(pcd)->
						core_global_regs->gdfifocfg,
						gbl_dfifo_cfg.d32);
			}
		}
	}
	/* Set initial data PID. */
	if (ptr_ep->dwc_ep.type == UE_BULK) {
		ptr_ep->dwc_ep.data_pid_start = 0;
	}

	/* Alloc DMA Descriptors */
	if (GET_CORE_IF(pcd)->dma_desc_enable != 0) {
			ptr_ep->dwc_ep.desc_addr =
			    dwc_otg_ep_alloc_desc_chain(&ptr_ep->
							dwc_ep.dma_desc_addr,
							MAX_DMA_DESC_CNT);
			if (!ptr_ep->dwc_ep.desc_addr) {
				DBG_Warn_Print("%s, can't allocate DMA descriptor\n",
					 __func__);
				retval = -DWC_E_SHUTDOWN;
				goto out_ep_en;
			}
	}

	DBG_USB_Print(DBG_PCD, "Activate %s: type=%d, mps=%d desc=%p\n",
		    (ptr_ep->dwc_ep.is_in ? "IN" : "OUT"),
		    ptr_ep->dwc_ep.type, ptr_ep->dwc_ep.maxpacket, ptr_ep->desc);

	dwc_otg_ep_activate(GET_CORE_IF(pcd), &ptr_ep->dwc_ep);

out_ep_en:
	return retval;
}

/**
 * This function is being called from gadget 
 * to disable PCD endpoint.
 */
int32_t dwc_otg_pcd_ep_disable(dwc_otg_pcd_t * pcd, void const *ep_handle)
{
	int32_t retval = 0;
	dwc_otg_pcd_ep_t *ptr_ep;
	dwc_otg_dev_dma_desc_t *ptr_desc_addr;
	dwc_dma_t str_dma_desc_addr;
	gdfifocfg_data_t gdfifocfgbase ;
	gdfifocfg_data_t gbl_dfifo_cfg ;
	fifosize_data_t dptxfsiz  ;
	gdfifocfgbase.d32 = 0 ;
	gbl_dfifo_cfg.d32 = 0 ;
	dptxfsiz.d32 = 0 ;

	ptr_ep = get_ep_from_handle(pcd, ep_handle);

	if ((!(ptr_ep)) || (!(ptr_ep->desc))) {
		DBG_USB_Print(DBG_PCD, "bad ep address\n");
		retval = -DWC_E_INVALID;
	}
	else {
		dwc_otg_request_nuke(ptr_ep);

		dwc_otg_ep_deactivate(GET_CORE_IF(pcd), &ptr_ep->dwc_ep);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		if (pcd->core_if->core_params->dev_out_nak != 0) {
			pcd->core_if->ep_xfer_info[ptr_ep->dwc_ep.num].state = 0;
		}
#endif
		ptr_ep->desc = NULL;
		ptr_ep->stopped = 1;

		gbl_dfifo_cfg.d32 =
			REG_RD(&GET_CORE_IF(pcd)->core_global_regs->gdfifocfg);
		gdfifocfgbase.d32 = gbl_dfifo_cfg.d32 >> 16;
		if (ptr_ep->dwc_ep.is_in != 0) {
			if (GET_CORE_IF(pcd)->en_multiple_tx_fifo != 0) {
				/* Flush the Tx FIFO */
				dwc_otg_flush_tx_fifo(GET_CORE_IF(pcd),
							  ptr_ep->dwc_ep.tx_fifo_num);
			}
			release_perio_tx_fifo(GET_CORE_IF(pcd), ptr_ep->dwc_ep.tx_fifo_num);
			release_tx_fifo(GET_CORE_IF(pcd), ptr_ep->dwc_ep.tx_fifo_num);
			if (GET_CORE_IF(pcd)->en_multiple_tx_fifo != 0) {
				/* Decreasing EPinfo Base Addr */
				dptxfsiz.d32 =
					(REG_RD
					 (&GET_CORE_IF(pcd)->
						core_global_regs->dtxfsiz[ptr_ep->dwc_ep.tx_fifo_num-1]) >> 16);
				gbl_dfifo_cfg.b.epinfobase = gdfifocfgbase.d32 - dptxfsiz.d32;
				if (GET_CORE_IF(pcd)->snpsid <= OTG_CORE_REV_2_94a) {
					REG_WR(&GET_CORE_IF(pcd)->core_global_regs->gdfifocfg,
							gbl_dfifo_cfg.d32);
				}
			}
		}

		/* Free DMA Descriptors */
		if (GET_CORE_IF(pcd)->dma_desc_enable != 0) {
				ptr_desc_addr = ptr_ep->dwc_ep.desc_addr;
				str_dma_desc_addr = ptr_ep->dwc_ep.dma_desc_addr;

				/* Cannot call dma_free_coherent() with IRQs disabled */
				dwc_otg_ep_free_desc_chain(ptr_desc_addr, str_dma_desc_addr,
							   MAX_DMA_DESC_CNT);
		}
		DBG_USB_Print(DBG_PCD, "%d %s disabled\n", ptr_ep->dwc_ep.num,
				ptr_ep->dwc_ep.is_in ? "IN" : "OUT");

	}
	return retval;
}

int32_t dwc_otg_pcd_ep_queue(dwc_otg_pcd_t * pcd, void const *ep_handle,
			 uint8_t * buf, dwc_dma_t dma_buf, uint32_t buflen,
			 int32_t zero, void *req_handle)
{
	dwc_otg_pcd_request_t *ptr_req;
	dwc_otg_pcd_ep_t *ptr_ep;    /*endpoint struct */
	uint32_t max_transfer;
	int32_t retval = 0;
	dwc_dma_t j;
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	ptr_ep = get_ep_from_handle(pcd, ep_handle);	/*Choose endpoint from ep arrays using usb_ep structure */
	if ((!(ptr_ep)) || ((!(ptr_ep->desc)) && (ptr_ep->dwc_ep.num != 0))) {				
		DBG_Warn_Print("bad ep\n");
		retval = -DWC_E_INVALID;
	}
	else {
			ptr_req = DWC_ALLOC(sizeof(*ptr_req));

		if (!ptr_req) {
			retval = -DWC_E_NO_MEMORY;
		}
		else {
			DWC_CIRCLEQ_INIT_ENTRY(ptr_req, queue_entry);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			if (!GET_CORE_IF(pcd)->core_params->opt) {
				if (ptr_ep->dwc_ep.num != 0) {
					DBG_Error_Print("queue req %p, len %d buf %p\n", req_handle, buflen, buf);
				}
			}
#endif
			ptr_req->buf = buf;
			ptr_req->dma = dma_buf;
			ptr_req->length = buflen;
			ptr_req->sent_zlp = zero;
			ptr_req->priv = req_handle;
			ptr_req->dw_align_buf = NULL;
			j = dma_buf & 0x3;
			if (((j) && (GET_CORE_IF(pcd)->dma_enable))
				&& (!(GET_CORE_IF(pcd)->dma_desc_enable)))
			{
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
				DBG_USB_Print(DBG_USB,"inside if dma_buf\n");
				ptr_req->dw_align_buf = (uint8_t *)DWC_ALLOC(buflen);	
			ptr_req->dw_align_buf_dma = (dwc_dma_t)ptr_req->dw_align_buf;
#endif
            }

			/*
			 * For EP0 IN without premature status, zlp is required?
			 */
			if ((ptr_ep->dwc_ep.num == 0) && (ptr_ep->dwc_ep.is_in)) {
				DBG_USB_Print(DBG_PCDV, "%d-OUT ZLP\n", ptr_ep->dwc_ep.num);
			}

			/* Start the transfer */
			if ((DWC_CIRCLEQ_EMPTY(&ptr_ep->queue)) && (!(ptr_ep->stopped))) {
				/* EP0 Transfer? */
				if (ptr_ep->dwc_ep.num == 0) {
					switch (pcd->ep0state) {
					case EP0_IN_DATA_PHASE:
						DBG_USB_Print(DBG_PCD,
								"%s ep0: EP0_IN_DATA_PHASE\n",
								__func__);
						break;

					case EP0_OUT_DATA_PHASE:
						if (pcd->request_config != 0) {
							/* Complete STATUS PHASE */
							ptr_ep->dwc_ep.is_in = 1;
							pcd->ep0state = EP0_IN_STATUS_PHASE;
						}
						break;

					case EP0_IN_STATUS_PHASE:
						DBG_USB_Print(DBG_PCD,
								"%s ep0: EP0_IN_STATUS_PHASE\n",
								__func__);
						break;

					default:
						DBG_USB_Print(DBG_USB, "ep0: odd state %d\n",
								pcd->ep0state);
						retval = -DWC_E_SHUTDOWN;
					}
					if (retval != -DWC_E_SHUTDOWN) {
						ptr_ep->dwc_ep.dma_addr = dma_buf;
						ptr_ep->dwc_ep.start_xfer_buff = buf;
						ptr_ep->dwc_ep.xfer_buff = buf;
						ptr_ep->dwc_ep.xfer_len = buflen;
						ptr_ep->dwc_ep.xfer_count = 0;
						ptr_ep->dwc_ep.sent_zlp = 0;
						ptr_ep->dwc_ep.total_len = ptr_ep->dwc_ep.xfer_len;

						if (zero != 0) {
							if (((ptr_ep->dwc_ep.xfer_len %
								 ptr_ep->dwc_ep.maxpacket) == 0)
								&& (ptr_ep->dwc_ep.xfer_len != 0)) {
								ptr_ep->dwc_ep.sent_zlp = 1;
							}

						}

						dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd),
									   &ptr_ep->dwc_ep);
					}
				}		/* non-ep0 endpoints */
				else {
					max_transfer =
						GET_CORE_IF(ptr_ep->pcd)->core_params->
						max_transfer_size;
							/* Setup and start the Transfer */
					if (ptr_req->dw_align_buf != 0) {
						if (ptr_ep->dwc_ep.is_in != 0) {
							dwc_memcpy(ptr_req->dw_align_buf,buf, buflen);
						}
						ptr_ep->dwc_ep.dma_addr = ptr_req->dw_align_buf_dma;
						ptr_ep->dwc_ep.start_xfer_buff = ptr_req->dw_align_buf;
						ptr_ep->dwc_ep.xfer_buff = ptr_req->dw_align_buf;
					} else {  /* FLOW */
						ptr_ep->dwc_ep.dma_addr = dma_buf;
						ptr_ep->dwc_ep.start_xfer_buff = buf;
						ptr_ep->dwc_ep.xfer_buff = buf;
					}
					ptr_ep->dwc_ep.xfer_len = 0;
					ptr_ep->dwc_ep.xfer_count = 0;
					ptr_ep->dwc_ep.sent_zlp = 0;
					ptr_ep->dwc_ep.total_len = buflen;

					ptr_ep->dwc_ep.maxxfer = max_transfer;
					if (GET_CORE_IF(pcd)->dma_desc_enable != 0) {
						uint32_t out_max_xfer =
							DDMA_MAX_TRANSFER_SIZE -
							(DDMA_MAX_TRANSFER_SIZE % 4);
						if (ptr_ep->dwc_ep.is_in != 0) {
							if (ptr_ep->dwc_ep.maxxfer >
								DDMA_MAX_TRANSFER_SIZE) {
								ptr_ep->dwc_ep.maxxfer =
									DDMA_MAX_TRANSFER_SIZE;
							}
						} else {
							if (ptr_ep->dwc_ep.maxxfer >
								out_max_xfer) {
								ptr_ep->dwc_ep.maxxfer =
									out_max_xfer;
							}
						}
					}
					if (ptr_ep->dwc_ep.maxxfer < ptr_ep->dwc_ep.total_len) {
						ptr_ep->dwc_ep.maxxfer -=
							(ptr_ep->dwc_ep.maxxfer %
							 ptr_ep->dwc_ep.maxpacket);
					}

					if (zero != 0) {
						if (((ptr_ep->dwc_ep.total_len %
							 ptr_ep->dwc_ep.maxpacket) == 0)
							&& (ptr_ep->dwc_ep.total_len != 0)) {
							ptr_ep->dwc_ep.sent_zlp = 1;
						}
					}
					dwc_otg_ep_start_transfer(GET_CORE_IF(pcd),
							  &ptr_ep->dwc_ep);
				}
			}
			if(retval != -DWC_E_SHUTDOWN) {
				if (ptr_req != 0) {
					++pcd->request_pending;
					DWC_CIRCLEQ_INSERT_TAIL(&ptr_ep->queue, ptr_req, queue_entry);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
                    if (((ptr_ep->dwc_ep.is_in) && (ptr_ep->stopped))
						&& (!(GET_CORE_IF(pcd)->dma_enable))) {
						/** @todo NGS Create a function for this. */
						diepmsk_data_t dev_ep_msk; 
						dev_ep_msk.d32 = 0 ;
						dev_ep_msk.b.intktxfemp = 1;
						if (GET_CORE_IF(pcd)->multiproc_int_enable != 0) {
							DWC_MODIFY_REG32(&GET_CORE_IF(pcd)->
									 dev_if->dev_global_regs->diepeachintmsk
									 [ptr_ep->dwc_ep.num], 0,
									 dev_ep_msk.d32);
						} else {
							DWC_MODIFY_REG32(&GET_CORE_IF(pcd)->
									 dev_if->dev_global_regs->
									 diepmsk, 0, dev_ep_msk.d32);
						}
					}
#endif
				}
			}
		}
	}
	return retval;
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
/**
 * This function initiates remote wakeup of the host from suspend state.
 */
void dwc_otg_pcd_rem_wkup_from_suspend(dwc_otg_pcd_t const * pcd, int32_t set)
{
	dctl_data_t dev_ctl = { 0 };
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
	dsts_data_t dev_sts;

	dev_sts.d32 = REG_RD(&ptr_core_if->dev_if->dev_global_regs->dsts);
	if (!dev_sts.b.suspsts) {
		DBG_Warn_Print("Remote wakeup while is not in suspend state\n");
	}
	/* Check if DEVICE_REMOTE_WAKEUP feature enabled */
	if (pcd->remote_wakeup_enable != 0) {
		if (set != 0) {

			dev_ctl.b.rmtwkupsig = 1;
			DWC_MODIFY_REG32(&ptr_core_if->dev_if->dev_global_regs->
					 dctl, 0, dev_ctl.d32);
			DBG_USB_Print(DBG_PCD, "Set Remote Wakeup\n");

			dwc_mdelay(2);
			DWC_MODIFY_REG32(&ptr_core_if->dev_if->dev_global_regs->
					 dctl, dev_ctl.d32, 0);
			DBG_USB_Print(DBG_PCD, "Clear Remote Wakeup\n");
		}
	} else {
		DBG_USB_Print(DBG_PCD, "Remote Wakeup is disabled\n");
	}
}

#ifdef CONFIG_USB_DWC_OTG_LPM
/**
 * This function initiates remote wakeup of the host from L1 sleep state.
 */
void dwc_otg_pcd_rem_wkup_from_sleep(dwc_otg_pcd_t * pcd, int32_t set)
{
	glpmcfg_data_t lpmcfg;
	pcgcctl_data_t pcgcctl = {.d32 = 0 };
	
	dwc_otg_core_if_t *core_if = GET_CORE_IF(pcd);

	lpmcfg.d32 = REG_RD(&core_if->core_global_regs->glpmcfg);

	/* Check if we are in L1 state */
	if (!lpmcfg.b.prt_sleep_sts) {
		DBG_USB_Print(DBG_PCD, "Device is not in sleep state\n");
		return;
	}

	/* Check if host allows remote wakeup */
	if (!lpmcfg.b.rem_wkup_en) {
		DBG_USB_Print(DBG_PCD, "Host does not allow remote wakeup\n");
		return;
	}

	/* Check if Resume OK */
	if (!lpmcfg.b.sleep_state_resumeok) {
		DBG_USB_Print(DBG_PCD, "Sleep state resume is not OK\n");
		return;
	}

	lpmcfg.d32 = REG_RD(&core_if->core_global_regs->glpmcfg);
	lpmcfg.b.en_utmi_sleep = 0;
	lpmcfg.b.hird_thres &= (~(1 << 4));
	
	/* Clear Enbl_L1Gating bit. */
	pcgcctl.b.enbl_sleep_gating = 1;
	DWC_MODIFY_REG32(core_if->pcgcctl, pcgcctl.d32,0);
			
	REG_WR(&core_if->core_global_regs->glpmcfg, lpmcfg.d32);

	if (set) {
		dctl_data_t dctl = {.d32 = 0 };
		dctl.b.rmtwkupsig = 1;
		/* Set RmtWkUpSig bit to start remote wakup signaling.
		 * Hardware will automatically clear this bit.
		 */
		DWC_MODIFY_REG32(&core_if->dev_if->dev_global_regs->dctl,
				 0, dctl.d32);
		DBG_USB_Print(DBG_PCD, "Set Remote Wakeup\n");
	}

}
#endif

/**
 * Performs remote wakeup.
 */
void dwc_otg_pcd_remote_wakeup(dwc_otg_pcd_t const * pcd, int32_t set)
{
#ifdef CONFIG_USB_DWC_OTG_LPM
		if (core_if->lx_state == DWC_OTG_L1) {
			dwc_otg_pcd_rem_wkup_from_sleep(pcd, set);
		} else {
#endif
			dwc_otg_pcd_rem_wkup_from_suspend(pcd, set);
#ifdef CONFIG_USB_DWC_OTG_LPM
		}
#endif
	return;
}

void dwc_otg_pcd_disconnect_us(dwc_otg_pcd_t const * pcd, uint32_t no_of_usecs)
{
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
	dctl_data_t dev_ctl = { 0 };

		dev_ctl.b.sftdiscon = 1;
		DWC_MODIFY_REG32(&ptr_core_if->dev_if->dev_global_regs->dctl, 0, dev_ctl.d32);
		dwc_udelay(no_of_usecs);
		DWC_MODIFY_REG32(&ptr_core_if->dev_if->dev_global_regs->dctl, dev_ctl.d32,0);
	return;

}

int32_t dwc_otg_pcd_wakeup(dwc_otg_pcd_t const * pcd)
{
	dsts_data_t dev_sts;
	gotgctl_data_t gbl_otg_ctl;

	/*
	 * This function starts the Protocol if no session is in progress. If
	 * a session is already in progress, but the device is suspended,
	 * remote wakeup signaling is started.
	 */

	/* Check if valid session */
	gbl_otg_ctl.d32 =
	    REG_RD(&(GET_CORE_IF(pcd)->core_global_regs->gotgctl));
	if (gbl_otg_ctl.b.bsesvld != 0) {
		/* Check if suspend state */
		dev_sts.d32 =
		    REG_RD(&
				   (GET_CORE_IF(pcd)->dev_if->
				    dev_global_regs->dsts));
		if (dev_sts.b.suspsts != 0) {
			dwc_otg_pcd_remote_wakeup(pcd, 1);
		}
	} else {
		dwc_otg_pcd_initiate_srp(pcd);
	}

	return 0;

}

/**
 * Start the SRP timer to detect when the SRP does not complete within
 * 6 seconds.
 *
 * @param pcd the pcd structure.
 */
void dwc_otg_pcd_initiate_srp(dwc_otg_pcd_t const * pcd)
{
	dwc_otg_initiate_srp(GET_CORE_IF(pcd));
}

#endif
uint32_t dwc_otg_pcd_get_frame_number(dwc_otg_pcd_t const * pcd)
{
	return dwc_otg_get_frame_number(GET_CORE_IF(pcd));
}

int32_t dwc_otg_pcd_is_lpm_enabled(dwc_otg_pcd_t const * pcd)
{
	return GET_CORE_IF(pcd)->core_params->lpm_enable;
}

int32_t dwc_otg_pcd_is_besl_enabled(dwc_otg_pcd_t const * pcd)
{
	return GET_CORE_IF(pcd)->core_params->besl_enable;
}

int32_t dwc_otg_pcd_get_param_baseline_besl(dwc_otg_pcd_t const * pcd)
{
	return GET_CORE_IF(pcd)->core_params->baseline_besl;
}

int32_t dwc_otg_pcd_get_param_deep_besl(dwc_otg_pcd_t const * pcd)
{
	return GET_CORE_IF(pcd)->core_params->deep_besl;
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
uint32_t get_b_hnp_enable(dwc_otg_pcd_t const * pcd)
{
	return pcd->b_hnp_enable;
}

uint32_t get_a_hnp_support(dwc_otg_pcd_t const * pcd)
{
	return pcd->a_hnp_support;
}

uint32_t get_a_alt_hnp_support(dwc_otg_pcd_t const * pcd)
{
	return pcd->a_alt_hnp_support;
}
#endif

int32_t dwc_otg_pcd_get_rmwkup_enable(dwc_otg_pcd_t const * pcd)
{
	return pcd->remote_wakeup_enable;
}

