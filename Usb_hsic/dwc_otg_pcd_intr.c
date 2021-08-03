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
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_pcd_intr.c $
 * $Revision: 1.13 $
 * $Date: 2016/07/18 23:50:47 $
 * $Change: 2234037 $
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

#include  <tc9560_common.h>
#include "dwc_os.h"
#include "dwc_otg_pcd.h"
#include "tc9560_uart.h"
#include "tc9560_reg_define.h"

#include "tc9560_uart.h"
#include "dwc_otg_dbg.h"

#define ECONNRESET      131     /* Connection reset by peer */
uint8_t  neu_device_desc_1[] = { 
		0x12,                      /* bLength   */
    0x01,                       /* bDescriptorType */    
    0x00, 0x02,                 /* bcdUSB (lsb first) */    
    0xFF,                       /* bDeviceClass         */      
    0xFF,                       /* bDeviceSubClass */
    0x00,                       /* bDeviceProtocol */
    0x40,                       /* bMaxPacketSize0 */

	0x30, 0x09,					 					/* idVendor (lsb first)*/
	0x05, 0x17,					 					/* idProduct (lsb first)*/
	
    0x00, 0x01,                 /* bcdDevice (lsb first)*/
    0x00,                       /* iManufacturer        */
    0x00,                      /* iProduct             */

    0x00,                       /* iSerialNumber        */
    0x01,                       /* bNumConfigurations*/
	};

uint8_t neu_config_desc[] = {
    0x09,                       /* Length                  */
    0x02,                       /* Type                    */
    0x4A, 0x00,                 /* TotalLength (lsb first) */
    0x01,                       /* NumInterfaces                 */    
    0x01,                       /* bConfigurationValue            */   
    0x00,                       /* iConfiguration                 */   
    0x80,                       /* bmAttributes (no remote wakeup) */   
    0x0F,                       /* MaxPower (*2mA)               */
	
   /* Begin Descriptor: Interface0, Alternate0             */    
    0x09,                       /* bLength                  */     
    0x04,                       /* bDescriptorType            */   
    0x00,                       /* bInterfaceNumber          */    
    0x00,                       /* bAlternateSetting          */   
    0x08,                       /* bNumEndpoints              */   
    0xFF,                       /* bInterfaceClass           */    
    0xFF,                       /* bInterfaceSubClass          */  
    0x00,                       /* bInterfaceProcotol          */  
    0x00,                       /* iInterface                  */  
  /*    Begin Descriptor: Endpoint1, Interface0, Alternate0    */  
    0x07,                       /* bLength                      */ 
    0x05,                      /* bDescriptorType               */
    0x01,                      /* bEndpointAddress (ep1, OUT)    */
    0x02,                      /* bmAttributes (Bulk)           */
    0x00, 										 /* wMaxPacketSize (lsb first)*/
		0x02,				               /* wMaxPacketSize (lsb first)   */ 
    0x05,                      /* bInterval                        */

    /* Begin Descriptor: Endpoint2, Interface0, Alternate0      */
    0x07,                      /* bLength                       */
    0x05,                      /* bDescriptorType               */
    0x02,                      /* bEndpointAddress (ep2, OUT)    */
    0x02,                      /* bmAttributes (Bulk)           */
    0x00, 										 /* wMaxPacketSize (lsb first)*/
		0x02,				               /* wMaxPacketSize (lsb first)    */
    0x05,                      /* bInterval                       */ 

		
    /* Begin Descriptor: Endpoint3, Interface0, Alternate0      */
    0x07,                      /* bLength                       */
    0x05,                      /* bDescriptorType               */
    0x03,                      /* bEndpointAddress (ep3, OUT)   */
    0x02,                      /* bmAttributes (Bulk)            */
    0x00,											 /* wMaxPacketSize (lsb first)*/
		0x02,                      /* wMaxPacketSize (lsb first)    */
    0x05,                      /* bInterval                     */

    /* Begin Descriptor: Endpoint4, Interface0, Alternate0        */
    0x07,                      /* bLength                       */
    0x05,                      /* bDescriptorType               */
    0x04,                      /* bEndpointAddress (ep4, OUT)    */
    0x02,                      /* bmAttributes (Bulk)           */
    0x00, 										 /* wMaxPacketSize (lsb first)    */
		0x02,				               /* wMaxPacketSize (lsb first)    */
    0x05,                      /* bInterval                     */
    /* Begin Descriptor: Endpoint5, Interface0, Alternate0      */
    0x07,                      /* bLength                       */
    0x05,                      /* bDescriptorType                */
    0x85,                      /* bEndpointAddress (ep5, IN)   */
    0x02,                      /* bmAttributes (Bulk)           */
    0x00,                      /* wMaxPacketSize (lsb)         */
    0x02,                      /* wMaxPacketSize (msb)            */
    0x05,                      /* bInterval                     */

    /* Begin Descriptor: Endpoint6, Interface0, Alternate0        */
    0x07,                      /* bLength                       */
    0x05,                      /* bDescriptorType               */
    0x86,                      /* bEndpointAddress (ep6, IN)    */
    0x02,                      /* bmAttributes (Bulk)           */
    0x00, 										 /* wMaxPacketSize (lsb first)    */
		0x02,                			 /* wMaxPacketSize (lsb first)    */
    0x05,                      /* bInterval             */

    /* Begin Descriptor: Endpoint7, Interface0, Alternate0        */
    0x07,                      /* bLength                       */
    0x05,                      /* bDescriptorType               */
    0x87,                      /* bEndpointAddress (ep7, IN)    */
    0x02,                      /* bmAttributes (Bulk)           */
    0x00, 										 /* wMaxPacketSize (lsb first)    */
		0x02,                			 /* wMaxPacketSize (lsb first)    */
    0x05,                      /* bInterval             */

    /* Begin Descriptor: Endpoint8, Interface0, Alternate0        */
    0x07,                      /* bLength                       */
    0x05,                      /* bDescriptorType               */
    0x88,                      /* bEndpointAddress (ep8, IN)    */
    0x02,                      /* bmAttributes (Bulk)           */
    0x00, 										 /* wMaxPacketSize (lsb first)    */
		0x02,                			 /* wMaxPacketSize (lsb first)    */
    0x05,                      /* bInterval             */

};


uint8_t  neu_interface_desc[] = { 
    0x09,                      /* bLength                       */
    0x04,                      /* bDescriptorType               */
    0x00,                      /* bInterfaceNumber              */
    0x00,                      /* bAlternateSetting             */
    0x08,                      /* bNumEndpoints                 */
    0xFF,                      /* bInterfaceClass               */
    0xFF,                      /* bInterfaceSubClass            */

    0x00,                      /* bInterfaceProcotol            */
    0x00,                      /* iInterface                    */
};


uint8_t  neu_endpoint_desc[] = { 
    0x07,                      /* bLength                      */ 
    0x05,                      /* bDescriptorType              */ 
    0x01,                      /* bEndpointAddress (ep1, OUT)    */
    0x02,                      /* bmAttributes (Bulk)           */
    0x00, 0x04,                /* wMaxPacketSize (lsb first)    */

    0x05,                      /* bInterval                        */
	
};


/**
 * This function updates OTG.
 */
static void dwc_otg_pcd_update_otg(dwc_otg_pcd_t * pcd, const USIGN reset)
{

	if (reset != 0) {
		pcd->b_hnp_enable = 0;
		pcd->a_hnp_support = 0;
		pcd->a_alt_hnp_support = 0;
	}

	if (pcd->fops->hnp_changed != 0) {
		pcd->fops->hnp_changed(pcd);
	}
}

/** @file
 * This file contains the implementation of the PCD Interrupt handlers.
 *
 * The PCD handles the device interrupts.  Many conditions can cause a
 * device interrupt. When an interrupt occurs, the device interrupt
 * service routine determines the cause of the interrupt and
 * dispatches handling to the appropriate function. These interrupt
 * handling functions are described below.
 * All interrupt registers are processed from LSB to MSB.
 */

/**
 * This function returns pointer to in ep struct with number ep_num
 */
static dwc_otg_pcd_ep_t *get_in_ep(dwc_otg_pcd_t * pcd, uint32_t ep_num)
{
	dwc_otg_pcd_ep_t *retval = 0;
	uint32_t i;
	uint32_t int_num_in_eps = GET_CORE_IF(pcd)->dev_if->num_in_eps;
	if (ep_num == 0) {
		retval = &pcd->ep0;
	} else {
		for (i = 0; i < int_num_in_eps; ++i) {
			if (pcd->in_ep[i].dwc_ep.num == ep_num) {
				retval = &pcd->in_ep[i];
			}
		}
	}
	return retval;
}

/**
 * This function returns pointer to out ep struct with number ep_num
 */
static inline dwc_otg_pcd_ep_t *get_out_ep(dwc_otg_pcd_t * pcd, uint32_t ep_num)
{
	dwc_otg_pcd_ep_t *retval = 0;
	uint32_t i;
	uint32_t int_num_out_eps = GET_CORE_IF(pcd)->dev_if->num_out_eps;
	if (ep_num == 0) {
		retval = &pcd->ep0;
	} else {
		for (i = 0; i < int_num_out_eps; ++i) {
			if (pcd->out_ep[i].dwc_ep.num == ep_num) {
				retval = &pcd->out_ep[i];
			}
		}
	}
	return retval;
}

/**
 * This functions gets a pointer to an EP from the wIndex address
 * value of the control request.
 */
dwc_otg_pcd_ep_t *get_ep_by_addr(dwc_otg_pcd_t * pcd, uint16_t wIndex)
{
	dwc_otg_pcd_ep_t *ptr_ep;
	uint32_t uint_ep_num = UE_GET_ADDR(wIndex);

	if (uint_ep_num == 0) {
		ptr_ep = &pcd->ep0;
	} else if (UE_GET_DIR(wIndex) == UE_DIR_IN) {	/* in ep */
		ptr_ep = &pcd->in_ep[uint_ep_num - 1];
	} else {
		ptr_ep = &pcd->out_ep[uint_ep_num - 1];
	}

	return ptr_ep;
}

/**
 * This function checks the EP request queue, if the queue is not
 * empty the next request is started.
 */
void start_next_request(dwc_otg_pcd_ep_t * ep)
{
	dwc_otg_pcd_request_t *ptr_req = 0;
	uint32_t out_max_xfer;
	uint32_t max_transfer =
	    GET_CORE_IF(ep->pcd)->core_params->max_transfer_size;

	if (!DWC_CIRCLEQ_EMPTY(&ep->queue)) {
		ptr_req = DWC_CIRCLEQ_FIRST(&ep->queue);

			/* Setup and start the Transfer */
			if (ptr_req->dw_align_buf != 0) {
				ep->dwc_ep.dma_addr = ptr_req->dw_align_buf_dma;
				ep->dwc_ep.start_xfer_buff = ptr_req->dw_align_buf;
				ep->dwc_ep.xfer_buff = ptr_req->dw_align_buf;
			} else {
				ep->dwc_ep.dma_addr = ptr_req->dma;
				ep->dwc_ep.start_xfer_buff = ptr_req->buf;
				ep->dwc_ep.xfer_buff = ptr_req->buf;
			}
			ep->dwc_ep.sent_zlp = 0;
			ep->dwc_ep.total_len = ptr_req->length;
			ep->dwc_ep.xfer_len = 0;
			ep->dwc_ep.xfer_count = 0;

			ep->dwc_ep.maxxfer = max_transfer;
			if (GET_CORE_IF(ep->pcd)->dma_desc_enable != 0) {
				out_max_xfer = DDMA_MAX_TRANSFER_SIZE - (DDMA_MAX_TRANSFER_SIZE % 4);
				if (ep->dwc_ep.is_in != 0)  {
					if (ep->dwc_ep.maxxfer >
					    DDMA_MAX_TRANSFER_SIZE) {
						ep->dwc_ep.maxxfer =
						    DDMA_MAX_TRANSFER_SIZE;
					}
				} else {
					if (ep->dwc_ep.maxxfer > out_max_xfer) {
						ep->dwc_ep.maxxfer =
						    out_max_xfer;
					}
				}
			}
			if (ep->dwc_ep.maxxfer < ep->dwc_ep.total_len) {
				ep->dwc_ep.maxxfer -=
				    (ep->dwc_ep.maxxfer % ep->dwc_ep.maxpacket);
			}
			if (ptr_req->sent_zlp != 0)  {
				if (((ep->dwc_ep.total_len %
				     ep->dwc_ep.maxpacket) == 0)
				    && (ep->dwc_ep.total_len != 0)) {
					ep->dwc_ep.sent_zlp = 1;
				}

			}
		dwc_otg_ep_start_transfer(GET_CORE_IF(ep->pcd), &ep->dwc_ep);
	} 
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
/**
 * This function handles the SOF Interrupts. At this time the SOF
 * Interrupt is disabled.
 */
int32_t dwc_otg_pcd_handle_sof_intr(dwc_otg_pcd_t const * pcd)
{
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);

	gintsts_data_t gbl_int_sts;

	/* Clear interrupt */
	gbl_int_sts.d32 = 0;
	gbl_int_sts.b.sofintr = 1;
	REG_WR(&ptr_core_if->core_global_regs->gintsts, gbl_int_sts.d32);

	return 1;
}

/**
 * This function handles the Rx Status Queue Level Interrupt, which
 * indicates that there is a least one packet in the Rx FIFO.  The
 * packets are moved from the FIFO to memory, where they will be
 * processed when the Endpoint Interrupt Register indicates Transfer
 * Complete or SETUP Phase Done.
 *
 * Repeat the following until the Rx Status Queue is empty:
 *	 -# Read the Receive Status Pop Register (GRXSTSP) to get Packet
 *		info
 *	 -# If Receive FIFO is empty then skip to step Clear the interrupt
 *		and exit
 *	 -# If OUT Data Packet call dwc_otg_read_packet to copy the data
 *		to the destination buffer
 */
int32_t dwc_otg_pcd_handle_rx_status_q_level_intr(dwc_otg_pcd_t * pcd)
{
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
	dwc_otg_core_global_regs_t *global_regs = ptr_core_if->core_global_regs;
	gintmsk_data_t gintmask;
	device_grxsts_data_t dev_rec_status;
	dwc_otg_pcd_ep_t *ptr_ep;
	gintsts_data_t gbl_int_sts;
	gintmask.d32 = 0 ;

	/* Disable the Rx Status Queue Level interrupt */
	gintmask.b.rxstsqlvl = 1;
	DWC_MODIFY_REG32(&global_regs->gintmsk, gintmask.d32, 0);

	/* Get the Status from the top of the FIFO */
	dev_rec_status.d32 = REG_RD(&global_regs->grxstsp);

	DBG_USB_Print(DBG_PCD, "EP:%d BCnt:%d "
		    "pktsts:%x Frame:%d(0x%0x)\n",
		    dev_rec_status.b.epnum, dev_rec_status.b.bcnt,
		    dev_rec_status.b.pktsts, dev_rec_status.b.fn, dev_rec_status.b.fn);
	/* Get pointer to EP structure */
	ptr_ep = get_out_ep(pcd, dev_rec_status.b.epnum);

	switch (dev_rec_status.b.pktsts) {
	case DWC_DSTS_GOUT_NAK:
		DBG_USB_Print(DBG_PCDV, "Global OUT NAK\n");
		break;
	case DWC_STS_DATA_UPDT:
		DBG_USB_Print(DBG_PCDV, "OUT Data Packet\n");
		if ((dev_rec_status.b.bcnt) && (ptr_ep->dwc_ep.xfer_buff)) {
			ptr_ep->dwc_ep.xfer_count += dev_rec_status.b.bcnt;
			ptr_ep->dwc_ep.xfer_buff += dev_rec_status.b.bcnt;
		}
		break;
	case DWC_STS_XFER_COMP:
		DBG_USB_Print(DBG_PCDV, "OUT Complete\n");
		break;
	case DWC_DSTS_SETUP_COMP:
		DBG_USB_Print(DBG_PCDV, "Setup Complete\n");
		break;
	case DWC_DSTS_SETUP_UPDT:
		DBG_USB_Print(DBG_PCD,
			    "SETUP PKT: %02x.%02x v%04x i%04x l%04x\n",
			    pcd->setup_pkt->req.bmRequestType,
			    pcd->setup_pkt->req.bRequest,
			    UGETW(pcd->setup_pkt->req.wValue),
			    UGETW(pcd->setup_pkt->req.wIndex),
			    UGETW(pcd->setup_pkt->req.wLength));
		ptr_ep->dwc_ep.xfer_count += dev_rec_status.b.bcnt;
		break;
	default:
		DBG_USB_Print(DBG_PCDV, "Invalid Packet Status (0x%0x)\n",
			    dev_rec_status.b.pktsts);
		break;
	}

	/* Enable the Rx Status Queue Level interrupt */
	DWC_MODIFY_REG32(&global_regs->gintmsk, 0, gintmask.d32);
	/* Clear interrupt */
	gbl_int_sts.d32 = 0;
	gbl_int_sts.b.rxstsqlvl = 1;
	REG_WR(&global_regs->gintsts, gbl_int_sts.d32);

	return 1;
}

/**
 * This function examines the Device IN Token Learning Queue to
 * determine the EP number of the last IN token received.  This
 * implementation is for the Mass Storage device where there are only
 * 2 IN EPs (Control-IN and BULK-IN).
 *
 * The EP numbers for the first six IN Tokens are in DTKNQR1 and there
 * are 8 EP Numbers in each of the other possible DTKNQ Registers.
 *
 * @param core_if Programming view of DWC_otg controller.
 *
 */
static uint32_t get_ep_of_last_in_token(dwc_otg_core_if_t const * core_if)
{
	dwc_otg_device_global_regs_t *ptr_dev_global_regs =
	    core_if->dev_if->dev_global_regs;
	const uint32_t TOKEN_Q_DEPTH = core_if->hwcfg2.b.dev_token_q_depth;
	/* Number of Token Queue Registers */
	const uint32_t DTKNQ_REG_CNT = (TOKEN_Q_DEPTH + 7) / 8;
	dtknq1_data_t dev_tkn_qr1;
	uint32_t in_tkn_epnums[4] = {0};
	int32_t ndx = 0;
	uint32_t i = 0;
	volatile uint32_t *ptr_addr = &ptr_dev_global_regs->dtknqr1;
	uint32_t uint_epnum = 0;

	/* Read the DTKNQ Registers */
	for (i = 0; i < DTKNQ_REG_CNT; i++) {
		in_tkn_epnums[i] = REG_RD(ptr_addr);
		DBG_USB_Print(DBG_PCDV, "DTKNQR%d=0x%08x\n", i + 1,
			    in_tkn_epnums[i]);
		if (ptr_addr == &ptr_dev_global_regs->dvbusdis) {
			ptr_addr = &ptr_dev_global_regs->dtknqr3_dthrctl;
		} else {
			++ptr_addr;
		}

	}

	/* Copy the DTKNQR1 data to the bit field. */
	dev_tkn_qr1.d32 = in_tkn_epnums[0];
	/* Get the EP numbers */
	in_tkn_epnums[0] = dev_tkn_qr1.b.epnums0_5;
	ndx = dev_tkn_qr1.b.intknwptr - 1;

	if (ndx == -1) {
		/** @todo Find a simpler way to calculate the max
		 * queue position.*/
		int32_t cnt = (int32_t)TOKEN_Q_DEPTH;
		if (TOKEN_Q_DEPTH <= 6) {
			cnt = (int32_t)(TOKEN_Q_DEPTH - 1);
		} else if (TOKEN_Q_DEPTH <= 14) {
			cnt = (int32_t)(TOKEN_Q_DEPTH - 7);
		} else if (TOKEN_Q_DEPTH <= 22) {
			cnt = (int32_t)(TOKEN_Q_DEPTH - 15);
		} else {
			cnt = (int32_t)(TOKEN_Q_DEPTH - 23);
		}
		uint_epnum = (in_tkn_epnums[DTKNQ_REG_CNT - 1] >> (cnt * 4)) & 0xF;
	} else {
		if (ndx <= 5) {
			uint_epnum = (in_tkn_epnums[0] >> (ndx * 4)) & 0xF;
		} else if (ndx <= 13) {
			ndx -= 6;
			uint_epnum = (in_tkn_epnums[1] >> (ndx * 4)) & 0xF;
		} else if (ndx <= 21) {
			ndx -= 14;
			uint_epnum = (in_tkn_epnums[2] >> (ndx * 4)) & 0xF;
		} else if (ndx <= 29) {
			ndx -= 22;
			uint_epnum = (in_tkn_epnums[3] >> (ndx * 4)) & 0xF;
		}
	}
	return uint_epnum;
}

/**
 * This interrupt occurs when the non-periodic Tx FIFO is half-empty.
 * The active request is checked for the next packet to be loaded into
 * the non-periodic Tx FIFO.
 */
int32_t dwc_otg_pcd_handle_np_tx_fifo_empty_intr(dwc_otg_pcd_t * pcd)
{
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
	dwc_otg_core_global_regs_t *global_regs = ptr_core_if->core_global_regs;
	gnptxsts_data_t txstatus;
	gintsts_data_t gbl_int_sts;
	dwc_otg_pcd_ep_t *ptr_ep = 0;
	
	uint32_t uint_epnum = 0;
	uint32_t len = 0;
	uint32_t dwords;
	txstatus.d32 = 0 ;

	/* Get the epnum from the IN Token Learning Queue. */
	uint_epnum = get_ep_of_last_in_token(ptr_core_if);
	ptr_ep = get_in_ep(pcd, uint_epnum);

	DBG_USB_Print(DBG_PCD, "NP TxFifo Empty: %d \n", uint_epnum);

	len = ptr_ep->dwc_ep.xfer_len - ptr_ep->dwc_ep.xfer_count;
	if (len > ptr_ep->dwc_ep.maxpacket) {
		len = ptr_ep->dwc_ep.maxpacket;
	}
	dwords = (len + 3) / 4;

	/* While there is space in the queue and space in the FIFO and
	 * More data to tranfer, Write packets to the Tx FIFO */
	txstatus.d32 = REG_RD(&global_regs->gnptxsts);
	DBG_USB_Print(DBG_PCDV, "b4 GNPTXSTS=0x%08x\n", txstatus.d32);

	while ((txstatus.b.nptxqspcavail > 0) &&
	       (txstatus.b.nptxfspcavail > dwords) &&
	       (ptr_ep->dwc_ep.xfer_count < ptr_ep->dwc_ep.xfer_len)) {
		/* Write the FIFO */
		dwc_otg_ep_write_packet(ptr_core_if, &ptr_ep->dwc_ep, 0);
		len = ptr_ep->dwc_ep.xfer_len - ptr_ep->dwc_ep.xfer_count;

		if (len > ptr_ep->dwc_ep.maxpacket) {
			len = ptr_ep->dwc_ep.maxpacket;
		}

		dwords = (len + 3) / 4;
		txstatus.d32 = REG_RD(&global_regs->gnptxsts);
		DBG_USB_Print(DBG_PCDV, "GNPTXSTS=0x%08x\n", txstatus.d32);
	}

	DBG_USB_Print(DBG_PCDV, "GNPTXSTS=0x%08x\n",
		    REG_RD(&global_regs->gnptxsts));

	/* Clear interrupt */
	gbl_int_sts.d32 = 0;
	gbl_int_sts.b.nptxfempty = 1;
	REG_WR(&global_regs->gintsts, gbl_int_sts.d32);

	return 1;
}

/**
 * This function is called when dedicated Tx FIFO Empty interrupt occurs.
 * The active request is checked for the next packet to be loaded into
 * apropriate Tx FIFO.
 */
static int32_t write_empty_tx_fifo(dwc_otg_pcd_t * pcd, uint32_t epnum)
{
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
	dwc_otg_dev_if_t *ptr_dev_if = ptr_core_if->dev_if;
	dtxfsts_data_t txstatus;
	dwc_otg_pcd_ep_t *ptr_ep = 0;
	uint32_t len = 0;
	uint32_t dwords;
	txstatus.d32 = 0 ;

	ptr_ep = get_in_ep(pcd, epnum);

	DBG_USB_Print(DBG_PCD, "Dedicated TxFifo Empty: %d \n", epnum);

	len = ptr_ep->dwc_ep.xfer_len - ptr_ep->dwc_ep.xfer_count;

	if (len > ptr_ep->dwc_ep.maxpacket) {
		len = ptr_ep->dwc_ep.maxpacket;
	}

	dwords = (len + 3) / 4;

	/* While there is space in the queue and space in the FIFO and
	 * More data to tranfer, Write packets to the Tx FIFO */
	txstatus.d32 = REG_RD(&ptr_dev_if->in_ep_regs[epnum]->dtxfsts);
	DBG_USB_Print(DBG_PCDV, "b4 dtxfsts[%d]=0x%08x\n", epnum, txstatus.d32);

	while ((txstatus.b.txfspcavail >= dwords) &&
	       (ptr_ep->dwc_ep.xfer_count < ptr_ep->dwc_ep.xfer_len) &&
	       (ptr_ep->dwc_ep.xfer_len != 0)) {
		/* Write the FIFO */
		dwc_otg_ep_write_packet(ptr_core_if, &ptr_ep->dwc_ep, 0);

		len = ptr_ep->dwc_ep.xfer_len - ptr_ep->dwc_ep.xfer_count;
		if (len > ptr_ep->dwc_ep.maxpacket) {
			len = ptr_ep->dwc_ep.maxpacket;
		}

		dwords = (len + 3) / 4;
		txstatus.d32 =
		    REG_RD(&ptr_dev_if->in_ep_regs[epnum]->dtxfsts);
		DBG_USB_Print(DBG_PCDV, "dtxfsts[%d]=0x%08x\n", epnum,
			    txstatus.d32);
	}

	DBG_USB_Print(DBG_PCDV, "b4 dtxfsts[%d]=0x%08x\n", epnum,
		    REG_RD(&ptr_dev_if->in_ep_regs[epnum]->dtxfsts));

	return 1;
}

/**
 * This function is called when the Device is disconnected. It stops
 * any active requests and informs the Gadget driver of the
 * disconnect.
 */
void dwc_otg_pcd_stop(dwc_otg_pcd_t * pcd)
{
	uint32_t i, int_num_in_eps, int_num_out_eps;
	dwc_otg_pcd_ep_t *ptr_ep;

	gintmsk_data_t intr_mask;
	intr_mask.d32 = 0 ;

	int_num_in_eps = GET_CORE_IF(pcd)->dev_if->num_in_eps;
	int_num_out_eps = GET_CORE_IF(pcd)->dev_if->num_out_eps;

	DBG_USB_Print(DBG_PCDV, "%s() \n", __func__);
	/* don't disconnect drivers more than once */
	if (pcd->ep0state == EP0_DISCONNECT) {
		DBG_USB_Print(DBG_USB, "%s() Already Disconnected\n", __func__);
	}else {
		pcd->ep0state = EP0_DISCONNECT;

		/* Reset the OTG state. */
		dwc_otg_pcd_update_otg(pcd, 1);

		/* Disable the NP Tx Fifo Empty Interrupt. */
		intr_mask.b.nptxfempty = 1;
		DWC_MODIFY_REG32(&GET_CORE_IF(pcd)->core_global_regs->gintmsk,
				 intr_mask.d32, 0);

		/* Flush the FIFOs */
		/**@todo NGS Flush Periodic FIFOs */
		dwc_otg_flush_tx_fifo(GET_CORE_IF(pcd), 0x10);
		dwc_otg_flush_rx_fifo(GET_CORE_IF(pcd));

		/* prevent new request submissions, kill any outstanding requests  */
		ptr_ep = &pcd->ep0;
		dwc_otg_request_nuke(ptr_ep);
		/* prevent new request submissions, kill any outstanding requests  */
		for (i = 0; i < int_num_in_eps; i++) {
			dwc_otg_pcd_ep_t *str_ptr_ep = &pcd->in_ep[i];
			dwc_otg_request_nuke(str_ptr_ep);
		}
		/* prevent new request submissions, kill any outstanding requests  */
		for (i = 0; i < int_num_out_eps; i++) {
			dwc_otg_pcd_ep_t *str_ptr_ep = &pcd->out_ep[i];
			dwc_otg_request_nuke(str_ptr_ep);
		}

		/* report disconnect; the driver is already quiesced */
		if ((pcd->fops->disconnect) != 0) {
			pcd->fops->disconnect(pcd);
		}
	}
	return;
}

/**
 * This interrupt indicates that ...
 */
int32_t dwc_otg_pcd_handle_i2c_intr(dwc_otg_pcd_t * pcd)
{
	gintmsk_data_t intr_mask;
	gintsts_data_t gbl_int_sts;
	intr_mask.d32 = 0 ;

	DBG_USB_Print(DBG_USB,"INTERRUPT Handler not implemented for %s\n", "i2cintr");
	intr_mask.b.i2cintr = 1;
	DWC_MODIFY_REG32(&GET_CORE_IF(pcd)->core_global_regs->gintmsk,
			 intr_mask.d32, 0);

	/* Clear interrupt */
	gbl_int_sts.d32 = 0;
	gbl_int_sts.b.i2cintr = 1;
	REG_WR(&GET_CORE_IF(pcd)->core_global_regs->gintsts,
			gbl_int_sts.d32);
	return 1;
}
#endif

/**
 * This interrupt indicates that ...
 */
int32_t dwc_otg_pcd_handle_early_suspend_intr(dwc_otg_pcd_t * pcd)
{
	gintsts_data_t gbl_int_sts;

	/* Clear interrupt */
	gbl_int_sts.d32 = 0;
	gbl_int_sts.b.erlysuspend = 1;
	REG_WR(&GET_CORE_IF(pcd)->core_global_regs->gintsts,
			gbl_int_sts.d32);
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	return 1;
}

/**
 * This function configures EPO to receive SETUP packets.
 *
 * @todo NGS: Update the comments from the HW FS.
 *
 *	-# Program the following fields in the endpoint specific registers
 *	for Control OUT EP 0, in order to receive a setup packet
 *	- DOEPTSIZ0.Packet Count = 3 (To receive up to 3 back to back
 *	  setup packets)
 *	- DOEPTSIZE0.Transfer Size = 24 Bytes (To receive up to 3 back
 *	  to back setup packets)
 *		- In DMA mode, DOEPDMA0 Register with a memory address to
 *		  store any setup packets received
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param pcd	  Programming view of the PCD.
 */
static inline void ep0_out_start(dwc_otg_core_if_t const * core_if,
				 dwc_otg_pcd_t const * pcd)
{
	int32_t flag = 0;
	dwc_otg_dev_if_t *ptr_dev_if = core_if->dev_if;
	dwc_otg_dev_dma_desc_t *dma_desc;
	depctl_data_t dev_ep_ctl;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED) 
    deptsiz0_data_t doeptsize0;
	doeptsize0.d32 = 0 ;
#endif
	dev_ep_ctl.d32 = 0 ;

#ifdef VERBOSE
	DBG_USB_Print(DBG_PCDV, "%s() doepctl0=%0x\n", __func__,
		    REG_RD(&dev_if->out_ep_regs[0]->doepctl));
#endif
	if (core_if->snpsid >= OTG_CORE_REV_3_00a) {
		dev_ep_ctl.d32 = REG_RD(&ptr_dev_if->out_ep_regs[0]->doepctl);
		if ((dev_ep_ctl.b.epena) != 0) {
			flag = 1;
		}
	}
	if(flag == 0){
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		doeptsize0.b.supcnt = 3;
		doeptsize0.b.pktcnt = 1;
		doeptsize0.b.xfersize = 8 * 3;

		if (core_if->dma_enable != 0) {
			if (!core_if->dma_desc_enable) {
				/** put here as for Hermes mode deptisz register should not be written */
				REG_WR(&ptr_dev_if->out_ep_regs[0]->doeptsiz,
						doeptsize0.d32);

				/** @todo dma needs to handle multiple setup packets (up to 3) */
				REG_WR(&ptr_dev_if->out_ep_regs[0]->doepdma,
						pcd->setup_pkt_dma_handle);
			} else {
#endif
				ptr_dev_if->setup_desc_index =
					(ptr_dev_if->setup_desc_index + 1) & 1;
				dma_desc =
					ptr_dev_if->setup_desc_addr[ptr_dev_if->setup_desc_index];

				/** DMA Descriptor Setup */
				dma_desc->status.b.bs = BS_HOST_BUSY;
				if (core_if->snpsid >= OTG_CORE_REV_3_00a) {
					dma_desc->status.b.sr = 0;
					dma_desc->status.b.mtrf = 0;
				}
				dma_desc->status.b.l = 1;
				dma_desc->status.b.ioc = 1;
				dma_desc->status.b.bytes = pcd->ep0.dwc_ep.maxpacket;
				dma_desc->buf = pcd->setup_pkt_dma_handle;
				dma_desc->status.b.sts = 0;
				dma_desc->status.b.bs = BS_HOST_READY;

				
				/** DOEPDMA0 Register write */
				REG_WR(&ptr_dev_if->out_ep_regs[0]->doepdma,
						ptr_dev_if->dma_setup_desc_addr
						[ptr_dev_if->setup_desc_index]);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			}        
		}       
        else {
			/** put here as for Hermes mode deptisz register should not be written */
			REG_WR(&ptr_dev_if->out_ep_regs[0]->doeptsiz,
					doeptsize0.d32); 
		}
 #endif
		/** DOEPCTL0 Register write cnak will be set after setup interrupt */
		dev_ep_ctl.d32 = 0;
		dev_ep_ctl.b.epena = 1;
		if (core_if->snpsid <= OTG_CORE_REV_2_94a) {
			dev_ep_ctl.b.cnak = 1;
			REG_WR(&ptr_dev_if->out_ep_regs[0]->doepctl, dev_ep_ctl.d32);
		} else {
			DWC_MODIFY_REG32(&ptr_dev_if->out_ep_regs[0]->doepctl, 0, dev_ep_ctl.d32);
		}
#ifdef VERBOSE
		DBG_USB_Print(DBG_PCDV, "doepctl0=%0x\n",
				REG_RD(&dev_if->out_ep_regs[0]->doepctl));
		DBG_USB_Print(DBG_PCDV, "diepctl0=%0x\n",
				REG_RD(&dev_if->in_ep_regs[0]->diepctl));
#endif
	}
	return;
}

/**
 * This interrupt occurs when a USB Reset is detected. When the USB
 * Reset Interrupt occurs the device state is set to DEFAULT and the
 * EP0 state is set to IDLE.
 *	-#	Set the NAK bit for all OUT endpoints (DOEPCTLn.SNAK = 1)
 *	-#	Unmask the following interrupt bits
 *		- DAINTMSK.INEP0 = 1 (Control 0 IN endpoint)
 *	- DAINTMSK.OUTEP0 = 1 (Control 0 OUT endpoint)
 *	- DOEPMSK.SETUP = 1
 *	- DOEPMSK.XferCompl = 1
 *	- DIEPMSK.XferCompl = 1
 *	- DIEPMSK.TimeOut = 1
 *	-# Program the following fields in the endpoint specific registers
 *	for Control OUT EP 0, in order to receive a setup packet
 *	- DOEPTSIZ0.Packet Count = 3 (To receive up to 3 back to back
 *	  setup packets)
 *	- DOEPTSIZE0.Transfer Size = 24 Bytes (To receive up to 3 back
 *	  to back setup packets)
 *		- In DMA mode, DOEPDMA0 Register with a memory address to
 *		  store any setup packets received
 * At this point, all the required initialization, except for enabling
 * the control 0 OUT endpoint is done, for receiving SETUP packets.
 */
int32_t dwc_otg_pcd_handle_usb_reset_intr(dwc_otg_pcd_t * pcd)
{
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
	dwc_otg_dev_if_t *ptr_dev_if = ptr_core_if->dev_if;
	depctl_data_t dev_oep_ctl; 
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	depctl_data_t dev_ep_ctl ;
#endif
	daint_data_t dev_all_int_msk; 
	doepmsk_data_t dev_oep_msk ;
	diepmsk_data_t dev_ep_msk ;
	dcfg_data_t dev_cfg; 
	grstctl_t resetctl; 
	dctl_data_t dev_ctl; 
	int32_t i = 0;
	gintsts_data_t gbl_int_sts;
	pcgcctl_data_t power;
	power.d32 = 0 ;
	dev_oep_ctl.d32 = 0 ;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	dev_ep_ctl.d32 = 0 ;
#endif
	dev_all_int_msk.d32 = 0 ;
	dev_oep_msk.d32 = 0 ;
	dev_ep_msk.d32 = 0 ;
	dev_cfg.d32 = 0 ;
	resetctl.d32 = 0 ;
	dev_ctl.d32 = 0 ;

	power.d32 = REG_RD(ptr_core_if->pcgcctl);
	if (power.b.stoppclk != 0) {
		power.d32 = 0;
		power.b.stoppclk = 1;
		DWC_MODIFY_REG32(ptr_core_if->pcgcctl, power.d32, 0);

		power.b.pwrclmp = 1;
		DWC_MODIFY_REG32(ptr_core_if->pcgcctl, power.d32, 0);

		power.b.rstpdwnmodule = 1;
		DWC_MODIFY_REG32(ptr_core_if->pcgcctl, power.d32, 0);
	}
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	ptr_core_if->lx_state = DWC_OTG_L0;
	ptr_core_if->otg_sts = 0;
	
	/* reset the HNP settings */
	dwc_otg_pcd_update_otg(pcd, 1);

	/* Clear the Remote Wakeup Signalling */
	dev_ctl.b.rmtwkupsig = 1;
	DWC_MODIFY_REG32(&ptr_core_if->dev_if->dev_global_regs->dctl, dev_ctl.d32, 0);

	/* Set NAK for all OUT EPs */
	dev_oep_ctl.b.snak = 1;
	for (i = 0; i <= ptr_dev_if->num_out_eps; i++) {
		REG_WR(&ptr_dev_if->out_ep_regs[i]->doepctl, dev_oep_ctl.d32);
	}

	/* Flush the NP Tx FIFO */
	dwc_otg_flush_tx_fifo(ptr_core_if, 0x10);
	/* Flush the Learning Queue */
	resetctl.b.intknqflsh = 1;
	REG_WR(&ptr_core_if->core_global_regs->grstctl, resetctl.d32);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	if ((!(ptr_core_if->core_params->en_multiple_tx_fifo)) && (ptr_core_if->dma_enable)) {
		ptr_core_if->start_predict = 0;
		for (i = 0; i <= ptr_core_if->dev_if->num_in_eps; ++i) {
			ptr_core_if->nextep_seq[i] = 0xff;	/* 0xff - EP not active */
		}
		ptr_core_if->nextep_seq[0] = 0;
		ptr_core_if->first_in_nextep_seq = 0;
		dev_ep_ctl.d32 = REG_RD(&ptr_dev_if->in_ep_regs[0]->diepctl);
		dev_ep_ctl.b.nextep = 0;
		REG_WR(&ptr_dev_if->in_ep_regs[0]->diepctl, dev_ep_ctl.d32);

		/* Update IN Endpoint Mismatch Count by active IN NP EP count + 1 */
		dev_cfg.d32 = REG_RD(&ptr_dev_if->dev_global_regs->dcfg);
		dev_cfg.b.endevoutnak = 1;
		dev_cfg.b.epmscnt = 2;
		REG_WR(&ptr_dev_if->dev_global_regs->dcfg, dev_cfg.d32);

		DBG_USB_Print(DBG_PCDV,
			    "%s first_in_nextep_seq= %2d; nextep_seq[]:\n",
			    __func__, ptr_core_if->first_in_nextep_seq);
		for (i = 0; i <= ptr_core_if->dev_if->num_in_eps; i++) {
			DBG_USB_Print(DBG_PCDV, "%2d\n", ptr_core_if->nextep_seq[i]);
		}
	}
	if (ptr_core_if->multiproc_int_enable != 0) {
		dev_all_int_msk.b.inep0 = 1;
		dev_all_int_msk.b.outep0 = 1;
		REG_WR(&ptr_dev_if->dev_global_regs->deachintmsk,
				dev_all_int_msk.d32);

		dev_oep_msk.b.setup = 1;
		dev_oep_msk.b.xfercompl = 1;
		dev_oep_msk.b.ahberr = 1;
		dev_oep_msk.b.epdisabled = 1;

		if (((ptr_core_if->dma_desc_enable) ||
		    (ptr_core_if->dma_enable))
		     && (ptr_core_if->snpsid >= OTG_CORE_REV_3_00a)) {
			dev_oep_msk.b.stsphsercvd = 1;
		}
		if (ptr_core_if->dma_desc_enable != 0) {
			dev_oep_msk.b.bna = 1;
		}
		dev_oep_msk.b.babble = 1;
		dev_oep_msk.b.nyet = 1;
		
		if (ptr_core_if->dma_enable != 0) {
			dev_oep_msk.b.nak = 1;
		}

		REG_WR(&ptr_dev_if->dev_global_regs->doepeachintmsk[0],
				dev_oep_msk.d32);

		dev_ep_msk.b.xfercompl = 1;
		dev_ep_msk.b.timeout = 1;
		dev_ep_msk.b.epdisabled = 1;
		dev_ep_msk.b.ahberr = 1;
		if ((!(ptr_core_if->en_multiple_tx_fifo)) && (ptr_core_if->dma_enable)) {
			dev_ep_msk.b.intknepmis = 0;
		}
		if (ptr_core_if->dma_desc_enable != 0) {
			dev_ep_msk.b.bna = 1;
		}

		
		if (ptr_core_if->dma_enable != 0) {
			dev_ep_msk.b.nak = 1;
		}

		REG_WR(&ptr_dev_if->dev_global_regs->diepeachintmsk[0],
				dev_ep_msk.d32);
	} else {
#endif
		dev_all_int_msk.b.inep0 = 1;
		dev_all_int_msk.b.outep0 = 1;
		REG_WR(&ptr_dev_if->dev_global_regs->daintmsk,
				dev_all_int_msk.d32);

		dev_oep_msk.b.setup = 1;
		dev_oep_msk.b.xfercompl = 1;
		dev_oep_msk.b.ahberr = 1;
		dev_oep_msk.b.epdisabled = 1;

		if ((ptr_core_if->dma_desc_enable) ||
		    ((ptr_core_if->dma_enable)
		     && ((ptr_core_if->snpsid) >= (OTG_CORE_REV_3_00a)))) {
			dev_oep_msk.b.stsphsercvd = 1;
		}
		if ((ptr_core_if->dma_desc_enable) != 0) {
			dev_oep_msk.b.bna = 1;
		}
		REG_WR(&ptr_dev_if->dev_global_regs->doepmsk, dev_oep_msk.d32);

		dev_ep_msk.b.xfercompl = 1;
		dev_ep_msk.b.timeout = 1;
		dev_ep_msk.b.epdisabled = 1;
		dev_ep_msk.b.ahberr = 1;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		if ((!(ptr_core_if->en_multiple_tx_fifo)) && (ptr_core_if->dma_enable)){
			dev_ep_msk.b.intknepmis = 0;
		}
#endif
		if (ptr_core_if->dma_desc_enable != 0) {
			dev_ep_msk.b.bna = 1;
		}


		REG_WR(&ptr_dev_if->dev_global_regs->diepmsk, dev_ep_msk.d32);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	}
#endif
	/* Reset Device Address */
	dev_cfg.d32 = REG_RD(&ptr_dev_if->dev_global_regs->dcfg);
	dev_cfg.b.endevoutnak = 1;
	dev_cfg.b.devaddr = 0;
	REG_WR(&ptr_dev_if->dev_global_regs->dcfg, dev_cfg.d32);
	
	/* setup EP0 to receive SETUP packets */
	if (ptr_core_if->snpsid <= OTG_CORE_REV_2_94a) {
		ep0_out_start(ptr_core_if, pcd);
	}
	/* Clear interrupt */
	gbl_int_sts.d32 = 0;
	gbl_int_sts.b.usbreset = 1;
	REG_WR(&ptr_core_if->core_global_regs->gintsts, gbl_int_sts.d32);

	return 1;
}

/**
 * Get the device speed from the device status register and convert it
 * to USB speed constant.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static int32_t get_device_speed(dwc_otg_core_if_t const * core_if)
{
	dsts_data_t dev_sts;
	int32_t int_speed = 0;
	dev_sts.d32 = REG_RD(&core_if->dev_if->dev_global_regs->dsts);

	switch (dev_sts.b.enumspd) {
	case DWC_DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ:
		int_speed = USB_SPEED_HIGH;
		break;
	case DWC_DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ:
	case DWC_DSTS_ENUMSPD_FS_PHY_48MHZ:
		int_speed = USB_SPEED_FULL;
		break;

	case DWC_DSTS_ENUMSPD_LS_PHY_6MHZ:
		int_speed = USB_SPEED_LOW_1;
		break;
	default :		
		break;
	}

	return int_speed;
}

/**
 * Read the device status register and set the device speed in the
 * data structure.
 * Set up EP0 to receive SETUP packets by calling dwc_ep0_activate.
 */
int32_t dwc_otg_pcd_handle_enum_done_intr(dwc_otg_pcd_t * pcd)
{
	dwc_otg_pcd_ep_t *ptr_ep0 = &pcd->ep0;
	gintsts_data_t gbl_int_sts;
	gusbcfg_data_t gbl_usb_cfg;
	dwc_otg_core_global_regs_t *global_regs =
	    GET_CORE_IF(pcd)->core_global_regs;
	uint8_t utmi16b, utmi8b;
	int32_t int_speed;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	dcfg_data_t dev_cfg;
#endif

	DBG_USB_Print(DBG_PCD, "SPEED ENUM\n");
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	/* WA for the case when SW gets SPEED ENUM without first USB RESET case
	* due to USB RESET issued by the host earlier. Anyways USB Reset routine
	* needs to be called to at least program EP 0 OUT - vahrama
	*/
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	dev_cfg.d32 = REG_RD(&pcd->core_if->dev_if->dev_global_regs->dcfg);
	if ((pcd->core_if->otg_ver) && (dev_cfg.b.devaddr)) {
		dwc_otg_pcd_handle_usb_reset_intr(pcd);
	}
#endif

	if (GET_CORE_IF(pcd)->snpsid >= OTG_CORE_REV_2_60a) {
		utmi16b = 6;	/* vahrama old value was 6;*/
		utmi8b = 9;
	} else {
		utmi16b = 4;
		utmi8b = 8;
	}
	dwc_otg_ep0_activate(GET_CORE_IF(pcd), &ptr_ep0->dwc_ep);
	if (GET_CORE_IF(pcd)->snpsid >= OTG_CORE_REV_3_00a) {
		ep0_out_start(GET_CORE_IF(pcd), pcd);
	}


	if (pcd->ep0state == EP0_DISCONNECT) {
		pcd->ep0state = EP0_IDLE;
	} else if (pcd->ep0state == EP0_STALL) {
		pcd->ep0state = EP0_IDLE;
	}

	pcd->ep0state = EP0_IDLE;

	ptr_ep0->stopped = 0;

	int_speed = get_device_speed(GET_CORE_IF(pcd));
	pcd->fops->connect(pcd, int_speed);

	/* Set USB turnaround time based on device speed and PHY interface. */
	gbl_usb_cfg.d32 = REG_RD(&global_regs->gusbcfg);
	if (int_speed == USB_SPEED_HIGH_1) {
		if (GET_CORE_IF(pcd)->hwcfg2.b.hs_phy_type ==
		    DWC_HWCFG2_HS_PHY_TYPE_ULPI) {
			/* ULPI interface */
			gbl_usb_cfg.b.usbtrdtim = 9;
		}
		if (GET_CORE_IF(pcd)->hwcfg2.b.hs_phy_type ==
		    DWC_HWCFG2_HS_PHY_TYPE_UTMI) {
			/* UTMI+ interface */
			if (GET_CORE_IF(pcd)->hwcfg4.b.utmi_phy_data_width == 0) {
				gbl_usb_cfg.b.usbtrdtim = utmi8b;
			} else if (GET_CORE_IF(pcd)->hwcfg4.
				   b.utmi_phy_data_width == 1) {
				gbl_usb_cfg.b.usbtrdtim = utmi16b;
			} else if (GET_CORE_IF(pcd)->
				   core_params->phy_utmi_width == 8) {
				gbl_usb_cfg.b.usbtrdtim = utmi8b;
			} else {
				gbl_usb_cfg.b.usbtrdtim = utmi16b;
			}
		}
		if (GET_CORE_IF(pcd)->hwcfg2.b.hs_phy_type ==
		    DWC_HWCFG2_HS_PHY_TYPE_UTMI_ULPI) {
			/* UTMI+  OR  ULPI interface */
			if (gbl_usb_cfg.b.ulpi_utmi_sel == 1) {
				/* ULPI interface */
				gbl_usb_cfg.b.usbtrdtim = 9;
			} else {
				/* UTMI+ interface */
				if (GET_CORE_IF(pcd)->
				    core_params->phy_utmi_width == 16) {
					gbl_usb_cfg.b.usbtrdtim = utmi16b;
				} else {
					gbl_usb_cfg.b.usbtrdtim = utmi8b;
				}
			}
		}
	} else {
		/* Full or low speed */
		gbl_usb_cfg.b.usbtrdtim = 9;
	}
	REG_WR(&global_regs->gusbcfg, gbl_usb_cfg.d32);
	/* Clear interrupt */
	gbl_int_sts.d32 = 0;
	gbl_int_sts.b.enumdone = 1;
	REG_WR(&GET_CORE_IF(pcd)->core_global_regs->gintsts,
			gbl_int_sts.d32);
	return 1;
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
/**
 * This interrupt indicates the end of the portion of the micro-frame
 * for periodic transactions.  If there is a periodic transaction for
 * the next frame, load the packets into the EP periodic Tx FIFO.
 */
int32_t dwc_otg_pcd_handle_end_periodic_frame_intr(dwc_otg_pcd_t * pcd)
{
	gintmsk_data_t intr_mask;
	gintsts_data_t gbl_int_sts;
	intr_mask.d32 = 0 ;
	DBG_USB_Print(DBG_USB,"INTERRUPT Handler not implemented for %s\n", "EOP");

	intr_mask.b.eopframe = 1;
	DWC_MODIFY_REG32(&GET_CORE_IF(pcd)->core_global_regs->gintmsk,
			 intr_mask.d32, 0);

	/* Clear interrupt */
	gbl_int_sts.d32 = 0;
	gbl_int_sts.b.eopframe = 1;
	REG_WR(&GET_CORE_IF(pcd)->core_global_regs->gintsts,
			gbl_int_sts.d32);

	return 1;
}

/**
 * This interrupt indicates that EP of the packet on the top of the
 * non-periodic Tx FIFO does not match EP of the IN Token received.
 *
 * The "Device IN Token Queue" Registers are read to determine the
 * order the IN Tokens have been received. The non-periodic Tx FIFO
 * is flushed, so it can be reloaded in the order seen in the IN Token
 * Queue.
 */
int32_t dwc_otg_pcd_handle_ep_mismatch_intr(dwc_otg_pcd_t const * pcd)
{
	gintsts_data_t gbl_int_sts;
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
	dctl_data_t dev_ctl;
    gintmsk_data_t intr_mask;
	intr_mask.d32 = 0 ;
	if ((!(ptr_core_if->en_multiple_tx_fifo)) && (ptr_core_if->dma_enable)) {
		ptr_core_if->start_predict = 1;

		DBG_USB_Print(DBG_PCDV, "%s(%p)\n", __func__, ptr_core_if);
	
		gbl_int_sts.d32 = REG_RD(&ptr_core_if->core_global_regs->gintsts);
		if (!gbl_int_sts.b.ginnakeff) {
			/* Disable EP Mismatch interrupt */
			intr_mask.d32 = 0;
			intr_mask.b.epmismatch = 1;
			DWC_MODIFY_REG32(&ptr_core_if->core_global_regs->gintmsk, intr_mask.d32, 0);
			/* Enable the Global IN NAK Effective Interrupt */
			intr_mask.d32 = 0;
			intr_mask.b.ginnakeff = 1;
			DWC_MODIFY_REG32(&ptr_core_if->core_global_regs->gintmsk, 0, intr_mask.d32);
			/* Set the global non-periodic IN NAK handshake */
			dev_ctl.d32 = REG_RD(&ptr_core_if->dev_if->dev_global_regs->dctl);
			dev_ctl.b.sgnpinnak = 1;
			REG_WR(&ptr_core_if->dev_if->dev_global_regs->dctl, dev_ctl.d32);
		}
		/* Disabling of all EP's will be done in dwc_otg_pcd_handle_in_nak_effective()
		 * handler after Global IN NAK Effective interrupt will be asserted */
	}
	/* Clear interrupt */
	gbl_int_sts.d32 = 0;
	gbl_int_sts.b.epmismatch = 1;
	REG_WR(&ptr_core_if->core_global_regs->gintsts, gbl_int_sts.d32);

	return 1;
}

/**
 * This interrupt is valid only in DMA mode. This interrupt indicates that the
 * core has stopped fetching data for IN endpoints due to the unavailability of
 * TxFIFO space or Request Queue space. This interrupt is used by the
 * application for an endpoint mismatch algorithm.
 * 
 * @param pcd The PCD 
 */
int32_t dwc_otg_pcd_handle_ep_fetsusp_intr(dwc_otg_pcd_t const * pcd)
{
	gintsts_data_t gbl_int_sts;
	gintmsk_data_t gbl_int_msk_data;
	dctl_data_t dev_ctl;
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
	DBG_USB_Print(DBG_PCDV, "%s(%p)\n", __func__, ptr_core_if);

	/* Clear the global non-periodic IN NAK handshake */
	dev_ctl.d32 = 0;
	dev_ctl.b.cgnpinnak = 1;
	DWC_MODIFY_REG32(&ptr_core_if->dev_if->dev_global_regs->dctl, dev_ctl.d32, dev_ctl.d32); 
	
	/* Mask GINTSTS.FETSUSP interrupt */
	gbl_int_msk_data.d32 = REG_RD(&ptr_core_if->core_global_regs->gintmsk);
	gbl_int_msk_data.b.fetsusp = 0;
	REG_WR(&ptr_core_if->core_global_regs->gintmsk, gbl_int_msk_data.d32);

	/* Clear interrupt */
	gbl_int_sts.d32 = 0;
	gbl_int_sts.b.fetsusp = 1;
	REG_WR(&ptr_core_if->core_global_regs->gintsts, gbl_int_sts.d32);

	return 1;
}
#endif

/**
 * This funcion stalls EP0.
 */
static inline void ep0_do_stall(dwc_otg_pcd_t * pcd, const int32_t err_val)
{
	dwc_otg_pcd_ep_t *ptr_ep0 = &pcd->ep0;
#if WRN_INFO
	usb_device_request_t *ctrl = &pcd->setup_pkt->req;
	DBG_Warn_Print("req %02x.%02x protocol STALL; err %d\n",
		 ctrl->bmRequestType, ctrl->bRequest, err_val);
#endif
	ptr_ep0->dwc_ep.is_in = 1;
	dwc_otg_ep_set_stall(GET_CORE_IF(pcd), &ptr_ep0->dwc_ep);
	ptr_ep0->dwc_ep.is_in = 0;
    dwc_otg_ep_set_stall(GET_CORE_IF(pcd), &ptr_ep0->dwc_ep);
	pcd->ep0.stopped = 1;
	pcd->ep0state = EP0_IDLE;
	ep0_out_start(GET_CORE_IF(pcd), pcd);
	(void)err_val;
	return;
}

/**
 * This functions delegates the setup command to the gadget driver.
 */
static inline void do_gadget_setup(dwc_otg_pcd_t * pcd,
				   usb_device_request_t const * ctrl)
{
	int32_t ret = 0;
	ret = pcd->fops->setup(pcd, (uint8_t *) ctrl);
	if (ret < 0) {
		ep0_do_stall(pcd, ret);
	}

	/** @todo This is a g_file_storage gadget driver specific
	 * workaround: a DELAYED_STATUS result from the fsg_setup
	 * routine will result in the gadget queueing a EP0 IN status
	 * phase for a two-stage control transfer. Exactly the same as
	 * a SET_CONFIGURATION/SET_INTERFACE except that this is a class
	 * specific request.  Need a generic way to know when the gadget
	 * driver will queue the status phase. Can we assume when we
	 * call the gadget driver setup() function that it will always
	 * queue and require the following flag? Need to look into
	 * this.
	 */

	if (ret == (256 + 999)) {
		pcd->request_config = 1;
	}
}


/**
 * This function starts the Zero-Length Packet for the IN status phase
 * of a 2 stage control transfer.
 */
static inline void do_setup_in_status_phase(dwc_otg_pcd_t * pcd)
{
	dwc_otg_pcd_ep_t *ptr_ep0 = &pcd->ep0;
	if (pcd->ep0state == EP0_STALL) {
		;
	}else {

		pcd->ep0state = EP0_IN_STATUS_PHASE;

		/* Prepare for more SETUP Packets */
		DBG_USB_Print(DBG_PCD, "EP0 IN ZLP\n");
		if ((GET_CORE_IF(pcd)->snpsid >= OTG_CORE_REV_3_00a)
			&& (pcd->core_if->dma_desc_enable)
			&& (ptr_ep0->dwc_ep.xfer_count < ptr_ep0->dwc_ep.total_len)) {
			pcd->data_terminated = 1;
		}
		ptr_ep0->dwc_ep.xfer_len = 0;
		ptr_ep0->dwc_ep.xfer_count = 0;
		ptr_ep0->dwc_ep.is_in = 1;
		ptr_ep0->dwc_ep.dma_addr = pcd->setup_pkt_dma_handle;
		dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &ptr_ep0->dwc_ep);

		/* Prepare for more SETUP Packets */
		ep0_out_start(GET_CORE_IF(pcd), pcd);
	}
	return;
}

/**
 * This function starts the Zero-Length Packet for the OUT status phase
 * of a 2 stage control transfer.
 */
static inline void do_setup_out_status_phase(dwc_otg_pcd_t * pcd)
{
	dwc_otg_pcd_ep_t *ptr_ep0 = &pcd->ep0;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	doepint_data_t dev_oep_int;
	dev_oep_int.d32 = REG_RD(&pcd->core_if->dev_if->out_ep_regs[0]->doepint);
#endif
	if (pcd->ep0state == EP0_STALL) {
		DBG_USB_Print(DBG_PCD, "EP0 STALLED\n");
	}else {
		pcd->ep0state = EP0_OUT_STATUS_PHASE;

		ptr_ep0->dwc_ep.xfer_len = 0;
		ptr_ep0->dwc_ep.xfer_count = 0;
		ptr_ep0->dwc_ep.is_in = 0;
		ptr_ep0->dwc_ep.dma_addr = pcd->setup_pkt_dma_handle;
		/* If there is xfercomplete on EP0 OUT do not start OUT Status stage.
		 * xfercomplete means that ZLP was already received as EP0 OUT is enabled 
		 * during IN Data stage
		 */
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		if ((dev_oep_int.b.xfercompl == 1) && (pcd->core_if->snpsid >= OTG_CORE_REV_3_00a)
			&& (pcd->core_if->dma_enable == 1) && (pcd->core_if->dma_desc_enable == 0)) {
		}else {
#endif
			dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &ptr_ep0->dwc_ep);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			/* Prepare for more SETUP Packets */
			if (GET_CORE_IF(pcd)->dma_enable == 0) {
				ep0_out_start(GET_CORE_IF(pcd), pcd);
			}
		}
#endif
	}
	return;
}

/**
 * Clear the EP halt (STALL) and if pending requests start the
 * transfer.
 */
static inline void pcd_clear_halt(dwc_otg_pcd_t * pcd, dwc_otg_pcd_ep_t * ep)
{
	if (ep->dwc_ep.stall_clear_flag != 0) {
		/* Start Control Status Phase */
		do_setup_in_status_phase(pcd);
	}else{

		dwc_otg_ep_clear_stall(GET_CORE_IF(pcd), &ep->dwc_ep);

		/* Reactive the EP */
		dwc_otg_ep_activate(GET_CORE_IF(pcd), &ep->dwc_ep);
		if ((ep->stopped) != 0) {
			ep->stopped = 0;
			/* If there is a request in the EP queue start it */

			/*
			 * Above fixme is solved by implmenting a tasklet to call the
			 * start_next_request(), outside of interrupt context at some
			 * time after the current time, after a clear-halt setup packet.
			 * Still need to implement ep mismatch in the future if a gadget
			 * ever uses more than one endpoint at once
			 */
			ep->queue_sof = 1;
			
		}
		/* Start Control Status Phase */
		do_setup_in_status_phase(pcd);
	}
	return;
}

 /**
 * This function process the GET_STATUS Setup Commands.
 */
static inline void do_get_status(dwc_otg_pcd_t * pcd)
{
	int32_t flag = 0;
	usb_device_request_t ctrl = pcd->setup_pkt->req;
	dwc_otg_pcd_ep_t *ptr_ep;
	dwc_otg_pcd_ep_t *ptr_ep0 = &pcd->ep0;
	uint16_t *ptr_status = pcd->status_buf;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
#endif

	DBG_USB_Print(DBG_PCD,
		    "GET_STATUS %02x.%02x v%04x i%04x l%04x\n",
		    ctrl.bmRequestType, ctrl.bRequest,
		    UGETW(ctrl.wValue), UGETW(ctrl.wIndex),
		    UGETW(ctrl.wLength));

	switch (UT_GET_RECIPIENT(ctrl.bmRequestType)) {
	case UT_DEVICE:
		if (UGETW(ctrl.wIndex) == 0xF000) {	/* OTG Status selector */

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			if ((ptr_core_if->otg_ver == 1)
			    && (ptr_core_if->core_params->otg_cap ==
			    DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE)) {
				uint8_t *otgsts = (uint8_t *) pcd->status_buf;
				*otgsts = (ptr_core_if->otg_sts & 0x1);
				pcd->ep0_pending = 1;
				ptr_ep0->dwc_ep.start_xfer_buff =
				    (uint8_t *) otgsts;
				ptr_ep0->dwc_ep.xfer_buff = (uint8_t *) otgsts;
				ptr_ep0->dwc_ep.dma_addr =
				    pcd->status_buf_dma_handle;
				ptr_ep0->dwc_ep.xfer_len = 1;
				ptr_ep0->dwc_ep.xfer_count = 0;
				ptr_ep0->dwc_ep.total_len = ptr_ep0->dwc_ep.xfer_len;
				dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd),
							   &ptr_ep0->dwc_ep);

			} else {
#endif
				ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			}
#endif
		} else {
			*ptr_status = 0x1U;	/* Self powered */
			*ptr_status |= (uint16_t)((pcd->remote_wakeup_enable) << 1U);
		}
			break;
	case UT_INTERFACE:
		*ptr_status = 0;
		break;

	case UT_ENDPOINT:
		ptr_ep = get_ep_by_addr(pcd, UGETW(ctrl.wIndex));
		if ((ptr_ep == 0) || (UGETW(ctrl.wLength) > 2)) {
			ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
			flag = 1;
		}
		/** @todo check for EP stall */
		if(flag == 0){
		*ptr_status = ptr_ep->stopped;
		}
		break;
	default :		
		break;
	}
	if (flag == 0){
		pcd->ep0_pending = 1;
		ptr_ep0->dwc_ep.start_xfer_buff = (uint8_t *) ptr_status;
		ptr_ep0->dwc_ep.xfer_buff = (uint8_t *) ptr_status;
		ptr_ep0->dwc_ep.dma_addr = pcd->status_buf_dma_handle;
		ptr_ep0->dwc_ep.xfer_len = 2;
		ptr_ep0->dwc_ep.xfer_count = 0;
		ptr_ep0->dwc_ep.total_len = ptr_ep0->dwc_ep.xfer_len;
		dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &ptr_ep0->dwc_ep);
	}
	return;
}

/* receive get_descriptor request */
static inline void do_get_descriptor(dwc_otg_pcd_t *pcd)
{
	usb_device_request_t ctrl = pcd->setup_pkt->req;
	dwc_otg_pcd_ep_t *ptr_ep0 = &pcd->ep0;

	int32_t val = 0;
	
	DBG_USB_Print(DBG_PCD,
		    "GET_STATUS %02x.%02x v%04x i%04x l%04x\n",
		    ctrl.bmRequestType, ctrl.bRequest,
		    UGETW(ctrl.wValue), UGETW(ctrl.wIndex),
		    UGETW(ctrl.wLength));
	val = UGETW(ctrl.wValue) >> 8;

	switch(val)
	{
		case 1:     /* Device */
			
				pcd->ep0_pending = 1;
				ptr_ep0->dwc_ep.start_xfer_buff = (uint8_t *) neu_device_desc_1;
				ptr_ep0->dwc_ep.xfer_buff = (uint8_t *) neu_device_desc_1;
				ptr_ep0->dwc_ep.dma_addr = (dwc_dma_t)neu_device_desc_1;		
				ptr_ep0->dwc_ep.xfer_len = UGETW(ctrl.wLength);
				ptr_ep0->dwc_ep.xfer_len = 0x12;
				ptr_ep0->dwc_ep.xfer_count = 0;
				ptr_ep0->dwc_ep.total_len = ptr_ep0->dwc_ep.xfer_len;
				ptr_ep0->dwc_ep.total_len = 0x12;
				dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd),
							   &ptr_ep0->dwc_ep);		
		
			/* Prepare for more SETUP Packets */
			ep0_out_start(GET_CORE_IF(pcd), pcd);
			break;
		
		case 2:			/* Config */
				pcd->ep0_pending = 1;
				ptr_ep0->dwc_ep.start_xfer_buff = (uint8_t *) neu_config_desc;
				ptr_ep0->dwc_ep.xfer_buff = (uint8_t *) neu_config_desc;
				ptr_ep0->dwc_ep.dma_addr = (dwc_dma_t)neu_config_desc;		
				ptr_ep0->dwc_ep.xfer_len = UGETW(ctrl.wLength);
				ptr_ep0->dwc_ep.xfer_count = 0;
				ptr_ep0->dwc_ep.total_len = UGETW(ctrl.wLength);
				dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd),
							   &ptr_ep0->dwc_ep);		
		
			/* Prepare for more SETUP Packets */
			ep0_out_start(GET_CORE_IF(pcd), pcd);
			break;

		case 4:			/* Interface */
				pcd->ep0_pending = 1;
				ptr_ep0->dwc_ep.start_xfer_buff = (uint8_t *) neu_interface_desc;
				ptr_ep0->dwc_ep.xfer_buff = (uint8_t *) neu_interface_desc;
				ptr_ep0->dwc_ep.dma_addr = (dwc_dma_t)neu_interface_desc;		
				ptr_ep0->dwc_ep.xfer_len = UGETW(ctrl.wLength);
				ptr_ep0->dwc_ep.xfer_count = 0;
				ptr_ep0->dwc_ep.total_len = UGETW(ctrl.wLength);
				dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd),
							   &ptr_ep0->dwc_ep);		
		
			/* Prepare for more SETUP Packets */
			ep0_out_start(GET_CORE_IF(pcd), pcd);
			break;

		case 5:			/*Endpoint */
		  /* val = UGETW(ctrl.wIndex); */
				pcd->ep0_pending = 1;
				ptr_ep0->dwc_ep.start_xfer_buff = (uint8_t *) neu_endpoint_desc;
				ptr_ep0->dwc_ep.xfer_buff = (uint8_t *) neu_endpoint_desc;
				ptr_ep0->dwc_ep.dma_addr = (dwc_dma_t)neu_endpoint_desc;		
				ptr_ep0->dwc_ep.xfer_len = UGETW(ctrl.wLength);
				ptr_ep0->dwc_ep.xfer_count = 0;
				ptr_ep0->dwc_ep.total_len = UGETW(ctrl.wLength);
				dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd),
							   &ptr_ep0->dwc_ep);		
	
			/* Prepare for more SETUP Packets */
			ep0_out_start(GET_CORE_IF(pcd), pcd);

			break;
        
		default :		
		break;
	}
	(void)val;
	return;
}	

/**
 * This function process the SET_FEATURE Setup Commands.
 */
static inline void do_set_feature(dwc_otg_pcd_t * pcd)
{
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
    usb_device_request_t ctrl = pcd->setup_pkt->req;
	dwc_otg_pcd_ep_t *ptr_ep = 0;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED || DBG_USB || DBG_CIL || DBG_CILV || DBG_PCD || DBG_PCDV || DBG_HCD || USB_REG_RD_WR)
	int32_t otg_cap_param = ptr_core_if->core_params->otg_cap;
#endif
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
    dwc_otg_core_global_regs_t *global_regs = ptr_core_if->core_global_regs;
	gotgctl_data_t gbl_otg_ctl;
	gintmsk_data_t gbl_int_msk;
	gbl_otg_ctl.d32 = 0 ;
	gbl_int_msk.d32 = 0 ;
#endif
	DBG_USB_Print(DBG_PCD, "SET_FEATURE:%02x.%02x v%04x i%04x l%04x\n",
		    ctrl.bmRequestType, ctrl.bRequest,
		    UGETW(ctrl.wValue), UGETW(ctrl.wIndex),
		    UGETW(ctrl.wLength));
	DBG_USB_Print(DBG_PCD, "otg_cap=%d\n", otg_cap_param);

	switch (UT_GET_RECIPIENT(ctrl.bmRequestType)) {
	case UT_DEVICE:
		switch (UGETW(ctrl.wValue)) {
		case UF_DEVICE_REMOTE_WAKEUP:
			pcd->remote_wakeup_enable = 1;
			do_setup_in_status_phase(pcd);
			break;

		case UF_TEST_MODE:
			/* Setup the Test Mode tasklet to do the Test
			 * Packet generation after the SETUP Status
			 * phase has completed. */

			/** @todo This has not been tested since the
			 * tasklet struct was put into the PCD
			 * struct! */
			pcd->test_mode = UGETW(ctrl.wIndex) >> 8;
			do_setup_in_status_phase(pcd);
			break;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		case UF_DEVICE_B_HNP_ENABLE:
			DBG_USB_Print(DBG_PCDV,
				    "SET_FEATURE: USB_DEVICE_B_HNP_ENABLE\n");

			/* dev may initiate HNP */
			if (otg_cap_param == DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE) {
				gbl_otg_ctl.b.devhnpen = 1;
				if ((ptr_core_if->otg_ver) != 0) {
					DWC_MODIFY_REG32(&global_regs->gotgctl, 0, gbl_otg_ctl.d32);
					/* Ensure that USB Suspend interrupt is unmasked */
					gbl_int_msk.b.usbsuspend = 1;
					DWC_MODIFY_REG32(&global_regs->gintmsk, 0, gbl_int_msk.d32);
				}
				else {
					pcd->b_hnp_enable = 1;
					dwc_otg_pcd_update_otg(pcd, 0);
					DBG_USB_Print(DBG_PCD, "Request B HNP\n");
					/**@todo Is the gotgctl.devhnpen cleared
					 * by a USB Reset? */
					gbl_otg_ctl.b.hnpreq = 1;
					REG_WR(&global_regs->gotgctl, gbl_otg_ctl.d32);
				}
				do_setup_in_status_phase(pcd);
			} else {
				ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
			}
			break;

		case UF_DEVICE_A_HNP_SUPPORT:
			/* RH port supports HNP */
			DBG_USB_Print(DBG_PCDV,
				    "SET_FEATURE: USB_DEVICE_A_HNP_SUPPORT\n");
			if (otg_cap_param == DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE) {
				pcd->a_hnp_support = 1;
				dwc_otg_pcd_update_otg(pcd, 0);
				do_setup_in_status_phase(pcd);
			} else {
				ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
			}
			break;

		case UF_DEVICE_A_ALT_HNP_SUPPORT:
			/* other RH port does */
			DBG_USB_Print(DBG_PCDV,
				    "SET_FEATURE: USB_DEVICE_A_ALT_HNP_SUPPORT\n");
			if (otg_cap_param == DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE) {
				pcd->a_alt_hnp_support = 1;
				dwc_otg_pcd_update_otg(pcd, 0);
				do_setup_in_status_phase(pcd);
			} else {
				ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
			}
			break;
#endif
		default:
			ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
			break;
		}
		break;

	case UT_INTERFACE:
		do_gadget_setup(pcd, &ctrl);
		break;

	case UT_ENDPOINT:
		if (UGETW(ctrl.wValue) == UF_ENDPOINT_HALT) {
			ptr_ep = get_ep_by_addr(pcd, UGETW(ctrl.wIndex));
			if (ptr_ep == 0) {
				ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
			}else {
				ptr_ep->stopped = 1;
				dwc_otg_ep_set_stall(ptr_core_if, &ptr_ep->dwc_ep);
				do_setup_in_status_phase(pcd);
			}
		}else {
			do_setup_in_status_phase(pcd);
		}
		break;
	default :
		break;
	}
	return;
}

/**
 * This function process the CLEAR_FEATURE Setup Commands.
 */
static inline void do_clear_feature(dwc_otg_pcd_t * pcd)
{
	usb_device_request_t ctrl = pcd->setup_pkt->req;
	dwc_otg_pcd_ep_t *ptr_ep = 0;

	DBG_USB_Print(DBG_PCD,
		    "CLEAR_FEATURE:%02x.%02x v%04x i%04x l%04x\n",
		    ctrl.bmRequestType, ctrl.bRequest,
		    UGETW(ctrl.wValue), UGETW(ctrl.wIndex),
		    UGETW(ctrl.wLength));

	switch (UT_GET_RECIPIENT(ctrl.bmRequestType)) {
	case UT_DEVICE:
		switch (UGETW(ctrl.wValue)) {
		case UF_DEVICE_REMOTE_WAKEUP:
			pcd->remote_wakeup_enable = 0;
			do_setup_in_status_phase(pcd);
			break;

		case UF_TEST_MODE:
			do_setup_in_status_phase(pcd);
			/** @todo Add CLEAR_FEATURE for TEST modes. */
			break;

		default:
			ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
			break;
		}
		
		break;

	case UT_ENDPOINT:
		ptr_ep = get_ep_by_addr(pcd, UGETW(ctrl.wIndex));
		if (ptr_ep == 0) {
			ep0_do_stall(pcd, -DWC_E_NOT_SUPPORTED);
			;
		}else {
			pcd_clear_halt(pcd, ptr_ep);
		}
		break;
	default :		
		break;
	}
	return;
}

/**
 * This function process the SET_ADDRESS Setup Commands.
 */
static inline void do_set_address(dwc_otg_pcd_t * pcd)
{
	dwc_otg_dev_if_t *ptr_dev_if = GET_CORE_IF(pcd)->dev_if;
	usb_device_request_t ctrl = pcd->setup_pkt->req;

	if (ctrl.bmRequestType == UT_DEVICE) {
		dcfg_data_t dev_cfg;
		dev_cfg.d32 = 0 ;

		dev_cfg.b.devaddr = UGETW(ctrl.wValue);
		
		DWC_MODIFY_REG32(&ptr_dev_if->dev_global_regs->dcfg, 0, dev_cfg.d32);
		do_setup_in_status_phase(pcd);
	}
}

/**
 *	This function processes SETUP commands. In Linux, the USB Command
 *	processing is done in two places - the first being the PCD and the
 *	second in the Gadget Driver (for example, the File-Backed Storage
 *	Gadget Driver).
 *
 * <table>
 * <tr><td>Command	</td><td>Driver </td><td>Description</td></tr>
 *
 * <tr><td>GET_STATUS </td><td>PCD </td><td>Command is processed as
 * defined in chapter 9 of the USB 2.0 Specification chapter 9
 * </td></tr>
 *
 * <tr><td>CLEAR_FEATURE </td><td>PCD </td><td>The Device and Endpoint
 * requests are the ENDPOINT_HALT feature is procesed, all others the
 * interface requests are ignored.</td></tr>
 *
 * <tr><td>SET_FEATURE </td><td>PCD </td><td>The Device and Endpoint
 * requests are processed by the PCD.  Interface requests are passed
 * to the Gadget Driver.</td></tr>
 *
 * <tr><td>SET_ADDRESS </td><td>PCD </td><td>Program the DCFG reg,
 * with device address received </td></tr>
 *
 * <tr><td>GET_DESCRIPTOR </td><td>Gadget Driver </td><td>Return the
 * requested descriptor</td></tr>
 *
 * <tr><td>SET_DESCRIPTOR </td><td>Gadget Driver </td><td>Optional -
 * not implemented by any of the existing Gadget Drivers.</td></tr>
 *
 * <tr><td>SET_CONFIGURATION </td><td>Gadget Driver </td><td>Disable
 * all EPs and enable EPs for new configuration.</td></tr>
 *
 * <tr><td>GET_CONFIGURATION </td><td>Gadget Driver </td><td>Return
 * the current configuration</td></tr>
 *
 * <tr><td>SET_INTERFACE </td><td>Gadget Driver </td><td>Disable all
 * EPs and enable EPs for new configuration.</td></tr>
 *
 * <tr><td>GET_INTERFACE </td><td>Gadget Driver </td><td>Return the
 * current interface.</td></tr>
 *
 * <tr><td>SYNC_FRAME </td><td>PCD </td><td>Display debug
 * message.</td></tr>
 * </table>
 *
 * When the SETUP Phase Done interrupt occurs, the PCD SETUP commands are
 * processed by pcd_setup. Calling the Function Driver's setup function from
 * pcd_setup processes the gadget SETUP commands.
 */

static inline void pcd_setup(dwc_otg_pcd_t * pcd)
{
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
	dwc_otg_dev_if_t *ptr_dev_if = ptr_core_if->dev_if;
#endif
	usb_device_request_t ctrl = pcd->setup_pkt->req;
	dwc_otg_pcd_ep_t *ptr_ep0 = &pcd->ep0;
	uint32_t reg_addr = 0,reg_val = 0;
	int32_t flag = 0;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	deptsiz0_data_t doeptsize0;
	doeptsize0.d32 = 0 ;

	doeptsize0.d32 = REG_RD(&ptr_dev_if->out_ep_regs[0]->doeptsiz);
	/** In BDMA more then 1 setup packet is not supported till 3.00a */
	if (((ptr_core_if->dma_enable) && (ptr_core_if->dma_desc_enable == 0))
	    && (doeptsize0.b.supcnt < 2)
	    && (ptr_core_if->snpsid < OTG_CORE_REV_2_94a)) {
		DBG_Error_Print
		    ("\n\n-----------	 CANNOT handle > 1 setup packet in DMA mode\n\n");
	}

	if ((ptr_core_if->snpsid >= OTG_CORE_REV_3_00a)
	    && (ptr_core_if->dma_enable == 1) && (ptr_core_if->dma_desc_enable == 0)) {
		if ((doeptsize0.b.supcnt == 3) && (ptr_ep0->dwc_ep.stp_rollover == 0)) {
			DBG_Error_Print(" !!! Setup packet count was not updated by the core\n");
			flag = 1;
		}else {
			ctrl =
				(pcd->setup_pkt +
				 (3 - (doeptsize0.b.supcnt - 1) +
				  ptr_ep0->dwc_ep.stp_rollover))->req;
		}
	}
#endif
		if(flag == 0){
			/* Clean up the request queue */
			ptr_ep0->stopped = 0;

			if ((ctrl.bmRequestType & UE_DIR_IN) != 0) {
				ptr_ep0->dwc_ep.is_in = 1;
				pcd->ep0state = EP0_IN_DATA_PHASE;
			} else {
				ptr_ep0->dwc_ep.is_in = 0;
				pcd->ep0state = EP0_OUT_DATA_PHASE;
			}

			if (UGETW(ctrl.wLength) == 0) {
				ptr_ep0->dwc_ep.is_in = 1;
				pcd->ep0state = EP0_IN_STATUS_PHASE;
			}
			if (UT_GET_TYPE(ctrl.bmRequestType) != UT_STANDARD) {
			
			
				if (UT_GET_TYPE(ctrl.bmRequestType) == UT_VENDOR) {
					
					if(ctrl.bRequest == 0x01)
					{
							reg_addr = UGETW(ctrl.wValue);
							reg_addr = 0x40000000 + reg_addr;
					}

					if(ctrl.bRequest == 0x02)
					{
							reg_addr = UGETW(ctrl.wValue);
							reg_addr = 0x40010000 + reg_addr;
					}

					/* Request from host to read data from Register */		
					if(	ptr_ep0->dwc_ep.is_in != 0)			
					{	
						reg_val = REG_RD((uint32_t *)reg_addr);
						*((uint32_t *)pcd->status_buf_dma_handle) = reg_val;
						pcd->ep0_pending = 1;
						ptr_ep0->dwc_ep.dma_addr = pcd->status_buf_dma_handle;
						ptr_ep0->dwc_ep.xfer_len = UGETW(ctrl.wLength);
						ptr_ep0->dwc_ep.xfer_len = UGETW(ctrl.wLength);
						ptr_ep0->dwc_ep.xfer_count = 0;
						ptr_ep0->dwc_ep.total_len = ptr_ep0->dwc_ep.xfer_len;
						ptr_ep0->dwc_ep.total_len = UGETW(ctrl.wLength);
						dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd),
									   &ptr_ep0->dwc_ep);		
					}		
					/* Request from host to Write data to Register */		
					if(	ptr_ep0->dwc_ep.is_in == 0)			
					{	
						pcd->ep0_pending = 1;
						ptr_ep0->dwc_ep.dma_addr = pcd->status_buf_dma_handle;
						ptr_ep0->dwc_ep.xfer_len = UGETW(ctrl.wLength);
						ptr_ep0->dwc_ep.xfer_len = UGETW(ctrl.wLength);
						ptr_ep0->dwc_ep.xfer_count = 0;
						ptr_ep0->dwc_ep.total_len = ptr_ep0->dwc_ep.xfer_len;
						ptr_ep0->dwc_ep.total_len = UGETW(ctrl.wLength);
							
						dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd),
									 &ptr_ep0->dwc_ep);		
					}		

				}		
			}else {
				/* --- Standard Request handling --- */

				switch (ctrl.bRequest) {
				case UR_GET_STATUS:
					do_get_status(pcd);
					break;

				case UR_CLEAR_FEATURE:
					do_clear_feature(pcd);
					break;

				case UR_SET_FEATURE:
					do_set_feature(pcd);
					break;

				case UR_SET_ADDRESS:
					do_set_address(pcd);
					break;

				case UR_SET_INTERFACE:
				case UR_SET_CONFIG:
				do_setup_in_status_phase(pcd);
				
				/* Call this function to get Initialize bulk endpoint after enumeration process */
				do_gadget_setup(pcd, &ctrl);
					break;

				case UR_SYNCH_FRAME:
					do_gadget_setup(pcd, &ctrl);
					break;
				
				case UR_GET_DESCRIPTOR:
					do_get_descriptor(pcd);
					break;
				default:
					break;
				}
			}
		}
	return;
}

/**
 * This function completes the ep0 control transfer.
 */
static int32_t ep0_complete_request(dwc_otg_pcd_ep_t * ep)
{
	int32_t retval = 0;
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(ep->pcd);
	dwc_otg_dev_if_t *ptr_dev_if = ptr_core_if->dev_if;
	dwc_otg_dev_in_ep_regs_t *ptr_in_ep_regs =
	    ptr_dev_if->in_ep_regs[ep->dwc_ep.num];
	dwc_otg_dev_out_ep_regs_t *ptr_out_ep_regs =
	    ptr_dev_if->out_ep_regs[ep->dwc_ep.num];
	deptsiz0_data_t deptsiz;
	dev_dma_desc_sts_t desc_sts;
	dwc_otg_pcd_request_t *ptr_req;
	int32_t is_last = 0;
	dwc_otg_pcd_t *ptr_pcd = ep->pcd;
	desc_sts.d32 = 0;
	deptsiz.d32 = 0;

	if ((ptr_pcd->ep0_pending) && (DWC_CIRCLEQ_EMPTY(&ep->queue))) {
		if ((ep->dwc_ep.is_in) != 0) {
			DBG_USB_Print(DBG_PCDV, "Do setup OUT status phase\n");
			do_setup_out_status_phase(ptr_pcd);
		} else {
			DBG_USB_Print(DBG_PCDV, "Do setup IN status phase\n");

			do_setup_in_status_phase(ptr_pcd);
		}
		ptr_pcd->ep0_pending = 0;
		retval = 1;
	}else {

		if (DWC_CIRCLEQ_EMPTY(&ep->queue)) {
			retval = 0;
		}else {
			ptr_req = DWC_CIRCLEQ_FIRST(&ep->queue);

			if ((ptr_pcd->ep0state == EP0_OUT_STATUS_PHASE)
				|| (ptr_pcd->ep0state == EP0_IN_STATUS_PHASE)) {
				is_last = 1;
			} else if (ep->dwc_ep.is_in != 0) {
				deptsiz.d32 = REG_RD(&ptr_in_ep_regs->dieptsiz);
				if (ptr_core_if->dma_desc_enable != 0) {
					desc_sts = ptr_dev_if->in_desc_addr->status;
				}
				DBG_USB_Print(DBG_PCDV, "%d len=%d  xfersize=%d pktcnt=%d\n",
						ep->dwc_ep.num, ep->dwc_ep.xfer_len,
						deptsiz.b.xfersize, deptsiz.b.pktcnt);

				if (((ptr_core_if->dma_desc_enable == 0)
					 && (deptsiz.b.xfersize == 0))
					|| ((ptr_core_if->dma_desc_enable != 0)
					&& (desc_sts.b.bytes == 0))) {
					ptr_req->actual = ep->dwc_ep.xfer_count;
					/* Is a Zero Len Packet needed? */
					if (ptr_req->sent_zlp != 0) {
						DBG_USB_Print(DBG_PCD, "Setup Rx ZLP\n");
						ptr_req->sent_zlp = 0;
					}
					do_setup_out_status_phase(ptr_pcd);
				}
			} else {
				/* ep0-OUT */
				deptsiz.d32 = REG_RD(&ptr_out_ep_regs->doeptsiz);
				DBG_USB_Print(DBG_PCDV, "%d len=%d xsize=%d pktcnt=%d\n",
						ep->dwc_ep.num, ep->dwc_ep.xfer_len,
						deptsiz.b.xfersize, deptsiz.b.pktcnt);
				ptr_req->actual = ep->dwc_ep.xfer_count;

				/* Is a Zero Len Packet needed? */
				if (ptr_req->sent_zlp != 0) {
					DBG_USB_Print(DBG_PCDV, "Setup Tx ZLP\n");
					ptr_req->sent_zlp = 0;
				}
				/* For older cores do setup in status phase in Slave/BDMA modes, 
				 * starting from 3.00 do that only in slave, and for DMA modes 
				 * just re-enable ep 0 OUT here*/
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
				if ((ptr_core_if->dma_enable == 0)
					|| ((ptr_core_if->dma_desc_enable == 0)
					&& (ptr_core_if->snpsid <= OTG_CORE_REV_2_94a))) {
					do_setup_in_status_phase(ptr_pcd);
				}
#endif
                else if (ptr_core_if->snpsid >= OTG_CORE_REV_3_00a) {
                DBG_USB_Print(DBG_PCDV,
                        "Enable out ep before in status phase\n");
                ep0_out_start(ptr_core_if, ptr_pcd);
				}
			}

			/* Complete the request */
			if (is_last != 0) {
				dwc_otg_request_done(ep, ptr_req, 0);
				ep->dwc_ep.start_xfer_buff = 0;
				ep->dwc_ep.xfer_buff = 0;
				ep->dwc_ep.xfer_len = 0;
				retval = 1;
			}
		}
	}
	(void)deptsiz.d32;
	return retval;
}

static void complete_ep(dwc_otg_pcd_ep_t * ep)
{
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(ep->pcd);
	dwc_otg_dev_if_t *ptr_dev_if = ptr_core_if->dev_if;
	dwc_otg_dev_in_ep_regs_t *ptr_in_ep_regs =
	    ptr_dev_if->in_ep_regs[ep->dwc_ep.num];
	deptsiz_data_t deptsiz;
	dev_dma_desc_sts_t desc_sts;
	dwc_otg_pcd_request_t *ptr_req = 0;
	dwc_otg_dev_dma_desc_t *dma_desc;
	uint32_t byte_count = 0;
	int32_t is_last = 0;
	uint32_t i;
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	DBG_USB_Print(DBG_PCDV, "%s() %d-%s\n", __func__, ep->dwc_ep.num,
		    (ep->dwc_ep.is_in ? "IN" : "OUT"));
	
	#if 1 
	if (ep->dwc_ep.is_in  != 0) 
	{
		DWC_FREE((void *)ep->dwc_ep.dma_addr);
		DBG_USB_Print(DBG_USB,"Free=%05x\n", ep->dwc_ep.dma_addr);

		if(ep->dwc_ep.num < 5)
		{
			DBG_Error_Print("Wrong in USB end point: %d\n",ep->dwc_ep.num);
		}
		else
		{
			malloc_free_counter[(((int32_t)ep->dwc_ep.num) - 5)] --;
		}
	}
	#endif

	/* Get any pending requests */
	if (!DWC_CIRCLEQ_EMPTY(&ep->queue)) {
		ptr_req = DWC_CIRCLEQ_FIRST(&ep->queue);
		if (!ptr_req) {
			DBG_USB_Print(DBG_USB,"complete_ep 0x%p, req = NULL!\n", ep);
		}else {
			DBG_USB_Print(DBG_PCD, "Requests %d\n", ep->pcd->request_pending);
			if (ep->dwc_ep.is_in != 0) {
				deptsiz.d32 = REG_RD(&ptr_in_ep_regs->dieptsiz);
				if (ptr_core_if->dma_enable != 0) {
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
					if (ptr_core_if->dma_desc_enable == 0) {
						if ((deptsiz.b.xfersize == 0)
							&& (deptsiz.b.pktcnt == 0)) {
							byte_count =
								ep->dwc_ep.xfer_len -
								ep->dwc_ep.xfer_count;

							ep->dwc_ep.xfer_buff += byte_count;
							ep->dwc_ep.dma_addr += byte_count;
							ep->dwc_ep.xfer_count += byte_count;

							DBG_USB_Print(DBG_PCDV,
									"%d-%s len=%d  xfersize=%d pktcnt=%d\n",
									ep->dwc_ep.num,
									(ep->dwc_ep.
									 is_in ? "IN" : "OUT"),
									ep->dwc_ep.xfer_len,
									deptsiz.b.xfersize,
									deptsiz.b.pktcnt);

							if (ep->dwc_ep.xfer_len <
								ep->dwc_ep.total_len) {
								dwc_otg_ep_start_transfer
									(ptr_core_if, &ep->dwc_ep);
							} else if (ep->dwc_ep.sent_zlp != 0) {
								/*     
								 * This fragment of code should initiate 0
								 * length transfer in case if it is queued
								 * a transfer with size divisible to EPs max
								 * packet size and with usb_request zero field
								 * is set, which means that after data is transfered,
								 * it is also should be transfered
								 * a 0 length packet at the end. For Slave and
								 * Buffer DMA modes in this case SW has
								 * to initiate 2 transfers one with transfer size,
								 * and the second with 0 size. For Descriptor
								 * DMA mode SW is able to initiate a transfer,
								 * which will handle all the packets including
								 * the last  0 length.
								 */
								ep->dwc_ep.sent_zlp = 0;
								dwc_otg_ep_start_zl_transfer
									(ptr_core_if, &ep->dwc_ep);
							} else {
								is_last = 1;
							}
						} else {
								DBG_Warn_Print
								("Incomplete transfer (%d - %s [siz=%d pkt=%d])\n",
								ep->dwc_ep.num,
								(ep->dwc_ep.is_in ? "IN" : "OUT"),
								deptsiz.b.xfersize,
								deptsiz.b.pktcnt);
						}
					} else {  /*FLOW */
#endif
						dma_desc = ep->dwc_ep.desc_addr;
						byte_count = 0;
						ep->dwc_ep.sent_zlp = 0;

							for (i = 0; i < ep->dwc_ep.desc_cnt;
								 ++i) {
								desc_sts = dma_desc->status;
								byte_count += desc_sts.b.bytes;
								dma_desc++;
							}
						if (byte_count == 0) {
							ep->dwc_ep.xfer_count =
								ep->dwc_ep.total_len;
							is_last = 1;
						} else {
							DBG_Warn_Print("Incomplete transfer\n");
						}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
					}
#endif
				} 
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
                else {
					if ((deptsiz.b.xfersize == 0) && (deptsiz.b.pktcnt == 0)) {
						DBG_USB_Print(DBG_PCDV,
								"%d-%s len=%d  xfersize=%d pktcnt=%d\n",
								ep->dwc_ep.num,
								ep->dwc_ep.is_in ? "IN" : "OUT",
								ep->dwc_ep.xfer_len,
								deptsiz.b.xfersize,
								deptsiz.b.pktcnt);

						/*      Check if the whole transfer was completed, 
						 *      if no, setup transfer for next portion of data
						 */
						if (ep->dwc_ep.xfer_len < ep->dwc_ep.total_len) {
							dwc_otg_ep_start_transfer(ptr_core_if,
										  &ep->dwc_ep);
						} else if (ep->dwc_ep.sent_zlp != 0) {
							/*     
							 * This fragment of code should initiate 0
							 * length trasfer in case if it is queued
							 * a trasfer with size divisible to EPs max
							 * packet size and with usb_request zero field
							 * is set, which means that after data is transfered,
							 * it is also should be transfered
							 * a 0 length packet at the end. For Slave and
							 * Buffer DMA modes in this case SW has
							 * to initiate 2 transfers one with transfer size,
							 * and the second with 0 size. For Desriptor
							 * DMA mode SW is able to initiate a transfer,
							 * which will handle all the packets including
							 * the last  0 legth.
							 */
							ep->dwc_ep.sent_zlp = 0;
							dwc_otg_ep_start_zl_transfer(ptr_core_if,
											 &ep->dwc_ep);
						} else {
							is_last = 1;
						}
					} else {
						DBG_Warn_Print
							("Incomplete transfer (%d-%s [siz=%d pkt=%d])\n",
							 ep->dwc_ep.num,
							 (ep->dwc_ep.is_in ? "IN" : "OUT"),
							 deptsiz.b.xfersize, deptsiz.b.pktcnt);
					}
				}
#endif
			} else {
				dwc_otg_dev_out_ep_regs_t *ptr_out_ep_regs =
					ptr_dev_if->out_ep_regs[ep->dwc_ep.num];
				desc_sts.d32 = 0;
				if (ptr_core_if->dma_enable != 0) {
					if (ptr_core_if->dma_desc_enable != 0) {
						dma_desc = ep->dwc_ep.desc_addr;
						byte_count = 0;
						ep->dwc_ep.sent_zlp = 0;


							for (i = 0; i < ep->dwc_ep.desc_cnt;
								 ++i) {
								desc_sts = dma_desc->status;
								byte_count += desc_sts.b.bytes;
								dma_desc++;
							}

						/* Checking for interrupt Out transfers with not 
						 * dword aligned mps sizes 
						 */
						/* FLOW: */
							ep->dwc_ep.xfer_count =
								(ep->dwc_ep.total_len - byte_count) +
								((4 -
								  (ep->dwc_ep.
								   total_len & 0x3U)) & 0x3U);
							is_last = 1;
						
					} 
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
                    else {
						deptsiz.d32 = 0;
						deptsiz.d32 =
							REG_RD(&ptr_out_ep_regs->doeptsiz);

						byte_count = (ep->dwc_ep.xfer_len -
								  ep->dwc_ep.xfer_count -
								  deptsiz.b.xfersize);
						ep->dwc_ep.xfer_buff += byte_count;
						ep->dwc_ep.dma_addr += byte_count;
						ep->dwc_ep.xfer_count += byte_count;

						/*      Check if the whole transfer was completed, 
						 *      if no, setup transfer for next portion of data
						 */
						if (ep->dwc_ep.xfer_len < ep->dwc_ep.total_len) {
							dwc_otg_ep_start_transfer(ptr_core_if,
										  &ep->dwc_ep);
						} else if (ep->dwc_ep.sent_zlp != 0) {
							/*     
							 * This fragment of code should initiate 0
							 * length trasfer in case if it is queued
							 * a trasfer with size divisible to EPs max
							 * packet size and with usb_request zero field
							 * is set, which means that after data is transfered,
							 * it is also should be transfered
							 * a 0 length packet at the end. For Slave and
							 * Buffer DMA modes in this case SW has
							 * to initiate 2 transfers one with transfer size,
							 * and the second with 0 size. For Desriptor
							 * DMA mode SW is able to initiate a transfer,
							 * which will handle all the packets including
							 * the last  0 legth.
							 */
							ep->dwc_ep.sent_zlp = 0;
							dwc_otg_ep_start_zl_transfer(ptr_core_if,
											 &ep->dwc_ep);
						} else {
							is_last = 1;
						}
					}
#endif
				} 
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
                else {
					/*      Check if the whole transfer was completed, 
					 *      if no, setup transfer for next portion of data
					 */
					if (ep->dwc_ep.xfer_len < ep->dwc_ep.total_len) {
						dwc_otg_ep_start_transfer(ptr_core_if, &ep->dwc_ep);
					} else if (ep->dwc_ep.sent_zlp != 0) {
						/*     
						 * This fragment of code should initiate 0
						 * length transfer in case if it is queued
						 * a transfer with size divisible to EPs max
						 * packet size and with usb_request zero field
						 * is set, which means that after data is transfered,
						 * it is also should be transfered
						 * a 0 length packet at the end. For Slave and
						 * Buffer DMA modes in this case SW has
						 * to initiate 2 transfers one with transfer size,
						 * and the second with 0 size. For Descriptor
						 * DMA mode SW is able to initiate a transfer,
						 * which will handle all the packets including
						 * the last  0 length.
						 */
						ep->dwc_ep.sent_zlp = 0;
						dwc_otg_ep_start_zl_transfer(ptr_core_if,
										 &ep->dwc_ep);
					} else {
						is_last = 1;
					}
				}
#endif

				DBG_USB_Print(DBG_PCDV,
						"addr %p,	 %d-%s len=%d cnt=%d xsize=%d pktcnt=%d\n",
						&ptr_out_ep_regs->doeptsiz, ep->dwc_ep.num,
						ep->dwc_ep.is_in ? "IN" : "OUT",
						ep->dwc_ep.xfer_len, ep->dwc_ep.xfer_count,
						deptsiz.b.xfersize, deptsiz.b.pktcnt);
                
                #if(DBG_USB == 0)
                    (void)ptr_out_ep_regs;
                #endif

			}
			/* Complete the request */
			if (is_last != 0) {
					ptr_req->actual = ep->dwc_ep.xfer_count;
				if (ptr_req->dw_align_buf != NULL) {
					if (!ep->dwc_ep.is_in) {
						dwc_memcpy(ptr_req->buf, ptr_req->dw_align_buf, ptr_req->length); 
					}
					DWC_FREE(ptr_req->dw_align_buf);
				}

				dwc_otg_request_done(ep, ptr_req, 0);

				ep->dwc_ep.start_xfer_buff = 0;
				ep->dwc_ep.xfer_buff = 0;
				ep->dwc_ep.xfer_len = 0;
				/* If there is a request in the queue start it. */
				start_next_request(ep);
			}
		}
	} else {
		DBG_USB_Print(DBG_USB,"complete_ep 0x%p, ep->queue empty!\n", ep);
	}
	(void)deptsiz;
	return;
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
/**
 * This function handle BNA interrupt for Non Isochronous EPs
 *
 */
static void dwc_otg_pcd_handle_noniso_bna(dwc_otg_pcd_ep_t * ep)
{
	dwc_ep_t *ptr_dwc_ep = &ep->dwc_ep;
	volatile uint32_t *ptr_addr;
	depctl_data_t depctl;
	dwc_otg_pcd_t *ptr_pcd = ep->pcd;
	dwc_otg_dev_dma_desc_t *dma_desc;
	dev_dma_desc_sts_t dma_sts;
	uint32_t i, uint_start;
	dwc_otg_core_if_t *ptr_core_if = ep->pcd->core_if;
	depctl.d32 = 0;
	dma_sts.d32 = 0 ;

	if (!ptr_dwc_ep->desc_cnt)
		DBG_Warn_Print("Ep%d %s Descriptor count = %d \n", ptr_dwc_ep->num,
			 (ptr_dwc_ep->is_in ? "IN" : "OUT"), ptr_dwc_ep->desc_cnt);

	if (((ptr_core_if->core_params->cont_on_bna) && (!ptr_dwc_ep->is_in))
							&& (ptr_dwc_ep->type != DWC_OTG_EP_TYPE_CONTROL)) {
		uint32_t uint_doepdma;
		dwc_otg_dev_out_ep_regs_t *out_regs =
			ptr_core_if->dev_if->out_ep_regs[ptr_dwc_ep->num];
		uint_doepdma = REG_RD(&(out_regs->doepdma));
		uint_start = (uint_doepdma - ptr_dwc_ep->dma_desc_addr)/sizeof(dwc_otg_dev_dma_desc_t);
		dma_desc = &(ptr_dwc_ep->desc_addr[uint_start]);
	} else {
		uint_start = 0;
		dma_desc = ptr_dwc_ep->desc_addr;
	}
	

	for (i = uint_start; i < ptr_dwc_ep->desc_cnt; ++i) {
		 ++dma_desc;
		dma_sts.d32 = dma_desc->status.d32;
		dma_sts.b.bs = BS_HOST_READY;
		dma_desc->status.d32 = dma_sts.d32;
	}

	if (ptr_dwc_ep->is_in == 0) {
		ptr_addr =
		    &GET_CORE_IF(ptr_pcd)->dev_if->out_ep_regs[ptr_dwc_ep->num]->
		    doepctl;
	} else {
		ptr_addr =
		    &GET_CORE_IF(ptr_pcd)->dev_if->in_ep_regs[ptr_dwc_ep->num]->diepctl;
	}
	depctl.b.epena = 1;
	depctl.b.cnak = 1;
	DWC_MODIFY_REG32(ptr_addr, 0, depctl.d32);
}
#endif

/**
 * This function handles EP0 Control transfers.
 *
 * The state of the control transfers are tracked in
 * <code>ep0state</code>.
 */
static void handle_ep0(dwc_otg_pcd_t * pcd)
{
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
	dwc_otg_pcd_ep_t *ptr_ep0 = &pcd->ep0;
	dev_dma_desc_sts_t desc_sts;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	deptsiz0_data_t deptsiz;
#endif
	usb_device_request_t ctrl = pcd->setup_pkt->req;
	uint32_t byte_count, reg_addr = 0, reg_val = 0;

	DBG_USB_Print(DBG_PCDV, "%s()\n", __func__);
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	switch (pcd->ep0state) {
	case EP0_DISCONNECT:
		break;

	case EP0_IDLE:
		pcd->request_config = 0;

		pcd_setup(pcd);
		break;

	case EP0_IN_DATA_PHASE:
		DBG_USB_Print(DBG_PCD, "DATA_IN EP%d-%s: type=%d, mps=%d\n",
			    ptr_ep0->dwc_ep.num, (ptr_ep0->dwc_ep.is_in ? "IN" : "OUT"),
			    ptr_ep0->dwc_ep.type, ptr_ep0->dwc_ep.maxpacket);

		if (ptr_core_if->dma_enable != 0) {
			/*
			 * For EP0 we can only program 1 packet at a time so we
			 * need to do the make calculations after each complete.
			 * Call write_packet to make the calculations, as in
			 * slave mode, and use those values to determine if we
			 * can complete.
			 */
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			if (ptr_core_if->dma_desc_enable == 0) {
				deptsiz.d32 =
				    REG_RD(&ptr_core_if->
						   dev_if->in_ep_regs[0]->
						   dieptsiz);
			  DBG_USB_Print(USB_REG_RD_WR,"REG RD From HOST, Adrs = %lx, Value = %lx\n", 
					     &ptr_core_if->dev_if->in_ep_regs[0]->dieptsiz, deptsiz.d32);
				
				byte_count =
				    ptr_ep0->dwc_ep.xfer_len - deptsiz.b.xfersize;
			} else {
#endif
				desc_sts =
				    ptr_core_if->dev_if->in_desc_addr->status;
				byte_count =
				    ptr_ep0->dwc_ep.xfer_len - desc_sts.b.bytes;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			}
#endif
			ptr_ep0->dwc_ep.xfer_count += byte_count;
			ptr_ep0->dwc_ep.xfer_buff += byte_count;
			ptr_ep0->dwc_ep.dma_addr += byte_count;
		}
		if (ptr_ep0->dwc_ep.xfer_count < ptr_ep0->dwc_ep.total_len) {
			dwc_otg_ep0_continue_transfer(GET_CORE_IF(pcd),
						      &ptr_ep0->dwc_ep);
			DBG_USB_Print(DBG_PCD, "CONTINUE TRANSFER\n");
		} else if (ptr_ep0->dwc_ep.sent_zlp != 0) {
			dwc_otg_ep0_continue_transfer(GET_CORE_IF(pcd),
						      &ptr_ep0->dwc_ep);
			ptr_ep0->dwc_ep.sent_zlp = 0;
			DBG_USB_Print(DBG_PCD, "CONTINUE TRANSFER sent zlp\n");
		} else {
			ep0_complete_request(ptr_ep0);
			DBG_USB_Print(DBG_PCD, "COMPLETE TRANSFER\n");
		}
		break;
	case EP0_OUT_DATA_PHASE:
		
	if (UT_GET_TYPE(ctrl.bmRequestType) != UT_STANDARD) {
		
		
			if (UT_GET_TYPE(ctrl.bmRequestType) == UT_VENDOR) {
				if(ctrl.bRequest == 0x01)
				{
						reg_addr = UGETW(ctrl.wValue);
						reg_addr = 0x40000000 + reg_addr;
				}

				if(ctrl.bRequest == 0x02)
				{
						reg_addr = UGETW(ctrl.wValue);
						reg_addr = 0x40010000 + reg_addr;
				}
					reg_val = *((uint32_t *)ptr_ep0->dwc_ep.dma_addr);
				  DBG_USB_Print(USB_REG_RD_WR,"REG WR From HOST, Adrs = %lx, Value = %lx\n", reg_addr, reg_val);
					REG_WR((uint32_t *)reg_addr,reg_val);			
			}
		}
		DBG_USB_Print(DBG_PCD, "DATA_OUT EP%d-%s: type=%d, mps=%d\n",
			    ptr_ep0->dwc_ep.num, (ptr_ep0->dwc_ep.is_in ? "IN" : "OUT"),
			    ptr_ep0->dwc_ep.type, ptr_ep0->dwc_ep.maxpacket);
		if (ptr_core_if->dma_enable != 0) {
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			if (ptr_core_if->dma_desc_enable == 0) {
				deptsiz.d32 =
				    REG_RD(&ptr_core_if->
						   dev_if->out_ep_regs[0]->
						   doeptsiz);
				byte_count =
				    ptr_ep0->dwc_ep.maxpacket - deptsiz.b.xfersize;
			} else {
#endif
				desc_sts =
				    ptr_core_if->dev_if->out_desc_addr->status;
				byte_count =
				    ptr_ep0->dwc_ep.maxpacket - desc_sts.b.bytes;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			}
#endif
			ptr_ep0->dwc_ep.xfer_count += byte_count;
			ptr_ep0->dwc_ep.xfer_buff += byte_count;
			ptr_ep0->dwc_ep.dma_addr += byte_count;
		}
		if (ptr_ep0->dwc_ep.xfer_count < ptr_ep0->dwc_ep.total_len) {
			dwc_otg_ep0_continue_transfer(GET_CORE_IF(pcd),
						      &ptr_ep0->dwc_ep);
			DBG_USB_Print(DBG_PCD, "CONTINUE TRANSFER\n");
		} else if (ptr_ep0->dwc_ep.sent_zlp != 0) {
			dwc_otg_ep0_continue_transfer(GET_CORE_IF(pcd),
						      &ptr_ep0->dwc_ep);
			ptr_ep0->dwc_ep.sent_zlp = 0;
			DBG_USB_Print(DBG_PCD, "CONTINUE TRANSFER sent zlp\n");
		} else {
			ep0_complete_request(ptr_ep0);
			DBG_USB_Print(DBG_PCD, "COMPLETE TRANSFER\n");
		}
		break;

	case EP0_IN_STATUS_PHASE:
	case EP0_OUT_STATUS_PHASE:
		DBG_USB_Print(DBG_PCD, "CASE: EP0_STATUS\n");
		ep0_complete_request(ptr_ep0);
		pcd->ep0state = EP0_IDLE;
		ptr_ep0->stopped = 1;
		ptr_ep0->dwc_ep.is_in = 0;	/* OUT for next SETUP */
		/* Prepare for more SETUP Packets */
		if (ptr_core_if->dma_enable != 0) {
			ep0_out_start(ptr_core_if, pcd);
		}
		break;

	case EP0_STALL:
		DBG_Error_Print("EP0 STALLed, should not get here pcd_setup()\n");
		break;
	default :
		break;
	}
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
/**
 * Restart transfer
 */
static void restart_transfer(dwc_otg_pcd_t * pcd, const uint32_t epnum)
{
	dwc_otg_core_if_t *ptr_core_if;
	dwc_otg_dev_if_t *ptr_dev_if;
	deptsiz_data_t dev_ep_tsiz;
	dwc_otg_pcd_ep_t *ptr_ep;
	dev_ep_tsiz.d32 = 0 ;

	ptr_ep = get_in_ep(pcd, epnum);

	ptr_core_if = GET_CORE_IF(pcd);
	ptr_dev_if = ptr_core_if->dev_if;

	dev_ep_tsiz.d32 = REG_RD(&ptr_dev_if->in_ep_regs[epnum]->dieptsiz);

	DBG_USB_Print(DBG_PCD, "xfer_buff=%p xfer_count=%0x xfer_len=%0x"
		    " stopped=%d\n", ptr_ep->dwc_ep.xfer_buff,
		    ptr_ep->dwc_ep.xfer_count, ptr_ep->dwc_ep.xfer_len, ptr_ep->stopped);
	/*
	 * If xfersize is 0 and pktcnt in not 0, resend the last packet.
	 */
	if (((dev_ep_tsiz.b.pktcnt) && (dev_ep_tsiz.b.xfersize == 0)) &&
	    (ptr_ep->dwc_ep.start_xfer_buff != 0)) {
		if (ptr_ep->dwc_ep.total_len <= ptr_ep->dwc_ep.maxpacket) {
			ptr_ep->dwc_ep.xfer_count = 0;
			ptr_ep->dwc_ep.xfer_buff = ptr_ep->dwc_ep.start_xfer_buff;
			ptr_ep->dwc_ep.xfer_len = ptr_ep->dwc_ep.xfer_count;
		} else {
			ptr_ep->dwc_ep.xfer_count -= ptr_ep->dwc_ep.maxpacket;
			/* convert packet size to dwords. */
			ptr_ep->dwc_ep.xfer_buff -= ptr_ep->dwc_ep.maxpacket;
			ptr_ep->dwc_ep.xfer_len = ptr_ep->dwc_ep.xfer_count;
		}
		ptr_ep->stopped = 0;
		DBG_USB_Print(DBG_PCD, "xfer_buff=%p xfer_count=%0x "
			    "xfer_len=%0x stopped=%d\n",
			    ptr_ep->dwc_ep.xfer_buff,
			    ptr_ep->dwc_ep.xfer_count, ptr_ep->dwc_ep.xfer_len,
			    ptr_ep->stopped);
		if (epnum == 0) {
			dwc_otg_ep0_start_transfer(ptr_core_if, &ptr_ep->dwc_ep);
		} else {
			dwc_otg_ep_start_transfer(ptr_core_if, &ptr_ep->dwc_ep);
		}
	}
}

/*
 * This function create new nextep sequnce based on Learn Queue.
 *
 * @param core_if Programming view of DWC_otg controller
 */
void predict_nextep_seq( dwc_otg_core_if_t * core_if)
{
	dwc_otg_device_global_regs_t *ptr_dev_global_regs =
	    core_if->dev_if->dev_global_regs;
	const uint32_t TOKEN_Q_DEPTH = core_if->hwcfg2.b.dev_token_q_depth;
	/* Number of Token Queue Registers */
	const uint32_t DTKNQ_REG_CNT = (TOKEN_Q_DEPTH + 7) / 8;
	dtknq1_data_t dev_tkn_qr1;
uint32_t in_tkn_epnums[4] = {0};
	uint32_t seqnum[MAX_EPS_CHANNELS];
	uint32_t intkn_seq[TOKEN_Q_DEPTH];
	grstctl_t resetctl;
	uint32_t temp;
	uint32_t ndx = 0;
	uint32_t uint_start = 0;
	int32_t end = 0;
	int32_t sort_done = 0;
	uint32_t i = 0;
	int32_t flag = 0;
	volatile uint32_t *ptr_addr = &ptr_dev_global_regs->dtknqr1;
	resetctl.d32 = 0 ;

	DBG_USB_Print(DBG_PCD, "dev_token_q_depth=%d\n", TOKEN_Q_DEPTH);

	/* Read the DTKNQ Registers */
	for (i = 0; i < DTKNQ_REG_CNT; i++) {
		in_tkn_epnums[i] = REG_RD(ptr_addr);
		DBG_USB_Print(DBG_PCDV, "DTKNQR%d=0x%08x\n", i + 1,
			    in_tkn_epnums[i]);
		if (ptr_addr == &ptr_dev_global_regs->dvbusdis) {
			ptr_addr = &ptr_dev_global_regs->dtknqr3_dthrctl;
		} else {
			++ptr_addr;
		}

	}

	/* Copy the DTKNQR1 data to the bit field. */
	dev_tkn_qr1.d32 = in_tkn_epnums[0];
	if (dev_tkn_qr1.b.wrap_bit != 0) {
		ndx = dev_tkn_qr1.b.intknwptr;
		end = (int32_t)(ndx - 1);
		if (end < 0) {
			end = (int32_t)(TOKEN_Q_DEPTH - 1);
		}
	} else {
		ndx = 0;
		end = dev_tkn_qr1.b.intknwptr - 1;
		if (end < 0) {
			end = 0;
		}
	}
	uint_start = ndx;

	/* Fill seqnum[] by initial values: EP number + 31 */
	for (i = 0; i <= core_if->dev_if->num_in_eps; i++) {
		seqnum[i] = i + 31;
	}

	/* Fill intkn_seq[] from in_tkn_epnums[0] */
	for (i = 0; i < 6; i++) {
		intkn_seq[i] = (in_tkn_epnums[0] >> ((7 - i) * 4)) & 0xf;
	}

	if (TOKEN_Q_DEPTH > 6) {
		/* Fill intkn_seq[] from in_tkn_epnums[1] */
		for (i = 6; i < 14; i++) {
			intkn_seq[i] =
			    (in_tkn_epnums[1] >> ((7 - (i - 6)) * 4)) & 0xf;
		}
	}

	if (TOKEN_Q_DEPTH > 14) {
		/* Fill intkn_seq[] from in_tkn_epnums[1] */
		for (i = 14; i < 22; i++) {
			intkn_seq[i] =
			    (in_tkn_epnums[2] >> ((7 - (i - 14)) * 4)) & 0xf;
		}
	}

	if (TOKEN_Q_DEPTH > 22) {
		/* Fill intkn_seq[] from in_tkn_epnums[1] */
		for (i = 22; i < 30; i++) {
			intkn_seq[i] =
			    (in_tkn_epnums[3] >> ((7 - (i - 22)) * 4)) & 0xf;
		}
	}

	DBG_USB_Print(DBG_PCDV, "%s start=%d end=%d intkn_seq[]:\n", __func__,
		    uint_start, end);
	for (i = 0; i < TOKEN_Q_DEPTH; i++)
		DBG_USB_Print(DBG_PCDV, "%d\n", intkn_seq[i]);

	/* Update seqnum based on intkn_seq[] */
	i = 0;
	do {
		seqnum[intkn_seq[ndx]] = i;
		ndx++;
		i++;
		if (ndx == TOKEN_Q_DEPTH) {
			ndx = 0;
		}
	} while (i < TOKEN_Q_DEPTH);

	/* Mark non active EP's in seqnum[] by 0xff */
	for (i = 0; i <= core_if->dev_if->num_in_eps; i++) {
		if (core_if->nextep_seq[i] == 0xff) {
			seqnum[i] = 0xff;
		}
	}

	/* Sort seqnum[] */
	sort_done = 0;
	while (!sort_done) {
		sort_done = 1;
		for (i = 0; i < core_if->dev_if->num_in_eps; i++) {
			if (seqnum[i] > seqnum[i + 1]) {
				temp = seqnum[i];
				seqnum[i] = seqnum[i + 1];
				seqnum[i + 1] = temp;
				sort_done = 0;
			}
		}
	}

	ndx = uint_start + seqnum[0];
	if (ndx >= TOKEN_Q_DEPTH) {
		ndx = ndx % TOKEN_Q_DEPTH;
	}
	core_if->first_in_nextep_seq = (uint8_t)intkn_seq[ndx];

	/* Update seqnum[] by EP numbers  */
	for (i = 0; i <= core_if->dev_if->num_in_eps; i++) {
		ndx = uint_start + i;
		if (seqnum[i] < 31) {
			ndx = uint_start + seqnum[i];
			if (ndx >= TOKEN_Q_DEPTH) {
				ndx = ndx % TOKEN_Q_DEPTH;
			}
			seqnum[i] = intkn_seq[ndx];
		} else {
			if (seqnum[i] < 0xff) {
				seqnum[i] = seqnum[i] - 31;
			} else {
				break;
			}
		}
	}

	/* Update nextep_seq[] based on seqnum[] */
	for (i = 0; i < core_if->dev_if->num_in_eps; i++) {
		if (seqnum[i] != 0xff) {
			if (seqnum[i + 1] != 0xff) {
				core_if->nextep_seq[seqnum[i]] = (uint8_t)seqnum[i + 1];
			} else {
				core_if->nextep_seq[seqnum[i]] = core_if->first_in_nextep_seq;
				flag = 1;
			}
		} else {
			flag = 1;
		}
		if (flag == 1){
			break;
		}
	}

	DBG_USB_Print(DBG_PCDV, "%s first_in_nextep_seq= %2d; nextep_seq[]:\n",
		    __func__, core_if->first_in_nextep_seq);
	for (i = 0; i <= core_if->dev_if->num_in_eps; i++) {
		DBG_USB_Print(DBG_PCDV, "%2d\n", core_if->nextep_seq[i]);
	}

	/* Flush the Learning Queue */
	resetctl.d32 = REG_RD(&core_if->core_global_regs->grstctl);
	resetctl.b.intknqflsh = 1;
	REG_WR(&core_if->core_global_regs->grstctl, resetctl.d32);
	
	(void)end;
}

#endif
/**
 * handle the IN EP disable interrupt.
 */
static inline void handle_in_ep_disable_intr(dwc_otg_pcd_t * pcd,
					     const uint32_t epnum)
{
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
	dwc_otg_dev_if_t *ptr_dev_if = ptr_core_if->dev_if;
	deptsiz_data_t dev_ept_siz;
	dctl_data_t dev_ctl;
	dwc_otg_pcd_ep_t *ptr_ep;
	dwc_ep_t *ptr_dwc_ep;
	gintmsk_data_t gbl_int_msk_data;
	depctl_data_t depctl;
	uint32_t i;
	uint32_t j;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
    uint32_t remain_to_transfer = 0;
    uint32_t dev_ep_dma;
	uint32_t xfer_size;
#endif
	dev_ept_siz.d32 = 0 ;
	dev_ctl.d32 = 0 ;

	ptr_ep = get_in_ep(pcd, epnum);
	ptr_dwc_ep = &ptr_ep->dwc_ep;

	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	DBG_USB_Print(DBG_PCD, "diepctl%d=%0x\n", epnum,
		    REG_RD(&ptr_dev_if->in_ep_regs[epnum]->diepctl));
	dev_ept_siz.d32 = REG_RD(&ptr_dev_if->in_ep_regs[epnum]->dieptsiz);
	depctl.d32 = REG_RD(&ptr_dev_if->in_ep_regs[epnum]->diepctl);

	DBG_USB_Print(DBG_USB, "pktcnt=%d size=%d\n",
		    dev_ept_siz.b.pktcnt, dev_ept_siz.b.xfersize);
	j = depctl.b.eptype & 1;
	if ((ptr_core_if->start_predict == 0) || (j)) {
		if (ptr_ep->stopped != 0) {
			if (ptr_core_if->en_multiple_tx_fifo != 0) {
				/* Flush the Tx FIFO */
				dwc_otg_flush_tx_fifo(ptr_core_if, ptr_dwc_ep->tx_fifo_num);
			}
			/* Clear the Global IN NP NAK */
			dev_ctl.d32 = 0;
			dev_ctl.b.cgnpinnak = 1;
			DWC_MODIFY_REG32(&ptr_dev_if->dev_global_regs->dctl, dev_ctl.d32, dev_ctl.d32); 
			/* Restart the transaction */
			if ((dev_ept_siz.b.pktcnt != 0) || (dev_ept_siz.b.xfersize != 0)) {
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
				restart_transfer(pcd, epnum);
#endif
			}
		} else {
			/* Restart the transaction */
			if ((dev_ept_siz.b.pktcnt != 0) || (dev_ept_siz.b.xfersize != 0)) {
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
				restart_transfer(pcd, epnum);
#endif
			}
			DBG_USB_Print(DBG_USB, "STOPPED!!!\n");
		}
	}else if (ptr_core_if->start_predict > 2) {	/* NP IN EP */
		ptr_core_if->start_predict--;
	}else {
		ptr_core_if->start_predict--;

		if (ptr_core_if->start_predict == 1) {	/* All NP IN Ep's disabled now */

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			predict_nextep_seq(ptr_core_if);
#endif

			/* Update all active IN EP's NextEP field based of nextep_seq[] */
			for (i = 0; i <= ptr_core_if->dev_if->num_in_eps; i++) {
				depctl.d32 =
					REG_RD(&ptr_dev_if->in_ep_regs[i]->diepctl);
				if (ptr_core_if->nextep_seq[i] != 0xff) {	/* Active NP IN EP */
					depctl.b.nextep = ptr_core_if->nextep_seq[i];
					REG_WR(&ptr_dev_if->in_ep_regs[i]->diepctl, depctl.d32);
				}
			}
			/* Flush Shared NP TxFIFO */
			dwc_otg_flush_tx_fifo(ptr_core_if, 0);
			/* Rewind buffers */
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			if (!ptr_core_if->dma_desc_enable) {		
				i = ptr_core_if->first_in_nextep_seq;
				do {
					ptr_ep = get_in_ep(pcd, i);
					dev_ept_siz.d32 = REG_RD(&ptr_dev_if->in_ep_regs[i]->dieptsiz);
					xfer_size = ptr_ep->dwc_ep.total_len - ptr_ep->dwc_ep.xfer_count;
					if (xfer_size > ptr_ep->dwc_ep.maxxfer)  {
						xfer_size = ptr_ep->dwc_ep.maxxfer;
					}
					depctl.d32 = REG_RD(&ptr_dev_if->in_ep_regs[i]->diepctl);
					if (dev_ept_siz.b.pktcnt != 0) {
						if (xfer_size == 0) {
							remain_to_transfer = 0;
						} else {
							if ((xfer_size % ptr_ep->dwc_ep.maxpacket) == 0) {
								remain_to_transfer = 
									dev_ept_siz.b.pktcnt * ptr_ep->dwc_ep.maxpacket;
							} else {
								remain_to_transfer = ((dev_ept_siz.b.pktcnt -1) * ptr_ep->dwc_ep.maxpacket) 
									+ (xfer_size % ptr_ep->dwc_ep.maxpacket);
							}
						}
						dev_ep_dma = REG_RD(&ptr_dev_if->in_ep_regs[i]->diepdma);
						dev_ept_siz.b.xfersize = remain_to_transfer;
						REG_WR(&ptr_dev_if->in_ep_regs[i]->dieptsiz, dev_ept_siz.d32);
						dev_ep_dma = ptr_ep->dwc_ep.dma_addr + (xfer_size - remain_to_transfer);
						REG_WR(&ptr_dev_if->in_ep_regs[i]->diepdma, dev_ep_dma);
					}
					i = ptr_core_if->nextep_seq[i];
				} while (i != ptr_core_if->first_in_nextep_seq);
			} else { /* dma_desc_enable */
					DBG_USB_Print(DBG_USB,"%s Learning Queue not supported in DDMA\n", (char_t *)__func__);
			}
#endif
			/* Restart transfers in predicted sequences */
			i = ptr_core_if->first_in_nextep_seq;
			do {
				dev_ept_siz.d32 = REG_RD(&ptr_dev_if->in_ep_regs[i]->dieptsiz);
				depctl.d32 = REG_RD(&ptr_dev_if->in_ep_regs[i]->diepctl);
				if (dev_ept_siz.b.pktcnt != 0) {
					depctl.d32 = REG_RD(&ptr_dev_if->in_ep_regs[i]->diepctl);
					depctl.b.epena = 1;
					depctl.b.cnak = 1;
					REG_WR(&ptr_dev_if->in_ep_regs[i]->diepctl, depctl.d32);
				}
				i = ptr_core_if->nextep_seq[i];
			} while (i != ptr_core_if->first_in_nextep_seq);

			/* Clear the global non-periodic IN NAK handshake */
			dev_ctl.d32 = 0;
			dev_ctl.b.cgnpinnak = 1;
			DWC_MODIFY_REG32(&ptr_dev_if->dev_global_regs->dctl, dev_ctl.d32, dev_ctl.d32); 
				
			/* Unmask EP Mismatch interrupt */
			gbl_int_msk_data.d32 = 0;
			gbl_int_msk_data.b.epmismatch = 1;
			DWC_MODIFY_REG32(&ptr_core_if->core_global_regs->gintmsk, 0, gbl_int_msk_data.d32);
			
			ptr_core_if->start_predict = 0;
		} 
	}
	return;
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
/**
 * Handler for the IN EP timeout handshake interrupt.
 */
static inline void handle_in_ep_timeout_intr(dwc_otg_pcd_t * pcd,
					     const uint32_t epnum)
{
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
	dwc_otg_dev_if_t *ptr_dev_if = ptr_core_if->dev_if;

	deptsiz_data_t dev_ept_siz;
	uint32_t uint_num = 0;
	dctl_data_t dev_ctl;
	dwc_otg_pcd_ep_t *ptr_ep;
	gintmsk_data_t intr_mask;
	dev_ept_siz.d32 = 0 ;
	dev_ctl.d32 = 0 ;

	intr_mask.d32 = 0 ;

	ptr_ep = get_in_ep(pcd, epnum);

	/* Disable the NP Tx Fifo Empty Interrrupt */
	if (!ptr_core_if->dma_enable) {
		intr_mask.b.nptxfempty = 1;
		DWC_MODIFY_REG32(&ptr_core_if->core_global_regs->gintmsk,
				 intr_mask.d32, 0);
	}
	/** @todo NGS Check EP type.
	 * Implement for Periodic EPs */
	/*
	 * Non-periodic EP
	 */
	/* Enable the Global IN NAK Effective Interrupt */
	intr_mask.b.ginnakeff = 1;
	DWC_MODIFY_REG32(&ptr_core_if->core_global_regs->gintmsk, 0, intr_mask.d32);

	/* Set Global IN NAK */
	dev_ctl.b.sgnpinnak = 1;
	DWC_MODIFY_REG32(&ptr_dev_if->dev_global_regs->dctl, dev_ctl.d32, dev_ctl.d32);

	ptr_ep->stopped = 1;

	dev_ept_siz.d32 = REG_RD(&ptr_dev_if->in_ep_regs[uint_num]->dieptsiz);
	DBG_USB_Print(DBG_USB, "pktcnt=%d size=%d\n",
		    dev_ept_siz.b.pktcnt, dev_ept_siz.b.xfersize);

#ifdef DISABLE_PERIODIC_EP
	/*
	 * Set the NAK bit for this EP to
	 * start the disable process.
	 */
	diepctl.d32 = 0;
	diepctl.b.snak = 1;
	DWC_MODIFY_REG32(&dev_if->in_ep_regs[num]->diepctl, diepctl.d32,
			 diepctl.d32);
	ep->disabling = 1;
	ep->stopped = 1;
#endif
	(void)dev_ept_siz.d32;
	return;
}

/**
 * Handler for the OUT EP Babble interrupt.
 */
static inline int32_t handle_out_ep_babble_intr(dwc_otg_pcd_t const * pcd,
						const uint32_t epnum)
{
	/** @todo implement ISR */
	dwc_otg_core_if_t *ptr_core_if;
	doepmsk_data_t intr_mask;
	intr_mask.d32 = 0 ;

	DBG_USB_Print(DBG_USB,"INTERRUPT Handler not implemented for %s\n",
		   "OUT EP Babble");
	ptr_core_if = GET_CORE_IF(pcd);
	intr_mask.b.babble = 1;

	if (ptr_core_if->multiproc_int_enable  != 0) {
		DWC_MODIFY_REG32(&ptr_core_if->dev_if->dev_global_regs->
				 doepeachintmsk[epnum], intr_mask.d32, 0);
	} else {
		DWC_MODIFY_REG32(&ptr_core_if->dev_if->dev_global_regs->doepmsk,
				 intr_mask.d32, 0);
	}
	return 1;
}

/**
 * Handler for the OUT EP NAK interrupt.
 */
static inline int32_t handle_out_ep_nak_intr(dwc_otg_pcd_t const * pcd,
					     const uint32_t epnum)
{
	/** @todo implement ISR */
	dwc_otg_core_if_t *ptr_core_if;
	doepmsk_data_t intr_mask;
	intr_mask.d32 = 0 ;

	DBG_USB_Print(DBG_USB, "INTERRUPT Handler not implemented for %s\n", "OUT EP NAK");
	ptr_core_if = GET_CORE_IF(pcd);
	intr_mask.b.nak = 1;

	if (ptr_core_if->multiproc_int_enable != 0) {
		DWC_MODIFY_REG32(&ptr_core_if->dev_if->dev_global_regs->
				 doepeachintmsk[epnum], intr_mask.d32, 0);
	} else {
		DWC_MODIFY_REG32(&ptr_core_if->dev_if->dev_global_regs->doepmsk,
				 intr_mask.d32, 0);
	}
	return 1;
}

/**
 * Handler for the OUT EP NYET interrupt.
 */
static inline int32_t handle_out_ep_nyet_intr(dwc_otg_pcd_t const * pcd,
					      const uint32_t epnum)
{
	/** @todo implement ISR */
	dwc_otg_core_if_t *ptr_core_if;
	doepmsk_data_t intr_mask;
	intr_mask.d32 = 0 ;

	DBG_USB_Print(DBG_USB,"INTERRUPT Handler not implemented for %s\n", "OUT EP NYET");
	ptr_core_if = GET_CORE_IF(pcd);
	intr_mask.b.nyet = 1;

	if (ptr_core_if->multiproc_int_enable != 0) {
		DWC_MODIFY_REG32(&ptr_core_if->dev_if->dev_global_regs->
				 doepeachintmsk[epnum], intr_mask.d32, 0);
	} else {
		DWC_MODIFY_REG32(&ptr_core_if->dev_if->dev_global_regs->doepmsk,
				 intr_mask.d32, 0);
	}
	return 1;
}

#endif
/**
 * This interrupt indicates that an IN EP has a pending Interrupt.
 * The sequence for handling the IN EP interrupt is shown below:
 * -#	Read the Device All Endpoint Interrupt register
 * -#	Repeat the following for each IN EP interrupt bit set (from
 *		LSB to MSB).
 * -#	Read the Device Endpoint Interrupt (DIEPINTn) register
 * -#	If "Transfer Complete" call the request complete function
 * -#	If "Endpoint Disabled" complete the EP disable procedure.
 * -#	If "AHB Error Interrupt" log error
 * -#	If "Time-out Handshake" log error
 * -#	If "IN Token Received when TxFIFO Empty" write packet to Tx
 *		FIFO.
 * -#	If "IN Token EP Mismatch" (disable, this is handled by EP
 *		Mismatch Interrupt)
 */
static int32_t dwc_otg_pcd_handle_in_ep_intr(dwc_otg_pcd_t * pcd)
{
#define CLEAR_IN_EP_INTR(__core_if,__epnum,__intr) \
do { \
		diepint_data_t str_dev_ep_int;\
		str_dev_ep_int.d32=0; \
		str_dev_ep_int.b.__intr = 1; \
		REG_WR(&__core_if->dev_if->in_ep_regs[__epnum]->diepint, \
		str_dev_ep_int.d32); \
} while (0)

	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
	dwc_otg_dev_if_t *ptr_dev_if = ptr_core_if->dev_if;
	diepint_data_t dev_ep_int;
	depctl_data_t depctl;
	uint32_t ep_intr;
	dwc_otg_pcd_ep_t *ptr_ep;
	dwc_ep_t *ptr_dwc_ep;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	gintmsk_data_t intr_mask;
	intr_mask.d32 = 0 ;
#endif
	uint32_t uint_epnum = 0;
	depctl.d32 = 0 ;

	DBG_USB_Print(DBG_PCDV, "%s(%p)\n", __func__, pcd);
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	/* Read in the device interrupt bits */
	ep_intr = dwc_otg_read_dev_all_in_ep_intr(ptr_core_if);

	/* Service the Device IN interrupts for each endpoint */
	while (ep_intr != 0) {
		if ((ep_intr & 0x1) != 0) {
			/* Get EP pointer */
			ptr_ep = get_in_ep(pcd, uint_epnum);
			ptr_dwc_ep = &ptr_ep->dwc_ep;

			depctl.d32 =
			    REG_RD(&ptr_dev_if->in_ep_regs[uint_epnum]->diepctl);

			DBG_USB_Print(DBG_PCDV,
				    "IN EP INTERRUPT - %d\ndiepctl - %8x\n",
				    uint_epnum, depctl.d32);

			DBG_USB_Print(DBG_PCD,
				    "EP%d-%s: type=%d, mps=%d\n",
				    ptr_dwc_ep->num, (ptr_dwc_ep->is_in ? "IN" : "OUT"),
				    ptr_dwc_ep->type, ptr_dwc_ep->maxpacket);
					
			DBG_Test_Print(DBG_TEST, "EP%d-%s \n", ptr_dwc_ep->num, (ptr_dwc_ep->is_in ? "IN" : "OUT"));					
			if(ptr_dwc_ep->num != 0)
				DBG_Test_Print(DBG_TEST_IN_EP, "EP%d-%s \n", ptr_dwc_ep->num, (ptr_dwc_ep->is_in ? "IN" : "OUT"));					

			dev_ep_int.d32 =
			    dwc_otg_read_dev_in_ep_intr(ptr_core_if, ptr_dwc_ep);

			DBG_USB_Print(DBG_PCDV,
				    "EP %d Interrupt Register - 0x%x\n", uint_epnum,
				    dev_ep_int.d32);
			/* Transfer complete */
			if (dev_ep_int.b.xfercompl != 0) {
				/* Disable the NP Tx FIFO Empty
				 * Interrupt */
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
				if (ptr_core_if->en_multiple_tx_fifo == 0) {
					intr_mask.b.nptxfempty = 1;
					DWC_MODIFY_REG32
					    (&ptr_core_if->core_global_regs->gintmsk,
					     intr_mask.d32, 0);
				} else {
#endif
					/* Disable the Tx FIFO Empty Interrupt for this EP */
					uint32_t fifoemptymsk =
					    0x1U << ptr_dwc_ep->num;
					DWC_MODIFY_REG32(&ptr_core_if->
							 dev_if->dev_global_regs->dtknqr4_fifoemptymsk,
							 fifoemptymsk, 0);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
				}
#endif
				/* Clear the bit in DIEPINTn for this interrupt */
				CLEAR_IN_EP_INTR(ptr_core_if, uint_epnum, xfercompl);

				/* Complete the transfer */
				if (uint_epnum == 0) {
					handle_ep0(pcd);
				}
				else {
						complete_ep(ptr_ep);
						if(dev_ep_int.b.nak != 0){
							CLEAR_IN_EP_INTR(ptr_core_if, uint_epnum, nak);
						}
					}
				}
                /* Endpoint disable      */
			if (dev_ep_int.b.epdisabled != 0) {
				DBG_USB_Print(DBG_USB, "EP%d IN disabled\n",
					    uint_epnum);
				handle_in_ep_disable_intr(pcd, uint_epnum);

				/* Clear the bit in DIEPINTn for this interrupt */
				CLEAR_IN_EP_INTR(ptr_core_if, uint_epnum, epdisabled);
			}
			/* AHB Error */
			if (dev_ep_int.b.ahberr != 0) {
				DBG_Error_Print("EP%d IN AHB Error\n", uint_epnum);
				/* Clear the bit in DIEPINTn for this interrupt */
				CLEAR_IN_EP_INTR(ptr_core_if, uint_epnum, ahberr);
			}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			/* TimeOUT Handshake (non-ISOC IN EPs) */
			if (dev_ep_int.b.timeout != 0) {
				DBG_Error_Print("EP%d IN Time-out\n", uint_epnum);
				handle_in_ep_timeout_intr(pcd, uint_epnum);

				CLEAR_IN_EP_INTR(ptr_core_if, uint_epnum, timeout);
			}
#endif
			/** IN Token received with TxF Empty */
			if (dev_ep_int.b.intktxfemp != 0) {
				DBG_USB_Print(DBG_USB,
					    "EP%d IN TKN TxFifo Empty\n",
					    uint_epnum);
				if ((!ptr_ep->stopped) && (uint_epnum != 0)) {

					diepmsk_data_t dev_ep_msk;
					dev_ep_msk.d32 = 0 ;
					dev_ep_msk.b.intktxfemp = 1;

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
					if (ptr_core_if->multiproc_int_enable != 0) {
						DWC_MODIFY_REG32
						    (&ptr_dev_if->dev_global_regs->diepeachintmsk
						     [uint_epnum], dev_ep_msk.d32, 0);
					} else {
#endif
						DWC_MODIFY_REG32
						    (&ptr_dev_if->dev_global_regs->diepmsk,
						     dev_ep_msk.d32, 0);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
					}
#endif
				} else if (((ptr_core_if->dma_desc_enable)
					   && (uint_epnum == 0))
					   && ((pcd->ep0state) ==
					   (EP0_OUT_STATUS_PHASE))) {
					/* EP0 IN set STALL */
					depctl.d32 =
					    REG_RD(&ptr_dev_if->in_ep_regs
							   [uint_epnum]->diepctl);

					/* set the disable and stall bits */
					if (depctl.b.epena != 0) {
						depctl.b.epdis = 1;
					}
					depctl.b.stall = 1;
					REG_WR(&ptr_dev_if->in_ep_regs
							[uint_epnum]->diepctl,
							depctl.d32);
				}
				CLEAR_IN_EP_INTR(ptr_core_if, uint_epnum, intktxfemp);
			}
			/** IN Token Received with EP mismatch */
			if (dev_ep_int.b.intknepmis != 0) {
				DBG_USB_Print(DBG_USB,
					    "EP%d IN TKN EP Mismatch\n", uint_epnum);
				CLEAR_IN_EP_INTR(ptr_core_if, uint_epnum, intknepmis);
			}
			/** IN Endpoint NAK Effective */
			if (dev_ep_int.b.inepnakeff != 0) {
				DBG_USB_Print(DBG_USB,
					    "EP%d IN EP NAK Effective\n",
					    uint_epnum);
				/* Periodic EP */
				if (ptr_ep->disabling != 0) {
					depctl.d32 = 0;
					depctl.b.snak = 1;
					depctl.b.epdis = 1;
					DWC_MODIFY_REG32(&ptr_dev_if->in_ep_regs
							 [uint_epnum]->diepctl,
							 depctl.d32,
							 depctl.d32);
				}
				CLEAR_IN_EP_INTR(ptr_core_if, uint_epnum, inepnakeff);

			}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			/** IN EP Tx FIFO Empty Intr */
			if (dev_ep_int.b.emptyintr != 0) {
				DBG_USB_Print(DBG_USB,
					    "EP%d Tx FIFO Empty Intr \n",
					    uint_epnum);
				write_empty_tx_fifo(pcd, uint_epnum);

				CLEAR_IN_EP_INTR(ptr_core_if, uint_epnum, emptyintr);

			}

			/** IN EP BNA Intr */
			if (dev_ep_int.b.bna != 0) {
				CLEAR_IN_EP_INTR(ptr_core_if, uint_epnum, bna);
				if (ptr_core_if->dma_desc_enable != 0) {
						dwc_otg_pcd_handle_noniso_bna(ptr_ep);
					}
			}
#endif
			/* NAK Interrupt */
			if (dev_ep_int.b.nak != 0) {
				DBG_USB_Print(DBG_USB, "EP%d IN NAK Interrupt\n",
					    uint_epnum);

				CLEAR_IN_EP_INTR(ptr_core_if, uint_epnum, nak);
			}
		}
		uint_epnum++;
		ep_intr >>= 1;
	}

	return 1;
#undef CLEAR_IN_EP_INTR
}
#if 1
void delay(volatile uint32_t time)
{
	while(time != 0){
		time--;
	}
}	
#endif
/**
 * This interrupt indicates that an OUT EP has a pending Interrupt.
 * The sequence for handling the OUT EP interrupt is shown below:
 * -#	Read the Device All Endpoint Interrupt register
 * -#	Repeat the following for each OUT EP interrupt bit set (from
 *		LSB to MSB).
 * -#	Read the Device Endpoint Interrupt (DOEPINTn) register
 * -#	If "Transfer Complete" call the request complete function
 * -#	If "Endpoint Disabled" complete the EP disable procedure.
 * -#	If "AHB Error Interrupt" log error
 * -#	If "Setup Phase Done" process Setup Packet (See Standard USB
 *		Command Processing)
 */
static int32_t dwc_otg_pcd_handle_out_ep_intr(dwc_otg_pcd_t * pcd)
{
#define CLEAR_OUT_EP_INTR(__core_if,__epnum,__intr) \
do { \
		doepint_data_t str_dev_ep_int; \
		str_dev_ep_int.d32=0; \
		str_dev_ep_int.b.__intr = 1; \
		REG_WR(&__core_if->dev_if->out_ep_regs[__epnum]->doepint, \
		str_dev_ep_int.d32); \
} while (0)

	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
	uint32_t ep_intr;
	doepint_data_t dev_oep_int;
	uint32_t uint_epnum = 0;
	dwc_otg_pcd_ep_t *ptr_ep;
	dwc_ep_t *ptr_dwc_ep;
	static int32_t tx_temp;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
    gintmsk_data_t gbl_int_msk;
    dctl_data_t dev_ctl;
    dev_ctl.d32 = 0 ;
	gbl_int_msk.d32 = 0 ;
#endif
	dev_oep_int.d32 = 0 ;

	/* Read in the device interrupt bits */
 	ep_intr = dwc_otg_read_dev_all_out_ep_intr(ptr_core_if);   /* READ DAINT  */
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	while (ep_intr != 0) {
		if ((ep_intr & 0x1) != 0) {
			/* Get EP pointer */
			ptr_ep = get_out_ep(pcd, uint_epnum);
			ptr_dwc_ep = &ptr_ep->dwc_ep;

#ifdef VERBOSE
			DBG_USB_Print(DBG_PCDV,
				    "EP%d-%s: type=%d, mps=%d\n",
				    dwc_ep->num, (dwc_ep->is_in ? "IN" : "OUT"),
				    dwc_ep->type, dwc_ep->maxpacket);
#endif
			DBG_Test_Print(DBG_TEST, "EP%d-%s \n", ptr_dwc_ep->num, (ptr_dwc_ep->is_in ? "IN" : "OUT"));
			dev_oep_int.d32 =
			    dwc_otg_read_dev_out_ep_intr(ptr_core_if, ptr_dwc_ep);

			/* Transfer complete */
			if (dev_oep_int.b.xfercompl != 0) {
				if(uint_epnum == 2) { tx_temp++;
				}
				if (uint_epnum == 0) {
					/* Clear the bit in DOEPINTn for this interrupt */
					CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, xfercompl); 
					if (ptr_core_if->snpsid >= OTG_CORE_REV_3_00a) {

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
						if ((ptr_core_if->snpsid >= OTG_CORE_REV_3_00a)
							&& (ptr_core_if->dma_enable == 0)) {
							doepint_data_t str_dev_oep_int;
							str_dev_oep_int.d32 = REG_RD(&ptr_core_if->dev_if->
														out_ep_regs[0]->doepint);
							if ((pcd->ep0state == EP0_IDLE) && (str_dev_oep_int.b.sr)) {
								CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, sr);
								if (str_dev_oep_int.b.stsphsercvd != 0) {
									CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, stsphsercvd);
								}
								goto exit_xfercompl;
							}
						}
#endif
						/* In case of DDMA  look at SR bit to go to the Data Stage */
						if (ptr_core_if->dma_desc_enable != 0) {
							dev_dma_desc_sts_t dev_status; 
							dev_status.d32 = 0;
							if (pcd->ep0state == EP0_IDLE) {
								dev_status.d32 = ptr_core_if->dev_if->setup_desc_addr[ptr_core_if->
											dev_if->setup_desc_index]->status.d32;
								if(pcd->data_terminated != 0) {
									 pcd->data_terminated = 0;
									 dev_status.d32 = ptr_core_if->dev_if->out_desc_addr->status.d32;
									 dwc_memcpy(&pcd->setup_pkt->req, pcd->backup_buf, 8);
								}
								if (dev_status.b.sr != 0) {
									if (dev_oep_int.b.setup != 0) {
										/* Already started data stage, clear setup */
										dev_oep_int.b.setup = 0;
										CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, setup);
										
										handle_ep0(pcd);
										/* Prepare for more setup packets */
										if ((pcd->ep0state == EP0_IN_STATUS_PHASE) || 
											(pcd->ep0state == EP0_IN_DATA_PHASE)) {
											ep0_out_start(ptr_core_if, pcd);
										}
										
										goto exit_xfercompl;
									} else {
										/* Prepare for more setup packets */
										ep0_out_start(ptr_core_if, pcd);
									}
								}
							} else {
								dwc_otg_pcd_request_t *ptr_req;
								dev_dma_desc_sts_t str_dev_status;
								diepint_data_t diepint0;
								str_dev_status.d32 = 0;
								diepint0.d32 = REG_RD(&ptr_core_if->dev_if->
															in_ep_regs[0]->diepint);

								if ((pcd->ep0state == EP0_STALL) || (pcd->ep0state == EP0_DISCONNECT)) {
									DBG_Error_Print("EP0 is stalled/disconnected\n");
								}

								/* Clear IN xfercompl if set */
								if ((diepint0.b.xfercompl) && ((pcd->ep0state == EP0_IN_STATUS_PHASE)
									|| (pcd->ep0state == EP0_IN_DATA_PHASE))) {
									REG_WR(&ptr_core_if->dev_if->
										in_ep_regs[0]->diepint, diepint0.d32);
								}

								str_dev_status.d32 = ptr_core_if->dev_if->setup_desc_addr[ptr_core_if->
									dev_if->setup_desc_index]->status.d32;

								if ((pcd->ep0state == EP0_OUT_STATUS_PHASE) || 
									((ptr_ep->dwc_ep.xfer_count != ptr_ep->dwc_ep.total_len)
									&& (pcd->ep0state == EP0_OUT_DATA_PHASE))) {
									str_dev_status.d32 = ptr_core_if->dev_if->out_desc_addr->status.d32;
									}
								if (str_dev_status.b.sr != 0) {
									if (DWC_CIRCLEQ_EMPTY(&ptr_ep->queue)) {
										DBG_USB_Print(DBG_PCDV, "Request queue empty!!\n");
									} else {
										DBG_USB_Print(DBG_PCDV, "complete req!!\n");
										ptr_req = DWC_CIRCLEQ_FIRST(&ptr_ep->queue);
										if ((ptr_ep->dwc_ep.xfer_count != ptr_ep->dwc_ep.total_len) &&
											(pcd->ep0state == EP0_OUT_DATA_PHASE)) {
												/* Read arrived setup packet from req->buf */
												dwc_memcpy(&pcd->setup_pkt->req, 
													(&ptr_req->buf) + (ptr_ep->dwc_ep.xfer_count), 8);
										}
										ptr_req->actual = ptr_ep->dwc_ep.xfer_count;
										dwc_otg_request_done(ptr_ep, ptr_req, -ECONNRESET);
										ptr_ep->dwc_ep.start_xfer_buff = 0;
										ptr_ep->dwc_ep.xfer_buff = 0;
										ptr_ep->dwc_ep.xfer_len = 0;
									}
									pcd->ep0state = EP0_IDLE;
									if (dev_oep_int.b.setup != 0) {
										DBG_USB_Print(DBG_PCDV, "EP0_IDLE SR=1 setup=1\n");
										/* Data stage started, clear setup */
										dev_oep_int.b.setup = 0;
										CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, setup);
										handle_ep0(pcd);
										/* Prepare for setup packets if ep0in was enabled*/
										if (pcd->ep0state == EP0_IN_STATUS_PHASE) {
											ep0_out_start(ptr_core_if, pcd);
										}

										goto exit_xfercompl;
									} else {
										/* Prepare for more setup packets */
										DBG_USB_Print(DBG_PCDV, 
											"EP0_IDLE SR=1 setup=0 new setup comes 2\n");
										ep0_out_start(ptr_core_if, pcd);
									}
								}
							}
 						}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
						if (((ptr_core_if->snpsid >= OTG_CORE_REV_3_00a) && (ptr_core_if->dma_enable))
							&& (ptr_core_if->dma_desc_enable == 0)) {
							doepint_data_t doepint_temp;
							deptsiz0_data_t doeptsize0;
							doepint_temp.d32 = 0;
							doeptsize0.d32 = 0 ;
							doepint_temp.d32 = REG_RD(&ptr_core_if->dev_if->
															out_ep_regs[ptr_ep->dwc_ep.num]->doepint);
							doeptsize0.d32 = REG_RD(&ptr_core_if->dev_if->
															out_ep_regs[ptr_ep->dwc_ep.num]->doeptsiz);
							if ((((ptr_ep->dwc_ep.xfer_count == ptr_ep->dwc_ep.total_len) || (doeptsize0.b.xfersize == 64)) &&
								(pcd->ep0state == EP0_OUT_DATA_PHASE) && (dev_oep_int.b.stsphsercvd)) ||
								((doeptsize0.b.xfersize == 24) && (pcd->ep0state == EP0_IN_STATUS_PHASE))) {
									CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, xfercompl);
									DBG_USB_Print(DBG_PCDV, "WA for xfercompl along with stsphs \n");
									dev_oep_int.b.xfercompl = 0;
									ep0_out_start(ptr_core_if, pcd);
									goto exit_xfercompl;
							}

							if (pcd->ep0state == EP0_IDLE) {
								if (doepint_temp.b.sr != 0) {
									CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, sr);	
								}
									/* Delay is needed for core to update setup 
									 * packet count from 3 to 2 after receiving 
									 * setup packet*/
									dev_oep_int.d32 = REG_RD(&ptr_core_if->dev_if->
																	out_ep_regs[0]->doepint);
									if (doeptsize0.b.supcnt == 3) {
										DBG_USB_Print(DBG_USB, "Rolling over!!!!!!!\n");
										ptr_ep->dwc_ep.stp_rollover = 1;
									}
									if (dev_oep_int.b.setup != 0) {
retry:
										/* Already started data stage, clear setup */
										dev_oep_int.b.setup = 0;
										CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, setup);
										dev_oep_int.b.setup = 0;
										handle_ep0(pcd);
										ptr_ep->dwc_ep.stp_rollover = 0;
										/* Prepare for more setup packets */
										if ((pcd->ep0state == EP0_IN_STATUS_PHASE) || 
											(pcd->ep0state == EP0_IN_DATA_PHASE)) {
											depctl_data_t depctl;
											depctl.d32 = 0;
											depctl.b.cnak = 1;
											ep0_out_start(ptr_core_if, pcd);
											/* Core not updating setup packet count 
											 * in case of PET testing - @TODO vahrama
											 * to check with HW team further */
											if (!ptr_core_if->otg_ver) {
												DWC_MODIFY_REG32(&ptr_core_if->dev_if->
													out_ep_regs[0]->doepctl, 0, depctl.d32);
											}
										}
										goto exit_xfercompl;
									} else {
										/* Prepare for more setup packets */
										DBG_USB_Print(DBG_USB, 
											"EP0_IDLE SR=1 setup=0 new setup comes\n");
										dev_oep_int.d32 = REG_RD(&ptr_core_if->dev_if->
																	out_ep_regs[0]->doepint);
										if(dev_oep_int.b.setup != 0) {
											goto retry;
										}
										ep0_out_start(ptr_core_if, pcd);
									}
							} else {
								dwc_otg_pcd_request_t *ptr_req;
								diepint_data_t diepint0;
								doepint_data_t doep_int_temp;
								depctl_data_t diepctl0;
								diepint0.d32 = 0;
								doep_int_temp.d32 = 0;
								diepint0.d32 = REG_RD(&ptr_core_if->dev_if->
																in_ep_regs[0]->diepint);
								diepctl0.d32 = REG_RD(&ptr_core_if->dev_if->
																in_ep_regs[0]->diepctl);
								
								if ((pcd->ep0state == EP0_IN_DATA_PHASE)
									|| (pcd->ep0state == EP0_IN_STATUS_PHASE)) {
									if (diepint0.b.xfercompl != 0) {
										REG_WR(&ptr_core_if->dev_if->
											in_ep_regs[0]->diepint, diepint0.d32);
									}
									if (diepctl0.b.epena != 0) {
										diepint_data_t dev_ep_int;
										dev_ep_int.d32 = 0;
										diepctl0.b.snak = 1;
										REG_WR(&ptr_core_if->dev_if->
														in_ep_regs[0]->diepctl, diepctl0.d32);
										do {
											dev_ep_int.d32 = REG_RD(&ptr_core_if->dev_if->
												in_ep_regs[0]->diepint);
										} while (!dev_ep_int.b.inepnakeff); 
										dev_ep_int.b.inepnakeff = 1;
										REG_WR(&ptr_core_if->dev_if->
											in_ep_regs[0]->diepint, dev_ep_int.d32);
										diepctl0.d32 = 0;
										diepctl0.b.epdis = 1;
										REG_WR(&ptr_core_if->dev_if->in_ep_regs[0]->diepctl,
														diepctl0.d32);
										do {
											dev_ep_int.d32 = REG_RD(&ptr_core_if->dev_if->
												in_ep_regs[0]->diepint);
										} while (!dev_ep_int.b.epdisabled); 
										dev_ep_int.b.epdisabled = 1;
										REG_WR(&ptr_core_if->dev_if->in_ep_regs[0]->diepint,
															dev_ep_int.d32);
									}
								}
								doep_int_temp.d32 = REG_RD(&ptr_core_if->dev_if->
																out_ep_regs[ptr_ep->dwc_ep.num]->doepint);
								if (doep_int_temp.b.sr != 0) {
									CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, sr);
									if (DWC_CIRCLEQ_EMPTY(&ptr_ep->queue) != 0) {
									} else {
										ptr_req = DWC_CIRCLEQ_FIRST(&ptr_ep->queue);
										if ((ptr_ep->dwc_ep.xfer_count != ptr_ep->dwc_ep.total_len) &&
											(pcd->ep0state == EP0_OUT_DATA_PHASE)) {
												/* Read arrived setup packet from req->buf */
												dwc_memcpy((&pcd->setup_pkt->req), 
													(&ptr_req->buf) + (ptr_ep->dwc_ep.xfer_count), 8);
										}
										ptr_req->actual = ptr_ep->dwc_ep.xfer_count;
										dwc_otg_request_done(ptr_ep, ptr_req, -ECONNRESET);
										ptr_ep->dwc_ep.start_xfer_buff = 0;
										ptr_ep->dwc_ep.xfer_buff = 0;
										ptr_ep->dwc_ep.xfer_len = 0;
									}
									pcd->ep0state = EP0_IDLE;
									if (dev_oep_int.b.setup != 0) {
										DBG_USB_Print(DBG_PCDV, "EP0_IDLE SR=1 setup=1\n");
										/* Data stage started, clear setup */
										dev_oep_int.b.setup = 0;
										CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, setup);
										
										handle_ep0(pcd);
										/* Prepare for setup packets if ep0in was enabled*/
										if (pcd->ep0state == EP0_IN_STATUS_PHASE) {
											depctl_data_t depctl ;
											depctl.d32 = 0;
											depctl.b.cnak = 1;
											ep0_out_start(ptr_core_if, pcd);
											/* Core not updating setup packet count 
											* in case of PET testing - @TODO vahrama
											* to check with HW team further */
											if (!ptr_core_if->otg_ver) {
												DWC_MODIFY_REG32(&ptr_core_if->dev_if->
														out_ep_regs[0]->doepctl, 0, depctl.d32);
											}
										}
										goto exit_xfercompl;
									} else {
										/* Prepare for more setup packets */
										ep0_out_start(ptr_core_if, pcd);
									}
								}
							}
						} 
#endif                            
						if ((ptr_core_if->dma_enable == 0) || (pcd->ep0state != EP0_IDLE)) {
							handle_ep0(pcd);
						}
exit_xfercompl:
						DBG_USB_Print(DBG_PCDV, "after DOEPINT=%x doepint=%x\n", 
							dwc_otg_read_dev_out_ep_intr(ptr_core_if, ptr_dwc_ep), dev_oep_int.d32);
					} else {
						if ((ptr_core_if->dma_desc_enable == 0)
							|| (pcd->ep0state != EP0_IDLE)) {
							handle_ep0(pcd);
							}
					}
				} 
				else {
					/* Clear the bit in DOEPINTn for this interrupt */
					CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum,
							  xfercompl);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
					if (ptr_core_if->core_params->dev_out_nak != 0) {
						pcd->core_if->ep_xfer_info[uint_epnum].state = 0;
					}
#endif
						complete_ep(ptr_ep);
					}
			}
			if (dev_oep_int.b.stsphsercvd != 0) {
				CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, stsphsercvd);
				if ((ptr_core_if->dma_desc_enable) || ((ptr_core_if->dma_enable) &&
					(ptr_core_if->snpsid >= OTG_CORE_REV_3_00a))) {
						do_setup_in_status_phase(pcd);
				}
			}
		   
			/* Endpoint disable      */
			if (dev_oep_int.b.epdisabled != 0) {
				/* Clear the bit in DOEPINTn for this interrupt */
				CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, epdisabled);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
				if (ptr_core_if->core_params->dev_out_nak != 0) {
					/* In case of timeout condition */
					if (ptr_core_if->ep_xfer_info[uint_epnum].state == 2) {
						dev_ctl.d32 = REG_RD(&ptr_core_if->dev_if->
										dev_global_regs->dctl);
						dev_ctl.b.cgoutnak = 1;
						REG_WR(&ptr_core_if->dev_if->dev_global_regs->dctl,
																dev_ctl.d32);
						/* Unmask goutnakeff interrupt which was masked
						 * during handle nak out interrupt */
						gbl_int_msk.b.goutnakeff = 1;
						DWC_MODIFY_REG32(&ptr_core_if->core_global_regs->gintmsk,
																0, gbl_int_msk.d32);
					
						complete_ep(ptr_ep);
					}
				}
#endif
			}
			/* AHB Error */
			if (dev_oep_int.b.ahberr != 0) {
				CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, ahberr);
			}
			/* Setup Phase Done (contorl EPs) */
			if (dev_oep_int.b.setup != 0) {
				CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, setup);

				handle_ep0(pcd);
			}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			/** OUT EP BNA Intr */
			if (dev_oep_int.b.bna != 0) {
				CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, bna);
				if (ptr_core_if->dma_desc_enable != 0) {
						dwc_otg_pcd_handle_noniso_bna(ptr_ep);
					}
			}
                /* Babble Interrupt */
			if (dev_oep_int.b.babble != 0) {
				handle_out_ep_babble_intr(pcd, uint_epnum);

				CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, babble);
			}
#endif
			if (dev_oep_int.b.outtknepdis != 0) {
					CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, outtknepdis);
			}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			/* NAK Interrutp */
			if (dev_oep_int.b.nak != 0) {
				handle_out_ep_nak_intr(pcd, uint_epnum);

				CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, nak);
			}
			/* NYET Interrutp */
			if (dev_oep_int.b.nyet != 0) {
				handle_out_ep_nyet_intr(pcd, uint_epnum);

				CLEAR_OUT_EP_INTR(ptr_core_if, uint_epnum, nyet);
			}
#endif
		}

		uint_epnum++;
		ep_intr >>= 1;
	}
	(void)dev_oep_int.b.xfercompl;
	return 1;

#undef CLEAR_OUT_EP_INTR
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
/**
 * This function handles the Global IN NAK Effective interrupt.
 *
 */
int32_t dwc_otg_pcd_handle_in_nak_effective(dwc_otg_pcd_t * pcd)
{
	dwc_otg_dev_if_t *ptr_dev_if = GET_CORE_IF(pcd)->dev_if;
	depctl_data_t dev_ep_ctl; 
	gintmsk_data_t intr_mask; 
	gintsts_data_t gbl_int_sts;
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
	int32_t i;
	uint32_t j;
	dev_ep_ctl.d32 = 0 ;
	intr_mask.d32 = 0 ;

	DBG_USB_Print(DBG_PCD, "Global IN NAK Effective\n");

	/* Disable all active IN EPs */
	for (i = 0; i <= ptr_dev_if->num_in_eps; i++) {
		dev_ep_ctl.d32 = REG_RD(&ptr_dev_if->in_ep_regs[i]->diepctl);
		j = ((dev_ep_ctl.b.eptype) & 1);
		if (((!j)) && (dev_ep_ctl.b.epena)) {
			if (ptr_core_if->start_predict > 0) {
				ptr_core_if->start_predict++;
			}
			dev_ep_ctl.b.epdis = 1;
			dev_ep_ctl.b.snak = 1;
			REG_WR(&ptr_dev_if->in_ep_regs[i]->diepctl, dev_ep_ctl.d32);
		}						
	}
	

	/* Disable the Global IN NAK Effective Interrupt */
	intr_mask.b.ginnakeff = 1;
	DWC_MODIFY_REG32(&GET_CORE_IF(pcd)->core_global_regs->gintmsk,
			 intr_mask.d32, 0);

	/* Clear interrupt */
	gbl_int_sts.d32 = 0;
	gbl_int_sts.b.ginnakeff = 1;
	REG_WR(&GET_CORE_IF(pcd)->core_global_regs->gintsts,
			gbl_int_sts.d32);

	return 1;
}

/**
 * OUT NAK Effective.
 *
 */
int32_t dwc_otg_pcd_handle_out_nak_effective(dwc_otg_pcd_t * pcd)
{
	dwc_otg_dev_if_t *ptr_dev_if = GET_CORE_IF(pcd)->dev_if;
	gintmsk_data_t intr_mask; 
	gintsts_data_t gbl_int_sts;
	depctl_data_t dev_oep_ctl;
	int32_t i;
	intr_mask.d32 = 0 ;

	/* Disable the Global OUT NAK Effective Interrupt */
	intr_mask.b.goutnakeff = 1;
	DWC_MODIFY_REG32(&GET_CORE_IF(pcd)->core_global_regs->gintmsk,
			 intr_mask.d32, 0);

	/* If DEV OUT NAK enabled */
	if (pcd->core_if->core_params->dev_out_nak != 0) {
		/* Run over all out endpoints to determine the ep number on
		 * which the timeout has happened 
		 */
		for (i = 0; i <= ptr_dev_if->num_out_eps; i++) {
			if (pcd->core_if->ep_xfer_info[i].state == 2) {
				break;
			}
		}
		if (i > ptr_dev_if->num_out_eps) {
			dctl_data_t dev_ctl;
			dev_ctl.d32 =
			    REG_RD(&ptr_dev_if->dev_global_regs->dctl);
			dev_ctl.b.cgoutnak = 1;
			REG_WR(&ptr_dev_if->dev_global_regs->dctl,
					dev_ctl.d32);
			/* Clear interrupt */
			gbl_int_sts.d32 = 0;
			gbl_int_sts.b.goutnakeff = 1;
			REG_WR(&GET_CORE_IF(pcd)->core_global_regs->gintsts,
			gbl_int_sts.d32);
		}else {

			/* Disable the endpoint */
			dev_oep_ctl.d32 = REG_RD(&ptr_dev_if->out_ep_regs[i]->doepctl);
			if (dev_oep_ctl.b.epena != 0) {
				dev_oep_ctl.b.epdis = 1;
				dev_oep_ctl.b.snak = 1;
			}
			REG_WR(&ptr_dev_if->out_ep_regs[i]->doepctl, dev_oep_ctl.d32);
		}
	}else {
		/* Clear interrupt */
			gbl_int_sts.d32 = 0;
			gbl_int_sts.b.goutnakeff = 1;
			REG_WR(&GET_CORE_IF(pcd)->core_global_regs->gintsts,
			gbl_int_sts.d32);
	}
	return 1;
}
#endif

/**
 * PCD interrupt handler.
 *
 * The PCD handles the device interrupts.  Many conditions can cause a
 * device interrupt. When an interrupt occurs, the device interrupt
 * service routine determines the cause of the interrupt and
 * dispatches handling to the appropriate function. These interrupt
 * handling functions are described below.
 *
 * All interrupt registers are processed from LSB to MSB.
 *
 */
int32_t dwc_otg_pcd_handle_intr(dwc_otg_pcd_t * pcd)
{
	dwc_otg_core_if_t *ptr_core_if = GET_CORE_IF(pcd);
#ifdef VERBOSE
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
#endif
	gintsts_data_t gintr_status;
	uint32_t retval = 0;
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	if (dwc_otg_check_haps_status(ptr_core_if) == -1 ) {
		DBG_Warn_Print("HAPS is disconnected");			
		;
	}else {

		/* Exit from ISR if core is hibernated */
		if (ptr_core_if->hibernation_suspend == 1) {
			;
		}else {
#ifdef VERBOSE
			DBG_USB_Print(DBG_USB, "%s() gintsts=%08x	 gintmsk=%08x\n",
				__func__,
			REG_RD(&global_regs->gintsts),
			REG_RD(&global_regs->gintmsk));
#endif

#ifdef VERBOSE
			DBG_USB_Print(DBG_PCDV, "%s() gintsts=%08x  gintmsk=%08x\n",
				__func__,
			REG_RD(&global_regs->gintsts),
			REG_RD(&global_regs->gintmsk));
#endif

			gintr_status.d32 = dwc_otg_read_core_intr(ptr_core_if);
	
			if (gintr_status.b.usbreset != 0) {
				retval |= (uint32_t)(dwc_otg_pcd_handle_usb_reset_intr(pcd));
			}
			if (gintr_status.b.enumdone != 0) {
				retval |= (uint32_t)dwc_otg_pcd_handle_enum_done_intr(pcd);
			}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			if (gintr_status.b.sofintr != 0) {
				retval |= (uint32_t)dwc_otg_pcd_handle_sof_intr(pcd);
			}
			if (gintr_status.b.rxstsqlvl != 0) {
				retval |=
					(uint32_t)dwc_otg_pcd_handle_rx_status_q_level_intr(pcd);
			}
			if (gintr_status.b.nptxfempty != 0) {
				retval |= (uint32_t)dwc_otg_pcd_handle_np_tx_fifo_empty_intr(pcd);
			}
			if (gintr_status.b.goutnakeff != 0) {
				retval |= (uint32_t)dwc_otg_pcd_handle_out_nak_effective(pcd);
			}
			if (gintr_status.b.i2cintr != 0) {
				retval |= (uint32_t)dwc_otg_pcd_handle_i2c_intr(pcd);
			}
			if (gintr_status.b.erlysuspend != 0) {
				retval |= (uint32_t)dwc_otg_pcd_handle_early_suspend_intr(pcd);
			}
			if (gintr_status.b.eopframe != 0) {
				retval |=
					(uint32_t)dwc_otg_pcd_handle_end_periodic_frame_intr(pcd);
			}
#endif
			if (gintr_status.b.inepint != 0) {
				if (!ptr_core_if->multiproc_int_enable) {
					retval |= (uint32_t)dwc_otg_pcd_handle_in_ep_intr(pcd);
				}
			}
			if (gintr_status.b.outepintr != 0) {
				if (!ptr_core_if->multiproc_int_enable) {
					retval |= (uint32_t)dwc_otg_pcd_handle_out_ep_intr(pcd);
				}
			}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			if (gintr_status.b.epmismatch != 0) {
				retval |= (uint32_t)dwc_otg_pcd_handle_ep_mismatch_intr(pcd);
			}
			if (gintr_status.b.fetsusp != 0) {
				retval |= (uint32_t)dwc_otg_pcd_handle_ep_fetsusp_intr(pcd);
			}
			if (gintr_status.b.ginnakeff != 0) {
				retval |= (uint32_t)dwc_otg_pcd_handle_in_nak_effective(pcd);
			}
#endif
			/* In MPI mode Device Endpoints interrupts are asserted
			 * without setting outepintr and inepint bits set, so these
			 * Interrupt handlers are called without checking these bit-fields
			 */
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			if (ptr_core_if->multiproc_int_enable != 0) {
				retval |= (uint32_t)dwc_otg_pcd_handle_in_ep_intr(pcd);
				retval |= (uint32_t)dwc_otg_pcd_handle_out_ep_intr(pcd);
			}
#endif
		}
	}
	return ((int32_t)retval);
}



