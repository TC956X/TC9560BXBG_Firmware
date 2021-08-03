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

/*
*********************************************************************************************************
*											INCLUDE FILES
*********************************************************************************************************
*/

#include  <includes.h>
#include  <tc9560_common.h>
#include "DWC_ETH_QOS_yregacc.h"
#include  "tc9560_uart.h"
#include "tc9560_reg_define.h"
#include "dwc_otg_dbg.h"
#include "tc9560_defs.h"

#define DEF_YES 1
#define DEF_NO  0

uint32_t TxDescNbr = TX_DESC_CNT;
uint32_t RxDescNbr = RX_DESC_CNT;
uint32_t RxBufLargeSize = 1500;
uint32_t  BaseAddr   = 0x4000a000;
uint32_t PhyBaseAddr = 1;
uint32_t DMABaseAddr = 0x40003000;

/*
*********************************************************************************************************
*											LOCAL DEFINES
*
* Note(s) : (1) Receive buffers usually MUST be aligned to some octet boundary.  However, adjusting 
*			   receive buffer alignment MUST be performed from within 'net_dev_cfg.h'.  Do not adjust 
*			   the value below as it is used for configuration checking only.
*********************************************************************************************************
*/

#define  RX_BUF_ALIGN_OCTETS				16u				 /* Rx bufs must be aligned to a 16 byte boundary.	   */
#define  TX_BUF_ALIGN_OCTETS				16u				 /* Tx bufs must be aligned to a 16 byte boundary.	   */
#define  RX_BUF_SIZE_MULT				   16u				 /* Rx bufs must be multiple of 16 bytes.				*/
#define  DEV_MTU_SIZE					   1600u			   /* Maximum Transmission Unit.						   */
#define  DEV_PTP_SIZE					   0xA0u			   /* Maximum Transmission Unit.						   */


/*
*********************************************************************************************************
*											LOCAL TABLES
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*									  LOCAL FUNCTION PROTOTYPES
*
* Note(s) : (1) Device driver functions may be arbitrarily named.  However, it is recommended that device 
*			   driver functions be named using the names provided below.  All driver function prototypes 
*			   should be located within the driver C source file ('net_dev_&&&.c') & be declared as 
*			   static functions to prevent name clashes with other network protocol suite device drivers.
*********************************************************************************************************
*/
/* ------------ FNCT'S COMMON TO ALL DEV'S ------------ */

static  void  NetDev_Init			   (void);
static  void  NetDev_Rx				 (void);
uint8_t first_time = 1;
static uint32_t DWC_ETH_QOS_reg_read(volatile ULONG *ptr);
uint8_t *buf_addr = NULL;		
synopGMACdevice	*gmacdev;
static void synopGMAC_set_rx_desc_rng_len(synopGMACdevice const *gbl_mac_dev, CPU_INT32U rnglen, CPU_INT08U chno);     
static void synopGMAC_set_tx_desc_rng_len(synopGMACdevice const *gbl_mac_dev, CPU_INT32U rnglen, CPU_INT08U chno);

/*
*********************************************************************************************************
*									   LOCAL GLOBAL VARIABLES
*
* Note(s) : (1) Global variables are highly discouraged and should only be used for storing NON-instance 
*			   specific data and the array of instance specific data.  Global variables, those that are 
*			   not declared within the NET_DEV_DATA area, are not multiple-instance safe and could lead 
*			   to incorrect driver operation if used to store device state information.
*********************************************************************************************************
*/

void tc9560_init_network(void)
{
	NetDev_Init();
	NetDev_Start();
  REG_WR(NVIC_ISER0,(REG_RD(NVIC_ISER0) | (0x003FF800)));
}	

uint8_t BULKOUT2EDMACH[] = {
	2,  /* BOUT0: legacy */
	3,
	4,
	0
};
	
void usb_start_transmit(CPU_INT08U const *p_data, struct usb_request *req, CPU_INT08U bulk_out_num)
{
	CPU_INT32U size = req->actual;
	req->bulk_out_num = bulk_out_num;
	NetDev_Tx (p_data,size,BULKOUT2EDMACH[bulk_out_num], req);
}	

/*Gmac configuration functions*/
/**
  * This is a wrapper function for platform dependent delay 
  * Take care while passing the argument to this function 
  * @param[in] buffer pointer to be freed
  */
void plat_delay(CPU_INT32U delay)
{
	while (delay != 0){
		delay--;
	}
	return;
}

/** 
  * Checks whether the tx descriptor is owned by DMA.
  * If descriptor is owned by DMA then the OWN bit is set to 1. This API is same for both ring and chain mode.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if Dma owns descriptor and false if not.
  */
bool synopGMAC_is_tx_desc_owned_by_dma(s_tx_norm_desc const * desc)
{
	/* need to check this */
	bool return_value;
	if((desc->TDES3 & ((uint32_t)DescOwnDMA)) == ((uint32_t)DescOwnDMA)) {
		return_value = 1;
	}
	else {
		return_value =0;
	}
	return (return_value);
}

/** 
  * Checks whether the tx descriptor is owned by DMA.
  * If descriptor is owned by DMA then the OWN bit is set to 1. This API is same for both ring and chain mode.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if Dma owns descriptor and false if not.
  */
bool synopGMAC_is_rx_desc_owned_by_dma(s_rx_norm_desc const * desc)
{
	/* need to check this */
	bool return_value;
	if((desc->RDES3 & (CPU_INT32U)DescOwnDMA) == (CPU_INT32U)DescOwnDMA) {
		return_value = 1;
	}
	else {
		return_value = 0;
	}
	return (return_value);
}

/**
  * Checks whether the descriptor is valid
  * if no errors such as CRC/Receive Error/Watchdog Timeout/Late collision/Giant Frame/Overflow/Descriptor
  * error the descritpor is said to be a valid descriptor.
  * @param[in] pointer to DmaDesc structure.
  * \return True if desc valid. false if error.
  */
bool synopGMAC_tx_has_error(CPU_INT32U status) /* status = TDES3 */
{
	bool return_value;
	if((status & (1 << 15) ) == 0) {
		return_value = 1;
	}
	else {
		return_value = 0;
	}
	return (return_value);  /* bit 15: Error Summary */
}

/**
  * Get the index and address of Tx desc.
  * This api is same for both ring mode and chain mode.
  * This function tracks the tx descriptor the DMA just closed after the transmission of data from this descriptor is 
  * over. This returns the descriptor fields to the caller.
  * @param[in] pointer to synopGMACdevice.
  * @param[out] status field of the descriptor.
  * @param[out] Dma-able buffer1 pointer.
  * @param[out] length of buffer1 (Max is 2048).
  * @param[out] virtual pointer for buffer1.
  * @param[out] Dma-able buffer2 pointer.
  * @param[out] length of buffer2 (Max is 2048).
  * @param[out] virtual pointer for buffer2.
  * @param[out] CPU_INT32U data indicating whether the descriptor is in ring mode or chain mode.
  * \return returns present tx descriptor index on success. Negative value if error.
  */
CPU_INT32S synopGMAC_get_tx_qptr(synopGMACdevice *gbl_mac_dev, CPU_INT32U *Status, CPU_INT32U txch)  /* from Tx Write-back format */
{
	/* CPU_INT32U 			tx_desc_rng_len; */
	CPU_INT32S retval;
	CPU_INT32U	  txover  = gbl_mac_dev->TxBusy[txch];
	s_tx_norm_desc *txdesc  = gbl_mac_dev->TxBusyDesc[txch];
	
	if(synopGMAC_is_tx_desc_owned_by_dma(txdesc) != 0) {
		retval = -1;
	}
	else {
	  if (txdesc->TDES3 == 0) {
			retval = -1;
		}
		else {
			(gbl_mac_dev->BusyTxDescNo[txch])--; /* busy tx descriptor is reduced by one as it will be handed over to Processor now */

			if(Status != 0)   {
				*Status = txdesc->TDES3;
			}
			if((txover + 1) > (TX_DESC_CNT - 1)){
				gbl_mac_dev->TxBusy[txch] = 0;
				gbl_mac_dev->TxBusyDesc[txch] = gbl_mac_dev->TxDesc[txch];
			}
			else{
				gbl_mac_dev->TxBusy[txch] = txover + 1;
				gbl_mac_dev->TxBusyDesc[txch] = (txdesc + 1);
			}

			/* need to check why this is required here */
			synopGMAC_tx_desc_init_ring(txdesc);
			retval = (int32_t)txover;
		}
	}
	return retval;
}

/**
  * Get back the descriptor from DMA after data has been received.
  * When the DMA indicates that the data is received (interrupt is generated), this function should be
  * called to get the descriptor and hence the data buffers received. With successful return from this
  * function caller gets the descriptor fields for processing. check the parameters to understand the 
  * fields returned.`
  * @param[in] pointer to synopGMACdevice.
  * @param[out] pointer to hold the status of DMA.
  * @param[out] Dma-able buffer1 pointer.
  * @param[out] pointer to hold length of buffer1 (Max is 2048).
  * @param[out] virtual pointer for buffer1.
  * @param[out] Dma-able buffer2 pointer.
  * @param[out] pointer to hold length of buffer2 (Max is 2048).
  * @param[out] virtual pointer for buffer2.
  * \return returns present rx descriptor index on success. Negative value if error.
  */
CPU_INT32S synopGMAC_get_rx_qptr(synopGMACdevice *gbl_mac_dev, CPU_INT32U *Status, CPU_INT32U *Buffer, CPU_INT32U *Length, CPU_INT08U chno)
{
	CPU_INT32S retval;
	CPU_INT32U rxnext		 = gbl_mac_dev->RxBusy[chno];	/* index of descriptor the DMA just completed. May be useful when data */
	static CPU_INT32U old_len = 0;	/*is spread over multiple buffers/descriptors */
	s_rx_norm_desc* rxdesc	= gbl_mac_dev->RxBusyDesc[chno];

	if(synopGMAC_is_rx_desc_owned_by_dma(rxdesc) != 0) {
		retval = -1;
	} else {
		if(Status != 0){
			*Status 	  = rxdesc->RDES0;
			*(Status+1) = rxdesc->RDES1;
			*(Status+2) = rxdesc->RDES2;
			*(Status+3) = rxdesc->RDES3;/* send the status of this descriptor */

			if((Status[3] & DescRxFD) == DescRxFD){
				old_len = Status[3] & DescRxPL;
				*Length = old_len;
			}else{
				*Length = (Status[3] & DescRxPL) - old_len;
			}

			if(synopGMAC_is_rx_desc_empty(rxdesc) != 0) {
				*Length = 0; 
				if (( Status[3] & (1 << 30) ) != 0) {  /* Context desc */
					*Buffer = (CPU_INT32U)gbl_mac_dev->RDES0Val[chno][rxnext];
				}
				else {
					*Buffer = 0;
				}
			}
			else {	
				*Buffer = (CPU_INT32U)gbl_mac_dev->RDES0Val[chno][rxnext];
			}
		}
		if((rxnext+1) > (RX_DESC_CNT-1))
		{
			gbl_mac_dev->RxBusy[chno] = 0;
			gbl_mac_dev->RxBusyDesc[chno] = gbl_mac_dev->RxDesc[chno];
		}else{
			gbl_mac_dev->RxBusy[chno] = rxnext + 1;
			gbl_mac_dev->RxBusyDesc[chno] = (rxdesc + 1);
		}
		
		(gbl_mac_dev->BusyRxDescNo[chno])--; /*This returns one descriptor to processor. So busy count will be decremented by one */

		retval = (int32_t)rxnext;
	}
	return retval;
}

/**
  * Take ownership of all the rx Descriptors.
  * This function is called when there is fatal error in DMA transmission.
  * When called it takes the ownership of all the rx descriptor in rx descriptor pool/queue from DMA.
  * The function is same for both the ring mode and the chain mode DMA structures.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  * \note Make sure to disable the transmission before calling this function, otherwise may result in racing situation.
  */
void synopGMAC_take_all_rx_desc_ownership(synopGMACdevice const * gbl_mac_dev)
{
	CPU_INT32S i;
	s_rx_norm_desc* ptr_desc;
	CPU_INT08U chno = 0;
	
	for (chno = 0; chno < RX_NO_CHAN; chno++) {
	ptr_desc = gbl_mac_dev->RxDesc[chno];
	for(i = 0; i < RX_DESC_CNT; i++){
		ptr_desc->RDES3 &= ((~(uint32_t)DescOwnDMA));
				ptr_desc = (ptr_desc+1);
	}
	}
}

/**
  * Sets the Mac address in to GMAC register.
  * This function sets the MAC address to the MAC register in question.
  * @param[in] pointer to synopGMACdevice to populate mac dma and phy addresses.
  * @param[in] Register offset for Mac address high
  * @param[in] Register offset for Mac address low
  * @param[in] buffer containing mac address to be programmed.
  * \return 0 upon success. Error code upon failure.
  */
CPU_INT32S synopGMAC_set_mac_addr( CPU_INT08U const *MacAddr)
{
	/* update the MAC address */
#if 1 
	MAC_MA0HR_RgWr(((((uint32_t) MacAddr[5]) << 8)  | (MacAddr[4])));
	MAC_MA0LR_RgWr(((((uint32_t) MacAddr[3]) << 24) | (((uint32_t)MacAddr[2]) << 16) | (((uint32_t)MacAddr[1]) << 8) | (MacAddr[0])));
#else /* use filter 1 for M3 MAC */
	MAC_MA0HR_RgWr(((0xaa << 8)  | (0x88)));
	MAC_MA0LR_RgWr(((0x66 << 24) | (0x44 << 16) | (0x22 << 8) | (0x00))); 
	
	
	MAC_MA1HR_AE_UdfWr(1);
	MAC_MA1HR_ADDRHI_UdfWr ((MacAddr[5] << 8)  | (MacAddr[4]));
	MAC_MA1LR_RgWr(((MacAddr[3] << 24) | (MacAddr[2] << 16) | (MacAddr[1] << 8) | (MacAddr[0])));  /* M3 Mac */
#endif

	return 0;
}
/**
  * Populate the tx desc structure with the buffer address.
  * Once the driver has a packet ready to be transmitted, this function is called with the 
  * valid dma-able buffer addresses and their lengths. This function populates the descriptor
  * and make the DMA the owner for the descriptor. This function also controls whetther Checksum
  * offloading to be done in hardware or not. 
  * This api is same for both ring mode and chain mode.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] Dma-able buffer1 pointer.
  * @param[in] length of buffer1 (Max is 2048).
  * @param[in] virtual pointer for buffer1.
  * @param[in] Dma-able buffer2 pointer.
  * @param[in] length of buffer2 (Max is 2048).
  * @param[in] virtual pointer for buffer2.
  * @param[in] CPU_INT32U data indicating whether the descriptor is in ring mode or chain mode.
  * @param[in] CPU_INT32U indicating whether the checksum offloading in HW/SW.
  * \return returns present tx descriptor index on success. Negative value if error.
  */
CPU_INT32S synopGMAC_set_tx_qptr(synopGMACdevice * gbl_mac_dev, CPU_INT32U Buffer, CPU_INT32U Length, 
								 CPU_INT32U offload_needed, CPU_INT08U txch, struct usb_request *req)
{
	CPU_INT32S retval;
	CPU_INT32S  txnext	  = (int32_t)gbl_mac_dev->TxNext[txch];
	s_tx_norm_desc* txdesc  = gbl_mac_dev->TxNextDesc[txch];
	/*static unsigned int32_t launch_ts = 0; */
	
	if(synopGMAC_is_tx_desc_owned_by_dma(txdesc) != 0) {
		retval = -1;
	}
	else {
		(gbl_mac_dev->BusyTxDescNo[txch])++; /*busy tx descriptor is incremented by one as it will be handed over to DMA */
		
		txdesc->TDES0 = Buffer;
		gbl_mac_dev->TDES0Val[txch][gbl_mac_dev->TxNext[txch]] = Buffer;
		gbl_mac_dev->USBREQ[txch][gbl_mac_dev->TxNext[txch]] = req;
		
		txdesc->TDES2 = (CPU_INT32U)(Length & DescBufLen) | (CPU_INT32U)DescIOC | (CPU_INT32U)DescTTSE;
#if 1 /* To timestamp comp */
		if((txch == 3) || (txch == 4)){
			txdesc->TDES2	&= ~(0x3U<<21);	
			txdesc->TDES2	|= (0x1U<<21);	
			
			/*launch_ts += 125000; */
			/*txdesc->TDES1 = launch_ts; */
		}
#endif	
		txdesc->TDES3 = (CPU_INT32U)DescOwnDMA | (Length & DescPktLen) | (DescFD	| DescLD) | DescCIC;
		
		if((txnext + 1) > (TX_DESC_CNT - 1)){
			gbl_mac_dev->TxNext[txch] = 0;
			gbl_mac_dev->TxNextDesc[txch] = gbl_mac_dev->TxDesc[txch];
		}else{
			gbl_mac_dev->TxNext[txch] = (uint32_t)txnext + 1;
			gbl_mac_dev->TxNextDesc[txch] = (txdesc + 1);
		}
		(void)offload_needed;
		retval = txnext;
	}
	return retval;
}
/**
  * Initialize the tx descriptors for ring or chain mode operation.
  * 	- Status field is initialized to 0.
  *	- EndOfRing set for the last descriptor.
  *	- buffer1 and buffer2 set to 0 for ring mode of operation. (note)
  *	- data1 and data2 set to 0. (note)
  * @param[in] pointer to DmaDesc structure.
  * @param[in] whether end of ring
  * \return void.
  * \note Initialization of the buffer1, buffer2, data1,data2 and status are not done here. This only initializes whether one wants to use this descriptor
  * in chain mode or ring mode. For chain mode of operation the buffer2 and data2 are programmed before calling this function.
  */
void synopGMAC_tx_desc_init_ring(s_tx_norm_desc* desc)
{
	desc->TDES0 = 0;
	desc->TDES1 = 0;
	desc->TDES2 = 0;
	desc->TDES3 = 0x00000000;
	return;
}

static void synopGMAC_set_rx_desc_rng_len(synopGMACdevice const *gbl_mac_dev, CPU_INT32U rnglen, CPU_INT08U chno)
{
	CPU_INT32U regoffset = 0;
		
	regoffset = (uint32_t)(DmaRxChDescRngLn00 + (chno * DMA_CHANNEL_REG_LEN));
	synopGMACWriteReg((CPU_INT32U*)gbl_mac_dev->DmaBase, regoffset, rnglen);
}

static void synopGMAC_set_tx_desc_rng_len(synopGMACdevice const *gbl_mac_dev, CPU_INT32U rnglen, CPU_INT08U chno)
{
	CPU_INT32U regoffset = 0;
	
	regoffset = (uint32_t)DmaTxChDescRngLn00 + (chno * DMA_CHANNEL_REG_LEN);
	synopGMACWriteReg((CPU_INT32U*)gbl_mac_dev->DmaBase, regoffset, rnglen);
}
/**
  * Enable the watchdog timer on the receiver. 
  * When enabled, Gmac enables Watchdog timer, and GMAC allows no more than
  * 2048 bytes of data (10,240 if Jumbo frame enabled).
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_wd_enable(void)
{
	MAC_MCR_WD_UdfWr(0U);  /* Clear WD bit */
	
	return;
}

/**
  * Enables the Jabber frame support. 
  * When enabled, GMAC disabled the jabber timer, and can transfer 16,384 byte frames.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_jab_enable(void)
{
	MAC_MCR_JD_UdfWr(1U);;
	return;
}

/**
  * Enables Frame bursting (Only in Half Duplex Mode). 
  * When enabled, GMAC allows frame bursting in GMII Half Duplex mode.
  * Reserved in 10/100 and Full-Duplex configurations.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_frame_burst_enable(void)
{
	MAC_MCR_BE_UdfWr(1U);  /* Set Enable bit */
	return;
}

/**
  * Disable Jumbo frame support. 
  * When Disabled GMAC does not supports jumbo frames.
  * Giant frame error is reported in receive frame status.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_jumbo_frame_disable(void)
{
	MAC_MCR_JE_UdfWr(0U);
	return;
}

/**
  * Enables Receive Own bit (Only in Half Duplex Mode). 
  * When enaled GMAC receives all the packets given by phy while transmitting.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_rx_own_enable(void)
{
	MAC_MCR_DRO_UdfWr(0U); /* Clear "Disable Rx Own" bit */
	return;
}
/**
  * GMAC tries retransmission (Only in Half Duplex mode).
  * If collision occurs on the GMII/MII, GMAC attempt retries based on the 
  * back off limit configured. 
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  * \note This function is tightly coupled with synopGMAC_back_off_limit(synopGMACdev *, CPU_INT32U).
  */
void synopGMAC_retry_enable(void)
{
	MAC_MCR_DR_UdfWr(0U); /* Clear "Disable Retry" bit */
	return;
}
/**
  * GMAC doesnot strips the Pad/FCS field of incoming frames.
  * GMAC will pass all the incoming frames to Host unmodified. 
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_pad_crc_strip_disable(void)
{
	MAC_MCR_ACS_UdfWr(0U);
	return;
}
/**
  * GMAC programmed with the back off limit value.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  * \note This function is tightly coupled with synopGMAC_retry_enable(synopGMACdevice * gmacdev)
  */
void synopGMAC_back_off_limit( CPU_INT32U value )
{
	MAC_MCR_BL_UdfWr(value);
	return;
}
/**
  * Disables the Deferral check in GMAC (Only in Half Duplex mode).
  * GMAC defers until the CRS signal goes inactive.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_deferral_check_disable(void)
{
	MAC_MCR_DEFC_UdfWr(0);
	return;
}
/*Receive frame filter configuration functions*/
/**
  * Enables reception of all the frames to application.
  * GMAC passes all the frames received to application irrespective of whether they
  * pass SA/DA address filtering or not.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_frame_filter_enable(void)
{
	MAC_MPFR_RA_UdfWr(0); /* Clear "Rx All" bit */
	return;
}
/**
  * Enables Hash or Perfect filter (only if Hash filter is enabled in H/W).
  * Only frames matching either perfect filtering or Hash Filtering as per HMC and HUC 
  * configuration are sent to application.
  * @param[in] pointer to synopGMACdevice.
  * \return void. 
  */
void synopGMAC_hash_perfect_filter_enable(void)
{
	MAC_MPFR_HPF_UdfWr(1);
	return;
}
/**
  * Disables Source address filtering.
  * When disabled GMAC forwards the received frames with updated SAMatch bit in RxStatus. 
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_src_addr_filter_disable(void)
{
	MAC_MPFR_SAF_UdfWr(0);
	return;
}
/**
  * Enables the normal Destination address filtering.
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_dst_addr_filter_normal(void)
{
	MAC_MPFR_DAIF_UdfWr(0);
	return;
}
/**
  * Enables forwarding of control frames.
  * When set forwards all the control frames (incl. unicast and multicast PAUSE frames).
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  * \note Depends on RFE of FlowControlRegister[2]
  */
void synopGMAC_set_pass_control(CPU_INT32U passcontrol)
{	
	MAC_MPFR_PCF_UdfWr(passcontrol);
	return;
}
/**
  * Enables Broadcast frames.
  * When enabled Address filtering module passes all incoming broadcast frames.
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_broadcast_enable(void)
{
	MAC_MPFR_DBF_UdfWr(0);
	return;
}
/**
  * Disable Multicast frames.
  * When disabled multicast frame filtering depends on HMC bit.
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_multicast_disable(void)
{
	MAC_MPFR_PM_UdfWr(0);
	return;
}
/**
  * Disables multicast hash filtering.
  * When disabled GMAC performs perfect destination address filtering for multicast frames, it compares 
  * DA field with the value programmed in DA register.
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_multicast_hash_filter_disable(void)
{
	MAC_MPFR_HMC_UdfWr(0);
	return;
}
/**
  * Disables multicast hash filtering.
  * When disabled GMAC performs perfect destination address filtering for unicast frames, it compares 
  * DA field with the value programmed in DA register.
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_unicast_hash_filter_disable(void)
{
	MAC_MPFR_HUC_UdfWr(0);
	return;
}
/**
  * Example mac initialization sequence.
  * This function calls the initialization routines to initialize the GMAC register.
  * One can change the functions invoked here to have different configuration as per the requirement
  * @param[in] pointer to synopGMACdevice.
  * \return Returns 0 on success.
  */
CPU_INT32S synopGMAC_mac_init(synopGMACdevice const * gbl_mac_dev)
{
	CPU_INT32U PHYreg;
	
	if(gbl_mac_dev->DuplexMode == FULLDUPLEX)
	{	
		synopGMAC_wd_enable( );
		synopGMAC_jab_enable( );
		synopGMAC_frame_burst_enable( );
		synopGMAC_jumbo_frame_disable( );
		synopGMAC_rx_own_enable( );
		config_mac_loopback_mode(0U);
		set_full_duplex( );
		synopGMAC_retry_enable( );
		synopGMAC_pad_crc_strip_disable( );
		synopGMAC_back_off_limit(GmacBackoffLimit0);
		synopGMAC_deferral_check_disable( );

		if(gbl_mac_dev->Speed == SPEED1000) {
			set_gmii_speed( );
		}
		else if(gbl_mac_dev->Speed == SPEED100) {
			set_mii_speed_100( );
		}
		else {
			set_mii_speed_10( );
		}

		/*Frame Filter Configuration*/
		synopGMAC_frame_filter_enable( );
		synopGMAC_set_pass_control(GmacPassControl0);
		synopGMAC_broadcast_enable( );
		synopGMAC_src_addr_filter_disable( );
		synopGMAC_multicast_disable( );
		synopGMAC_dst_addr_filter_normal( );
		synopGMAC_multicast_hash_filter_disable( );
		disable_promiscuous_mode( );
		config_mac_pkt_filter_reg(0, /* pr_mode, */
							  1, /* huc_mode, */
							  0, /* hmc_mode, */
							  0, /* pm_mode, */
							  1  /*  hpf_mode */ );

		
		/*Flow Control Configuration*/
		enable_rx_flow_ctrl( );
		enable_tx_flow_ctrl(TX_QUEUE0);
	}
	else{/*for Half Duplex configuration */
		synopGMAC_wd_enable( );
		synopGMAC_jab_enable( );
		synopGMAC_frame_burst_enable( );
		synopGMAC_jumbo_frame_disable( );
		synopGMAC_rx_own_enable( );
		config_mac_loopback_mode(0U);  /* loopback mode Off */
		set_half_duplex( );
		synopGMAC_retry_enable( );
		synopGMAC_pad_crc_strip_disable( );
		synopGMAC_back_off_limit(GmacBackoffLimit0);
		synopGMAC_deferral_check_disable( );
		
		if(gbl_mac_dev->Speed == SPEED1000) {
			set_gmii_speed( );
		}
		else if(gbl_mac_dev->Speed == SPEED100) {
			set_mii_speed_100( );
		}
		else {
			set_mii_speed_10( );
		}

		/*Frame Filter Configuration*/
	 	synopGMAC_frame_filter_enable( );
		synopGMAC_set_pass_control(GmacPassControl0);
		synopGMAC_broadcast_enable( );
		synopGMAC_src_addr_filter_disable( );
		synopGMAC_multicast_disable( );
		synopGMAC_dst_addr_filter_normal( );
		synopGMAC_multicast_hash_filter_disable( );
		disable_promiscuous_mode( );
		synopGMAC_unicast_hash_filter_disable( );
		
		/*Flow Control Configuration*/
		disable_rx_flow_ctrl( );
		disable_tx_flow_ctrl(TX_QUEUE0);

		/*To set PHY register to enable CRS on Transmit*/
		synopGMACWriteReg((CPU_INT32U *)gbl_mac_dev->MacBase, GmacGmiiAddr, GmiiBusy | 0x00000408);
		PHYreg = synopGMACReadReg((CPU_INT32U *)gbl_mac_dev->MacBase,GmacGmiiData);
		synopGMACWriteReg((CPU_INT32U *)gbl_mac_dev->MacBase, GmacGmiiData, PHYreg   | 0x00000800);
		synopGMACWriteReg((CPU_INT32U *)gbl_mac_dev->MacBase, GmacGmiiAddr, GmiiBusy | 0x0000040a);
	}
	return 0;
}
/**
  * Checks whether the tx descriptor is empty.
  * If the buffer1 and buffer2 lengths are zero in ring mode descriptor is empty.
  * In chain mode buffer2 length is 0 but buffer2 itself contains the next descriptor address.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if descriptor is empty, false if not empty.
  */
bool synopGMAC_is_rx_desc_empty(s_rx_norm_desc const * rxdesc)
{
	bool retval;
	if(((rxdesc->RDES2 & 0x3F) == 0)  && ((rxdesc->RDES3 & 0x7FF) == 0)) {  /* Rx Write-back */
		retval = 1;
	}
	else {
		retval = 0;
	}
	return retval;
}
/**
  * Function to set the MDC clock for mdio transactiona
  *
  * @param[in] pointer to device structure.
  * @param[in] clk divider value.
  * \return Reuturns 0 on success else return the error value.
  */
CPU_INT32S synopGMAC_set_mdc_clk_div(synopGMACdevice const *gbl_mac_dev,CPU_INT32U clk_div_val)
{
	CPU_INT32U orig_data;
	orig_data = synopGMACReadReg((CPU_INT32U *)gbl_mac_dev->MacBase,GmacGmiiAddr); /* set the mdc clock to the user defined value */
	orig_data &= (~ (uint32_t)GmiiCsrClkMask);	   
	orig_data |= clk_div_val;
	synopGMACWriteReg((CPU_INT32U *)gbl_mac_dev->MacBase, GmacGmiiAddr ,orig_data);
	return 0;
}
/**
  * Returns the current MDC divider value programmed in the ip.
  *
  * @param[in] pointer to device structure.
  * @param[in] clk divider value.
  * \return Returns the MDC divider value read.
  */
CPU_INT32U synopGMAC_get_mdc_clk_div(synopGMACdevice const *gbl_mac_dev)
{
	CPU_INT32U uint_data;
	uint_data = synopGMACReadReg((CPU_INT32U *)gbl_mac_dev->MacBase,GmacGmiiAddr);
	uint_data &= GmiiCsrClkMask;
	return uint_data;
}
/**
  * Enable all the tx interrupts.
  * Enables the DMA interrupt as specified by the bit mask.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] bit mask of interrupts to be enabled.
  * \return returns void.
  */
void wrapperDMA_enable_tx_interrupt(synopGMACdevice const *gbl_mac_dev, CPU_INT32U interrupts, uint32_t txchno)
{
	CPU_INT32U uint_data;
	
	uint_data = synopGMACReadReg((CPU_INT32U *)(gbl_mac_dev->DmaBase), DmaTxChIntMsk00 + (txchno * DMA_CHANNEL_REG_LEN));
	uint_data |= (interrupts); /* enable interrupts bits */
	synopGMACWriteReg((CPU_INT32U *)(gbl_mac_dev->DmaBase), DmaTxChIntMsk00 + (txchno * DMA_CHANNEL_REG_LEN), uint_data);
	
	return;
}

/**
  * Enable all the tx interrupts.
  * Enables the DMA interrupt as specified by the bit mask.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] bit mask of interrupts to be enabled.
  * \return returns void.
  */
void wrapperDMA_enable_rx_interrupt(synopGMACdevice const *gbl_mac_dev, CPU_INT32U interrupts, CPU_INT08U dmachno)
{
	CPU_INT32U uint_data, regoffset = 0;
	
	regoffset = (uint32_t)DmaRxChIntMsk00 + (dmachno * DMA_CHANNEL_REG_LEN);
	
	uint_data = synopGMACReadReg((CPU_INT32U *)gbl_mac_dev->DmaBase, regoffset);
	uint_data |= (interrupts); /* enable the bits */
	synopGMACWriteReg((CPU_INT32U *)gbl_mac_dev->DmaBase, regoffset, uint_data);
	
	return;
}

/**
  * Function to read the GMAC IP Version and populates the same in device data structure.
  * @param[in] pointer to synopGMACdevice.
  * \return Always return 0.
  */

CPU_INT32S synopGMAC_read_version (synopGMACdevice * gbl_mac_dev) 
{	
	CPU_INT32U uint_data = 0;
	uint_data = synopGMACReadReg((CPU_INT32U *)gbl_mac_dev->MacBase, GmacVersion );
	gbl_mac_dev->Version = uint_data;

	return 0;
}

/**
  * Function to program DMA bus mode register. 
  * 
  * The Bus Mode register is programmed with the value given. The bits to be set are
  * bit wise or'ed and sent as the second argument to this function.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] the data to be programmed.
  * \return 0 on success else return the error status.
  */
CPU_INT32S synopGMAC_dma_bus_mode_init(synopGMACdevice const * gbl_mac_dev, CPU_INT32U init_value )
{
	synopGMACWriteReg((CPU_INT32U *)gbl_mac_dev->DmaBase, DmaBusCfg ,init_value);
	
	return 0;
}

/**
  * Function to reset the GMAC core. 
  * This reests the DMA and GMAC core. After reset all the registers holds their respective reset value
  * @param[in] pointer to synopGMACdevice.
  * \return 0 on success else return the error status.
  */
CPU_INT32S synopGMAC_reset (synopGMACdevice const * gbl_mac_dev ) 
{	
	CPU_INT32U uint_data = 0;
	
	uint_data = synopGMACReadReg((CPU_INT32U *)TC9560_REG_BASE, 0x1008);
	uint_data |= (1 << 7); /* Bit 7: MACRDT */
	synopGMACWriteReg((CPU_INT32U *)TC9560_REG_BASE, 0x1008, uint_data);
	
  plat_delay(DEFAULT_LOOP_VARIABLE);
	uint_data &= ~(1U << 7U); /* Bit 7: MACRDT */
	synopGMACWriteReg((CPU_INT32U *)TC9560_REG_BASE, 0x1008, uint_data);
	(void)*gbl_mac_dev;
	return 0;	
}

static void tc9560INTC_disable_rxtx_interrupt(CPU_INT32U channels)
{
	CPU_INT32U mask;

	mask = synopGMACReadReg((CPU_INT32U *)TC9560_INTC_REG_BASE, INTMCUMASK1_OFFS);
	mask |= channels;
	synopGMACWriteReg((CPU_INT32U *)TC9560_INTC_REG_BASE, INTMCUMASK1_OFFS, mask);
}

void tc9560INTC_enable_rxtx_interrupt(CPU_INT32U channels)
{
	CPU_INT32U mask;

	mask = synopGMACReadReg((CPU_INT32U *)TC9560_INTC_REG_BASE, INTMCUMASK1_OFFS);
	mask &= ~(channels);
	synopGMACWriteReg((CPU_INT32U *)TC9560_INTC_REG_BASE, INTMCUMASK1_OFFS, mask);
}

/**
  * Checks and initialze phy.
  * This function checks whether the phy initialization is complete. 
  * @param[in] pointer to synopGMACdevice.
  * \return 0 if success else returns the error number.
  */
CPU_INT32S synopGMAC_check_phy_init (synopGMACdevice * gbl_mac_dev) 
{
	CPU_INT32S retval = -ESYNOPGMACNOERR;
	uint32_t uint_data;
	CPU_INT32S int_status = -ESYNOPGMACNOERR;		
	CPU_INT32S loop_count;
	int32_t flag = 0;
	loop_count = DEFAULT_LOOP_VARIABLE;
	while(loop_count-- > 0)
	{
		int_status = read_phy_regs(gbl_mac_dev->PhyBase,PHY_STATUS_REG, &uint_data);
		if(int_status != 0) {
			retval = int_status;
			flag = 1;
		}else {
		  if((uint_data & Mii_AutoNegCmplt) != 0){
				break;
			}
		}
	}
	if (flag == 0){
		int_status = read_phy_regs(gbl_mac_dev->PhyBase,PHY_SPECIFIC_STATUS_REG, &uint_data);
		if(int_status != 0) {
			retval = int_status;
			flag = 1;
		} else {
		  if((uint_data & Mii_phy_status_link_up) == 0){
				gbl_mac_dev->LinkState = LINKDOWN; 
				retval = -ESYNOPGMACPHYERR;
				flag = 1;
			}
			else{
				gbl_mac_dev->LinkState = LINKUP; 
			}
				int_status = read_phy_regs(gbl_mac_dev->PhyBase,PHY_SPECIFIC_STATUS_REG, &uint_data);
				if(int_status != 0) {
					retval = int_status;
					flag = 1;
				}
		}
		if(flag == 0) {
			gbl_mac_dev->DuplexMode = ((uint_data & Mii_phy_status_full_duplex) != 0)  ? FULLDUPLEX: HALFDUPLEX ;
			/*if not set to Master configuration in case of Half duplex mode set it manually as Master*/
			if(gbl_mac_dev->DuplexMode == HALFDUPLEX){
				int_status = read_phy_regs(gbl_mac_dev->PhyBase,PHY_CONTROL_REG, &uint_data);
				if(int_status != 0) {
					retval = int_status;
					flag = 1;
				} else {
					int_status = write_phy_regs(gbl_mac_dev->PhyBase,PHY_CONTROL_REG, uint_data | Mii_Manual_Master_Config);
					if(int_status != 0) {
						retval = int_status;
						flag = 1;
					}
				}				
			}
		}
		if (flag == 0) {
			int_status = read_phy_regs(gbl_mac_dev->PhyBase,PHY_SPECIFIC_STATUS_REG, &uint_data);
			if(int_status != 0) {
				retval = int_status;
				flag = 1;
			} else {
				if((uint_data & Mii_phy_status_speed_1000) != 0) {
						gbl_mac_dev->Speed	  =   SPEED1000;
				}
				else if((uint_data & Mii_phy_status_speed_100) != 0) {
					gbl_mac_dev->Speed	  =   SPEED100;
				}
				else {
					gbl_mac_dev->Speed	  =   SPEED10;
				}
			}
		}
	}
	return retval;
}

/**
  * Prepares the descriptor to receive packets.
  * The descriptor is allocated with the valid buffer addresses (sk_buff address) and the length fields
  * and handed over to DMA by setting the ownership. After successful return from this function the
  * descriptor is added to the receive descriptor pool/queue.
  * This api is same for both ring mode and chain mode.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] Dma-able buffer1 pointer.
  * @param[in] length of buffer1 (Max is 2048).
  * @param[in] Dma-able buffer2 pointer.
  * @param[in] length of buffer2 (Max is 2048).
  * @param[in] u32 data indicating whether the descriptor is in ring mode or chain mode.
  * \return returns present rx descriptor index on success. Negative value if error.
  */
CPU_INT32U synopGMAC_set_rx_qptr(synopGMACdevice * gbl_mac_dev, CPU_INT32U Buffer)
{
	CPU_INT32U  rxnext	  = gbl_mac_dev->RxNext[gbl_mac_dev->rxchno];
	s_rx_norm_desc* rxdesc  = gbl_mac_dev->RxNextDesc[gbl_mac_dev->rxchno];
	rxdesc->RDES0 = Buffer;
	gbl_mac_dev->RDES0Val[gbl_mac_dev->rxchno][gbl_mac_dev->RxNext[gbl_mac_dev->rxchno]] = Buffer;
	rxdesc->RDES1 = 0;
	rxdesc->RDES2 = 0;
	rxdesc->RDES3 = DescRxIOC |((uint32_t)DescRxOWN) | (1U << 24);  /* BUF1V */

	if((rxnext+1) > (RX_DESC_CNT - 1)){
		gbl_mac_dev->RxNext[gbl_mac_dev->rxchno] = 0;
		gbl_mac_dev->RxNextDesc[gbl_mac_dev->rxchno] = gbl_mac_dev->RxDesc[gbl_mac_dev->rxchno];
	}else{
		gbl_mac_dev->RxNext[gbl_mac_dev->rxchno] = rxnext + 1;
		gbl_mac_dev->RxNextDesc[gbl_mac_dev->rxchno] = (rxdesc + 1);
	}
	(gbl_mac_dev->BusyRxDescNo[gbl_mac_dev->rxchno])++; /* One descriptor will be given to Hardware. So busy count incremented by one */
	return rxnext;
}

static void TC9560_RXCH_Handler(CPU_INT08U  dmachno )
{
	CPU_INT32U interrupt,i,j;
	CPU_INT32U ch_offset = 0;
		DBG_eMAC_Print(DBG_EMAC_RX,"TC9560_RXCH_Handler:%d\n",(int32_t)dmachno);
		DBG_Test_Print(DBG_TEST, "%s \n", __func__);
		gmacdev->rxchno = dmachno;
		ch_offset = (uint32_t)DmaRxChSts00 + (dmachno * DMA_CHANNEL_REG_LEN);
	
		interrupt = (uint32_t)synopGMACReadReg((CPU_INT32U*)gmacdev->DmaBase, ch_offset);
		synopGMACWriteReg((CPU_INT32U*)gmacdev->DmaBase, (uint32_t)ch_offset, interrupt); /* clear the interrupt */
		
		interrupt &= 0xFFF;	
		i = interrupt & (3 << 5);
		j = interrupt & (1 << 1);
		if(( interrupt == 0) || (i) || (j)){  /* no interrupt occued ??*/
		}
		else {
			NetDev_Rx ();	
		}
		return;			
}

static void TC9560_TXCH_Handler(CPU_INT08U  dmachno )
{
	CPU_INT32U interrupt;
	CPU_INT32U ch_offset = 0;
		DBG_eMAC_Print(DBG_EMAC_TX,"TC9560_TXCH_Handler:%d\n",(int32_t)dmachno);
		DBG_Test_Print(DBG_TEST, "%s \n", __func__);
		ch_offset = (uint32_t)DmaTxChSts00 + (dmachno * DMA_CHANNEL_REG_LEN);
		
		interrupt = synopGMACReadReg((CPU_INT32U*)gmacdev->DmaBase, ch_offset);
		synopGMACWriteReg((CPU_INT32U*)gmacdev->DmaBase, ch_offset, interrupt); /* clear the interrupt */

		interrupt &= 0xFFF;
		if( interrupt == 0 ) {  /* no interrupt occued ??*/
		}
		else {
			if((interrupt & (1 << 4)) != 0){ /* Bit 4: fatal bus error */
						
			}	 
			if((interrupt & ( 1<< 1)) != 0){
		
			}
			if((interrupt & (1 << 0)) != 0){  /* TX complete */
				synop_handle_transmit_over((int32_t)dmachno);/*Do whatever you want after the transmission is over */
			}
		}
		return;
}

static void  TC9560_TXCH0_Handler(void)/* GPTP */
{
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	TC9560_TXCH_Handler(0);
	*(unsigned int *)(TC9560_M3_DBG_CNT_START + 4*0) += 1;
}	

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
static void  TC9560_TXCH1_Handler(void)
{
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	TC9560_TXCH_Handler(1);
	*(unsigned int *)(TC9560_M3_DBG_CNT_START + 4*1) += 1;
}	
#endif
static void  TC9560_TXCH2_Handler(void)/* Legacy */
{
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	TC9560_TXCH_Handler(2);
	*(unsigned int *)(TC9560_M3_DBG_CNT_START + 4*2) += 1;
}	

static void  TC9560_TXCH3_Handler(void)
{
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	TC9560_TXCH_Handler(3);
	*(unsigned int *)(TC9560_M3_DBG_CNT_START + 4*3) += 1;
}	

static void  TC9560_TXCH4_Handler(void)
{
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	TC9560_TXCH_Handler(4);
	*(unsigned int *)(TC9560_M3_DBG_CNT_START + 4*4) += 1;
}	

static void  TC9560_RXCH0_Handler(void)
{
	DBG_Test_Print(DBG_TEST_IN_EP, "%s \n", __func__);
	TC9560_RXCH_Handler(0);
	*(unsigned int *)(TC9560_M3_DBG_CNT_START + 4*6) += 1;
}	

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
static void  TC9560_RXCH1_Handler(void)
{
	DBG_Test_Print(DBG_TEST_IN_EP, "%s \n", __func__);
	TC9560_RXCH_Handler(1);
	*(unsigned int *)(TC9560_M3_DBG_CNT_START + 4*7) += 1;
}
#endif

static void  TC9560_RXCH2_Handler(void)
{
	DBG_Test_Print(DBG_TEST_IN_EP, "%s \n", __func__);
	TC9560_RXCH_Handler(2);
	*(unsigned int *)(TC9560_M3_DBG_CNT_START + 4*8) += 1;
}	

static void  TC9560_RXCH3_Handler(void)/* GPTP */
{
	DBG_Test_Print(DBG_TEST_IN_EP, "%s \n", __func__);
	TC9560_RXCH_Handler(3);
	*(unsigned int *)(TC9560_M3_DBG_CNT_START + 4*9) += 1;
}	

static void  TC9560_RXCH4_Handler(void)
{
	DBG_Test_Print(DBG_TEST_IN_EP, "%s \n", __func__);
	TC9560_RXCH_Handler(4);
	*(unsigned int *)(TC9560_M3_DBG_CNT_START + 4*10) += 1;
}	

static void  TC9560_RXCH5_Handler(void)
{
	DBG_Test_Print(DBG_TEST_IN_EP, "%s \n", __func__);
	TC9560_RXCH_Handler(5);
	*(unsigned int *)(TC9560_M3_DBG_CNT_START + 4*11) += 1;
}	


static  void  NetDev_RxDescInit (CPU_INT08U chan)  
{
	s_rx_norm_desc	 *pdesc;

	CPU_INT32U		  nbytes;
	CPU_INT08U		  i;	
	CPU_INT08U 					last_index;
	CPU_INT32U		   dma_buf;

	nbytes		= (RxDescNbr * sizeof(s_rx_norm_desc));
	
	pdesc	 = DWC_ALLOC(nbytes);

	/* --------------- INIT DESCRIPTOR PTRS  -------------- */
	gmacdev->RxDescCount[chan] = RX_DESC_CNT;
	gmacdev->RxDesc[chan]	  = pdesc;

	gmacdev->RxNext[chan] = 0;
	gmacdev->RxBusy[chan] = 0;
	gmacdev->RxNextDesc[chan] = gmacdev->RxDesc[chan];
	gmacdev->RxBusyDesc[chan] = gmacdev->RxDesc[chan];

	gmacdev->BusyRxDescNo[chan] = 0; 

	/* --------------- INIT RX DESCRIPTORS ---------------- */
	memset(pdesc, 0 ,sizeof(s_rx_norm_desc) * RxDescNbr);
	
	for (i = 0u; i < RxDescNbr; i++) {
		if (chan == 3) {  /* GPTP channel */
			dma_buf = (CPU_INT32U)DWC_ALLOC((DEV_PTP_SIZE));
		}
		else {
			dma_buf = (CPU_INT32U)DWC_ALLOC((DEV_MTU_SIZE));
		}

		gmacdev->RDES0Val[chan][i] =  dma_buf;  /* save the poiter value */

		/* update buffer 1 address pointer */
		RX_NORMAL_DESC_RDES0_Ml_Wr(pdesc->RDES0, dma_buf);
		/* set to zero  */
		RX_NORMAL_DESC_RDES1_Ml_Wr(pdesc->RDES1, 0);

		/* set buffer 2 address pointer to zero */
		RX_NORMAL_DESC_RDES2_Ml_Wr(pdesc->RDES2, 0);
		/* set control bits - OWN, INTE and BUF1V */

		RX_NORMAL_DESC_RDES3_Ml_Wr(pdesc->RDES3, (0xC1000000U));
		
		pdesc++;
	}
	pdesc--;
	
	/* update the total no of Rx descriptors count */		
	last_index = GET_CURRENT_RCVD_LAST_DESC_INDEX(0, 0); /* start_index = 0, offset = 0 */

	synopGMACWriteReg((CPU_INT32U *)(gmacdev->DmaBase), (uint32_t)DmaRxChControl00   + (chan * DMA_CHANNEL_REG_LEN), 0x00101000); /* RBSZ */
	
	/* update the total no of Rx descriptors count */
	synopGMACWriteReg((CPU_INT32U *)(gmacdev->DmaBase), (uint32_t)DmaRxChDescRngLn00 + (chan * DMA_CHANNEL_REG_LEN), (RX_DESC_CNT - 1));
	/* update the starting address of desc chain/ring */
	synopGMACWriteReg((CPU_INT32U *)(gmacdev->DmaBase), (uint32_t)DmaRxChDescLstLA00 + (chan * DMA_CHANNEL_REG_LEN), GET_RX_DESC_DMA_ADDR(chan, 0));
	last_index = GET_CURRENT_RCVD_LAST_DESC_INDEX(0, 0); /* start_index = 0, offset = 0 */
	synopGMACWriteReg((CPU_INT32U *)(gmacdev->DmaBase), (uint32_t)DmaRxChDescTailPt00 + (chan * DMA_CHANNEL_REG_LEN), GET_RX_DESC_DMA_ADDR(chan, last_index));
	
	return;
}

static  void  NetDev_TxDescInit (CPU_INT08U chan)
{
	s_tx_norm_desc	 *pdesc;
	uint32_t		nbytes;
	CPU_INT16U		  i;
													  /* -- OBTAIN REFERENCE TO DEVICE CFG/DATA/REGISTERS --- */
	nbytes	= (TxDescNbr * sizeof(s_tx_norm_desc));
	
		pdesc = DWC_ALLOC(nbytes);
	
		gmacdev->TxDescCount[chan] = TX_DESC_CNT;
		gmacdev->TxDesc[chan]	  = pdesc;
		gmacdev->TxNext[chan]	  = 0;
		gmacdev->TxBusy[chan]	  = 0;
		gmacdev->TxNextDesc[chan]  = gmacdev->TxDesc[chan];
		gmacdev->TxBusyDesc[chan]  = gmacdev->TxDesc[chan];
		gmacdev->BusyTxDescNo[chan]  = 0;
	 
		/* --------------- INIT TX DESCRIPTORS ---------------- */
		memset(pdesc, 0, sizeof(s_tx_norm_desc) * TxDescNbr);
		for (i = 0u; i < TxDescNbr; i++) {				/* Initialize Tx desc ring.							 */		
				gmacdev->TDES0Val[chan][i] = 0;

			pdesc->TDES0 = 0;
				/* update buffer 1 address pointer to zero */
				TX_NORMAL_DESC_TDES0_Ml_Wr(pdesc->TDES0, 0);
				/* update buffer 2 address pointer to zero */
				TX_NORMAL_DESC_TDES1_Ml_Wr(pdesc->TDES1, 0);
				/* set all other control bits (IC, TTSE, B2L & B1L) to zero */
				TX_NORMAL_DESC_TDES2_Ml_Wr(pdesc->TDES2, 0x40000000);
				/* set all other control bits (OWN, CTXT, FD, LD, CPC, CIC etc) to zero */
				TX_NORMAL_DESC_TDES3_Ml_Wr(pdesc->TDES3, 0);
				pdesc++;
		}
						 
		synopGMACWriteReg((CPU_INT32U *)(gmacdev->DmaBase), (uint32_t)DmaTxChDescRngLn00 + (chan * DMA_CHANNEL_REG_LEN), (TX_DESC_CNT - 1));
		synopGMACWriteReg((CPU_INT32U *)(gmacdev->DmaBase), (uint32_t)DmaTxChDescLstLA00 + (chan * DMA_CHANNEL_REG_LEN), GET_TX_DESC_DMA_ADDR(chan, 0)); 
}

static  void  NetDev_Init(void)
{
	CPU_INT32U uint_status;
	CPU_INT08U i = 0;
	CPU_INT08U qInx;
	
	/* Allocate the device data structure */
	gmacdev = DWC_ALLOC(sizeof(synopGMACdevice));
	
	gmacdev->MacBase = BaseAddr;
	gmacdev->PhyBase = PhyBaseAddr;
	gmacdev->DmaBase = DMABaseAddr;
	
	synopGMAC_reset(gmacdev);
	
	/*Lets read the version of ip in to device structure*/	
	synopGMAC_read_version(gmacdev);
	
	DWC_ETH_QOS_get_all_hw_features(gmacdev);

	DWC_ETH_QOS_mmc_setup(gmacdev);

	/*Check for Phy initialization*/
	synopGMAC_set_mdc_clk_div(gmacdev,GmiiCsrClk1);  /* For TC9560 FPGA! GmiiCsrClk2 --> GmiiCsrClk1 */
	gmacdev->ClockDivMdc = synopGMAC_get_mdc_clk_div(gmacdev);

	uint_status = (uint32_t)synopGMAC_check_phy_init(gmacdev);
	if (gmacdev->LinkState == LINKDOWN)	
	{
		CPU_INT16U			reg_val;
        CPU_INT16U          reg_val_temp;
		CPU_INT16U			retries;
        
        /* Rd ctrl reg, get reset bit.						  */
		NetDev_MII_Rd(gmacdev->PhyBase, PHY_CONTROL_REG, &reg_val_temp);
		NetDev_MII_Wr(gmacdev->PhyBase, PHY_CONTROL_REG, reg_val_temp|Mii_reset);
 
		/* Rd ctrl reg, get reset bit.						  */
		NetDev_MII_Rd(gmacdev->PhyBase, PHY_CONTROL_REG, &reg_val);

		reg_val &= Mii_reset;									  /* Mask out reset status bit.						   */
		retries = 5;
		while ((reg_val == Mii_reset) && (retries > 0u)) {		 /* Wait for reset to complete.						  */
			NetDev_MII_Rd(gmacdev->PhyBase, PHY_CONTROL_REG, &reg_val);

			reg_val &= Mii_reset;
			retries--;
		}
        /* If link is down check if the Master/ Slave configuration has been set */
	}

	(void)uint_status;

	for(i = 0 ; i < RX_NO_CHAN; i++){  /*  2 RX Ch	*/	
		/* set rx descriptor ring length */
		gmacdev->rxchno = i;
		synopGMAC_set_rx_desc_rng_len(gmacdev, RxDescNbr-1, i);
	} /* for RX_NO_CHAN */

	for(i = 0 ; i < TX_NO_CHAN ; i++)
	{  /* TX Ch */
		/* update the total no of Tx descriptors count */
	  gmacdev->txchno = i;
		synopGMAC_set_tx_desc_rng_len(gmacdev, TxDescNbr-1, i);
	}

  /* Toshiba DMA Wrapper	*/
	synopGMACWriteReg((CPU_INT32U *)gmacdev->DmaBase, EVB_CTRL,  0x00400000);  /* MAC[1] filter --> RXCH1, MAC[2]-->RXCH2 */
		
	config_phy_autoneg(0,0);
	
	/* MAC registers */
	configure_mac();  /*done in Start */
		
	/* reset mmc counters */
	MMC_CNTRL_RgWr(0x5);
	
	for (qInx = 0; qInx < M3_MAX_QUEUE; qInx++) {  /* INIT Queue */
		configure_mtl_queue(qInx);
	}
 	
	for(i = 0 ; i < RX_NO_CHAN; i++){
		MTL_QROMR_RSF_UdfWr(i, 0x1);  
	}

	/* disable tx drop status */
	MTL_OMR_DTXSTS_UdfWr(0x0);	

  /* Toshiba DMA Wrapper	*/
	synopGMAC_dma_bus_mode_init(gmacdev, 0xe);	/* Setting INCRx to set burst length to 16 */
	
	/*Initialize the mac interface*/
	synopGMAC_mac_init(gmacdev);
	
	config_phy_autoneg(1,0);
	
	/* For AVB */
  synopGMAC_frame_filter_enable();
	synopGMAC_hash_perfect_filter_enable();
	MAC_MPFR_PM_UdfWr(1);
	MAC_RQC0R_RgWr(0x56);
	MAC_RQC1R_RgWr(0x12);
	MAC_RQC2R_RgWr(0x0C000000);
	
	MAC_TCR_TSENA_UdfWr(1);
	MAC_TCR_TSVER2ENA_UdfWr(1U);
	MAC_TCR_TSIPENA_UdfWr(1U);
	MAC_TCR_TSIPV4ENA_UdfWr(0U);
	init_systime(0x1234, 0x56789);
	
	MAC_VLANTR_EVLS_UdfWr(0x00);

	set_dcb_algorithm(0x3);
	
	global_isr_table[11] = TC9560_TXCH0_Handler;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	global_isr_table[12] = TC9560_TXCH1_Handler;
#endif
	global_isr_table[13] = TC9560_TXCH2_Handler;
	global_isr_table[14] = TC9560_TXCH3_Handler;
	global_isr_table[15] = TC9560_TXCH4_Handler;
	global_isr_table[16] = TC9560_RXCH0_Handler;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	global_isr_table[17] = TC9560_RXCH1_Handler;
#endif
	global_isr_table[18] = TC9560_RXCH2_Handler;
	global_isr_table[19] = TC9560_RXCH3_Handler;
	global_isr_table[20] = TC9560_RXCH4_Handler;
	global_isr_table[21] = TC9560_RXCH5_Handler;
}

void  NetDev_Start (void)
{ 
	CPU_INT08U		  hw_addr[6];
	CPU_INT08U		  chno;
	CPU_BOOLEAN		 hw_addr_cfg;
		CPU_INT32U 					uint_data;

		hw_addr[0] =  0xE8;
		hw_addr[1] =  0xE0;
		hw_addr[2] =  0xB7;
		hw_addr[3] =  0xB5;
		hw_addr[4] =  0x7D;
		hw_addr[5] =  0xF8;

	hw_addr_cfg = DEF_YES;									   /* See Notes #4 & #5.								   */
	
	if (hw_addr_cfg == DEF_YES) {							   /* If necessary, set dev HW MAC addr.				   */
			synopGMAC_set_mac_addr(hw_addr);  /* TC9560_TBD!!!: When HSIC host is running, M3 should use GmacAddr1.... */
	}
		
		for (chno = 0; chno < RX_NO_CHAN; chno++) {
			if (chno != 1) {   /* CH 1 is not used!! */
				NetDev_RxDescInit(chno);					   /* Init Rx desc's. for chno							*/
			}
		}

	for (chno = 0; chno < TX_NO_CHAN; chno++) 		
	{
		if (chno != 1) {   /* CH 1 is not used!! */
			NetDev_TxDescInit(chno);						/* Init Tx desc's.									  */
		}
	}
			 
	for (chno = 0; chno < RX_NO_CHAN; chno++) {
		wrapperDMA_enable_rx_interrupt(gmacdev, DmaIntRxAll, chno);	 
		uint_data = synopGMACReadReg((CPU_INT32U *)(gmacdev->DmaBase), (uint32_t)DmaRxChControl00 + (chno * DMA_CHANNEL_REG_LEN));
		synopGMACWriteReg((CPU_INT32U *)(gmacdev->DmaBase), (uint32_t)DmaRxChControl00 + (chno * DMA_CHANNEL_REG_LEN), uint_data | 1);
	}
	for (chno = 0; chno < TX_NO_CHAN; chno++) {
		wrapperDMA_enable_tx_interrupt(gmacdev, DmaIntTxAll, chno);	 
		uint_data = synopGMACReadReg((CPU_INT32U *)(gmacdev->DmaBase), (uint32_t)DmaTxChControl00 + (chno * DMA_CHANNEL_REG_LEN));
		synopGMACWriteReg((CPU_INT32U *)(gmacdev->DmaBase), (uint32_t)DmaTxChControl00 + (chno * DMA_CHANNEL_REG_LEN), uint_data | 1);		
	}
	
#if 1 /* For debugging timestamp comp */
	/*synopGMACWriteReg((CPU_INT32U *)gmacdev->DmaBase, ,  );	*/
	synopGMACWriteReg((CPU_INT32U *)gmacdev->MacBase, 0xab00,  0x10057E01);	
	synopGMACWriteReg((CPU_INT32U *)gmacdev->MacBase, 0xab04,  0x00140000);	
	synopGMACWriteReg((CPU_INT32U *)gmacdev->MacBase, 0xab10,  0x56663db2);	
	synopGMACWriteReg((CPU_INT32U *)gmacdev->MacBase, 0xab14,  0x1b0d6949);	
	synopGMACWriteReg((CPU_INT32U *)gmacdev->MacBase, 0xab18,  0x66666666);	
#endif
}

static  void  NetDev_Rx(void)
{
	static uint8_t EDMA2BULKIN[ ] = {
	0,
	0,
	0,
	3,
	2, /* BHINX_IN2, */
	1, /* BHINX_IN1 */
	};
	/* Need to pass below two parameters in this function. */
	CPU_INT08U  *p_data = NULL;

	CPU_INT08U *prxbuf_new;
	CPU_INT32S desc_index;
	CPU_INT32U rx_len;
	CPU_INT32U dma_addr1;
	CPU_INT32U uint_status[4] = {0};
	CPU_BOOLEAN RxError = 0;
	CPU_INT32U j,k;
	CPU_INT08U chno;
	int32_t retval;
	CPU_INT32S tail_index;
	int32_t idx;
	int32_t update_tail_ptr = 0;
	chno = gmacdev->rxchno;
	
	tc9560INTC_disable_rxtx_interrupt(0x1U << (chno + 16));/* INTC_Mask_RXCHS); */
	
	NetDev_RX_callback[EDMA2BULKIN[chno]] = 0;
	DBG_Test_Print(DBG_TEST_IN_EP, "%s \n", __func__);
	for(idx=0; idx<(RX_DESC_CNT/2); idx++)
	{	
		desc_index = synopGMAC_get_rx_qptr(gmacdev, &uint_status[0], &dma_addr1, &rx_len, chno);
		if (desc_index < 0) 
		{
			/* SW doesn't own the next descriptor */
			goto exit_NetDev_Rx;
	}
	j = uint_status[3] & (1<<30);
	/* Invalid frame, ignore it and free buffer.			*/
	if  ( ( (uint_status[3] & DescRxPL) > DEV_MTU_SIZE )  || ( rx_len <= 0  ) || (j) )
	{
		k = (uint_status[3] & (1<<30));
			/* Drop invalid  packet*/
			if  (!k)
			{
				DBG_Warn_Print("Droping invalid  packet, Size = %d, %x\n", rx_len, (uint_status[3] & DescRxCTXT) );
			}
			/* Reuse the buffer and recycle the descriptor */
		prxbuf_new = (CPU_INT08U *)dma_addr1;
			update_tail_ptr = 1;
			goto set_desc_again;
	}
	/* Check for receive errors */
		RxError = 0;
		if((uint_status[1] & DescRxIPCE) != 0){	RxError = 1; }
		if((uint_status[1] & DescRxIPHE) != 0){	RxError = 1; }
		if((uint_status[3] & DescRxCE) != 0)  {	RxError = 1; }
		if((uint_status[3] & DescRxRWDT) != 0){	RxError = 1; }
		if((uint_status[3] & DescRxOE) != 0)  {	RxError = 1; }
		if((uint_status[3] & DescRxDE) != 0)  {	RxError = 1; }
	
		if ( RxError != 0 ) 
		{
			DBG_Warn_Print("ZZ: Rx Error %x, %x\n", uint_status[1], uint_status[3]);
			/* Reuse the buffer and recycle the descriptor */
			prxbuf_new = (CPU_INT08U *)dma_addr1;
			update_tail_ptr = 1;
			goto set_desc_again;
	}
	p_data	   = (CPU_INT08U *)dma_addr1;					 /* Return a ptr to the newly Rx'd data area.			*/
	if(dma_addr1 != 0)
	{	
		retval = neu_ep_queue_in( (uint8_t *)p_data, EDMA2BULKIN[chno], rx_len);	
			
		if(retval < 0)
		{
			DBG_Warn_Print("NetDev_Rx: HSIC Packet submission error, Dropping RX packet, DMA CH = %d ret = %d\n", chno,retval);
			/* HSIC Can't consume this packet, so drop it */
			/* Reuse the buffer and recycle the descriptor */
			prxbuf_new = (CPU_INT08U *)dma_addr1;
			update_tail_ptr = 1;
			goto set_desc_again;
		}	

		/* old buffer submitted to HSIC, get new buffer */	
		if (chno == 3) 
		{   /* GPTP */
			prxbuf_new = ((CPU_INT08U *)DWC_ALLOC(DEV_PTP_SIZE));
		} 
		else 
		{
			prxbuf_new = ((CPU_INT08U *)DWC_ALLOC(DEV_MTU_SIZE));	
		}

		if(!prxbuf_new)
		{	
			DBG_Warn_Print("NetDev_Rx: prxbuf_new cann't be null, Error\n");
			goto exit_NetDev_Rx;
		}
		malloc_free_counter[EDMA2BULKIN[chno]] ++;
			DBG_eMAC_Print(DBG_EMAC_RX,"Malloc count value: %d,%d,%d,%d\n", malloc_free_counter[0], malloc_free_counter[1], malloc_free_counter[2], malloc_free_counter[3]);
		update_tail_ptr = 1;
		goto set_desc_again;
	}
	else
	{
		DBG_Warn_Print("NetDev_Rx: dma_addr1 cann't be NULL, Error\n");
		goto exit_NetDev_Rx;
	}
		
set_desc_again:
		synopGMAC_set_rx_qptr(gmacdev,(CPU_INT32U)prxbuf_new);
	}	
	
exit_NetDev_Rx:
	if(update_tail_ptr != 0)
	{
		/* At least one descriptor is consumed, advance the tail pointer */	
		tail_index = (int32_t)gmacdev->RxNext[chno];
		DECR_RX_DESC_INDEX(tail_index);
		synopGMACWriteReg((CPU_INT32U *)(gmacdev->DmaBase), ((uint32_t)DmaRxChDescTailPt00 + (chno * DMA_CHANNEL_REG_LEN)), GET_RX_DESC_DMA_ADDR(chno, tail_index));	
	}
	if(malloc_free_counter[EDMA2BULKIN[chno]] >= IN_REQ_NUMBER)
	{
		NetDev_RX_callback[EDMA2BULKIN[chno]] = 1;
	}
	else
	{
	tc9560INTC_enable_rxtx_interrupt(0x1U << (chno + 16));
	}
	return;
}

static  void  NetDev_Tx (CPU_INT08U const *p_data, CPU_INT32U size, CPU_INT08U txchno,struct usb_request *req)
{
	CPU_INT32S 					int_status = 0;
	CPU_INT32S  				last_index;
	uint32_t zz;
	static uint32_t last_zz = 0;
	
	gmacdev->txchno = txchno;
	
	int_status = synopGMAC_set_tx_qptr(gmacdev, (CPU_INT32U)p_data, size, 0, txchno, req);

	if(int_status < 0){
		DBG_eMAC_Print(DBG_EMAC_TX,"ZZ: Tx not free %x\n", int_status);

	}	
	else{
		if (txchno == 3) {
			zz = *(uint32_t *)((int32_t)p_data + 0x14) & 0xFFU;
			if ( ((last_zz+1U) & 0xFFU) != zz ) {
			}
			last_zz = (zz & 0xFF);
			
		}
		last_index = (int32_t)gmacdev->TxNext[txchno];

		/* DECR_TX_DESC_INDEX(last_index); */
		synopGMACWriteReg((CPU_INT32U *)(gmacdev->DmaBase), ((uint32_t)DmaTxChDescTailPt00 + (txchno * DMA_CHANNEL_REG_LEN)), GET_TX_DESC_DMA_ADDR(txchno, last_index));	
	}
	return;
}
/**
 * Function to handle housekeeping after a packet is transmitted over the wire.
 * After the transmission of a packet DMA generates corresponding interrupt 
 * (if it is enabled). It takes care of returning the sk_buff to the linux
 * kernel, updating the networking statistics and tracking the descriptors.
 * @param[in] pointer to net_device structure. 
 * \return void.
 * \note This function runs in interrupt context
 */
void synop_handle_transmit_over(int32_t txdmach)
{
	CPU_INT32S desc_index;
	CPU_INT32U uint_status;
	struct usb_request *usb_req;
	struct neu_buffhd *bh;
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	do{
		desc_index = synopGMAC_get_tx_qptr(gmacdev, &uint_status, (uint32_t)txdmach);
		if(desc_index >= 0){
		if(txdmach < TX_NO_CHAN){
			usb_req = (struct usb_request *)gmacdev->USBREQ[txdmach][desc_index];
			bh = (struct neu_buffhd *)usb_req->context;
			bh->buf = (void *)gmacdev->TDES0Val[txdmach][desc_index];
			DBG_eMAC_Print(DBG_EMAC_TX,"zz: O4C %d buf=%p, newBuf=%p\n",usb_req->req_num, bh->zoutreq[usb_req->req_num]->buf,bh->buf);
			bh->zoutreq[usb_req->req_num]->buf  = bh->buf;
			bh->zoutreq[usb_req->req_num]->req_num = usb_req->req_num; 

			neu_ep_queue_out(usb_req->req_num, "ep1out", usb_req->bulk_out_num+1);			
        }
			
			if(synopGMAC_tx_has_error(uint_status) != 0){  /* status = TDES3 */
				/*adapter->synopGMACNetStats.collisions += synopGMAC_get_tx_collision_count(status); */
			}
		}	
	} while(desc_index >= 0);
}





void  NetDev_MII_Rd (CPU_INT08U   phy_addr,
							 CPU_INT08U   reg_addr,
							 CPU_INT16U  *p_data)
{
	read_phy_regs((CPU_INT32U)phy_addr,(CPU_INT32U)reg_addr, (uint32_t *)p_data);
	/* *perr = NET_PHY_ERR_NONE; */
}


void  NetDev_MII_Wr (CPU_INT08U   phy_addr,
							 CPU_INT08U   reg_addr,
							 CPU_INT16U   data)
{
	write_phy_regs((CPU_INT32U)phy_addr,(CPU_INT32U)reg_addr, (uint32_t)data);
/*	*perr = NET_PHY_ERR_NONE; */
}

/*!
* \brief API to get all hw features.
*
* \details This function is used to check what are all the different
* features the device supports.
*
* \param[in] gmacdev - pointer to driver private structure
*
* \return none
*/
void DWC_ETH_QOS_get_all_hw_features(synopGMACdevice *gbl_mac_dev)
{
	uint32_t varMAC_HFR0;
	uint32_t varMAC_HFR1;
	uint32_t varMAC_HFR2;

	DBG_eMAC_Print(DBG_EMAC,"-->DWC_ETH_QOS_get_all_hw_features\n");

	MAC_HFR0_RgRd(varMAC_HFR0);
	MAC_HFR1_RgRd(varMAC_HFR1);
	MAC_HFR2_RgRd(varMAC_HFR2);

	memset(&gbl_mac_dev->hw_feat, 0, sizeof(gbl_mac_dev->hw_feat));
	gbl_mac_dev->hw_feat.mii_sel = ((varMAC_HFR0 >> 0) & MAC_HFR0_MIISEL_Mask);
	gbl_mac_dev->hw_feat.gmii_sel = ((varMAC_HFR0 >> 1) & MAC_HFR0_GMIISEL_Mask);
	gbl_mac_dev->hw_feat.hd_sel = ((varMAC_HFR0 >> 2) & MAC_HFR0_HDSEL_Mask);
	gbl_mac_dev->hw_feat.pcs_sel = ((varMAC_HFR0 >> 3) & MAC_HFR0_PCSSEL_Mask);
	gbl_mac_dev->hw_feat.vlan_hash_en =
		((varMAC_HFR0 >> 4) & MAC_HFR0_VLANHASEL_Mask);
	gbl_mac_dev->hw_feat.sma_sel = ((varMAC_HFR0 >> 5) & MAC_HFR0_SMASEL_Mask);
	gbl_mac_dev->hw_feat.rwk_sel = ((varMAC_HFR0 >> 6) & MAC_HFR0_RWKSEL_Mask);
	gbl_mac_dev->hw_feat.mgk_sel = ((varMAC_HFR0 >> 7) & MAC_HFR0_MGKSEL_Mask);
	gbl_mac_dev->hw_feat.mmc_sel = ((varMAC_HFR0 >> 8) & MAC_HFR0_MMCSEL_Mask);
	gbl_mac_dev->hw_feat.arp_offld_en =
		((varMAC_HFR0 >> 9) & MAC_HFR0_ARPOFFLDEN_Mask);
	gbl_mac_dev->hw_feat.ts_sel =
		((varMAC_HFR0 >> 12) & MAC_HFR0_TSSSEL_Mask);
	gbl_mac_dev->hw_feat.eee_sel = ((varMAC_HFR0 >> 13) & MAC_HFR0_EEESEL_Mask);
	gbl_mac_dev->hw_feat.tx_coe_sel =
		((varMAC_HFR0 >> 14) & MAC_HFR0_TXCOESEL_Mask);
	gbl_mac_dev->hw_feat.rx_coe_sel =
		((varMAC_HFR0 >> 16) & MAC_HFR0_RXCOE_Mask);
	gbl_mac_dev->hw_feat.mac_addr16_sel =
		((varMAC_HFR0 >> 18) & MAC_HFR0_ADDMACADRSEL_Mask);
	gbl_mac_dev->hw_feat.mac_addr32_sel =
		((varMAC_HFR0 >> 23) & MAC_HFR0_MACADR32SEL_Mask);
	gbl_mac_dev->hw_feat.mac_addr64_sel =
		((varMAC_HFR0 >> 24) & MAC_HFR0_MACADR64SEL_Mask);
	gbl_mac_dev->hw_feat.tsstssel =
		((varMAC_HFR0 >> 25) & MAC_HFR0_TSINTSEL_Mask);
	gbl_mac_dev->hw_feat.sa_vlan_ins =
		((varMAC_HFR0 >> 27) & MAC_HFR0_SAVLANINS_Mask);
	gbl_mac_dev->hw_feat.act_phy_sel =
		((varMAC_HFR0 >> 28) & MAC_HFR0_ACTPHYSEL_Mask);

	gbl_mac_dev->hw_feat.rx_fifo_size =
		((varMAC_HFR1 >> 0) & MAC_HFR1_RXFIFOSIZE_Mask);
		/*8; */
	gbl_mac_dev->hw_feat.tx_fifo_size =
		((varMAC_HFR1 >> 6) & MAC_HFR1_TXFIFOSIZE_Mask);
		/*8; */
	gbl_mac_dev->hw_feat.adv_ts_hword =
		((varMAC_HFR1 >> 13) & MAC_HFR1_ADVTHWORD_Mask);
	gbl_mac_dev->hw_feat.dcb_en = ((varMAC_HFR1 >> 16) & MAC_HFR1_DCBEN_Mask);
	gbl_mac_dev->hw_feat.sph_en = ((varMAC_HFR1 >> 17) & MAC_HFR1_SPHEN_Mask);
	gbl_mac_dev->hw_feat.tso_en = ((varMAC_HFR1 >> 18) & MAC_HFR1_TSOEN_Mask);
	gbl_mac_dev->hw_feat.dma_debug_gen =
		((varMAC_HFR1 >> 19) & MAC_HFR1_DMADEBUGEN_Mask);
	gbl_mac_dev->hw_feat.av_sel = ((varMAC_HFR1 >> 20) & MAC_HFR1_AVSEL_Mask);
	gbl_mac_dev->hw_feat.lp_mode_en =
		((varMAC_HFR1 >> 23) & MAC_HFR1_LPMODEEN_Mask);
	gbl_mac_dev->hw_feat.hash_tbl_sz =
		((varMAC_HFR1 >> 24) & MAC_HFR1_HASHTBLSZ_Mask);
	gbl_mac_dev->hw_feat.l3l4_filter_num =
		((varMAC_HFR1 >> 27) & MAC_HFR1_L3L4FILTERNUM_Mask);

	gbl_mac_dev->hw_feat.rx_q_cnt = ((varMAC_HFR2 >> 0) & MAC_HFR2_RXQCNT_Mask);
	gbl_mac_dev->hw_feat.tx_q_cnt = ((varMAC_HFR2 >> 6) & MAC_HFR2_TXQCNT_Mask);
	gbl_mac_dev->hw_feat.rx_ch_cnt =
		((varMAC_HFR2 >> 12) & MAC_HFR2_RXCHCNT_Mask);
	gbl_mac_dev->hw_feat.tx_ch_cnt =
		((varMAC_HFR2 >> 18) & MAC_HFR2_TXCHCNT_Mask);
	gbl_mac_dev->hw_feat.pps_out_num =
		((varMAC_HFR2 >> 24) & MAC_HFR2_PPSOUTNUM_Mask);
	gbl_mac_dev->hw_feat.aux_snap_num =
		((varMAC_HFR2 >> 28) & MAC_HFR2_AUXSNAPNUM_Mask);

	DBG_eMAC_Print(DBG_EMAC,"<--DWC_ETH_QOS_get_all_hw_features\n");
}

#if 1
/*!
 * \details This function is invoked by open() function. This function will
 * clear MMC structure.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

void DWC_ETH_QOS_mmc_setup(synopGMACdevice *gbl_mac_dev)
{
	DBG_eMAC_Print(DBG_EMAC,"-->DWC_ETH_QOS_mmc_setup\n");

	if (gbl_mac_dev->hw_feat.mmc_sel != 0) {
		memset(&gbl_mac_dev->mmc, 0, sizeof(struct DWC_ETH_QOS_mmc_counters));
	} 
		DBG_eMAC_Print(DBG_EMAC,"No MMC/RMON module available in the HW\n");

	DBG_eMAC_Print(DBG_EMAC,"<--DWC_ETH_QOS_mmc_setup\n");
}

/*!
 * \details This function is invoked by ethtool function when user wants to
 * read MMC counters. This function will read the MMC if supported by core
 * and store it in DWC_ETH_QOS_mmc_counters structure. By default all the
 * MMC are programmed "read on reset" hence all the fields of the
 * DWC_ETH_QOS_mmc_counters are incremented.
 *
 * open() function. This function will
 * initialize MMC control register ie it disable all MMC interrupt and all
 * MMC register are configured to clear on read.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

static uint32_t DWC_ETH_QOS_reg_read(volatile ULONG *ptr)
{
		return ioread32((void *)ptr);
}

void DWC_ETH_QOS_mmc_read(struct DWC_ETH_QOS_mmc_counters *mmc)
{
	DBG_eMAC_Print(DBG_EMAC,"-->DWC_ETH_QOS_mmc_read\n");

	/* MMC TX counter registers */
	mmc->mmc_tx_octetcount_gb += DWC_ETH_QOS_reg_read(MMC_TXOCTETCOUNT_GB_RgOffAddr);
	mmc->mmc_tx_framecount_gb += DWC_ETH_QOS_reg_read(MMC_TXPACKETCOUNT_GB_RgOffAddr);
	mmc->mmc_tx_broadcastframe_g += DWC_ETH_QOS_reg_read(MMC_TXBROADCASTPACKETS_G_RgOffAddr);
	mmc->mmc_tx_multicastframe_g += DWC_ETH_QOS_reg_read(MMC_TXMULTICASTPACKETS_G_RgOffAddr);
	mmc->mmc_tx_64_octets_gb += DWC_ETH_QOS_reg_read(MMC_TX64OCTETS_GB_RgOffAddr);
	mmc->mmc_tx_65_to_127_octets_gb += DWC_ETH_QOS_reg_read(MMC_TX65TO127OCTETS_GB_RgOffAddr);
	mmc->mmc_tx_128_to_255_octets_gb += DWC_ETH_QOS_reg_read(MMC_TX128TO255OCTETS_GB_RgOffAddr);
	mmc->mmc_tx_256_to_511_octets_gb += DWC_ETH_QOS_reg_read(MMC_TX256TO511OCTETS_GB_RgOffAddr);
	mmc->mmc_tx_512_to_1023_octets_gb += DWC_ETH_QOS_reg_read(MMC_TX512TO1023OCTETS_GB_RgOffAddr);
	mmc->mmc_tx_1024_to_max_octets_gb += DWC_ETH_QOS_reg_read(MMC_TX1024TOMAXOCTETS_GB_RgOffAddr);
	mmc->mmc_tx_unicast_gb += DWC_ETH_QOS_reg_read(MMC_TXUNICASTPACKETS_GB_RgOffAddr);
	mmc->mmc_tx_multicast_gb += DWC_ETH_QOS_reg_read(MMC_TXMULTICASTPACKETS_GB_RgOffAddr);
	mmc->mmc_tx_broadcast_gb += DWC_ETH_QOS_reg_read(MMC_TXBROADCASTPACKETS_GB_RgOffAddr);
	mmc->mmc_tx_underflow_error += DWC_ETH_QOS_reg_read(MMC_TXUNDERFLOWERROR_RgOffAddr);
	mmc->mmc_tx_singlecol_g += DWC_ETH_QOS_reg_read(MMC_TXSINGLECOL_G_RgOffAddr);
	mmc->mmc_tx_multicol_g += DWC_ETH_QOS_reg_read(MMC_TXMULTICOL_G_RgOffAddr);
	mmc->mmc_tx_deferred += DWC_ETH_QOS_reg_read(MMC_TXDEFERRED_RgOffAddr);
	mmc->mmc_tx_latecol += DWC_ETH_QOS_reg_read(MMC_TXLATECOL_RgOffAddr);
	mmc->mmc_tx_exesscol += DWC_ETH_QOS_reg_read(MMC_TXEXESSCOL_RgOffAddr);
	mmc->mmc_tx_carrier_error += DWC_ETH_QOS_reg_read(MMC_TXCARRIERERROR_RgOffAddr);
	mmc->mmc_tx_octetcount_g += DWC_ETH_QOS_reg_read(MMC_TXOCTETCOUNT_G_RgOffAddr);
	mmc->mmc_tx_framecount_g += DWC_ETH_QOS_reg_read(MMC_TXPACKETSCOUNT_G_RgOffAddr);
	mmc->mmc_tx_excessdef += DWC_ETH_QOS_reg_read(MMC_TXEXCESSDEF_RgOffAddr);
	mmc->mmc_tx_pause_frame += DWC_ETH_QOS_reg_read(MMC_TXPAUSEPACKETS_RgOffAddr);
	mmc->mmc_tx_vlan_frame_g += DWC_ETH_QOS_reg_read(MMC_TXVLANPACKETS_G_RgOffAddr);
	mmc->mmc_tx_osize_frame_g += DWC_ETH_QOS_reg_read(MMC_TXOVERSIZE_G_RgOffAddr);

	/* MMC RX counter registers */
	mmc->mmc_rx_framecount_gb += DWC_ETH_QOS_reg_read(MMC_RXPACKETCOUNT_GB_RgOffAddr);
	mmc->mmc_rx_octetcount_gb += DWC_ETH_QOS_reg_read(MMC_RXOCTETCOUNT_GB_RgOffAddr);
	mmc->mmc_rx_octetcount_g += DWC_ETH_QOS_reg_read(MMC_RXOCTETCOUNT_G_RgOffAddr);
	mmc->mmc_rx_broadcastframe_g += DWC_ETH_QOS_reg_read(MMC_RXBROADCASTPACKETS_G_RgOffAddr);
	mmc->mmc_rx_multicastframe_g += DWC_ETH_QOS_reg_read(MMC_RXMULTICASTPACKETS_G_RgOffAddr);
	mmc->mmc_rx_crc_errror += DWC_ETH_QOS_reg_read(MMC_RXCRCERROR_RgOffAddr);
	mmc->mmc_rx_align_error += DWC_ETH_QOS_reg_read(MMC_RXALIGNMENTERROR_RgOffAddr);
	mmc->mmc_rx_run_error += DWC_ETH_QOS_reg_read(MMC_RXRUNTERROR_RgOffAddr);
	mmc->mmc_rx_jabber_error += DWC_ETH_QOS_reg_read(MMC_RXJABBERERROR_RgOffAddr);
	mmc->mmc_rx_undersize_g += DWC_ETH_QOS_reg_read(MMC_RXUNDERSIZE_G_RgOffAddr);
	mmc->mmc_rx_oversize_g += DWC_ETH_QOS_reg_read(MMC_RXOVERSIZE_G_RgOffAddr);
	mmc->mmc_rx_64_octets_gb += DWC_ETH_QOS_reg_read(MMC_RX64OCTETS_GB_RgOffAddr);
	mmc->mmc_rx_65_to_127_octets_gb += DWC_ETH_QOS_reg_read(MMC_RX65TO127OCTETS_GB_RgOffAddr);
	mmc->mmc_rx_128_to_255_octets_gb += DWC_ETH_QOS_reg_read(MMC_RX128TO255OCTETS_GB_RgOffAddr);
	mmc->mmc_rx_256_to_511_octets_gb += DWC_ETH_QOS_reg_read(MMC_RX256TO511OCTETS_GB_RgOffAddr);
	mmc->mmc_rx_512_to_1023_octets_gb += DWC_ETH_QOS_reg_read(MMC_RX512TO1023OCTETS_GB_RgOffAddr);
	mmc->mmc_rx_1024_to_max_octets_gb += DWC_ETH_QOS_reg_read(MMC_RX1024TOMAXOCTETS_GB_RgOffAddr);
	mmc->mmc_rx_unicast_g += DWC_ETH_QOS_reg_read(MMC_RXUNICASTPACKETS_G_RgOffAddr);
	mmc->mmc_rx_length_error += DWC_ETH_QOS_reg_read(MMC_RXLENGTHERROR_RgOffAddr);
	mmc->mmc_rx_outofrangetype += DWC_ETH_QOS_reg_read(MMC_RXOUTOFRANGETYPE_RgOffAddr);
	mmc->mmc_rx_pause_frames += DWC_ETH_QOS_reg_read(MMC_RXPAUSEPACKETS_RgOffAddr);
	mmc->mmc_rx_fifo_overflow += DWC_ETH_QOS_reg_read(MMC_RXFIFOOVERFLOW_RgOffAddr);
	mmc->mmc_rx_vlan_frames_gb += DWC_ETH_QOS_reg_read(MMC_RXVLANPACKETS_GB_RgOffAddr);
	mmc->mmc_rx_watchdog_error += DWC_ETH_QOS_reg_read(MMC_RXWATCHDOGERROR_RgOffAddr);
	mmc->mmc_rx_receive_error += DWC_ETH_QOS_reg_read(MMC_RXRCVERROR_RgOffAddr);
	mmc->mmc_rx_ctrl_frames_g += DWC_ETH_QOS_reg_read(MMC_RXCTRLPACKETS_G_RgOffAddr);

	/* IPC */
	mmc->mmc_rx_ipc_intr_mask += DWC_ETH_QOS_reg_read(MMC_IPC_INTR_MASK_RX_RgOffAddr);
	mmc->mmc_rx_ipc_intr += DWC_ETH_QOS_reg_read(MMC_IPC_INTR_RX_RgOffAddr);

	/* IPv4 */
	mmc->mmc_rx_ipv4_gd += DWC_ETH_QOS_reg_read(MMC_RXIPV4_GD_PKTS_RgOffAddr);
	mmc->mmc_rx_ipv4_hderr += DWC_ETH_QOS_reg_read(MMC_RXIPV4_HDRERR_PKTS_RgOffAddr);
	mmc->mmc_rx_ipv4_nopay += DWC_ETH_QOS_reg_read(MMC_RXIPV4_NOPAY_PKTS_RgOffAddr);
	mmc->mmc_rx_ipv4_frag += DWC_ETH_QOS_reg_read(MMC_RXIPV4_FRAG_PKTS_RgOffAddr);
	mmc->mmc_rx_ipv4_udsbl += DWC_ETH_QOS_reg_read(MMC_RXIPV4_UBSBL_PKTS_RgOffAddr);

	/* IPV6 */
	mmc->mmc_rx_ipv6_gd += DWC_ETH_QOS_reg_read(MMC_RXIPV6_GD_PKTS_RgOffAddr);
	mmc->mmc_rx_ipv6_hderr += DWC_ETH_QOS_reg_read(MMC_RXIPV6_HDRERR_PKTS_RgOffAddr);
	mmc->mmc_rx_ipv6_nopay += DWC_ETH_QOS_reg_read(MMC_RXIPV6_NOPAY_PKTS_RgOffAddr);

	/* Protocols */
	mmc->mmc_rx_udp_gd += DWC_ETH_QOS_reg_read(MMC_RXUDP_GD_PKTS_RgOffAddr);
	mmc->mmc_rx_udp_err += DWC_ETH_QOS_reg_read(MMC_RXUDP_ERR_PKTS_RgOffAddr);
	mmc->mmc_rx_tcp_gd += DWC_ETH_QOS_reg_read(MMC_RXTCP_GD_PKTS_RgOffAddr);
	mmc->mmc_rx_tcp_err += DWC_ETH_QOS_reg_read(MMC_RXTCP_ERR_PKTS_RgOffAddr);
	mmc->mmc_rx_icmp_gd += DWC_ETH_QOS_reg_read(MMC_RXICMP_GD_PKTS_RgOffAddr);
	mmc->mmc_rx_icmp_err += DWC_ETH_QOS_reg_read(MMC_RXICMP_ERR_PKTS_RgOffAddr);

	/* IPv4 */
	mmc->mmc_rx_ipv4_gd_octets += DWC_ETH_QOS_reg_read(MMC_RXIPV4_GD_OCTETS_RgOffAddr);
	mmc->mmc_rx_ipv4_hderr_octets += DWC_ETH_QOS_reg_read(MMC_RXIPV4_HDRERR_OCTETS_RgOffAddr);
	mmc->mmc_rx_ipv4_nopay_octets += DWC_ETH_QOS_reg_read(MMC_RXIPV4_NOPAY_OCTETS_RgOffAddr);
	mmc->mmc_rx_ipv4_frag_octets += DWC_ETH_QOS_reg_read(MMC_RXIPV4_FRAG_OCTETS_RgOffAddr);
	mmc->mmc_rx_ipv4_udsbl_octets += DWC_ETH_QOS_reg_read(MMC_RXIPV4_UDSBL_OCTETS_RgOffAddr);

	/* IPV6 */
	mmc->mmc_rx_ipv6_gd_octets += DWC_ETH_QOS_reg_read(MMC_RXIPV6_GD_OCTETS_RgOffAddr);
	mmc->mmc_rx_ipv6_hderr_octets += DWC_ETH_QOS_reg_read(MMC_RXIPV6_HDRERR_OCTETS_RgOffAddr);
	mmc->mmc_rx_ipv6_nopay_octets += DWC_ETH_QOS_reg_read(MMC_RXIPV6_NOPAY_OCTETS_RgOffAddr);

	/* Protocols */
	mmc->mmc_rx_udp_gd_octets += DWC_ETH_QOS_reg_read(MMC_RXUDP_GD_OCTETS_RgOffAddr);
	mmc->mmc_rx_udp_err_octets += DWC_ETH_QOS_reg_read(MMC_RXUDP_ERR_OCTETS_RgOffAddr);
	mmc->mmc_rx_tcp_gd_octets += DWC_ETH_QOS_reg_read(MMC_RXTCP_GD_OCTETS_RgOffAddr);
	mmc->mmc_rx_tcp_err_octets += DWC_ETH_QOS_reg_read(MMC_RXTCP_ERR_OCTETS_RgOffAddr);
	mmc->mmc_rx_icmp_gd_octets += DWC_ETH_QOS_reg_read(MMC_RXICMP_GD_OCTETS_RgOffAddr);
	mmc->mmc_rx_icmp_err_octets += DWC_ETH_QOS_reg_read(MMC_RXICMP_ERR_OCTETS_RgOffAddr);

	DBG_eMAC_Print(DBG_EMAC,"<--DWC_ETH_QOS_mmc_read\n");
}

void DWC_ETH_QOS_mmc_dump(struct DWC_ETH_QOS_mmc_counters const *mmc)
{
	DBG_eMAC_Print(DBG_EMAC,"\n============================= MMC Dump ================================\n");
	DBG_eMAC_Print(DBG_EMAC,"  \nMMC TX counters:\n" );
	if(mmc->mmc_tx_octetcount_gb != 0 )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_octetcount_gb            : 0x%x\n", mmc->mmc_tx_octetcount_gb         );
	if(mmc->mmc_tx_framecount_gb    != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_framecount_gb            : 0x%x\n", mmc->mmc_tx_framecount_gb         );
	if(mmc->mmc_tx_broadcastframe_g   != 0  )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_broadcastframe_g         : 0x%x\n", mmc->mmc_tx_broadcastframe_g      );
	if(mmc->mmc_tx_multicastframe_g  != 0   )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_multicastframe_g         : 0x%x\n", mmc->mmc_tx_multicastframe_g      );
	if(mmc->mmc_tx_64_octets_gb      != 0   )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_64_octets_gb             : 0x%x\n", mmc->mmc_tx_64_octets_gb          );
	if(mmc->mmc_tx_65_to_127_octets_gb  != 0 )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_65_to_127_octets_gb      : 0x%x\n", mmc->mmc_tx_65_to_127_octets_gb   );
	if(mmc->mmc_tx_128_to_255_octets_gb  != 0)  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_128_to_255_octets_gb     : 0x%x\n", mmc->mmc_tx_128_to_255_octets_gb  );
	if(mmc->mmc_tx_256_to_511_octets_gb  != 0)  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_256_to_511_octets_gb     : 0x%x\n", mmc->mmc_tx_256_to_511_octets_gb  );
	if(mmc->mmc_tx_512_to_1023_octets_gb  != 0) DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_512_to_1023_octets_gb    : 0x%x\n", mmc->mmc_tx_512_to_1023_octets_gb );
	if(mmc->mmc_tx_1024_to_max_octets_gb  != 0) DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_1024_to_max_octets_gb    : 0x%x\n", mmc->mmc_tx_1024_to_max_octets_gb );
	if(mmc->mmc_tx_unicast_gb      != 0     )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_unicast_gb               : 0x%x\n", mmc->mmc_tx_unicast_gb            );
	if(mmc->mmc_tx_multicast_gb     != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_multicast_gb             : 0x%x\n", mmc->mmc_tx_multicast_gb          );
	if(mmc->mmc_tx_broadcast_gb     != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_broadcast_gb             : 0x%x\n", mmc->mmc_tx_broadcast_gb          );
	if(mmc->mmc_tx_underflow_error   != 0   )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_underflow_error          : 0x%x\n", mmc->mmc_tx_underflow_error       );
	if(mmc->mmc_tx_singlecol_g      != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_singlecol_g              : 0x%x\n", mmc->mmc_tx_singlecol_g           );
	if(mmc->mmc_tx_multicol_g      != 0     )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_multicol_g               : 0x%x\n", mmc->mmc_tx_multicol_g            );
	if(mmc->mmc_tx_deferred         != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_deferred                 : 0x%x\n", mmc->mmc_tx_deferred              );
	if(mmc->mmc_tx_latecol          != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_latecol                  : 0x%x\n", mmc->mmc_tx_latecol               );
	if(mmc->mmc_tx_exesscol         != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_exesscol                 : 0x%x\n", mmc->mmc_tx_exesscol              );
	if(mmc->mmc_tx_carrier_error    != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_carrier_error            : 0x%x\n", mmc->mmc_tx_carrier_error         );
	if(mmc->mmc_tx_octetcount_g     != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_octetcount_g             : 0x%x\n", mmc->mmc_tx_octetcount_g          );
	if(mmc->mmc_tx_framecount_g     != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_framecount_g             : 0x%x\n", mmc->mmc_tx_framecount_g          );
	if(mmc->mmc_tx_excessdef        != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_excessdef                : 0x%x\n", mmc->mmc_tx_excessdef             );
	if(mmc->mmc_tx_pause_frame       != 0   )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_pause_frame              : 0x%x\n", mmc->mmc_tx_pause_frame           );
	if(mmc->mmc_tx_vlan_frame_g     != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_vlan_frame_g             : 0x%x\n", mmc->mmc_tx_vlan_frame_g          );
	if(mmc->mmc_tx_osize_frame_g     != 0   )  DBG_eMAC_Print(DBG_EMAC,"  mmc_tx_osize_frame_g            : 0x%x\n", mmc->mmc_tx_osize_frame_g         );

	DBG_eMAC_Print(DBG_EMAC,"  \nMMC RX counters:\n" );                  
	if(mmc->mmc_rx_framecount_gb    != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_framecount_gb            : 0x%x\n", mmc->mmc_rx_framecount_gb         );
	if(mmc->mmc_rx_octetcount_gb    != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_octetcount_gb            : 0x%x\n", mmc->mmc_rx_octetcount_gb         );
	if(mmc->mmc_rx_octetcount_g      != 0   )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_octetcount_g             : 0x%x\n", mmc->mmc_rx_octetcount_g          );
	if(mmc->mmc_rx_broadcastframe_g != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_broadcastframe_g         : 0x%x\n", mmc->mmc_rx_broadcastframe_g      );
	if(mmc->mmc_rx_multicastframe_g != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_multicastframe_g         : 0x%x\n", mmc->mmc_rx_multicastframe_g      );
	if(mmc->mmc_rx_crc_errror       != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_crc_errror               : 0x%x\n", mmc->mmc_rx_crc_errror            );
	if(mmc->mmc_rx_align_error      != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_align_error              : 0x%x\n", mmc->mmc_rx_align_error           );
	if(mmc->mmc_rx_run_error        != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_run_error                : 0x%x\n", mmc->mmc_rx_run_error             );
	if(mmc->mmc_rx_jabber_error     != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_jabber_error             : 0x%x\n", mmc->mmc_rx_jabber_error          );
	if(mmc->mmc_rx_undersize_g      != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_undersize_g              : 0x%x\n", mmc->mmc_rx_undersize_g           );
	if(mmc->mmc_rx_oversize_g      != 0     )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_oversize_g               : 0x%x\n", mmc->mmc_rx_oversize_g            );
	if(mmc->mmc_rx_64_octets_gb     != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_64_octets_gb             : 0x%x\n", mmc->mmc_rx_64_octets_gb          );
	if(mmc->mmc_rx_65_to_127_octets_gb != 0 )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_65_to_127_octets_gb      : 0x%x\n", mmc->mmc_rx_65_to_127_octets_gb   );
	if(mmc->mmc_rx_128_to_255_octets_gb != 0)  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_128_to_255_octets_gb     : 0x%x\n", mmc->mmc_rx_128_to_255_octets_gb  );
	if(mmc->mmc_rx_256_to_511_octets_gb != 0)  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_256_to_511_octets_gb     : 0x%x\n", mmc->mmc_rx_256_to_511_octets_gb  );
	if(mmc->mmc_rx_512_to_1023_octets_gb != 0) DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_512_to_1023_octets_gb    : 0x%x\n", mmc->mmc_rx_512_to_1023_octets_gb );
	if(mmc->mmc_rx_1024_to_max_octets_gb != 0) DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_1024_to_max_octets_gb    : 0x%x\n", mmc->mmc_rx_1024_to_max_octets_gb );
	if(mmc->mmc_rx_unicast_g      != 0      )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_unicast_g                : 0x%x\n", mmc->mmc_rx_unicast_g             );
	if(mmc->mmc_rx_length_error    != 0     )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_length_error             : 0x%x\n", mmc->mmc_rx_length_error          );
	if(mmc->mmc_rx_outofrangetype != 0      )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_outofrangetype           : 0x%x\n", mmc->mmc_rx_outofrangetype        );
	if(mmc->mmc_rx_pause_frames   != 0      )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_pause_frames             : 0x%x\n", mmc->mmc_rx_pause_frames          );
	if(mmc->mmc_rx_fifo_overflow  != 0      )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_fifo_overflow            : 0x%x\n", mmc->mmc_rx_fifo_overflow         );
	if(mmc->mmc_rx_vlan_frames_gb != 0      )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_vlan_frames_gb           : 0x%x\n", mmc->mmc_rx_vlan_frames_gb        );
	if(mmc->mmc_rx_watchdog_error != 0      )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_watchdog_error           : 0x%x\n", mmc->mmc_rx_watchdog_error        );
	if(mmc->mmc_rx_receive_error  != 0      )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_receive_error            : 0x%x\n", mmc->mmc_rx_receive_error         );
	if(mmc->mmc_rx_ctrl_frames_g  != 0      )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ctrl_frames_g            : 0x%x\n", mmc->mmc_rx_ctrl_frames_g         );

	DBG_eMAC_Print(DBG_EMAC,"  \nIPC: \n" );                            
	if(mmc->mmc_rx_ipc_intr_mask  != 0      )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipc_intr_mask            : 0x%x\n", mmc->mmc_rx_ipc_intr_mask         );
	if(mmc->mmc_rx_ipc_intr       != 0      )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipc_intr                 : 0x%x\n", mmc->mmc_rx_ipc_intr              );

	DBG_eMAC_Print(DBG_EMAC,"  \nIPv4:\n" );
	if(mmc->mmc_rx_ipv4_gd       != 0       )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipv4_gd                  : 0x%x\n", mmc->mmc_rx_ipv4_gd               );
	if(mmc->mmc_rx_ipv4_hderr    != 0       )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipv4_hderr               : 0x%x\n", mmc->mmc_rx_ipv4_hderr            );
	if(mmc->mmc_rx_ipv4_nopay    != 0       )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipv4_nopay               : 0x%x\n", mmc->mmc_rx_ipv4_nopay            );
	if(mmc->mmc_rx_ipv4_frag    != 0        )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipv4_frag                : 0x%x\n", mmc->mmc_rx_ipv4_frag             );
	if(mmc->mmc_rx_ipv4_udsbl    != 0       )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipv4_udsbl               : 0x%x\n", mmc->mmc_rx_ipv4_udsbl            );

	DBG_eMAC_Print(DBG_EMAC,"  \nIPV6:\n"  );                           
	if(mmc->mmc_rx_ipv6_gd_octets   != 0    )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipv6_gd_octets           : 0x%x\n", mmc->mmc_rx_ipv6_gd_octets        );
	if(mmc->mmc_rx_ipv6_hderr_octets != 0   )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipv6_hderr_octets        : 0x%x\n", mmc->mmc_rx_ipv6_hderr_octets     );
	if(mmc->mmc_rx_ipv6_nopay_octets  != 0  )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipv6_nopay_octets        : 0x%x\n", mmc->mmc_rx_ipv6_nopay_octets     );

	DBG_eMAC_Print(DBG_EMAC,"  \nProtocols: \n" );                         
	if(mmc->mmc_rx_udp_gd     != 0          )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_udp_gd                   : 0x%x\n", mmc->mmc_rx_udp_gd                );
	if(mmc->mmc_rx_udp_err    != 0          )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_udp_err                  : 0x%x\n", mmc->mmc_rx_udp_err               );
	if(mmc->mmc_rx_tcp_gd     != 0          )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_tcp_gd                   : 0x%x\n", mmc->mmc_rx_tcp_gd                );
	if(mmc->mmc_rx_tcp_err    != 0          )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_tcp_err                  : 0x%x\n", mmc->mmc_rx_tcp_err               );
	if(mmc->mmc_rx_icmp_gd    != 0          )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_icmp_gd                  : 0x%x\n", mmc->mmc_rx_icmp_gd               );
	if(mmc->mmc_rx_icmp_err   != 0          )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_icmp_err                 : 0x%x\n", mmc->mmc_rx_icmp_err              );

	DBG_eMAC_Print(DBG_EMAC,"  \nIPv4: \n" );                           
	if(mmc->mmc_rx_ipv4_gd_octets    != 0   )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipv4_gd_octets           : 0x%x\n", mmc->mmc_rx_ipv4_gd_octets        );
	if(mmc->mmc_rx_ipv4_hderr_octets != 0   )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipv4_hderr_octets        : 0x%x\n", mmc->mmc_rx_ipv4_hderr_octets     );
	if(mmc->mmc_rx_ipv4_nopay_octets  != 0  )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipv4_nopay_octets        : 0x%x\n", mmc->mmc_rx_ipv4_nopay_octets     );
	if(mmc->mmc_rx_ipv4_frag_octets  != 0   )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipv4_frag_octets         : 0x%x\n", mmc->mmc_rx_ipv4_frag_octets      );
	if(mmc->mmc_rx_ipv4_udsbl_octets != 0   )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipv4_udsbl_octets        : 0x%x\n", mmc->mmc_rx_ipv4_udsbl_octets     );

	DBG_eMAC_Print(DBG_EMAC,"  \nIPV6: \n"   );                           
	if(mmc->mmc_rx_ipv6_gd    != 0          )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipv6_gd                  : 0x%x\n", mmc->mmc_rx_ipv6_gd               );
	if(mmc->mmc_rx_ipv6_hderr  != 0         )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipv6_hderr               : 0x%x\n", mmc->mmc_rx_ipv6_hderr            );
	if(mmc->mmc_rx_ipv6_nopay  != 0         )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_ipv6_nopay               : 0x%x\n", mmc->mmc_rx_ipv6_nopay            );

	DBG_eMAC_Print(DBG_EMAC,"  \nProtocols: \n" );                       
	if(mmc->mmc_rx_udp_gd_octets  != 0      )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_udp_gd_octets            : 0x%x\n", mmc->mmc_rx_udp_gd_octets         );
	if(mmc->mmc_rx_udp_err_octets != 0      )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_udp_err_octets           : 0x%x\n", mmc->mmc_rx_udp_err_octets        );
	if(mmc->mmc_rx_tcp_gd_octets  != 0      )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_tcp_gd_octets            : 0x%x\n", mmc->mmc_rx_tcp_gd_octets         );
	if(mmc->mmc_rx_tcp_err_octets != 0      )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_tcp_err_octets           : 0x%x\n", mmc->mmc_rx_tcp_err_octets        );
	if(mmc->mmc_rx_icmp_gd_octets != 0      )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_icmp_gd_octets           : 0x%x\n", mmc->mmc_rx_icmp_gd_octets        );
	if(mmc->mmc_rx_icmp_err_octets != 0     )  DBG_eMAC_Print(DBG_EMAC,"  mmc_rx_icmp_err_octets          : 0x%x\n", mmc->mmc_rx_icmp_err_octets       );
}
#endif
