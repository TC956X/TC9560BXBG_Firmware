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

/* =========================================================================
 * The Synopsys DWC ETHER QOS Software Driver and documentation (hereinafter
 * "Software") is an unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto.  Permission is hereby granted,
 * free of charge, to any person obtaining a copy of this software annotated
 * with this license and the Software, to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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
 * ========================================================================= */

/*!@file: DWC_ETH_QOS_dev.c
 * @brief: Driver functions.
 */
 
#include <includes.h>
#include "DWC_ETH_QOS_yregacc.h"
#include "dwc_otg_dbg.h"

/*!
* \brief This sequence is used to enable/disable MAC loopback mode
* \param[in] enb_dis
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t config_mac_loopback_mode(uint32_t enb_dis)
{

	MAC_MCR_LM_UdfWr(enb_dis);

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to enable/disable VLAN filtering and
* also selects VLAN filtering mode- perfect/hash
* \param[in] filter_enb_dis
* \param[in] perfect_hash
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t config_vlan_filtering(uint32_t filter_enb_dis, uint32_t perfect_hash_filtering, uint32_t perfect_inverse_match)
{
	MAC_MPFR_VTFE_UdfWr(filter_enb_dis);
	MAC_VLANTR_VTIM_UdfWr(perfect_inverse_match);
	MAC_VLANTR_VTHM_UdfWr(perfect_hash_filtering);
	/* To enable only HASH filtering then VL/VID
	* should be > zero. Hence we are writting 1 into VL.
	* It also means that MAC will always receive VLAN pkt with
	* VID = 1 if inverse march is not set.
	* */
	if (perfect_hash_filtering != 0) {
		MAC_VLANTR_VL_UdfWr(0x1U);
	}
	/* By default enable MAC to calculate vlan hash on
	* only 12-bits of received VLAN tag (ie only on
	* VLAN id and ignore priority and other fields)
	* */
	if (perfect_hash_filtering != 0) {
		  MAC_VLANTR_ETV_UdfWr(0x1U);
	}
	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to configure MAC in differnet pkt processing
* modes like promiscuous, multicast, unicast, hash unicast/multicast.
* \param[in] pr_mode
* \param[in] huc_mode
* \param[in] hmc_mode
* \param[in] pm_mode
* \param[in] hpf_mode
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t config_mac_pkt_filter_reg(UCHAR pr_mode, UCHAR huc_mode, UCHAR hmc_mode, UCHAR pm_mode, UCHAR hpf_mode)
{
	ULONG varMAC_MPFR;
	/* configure device in differnet modes */
	/* promiscuous, hash unicast, hash multicast, */
	/* all multicast and perfect/hash filtering mode. */
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	MAC_MPFR_RgRd(varMAC_MPFR);
	varMAC_MPFR = varMAC_MPFR & (ULONG)(0x803103e8U);
	varMAC_MPFR = varMAC_MPFR | ((pr_mode) << 0) | ((huc_mode) << 1) | ((hmc_mode) << 2) | ((pm_mode) << 4) | ((ULONG)(hpf_mode) << 10);
	MAC_MPFR_RgWr(varMAC_MPFR);
	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to initialize the system time
* \param[in] sec
* \param[in] nsec
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t init_systime(uint32_t sec, uint32_t nsec)
{
	ULONG 			retryCount = 100000;
	ULONG 			vy_count;
	volatile ULONG 	varMAC_TCR;
	int32_t 		flag = 0;
	int32_t 		retval = Y_SUCCESS;
	/* wait for previous(if any) time initialize to complete. */
	/*Poll*/
	vy_count = 0;
	while(flag == 0){
		if(vy_count > retryCount) {
			retval = -Y_FAILURE;
			flag = 1;
		}
		else {
			MAC_TCR_RgRd(varMAC_TCR);
			if (GET_VALUE(varMAC_TCR, MAC_TCR_TSINIT_LPOS, MAC_TCR_TSINIT_HPOS) == 0) {
				break;
			}
			vy_count++;
			mdelay(1);
		}
	}
	if(flag == 0){
		MAC_STSUR_RgWr(sec);
		MAC_STNSUR_RgWr(nsec);
		/* issue command to initialize system time with the value */
		/* specified in MAC_STSUR and MAC_STNSUR. */
		MAC_TCR_TSINIT_UdfWr(0x1U);
		/* wait for present time initialize to complete. */
		
		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				retval = -Y_FAILURE;
				flag = 1;
			}
			else {
				MAC_TCR_RgRd(varMAC_TCR);
				if (GET_VALUE(varMAC_TCR, MAC_TCR_TSINIT_LPOS, MAC_TCR_TSINIT_HPOS) == 0) {
					break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	}
	return retval;
}

/*!
* \brief This sequence is used to select Tx Scheduling Algorithm for DCB feature
* \param[in] dcb_algo
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t set_dcb_algorithm(UCHAR dcb_algo)
{
	MTL_OMR_SCHALG_UdfWr(dcb_algo);
	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to disables all Tx/Rx MMC interrupts
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t disable_mmc_interrupts(void)
{

	/* disable all TX interrupts */
	MMC_INTR_MASK_TX_RgWr(0xffffffffU);
	/* disable all RX interrupts */
	MMC_INTR_MASK_RX_RgWr(0xffffffffU);
	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to configure MMC counters
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t config_mmc_counters(void)
{
	ULONG varMMC_CNTRL;
	/* set COUNTER RESET */
	/* set RESET ON READ */
	/* set COUNTER PRESET */
	/* set FULL_HALF PRESET */
	MMC_CNTRL_RgRd(varMMC_CNTRL);
	varMMC_CNTRL = varMMC_CNTRL & (ULONG)(0x10a);
	/*varMMC_CNTRL = varMMC_CNTRL | ((0x1) << 0) | ((0x1) << 2) | ((0x1) << 4) | ((0x1) << 5); */
	varMMC_CNTRL = varMMC_CNTRL | ((0x1) << 0) | ((0x1) << 2) ;
	MMC_CNTRL_RgWr(varMMC_CNTRL);
	return Y_SUCCESS;
}


/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

int32_t configure_mac_for_vlan_pkt(void)
{
	/* Enable VLAN Tag stripping always */
	MAC_VLANTR_EVLS_UdfWr(0x3U);
	/* Enable operation on the outer VLAN Tag, if present */
	MAC_VLANTR_ERIVLT_UdfWr(0U);
	/* Disable double VLAN Tag processing on TX and RX */
	MAC_VLANTR_EDVLP_UdfWr(0U);
	/* Enable VLAN Tag in RX Status. */
	MAC_VLANTR_EVLRXS_UdfWr(0x1U);
	/* Disable VLAN Type Check */
	MAC_VLANTR_DOVLTC_UdfWr(0x1U);

	/* configure MAC to get VLAN Tag to be inserted/replaced from */
	/* TX descriptor(context desc) */
	MAC_VLANTIRR_VLTI_UdfWr(0x1U);
	/* insert/replace C_VLAN in 13th ans 14th bytes of transmitted frames */
	MAC_VLANTIRR_CSVL_UdfWr(0U);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t disable_tx_flow_ctrl(uint32_t qInx)
{
	MAC_QTFCR_TFE_UdfWr(qInx, 0U);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t enable_tx_flow_ctrl(uint32_t qInx)
{
	MAC_QTFCR_TFE_UdfWr(qInx, 1U);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t disable_rx_flow_ctrl(void)
{
	MAC_RFCR_RFE_UdfWr(0U);

	return Y_SUCCESS;
}

/*!
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t enable_rx_flow_ctrl(void)
{
	MAC_RFCR_RFE_UdfWr(0x1U);

	return Y_SUCCESS;
}

/*!
* \param[in] qInx
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t stop_dma_rx(uint32_t qInx)
{
	ULONG 			retryCount = 10;
	ULONG 			vy_count;
	volatile ULONG 	varDMA_DSR0;
	volatile ULONG	varDMA_DSR1;
	volatile ULONG 	varDMA_DSR2;
	int32_t flag = 0;
	int32_t retval = Y_SUCCESS;
	/* issue Rx dma stop command */
	DMA_RCR_ST_UdfWr(qInx, 0U);
	/* wait for Rx DMA to stop, ie wait till Rx DMA
	* goes in either Running or Suspend state.
	* */
	if (qInx == 0) {

		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				/*printk(KERN_ALERT "ERROR: Rx Channel 0 stop failed, DSR0 = %#lx\n",varDMA_DSR0); */
				retval = -Y_FAILURE;
				flag = 1;
			}
			else {
				DMA_DSR0_RgRd(varDMA_DSR0);
				if ((GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS0_LPOS, DMA_DSR0_RPS0_HPOS) == 0x3)
					|| (GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS0_LPOS, DMA_DSR0_RPS0_HPOS) == 0x4)
					|| (GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS0_LPOS, DMA_DSR0_RPS0_HPOS) == 0x0)) {
						break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	}
	else if (qInx == 1) {

		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				/*printk(KERN_ALERT "ERROR: Rx Channel 1 stop failed, DSR0 = %#lx\n",varDMA_DSR0); */
				retval = -Y_FAILURE;
				flag = 1;
			}
			else {
				DMA_DSR0_RgRd(varDMA_DSR0);
				if ((GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS1_LPOS, DMA_DSR0_RPS1_HPOS) == 0x3)
					|| (GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS1_LPOS, DMA_DSR0_RPS1_HPOS) == 0x4)
					|| (GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS1_LPOS, DMA_DSR0_RPS1_HPOS) == 0x0)) {
						break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	} else if (qInx == 2) {

		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				/*printk(KERN_ALERT "ERROR: Rx Channel 2 stop failed, DSR0 = %#lx\n",varDMA_DSR0); */
				retval = -Y_FAILURE;
				flag = 1;
			}
			else {
				DMA_DSR0_RgRd(varDMA_DSR0);
				if ((GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS2_LPOS, DMA_DSR0_RPS2_HPOS) == 0x3)
					|| (GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS2_LPOS, DMA_DSR0_RPS2_HPOS) == 0x4)
					|| (GET_VALUE(varDMA_DSR0, DMA_DSR0_RPS2_LPOS, DMA_DSR0_RPS2_HPOS) == 0x0)) {
						break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	} else if (qInx == 3) {

		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				/*printk(KERN_ALERT "ERROR: Rx Channel 3 stop failed, DSR0 = %#lx\n", varDMA_DSR1); */
				retval = -Y_FAILURE;
				flag = 1;
			}
			else {
				DMA_DSR1_RgRd(varDMA_DSR1);
				if ((GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS3_LPOS, DMA_DSR1_RPS3_HPOS) == 0x3)
					|| (GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS3_LPOS, DMA_DSR1_RPS3_HPOS) == 0x4)
					|| (GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS3_LPOS, DMA_DSR1_RPS3_HPOS) == 0x0)) {
						break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	} else if (qInx == 4) {

		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				/*printk(KERN_ALERT "ERROR: Rx Channel 4 stop failed, DSR0 = %#lx\n", varDMA_DSR1); */
				retval = -Y_FAILURE;
				flag = 1;
			}
			else {
				DMA_DSR1_RgRd(varDMA_DSR1);
				if ((GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS4_LPOS, DMA_DSR1_RPS4_HPOS) == 0x3)
					|| (GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS4_LPOS, DMA_DSR1_RPS4_HPOS) == 0x4)
					|| (GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS4_LPOS, DMA_DSR1_RPS4_HPOS) == 0x0)) {
						break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	} else if (qInx == 5) {

		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				/*printk(KERN_ALERT "ERROR: Rx Channel 5 stop failed, DSR0 = %#lx\n", varDMA_DSR1);*/
				retval = -Y_FAILURE;
				flag = 1;
			}
			else {
				DMA_DSR1_RgRd(varDMA_DSR1);
				if ((GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS5_LPOS, DMA_DSR1_RPS5_HPOS) == 0x3)
					|| (GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS5_LPOS, DMA_DSR1_RPS5_HPOS) == 0x4)
					|| (GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS5_LPOS, DMA_DSR1_RPS5_HPOS) == 0x0)) {
					break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	} else if (qInx == 6) {

		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				/*printk(KERN_ALERT "ERROR: Rx Channel 6 stop failed, DSR0 = %#lx\n",varDMA_DSR1);*/
				retval = -Y_FAILURE;
				flag = 1;
			}
			else {
				DMA_DSR1_RgRd(varDMA_DSR1);
				if ((GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS6_LPOS, DMA_DSR1_RPS6_HPOS) == 0x3)
					|| (GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS6_LPOS, DMA_DSR1_RPS6_HPOS) == 0x4)
					|| (GET_VALUE(varDMA_DSR1, DMA_DSR1_RPS6_LPOS, DMA_DSR1_RPS6_HPOS) == 0x0)) {
					break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	} else if (qInx == 7) {

		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				/*printk(KERN_ALERT "ERROR: Rx Channel 7 stop failed, DSR0 = %#lx\n",varDMA_DSR2);*/
				retval = -Y_FAILURE;
				flag = 1;
			}
			else {
				DMA_DSR2_RgRd(varDMA_DSR2);
				if ((GET_VALUE(varDMA_DSR2, DMA_DSR2_RPS7_LPOS, DMA_DSR2_RPS7_HPOS) == 0x3)
					|| (GET_VALUE(varDMA_DSR2, DMA_DSR2_RPS7_LPOS, DMA_DSR2_RPS7_HPOS) == 0x4)
					|| (GET_VALUE(varDMA_DSR2, DMA_DSR2_RPS7_LPOS, DMA_DSR2_RPS7_HPOS) == 0x0)) {
					break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	} 
	return retval;
}

/*!
* \param[in] qInx
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t stop_dma_tx(uint32_t qInx)
{
	int32_t 		retval = Y_SUCCESS;
	int32_t 		flag = 0;
	ULONG 			retryCount = 10U;
	ULONG 			vy_count;
	volatile ULONG 	varDMA_DSR0;
	volatile ULONG 	varDMA_DSR1;
	volatile ULONG 	varDMA_DSR2;
	/* issue Tx dma stop command */
	DMA_TCR_ST_UdfWr(qInx, 0U);
	/* wait for Tx DMA to stop, ie wait till Tx DMA
	* goes in Suspend state or stopped state.
	*/
	if (qInx == 0) {
		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				/*printk(KERN_ALERT "ERROR: Channel 0 stop failed, DSR0 = %lx\n", varDMA_DSR0);*/
				retval = -Y_FAILURE;
				flag = 1;
			}
			else {
				DMA_DSR0_RgRd(varDMA_DSR0);
				if ((GET_VALUE(varDMA_DSR0, DMA_DSR0_TPS0_LPOS, DMA_DSR0_TPS0_HPOS) == 0x6) ||
					(GET_VALUE(varDMA_DSR0, DMA_DSR0_TPS0_LPOS, DMA_DSR0_TPS0_HPOS) == 0x0)) {
					break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	} else if (qInx == 1) {
		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				/*printk(KERN_ALERT "ERROR: Channel 1 stop failed, DSR0 = %lx\n", varDMA_DSR0);*/
				retval = -Y_FAILURE;
				flag = 1;
			}
			else {
				DMA_DSR0_RgRd(varDMA_DSR0);
				if ((GET_VALUE(varDMA_DSR0, DMA_DSR0_TPS1_LPOS, DMA_DSR0_TPS1_HPOS) == 0x6) ||
					(GET_VALUE(varDMA_DSR0, DMA_DSR0_TPS1_LPOS, DMA_DSR0_TPS1_HPOS) == 0x0)) {
					break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	} else if (qInx == 2) {
		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				/*printk(KERN_ALERT "ERROR: Channel 2 stop failed, DSR0 = %lx\n",varDMA_DSR0);*/
				retval = -Y_FAILURE;
				flag = 1;
			}
			else {
				DMA_DSR0_RgRd(varDMA_DSR0);
				if ((GET_VALUE(varDMA_DSR0, DMA_DSR0_TPS2_LPOS, DMA_DSR0_TPS2_HPOS) == 0x6) ||
					(GET_VALUE(varDMA_DSR0, DMA_DSR0_TPS2_LPOS, DMA_DSR0_TPS2_HPOS) == 0x0)) {
					break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	} else if (qInx == 3) {
		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				/*printk(KERN_ALERT "ERROR: Channel 3 stop failed, DSR0 = %lx\n", varDMA_DSR1);*/
				retval = -Y_FAILURE;
				flag = 1;
			}
			else {
				DMA_DSR1_RgRd(varDMA_DSR1);
				if ((GET_VALUE(varDMA_DSR1, DMA_DSR1_TPS3_LPOS, DMA_DSR1_TPS3_HPOS) == 0x6) ||
					(GET_VALUE(varDMA_DSR1, DMA_DSR1_TPS3_LPOS, DMA_DSR1_TPS3_HPOS) == 0x0)) {
					break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	} else if (qInx == 4) {
		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				/*printk(KERN_ALERT "ERROR: Channel 4 stop failed, DSR0 = %lx\n", varDMA_DSR1);*/
				retval = -Y_FAILURE;
				flag = 1;
			}
			else {
				DMA_DSR1_RgRd(varDMA_DSR1);
				if ((GET_VALUE(varDMA_DSR1, DMA_DSR1_TPS4_LPOS, DMA_DSR1_TPS4_HPOS) == 0x6) ||
					(GET_VALUE(varDMA_DSR1, DMA_DSR1_TPS4_LPOS, DMA_DSR1_TPS4_HPOS) == 0x0)) {
					break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	} else if (qInx == 5) {
		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				/*printk(KERN_ALERT "ERROR: Channel 5 stop failed, DSR0 = %lx\n", varDMA_DSR1);*/
				retval = -Y_FAILURE;
				flag = 1;
			}
			else{
				DMA_DSR1_RgRd(varDMA_DSR1);
				if ((GET_VALUE(varDMA_DSR1, DMA_DSR1_TPS5_LPOS, DMA_DSR1_TPS5_HPOS) == 0x6) ||
					(GET_VALUE(varDMA_DSR1, DMA_DSR1_TPS5_LPOS, DMA_DSR1_TPS5_HPOS) == 0x0)) {
					break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	} else if (qInx == 6) {
		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				/*printk(KERN_ALERT "ERROR: Channel 6 stop failed, DSR0 = %lx\n",varDMA_DSR1);*/
				retval = -Y_FAILURE;
				flag = 1;
			}
			else {
				DMA_DSR1_RgRd(varDMA_DSR1);
				if ((GET_VALUE(varDMA_DSR1, DMA_DSR1_TPS6_LPOS, DMA_DSR1_TPS6_HPOS) == 0x6) ||
					(GET_VALUE(varDMA_DSR1, DMA_DSR1_TPS6_LPOS, DMA_DSR1_TPS6_HPOS) == 0x0)) {
					break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	} else if (qInx == 7) {
		/*Poll*/
		vy_count = 0;
		while(flag == 0){
			if(vy_count > retryCount) {
				/*printk(KERN_ALERT "ERROR: Channel 7 stop failed, DSR0 = %lx\n", varDMA_DSR2);*/
				retval = -Y_FAILURE;
				flag = 1;
			}
			else {
				DMA_DSR2_RgRd(varDMA_DSR2);
				if ((GET_VALUE(varDMA_DSR2, DMA_DSR2_TPS7_LPOS, DMA_DSR2_TPS7_HPOS) == 0x6) ||
					(GET_VALUE(varDMA_DSR2, DMA_DSR2_TPS7_LPOS, DMA_DSR2_TPS7_HPOS) == 0x0)) {
					break;
				}
				vy_count++;
				mdelay(1);
			}
		}
	}
	return retval;
}

/*!
* \brief This sequence is used to configure the MAC registers for
* GMII-1000Mbps speed
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t set_gmii_speed(void)
{
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	MAC_MCR_PS_UdfWr(0U);
	MAC_MCR_FES_UdfWr(0U);

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to configure the MAC registers for
* MII-10Mpbs speed
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t set_mii_speed_10(void)
{
    DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	MAC_MCR_PS_UdfWr(0x1U);
	MAC_MCR_FES_UdfWr(0U);

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to configure the MAC registers for
* MII-100Mpbs speed
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t set_mii_speed_100(void)
{
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	MAC_MCR_PS_UdfWr(0x1U);
	MAC_MCR_FES_UdfWr(0x1U);

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to configure the MAC registers for
* half duplex mode
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t set_half_duplex(void)
{
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
    MAC_MCR_DM_UdfWr(0U);

	return Y_SUCCESS;
}

/*!
* \brief This sequence is used to configure the MAC registers for
* full duplex mode
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t set_full_duplex(void)
{
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	MAC_MCR_DM_UdfWr(0x1U);

	return Y_SUCCESS;
}

int32_t disable_promiscuous_mode(void)
{
	MAC_MPFR_PR_UdfWr(0x0U);

	return Y_SUCCESS;
}

/*!
* \brief This function configuraes the PHY autonegotion
* \param[in] enb_dis : 1-Enable, 0-disable
* \param[in] restart : 1-Restart auto negotaion, 0-No effect
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t config_phy_autoneg(int32_t enb_dis, int32_t restart)
{
	uint32_t 	phy_id = 1;
	uint32_t 	phy_reg = PHY_CONTROL_REG;
	uint32_t	phy_reg_data;
	int32_t 	ret;
	
	enb_dis = !!enb_dis;
	restart = !!restart;

	ret = read_phy_regs(phy_id, phy_reg, &phy_reg_data);
	if(ret != Y_SUCCESS){
		DBG_Error_Print("config_phy_autoneg, phy reg read\n");
	}
	else {
		phy_reg_data &= ~((uint32_t)Mii_En_AutoNeg | (uint32_t)Mii_Restart_AutoNeg);
			
		if(enb_dis != 0) {
			phy_reg_data |= Mii_En_AutoNeg;
		}
		if(restart != 0) {
			phy_reg_data |= Mii_Restart_AutoNeg;
		}
		ret = write_phy_regs(phy_id, phy_reg, phy_reg_data);
		if(ret != Y_SUCCESS){
			DBG_Error_Print("config_phy_autoneg, phy reg write\n");
		}
	}
	return ret;
}

/*!
* \brief This sequence is used to write into phy registers
* \param[in] phy_id
* \param[in] phy_reg
* \param[in] phy_reg_data
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t write_phy_regs(uint32_t phy_id, uint32_t phy_reg, uint32_t phy_reg_data)
{
	ULONG 			retryCount = 1000;
	ULONG 			vy_count;
	volatile ULONG 	varMAC_GMIIAR;
	int32_t			flag = 0;
	int32_t 		retval = Y_SUCCESS;
	/* wait for any previous MII read/write operation to complete */
	/*Poll Until Poll Condition */
	vy_count = 0;
	while (flag == 0) {
		if (vy_count > retryCount) {
			retval = -Y_FAILURE;
			flag = 1;
		} else {
			vy_count++;
			mdelay(1);
			MAC_GMIIAR_RgRd(varMAC_GMIIAR);
			if (GET_VALUE(varMAC_GMIIAR, MAC_GMIIAR_GB_LPOS, MAC_GMIIAR_GB_HPOS) == 0) {
				break;
			}
		}
	}
	if(flag == 0) {
		/* write the data */
		MAC_GMIIDR_GD_UdfWr(phy_reg_data);
		/* set busy bit */
		MAC_GMIIAR_RgRd(varMAC_GMIIAR);
		varMAC_GMIIAR = varMAC_GMIIAR & (ULONG) (0x12U);
		varMAC_GMIIAR = varMAC_GMIIAR | ((phy_id) << 21U) | ((phy_reg) << 16U) | ((0x2U) << 8U) | ((0x1U) << 2U) | ((0x1U) << 0U);
		MAC_GMIIAR_RgWr(varMAC_GMIIAR);

		/*DELAY IMPLEMENTATION USING udelay() */
		udelay(10);
		/* wait for MII write operation to complete */

		/*Poll Until Poll Condition */
		vy_count = 0;
		while (flag == 0) {
			if (vy_count > retryCount) {
				retval = -Y_FAILURE;
				flag = 1;
			} else {
				vy_count++;
				mdelay(1);
				MAC_GMIIAR_RgRd(varMAC_GMIIAR);
				if (GET_VALUE(varMAC_GMIIAR, MAC_GMIIAR_GB_LPOS, MAC_GMIIAR_GB_HPOS) == 0) {
					break;
				}
			}
		}
	}
	return retval;
}

/*!
* \brief This sequence is used to read the phy registers
* \param[in] phy_id
* \param[in] phy_reg
* \param[out] phy_reg_data
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/

int32_t read_phy_regs(uint32_t phy_id, uint32_t phy_reg, uint32_t *phy_reg_data)
{
	ULONG 			retryCount = 1000;
	ULONG 			vy_count;
	volatile ULONG 	varMAC_GMIIAR;
	ULONG 			varMAC_GMIIDR;
	int32_t			retval = Y_SUCCESS;
	int32_t			flag = 0;
	/* wait for any previous MII read/write operation to complete */
	/*Poll Until Poll Condition */
	vy_count = 0;
	while (flag == 0) {
		if (vy_count > retryCount) {
			retval = -Y_FAILURE;
			flag = 1;
		} else {
			vy_count++;
			mdelay(1);
			MAC_GMIIAR_RgRd(varMAC_GMIIAR);
			if (GET_VALUE(varMAC_GMIIAR, MAC_GMIIAR_GB_LPOS, MAC_GMIIAR_GB_HPOS) == 0) {
				break;
			}
		}
	}
	if(flag == 0){
		/* initiate the MII read operation by updating desired */
		/* phy address/id (0 - 31) */
		/* phy register offset */
		/* CSR Clock Range (20 - 35MHz) */
		/* Select read operation */
		/* set busy bit */
		MAC_GMIIAR_RgRd(varMAC_GMIIAR);
		varMAC_GMIIAR = varMAC_GMIIAR & (ULONG) (0x12);
		varMAC_GMIIAR =
			varMAC_GMIIAR | ((phy_id) << 21) | ((phy_reg) << 16) | ((0x2) << 8)
			| ((0x3) << 2) | ((0x1) << 0);
		MAC_GMIIAR_RgWr(varMAC_GMIIAR);

		/*DELAY IMPLEMENTATION USING udelay() */
		udelay(10);
		/* wait for MII write operation to complete */

		/*Poll Until Poll Condition */
		vy_count = 0;
		while (1) {
			if (vy_count > retryCount) {
				retval = -Y_FAILURE;
				flag = 1;
			} else {
				vy_count++;
				mdelay(1);
				MAC_GMIIAR_RgRd(varMAC_GMIIAR);
				if (GET_VALUE(varMAC_GMIIAR, MAC_GMIIAR_GB_LPOS, MAC_GMIIAR_GB_HPOS) == 0) {
					break;
				}
			}
		}
		if(flag == 0){
			/* read the data */
			MAC_GMIIDR_RgRd(varMAC_GMIIDR);
			*phy_reg_data = GET_VALUE(varMAC_GMIIDR, MAC_GMIIDR_GD_LPOS, MAC_GMIIDR_GD_HPOS);
		}
	}
	return retval;
}

/*!
* \details This API will calculate per queue FIFO size.
*
* \param[in] fifo_size - total fifo size in h/w register
* \param[in] queue_count - total queue count
*
* \return returns integer
* \retval - fifo size per queue.
*/
uint32_t calculate_per_queue_fifo(ULONG fifo_size, UCHAR queue_count)
{
	ULONG q_fifo_size = 0U;	/* calculated fifo size per queue */
	ULONG p_fifo = eDWC_ETH_QOS_256; /* per queue fifo size programmable value */

	/* calculate Tx/Rx fifo share per queue */
	switch (fifo_size) {
	case 0:
		q_fifo_size = FIFO_SIZE_B(128);
		break;
	case 1:
		q_fifo_size = FIFO_SIZE_B(256);
		break;
	case 2:
		q_fifo_size = FIFO_SIZE_B(512);
		break;
	case 3:
		q_fifo_size = FIFO_SIZE_KB(1);
		break;
	case 4:
		q_fifo_size = FIFO_SIZE_KB(2);
		break;
	case 5:
		q_fifo_size = FIFO_SIZE_KB(4);
		break;
	case 6:
		q_fifo_size = FIFO_SIZE_KB(8);
		break;
	case 7:
		q_fifo_size = FIFO_SIZE_KB(16);
		break;
	case 8:
		q_fifo_size = FIFO_SIZE_KB(32);
		break;
	case 9:
		q_fifo_size = FIFO_SIZE_KB(64);
		break;
	case 10:
		q_fifo_size = FIFO_SIZE_KB(128);
		break;
	case 11:
		q_fifo_size = FIFO_SIZE_KB(256);
		break;
	default :
		break;
	}

	q_fifo_size = q_fifo_size/queue_count;

	if (q_fifo_size >= FIFO_SIZE_KB(32)) {
		p_fifo = eDWC_ETH_QOS_32k;
	} else if (q_fifo_size >= FIFO_SIZE_KB(16)) {
		p_fifo = eDWC_ETH_QOS_16k;
	} else if (q_fifo_size >= FIFO_SIZE_KB(8)) {
		p_fifo = eDWC_ETH_QOS_8k;
	} else if (q_fifo_size >= FIFO_SIZE_KB(4)) {
		p_fifo = eDWC_ETH_QOS_4k;
	} else if (q_fifo_size >= FIFO_SIZE_KB(2)) {
		p_fifo = eDWC_ETH_QOS_2k;
	} else if (q_fifo_size >= FIFO_SIZE_KB(1)) {
		p_fifo = eDWC_ETH_QOS_1k;
	} else if (q_fifo_size >= FIFO_SIZE_B(512)) {
		p_fifo = eDWC_ETH_QOS_512;
	} else if (q_fifo_size >= FIFO_SIZE_B(256)) {
		p_fifo = eDWC_ETH_QOS_256;
	} 
	
	return p_fifo;
}

int32_t configure_mtl_queue(uint32_t qInx)
{
	ULONG 			retryCount = 1000;
	ULONG 			vy_count;
	volatile ULONG 	varMTL_QTOMR;
	uint32_t 		p_rx_fifo = eDWC_ETH_QOS_256;
	uint32_t		p_tx_fifo = eDWC_ETH_QOS_256;
	uint32_t 		varMAC_HFR1, hw_rx_fifo_size;
	uint32_t		hw_tx_fifo_size;
	int32_t 		retval = Y_SUCCESS;
	int32_t 		flag = 0;
	/*DBGPR("-->configure_mtl_queue\n");*/

	MAC_HFR1_RgRd(varMAC_HFR1);  /* read Hardware feature register1*/
	hw_rx_fifo_size = ((varMAC_HFR1 >> 0) & MAC_HFR1_RXFIFOSIZE_Mask);   
	hw_tx_fifo_size = ((varMAC_HFR1 >> 6) & MAC_HFR1_TXFIFOSIZE_Mask);   
	
	/*Flush Tx Queue */
	MTL_QTOMR_FTQ_UdfWr(qInx, 0x1U);

	/*Poll Until Poll Condition */
	vy_count = 0;
	while (flag == 0) {
		if (vy_count > retryCount) {
			retval = -Y_FAILURE;
			flag = 1;
		} else {
			vy_count++;
			mdelay(1);
			MTL_QTOMR_RgRd(qInx, varMTL_QTOMR);  /* TxQ0 Operation Mode*/
				if (GET_VALUE(varMTL_QTOMR, MTL_QTOMR_FTQ_LPOS, MTL_QTOMR_FTQ_HPOS) == 0) { /* check Flush TX Queue*/
					break;
				}
		}
	}
	if(flag == 0) {
		/*Enable Store and Forward mode for TX */
		MTL_QTOMR_TSF_UdfWr(qInx, 0x1U);           /* @0xD00*/
		/* Program Tx operating mode */
		MTL_QTOMR_TXQEN_UdfWr(qInx, 0x2U);         /* TX Queue enabled */
		/* Transmit Queue weight */
		MTL_QW_ISCQW_UdfWr(qInx, (0x10U + qInx));  /* @0xD18*/

		MTL_QROMR_FEP_UdfWr(qInx, 0x1U);           /* @0xD30*/

		p_rx_fifo = calculate_per_queue_fifo(hw_rx_fifo_size, 4); /* total RX Queues are 4*/
		p_tx_fifo = calculate_per_queue_fifo(hw_tx_fifo_size, 3); /* total TX Queues are 3*/

		/* Transmit/Receive queue fifo size programmed */
		MTL_QROMR_RQS_UdfWr(qInx, p_rx_fifo);
		MTL_QTOMR_TQS_UdfWr(qInx, p_tx_fifo);
		/*printf( "Queue%d Tx fifo size %d, Rx fifo size %d\n",	qInx, ((p_tx_fifo + 1) * 256), ((p_rx_fifo + 1) * 256));*/

		/* flow control will be used only if
		 * each channel gets 8KB or more fifo */
		if (p_rx_fifo >= eDWC_ETH_QOS_4k) {
			/* Enable Rx FLOW CTRL in MTL and MAC
				 Programming is valid only if Rx fifo size is greater than
				 or equal to 8k */
			/*if ((pdata->flow_ctrl & DWC_ETH_QOS_FLOW_CTRL_TX) == DWC_ETH_QOS_FLOW_CTRL_TX) */
			{

				MTL_QROMR_EHFC_UdfWr(qInx, 0x1U);

				/* Set Threshold for Activating Flow Contol space for min 2 frames
				 * ie, (1500 * 2) + (64 * 2) = 3128 bytes, rounding off to 4k
				 *
				 * Set Threshold for Deactivating Flow Contol for space of
				 * min 1 frame (frame size 1500bytes) in receive fifo */
				if (p_rx_fifo == eDWC_ETH_QOS_4k) {
					/* This violates the above formula because of FIFO size limit
					 * therefore overflow may occur inspite of this
					 * */
					MTL_QROMR_RFD_UdfWr(qInx, 0x2U);
					MTL_QROMR_RFA_UdfWr(qInx, 0x1U);
				}
				else if (p_rx_fifo == eDWC_ETH_QOS_8k) {
					MTL_QROMR_RFD_UdfWr(qInx, 0x4U);
					MTL_QROMR_RFA_UdfWr(qInx, 0x2U);
				}
				else if (p_rx_fifo == eDWC_ETH_QOS_16k) {
					MTL_QROMR_RFD_UdfWr(qInx, 0x5U);
					MTL_QROMR_RFA_UdfWr(qInx, 0x2U);
				}
				else if (p_rx_fifo == eDWC_ETH_QOS_32k) {
					MTL_QROMR_RFD_UdfWr(qInx, 0x7U);
					MTL_QROMR_RFA_UdfWr(qInx, 0x2U);
				}
			}
		}
	}
	return retval;
}

/*!
* \brief This sequence is used to enable MAC interrupts
* \return Success or Failure
* \retval  0 Success
* \retval -1 Failure
*/
int32_t enable_mac_interrupts(void)
{
  uint64_t varmac_imr;

  /* Enable following interrupts */
  /* RGSMIIIM - RGMII/SMII interrupt Enable */
  /* PCSLCHGIM -  PCS Link Status Interrupt Enable */
  /* PCSANCIM - PCS AN Completion Interrupt Enable */
  /* PMTIM - PMT Interrupt Enable */
  /* LPIIM - LPI Interrupt Enable */
  MAC_IMR_RgRd(varmac_imr);
  varmac_imr = varmac_imr & (uint64_t)(0x1008);
  varmac_imr = varmac_imr | ((0x1) << 0) | ((0x1) << 1) | ((0x1) << 2) |
                ((0x1) << 4) | ((0x1) << 5);
  MAC_IMR_RgWr(varmac_imr);

  return Y_SUCCESS;
}

int32_t configure_mac(void)
{
	uint64_t 	varMAC_MCR;
	uint32_t 		qInx;

	for (qInx = 0U; qInx < DWC_ETH_QOS_RX_QUEUE_CNT; qInx++) {
		MAC_RQC0R_RXQEN_UdfWr(qInx, 0x2);
	}

	/* Set Tx flow control parameters */
	for (qInx = 0; qInx < DWC_ETH_QOS_TX_QUEUE_CNT; qInx++) {
		/* set Pause Time */
		MAC_QTFCR_PT_UdfWr(qInx, 0xffffU);
		/* Assign priority for RX flow control */
		/* Assign priority for TX flow control */
		switch(qInx) {
		case 0:
			MAC_TQPM0R_PSTQ0_UdfWr(0U);
			MAC_RQC2R_PSRQ0_UdfWr(0x1U << qInx);
			break;
		case 1:
			MAC_TQPM0R_PSTQ1_UdfWr(1U);
			MAC_RQC2R_PSRQ1_UdfWr(0x1U << qInx);
			break;
		case 2:
			MAC_TQPM0R_PSTQ2_UdfWr(2U);
			MAC_RQC2R_PSRQ2_UdfWr(0x1U << qInx);
			break;
		default :
			break;
		}

		enable_tx_flow_ctrl(qInx);
		/*disable_tx_flow_ctrl(qInx);*/
	}

	/* Set Rx flow control parameters */
	enable_rx_flow_ctrl();
	/*disable_rx_flow_ctrl();*/

	
	{
		MAC_MCR_JE_UdfWr(0x0U);
		MAC_MCR_WD_UdfWr(0x0U);
		MAC_MCR_GPSLCE_UdfWr(0x0U);
		MAC_MCR_JD_UdfWr(0x0U);
		/*printf("Disabled JUMBO pkt\n");*/
	}

#if ( TC9560_USE_TSB_DMA_WRAPPER == DEF_DISABLED ) /* Synopsys DMA 	*/
	/* update the MAC address */
	MAC_MA2HR_RgWr(((hw_addr[5] << 8)  | (hw_addr[4])));
	MAC_MA2LR_RgWr(((hw_addr[3] << 24) | (hw_addr[2] << 16) | (hw_addr[1] << 8) | (hw_addr[0])));
	MAC_MA2HR_AE_UdfWr(0x1);
	REG_WR((volatile uint32_t *)0x40003004,0x02FF0000);
#endif
		
	/*Enable MAC Transmit process */
	/*Enable MAC Receive process */
	/*Enable padding - disabled */
	/*Enable CRC stripping - disabled */
	MAC_MCR_RgRd(varMAC_MCR);
	varMAC_MCR = varMAC_MCR & (ULONG) (0xffcfff7cU);
	varMAC_MCR = varMAC_MCR | ((0x1) << 20) | ((0x1) << 21);

	MAC_MCR_RgWr(varMAC_MCR);

	MAC_MCR_IPC_UdfWr(0x1U);

#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
	configure_mac_for_vlan_pkt();
	/*config_vlan_filtering(1, 1, 0);*/
	config_vlan_filtering(0U, 1U, 0U);
#endif

	/* disable all MMC intterrupt as MMC are managed in SW and
	 * registers are cleared on each READ eventually
	 * */
	disable_mmc_interrupts();
	config_mmc_counters();

	enable_mac_interrupts();

	/* DBGPR("<--configure_mac\n");*/
	return Y_SUCCESS;
}
