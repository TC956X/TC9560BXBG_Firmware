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
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_cil.c $
 * $Revision: 1.10 $
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
 *
 * The Core Interface Layer provides basic services for accessing and
 * managing the DWC_otg hardware. These services are used by both the
 * Host Controller Driver and the Peripheral Controller Driver.
 *
 * The CIL manages the memory map for the core so that the HCD and PCD
 * don't have to do this separately. It also handles basic tasks like
 * reading/writing the registers and data FIFOs in the controller.
 * Some of the data access functions provide encapsulation of several
 * operations required to perform a task, such as writing multiple
 * registers to start a transfer. Finally, the CIL performs basic
 * services that are not specific to either the host or device modes
 * of operation. These services include management of the OTG Host
 * Negotiation Protocol (HNP) and Session Request Protocol (SRP). A
 * Diagnostic API is also provided to allow testing of the controller
 * hardware.
 *
 * The Core Interface Layer has the following requirements:
 * - Provides basic controller operations.
 * - Minimal use of OS services. 
 * - The OS services used will be abstracted by using inline functions
 *	 or macros.
 *
 */

#include "dwc_os.h"
#include "dwc_otg_regs.h"
#include "dwc_otg_cil.h"
#include "dwc_otg_driver.h"
#include "dwc_otg_dbg.h"
#include "tc9560_reg_define.h"

static int32_t dwc_otg_setup_params(dwc_otg_core_if_t * core_if);

/**
 * This function is called to initialize the DWC_otg CSR data
 * structures. The register addresses in the device and host
 * structures are initialized from the base address supplied by the
 * caller. The calling function must make the OS calls to get the
 * base address of the DWC_otg controller registers. The core_params
 * argument holds the parameters that specify how the core should be
 * configured.
 *
 * @param reg_base_addr Base address of DWC_otg core registers
 *
 */
dwc_otg_core_if_t *dwc_otg_cil_init(const uint32_t * reg_base_addr)
{
	dwc_otg_core_if_t *ptr_core_if = 0;
	dwc_otg_dev_if_t *ptr_dev_if = 0;
	uint8_t *reg_base = (uint8_t *) reg_base_addr;
	int32_t i = 0;


	DBG_USB_Print(DBG_CILV, "%s(%p)\n", __func__, reg_base_addr);

	ptr_core_if = DWC_ALLOC(sizeof(dwc_otg_core_if_t));

	if (ptr_core_if == NULL) {
		DBG_USB_Print(DBG_CIL,
			    "Allocation of dwc_otg_core_if_t failed\n");

	}
	else{
		ptr_core_if->core_global_regs = (dwc_otg_core_global_regs_t *) reg_base;

		/*
		 * Allocate the Device Mode structures.
		 */
		ptr_dev_if = DWC_ALLOC(sizeof(dwc_otg_dev_if_t));

		if (ptr_dev_if == NULL) {
			DBG_USB_Print(DBG_CIL, "Allocation of dwc_otg_dev_if_t failed\n");
			DWC_FREE(ptr_core_if);
		}
		else{
			ptr_dev_if->dev_global_regs =
				(dwc_otg_device_global_regs_t *) (reg_base +
								  DWC_DEV_GLOBAL_REG_OFFSET);

			for (i = 0; i < MAX_EPS_CHANNELS; i++) {
				ptr_dev_if->in_ep_regs[i] = (dwc_otg_dev_in_ep_regs_t *)
					(reg_base + DWC_DEV_IN_EP_REG_OFFSET +
					 (i * DWC_EP_REG_OFFSET));

				ptr_dev_if->out_ep_regs[i] = (dwc_otg_dev_out_ep_regs_t *)
					(reg_base + DWC_DEV_OUT_EP_REG_OFFSET +
					 (i * DWC_EP_REG_OFFSET));
				DBG_USB_Print(DBG_CILV, "in_ep_regs[%d]->diepctl=%p\n",
						i, &ptr_dev_if->in_ep_regs[i]->diepctl);
				DBG_USB_Print(DBG_CILV, "out_ep_regs[%d]->doepctl=%p\n",
						i, &ptr_dev_if->out_ep_regs[i]->doepctl);
			}

			ptr_dev_if->speed = 0;	/* unknown */

			ptr_core_if->dev_if = ptr_dev_if;

			ptr_core_if->pcgcctl = (uint32_t *) (reg_base + DWC_OTG_PCGCCTL_OFFSET);

			/* Initiate lx_state to L3 disconnected state */
			ptr_core_if->lx_state = DWC_OTG_L3;
			/*
			 * Store the contents of the hardware configuration registers here for
			 * easy access later.
			 */
			ptr_core_if->hwcfg1.d32 =
				REG_RD(&ptr_core_if->core_global_regs->ghwcfg1);
			ptr_core_if->hwcfg2.d32 =
				REG_RD(&ptr_core_if->core_global_regs->ghwcfg2);
			ptr_core_if->hwcfg3.d32 =
				REG_RD(&ptr_core_if->core_global_regs->ghwcfg3);
			ptr_core_if->hwcfg4.d32 =
				REG_RD(&ptr_core_if->core_global_regs->ghwcfg4);

			/* Force host mode to get HPTXFSIZ exact power on value */
			{
				/* gusbcfg_data_t gusbcfg = {.d32 = 0 }; */
				gusbcfg_data_t gbl_usb_cfg;
				gbl_usb_cfg.d32 = 0 ;
				gbl_usb_cfg.d32 =  REG_RD(&ptr_core_if->core_global_regs->gusbcfg);
				gbl_usb_cfg.b.force_host_mode = 1;
				REG_WR(&ptr_core_if->core_global_regs->gusbcfg, gbl_usb_cfg.d32);
				dwc_mdelay(100);
				ptr_core_if->hptxfsiz.d32 =
					REG_RD(&ptr_core_if->core_global_regs->hptxfsiz);
				gbl_usb_cfg.d32 =  REG_RD(&ptr_core_if->core_global_regs->gusbcfg);
				gbl_usb_cfg.b.force_host_mode = 0;
				REG_WR(&ptr_core_if->core_global_regs->gusbcfg, gbl_usb_cfg.d32);
				dwc_mdelay(100);
			}

			DBG_USB_Print(DBG_CILV, "hwcfg1=%08x\n", ptr_core_if->hwcfg1.d32);
			DBG_USB_Print(DBG_CILV, "hwcfg2=%08x\n", ptr_core_if->hwcfg2.d32);
			DBG_USB_Print(DBG_CILV, "hwcfg3=%08x\n", ptr_core_if->hwcfg3.d32);
			DBG_USB_Print(DBG_CILV, "hwcfg4=%08x\n", ptr_core_if->hwcfg4.d32);

			ptr_core_if->dcfg.d32 =
				REG_RD(&ptr_core_if->dev_if->dev_global_regs->dcfg);

			DBG_USB_Print(DBG_CILV, "hcfg=%08x\n", ptr_core_if->hcfg.d32);
			DBG_USB_Print(DBG_CILV, "dcfg=%08x\n", ptr_core_if->dcfg.d32);

			DBG_USB_Print(DBG_CILV, "op_mode=%0x\n", ptr_core_if->hwcfg2.b.op_mode);
			DBG_USB_Print(DBG_CILV, "arch=%0x\n", ptr_core_if->hwcfg2.b.architecture);
			DBG_USB_Print(DBG_CILV, "num_dev_ep=%d\n", ptr_core_if->hwcfg2.b.num_dev_ep);
			DBG_USB_Print(DBG_CILV, "num_host_chan=%d\n",
					ptr_core_if->hwcfg2.b.num_host_chan);
			DBG_USB_Print(DBG_CILV, "nonperio_tx_q_depth=0x%0x\n",
					ptr_core_if->hwcfg2.b.nonperio_tx_q_depth);
			DBG_USB_Print(DBG_CILV, "host_perio_tx_q_depth=0x%0x\n",
					ptr_core_if->hwcfg2.b.host_perio_tx_q_depth);
			DBG_USB_Print(DBG_CILV, "dev_token_q_depth=0x%0x\n",
					ptr_core_if->hwcfg2.b.dev_token_q_depth);

			DBG_USB_Print(DBG_CILV, "Total FIFO SZ=%d\n",
					ptr_core_if->hwcfg3.b.dfifo_depth);
			DBG_USB_Print(DBG_CILV, "xfer_size_cntr_width=%0x\n",
					ptr_core_if->hwcfg3.b.xfer_size_cntr_width);

			/*
			 * Set the SRP sucess bit for FS-I2c
			 */
			ptr_core_if->srp_success = 0;
			ptr_core_if->srp_timer_started = 0;

			ptr_core_if->snpsid = REG_RD(&ptr_core_if->core_global_regs->gsnpsid);

			DBG_USB_Print(DBG_USB,"Core Release: %x.%x%x%x\n",
				   (ptr_core_if->snpsid >> 12 & 0xF),
				   (ptr_core_if->snpsid >> 8 & 0xF),
				   (ptr_core_if->snpsid >> 4 & 0xF), (ptr_core_if->snpsid & 0xF));

			if (dwc_otg_setup_params(ptr_core_if) != 0) {
				DBG_Warn_Print("Error while setting core params\n");
			}

			ptr_core_if->hibernation_suspend = 0;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			if (ptr_core_if->otg_ver != 0) {
				ptr_core_if->test_mode = 0;
			}
#endif
		}
	}
	return ptr_core_if;
}

/**
 * This function frees the structures allocated by dwc_otg_cil_init().
 *
 * @param core_if The core interface pointer returned from
 * 		  dwc_otg_cil_init().
 *
 */
void dwc_otg_cil_remove(dwc_otg_core_if_t * core_if)
{
	dctl_data_t data_ctl;
	data_ctl.d32 = 0;
	
	/* Disable all interrupts */
	DWC_MODIFY_REG32(&core_if->core_global_regs->gahbcfg, 1, 0);
	REG_WR(&core_if->core_global_regs->gintmsk, 0);
DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	data_ctl.b.sftdiscon = 1;
	if (core_if->snpsid >= OTG_CORE_REV_3_00a) {
		DWC_MODIFY_REG32(&core_if->dev_if->dev_global_regs->dctl, 0,
				 data_ctl.d32);
	}

	if (core_if->dev_if != 0) {
		DWC_FREE(core_if->dev_if);
	}

	if (core_if->core_params != 0) {
		DWC_FREE(core_if->core_params);
	}
	DWC_FREE(core_if);
}

/**
 * This function enables the controller's Global Interrupt in the AHB Config
 * register.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
void dwc_otg_enable_global_interrupts(dwc_otg_core_if_t const * core_if)
{
	gahbcfg_data_t ahbcfg; 
	ahbcfg.d32 = 0 ;
	ahbcfg.b.glblintrmsk = 1;	/* Enable interrupts */
	DWC_MODIFY_REG32(&core_if->core_global_regs->gahbcfg, 0, ahbcfg.d32);
}

/**
 * This function disables the controller's Global Interrupt in the AHB Config
 * register.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
void dwc_otg_disable_global_interrupts(dwc_otg_core_if_t const * core_if)
{
	gahbcfg_data_t ahbcfg; 
	ahbcfg.d32 = 0 ;
	ahbcfg.b.glblintrmsk = 1;	/* Disable interrupts */
	DWC_MODIFY_REG32(&core_if->core_global_regs->gahbcfg, ahbcfg.d32, 0);
}

/**
 * This function initializes the commmon interrupts, used in both
 * device and host modes.
 *
 * @param core_if Programming view of the DWC_otg controller
 *
 */
static void dwc_otg_enable_common_interrupts(dwc_otg_core_if_t const * core_if)
{
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	gintmsk_data_t intr_mask; 
	intr_mask.d32 = 0 ;

	/* Clear any pending OTG Interrupts */
	REG_WR(&global_regs->gotgint, 0xFFFFFFFFU);

	/* Clear any pending interrupts */
	REG_WR(&global_regs->gintsts, 0xFFFFFFFFU);

	/*
	 * Enable the interrupts in the GINTMSK.
	 */
	if (!core_if->core_params->otg_ver) {
	/* To avoid system hang during OTG 2.0 role switch */
		intr_mask.b.modemismatch = 1;
	}
	intr_mask.b.otgintr = 1;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	if (!core_if->dma_enable) {
		intr_mask.b.rxstsqlvl = 1;
	}
#endif
	intr_mask.b.conidstschng = 1;
	intr_mask.b.wkupintr = 1;
	intr_mask.b.disconnect = 0;
	intr_mask.b.usbsuspend = 1;
	intr_mask.b.sessreqintr = 1;
	
	
	REG_WR(&global_regs->gintmsk, intr_mask.d32);
	/*REG_WR(&global_regs->gintmsk, 0x280e0000);  */  /* Modify register value based on Simulation sequence */
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
/*
 * The restore operation is modified to support Synopsys Emulated Powerdown and
 * Hibernation. This function is for exiting from Device mode hibernation by
 * Host Initiated Resume/Reset and Device Initiated Remote-Wakeup.
 * @param core_if Programming view of DWC_otg controller.
 * @param rem_wakeup - indicates whether resume is initiated by Device or Host.
 * @param reset - indicates whether resume is initiated by Reset.
 */
uint32_t dwc_otg_device_hibernation_restore(dwc_otg_core_if_t * core_if,
				       int32_t rem_wakeup, int32_t reset)
{
	int32_t flag = 0;
	gpwrdn_data_t gbl_pwr_dn; 
	pcgcctl_data_t pcgc_ctl;
	dctl_data_t data_ctl;
	int32_t time_out = 2000;
	gbl_pwr_dn.d32 = 0 ;
	pcgc_ctl.d32 = 0 ;
	data_ctl.d32 = 0 ;
	if (!core_if->hibernation_suspend) {
		DBG_USB_Print(DBG_USB,"Already exited from Hibernation\n");
		flag = 1;
	}
	if(flag == 0){
		DBG_USB_Print(DBG_PCD, "%s called\n", __FUNCTION__);
		/* Switch-on voltage to the core */
		gbl_pwr_dn.b.pwrdnswtch = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
		dwc_udelay(10);

		/* Reset core */
		gbl_pwr_dn.d32 = 0;
		gbl_pwr_dn.b.pwrdnrstn = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
		dwc_udelay(10);

		/* Assert Restore signal */
		gbl_pwr_dn.d32 = 0;
		gbl_pwr_dn.b.restore = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, 0, gbl_pwr_dn.d32);
		dwc_udelay(10);

		/* Disable power clamps */
		gbl_pwr_dn.d32 = 0;
		gbl_pwr_dn.b.pwrdnclmp = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);

		if (rem_wakeup != 0) {
			dwc_udelay(70);
		}

		/* Deassert Reset core */
		gbl_pwr_dn.d32 = 0;
		gbl_pwr_dn.b.pwrdnrstn = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, 0, gbl_pwr_dn.d32);
		dwc_udelay(10);

		/* Disable PMU interrupt */
		gbl_pwr_dn.d32 = 0;
		gbl_pwr_dn.b.pmuintsel = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);

		/* Mask interrupts from gpwrdn */
		gbl_pwr_dn.d32 = 0;
		gbl_pwr_dn.b.connect_det_msk = 1;
		gbl_pwr_dn.b.srp_det_msk = 1;
		gbl_pwr_dn.b.disconn_det_msk = 1;
		gbl_pwr_dn.b.rst_det_msk = 1;
		gbl_pwr_dn.b.lnstchng_msk = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);

		/* Indicates that we are going out from hibernation */
		core_if->hibernation_suspend = 0;

		/*
		 * Set Restore Essential Regs bit in PCGCCTL register, restore_mode = 1
		 * indicates restore from remote_wakeup
		 */
		restore_essential_regs(core_if, rem_wakeup, 0);

		/*
		 * Wait a little for seeing new value of variable hibernation_suspend if
		 * Restore done interrupt received before polling
		 */
		dwc_udelay(10);

		if (core_if->hibernation_suspend == 0) {
			/*
			 * Wait For Restore_done Interrupt. This mechanism of polling the 
			 * interrupt is introduced to avoid any possible race conditions
			 */
			do {
				gintsts_data_t gbl_int_sts;
				gbl_int_sts.d32 =
					REG_RD(&core_if->core_global_regs->gintsts);
				if (gbl_int_sts.b.restoredone != 0) {
					gbl_int_sts.d32 = 0;
					gbl_int_sts.b.restoredone = 1;
					REG_WR(&core_if->core_global_regs->
							gintsts, gbl_int_sts.d32);
					DBG_USB_Print(DBG_USB,"Restore Done Interrupt seen\n");
					break;
				}
				dwc_udelay(10);
				--time_out;
			} while (time_out != 0);
			if (!time_out) {
				DBG_USB_Print(DBG_USB,"Restore Done interrupt wasn't generated here\n");
			}
		}
		/* Clear all pending interupts */
		REG_WR(&core_if->core_global_regs->gintsts, 0xFFFFFFFFU);

		/* De-assert Restore */
		gbl_pwr_dn.d32 = 0;
		gbl_pwr_dn.b.restore = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
		dwc_udelay(10);

		if (!rem_wakeup) {
			pcgc_ctl.d32 = 0;
			pcgc_ctl.b.rstpdwnmodule = 1;
			DWC_MODIFY_REG32(core_if->pcgcctl, pcgc_ctl.d32, 0);
		}

		/* Restore GUSBCFG and DCFG */
		REG_WR(&core_if->core_global_regs->gusbcfg,
				core_if->gr_backup->gusbcfg_local);
		REG_WR(&core_if->dev_if->dev_global_regs->dcfg,
				core_if->dr_backup->dcfg);

		/* De-assert Wakeup Logic */
		gbl_pwr_dn.d32 = 0;
		gbl_pwr_dn.b.pmuactv = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
		dwc_udelay(10);

		if (!rem_wakeup) {
			/* Set Device programming done bit */
			data_ctl.b.pwronprgdone = 1;
			DWC_MODIFY_REG32(&core_if->dev_if->dev_global_regs->dctl, 0, data_ctl.d32);
		} else {
			/* Start Remote Wakeup Signaling */
			data_ctl.d32 = core_if->dr_backup->dctl;
			data_ctl.b.rmtwkupsig = 1;
			REG_WR(&core_if->dev_if->dev_global_regs->dctl, data_ctl.d32);
		}

		dwc_mdelay(2);
		/* Clear all pending interupts */
		REG_WR(&core_if->core_global_regs->gintsts, 0xFFFFFFFFU);

		/* Restore global registers */
		dwc_otg_restore_global_regs(core_if);
		/* Restore device global registers */
		dwc_otg_restore_dev_regs(core_if, rem_wakeup);

		if (rem_wakeup != 0) {
			dwc_mdelay(7);
			data_ctl.d32 = 0;
			data_ctl.b.rmtwkupsig = 1;
			DWC_MODIFY_REG32(&core_if->dev_if->dev_global_regs->dctl, data_ctl.d32, 0);
		}

		core_if->hibernation_suspend = 0;
		/* The core will be in ON STATE */
		core_if->lx_state = DWC_OTG_L0;
		DBG_USB_Print(DBG_USB,"Hibernation recovery completes here\n");
		(void)reset;
	}
	return 1;
}

/*
 * The restore operation is modified to support Synopsys Emulated Powerdown and
 * Hibernation. This function is for exiting from Host mode hibernation by
 * Host Initiated Resume/Reset and Device Initiated Remote-Wakeup.
 * @param core_if Programming view of DWC_otg controller.
 * @param rem_wakeup - indicates whether resume is initiated by Device or Host.
 * @param reset - indicates whether resume is initiated by Reset.
 */
int32_t dwc_otg_host_hibernation_restore(dwc_otg_core_if_t * core_if,
				     int32_t rem_wakeup, int32_t reset)
{
	gpwrdn_data_t gbl_pwr_dn; 
	int32_t time_out = 2000;
	gbl_pwr_dn.d32 = 0 ;


	DBG_USB_Print(DBG_HCD, "%s called\n", __FUNCTION__);
	/* Switch-on voltage to the core */
	gbl_pwr_dn.b.pwrdnswtch = 1;
	DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
	dwc_udelay(10);

	/* Reset core */
	gbl_pwr_dn.d32 = 0;
	gbl_pwr_dn.b.pwrdnrstn = 1;
	DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
	dwc_udelay(10);

	/* Assert Restore signal */
	gbl_pwr_dn.d32 = 0;
	gbl_pwr_dn.b.restore = 1;
	DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, 0, gbl_pwr_dn.d32);
	dwc_udelay(10);

	/* Disable power clamps */
	gbl_pwr_dn.d32 = 0;
	gbl_pwr_dn.b.pwrdnclmp = 1;
	DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);

	if (!rem_wakeup) {
		dwc_udelay(50);
	}

	/* Deassert Reset core */
	gbl_pwr_dn.d32 = 0;
	gbl_pwr_dn.b.pwrdnrstn = 1;
	DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, 0, gbl_pwr_dn.d32);
	dwc_udelay(10);

	/* Disable PMU interrupt */
	gbl_pwr_dn.d32 = 0;
	gbl_pwr_dn.b.pmuintsel = 1;
	DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);

	gbl_pwr_dn.d32 = 0;
	gbl_pwr_dn.b.connect_det_msk = 1;
	gbl_pwr_dn.b.srp_det_msk = 1;
	gbl_pwr_dn.b.disconn_det_msk = 1;
	gbl_pwr_dn.b.rst_det_msk = 1;
	gbl_pwr_dn.b.lnstchng_msk = 1;
	DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);

	/* Indicates that we are going out from hibernation */
	core_if->hibernation_suspend = 0;

	/* Set Restore Essential Regs bit in PCGCCTL register */
	restore_essential_regs(core_if, rem_wakeup, 1);

	/* Wait a little for seeing new value of variable hibernation_suspend if
	 * Restore done interrupt received before polling */
	dwc_udelay(10);

	if (core_if->hibernation_suspend == 0) {
		/* Wait For Restore_done Interrupt. This mechanism of polling the
		 * interrupt is introduced to avoid any possible race conditions
		 */
		do {
			gintsts_data_t gbl_int_sts;
			gbl_int_sts.d32 = REG_RD(&core_if->core_global_regs->gintsts);
			if ((gbl_int_sts.b.restoredone) != 0) {
				gbl_int_sts.d32 = 0;
				gbl_int_sts.b.restoredone = 1;
         		REG_WR(&core_if->core_global_regs->gintsts, gbl_int_sts.d32);
				DBG_USB_Print(DBG_HCD,"Restore Done Interrupt seen\n");	
				break;
			}
			dwc_udelay(10);
			--time_out;
		} while (time_out != 0);
		if (!time_out) {
			DBG_Warn_Print("Restore Done interrupt wasn't generated\n");
		}
	}
	/* Set the flag's value to 0 again after receiving restore done interrupt */
	core_if->hibernation_suspend = 0;

	/* This step is not described in functional spec but if not wait for this
	 * delay, mismatch interrupts occurred because just after restore core is
	 * in Device mode(gintsts.curmode == 0) */
	dwc_mdelay(100);

	/* Clear all pending interrupts */
	REG_WR(&core_if->core_global_regs->gintsts, 0xFFFFFFFFU);

	/* De-assert Restore */
	gbl_pwr_dn.d32 = 0;
	gbl_pwr_dn.b.restore = 1;
	DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
	dwc_udelay(10);

	/* Restore GUSBCFG and HCFG */
	REG_WR(&core_if->core_global_regs->gusbcfg,
			core_if->gr_backup->gusbcfg_local);

	/* De-assert Wakeup Logic */
	gbl_pwr_dn.d32 = 0;
	gbl_pwr_dn.b.pmuactv = 1;
	DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
	dwc_udelay(10);
	DBG_USB_Print(DBG_USB,"Resume Starts Now\n");

	/* Clear all pending interupts */
	REG_WR(&core_if->core_global_regs->gintsts, 0xFFFFFFFFU);

	/* Restore global registers */
	dwc_otg_restore_global_regs(core_if);

	/* The core will be in ON STATE */
	core_if->lx_state = DWC_OTG_L0;
	DBG_USB_Print(DBG_USB,"Hibernation recovery is complete here\n");
	(void)reset;
	return 0;
}

/** Saves some register values into system memory. */
int32_t dwc_otg_save_global_regs(dwc_otg_core_if_t * core_if)
{
	struct dwc_otg_global_regs_backup *gr;
	int32_t i;
	int32_t retval = 0;
	gr = core_if->gr_backup;
	if (!gr) {
		gr = DWC_ALLOC(sizeof(*gr));
		if (!gr) {
			retval = -DWC_E_NO_MEMORY;
		}
		else {
			core_if->gr_backup = gr;
		}
	}
	if(gr != 0){
		gr->gotgctl_local = REG_RD(&core_if->core_global_regs->gotgctl);
		gr->gintmsk_local = REG_RD(&core_if->core_global_regs->gintmsk);
		gr->gahbcfg_local = REG_RD(&core_if->core_global_regs->gahbcfg);
		gr->gusbcfg_local = REG_RD(&core_if->core_global_regs->gusbcfg);
		gr->grxfsiz_local = REG_RD(&core_if->core_global_regs->grxfsiz);
		gr->gnptxfsiz_local = REG_RD(&core_if->core_global_regs->gnptxfsiz);
		gr->hptxfsiz_local = REG_RD(&core_if->core_global_regs->hptxfsiz);
#ifdef CONFIG_USB_DWC_OTG_LPM
		gr->glpmcfg_local = REG_RD(&core_if->core_global_regs->glpmcfg);
#endif
		gr->gi2cctl_local = REG_RD(&core_if->core_global_regs->gi2cctl);
		gr->pcgcctl_local = REG_RD(core_if->pcgcctl);
		gr->gdfifocfg_local =
			REG_RD(&core_if->core_global_regs->gdfifocfg);
		for (i = 0; i < MAX_EPS_CHANNELS; i++) {
			gr->dtxfsiz_local[i] =
				REG_RD(&(core_if->core_global_regs->dtxfsiz[i]));
		}

		DBG_USB_Print(DBG_USB, "===========Backing Global registers==========\n");
		DBG_USB_Print(DBG_USB, "Backed up gotgctl   = %08x\n", gr->gotgctl_local);
		DBG_USB_Print(DBG_USB, "Backed up gintmsk   = %08x\n", gr->gintmsk_local);
		DBG_USB_Print(DBG_USB, "Backed up gahbcfg   = %08x\n", gr->gahbcfg_local);
		DBG_USB_Print(DBG_USB, "Backed up gusbcfg   = %08x\n", gr->gusbcfg_local);
		DBG_USB_Print(DBG_USB, "Backed up grxfsiz   = %08x\n", gr->grxfsiz_local);
		DBG_USB_Print(DBG_USB, "Backed up gnptxfsiz = %08x\n",
				gr->gnptxfsiz_local);
		DBG_USB_Print(DBG_USB, "Backed up hptxfsiz  = %08x\n",
				gr->hptxfsiz_local);
#ifdef CONFIG_USB_DWC_OTG_LPM
		DBG_USB_Print(DBG_USB, "Backed up glpmcfg   = %08x\n", gr->glpmcfg_local);
#endif
		DBG_USB_Print(DBG_USB, "Backed up gi2cctl   = %08x\n", gr->gi2cctl_local);
		DBG_USB_Print(DBG_USB, "Backed up pcgcctl   = %08x\n", gr->pcgcctl_local);
		DBG_USB_Print(DBG_USB,"Backed up gdfifocfg   = %08x\n",gr->gdfifocfg_local);
	}
	return retval;
}

int32_t dwc_otg_save_dev_regs(dwc_otg_core_if_t * core_if)
{
	struct dwc_otg_dev_regs_backup *dr=NULL;
	int32_t i;
	int32_t retval = 0;

	
	dr = core_if->dr_backup;
	if (!dr) {
		dr = DWC_ALLOC(sizeof(*dr));
		if (!dr) {
			retval = -DWC_E_NO_MEMORY;
		}
		else {
			core_if->dr_backup = dr;
		}
	}
	if(dr != 0) {
		dr->dcfg = REG_RD(&core_if->dev_if->dev_global_regs->dcfg);
		dr->dctl = REG_RD(&core_if->dev_if->dev_global_regs->dctl);
		dr->daintmsk =
			REG_RD(&core_if->dev_if->dev_global_regs->daintmsk);
		dr->diepmsk =
			REG_RD(&core_if->dev_if->dev_global_regs->diepmsk);
		dr->doepmsk =
			REG_RD(&core_if->dev_if->dev_global_regs->doepmsk);

		for (i = 0; i < core_if->dev_if->num_in_eps; ++i) {
			dr->diepctl[i] =
				REG_RD(&core_if->dev_if->in_ep_regs[i]->diepctl);
			dr->dieptsiz[i] =
				REG_RD(&core_if->dev_if->in_ep_regs[i]->dieptsiz);
			dr->diepdma[i] =
				REG_RD(&core_if->dev_if->in_ep_regs[i]->diepdma);
		}

		DBG_USB_Print(DBG_USB,
				"=============Backing Host registers==============\n");
		DBG_USB_Print(DBG_USB, "Backed up dcfg            = %08x\n", dr->dcfg);
		DBG_USB_Print(DBG_USB, "Backed up dctl        = %08x\n", dr->dctl);
		DBG_USB_Print(DBG_USB, "Backed up daintmsk            = %08x\n",
				dr->daintmsk);
		DBG_USB_Print(DBG_USB, "Backed up diepmsk        = %08x\n", dr->diepmsk);
		DBG_USB_Print(DBG_USB, "Backed up doepmsk        = %08x\n", dr->doepmsk);
		for (i = 0; i < core_if->dev_if->num_in_eps; ++i) {
			DBG_USB_Print(DBG_USB, "Backed up diepctl[%d]        = %08x\n", i,
					dr->diepctl[i]);
			DBG_USB_Print(DBG_USB, "Backed up dieptsiz[%d]        = %08x\n",
					i, dr->dieptsiz[i]);
			DBG_USB_Print(DBG_USB, "Backed up diepdma[%d]        = %08x\n", i,
					dr->diepdma[i]);
		}
	}
	return retval;
}

int32_t dwc_otg_restore_global_regs(dwc_otg_core_if_t * core_if)
{
	struct dwc_otg_global_regs_backup *gr;
	int32_t i;
	int32_t retval = 0;


	gr = core_if->gr_backup;
	if (!gr) {
		retval = -DWC_E_INVALID;
	}
	else{
		REG_WR(&core_if->core_global_regs->gotgctl, gr->gotgctl_local);
		REG_WR(&core_if->core_global_regs->gintmsk, gr->gintmsk_local);
		REG_WR(&core_if->core_global_regs->gusbcfg, gr->gusbcfg_local);
		REG_WR(&core_if->core_global_regs->gahbcfg, gr->gahbcfg_local);
		REG_WR(&core_if->core_global_regs->grxfsiz, gr->grxfsiz_local);
		REG_WR(&core_if->core_global_regs->gnptxfsiz,
				gr->gnptxfsiz_local);
		REG_WR(&core_if->core_global_regs->hptxfsiz,
				gr->hptxfsiz_local);
		REG_WR(&core_if->core_global_regs->gdfifocfg,
				gr->gdfifocfg_local);
		for (i = 0; i < MAX_EPS_CHANNELS; i++) {
			REG_WR(&core_if->core_global_regs->dtxfsiz[i],
					gr->dtxfsiz_local[i]);
		}

		REG_WR(&core_if->core_global_regs->gintsts, 0xFFFFFFFFU);
		REG_WR(&core_if->core_global_regs->gahbcfg,
				(gr->gahbcfg_local));
	}
	return retval;
}

int32_t dwc_otg_restore_dev_regs(dwc_otg_core_if_t * core_if, int32_t rem_wakeup)
{
	struct dwc_otg_dev_regs_backup *dr;
	int32_t i;
	int32_t retval = 0;

	dr = core_if->dr_backup;

	if (!dr) {
		retval = -DWC_E_INVALID;
	}
	else{
		if (!rem_wakeup) {
			REG_WR(&core_if->dev_if->dev_global_regs->dctl,
					dr->dctl);
		}
		
		REG_WR(&core_if->dev_if->dev_global_regs->daintmsk, dr->daintmsk);
		REG_WR(&core_if->dev_if->dev_global_regs->diepmsk, dr->diepmsk);
		REG_WR(&core_if->dev_if->dev_global_regs->doepmsk, dr->doepmsk);

		for (i = 0; i < core_if->dev_if->num_in_eps; ++i) {
			REG_WR(&core_if->dev_if->in_ep_regs[i]->dieptsiz, dr->dieptsiz[i]);
			REG_WR(&core_if->dev_if->in_ep_regs[i]->diepdma, dr->diepdma[i]);
			REG_WR(&core_if->dev_if->in_ep_regs[i]->diepctl, dr->diepctl[i]);
		}
	}
	return retval;
}

int32_t restore_lpm_i2c_regs(dwc_otg_core_if_t * core_if)
{
	struct dwc_otg_global_regs_backup *gr;

	gr = core_if->gr_backup;

	/* Restore values for LPM and I2C */
#ifdef CONFIG_USB_DWC_OTG_LPM
	REG_WR(&core_if->core_global_regs->glpmcfg, gr->glpmcfg_local);
#endif
	REG_WR(&core_if->core_global_regs->gi2cctl, gr->gi2cctl_local);

	return 0;
}

int32_t restore_essential_regs(dwc_otg_core_if_t * core_if, int32_t rmode, int32_t is_host)
{
	struct dwc_otg_global_regs_backup *gr;
	pcgcctl_data_t pcgc_ctl;
	gahbcfg_data_t gbl_ahb_cfg; 
	gusbcfg_data_t gbl_usb_cfg;
	gintmsk_data_t gbl_int_msk;
	dcfg_data_t data_cfg;
	gbl_ahb_cfg.d32 = 0 ;
	pcgc_ctl.d32 = 0 ;
	gbl_usb_cfg.d32 = 0 ;
	gbl_int_msk.d32 = 0 ;

	/* Restore LPM and I2C registers */
	restore_lpm_i2c_regs(core_if);

	/* Set PCGCCTL to 0 */
	REG_WR(core_if->pcgcctl, 0x00000000U);

	gr = core_if->gr_backup;
	/* Load restore values for [31:14] bits */
	REG_WR(core_if->pcgcctl,
			((gr->pcgcctl_local & 0xffffc000U) | 0x00020000U));

	/* Umnask global Interrupt in GAHBCFG and restore it */
	gbl_ahb_cfg.d32 = gr->gahbcfg_local;
	gbl_ahb_cfg.b.glblintrmsk = 1;
	REG_WR(&core_if->core_global_regs->gahbcfg, gbl_ahb_cfg.d32);

	/* Clear all pending interupts */
	REG_WR(&core_if->core_global_regs->gintsts, 0xFFFFFFFFU);

	/* Unmask restore done interrupt */
	gbl_int_msk.b.restoredone = 1;
	REG_WR(&core_if->core_global_regs->gintmsk, gbl_int_msk.d32);

	/* Restore GUSBCFG and HCFG/DCFG */
	gbl_usb_cfg.d32 = core_if->gr_backup->gusbcfg_local;
	REG_WR(&core_if->core_global_regs->gusbcfg, gbl_usb_cfg.d32);

		data_cfg.d32 = 0 ;
		data_cfg.d32 = core_if->dr_backup->dcfg;
		REG_WR(&core_if->dev_if->dev_global_regs->dcfg, data_cfg.d32);

		/* Load restore values for [31:14] bits */
		pcgc_ctl.d32 = gr->pcgcctl_local & 0xffffc000U;
		pcgc_ctl.d32 = gr->pcgcctl_local | 0x00020000U;
		if (!rmode) {
			pcgc_ctl.d32 |= 0x208U;
		}
		REG_WR(core_if->pcgcctl, pcgc_ctl.d32);
		dwc_udelay(10);

		/* Load restore values for [31:14] bits */
		pcgc_ctl.d32 = gr->pcgcctl_local & 0xffffc000U;
		pcgc_ctl.d32 = gr->pcgcctl_local | 0x00020000U;
		pcgc_ctl.b.ess_reg_restored = 1;
		if (!rmode) {
			pcgc_ctl.d32 |= 0x208U;
		}
		REG_WR(core_if->pcgcctl, pcgc_ctl.d32);
	(void)is_host;
	return 0;
}

#endif
/**
 * Initializes the DevSpd field of the DCFG register depending on the PHY type
 * and the enumeration speed of the device.
 */
static void init_devspd(dwc_otg_core_if_t const * core_if)
{
	uint32_t val;
	dcfg_data_t data_cfg;

	if (((core_if->hwcfg2.b.hs_phy_type == 2) &&
	     (core_if->hwcfg2.b.fs_phy_type == 1) &&
	     (core_if->core_params->ulpi_fs_ls)) ||
	    (core_if->core_params->phy_type == DWC_PHY_TYPE_PARAM_FS)) {
		/* Full speed PHY */
		val = 0x3;
	} else if (core_if->core_params->speed == DWC_SPEED_PARAM_FULL) {
		/* High speed PHY running at full speed */
		val = 0x1;
	} else {
		/* High speed PHY running at high speed */
		val = 0x0;
	}

	DBG_USB_Print(DBG_CIL, "Initializing DCFG.DevSpd to 0x%1x\n", val);

	data_cfg.d32 = REG_RD(&core_if->dev_if->dev_global_regs->dcfg);
	data_cfg.b.devspd = val;
	REG_WR(&core_if->dev_if->dev_global_regs->dcfg, data_cfg.d32);
}

/**
 * This function calculates the number of IN EPS
 * using GHWCFG1 and GHWCFG2 registers values
 *
 * @param core_if Programming view of the DWC_otg controller
 */
static uint32_t calc_num_in_eps(dwc_otg_core_if_t const * core_if)
{
	uint32_t uint_num_in_eps = 0;
	uint32_t uint_num_eps = core_if->hwcfg2.b.num_dev_ep;
	uint32_t hw_cfg1 = core_if->hwcfg1.d32 >> 3;
	uint32_t num_tx_fifos = core_if->hwcfg4.b.num_in_eps;
	uint32_t i,j;

	for (i = 0; i < uint_num_eps; ++i) {
		j = (hw_cfg1 & 0x1);
		if (!j) {
			uint_num_in_eps++;
		}

		hw_cfg1 >>= 2;
	}

	if ((core_if->hwcfg4.b.ded_fifo_en) != 0) {
		uint_num_in_eps =
		    (uint_num_in_eps > num_tx_fifos) ? num_tx_fifos : uint_num_in_eps;
	}

	return uint_num_in_eps;
}

/**
 * This function calculates the number of OUT EPS
 * using GHWCFG1 and GHWCFG2 registers values
 *
 * @param core_if Programming view of the DWC_otg controller
 */
static uint32_t calc_num_out_eps(dwc_otg_core_if_t const * core_if)
{
	uint32_t uint_num_out_eps = 0;
	uint32_t uint_num_eps = core_if->hwcfg2.b.num_dev_ep;
	uint32_t hw_cfg1 = core_if->hwcfg1.d32 >> 2;
	uint32_t i,j;

	for (i = 0; i < uint_num_eps; ++i) {
		j=hw_cfg1 & 0x1;
		if (!(j)) {
			uint_num_out_eps++;
		}

		hw_cfg1 >>= 2;
	}
	return uint_num_out_eps;
}

/**
 * This function initializes the DWC_otg controller registers and
 * prepares the core for device mode or host mode operation.
 *
 * @param core_if Programming view of the DWC_otg controller
 *
 */
void dwc_otg_core_init(dwc_otg_core_if_t * core_if)
{
	int32_t i = 0;
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	dwc_otg_dev_if_t *ptr_dev_if = core_if->dev_if;
	gahbcfg_data_t ahbcfg;
	gusbcfg_data_t usbcfg;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	gi2cctl_data_t i2cctl;
	i2cctl.d32 = 0 ;
#endif
	ahbcfg.d32 = 0 ;
	usbcfg.d32 = 0 ;

	DBG_USB_Print(DBG_CILV, "dwc_otg_core_init(%p)\n", core_if);

	/* Common Initialization */
	usbcfg.d32 = REG_RD(&global_regs->gusbcfg);

	/* Program the ULPI External VBUS bit if needed */
	usbcfg.b.ulpi_ext_vbus_drv =
	    (core_if->core_params->phy_ulpi_ext_vbus ==
	     DWC_PHY_ULPI_EXTERNAL_VBUS) ? 1 : 0;

	/* Set external TS Dline pulsing */
	usbcfg.b.term_sel_dl_pulse =
	    (core_if->core_params->ts_dline == 1) ? 1 : 0;
	REG_WR(&global_regs->gusbcfg, usbcfg.d32);

	/* Reset the Controller */
	dwc_otg_core_reset(core_if);

	core_if->adp_enable = core_if->core_params->adp_supp_enable;
	core_if->power_down = core_if->core_params->power_down;

	/* Initialize parameters from Hardware configuration registers. */
	ptr_dev_if->num_in_eps = (uint8_t)calc_num_in_eps(core_if);
	ptr_dev_if->num_out_eps = (uint8_t)calc_num_out_eps(core_if);

	DBG_USB_Print(DBG_CIL, "num_dev_perio_in_ep=%d\n",
		    core_if->hwcfg4.b.num_dev_perio_in_ep);

	for (i = 0; i < core_if->hwcfg4.b.num_dev_perio_in_ep; i++) {
		ptr_dev_if->perio_tx_fifo_size[i] = (uint16_t)
		    (REG_RD(&global_regs->dtxfsiz[i]) >> 16);
		DBG_USB_Print(DBG_CIL, "Periodic Tx FIFO SZ #%d=0x%0x\n",
			    i, ptr_dev_if->perio_tx_fifo_size[i]);
	}

	for (i = 0; i < core_if->hwcfg4.b.num_in_eps; i++) {
		ptr_dev_if->tx_fifo_size[i] =(uint16_t)
		    (REG_RD(&global_regs->dtxfsiz[i]) >> 16);
		DBG_USB_Print(DBG_CIL, "Tx FIFO SZ #%d=0x%0x\n",
			    i, ptr_dev_if->tx_fifo_size[i]);
	}

	core_if->total_fifo_size = (uint16_t)core_if->hwcfg3.b.dfifo_depth;
	core_if->rx_fifo_size = (uint16_t)REG_RD(&global_regs->grxfsiz);
	core_if->nperio_tx_fifo_size =(uint16_t)
	    (REG_RD(&global_regs->gnptxfsiz) >> 16);

	DBG_USB_Print(DBG_CIL, "Total FIFO SZ=%d\n", core_if->total_fifo_size);
	DBG_USB_Print(DBG_CIL, "Rx FIFO SZ=%d\n", core_if->rx_fifo_size);
	DBG_USB_Print(DBG_CIL, "NP Tx FIFO SZ=%d\n",
		    core_if->nperio_tx_fifo_size);

	/* This programming sequence needs to happen in FS mode before any other
	 * programming occurs */
	if ((core_if->core_params->speed == DWC_SPEED_PARAM_FULL) &&
	    (core_if->core_params->phy_type == DWC_PHY_TYPE_PARAM_FS)) {
		/* If FS mode with FS PHY */

		/* core_init() is now called on every switch so only call the
		 * following for the first time through. */
		if (!core_if->phy_init_done) {
			core_if->phy_init_done = 1;
			DBG_USB_Print(DBG_CIL, "FS_PHY detected\n");
			usbcfg.d32 = REG_RD(&global_regs->gusbcfg);
			usbcfg.b.physel = 1;
			REG_WR(&global_regs->gusbcfg, usbcfg.d32);

			/* Reset after a PHY select */
			dwc_otg_core_reset(core_if);
		}
			init_devspd(core_if);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		if ((core_if->core_params->i2c_enable) != 0) {
			DBG_USB_Print(DBG_CIL, "FS_PHY Enabling I2c\n");
			/* Program GUSBCFG.OtgUtmifsSel to I2C */
			usbcfg.d32 = REG_RD(&global_regs->gusbcfg);
			usbcfg.b.otgutmifssel = 1;
			REG_WR(&global_regs->gusbcfg, usbcfg.d32);

			/* Program GI2CCTL.I2CEn */
			i2cctl.d32 = REG_RD(&global_regs->gi2cctl);
			i2cctl.b.i2cdevaddr = 1;
			i2cctl.b.i2cen = 0;
			REG_WR(&global_regs->gi2cctl, i2cctl.d32);
			i2cctl.b.i2cen = 1;
			REG_WR(&global_regs->gi2cctl, i2cctl.d32);
		}
#endif
	} /* endif speed == DWC_SPEED_PARAM_FULL */
	else {
		/* High speed PHY. */
		if (!core_if->phy_init_done) {
			core_if->phy_init_done = 1;
			/* HS PHY parameters.  These parameters are preserved
			 * during soft reset so only program the first time.  Do
			 * a soft reset immediately after setting phyif.  */

			if (core_if->core_params->phy_type == 2) {
				/* ULPI interface */
				usbcfg.b.ulpi_utmi_sel = 1;
				usbcfg.b.phyif = 0;
				usbcfg.b.ddrsel =
				    core_if->core_params->phy_ulpi_ddr;
			} else if (core_if->core_params->phy_type == 1) {
				/* UTMI+ interface */
				usbcfg.b.ulpi_utmi_sel = 0;
				if (core_if->core_params->phy_utmi_width == 16) {
					usbcfg.b.phyif = 1;

				} else {
					usbcfg.b.phyif = 0;
				}
			} else {
				DBG_Error_Print("FS PHY TYPE\n");
			}
			REG_WR(&global_regs->gusbcfg, usbcfg.d32);
			/* Reset after setting the PHY parameters */
			dwc_otg_core_reset(core_if);
		}
	}
	if ((core_if->hwcfg2.b.hs_phy_type == 2) &&
	    (core_if->hwcfg2.b.fs_phy_type == 1) &&
	    (core_if->core_params->ulpi_fs_ls)) {
		DBG_USB_Print(DBG_CIL, "Setting ULPI FSLS\n");
		usbcfg.d32 = REG_RD(&global_regs->gusbcfg);
		usbcfg.b.ulpi_fsls = 1;
		usbcfg.b.ulpi_clk_sus_m = 1;
		REG_WR(&global_regs->gusbcfg, usbcfg.d32);
	} else {
		usbcfg.d32 = REG_RD(&global_regs->gusbcfg);
		usbcfg.b.ulpi_fsls = 0;
		usbcfg.b.ulpi_clk_sus_m = 0;
		REG_WR(&global_regs->gusbcfg, usbcfg.d32);
	}
	/* Program the GAHBCFG Register. */
	switch (core_if->hwcfg2.b.architecture) {

	case DWC_SLAVE_ONLY_ARCH:
		DBG_USB_Print(DBG_CIL, "Slave Only Mode\n");
		ahbcfg.b.nptxfemplvl_txfemplvl =
		    DWC_GAHBCFG_TXFEMPTYLVL_HALFEMPTY;
		ahbcfg.b.ptxfemplvl = DWC_GAHBCFG_TXFEMPTYLVL_HALFEMPTY;
		core_if->dma_enable = 0;
		core_if->dma_desc_enable = 0;
		break;
	case DWC_EXT_DMA_ARCH:
		DBG_USB_Print(DBG_CIL, "External DMA Mode\n");
		{
			uint32_t brst_sz = core_if->core_params->dma_burst_size;
			ahbcfg.b.hburstlen = 0;
			while (brst_sz > 1) {
				ahbcfg.b.hburstlen++;
				brst_sz >>= 1;
			}
		}
		core_if->dma_enable = (core_if->core_params->dma_enable != 0);
		core_if->dma_desc_enable =
		    (uint8_t)(core_if->core_params->dma_desc_enable != 0);
		break;
	case DWC_INT_DMA_ARCH:
		DBG_USB_Print(DBG_CIL, "Internal DMA Mode\n");
		/* Old value was DWC_GAHBCFG_INT_DMA_BURST_INCR - done for 
		   Host mode ISOC in issue fix - vahrama */
		ahbcfg.b.hburstlen = DWC_GAHBCFG_INT_DMA_BURST_INCR4;
		core_if->dma_enable = (core_if->core_params->dma_enable != 0);
		core_if->dma_desc_enable =
		    (uint8_t)(core_if->core_params->dma_desc_enable != 0);
		break;
	default :
		break;
	}
	if ((core_if->dma_enable) != 0) {
		if ((core_if->dma_desc_enable) != 0) {
			DBG_USB_Print(DBG_USB,"Using Descriptor DMA mode\n");
		}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
        else {
			DBG_USB_Print(DBG_USB,"Using Buffer DMA mode\n");
		}
#endif
	}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
    else {
		DBG_USB_Print(DBG_USB,"Using Slave mode\n");
		core_if->dma_desc_enable = 0;
	}
    
	if ((core_if->core_params->ahb_single) != 0) {
		ahbcfg.b.ahbsingle = 1;
	}
#endif
    
	core_if->en_multiple_tx_fifo = core_if->hwcfg4.b.ded_fifo_en;

	core_if->pti_enh_enable = (uint8_t)core_if->core_params->pti_enable != 0;
	core_if->multiproc_int_enable = (uint8_t)core_if->core_params->mpi_enable;
	DBG_USB_Print(DBG_USB,"Periodic Transfer Interrupt Enhancement - %s\n",
		   ((core_if->pti_enh_enable) ? "enabled" : "disabled"));
	DBG_USB_Print(DBG_USB,"Multiprocessor Interrupt Enhancement - %s\n",
		   ((core_if->multiproc_int_enable) ? "enabled" : "disabled"));
	/*
	 * Program the GUSBCFG register.
	 */
	usbcfg.d32 = REG_RD(&global_regs->gusbcfg);
	switch (core_if->hwcfg2.b.op_mode) {
	case DWC_MODE_HNP_SRP_CAPABLE:
		usbcfg.b.hnpcap = (core_if->core_params->otg_cap ==
				   DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE);
		usbcfg.b.srpcap = (core_if->core_params->otg_cap !=
				   DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
		break;
	case DWC_MODE_SRP_ONLY_CAPABLE:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = (core_if->core_params->otg_cap !=
				   DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
		break;
	case DWC_MODE_NO_HNP_SRP_CAPABLE:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = 0;
		break;
	case DWC_MODE_SRP_CAPABLE_DEVICE:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = (core_if->core_params->otg_cap !=
				   DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
		break;
	case DWC_MODE_NO_SRP_CAPABLE_DEVICE:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = 0;
		break;
	case DWC_MODE_SRP_CAPABLE_HOST:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = (core_if->core_params->otg_cap !=
				   DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
		break;
	case DWC_MODE_NO_SRP_CAPABLE_HOST:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = 0;
		break;
	default :
		break;
	}
	REG_WR(&global_regs->gusbcfg, usbcfg.d32);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	if ((core_if->core_params->ic_usb_cap) != 0) {
		gusbcfg_data_t gbl_usb_cfg;
		gbl_usb_cfg.d32 = 0 ;
		gbl_usb_cfg.b.ic_usb_cap = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gusbcfg,
				 0, gbl_usb_cfg.d32);
	}
#endif
	{
		gotgctl_data_t gbl_otg_ctl; 
		gbl_otg_ctl.d32 = 0 ;
		gbl_otg_ctl.b.otgver = core_if->core_params->otg_ver;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gotgctl, 0,
				 gbl_otg_ctl.d32);
		/* Set OTG version supported */
		core_if->otg_ver = core_if->core_params->otg_ver;
		DBG_USB_Print(DBG_USB,"OTG VER PARAM: %d, OTG VER FLAG: %d\n",
			   core_if->core_params->otg_ver, core_if->otg_ver);
	}
	/* Enable common interrupts */
	dwc_otg_enable_common_interrupts(core_if);

	/* Do device or host intialization based on mode during PCD
	 * and HCD initialization  */
	DBG_USB_Print(DBG_USB, "Device Mode\n");
	core_if->op_state = B_PERIPHERAL;
	dwc_otg_core_dev_init(core_if);
}

/**
 * This function enables the Device mode interrupts.
 *
 * @param core_if Programming view of DWC_otg controller
 */
void dwc_otg_enable_device_interrupts(dwc_otg_core_if_t const * core_if)
{
	gintmsk_data_t intr_mask;
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	intr_mask.d32 = 0 ;

	DBG_USB_Print(DBG_CIL, "%s()\n", __func__);

	/* Disable all interrupts. */
	REG_WR(&global_regs->gintmsk, 0);

	/* Clear any pending interrupts */
	REG_WR(&global_regs->gintsts, 0xFFFFFFFFU);

	/* Enable the common interrupts */
	dwc_otg_enable_common_interrupts(core_if);

	/* Enable interrupts */
	intr_mask.b.usbreset = 1;
	intr_mask.b.enumdone = 1;
	intr_mask.b.epmismatch = 1;

	/* Disable Disconnect interrupt in Device mode */
	intr_mask.b.disconnect = 0;

	if (!core_if->multiproc_int_enable) {
		intr_mask.b.inepintr = 1;
		intr_mask.b.outepintr = 1;
	}

	intr_mask.b.erlysuspend = 1;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	if (core_if->en_multiple_tx_fifo == 0) {
		intr_mask.b.epmismatch = 1;
	}
	if (!core_if->dma_desc_enable) {
		intr_mask.b.incomplisoin = 1;
	}
#endif
	DWC_MODIFY_REG32(&global_regs->gintmsk, intr_mask.d32, intr_mask.d32);

	DBG_USB_Print(DBG_CIL, "%s() gintmsk=%0x\n", __func__,
		    REG_RD(&global_regs->gintmsk));
}

/**
 * This function initializes the DWC_otg controller registers for
 * device mode.
 *
 * @param core_if Programming view of DWC_otg controller
 *
 */
void dwc_otg_core_dev_init(dwc_otg_core_if_t * core_if)
{
	int32_t i;
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	dwc_otg_dev_if_t *ptr_dev_if = core_if->dev_if;
	dwc_otg_core_params_t *params = core_if->core_params;
	dcfg_data_t data_cfg;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	depctl_data_t dev_ep_ctl;
	fifosize_data_t ptxfifosize;
	gotgctl_data_t gbl_otg_ctl;
#endif
	grstctl_t resetctl; 
	uint32_t uint_rx_fifo_size;
	fifosize_data_t nptxfifosize;
	fifosize_data_t txfifosize;
	dthrctl_data_t dthrctl;
	uint16_t rxfsiz, nptxfsiz;
	gdfifocfg_data_t gbl_data_fifo_cfg;
 
	pcgcctl_data_t pcgc_ctl;
	data_cfg.d32 = 0 ;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	dev_ep_ctl.d32 = 0 ;
	gbl_otg_ctl.d32 = 0 ;
#endif
	resetctl.d32 = 0 ;
	gbl_data_fifo_cfg.d32 = 0 ;


	/* Restart the Phy Clock */
	pcgc_ctl.d32 = 0 ;
	/* Restart the Phy Clock */
	pcgc_ctl.b.stoppclk = 1;
	DWC_MODIFY_REG32(core_if->pcgcctl, pcgc_ctl.d32, 0);
	dwc_udelay(10);

	/* Device configuration register */
	init_devspd(core_if);
	data_cfg.d32 = REG_RD(&ptr_dev_if->dev_global_regs->dcfg);
	data_cfg.b.descdma = ((core_if->dma_desc_enable) != 0) ? 1 : 0;
	data_cfg.b.perfrint = DWC_DCFG_FRAME_INTERVAL_80;
	/* Enable Device OUT NAK in case of DDMA mode */

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	if (core_if->core_params->dev_out_nak != 0) {
		data_cfg.b.endevoutnak = 1;
	}

	if (core_if->core_params->cont_on_bna != 0) {
		dctl_data_t data_ctl;
		data_ctl.d32 = 0 ;
		data_ctl.b.encontonbna = 1;
		DWC_MODIFY_REG32(&ptr_dev_if->dev_global_regs->dctl, 0, data_ctl.d32);
	}
	/** should be done before every reset */
	if (core_if->otg_ver != 0) {
		core_if->otg_sts = 0;
		gbl_otg_ctl.b.devhnpen = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gotgctl, gbl_otg_ctl.d32, 0);
	}
#endif	
	REG_WR(&ptr_dev_if->dev_global_regs->dcfg, data_cfg.d32);

	/* Configure data FIFO sizes */
	if ((((core_if->hwcfg2).b).dynamic_fifo) && (params->enable_dynamic_fifo)) {
		DBG_USB_Print(DBG_CIL, "Total FIFO Size=%d\n",
			    core_if->total_fifo_size);
		DBG_USB_Print(DBG_CIL, "Rx FIFO Size=%d\n",
			    params->dev_rx_fifo_size);
		DBG_USB_Print(DBG_CIL, "NP Tx FIFO Size=%d\n",
			    params->dev_nperio_tx_fifo_size);

		/* Rx FIFO */
		DBG_USB_Print(DBG_CIL, "initial grxfsiz=%08x\n",
			    REG_RD(&global_regs->grxfsiz));

		uint_rx_fifo_size = params->dev_rx_fifo_size;
		REG_WR(&global_regs->grxfsiz, uint_rx_fifo_size);
		REG_WR(&global_regs->grxfsiz, 0x00000215);    /* Fifo size configure based on sim sequence */

		DBG_USB_Print(DBG_CIL, "new grxfsiz=%08x\n",
			    REG_RD(&global_regs->grxfsiz));

		/** Set Periodic Tx FIFO Mask all bits 0 */
		core_if->p_tx_msk = 0;

		/** Set Tx FIFO Mask all bits 0 */
		core_if->tx_msk = 0;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		if (core_if->en_multiple_tx_fifo == 0) {
			/* Non-periodic Tx FIFO */
			DBG_USB_Print(DBG_CIL, "initial gnptxfsiz=%08x\n",
				    REG_RD(&global_regs->gnptxfsiz));

			nptxfifosize.b.depth = params->dev_nperio_tx_fifo_size;
			nptxfifosize.b.startaddr = params->dev_rx_fifo_size;

			REG_WR(&global_regs->gnptxfsiz,
					nptxfifosize.d32);

			DBG_USB_Print(DBG_CIL, "new gnptxfsiz=%08x\n",
				    REG_RD(&global_regs->gnptxfsiz));

			/*
			 * Periodic Tx FIFOs These FIFOs are numbered from 1 to 15.
			 * Indexes of the FIFO size module parameters in the
			 * dev_perio_tx_fifo_size array and the FIFO size registers in
			 * the dptxfsiz array run from 0 to 14.
			 */
			/** @todo Finish debug of this */
			ptxfifosize.b.startaddr =
			    nptxfifosize.b.startaddr + nptxfifosize.b.depth;
			for (i = 0; i < core_if->hwcfg4.b.num_dev_perio_in_ep; i++) {
				ptxfifosize.b.depth =
				    params->dev_perio_tx_fifo_size[i];
				DBG_USB_Print(DBG_CIL,
					    "initial dtxfsiz[%d]=%08x\n", i,
					    REG_RD(&global_regs->dtxfsiz
							   [i]));
				REG_WR(&global_regs->dtxfsiz[i],
						ptxfifosize.d32);
				DBG_USB_Print(DBG_CIL, "new dtxfsiz[%d]=%08x\n",
					    i,
					    REG_RD(&global_regs->dtxfsiz
							   [i]));
				ptxfifosize.b.startaddr += ptxfifosize.b.depth;
			}
		} else {
#endif
			/*
			 * Tx FIFOs These FIFOs are numbered from 1 to 15.
			 * Indexes of the FIFO size module parameters in the
			 * dev_tx_fifo_size array and the FIFO size registers in
			 * the dtxfsiz array run from 0 to 14.
			 */

			/* Non-periodic Tx FIFO */
			DBG_USB_Print(DBG_CIL, "initial gnptxfsiz=%08x\n",
				    REG_RD(&global_regs->gnptxfsiz));

			nptxfifosize.b.depth = params->dev_nperio_tx_fifo_size;
			nptxfifosize.b.startaddr = params->dev_rx_fifo_size;

			REG_WR(&global_regs->gnptxfsiz,
					nptxfifosize.d32);

			REG_WR(&global_regs->gnptxfsiz,0x00100216); 
			DBG_USB_Print(DBG_CIL, "new gnptxfsiz=%08x\n",
				    REG_RD(&global_regs->gnptxfsiz));

			txfifosize.b.startaddr =
			    nptxfifosize.b.startaddr + nptxfifosize.b.depth;

			for (i = 0; i < core_if->hwcfg4.b.num_in_eps; i++) {

				txfifosize.b.depth =
				    params->dev_tx_fifo_size[i];

				DBG_USB_Print(DBG_CIL,
					    "initial dtxfsiz[%d]=%08x\n",
					    i,
					    REG_RD(&global_regs->dtxfsiz
							   [i]));

				REG_WR(&global_regs->dtxfsiz[i],
						txfifosize.d32);

				DBG_USB_Print(DBG_CIL,
					    "new dtxfsiz[%d]=%08x\n",
					    i,
					    REG_RD(&global_regs->dtxfsiz
							   [i]));

				txfifosize.b.startaddr += txfifosize.b.depth;
			}
			/* Calculating DFIFOCFG for Device mode to include RxFIFO and NPTXFIFO 
			 * Before 3.00a EpInfoBase was being configured in ep enable/disable 
			 * routine as well. Starting from 3.00a it will be set to the end of
			 * allocated FIFO space here due to ep 0 OUT always keeping enabled
			 */
			gbl_data_fifo_cfg.d32 = REG_RD(&global_regs->gdfifocfg);
			gbl_data_fifo_cfg.b.gdfifocfg = (REG_RD(&global_regs->ghwcfg3) >> 16);
			REG_WR(&global_regs->gdfifocfg, gbl_data_fifo_cfg.d32);
			if (core_if->snpsid <= OTG_CORE_REV_2_94a) {
				rxfsiz = (uint16_t)(REG_RD(&global_regs->grxfsiz) & 0x0000ffff);
				nptxfsiz = (uint16_t)(REG_RD(&global_regs->gnptxfsiz) >> 16);
				gbl_data_fifo_cfg.b.epinfobase = rxfsiz + nptxfsiz;
			} else {
				gbl_data_fifo_cfg.b.epinfobase = txfifosize.b.startaddr;
			}
			REG_WR(&global_regs->gdfifocfg, gbl_data_fifo_cfg.d32);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		}
#endif
	}

	/* Flush the FIFOs */
	dwc_otg_flush_tx_fifo(core_if, 0x10);	/* all Tx FIFOs */
	dwc_otg_flush_rx_fifo(core_if);

	/* Flush the Learning Queue. */
	resetctl.b.intknqflsh = 1;
	REG_WR(&core_if->core_global_regs->grstctl, resetctl.d32);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	if ((!(core_if->core_params->en_multiple_tx_fifo)) && (core_if->dma_enable)) {
		core_if->start_predict = 0;
		for (i = 0; i <= core_if->dev_if->num_in_eps; ++i) {
			core_if->nextep_seq[i] = 0xff;	/* 0xff - EP not active */
		}
		core_if->nextep_seq[0] = 0;
		core_if->first_in_nextep_seq = 0;
		dev_ep_ctl.d32 = REG_RD(&ptr_dev_if->in_ep_regs[0]->diepctl);
		dev_ep_ctl.b.nextep = 0;
		REG_WR(&ptr_dev_if->in_ep_regs[0]->diepctl, dev_ep_ctl.d32);

		/* Update IN Endpoint Mismatch Count by active IN NP EP count + 1 */
		data_cfg.d32 = REG_RD(&ptr_dev_if->dev_global_regs->dcfg);
		data_cfg.b.epmscnt = 2;
		REG_WR(&ptr_dev_if->dev_global_regs->dcfg, data_cfg.d32);

		DBG_USB_Print(DBG_CILV,
			    "%s first_in_nextep_seq= %2d; nextep_seq[]:\n",
			    __func__, core_if->first_in_nextep_seq);
		for (i = 0; i <= core_if->dev_if->num_in_eps; i++) {
			DBG_USB_Print(DBG_CILV, "%2d ", core_if->nextep_seq[i]);
		}
		DBG_USB_Print(DBG_CILV, "\n");
	}
#endif
	/* Clear all pending Device Interrupts */
	/** @todo - if the condition needed to be checked
	 *  or in any case all pending interrutps should be cleared?
     */
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	if (core_if->multiproc_int_enable != 0) {
		for (i = 0; i < core_if->dev_if->num_in_eps; ++i) {
			REG_WR(&ptr_dev_if->dev_global_regs->
					diepeachintmsk[i], 0);
		}

		for (i = 0; i < core_if->dev_if->num_out_eps; ++i) {
			REG_WR(&ptr_dev_if->dev_global_regs->
					doepeachintmsk[i], 0);
		}
		REG_WR(&ptr_dev_if->dev_global_regs->deachint, 0xFFFFFFFFU);
		REG_WR(&ptr_dev_if->dev_global_regs->deachintmsk, 0);
	} else {
#endif
		REG_WR(&ptr_dev_if->dev_global_regs->diepmsk, 0);
		REG_WR(&ptr_dev_if->dev_global_regs->diepmsk, 0xFFFFFFFFU);
		REG_WR(&ptr_dev_if->dev_global_regs->doepmsk, 0);
		REG_WR(&ptr_dev_if->dev_global_regs->doepmsk, 0xFFFFFFFFU);
		REG_WR(&ptr_dev_if->dev_global_regs->daint, 0xFFFFFFFFU);
		REG_WR(&ptr_dev_if->dev_global_regs->daintmsk, 0);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	}
#endif

	for (i = 0; i <= ptr_dev_if->num_in_eps; i++) {
		depctl_data_t depctl;
		depctl.d32 = REG_RD(&ptr_dev_if->in_ep_regs[i]->diepctl);
		if (depctl.b.epena != 0) {
			depctl.d32 = 0;
			depctl.b.epdis = 1;
			depctl.b.snak = 1;
		} else {
			depctl.d32 = 0;
		}

		REG_WR(&ptr_dev_if->in_ep_regs[i]->diepctl, depctl.d32);

		REG_WR(&ptr_dev_if->in_ep_regs[i]->dieptsiz, 0);
		REG_WR(&ptr_dev_if->in_ep_regs[i]->diepdma, 0);
		REG_WR(&ptr_dev_if->in_ep_regs[i]->diepint, 0xFF);
	}
	for (i = 1; i <= ptr_dev_if->num_out_eps; i++) {
		depctl_data_t depctl;
		depctl.d32 = REG_RD(&ptr_dev_if->out_ep_regs[i]->doepctl);
		if (depctl.b.epena != 0) {
			int32_t j = 0;
			dctl_data_t data_ctl;
			gintmsk_data_t gbl_int_sts;
			doepint_data_t dev_oep_int;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			device_grxsts_data_t dev_rec_status;
#endif
			data_ctl.d32 = 0 ;
			gbl_int_sts.d32 = 0 ;
			dev_oep_int.d32 = 0 ;
			data_ctl.b.sgoutnak = 1;
			DWC_MODIFY_REG32(&core_if->dev_if->dev_global_regs->dctl, 0, data_ctl.d32);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			if (!core_if->dma_enable) {
				do {
					j++;
					/* dwc_udelay(10); */
					gbl_int_sts.d32 = REG_RD(&core_if->core_global_regs->gintsts);
					if (j == 100000) {
						DBG_Error_Print("SNAK is not set during 10s\n");
						break;
					}
				} while (!gbl_int_sts.b.rxstsqlvl);
				dev_rec_status.d32 = REG_RD(&global_regs->grxstsp);
				if (dev_rec_status.b.pktsts == DWC_DSTS_GOUT_NAK)
					DBG_USB_Print(DBG_PCDV, "Global OUT NAK\n");
				gbl_int_sts.d32 = 0;
				gbl_int_sts.b.rxstsqlvl = 1;
				REG_WR(&global_regs->gintsts, gbl_int_sts.d32);
			}
#endif
			j = 0;
			do {
				j++;
				/*dwc_udelay(10); */
				gbl_int_sts.d32 = REG_RD(&core_if->core_global_regs->gintsts);
				if (j == 100000) {
					DBG_Error_Print("SNAK is not set during 10s\n");
					break;
				}
			} while (!gbl_int_sts.b.goutnakeff);
			gbl_int_sts.d32 = 0;
			gbl_int_sts.b.goutnakeff = 1;
			REG_WR(&core_if->core_global_regs->gintsts, gbl_int_sts.d32);

			depctl.d32 = 0;
			depctl.b.epdis = 1;
			depctl.b.snak = 1;
			j = 0;
			REG_WR(&core_if->dev_if->out_ep_regs[i]->doepctl, depctl.d32);
			do {
				/* dwc_udelay(10); */
				j++;
				dev_oep_int.d32 = REG_RD(&core_if->dev_if->
					out_ep_regs[i]->doepint);
				if (j == 100000) {
					DBG_Error_Print("EPDIS was not set during 10s\n");
					break;
				}
			} while (!dev_oep_int.b.epdisabled);

			dev_oep_int.b.epdisabled = 1;
			REG_WR(&core_if->dev_if->out_ep_regs[i]->doepint, dev_oep_int.d32);

			data_ctl.d32 = 0;
			data_ctl.b.cgoutnak = 1;
			DWC_MODIFY_REG32(&core_if->dev_if->dev_global_regs->dctl, 0, data_ctl.d32);
		} else {
			depctl.d32 = 0;
		}

		REG_WR(&ptr_dev_if->out_ep_regs[i]->doepctl, depctl.d32);
		REG_WR(&ptr_dev_if->out_ep_regs[i]->doeptsiz, 0);
		REG_WR(&ptr_dev_if->out_ep_regs[i]->doepdma, 0);
		REG_WR(&ptr_dev_if->out_ep_regs[i]->doepint, 0xFF);
	}
	if ((core_if->en_multiple_tx_fifo) && (core_if->dma_enable)) {
		ptr_dev_if->non_iso_tx_thr_en = (uint16_t)params->thr_ctl & 0x1;
		ptr_dev_if->rx_thr_en = (uint16_t)(params->thr_ctl >> 2) & 0x1;

		ptr_dev_if->rx_thr_length = (uint16_t)params->rx_thr_length;
		ptr_dev_if->tx_thr_length = (uint16_t)params->tx_thr_length;

		ptr_dev_if->setup_desc_index = 0;

		dthrctl.d32 = 0;
		dthrctl.b.non_iso_thr_en = ptr_dev_if->non_iso_tx_thr_en;
		dthrctl.b.tx_thr_len = ptr_dev_if->tx_thr_length;
		dthrctl.b.rx_thr_en = ptr_dev_if->rx_thr_en;
		dthrctl.b.rx_thr_len = ptr_dev_if->rx_thr_length;
		dthrctl.b.ahb_thr_ratio = params->ahb_thr_ratio;

		REG_WR(&ptr_dev_if->dev_global_regs->dtknqr3_dthrctl,
				dthrctl.d32);

		DBG_USB_Print(DBG_CIL,
			    "Non ISO Tx Thr - %d\nRx Thr - %d\nTx Thr Len - %d\nRx Thr Len - %d\n",
			    dthrctl.b.non_iso_thr_en, 
			    dthrctl.b.rx_thr_en, dthrctl.b.tx_thr_len,
			    dthrctl.b.rx_thr_len);

	}
	dwc_otg_enable_device_interrupts(core_if);

	{
		diepmsk_data_t msk; 
		msk.d32 = 0 ;
		msk.b.txfifoundrn = 1;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		if (core_if->multiproc_int_enable != 0) {
			DWC_MODIFY_REG32(&ptr_dev_if->dev_global_regs->
					 diepeachintmsk[0], msk.d32, msk.d32);
		} else {
#endif
			DWC_MODIFY_REG32(&ptr_dev_if->dev_global_regs->diepmsk,
					 msk.d32, msk.d32);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		}
#endif
	}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	if (core_if->multiproc_int_enable != 0) {
		/* Set NAK on Babble */
		dctl_data_t data_ctl; 
		data_ctl.d32 = 0 ;
		data_ctl.b.nakonbble = 1;
		DWC_MODIFY_REG32(&ptr_dev_if->dev_global_regs->dctl, 0, data_ctl.d32);
	}
#endif
	if (core_if->snpsid >= OTG_CORE_REV_2_94a) {
		dctl_data_t data_ctl;
		data_ctl.d32 = 0 ;
		data_ctl.d32 = REG_RD(&ptr_dev_if->dev_global_regs->dctl);
		data_ctl.b.sftdiscon = 0;
		REG_WR(&ptr_dev_if->dev_global_regs->dctl, data_ctl.d32);
	}
}

/**
 * Gets the current USB frame number. This is the frame number from the last
 * SOF packet.
 */
uint32_t dwc_otg_get_frame_number(dwc_otg_core_if_t const * core_if)
{
	dsts_data_t data_sts;
	data_sts.d32 = REG_RD(&core_if->dev_if->dev_global_regs->dsts);

	/* read current frame/microframe number from DSTS register */
	return data_sts.b.soffn;
}

/**
 * This function enables EP0 OUT to receive SETUP packets and configures EP0
 * IN for transmitting packets. It is normally called when the
 * "Enumeration Done" interrupt occurs.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP0 data.
 */
void dwc_otg_ep0_activate(dwc_otg_core_if_t const * core_if, dwc_ep_t * ep)
{
	dwc_otg_dev_if_t *ptr_dev_if = core_if->dev_if;
	dsts_data_t data_sts;
	depctl_data_t dev_ep_ctl;
	depctl_data_t dev_oep_ctl;
	dctl_data_t data_ctl;
	data_ctl.d32 = 0 ;

	ep->stp_rollover = 0;
	/* Read the Device Status and Endpoint 0 Control registers */
	data_sts.d32 = REG_RD(&ptr_dev_if->dev_global_regs->dsts);
	dev_ep_ctl.d32 = REG_RD(&ptr_dev_if->in_ep_regs[0]->diepctl);
	dev_oep_ctl.d32 = REG_RD(&ptr_dev_if->out_ep_regs[0]->doepctl);

	/* Set the MPS of the IN EP based on the enumeration speed */
	switch (data_sts.b.enumspd) {
	case DWC_DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ:
	case DWC_DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ:
	case DWC_DSTS_ENUMSPD_FS_PHY_48MHZ:
		dev_ep_ctl.b.mps = DWC_DEP0CTL_MPS_64;
		break;
	case DWC_DSTS_ENUMSPD_LS_PHY_6MHZ:
		dev_ep_ctl.b.mps = DWC_DEP0CTL_MPS_8;
		break;
	default :
		break;
	}

	REG_WR(&ptr_dev_if->in_ep_regs[0]->diepctl, dev_ep_ctl.d32);

	/* Enable OUT EP for receive */
	if (core_if->snpsid <= OTG_CORE_REV_2_94a) {
		dev_oep_ctl.b.epena = 1;
		REG_WR(&ptr_dev_if->out_ep_regs[0]->doepctl, dev_oep_ctl.d32);
	}
#ifdef VERBOSE
	DBG_USB_Print(DBG_PCDV, "doepctl0=%0x\n",
		    REG_RD(&dev_if->out_ep_regs[0]->doepctl));
	DBG_USB_Print(DBG_PCDV, "diepctl0=%0x\n",
		    REG_RD(&dev_if->in_ep_regs[0]->diepctl));
#endif
	data_ctl.b.cgnpinnak = 1;

	DWC_MODIFY_REG32(&ptr_dev_if->dev_global_regs->dctl, data_ctl.d32, data_ctl.d32);
	DBG_USB_Print(DBG_PCDV, "dctl=%0x\n",
		    REG_RD(&ptr_dev_if->dev_global_regs->dctl));

}

/**
 * This function activates an EP.  The Device EP control register for
 * the EP is configured as defined in the ep structure. Note: This
 * function is not used for EP0.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to activate.
 */
void dwc_otg_ep_activate(dwc_otg_core_if_t * core_if, dwc_ep_t * ep)
{
	dwc_otg_dev_if_t *ptr_dev_if = core_if->dev_if;
	depctl_data_t depctl;
	volatile uint32_t *ptr_addr;
	daint_data_t dev_all_int_msk;
	dcfg_data_t data_cfg;
	USIGN j;
	uint8_t i;
	dev_all_int_msk.d32 = 0 ;

	DBG_USB_Print(DBG_PCDV, "%s() EP%d-%s\n", __func__, ep->num,
		    (ep->is_in ? "IN" : "OUT"));

	/* Read DEPCTLn register */
	if (ep->is_in == 1) {
		ptr_addr = &ptr_dev_if->in_ep_regs[ep->num]->diepctl;
		dev_all_int_msk.ep.in = 1 << ep->num;
	} else {
		ptr_addr = &ptr_dev_if->out_ep_regs[ep->num]->doepctl;
		dev_all_int_msk.ep.out = 1 << ep->num;
	}

	/* If the EP is already active don't change the EP Control
	 * register. */
	depctl.d32 = REG_RD(ptr_addr);
	if (!depctl.b.usbactep) {
		depctl.b.mps = ep->maxpacket;
		depctl.b.eptype = ep->type;
		depctl.b.txfnum = ep->tx_fifo_num;

			depctl.b.setd0pid = 1;

		depctl.b.usbactep = 1;

		/* Update nextep_seq array and EPMSCNT in DCFG */
		j = (depctl.b.eptype & 1);
		if ((!j) && (ep->is_in == 1)) {	/* NP IN EP */
			for (i = 0; i <= core_if->dev_if->num_in_eps; i++) {
				if (core_if->nextep_seq[i] == core_if->first_in_nextep_seq) {
					break;
				}
			}
			core_if->nextep_seq[i] = ep->num;
			core_if->nextep_seq[ep->num] = core_if->first_in_nextep_seq;
			depctl.b.nextep = core_if->nextep_seq[ep->num];
			data_cfg.d32 = REG_RD(&ptr_dev_if->dev_global_regs->dcfg);
			data_cfg.b.epmscnt++;
			REG_WR(&ptr_dev_if->dev_global_regs->dcfg, data_cfg.d32);

			DBG_USB_Print(DBG_PCDV,
				    "%s first_in_nextep_seq= %2d; nextep_seq[]:\n",
				    __func__, core_if->first_in_nextep_seq);
			for (i = 0; i <= core_if->dev_if->num_in_eps; i++) {
				DBG_USB_Print(DBG_PCDV, "%2d\n",
					    core_if->nextep_seq[i]);
			}

		}


		REG_WR(ptr_addr, depctl.d32);
		DBG_USB_Print(DBG_PCDV, "DEPCTL=%08x\n", REG_RD(ptr_addr));
	}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	/* Enable the Interrupt for this EP */
	if (core_if->multiproc_int_enable != 0) {
		if (ep->is_in == 1) {
			diepmsk_data_t dev_ep_msk;
			dev_ep_msk.d32 = 0 ;
			dev_ep_msk.b.xfercompl = 1;
			dev_ep_msk.b.timeout = 1;
			dev_ep_msk.b.epdisabled = 1;
			dev_ep_msk.b.ahberr = 1;
			dev_ep_msk.b.intknepmis = 1;
			if ((!(core_if->en_multiple_tx_fifo)) && (core_if->dma_enable)) {
				dev_ep_msk.b.intknepmis = 0;
			}
			dev_ep_msk.b.txfifoundrn = 1;	
			REG_WR(&ptr_dev_if->dev_global_regs->
					diepeachintmsk[ep->num], dev_ep_msk.d32);
		} else {
			doepmsk_data_t dev_oep_msk;
			dev_oep_msk.d32 = 0 ;
			dev_oep_msk.b.xfercompl = 1;
			dev_oep_msk.b.ahberr = 1;
			dev_oep_msk.b.epdisabled = 1;
			REG_WR(&ptr_dev_if->dev_global_regs->
					doepeachintmsk[ep->num], dev_oep_msk.d32);
		}
		DWC_MODIFY_REG32(&ptr_dev_if->dev_global_regs->deachintmsk,
				 0, dev_all_int_msk.d32);
	} else {
#endif
		DWC_MODIFY_REG32(&ptr_dev_if->dev_global_regs->daintmsk,
				 0, dev_all_int_msk.d32);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	}
#endif
	DBG_USB_Print(DBG_PCDV, "DAINTMSK=%0x\n",
		    REG_RD(&ptr_dev_if->dev_global_regs->daintmsk));
	ep->stall_clear_flag = 0;
	return;
}

/**
 * This function deactivates an EP. This is done by clearing the USB Active
 * EP bit in the Device EP control register. Note: This function is not used
 * for EP0. EP0 cannot be deactivated.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to deactivate.
 */
void dwc_otg_ep_deactivate(dwc_otg_core_if_t * core_if, dwc_ep_t const * ep)
{
	depctl_data_t depctl;
	volatile uint32_t *ptr_addr;
	daint_data_t dev_all_int_msk;
	uint8_t i = 0;
	USIGN j;
	dcfg_data_t dev_cfg;
	depctl.d32 = 0;
	dev_all_int_msk.d32 = 0 ;
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	/* Read DEPCTLn register */
	if (ep->is_in == 1) {
		ptr_addr = &core_if->dev_if->in_ep_regs[ep->num]->diepctl;
		dev_all_int_msk.ep.in = 1 << ep->num;
	} else {
		ptr_addr = &core_if->dev_if->out_ep_regs[ep->num]->doepctl;
		dev_all_int_msk.ep.out = 1 << ep->num;
	}
	depctl.d32 = REG_RD(ptr_addr);
	depctl.b.usbactep = 0;
	j = (((depctl.b).eptype) & 1);
	/* Update nextep_seq array and EPMSCNT in DCFG */
	if ((!(j)) && ((ep->is_in) == 1)) {	/* NP EP IN */
		for (i = 0; i <= core_if->dev_if->num_in_eps; i++) {
			if (core_if->nextep_seq[i] == ep->num) {
				break;
			}
		}
		core_if->nextep_seq[i] = core_if->nextep_seq[ep->num];
		if (core_if->first_in_nextep_seq == ep->num) {
			core_if->first_in_nextep_seq = i;
		}
		core_if->nextep_seq[ep->num] = 0xff;
		depctl.b.nextep = 0;
		dev_cfg.d32 =
		    REG_RD(&core_if->dev_if->dev_global_regs->dcfg);
		dev_cfg.b.epmscnt--;
		REG_WR(&core_if->dev_if->dev_global_regs->dcfg,
				dev_cfg.d32);

		DBG_USB_Print(DBG_PCDV,
			    "%s first_in_nextep_seq= %2d; nextep_seq[]:\n",
			    __func__, core_if->first_in_nextep_seq);
		for (i = 0; i <= core_if->dev_if->num_in_eps; i++) {
			DBG_USB_Print(DBG_PCDV, "%2d\n", core_if->nextep_seq[i]);
		}
	}
	if (ep->is_in == 1) {
		depctl.b.txfnum = 0;
	}
	if (core_if->dma_desc_enable != 0) {
		depctl.b.epdis = 1;
	}
	REG_WR(ptr_addr, depctl.d32);
	depctl.d32 = REG_RD(ptr_addr);
	/* Disable the Interrupt for this EP */
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	if (core_if->multiproc_int_enable != 0) {
		DWC_MODIFY_REG32(&core_if->dev_if->dev_global_regs->deachintmsk,
				 dev_all_int_msk.d32, 0);

		if (ep->is_in == 1) {
			REG_WR(&core_if->dev_if->dev_global_regs->
					diepeachintmsk[ep->num], 0);
		} else {
			REG_WR(&core_if->dev_if->dev_global_regs->
					doepeachintmsk[ep->num], 0);
		}
	} else {
#endif
		DWC_MODIFY_REG32(&core_if->dev_if->dev_global_regs->daintmsk,
				 dev_all_int_msk.d32, 0);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	}
#endif
}

/**
 * This function initializes dma descriptor chain.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to start the transfer on.
 */
static void init_dma_desc_chain(dwc_otg_core_if_t const * core_if, dwc_ep_t * ep)
{
	dwc_otg_dev_dma_desc_t *dma_desc;
	uint32_t offset;
	uint32_t xfer_est;
	uint32_t i;
	USIGN maxxfer_local, u_total_len,j;

		maxxfer_local = ep->maxxfer;
		u_total_len = ep->total_len;

	ep->desc_cnt = ((((u_total_len / maxxfer_local) +
	    (u_total_len % maxxfer_local)) != 0) ? 1 : 0);

	if (!ep->desc_cnt) {
		ep->desc_cnt = 1;
	}

	if (ep->desc_cnt > MAX_DMA_DESC_CNT) {
		ep->desc_cnt = MAX_DMA_DESC_CNT;
	}

	dma_desc = ep->desc_addr;
	if (maxxfer_local == (uint32_t)ep->maxpacket) {
		j = u_total_len % maxxfer_local;
		if ((j) &&
		    ((u_total_len / maxxfer_local) < MAX_DMA_DESC_CNT)) {
			xfer_est = ((ep->desc_cnt - 1) * maxxfer_local) +
			    (u_total_len % maxxfer_local);
		} else {
			xfer_est = ep->desc_cnt * maxxfer_local;
		}
	} else {
		xfer_est = u_total_len;
	}
	offset = 0;
	for (i = 0; i < ep->desc_cnt; ++i) {
		/** DMA Descriptor Setup */
		if (xfer_est > maxxfer_local) {
			dma_desc->status.b.bs = BS_HOST_BUSY;
			dma_desc->status.b.l = 0;
			dma_desc->status.b.ioc = 0;
			dma_desc->status.b.sp = 0;
			dma_desc->status.b.bytes = maxxfer_local;
			dma_desc->buf = ep->dma_addr + offset;
			dma_desc->status.b.sts = 0;
			dma_desc->status.b.bs = BS_HOST_READY;

			xfer_est -= maxxfer_local;
			offset += maxxfer_local;
		} else {
			dma_desc->status.b.bs = BS_HOST_BUSY;
			dma_desc->status.b.l = 1;
			dma_desc->status.b.ioc = 1;
			if (ep->is_in != 0) {
				dma_desc->status.b.sp =
				    ((xfer_est %
				     (uint32_t)ep->maxpacket) != 0) ? 1 : (((ep->
							    sent_zlp) != 0) ? 1 : 0); 
				dma_desc->status.b.bytes = xfer_est;
			} else {
				if (maxxfer_local == (uint32_t)ep->maxpacket) {
					dma_desc->status.b.bytes = xfer_est;
				}
				else	{
					dma_desc->status.b.bytes =
				    		xfer_est + ((4 - (xfer_est & 0x3)) & 0x3);
				}
			}

			dma_desc->buf = ep->dma_addr + offset;
			dma_desc->status.b.sts = 0;
			dma_desc->status.b.bs = BS_HOST_READY;
		}
		dma_desc++;
	}
	(void)core_if;
}

/**
 * This function does the setup for a data transfer for an EP and
 * starts the transfer. For an IN transfer, the packets will be
 * loaded into the appropriate Tx FIFO in the ISR. For OUT transfers,
 * the packets are unloaded from the Rx FIFO in the ISR.  the ISR.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to start the transfer on.
 */

void dwc_otg_ep_start_transfer(dwc_otg_core_if_t * core_if, dwc_ep_t * ep)
{
	int32_t flag = 0;
	depctl_data_t depctl;
	deptsiz_data_t deptsiz;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)	
	gintmsk_data_t intr_mask;
	intr_mask.d32 = 0 ;
#endif
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	DBG_USB_Print(((DBG_PCDV | DBG_CILV) != 0), "%s()\n", __func__);
	DBG_USB_Print(DBG_PCD, "ep%d-%s xfer_len=%d xfer_cnt=%d "
		    "xfer_buff=%p start_xfer_buff=%p, total_len = %d\n",
		    ep->num, (ep->is_in ? "IN" : "OUT"), ep->xfer_len,
		    ep->xfer_count, ep->xfer_buff, ep->start_xfer_buff,
		    ep->total_len);
	/* IN endpoint */
	if (ep->is_in == 1) {
		dwc_otg_dev_in_ep_regs_t *in_regs =
		    core_if->dev_if->in_ep_regs[ep->num];

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		gnptxsts_data_t gtxstatus;

		gtxstatus.d32 =
		    REG_RD(&core_if->core_global_regs->gnptxsts);
		if (((core_if->en_multiple_tx_fifo == 0)
		    && (gtxstatus.b.nptxqspcavail == 0)) && (!core_if->dma_enable)) {
			DBG_USB_Print(DBG_USB,"TX Queue Full (0x%0x)\n", gtxstatus.d32);
			flag = 1;
		}
#endif
		if(flag == 0){
			depctl.d32 = REG_RD(&(in_regs->diepctl));
			deptsiz.d32 = REG_RD(&(in_regs->dieptsiz));

			if (((uint32_t)ep->maxpacket) > (((uint32_t)ep->maxxfer) / ((uint32_t)MAX_PKT_CNT))) {
				ep->xfer_len =((((uint32_t)ep->xfer_len) + ((uint32_t)ep->maxxfer)) < (((uint32_t)ep->total_len) - ((uint32_t)ep->xfer_len))) ?
						((uint32_t)ep->maxxfer) : (((uint32_t)ep->total_len) - ((uint32_t)ep->xfer_len));
			}
			else  {
				ep->xfer_len += ((MAX_PKT_CNT * (ep->maxpacket)) < ((ep->total_len) - (ep->xfer_len))) ?
					 (MAX_PKT_CNT * (ep->maxpacket)) : ((ep->total_len) - (ep->xfer_len));

			}
			/* Zero Length Packet? */
			if ((ep->xfer_len - ep->xfer_count) == 0) {
				deptsiz.b.xfersize = 0;
				deptsiz.b.pktcnt = 1;
			} else {
				/* Program the transfer size and packet count
				 *      as follows: xfersize = N * maxpacket +
				 *      short_packet pktcnt = N + (short_packet
				 *      exist ? 1 : 0) 
				 */
				deptsiz.b.xfersize = ep->xfer_len - ep->xfer_count;
				deptsiz.b.pktcnt =
					((ep->xfer_len - ep->xfer_count) - (1 +
					 ep->maxpacket)) / ep->maxpacket;
				if (deptsiz.b.pktcnt > MAX_PKT_CNT) {
					deptsiz.b.pktcnt = MAX_PKT_CNT;
					deptsiz.b.xfersize = deptsiz.b.pktcnt * ep->maxpacket;
				} 
			}

			/* Write the DMA register */
			if ((core_if->dma_enable) != 0) {
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
				if (core_if->dma_desc_enable == 0) {
						deptsiz.b.mc = 1;
					REG_WR(&in_regs->dieptsiz,
							deptsiz.d32);
					REG_WR(&(in_regs->diepdma),
							(uint32_t) ep->dma_addr);
				} else {
#endif
						init_dma_desc_chain(core_if, ep);
					/** DIEPDMAn Register write */
						REG_WR(&in_regs->diepdma,
								ep->dma_desc_addr);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
				}
#endif
			} 
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)            
            else {
				REG_WR(&in_regs->dieptsiz, deptsiz.d32);
					/**
					 * Enable the Non-Periodic Tx FIFO empty interrupt,
					 * or the Tx FIFO epmty interrupt in dedicated Tx FIFO mode,
					 * the data will be written into the fifo by the ISR.
					 */
                if (core_if->en_multiple_tx_fifo == 0) {
						intr_mask.b.nptxfempty = 1;
						DWC_MODIFY_REG32
							(&core_if->core_global_regs->gintmsk,
							 intr_mask.d32, intr_mask.d32);
					} else {
						/* Enable the Tx FIFO Empty Interrupt for this EP */
						if (ep->xfer_len > 0) {
							uint32_t fifoemptymsk = 0;
							fifoemptymsk = 1U << ep->num;
							DWC_MODIFY_REG32
								(&core_if->dev_if->dev_global_regs->dtknqr4_fifoemptymsk,
								 0, fifoemptymsk);

						}
					}
				} 
			if ((!(core_if->core_params->en_multiple_tx_fifo)) && (core_if->dma_enable)) {
				depctl.b.nextep = core_if->nextep_seq[ep->num];
			}
#endif

			/* EP enable, IN data in FIFO */
			depctl.b.cnak = 1;
			depctl.b.epena = 1;
			REG_WR(&in_regs->diepctl, depctl.d32);
		}
	} else {
		/* OUT endpoint */
		dwc_otg_dev_out_ep_regs_t *out_regs =
		    core_if->dev_if->out_ep_regs[ep->num];

		depctl.d32 = REG_RD(&(out_regs->doepctl));
		deptsiz.d32 = REG_RD(&(out_regs->doeptsiz));
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		if (!core_if->dma_desc_enable) {	
			if (((uint32_t)ep->maxpacket) > (((uint32_t)ep->maxxfer) / ((uint32_t)MAX_PKT_CNT))) {
				ep->xfer_len = ((((uint32_t)ep->xfer_len) + ((uint32_t)ep->maxxfer)) < (((uint32_t)(ep->total_len - ep->xfer_len)))) ?
                        	((uint32_t)ep->maxxfer) : (((uint32_t)ep->total_len) - ((uint32_t)ep->xfer_len));
			}
                else {
					ep->xfer_len += ((MAX_PKT_CNT * (ep->maxpacket)) < ((ep->total_len) 
					- (ep->xfer_len))) ? (MAX_PKT_CNT * (ep->maxpacket)) : ((ep->total_len) - (ep->xfer_len));
				}
		}
#endif
		/* Program the transfer size and packet count as follows:
		 *
		 *      pktcnt = N                                                                                
		 *      xfersize = N * maxpacket
		 */
		if ((ep->xfer_len - ep->xfer_count) == 0) {
			/* Zero Length Packet */
			deptsiz.b.xfersize = ep->maxpacket;
			deptsiz.b.pktcnt = 1;
		} else {
			deptsiz.b.pktcnt =
			    ((ep->xfer_len - ep->xfer_count) +
			     (ep->maxpacket - 1)) / ep->maxpacket;
			if (deptsiz.b.pktcnt > MAX_PKT_CNT) {
				deptsiz.b.pktcnt = MAX_PKT_CNT;
			}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			if (!core_if->dma_desc_enable) {
				ep->xfer_len =
			    		(deptsiz.b.pktcnt * ep->maxpacket) + ep->xfer_count;
			}
#endif
			deptsiz.b.xfersize = ep->xfer_len - ep->xfer_count;
		}

		DBG_USB_Print(DBG_PCDV, "ep%d xfersize=%d pktcnt=%d\n",
			    ep->num, deptsiz.b.xfersize, deptsiz.b.pktcnt);

		if (core_if->dma_enable != 0) {
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			if (!core_if->dma_desc_enable) {
				REG_WR(&out_regs->doeptsiz,
						deptsiz.d32);

				REG_WR(&(out_regs->doepdma),
						(uint32_t) ep->dma_addr);
			} else {
#endif
					/** This is used for interrupt out transfers*/
					if (!ep->xfer_len) {
						ep->xfer_len = ep->total_len;
					}
					init_dma_desc_chain(core_if, ep);

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
					if ((core_if->core_params->dev_out_nak) != 0) {
						if (ep->type == DWC_OTG_EP_TYPE_BULK) {
							deptsiz.b.pktcnt = (ep->total_len +
								(ep->maxpacket - 1)) / ep->maxpacket;
							deptsiz.b.xfersize = ep->total_len;
							/* Remember initial value of doeptsiz */
							core_if->start_doeptsiz_val[ep->num] = deptsiz.d32;
							REG_WR(&out_regs->doeptsiz,
								deptsiz.d32);													
						}
					}
#endif
				/** DOEPDMAn Register write */
					REG_WR(&out_regs->doepdma,
							ep->dma_desc_addr);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			}
#endif
		} 
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
        else {
			REG_WR(&out_regs->doeptsiz, deptsiz.d32);
		}
#endif
		/* EP enable */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;

		REG_WR(&out_regs->doepctl, depctl.d32);

		DBG_USB_Print(DBG_PCD, "DOEPCTL=%08x DOEPTSIZ=%08x\n",
			    REG_RD(&out_regs->doepctl),
			    REG_RD(&out_regs->doeptsiz));
		DBG_USB_Print(DBG_PCD, "DAINTMSK=%08x GINTMSK=%08x\n",
			    REG_RD(&core_if->dev_if->dev_global_regs->
					   daintmsk),
			    REG_RD(&core_if->core_global_regs->
					   gintmsk));

		/* Timer is scheduling only for out bulk transfers for 
		 * "Device DDMA OUT NAK Enhancement" feature to inform user 
		 * about received data payload in case of timeout 
		 */

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		if (core_if->core_params->dev_out_nak != 0) {
			if (ep->type == DWC_OTG_EP_TYPE_BULK) {
				core_if->ep_xfer_info[ep->num].core_if = core_if;
				core_if->ep_xfer_info[ep->num].ep = ep;
				core_if->ep_xfer_info[ep->num].state = 1;

			}
		}
#endif
	}
	return;
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
/**
 * This function setup a zero length transfer in Buffer DMA and
 * Slave modes for usb requests with zero field set
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to start the transfer on.
 *
 */
void dwc_otg_ep_start_zl_transfer(dwc_otg_core_if_t const * core_if, dwc_ep_t const * ep)
{

	depctl_data_t depctl;
	deptsiz_data_t deptsiz;
	gintmsk_data_t intr_mask;
	intr_mask.d32 = 0 ;

	DBG_USB_Print(((DBG_PCDV | DBG_CILV) != 0), "%s()\n", __func__);
	DBG_USB_Print(DBG_USB,"zero length transfer is called\n");

	/* IN endpoint */
	if (ep->is_in == 1) {
		dwc_otg_dev_in_ep_regs_t *in_regs =
		    core_if->dev_if->in_ep_regs[ep->num];

		depctl.d32 = REG_RD(&(in_regs->diepctl));
		deptsiz.d32 = REG_RD(&(in_regs->dieptsiz));

		deptsiz.b.xfersize = 0;
		deptsiz.b.pktcnt = 1;

		/* Write the DMA register */
		if (core_if->dma_enable != 0) {
			if (core_if->dma_desc_enable == 0) {
				deptsiz.b.mc = 1;
				REG_WR(&in_regs->dieptsiz,
						deptsiz.d32);
				REG_WR(&(in_regs->diepdma),
						(uint32_t) ep->dma_addr);
			}
		} 
        else {
			REG_WR(&in_regs->dieptsiz, deptsiz.d32);
			/**
			 * Enable the Non-Periodic Tx FIFO empty interrupt,
			 * or the Tx FIFO epmty interrupt in dedicated Tx FIFO mode,
			 * the data will be written into the fifo by the ISR.
			 */
			if (core_if->en_multiple_tx_fifo == 0) {
				intr_mask.b.nptxfempty = 1;
				DWC_MODIFY_REG32(&core_if->
						 core_global_regs->gintmsk,
						 intr_mask.d32, intr_mask.d32);
			} else {
				/* Enable the Tx FIFO Empty Interrupt for this EP */
				if (ep->xfer_len > 0) {
					uint32_t fifoemptymsk = 0;
					fifoemptymsk = 1U << ep->num;
					DWC_MODIFY_REG32(&core_if->
							 dev_if->dev_global_regs->dtknqr4_fifoemptymsk,
							 0, fifoemptymsk);
				}
			}
		}
		if ((!(core_if->core_params->en_multiple_tx_fifo)) && (core_if->dma_enable)) {
			depctl.b.nextep = core_if->nextep_seq[ep->num];
		}
		/* EP enable, IN data in FIFO */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		REG_WR(&in_regs->diepctl, depctl.d32);

	} else {
		/* OUT endpoint */
		dwc_otg_dev_out_ep_regs_t *out_regs =
		    core_if->dev_if->out_ep_regs[ep->num];

		depctl.d32 = REG_RD(&(out_regs->doepctl));
		deptsiz.d32 = REG_RD(&(out_regs->doeptsiz));

		/* Zero Length Packet */
		deptsiz.b.xfersize = ep->maxpacket;
		deptsiz.b.pktcnt = 1;

		if (core_if->dma_enable != 0) {
			if (!core_if->dma_desc_enable) {
				REG_WR(&out_regs->doeptsiz,
						deptsiz.d32);

				REG_WR(&(out_regs->doepdma),
						(uint32_t) ep->dma_addr);
			}
		} 
        else {
			REG_WR(&out_regs->doeptsiz, deptsiz.d32);
		}
		/* EP enable */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		REG_WR(&out_regs->doepctl, depctl.d32);
	}
}

#endif
/**
 * This function does the setup for a data transfer for EP0 and starts
 * the transfer.  For an IN transfer, the packets will be loaded into
 * the appropriate Tx FIFO in the ISR. For OUT transfers, the packets are
 * unloaded from the Rx FIFO in the ISR.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP0 data.
 */
void dwc_otg_ep0_start_transfer(dwc_otg_core_if_t const * core_if, dwc_ep_t * ep)
{
	int32_t flag = 0;
	depctl_data_t depctl;
	deptsiz0_data_t deptsiz;
	dwc_otg_dev_dma_desc_t *dma_desc;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
    gintmsk_data_t intr_mask;
	intr_mask.d32 = 0 ;
#endif
	DBG_USB_Print(DBG_PCD, "ep%d-%s xfer_len=%d xfer_cnt=%d "
		    "xfer_buff=%p start_xfer_buff=%p \n",
		    ep->num, (ep->is_in ? "IN" : "OUT"), ep->xfer_len,
		    ep->xfer_count, ep->xfer_buff, ep->start_xfer_buff);
	ep->total_len = ep->xfer_len;
	/* IN endpoint */
	if (ep->is_in == 1) {
		dwc_otg_dev_in_ep_regs_t *in_regs =
		    core_if->dev_if->in_ep_regs[0];
        
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		gnptxsts_data_t gtxstatus;
#endif
		if (core_if->snpsid >= OTG_CORE_REV_3_00a) {
			depctl.d32 = REG_RD(&in_regs->diepctl);
			if (depctl.b.epena != 0) {
				flag = 1;
			}
		}
		if (flag == 0){
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			gtxstatus.d32 =
				REG_RD(&core_if->core_global_regs->gnptxsts);

			/* If dedicated FIFO every time flush fifo before enable ep*/
			/* if (core_if->en_multiple_tx_fifo && core_if->snpsid >= OTG_CORE_REV_3_00a) */
			/*	dwc_otg_flush_tx_fifo(core_if, ep->tx_fifo_num); */

			if (((core_if->en_multiple_tx_fifo == 0)
				&& (gtxstatus.b.nptxqspcavail == 0))
				&& (!core_if->dma_enable)) {
				deptsiz.d32 = REG_RD(&in_regs->dieptsiz);
				DBG_USB_Print(DBG_PCD, "DIEPCTL0=%0x\n",
						REG_RD(&in_regs->diepctl));
				DBG_USB_Print(DBG_PCD, "DIEPTSIZ0=%0x (sz=%d, pcnt=%d)\n",
						deptsiz.d32,
						deptsiz.b.xfersize, deptsiz.b.pktcnt);
				DBG_USB_Print(DBG_USB,"TX Queue or FIFO Full (0x%0x)\n",
					   gtxstatus.d32);
				flag = 1;
			}
#endif
			if(flag == 0){
				depctl.d32 = REG_RD(&in_regs->diepctl);
				deptsiz.d32 = REG_RD(&in_regs->dieptsiz);

				/* Zero Length Packet? */
				if (ep->xfer_len == 0) {
					deptsiz.b.xfersize = 0;
					deptsiz.b.pktcnt = 1;
				} else {
					/* Program the transfer size and packet count
					 *      as follows: xfersize = N * maxpacket +
					 *      short_packet pktcnt = N + (short_packet
					 *      exist ? 1 : 0) 
					 */
					if (ep->xfer_len > ep->maxpacket) {
						ep->xfer_len = ep->maxpacket;
						deptsiz.b.xfersize = ep->maxpacket;
					} else {
						deptsiz.b.xfersize = ep->xfer_len;
					}
					deptsiz.b.pktcnt = 1;

				}
				DBG_USB_Print(DBG_PCDV,
						"IN len=%d  xfersize=%d pktcnt=%d [%08x]\n",
						ep->xfer_len, deptsiz.b.xfersize, deptsiz.b.pktcnt,
						deptsiz.d32);

				/* Write the DMA register */
				if (core_if->dma_enable != 0)  {
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
					if (core_if->dma_desc_enable == 0) {
						REG_WR(&in_regs->dieptsiz,
								deptsiz.d32);

						REG_WR(&(in_regs->diepdma),
								(uint32_t) ep->dma_addr);
					} else {
#endif
						dma_desc = core_if->dev_if->in_desc_addr;

						/** DMA Descriptor Setup */
						dma_desc->status.b.bs = BS_HOST_BUSY;
						dma_desc->status.b.l = 1;
						dma_desc->status.b.ioc = 1;
						dma_desc->status.b.sp =
							(ep->xfer_len == ep->maxpacket) ? 0 : 1;
						dma_desc->status.b.bytes = ep->xfer_len;
						dma_desc->buf = ep->dma_addr;
						dma_desc->status.b.sts = 0;
						dma_desc->status.b.bs = BS_HOST_READY;

						/** DIEPDMA0 Register write */
						REG_WR(&in_regs->diepdma,
								core_if->
								dev_if->dma_in_desc_addr);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
					}
#endif
				} 
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)                
                else {
					REG_WR(&in_regs->dieptsiz, deptsiz.d32);
				}
				if ((!(core_if->core_params->en_multiple_tx_fifo)) && (core_if->dma_enable)) {
					depctl.b.nextep = core_if->nextep_seq[ep->num];
				}
#endif
                dwc_otg_flush_tx_fifo(core_if, ep->tx_fifo_num);
				/* EP enable, IN data in FIFO */
				depctl.b.cnak = 1;
				depctl.b.epena = 1;
				REG_WR(&in_regs->diepctl, depctl.d32);

				/**
				 * Enable the Non-Periodic Tx FIFO empty interrupt, the
				 * data will be written into the fifo by the ISR.
				 */
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
				if (!core_if->dma_enable) {
					if (core_if->en_multiple_tx_fifo == 0) {
						intr_mask.b.nptxfempty = 1;
						DWC_MODIFY_REG32(&core_if->
								 core_global_regs->gintmsk,
								 intr_mask.d32, intr_mask.d32);
					} else {
						/* Enable the Tx FIFO Empty Interrupt for this EP */
						if (ep->xfer_len != 0) {
							uint32_t fifoemptymsk = 0;
							fifoemptymsk |= 1U << ep->num;
							DWC_MODIFY_REG32(&core_if->
									 dev_if->dev_global_regs->dtknqr4_fifoemptymsk,
									 0, fifoemptymsk);
						}
					}
				}
#endif
			}
		}
	} else {
		/* OUT endpoint */
		dwc_otg_dev_out_ep_regs_t *out_regs =
		    core_if->dev_if->out_ep_regs[0];

		depctl.d32 = REG_RD(&out_regs->doepctl);
		deptsiz.d32 = REG_RD(&out_regs->doeptsiz);
		/* Program the transfer size and packet count as follows:
		 *      xfersize = N * (maxpacket + 4 - (maxpacket % 4))
		 *      pktcnt = N    */
		/* Zero Length Packet */
		deptsiz.b.xfersize = ep->maxpacket;
		deptsiz.b.pktcnt = 1;
		if (core_if->snpsid >= OTG_CORE_REV_3_00a) {
			deptsiz.b.supcnt = 3;
		}
		if (core_if->dma_enable != 0) {
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			if (!core_if->dma_desc_enable) {
				REG_WR(&out_regs->doeptsiz,
						deptsiz.d32);

				REG_WR(&(out_regs->doepdma),
						(uint32_t) ep->dma_addr);
			} else {
#endif
				dma_desc = core_if->dev_if->out_desc_addr;

				
				/** DMA Descriptor Setup */
				dma_desc->status.b.bs = BS_HOST_BUSY;
				if (core_if->snpsid >= OTG_CORE_REV_3_00a) {
					dma_desc->status.b.mtrf = 0;
					dma_desc->status.b.sr = 0;
				}
				dma_desc->status.b.l = 1;
				dma_desc->status.b.ioc = 1;
				dma_desc->status.b.bytes = ep->maxpacket;
				dma_desc->buf = ep->dma_addr;
				dma_desc->status.b.sts = 0;
				dma_desc->status.b.bs = BS_HOST_READY;
				/** DOEPDMA0 Register write */
				REG_WR(&out_regs->doepdma,
						core_if->dev_if->
						dma_out_desc_addr);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			}
#endif
		} 
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
        else {
			REG_WR(&out_regs->doeptsiz, deptsiz.d32);
		}
#endif
		/* EP enable */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		REG_WR(&(out_regs->doepctl), depctl.d32);
	}
#if(DBG_USB == 0)
    (void)deptsiz;
#endif
	return;
}

/**
 * This function continues control IN transfers started by
 * dwc_otg_ep0_start_transfer, when the transfer does not fit in a
 * single packet.  NOTE: The DIEPCTL0/DOEPCTL0 registers only have one
 * bit for the packet count.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP0 data.
 */
void dwc_otg_ep0_continue_transfer(dwc_otg_core_if_t const * core_if, dwc_ep_t * ep)
{
	depctl_data_t depctl;
	deptsiz0_data_t deptsiz;
    dwc_otg_dev_dma_desc_t *dma_desc;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	gintmsk_data_t intr_mask;
	intr_mask.d32 = 0;
#endif

	if (ep->is_in != 0) {
		dwc_otg_dev_in_ep_regs_t *in_regs =
		    core_if->dev_if->in_ep_regs[0];
		/** @todo Should there be check for room in the Tx
		 * Status Queue.  If not remove the code above this comment. */
		depctl.d32 = REG_RD(&in_regs->diepctl);
		deptsiz.d32 = REG_RD(&in_regs->dieptsiz);
		/* Program the transfer size and packet count
		 *      as follows: xfersize = N * maxpacket +
		 *      short_packet pktcnt = N + (short_packet
		 *      exist ? 1 : 0) 
		 */
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		if (core_if->dma_desc_enable == 0) {
			deptsiz.b.xfersize =
			    ((ep->total_len - ep->xfer_count) >
			    ep->maxpacket) ? ep->maxpacket : (ep->total_len -
							     ep->xfer_count);
			deptsiz.b.pktcnt = 1;
			if (core_if->dma_enable == 0) {
				ep->xfer_len += deptsiz.b.xfersize;
			} else {
				ep->xfer_len = deptsiz.b.xfersize;
			}
			REG_WR(&in_regs->dieptsiz, deptsiz.d32);
		} else {
#endif
			ep->xfer_len =
			    ((ep->total_len - ep->xfer_count) >
			    ep->maxpacket) ? ep->maxpacket : (ep->total_len -
							     ep->xfer_count);

			dma_desc = core_if->dev_if->in_desc_addr;

			/** DMA Descriptor Setup */
			dma_desc->status.b.bs = BS_HOST_BUSY;
			dma_desc->status.b.l = 1;
			dma_desc->status.b.ioc = 1;
			dma_desc->status.b.sp =
			    (ep->xfer_len == ep->maxpacket) ? 0 : 1;
			dma_desc->status.b.bytes = ep->xfer_len;
			dma_desc->buf = ep->dma_addr;
			dma_desc->status.b.sts = 0;
			dma_desc->status.b.bs = BS_HOST_READY;

			/** DIEPDMA0 Register write */
			REG_WR(&in_regs->diepdma,
					core_if->dev_if->dma_in_desc_addr);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		}
#endif

		DBG_USB_Print(DBG_PCDV,
			    "IN len=%d  xfersize=%d pktcnt=%d [%08x]\n",
			    ep->xfer_len, deptsiz.b.xfersize, deptsiz.b.pktcnt,
			    deptsiz.d32);

		/* Write the DMA register */
		if (core_if->hwcfg2.b.architecture == DWC_INT_DMA_ARCH) {
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			if (core_if->dma_desc_enable == 0) {
				REG_WR(&(in_regs->diepdma),
						(uint32_t) ep->dma_addr);
			}
#endif
		}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		if ((!(core_if->core_params->en_multiple_tx_fifo)) && (core_if->dma_enable)) {
			depctl.b.nextep = core_if->nextep_seq[ep->num]; 
		}
#endif
        dwc_otg_flush_tx_fifo(core_if, ep->tx_fifo_num);
		/* EP enable, IN data in FIFO */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		REG_WR(&in_regs->diepctl, depctl.d32);

		/**
		 * Enable the Non-Periodic Tx FIFO empty interrupt, the
		 * data will be written into the fifo by the ISR.
		 */
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		if (!core_if->dma_enable) {
			if (core_if->en_multiple_tx_fifo == 0) {
				/* First clear it from GINTSTS */
				intr_mask.b.nptxfempty = 1;
				DWC_MODIFY_REG32(&core_if->
						 core_global_regs->gintmsk,
						 intr_mask.d32, intr_mask.d32);

			} else {
				/* Enable the Tx FIFO Empty Interrupt for this EP */
				if (ep->xfer_len > 0) {
					uint32_t fifoemptymsk = 0;
					fifoemptymsk |= 1U << ep->num;
					DWC_MODIFY_REG32(&core_if->
							 dev_if->dev_global_regs->dtknqr4_fifoemptymsk,
							 0, fifoemptymsk);
				}
			}
		}
#endif
	} else {
		dwc_otg_dev_out_ep_regs_t *out_regs =
		    core_if->dev_if->out_ep_regs[0];

		depctl.d32 = REG_RD(&out_regs->doepctl);
		deptsiz.d32 = REG_RD(&out_regs->doeptsiz);

		/* Program the transfer size and packet count
		 *      as follows: xfersize = N * maxpacket +
		 *      short_packet pktcnt = N + (short_packet
		 *      exist ? 1 : 0) 
		 */
		deptsiz.b.xfersize = ep->maxpacket;
		deptsiz.b.pktcnt = 1;

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		if (core_if->dma_desc_enable == 0) {
			REG_WR(&out_regs->doeptsiz, deptsiz.d32);
		} else {
#endif
			dma_desc = core_if->dev_if->out_desc_addr;

			/** DMA Descriptor Setup */
			dma_desc->status.b.bs = BS_HOST_BUSY;
			dma_desc->status.b.l = 1;
			dma_desc->status.b.ioc = 1;
			dma_desc->status.b.bytes = ep->maxpacket;
			dma_desc->buf = ep->dma_addr;
			dma_desc->status.b.sts = 0;
			dma_desc->status.b.bs = BS_HOST_READY;

			/** DOEPDMA0 Register write */
			REG_WR(&out_regs->doepdma,
					core_if->dev_if->dma_out_desc_addr);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		}
#endif

		DBG_USB_Print(DBG_PCDV,
			    "IN len=%d  xfersize=%d pktcnt=%d [%08x]\n",
			    ep->xfer_len, deptsiz.b.xfersize, deptsiz.b.pktcnt,
			    deptsiz.d32);

		/* Write the DMA register */
		if (core_if->hwcfg2.b.architecture == DWC_INT_DMA_ARCH) {
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
			if (core_if->dma_desc_enable == 0) {
				REG_WR(&(out_regs->doepdma),
						(uint32_t) ep->dma_addr);
            }
#endif
		}
		/* EP enable, IN data in FIFO */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		REG_WR(&out_regs->doepctl, depctl.d32);

	}
#if(DBG_USB == 0)
    (void)deptsiz;
#endif

}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
/**
 * This function writes a packet into the Tx FIFO associated with the
 * EP. For non-periodic EPs the non-periodic Tx FIFO is written.  For
 * periodic EPs the periodic Tx FIFO associated with the EP is written
 * with all packets for the next micro-frame.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to write packet for.
 * @param dma Indicates if DMA is being used.
 */
void dwc_otg_ep_write_packet(dwc_otg_core_if_t const * core_if, dwc_ep_t * ep,
			     int32_t dma)
{
	/**
	 * The buffer is padded to DWORD on a per packet basis in
	 * slave/dma mode if the MPS is not DWORD aligned. The last
	 * packet, if short, is also padded to a multiple of DWORD.
	 *
	 * ep->xfer_buff always starts DWORD aligned in memory and is a
	 * multiple of DWORD in length
	 *
	 * ep->xfer_len can be any number of bytes
	 *
	 * ep->xfer_count is a multiple of ep->maxpacket until the last
	 *	packet
	 *
	 * FIFO access is DWORD */
	int32_t flag = 0;
	uint32_t byte_count;
	DBG_USB_Print(((DBG_PCDV | DBG_CILV) != 0), "%s(%p,%p)\n", __func__, core_if,
		    ep);
	if (ep->xfer_count >= ep->xfer_len) {
		DBG_Warn_Print("%s() No data for EP%d!!!\n", __func__, ep->num);
		flag = 1;
	}
	if(flag == 0){
		/* Find the byte length of the packet either short packet or MPS */
		if ((ep->xfer_len - ep->xfer_count) < ep->maxpacket) {
			byte_count =(uint32_t) (ep->xfer_len - ep->xfer_count);
		} else {
			byte_count = ep->maxpacket;
		}
		/* Find the DWORD length, padded by extra bytes as neccessary if MPS
		 * is not a multiple of DWORD */
		/*dword_count = (byte_count + 3) / 4; */
#ifdef VERBOSE
		dump_msg(ep->xfer_buff, byte_count);
#endif
		ep->xfer_count = (((uint32_t)ep->xfer_count)+((uint32_t)byte_count));
		ep->xfer_buff += byte_count;
		ep->dma_addr += byte_count;
		(void)core_if;
		(void)dma;
	}
	return;
}
#endif

/**
 * Set the EP STALL.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to set the stall on.
 */
void dwc_otg_ep_set_stall(dwc_otg_core_if_t const * core_if, dwc_ep_t const * ep)
{
	depctl_data_t depctl;
	volatile uint32_t *depctl_addr;
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	DBG_USB_Print(DBG_PCD, "%s ep%d-%s\n", __func__, ep->num,
		    (ep->is_in ? "IN" : "OUT"));

	if (ep->is_in == 1) {
		depctl_addr = &(core_if->dev_if->in_ep_regs[ep->num]->diepctl);
		depctl.d32 = REG_RD(depctl_addr);

		/* set the disable and stall bits */
		if (depctl.b.epena != 0) {
			depctl.b.epdis = 1;
		}
		depctl.b.stall = 1;
		REG_WR(depctl_addr, depctl.d32);
	} else {
		depctl_addr = &(core_if->dev_if->out_ep_regs[ep->num]->doepctl);
		depctl.d32 = REG_RD(depctl_addr);

		/* set the stall bit */
		depctl.b.stall = 1;
		REG_WR(depctl_addr, depctl.d32);
	}

	DBG_USB_Print(DBG_PCD, "DEPCTL=%0x\n", REG_RD(depctl_addr));

	return;
}

/**
 * Clear the EP STALL.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to clear stall from.
 */
void dwc_otg_ep_clear_stall(dwc_otg_core_if_t const * core_if, dwc_ep_t const * ep)
{
	depctl_data_t depctl;
	volatile uint32_t *depctl_addr;
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
	DBG_USB_Print(DBG_PCD, "%s ep%d-%s\n", __func__, ep->num,
		    (ep->is_in ? "IN" : "OUT"));

	if (ep->is_in == 1) {
		depctl_addr = &(core_if->dev_if->in_ep_regs[ep->num]->diepctl);
	} else {
		depctl_addr = &(core_if->dev_if->out_ep_regs[ep->num]->doepctl);
	}

	depctl.d32 = REG_RD(depctl_addr);

	/* clear the stall bits */
	depctl.b.stall = 0;

	/*
	 * USB Spec 9.4.5: For endpoints using data toggle, regardless
	 * of whether an endpoint has the Halt feature set, a
	 * ClearFeature(ENDPOINT_HALT) request always results in the
	 * data toggle being reinitialized to DATA0.
	 */
	if (ep->type == DWC_OTG_EP_TYPE_BULK) {
		depctl.b.setd0pid = 1;	/* DATA0 */
	}

	REG_WR(depctl_addr, depctl.d32);
	DBG_USB_Print(DBG_PCD, "DEPCTL=%0x\n", REG_RD(depctl_addr));
	return;
}

/**
 * This functions reads the device registers and prints them
 *
 * @param core_if Programming view of DWC_otg controller.
 */
void dwc_otg_dump_dev_registers(dwc_otg_core_if_t const * core_if)
{
	int32_t i;
	volatile uint32_t *ptr_addr;
	DBG_USB_Print(DBG_USB,"Device Global Registers\n");
	ptr_addr = &core_if->dev_if->dev_global_regs->dcfg;
	DBG_USB_Print(DBG_USB,"DCFG		 @0x%08lX : 0x%08X\n",
		   (uint64_t)ptr_addr, REG_RD(ptr_addr));
	ptr_addr = &core_if->dev_if->dev_global_regs->dctl;
	DBG_USB_Print(DBG_USB,"DCTL		 @0x%08lX : 0x%08X\n",
		   (uint64_t)ptr_addr, REG_RD(ptr_addr));
	ptr_addr = &core_if->dev_if->dev_global_regs->dsts;
	DBG_USB_Print(DBG_USB,"DSTS		 @0x%08lX : 0x%08X\n",
		   (uint64_t)ptr_addr, REG_RD(ptr_addr));
	ptr_addr = &core_if->dev_if->dev_global_regs->diepmsk;
	DBG_USB_Print(DBG_USB,"DIEPMSK	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->dev_if->dev_global_regs->doepmsk;
	DBG_USB_Print(DBG_USB,"DOEPMSK	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->dev_if->dev_global_regs->daint;
	DBG_USB_Print(DBG_USB,"DAINT	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->dev_if->dev_global_regs->daintmsk;
	DBG_USB_Print(DBG_USB,"DAINTMSK	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->dev_if->dev_global_regs->dtknqr1;
	DBG_USB_Print(DBG_USB,"DTKNQR1	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	if (core_if->hwcfg2.b.dev_token_q_depth > 6) {
		ptr_addr = &core_if->dev_if->dev_global_regs->dtknqr2;
		DBG_USB_Print(DBG_USB,"DTKNQR2	 @0x%08lX : 0x%08X\n",
			   (uint64_t)ptr_addr, REG_RD(ptr_addr));
	}
	ptr_addr = &core_if->dev_if->dev_global_regs->dvbusdis;
	DBG_USB_Print(DBG_USB,"DVBUSID	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));

	ptr_addr = &core_if->dev_if->dev_global_regs->dvbuspulse;
	DBG_USB_Print(DBG_USB,"DVBUSPULSE	@0x%08lX : 0x%08X\n",
		   (uint64_t)ptr_addr, REG_RD(ptr_addr));
	ptr_addr = &core_if->dev_if->dev_global_regs->dtknqr3_dthrctl;
	DBG_USB_Print(DBG_USB,"DTKNQR3_DTHRCTL	 @0x%08lX : 0x%08X\n",
		   (uint64_t)ptr_addr, REG_RD(ptr_addr));
	if (core_if->hwcfg2.b.dev_token_q_depth > 22) {
		ptr_addr = &core_if->dev_if->dev_global_regs->dtknqr4_fifoemptymsk;
		DBG_USB_Print(DBG_USB,"DTKNQR4	 @0x%08lX : 0x%08X\n",
			   (uint64_t)ptr_addr, REG_RD(ptr_addr));
	}
	ptr_addr = &core_if->dev_if->dev_global_regs->dtknqr4_fifoemptymsk;
	DBG_USB_Print(DBG_USB,"FIFOEMPMSK	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	if (core_if->hwcfg2.b.multi_proc_int != 0) {
		ptr_addr = &core_if->dev_if->dev_global_regs->deachint;
		DBG_USB_Print(DBG_USB,"DEACHINT	 @0x%08lX : 0x%08X\n",
			   (uint64_t)ptr_addr, REG_RD(ptr_addr));
		ptr_addr = &core_if->dev_if->dev_global_regs->deachintmsk;
		DBG_USB_Print(DBG_USB,"DEACHINTMSK	 @0x%08lX : 0x%08X\n",
			   (uint64_t)ptr_addr, REG_RD(ptr_addr));
		for (i = 0; i <= core_if->dev_if->num_in_eps; i++) {
			ptr_addr =
			    &core_if->dev_if->
			    dev_global_regs->diepeachintmsk[i];
			DBG_USB_Print(DBG_USB,"DIEPEACHINTMSK[%d]	 @0x%08lX : 0x%08X\n",
				   i, (uint64_t)ptr_addr,
				   REG_RD(ptr_addr));
		}
		for (i = 0; i <= core_if->dev_if->num_out_eps; i++) {
			ptr_addr =
			    &core_if->dev_if->
			    dev_global_regs->doepeachintmsk[i];
			DBG_USB_Print(DBG_USB,"DOEPEACHINTMSK[%d]	 @0x%08lX : 0x%08X\n",
				   i, (uint64_t)ptr_addr,
				   REG_RD(ptr_addr));
		}
	}
	for (i = 0; i <= core_if->dev_if->num_in_eps; i++) {
		DBG_USB_Print(DBG_USB,"Device IN EP %d Registers\n", i);
		ptr_addr = &core_if->dev_if->in_ep_regs[i]->diepctl;
		DBG_USB_Print(DBG_USB,"DIEPCTL	 @0x%08lX : 0x%08X\n",
			   (uint64_t)ptr_addr, REG_RD(ptr_addr));
		ptr_addr = &core_if->dev_if->in_ep_regs[i]->diepint;
		DBG_USB_Print(DBG_USB,"DIEPINT	 @0x%08lX : 0x%08X\n",
			   (uint64_t)ptr_addr, REG_RD(ptr_addr));
		ptr_addr = &core_if->dev_if->in_ep_regs[i]->dieptsiz;
		DBG_USB_Print(DBG_USB,"DIETSIZ	 @0x%08lX : 0x%08X\n",
			   (uint64_t)ptr_addr, REG_RD(ptr_addr));
		ptr_addr = &core_if->dev_if->in_ep_regs[i]->diepdma;
		DBG_USB_Print(DBG_USB,"DIEPDMA	 @0x%08lX : 0x%08X\n",
			   (uint64_t)ptr_addr, REG_RD(ptr_addr));
		ptr_addr = &core_if->dev_if->in_ep_regs[i]->dtxfsts;
		DBG_USB_Print(DBG_USB,"DTXFSTS	 @0x%08lX : 0x%08X\n",
			   (uint64_t)ptr_addr, REG_RD(ptr_addr));
		ptr_addr = &core_if->dev_if->in_ep_regs[i]->diepdmab;
		DBG_USB_Print(DBG_USB,"DIEPDMAB	 @0x%08lX : 0x%08X\n",
			   (uint64_t)ptr_addr, 0 /*REG_RD(addr) */ );
	}
	for (i = 0; i <= core_if->dev_if->num_out_eps; i++) {
		DBG_USB_Print(DBG_USB,"Device OUT EP %d Registers\n", i);
		ptr_addr = &core_if->dev_if->out_ep_regs[i]->doepctl;
		DBG_USB_Print(DBG_USB,"DOEPCTL	 @0x%08lX : 0x%08X\n",
			   (uint64_t)ptr_addr, REG_RD(ptr_addr));
		ptr_addr = &core_if->dev_if->out_ep_regs[i]->doepint;
		DBG_USB_Print(DBG_USB,"DOEPINT	 @0x%08lX : 0x%08X\n",
			   (uint64_t)ptr_addr, REG_RD(ptr_addr));
		ptr_addr = &core_if->dev_if->out_ep_regs[i]->doeptsiz;
		DBG_USB_Print(DBG_USB,"DOETSIZ	 @0x%08lX : 0x%08X\n",
			   (uint64_t)ptr_addr, REG_RD(ptr_addr));
		ptr_addr = &core_if->dev_if->out_ep_regs[i]->doepdma;
		DBG_USB_Print(DBG_USB,"DOEPDMA	 @0x%08lX : 0x%08X\n",
			   (uint64_t)ptr_addr, REG_RD(ptr_addr));
		if ((core_if->dma_enable) != 0) {	/* Don't access this register in SLAVE mode */
			ptr_addr = &core_if->dev_if->out_ep_regs[i]->doepdmab;
			DBG_USB_Print(DBG_USB,"DOEPDMAB	 @0x%08lX : 0x%08X\n",
				   (uint64_t)ptr_addr, REG_RD(ptr_addr));
		}
	}
	(void)ptr_addr;
}

/**
 * This functions reads the SPRAM and prints its content
 *
 * @param core_if Programming view of DWC_otg controller.
 */
void dwc_otg_dump_spram(dwc_otg_core_if_t const * core_if)
{
	volatile uint8_t *ptr_addr, *start_addr, *end_addr;

	DBG_USB_Print(DBG_USB,"SPRAM Data:\n");
	start_addr = (void *)core_if->core_global_regs;
	DBG_USB_Print(DBG_USB,"Base Address: 0x%8lX\n", (uint64_t)start_addr);
	start_addr += 0x00028000;
	end_addr = (void *)core_if->core_global_regs;
	end_addr += 0x000280e0;
	for (ptr_addr = start_addr; ptr_addr < end_addr; ptr_addr += 16) {
		DBG_USB_Print
		    (DBG_USB,"0x%8lX:\t%2X %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X\n",
		     (uint64_t)ptr_addr, ptr_addr[0], ptr_addr[1], ptr_addr[2], ptr_addr[3],
		     ptr_addr[4], ptr_addr[5], ptr_addr[6], ptr_addr[7], ptr_addr[8], ptr_addr[9],
		     ptr_addr[10], ptr_addr[11], ptr_addr[12], ptr_addr[13], ptr_addr[14], ptr_addr[15]
		    );
	}
	return;
}

/**
 * This function reads the core global registers and prints them
 *
 * @param core_if Programming view of DWC_otg controller.
 */
void dwc_otg_dump_global_registers(dwc_otg_core_if_t const * core_if)
{
	int32_t i, int_ep_num;
	volatile uint32_t *ptr_addr;
	char_t *txfsiz;

	DBG_USB_Print(DBG_USB,"Core Global Registers\n");
	ptr_addr = &core_if->core_global_regs->gotgctl;
	DBG_USB_Print(DBG_USB,"GOTGCTL	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->gotgint;
	DBG_USB_Print(DBG_USB,"GOTGINT	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->gahbcfg;
	DBG_USB_Print(DBG_USB,"GAHBCFG	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->gusbcfg;
	DBG_USB_Print(DBG_USB,"GUSBCFG	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->grstctl;
	DBG_USB_Print(DBG_USB,"GRSTCTL	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->gintsts;
	DBG_USB_Print(DBG_USB,"GINTSTS	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->gintmsk;
	DBG_USB_Print(DBG_USB,"GINTMSK	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->grxstsr;
	DBG_USB_Print(DBG_USB,"GRXSTSR	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->grxfsiz;
	DBG_USB_Print(DBG_USB,"GRXFSIZ	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->gnptxfsiz;
	DBG_USB_Print(DBG_USB,"GNPTXFSIZ @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->gnptxsts;
	DBG_USB_Print(DBG_USB,"GNPTXSTS	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->gi2cctl;
	DBG_USB_Print(DBG_USB,"GI2CCTL	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->gpvndctl;
	DBG_USB_Print(DBG_USB,"GPVNDCTL	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->ggpio;
	DBG_USB_Print(DBG_USB,"GGPIO	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->guid;
	DBG_USB_Print(DBG_USB,"GUID		 @0x%08lX : 0x%08X\n",
		   (uint64_t)ptr_addr, REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->gsnpsid;
	DBG_USB_Print(DBG_USB,"GSNPSID	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->ghwcfg1;
	DBG_USB_Print(DBG_USB,"GHWCFG1	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->ghwcfg2;
	DBG_USB_Print(DBG_USB,"GHWCFG2	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->ghwcfg3;
	DBG_USB_Print(DBG_USB,"GHWCFG3	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->ghwcfg4;
	DBG_USB_Print(DBG_USB,"GHWCFG4	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->glpmcfg;
	DBG_USB_Print(DBG_USB,"GLPMCFG	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->gpwrdn;
	DBG_USB_Print(DBG_USB,"GPWRDN	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->gdfifocfg;
	DBG_USB_Print(DBG_USB,"GDFIFOCFG	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	ptr_addr = &core_if->core_global_regs->hptxfsiz;
	DBG_USB_Print(DBG_USB,"HPTXFSIZ	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	if (core_if->en_multiple_tx_fifo == 0) {
		int_ep_num = core_if->hwcfg4.b.num_dev_perio_in_ep;
		txfsiz = "DPTXFSIZ";
	} else {
#endif
		int_ep_num = core_if->hwcfg4.b.num_in_eps;
		txfsiz = "DIENPTXF";
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	}
#endif
	for (i = 0; i < int_ep_num; i++) {
		ptr_addr = &core_if->core_global_regs->dtxfsiz[i];
		DBG_USB_Print(DBG_USB,"%s[%d] @0x%08lX : 0x%08X\n", txfsiz, i + 1,
			   (uint64_t)ptr_addr, REG_RD(ptr_addr));
	}
	ptr_addr = core_if->pcgcctl;
	DBG_USB_Print(DBG_USB,"PCGCCTL	 @0x%08lX : 0x%08X\n", (uint64_t)ptr_addr,
		   REG_RD(ptr_addr));
	(void)txfsiz;
	(void)ptr_addr;
}

/**
 * Flush a Tx FIFO.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param num Tx FIFO to flush.
 */
void dwc_otg_flush_tx_fifo(dwc_otg_core_if_t const * core_if, const uint32_t num)
{
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	volatile grstctl_t greset;
	int32_t count = 0;
	greset.d32 = 0 ;
	DBG_USB_Print(((DBG_CIL | DBG_PCDV) != 0), "Flush Tx FIFO %d\n", num);
	greset.b.txfflsh = 1;
	greset.b.txfnum = num;
	REG_WR(&global_regs->grstctl, greset.d32);
	do {
		greset.d32 = REG_RD(&global_regs->grstctl);
		if (++count > 10000) {
			DBG_Warn_Print("%s() HANG! GRSTCTL=%0x GNPTXSTS=0x%08x\n",
				 __func__, greset.d32,
				 REG_RD(&global_regs->gnptxsts));
			break;
		}
		dwc_udelay(1);
	} while (greset.b.txfflsh == 1);
	/* Wait for 3 PHY Clocks */
	dwc_udelay(1);
}

/**
 * Flush Rx FIFO.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
void dwc_otg_flush_rx_fifo(dwc_otg_core_if_t const * core_if)
{
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	volatile grstctl_t greset; 
	int32_t count = 0;
	greset.d32 = 0 ;
	DBG_USB_Print(((DBG_CIL | DBG_PCDV) != 0), "%s\n", __func__);
	greset.b.rxfflsh = 1;
	REG_WR(&global_regs->grstctl, greset.d32);

	do {
		greset.d32 = REG_RD(&global_regs->grstctl);
		if (++count > 10000) {
			DBG_Warn_Print("%s() HANG! GRSTCTL=%0x\n", __func__,
				 greset.d32);
			break;
		}
		dwc_udelay(1);
	} while (greset.b.rxfflsh == 1);
	/* Wait for 3 PHY Clocks */
	dwc_udelay(1);
}

/**
 * Do core a soft reset of the core.  Be careful with this because it
 * resets all the internal state machines of the core.
 */
void dwc_otg_core_reset(dwc_otg_core_if_t const * core_if)
{
	int32_t flag = 0;
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	volatile grstctl_t greset;
	int32_t count = 0;
	greset.d32 = 0 ;

	DBG_USB_Print(DBG_CILV, "%s\n", __func__);
	/* Wait for AHB master IDLE state. */
	do {
		dwc_udelay(10);
		greset.d32 = REG_RD(&global_regs->grstctl);
		if (++count > 100000) {
			DBG_Warn_Print("%s() HANG! AHB Idle GRSTCTL=%0x\n", __func__,
				 greset.d32);
			flag = 1;
			break;
		}
	}
	while (greset.b.ahbidle == 0);
	if(flag == 0){
	/* Wait for 3 PHY Clocks */
	dwc_mdelay(100);
	}
	return;
}

uint8_t dwc_otg_is_device_mode(dwc_otg_core_if_t * core_if)
{
	return (dwc_otg_mode(core_if) != DWC_HOST_MODE);
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
/**
 * Register HCD callbacks. The callbacks are used to start and stop
 * the HCD for interrupt processing.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param cb the HCD callback structure.
 * @param p pointer to be passed to callback function (usb_hcd*).
 */
void dwc_otg_cil_register_hcd_callbacks(dwc_otg_core_if_t * core_if,
					dwc_otg_cil_callbacks_t * cb, void *p)
{
	core_if->hcd_cb = cb;
	cb->p = p;
}

#endif
/**
 * Register PCD callbacks. The callbacks are used to start and stop
 * the PCD for interrupt processing.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param cb the PCD callback structure.
 * @param p pointer to be passed to callback function (pcd*).
 */
void dwc_otg_cil_register_pcd_callbacks(dwc_otg_core_if_t * core_if,
					dwc_otg_cil_callbacks_t * cb, void *p)
{
	core_if->pcd_cb = cb;
	cb->p = p;
}

static void dwc_otg_set_uninitialized(int32_t * p, int32_t size)
{
	int32_t i;
	for (i = 0; i < size; i++) {
		p[i] = -1;
	}
}

static int32_t dwc_otg_param_initialized(int32_t val)
{
	return val != -1;
}

static int32_t dwc_otg_setup_params(dwc_otg_core_if_t * core_if)
{
	int32_t i;
	int32_t retval = 0;
	int32_t flag = 0;
	gintsts_data_t gbl_int_sts;
	gbl_int_sts.d32 = REG_RD(&core_if->core_global_regs->gintsts);

	core_if->core_params = DWC_ALLOC(sizeof(*core_if->core_params));
	if (!core_if->core_params) {
		retval = -DWC_E_NO_MEMORY;
		flag = 1;
	}
	if(flag == 0){
		dwc_otg_set_uninitialized((int32_t *) core_if->core_params,
					  sizeof(*core_if->core_params) /
					  sizeof(int32_t));
		DBG_USB_Print(DBG_USB,"Setting default values for core params\n");
		dwc_otg_set_param_otg_cap(core_if, dwc_param_otg_cap_default);
		dwc_otg_set_param_dma_enable(core_if, dwc_param_dma_enable_default);
		dwc_otg_set_param_dma_desc_enable(core_if,
						  dwc_param_dma_desc_enable_default);
		dwc_otg_set_param_opt(core_if, dwc_param_opt_default);
		dwc_otg_set_param_dma_burst_size(core_if,
						 dwc_param_dma_burst_size_default);
		dwc_otg_set_param_host_support_fs_ls_low_power(core_if,
								   dwc_param_host_support_fs_ls_low_power_default);
		dwc_otg_set_param_enable_dynamic_fifo(core_if,
							  dwc_param_enable_dynamic_fifo_default);
		dwc_otg_set_param_data_fifo_size(core_if,
						 dwc_param_data_fifo_size_default);
		dwc_otg_set_param_dev_rx_fifo_size(core_if,
						   dwc_param_dev_rx_fifo_size_default);
		dwc_otg_set_param_dev_nperio_tx_fifo_size(core_if,
							  dwc_param_dev_nperio_tx_fifo_size_default);
		dwc_otg_set_param_host_rx_fifo_size(core_if,
							dwc_param_host_rx_fifo_size_default);
		dwc_otg_set_param_host_nperio_tx_fifo_size(core_if,
							   dwc_param_host_nperio_tx_fifo_size_default);
		dwc_otg_set_param_host_perio_tx_fifo_size(core_if,
							  dwc_param_host_perio_tx_fifo_size_default);
		dwc_otg_set_param_max_transfer_size(core_if,
							dwc_param_max_transfer_size_default);
		dwc_otg_set_param_max_packet_count(core_if,
						   dwc_param_max_packet_count_default);
		dwc_otg_set_param_host_channels(core_if,
						dwc_param_host_channels_default);
		dwc_otg_set_param_dev_endpoints(core_if,
						dwc_param_dev_endpoints_default);
		dwc_otg_set_param_phy_type(core_if, dwc_param_phy_type_default);
		dwc_otg_set_param_speed(core_if, dwc_param_speed_default);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
        dwc_otg_set_param_host_ls_low_power_phy_clk(core_if,
								dwc_param_host_ls_low_power_phy_clk_default);
        dwc_otg_set_param_i2c_enable(core_if, dwc_param_i2c_enable_default);
#endif
		dwc_otg_set_param_phy_ulpi_ddr(core_if, dwc_param_phy_ulpi_ddr_default);
		dwc_otg_set_param_phy_ulpi_ext_vbus(core_if,
							dwc_param_phy_ulpi_ext_vbus_default);
		dwc_otg_set_param_phy_utmi_width(core_if,
						 dwc_param_phy_utmi_width_default);
		dwc_otg_set_param_ts_dline(core_if, dwc_param_ts_dline_default);
		dwc_otg_set_param_ulpi_fs_ls(core_if, dwc_param_ulpi_fs_ls_default);
		dwc_otg_set_param_en_multiple_tx_fifo(core_if,
							  dwc_param_en_multiple_tx_fifo_default);
		
		if ((gbl_int_sts.b.curmode) != 0) {
			/* Force device mode to get power-on values of device FIFOs */
			gusbcfg_data_t gbl_usb_cfg;
			gbl_usb_cfg.d32 = 0;
			gbl_usb_cfg.d32 =  REG_RD(&core_if->core_global_regs->gusbcfg);
			gbl_usb_cfg.b.force_dev_mode = 1;
			REG_WR(&core_if->core_global_regs->gusbcfg, gbl_usb_cfg.d32);
			dwc_mdelay(100);
			for (i = 0; i < 15; i++) {
			dwc_otg_set_param_dev_perio_tx_fifo_size(core_if,
								 dwc_param_dev_perio_tx_fifo_size_default, i);
			}
			for (i = 0; i < 15; i++) {
				dwc_otg_set_param_dev_tx_fifo_size(core_if,
								   dwc_param_dev_tx_fifo_size_default, i);
			}
			gbl_usb_cfg.d32 =  REG_RD(&core_if->core_global_regs->gusbcfg);
			gbl_usb_cfg.b.force_dev_mode = 0;
			REG_WR(&core_if->core_global_regs->gusbcfg, gbl_usb_cfg.d32);
			dwc_mdelay(100);
		} else {
			for (i = 0; i < 15; i++) {
				dwc_otg_set_param_dev_perio_tx_fifo_size(core_if,
					dwc_param_dev_perio_tx_fifo_size_default, i);
			}
			for (i = 0; i < 15; i++) {
				dwc_otg_set_param_dev_tx_fifo_size(core_if,
					dwc_param_dev_tx_fifo_size_default, i);
			}
		}

		dwc_otg_set_param_thr_ctl(core_if, dwc_param_thr_ctl_default);
		dwc_otg_set_param_mpi_enable(core_if, dwc_param_mpi_enable_default);
		dwc_otg_set_param_pti_enable(core_if, dwc_param_pti_enable_default);
		dwc_otg_set_param_lpm_enable(core_if, dwc_param_lpm_enable_default);
			
		dwc_otg_set_param_besl_enable(core_if, dwc_param_besl_enable_default);
		dwc_otg_set_param_baseline_besl(core_if, dwc_param_baseline_besl_default);
		dwc_otg_set_param_deep_besl(core_if, dwc_param_deep_besl_default);
		
		dwc_otg_set_param_ic_usb_cap(core_if, dwc_param_ic_usb_cap_default);
		dwc_otg_set_param_tx_thr_length(core_if,
						dwc_param_tx_thr_length_default);
		dwc_otg_set_param_rx_thr_length(core_if,
						dwc_param_rx_thr_length_default);
		dwc_otg_set_param_ahb_thr_ratio(core_if,
						dwc_param_ahb_thr_ratio_default);
		dwc_otg_set_param_power_down(core_if, dwc_param_power_down_default);
		dwc_otg_set_param_reload_ctl(core_if, dwc_param_reload_ctl_default);
		dwc_otg_set_param_dev_out_nak(core_if, dwc_param_dev_out_nak_default);
		dwc_otg_set_param_cont_on_bna(core_if, dwc_param_cont_on_bna_default);
		dwc_otg_set_param_ahb_single(core_if, dwc_param_ahb_single_default);
		dwc_otg_set_param_otg_ver(core_if, dwc_param_otg_ver_default);
	}
	return retval;
}

int32_t dwc_otg_set_param_otg_cap(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t valid;
	int32_t retval = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 2)) {
		DBG_Warn_Print("Wrong value for otg_cap parameter\n");
		DBG_Warn_Print("otg_cap parameter must be 0,1 or 2\n");
		retval = -DWC_E_INVALID;
	}
	else {
		valid = 1;
		switch (val) {
		case DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE:
			if (core_if->hwcfg2.b.op_mode !=
				DWC_HWCFG2_OP_MODE_HNP_SRP_CAPABLE_OTG) {
				valid = 0; }
			break;
		case DWC_OTG_CAP_PARAM_SRP_ONLY_CAPABLE:
			if ((core_if->hwcfg2.b.op_mode !=
				 DWC_HWCFG2_OP_MODE_HNP_SRP_CAPABLE_OTG)
				&& (core_if->hwcfg2.b.op_mode !=
				DWC_HWCFG2_OP_MODE_SRP_ONLY_CAPABLE_OTG)
				&& (core_if->hwcfg2.b.op_mode !=
				DWC_HWCFG2_OP_MODE_SRP_CAPABLE_DEVICE)
				&& (core_if->hwcfg2.b.op_mode !=
				DWC_HWCFG2_OP_MODE_SRP_CAPABLE_HOST)) {
				valid = 0;
			}
			break;
		case DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE:
			/* always valid */
			break;
		default :
			break;
		}
		if (!valid) {
			if ((dwc_otg_param_initialized(core_if->core_params->otg_cap)) != 0) {
				DBG_Error_Print
					("%d invalid for otg_cap paremter. Check HW configuration.\n",
					 val);
			}
			val =
				(((core_if->hwcfg2.b.op_mode ==
				   DWC_HWCFG2_OP_MODE_HNP_SRP_CAPABLE_OTG)
				  || (core_if->hwcfg2.b.op_mode ==
				  DWC_HWCFG2_OP_MODE_SRP_ONLY_CAPABLE_OTG)
				  || (core_if->hwcfg2.b.op_mode ==
				  DWC_HWCFG2_OP_MODE_SRP_CAPABLE_DEVICE)
				  || (core_if->hwcfg2.b.op_mode ==
				  DWC_HWCFG2_OP_MODE_SRP_CAPABLE_HOST)) ?
				 DWC_OTG_CAP_PARAM_SRP_ONLY_CAPABLE :
				 DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->otg_cap = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_opt(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("Wrong value for opt parameter\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
		core_if->core_params->opt = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_dma_enable(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("Wrong value for dma enable\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
	if ((val == 1) && (core_if->hwcfg2.b.architecture == 0)) {
		if ((dwc_otg_param_initialized(core_if->core_params->dma_enable)) != 0) {
			DBG_Error_Print
			    ("%d invalid for dma_enable paremter. Check HW configuration.\n",
			     val);
		}
		val = 0;
		retval = -DWC_E_INVALID;
	}

	core_if->core_params->dma_enable = val;
	if (val == 0) {
		dwc_otg_set_param_dma_desc_enable(core_if, 0);
	}
	}
	return retval;
}

int32_t dwc_otg_get_param_dma_enable(dwc_otg_core_if_t const * core_if)
{
	return core_if->core_params->dma_enable;
}

int32_t dwc_otg_set_param_dma_desc_enable(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("Wrong value for dma_enable\n");
		DBG_Warn_Print("dma_desc_enable must be 0 or 1\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
		if ((val == 1)
			&& ((dwc_otg_get_param_dma_enable(core_if) == 0)
			|| (core_if->hwcfg4.b.desc_dma == 0))) {
			if ((dwc_otg_param_initialized
				(core_if->core_params->dma_desc_enable)) != 0) {
				DBG_Error_Print
					("%d invalid for dma_desc_enable paremter. Check HW configuration.\n",
					 val);
			}
			val = 0;
			retval = -DWC_E_INVALID;
		}
		core_if->core_params->dma_desc_enable = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_host_support_fs_ls_low_power(dwc_otg_core_if_t * core_if,
						   int32_t val)
{
	int32_t retval = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("Wrong value for host_support_fs_low_power\n");
		DBG_Warn_Print("host_support_fs_low_power must be 0 or 1\n");
		retval = -DWC_E_INVALID;
	} else {
	core_if->core_params->host_support_fs_ls_low_power = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_enable_dynamic_fifo(dwc_otg_core_if_t * core_if,
					  int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("Wrong value for enable_dynamic_fifo\n");
		DBG_Warn_Print("enable_dynamic_fifo must be 0 or 1\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
		if ((val == 1) && (core_if->hwcfg2.b.dynamic_fifo == 0)) {
			if ((dwc_otg_param_initialized
				(core_if->core_params->enable_dynamic_fifo)) != 0) {
				DBG_Error_Print
					("%d invalid for enable_dynamic_fifo paremter. Check HW configuration.\n",
					 val);
			}
			val = 0;
			retval = -DWC_E_INVALID;
		}
		core_if->core_params->enable_dynamic_fifo = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_data_fifo_size(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 32, 32768)) {
		DBG_Warn_Print("Wrong value for data_fifo_size\n");
		DBG_Warn_Print("data_fifo_size must be 32-32768\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
		if (val > core_if->hwcfg3.b.dfifo_depth) {
			if  ((dwc_otg_param_initialized
				(core_if->core_params->data_fifo_size)) != 0) {
				DBG_Error_Print
					("%d invalid for data_fifo_size parameter. Check HW configuration.\n",
					 val);
			}
			val = core_if->hwcfg3.b.dfifo_depth;
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->data_fifo_size = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_dev_rx_fifo_size(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 16, 32768)) {
		DBG_Warn_Print("Wrong value for dev_rx_fifo_size\n");
		DBG_Warn_Print("dev_rx_fifo_size must be 16-32768\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
		if ((uint32_t)val > (REG_RD(&core_if->core_global_regs->grxfsiz))) {
			if ((dwc_otg_param_initialized((int32_t)core_if->core_params->dev_rx_fifo_size)) != 0) {
			DBG_Warn_Print("%d invalid for dev_rx_fifo_size parameter\n", val);
			}
			val = (int32_t)REG_RD(&core_if->core_global_regs->grxfsiz);
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->dev_rx_fifo_size = (uint32_t)val;
	}
	return retval;
}

int32_t dwc_otg_set_param_dev_nperio_tx_fifo_size(dwc_otg_core_if_t * core_if,
					      int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 16, 32768)) {
		DBG_Warn_Print("Wrong value for dev_nperio_tx_fifo\n");
		DBG_Warn_Print("dev_nperio_tx_fifo must be 16-32768\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
		if ((uint32_t)val > ((REG_RD(&core_if->core_global_regs->gnptxfsiz) >> 16))) {
			if ((dwc_otg_param_initialized
				(core_if->core_params->dev_nperio_tx_fifo_size)) != 0) {
				DBG_Error_Print
					("%d invalid for dev_nperio_tx_fifo_size. Check HW configuration.\n",
					 val);
			}
			val =(int32_t)
				(REG_RD(&core_if->core_global_regs->gnptxfsiz) >>
				 16);
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->dev_nperio_tx_fifo_size = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_host_rx_fifo_size(dwc_otg_core_if_t * core_if,
					int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 16, 32768)) {
		DBG_Warn_Print("Wrong value for host_rx_fifo_size\n");
		DBG_Warn_Print("host_rx_fifo_size must be 16-32768\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
		if ((uint32_t)val > (REG_RD(&core_if->core_global_regs->grxfsiz))) {
			if ((dwc_otg_param_initialized
				(core_if->core_params->host_rx_fifo_size)) != 0) {
				DBG_Error_Print
					("%d invalid for host_rx_fifo_size. Check HW configuration.\n",
					 val);
			}
			val = (int32_t)REG_RD(&core_if->core_global_regs->grxfsiz);
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->host_rx_fifo_size = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_host_nperio_tx_fifo_size(dwc_otg_core_if_t * core_if,
					       int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 16, 32768)) {
		DBG_Warn_Print("Wrong value for host_nperio_tx_fifo_size\n");
		DBG_Warn_Print("host_nperio_tx_fifo_size must be 16-32768\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
		if ((uint32_t)val > ((REG_RD(&core_if->core_global_regs->gnptxfsiz) >> 16))) {
			if ((dwc_otg_param_initialized
				(core_if->core_params->host_nperio_tx_fifo_size)) != 0) {
				DBG_Error_Print
					("%d invalid for host_nperio_tx_fifo_size. Check HW configuration.\n",
					 val);
			}
			val =(int32_t)
				(REG_RD(&core_if->core_global_regs->gnptxfsiz) >>
				 16);
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->host_nperio_tx_fifo_size = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_host_perio_tx_fifo_size(dwc_otg_core_if_t * core_if,
					      int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 16, 32768)) {
		DBG_Warn_Print("Wrong value for host_perio_tx_fifo_size\n");
		DBG_Warn_Print("host_perio_tx_fifo_size must be 16-32768\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0) {
		if ((uint32_t)val > (((core_if->hptxfsiz.d32) >> 16))) {
			if ((dwc_otg_param_initialized
				(core_if->core_params->host_perio_tx_fifo_size)) != 0) {
				DBG_Error_Print
					("%d invalid for host_perio_tx_fifo_size. Check HW configuration.\n",
					 val);
			}
			val = ((int32_t)((core_if->hptxfsiz.d32) >> 16));
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->host_perio_tx_fifo_size = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_max_transfer_size(dwc_otg_core_if_t * core_if,
					int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 2047, 524288)) {
		DBG_Warn_Print("Wrong value for max_transfer_size\n");
		DBG_Warn_Print("max_transfer_size must be 2047-524288\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
		if (val >= (1 << (core_if->hwcfg3.b.xfer_size_cntr_width + 11))) {
			if ((dwc_otg_param_initialized
				((int32_t)core_if->core_params->max_transfer_size)) != 0) {
				DBG_Error_Print
					("%d invalid for max_transfer_size. Check HW configuration.\n",
					 val);
			}
			val =
				((1 << (core_if->hwcfg3.b.packet_size_cntr_width + 11)) -
				 1);
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->max_transfer_size = (uint32_t)val;
	}
	return retval;
}

int32_t dwc_otg_set_param_max_packet_count(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 15, 511)) {
		DBG_Warn_Print("Wrong value for max_packet_count\n");
		DBG_Warn_Print("max_packet_count must be 15-511\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
		if (val > (1 << (core_if->hwcfg3.b.packet_size_cntr_width + 4))) {
			if ((dwc_otg_param_initialized
				(core_if->core_params->max_packet_count)) != 0) {
				DBG_Error_Print
					("%d invalid for max_packet_count. Check HW configuration.\n",
					 val);
			}
			val =
				((1 << (core_if->hwcfg3.b.packet_size_cntr_width + 4)) - 1);
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->max_packet_count = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_host_channels(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 1, 16)) {
		DBG_Warn_Print("Wrong value for host_channels\n");
		DBG_Warn_Print("host_channels must be 1-16\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
		if (val > (core_if->hwcfg2.b.num_host_chan + 1)) {
			if ((dwc_otg_param_initialized
				(core_if->core_params->host_channels)) != 0) {
				DBG_Error_Print
					("%d invalid for host_channels. Check HW configurations.\n",
					 val);
			}
			val = (core_if->hwcfg2.b.num_host_chan + 1);
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->host_channels = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_dev_endpoints(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 1, 15)) {
		DBG_Warn_Print("Wrong value for dev_endpoints\n");
		DBG_Warn_Print("dev_endpoints must be 1-15\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
		if (val > (core_if->hwcfg2.b.num_dev_ep)) {
			if ((dwc_otg_param_initialized
				(core_if->core_params->dev_endpoints)) != 0) {
				DBG_Error_Print
					("%d invalid for dev_endpoints. Check HW configurations.\n",
					 val);
			}
			val = (int32_t)core_if->hwcfg2.b.num_dev_ep;
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->dev_endpoints = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_phy_type(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t valid = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 2)) {
		DBG_Warn_Print("Wrong value for phy_type\n");
		DBG_Warn_Print("phy_type must be 0,1 or 2\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0) {
#ifndef NO_FS_PHY_HW_CHECKS
		if ((val == DWC_PHY_TYPE_PARAM_UTMI) &&
			((core_if->hwcfg2.b.hs_phy_type == 1) ||
			 (core_if->hwcfg2.b.hs_phy_type == 3))) {
			valid = 1;
		} else if ((val == DWC_PHY_TYPE_PARAM_ULPI) &&
			   ((core_if->hwcfg2.b.hs_phy_type == 2) ||
				(core_if->hwcfg2.b.hs_phy_type == 3))) {
			valid = 1;
		} else if ((val == DWC_PHY_TYPE_PARAM_FS) &&
			   (core_if->hwcfg2.b.fs_phy_type == 1)) {
			valid = 1;
		}
		if (!valid) {
			if ((dwc_otg_param_initialized(core_if->core_params->phy_type)) != 0) {
				DBG_Error_Print
					("%d invalid for phy_type. Check HW configurations.\n",
					 val);
			}
			if ((core_if->hwcfg2.b.hs_phy_type) != 0) {
				if ((core_if->hwcfg2.b.hs_phy_type == 3) ||
					(core_if->hwcfg2.b.hs_phy_type == 1)) {
					val = DWC_PHY_TYPE_PARAM_UTMI;
				} else {
					val = DWC_PHY_TYPE_PARAM_ULPI;
				}
			}
			retval = -DWC_E_INVALID;
		}
#endif
		core_if->core_params->phy_type = val;
	}
	return retval;
}

int32_t dwc_otg_get_param_phy_type(dwc_otg_core_if_t const * core_if)
{
	return core_if->core_params->phy_type;
}

int32_t dwc_otg_set_param_speed(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("Wrong value for speed parameter\n");
		DBG_Warn_Print("max_speed parameter must be 0 or 1\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
		if ((val == 0)
			&& (dwc_otg_get_param_phy_type(core_if) == DWC_PHY_TYPE_PARAM_FS)) {
			if ((dwc_otg_param_initialized(core_if->core_params->speed)) != 0) {
				DBG_Error_Print
					("%d invalid for speed paremter. Check HW configuration.\n",
					 val);
			}
			val =
				((dwc_otg_get_param_phy_type(core_if) ==
				 DWC_PHY_TYPE_PARAM_FS) ? 1 : 0);
			retval = -DWC_E_INVALID;
		}
		core_if->core_params->speed = val;
	}
	return retval;
}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
int32_t dwc_otg_set_param_host_ls_low_power_phy_clk(dwc_otg_core_if_t * core_if,
						int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print
		    ("Wrong value for host_ls_low_power_phy_clk parameter\n");
		DBG_Warn_Print("host_ls_low_power_phy_clk must be 0 or 1\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
		if ((val == DWC_HOST_LS_LOW_POWER_PHY_CLK_PARAM_48MHZ)
			&& (dwc_otg_get_param_phy_type(core_if) == DWC_PHY_TYPE_PARAM_FS)) {
			if ((dwc_otg_param_initialized
				(core_if->core_params->host_ls_low_power_phy_clk)) != 0) {
				DBG_Error_Print
					("%d invalid for host_ls_low_power_phy_clk. Check HW configuration.\n",
					 val);
			}
			val =
				(dwc_otg_get_param_phy_type(core_if) ==
				 DWC_PHY_TYPE_PARAM_FS) ?
				DWC_HOST_LS_LOW_POWER_PHY_CLK_PARAM_6MHZ :
				DWC_HOST_LS_LOW_POWER_PHY_CLK_PARAM_48MHZ;
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->host_ls_low_power_phy_clk = val;
	}
	return retval;
}
#endif

int32_t dwc_otg_set_param_phy_ulpi_ddr(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t return_val = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("Wrong value for phy_ulpi_ddr\n");
		DBG_Warn_Print("phy_upli_ddr must be 0 or 1\n");
		return_val = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0) {
	core_if->core_params->phy_ulpi_ddr = val;
	}
	return return_val;
}

int32_t dwc_otg_set_param_phy_ulpi_ext_vbus(dwc_otg_core_if_t * core_if,
					int32_t val)
{
	int32_t return_val = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("Wrong valaue for phy_ulpi_ext_vbus\n");
		DBG_Warn_Print("phy_ulpi_ext_vbus must be 0 or 1\n");
		return_val = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0) {
		core_if->core_params->phy_ulpi_ext_vbus = val;
	}
	return return_val;
}

int32_t dwc_otg_set_param_phy_utmi_width(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t return_val = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 8, 8) && DWC_OTG_PARAM_TEST(val, 16, 16)) {
		DBG_Warn_Print("Wrong valaue for phy_utmi_width\n");
		DBG_Warn_Print("phy_utmi_width must be 8 or 16\n");
		return_val = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0) {
		core_if->core_params->phy_utmi_width = val;
	}
	return return_val;
}

int32_t dwc_otg_set_param_ulpi_fs_ls(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t return_val = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("Wrong valaue for ulpi_fs_ls\n");
		DBG_Warn_Print("ulpi_fs_ls must be 0 or 1\n");
		return_val = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0) {
		core_if->core_params->ulpi_fs_ls = val;
	}
	return return_val;
}

int32_t dwc_otg_set_param_ts_dline(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t return_val = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("Wrong valaue for ts_dline\n");
		DBG_Warn_Print("ts_dline must be 0 or 1\n");
		return_val = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
		core_if->core_params->ts_dline = val;
	}
	return return_val;
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
int32_t dwc_otg_set_param_i2c_enable(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("Wrong valaue for i2c_enable\n");
		DBG_Warn_Print("i2c_enable must be 0 or 1\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
#ifndef NO_FS_PHY_HW_CHECK
		if ((val == 1) && (core_if->hwcfg3.b.i2c == 0)) {
			if ((dwc_otg_param_initialized(core_if->core_params->i2c_enable)) != 0) {
				DBG_Error_Print
					("%d invalid for i2c_enable. Check HW configuration.\n",
					 val);
			}
			val = 0;
			retval = -DWC_E_INVALID;
		}
#endif

		core_if->core_params->i2c_enable = val;
	}
	return retval;
}
#endif

int32_t dwc_otg_set_param_dev_perio_tx_fifo_size(dwc_otg_core_if_t * core_if,
					     int32_t val, int32_t fifo_num)
{
	int32_t retval = 0;
	int32_t flag = 0;
	/*gintsts_data_t gintsts; */
	/*gintsts.d32 = REG_RD(&core_if->core_global_regs->gintsts); */

	if (DWC_OTG_PARAM_TEST(val, 4, 768)) {
		DBG_Warn_Print("Wrong value for dev_perio_tx_fifo_size\n");
		DBG_Warn_Print("dev_perio_tx_fifo_size must be 4-768\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){ 
		if ((uint32_t)val >(
			(REG_RD(&core_if->core_global_regs->dtxfsiz[fifo_num]) >> 16))) {
			DBG_Warn_Print("Value is larger then power-on FIFO size\n");
			if ((dwc_otg_param_initialized
				(core_if->core_params->dev_perio_tx_fifo_size[fifo_num])) != 0) {
				DBG_Error_Print
					("`%d' invalid for parameter `dev_perio_fifo_size_%d'. Check HW configuration.\n",
					 val, fifo_num);
			}
			val = (int32_t)(REG_RD(&core_if->core_global_regs->dtxfsiz[fifo_num]) >> 16);
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->dev_perio_tx_fifo_size[fifo_num] = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_en_multiple_tx_fifo(dwc_otg_core_if_t * core_if,
					  int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("Wrong valaue for en_multiple_tx_fifo,\n");
		DBG_Warn_Print("en_multiple_tx_fifo must be 0 or 1\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0){
		if ((val == 1) && (core_if->hwcfg4.b.ded_fifo_en == 0)) {
			if ((dwc_otg_param_initialized
				(core_if->core_params->en_multiple_tx_fifo)) != 0) {
				DBG_Error_Print
					("%d invalid for parameter en_multiple_tx_fifo. Check HW configuration.\n",
					 val);
			}
			val = 0;
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->en_multiple_tx_fifo = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_dev_tx_fifo_size(dwc_otg_core_if_t * core_if, int32_t val,
				       int32_t fifo_num)
{
	int32_t retval = 0;
	int32_t flag = 0;
	fifosize_data_t txfifosize;
	txfifosize.d32 = REG_RD(&core_if->core_global_regs->dtxfsiz[fifo_num]);	

	if (DWC_OTG_PARAM_TEST(val, 16, 32768)) {
		DBG_Warn_Print("Wrong value for dev_tx_fifo_size\n");
		DBG_Warn_Print("dev_tx_fifo_size must be 16-32768\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag ==0){
		if (val > txfifosize.b.depth) {
			DBG_Warn_Print("Value is larger then power-on FIFO size\n");
			if ((dwc_otg_param_initialized
				(core_if->core_params->dev_tx_fifo_size[fifo_num])) != 0) {
				DBG_Error_Print
					("`%d' invalid for parameter `dev_tx_fifo_size_%d'. Check HW configuration.\n",
					 val, fifo_num);
			}
			val = (int32_t)txfifosize.b.depth;
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->dev_tx_fifo_size[fifo_num] = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_thr_ctl(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 7)) {
		DBG_Warn_Print("Wrong value for thr_ctl\n");
		DBG_Warn_Print("thr_ctl must be 0-7\n");
		retval = -DWC_E_INVALID;
	}
	else {
		if ((val != 0) &&
			((!(dwc_otg_get_param_dma_enable(core_if))) ||
			 (!(core_if->hwcfg4.b.ded_fifo_en)))) {
			if (dwc_otg_param_initialized((int32_t)core_if->core_params->thr_ctl) != 0) {
				DBG_Error_Print
					("%d invalid for parameter thr_ctl. Check HW configuration.\n",
					 val);
			}
			val = 0;
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->thr_ctl = (uint32_t)val;
	}
	return retval;
}


int32_t dwc_otg_set_param_lpm_enable(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("Wrong value for lpm_enable\n");
		DBG_Warn_Print("lpm_enable must be 0 or 1\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0) {
		if (val && (!(core_if->hwcfg3.b.otg_lpm_en))) {
			if ((dwc_otg_param_initialized(core_if->core_params->lpm_enable)) != 0) {
				DBG_Error_Print
					("%d invalid for parameter lpm_enable. Check HW configuration.\n",
					 val);
			}
			val = 0;
			retval = -DWC_E_INVALID;
		}

		core_if->core_params->lpm_enable = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_besl_enable(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("Wrong value for besl_enable\n");
		DBG_Warn_Print("besl_enable must be 0 or 1\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag ==0){
		core_if->core_params->besl_enable = val;
		
		if(val != 0)
		{
			retval += dwc_otg_set_param_lpm_enable(core_if,val);
		}
	}
	return retval;
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
int32_t dwc_otg_get_param_besl_enable(dwc_otg_core_if_t const * core_if)
{
	return core_if->core_params->besl_enable;
}
#endif

int32_t dwc_otg_set_param_baseline_besl(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 15)) {
		DBG_Warn_Print("Wrong value for baseline_besl\n");
		DBG_Warn_Print("baseline_besl must be 0-15\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if (flag == 0){
		core_if->core_params->baseline_besl = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_deep_besl(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 15)) {
		DBG_Warn_Print("Wrong value for deep_besl\n");
		DBG_Warn_Print("deep_besl must be 0-15\n");
		retval = -DWC_E_INVALID;
		flag=1;
	}
	if(flag == 0){
		core_if->core_params->deep_besl = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_tx_thr_length(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t flag = 0;
	int32_t return_val = 0;
	if (DWC_OTG_PARAM_TEST(val, 8, 128)) {
		DBG_Warn_Print("Wrong valaue for tx_thr_length\n");
		DBG_Warn_Print("tx_thr_length must be 8 - 128\n");
		return_val = -DWC_E_INVALID;
		flag=1;
	}
	if(flag == 0){
		core_if->core_params->tx_thr_length = val;
	}
	return return_val;
}

int32_t dwc_otg_set_param_rx_thr_length(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t flag = 0;
	int32_t return_val = 0;
	if (DWC_OTG_PARAM_TEST(val, 8, 128)) {
		DBG_Warn_Print("Wrong valaue for rx_thr_length\n");
		DBG_Warn_Print("rx_thr_length must be 8 - 128\n");
		return_val = -DWC_E_INVALID;
		flag=1;
	}
	if(flag == 0){
		core_if->core_params->rx_thr_length = val;
	}
	return return_val;
}

int32_t dwc_otg_set_param_dma_burst_size(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t flag = 0;
	int32_t return_val = 0;
	if (DWC_OTG_PARAM_TEST(val, 1, 1) &&
	    DWC_OTG_PARAM_TEST(val, 4, 4) &&
	    DWC_OTG_PARAM_TEST(val, 8, 8) &&
	    DWC_OTG_PARAM_TEST(val, 16, 16) &&
	    DWC_OTG_PARAM_TEST(val, 32, 32) &&
	    DWC_OTG_PARAM_TEST(val, 64, 64) &&
	    DWC_OTG_PARAM_TEST(val, 128, 128) &&
	    DWC_OTG_PARAM_TEST(val, 256, 256)) {
		DBG_Warn_Print("`%d' invalid for parameter `dma_burst_size'\n", val);
		return_val = -DWC_E_INVALID;
		flag=1;
	}
	if(flag == 0){
		core_if->core_params->dma_burst_size = (uint32_t)val;
	}
	return return_val;
}

int32_t dwc_otg_set_param_pti_enable(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("`%d' invalid for parameter `pti_enable'\n", val);
		retval = -DWC_E_INVALID;
	}
	else{
		if (val && (core_if->snpsid < OTG_CORE_REV_2_72a)) {
			if ((dwc_otg_param_initialized(core_if->core_params->pti_enable)) != 0) {
				DBG_Error_Print
					("%d invalid for parameter pti_enable. Check HW configuration.\n",
					 val);
			}
			retval = -DWC_E_INVALID;
			val = 0;
		}
		core_if->core_params->pti_enable = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_mpi_enable(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t flag = 0;
	int32_t retval = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("`%d' invalid for parameter `mpi_enable'\n", val);
		retval = -DWC_E_INVALID;
		flag=1;
	}
	if(flag == 0){
		if (val && (core_if->hwcfg2.b.multi_proc_int == 0)) {
			if ((dwc_otg_param_initialized(core_if->core_params->mpi_enable)) != 0) {
				DBG_Error_Print
					("%d invalid for parameter mpi_enable. Check HW configuration.\n",
					 val);
			}
			retval = -DWC_E_INVALID;
			val = 0;
		}
		core_if->core_params->mpi_enable = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_ic_usb_cap(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t flag = 0;
	int32_t retval = 0;
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("`%d' invalid for parameter `ic_usb_cap'\n", val);
		DBG_Warn_Print("ic_usb_cap must be 0 or 1\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}

	if(flag == 0) {
		if (val && (core_if->hwcfg2.b.otg_enable_ic_usb == 0)) {
			if ((dwc_otg_param_initialized(core_if->core_params->ic_usb_cap)) != 0) {
				DBG_Error_Print
					("%d invalid for parameter ic_usb_cap. Check HW configuration.\n",
					 val);
			}
			retval = -DWC_E_INVALID;
			val = 0;
		}
		core_if->core_params->ic_usb_cap = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_ahb_thr_ratio(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t flag = 0;
	int32_t retval = 0;
	int32_t valid = 1;

	if (DWC_OTG_PARAM_TEST(val, 0, 3)) {
		DBG_Warn_Print("`%d' invalid for parameter `ahb_thr_ratio'\n", val);
		DBG_Warn_Print("ahb_thr_ratio must be 0 - 3\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}

	if(flag == 0) {

		if (valid == 0) {
			if ( (dwc_otg_param_initialized
				(core_if->core_params->ahb_thr_ratio)) != 0) {
				DBG_Error_Print
					("%d invalid for parameter ahb_thr_ratio. Check HW configuration.\n",
					 val);
			}
			retval = -DWC_E_INVALID;
			val = 0;
		}

		core_if->core_params->ahb_thr_ratio = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_power_down(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t flag =0;
	int32_t retval = 0;
	int32_t valid = 1;
	hwcfg4_data_t hw_cfg4;
	hw_cfg4.d32 = 0;
	hw_cfg4.d32 = REG_RD(&core_if->core_global_regs->ghwcfg4);
	
	if (DWC_OTG_PARAM_TEST(val, 0, 3)) {
		DBG_Warn_Print("`%d' invalid for parameter `power_down'\n", val);
		DBG_Warn_Print("power_down must be 0 - 2\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if(flag == 0) {
		if ((val == 2) && (core_if->snpsid < OTG_CORE_REV_2_91a)) {
			valid = 0;
		}
		if ((val == 3)
			&& ((core_if->snpsid < OTG_CORE_REV_3_00a)
			|| (hw_cfg4.b.xhiber == 0))) {
			valid = 0;
		}
		if (valid == 0) {
			if ((dwc_otg_param_initialized(core_if->core_params->power_down)) != 0) {
				DBG_Error_Print
					("%d invalid for parameter power_down. Check HW configuration.\n",
					 val);
			}
			retval = -DWC_E_INVALID;
			val = 0;
		}
		core_if->core_params->power_down = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_reload_ctl(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t valid = 1;
	int32_t flag = 0;
	

	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("`%d' invalid for parameter `reload_ctl'\n", val);
		DBG_Warn_Print("reload_ctl must be 0 or 1\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}

	if(flag == 0) {
		if ((val == 1) && (core_if->snpsid < OTG_CORE_REV_2_92a)) {
			valid = 0;
		}
		if (valid == 0) {
			if ((dwc_otg_param_initialized(core_if->core_params->reload_ctl)) != 0) {
				DBG_Error_Print("%d invalid for parameter reload_ctl."
					  "Check HW configuration.\n", val);
			}
			retval = -DWC_E_INVALID;
			val = 0;
		}
		core_if->core_params->reload_ctl = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_dev_out_nak(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t valid = 1;
	int32_t flag = 0;

	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("`%d' invalid for parameter `dev_out_nak'\n", val);
		DBG_Warn_Print("dev_out_nak must be 0 or 1\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}

	if (flag == 0) {
		if ((val == 1) && (((core_if->snpsid) < OTG_CORE_REV_2_93a) ||
				   (!(core_if->core_params->dma_desc_enable)))) {
			valid = 0;
		}
		if (valid == 0) {
			if ( (dwc_otg_param_initialized(core_if->core_params->dev_out_nak)) != 0) {
				DBG_Error_Print("%d invalid for parameter dev_out_nak."
					  "Check HW configuration.\n", val);
			}
			retval = -DWC_E_INVALID;
			val = 0;
		}
		core_if->core_params->dev_out_nak = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_cont_on_bna(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t valid = 1;
	int32_t flag = 0;

	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("`%d' invalid for parameter `cont_on_bna'\n", val);
		DBG_Warn_Print("cont_on_bna must be 0 or 1\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if (flag == 0) {

		if ((val == 1) && (((core_if->snpsid) < OTG_CORE_REV_2_94a) ||
				   (!(core_if->core_params->dma_desc_enable)))) {
			valid = 0;
		}
		if (valid == 0) {
			if ((dwc_otg_param_initialized(core_if->core_params->cont_on_bna)) != 0) {
				DBG_Error_Print("%d invalid for parameter cont_on_bna."
					"Check HW configuration.\n", val);
			}
			retval = -DWC_E_INVALID;
			val = 0;
		}
		core_if->core_params->cont_on_bna = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_ahb_single(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t valid = 1;
	int32_t flag = 0;
		
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("`%d' invalid for parameter `ahb_single'\n", val);
		DBG_Warn_Print("ahb_single must be 0 or 1\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if (flag == 0) {
		if ((val == 1) && (core_if->snpsid < OTG_CORE_REV_2_94a)) {
			valid = 0;
		}
		if (valid == 0) {
			if ((dwc_otg_param_initialized(core_if->core_params->ahb_single)) != 0) {
				DBG_Error_Print("%d invalid for parameter ahb_single."
					  "Check HW configuration.\n", val);
			}
			retval = -DWC_E_INVALID;
			val = 0;
		}
		core_if->core_params->ahb_single = val;
	}
	return retval;
}

int32_t dwc_otg_set_param_otg_ver(dwc_otg_core_if_t * core_if, int32_t val)
{
	int32_t retval = 0;
	int32_t flag = 0;
	
	if (DWC_OTG_PARAM_TEST(val, 0, 1)) {
		DBG_Warn_Print("`%d' invalid for parameter `otg_ver'\n", val);
		DBG_Warn_Print
		    ("otg_ver must be 0(for OTG 1.3 support) or 1(for OTG 2.0 support)\n");
		retval = -DWC_E_INVALID;
		flag = 1;
	}
	if (flag == 0) {
		core_if->core_params->otg_ver = val;
	}
	return retval;
}

uint32_t dwc_otg_get_hnpstatus(dwc_otg_core_if_t const * core_if)
{
	gotgctl_data_t otgctl;
	otgctl.d32 = REG_RD(&core_if->core_global_regs->gotgctl);
	return otgctl.b.hstnegscs;
}

uint32_t dwc_otg_get_srpstatus(dwc_otg_core_if_t const * core_if)
{
	gotgctl_data_t otgctl;
	otgctl.d32 = REG_RD(&core_if->core_global_regs->gotgctl);
	return otgctl.b.sesreqscs;
}

void dwc_otg_set_hnpreq(dwc_otg_core_if_t * core_if, uint32_t val)
{
	if(core_if->otg_ver == 0) {
		gotgctl_data_t otgctl;
		otgctl.d32 = REG_RD(&core_if->core_global_regs->gotgctl);
		otgctl.b.hnpreq = val;
		REG_WR(&core_if->core_global_regs->gotgctl, otgctl.d32);
	} else {
		core_if->otg_sts = (uint8_t)val;
	}
}

uint32_t dwc_otg_get_mode(dwc_otg_core_if_t const * core_if)
{
	gintsts_data_t gbl_int_sts;
	gbl_int_sts.d32 = REG_RD(&core_if->core_global_regs->gintsts);
	return gbl_int_sts.b.curmode;
}

uint32_t dwc_otg_get_hnpcapable(dwc_otg_core_if_t const * core_if)
{
	gusbcfg_data_t usbcfg;
	usbcfg.d32 = REG_RD(&core_if->core_global_regs->gusbcfg);
	return usbcfg.b.hnpcap;
}

void dwc_otg_set_hnpcapable(dwc_otg_core_if_t * core_if, uint32_t val)
{
	gusbcfg_data_t usbcfg;
	usbcfg.d32 = REG_RD(&core_if->core_global_regs->gusbcfg);
	usbcfg.b.hnpcap = val;
	REG_WR(&core_if->core_global_regs->gusbcfg, usbcfg.d32);
}

uint32_t dwc_otg_get_srpcapable(dwc_otg_core_if_t const * core_if)
{
	gusbcfg_data_t usbcfg;
	usbcfg.d32 = REG_RD(&core_if->core_global_regs->gusbcfg);
	return usbcfg.b.srpcap;
}

void dwc_otg_set_srpcapable(dwc_otg_core_if_t * core_if, uint32_t val)
{
	gusbcfg_data_t usbcfg;
	usbcfg.d32 = REG_RD(&core_if->core_global_regs->gusbcfg);
	usbcfg.b.srpcap = val;
	REG_WR(&core_if->core_global_regs->gusbcfg, usbcfg.d32);
}

uint32_t dwc_otg_get_devspeed(dwc_otg_core_if_t const * core_if)
{
	dcfg_data_t dev_cfg;
	dev_cfg.d32 = REG_RD(&core_if->dev_if->dev_global_regs->dcfg);
	return dev_cfg.b.devspd;
}

void dwc_otg_set_devspeed(dwc_otg_core_if_t * core_if, uint32_t val)
{
	dcfg_data_t dev_cfg;
	dev_cfg.d32 = REG_RD(&core_if->dev_if->dev_global_regs->dcfg);
	dev_cfg.b.devspd = val;
	REG_WR(&core_if->dev_if->dev_global_regs->dcfg, dev_cfg.d32);
}

uint32_t dwc_otg_get_enumspeed(dwc_otg_core_if_t const * core_if)
{
	dsts_data_t dev_sts;
	dev_sts.d32 = REG_RD(&core_if->dev_if->dev_global_regs->dsts);
	return dev_sts.b.enumspd;
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
/**
 * Start the SRP timer to detect when the SRP does not complete within
 * 6 seconds.
 *
 * @param core_if the pointer to core_if strucure.
 */
void dwc_otg_pcd_start_srp_timer(dwc_otg_core_if_t * core_if)
{
	core_if->srp_timer_started = 1;
}

void dwc_otg_initiate_srp(void * p)
{
	int32_t flag=0;
	dwc_otg_core_if_t * ptr_core_if = p;
	uint32_t *ptr_addr = (uint32_t *) & (ptr_core_if->core_global_regs->gotgctl);
	gotgctl_data_t mem;
	gotgctl_data_t val;

	val.d32 = REG_RD(ptr_addr);
	if (val.b.sesreq != 0) {
		DBG_Error_Print("Session Request Already active!\n");
		flag = 1;
	}

	if(flag == 0){
		/*DWC_INFO("Session Request Initated\n"); */	/*NOTICE */
		mem.d32 = REG_RD(ptr_addr);
		mem.b.sesreq = 1;
		REG_WR(ptr_addr, mem.d32);

		/* Start the SRP timer */
		dwc_otg_pcd_start_srp_timer(ptr_core_if);
	}
	return;
}

#endif

int32_t dwc_otg_check_haps_status(dwc_otg_core_if_t const * core_if)
{
   int32_t retval = 0;
   int32_t return_val ;
   if(REG_RD(&core_if->core_global_regs->gsnpsid) == 0xffffffffU)
   {
		return_val = -1;
   } else {
		return_val = retval;
   } 
   return return_val;
}
