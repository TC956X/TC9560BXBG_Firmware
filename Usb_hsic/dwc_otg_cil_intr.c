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
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_cil_intr.c $
 * $Revision: 1.13 $
 * $Date: 2016/07/18 23:50:47 $
 * $Change: 2207267 $
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

/** @ file
 *
 * The Core Interface Layer provides basic services for accessing and
 * managing the DWC_otg hardware. These services are used by both the
 * Host Controller Driver and the Peripheral Controller Driver.
 *
 * This file contains the Common Interrupt handlers.
 */
 
#include "dwc_otg_pcd.h"

const char_t *op_state_str(dwc_otg_core_if_t const * core_if)
{
	return ((core_if->op_state == A_HOST) ? "a_host" :
		((core_if->op_state == A_SUSPEND) ? "a_suspend" :
		 ((core_if->op_state == A_PERIPHERAL) ? "a_peripheral" :
		  ((core_if->op_state == B_PERIPHERAL) ? "b_peripheral" :
		   ((core_if->op_state == B_HOST) ? "b_host" : "unknown")))));
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
/** This function will log a debug message
 *
 * @param core_if Programming view of DWC_otg controller.
 */
uint32_t dwc_otg_handle_mode_mismatch_intr(dwc_otg_core_if_t * core_if)
{
	gintsts_data_t gbl_int_sts;
	DBG_Warn_Print("Mode Mismatch Interrupt: currently in %s mode\n",
		 dwc_otg_mode(core_if) ? "Host" : "Device");

	/* Clear interrupt */
	gbl_int_sts.d32 = 0;
	gbl_int_sts.b.modemismatch = 1;
	REG_WR(&core_if->core_global_regs->gintsts, gbl_int_sts.d32);
	return 1;
}

/**
 * This function handles the OTG Interrupts. It reads the OTG
 * Interrupt Register (GOTGINT) to determine what interrupt has
 * occurred.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
uint32_t dwc_otg_handle_otg_intr(dwc_otg_core_if_t * core_if)
{
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	gotgint_data_t gbl_otg_int;
	gotgctl_data_t gbl_otg_ctl;

	gbl_otg_int.d32 = REG_RD(&global_regs->gotgint);
	gbl_otg_ctl.d32 = REG_RD(&global_regs->gotgctl);
	DBG_USB_Print(DBG_CIL, "++OTG Interrupt gotgint=%0x [%s]\n", gbl_otg_int.d32,
		    op_state_str(core_if));

	if (gbl_otg_int.b.sesenddet != 0) {
		DBG_USB_Print(DBG_USB, " ++OTG Interrupt: "
			    "Session End Detected++ (%s)\n",
			    op_state_str(core_if));
		gbl_otg_int.d32 = REG_RD(&global_regs->gotgctl);

		if (core_if->op_state == B_HOST) {
			cil_pcd_start(core_if);
			core_if->op_state = B_PERIPHERAL;
		} else {
			/* If not B_HOST and Device HNP still set. HNP
			 * Did not succeed!*/
			if (gbl_otg_ctl.b.devhnpen != 0) {
				DBG_USB_Print(DBG_USB, "Session End Detected\n");
				DBG_USB_Print(DBG_USB,"Device Not Connected/Responding!\n");
			}
			/* If Session End Detected the B-Cable has
			 * been disconnected. */
			/* Reset PCD and Gadget driver to a
			 * clean state. */
			core_if->lx_state = DWC_OTG_L0;
			cil_pcd_stop(core_if);
			if (core_if->otg_ver != 0) {
				/** PET testing*/
				gbl_otg_ctl.d32 = 0;
				gbl_otg_ctl.b.devhnpen = 1;
				DWC_MODIFY_REG32(&global_regs->gotgctl, gbl_otg_ctl.d32, 0);
				if (core_if->test_mode == 6) {
					core_if->test_mode = 0;
				}

			}
		}
/* exit_interrupt: */
		if (core_if->otg_ver == 0) {
			gbl_otg_ctl.d32 = 0;
			gbl_otg_ctl.b.devhnpen = 1;
			DWC_MODIFY_REG32(&global_regs->gotgctl, gbl_otg_ctl.d32, 0);
		}
	}
	if (gbl_otg_int.b.sesreqsucstschng != 0) {
		DBG_USB_Print(DBG_USB, " ++OTG Interrupt: "
			    "Session Reqeust Success Status Change++\n");
		gbl_otg_ctl.d32 = REG_RD(&global_regs->gotgctl);
		if (gbl_otg_ctl.b.sesreqscs != 0) {

			if ((core_if->core_params->phy_type ==
			     DWC_PHY_TYPE_PARAM_FS) && (core_if->core_params->i2c_enable)) {
				core_if->srp_success = 1;
			} else {
				cil_pcd_resume(core_if);
				/* Clear Session Request */
				gbl_otg_ctl.d32 = 0;
				gbl_otg_ctl.b.sesreq = 1;
				DWC_MODIFY_REG32(&global_regs->gotgctl,
						 gbl_otg_ctl.d32, 0);
			}
		}
	}
	if (gbl_otg_int.b.hstnegsucstschng != 0) {
		/* Print statements during the HNP interrupt handling
		 * can cause it to fail.*/
		gbl_otg_ctl.d32 = REG_RD(&global_regs->gotgctl);
		/* WA for 3.00a- HW is not setting cur_mode, even sometimes
		 * this does not help*/
		if (core_if->snpsid >= OTG_CORE_REV_3_00a) {
			dwc_udelay(100);
		}
		if (gbl_otg_ctl.b.hstnegscs != 0) {
		} else {
			gbl_otg_ctl.d32 = 0;
			gbl_otg_ctl.b.hnpreq = 1;
			gbl_otg_ctl.b.devhnpen = 1;
			DWC_MODIFY_REG32(&global_regs->gotgctl, gbl_otg_ctl.d32, 0);
			DBG_USB_Print(DBG_USB, "HNP Failed\n");
			DBG_USB_Print(DBG_USB,"Device Not Connected/Responding\n");
		}
	}
	if (gbl_otg_int.b.hstnegdet != 0) {
		/* The disconnect interrupt is set at the same time as
		 * Host Negotiation Detected.  During the mode
		 * switch all interrupts are cleared so the disconnect
		 * interrupt handler will not get executed.
		 */
		DBG_USB_Print(DBG_USB, " ++OTG Interrupt: "
			    "Host Negotiation Detected++ (%s)\n",
			    (dwc_otg_is_device_mode(core_if) ? "Device" : "Host" ));
			DBG_USB_Print(DBG_USB, "a_suspend->a_peripheral (%d)\n",
				    core_if->op_state);
			cil_hcd_disconnect(core_if);
			cil_pcd_start(core_if);
			core_if->op_state = A_PERIPHERAL;
	}
	if (gbl_otg_int.b.adevtoutchng != 0) {
		DBG_USB_Print(DBG_USB, " ++OTG Interrupt: "
			    "A-Device Timeout Change++\n");
	}
	if (gbl_otg_int.b.debdone != 0) {
		DBG_USB_Print(DBG_USB, " ++OTG Interrupt: " "Debounce Done++\n");
		/* Need to power off VBUS after 10s if OTG2 non-hnp capable host*/
		if (((core_if->otg_ver) && (core_if->op_state)) == (A_PERIPHERAL)) {
			DBG_USB_Print(DBG_USB, "a_peripheral->a_host\n");
			/* Clear the a_peripheral flag, back to a_host. */
			cil_pcd_stop(core_if);
			cil_hcd_start(core_if);
			core_if->op_state = A_HOST;
		}

		if(core_if->otg_ver == 1) {
			cil_hcd_session_start(core_if);
		}
	}

	/* Clear GOTGINT */
	REG_WR(&core_if->core_global_regs->gotgint, gbl_otg_int.d32);

	return 1;
}

/**
 * This function handles the Connector ID Status Change Interrupt.  It
 * reads the OTG Interrupt Register (GOTCTL) to determine whether this
 * is a Device to Host Mode transition or a Host Mode to Device
 * Transition. 
 *
 * This only occurs when the cable is connected/removed from the PHY
 * connector.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
uint32_t dwc_otg_handle_conn_id_status_change_intr(dwc_otg_core_if_t * core_if)
{

	/*
	 * Need to disable SOF interrupt immediately. If switching from device
	 * to host, the PCD interrupt handler won't handle the interrupt if
	 * host mode is already set. The HCD interrupt handler won't get
	 * called if the HCD state is HALT. This means that the interrupt does
	 * not get handled and Linux complains loudly.
	 */
	gintmsk_data_t gbl_int_msk; 
	gintsts_data_t gbl_int_sts; 
	gbl_int_msk.d32 = 0 ;
	gbl_int_sts.d32 = 0 ;

	gbl_int_msk.b.sofintr = 1;
	DWC_MODIFY_REG32(&core_if->core_global_regs->gintmsk, gbl_int_msk.d32, 0);

	DBG_USB_Print(DBG_CIL,
		    " ++Connector ID Status Change Interrupt++  (%s)\n",
		    (dwc_otg_is_device_mode(core_if) ? "Device" : "Host"));


	/* Set flag and clear interrupt */
	gbl_int_sts.b.conidstschng = 1;
	REG_WR(&core_if->core_global_regs->gintsts, gbl_int_sts.d32);

	return 1;
}

/**
 * This interrupt indicates that a device is initiating the Session
 * Request Protocol to request the host to turn on bus power so a new
 * session can begin. The handler responds by turning on bus power. If
 * the DWC_otg controller is in low power mode, the handler brings the
 * controller out of low power mode before turning on bus power.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
uint32_t dwc_otg_handle_session_req_intr(dwc_otg_core_if_t * core_if)
{
	gintsts_data_t gbl_int_sts;
	gotgctl_data_t gbl_otg_ctl;

	DBG_USB_Print(DBG_USB, "++Session Request Interrupt++\n");

	gbl_otg_ctl.d32 = 0 ;
	DBG_USB_Print(DBG_PCD, "SRP: Device mode\n");
	gbl_otg_ctl.d32 =
	REG_RD(&core_if->core_global_regs->gotgctl);
	/*if (gotgctl.b.sesreqscs)*/
	DBG_USB_Print(DBG_USB,"SRP Success\n");
	/* 	else */
	DBG_USB_Print(DBG_USB,"SRP Fail\n");

	if (core_if->otg_ver != 0) {
		gbl_otg_ctl.d32 = 0 ;	
		gbl_otg_ctl.b.devhnpen = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gotgctl, gbl_otg_ctl.d32, 0);
	}

	/* Clear interrupt */
	gbl_int_sts.d32 = 0;
	gbl_int_sts.b.sessreqintr = 1;
	REG_WR(&core_if->core_global_regs->gintsts, gbl_int_sts.d32);

	return 1;
}

/**
 * This interrupt indicates that the DWC_otg controller has detected a
 * resume or remote wakeup sequence. If the DWC_otg controller is in
 * low power mode, the handler must brings the controller out of low
 * power mode. The controller automatically begins resume
 * signaling. The handler schedules a time to stop resume signaling.
 */
uint32_t dwc_otg_handle_wakeup_detected_intr(dwc_otg_core_if_t * core_if)
{
	gintsts_data_t gbl_int_sts;
	dctl_data_t dev_ctl;

	DBG_USB_Print(DBG_USB,
		    "++Resume and Remote Wakeup Detected Interrupt++\n");

	DBG_USB_Print(DBG_USB,"%s lxstate = %d\n",(char_t *) __func__, core_if->lx_state);

			dev_ctl.d32 = 0 ;
		DBG_USB_Print(DBG_PCD, "DSTS=0x%0x\n",
			    REG_RD(&core_if->dev_if->dev_global_regs->
					   dsts));
		if (core_if->lx_state == DWC_OTG_L2) {
#ifdef PARTIAL_POWER_DOWN
			if (core_if->hwcfg4.b.power_optimiz) {
				pcgcctl_data_t power = {.d32 = 0 };

				power.d32 = REG_RD(core_if->pcgcctl);
				DBG_USB_Print(DBG_CIL, "PCGCCTL=%0x\n",
					    power.d32);

				power.b.stoppclk = 0;
				REG_WR(core_if->pcgcctl, power.d32);

				power.b.pwrclmp = 0;
				REG_WR(core_if->pcgcctl, power.d32);

				power.b.rstpdwnmodule = 0;
				REG_WR(core_if->pcgcctl, power.d32);
			}
#endif
			/* Clear the Remote Wakeup Signaling */
			dev_ctl.b.rmtwkupsig = 1;
			DWC_MODIFY_REG32(&core_if->dev_if->dev_global_regs->
					 dctl, dev_ctl.d32, 0);

			if ((core_if->pcd_cb) && (core_if->pcd_cb->resume_wakeup)) {
				core_if->pcd_cb->resume_wakeup(core_if->pcd_cb->p);
			}
		} else {
			glpmcfg_data_t lpmcfg;
			pcgcctl_data_t pcgc_ctl; 
			pcgc_ctl.d32 = 0 ;

			lpmcfg.d32 =
			    REG_RD(&core_if->core_global_regs->glpmcfg);
			lpmcfg.b.hird_thres = (((USIGN)lpmcfg.b.hird_thres) & (~(1U << 4U)));	
	       	lpmcfg.b.en_utmi_sleep = 0; 

			/* Clear Enbl_L1Gating bit. */
			pcgc_ctl.b.enbl_sleep_gating = 1;
			DWC_MODIFY_REG32(core_if->pcgcctl, pcgc_ctl.d32,0);

			REG_WR(&core_if->core_global_regs->glpmcfg,
					lpmcfg.d32);
		}
		/** Change to L0 state*/
		core_if->lx_state = DWC_OTG_L0;

	/* Clear interrupt */
	gbl_int_sts.d32 = 0;
	gbl_int_sts.b.wkupintr = 1;
	REG_WR(&core_if->core_global_regs->gintsts, gbl_int_sts.d32);

	return 1;
}

/**
 * This interrupt indicates that the Wakeup Logic has detected a
 * Device disconnect.
 */
static int32_t dwc_otg_handle_pwrdn_disconnect_intr(dwc_otg_core_if_t * core_if)
{
	gpwrdn_data_t gbl_pwr_dn;
	gpwrdn_data_t gpwrdn_temp; 
	gbl_pwr_dn.d32 = 0 ;
	gpwrdn_temp.d32 = 0 ;
	gpwrdn_temp.d32 = REG_RD(&core_if->core_global_regs->gpwrdn);

	 DBG_USB_Print(DBG_USB,"%s called\n",(char_t *) __FUNCTION__);

	if (!core_if->hibernation_suspend) {
		 DBG_USB_Print(DBG_USB,"Already exited from Hibernation\n");
	}
	else{
		/* Switch on the voltage to the core */
		gbl_pwr_dn.b.pwrdnswtch = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
		dwc_udelay(10);

		/* Reset the core */
		gbl_pwr_dn.d32 = 0;
		gbl_pwr_dn.b.pwrdnrstn = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
		dwc_udelay(10);

		/* Disable power clamps */
		gbl_pwr_dn.d32 = 0;
		gbl_pwr_dn.b.pwrdnclmp = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);

		/* Remove reset the core signal */
		gbl_pwr_dn.d32 = 0;
		gbl_pwr_dn.b.pwrdnrstn = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, 0, gbl_pwr_dn.d32);
		dwc_udelay(10);

		/* Disable PMU interrupt */
		gbl_pwr_dn.d32 = 0;
		gbl_pwr_dn.b.pmuintsel = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);

		core_if->hibernation_suspend = 0;

		/* Disable PMU */
		gbl_pwr_dn.d32 = 0;
		gbl_pwr_dn.b.pmuactv = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
		dwc_udelay(10);

		if (gpwrdn_temp.b.idsts != 0) {
			core_if->op_state = B_PERIPHERAL;
			dwc_otg_core_init(core_if);
			dwc_otg_enable_global_interrupts(core_if);
			cil_pcd_start(core_if);
		} else {
			core_if->op_state = A_HOST;
			dwc_otg_core_init(core_if);
			dwc_otg_enable_global_interrupts(core_if);
			cil_hcd_start(core_if);
		}
	}

	return 1;
}

/**
 * This interrupt indicates that the Wakeup Logic has detected a
 * remote wakeup sequence.
 */
static int32_t dwc_otg_handle_pwrdn_wakeup_detected_intr(dwc_otg_core_if_t * core_if)
{
	gpwrdn_data_t gbl_pwr_dn; 
	gbl_pwr_dn.d32 = 0 ;
	DBG_USB_Print(DBG_USB,
		    "++Powerdown Remote Wakeup Detected Interrupt++\n");

	if (!core_if->hibernation_suspend) {
		 DBG_USB_Print(DBG_USB,"Already exited from Hibernation\n");
	}
	else {
		gbl_pwr_dn.d32 = REG_RD(&core_if->core_global_regs->gpwrdn);
		if (gbl_pwr_dn.b.idsts != 0) {	/* Device Mode */
			if ((core_if->power_down == 2)
				&& (core_if->hibernation_suspend == 1)) {
				dwc_otg_device_hibernation_restore(core_if, 0, 0);
			}
		} else {
			if ((core_if->power_down == 2)
				&& (core_if->hibernation_suspend == 1)) {
				dwc_otg_host_hibernation_restore(core_if, 1, 0);
			}
		}
	}
	return 1;
}

static int32_t dwc_otg_handle_pwrdn_idsts_change(dwc_otg_device_t const * otg_dev)
{
	gpwrdn_data_t gbl_pwr_dn;
	gpwrdn_data_t gpwrdn_temp; 
	dwc_otg_core_if_t *core_if_ptr = otg_dev->core_if;
	gbl_pwr_dn.d32 = 0 ;
	gpwrdn_temp.d32 = 0 ;

	DBG_USB_Print(DBG_USB, "%s called\n", __FUNCTION__);
	gpwrdn_temp.d32 = REG_RD(&core_if_ptr->core_global_regs->gpwrdn);
	if (core_if_ptr->power_down == 2) {
		if (!core_if_ptr->hibernation_suspend) {
			 DBG_USB_Print(DBG_USB,"Already exited from Hibernation\n");
		}
		else {
			DBG_USB_Print(DBG_USB, "Exit from hibernation on ID sts change\n");
			/* Switch on the voltage to the core */
			gbl_pwr_dn.b.pwrdnswtch = 1;
			DWC_MODIFY_REG32(&core_if_ptr->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
			dwc_udelay(10);

			/* Reset the core */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.pwrdnrstn = 1;
			DWC_MODIFY_REG32(&core_if_ptr->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
			dwc_udelay(10);

			/* Disable power clamps */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.pwrdnclmp = 1;
			DWC_MODIFY_REG32(&core_if_ptr->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);

			/* Remove reset the core signal */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.pwrdnrstn = 1;
			DWC_MODIFY_REG32(&core_if_ptr->core_global_regs->gpwrdn, 0, gbl_pwr_dn.d32);
			dwc_udelay(10);

			/* Disable PMU interrupt */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.pmuintsel = 1;
			DWC_MODIFY_REG32(&core_if_ptr->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);

			/*Indicates that we are exiting from hibernation */
			core_if_ptr->hibernation_suspend = 0;

			/* Disable PMU */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.pmuactv = 1;
			DWC_MODIFY_REG32(&core_if_ptr->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
			dwc_udelay(10);

			gbl_pwr_dn.d32 = core_if_ptr->gr_backup->gpwrdn_local;
			if (gbl_pwr_dn.b.dis_vbus == 1) {
				gbl_pwr_dn.d32 = 0;
				gbl_pwr_dn.b.dis_vbus = 1;
				DWC_MODIFY_REG32(&core_if_ptr->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
			}

			if (gpwrdn_temp.b.idsts != 0) {
				core_if_ptr->op_state = B_PERIPHERAL;
				dwc_otg_core_init(core_if_ptr);
				dwc_otg_enable_global_interrupts(core_if_ptr);
				cil_pcd_start(core_if_ptr);
			} else {
				core_if_ptr->op_state = A_HOST;
				dwc_otg_core_init(core_if_ptr);
				dwc_otg_enable_global_interrupts(core_if_ptr);
				cil_hcd_start(core_if_ptr);
			}
		}
	}

	return 1;
}

static int32_t dwc_otg_handle_pwrdn_session_change(dwc_otg_core_if_t * core_if)
{
	gpwrdn_data_t gbl_pwr_dn ;
	int32_t otg_cap_param = core_if->core_params->otg_cap;
	gbl_pwr_dn.d32 = 0 ;
	DBG_USB_Print(DBG_USB, "%s called\n", __FUNCTION__);

	gbl_pwr_dn.d32 = REG_RD(&core_if->core_global_regs->gpwrdn);
	if (core_if->power_down == 2) {
		if (!core_if->hibernation_suspend) {
			 DBG_USB_Print(DBG_USB,"Already exited from Hibernation\n");
		}
		else if (((otg_cap_param != DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE) ||
		     (otg_cap_param != DWC_OTG_CAP_PARAM_SRP_ONLY_CAPABLE)) &&
		    (gbl_pwr_dn.b.bsessvld == 0)) {
			/* Save gpwrdn register for further usage if stschng interrupt */
			core_if->gr_backup->gpwrdn_local =
			    REG_RD(&core_if->core_global_regs->gpwrdn);
			/*Exit from ISR and wait for stschng interrupt with bsessvld = 1 */
		}
		else{
			/* Switch on the voltage to the core */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.pwrdnswtch = 1;
			DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
			dwc_udelay(10);

			/* Reset the core */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.pwrdnrstn = 1;
			DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
			dwc_udelay(10);

			/* Disable power clamps */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.pwrdnclmp = 1;
			DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);

			/* Remove reset the core signal */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.pwrdnrstn = 1;
			DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, 0, gbl_pwr_dn.d32);
			dwc_udelay(10);

			/* Disable PMU interrupt */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.pmuintsel = 1;
			DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
			dwc_udelay(10);

			/*Indicates that we are exiting from hibernation */
			core_if->hibernation_suspend = 0;

			/* Disable PMU */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.pmuactv = 1;
			DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
			dwc_udelay(10);

			core_if->op_state = B_PERIPHERAL;
			dwc_otg_core_init(core_if);
			dwc_otg_enable_global_interrupts(core_if);
			cil_pcd_start(core_if);

			if ((otg_cap_param == DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE) ||
				(otg_cap_param == DWC_OTG_CAP_PARAM_SRP_ONLY_CAPABLE)) {
				/*
				 * Initiate SRP after initial ADP probe.
				 */
				dwc_otg_initiate_srp(core_if);
			}
		}
	}
	return 1;
}

/**
 * This interrupt indicates that the Wakeup Logic has detected a
 * status change either on IDDIG or BSessVld.
 */
static int32_t dwc_otg_handle_pwrdn_stschng_intr(dwc_otg_device_t const * otg_dev)
{
	int32_t retval=0;
	gpwrdn_data_t gbl_pwr_dn; 
	gpwrdn_data_t gpwrdn_temp; 
	dwc_otg_core_if_t *core_if_ptr = otg_dev->core_if;
	gbl_pwr_dn.d32 = 0 ;
	gpwrdn_temp.d32 = 0 ;

	DBG_USB_Print(DBG_CIL, "%s called\n", __FUNCTION__);

	if (core_if_ptr->power_down == 2) {
		if (core_if_ptr->hibernation_suspend <= 0) {
			 DBG_USB_Print(DBG_USB,"Already exited from Hibernation\n");
		} else {
			gpwrdn_temp.d32 = core_if_ptr->gr_backup->gpwrdn_local;
		
	
			gbl_pwr_dn.d32 = REG_RD(&core_if_ptr->core_global_regs->gpwrdn);

			if ((gbl_pwr_dn.b.idsts ^ gpwrdn_temp.b.idsts) != 0) {
				retval = dwc_otg_handle_pwrdn_idsts_change(otg_dev);
			} else if ((gbl_pwr_dn.b.bsessvld ^ gpwrdn_temp.b.bsessvld) != 0) {
				retval = dwc_otg_handle_pwrdn_session_change(core_if_ptr);
			}
		}
	}		
	return retval;
}

/**
 * This interrupt indicates that the Wakeup Logic has detected a
 * SRP.
 */
/** This interrupt indicates that restore command after Hibernation
 * was completed by the core. */
int32_t dwc_otg_handle_restore_done_intr(dwc_otg_core_if_t const * core_if)
{
	pcgcctl_data_t pcgc_ctl;
	DBG_USB_Print(DBG_USB, "++Restore Done Interrupt++\n");

	/* TODO De-assert restore signal. 8.a */
	pcgc_ctl.d32 = REG_RD(core_if->pcgcctl);
	if (pcgc_ctl.b.restoremode == 1) {
		gintmsk_data_t gbl_int_msk;
		gbl_int_msk.d32 = 0 ;
		/*
		 * If restore mode is Remote Wakeup,
		 * unmask Remote Wakeup interrupt.
		 */
		gbl_int_msk.b.wkupintr = 1;
		DWC_MODIFY_REG32(&core_if->core_global_regs->gintmsk,
				 0, gbl_int_msk.d32);
	}

	return 1;
}

/**
 * This interrupt indicates that a device has been disconnected from
 * the root port.
 */
uint32_t dwc_otg_handle_disconnect_intr(dwc_otg_core_if_t * core_if)
{
	gintsts_data_t gbl_int_sts;

	DBG_USB_Print(DBG_USB, "++Disconnect Detected Interrupt++ (%s) %s\n",
		    (dwc_otg_is_device_mode(core_if) ? "Device" : "Host"),
		    op_state_str(core_if));

	if (core_if->op_state == B_HOST) {
		/* If in device mode Disconnect and stop the HCD, then
		 * start the PCD. */
		cil_hcd_disconnect(core_if);
		cil_pcd_start(core_if);
		core_if->op_state = B_PERIPHERAL;
	} else {
		gotgctl_data_t gbl_otgctl; 
		gbl_otgctl.d32 = 0 ;
		gbl_otgctl.d32 =
		    REG_RD(&core_if->core_global_regs->gotgctl);
		if (gbl_otgctl.b.hstsethnpen == 1) {
			/* Do nothing, if HNP in process the OTG
			 * interrupt "Host Negotiation Detected"
			 * interrupt will do the mode switch.
			 */
		} else if (gbl_otgctl.b.devhnpen == 0) {
			/* If in device mode Disconnect and stop the HCD, then
			 * start the PCD. */
			cil_hcd_disconnect(core_if);
			cil_pcd_start(core_if);
			core_if->op_state = B_PERIPHERAL;
		} else {
			DBG_USB_Print(DBG_USB, "!a_peripheral && !devhnpen\n");
		}
		}

	/* Change to L3(OFF) state */
	core_if->lx_state = DWC_OTG_L3;

	gbl_int_sts.d32 = 0;
	gbl_int_sts.b.disconnect = 1;
	REG_WR(&core_if->core_global_regs->gintsts, gbl_int_sts.d32);
	return 1;
}

/**
 * This interrupt indicates that SUSPEND state has been detected on
 * the USB.
 *
 * For HNP the USB Suspend interrupt signals the change from
 * "a_peripheral" to "a_host".
 *
 * When power management is enabled the core will be put in low power
 * mode.
 */

static int32_t dwc_otg_handle_xhib_exit_intr(dwc_otg_core_if_t * core_if)
{
	gpwrdn_data_t gbl_pwr_dn; 
	pcgcctl_data_t pcgc_ctl; 
	gahbcfg_data_t gbl_ahb_cfg;
	gbl_pwr_dn.d32 = 0 ;
	pcgc_ctl.d32 = 0 ;
	gbl_ahb_cfg.d32 = 0 ;

	dwc_udelay(10);

	/* Program GPIO register while entering to xHib */
	REG_WR(&core_if->core_global_regs->ggpio, 0x0);

	pcgc_ctl.d32 = core_if->gr_backup->xhib_pcgcctl;
	pcgc_ctl.b.extnd_hiber_pwrclmp = 0;
	REG_WR(core_if->pcgcctl, pcgc_ctl.d32);
	dwc_udelay(10);

	gbl_pwr_dn.d32 = core_if->gr_backup->xhib_gpwrdn;
	gbl_pwr_dn.b.restore = 1;
	REG_WR(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32);
	dwc_udelay(10);

	restore_lpm_i2c_regs(core_if);

	pcgc_ctl.d32 = core_if->gr_backup->pcgcctl_local & (0x3FFFFU << 14);
	pcgc_ctl.b.max_xcvrselect = 1;
	pcgc_ctl.b.ess_reg_restored = 0;
	pcgc_ctl.b.extnd_hiber_switch = 0;
	pcgc_ctl.b.extnd_hiber_pwrclmp = 0;
	pcgc_ctl.b.enbl_extnd_hiber = 1;
	REG_WR(core_if->pcgcctl, pcgc_ctl.d32);

	gbl_ahb_cfg.d32 = core_if->gr_backup->gahbcfg_local;
	gbl_ahb_cfg.b.glblintrmsk = 1;
	REG_WR(&core_if->core_global_regs->gahbcfg, gbl_ahb_cfg.d32);

	REG_WR(&core_if->core_global_regs->gintsts, 0xFFFFFFFFU);
	REG_WR(&core_if->core_global_regs->gintmsk, 0x1 << 16);

	REG_WR(&core_if->core_global_regs->gusbcfg,
			core_if->gr_backup->gusbcfg_local);
	REG_WR(&core_if->dev_if->dev_global_regs->dcfg,
			core_if->dr_backup->dcfg);

	pcgc_ctl.d32 = 0;
	pcgc_ctl.d32 = core_if->gr_backup->pcgcctl_local & (0x3FFFFU << 14);
	pcgc_ctl.b.max_xcvrselect = 1;
	pcgc_ctl.d32 |= 0x608;
	REG_WR(core_if->pcgcctl, pcgc_ctl.d32);
	dwc_udelay(10);

	pcgc_ctl.d32 = 0;
	pcgc_ctl.d32 = core_if->gr_backup->pcgcctl_local & (0x3FFFFU << 14);
	pcgc_ctl.b.max_xcvrselect = 1;
	pcgc_ctl.b.ess_reg_restored = 1;
	pcgc_ctl.b.enbl_extnd_hiber = 1;
	pcgc_ctl.b.rstpdwnmodule = 1;
	pcgc_ctl.b.restoremode = 1;
	REG_WR(core_if->pcgcctl, pcgc_ctl.d32);

	DBG_USB_Print(DBG_USB, "%s called\n", __FUNCTION__);

	return 1;
}

static int32_t dwc_otg_handle_pwrdn_srp_intr(dwc_otg_core_if_t * core_if)
{
	gpwrdn_data_t gbl_pwr_dn;
	gbl_pwr_dn.d32 = 0 ;

	
	if (core_if->power_down == 2) {
		if (!core_if->hibernation_suspend) {
			;
		}
		else {
#ifdef DWC_DEV_SRPCAP
			if (core_if->pwron_timer_started) {
				core_if->pwron_timer_started = 0;
			}
#endif

			/* Switch on the voltage to the core */
			gbl_pwr_dn.b.pwrdnswtch = 1;
			DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
			dwc_udelay(10);

			/* Reset the core */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.pwrdnrstn = 1;
			DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
			dwc_udelay(10);

			/* Disable power clamps */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.pwrdnclmp = 1;
			DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);

			/* Remove reset the core signal */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.pwrdnrstn = 1;
			DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, 0, gbl_pwr_dn.d32);
			dwc_udelay(10);

			/* Disable PMU interrupt */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.pmuintsel = 1;
			DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);

			/* Indicates that we are exiting from hibernation */
			core_if->hibernation_suspend = 0;

			/* Disable PMU */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.pmuactv = 1;
			DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
			dwc_udelay(10);

			/* Programm Disable VBUS to 0 */
			gbl_pwr_dn.d32 = 0;
			gbl_pwr_dn.b.dis_vbus = 1;
			DWC_MODIFY_REG32(&core_if->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);

			/*Initialize the core as Host */
			core_if->op_state = A_HOST;
			dwc_otg_core_init(core_if);
			dwc_otg_enable_global_interrupts(core_if);
			cil_hcd_start(core_if);
		}
	}
        
	return 1;
}

#endif
/**
 * This interrupt indicates that SUSPEND state has been detected on
 * the USB.
 *
 * For HNP the USB Suspend interrupt signals the change from
 * "a_peripheral" to "a_host".
 *
 * When power management is enabled the core will be put in low power
 * mode.
 */
int32_t dwc_otg_handle_usb_suspend_intr(dwc_otg_core_if_t * core_if)
{
	gintsts_data_t gbl_int_sts;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	dcfg_data_t dev_cfg;
#endif
	dsts_data_t dev_sts;
	int32_t flag = 0;
	
	DBG_USB_Print(DBG_PCD,"USB SUSPEND\n");
	DBG_USB_Print(DBG_PCD,"lx_state = %08x\n",core_if->lx_state);
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	if ((core_if->otg_ver == 1) && (core_if->op_state == A_PERIPHERAL)) {
			DBG_USB_Print(DBG_PCD,"A_PERIPHERAL\n");
		core_if->lx_state = DWC_OTG_L2;

		/* Clear interrupt */
		gbl_int_sts.d32 = 0;
		gbl_int_sts.b.usbsuspend = 1;
		REG_WR(&core_if->core_global_regs->gintsts, gbl_int_sts.d32);
		flag = 1;
	}else {
#endif
		/* Check the Device status register to determine if the Suspend
		 * state is active. */
	
		dev_sts.d32 =
		    REG_RD(&core_if->dev_if->dev_global_regs->dsts);
		DBG_USB_Print(DBG_PCD, "DSTS=0x%0x\n", dev_sts.d32);
		DBG_USB_Print(DBG_PCD, "DSTS.Suspend Status=%d "
			    "HWCFG4.power Optimize=%d\n",
			    dev_sts.b.suspsts, core_if->hwcfg4.b.power_optimiz);
		/* PCD callback for suspend. Release the lock inside of callback function */
		cil_pcd_suspend(core_if);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		if (core_if->power_down == 2) {
			dev_cfg.d32 = REG_RD(&core_if->dev_if->dev_global_regs->dcfg);
			DBG_USB_Print(DBG_USB,"lx_state = %08x\n",core_if->lx_state);
			DBG_USB_Print(DBG_USB," device address = %08d\n",dev_cfg.b.devaddr);
			if (((core_if->lx_state) != DWC_OTG_L3) && (dev_cfg.b.devaddr)) {
				pcgcctl_data_t pwr_clk_gcctl;
				gpwrdn_data_t gbl_pwr_dn; 
				gusbcfg_data_t gbl_usb_cfg;
				pwr_clk_gcctl.d32 = 0;
				gbl_pwr_dn.d32 = 0 ;
				gbl_usb_cfg.d32 = 0;
				/* Change to L2(suspend) state */
				core_if->lx_state = DWC_OTG_L2;
				/* Clear interrupt in gintsts */
				gbl_int_sts.d32 = 0;
				gbl_int_sts.b.usbsuspend = 1;
				REG_WR(&core_if->core_global_regs->
						gintsts, gbl_int_sts.d32);
				 DBG_USB_Print(DBG_USB,"Start of hibernation completed\n");
				dwc_otg_save_global_regs(core_if);
				dwc_otg_save_dev_regs(core_if);
				gbl_usb_cfg.d32 =
				    REG_RD(&core_if->core_global_regs->
						   gusbcfg);
				if (gbl_usb_cfg.b.ulpi_utmi_sel == 1) {
					/* ULPI interface */
					/* Suspend the Phy Clock */
					pwr_clk_gcctl.d32 = 0;
					pwr_clk_gcctl.b.stoppclk = 1;
					DWC_MODIFY_REG32(core_if->pcgcctl, 0,
							 pwr_clk_gcctl.d32);
					dwc_udelay(10);
					gbl_pwr_dn.b.pmuactv = 1;
					DWC_MODIFY_REG32(&core_if->
							 core_global_regs->
							 gpwrdn, 0, gbl_pwr_dn.d32);
				} else {
					/* UTMI+ Interface */
					gbl_pwr_dn.b.pmuactv = 1;
					DWC_MODIFY_REG32(&core_if->
							 core_global_regs->
							 gpwrdn, 0, gbl_pwr_dn.d32);
					dwc_udelay(10);
					pwr_clk_gcctl.b.stoppclk = 1;
					DWC_MODIFY_REG32(core_if->pcgcctl, 0,
							 pwr_clk_gcctl.d32);
					dwc_udelay(10);
				}
				/* Set flag to indicate that we are in hibernation */
				core_if->hibernation_suspend = 1;
				/* Enable interrupts from wake up logic */
				gbl_pwr_dn.d32 = 0;
				gbl_pwr_dn.b.pmuintsel = 1;
				DWC_MODIFY_REG32(&core_if->core_global_regs->
						 gpwrdn, 0, gbl_pwr_dn.d32);
				dwc_udelay(10);
				/* Unmask device mode interrupts in GPWRDN */
				gbl_pwr_dn.d32 = 0;
				gbl_pwr_dn.b.rst_det_msk = 1;
				gbl_pwr_dn.b.lnstchng_msk = 1;
				gbl_pwr_dn.b.sts_chngint_msk = 1;
				DWC_MODIFY_REG32(&core_if->core_global_regs->
						 gpwrdn, 0, gbl_pwr_dn.d32);
				dwc_udelay(10);
				/* Enable Power Down Clamp */
				gbl_pwr_dn.d32 = 0;
				gbl_pwr_dn.b.pwrdnclmp = 1;
				DWC_MODIFY_REG32(&core_if->core_global_regs->
						 gpwrdn, 0, gbl_pwr_dn.d32);
				dwc_udelay(10);
				/* Switch off VDD */
				gbl_pwr_dn.d32 = 0;
				gbl_pwr_dn.b.pwrdnswtch = 1;
				DWC_MODIFY_REG32(&core_if->core_global_regs->
						 gpwrdn, 0, gbl_pwr_dn.d32);
				/* Save gpwrdn register for further usage if stschng interrupt */
				core_if->gr_backup->gpwrdn_local =
							REG_RD(&core_if->core_global_regs->gpwrdn);
				 DBG_USB_Print(DBG_USB,"Hibernation completed\n");
				flag = 1;
			}
		} else if (core_if->power_down == 3) {
			pcgcctl_data_t pwr_clk_gcctl;
			pwr_clk_gcctl.d32 = 0 ;
			dev_cfg.d32 = REG_RD(&core_if->dev_if->dev_global_regs->dcfg);
			DBG_USB_Print(DBG_USB, "lx_state = %08x\n",core_if->lx_state);
			DBG_USB_Print(DBG_USB, " device address = %08d\n",dev_cfg.b.devaddr);
			if (((core_if->lx_state) != DWC_OTG_L3) && (dev_cfg.b.devaddr)) {
				DBG_USB_Print(DBG_USB, "Start entering to extended hibernation\n");
				core_if->xhib = 1;
				/* Clear interrupt in gintsts */
				gbl_int_sts.d32 = 0;
				gbl_int_sts.b.usbsuspend = 1;
				REG_WR(&core_if->core_global_regs->
					gintsts, gbl_int_sts.d32);
				dwc_otg_save_global_regs(core_if);
				dwc_otg_save_dev_regs(core_if);
				/* Wait for 10 PHY clocks */
				dwc_udelay(10);
				/* Program GPIO register while entering to xHib */
				REG_WR(&core_if->core_global_regs->ggpio, 0x1);
				pwr_clk_gcctl.b.enbl_extnd_hiber = 1;
				DWC_MODIFY_REG32(core_if->pcgcctl, 0, pwr_clk_gcctl.d32);
				DWC_MODIFY_REG32(core_if->pcgcctl, 0, pwr_clk_gcctl.d32);
				pwr_clk_gcctl.d32 = 0;
				pwr_clk_gcctl.b.extnd_hiber_pwrclmp = 1;
				DWC_MODIFY_REG32(core_if->pcgcctl, 0, pwr_clk_gcctl.d32);
				pwr_clk_gcctl.d32 = 0;
				pwr_clk_gcctl.b.extnd_hiber_switch = 1;
				core_if->gr_backup->xhib_gpwrdn = REG_RD(&core_if->core_global_regs->gpwrdn);
				core_if->gr_backup->xhib_pcgcctl = REG_RD(core_if->pcgcctl) | pwr_clk_gcctl.d32;
				DWC_MODIFY_REG32(core_if->pcgcctl, 0, pwr_clk_gcctl.d32);
				DBG_USB_Print(DBG_USB, "Finished entering to extended hibernation\n");
				flag = 1;
			}
		} else{
            if ((core_if->otg_ver == 1) && (core_if->core_params->otg_cap == DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE)) {
				gotgctl_data_t gbl_otg_ctl;
				DBG_USB_Print(DBG_USB, "DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE\n");
				gbl_otg_ctl.d32 = 0 ;
				gbl_otg_ctl.d32 = REG_RD(&core_if->core_global_regs->gotgctl);
				if ((gbl_otg_ctl.b.devhnpen) && (core_if->otg_ver == 1)){
					gotgctl_data_t gbl_otgctl;
					gbl_otgctl.d32 = 0 ;
					dwc_mdelay(5);
					/**@todo Is the gotgctl.devhnpen cleared
					 * by a USB Reset? */
					gbl_otgctl.b.devhnpen = 1;
					gbl_otgctl.b.hnpreq = 1;
					REG_WR(&core_if->core_global_regs->gotgctl,
							gbl_otgctl.d32);
				}
			}
		}
#endif
		if (flag == 0){
			/* Change to L2(suspend) state */
			core_if->lx_state = DWC_OTG_L2;

			/* Clear interrupt */
			gbl_int_sts.d32 = 0;
			gbl_int_sts.b.usbsuspend = 1;
			REG_WR(&core_if->core_global_regs->gintsts, gbl_int_sts.d32);
		}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	}
#endif
    (void)dev_sts;
	return 1;
}

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)

#ifdef CONFIG_USB_DWC_OTG_LPM
/**
 * This function hadles LPM transaction received interrupt.
 */
static uint32_t dwc_otg_handle_lpm_intr(dwc_otg_core_if_t * core_if)
{
	glpmcfg_data_t lpmcfg;
	gintsts_data_t gintsts;

	if (!core_if->core_params->lpm_enable) {
		 DBG_USB_Print(DBG_USB,"Unexpected LPM interrupt\n");
	}

	lpmcfg.d32 = REG_RD(&core_if->core_global_regs->glpmcfg);
	 DBG_USB_Print(DBG_USB,"LPM config register = 0x%08x\n", lpmcfg.d32);

		pcgcctl_data_t pcgcctl = {.d32 = 0 };

		lpmcfg.b.hird_thres |= (1 << 4);
		lpmcfg.b.en_utmi_sleep = 1;

		pcgcctl.b.enbl_sleep_gating = 1;
	   	DWC_MODIFY_REG32(core_if->pcgcctl,0,pcgcctl.d32);

		if(dwc_otg_get_param_besl_enable(core_if)) {
			lpmcfg.b.en_besl = 1;				
		}

		REG_WR(&core_if->core_global_regs->glpmcfg,
				lpmcfg.d32);		

	/* Examine prt_sleep_sts after TL1TokenTetry period max (10 us) */
	dwc_udelay(10);
	lpmcfg.d32 = REG_RD(&core_if->core_global_regs->glpmcfg);
	if (lpmcfg.b.prt_sleep_sts) {
		/* Save the current state */
		core_if->lx_state = DWC_OTG_L1;
	}

	/* Clear interrupt  */
	gintsts.d32 = 0;
	gintsts.b.lpmtranrcvd = 1;
	REG_WR(&core_if->core_global_regs->gintsts, gintsts.d32);
	return 1;
}
#endif /* CONFIG_USB_DWC_OTG_LPM */

#endif
/**
 * This function returns the Core Interrupt register.
 */
static inline uint32_t dwc_otg_read_common_intr(dwc_otg_core_if_t const * core_if)
{
	uint32_t retval = 0;
	gahbcfg_data_t gbl_ahb_cfg; 
	gintsts_data_t gbl_int_sts;
	gintmsk_data_t gbl_int_msk;
	gintmsk_data_t gintmsk_common ;
	gbl_ahb_cfg.d32 = 0 ;
	gintmsk_common.d32 = 0 ;
	gintmsk_common.b.wkupintr = 1;
	gintmsk_common.b.sessreqintr = 1;
	gintmsk_common.b.conidstschng = 1;
	gintmsk_common.b.otgintr = 1;
	gintmsk_common.b.modemismatch = 1;
	gintmsk_common.b.disconnect = 1;
	gintmsk_common.b.usbsuspend = 1;
#ifdef CONFIG_USB_DWC_OTG_LPM
	gintmsk_common.b.lpmtranrcvd = 1;
#endif
	gintmsk_common.b.restoredone = 1;
	/** @todo: The port interrupt occurs while in device
         * mode. Added code to CIL to clear the interrupt for now!
         */
	gintmsk_common.b.portintr = 1;

	gbl_int_sts.d32 = REG_RD(&core_if->core_global_regs->gintsts);
	gbl_int_msk.d32 = REG_RD(&core_if->core_global_regs->gintmsk);
	gbl_ahb_cfg.d32 = REG_RD(&core_if->core_global_regs->gahbcfg);


	/* if any common interrupts set */
	if ((gbl_int_sts.d32 & gintmsk_common.d32) != 0) {
		DBG_USB_Print(DBG_USB,"gintsts=%08x  gintmsk=%08x\n",
			    gbl_int_sts.d32, gbl_int_msk.d32);
	}

	if (gbl_ahb_cfg.b.glblintrmsk != 0) {
		retval = ((gbl_int_sts.d32 & gbl_int_msk.d32) & gintmsk_common.d32);
	}
	return retval;
}

/**
 * Common interrupt handler.
 *
 * The common interrupts are those that occur in both Host and Device mode.
 * This handler handles the following interrupts:
 * - Mode Mismatch Interrupt
 * - Disconnect Interrupt
 * - OTG Interrupt
 * - Connector ID Status Change Interrupt
 * - Session Request Interrupt.
 * - Resume / Remote Wakeup Detected Interrupt.
 * - LPM Transaction Received Interrupt
 * - ADP Transaction Received Interrupt
 *
 */
int32_t dwc_otg_handle_common_intr(void *dev)
{
	uint32_t retval = 0;
	gintsts_data_t gbl_int_sts;
	dwc_otg_device_t *otg_dev_ptr = dev;
	dwc_otg_core_if_t *core_if_ptr = otg_dev_ptr->core_if;
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
	gpwrdn_data_t gbl_pwr_dn; 
	gbl_pwr_dn.d32 = 0 ;
	
	gbl_pwr_dn.d32 = REG_RD(&core_if_ptr->core_global_regs->gpwrdn);
#endif
    
	if (dwc_otg_check_haps_status(core_if_ptr) == -1 ) {
		DBG_Warn_Print("HAPS is disconnected");			
	}
	else {
		core_if_ptr->frame_num = dwc_otg_get_frame_number(core_if_ptr);
	
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
		if ((core_if_ptr->power_down == 3) && (core_if_ptr->xhib == 1)) {
			DBG_USB_Print(DBG_USB, "Exiting from xHIB state\n");
			retval |= (uint32_t)dwc_otg_handle_xhib_exit_intr(core_if_ptr);
			core_if_ptr->xhib = 2;
		}
		else {
#endif
			if (core_if_ptr->hibernation_suspend <= 0) {
				gbl_int_sts.d32 = dwc_otg_read_common_intr(core_if_ptr);

#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
				if (gbl_int_sts.b.modemismatch != 0) {
					retval |= (uint32_t)dwc_otg_handle_mode_mismatch_intr(core_if_ptr);
				}
				if (gbl_int_sts.b.otgintr != 0) {
					retval |= (uint32_t)dwc_otg_handle_otg_intr(core_if_ptr);
				}
				if (gbl_int_sts.b.conidstschng != 0) {
					retval |=
						(uint32_t)dwc_otg_handle_conn_id_status_change_intr(core_if_ptr);
				}
				if (gbl_int_sts.b.disconnect != 0) {
					retval |= (uint32_t)dwc_otg_handle_disconnect_intr(core_if_ptr);
				}
				if (gbl_int_sts.b.sessreqintr != 0) {
					retval |= (uint32_t)dwc_otg_handle_session_req_intr(core_if_ptr);
				}
				if (gbl_int_sts.b.wkupintr != 0) {
					retval |=(uint32_t) dwc_otg_handle_wakeup_detected_intr(core_if_ptr);
				}
#endif
				if (gbl_int_sts.b.usbsuspend != 0) {
					retval |= (uint32_t)dwc_otg_handle_usb_suspend_intr(core_if_ptr);
				}
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
#ifdef CONFIG_USB_DWC_OTG_LPM
				if (gintsts.b.lpmtranrcvd != 0) {
					retval |= (uint32_t)dwc_otg_handle_lpm_intr(core_if);
				}
#endif
				if (gbl_int_sts.b.restoredone != 0) {
					gbl_int_sts.d32 = 0;
					if (core_if_ptr->power_down == 2) {
						core_if_ptr->hibernation_suspend = -1;
					}
					else if ((core_if_ptr->power_down == 3) && (core_if_ptr->xhib == 2)) {
						/* gpwrdn_data_t gbl_pwr_dn; */
						pcgcctl_data_t pcgc_ctl;
						dctl_data_t dev_ctl ;
						gbl_pwr_dn.d32 = 0 ;
						pcgc_ctl.d32 = 0 ;
						dev_ctl.d32 = 0 ;

						REG_WR(&core_if_ptr->core_global_regs->
								gintsts, 0xFFFFFFFFU);

						DBG_USB_Print(DBG_USB,
								"RESTORE DONE generated\n");

						gbl_pwr_dn.b.restore = 1;
						DWC_MODIFY_REG32(&core_if_ptr->core_global_regs->gpwrdn, gbl_pwr_dn.d32, 0);
						dwc_udelay(10);

						pcgc_ctl.b.rstpdwnmodule = 1;
						DWC_MODIFY_REG32(core_if_ptr->pcgcctl, pcgc_ctl.d32, 0);

						REG_WR(&core_if_ptr->core_global_regs->gusbcfg, core_if_ptr->gr_backup->gusbcfg_local);
						REG_WR(&core_if_ptr->dev_if->dev_global_regs->dcfg, core_if_ptr->dr_backup->dcfg);
						REG_WR(&core_if_ptr->dev_if->dev_global_regs->dctl, core_if_ptr->dr_backup->dctl);
						dwc_udelay(50);
						
						dev_ctl.b.pwronprgdone = 1;
						DWC_MODIFY_REG32(&core_if_ptr->dev_if->dev_global_regs->dctl, 0, dev_ctl.d32);
						dwc_udelay(10);

						dwc_otg_restore_global_regs(core_if_ptr);
						dwc_otg_restore_dev_regs(core_if_ptr, 0);

						dev_ctl.d32 = 0;
						dev_ctl.b.pwronprgdone = 1;
						DWC_MODIFY_REG32(&core_if_ptr->dev_if->dev_global_regs->dctl, dev_ctl.d32, 0);
						dwc_udelay(10);

						pcgc_ctl.d32 = 0;
						pcgc_ctl.b.enbl_extnd_hiber = 1;
						DWC_MODIFY_REG32(core_if_ptr->pcgcctl, pcgc_ctl.d32, 0);

						/* The core will be in ON STATE */
						core_if_ptr->lx_state = DWC_OTG_L0;
						core_if_ptr->xhib = 0;

						if ((core_if_ptr->pcd_cb) && (core_if_ptr->pcd_cb->resume_wakeup)) {
							core_if_ptr->pcd_cb->resume_wakeup(core_if_ptr->pcd_cb->p);
						}
					} 
					gbl_int_sts.b.restoredone = 1;
					REG_WR(&core_if_ptr->core_global_regs->gintsts,gbl_int_sts.d32);
					retval |= 1;
				}
				if (gbl_int_sts.b.portintr != 0) {
					/* The port interrupt occurs while in device mode with HPRT0
					 * Port Enable/Disable.
					 */
					gbl_int_sts.d32 = 0;
					gbl_int_sts.b.portintr = 1;
					REG_WR(&core_if_ptr->core_global_regs->gintsts,gbl_int_sts.d32);
					retval |= 1;
				}
#endif
			} 
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
            else {
				DBG_USB_Print(DBG_USB, "gpwrdn=%08x\n", gbl_pwr_dn.d32);

				if (((gbl_pwr_dn.b).disconn_det) && ((gbl_pwr_dn.b).disconn_det_msk)) {
					CLEAR_GPWRDN_INTR(core_if_ptr, disconn_det);
					if (gbl_pwr_dn.b.linestate == 0) {
						dwc_otg_handle_pwrdn_disconnect_intr(core_if_ptr);
					} else {
						 DBG_USB_Print(DBG_USB,"Disconnect detected while linestate is not 0\n");
					}

					retval |= 1;
				}
				if (((gbl_pwr_dn.b).lnstschng) && ((gbl_pwr_dn.b).lnstchng_msk)) {
					CLEAR_GPWRDN_INTR(core_if_ptr, lnstschng);
					/* remote wakeup from hibernation */
					if ((gbl_pwr_dn.b.linestate == 2) || (gbl_pwr_dn.b.linestate == 1)) {
						dwc_otg_handle_pwrdn_wakeup_detected_intr(core_if_ptr);
					} else {
						 DBG_USB_Print(DBG_USB,"gpwrdn.linestate = %d\n", gbl_pwr_dn.b.linestate);
					}
					retval |= 1;
				}
				if (((gbl_pwr_dn.b).rst_det) && ((gbl_pwr_dn.b).rst_det_msk)) {
					CLEAR_GPWRDN_INTR(core_if_ptr, rst_det);
					if (gbl_pwr_dn.b.linestate == 0) {
						 DBG_USB_Print(DBG_USB,"Reset detected\n");
						retval |= (uint32_t)dwc_otg_device_hibernation_restore(core_if_ptr, 0, 1);
					}
				}
				if (((gbl_pwr_dn.b).srp_det) && ((gbl_pwr_dn.b).srp_det_msk)) {
					CLEAR_GPWRDN_INTR(core_if_ptr, srp_det);
					dwc_otg_handle_pwrdn_srp_intr(core_if_ptr);

					retval |= 1;
				}
			}
			if (((gbl_pwr_dn.b).sts_chngint) && ((gbl_pwr_dn.b).sts_chngint_msk)) {
				CLEAR_GPWRDN_INTR(core_if_ptr, sts_chngint);
				dwc_otg_handle_pwrdn_stschng_intr(otg_dev_ptr);

				retval |= 1;
			}
			if (((gbl_pwr_dn.b).srp_det) && ((gbl_pwr_dn.b).srp_det_msk)) {
				CLEAR_GPWRDN_INTR(core_if_ptr, srp_det);
				dwc_otg_handle_pwrdn_srp_intr(core_if_ptr);
				retval |= 1;
			}
		}
#endif
	}
	return (int32_t)retval;
}
