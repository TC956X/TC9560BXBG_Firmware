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
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_driver.c $
 * $Revision: 1.10 $
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

/** @file
 * The dwc_otg_driver module provides the initialization and cleanup entry
 * points for the DWC_otg driver. This module will be dynamically installed
 * after Linux is booted using the insmod command. When the module is
 * installed, the dwc_otg_driver_init function is called. When the module is
 * removed (using rmmod), the dwc_otg_driver_cleanup function is called.
 *
 * This module also defines a data structure for the dwc_otg_driver, which is
 * used in conjunction with the standard ARM lm_device structure. These
 * structures allow the OTG driver to comply with the standard Linux driver
 * model in which devices and drivers are registered with a bus driver. This
 * has the benefit that Linux can expose attributes of the driver and device
 * in its special sysfs file system. Users can then read or write files in
 * this file system to perform diagnostics on the driver components or the
 * device.
 */

#include "dwc_otg_os_dep.h"
#include "dwc_os.h" 
#include "neu_os.h"
#include "dwc_otg_dbg.h"
#include "dwc_otg_driver.h"
#include "dwc_otg_core_if.h"
#include "dwc_otg_pcd_if.h"
#include "tc9560_reg_define.h"
#include "tc9560_uart.h"

static void dwc_otg_driver_remove(
#ifdef LM_INTERFACE
					 struct lm_device *_dev
#endif
    );

cb_handler_t cb_array[2] = {0};
dwc_otg_device_t *gbl_dwc_otg_device = NULL;
/*-------------------------------------------------------------------------*/
/* Encapsulate the module parameter settings */

struct dwc_otg_driver_module_params {
	int32_t opt;
	int32_t otg_cap;
	int32_t dma_enable;
	int32_t dma_desc_enable;
	int32_t dma_burst_size;
	int32_t speed;
	int32_t host_support_fs_ls_low_power;
	int32_t host_ls_low_power_phy_clk;
	int32_t enable_dynamic_fifo;
	int32_t data_fifo_size;
	int32_t dev_rx_fifo_size;
	int32_t dev_nperio_tx_fifo_size;
	int32_t dev_perio_tx_fifo_size[MAX_PERIO_FIFOS];
	int32_t host_rx_fifo_size;
	int32_t host_nperio_tx_fifo_size;
	int32_t host_perio_tx_fifo_size;
	int32_t max_transfer_size;
	int32_t max_packet_count;
	int32_t host_channels;
	int32_t dev_endpoints;
	int32_t phy_type;
	int32_t phy_utmi_width;
	int32_t phy_ulpi_ddr;
	int32_t phy_ulpi_ext_vbus;
	int32_t i2c_enable;
	int32_t ulpi_fs_ls;
	int32_t ts_dline;
	int32_t en_multiple_tx_fifo;
	int32_t dev_tx_fifo_size[MAX_TX_FIFOS];
	int32_t thr_ctl;
	int32_t tx_thr_length;
	int32_t rx_thr_length;
	int32_t pti_enable;
	int32_t mpi_enable;
	int32_t lpm_enable;
	int32_t besl_enable;
	int32_t baseline_besl;
	int32_t deep_besl;
	int32_t ic_usb_cap;
	int32_t ahb_thr_ratio;
	int32_t power_down;
	int32_t reload_ctl;
	int32_t dev_out_nak;
	int32_t cont_on_bna;
	int32_t ahb_single;
	int32_t otg_ver;
	int32_t adp_enable;
};

#ifdef DRIVERCODE
/**
 * This function shows the Driver Version.
 */

static ssize_t version_show(struct device_driver *dev, char_t *buf)
{
	return snprintf(buf, sizeof(DWC_DRIVER_VERSION) + 2, "%s\n",
			DWC_DRIVER_VERSION);
}

static DRIVER_ATTR(version, S_IRUGO, version_show, NULL);

/**
 * Global Debug Level Mask.
 */
uint32_t g_dbg_lvl = 0;		/* OFF */

/**
 * This function shows the driver Debug Level.
 */
static ssize_t dbg_level_show(struct device_driver *drv, char_t *buf)
{
	return sprintf(buf, "0x%0x\n", g_dbg_lvl);
}

/**
 * This function stores the driver Debug Level.
 */
static ssize_t dbg_level_store(struct device_driver *drv, const char_t *buf,
			       size_t count)
{
	g_dbg_lvl = strtoul(buf, NULL, 16);
	return count;
}

static DRIVER_ATTR(debuglevel, S_IRUGO | S_IWUSR, dbg_level_show,
		   dbg_level_store);

#endif //TODO: CLEANUP

/**
 * This function is the top level interrupt handler for the Common
 * (Device and host modes) interrupts.
 */
irqreturn_t dwc_otg_common_irq(void *dev)
{
	uint32_t retval = IRQ_NONE;

	retval = ((uint32_t)dwc_otg_handle_common_intr(dev));
	if (retval != 0) {
		S3C2410X_CLEAR_EINTPEND();
	}
	return IRQ_RETVAL(retval);
}

void USB_ISR(void)
{
    /* Read the GINTSTS register */
    uint32_t  gintmsk = REG_RD((volatile uint32_t *)GINTMSK);	
    uint32_t  gintsts = REG_RD((volatile uint32_t *)GINTSTS);	
	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
    /* Call the appropriate handler */
    if((gintsts & (0x004EF6F8U & gintmsk)) != 0) {
				cb_array[1].handler(cb_array[1].dev);
	}
    else if((gintsts & 0xF9010806U & gintmsk) != 0) {
				 cb_array[0].handler(cb_array[0].dev);	
	}
}

/**
 * This function is called when a lm_device is unregistered with the
 * dwc_otg_driver. This happens, for example, when the rmmod command is
 * executed. The device may or may not be electrically present. If it is
 * present, the driver stops device processing. Any resources used on behalf
 * of this device are freed.
 *
 * @param _dev
 */
static void dwc_otg_driver_remove(
#ifdef LM_INTERFACE
					 struct lm_device *_dev
#endif
    )
{
#ifdef LM_INTERFACE
	dwc_otg_device_t *otg_dev = lm_get_drvdata(_dev);
#endif

	DBG_USB_Print(DBG_USB, "%s(%p)\n", __func__, _dev);

	DBG_Test_Print(DBG_TEST, "%s \n", __func__);
 	if (!otg_dev) {
		/* Memory allocation for the dwc_otg_device failed. */
		DBG_USB_Print(DBG_USB, "%s: otg_dev NULL!\n", __func__);
		/*return;*/
	}else {
			if (otg_dev->pcd == 0) 
			{
				/*
				* Free the IRQ
				*/
				if (otg_dev->common_irq_installed != 0) {
					/* free_irq(_dev->irq, otg_dev);  // TODO: to be implemented */
					if (otg_dev->core_if != 0) {
						dwc_otg_cil_remove(otg_dev->core_if);
						/*
						* Clear the drvdata pointer.
						*/
	#ifdef LM_INTERFACE
						lm_set_drvdata(_dev, 0);
	#endif
					} else {
						DBG_USB_Print(DBG_USB, "%s: otg_dev->core_if NULL!\n", __func__);
						/*return;*/
					}
				} else {
					DBG_USB_Print(DBG_USB, "%s: There is no installed irq!\n", __func__);
					/*return;*/
				}
			} else {
				DBG_USB_Print(DBG_USB, "%s: otg_dev->pcd NULL!\n", __func__);
				/*return;*/
			}
		}
	return;
}

/**
 * This function is called when an lm_device is bound to a
 * dwc_otg_driver. It creates the driver components required to
 * control the device (CIL, HCD, and PCD) and it initializes the
 * device. The driver components are stored in a dwc_otg_device
 * structure. A reference to the dwc_otg_device is saved in the
 * lm_device. This allows the driver to access the dwc_otg_device
 * structure on subsequent calls to driver methods for this device.
 *
 * @param _dev Bus device
 */
int32_t dwc_otg_driver_probe(struct lm_device *_dev)
{
	int32_t retval = 0;
	dwc_otg_device_t *ptr_dwc_otg_device = NULL;
	
	ptr_dwc_otg_device = DWC_ALLOC(sizeof(dwc_otg_device_t));
	if (!ptr_dwc_otg_device) {
		DBG_Error_Print("kmalloc of dwc_otg_device failed!\n");
		retval = -ALLOCERR;
	}
	else {

		memset(ptr_dwc_otg_device, 0, sizeof(*gbl_dwc_otg_device));
		
		ptr_dwc_otg_device->os_dep.reg_offset = 0xFFFFFFFFU;
		/*
		 * Map the DWC_otg Core memory
		 */
		ptr_dwc_otg_device->os_dep.base = (void *)HSIC_BASE;
		/*
		 * Initialize driver data to point to the global DWC_otg
		 * Device structure.
		 */
	#ifdef LM_INTERFACE
		 lm_set_drvdata(_dev, ptr_dwc_otg_device);
	#endif
	 /* call cil init at time of core initialization*/
		ptr_dwc_otg_device->core_if = dwc_otg_cil_init(ptr_dwc_otg_device->os_dep.base);
		if (!ptr_dwc_otg_device->core_if) {
				DBG_Error_Print("CIL initialization failed!\n");
			  retval = -ALLOCERR;
			  dwc_otg_driver_remove(_dev);
		} else {
            /*
             * Disable the global interrupt until all the interrupt
             * handlers are installed.
             */
             /* Call disable interrupt when init core*/
            dwc_otg_disable_global_interrupts(ptr_dwc_otg_device->core_if);

          /*
             * Install the interrupt handler for the common interrupts before
             * enabling common interrupts in core_init below.
             */
            DBG_USB_Print(DBG_CIL, "registering (common) handler for irq%d\n",
                    _dev->irq);

                    global_isr_table[7] = USB_ISR;
                    
                /* Handler registration */
                cb_array[0].handler = (cb_function) dwc_otg_common_irq;
                cb_array[0].dev = ptr_dwc_otg_device;
                
            if (retval != 0) {
                DBG_Error_Print("request of irq%d failed\n", _dev->irq);
                retval = -RESBSYERR;
                dwc_otg_driver_remove(_dev);
            } else {
                ptr_dwc_otg_device->common_irq_installed = 1;
                /*
                 * Initialize the DWC_otg core.
                 */
                dwc_otg_core_init(ptr_dwc_otg_device->core_if);
                    
                if (dwc_otg_is_device_mode(ptr_dwc_otg_device->core_if) != 0) {
                    DBG_USB_Print(DBG_USB,"TC9560 HSIC is in Device Mode.\r\n");
                    /*
                    * Initialize the PCD
                    */
                    retval = pcd_init(_dev);
                    if (retval != 0) {
                        DBG_Error_Print("pcd_init failed\n");
                        ptr_dwc_otg_device->pcd = NULL;
                        dwc_otg_driver_remove(_dev);
                    }else {
                        /*
                        * Enable the global interrupt after all the interrupt
                        * handlers are installed if there is no ADP support else 
                        * perform initial actions required for Internal ADP logic.
                        */
                        dwc_otg_enable_global_interrupts(ptr_dwc_otg_device->core_if);
                    }
                }else{
                    DBG_USB_Print(DBG_USB,"TC9560 HSIC is in Host Mode. Exiting..\r\n");
                    retval = -RESBSYERR;
                    dwc_otg_driver_remove(_dev);
                }
                
            }
		}
	}
	return retval;
}
