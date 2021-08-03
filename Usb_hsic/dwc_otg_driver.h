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
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_driver.h $
 * $Revision: 1.5 $
 * $Date: 2016/07/18 23:50:47 $
 * $Change: 1627671 $
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

#ifndef __DWC_OTG_DRIVER_H__
#define __DWC_OTG_DRIVER_H__

#include "neu_os.h"

#ifndef LM_INTERFACE
#define LM_INTERFACE
#endif

/* Driver Error Codes */ 
#define DEVERR				701
#define ALLOCERR			702
#define INVALOPERR			703
#define RESBSYERR			704

static void *dev_get_drvdata(const struct device * dev)
{
	return dev->driver_data;
}

static void dev_set_drvdata(struct device *dev, void * data)
{
	dev->driver_data = data;
}

/* Refered from ARM lm.h file */
#define lm_get_drvdata(lm)		dev_get_drvdata(& (lm)->dev)
#define lm_set_drvdata(lm,d)	dev_set_drvdata(& (lm)->dev, d)

	/** @file
 * This file contains the interface to the Linux driver.
 */
#include "dwc_otg_os_dep.h"
#include "dwc_otg_core_if.h"

/* Type declarations */
struct dwc_otg_pcd;
struct dwc_otg_hcd;

/**
 * This structure is a wrapper that encapsulates the driver components used to
 * manage a single DWC_otg controller.
 */
typedef struct dwc_otg_device {
	/** Structure containing OS-dependent stuff. KEEP THIS STRUCT AT THE
	 * VERY BEGINNING OF THE DEVICE STRUCT. OSes such as FreeBSD and NetBSD
	 * require this. */
	struct os_dependent os_dep;

	/** Pointer to the core interface structure. */
	dwc_otg_core_if_t * core_if;

	/** Pointer to the PCD structure. */
	struct dwc_otg_pcd * pcd;

	/** Pointer to the HCD structure. */
	struct dwc_otg_hcd * hcd;

	/** Flag to indicate whether the common IRQ handler is installed. */
	uint8_t common_irq_installed;

} dwc_otg_device_t;

extern int32_t dwc_otg_driver_probe( struct lm_device * _dev ); 
extern void (* global_isr_table[])(void);

extern int32_t pcd_init(
#ifdef LM_INTERFACE
	struct lm_device const * _dev
#endif
	);
 
/*We must clear S3C24XX_EINTPEND external interrupt register 
 * because after clearing in this register trigerred IRQ from 
 * H/W core in kernel interrupt can be occured again before OTG
 * handlers clear all IRQ sources of Core registers because of
 * timing latencies and Low Level IRQ Type.
 */
#ifdef CONFIG_MACH_IPMATE
#define	 S3C2410X_CLEAR_EINTPEND()	\
do { \
	__raw_writel(1UL << 11, S3C24XX_EINTPEND); \
} while (0)
#else
#define	 S3C2410X_CLEAR_EINTPEND()	 do { } while (0)
#endif

#endif
