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

#ifndef __DWC_OTG_DBG_H__
#define __DWC_OTG_DBG_H__

#include "tc9560_uart.h"

/** @file
 * This file defines debug levels.
 * Debugging support vanishes in non-debug builds.  
 */

/** When DBG_CIL is define as 1, display CIL Debug messages. */
#define DBG_CIL			0

/** When DBG_CILV is define as 1, display CIL Verbose debug messages */
#define DBG_CILV		0

/**  When DBG_PCD is define as 1, display PCD (Device) debug messages */
#define DBG_PCD			0

/** When DBG_PCDV is define as 1, display PCD (Device) Verbose debug messages */
#define DBG_PCDV		0

/** When DBG_HCD is define as 1, display Host debug messages */
#define DBG_HCD			0

/** When DBG_USB is define as 1, display USB debug messages */
#define DBG_USB			0

/** When DBG_USB is define as 1, display USB debug messages */
#define USB_REG_RD_WR	0
 
/** When DBG_EMAC is define as 1, display eMAC debug messages */
#define DBG_EMAC		0

/** When DBG_INFO is define as 1, display info messages */
#define DBG_INFO		1

/** When DBG_EMAC_RX is define as 1, display eMAC RX debug messages */
#define DBG_EMAC_RX		0

/** When DBG_EMAC_TX is define as 1, display eMAC TX debug messages */
#define DBG_EMAC_TX		0

/** When DBG_TEST is define as 1, display test debug messages during unit testing */
#define DBG_TEST		0

/** When DBG_TEST is define as 1, display test debug messages during unit testing */
#define DBG_TEST_IN_EP	0

/** When ERR_INFO is define as 1, display error messages */
#define ERR_INFO		0

/** When WRN_INFO is define as 1, display warning messages */
#define WRN_INFO		0

#if(DBG_EMAC || DBG_EMAC_RX || DBG_EMAC_TX)
#define DBG_eMAC_Print(lvl, x...) \
	{\
		if(lvl) {\
				TC9560_Ser_Printf(x);\
		}\
	}
#else
#define DBG_eMAC_Print(lvl, x...) 
#endif	

#if DBG_INFO
#define DBG_Info_Print(x...)\
 			{\
				if(DBG_INFO) \
				{\
					TC9560_Ser_Printf(x);\
				}\
			}
#else
#define DBG_Info_Print(x...) 
#endif
            
#if ERR_INFO            
#define DBG_Error_Print(x...) 		{ TC9560_Ser_Printf("ERROR: "); TC9560_Ser_Printf(x); }
#else
#define DBG_Error_Print(x...)
#endif

#if WRN_INFO
#define DBG_Warn_Print(x...) 		{ TC9560_Ser_Printf("WARNING: "); TC9560_Ser_Printf(x); }
#else
#define DBG_Warn_Print(x...)
#endif

#if(DBG_USB || DBG_CIL || DBG_CILV || DBG_PCD || DBG_PCDV || DBG_HCD || USB_REG_RD_WR)
#define DBG_USB_Print(lvl, x...)\
	{\
		if(lvl)\
		{\
			TC9560_Ser_Printf(x);\
		}\
	}
#else
#define DBG_USB_Print(lvl, x...)
#endif    

#if(DBG_TEST || DBG_TEST_IN_EP)
#define DBG_Test_Print(lvl, x...)	{ if(lvl) TC9560_Ser_Printf(x); }
#else
#define DBG_Test_Print(lvl, x...)
#endif

#endif
