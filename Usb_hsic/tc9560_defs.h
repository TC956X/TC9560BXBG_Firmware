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

#ifndef __TC9560_DEFS_H__
#define __TC9560_DEFS_H__
#include "tc9560_common.h"

#if 1 /* for debug purpose */
#define TC9560_M3_DBG_CNT_START         0x7f000  // Debugging count SRAM area start address
	/* TC9560_M3_DBG_CNT_START + 4*0:   DMA TX0
	 *   .........................
	 * TC9560_M3_DBG_CNT_START + 4*4:   DMA TX4
	 * TC9560_M3_DBG_CNT_START + 4*5:   MAC LPI/PWR/EVENT
	 * TC9560_M3_DBG_CNT_START + 4*6:   DMA RX0
	 *   .........................
	 * TC9560_M3_DBG_CNT_START + 4*11:  DMA RX5
	 * TC9560_M3_DBG_CNT_START + 4*12:  Reserved
	 * TC9560_M3_DBG_CNT_START + 4*13:  Reserved
	 * TC9560_M3_DBG_CNT_START + 4*14:  Reserved
	 * TC9560_M3_DBG_CNT_START + 4*15:  MS count
	 */
#endif


/**
 * \brief
 *  Type defines
 */
extern void (* global_isr_table[])(void);
extern void OS_CPU_IntHandler (void);
extern void OS_CPU_SysTickHandler (void);
extern void OS_CPU_PendSVHandler(void);
extern void generic_isr(void);
extern void NMI_isr(void);
extern void HardFault_isr(void);
extern void MemManage_isr(void);
extern void BusFault_isr(void);
extern void UsageFault_isr(void);
extern void SVC_isr(void);
extern void DebugMon_isr(void);
extern void PendSV_isr(void);
extern void SysTick_isr(void);
#endif
/************************************** EOF *********************************/
