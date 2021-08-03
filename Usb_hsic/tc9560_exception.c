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

#include "tc9560_defs.h"
#include "dwc_otg_dbg.h"
#include <tc9560_common.h>

uint32_t  isr_no;
void  OS_CPU_IntHandler (void) 
{
	DBG_Error_Print("OS_CPU_IntHandler-------------------->\n");
	while(1) {
		;
	}
}

void  OS_CPU_SysTickHandler (void)
{
	DBG_Error_Print("OS_CPU_SysTickHandler-------------------->\n");
	while(1){
		;
	}		
}

void OS_CPU_PendSVHandler(void)
{
	DBG_Error_Print("OS_CPU_SysTickHandler-------------------->\n");
	while(1){
		;
	}

}	
/*****************************************************************************/
/*								  General ISR							  */
/*****************************************************************************/
/**
 * \brief
 *	This is default ISR function which is going to be executed in case if any
 *  ISR is not registered.
 *
 * \param  none
 * \return none
 */																						
void generic_isr(void)
{
	DBG_Warn_Print("ISR %d not registered-------------------->\n",isr_no);
}

/*****************************************************************************/
/*								  Cortex ISR							   */
/*****************************************************************************/
/**
 * \brief
 *	NMI interrupt service routine
 *
 * \param  none
 * \return none
 */
void NMI_isr(void)
{
	DBG_Error_Print("NMI isr generated-------------------->\n");
	while(1) {
		;
	}
}

/**
 * \brief
 *	HardFault interrupt service routine
 *
 * \param  none
 * \return none
 */
void HardFault_isr(void)
{
	DBG_Error_Print("HardFault isr generated---------->\n");
	while(1) {
		;
	}
}

/**
 * \brief
 *	MemManage interrupt service routine
 *
 * \param  none
 * \return none
 */
void MemManage_isr(void)
{
	DBG_Error_Print("MemManage isr generated-------------------->\n");
	while(1) {
		;
	}
}

/**
 * \brief
 *	BusFault interrupt service routine
 *
 * \param  none
 * \return none
 */
void BusFault_isr(void)
{
	DBG_Error_Print("BusFault isr generated-------------------->\n");
	while(1) {
		;
	}
}

/**
 * \brief
 *	UsageFault interrupt service routine
 *
 * \param  none
 * \return none
 */
void UsageFault_isr(void)
{
	DBG_Error_Print("UsageFault isr generated-------------------->\n");
	while(1){
		;
	}
}

/**
 * \brief
 *	SVC interrupt service routine
 *
 * \param  none
 * \return none
 */
void SVC_isr(void)
{
	DBG_Error_Print("SVC isr generated-------------------->\n");
	while(1){
		;
	}
}

/**
 * \brief
 *	interrupt service routine
 *
 * \param  none
 * \return none
 */
void DebugMon_isr(void)
{
	DBG_Error_Print("DebugMon isr generated-------------------->\n");
	while(1){
		;
	}
}

/**
 * \brief
 *	PendSV interrupt service routine
 *
 * \param  none
 * \return none
 */
void PendSV_isr(void)
{
	DBG_Error_Print("PendSV isr generated-------------------->\n");
	while(1){
		;
	}
}

/**
 * \brief
 *	SysTick interrupt service routine
 *
 * \param  none
 * \return none
 */

volatile u32 msTicks; /* timeTicks counter*/ 
void SysTick_isr(void)
{
	msTicks++;  /* increment timeTicks counter*/
	*(unsigned int *)(TC9560_M3_DBG_CNT_START + 4*15) += 1;
}

/*****************************************************************************/
/*								SoC ISR Table							  */
/*****************************************************************************/
/**
 * \brief
 *	Second level ISR vector table, It is According to Spec 0.95
 *
 *  ISR Table
 */

void (* global_isr_table[])(void) =
{
	generic_isr, /*0:  External interrupt input (GPIO09)*/
	generic_isr, /*1:  External interrupt input (GPIO10)*/
	generic_isr, /*2:  External interrupt input (GPIO11)*/
	generic_isr, /*3:  External interrupt input (INT_i)*/
	generic_isr, /*4:  I2C slave interrupt*/
	generic_isr, /*5:  I2C master interrupt*/
	generic_isr, /*6:  SPI slave interrupt*/
	generic_isr, /*7:  HSIC general interrupt*/
	generic_isr, /*8:  HSIC endpoint interrupt*/
	generic_isr, /*9:  MAC LPI exit interrupt*/
	generic_isr, /*10: MAC Power management interrupt*/
	generic_isr, /*11: MAC interrupt to indicate event from LPI, RGMII, Management counters, power management or MAC core.*/
	generic_isr, /*12: MAC interrupt from eMAC Tx DMA channels.*/
	generic_isr, /*13: MAC interrupt from eMAC Rx DMA channels.*/
	generic_isr, /*14: TDM interrupt*/
	generic_isr, /*15: qSPI interrupt*/
	generic_isr, /*16: GDMA CH Interrupt*/
	generic_isr, /*17: GDMA general interrupt*/
	generic_isr, /*18: SHA interrupt*/
	generic_isr, /*19: UART interrupt*/
	generic_isr, /*20: CAN interrupt*/
	generic_isr, /*21: CAN interrupt*/
	generic_isr, /*22: PCIe controller interrupt*/
	generic_isr, /*23: MCU FLAG interrupt (This is set when intr_mcu_flag is non-zero)*/
	generic_isr, /*24: EXT FLAG interrupt (This is set when intr_ext_flag is non-zero)*/
};

/************************************** EOF *********************************/
