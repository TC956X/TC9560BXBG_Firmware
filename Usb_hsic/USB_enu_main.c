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
 *  18 July 2016 :  
 */
 
#include <stdlib.h>
#include <neu_os.h>
#include <dwc_os.h>
#include <tc9560_common.h>
#include "tc9560_defs.h"
#include "dwc_otg_regs.h"
#include "dwc_otg_driver.h"
#include "tc9560_uart.h"
#include "tc9560_reg_define.h"
#include "net_dev_neu.h"
#include "DWC_ETH_QOS_yregacc.h"
#include "dwc_otg_dbg.h"
#include "tc9560_gpio.h"
#define TC9560_HSIC_FW_VER ("01.02.00")

volatile uint32_t  malloc_free_counter[4];
volatile uint8_t NetDev_RX_callback[4];

int32_t main(void)
 {
	static int32_t 	 rc = 0;
	struct lm_device dev_lm;
	neu_hs_bulk_out1_desc_defn();
	neu_hs_bulk_out2_desc_defn();
	neu_hs_bulk_out3_desc_defn();
	neu_hs_bulk_out4_desc_defn();
	neu_hs_bulk_in5_desc_defn();
	neu_hs_bulk_in6_desc_defn();
	neu_hs_bulk_in7_desc_defn();
	neu_hs_bulk_in8_desc_defn();
	neu_driver_call ();
	neu_intf_desc_call();

	/*TC9560 Clock and reset control*/
	REG_WR((volatile uint32_t *)NCLKCTRL,0x00006199U);
	REG_WR((volatile uint32_t *)NRSTCTRL,0x00003FFCU);	/*reset all peripherals*/
	REG_WR((volatile uint32_t *)NRSTCTRL,0x00000000U);
	

	/* UnMask all Global Interrupts and Select DMA Mode for Data Transfer*/
	REG_WR((volatile uint32_t *)GAHBCFG,0x000001A3U); 

	REG_WR(0xE000E404U,0x20000000U);
	REG_WR(0xE000E408U,0x20000000U);
	REG_WR(0xE000E40CU,0x20202020U);
	REG_WR(0xE000E410U,0x20202020U);
	REG_WR(0xE000E414U,0x00002020U);

	#if 1 /* for debug purpose */
	//1 ms
	REG_WR(0xE000E014,187000000/1000 - 1);
	REG_WR(0xE000ED20,0x00E00000);
	*(unsigned int *)(0xE000E010) |= (0x00000004 | 0x00000001);
	*(unsigned int *)(0xE000E010) |= (0x00000002);
	
	{
		int i;
		for (i = 0; i < 0x100; i++)
			*(unsigned int *)(TC9560_M3_DBG_CNT_START + 4*i) = 0;
	}
	#endif

	memset((void *)malloc_free_counter,0,sizeof(volatile uint32_t )*4);
	memset((void *)NetDev_RX_callback,0,sizeof(volatile uint8_t)*4);

	uart_initialize(115200);

	
	DBG_Info_Print("Welcome to TC9560 HSIC Firmware world!\n");
	DBG_Info_Print("Version: %s\n", TC9560_HSIC_FW_VER);
	
	
	REG_WR((volatile uint32_t *)NEMACCTL,0x303000A);
	REG_WR((volatile uint32_t *)NHPLLUPDT,0x3000080);

	/* Enable Interrupts in INTC Block */
	REG_WR((volatile uint32_t *)INTMCUMASK0,0xFFFFFF7FU);
	REG_WR((volatile uint32_t *)INTMCUMASK1,0xFFC007FFU);
	REG_WR((volatile uint32_t *)INTEXTMASK0,0xFFFFFF7FU);
	REG_WR((volatile uint32_t *)INTEXTMASK1,0xFFC007FFU);
	REG_WR((volatile uint32_t *)INTINTXMASK0,0xFFFFFF7FU);
	REG_WR((volatile uint32_t *)INTINTXMASK1,0xFFC007FFU);

	/* Initialize USB Endpoint structure and Enable HSIC endpoint related Interrupt */ 	
	rc = dwc_otg_driver_probe(&dev_lm);
	if(rc != Y_SUCCESS)
    {
		DBG_Error_Print("DWC OTG probe function fails to Initialize Peripheral Controller Driver\n");
    }

	/* Initialize USB Enumeration related Sturcures.*/
	rc = neu_init();
	if(rc != Y_SUCCESS)
	{
		DBG_Error_Print("TC9560 Enumeration Specific Initialization Fails\n");
	}
	else
	{
		DBG_Test_Print(DBG_TEST, "TC9560 Enumeration Specific Initialization Passed \n");
	}

	/* HSIC Device Control */
	REG_WR(DCTL,0x502);	

	REG_WR(GLPMCFG,0x40000000U);		/*Core LPM Config*/
	REG_WR(DCFG,0x8900800U);				/*Device Configuration*/
	REG_WR(DIEPMSK,0xffffffffU);
	REG_WR(DOEPMSK,0xffffffffU);
	REG_WR(DAINTMSK,0xffffffffU);

	REG_WR(DPTXFSIZ1,0x01000226U);
	REG_WR(DPTXFSIZ2,0x01000326U);
	REG_WR(DPTXFSIZ3,0x01000426U);
	REG_WR(DPTXFSIZ4,0x01000526U);
	REG_WR(DPTXFSIZ5,0x01000626U);

	REG_WR(USB_CNTR_STS_REG,0x1d0000U);

	/* Enable Cortex M3 NVIC Interrupt for HSIC */
	REG_WR(NVIC_ISER0,(REG_RD(NVIC_ISER0) | (1<<7)));

	REG_WR(DCTL,0x500U); 

	/* Initialize eMAC and enable NVIC Interrupt*/
	tc9560_init_network();

    /* Configuration of GPIO0 and GPIO1 Output mode*/
	tc9560_gpio0_config_output(GPIO0 | GPIO1);
    
    while(1)
	{
		#if DBG_EMAC_RX
			int32_t i;
			for(i=0;i<4;i++)
			{
				if(malloc_free_counter[i] != 0)
					DBG_eMAC_Print(DBG_EMAC_RX,"Malloc count value ch %d: %d\n", i, malloc_free_counter[i]);
			}
		#endif
		if((malloc_free_counter[0] < IN_REQ_NUMBER - 3)  && (NetDev_RX_callback[0]))
		{
			tc9560INTC_enable_rxtx_interrupt(INTC_Mask_RXCH0);
			tc9560INTC_enable_rxtx_interrupt(INTC_Mask_RXCH1);
			tc9560INTC_enable_rxtx_interrupt(INTC_Mask_RXCH2);
		}
		else if((malloc_free_counter[1] < IN_REQ_NUMBER - 3)  && (NetDev_RX_callback[1]))
		{
			tc9560INTC_enable_rxtx_interrupt(INTC_Mask_RXCH5);
		}
		else if((malloc_free_counter[2] < IN_REQ_NUMBER - 3)  && (NetDev_RX_callback[2]))
		{
			tc9560INTC_enable_rxtx_interrupt(INTC_Mask_RXCH4);
		}
		else if((malloc_free_counter[3] < IN_REQ_NUMBER - 3)  && (NetDev_RX_callback[3]))
		{
			tc9560INTC_enable_rxtx_interrupt(INTC_Mask_RXCH3);
		}
	}
}
