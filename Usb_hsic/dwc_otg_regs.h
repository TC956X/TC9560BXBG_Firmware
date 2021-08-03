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
 *      18 July 2016 : Revision: 1.0 
 */

/* ==========================================================================
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_regs.h $
 * $Revision: 1.5 $
 * $Date: 2016/07/18 23:50:47 $
 * $Change: 2123206 $
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

#ifndef __DWC_OTG_REGS_H__
#define __DWC_OTG_REGS_H__

#include "dwc_otg_core_if.h"
#include "tc9560_common.h"
/**
 * @file
 *
 * This file contains the data structures for accessing the DWC_otg core registers.
 *
 * The application interfaces with the HS OTG core by reading from and
 * writing to the Control and Status Register (CSR) space through the
 * AHB Slave interface. These registers are 32 bits wide, and the
 * addresses are 32-bit-block aligned.
 * CSRs are classified as follows:
 * - Core Global Registers
 * - Device Mode Registers
 * - Device Global Registers
 * - Device Endpoint Specific Registers
 * - Host Mode Registers
 * - Host Global Registers
 * - Host Port CSRs
 * - Host Channel Specific Registers
 *
 * Only the Core Global registers can be accessed in both Device and
 * Host modes. When the HS OTG core is operating in one mode, either
 * Device or Host, the application must not access registers from the
 * other mode. When the core switches from one mode to another, the
 * registers in the new mode of operation must be reprogrammed as they
 * would be after a power-on reset.
 */

/****************************************************************************/
/** DWC_otg Core registers . 
 * The dwc_otg_core_global_regs structure defines the size
 * and relative field offsets for the Core Global registers.
 */
typedef struct dwc_otg_core_global_regs {
	/** OTG Control and Status Register.  <i>Offset: 000h</i> */
	volatile uint32_t gotgctl;
	/** OTG Interrupt Register.	 <i>Offset: 004h</i> */
	volatile uint32_t gotgint;
	/**Core AHB Configuration Register.	 <i>Offset: 008h</i> */
	volatile uint32_t gahbcfg;

#define DWC_GLBINTRMASK				0x0001
#define DWC_DMAENABLE				0x0020
#define DWC_NPTXEMPTYLVL_EMPTY		0x0080
#define DWC_NPTXEMPTYLVL_HALFEMPTY	0x0000
#define DWC_PTXEMPTYLVL_EMPTY		0x0100
#define DWC_PTXEMPTYLVL_HALFEMPTY	0x0000

	/**Core USB Configuration Register.	 <i>Offset: 00Ch</i> */
	volatile uint32_t gusbcfg;
	/**Core Reset Register.	 <i>Offset: 010h</i> */
	volatile uint32_t grstctl;
	/**Core Interrupt Register.	 <i>Offset: 014h</i> */
	volatile uint32_t gintsts;
	/**Core Interrupt Mask Register.  <i>Offset: 018h</i> */
	volatile uint32_t gintmsk;
	/**Receive Status Queue Read Register (Read Only).	<i>Offset: 01Ch</i> */
	volatile uint32_t grxstsr;
	/**Receive Status Queue Read & POP Register (Read Only).  <i>Offset: 020h</i>*/
	volatile uint32_t grxstsp;
	/**Receive FIFO Size Register.	<i>Offset: 024h</i> */
	volatile uint32_t grxfsiz;
	/**Non Periodic Transmit FIFO Size Register.  <i>Offset: 028h</i> */
	volatile uint32_t gnptxfsiz;
	/**Non Periodic Transmit FIFO/Queue Status Register (Read
	 * Only). <i>Offset: 02Ch</i> */
	volatile uint32_t gnptxsts;
	/**I2C Access Register.	 <i>Offset: 030h</i> */
	volatile uint32_t gi2cctl;
	/**PHY Vendor Control Register.	 <i>Offset: 034h</i> */
	volatile uint32_t gpvndctl;
	/**General Purpose Input/Output Register.  <i>Offset: 038h</i> */
	volatile uint32_t ggpio;
	/**User ID Register.  <i>Offset: 03Ch</i> */
	volatile uint32_t guid;
	/**Synopsys ID Register (Read Only).  <i>Offset: 040h</i> */
	volatile uint32_t gsnpsid;
	/**User HW Config1 Register (Read Only).  <i>Offset: 044h</i> */
	volatile uint32_t ghwcfg1;
	/**User HW Config2 Register (Read Only).  <i>Offset: 048h</i> */
	volatile uint32_t ghwcfg2;
#define DWC_SLAVE_ONLY_ARCH 0
#define DWC_EXT_DMA_ARCH 1
#define DWC_INT_DMA_ARCH 2
#define DWC_MODE_HNP_SRP_CAPABLE	0
#define DWC_MODE_SRP_ONLY_CAPABLE	1
#define DWC_MODE_NO_HNP_SRP_CAPABLE		2
#define DWC_MODE_SRP_CAPABLE_DEVICE		3
#define DWC_MODE_NO_SRP_CAPABLE_DEVICE	4
#define DWC_MODE_SRP_CAPABLE_HOST	5
#define DWC_MODE_NO_SRP_CAPABLE_HOST	6

	/**User HW Config3 Register (Read Only).  <i>Offset: 04Ch</i> */
	volatile uint32_t ghwcfg3;
	/**User HW Config4 Register (Read Only).  <i>Offset: 050h</i>*/
	volatile uint32_t ghwcfg4;
	/** Core LPM Configuration register <i>Offset: 054h</i>*/
	volatile uint32_t glpmcfg;
	/** Global PowerDn Register <i>Offset: 058h</i> */
	volatile uint32_t gpwrdn;
	/** Global DFIFO SW Config Register  <i>Offset: 05Ch</i> */
	volatile uint32_t gdfifocfg;
	/** ADP Control Register  <i>Offset: 060h</i> */
	volatile uint32_t adpctl;
	/** Reserved  <i>Offset: 064h-0FFh</i> */
	volatile uint32_t reserved39[39];
	/** Host Periodic Transmit FIFO Size Register. <i>Offset: 100h</i> */
	volatile uint32_t hptxfsiz;
	/** Device Periodic Transmit FIFO#n Register if dedicated fifos are disabled,
		otherwise Device Transmit FIFO#n Register.
	 * <i>Offset: 104h + (FIFO_Number-1)*04h, 1 <= FIFO Number <= 15 (1<=n<=15).</i> */
	volatile uint32_t dtxfsiz[15];
} dwc_otg_core_global_regs_t;

/**
 * This union represents the bit fields of the Core OTG Control
 * and Status Register (GOTGCTL).  Set the bits using the bit
 * fields then write the <i>d32</i> value to the register.
 */
typedef union gotgctl_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN sesreqscs:1;
		USIGN sesreq:1;
		USIGN vbvalidoven:1;
		USIGN vbvalidovval:1;
		USIGN avalidoven:1;
		USIGN avalidovval:1;
		USIGN bvalidoven:1;
		USIGN bvalidovval:1;
		USIGN hstnegscs:1;
		USIGN hnpreq:1;
		USIGN hstsethnpen:1;
		USIGN devhnpen:1;
		USIGN reserved12_15:4;
		USIGN conidsts:1;
		USIGN dbnctime:1;
		USIGN asesvld:1;
		USIGN bsesvld:1;
		USIGN otgver:1;
		USIGN reserved1:1;
		USIGN multvalidbc:5;
		USIGN chirpen:1;
		USIGN reserved28_31:4;
	} b;
} gotgctl_data_t;

/**
 * This union represents the bit fields of the Core OTG Interrupt Register
 * (GOTGINT).  Set/clear the bits using the bit fields then write the <i>d32</i>
 * value to the register.
 */
typedef union gotgint_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/** Current Mode */
		USIGN reserved0_1:2;

		/** Session End Detected */
		USIGN sesenddet:1;

		USIGN reserved3_7:5;

		/** Session Request Success Status Change */
		USIGN sesreqsucstschng:1;
		/** Host Negotiation Success Status Change */
		USIGN hstnegsucstschng:1;

		USIGN reserved10_16:7;

		/** Host Negotiation Detected */
		USIGN hstnegdet:1;
		/** A-Device Timeout Change */
		USIGN adevtoutchng:1;
		/** Debounce Done */
		USIGN debdone:1;
		/** Multi-Valued input changed */
		USIGN mvic:1;

		USIGN reserved31_21:11;

	} b;
} gotgint_data_t;

/**
 * This union represents the bit fields of the Core AHB Configuration
 * Register (GAHBCFG). Set/clear the bits using the bit fields then
 * write the <i>d32</i> value to the register.
 */
typedef union gahbcfg_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN glblintrmsk:1;
#define DWC_GAHBCFG_GLBINT_ENABLE			1
		USIGN hburstlen:4;
#define DWC_GAHBCFG_INT_DMA_BURST_SINGLE	0
#define DWC_GAHBCFG_INT_DMA_BURST_INCR		1
#define DWC_GAHBCFG_INT_DMA_BURST_INCR4		3
#define DWC_GAHBCFG_INT_DMA_BURST_INCR8		5
#define DWC_GAHBCFG_INT_DMA_BURST_INCR16	7
		USIGN dmaenable:1;
#define DWC_GAHBCFG_DMAENABLE				1
		USIGN reserved:1;
		USIGN nptxfemplvl_txfemplvl:1;
		USIGN ptxfemplvl:1;
#define DWC_GAHBCFG_TXFEMPTYLVL_EMPTY		1
#define DWC_GAHBCFG_TXFEMPTYLVL_HALFEMPTY	0
		USIGN reserved9_20:12;
		USIGN remmemsupp:1;
		USIGN notialldmawrit:1;
		USIGN ahbsingle:1;
		USIGN reserved24_31:8;
	} b;
} gahbcfg_data_t;

/**
 * This union represents the bit fields of the Core USB Configuration
 * Register (GUSBCFG). Set the bits using the bit fields then write
 * the <i>d32</i> value to the register.
 */
typedef union gusbcfg_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN toutcal:3;
		USIGN phyif:1;
		USIGN ulpi_utmi_sel:1;
		USIGN fsintf:1;
		USIGN physel:1;
		USIGN ddrsel:1;
		USIGN srpcap:1;
		USIGN hnpcap:1;
		USIGN usbtrdtim:4;
		USIGN reserved1:1;
		USIGN phylpwrclksel:1;
		USIGN otgutmifssel:1;
		USIGN ulpi_fsls:1;
		USIGN ulpi_auto_res:1;
		USIGN ulpi_clk_sus_m:1;
		USIGN ulpi_ext_vbus_drv:1;
		USIGN ulpi_int_vbus_indicator:1;
		USIGN term_sel_dl_pulse:1;
		USIGN indicator_complement:1;
		USIGN indicator_pass_through:1;
		USIGN ulpi_int_prot_dis:1;
		USIGN ic_usb_cap:1;
		USIGN ic_traffic_pull_remove:1;
		USIGN tx_end_delay:1;
		USIGN force_host_mode:1;
		USIGN force_dev_mode:1;
		USIGN reserved31:1;
	} b;
} gusbcfg_data_t;

/**
 * This union represents the bit fields of the Core Reset Register
 * (GRSTCTL).  Set/clear the bits using the bit fields then write the
 * <i>d32</i> value to the register.
 */
typedef union grstctl_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/** Core Soft Reset (CSftRst) (Device and Host)
		 *
		 * The application can flush the control logic in the
		 * entire core using this bit. This bit resets the
		 * pipelines in the AHB Clock domain as well as the
		 * PHY Clock domain.
		 *
		 * The state machines are reset to an IDLE state, the
		 * control bits in the CSRs are cleared, all the
		 * transmit FIFOs and the receive FIFO are flushed.
		 *
		 * The status mask bits that control the generation of
		 * the interrupt, are cleared, to clear the
		 * interrupt. The interrupt status bits are not
		 * cleared, so the application can get the status of
		 * any events that occurred in the core after it has
		 * set this bit.
		 *
		 * Any transactions on the AHB are terminated as soon
		 * as possible following the protocol. Any
		 * transactions on the USB are terminated immediately.
		 *
		 * The configuration settings in the CSRs are
		 * unchanged, so the software doesn't have to
		 * reprogram these registers (Device
		 * Configuration/Host Configuration/Core System
		 * Configuration/Core PHY Configuration).
		 *
		 * The application can write to this bit, any time it
		 * wants to reset the core. This is a self clearing
		 * bit and the core clears this bit after all the
		 * necessary logic is reset in the core, which may
		 * take several clocks, depending on the current state
		 * of the core.
		 */
		USIGN csftrst:1;
		/** Hclk Soft Reset
		 *
		 * The application uses this bit to reset the control logic in
		 * the AHB clock domain. Only AHB clock domain pipelines are
		 * reset.
		 */
		USIGN hsftrst:1;
		/** Host Frame Counter Reset (Host Only)<br>
		 *
		 * The application can reset the (micro)frame number
		 * counter inside the core, using this bit. When the
		 * (micro)frame counter is reset, the subsequent SOF
		 * sent out by the core, will have a (micro)frame
		 * number of 0.
		 */
		USIGN hstfrm:1;
		/** In Token Sequence Learning Queue Flush
		 * (INTknQFlsh) (Device Only)
		 */
		USIGN intknqflsh:1;
		/** RxFIFO Flush (RxFFlsh) (Device and Host)
		 *
		 * The application can flush the entire Receive FIFO
		 * using this bit. The application must first
		 * ensure that the core is not in the middle of a
		 * transaction. The application should write into
		 * this bit, only after making sure that neither the
		 * DMA engine is reading from the RxFIFO nor the MAC
		 * is writing the data in to the FIFO. The
		 * application should wait until the bit is cleared
		 * before performing any other operations. This bit
		 * will takes 8 clocks (slowest of PHY or AHB clock)
		 * to clear.
		 */
		USIGN rxfflsh:1;
		/** TxFIFO Flush (TxFFlsh) (Device and Host). 
		 *
		 * This bit is used to selectively flush a single or
		 * all transmit FIFOs. The application must first
		 * ensure that the core is not in the middle of a
		 * transaction. The application should write into
		 * this bit, only after making sure that neither the
		 * DMA engine is writing into the TxFIFO nor the MAC
		 * is reading the data out of the FIFO. The
		 * application should wait until the core clears this
		 * bit, before performing any operations. This bit
		 * will takes 8 clocks (slowest of PHY or AHB clock)
		 * to clear.
		 */
		USIGN txfflsh:1;

		/** TxFIFO Number (TxFNum) (Device and Host).
		 *
		 * This is the FIFO number which needs to be flushed,
		 * using the TxFIFO Flush bit. This field should not
		 * be changed until the TxFIFO Flush bit is cleared by
		 * the core.
		 *	 - 0x0 : Non Periodic TxFIFO Flush
		 *	 - 0x1 : Periodic TxFIFO #1 Flush in device mode
		 *	   or Periodic TxFIFO in host mode
		 *	 - 0x2 : Periodic TxFIFO #2 Flush in device mode.
		 *	 - ...
		 *	 - 0xF : Periodic TxFIFO #15 Flush in device mode
		 *	 - 0x10: Flush all the Transmit NonPeriodic and
		 *	   Transmit Periodic FIFOs in the core
		 */
		USIGN txfnum:5;
		/** Reserved */
		USIGN reserved11_29:19;
		/** DMA Request Signal.	 Indicated DMA request is in
		 * probress. Used for debug purpose. */
		USIGN dmareq:1;
		/** AHB Master Idle.  Indicates the AHB Master State
		 * Machine is in IDLE condition. */
		USIGN ahbidle:1;
	} b;
} grstctl_t;

/**
 * This union represents the bit fields of the Core Interrupt Mask
 * Register (GINTMSK). Set/clear the bits using the bit fields then
 * write the <i>d32</i> value to the register.
 */
typedef union gintmsk_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN reserved0:1;
		USIGN modemismatch:1;
		USIGN otgintr:1;
		USIGN sofintr:1;
		USIGN rxstsqlvl:1;
		USIGN nptxfempty:1;
		USIGN ginnakeff:1;
		USIGN goutnakeff:1;
		USIGN ulpickint:1;
		USIGN i2cintr:1;
		USIGN erlysuspend:1;
		USIGN usbsuspend:1;
		USIGN usbreset:1;
		USIGN enumdone:1;
		USIGN isooutdrop:1;
		USIGN eopframe:1;
		USIGN restoredone:1;
		USIGN epmismatch:1;
		USIGN inepintr:1;
		USIGN outepintr:1;
		USIGN incomplisoin:1;
		USIGN incomplisoout:1;
		USIGN fetsusp:1;
		USIGN resetdet:1;
		USIGN portintr:1;
		USIGN hcintr:1;
		USIGN ptxfempty:1;
		USIGN lpmtranrcvd:1;
		USIGN conidstschng:1;
		USIGN disconnect:1;
		USIGN sessreqintr:1;
		USIGN wkupintr:1;
	} b;
} gintmsk_data_t;
/**
 * This union represents the bit fields of the Core Interrupt Register
 * (GINTSTS).  Set/clear the bits using the bit fields then write the
 * <i>d32</i> value to the register.
 */
typedef union gintsts_data {
	/** raw register data */
	uint32_t d32;
#define DWC_SOF_INTR_MASK 0x0008
	/** register bits */
	struct {
#define DWC_HOST_MODE 1
		USIGN curmode:1;
		USIGN modemismatch:1;
		USIGN otgintr:1;
		USIGN sofintr:1;
		USIGN rxstsqlvl:1;
		USIGN nptxfempty:1;
		USIGN ginnakeff:1;
		USIGN goutnakeff:1;
		USIGN ulpickint:1;
		USIGN i2cintr:1;
		USIGN erlysuspend:1;
		USIGN usbsuspend:1;
		USIGN usbreset:1;
		USIGN enumdone:1;
		USIGN isooutdrop:1;
		USIGN eopframe:1;
		USIGN restoredone:1;
		USIGN epmismatch:1;
		USIGN inepint:1;
		USIGN outepintr:1;
		USIGN incomplisoin:1;
		USIGN incomplisoout:1;
		USIGN fetsusp:1;
		USIGN resetdet:1;
		USIGN portintr:1;
		USIGN hcintr:1;
		USIGN ptxfempty:1;
		USIGN lpmtranrcvd:1;
		USIGN conidstschng:1;
		USIGN disconnect:1;
		USIGN sessreqintr:1;
		USIGN wkupintr:1;
	} b;
} gintsts_data_t;

/**
 * This union represents the bit fields in the Device Receive Status Read and
 * Pop Registers (GRXSTSR, GRXSTSP) Read the register into the <i>d32</i>
 * element then read out the bits using the <i>b</i>it elements.
 */
typedef union device_grxsts_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN epnum:4;
		USIGN bcnt:11;
		USIGN dpid:2;

#define DWC_STS_DATA_UPDT					0x2	// OUT Data Packet
#define DWC_STS_XFER_COMP					0x3	// OUT Data Transfer Complete

#define DWC_DSTS_GOUT_NAK					0x1	// Global OUT NAK
#define DWC_DSTS_SETUP_COMP					0x4	// Setup Phase Complete
#define DWC_DSTS_SETUP_UPDT 				0x6	// SETUP Packet
		USIGN pktsts:4;
		USIGN fn:4;
		USIGN reserved25_31:7;
	} b;
} device_grxsts_data_t;

/**
 * This union represents the bit fields in the Host Receive Status Read and
 * Pop Registers (GRXSTSR, GRXSTSP) Read the register into the <i>d32</i>
 * element then read out the bits using the <i>b</i>it elements.
 */
typedef union host_grxsts_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN chnum:4;
		USIGN bcnt:11;
		USIGN dpid:2;

		USIGN pktsts:4;
#define DWC_GRXSTS_PKTSTS_IN				0x2
#define DWC_GRXSTS_PKTSTS_IN_XFER_COMP		0x3
#define DWC_GRXSTS_PKTSTS_DATA_TOGGLE_ERR	0x5
#define DWC_GRXSTS_PKTSTS_CH_HALTED			0x7

		USIGN reserved21_31:11;
	} b;
} host_grxsts_data_t;

/**
 * This union represents the bit fields in the FIFO Size Registers (HPTXFSIZ,
 * GNPTXFSIZ, DPTXFSIZn, DIEPTXFn). Read the register into the <i>d32</i> element 
 * then read out the bits using the <i>b</i>it elements.
 */
typedef union fifosize_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN startaddr:16;
		USIGN depth:16;
	} b;
} fifosize_data_t;

/**
 * This union represents the bit fields in the Non-Periodic Transmit
 * FIFO/Queue Status Register (GNPTXSTS). Read the register into the
 * <i>d32</i> element then read out the bits using the <i>b</i>it
 * elements.
 */
typedef union gnptxsts_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN nptxfspcavail:16;
		USIGN nptxqspcavail:8;
		/** Top of the Non-Periodic Transmit Request Queue
		 *	- bit 24 - Terminate (Last entry for the selected
		 *	  channel/EP)
		 *	- bits 26:25 - Token Type
		 *	  - 2'b00 - IN/OUT
		 *	  - 2'b01 - Zero Length OUT
		 *	  - 2'b10 - PING/Complete Split
		 *	  - 2'b11 - Channel Halt
		 *	- bits 30:27 - Channel/EP Number
		 */
		USIGN nptxqtop_terminate:1;
		USIGN nptxqtop_token:2;
		USIGN nptxqtop_chnep:4;
		USIGN reserved:1;
	} b;
} gnptxsts_data_t;

/**
 * This union represents the bit fields in the Transmit
 * FIFO Status Register (DTXFSTS). Read the register into the
 * <i>d32</i> element then read out the bits using the <i>b</i>it
 * elements.
 */
typedef union dtxfsts_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN txfspcavail:16;
		USIGN reserved:16;
	} b;
} dtxfsts_data_t;

/**
 * This union represents the bit fields in the I2C Control Register
 * (I2CCTL). Read the register into the <i>d32</i> element then read out the
 * bits using the <i>b</i>it elements.
 */
typedef union gi2cctl_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN rwdata:8;
		USIGN regaddr:8;
		USIGN addr:7;
		USIGN i2cen:1;
		USIGN ack:1;
		USIGN i2csuspctl:1;
		USIGN i2cdevaddr:2;
		USIGN i2cdatse0:1;
		USIGN reserved:1;
		USIGN rw:1;
		USIGN bsydne:1;
	} b;
} gi2cctl_data_t;

/**
 * This union represents the bit fields in the PHY Vendor Control Register
 * (GPVNDCTL). Read the register into the <i>d32</i> element then read out the
 * bits using the <i>b</i>it elements.
 */
typedef union gpvndctl_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN regdata:8;
		USIGN vctrl:8;
		USIGN regaddr16_21:6;
		USIGN regwr:1;
		USIGN reserved23_24:2;
		USIGN newregreq:1;
		USIGN vstsbsy:1;
		USIGN vstsdone:1;
		USIGN reserved28_30:3;
		USIGN disulpidrvr:1;
	} b;
} gpvndctl_data_t;

/**
 * This union represents the bit fields in the General Purpose 
 * Input/Output Register (GGPIO).
 * Read the register into the <i>d32</i> element then read out the
 * bits using the <i>b</i>it elements.
 */
typedef union ggpio_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN gpi:16;
		USIGN gpo:16;
	} b;
} ggpio_data_t;

/**
 * This union represents the bit fields in the User ID Register
 * (GUID). Read the register into the <i>d32</i> element then read out the
 * bits using the <i>b</i>it elements.
 */
typedef union guid_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN rwdata:32;
	} b;
} guid_data_t;

/**
 * This union represents the bit fields in the Synopsys ID Register
 * (GSNPSID). Read the register into the <i>d32</i> element then read out the
 * bits using the <i>b</i>it elements.
 */
typedef union gsnpsid_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN rwdata:32;
	} b;
} gsnpsid_data_t;

/**
 * This union represents the bit fields in the User HW Config1
 * Register.  Read the register into the <i>d32</i> element then read
 * out the bits using the <i>b</i>it elements.
 */
typedef union hwcfg1_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN ep_dir0:2;
		USIGN ep_dir1:2;
		USIGN ep_dir2:2;
		USIGN ep_dir3:2;
		USIGN ep_dir4:2;
		USIGN ep_dir5:2;
		USIGN ep_dir6:2;
		USIGN ep_dir7:2;
		USIGN ep_dir8:2;
		USIGN ep_dir9:2;
		USIGN ep_dir10:2;
		USIGN ep_dir11:2;
		USIGN ep_dir12:2;
		USIGN ep_dir13:2;
		USIGN ep_dir14:2;
		USIGN ep_dir15:2;
	} b;
} hwcfg1_data_t;

/**
 * This union represents the bit fields in the User HW Config2
 * Register.  Read the register into the <i>d32</i> element then read
 * out the bits using the <i>b</i>it elements.
 */
typedef union hwcfg2_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/* GHWCFG2 */
		USIGN op_mode:3;
#define DWC_HWCFG2_OP_MODE_HNP_SRP_CAPABLE_OTG		0
#define DWC_HWCFG2_OP_MODE_SRP_ONLY_CAPABLE_OTG		1
#define DWC_HWCFG2_OP_MODE_NO_HNP_SRP_CAPABLE_OTG	2
#define DWC_HWCFG2_OP_MODE_SRP_CAPABLE_DEVICE		3
#define DWC_HWCFG2_OP_MODE_NO_SRP_CAPABLE_DEVICE	4
#define DWC_HWCFG2_OP_MODE_SRP_CAPABLE_HOST			5
#define DWC_HWCFG2_OP_MODE_NO_SRP_CAPABLE_HOST		6

		USIGN architecture:2;
		USIGN point2point:1;
		USIGN hs_phy_type:2;
#define DWC_HWCFG2_HS_PHY_TYPE_NOT_SUPPORTED		0
#define DWC_HWCFG2_HS_PHY_TYPE_UTMI					1
#define DWC_HWCFG2_HS_PHY_TYPE_ULPI					2
#define DWC_HWCFG2_HS_PHY_TYPE_UTMI_ULPI			3

		USIGN fs_phy_type:2;
		USIGN num_dev_ep:4;
		USIGN num_host_chan:4;
		USIGN perio_ep_supported:1;
		USIGN dynamic_fifo:1;
		USIGN multi_proc_int:1;
		USIGN reserved21:1;
		USIGN nonperio_tx_q_depth:2;
		USIGN host_perio_tx_q_depth:2;
		USIGN dev_token_q_depth:5;
		USIGN otg_enable_ic_usb:1;
	} b;
} hwcfg2_data_t;

/**
 * This union represents the bit fields in the User HW Config3
 * Register.  Read the register into the <i>d32</i> element then read
 * out the bits using the <i>b</i>it elements.
 */
typedef union hwcfg3_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/* GHWCFG3 */
		USIGN xfer_size_cntr_width:4;
		USIGN packet_size_cntr_width:3;
		USIGN otg_func:1;
		USIGN i2c:1;
		USIGN vendor_ctrl_if:1;
		USIGN optional_features:1;
		USIGN synch_reset_type:1;
		USIGN adp_supp:1;
		USIGN otg_enable_hsic:1;
		USIGN bc_support:1;
		USIGN otg_lpm_en:1;
		SIGN dfifo_depth:16;
	} b;
} hwcfg3_data_t;

/**
 * This union represents the bit fields in the User HW Config4
 * Register.  Read the register into the <i>d32</i> element then read
 * out the bits using the <i>b</i>it elements.
 */
typedef union hwcfg4_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN num_dev_perio_in_ep:4;
		USIGN power_optimiz:1;
		USIGN min_ahb_freq:1;
		USIGN hiber:1;
		USIGN xhiber:1;
		USIGN reserved:6;
		USIGN utmi_phy_data_width:2;
		USIGN num_dev_mode_ctrl_ep:4;
		USIGN iddig_filt_en:1;
		USIGN vbus_valid_filt_en:1;
		USIGN a_valid_filt_en:1;
		USIGN b_valid_filt_en:1;
		USIGN session_end_filt_en:1;
		USIGN ded_fifo_en:1;
		USIGN num_in_eps:4;
		USIGN desc_dma:1;
		USIGN desc_dma_dyn:1;
	} b;
} hwcfg4_data_t;

/**
 * This union represents the bit fields of the Core LPM Configuration
 * Register (GLPMCFG). Set the bits using bit fields then write
 * the <i>d32</i> value to the register.
 */
typedef union glpmctl_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/** LPM-Capable (LPMCap) (Device and Host)
		 * The application uses this bit to control
		 * the DWC_otg core LPM capabilities.
		 */
		USIGN lpm_cap_en:1;
		/** LPM response programmed by application (AppL1Res) (Device)
		 * Handshake response to LPM token pre-programmed
		 * by device application software.
		 */
		USIGN appl_resp:1;
		/** Host Initiated Resume Duration (HIRD) (Device and Host)
		 * In Host mode this field indicates the value of HIRD
		 * to be sent in an LPM transaction.
		 * In Device mode this field is updated with the
		 * Received LPM Token HIRD bmAttribute
		 * when an ACK/NYET/STALL response is sent
		 * to an LPM transaction.
		 */
		USIGN hird:4;
		/** RemoteWakeEnable (bRemoteWake) (Device and Host)
		 * In Host mode this bit indicates the value of remote
		 * wake up to be sent in wIndex field of LPM transaction.
		 * In Device mode this field is updated with the
		 * Received LPM Token bRemoteWake bmAttribute
		 * when an ACK/NYET/STALL response is sent
		 * to an LPM transaction.
		 */
		USIGN rem_wkup_en:1;
		/** Enable utmi_sleep_n (EnblSlpM) (Device and Host)
		 * The application uses this bit to control
		 * the utmi_sleep_n assertion to the PHY when in L1 state.
		 */
		USIGN en_utmi_sleep:1;
		/** HIRD Threshold (HIRD_Thres) (Device and Host)
		 */
		USIGN hird_thres:5;
		/** LPM Response (CoreL1Res) (Device and Host)
		 * In Host mode this bit contains handsake response to
		 * LPM transaction.
		 * In Device mode the response of the core to
		 * LPM transaction received is reflected in these two bits.
		 	- 0x0 : ERROR (No handshake response)
			- 0x1 : STALL
			- 0x2 : NYET
			- 0x3 : ACK			
		 */
		USIGN lpm_resp:2;
		/** Port Sleep Status (SlpSts) (Device and Host)
		 * This bit is set as long as a Sleep condition
		 * is present on the USB bus.
		 */
		USIGN prt_sleep_sts:1;
		/** Sleep State Resume OK (L1ResumeOK) (Device and Host)
		 * Indicates that the application or host
		 * can start resume from Sleep state.
		 */
		USIGN sleep_state_resumeok:1;
		/** LPM channel Index (LPM_Chnl_Indx) (Host)
		 * The channel number on which the LPM transaction
		 * has to be applied while sending
		 * an LPM transaction to the local device.
		 */
		USIGN lpm_chan_index:4;
		/** LPM Retry Count (LPM_Retry_Cnt) (Host)
		 * Number host retries that would be performed
		 * if the device response was not valid response.
		 */
		USIGN retry_count:3;
		/** Send LPM Transaction (SndLPM) (Host)
		 * When set by application software,
		 * an LPM transaction containing two tokens
		 * is sent.
		 */
		USIGN send_lpm:1;
		/** LPM Retry status (LPM_RetryCnt_Sts) (Host)
		 * Number of LPM Host Retries still remaining
		 * to be transmitted for the current LPM sequence
		 */
		USIGN retry_count_sts:3;
		/** Enable Best Effort Service Latency (BESL) (Device and Host)
		 *  This bit enables the BESL features as defined in the LPM errata
		 */
		USIGN en_besl:1;
		USIGN reserved29:1;
		/** In host mode once this bit is set, the host
		 * configures to drive the HSIC Idle state on the bus.
		 * It then waits for the  device to initiate the Connect sequence.
		 * In device mode once this bit is set, the device waits for
		 * the HSIC Idle line state on the bus. Upon receving the Idle
		 * line state, it initiates the HSIC Connect sequence.
		 */
		USIGN hsic_connect:1;
		/** This bit overrides and functionally inverts
		 * the if_select_hsic input port signal.
		 */
		USIGN inv_sel_hsic:1;
	} b;
} glpmcfg_data_t;

/**
 * This union represents the bit fields of the Core ADP Timer, Control and
 * Status Register (ADPTIMCTLSTS). Set the bits using bit fields then write
 * the <i>d32</i> value to the register.
 */
typedef union adpctl_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/** Probe Discharge (PRB_DSCHG)
		 *  These bits set the times for TADP_DSCHG. 
		 *  These bits are defined as follows:
		 *  2'b00 - 4 msec
		 *  2'b01 - 8 msec
		 *  2'b10 - 16 msec
		 *  2'b11 - 32 msec
		 */
		USIGN prb_dschg:2;
		/** Probe Delta (PRB_DELTA)
		 *  These bits set the resolution for RTIM   value.
		 *  The bits are defined in units of 32 kHz clock cycles as follows:
		 *  2'b00  -  1 cycles
		 *  2'b01  -  2 cycles
		 *  2'b10 -  3 cycles
		 *  2'b11 - 4 cycles
		 *  For example if this value is chosen to 2'b01, it means that RTIM
		 *  increments for every 3(three) 32Khz clock cycles.
		 */
		USIGN prb_delta:2;
		/** Probe Period (PRB_PER)
		 *  These bits sets the TADP_PRD as shown in Figure 4 as follows:
		 *  2'b00  -  0.625 to 0.925 sec (typical 0.775 sec)
		 *  2'b01  -  1.25 to 1.85 sec (typical 1.55 sec)
		 *  2'b10  -  1.9 to 2.6 sec (typical 2.275 sec)
		 *  2'b11  -  Reserved
		 */
		USIGN prb_per:2;
		/** These bits capture the latest time it took for VBUS to ramp from 
		 *  VADP_SINK to VADP_PRB. 
		 *  0x000  -  1 cycles
		 *  0x001  -  2 cycles
		 *  0x002  -  3 cycles
		 *  etc
		 *  0x7FF  -  2048 cycles
		 *  A time of 1024 cycles at 32 kHz corresponds to a time of 32 msec.
		*/
		USIGN rtim:11;
		/** Enable Probe (EnaPrb)
		 *  When programmed to 1'b1, the core performs a probe operation.
		 *  This bit is valid only if OTG_Ver = 1'b1.
		 */
		USIGN enaprb:1;
		/** Enable Sense (EnaSns)
		 *  When programmed to 1'b1, the core performs a Sense operation.
		 *  This bit is valid only if OTG_Ver = 1'b1.
		 */
		USIGN enasns:1;
		/** ADP Reset (ADPRes)
		 *  When set, ADP controller is reset.
		 *  This bit is valid only if OTG_Ver = 1'b1.
 		 */
		USIGN adpres:1;
		/** ADP Enable (ADPEn)
		 *  When set, the core performs either ADP probing or sensing
		 *  based on EnaPrb or EnaSns.
		 *  This bit is valid only if OTG_Ver = 1'b1.
		 */
		USIGN adpen:1;
		/** ADP Probe Interrupt (ADP_PRB_INT)
		 *  When this bit is set, it means that the VBUS
		 *  voltage is greater than VADP_PRB or VADP_PRB is reached.
		 *  This bit is valid only if OTG_Ver = 1'b1.
		 */
		USIGN adp_prb_int:1;
		/**
		 *  ADP Sense Interrupt (ADP_SNS_INT)
		 *  When this bit is set, it means that the VBUS voltage is greater than 
		 *  VADP_SNS value or VADP_SNS is reached.
		 *  This bit is valid only if OTG_Ver = 1'b1.
		 */
		USIGN adp_sns_int:1;
		/** ADP Tomeout Interrupt (ADP_TMOUT_INT)
		 *  This bit is relevant only for an ADP probe.
		 *  When this bit is set, it means that the ramp time has
		 *  completed ie ADPCTL.RTIM has reached its terminal value
		 *  of 0x7FF.  This is a debug feature that allows software
		 *  to read the ramp time after each cycle.
		 *  This bit is valid only if OTG_Ver = 1'b1.
		 */
		USIGN adp_tmout_int:1;
		/** ADP Probe Interrupt Mask (ADP_PRB_INT_MSK)
		 *  When this bit is set, it unmasks the interrupt due to ADP_PRB_INT.
		 *  This bit is valid only if OTG_Ver = 1'b1.
		 */
		USIGN adp_prb_int_msk:1;
		/** ADP Sense Interrupt Mask (ADP_SNS_INT_MSK)
		 *  When this bit is set, it unmasks the interrupt due to ADP_SNS_INT.
		 *  This bit is valid only if OTG_Ver = 1'b1.
		 */
		USIGN adp_sns_int_msk:1;
		/** ADP Timoeout Interrupt Mask (ADP_TMOUT_MSK)
		 *  When this bit is set, it unmasks the interrupt due to ADP_TMOUT_INT.
		 *  This bit is valid only if OTG_Ver = 1'b1.
		 */
		USIGN adp_tmout_int_msk:1;
		/** Access Request
		 * 2'b00 - Read/Write Valid (updated by the core) 
		 * 2'b01 - Read
		 * 2'b00 - Write
		 * 2'b00 - Reserved
		 */
		USIGN ar:2;
		 /** Reserved */
		USIGN reserved29_31:3;
	} b;
} adpctl_data_t;

////////////////////////////////////////////
// Device Registers
/**
 * Device Global Registers. <i>Offsets 800h-BFFh</i>
 *
 * The following structures define the size and relative field offsets
 * for the Device Mode Registers.
 *
 * <i>These registers are visible only in Device mode and must not be
 * accessed in Host mode, as the results are unknown.</i>
 */
typedef struct dwc_otg_dev_global_regs {
	/** Device Configuration Register. <i>Offset 800h</i> */
	volatile uint32_t dcfg;
	/** Device Control Register. <i>Offset: 804h</i> */
	volatile uint32_t dctl;
	/** Device Status Register (Read Only). <i>Offset: 808h</i> */
	volatile uint32_t dsts;
	/** Reserved. <i>Offset: 80Ch</i> */
	uint32_t unused;
	/** Device IN Endpoint Common Interrupt Mask
	 * Register. <i>Offset: 810h</i> */
	volatile uint32_t diepmsk;
	/** Device OUT Endpoint Common Interrupt Mask
	 * Register. <i>Offset: 814h</i> */
	volatile uint32_t doepmsk;
	/** Device All Endpoints Interrupt Register.  <i>Offset: 818h</i> */
	volatile uint32_t daint;
	/** Device All Endpoints Interrupt Mask Register.  <i>Offset:
	 * 81Ch</i> */
	volatile uint32_t daintmsk;
	/** Device IN Token Queue Read Register-1 (Read Only).
	 * <i>Offset: 820h</i> */
	volatile uint32_t dtknqr1;
	/** Device IN Token Queue Read Register-2 (Read Only).
	 * <i>Offset: 824h</i> */
	volatile uint32_t dtknqr2;
	/** Device VBUS	 discharge Register.  <i>Offset: 828h</i> */
	volatile uint32_t dvbusdis;
	/** Device VBUS Pulse Register.	 <i>Offset: 82Ch</i> */
	volatile uint32_t dvbuspulse;
	/** Device IN Token Queue Read Register-3 (Read Only). /
	 *	Device Thresholding control register (Read/Write)
	 * <i>Offset: 830h</i> */
	volatile uint32_t dtknqr3_dthrctl;
	/** Device IN Token Queue Read Register-4 (Read Only). /
	 *	Device IN EPs empty Inr. Mask Register (Read/Write)
	 * <i>Offset: 834h</i> */
	volatile uint32_t dtknqr4_fifoemptymsk;
	/** Device Each Endpoint Interrupt Register (Read Only). /
	 * <i>Offset: 838h</i> */
	volatile uint32_t deachint;
	/** Device Each Endpoint Interrupt mask Register (Read/Write). /
	 * <i>Offset: 83Ch</i> */
	volatile uint32_t deachintmsk;
	/** Device Each In Endpoint Interrupt mask Register (Read/Write). /
	 * <i>Offset: 840h</i> */
	volatile uint32_t diepeachintmsk[MAX_EPS_CHANNELS];
	/** Device Each Out Endpoint Interrupt mask Register (Read/Write). /
	 * <i>Offset: 880h</i> */
	volatile uint32_t doepeachintmsk[MAX_EPS_CHANNELS];
} dwc_otg_device_global_regs_t;

/**
 * This union represents the bit fields in the Device Configuration
 * Register.  Read the register into the <i>d32</i> member then
 * set/clear the bits using the <i>b</i>it elements.  Write the
 * <i>d32</i> member to the dcfg register.
 */
typedef union dcfg_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/** Device Speed */
		USIGN devspd:2;
		/** Non Zero Length Status OUT Handshake */
		USIGN nzstsouthshk:1;
#define DWC_DCFG_SEND_STALL 1

		USIGN ena32khzs:1;
		/** Device Addresses */
		USIGN devaddr:7;
		/** Periodic Frame Interval */
		USIGN perfrint:2;
#define DWC_DCFG_FRAME_INTERVAL_80		0
#define DWC_DCFG_FRAME_INTERVAL_85		1
#define DWC_DCFG_FRAME_INTERVAL_90		2
#define DWC_DCFG_FRAME_INTERVAL_95		3

		/** Enable Device OUT NAK for bulk in DDMA mode */
		USIGN endevoutnak:1;

		USIGN reserved14_17:4;
		/** In Endpoint Mis-match count */
		USIGN epmscnt:5;
		/** Enable Descriptor DMA in Device mode */
		USIGN descdma:1;
		USIGN perschintvl:2;
		USIGN resvalid:6;
	} b;
} dcfg_data_t;

/**
 * This union represents the bit fields in the Device Control
 * Register.  Read the register into the <i>d32</i> member then
 * set/clear the bits using the <i>b</i>it elements.
 */
typedef union dctl_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/** Remote Wakeup */
		USIGN rmtwkupsig:1;
		/** Soft Disconnect */
		USIGN sftdiscon:1;
		/** Global Non-Periodic IN NAK Status */
		USIGN gnpinnaksts:1;
		/** Global OUT NAK Status */
		USIGN goutnaksts:1;
		/** Test Control */
		USIGN tstctl:3;
		/** Set Global Non-Periodic IN NAK */
		USIGN sgnpinnak:1;
		/** Clear Global Non-Periodic IN NAK */
		USIGN cgnpinnak:1;
		/** Set Global OUT NAK */
		USIGN sgoutnak:1;
		/** Clear Global OUT NAK */
		USIGN cgoutnak:1;
		/** Power-On Programming Done */
		USIGN pwronprgdone:1;
		/** Reserved */
		USIGN reserved:1;
		/** Global Multi Count */
		USIGN gmc:2;
		/** Ignore Frame Number for ISOC EPs */
		USIGN ifrmnum:1;
		/** NAK on Babble */
		USIGN nakonbble:1;
		/** Enable Continue on BNA */
		USIGN encontonbna:1;
		/** Enable deep sleep besl reject feature*/
		USIGN besl_reject:1;

		USIGN reserved17_31:13;
	} b;
} dctl_data_t;

/**
 * This union represents the bit fields in the Device Status
 * Register.  Read the register into the <i>d32</i> member then
 * set/clear the bits using the <i>b</i>it elements.
 */
typedef union dsts_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/** Suspend Status */
		USIGN suspsts:1;
		/** Enumerated Speed */
		USIGN enumspd:2;
#define DWC_DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ 0
#define DWC_DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ 1
#define DWC_DSTS_ENUMSPD_LS_PHY_6MHZ		   2
#define DWC_DSTS_ENUMSPD_FS_PHY_48MHZ		   3
		/** Erratic Error */
		USIGN errticerr:1;
		USIGN reserved4_7:4;
		/** Frame or Microframe Number of the received SOF */
		USIGN soffn:14;
		USIGN reserved22_31:10;
	} b;
} dsts_data_t;

/**
 * This union represents the bit fields in the Device IN EP Interrupt
 * Register and the Device IN EP Common Mask Register.
 *
 * - Read the register into the <i>d32</i> member then set/clear the
 *	 bits using the <i>b</i>it elements.
 */
typedef union diepint_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/** Transfer complete mask */
		USIGN xfercompl:1;
		/** Endpoint disable mask */
		USIGN epdisabled:1;
		/** AHB Error mask */
		USIGN ahberr:1;
		/** TimeOUT Handshake mask (non-ISOC EPs) */
		USIGN timeout:1;
		/** IN Token received with TxF Empty mask */
		USIGN intktxfemp:1;
		/** IN Token Received with EP mismatch mask */
		USIGN intknepmis:1;
		/** IN Endpoint NAK Effective mask */
		USIGN inepnakeff:1;
		/** Reserved */
		USIGN emptyintr:1;

		USIGN txfifoundrn:1;

		/** BNA Interrupt mask */
		USIGN bna:1;

		USIGN reserved10_12:3;
		/** BNA Interrupt mask */
		USIGN nak:1;

		USIGN reserved14_31:18;
	} b;
} diepint_data_t;

/**
 * This union represents the bit fields in the Device IN EP
 * Common/Dedicated Interrupt Mask Register.
 */
typedef union diepint_data diepmsk_data_t;

/**
 * This union represents the bit fields in the Device OUT EP Interrupt
 * Registerand Device OUT EP Common Interrupt Mask Register.
 *
 * - Read the register into the <i>d32</i> member then set/clear the
 *	 bits using the <i>b</i>it elements.
 */
typedef union doepint_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/** Transfer complete */
		USIGN xfercompl:1;
		/** Endpoint disable  */
		USIGN epdisabled:1;
		/** AHB Error */
		USIGN ahberr:1;
		/** Setup Phase Done (contorl EPs) */
		USIGN setup:1;
		/** OUT Token Received when Endpoint Disabled */
		USIGN outtknepdis:1;

		USIGN stsphsercvd:1;
		/** Back-to-Back SETUP Packets Received */
		USIGN back2backsetup:1;

		USIGN reserved7:1;
		/** OUT packet Error */
		USIGN outpkterr:1;
		/** BNA Interrupt */
		USIGN bna:1;

		USIGN reserved10:1;
		/** Packet Drop Status */
		USIGN pktdrpsts:1;
		/** Babble Interrupt */
		USIGN babble:1;
		/** NAK Interrupt */
		USIGN nak:1;
		/** NYET Interrupt */
		USIGN nyet:1;
		/** Bit indicating setup packet received */
		USIGN sr:1;

		USIGN reserved16_31:16;
	} b;
} doepint_data_t;

/**
 * This union represents the bit fields in the Device OUT EP
 * Common/Dedicated Interrupt Mask Register.
 */
typedef union doepint_data doepmsk_data_t;

/**
 * This union represents the bit fields in the Device All EP Interrupt
 * and Mask Registers.
 * - Read the register into the <i>d32</i> member then set/clear the
 *	 bits using the <i>b</i>it elements.
 */
typedef union daint_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/** IN Endpoint bits */
		USIGN in:16;
		/** OUT Endpoint bits */
		USIGN out:16;
	} ep;
	struct {
		/** IN Endpoint bits */
		USIGN inep0:1;
		USIGN inep1:1;
		USIGN inep2:1;
		USIGN inep3:1;
		USIGN inep4:1;
		USIGN inep5:1;
		USIGN inep6:1;
		USIGN inep7:1;
		USIGN inep8:1;
		USIGN inep9:1;
		USIGN inep10:1;
		USIGN inep11:1;
		USIGN inep12:1;
		USIGN inep13:1;
		USIGN inep14:1;
		USIGN inep15:1;
		/** OUT Endpoint bits */
		USIGN outep0:1;
		USIGN outep1:1;
		USIGN outep2:1;
		USIGN outep3:1;
		USIGN outep4:1;
		USIGN outep5:1;
		USIGN outep6:1;
		USIGN outep7:1;
		USIGN outep8:1;
		USIGN outep9:1;
		USIGN outep10:1;
		USIGN outep11:1;
		USIGN outep12:1;
		USIGN outep13:1;
		USIGN outep14:1;
		USIGN outep15:1;
	} b;
} daint_data_t;

/**
 * This union represents the bit fields in the Device IN Token Queue
 * Read Registers.
 * - Read the register into the <i>d32</i> member.
 * - READ-ONLY Register
 */
typedef union dtknq1_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/** In Token Queue Write Pointer */
		USIGN intknwptr:5;
		/** Reserved */
		USIGN reserved05_06:2;
		/** write pointer has wrapped. */
		USIGN wrap_bit:1;
		/** EP Numbers of IN Tokens 0 ... 4 */
		USIGN epnums0_5:24;
	} b;
} dtknq1_data_t;

/**
 * This union represents Threshold control Register
 * - Read and write the register into the <i>d32</i> member.
 * - READ-WRITABLE Register
 */
typedef union dthrctl_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/** non ISO Tx Thr. Enable */
		USIGN non_iso_thr_en:1;
		/** ISO Tx Thr. Enable */
		USIGN iso_thr_en:1;
		/** Tx Thr. Length */
		USIGN tx_thr_len:9;
		/** AHB Threshold ratio */
		USIGN ahb_thr_ratio:2;
		/** Reserved */
		USIGN reserved13_15:3;
		/** Rx Thr. Enable */
		USIGN rx_thr_en:1;
		/** Rx Thr. Length */
		USIGN rx_thr_len:9;
		USIGN reserved26:1;
		/** Arbiter Parking Enable*/
		USIGN arbprken:1;
		/** Reserved */
		USIGN reserved28_31:4;
	} b;
} dthrctl_data_t;

/**
 * Device Logical IN Endpoint-Specific Registers. <i>Offsets
 * 900h-AFCh</i>
 *
 * There will be one set of endpoint registers per logical endpoint
 * implemented.
 *
 * <i>These registers are visible only in Device mode and must not be
 * accessed in Host mode, as the results are unknown.</i>
 */
typedef struct dwc_otg_dev_in_ep_regs {
	/** Device IN Endpoint Control Register. <i>Offset:900h +
	 * (ep_num * 20h) + 00h</i> */
	volatile uint32_t diepctl;
	/** Reserved. <i>Offset:900h + (ep_num * 20h) + 04h</i> */
	uint32_t reserved04;
	/** Device IN Endpoint Interrupt Register. <i>Offset:900h +
	 * (ep_num * 20h) + 08h</i> */
	volatile uint32_t diepint;
	/** Reserved. <i>Offset:900h + (ep_num * 20h) + 0Ch</i> */
	uint32_t reserved0C;
	/** Device IN Endpoint Transfer Size
	 * Register. <i>Offset:900h + (ep_num * 20h) + 10h</i> */
	volatile uint32_t dieptsiz;
	/** Device IN Endpoint DMA Address Register. <i>Offset:900h +
	 * (ep_num * 20h) + 14h</i> */
	volatile uint32_t diepdma;
	/** Device IN Endpoint Transmit FIFO Status Register. <i>Offset:900h +
	 * (ep_num * 20h) + 18h</i> */
	volatile uint32_t dtxfsts;
	/** Device IN Endpoint DMA Buffer Register. <i>Offset:900h +
	 * (ep_num * 20h) + 1Ch</i> */
	volatile uint32_t diepdmab;
} dwc_otg_dev_in_ep_regs_t;

/**
 * Device Logical OUT Endpoint-Specific Registers. <i>Offsets:
 * B00h-CFCh</i>
 *
 * There will be one set of endpoint registers per logical endpoint
 * implemented.
 *
 * <i>These registers are visible only in Device mode and must not be
 * accessed in Host mode, as the results are unknown.</i>
 */
typedef struct dwc_otg_dev_out_ep_regs {
	/** Device OUT Endpoint Control Register. <i>Offset:B00h +
	 * (ep_num * 20h) + 00h</i> */
	volatile uint32_t doepctl;
	/** Reserved. <i>Offset:B00h + (ep_num * 20h) + 04h</i> */
	uint32_t reserved04;
	/** Device OUT Endpoint Interrupt Register. <i>Offset:B00h +
	 * (ep_num * 20h) + 08h</i> */
	volatile uint32_t doepint;
	/** Reserved. <i>Offset:B00h + (ep_num * 20h) + 0Ch</i> */
	uint32_t reserved0C;
	/** Device OUT Endpoint Transfer Size Register. <i>Offset:
	 * B00h + (ep_num * 20h) + 10h</i> */
	volatile uint32_t doeptsiz;
	/** Device OUT Endpoint DMA Address Register. <i>Offset:B00h
	 * + (ep_num * 20h) + 14h</i> */
	volatile uint32_t doepdma;
	/** Reserved. <i>Offset:B00h + 	 * (ep_num * 20h) + 18h</i> */
	uint32_t unused;
	/** Device OUT Endpoint DMA Buffer Register. <i>Offset:B00h
	 * + (ep_num * 20h) + 1Ch</i> */
	uint32_t doepdmab;
} dwc_otg_dev_out_ep_regs_t;

/**
 * This union represents the bit fields in the Device EP Control
 * Register.  Read the register into the <i>d32</i> member then
 * set/clear the bits using the <i>b</i>it elements.
 */
typedef union depctl_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/** Maximum Packet Size
		 * IN/OUT EPn
		 * IN/OUT EP0 - 2 bits
		 *	 2'b00: 64 Bytes
		 *	 2'b01: 32
		 *	 2'b10: 16
		 *	 2'b11: 8 */
		USIGN mps:11;
#define DWC_DEP0CTL_MPS_64				0
#define DWC_DEP0CTL_MPS_32				1
#define DWC_DEP0CTL_MPS_16				2
#define DWC_DEP0CTL_MPS_8				3

		/** Next Endpoint
		 * IN EPn/IN EP0
		 * OUT EPn/OUT EP0 - reserved */
		USIGN nextep:4;

		/** USB Active Endpoint */
		USIGN usbactep:1;

		/** Endpoint DPID (INTR/Bulk IN and OUT endpoints)
		 * This field contains the PID of the packet going to
		 * be received or transmitted on this endpoint. The
		 * application should program the PID of the first
		 * packet going to be received or transmitted on this
		 * endpoint , after the endpoint is
		 * activated. Application use the SetD1PID and
		 * SetD0PID fields of this register to program either
		 * D0 or D1 PID.
		 *
		 * The encoding for this field is
		 *	 - 0: D0
		 *	 - 1: D1
		 */
		USIGN dpid:1;

		/** NAK Status */
		USIGN naksts:1;

		/** Endpoint Type
		 *	2'b00: Control
		 *	2'b01: Isochronous
		 *	2'b10: Bulk
		 *	2'b11: Interrupt */
		USIGN eptype:2;

		/** Snoop Mode
		 * OUT EPn/OUT EP0
		 * IN EPn/IN EP0 - reserved */
		USIGN snp:1;

		/** Stall Handshake */
		USIGN stall:1;

		/** Tx Fifo Number
		 * IN EPn/IN EP0
		 * OUT EPn/OUT EP0 - reserved */
		USIGN txfnum:4;

		/** Clear NAK */
		USIGN cnak:1;
		/** Set NAK */
		USIGN snak:1;
		/** Set DATA0 PID (INTR/Bulk IN and OUT endpoints)
		 * Writing to this field sets the Endpoint DPID (DPID)
		 * field in this register to DATA0. Set Even
		 * (micro)frame (SetEvenFr) (ISO IN and OUT Endpoints)
		 * Writing to this field sets the Even/Odd
		 * (micro)frame (EO_FrNum) field to even (micro)
		 * frame.
		 */
		USIGN setd0pid:1;
		/** Set DATA1 PID (INTR/Bulk IN and OUT endpoints)
		 * Writing to this field sets the Endpoint DPID (DPID)
		 * field in this register to DATA1 Set Odd
		 * (micro)frame (SetOddFr) (ISO IN and OUT Endpoints)
		 * Writing to this field sets the Even/Odd
		 * (micro)frame (EO_FrNum) field to odd (micro) frame.
		 */
		USIGN setd1pid:1;

		/** Endpoint Disable */
		USIGN epdis:1;
		/** Endpoint Enable */
		USIGN epena:1;
	} b;
} depctl_data_t;

/**
 * This union represents the bit fields in the Device EP Transfer
 * Size Register.  Read the register into the <i>d32</i> member then
 * set/clear the bits using the <i>b</i>it elements.
 */
typedef union deptsiz_data {
		/** raw register data */
	uint32_t d32;
		/** register bits */
	struct {
		/** Transfer size */
		USIGN xfersize:19;
/** Max packet count for EP (pow(2,10)-1) */
#define MAX_PKT_CNT					1023
		/** Packet Count */
		USIGN pktcnt:10;
		/** Multi Count - Periodic IN endpoints */
		USIGN mc:2;
		USIGN reserved:1;
	} b;
} deptsiz_data_t;

/**
 * This union represents the bit fields in the Device EP 0 Transfer
 * Size Register.  Read the register into the <i>d32</i> member then
 * set/clear the bits using the <i>b</i>it elements.
 */
typedef union deptsiz0_data {
		/** raw register data */
	uint32_t d32;
		/** register bits */
	struct {
		/** Transfer size */
		USIGN xfersize:7;
				/** Reserved */
		USIGN reserved7_18:12;
		/** Packet Count */
		USIGN pktcnt:2;
				/** Reserved */
		USIGN reserved21_28:8;
				/**Setup Packet Count (DOEPTSIZ0 Only) */
		USIGN supcnt:2;
		USIGN reserved31;
	} b;
} deptsiz0_data_t;

/////////////////////////////////////////////////
// DMA Descriptor Specific Structures
//

/** Buffer status definitions */

#define BS_HOST_READY				0x0
#define BS_DMA_BUSY					0x1
#define BS_DMA_DONE					0x2
#define BS_HOST_BUSY				0x3

/** Receive/Transmit status definitions */

#define RTS_SUCCESS					0x0
#define RTS_BUFFLUSH				0x1
#define RTS_RESERVED				0x2
#define RTS_BUFERR					0x3

/**
 * This union represents the bit fields in the DMA Descriptor
 * status quadlet. Read the quadlet into the <i>d32</i> member then
 * set/clear the bits using the <i>b</i>it, <i>b_iso_out</i> and
 * <i>b_iso_in</i> elements.
 */
typedef union dev_dma_desc_sts {
		/** raw register data */
	uint32_t d32;
		/** quadlet bits */
	struct {
		/** Received number of bytes */
		USIGN bytes:16;
		/** NAK bit - only for OUT EPs */
		USIGN nak:1;
		USIGN reserved17_22:6;
		/** Multiple Transfer - only for OUT EPs */
		USIGN mtrf:1;
		/** Setup Packet received - only for OUT EPs */
		USIGN sr:1;
		/** Interrupt On Complete */
		USIGN ioc:1;
		/** Short Packet */
		USIGN sp:1;
		/** Last */
		USIGN l:1;
		/** Receive Status */
		USIGN sts:2;
		/** Buffer Status */
		USIGN bs:2;
	} b;

} dev_dma_desc_sts_t;

/**
 * DMA Descriptor structure
 *
 * DMA Descriptor structure contains two quadlets:
 * Status quadlet and Data buffer pointer.
 */
typedef struct dwc_otg_dev_dma_desc {
	/** DMA Descriptor status quadlet */
	dev_dma_desc_sts_t status;
	/** DMA Descriptor data buffer pointer */
	uint32_t buf;
} dwc_otg_dev_dma_desc_t;

/**
 * The dwc_otg_dev_if structure contains information needed to manage
 * the DWC_otg controller acting in device mode. It represents the
 * programming view of the device-specific aspects of the controller.
 */
typedef struct dwc_otg_dev_if {
	/** Pointer to device Global registers.
	 * Device Global Registers starting at offset 800h
	 */
	dwc_otg_device_global_regs_t *dev_global_regs;
#define DWC_DEV_GLOBAL_REG_OFFSET		0x800

	/**
	 * Device Logical IN Endpoint-Specific Registers 900h-AFCh
	 */
	dwc_otg_dev_in_ep_regs_t *in_ep_regs[MAX_EPS_CHANNELS];
#define DWC_DEV_IN_EP_REG_OFFSET		0x900
#define DWC_EP_REG_OFFSET				0x20

	/** Device Logical OUT Endpoint-Specific Registers B00h-CFCh */
	dwc_otg_dev_out_ep_regs_t *out_ep_regs[MAX_EPS_CHANNELS];
#define DWC_DEV_OUT_EP_REG_OFFSET		0xB00

	/* Device configuration information */
	uint8_t speed;				 /**< Device Speed	0: Unknown, 1: LS, 2:FS, 3: HS */
	uint8_t num_in_eps;		 /**< Number # of Tx EP range: 0-15 exept ep0 */
	uint8_t num_out_eps;		 /**< Number # of Rx EP range: 0-15 exept ep 0*/

	/** Size of periodic FIFOs (Bytes) */
	uint16_t perio_tx_fifo_size[MAX_PERIO_FIFOS];

	/** Size of Tx FIFOs (Bytes) */
	uint16_t tx_fifo_size[MAX_TX_FIFOS];

	/** Thresholding enable flags and length varaiables **/
	uint16_t rx_thr_en;
	uint16_t non_iso_tx_thr_en;

	uint16_t rx_thr_length;
	uint16_t tx_thr_length;

	/**
	 * Pointers to the DMA Descriptors for EP0 Control
	 * transfers (virtual and physical)
	 */

	/** 2 descriptors for SETUP packets */
	dwc_dma_t dma_setup_desc_addr[2];
	dwc_otg_dev_dma_desc_t *setup_desc_addr[2];

	/** Pointer to Descriptor with latest SETUP packet */
	dwc_otg_dev_dma_desc_t *psetup;

	/** Index of current SETUP handler descriptor */
	uint32_t  setup_desc_index;

	/** Descriptor for Data In or Status In phases */
	dwc_dma_t dma_in_desc_addr;
	dwc_otg_dev_dma_desc_t *in_desc_addr;

	/** Descriptor for Data Out or Status Out phases */
	dwc_dma_t dma_out_desc_addr;
	dwc_otg_dev_dma_desc_t *out_desc_addr;

	/** Setup Packet Detected - if set clear NAK when queueing */
	uint32_t spd;

} dwc_otg_dev_if_t;

/////////////////////////////////////////////////
// Host Mode Register Structures
//
/**
 * The Host Global Registers structure defines the size and relative
 * field offsets for the Host Mode Global Registers.  Host Global
 * Registers offsets 400h-7FFh.
*/
typedef struct dwc_otg_host_global_regs {
	/** Host Configuration Register.   <i>Offset: 400h</i> */
	volatile uint32_t hcfg;
	/** Host Frame Interval Register.	<i>Offset: 404h</i> */
	volatile uint32_t hfir;
	/** Host Frame Number / Frame Remaining Register. <i>Offset: 408h</i> */
	volatile uint32_t hfnum;
	/** Reserved.	<i>Offset: 40Ch</i> */
	uint32_t reserved40C;
	/** Host Periodic Transmit FIFO/ Queue Status Register. <i>Offset: 410h</i> */
	volatile uint32_t hptxsts;
	/** Host All Channels Interrupt Register. <i>Offset: 414h</i> */
	volatile uint32_t haint;
	/** Host All Channels Interrupt Mask Register. <i>Offset: 418h</i> */
	volatile uint32_t haintmsk;
	/** Host Frame List Base Address Register . <i>Offset: 41Ch</i> */
	volatile uint32_t hflbaddr;
} dwc_otg_host_global_regs_t;

/**
 * This union represents the bit fields in the Host Configuration Register.
 * Read the register into the <i>d32</i> member then set/clear the bits using
 * the <i>b</i>it elements. Write the <i>d32</i> member to the hcfg register.
 */
typedef union hcfg_data {
	/** raw register data */
	uint32_t d32;

	/** register bits */
	struct {
		/** FS/LS Phy Clock Select */
		USIGN fslspclksel:2;
#define DWC_HCFG_30_60_MHZ				0
#define DWC_HCFG_48_MHZ	  				1
#define DWC_HCFG_6_MHZ	  				2

		/** FS/LS Only Support */
		USIGN fslssupp:1;
		USIGN reserved3_6:4;
		/** Enable 32-KHz Suspend Mode */
		USIGN ena32khzs:1;
		/** Resume Validation Periiod */
		USIGN resvalid:8;
		USIGN reserved16_22:7;
		/** Enable Scatter/gather DMA in Host mode */
		USIGN descdma:1;
		/** Frame List Entries */
		USIGN frlisten:2;
		/** Enable Periodic Scheduling */
		USIGN perschedena:1;
		USIGN reserved27_30:4;
		USIGN modechtimen:1;
	} b;
} hcfg_data_t;

/**
 * This union represents the bit fields in the Host Frame Remaing/Number
 * Register. 
 */
typedef union hfir_data {
	/** raw register data */
	uint32_t d32;

	/** register bits */
	struct {
		USIGN frint:16;
		USIGN hfirrldctrl:1;
		USIGN reserved:15;
	} b;
} hfir_data_t;

/**
 * This union represents the bit fields in the Host Frame Remaing/Number
 * Register. 
 */
typedef union hfnum_data {
	/** raw register data */
	uint32_t d32;

	/** register bits */
	struct {
		USIGN frnum:16;
#define DWC_HFNUM_MAX_FRNUM				0x3FFF
		USIGN frrem:16;
	} b;
} hfnum_data_t;

typedef union hptxsts_data {
	/** raw register data */
	uint32_t d32;

	/** register bits */
	struct {
		USIGN ptxfspcavail:16;
		USIGN ptxqspcavail:8;
		/** Top of the Periodic Transmit Request Queue
		 *	- bit 24 - Terminate (last entry for the selected channel)
		 *	- bits 26:25 - Token Type
		 *	  - 2'b00 - Zero length
		 *	  - 2'b01 - Ping
		 *	  - 2'b10 - Disable
		 *	- bits 30:27 - Channel Number
		 *	- bit 31 - Odd/even microframe
		 */
		USIGN ptxqtop_terminate:1;
		USIGN ptxqtop_token:2;
		USIGN ptxqtop_chnum:4;
		USIGN ptxqtop_odd:1;
	} b;
} hptxsts_data_t;

/**
 * This union represents the bit fields in the Host Port Control and Status
 * Register. Read the register into the <i>d32</i> member then set/clear the
 * bits using the <i>b</i>it elements. Write the <i>d32</i> member to the
 * hprt0 register.
 */
typedef union hprt0_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN prtconnsts:1;
		USIGN prtconndet:1;
		USIGN prtena:1;
		USIGN prtenchng:1;
		USIGN prtovrcurract:1;
		USIGN prtovrcurrchng:1;
		USIGN prtres:1;
		USIGN prtsusp:1;
		USIGN prtrst:1;
		USIGN reserved9:1;
		USIGN prtlnsts:2;
		USIGN prtpwr:1;
		USIGN prttstctl:4;
		USIGN prtspd:2;
#define DWC_HPRT0_PRTSPD_HIGH_SPEED		0
#define DWC_HPRT0_PRTSPD_FULL_SPEED		1
#define DWC_HPRT0_PRTSPD_LOW_SPEED		2
		USIGN reserved19_31:13;
	} b;
} hprt0_data_t;

/**
 * This union represents the bit fields in the Host All Interrupt
 * Register. 
 */
typedef union haint_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN ch0:1;
		USIGN ch1:1;
		USIGN ch2:1;
		USIGN ch3:1;
		USIGN ch4:1;
		USIGN ch5:1;
		USIGN ch6:1;
		USIGN ch7:1;
		USIGN ch8:1;
		USIGN ch9:1;
		USIGN ch10:1;
		USIGN ch11:1;
		USIGN ch12:1;
		USIGN ch13:1;
		USIGN ch14:1;
		USIGN ch15:1;
		USIGN reserved:16;
	} b;

	struct {
		USIGN chint:16;
		USIGN reserved:16;
	} b2;
} haint_data_t;

/**
 * This union represents the bit fields in the Host All Interrupt
 * Register. 
 */
typedef union haintmsk_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN ch0:1;
		USIGN ch1:1;
		USIGN ch2:1;
		USIGN ch3:1;
		USIGN ch4:1;
		USIGN ch5:1;
		USIGN ch6:1;
		USIGN ch7:1;
		USIGN ch8:1;
		USIGN ch9:1;
		USIGN ch10:1;
		USIGN ch11:1;
		USIGN ch12:1;
		USIGN ch13:1;
		USIGN ch14:1;
		USIGN ch15:1;
		USIGN reserved:16;
	} b;

	struct {
		USIGN chint:16;
		USIGN reserved:16;
	} b2;
} haintmsk_data_t;

/**
 * Host Channel Specific Registers. <i>500h-5FCh</i>
 */
typedef struct dwc_otg_hc_regs {
	/** Host Channel 0 Characteristic Register. <i>Offset: 500h + (chan_num * 20h) + 00h</i> */
	volatile uint32_t hcchar;
	/** Host Channel 0 Split Control Register. <i>Offset: 500h + (chan_num * 20h) + 04h</i> */
	volatile uint32_t hcsplt;
	/** Host Channel 0 Interrupt Register. <i>Offset: 500h + (chan_num * 20h) + 08h</i> */
	volatile uint32_t hcint;
	/** Host Channel 0 Interrupt Mask Register. <i>Offset: 500h + (chan_num * 20h) + 0Ch</i> */
	volatile uint32_t hcintmsk;
	/** Host Channel 0 Transfer Size Register. <i>Offset: 500h + (chan_num * 20h) + 10h</i> */
	volatile uint32_t hctsiz;
	/** Host Channel 0 DMA Address Register. <i>Offset: 500h + (chan_num * 20h) + 14h</i> */
	volatile uint32_t hcdma;
	volatile uint32_t reserved;
	/** Host Channel 0 DMA Buffer Address Register. <i>Offset: 500h + (chan_num * 20h) + 1Ch</i> */
	volatile uint32_t hcdmab;
} dwc_otg_hc_regs_t;

/**
 * This union represents the bit fields in the Host Channel Characteristics
 * Register. Read the register into the <i>d32</i> member then set/clear the
 * bits using the <i>b</i>it elements. Write the <i>d32</i> member to the
 * hcchar register.
 */
typedef union hcchar_data {
	/** raw register data */
	uint32_t d32;

	/** register bits */
	struct {
		/** Maximum packet size in bytes */
		USIGN mps:11;

		/** Endpoint number */
		USIGN epnum:4;

		/** 0: OUT, 1: IN */
		USIGN epdir:1;

		USIGN reserved:1;

		/** 0: Full/high speed device, 1: Low speed device */
		USIGN lspddev:1;

		/** 0: Control, 1: Isoc, 2: Bulk, 3: Intr */
		USIGN eptype:2;

		/** Packets per frame for periodic transfers. 0 is reserved. */
		USIGN multicnt:2;

		/** Device address */
		USIGN devaddr:7;

		/**
		 * Frame to transmit periodic transaction.
		 * 0: even, 1: odd
		 */
		USIGN oddfrm:1;

		/** Channel disable */
		USIGN chdis:1;

		/** Channel enable */
		USIGN chen:1;
	} b;
} hcchar_data_t;

typedef union hcsplt_data {
	/** raw register data */
	uint32_t d32;

	/** register bits */
	struct {
		/** Port Address */
		USIGN prtaddr:7;

		/** Hub Address */
		USIGN hubaddr:7;

		/** Transaction Position */
		USIGN xactpos:2;
#define DWC_HCSPLIT_XACTPOS_MID		0
#define DWC_HCSPLIT_XACTPOS_END		1
#define DWC_HCSPLIT_XACTPOS_BEGIN	2
#define DWC_HCSPLIT_XACTPOS_ALL		3

		/** Do Complete Split */
		USIGN compsplt:1;

		/** Reserved */
		USIGN reserved:14;

		/** Split Enble */
		USIGN spltena:1;
	} b;
} hcsplt_data_t;

/**
 * This union represents the bit fields in the Host All Interrupt
 * Register. 
 */
typedef union hcint_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/** Transfer Complete */
		USIGN xfercomp:1;
		/** Channel Halted */
		USIGN chhltd:1;
		/** AHB Error */
		USIGN ahberr:1;
		/** STALL Response Received */
		USIGN stall:1;
		/** NAK Response Received */
		USIGN nak:1;
		/** ACK Response Received */
		USIGN ack:1;
		/** NYET Response Received */
		USIGN nyet:1;
		/** Transaction Err */
		USIGN xacterr:1;
		/** Babble Error */
		USIGN bblerr:1;
		/** Frame Overrun */
		USIGN frmovrun:1;
		/** Data Toggle Error */
		USIGN datatglerr:1;
		/** Buffer Not Available (only for DDMA mode) */
		USIGN bna:1;
		/** Exessive transaction error (only for DDMA mode) */
		USIGN xcs_xact:1;
		/** Frame List Rollover interrupt */
		USIGN frm_list_roll:1;
		/** Reserved */
		USIGN reserved14_31:18;
	} b;
} hcint_data_t;

/**
 * This union represents the bit fields in the Host Channel Interrupt Mask
 * Register. Read the register into the <i>d32</i> member then set/clear the
 * bits using the <i>b</i>it elements. Write the <i>d32</i> member to the
 * hcintmsk register.
 */
typedef union hcintmsk_data {
	/** raw register data */
	uint32_t d32;

	/** register bits */
	struct {
		USIGN xfercompl:1;
		USIGN chhltd:1;
		USIGN ahberr:1;
		USIGN stall:1;
		USIGN nak:1;
		USIGN ack:1;
		USIGN nyet:1;
		USIGN xacterr:1;
		USIGN bblerr:1;
		USIGN frmovrun:1;
		USIGN datatglerr:1;
		USIGN bna:1;
		USIGN xcs_xact:1;
		USIGN frm_list_roll:1;
		USIGN reserved14_31:18;
	} b;
} hcintmsk_data_t;

/**
 * This union represents the bit fields in the Host Channel Transfer Size
 * Register. Read the register into the <i>d32</i> member then set/clear the
 * bits using the <i>b</i>it elements. Write the <i>d32</i> member to the
 * hcchar register.
 */

typedef union hctsiz_data {
	/** raw register data */
	uint32_t d32;

	/** register bits */
	struct {
		/** Total transfer size in bytes */
		USIGN xfersize:19;

		/** Data packets to transfer */
		USIGN pktcnt:10;

		/**
		 * Packet ID for next data packet
		 * 0: DATA0
		 * 1: DATA2
		 * 2: DATA1
		 * 3: MDATA (non-Control), SETUP (Control)
		 */
		USIGN pid:2;
#define DWC_HCTSIZ_DATA0				0
#define DWC_HCTSIZ_DATA1				2
#define DWC_HCTSIZ_DATA2				1
#define DWC_HCTSIZ_MDATA				3
#define DWC_HCTSIZ_SETUP				3

		/** Do PING protocol when 1 */
		USIGN dopng:1;
	} b;

	/** register bits */
	struct {
		/** Scheduling information */
		USIGN schinfo:8;

		/** Number of transfer descriptors.
		 * Max value:
		 * 64 in general,
		 * 256 only for HS isochronous endpoint.
		 */
		USIGN ntd:8;

		/** Data packets to transfer */
		USIGN reserved16_28:13;

		/**
		 * Packet ID for next data packet
		 * 0: DATA0
		 * 1: DATA2
		 * 2: DATA1
		 * 3: MDATA (non-Control)
		 */
		USIGN pid:2;

		/** Do PING protocol when 1 */
		USIGN dopng:1;
	} b_ddma;
} hctsiz_data_t;

/**
 * This union represents the bit fields in the Host DMA Address 
 * Register used in Descriptor DMA mode.
 */
typedef union hcdma_data {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		USIGN reserved0_2:3;
		/** Current Transfer Descriptor. Not used for ISOC */
		USIGN ctd:8;
		/** Start Address of Descriptor List */
		USIGN dma_addr:21;
	} b;
} hcdma_data_t;

#define	MAX_DMA_DESC_SIZE				131071
#define MAX_DMA_DESC_NUM_GENERIC		64
#define MAX_FRLIST_EN_NUM				64

/**
 * This union represents the bit fields in the Power and Clock Gating Control
 * Register. Read the register into the <i>d32</i> member then set/clear the
 * bits using the <i>b</i>it elements.
 */
typedef union pcgcctl_data {
	/** raw register data */
	uint32_t d32;

	/** register bits */
	struct {
		/** Stop Pclk */
		USIGN stoppclk:1;
		/** Gate Hclk */
		USIGN gatehclk:1;
		/** Power Clamp */
		USIGN pwrclmp:1;
		/** Reset Power Down Modules */
		USIGN rstpdwnmodule:1;
		/** Reserved */
		USIGN reserved:1;
		/** Enable Sleep Clock Gating (Enbl_L1Gating) */
		USIGN enbl_sleep_gating:1;
		/** PHY In Sleep (PhySleep) */
		USIGN phy_in_sleep:1;
		/** Deep Sleep*/
		USIGN deep_sleep:1;
		USIGN resetaftsusp:1;
		USIGN restoremode:1;
		USIGN enbl_extnd_hiber:1;
		USIGN extnd_hiber_pwrclmp:1;
		USIGN extnd_hiber_switch:1;
		USIGN ess_reg_restored:1;
		USIGN prt_clk_sel:2;
		USIGN port_power:1;
		USIGN max_xcvrselect:2;
		USIGN max_termsel:1;
		USIGN mac_dev_addr:7;
		USIGN p2hd_dev_enum_spd:2;
		USIGN p2hd_prt_spd:2;
		USIGN if_dev_mode:1;
	} b;
} pcgcctl_data_t;

/**
 * This union represents the bit fields in the Global Data FIFO Software
 * Configuration Register. Read the register into the <i>d32</i> member then
 * set/clear the bits using the <i>b</i>it elements.
 */
typedef union gdfifocfg_data {
	/* raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		/** OTG Data FIFO depth */
		USIGN gdfifocfg:16;
		/** Start address of EP info controller */
		USIGN epinfobase:16;
	} b;
} gdfifocfg_data_t;

/**
 * This union represents the bit fields in the Global Power Down Register
 * Register. Read the register into the <i>d32</i> member then set/clear the
 * bits using the <i>b</i>it elements.
 */
typedef union gpwrdn_data {
	/* raw register data */
	uint32_t d32;

	/** register bits */
	struct {
		/** PMU Interrupt Select */
		USIGN pmuintsel:1;
		/** PMU Active */
		USIGN pmuactv:1;
		/** Restore */
		USIGN restore:1;
		/** Power Down Clamp */
		USIGN pwrdnclmp:1;
		/** Power Down Reset */
		USIGN pwrdnrstn:1;
		/** Power Down Switch */
		USIGN pwrdnswtch:1;
		/** Disable VBUS */
		USIGN dis_vbus:1;
		/** Line State Change */
		USIGN lnstschng:1;
		/** Line state change mask */
		USIGN lnstchng_msk:1;
		/** Reset Detected */
		USIGN rst_det:1;
		/** Reset Detect mask */
		USIGN rst_det_msk:1;
		/** Disconnect Detected */
		USIGN disconn_det:1;
		/** Disconnect Detect mask */
		USIGN disconn_det_msk:1;
		/** Connect Detected*/
		USIGN connect_det:1;
		/** Connect Detected Mask*/
		USIGN connect_det_msk:1;
		/** SRP Detected */
		USIGN srp_det:1;
		/** SRP Detect mask */
		USIGN srp_det_msk:1;
		/** Status Change Interrupt */
		USIGN sts_chngint:1;
		/** Status Change Interrupt Mask */
		USIGN sts_chngint_msk:1;
		/** Line State */
		USIGN linestate:2;
		/** Indicates current mode(status of IDDIG signal) */
		USIGN idsts:1;
		/** B Session Valid signal status*/
		USIGN bsessvld:1;
		/** ADP Event Detected */
		USIGN adp_int:1;
		/** Multi Valued ID pin */
		USIGN mult_val_id_bc:5;
		/** Reserved 24_31 */
		USIGN reserved29_31:3;
	} b;
} gpwrdn_data_t;

#endif
