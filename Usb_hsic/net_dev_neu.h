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

/*
*********************************************************************************************************
*												MODULE
*********************************************************************************************************
*/

#ifndef	 __NET_DEV_NEU_H__
#define	 __NET_DEV_NEU_H__

#include "includes.h"
#include "dwc_otg_pcd.h"
#include "tc9560_common.h"

/*
*********************************************************************************************************
*									   DEVICE DRIVER ERROR CODES
*
* Note(s) : (1) ALL device-independent error codes #define'd in		 'net_err.h';
*				ALL device-specific	   error codes #define'd in this 'net_dev_&&&.h'.
*
*			(2) Network error code '11,000' series reserved for network device drivers.
*				See 'net_err.h	NETWORK DEVICE ERROR CODES' to ensure that device-specific 
*				error codes do NOT conflict with device-independent error codes.
*********************************************************************************************************
*/

#define GMAC_BASE_ADD 0 
#define MTL_BASE_ADD GMAC_BASE_ADD

/* Error Codes */
#define ESYNOPGMACNOERR			0
#define ESYNOPGMACNOMEM			1
#define ESYNOPGMACPHYERR		2
#define ESYNOPGMACBUSY			3

#define DEFAULT_DELAY_VARIABLE	10
#define DEFAULT_LOOP_VARIABLE	10000

/* -------------------------	  R E G I S T E R S ------------------- */

/**********************************************************
 * GMAC registers Map
 **********************************************************/
enum GmacRegisters				
{
	GmacConfig					= 0x0000U,	/* Mac config Register						 */	   
	GmacExtConfig				= 0x0004U,	/* Mac extended operaion Register			 */	 
	GmacFrameFilter				= 0x0008U,	/* Mac packet filtering controls			 */
	GmacWdTimeout				= 0x000CU,	/* Controls the watchdog timeout for received packets.*/	
	GmacHashReg0				= 0x0010U,	/* Multi-cast hash table Reg0				 */
	GmacHashReg1				= 0x0014U,	/* Multi-cast hash table Reg1				 */
	GmacHashReg2				= 0x0018U,	/* Multi-cast hash table Reg2				 */
	GmacHashReg3				= 0x001CU,	/* Multi-cast hash table Reg3				 */
	GmacHashReg4				= 0x0020U,	/* Multi-cast hash table Reg4				 */
	GmacHashReg5				= 0x0024U,	/* Multi-cast hash table Reg5				 */
	GmacHashReg6				= 0x0028U,	/* Multi-cast hash table Reg6				 */
	GmacHashReg7				= 0x002CU,	/* Multi-cast hash table Reg7				 */
	/*************************** 0x0030	  to   0x004C : Reserved ************************************/		
	GmacVlan					= 0x0050U,	/* VLAN tag Register (IEEE 802.1Q)			 */
	/*************************** 0x0054 : Reserved **************************************************/		
	GmacVlanHashTable			= 0x0058U,	/* register contains the VLAN hash table.	 */
	/*************************** 0x005C : Reserved **************************************************/		
	GmacVlanIncl				= 0x0060U,	/* VLAN tag for insertion or replacement	 */	 
	GmacInnVlanIncl				= 0x0064U,	/* Inner VLAN tag for insertion or replacement	   */	  
	/*************************** 0x0068	  to   0x006C : Reserved ************************************/	 
	GmacQ0TxCtrl				= 0x0070U,	/*register control packets in Queue 0.*/	
	GmacQ1TxCtrl				= 0x0074U,	/*register control packets in Queue 1.*/	
	GmacQ2TxCtrl				= 0x0078U,	/*register control packets in Queue 2.*/	
	GmacQ3TxCtrl				= 0x007CU,	/*register control packets in Queue 3.*/	
	GmacQ4TxCtrl				= 0x0080U,	/*register control packets in Queue 4.*/	
	GmacQ5TxCtrl				= 0x0084U,	/*register control packets in Queue 5.*/	
	GmacQ6TxCtrl				= 0x0088U,	/*register control packets in Queue 6.*/	
	GmacQ7TxCtrl				= 0x008CU,	/*register control packets in Queue 7.*/	
	GmacRxCtrl					= 0x0090U,	/* MAC Transmit based on received Pause packe */
	/*************************** 0x0094 : Reserved **************************************************/ 
	GmacTxQPrtyMap0				= 0x0098U,	/* set of priority values assigned to Transmit Queue 0 to 3 */
	GmacTxQPrtyMap1				= 0x009CU,	/* set of priority values assigned to Transmit Queue 4 to 7 */
	/*************************** 0x00A0	  to   0x00A4 : Reserved ************************************/
	GmacRxQCtrl0				= 0x00A8U,	/* controls the queue management in the MAC Receiver */
	GmacRxQCtrl1				= 0x00ACU,	/* controls the queue management in the MAC Receiver */
	GmacInterruptStatus			= 0x00B0U,	/* Mac Interrupt ststus register		   */  
	GmacInterruptEnable			= 0x00B4U,	/* Mac Interrupt Mask register		   */  
	GmacRxTxStat				= 0x00B8U,	/* register contains the receive and transmitt Error status	 */ 
	/*************************** 0x00BC : Reserved **************************************************/ 
	GmacPmtCtrlStatus			= 0x00C0U,	/* Present only when you select PMT module */ 
	GmacWakeupAddr				= 0x00C4U,	/* Present only when you select PMT module Remote Wake Up*/	 
	/*************************** 0x00C8	  to   0x00CC : Reserved ************************************/
	#ifdef LPI_SUPPORT
	GmacLPICtrlSts				= 0x00D0U,	/* LPI (low power idle) Control and Status Register			 */
	GmacLPITimerCtrl			= 0x00D4U,	/* LPI timer control register				*/
	#endif
	/*************************** 0x00D8	  to   0x00DC : Reserved ************************************/
	GmacANCrtl					= 0x00E0U,	/* register enables and/or restarts auto-negotiation */
	GmacANStat					= 0x00E4U,	/* register indicates the link and auto-negotiation status */
	GmacANAdvert				= 0x00E8U,	/* register is configured before auto-negotiation begins */
	GmacANLinkPrtnr				= 0x00ECU,	/* This register contains the advertised ability of the link partner */
	GmacANExpan					= 0x00F0U,	/* Register indicates whether new base page has been received from the link partner*/
	GmacTBIExtend				= 0x00F4U,	/* Register indicates all modes of operation of the MAC */
	GmacPHYIFCtrl				= 0x00F8U,	/* Register contains the control for RGMII, SGMII and SMII PHY interface*/
	/*************************** 0x00FC	  to   0x0010C : Reserved ************************************/
	GmacVersion					= 0x00110U,	 /* GMAC Core Version Register				  */ 
	GmacDegug					= 0x00114U,	 /* register provides the debug status of various MAC blocks */
	/*************************** 0x00118 : Reserved **************************************************/ 
	GmacHW0						= 0x0011CU,	 /* optional features of the DWC_ether_qos core */
	GmacHW1						= 0x00120U,	 /* optional features of the DMA and the MTL */
	GmacHW2						= 0x00124U,	 /* number of channels selected in the DMA the number of queues selected in the MTL */		 
	/*************************** 0x0128	  to   0x001FC : Reserved ************************************/
	GmacGmiiAddr				= 0x200U,	 /* MAC_GMII_Address Register						*/
	GmacGmiiData				= 0x204U,	 /* MAC_GMII_Data Register							*/
	GmacGPIOControl				= 0x208U,	 /* MAC_GPIO_Control Register						*/
	GmacGPIOStatus				= 0x20CU,	 /* MAC_GPIO_Status Register						*/
	GmacAddr0High				= 0x300U,	 /* MAC_Address0_High Register						*/
	GmacAddr0Low				= 0x304U,	 /* MAC_Address0_Low Register						*/
	GmacAddr1High				= 0x308U,	 /* MAC_Address0_High								*/
	GmacAddr1Low				= 0x30CU,	 /* MAC_Address1_Low Register						*/
	GmacAddr32High				= 0x400U,	 /* MAC_Address32_High Register						*/
	GmacAddr32Low				= 0x404U,	 /* MAC_Address32_Low Register						*/
	
	GmacL3L4Control0			= 0x0900U,	/* MAC_L3_L4_Control0 Register					*/
	GmacLayer4Address0			= 0x0904U,	/* MAC_Layer4_Addr0 Register						*/
	GmacLayer3Addr0Reg0			= 0x0910U,	/* MAC_Layer3_Addr0_Reg0							*/
	GmacLayer3Addr1Reg0			= 0x0914U,	/* MAC_Layer3_Addr1_Reg0 Register						*/
	GmacLayer3Addr2Reg0			= 0x0918U,	/* MAC_Layer3_Addr2_Reg0 Register						*/
	GmacLayer3Addr3Reg0			= 0x091CU,	/* MAC_Layer3_Addr3_Reg0 Register						*/
	
	GmacL3L4Control1			= 0x0930U,	/* MAC_L3_L4_Control1 Register					*/
	GmacLayer4Address1			= 0x0934U,	/* MAC_Layer4_Addr1 Register						*/
	GmacLayer3Addr0Reg1			= 0x0940U,	/* MAC_Layer3_Addr0_Reg1							*/
	GmacLayer3Addr1Reg1			= 0x0944U,	/* MAC_Layer3_Addr1_Reg1 Register						*/
	GmacLayer3Addr2Reg1			= 0x0948U,	/* MAC_Layer3_Addr2_Reg1 Register						*/
	GmacLayer3Addr3Reg1			= 0x094CU,	/* MAC_Layer3_Addr3_Reg1 Register						*/
	
	GmacL3L4Control2			= 0x0960U,	/* MAC_L3_L4_Control2 Register					*/
	GmacLayer4Address2			= 0x0964U,	/* MAC_Layer4_Addr2 Register						*/
	GmacLayer3Addr0Reg2			= 0x0970U,	/* MAC_Layer3_Addr0_Reg2							*/
	GmacLayer3Addr1Reg2			= 0x0974U,	/* MAC_Layer3_Addr1_Reg2 Register						*/
	GmacLayer3Addr2Reg2			= 0x0978U,	/* MAC_Layer3_Addr2_Reg2 Register						*/
	GmacLayer3Addr3Reg2			= 0x097CU,	/* MAC_Layer3_Addr3_Reg2 Register						*/
	
	GmacL3L4Control3			= 0x0990U,	/* MAC_L3_L4_Control3 Register					*/
	GmacLayer4Address3			= 0x0994U,	/* MAC_Layer4_Addr3 Register						*/
	GmacLayer3Addr0Reg3			= 0x09A0U,	/* MAC_Layer3_Addr0_Reg3							*/
	GmacLayer3Addr1Reg3			= 0x09A4U,	/* MAC_Layer3_Addr1_Reg3 Register						*/
	GmacLayer3Addr2Reg3			= 0x09A8U,	/* MAC_Layer3_Addr2_Reg3 Register						*/
	GmacLayer3Addr3Reg3			= 0x09ACU,	/* MAC_Layer3_Addr3_Reg3 Register						*/
	
	GmacL3L4Control4			= 0x09C0U,	/* MAC_L3_L4_Control4 Register					*/
	GmacLayer4Address4			= 0x09C4U,	/* MAC_Layer4_Addr4 Register						*/
	GmacLayer3Addr0Reg4			= 0x09D0U,	/* MAC_Layer3_Addr0_Reg4							*/
	GmacLayer3Addr1Reg4			= 0x09D4U,	/* MAC_Layer3_Addr1_Reg4 Register						*/
	GmacLayer3Addr2Reg4			= 0x09D8U,	/* MAC_Layer3_Addr2_Reg4 Register						*/
	GmacLayer3Addr3Reg4			= 0x09DCU,	/* MAC_Layer3_Addr3_Reg4 Register						*/
	
	GmacL3L4Control5			= 0x09F0U,	/* MAC_L3_L4_Control5 Register					*/
	GmacLayer4Address5			= 0x09F8U,	/* MAC_Layer4_Addr5 Register						*/
	GmacLayer3Addr0Reg5			= 0x0A00U,	/* MAC_Layer3_Addr0_Reg5							*/
	GmacLayer3Addr1Reg5			= 0x0A04U,	/* MAC_Layer3_Addr1_Reg5 Register						*/
	GmacLayer3Addr2Reg5			= 0x0A08U,	/* MAC_Layer3_Addr2_Reg5 Register						*/
	GmacLayer3Addr3Reg5			= 0x0A0CU,	/* MAC_Layer3_Addr3_Reg5 Register						*/
	
	GmacL3L4Control6			= 0x0A20U,	/* MAC_L3_L4_Control6 Register					*/
	GmacLayer4Address6			= 0x0A24U,	/* MAC_Layer4_Addr6 Register						*/
	GmacLayer3Addr0Reg6			= 0x0A30U,	/* MAC_Layer3_Addr0_Reg6							*/
	GmacLayer3Addr1Reg6			= 0x0A34U,	/* MAC_Layer3_Addr1_Reg6 Register						*/
	GmacLayer3Addr2Reg6			= 0x0A38U,	/* MAC_Layer3_Addr2_Reg6 Register						*/
	GmacLayer3Addr3Reg6			= 0x0A3CU,	/* MAC_Layer3_Addr3_Reg6 Register						*/
	
	GmacL3L4Control7			= 0x0A50U,	/* MAC_L3_L4_Control7 Register					*/
	GmacLayer4Address7			= 0x0A54U,	/* MAC_Layer4_Addr7 Register						*/
	GmacLayer3Addr0Reg7			= 0x0A60U,	/* MAC_Layer3_Addr0_Reg7							*/
	GmacLayer3Addr1Reg7			= 0x0A64U,	/* MAC_Layer3_Addr1_Reg7 Register						*/
	GmacLayer3Addr2Reg7			= 0x0A68U,	/* MAC_Layer3_Addr2_Reg7 Register						*/
	GmacLayer3Addr3Reg7			= 0x0A6CU,	/* MAC_Layer3_Addr3_Reg7 Register						*/
	
	GmacARPAddress				= 0xae0U,	 /* MAC_ARP_Address Register						*/
	GmacTSControl				= 0xb00U,	 /* MAC_Timestamp_Control Register					*/
	GmacSubSecondIncrement		= 0xb04U,	 /* MAC_Sub_Second_Increment Register				*/
	GmacTSHigh					= 0xb08U,	 /* MAC_System_Time_Seconds Register				*/
	GmacTSLow					= 0xb0cU,	 /* MAC_System_Time_Nanoseconds Register			*/
	GmacSTSecUpdate				= 0xb10U,	 /* MAC-System_Time_Seconds_Update Register			*/
	GmacSTNanoSecUpdate			= 0xb14U,	 /* MAC_System_Time_Nanoseconds_Update Register		*/
	GmacTSAddend				= 0xb18U,	 /* MAC_Timestamp_Addend Register					*/
	GmacSTHigherWordSeconds		= 0xb1cU,	 /* MAC_System_Time_Higher_Word_Seconds Register	*/
	GmacTSStatus				= 0xb20U,	 /* MAC_Timestamp_Status Register					*/
	GmacTxTSStatusNanoseconds	= 0xb30U,	 /* MAC_TxTimestamp_Status_Nanoseconds				*/
	GmacTxTSStatusSeconds		= 0xb34U,	 /* MAC_TxTimestamp_Status_Seconds					*/
	GmacAuxControl				= 0xb40U,	 /* MAC_Auxiliary_Control Register					*/
	GmacAuxTSNanoseconds		= 0xb48U,	 /* MAC_Auxiliary_Timestamp_Nanoseconds Register	*/
	GmacAuxTSSeconds			= 0xb4cU,	 /* MAC_Auxiliary_Timestamp_Seconds Register		*/
	GmacTSIngressAsymCorr		= 0xb50U,	 /* MAC_Auxiliary_Timestamp_Seconds Register		*/
	GmacTSEgressAsymCorr		= 0xb54U,	 /* MAC_Auxiliary_Timestamp_Seconds Register		*/	
	GmacPPSControl				= 0xb70U,	 /* MAC_PPS_Control Register						*/
	GmacPPS0TTSeconds			= 0xb80U,	 /* MAC_PPS0_Target_Time_Seconds Register			*/
	GmacPPS0TTNanoseconds		= 0xb84U,	 /* MAC_PPS0_Target_Time_Nanoseconds Register		*/
	GmacPPS0Interval			= 0xb88U,	 /* MAC_PPS0_Interval Register						*/
	GmacPPS0Width				= 0xb8cU,	 /* MAC_PPS0_Width Register							*/
	GmacPPS1TTSeconds			= 0xb90U,	 /* MAC_PPS1_Target_Time_Seconds Register			*/
	GmacPPS1TTNanoseconds		= 0xb94U,	 /* MAC_PPS1_Target_Time_Nanoseconds Register		*/
	GmacPPS1Interval			= 0x0B98U,	/* MAC_PPS1_Interval Register							*/
	GmacPPS1Width				= 0x0B9CU,	/* MAC_PPS1_Width Register						*/
	GmacPPS2TTSeconds			= 0x0BA0U,	/* MAC_PPS2_Target_Time_Seconds Register			*/
	GmacPPS2TTNanoseconds		= 0x0BA4U,	/* MAC_PPS2_Target_Time_Nanoseconds Register		*/
	GmacPPS2Interval			= 0x0BA8U,	/* MAC_PPS2_Interval Register							*/
	GmacPPS2Width				= 0x0BACU,	/* MAC_PPS2_Width Register						*/
	GmacPPS3TTSeconds			= 0x0BB0U,	/* MAC_PPS3_Target_Time_Seconds Register			*/
	GmacPPS3TTNanoseconds		= 0x0BB4U,	/* MAC_PPS3_Target_Time_Nanoseconds Register		*/
	GmacPPS3Interval			= 0x0BB8U,	/* MAC_PPS3_Interval Register							*/
	GmacPPS3Width				= 0x0BBCU,	/* MAC_PPS3_Width Register						*/
  
};

/**********************************************************
 * GMAC DMA registers
 **********************************************************/
#if ( TC9560_USE_TSB_DMA_WRAPPER == DEF_ENABLED ) // Toshiba DMA Wrapper
enum DmaRegisters			  
{
	EVB_WCTRL					= 0x0000U,
	EVB_CTRL					= 0x0004U,
	DmaBusCfg					= 0x0100U,	 /* Bus configuation */
	DmaTxChSts00				= 0x0200U,	  /* Tx Channel Status								   */
	DmaTxChIntMsk00				= 0x0204U,	  /* Tx Channel Interrupt Mask						   */
	DmaTxChControl00			= 0x0208U,	  /* Tx Channel Control								   */
	DmaTxChDescLstHA00			= 0x020CU,	  /* DMA TX Channel Desc List High Address */
	DmaTxChDescLstLA00			= 0x0210U,	  /* DMA TX Channel Desc List Low Address */
	DmaTxChDescTailPt00			= 0x0214U,	  /* DMA TX Channel Tail Pointer */
	DmaTxChDescRngLn00			= 0x0218U,	  /* DMA TX Channel Desc Ring Length */
	DmaTxChCurDesc00			= 0x021CU,	  /* DMA TX Channel Current Desc */
	DmaTxChCurBufHA00			= 0x0220U,	  /* DMA TX Channel Curtent Buff High Address */
	DmaTxChCurBufLA00			= 0x0224U,	  /* DMA TX Channel Curtent Buff Low Address */
	DmaRxChSts00				= 0x0800U,	  /* Rx Channel Status								   */
	DmaRxChIntMsk00				= 0x0804U,	  /* Rx Channel Interrupt Mask						   */
	DmaRxChControl00			= 0x0808U,	  /* Rx Channel Control								   */
	DmaRxChDescLstHA00			= 0x080CU,	  /* DMA RX Channel Desc List High Address */
	DmaRxChDescLstLA00			= 0x0810U,	  /* DMA RX Channel Desc List Low Address */
	DmaRxChDescTailPt00			= 0x0814U,	  /* DMA RX Channel Tail Pointer */
	DmaRxChDescRngLn00			= 0x0818U,	  /* DMA RX Channel Desc Ring Length */
	DmaRxChCurDesc00			= 0x081CU,	  /* DMA RX Channel current descriptor */
	DmaRxChCurBufHA00			= 0x0820U,	  /* DMA RX Channel current buffer high address */
	DmaRxChCurBufLA00			= 0x0824U,	  /* DMA RX Channel current buffer low address */
	DmaRxChCurWatchDogTimer00	= 0x0828U, 
	DmaRxChMissCounter00		= 0x082CU, 
};
#define DMA_CHANNEL_REG_LEN		0x40

#else // Synopsys EMAC DMA
enum SynopDmaRegisters			   
{
  DmaMode						= 0x1000U,	   /* Bus configuation */
  DmaSysBusMode					= 0x1004U,	   /* DMA RX Int. WDT Time */
  DmaIntStatus					= 0x1008U,

	DmaCh0Control				= 0x1100U,
	DmaCh0TxControl				= 0x1104U,
	DmaCh0RxControl				= 0x1108U,
	
	DmaCh0TxDescListHAddr		= 0x1110U,
	DmaCh0TxDescListAddr		= 0x1114U,
	DmaCh0RxDescListHAddr		= 0x1118U,
	DmaCh0RxDescListAddr		= 0x111CU,

	DmaCh0TxDescTailPtr			= 0x1120U,
	DmaCh0RxDescTailPtr			= 0x1128U,

	DmaCh0TxDescRingLen			= 0x112CU,
	DmaCh0RxDescRingLen			= 0x1130U,

	DmaCh0IntEnable				= 0x1134U,
	DmaCh0RxIntWatchDogTmr		= 0x1138U,
	DmaCh0SlotFuncCntlStatus	= 0x113CU,

	DmaCh0CurrAppTxDes			= 0x1144U,
	DmaCh0CurrAppRxDes			= 0x114CU,
	DmaCh0CurrAppTxBufH			= 0x1150U,
	DmaCh0CurrAppTxBuf			= 0x1154U,
	DmaCh0CurrAppRxBufH			= 0x1158U,
	DmaCh0CurrAppRxBuf			= 0x115CU,

	DmaCh0Status				= 0x1160U,
	DmaCh0MissFrameCount		= 0x116CU,	
};
#define DMA_CHANNEL_REG_LEN		0x80
/* For TC9560 test: we assign DMA CH0 -->Rx Ch0, DMA CH1 --> Rx Ch1, DMA Ch2 --> Tx Ch1 */
#endif
//enum Dma_Mode
//{
//	SWR	  = 0x0000		
//};
/*------------------------------------ BKY REGISTERS ---- END---------------------------------------*/
/*SynopGMAC can support up to 32 phys*/
//#define DEFAULT_PHY_BASE PHY0			//We use First Phy 
#define DEFAULT_PHY_BASE		3		//For TC9560 FPGA!!!
#define MACBASE					0x0000	// The Mac Base address offset is 0x0000
#define DMABASE					0x1000	// Dma base address starts with an offset 0x1000
#define TRANSMIT_DESC_SIZE		12		//Tx Descriptors needed in the Descriptor pool/queue
#define RECEIVE_DESC_SIZE		12		//Rx Descriptors needed in the Descriptor pool/queue
#define ETHERNET_HEADER			14		//6 byte Dest addr, 6 byte Src addr, 2 byte length/type
#define ETHERNET_CRC			4		//Ethernet CRC
#define ETHERNET_EXTRA			2		//Only God knows about this?????
#define ETHERNET_PACKET_COPY	250		// Maximum length when received data is copied on to a new skb	
#define ETHERNET_PACKET_EXTRA	18		// Preallocated length for the rx packets is MTU + ETHERNET_PACKET_EXTRA  
#define VLAN_TAG				4		//optional 802.1q VLAN Tag
#define MIN_ETHERNET_PAYLOAD	46		//Minimum Ethernet payload size
#define MAX_ETHERNET_PAYLOAD	1500	//Maximum Ethernet payload size
//#define MAX_ETHERNET_PAYLOAD	1024	//Maximum Ethernet payload size
#define JUMBO_FRAME_PAYLOAD		9000	//Jumbo frame payload size
#define TX_BUF_SIZE				ETHERNET_HEADER + ETHERNET_CRC + MAX_ETHERNET_PAYLOAD + VLAN_TAG

// This is the IP's phy address. This is unique address for every MAC in the universe
//#define DEFAULT_MAC_ADDRESS[] = {0x00, 0x55, 0x7B, 0xB5, 0x7D, 0xF7}
#define DEFAULT_MAC_ADDRESS		{0xE8, 0xE0, 0xB7, 0xB5, 0x7D, 0xF7}

/*
DMA Descriptor Structure
The structure is common for both receive and transmit descriptors
The descriptor is of 4 words, but our structrue contains 6 words where
last two words are to hold the virtual address of the network buffer pointers
for driver's use
From the GMAC core release 3.50a onwards, the Enhanced Descriptor structure got changed.
The descriptor (both transmit and receive) are of 8 words each rather the 4 words of normal 
descriptor structure.
Whenever IEEE 1588 Timestamping is enabled TX/RX DESC6 provides the lower 32 bits of Timestamp value and
										   TX/RX DESC7 provides the upper 32 bits of Timestamp value
In addition to this whenever extended status bit is set (RX DESC0 bit 0), RX DESC4 contains the extended status information
*/

#define MODULO_INTERRUPT   1 // if it is set to 1, interrupt is available for all the descriptors or else interrupt is available only for 
				 // descriptor whose index%MODULO_INTERRUPT is zero
typedef struct DmaDescStruct	
{								
	uint32_t   status;		   /* Status									*/
	uint32_t   length;		   /* Buffer 1	and Buffer 2 length							*/
	uint32_t   bufferla;		/* Network Buffer low address pointer (Dma-able)							*/
	uint32_t   bufferha;		/* Network Buffer high address pointer (Dma-able)	*/
}DmaDesc;

typedef struct TX_NORMAL_DESC
{
	uint32_t TDES0;
	uint32_t TDES1;
	uint32_t  TDES2;
	uint32_t TDES3;
}s_tx_norm_desc;

typedef struct RX_NORMAL_DESC
{
	uint32_t RDES0;
	uint32_t RDES1;
	uint32_t RDES2;
	uint32_t  RDES3;
}s_rx_norm_desc;

/* synopGMAC device data */
/*
 * This structure contains different flags and each flags will indicates
 * different hardware features.
 */
struct DWC_ETH_QOS_hw_features {
	/* HW Feature Register0 */
	uint32_t mii_sel;	/* 10/100 Mbps support */
	uint32_t gmii_sel;	/* 1000 Mbps support */
	uint32_t hd_sel;	/* Half-duplex support */
	uint32_t pcs_sel;	/* PCS registers(TBI, SGMII or RTBI PHY interface) */
	uint32_t vlan_hash_en;	/* VLAN Hash filter selected */
	uint32_t sma_sel;	/* SMA(MDIO) Interface */
	uint32_t rwk_sel;	/* PMT remote wake-up packet */
	uint32_t mgk_sel;	/* PMT magic packet */
	uint32_t mmc_sel;	/* RMON module */
	uint32_t arp_offld_en;	/* ARP Offload features is selected */
	uint32_t ts_sel;	/* IEEE 1588-2008 Adavanced timestamp */
	uint32_t eee_sel;	/* Energy Efficient Ethernet is enabled */
	uint32_t tx_coe_sel;	/* Tx Checksum Offload is enabled */
	uint32_t rx_coe_sel;	/* Rx Checksum Offload is enabled */
	uint32_t mac_addr16_sel;	/* MAC Addresses 1-16 are selected */
	uint32_t mac_addr32_sel;	/* MAC Addresses 32-63 are selected */
	uint32_t mac_addr64_sel;	/* MAC Addresses 64-127 are selected */
	uint32_t tsstssel;	/* Timestamp System Time Source */
	uint32_t speed_sel;	/* Speed Select */
	uint32_t sa_vlan_ins;	/* Source Address or VLAN Insertion */
	uint32_t act_phy_sel;	/* Active PHY Selected */

	/* HW Feature Register1 */
	uint32_t rx_fifo_size;	/* MTL Receive FIFO Size */
	uint32_t tx_fifo_size;	/* MTL Transmit FIFO Size */
	uint32_t adv_ts_hword;	/* Advance timestamping High Word selected */
	uint32_t dcb_en;	/* DCB Feature Enable */
	uint32_t sph_en;	/* Split Header Feature Enable */
	uint32_t tso_en;	/* TCP Segmentation Offload Enable */
	uint32_t dma_debug_gen;	/* DMA debug registers are enabled */
	uint32_t av_sel;	/* AV Feature Enabled */
	uint32_t lp_mode_en;	/* Low Power Mode Enabled */
	uint32_t hash_tbl_sz;	/* Hash Table Size */
	uint32_t l3l4_filter_num;	/* Total number of L3-L4 Filters */

	/* HW Feature Register2 */
	uint32_t rx_q_cnt;	/* Number of MTL Receive Queues */
	uint32_t tx_q_cnt;	/* Number of MTL Transmit Queues */
	uint32_t rx_ch_cnt;	/* Number of DMA Receive Channels */
	uint32_t tx_ch_cnt;	/* Number of DMA Transmit Channels */
	uint32_t pps_out_num;	/* Number of PPS outputs */
	uint32_t aux_snap_num;	/* Number of Auxillary snapshot inputs */
};

/* structure to hold MMC values */
struct DWC_ETH_QOS_mmc_counters {
	/* MMC TX counters */
	ULONG mmc_tx_octetcount_gb;
	ULONG mmc_tx_framecount_gb;
	ULONG mmc_tx_broadcastframe_g;
	ULONG mmc_tx_multicastframe_g;
	ULONG mmc_tx_64_octets_gb;
	ULONG mmc_tx_65_to_127_octets_gb;
	ULONG mmc_tx_128_to_255_octets_gb;
	ULONG mmc_tx_256_to_511_octets_gb;
	ULONG mmc_tx_512_to_1023_octets_gb;
	ULONG mmc_tx_1024_to_max_octets_gb;
	ULONG mmc_tx_unicast_gb;
	ULONG mmc_tx_multicast_gb;
	ULONG mmc_tx_broadcast_gb;
	ULONG mmc_tx_underflow_error;
	ULONG mmc_tx_singlecol_g;
	ULONG mmc_tx_multicol_g;
	ULONG mmc_tx_deferred;
	ULONG mmc_tx_latecol;
	ULONG mmc_tx_exesscol;
	ULONG mmc_tx_carrier_error;
	ULONG mmc_tx_octetcount_g;
	ULONG mmc_tx_framecount_g;
	ULONG mmc_tx_excessdef;
	ULONG mmc_tx_pause_frame;
	ULONG mmc_tx_vlan_frame_g;
	ULONG mmc_tx_osize_frame_g;
	/* MMC RX counters */
	ULONG mmc_rx_framecount_gb;
	ULONG mmc_rx_octetcount_gb;
	ULONG mmc_rx_octetcount_g;
	ULONG mmc_rx_broadcastframe_g;
	ULONG mmc_rx_multicastframe_g;
	ULONG mmc_rx_crc_errror;
	ULONG mmc_rx_align_error;
	ULONG mmc_rx_run_error;
	ULONG mmc_rx_jabber_error;
	ULONG mmc_rx_undersize_g;
	ULONG mmc_rx_oversize_g;
	ULONG mmc_rx_64_octets_gb;
	ULONG mmc_rx_65_to_127_octets_gb;
	ULONG mmc_rx_128_to_255_octets_gb;
	ULONG mmc_rx_256_to_511_octets_gb;
	ULONG mmc_rx_512_to_1023_octets_gb;
	ULONG mmc_rx_1024_to_max_octets_gb;
	ULONG mmc_rx_unicast_g;
	ULONG mmc_rx_length_error;
	ULONG mmc_rx_outofrangetype;
	ULONG mmc_rx_pause_frames;
	ULONG mmc_rx_fifo_overflow;
	ULONG mmc_rx_vlan_frames_gb;
	ULONG mmc_rx_watchdog_error;
	ULONG mmc_rx_receive_error;
	ULONG mmc_rx_ctrl_frames_g;
	/* IPC */
	ULONG mmc_rx_ipc_intr_mask;
	ULONG mmc_rx_ipc_intr;
	/* IPv4 */
	ULONG mmc_rx_ipv4_gd;
	ULONG mmc_rx_ipv4_hderr;
	ULONG mmc_rx_ipv4_nopay;
	ULONG mmc_rx_ipv4_frag;
	ULONG mmc_rx_ipv4_udsbl;
	/* IPV6 */
	ULONG mmc_rx_ipv6_gd_octets;
	ULONG mmc_rx_ipv6_hderr_octets;
	ULONG mmc_rx_ipv6_nopay_octets;
	/* Protocols */
	ULONG mmc_rx_udp_gd;
	ULONG mmc_rx_udp_err;
	ULONG mmc_rx_tcp_gd;
	ULONG mmc_rx_tcp_err;
	ULONG mmc_rx_icmp_gd;
	ULONG mmc_rx_icmp_err;
	/* IPv4 */
	ULONG mmc_rx_ipv4_gd_octets;
	ULONG mmc_rx_ipv4_hderr_octets;
	ULONG mmc_rx_ipv4_nopay_octets;
	ULONG mmc_rx_ipv4_frag_octets;
	ULONG mmc_rx_ipv4_udsbl_octets;
	/* IPV6 */
	ULONG mmc_rx_ipv6_gd;
	ULONG mmc_rx_ipv6_hderr;
	ULONG mmc_rx_ipv6_nopay;
	/* Protocols */
	ULONG mmc_rx_udp_gd_octets;
	ULONG mmc_rx_udp_err_octets;
	ULONG mmc_rx_tcp_gd_octets;
	ULONG mmc_rx_tcp_err_octets;
	ULONG mmc_rx_icmp_gd_octets;
	ULONG mmc_rx_icmp_err_octets;
};

typedef struct synopGMACDeviceStruct	  
{
	uint32_t MacBase;							/* base address of MAC registers		   */
	uint32_t DmaBase;							/* base address of DMA registers		   */
	uint32_t PhyBase;							/* PHY device address on MII interface	   */
	uint32_t Version;							/* Gmac Revision version					*/		
	s_tx_norm_desc * TxDesc[TX_NO_CHAN];		/* start address of TX descriptors ring or chain, this is used by the driver	*/
	s_rx_norm_desc * RxDesc[RX_NO_CHAN];		/* start address of RX descriptors ring or chain, this is used by the driver	*/
	uint32_t BusyTxDescNo[TX_NO_CHAN];			/* Number of Tx Descriptors owned by DMA at any given time*/  
	uint32_t BusyRxDescNo[RX_NO_CHAN];			/* Number of Rx Descriptors owned by DMA at any given time*/  
	uint32_t  RxDescCount[RX_NO_CHAN];			/* number of rx descriptors in the tx descriptor queue/pool */
	uint32_t  TxDescCount[TX_NO_CHAN];			/* number of tx descriptors in the rx descriptor queue/pool */
	uint32_t  TxBusy[TX_NO_CHAN];				/* index of the tx descriptor owned by DMA, is obtained by synopGMAC_get_tx_qptr()				  */
	uint32_t  TxNext[TX_NO_CHAN];				/* index of the tx descriptor next available with driver, given to DMA by synopGMAC_set_tx_qptr() */
	uint32_t  RxBusy[RX_NO_CHAN];				/* index of the rx descriptor owned by DMA, obtained by synopGMAC_get_rx_qptr()					  */
	uint32_t  RxNext[RX_NO_CHAN];				/* index of the rx descriptor next available with driver, given to DMA by synopGMAC_set_rx_qptr() */
	s_tx_norm_desc * TxBusyDesc[TX_NO_CHAN];	/* Tx Descriptor address corresponding to the index TxBusy */
	s_tx_norm_desc * TxNextDesc[TX_NO_CHAN];	/* Tx Descriptor address corresponding to the index TxNext */
	s_rx_norm_desc * RxBusyDesc[RX_NO_CHAN];	/* Rx Descriptor address corresponding to the index TxBusy */
	s_rx_norm_desc * RxNextDesc[RX_NO_CHAN];	/* Rx Descriptor address corresponding to the index RxNext */
	UCHAR	   RxDescPool[RX_NO_CHAN];
	UCHAR	   TxDescPool[TX_NO_CHAN];			   
	/*Phy related stuff*/
	uint32_t ClockDivMdc;						/* Clock divider value programmed in the hardware		   */
	/* The status of the link */
	uint32_t LinkState;							/* Link status as reported by the Marvel Phy				 */
	uint32_t DuplexMode;						/* Duplex mode of the Phy					*/
	uint32_t Speed;								/* Speed of the Phy						*/
	uint32_t LoopBackMode;						/* Loopback status of the Phy					*/
	UCHAR txchno;
	UCHAR rxchno;
	uint32_t  TDES0Val[TX_NO_CHAN][TX_DESC_CNT];  /* TX Buffer address for each Desc*/
	uint32_t  RDES0Val[RX_NO_CHAN][RX_DESC_CNT];
	void * USBREQ[TX_NO_CHAN][TX_DESC_CNT];	
	struct DWC_ETH_QOS_hw_features hw_feat;
	struct DWC_ETH_QOS_mmc_counters mmc;
	/* for filtering */
	int32_t max_hash_table_size;
	int32_t max_addr_reg_cnt;
	/* L3/L4 filtering */
	uint32_t l3_l4_filter;
	UCHAR vlan_hash_filtering;
	uint32_t l2_filtering_mode; /* 0 - if perfect and 1 - if hash filtering */
} synopGMACdevice;

enum DescMode
{
	RINGMODE	= 0x00000001U,
	CHAINMODE	= 0x00000002U,
};

enum BufferMode
{
	SINGLEBUF	= 0x00000001U,
	DUALBUF		= 0x00000002U,
};

/* Below is "88E1011/88E1011S Integrated 10/100/1000 Gigabit Ethernet Transceiver" 
 * Register and their layouts. This Phy has been used in the Dot Aster GMAC Phy daughter.
 * Since the Phy register map is standard, this map hardly changes to a different Ppy
 */

enum MiiRegisters
{
  PHY_CONTROL_REG					= 0x0000U,		/*Control Register*/
  PHY_STATUS_REG					= 0x0001U,		/*Status Register */
  PHY_ID_HI_REG						= 0x0002U,		/*PHY Identifier High Register*/
  PHY_ID_LOW_REG					= 0x0003U,		/*PHY Identifier High Register*/
  PHY_AN_ADV_REG					= 0x0004U,		/*Auto-Negotiation Advertisement Register*/
  PHY_LNK_PART_ABl_REG				= 0x0005U,		/*Link Partner Ability Register (Base Page)*/
  PHY_AN_EXP_REG					= 0x0006U,		/*Auto-Negotiation Expansion Register*/
  PHY_AN_NXT_PAGE_TX_REG			= 0x0007U,		/*Next Page Transmit Register*/
  PHY_LNK_PART_NXT_PAGE_REG			= 0x0008U,		/*Link Partner Next Page Register*/
  PHY_1000BT_CTRL_REG				= 0x0009U,		/*1000BASE-T Control Register*/
  PHY_1000BT_STATUS_REG				= 0x000aU,		/*1000BASE-T Status Register*/
  PHY_SPECIFIC_CTRL_REG				= 0x0010U,		/*Phy specific control register*/
  PHY_SPECIFIC_STATUS_REG			= 0x0011U,		/*Phy specific status register*/
  PHY_INTERRUPT_ENABLE_REG			= 0x0012U,		/*Phy interrupt enable register*/
  PHY_INTERRUPT_STATUS_REG			= 0x0013U,		/*Phy interrupt status register*/
  PHY_EXT_PHY_SPC_CTRL				= 0x0014U,		/*Extended Phy specific control*/
  PHY_RX_ERR_COUNTER				= 0x0015U,		/*Receive Error Counter*/
  PHY_EXT_ADDR_CBL_DIAG				= 0x0016U,		/*Extended address for cable diagnostic register*/
  PHY_LED_CONTROL					= 0x0018U,		/*LED Control*/			
  PHY_MAN_LED_OVERIDE				= 0x0019U,		/*Manual LED override register*/
  PHY_EXT_PHY_SPC_CTRL2				= 0x001aU,		/*Extended Phy specific control 2*/
  PHY_EXT_PHY_SPC_STATUS			= 0x001bU,		/*Extended Phy specific status*/
  PHY_CBL_DIAG_REG					= 0x001cU,		/*Cable diagnostic registers*/
};

/* This is Control register layout. Control register is of 16 bit wide.
*/

#if 1 //TI PHY
/* This is Control register layout. Control register is of 16 bit wide.
 * */

enum Mii_GEN_CTRL	//BMCR
{							  /*	Description				   bits		   R/W	default value  */
	Mii_reset						= 0x8000U, 
	Mii_Speed_10					= 0x0000U,	  /* 10	  Mbps					  6:13(0,0)				RW						*/
	Mii_Speed_100					= 0x2000U,	  /* 100  Mbps					  6:13(0,1)				RW						*/
	Mii_Speed_1000					= 0x0040U,	  /* 1000 Mbit/s				  6:13(1,0)				RW						*/
	Mii_Duplex						= 0x0100U,	  /* Full Duplex mode			  8					RW						*/
	Mii_En_Master_Slave_Config		= 0x1000U,	  /* En Master Slave Config		 12					 RW						 */	 
	Mii_En_AutoNeg					= 0x1000U,	  /* En Auto-negotiation 12				   RW					   */  
	Mii_Restart_AutoNeg				= 0x0200U,	  /* Restart Auto-negotiation 9					RW						*/	
	Mii_Manual_Master_Config		= 0x0800U,	  /* Manual Master Config		  11				RW			*/
	Mii_Master_Mode					= 0x0001U,	  /* PHY is in Master mode		  0					 RO						 */
	Mii_Slave_Mode					= 0x0000U,	  /* PHY is in Slave mode		  0					 R0						 */ 
	Mii_Loopback					= 0x4000U,	  /* Enable Loop back			  14				RW						*/
	Mii_NoLoopback					= 0x0000U,	  /* Enable Loop back			  14				RW						*/
};

enum Mii_Phy_Status
{
	Mii_phy_status_speed_10			= 0x0000U,
	Mii_phy_status_speed_100		= 0x0008U,
	Mii_phy_status_speed_1000		= 0x0010U,
	Mii_phy_status_full_duplex		= 0x0002U,
	Mii_phy_status_half_duplex		= 0x0000U,
	Mii_phy_status_link_up			= 0x0004U,
};
/* This is Status register layout. Status register is of 16 bit wide.
 * */
enum Mii_GEN_STATUS		//BMSR)
{
	Mii_AutoNegCmplt				= 0x0020U,	  /* Autonegotiation completed		5			   RW					*/
	Mii_Link						= 0x0004U,	  /* Link status					2			   RW					*/
};

#else
enum Mii_GEN_CTRL
{							  /*	Description				   bits		   R/W	default value  */
	Mii_reset						= 0x8000U, 
	Mii_Speed_10					= 0x0000U,	  /* 10	  Mbps					  6:13			RW						*/
	Mii_Speed_100					= 0x2000U,	  /* 100  Mbps					  6:13			RW						*/
	Mii_Speed_1000					= 0x0040U,	  /* 1000 Mbit/s				  6:13			RW						*/

	Mii_Duplex						= 0x0100U,	  /* Full Duplex mode			  8					RW						*/
	Mii_En_Master_Slave_Config		= 0x1000U,	
	Mii_Manual_Master_Config		= 0x0800U,	  /* Manual Master Config		  11				RW			*/
	Mii_Master_Mode					= 0x0001U,
	Mii_Slave_Mode					= 0x0000U,

	Mii_Loopback					= 0x4000U,	  /* Enable Loop back			  14				RW						*/
	Mii_NoLoopback					= 0x0000U,	  /* Enable Loop back			  14				RW						*/
};

enum Mii_Phy_Status
{
	Mii_phy_status_speed_10			= 0x0000U,
	Mii_phy_status_speed_100		= 0x4000U,
	Mii_phy_status_speed_1000		= 0x8000U,

	Mii_phy_status_full_duplex		= 0x2000U,
	Mii_phy_status_half_duplex		= 0x0000U,

	Mii_phy_status_link_up			= 0x0400U,
};
/* This is Status register layout. Status register is of 16 bit wide.
*/
enum Mii_GEN_STATUS
{
	Mii_AutoNegCmplt				= 0x0020U,	  /* Autonegotiation completed		5			   RW					*/
	Mii_Link						= 0x0004U,	  /* Link status					2			   RW					*/
};

#endif

enum Mii_Link_Status
{
	LINKDOWN	= 0,
	LINKUP		= 1,
};

enum Mii_Duplex_Mode
{
	HALFDUPLEX = 1,
	FULLDUPLEX = 2,
};
enum Mii_Link_Speed
{
	SPEED10		= 1,
	SPEED100	= 2,
	SPEED1000	= 3,
};

enum Mii_Loop_Back
{
	NOLOOPBACK	= 0,
	LOOPBACK	= 1,
};

/**********************************************************
 * GMAC Network interface registers
 * This explains the Register's Layout
 
 * FES is Read only by default and is enabled only when Tx 
 * Config Parameter is enabled for RGMII/SGMII interface
 * during CoreKit Config.
 
 * DM is Read only with value 1'b1 in Full duplex only Config
 **********************************************************/

/* GmacConfig			   = 0x0000,	Mac config Register	 Layout */
enum GmacConfigReg		
{ 
											 /* Bit description						 Bits		R/W		Resetvalue	 */

  GmacARPOffload					= (int32_t)0x80000000U,	   /* ARP offload Enable					31		   RW		   0	   */	

  GmacSARC							= 0x06000000,	 /*Source Address Insertion				30:28		 RW			 0		 */
											 /* or Replacement Control												 */

  GmacRxIpcOffload					= 0x00000400,	 /* IPC checksum offload				  27		 RW			 0		 */
  
  GmacInterFrameGap7				= 0x07000000,	 /* (IPG) Config7 - 40 bit times		 26:24		 RW					 */
  GmacInterFrameGap6				= 0x06000000,		/* (IPG) Config6 - 48 bit times											*/
  GmacInterFrameGap5				= 0x05000000,		/* (IPG) Config5 - 56 bit times											*/
  GmacInterFrameGap4				= 0x04000000,		/* (IPG) Config4 - 64 bit times											*/
  GmacInterFrameGap3				= 0x03000000,		/* (IPG) Config3 - 72 bit times											*/
  GmacInterFrameGap2				= 0x02000000,		/* (IPG) Config2 - 80 bit times											*/
  GmacInterFrameGap1				= 0x01000000,		/* (IPG) Config1 - 88 bit times											*/
  GmacInterFrameGap0				= 0x00000000,		/* (IPG) Config0 - 96 bit times									000		*/

  GmacGPSLCE						= 0x00800000,	 /* Giant Packet Size Limit Control		  23		 RW			 0		 */
  GmacS2KP							= 0x00400000,	 /* IEEE 802.3as Support for 2K Packets	  22		 RW			 0		 */
  GmacCRCstrip						= 0x00200000,	 /* CRC stripping						  21		 RW					 */


  GmacPadCrcStrip					= 0x00100000,
  GmacPadCrcStripEnable				= 0x00100000,	 /* (ACS) Automatic Pad/Crc strip enable  20		   RW				 */
  GmacPadCrcStripDisable			= 0x00000000,	 /* Automatic Pad/Crc stripping disable							 0		 */

  GmacWatchdog						= 0x00080000,
  GmacWatchdogDisable				= 0x00080000,	 /* (WD)Disable watchdog timer on Rx	  19		   RW				 */
  GmacWatchdogEnable				= 0x00000000,	 /* Enable watchdog timer										 0		 */

  GmacFrameBurst					= 0x00040000,
  GmacFrameBurstEnable				= 0x00040000,	 /* (BE)Enable frame bursting during Tx	  18		   RW				 */
  GmacFrameBurstDisable				= 0x00000000,	 /* Disable frame bursting										 0		 */
  
  GmacJabber						= 0x00020000,
  GmacJabberDisable					= 0x00020000,	 /* (JD)Disable jabber timer on Tx		  17		   RW				 */
  GmacJabberEnable					= 0x00000000,	 /* Enable jabber timer											 0		 */
 
  GmacJumboFrame					= 0x00010000,
  GmacJumboFrameEnable				= 0x00010000,	 /* (JE)Enable jumbo frame for Tx		  16		   RW				 */
  GmacJumboFrameDisable				= 0x00000000,	 /* Disable jumbo frame											 0		 */

  GmacMiiGmii						= 0x00008000,
  GmacSelectMii						= 0x00008000,	 /* (PS)Port Select-MII mode			  15		   RW				 */
  GmacSelectGmii					= 0x00000000,	 /* GMII mode													 0		 */

  GmacFESpeed100					= 0x00004000,	 /*(FES)Fast Ethernet speed 100Mbps		  14		   RW				 */ 
  GmacFESpeed10						= 0x00000000,	 /* 10Mbps														 0		 */ 

  
  GmacDuplex						= 0x00002000,
  GmacFullDuplex					= 0x00002000,	 /* (DM)Full duplex mode				  13		   RW				 */
  GmacHalfDuplex					= 0x00000000,	 /* Half duplex mode											 0		 */
 
  GmacLoopback						= 0x00001000,
  GmacLoopbackOn					= 0x00001000,	 /* (LM)Loopback mode for GMII/MII		  12		   RW				 */
  GmacLoopbackOff					= 0x00000000,	 /* Normal mode													 0		 */


  GmacEnableCrs						= 0x00000800,	 /* Enable Carrier Sense During			  11	   RW		 0		 */	
											 /* Before Transmission													 */ 
  GmacRxOwn							= 0x00000400, 
  GmacDisableRxOwn					= 0x00000400,	 /* (DO)Disable receive own packets		  10		   RW				 */
  GmacEnableRxOwn					= 0x00000000,	 /* Enable receive own packets									 0		 */

  GmacDisableCrs					= 0x00000200,	 /* Disable Carrier Sense During			   9	   RW		 0		 */	
											 /* Transmission														  */
  GmacRetry							= 0x00000100,
  GmacRetryDisable					= 0x00000100,	 /* (DR)Disable Retry					   8		   RW				 */
  GmacRetryEnable					= 0x00000000,	 /* Enable retransmission as per BL								 0		 */

//	GmacLinkUp						= 0x00000100,	   /* (LUD)Link UP							8			RW				  */ 
//	GmacLinkDown					= 0x00000100,	   /* Link Down													   0	   */ 
  
  
  GmacBackoffLimit					= 0x00000060U,
  GmacBackoffLimit3					= 0x00000060U,	  /* (BL)Back-off limit in HD mode			6:5			RW				  */
  GmacBackoffLimit2					= 0x00000040U,	  /*																	  */
  GmacBackoffLimit1					= 0x00000020U,	  /*																	  */
  GmacBackoffLimit0					= 0x00000000U,	  /*															  00	  */

  GmacDeferralCheck					= 0x00000010U,
  GmacDeferralCheckEnable			= 0x00000010U,	  /* (DC)Deferral check enable in HD mode	4			RW				  */
  GmacDeferralCheckDisable			= 0x00000000U,	  /* Deferral check disable										  0		  */
  
  GmacPreambleLength7byte			= 0x00000000U,	  /* MAC is operating in the full-duplex	3:2			RW		  00	   */
  GmacPreambleLength5byte			= 0x00000004U,	  /*															 01		  */
  GmacPreambleLength3byte			= 0x00000008U,	  /*																 10		  */
  GmacPreambleLengthRes				= 0x0000000CU,	  /*																 11		  */
 
  GmacTx							= 0x00000002U,
  GmacTxEnable						= 0x00000002U,	  /* (TE)Transmitter enable					1			RW				  */
  GmacTxDisable						= 0x00000000U,	  /* Transmitter disable										  0		  */

  GmacRx							= 0x00000001U,
  GmacRxEnable						= 0x00000001U,	  /* (RE)Receiver enable					0			RW				  */
  GmacRxDisable						= 0x00000000U,	  /* Receiver disable											  0		  */
};
/* GmacExtConfig		   = 0x0004,	Mac extended operaion Register		 */
enum GmacExtConfigReg
{
	GmacHDSMS						= 0x00700000U,	  /* Maximum Size for Splitting the Header Data		  22:20	 RW		 000			   */
	GmacUSP							= 0x00040000U,	  /* Unicast Slow Protocol Packet Detect			  18	 RW		 0				   */ 
	GmacSPEN						= 0x00020000U,	  /* Slow Protocol Detection Enable					  17	 RW		 0				   */
	GmacDCRCC						= 0x00010000U,	  /* Disable CRC Checking for Received Packets		  16	 RW		 0				   */
	GmacGPSL						= 0x00007FFFU,	  /*Giant Packet Size Limit							 14:0	 RW		 0000			   */

}; 
/* GmacFrameFilter	  = 0x0008,		Mac frame filtering controls Register Layout*/
enum GmacFrameFilterReg 
{
  GmacFilter						= (int32_t)0x80000000,
  GmacFilterOff						= (int32_t)0x80000000,	  /* (RA)Receive all incoming packets		31		   RW				  */
  GmacFilterOn						= 0x00000000U,	  /* Receive filtered packets only								  0		  */

  GmacDNTU							= 0x00200000U,	  /* Drop Non-TCP/UDP over IP Packets		21		   RW		  0		  */

  GmacIPFE							= 0x00100000U,	  /* Layer 3 and Layer 4 Filter Enable		20		   RW		  0		  */ 
  
  GmacVTFE							= 0x00010000U,	  /* VLAN Tag Filter Enable					16		   RW		  0		  */ 

  GmacHashPerfectFilter				= 0x00000400U,	  /*Hash or Perfect Filter enable			10		   RW		  0		  */

  GmacSrcAddrFilter					= 0x00000200U,
  GmacSrcAddrFilterEnable			= 0x00000200U,	  /* (SAF)Source Address Filter enable		 9		   RW				  */
  GmacSrcAddrFilterDisable			= 0x00000000U,	  /*															  0		  */

  GmacSrcInvaAddrFilter				= 0x00000100U,
  GmacSrcInvAddrFilterEn			= 0x00000100U,	  /* (SAIF)Inv Src Addr Filter enable		 8		   RW				  */
  GmacSrcInvAddrFilterDis			= 0x00000000U,	  /*															  0		  */

  GmacPassControl					= 0x000000C0U,
  GmacPassControl3					= 0x000000C0U,	  /* (PCS)Forwards ctrl frms that pass AF	 7:6	   RW				  */
  GmacPassControl2					= 0x00000080U,	  /* Forwards all control frames										  */
  GmacPassControl1					= 0x00000040U,	  /* Does not pass control frames										  */
  GmacPassControl0					= 0x00000000U,	  /* Does not pass control frames								  00	  */

  GmacBroadcast						= 0x00000020U,
  GmacBroadcastDisable				= 0x00000020U,	  /* (DBF)Disable Rx of broadcast frames	 5		   RW				  */
  GmacBroadcastEnable				= 0x00000000U,	  /* Enable broadcast frames									  0		  */

  GmacMulticastFilter				= 0x00000010U,
  GmacMulticastFilterOff			= 0x00000010U,	  /* (PM) Pass all multicast packets		 4		   RW				  */
  GmacMulticastFilterOn				= 0x00000000U,	  /* Pass filtered multicast packets							  0		  */

  GmacDestAddrFilter				= 0x00000008U,
  GmacDestAddrFilterInv				= 0x00000008U,	  /* (DAIF)Inverse filtering for DA			 3		   RW				  */
  GmacDestAddrFilterNor				= 0x00000000U,	  /* Normal filtering for DA									  0		  */

  GmacMcastHashFilter				= 0x00000004U,
  GmacMcastHashFilterOn				= 0x00000004U,	  /* (HMC)perfom multicast hash filtering	 2		   RW				  */
  GmacMcastHashFilterOff			= 0x00000000U,	  /* perfect filtering only										  0		  */

  GmacUcastHashFilter				= 0x00000002U,
  GmacUcastHashFilterOn				= 0x00000002U,	  /* (HUC)Unicast Hash filtering only		 1		   RW				  */
  GmacUcastHashFilterOff			= 0x00000000U,	  /* perfect filtering only										  0		  */

  GmacPromiscuousMode				= 0x00000001U,
  GmacPromiscuousModeOn				= 0x00000001U,	  /* Receive all frames						 0		   RW				  */
  GmacPromiscuousModeOff			= 0x00000000U,	  /* Receive filtered packets only								  0		  */
};

//	GmacWdTimeout		= (0x000C),	   /* Controls the watchdog timeout for received packets.*/	  
enum GmacWdTimeout
{
	GmacWatchdogTimeout				= 0x0000000FU,	  /* Watchdog Timeout								  3:0	 RW		 000   */
	GmacPWE							= 0x00000100U,	  /* Programmable Watchdog Enable					   8	 RW		  0	   */
};
//	  GmacVlan						= (0x0050),	 /* VLAN tag Register (IEEE 802.1Q)			  */
enum GmacVlan
{
	GmacEIVLRXS						= (int32_t)0x80000000,	  /* Enable Inner VLAN Tag in Rx Status				   31	 RW		 0				   */	 
	GmacEIVLS						= 0x30000000U,	  /* Enable Inner VLAN Tag Stripping on Receive		  29:28	 RW		 0				   */  
	GmacERIVLT						= 0x08000000U,	  /* Enable Inner VLAN Tag							   27	 RW		 0				   */  
	GmacEDVLP						= 0x04000000U,	  /* Enable Double VLAN Processing					   26	 RW		 0				   */  
	GmacVTHM						= 0x02000000U,	  /* VLAN Tag Hash Table Match Enable				   25	 RW		 0				   */  
	GmacEVLRXS						= 0x01000000U,	  /* Enable VLAN Tag in Rx status					   24	 RW		 0				   */	   
	GmacEVLS						= 0x00600000U,	  /* Enable VLAN Tag Stripping on Receive			  22:21	 RW		 0				   */  
	GmacDOVLTC						= 0x00100000U,	  /* Disable VLAN Type check						   20	 RW		 0				   */  
	GmacERSVLM						= 0x00080000U,	  /* Enable Receive S VLAN Match					   19	 RW		 0				   */
	GmacESVL						= 0x00040000U,	  /* Enable S-VLAN									   18	 RW		 0				   */	 
	GmacVTIM						= 0x00020000U,	  /* VLAN Tag Inverse Match Enable					   17	 RW		 0				   */
	GmacETV							= 0x00010000U,	  /* Enable 12-Bit VLAN Tag Comparison				   16	 RW		 0				   */
	GmacVL							= 0x0000FFFFU,	  /*VLAN Tag Identifier for Receive Packets			  15:0	 RW		 0000			   */
};
//	GmacVlanHashTable	  = (0x0058),	 /* register contains the VLAN hash table.	  */
enum GmacVlanHashTable
{
	GmacVLHT						= 0x0000FFFFU,	  /* VLAN Hash Table								  15:0	  RW	 0000			   */  
	

};
//GmacVlanIncl		  = (0x0060),	 /* VLAN tag for insertion or replacement	  */  
//GmacInnVlanIncl	  = (0x0064),	 /* Inner VLAN tag for insertion or replacement		*/	 
enum GmacVlanIncl
{	 


	GmacVLTI						= 0x00000000U,	  /* VLAN Tag Input									   20	 RW		 0				   */  
	GmacCSVL						= 0x00000000U,	  /* C-VLAN or S-VLAN								   19	 RW		 0				   */  
	GmacVLP							= 0x00000000U,	  /* VLAN Priority Control							   18	 RW		 0				   */  
	GmacVLC							= 0x00000000U,	  /* VLAN Tag Control in Transmit Packets			  17:16	  RW	 0				   */  
	GmacVLT							= 0x00000000U,	  /* VLAN Tag for Transmit Packets					  15:0	  RW	 0				   */  
};


//	GmacQ0TxCtrl		= (0x0070),	   /*register control packets in Queue 0.*/	   
enum GmacQ0TxCtrl
{
	GmacPT							= (int32_t)0xFFFF0000,	 /* Pause Time										 31:16	 RW		 0000			  */
	GmacPSRQ						= 0x0000FF00U,	 /* Priorities Selected in the Receive Queue		 15:8	 RW		 00				  */
	GmacDZPQ						= 0x00000080U,	 /* Disable Zero-Quanta Pause						 7		 RW		 0				  */
	GmacPLT							= 0x00000070U,	 /* Pause Low Threshold								 6:4	 RW		 00				  */
	GmacTFE							= 0x00000002U,	 /* Transmit Flow control Enable					 1		 RW		 0				  */
	GmacFCB_BPA						= 0x00000001U,	 /* Flow Control Busy or Backpressure Activate		 0		 RW		 0				  */

};
//********				GmacQ1TxCtrl to GmacQ7TxCtrl  are similar to GmacQ0TxCtrl					  **********//

//GmacRxCtrl		  = (0x0090),	 /* MAC Transmit based on received Pause packe */
enum GmacRxCtrl
{
	GmacPFCE						= 0x00010000U,	 /* Priority Based Flow Control Enable				 8		 RW		 0				  */
	GmacUP							= 0x00000002U,	 /* Unicast Pause Packet Detect						 1		 RW		 0				  */
	GmacRFE							= 0x00000001U,	 /* Disable Zero-Quanta Pause						 0		 RW		 0				  */

};
//GmacTxQPrtyMap0	  = (0x0098),	 /* set of priority values assigned to Transmit Queue 0 to 3 */
enum GmacTxQPrtyMap0
{
	GmacPSTQ3						= (int32_t)0xFF000000,	 /* Priorities Selected in Transmit Queue 3			 31:24	 RW		 0000			  */
	GmacPSTQ2						= 0x00FF0000U,	 /* Priorities Selected in Transmit Queue 2			 23:16	 RW		 00				  */
	GmacPSTQ1						= 0x0000FF00U,	 /* Priorities Selected in Transmit Queue 1			 15:8	 RW		 0				  */
	GmacPSTQ0						= 0x000000FFU,	 /* Priorities Selected in Transmit Queue 0			 7:0	 RW		 00				  */
};

//GmacTxQPrtyMap1	  = (0x009C),	 /* set of priority values assigned to Transmit Queue 4 to 7 */
enum GmacTxQPrtyMap1
{
	GmacPSTQ7						= (int32_t)0xFF000000,	 /* Priorities Selected in Transmit Queue 7			 31:24	 RW		 0000			  */
	GmacPSTQ6						= 0x00FF0000U,	 /* Priorities Selected in Transmit Queue 6			 23:16	 RW		 00				  */
	GmacPSTQ5						= 0x0000FF00U,	 /* Priorities Selected in Transmit Queue 5			 15:8	 RW		 0				  */
	GmacPSTQ4						= 0x000000FFU,	 /* Priorities Selected in Transmit Queue 4			 7:0	 RW		 00				  */
};

//GmacRxQCtrl0		  = (0x00A8),	 /* controls the queue management in the MAC Receiver */
enum GmacRxQCtrl0
{
	GmacRXQ7EN						= 0x0000C000U,	 /* Receive Queue 7 Enable							 15:14	 RW		 00				  */
	GmacRXQ6EN						= 0x00003000U,	 /* Receive Queue 6 Enable							 13:12	 RW		 00				  */
	GmacRXQ5EN						= 0x00000C00U,	 /* Receive Queue 5 Enable							 11:10	 RW		 00				  */
	GmacRXQ4EN						= 0x00000300U,	 /* Receive Queue 4 Enable							 9:8	 RW		 00				  */
	GmacRXQ3EN						= 0x000000C0U,	 /* Receive Queue 3 Enable							 7:6	 RW		 00				  */
	GmacRXQ2EN						= 0x00000030U,	 /* Receive Queue 2 Enable							 5:4	 RW		 00				  */
	GmacRXQ1EN						= 0x0000000CU,	 /* Receive Queue 1 Enable							 3:2	 RW		 00				  */
	GmacRXQ0EN						= 0x00000003U,	 /* Receive Queue 0 Enable							 1:0	 RW		 00				  */	
};
//GmacRxQCtrl1		  = (0x00AC),	 /* controls the queue management in the MAC Receiver */
enum GmacRxQCtrl1
{
	GmacDCBCPQ						= 0x00000700U,	 /* DCB Control Packets Queue						 10:8	 RW		 000			  */
	GmacAVPTPQ						= 0x00000070U,	 /* AV PTP Packets Queue							 6:4	 RW		 000			  */
	GmacAVCPQ						= 0x00000007U,	 /* AV Untagged Control Packets Queue				 2:0	 RW		 000			  */
};
//GmacInterruptStatus = (0x00B0),	 /* Mac Interrupt ststus register		   */  
enum GmacInterruptStatus
{
	GmacGPIIS						= 0x00008000U,	 /* GPI Interrupt Status							 15		 RW		 0				  */
	GmacRXSTSIS						= 0x00004000U,	 /* Receive Status Interrupt						 14		 RW		 0				  */
	GmacTXSTSIS						= 0x00002000U,	 /* Transmit Status Interrupt						 13		 RW		 0				  */
	GmacTSIS						= 0x00001000U,	 /* Timestamp Interrupt Status						 12		 RW		 0				  */
	GmacMMCRXIPIS					= 0x00000800U,	 /* MMC Receive Checksum Offload Interrupt Status	 11		 RW		 0				  */
	GmacMMCTXIS						= 0x00000400U,	 /* MMC Transmit Interrupt Status					 10		 RW		 0				  */
	GmacMMCRXIS						= 0x00000200U,	 /* MMC Receive Interrupt Status					 9		 RW		 0				  */
	GmacMMCIS						= 0x00000100U,	 /* MMC Interrupt Status							 8		 RW		 0				  */	
	GmacLPIIS						= 0x00000020U,	 /* LPI Interrupt Status							 5		 RW		 0				  */
	GmacPMTIS						= 0x00000010U,	 /* PMT Interrupt Status							 4		 RW		 0				  */
	GmacPHYIS						= 0x00000008U,	 /* PHY Interrupt									 3		 RW		 0				  */
	GmacPCSANCIS					= 0x00000004U,	 /* PCS Auto-Negotiation Complete					 2		 RW		 0				  */
	GmacPCSLCHGIS					= 0x00000002U,	 /* PCS Link Status Changed							 1		 RW		 0				  */
	GmacRGSMIIIS					= 0x00000001U,	 /* RGMII or SMII Interrupt Status					 0		 RW		 0				  */

};

//GmacInterruptMask	  = (0x00B4),	 /* Mac Interrupt Mask register		   */  
enum GmacInterruptEnableBit
{
	GmacRXSTSIE						= 0x00004000U,	 /* Receive Status Interrupt Enable					 14		 RW		 0				  */
	GmacTXSTSIE						= 0x00002000U,	 /* Transmit Status Interrupt Enable				 13		 RW		 0				  */
	GmacTSIE						= 0x00001000U,	 /* Timestamp Interrupt Mask						 12		 RW		 0				  */
	GmacLPIIE						= 0x00000020U,	 /* LPI Interrupt Enable							 5		 RW		 0				  */
	GmacPMTIE						= 0x00000010U,	 /* PMT Interrupt Enable							 4		 RW		 0				  */
	GmacPHYIE						= 0x00000008U,	 /* PHY Interrupt Enable							 3		 RW		 0				  */
	GmacPCSANCIE					= 0x00000004U,	 /* PCS AN Completion Interrupt Mask				 2		 RW		 0				  */
	GmacPCSLCHGIE					= 0x00000002U,	 /* PCS Link Status Interrupt Enable				 1		 RW		 0				  */	
	GmacRGSMIIE						= 0x00000001U,	 /* RGMII or SMII Interrupt Enable					 0		 RW		 0				  */
};

//GmacRxTxStat		  = (0x00B8),	 /* register contains the receive and transmitt Error status  */ 
enum GmacRxTxStat
{
	GmacRWT							= 0x00000100U,	/* Receive Watchdog Timeout							 8		 RW		 0				  */
	GmacEXCOL						= 0x00000020U,	/* Excessive collision								 5		 RW		 0				  */
	GmacLCOL						= 0x00000010U,	/* Late Collision									 4		 RW		 0				  */
	GmacEXDEF						= 0x00000008U,	/* Excessive Deferral								 3		 RW		 0				  */
	GmacLCARR						= 0x00000004U,	/* Loss Carrier										 2		 RW		 0				  */
	GmacNCARR						= 0x00000002U,	/* No Carrier										 1		 RW		 0				  */
	GmacTJT							= 0x00000001U,	/* Transmit Jabber Timeout							 0		 RW		 0				  */

};
//GmacPMTCtrl		  = (0x00C0),	 /* Present only when you select PMT module */ 
enum GmacPmtCtrlStatus
{
	GmacPmtFrmFilterPtrReset		= (int32_t)0x80000000,	  /* Remote Wake-Up Packet Filter Register Pointer Rst 31	   RW	   0				*/
	GmacRWKPTR						= 0x07000000U,	/* Remote Wake-up FIFO Pointer						 26:24	 RW		 000			  */
	GmacPmtGlobalUnicast			= 0x00000200U,	/* Global Unicast									 9		 RW		 0				  */
	GmacPmtWakeupFrameReceived		= 0x00000040U,	/* Remote Wake-Up Packet Receive					 6		 RW		 0				  */
	GmacPmtMagicPktReceived			= 0x00000020U,	  /* Magic Packet Receive							   5	   RW	   0				*/
	GmacPmtWakeupFrameEnable		= 0x00000004U,	  /* Remote Wake-Up Packet Enable					   2	   RW	   0				*/
	GmacPmtMagicPktEnable			= 0x00000002U,	/* Magic Packet Enable								 1		 RW		 0				  */
	GmacPmtPowerDown				= 0x00000001U,	/* Power Down										 0		 RW		 0				  */
};
//	  GmacLPICtrlSts	  = (0x00D0),	 /* LPI (low power idle) Control and Status Register		  */

enum GmacLPICtrlSts
{
	GmacLPITXA						= 0x00080000U,	/* LPI Tx Automate									19		RW		0000			 */
	GmacPLSEN						= 0x00040000U,	/* PHY Link Status Enable							18		RW		00				 */
	GmacPLS							= 0x00020080U,	/* PHY Link Status									17		RW		0				 */
	GmacLPIEN						= 0x00010000U,	/* LPI Enable										16		RW		0				*/
	GmacRLPIST						= 0x00000200U,	/* Receive LPI State								9		RW		0				 */
	GmacTLPIST						= 0x00000100U,	/* Transmit LPI State								8		RW		0				 */
	GmacRLPIEX						= 0x00000008U,	/* Receive LPI Exit									3		RW		0				 */
	GmacRLPIEN						= 0x00000004U,	/* Receive LPI Entry								2		RW		0				 */
	GmacTLPIEX						= 0x00000002U,	/* Transmit LPI Exit								1		RW		0				 */
	GmacTLPIEN						= 0x00000001U,	/* Transmit LPI Entry								0		RW		0				 */
};

//GmacLPITimerCtrl	  = (0x00D4),	 /* LPI timer control register				 */

enum GmacLPITimerCtrl
{
	GmacLST							= 0x00FF0000U,	/* LPI LS Timer										 25:16	 RW		 0000			  */
	GmacTWT							= 0x0000FFFFU,	/* LPI LW Timer										 15:0	 RW		 00				  */
};

// GmacANCrtl		   = (0x00E0),	  /* register enables and/or restarts auto-negotiation */
enum GmacANCrtl
{
	GmacSGMRAL						= 0x00040000U,	/* SGMII RAL Control								18		RW		0000			 */
	GmacLR							= 0x00020000U,	/* Lock to Reference								17		RW		00				 */
	GmacECD							= 0x00010080U,	/* Enable Comma Detect								16		RW		0				 */
	GmacELE							= 0x00004000U,	/* External Loopback Enable							14		RW		0				*/
	GmacANE							= 0x00001000U,	/* Auto-Negotiation Enable							12		RW		0				 */
	GmacRAN							= 0x00000200U,	/* Restart Auto-Negotiation							9		RW		0				 */
	
};
//GmacANStat		  = (0x00E4),	 /* register indicates the link and auto-negotiation status */
enum GmacANStat
{
	GmacES				   = 0x00000100U,	/* Extended Status									 8		 RW		 0000			  */
	GmacANC				   = 0x00000020U,	/* Auto-Negotiation Complete						 5		 RW		 00				  */
	GmacANA				   = 0x00000008U,	/* Auto-Negotiation Ability							 3		 RW		 0				  */
	GmacLS				   = 0x00000004U,	/* Link Status										 2		 RW		 0				 */
	
}; 

//GmacANAdvert		  = (0x00E8),	 /* register is configured before auto-negotiation begins */
 
//GmacANLinkPrtnr	  = (0x00EC),	 /* This register contains the advertised ability of the link partner */
enum GmacANLinkPrtnr
{
	GmacNP				   = 0x00008000U,	/* Next Page Support								 15		 RW		 0				 */
	GmacACK				   = 0x00004000U,	/* Acknowledge										 14		 RW		 0				 */
	GmacREF				   = 0x00003000U,	/* Remote Fault Encoding							 13:12	 RW		 00				 */
	GmacPSE				   = 0x00000180U,	/* Pause Encoding									 8:7	 RW		 00				 */
	GmacHD				   = 0x00000040U,	/* Half Duplex										 6		 RW		 0				 */
	GmacFD				   = 0x00000020U,	/* Full Duplex										 5		 RW		 0				 */
	
}; 
//GmacANExpan		  = (0x00F0),	 /* Register indicates whether new base page has been received from the link partner*/
enum GmacANExpan
{
	GmacNPA				   = 0x00000004U,	/* Next Page Ability								 2		 RW		 0				 */
	GmacNPR				   = 0x00000002U,	/* Next Page Received								 1		 RW		 0				 */
	
}; 

//	GmacTBIExtend		= (0x00F4),	   /* Register indicates all modes of operation of the MAC */
enum GmacTBIExtend
{
	GmacGFD				   = 0x00008000U,	/* 1000BASE-X Full-Duplex Capable					 15		 RW		 0				 */
	GmacGHD				   = 0x00004000U,	/* 100BASE-X Half-Duplex Capable					 14		 RW		 0				 */
	
}; 
//GmacPHYIFCtrl		  = (0x00F8),	 /* Register contains the control for RGMII, SGMII and SMII PHY interface*/
enum GmacPHYIFCtrl
{
	GmacFALSCARDET		   = 0x00200000U,	/* False Carrier Detected							21		RW		0				 */
	GmacJABTO			   = 0x00100000U,	/* Jabber Timeout									20		RW		0				 */
	GmacLNKSTS			   = 0x00080080U,	/* Link Status										19		RW		0				 */
	GmacLNKSPEED		   = 0x00060000U,	/* Link Speed										18:17	RW		00				 */
	GmacLNKMOD			   = 0x00010000U,	/* Link Mode										16		RW		0				 */
	GmacSMIDRXS			   = 0x00000010U,	/* SMII Rx Data Sampling							4		RW		0				 */
	GmacSFTERR			   = 0x00000004U,	/* SMII Force Transmit Error						2		RW		0				 */
	GmacLUD				   = 0x00000002U,	/* Link Up or Down									1		RW		0				 */
	GmacTC				   = 0x00000001U,	/* Transmit Configuration in RGMII, SGMII, or SMII	0		RW		0				 */
};

/* MAC_Version	= 0x110,  */
enum GmacMACVersion
{
	GmacUSERVER		= 0x0000FF00U,	   /* User-defined Version (configured with coreConsultant)		15:8	RO		xxH		*/
	GmacSNPSVER		= 0x000000FFU,	   /* Synopsys-defined Version (3.7)								7:0		RO		37H		*/
};
	
 /* MAC_Debug  = 0x114,	 */
enum GmacMACDebug
{
	GmacTFCSTS			= 0x00060000U,	 /* MAC Transmit Packet Controller Status				18:17	RO			*/
	GmacTPESTS			= 0x00010000U,	 /* MAC GMII or MII Transmit Protocol Engine Status		16		RO			*/
	GmacRFCFCSTS		= 0x00000006U,	 /* MAC Receive Packet Controller FIFO Status			2:1		RO			*/
	GmacRPESTS			= 0x00000001U,		/* MAC GMII or MII Receive Protocol Engine Status		0		RO		0	*/
};	
 
/* MAC_HW_Feature1	= 0x120,  */
enum GmacHWFeature1
{
	GmacL3L4FNUM			= 0x78000000U,	   /* Total number of L3 or L4 Filters		30:27	RO			*/
	GmacHASHTBLSZ			= 0x03000000U,	   /* Hash Table Size						25:24	RO			*/
	GmacLPMODEEN			= 0x00800000U,	   /* Low Power Mode Enabled					23		RO			*/
	GmacAVSEL				= 0x00100000U,	   /* AV Feature Enabled						20		RO			*/
	GmacDBGMEMA				= 0x00080000U,	   /* DMA Debug Registers Enabled			19		RO			*/
	GmacTSOEN				= 0x00040000U,	   /* TCP Segmentation Offload Enable		18		RO			*/
	GmacSPHEN				= 0x00020000U,	   /* Split Header Feature Enable			17		RO			*/
	GmacDCBEN				= 0x00010000U,	   /* DCB Feature Enable						16		RO			*/
	GmacADVTHWORD			= 0x00002000U,	   /* IEEE 1588 High Word Register Enable	13		RO			*/
	GmacTXFIFOSIZE			= 0x000007C0U,	   /* MTL Transmit FIFO Size					10:6	RO			*/
	GmacRXFIFOSIZE			= 0x0000001FU,	   /* MTL Receive FIFO Size					4:0		RO			*/
};	

/* MAC_HW_Feature2	= 0x124,  */
enum GmacHWFeature2
{
	GmacAUXSNAPNUM		= 0x70000000U,	   /* Number of Auxiliary Snapshot Inputs		30:28	RO			*/
	GmacPPSOUTNUM		= 0x07000000U,	   /* Number of PPS Outputs						26:24	RO			*/
	GmacTXCHCNT			= 0x003C0000U,	   /* Number of DMA Transmit Channels			21:18	RO			*/
	GmacRXCHCNT			= 0x0000F000U,	   /* Number of DMA Receive Channels				15:12	RO			*/
	GmacTXQCNT			= 0x000003C0U,	   /* Number of MTL Transmit Queues				9:6		RO			*/
	GmacRXQCNT			= 0x0000000FU,	   /* Number of MTL Receive Queues				3:0		RO			*/
};	
   
/* MAC_GMII_Address	 = 0x200,  */
enum GmacGmiiAddr
{
	GmiiDevMask			= 0x03E00000U,	   /* Physical Layer Address			25:21	RW		00000		*/
	GmiiDevShift		= 21U,
	GmiiRegMask			= 0x001F0000U,	   /* GMII Register					20:16	RW		00000		*/
	GmiiRegShift		= 16U,
	GmiiCsrClkMask		= 0x00000F00U,	   /* CSR Clock Range				11:8	RW		0000   */
	GmiiCsrClk5			= 0x00000500U,	   /* (CR)CSR Clock Range	  250-300 MHz	   4:2		RW		   000	   */
	GmiiCsrClk4			= 0x00000400U,	   /*						  150-250 MHz								   */
	GmiiCsrClk3			= 0x00000300U,	   /*						  35-60 MHz									   */
	GmiiCsrClk2			= 0x00000200U,	   /*						  20-35 MHz									   */
	GmiiCsrClk1			= 0x00000100U,	   /*						  100-150 MHz								   */
	GmiiCsrClk0			= 0x00000000U,	   /*						  60-100 MHz */ 
	GmacSKAP			= 0x00000010U,	   /* Skip Address Packet			4		RW		0		*/
	GmiiWrite			= 0x00000004U,	   /* GMII Command write			3:2		RW		00	 */
	GmiiRead			= 0x0000000CU,	   /* GMII Operation Command	Read		3:2		RW		00	 */
	GmacC45E			= 0x00000002U,	   /* Clause 45 PHY Enable			1		RW		0		*/
	GmiiBusy			= 0x00000001U,	   /* GMII Busy						0		R_WS_SC	0	*/
};	  
	
/* MAC_GMII_Data  = 0x204,	*/
enum GmacGmiiData
{
	GmacRA			= (int32_t)0xFFFF0000,	   /* Register Address				31:16	RW	0000H		*/
	GmiiDataMask	= 0x0000FFFFU,	   /* GMII Data						15:0	RW	0000H	*/
};	
  
/* MAC_GPIO_Control	 = 0x208,  */
enum GmacGPIOControl
{
	GmacGPIT	= (int32_t)0xFFFF0000,	   /* GPI Type				31:16	RW	0000H		*/
	GmacGPIE	= 0x0000000FU,	   /* GPI Interrupt Enable	 3:0	RW	0H	 */
};	
	
/* MAC_GPIO_Status	= 0x20c,  */
enum GmacGPIOStatus
{
	GmacGPO		= (int32_t)0xFFFF0000,	   /* General Purpose Output		31:16	RW		0000H		*/
	GmacGPIS	= 0x0000FFFFU,	   /* General Purpose Input Status	 15:0	LL, LH	0000H	*/
};

/* MAC_L3_L4_Control0  = 0x900,	 */
enum GmacL3L4Control0
{
	GmacL4DPIM0		= 0x00200000U,		/* Layer 4 Destination Port Inverse Match Enable		21		RW		0		*/
	GmacL4DPM0		= 0x00100000U,		/* Layer 4 Destination Port Match Enable				20		RW		0		*/
	GmacL4SPIM0		= 0x00080000U,			/* Layer 4 Source Port Inverse Match Enable				19		RW		0		*/
	GmacL4SPM0		= 0x00040000U,			/* Layer 4 Source Port Match Enable						18		RW		0		*/
	GmacL4PEN0		= 0x00010000U,		/* Layer 4 Protocol Enable								16		RW		0		*/
	GmacL3HDBM0		= 0x0000F800U,		/* Layer 3 IP DA Higher Bits Match						15:11	RW		00H		*/
	GmacL3HSBM0		= 0x000007C0U,		/* Layer 3 IP SA Higher Bits Match						10:6	RW		00H		*/
	GmacL3DAIM0		= 0x00000020U,		/* Layer 3 IP DA Inverse Match Enable					5		RW		0		*/
	GmacL3DAM0		= 0x00000010U,		/* Layer 3 IP DA Match Enable							4		RW		0		*/
	GmacL3SAIM0		= 0x00000008U,		/* Layer 3 IP SA Inverse Match Enable					3		RW		0		*/
	GmacL3SAM0		= 0x00000004U,		/* Layer 3 IP SA Match Enable							2		RW		0		*/
	GmacL3PEN0		= 0x00000001U		/* Layer 3 Protocol Enable								0		RW		0		*/
};	
  
/* MAC_Layer4_Address0	= 0x904,  */
enum GmacLayer4Address0
{
	GmacL4DP0			 = (int32_t)0xFFFF0000,		  /* Layer 4 Destination Port Number Field		31:16		RW		   0000H	*/
	GmacL4SP0			 = 0x0000FFFFU,		  /* Layer 4 Source Port Number Field			15:0		RW		   0000H	*/
	
};	
  
/* MAC_Layer3_Addr0_Reg0  = 0x910,	*/
enum GmacLayer3Addr0Reg0
{
	GmacL3A00			 = (int32_t)0xFFFFFFFF,		  /* Layer 3 Address 0 Field	 31:0			RW		   00000000H	*/
};	
  
/* MAC_Layer3_Addr1_Reg0  = 0x914,	*/
enum GmacLayer3Addr1Reg0
{
	GmacL3A10			 = (int32_t)0xFFFFFFFF,		  /* Layer 3 Address 1 Field	 31:0			RW		   00000000H	*/
};	
  
/* MAC_Layer3_Addr2_Reg0  = 0x918,	*/
enum GmacLayer3Addr2Reg0
{
	GmacL3A20			 = (int32_t)0xFFFFFFFF,		  /* Layer 3 Address 2 Field	 31:0			RW		   00000000H	*/
};	
  
/* MAC_Layer3_Addr3_Reg0  = 0x91c,	*/
enum GmacLayer3Addr3Reg0
{
	GmacL3A30			 = (int32_t)0xFFFFFFFF,		  /* Layer 3 Address 3 Field	 31:0			RW		   00000000H	*/
};	 
  
/* MAC_ARP_Address	= 0xae0,  */
enum GmacARPAddress
{
	GmacARPPA			 = (int32_t)0xFFFFFFFF,		  /* ARP Protocol Address	  31:0			 RW			00000000H	 */
};	
	
/* GmacTSControl  = 0xb00,	 Controls the Timestamp update logic  : only when IEEE 1588 time stamping is enabled in corekit			*/
enum GmacTSControlReg
{
  //GmacTSENMACADDR	  = 0x00040000,		/* Enable Mac Addr for PTP filtering	 18			   RW		  0		*/
  
  //GmacTSOrdClk		  = 0x00000000,		/* 00=> Ordinary clock*/
  //GmacTSBouClk		  = 0x00010000,		/* 01=> Boundary clock*/
  //GmacTSEtoEClk		  = 0x00020000,		/* 10=> End-to-End transparent clock*/
  //GmacTSPtoPClk		  = 0x00030000,		/* 11=> P-to-P transparent clock*/

	GmacAV8021ASMEN		= 0x10000000U,			/*	AV 802.1AS Mode Enable							28			  RW		0	*/
	GmacTXTSSTSM		= 0x01000000U,		/*	Transmit Timestamp Status Mode					24			  RW		0	*/
	GmacESTI			= 0x00100000U,		/*	External System Time Input						20			  RW		0	*/
	GmacTSENMACADDR		= 0x00040000U,		/*	Enable MAC Address for PTP Packet Filtering		18			  RW		0	*/
//	GmacTSCLKTYPE		= 0x00030000,		/*	Select PTP packets for Taking Snapshots			17:16		  RW		 0		*/
	GmacTSCLKTYPE		= 0x00010000U,		/*	Select PTP packets for Taking Snapshots			17:16		  RW		 0		*/
  /*
	  TSCLKTYPE		   TSMSTRENA	  TSEVNTENA			Messages for wihich TS snapshot is taken
	   00/01				X			  0				 SYNC, FOLLOW_UP, DELAY_REQ, DELAY_RESP
	   00/01				1			  0				 DELAY_REQ
	   00/01				0			  1				 SYNC
		10					NA			  0				 SYNC, FOLLOW_UP, DELAY_REQ, DELAY_RESP
		10					NA			  1				 SYNC, FOLLOW_UP
		11					NA			  0				 SYNC, FOLLOW_UP, DELAY_REQ, DELAY_RESP, PDELAY_REQ, PDELAY_RESP
		11					NA			  1				 SYNC, PDELAY_REQ, PDELAY_RESP		  
  */

	GmacTSMSTRENA		= 0x00008000U,		/*	Ena TS Snapshot for Master Messages				15			  RW		 0		*/
	GmacTSEVNTENA		= 0x00004000U,		/*	Ena TS Snapshot for Event Messages				14			  RW		 0		*/
	GmacTSIPV4ENA		= 0x00002000U,		/*	Ena TS snapshot for IPv4						13			  RW		 1		*/
	GmacTSIPV6ENA		= 0x00001000U,		/*	Ena TS snapshot for IPv6						12			  RW		 0		*/
	GmacTSIPENA			= 0x00000800U,		/*	Ena TS snapshot for PTP over E'net				11			  RW		 0		*/
	GmacTSVER2ENA		= 0x00000400U,		/*	Ena PTP snooping for version 2					10			  RW		 0		*/
	GmacTSCTRLSSR		= 0x00000200U,			/* Digital or Binary Rollover						9			  RW		 0		*/
	GmacTSENALL			= 0x00000100U,			/* Enable TS for all frames (Ver2 only)				8			  RW		 0		*/
	GmacTSADDREG		= 0x00000020U,		/* Addend Register Update							5			  RW_SC		 0		*/
	GmacTSTRIG			= 0x00000010U,		/* Time stamp interrupt Trigger Enable				4			  RW_SC		 0		*/
	GmacTSUPDT			= 0x00000008U,		/* Time Stamp Update								3			  RW_SC		 0		*/
	GmacTSINT			= 0x00000004U,		/* Time Atamp Initialize							2			  RW_SC		 0		*/
	GmacTSCFUPDT		= 0x00000002U,		/* Time Stamp Fine/Coarse							1			  RW		 0		*/
	GmacTSCUPDTCoarse	= 0x00000000U,		/* 0=> Time Stamp update method is coarse										*/
	GmacTSCUPDTFine		= 0x00000002U,		/* 1=> Time Stamp update method is fine										*/
	GmacTSENA			= 0x00000001U,		/* Time Stamp Enable								0			  RW		 0		*/
};


/* GmacTSSubSecIncr			  = 0xb04,	 8 bit value by which sub second register is incremented	 : only when IEEE 1588 time stamping without external timestamp input */
enum GmacTSSubSecIncrReg
{
  GmacSSINCMsk			  = 0x000000FFU,	   /* Only Lower 8 bits are valid bits	   7:0			 RW			00	  */
};

/* MACSystemTimeSeconds = 0xb08*/

/* MACSystemTimeNanoseconds = 0xb0c */

/* MACSystemTimeSecondsUpdate = 0xb10 */
enum GmacSTSecUpdate
{
  GmacTSS			 = (int32_t)0xFFFFFFFF,		  /* Timestamp Second		31:0		   RW		  00000000H	   */
};

/* MACSystemTimeNanosecondsUpadate = 0xb14 */
enum GmacSTNanoSecUpdate
{
  GmacTSSS			  = 0x7FFFFFFFU,	   /* Timestamp Sub Seconds	   30:0			  RW		 00000000H	  */
};

/* MAC_Timestamp_Addend = 0xb18 */
enum GmacTSAddend
{
  GmacTSAR			  = (int32_t)0xFFFFFFFF,	   /* Timestamp Addend Register	   31:0			  RW		 00000000H	  */
};

/* MAC_System_Time_Higher_Word_Seconds = 0xb1c */
enum GmacSTHigherWordSeconds
{
  GmacTSHWR			   = 0x0000FFFFU,		/* Timestamp Higher Word Register	 15:0			R_W_SU		   0000H	*/
};

/*	GmacTSStatus			= 0xb20,   Time Stamp Status Register																										*/
enum GmacTSStatusReg
{
  GmacATSNS			= 0x3E000000U,	   /* Number of Auxiliary Timestamp Snapshots					29:25		  RO			00000	*/
  GmacATSSTM		= 0x01000000U,	   /* Auxiliary Timestamp Snapshot Trigger Missed				24			  RO			0		*/
  GmacATSSTN		= 0x000F0000U,	   /* Auxiliary Timestamp Snapshot Trigger Identifier			19:16		  R_SS_RC		0000	*/
  GmacTSTRGTERR3	= 0x00000200U,	   /* Timestamp Target Time Error								9			  R_SS_RC		0		*/
  GmacTSTARGT3		= 0x00000100U,	   /* Timestamp Target Time Reached for Target Time PPS3			8			  R_SS_RC		0		*/
  GmacTSTRGTERR2	= 0x00000080U,	   /* Timestamp Target Time Error								7			  R_SS_RC		0		*/
  GmacTSTARGT2		= 0x00000040U,	   /* Timestamp Target Time Reached for Target Time PPS2			6			  R_SS_RC		0		*/
  GmacTSTRGTERR1	= 0x00000020U,	   /* Timestamp Target Time Error								5			  R_SS_RC		0		*/
  GmacTSTARGT1		= 0x00000010U,	   /* Timestamp Target Time Reached for Target Time PPS1			4			  R_SS_RC		0		*/
  GmacTSTRGTERR0	= 0x00000008U,	   /* Timestamp Target Time Error								3			  R_SS_RC		0		*/
  GmacAUXTSTRIG		= 0x00000004U,	   /* Auxiliary Timestamp Trigger Snapshot							2			  R_SS_RC		0		*/
  GmacTSTARGT0		= 0x00000002U,	   /* Timestamp Target Time Reached									1			  R_SS_RC		0		*/
  GmacTSSOVF		= 0x00000001U	   /* Timestamp Seconds Overflow								0			  R_SS_RC		0		*/ 
};

/* MAC_TxTimestamp_Status_Nanoseconds =	 */
enum GmacTxTSStatusNanoseconds
{
  GmacTXTSSTSMIS		= (int32_t)0x80000000,	 /* Transmit Timestamp Status Missed		31				R_SS_RC			0			*/
  GmacTXTSSTSLO			= 0x7FFFFFFFU,		/* Transmit Timestamp Status Low		30:0			RO				00000000H	*/
};

/* MAC_TxTimestamp_Status_Seconds =	 */
enum GmacTxTSStatusSeconds
{
  GmacTXTSSTSHI			= (int32_t)0xFFFFFFFF,		/* Transmit Timestamp Status High		31:0			RO				00000000H	*/
};

/* MAC_Auxiliary_Control = 0xb40 */
enum GmacAuxControl
{
  GmacATSEN3		= 0x00000080U,		/* Auxiliary Snapshot 3 Enable		7			RW			0	*/
  GmacATSEN2		= 0x00000040U,	 /* Auxiliary Snapshot 2 Enable		6			RW			0	*/
  GmacATSEN1		= 0x00000020U,		/* Auxiliary Snapshot 2 Enable		5			RW			0	*/
  GmacATSEN0		= 0x00000010U,	 /* Auxiliary Snapshot 2 Enable		4			RW			0	*/
  GmacATSFC			= 0x00000001U,		/* Auxiliary Snapshot FIFO Clear	0			R_WS_SC		0	*/
};

/* MAC_Auxiliary_Timestamp_Nanoseconds = 0xb48*/
enum GmacAuxTSNanoseconds
{
  GmacAUXTSLO		= 0x7FFFFFFFU	/* Contains the lower 31 bits (nano-seconds field) of the auxiliary timestamp	30:0	 RO		00000000H	*/
};

/* MAC_Auxiliary_Timestamp_Seconds = 0xb4c*/
enum GmacAuxTSSeconds
{
  GmacAUXTSHI		= (int32_t)0xFFFFFFFF	/* Contains the lower 32 bits of the Seconds field of the auxiliary timestamp	31:0	 RO		00000000H	*/
};

/* MAC_Timestamp_Ingress_Asym_Corr = 0xb50*/

/* MAC_Timestamp_Egress_Asym_Corr = 0xb54*/
enum GmacTSEgressAsymCorr
{
  GmacOSTEAC		= (int32_t)0xFFFFFFFF	/* One-Step Timestamp Egress Asymmetry Correction	  31:0	   RW	  00000000H	  */
};

/* MAC_PPS_Control = 0xb70 */
enum GmacPPSControl
{
  GmacTRGTMODSEL3	= 0x60000000U,		/* Target Time Register Mode for PPS3 Output		30:29	RW			00	 */
  GmacPPSCMD3		= 0x07000000U,		/* Flexible PPS3 Output Control						26:24	R_WS_SC		000	  */
  GmacTRGTMODSEL2	= 0x00600000U,		/* Target Time Register Mode for PPS2 Output		22:21	RW			00	 */
  GmacPPSCMD2		= 0x00070000U,		/* Flexible PPS2 Output Control						18:16	R_WS_SC		000	  */
  GmacTRGTMODSEL1	= 0x00006000U,		/* Target Time Register Mode for PPS1 Output		14:13	RW			00	 */
  GmacPPSCMD1		= 0x00000700U,		/* Flexible PPS1 Output Control						10:8	R_WS_SC		00	 */
  GmacTRGTMODSEL0	= 0x00000060U,		/* Target Time Register Mode for PPS0 Output		6:5		RW			00	 */
  GmacPPSEN0		= 0x00000010U,		/* Flexible PPS Output Mode Enable					4		RW			0	*/
  GmacPPSCTRL0		= 0x0000000FU,		/* PPS Output Frequency Control						3:0		RW			0H	 */
};

/* MAC_PPS0_Target_Time_Seconds = 0xb80 */
enum GmacPPS0TTSeconds
{
  GmacTSTRH0		= (int32_t)0xFFFFFFFF	/* PPS0 Target Time Seconds Register	 31:0	  RW	 00000000H	 */
};

/* MAC_PPS0_Target_Time_Nanoseconds = 0xb84 */
enum GmacPPS0TTNanoseconds
{
  GmacTRGTBUSY0			= (int32_t)0x80000000,		/* PPS0 Target Time Register Busy		 31			R_WS_SC		0		   */
  GmacTTSL0				= 0x7FFFFFFFU,		/* Target Time Low for PPS0 Register	 30:0		RW			00000000H	*/
};

/* MAC_PPS0_Interval = 0xb88 */
enum GmacPPS0Interval
{
  GmacPPSINT0		= (int32_t)0xFFFFFFFF	/* PPS0 Output Signal Interval	   31:0		RW	   00000000H   */
};

/* MAC_PPS0_Width = 0xb8c */
enum GmacMACPPS0Width
{
  GmacPPSWIDTH0		= (int32_t)0xFFFFFFFF	/* PPS0 Output Signal Width		31:0	 RW		00000000H	*/
};

/* MAC_PPS1_Target_Time_Seconds = 0xb90 */
enum GmacPPS1TargetTimeSeconds
{
  GmacTSTRH1		= (int32_t)0xFFFFFFFF	/* PPS1 Target Time Seconds		31:0	 RW		00000000H	*/
};

/* MAC_PPS1_Target_Time_Nanoseconds = 0xb94 */
enum GmacPPS1TargetTimeNanoseconds
{
  GmacTRGTBUSY1		= (int32_t)0x80000000,		/* PPS1 Target Time Register Busy		31		R_WS_SC		0		   */
  GmacTTSL1			= 0x7FFFFFFFU,		/* Target Time Low for PPS1 Register	30:0	RW			00000000H	*/
};



/*GmacFlowControl	 = 0x0018,	  Flow control Register	  Layout				  */
enum GmacFlowControlReg	 
{										   
  GmacPauseTimeMask		   		= (int32_t)0xFFFF0000,	  /* (PT) PAUSE TIME field in the control frame	 31:16	 RW		  0x0000  */
  GmacPauseTimeShift	   		= 16,
  
  GmacPauseLowThresh	   		= 0x00000030U,
  GmacPauseLowThresh3	   		= 0x00000030U,	  /* (PLT)thresh for pause tmr 256 slot time	  5:4	 RW				  */
  GmacPauseLowThresh2	   		= 0x00000020U,	  /*						   144 slot time							  */
  GmacPauseLowThresh1	   		= 0x00000010U,	  /*							28 slot time							  */
  GmacPauseLowThresh0	   		= 0x00000000U,	  /*							 4 slot time					   000	  */

  GmacUnicastPauseFrame	   		= 0x00000008U,
  GmacUnicastPauseFrameOn  		= 0x00000008U,	  /* (UP)Detect pause frame with unicast addr.	   3	RW				  */
  GmacUnicastPauseFrameOff 		= 0x00000000U,	  /* Detect only pause frame with multicast addr.					0	  */

  GmacRxFlowControl		   		= 0x00000004U,
  GmacRxFlowControlEnable  		= 0x00000004U,	  /* (RFE)Enable Rx flow control				   2	RW				  */
  GmacRxFlowControlDisable 		= 0x00000000U,	  /* Disable Rx flow control										0	  */

  GmacTxFlowControl		   		= 0x00000002U,
  GmacTxFlowControlEnable  		= 0x00000002U,	  /* (TFE)Enable Tx flow control				   1	RW				  */
  GmacTxFlowControlDisable 		= 0x00000000U,	  /* Disable flow control											0	  */

  GmacFlowControlBackPressure	= 0x00000001U,
  GmacSendPauseFrame	   		= 0x00000001U,	  /* (FCB/PBA)send pause frm/Apply back pressure   0	RW			0	  */
};

/*	GmacInterruptStatus	  = 0x0038,		Mac Interrupt ststus register		   */  
enum GmacInterruptStatusBitDefinition
{
  GmacTSIntSts		   		= 0x00000200U,	 /* set if int generated due to TS (Read Time Stamp Status Register to know details)*/
  GmacMmcRxChksumOffload	= 0x00000080U,	 /* set if int generated in MMC RX CHECKSUM OFFLOAD int register					  */ 
  GmacMmcTxIntSts	   		= 0x00000040U,	 /* set if int generated in MMC TX Int register			   */
  GmacMmcRxIntSts	   		= 0x00000020U,	 /* set if int generated in MMC RX Int register				   */
  GmacMmcIntSts		   		= 0x00000010U,	 /* set if any of the above bit [7:5] is set			   */
  GmacPmtIntSts		   		= 0x00000008U,	 /* set whenver magic pkt/wake-on-lan frame is received		   */
  GmacPcsAnComplete	   		= 0x00000004U,	 /* set when AN is complete in TBI/RTBI/SGMIII phy interface		*/
  GmacPcsLnkStsChange  		= 0x00000002U,	 /* set if any lnk status change in TBI/RTBI/SGMII interface		*/
  GmacRgmiiIntSts	   		= 0x00000001U,	 /* set if any change in lnk status of RGMII interface		   */

};




/**********************************************************
 * DMA Engine registers Layout
 **********************************************************/

/*DmaBusCfg				  = 0x0000,	   CSR0 - Bus Mode */
enum DmaBusCfgReg		  
{											 /* Bit description								   Bits		R/W	  Reset value */
	DmaBusReserved			= (int32_t )0xFFFFFFC0,	   /*  Rserved fileds															*/
	DmaWChSts				= 0x00000020U,	  /* AXI Master Write Chanel Status		   5		RO	   0	  */
	DmaRChSts				= 0x00000010U,	  /* AXI Master Read Chanel Status		  4		   RO	  0		 */
	DmaBurst16En			= 0x00000008U,	  /* Burst size 16 enable						 3		  RW	1	   */
	DmaBurst8En				= 0x00000004U,	  /* Burst size 8 enable						  2		   RW	 0		*/
	DmaBurst4En				= 0x00000002U,	  /* Burst size 4 enable						  1		   RW	 0		*/
	DmaFixedBurstMode		= 0x00000001U,	  /* Fixed burst mode							 0		 RW		0	   */
};

enum DmaTxChSts
{
											 /* Bit description								   Bits		R/W	  Reset value */
	DmaTxReserved			= (int32_t )0xFFFFF800,	   /*  Rserved fileds								 31-11						*/
	DmaTxChSts				= 0x00000700U,	  /* TX Channel status							 8-10	RO_S	 0		*/
	DmaFErrSts				= 0x000000E0U,	  /* Fatal bus error status						 5-7	 RO_S	  0		 */
	DmaFtlBusErr			= 0x00000010U,	  /* Fatal Bus Error								4		 RW_1C	 0		*/
	DmaETC					= 0x00000008U,	  /* Early transmit complete				   3		RW_1C	0	   */
	DmaUnderFlow			= 0x00000004U,	  /* Underflow										 2		  RW_1C	  0		 */
	DmaTrnsfStop			= 0x00000002U,	  /* Dma transfer Stopped					   1		RW_1C	0	   */
	DmaTrnsfComp			= 0x00000001U,	  /* Dma Transfer Complete					  0		   RW_1C   0	  */
};

enum DmaRxChSts
{
											 /* Bit description								   Bits		R/W	  Reset value */
	DmaRxReserved			= (int32_t )0xFFFFF800,	   /*  Rserved fileds								 31-11						*/
	DmaRxChSts				= 0x00000700U,	  /* TX Channel status							 8-10	RO_S	 0		*/
	DmaRxFErrSts			= 0x000000E0U,	  /* Fatal bus error status						 5-7	 RO_S	  0		 */
	DmaRxFtlBusErr			= 0x00000010U,	  /* Fatal Bus Error								4		 RW_1C	 0		*/
	DmaRxReserved1			= 0x00000008U,	  /* Reserved										 3							   */
	DmaRxUnderFlow			= 0x00000004U,	  /* Underflow										 2		  RW_1C	  0		 */
	DmaRecvStop				= 0x00000002U,	  /* Dma transfer Stopped					   1		RW_1C	0	   */
	DmaRecvComp				= 0x00000001U,	  /* Dma Transfer Complete					  0		   RW_1C   0	  */
};


enum DmaTxChIntMask
{
											 /* Bit description											Bits	 R/W   Reset value */
	DmaTxIntRsvd			= (int32_t )0xFFFFFFC0,	   /*  Rserved fileds										  31-5								 */
	DmaFErrIntEn			= 0x00000010U,	  /*  Fatal bus error interrupt enable				   4	   RW_1C	0			 */
	DmaETCEn				= 0x00000008U,	  /*  Early transmit complete interrupt enable	 3		 RW_1C	  0				*/
	DmaUNFEn				= 0x00000004U,	  /*  Underflow	 interrupt enable					   2	   RW_1C	0			  */
	DmaTSEn					= 0x00000002U,	  /*  Transfer Stopped	interrupt enable			1		RW_1C	 0			   */
	DmaTCEn					= 0x00000001U,	  /*  Transfer Complete	 interrupt enable		   0	   RW_1C	0			   */
};

enum DmaRxChIntMask
{
											 /* Bit description											Bits	 R/W   Reset value */
	DmaRxIntRsvd			= (int32_t )0xFFFFFFC0,	   /*  Rserved fileds										  31-5								 */
	DmaRxFErrIntEn			= 0x00000010U,	  /*  Fatal bus error interrupt enable				   4	   RW_1C	0			 */
	DmaRxIntRsvd1			= 0x00000008U,	  /*  Rsvd filed												3					   0			 */
	DmaRxUNFEn				= 0x00000004U,	  /*  Underflow	 interrupt enable					   2	   RW_1C	0			  */
	DmaRSEn					= 0x00000002U,	  /*  Receive Stopped  interrupt enable			   1	   RW_1C	0			  */
	DmaRCEn					= 0x00000001U,	  /*  Receive Complete	interrupt enable		  0		  RW_1C	   0			  */
};

enum DmaTxChCtl
{
											 /* Bit description											Bits	 R/W   Reset value */
	DmaTxCtlRsvd			= (int32_t )0xFFFFFFF0,	   /*  Rserved fileds																			   */
	DmaDSL					= 0x0000000EU,	  /*  Descriptor Skip length							 3-1	  RW	  0				*/
	DmaTxStartStop			  = 0x00000001U,	/*	Dma Start/Stop bit								   0		  RW	  0				*/
};

enum DmaRxChCtl
{
											 /* Bit description											Bits	 R/W   Reset value */
	DmaRxCtlRsvd			= (int32_t )0xFFFF0000,	   /*  Rserved fileds																			   */
	DmaRcIntWDtC			= 0x0000FF00U,	  /*  Receive Interrupt Watchdog Timer Count  15-8	  RW	  0				*/
	DmaRxCtlRsvd1			= 0x000000F0U,	  /* Reserved												   7-4								 */
	DmaRxDSL				= 0x0000000EU,	  /*  Descriptor Skip length							 3-1	  RW	  0				*/
	DmaRxStartStop			= 0x00000001U,	  /*  Dma Start/Stop bit								 0			RW		0			  */
};

enum DmaTxRxChDescLstHA
{
											 /* Bit description											Bits	 R/W   Reset value */
	DmaDescLstRsvd			= (int32_t )0xFFFFFFE0,	   /*  Rserved fileds																			   */
	DmaDesHA				= 0x0000001FU,	  /*  Upper address bits for Descriptor list		 4-0	 RW		  0			   */
};

enum DmaTxRxChDescLstLA
{
											 /* Bit description											Bits	 R/W   Reset value */
	DmaDesLA				= (int32_t )0xFFFFFFFF,	   /*  Lower address bits for Descriptor list		 31-0	  RW	   0			*/
};

enum DmaTxRxChDescTlPt
{
											 /* Bit description											Bits	 R/W   Reset value */
	DmaDesTail				= (int32_t )0xFFFFFFFF,	   /*  Tail Pointer for Descriptor list					  31-0	   RW		0			 */
};

enum DmaTxRChRngLn
{
											 /* Bit description											Bits	 R/W   Reset value */
	DmaDesLen				= (int32_t )0xFFFFFFFF,	   /*  Transmit descriptor ring length				  31-0	   RW		0			 */
};

enum DmaTxRxChCurDesc
{
											 /* Bit description											Bits	 R/W   Reset value */
	DmaCurDescPtr			= (int32_t )0xFFFFFFFF,	   /*  Current descriptor pointer						 31-0	  RO_S		 0			  */
};

enum DmaTxRxChCurBufHa
{
											 /* Bit description												   Bits		R/W	  Reset value */
	DmaReserved				= (int32_t )0xFFFFFFE0,	   /* Reserved fileds												31-5						   */
	DmaCurBufPHa			= 0x0000001FU,	  /*  Current buffer address pointer high address	  4-0	  RO_S		 0			  */
};

enum DmaTxRxChCurBufLa
{
											 /* Bit description												   Bits		R/W	  Reset value */
	DmaCurBufPLa			= (int32_t )0xFFFFFFFF,	   /*  Current buffer address pointer low address	   31-0		RO_S	   0			*/
};

/*DmaStatus			= 0x0014,	 CSR5 - Dma status Register						   */
enum DmaStatusReg		  
{ 
  GmacPmtIntr			  = 0x10000000U,   /* (GPI)Gmac subsystem interrupt						 28		RO		 0		 */ 
  GmacMmcIntr			  = 0x08000000U,   /* (GMI)Gmac MMC subsystem interrupt					 27		RO		 0		 */ 
  GmacLineIntfIntr		  = 0x04000000U,   /* Line interface interrupt							 26		RO		 0		 */ 
};

/*DmaControl		= 0x0018,	  CSR6 - Dma Operation Mode Register				*/
enum DmaControlReg		  
{ 
  DmaDisableDropTcpCs	  = 0x04000000U,   /* (DT) Dis. drop. of tcp/ip CS error frames		   26	   RW		 0		 */
										
  DmaStoreAndForward	  = 0x00200000U,   /* (SF)Store and forward							   21	   RW		 0		 */
  DmaFlushTxFifo		  = 0x00100000U,   /* (FTF)Tx FIFO controller is reset to default	   20	   RW		 0		 */ 
  
  DmaTxThreshCtrl		  = 0x0001C000U,   /* (TTC)Controls thre Threh of MTL tx Fifo		   16:14   RW				 */ 
  DmaTxThreshCtrl16		  = 0x0001C000U,   /* (TTC)Controls thre Threh of MTL tx Fifo 16	   16:14   RW				 */ 
  DmaTxThreshCtrl24		  = 0x00018000U,   /* (TTC)Controls thre Threh of MTL tx Fifo 24	   16:14   RW				 */ 
  DmaTxThreshCtrl32		  = 0x00014000U,   /* (TTC)Controls thre Threh of MTL tx Fifo 32	   16:14   RW				 */ 
  DmaTxThreshCtrl40		  = 0x00010000U,   /* (TTC)Controls thre Threh of MTL tx Fifo 40	   16:14   RW				 */	  
  DmaTxThreshCtrl256	  = 0x0000c000U,   /* (TTC)Controls thre Threh of MTL tx Fifo 256	   16:14   RW				 */	  
  DmaTxThreshCtrl192	  = 0x00008000U,   /* (TTC)Controls thre Threh of MTL tx Fifo 192	   16:14   RW				 */	  
  DmaTxThreshCtrl128	  = 0x00004000U,   /* (TTC)Controls thre Threh of MTL tx Fifo 128	   16:14   RW				 */	  
  DmaTxThreshCtrl64		  = 0x00000000U,   /* (TTC)Controls thre Threh of MTL tx Fifo 64	   16:14   RW		 000	 */ 
  
  DmaTxStart			  = 0x00002000U,   /* (ST)Start/Stop transmission					   13	   RW		 0		 */

  DmaRxFlowCtrlDeact	  = 0x00401800U,   /* (RFD)Rx flow control deact. threhold			   [22]:12:11	RW				   */ 
  DmaRxFlowCtrlDeact1K	  = 0x00000000U,   /* (RFD)Rx flow control deact. threhold (1kbytes)   [22]:12:11	RW		  00	   */ 
  DmaRxFlowCtrlDeact2K	  = 0x00000800U,   /* (RFD)Rx flow control deact. threhold (2kbytes)   [22]:12:11	RW				   */ 
  DmaRxFlowCtrlDeact3K	  = 0x00001000U,   /* (RFD)Rx flow control deact. threhold (3kbytes)   [22]:12:11	RW				   */ 
  DmaRxFlowCtrlDeact4K	  = 0x00001800U,   /* (RFD)Rx flow control deact. threhold (4kbytes)   [22]:12:11	RW				   */	
  DmaRxFlowCtrlDeact5K	  = 0x00400000U,   /* (RFD)Rx flow control deact. threhold (4kbytes)   [22]:12:11	RW				   */	
  DmaRxFlowCtrlDeact6K	  = 0x00400800U,   /* (RFD)Rx flow control deact. threhold (4kbytes)   [22]:12:11	RW				   */	
  DmaRxFlowCtrlDeact7K	  = 0x00401000U,   /* (RFD)Rx flow control deact. threhold (4kbytes)   [22]:12:11	RW				   */	
  
  DmaRxFlowCtrlAct		  = 0x00800600U,   /* (RFA)Rx flow control Act. threhold			  [23]:10:09   RW				  */ 
  DmaRxFlowCtrlAct1K	  = 0x00000000U,   /* (RFA)Rx flow control Act. threhold (1kbytes)	  [23]:10:09   RW		 00		  */ 
  DmaRxFlowCtrlAct2K	  = 0x00000200U,   /* (RFA)Rx flow control Act. threhold (2kbytes)	  [23]:10:09   RW				  */ 
  DmaRxFlowCtrlAct3K	  = 0x00000400U,   /* (RFA)Rx flow control Act. threhold (3kbytes)	  [23]:10:09   RW				  */ 
  DmaRxFlowCtrlAct4K	  = 0x00000300U,   /* (RFA)Rx flow control Act. threhold (4kbytes)	  [23]:10:09   RW				  */	
  DmaRxFlowCtrlAct5K	  = 0x00800000U,   /* (RFA)Rx flow control Act. threhold (5kbytes)	  [23]:10:09   RW				  */	
  DmaRxFlowCtrlAct6K	  = 0x00800200U,   /* (RFA)Rx flow control Act. threhold (6kbytes)	  [23]:10:09   RW				  */	
  DmaRxFlowCtrlAct7K	  = 0x00800400U,   /* (RFA)Rx flow control Act. threhold (7kbytes)	  [23]:10:09   RW				  */	
  
  DmaRxThreshCtrl		  = 0x00000018U,   /* (RTC)Controls thre Threh of MTL rx Fifo		   4:3	 RW				   */ 
  DmaRxThreshCtrl64		  = 0x00000000U,   /* (RTC)Controls thre Threh of MTL tx Fifo 64	   4:3	 RW				   */ 
  DmaRxThreshCtrl32		  = 0x00000008U,   /* (RTC)Controls thre Threh of MTL tx Fifo 32	   4:3	 RW				   */ 
  DmaRxThreshCtrl96		  = 0x00000010U,   /* (RTC)Controls thre Threh of MTL tx Fifo 96	   4:3	 RW				   */ 
  DmaRxThreshCtrl128	  = 0x00000018U,   /* (RTC)Controls thre Threh of MTL tx Fifo 128	   4:3	 RW				   */ 

  DmaEnHwFlowCtrl		  = 0x00000100U,   /* (EFC)Enable HW flow control					   8	   RW				  */ 
  DmaDisHwFlowCtrl		  = 0x00000000U,   /* Disable HW flow control											 0		  */ 
		
  DmaFwdErrorFrames		  = 0x00000080U,   /* (FEF)Forward error frames						   7	   RW		 0		 */
  DmaFwdUnderSzFrames	  = 0x00000040U,   /* (FUF)Forward undersize frames					   6	   RW		 0		 */
  DmaTxSecondFrame		  = 0x00000004U,   /* (OSF)Operate on second frame					   4	   RW		 0		 */
  DmaRxStart			  = 0x00000002U,   /* (SR)Start/Stop reception						   1	   RW		 0		 */
};


/**********************************************************
 * DMA Engine descriptors
 **********************************************************/
/*

********** Default Descritpor structure	 ****************************
DmaTxBaseAddr	  = 0x0020,	  CSR3 - Receive Descriptor list base address for channel 0		  
DmaRxBaseAddr is the pointer to the first Rx Descriptors. the Descriptor format in Little endian with a
32 bit Data bus is as shown below 

Similarly 
DmaRxBaseAddr	  = 0x0010,	 CSR4 - Transmit Descriptor list base address
DmaTxBaseAddr is the pointer to the first Rx Descriptors. the Descriptor format in Little endian with a
32 bit Data bus is as shown below
				  --------------------------------------------------------------------
	TDES0  |Buffer Address[31:0]												   |

	TDES1  |Launch Time Value											  |
		  --------------------------------------------------------------------
	TDES2  |IOC |TTSE |RSVD |TMODE |Buffer Address[36:32] |VTIR | Buffer Length[13:0]			   |
		  --------------------------------------------------------------------
	TDES3  |OWN |Control | Packet Length[13:0]						 |
		  --------------------------------------------------------------------
*/
enum DmaTxDescStatus	/* status word of DMA descriptor */
{
	/* This eplains the TX Desc 2 */
	DescIOC			 = (int32_t )0x80000000,   /* Interupt on completion					31		RW					*/
	DescTTSE		 = 0x40000000U,				  /* Transmit TimeStamp Enable			  30						*/
	DescTMODE		 = 0x00600000U,				  /* Timestamp mode							  22-21					  */
	DescBufHA		 = 0x001F0000U,				  /* Buffer High Address					   20-16				   */
	DescVTIR		 = 0x0000C000U,				  /* VLAN Tag Insertion or Replacement 15-14				  */
	DescBufLen		 = 0x00003FFFU,				  /* Buffer length									13-0				   */
	/* This explains the TDES3 bits layout */
	DescOwnDMA		 = (int32_t )0x80000000,  /*   Own Bit										   31	 */
	DescCtxType		 = 0x40000000U,				 /*	  Context Type									30	 */
	DescFD			 = 0x20000000U,				 /*	  First Descriptor								 29	  */
	DescLD			 = 0x10000000U,				 /*	  Last Descriptor								 28	  */
	DescCRCPADCtrl	 = 0x0C000000U,				 /*	  CRC Pad Control							   27-26  */
	DescSAInCtrl	 = 0x03800000U,				 /*	  SA Insertion Control						  25-23	 */
	DescCIC			 = 0x00030000U,				 /*	  Checksum Insertion Control			  17-16	 */
	DescPktLen		 = 0x00007FFFU,				 /*	  Pkt Length									   14-0	   */
	/* Tx Desc3 Write Back */
	DescWBTTSS		 = 0x00020000U,				 /*	  Tx Timestamp Status						   17		*/
	DescWBErrSum	 = 0x00008000U,				 /*	  Error Summary									  15	   */
	DescWBJbTO		 = 0x00004000U,				 /*	  Jabber Timeout								   14		*/
	DescWBFF		 = 0x00002000U,				 /*	  Packet Flushed									13		 */
	DescWBCE		 = 0x00001000U,				 /*	  Payload Checksum Error					  12	   */
	DescWBLOC		 = 0x00000800U,				 /*	  Loss of Carrier									 11		  */
	DescWBNC		 = 0x00000400U,				 /*	  No Carrier										  10	   */
	DescWBIHE		 = 0x00000001U,				 /*	  IP Header Error									 0		  */
};

/*
	RDES0  |Buffer Address[31:0]												   |

	RDES1  |Buffer Address[36:32]												 |
		  --------------------------------------------------------------------
	RDES2  |Reserved																   |
		  --------------------------------------------------------------------
	RDES3  |OWN |IOC |Reserved												   |
		  --------------------------------------------------------------------
*/
enum DmaRxDescStatus	/* status word of DMA descriptor */
{
	/* This explains the Read Format RX Descriptor 3 */
	DescRxOWN		   = (int32_t )0x80000000,	 /*	  Own Bit						 31		 RW				 */
	DescRxIOC		   = 0x40000000U,				/*	 Interupt on completion	 30								*/
	
	/* This explains the RX desc 0 Write Back */
	DescRxIVT		   = (int32_t )0xFFFF0000,	 /*	  Inner VLAN Tag			 31-16						 */
	DescRxOVT		   = 0x0000FFFFU,				/*	 Outer VLAN Tag				15-0						*/
	
		/* This expalins the RX Desc 1 Write Back */
	DescRxOPC		   = (int32_t )0xFFFF0000,	 /*	  OAM Sub-Type Code, or MAC Control Packet opcode 31-16	 */
	DescRxTD		   = 0x00008000U,				/*	 Timestamp Dropped		 15								*/
	DescRxTSA		   = 0x00004000U,				/*	 Timestamp Available	  14							*/
	DescRxPV		   = 0x00002000U,				/*	 PTP Version					13						*/
	DescRxPFT		   = 0x00001000U,				/*	 PTP Packet Type			  12						*/
	DescRxPMT		   = 0x00000F00U,				/*	 PTP Message Type			11-8						*/
	DescRxIPCE		   = 0x00000080U,				/*	 IP Payload Error				7						*/
	DescRxIPCB		   = 0x00000040U,				/*	 IP Checksum Bypassed	  6								*/
	DescRxIPV6		   = 0x00000020U,				/*	 IPv6 header Present		  5							*/
	DescRxIPV4		   = 0x00000010U,				/*	 IPv4 header Present		  4							*/
	DescRxIPHE		   = 0x00000008U,				/*	 IP header error				  3						*/
	DescRxPT		   = 0x00000007U,				/*	 Payload Type					  2-0					*/
	
		/* This explains the RX Desc 2 Write Back */
	DescRxMadrm		   = 0x07F80000U,				/*	 MAC Address Match or Hash Value		 26-19			*/
	DescRxHF		   = 0x00040000U,				/*	 Hash Filter Status							  18		*/
	DescRxDAF		   = 0x00020000U,				/*	 Destination Address Filter Fail			  17		*/
	DescRxSAF		   = 0x00010000U,				/*	 SA Address Filter Fail						  16		*/
	DescRxVF		   = 0x00008000U,				/*	 VLAN Filter Status							  15		*/

		/* This expalins the RX Desc 3 Write Back */
	DescRxCTXT		   = 0x40000000U,				/*	 Receive Context Descriptor					   30		 */
	DescRxFD		   = 0x20000000U,				/*	 First Descriptor							  29		 */
	DescRxLD		   = 0x10000000U,				/*	 Last Descriptor							   28		 */
	DescRxRS2V		   = 0x08000000U,				/*	 Receive Status RDES2 Valid					  27		 */
	DescRxRS1V		   = 0x04000000U,				/*	 Receive Status RDES1 Valid					  26		 */
	DescRxRS0V		   = 0x02000000U,				/*	 Receive Status RDES0 Valid					  25		 */
	DescRxCE		   = 0x01000000U,				/*	 CRC Error									  24		 */
	DescRxGP		   = 0x00800000U,				/*	 Giant Packet								  23		 */
	DescRxRWDT		   = 0x00400000U,				/*	 Receive Watchdog Timeout					  22		 */
	DescRxOE		   = 0x00200000U,				/*	 Overflow Error								   21		 */
	DescRxRE		   = 0x00100000U,				/*	 Receive Error								   20		 */
	DescRxDE		   = 0x00080000U,				/*	 Dribble Bit Error							  19		 */
	DescRxLT		   = 0x00070000U,				/*	 Length/Type Field								 18-16	 */
	DescRxES		   = 0x00008000U,				/*	 Error Summary								   15		 */
	DescRxPL		   = 0x00007FFFU,				/*	 Packet Length									 14-0	 */
};

// Rx Descriptor COE type2 encoding
enum RxDescCOEEncode
{
  RxLenLT600				= 0,	/* Bit(5:7:0)=>0 IEEE 802.3 type frame Length field is Lessthan 0x0600			*/
  RxIpHdrPayLoadChkBypass	= 1,	/* Bit(5:7:0)=>1 Payload & Ip header checksum bypassed (unsuppported payload)		*/
  RxIpHdrPayLoadRes			= 2,	/* Bit(5:7:0)=>2 Reserved								*/
  RxChkBypass				= 3,	/* Bit(5:7:0)=>3 Neither IPv4 nor IPV6. So checksum bypassed				*/
  RxNoChkError				= 4,	/* Bit(5:7:0)=>4 No IPv4/IPv6 Checksum error detected					*/
  RxPayLoadChkError			= 5,	/* Bit(5:7:0)=>5 Payload checksum error detected for Ipv4/Ipv6 frames			*/
  RxIpHdrChkError			= 6,	/* Bit(5:7:0)=>6 Ip header checksum error detected for Ipv4 frames			*/
  RxIpHdrPayLoadChkError	= 7,	/* Bit(5:7:0)=>7 Payload & Ip header checksum error detected for Ipv4/Ipv6 frames	*/
};

/**********************************************************
 * DMA engine interrupt handling functions
 **********************************************************/
 
 enum wrappedDmaIntEnum	 /* Intrerrupt types */
{
	wrappedDmaRxNormal		= 0x01U,   /* normal receiver interrupt */
	wrappedDmaRxStopped		= 0x02U,	 /* abnormal receiver interrupt */
	wrappedDmaRxUnderflow		= 0x04U,   /* abnormal receiver interrupt */

	wrappedDmaError			= 0x10U,	 /* Dma engine error for Tx and Rx */

	wrappedDmaTxNormal		= 0x11U,   /* normal transmitter interrupt */
	wrappedDmaTxStopped		= 0x12U,   /* transmitter stopped */
	wrappedDmaTxUnderflow 	= 0x14U,	 /* transmitter underflow interrupt */
	wrappedDmaTxETC			= 0x18U,   /* Early tranmit complete */
};


/**********************************************************
 * Initial register values
 **********************************************************/
enum InitialRegisters
{
   /* Full-duplex mode with perfect filter on */
  GmacConfigInitFdx1000	  = GmacWatchdogEnable | GmacJabberEnable		  | GmacFrameBurstEnable | GmacJumboFrameDisable
						  | GmacSelectGmii	   | GmacEnableRxOwn		  | GmacLoopbackOff
						  | GmacFullDuplex	   | GmacRetryEnable		  | GmacPadCrcStripDisable
						  | GmacBackoffLimit0  | GmacDeferralCheckDisable | GmacTxEnable		  | GmacRxEnable,
  
  /* Full-duplex mode with perfect filter on */
  GmacConfigInitFdx110	  = GmacWatchdogEnable | GmacJabberEnable		  | GmacFrameBurstEnable  | GmacJumboFrameDisable
						  | GmacSelectMii	   | GmacEnableRxOwn		  | GmacLoopbackOff
						  | GmacFullDuplex	   | GmacRetryEnable		  | GmacPadCrcStripDisable
						  | GmacBackoffLimit0  | GmacDeferralCheckDisable | GmacTxEnable		  | GmacRxEnable,

   /* Full-duplex mode */
   // CHANGED: Pass control config, dest addr filter normal, added source address filter, multicast & unicast 
   // Hash filter. 
   /*						 = GmacFilterOff		 | GmacPassControlOff | GmacBroadcastEnable */
   GmacFrameFilterInitFdx	= GmacFilterOn			| GmacPassControl0	   | GmacBroadcastEnable	|	GmacSrcAddrFilterDisable
							| GmacMulticastFilterOn | GmacDestAddrFilterNor| GmacMcastHashFilterOff
							| GmacPromiscuousModeOff| GmacUcastHashFilterOff,
   
   /* Full-duplex mode */
   GmacFlowControlInitFdx = GmacUnicastPauseFrameOff | GmacRxFlowControlEnable | GmacTxFlowControlEnable,

   /* Full-duplex mode */
   GmacGmiiAddrInitFdx	  = GmiiCsrClk1,  // For TC9560 FPGA!	orig GmiiCsrClk2


   /* Half-duplex mode with perfect filter on */
   // CHANGED: Removed Endian configuration, added single bit config for PAD/CRC strip,				  
   /*| GmacSelectMii	  | GmacLittleEndian		 | GmacDisableRxOwn		 | GmacLoopbackOff*/
   GmacConfigInitHdx1000  = GmacWatchdogEnable | GmacJabberEnable		  | GmacFrameBurstEnable  | GmacJumboFrameDisable
						  | GmacSelectGmii	   | GmacDisableRxOwn		  | GmacLoopbackOff
						  | GmacHalfDuplex	   | GmacRetryEnable		  | GmacPadCrcStripDisable	 
						  | GmacBackoffLimit0  | GmacDeferralCheckDisable | GmacTxEnable		  | GmacRxEnable,

   /* Half-duplex mode with perfect filter on */
   GmacConfigInitHdx110	  = GmacWatchdogEnable | GmacJabberEnable		  | GmacFrameBurstEnable  | GmacJumboFrameDisable
						  | GmacSelectMii	   | GmacDisableRxOwn		  | GmacLoopbackOff
						  | GmacHalfDuplex	   | GmacRetryEnable		  | GmacPadCrcStripDisable 
						  | GmacBackoffLimit0  | GmacDeferralCheckDisable | GmacTxEnable		  | GmacRxEnable,

   /* Half-duplex mode */
   GmacFrameFilterInitHdx = GmacFilterOn		  | GmacPassControl0		| GmacBroadcastEnable | GmacSrcAddrFilterDisable
						  | GmacMulticastFilterOn | GmacDestAddrFilterNor	| GmacMcastHashFilterOff
						  | GmacUcastHashFilterOff| GmacPromiscuousModeOff,

   /* Half-duplex mode */
   GmacFlowControlInitHdx = GmacUnicastPauseFrameOff | GmacRxFlowControlDisable | GmacTxFlowControlDisable,

   /* Half-duplex mode */
   GmacGmiiAddrInitHdx	  = GmiiCsrClk1, // For TC9560 FPGA!  orig GmiiCsrClk2



   /**********************************************
   *DMA configurations
   **********************************************/

//	DmaBusModeInit		   = DmaFixedBurstEnable |	 DmaBurstLength8   | DmaDescriptorSkip2		  | DmaResetOff,
//	 DmaBusModeInit			= DmaFixedBurstEnable |	  DmaBurstLength8	| DmaDescriptorSkip4	   | DmaResetOff,
   
   /* 1000 Mb/s mode */
   DmaControlInit1000	  = DmaStoreAndForward,//		| DmaTxSecondFrame ,

   /* 100 Mb/s mode */
   DmaControlInit100	  = DmaStoreAndForward,
   
   /* 10 Mb/s mode */
   DmaControlInit10		  = DmaStoreAndForward,

#if ( TC9560_USE_TSB_DMA_WRAPPER == DEF_ENABLED ) // Toshiba DMA Wrapper
  DmaIntTxAll			 = DmaFErrIntEn | DmaETCEn | DmaUNFEn | DmaTSEn | DmaTCEn,
  DmaIntRxAll			 = DmaRxFErrIntEn | DmaRxUNFEn | DmaRSEn | DmaRCEn,
#else
	DmaIntAll = 0xFFC7, // ALL 
#endif
  
};


/**********************************************************
 * Mac Management Counters (MMC)
 **********************************************************/
enum MMC_Reg {
	GmacMmcCntrl					= 0x0700U,	/* mmc control for operating mode of MMC						*/
	GmacMmcIntrRx					= 0x0704U,	/* maintains interrupts generated by rx counters					*/
	GmacMmcIntrTx					= 0x0708U,	/* maintains interrupts generated by tx counters					*/
	GmacMmcIntrMaskRx				= 0x070CU,	/* mask for interrupts generated from rx counters					*/
	GmacMmcIntrMaskTx				= 0x0710U,	/* mask for interrupts generated from tx counters					*/

	GmacMmcTxOctetCountGb			= 0x0714U,	/*Bytes Tx excl. of preamble and retried bytes	   (Good or Bad)			*/
	GmacMmcTxFrameCountGb			= 0x0718U,	/*Frames Tx excl. of retried frames			   (Good or Bad)			*/
	GmacMmcTxBcFramesG				= 0x071CU,	/*Broadcast Frames Tx				   (Good)				*/
	GmacMmcTxMcFramesG				= 0x0720U,	/*Multicast Frames Tx				   (Good)				*/
	
	GmacMmcTx64OctetsGb				= 0x0724U,	/*Tx with len 64 bytes excl. of pre and retried	   (Good or Bad)			*/
	GmacMmcTx65To127OctetsGb		= 0x0728U,	/*Tx with len >64 bytes <=127 excl. of pre and retried	  (Good or Bad)			*/
	GmacMmcTx128To255OctetsGb		= 0x072CU,	/*Tx with len >128 bytes <=255 excl. of pre and retried	  (Good or Bad)			*/
	GmacMmcTx256To511OctetsGb		= 0x0730U,	/*Tx with len >256 bytes <=511 excl. of pre and retried	  (Good or Bad)			*/
	GmacMmcTx512To1023OctetsGb		= 0x0734U,	/*Tx with len >512 bytes <=1023 excl. of pre and retried  (Good or Bad)			*/
	GmacMmcTx1024ToMaxOctetsGb		= 0x0738U,	/*Tx with len >1024 bytes <=MaxSize excl. of pre and retried (Good or Bad)		*/
	
	GmacMmcTxUcFramesGb				= 0x073CU,	/*Unicast Frames Tx						 (Good or Bad)			*/
	GmacMmcTxMcFramesGb				= 0x0740U,	/*Multicast Frames Tx				   (Good and Bad)			*/
	GmacMmcTxBcFramesGb				= 0x0744U,	/*Broadcast Frames Tx				   (Good and Bad)			*/
	GmacMmcTxUnderFlowError			= 0x0748U,	/*Frames aborted due to Underflow error							*/
	GmacMmcTxSingleColG				= 0x074CU,	/*Successfully Tx Frames after singel collision in Half duplex mode			*/
	GmacMmcTxMultiColG				= 0x0750U,	/*Successfully Tx Frames after more than singel collision in Half duplex mode		*/
	GmacMmcTxDeferred				= 0x0754U,	/*Successfully Tx Frames after a deferral in Half duplex mode				*/
	GmacMmcTxLateCol				= 0x0758U,	/*Frames aborted due to late collision error						*/
	GmacMmcTxExessCol				= 0x075CU,	/*Frames aborted due to excessive (16) collision errors					*/
	GmacMmcTxCarrierError			= 0x0760U,	/*Frames aborted due to carrier sense error (No carrier or Loss of carrier)		*/
	GmacMmcTxOctetCountG			= 0x0764U,	/*Bytes Tx excl. of preamble and retried bytes	   (Good)				*/
	GmacMmcTxFrameCountG			= 0x0768U,	/*Frames Tx							   (Good)				*/
	GmacMmcTxExessDef				= 0x076CU,	/*Frames aborted due to excessive deferral errors (deferred for more than 2 max-sized frame times)*/
	
	GmacMmcTxPauseFrames			= 0x0770U,	/*Number of good pause frames Tx.							*/
	GmacMmcTxVlanFramesG			= 0x0774U,	/*Number of good Vlan frames Tx excl. retried frames					*/

	GmacMmcRxFrameCountGb			= 0x0780U,	/*Frames Rx							   (Good or Bad)			*/
	GmacMmcRxOctetCountGb			= 0x0784U,	/*Bytes Rx excl. of preamble and retried bytes	   (Good or Bad)			*/
	GmacMmcRxOctetCountG			= 0x0788U,	/*Bytes Rx excl. of preamble and retried bytes	   (Good)				*/
	GmacMmcRxBcFramesG				= 0x078CU,	/*Broadcast Frames Rx				   (Good)				*/
	GmacMmcRxMcFramesG				= 0x0790U,	/*Multicast Frames Rx				   (Good)				*/
	
	GmacMmcRxCrcError				= 0x0794U,	/*Number of frames received with CRC error						*/
	GmacMmcRxAlignError				= 0x0798U,	/*Number of frames received with alignment (dribble) error. Only in 10/100mode		*/
	GmacMmcRxRuntError				= 0x079CU,	/*Number of frames received with runt (<64 bytes and CRC error) error			*/
	GmacMmcRxJabberError			= 0x07A0U,	/*Number of frames rx with jabber (>1518/1522 or >9018/9022 and CRC)			*/
	GmacMmcRxUnderSizeG				= 0x07A4U,	/*Number of frames received with <64 bytes without any error				*/
	GmacMmcRxOverSizeG				= 0x07A8U,	/*Number of frames received with >1518/1522 bytes without any error			*/
	
	GmacMmcRx64OctetsGb				= 0x07ACU,	/*Rx with len 64 bytes excl. of pre and retried	   (Good or Bad)			*/
	GmacMmcRx65To127OctetsGb		= 0x07B0U,	/*Rx with len >64 bytes <=127 excl. of pre and retried	  (Good or Bad)			*/
	GmacMmcRx128To255OctetsGb		= 0x07B4U,	/*Rx with len >128 bytes <=255 excl. of pre and retried	  (Good or Bad)			*/
	GmacMmcRx256To511OctetsGb		= 0x07B8U,	/*Rx with len >256 bytes <=511 excl. of pre and retried	  (Good or Bad)			*/
	GmacMmcRx512To1023OctetsGb		= 0x07BCU,	/*Rx with len >512 bytes <=1023 excl. of pre and retried  (Good or Bad)			*/
	GmacMmcRx1024ToMaxOctetsGb		= 0x07C0U,	/*Rx with len >1024 bytes <=MaxSize excl. of pre and retried (Good or Bad)		*/
	
	GmacMmcRxUcFramesG				= 0x07C4U,	/*Unicast Frames Rx						 (Good)				*/
	GmacMmcRxLengthError			= 0x07C8U,	/*Number of frames received with Length type field != frame size			*/
	GmacMmcRxOutOfRangeType			= 0x07CCU,	/*Number of frames received with length field != valid frame size			*/
	
	GmacMmcRxPauseFrames			= 0x07D0U,	/*Number of good pause frames Rx.							*/
	GmacMmcRxFifoOverFlow			= 0x07D4U,	/*Number of missed rx frames due to FIFO overflow					*/
	GmacMmcRxVlanFramesGb			= 0x07D8U,	/*Number of good Vlan frames Rx								*/
	
	GmacMmcRxWatchdobError			= 0x07DCU,	/*Number of frames rx with error due to watchdog timeout error				*/
	
	/* TC9560_TBD: add 0x07E0 -- 0x07F8 registers here!!!!! */
};

enum MMC_IP_RELATED_Reg
{
	GmacMmcRxIpcIntrMask			= 0x0800U,	/*Maintains the mask for interrupt generated from rx IPC statistic counters			*/
	GmacMmcRxIpcIntr				= 0x0808U,	/*Maintains the interrupt that rx IPC statistic counters generate			*/
	
	GmacMmcRxIpV4FramesG			= 0x0810U,	/*Good IPV4 datagrams received								*/
	GmacMmcRxIpV4HdrErrFrames		= 0x0814U,	/*Number of IPV4 datagrams received with header errors					*/
	GmacMmcRxIpV4NoPayFrames		= 0x0818U,	/*Number of IPV4 datagrams received which didnot have TCP/UDP/ICMP payload		*/
	GmacMmcRxIpV4FragFrames			= 0x081CU,	/*Number of IPV4 datagrams received with fragmentation					*/
	GmacMmcRxIpV4UdpChkDsblFrames	= 0x0820U,	/*Number of IPV4 datagrams received that had a UDP payload checksum disabled		*/
	
	GmacMmcRxIpV6FramesG			= 0x0824U,	/*Good IPV6 datagrams received								*/
	GmacMmcRxIpV6HdrErrFrames		= 0x0828U,	/*Number of IPV6 datagrams received with header errors					*/
	GmacMmcRxIpV6NoPayFrames		= 0x082CU,	/*Number of IPV6 datagrams received which didnot have TCP/UDP/ICMP payload		*/
	
	GmacMmcRxUdpFramesG				= 0x0830U,	/*Number of good IP datagrams with good UDP payload					*/
	GmacMmcRxUdpErrorFrames			= 0x0834U,	/*Number of good IP datagrams with UDP payload having checksum error			*/
	
	GmacMmcRxTcpFramesG				= 0x0838U,	/*Number of good IP datagrams with good TDP payload					*/
	GmacMmcRxTcpErrorFrames			= 0x083CU,	/*Number of good IP datagrams with TCP payload having checksum error			*/

	GmacMmcRxIcmpFramesG			= 0x0840U,	/*Number of good IP datagrams with good Icmp payload					*/
	GmacMmcRxIcmpErrorFrames		= 0x0844U,	/*Number of good IP datagrams with Icmp payload having checksum error			*/
	
	GmacMmcRxIpV4OctetsG			= 0x0850U,	/*Good IPV4 datagrams received excl. Ethernet hdr,FCS,Pad,Ip Pad bytes			*/
	GmacMmcRxIpV4HdrErrorOctets		= 0x0854U,	/*Number of bytes in IPV4 datagram with header errors					*/
	GmacMmcRxIpV4NoPayOctets		= 0x0858U,	/*Number of bytes in IPV4 datagram with no TCP/UDP/ICMP payload				*/
	GmacMmcRxIpV4FragOctets			= 0x085CU,	/*Number of bytes received in fragmented IPV4 datagrams					*/
	GmacMmcRxIpV4UdpChkDsblOctets	= 0x0860U,	/*Number of bytes received in UDP segment that had UDP checksum disabled		*/
	
	GmacMmcRxIpV6OctetsG			= 0x0864U,	/*Good IPV6 datagrams received excl. Ethernet hdr,FCS,Pad,Ip Pad bytes			*/
	GmacMmcRxIpV6HdrErrorOctets		= 0x0868U,	/*Number of bytes in IPV6 datagram with header errors					*/
	GmacMmcRxIpV6NoPayOctets		= 0x086CU,	/*Number of bytes in IPV6 datagram with no TCP/UDP/ICMP payload				*/
	
	GmacMmcRxUdpOctetsG				= 0x0870U,	/*Number of bytes in IP datagrams with good UDP payload					*/
	GmacMmcRxUdpErrorOctets			= 0x0874U,	/*Number of bytes in IP datagrams with UDP payload having checksum error		*/
	
	GmacMmcRxTcpOctetsG				= 0x0878U,	/*Number of bytes in IP datagrams with good TDP payload					*/
	GmacMmcRxTcpErrorOctets			= 0x087CU,	/*Number of bytes in IP datagrams with TCP payload having checksum error		*/
	
	GmacMmcRxIcmpOctetsG			= 0x0880U,	/*Number of bytes in IP datagrams with good Icmp payload				*/
	GmacMmcRxIcmpErrorOctets		= 0x0884U,	/*Number of bytes in IP datagrams with Icmp payload having checksum error		*/
};


enum MMC_CNTRL_REG_BIT_DESCRIPTIONS
{
	GmacMmcCounterFreeze			= 0x00000008U,		/* when set MMC counters freeze to current value				*/
	GmacMmcCounterResetOnRead		= 0x00000004U,		/* when set MMC counters will be reset to 0 after read				*/
	GmacMmcCounterStopRollover		= 0x00000002U,		/* when set counters will not rollover after max value				*/
	GmacMmcCounterReset				= 0x00000001U,		/* when set all counters wil be reset (automatically cleared after 1 clk)	*/
	
};

enum MMC_RX_INTR_MASK_AND_STATUS_BIT_DESCRIPTIONS
{
	GmacMmcRxWDInt					= 0x00800000U,		/* set when rxwatchdog error reaches half of max value				*/
	GmacMmcRxVlanInt				= 0x00400000U,		/* set when GmacMmcRxVlanFramesGb counter reaches half of max value		*/
	GmacMmcRxFifoOverFlowInt		= 0x00200000U,		/* set when GmacMmcRxFifoOverFlow counter reaches half of max value		*/
	GmacMmcRxPauseFrameInt			= 0x00100000U,		/* set when GmacMmcRxPauseFrames counter reaches half of max value		*/
	GmacMmcRxOutOfRangeInt			= 0x00080000U,		/* set when GmacMmcRxOutOfRangeType counter reaches half of max value		*/
	GmacMmcRxLengthErrorInt			= 0x00040000U,		/* set when GmacMmcRxLengthError counter reaches half of max value		*/
	GmacMmcRxUcFramesInt			= 0x00020000U,		/* set when GmacMmcRxUcFramesG counter reaches half of max value		*/
	GmacMmcRx1024OctInt				= 0x00010000U,		/* set when GmacMmcRx1024ToMaxOctetsGb counter reaches half of max value	*/
	GmacMmcRx512OctInt				= 0x00008000U,		/* set when GmacMmcRx512To1023OctetsGb counter reaches half of max value	*/
	GmacMmcRx256OctInt				= 0x00004000U,		/* set when GmacMmcRx256To511OctetsGb counter reaches half of max value		*/
	GmacMmcRx128OctInt				= 0x00002000U,		/* set when GmacMmcRx128To255OctetsGb counter reaches half of max value		*/
	GmacMmcRx65OctInt				= 0x00001000U,		/* set when GmacMmcRx65To127OctetsG counter reaches half of max value		*/
	GmacMmcRx64OctInt				= 0x00000800U,		/* set when GmacMmcRx64OctetsGb counter reaches half of max value		*/
	GmacMmcRxOverSizeInt			= 0x00000400U,		/* set when GmacMmcRxOverSizeG counter reaches half of max value		*/
	GmacMmcRxUnderSizeInt			= 0x00000200U,		/* set when GmacMmcRxUnderSizeG counter reaches half of max value		*/
	GmacMmcRxJabberErrorInt			= 0x00000100U,		/* set when GmacMmcRxJabberError counter reaches half of max value		*/
	GmacMmcRxRuntErrorInt			= 0x00000080U,		/* set when GmacMmcRxRuntError counter reaches half of max value		*/
	GmacMmcRxAlignErrorInt			= 0x00000040U,		/* set when GmacMmcRxAlignError counter reaches half of max value		*/
	GmacMmcRxCrcErrorInt			= 0x00000020U,		/* set when GmacMmcRxCrcError counter reaches half of max value			*/
	GmacMmcRxMcFramesInt			= 0x00000010U,		/* set when GmacMmcRxMcFramesG counter reaches half of max value		*/
	GmacMmcRxBcFramesInt			= 0x00000008U,		/* set when GmacMmcRxBcFramesG counter reaches half of max value		*/
	GmacMmcRxOctetGInt				= 0x00000004U,		/* set when GmacMmcRxOctetCountG counter reaches half of max value		*/
	GmacMmcRxOctetGbInt				= 0x00000002U,		/* set when GmacMmcRxOctetCountGb counter reaches half of max value		*/
	GmacMmcRxFrameInt				= 0x00000001U,		/* set when GmacMmcRxFrameCountGb counter reaches half of max value		*/
};

enum MMC_TX_INTR_MASK_AND_STATUS_BIT_DESCRIPTIONS
{
	GmacMmcTxVlanInt				= 0x01000000U,		/* set when GmacMmcTxVlanFramesG counter reaches half of max value		*/
	GmacMmcTxPauseFrameInt			= 0x00800000U,		/* set when GmacMmcTxPauseFrames counter reaches half of max value		*/
	GmacMmcTxExessDefInt			= 0x00400000U,		/* set when GmacMmcTxExessDef counter reaches half of max value			*/
	GmacMmcTxFrameInt				= 0x00200000U,		/* set when GmacMmcTxFrameCount counter reaches half of max value		*/
	GmacMmcTxOctetInt				= 0x00100000U,		/* set when GmacMmcTxOctetCountG counter reaches half of max value		*/
	GmacMmcTxCarrierErrorInt		= 0x00080000U,		/* set when GmacMmcTxCarrierError counter reaches half of max value		*/
	GmacMmcTxExessColInt			= 0x00040000U,		/* set when GmacMmcTxExessCol counter reaches half of max value			*/
	GmacMmcTxLateColInt				= 0x00020000U,		/* set when GmacMmcTxLateCol counter reaches half of max value			*/
	GmacMmcTxDeferredInt			= 0x00010000U,		/* set when GmacMmcTxDeferred counter reaches half of max value			*/
	GmacMmcTxMultiColInt			= 0x00008000U,		/* set when GmacMmcTxMultiColG counter reaches half of max value		*/
	GmacMmcTxSingleCol				= 0x00004000U,		/* set when GmacMmcTxSingleColG	counter reaches half of max value		*/
	GmacMmcTxUnderFlowErrorInt		= 0x00002000U,		/* set when GmacMmcTxUnderFlowError counter reaches half of max value		*/
	GmacMmcTxBcFramesGbInt			= 0x00001000U,		/* set when GmacMmcTxBcFramesGb	counter reaches half of max value		*/
	GmacMmcTxMcFramesGbInt			= 0x00000800U,		/* set when GmacMmcTxMcFramesGb	counter reaches half of max value		*/
	GmacMmcTxUcFramesInt			= 0x00000400U,		/* set when GmacMmcTxUcFramesGb counter reaches half of max value		*/
	GmacMmcTx1024OctInt				= 0x00000200U,		/* set when GmacMmcTx1024ToMaxOctetsGb counter reaches half of max value	*/
	GmacMmcTx512OctInt				= 0x00000100U,		/* set when GmacMmcTx512To1023OctetsGb counter reaches half of max value	*/
	GmacMmcTx256OctInt				= 0x00000080U,		/* set when GmacMmcTx256To511OctetsGb counter reaches half of max value		*/
	GmacMmcTx128OctInt				= 0x00000040U,		/* set when GmacMmcTx128To255OctetsGb counter reaches half of max value		*/
	GmacMmcTx65OctInt				= 0x00000020U,		/* set when GmacMmcTx65To127OctetsGb counter reaches half of max value		*/
	GmacMmcTx64OctInt				= 0x00000010U,		/* set when GmacMmcTx64OctetsGb	counter reaches half of max value		*/
	GmacMmcTxMcFramesInt			= 0x00000008U,		/* set when GmacMmcTxMcFramesG counter reaches half of max value		*/
	GmacMmcTxBcFramesInt			= 0x00000004U,		/* set when GmacMmcTxBcFramesG counter reaches half of max value		*/
	GmacMmcTxFrameGbInt				= 0x00000002U,		/* set when GmacMmcTxFrameCountGb counter reaches half of max value		*/
	GmacMmcTxOctetGbInt				= 0x00000001U,		/* set when GmacMmcTxOctetCountGb counter reaches half of max value		*/
};

/**
 * The Low level function to read register contents from Hardware.
 * 
 * @param[in] pointer to the base of register map  
 * @param[in] Offset from the base
 * \return	Returns the register contents 
 */
static __inline uint32_t  synopGMACReadReg(uint32_t *RegBase, uint32_t RegOffset)
{

  volatile uint32_t *addr = (volatile uint32_t *)((uint32_t )RegBase + RegOffset);
  uint32_t data = *(addr);
  return data;
}

/**
 * The Low level function to write to a register in Hardware.
 * 
 * @param[in] pointer to the base of register map  
 * @param[in] Offset from the base
 * @param[in] Data to be written 
 * \return	void 
 */
static __inline void synopGMACWriteReg(uint32_t *RegBase, uint32_t RegOffset, uint32_t RegData)
{
  volatile uint32_t *addr = (volatile uint32_t *)((uint32_t )RegBase + RegOffset);
	//printf(" Reg_write: 0x%x, 0x%x\n", addr, RegData);
  *addr = RegData;
  return;
}

/**
 * The Low level function to set bits of a register in Hardware.
 * 
 * @param[in] pointer to the base of register map  
 * @param[in] Offset from the base
 * @param[in] Bit mask to set bits to logical 1 
 * \return	void 
 */
static __inline void synopGMACSetBits(uint32_t *RegBase, uint32_t RegOffset, uint32_t BitPos)
{
  volatile uint32_t *addr = (volatile uint32_t *)((uint32_t )RegBase + RegOffset);
  uint32_t data = *addr;
  data |= BitPos; 
//	TR("%s !!!!!!!!!!!!! RegOffset = 0x%08x RegData = 0x%08x\n", __FUNCTION__, RegOffset, data );
   *addr = data;
//	TR("%s !!!!!!!!!!!!! RegOffset = 0x%08x RegData = 0x%08x\n", __FUNCTION__, RegOffset, data );
  return;
}


/**
 * The Low level function to clear bits of a register in Hardware.
 * 
 * @param[in] pointer to the base of register map  
 * @param[in] Offset from the base
 * @param[in] Bit mask to clear bits to logical 0 
 * \return	void 
 */
static __inline void synopGMACClearBits(uint32_t *RegBase, uint32_t RegOffset, uint32_t BitPos)
{
  volatile uint32_t *addr = (volatile uint32_t *)((uint32_t )RegBase + RegOffset);
  uint32_t data = *addr;
  data &= (~BitPos); 
//	TR("%s !!!!!!!!!!!!!! RegOffset = 0x%08x RegData = 0x%08x\n", __FUNCTION__, RegOffset, data );
  *addr = data;
//	TR("%s !!!!!!!!!!!!! RegOffset = 0x%08x RegData = 0x%08x\n", __FUNCTION__, RegOffset, data );
  return;
}

/**
 * The Low level function to Check the setting of the bits.
 * 
 * @param[in] pointer to the base of register map  
 * @param[in] Offset from the base
 * @param[in] Bit mask to set bits to logical 1 
 * \return	returns TRUE if set to '1' returns FALSE if set to '0'. Result undefined there are no bit set in the BitPos argument.
 * 
 */
static __inline bool synopGMACCheckBits(uint32_t *RegBase, uint32_t RegOffset, uint32_t BitPos)
{
  volatile uint32_t * addr = (volatile uint32_t *)((uint32_t )RegBase + RegOffset);
  uint32_t data = * addr;
  data &= BitPos; 
  if(data)	return 1;
  else		return 0;

}

/**********************************************************
 * Common functions
 **********************************************************/
int32_t	 synopGMAC_set_mdc_clk_div(synopGMACdevice const * gbl_mac_dev,uint32_t clk_div_val);
uint32_t synopGMAC_get_mdc_clk_div(synopGMACdevice const * gbl_mac_dev);
int32_t	 synopGMAC_phy_loopback(synopGMACdevice * gbl_mac_dev, bool loopback);
int32_t	 synopGMAC_read_version (synopGMACdevice * gbl_mac_dev) ;
int32_t	 synopGMAC_reset (synopGMACdevice const * gbl_mac_dev ); 
int32_t	 synopGMAC_dma_bus_mode_init(synopGMACdevice const * gbl_mac_dev, uint32_t init_value );

void synopGMAC_rx_own_enable(void);
void synopGMAC_retry_enable(void);
void synopGMAC_pad_crc_strip_disable(void);
void synopGMAC_back_off_limit(uint32_t value);
void synopGMAC_deferral_check_disable(void);

void synopGMAC_frame_filter_enable(void);
void synopGMAC_hash_perfect_filter_enable(void);
void synopGMAC_src_addr_filter_disable(void);
void synopGMAC_dst_addr_filter_normal(void);
void synopGMAC_set_pass_control(uint32_t passcontrol);
void synopGMAC_broadcast_enable(void);
void synopGMAC_multicast_disable(void);
void synopGMAC_multicast_hash_filter_disable(void);
void synopGMAC_unicast_hash_filter_disable(void);

int32_t	 synopGMAC_mac_init(synopGMACdevice const * gbl_mac_dev);
int32_t	 synopGMAC_check_phy_init (synopGMACdevice * gbl_mac_dev);
int32_t	 synopGMAC_set_mac_addr( UCHAR const *MacAddr);
int32_t	 synopGMAC_get_mac_addr(synopGMACdevice *gbl_mac_dev, uint32_t MacHigh, uint32_t MacLow, UCHAR *MacAddr);
int32_t	 synopGMAC_attach (synopGMACdevice * gbl_mac_dev, uint32_t macBase, uint32_t dmaBase, uint32_t phyBase); 
void synopGMAC_tx_desc_init_ring(s_tx_norm_desc *desc);

void synopGMAC_set_desc_sof(DmaDesc * desc);
void synopGMAC_set_desc_eof(DmaDesc * desc);
bool synopGMAC_is_sof_in_rx_desc(DmaDesc * desc);
bool synopGMAC_is_eof_in_rx_desc(DmaDesc * desc);
bool synopGMAC_is_da_filter_failed(DmaDesc * desc);
bool synopGMAC_is_sa_filter_failed(DmaDesc * desc);
bool synopGMAC_is_tx_desc_owned_by_dma(s_tx_norm_desc const * desc);
bool synopGMAC_is_rx_desc_owned_by_dma(s_rx_norm_desc const * desc);
bool synopGMAC_tx_has_error(uint32_t status);
bool synopGMAC_is_rx_desc_empty(s_rx_norm_desc const * rxdesc);
bool synopGMAC_is_tx_aborted(uint32_t status);
bool synopGMAC_is_tx_carrier_error(uint32_t status);
uint32_t synopGMAC_get_tx_collision_count(uint32_t status);
uint32_t synopGMAC_is_exc_tx_collisions(uint32_t status);
bool synopGMAC_is_rx_frame_damaged(uint32_t status);
bool synopGMAC_is_rx_frame_collision(uint32_t status);
bool synopGMAC_is_rx_crc(uint32_t status);
bool synopGMAC_is_frame_dribbling_errors(uint32_t status);
bool synopGMAC_is_rx_frame_length_errors(uint32_t status);

int32_t synopGMAC_get_tx_qptr(synopGMACdevice * gbl_mac_dev, uint32_t * Status, uint32_t txch);
int32_t synopGMAC_set_tx_qptr(synopGMACdevice * gbl_mac_dev, uint32_t Buffer, uint32_t Length,uint32_t offload_needed, UCHAR txch, struct usb_request *req);
uint32_t synopGMAC_set_rx_qptr(synopGMACdevice * gbl_mac_dev, uint32_t Buffer);
int32_t synopGMAC_get_rx_qptr(synopGMACdevice * gbl_mac_dev, uint32_t * Status, uint32_t * Buffer, uint32_t * Length, UCHAR chno);
void synopGMAC_clear_interrupt(synopGMACdevice *gbl_mac_dev);
uint32_t synopGMAC_get_interrupt_mask(synopGMACdevice *gbl_mac_dev);
void synopGMAC_enable_interrupt(synopGMACdevice *gbl_mac_dev, uint32_t interrupts);
void synopGMAC_disable_interrupt(synopGMACdevice *gbl_mac_dev, uint32_t interrupts);
void synopGMAC_take_all_rx_desc_ownership(synopGMACdevice const * gbl_mac_dev);

/******Following APIs are valid only for Enhanced Descriptor from 3.50a release onwards*******/
bool synopGMAC_is_ext_status(synopGMACdevice * gmacdev, uint32_t status);			  
bool synopGMAC_ES_is_IP_header_error(synopGMACdevice * gmacdev, uint32_t ext_status);		  
bool synopGMAC_ES_is_rx_checksum_bypassed(synopGMACdevice * gmacdev, uint32_t ext_status);
bool synopGMAC_ES_is_IP_payload_error(synopGMACdevice * gmacdev, uint32_t ext_status);
/*******************PMT APIs***************************************/
void synopGMAC_power_down_enable(synopGMACdevice * gmacdev);
void synopGMAC_power_down_disable(synopGMACdevice * gmacdev);
void synopGMAC_enable_pmt_interrupt(synopGMACdevice * gmacdev);
void synopGMAC_disable_pmt_interrupt(synopGMACdevice * gmacdev);
void synopGMAC_magic_packet_enable(synopGMACdevice * gmacdev);
void synopGMAC_wakeup_frame_enable(synopGMACdevice * gmacdev);
void synopGMAC_pmt_unicast_enable(synopGMACdevice * gmacdev);
void synopGMAC_write_wakeup_frame_register(synopGMACdevice * gmacdev, uint32_t * filter_contents);
/*******************MMC APIs***************************************/
void synopGMAC_mmc_counters_stop(synopGMACdevice * gmacdev);
void synopGMAC_mmc_counters_resume(synopGMACdevice * gmacdev);
void synopGMAC_mmc_counters_set_selfclear(synopGMACdevice * gmacdev);
void synopGMAC_mmc_counters_reset_selfclear(synopGMACdevice * gmacdev);
void synopGMAC_mmc_counters_disable_rollover(synopGMACdevice * gmacdev);
void synopGMAC_mmc_counters_enable_rollover(synopGMACdevice * gmacdev);
uint32_t synopGMAC_read_mmc_counter(synopGMACdevice * gmacdev, uint32_t counter);
uint32_t synopGMAC_read_mmc_rx_int_status(synopGMACdevice * gmacdev);
uint32_t synopGMAC_read_mmc_tx_int_status(synopGMACdevice * gmacdev);



/*******************Ip checksum offloading APIs***************************************/
void synopGMAC_enable_rx_chksum_offload(synopGMACdevice * gmacdev);
void synopGMAC_disable_rx_Ipchecksum_offload(synopGMACdevice * gmacdev);
void synopGMAC_rx_tcpip_chksum_drop_enable(synopGMACdevice * gmacdev);
void synopGMAC_rx_tcpip_chksum_drop_disable(synopGMACdevice * gmacdev);
uint32_t  synopGMAC_is_rx_checksum_error(synopGMACdevice * gmacdev, uint32_t status);
bool synopGMAC_is_tx_ipv4header_checksum_error(synopGMACdevice * gmacdev, uint32_t status);
bool synopGMAC_is_tx_payload_checksum_error(synopGMACdevice * gmacdev, uint32_t status);
void synopGMAC_tx_checksum_offload_bypass(synopGMACdevice * gmacdev, DmaDesc * desc);
//void synopGMAC_tx_checksum_offload_ipv4hdr(synopGMACdevice * gmacdev, DmaDesc * desc);
//void synopGMAC_tx_checksum_offload_tcponly(synopGMACdevice * gmacdev, DmaDesc * desc);
void synopGMAC_tx_checksum_offload_tcp_pseudo(synopGMACdevice * gmacdev, DmaDesc * desc);

void tc9560INTC_enable_rxtx_interrupt(CPU_INT32U channels);

void DWC_ETH_QOS_mmc_read(struct DWC_ETH_QOS_mmc_counters * mmc);
void DWC_ETH_QOS_mmc_dump(struct DWC_ETH_QOS_mmc_counters const * mmc);


void tc9560_init_network(void);
/**********************************from net_dev_neu.c************************************/
extern void USB_ISR(void);
extern volatile uint32_t malloc_free_counter[4];
extern volatile UCHAR NetDev_RX_callback[4];

/************************************function declaration********************************/
static void	 tc9560INTC_disable_rxtx_interrupt(CPU_INT32U channels);
static void	 TC9560_RXCH_Handler(CPU_INT08U  dmachno );
static void	 TC9560_TXCH_Handler(CPU_INT08U  dmachno );
static void	 TC9560_TXCH0_Handler(void);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
static void	 TC9560_TXCH1_Handler(void);
static void	 TC9560_RXCH1_Handler(void);
#endif
static void	 TC9560_TXCH2_Handler(void);
static void	 TC9560_TXCH3_Handler(void);
static void	 TC9560_TXCH4_Handler(void);
static void	 TC9560_RXCH0_Handler(void);
static void	 TC9560_RXCH2_Handler(void);
static void	 TC9560_RXCH3_Handler(void);
static void	 TC9560_RXCH4_Handler(void);
static void	 TC9560_RXCH5_Handler(void);
static void	 NetDev_RxDescInit (CPU_INT08U chan);
static void	 NetDev_TxDescInit (CPU_INT08U chan);
static void	 NetDev_Init(void);
static void	 NetDev_Rx(void);
static void	 NetDev_Tx (CPU_INT08U const * p_data, CPU_INT32U	size, CPU_INT08U   txchno, struct usb_request * req);


void  NetDev_Start (void);
void synop_handle_transmit_over (int32_t txdmach);
extern void (* global_isr_table[]) (void);
extern int32_t neu_ep_queue_in (uint8_t * buf_addr, uint8_t bulk_in_num, uint32_t size);
extern void neu_ep_queue_out (uint32_t req_num, const char_t * ep_name,uint8_t out_ep);
extern void DWC_ETH_QOS_get_all_hw_features (synopGMACdevice * gbl_mac_dev);
extern void DWC_ETH_QOS_mmc_setup (synopGMACdevice * gbl_mac_dev);
void  NetDev_MII_Rd	(CPU_INT08U phy_addr, CPU_INT08U  reg_addr, CPU_INT16U * p_data);
void  NetDev_MII_Wr	(CPU_INT08U phy_addr, CPU_INT08U  reg_addr, CPU_INT16U	data);
/*****************************************************************************************/
#endif															
