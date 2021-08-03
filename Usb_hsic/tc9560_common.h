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
 
#ifndef	 __TC9560_COMMON_H__
#define	 __TC9560_COMMON_H__
typedef int						s32;
typedef short					s16;
typedef char					s8;
typedef unsigned long			u64;
typedef unsigned int			u32;
typedef unsigned short			u16;
typedef unsigned char			u8;
typedef volatile unsigned int	r32;
typedef volatile unsigned long	r64;
typedef volatile unsigned short	r16;
typedef volatile unsigned char	r8;
typedef char					char_t;
typedef void					NET_ERR;
typedef void					NET_IF;
typedef unsigned				USIGN;
typedef unsigned long			ULONG;
typedef unsigned short			USHORT;
typedef unsigned char			UCHAR;
typedef signed                  SIGN;
typedef signed int				CPU_INT32S;
typedef unsigned char			CPU_INT08U;
typedef unsigned int			CPU_INT32U;
typedef unsigned short			CPU_INT16U;
typedef unsigned char			CPU_BOOLEAN;
typedef unsigned long long		ULONG_LONG;
typedef unsigned int			uint32_t;
typedef unsigned char			uint8_t;
typedef unsigned short			uint16_t;
typedef signed int				int32_t;
typedef unsigned long long		uint64_t;
typedef int						gfp_t;
typedef uint8_t					dwc_bool_t;
typedef unsigned long long		dma_addr_t;
typedef	 CPU_BOOLEAN			bool;

/** Type for the 'flags' argument to spinlock funtions */
typedef unsigned long dwc_irqflags_t;

typedef void (*cb_function)(void*);

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The USB records contain some unaligned little-endian word
 * components.	The U[SG]ETW macros take care of both the alignment
 * and endian problem and should always be used to access non-byte
 * values.
 */
typedef uint8_t uByte;
typedef uint8_t uWord[2];
typedef uint8_t uDWord[4];

typedef signed char int8_t;
typedef unsigned short uid16_t;
typedef unsigned int dwc_dma_t;
#if 1
typedef int ssize_t;
#endif

#ifdef __cplusplus
}
#endif


#define NET_DEV_ERR_NONE				11000u
#define DEF_ENABLED						1
#define DEF_DISABLED					0
#define ENABLED							0xFFU
#define DISABLED						0x00U
typedef enum emac_dma_channels
{
	EMACDMA_TX_CH0 = 1,
	EMACDMA_TX_CH1 = 2,
	EMACDMA_TX_CH2 = 3,
	EMACDMA_TX_CH3 = 4,
	EMACDMA_TX_CH4 = 5,
	EMACDMA_RX_CH0 = 6,
	EMACDMA_RX_CH1 = 7,
	EMACDMA_RX_CH2 = 8,
	EMACDMA_RX_CH3 = 9,
	EMACDMA_RX_CH4 = 10,
	EMACDMA_RX_CH5 = 11
}EMAC_DMA_CHANNELS;
#define NET_DEV_ISR_TYPE_RX_CH0			0xF0u
#define NET_DEV_ISR_TYPE_RX_CH1			0xF1u
#define NET_DEV_ISR_TYPE_RX_CH2			0xF2u
#define NET_DEV_ISR_TYPE_RX_CH3			0xF3u
#define NET_DEV_ISR_TYPE_RX_CH4			0xF4u
#define NET_DEV_ISR_TYPE_RX_CH5			0xF5u
#define NET_DEV_ISR_TYPE_TX_CH0			0xF6u
#define NET_DEV_ISR_TYPE_TX_CH1			0xF7u
#define NET_DEV_ISR_TYPE_TX_CH2			0xF8u
#define NET_DEV_ISR_TYPE_TX_CH3			0xF9u
#define NET_DEV_ISR_TYPE_TX_CH4			0xFAu
#define TC9560_USE_TSB_DMA_WRAPPER			DEF_ENABLED	  // Should be always ENABLED
#define TC9560_FPGA_BOARD					DEF_DISABLED   // FPGA or Silicon Evaluation board
/* Also set CPU_SYSFREQ_TC9560 to 187000000 for Silicon in cpu_cfg.h */
#define TC9560_EMAC_VERIFICATION			DEF_DISABLED	 // Verify EMAC TX RX paths
#define TC9560_WRAP_EMAC_VERIFICATION		DEF_DISABLED   // Verify TAEC WRAP EMAC TX RX paths
#define TC9560_INT_POLLING					DEF_DISABLED
#define TC9560_MMC_DUMP					DEF_DISABLED

#define TC9560_M3_STANDALONE				DEF_ENABLED

#define TC9560_MEMORY_TEST					DEF_DISABLED
#define TC9560_UART_RX_TEST				DEF_DISABLED
#define TC9560_GPIO_TEST					DEF_DISABLED
#define TC9560_GDMA_TEST					DEF_DISABLED
#define TC9560_TX_BW_TEST					DEF_DISABLED

/***************************************************************************** *
* NOTE: 
* The Below macro is used to Enable or Diable the All Untested features which includes Interrupts, Configuration based code
* Configuration is done in initialization under the API "dwc_otg_setup_params"
* If you want to change any configuration please check code under "HSIC_FW_UNTESTED_FEATURE"

* HSIC_FW_UNTESTED_FEATURE is ENABLED then all the code under this one is enabled & compiled
* HSIC_FW_UNTESTED_FEATURE is DISABLED then all the code under this one is diabled.

****************************************************************************** */

#define HSIC_FW_UNTESTED_FEATURE        ENABLED

#if (TC9560_M3_STANDALONE == DEF_ENABLED)	
	#define M3_MAX_QUEUE				4
	#define RX_NO_CHAN					6
	#define TX_NO_CHAN					5
	#define M3_TX_DMACH					1	// 0 0r 1
#else
	#define M3_RX0_DMACH				0	// Rx Channel 0 is always 0
	#define M3_RX_2ND_DMACH				1	// This is used for second RX Channel
	#define M3_TX_DMACH					1	// 0 0r 1

	#define M3_MAX_QUEUE				1
	#define RX_NO_CHAN					(M3_RX_2ND_DMACH + 1)
	#define TX_NO_CHAN					(M3_TX_DMACH + 1)  
#endif

#define TX_DESC_CNT						8
#define RX_DESC_CNT						8	

//extern unsigned int UART_CLK_F, tstdiv;
enum {
	RX_QUEUE0 = 0,
	RX_QUEUE1,
	RX_QUEUE2,
	RX_QUEUE3,
	RX_QUEUE4,
	RX_QUEUE5,
	RX_QUEUE6,
	RX_QUEUE7
};
enum {
	TX_QUEUE0 = 0,
	TX_QUEUE1,
	TX_QUEUE2,
	TX_QUEUE3,
	TX_QUEUE4
};
#define TC9560_REG_BASE					0x40000000
/* EMAC registers */
#define TC9560_EMAC_REG_BASE				0x4000A000

#define MSI_SEND_TRIGGER_OFFS			0x209C
/* Interrupt registers */
#define TC9560_INTC_REG_BASE				0x40008000
#define INTSTATUS_OFFS					0x0000
#define GDMASTATUS_OFFS					0x0008
#define MACSTATUS_OFFS					0x000C
#define TDMSTATUS_OFFS					0x0010
#define EXTINTFLG_OFFS					0x0014
#define I2CSPIINTFLG_OFFS				0x001C
#define INTMCUMASK0_OFFS				0x0020
#define INTMCUMASK1_OFFS				0x0024
#define INTMCUMASK2_OFFS				0x0028
#define EXTINTCFG_OFFS					0x004C
#define MCUFLG_OFFS						0x0054
#define EXT_FLG_OFFS					0x0058
#define	INTC_Mask_TXCH0					(0x1 << 11)
#define	INTC_Mask_TXCH1					(0x1 << 12)
#define	INTC_Mask_TXCH2					(0x1 << 13)
#define	INTC_Mask_TXCH3					(0x1 << 14)
#define	INTC_Mask_TXCH4					(0x1 << 15)
#define	INTC_Mask_RXCH0					(0x1 << 16)
#define	INTC_Mask_RXCH1					(0x1 << 17)
#define	INTC_Mask_RXCH2					(0x1 << 18)
#define	INTC_Mask_RXCH3					(0x1 << 19)
#define	INTC_Mask_RXCH4					(0x1 << 20)
#define	INTC_Mask_RXCH5					(0x1 << 21)

#if (TC9560_M3_STANDALONE == DEF_ENABLED)
	#define INTC_Mask_RXCHS				(0x3F << 16)	 // ALL
	#define INTC_Mask_TXCH				(0x1F << 11)	 // ALL
#else
	#define INTC_Mask_RXCHS				(INTC_Mask_RXCH0 | INTC_Mask_RXCH1)	 
	#if (M3_TX_DMACH == 1)
	#define INTC_Mask_TXCH				(INTC_Mask_TXCH1)
	#else
	#define INTC_Mask_TXCH				(INTC_Mask_TXCH0)
	#endif
#endif
	
static	void hw_reg_write32 (uint32_t  addr_base, uint32_t	offset, uint32_t  val)
{
	volatile uint32_t  *addr = (volatile uint32_t  *)(uint32_t	*)(addr_base + offset);
	*addr = val;
}
static	uint32_t  hw_reg_read32 (uint32_t  addr_base, uint32_t	offset)
{
	volatile uint32_t  * addr = (volatile uint32_t  *)(uint32_t	*)(addr_base + offset);
	return (* addr);
}

static	void hw_write32(volatile uint32_t  * addr,uint32_t  val)
{ 
	* addr = val;
}
static	void hw_write16(volatile unsigned short * addr,uint16_t val)
{ 
	* addr = val;
}
static	void hw_write8(volatile uint8_t * addr, uint8_t val)
{ 
	* addr = val;
}

static	uint32_t clearbit(uint32_t input_value, uint32_t position)
{
	return (input_value & (~ (1 << position)));
}

static	uint32_t setbit(uint32_t  input_value, uint32_t position)
{
	return (input_value | (1 << position));
}

static	void __DMB(void) {__asm volatile ("dmb");}

//target's valid data is right-aligned (bit 0 to bit bit_count-1 contains valid data to set)
//input_value is the number that will be operated on, target contains the bits that will be set at specified location
static	uint32_t  setrange(uint32_t input_value, uint32_t lsb_position, uint32_t bit_count, uint32_t target)
{
	uint32_t  mask = 0xffffffff, one = 0x1, ctr;
	target <<= lsb_position;
	one <<= lsb_position;
	for(ctr = 0; ctr < bit_count; ctr++)
	{
		mask ^= one;
		one <<= 1;
	}
	input_value &= mask;
	return (input_value | target);
}


#if ( TC9560_MEMORY_TEST == DEF_ENABLED )
//CPU_INT32U tc9560_dmem_test(void);
#endif

#if (TC9560_GPIO_TEST == DEF_ENABLED)
void taec_gpio0_config_output(uint32_t data);
void taec_gpio0_output_data(uint32_t  data);
#endif

#if (TC9560_GDMA_TEST == DEF_ENABLED)
/* test_type 1: SRAM --> SRAM */
/* test_type 2: DRAM --> SRAM --> SRAM --> DRAM --> DRAM */
void gdma_test(uint8_t test_type);
#endif

#if ( TC9560_EMAC_VERIFICATION == DEF_ENABLED )
int32_t emac_tx_rx(void);
#endif

#if ( TC9560_WRAP_EMAC_VERIFICATION == DEF_ENABLED )
int32_t wrap_emac_tx_rx(void);
#endif
#if 1 // EMAC_INIT TEST
void emac_init_test(void);
#endif
extern int32_t test_value;
extern void neu_hs_bulk_out1_desc_defn(void);
extern void neu_hs_bulk_out2_desc_defn(void);
extern void neu_hs_bulk_out2_desc_defn(void);
extern void neu_hs_bulk_out3_desc_defn(void);
extern void neu_hs_bulk_out4_desc_defn(void);
extern void neu_hs_bulk_in5_desc_defn(void);
extern void neu_hs_bulk_in6_desc_defn(void);
extern void neu_hs_bulk_in7_desc_defn(void);
extern void neu_hs_bulk_in8_desc_defn(void);
extern void dwc_otg_pcd_ops_defn(void);
extern void neu_driver_call(void);
extern void neu_intf_desc_call(void);
#endif
