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

/*
 *********************************************************************************************************
 *
 * Filename      : tc9560_uart.c
 * Programmer(s) : WZ
 *                 
 *********************************************************************************************************
 */

#include <stdlib.h>
#include <stdarg.h>
#include "tc9560_common.h"
#include "tc9560_uart.h"
#include "tc9560_reg_define.h"

/* Private define ------------------------------------------------------------*/
char_t 		gSIOTxBuffer[BUFFER_SIZE] = { 0U };
uint8_t 	gSIORdIndex = 0U;
uint8_t 	gSIOWrIndex = 0U;
uint8_t 	fSIO_INT = 0U;
/*unsigned char fSIOTxOK = NO;*/

/*
*==============================================================================
*SUPPORT FUNCTIONS
*==============================================================================
*------------------------------------------------------------------------------
*/

void uart_initialize(uint32_t baudrate)
{
	uint32_t  	mclk_freq;
	uint32_t 	divider;
	uint32_t	remainder;
	uint32_t	fraction;
    mclk_freq = 62500000; 										
	hw_reg_write32(UART_BASE, UART_ICR, 0x07ff);	/* Clear all interrupt status */
	divider = mclk_freq / (16 * baudrate);
	remainder = mclk_freq % (16 * baudrate);
	fraction = ((8 * remainder) / baudrate) >> 1;
	fraction += ((8 * remainder) / baudrate) & 1;
		
	hw_reg_write32(UART_BASE, UART_IBRD, divider);
	hw_reg_write32(UART_BASE, UART_FBRD, fraction);

	/* Set N, 8, 1, FIFO enable */
	hw_reg_write32(UART_BASE, UART_LCRH, (LCRH_WLEN8 | LCRH_FEN));		/* FIFO enabled*/

	/* Enable UART */
	hw_reg_write32(UART_BASE, UART_CR, (CR_RXE | CR_TXE | CR_UARTEN));		

	/* Enable TX/RX interrupt */
	hw_reg_write32(UART_BASE, UART_IMSC, (IMSC_RX | IMSC_TX | IMSC_RT)); 
	
	/* Pinmux control */
	REG_WR(NFUNCEN1, (REG_RD(NFUNCEN1)| (1 << 7)));
}

void uart_send_data(uint8_t const *data, uint32_t count)
{
	while(count != 0U)
    {
		while ((hw_reg_read32(UART_BASE, UART_FR) & FR_TXFF) != 0) {
			;
		}
		hw_reg_write32(UART_BASE, UART_DR, *data++);
		count--;
    }
}

int32_t uart_get_data(uint8_t *data, uint32_t count)
{
	uint8_t inp;
	while(count != 0U)
	{
		/*time_out = m3Ticks + UART_TIME_OUT;*/
		while( (hw_reg_read32(UART_BASE, UART_FR) & FR_RXFE) != 0 ) {
			; 
		}
		inp = (uint8_t)(hw_reg_read32(UART_BASE, UART_DR));
		*data++ = inp;
		while ((hw_reg_read32(UART_BASE, UART_FR) & FR_TXFF) != 0) {
			;
		}
		hw_reg_write32(UART_BASE, UART_DR, inp);	
		if(inp == 0x0D) {
			break;
		}
		count--;
   }
   return 0;
}

void  TC9560_Ser_Printf (char_t *format, ...)
{
	char_t  			buffer[255u + 1u];
    va_list   		vArgs;
	uint8_t 	len = 0;
    va_start(vArgs, format);
    vsprintf((char_t *)buffer, (char_t const *)format, vArgs);
    va_end(vArgs);
	while (buffer[len] != 0)
	{
		len++;
	}
    uart_send_data((uint8_t *) buffer, len--);
	(void)len;
}
