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
 * Filename      : nt_uart.h
 * Programmer(s) : WZ
 *                 
 *********************************************************************************************************
 */
#ifndef __UART_H__
#define __UART_H__

#include "includes.h"
#include "usb.h"

#define UART_TIME_OUT 				1000 	// 1 second time out
#define UART_BASE					0x40007000

/* UART Registers Offset */
#define UART_DR						0x00
#define UART_RSR					0x04
#define UART_ECR					0x04
#define UART_FR						0x18
#define UART_IBRD					0x24
#define UART_FBRD					0x28
#define UART_LCRH					0x2c
#define UART_CR						0x30
#define UART_IMSC					0x38
#define UART_MIS					0x40
#define UART_ICR					0x44

/* Flag register */
#define FR_RXFE						0x10	/* Receive FIFO empty */
#define FR_TXFF						0x20	/* Transmit FIFO full */

/* Masked interrupt status register */
#define MIS_RX						0x10	/* Receive interrupt */
#define MIS_TX						0x20	/* Transmit interrupt */
#define MIS_RT        				0x40

/* Interrupt clear register */
#define ICR_RX						0x10	/* Clear receive interrupt */
#define ICR_TX						0x20	/* Clear transmit interrupt */
#define ICR_RT						0x40	/* Clear receive timeout interrupt */

/* Line control register (High) */
#define LCRH_WLEN8					0x60	/* 8 bits */
#define LCRH_FEN					0x10	/* Enable FIFO */

/* Control register */
#define CR_UARTEN					0x0001	/* UART enable */
#define CR_TXE						0x0100	/* Transmit enable */
#define CR_RXE						0x0200	/* Receive enable */
#define CR_LBE      				0x0080  /* Loopback enable for test!!!!!!!!!!!!!! */

/* Interrupt mask set/clear register */
#define IMSC_RX						0x10	/* Receive interrupt mask */
#define IMSC_TX						0x20	/* Transmit interrupt mask */
#define IMSC_RT						0x40	/* Receive timeout mask */

#define RX_FIFO_SIZE				64

/* Private define ------------------------------------------------------------*/
#define BUFFER_SIZE 				16U
#define SET    						0x01U           /* flag is set */
#define CLEAR  						0x00U           /* flag is cleared */
//#define UART_NO     				0x00U           /* Send finish NG */
//#define YES    					0x01U           /* Send finish OK */

/* external variables --------------------------------------------------------*/
extern char_t gSIOTxBuffer[];
extern uint8_t gSIORdIndex;
extern uint8_t gSIOWrIndex;
extern uint8_t fSIO_INT;
//extern unsigned char fSIOTxOK;

/* Exported functions ------------------------------------------------------- */
uint8_t send_char(uint8_t ch);
void uart_initialize(uint32_t baudrate);
void uart_send_data(uint8_t const * data, uint32_t count);
int32_t  uart_get_data (uint8_t * data, uint32_t count);
void  TC9560_Ser_Printf (char_t * format, ...);
#endif 
