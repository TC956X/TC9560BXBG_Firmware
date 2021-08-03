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

/* =========================================================================
 * $File: //dwh/usb_iip/dev/software/dwc_common_port_2/dwc_os.h $
 * $Revision: 1.9 $
 * $Date: 2016/07/18 23:50:47 $
 * $Change: 1621695 $
 *
 * Synopsys Portability Library Software and documentation
 * (hereinafter, "Software") is an Unsupported proprietary work of
 * Synopsys, Inc. unless otherwise expressly agreed to in writing
 * between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product
 * under any End User Software License Agreement or Agreement for
 * Licensed Product with Synopsys or any supplement thereto. You are
 * permitted to use and redistribute this Software in source and binary
 * forms, with or without modification, provided that redistributions
 * of source code must retain this notice. You may not view, use,
 * disclose, copy or distribute this file or any information contained
 * herein except pursuant to this license grant from Synopsys. If you
 * do not agree with this notice, including the disclaimer below, then
 * you are not authorized to use the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS"
 * BASIS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE HEREBY DISCLAIMED. IN NO EVENT SHALL
 * SYNOPSYS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * ========================================================================= */
#ifndef __DWC_OS_H__
#define __DWC_OS_H__

#include "includes.h"
#include "neu_os.h"
#include "dwc_otg_dbg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @file
 *
 * DWC portability library, low level os-wrapper functions
 *
 */

/* These basic types need to be defined by some OS header file or custom header
 * file for your specific target architecture.
 *
 * uint8_t, int8_t, uint16_t, int16_t, uint32_t, int32_t, uint64_t, int64_t
 *
 * Any custom or alternate header file must be added and enabled here.
 */

#if defined(DWC_FREEBSD) || defined(DWC_NETBSD)
# include <os_dep.h>
#endif


/** @name Primitive Types and Values */

/** We define a boolean type for consistency.  Can be either YES or NO */

#define YES  1
#define NO   0

/** @name Error Codes */
#define DWC_E_INVALID		1001
#define DWC_E_NO_MEMORY		1002
#define DWC_E_NO_DEVICE		1003
#define DWC_E_NOT_SUPPORTED	1004
#define DWC_E_TIMEOUT		1005
#define DWC_E_BUSY			1006
#define DWC_E_AGAIN			1007
#define DWC_E_RESTART		1008
#define DWC_E_ABORT			1009
#define DWC_E_SHUTDOWN		1010
#define DWC_E_NO_DATA		1011
#define DWC_E_DISCONNECT	2000
#define DWC_E_UNKNOWN		3000
#define DWC_E_NO_STREAM_RES	4001
#define DWC_E_COMMUNICATION	4002
#define DWC_E_OVERFLOW		4003
#define DWC_E_PROTOCOL		4004
#define DWC_E_IN_PROGRESS	4005
#define DWC_E_PIPE			4006
#define DWC_E_IO			4007
#define DWC_E_NO_SPACE		4008

/** @name Tracing/Logging Functions
 *
 * These function provide the capability to add tracing, debugging, and error
 * messages, as well exceptions as assertions.  The WUDEV uses these
 * extensively.  These could be logged to the main console, the serial port, an
 * internal buffer, etc.  These functions could also be no-op if they are too
 * expensive on your system.  By default undefining the DEBUG macro already
 * no-ops some of these functions. */
 

/** @name Byte Ordering
 * The following functions are for conversions between processor's byte ordering
 * and specific ordering you want.
 */

/** Allocates a block of memory and zeroes its contents. */
extern void *__DWC_ALLOC(void const *mem_ctx, uint32_t size);

/** Frees a previously allocated buffer. */
extern void __DWC_FREE(void const *mem_ctx, void *addr);


#define DWC_ALLOC(_size_) __DWC_ALLOC(NULL, _size_)
#define DWC_FREE(_addr_) __DWC_FREE(NULL, _addr_)

#define dwc_alloc(_ctx_,_size_) DWC_ALLOC(_size_)
#define dwc_free(_ctx_,_addr_) DWC_FREE(_addr_)

/** @name Memory and String Processing */

/** memset() clone */
extern void * DWC_MEMSET( void * dest, const uint8_t byte, const uint32_t size);
#define dwc_memset DWC_MEMSET

/** memcpy() clone */
extern void * DWC_MEMCPY(void * dest, void const *src, uint32_t size);
#define dwc_memcpy DWC_MEMCPY

/** memmove() clone */
extern void * DWC_MEMMOVE(void * const dest, const void * const src, const  uint32_t size);
#define dwc_memmove DWC_MEMMOVE

/** memcmp() clone */
extern int DWC_MEMCMP(const void * const m1, const void * const m2, const  uint32_t size);
#define dwc_memcmp DWC_MEMCMP

/** strcmp() clone */
extern int DWC_STRCMP(const void * const s1, const void * const s2);
#define dwc_strcmp DWC_STRCMP

/** strncmp() clone */
extern int DWC_STRNCMP(const void * const s1, const void * const s2, const uint32_t size);
#define dwc_strncmp DWC_STRNCMP

/** strlen() clone, for NULL terminated ASCII strings */
extern uint32_t DWC_STRLEN(char_t const *str);
#define dwc_strlen DWC_STRLEN

/** strcpy() clone, for NULL terminated ASCII strings */
extern char_t * DWC_STRCPY(char_t * to, const char_t * from);
#define dwc_strcpy DWC_STRCPY

/** Microsecond delay.
 *
 * @param usecs  Microseconds to delay.
 */
extern void DWC_UDELAY(uint32_t usecs);
#define dwc_udelay DWC_UDELAY

extern void udelay(uint32_t usecs);

/** Millisecond delay.
 *
 * @param msecs  Milliseconds to delay.
 */
extern void DWC_MDELAY(uint32_t msecs);
#define dwc_mdelay DWC_MDELAY

/** Non-busy waiting.
 * Sleeps for specified number of milliseconds.
 *
 * @param msecs Milliseconds to sleep.
 */
extern void DWC_MSLEEP(uint32_t msecs);
#define dwc_msleep DWC_MSLEEP

typedef struct cb_handler{
    cb_function handler;
    void* dev;
}cb_handler_t;

#ifdef __cplusplus
}
#endif

#endif /* _DWC_OS_H_ */
