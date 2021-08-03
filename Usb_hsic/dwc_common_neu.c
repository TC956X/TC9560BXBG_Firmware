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

#include "dwc_os.h"
#include "dwc_otg_dbg.h"

void *DWC_MEMSET(void * const dest,const uint8_t byte,const uint32_t size)		
{
	return memset(dest,(int32_t) byte, size);									
}

void *DWC_MEMCPY(void *const dest,const void *const src,const  uint32_t size)
{
	return memcpy(dest, src, size);
}

void *DWC_MEMMOVE(void *const dest,const void *const src,const  uint32_t size)
{
	return memmove(dest, src, size);
}

int32_t DWC_MEMCMP(const void *const m1,const void *const m2,const  uint32_t size)
{
	return memcmp(m1, m2, size);
}

int32_t DWC_STRNCMP(const void *const s1,const void *const s2,const uint32_t size)
{
	return strncmp(s1, s2, size);
}

int32_t DWC_STRCMP(const void *const s1,const void *const s2)
{
	return strcmp(s1, s2);
}

uint32_t DWC_STRLEN(const char_t *const str)												
{
	return strlen(str);
}

char_t *DWC_STRCPY(char_t *const to,const char_t *from)
{
	return strcpy(to, from);
}


int32_t DWC_VSNPRINTF(char_t *str, int32_t size, char_t *format, va_list args)
{
	return vsnprintf(str, size, format, args);
}


void *__DWC_ALLOC(void const *mem_ctx, uint32_t size)
{
	void *buf = malloc((size_t)size);
	DBG_USB_Print(DBG_USB,"Malloc : A=%05x S=%d\r\n", buf, size);
	if (!buf) {
		buf=NULL;
		DBG_Warn_Print("__DWC_ALLOC error\r\n");
	}
	(void)mem_ctx;
	return buf;
}

void __DWC_FREE(void const *mem_ctx, void *addr)
{
	DBG_USB_Print(DBG_USB,"Free : A=%05x\r\n", addr);
	  free((void *)addr);
	  (void)mem_ctx;
}

/* Register Acess */

void DWC_MODIFY_REG32(volatile uint32_t *const reg, uint32_t const clear_mask, uint32_t const set_mask)		
{
	*reg = ((*(reg)) & ~clear_mask) | set_mask;
}

/* Timing */

void udelay(uint32_t usecs)
{
	while(usecs != 0)
	{
		usecs--;
	}		
}

void DWC_UDELAY(uint32_t const usecs)
{
	udelay(usecs);
}

void DWC_MDELAY(uint32_t const msecs)
{
	udelay(msecs * 1000);
}

void DWC_MSLEEP(uint32_t const msecs)
{
	udelay(msecs * 1000);
}
