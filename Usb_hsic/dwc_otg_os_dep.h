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

#ifndef __DWC_OTG_OS_DEP_H__
#define __DWC_OTG_OS_DEP_H__

#include "tc9560_common.h"

#ifndef LM_INTERFACE
#define LM_INTERFACE
#endif

/** The OS page size */
#define DWC_OS_PAGE_SIZE	PAGE_SIZE


typedef struct os_dependent {
	/** Base address returned from ioremap() */
	void * base;

	/** Register offset for Diagnostic API */
	uint32_t reg_offset;

#ifdef LM_INTERFACE
	struct lm_device * lmdev;
#endif
} os_dependent_t;

#endif 
