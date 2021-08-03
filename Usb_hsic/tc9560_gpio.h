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
*       25-Oct-2015 : Initial
*/
#ifndef __TC9560_GPIO_H__
#define __TC9560_GPIO_H__

#include "tc9560_common.h"

#define  TC9560_GPIO_REG_BASE            (uint32_t)(0x40001200) 
#define  TC9560_GPIO_INPUT0              (* (uint32_t *)(TC9560_GPIO_REG_BASE + 0x0000))
#define  TC9560_GPIO_INPUT1              (* (uint32_t *)(TC9560_GPIO_REG_BASE + 0x0004))
#define  TC9560_GPIO_INPUT_ENABLE0       (* (uint32_t *)(TC9560_GPIO_REG_BASE + 0x0008)) /* 0-31 */
#define  TC9560_GPIO_INPUT_ENABLE1       (* (uint32_t *)(TC9560_GPIO_REG_BASE + 0x000C)) /* 32-63 */
#define  TC9560_GPIO_OUTPUT0             (* (uint32_t *)(TC9560_GPIO_REG_BASE + 0x0010))
#define  TC9560_GPIO_OUTPUT1             (* (uint32_t *)(TC9560_GPIO_REG_BASE + 0x0014))


#define	GPIO0	(1 << 0)
#define	GPIO1	(1 << 1)
#define	GPIO2	(1 << 2)
#define	GPIO3	(1 << 3)
#define	GPIO4	(1 << 4)
#define	GPIO5	(1 << 5)
#define	GPIO6	(1 << 6)
#define	GPIO7	(1 << 7)
#define	GPIO8	(1 << 8)
#define	GPIO9	(1 << 9)
#define	GPIO10	(1 << 10)
#define	GPIO11	(1 << 11)
#define	GPIO12	(1 << 12)
#define	GPIO13	(1 << 13)
#define	GPIO14	(1 << 14)
#define	GPIO15	(1 << 15)
#define	GPIO16	(1 << 16)
#define	GPIO17	(1 << 17)
#define	GPIO18	(1 << 18)
#define	GPIO19	(1 << 19)
#define	GPIO20	(1 << 20)
#define	GPIO21	(1 << 21)
#define	GPIO22	(1 << 22)
#define	GPIO23	(1 << 23)
#define	GPIO24	(1 << 24)
#define	GPIO25	(1 << 25)
#define	GPIO26	(1 << 26)
#define	GPIO27	(1 << 27)
#define	GPIO28	(1 << 28)
#define	GPIO29	(1 << 29)
#define	GPIO30	(1 << 30)
#define	GPIO31	(1 << 31)
#define	GPIO32	(1 << 0)
#define	GPIO33	(1 << 1)
#define	GPIO34	(1 << 2)
#define	GPIO35	(1 << 3)
#define	GPIO36	(1 << 4)

void tc9560_gpio0_config_output(uint32_t data);
void tc9560_gpio0_output_data_high(uint32_t data);
void tc9560_gpio0_output_data_low(uint32_t data);
void tc9560_gpio1_config_output(uint32_t data);
void tc9560_gpio1_output_data_high(uint32_t data);
void tc9560_gpio1_output_data_low(uint32_t data);
void tc9560_gpio0_config_input(uint32_t data);
void tc9560_gpio0_input_data(uint32_t * data);
void tc9560_gpio1_config_input(uint32_t data);
void tc9560_gpio1_input_data(uint32_t * data);
void TC9560_Ser_Printf(char * format, ...);	/*for testing */
#endif 

