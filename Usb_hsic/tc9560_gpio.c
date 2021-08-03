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

/*====================================================================
INCLUDE FILES
==================================================================== */
#include "tc9560_gpio.h"
/*====================================================================
FUNCTION DEFINITION
=================================================================== */
/*
*    Function    :  tc9560_gpio0_config_output(uiData)
*    Purpose     :  To set GPIO0 register pins (GPIO0 to GPIO31) as output pins
*    Input       :  uiData (GPIO pin number from GPIO0 to GPIO31)
*    Outputs     :  None
*    Return Value:  None
*    Limitations :  None
*/
void tc9560_gpio0_config_output(uint32_t data)
{
	uint32_t config; 
	config = ~(data);
	TC9560_GPIO_INPUT_ENABLE0	&=	config;    /* config as output */
}
/* End of tc9560_gpio0_config_output */

/*
*    Function    :  tc9560_gpio0_output_data_high(uiData)
*    Purpose     :  To set GPIO0 register pins (GPIO0 to GPIO31) output value high
*    Input       :  uiData (GPIO pin number from GPIO0 to GPIO31)
*    Outputs     :  None
*    Return Value:  None
*    Limitations :  None
*/
void tc9560_gpio0_output_data_high(uint32_t data)
{
	TC9560_GPIO_OUTPUT0	|=	data;    /* drive 1 on gpio pins[31:0]  */
}
/* End of tc9560_gpio0_output_data_high */

/*
*    Function    :  tc9560_gpio0_output_data_low(uiData)
*    Purpose     :  To set GPIO0 register pins (GPIO0-GPIO31) output value low
*    Inputs      :  uiData (GPIO pin number from GPIO0 to GPIO31)
*    Outputs     :  None
*    Return Value:  None
*    Limitations :  None
*/
void tc9560_gpio0_output_data_low(uint32_t data)
{
	uint32_t low_pin; 
	low_pin = ~(data);
	TC9560_GPIO_OUTPUT0	&=	low_pin;    /* drive 1 on gpio pins[31:0]  */
}
/* End of tc9560_gpio0_output_data_low */

/*
*    Function    :  tc9560_gpio1_config_output(uiData)
*    Purpose     :  To set GPIO1 register pins (GPIO32 to GPIO36) as output
*    Inputs      :  uiData (GPIO pin number from GPIO32 to GPIO36)
*    Outputs     :  None
*    Return Value:  None
*    Limitations :  None
*/
void tc9560_gpio1_config_output(uint32_t data)
{
	uint32_t config; 
	config = ~(data);
	TC9560_GPIO_INPUT_ENABLE1	&=	config;    /* config as output */
}
/* End of tc9560_gpio1_config_output */

/*
*    Function    :  tc9560_gpio1_output_data_high(uiData)
*    Purpose     :  To set GPIO1 register pins (GPIO32 to GPIO36) output value high
*    Inputs      :  uiData (GPIO pin number from GPIO32 to GPIO36)
*    Outputs     :  None
*    Return Value:  None
*    Limitations :  None
*/
void tc9560_gpio1_output_data_high(uint32_t data)
{
	TC9560_GPIO_OUTPUT1	|=	data;    /* drive 1 on gpio pins[31:0]  */
}
/* End of tc9560_gpio1_output_data_high */

/*
*    Function    :  tc9560_gpio1_output_data_low(uiData)
*    Purpose     :  To set GPIO1 register pins (GPIO32 to GPIO36) output value low
*    Inputs      :  uiData (GPIO pin number from GPIO32 to GPIO36)
*    Outputs     :  None
*    Return Value:  None
*    Limitations :  None
*/
void tc9560_gpio1_output_data_low(uint32_t data)
{
	uint32_t low_pin; 
	low_pin = ~(data);
	TC9560_GPIO_OUTPUT1	&=	low_pin;    /* drive 0 on gpio pins[31:0]  */
}
/* End of tc9560_gpio1_output_data_low */

/*
*    Function    :  tc9560_gpio0_config_input(uiData)
*    Purpose     :  To set set GPIO0 register pins (GPIO0 to GPIO31) as input
*    Inputs      :  uiData (GPIO pin number from GPIO0 to GPIO31)
*    Outputs     :  None
*    Return Value:  None
*    Limitations :  None
*/
void tc9560_gpio0_config_input(uint32_t data)
{
	TC9560_GPIO_INPUT_ENABLE0	|=	data;    /* config as input */
}
/* End of tc9560_gpio0_config_input */

/*
*    Function    :  tc9560_gpio0_input_data(uiData)
*    Purpose     :  To read GPIO0 register pins (GPIO0 to GPIO31) input value
*    Inputs      :  uiData - pointer to store GPIO input value
*    Outputs     :  None
*    Return Value:  None
*    Limitations :  None
*/
void tc9560_gpio0_input_data(uint32_t *data)
{
	*data = TC9560_GPIO_INPUT0;    /* read values of gpio0 register(GPIO0 to GPIO331)  */
	TC9560_Ser_Printf("gpio0 = 0x%x\n",TC9560_GPIO_INPUT0);
}
/* End of tc9560_gpio0_input_data */

/*
*    Function    :  tc9560_gpio1_config_input(uiData)
*    Purpose     :  To set GPIO1 register (GPIO32 to GPIO36) pins as input
*    Inputs      :  uiData (GPIO pin number from GPIO32 to GPIO36)
*    Outputs     :  None
*    Return Value:  None
*    Limitations :  None
*/
void tc9560_gpio1_config_input(uint32_t data)
{
	TC9560_GPIO_INPUT_ENABLE1	|=	data;    /* config as input  */
}
/* End of tc9560_gpio1_config_input */

/*
*    Function    :  tc9560_gpio1_input_data(uiData)
*    Purpose     :  To read GPIO1 register pins (GPIO32 to GPIO36) output value
*    Inputs      :  uiData - pointer to store GPIO1 register value
*    Outputs     :  None
*    Return Value:  None
*    Limitations :  None
*/
void tc9560_gpio1_input_data(uint32_t *data)
{
	*data = TC9560_GPIO_INPUT1;     /* read values of gpio1 register(GPIO32 to GPIO36)  */
	TC9560_Ser_Printf("gpio1 = 0x%x\n",TC9560_GPIO_INPUT1);
}
/* End of tc9560_gpio1_input_data */
