/**
 ******************************************************************************
 *
 * @file       led.h
 * @author     Stephen Caudle Copyright (C) 2010.
 * @brief      Main header.
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


#ifndef LED_H
#define LED_H

/**
 * @brief  Configures the LED
 * @param  None
 * @retval None
 */
int LED_Configuration(void);

/**
 * @brief  Sets a message on a line of the LED display
 * @param  line A zero-based index of the line to set
 * @param  message The message to display
 * @retval None
 */
int LED_SetLine(uint8_t line, char *message);

/**
 * @brief  Redraws the LED display
 * @param  None
 * @retval None
 */
int LED_Refresh();

#endif /* LED_H */

