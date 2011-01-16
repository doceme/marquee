/**
 ******************************************************************************
 *
 * @file       remote.h
 * @author     Stephen Caudle Copyright (C) 2010
 * @brief      Remote header
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

#ifndef REMOTE_H
#define REMOTE_H

enum remote_protocol_t
{
	REMOTE_PROTOCOL_RC5,
	REMOTE_PROTOCOL_RC6
};

struct remote_button_t
{
	enum remote_protocol_t protocol;
	uint8_t mode;
	uint8_t trailer;
	uint8_t address;
	uint8_t data;
};

/**
 * @brief  Configures the remote
 * @param  None
 * @retval None
 */
int Remote_Configuration(void);

/**
 * @brief  Returns the button queue for press events
 * @param  None
 * @retval None
 */
xQueueHandle Remote_GetButtonQueue();


#endif /* REMOTE_H */
