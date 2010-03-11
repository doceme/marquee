/**
 ******************************************************************************
 *
 * @file       network.h
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


#ifndef NETWORK_H
#define NETWORK_H

/**
 * @brief  Configures the network
 * @param  None
 * @retval None
 */
int Network_Configuration(void);

/**
 * @brief  Sends a network command and optionally waits for a response
 * @param  command The command to send
 * @param  response If not NULL, the response string for which to wait
 *                  If NULL, returns without waiting for a response
 * @param  timeout A timeout in milliseconds for which to wait for the response
 *                 before returning. Waits indefinitely if set to 0.
 * @retval 0 if successful
 * @retval -ERR_PARAM if command is NULL
 * @retval -ERR_TIMEOUT if response was not received in timeout milliseconds
 * @retval -ERR_GENERIC if the command failed to be sent
 */
int Network_SendWait(char *command, char* response, uint32_t timeout);

/**
 * @brief  Sends a network command and returns the response string
 *         between the start character and the end character
 * @param  command The command to send
 * @param  start The character before the response string
 * @param  end The character after the response string
 * @param  response Pointer to the buffer for storing the response
 *                  This must be allocated by the caller.
 * @param  timeout A timeout in milliseconds for which to wait for the response
 *                 before returning. Waits indefinitely if set to 0.
 * @retval 0 if successful
 * @retval -ERR_PARAM if command is NULL
 * @retval -ERR_TIMEOUT if response was not received in timeout milliseconds
 * @retval -ERR_GENERIC if the command failed to be sent
 */
int Network_SendGetByChar(char *command, char start, char end, char* response, uint32_t timeout);

#endif /* NETWORK_H */

