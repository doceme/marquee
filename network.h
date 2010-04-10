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

/* Definitions */
#define NETWORK_SSID_NAME_MAX_LENGTH 33
#define NETWORK_MAC_ADDRESS_MAX_LENGTH 18

/* Enumerations */
typedef enum NetworkWlanSecurityType_t
{
	NETWORK_WLAN_SECURITY_TYPE_NONE,
	NETWORK_WLAN_SECURITY_TYPE_WEP64,
	NETWORK_WLAN_SECURITY_TYPE_WEP128,
	NETWORK_WLAN_SECURITY_TYPE_WPA,
	NETWORK_WLAN_SECURITY_TYPE_WPA2
} NetworkWlanSecurityType_t;

typedef enum NetworkWlanWPAStatus_t
{
	NETWORK_WLAN_WPA_STATUS_NOTCOMPLETED,
	NETWORK_WLAN_WPA_STATUS_COMPLETED
} NetworkWlanWPAStatus_t;

/* Structures */
typedef struct NetworkWlanConnection_t
{
	char ssid[NETWORK_SSID_NAME_MAX_LENGTH];
	char bssid[NETWORK_MAC_ADDRESS_MAX_LENGTH];
	NetworkWlanSecurityType_t securityType;
	NetworkWlanWPAStatus_t wpaStatus;
	uint8_t channel;
	uint8_t snr;
} NetworkWlanConnection_t;

typedef struct NetworkDateTime_t
{
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
} NetworkDateTime_t;

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

/**
 * @brief  Sends a network command and returns the first line of the response
 * @param  command The command to send
 * @param  response If not NULL, the first line of the response string
 *                  NULL if the response was emptu or timeout reached
 * @param  timeout A timeout in milliseconds for which to wait for the response
 *                 before returning. Waits indefinitely if set to 0.
 * @retval If successful, the number of characters in the line
 * @retval -ERR_PARAM if command is NULL
 * @retval -ERR_TIMEOUT if response was not received in timeout milliseconds
 * @retval -ERR_GENERIC if the command failed to be sent
 */
int Network_SendGetLine(char *command, char* response, uint32_t timeout);

/**
 * @brief  Gets the current WLAN connection information
 * @param  connection Pointer to the returned connection structure
 * @param  timeout A timeout in milliseconds for which to wait for the response
 *                 before returning. Waits indefinitely if set to 0.
 * @retval 0 if successful
 * @retval -ERR_PARAM if connection is NULL
 * @retval -ERR_TIMEOUT if response was not received in timeout milliseconds
 * @retval -ERR_NOCONNECT if not connected
 * @retval -ERR_GENERIC if the command failed to be sent
 */
int Network_GetWlanConnection(NetworkWlanConnection_t *connection, uint32_t timeout);

/**
 * @brief  Gets the current IP Address
 * @param  address Pointer to the returned IP address
 * @retval 0 if successful
 * @retval -ERR_PARAM if address is NULL
 * @retval -ERR_TIMEOUT if response was not received in timeout milliseconds
 * @retval -ERR_NOCONNECT if not connected
 * @retval -ERR_GENERIC if the command failed to be sent
 */
int Network_GetIPAddress(char *address, uint32_t timeout);

/**
 * @brief  Get an email on the server
 * @param  subject Pointer to the returned email subject
 * @param  body Pointer to the returned email body
 * @retval 0 if successful
 * @retval -ERR_PARAM if subject or body is NULL
 * @retval -ERR_TIMEOUT if email was not received in timeout milliseconds
 * @retval -ERR_NOCONNECT if not connected
 * @retval -ERR_GENERIC if the command failed to be sent
 */
int Network_GetEmail(char *subject, char* body, uint32_t timeout);

/**
 * @brief  Get the current date and time
 * @param  time Pointer to the returned date and time
 * @retval 0 if successful
 * @retval -ERR_PARAM if subject or body is NULL
 * @retval -ERR_TIMEOUT if email was not received in timeout milliseconds
 * @retval -ERR_NOCONNECT if not connected
 * @retval -ERR_GENERIC if the command failed to be sent
 */
int Network_GetDateTime(NetworkDateTime_t *dateTime, uint32_t timeout);

#endif /* NETWORK_H */

