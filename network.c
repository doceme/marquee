/**
 ******************************************************************************
 *
 * @file       network.c
 * @author     Stephen Caudle Copyright (C) 2010
 * @brief      Network implementation
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

/* Includes */
#include "marquee.h"
#include "common.h"
#include "network.h"
#include "tprintf.h"
#include "string.h"

#define USART_NVIC_PRIO		0xf

#define NETWORK_APB1_PERIPH 	RCC_APB1Periph_USART2
#define NETWORK_APB2_PERIPH 	RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO

#define NETWORK_GPIO		GPIOA
#define NETWORK_GPIO_PIN_TX	GPIO_Pin_2
#define NETWORK_GPIO_PIN_RX	GPIO_Pin_3
#define NETWORK_GPIO_SPEED	GPIO_Speed_50MHz
#define NETWORK_GPIO_MODE	GPIO_Mode_AF_PP

#define NETWORK_USART		USART2

#define NETWORK_QUEUE_SIZE	256
#define NETWORK_BUFFER_SIZE	256
#define NETWORK_CMD_HEADER_SIZE 4
#define NETWORK_CMD_FOOTER_SIZE 1
#define NETWORK_LINE_MAX_LENGTH 80

/* Local Variables */
static xQueueHandle xQueue;
static xSemaphoreHandle xMutex;

/* One extra bytes for null terminator for debug purposes */
static uint8_t txBuffer[NETWORK_BUFFER_SIZE + 1];
static uint32_t txIndex;
static uint32_t txSize;
static char rxLine[NETWORK_LINE_MAX_LENGTH + 1];
static char rxLineAlt[NETWORK_LINE_MAX_LENGTH + 1];

/* Function Prototypes */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void NVIC_Configuration(void);
static void USART_Configuration(void);
static int SendCommand(char *command, uint32_t timeout);
static int WaitForResponse(char* response, uint32_t timeout);
static int GetResponse(char start, char end, char* response, uint32_t timeout);
static int GetLine(char* response, uint32_t timeout);
static int WaitForMessage(char *message, uint32_t timeout);

int Network_Configuration(void)
{
	RCC_Configuration();
	GPIO_Configuration();
	NVIC_Configuration();
	USART_Configuration();

	xQueue = xQueueCreate(NETWORK_QUEUE_SIZE, sizeof(char));
	assert_param(xQueue);

	xMutex = xSemaphoreCreateMutex();
	assert_param(xMutex);

	return 0;
}

int Network_SendWait(char *command, char* response, uint32_t timeout)
{
	int result;

	if (!command)
	{
		return -ERR_PARAM;
	}

	if (xSemaphoreTake(xMutex, timeout) != pdTRUE)
		return -ERR_TIMEOUT;

	result = SendCommand(command, timeout);

	if (result == 0 && response)
	{
		/* Wait for response */
		result = WaitForResponse(response, timeout);
	}

	xSemaphoreGive(xMutex);

	return result;
}

int Network_SendGetByChar(char *command, char start, char end, char* response, uint32_t timeout)
{
	int result;

	if (!command || !response)
		return -ERR_PARAM;

	if (xSemaphoreTake(xMutex, timeout) != pdTRUE)
		return -ERR_TIMEOUT;

	result = SendCommand(command, timeout);

	if (result == 0)
	{
		/* Get the response */
		result = GetResponse(start, end, response, timeout);
	}

	xSemaphoreGive(xMutex);

	return result;
}

int Network_SendGetLine(char *command, char* response, uint32_t timeout)
{
	int result;

	if (!command || !response)
		return -ERR_PARAM;

	if (xSemaphoreTake(xMutex, timeout) != pdTRUE)
		return -ERR_TIMEOUT;

	result = SendCommand(command, timeout);

	if (result == 0)
	{
		/* Get the response */
		result = GetLine(response, timeout);
	}

	xSemaphoreGive(xMutex);

	return result;
}

int Network_GetWlanConnection(NetworkWlanConnection_t *connection, uint32_t timeout)
{
	int result;
	int i = 0;

	if (connection == NULL)
		return -ERR_PARAM;

	result = Network_SendGetLine("!RP10", rxLine, timeout);

	if (result > 0)
	{
		char *pch = rxLine;
		char *pcomma = strchr(pch, ',');

		/* Initialize result to report an error in case something goes wrong */
		result = -ERR_GENERIC;

		while (pcomma++ != NULL)
		{
			size_t num = pcomma - pch - 1;
			switch (i++)
			{
				case 0:
				{
					strncpy(connection->ssid, pch, num);
				} break;

				case 1:
				{
					strncpy(connection->bssid, pch, num);
				} break;

				case 2:
				{
					if (strncmp(pch, "NONE", num) == 0)
					{
						connection->securityType = NETWORK_WLAN_SECURITY_TYPE_NONE;
					}
					else if (strncmp(pch, "WEP64", num) == 0)
					{
						connection->securityType = NETWORK_WLAN_SECURITY_TYPE_WEP64;
					}
					else if (strncmp(pch, "WEP128", num) == 0)
					{
						connection->securityType = NETWORK_WLAN_SECURITY_TYPE_WEP128;
					}
					else if (strncmp(pch, "WPA", num) == 0)
					{
						connection->securityType = NETWORK_WLAN_SECURITY_TYPE_WPA;
					}
					else if (strncmp(pch, "WPA2", num) == 0)
					{
						connection->securityType = NETWORK_WLAN_SECURITY_TYPE_WPA2;
					}
					else
					{
						return -ERR_GENERIC;
					}
				} break;

				case 3:
				{
					if (strncmp(pch, "NotCompleted", num) == 0)
					{
						connection->wpaStatus = NETWORK_WLAN_WPA_STATUS_NOTCOMPLETED;
					}
					else if (strncmp(pch, "Completed", num) == 0)
					{
						connection->wpaStatus = NETWORK_WLAN_WPA_STATUS_COMPLETED;
					}
					else
					{
						return -ERR_GENERIC;
					}
				} break;

				case 4:
				{
					connection->channel = atoi(pch);

					pch = pcomma;

					connection->snr = atoi(pch);
					result = 0;
				} break;

				default:
					return -ERR_GENERIC;
			}

			pch = pcomma;
			pcomma = strchr(pch, ',');
		}
	}
	else
	{
		result = -ERR_NOCONNECT;
	}

	return result;
}

int Network_GetIPAddress(char *address, uint32_t timeout)
{
	int result;

	if (!address)
		return -ERR_PARAM;

	if (xSemaphoreTake(xMutex, timeout) != pdTRUE)
		return -ERR_TIMEOUT;

	result = SendCommand("IPA?", timeout);

	*address = '\0';

	while (result == 0 && *address == '\0')
	{
		/* Wait for the subject line */
		result = GetLine(rxLineAlt, timeout);

		if (result > 0 && rxLineAlt[0] != '\0' && rxLineAlt[0] != 'I')
		{
			strcpy(address, rxLineAlt);
			result = 0;
		}
	}

	if (result == 0 && *address == '\0')
		result = -ERR_GENERIC;

	xSemaphoreGive(xMutex);

	return result;
}

int Network_GetMessage(char* message, uint32_t timeout)
{
	int result;

	if (!message)
		return -ERR_PARAM;

	if (xSemaphoreTake(xMutex, timeout) != pdTRUE)
		return -ERR_TIMEOUT;

	result = SendCommand("!RLNK:\"http://ledmarquee.appspot.com/?h=8347d096bc2c059f285807eccde44478\"", timeout);

	if (result == 0)
	{
		/* Wait for the subject line */
		result = WaitForMessage(message, timeout);

		if (result == 0)
		{
			result = WaitForResponse("I/ONLINE", timeout);

			if (result == 0 && 
				((strcmp(message, "error") == 0) || strcmp(message, "denied") == 0))
			{
				message[0] = '\0';
				result = -ERR_GENERIC;
			}
		}
	}

	xSemaphoreGive(xMutex);

	return result;
}

int Network_DeleteMessage(uint32_t timeout)
{
	int result;
	char response[NETWORK_LINE_MAX_LENGTH + 1];

	if (xSemaphoreTake(xMutex, timeout) != pdTRUE)
		return -ERR_TIMEOUT;

	result = SendCommand("!RLNK:\"http://ledmarquee.appspot.com/d?h=8347d096bc2c059f285807eccde44478\"", timeout);

	if (result == 0)
	{
		/* Wait for the subject line */
		result = WaitForMessage(response, timeout);

		if (result == 0 && strcmp(response, "ok") != 0)
			result = -ERR_GENERIC;

		if (result == 0)
			result = WaitForResponse("I/ONLINE", timeout);
	}

	xSemaphoreGive(xMutex);

	return result;
}

int Network_GetDateTime(NetworkDateTime_t *dateTime, uint32_t timeout)
{
	int result;

	if (!dateTime)
		return -ERR_PARAM;

	if (xSemaphoreTake(xMutex, timeout) != pdTRUE)
		return -ERR_TIMEOUT;

	result = SendCommand("RP8", timeout);

	dateTime->year = 0;

	while (result == 0 && dateTime->year == 0)
	{
		/* Wait for the subject line */
		result = GetLine(rxLineAlt, timeout);

		if (result > 0)
		{
			result = 0;

			if (rxLineAlt[0] >= '0' && rxLineAlt[0] <= '9')
			{
				char *ch = rxLineAlt;

				rxLineAlt[4] = '\0';
				dateTime->year = (uint16_t)atoi(ch);

				ch += 5;
				rxLineAlt[7] = '\0';
				dateTime->month = (uint8_t)atoi(ch);

				ch += 3;
				rxLineAlt[10] = '\0';
				dateTime->day = (uint8_t)atoi(ch);

				ch += 3;
				rxLineAlt[13] = '\0';
				dateTime->hours = (uint8_t)atoi(ch);

				ch += 3;
				rxLineAlt[16] = '\0';
				dateTime->minutes = (uint8_t)atoi(ch);

				ch += 3;
				dateTime->seconds = (uint8_t)atoi(ch);
			}
		}
	}

	if (result == 0 && dateTime->year == 0)
		result = -ERR_GENERIC;

	xSemaphoreGive(xMutex);

	return result;
}

int WaitForMessage(char* message, uint32_t timeout)
{
	int result = 0;

	*message = '\0';

	while (result == 0 && *message == '\0')
	{
		result = GetLine(rxLineAlt, timeout);

		if (result > 0 && strncmp(rxLineAlt, "I/", 2) != 0)
		{
			result = 0;
			strcpy(message, rxLineAlt);
			break;
		}
		else if (result > 0)
		{
			result = 0;
		}
	}

	return result;
}

int SendCommand(char *command, uint32_t timeout)
{
	int result = 0;
	uint8_t *ch = txBuffer + NETWORK_CMD_HEADER_SIZE;

	if (!command)
	{
		return -ERR_PARAM;
	}

	/* Empty the receive queue */
	while (xQueueReceive(xQueue, ch, 0));

	txIndex = 0;
	txSize = 0;

	while (*command != '\0')
	{
		*ch++ = *command++;

		if (++txSize >= (NETWORK_BUFFER_SIZE - (NETWORK_CMD_HEADER_SIZE + NETWORK_CMD_FOOTER_SIZE)))
		{
			return -ERR_OVERFLOW;
		}
	}

	/* Add footer bytes */
	*ch++ = '\r';
	*ch = '\0';

	ch = txBuffer;

	/* Add header bytes */
	*ch++ = 'A';
	*ch++ = 'T';
	*ch++ = '+';
	*ch++ = 'i';
	txSize += NETWORK_CMD_HEADER_SIZE + NETWORK_CMD_FOOTER_SIZE;

	/* Enable transmit empty interrupt */
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

	return result;
}

int WaitForResponse(char* response, uint32_t timeout)
{
	portTickType elapsed;
	portTickType start = xTaskGetTickCount();
	portTickType block = (timeout == 0 ? portMAX_DELAY : timeout / portTICK_RATE_MS);
	char ch;
	char *match = response;

	if (!response)
	{
		return -ERR_PARAM;
	}

	while (*match != '\0')
	{
		portBASE_TYPE result = xQueueReceive(xQueue, &ch, block);

		if (result)
		{
			if (block != portMAX_DELAY)
			{
				elapsed = xTaskGetTickCount() - start;

				if (elapsed < timeout)
				{
					block = (timeout - elapsed) / portTICK_RATE_MS;
				}
				else
				{
					return -ERR_TIMEOUT;
				}
			}

			if (ch != *match++)
			{
				match = response;
			}
		}
		else
		{
			return -ERR_TIMEOUT;
		}
	}

	return 0;
}

int GetResponse(char start_ch, char end_ch, char* response, uint32_t timeout)
{
	portTickType elapsed;
	portTickType start = xTaskGetTickCount();
	portTickType block = (timeout == 0 ? portMAX_DELAY : timeout / portTICK_RATE_MS);
	char ch;

	if (!response)
	{
		return -ERR_PARAM;
	}

	do
	{
		portBASE_TYPE result = xQueueReceive(xQueue, &ch, block);

		if (result)
		{
			if (block != portMAX_DELAY)
			{
				elapsed = xTaskGetTickCount() - start;

				if (elapsed < timeout)
				{
					block = (timeout - elapsed) / portTICK_RATE_MS;
				}
				else
				{
					return -ERR_TIMEOUT;
				}
			}
		}
		else
		{
			return -ERR_TIMEOUT;
		}
	} while (ch != start_ch);

	portBASE_TYPE result = xQueueReceive(xQueue, &ch, block);

	if (result)
	{
		while (ch != end_ch)
		{
			*response++ = ch;

			portBASE_TYPE result = xQueueReceive(xQueue, &ch, block);

			if (result)
			{
				if (block != portMAX_DELAY)
				{
					elapsed = xTaskGetTickCount() - start;

					if (elapsed < timeout)
					{
						block = (timeout - elapsed) / portTICK_RATE_MS;
					}
					else
					{
						return -ERR_TIMEOUT;
					}
				}
			}
			else
			{
				return -ERR_TIMEOUT;
			}
		}

		if (block != portMAX_DELAY)
		{
			elapsed = xTaskGetTickCount() - start;

			if (elapsed < timeout)
			{
				block = (timeout - elapsed) / portTICK_RATE_MS;
			}
			else
			{
				return -ERR_TIMEOUT;
			}
		}
	}
	else
	{
		return -ERR_TIMEOUT;
	}

	/* Null terminate the response */
	*response = '\0';
	return 0;
}

int GetLine(char* response, uint32_t timeout)
{
	portTickType elapsed;
	portTickType start = xTaskGetTickCount();
	portTickType block = (timeout == 0 ? portMAX_DELAY : timeout / portTICK_RATE_MS);
	portBASE_TYPE result = pdTRUE;
	char ch = '\r';
	int i = 0;

	if (!response)
	{
		return -ERR_PARAM;
	}

	while ((ch == '\r' || ch == '\n') && result)
	{
		result = xQueueReceive(xQueue, &ch, block);

		if (result)
		{

			if (block != portMAX_DELAY)
			{
				elapsed = xTaskGetTickCount() - start;

				if (elapsed < timeout)
				{
					block = (timeout - elapsed) / portTICK_RATE_MS;
				}
				else
				{
					return -ERR_TIMEOUT;
				}
			}
		}
		else
		{
			return -ERR_TIMEOUT;
		}
	}

	while (ch != '\r' && ch != '\n' && i < NETWORK_LINE_MAX_LENGTH)
	{
		rxLine[i++] = ch;

		portBASE_TYPE result = xQueueReceive(xQueue, &ch, block);

		if (result)
		{

			if (block != portMAX_DELAY)
			{
				elapsed = xTaskGetTickCount() - start;

				if (elapsed < timeout)
				{
					block = (timeout - elapsed) / portTICK_RATE_MS;
				}
				else
				{
					return -ERR_TIMEOUT;
				}
			}
		}
		else
		{
			return -ERR_TIMEOUT;
		}
	}

	/* Null terminate the response */
	rxLine[i] = '\0';
	strcpy(response, rxLine);
	return i;
}

/**
 * @brief  Configures the network system clocks
 * @param  None
 * @retval None
 */
void RCC_Configuration(void)
{
	/* USART clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Enable GPIO peripheral clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
}

/**
 * @brief  Configures the network GPIO ports
 * @param  None
 * @retval None
 */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
  * @brief  Configure the NVIC for USART
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
	/* Configure the USART Interrupts */
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART_NVIC_PRIO;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART_NVIC_PRIO;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Configures the network USART
 * @param  None
 * @retval None
 */
void USART_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(USART1, &USART_InitStructure);
	USART_Init(USART2, &USART_InitStructure);

	/* Enable receive interrupts */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);
	USART_Cmd(USART2, ENABLE);
}

#if 0
/**
  * @brief  This function handles USART1 global interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		/* Clear interrupt */
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);

		if (USART_GetFlagStatus(USART2, USART_FLAG_TXE) != RESET)
		{
			/* Read one byte from the receive data register */
			uint16_t ch = USART_ReceiveData(USART1);
			USART_SendData(USART2, ch);
		}
	}
}
#endif

/**
  * @brief  This function handles USART2 global interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		/* Read one character and enqueue */
		char ch = USART_ReceiveData(USART2);
		xQueueSendToBackFromISR(xQueue, &ch, &xHigherPriorityTaskWoken);

#if 0
		if (USART_GetFlagStatus(USART1, USART_FLAG_TXE) != RESET)
			USART_SendData(USART1, ch);
#endif

		/* Clear interrupt */
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}

	if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
	{
		if (txSize)
		{
			USART_SendData(USART2, txBuffer[txIndex++]);
			txSize--;
		}
		else
		{
			/* Disable transmit empty interrupt */
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		}
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
