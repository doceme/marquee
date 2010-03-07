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

#define USART_NVIC_PRIO		IRQ_PRIO_HIGHEST

#define NETWORK_APB1_PERIPH 	RCC_APB1Periph_USART2
#define NETWORK_APB2_PERIPH 	RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO

#define NETWORK_GPIO		GPIOA
#define NETWORK_GPIO_PIN_TX	GPIO_Pin_2
#define NETWORK_GPIO_PIN_RX	GPIO_Pin_3
#define NETWORK_GPIO_SPEED	GPIO_Speed_50MHz
#define NETWORK_GPIO_MODE	GPIO_Mode_AF_PP

#define NETWORK_USART		USART2

#define NETWORK_QUEUE_SIZE	16
#define NETWORK_BUFFER_SIZE	256

/* Local Variables */
static xQueueHandle xQueue = NULL;

static uint8_t txBuffer[NETWORK_BUFFER_SIZE];
static uint32_t txIndex;
static uint32_t txSize;

/* Function Prototypes */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void NVIC_Configuration(void);
static void USART_Configuration(void);
static int WaitForResponse(char* response, uint32_t timeout);

int Network_Configuration(void)
{
	RCC_Configuration();
	GPIO_Configuration();
	NVIC_Configuration();
	USART_Configuration();

	xQueue = xQueueCreate(NETWORK_QUEUE_SIZE, sizeof(uint8_t));
	assert_param(xQueue);

	return 0;
}
int Network_SendCommand(char *command, char* response, uint32_t timeout)
{
	int result = 0;
	int i = 0;
	uint8_t *ch = txBuffer;

	if (!command || !response)
	{
		return -ERR_PARAM;
	}

	portENTER_CRITICAL();

	txIndex = 0;
	txSize = 0;

	while (*command != '\0')
	{
		*ch++ = *command++;

		if (++txSize >= (NETWORK_BUFFER_SIZE - 1))
		{
			return -ERR_OVERFLOW;
		}
	}

	*ch = '\0';

	/* Enable transmit empty interrupt */
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

	portEXIT_CRITICAL();

	/* Wait for response */
	if (response)
	{
		result = WaitForResponse(response, timeout);
	}

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
			if (ch != *match++)
			{
				match = response;
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
	}

	return 0;
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

/**
  * @brief  This function handles USART2 global interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		uint16_t ch;
		portBASE_TYPE xHigherPriorityTaskWoken;

		/* Read one character and enqueue */
		ch = USART_ReceiveData(USART2);
		xQueueSendToBackFromISR(xQueue, &ch, &xHigherPriorityTaskWoken);

		/* Clear interrupt */
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);

		if (USART_GetFlagStatus(USART1, USART_FLAG_TXE) != RESET)
		{
			USART_SendData(USART1, ch);
		}
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
}
