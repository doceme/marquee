/**
 ******************************************************************************
 *
 * @file       led.c
 * @author     Stephen Caudle Copyright (C) 2010
 * @brief      LED display implementation
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
#include "led.h"
#include "font.h"
#include "string.h"

/* Defines */
#define LED_NVIC_PRIO		IRQ_PRIO_HIGHEST
#define LED_TIMER		TIM2
#define LED_IRQn		TIM2_IRQn

#define LED_NUM_LINES		2
#define LED_NUM_DISPLAYS	5
#define LED_MAX_ADDRESS		0x5F
#define LED_BUFFER_SIZE		(24 * LED_NUM_DISPLAYS)
#define LED_MAX_CHARS_PER_LINE	20

#define MAX_CHARS_5X7		6
#define CHAR_WIDTH_5X7		5

#define SPI_CS_LOW(n)	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET); \
				GPIO_ResetBits(SPI_GPIO_CS, n);
#define SPI_CS_HIGH(n)	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_BSY) == SET); \
				GPIO_SetBits(SPI_GPIO_CS, n);

#define SPI			SPI2
#define SPI_CLK			RCC_APB1Periph_SPI2
#define SPI_GPIO		GPIOB
#define SPI_GPIO_CS		GPIOC
#define SPI_GPIO_CLK		RCC_APB2Periph_GPIOB
#define SPI_GPIO_CS_CLK		RCC_APB2Periph_GPIOC
#define SPI_PIN_CS_0		GPIO_Pin_0
#define SPI_PIN_CS_1		GPIO_Pin_1
#define SPI_PIN_CS_2		GPIO_Pin_2
#define SPI_PIN_CS_3		GPIO_Pin_3
#define SPI_PIN_CS_4		GPIO_Pin_4
#define SPI_PIN_CS_MASTER	SPI_PIN_CS_0
#define SPI_PIN_CS_SLAVES	(SPI_PIN_CS_1 | SPI_PIN_CS_2 | SPI_PIN_CS_3 | SPI_PIN_CS_4)
#define SPI_PIN_CS_ALL		(SPI_PIN_CS_MASTER | SPI_PIN_CS_SLAVES)
#define SPI_PIN_SCK		GPIO_Pin_13
#define SPI_PIN_MISO		GPIO_Pin_14
#define SPI_PIN_MOSI		GPIO_Pin_15
#define SPI_MASTER_IRQn		SPI2_IRQn

/* Local Variables */
static uint8_t lines[LED_NUM_LINES][LED_BUFFER_SIZE];
#ifdef LED_INTERRUPT
static uint16_t display[LED_BUFFER_SIZE];
static xTaskHandle xLEDTask = NULL;
static uint8_t enableInterrupt = 0;
#endif

/* Function Prototypes */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
#ifdef LED_INTERRUPT
static void NVIC_Configuration(void);
static void Timer_Configuration(void);
static void LED_Task(void *pvParameters);
#endif
static void SPI_Configuration(void);
static void LED_WriteCommand(uint16_t cs, uint8_t command);
static void LED_WriteData(uint16_t cs, uint8_t address, uint8_t data);

int LED_Configuration(void)
{
	int j;

	RCC_Configuration();
	GPIO_Configuration();
	SPI_Configuration();
#ifdef LED_INTERRUPT
	NVIC_Configuration();
	Timer_Configuration();
#endif

	LED_WriteCommand(SPI_PIN_CS_ALL, 0x00);
	LED_WriteCommand(SPI_PIN_CS_ALL, 0x24);
	LED_WriteCommand(SPI_PIN_CS_MASTER, 0x18);
	LED_WriteCommand(SPI_PIN_CS_SLAVES, 0x10);
	LED_WriteCommand(SPI_PIN_CS_ALL, 0x01);
	LED_WriteCommand(SPI_PIN_CS_ALL, 0x03);
	LED_WriteCommand(SPI_PIN_CS_ALL, 0x08);
	LED_WriteCommand(SPI_PIN_CS_ALL, 0xa6);

	for (j = LED_MAX_ADDRESS; j >= 0; j--)
	{
		LED_WriteData(SPI_PIN_CS_ALL, j, 0x0);
	}

#if 0
	for (j = LED_MAX_ADDRESS; j >= 0; j--)
	{
		LED_WriteData(SPI_PIN_CS_0, j, 0xf);
	}
#endif

#if 0
	for (j = LED_MAX_ADDRESS; j >= 0; j--)
	{
		uint16_t cs = SPI_PIN_CS_2;
		LED_WriteData(cs, j, 0x1);
		vTaskDelay(100 / portTICK_RATE_MS);
		LED_WriteData(cs, j, 0x2);
		vTaskDelay(100 / portTICK_RATE_MS);
		LED_WriteData(cs, j, 0x4);
		vTaskDelay(100 / portTICK_RATE_MS);
		LED_WriteData(cs, j, 0x8);
		vTaskDelay(100 / portTICK_RATE_MS);
		LED_WriteData(cs, j, 0x0);
	}
#endif

#if 0
	while (1)
	{
		LED_SetLine(0, "01234567890123456789");
		LED_SetLine(1, "ABCDEFGHIJKLMNOPQRST");
		LED_Refresh();

		vTaskDelay(1000 / portTICK_RATE_MS);
		LED_ScrollOut(0);
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
#endif

#ifdef LED_INTERRUPT
	/* Create a FreeRTOS task */
	xTaskCreate(LED_Task, (signed portCHAR *)"LED", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY + 2, &xLEDTask);
	assert_param(xLEDTask);
#endif

	return 0;
}

int LED_SetLine(uint8_t line, char *message)
{
	uint8_t i = 0;
	uint8_t j;
	uint8_t k = 0;
	uint8_t *buffer = lines[line];
	uint32_t len;

	if (message && line >= LED_NUM_LINES)
		return -ERR_PARAM;

	len = strlen(message);

	/* Clear the display */
	memset(buffer, 0, LED_BUFFER_SIZE);

	if (len == 0)
		return 0;

#ifdef LED_INTERRUPT
	portENTER_CRITICAL();

	if (len > LED_MAX_CHARS_PER_LINE)
		enableInterrupt = 1;
#endif

	while ((*message != 0) && (k < LED_MAX_CHARS_PER_LINE))
	{
		unsigned int index = (unsigned char)*message;
		if (index < 0 || index > 127)
		{
			index = 0;
		}
		else
		{
			index *= CHAR_WIDTH_5X7;
		}

		for (j = 0; j < CHAR_WIDTH_5X7; j++)
		{
			buffer[i++] = font5x7[index + j];
		}

		i++;
		k++;

		message++;
	}

#ifdef LED_INTERRUPT
	if (enableInterrupt)
	{
	}

	portENTER_CRITICAL();
#endif

	return 0;
}

int LED_Refresh(void)
{
	int i = 0;
	int j = 0;
	int k = 0;
	uint16_t cs = SPI_PIN_CS_4;

	uint8_t* line0 = lines[0];
	uint8_t* line1 = lines[1];;

	for (j = 0; j < LED_BUFFER_SIZE; j++)
	{
		LED_WriteData(cs, i++, (line0[j]) & 0xf);
		LED_WriteData(cs, i++, (line0[j] >> 4) & 0xf);
		LED_WriteData(cs, i++, (line1[j]) & 0xf);
		LED_WriteData(cs, i++, (line1[j] >> 4) & 0xf);
		if (i > LED_MAX_ADDRESS) {
			cs >>= 1;
			i = 0;
			k++;
		}
	}

	return 0;
}

int LED_ScrollIn(uint8_t line, char* message)
{
	int i = 0;
	int j = 0;

	for (i = 0; i < LED_BUFFER_SIZE; i++)
	{
		uint8_t* line0 = &lines[0][LED_BUFFER_SIZE - 1];
		uint8_t* line1 = &lines[1][LED_BUFFER_SIZE - 1];
		uint8_t* line0_next = &lines[0][LED_BUFFER_SIZE - 1];
		uint8_t* line1_next = &lines[1][LED_BUFFER_SIZE - 1];

		for (j = 0; j < LED_BUFFER_SIZE; j++)
		{
			if (j < (LED_BUFFER_SIZE - 1))
			{
				*line0++ = *(++line0_next);
				*line1++ = *(++line1_next);
			}
			else
			{
				*line0 = 0;
				*line1 = 0;
			}
		}

		LED_Refresh();
		//vTaskDelay(1 / portTICK_RATE_MS);
	}

	return 0;
}

int LED_ScrollOut(uint8_t line)
{
	int i = 0;
	int j = 0;

	for (i = 0; i < LED_BUFFER_SIZE; i++)
	{
		uint8_t* line0 = lines[0];
		uint8_t* line1 = lines[1];
		uint8_t* line0_next = lines[0];
		uint8_t* line1_next = lines[1];

		for (j = 0; j < LED_BUFFER_SIZE; j++)
		{
			if (j < (LED_BUFFER_SIZE - 1))
			{
				*line0++ = *(++line0_next);
				*line1++ = *(++line1_next);
			}
			else
			{
				*line0 = 0;
				*line1 = 0;
			}
		}

		LED_Refresh();
		//vTaskDelay(1 / portTICK_RATE_MS);
	}

	return 0;
}

/**
 * @brief  Configures the LED system clocks
 * @param  None
 * @retval None
 */
void RCC_Configuration(void)
{
	/* Enable GPIOC peripheral clock */
	RCC_APB2PeriphClockCmd(SPI_GPIO_CLK | SPI_GPIO_CS_CLK | RCC_APB2Periph_AFIO, ENABLE);

	/* Enable SPI2 Periph clock */
#ifdef LED_INTERRUPT
	RCC_APB1PeriphClockCmd(SPI_CLK | RCC_APB1Periph_TIM4, ENABLE);
#else
	RCC_APB1PeriphClockCmd(SPI_CLK, ENABLE);
#endif
}

/**
 * @brief  Configures the LED GPIO ports
 * @param  None
 * @retval None
 */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Initialize GPIO */
	GPIO_InitStructure.GPIO_Pin = SPI_PIN_CS_ALL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SPI_GPIO_CS, &GPIO_InitStructure);

	/* Configure SPI pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = SPI_PIN_SCK | SPI_PIN_MISO | SPI_PIN_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

	/* Inactivate CS */
	GPIO_SetBits(SPI_GPIO, SPI_PIN_CS_ALL);
}

#ifdef LED_INTERRUPT
/**
  * @brief  Configure the NVIC for the LED
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
	/* Configure the buzzer Interrupts */
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the buzzer Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = LED_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LED_NVIC_PRIO;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
#endif

/**
 * @brief  Configures the LED SPI port
 * @param  None
 * @retval None
 */
void SPI_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStructure;

	/* SPI_MASTER configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI, &SPI_InitStructure);

	/* Enable SPI_MASTER TXE interrupt */
	//SPI_I2S_ITConfig(SPI, SPI_I2S_IT_TXE, ENABLE);

	/* Enable SPI */
	SPI_Cmd(SPI, ENABLE);
}

#ifdef LED_INTERRUPT
/**
 * @brief  Configures the LED timer
 * @param  None
 * @retval None
 */
void Timer_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_ClocksTypeDef RCC_ClockFreq;

	/* Set CCR value based on PCLK1 frequency and desired buzzer frequency */
	RCC_GetClocksFreq(&RCC_ClockFreq);

	/* Toggle timer base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = (RCC_ClockFreq.PCLK1_Frequency / 1000) - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(LED_TIMER, &TIM_TimeBaseStructure);

	/* Disable timer counters */
	TIM_Cmd(LED_TIMER, DISABLE);
}
#endif

/**
 * @brief  Writes an LED command
 * @param  None
 * @retval None
 */
void LED_WriteCommand(uint16_t cs, uint8_t command)
{
	uint16_t spi_command = (0x8000 | (command << 5)) & 0xFFE0;

	/* Wait for SPI_MASTER Tx buffer empty */
	SPI_CS_LOW(cs);
	SPI_I2S_SendData(SPI, spi_command);
	SPI_CS_HIGH(cs);
}

/**
 * @brief  Writes LED data
 * @param  None
 * @retval None
 */
void LED_WriteData(uint16_t cs, uint8_t address, uint8_t data)
{
	uint16_t spi_command = (0xa000 | ((address & 0x7F) << 6) | ((data & 0xF) << 2)) & 0xFFFC;

	/* Wait for SPI_MASTER Tx buffer empty */
	SPI_CS_LOW(cs);
	SPI_I2S_SendData(SPI, spi_command);
	SPI_CS_HIGH(cs);
}

void LED_Task(void *pvParameters)
{
	portTickType xLastWakeTime;

	/* Initialise the xLastExecutionTime variable on task entry */
	xLastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		vTaskSuspend(NULL);
	}
}
