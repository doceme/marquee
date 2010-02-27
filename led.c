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

/* Defines */
#define LED_VALUE		0xa

#define LED_NUM_LINES 2
#define LED_BUFFER_SIZE		24

#define MAX_CHARS_5X7		6

#define SPI_CS_LOW	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET); \
				GPIO_ResetBits(SPI_GPIO, SPI_PIN_CS);
#define SPI_CS_HIGH	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_BSY) == SET); \
				GPIO_SetBits(SPI_GPIO, SPI_PIN_CS);

#define SPI			SPI2
#define SPI_CLK			RCC_APB1Periph_SPI2
#define SPI_GPIO		GPIOB
#define SPI_GPIO_CLK		RCC_APB2Periph_GPIOB
#define SPI_PIN_CS		GPIO_Pin_12
#define SPI_PIN_SCK		GPIO_Pin_13
#define SPI_PIN_MISO		GPIO_Pin_14
#define SPI_PIN_MOSI		GPIO_Pin_15
#define SPI_MASTER_IRQn         SPI2_IRQn

/* Local Variables */
static uint8_t lines[LED_NUM_LINES][LED_BUFFER_SIZE];

/* Function Prototypes */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void SPI_Configuration(void);
static void LED_WriteCommand(uint8_t command);
static void LED_WriteData(uint8_t address, uint8_t data);

int LED_Configuration(void)
{
	int i;

	RCC_Configuration();
	GPIO_Configuration();
	SPI_Configuration();

	LED_WriteCommand(0x01);
	LED_WriteCommand(0x03);
	LED_WriteCommand(0x2c);
	LED_WriteCommand(0xaf);
	LED_WriteCommand(0x09);

	for (i = 0; i < 96; i++)
	{
		LED_WriteData(i, 0x0);
	}

#if 0
	LED_SetLine(0, "LED");
	LED_SetLine(1, "Test");
	LED_Refresh();
#endif

	return 0;
}

int LED_SetLine(uint8_t line, char *message)
{
	uint8_t i = 0;
	uint8_t *buffer = lines[line];

	if (message && line >= LED_NUM_LINES)
		return -ERR_PARAM;

	if (strlen(message) == 0)
		return 0;

	/* Clear the line */
	memset(buffer, 0, LED_BUFFER_SIZE);

	while ((*message != 0) && (i < 24))
	{
		uint32_t value;
		int k = (char)*message;
		if (k < 0 || k > 127)
		{
			k = 0;
		}
		k *= 5;

		value = font5x7[k] << 24;
		asm ("rbit %0, %1;" : "=r" (value) : "r" (value)); /* Bit reverse */
		buffer[i++] = (uint8_t)value;

		value = font5x7[k + 1] << 24;
		asm ("rbit %0, %1;" : "=r" (value) : "r" (value)); /* Bit reverse */
		buffer[i++] = (uint8_t)value;

		value = font5x7[k + 2] << 24;
		asm ("rbit %0, %1;" : "=r" (value) : "r" (value)); /* Bit reverse */
		buffer[i++] = (uint8_t)value;

		value = font5x7[k + 3] << 24;
		asm ("rbit %0, %1;" : "=r" (value) : "r" (value)); /* Bit reverse */
		buffer[i++] = (uint8_t)value;

		value = font5x7[k + 4] << 24;
		asm ("rbit %0, %1;" : "=r" (value) : "r" (value)); /* Bit reverse */
		buffer[i++] = (uint8_t)value;
		i++;

		message++;
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
	RCC_APB2PeriphClockCmd(SPI_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

	/* Enable SPI2 Periph clock */
	RCC_APB1PeriphClockCmd(SPI_CLK, ENABLE);
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
	GPIO_InitStructure.GPIO_Pin = SPI_PIN_CS;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

	/* Configure SPI pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = SPI_PIN_SCK | SPI_PIN_MISO | SPI_PIN_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

	/* Inactivate CS */
	GPIO_SetBits(SPI_GPIO, SPI_PIN_CS);
}

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

/**
 * @brief  Writes an LED command
 * @param  None
 * @retval None
 */
void LED_WriteCommand(uint8_t command)
{
	uint16_t spi_command = (0x8000 | (command << 5)) & 0xFFE0;

	/* Wait for SPI_MASTER Tx buffer empty */
	SPI_CS_LOW;
	SPI_I2S_SendData(SPI, spi_command);
	SPI_CS_HIGH;
}

/**
 * @brief  Writes LED data
 * @param  None
 * @retval None
 */
void LED_WriteData(uint8_t address, uint8_t data)
{
	uint16_t spi_command = (0xa000 | ((address & 0x7F) << 6) | ((data & 0xF) << 2)) & 0xFFFC;

	/* Wait for SPI_MASTER Tx buffer empty */
	SPI_CS_LOW;
	SPI_I2S_SendData(SPI, spi_command);
	SPI_CS_HIGH;
}

