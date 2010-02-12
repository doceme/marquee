/**
 ******************************************************************************
 *
 * @file       marquee.c
 * @author     Stephen Caudle Copyright (C) 2010.
 * @brief      Sets up RTOS tasks
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


/* Project Includes */
#include "marquee.h"
#include <stdio.h>

/* Definitions */
#define TASK_SPIN_DELAY		(500 / portTICK_RATE_MS)
#define NUMBER_OF_BEEPS		3
#define SPI_CS_DELAY		10
#define LED_VALUE		0xa
#define LED_BUFFER_SIZE		24

#define MAX_CHARS_5X7		6

#define BEEP(n)		do { beep_count = n; vTaskResume(xBuzzerTask); } while (0);

#define SPI_CS_LOW	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET); \
				GPIO_ResetBits(SPI_GPIO, SPI_PIN_CS);
#define SPI_CS_HIGH	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_BSY) == SET); \
				GPIO_SetBits(SPI_GPIO, SPI_PIN_CS);

/* Global Variables */
uint8_t ready = 0;

/* Local Variables */
static uint8_t beep_count = 0;
static xTaskHandle xBuzzerTask = NULL;
static uint8_t line1[LED_BUFFER_SIZE];
static uint8_t line2[LED_BUFFER_SIZE];

// 5x7 Font Table
// ASCII characters 0x20-0x7F (32-127)
static const unsigned char font5x7[] = {
        0x00, 0x00, 0x00, 0x00, 0x00,// (space)
        0x00, 0x00, 0x5F, 0x00, 0x00,// !
        0x00, 0x07, 0x00, 0x07, 0x00,// "
        0x14, 0x7F, 0x14, 0x7F, 0x14,// #
        0x24, 0x2A, 0x7F, 0x2A, 0x12,// $
        0x23, 0x13, 0x08, 0x64, 0x62,// %
        0x36, 0x49, 0x55, 0x22, 0x50,// &
        0x00, 0x05, 0x03, 0x00, 0x00,// '
        0x00, 0x1C, 0x22, 0x41, 0x00,// (
        0x00, 0x41, 0x22, 0x1C, 0x00,// )
        0x08, 0x2A, 0x1C, 0x2A, 0x08,// *
        0x08, 0x08, 0x3E, 0x08, 0x08,// +
        0x00, 0x50, 0x30, 0x00, 0x00,// ,
        0x08, 0x08, 0x08, 0x08, 0x08,// -
        0x00, 0x60, 0x60, 0x00, 0x00,// .
        0x20, 0x10, 0x08, 0x04, 0x02,// /
        0x3E, 0x51, 0x49, 0x45, 0x3E,// 0
        0x00, 0x42, 0x7F, 0x40, 0x00,// 1
        0x42, 0x61, 0x51, 0x49, 0x46,// 2
        0x21, 0x41, 0x45, 0x4B, 0x31,// 3
        0x18, 0x14, 0x12, 0x7F, 0x10,// 4
        0x27, 0x45, 0x45, 0x45, 0x39,// 5
        0x3C, 0x4A, 0x49, 0x49, 0x30,// 6
        0x01, 0x71, 0x09, 0x05, 0x03,// 7
        0x36, 0x49, 0x49, 0x49, 0x36,// 8
        0x06, 0x49, 0x49, 0x29, 0x1E,// 9
        0x00, 0x36, 0x36, 0x00, 0x00,// :
        0x00, 0x56, 0x36, 0x00, 0x00,// ;
        0x00, 0x08, 0x14, 0x22, 0x41,// <
        0x14, 0x14, 0x14, 0x14, 0x14,// =
        0x41, 0x22, 0x14, 0x08, 0x00,// >
        0x02, 0x01, 0x51, 0x09, 0x06,// ?
        0x32, 0x49, 0x79, 0x41, 0x3E,// @
        0x7E, 0x11, 0x11, 0x11, 0x7E,// A
        0x7F, 0x49, 0x49, 0x49, 0x36,// B
        0x3E, 0x41, 0x41, 0x41, 0x22,// C
        0x7F, 0x41, 0x41, 0x22, 0x1C,// D
        0x7F, 0x49, 0x49, 0x49, 0x41,// E
        0x7F, 0x09, 0x09, 0x01, 0x01,// F
        0x3E, 0x41, 0x41, 0x51, 0x32,// G
        0x7F, 0x08, 0x08, 0x08, 0x7F,// H
        0x00, 0x41, 0x7F, 0x41, 0x00,// I
        0x20, 0x40, 0x41, 0x3F, 0x01,// J
        0x7F, 0x08, 0x14, 0x22, 0x41,// K
        0x7F, 0x40, 0x40, 0x40, 0x40,// L
        0x7F, 0x02, 0x04, 0x02, 0x7F,// M
        0x7F, 0x04, 0x08, 0x10, 0x7F,// N
        0x3E, 0x41, 0x41, 0x41, 0x3E,// O
        0x7F, 0x09, 0x09, 0x09, 0x06,// P
        0x3E, 0x41, 0x51, 0x21, 0x5E,// Q
        0x7F, 0x09, 0x19, 0x29, 0x46,// R
        0x46, 0x49, 0x49, 0x49, 0x31,// S
        0x01, 0x01, 0x7F, 0x01, 0x01,// T
        0x3F, 0x40, 0x40, 0x40, 0x3F,// U
        0x1F, 0x20, 0x40, 0x20, 0x1F,// V
        0x7F, 0x20, 0x18, 0x20, 0x7F,// W
        0x63, 0x14, 0x08, 0x14, 0x63,// X
        0x03, 0x04, 0x78, 0x04, 0x03,// Y
        0x61, 0x51, 0x49, 0x45, 0x43,// Z
        0x00, 0x00, 0x7F, 0x41, 0x41,// [
        0x02, 0x04, 0x08, 0x10, 0x20,// "\"
        0x41, 0x41, 0x7F, 0x00, 0x00,// ]
        0x04, 0x02, 0x01, 0x02, 0x04,// ^
        0x40, 0x40, 0x40, 0x40, 0x40,// _
        0x00, 0x01, 0x02, 0x04, 0x00,// `
        0x20, 0x54, 0x54, 0x54, 0x78,// a
        0x7F, 0x48, 0x44, 0x44, 0x38,// b
        0x38, 0x44, 0x44, 0x44, 0x20,// c
        0x38, 0x44, 0x44, 0x48, 0x7F,// d
        0x38, 0x54, 0x54, 0x54, 0x18,// e
        0x08, 0x7E, 0x09, 0x01, 0x02,// f
        0x08, 0x14, 0x54, 0x54, 0x3C,// g
        0x7F, 0x08, 0x04, 0x04, 0x78,// h
        0x00, 0x44, 0x7D, 0x40, 0x00,// i
        0x20, 0x40, 0x44, 0x3D, 0x00,// j
        0x00, 0x7F, 0x10, 0x28, 0x44,// k
        0x00, 0x41, 0x7F, 0x40, 0x00,// l
        0x7C, 0x04, 0x18, 0x04, 0x78,// m
        0x7C, 0x08, 0x04, 0x04, 0x78,// n
        0x38, 0x44, 0x44, 0x44, 0x38,// o
        0x7C, 0x14, 0x14, 0x14, 0x08,// p
        0x08, 0x14, 0x14, 0x18, 0x7C,// q
        0x7C, 0x08, 0x04, 0x04, 0x08,// r
        0x48, 0x54, 0x54, 0x54, 0x20,// s
        0x04, 0x3F, 0x44, 0x40, 0x20,// t
        0x3C, 0x40, 0x40, 0x20, 0x7C,// u
        0x1C, 0x20, 0x40, 0x20, 0x1C,// v
        0x3C, 0x40, 0x30, 0x40, 0x3C,// w
        0x44, 0x28, 0x10, 0x28, 0x44,// x
        0x0C, 0x50, 0x50, 0x50, 0x3C,// y
        0x44, 0x64, 0x54, 0x4C, 0x44,// z
        0x00, 0x08, 0x36, 0x41, 0x00,// {
        0x00, 0x00, 0x7F, 0x00, 0x00,// |
        0x00, 0x41, 0x36, 0x08, 0x00,// }
        0x08, 0x08, 0x2A, 0x1C, 0x08,// ->
        0x08, 0x1C, 0x2A, 0x08, 0x08 // <-
};

/* Function Prototypes */
static void TaskBuzzer(void *pvParameters);
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
//static void NVIC_Configuration(void);
static void SPI_Configuration(void);
static void LED_Configuration(void);
static void LED_WriteCommand(uint8_t command);
static void LED_WriteData(uint8_t address, uint8_t data);
static void LED_Update(void);
static void LED_SetLine(uint8_t line, const char *str);
#ifdef ENABLE_DELAY
static void Delay(__IO uint32_t nCount);
#endif

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * Main function
 */
int main()
{
	USART_InitTypeDef USART_InitStructure;

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t CCR1_Val = 500; // 50% duty cycle

	/* System clocks configuration */
	RCC_Configuration();

	/* GPIO configuration */
	GPIO_Configuration();

	/* NVIC configuration */
	//NVIC_Configuration();

	SPI_Configuration();
	/* LED configuration */

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(USART1, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);

	LED_Configuration();

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 999;
	TIM_TimeBaseStructure.TIM_Prescaler = 17; // 72MHz / ((17 + 1) * 1000) = 4KHz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	/* Create a FreeRTOS task */
	xTaskCreate(TaskBuzzer, (signed portCHAR *)"Buzzer", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY + 1, &xBuzzerTask);

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();

	/* If all is well we will never reach here as the scheduler will now be running. */
	/* If we do get here, it will most likely be because we ran out of heap space. */
	return 0;
}

/**
 * @brief  Configures the different system clocks.
 * @param  None
 * @retval None
 */
void RCC_Configuration(void)
{
	/* Setup the microcontroller system. Initialize the Embedded Flash Interface,
	   initialize the PLL and update the SystemFrequency variable. */
	SystemInit();

	/* PCLK2 = HCLK/2 */
	RCC_PCLK2Config(RCC_HCLK_Div2);

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* Enable GPIOC peripheral clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
				RCC_APB2Periph_GPIOC |
				RCC_APB2Periph_GPIOB |
				RCC_APB2Periph_USART1 |
				RCC_APB2Periph_AFIO,
				ENABLE);

	/* Enable SPI2 Periph clock */
	RCC_APB1PeriphClockCmd(SPI_CLK, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 * @param  None
 * @retval None
 */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Initialize GPIOB */
	GPIO_InitStructure.GPIO_Pin = SPI_PIN_CS;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Initialize GPIOC */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

	/* Configure SPI pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = SPI_PIN_SCK | SPI_PIN_MISO | SPI_PIN_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(SPI_GPIO, SPI_PIN_CS); // CS High

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

#if 0
/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* 1 bit for pre-emption priority, 3 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  /* Configure and enable SPI_MASTER interrupt -------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = SPI_MASTER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
#endif

#define INT_DIGITS 19           /* enough for 64 bit integer */

#if 0
itoa(i, a)
register int    i;
register char   *a;
{
	register char   *j;
	char            b[6];

	if (i < 0)
	{
		*a++ = '-';
		i = -i;
	}
	j = &b[5];
	*j-- = 0;
	do
	{
		*j-- = i % 10 + '0';
		i /= 10;
	} while (i);
	do
	{
		*a++ = *++j;
	} while (*j);
	return (0);
}
#endif

caddr_t _sbrk(int incr) {
	extern char _end;		/* Defined by the linker */
	static char *heap_end;
	char *prev_heap_end;

	if (heap_end == 0) {
		heap_end = &_end;
	}
	register caddr_t stack_ptr asm ("sp");
	prev_heap_end = heap_end;
	if (heap_end + incr > stack_ptr) {
		while(1);
	}

	heap_end += incr;
	return (caddr_t) prev_heap_end;
}
/**
 * @brief  Configures LED marquee.
 * @param  None
 * @retval None
 */
void LED_Configuration(void)
{
	int i;
	//int j;
	//int8_t c = 'T';
	LED_WriteCommand(0x01);
	LED_WriteCommand(0x03);
	LED_WriteCommand(0x2c);
	LED_WriteCommand(0xaf);

	for (i = 0; i < 96; i++)
	{
		LED_WriteData(i, 0x0);
	}

	LED_SetLine(0, "Test");

#if 0
	for (i = 0; i < LED_BUFFER_SIZE; i++)
	{
		line1[i] = 0xaa;
		line2[i] = 0xff;
	}
	//LED_SetLine(0, "Test");
	i = 0;
	uint32_t index = ('T' - ' ') * 5;
	//char test[] = "test";
	char *ch = itoa(index);
	//char *ch = test;
	while (*ch != '\0')
	{
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, *ch++);
	}

		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, ' ');

	ch = itoa(font5x7[index]);
	//char *ch = test;
	while (*ch != '\0')
	{
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, *ch++);
	}
	for (i = 0; i < index; i++)
	{
		USART_SendData(USART1, '1');
		/* Wait the byte is entirely sent to USART1 */  
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	}

	//uint8_t index = (c - (int8_t)' ');
	//uint8_t index = 5;
	line1[i++] = font5x7[index];
	line1[i++] = font5x7[index + 1];
	line1[i++] = font5x7[index + 2];
	line1[i++] = font5x7[index + 3];
	line1[i++] = font5x7[index + 4];
	i++;

	index = 10;

	line1[i++] = font5x7[index];
	line1[i++] = font5x7[index + 1];
	line1[i++] = font5x7[index + 2];
	line1[i++] = font5x7[index + 3];
	line1[i++] = font5x7[index + 4];
	i++;

	index = 15;

	line1[i++] = font5x7[index];
	line1[i++] = font5x7[index + 1];
	line1[i++] = font5x7[index + 2];
	line1[i++] = font5x7[index + 3];
	line1[i++] = font5x7[index + 4];
	i++;

	index = 20;

	line1[i++] = font5x7[index];
	line1[i++] = font5x7[index + 1];
	line1[i++] = font5x7[index + 2];
	line1[i++] = font5x7[index + 3];
	line1[i++] = font5x7[index + 4];
	i++;


	i = 0;
	j = 0;
	line1[i++] = font5x7[j++];
	line1[i++] = font5x7[j++];
	line1[i++] = font5x7[j++];
	line1[i++] = font5x7[j++];
	line1[i++] = font5x7[j++];
	i++;
	line1[i++] = font5x7[j++];
	line1[i++] = font5x7[j++];
	line1[i++] = font5x7[j++];
	line1[i++] = font5x7[j++];
	line1[i++] = font5x7[j++];
	i++;
	line1[i++] = font5x7[j++];
	line1[i++] = font5x7[j++];
	line1[i++] = font5x7[j++];
	line1[i++] = font5x7[j++];
	line1[i++] = font5x7[j++];
	i++;
	line1[i++] = font5x7[j++];
	line1[i++] = font5x7[j++];
	line1[i++] = font5x7[j++];
	line1[i++] = font5x7[j++];
	line1[i++] = font5x7[j++];
	i++;
#endif

	LED_Update();
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

#if 0
itoa(i, a)
	register int    i;
	register char   *a;
{
	register char   *j;
	char            b[6];

	if (i < 0)
	{
		*a++ = '-';
		i = -i;
	}
	j = &b[5];
	*j-- = 0;
	do
	{
		*j-- = i % 10 + '0';
		i /= 10;
	} while (i);
	do
	{
		*a++ = *++j;
	} while (*j);
	return (0);
}
#endif

/**
 * @brief  Updates all the LEDs
 * @param  None
 * @retval None
 */
void LED_SetLine(uint8_t line, const char *str)
{
	uint8_t *buffer = line ? line2 : line1;

	/* Clear the line */
	memset(buffer, 0, LED_BUFFER_SIZE);

	if (str != NULL)
	{
		uint8_t i = 0;

		while ((*str != 0) && (i < 24))
		{
			uint32_t value;
			int k = (*str - 32);
			if (k < 32 || k > 127)
			{
				k = 0;
			}
			k *= 5;

#if 1
			static char test[80];
			char *ch = test;
			siprintf(test, "%d", 1234);
			while (*ch != '\0')
			{
				while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
				USART_SendData(USART1, *ch);
				ch++;
			}
#endif

			value = font5x7[k] << 24;
			asm ("rbit %0, %1;" : "=r" (value) : "r" (value)); /* Bit reverse */
			//value = 0x80;
			buffer[i++] = (uint8_t)value;

			value = font5x7[k + 1];
			asm ("rbit %0, %1;" : "=r" (value) : "r" (value)); /* Bit reverse */
			//value = 0x80;
			buffer[i++] = (uint8_t)value;

			value = font5x7[k + 2];
			asm ("rbit %0, %1;" : "=r" (value) : "r" (value)); /* Bit reverse */
			//value = 0xfe;
			buffer[i++] = (uint8_t)value;

			value = font5x7[k + 3];
			asm ("rbit %0, %1;" : "=r" (value) : "r" (value)); /* Bit reverse */
			//value = 0x80;
			buffer[i++] = (uint8_t)value;

			value = font5x7[k + 4];
			asm ("rbit %0, %1;" : "=r" (value) : "r" (value)); /* Bit reverse */
			//value = 0x80;
			buffer[i++] = (uint8_t)value;
			i++;

			str++;
		}
	}
}

/**
 * @brief  Updates all the LEDs
 * @param  None
 * @retval None
 */
void LED_Update(void)
{
	int i;
	int j;

	/* Line 1 */
	j = 0;
	for (i = 0x1d; i >= 1; i-=4, j++)
	{
		LED_WriteData(i, (line1[j]) & 0xf);
		LED_WriteData(i - 1, (line1[j] >> 4) & 0xf);
	}
	for (i = 0x3d; i >= 0x21; i-=4, j++)
	{
		LED_WriteData(i, (line1[j]) & 0xf);
		LED_WriteData(i - 1, (line1[j] >> 4) & 0xf);
	}
	for (i = 0x5d; i >= 0x41; i-=4, j++)
	{
		LED_WriteData(i, (line1[j]) & 0xf);
		LED_WriteData(i - 1, (line1[j] >> 4) & 0xf);
	}

	/* Line 2 */
	j = 0;
	for (i = 0x1f; i >= 1; i-=4, j++)
	{
		LED_WriteData(i, (line2[j]) & 0xf);
		LED_WriteData(i - 1, (line2[j] >> 4) & 0xf);
	}
	for (i = 0x3f; i >= 0x21; i-=4, j++)
	{
		LED_WriteData(i, (line2[j]) & 0xf);
		LED_WriteData(i - 1, (line2[j] >> 4) & 0xf);
	}
	for (i = 0x5f; i >= 0x41; i-=4, j++)
	{
		LED_WriteData(i, (line2[j]) & 0xf);
		LED_WriteData(i - 1, (line2[j] >> 4) & 0xf);
	}
}


/**
 * @brief  Configures the different SPI ports.
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
	SPI_I2S_ITConfig(SPI, SPI_I2S_IT_TXE, ENABLE);

	/* Enable SPI */
	SPI_Cmd(SPI, ENABLE);
}

/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
#ifdef ENABLE_DELAY
void Delay(__IO uint32_t nCount)
{
	for(; nCount != 0; nCount--);
}
#endif

void TaskBuzzer(void *pvParameters)
{
	portTickType xLastWakeTime;

	/* Initialise the xLastExecutionTime variable on task entry */
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		vTaskSuspend(NULL);

		for(; beep_count > 0; beep_count--)
		{
			/* TIM3 enable counter */
			TIM_Cmd(TIM3, ENABLE);
			vTaskDelayUntil(&xLastWakeTime, TASK_SPIN_DELAY);
			/* TIM3 disable counter */
			TIM_Cmd(TIM3, DISABLE);
			vTaskDelayUntil(&xLastWakeTime, TASK_SPIN_DELAY);
		}
	}
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

/**
* Idle hook function
*/
void vApplicationIdleHook(void)
{
	/* Called when the scheduler has no tasks to run */
}

