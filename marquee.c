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

/* Definitions */
#define TASK_SPIN_DELAY             (500 / portTICK_RATE_MS)
#define NUMBER_OF_BEEPS             3

#define SPI					SPI2
#define SPI_CLK				RCC_APB1Periph_SPI2
#define SPI_GPIO			GPIOB
#define SPI_GPIO_CLK		RCC_APB2Periph_GPIOB
#define SPI_PIN_SCK			GPIO_Pin_13
#define SPI_PIN_MISO		GPIO_Pin_14
#define SPI_PIN_MOSI		GPIO_Pin_15

#define BEEP(n)		do { beep_count = n; vTaskResume(xBuzzerTask); } while (0);

/* Global Variables */
static uint8_t beep_count = 0;
static xTaskHandle xBuzzerTask = NULL;

/* Local Variables */

/* Function Prototypes */
static void TaskBuzzer(void *pvParameters);
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
#ifdef ENABLE_DELAY
static void Delay(__IO uint32_t nCount);
#endif

/**
 * Main function
 */
int main()
{

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t CCR1_Val = 500; // 50% duty cycle

	/* System clocks configuration */
	RCC_Configuration();

	/* GPIO configuration */
	GPIO_Configuration();

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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);


#if ENABLE_SPI
	/* Enable peripheral clocks --------------------------------------------------*/
	/* Enable GPIO clock for SPI */
	RCC_APB2PeriphClockCmd(SPI_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

	/* Enable SPI2 Periph clock */
	RCC_APB1PeriphClockCmd(SPI_CLK, ENABLE);
#endif
}

/**
 * @brief  Configures the different GPIO ports.
 * @param  None
 * @retval None
 */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Initialize GPIOC */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

#if ENABLE_SPI
	/* Enable SPI Pins Software Remapping */
	GPIO_PinRemapConfig(GPIO_Remap_SPI2, ENABLE);

	/* Configure SPI pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = SPI_PIN_SCK | SPI_PIN_MISO | SPI_PIN_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
#endif
}

/**
 * @brief  Configures the different SPI ports.
 * @param  None
 * @retval None
 */
#if ENABLE_SPI
void SPI_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStructure;

	/* SPI_MASTER configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI_MASTER, &SPI_InitStructure);
}
#endif

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
* Idle hook function
*/
void vApplicationIdleHook(void)
{
	static uint8_t once = 1;

	if (once)
	{
		once = 0;
	}
	/* Called when the scheduler has no tasks to run */
}

