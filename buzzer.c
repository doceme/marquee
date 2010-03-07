/**
 ******************************************************************************
 *
 * @file       buzzer.c
 * @author     Stephen Caudle Copyright (C) 2010
 * @brief      Buzzer implementation
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
#include "buzzer.h"

#define BUZZER_NVIC_PRIO	IRQ_PRIO_HIGHEST

#define DEFAULT_BEEP_DURATION	100 /* 500ms = 1/2 second */

#define BUZZER_APB1_PERIPH 	RCC_APB1Periph_TIM3
#define BUZZER_APB2_PERIPH 	RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO

#define BUZZER_GPIO		GPIOC
#define BUZZER_GPIO_PIN		GPIO_Pin_6
#define BUZZER_GPIO_SPEED	GPIO_Speed_50MHz
#define BUZZER_GPIO_MODE	GPIO_Mode_AF_PP
#define BUZZER_GPIO_REMAP 	GPIO_FullRemap_TIM3

#define BUZZER_TIMER		TIM3

#define BUSY_BIT_BUZZER		1

//#define USE_TASK
#define USE_IRQ

/* Local Variables */
#ifdef USE_TASK
static volatile uint8_t beepCount = 0;
static xTaskHandle xBuzzerTask = NULL;
#endif
static uint32_t onDuration = DEFAULT_BEEP_DURATION / portTICK_RATE_MS;
static uint32_t offDuration = DEFAULT_BEEP_DURATION / portTICK_RATE_MS;

static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
static TIM_OCInitTypeDef  TIM_OCInitStructure;
static uint16_t CCR1_Val = 500; // 50% duty cycle

/* Function Prototypes */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void NVIC_Configuration(void);
static void Timer_Configuration(void);
#ifdef USE_TASK
static void Buzzer_Task(void *pvParameters);
#endif

int Buzzer_Configuration(void)
{
#ifdef USE_TASK
	if (xBuzzerTask)
		return -ERR_EXIST;
#endif

	RCC_Configuration();
	GPIO_Configuration();
	NVIC_Configuration();
	Timer_Configuration();

#ifdef USE_TASK
	/* Create a FreeRTOS task */
	xTaskCreate(Buzzer_Task, (signed portCHAR *)"Buzzer", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY + 2, &xBuzzerTask);
	assert_param(xBuzzerTask);
#endif

	return 0;
}

int Buzzer_SetOnDuration(uint32_t duration)
{
	onDuration = duration / portTICK_RATE_MS;
	return 0;
}

int Buzzer_SetOffDuration(uint32_t duration)
{
	offDuration = duration / portTICK_RATE_MS;
	return 0;
}

int Buzzer_Beep(uint32_t count)
{
#if defined(USE_TASK)
	BUSY(BUZZER);
	if (xBuzzerTask)
	{
		beepCount = count;
		vTaskResume(xBuzzerTask);
		return 0;
	}
	else
	{
		return -ERR_NOINIT;
	}
#elif defined(USE_IRQ)
#else
	for(; count > 0; count--)
	{
		/* Timer enable counter */
		TIM_Cmd(BUZZER_TIMER, ENABLE);
		//vTaskDelayUntil(&xLastWakeTime, onDuration);
		vTaskDelay(onDuration);

		/* Timer disable counter */
		TIM_Cmd(BUZZER_TIMER, DISABLE);

		if (count > 1)
		{
			//vTaskDelayUntil(&xLastWakeTime, offDuration);
			vTaskDelay(offDuration);
		}
	}
	return 0;
#endif
}

int Buzzer_IsBeeping(void)
{
#ifdef USE_TASK
	return (beepCount == 0 ? 0 : 1);
#else
	return 0;
#endif
}

/**
 * @brief  Configures the different system clocks
 * @param  None
 * @retval None
 */
void RCC_Configuration(void)
{
#ifdef BUZZER_APB1_PERIPH
	/* Timer clock enable */
	RCC_APB1PeriphClockCmd(BUZZER_APB1_PERIPH, ENABLE);
#endif

#ifdef BUZZER_APB2_PERIPH
	/* Enable GPIO peripheral clock */
	RCC_APB2PeriphClockCmd(BUZZER_APB2_PERIPH, ENABLE);
#endif
}

/**
 * @brief  Configures the different GPIO ports
 * @param  None
 * @retval None
 */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Initialize GPIO */
	GPIO_InitStructure.GPIO_Pin = BUZZER_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = BUZZER_GPIO_SPEED;
	GPIO_InitStructure.GPIO_Mode = BUZZER_GPIO_MODE;
	GPIO_Init(BUZZER_GPIO, &GPIO_InitStructure);

#ifdef BUZZER_GPIO_REMAP
	GPIO_PinRemapConfig(BUZZER_GPIO_REMAP, ENABLE);
#endif
}

/**
  * @brief  Configure the NVIC for the buzzer
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
	/* Configure the buzzer Interrupts */
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the buzzer Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = BUZZER_NVIC_PRIO;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Configures the buzzer timer
 * @param  None
 * @retval None
 */
void Timer_Configuration(void)
{

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 999;
	TIM_TimeBaseStructure.TIM_Prescaler = 17; /* 72MHz / ((17 + 1) * 1000) = 4KHz */
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(BUZZER_TIMER, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(BUZZER_TIMER, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(BUZZER_TIMER, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(BUZZER_TIMER, ENABLE);

	/* Enable timer interrupt */
	TIM_ITConfig(BUZZER_TIMER, TIM_IT_CC1, ENABLE);
}

#ifdef USE_TASK
void Buzzer_Task(void *pvParameters)
{
	portTickType xLastWakeTime;

	/* Initialise the xLastExecutionTime variable on task entry */
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		vTaskSuspend(NULL);
		for(; beepCount > 0; beepCount--)
		{
			/* Timer enable counter */
			TIM_Cmd(BUZZER_TIMER, ENABLE);
			vTaskDelayUntil(&xLastWakeTime, onDuration);

			/* Timer disable counter */
			TIM_Cmd(BUZZER_TIMER, DISABLE);

			if (beepCount > 1)
				vTaskDelayUntil(&xLastWakeTime, offDuration);
		}

		IDLE(BUZZER);
	}
}
#endif

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(BUZZER_TIMER, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(BUZZER_TIMER, TIM_IT_CC1);
	}
}

