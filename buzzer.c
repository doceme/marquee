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

/* Defines */
#define BUZZER_NVIC_PRIO	IRQ_PRIO_HIGHEST

#define DEFAULT_BEEP_DURATION	100 /* 500ms = 1/2 second */

#define BUZZER_APB1_PERIPH 	RCC_APB1Periph_TIM5
#define BUZZER_APB2_PERIPH 	RCC_APB2Periph_TIM8 | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO

#define BUZZER_GPIO		GPIOC
#define BUZZER_GPIO_PIN		GPIO_Pin_6
#define BUZZER_GPIO_SPEED	GPIO_Speed_50MHz
#define BUZZER_GPIO_MODE	GPIO_Mode_AF_PP
#define BUZZER_PWM_IRQn		TIM8_UP_IRQn
#define BUZZER_TOGGLE_IRQn	TIM5_IRQn
#define BUZZER_IT		TIM_IT_Update
#define BUZZER_OC_INIT		TIM_OC1Init
#define BUZZER_PRELOAD_CONFIG	TIM_OC1PreloadConfig

#define BUZZER_PWM_TIMER		TIM8
#define BUZZER_TOGGLE_TIMER		TIM5

#define BUSY_BIT_BUZZER		1

/* Local Variables */
static volatile uint32_t beepCount = 0;
static uint32_t onDuration = DEFAULT_BEEP_DURATION;
static uint32_t offDuration = DEFAULT_BEEP_DURATION;
static uint32_t curOnDuration;
static uint32_t curOffDuration;

/* Function Prototypes */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void NVIC_Configuration(void);
static void Timer_Configuration(uint16_t frequency);

int Buzzer_Configuration(uint16_t frequency)
{
	RCC_Configuration();
	GPIO_Configuration();
	NVIC_Configuration();
	Timer_Configuration(frequency);

	return 0;
}

int Buzzer_SetOnDuration(uint32_t duration)
{
	while (Buzzer_IsBeeping());
	onDuration = duration;
	return 0;
}

int Buzzer_SetOffDuration(uint32_t duration)
{
	while (Buzzer_IsBeeping());
	offDuration = duration;
	return 0;
}

int Buzzer_Beep(uint32_t count)
{
	beepCount = count;
	curOnDuration = onDuration;
	curOffDuration = offDuration;

	/* Enable toggle timer interrupt */
	TIM_ITConfig(BUZZER_TOGGLE_TIMER, BUZZER_IT, ENABLE);

	/* Enable PWM output */
	TIM_CtrlPWMOutputs(BUZZER_PWM_TIMER, ENABLE);

	/* Enable timer counters */
	TIM_Cmd(BUZZER_PWM_TIMER, ENABLE);
	TIM_Cmd(BUZZER_TOGGLE_TIMER, ENABLE);
	return 0;
}

inline int Buzzer_IsBeeping(void)
{
	return (beepCount == 0 ? 0 : 1);
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
	NVIC_InitStructure.NVIC_IRQChannel = BUZZER_TOGGLE_IRQn;
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
void Timer_Configuration(uint16_t frequency)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_ClocksTypeDef RCC_ClockFreq;
	uint16_t CCR_Val;

	/* Set CCR value based on PCLK1 frequency and desired buzzer frequency */
	RCC_GetClocksFreq(&RCC_ClockFreq);
	CCR_Val = (RCC_ClockFreq.PCLK1_Frequency / frequency);

	/* PWM timer base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = (CCR_Val * 2) - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(BUZZER_PWM_TIMER, &TIM_TimeBaseStructure);

	/* Toggle timer base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = (RCC_ClockFreq.PCLK1_Frequency / 1000) - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(BUZZER_TOGGLE_TIMER, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR_Val - 1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
	BUZZER_OC_INIT(BUZZER_PWM_TIMER, &TIM_OCInitStructure);

	BUZZER_PRELOAD_CONFIG(BUZZER_PWM_TIMER, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(BUZZER_PWM_TIMER, ENABLE);

	/* Disable PWM output */
	TIM_CtrlPWMOutputs(BUZZER_PWM_TIMER, DISABLE);

	/* Disable timer counters */
	TIM_Cmd(BUZZER_PWM_TIMER, DISABLE);
	TIM_Cmd(BUZZER_TOGGLE_TIMER, DISABLE);
}

void TIM5_IRQHandler(void)
{
	if (TIM_GetITStatus(BUZZER_TOGGLE_TIMER, BUZZER_IT) != RESET)
	{
		if (beepCount)
		{
			if (curOnDuration)
			{
				curOnDuration--;
			}
			else if (curOffDuration)
			{
				if (curOffDuration == offDuration)
				{
					TIM_CtrlPWMOutputs(BUZZER_PWM_TIMER, DISABLE);
				}

				curOffDuration--;
			}
			else
			{
				beepCount--;

				if (beepCount)
				{
					curOnDuration = onDuration;
					curOffDuration = offDuration;
					TIM_CtrlPWMOutputs(BUZZER_PWM_TIMER, ENABLE);
				}
			}
		}
		else
		{
			/* Disable timer interrupt */
			TIM_ITConfig(BUZZER_PWM_TIMER, BUZZER_IT, DISABLE);

			/* Disable timer counters */
			TIM_Cmd(BUZZER_PWM_TIMER, DISABLE);
			TIM_Cmd(BUZZER_TOGGLE_TIMER, DISABLE);
		}

		TIM_ClearITPendingBit(BUZZER_TOGGLE_TIMER, BUZZER_IT);
	}
}

