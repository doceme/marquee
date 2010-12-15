/**
 *****************************************************************************
 *
 * @file       remote.c
 * @author     Stephen Caudle Copyright (C) 2010
 * @brief      Remote implementation
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
#include "remote.h"
#include "tprintf.h"

/* Defines */
#define REMOTE_NVIC_PRIO	IRQ_PRIO_HIGHEST

#define DEFAULT_BEEP_DURATION	100 /* 500ms = 1/2 second */

#define REMOTE_APB1_PERIPH 	RCC_APB1Periph_TIM4
#define REMOTE_APB2_PERIPH 	RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO

#define REMOTE_GPIO		GPIOB
#define REMOTE_GPIO_PIN		GPIO_Pin_6
#define REMOTE_GPIO_SPEED	GPIO_Speed_50MHz
#define REMOTE_GPIO_MODE	GPIO_Mode_IN_FLOATING
#define REMOTE_IRQn		EXTI9_5_IRQn
#define REMOTE_STATE_IRQn	TIM4_IRQn
#define REMOTE_IT		TIM_IT_Update

#define REMOTE_TIMER		TIM4
#define REMOTE_STATE_FREQ	1000000

#define REMOTE_PULSE_TOLERANCE	15
#define REMOTE_PULSE_RISE_MASK	0x8000
#define REMOTE_PULSE_TIME_MASK	0x7fff
#define REMOTE_PULSE_LAST	REMOTE_PULSE_TIME_MASK
#define REMOTE_PULSE_RISING(t)	(t >> 15)
#define REMOTE_PULSE_TIME(t)	(t & REMOTE_PULSE_TIME_MASK)

enum remote_state_t
{
	REMOTE_STATE_IDLE,
	REMOTE_STATE_MEASURE
};

enum remote_field_t
{
	REMOTE_FIELD_NONE,
	REMOTE_FIELD_LEADER,
	REMOTE_FIELD_START,
	REMOTE_FIELD_MODE,
	REMOTE_FIELD_TRAILER,
	REMOTE_FIELD_CONTROL,
	REMOTE_FIELD_INFO
};

#define REMOTE_PULSE_UNKNOWN	0
#define REMOTE_PULSE_1T		47
#define REMOTE_PULSE_2T		85
#define REMOTE_PULSE_3T		132
#define REMOTE_PULSE_LEADER	271
#define REMOTE_PULSE_MAX	500

#if 0
static uint16_t codes[][] =
{
	{
		REMOTE_PULSE_LEADER | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_2T,
		REMOTE_PULSE_1T | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_2T,
		REMOTE_PULSE_1T | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_1T,
		REMOTE_PULSE_1T | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_1T,
		REMOTE_PULSE_3T | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_3T,
		REMOTE_PULSE_1T | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_1T,
		REMOTE_PULSE_1T | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_1T,
		REMOTE_PULSE_1T | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_1T,
		REMOTE_PULSE_1T | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_1T,
		REMOTE_PULSE_2T | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_2T,
		REMOTE_PULSE_1T | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_1T,
		REMOTE_PULSE_1T | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_1T,
		REMOTE_PULSE_2T | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_2T,
		REMOTE_PULSE_2T | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_1T,
		REMOTE_PULSE_1T | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_1T,
		REMOTE_PULSE_1T | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_2T,
		REMOTE_PULSE_1T | REMOTE_PULSE_RISE_MASK,
		REMOTE_PULSE_1T,
		REMOTE_PULSE_1T | REMOTE_PULSE_RISE_MASK
	}
	//^LRv2T^1Tv2T^1Tv1T^1Tv1T^1Tv2T^2Tv1T^1Tv1T^1Tv1T^1Tv1T^1Tv1T^2Tv2T^1Tv1T^1Tv1T^2Tv2T^2Tv1T^1Tv1T^1Tv2T^1Tv1T^1T
	//^LRv2T^1Tv2T^1Tv1T^1Tv1T^1Tv2T^2Tv1T^1Tv1T^1Tv1T^1Tv1T^1Tv1T^2Tv2T^1Tv1T^1Tv1T^2Tv2T^2Tv1T^1Tv1T^1Tv2T^1Tv1T^1T
};
#endif

struct remote_pulse_t
{
	enum remote_field_t field;
	uint16_t curr;
	uint16_t prev;
	uint8_t rising;
	uint8_t index;
	uint8_t mode;
	uint8_t control;
	uint8_t info;
};

/* Local Variables */
static enum remote_state_t remote_state;
static uint32_t remote_count;
static xQueueHandle timer_queue = NULL;
static xTaskHandle xRemoteTask = NULL;

/* Function Prototypes */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void EXTI_Configuration(void);
static void NVIC_Configuration(void);
static void Timer_Configuration(uint32_t frequency);
static void remote_task(void *pvParameters);
static uint16_t remote_get_pulse(uint16_t time);
static void remote_print_pulse(struct remote_pulse_t *pulse, uint16_t time);
static void remote_decode_pulse(struct remote_pulse_t *pulse);

int Remote_Configuration(void)
{
	RCC_Configuration();
	GPIO_Configuration();
	EXTI_Configuration();
	NVIC_Configuration();
	Timer_Configuration(REMOTE_STATE_FREQ);

	xTaskCreate(remote_task, (signed portCHAR *)"Remote", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY + 1, &xRemoteTask);
	assert_param(xRemoteTask);

	timer_queue = xQueueCreate(256, sizeof(uint16_t));
	assert_param(timer_queue);

	return 0;
}

/**
 * @brief  Configures the different system clocks
 * @param  None
 * @retval None
 */
void RCC_Configuration(void)
{
#ifdef REMOTE_APB1_PERIPH
	/* Timer clock enable */
	RCC_APB1PeriphClockCmd(REMOTE_APB1_PERIPH, ENABLE);
#endif

#ifdef REMOTE_APB2_PERIPH
	/* Enable GPIO peripheral clock */
	RCC_APB2PeriphClockCmd(REMOTE_APB2_PERIPH, ENABLE);
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
	GPIO_InitStructure.GPIO_Pin = REMOTE_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = REMOTE_GPIO_SPEED;
	GPIO_InitStructure.GPIO_Mode = REMOTE_GPIO_MODE;
	GPIO_Init(REMOTE_GPIO, &GPIO_InitStructure);

	/* Connect Button EXTI Line to Button GPIO Pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);
}

/**
  * @brief  Configures EXTI Lines
  * @param  None
  * @retval None
  */
void EXTI_Configuration(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_ClearITPendingBit(EXTI_Line6);
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
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

	/* Enable the remote pin interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = REMOTE_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = REMOTE_NVIC_PRIO;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the remote timer interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = REMOTE_STATE_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = REMOTE_NVIC_PRIO;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Configures the buzzer timer
 * @param  None
 * @retval None
 */
void Timer_Configuration(uint32_t frequency)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_ClocksTypeDef RCC_ClockFreq;
	uint16_t CCR_Val;

	/* Set CCR value based on PCLK1 frequency and desired buzzer frequency */
	RCC_GetClocksFreq(&RCC_ClockFreq);
	CCR_Val = (RCC_ClockFreq.HCLK_Frequency / frequency);

	/* Count timer base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = REMOTE_PULSE_MAX - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = CCR_Val * 10 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(REMOTE_TIMER, &TIM_TimeBaseStructure);

	/* Clear remote timer interrupt pending bit */
	TIM_ClearITPendingBit(REMOTE_TIMER, REMOTE_IT);

	/* Enable toggle timer interrupt */
	TIM_ITConfig(REMOTE_TIMER, REMOTE_IT, ENABLE);

	/* Disable timer counters */
	TIM_Cmd(REMOTE_TIMER, DISABLE);
}

/**
  * @brief  This function handles External lines 9 to 5 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken;

	if(EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		switch (remote_state)
		{
			case REMOTE_STATE_IDLE:
			{
				TIM_SetCounter(REMOTE_TIMER, 0);
				remote_state = REMOTE_STATE_MEASURE;

				/* Enable timer counters */
				TIM_Cmd(REMOTE_TIMER, ENABLE);
			} break;

			case REMOTE_STATE_MEASURE:
			{
				uint16_t data;

				remote_count = TIM_GetCounter(REMOTE_TIMER);
				TIM_SetCounter(REMOTE_TIMER, 0);

				data = remote_count | (GPIO_ReadInputDataBit(REMOTE_GPIO, REMOTE_GPIO_PIN) << 15);

				xQueueSendToBackFromISR(timer_queue, &data, &xHigherPriorityTaskWoken);
			} break;

			default:
			{
				remote_state = REMOTE_STATE_IDLE;

				/* Disable timer counters */
				TIM_Cmd(REMOTE_TIMER, DISABLE);
			} break;
		}

		/* Clear the Key Button EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line6);
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void TIM4_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken;

	if (TIM_GetITStatus(REMOTE_TIMER, REMOTE_IT) != RESET)
	{
		switch (remote_state)
		{
			case REMOTE_STATE_MEASURE:
			{
				uint16_t data = REMOTE_PULSE_LAST;

				/* Disable timer counters */
				TIM_Cmd(REMOTE_TIMER, DISABLE);

				remote_state = REMOTE_STATE_IDLE;
				xQueueSendToBackFromISR(timer_queue, &data, &xHigherPriorityTaskWoken);
			} break;

			default:
			{
				/* Disable timer counters */
				TIM_Cmd(REMOTE_TIMER, DISABLE);

				remote_state = REMOTE_STATE_IDLE;
			} break;
		}

		/* Clear remote timer interrupt pending bit */
		TIM_ClearITPendingBit(REMOTE_TIMER, REMOTE_IT);
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static void remote_print_pulse(struct remote_pulse_t *pulse, uint16_t time)
{
	if (pulse->rising)
		tprintf("^");
	else
		tprintf("v");

	tprintf("(%d,%d)", time, pulse->curr);
	//tprintf("%d,", abs(time - pulse->curr));
#if 0
	switch (pulse->curr)
	{
		case REMOTE_PULSE_LEADER:
			tprintf("LR");
			break;

		case REMOTE_PULSE_1T:
			tprintf("1T");
			break;

		case REMOTE_PULSE_2T:
			tprintf("2T");
			break;

		case REMOTE_PULSE_3T:
			tprintf("3T");
			break;

		default:
			tprintf("UN");
			break;
	}
#endif
}

static uint16_t remote_get_pulse(uint16_t time)
{
	uint16_t result = REMOTE_PULSE_UNKNOWN;

	if (abs(time - REMOTE_PULSE_1T) <= REMOTE_PULSE_TOLERANCE)
	{
		result = REMOTE_PULSE_1T;
	}
	else if (abs(time - REMOTE_PULSE_2T) <= REMOTE_PULSE_TOLERANCE)
	{
		result = REMOTE_PULSE_2T;
	}
	else if (abs(time - REMOTE_PULSE_3T) <= REMOTE_PULSE_TOLERANCE)
	{
		result = REMOTE_PULSE_3T;
	}
	else if (abs(time - REMOTE_PULSE_LEADER) <= REMOTE_PULSE_TOLERANCE)
	{
		result = REMOTE_PULSE_LEADER;
	}

	return result;
}

static void remote_decode_pulse(struct remote_pulse_t *pulse)
{
	switch(pulse->field)
	{
		case REMOTE_FIELD_NONE:
		{
			if (pulse->curr == REMOTE_PULSE_LEADER && pulse->rising)
			{
				pulse->field = REMOTE_FIELD_LEADER;
			}
		} break;

		case REMOTE_FIELD_LEADER:
		{
			if (pulse->curr == REMOTE_PULSE_2T && !pulse->rising)
			{
				pulse->field = REMOTE_FIELD_START;
			}
			else
			{
				pulse->field = REMOTE_FIELD_NONE;
			}
		} break;

		case REMOTE_FIELD_START:
		{
			if (pulse->curr == REMOTE_PULSE_2T &&
				pulse->prev == REMOTE_PULSE_1T &&
				pulse->rising)
			{
				pulse->index++;
				pulse->field = REMOTE_FIELD_MODE;
			}
			else if (pulse->curr == REMOTE_PULSE_1T &&
				pulse->prev == REMOTE_PULSE_1T &&
				!pulse->rising)
			{
				pulse->index = 0;
				pulse->field = REMOTE_FIELD_MODE;
			}
			else
			{
				pulse->index = 0;
				pulse->field = REMOTE_FIELD_NONE;
			}
		} break;

		case REMOTE_FIELD_MODE:
		{
			tprintf("M");
			pulse->field = REMOTE_FIELD_NONE;
#if 0
			if (pulse->curr == REMOTE_PULSE_1T && pulse->prev != REMOTE_PULSE_2T)
			{
				pulse->index++;
			}
			else if (pulse->curr != REMOTE_PULSE_1T)
			{
				pulse->field = REMOTE_FIELD_NONE;
			}
#endif
		} break;

		case REMOTE_FIELD_TRAILER:
		{
		} break;

		case REMOTE_FIELD_CONTROL:
		{
		} break;

		case REMOTE_FIELD_INFO:
		{
		} break;

		default:
		{
		} break;
	}
}

void remote_task(void *pvParameters)
{
	portTickType xLastWakeTime;
	portTickType wait = portMAX_DELAY;
	struct remote_pulse_t pulse;

	/* Initialise the xLastExecutionTime variable on task entry */
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		uint16_t time;
		pulse.prev = pulse.curr;

		if (xQueueReceive(timer_queue, &time, wait))
		{
			pulse.rising = REMOTE_PULSE_RISING(time);
			pulse.curr = remote_get_pulse(REMOTE_PULSE_TIME(time));

			remote_decode_pulse(&pulse);

			if (time != REMOTE_PULSE_LAST)
			{
				wait = 10 / portTICK_RATE_MS;
				remote_print_pulse(&pulse, REMOTE_PULSE_TIME(time));
			}
			else
			{
				wait = portMAX_DELAY;
				tprintf("\r\n");
			}
		}
		else if (wait != portMAX_DELAY)
		{
			pulse.field = REMOTE_FIELD_NONE;
			pulse.curr = 0;
			pulse.index = 0;
			wait = portMAX_DELAY;
			tprintf("\r\n");
		}
	}
}
