/**
 ******************************************************************************
 *
 * @file       marquee.c
 * @author     Stephen Caudle Copyright (C) 2010
 * @brief      Main marquee implementation
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
#include "tprintf.h"
#include "buzzer.h"
#include "led.h"
#include "network.h"

#if 0
typedef enum NetworkState
{
	NotConnected,
	Connected
} NetworkState;
#endif

/* Definitions */
#define DEBUG
#define BUSY_BIT_MARQUEE	0
#define SLEEP_TIME		10
#define DEFAULT_TIMEOUT		3000
#define MS_PER_SEC		1000

/* Global Variables */
volatile uint32_t busy = 0;
xSemaphoreHandle xBusyMutex = NULL;

/* Local Variables */
static volatile uint8_t program = 0;
static xTaskHandle xMainTask = NULL;
//static NetworkState networkState = NotConnected;

/* Function Prototypes */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void EXTI_Configuration(void);
static void RTC_Configuration(void);
static void NVIC_Configuration(void);
#ifndef DEBUG
static void SYSCLKConfig_STOP(void);
#endif
static void Main_Task(void *pvParameters);

void assert_failed(uint8_t *function, uint32_t line)
{
	tprintf("!\r\n");
	while (1);
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
int outbyte(int ch)
{
	USART_SendData(USART1, (uint8_t)ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	return ch;
}

/**
 * Main function
 */
int main()
{
	RCC_Configuration();
	GPIO_Configuration();
	EXTI_Configuration();
	RTC_Configuration();
	NVIC_Configuration();

	/* Wait for transmit enable flag to be set for debug prints */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

	xBusyMutex = xSemaphoreCreateMutex();
	assert_param(xBusyMutex);

	/* Create a FreeRTOS task */
	xTaskCreate(Main_Task, (signed portCHAR *)"Main", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY + 1, &xMainTask);
	assert_param(xMainTask);

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

	/* Enable PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 * @param  None
 * @retval None
 */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure button input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect Button EXTI Line to Button GPIO Pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11);
}

/**
  * @brief  Configures EXTI Lines
  * @param  None
  * @retval None
  */
void EXTI_Configuration(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	/* Configure EXTI Line17(RTC Alarm) to generate an interrupt on rising edge */
	EXTI_ClearITPendingBit(EXTI_Line17);
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	EXTI_ClearITPendingBit(EXTI_Line11);
	EXTI_InitStructure.EXTI_Line = EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

/**
  * @brief  Configures RTC clock source and prescaler
  * @param  None
  * @retval None
  */
void RTC_Configuration(void)
{
	/* RTC clock source configuration ------------------------------------------*/
	/* Allow access to BKP Domain */
	PWR_BackupAccessCmd(ENABLE);

	/* Reset Backup Domain */
	BKP_DeInit();

	/* Enable the LSE OSC */
	RCC_LSEConfig(RCC_LSE_ON);

	/* Wait till LSE is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
	{
	}

	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

	/* Enable the RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* RTC configuration -------------------------------------------------------*/
	/* Wait for RTC APB registers synchronisation */
	RTC_WaitForSynchro();

	/* Set the RTC time base to 1s */
	RTC_SetPrescaler(32767);

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Enable the RTC Alarm interrupt */
	RTC_ITConfig(RTC_IT_ALR, ENABLE);

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
}

/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Set the Vector Table base address as specified in .ld file */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	/* 4 bits for Interupt priorities so no sub priorities */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

	NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable and set Button EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configures system clock after wake-up from STOP: enable HSE, PLL
  *   and select PLL as system clock source
  * @param  None
  * @retval None
  */
#ifndef DEBUG
void SYSCLKConfig_STOP(void)
{
	ErrorStatus HSEStartUpStatus;

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus == SUCCESS)
	{

#ifdef STM32F10X_CL
		/* Enable PLL2 */ 
		RCC_PLL2Cmd(ENABLE);

		/* Wait till PLL2 is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
		{
		}
#endif

		/* Enable PLL */ 
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
		}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while(RCC_GetSYSCLKSource() != 0x08)
		{
		}
	}
}
#endif

/**
* Idle hook function
*/
void vApplicationIdleHook(void)
{
#ifndef DEBUG
		if (program || busy)
			return;

		/* Set wakeup time */
		RTC_SetAlarm(RTC_GetCounter() + SLEEP_TIME);

		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();

		/* Request to enter STOP mode with regulator in low power mode*/
		PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

		/* Configures system clock after wake-up from STOP: enable HSE, PLL and select
		   PLL as system clock source (HSE and PLL are disabled in STOP mode) */
		SYSCLKConfig_STOP();

		if (!program)
			vTaskResume(xMainTask);
#endif
}

/**
  * @brief  This function handles External lines 9 to 5 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	{
		program = 1;

		/* Clear the Key Button EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line11);
	}
}

/**
  * @brief  This function handles RTC Alarm interrupt request
  * @param  None
  * @retval None
  */
void RTCAlarm_IRQHandler(void)
{
	if(RTC_GetITStatus(RTC_IT_ALR) != RESET)
	{
		/* Clear EXTI line17 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line17);

		/* Check if the Wake-Up flag is set */
		if(PWR_GetFlagStatus(PWR_FLAG_WU) != RESET)
		{
			/* Clear Wake Up flag */
			PWR_ClearFlag(PWR_FLAG_WU);
		}

		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();

		/* Clear RTC Alarm interrupt pending bit */
		RTC_ClearITPendingBit(RTC_IT_ALR);

		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();
	}
}

void Main_Task(void *pvParameters)
{
	int result = 0;
	portTickType xLastWakeTime;
	char ip[16];

	/* Initialise the xLastExecutionTime variable on task entry */
	xLastWakeTime = xTaskGetTickCount();

	/* Set buzzer to 4KHz */
	result = Buzzer_Configuration(4000);
	assert_param(result >= 0);

	result = LED_Configuration();
	assert_param(result >= 0);

	result = Network_Configuration();
	assert_param(result >= 0);

	for(;;)
	{
		BUSY(MARQUEE);

		result = Network_SendWait("", "I/OK", DEFAULT_TIMEOUT);

		if (result == -ERR_TIMEOUT)
		{
			Buzzer_Beep(1);
			tprintf("Timeout!\n");
		}
		else
		{
			Network_SendGetByChar("IPA?", '\n', '\r', ip, DEFAULT_TIMEOUT);
		}

		IDLE(MARQUEE);
#ifndef DEBUG
		vTaskSuspend(NULL);
#else
		if (program)
		{
			vTaskSuspend(NULL);
		}
		else
		{
			vTaskDelay((SLEEP_TIME * MS_PER_SEC) / portTICK_RATE_MS);
		}
#endif
	}
}

