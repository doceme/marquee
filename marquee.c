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
#include "remote.h"
#include "string.h"

typedef enum NetworkState
{
	NetworkState_NotConnected,
	NetworkState_Connected,
	NetworkState_Online,
	NetworkState_Last
} NetworkState;

/* Definitions */
#define DEBUG
//#define DEBUG_AMBIENT
#define BUSY_BIT_MARQUEE	0
#define SLEEP_TIME		10
#define DEFAULT_TIMEOUT		3000
#define MS_PER_SEC		1000

/* Global Variables */
volatile uint32_t busy = 0;
xSemaphoreHandle xBusyMutex = NULL;
__IO uint16_t ADCConvertedValue;

/* Local Variables */
static volatile uint8_t program = 0;
static xTaskHandle xMainTask = NULL;
static NetworkState networkState = NetworkState_NotConnected;
static NetworkWlanConnection_t networkConnection;
static char response[80];
static const uint32_t sleepDurations[NetworkState_Last] = { 3, 3, 20 };
static uint32_t sleepDuration;
static uint8_t showTime = 0;
static uint8_t validTime = 0;
static NetworkDateTime_t dateTime;
static const uint32_t ADC1_DR_Address = 0x4001244C;

static const uint16_t brightnessToADC[LedBrightness_Last - 1] = {
	20,	// LedBrightness_0
	40,	// LedBrightness_1
	60,	// LedBrightness_2
	80,	// LedBrightness_3
	100,	// LedBrightness_4
	120,	// LedBrightness_5
	140,	// LedBrightness_6
	160,	// LedBrightness_7
	165,	// LedBrightness_8
	170,	// LedBrightness_9
	175,	// LedBrightness_10
	180,	// LedBrightness_11
	185,	// LedBrightness_12
	190,	// LedBrightness_13
	195,	// LedBrightness_14
};

#if 0
static xQueueHandle xQueue = NULL;
#endif

/* Function Prototypes */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void EXTI_Configuration(void);
static void RTC_Configuration(void);
static void DMA_Configuration(void);
static void ADC_Configuration(void);
static void NVIC_Configuration(void);
#ifndef DEBUG
static void SYSCLKConfig_STOP(void);
#endif
static inline void ShowTime(void);
static void GetDateTime(void);
static LedBrightness GetLedBrightnessFromADC(uint16_t value);
static void Main_Task(void *pvParameters) NORETURN;
static void main_noreturn(void) NORETURN;

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
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, (uint8_t)ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	return ch;
}

/**
 * Main function
 */
int main(void)
{
	main_noreturn();
}

inline void main_noreturn(void)
{
	RCC_Configuration();
	GPIO_Configuration();
	EXTI_Configuration();
	RTC_Configuration();
	DMA_Configuration();
	ADC_Configuration();
	NVIC_Configuration();

	/* Create a FreeRTOS task */
	xTaskCreate(Main_Task, (signed portCHAR *)"Main", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY + 1, &xMainTask);
	assert_param(xMainTask);

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();

	while (1);
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

	/* Enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* Enable PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 * @param  None
 * @retval None
 */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure PC5 (ADC Channel15) as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

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

#ifndef DEBUG
	/* Configure EXTI Line17(RTC Alarm) to generate an interrupt on rising edge */
	EXTI_ClearITPendingBit(EXTI_Line17);
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
#endif

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

	/* Set the RTC time base to 1min */
	RTC_SetPrescaler(32767);

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Enable the RTC Alarm interrupt */
	//RTC_ITConfig(RTC_IT_ALR, ENABLE);

	/* Enable the RTC second interrupt */
	RTC_ITConfig(RTC_IT_SEC, ENABLE);

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
}

/**
  * @brief  Configure the DMA controller.
  * @param  None
  * @retval None
  */
void DMA_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	/* DMA1 channel1 configuration */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* Enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

/**
  * @brief  Configure the analog to digital converter controller.
  * @param  None
  * @retval None
  */
void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;

	/* ADC1 configuration */
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel14 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_239Cycles5);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC1 reset calibaration register */
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while (ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);

	/* Check the end of ADC1 calibration */
	while (ADC_GetCalibrationStatus(ADC1));

	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
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

	//NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
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
* Stack overflow check hook function
*/
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName)
{
	if (pcTaskName)
		tprintf("Stack overflow! task=%s\r\n");

	while (1);
}

/**
* Idle hook function
*/
void vApplicationIdleHook(void)
{
#ifndef DEBUG
		if (program || busy)
			return;

		/* Set wakeup time */
		RTC_SetAlarm(RTC_GetCounter() + sleepDuration);

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
#if 0
		uint8_t ch = 'A';
		portBASE_TYPE xHigherPriorityTaskWoken;
		xQueueSendToBackFromISR(xQueue, &ch, &xHigherPriorityTaskWoken);
#endif

		program = 1;

		/* Clear the Key Button EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line11);
	}
}

void ShowTime(void)
{
	uint8_t hours = dateTime.hours;
	LedBrightness brightness;
	uint16_t adcValue = ADCConvertedValue;

	if (dateTime.hours == 0)
	{
		hours = 12;
	}
	else if (dateTime.hours > 12)
	{
		hours -= 12;
	}

	if (dateTime.hours >= 12)
	{
		tsprintf(response, "       %d:%02dpm", hours, dateTime.minutes);
	}
	else
	{
		tsprintf(response, "       %d:%02dam", hours, dateTime.minutes);
	}

#ifdef DEBUG_AMBIENT
	LED_SetLine(0, response);
#else
	LED_SetMiddleLine(response);
#endif

	brightness = GetLedBrightnessFromADC(adcValue);
	LED_SetBrightness(brightness);
#ifdef DEBUG_AMBIENT
	tprintf("Brightness,ADC: %d,%d\r\n", brightness, adcValue);
	tsprintf(response, "%d %d", brightness, adcValue);
	LED_SetLine(1, response);
#endif
	LED_Refresh();
}

/**
  * @brief  This function handles RTC interrupt request
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
	if(RTC_GetITStatus(RTC_IT_SEC) != RESET)
	{
		uint32_t counter = RTC_GetCounter();

		if (showTime)
		{
			dateTime.seconds = (counter % 3600) % 60;

			if (dateTime.seconds == 0)
			{
				dateTime.hours = counter / 3600;
				dateTime.minutes = (counter % 3600) / 60;

				ShowTime();
			}
		}

		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();

		/* Clear RTC Alarm interrupt pending bit */
		RTC_ClearITPendingBit(RTC_IT_SEC);

		/* Reset RTC Counter when Time is 23:59:59 */
		if (counter == 0x00015180)
		{
			RTC_SetCounter(0);
			/* Wait until last write operation on RTC registers has finished */
			RTC_WaitForLastTask();
		}
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


void GetDateTime(void)
{
	int result = Network_GetDateTime(&dateTime, DEFAULT_TIMEOUT);

	if (result != 0 || dateTime.month > 12 || dateTime.day > 31 || dateTime.hours > 24 || dateTime.minutes > 60 || dateTime.seconds > 60)
	{
		tprintf("Invalid date/time!\r\n");
		validTime = 0;
	}
	else
	{
		validTime = 1;
		tprintf("Year: %d\r\n", dateTime.year);
		tprintf("Month: %d\r\n", dateTime.month);
		tprintf("Day: %d\r\n", dateTime.day);
		tprintf("Hours: %d\r\n", dateTime.hours);
		tprintf("Minutes: %d\r\n", dateTime.minutes);
		tprintf("Seconds: %d\r\n", dateTime.seconds);

		ShowTime();

		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();

		/* Change the current time */
		RTC_SetCounter(dateTime.hours * 3600 + dateTime.minutes * 60 + dateTime.seconds);

		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();
		showTime = 1;
	}
}

LedBrightness GetLedBrightnessFromADC(uint16_t value)
{
	LedBrightness result = LedBrightness_0;
	LedBrightness i;

	for (i = LedBrightness_14; i > LedBrightness_0; i--)
	{
		if (value >= brightnessToADC[i])
		{
			result = i;
			break;
		}
	}

	return result;
}

void Main_Task(void *pvParameters)
{
	int result = 0;
	uint8_t skipOneSleep = 0;
	uint8_t messageTimeout = 0;
	portTickType xLastWakeTime;

	/* Initialise the xLastExecutionTime variable on task entry */
	xLastWakeTime = xTaskGetTickCount();

	xBusyMutex = xSemaphoreCreateMutex();
	assert_param(xBusyMutex);

	/* Set buzzer to 4KHz */
	result = Buzzer_Configuration(4000);
	assert_param(result >= 0);

	result = LED_Configuration();
	assert_param(result >= 0);

	result = Network_Configuration();
	assert_param(result >= 0);

	tprintf("Remote Test\r\n");

	result = Remote_Configuration();
	assert_param(result >= 0);

#if 0
	result = Network_SendWait("E0", "I/OK\r\n", DEFAULT_TIMEOUT);

	while (result != 0)
		result = Network_SendWait("E0", "I/OK\r\n", DEFAULT_TIMEOUT);

	result = Network_SendWait("AWS=1", "I/OK\r\n", DEFAULT_TIMEOUT);

	while (result != 0)
		result = Network_SendWait("AWS=1", "I/OK\r\n", DEFAULT_TIMEOUT);

	/* Set baud rate to 115200 */
	result = Network_SendWait("BDRF=9", "I/OK\r\n", DEFAULT_TIMEOUT);
	assert_param(result >= 0);
#endif

#if 0
	xQueue = xQueueCreate(10, sizeof(char));
	assert_param(xQueue);
#endif

	for(;;)
	{
		skipOneSleep = 0;
		sleepDuration = sleepDurations[networkState];
#if 0
		portBASE_TYPE result = xQueueReceive(xQueue, &ch, portMAX_DELAY);
		if (result)
		{
			tprintf("%c\r\n", ch);
		}
		else
		{
			tprintf("!result\r\n");
		}
#endif
//		BUSY(MARQUEE);
		vTaskSuspend(NULL);
		continue;

		switch (networkState)
		{
			case NetworkState_NotConnected:
			{
				tprintf("#networkState: NotConnected\r\n");
				//result = Network_SendGetByChar("!RP10", '\n', '\r', response, DEFAULT_TIMEOUT);
				//result = Network_SendGetLine("!RP10", response, DEFAULT_TIMEOUT);
				result = Network_GetWlanConnection(&networkConnection, DEFAULT_TIMEOUT);

				if (result == 0)
				{
					networkState = NetworkState_Connected;
					skipOneSleep = 1;
#if 0
					if (strstr(response, "Completed") != NULL)
					{
						networkState = Connected;
					}
					else
					{
						tprintf("#%s!\r\n", response);
					}
#endif
				}
				else if (result == -ERR_TIMEOUT)
				{
					tprintf("#Timeout!\r\n");
				}
			} break;

			case NetworkState_Connected:
			{
				tprintf("#networkState: Connected\r\n");
				result = Network_GetIPAddress(response, DEFAULT_TIMEOUT);

				if (result == 0 && (strcmp(response, "0.0.0.0") != 0))
				{
					tprintf("IP: %s\r\n", response);
					LED_SetMiddleLine(response);
					LED_Refresh();
					networkState = NetworkState_Online;
					skipOneSleep = 1;
				}
				else if (result == -ERR_TIMEOUT)
					tprintf("#Timeout!\r\n");
			} break;

			case NetworkState_Online:
			{
				tprintf("#networkState: Online\r\n");
				if (!validTime)
				{
					GetDateTime();
				}

				result = Network_GetEmail(response, NULL, 10000);
				if (result == 0)
				{
					messageTimeout = 1;
					LED_SetLine(0, response);
					if (strlen(response) > 20)
					{
						LED_SetLine(1, &response[20]);
					}
					else
					{
						LED_SetLine(1, "");
					}
					showTime = 0;
					LED_Refresh();
					Buzzer_Beep(2);
					result = Network_SendWait("!RMM", "I/OK", DEFAULT_TIMEOUT);
				}
				else
				{
					if (messageTimeout && --messageTimeout == 0)
					{
						LED_ScrollOut(0);
						showTime = 1;
					}

					ShowTime();
				}
			} break;

			default:
				tprintf("#networkState: Invalid\r\n");
				assert_param(0);
				break;
		}

//		IDLE(MARQUEE);
#ifndef DEBUG
		if (!skipOneSleep)
			vTaskSuspend(NULL);
#else
		if (program)
			vTaskSuspend(NULL);
		else if (!skipOneSleep)
			vTaskDelay((sleepDuration * MS_PER_SEC) / portTICK_RATE_MS);
#endif
	}
}

