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

/* Definitions */
#define DEBUG
//#define DEBUG_AMBIENT
#define BUSY_BIT_MARQUEE	0
#define SLEEP_TIME		10
#define DEFAULT_TIMEOUT		3000
#define MS_PER_SEC		1000

typedef enum NetworkState
{
	NetworkState_NotConnected,
	NetworkState_Connected,
	NetworkState_Online,
	NetworkState_Last
} NetworkState;

enum marquee_state_t
{
	MARQUEE_STATE_IDLE,
	MARQUEE_STATE_MESSAGE,
	MARQUEE_STATE_MENU,
	MARQUEE_STATE_SET_TIMER,
	MARQUEE_STATE_TIMER,
	MARQUEE_STATE_CONTACT
};

enum marquee_menus_t
{
	MARQUEE_MENU_NONE,
	MARQUEE_MENU_TIMER,
	MARQUEE_MENU_CALL,
	MARQUEE_MENU_LAST
};

struct marquee_menu_t
{
	enum marquee_menus_t item;
	enum led_icon16_t icon;
	char *text;
};

/* Global Variables */
volatile uint32_t busy = 0;
xSemaphoreHandle xBusyMutex = NULL;
__IO uint16_t ADCConvertedValue;

/* Local Variables */
static volatile uint8_t program = 0;
static xTaskHandle xMainTask;
static xSemaphoreHandle xMutex;
static NetworkState networkState = NetworkState_NotConnected;
static NetworkWlanConnection_t networkConnection;
static char response[80];
static char message[80];
static const uint32_t sleepDurations[NetworkState_Last] = { 3, 3, 10 };
static uint32_t sleepDuration;
static uint8_t validTime = 0;
static NetworkDateTime_t dateTime;
static const uint32_t ADC1_DR_Address = 0x4001244C;
static enum marquee_state_t marquee_state;
static enum marquee_state_t marquee_prev_state;
static struct marquee_menu_t *marquee_menu;
static uint8_t button_trailer = 2;
static uint8_t cursor_pos;
static uint8_t cursor_state;

static const struct marquee_menu_t marquee_menu_items[] =
{
	{MARQUEE_MENU_NONE, 0, ""},
	{MARQUEE_MENU_TIMER, LED_ICON16_TIMER, "Timer"},
	{MARQUEE_MENU_CALL, LED_ICON16_CALL, "Call"},
	{MARQUEE_MENU_LAST, 0, ""}
};

enum remote_buttons_t
{
	REMOTE_BUTTON_NONE,
	REMOTE_BUTTON_UP,
	REMOTE_BUTTON_DOWN,
	REMOTE_BUTTON_LEFT,
	REMOTE_BUTTON_RIGHT,
	REMOTE_BUTTON_OK,
	REMOTE_BUTTON_MENU,
	REMOTE_BUTTON_LAST_CH,
	REMOTE_BUTTON_INFO,
	REMOTE_BUTTON_0,
	REMOTE_BUTTON_1,
	REMOTE_BUTTON_2,
	REMOTE_BUTTON_3,
	REMOTE_BUTTON_4,
	REMOTE_BUTTON_5,
	REMOTE_BUTTON_6,
	REMOTE_BUTTON_7,
	REMOTE_BUTTON_8,
	REMOTE_BUTTON_9,
	REMOTE_BUTTON_EXIT,
	REMOTE_BUTTON_PAUSE,
	REMOTE_BUTTON_DEL,
	REMOTE_BUTTON_RW,
	REMOTE_BUTTON_FF,
	REMOTE_BUTTON_PLAY,
	REMOTE_BUTTON_STOP,
	REMOTE_BUTTON_REC,
	REMOTE_BUTTON_INPUT,
	REMOTE_BUTTON_LAST
};

static struct remote_button_t buttons[] =
{
	{0, 0, 0, 0, 0},			/* REMOTE_BUTTON_NONE */
	{REMOTE_PROTOCOL_RC6, 0, 0, 4, 88},	/* REMOTE_BUTTON_UP */
	{REMOTE_PROTOCOL_RC6, 0, 0, 4, 89},	/* REMOTE_BUTTON_DOWN */
	{REMOTE_PROTOCOL_RC6, 0, 0, 4, 90},	/* REMOTE_BUTTON_LEFT */
	{REMOTE_PROTOCOL_RC6, 0, 0, 4, 91},	/* REMOTE_BUTTON_RIGHT */
	{REMOTE_PROTOCOL_RC6, 0, 0, 4, 92},	/* REMOTE_BUTTON_OK */
	{REMOTE_PROTOCOL_RC6, 0, 0, 4, 130},	/* REMOTE_BUTTON_MENU */
	{REMOTE_PROTOCOL_RC6, 0, 0, 4, 131},	/* REMOTE_BUTTON_LAST_CH */
	{REMOTE_PROTOCOL_RC6, 0, 0, 4, 201},	/* REMOTE_BUTTON_INFO */

	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 0},	/* REMOTE_BUTTON_0 */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 1},	/* REMOTE_BUTTON_1 */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 2},	/* REMOTE_BUTTON_2 */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 3},	/* REMOTE_BUTTON_3 */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 4},	/* REMOTE_BUTTON_4 */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 5},	/* REMOTE_BUTTON_5 */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 6},	/* REMOTE_BUTTON_6 */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 7},	/* REMOTE_BUTTON_7 */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 8},	/* REMOTE_BUTTON_8 */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 9},	/* REMOTE_BUTTON_9 */

	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 15},	/* REMOTE_BUTTON_EXIT */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 41},	/* REMOTE_BUTTON_PAUSE */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 49},	/* REMOTE_BUTTON_DEL */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 50},	/* REMOTE_BUTTON_RW */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 52},	/* REMOTE_BUTTON_FF */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 53},	/* REMOTE_BUTTON_PLAY */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 54},	/* REMOTE_BUTTON_STOP */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 55},	/* REMOTE_BUTTON_REC */
	{REMOTE_PROTOCOL_RC5, 0, 0, 5, 62},	/* REMOTE_BUTTON_INPUT */
	{0, 0, 0, 0, 0}			/* REMOTE_BUTTON_LAST */
};

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
static inline void ShowIdle(void);
static void GetDateTime(void);
static LedBrightness GetLedBrightnessFromADC(uint16_t value);
static void Main_Task(void *pvParameters) NORETURN;
static void main_noreturn(void) NORETURN;

static inline void marquee_change_state(enum marquee_state_t state)
{
	marquee_prev_state = marquee_state;
	marquee_state = state;
}

void OnButtonCallback(struct remote_button_t *button);

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

	xMutex = xSemaphoreCreateMutex();
	assert_param(xMutex);

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

void ShowIdle(void)
{
	uint8_t hours = dateTime.hours;
	LedBrightness brightness;
	uint16_t adcValue = ADCConvertedValue;

	marquee_change_state(MARQUEE_STATE_IDLE);
	marquee_menu = (struct marquee_menu_t*)&marquee_menu_items[MARQUEE_MENU_NONE];

	if (dateTime.hours == 0)
	{
		hours = 12;
	}
	else if (dateTime.hours > 12)
	{
		hours -= 12;
	}

	xSemaphoreTake(xMutex, portMAX_DELAY);

	if (dateTime.hours >= 12)
	{
		tsprintf(response, "%d:%02dpm", hours, dateTime.minutes);
	}
	else
	{
		tsprintf(response, "%d:%02dam", hours, dateTime.minutes);
	}

#ifdef DEBUG_AMBIENT
	LED_SetLine(0, response);
#else
	/* LED_SetMiddleLine(response); */
	LED_Clear();
	LED_SetString(2, 42, response, 0);
	LED_Refresh();
#endif

	brightness = GetLedBrightnessFromADC(adcValue);
	LED_SetBrightness(brightness);
#ifdef DEBUG_AMBIENT
	tprintf("Brightness,ADC: %d,%d\r\n", brightness, adcValue);
	tsprintf(response, "%d %d", brightness, adcValue);
	LED_SetLine(1, response);
#endif
	/* LED_Refresh(); */

	xSemaphoreGive(xMutex);
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

		dateTime.seconds = (counter % 3600) % 60;

		if (dateTime.seconds == 0)
		{
			dateTime.hours = counter / 3600;
			dateTime.minutes = (counter % 3600) / 60;

			if (marquee_state == MARQUEE_STATE_IDLE)
			{
				ShowIdle();
			}
		}

		if (marquee_state == MARQUEE_STATE_SET_TIMER)
		{
			cursor_state ^= 1;
			LED_DrawCursor(1, cursor_pos, cursor_state);
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
#if 0
		tprintf("Year: %d\r\n", dateTime.year);
		tprintf("Month: %d\r\n", dateTime.month);
		tprintf("Day: %d\r\n", dateTime.day);
		tprintf("Hours: %d\r\n", dateTime.hours);
		tprintf("Minutes: %d\r\n", dateTime.minutes);
		tprintf("Seconds: %d\r\n", dateTime.seconds);
#endif

		ShowIdle();

		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();

		/* Change the current time */
		RTC_SetCounter(dateTime.hours * 3600 + dateTime.minutes * 60 + dateTime.seconds);

		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();
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

static uint8_t remote_button_match(struct remote_button_t *src, struct remote_button_t *match)
{
	if ((src->protocol == match->protocol) &&
		(src->mode == match->mode) &&
		(src->address == match->address) &&
		(src->data == match->data))
	{
		return 1;
	}

	return 0;
}

static enum remote_buttons_t remote_button_type(struct remote_button_t *button)
{
	enum remote_buttons_t result = REMOTE_BUTTON_NONE;
	struct remote_button_t *match;
	int i;

	for (i = 0; i < REMOTE_BUTTON_LAST; i++)
	{
		match = &buttons[i];
		if (remote_button_match(button, match))
		{
			result = i;
			break;
		}
	}

	return result;
}

void OnButtonCallback(struct remote_button_t *button)
{
	if ((button->trailer == button_trailer) && (button_trailer != 2))
	{
		return;
	}
	else
	{
		button_trailer = button->trailer;
#if 0
		tprintf("button[protocol=%d,mode=%d,trailer=%d,address=%d,data=%d]\r\n", button->protocol, button->mode, button->trailer, button->address, button->data);
#endif
	}

	enum remote_buttons_t button_type = remote_button_type(button);

	if ((button_type == REMOTE_BUTTON_EXIT) && (marquee_state != MARQUEE_STATE_IDLE))
	{
		ShowIdle();
	}
	else
	{
#if 0
		tprintf("button_type=%d, marquee_state=%d\r\n", button_type, marquee_state);
#endif

		switch (marquee_state)
		{
			case MARQUEE_STATE_MESSAGE:
			{
				if (button_type == REMOTE_BUTTON_OK)
				{
					int result;

					if (marquee_prev_state == MARQUEE_STATE_IDLE)
					{
						ShowIdle();
					}
					else
					{
						marquee_change_state(marquee_prev_state);
					}

					result = Network_DeleteMessage(20000);

					if (result == 0)
					{
						message[0] = '\0';
						tprintf("dismissed\r\n");
					}
					else
					{
						tprintf("error deleting message!\r\n");
					}
				}
			} break;

			case MARQUEE_STATE_IDLE:
			{
				switch (button_type)
				{
					case REMOTE_BUTTON_DOWN:
					{
						marquee_change_state(MARQUEE_STATE_MENU);
						marquee_menu = (struct marquee_menu_t*)&marquee_menu_items[MARQUEE_MENU_TIMER];

						LED_Clear();
						LED_SetIcon16(0, marquee_menu->icon);
						LED_SetString(2, 17, marquee_menu->text, 0);
						LED_Refresh();
					} break;

					case REMOTE_BUTTON_UP:
					{
						marquee_change_state(MARQUEE_STATE_MENU);
						marquee_menu = (struct marquee_menu_t*)&marquee_menu_items[MARQUEE_MENU_CALL];

						LED_Clear();
						LED_SetIcon16(0, marquee_menu->icon);
						LED_SetString(2, 17, marquee_menu->text, 0);
						LED_Refresh();
					} break;

					default:
						break;
				}
			} break;

			case MARQUEE_STATE_MENU:
			{
				switch (button_type)
				{
					case REMOTE_BUTTON_DOWN:
					{
						marquee_menu++;

						if (marquee_menu->item == MARQUEE_MENU_LAST)
						{
							marquee_menu = (struct marquee_menu_t*)&marquee_menu_items[MARQUEE_MENU_NONE + 1];
						}

						LED_Clear();
						LED_SetIcon16(0, marquee_menu->icon);
						LED_SetString(2, 17, marquee_menu->text, 0);
						LED_Refresh();
					} break;

					case REMOTE_BUTTON_UP:
					{
						marquee_menu--;

						if (marquee_menu->item == MARQUEE_MENU_NONE)
						{
							marquee_menu = (struct marquee_menu_t*)&marquee_menu_items[MARQUEE_MENU_LAST - 1];
						}

						LED_Clear();
						LED_SetIcon16(0, marquee_menu->icon);
						LED_SetString(2, 17, marquee_menu->text, 0);
						LED_Refresh();
					} break;

					case REMOTE_BUTTON_OK:
					{
						if (marquee_menu->item == MARQUEE_MENU_TIMER)
						{
							cursor_pos = 45;
							marquee_change_state(MARQUEE_STATE_SET_TIMER);
							LED_Clear();
							LED_SetString(0, 37, "Set time", 0);
							LED_SetString(1, 45, "00:00", 1);
							LED_Refresh();
						}
						else
						{
							ShowIdle();
						}
					} break;

					default:
						break;
				}
			} break;

			default:
				break;
		}
	}
}

void Main_Task(void *pvParameters)
{
	int result = 0;
	uint8_t skipOneSleep = 0;
	portTickType xLastWakeTime;

	response[0] = '\0';
	message[0] = '\0';

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

	result = Remote_Configuration();
	assert_param(result >= 0);

	result = Remote_SetCallback(OnButtonCallback);
	assert_param(result >= 0);

	marquee_change_state(MARQUEE_STATE_MENU);
	marquee_menu = (struct marquee_menu_t*)&marquee_menu_items[MARQUEE_MENU_TIMER];

	tprintf("=== LED Marquee ===\r\n");

#if 1
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

		switch (networkState)
		{
			case NetworkState_NotConnected:
			{
#ifdef DEBUG_STATE_NETWORK
				tprintf("#networkState: NotConnected\r\n");
#endif
				result = Network_GetWlanConnection(&networkConnection, DEFAULT_TIMEOUT);

				if (result == 0)
				{
					networkState = NetworkState_Connected;
					skipOneSleep = 1;
				}
				else if (result == -ERR_TIMEOUT)
				{
					tprintf("#Timeout!\r\n");
				}
			} break;

			case NetworkState_Connected:
			{
#ifdef DEBUG_STATE_NETWORK
				tprintf("#networkState: Connected\r\n");
#endif
				xSemaphoreTake(xMutex, portMAX_DELAY);
				response[0] = '\0';

				result = Network_GetIPAddress(response, DEFAULT_TIMEOUT);

				if (result == 0 && (strcmp(response, "0.0.0.0") != 0))
				{
#ifdef DEBUG_STATE_NETWORK
					tprintf("IP: %s\r\n", response);
#endif
					LED_Clear();
					LED_SetString(2, 0, response, 0);
					LED_Refresh();
					networkState = NetworkState_Online;
					skipOneSleep = 1;
				}
				else if (result == -ERR_TIMEOUT)
					tprintf("#Timeout!\r\n");

				xSemaphoreGive(xMutex);
			} break;

			case NetworkState_Online:
			{
#ifdef DEBUG_STATE_NETWORK
				tprintf("#networkState: Online\r\n");
#endif
				if (!validTime)
				{
					GetDateTime();
				}

				xSemaphoreTake(xMutex, portMAX_DELAY);
				response[0] = '\0';

				result = Network_GetMessage(response, 20000);
				if (result == 0 && response[0] != '\0' && strcmp(response, message) != 0)
				{
					strcpy(message, response);
					response[0] = '\0';

					tprintf("message: %s\r\n", message);

					marquee_change_state(MARQUEE_STATE_MESSAGE);

					LED_Clear();
					LED_SetString(0, 0, message, 0);
					LED_Refresh();

					Buzzer_Beep(2);
					xSemaphoreGive(xMutex);
				}
				else if (marquee_state == MARQUEE_STATE_IDLE)
				{
					xSemaphoreGive(xMutex);
					ShowIdle();
				}
			} break;

			default:
#ifdef DEBUG_STATE_NETWORK
				tprintf("#networkState: Invalid\r\n");
#endif
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

