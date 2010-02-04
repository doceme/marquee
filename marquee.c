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

/* Global Variables */

/* Local Variables */

/* Function Prototypes */
static void TaskSpin(void *pvParameters);

/**
* Main function
*/
int main()
{

	/* Initialize STM32 code */
	SystemInit();

	/* Create a FreeRTOS task */
	xTaskCreate(TaskSpin, (signed portCHAR *)"Spin", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY + 1, NULL);

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();

	/* If all is well we will never reach here as the scheduler will now be running. */
	/* If we do get here, it will most likely be because we ran out of heap space. */
	return 0;
}

void TaskSpin(void *pvParameters)
{
	portTickType xLastExecutionTime;

	/* Initialise the xLastExecutionTime variable on task entry */
	xLastExecutionTime = xTaskGetTickCount();

	for(;;)
	{
		vTaskDelayUntil(&xLastExecutionTime, TASK_SPIN_DELAY);
	}
}

/**
* Idle hook function
*/
void vApplicationIdleHook(void)
{
	/* Called when the scheduler has no tasks to run */
}

