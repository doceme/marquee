/**
 ******************************************************************************
 *
 * @file       marquee.h
 * @author     Stephen Caudle Copyright (C) 2010.
 * @brief      Main header.
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


#ifndef MARQUEE_H
#define MARQUEE_H

/* C Lib Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
//#include <math.h>

/* FreeRTOS Includes */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* STM32 standard peripheral library */
#include <stm32f10x.h>
#include <stm32f10x_conf.h>

//#define ENABLE_SPI

#define SPI			SPI2
#define SPI_CLK			RCC_APB1Periph_SPI2
#define SPI_GPIO		GPIOB
#define SPI_GPIO_CLK		RCC_APB2Periph_GPIOB
#define SPI_PIN_CS		GPIO_Pin_12
#define SPI_PIN_SCK		GPIO_Pin_13
#define SPI_PIN_MISO		GPIO_Pin_14
#define SPI_PIN_MOSI		GPIO_Pin_15
#define SPI_MASTER_IRQn         SPI2_IRQn

extern uint8_t ready;

#endif /* MARQUEE_H */
