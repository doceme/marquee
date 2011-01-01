/**
 ******************************************************************************
 *
 * @file       led.h
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


#ifndef LED_H
#define LED_H

typedef enum LedBrightness
{
	LedBrightness_0,
	LedBrightness_1,
	LedBrightness_2,
	LedBrightness_3,
	LedBrightness_4,
	LedBrightness_5,
	LedBrightness_6,
	LedBrightness_7,
	LedBrightness_8,
	LedBrightness_9,
	LedBrightness_10,
	LedBrightness_11,
	LedBrightness_12,
	LedBrightness_13,
	LedBrightness_14,
	LedBrightness_15,
	LedBrightness_Last
} LedBrightness;

enum led_icon16_t
{
	LED_ICON16_TIMER,
	LED_ICON16_CALL,
	LED_ICON16_LAST
};

/**
 * @brief  Configures the LED
 * @param  None
 * @retval None
 */
int LED_Configuration(void);

/**
 * @brief  Draws a string on a line of the LED display
 * @param  line Vertical index of the line to set
 *         0 = Top line
 *         1 = Bottom line
 *         2 = Middle line
 * @param  pos The horizontal position to start drawing
 * @param  text The message to display
 * @retval None
 */
int LED_SetString(uint8_t line, uint8_t pos, char* text, uint8_t fixed_width);

/**
 * @brief  Draws a string on a line of the LED display
 * @param  line Vertical index of the line to set
 *         0 = Top line
 *         1 = Bottom line
 *         2 = Middle line
 * @param  pos The horizontal position to start drawing
 * @param  text The message to display
 * @retval None
 */
int LED_DrawString(uint8_t line, uint8_t pos, char* text);

/**
 * @brief  Draws an icon on the LED display
 * @param  pos The horizontal position to start drawing
 * @param  icon The icon to display
 * @retval None
 */
int LED_SetIcon16(uint8_t pos, enum led_icon16_t icon);

/**
 * @brief  Draws an icon on the LED display
 * @param  pos The horizontal position to start drawing
 * @param  icon The icon to display
 * @retval None
 */
int LED_DrawIcon16(uint8_t pos, enum led_icon16_t icon);

/**
 * @brief  Sets a message on a line of the LED display
 * @param  line A zero-based index of the line to set
 * @param  message The message to display
 * @retval None
 */
int LED_SetLine(uint8_t line, char *message);

/**
 * @brief  Sets a message in the middle of the LED display
 * @param  message The message to display
 * @retval None
 */
int LED_SetMiddleLine(char *message);

/**
 * @brief  Redraws the LED display
 * @param  None
 * @retval None
 */
int LED_Refresh();

/**
 * @brief  Scrolls the existing message on a line of the LED display
 * @param  line A zero-based index of the line to set
 * @param  message The message to display
 * @retval None
 */
int LED_ScrollOut(uint8_t line);

/**
 * @brief  Scrolls in a message on a line of the LED display
 * @param  line A zero-based index of the line to set
 * @param  message The message to display
 * @retval None
 */
int LED_ScrollIn(uint8_t line, char *message);

/**
 * @brief  Sets the brightness of the LED
 * @retval 0 if not blank
 * @retval 1 if blank
 */
int LED_SetBrightness(LedBrightness brightness);

/**
 * @brief  Draw a cursor on the display
 * @param  line A zero-based index of the line to set
 * @param  message The message to display
 * @retval None
 */
int LED_DrawCursor(uint8_t line, uint8_t pos, uint8_t state);

/**
 * @brief  Redraw a portion of a line on the display
 * @param  line A zero-based index of the line to set
 * @param  start The start position of the line to redraw
 * @param  end The end position of the line to redraw
 * @retval None
 */
int LED_Redraw(uint8_t line, uint8_t start, uint8_t end);

/**
 * @brief  Clears the LED display
 * @param  None
 * @retval None
 */
int LED_Clear();

#endif /* LED_H */

