/**
 ******************************************************************************
 *
 * @file       buzzer.h
 * @author     Stephen Caudle Copyright (C) 2010
 * @brief      Buzzer header
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

#ifndef BUZZER_H
#define BUZZER_H

/**
 * @brief  Configures the buzzer
 * @param  None
 * @retval None
 */
int Buzzer_Configuration(uint16_t frequency);

/**
 * @brief  Sets the on duration of the buzzer
 * @param  duration Duration the buzzer should be on in milliseconds
 * @retval 0 on Success
 * @retval Negative error code on failure
 */
int Buzzer_SetOnDuration(uint32_t duration);

/**
 * @brief  Sets the off duration of the buzzer
 * @param  duration Duration the buzzer should be off in milliseconds
 * @retval 0 off Success
 * @retval Negative error code on failure
 */
int Buzzer_SetOffDuration(uint32_t duration);

/**
 * @brief  Activates the buzzer for count times
 * @param  count Number of times to beep the buzzer
 * @retval 0 on Success
 * @retval Negative error code on failure
 */
int Buzzer_Beep(uint32_t count);

/**
 * @brief  Determines whether or not the buzzer is beeping
 * @param  None
 * @retval 0 if not beeping
 * @retval 1 if beeping
 * @retval Negative error code on failure
 */
int Buzzer_IsBeeping(void);

#endif /* BUZZER_H */
