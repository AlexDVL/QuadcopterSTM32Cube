//------------------------------------------------------
/* File:       Library for HD44780 compatible displays  */
/* Version:	   v2.01  						 			*/
/* Author:     GrAnd/www.MakeSystem.net		 			*/
/* Tested on:  AVR, STM32F10X			 	 		 	*/
/* License:	   GNU LGPLv2.1		 		 	 			*/
//------------------------------------------------------
/* Copyright (C)2014 GrAnd. All right reserved 			*/
//------------------------------------------------------

/*
	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

Contact information :
						mail@makesystem.net
						http://makesystem.net/?page_id=2
*/

#ifndef LCD_CFG_H
#define LCD_CFG_H

/* Include here the header file of your microcontroller */
#include "stm32f1xx_hal.h"

//-------------------------------
/* SET LCD<->MCU CONNECTIONS */
//-------------------------------
/* E - Starts data read/write. */
#define LCD_E_OUT					GPIOC->ODR	/* Output register */
#define LCD_E 	 					12u	/* Pin number */

/* RS - Selects registers.
 * 0:	Instruction register (for write)
 * 0:	Busy flag + address counter (for read)
 * 1:	Data register (for write and read) */
#define LCD_RS_OUT					GPIOD->ODR	/* Output register */
#define LCD_RS 						2u	/* Pin number */

/* R/W - Selects read or write.
 * 0:	Write
 * 1:	Read */
#define LCD_RW_OUT					GPIOB->ODR	/* Output register */
#define LCD_RW						13u	/* Pin number */

/* DB4 to DB7 - Four high order bidirectional tristate data bus pins.
 * Used for data transfer and receive between the MPU and the HD44780U.
 * DB7 can be used as a busy flag. */
#define LCD_D7_OUT					GPIOB->ODR	/* Output register */
#define LCD_D7_IN					GPIOB->IDR	/* Input register */
#define LCD_D7 	 					6u	/* Pin number */

#define LCD_D6_OUT					GPIOB->ODR	/* Output register */
#define LCD_D6						7u	/* Pin number */

#define LCD_D5_OUT					GPIOB->ODR	/* Output register */
#define LCD_D5						8u	/* Pin number */

#define LCD_D4_OUT					GPIOB->ODR	/* Output register */
#define LCD_D4						9u	/* Pin number */

//-------------------------------
// DEFAULT CONFIGURATIONS
//-------------------------------
#define DEFAULT_DISPLAY_CONFIG		        DISPLAY_CONFIG_4bit_2L_5x8
#define DEFAULT_ENTRY_MODE			ENTRY_MODE_INC_NO_SHIFT
#define DEFAULT_VIEW_MODE			VIEW_MODE_DispOn_BlkOff_CrsOff
#define USE_BUSY_FLAG				0u	/* 1 (true) or 0 (false) */

//-------------------------------
// SET MCU TIMINGS
//-------------------------------
#define MCU_FREQ_VALUE				16u		/* MHz. Minimal value = 1 MHz */
#define BUSY_CYCLE_TIME				5u		/* x 10us. See datasheet for minimal value. */
#define CLRSCR_CYCLE_TIME			200u	/* x 10us. See datasheet for minimal value. */
#define RETHOME_CYCLE_TIME			200u	/* x 10us. See datasheet for minimal value. */

//-------------------------------
// CONFIGURE LCD WITH 4 LINES
//-------------------------------
#define START_ADDRESS_3rd_LINE		0x10u
#define START_ADDRESS_4th_LINE		0x50u

//-------------------------------
// SET FORMATTED OUTPUT OPTIONS
//-------------------------------
#define USE_FORMATTED_OUTPUT	 	1u	/* 1 (true) or 0 (false) */
#define TAB_SPACE					4u	/* 1 .. 255 */

//-------------------------------
// PROGRESS BAR OPTIONS
//-------------------------------
#define USE_PROGRESS_BAR			1u	/* 1 (true) or 0 (false) */
#define USE_REGRESS_BAR				1u	/* 1 (true) or 0 (false) */
#define PROGRESS_BAR_LINE			LCD_2nd_LINE	/* Select lcd line: 1, 2, 3, 4, ... */
#define PROGRESS_BAR_HEIGHT			3u  /* in pixel: 1(min), 2, 3, 4, 5, 6, 7, 8(max) */
#define PROGRESS_BAR_WIDTH			10u /* Number of chars in lcd line:  1, 2, .. , 8, 16, 20 */

//-------------------------------
// LCDlib CALLBACKS
//-------------------------------
#if (USE_BUSY_FLAG)

#define Set_D7_as_Input()	\
	GPIOB->CRL &= 0xFFF0FFFFu;	\
	GPIOB->CRL |= 0x00040000u;

#define Set_D7_as_Outut()	\
	GPIOB->CRL &= 0xFFF0FFFFu;	\
	GPIOB->CRL |= 0x00030000u;

#endif /* USE_BUSY_FLAG */

extern void gpio_init(void);
#define Set_All_pins_as_Outputs()	

#endif /* LCD_CFG_H */

//-------------------------------
/* THE END */
//-------------------------------
