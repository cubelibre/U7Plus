/* wisky/keyboard/wisky_keyboard.c
 *
 * Copyright (C) 2012 Wisky Ltd
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * LOG
 * ------------------------------------------------------------
 * V001:20120421 cd huang
 *	1.Create wisky_lcd.c for LCD module select.
 */
 
//#define WISKY_DEBUG
#include <linux/wisky.h>

#include "wisky_keyboard_data.c"

#if defined(WISKY_KEYBOARD_GPIO)
#include "wisky_keyboard_gpio.c"
#endif

