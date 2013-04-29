/* wisky/led/wisky_led.c
 *
 * Copyright (C) 2011 Wisky Ltd
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
 * V001:20110415 cd huang
 *	1.Create for LED module select.
 */

//#define WISKY_DEBUG
#include <linux/wisky.h>

//#if (WISKY_LED_GPIO)
#include "wisky_led_gpio.c"
//#endif