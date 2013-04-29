/* wisky/switch/wisky_switch.c
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
 * V001:20110312 cd huang
 *	1.Create for headphone/speaker switch driver config.
 */

//#define WISKY_DEBUG
#include <linux/wisky.h>

#if defined(WISKY_SWITCH_GPIO)
#include "wisky_switch_gpio.c"
#endif