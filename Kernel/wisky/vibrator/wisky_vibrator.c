/* wisky/vibrator/wisky_vibrator.c
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
 * V001:20110323 cd huang
 *	1.Create for Timed GPIO vibrator driver config.
 */

//#define WISKY_DEBUG
#include <linux/wisky.h>

#if WISKY_ENABLE_VIBRATOR

#if (WISKY_VIBRATOR_TIMED_GPIO)
#include "wisky_vibrator_timed_gpio.c"
#endif

#endif