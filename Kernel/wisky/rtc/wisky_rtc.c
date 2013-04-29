/* wisky/rtc/wisky_rtc.c
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
 * V001:20110301 cd huang
 *	1.Create for RTC chip module select.
 */

//#define WISKY_DEBUG
#include <linux/wisky.h>

#if (WISKY_RTC_HYM8563)
#include "wisky_rtc_hym8563.c"
#elif defined(WISKY_PMU_TPS65910) //it integrated in pmu tps65910 IC
#include "wisky_rtc_tps65910.c"
#else

#endif