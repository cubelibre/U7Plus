/* wisky/gsensor/wisky_gsensor.c
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
 *	1.Create wisky_gsensor.c for G-Sensor module select.
 */
 
//#define WISKY_DEBUG
#include <linux/wisky.h>

#if WISKY_ENABLE_GSENSOR

#if defined(WISKY_GSENSOR_MMA7660)
#include "wisky_gsensor_mma7660.c"
#elif defined(WISKY_GSENSOR_DMARD06)
#include "wisky_gsensor_dmard06.c"
#endif

#endif