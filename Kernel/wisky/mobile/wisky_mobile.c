/* wisky/mobile/wisky_mobile.c
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
 * V001:20110407 cd huang
 *	1.Create for moblie 3G net device driver config.
 */
 
//#define WISKY_DEBUG
#include <linux/wisky_linux.h>

#if WISKY_ENABLE_3G

#if WISKY_MOBILE_DEMO
#include "wisky_mobile_base.c"
#include "wisky_mobile_drv.c"
#elif defined (WISKY_MOBILE_MU509)
#include "wisky_mobile_mu509.c"
#elif defined (WISKY_MOBILE_TDM330)
#include "wisky_mobile_tdm330.c"
#endif

#endif