/* wisky/config/wisky_config.c
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
 *	1.Create driver for kernel function config
 */
//#define WISKY_DEBUG
#include <linux/wisky.h>

//#if WISKY_USE_CONFIG_FILE
#include "wisky_config_main.c"
//#endif

#ifdef WISKY_DEBUG_LOGFILE
#include "wisky_debug_log.c"
#include "wisky_debug_logex.c"
#endif