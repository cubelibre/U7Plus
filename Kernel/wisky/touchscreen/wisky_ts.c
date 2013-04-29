/* wisky/touchscreen/wisky_ts.c
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
 * V001:20120423 cd huang
 *	1.Create for Touch screen module select.
 */
 
//#define WISKY_DEBUG
#include <linux/wisky.h>

#include "wisky_ts_check.h"
#include "wisky_ts_check.c"

static int Ts_probe_success_flag = 0;

#if defined(WISKY_TS_GT8XX)
#include "wisky_ts_gt8xx_tool.c"
#include "wisky_ts_gt8xx_update.c"
#include "wisky_ts_gt8xx.c"
#endif
#if defined(WISKY_TS_GT9XX)
#include "wisky_ts_gt9xx_tool.c"
#include "wisky_ts_gt9xx_update.c"
#include "wisky_ts_gt9xx.c"
#endif

#if defined(WISKY_TS_FT5306)
#include "wisky_ts_ft5306.c"
#endif

#if defined(WISKY_TS_FT5X0X)
#include "wisky_ts_ft5x0x.c"
#endif

#if defined(WISKY_TS_ZET6221)
#include "wisky_ts_zet6221.c"
#endif

#if defined(WISKY_TS_GT811)
#include "wisky_ts_gt811.c"
#endif
