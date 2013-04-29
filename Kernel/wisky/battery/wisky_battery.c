/* wisky/battery/wisky_battery.c
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
 *	1.Create driver for battery module select.
 */

//#define WISKY_DEBUG
#include <linux/wisky.h>

#if defined(WISKY_BATTERY_ADC)
#include "wisky_battery_adc.c"
#elif defined(WISKY_BATTERY_OZ8555)
#include "wisky_battery_oz8555.c"
#elif defined(WISKY_BATTERY_OZ8806)
#include "wisky_battery_charge_ctrl.c"
#include "wisky_battery_oz8806.c"
//#elif defined(WISKY_BATTERY_SMB347)
//#include "wisky_battery_smb347.c"
#endif