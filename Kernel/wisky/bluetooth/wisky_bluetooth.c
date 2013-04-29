/* wisky/bluetooth/wisky_bluetooth.c
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
 * V001:20110413 cd huang
 *	1.Create driver for bluetooth driver setup.
 */

//#define WISKY_DEBUG
#include <linux/wisky_linux.h>

#if WISKY_ENABLE_BLUETOOTH

#if (WISKY_WIFI_BT_BCM4329)
#include "wisky_bluetooth_bcm4329.c"
#endif

#endif
