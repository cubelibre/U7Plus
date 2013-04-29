/* wisky/camera/wisky_camera_front.c
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
 *	1.Create wisky_camera_front.c for front panel camera module select.
 */

#define WISKY_DEBUG
#include <linux/wisky.h>

#if WISKY_ENABLE_CAMERA

#if defined(WISKY_CAMERA_HI704_F)
#include "wisky_camera_hi704.c"
#endif
#endif