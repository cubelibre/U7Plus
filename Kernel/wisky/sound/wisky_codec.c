/* wisky/sound/wisky_codec.c
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
 *	1.Create for sound card codec chip module select.
 */

//#define WISKY_DEBUG
#include <linux/wisky.h>

#if defined(WISKY_CODEC_RT5631)
#include "wisky_codec_rt5631.c"
#elif defined(WISKY_CODEC_ES8323)
#include "wisky_codec_es8323.c"
#endif

