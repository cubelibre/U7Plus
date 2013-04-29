/* wisky/logo/wisky_logo_data_all.c
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
 * V001:20110729 cd huang
 *	1.kernel boot logo data include
 */
 
//#define WISKY_DEBUG
#include <linux/wisky.h>

#if defined(WISKY_LOGO_LINUX)
#include "./logo_data_all/logo_linux_clut224.c"
#elif defined(WISKY_LOGO_PLOYER_1024X768)
#include "./logo_data_all/logo_ployer_1024x768.c"
#elif defined(WISKY_LOGO_PLOYER_MOMO9_800X480)
#include "./logo_data_all/logo_ployer_momo9_800x480.c"
#elif defined(WISKY_LOGO_TECLAST_1024X600_H)
#include "./logo_data_all/logo_teclast_1024x600_h.c"
#elif defined(WISKY_LOGO_TECLAST_768X1024)
#include "./logo_data_all/logo_teclast_768x1024.c"
#elif defined(WISKY_LOGO_DANEW_DSLIDE972)
#include "./logo_data_all/logo_danew_dslide972.c"
#elif defined(WISKY_LOGO_EXPLAY_768X1024)	
#include"./logo_data_all/logo_explay_768x1024.c"
#elif defined(WISKY_LOGO_EXPLAY_SURFER801_1024x768)	
#include"./logo_data_all/logo_explay_surfer801_1024x768.c"
#elif defined(WISKY_LOGO_TIANJIAO_MID8500)
#include"./logo_data_all/logo_tianjiao_mid8500.c"
#elif defined(WISKY_LOGO_DNS_1024x768)
#include "./logo_data_all/logo_dns_1024x768.c"
#elif defined(WISKY_LOGO_DNS_800x480)
#include "./logo_data_all/logo_dns_800x480.c"
#elif defined(WISKY_LOGO_DNS_1024x600)
#include "./logo_data_all/logo_dns_1024x600.c"
#elif defined(WISKY_LOGO_TECLAST_480X800)
#include "./logo_data_all/logo_teclast_480x800.c"
#elif defined(WISKY_LOGO_MINGXUAN_1024x768)
#include "./logo_data_all/logo_mingxuan_1024x768.c"
#elif defined(WISKY_LOGO_TECLAST_P98_768X1024)
#include "./logo_data_all/logo_teclast_p98_768x1024.c"
#elif defined(WISKY_LOGO_POCKETBOOK_U7PLUS_800X1280)
#include "./logo_data_all/logo_pocketbook_u7plus_800x1280.c"

#else
#include "./logo_data_all/logo_linux_clut224.c"
#endif

