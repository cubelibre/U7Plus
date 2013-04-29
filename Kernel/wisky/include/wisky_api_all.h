/* wisky/include/wisk_api_all.h
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
 * V001:20110415 cd huang
 *		1.add for wisky public driver api for device driver.
 */


#ifndef _WISKY_API_ALL_H_
#define _WISKY_API_ALL_H_

/*
 * touch key board led control function
 * param: 
 *	enable---positive number to turn on touch keyboard LED, 0 to turn off touch keyboard LED
 * return: return 0 if success, else negative number
 */
extern int wisky_touchkey_led_set(int enable);
#if defined(WISKY_DEBUG_LOGFILE)
extern int wisky_debug_write_log_thread(char *plog);
#endif



#endif
 