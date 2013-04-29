/* wisky/lcd/wisky_lcd.c
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
 * V001:20120423 cd huang
 *	1.Create wisky_lcd.c for LCD module select.
 */

//#define WISKY_DEBUG
#include <linux/wisky.h>

#if defined(WISKY_LCD_20810800300017)
	#include "wisky_lcd_20810800300017.c"
#elif defined(WISKY_LCD_EJ080NA04B)
	#include "wisky_lcd_ej080na04b.c"
#elif defined(WISKY_LCD_CLAA100XA21XV)
	#include "wisky_lcd_claa100xa21xv.c"
#elif defined(WISKY_LCD_BF097XN)
	#include "wisky_lcd_bf097xn.c"
#elif defined(WISKY_LCD_HKZ0099)
	#include "wisky_lcd_hkz0099.c"
#elif defined(WISKY_LCD_HKZ070)
	#include "wisky_lcd_hkz070.c"
#elif defined(WISKY_LCD_LTN097XL01)
	#include "wisky_lcd_ltn097xl01.c"
#elif defined(WISKY_LCD_LD070WS2)
	#include "wisky_lcd_ld070ws2.c"
#elif defined(WISKY_LCD_CRD070TN)
	#include "wisky_lcd_crd070tn.c"
#elif defined(WISKY_LCD_LP097X02)
	#include "wisky_lcd_lp097x02.c"
#elif defined(WISKY_LCD_HJ080IA)
	#include "wisky_lcd_hj080ia.c"	
#elif defined(WISKY_RK610_LCD_HE080IA)
	#include "wisky_rk610_lcd_he080ia.c"	
#elif defined(WISKY_RK610_LCD_BF097XN02)
	#include "wisky_rk610_lcd_bf097xn02.c"	
#elif defined(WISKY_RK610_LCD_COMPA)
	#include "wisky_rk610_lcd_compa.c"	
#elif defined(WISKY_RK610_LCD_CLAA070WP03)
	#include "wisky_rk610_lcd_claa070wp03.c"		
	
#endif
