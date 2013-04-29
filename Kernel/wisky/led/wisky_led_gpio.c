/* wisky/led/wisky_led_gpio.c
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
 *	1.Create driver for LED control by GPIO.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <mach/gpio.h>

/*
 * touch key board led control function
 * param: 
 *	enable---positive number to turn on touch keyboard LED, 0 to turn off touch keyboard LED
 * return: return 0 if success, else negative number
 */
int wisky_touchkey_led_set(int enable)
{
	if(TOUCHKEY_LED_PIN != INVALID_GPIO){
		if(!strcmp(ANDROID_CONFIG_X7, "1")){
			if(enable != 0){
				gpio_set_value(TOUCHKEY_LED_PIN, TOUCHKEY_LED_ACTIVE);
			}else{
				gpio_set_value(TOUCHKEY_LED_PIN, !TOUCHKEY_LED_ACTIVE);
			}
		}
	}
	return 0;
}
EXPORT_SYMBOL(wisky_touchkey_led_set);

/*
 * gps led control function
 * param: 
 *	enable---positive number to turn on GPS LED, 0 to turn off LED
 * return: return 0 if success, else negative number
 * -------TODO if we have GPS led in hardware------
 */
 #if 0
int wisky_gps_led_set(int enable)
{
	if(GPS_LED_PIN != NULL_GPIO){
		if(enable != 0){
			gpio_set_value(GPS_LED_PIN, GPIO_HIGH);
		}else{
			gpio_set_value(GPS_LED_PIN, GPIO_LOW);
		}
	}
	return 0;	
}
 EXPORT_SYMBOL(wisky_gps_led_set);
#endif
