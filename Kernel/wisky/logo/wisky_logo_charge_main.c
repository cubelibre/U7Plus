/* wisky/logo/wisky_logo_charge.c
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
 *	1.Create for linux kernel boot up battery charge logo display driver.
 */
 
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/linux_logo.h>
#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/cpufreq.h>
#include <linux/fb.h>
#include <asm/tlbflush.h>
#include <mach/gpio.h>
#include <mach/ddr.h>
#include <mach/cru.h>
#include <mach/board.h>
#include "wisky_logo_charge_main.h"
#include <mach/iomux.h>

#define LOOPS_PER_USEC	13
#define LOOPS_PER_MSEC	12000
#define LOOP(loops) do { unsigned int i = loops; barrier(); while (--i) barrier(); } while (0)

int boot_charge_mode = 0;//0---no boot charge, 1---boot charge exist

#if defined (CONFIG_AXP_PEKINFO)
extern int	axp_pwr_pressshort;
extern int axp_pwr_presslong;
#endif


#define BATTERY_ADC_ADJUST	35//25
extern int battery_adc_value;
extern void wisky_battery_ealry_suspend(struct early_suspend *h);
extern void wisky_battery_ealry_resume(struct early_suspend *h);
//extern void request_suspend_state(suspend_state_t state);

extern int dc_adaptor_status;//0---dc removed, 1---dc insert;
extern int battery_capacity;
extern int usb_otg_op_state;
#if defined(WISKY_GPS_SIRF)
extern int wisky_gps_power_off(void);
#endif

static unsigned char blank_clut224_data[WISKY_LCD_WIDTH*WISKY_LCD_HEIGHT] __initdata = {0x20};
static unsigned char blank_clut224_clut[] __initdata = {
	0x00, 0x00, 0x00
};

struct linux_logo blank_clut224 __initdata = {
	.type	= LINUX_LOGO_CLUT224,
	.width	= WISKY_LCD_WIDTH,
	.height	= WISKY_LCD_HEIGHT,
	.clutsize	= 1,
	.clut	= blank_clut224_clut,
	.data = blank_clut224_data
};
extern struct fb_info *g_fb0_inf;
extern void fb_show_charge_logo(struct linux_logo *logo);
int wisky_charger_logo_display(int index)
{
	
		
	if(NULL == g_fb0_inf){
		pr_info("NULL framebuffer information, show logo fail!\n");
		return -EINVAL;
	}
	
	switch(index)
	{
		case LOGO_BLANK:
			{
				memset(blank_clut224_data, 0x20, WISKY_LCD_WIDTH*WISKY_LCD_HEIGHT);
				if(!fb_prepare_logo(g_fb0_inf, FB_ROTATE_UR,&blank_clut224))
					return 0;
			}
			break;
		case LOGO_NUM1:
			if(!fb_prepare_logo(g_fb0_inf, FB_ROTATE_UR,&logo_charge0_clut224))
					return 0;
			break;
		case LOGO_NUM2:
			if(!fb_prepare_logo(g_fb0_inf, FB_ROTATE_UR,&logo_charge1_clut224))
					return 0;
			break;
		case LOGO_NUM3:
			if(!fb_prepare_logo(g_fb0_inf, FB_ROTATE_UR,&logo_charge2_clut224))
					return 0;
			break;
		case LOGO_NUM4:
			if(!fb_prepare_logo(g_fb0_inf, FB_ROTATE_UR,&logo_charge3_clut224))
					return 0;
			break;
		case LOGO_NUM5:
			if(!fb_prepare_logo(g_fb0_inf, FB_ROTATE_UR,&logo_charge4_clut224))
					return 0;
			break;
//		case LOGO_NUM6:
//			set_logo(&logo_charge6_clut224);
//			fb_show_logo(inf->fb0, 0);
//			break;
		case LOGO_POOR:
			if(!fb_prepare_logo(g_fb0_inf, FB_ROTATE_UR,&logo_charge_poor_clut224))
					return 0;
			break;
		case LOGO_FULL:
			if(!fb_prepare_logo(g_fb0_inf, FB_ROTATE_UR,&logo_charge_full_clut224))
					return 0;
			break;
		case LOGO_BOOTUP:
			if(!fb_prepare_logo(g_fb0_inf, FB_ROTATE_UR,NULL))
					return 0;
			break;
		default:
			memset(blank_clut224_data, 0x20, WISKY_LCD_WIDTH*WISKY_LCD_HEIGHT);
			if(!fb_prepare_logo(g_fb0_inf, FB_ROTATE_UR,&blank_clut224))
					return 0;
			break;
	}
	fb_set_cmap(&g_fb0_inf->cmap, g_fb0_inf);
	fb_show_logo(g_fb0_inf, FB_ROTATE_UR);
	g_fb0_inf->fbops->fb_pan_display(&(g_fb0_inf->var), g_fb0_inf);
	
	return 0;
}

static inline void delay_500ns(void)
{
	LOOP(LOOPS_PER_USEC);
}

static inline void delay_300us(void)
{
	LOOP(300 * LOOPS_PER_USEC);
}

//param: when enable==1 set backlight on, enable==0 set off
static void wisky_boot_charge_bl(int enable)
{
	if(enable){
		#if defined(LCD_BL_EN_PIN)
		if(LCD_BL_EN_PIN != INVALID_GPIO){
			gpio_set_value(LCD_BL_EN_PIN, LCD_BL_EN_VALUE);
		}
		#endif
		
		#if defined(LCD_LVDS_PDN_PIN)
		if(LCD_LVDS_PDN_PIN != INVALID_GPIO){
			gpio_set_value(LCD_LVDS_PDN_PIN, LCD_LVDS_PDN_VALUE);
		}
		#endif

		#if defined(LCD_BL_PWM_PIN)
		rk30_mux_api_set(LCD_BL_PWM_MUX_NAME, LCD_BL_PWM_MUX_GPIO);
		gpio_direction_output(LCD_BL_PWM_PIN, LCD_BL_PWM_VALUE);
		#endif
	}else{
		#if defined(LCD_BL_EN_PIN)
		if(LCD_BL_EN_PIN != INVALID_GPIO){
			gpio_set_value(LCD_BL_EN_PIN, !LCD_BL_EN_VALUE);
		}
		#endif

		#if defined(LCD_LVDS_PDN_PIN)
		if(LCD_LVDS_PDN_PIN != INVALID_GPIO){
			gpio_set_value(LCD_LVDS_PDN_PIN, !LCD_LVDS_PDN_VALUE);
		}
		#endif
		
		#if defined(LCD_BL_PWM_PIN)
		rk30_mux_api_set(LCD_BL_PWM_MUX_NAME, LCD_BL_PWM_MUX_GPIO);
		gpio_direction_output(LCD_BL_PWM_PIN, !LCD_BL_PWM_VALUE);
		#endif
	}
}

static void wisky_boot_charge_poweroff(void)
{
	if(KEY_SHUTDOWN_PIN != INVALID_GPIO){
		gpio_request(KEY_SHUTDOWN_PIN, "poweronpin");
		gpio_direction_output(KEY_SHUTDOWN_PIN, GPIO_LOW);
		gpio_set_value(KEY_SHUTDOWN_PIN, GPIO_LOW);
	}
	if(pm_power_off){
		pm_power_off();
	}
}

//power off system when dc adaptor remove
static void dc_adaptor_check_and_poweroff(void)
{
	if(dc_adaptor_status == 0){
		pr_info("%s:DC removed, system will shut down...\n", __FUNCTION__);
		
	wisky_boot_charge_bl(0);
	
#if (WISKY_ENABLE_GPS)
		wisky_gps_power_off();
#endif
		wisky_boot_charge_poweroff();
	}
}

u32 apll, cpll, gpll, mode, clksel0;
u32 clkgate[4];
static void wisky_boot_charge_suspend(void)
{

	wisky_boot_charge_bl(0);
	wisky_charger_logo_display(LOGO_BLANK);
	//lcd_io_disable();
	wisky_battery_ealry_suspend((void *)0);
	pr_info("------>%s[%d]\n", __FUNCTION__, __LINE__);
	
//	cpufreq_driver_target(cpufreq_cpu_get(0), 408*1000, 0x00);//10,wisky-lxh@20111202^解决休眠唤醒后引起cpu频率无法调节的问题
}

static void wisky_boot_charge_resume(void)
{
//	cpufreq_driver_target(cpufreq_cpu_get(0), 816*1000, 0x00);

	pr_info("------>%s[%d]\n", __FUNCTION__, __LINE__);

	wisky_battery_ealry_resume((void *)0);
	//lcd_io_enable();
	mdelay(300);
	wisky_boot_charge_bl(1);
}

static void wisky_boot_charge_shutdown(void)
{

}

static void wisky_boot_charge_restart(void)
{
	gpio_request(KEY_SHUTDOWN_PIN, "poweronpin");
	gpio_direction_output(KEY_SHUTDOWN_PIN, GPIO_HIGH);
	gpio_set_value(KEY_SHUTDOWN_PIN,GPIO_HIGH);
	kernel_restart(NULL);	
}

//return: 0---no power key press, 1-- power key pressed
static int power_key_check(void)
{
	if(KEY_POWER_PIN != INVALID_GPIO){
		if(gpio_get_value(KEY_POWER_PIN) == GPIO_LOW){
			return 1;
		}else{
			return 0;
		}
	}

#if defined (CONFIG_AXP_PEKINFO)
	if(axp_pwr_pressshort){
		axp_pwr_pressshort = 0;
		return 1;
	}else{
		return 0;
	}
#endif
}

static int wisky_boot_charge_get_playkey_status(void)
{
#if defined (CONFIG_AXP_PEKINFO)
	if(axp_pwr_presslong){
		axp_pwr_presslong = 0;
		return 1;
	}else{
		return 0;
	}
#else
	if(power_key_check()){
		dc_adaptor_check_and_poweroff();
		mdelay(DELAY_TIME_MS);
		dc_adaptor_check_and_poweroff();
		if(power_key_check()){
			return 1;//key down
		}
	}
	return 0;
#endif
}

static int wisky_boot_charge_show_logos(int count, int bat_level)
{
	int i, j, logo_max;

	logo_max = 5;
	for(i = 0; i< count; i++){		
		for(j = 0; j < logo_max; j++){
			dc_adaptor_check_and_poweroff();
			wisky_charger_logo_display(LOGO_NUM1+j);
			dc_adaptor_check_and_poweroff();
			mdelay(DELAY_TIME_MS);
			dc_adaptor_check_and_poweroff();
			if(wisky_boot_charge_get_playkey_status())
				return 1;
		}
	}
	return 0;
}

static int wisky_boot_charge_show_full_logo(void)
{
	int i;
	
	for(i = 0; i < 10; i++){
		dc_adaptor_check_and_poweroff();
		wisky_charger_logo_display(LOGO_FULL);
		dc_adaptor_check_and_poweroff();
		mdelay(DELAY_TIME_MS);
		dc_adaptor_check_and_poweroff();
		if(wisky_boot_charge_get_playkey_status()){
			return 1;
		}
	}
	return 0;
}

int __init wisky_boot_charge_main(void)
{
	#if 0	//for test ...Lee
	printk("here... ...wulala... wisky_boot_charge_main\n");
	wisky_charger_logo_display(LOGO_BLANK);
	wisky_charger_logo_display(LOGO_BOOTUP);
	return 0;
	#endif
	int charge_full_cnt = 0;
	int full_display_times = 1;//充电满后提示次数	

	boot_charge_mode = 0;
	if(1 == usb_otg_op_state){
		//do not enter charge mode when USB OTG insert
		return 0;
	}

	WPRINTK("--------dc_adaptor_status=%d\n", dc_adaptor_status);
	//WPRINTK("--------battery_adc_value=%d\n", battery_adc_value);
	pr_info("--------battery_capacity=%d\n", battery_capacity);
	
	if(dc_adaptor_status == 0){
		//no DC adaptor insert, do not display charge
		//#if defined(WISKY_BATTERY_OZ8806)
		//电量低于阀值时显示低电logo
		if(battery_capacity < BATT_LEVEL_POOR){
		//#else
		//电量低于阀值时显示低电logo
		//if((battery_adc_value < BAT_DISCHARGE_ADC_MIN) || (battery_adc_value -BATTERY_ADC_ADJUST < BAT_DISCHARGE_ADC_MIN)){
		//#endif
			//clear lcd before show charge logo
			wisky_charger_logo_display(LOGO_BLANK);
			msleep(100);
			wisky_boot_charge_bl(1);			
			wisky_charger_logo_display(LOGO_POOR);
			
			mdelay(1000);
			wisky_boot_charge_bl(0);
			mdelay(500);
			pr_info("%s:low battery, shut down...\n", __FUNCTION__);
			wisky_boot_charge_poweroff();
			while(1);
		}else{

			wisky_charger_logo_display(LOGO_BLANK);
			msleep(100);
			//wisky_boot_charge_bl(1);			
			wisky_charger_logo_display(LOGO_BOOTUP);
			return 0;
		}
	}

	pr_info("------>Charge mode enter\n");
	//clear lcd before show charge logo
	wisky_charger_logo_display(LOGO_BLANK);
	printk("wisky_charger_logo_display(LOGO_BLANK);\n");
	msleep(100);//延时防止闪白屏
	wisky_boot_charge_bl(1);	
	while(1){
		if(battery_capacity == BATT_LEVEL_FULL){
			dc_adaptor_check_and_poweroff();
			wisky_charger_logo_display(LOGO_BLANK);
			if(wisky_boot_charge_show_full_logo() == 1){
				goto exit;
			}
		}else{
			dc_adaptor_check_and_poweroff();
			wisky_charger_logo_display(LOGO_BLANK);
			if(wisky_boot_charge_show_logos(3, battery_capacity) == 1){
				goto exit;
			}
		}
		#if defined (CONFIG_AXP_PEKINFO)
		if(axp_pwr_pressshort){
			axp_pwr_pressshort = 0;
		}
		#endif

		dc_adaptor_check_and_poweroff();
		wisky_boot_charge_suspend();

		while(1){
			if(battery_capacity == BATT_LEVEL_FULL && full_display_times > 0){
				if(charge_full_cnt++ > 10/*FULL_SHOW_LOGO_TIME*/){
					charge_full_cnt = 0;
					full_display_times--;
					break;
				}
			}
			dc_adaptor_check_and_poweroff();
			if(power_key_check()){
				break;
			}

			dc_adaptor_check_and_poweroff();
			mdelay(10);
		}
		
		dc_adaptor_check_and_poweroff();		
		wisky_boot_charge_resume();		
	}
exit:
	pr_info("------>Charge mode exit\n");
	rk30_mux_api_set(LCD_BL_PWM_MUX_NAME, LCD_BL_PWM_MUX_MODE);
	//lcd backlight off here, will be backlight on in backlihgt driver init
	if(LCD_BL_EN_PIN != INVALID_GPIO){
		gpio_set_value(LCD_BL_EN_PIN, !LCD_BL_EN_VALUE);
	}
	//
	
	boot_charge_mode = 1;
	wisky_charger_logo_display(LOGO_BLANK);
	wisky_charger_logo_display(LOGO_BOOTUP);
	gpio_request(KEY_SHUTDOWN_PIN, "poweronpin");
	gpio_direction_output(KEY_SHUTDOWN_PIN, GPIO_HIGH);
	gpio_set_value(KEY_SHUTDOWN_PIN, GPIO_HIGH);

	return 0;
}
EXPORT_SYMBOL(wisky_boot_charge_main);
