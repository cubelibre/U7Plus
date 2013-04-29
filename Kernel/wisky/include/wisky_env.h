/* wisky/include/wisk_env.h
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
 * V001:20120421 cd huang^create for rk3066 platform
 */

#ifndef _WISKY_ENV_H_
#define _WISKY_ENV_H_

#ifndef NULL
#define NULL ((void *)0)
#endif

#ifndef TRUE
#define TRUE		1
#endif

#ifndef FALSE
#define FALSE		0
#endif

//please define "WISKY_DEBUG" in your driver to enable local debug infomation
//#define WISKY_DEBUG
#ifdef WISKY_DEBUG
#define WPRINTK(format, args...)	do {		\
		printk(KERN_INFO "<WISKY-DEBUG> " format , ## args);	\
		} while (0)
#else
#define WPRINTK(format, args...)
#endif

/*
 * kernle log level, anything MORE serious than WISKY_LOG_LEVEL will print:
 *		1:KERN_ALERT
 *		2:KERN_CRIT
 *		3:KERN_ERR
 *		4:KERN_WARNING
 *		5:KERN_NOTICE
 *		6:KERN_INFO
 *		7:KERN_DEBUG
 */
#define WISKY_LOG_LEVEL		7

/*���ں����������޸�����������wisky MIDʱ����������ʹ��WISKY_MID��*/
#define WISKY_MID			1
/*�������ӹ��������ļ����ں�sys�ļ�ϵͳ�У���androidʹ�÷�����������*/
#define WISKY_USE_CONFIG_FILE		1
//����DEBUG�µ�LOG��־
//#define WISKY_DEBUG_LOGFILE   1
//�����Ե����豸��������/���ѹ��������ķ�ʱ��
//#define WISKY_DEBUG_DRV_SUSPEND

#define WISKY_KERNEL_VERSION    "ker_ver1.19@20130424"

/***************************************************************/
//ƽ̨ѡ��,һʱ��ֻ����ѡ������һ��
/***************************************************************/
//#define ROCKCHIP_RK29XX		1
#define ROCKCHIP_RK30XX		1

/***************************************************************/
//Ӳ���汾����,һʱ��ֻ����ѡ������һ��
//��"#define"��Ϊ"#!define"���Խ�ֹ��Ӧ�걻ȫ��������build_all.shִ�е�
//ע��:�궨����ĩβ��ֹʹ������ע��
/***************************************************************/
#define WISKY_BOARD_U7PLUS_TV10	1	//1280*800,PMU:AXP202,RK610
/***************************************************************/
//�ͻ�ѡ��,һʱ��ֻ����ѡ������һ��
//��"#define"��Ϊ"#!define"���Խ�ֹ��Ӧ�걻ȫ��������build_all.shִ�е�
//ע��:�궨����ĩβ��ֹʹ������ע��
/***************************************************************/
	/*---Android Ĭ��,_3GΪ������3G����---*/
//#define WISKY_OEM_DEFAULT			1
//#!define WISKY_OEM_DEFAULT_3G		1	//with on board 3G model
//#!define WISKY_OEM_DEFAULT_GPS	1	//with GPS model
	/*--- ���Թ���Ĭ�ϴ���---*/
//#!define WISKY_OEM_ALL_ON			1
	/*---����ΰҵ---*/
//#!define WISKY_OEM_WISKY			1
/*------pocketbook------*/
#define WISKY_OEM_POCKETBOOK_U7PLUS    1
/***************************************************************/
//Ӳ���������������ļ�����
/***************************************************************/
//U7plus TV 1.0 Hardware config
#include "../../wisky/include/cfg/wisky_cfg_u7plus_tv10.h"
#include "../../wisky/include/gpio/wisky_gpio_u7plus_tv10.h"
/***************************************************************/
//OEM�ͻ����������ļ�����
/***************************************************************/
#include "../../wisky/include/oem/config_pocketbook_u7plus.h"
/**************************************************************
 * �豸����������Դ����Ҫ����Ӧ��config�����ļ�������
 * ÿ����һ���µ�Ӳ��ģ�飬����������Ӧ�����Ӻ꣬
 * ��ʽ���£�WISKY_ģ������_ģ����
 *-------------------------------------------------------------
 *LCD--->:
 *	WISKY_LCD_20810800300017 = ��Դ8 inch 1024x768 LCD module
 *	WISKY_LCD_EJ080NA04B = ����8 inch 1024x768 LCD module
 *	WISKY_LCD_CLAA100XA21XV = 10 ��1024x768 ��
 *	WISKY_LCD_BF097XN = ����9.7 inch 1024x768
 *	WISKY_LCD_HKZ0099 = Ⱥ��8D TFT ������8"
 *	WISKY_LCD_HKZ070  = Ⱥ��7 inch 800*480
 *	WISKY_LCD_LTN097XL01 = ����1024x768 TFT LCD(����ʱ�豣��LVDS /PDN�Ÿߵ�ƽ������Ӱ)
 *	WISKY_LCD_LD070WS2 = 7��1024*600������
 *LCD Resolution--->:
 *	WISKY_LCD_WIDTH = ��ʾ��ֱ��ʿ���
 *	WISKY_LCD_HEIGHT = ��ʾ��ֱ��ʸ߶� 
 *Touchscreen--->:
 *	(������x, y�ἰ�����任P:��, N:��)
 *		TS_SWAP_PX_PY : input[x,y]=reg[x,y]
 *		TS_SWAP_PY_PX : input[x,y]=reg[y,x]
 *		TS_SWAP_PX_NY : input[x,y]=reg[x,-y]
 *		TS_SWAP_NX_PY : input[x,y]=reg[-x,y]
 *		TS_SWAP_NX_NY : input[x,y]=reg[-x,-y]
 *	WISKY_TS_GT8XX = GT828,GT827,GT813,GT816,GT818, ��ָ���ݴ�����
 *		[wisky_ts_gt8xx_m101a_gt828_pingbo.cfg]:M101A��GT828+ƽ����������
 *		[wisky_ts_gt8xx_m101b_gt813_pingbo.cfg]:M101B��GT813+ƽ����������
 *	WISKY_TS_FT5302 = FT5302 ��ָ���ݴ�����
 *	WISKY_TS_FT5306 = FT5306 ��ָ���ݴ�����
 *	WISKY_TS_FT5X0X = FT5X0Xϵ�ж�ָ���ݴ�����
 *		[wisky_ts_ft5x0x_m868_pingbo.fw]:M868R ��FT5x0x+ƽ����������
 *		[wisky_ts_ft5x0x_m868_qiutian.fw]:M868R ��FT5x0x+������������
 *		[wisky_ts_ft5x0x_m868r_ruishi.fw]:M868R ��FT5x0x+���ӹ�����������
 *  WISKY_TS_ZET6221 = ��ҫ ��ָ���ݴ�����
 *Touchscreen Resolution--->:
 *	WISKY_TOUCH_WIDTH = ������ֱ��ʿ���
 *	WISKY_TOUCH_HEIGHT = ������ֱ��ʸ߶� 
 *Keyboard--->:
 *	WISKY_KEYBOARD_GPIO
 *TouchKey--->:
 *	WISKY_TOUCHKEY_HA2605 =�㶥I2C�ӿ�HA2605��������
 *	WISKY_TOUCHKEY_CP2526 = ����΢I2C�ӿ�CP2526��������IC
 *G-Sensor--->:
 *	WISKY_GSENSOR_MMA7660
 *	WISKY_GSENSOR_DMARD06
 *Gsensor Direction--->:
 *	�������������ϱ�ֵxyz����Ӧ��GSENSOR�Ĵ���xyz���������,PΪ����NΪ��
 *	WISKY_GSENSOR_PX_PY_PZ = input[x,y,z] = reg[x,y,z]
 *	WISKY_GSENSOR_PY_NX_PZ = input[x,y,z] = reg[y,-x,z]
 *	WISKY_GSENSOR_NY_NX_PZ = input[x,y,z] = ret[-y,-x,z]
 *Gyroscope--->:
 *
 *Magnetometer--->:
 *	WISKY_MSENSOR_MMC3280 = I2C�ӿڴų�������ָ����
 *Light & Proximity Sensor--->
 *	WISKY_LIGHT_SENSOR_LTR502 = ʹ��LITE-ON LTR-502ALS-WA ���;�������һ������
 *Backlight--->:
 *	WISKY_BACKLIGHT_DEMO =sdk platform backlight driver
 *Battery--->:
 *	WISKY_BATTERY_ADC = ʹ��ADC ��������ĵ��ع�������
 *	WISKY_BATTERY_SMB347 = I2C�ӿ�SUMMIT SMB347 ����������
 *	WISKY_DC_USB_CHARGE_2IN1 = DC & USB ����������һ����
 *	WISKY_BATTERY_OZ8555 = ʹ��ADC�������������OZ8555��������IC
 *	WISKY_BATTERY_OZ8806 = OZ8806�����
 *	����������:
 *		CHARGE_CURRENT_LEVEL_3 = ���� ������
 *		CHARGE_CURRENT_LEVEL_2 = ��� ������
 *	
 *Sound--->:
 *	WISKY_CODEC_RT5631 = select rt5631 codec chip
 *RTC--->:
 *	WISKY_RTC_WM8326 = WM8326 I2C RTC chip support
 *PMU--->:
 *	WISKY_PMU_WM8326 = Wolfson WM8326 PMUоƬ�����ڲ�������RTC ����
 *Camera--->:
 *	ǰ������ͷѡ��
 *	WISKY_CAMERA_HI704_F = ʹ��HI704 Sensor ����ͷ
 *	��������ͷѡ��
 *	WISKY_CAMERA_HI253_B = ʹ��HI253 Sensor ����ͷ
 *GPS--->:
 *
 *HDMI--->:
 *
 *Headhone & Speaker switch--->:
 *	WISKY_SWITCH_GPIO = ����������л����⹦��
 *Vibrator--->:
 *	WISKY_VIBRATOR_TIMED_GPIO = ʹ������������
 *LED--->:
 *	WISKY_LED_GPIO = ֧��GPIO �ڿ���LED ����ͳһ�ӿڣ�����LED ��˸
 *Moblie 3G--->:
 *	WISKY_MOBILE_DEMO = 3G ����ģ������֧��
 *WiFi & Bluetooth--->:
 *	WISKY_WIFI_RTL8192C = Realteck RTL8192C USB WiFi Only ģ��
 *Encrypt--->:
 *	WISKY_ENCRYPT_PSIC307 = ʹ�ü���оƬPSIC307
 *CPU MAX Frequency--->:
 *	WISKY_MAX_CPUFREQ_1200M = ����CPU ����Ƶ��Ϊ1200 MHz
 *	WISKY_MAX_CPUFREQ_1400M = ����CPU ����Ƶ��Ϊ1400 MHz
 *	WISKY_MAX_CPUFREQ_1500M = ����CPU ����Ƶ��Ϊ1500 MHz
 *	WISKY_MAX_CPUFREQ_1600M = ����CPU ����Ƶ��Ϊ1600 MHz
 *DDR3 MAX Frequency--->:
 *	WISKY_DDR_FREQ = ����DDR3 Ƶ�ʣ���Χ:  ��400 MHz
 *DDR3 Size--->:
 *	WISKY_DDR_SIZE_512M = DDR3�����Ϊ512MB
 *	WISKY_DDR_SIZE_1024M = DDR3�����Ϊ1024MB
 *Boot charge logo--->:
 *	WISKY_LOGO_CHARGE_H = ����ػ�����������ͨ�ð�
 *Boot Logo--->:
 *	WISKY_LOGO_LINUX = linuxĬ�Ͽ���logo
 *	WISKY_LOGO_TECLAST_1024X600_H = ̨��ͨ��1024x600 ����
 *	WISKY_LOGO_PLOYER_1024X768 = ���ζ�MOMO8 1024x768 ����
 *  WISKY_LOGO_PLOYER_MOMO9_800X480 = ���ζ�MOMO9 800x480 ����
 *	WISKY_LOGO_TECLAST_768X1024 = ̨��768x1024����
 *	WISKY_LOGO_DANEW_DSLIDE972 = Danew Dslide972����logo
 *	WISKY_LOGO_EXPLAY_768X1024=����768x1024����logo
 *  WISKY_LOGO_EXPLAY_SURFER801_1024x768=����surfer801 768x1024����logo
 *	WISKY_LOGO_TIANJIAO_MID8500 = �콾MID8500
 *  WISKY_LOGO_DNS_800x480 = DNS 800x480
 *  WISKY_LOGO_DNS_1024x600 = DNS 1024x600
 *  WISKY_LOGO_DNS_1024x768 = DNS 1024x768
 ***************************************************************/

 
/*
*Select camera module for front or back
*/
//ǰ������ͷ����
#if defined(WISKY_CAMERA_HI704_F)
	#define WISKY_CAMERA_FRONT_NAME0	hi704
	#define WISKY_CAMERA_FRONT_ADDR0	 	0x60
#else	
	#define WISKY_CAMERA_FRONT_NAME0	nc
	#define WISKY_CAMERA_FRONT_ADDR0	 	0x00
#endif

#if defined(WISKY_CAMERA_GC0308_F)
	#define WISKY_CAMERA_FRONT_NAME1	gc0308
	#define WISKY_CAMERA_FRONT_ADDR1		0x42
#else
	#define WISKY_CAMERA_FRONT_NAME1	nc
	#define WISKY_CAMERA_FRONT_ADDR1		0x00
#endif

//��������ͷ����
#if defined(WISKY_CAMERA_HI253_B)
	#define WISKY_CAMERA_BACK_NAME		hi253
	#define WISKY_CAMERA_BACK_ADDR		0x40
#else
	#define WISKY_CAMERA_BACK_NAME		nc
	#define WISKY_CAMERA_BACK_ADDR		0x00
#endif


#endif//_WISKY_ENV_H_
