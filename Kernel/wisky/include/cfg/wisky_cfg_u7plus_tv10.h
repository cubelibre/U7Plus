/* 
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
 */

 #if defined(WISKY_BOARD_U7PLUS_TV10)

#ifndef _WISKY_CFG_U7PLUS_TV10_H_
#define _WISKY_CFG_U7PLUS_TV10_H_

//Ӳ���汾���ַ���
	#define WISKY_HW_VERSION		"U7+ TV1.0"
//����ģ��ѡ�����ã���wisky_env.h����˵��

	#define WISKY_RK610_LCD_CLAA070WP03		1	
	
	#define WISKY_TS_FT5X0X		1
	#define FT5X0X_FIRMWARE_DEFAULT		"wisky_ts_ft5x0x_u7plus.fw"
	#define FT5X0X_FIRMWARE_QiuTian		"wisky_ts_ft5x0x_u7plus.fw"
	
	#define FT5X0X_FIRMWARE_PingBo		"wisky_ts_ft5x0x_u7plus.fw"
	#define FT5X0X_FIRMWARE_RuiShi		"wisky_ts_ft5x0x_u7plus.fw"
	#define TS_SWAP_NX_NY 1
	//#define WISKY_TS_GT8XX		1
	#define GT8XX_CFG_DEFAULT	"wisky_ts_gt8xx_m830r_gt813_pingbo.cfg"
	#define GT8XX_CFG_QiuTian	"wisky_ts_gt8xx_m830r_gt813_qiutian.cfg"
	#define GT8XX_CFG_PingBo	"wisky_ts_gt8xx_m830r_gt813_pingbo.cfg"
	#define GT8XX_CFG_RuiShi	"wisky_ts_gt8xx_m830r_gt813_ruishi.cfg"

    
	#define WISKY_TS_GT9XX		1

	//#define WISKY_TS_ZET6221   1
	#define ZET6221_FIRMWARE_DEFAULT		"wisky_ts_zet6221_qiutian_w001.fw"
	#define ZET6221_FIRMWARE_QiuTian		"wisky_ts_zet6221_qiutian_w001.fw"
	#define ZET6221_FIRMWARE_PingBo 		"wisky_ts_zet6221_qiutian_w001.fw"
	#define ZET6221_FIRMWARE_RuiShi 		"wisky_ts_zet6221_ruishi_w001.fw"
	#define ZET6221_FIRMWARE_HaoEn	  	"wisky_ts_zet6221_qiutian_w001.fw"
	
	#define WISKY_LCD_WIDTH		800
	#define WISKY_LCD_HEIGHT		1280
	#define WISKY_TOUCH_WIDTH	1280
	#define WISKY_TOUCH_HEIGHT	800
	

	#define WISKY_GSENSOR_MMA7660		1
	#define WISKY_VIBRATOR_TIMED_GPIO	1
	#define WISKY_KEYBOARD_GPIO			1
	#define WISKY_BACKLIGHT_DEMO		1
	#define WISKY_DC_USB_CHARGE_2IN1	1
	//#define WISKY_CAMERA_HI253_B	1
	//#define WISKY_CAMERA_REG_B	"wisky_camera_hi253_w008.h"
	#define WISKY_CAMERA_HI704_F	1
	#define WISKY_CAMERA_REG_F	"wisky_camera_hi704_w008.h"	
	//#define WISKY_MAX_CPUFREQ_1600M_W001	1
	#define WISKY_MAX_CPUFREQ_1600M_W008      1
	#define WISKY_DDR_FREQ			360
	#define WISKY_LOGO_CHARGE_H		1
	#define WISKY_GSENSOR_NX_PZ_PY	1 
	#define WISKY_RTC_HYM8563		1
//  #define WISKY_CODEC_RT5631 1
	#define WISKY_SWITCH_GPIO	1
	#define WISKY_WIFI_RTL8188EU 1
	//#define WISKY_WIFI_RTL8192C 1
	#define WISKY_RK3066_RK3066B 1
/*---------------------------------------------------------
* ֧�ֵ�Ӳ��ģ��: RTL8192C WiFi + HDMI + Camera + Gsensor
* ��֧�ֵ�Ӳ��ģ��: 3G - Vibrate - LightSensor
----------------------------------------------------------*/
//WiFiģ��֧��:0--�ر�,1--����
#define WISKY_ENABLE_WIFI			1
//���ģ��֧��:0--�ر�,1--����
#if defined(WISKY_WIFI_BT_BCM4329)
#define WISKY_ENABLE_BLUETOOTH	1
#else
#define WISKY_ENABLE_BLUETOOTH	0	
#endif
//HDMIģ��֧��:0--�ر�,1--����
#define WISKY_ENABLE_HDMI		1
//GPSģ��֧��:0--�ر�,1--����
#define WISKY_ENABLE_GPS		0
///3Gģ��֧��:0--�ر�,1--����
#define WISKY_ENABLE_3G			1
//Cameraģ��֧��:0--�ر�,1--����
#define WISKY_ENABLE_CAMERA		1	
//Vibrateģ��֧��:0--�ر�,1--����
#define WISKY_ENABLE_VIBRATOR	0
//LightSensorģ��֧��:0--�ر�,1--����
#define WISKY_ENABLE_LSENSOR	0
//GSensorģ��֧��:0--�ر�,1--����
#define WISKY_ENABLE_GSENSOR	1	


/*
 *���败����У��������Ĭ�ϳ�ʼֵ��
 *��ֵ���ڲ�ͬ�Ļ����������Ӳ����λ�����ص�
 *��ͬ�����ܲ�ͬ����ͬ�Ļ�����Ҫ���²�����á�
*/
#define WISKY_CALIB_DEFAULT_X1          306
#define WISKY_CALIB_DEFAULT_Y1          3622
#define WISKY_CALIB_DEFAULT_X2          3743
#define WISKY_CALIB_DEFAULT_Y2          3652
#define WISKY_CALIB_DEFAULT_X3          286
#define WISKY_CALIB_DEFAULT_Y3          454
#define WISKY_CALIB_DEFAULT_X4          3737
#define WISKY_CALIB_DEFAULT_Y4          468
#define WISKY_CALIB_DEFAULT_X5          2016
#define WISKY_CALIB_DEFAULT_Y5          2029

#endif

#endif

