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
 * V001:20120618 cd huang^add for wisky mid gpio config
 */

#if defined(WISKY_BOARD_U7PLUS_TV10)

#ifndef _WISKY_GPIO_U7PLUS_TV10_H_
#define _WISKY_GPIO_U7PLUS_TV10_H_

/*******************LCD************************/
#define LCD_PWR_EN_PIN			RK30_PIN6_PB0	//LCD电源供电使能脚
#define LCD_PWR_EN_MUX_NAME	NULL
#define LCD_PWR_EN_MUX_MODE	NULL
#define LCD_PWR_EN_VALUE		GPIO_HIGH		//使能有效电平

#define LCD_RESET_PIN			RK30_PIN1_PC1		//LCD模块复位控制脚
#define LCD_RESET_MUX_NAME		GPIO1C1_CIFDATA3_RMIITXEN_NAME
#define LCD_RESET_MUX_MODE		GPIO1C_GPIO1C1
#define LCD_RESET_VALUE			GPIO_HIGH

#define LCD_STANDBY_PIN			RK30_PIN2_PC6	//LCD模块standby睡眠控制脚
#define LCD_STANDBY_MUX_NAME	GPIO2C6_LCDC1DATA22_SPI1RXD_HSADCDATA3_NAME
#define LCD_STANDBY_MUX_MODE	GPIO2C_GPIO2C6
#define LCD_STANDBY_VALUE		GPIO_HIGH		//LCD模块standby睡眠有效电平

#define LCD_CS_PIN			RK30_PIN1_PC3
#define LCD_CS_MUX_NAME	GPIO1C3_CIFDATA5_RMIITXD0_NAME
#define LCD_CS_MUX_MODE	GPIO1C_GPIO1C3
#define LCD_CS_VALUE		GPIO_HIGH

#define LCD_MODEL_DET1_PIN			RK30_PIN0_PD0
#define LCD_MODEL_DET1_MUX_NAME	GPIO0D0_I2S22CHCLK_SMCCSN0_NAME
#define LCD_MODEL_DET1_MUX_MODE	GPIO0D_GPIO0D0
#define LCD_MODEL_DET1_VALUE		GPIO_HIGH

#define LCD_MODEL_DET2_PIN			RK30_PIN0_PD1
#define LCD_MODEL_DET2_MUX_NAME	GPIO0D1_I2S22CHSCLK_SMCWEN_NAME
#define LCD_MODEL_DET2_MUX_MODE	GPIO0D_GPIO0D1
#define LCD_MODEL_DET2_VALUE		GPIO_HIGH

/*******************Backlight************************/
#define LCD_BL_EN_PIN			RK30_PIN6_PB3	//LCD背光电源使能控制脚
#define LCD_BL_EN_MUX_NAME		NULL
#define LCD_BL_EN_MUX_MODE		NULL
#define LCD_BL_EN_VALUE			GPIO_LOW		//LCD背光电源使能有效电平

#define LCD_BL_PWM_ID			2
#define LCD_BL_PWM_PIN			RK30_PIN0_PD6
#define LCD_BL_PWM_MUX_NAME	GPIO0D6_PWM2_NAME
#define LCD_BL_PWM_MUX_MODE	GPIO0D_PWM2
#define LCD_BL_PWM_MUX_GPIO	    GPIO0D_GPIO0D6
#define LCD_BL_PWM_VALUE		GPIO_HIGH

#define LCD_BL_MIN_BRIGHTNESS	40		//LCD背光亮度调节最小值

/*******************charge dalicate led************************/
#define CHARGE_BLINK_PIN			RK30_PIN4_PD6
#define CHARGE_BLINK_MUX_NAME	GPIO4D6_SMCDATA14_TRACEDATA14_NAME
#define CHARGE_BLINK_MUX_GPIO	        GPIO4D_GPIO4D6
#define CHARGE_BLINK_EFFECT_VALUE		GPIO_HIGH

/*******************Keyboard************************/
//GPIO key
#define KEY_SHUTDOWN_PIN		INVALID_GPIO
#define KEY_SHUTDOWN_MUX_NAME	NULL
#define KEY_SHUTDOWN_MUX_MODE	NULL

#define KEY_POWER_PIN			INVALID_GPIO
#define KEY_POWER_MUX_NAME		NULL
#define KEY_POWER_MUX_MODE		NULL
#define KEY_HOME_PIN				INVALID_GPIO
#define KEY_HOME_MUX_NAME		NULL
#define KEY_HOME_MUX_MODE		NULL
#define KEY_MENU_PIN				INVALID_GPIO
#define KEY_MENU_MUX_NAME		NULL
#define KEY_MENU_MUX_MODE		NULL
#define KEY_ESC_PIN				INVALID_GPIO
#define KEY_ESC_MUX_NAME		NULL
#define KEY_ESC_MUX_MODE		NULL
#define KEY_VOLUMEUP_PIN		INVALID_GPIO
#define KEY_VOLUMEUP_MUX_NAME	NULL
#define KEY_VOLUMEUP_MUX_MODE	NULL
#define KEY_VOLUMEDOWN_PIN		INVALID_GPIO
#define KEY_VOLUMEDOWN_MUX_NAME	NULL
#define KEY_VOLUMEDOWN_MUX_MODE	NULL
#define KEY_SEARCH_PIN			INVALID_GPIO
#define KEY_SEARCH_MUX_NAME	NULL
#define KEY_SEARCH_MUX_MODE	NULL
#define KEY_CAMERA_PIN			INVALID_GPIO
#define KEY_CAMERA_MUX_NAME	NULL
#define KEY_CAMERA_MUX_MODE	NULL

//ADC key ADC channel
#define KEY_ADC_CHANNEL				1	//ADC按键采样通道, if do not use ADC, set to -1
//ADC key value
#define KEY_INVALID	1024
#define KEY_HOME_ADC_VALUE			332//KEY_INVALID//	//HOME按键ADC值
#define KEY_MENU_ADC_VALUE			KEY_INVALID//MENU按键ADC值
#define KEY_ESC_ADC_VALUE			KEY_INVALID//Back返回按键ADC值
#define KEY_VOLUMEUP_ADC_VALUE		0		//音量加按键ADC值
#define KEY_VOLUMEDOWN_ADC_VALUE	134		//音量减按键ADC值


/*******************Touchscreen********************/
#define TS_POWER_PIN   			RK30_PIN1_PC4	//触摸屏电源使能控制脚INVALID_GPIO
#define TS_POWER_MUX_NAME		GPIO1C4_CIFDATA6_RMIIRXERR_NAME
#define TS_POWER_MUX_MODE		GPIO1C_GPIO1C4
#define TS_POWER_ON_VALUE		GPIO_HIGH	//触摸屏电源使能有效电平
#define TS_INT_PIN				RK30_PIN4_PC2	//触摸屏中断脚
#define TS_INT_MUX_NAME			GPIO4C2_SMCDATA2_TRACEDATA2_NAME
#define TS_INT_MUX_MODE			GPIO4C_GPIO4C2
#define TS_RESET_PIN 				RK30_PIN1_PC0	//触摸屏复位脚
#define TS_RESET_MUX_NAME		GPIO1C0_CIF1DATA2_RMIICLKOUT_RMIICLKIN_NAME
#define TS_RESET_MUX_MODE		GPIO1C_GPIO1C0
#define TS_RESET_VALUE			GPIO_HIGH
#define TS_SHUTDOWN_PIN 			INVALID_GPIO	//触摸屏睡眠控制脚
#define TS_SHUTDOWN_MUX_NAME	NULL
#define TS_SHUTDOWN_MUX_MODE	NULL
#define TS_SHUTDOWN_VALUE		GPIO_LOW	//触摸屏睡眠有效电平

#define TS_CHECK_ADC_CHANNEL	2	//触摸屏类型检测ADC通道

/*******************TouchKey********************/
//I2C Touch Key
#define TOUCHKEY_INT_PIN				INVALID_GPIO
#define TOUCHKEY_INT_MUX_NAME		NULL
#define TOUCHKEY_INT_MUX_MODE		NULL
#define TOUCHKEY_LED_PIN				INVALID_GPIO
#define TOUCHKEY_LED_MUX_NAME		NULL
#define TOUCHKEY_LED_MUX_MODE		NULL
#define TOUCHKEY_LED_ACTIVE			GPIO_HIGH		//LED灯点亮有效电平

/*******************G-Sensor********************/
#define GSENSOR_INT_PIN			RK30_PIN4_PC0
#define GSENSOR_INT_MUX_NAME	GPIO4C0_SMCDATA0_TRACEDATA0_NAME
#define GSENSOR_INT_MUX_MODE	GPIO4C_GPIO4C0

#define GSENSOR_ADJUST_X		0	//Gsensor x轴水平放置时校准值
#define GSENSOR_ADJUST_Y		0	//Gsensor y轴水平放置时校准值
#define GSENSOR_ADJUST_Z			0	//Gsensor z轴水平放置时校准值

/*****************Light & Proximity Sensor*************/
#define LIGHT_SENSOR_INT_PIN			INVALID_GPIO
#define LIGHT_SENSOR_INT_MUX_NAME		NULL
#define LIGHT_SENSOR_INT_MUX_MODE		NULL

/*******************Audio********************/
#if 1
#define HEADPHONE_DET_PIN		RK30_PIN2_PC7	//headphone detect pin
#define HEADPHONE_DET_MUX_NAME	GPIO2C7_LCDC1DATA23_SPI1CSN1_HSADCDATA4_NAME
#define HEADPHONE_DET_MUX_MODE	GPIO2C_GPIO2C7
#define HEADPHONE_DET_IN_VALUE		GPIO_HIGH		//headphone input level
#define HEADPHONE_MUTE_PIN		RK30_PIN1_PC2		//headphone mute pin
#define HEADPHONE_MUTE_MUX_NAME	GPIO1C2_CIF1DATA4_RMIITXD1_NAME
#define HEADPHONE_MUTE_MUX_MODE	GPIO1C_GPIO1C2
#define HEADPHONE_MUTE_ACTIVE_LEVEL	GPIO_HIGH	//headphone mute active level
#define SPEAKER_CTRL_PIN       RK30_PIN2_PA1   //speaker enable pin
#define SPEAKER_CTRL_MUX_NAME      GPIO2A1_LCDC1DATA1_SMCADDR5_NAME
#define SPEAKER_CTRL_MUX_MODE      GPIO2A_GPIO2A1
#define SPEAKER_EN_VALUE		GPIO_HIGH		//speaker enable active level
#else
#define HEADPHONE_DET_PIN		INVALID_GPIO	//headphone detect pin
#define HEADPHONE_DET_MUX_NAME	NULL
#define HEADPHONE_DET_MUX_MODE	NULL
#define HEADPHONE_DET_IN_VALUE		GPIO_HIGH		//headphone input level
#define HEADPHONE_MUTE_PIN		INVALID_GPIO		//headphone mute pin
#define HEADPHONE_MUTE_MUX_NAME	NULL
#define HEADPHONE_MUTE_MUX_MODE	NULL
#define HEADPHONE_MUTE_ACTIVE_LEVEL	GPIO_LOW	//headphone mute active level
#define SPEAKER_CTRL_PIN		INVALID_GPIO	//speaker enable pin
#define SPEAKER_CTRL_MUX_NAME		NULL
#define SPEAKER_CTRL_MUX_MODE		NULL
#define SPEAKER_EN_VALUE		GPIO_HIGH		//speaker enable active level
#endif
#define HEADPHONE_VOLUME	0xdF
#define SPEAKER_VOLUME		0xc8
#define MIC_BOOST_GAIN		0x1100

/*******************RTC********************/
#define RTC_INT_PIN			RK30_PIN6_PA2
#define RTC_INT_MUX_NAME		NULL
#define RTC_INT_MUX_MODE		NULL

/****************WiFi & Bluetooth*****************/
#define WIFI_BT_POWER_PIN		RK30_PIN3_PD0	//WiFi && Bluetooth电源使能控制脚
#define WIFI_BT_POWER_MUX_NAME	GPIO3D0_SDMMC1PWREN_NAME
#define WIFI_BT_POWER_MUX_MODE	GPIO3D_GPIO3D0


#define WIFI_RESET_PIN			INVALID_GPIO	//WiFi Reset 控制脚
#define WIFI_RESET_MUX_NAME		NULL
#define WIFI_RESET_MUX_MODE		NULL
#define BT_RESET_PIN				INVALID_GPIO	//BT Reset 控制脚
#define BT_RESET_MUX_NAME		NULL
#define BT_RESET_MUX_MODE		NULL
#define BT_WAKE_PIN				INVALID_GPIO	//BT Wake 控制脚
#define BT_WAKE_MUX_NAME		NULL
#define BT_WAKE_MUX_MODE		NULL

/*******************Mobile 3G********************/
/*#define MOBILE_PWR_ON_PIN		INVALID_GPIO	///3G模块电源使能控制脚
#define MOBILE_PWR_ON_MUX_NAME	NULL
#define MOBILE_PWR_ON_MUX_MODE	NULL
//#define MOBILE_RESET_PIN			INVALID_GPIO	///3G模块复位控制脚
//#define MOBILE_RESET_MUX_NAME	NULL
//#define MOBILE_RESET_MUX_MODE	NULL
*/

/*******************HDMI********************/
/*#define HDMI_INT_PIN				INVALID_GPIO	//for HDMI irq
#define HDMI_INT_MUX_NAME			NULL
#define HDMI_INT_MUX_MODE			NULL
*/
/*******************Camera********************/
#define CAMERA_PWR_EN_PIN			RK30_PIN1_PC6
#define CAMERA_PWR_EN_MUX_NAME	GPIO1C6_CIFDATA8_RMIIRXD1_NAME
#define CAMERA_PWR_EN_MUX_MODE	GPIO1C_GPIO1C6
#define CAMERA_PWR_EN_VALUE		GPIO_HIGH
#define CAMERA_FRONT_PDN_PIN			RK30_PIN1_PB4	//前置摄像头PowerDown控制脚
#define CAMERA_FRONT_PDN_MUX_NAME	GPIO1B4_CIF0DATA0_NAME
#define CAMERA_FRONT_PDN_MUX_MODE	GPIO1B_GPIO1B4
#define CAMERA_BACK_PDN_PIN				RK30_PIN1_PB5	//后置摄像头PowerDown控制脚
#define CAMERA_BACK_PDN_MUX_NAME	GPIO1B5_CIF0DATA1_NAME
#define CAMERA_BACK_PDN_MUX_MODE	GPIO1B_GPIO1B5

/*******************GPS********************/
/*#define GPS_POWER_PIN			INVALID_GPIO	//GPS 模块电源控制
#define GPS_POWER_MUX_NAME		NULL
#define GPS_POWER_MUX_MODE		NULL
#define GPS_ANT_POWER_PIN			NULL_GPIO 	//GPS天线电源控制脚
#define GPS_ANT_POWER_MUX_NAME	NULL
#define GPS_ANT_POWER_MUX_MODE	NULL
#define GPS_RESET_PIN				INVALID_GPIO
#define GPS_RESET_MUX_NAME		NULL
#define GPS_RESET_MUX_MODE		NULL
#define GPS_ON_OFF_PIN			INVALID_GPIO
#define GPS_ON_OFF_MUX_NAME		NULL
#define GPS_ON_OFF_MUX_MODE		NULL
*/
/*******************Power Manager*****************/
#define USB_DETECT_PIN			INVALID_GPIO//RK30_PIN6_PA3	//USB插拔检测脚
#define USB_DETECT_MUX_NAME		NULL
#define USB_DETECT_MUX_MODE		0
#define USB_DETECT_VALUE			GPIO_LOW
#define DC_DETECT_PIN			INVALID_GPIO	//DC直流电插/拔检测脚
#define DC_DETECT_MUX_NAME		NULL
#define DC_DETECT_MUX_MODE		0
#define DC_DETECT_VALUE			GPIO_LOW		//DC插入状态电平
#define HOLE_DC_DETECT_PIN			RK30_PIN6_PA5	//圆孔DC直流电插/拔检测脚
#define HOLE_DC_DETECT_MUX_NAME	NULL
#define HOLE_DC_DETECT_MUX_MODE	0
#define HOLE_DC_DETECT_VALUE		GPIO_LOW
#define LOW_BATT_WAKEUP_PIN		RK30_PIN6_PA0
#define LOW_BATT_WAKEUP_MUX_NAME	NULL
#define LOW_BATT_WAKEUP_MUX_MODE	0

#define BATTERY_CHARGE_PWR_EN_PIN			INVALID_GPIO	//充电电源使能控制
#define BATTERY_CHARGE_PWR_EN_MUX_NAME	NULL
#define BATTERY_CHARGE_PWR_EN_MUX_MODE	NULL
#define BATTERY_CHARGE_PWR_EN_VALUE		GPIO_LOW

#define BATTERY_CHARGE_EN_PIN			INVALID_GPIO//RK30_PIN0_PD4	//电池充电使能控制脚
#define BATTERY_CHARGE_EN_MUX_NAME		NULL//GPIO0D4_I2S22CHSDI_SMCADDR0_NAME
#define BATTERY_CHARGE_EN_MUX_MODE		0//GPIO0D_GPIO0D4
#define BATTERY_CHARGE_EN_VALUE			GPIO_HIGH	//电池充电使能有效电平

#define BATTERY_CHARGE_STAT1_PIN		INVALID_GPIO	//电池充电指示检测脚1
#define BATTERY_CHARGE_STAT1_MUX_NAME	NULL
#define BATTERY_CHARGE_STAT1_MUX_MODE	NULL
#define BATTERY_CHARGE_STAT2_PIN		RK30_PIN6_PA6	//电池充电指示检测脚2
#define BATTERY_CHARGE_STAT2_MUX_NAME	NULL
#define BATTERY_CHARGE_STAT2_MUX_MODE	0

#define CHARGE_CURRENT_CTL1_PIN			INVALID_GPIO	//电池充电电流控制脚1
#define CHARGE_CURRENT_CTL1_MUX_NAME	NULL
#define CHARGE_CURRENT_CTL1_MUX_MODE	0
#define CHARGE_CURRENT_CTL1_ON_VALUE	GPIO_HIGH
#define CHARGE_CURRENT_CTL2_PIN			INVALID_GPIO//RK30_PIN0_PD5	//电池充电电流控制脚2
#define CHARGE_CURRENT_CTL2_MUX_NAME	GPIO0D5_I2S22CHSDO_SMCADDR1_NAME
#define CHARGE_CURRENT_CTL2_MUX_MODE	GPIO0D_GPIO0D5
#define CHARGE_CURRENT_CTL2_ON_VALUE	GPIO_LOW

#define CHARGE_CURRENT_LEVEL_2		1

#define BAT_DISCHARGE_ADC_MIN			690		//电池放电低压对应ADC 检测值
#define BAT_DISCHARGE_ADC_MAX			790		//电池放电高压对应ADC 检测值
#define BAT_DCIN_NOCHARGE_ADC_MIN	(BAT_DISCHARGE_ADC_MIN+40)	//接充电器不充电时,电池低压对应ADC 检测值
#define BAT_DCIN_NOCHARGE_ADC_MAX	(BAT_DISCHARGE_ADC_MAX+40)	//接充电器不充电时,电池高压对应ADC 检测值
#define BAT_CHARGE_ADC_MIN		760	//接充电器充电时,电池低压对应ADC 检测值
#define BAT_CHARGE_ADC_MAX		840	//接充电器充电时,电池高压对应ADC 检测值
#define BATTERY_VOLTAGE_MIN		3500		//关机对应电池电压
#define BATTERY_VOLTAGE_MAX		4150		//满电对应电池电压

#define SAMPLE_COUNT_DISCHARGE		450//300 //放电时采样时间:300--1 minutes, 600--2 minutes
#define SAMPLE_COUNT_CHARGE_MIN	550 //电量低于60%充电时采样时间:300--1 minutes, 600--2 minutes
#define SAMPLE_COUNT_CHARGE_MAX	600 //电量高于60%充电时采样时间:300--1 minutes, 600--2 minutes
#define BATTER_CHARGE_FULL_TIME_MIN		30//minutes, charge full after trigger 40 minutes

#define BATTERY_ADC_CHANNEL		0			//电池电压ADC转换通道
#define DC_ADC_CHANNEL			-1			//DC直流电电压ADC转换通道
#define HOLE_DC_ADC_CHANNEL		-1			//圆孔充电器ADC转换通道

#define USB_CHARGE_ADC_ADJUST_VALUE		28	//使用PC机USB充电时ADC采样校准值
#define DC_CHARGE_ADC_ADJUST_VALUE			45	//使用DC直流充电器充电时ADC采样校准值
#define HOLE_DC_CHARGE_ADC_ADJUST_VALUE	50	//使用圆孔DC直流充电器充电时ADC采样校准值

//axp202 battery config
#define AXP_BATTERY_CAP       4500	//电池总容量mAh
#define AXP_BATTERY_RDC        130		// 电池内阻 毫欧
#define AXP_CHARGE_CUR_LCDON	600	//亮屏时充电电流mA
#define AXP_CHARGE_CUR_LCDOFF	1500//1400	//灭屏时充电电流mA

/******************Vibrator*****************/
#define VIBRATOR_EN_PIN			RK30_PIN0_PA4	//振荡器控制脚
#define VIBRATOR_EN_MUX_NAME	GPIO0A4_PWM1_NAME
#define VIBRATOR_EN_MUX_MODE	GPIO0A_GPIO0A4	//GPIO0A_PWM1
#define VIBRATOR_ACTIVE_LOW		FALSE	//是否低电平使能

/******************Ethernet*****************/
#define ETH_PHY_POWER_PIN		INVALID_GPIO
#define ETH_PHY_POWER_MUX_NAME	NULL
#define ETH_PHY_POWER_MUX_MODE	NULL

/******************* tp adc value *******************/
#define TP_ADC_VALUE_QIUTIAN		133
#define TP_ADC_VALUE_PINGBO		335
#define TP_ADC_VALUE_RUISHI		610
#define TP_ADC_VALUE_HAOEN		750
#define TP_ADC_VALUE_NULL1		860
/******************Recover Setup*****************/
//#define RECOVER_PIN				INVALID_GPIO	//升级键
/********************OTG***************************/
#define OTG_DEV_BUS_PIN 
#endif
#endif

