/* wisky/battery/wisky_battery_adc.c
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
 * V001:20120418 cd huang
 *	1.Create battery charge manager driver for wisky MID project.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/adc.h>
#include <linux/rtc.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <mach/iomux.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <asm/types.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/mach/time.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <linux/wakelock.h>
#include <linux/wisky_api.h>
#include "wisky_battery_smb347.h"

#define BATT_DRIVER_NAME			"wisky_battery"

#define AC_INSERT_VALUE     		600//700
#define HOLE_AC_INSERT_VALUE 	600

#define BATT_MIN_VOL_VALUE		BATTERY_VOLTAGE_MIN
#define BATT_FULL_VOL_VALUE		BATTERY_VOLTAGE_MAX
#define BATT_RANGE_VOL_VALUE	(BATT_FULL_VOL_VALUE-BATT_MIN_VOL_VALUE)

#define BATT_MIN_ADC_VALUE		BAT_DISCHARGE_ADC_MIN
#define BATT_FULL_ADC_VALUE		BAT_DISCHARGE_ADC_MAX
#define BATT_MIN_DCIN_NOCHARGE_ADC	BAT_DCIN_NOCHARGE_ADC_MIN
#define BATT_FULL_DCIN_NOCHARGE_ADC	BAT_DCIN_NOCHARGE_ADC_MAX
#define BATT_MIN_CHARGE_ADC_VALUE		BAT_CHARGE_ADC_MIN
#define BATT_FULL_CHARGE_ADC_VALUE		BAT_CHARGE_ADC_MAX


#if 0//defined(WISKY_DEBUG)
#undef SAMPLE_COUNT_DISCHARGE
#undef SAMPLE_COUNT_CHARGE_MIN
#undef SAMPLE_COUNT_CHARGE_MAX
#define SAMPLE_COUNT_DISCHARGE		100	//放电时采样时间:300--1 minutes, 600--2 minutes
#define SAMPLE_COUNT_CHARGE_MIN	100	//电量低于60%充电时采样时间:300--1 minutes, 600--2 minutes
#define SAMPLE_COUNT_CHARGE_MAX	200	//电量高于60%充电时采样时间:300--1 minutes, 600--2 minutes
#endif

#define ADC_SAMPLE_RATE		1//10	//sample 10 times than get average value

#define BATT_LEVEL_FULL		100
#define BATT_LEVEL_EMPTY		0
#define BATT_PRESENT_TRUE	1
#define BATT_PRESENT_FALSE	0

#define AD_NO_BATT_VALE		200
#define AD_NO_DC_VALE		200

//define usb plug
#define USB_PLUGIN	1
#define USB_PLUGOUT	0

//DC status by ADC detect usb power pin
//#define DC_REMOVE		(0x00)
#define DC_INSERT			(0x01)
#define HOLE_DC_INSERT 	(0x02)	//hole dc  status
#define USB_HOLE_DC_MASK (DC_INSERT|HOLE_DC_INSERT)

int hole_dc_irq = -1;

#define BATT_CHARGE_POLL		(jiffies+20)//10ms each jiffy

//#define USE_BATT_LEVEL_TABLE 	1

//check battery charge full by read RTC timer
#define CHARGE_FULL_CHECK_BY_RTC		1

struct bat_level_filter_tag{
	#define BAT_FILTER_SIZE  10	//sample times
	int count;
	int level[BAT_FILTER_SIZE];
	int level_temp;
};
static struct bat_level_filter_tag bat_level_filter;


struct bat_adc_filter_tag{
	#define BAT_ADC_SIZE  10	//sample times
	int count;
	int adc[BAT_ADC_SIZE];
	int adc_temp;
};
static struct bat_adc_filter_tag bat_adc_filter;

struct power_property_info{
	int status;
	int health;
	int capacity;
	int present;
	int voltage_now;
};

struct wisky_battery_info {
	#if defined(WISKY_BATTERY_SMB347)
	struct i2c_client *client;
	#endif
	int early_suspend_state;//driver early suspend status 0---wake, 1--suspend
	int suspend_state;//driver early suspend status 0---wake, 1--suspend
	struct wake_lock wakelock;	//hold wake state when external dc insert
	bool wakelock_state;
	struct workqueue_struct *workqueue;
#if defined(LOW_BATT_WAKEUP_PIN)
	int low_batt_gpio;
	int low_batt_irq;
	int low_batt_wakeup;	//1--low battery wakeup, 0---normal battery
	int low_batt_wakeup_count;
	struct delayed_work lowbat_delay_work;
#endif
	struct power_supply ac;
	struct power_supply battery;
	struct power_supply usb;
	struct adc_client *bat_adc_client;//battery adc client
	struct adc_client *dc_adc_client;//DC adc client
	struct adc_client *hole_dc_adc_client;//HOLE DC adc client,wisky-lxh@20110622
	struct timer_list timer;
	struct power_property_info property;
	struct rtc_time suspend_rtc;
	struct rtc_time resume_rtc;
	int sleep_too_long;	//0--system sleep a little time, 1---system sleep too LONG, need to update battery level
#if defined(CHARGE_FULL_CHECK_BY_RTC)
	struct delayed_work delay_work;
	struct rtc_time curr_rtc;
	struct rtc_time trigger_rtc;
	int rtc_record_trigger;	//start to record rtc when battery charge up to 99%
	#define RTC_RECORD_TRIGGER_BAT_LEVEL	99
	#define BATTER_CHARGE_FULL_TIME_MIN	30//minutes, charge full after trigger 40 minutes
#endif

	//battery adc
	int bat_adc_base;//battery adc before system boot
	int bat_suspend_adc_triger;//
		#define SUSPEND_ADC_TRIGGER_CNT	10
	int bat_adc;//battery adc value
	int bat_adc_cnt;//battery adc sample counter
	int bat_adc_sum;//battery adc sample sum value
	//dc adc
	int dc_adc;//DC adc value
	int hole_dc_adc;//hole dc adc value,wisky-lxh@20110620
	int dc_adc_cnt;//DC adc sample counter
	int dc_adc_sum;//DC adc sample sum value

	int bat_adc_min;//接充电器,usb等不同情况下电池ADC值范围最小值
	int bat_adc_max;//接充电器,usb等不同情况下电池ADC值范围最大值
	int bat_adc_range;//电池ADC值范围
	int usb_status;
	int dc_status; 
	int charge_status;//充电状态
	int last_capacity;

	int sample_batt_state;// 1---sample is going, 0---sample is done
		#define SAMPLE_BATT_STOP		0
		#define SAMPLE_BATT_START	1
	int sample_batt_count;//timer counter for battery adc

	int sample_batt_update;// 1---sample is done, should be update
		#define BATT_UPDATE_OFF		0
		#define BATT_UPDATE_ON		1
	unsigned int boot_first;	//for update battery level when system power on first time
		#define BATT_BOOT_ADC_COUNT		20
	unsigned int system_ready;
		#define SYSTEM_READY_COUNT		0//200	//系统启动完成后需要校准电压的次数
		
	int batt_charge_state;// 1---enable charge, 0---disable charge
		#define BATT_CHARGE_DISABLE		0
		#define BATT_CHARGE_ENABLE		1
};

struct battery_percent{
	int voltage_low;
	int voltage_high;
	int capacity;
};


struct wisky_battery_info *battery_info;
int battery_adc_value = 0;
//DC status by ADC detect DC power pin
int dc_adaptor_status = 0;//0---dc removed, 1---dc insert
int battery_capacity = 0;
int external_power_status = 0;
int low_battery_poweroff_cnt = 0;
EXPORT_SYMBOL(external_power_status);

extern int get_msc_connect_flag( void );
extern int usb_otg_op_state;
extern int get_usb_connect_flag(void);

extern int wisky_rtc_read_time(struct rtc_time *tm);

#if defined(USE_BATT_LEVEL_TABLE)
/*
 * state_0::maxcapacity 4
 * state_10::maxlevel 14
 * state_20::maxlevel 29
 * state_40::maxlevel 49
 * state_60::maxlevel 69
 * state_80::maxlevel 89
 * state_100::maxlevel 100
 */
const struct battery_percent battery_percent_table[] = {
    // low, high, capacity
#if 0    
	{ BATT_MIN_ADC_VALUE, 748,  0},     // 3.4v ~ 3.5v
	{ 749, 758,  10},     // 3.5v ~ 3.55v
    	{ 759, 768,  15},     // 3.55v ~ 3.6v
	{ 769, 780,  20},     // 3.6v ~ 3.65v
	{ 781, 790,  25},     // 3.65v ~ 3.7v
	{ 791, 800,  30},     // 3.7v ~ 3.75v
	{ 801, 810,  35},     // 3.75v ~ 3.8v
	{ 811, 820,  40},     // 3.8v ~ 3.85v
	{ 821, 830,  50},     // 3.85v ~ 3.9v
	{ 831, 848,  60},     // 3.9v ~ 4.0v
	{ 849, 868,  80},     // 4.0v ~ 4.1v	
	{ 869, 999,  100},    // 4.1v ~ 4.2v
#else
	{ BATT_MIN_ADC_VALUE, 717,  0},     // 3.5v ~ 3.55v
	{ 718, 734,  5},     // 3.55v ~ 3.6v
	{ 735, 754,  10},     // 3.6v ~ 3.7v
	{ 755, 775,  20},     // 3.7v ~ 3.8v
	{ 776, 795,  40},     // 3.8v ~ 3.9v
	{ 796, 820,  60},     // 3.9v ~ 4.0v
	{ 821, 839,  80},     // 4.0v ~ 4.1v	
	{ 840, 999,  100},    // 4.1v ~ 4.2v
#endif
};
#endif

static int wisky_ac_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val);
/*static int wisky_usb_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val);*/
static int wisky_battery_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val);


static enum power_supply_property wisky_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property wisky_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

/*static enum power_supply_property wisky_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};*/

static struct power_supply wisky_power_supplies[] = {
	{
		.name = "mains",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.properties = wisky_ac_properties,
		.num_properties = ARRAY_SIZE(wisky_ac_properties),
		.get_property = wisky_ac_get_property,
	},
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = wisky_battery_properties,
		.num_properties = ARRAY_SIZE(wisky_battery_properties),
		.get_property = wisky_battery_get_property,
	},
	/*{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.properties = wisky_usb_properties,
		.num_properties = ARRAY_SIZE(wisky_usb_properties),
		.get_property = wisky_usb_get_property,
	},*/
};

//wisky-lxh@20110721,hole dc input irq
static irqreturn_t hole_dc_wakeup_handler(int irq, void *dev_id)
{
	WPRINTK("hole dc input ,wake up---->\n");
	rk28_send_wakeup_key();
	
	return IRQ_HANDLED;
}

//end-wisky-lxh@20110721,hole dc input irq
extern suspend_state_t get_suspend_state(void);

#if defined(WISKY_BATTERY_SMB347)
static const struct i2c_device_id smb347_id[] = {
	{SMB347_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, smb347_id);

//return read bytes number if success, else negative number
static int smb347_read(const char reg, char *buf, int count)
{
	int ret = 0;

	ret = i2c_master_reg8_recv(battery_info->client, reg, buf, count, SMB347_I2C_SCL_RATE);

	return ret;
}

static int smb347_write(const char reg, char *buf, int count)
{
	int ret = 0;

	ret = i2c_master_reg8_send(battery_info->client, reg, buf, count, SMB347_I2C_SCL_RATE);

	return ret;
}

static int smb347_powerup_init(void)
{
	char buf[2];

	//充电状态STAT 检测配置
	smb347_read(SMB347_STAT_TIMER_CTRL, buf, 1);
	buf[0] =  (buf[0] & 0x1F) | STAT_OUTPUT_POLARITY_LOW |STAT_OUTPUT_MODE_CHARGING| STAT_OUTPUT_CTRL_ENABLE;
	pr_info("%s[%d]: SMB347_STAT_TIMER_CTRL = 0x%x\n", __FUNCTION__, __LINE__, buf[0]);
	smb347_write(SMB347_STAT_TIMER_CTRL, buf, 1);

	smb347_read(SMB347_CMD_REGA, buf, 1);
	buf[0] = (buf[0] & 0xEF) | OTG_DISABLE;//OTG disable
	pr_info("%s[%d]: SMB347_CMD_REGA = 0x%x\n", __FUNCTION__, __LINE__, buf[0]);
	smb347_write(SMB347_CMD_REGA, buf, 1);

	//根据OTG ID检测脚控制进入OTG模式并供电,OTG ID 低电平有效
	smb347_read(SMB347_OTHER_CTRLA, buf, 1);
	buf[0] = (buf[0]& 0x1F) | 1<<6 | 1 << 5;//RID Disabled, OTG Pin Control ||OTG Pin Active LOW
//	buf[0] = (buf[0]& 0x3F) | 3<<6;//RID Enabled, Auto-OTG
	pr_info("%s[%d]: SMB347_OTHER_CTRLA = 0x%x\n", __FUNCTION__, __LINE__, buf[0]);
	smb347_write(SMB347_OTHER_CTRLA, buf, 1);
}

//mode: positive number---set to OTG mode, else charge mode
int smb347_otg_mode(int mode)
{	
	char buf[2];
	
	if(mode > 0){
		smb347_read(SMB347_CMD_REGA, buf, 1);
		buf[0] = (buf[0] & 0xEF) | OTG_ENABLE;//OTG enable
		WPRINTK("%s[%d]: SMB347_CMD_REGA = 0x%x\n", __FUNCTION__, __LINE__, buf[0]);
		smb347_write(SMB347_CMD_REGA, buf, 1);
	}else{
		smb347_read(SMB347_CMD_REGA, buf, 1);
		buf[0] = (buf[0] & 0xEF) | OTG_DISABLE;//OTG disable
		WPRINTK("%s[%d]: SMB347_CMD_REGA = 0x%x\n", __FUNCTION__, __LINE__, buf[0]);
		smb347_write(SMB347_CMD_REGA, buf, 1);
	}
}
EXPORT_SYMBOL(smb347_otg_mode);

static int smb347_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	
	WPRINTK("%s[%d]: i2c driver probe enter\n", __FUNCTION__, __LINE__);

	battery_info->client = client;
	i2c_set_clientdata(client, battery_info);

	smb347_powerup_init();
		
	return ret;
}

static int smb347_i2c_remove(struct i2c_client *client)
{
	int ret = 0;

	return ret;
}

static struct i2c_driver smb347_i2c_driver = {
	.probe = smb347_i2c_probe,
	.remove = smb347_i2c_remove,
	.id_table = smb347_id,
	.driver = {
		.name = SMB347_NAME,
	},
};
#endif

#if defined(LOW_BATT_WAKEUP_PIN)
static void lowbat_delay_work_hdl(struct work_struct *work)
{
	struct wisky_battery_info *bat_info = container_of(to_delayed_work(work), struct wisky_battery_info, lowbat_delay_work);

	if(gpio_get_value(battery_info->low_batt_gpio) == GPIO_LOW){
		if(bat_info->low_batt_wakeup_count++ >= 3){
			bat_info->low_batt_wakeup = 1;
		}else{
			queue_delayed_work(battery_info->workqueue, &battery_info->lowbat_delay_work, msecs_to_jiffies(1000));
		}
	}
}

static irqreturn_t low_battery_wakeup_handler(int irq, void *dev_id)
{
	pr_info("low battery detect ,wake up---->\n");
	
	if(!external_power_status){		
		rk28_send_wakeup_key();
		if(gpio_get_value(battery_info->low_batt_gpio) == GPIO_LOW){
			battery_info->low_batt_wakeup_count = 0;
			queue_delayed_work(battery_info->workqueue, &battery_info->lowbat_delay_work, msecs_to_jiffies(1000));
		}
	}
	
	return IRQ_HANDLED;
}

static int low_battery_wakeup_init(int gpio)
{
	int ret = 0;

	if(gpio != INVALID_GPIO){
		battery_info->low_batt_gpio = gpio;
		if(gpio_request(gpio, "low battery wakeup") != 0){
			gpio_free(gpio);
			pr_info("Low battery wake up gpio request failed!");
		}else{
			battery_info->low_batt_irq = gpio_to_irq(gpio);
			ret = request_irq(battery_info->low_batt_irq, low_battery_wakeup_handler, IRQF_TRIGGER_FALLING, "low battery detect", NULL);
			if (ret < 0) {
				pr_err("%s: request_irq(%d) failed\n", __func__, battery_info->low_batt_irq);
				//gpio_free(gpio);
				//return ret;
			}
			enable_irq_wake(battery_info->low_batt_irq);	
			disable_irq(battery_info->low_batt_irq);
		}
	}
	battery_info->low_batt_wakeup = 0;
	battery_info->low_batt_wakeup_count = 0;

	return ret;
}
#endif

/*
 *detect DC insert, return 1, else return 0
 * DC_DETECT_PIN: USB充电器检测脚
 * HOLE_DC_DETECT_PIN:圆孔充电器检测脚
 */
static int get_dc_status(struct wisky_battery_info *bat_info)
{
	unsigned int status = 0;

	if(DC_DETECT_PIN != INVALID_GPIO){
	 	if(gpio_get_value(DC_DETECT_PIN) == DC_DETECT_VALUE){
		 	if(1 == usb_otg_op_state){
				status &= ~DC_INSERT;
				WPRINTK("DC(otg) remove%d\n", __LINE__);
			}else{
				WPRINTK("DC insert\n");
				status |= DC_INSERT;
			}
	 	}
		else 
		{
		 	status &= (~DC_INSERT);
			WPRINTK("DC remove\n");
		}
	}/*else{
	 	if(bat_info->dc_adc >= AC_INSERT_VALUE){
			if(1 == usb_otg_op_state){
				status &= (~DC_INSERT);
				WPRINTK("DC remove-%d\n", __LINE__);
			}else{
				status |= DC_INSERT;
				WPRINTK("dc insert\n");
			}
	 	}
		else {
		 	status &= (~DC_INSERT);
			WPRINTK("DC remove-%d\n", __LINE__);
		}
	}*/
#if defined(HOLE_DC_DETECT_PIN)
	//wisky-lxh@20110618,hole dc detect
	if (HOLE_DC_DETECT_PIN != INVALID_GPIO)
	{
		if(gpio_get_value(HOLE_DC_DETECT_PIN) == HOLE_DC_DETECT_VALUE)
		{
			status |= HOLE_DC_INSERT;
			WPRINTK("hole dc in\n");
		}
		else
		{
			status &= (~HOLE_DC_INSERT);
			WPRINTK("hole dc remove\n");
		}
		
	}
#endif
	
	if(dc_adaptor_status != status){//reset adc sample when dc/usb status change
		bat_info->sample_batt_count = 0;
		bat_info->sample_batt_state = SAMPLE_BATT_STOP;
		bat_info->bat_adc_cnt = 0;
		bat_info->bat_adc_sum = 0;
	}
	
	dc_adaptor_status = status;
	
	return status;
}

//usb 口DC检测
//return: 1--usb dc insert, 0--usb dc remove
static int get_usb_dc_status(void)
{
	int level;
	
	if(USB_DETECT_PIN != INVALID_GPIO){
		level = gpio_get_value(USB_DETECT_PIN);
		return (USB_DETECT_VALUE == level);
	}else{
		return 0;
	}
}

	
static void battery_charge_enable_set(struct wisky_battery_info *batt_info, int enable)
{
	if(enable == BATT_CHARGE_ENABLE){
#if defined(BATTERY_CHARGE_PWR_EN_PIN)
		if(BATTERY_CHARGE_PWR_EN_PIN != INVALID_GPIO){
			gpio_set_value(BATTERY_CHARGE_PWR_EN_PIN, BATTERY_CHARGE_PWR_EN_VALUE);
		}
#endif
		if(BATTERY_CHARGE_EN_PIN != INVALID_GPIO){
			WPRINTK("------>Enable battery charge.\n");
			gpio_set_value(BATTERY_CHARGE_EN_PIN, BATTERY_CHARGE_EN_VALUE);//enable battery charge
		}
		batt_info->batt_charge_state = BATT_CHARGE_ENABLE;
	}else{
		batt_info->batt_charge_state = BATT_CHARGE_DISABLE;
#if defined(BATTERY_CHARGE_PWR_EN_PIN)
		if(BATTERY_CHARGE_PWR_EN_PIN != INVALID_GPIO){
			gpio_set_value(BATTERY_CHARGE_PWR_EN_PIN, !BATTERY_CHARGE_PWR_EN_VALUE);
		}
#endif
		if(BATTERY_CHARGE_EN_PIN != INVALID_GPIO){
			WPRINTK("------>Disable battery charge.\n");
			gpio_set_value(BATTERY_CHARGE_EN_PIN, !BATTERY_CHARGE_EN_VALUE);//disable battery charge
		}	
	}
}
//wisky-lxh@20110519,圆孔充电器无需延时
#if defined(WISKY_DC_USB_CHARGE_2IN1)
#define THREAD_START_TIME			3500//6000 jiffies(1 minute) after driver init, run thread
#else
#define THREAD_START_TIME			6//6000 jiffies(1 minute) after driver init, run thread
#endif
//wisky-lxh@20110519
#define THREAD_SLEEP_TIME_MS		200	//ms
#define DC_ADC_SAMPLE_MAX		10	//200ms x 10 = 2000ms

int bat_charge_level = 0;
unsigned int dc_adc_value = 0;//DC adaptor voltage ADC value
unsigned int sample_time = 0;//DC ADC sample time counter
unsigned int min_dc_voltage_cnt = 0;
unsigned int middle_dc_voltage_cnt = 0;
unsigned int max_dc_voltage_cnt = 0;
int min_charge_flag = 0;
int middle_charge_flag = 0;
int max_charge_flag = 0;
int charge_voltage_down = 0;//set to 1 if DC adaptor voltage decrease

unsigned long thread_run_time = 0;//use for thread run after system boot complete

//extern unsigned int usb_unmount;

#if defined(CHARGE_CURRENT_LEVEL_3)
//battery charge current has 3 level, low, middle, and high
#define BAT_CHARGE_LEVEL_DISABLE	0	//turn off charge
#define BAT_CHARGE_LEVEL_MIN 		1	//about 200mA
#define BAT_CHARGE_LEVEL_MIDDLE 	2	//about 600mA
#define BAT_CHARGE_LEVEL_MAX 		3	//about 1200mA

#define DC_ADC_VALUE_4DV		740
#define DC_ADC_VALUE_4D4V		780
#define DC_ADC_VALUE_4D6V		800
#define DC_ADC_VALUE_4D7V		818

static void set_bat_charge_current_mode(struct wisky_battery_info *batt_info, int mode)
{
	switch(mode){
		case BAT_CHARGE_LEVEL_MIN:
			battery_charge_enable_set(batt_info, BATT_CHARGE_ENABLE);
			if(CHARGE_CURRENT_CTL1_PIN != INVALID_GPIO){
				gpio_set_value(CHARGE_CURRENT_CTL1_PIN, !CHARGE_CURRENT_CTL1_ON_VALUE);
			}
			if(CHARGE_CURRENT_CTL2_PIN != INVALID_GPIO){
				gpio_set_value(CHARGE_CURRENT_CTL2_PIN, !CHARGE_CURRENT_CTL2_ON_VALUE);
			}
			break;
		case BAT_CHARGE_LEVEL_MIDDLE:
			battery_charge_enable_set(batt_info, BATT_CHARGE_ENABLE);
			if(CHARGE_CURRENT_CTL1_PIN != INVALID_GPIO){
				gpio_set_value(CHARGE_CURRENT_CTL1_PIN, CHARGE_CURRENT_CTL1_ON_VALUE);
			}
			if(CHARGE_CURRENT_CTL2_PIN != INVALID_GPIO){
				gpio_set_value(CHARGE_CURRENT_CTL2_PIN, !CHARGE_CURRENT_CTL2_ON_VALUE);
			}
			break;
		case BAT_CHARGE_LEVEL_MAX:
			battery_charge_enable_set(batt_info, BATT_CHARGE_ENABLE);
			if(CHARGE_CURRENT_CTL1_PIN != INVALID_GPIO){
				gpio_set_value(CHARGE_CURRENT_CTL1_PIN, CHARGE_CURRENT_CTL1_ON_VALUE);
			}
			if(CHARGE_CURRENT_CTL2_PIN != INVALID_GPIO){
				gpio_set_value(CHARGE_CURRENT_CTL2_PIN, CHARGE_CURRENT_CTL2_ON_VALUE);
			}
			break;
		case BAT_CHARGE_LEVEL_DISABLE:
		default:
			battery_charge_enable_set(batt_info, BATT_CHARGE_DISABLE);
			if(CHARGE_CURRENT_CTL1_PIN != INVALID_GPIO){
				gpio_set_value(CHARGE_CURRENT_CTL1_PIN, !CHARGE_CURRENT_CTL1_ON_VALUE);
			}
			if(CHARGE_CURRENT_CTL2_PIN != INVALID_GPIO){
				gpio_set_value(CHARGE_CURRENT_CTL2_PIN, !CHARGE_CURRENT_CTL2_ON_VALUE);
			}
			break;
	}

	bat_charge_level = mode;		
}


static pid_t bat_charge_thread(void *arg)
{
	struct task_struct *task = current;
	int usb_connect_status = 0, dc_connect_status = 0;
	int next_charge_level = BAT_CHARGE_LEVEL_MAX;
	int dc_insert_cnt = 0;//
	int dc_remove_cnt = 0;
	
	daemonize("BatChargeThread");
	allow_signal(SIGKILL);

	while(!signal_pending(task)){
		if(time_before(jiffies, thread_run_time)){
			set_bat_charge_current_mode(battery_info, BAT_CHARGE_LEVEL_DISABLE);
			//before system run 1 minute
			goto THREAD_SLEEP;
		}
		dc_adc_value = battery_info->dc_adc;
		WPRINTK("[3] charge current level thread!\n");
		WPRINTK("---->: dc = %d, bat = %d, charge_level = %d\n", dc_adc_value, battery_info->bat_adc, bat_charge_level);

		usb_connect_status = get_usb_connect_flag();
		dc_connect_status = get_dc_status(battery_info);

		//if(!strcmp(ANDROID_CONFIG_XM, "1")){
			if(usb_connect_status || dc_connect_status){
				if(false == battery_info->wakelock_state){
					wake_lock(&battery_info->wakelock);
					battery_info->wakelock_state = true;
				}
			}else{
				if(true == battery_info->wakelock_state){
					wake_unlock(&battery_info->wakelock);
					battery_info->wakelock_state = false;
				}
			}
		//}
		
		if (usb_connect_status == USB_PLUGIN){//usb in
			dc_remove_cnt = 0;
			WPRINTK("----->: <USB_PLUGIN>\n");
			if(1 == battery_info->early_suspend_state){
				//system suspend, enable battery charge
				set_bat_charge_current_mode(battery_info, BAT_CHARGE_LEVEL_MIN);//USB插入，总是以【最小电流】档进行充电	
			}else{//system wake up, disable battery charge
				set_bat_charge_current_mode(battery_info, BAT_CHARGE_LEVEL_DISABLE);
			}
		}else if(dc_connect_status){
			dc_remove_cnt = 0;
			if(0 == dc_insert_cnt){//插入充电器
				set_bat_charge_current_mode(battery_info, BAT_CHARGE_LEVEL_MAX);
			}else{
				if(1 == battery_info->early_suspend_state){
					set_bat_charge_current_mode(battery_info, 
						next_charge_level<BAT_CHARGE_LEVEL_MAX?(next_charge_level+1):next_charge_level);
				}else{
					set_bat_charge_current_mode(battery_info, next_charge_level);
				}
			}
			
			dc_insert_cnt++;
		}else{
			if(0 == dc_remove_cnt){
				if(BAT_CHARGE_LEVEL_MAX == bat_charge_level){
					next_charge_level = BAT_CHARGE_LEVEL_MIDDLE;
				}else if(BAT_CHARGE_LEVEL_MIDDLE == bat_charge_level){
					next_charge_level = BAT_CHARGE_LEVEL_MIN;
				}
			}
			
			if(dc_remove_cnt++ > 10){//200ms * 15 = 3 s
				dc_insert_cnt = 0;//检测超过2秒无DC，则清除DC状态计数
				next_charge_level = BAT_CHARGE_LEVEL_MAX;
			}
			set_bat_charge_current_mode(battery_info, BAT_CHARGE_LEVEL_DISABLE);
		}

THREAD_SLEEP:
		msleep(THREAD_SLEEP_TIME_MS);
	}
	return 0;
}
#elif defined(CHARGE_CURRENT_LEVEL_2)
//battery charge current has 2 level, low, and high
#define BAT_CHARGE_LEVEL_DISABLE	0	//turn off charge
#define BAT_CHARGE_LEVEL_MIN 		1	//about 400mA
#define BAT_CHARGE_LEVEL_MAX 		2	//about 1000mA

#define DC_ADC_VALUE_4DV		740
#define DC_ADC_VALUE_4D4V		780
#define DC_ADC_VALUE_4D6V		800
#define DC_ADC_VALUE_4D7V		818
static void set_bat_charge_current_mode(struct wisky_battery_info *batt_info, int mode)
{
	switch(mode){
		case BAT_CHARGE_LEVEL_MIN:
			battery_charge_enable_set(batt_info, BATT_CHARGE_ENABLE);
			if(CHARGE_CURRENT_CTL1_PIN != INVALID_GPIO){
				gpio_set_value(CHARGE_CURRENT_CTL1_PIN, !CHARGE_CURRENT_CTL1_ON_VALUE);
			}
			if(CHARGE_CURRENT_CTL2_PIN != INVALID_GPIO){
				gpio_set_value(CHARGE_CURRENT_CTL2_PIN, !CHARGE_CURRENT_CTL2_ON_VALUE);
			}
			break;
		case BAT_CHARGE_LEVEL_MAX:
			battery_charge_enable_set(batt_info, BATT_CHARGE_ENABLE);
			if(CHARGE_CURRENT_CTL1_PIN != INVALID_GPIO){
				gpio_set_value(CHARGE_CURRENT_CTL1_PIN, CHARGE_CURRENT_CTL1_ON_VALUE);
			}
			if(CHARGE_CURRENT_CTL2_PIN != INVALID_GPIO){
				gpio_set_value(CHARGE_CURRENT_CTL2_PIN, CHARGE_CURRENT_CTL2_ON_VALUE);
			}
			break;
		case BAT_CHARGE_LEVEL_DISABLE:
		default:
			battery_charge_enable_set(batt_info, BATT_CHARGE_DISABLE);
			if(CHARGE_CURRENT_CTL1_PIN != INVALID_GPIO){
				gpio_set_value(CHARGE_CURRENT_CTL1_PIN, !CHARGE_CURRENT_CTL1_ON_VALUE);
			}
			if(CHARGE_CURRENT_CTL2_PIN != INVALID_GPIO){
				gpio_set_value(CHARGE_CURRENT_CTL2_PIN, !CHARGE_CURRENT_CTL2_ON_VALUE);
			}
			break;
	}

	bat_charge_level = mode;
}
extern void set_backlight_usb_in(void);
extern void set_backlight_usb_out(void);

static pid_t bat_charge_thread(void *arg)
{
	struct task_struct *task = current;
	int usb_connect_status = 0, dc_connect_status = 0;
	int next_charge_level = BAT_CHARGE_LEVEL_MAX;
	int dc_insert_cnt = 0;//
	int dc_remove_cnt = 0;
	
	daemonize("BatChargeThread");
	allow_signal(SIGKILL);

	while(!signal_pending(task)){
		if(time_before(jiffies, thread_run_time)){
			set_bat_charge_current_mode(battery_info, BAT_CHARGE_LEVEL_DISABLE);
			//before system run 1 minute
			goto THREAD_SLEEP;
		}
		dc_adc_value = battery_info->dc_adc;
		WPRINTK("[3] charge current level thread!\n");
		WPRINTK("---->: dc = %d, bat = %d, charge_level = %d\n", dc_adc_value, battery_info->bat_adc, bat_charge_level);

		usb_connect_status = get_usb_connect_flag();
		dc_connect_status = get_dc_status(battery_info);

		//if(!strcmp(ANDROID_CONFIG_XM, "1")){
			if(usb_connect_status || dc_connect_status){
				if(false == battery_info->wakelock_state){
					wake_lock(&battery_info->wakelock);
					battery_info->wakelock_state = true;
				}
			}else{
				if(true == battery_info->wakelock_state){
					wake_unlock(&battery_info->wakelock);
					battery_info->wakelock_state = false;
				}
			}
		//}
		
		if (usb_connect_status == USB_PLUGIN){//usb in
			dc_remove_cnt = 0;
			WPRINTK("----->: <USB_PLUGIN>\n");
			if(1 == battery_info->early_suspend_state){
				//system suspend, enable battery charge
				set_bat_charge_current_mode(battery_info, BAT_CHARGE_LEVEL_MIN);//USB插入，总是以【最小电流】档进行充电	
			}else{//system wake up, disable battery charge
				set_bat_charge_current_mode(battery_info, BAT_CHARGE_LEVEL_DISABLE);
			}
		}else if(dc_connect_status){
			dc_remove_cnt = 0;
			if(0 == dc_insert_cnt){//插入充电器
				set_bat_charge_current_mode(battery_info, BAT_CHARGE_LEVEL_MAX);
			}else{
				if(1 == battery_info->early_suspend_state){
					set_bat_charge_current_mode(battery_info, 
						next_charge_level<BAT_CHARGE_LEVEL_MAX?(next_charge_level+1):next_charge_level);
				}else{
					set_bat_charge_current_mode(battery_info, next_charge_level);
				}
			}
			
			dc_insert_cnt++;
		}else{
			if(0 == dc_remove_cnt){
				if(BAT_CHARGE_LEVEL_MAX == bat_charge_level){
					next_charge_level = BAT_CHARGE_LEVEL_MIN;
				}
			}
			
			if(dc_remove_cnt++ > 10){//200ms * 15 = 3 s
				dc_insert_cnt = 0;//检测超过2秒无DC，则清除DC状态计数
				next_charge_level = BAT_CHARGE_LEVEL_MAX;
			}
			set_bat_charge_current_mode(battery_info, BAT_CHARGE_LEVEL_DISABLE);
		}

THREAD_SLEEP:
		msleep(THREAD_SLEEP_TIME_MS);
	}
	return 0;
}
#endif

//set charge LED On/Off
static void set_charge_led(unsigned int on)
{
	if(!strcmp(ANDROID_CONFIG_Z4, "1")){
		wisky_touchkey_led_set(on);
	}
}

static void flash_charging_led(struct wisky_battery_info *bat_info)
{
	static int charge_led_on = 0;
	static int charge_led_cnt = 0;
	
	if(POWER_SUPPLY_STATUS_CHARGING == bat_info->property.status){
		if((!strcmp(ANDROID_CONFIG_Z4, "1")) && (bat_charge_level != BAT_CHARGE_LEVEL_DISABLE)
			&& (1 == bat_info->early_suspend_state)){
			charge_led_cnt++;
			if(7 == charge_led_cnt){
				charge_led_on = 1;
				set_charge_led(charge_led_on);
			}else if(charge_led_cnt > 10){
				charge_led_cnt = 0;
				charge_led_on = 0;
				set_charge_led(charge_led_on);
			}
		}else{
			charge_led_on = 0;
			set_charge_led(charge_led_on);
		}
	}else{
		if(charge_led_on){
			charge_led_on = 0;
			set_charge_led(charge_led_on);
		}
	}
}


//STAT2(PIN1): Low--Charging, HIGH---Charge FULL or No charge
//return 1 if charge full, 0 if charging, else return negative if error
static int get_charge_status(void)
{
	int state1=-1, state2=-1;
	int charge_status = 0;
	
#if defined(BATTERY_CHARGE_STAT1_PIN) && defined(BATTERY_CHARGE_STAT2_PIN)
	if(BATTERY_CHARGE_STAT1_PIN != INVALID_GPIO && BATTERY_CHARGE_STAT2_PIN != INVALID_GPIO){
	//使用两个状态监测 脚STAT1 & STAT2
		state1 = gpio_get_value(BATTERY_CHARGE_STAT1_PIN);
		state2 = gpio_get_value(BATTERY_CHARGE_STAT2_PIN);
		if(GPIO_HIGH == state1 && GPIO_HIGH == state2){
			charge_status = POWER_SUPPLY_STATUS_DISCHARGING;//POWER_SUPPLY_STATUS_NOT_CHARGING;
		}else if(GPIO_HIGH == state1 && GPIO_LOW == state2){
			charge_status = POWER_SUPPLY_STATUS_CHARGING;
		}else if(GPIO_LOW == state1 && GPIO_HIGH == state2){
			charge_status = POWER_SUPPLY_STATUS_FULL;
		}
		WPRINTK("Read BAT Charge IC-->state1=%d, state2=%d\n", state1, state2);
		WPRINTK("BAT Charge IC-->charge_status=%d\n", charge_status);
	}else if(BATTERY_CHARGE_STAT2_PIN != INVALID_GPIO){
	//只使用一个状态监测脚 STAT2
		state2 = gpio_get_value(BATTERY_CHARGE_STAT2_PIN);
		if(GPIO_LOW == state2){
			//STAT2 low, charging
			charge_status = POWER_SUPPLY_STATUS_CHARGING;
		}else{
			//STAT2 high, charge full or not charge
			charge_status = POWER_SUPPLY_STATUS_FULL;
		}
		WPRINTK("Read BAT Charge IC-->state2=%d\n", state2);
		WPRINTK("BAT Charge IC-->charge_status=%d\n", charge_status);
	}
#elif defined(BATTERY_CHARGE_STAT2_PIN)
	 if(BATTERY_CHARGE_STAT2_PIN != INVALID_GPIO){
	//只使用一个状态监测脚 STAT2
		state2 = gpio_get_value(BATTERY_CHARGE_STAT2_PIN);
		if(GPIO_LOW == state2){
			//STAT2 low, charging
			charge_status = POWER_SUPPLY_STATUS_CHARGING;
		}else{
			//STAT2 high, charge full or not charge
			charge_status = POWER_SUPPLY_STATUS_FULL;
		}
		WPRINTK("Read BAT Charge IC-->state2=%d\n", state2);
		WPRINTK("BAT Charge IC-->charge_status=%d\n", charge_status);
	}
#endif
	return charge_status;
}

#if defined(CHARGE_FULL_CHECK_BY_RTC)
/*delay minute by recored RTC time
 * 一小时以内的时间延时
 *return: return 1 if delay miuntes time is up, else return 0.
 */
static int rtc_delay_minute(struct rtc_time *old_rtc, struct rtc_time *new_rtc, int minute)
{
//	WPRINTK("%s[%d]---------------new_rtc:year[%d] mon[%d] hour[%d] min[%d]\n", __FUNCTION__, __LINE__, new_rtc->tm_year, new_rtc->tm_mon, new_rtc->tm_hour, new_rtc->tm_min);
//	WPRINTK("%s[%d]---------------old_rtc:year[%d] mon[%d] hour[%d] min[%d]\n", __FUNCTION__, __LINE__, old_rtc->tm_year, old_rtc->tm_mon, old_rtc->tm_hour, old_rtc->tm_min);	
	if(new_rtc->tm_year != old_rtc->tm_year || new_rtc->tm_mon != old_rtc->tm_mon){
		return 1;
	}

	if(new_rtc->tm_hour == old_rtc->tm_hour){
		if(new_rtc->tm_min > old_rtc->tm_min){
			if((new_rtc->tm_min - old_rtc->tm_min) >= minute){
				return 1;
			}
		}else if(new_rtc->tm_min < old_rtc->tm_min){
			return 1;
		}
	}
	
	if(new_rtc->tm_hour != old_rtc->tm_hour){
		if((new_rtc->tm_min + 60 - old_rtc->tm_min) >= minute){
			return 1;
		}
	}

	return 0;
}
#endif

/*delay minute by recored RTC time
 *return: return 1 if delay hour time is up, else return 0.
 */
static int rtc_delay_hour(struct rtc_time *old_rtc, struct rtc_time *new_rtc, int hour)
{
	if(new_rtc->tm_hour > old_rtc->tm_hour){
		if((new_rtc->tm_hour - old_rtc->tm_hour) >= hour){
			return 1;
		}
	}else if(new_rtc->tm_hour < old_rtc->tm_hour){
		return 1;
	}

	return 0;
}

static void battery_adc_filter(struct wisky_battery_info *bat_info)
{
	int i;
	
	for(i = 0; i < BAT_ADC_SIZE; i++){
		if((BAT_ADC_SIZE-1) == i){
			bat_adc_filter.adc[i] = bat_info->bat_adc;
		}else{
			bat_adc_filter.adc[i] = bat_adc_filter.adc[i+1];
		}
	}

	bat_adc_filter.count++;
	bat_adc_filter.adc_temp = 0;

	for(i = 0; i < BAT_ADC_SIZE; i++){
		bat_adc_filter.adc_temp += bat_adc_filter.adc[i];
	}

	if(bat_adc_filter.count > BAT_ADC_SIZE){//计数到达BAT_ADC_SIZE
		bat_info->bat_adc = bat_adc_filter.adc_temp/BAT_ADC_SIZE;
	}else{//计数小于BAT_ADC_SIZE
		bat_info->bat_adc = bat_adc_filter.adc_temp/bat_adc_filter.count;
	}
}

/*
 *return 1 if battery charge full, else return 0
 */
static int is_battery_full(struct wisky_battery_info *bat_info)
{
	int batt_full = 0;

	if((BATT_CHARGE_ENABLE == bat_info->batt_charge_state) && get_charge_status() == POWER_SUPPLY_STATUS_FULL){
		batt_full = 1;
	}
#if defined(CHARGE_FULL_CHECK_BY_RTC)
	if(rtc_delay_minute(&bat_info->trigger_rtc, &bat_info->curr_rtc, BATTER_CHARGE_FULL_TIME_MIN)){
		batt_full = 1;
	}
	//queue work to updata rtc time
	queue_delayed_work(bat_info->workqueue, &bat_info->delay_work, msecs_to_jiffies(30000));
#endif	
	return batt_full;
}

static int wisky_ac_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	if(NULL == battery_info){
		return -EFAULT;
	}
	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = (external_power_status>0)?1:0;
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

/*static int wisky_usb_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	if(NULL == battery_info){
		return -EFAULT;
	}
	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = battery_info->dc_status;
			break;
		default:		
			return -EINVAL;
	}

	return 0;
}*/

static int wisky_battery_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	if(NULL == battery_info){
		return -EFAULT;
	}
	switch (psp) {
		/*case POWER_SUPPLY_PROP_ONLINE:
			val->intval = battery_info->dc_status;
			break;*/
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = battery_info->property.status;
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = battery_info->property.health;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = battery_info->property.present;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = battery_info->property.capacity;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val ->intval = battery_info->property.voltage_now*1000;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX:
			val->intval = battery_info->bat_adc_max;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MIN:
			val->intval = battery_info->bat_adc_min;
			break;
		default:
			return -EINVAL;
	}

	return 0;
}


static void update_battery_capacity(struct wisky_battery_info *bat_info)
{
	int bat_adc, bat_temp, bat_base;
	static int not_charge_count = 0;
	static int charge_full_cnt = 0;
	static int charge_full_led_on = 0;

	bat_base = bat_info->bat_adc_base/100;
	/*if(external_power_status){
		if(POWER_SUPPLY_STATUS_CHARGING == bat_info->charge_status){
			bat_info->bat_adc_min = BATT_MIN_CHARGE_ADC_VALUE;
			bat_info->bat_adc_max = BATT_FULL_CHARGE_ADC_VALUE;
			bat_info->bat_adc_range = (bat_info->bat_adc_max - bat_info->bat_adc_min);
		}else{
			bat_info->bat_adc_min = BATT_MIN_DCIN_NOCHARGE_ADC;
			bat_info->bat_adc_max = BATT_FULL_DCIN_NOCHARGE_ADC;
			bat_info->bat_adc_range = (bat_info->bat_adc_max - bat_info->bat_adc_min);
		}
	}else*/{
		bat_info->bat_adc_min = BATT_MIN_ADC_VALUE;
		bat_info->bat_adc_max = BATT_FULL_ADC_VALUE;
		bat_info->bat_adc_range = (bat_info->bat_adc_max - bat_info->bat_adc_min);
	}

/*	capacity range:	 from 0 to 99
 *	battery adc range: from BATT_MIN_ADC_VALUE to BATT_FULL_ADC_VALUE
 *	capacity_now = (99*(bat_adc_now - -BATT_MIN_ADC_VALUE))/(BATT_FULL_ADC_VALUE-BATT_MIN_ADC_VALUE)
 */
 	if(bat_base > bat_info->bat_adc_max){
		bat_adc = bat_info->bat_adc_max;
	}else if(bat_base  < bat_info->bat_adc_min){
		bat_adc = bat_info->bat_adc_min;
	}else{
		bat_adc = bat_base;
	}

	bat_temp = (bat_adc-bat_info->bat_adc_min)*99;
	bat_info->property.capacity = bat_temp/(bat_info->bat_adc_range);

	if(external_power_status && (bat_info->property.capacity <= 0)){
		bat_info->property.capacity = 1;
	}

	#if defined(BAT_LOW_POWEROFF_ADC)
	if(!external_power_status && (bat_info->property.capacity < 15)){
		if(bat_info->sample_batt_count >= SAMPLE_COUNT_DISCHARGE){
			if(bat_info->bat_adc < BAT_LOW_POWEROFF_ADC){
				if(++low_battery_poweroff_cnt >= 3){//连续3次检测第一3.4V，则提示关机
					pr_info("%s[%d]: battery low check 3 times, system power off\n", __FUNCTION__, __LINE__);
					bat_info->property.capacity = 0;
				}
			}else{
				low_battery_poweroff_cnt = 0;
			}
		}
	}
	#endif

	//pr_info("%s[%d]: property.capacity = %d\n", __FUNCTION__, __LINE__, bat_info->property.capacity);
}

static void battery_adc_callback(struct adc_client *client, void *client_param, int result)
{
	struct wisky_battery_info *bat_info = (struct wisky_battery_info *)client_param;
	int adjuct_value = 0;

	bat_info->bat_adc_sum += result;
	if(++bat_info->bat_adc_cnt == ADC_SAMPLE_RATE){
		bat_info->bat_adc = bat_info->bat_adc_sum/ADC_SAMPLE_RATE;
		
		WPRINTK("%s[%d]: ----->New battery raw ADC = (%d)\n", __FUNCTION__, __LINE__, bat_info->bat_adc);
		
		//battery_adc_filter(bat_info);
		battery_adc_value = bat_info->bat_adc;
		if(0 == bat_info->bat_adc_base){
			if(get_usb_dc_status() || get_dc_status(bat_info)){				
				bat_info->bat_adc_base = ((bat_info->bat_adc-7)>BAT_DISCHARGE_ADC_MAX?BAT_DISCHARGE_ADC_MAX:(bat_info->bat_adc-7))*100;
				WPRINTK("%s[%d]: ----->init DC Insert (%d)\n", __FUNCTION__, __LINE__, bat_info->bat_adc_base);
			}else{
				bat_info->bat_adc_base = (bat_info->bat_adc>BAT_DISCHARGE_ADC_MAX?(BAT_DISCHARGE_ADC_MAX+3):bat_info->bat_adc)*100;
				WPRINTK("%s[%d]: ----->init DC Remove (%d)\n", __FUNCTION__, __LINE__, bat_info->bat_adc_base);
			}
			goto exit;
		}

		if(external_power_status){
			WPRINTK("%s[%d]----Charging---battery @=%d, bat_charge_level=%d\n", __FUNCTION__, __LINE__, bat_info->bat_adc - bat_info->bat_adc_base/100, bat_charge_level);
			#if defined(WISKY_BOARD_M101B_TV10)||defined(WISKY_BOARD_M101B_V10)//4400 mAh
			if(BAT_CHARGE_LEVEL_DISABLE == bat_charge_level){
				adjuct_value = 0;
			}else if(BAT_CHARGE_LEVEL_MIN == bat_charge_level){				
				adjuct_value = 30;//15;
			#if defined(BAT_CHARGE_LEVEL_MIDDLE)
			}else if(BAT_CHARGE_LEVEL_MIDDLE== bat_charge_level){
				adjuct_value = 55;//40;
			#endif
			}else if(BAT_CHARGE_LEVEL_MAX == bat_charge_level){
				adjuct_value = 70;//50;
			}
			#else
			if(BAT_CHARGE_LEVEL_DISABLE == bat_charge_level){
				adjuct_value = 0;
			}else if(BAT_CHARGE_LEVEL_MIN == bat_charge_level){				
				adjuct_value = 15;
			#if defined(BAT_CHARGE_LEVEL_MIDDLE)
			}else if(BAT_CHARGE_LEVEL_MIDDLE== bat_charge_level){
				adjuct_value = 40;
			#endif
			}else if(BAT_CHARGE_LEVEL_MAX == bat_charge_level){
				adjuct_value = 50;
			}
			#endif
			bat_info->bat_adc_base = (bat_info->bat_adc_base/100 > BAT_DISCHARGE_ADC_MAX)?bat_info->bat_adc_base:bat_info->bat_adc_base+adjuct_value;
		}else{
			WPRINTK("%s[%d]----Discharging---battery @=%d\n", __FUNCTION__, __LINE__, bat_info->bat_adc_base/100-bat_info->bat_adc );
			if(bat_info->early_suspend_state/* && bat_info->bat_suspend_adc_triger == SUSPEND_ADC_TRIGGER_CNT*/){//休眠唤醒后，重新修正基准电压ADC值
				if(bat_info->bat_suspend_adc_triger == SUSPEND_ADC_TRIGGER_CNT){
					if(bat_info->bat_adc - (bat_info->bat_adc_base/100) > 3){
						bat_info->bat_adc_base = bat_info->bat_adc_base+3;
					}else if((bat_info->bat_adc_base/100) - bat_info->bat_adc > 3){
						bat_info->bat_adc_base = bat_info->bat_adc_base-3;
					}else{
						//bat_info->bat_adc_base = bat_info->bat_adc*100;
					}
				}else{
					
				}
				goto exit;
			}

			#if defined(WISKY_BOARD_M101B_TV10)||defined(WISKY_BOARD_M101B_V10)//4400 mAh
			if(bat_info->bat_adc_base/100 -bat_info->bat_adc < 20){
				bat_info->bat_adc_base -= 70;//75;//60;//50;//35;//25;
			}else if(bat_info->bat_adc_base/100 -bat_info->bat_adc < 40){//亮屏不操作
				bat_info->bat_adc_base -= 75;//80;//63;//53;//38;//40;
			}else if(bat_info->bat_adc_base/100 -bat_info->bat_adc < 60){//播放一般视频
				bat_info->bat_adc_base -= 80;//85;//71;//61;//46;//50;
			}else if(bat_info->bat_adc_base/100 -bat_info->bat_adc < 80){//WiFi打开，不浏览网页
				bat_info->bat_adc_base -= 85;//90;//81;//71;//56;//60;
			}else if(bat_info->bat_adc_base/100 -bat_info->bat_adc < 100){//播放网络视频
				bat_info->bat_adc_base -= 90;//95;//81;//66;//70;
			}else if(bat_info->bat_adc_base/100 > bat_info->bat_adc){//其他较耗电情况下
				bat_info->bat_adc_base -= 100;//95;//80;
			}
			#else//M8686R 3900 mAh
			if(bat_info->bat_adc_base/100 -bat_info->bat_adc < 20){
				bat_info->bat_adc_base -= 30;//35;//25;
			}else if(bat_info->bat_adc_base/100 -bat_info->bat_adc < 40){//亮屏不操作
				bat_info->bat_adc_base -= 45;//53;//40;
			}else if(bat_info->bat_adc_base/100 -bat_info->bat_adc < 60){//播放一般视频
				bat_info->bat_adc_base -= 55;//61;//50;
			}else if(bat_info->bat_adc_base/100 -bat_info->bat_adc < 80){//WiFi打开，不浏览网页
				bat_info->bat_adc_base -= 65;//71;//60;
			}else if(bat_info->bat_adc_base/100 -bat_info->bat_adc < 100){//播放网络视频
				bat_info->bat_adc_base -= 75;//81;//70;
			}else if(bat_info->bat_adc_base/100 > bat_info->bat_adc){//其他较耗电情况下
				bat_info->bat_adc_base -= 95;//80;
			}
			#endif
		}

exit:
		WPRINTK("%s[%d]-------bat_adc_base=%d\n", __FUNCTION__, __LINE__, bat_info->bat_adc_base);
		update_battery_capacity(bat_info);		
		WPRINTK("%s[%d]-------base capacity=%d\n", __FUNCTION__, __LINE__, bat_info->property.capacity);

		bat_info->bat_adc_cnt = 0;
		bat_info->bat_adc_sum = 0;

		bat_info->sample_batt_state = SAMPLE_BATT_STOP;
		bat_info->sample_batt_count = 0;
		bat_info->sample_batt_update = BATT_UPDATE_ON;
	}
}

//wisky-lxh@20110620,add hole dc adc
static void hole_dc_adc_callback(struct adc_client *client, void *client_param, int result)
{
	struct wisky_battery_info *bat_info = (struct wisky_battery_info *)client_param;

	
	//WPRINTK("%s\n",__FUNCTION__);
#if 0
	if(bat_info->dc_adc_cnt >= ADC_SAMPLE_RATE){
		bat_info->dc_adc = bat_info->dc_adc_sum/ADC_SAMPLE_RATE;
		bat_info->dc_adc_cnt = 0;
		bat_info->dc_adc_sum = 0;
	}else{
		bat_info->dc_adc_sum += result;
		bat_info->dc_adc_cnt++;
	}
#else
	bat_info->hole_dc_adc = result;
#endif
}
//end-wisky-lxh@20110620

static void dc_adc_callback(struct adc_client *client, void *client_param, int result)
{
	struct wisky_battery_info *bat_info = (struct wisky_battery_info *)client_param;

#if 0
	if(bat_info->dc_adc_cnt >= ADC_SAMPLE_RATE){
		bat_info->dc_adc = bat_info->dc_adc_sum/ADC_SAMPLE_RATE;
		bat_info->dc_adc_cnt = 0;
		bat_info->dc_adc_sum = 0;
	}else{
		bat_info->dc_adc_sum += result;
		bat_info->dc_adc_cnt++;
	}
#else
	bat_info->dc_adc = result;
#endif
}

static void update_battery_status(struct wisky_battery_info *bat_info)
{
	int bat_adc, bat_temp;
	static int not_charge_count = 0;
	static int charge_full_cnt = 0;
	static int charge_full_led_on = 0;

	bat_info->usb_status = get_usb_connect_flag();
	bat_info->dc_status = get_dc_status(bat_info);
	bat_info->charge_status = get_charge_status();

	external_power_status = bat_info->usb_status |bat_info->dc_status;

	//if(!strcmp(ANDROID_CONFIG_XM, "1")){
		if(external_power_status){
			if(false == bat_info->wakelock_state){
				wake_lock(&bat_info->wakelock);
				bat_info->wakelock_state = true;
			}
		}else{
			if(true == bat_info->wakelock_state){
				wake_unlock(&bat_info->wakelock);
				bat_info->wakelock_state = false;
			}
		}
	//}

	if(external_power_status){
		if(POWER_SUPPLY_STATUS_CHARGING == bat_info->charge_status){
			bat_info->bat_adc_min = BATT_MIN_CHARGE_ADC_VALUE;
			bat_info->bat_adc_max = BATT_FULL_CHARGE_ADC_VALUE;
			bat_info->bat_adc_range = (bat_info->bat_adc_max - bat_info->bat_adc_min);
		}else{
			bat_info->bat_adc_min = BATT_MIN_DCIN_NOCHARGE_ADC;
			bat_info->bat_adc_max = BATT_FULL_DCIN_NOCHARGE_ADC;
			bat_info->bat_adc_range = (bat_info->bat_adc_max - bat_info->bat_adc_min);
		}
	}else{
		bat_info->bat_adc_min = BATT_MIN_ADC_VALUE;
		bat_info->bat_adc_max = BATT_FULL_ADC_VALUE;
		bat_info->bat_adc_range = (bat_info->bat_adc_max - bat_info->bat_adc_min);
	}
	
	if ((bat_info->dc_status & USB_HOLE_DC_MASK)/* || USB_PLUGIN == bat_info->usb_status*/){		
#if defined(CHARGE_FULL_CHECK_BY_RTC)
		if(bat_info->last_capacity < RTC_RECORD_TRIGGER_BAT_LEVEL){//99
			//电量小于99时清除记录，只有>=99时开始记录充满电起始时间点
			bat_info->rtc_record_trigger = 0;
		}
#endif
		if(is_battery_full(bat_info)){
			if(++charge_full_cnt > 200){
				WPRINTK("--->>>Battery Charge FULL!!!\n");
				bat_info->property.capacity = BATT_LEVEL_FULL;
				bat_info->property.status = POWER_SUPPLY_STATUS_FULL;
				charge_full_led_on = 1;
				set_charge_led(charge_full_led_on);
			}
			return;
		}else{
			if(bat_charge_level != BAT_CHARGE_LEVEL_DISABLE){
				bat_info->property.status = POWER_SUPPLY_STATUS_CHARGING;
			}else{
				not_charge_count++;
				if(not_charge_count > 30){
					bat_info->property.status = POWER_SUPPLY_STATUS_NOT_CHARGING;
				}
			}
			charge_full_cnt = 0;
		}		
	}else{		
		bat_info->property.status = POWER_SUPPLY_STATUS_DISCHARGING;
		charge_full_cnt = 0;
		not_charge_count = 0;
		if(1 == charge_full_led_on){
			charge_full_led_on = 0;
			set_charge_led(charge_full_led_on);	
		}
#if defined(CHARGE_FULL_CHECK_BY_RTC)
		bat_info->rtc_record_trigger = 0;
#endif
	}
	
	bat_info->property.health = POWER_SUPPLY_HEALTH_GOOD;

}


#if defined(CHARGE_FULL_CHECK_BY_RTC)
static void battery_delaywork(struct work_struct *work)
{
	struct wisky_battery_info *bat_info = container_of(to_delayed_work(work), struct wisky_battery_info, delay_work);
	
	wisky_rtc_read_time(&bat_info->curr_rtc);
	if(0 == bat_info->rtc_record_trigger){
		memcpy(&bat_info->trigger_rtc, &bat_info->curr_rtc, sizeof(struct rtc_time));
		bat_info->rtc_record_trigger = 1;
	}
}
#endif

extern int wisky_debug_write_log_directly(char* plog);
extern int wisky_debug_write_log_thread(char* plog);
static void wisky_battery_timer_func(unsigned long arg)
{
	struct wisky_battery_info *bat_info = (struct wisky_battery_info *)arg;
	int old_bat_status = 0;
	int old_bat_capacity = 0;
	int charge_sample_count = 0;

	if(battery_info->early_suspend_state && !external_power_status){
		if(bat_info->bat_suspend_adc_triger++ <= SUSPEND_ADC_TRIGGER_CNT){
			adc_async_read(bat_info->bat_adc_client);
		}
	}
	
//	charge_sample_count = (bat_info->last_capacity < 60)?SAMPLE_COUNT_CHARGE_MIN:SAMPLE_COUNT_CHARGE_MAX;
//	if((++bat_info->sample_batt_count >= (external_power_status?charge_sample_count:SAMPLE_COUNT_DISCHARGE))) //设定的采样时间到要进行ADC
	if(++bat_info->sample_batt_count >= SAMPLE_COUNT_DISCHARGE) //设定的采样时间到要进行ADC
	{
		if(bat_info->bat_adc_client){
			bat_info->sample_batt_state = SAMPLE_BATT_START;
			adc_async_read(bat_info->bat_adc_client);
		}
	}
	if(bat_info->dc_adc_client){
		adc_async_read(bat_info->dc_adc_client);
	}
	//wisky-lxh@20110622,add hole dc adc
	if(bat_info->hole_dc_adc_client){
		adc_async_read(bat_info->hole_dc_adc_client);
	}
	//end-wisky-lxh@20110622

	//store old status
	old_bat_status = bat_info->property.status;
	old_bat_capacity = bat_info->property.capacity;
	//updata new status
	update_battery_status(bat_info);

	flash_charging_led(bat_info);
	
	if(old_bat_status != bat_info->property.status){
		WPRINTK("%s[%d]: ----->DC status change, last_capacity=(%d), capacity=(%d)\n", 
			__FUNCTION__, __LINE__, bat_info->last_capacity, bat_info->property.capacity);
		//use last update capacity when DC/USB changed
		if(POWER_SUPPLY_STATUS_FULL != bat_info->property.status){
			if(bat_info->last_capacity==BATT_LEVEL_FULL && POWER_SUPPLY_STATUS_FULL != old_bat_status \
				&& POWER_SUPPLY_STATUS_CHARGING==bat_info->property.status){
				bat_info->property.capacity = 99;
			}else{
				//bat_info->property.capacity = bat_info->last_capacity;
			}
		}

		bat_info->property.voltage_now = (bat_info->property.capacity)*BATT_RANGE_VOL_VALUE/100+BATT_MIN_VOL_VALUE;
		power_supply_changed(&bat_info->ac);
		power_supply_changed(&bat_info->battery);		
//		bat_info->last_capacity = bat_info->property.capacity;
		battery_capacity = bat_info->property.capacity;
//		bat_info->sample_batt_count = 0;
//		bat_info->sample_batt_state = SAMPLE_BATT_STOP;
//		bat_info->bat_adc_cnt = 0;
//		bat_info->bat_adc_sum = 0;

	//}else if(BATT_UPDATE_ON == bat_info->sample_batt_update){
	}	else if(bat_info->last_capacity != bat_info->property.capacity){
		WPRINTK("%s[%d]: ----->UPDATE: last_capacity=(%d), capacity=(%d)\n", 
			__FUNCTION__, __LINE__, bat_info->last_capacity, bat_info->property.capacity);
		bat_info->property.voltage_now = (bat_info->property.capacity)*BATT_RANGE_VOL_VALUE/100+BATT_MIN_VOL_VALUE;
		power_supply_changed(&bat_info->battery);
		bat_info->last_capacity = bat_info->property.capacity;
		bat_info->sample_batt_update = BATT_UPDATE_OFF;
		battery_capacity = bat_info->property.capacity;
	}
#if defined(LOW_BATT_WAKEUP_PIN)
	else if(1 == bat_info->low_batt_wakeup){
		bat_info->property.capacity = 0;
		pr_info("%s[%d]: ----->Low battery wakeup, capacity=(%d)\n", 
			__FUNCTION__, __LINE__, bat_info->property.capacity);
		power_supply_changed(&bat_info->battery);
	}
#endif

	//start next timer
	bat_info->timer.expires = BATT_CHARGE_POLL;
	add_timer(&bat_info->timer);
}


#ifdef CONFIG_HAS_EARLYSUSPEND
void wisky_battery_ealry_suspend(struct early_suspend *h)
{
	int i;
	
	battery_info->early_suspend_state = 1;
	
	if(get_usb_connect_flag() == USB_PLUGIN){
		set_bat_charge_current_mode(battery_info, BAT_CHARGE_LEVEL_MIN);
	}else{
		set_bat_charge_current_mode(battery_info, BAT_CHARGE_LEVEL_MAX);
	}

	//wisky-lxh@20110721,hole dc in irq
#if defined(HOLE_DC_DETECT_PIN)
	if (HOLE_DC_DETECT_PIN != INVALID_GPIO)
		enable_irq(hole_dc_irq);
#endif
	//end-wisky-lxh@20110721
}

void wisky_battery_ealry_resume(struct early_suspend *h)
{
	int i;
	
	battery_info->early_suspend_state = 0;

	battery_info->bat_suspend_adc_triger = 0;

	if(get_usb_connect_flag() == USB_PLUGIN){
		set_bat_charge_current_mode(battery_info, BAT_CHARGE_LEVEL_MIN);
	}else{
		set_bat_charge_current_mode(battery_info, BAT_CHARGE_LEVEL_MAX);
	}
	
	/*if(1 == min_charge_flag || 1 == max_charge_flag){
		//make sure enter min charge level when resume,avoid big current
		set_bat_charge_current_mode(battery_info, BAT_CHARGE_LEVEL_MIN);
	}*/

#if defined(HOLE_DC_DETECT_PIN)
	if (HOLE_DC_DETECT_PIN != INVALID_GPIO)
		disable_irq(hole_dc_irq);
#endif
}

static struct early_suspend wisky_battery_early_suspend_info = {
	.suspend =  wisky_battery_ealry_suspend,
	.resume =  wisky_battery_ealry_resume,
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
};

#endif

static int wisky_battery_probe(struct platform_device *pdev)
{
	int  rc = -1, i = 0;
	struct wisky_battery_info  *bat_info;
	struct battery_platform_data *pdata = (struct battery_platform_data *)pdev->dev.platform_data;

	bat_info=kzalloc(sizeof(*bat_info), GFP_KERNEL);
	if(bat_info == NULL){
		rc = -ENOMEM;
		goto failed1;
	}
	platform_set_drvdata(pdev, bat_info);
	battery_info = bat_info;
	
	if(pdata && pdata->io_init){
		pdata->io_init();
	}

	bat_info->ac = wisky_power_supplies[0];
	bat_info->battery = wisky_power_supplies[1];
//	bat_info->usb = wisky_power_supplies[2];

	rc = power_supply_register(&pdev->dev, &bat_info->ac);
	if (rc){
		printk(KERN_ERR "Failed to register ac power supply (%d)\n", rc);
		goto failed2;
	}
	rc = power_supply_register(&pdev->dev, &bat_info ->battery);
	if (rc){
		printk(KERN_ERR "Failed to register battery power supply (%d)\n", rc);
		goto failed2;
	}
	/*rc = power_supply_register(&pdev->dev, &bat_info ->usb);
	if (rc){
		printk(KERN_ERR "Failed to register usb power supply (%d)\n", rc);
		goto failed2;
	}*/
	
	bat_info->early_suspend_state = 0;
	bat_info->suspend_state = 0;
	bat_info->bat_adc_base = 0;
	bat_info->bat_adc = 0;
	bat_info->dc_adc = 0;
	bat_info->hole_dc_adc = 0;//wisky-lxh@20110620
	bat_info->bat_adc_cnt = 0;
	bat_info->bat_adc_sum = 0;
	bat_info->last_capacity = 0;
	bat_info->bat_adc_min = BATT_MIN_ADC_VALUE;
	bat_info->bat_adc_max = BATT_FULL_ADC_VALUE;
	bat_info->bat_adc_range = (bat_info->bat_adc_max - bat_info->bat_adc_min);

	bat_info->sample_batt_state = SAMPLE_BATT_STOP;
	bat_info->sample_batt_count = 0;
	bat_info->sample_batt_update = 0;
	bat_info->boot_first = 0;
	bat_info->system_ready = 0;
	bat_info->batt_charge_state = BATT_CHARGE_DISABLE;
	
	bat_info->property.capacity = 80;
	bat_info->property.health = POWER_SUPPLY_HEALTH_GOOD;
	bat_info->property.present = BATT_PRESENT_TRUE;
	bat_info->property.status = POWER_SUPPLY_STATUS_DISCHARGING;//POWER_SUPPLY_STATUS_NOT_CHARGING;
	bat_info->property.voltage_now = 0;

	bat_level_filter.count = 0;
	bat_level_filter.level_temp = 0;
	memset(bat_level_filter.level, 0, BAT_FILTER_SIZE*sizeof(bat_level_filter.level[0]));

	bat_adc_filter.count = 0;
	bat_adc_filter.adc_temp = 0;
	memset(bat_adc_filter.adc, 0, BAT_ADC_SIZE*sizeof(bat_adc_filter.adc[0]));

	bat_info->sleep_too_long = 0;
	
	if(pdata->bat_adc_ch >= 0){
		bat_info->bat_adc_client = adc_register(pdata->bat_adc_ch, battery_adc_callback, (void*)bat_info);
		if(!bat_info->bat_adc_client){
			printk("%s[%d]: adc register failed!\n", __FUNCTION__, __LINE__);
			goto failed2;
		}
	}
	if(pdata->dc_adc_ch >= 0){
		bat_info->dc_adc_client = adc_register(pdata->dc_adc_ch, dc_adc_callback, (void*)bat_info);
		if(!bat_info->dc_adc_client){
			printk("%s[%d]: adc register failed!\n", __FUNCTION__, __LINE__);
			goto failed3;
		}
	}
	//wisky-lxh@20110620,add hole dc adc 
	if(pdata->hole_dc_adc_ch >= 0){
		bat_info->hole_dc_adc_client = adc_register(pdata->hole_dc_adc_ch, hole_dc_adc_callback, (void*)bat_info);
		if(!bat_info->hole_dc_adc_client){
			printk("%s[%d]: hole adc register failed!\n", __FUNCTION__, __LINE__);
			goto failed2;
		}
	}

	get_dc_status(bat_info);
	//for(i = 0; i < 3; i ++){
		if(bat_info->bat_adc_client){
			adc_async_read(bat_info->bat_adc_client);
		}
		if(bat_info->dc_adc_client){
			adc_async_read(bat_info->dc_adc_client);
		}
		if(bat_info->hole_dc_adc_client){
			adc_async_read(bat_info->hole_dc_adc_client);
		}
	//}
	
#if defined(HOLE_DC_DETECT_PIN)
	//when in suspend,hole dc input will wake up system
	if (HOLE_DC_DETECT_PIN != INVALID_GPIO)
	{
		hole_dc_irq = gpio_to_irq(HOLE_DC_DETECT_PIN);
		rc = request_irq(hole_dc_irq, hole_dc_wakeup_handler, IRQF_TRIGGER_FALLING, "hole dc in", NULL);
		if (rc < 0) {
			pr_err("%s: request_irq(%d) failed\n", __func__, hole_dc_irq);
			gpio_free(HOLE_DC_DETECT_PIN);
			return rc;
		}
		enable_irq_wake(hole_dc_irq);		
		disable_irq(hole_dc_irq);
	}
#endif
	//end-wisky-lxh@20110620

	init_timer(&bat_info->timer);
	bat_info->timer.function = &wisky_battery_timer_func;
	bat_info->timer.data = (unsigned long)bat_info;
	bat_info->timer.expires = BATT_CHARGE_POLL;
	add_timer(&bat_info->timer);
	
	bat_info->workqueue = create_workqueue("battery_workqueue");
#if defined(CHARGE_FULL_CHECK_BY_RTC)
	INIT_DELAYED_WORK(&bat_info->delay_work, battery_delaywork);
#endif

	if(!strcmp(ANDROID_CONFIG_XM, "1")){
		bat_info->wakelock_state = false;
		wake_lock_init(&bat_info->wakelock, WAKE_LOCK_SUSPEND, "battery_wake");
	}

	pr_info("create battery charge thread\n");
	thread_run_time = jiffies + THREAD_START_TIME;
	kernel_thread(bat_charge_thread, (void *)bat_info, CLONE_KERNEL|SIGCHLD);

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&wisky_battery_early_suspend_info);
#endif

#if defined(LOW_BATT_WAKEUP_PIN)
	INIT_DELAYED_WORK(&bat_info->lowbat_delay_work, lowbat_delay_work_hdl);
	rk30_mux_api_set(LOW_BATT_WAKEUP_MUX_NAME, LOW_BATT_WAKEUP_MUX_MODE);
	low_battery_wakeup_init(LOW_BATT_WAKEUP_PIN);
#endif

	#if defined(WISKY_BATTERY_SMB347)
	rc = i2c_add_driver(&smb347_i2c_driver);
	if(rc < 0){
		pr_err("%s[%d]: SMB347 i2c driver add failed!\n", __FUNCTION__, __LINE__);
	}
	#endif

	printk("%s[%d]: battery driver probe OK.\n", __FUNCTION__, __LINE__);
	return 0;

failed3:
	adc_unregister(bat_info->bat_adc_client);

failed2:
	power_supply_unregister(&bat_info->ac);
	power_supply_unregister(&bat_info->battery);
	kfree(bat_info);
	
failed1:
 	printk("%s[%d]: battery driver probe failed!\n", __FUNCTION__, __LINE__);
	return rc;
		
}

static int __devexit wisky_battery_remove(struct platform_device *pdev)
{
	struct battery_platform_data *pdata = (struct battery_platform_data *)pdev->dev.platform_data;
	struct wisky_battery_info  *bat_info = platform_get_drvdata(pdev);

#ifdef CONFIG_HAS_EARLYSUSPEND
        unregister_early_suspend(&wisky_battery_early_suspend_info);
#endif

	if(!strcmp(ANDROID_CONFIG_XM, "1")){		
		wake_lock_destroy(&bat_info->wakelock);
	}
	
	if(pdata->bat_adc_ch >= 0 && bat_info->bat_adc_client);
		adc_unregister(bat_info->bat_adc_client);
	if(pdata->dc_adc_ch >= 0 && bat_info->dc_adc_client);
		adc_unregister(bat_info->dc_adc_client);
	//wisky-lxh@20110622,add hole dc detect	
	if(pdata->hole_dc_adc_ch >= 0 && bat_info->hole_dc_adc_client);
		adc_unregister(bat_info->hole_dc_adc_client);
	//end-wisky-lxh@20110622	

	#if defined(WISKY_BATTERY_SMB347)
	i2c_del_driver(&smb347_i2c_driver);
	#endif
	if(pdata && pdata->io_deinit){
		pdata->io_deinit();
	}
	
	return 0;
}

static int wisky_battery_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct wisky_battery_info *bat_info = platform_get_drvdata(pdev);
	
	WPRINTK("%s[%d]: \n", __FUNCTION__, __LINE__);	

#if defined(LOW_BATT_WAKEUP_PIN)
	enable_irq(battery_info->low_batt_irq);
	battery_info->low_batt_wakeup = 0;
	battery_info->low_batt_wakeup_count = 0;
#endif

	if(POWER_SUPPLY_STATUS_CHARGING == bat_info->property.status){
		set_charge_led(0);
	}
	bat_info->sleep_too_long = 0;
	wisky_rtc_read_time(&bat_info->suspend_rtc);
	
	bat_info->suspend_state = 1;

	return 0;
}

static int wisky_battery_resume(struct platform_device *pdev)
{
	struct wisky_battery_info *bat_info = platform_get_drvdata(pdev);
	
	WPRINTK("%s[%d]: \n", __FUNCTION__, __LINE__);

#if defined(LOW_BATT_WAKEUP_PIN)
	disable_irq(battery_info->low_batt_irq);
#endif

	wisky_rtc_read_time(&battery_info->resume_rtc);
	if(rtc_delay_minute(&battery_info->suspend_rtc, &battery_info->resume_rtc, 30)){
		battery_info->sleep_too_long = 120;//120-->adc sample times about 12
	}
	
	bat_info->suspend_state = 0;

	return 0;
}

#if 0
static ssize_t ac_power_mode_show(struct device_driver *_drv,char *_buf)
{
        int count;

        count = sprintf(_buf,"%d",ac_power_off);
        return count;
}

static ssize_t ac_power_mode_store(struct device_driver *_drv,const char *_buf,size_t _count)
{
          
          ac_power_off  = (int)simple_strtol(_buf, NULL, 10);

          return _count;

}

static DRIVER_ATTR(ac_power_off,0666,ac_power_mode_show,ac_power_mode_store);

#endif

static struct platform_driver wisky_battery_driver = {
	.driver	= {
		.name	= BATT_DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe = wisky_battery_probe,
	.remove = wisky_battery_remove,
	.suspend = wisky_battery_suspend,
	.resume = wisky_battery_resume,
};


static int __init wisky_battery_init(void)
{
	int ret = platform_driver_register(&wisky_battery_driver);
	
#if 0
	if (ret == 0){
		ret = driver_create_file(&wisky_battery_driver.driver, &driver_attr_ac_power_off);
	}
#endif

	return ret;
}

subsys_initcall(wisky_battery_init);
MODULE_DESCRIPTION("Wisky Battery Driver");
MODULE_LICENSE("GPL");

