/* wisky/battery/wisky_battery_smb347.c
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
 * V001:20120508 cd huang
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
#include "wisky_battery_smb347.h"

#define BATT_PLATFORM_DRIVER_NAME			"wisky_battery"

#define BATTERY_TIMER_POLL		(jiffies+20)//10ms each jiffy
#define THREAD_SLEEP_POLL_MS		200	//ms

struct power_property{
	int status;
	int health;
	int capacity;
	int present;
	int voltage_now;
};

struct smb347_driver_data{
	struct i2c_client *client;
	struct wake_lock wakelock;	//hold wake state when external dc insert
	bool wakelock_state;
	struct power_supply ac;
	struct power_supply battery;
	struct power_property property;
	int ac_status;
	int usb_status;
	struct timer_list timer;
};

static struct smb347_driver_data *gSmb347Driver = NULL;

static const struct i2c_device_id smb347_id[] = {
	{SMB347_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, smb347_id);

//return read bytes number if success, else negative number
static int smb347_read(const char reg, char *buf, int count)
{
	int ret = 0;

	ret = i2c_master_reg8_recv(gSmb347Driver->client, reg, buf, count, SMB347_I2C_SCL_RATE);

	return ret;
}

static int smb347_write(const char reg, char *buf, int count)
{
	int ret = 0;

	ret = i2c_master_reg8_send(gSmb347Driver->client, reg, buf, count, SMB347_I2C_SCL_RATE);

	return ret;
}

static int smb347_powerup_init(void)
{
	char buf[2];

	smb347_read(SMB347_CMD_REGA, buf, 1);
	buf[0] = buf[0] | OTG_ENABLE;//OTG enable
	smb347_write(SMB347_CMD_REGA, buf, 1);
}

static int smb347_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	
	WPRINTK("%s[%d]: i2c driver probe enter\n", __FUNCTION__, __LINE__);

	gSmb347Driver->client = client;
	i2c_set_clientdata(client, gSmb347Driver);

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

static enum power_supply_property smb347_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property smb347_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int smb347_ac_get_property(struct power_supply *ps, enum power_supply_property psp, 
	union power_supply_propval *val)
{
	if(NULL == gSmb347Driver){
		return -EFAULT;
	}
	
	switch(psp){
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = (gSmb347Driver->ac_status > 0)?1:0;
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int smb347_battery_get_property(struct power_supply *ps, enum power_supply_property psp, 
	union power_supply_propval *val)
{
	if(NULL == gSmb347Driver){
		return -EFAULT;
	}

	switch (psp) {
		/*case POWER_SUPPLY_PROP_ONLINE:
			val->intval = battery_info->dc_status;
			break;*/
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = gSmb347Driver->property.status;
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = gSmb347Driver->property.health;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = gSmb347Driver->property.present;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = gSmb347Driver->property.capacity;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val ->intval = gSmb347Driver->property.voltage_now;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MIN:
			val->intval = BATTERY_VOLTAGE_MIN*1000;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX:
			val->intval = BATTERY_VOLTAGE_MAX*1000;
			break;
		default:		
			return -EINVAL;
	}

	return 0;
}

static struct power_supply smb347_power_supply[] = {
	{
		.name = "mains",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.properties = smb347_ac_properties,
		.num_properties = ARRAY_SIZE(smb347_ac_properties),
		.get_property = smb347_ac_get_property,
	},
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = smb347_battery_properties,
		.num_properties = ARRAY_SIZE(smb347_battery_properties),
		.get_property = smb347_battery_get_property,
	},
};

#if 0
static void smb347_timer(unsigned long arg)
{
	struct smb347_driver_data *smb347_driver = (struct smb347_driver_data *)arg;

	//start next timer
	smb347_driver->timer.expires = BATTERY_TIMER_POLL;
	add_timer(&smb347_driver->timer);

	power_supply_changed(&smb347_driver->ac);
	power_supply_changed(&smb347_driver->battery);
}
#else
static pid_t smb347_charge_thread(void *arg)
{
	struct task_struct *task = current;
	struct smb347_driver_data *smb347_driver = (struct smb347_driver_data *)arg;
	char buf[2];

	pr_info("%s[%d]: smb347 battery charge thread start\n", __FUNCTION__, __LINE__);
	
	daemonize("smb347_thread");
	allow_signal(SIGKILL);

	while(!signal_pending(task)){
//		WPRINTK("%s[%d]: update power sypply\n", __FUNCTION__, __LINE__);
/*
		smb347_read(SMB347_STATUS_REGA, buf, 1);
		WPRINTK("status reg A: 0x%x\n", buf[0]);
		smb347_read(SMB347_STATUS_REGB, buf, 1);
		WPRINTK("status reg B: 0x%x\n", buf[0]);*/

		power_supply_changed(&smb347_driver->ac);
		power_supply_changed(&smb347_driver->battery);
		
thread_sleep:
		msleep(THREAD_SLEEP_POLL_MS);
	}
	
	pr_info("%s[%d]: smb347 battery charge thread exit!!!\n", __FUNCTION__, __LINE__);
}
#endif

static int smb347_battery_probe(struct platform_device *pdev)
{
	struct smb347_driver_data *smb347_driver;
	struct battery_platform_data *pdata = (struct battery_platform_data *)pdev->dev.platform_data;
	int ret = 0;
	
	smb347_driver = kzalloc(sizeof(struct smb347_driver_data), GFP_KERNEL);
	if(NULL == smb347_driver){
		ret = -ENOMEM;
		goto err1;
	}
	gSmb347Driver = smb347_driver;
	
	platform_set_drvdata(pdev, smb347_driver);

	if(pdata && pdata->io_init){
		pdata->io_init();
	}

	smb347_driver->ac = smb347_power_supply[0];
	smb347_driver->battery = smb347_power_supply[1];
	ret = power_supply_register(&pdev->dev, &smb347_driver->ac);
	if(ret < 0){
		printk(KERN_ERR "Failed to register smb347 power supply (AC)!\n");
		ret = -EFAULT;
		goto err2;
	}
	ret = power_supply_register(&pdev->dev, &smb347_driver->battery);
	if(ret < 0){
		printk(KERN_ERR "Failed to register smb347 power supply(Battery)!\n");
		ret = -EFAULT;
		goto err2;
	}

	smb347_driver->property.status = POWER_SUPPLY_STATUS_DISCHARGING;
	smb347_driver->property.health = POWER_SUPPLY_HEALTH_GOOD;
	smb347_driver->property.capacity = 50;
	smb347_driver->property.present = TRUE;
	smb347_driver->property.voltage_now = 0;
	
#if 0
	init_timer(&smb347_driver->timer);
	smb347_driver->timer.function = &smb347_timer;
	smb347_driver->timer.data = (unsigned long)smb347_driver;
	smb347_driver->timer.expires = BATTERY_TIMER_POLL;
	add_timer(&smb347_driver->timer);
#else
	kernel_thread(smb347_charge_thread, (void *)smb347_driver, CLONE_KERNEL|SIGCHLD);
#endif

	smb347_driver->wakelock_state = false;
	wake_lock_init(&smb347_driver->wakelock, WAKE_LOCK_SUSPEND, "smb347_wakelock");

	ret = i2c_add_driver(&smb347_i2c_driver);
	if(ret < 0){
		pr_err("%s[%d]: SMB347 i2c driver add failed!\n", __FUNCTION__, __LINE__);
	}
	
	pr_info("%s[%d]: SMB347 battery manager driver probe done.\n", __FUNCTION__, __LINE__);
	return 0;

err2:
	kfree(smb347_driver);
	gSmb347Driver = NULL;
err1:
	pr_err("%s[%d]: SMB347 battery manager driver probe failed!\n", __FUNCTION__, __LINE__);
	return ret;	
}

static int smb347_battery_remove(struct platform_device *pdev)
{
	struct smb347_driver_data *smb347_driver;
	struct battery_platform_data *pdata = (struct battery_platform_data *)pdev->dev.platform_data;

	smb347_driver = platform_get_drvdata(pdev);
	
	wake_lock_destroy(&smb347_driver->wakelock);
	power_supply_unregister(&smb347_driver->ac);
	power_supply_unregister(&smb347_driver->battery);
	
	if(pdata && pdata->io_deinit){
		pdata->io_deinit();
	}

	i2c_del_driver(&smb347_i2c_driver);
	
	kfree(smb347_driver);
	gSmb347Driver = NULL;
	
	return 0;
}

static int smb347_battery_suspend(struct platform_device *pdev, pm_message_t state)
{

	return 0;
}

static int smb347_battery_resume(struct platform_device *pdev)
{

	return 0;
}

static struct platform_driver smb347_battery_driver = {
	.driver	= {
		.name = BATT_PLATFORM_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = smb347_battery_probe,
	.remove = smb347_battery_remove,
	.suspend = smb347_battery_suspend,
	.resume = smb347_battery_resume,
};

static int __init smb347_battery_init(void)
{
	int ret = platform_driver_register(&smb347_battery_driver);

	return ret;
}

static void __exit smb347_battery_exit(void)
{
	platform_driver_unregister(&smb347_battery_driver);
}


subsys_initcall(smb347_battery_init);
module_exit(smb347_battery_exit);

MODULE_DESCRIPTION("Wisky Battery Driver");
MODULE_LICENSE("GPL");

