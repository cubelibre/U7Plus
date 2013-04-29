/* wisky/gsensor/wisky_gsensor_dmard06.c
 *	dmard06 accelerometer driver
 *
 * Copyright (C) 2007-2008 HTC Corporation.
 * Author: Hou-Kun Chen <houkun.chen@gmail.com>
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
 * V001:20110226 cd lan
 *	1.Modify for wisky MID project from rk2918 sdk.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <mach/gpio.h>
#include <mach/board.h> 
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif
#include "wisky_gsensor_dmard06.h"

static int  dmard06_probe(struct i2c_client *client, const struct i2c_device_id *id);

#define DMARD06_SPEED		(300 * 1000)

#define DMARD06_DATA_FILTER	1


#define DMARD06_DEFAULT_RATE	32
#define DMARD06_MAX_RATE	64
//#define SENSOR_DEBUG
#ifdef SENSOR_DEBUG
#define Sensor_debug(format, args...)	do {		\
			printk(KERN_INFO "<SENSOR-DEBUG> " format , ## args);	\
		} while (0)
#else
#define Sensor_debug(format, args...)
#endif

#ifdef DMARD06_DATA_FILTER
struct dmard06_data_tag{
	#define DMARD06_TEMP_SIZE  10	//sample times
	int count;
	int axis_x[DMARD06_TEMP_SIZE];
	int axis_y[DMARD06_TEMP_SIZE];
	int axis_z[DMARD06_TEMP_SIZE];
};
static struct dmard06_data_tag dmard06_adjust;
#endif

/* Addresses to scan -- protected by sense_data_mutex */
//static char sense_data[RBUFF_SIZE + 1];
static struct i2c_client *this_client;
static struct miscdevice dmard06_device;

static DECLARE_WAIT_QUEUE_HEAD(rd06_data_ready_wq);

#ifdef CONFIG_ANDROID_POWER
static android_early_suspend_t dmard06_early_suspend;
#endif
static int rd06_revision = -1;
/* AKM HW info */
static ssize_t rd06_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%#x\n", rd06_revision);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(rd06_vendor, 0444, rd06_vendor_show, NULL);

static struct kobject *android_gsensor_kobj;

static int rd06_sysfs_init(void)
{
	int ret ;

	android_gsensor_kobj = kobject_create_and_add("android_gsensor", NULL);
	if (android_gsensor_kobj == NULL) {
		printk(KERN_ERR
		       "DMARD06 gsensor_sysfs_init:"
		       "subsystem_register failed\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = sysfs_create_file(android_gsensor_kobj, &dev_attr_rd06_vendor.attr);
	if (ret) {
		printk(KERN_ERR
		       "DMARD06 gsensor_sysfs_init:"
		       "sysfs_create_group failed\n");
		goto err4;
	}

	return 0 ;
err4:
	kobject_del(android_gsensor_kobj);
err:
	return ret ;
}

static int dmard06_rx_data(struct i2c_client *client, char *rxData, int length)
{
	int ret = 0;
	char reg = rxData[0];
	ret = i2c_master_reg8_recv(client, reg, rxData, length, DMARD06_SPEED);
	return (ret > 0)? 0 : ret;
}

static int dmard06_tx_data(struct i2c_client *client, char *txData, int length)
{
	int ret = 0;
	char reg = txData[0];
	ret = i2c_master_reg8_send(client, reg, &txData[1], length-1, DMARD06_SPEED);
	return (ret > 0)? 0 : ret;
}

static int dmard06_set_rate(struct i2c_client *client, char rate)
{
/*	char buffer[2];
	int ret = 0;
	int i;
	
	if (rate > 128)
        return -EINVAL;

    for (i = 0; i < 7; i++) {
        if (rate & (0x1 << i))
            break;
    }   

	buffer[0] = DMARD06_REG_SR;
	buffer[1] = 0xf8 | (0x07 & (~i));

	ret = dmard06_tx_data(client, &(buffer[0]), 2);
	ret = dmard06_rx_data(client, &(buffer[0]), 1);

	return ret;*/
}
static int dmard06_init(struct i2c_client *client)
{
	printk("dmard06_init\n");
	char buffer[2];
	int ret = 0;

	buffer[0] = 0x06;
	ret = dmard06_rx_data(client, &buffer[0], 1);
	printk("DMARD06_ID=%d\n",buffer[0]);
	
	buffer[0] = 0x53;

	ret = dmard06_rx_data(client, &buffer[0], 1);
	printk("DMARD06_REG_INT=%d\n",buffer[0]);
}
static int dmard06_start_dev(struct i2c_client *client, char rate)
{
	char buffer[2];
	int ret = 0;

	buffer[0] = DMARD06_REG_PM;
	buffer[1] = 0x27;	//0x10; modify by zhao
	ret = dmard06_tx_data(client, &buffer[0], 2);
	ret = dmard06_rx_data(client, &buffer[0], 1);
	printk("DMARD06_REG_PM=%d\n",buffer[0]);

	enable_irq(client->irq);
	
	return ret;
}

static int dmard06_test_dev(struct i2c_client *client, char rate)
{
	char buffer[2];
	int ret = 0;

	buffer[0] = DMARD06_REG_PM;
	buffer[1] = 0x27;
	ret = dmard06_tx_data(client, &buffer[0], 2);
	ret = dmard06_rx_data(client, &buffer[0], 1);
	printk("ret=%d\n",buffer[0]);
	
	return ret;
}

static int dmard06_start(struct i2c_client *client, char rate)
{ 
    struct dmard06_data *dmard06 = (struct dmard06_data *)i2c_get_clientdata(client);
    
    if (dmard06->status == DMARD06_OPEN) {
        return 0;      
    }
    dmard06->status = DMARD06_OPEN;
    return dmard06_start_dev(client, rate);
}

static int dmard06_close_dev(struct i2c_client *client)
{    	
	char buffer[2];

	disable_irq_nosync(client->irq);

	buffer[0] = DMARD06_REG_PM;
	buffer[1] = 0x07;
	
	return dmard06_tx_data(client, buffer, 2);
}

static int dmard06_close(struct i2c_client *client)
{
    struct dmard06_data *dmard06 = (struct dmard06_data *)i2c_get_clientdata(client);
    
    dmard06->status = DMARD06_CLOSE;
    
    return dmard06_close_dev(client);
}

/*
*系统使用的共用四种速率:
*	 FASTEST(0), GAME(20), UI(60), NORMAL(200)
*/
static int dmard06_reset_rate(struct i2c_client *client, char rate)
{
/*	int ret = 0;

#if defined(WISKY_MID)
	if(rate >= 200){
		rate = DMARD06_DEFAULT_RATE;
	}else{
		rate = DMARD06_MAX_RATE;
	}
#endif

	ret = dmard06_close_dev(client);
	ret = dmard06_start_dev(client, rate);
    
	return ret ;*/
}

static inline int dmard06_convert_to_int(char value)
{
    int result;
		
    if (value <90) {
       result = value *DMARD06_GRAVITY_STEP;
    } else {
       result = (value-255)* DMARD06_GRAVITY_STEP;
    }
  	
    return result;

}

static void dmard06_report_value(struct i2c_client *client, struct dmard06_axis *axis)
{
	struct dmard06_data *dmard06 = i2c_get_clientdata(client);
    //struct dmard06_axis *axis = (struct dmard06_axis *)rbuf;
#ifdef DMARD06_DATA_FILTER
	int i = 0, axis_x=0, axis_y=0, axis_z=0;
#endif

#ifdef DMARD06_DATA_FILTER
	for(i = 0; i < DMARD06_TEMP_SIZE; i++){
		if((DMARD06_TEMP_SIZE-1) == i){
			dmard06_adjust.axis_x[i] = axis->x;
			dmard06_adjust.axis_y[i] = axis->y;
			dmard06_adjust.axis_z[i] = axis->z;
		}else{
			dmard06_adjust.axis_x[i] = dmard06_adjust.axis_x[i+1];
			dmard06_adjust.axis_y[i] = dmard06_adjust.axis_y[i+1];
			dmard06_adjust.axis_z[i] = dmard06_adjust.axis_z[i+1];
		}
	}
	for(i = 0; i < DMARD06_TEMP_SIZE; i++){
		axis_x += dmard06_adjust.axis_x[i];
		axis_y += dmard06_adjust.axis_y[i];
		axis_z += dmard06_adjust.axis_z[i];
	}

	if(++dmard06_adjust.count > DMARD06_TEMP_SIZE){
		axis->x = axis_x/DMARD06_TEMP_SIZE;
		axis->y = axis_y/DMARD06_TEMP_SIZE;
		axis->z = axis_z/DMARD06_TEMP_SIZE;
	}
#endif

	/* Report acceleration sensor information */
    input_report_abs(dmard06->input_dev, ABS_X, axis->x);
    input_report_abs(dmard06->input_dev, ABS_Y, axis->y);
    input_report_abs(dmard06->input_dev, ABS_Z, axis->z);
    input_sync(dmard06->input_dev);
    Sensor_debug("Gsensor x = %d  y = %d z = %d\n", axis->x, axis->y, axis->z);
}

static int dmard06_get_data(struct i2c_client *client)
{
	char buffer[3];
	int ret;
	struct dmard06_axis axis;
//    struct dmard06_platform_data *pdata = client->dev.platform_data;
	
	memset(buffer, 0, 3);
	buffer[0] = DMARD06_REG_T_OUT;
	ret = dmard06_rx_data(client, &buffer[0], 4);
	if (ret < 0){
		printk("read_data_failed\n");
		return ret;
	}
	Sensor_debug("raw data:x=%d,y=%d,z=%d\n",buffer[0],buffer[1],buffer[2]);
#if defined(WISKY_GSENSOR_NY_NX_NZ)
	axis.x = -dmard06_convert_to_int(buffer[1]);
	axis.y = -dmard06_convert_to_int(buffer[2]);
	axis.z = -dmard06_convert_to_int(buffer[3]);
#else
	axis.x = dmard06_convert_to_int(buffer[1]);
	axis.y = dmard06_convert_to_int(buffer[2]);
	axis.z = dmard06_convert_to_int(buffer[3]);
#endif

	/*
	if(pdata->swap_xy)
	{
	axis.y = -axis.y;
	swap(axis.x,axis.y);
	}*/

	dmard06_report_value(client, &axis);

	return 0;
}

/*
static int dmard06_trans_buff(char *rbuf, int size)
{
	//wait_event_interruptible_timeout(rd06_data_ready_wq,
	//				 atomic_read(&data_ready), 1000);
	wait_event_interruptible(rd06_data_ready_wq,
					 atomic_read(&data_ready));

	atomic_set(&data_ready, 0);
	memcpy(rbuf, &sense_data[0], size);

	return 0;
}
*/

static int dmard06_open(struct inode *inode, struct file *file)
{
	return 0;//nonseekable_open(inode, file);
}

static int dmard06_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long dmard06_ioctl( struct file *file, unsigned int cmd,unsigned long arg)
{

	void __user *argp = (void __user *)arg;
//	char msg[RBUFF_SIZE + 1];
	int ret = -1;
	char rate;
	struct i2c_client *client = container_of(dmard06_device.parent, struct i2c_client, dev);

	switch (cmd) {
	case GSENSOR_IOCTL_SET_RATE:
		if (copy_from_user(&rate, argp, sizeof(rate)))
			return -EFAULT;
		break;
	default:
		break;
	}

	switch (cmd) {
	case GSENSOR_IOCTL_START:
		WPRINTK("%s-%d: start\n", __FUNCTION__, __LINE__);
		ret = dmard06_start(client, 9);
		//if (ret < 0)
		//	return ret;
		break;
	case GSENSOR_IOCTL_CLOSE:
		WPRINTK("%s-%d: close\n", __FUNCTION__, __LINE__);
		ret = dmard06_close(client);
		if (ret < 0)
			return ret;
		break;
	case GSENSOR_IOCTL_SET_RATE:
		WPRINTK("%s-%d: set rate = %d\n", __FUNCTION__, __LINE__, rate);
		ret = dmard06_reset_rate(client, rate);
		if (ret < 0)
			return ret;
		break;
    /*
	case GSENSOR_IOCTL_GETDATA:
		ret = dmard06_trans_buff(msg, RBUFF_SIZE);
		if (ret < 0)
			return ret;
		break;
	*/	
	default:
		return -ENOTTY;
	}
/*
	switch (cmd) {
	case GSENSOR_IOCTL_GETDATA:
		WPRINTK("%s-%d: get data\n", __FUNCTION__, __LINE__);
		if (copy_to_user(argp, &msg, sizeof(msg)))
			return -EFAULT;
		break;
	default:
		break;
	}*/

	return 0;
}

#if 0
static void dmard06_work_func(struct work_struct *work)
{
	struct dmard06_data *dmard06 = container_of(work, struct dmard06_data, work);
	struct i2c_client *client = dmard06->client;

	if (dmard06_get_data(client) < 0) 
		WPRINTK(KERN_ERR "DMARD06 mma_work_func: Get data failed\n");
		
	enable_irq(client->irq);		
}
#endif

static void  dmard06_delaywork_func(struct work_struct *work)
{
	struct delayed_work *delaywork = container_of(work, struct delayed_work, work);
	struct dmard06_data *dmard06 = container_of(delaywork, struct dmard06_data, delaywork);
	struct i2c_client *client = dmard06->client;

	if (dmard06_get_data(client) < 0) 
		WPRINTK(KERN_ERR "DMARD06 mma_work_func: Get data failed\n");
		
	enable_irq(client->irq);
//	printk("dmard06_delaywork_func\n");
}

static irqreturn_t dmard06_interrupt(int irq, void *dev_id)
{
	//printk("dmard06_interrupt\n");
	struct dmard06_data *dmard06 = (struct dmard06_data *)dev_id;

	disable_irq_nosync(irq);
	schedule_delayed_work(&dmard06->delaywork, msecs_to_jiffies(5));
	
	return IRQ_HANDLED;
}

static struct file_operations dmard06_fops = {
	.owner = THIS_MODULE,
	.open = dmard06_open,
	.release = dmard06_release,
	.unlocked_ioctl = dmard06_ioctl,
};

static struct miscdevice dmard06_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = GSENSOR_MISC_DEV_NAME,
	.fops = &dmard06_fops,
};

static int dmard06_remove(struct i2c_client *client)
{
	struct dmard06_data *dmard06 = i2c_get_clientdata(client);
	
    misc_deregister(&dmard06_device);
    input_unregister_device(dmard06->input_dev);
    input_free_device(dmard06->input_dev);
    free_irq(client->irq, dmard06);
    kfree(dmard06); 
#ifdef CONFIG_ANDROID_POWER
    android_unregister_early_suspend(&dmard06_early_suspend);
#endif      
    this_client = NULL;
	return 0;
}

#ifdef CONFIG_ANDROID_POWER
static int dmard06_suspend(android_early_suspend_t *h)
{
	struct i2c_client *client = container_of(dmard06_device.parent, struct i2c_client, dev);
	WPRINTK("Gsensor dmard06 enter suspend\n");
	return dmard06_close_dev(client);
}

static int dmard06_resume(android_early_suspend_t *h)
{
	struct i2c_client *client = container_of(dmard06_device.parent, struct i2c_client, dev);
    struct dmard06_data *dmard06 = (struct dmard06_data *)i2c_get_clientdata(client);
	WPRINTK("Gsensor dmard06 resume!!\n");
	return dmard06_start_dev(dmard06->curr_tate);
}
#else
static int dmard06_suspend(struct i2c_client *client, pm_message_t mesg)
{
	WPRINTK("Gsensor dmard06 enter 2 level  suspend\n");
	return dmard06_close_dev(client);
}
static int dmard06_resume(struct i2c_client *client)
{
	struct dmard06_data *dmard06 = (struct dmard06_data *)i2c_get_clientdata(client);
	WPRINTK("Gsensor dmard06 2 level resume!!\n");
	return dmard06_start_dev(client, dmard06->curr_tate);
}
#endif

static const struct i2c_device_id dmard06_id[] = {
		{"gs_dmard06", 0},
		{ }
};

static struct i2c_driver dmard06_driver = {
	.driver = {
		.name = "gs_dmard06",
	    },
	.id_table 	= dmard06_id,
	.probe		= dmard06_probe,
	.remove		= __devexit_p(dmard06_remove),
#ifndef CONFIG_ANDROID_POWER	
	.suspend = &dmard06_suspend,
	.resume = &dmard06_resume,
#endif	
};


static int dmard06_init_client(struct i2c_client *client)
{
	struct dmard06_data *dmard06;
	int ret;
	dmard06 = i2c_get_clientdata(client);
	WPRINTK("gpio_to_irq(%d) is %d\n",client->irq,gpio_to_irq(client->irq));
	if ( !gpio_is_valid(client->irq)) {
		WPRINTK("+++++++++++gpio_is_invalid\n");
		return -EINVAL;
	}
	
	ret = gpio_request(client->irq, "dmard06_int");
	if (ret) {
		WPRINTK( "failed to request dmard06_trig GPIO%d\n",gpio_to_irq(client->irq));
		return ret;
	}
	
	ret = gpio_direction_input(client->irq);
	if (ret) {
		WPRINTK("failed to set dmard06_trig GPIO gpio input\n");
		return ret;
	}

	
	gpio_pull_updown(client->irq, GPIOPullUp);
	client->irq = gpio_to_irq(client->irq);
	ret = request_irq(client->irq, dmard06_interrupt, IRQF_TRIGGER_HIGH, client->dev.driver->name, dmard06);
	WPRINTK("request irq is %d,ret is  0x%x\n",client->irq,ret);
	if (ret ) {
		WPRINTK(KERN_ERR "dmard06_init_client: request irq failed,ret is %d\n",ret);
        return ret;
	}
	
	disable_irq(client->irq);
	init_waitqueue_head(&rd06_data_ready_wq);
 
	return 0;
}

static int  dmard06_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct dmard06_data *dmard06;
	int err;

	dmard06 = kzalloc(sizeof(struct dmard06_data), GFP_KERNEL);
	if (!dmard06) {
		WPRINTK("[dmard06]:alloc data failed.\n");
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
    
//	INIT_WORK(&dmard06->work, dmard06_work_func);
	INIT_DELAYED_WORK(&dmard06->delaywork, dmard06_delaywork_func);

	dmard06->client = client;

	i2c_set_clientdata(client, dmard06);

	this_client = client;

	err = dmard06_test_dev(client, 0);
	if(err < 0){
		pr_err("RD06 probe error: err=%d\n", err);
		goto exit_request_gpio_irq_failed;
	}
	
	err = dmard06_init_client(client);
	if (err < 0) {
		WPRINTK(KERN_ERR
		       "dmard06_probe: dmard06_init_client failed\n");
		goto exit_request_gpio_irq_failed;
	}
		
	dmard06->input_dev = input_allocate_device();
	if (!dmard06->input_dev) {
		err = -ENOMEM;
		WPRINTK(KERN_ERR
		       "dmard06_probe: Failed to allocate input device\n");
		goto exit_input_allocate_device_failed;
	}

	set_bit(EV_ABS, dmard06->input_dev->evbit);

	/* x-axis acceleration */
	input_set_abs_params(dmard06->input_dev, ABS_X, -1500, 1500, 0, 0);
	/* y-axis acceleration */
	input_set_abs_params(dmard06->input_dev, ABS_Y, -1500, 1500, 0, 0);
	/* z-axis acceleration */
	input_set_abs_params(dmard06->input_dev, ABS_Z, -1500, 1500, 0, 0);

	dmard06->input_dev->name = GSENSOR_INPUT_DEV_NAME;
	dmard06->input_dev->dev.parent = &client->dev;

	err = input_register_device(dmard06->input_dev);
	if (err < 0) {
		WPRINTK(KERN_ERR
		       "dmard06_probe: Unable to register input device: %s\n",
		       dmard06->input_dev->name);
		goto exit_input_register_device_failed;
	}

	dmard06_device.parent = &client->dev;
	err = misc_register(&dmard06_device);
	if (err < 0) {
		pr_err("dmard06_probe: misc device register failed\n");
		goto exit_misc_device_register_dmard06_device_failed;
	}

	err = rd06_sysfs_init();
	if (err < 0) {
		pr_err("dmard06_probe: gsensor sysfs init failed\n");
		goto exit_gsensor_sysfs_init_failed;
	}
	
#ifdef CONFIG_ANDROID_POWER
    dmard06_early_suspend.suspend = dmard06_suspend;
    dmard06_early_suspend.resume = dmard06_resume;
    dmard06_early_suspend.level = 0x2;
    android_register_early_suspend(&dmard06_early_suspend);
#endif
	pr_info("dmard06 probe ok\n");
	dmard06->status = -1;	

	return 0;

exit_gsensor_sysfs_init_failed:
    misc_deregister(&dmard06_device);
exit_misc_device_register_dmard06_device_failed:
    input_unregister_device(dmard06->input_dev);
exit_input_register_device_failed:
	input_free_device(dmard06->input_dev);
exit_input_allocate_device_failed:
    free_irq(client->irq, dmard06);
exit_request_gpio_irq_failed:
	kfree(dmard06);	
exit_alloc_data_failed:
    ;
	pr_info("dmard06 probe failed!\n");
	return err;
}


static int __init dmard06_i2c_init(void)
{
	printk("dmard06_i2c_init\n");
	return i2c_add_driver(&dmard06_driver);
}

static void __exit dmard06_i2c_exit(void)
{
	i2c_del_driver(&dmard06_driver);
}

module_init(dmard06_i2c_init);
module_exit(dmard06_i2c_exit);



