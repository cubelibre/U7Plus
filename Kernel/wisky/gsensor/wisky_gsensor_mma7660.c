/* wisky/gsensor/wisky_gsensor_mma7660.c
 *	mma7660 accelerometer driver
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
 * V001:20110226 cd huang
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
#include "wisky_gsensor_mma7660.h"
#include <linux/earlysuspend.h>

static int  mma7660_probe(struct i2c_client *client, const struct i2c_device_id *id);

#define MMA7660_SPEED		(400 * 1000)

#define MMA7660_DATA_FILTER	1

#define MMA7660_DEFAULT_RATE	32
//#if defined(WISKY_BOARD_M818PMU_V60)
//#define MMA7660_MAX_RATE	120//64
//#else
#define MMA7660_MAX_RATE	64//64
//#endif


#ifdef MMA7660_DATA_FILTER
struct mma7660_data_tag{
	#define MMA7660_TEMP_SIZE  3//15	//sample times
	int count;
	int axis_x[MMA7660_TEMP_SIZE];
	int axis_y[MMA7660_TEMP_SIZE];
	int axis_z[MMA7660_TEMP_SIZE];
};
static struct mma7660_data_tag mma7660_adjust;
#endif

/* Addresses to scan -- protected by sense_data_mutex */
//static char sense_data[RBUFF_SIZE + 1];
static struct i2c_client *this_client;
static struct miscdevice mma7660_device;


static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
#if 0
#ifdef CONFIG_ANDROID_POWER
static android_early_suspend_t mma7660_early_suspend;
#endif
#endif
static int revision = -1;
/* AKM HW info */
static ssize_t gsensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%#x\n", revision);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(vendor, 0444, gsensor_vendor_show, NULL);

static struct kobject *android_gsensor_kobj;

static int gsensor_sysfs_init(void)
{
	int ret ;

	android_gsensor_kobj = kobject_create_and_add("android_gsensor", NULL);
	if (android_gsensor_kobj == NULL) {
		printk(KERN_ERR
		       "MMA7660 gsensor_sysfs_init:"
		       "subsystem_register failed\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = sysfs_create_file(android_gsensor_kobj, &dev_attr_vendor.attr);
	if (ret) {
		printk(KERN_ERR
		       "MMA7660 gsensor_sysfs_init:"
		       "sysfs_create_group failed\n");
		goto err4;
	}

	return 0 ;
err4:
	kobject_del(android_gsensor_kobj);
err:
	return ret ;
}

static int mma7660_rx_data(struct i2c_client *client, char *rxData, int length)
{
	int ret = 0;
	char reg = rxData[0];
	ret = i2c_master_reg8_recv(client, reg, rxData, length, MMA7660_SPEED);
	return (ret > 0)? 0 : ret;
}

static int mma7660_tx_data(struct i2c_client *client, char *txData, int length)
{
	int ret = 0;
	char reg = txData[0];
	ret = i2c_master_reg8_send(client, reg, &txData[1], length-1, MMA7660_SPEED);
	return (ret > 0)? 0 : ret;
}

static int mma7660_set_rate(struct i2c_client *client, char rate)
{
	struct mma7660_data *mma7660 = (struct mma7660_data *)i2c_get_clientdata(client);
	char buffer[2];
	int ret = 0;
	int i;
	
	//if (rate > 128)
	//	return -EINVAL;

	rate = MMA7660_MAX_RATE;
	
	//williamdeng wisky 20120424 GSENSOR��Ƶ�ʶ������������������޸��µĶ������㷽ʽ
	
	/*for (i = 0; i < 7; i++) {
		if (rate & (0x1 << i))
			break;
	}*/
	switch(rate)
	{
	case 1:  i=7;break;
	case 2:  i=6;break;
	case 4:  i=5;break;
	case 8:  i=4;break;
	case 16: i=3;break;
	case 32: i=2;break;
	case 64: i=1;break;
	case 120:i=0;break;
	default: i=1;break;//set default to 64 samples/second
	}
	//end williamdeng wisky 20120424
	
	buffer[0] = MMA7660_REG_SR;
	buffer[1] = 0xf8 | (0x07 & (i));

	ret = mma7660_tx_data(client, &(buffer[0]), 2);
	ret = mma7660_rx_data(client, &(buffer[0]), 1);

	mma7660->curr_tate = rate;
		
	return ret;
}

static int mma7660_start_dev(struct i2c_client *client, char rate)
{
	char buffer[MMA7660_REG_LEN];
	int ret = 0;

	buffer[0] = MMA7660_REG_INTSU;
	buffer[1] = 0x10;	//0x10; modify by zhao
	ret = mma7660_tx_data(client, &buffer[0], 2);
	ret = mma7660_rx_data(client, &buffer[0], 1);

	ret = mma7660_set_rate(client, rate);

	buffer[0] = MMA7660_REG_MODE;
	buffer[1] = 0x01;
	ret = mma7660_tx_data(client, &buffer[0], 2);
	ret = mma7660_rx_data(client, &buffer[0], 1);

	//enable_irq(client->irq);

#ifdef MMA7660_DATA_FILTER
	mma7660_adjust.count = 0;
#endif

	return ret;
}

static int mma7660_start(struct i2c_client *client, char rate)
{ 
    struct mma7660_data *mma7660 = (struct mma7660_data *)i2c_get_clientdata(client);
    
    if (mma7660->status == MMA7660_OPEN) {
        return 0;      
    }
    mma7660->status = MMA7660_OPEN;
    return mma7660_start_dev(client, rate);
}

static int mma7660_close_dev(struct i2c_client *client)
{    	
	char buffer[2];

	//disable_irq_nosync(client->irq);

	buffer[0] = MMA7660_REG_MODE;
	buffer[1] = 0x00;
	
	return mma7660_tx_data(client, buffer, 2);
}

static int mma7660_close(struct i2c_client *client)
{
    struct mma7660_data *mma7660 = (struct mma7660_data *)i2c_get_clientdata(client);
    
    mma7660->status = MMA7660_CLOSE;
    
    return mma7660_close_dev(client);
}

/*
*ϵͳʹ�õĹ�����������:
*	 FASTEST(0), GAME(20), UI(60), NORMAL(200)
*/
static int mma7660_reset_rate(struct i2c_client *client, char rate)
{
	int ret = 0;

	ret = mma7660_close_dev(client);
	ret = mma7660_start_dev(client, rate);
    
	return ret ;
}

static inline int mma7660_convert_to_int(char value)
{
	int result;

	if (value < MMA7660_BOUNDARY) {
		result = value * MMA7660_GRAVITY_STEP;
	} else {
		result = ~(((~value & 0x3f) + 1)* MMA7660_GRAVITY_STEP) + 1;
	}
	return result;
}

static void mma7660_report_value(struct i2c_client *client, struct mma7660_axis *axis)
{
	struct mma7660_data *mma7660 = i2c_get_clientdata(client);
	//struct mma7660_axis *axis = (struct mma7660_axis *)rbuf;
#ifdef MMA7660_DATA_FILTER
	int i = 0, axis_x=0, axis_y=0, axis_z=0;
#endif

#ifdef MMA7660_DATA_FILTER	
	for(i = 0; i < MMA7660_TEMP_SIZE; i++){
		if((MMA7660_TEMP_SIZE-1) == i){
			mma7660_adjust.axis_x[i] = axis->x;
			mma7660_adjust.axis_y[i] = axis->y;
			mma7660_adjust.axis_z[i] = axis->z;
		}else{
			mma7660_adjust.axis_x[i] = mma7660_adjust.axis_x[i+1];
			mma7660_adjust.axis_y[i] = mma7660_adjust.axis_y[i+1];
			mma7660_adjust.axis_z[i] = mma7660_adjust.axis_z[i+1];
		}
	}
	for(i = 0; i < MMA7660_TEMP_SIZE; i++){
		axis_x += mma7660_adjust.axis_x[i];
		axis_y += mma7660_adjust.axis_y[i];
		axis_z += mma7660_adjust.axis_z[i];
	}

	if(++mma7660_adjust.count > MMA7660_TEMP_SIZE){
		axis->x = axis_x/MMA7660_TEMP_SIZE;
		axis->y = axis_y/MMA7660_TEMP_SIZE;
		axis->z = axis_z/MMA7660_TEMP_SIZE;
	}
#endif

	/* Report acceleration sensor information */
	input_report_abs(mma7660->input_dev, ABS_X, axis->x);
	input_report_abs(mma7660->input_dev, ABS_Y, axis->y);
	input_report_abs(mma7660->input_dev, ABS_Z, axis->z);
	input_sync(mma7660->input_dev);
	WPRINTK("%s,x=%d,y=%d,z=%d\n",__func__,axis->x,axis->y,axis->z);
}

static int mma7660_get_data(struct i2c_client *client)
{
	char buffer[3];
	int ret;
	struct mma7660_axis axis;
//    struct mma7660_platform_data *pdata = client->dev.platform_data;
	
	do {
		memset(buffer, 0, 3);
		buffer[0] = MMA7660_REG_X_OUT;
		ret = mma7660_rx_data(client, &buffer[0], 3);
		if (ret < 0)
			return ret;
	} while ((buffer[0] & 0x40) || (buffer[1] & 0x40) || (buffer[2] & 0x40));
	WPRINTK("raw x=%d,y=%d,z=%d\n",buffer[0],buffer[1],buffer[2]);
#if defined(WISKY_GSENSOR_NX_PY_PZ)
	axis.x = -mma7660_convert_to_int(buffer[MMA7660_REG_X_OUT]);
	axis.y = mma7660_convert_to_int(buffer[MMA7660_REG_Y_OUT]);
	axis.z = mma7660_convert_to_int(buffer[MMA7660_REG_Z_OUT]);
#elif defined(WISKY_GSENSOR_NX_PZ_PY)
    axis.x = mma7660_convert_to_int(buffer[MMA7660_REG_Y_OUT]);
    axis.y = -mma7660_convert_to_int(buffer[MMA7660_REG_Z_OUT]);
    axis.z = mma7660_convert_to_int(buffer[MMA7660_REG_X_OUT]);
#elif defined(WISKY_GSENSOR_NX_PY_NZ)
	axis.x = -mma7660_convert_to_int(buffer[MMA7660_REG_X_OUT]);
	axis.y = mma7660_convert_to_int(buffer[MMA7660_REG_Y_OUT]);
	axis.z = -mma7660_convert_to_int(buffer[MMA7660_REG_Z_OUT]);
#elif defined(WISKY_GSENSOR_PX_PY_PZ)
	axis.x = mma7660_convert_to_int(buffer[MMA7660_REG_X_OUT]);
	axis.y = mma7660_convert_to_int(buffer[MMA7660_REG_Y_OUT]);
	axis.z = mma7660_convert_to_int(buffer[MMA7660_REG_Z_OUT]);
#elif defined(WISKY_GSENSOR_NX_NY_PZ)
	axis.x = -mma7660_convert_to_int(buffer[MMA7660_REG_X_OUT]);
	axis.y = -mma7660_convert_to_int(buffer[MMA7660_REG_Y_OUT]);
	axis.z = mma7660_convert_to_int(buffer[MMA7660_REG_Z_OUT]);
#elif defined(WISKY_GSENSOR_NX_NY_NZ)
	axis.x = -mma7660_convert_to_int(buffer[MMA7660_REG_X_OUT]);
	axis.y = -mma7660_convert_to_int(buffer[MMA7660_REG_Y_OUT]);
	axis.z = -mma7660_convert_to_int(buffer[MMA7660_REG_Z_OUT]);
#elif defined(WISKY_GSENSOR_NY_NX_NZ)
	axis.x = -mma7660_convert_to_int(buffer[MMA7660_REG_Y_OUT]);
	axis.y = -mma7660_convert_to_int(buffer[MMA7660_REG_X_OUT]);
	axis.z = -mma7660_convert_to_int(buffer[MMA7660_REG_Z_OUT]);
#elif defined(WISKY_GSENSOR_PY_PX_NZ)
	axis.x = mma7660_convert_to_int(buffer[MMA7660_REG_Y_OUT]);
	axis.y = mma7660_convert_to_int(buffer[MMA7660_REG_X_OUT]);
	axis.z = -mma7660_convert_to_int(buffer[MMA7660_REG_Z_OUT]);
#elif defined(WISKY_GSENSOR_PY_PX_PZ)
	axis.x = mma7660_convert_to_int(buffer[MMA7660_REG_Y_OUT]);
	axis.y = mma7660_convert_to_int(buffer[MMA7660_REG_X_OUT]);
	axis.z = mma7660_convert_to_int(buffer[MMA7660_REG_Z_OUT]);	
#elif defined(WISKY_GSENSOR_NY_NX_PZ)
	axis.x = -mma7660_convert_to_int(buffer[MMA7660_REG_Y_OUT]);
	axis.y = -mma7660_convert_to_int(buffer[MMA7660_REG_X_OUT]);
	axis.z = mma7660_convert_to_int(buffer[MMA7660_REG_Z_OUT]); 
#elif defined(WISKY_GSENSOR_PX_NY_PZ)
	axis.x = mma7660_convert_to_int(buffer[MMA7660_REG_X_OUT]);
	axis.y = -mma7660_convert_to_int(buffer[MMA7660_REG_Y_OUT]);
	axis.z = mma7660_convert_to_int(buffer[MMA7660_REG_Z_OUT]);
#elif defined(WISKY_GSENSOR_PY_NX_PZ)
	axis.x = mma7660_convert_to_int(buffer[MMA7660_REG_Y_OUT]);
	axis.y = -mma7660_convert_to_int(buffer[MMA7660_REG_X_OUT]);	
	axis.z = mma7660_convert_to_int(buffer[MMA7660_REG_Z_OUT]);
#elif defined(WISKY_GSENSOR_PX_NY_NZ)
	axis.x = mma7660_convert_to_int(buffer[MMA7660_REG_X_OUT]);
	axis.y = -mma7660_convert_to_int(buffer[MMA7660_REG_Y_OUT]);
	axis.z = -mma7660_convert_to_int(buffer[MMA7660_REG_Z_OUT]);
#else
	axis.x = mma7660_convert_to_int(buffer[MMA7660_REG_Y_OUT]);
	axis.y = mma7660_convert_to_int(buffer[MMA7660_REG_X_OUT]);
	axis.z = mma7660_convert_to_int(buffer[MMA7660_REG_Z_OUT]);
#endif

	#ifdef MMA7660_DATA_FILTER
	axis.x += GSENSOR_ADJUST_X;
	axis.y += GSENSOR_ADJUST_Y;
	axis.z += GSENSOR_ADJUST_Z;
	#endif
	WPRINTK("x=%d,y=%d,z=%d\n",axis.x,axis.y,axis.z);
	mma7660_report_value(client, &axis);

	return 0;
}

/*
static int mma7660_trans_buff(char *rbuf, int size)
{
	//wait_event_interruptible_timeout(data_ready_wq,
	//				 atomic_read(&data_ready), 1000);
	wait_event_interruptible(data_ready_wq,
					 atomic_read(&data_ready));

	atomic_set(&data_ready, 0);
	memcpy(rbuf, &sense_data[0], size);

	return 0;
}
*/

static int mma7660_open(struct inode *inode, struct file *file)
{
	return 0;//nonseekable_open(inode, file);
}

static int mma7660_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long mma7660_ioctl( struct file *file, unsigned int cmd,unsigned long arg)
{

	void __user *argp = (void __user *)arg;
//	char msg[RBUFF_SIZE + 1];
	int ret = -1;
	char rate;
	struct i2c_client *client = container_of(mma7660_device.parent, struct i2c_client, dev);

	WPRINTK("%s-%d: cmd = %d\n", __FUNCTION__, __LINE__, cmd);
	switch (cmd) {
	case MMA_IOCTL_APP_SET_RATE:
		if (copy_from_user(&rate, argp, sizeof(rate)))
			return -EFAULT;
		break;
	default:
		break;
	}

	switch (cmd) {
	case MMA_IOCTL_START:
		WPRINTK("%s-%d: start\n", __FUNCTION__, __LINE__);
		ret = mma7660_start(client, MMA7660_RATE_32);
		if (ret < 0)
			return ret;
		break;
	case MMA_IOCTL_CLOSE:
		WPRINTK("%s-%d: close\n", __FUNCTION__, __LINE__);
		ret = mma7660_close(client);
		if (ret < 0)
			return ret;
		break;
	case MMA_IOCTL_APP_SET_RATE:
		WPRINTK("%s-%d: set rate = %d\n", __FUNCTION__, __LINE__, rate);
		ret = mma7660_reset_rate(client, rate);
		if (ret < 0)
			return ret;
		break;
    /*
	case MMA_IOCTL_GETDATA:
		ret = mma7660_trans_buff(msg, RBUFF_SIZE);
		if (ret < 0)
			return ret;
		break;
	*/	
	default:
		return -ENOTTY;
	}
/*
	switch (cmd) {
	case MMA_IOCTL_GETDATA:
		WPRINTK("%s-%d: get data\n", __FUNCTION__, __LINE__);
		if (copy_to_user(argp, &msg, sizeof(msg)))
			return -EFAULT;
		break;
	default:
		break;
	}*/

	return 0;
}

static void mma7660_work_func(struct work_struct *work)
{
	struct mma7660_data *mma7660 = container_of(work, struct mma7660_data, work);
	struct i2c_client *client = mma7660->client;

	if (mma7660_get_data(client) < 0) 
		WPRINTK(KERN_ERR "MMA7660 mma_work_func: Get data failed\n");
		
	//enable_irq(client->irq);		
}

static void  mma7660_delaywork_func(struct work_struct *work)
{
	struct delayed_work *delaywork = container_of(work, struct delayed_work, work);
	struct mma7660_data *mma7660 = container_of(delaywork, struct mma7660_data, delaywork);
	struct i2c_client *client = mma7660->client;

	if (mma7660_get_data(client) < 0) 
		WPRINTK(KERN_ERR "MMA7660 mma_work_func: Get data failed\n");
		
	enable_irq(client->irq);		
}

static enum hrtimer_restart mma7660_timer_func(struct hrtimer *timer)
{
	struct mma7660_data *mma7660 = container_of(timer, struct mma7660_data, timer);
	queue_work(mma7660->mma7660_wq, &mma7660->work);
	hrtimer_start(&mma7660->timer, ktime_set(0, 20*1000000), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

#if 0
static irqreturn_t mma7660_interrupt(int irq, void *dev_id)
{
	struct mma7660_data *mma7660 = (struct mma7660_data *)dev_id;

	disable_irq_nosync(irq);
	schedule_delayed_work(&mma7660->delaywork, msecs_to_jiffies(5));//30
	
	return IRQ_HANDLED;
}
#endif

static struct file_operations mma7660_fops = {
	.owner = THIS_MODULE,
	.open = mma7660_open,
	.release = mma7660_release,
	.unlocked_ioctl = mma7660_ioctl,
};

static struct miscdevice mma7660_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mma8452_daemon",
	.fops = &mma7660_fops,
};


#ifdef CONFIG_HAS_EARLYSUSPEND
static int mma7660_early_suspend(struct early_suspend *h)
{
	struct i2c_client *client = container_of(mma7660_device.parent, struct i2c_client, dev);
	WPRINTK("Gsensor mma7760 enter early suspend\n");
	struct mma7660_data *mma7660  =  i2c_get_clientdata(client);
	hrtimer_cancel(&mma7660->timer);
	return mma7660_close_dev(client);
}

static void mma7660_late_resume(struct early_suspend *h)
{
	int ret = 0;
	struct i2c_client *client = container_of(mma7660_device.parent, struct i2c_client, dev);
    struct mma7660_data *mma7660 = (struct mma7660_data *)i2c_get_clientdata(client);
	WPRINTK("Gsensor mma7760 resume!!\n");
	ret = mma7660_start_dev(client, mma7660->curr_tate);
	hrtimer_start(&mma7660->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	return  ret ;
}
#else
static int mma7660_suspend(struct i2c_client *client, pm_message_t mesg)
{
	WPRINTK("Gsensor mma7760 enter 2 level  suspend\n");
	struct mma7660_data *mma7660  =  i2c_get_clientdata(client);
	hrtimer_cancel(&mma7660->timer);
	return mma7660_close_dev(client);
	
}
static int mma7660_resume(struct i2c_client *client)
{
	int ret = 0;
	struct mma7660_data *mma7660 = (struct mma7660_data *)i2c_get_clientdata(client);
	WPRINTK("Gsensor mma7760 2 level resume!!\n");
	ret = mma7660_start_dev(client, mma7660->curr_tate);
	hrtimer_start(&mma7660->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	return ret;
}
#endif

static int mma7660_remove(struct i2c_client *client)
{
	struct mma7660_data *mma7660 = i2c_get_clientdata(client);
	
hrtimer_cancel(&mma7660->timer);
    misc_deregister(&mma7660_device);
    input_unregister_device(mma7660->input_dev);
    input_free_device(mma7660->input_dev);
    free_irq(client->irq, mma7660);
    kfree(mma7660); 
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&mma7660_early_suspend);
#endif      
    this_client = NULL;
	return 0;
}


static const struct i2c_device_id mma7660_id[] = {
		{"gs_mma7660", 0},
		{ }
};

static struct i2c_driver mma7660_driver = {
	.driver = {
		.name = "gs_mma7660",
	    },
	.id_table 	= mma7660_id,
	.probe		= mma7660_probe,
	.remove		= __devexit_p(mma7660_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND	
	.suspend = &mma7660_suspend,
	.resume = &mma7660_resume,
#endif	
};


#if 0
static int mma7660_init_client(struct i2c_client *client)
{
	struct mma7660_data *mma7660;
	int ret;
	mma7660 = i2c_get_clientdata(client);
	
	WPRINTK("gpio_to_irq(%d) is %d\n",client->irq,gpio_to_irq(client->irq));
	if ( !gpio_is_valid(client->irq)) {
		WPRINTK("+++++++++++gpio_is_invalid\n");
		return -EINVAL;
	}
	
	ret = gpio_request(client->irq, "mma7660_int");
	if (ret) {
		WPRINTK( "failed to request mma7990_trig GPIO%d\n",gpio_to_irq(client->irq));
		return ret;
	}
	
	ret = gpio_direction_input(client->irq);
	if (ret) {
		WPRINTK("failed to set mma7990_trig GPIO gpio input\n");
		return ret;
	}
	
	gpio_pull_updown(client->irq, GPIOPullUp);
	client->irq = gpio_to_irq(client->irq);
	ret = request_irq(client->irq, mma7660_interrupt, IRQF_TRIGGER_LOW, client->dev.driver->name, mma7660);
	WPRINTK("request irq is %d,ret is  0x%x\n",client->irq,ret);
	if (ret ) {
		WPRINTK(KERN_ERR "mma7660_init_client: request irq failed,ret is %d\n",ret);
        return ret;
	}
	
	disable_irq(client->irq);

	init_waitqueue_head(&data_ready_wq);

 
	return 0;
}
	#endif
static int  mma7660_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mma7660_data *mma7660;
	int err;
	
	mma7660 = kzalloc(sizeof(struct mma7660_data), GFP_KERNEL);
	if (!mma7660) {
		WPRINTK("[mma7660]:alloc data failed.\n");
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
    
	mma7660->mma7660_wq = create_singlethread_workqueue("mma7660_wq");
	if (!mma7660->mma7660_wq )
	{
		err = -ENOMEM;
		goto err_create_workqueue_failed;
	}
	INIT_WORK(&mma7660->work, mma7660_work_func);
//	INIT_DELAYED_WORK(&mma7660->delaywork, mma7660_delaywork_func);

	mma7660->client = client;

	i2c_set_clientdata(client, mma7660);

	this_client = client;

/*
	err = mma7660_init_client(client);
	if (err < 0) {
		WPRINTK(KERN_ERR
		       "mma7660_probe: mma7660_init_client failed\n");
		goto exit_request_gpio_irq_failed;
	}
*/		
	mma7660->input_dev = input_allocate_device();
	if (!mma7660->input_dev) {
		err = -ENOMEM;
		WPRINTK(KERN_ERR
		       "mma7660_probe: Failed to allocate input device\n");
		goto exit_input_allocate_device_failed;
	}

	set_bit(EV_ABS, mma7660->input_dev->evbit);

	/* x-axis acceleration */
	input_set_abs_params(mma7660->input_dev, ABS_X, -MMA7660_RANGE, MMA7660_RANGE, 0, 0);
	/* y-axis acceleration */
	input_set_abs_params(mma7660->input_dev, ABS_Y, -MMA7660_RANGE, MMA7660_RANGE, 0, 0);
	/* z-axis acceleration */
	input_set_abs_params(mma7660->input_dev, ABS_Z, -MMA7660_RANGE, MMA7660_RANGE, 0, 0);

	mma7660->input_dev->name = "gsensor";
	mma7660->input_dev->dev.parent = &client->dev;

	err = input_register_device(mma7660->input_dev);
	if (err < 0) {
		WPRINTK(KERN_ERR
		       "mma7660_probe: Unable to register input device: %s\n",
		       mma7660->input_dev->name);
		goto exit_input_register_device_failed;
	}

	mma7660_device.parent = &client->dev;
	err = misc_register(&mma7660_device);
	if (err < 0) {
		pr_err("mma7660_probe: misc device register failed\n");
		goto exit_misc_device_register_mma7660_device_failed;
	}

	err = gsensor_sysfs_init();
	if (err < 0) {
		pr_err("mma7660_probe: gsensor sysfs init failed\n");
		goto exit_gsensor_sysfs_init_failed;
	}
	
	hrtimer_init(&mma7660->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mma7660->timer.function = mma7660_timer_func;
	hrtimer_start(&mma7660->timer, ktime_set(10, 0), HRTIMER_MODE_REL);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	mma7660->early_suspend.level = 50 + 1;
	mma7660->early_suspend.suspend = mma7660_early_suspend;
	mma7660->early_suspend.resume = mma7660_late_resume;
	register_early_suspend(&mma7660->early_suspend);
#endif

	pr_info("mma7660 probe ok\n");
	mma7660->status = -1;
	
#if 0	
	mma7660_start(client, MMA7660_RATE_32);
#endif

	return 0;

exit_gsensor_sysfs_init_failed:
    misc_deregister(&mma7660_device);
exit_misc_device_register_mma7660_device_failed:
    input_unregister_device(mma7660->input_dev);
exit_input_register_device_failed:
	input_free_device(mma7660->input_dev);
exit_input_allocate_device_failed:
	destroy_workqueue(mma7660->mma7660_wq);	
    //free_irq(client->irq, mma7660);
//exit_request_gpio_irq_failed:
//	kfree(mma7660);	
err_create_workqueue_failed:
	kfree(mma7660);	
exit_alloc_data_failed:
	pr_info("mma7660 probe failed!\n");
	return err;
}


static int __init mma7660_i2c_init(void)
{
	return i2c_add_driver(&mma7660_driver);
}

static void __exit mma7660_i2c_exit(void)
{
	i2c_del_driver(&mma7660_driver);
}

module_init(mma7660_i2c_init);
module_exit(mma7660_i2c_exit);



