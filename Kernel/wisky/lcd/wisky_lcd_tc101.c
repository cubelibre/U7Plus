/****************************************************************************************
 * driver/input/touchscreen/hannstar_p1003.c
 *Copyright 	:ROCKCHIP  Inc
 *Author	: 	sfm
 *Date		:  2010.2.5
 *This driver use for rk28 chip extern touchscreen. Use i2c IF ,the chip is Hannstar
 *description£º
 ********************************************************************************************/
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <mach/gpio.h>
#include <linux/irq.h>
#include <mach/board.h>
#include <linux/miscdevice.h>

#define TC101_DEBUG 0 
#ifdef TC101_DEBUG
	#define tc101_printk(x...) printk(x)
#else
	#define tc101_printk(x...) 
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend tc101_early_suspend;
#endif

//for test pattern
#define test_mode 0

static struct i2c_client *this_client;
int tc101_set(int on);

int  tc101_start(struct i2c_client *client)
{
	char buf1[3] = {0xf8 , 0x30 , 0xb2};//b2
	char buf2[3] = {0xf8 , 0x31 , 0xf0};
	char buf3[3] = {0xf8 , 0x33 , 0xc2};
	char buf4[3] = {0xf8 , 0x40 , 0x80};  
	char buf5[3] = {0xf8 , 0x81 , 0xec};

#if test_mode
	char buf6[3] = {0xf8 , 0x40 , 0x88}; 	//fan xiang 
	char buf7[3] = {0xf8 , 0x50 , 0x00};
	char buf8[3] = {0xf8 , 0x50 , 0x01};
	char buf9[3] = {0xf8 , 0x50 , 0x02};
	char buf10[3] = {0xf8 , 0x50 , 0x03};
	char buf11[3] = {0xf8 , 0x50 , 0x04};
	char buf12[3] = {0xf8 , 0x50 , 0x05};
#endif
    int data;
	
	printk("enter tc101_init!!!\n");
	data = i2c_master_normal_send(client,buf1, 3,100*1000);
	if(data<0)
	{
		printk("tc101 init error!!\n");
		return data;
	}
	//mdelay(120);
	data = i2c_master_normal_send(client,buf2, 3,100*1000);
	if(data<0)
	{
		printk("tc101 init error!!\n");
		return data;

	}
	//mdelay(1);
	data = i2c_master_normal_send(client,buf3, 3,100*1000);
	if(data<0)
	{
		printk("tc101 init error!!\n");
		return data;

	}
	//mdelay(1);
	data = i2c_master_normal_send(client,buf4, 3,100*1000);
	if(data<0)
	{
		printk("tc101 init error!!\n");
		return data;

	}
	//mdelay(1);
	data = i2c_master_normal_send(client,buf5, 3,100*1000);
	if(data<0)
	{
		printk("tc101 init error!!\n");
		return data;

	}
	 
	#if test_mode		
	//mdelay(100);
	data = i2c_master_normal_send(client,buf6, 3,100*1000);
	if(data<0)
	{
		printk("tc101 init error!!\n");
		return data;	
	}
	//mdelay(100);
	data = i2c_master_normal_send(client,buf7, 3,100*1000);
	if(data<0)
	{
		printk("tc101 init error!!\n");
		return data;
	}
	mdelay(2000);
		data = i2c_master_normal_send(client,buf8, 3,100*1000);
	if(data<0)
	{
		printk("tc101 init error!!\n");
		return data;
	}
	mdelay(2000);
		data = i2c_master_normal_send(client,buf9, 3,100*1000);
	if(data<0)
	{
		printk("tc101 init error!!\n");
		return data;
	}
	mdelay(2000);
		data = i2c_master_normal_send(client,buf10, 3,100*1000);
	if(data<0)
	{
		printk("tc101 init error!!\n");
		return data;
	}
	mdelay(2000);
		data = i2c_master_normal_send(client,buf11, 3,100*1000);
	if(data<0)
	{
		printk("tc101 init error!!\n");
		return data;
	}
	mdelay(2000);
		data = i2c_master_normal_send(client,buf12, 3,100*1000);
	if(data<0)
	{
		printk("tc101 init error!!\n");
		return data;
	}
	mdelay(2000);
	#endif
	#if 0
	//mdelay(100);
	data = i2c_master_normal_send(client,buf8, 3,100*1000);
	if(data<0)
	{
		printk("tc101 init error!!\n");
		return data;
	}
	#endif
	
	// char buf8[1];
	// data = i2c_master_reg8_recv(client,0xf884,buf8,1,100*1000);
	// printk("tc101 init 0xF857 is %x",buf8[0]);//50
	tc101_printk("init tc101 ok\n");
	return 0;

}


#ifdef CONFIG_HAS_EARLYSUSPEND
	 static void tc101_suspend(struct early_suspend *h)
	 {
		tc101_set(0);
		tc101_printk(" tc101_suspend\n");
	 }
	 
	 static void tc101_resume(struct early_suspend *h)
	 {
		//tc101_start(this_client);
		tc101_set(1);
		tc101_printk(" tc101_resume\n");
	 }
#endif 
static int __devinit tc101_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	printk("%s\n",__FUNCTION__);
	
    tc101_start(client);
 	this_client = client; 

	
	#ifdef CONFIG_HAS_EARLYSUSPEND
	    tc101_early_suspend.suspend = tc101_suspend;
	    tc101_early_suspend.resume = tc101_resume;
	    tc101_early_suspend.level = 0x2;
	    register_early_suspend(&tc101_early_suspend);
	#endif
	return 0; //libing return 0;->add
	
}

static int __devexit tc101_remove(struct i2c_client *client)
{
	#ifdef CONFIG_HAS_EARLYSUSPEND
    	unregister_early_suspend(&tc101_early_suspend);
	#endif 
	return 0;
}


static struct i2c_device_id tc101_idtable[] = {
	{ "lvds_tc101", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tc101_idtable);

static struct i2c_driver tc101_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "lvds_tc101"
	},
	.id_table	= tc101_idtable,
	.probe		= tc101_probe,
	.remove		= __devexit_p(tc101_remove),
};

int tc101_set(int on)
{
	int data;

	if(on==1)
	{
		tc101_printk(" tc101_set-----------open-----------\n");
		char buf1[3] = {0xf8 , 0x30 , 0xb2};
		data = i2c_master_normal_send(this_client,buf1, 3,100*1000);
		
		if(data<0)
		{
			printk("tc101 init error!!\n");
			return data;
		}
	}
	else
	{
		tc101_printk(" tc101_set1 ----------------remove------\n");
		char buf1[3] = {0xf8 , 0x30 , 0xb3};
		data = i2c_master_normal_send(this_client,buf1, 3,100*1000);
		
		if(data<0)
		{
			printk("tc101 init error!!\n");
			return data;
		}
	}

#if 0
	if(on==1)
	{
		#ifdef CONFIG_HAS_EARLYSUSPEND
		if(test != 1)
		{
			register_early_suspend(&tc101_early_suspend);
		}
		#endif
		tc101_printk(" tc101_set-----------open-----------\n");
	}
	else
	{
		#ifdef CONFIG_HAS_EARLYSUSPEND
		if(test != 1)
		{
			unregister_early_suspend(&tc101_early_suspend);
		}
		#endif
		tc101_printk(" tc101_set1 ----------------remove------\n");
	}
	test= test+1;
#endif
	return data;
}

static void __init tc101_init_async(void *unused, async_cookie_t cookie)
{
	printk("--------> %s <-------------\n",__func__);
	i2c_add_driver(&tc101_driver);
}

 int __init tc101_init(void) // int __init tc101_init(void)->static int __init tc101_init(void)
{
	async_schedule(tc101_init_async, NULL);

	return 0;
}

 void __exit tc101_exit(void) //static void __exit tc101_exit(void) -> void __exit tc101_exit(void)
{
	return i2c_del_driver(&tc101_driver);
}
fs_initcall(tc101_init);
module_exit(tc101_exit);
MODULE_LICENSE("GPL");


