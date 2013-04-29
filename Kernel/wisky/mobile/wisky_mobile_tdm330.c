#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/circ_buf.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <mach/iomux.h>
#include <mach/gpio.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/tdm330.h>
MODULE_LICENSE("GPL");

#define DEBUG
#ifdef DEBUG
#define MODEMDBG(x...) printk(x)
#else
#define MODEMDBG(fmt,argss...)
#endif
#define SLEEP 1
#define READY 0
#define IRQ_BB_WAKEUP_AP_TRIGGER    IRQF_TRIGGER_FALLING
#define TDM330_RESET 0x01
#define WRITE_TIMEOUT 60
struct rk29_tdm330_data *gpdata = NULL;
struct class *modem_class = NULL; 
static int do_wakeup_irq = 0;
static int modem_status;
static int wifi_ap_status;

static struct wake_lock reset_wake_lock;

static void set_ap_wakeup_bp(int value)
{
	struct rk29_tdm330_data *pdata = gpdata;

	if(pdata->ap_wakeup_bp > 0)
		gpio_set_value(pdata->ap_wakeup_bp,value);
 }

static irqreturn_t tdm330_irq(int irq, void *dev_id)
{
	struct rk29_tdm330_data *pdata = (struct rk29_tdm330_data *)dev_id;

	set_ap_wakeup_bp(GPIO_LOW);
	pdata->resume_finish =1;
	wake_up(&pdata->wait_resume);
	MODEMDBG("tdm330_iqr\n");
	return IRQ_HANDLED;
}

static void set_bp_power(int value)
{
	struct rk29_tdm330_data *pdata = gpdata;

	if(pdata->bp_power>0)
		gpio_set_value(pdata->bp_power,value);
}
static void set_bp_on(int value)
{
	struct rk29_tdm330_data *pdata = gpdata;

	if(pdata->bp_on>0)
		gpio_set_value(pdata->bp_on,value);
}
static void set_bp_reset(int value)
{
	struct rk29_tdm330_data *pdata = gpdata;

	if(pdata->bp_reset>0)
		gpio_set_value(pdata->bp_reset,value);
}
static void set_output(int pin,int value)
{
	struct rk29_tdm330_data *pdata = gpdata;

	if(pin > 0)
		gpio_direction_output(pin,value);
}
static void set_input(int pin)
{
	struct rk29_tdm330_data *pdata = gpdata;
	if(pin > 0)
		gpio_direction_input(pin);
}
static void do_wakeup(void)
{
	int i;
	int ret;
	struct rk29_tdm330_data *pdata = gpdata;

	MODEMDBG("do_wakeup bp_statue fisrt level=%d\n",gpio_get_value(pdata->bp_statue));
	set_ap_wakeup_bp(GPIO_HIGH);
	ret = wait_event_timeout(pdata->wait_resume,
			(pdata->resume_finish),
			 msecs_to_jiffies(WRITE_TIMEOUT));
	if(pdata->resume_finish == 0)
		MODEMDBG("resume_finish can do wakeup\n");
	pdata->resume_finish = 0;
	set_ap_wakeup_bp(GPIO_LOW);
	MODEMDBG("do_wakeup bp_statue sec level = %d\n",gpio_get_value(pdata->bp_statue));
	  	
}

static int tdm330_open(struct inode *inode, struct file *file)
{
	struct rk29_tdm330_data *pdata = gpdata;
	struct platform_data *pdev = container_of(pdata, struct device, platform_data);

	device_init_wakeup(&pdev, 1);
	return 0;
}

static int tdm330_release(struct inode *inode, struct file *file)
{
	//MODEMDBG("-------------%s\n",__FUNCTION__);
	return 0;
}

static int tdm330_ioctl(struct inode *inode,struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rk29_tdm330_data *pdata = gpdata;

	//MODEMDBG("-------------%s\n",__FUNCTION__);
	switch(cmd)
	{
		case TDM330_RESET:
			if(pdata->bp_reset > 0)  //对于旧的 模块是没有reset的 
			{
				set_bp_reset(GPIO_LOW);
				msleep(1000);
				set_bp_reset(GPIO_HIGH);
			}
			else
			{
				set_bp_power(GPIO_LOW);
				msleep(1000);
				set_bp_power(GPIO_HIGH);
			}
			set_ap_wakeup_bp(GPIO_LOW);
			wake_lock_timeout(&reset_wake_lock,20*HZ);
			break;
		default:
			break;
	}
	return 0;
}

static struct file_operations tdm330_fops = {
	.owner = THIS_MODULE,
	.open = tdm330_open,
	.release = tdm330_release,
	.ioctl = tdm330_ioctl
};

static struct miscdevice tdm330_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MODEM_NAME,
	.fops = &tdm330_fops
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
static ssize_t modem_status_read(struct class *cls, struct class_attribute *attr, char *_buf)
#else
static ssize_t modem_status_read(struct class *cls, char *_buf)
#endif
{
	return sprintf(_buf, "%d\n", modem_status);
	
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
static ssize_t modem_status_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
#else
static ssize_t modem_status_write(struct class *cls, const char *_buf, size_t _count)
#endif
{
    MODEMDBG("Read data from Android: %s\n", _buf);
   struct rk29_tdm330_data *pdata = gpdata;
   int new_state = simple_strtoul(_buf, NULL, 16);

   if (new_state == 1){
         MODEMDBG("%s, c(%d), open modem \n", __FUNCTION__, new_state);
	 set_bp_power(GPIO_HIGH);
	 msleep(1000);
	 set_bp_on(GPIO_HIGH);
	 set_bp_reset(GPIO_LOW);
	 msleep(1000);
	 set_bp_reset(GPIO_HIGH);
   }else if(new_state == 0){
	MODEMDBG("%s, c(%d), close modem \n", __FUNCTION__, new_state);
	set_bp_on(GPIO_LOW);
	msleep(1000);
	set_bp_power(GPIO_LOW);
   }else{
         printk("%s, invalid parameter \n", __FUNCTION__);
   }
    modem_status = new_state;
    return _count; 
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
static ssize_t wifi_ap_status_read(struct class *cls, struct class_attribute *attr, char *_buf)
#else
static ssize_t wifi_ap_status_read(struct class *cls, char *_buf)
#endif
{
	return sprintf(_buf, "%d\n", wifi_ap_status);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
static ssize_t wifi_ap_status_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
#else
static ssize_t wifi_ap_status_write(struct class *cls, const char *_buf, size_t _count)
#endif
{
    MODEMDBG("Read data from Android: %s\n", _buf);
   int new_state = simple_strtoul(_buf, NULL, 16);
   
   if (new_state == 1){
         MODEMDBG("%s,  wifi_ap_status == %d \n", __FUNCTION__, new_state);
		 wifi_ap_status = 1;
   }else if(new_state == 0){
         MODEMDBG("%s,  wifi_ap_status == $d \n", __FUNCTION__, new_state);
		 wifi_ap_status = 0;
   }else{
         printk("%s, invalid parameter \n", __FUNCTION__);
   }
    wifi_ap_status = new_state;
    return _count; 
}

static CLASS_ATTR(modem_status, 0777, modem_status_read, modem_status_write);
static CLASS_ATTR(wifi_ap_status, 0777, wifi_ap_status_read, wifi_ap_status_write);
static int tdm330_probe(struct platform_device *pdev)
{
	struct rk29_tdm330_data *pdata = gpdata = pdev->dev.platform_data;
	struct modem_dev *tdm330_data = NULL;
	int result, irq = 0;

	tdm330_data = kzalloc(sizeof(struct modem_dev), GFP_KERNEL);
	if(tdm330_data == NULL)
	{
		printk("failed to request tdm330_data\n");
		goto err3;
	}
	platform_set_drvdata(pdev, tdm330_data);		
        if(pdata->io_init)
	{
		pdata->io_init();
	}
	if(pdata->bp_power > 0)
	{
		result = gpio_request(pdata->bp_power, "tdm330");
		if (result) {
			printk("failed to request bp_power gpio\n");
			goto err2;
		}
	}
	if(pdata->bp_on > 0)
	{
		result = gpio_request(pdata->bp_on, "tdm330");
		if (result) {
			printk("failed to request bp_on gpio\n");
			goto err1;
		}
	}
	if(pdata->bp_reset > 0)
	{
		result = gpio_request(pdata->bp_reset, "tdm330");
		if (result) {
			printk("failed to request bp_reset gpio\n");
			goto err0;
		}
	}
	if(pdata->ap_wakeup_bp > 0)
	{
		result = gpio_request(pdata->ap_wakeup_bp,"tdm330");
		if(result) {
			printk("failed to requeset ap_wakeup_bp gpio\n");
			goto err5;
		}
	}
	
	if(pdata->bp_statue > 0)
	{
		result = gpio_request(pdata->bp_statue,"tdm330");
		if(result) {
			printk("failed to requeset bp_status gpio\n");
			goto err4;
		}
	}  
	set_output(pdata->bp_power, GPIO_HIGH);
	msleep(100);
	set_output(pdata->bp_on, GPIO_HIGH);
	msleep(300);
	set_output(pdata->bp_reset,GPIO_LOW);
	msleep(500);
	set_output(pdata->bp_reset,GPIO_HIGH);
	set_output(pdata->ap_wakeup_bp,GPIO_LOW);
	modem_status = 1;
	wifi_ap_status = 0;

	set_input(pdata->bp_statue);
	pdata->irq = gpio_to_irq(pdata->bp_statue);
	result = request_irq(pdata->irq, tdm330_irq, IRQF_TRIGGER_FALLING, "tdm330", pdata);
	if(result < 0){
		 printk("tdm330 irq error\n");
		 goto err4;
	}
	pdata->resume_finish = 0;
	init_waitqueue_head(&pdata->wait_resume);
	wake_lock_init(&reset_wake_lock,WAKE_LOCK_SUSPEND,"TD330_WAKE_LOCK");
	result = misc_register(&tdm330_misc);
	if(result)
	{
		printk("misc_register err\n");
	}
	
	printk("TDM330 TD-SCDMA modem probe OK.\n");

	return result;
	
err4:
	gpio_free(pdata->bp_statue);
err5:
	gpio_free(pdata->ap_wakeup_bp);
err0:
	gpio_free(pdata->bp_reset);
err1:
	gpio_free(pdata->bp_on);
err2:
	gpio_free(pdata->bp_power);
err3:
	kfree(tdm330_data);

	printk("TDM330 TD-SCDMA modem probe Failed!\n");
	return 0;
}

int tdm330_suspend(struct platform_device *pdev)
{
        MODEMDBG("-------------%s\n",__FUNCTION__);

	return 0;
}

static int  get_bp_statue(struct platform_device *pdev)
{
	struct rk29_tdm330_data *pdata = pdev->dev.platform_data;
	
	return gpio_get_value(pdata->bp_statue);//1:sleep

}

int tdm330_resume(struct platform_device *pdev)
{
	MODEMDBG("-------------%s\n",__FUNCTION__);
	do_wakeup();
	return 0;
}

void tdm330_shutdown(struct platform_device *pdev, pm_message_t state)
{
	struct rk29_tdm330_data *pdata = pdev->dev.platform_data;
	struct modem_dev *tdm330_data = platform_get_drvdata(pdev);
	
	MODEMDBG("-------------%s\n",__FUNCTION__);
	set_bp_on(GPIO_LOW);
	mdelay(1000);
        set_bp_power(GPIO_LOW);
	gpio_free(pdata->bp_power);
	gpio_free(pdata->bp_on);
	gpio_free(pdata->bp_reset);
	gpio_free(pdata->ap_wakeup_bp);
	gpio_free(pdata->bp_statue);

	kfree(tdm330_data);
}

static struct platform_driver tdm330_driver = {
	.probe	= tdm330_probe,
	.shutdown	= tdm330_shutdown,
	.suspend  	= tdm330_suspend,
	.resume		= tdm330_resume,
	.driver	= {
		.name	= "tdm330",
		.owner	= THIS_MODULE,
	},
};

static int __init tdm330_init(void)
{
	int ret;
	//MODEMDBG("-------------%s\n",__FUNCTION__);

	modem_class = class_create(THIS_MODULE, "rk291x_modem");
	ret =  class_create_file(modem_class, &class_attr_modem_status);
	ret =  class_create_file(modem_class, &class_attr_wifi_ap_status);
	if (ret)
	{
		printk("Fail to class rk291x_modem.\n");
	}
	return platform_driver_register(&tdm330_driver);
}

static void __exit tdm330_exit(void)
{
	//MODEMDBG("-------------%s\n",__FUNCTION__);
	platform_driver_unregister(&tdm330_driver);
	class_remove_file(modem_class, &class_attr_modem_status);
	class_remove_file(modem_class, &class_attr_wifi_ap_status);
}

module_init(tdm330_init);
module_exit(tdm330_exit);
