/* arch/arm/mach-rockchip/rk28_headset.c
 *
 * Copyright (C) 2009 Rockchip Corporation.
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
 */

#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <asm/gpio.h>
#include <asm/atomic.h>
#include <asm/mach-types.h>
#include <linux/earlysuspend.h>
#include <linux/gpio.h>
#include <mach/board.h>

#include "wisky_headset_gpio.h"

/* Debug */
#if 1
#define DBG(x...) printk(x)
#else
#define DBG(x...) do { } while (0)
#endif

#define BIT_HEADSET             (1 << 0)
#define BIT_HEADSET_NO_MIC      (1 << 1)

#define HEADSET 0
#define HOOK 1
#define HEADSET_MIC 2

#define HEADSET_IN 1
#define HEADSET_OUT 0
#define HOOK_DOWN 1
#define HOOK_UP 0
#define enable 1
#define disable 0

extern int wm8994_set_status(void);
struct codec_mic_bias HS_mic_bias = {
	.get_micbias = NULL,
	.set_micbias = NULL,
};

/* headset private data */
struct headset_priv {
	struct input_dev *input_dev;
	struct rk_headset_pdata *pdata;
	unsigned int headset_status:1;
	unsigned int hook_status:1;
	unsigned int isMic:1;
	unsigned int isHook_irq:1;
	int cur_headset_status; 
	
	unsigned int irq[2];
	unsigned int irq_type[2];
	struct delayed_work h_delayed_work[3];
	struct switch_dev sdev;
	struct mutex mutex_lock[2];	
	//struct timer_list headset_timer;
	unsigned char *keycodes;
};
static struct headset_priv *headset_info;

#define AUDIO_MUTE_CLASS	"audio_mute"
static struct class *audio_mute_class = NULL;
int audio_volume = 1;//record current media audio volume, initialize as not zero

extern int audio_trigger_flag;
extern void codec_set_spk(int on);
#if WISKY_ENABLE_HDMI
extern int hdmi_status;// 0---lcd. 1--hdmi enable
#endif

int Headset_isMic(void)
{
	return headset_info->isMic;
}
EXPORT_SYMBOL_GPL(Headset_isMic);

static irqreturn_t headset_interrupt(int irq, void *dev_id)
{
	WPRINTK("---headset_interrupt---\n");
	schedule_delayed_work(&headset_info->h_delayed_work[HEADSET], msecs_to_jiffies(200));
	return IRQ_HANDLED;
}

static irqreturn_t Hook_interrupt(int irq, void *dev_id)
{
	WPRINTK("---Hook_interrupt---\n");
//	disable_irq_nosync(headset_info->irq[HOOK]);
	schedule_delayed_work(&headset_info->h_delayed_work[HOOK], msecs_to_jiffies(120));//100
	return IRQ_HANDLED;
}

static int headset_change_irqtype(int type,unsigned int irq_type)
{
	int ret = 0;
	WPRINTK("--------%s----------\n",__FUNCTION__);
	free_irq(headset_info->irq[type],NULL);
	
	switch(type)
	{
	case HOOK:
		ret = request_irq(headset_info->irq[type], Hook_interrupt, irq_type, NULL, NULL);
		break;
	case HEADSET:
		ret = request_irq(headset_info->irq[type], headset_interrupt, irq_type, NULL, NULL);
		break;
	default:
		ret = -1;
		break;
	}

	if (ret<0) 
	{
		WPRINTK("headset_change_irqtype: request irq failed\n");
        return ret;
	}
	return ret;
}

static void headsetobserve_work(struct work_struct *work)
{
	int i,level = 0;
	struct rk_headset_pdata *pdata = headset_info->pdata;
	static unsigned int old_status = 0;
	
	WPRINTK("---headsetobserve_work---\n");
	mutex_lock(&headset_info->mutex_lock[HEADSET]);

	for(i=0; i<3; i++)
	{
		level = gpio_get_value(pdata->Headset_gpio);
		if(level < 0)
		{
			printk("%s:get pin level again,pin=%d,i=%d\n",__FUNCTION__,pdata->Headset_gpio,i);
			msleep(1);
			continue;
		}
		else
		break;
	}
	if(level < 0)
	{
		printk("%s:get pin level  err!\n",__FUNCTION__);
		goto RE_ERROR;
	}

	old_status = headset_info->headset_status;
	switch(pdata->headset_in_type)
	{
	case HEADSET_IN_HIGH:
		if(level > 0)
			headset_info->headset_status = HEADSET_IN;
		else if(level == 0)
			headset_info->headset_status = HEADSET_OUT;	
		break;
	case HEADSET_IN_LOW:
		if(level == 0)
			headset_info->headset_status = HEADSET_IN;
		else if(level > 0)
			headset_info->headset_status = HEADSET_OUT;		
		break;			
	default:
		WPRINTK("---- ERROR: on headset headset_in_type error -----\n");
		break;			
	}
	if(old_status == headset_info->headset_status)
	{
		WPRINTK("%s:old_status == headset_info->headset_status\n",__FUNCTION__);
		goto RE_ERROR;
	}

	switch(pdata->headset_in_type)
	{
	case HEADSET_IN_HIGH:
		if(level > 0)
		{//in--High level
			WPRINTK("--- HEADSET_IN_HIGH headset in HIGH---\n");
			headset_info->cur_headset_status = BIT_HEADSET;
			headset_change_irqtype(HEADSET,IRQF_TRIGGER_FALLING);//
			if(pdata->Hook_gpio){
				schedule_delayed_work(&headset_info->h_delayed_work[HEADSET_MIC], msecs_to_jiffies(300));
				goto RE_ERROR;
			} else {
				headset_info->cur_headset_status |= BIT_HEADSET_NO_MIC;
			}
		}
		else if(level == 0)
		{//out--Low level
			WPRINTK("---HEADSET_IN_HIGH headset out HIGH---\n");	
			if(headset_info->isHook_irq == enable)
			{
				WPRINTK("disable_irq\n");
				headset_info->isHook_irq = disable;
				if(pdata->Hook_gpio)
					disable_irq(headset_info->irq[HOOK]);		
			}	
			headset_info->cur_headset_status = 0;
			headset_change_irqtype(HEADSET,IRQF_TRIGGER_RISING);//
		}
		break;
	case HEADSET_IN_LOW:
		if(level == 0)
		{//in--High level
			WPRINTK("---HEADSET_IN_LOW headset in LOW ---\n");
			headset_info->cur_headset_status = BIT_HEADSET;
			headset_change_irqtype(HEADSET,IRQF_TRIGGER_RISING);//
			if(pdata->Hook_gpio) {
				schedule_delayed_work(&headset_info->h_delayed_work[HEADSET_MIC], msecs_to_jiffies(300));
				goto RE_ERROR;
			} else {
				headset_info->cur_headset_status |= BIT_HEADSET_NO_MIC;
			}
		}
		else if(level > 0)
		{//out--High level
			WPRINTK("---HEADSET_IN_LOW headset out LOW ---\n");
			if(headset_info->isHook_irq == enable)
			{
				WPRINTK("disable_irq\n");
				headset_info->isHook_irq = disable;
				if(pdata->Hook_gpio)
					disable_irq(headset_info->irq[HOOK]);		
			}				
			headset_info->cur_headset_status = 0;
			headset_change_irqtype(HEADSET,IRQF_TRIGGER_FALLING);//
		}
		break;			
	default:
		WPRINTK("---- ERROR: on headset headset_in_type error -----\n");
		break;			
	}
	rk28_send_wakeup_key();
	switch_set_state(&headset_info->sdev, headset_info->cur_headset_status);	
	
RE_ERROR:
	mutex_unlock(&headset_info->mutex_lock[HEADSET]);
	WPRINTK("%s:cur_headset_status = %d\n", __FUNCTION__, headset_info->cur_headset_status);

	if(headset_info->cur_headset_status != 0){//headset inset
		if(0 == audio_volume || 0 == audio_trigger_flag){
#if defined(HEADPHONE_MUTE_PIN)
			if(HEADPHONE_MUTE_PIN != NULL_GPIO){
				gpio_set_value(HEADPHONE_MUTE_PIN, HEADPHONE_MUTE_ACTIVE_LEVEL);//disable headphone
			}
#endif
		}else{
#if defined(HEADPHONE_MUTE_PIN)
			if(HEADPHONE_MUTE_PIN != NULL_GPIO){
				gpio_set_value(HEADPHONE_MUTE_PIN, !HEADPHONE_MUTE_ACTIVE_LEVEL);//enable headphone
			}
#endif
		}
		codec_set_spk(0);
	}else{//headset remove
#if (defined(CONFIG_ANX7150) || defined(WISKY_HDMI_ANX7150)) && (WISKY_ENABLE_HDMI == 1)
		if(0 == hdmi_status){
			codec_set_spk(1);//enable speaker
		}else{
			codec_set_spk(0);//disable speaker
		}
#else
		codec_set_spk(1);
#endif
	}
}

static void Hook_work(struct work_struct *work)
{
	int i,level = 0;
	struct rk_headset_pdata *pdata = headset_info->pdata;
	static unsigned int old_status = HOOK_UP;

	WPRINTK("---Hook_work---\n");
	mutex_lock(&headset_info->mutex_lock[HOOK]);
	if(headset_info->headset_status == HEADSET_OUT)
	{
		WPRINTK("%s:Headset is out\n",__FUNCTION__);
		goto RE_ERROR;
	}	
	#ifdef CONFIG_SND_SOC_WM8994
	if(wm8994_set_status() < 0)
	{
		WPRINTK("wm8994 is not set on heatset channel or suspend\n");
		goto RE_ERROR;
	}
	#endif
	
 	if(HS_mic_bias.get_micbias)
		if(!HS_mic_bias.get_micbias())
			goto RE_ERROR;
	
	for(i=0; i<3; i++)
	{
		level = gpio_get_value(pdata->Hook_gpio);
		if(level < 0){
			printk("%s:get pin level again,pin=%d,i=%d\n",__FUNCTION__,pdata->Hook_gpio,i);
			msleep(1);
			continue;
		}else{
			//break;
			mdelay(80);
			if(gpio_get_value(pdata->Hook_gpio) != level){
				pr_info("%s[%d]--->HOOK gpio level change, cancel hook key report !\n", __FUNCTION__, __LINE__);
				goto RE_ERROR;
			}
		}
	}
	if(level < 0)
	{
		printk("%s:get pin level  err!\n",__FUNCTION__);
		goto RE_ERROR;
	}
	
	old_status = headset_info->hook_status;
	if(level == 0)
		headset_info->hook_status = HOOK_DOWN;
	else if(level > 0)	
		headset_info->hook_status = HOOK_UP;
	if(old_status == headset_info->hook_status)
	{
		WPRINTK("%s:old_status == headset_info->hook_status\n",__FUNCTION__);
		goto RE_ERROR;
	}	
	
	if(level == 0)
	{
		WPRINTK("---HOOK Down ---\n");
		headset_change_irqtype(HOOK,IRQF_TRIGGER_RISING);//
		input_report_key(headset_info->input_dev,pdata->hook_key_code,headset_info->hook_status);
		input_sync(headset_info->input_dev);
	}
	else if(level > 0)
	{
		WPRINTK("---HOOK Up ---\n");
		headset_change_irqtype(HOOK,IRQF_TRIGGER_FALLING);//
		input_report_key(headset_info->input_dev,pdata->hook_key_code,headset_info->hook_status);
		input_sync(headset_info->input_dev);
	}
RE_ERROR:
	mutex_unlock(&headset_info->mutex_lock[HOOK]);
}

//static void headset_timer_callback(unsigned long arg)
static void headset_mic_work(struct work_struct *work)
{
	//struct headset_priv *headset = (struct headset_priv *)(arg);
	struct headset_priv *headset = headset_info;
	struct rk_headset_pdata *pdata = headset->pdata;
	int i,level = 0;
	
//	WPRINTK("%s,headset->headset_status=%d\n",__FUNCTION__, headset->headset_status);	
	mutex_lock(&headset_info->mutex_lock[HEADSET]);
	if(headset->headset_status == HEADSET_OUT)
	{
		WPRINTK("%s:Headset is out\n",__FUNCTION__);
		headset_info->cur_headset_status = 0;
		goto out;
	}
	
	#ifdef CONFIG_SND_SOC_WM8994
	headset_info->cur_headset_status = BIT_HEADSET;
	if(wm8994_set_status() < 0)
	{
		WPRINTK("wm8994 is not set on heatset channel\n");
		schedule_delayed_work(&headset_info->h_delayed_work[HEADSET_MIC], msecs_to_jiffies(500));
		goto ERROR;
	}
	goto out;
	#endif

	if(HS_mic_bias.get_micbias)
		if(!HS_mic_bias.get_micbias()){
			if(HS_mic_bias.set_micbias){
				HS_mic_bias.set_micbias();
				printk("%s:set_micbias\n",__FUNCTION__);
			}
			schedule_delayed_work(&headset_info->h_delayed_work[HEADSET_MIC], msecs_to_jiffies(500));
			goto ERROR;
		}

	for(i=0; i<3; i++)
	{
		level = gpio_get_value(pdata->Hook_gpio);
		if(level < 0)
		{
			printk("%s:get pin level again,pin=%d,i=%d\n",__FUNCTION__,pdata->Hook_gpio,i);
			msleep(1);
			continue;
		}else{
			//break;
			mdelay(80);
			if(gpio_get_value(pdata->Hook_gpio) != level){
				pr_info("%s[%d]--->HOOK gpio level change, cancel hook key report !\n", __FUNCTION__, __LINE__);
				goto ERROR;
			}
		}
	}
	if(level < 0)
	{
		printk("%s:get pin level  err!\n",__FUNCTION__);
		goto out;
	}

	if(level == 0) {
		headset->isMic= 0;//No microphone
		headset_info->cur_headset_status |= BIT_HEADSET_NO_MIC;//no headset, no mic.
#if 1	
		if(pdata->Hook_gpio){
			enable_irq(headset_info->irq[HOOK]);
			headset->isHook_irq = enable;
		}
#endif
	}
	else if(level > 0)	
	{	
		headset->isMic = 1;//have mic
	//	WPRINTK("enable_irq\n");	
		if(pdata->Hook_gpio)
			enable_irq(headset_info->irq[HOOK]);
		headset->isHook_irq = enable;
		headset_info->cur_headset_status &= ~BIT_HEADSET_NO_MIC;//have mic
	}	
	if(pdata->Hook_gpio == 0)
		headset_info->cur_headset_status |= BIT_HEADSET_NO_MIC;
	WPRINTK("headset->isMic = %d\n",headset->isMic);
out:
	rk28_send_wakeup_key();
	switch_set_state(&headset_info->sdev, headset_info->cur_headset_status);
	
ERROR:
	mutex_unlock(&headset_info->mutex_lock[HEADSET]);
	WPRINTK("%s:report headset status = 0x%x\n",__FUNCTION__, headset_info->cur_headset_status);

	return;
}

static ssize_t h2w_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "Headset\n");
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void headset_early_resume(struct early_suspend *h)
{
	//schedule_delayed_work(&headset_info->h_delayed_work[HEADSET], msecs_to_jiffies(200));
	//WPRINTK(">>>>>headset_early_resume\n");
}

static struct early_suspend hs_early_suspend;
#endif

static int rk_Hskey_open(struct input_dev *dev)
{
	//struct rk28_adckey *adckey = input_get_drvdata(dev);
//	WPRINTK("===========rk_Hskey_open===========\n");
	return 0;
}

static void rk_Hskey_close(struct input_dev *dev)
{
//	WPRINTK("===========rk_Hskey_close===========\n");
//	struct rk28_adckey *adckey = input_get_drvdata(dev);

}

static ssize_t volume_store(struct class *cls, const char *buf, size_t count)
{
	char temp[10];

	strncpy(temp, buf, count);
	audio_volume = simple_strtol(temp, NULL, 10);

	if(0 == audio_volume){//disable headphone when media audio volume is zero
#if defined(HEADPHONE_MUTE_PIN)
		if(HEADPHONE_MUTE_PIN != NULL_GPIO){
			gpio_set_value(HEADPHONE_MUTE_PIN, HEADPHONE_MUTE_ACTIVE_LEVEL);//disable headphone
		}
#endif
	}else{
#if defined(HEADPHONE_MUTE_PIN)
		if(HEADPHONE_MUTE_PIN != NULL_GPIO){
			if(HEADPHONE_DET_IN_VALUE != gpio_get_value(HEADPHONE_DET_PIN)){
			//headphone remove
				gpio_set_value(HEADPHONE_MUTE_PIN, HEADPHONE_MUTE_ACTIVE_LEVEL);//disable headphone
			}else{//headphone insert
				gpio_set_value(HEADPHONE_MUTE_PIN, !HEADPHONE_MUTE_ACTIVE_LEVEL);//enable headphone
			}
			
		}
#endif
	}

	return count;
}

static CLASS_ATTR(volume, 0222, NULL, volume_store);
//wisky[e]

static int rockchip_headsetobserve_probe(struct platform_device *pdev)
{
	int ret;
	struct headset_priv *headset;
	struct rk_headset_pdata *pdata;
	
	audio_mute_class = class_create(THIS_MODULE, AUDIO_MUTE_CLASS);
	if(IS_ERR(audio_mute_class)){
		pr_err("%s-%d: create audio mute class failed\n", __FUNCTION__, __LINE__);
	}else{
		ret = class_create_file(audio_mute_class, &class_attr_volume);
	}

	headset = kzalloc(sizeof(struct headset_priv), GFP_KERNEL);
	if (headset == NULL) {
		dev_err(&pdev->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}	
	headset->pdata = pdev->dev.platform_data;
	pdata = headset->pdata;
	headset->headset_status = HEADSET_OUT;
	headset->hook_status = HOOK_UP;
	headset->isHook_irq = disable;
	headset->cur_headset_status = 0;
	headset->sdev.name = "h2w";
	headset->sdev.print_name = h2w_print_name;
	ret = switch_dev_register(&headset->sdev);
	if (ret < 0)
		goto failed_free;
	
	mutex_init(&headset->mutex_lock[HEADSET]);
	mutex_init(&headset->mutex_lock[HOOK]);
	
	INIT_DELAYED_WORK(&headset->h_delayed_work[HEADSET], headsetobserve_work);
	INIT_DELAYED_WORK(&headset->h_delayed_work[HOOK], Hook_work);
	INIT_DELAYED_WORK(&headset->h_delayed_work[HEADSET_MIC], headset_mic_work);

	headset->isMic = 0;
	//setup_timer(&headset->headset_timer, headset_timer_callback, (unsigned long)headset);
//	headset->headset_timer.expires = jiffies + 1000;
//	add_timer(&headset->headset_timer);	
//------------------------------------------------------------------
	if(pdata->control_io){
		ret = gpio_request(pdata->control_io, NULL);
		if (ret) {
			printk("request heaset control io failed!\n");
			goto failed_free;
		}
		gpio_direction_output(pdata->control_io, 1);
	}

	ret = gpio_request(pdata->Headset_gpio, NULL);
	if (ret) 
		goto failed_free;

#ifdef CONFIG_MACH_RK29_LIRUI
	gpio_pull_updown(pdata->Headset_gpio, GPIOPullUp);//
#else
	gpio_pull_updown(pdata->Headset_gpio, PullDisable);//
#endif
	gpio_direction_input(pdata->Headset_gpio);

	if(pdata->headset_in_type == HEADSET_IN_HIGH) {
		headset->irq_type[HEADSET] = IRQF_TRIGGER_RISING;
	}
	else {
		headset->irq_type[HEADSET] = IRQF_TRIGGER_FALLING;
	}	

//------------------------------------------------------------------
	ret = gpio_request(pdata->Hook_gpio , NULL);
	if (ret) 
		goto home;
	gpio_pull_updown(pdata->Hook_gpio, PullDisable);//PullDisable 
	gpio_direction_input(pdata->Hook_gpio);

//------------------------------------------------------------------		
	// Create and register the input driver. 
	headset->input_dev = input_allocate_device();
	if (!headset->input_dev) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		ret = -ENOMEM;
		goto failed_free;
	}	
	headset->input_dev->name = pdev->name;
	headset->input_dev->open = rk_Hskey_open;
	headset->input_dev->close = rk_Hskey_close;
	headset->input_dev->dev.parent = &pdev->dev;
	//input_dev->phys = KEY_PHYS_NAME;
	headset->input_dev->id.vendor = 0x0001;
	headset->input_dev->id.product = 0x0001;
	headset->input_dev->id.version = 0x0100;
	// Register the input device 
	
	ret = input_register_device(headset->input_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed_free;
	}
	input_set_capability(headset->input_dev, EV_KEY, pdata->hook_key_code);

	headset->irq[HOOK] = gpio_to_irq(pdata->Hook_gpio);
	headset->irq_type[HOOK] = IRQF_TRIGGER_FALLING;
	ret = request_irq(headset->irq[HOOK], Hook_interrupt, headset->irq_type[HOOK] , NULL, NULL);
	if (ret) 
		goto failed_free_dev;
	disable_irq(headset->irq[HOOK]);
home:

	//request irq
	headset->irq[HEADSET] = gpio_to_irq(pdata->Headset_gpio);
	ret = request_irq(headset->irq[HEADSET], headset_interrupt, headset->irq_type[HEADSET], NULL, NULL);
	if (ret) 
		goto failed_free_dev;
	enable_irq_wake(headset->irq[HEADSET]);


//	headset->input_dev->keycode = headset->keycodes;
//	headset->input_dev->keycodesize = sizeof(unsigned char);
//	headset->input_dev->keycodemax = 2;
	
//	set_bit(KEY_MEDIA, headset->input_dev->keybit);
//	clear_bit(0, headset->input_dev->keybit);
//	input_set_capability(headset->input_dev, EV_SW, SW_HEADPHONE_INSERT);
//	input_set_capability(headset->input_dev, EV_KEY, KEY_END);

//	headset->input_dev->evbit[0] = BIT_MASK(EV_KEY);
	
	headset_info = headset;
	schedule_delayed_work(&headset->h_delayed_work[HEADSET], msecs_to_jiffies(25000));

#ifdef CONFIG_HAS_EARLYSUSPEND
	hs_early_suspend.suspend = NULL;
	hs_early_suspend.resume = headset_early_resume;
	hs_early_suspend.level = ~0x0;
	register_early_suspend(&hs_early_suspend);
#endif
	WPRINTK("%s[%d]: done.\n",__FUNCTION__,__LINE__);
	
	return 0;	
	
failed_free_dev:
	platform_set_drvdata(pdev, NULL);
	input_free_device(headset->input_dev);
failed_free:
	kfree(headset);	
	WPRINTK("%s[%d]: Failed!\n",__FUNCTION__,__LINE__);
	return ret;
}

static int rockchip_headsetobserve_suspend(struct platform_device *pdev, pm_message_t state)
{
	WPRINTK("%s----%d\n",__FUNCTION__,__LINE__);
	disable_irq(headset_info->irq[HEADSET]);
	disable_irq(headset_info->irq[HOOK]);

	return 0;
}

static int rockchip_headsetobserve_resume(struct platform_device *pdev)
{
	WPRINTK("%s----%d\n",__FUNCTION__,__LINE__);	
	enable_irq(headset_info->irq[HEADSET]);
	enable_irq(headset_info->irq[HOOK]);
	
	return 0;
}

static int __devexit rockchip_headsetobserve_remove(struct platform_device *pdev)
{
	class_remove_file(audio_mute_class, &class_attr_volume);
	class_destroy(audio_mute_class);

	return 0;
}

static struct platform_driver rockchip_headsetobserve_driver = {
	.probe	= rockchip_headsetobserve_probe,
	.remove = __devexit_p(rockchip_headsetobserve_remove),
//	.resume = 	rockchip_headsetobserve_resume,	
//	.suspend = 	rockchip_headsetobserve_suspend,	
	.driver	= {
		.name	= "rk_headsetdet",
		.owner	= THIS_MODULE,
	},
};

static int __init rockchip_headsetobserve_init(void)
{
	platform_driver_register(&rockchip_headsetobserve_driver);
	return 0;
}
module_init(rockchip_headsetobserve_init);
MODULE_DESCRIPTION("Rockchip Headset Driver");
MODULE_LICENSE("GPL");
