/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
//wisky libai 20120905
#include <linux/delay.h>
#include <linux/wisky.h>
#include <mach/iomux.h>

#define SWITCH_DEBUG 0
#if SWITCH_DEBUG
#define SWITCH_DBG(format, args...)	do {		\
		printk(KERN_INFO "<SWITCH-DEBUG> " format , ## args);	\
		} while (0)
#else
#define SWITCH_DBG(format, args...)
#endif

struct gpio_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	struct hrtimer timer;
	int irq;
	struct work_struct work;
};

int earphone_hdmi_insert = 0;
int earphone_hdmi_insert_status = -1;
extern voice_on_flag;
int voice_on_status = 0;

static int det_status=1;
static int status_times=0;
#define MicroTimeTInterupt	(25000000)
extern void codec_set_spk(int);

static void gpio_switch_work(struct work_struct *work)
{
	int state;
	struct gpio_switch_data	*data =
		container_of(work, struct gpio_switch_data, work);
	state = gpio_get_value(data->gpio);
	
	SWITCH_DBG("earphone_hdmi_insert value:%d voice_on_flag:%d\n", earphone_hdmi_insert, voice_on_flag);
	
	if(det_status!=state){
		status_times++;
		SWITCH_DBG("status_times:%d\n",status_times);
		if(status_times>=3){
            if(state==1)
            { 	
                //SWITCH_DBG("speaker on... ...\n");
                //gpio_set_value(SPEAKER_CTRL_PIN,GPIO_HIGH);	
                SWITCH_DBG("earphone out... ...\n");	
                earphone_hdmi_insert--;
                status_times=0;
                
            }
            else
            {
                //SWITCH_DBG("speaker off... ...\n");
                //gpio_set_value(SPEAKER_CTRL_PIN,GPIO_LOW);
                SWITCH_DBG("earphone insert... ...\n");	
                earphone_hdmi_insert++;
                status_times=0;
            }
            det_status=state;
            //ȥ����Ȼ�ζ���ʱ�Ῠ��һ��
            //switch_set_state(&data->sdev, !state); 
            SWITCH_DBG("%s state:%d\n",__func__,state);
		}
    } else 
        status_times=0;
    if ((earphone_hdmi_insert_status != earphone_hdmi_insert) || (voice_on_status != voice_on_flag)) {
        if (( !earphone_hdmi_insert)&&(voice_on_flag)){
			SWITCH_DBG("speaker on... ...while earphone_hdmi_insert=%d  voice_on_flag:%d\n ", 
               earphone_hdmi_insert, voice_on_flag);
			gpio_set_value(SPEAKER_CTRL_PIN,GPIO_HIGH);
		} else {
			SWITCH_DBG("speaker off... ...while earphone_hdmi_insert=%d  voice_on_flag:%d\n ", 
               earphone_hdmi_insert, voice_on_flag);
			gpio_set_value(SPEAKER_CTRL_PIN,GPIO_LOW);
        }
        earphone_hdmi_insert_status = earphone_hdmi_insert;
        voice_on_status = voice_on_flag;
    }	
	
}
static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data =
	    (struct gpio_switch_data *)dev_id;
	//SWITCH_DBG("%s\n",__func__);
	schedule_work(&switch_data->work);
	return IRQ_HANDLED;
}
static enum hrtimer_restart switch_timer(struct hrtimer *timer)
{	
	struct gpio_switch_data *switch_data= container_of(timer, struct gpio_switch_data, timer);
	schedule_work(&switch_data->work);
	hrtimer_start(&switch_data->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}
static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct gpio_switch_data	*switch_data =
		container_of(sdev, struct gpio_switch_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static int gpio_switch_probe(struct platform_device *pdev)
{        
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_switch_data *switch_data;
	int ret = 0;
	
	if (!pdata)
		return -EBUSY;

	switch_data = kzalloc(sizeof(struct gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = pdata->name;
	switch_data->gpio = pdata->gpio;
	switch_data->name_on = pdata->name_on;
	switch_data->name_off = pdata->name_off;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
	switch_data->sdev.print_state = switch_gpio_print_state;

    ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	ret = gpio_request(switch_data->gpio, pdev->name);
	if (ret < 0)
		goto err_request_gpio;
	rk30_mux_api_set(GPIO2C7_LCDC1DATA23_SPI1CSN1_HSADCDATA4_NAME,0); 
	#if 0
	gpio_pull_updown(switch_data->gpio, GPIOPullUp);	
	gpio_direction_output(switch_data->gpio,1);
	mdelay(10);
	#else
	
	gpio_pull_updown(switch_data->gpio, PullDisable);
	mdelay(10);
	gpio_direction_input(switch_data->gpio);
	mdelay(10);
	#endif
	INIT_WORK(&switch_data->work, gpio_switch_work);
#if 0
	switch_data->irq = gpio_to_irq(switch_data->gpio);
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;
	}

	ret = request_irq(switch_data->irq, gpio_irq_handler,
			  IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, pdev->name, switch_data);//wisky libai IRQF_TRIGGER_LOW
#else
		hrtimer_init(&switch_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		switch_data->timer.function = switch_timer;
		//hrtimer_start(&switch_data->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		hrtimer_start(&switch_data->timer, ktime_set(10, 0), HRTIMER_MODE_REL);
#endif
	if (ret < 0)
		goto err_request_irq;

	
	/* Perform initial detection */
	//gpio_switch_work(&switch_data->work);
	return 0;

err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->gpio);
err_request_gpio:
    switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int __devexit gpio_switch_remove(struct platform_device *pdev)
{
	struct gpio_switch_data *switch_data = platform_get_drvdata(pdev);

	cancel_work_sync(&switch_data->work);
	gpio_free(switch_data->gpio);
    switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

static struct platform_driver gpio_switch_driver = {
	.probe		= gpio_switch_probe,
	.remove		= __devexit_p(gpio_switch_remove),
	.driver		= {
		.name	= "switch-gpio",
		.owner	= THIS_MODULE,
	},
};

static int __init gpio_switch_init(void)
{
	return platform_driver_register(&gpio_switch_driver);
}

static void __exit gpio_switch_exit(void)
{
	platform_driver_unregister(&gpio_switch_driver);
}

module_init(gpio_switch_init);
module_exit(gpio_switch_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");
