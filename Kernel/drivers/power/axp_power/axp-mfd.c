/*
 * Base driver for X-Powers AXP
 *
 * Copyright (C) 2011 X-Powers, Ltd.
 *  Zhang Donglu <zhangdonglu@x-powers.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <mach/system.h>

#include "axp-cfg.h"
#include "axp-rw.h"
//#include "axp-board.h"
#include <linux/io.h>
#include <mach/io.h>
#include <mach/sram.h>
#include <mach/gpio.h>

#define cru_readl(offset)	readl_relaxed(RK30_CRU_BASE + offset)
#define cru_writel(v, offset)	do { writel_relaxed(v, RK30_CRU_BASE + offset); dsb(); } while (0)

#define grf_readl(offset)	readl_relaxed(RK30_GRF_BASE + offset)
#define grf_writel(v, offset)	do { writel_relaxed(v, RK30_GRF_BASE + offset); dsb(); } while (0)

#define CRU_CLKGATE5_CON_ADDR 0x00e4
#define GRF_GPIO6L_DIR_ADDR 0x0030
#define GRF_GPIO6L_DO_ADDR 0x0068
#define GRF_GPIO6L_EN_ADDR 0x00a0
#define CRU_CLKGATE5_GRFCLK_ON 0x00100000
#define CRU_CLKGATE5_GRFCLK_OFF 0x00100010
#define GPIO6_PB1_DIR_OUT  0x02000200
#define GPIO6_PB1_DO_LOW  0x02000000
#define GPIO6_PB1_DO_HIGH  0x02000200
#define GPIO6_PB1_EN_MASK  0x02000200
#define GPIO6_PB1_UNEN_MASK  0x02000000

#define PMU_POWER_SLEEP RK30_PIN6_PB1

#ifdef CONFIG_CLK_SWITCH_TO_32K
void __sramfunc board_pmu_suspend(void)
{	
	cru_writel(CRU_CLKGATE5_GRFCLK_ON,CRU_CLKGATE5_CON_ADDR); //open grf clk
	grf_writel(GPIO6_PB1_DIR_OUT, GRF_GPIO6L_DIR_ADDR);
	grf_writel(GPIO6_PB1_DO_HIGH, GRF_GPIO6L_DO_ADDR);  //set gpio6_b1 output low
	grf_writel(GPIO6_PB1_EN_MASK, GRF_GPIO6L_EN_ADDR);
}
void __sramfunc board_pmu_resume(void)
{
	grf_writel(GPIO6_PB1_DIR_OUT, GRF_GPIO6L_DIR_ADDR);
	grf_writel(GPIO6_PB1_DO_LOW, GRF_GPIO6L_DO_ADDR);     //set gpio6_b1 output high
	grf_writel(GPIO6_PB1_EN_MASK, GRF_GPIO6L_EN_ADDR);
#ifdef CONFIG_CLK_SWITCH_TO_32K
	sram_32k_udelay(10000);
#else
	sram_udelay(10000);
#endif
}
#endif
//wisky[e]

static int axp_init_chip(struct axp_mfd_chip *chip)
{
	uint8_t chip_id;
	int err;
#if defined (CONFIG_AXP_DEBUG)
	uint8_t val[AXP_DATA_NUM];
	int i;
#endif
	/*read chip id*/
	err =  __axp_read(chip->client, AXP_IC_TYPE, &chip_id);
	if (err) {
	    printk("[AXP-MFD] try to read chip id failed!\n");
		return err;
	}

	dev_info(chip->dev, "AXP (CHIP ID: 0x%02x) detected\n", chip_id);

	/* mask and clear all IRQs */
#if defined (CONFIG_KP_AXP20)
	chip->ops->disable_irqs(chip,0xffffffffff);
	chip->ops->clear_irqs(chip,0xffffffffff);
#endif

#if defined (CONFIG_KP_AXP19)
	chip->ops->disable_irqs(chip,0xffffffff);
	chip->ops->clear_irqs(chip,0xffffffff);
#endif

#if 0 //defined (CONFIG_AXP_DEBUG)
	axp_reads(&axp->dev,AXP_DATA_BUFFER0,AXP_DATA_NUM,val);
	for( i = 0; i < AXP_DATA_NUM; i++){
		printk("REG[0x%x] = 0x%x\n",i+AXP_DATA_BUFFER0,val[i]);	
	}
#endif

	gpio_request(PMU_POWER_SLEEP, "NULL");
	gpio_direction_output(PMU_POWER_SLEEP, GPIO_LOW);
	
	printk("%s:%d end\n", __func__, __LINE__);
	return 0;
}

static int axp_disable_irqs(struct axp_mfd_chip *chip, uint64_t irqs)
{
#if defined (CONFIG_KP_AXP20)
	uint8_t v[9];
#endif

#if defined (CONFIG_KP_AXP19)
	uint8_t v[7];
#endif
	int ret;

	chip->irqs_enabled &= ~irqs;

	v[0] = ((chip->irqs_enabled) & 0xff);
	v[1] = AXP_INTEN2;
	v[2] = ((chip->irqs_enabled) >> 8) & 0xff;
	v[3] = AXP_INTEN3;
	v[4] = ((chip->irqs_enabled) >> 16) & 0xff;
	v[5] = AXP_INTEN4;
	v[6] = ((chip->irqs_enabled) >> 24) & 0xff;
#if defined (CONFIG_KP_AXP20)
	v[7] = AXP_INTEN5;
	v[8] = ((chip->irqs_enabled) >> 32) & 0xff;	
	ret =  __axp_writes(chip->client, AXP_INTEN1, 9, v);
#endif
#if defined (CONFIG_KP_AXP19)
	ret =  __axp_writes(chip->client, AXP_INTEN1, 7, v);
#endif
	return ret;

}

static int axp_enable_irqs(struct axp_mfd_chip *chip, uint64_t irqs)
{
#if defined (CONFIG_KP_AXP20)
	uint8_t v[9];
#endif

#if defined (CONFIG_KP_AXP19)
	uint8_t v[7];
#endif
	int ret;

	chip->irqs_enabled |=  irqs;

	v[0] = ((chip->irqs_enabled) & 0xff);
	v[1] = AXP_INTEN2;
	v[2] = ((chip->irqs_enabled) >> 8) & 0xff;
	v[3] = AXP_INTEN3;
	v[4] = ((chip->irqs_enabled) >> 16) & 0xff;
	v[5] = AXP_INTEN4;
	v[6] = ((chip->irqs_enabled) >> 24) & 0xff;
#if defined (CONFIG_KP_AXP20)
	v[7] = AXP_INTEN5;
	v[8] = ((chip->irqs_enabled) >> 32) & 0xff;
	ret =  __axp_writes(chip->client, AXP_INTEN1, 9, v);
#endif
#if defined (CONFIG_KP_AXP19)
	ret =  __axp_writes(chip->client, AXP_INTEN1, 7, v);
#endif
	return ret;
}

#if defined (CONFIG_KP_AXP20)
static int axp_read_irqs(struct axp_mfd_chip *chip, uint64_t *irqs)
{
	uint8_t v[5];
	int ret;
	ret =  __axp_reads(chip->client, AXP_INTSTS1, 5, v);
	
	*irqs =(((uint64_t) v[4]) << 32) |(((uint64_t) v[3]) << 24) | (((uint64_t) v[2])<< 16) | (((uint64_t)v[1]) << 8) | ((uint64_t) v[0]);
	return ret;
}
#endif

#if defined (CONFIG_KP_AXP19)
static int axp_read_irqs(struct axp_mfd_chip *chip, uint64_t *irqs)
{
	uint8_t v[4];
	int ret;
	ret =  __axp_reads(chip->client, AXP_INTSTS1, 4, v);
	
	*irqs =((((uint64_t) v[3]) << 24) | (((uint64_t) v[2])<< 16) | (((uint64_t)v[1]) << 8) | ((uint64_t) v[0]));
	return ret;
}
#endif

static int axp_clear_irqs(struct axp_mfd_chip *chip, uint64_t irqs)
{
#if defined (CONFIG_KP_AXP20)
	uint8_t v[9];
#endif

#if defined (CONFIG_KP_AXP19)
	uint8_t v[7];
#endif
	int ret;
	v[0] = (irqs >>  0)& 0xFF;
	v[1] = AXP_INTSTS2;
	v[2] = (irqs >>  8) & 0xFF;
	v[3] = AXP_INTSTS3;
	v[4] = (irqs >> 16) & 0xFF;
	v[5] = AXP_INTSTS4;
	v[6] = (irqs >> 24) & 0xFF;
#if defined (CONFIG_KP_AXP20)
	v[7] = AXP_INTSTS5;
	v[8] = (irqs >> 32) & 0xFF;	
	ret =  __axp_writes(chip->client, AXP_INTSTS1, 9, v);
#endif
#if defined (CONFIG_KP_AXP19)
	ret =  __axp_writes(chip->client, AXP_INTSTS1, 7, v);
#endif
	return ret;
}

static struct axp_mfd_chip_ops axp_mfd_ops[] = {
	[0] = {
		.init_chip		= axp_init_chip,
		.enable_irqs	= axp_enable_irqs,
		.disable_irqs	= axp_disable_irqs,
		.read_irqs		= axp_read_irqs,
		.clear_irqs		= axp_clear_irqs,
	},
};
#if defined (CONFIG_KP_USEIRQ)
static void axp_mfd_irq_work(struct work_struct *work)
{
	struct axp_mfd_chip *chip =
		container_of(work, struct axp_mfd_chip, irq_work);
	uint64_t irqs = 0;

	while (1) {
		if (chip->ops->read_irqs(chip, &irqs))
			break;

		irqs &= chip->irqs_enabled;
		if (irqs == 0)
			break;
		blocking_notifier_call_chain(
				&chip->notifier_list, irqs, NULL);
	}
	enable_irq(chip->client->irq);
}

static irqreturn_t axp_mfd_irq_handler(int irq, void *data)
{
	struct axp_mfd_chip *chip = data;
	//printk("axp_mfd_irq_handler\n");
	disable_irq_nosync(irq);
	(void)schedule_work(&chip->irq_work);

	return IRQ_HANDLED;
}
#endif

static const struct i2c_device_id axp_mfd_id_table[] = {
	{ "axp_mfd", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, axp_mfd_id_table);

static int __remove_subdev(struct device *dev, void *unused)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static int axp_mfd_remove_subdevs(struct axp_mfd_chip *chip)
{
	return device_for_each_child(chip->dev, NULL, __remove_subdev);
}

static int __devinit axp_mfd_add_subdevs(struct axp_mfd_chip *chip,
					struct axp_platform_data *pdata)
{
	struct axp_funcdev_info *regl_dev;
	struct axp_funcdev_info *sply_dev;
	struct platform_device *pdev;
	int i, ret = 0;
	printk("%s:%d:start...\n", __func__, __LINE__);
	/* register for regultors */
	for (i = 0; i < pdata->num_regl_devs; i++) {
		regl_dev = &pdata->regl_devs[i];
		pdev = platform_device_alloc(regl_dev->name, regl_dev->id);
		if (pdev == NULL ) {
			printk("%s:%d: platform_device_alloc error!", __func__, __LINE__);
			goto failed;
		}
		pdev->dev.parent = chip->dev;
		pdev->dev.platform_data = regl_dev->platform_data;
		ret = platform_device_add(pdev);
		if (ret) {
			printk("%s:%d: platform_device_add failured!", __func__, __LINE__);
			goto failed;
		}
	}

	/* register for power supply */
	for (i = 0; i < pdata->num_sply_devs; i++) {
		sply_dev = &pdata->sply_devs[i];
		pdev = platform_device_alloc(sply_dev->name, sply_dev->id);
		if (pdev == NULL ) {
			printk("%s:%d: platform_device_alloc error!", __func__, __LINE__);
			goto failed;
		}
		pdev->dev.parent = chip->dev;
		pdev->dev.platform_data = sply_dev->platform_data;
		ret = platform_device_add(pdev);
		if (ret) {
			printk("%s:%d: platform_device_add failured!", __func__, __LINE__);
			goto failed;
		}
	}

	return 0;

failed:
	axp_mfd_remove_subdevs(chip);
	return ret;
}
/* 系统软件关机函数 */
void axp_power_off(void)
{
	uint8_t val;
	/* 清库仑计 */
#if defined (CONFIG_USE_OCV)
	axp_read(&axp->dev, AXP_COULOMB_CTL, &val);
	val &= 0x3f;
	axp_write(&axp->dev, AXP_COULOMB_CTL, val);
	val |= 0x80;
	val &= 0xbf;
	axp_write(&axp->dev, AXP_COULOMB_CTL, val);
#endif

	printk("[axp] send power-off command!\n");
	mdelay(20);
	/* 需要做插入火牛、usb关机重启进boot时加入下面一段代码，不需要就注释掉 */
//	axp_read(&axp->dev, AXP_STATUS, &val);
//	if(val & 0xF0){
//		axp_read(&axp->dev, AXP_MODE_CHGSTATUS, &val);
//		if(val & 0x20){
//			printk("[axp] set flag!\n");
//			mdelay(20);
//			printk("[axp] reboot!\n");
//			arch_reset(0,NULL);
//			printk("[axp] warning!!! arch can't ,reboot, maybe some error happend!\n");
//		}
//	}
	axp_set_bits(&axp->dev, AXP_OFF_CTL, 0x80);
	mdelay(20);
	printk("[axp] warning!!! axp can't power-off, maybe some error happend!\n");
}
 
extern int platform_data_int;
static int __devinit axp_mfd_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct axp_platform_data *pdata = client->dev.platform_data;
	struct axp_mfd_chip *chip;
	int ret;
	if (pdata == NULL) {
		printk("%s:%d: platform_data is null\n", __func__, __LINE__);
		pdata = (struct axp_platform_data *)platform_data_int;
		//return -1;
	} 
	chip = kzalloc(sizeof(struct axp_mfd_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;
		
	axp = client;

	chip->client = client;
	chip->dev = &client->dev;
	chip->ops = &axp_mfd_ops[id->driver_data];

	mutex_init(&chip->lock);

	i2c_set_clientdata(client, chip);

	ret = chip->ops->init_chip(chip);
	if (ret)
		goto out_free_chip;

#if defined (CONFIG_KP_USEIRQ)
	INIT_WORK(&chip->irq_work, axp_mfd_irq_work);
	BLOCKING_INIT_NOTIFIER_HEAD(&chip->notifier_list);
	client->irq = gpio_to_irq(client->irq);
	ret = request_irq(client->irq, axp_mfd_irq_handler,
		IRQF_DISABLED, "axp_mfd", chip);
  	if (ret) {
  		dev_err(&client->dev, "failed to request irq %d\n",
  				client->irq);
  		goto out_free_chip;
  	}
	enable_irq_wake(client->irq);
#endif
	printk("axp_mfd_add_subdevs before\n");
	ret = axp_mfd_add_subdevs(chip, pdata);
	if (ret)
		goto out_free_irq;
 	printk("axp_mfd_add_subdevs end\n");
	/* PM hookup */
	if(!pm_power_off)
		pm_power_off = axp_power_off;

	printk("%s succseefull!\n", __func__);
	return ret;

out_free_irq:
	#if defined (CONFIG_KP_USEIRQ)
	free_irq(client->irq, chip);
	#endif

out_free_chip:
	i2c_set_clientdata(client, NULL);
	kfree(chip);

	return ret;
}

static int __devexit axp_mfd_remove(struct i2c_client *client)
{
	struct axp_mfd_chip *chip = i2c_get_clientdata(client);

	pm_power_off = NULL;
	axp = NULL;

	axp_mfd_remove_subdevs(chip);
	kfree(chip);
	return 0;
}
/*
static const unsigned short axp_i2c[] = {
	AXP_DEVICES_ADDR, I2C_CLIENT_END };
*/
static struct i2c_driver axp_mfd_driver = {
	//.class = I2C_CLASS_HWMON,
	.driver	= {
		.name	= "axp_mfd",
		.owner	= THIS_MODULE,
	},
	.probe		= axp_mfd_probe,
	.remove		= __devexit_p(axp_mfd_remove),
	.id_table	= axp_mfd_id_table,
	//.detect         = axp_detect,
	//.address_list   = axp_i2c,
};

static int __init axp_mfd_init(void)
{
	return i2c_add_driver(&axp_mfd_driver);
}
subsys_initcall(axp_mfd_init);

static void __exit axp_mfd_exit(void)
{
	i2c_del_driver(&axp_mfd_driver);
}
module_exit(axp_mfd_exit);

MODULE_DESCRIPTION("MFD Driver for X-Powers AXP PMIC");
MODULE_AUTHOR("Donglu Zhang, <zhangdonglu@x-powers.com>");
MODULE_LICENSE("GPL");