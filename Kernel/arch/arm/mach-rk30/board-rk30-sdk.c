/* arch/arm/mach-rk30/board-rk30-sdk.c
 *
 * Copyright (C) 2012 ROCKCHIP, Inc.
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

#include <linux/wisky.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/mmc/host.h>
#include <linux/ion.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <mach/dvfs.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/hardware/gic.h>

#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <linux/fb.h>
#include <linux/regulator/machine.h>
#include <linux/rfkill-rk.h>
#include <linux/sensor-dev.h>
#include <linux/switch.h>

#ifdef CONFIG_RK30_PWM_REGULATOR
#include <linux/regulator/rk29-pwm-regulator.h>
#endif

#if defined(CONFIG_HDMI_RK30)
	#include "../../../drivers/video/rockchip/hdmi/rk_hdmi.h"
#endif

#if defined(CONFIG_SPIM_RK29)
#include "../../../drivers/spi/rk29_spim.h"
#endif
#if defined(CONFIG_MU509)
#include <linux/mu509.h>
#endif
#if defined(CONFIG_MW100)
#include <linux/mw100.h>
#endif
#if defined(CONFIG_MT6229)
#include <linux/mt6229.h>
#endif
#if defined(CONFIG_SEW868)
#include <linux/sew868.h>
#endif
#if defined(CONFIG_ANDROID_TIMED_GPIO)
#include "../../../drivers/staging/android/timed_gpio.h"
#endif

#if defined(WISKY_VIBRATOR_TIMED_GPIO)
#include "../../../wisky/vibrator/wisky_vibrator_timed_gpio.h"
#endif

#ifdef  CONFIG_THREE_FB_BUFFER
#define RK30_FB0_MEM_SIZE 12*SZ_1M
#else
#define RK30_FB0_MEM_SIZE 8*SZ_1M
#endif
int lcd_shutdown_flag =0;
extern void rk610_lcd_suspend(void);
extern void rk610_lcd_resume(void);

#ifdef CONFIG_VIDEO_RK29
//wisky-lxh@20120913,for camera compatible
int f_cam_probe_success_flag = 0;
int b_cam_probe_success_flag = 0;
EXPORT_SYMBOL(f_cam_probe_success_flag);
EXPORT_SYMBOL(b_cam_probe_success_flag);
//end-wisky-lxh@20120913
/*---------------- Camera Sensor Macro Define Begin  ------------------------*/
/*---------------- Camera Sensor Configuration Macro Begin ------------------------*/
#define CONFIG_SENSOR_0 	WISKY_CAMERA_BACK_NAME//RK29_CAM_SENSOR_HI253						/* back camera sensor */
#define CONFIG_SENSOR_IIC_ADDR_0		WISKY_CAMERA_BACK_ADDR//0x78
#define CONFIG_SENSOR_IIC_ADAPTER_ID_0	  3
#define CONFIG_SENSOR_CIF_INDEX_0                    0
#define CONFIG_SENSOR_ORIENTATION_0 	  90//270
#define CONFIG_SENSOR_POWER_PIN_0		  INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_0		  INVALID_GPIO//RK30_PIN1_PC7
#define CONFIG_SENSOR_POWERDN_PIN_0 	CAMERA_BACK_PDN_PIN
#define CONFIG_SENSOR_FALSH_PIN_0		  INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_0 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_0 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_0 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_0 RK29_CAM_FLASHACTIVE_L

#define CONFIG_SENSOR_QCIF_FPS_FIXED_0		15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_0   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_0		15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_0		15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_0		15000
#define CONFIG_SENSOR_480P_FPS_FIXED_0		15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_0		15000
#define CONFIG_SENSOR_720P_FPS_FIXED_0		30000

#define CONFIG_SENSOR_01  RK29_CAM_SENSOR_OV5642                   /* back camera sensor 1 */
#define CONFIG_SENSOR_IIC_ADDR_01 	    0x00
#define CONFIG_SENSOR_CIF_INDEX_01                    1
#define CONFIG_SENSOR_IIC_ADAPTER_ID_01    3
#define CONFIG_SENSOR_ORIENTATION_01       90
#define CONFIG_SENSOR_POWER_PIN_01         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_01         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_01       INVALID_GPIO// CAMERA_BACK_PDN_PIN
#define CONFIG_SENSOR_FALSH_PIN_01         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_01 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_01 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_01 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_01 RK29_CAM_FLASHACTIVE_L

#define CONFIG_SENSOR_QCIF_FPS_FIXED_01      15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_01   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_01      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_01       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_01       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_01      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_01      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_01     30000

#define CONFIG_SENSOR_02 RK29_CAM_SENSOR_OV5640                      /* back camera sensor 2 */
#define CONFIG_SENSOR_IIC_ADDR_02 	    0x00
#define CONFIG_SENSOR_CIF_INDEX_02                    1
#define CONFIG_SENSOR_IIC_ADAPTER_ID_02    4
#define CONFIG_SENSOR_ORIENTATION_02       90
#define CONFIG_SENSOR_POWER_PIN_02         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_02         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_02       INVALID_GPIO//CAMERA_BACK_PDN_PIN
#define CONFIG_SENSOR_FALSH_PIN_02         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_02 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_02 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_02 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_02 RK29_CAM_FLASHACTIVE_L

#define CONFIG_SENSOR_QCIF_FPS_FIXED_02      15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_02   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_02      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_02       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_02       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_02      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_02      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_02      30000

#define CONFIG_SENSOR_1 	WISKY_CAMERA_FRONT_NAME0// nc//RK29_CAM_SENSOR_HI704                      /* front camera sensor 0 */
#define CONFIG_SENSOR_IIC_ADDR_1 	   WISKY_CAMERA_FRONT_ADDR0//0x00
#define CONFIG_SENSOR_IIC_ADAPTER_ID_1	  3
#define CONFIG_SENSOR_CIF_INDEX_1				  0
#define CONFIG_SENSOR_ORIENTATION_1       0//90
#define CONFIG_SENSOR_POWER_PIN_1         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_1         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_1 	  CAMERA_FRONT_PDN_PIN// CAMERA_FRONT_PDN_PIN
#define CONFIG_SENSOR_FALSH_PIN_1         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_1 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_1 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_1 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_1 RK29_CAM_FLASHACTIVE_L

#define CONFIG_SENSOR_QCIF_FPS_FIXED_1		15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_1   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_1		15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_1		15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_1		15000
#define CONFIG_SENSOR_480P_FPS_FIXED_1		15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_1		15000
#define CONFIG_SENSOR_720P_FPS_FIXED_1		30000


#define CONFIG_SENSOR_11		nc//WISKY_CAMERA_FRONT_NAME0                     /* front camera sensor 1 */
#define CONFIG_SENSOR_IIC_ADDR_11 	   0x00// WISKY_CAMERA_FRONT_ADDR0
#define CONFIG_SENSOR_IIC_ADAPTER_ID_11    3
#define CONFIG_SENSOR_CIF_INDEX_11				  0
#define CONFIG_SENSOR_ORIENTATION_11       270
#define CONFIG_SENSOR_POWER_PIN_11         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_11         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_11      INVALID_GPIO// CAMERA_FRONT_PDN_PIN//RK30_PIN1_PB7
#define CONFIG_SENSOR_FALSH_PIN_11         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_11 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_11 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_11 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_11 RK29_CAM_FLASHACTIVE_L

#define CONFIG_SENSOR_QCIF_FPS_FIXED_11      15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_11   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_11      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_11       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_11       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_11      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_11      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_11      30000

#define CONFIG_SENSOR_12 WISKY_CAMERA_FRONT_NAME1//RK29_CAM_SENSOR_OV2655                      /* front camera sensor 2 */
#define CONFIG_SENSOR_IIC_ADDR_12 	   0x00
#define CONFIG_SENSOR_IIC_ADAPTER_ID_12    3
#define CONFIG_SENSOR_CIF_INDEX_12				  0
#define CONFIG_SENSOR_ORIENTATION_12       270
#define CONFIG_SENSOR_POWER_PIN_12         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_12         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_12       INVALID_GPIO//CAMERA_FRONT_PDN_PIN//RK30_PIN1_PB7
#define CONFIG_SENSOR_FALSH_PIN_12         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_12 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_12 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_12 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_12 RK29_CAM_FLASHACTIVE_L

#define CONFIG_SENSOR_QCIF_FPS_FIXED_12      15000
#define CONFIG_SENSOR_240X160_FPS_FIXED_12   15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_12      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_12       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_12       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_12      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_12      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_12      30000


#endif  //#ifdef CONFIG_VIDEO_RK29
/*---------------- Camera Sensor Configuration Macro End------------------------*/
#include "../../../drivers/media/video/rk30_camera.c"
/*---------------- Camera Sensor Macro Define End  ---------*/

#define PMEM_CAM_SIZE PMEM_CAM_NECESSARY
/*****************************************************************************************
 * camera  devices
 * author: ddl@rock-chips.com
 *****************************************************************************************/
#ifdef CONFIG_VIDEO_RK29
#define CONFIG_SENSOR_POWER_IOCTL_USR	   1 //define this refer to your board layout
#define CONFIG_SENSOR_RESET_IOCTL_USR	   0
#define CONFIG_SENSOR_POWERDOWN_IOCTL_USR	   0
#define CONFIG_SENSOR_FLASH_IOCTL_USR	   0

static void rk_cif_power(int on)
{
    struct regulator *ldo_18,*ldo_28;
	ldo_28 = regulator_get(NULL, "ldo7");	// vcc28_cif
	ldo_18 = regulator_get(NULL, "ldo1");	// vcc18_cif
	if (ldo_28 == NULL || IS_ERR(ldo_28) || ldo_18 == NULL || IS_ERR(ldo_18)){
        printk("get cif ldo failed!\n");
		return;
	    }
    if(on == 0){	
    	regulator_disable(ldo_28);
    	regulator_put(ldo_28);
    	regulator_disable(ldo_18);
    	regulator_put(ldo_18);
    	mdelay(500);
        }
    else{
    	regulator_set_voltage(ldo_28, 2800000, 2800000);
    	regulator_enable(ldo_28);
   // 	printk("%s set ldo7 vcc28_cif=%dmV end\n", __func__, regulator_get_voltage(ldo_28));
    	regulator_put(ldo_28);

    	regulator_set_voltage(ldo_18, 1800000, 1800000);
    //	regulator_set_suspend_voltage(ldo, 1800000);
    	regulator_enable(ldo_18);
    //	printk("%s set ldo1 vcc18_cif=%dmV end\n", __func__, regulator_get_voltage(ldo_18));
    	regulator_put(ldo_18);
        }
}

static void rk_cif_power_axp(int on)
{	
	
	#if 1
	if(on == 0){
		#if defined(CAMERA_PWR_EN_PIN)
		gpio_set_value(CAMERA_PWR_EN_PIN, !CAMERA_PWR_EN_VALUE);
		mdelay(500);
		#endif
	}else{
		#if defined(CAMERA_PWR_EN_PIN)
		
		gpio_set_value(CAMERA_PWR_EN_PIN, CAMERA_PWR_EN_VALUE);
		#endif
	}
	#endif
	
}

#if CONFIG_SENSOR_POWER_IOCTL_USR
static int sensor_power_usr_cb (struct rk29camera_gpio_res *res,int on)
{
	//#error "CONFIG_SENSOR_POWER_IOCTL_USR is 1, sensor_power_usr_cb function must be writed!!";
#if defined(CONFIG_KP_AXP)
	rk_cif_power_axp(on);
#else
	rk_cif_power(on);
#endif
}
#endif

#if CONFIG_SENSOR_RESET_IOCTL_USR
static int sensor_reset_usr_cb (struct rk29camera_gpio_res *res,int on)
{
	#error "CONFIG_SENSOR_RESET_IOCTL_USR is 1, sensor_reset_usr_cb function must be writed!!";
}
#endif

#if CONFIG_SENSOR_POWERDOWN_IOCTL_USR
static int sensor_powerdown_usr_cb (struct rk29camera_gpio_res *res,int on)
{
	#error "CONFIG_SENSOR_POWERDOWN_IOCTL_USR is 1, sensor_powerdown_usr_cb function must be writed!!";
}
#endif

#if CONFIG_SENSOR_FLASH_IOCTL_USR
static int sensor_flash_usr_cb (struct rk29camera_gpio_res *res,int on)
{
	#error "CONFIG_SENSOR_FLASH_IOCTL_USR is 1, sensor_flash_usr_cb function must be writed!!";
}
#endif

static struct rk29camera_platform_ioctl_cb	sensor_ioctl_cb = {
	#if CONFIG_SENSOR_POWER_IOCTL_USR
	.sensor_power_cb = sensor_power_usr_cb,
	#else
	.sensor_power_cb = NULL,
	#endif

	#if CONFIG_SENSOR_RESET_IOCTL_USR
	.sensor_reset_cb = sensor_reset_usr_cb,
	#else
	.sensor_reset_cb = NULL,
	#endif

	#if CONFIG_SENSOR_POWERDOWN_IOCTL_USR
	.sensor_powerdown_cb = sensor_powerdown_usr_cb,
	#else
	.sensor_powerdown_cb = NULL,
	#endif

	#if CONFIG_SENSOR_FLASH_IOCTL_USR
	.sensor_flash_cb = sensor_flash_usr_cb,
	#else
	.sensor_flash_cb = NULL,
	#endif
};

#if CONFIG_SENSOR_IIC_ADDR_0
static struct reginfo_t rk_init_data_sensor_reg_0[] =
{
		{0x0000, 0x00,0,0}
	};
static struct reginfo_t rk_init_data_sensor_winseqreg_0[] ={
	{0x0000, 0x00,0,0}
	};
#endif

#if CONFIG_SENSOR_IIC_ADDR_1
static struct reginfo_t rk_init_data_sensor_reg_1[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_1[] =
{
       {0x0000, 0x00,0,0}
};
#endif
#if CONFIG_SENSOR_IIC_ADDR_01
static struct reginfo_t rk_init_data_sensor_reg_01[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_01[] =
{
       {0x0000, 0x00,0,0}
};
#endif
#if CONFIG_SENSOR_IIC_ADDR_02
static struct reginfo_t rk_init_data_sensor_reg_02[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_02[] =
{
       {0x0000, 0x00,0,0}
};
#endif
#if CONFIG_SENSOR_IIC_ADDR_11
static struct reginfo_t rk_init_data_sensor_reg_11[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_11[] =
{
       {0x0000, 0x00,0,0}
};
#endif
#if CONFIG_SENSOR_IIC_ADDR_12
static struct reginfo_t rk_init_data_sensor_reg_12[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_12[] =
{
       {0x0000, 0x00,0,0}
};
#endif
static rk_sensor_user_init_data_s rk_init_data_sensor[RK_CAM_NUM] = 
{
    #if CONFIG_SENSOR_IIC_ADDR_0
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_0,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_0,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_0) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_0) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif
    #if CONFIG_SENSOR_IIC_ADDR_1
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_1,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_1,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_1) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_1) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif
    #if CONFIG_SENSOR_IIC_ADDR_01
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_01,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_01,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_01) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_01) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif
    #if CONFIG_SENSOR_IIC_ADDR_02
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_02,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_02,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_02) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_02) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif
    #if CONFIG_SENSOR_IIC_ADDR_11
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_11,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_11,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_11) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_11) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif
    #if CONFIG_SENSOR_IIC_ADDR_12
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_12,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_12,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_12) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_12) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif

 };
#include "../../../drivers/media/video/rk30_camera.c"

#endif /* CONFIG_VIDEO_RK29 */


#if defined(CONFIG_WISKY_BATTERY)
static int battery_io_init(void)
{
	int ret = 0;

#if defined(DC_DETECT_PIN)
	if((DC_DETECT_PIN >= 0)/*&&(DC_DETECT_PIN != USB_DETECT_PIN)*/){
		rk30_mux_api_set(DC_DETECT_MUX_NAME, DC_DETECT_MUX_MODE);
		ret = gpio_request(DC_DETECT_PIN, "DC detect");
		if(ret){
			pr_err("request dc detect gpio failed!\n");
			//return ret;
		}
		gpio_direction_input(DC_DETECT_PIN);
	}
#endif

#if defined(HOLE_DC_DETECT_PIN)
	if(HOLE_DC_DETECT_PIN >= 0){
		rk30_mux_api_set(HOLE_DC_DETECT_MUX_NAME, HOLE_DC_DETECT_MUX_MODE);
		ret = gpio_request(HOLE_DC_DETECT_PIN, "HOLE DC detect");
		if(ret){
			pr_err("request hole dc detect gpio failed!\n");
			//return ret;
		}
		gpio_direction_input(HOLE_DC_DETECT_PIN);
	}
#endif

#if defined(BATTERY_CHARGE_PWR_EN_PIN)
	if(BATTERY_CHARGE_PWR_EN_PIN != INVALID_GPIO){
		rk30_mux_api_set(BATTERY_CHARGE_PWR_EN_MUX_NAME, BATTERY_CHARGE_PWR_EN_MUX_MODE);
		ret = gpio_request(BATTERY_CHARGE_PWR_EN_PIN, "charge power supply enable");
		if(ret){
			pr_err("request battery charge power supply enable gpio failed!\n");
			//return ret;
		}
		gpio_direction_output(BATTERY_CHARGE_PWR_EN_PIN, !BATTERY_CHARGE_PWR_EN_VALUE);
	}
#endif

#if defined(BATTERY_CHARGE_EN_PIN)
	if(BATTERY_CHARGE_EN_PIN >= 0){
		rk30_mux_api_set(BATTERY_CHARGE_EN_MUX_NAME, BATTERY_CHARGE_EN_MUX_MODE);
		ret = gpio_request(BATTERY_CHARGE_EN_PIN, "battery charge en");
		if(ret){
			pr_err("request battery charge enable gpio failed!\n");
			//return ret;
		}
		gpio_direction_output(BATTERY_CHARGE_EN_PIN, !BATTERY_CHARGE_EN_VALUE);//disable battery charge
	}
#endif

#if defined(CHARGE_CURRENT_CTL1_PIN)
	if(CHARGE_CURRENT_CTL1_PIN >= 0){
		rk30_mux_api_set(CHARGE_CURRENT_CTL1_MUX_NAME, CHARGE_CURRENT_CTL1_MUX_MODE);
		ret = gpio_request(CHARGE_CURRENT_CTL1_PIN, "charge current ctrl 1");
		if(ret){
			pr_err("request battery charge current control 1 gpio failed!\n");
			//return ret;
		}
		gpio_direction_output(CHARGE_CURRENT_CTL1_PIN, !CHARGE_CURRENT_CTL1_ON_VALUE);
	}
#endif

#if defined(CHARGE_CURRENT_CTL2_PIN)
	if(CHARGE_CURRENT_CTL2_PIN >= 0){
		rk30_mux_api_set(CHARGE_CURRENT_CTL2_MUX_NAME, CHARGE_CURRENT_CTL2_MUX_MODE);
		ret = gpio_request(CHARGE_CURRENT_CTL2_PIN, "charge current ctrl 2");
		if(ret){
			pr_err("request battery charge current control 2 gpio failed!\n");
			//return ret;
		}
		gpio_direction_output(CHARGE_CURRENT_CTL2_PIN, !CHARGE_CURRENT_CTL2_ON_VALUE);
	}
#endif

#if defined(BATTERY_CHARGE_STAT1_PIN)
	if(BATTERY_CHARGE_STAT1_PIN != INVALID_GPIO){
		rk30_mux_api_set(BATTERY_CHARGE_STAT1_MUX_NAME, BATTERY_CHARGE_STAT1_MUX_MODE);
		ret = gpio_request(BATTERY_CHARGE_STAT1_PIN, "battery charge stat 1");
		if(ret){
			pr_err("request battery charge stat 1 gpio failed!\n");
			//return ret;
		}
		gpio_direction_input(BATTERY_CHARGE_STAT1_PIN);
	}
#endif

#if defined(BATTERY_CHARGE_STAT2_PIN)
	if(BATTERY_CHARGE_STAT2_PIN != INVALID_GPIO){
		rk30_mux_api_set(BATTERY_CHARGE_STAT2_MUX_NAME, BATTERY_CHARGE_STAT2_MUX_MODE);
		ret = gpio_request(BATTERY_CHARGE_STAT2_PIN, "battery charge stat 2");
		if(ret){
			pr_err("request battery charge stat 2 gpio failed!\n");
			//return ret;
		}
		gpio_direction_input(BATTERY_CHARGE_STAT2_PIN);
	}
#endif

	return ret;
}

static int battery_io_deinit(void)
{
	int ret = 0;

#if defined(DC_DETECT_PIN)
	if(DC_DETECT_PIN >= 0){
		gpio_free(DC_DETECT_PIN);
	}
#endif

#if defined(HOLE_DC_DETECT_PIN)
	if(HOLE_DC_DETECT_PIN >= 0){
		gpio_free(HOLE_DC_DETECT_PIN);
	}
#endif

#if defined(BATTERY_CHARGE_PWR_EN_PIN)
	if(BATTERY_CHARGE_PWR_EN_PIN != INVALID_GPIO){
		gpio_free(BATTERY_CHARGE_PWR_EN_PIN);
	}
#endif
#if defined(BATTERY_CHARGE_EN_PIN)
	if(BATTERY_CHARGE_EN_PIN >= 0){
		gpio_free(BATTERY_CHARGE_EN_PIN);
	}
#endif
#if defined(CHARGE_CURRENT_CTL1_PIN)
	if(CHARGE_CURRENT_CTL1_PIN >= 0){
		gpio_free(CHARGE_CURRENT_CTL1_PIN);
	}
#endif
#if defined(CHARGE_CURRENT_CTL2_PIN)
	if(CHARGE_CURRENT_CTL2_PIN >= 0){
		gpio_free(CHARGE_CURRENT_CTL2_PIN);
	}
#endif
#if defined(BATTERY_CHARGE_STAT1_PIN)
	if(BATTERY_CHARGE_STAT1_PIN != INVALID_GPIO){
		gpio_free(BATTERY_CHARGE_STAT1_PIN);
	}
#endif
#if defined(BATTERY_CHARGE_STAT2_PIN)
	if(BATTERY_CHARGE_STAT2_PIN != INVALID_GPIO){
		gpio_free(BATTERY_CHARGE_STAT2_PIN);
	}
#endif
	
	return ret;
}

struct battery_platform_data wisky_battery_platform_data = {
	.bat_adc_ch = BATTERY_ADC_CHANNEL,
	.dc_adc_ch = DC_ADC_CHANNEL,
	.hole_dc_adc_ch = HOLE_DC_ADC_CHANNEL,
	.io_init = battery_io_init,
	.io_deinit = battery_io_deinit,
};

struct platform_device wisky_device_battery = {
	.name = "wisky_battery",
	.id = -1,
	.dev = {
		.platform_data  = &wisky_battery_platform_data,
	}
};
#endif

#if defined(WISKY_SWITCH_GPIO)
static struct gpio_switch_platform_data wisky_switch_gpio_data = {
	.name = "h2w",//"wisky_switch",
	.gpio = HEADPHONE_DET_PIN,
	.mux_name = HEADPHONE_DET_MUX_NAME,
	.mux_mode = HEADPHONE_DET_MUX_MODE,
};

static struct platform_device wisky_switch_gpio_device = {
	.name = "switch-gpio",
	.id = -1,
	.dev = {
		.platform_data = &wisky_switch_gpio_data,
	},
};
#endif

#if defined(WISKY_VIBRATOR_TIMED_GPIO)
static struct timed_gpio wisky_vibrator_gpios = {
		.name = "vibrator",
		.gpio = VIBRATOR_EN_PIN,
		.mux_name = VIBRATOR_EN_MUX_NAME,
		.mux_mode = VIBRATOR_EN_MUX_MODE,
		.max_timeout = 10000, //10*000 ms
		.active_low = VIBRATOR_ACTIVE_LOW,
};

static struct timed_gpio_platform_data wisky_timed_gpio_data = {
	.num_gpios = 1,
	.gpios = &wisky_vibrator_gpios,
};

static struct platform_device wisky_vibrator_device = {
	.name = TIMED_GPIO_NAME,
	.id = -1,
	.dev = {
		.platform_data = &wisky_timed_gpio_data,
	},
};
#endif

#ifdef WISKY_TS_FT5306
#define TOUCH_PWR_PIN    			TS_POWER_PIN
#define TOUCH_PWR_MUX_NAME		TS_POWER_MUX_NAME
#define TOUCH_PWR_MUX_MODE		TS_POWER_MUX_MODE
#define TOUCH_PWR_VALUE			TS_POWER_ON_VALUE

#define TOUCH_RESET_PIN  			TS_RESET_PIN
#define TOUCH_RESET_MUX_NAME	TS_RESET_MUX_NAME
#define TOUCH_RESET_MUX_MODE	TS_RESET_MUX_MODE
#define TOUCH_RESET_VALUE		TS_RESET_VALUE

#define TOUCH_SHUTDOWN_PIN		TS_SHUTDOWN_PIN
#define TOUCH_SHUTDOWN_MUX_NAME	TS_SHUTDOWN_MUX_NAME
#define TOUCH_SHUTDOWN_MUX_MODE	TS_SHUTDOWN_MUX_MODE
#define TOUCH_SHUTDOWN_VALUE	TS_SHUTDOWN_VALUE
int ft5306_init_platform_hw(void)
{
	int ret;

	rk30_mux_api_set(TOUCH_RESET_MUX_NAME, TOUCH_RESET_MUX_MODE);
	if (TOUCH_RESET_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_RESET_PIN, "ft5306 reset pin");
		if (ret != 0) {
			gpio_free(TOUCH_RESET_PIN);
			printk("ft5306 gpio_request error\n");
			return -EIO;
		}
	}

	rk30_mux_api_set(TOUCH_SHUTDOWN_MUX_NAME, TOUCH_SHUTDOWN_MUX_MODE);
	if (TOUCH_SHUTDOWN_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_SHUTDOWN_PIN, "ft5306 shutdown pin");
		if (ret != 0) {
			gpio_free(TOUCH_SHUTDOWN_PIN);
			printk("ft5306 gpio_request shutdown error\n");
			return -EIO;
		}
	}

	gpio_direction_output(TOUCH_SHUTDOWN_PIN, !TOUCH_SHUTDOWN_VALUE);//wake up touch screen
	
	gpio_direction_output(TOUCH_RESET_PIN, GPIO_HIGH);
	mdelay(50);
	gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);
	mdelay(500);
	gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
	mdelay(100);

	return 0;
}

int ft5306_platform_sleep(void)
{
	struct regulator *ldo;

	//power supply off
	ldo = regulator_get(NULL, "ldo9");//VCC_TP
	regulator_disable(ldo);
	regulator_put(ldo);

	//reset pulldown
	gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);	
}

int ft5306_platform_wakeup(void)
{
	struct regulator *ldo;

	//power supply on
	ldo = regulator_get(NULL, "ldo9");//VCC_TP
//	regulator_set_voltage(ldo, 3300000, 3300000);
	regulator_enable(ldo);
	regulator_put(ldo);

	//reset up
#if 0
	gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
#else
	gpio_direction_output(TOUCH_SHUTDOWN_PIN, !TOUCH_SHUTDOWN_VALUE);//wake up touch screen
	
	gpio_direction_output(TOUCH_RESET_PIN, GPIO_HIGH);
	mdelay(50);
	gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);
	mdelay(500);
	gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
	mdelay(100);
#endif
}

struct ft5306_platform_data ft5306_info = {
	.model = 5306,
	.init_platform_hw = ft5306_init_platform_hw,
	.ft5306_platform_sleep = ft5306_platform_sleep,
	.ft5306_platform_wakeup = ft5306_platform_wakeup,
};
#endif

#ifdef WISKY_TS_FT5302
#define TOUCH_PWR_PIN    			TS_POWER_PIN
#define TOUCH_PWR_MUX_NAME		TS_POWER_MUX_NAME
#define TOUCH_PWR_MUX_MODE		TS_POWER_MUX_MODE
#define TOUCH_PWR_VALUE			TS_POWER_ON_VALUE

#define TOUCH_RESET_PIN  			TS_RESET_PIN
#define TOUCH_RESET_MUX_NAME	TS_RESET_MUX_NAME
#define TOUCH_RESET_MUX_MODE	TS_RESET_MUX_MODE
#define TOUCH_RESET_VALUE		TS_RESET_VALUE

#define TOUCH_SHUTDOWN_PIN		TS_SHUTDOWN_PIN
#define TOUCH_SHUTDOWN_MUX_NAME	TS_SHUTDOWN_MUX_NAME
#define TOUCH_SHUTDOWN_MUX_MODE	TS_SHUTDOWN_MUX_MODE
#define TOUCH_SHUTDOWN_VALUE	TS_SHUTDOWN_VALUE

int ft5302_init_platform_hw(void)
{
	int ret;

	rk30_mux_api_set(TOUCH_RESET_MUX_NAME, TOUCH_RESET_MUX_MODE);
	if (TOUCH_RESET_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_RESET_PIN, "ft5302 reset pin");
		if (ret != 0) {
			gpio_free(TOUCH_RESET_PIN);
			printk("ft5302 gpio_request error\n");
			return -EIO;
		}
	}

	rk30_mux_api_set(TOUCH_SHUTDOWN_MUX_NAME, TOUCH_SHUTDOWN_MUX_MODE);
	if (TOUCH_SHUTDOWN_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_SHUTDOWN_PIN, "ft5302 shutdown pin");
		if (ret != 0) {
			gpio_free(TOUCH_SHUTDOWN_PIN);
			printk("ft5302 gpio_request shutdown error\n");
			return -EIO;
		}
	}

	gpio_direction_output(TOUCH_SHUTDOWN_PIN, !TS_SHUTDOWN_VALUE);//wake up touch screen
	
	gpio_direction_output(TOUCH_RESET_PIN, GPIO_HIGH);
	mdelay(50);
	gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);
	mdelay(500);
	gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
	mdelay(100);

	return 0;
}

struct ft5302_platform_data ft5302_info = {
	.model = 5302,
	.init_platform_hw = ft5302_init_platform_hw,
};
#endif

#ifdef WISKY_TS_FT5X0X
#define TOUCH_PWR_PIN    			TS_POWER_PIN
#define TOUCH_PWR_MUX_NAME		TS_POWER_MUX_NAME
#define TOUCH_PWR_MUX_MODE		TS_POWER_MUX_MODE
#define TOUCH_PWR_VALUE			TS_POWER_ON_VALUE

#define TOUCH_RESET_PIN  			TS_RESET_PIN
#define TOUCH_RESET_MUX_NAME	TS_RESET_MUX_NAME
#define TOUCH_RESET_MUX_MODE	TS_RESET_MUX_MODE
#define TOUCH_RESET_VALUE		TS_RESET_VALUE

#define TOUCH_SHUTDOWN_PIN		TS_SHUTDOWN_PIN
#define TOUCH_SHUTDOWN_MUX_NAME	TS_SHUTDOWN_MUX_NAME
#define TOUCH_SHUTDOWN_MUX_MODE	TS_SHUTDOWN_MUX_MODE
#define TOUCH_SHUTDOWN_VALUE	TS_SHUTDOWN_VALUE
int ft5x0x_init_platform_hw(void)
{
	int ret;

	//wisky libai 20120924
	#ifdef WISKY_BOARD_M101B_TV10
	  rk30_mux_api_set(TOUCH_RESET_MUX_NAME, TOUCH_RESET_MUX_MODE);
	  gpio_direction_output(TOUCH_RESET_PIN, GPIO_HIGH);
		mdelay(10);
  	#endif 
	
	if (TOUCH_PWR_PIN != INVALID_GPIO) {
		rk30_mux_api_set(TS_POWER_MUX_NAME, TS_POWER_MUX_MODE);
		ret = gpio_request(TOUCH_PWR_PIN, "ft5x0x power pin");
		if (ret != 0) {
			gpio_free(TOUCH_PWR_PIN);
			printk("ft5x0x power error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_PWR_PIN, TS_POWER_ON_VALUE);
		msleep(300); //wisky libai 20120924 
	}
	//wisky libai 20120924 
	#ifdef WISKY_BOARD_M101B_TV10
  	  return 0;
  	#endif
   //end
	rk30_mux_api_set(TOUCH_RESET_MUX_NAME, TOUCH_RESET_MUX_MODE);
	if (TOUCH_RESET_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_RESET_PIN, "ft5x0x reset pin");
		if (ret != 0) {
			gpio_free(TOUCH_RESET_PIN);
			printk("ft5x0x gpio_request error\n");
			return -EIO;
		}
	}

	rk30_mux_api_set(TOUCH_SHUTDOWN_MUX_NAME, TOUCH_SHUTDOWN_MUX_MODE);
	if (TOUCH_SHUTDOWN_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_SHUTDOWN_PIN, "ft5x0x shutdown pin");
		if (ret != 0) {
			gpio_free(TOUCH_SHUTDOWN_PIN);
			printk("ft5x0x gpio_request shutdown error\n");
			return -EIO;
		}
	}
	if (TOUCH_SHUTDOWN_PIN != INVALID_GPIO) {
		gpio_direction_output(TOUCH_SHUTDOWN_PIN, !TOUCH_SHUTDOWN_VALUE);//wake up touch screen
	}
	gpio_direction_output(TOUCH_RESET_PIN, GPIO_HIGH);
	mdelay(50);
	gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);
	mdelay(500);
	gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
	mdelay(100);

	return 0;
}

int ft5x0x_platform_sleep(void)
{
	struct regulator *ldo;

	#if defined (CONFIG_MFD_WM831X_I2C)
	//power supply off
	ldo = regulator_get(NULL, "ldo9");//VCC_TP
	regulator_disable(ldo);
	regulator_put(ldo);
	#endif
	//reset pulldown
	//wisky-lxh@20120801,powr off TP,
	gpio_set_value(TOUCH_PWR_PIN, GPIO_LOW);
	//end-wisky-lxh
	gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);

	return 0;
}

int ft5x0x_platform_wakeup(void)
{
	struct regulator *ldo;

	//power supply on
	#if defined (CONFIG_MFD_WM831X_I2C)
	ldo = regulator_get(NULL, "ldo9");//VCC_TP
//	regulator_set_voltage(ldo, 3300000, 3300000);
	regulator_enable(ldo);
	regulator_put(ldo);
    #endif
	//reset up
#if 0
	gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
#else
	//wisky-lxh@20120801,powr on TP,
	gpio_set_value(TOUCH_PWR_PIN, TS_POWER_ON_VALUE);
		msleep(10);
	//wisky-lxh@20120801	
	if (TOUCH_SHUTDOWN_PIN != INVALID_GPIO) {
		gpio_direction_output(TOUCH_SHUTDOWN_PIN, !TOUCH_SHUTDOWN_VALUE);//wake up touch screen
	}
	gpio_request(TOUCH_RESET_PIN, "ft5x0x reset pin");
	gpio_direction_output(TOUCH_RESET_PIN, GPIO_HIGH);
	mdelay(50);
	gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);
  mdelay(500);
	gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
  mdelay(100);
#endif
	return 0;
}

void ft5x0x_exit_platform_hw(void)
{
	//wisky-lxh@20120905,free any gpio resource,
	if (TOUCH_RESET_PIN != INVALID_GPIO)
		gpio_free(TOUCH_RESET_PIN);
	if (TOUCH_SHUTDOWN_PIN != INVALID_GPIO)
		gpio_free(TOUCH_SHUTDOWN_PIN);	
	if (TOUCH_PWR_PIN != INVALID_GPIO)
		gpio_free(TOUCH_PWR_PIN);	
	//end-wisky-lxh@20120905
}

struct ft5x0x_platform_data ft5x0x_info = {
	.model = 5000,
	.pwr_pin = TOUCH_PWR_PIN,
	.pwr_on_value = TOUCH_PWR_VALUE,
	.reset_pin = TOUCH_RESET_PIN,
	.reset_value = TOUCH_RESET_VALUE,
	.init_platform_hw = ft5x0x_init_platform_hw,
	.ft5x0x_platform_sleep = ft5x0x_platform_sleep,
	.ft5x0x_platform_wakeup = ft5x0x_platform_wakeup,
	.exit_platform_hw = ft5x0x_exit_platform_hw
};
#endif

#ifdef WISKY_TS_ZET6221
#define TOUCH_PWR_PIN    			TS_POWER_PIN
#define TOUCH_PWR_MUX_NAME		TS_POWER_MUX_NAME
#define TOUCH_PWR_MUX_MODE		TS_POWER_MUX_MODE
#define TOUCH_PWR_VALUE			TS_POWER_ON_VALUE

#define TOUCH_RESET_PIN  			TS_RESET_PIN
#define TOUCH_RESET_MUX_NAME	TS_RESET_MUX_NAME
#define TOUCH_RESET_MUX_MODE	TS_RESET_MUX_MODE
#define TOUCH_RESET_VALUE		TS_RESET_VALUE

#define TOUCH_SHUTDOWN_PIN		TS_SHUTDOWN_PIN
#define TOUCH_SHUTDOWN_MUX_NAME	TS_SHUTDOWN_MUX_NAME
#define TOUCH_SHUTDOWN_MUX_MODE	TS_SHUTDOWN_MUX_MODE
#define TOUCH_SHUTDOWN_VALUE	TS_SHUTDOWN_VALUE
int zet6221_ts_init_platform_hw(void)
{
	int ret;

	if (TOUCH_PWR_PIN != INVALID_GPIO) {
		rk30_mux_api_set(TS_POWER_MUX_NAME, TS_POWER_MUX_MODE);
		ret = gpio_request(TOUCH_PWR_PIN, "zet6221 power pin");
		if (ret != 0) {
			gpio_free(TOUCH_PWR_PIN);
			printk("zet6221 power error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_PWR_PIN, TS_POWER_ON_VALUE);
		msleep(100);
	}
	
	rk30_mux_api_set(TOUCH_RESET_MUX_NAME, TOUCH_RESET_MUX_MODE);
	if (TOUCH_RESET_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_RESET_PIN, "ts_model reset pin");
		if (ret != 0) {
			gpio_free(TOUCH_RESET_PIN);
			printk("ts_model gpio_request reset error\n");
			return -EIO;
		}
	}

	rk30_mux_api_set(TOUCH_SHUTDOWN_MUX_NAME, TOUCH_SHUTDOWN_MUX_MODE);
	if (TOUCH_SHUTDOWN_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_SHUTDOWN_PIN, "ts_model shutdown pin");
		if (ret != 0) {
			gpio_free(TOUCH_RESET_PIN);
			gpio_free(TOUCH_SHUTDOWN_PIN);
			printk("ts_model gpio_request shutdown error\n");
			return -EIO;
		}
	}
	if (TOUCH_SHUTDOWN_PIN != INVALID_GPIO) {
		gpio_direction_output(TOUCH_SHUTDOWN_PIN, !TOUCH_SHUTDOWN_VALUE);//wake up touch screen
	}
	gpio_direction_output(TOUCH_RESET_PIN, GPIO_HIGH);
	mdelay(50);
	gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);
	mdelay(100);
	gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
	mdelay(300);

	return 0;
}

void zet6221_ts_reset(void)
{
	gpio_direction_output(TOUCH_RESET_PIN, GPIO_HIGH);
	mdelay(50);
	gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);
	mdelay(100);
	gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
	mdelay(300);
}

int zet6221_ts_set_reset_pin(int gpio_state)
{
	if (!(gpio_state == 0 || gpio_state == 1)) {
		printk("%s:%d arg invalid!\n", __func__, __LINE__);
		return -1;
	}
	gpio_set_value(TOUCH_RESET_PIN, gpio_state);
	mdelay(50);
	return 0;
}

int zet6221_ts_platform_sleep(void)
{
	struct regulator *ldo;
	//printk("%s called!\n", __func__);

	#if defined (CONFIG_MFD_WM831X_I2C)
	//power supply off
	ldo = regulator_get(NULL, "ldo9");//VCC_TP
	regulator_disable(ldo);
	regulator_put(ldo);
	#endif
	//reset pulldown
	gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);

	return 0;
}

int zet6221_ts_platform_wakeup(void)
{
	struct regulator *ldo;

	//printk("%s called!\n", __func__);
	//power supply on
	#if defined (CONFIG_MFD_WM831X_I2C)
	ldo = regulator_get(NULL, "ldo9");//VCC_TP
//	regulator_set_voltage(ldo, 3300000, 3300000);
	regulator_enable(ldo);
	regulator_put(ldo);
    #endif
	//reset up
#if 0
	gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
#else
	if (TOUCH_SHUTDOWN_PIN != INVALID_GPIO) {
		gpio_direction_output(TOUCH_SHUTDOWN_PIN, !TOUCH_SHUTDOWN_VALUE);//wake up touch screen
	}
	
	gpio_direction_output(TOUCH_RESET_PIN, GPIO_HIGH);
	mdelay(50);
	gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);
	mdelay(50);//mdelay(500);
	gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
	mdelay(300);//mdelay(100);
#endif
	return 0;
}

void zet6221_ts_exit_platform_hw(void)
{
	//wisky-lxh@20120905,free any gpio resource
	if (TOUCH_RESET_PIN != INVALID_GPIO)
		gpio_free(TOUCH_RESET_PIN);
	if (TOUCH_PWR_PIN!= INVALID_GPIO)
		gpio_free(TOUCH_PWR_PIN);
	if (TOUCH_SHUTDOWN_PIN != INVALID_GPIO) {
		gpio_free(TOUCH_SHUTDOWN_PIN);
	}
	//end-wisky-lxh@20120905
}
struct zet6221_ts_platform_data zet6221_ts_info = {
	.model = 5000,
	.pwr_pin = TOUCH_PWR_PIN,
	.pwr_on_value = TOUCH_PWR_VALUE,
	.reset_pin = TOUCH_RESET_PIN,
	.reset_value = TOUCH_RESET_VALUE,
	.init_platform_hw = zet6221_ts_init_platform_hw,
	.set_reset_pin = zet6221_ts_set_reset_pin,
	.zet6221_ts_reset = zet6221_ts_reset,
	.zet6221_ts_platform_sleep = zet6221_ts_platform_sleep,
	.zet6221_ts_platform_wakeup = zet6221_ts_platform_wakeup,
	.exit_platform_hw = zet6221_ts_exit_platform_hw,
};
#endif


#ifdef WISKY_TS_GT8XX
#define TOUCH_RESET_PIN  			TS_RESET_PIN
#define TOUCH_RESET_MUX_NAME	TS_RESET_MUX_NAME
#define TOUCH_RESET_MUX_MODE	TS_RESET_MUX_MODE
#define TOUCH_RESET_VALUE		TS_RESET_VALUE

int gt82x_init_platform_hw(void)
{
	int ret;

	rk30_mux_api_set(TOUCH_RESET_MUX_NAME, TOUCH_RESET_MUX_MODE);
	if (TOUCH_RESET_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_RESET_PIN, "gt82x reset pin");
		if (ret != 0) {
			gpio_free(TOUCH_RESET_PIN);
			printk("gt82x gpio_request error\n");
			return -EIO;
		}
	}
	
	gpio_direction_output(TOUCH_RESET_PIN, TS_RESET_VALUE);

	return 0;
}

void gt82x_exit_platform_hw(void)
{
	if (TOUCH_RESET_PIN != INVALID_GPIO)
	gpio_free(TOUCH_RESET_PIN);
}

struct gt82x_platform_data gt82x_info = {
	.model = 828,
	.init_platform_hw = gt82x_init_platform_hw,
	.exit_platform_hw = gt82x_exit_platform_hw,
};
#endif
//wisky[e]

#if defined(CONFIG_TOUCHSCREEN_GT8XX)
#define TOUCH_RESET_PIN  RK30_PIN4_PD0
#define TOUCH_PWR_PIN    INVALID_GPIO
int goodix_init_platform_hw(void)
{
	int ret;
	
	rk30_mux_api_set(GPIO4D0_SMCDATA8_TRACEDATA8_NAME, GPIO4D_GPIO4D0);
	rk30_mux_api_set(GPIO4C2_SMCDATA2_TRACEDATA2_NAME, GPIO4C_GPIO4C2);
	printk("%s:0x%x,0x%x\n",__func__,rk30_mux_api_get(GPIO4D0_SMCDATA8_TRACEDATA8_NAME),rk30_mux_api_get(GPIO4C2_SMCDATA2_TRACEDATA2_NAME));

	if (TOUCH_PWR_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_PWR_PIN, "goodix power pin");
		if (ret != 0) {
			gpio_free(TOUCH_PWR_PIN);
			printk("goodix power error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_PWR_PIN, 0);
		gpio_set_value(TOUCH_PWR_PIN, GPIO_LOW);
		msleep(100);
	}

	if (TOUCH_RESET_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_RESET_PIN, "goodix reset pin");
		if (ret != 0) {
			gpio_free(TOUCH_RESET_PIN);
			printk("goodix gpio_request error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_RESET_PIN, 1);
                msleep(100);
		//gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);
		//msleep(100);
		//gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
		//msleep(500);
	}
	return 0;
}

struct goodix_platform_data goodix_info = {
	.model = 8105,
	.irq_pin = RK30_PIN4_PC2,
	.rest_pin = TOUCH_RESET_PIN,
	.init_platform_hw = goodix_init_platform_hw,
};
#endif

//#if defined(CONFIG_TOUCHSCREEN_GT8XX)
#if defined(WISKY_TS_GT811)
#define TOUCH_RESET_PIN  RK30_PIN4_PD0
#define TOUCH_PWR_PIN    RK30_PIN0_PD5
#define TOUCH_PWR_MUX_NAME		TS_POWER_MUX_NAME
#define TOUCH_PWR_MUX_MODE		TS_POWER_MUX_MODE

//#define TOUCH_RESET_PIN  			TS_RESET_PIN
#define TOUCH_RESET_MUX_NAME	TS_RESET_MUX_NAME
#define TOUCH_RESET_MUX_MODE	TS_RESET_MUX_MODE
#define TOUCH_RESET_VALUE		TS_RESET_VALUE
int gt811_init_platform_hw(void)
{
	int ret;

	printk("======%s starting=========\n",__FUNCTION__);
	rk30_mux_api_set(TOUCH_PWR_MUX_NAME, TOUCH_PWR_MUX_MODE	);
	rk30_mux_api_set(TOUCH_RESET_MUX_NAME, TOUCH_RESET_MUX_MODE);

	if (TOUCH_PWR_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_PWR_PIN, "goodix power pin");
		if (ret != 0) {
			gpio_free(TOUCH_PWR_PIN);
			printk("goodix power error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_PWR_PIN, 1);
		gpio_set_value(TOUCH_PWR_PIN, GPIO_HIGH);
		msleep(100);
		printk("======%s over=========\n",__FUNCTION__);
	}
#if 1
	if (TOUCH_RESET_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_RESET_PIN, "goodix reset pin");
		if (ret != 0) {
			gpio_free(TOUCH_RESET_PIN);
			printk("goodix gpio_request error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_RESET_PIN, 1);
                msleep(100);
	
		gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);
		msleep(100);
		gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
		msleep(500);
	}
#endif
	return 0;
}

struct gt811_platform_data gt811_info = {
	.model = 8105,
	.irq_pin = RK30_PIN4_PC2,
	.rest_pin = RK30_PIN4_PD0,
	.init_platform_hw = gt811_init_platform_hw,
};
#endif

#if defined(WISKY_TS_GT9XX)
#define TOUCH_RESET_PIN  INVALID_GPIO
#define TOUCH_PWR_PIN    TS_POWER_PIN
int goodix_9xx_init_platform_hw(void)
{
	int ret;
	
	rk30_mux_api_set(GPIO4D0_SMCDATA8_TRACEDATA8_NAME, GPIO4D_GPIO4D0);
	rk30_mux_api_set(GPIO4C2_SMCDATA2_TRACEDATA2_NAME, GPIO4C_GPIO4C2);
	rk30_mux_api_set(GPIO0D5_I2S22CHSDO_SMCADDR1_NAME, GPIO0D_GPIO0D5);
	printk("%s:0x%x,0x%x\n",__func__,rk30_mux_api_get(GPIO4D0_SMCDATA8_TRACEDATA8_NAME),rk30_mux_api_get(GPIO4C2_SMCDATA2_TRACEDATA2_NAME));

	if (TOUCH_PWR_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_PWR_PIN, "goodix power pin");
		if (ret != 0) {
			gpio_free(TOUCH_PWR_PIN);
			printk("goodix power error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_PWR_PIN, 1);
		gpio_set_value(TOUCH_PWR_PIN, GPIO_HIGH);
		msleep(100);
	}

	if (TOUCH_RESET_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_RESET_PIN, "goodix reset pin");
		if (ret != 0) {
			gpio_free(TOUCH_RESET_PIN);
			printk("goodix gpio_request error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_RESET_PIN, 1);
                msleep(100);
		//gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);
		//msleep(100);
		//gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
		//msleep(500);
	}
	return 0;
}

struct goodix_platform_data gt911_info = {
	.model = 8105,
	.irq_pin = TS_INT_PIN,
	.rest_pin = TS_RESET_PIN,
	.init_platform_hw = goodix_9xx_init_platform_hw,
};
#endif


static struct spi_board_info board_spi_devices[] = {
};

/***********************************************************
*	rk30  backlight
************************************************************/
#if defined(CONFIG_BACKLIGHT_RK29_BL) || defined(CONFIG_WISKY_BACKLIGHT)
#define PWM_ID            		LCD_BL_PWM_ID
#define PWM_MUX_NAME      LCD_BL_PWM_MUX_NAME
#define PWM_MUX_MODE      LCD_BL_PWM_MUX_MODE
#define PWM_MUX_MODE_GPIO LCD_BL_PWM_MUX_GPIO
#define PWM_GPIO 	  	LCD_BL_PWM_PIN
#define PWM_EFFECT_VALUE  LCD_BL_PWM_VALUE///1	//wisky[]cd huang@20120411^changed to '0' for M868

#define LCD_DISP_ON_PIN

#ifdef  LCD_DISP_ON_PIN
#define BL_EN_PIN			LCD_BL_EN_PIN
#define BL_EN_VALUE		LCD_BL_EN_VALUE
#define BL_EN_MUX_NAME    LCD_BL_EN_MUX_NAME
#define BL_EN_MUX_MODE    LCD_BL_EN_MUX_MODE

#endif
static int rk29_backlight_io_init(void)
{
	int ret = 0;
	pr_info("%s[%d]: ------>\n", __FUNCTION__, __LINE__);
	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE);
#ifdef  LCD_DISP_ON_PIN
	//pr_info("%s[%d]: ------>\n", __FUNCTION__, __LINE__);
	
	//wisky libai 20120920 修改开机屏会闪一下
  #ifdef WISKY_BOARD_M876R_TV30
  mdelay(1000);
  #endif
  //end
  
	rk30_mux_api_set(BL_EN_MUX_NAME, BL_EN_MUX_MODE);
	ret = gpio_request(BL_EN_PIN, NULL);
	if (ret != 0) {
		//gpio_free(BL_EN_PIN);
	}

	gpio_direction_output(BL_EN_PIN, BL_EN_VALUE);
//	gpio_set_value(BL_EN_PIN, BL_EN_VALUE);
#endif
	return ret;
}
void rk30_backlight_off(void)
{
	gpio_set_value(BL_EN_PIN, !BL_EN_VALUE);
	return ;
}

static int rk29_backlight_io_deinit(void)
{
	int ret = 0;
	
	#ifdef  LCD_DISP_ON_PIN
	pr_info("%s[%d]: ------>\n", __FUNCTION__, __LINE__);
	gpio_set_value(BL_EN_PIN, !BL_EN_VALUE);
	//gpio_free(BL_EN_PIN);
	#endif
	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE_GPIO);
	if (gpio_request(PWM_GPIO, NULL)) {
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
		//return -1;
	}
	gpio_direction_output(PWM_GPIO, !PWM_EFFECT_VALUE);
	mdelay(10);
	lcd_io_disable();
	mdelay(10);
	rk610_lcd_suspend();
	mdelay(250);
	
	return ret;
}

static int rk29_backlight_pwm_suspend(void)
{
	int ret = 0;
#if 1
	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE_GPIO);
	if (gpio_request(PWM_GPIO, NULL)) {
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
		//return -1;
	}
#endif
	gpio_direction_output(PWM_GPIO, !PWM_EFFECT_VALUE);

#ifdef  LCD_DISP_ON_PIN
	pr_info("%s[%d]: ------>\n", __FUNCTION__, __LINE__);
	//gpio_direction_output(BL_EN_PIN, !BL_EN_VALUE);
	gpio_set_value(BL_EN_PIN, !BL_EN_VALUE);
#endif

	return ret;
}

static int rk29_backlight_pwm_resume(void)
{
	//wisky[s]^延时开屏,避免唤醒功耗过大
	//mdelay(500);
	//wisky[e]

	mdelay(200);
	pr_info("%s[%d]: ------>\n", __FUNCTION__, __LINE__);
	gpio_free(PWM_GPIO);
	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE);
#ifdef  LCD_DISP_ON_PIN
	//gpio_direction_output(BL_EN_PIN, BL_EN_VALUE);
	gpio_set_value(BL_EN_PIN, BL_EN_VALUE);
#endif
	return 0;
}

static struct rk29_bl_info rk29_bl_info = {
	.pwm_id = PWM_ID,
	.bl_ref = PWM_EFFECT_VALUE,
	.io_init = rk29_backlight_io_init,
	.io_deinit = rk29_backlight_io_deinit,
	.pwm_suspend = rk29_backlight_pwm_suspend,
	.pwm_resume = rk29_backlight_pwm_resume,
	.min_brightness = LCD_BL_MIN_BRIGHTNESS,
};

static struct platform_device rk29_device_backlight = {
	.name	= "rk29_backlight",
	.id 	= -1,
	.dev	= {
		.platform_data  = &rk29_bl_info,
	}
};

#endif

#ifdef CONFIG_RK29_SUPPORT_MODEM

#define RK30_MODEM_POWER        RK30_PIN4_PD1
#define RK30_MODEM_POWER_IOMUX  rk29_mux_api_set(GPIO4D1_SMCDATA9_TRACEDATA9_NAME, GPIO4D_GPIO4D1)

static int rk30_modem_io_init(void)
{
    printk("%s\n", __FUNCTION__);
    RK30_MODEM_POWER_IOMUX;

	return 0;
}

static struct rk29_io_t rk30_modem_io = {
    .io_addr    = RK30_MODEM_POWER,
    .enable     = GPIO_HIGH,
    .disable    = GPIO_LOW,
    .io_init    = rk30_modem_io_init,
};

static struct platform_device rk30_device_modem = {
	.name	= "rk30_modem",
	.id 	= -1,
	.dev	= {
		.platform_data  = &rk30_modem_io,
	}
};
#endif
#if defined(CONFIG_MU509)
static int mu509_io_init(void)
{

	rk30_mux_api_set(GPIO2B6_LCDC1DATA14_SMCADDR18_TSSYNC_NAME, GPIO2B_GPIO2B6);
        rk30_mux_api_set(GPIO4D2_SMCDATA10_TRACEDATA10_NAME, GPIO4D_GPIO4D2);
	rk30_mux_api_set(GPIO2B7_LCDC1DATA15_SMCADDR19_HSADCDATA7_NAME, GPIO2B_GPIO2B7);
	rk30_mux_api_set(GPIO2C0_LCDCDATA16_GPSCLK_HSADCCLKOUT_NAME, GPIO2C_GPIO2C0);
	return 0;
}

static int mu509_io_deinit(void)
{
	
	return 0;
}
 
struct rk29_mu509_data rk29_mu509_info = {
	.io_init = mu509_io_init,
  	.io_deinit = mu509_io_deinit,
	.modem_power_en = RK30_PIN6_PB2,//RK30_PIN4_PD1,
	.bp_power = RK30_PIN2_PB6,//RK30_PIN4_PD1,
	.bp_reset = RK30_PIN4_PD2,
	.ap_wakeup_bp = RK30_PIN2_PB7,
	.bp_wakeup_ap = RK30_PIN6_PA0, 
};
struct platform_device rk29_device_mu509 = {	
        .name = "mu509",	
    	.id = -1,	
	.dev		= {
		.platform_data = &rk29_mu509_info,
	}    	
    };
#endif
#if defined(CONFIG_MW100)
static int mw100_io_init(void)
{
	 rk30_mux_api_set(GPIO2B6_LCDC1DATA14_SMCADDR18_TSSYNC_NAME, GPIO2B_GPIO2B6);
	 rk30_mux_api_set(GPIO4D2_SMCDATA10_TRACEDATA10_NAME, GPIO4D_GPIO4D2);
	 rk30_mux_api_set(GPIO2B7_LCDC1DATA15_SMCADDR19_HSADCDATA7_NAME, GPIO2B_GPIO2B7);
	 rk30_mux_api_set(GPIO2C0_LCDCDATA16_GPSCLK_HSADCCLKOUT_NAME, GPIO2C_GPIO2C0);
	return 0;
}

static int mw100_io_deinit(void)
{
	
	return 0;
}
 
struct rk29_mw100_data rk29_mw100_info = {
	.io_init = mw100_io_init,
  	.io_deinit = mw100_io_deinit,
	.modem_power_en = RK30_PIN6_PB2,
	.bp_power = RK30_PIN2_PB6,
	.bp_reset = RK30_PIN4_PD2,
	.ap_wakeup_bp = RK30_PIN2_PB7,
	.bp_wakeup_ap = RK30_PIN6_PA0,
};
struct platform_device rk29_device_mw100 = {	
        .name = "mw100",	
    	.id = -1,	
	.dev		= {
		.platform_data = &rk29_mw100_info,
	}    	
    };
#endif
#if defined(CONFIG_MT6229)
static int mt6229_io_init(void)
{
	 rk30_mux_api_set(GPIO2B6_LCDC1DATA14_SMCADDR18_TSSYNC_NAME, GPIO2B_GPIO2B6);
	 rk30_mux_api_set(GPIO4D2_SMCDATA10_TRACEDATA10_NAME, GPIO4D_GPIO4D2);
	 rk30_mux_api_set(GPIO2B7_LCDC1DATA15_SMCADDR19_HSADCDATA7_NAME, GPIO2B_GPIO2B7);
	 rk30_mux_api_set(GPIO2C0_LCDCDATA16_GPSCLK_HSADCCLKOUT_NAME, GPIO2C_GPIO2C0);
	return 0;
}

static int mt6229_io_deinit(void)
{
	
	return 0;
}
 
struct rk29_mt6229_data rk29_mt6229_info = {
	.io_init = mt6229_io_init,
  	.io_deinit = mt6229_io_deinit,
	.modem_power_en = RK30_PIN6_PB2,
	.bp_power = RK30_PIN2_PB7,
	.bp_reset = RK30_PIN4_PD2,
	.ap_wakeup_bp = RK30_PIN2_PC0,
	.bp_wakeup_ap = RK30_PIN6_PA0,
};
struct platform_device rk29_device_mt6229 = {	
        .name = "mt6229",	
    	.id = -1,	
	.dev		= {
		.platform_data = &rk29_mt6229_info,
	}    	
    };
#endif
#if defined(CONFIG_SEW868)
static int sew868_io_init(void)
{
	rk30_mux_api_set(GPIO2B6_LCDC1DATA14_SMCADDR18_TSSYNC_NAME, GPIO2B_GPIO2B6);
    rk30_mux_api_set(GPIO4D2_SMCDATA10_TRACEDATA10_NAME, GPIO4D_GPIO4D2);
	rk30_mux_api_set(GPIO4D4_SMCDATA12_TRACEDATA12_NAME, GPIO4D_GPIO4D4);
	return 0;
}
static int sew868_io_deinit(void)
{
	return 0;
}
struct rk30_sew868_data rk30_sew868_info = {
	.io_init = sew868_io_init,
  	.io_deinit = sew868_io_deinit,
	.bp_power = RK30_PIN6_PB2, 
	.bp_power_active_low = 1,
	.bp_sys = RK30_PIN2_PB6, 
	.bp_reset = RK30_PIN4_PD2, 
	.bp_reset_active_low = 1,
	.bp_wakeup_ap = RK30_PIN4_PD4, 
	.ap_wakeup_bp = NULL,
};

struct platform_device rk30_device_sew868 = {	
        .name = "sew868",	
    	.id = -1,	
	.dev		= {
		.platform_data = &rk30_sew868_info,
	}    	
    };
#endif

/*MMA8452 gsensor*/
#if defined (CONFIG_GS_MMA8452)
#define MMA8452_INT_PIN   RK30_PIN4_PC0

static int mma8452_init_platform_hw(void)
{
	rk30_mux_api_set(GPIO4C0_SMCDATA0_TRACEDATA0_NAME, GPIO4C_GPIO4C0);

	return 0;
}

static struct sensor_platform_data mma8452_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = mma8452_init_platform_hw,
        .orientation = {-1, 0, 0, 0, 0, 1, 0, -1, 0},
};
#endif
#if defined (CONFIG_GS_LIS3DH)
#define LIS3DH_INT_PIN   RK30_PIN4_PC0

static int lis3dh_init_platform_hw(void)
{
        rk30_mux_api_set(GPIO4C0_SMCDATA0_TRACEDATA0_NAME, GPIO4C_GPIO4C0);

        return 0;
}

static struct sensor_platform_data lis3dh_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = lis3dh_init_platform_hw,
	.orientation = {-1, 0, 0, 0, 0, 1, 0, -1, 0},
};
#endif


/*MMA8452 gsensor*/
#if defined (CONFIG_GS_MMA8452)
#define MMA8452_INT_PIN   RK30_PIN4_PC0

static int mma8452_init_platform_hw(void)
{
	rk30_mux_api_set(GPIO4C0_SMCDATA0_TRACEDATA0_NAME, GPIO4C_GPIO4C0);

	if (gpio_request(MMA8452_INT_PIN, NULL) != 0) {
		gpio_free(MMA8452_INT_PIN);
		printk("mma8452_init_platform_hw gpio_request error\n");
		return -EIO;
	}
	gpio_pull_updown(MMA8452_INT_PIN, 1);
	return 0;
}

static struct gsensor_platform_data mma8452_info = {
	.model = 8452,
	.swap_xy = 0,
	.swap_xyz = 1,
	.init_platform_hw = mma8452_init_platform_hw,
	.orientation = {-1, 0, 0, 0, 0, 1, 0, -1, 0},
};
#endif
/*MMA7660 gsensor*/
#if defined(WISKY_GSENSOR_MMA7660)
#define MMA7660_INT_PIN   GSENSOR_INT_PIN

int mma7660_init_platform_hw(void)
{
	rk30_mux_api_set(GSENSOR_INT_MUX_NAME, GSENSOR_INT_MUX_MODE);
	if(gpio_request(MMA7660_INT_PIN,NULL) != 0){
		gpio_free(MMA7660_INT_PIN);
		printk("mma7660_init_platform_hw gpio_request error\n");
		return -EIO;
	}
	gpio_pull_updown(MMA7660_INT_PIN, 1);
	
	return 0;
}

struct mma7660_platform_data mma7660_info = {
  .model= 7660,
  .swap_xy = 0,
  .init_platform_hw= mma7660_init_platform_hw,
};
#endif
//wisky[s]lxh@20120719
/*MMA7660 gsensor*/
#if defined(WISKY_GSENSOR_DMARD06)
#define DMARD06_INT_PIN   GSENSOR_INT_PIN

int dmard06_init_platform_hw(void)
{
	rk30_mux_api_set(GSENSOR_INT_MUX_NAME, GSENSOR_INT_MUX_MODE);
	if(gpio_request(DMARD06_INT_PIN,NULL) != 0){
		gpio_free(DMARD06_INT_PIN);
		printk("dmard06_init_platform_hw gpio_request error\n");
		return -EIO;
	}
	gpio_pull_updown(DMARD06_INT_PIN, 1);
	
	return 0;
}

struct dmard06_platform_data dmard06_info = {
  .model= 6,
  .swap_xy = 0,
  .init_platform_hw= dmard06_init_platform_hw,
};
#endif
//wisky[e]

#if defined (CONFIG_COMPASS_AK8975)
static struct sensor_platform_data akm8975_info =
{
	.type = SENSOR_TYPE_COMPASS,
	.irq_enable = 1,
	.poll_delay_ms = 30,
	.m_layout = 
	{
		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},
	}
};

#endif

#if defined(CONFIG_GYRO_L3G4200D)

#include <linux/l3g4200d.h>
#define L3G4200D_INT_PIN  RK30_PIN4_PC3

static int l3g4200d_init_platform_hw(void)
{
	rk30_mux_api_set(GPIO4C3_SMCDATA3_TRACEDATA3_NAME, GPIO4C_GPIO4C3);
	
	return 0;
}

static struct sensor_platform_data l3g4200d_info = {
	.type = SENSOR_TYPE_GYROSCOPE,
	.irq_enable = 1,
	.poll_delay_ms = 30,
	.orientation = {0, 1, 0, -1, 0, 0, 0, 0, 1},
	.init_platform_hw = l3g4200d_init_platform_hw,
	.x_min = 40,//x_min,y_min,z_min = (0-100) according to hardware
	.y_min = 40,
	.z_min = 20,
};

#endif

#ifdef CONFIG_LS_CM3217
static struct sensor_platform_data cm3217_info = {
	.type = SENSOR_TYPE_LIGHT,
	.irq_enable = 0,
	.poll_delay_ms = 500,
};

#endif

#ifdef CONFIG_FB_ROCKCHIP
#if 0
#define LCD_EN_MUX_NAME    GPIO4C7_SMCDATA7_TRACEDATA7_NAME
#define LCD_EN_PIN         RK30_PIN4_PC7
#define LCD_EN_VALUE       GPIO_HIGH
#else//for wisky mid project
//wisky[]lcd lvds /PDN control
#define LCD_CS_PIN         		LCD_LVDS_PDN_PIN
#define LCD_CS_MUX_NAME		LCD_LVDS_PDN_MUX_NAME
#define LCD_CS_MUX_MODE		LCD_LVDS_PDN_MUX_MODE
#define LCD_CS_VALUE       		LCD_LVDS_PDN_VALUE

#define RK_LCD_EN_PIN         		LCD_PWR_EN_PIN
#define RK_LCD_EN_MUX_NAME		LCD_PWR_EN_MUX_NAME
#define RK_LCD_EN_MUX_MODE		LCD_PWR_EN_MUX_MODE
#define RK_LCD_EN_VALUE			LCD_PWR_EN_VALUE

#define RK_LCD_RESET_PIN				LCD_RESET_PIN
#define RK_LCD_RESET_MUX_NAME		LCD_RESET_MUX_NAME
#define RK_LCD_RESET_MUX_MODE		LCD_RESET_MUX_MODE
#define RK_LCD_RESET_VALUE			LCD_RESET_VALUE

#define RK_LCD_STANDBY_PIN			LCD_STANDBY_PIN
#define RK_LCD_STANDBY_MUX_NAME	LCD_STANDBY_MUX_NAME
#define RK_LCD_STANDBY_MUX_MODE	LCD_STANDBY_MUX_MODE
#define RK_LCD_STANDBY_VALUE		GPIO_HIGH
//wisky[e]
#endif
static int rk_fb_io_init(struct rk29_fb_setting_info *fb_setting)
{
#if 0
	int ret = 0;
	rk30_mux_api_set(LCD_EN_MUX_NAME, GPIO4C_GPIO4C7);
	ret = gpio_request(LCD_EN_PIN, NULL);
	if (ret != 0)
	{
		gpio_free(LCD_EN_PIN);
		printk(KERN_ERR "request lcd en pin fail!\n");
		return -1;
	}
	else
	{
		gpio_direction_output(LCD_EN_PIN, 1);
		gpio_set_value(LCD_EN_PIN, LCD_EN_VALUE);
	}
#else//for wisky mid project
	//init lcd control gpio
	rk29_backlight_io_init();
	pr_info("%s[%d]: ------>\n", __FUNCTION__, __LINE__);
	lcd_io_init();
#if 0
	msleep(200);
	lcd_io_disable();
	msleep(100);
	rk610_lcd_suspend();
	msleep(100);
	rk610_lcd_resume();
	msleep(100);
	lcd_io_enable();
#endif

#endif

	return 0;
}
static int rk_fb_io_disable(void)
{
#if 0
	gpio_set_value(LCD_EN_PIN, LCD_EN_VALUE? 0:1);
#else//for wisky mid project
	lcd_io_disable();
#endif

	return 0;
}
static int rk_fb_io_enable(void)
{
#if 0
	gpio_set_value(LCD_EN_PIN, LCD_EN_VALUE);
#else//for wisky mid project
	lcd_io_enable();
#endif

	return 0;
}

#if defined(CONFIG_LCDC0_RK30)
struct rk29fb_info lcdc0_screen_info = {
	.prop	   = PRMRY,		//primary display device
	.io_init   = rk_fb_io_init,
	.io_disable = rk_fb_io_disable,
	.io_enable = rk_fb_io_enable,
	.set_screen_info = set_lcd_info,
};
#endif

#if defined(CONFIG_LCDC1_RK30)
struct rk29fb_info lcdc1_screen_info = {
	#if defined(CONFIG_HDMI_RK30)
	.prop		= EXTEND,	//extend display device
	.lcd_info  = NULL,
	.set_screen_info = hdmi_init_lcdc,
	#endif
};
#endif

static struct resource resource_fb[] = {
	[0] = {
		.name  = "fb0 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "ipp buf",  //for rotate
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.name  = "fb2 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device device_fb = {
	.name		= "rk-fb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resource_fb),
	.resource	= resource_fb,
};
#endif

#ifdef CONFIG_ANDROID_TIMED_GPIO
static struct timed_gpio timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = RK30_PIN0_PA4,
		.max_timeout = 1000,
		.active_low = 0,
		.adjust_time =20,      //adjust for diff product
	},
};

static struct timed_gpio_platform_data rk29_vibrator_info = {
	.num_gpios = 1,
	.gpios = timed_gpios,
};

static struct platform_device rk29_device_vibrator = {
	.name = "timed-gpio",
	.id = -1,
	.dev = {
		.platform_data = &rk29_vibrator_info,
	},

};
#endif

#ifdef CONFIG_LEDS_GPIO_PLATFORM
static struct gpio_led rk29_leds[] = {
	{
		.name = "button-backlight",
		.gpio = RK30_PIN4_PD7,
		.default_trigger = "timer",
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
};

static struct gpio_led_platform_data rk29_leds_pdata = {
	.leds = rk29_leds,
	.num_leds = ARRAY_SIZE(rk29_leds),
};

static struct platform_device rk29_device_gpio_leds = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data  = &rk29_leds_pdata,
	},
};
#endif

#ifdef CONFIG_RK_IRDA
#define IRDA_IRQ_PIN           RK30_PIN6_PA1

static int irda_iomux_init(void)
{
	int ret = 0;

	//irda irq pin
	ret = gpio_request(IRDA_IRQ_PIN, NULL);
	if (ret != 0) {
		gpio_free(IRDA_IRQ_PIN);
		printk(">>>>>> IRDA_IRQ_PIN gpio_request err \n ");
	}
	gpio_pull_updown(IRDA_IRQ_PIN, PullDisable);
	gpio_direction_input(IRDA_IRQ_PIN);

	return 0;
}

static int irda_iomux_deinit(void)
{
	gpio_free(IRDA_IRQ_PIN);
	return 0;
}

static struct irda_info rk29_irda_info = {
	.intr_pin = IRDA_IRQ_PIN,
	.iomux_init = irda_iomux_init,
	.iomux_deinit = irda_iomux_deinit,
	//.irda_pwr_ctl = bu92747guw_power_ctl,
};

static struct platform_device irda_device = {
#ifdef CONFIG_RK_IRDA_NET
	.name = "rk_irda",
#else
	.name = "bu92747_irda",
#endif
	.id = -1,
	.dev = {
		.platform_data = &rk29_irda_info,
	}
};
#endif

#ifdef CONFIG_ION
#define ION_RESERVE_SIZE        (80 * SZ_1M)
static struct ion_platform_data rk30_ion_pdata = {
	.nr = 1,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = ION_NOR_HEAP_ID,
			.name = "norheap",
			.size = ION_RESERVE_SIZE,
		}
	},
};

static struct platform_device device_ion = {
	.name = "ion-rockchip",
	.id = 0,
	.dev = {
		.platform_data = &rk30_ion_pdata,
	},
};
#endif

/**************************************************************************************************
 * SDMMC devices,  include the module of SD,MMC,and sdio.noted by xbw at 2012-03-05
**************************************************************************************************/
#ifdef CONFIG_SDMMC_RK29
#include "board-rk30-sdk-sdmmc.c"

#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
#define SDMMC0_WRITE_PROTECT_PIN	RK30_PIN3_PB7	//According to your own project to set the value of write-protect-pin.
#endif

#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
#define SDMMC1_WRITE_PROTECT_PIN	RK30_PIN3_PC7	//According to your own project to set the value of write-protect-pin.
#endif

#define RK29SDK_WIFI_SDIO_CARD_DETECT_N    RK30_PIN6_PB2

#endif //endif ---#ifdef CONFIG_SDMMC_RK29

#ifdef CONFIG_SDMMC0_RK29
static int rk29_sdmmc0_cfg_gpio(void)
{
#ifdef CONFIG_SDMMC_RK29_OLD
	rk30_mux_api_set(GPIO3B1_SDMMC0CMD_NAME, GPIO3B_SDMMC0_CMD);
	rk30_mux_api_set(GPIO3B0_SDMMC0CLKOUT_NAME, GPIO3B_SDMMC0_CLKOUT);
	rk30_mux_api_set(GPIO3B2_SDMMC0DATA0_NAME, GPIO3B_SDMMC0_DATA0);
	rk30_mux_api_set(GPIO3B3_SDMMC0DATA1_NAME, GPIO3B_SDMMC0_DATA1);
	rk30_mux_api_set(GPIO3B4_SDMMC0DATA2_NAME, GPIO3B_SDMMC0_DATA2);
	rk30_mux_api_set(GPIO3B5_SDMMC0DATA3_NAME, GPIO3B_SDMMC0_DATA3);

	rk30_mux_api_set(GPIO3B6_SDMMC0DETECTN_NAME, GPIO3B_GPIO3B6);

	rk30_mux_api_set(GPIO3A7_SDMMC0PWREN_NAME, GPIO3A_GPIO3A7);
	gpio_request(RK30_PIN3_PA7, "sdmmc-power");
	gpio_direction_output(RK30_PIN3_PA7, GPIO_LOW);

#else
	rk29_sdmmc_set_iomux(0, 0xFFFF);

	rk30_mux_api_set(GPIO3B6_SDMMC0DETECTN_NAME, GPIO3B_SDMMC0_DETECT_N);

#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	gpio_request(SDMMC0_WRITE_PROTECT_PIN, "sdmmc-wp");
	gpio_direction_input(SDMMC0_WRITE_PROTECT_PIN);
#endif

#endif
	
	return 0;
}

#define CONFIG_SDMMC0_USE_DMA
struct rk29_sdmmc_platform_data default_sdmmc0_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34 | MMC_VDD_34_35 | MMC_VDD_35_36),
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
	.io_init = rk29_sdmmc0_cfg_gpio,

#if !defined(CONFIG_SDMMC_RK29_OLD)
	.set_iomux = rk29_sdmmc_set_iomux,
#endif

	.dma_name = "sd_mmc",
#ifdef CONFIG_SDMMC0_USE_DMA
	.use_dma = 1,
#else
	.use_dma = 0,
#endif
	.detect_irq = RK30_PIN3_PB6,	// INVALID_GPIO
	.enable_sd_wakeup = 0,

#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	.write_prt = SDMMC0_WRITE_PROTECT_PIN,
#else
	.write_prt = INVALID_GPIO,
#endif
};
#endif // CONFIG_SDMMC0_RK29

#ifdef CONFIG_SDMMC1_RK29
#define CONFIG_SDMMC1_USE_DMA
static int rk29_sdmmc1_cfg_gpio(void)
{
#if defined(CONFIG_SDMMC_RK29_OLD)
	rk30_mux_api_set(GPIO3C0_SMMC1CMD_NAME, GPIO3C_SMMC1_CMD);
	rk30_mux_api_set(GPIO3C5_SDMMC1CLKOUT_NAME, GPIO3C_SDMMC1_CLKOUT);
	rk30_mux_api_set(GPIO3C1_SDMMC1DATA0_NAME, GPIO3C_SDMMC1_DATA0);
	rk30_mux_api_set(GPIO3C2_SDMMC1DATA1_NAME, GPIO3C_SDMMC1_DATA1);
	rk30_mux_api_set(GPIO3C3_SDMMC1DATA2_NAME, GPIO3C_SDMMC1_DATA2);
	rk30_mux_api_set(GPIO3C4_SDMMC1DATA3_NAME, GPIO3C_SDMMC1_DATA3);
	//rk30_mux_api_set(GPIO3C6_SDMMC1DETECTN_NAME, GPIO3C_SDMMC1_DETECT_N);

#else

#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	gpio_request(SDMMC1_WRITE_PROTECT_PIN, "sdio-wp");
	gpio_direction_input(SDMMC1_WRITE_PROTECT_PIN);
#endif

#endif

	return 0;
}

struct rk29_sdmmc_platform_data default_sdmmc1_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34),

#if !defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
	.host_caps = (MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ |
		      MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#else
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#endif

	.io_init = rk29_sdmmc1_cfg_gpio,

#if !defined(CONFIG_SDMMC_RK29_OLD)
	.set_iomux = rk29_sdmmc_set_iomux,
#endif

	.dma_name = "sdio",
#ifdef CONFIG_SDMMC1_USE_DMA
	.use_dma = 1,
#else
	.use_dma = 0,
#endif

#if !defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
#ifdef CONFIG_WIFI_CONTROL_FUNC
	.status = rk29sdk_wifi_status,
	.register_status_notify = rk29sdk_wifi_status_register,
#endif
#if 0
	.detect_irq = RK29SDK_WIFI_SDIO_CARD_DETECT_N,
#endif

#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	.write_prt = SDMMC1_WRITE_PROTECT_PIN,
#else
	.write_prt = INVALID_GPIO,
#endif

#else
	.detect_irq = INVALID_GPIO,
	.enable_sd_wakeup = 0,
#endif

};
#endif //endif--#ifdef CONFIG_SDMMC1_RK29

/**************************************************************************************************
 * the end of setting for SDMMC devices
**************************************************************************************************/

#ifdef CONFIG_BATTERY_RK30_ADC
static struct rk30_adc_battery_platform_data rk30_adc_battery_platdata = {
        .dc_det_pin      = RK30_PIN6_PA5,
        .batt_low_pin    = RK30_PIN6_PA0,
        .charge_set_pin  = INVALID_GPIO,
        .charge_ok_pin   = RK30_PIN6_PA6,
        .dc_det_level    = GPIO_LOW,
        .charge_ok_level = GPIO_HIGH,
};

static struct platform_device rk30_device_adc_battery = {
        .name   = "rk30-battery",
        .id     = -1,
        .dev = {
                .platform_data = &rk30_adc_battery_platdata,
        },
};
#endif

#ifdef CONFIG_RK29_VMAC
#define PHY_PWR_EN_GPIO	ETH_PHY_POWER_PIN//RK30_PIN1_PD6
#include "board-rk30-sdk-vmac.c"
#endif

#ifdef CONFIG_RFKILL_RK
// bluetooth rfkill device, its driver in net/rfkill/rfkill-rk.c
static struct rfkill_rk_platform_data rfkill_rk_platdata = {
    .type               = RFKILL_TYPE_BLUETOOTH,

    .poweron_gpio       = { // BT_REG_ON
        .io             = RK30_PIN3_PC7,
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = GPIO3C7_SDMMC1WRITEPRT_NAME,
            .fgpio      = GPIO3C_GPIO3C7,
        },
    },

    .reset_gpio         = { // BT_RST
        .io             = RK30_PIN1_PA2,//RK30_PIN3_PD1, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = GPIO1A2_UART0CTSN_NAME,//GPIO3D1_SDMMC1BACKENDPWR_NAME,
            .fgpio      = GPIO1A_GPIO1A2,//GPIO3D_GPIO3D1,
        },
    },

    .wake_gpio          = { // BT_WAKE, use to control bt's sleep and wakeup
        .io             = RK30_PIN3_PC6, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = GPIO3C6_SDMMC1DETECTN_NAME,
            .fgpio      = GPIO3C_GPIO3C6,
        },
    },

    .wake_host_irq      = { // BT_HOST_WAKE, for bt wakeup host when it is in deep sleep
        .gpio           = {
            .io         = RK30_PIN6_PA7, // set io to INVALID_GPIO for disable it
            .enable     = GPIO_LOW,      // set GPIO_LOW for falling, set 0 for rising
            .iomux      = {
                .name   = NULL,
            },
        },
    },

    .rts_gpio           = { // UART_RTS, enable or disable BT's data coming
	  
        .io             = RK30_PIN3_PD1,//RK30_PIN1_PA2, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       =GPIO3D1_SDMMC1BACKENDPWR_NAME,// GPIO1A3_UART0RTSN_NAME,
            .fgpio      = GPIO3D_GPIO3D1,//GPIO1A_GPIO1A3,
            .fmux       = GPIO1A_UART0_RTS_N,
        },
    },
};

static struct platform_device device_rfkill_rk = {
    .name   = "rfkill_rk",
    .id     = -1,
    .dev    = {
        .platform_data = &rfkill_rk_platdata,
    },
};
#endif


#if CONFIG_RK30_PWM_REGULATOR
const static int pwm_voltage_map[] = {
	1000000, 1025000, 1050000, 1075000, 1100000, 1125000, 1150000, 1175000, 1200000, 1225000, 1250000, 1275000, 1300000, 1325000, 1350000, 1375000, 1400000
};

static struct regulator_consumer_supply pwm_dcdc1_consumers[] = {
	{
		.supply = "vdd_core",
	}
};

struct regulator_init_data pwm_regulator_init_dcdc[1] =
{
	{
		.constraints = {
			.name = "PWM_DCDC1",
			.min_uV = 600000,
			.max_uV = 1800000,	//0.6-1.8V
			.apply_uV = true,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(pwm_dcdc1_consumers),
		.consumer_supplies = pwm_dcdc1_consumers,
	},
};
static struct pwm_platform_data pwm_regulator_info[1] = {
	{
		.pwm_id = 3,
		.pwm_gpio = RK30_PIN0_PD7,
		.pwm_iomux_name = GPIO0D7_PWM3_NAME,
		.pwm_iomux_pwm = GPIO0D_PWM3,
		.pwm_iomux_gpio = GPIO0D_GPIO0D6,
		.pwm_voltage = 1100000,
		.suspend_voltage = 1050000,
		.min_uV = 1000000,
		.max_uV	= 1400000,
		.coefficient = 455,	//45.5%
		.pwm_voltage_map = pwm_voltage_map,
		.init_data	= &pwm_regulator_init_dcdc[0],
	},
};

struct platform_device pwm_regulator_device[1] = {
	{
		.name = "pwm-voltage-regulator",
		.id = 0,
		.dev		= {
			.platform_data = &pwm_regulator_info[0],
		}
	},
};
#endif

static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_BACKLIGHT_RK29_BL) || defined(CONFIG_WISKY_BACKLIGHT)
	&rk29_device_backlight,
#endif
#ifdef CONFIG_FB_ROCKCHIP
	&device_fb,
#endif
#ifdef CONFIG_ION
	&device_ion,
#endif
#ifdef CONFIG_ANDROID_TIMED_GPIO
	&rk29_device_vibrator,
#endif
#ifdef CONFIG_LEDS_GPIO_PLATFORM
	&rk29_device_gpio_leds,
#endif
#ifdef CONFIG_RK_IRDA
	&irda_device,
#endif
#ifdef CONFIG_WIFI_CONTROL_FUNC
	&rk29sdk_wifi_device,
#endif
#ifdef CONFIG_RK29_SUPPORT_MODEM
	&rk30_device_modem,
#endif
#if defined(CONFIG_MU509)
	&rk29_device_mu509,
#endif
#if defined(CONFIG_MW100)
	&rk29_device_mw100,
#endif
#if defined(CONFIG_MT6229)
	&rk29_device_mt6229,
#endif
#if defined(CONFIG_SEW868)
	&rk30_device_sew868,
#endif
#ifdef CONFIG_BATTERY_RK30_ADC
 	&rk30_device_adc_battery,
#endif
#ifdef CONFIG_RFKILL_RK
	&device_rfkill_rk,
#endif
#if defined(CONFIG_WISKY_BATTERY)
	&wisky_device_battery,
#endif
#if defined(WISKY_SWITCH_GPIO)
	&wisky_switch_gpio_device,
#endif
#if defined(WISKY_VIBRATOR_TIMED_GPIO)
	&wisky_vibrator_device,
#endif
#ifdef CONFIG_RK30_PWM_REGULATOR
	&pwm_regulator_device[0],
#endif
};

// i2c
#ifdef CONFIG_I2C0_RK30
static struct i2c_board_info __initdata i2c0_info[] = {
#if defined (CONFIG_GS_MMA8452)
	{
		.type	        = "gs_mma8452",
		.addr	        = 0x1d,
		.flags	        = 0,
		.irq	        = MMA8452_INT_PIN,
		.platform_data = &mma8452_info,
	},
#endif
#if defined (CONFIG_GS_LIS3DH)
	{
		.type	        = "gs_lis3dh",
		.addr	        = 0x19,   //0x19(SA0-->VCC), 0x18(SA0-->GND)
		.flags	        = 0,
		.irq	        = LIS3DH_INT_PIN,
		.platform_data = &lis3dh_info,
	},
#endif
#if defined (CONFIG_COMPASS_AK8975)
	{
		.type          = "ak8975",
		.addr          = 0x0d,
		.flags         = 0,
		.irq           = RK30_PIN4_PC1,
		.platform_data = &akm8975_info,
	},
#endif
#if defined (CONFIG_GYRO_L3G4200D)
	{
		.type          = "l3g4200d_gryo",
		.addr          = 0x69,
		.flags         = 0,
		.irq           = L3G4200D_INT_PIN,
		.platform_data = &l3g4200d_info,
	},
#endif
#if defined (CONFIG_SND_SOC_RK1000)
	{
		.type          = "rk1000_i2c_codec",
		.addr          = 0x60,
		.flags         = 0,
	},
	{
		.type          = "rk1000_control",
		.addr          = 0x40,
		.flags         = 0,
	},
#endif
#if defined (WISKY_CODEC_RT5631)
        {
                .type                   = "rt5631",
                .addr                   = 0x1a,
                .flags                  = 0,
        },
#endif
#if defined (WISKY_CODEC_ES8323)
        {
                .type                   = "es8323",
                .addr                   = 0x10,
                .flags                  = 0,
        },
#endif

#if defined (CONFIG_RTC_HYM8563) || defined(WISKY_RTC_HYM8563)
	{
		.type    		= "rtc_hym8563",
		.addr           = 0x51,
		.flags			= 0,
		.irq            = RTC_INT_PIN,
	},
#endif

#ifdef CONFIG_MFD_RK610
		{
			.type			= "rk610_ctl",
			.addr			= 0x40,
			.flags			= 0,
		},
#endif
#ifdef CONFIG_RK610_TVOUT
		{
			.type			= "rk610_tvout",
			.addr			= 0x42,
			.flags			= 0,
		},
#endif
#ifdef CONFIG_HDMI_RK610
		{
			.type			= "rk610_hdmi",
			.addr			= 0x46,
			.flags			= 0,
			//.irq			= RK29_PIN5_PA2,
		},
#endif
#ifdef CONFIG_SND_SOC_RK610
		{//RK610_CODEC addr  from 0x60 to 0x80 (0x60~0x80)
			.type			= "rk610_i2c_codec",
			.addr			= 0x60,
			.flags			= 0,
		},
#endif
#if defined(WISKY_GSENSOR_MMA7660)
	{
		.type	        = "gs_mma7660",
		.addr	        = 0x4c,
		.flags	        = 0,
		.irq	        = MMA7660_INT_PIN,
		.platform_data = &mma7660_info,
	},
#endif
//wisky[s]lxh@20120719
#if defined(WISKY_GSENSOR_DMARD06)
	{
		.type	        = "gs_dmard06",
		.addr	        = 0x1c,
		.flags	        = 0,
		.irq	        = DMARD06_INT_PIN,
		.platform_data = &dmard06_info,
	},
#endif
//wisky[e]
};
#endif
#define PMIC_TYPE_WM8326	1
#define PMIC_TYPE_TPS65910	2
int __sramdata g_pmic_type =  0;
#ifdef CONFIG_I2C1_RK30
#if defined (CONFIG_KP_AXP)
#include "board-rk30-sdk-axp.c"
#else
#endif
#ifdef CONFIG_MFD_WM831X_I2C
#include "board-rk30-sdk-wm8326.c"
#endif
#ifdef CONFIG_MFD_TPS65910
#define TPS65910_HOST_IRQ        RK30_PIN6_PA4
#include "board-rk30-sdk-tps65910.c"
#endif

static struct i2c_board_info __initdata i2c1_info[] = {
#if defined (CONFIG_MFD_WM831X_I2C)
	{
		.type          = "wm8326",
		.addr          = 0x34,
		.flags         = 0,
		.irq           = RK30_PIN6_PA4,
		.platform_data = &wm831x_platdata,
	},
#endif
#if defined (CONFIG_KP_AXP)
    {
        .type       	= "axp_mfd",
        .addr       	= AXP_DEVICES_ADDR,
        .irq        	= RK30_PIN6_PA4,
        .flags      	= 0,
        .platform_data	= &axp_pdata,
    },
#endif
#if defined(WISKY_BATTERY_SMB347)
	{
		.type = "smb347",
		.addr= 0x06,
		.flags = 0,
//		.platform_data = &
	},
#endif
#if defined(WISKY_BATTERY_OZ8806)
	{
		.type = "OZ8806",
		.addr = 0x2F,
		.flags = 0,
	},
#endif
#if defined (CONFIG_MFD_TPS65910)
	{
        .type           = "tps65910",
        .addr           = TPS65910_I2C_ID0,
        .flags          = 0,
        .irq            = TPS65910_HOST_IRQ,
    	.platform_data = &tps65910_data,
	},
#endif
};
#endif
//wisky-lxh@20120912,board_pmu_suspend has defined in axp-mfd.c
#if defined(CONFIG_MFD_WM831X_I2C) || defined(CONFIG_MFD_TPS65910)
void __sramfunc board_pmu_suspend(void)
{      
	#if defined (CONFIG_MFD_WM831X_I2C)
       if(g_pmic_type == PMIC_TYPE_WM8326)
       board_pmu_wm8326_suspend();
	#endif
	#if defined (CONFIG_MFD_TPS65910)
       if(g_pmic_type == PMIC_TYPE_TPS65910)
       board_pmu_tps65910_suspend(); 
    #endif   
}

void __sramfunc board_pmu_resume(void)
{      
	#if defined (CONFIG_MFD_WM831X_I2C)
       if(g_pmic_type == PMIC_TYPE_WM8326)
       board_pmu_wm8326_resume();
	#endif
	#if defined (CONFIG_MFD_TPS65910)
       if(g_pmic_type == PMIC_TYPE_TPS65910)
       board_pmu_tps65910_resume(); 
	#endif
}
#endif
//end-wisky-lxh@20120912
int __sramdata gpio0d7_iomux,gpio0d7_do,gpio0d7_dir,gpio0d7_en;

void __sramfunc rk30_pwm_logic_suspend_voltage(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR

//	int gpio0d7_iomux,gpio0d7_do,gpio0d7_dir,gpio0d7_en;
	sram_udelay(10000);
	gpio0d7_iomux = readl_relaxed(GRF_GPIO0D_IOMUX);
	gpio0d7_do = grf_readl(GRF_GPIO0H_DO);
	gpio0d7_dir = grf_readl(GRF_GPIO0H_DIR);
	gpio0d7_en = grf_readl(GRF_GPIO0H_EN);

	writel_relaxed((1<<30), GRF_GPIO0D_IOMUX);
	grf_writel((1<<31)|(1<<15), GRF_GPIO0H_DIR);
	grf_writel((1<<31)|(1<<15), GRF_GPIO0H_DO);
	grf_writel((1<<31)|(1<<15), GRF_GPIO0H_EN);
#endif 
}
void __sramfunc rk30_pwm_logic_resume_voltage(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	writel_relaxed((1<<30)|gpio0d7_iomux, GRF_GPIO0D_IOMUX);
	grf_writel((1<<31)|gpio0d7_en, GRF_GPIO0H_EN);
	grf_writel((1<<31)|gpio0d7_dir, GRF_GPIO0H_DIR);
	grf_writel((1<<31)|gpio0d7_do, GRF_GPIO0H_DO);
	sram_udelay(10000);

#endif

}
extern void pwm_suspend_voltage(void);
extern void pwm_resume_voltage(void);
void  rk30_pwm_suspend_voltage_set(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	pwm_suspend_voltage();
#endif
}
void  rk30_pwm_resume_voltage_set(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	pwm_resume_voltage();
#endif
}


#ifdef CONFIG_I2C2_RK30
static struct i2c_board_info __initdata i2c2_info[] = {
#if defined (CONFIG_TOUCHSCREEN_GT8XX)
	{
		.type          = "Goodix-TS",
		.addr          = 0x55,
		.flags         = 0,
		.irq           = RK30_PIN4_PC2,
		.platform_data = &goodix_info,
	},
#endif
#if defined (WISKY_TS_GT811)
	{
		.type          = "Gt811-TS",
		.addr          = 0x5d,
		.flags         = 0,
		.irq           = RK30_PIN4_PC2,
		.platform_data = &gt811_info,
	},
#endif
#if defined (WISKY_TS_GT9XX)
	{
		.type          = "Gt911-TS",
		.addr          = 0x14,
		.flags         = 0,
		.irq           = TS_INT_PIN,
		.platform_data = &gt911_info,
	},
#endif
#if defined (CONFIG_LS_CM3217)
	{
		.type          = "lightsensor",
		.addr          = 0x10,
		.flags         = 0,
		.platform_data = &cm3217_info,
	},
#endif
#ifdef WISKY_TS_FT5306
	{
		.type          = "ft5306",
		.addr          = 0x38,
		.flags         = 0,
		.irq           = TS_INT_PIN,
		.platform_data = &ft5306_info,
	},
#endif
#ifdef WISKY_TS_FT5302
	{
		.type          = "ft5302",
		.addr          = 0x38,
		.flags         = 0,
		.irq           = TS_INT_PIN,
		.platform_data = &ft5302_info,
	},
#endif 

#ifdef WISKY_TS_ZET6221
	{
		.type          = "zet6221_ts",
		.addr          = 0x76,
		.flags         = 0,
		.irq           = TS_INT_PIN,
		.platform_data = &zet6221_ts_info,
	},
#endif
#ifdef WISKY_TS_FT5X0X
	{
		.type          = "ft5x0x",
		.addr          = 0x38,
		.flags         = 0,
		.irq           = TS_INT_PIN,
		.platform_data = &ft5x0x_info,
	},
#endif
#ifdef WISKY_TS_GT8XX
	{
		.type          = "Goodix-TS",
		.addr          = 0x5d,
		.flags         = 0,
		.irq           = TS_INT_PIN,
		.platform_data = &gt82x_info,
	},
#endif
//wisky[e]
};
#endif

#ifdef CONFIG_I2C3_RK30
static struct i2c_board_info __initdata i2c3_info[] = {
};
#endif

#ifdef CONFIG_I2C4_RK30
static struct i2c_board_info __initdata i2c4_info[] = {
};
#endif

#ifdef CONFIG_I2C_GPIO_RK30
#define I2C_SDA_PIN     INVALID_GPIO// RK30_PIN2_PD6   //set sda_pin here
#define I2C_SCL_PIN     INVALID_GPIO//RK30_PIN2_PD7   //set scl_pin here
static int rk30_i2c_io_init(void)
{
        //set iomux (gpio) here
        //rk30_mux_api_set(GPIO2D7_I2C1SCL_NAME, GPIO2D_GPIO2D7);
        //rk30_mux_api_set(GPIO2D6_I2C1SDA_NAME, GPIO2D_GPIO2D6);

        return 0;
}
struct i2c_gpio_platform_data default_i2c_gpio_data = {
       .sda_pin = I2C_SDA_PIN,
       .scl_pin = I2C_SCL_PIN,
       .udelay = 5, // clk = 500/udelay = 100Khz
       .timeout = 100,//msecs_to_jiffies(100),
       .bus_num    = 5,
       .io_init = rk30_i2c_io_init,
};
static struct i2c_board_info __initdata i2c_gpio_info[] = {
};
#endif

static void __init rk30_i2c_register_board_info(void)
{
#ifdef CONFIG_I2C0_RK30
	i2c_register_board_info(0, i2c0_info, ARRAY_SIZE(i2c0_info));
#endif
#ifdef CONFIG_I2C1_RK30
	i2c_register_board_info(1, i2c1_info, ARRAY_SIZE(i2c1_info));
#endif
#ifdef CONFIG_I2C2_RK30
	i2c_register_board_info(2, i2c2_info, ARRAY_SIZE(i2c2_info));
#endif
#ifdef CONFIG_I2C3_RK30
	i2c_register_board_info(3, i2c3_info, ARRAY_SIZE(i2c3_info));
#endif
#ifdef CONFIG_I2C4_RK30
	i2c_register_board_info(4, i2c4_info, ARRAY_SIZE(i2c4_info));
#endif
#ifdef CONFIG_I2C_GPIO_RK30
	i2c_register_board_info(5, i2c_gpio_info, ARRAY_SIZE(i2c_gpio_info));
#endif
}
//end of i2c

#define POWER_ON_PIN 	KEY_SHUTDOWN_PIN//RK30_PIN6_PB0   //power_hold
static void rk30_pm_power_off(void)
{
	printk(KERN_ERR "rk30_pm_power_off start...\n");

	gpio_set_value(BL_EN_PIN, !BL_EN_VALUE);
printk("backlight disable...\n");
//	lcd_io_disable();
printk("rk30_pm_power_off... ...\n");
	
	gpio_direction_output(POWER_ON_PIN, GPIO_LOW);
	#if defined(CONFIG_MFD_WM831X)	
	if(g_pmic_type == PMIC_TYPE_WM8326)
	{
		wm831x_set_bits(Wm831x,WM831X_GPIO_LEVEL,0x0001,0x0000);  //set sys_pwr 0
		wm831x_device_shutdown(Wm831x);//wm8326 shutdown
	}
	#endif
	#if defined(CONFIG_MFD_TPS65910)
	if(g_pmic_type == PMIC_TYPE_TPS65910)
	{
		tps65910_device_shutdown();//tps65910 shutdown
	}
	#endif

#if defined(CONFIG_KP_AXP)
	printk("%s: axp set power off", __func__);
	void axp_power_off(void);
	axp_power_off();
	//axp_set_bits(&axp->dev, AXP_OFF_CTL, 0x80);
	//mdelay(20);
#endif
	while (1);
}


static void audio_gpio_init(void)
{
#if defined(AUDIO_PWR_EN_GPIO)
	if(AUDIO_PWR_EN_GPIO != INVALID_GPIO){
		rk30_mux_api_set(AUDIO_PWR_EN_MUX_NAME, AUDIO_PWR_EN_MUX_MODE);
		gpio_request(AUDIO_PWR_EN_GPIO, NULL);
		gpio_direction_output(AUDIO_PWR_EN_GPIO, AUDIO_PWR_EN_VALUE);
	}
#endif
}

static void camera_gpio_init(void)
{
#if 1
#if defined(CAMERA_PWR_EN_PIN)
	if(CAMERA_PWR_EN_PIN != INVALID_GPIO){
		rk30_mux_api_set(CAMERA_PWR_EN_MUX_NAME, CAMERA_PWR_EN_MUX_MODE);
		gpio_request(CAMERA_PWR_EN_PIN, "camera power enalbe");
		gpio_direction_output(CAMERA_PWR_EN_PIN, CAMERA_PWR_EN_VALUE);
		//gpio_set_value(CAMERA_PWR_EN_PIN, GPIO_HIGH);
	}
	#if 0
	//ltm++
		rk30_mux_api_set(GPIO1C7_CIFDATA9_RMIIRXD0_NAME, GPIO1C_GPIO1C7);
		gpio_request(RK30_PIN1_PC7, "camera 0 reset");
		//ltm++
	gpio_direction_output(RK30_PIN1_PC7, GPIO_HIGH);
	mdelay(10);
	gpio_set_value(RK30_PIN1_PC7, GPIO_LOW);
	mdelay(10);
	gpio_set_value(RK30_PIN1_PC7, GPIO_HIGH);
	mdelay(10);
	//gpio_pull_updown(RK30_PIN1_PC7, GPIOPullDown);
	#endif
	#if 0
	//wisky-lxh@20110608^摄像头PDN控制脚提前到board_init里面去初始化，否则导致其他I2C设备启动失败
	if(CAMERA_BACK_PDN_MUX_NAME != NULL){
		rk29_mux_api_set(CAMERA_BACK_PDN_MUX_NAME, CAMERA_BACK_PDN_MUX_MODE);
	}
	if (CAMERA_BACK_PDN_PIN != INVALID_GPIO)
	{
		printk("init PND0:%d\n",CAMERA_BACK_PDN_PIN);
		if(gpio_request(CAMERA_BACK_PDN_PIN,NULL) != 0){
			gpio_free(CAMERA_BACK_PDN_PIN);
			printk("camera PND0 failed\n");
			return -EIO;
		}
		gpio_direction_output(CAMERA_BACK_PDN_PIN, 1);
		gpio_set_value(CAMERA_BACK_PDN_PIN,GPIO_HIGH);
		
		gpio_pull_updown(CAMERA_BACK_PDN_PIN, GPIOPullDown);
		mdelay(10);
	}

	if(CAMERA_FRONT_PDN_MUX_NAME != NULL){
		rk29_mux_api_set(CAMERA_FRONT_PDN_MUX_NAME, CAMERA_FRONT_PDN_MUX_MODE);
	}
	if (CAMERA_FRONT_PDN_PIN != INVALID_GPIO)
	{
		if(gpio_request(CAMERA_FRONT_PDN_PIN,NULL) != 0){
			gpio_free(CAMERA_FRONT_PDN_PIN);
			printk("camera PND1 failed\n");
			return -EIO;
		}
		gpio_direction_output(CAMERA_FRONT_PDN_PIN, 1);
		gpio_set_value(CAMERA_FRONT_PDN_PIN,GPIO_HIGH);
		
		gpio_pull_updown(CAMERA_BACK_PDN_PIN, GPIOPullDown);
		mdelay(10);
	}
	
	
	
//wisky[s]cd huang@20120417^free gpio after setup camera PDN
	gpio_free(CAMERA_BACK_PDN_PIN);
	gpio_free(CAMERA_FRONT_PDN_PIN);
	#endif
#endif
#endif

}


static void battery_charge_gpio_init(void)
{
	int ret;
	
#if defined(BATTERY_CHARGE_PWR_EN_PIN)
	if(BATTERY_CHARGE_PWR_EN_PIN != INVALID_GPIO){
		rk30_mux_api_set(BATTERY_CHARGE_PWR_EN_MUX_NAME, BATTERY_CHARGE_PWR_EN_MUX_MODE);
		ret = gpio_request(BATTERY_CHARGE_PWR_EN_PIN, "charge power supply enable");
		if(ret){
			pr_err("request battery charge power supply enable gpio failed!\n");
		}
		gpio_direction_output(BATTERY_CHARGE_PWR_EN_PIN, !BATTERY_CHARGE_PWR_EN_VALUE);
	}
#endif

#if defined(BATTERY_CHARGE_EN_PIN)
	if(BATTERY_CHARGE_EN_PIN >= 0){
		rk30_mux_api_set(BATTERY_CHARGE_EN_MUX_NAME, BATTERY_CHARGE_EN_MUX_MODE);
		ret = gpio_request(BATTERY_CHARGE_EN_PIN, "battery charge en");
		if(ret){
			pr_err("request battery charge enable gpio failed!\n");
		}
		gpio_direction_output(BATTERY_CHARGE_EN_PIN, !BATTERY_CHARGE_EN_VALUE);//disable battery charge
	}
#endif
}
//wisky[e]

static void backlight_gpio_init(void)
{

	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE_GPIO);
	mdelay(20);
	if (gpio_request(PWM_GPIO, NULL)) {
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
	}
	gpio_direction_output(PWM_GPIO, !PWM_EFFECT_VALUE);

	
	#if defined(BL_EN_PIN)
	if(BL_EN_PIN != INVALID_GPIO){
		rk30_mux_api_set(BL_EN_MUX_NAME, BL_EN_MUX_MODE);
		gpio_request(BL_EN_PIN, NULL);
		gpio_direction_output(BL_EN_PIN, !BL_EN_VALUE);
	}
	#endif
}

#if WISKY_BOARD_U7PLUS_TV10
static void charge_blink_gpio_init(void)
{

	rk30_mux_api_set(CHARGE_BLINK_MUX_NAME, CHARGE_BLINK_MUX_GPIO);
	mdelay(20);
	if (gpio_request(CHARGE_BLINK_PIN, NULL)) {
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
	}
	gpio_direction_output(CHARGE_BLINK_PIN, CHARGE_BLINK_EFFECT_VALUE);
	
}
#endif


#if defined(CONFIG_KP_AXP)
extern void axp_power_off(void);
#endif

static void __init machine_rk30_board_init(void)
{
	avs_init();
	gpio_request(POWER_ON_PIN, "poweronpin");
	gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);
	
	#if defined(CONFIG_KP_AXP)
	pm_power_off = axp_power_off;
	#else
	pm_power_off = rk30_pm_power_off;
	#endif
	
	rk30_i2c_register_board_info();
	spi_register_board_info(board_spi_devices, ARRAY_SIZE(board_spi_devices));
	platform_add_devices(devices, ARRAY_SIZE(devices));
	//board_usb_detect_init(RK30_PIN6_PA3);
	board_usb_detect_init(USB_DETECT_PIN);
#if WISKY_BOARD_U7PLUS_TV10
	charge_blink_gpio_init();
#endif
	backlight_gpio_init();
	audio_gpio_init();
	camera_gpio_init();	
	battery_charge_gpio_init();

#ifdef CONFIG_WIFI_CONTROL_FUNC
	rk29sdk_wifi_bt_gpio_control_init();
#endif
}

static void __init rk30_reserve(void)
{
#ifdef CONFIG_ION
	rk30_ion_pdata.heaps[0].base = board_mem_reserve_add("ion", ION_RESERVE_SIZE);
#endif
#ifdef CONFIG_FB_ROCKCHIP
   resource_fb[0].start = board_mem_reserve_add("fb0 buf", RK30_FB0_MEM_SIZE);
   resource_fb[0].end = resource_fb[0].start + RK30_FB0_MEM_SIZE - 1;
#if 0
        resource_fb[1].start = board_mem_reserve_add("ipp buf", RK30_FB0_MEM_SIZE);
        resource_fb[1].end = resource_fb[1].start + RK30_FB0_MEM_SIZE - 1;
#endif

#if defined(CONFIG_FB_ROTATE) || !defined(CONFIG_THREE_FB_BUFFER)
   resource_fb[2].start = board_mem_reserve_add("fb2 buf", RK30_FB0_MEM_SIZE);
	resource_fb[2].end = resource_fb[2].start + RK30_FB0_MEM_SIZE - 1;
#endif
#endif
#ifdef CONFIG_VIDEO_RK29
	rk30_camera_request_reserve_mem();
#endif
	board_mem_reserved();
}

/**
 * dvfs_cpu_logic_table: table for arm and logic dvfs 
 * @frequency	: arm frequency
 * @cpu_volt	: arm voltage depend on frequency
 * @logic_volt	: logic voltage arm requests depend on frequency
 * comments	: min arm/logic voltage
 */
#if defined(WISKY_MAX_CPUFREQ_1200M)
 static struct dvfs_arm_table dvfs_cpu_logic_table[] = {
	{.frequency = 252 * 1000,	.cpu_volt = 1075 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 504 * 1000,	.cpu_volt = 1100 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 816 * 1000,	.cpu_volt = 1125 * 1000,	.logic_volt = 1150 * 1000},///1.000V/1.025V
	{.frequency = 1008 * 1000,	.cpu_volt = 1125 * 1000,	.logic_volt = 1150 * 1000},///1.025V/1.050V
	{.frequency = 1200 * 1000,	.cpu_volt = 1175 * 1000,	.logic_volt = 1200 * 1000},///1.100V/1.050V
	{.frequency = 1272 * 1000,	.cpu_volt = 1225 * 1000,	.logic_volt = 1200 * 1000},///1.150V/1.100V

	{.frequency = CPUFREQ_TABLE_END},
};
#elif defined(WISKY_MAX_CPUFREQ_1400M)
 static struct dvfs_arm_table dvfs_cpu_logic_table[] = {
	{.frequency = 252 * 1000,	.cpu_volt = 1075 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 504 * 1000,	.cpu_volt = 1100 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 816 * 1000,	.cpu_volt = 1125 * 1000,	.logic_volt = 1150 * 1000},///1.000V/1.025V
	{.frequency = 1008 * 1000,	.cpu_volt = 1125 * 1000,	.logic_volt = 1150 * 1000},///1.025V/1.050V
	{.frequency = 1200 * 1000,	.cpu_volt = 1175 * 1000,	.logic_volt = 1200 * 1000},///1.100V/1.050V
	{.frequency = 1272 * 1000,	.cpu_volt = 1225 * 1000,	.logic_volt = 1200 * 1000},///1.150V/1.100V
	{.frequency = 1416 * 1000,	.cpu_volt = 1300 * 1000,	.logic_volt = 1200 * 1000},///1.225V/1.100V

	{.frequency = CPUFREQ_TABLE_END},
};
#elif defined(WISKY_MAX_CPUFREQ_1500M)
 static struct dvfs_arm_table dvfs_cpu_logic_table[] = {
	{.frequency = 252 * 1000,	.cpu_volt = 1075 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 504 * 1000,	.cpu_volt = 1100 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 816 * 1000,	.cpu_volt = 1125 * 1000,	.logic_volt = 1150 * 1000},///1.000V/1.025V
	{.frequency = 1008 * 1000,	.cpu_volt = 1125 * 1000,	.logic_volt = 1150 * 1000},///1.025V/1.050V
	{.frequency = 1200 * 1000,	.cpu_volt = 1175 * 1000,	.logic_volt = 1200 * 1000},///1.100V/1.050V
	{.frequency = 1272 * 1000,	.cpu_volt = 1250 * 1000,	.logic_volt = 1200 * 1000},///1.150V/1.100V
	{.frequency = 1416 * 1000,	.cpu_volt = 1325 * 1000,	.logic_volt = 1200 * 1000},///1.225V/1.100V
	{.frequency = 1512 * 1000,	.cpu_volt = 1370 * 1000,	.logic_volt = 1250 * 1000},///1.300V/1.150V
	{.frequency = CPUFREQ_TABLE_END},
}; 
#elif defined(WISKY_MAX_CPUFREQ_1600M)
 static struct dvfs_arm_table dvfs_cpu_logic_table[] = {
	{.frequency = 252 * 1000,	.cpu_volt = 1075 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 504 * 1000,	.cpu_volt = 1100 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 816 * 1000,	.cpu_volt = 1125 * 1000,	.logic_volt = 1150 * 1000},///1.000V/1.025V
	{.frequency = 1008 * 1000,	.cpu_volt = 1125 * 1000,	.logic_volt = 1150 * 1000},///1.025V/1.050V
	{.frequency = 1200 * 1000,	.cpu_volt = 1175 * 1000,	.logic_volt = 1200 * 1000},///1.100V/1.050V
	{.frequency = 1272 * 1000,	.cpu_volt = 1250 * 1000,	.logic_volt = 1200 * 1000},///1.150V/1.100V
	{.frequency = 1416 * 1000,	.cpu_volt = 1325 * 1000,	.logic_volt = 1200 * 1000},///1.225V/1.100V
	{.frequency = 1512 * 1000,	.cpu_volt = 1370 * 1000,	.logic_volt = 1250 * 1000},///1.300V/1.150V
	{.frequency = 1608 * 1000,	.cpu_volt = /*1425*/1400 * 1000,	.logic_volt = 1300 * 1000},///1.325V/1.175V
	{.frequency = CPUFREQ_TABLE_END},
};
#elif defined(WISKY_MAX_CPUFREQ_1600M_W001)
 static struct dvfs_arm_table dvfs_cpu_logic_table[] = {
	#if 0
	/*
	{.frequency = 252 * 1000,	.cpu_volt = 1170 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 504 * 1000,	.cpu_volt = 1170 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 816 * 1000,	.cpu_volt = 1170 * 1000,	.logic_volt = 1155 * 1000},///1.000V/1.025V
	*/
	{.frequency = 252 * 1000,	.cpu_volt = 1370 * 1000,	.logic_volt = 1275 * 1000},//0.975V/1.000V
	{.frequency = 504 * 1000,	.cpu_volt = 1370 * 1000,	.logic_volt = 1275 * 1000},//0.975V/1.000V
	{.frequency = 816 * 1000,	.cpu_volt = 1370 * 1000,	.logic_volt = 1275 * 1000},///1.000V/1.025V
	{.frequency = 1008 * 1000,	.cpu_volt = 1370 * 1000,	.logic_volt = 1300 * 1000},///1.025V/1.050V
	{.frequency = 1200 * 1000,	.cpu_volt = 1370 * 1000,	.logic_volt = 1300 * 1000},///1.100V/1.050V
	{.frequency = 1272 * 1000,	.cpu_volt = 1370 * 1000,	.logic_volt = 1300 * 1000},///1.150V/1.100V
	{.frequency = 1416 * 1000,	.cpu_volt = 1370 * 1000,	.logic_volt = 1300 * 1000},///1.225V/1.100V
	{.frequency = 1512 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1325 * 1000},///1.300V/1.150V
	{.frequency = 1608 * 1000,	.cpu_volt = 1475 * 1000,	.logic_volt = 1375 * 1000},///1.325V/1.175V
	{.frequency = CPUFREQ_TABLE_END},
	#else
	{.frequency = 252 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1325 * 1000},//0.975V/1.000V
	{.frequency = 504 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1325 * 1000},//0.975V/1.000V
	{.frequency = 816 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1325 * 1000},///1.000V/1.025V
	{.frequency = 1008 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1325 * 1000},///1.025V/1.050V
	{.frequency = 1200 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1325 * 1000},///1.100V/1.050V
	{.frequency = 1272 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1325 * 1000},///1.150V/1.100V
	{.frequency = 1416 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1325 * 1000},///1.225V/1.100V
	{.frequency = 1512 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1325 * 1000},///1.300V/1.150V
	{.frequency = 1608 * 1000,	.cpu_volt = 1475 * 1000,	.logic_volt = 1375 * 1000},///1.325V/1.175V
	{.frequency = CPUFREQ_TABLE_END},
	#endif
};	
#elif defined(WISKY_MAX_CPUFREQ_1600M_W008)
 static struct dvfs_arm_table dvfs_cpu_logic_table[] = {
	{.frequency = 252 * 1000,	.cpu_volt = 1175 * 1000,	.logic_volt = 1200 * 1000},//0.975V/1.000V
	{.frequency = 504 * 1000,	.cpu_volt = 1175 * 1000,	.logic_volt = 1200 * 1000},//0.975V/1.000V
	{.frequency = 816 * 1000,	.cpu_volt = 1250 * 1000,	.logic_volt = 1200 * 1000},///1.000V/1.025V
	{.frequency = 1008 * 1000,	.cpu_volt = 1250 * 1000,	.logic_volt = 1200 * 1000},///1.025V/1.050V
	{.frequency = 1200 * 1000,	.cpu_volt = 1325 * 1000,	.logic_volt = 1300 * 1000},///1.100V/1.050V
	{.frequency = 1272 * 1000,	.cpu_volt = 1325 * 1000,	.logic_volt = 1325 * 1000},///1.150V/1.100V
	{.frequency = 1416 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1325 * 1000},///1.225V/1.100V
	{.frequency = 1512 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1325 * 1000},///1.300V/1.150V
	{.frequency = 1608 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1325 * 1000},///1.325V/1.175V
	{.frequency = CPUFREQ_TABLE_END},
};
#elif defined(WISKY_MAX_CPUFREQ_1600M_MOMO9)	
	static struct dvfs_arm_table dvfs_cpu_logic_table[] = {
	{.frequency = 252 * 1000,	.cpu_volt = 1370 * 1000,	.logic_volt = 1275 * 1000},//0.975V/1.000V
	{.frequency = 504 * 1000,	.cpu_volt = 1370 * 1000,	.logic_volt = 1275 * 1000},//0.975V/1.000V
	{.frequency = 816 * 1000,	.cpu_volt = 1370 * 1000,	.logic_volt = 1275 * 1000},///1.000V/1.025V
	{.frequency = 1008 * 1000,	.cpu_volt = 1370 * 1000,	.logic_volt = 1300 * 1000},///1.025V/1.050V
	{.frequency = 1200 * 1000,	.cpu_volt = 1370 * 1000,	.logic_volt = 1300 * 1000},///1.100V/1.050V
	{.frequency = 1272 * 1000,	.cpu_volt = 1370 * 1000,	.logic_volt = 1300 * 1000},///1.150V/1.100V
	{.frequency = 1416 * 1000,	.cpu_volt = 1370 * 1000,	.logic_volt = 1300 * 1000},///1.225V/1.100V
	{.frequency = 1512 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1325 * 1000},///1.300V/1.150V
	{.frequency = 1608 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1325 * 1000},///1.325V/1.175V
	{.frequency = CPUFREQ_TABLE_END},
};
#else
static struct dvfs_arm_table dvfs_cpu_logic_table[] = {
	{.frequency = 252 * 1000,	.cpu_volt = 1075 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 504 * 1000,	.cpu_volt = 1100 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 816 * 1000,	.cpu_volt = 1125 * 1000,	.logic_volt = 1150 * 1000},//1.000V/1.025V
	{.frequency = 1008 * 1000,	.cpu_volt = 1125 * 1000,	.logic_volt = 1150 * 1000},//1.025V/1.050V
	{.frequency = 1200 * 1000,	.cpu_volt = 1175 * 1000,	.logic_volt = 1200 * 1000},//1.100V/1.050V
	{.frequency = 1272 * 1000,	.cpu_volt = 1225 * 1000,	.logic_volt = 1200 * 1000},//1.150V/1.100V
	{.frequency = 1416 * 1000,	.cpu_volt = 1300 * 1000,	.logic_volt = 1200 * 1000},//1.225V/1.100V
	{.frequency = 1512 * 1000,	.cpu_volt = 1350 * 1000,	.logic_volt = 1250 * 1000},//1.300V/1.150V
	{.frequency = 1608 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1300 * 1000},//1.325V/1.175V
	{.frequency = CPUFREQ_TABLE_END},
};
 #endif

static struct cpufreq_frequency_table dvfs_gpu_table[] = {
	{.frequency = 266 * 1000,	.index = 1050 * 1000},
	{.frequency = 400 * 1000,	.index = 1275 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table[] = {
	{.frequency = 300 * 1000,	.index = 1050 * 1000},
	{.frequency = 400 * 1000,	.index = 1125 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

#define DVFS_CPU_TABLE_SIZE	(ARRAY_SIZE(dvfs_cpu_logic_table))
static struct cpufreq_frequency_table cpu_dvfs_table[DVFS_CPU_TABLE_SIZE];
static struct cpufreq_frequency_table dep_cpu2core_table[DVFS_CPU_TABLE_SIZE];

void __init board_clock_init(void)
{
	rk30_clock_data_init(periph_pll_default, codec_pll_default, RK30_CLOCKS_DEFAULT_FLAGS);
	dvfs_set_arm_logic_volt(dvfs_cpu_logic_table, cpu_dvfs_table, dep_cpu2core_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "gpu"), dvfs_gpu_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table);
}

MACHINE_START(RK30, "RK30board")
	.boot_params	= PLAT_PHYS_OFFSET + 0x800,
	.fixup		= rk30_fixup,
	.reserve	= &rk30_reserve,
	.map_io		= rk30_map_io,
	.init_irq	= rk30_init_irq,
	.timer		= &rk30_timer,
	.init_machine	= machine_rk30_board_init,
MACHINE_END
