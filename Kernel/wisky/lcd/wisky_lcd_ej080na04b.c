#include <linux/fb.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>

#include "../../drivers/video/rk29_fb.h"
#include "../../drivers/video/display/screen/screen.h"

/* Base */
#define OUT_TYPE	    SCREEN_RGB

#define OUT_FACE	    OUT_D888_P666


#define OUT_CLK	          65000000
#define LCDC_ACLK        300000000           //29 lcdc axi DMA 频率

/* Timing */
#define H_PW			10
#define H_BP			100
#define H_VD			WISKY_LCD_WIDTH//1024
#define H_FP			18

#define V_PW			2
#define V_BP			8
#define V_VD			WISKY_LCD_HEIGHT//768
#define V_FP			6

//根据LCD 屏资料填写有效区域
#define LCD_ACTIVE_WIDTH_MM          162
#define LCD_ACTIVE_HEIGHT_MM         122

/* Other */
#define DCLK_POL		0
#define SWAP_RB		0


void set_lcd_info(struct rk29fb_screen *screen, struct rk29lcd_info *lcd_info )
{
    /* screen type & face */
    screen->type = OUT_TYPE;
    screen->face = OUT_FACE;

    /* Screen size */
    screen->x_res = H_VD;
    screen->y_res = V_VD;

    screen->width = LCD_ACTIVE_WIDTH_MM;
    screen->height = LCD_ACTIVE_HEIGHT_MM;

    /* Timing */
    screen->lcdc_aclk = LCDC_ACLK;
    screen->pixclock = OUT_CLK;
	screen->left_margin = H_BP;
	screen->right_margin = H_FP;
	screen->hsync_len = H_PW;
	screen->upper_margin = V_BP;
	screen->lower_margin = V_FP;
	screen->vsync_len = V_PW;

	/* Pin polarity */
	screen->pin_hsync = 0;
	screen->pin_vsync = 0;
	screen->pin_den = 0;
	screen->pin_dclk = DCLK_POL;

	/* Swap rule */
    screen->swap_rb = SWAP_RB;
    screen->swap_rg = 0;
    screen->swap_gb = 0;
    screen->swap_delta = 0;
    screen->swap_dumy = 0;

    /* Operation function*/
    screen->init = NULL;
    screen->standby = NULL;
}


#define LCD_CS_PIN         			LCD_LVDS_PDN_PIN
#define LCD_CS_MUX_NAME			LCD_LVDS_PDN_MUX_NAME
#define LCD_CS_MUX_MODE			LCD_LVDS_PDN_MUX_MODE
#define LCD_CS_VALUE       			LCD_LVDS_PDN_VALUE

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


int lcd_io_init(void)
{
	int ret = 0;

	//pr_info("%s[%d]: ------\n", __FUNCTION__, __LINE__);
	if(RK_LCD_STANDBY_PIN != INVALID_GPIO){
		rk30_mux_api_set(RK_LCD_STANDBY_MUX_NAME, RK_LCD_STANDBY_MUX_MODE);
		ret = gpio_request(RK_LCD_STANDBY_PIN, NULL);
		if(ret !=0){
			printk(KERN_ERR "request lcd standby pin failed!\n");
		}else{
			gpio_direction_output(RK_LCD_STANDBY_PIN, GPIO_HIGH);
			gpio_set_value(RK_LCD_STANDBY_PIN, RK_LCD_STANDBY_VALUE);
			mdelay(5);
		}
	}

	if(RK_LCD_EN_PIN != INVALID_GPIO){
		rk30_mux_api_set(RK_LCD_EN_MUX_NAME, RK_LCD_EN_MUX_MODE);
		ret = gpio_request(RK_LCD_EN_PIN, NULL);
		if (ret != 0){
			printk(KERN_ERR "request lcd en2 pin fail!\n");
		}
		else{
			gpio_direction_output(RK_LCD_EN_PIN, 1);
			gpio_set_value(RK_LCD_EN_PIN, RK_LCD_EN_VALUE);
		}
	}

	if(RK_LCD_RESET_PIN != INVALID_GPIO){
		rk30_mux_api_set(RK_LCD_RESET_MUX_NAME, RK_LCD_RESET_MUX_MODE);
		ret = gpio_request(RK_LCD_RESET_PIN, NULL);
		if(ret !=0){
			printk(KERN_ERR "request lcd reset pin failed!\n");
		}else{
			mdelay(50);
			gpio_pull_updown(RK_LCD_RESET_PIN, GPIOPullUp);
			gpio_direction_output(RK_LCD_RESET_PIN, GPIO_HIGH);
			gpio_set_value(RK_LCD_RESET_PIN, RK_LCD_RESET_VALUE);
		}
	}

	if(LCD_CS_PIN != INVALID_GPIO){
		rk30_mux_api_set(LCD_CS_MUX_NAME, LCD_CS_MUX_MODE);
		ret = gpio_request(LCD_CS_PIN, NULL);
		if (ret != 0){
			printk(KERN_ERR "request lcd en pin fail!\n");
		}
		else{
			gpio_direction_output(LCD_CS_PIN, 1);
			gpio_set_value(LCD_CS_PIN, LCD_CS_VALUE);
		}
	}

	return 0;
}

int lcd_io_disable(void)
{
	//pr_info("%s[%d]: ------\n", __FUNCTION__, __LINE__);
	if(LCD_CS_PIN != INVALID_GPIO){
		gpio_set_value(LCD_CS_PIN, !LCD_CS_VALUE);
	}

	if(RK_LCD_STANDBY_PIN != INVALID_GPIO){
		gpio_set_value(RK_LCD_STANDBY_PIN, !RK_LCD_STANDBY_VALUE);
	}
	if(RK_LCD_RESET_PIN != INVALID_GPIO){
		gpio_set_value(RK_LCD_RESET_PIN, !RK_LCD_RESET_VALUE);
	}
	mdelay(20);
	if(RK_LCD_EN_PIN != INVALID_GPIO){
		#if 1
		gpio_set_value(RK_LCD_EN_PIN, !RK_LCD_EN_VALUE);
		#else
		//休眠时不关闭LCD PWR EN，避免低电唤醒死机问题。
		gpio_set_value(RK_LCD_EN_PIN, RK_LCD_EN_VALUE);
		#endif
	}

	return 0;
}

void loop(int value)
{
	int i;
	
	for(i = 0; i < value; i++){
		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");
	}
}

int lcd_io_enable(void)
{
	int i, j, k;
	
	if(RK_LCD_STANDBY_PIN != INVALID_GPIO){
		gpio_set_value(RK_LCD_STANDBY_PIN, RK_LCD_STANDBY_VALUE);
		mdelay(5);
	}	
	if(RK_LCD_EN_PIN != INVALID_GPIO){
		#if 0
		gpio_set_value(RK_LCD_EN_PIN, RK_LCD_EN_VALUE);
		#else

		j = 0;
		k = 10000;

		for(i = 0; i < k; i++, j++){
			gpio_set_value(RK_LCD_EN_PIN, RK_LCD_EN_VALUE);	
			loop(1+j);
			gpio_set_value(RK_LCD_EN_PIN, !RK_LCD_EN_VALUE);			
			loop(k-j);
		}

		gpio_set_value(RK_LCD_EN_PIN, RK_LCD_EN_VALUE);
		#endif
	}
	if(RK_LCD_RESET_PIN != INVALID_GPIO){
		mdelay(50);
		gpio_set_value(RK_LCD_RESET_PIN, RK_LCD_RESET_VALUE);
	}
	if(LCD_CS_PIN != INVALID_GPIO){
		gpio_set_value(LCD_CS_PIN, LCD_CS_VALUE);
	}

	return 0;
}
//wisky[e]

