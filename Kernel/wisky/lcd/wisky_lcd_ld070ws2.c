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


#define OUT_CLK	          51200000
#define LCDC_ACLK        300000000           //29 lcdc axi DMA 频率

/* Timing */
#define H_PW			10
#define H_BP			160
#define H_VD			WISKY_LCD_WIDTH
#define H_FP			160

#define V_PW			2
#define V_BP			23
#define V_VD			WISKY_LCD_HEIGHT
#define V_FP			12

//根据LCD 屏资料填写有效区域
#define LCD_ACTIVE_WIDTH_MM          154
#define LCD_ACTIVE_HEIGHT_MM         90

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



#define LCD_delay_us udelay

#define td043_spi_scen(data)		gpio_set_value(LCD_SPI_CS_PIN, data)
#define td043_spi_scl(data)		gpio_set_value(LCD_SPI_SCL_PIN, data)
#define td043_spi_sda(data)		gpio_set_value(LCD_SPI_SDA_PIN, data)

void td043_spi_wr(__u8 Addr, __u8 Data)
{
	int i;

	LCD_delay_us(8);

	td043_spi_scen(1);//CS_SET();
	td043_spi_sda(1);//TXD_SET();
	td043_spi_scl(1);//CLK_SET();
	//DRVDelayUs(2);
	LCD_delay_us(4);

	td043_spi_scen(0);//CS_CLR();
	for(i = 0; i < 6; i++)  //reg
	{
		if(Addr &(1<<(5-i)))
			td043_spi_sda(1);//TXD_SET();
		else
			td043_spi_sda(0);//TXD_CLR();

		// 模拟CLK
		td043_spi_scl(0);//CLK_CLR();
		LCD_delay_us(8);
		td043_spi_scl(1);//CLK_SET();
		//DRVDelayUs(4);
		LCD_delay_us(8);
	}

	td043_spi_sda(0);//TXD_CLR();  //write

	// 模拟CLK
	td043_spi_scl(0);//CLK_CLR();
	//DRVDelayUs(4);
	LCD_delay_us(8);
	td043_spi_scl(1);//CLK_SET();
	//DRVDelayUs(4);
	LCD_delay_us(8);

	td043_spi_sda(1);//TXD_SET();  //highz

	// 模拟CLK
	td043_spi_scl(0);//CLK_CLR();
	// DRVDelayUs(4);
	LCD_delay_us(8);
	td043_spi_scl(1);//CLK_SET();
	//DRVDelayUs(4);
	LCD_delay_us(8);

      
	for(i = 0; i < 8; i++)  //data
	{
		if(Data &(1<<(7-i)))
			td043_spi_sda(1);//TXD_SET();
		else
			td043_spi_sda(0);//TXD_CLR();

		// 模拟CLK
		td043_spi_scl(0);//CLK_CLR();
		//DRVDelayUs(4);
		LCD_delay_us(8);
		td043_spi_scl(1);//CLK_SET();
		//DRVDelayUs(4);
		LCD_delay_us(8);
	}

	td043_spi_scen(1);//CS_SET();
	td043_spi_scl(0);//CLK_CLR();
	td043_spi_sda(0);//TXD_CLR();
	//DRVDelayUs(2);
	LCD_delay_us(500);

}

static void td043_init(void)
{	
	td043_spi_wr(0x00,0xaf); 
	td043_spi_wr(0x01,0x30);
	td043_spi_wr(0x02,0x40);
	td043_spi_wr(0x0e,0x5f); 
	td043_spi_wr(0x0f,0xa4);
	td043_spi_wr(0x0d,0x00);
	td043_spi_wr(0x02,0x43);
	td043_spi_wr(0x0a,0x28);
	td043_spi_wr(0x10,0x41);
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

	if(LCD_SPI_CS_PIN != INVALID_GPIO){
		rk30_mux_api_set(LCD_SPI_CS_MUX_NAME, LCD_SPI_CS_MUX_MODE);
		ret = gpio_request(LCD_SPI_CS_PIN, NULL);
		if (ret != 0){
			printk(KERN_ERR "request lcd spi cs pin fail!\n");
		}else{
			gpio_direction_output(LCD_SPI_CS_PIN, 1);
			gpio_set_value(LCD_SPI_CS_PIN, GPIO_LOW);
		}
	}

	if(LCD_SPI_SCL_PIN != INVALID_GPIO){
		rk30_mux_api_set(LCD_SPI_SCL_MUX_NAME, LCD_SPI_SCL_MUX_MODE);
		ret = gpio_request(LCD_SPI_SCL_PIN, NULL);
		if (ret != 0){
			printk(KERN_ERR "request lcd spi scl pin fail!\n");
		}else{
			gpio_direction_output(LCD_SPI_SCL_PIN, 1);
			gpio_set_value(LCD_SPI_SCL_PIN, GPIO_LOW);
		}
	}

	if(LCD_SPI_SDA_PIN != INVALID_GPIO){
		rk30_mux_api_set(LCD_SPI_SDA_MUX_NAME, LCD_SPI_SDA_MUX_MODE);
		ret = gpio_request(LCD_SPI_SDA_PIN, NULL);
		if (ret != 0){
			printk(KERN_ERR "request lcd spi sda pin fail!\n");
		}else{
			gpio_direction_output(LCD_SPI_SDA_PIN, 1);
			gpio_set_value(LCD_SPI_SDA_PIN, GPIO_LOW);
		}
	}

	td043_init();
	
	return 0;
}

int lcd_io_disable(void)
{
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
		gpio_set_value(RK_LCD_EN_PIN, !RK_LCD_EN_VALUE);
	}

	return 0;
}

int lcd_io_enable(void)
{
	if(RK_LCD_STANDBY_PIN != INVALID_GPIO){
		gpio_set_value(RK_LCD_STANDBY_PIN, RK_LCD_STANDBY_VALUE);
		mdelay(5);
	}	
	if(RK_LCD_EN_PIN != INVALID_GPIO){
		gpio_set_value(RK_LCD_EN_PIN, RK_LCD_EN_VALUE);
	}
	if(RK_LCD_RESET_PIN != INVALID_GPIO){
		mdelay(50);
		gpio_set_value(RK_LCD_RESET_PIN, RK_LCD_RESET_VALUE);
	}
	if(LCD_CS_PIN != INVALID_GPIO){
		gpio_set_value(LCD_CS_PIN, LCD_CS_VALUE);
	}

	td043_init();
	
	return 0;
}
//wisky[e]


