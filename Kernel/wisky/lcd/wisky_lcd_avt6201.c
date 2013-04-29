#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include "../../drivers/video/rk29_fb.h"
#include "../../drivers/video/display/screen/screen.h"
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>

#include "wisky_lcd_avt6201.h"
#include "wisky_lcd_avt6201_ioctl.h"

/* Base */
#define OUT_TYPE		SCREEN_MCU
#define OUT_FACE		OUT_P16BPP4

/* Timing */
#define H_PW			1
#define H_BP			1
#ifdef AVT6201_RGB565_MODE
#define H_VD			BS60_INIT_HSIZE
#else
#define H_VD			BS60_INIT_VSIZE
#endif
#define H_FP			5

#define V_PW			1
#define V_BP			1
#ifdef AVT6201_RGB565_MODE
#define V_VD			BS60_INIT_VSIZE
#else
#define V_VD			BS60_INIT_HSIZE
#endif
#define V_FP			1

#define P_WR            70

/* Other */
#define DCLK_POL		0
#define SWAP_RB			1


static int avt6201_if_refresh(u8 arg);

#define GPIO_RESET_L		NULL_GPIO//	//reset pin
#if 0
#define GPIO_HIRQ			GPIOPortC_Pin1	//IRQ
#define GPIO_HDC			GPIOPortC_Pin2	//Data(HIHG) or Command(LOW)
#define GPIO_HCS_L		GPIOPortC_Pin3	//Chip select
#define GPIO_HRD_L		GPIOPortC_Pin4	//Read mode, low active
#define GPIO_HWE_L		GPIOPortC_Pin5	//Write mode, low active
#endif
#define GPIO_HRDY		NULL_GPIO//	//Bus ready
#define GPIO_RMODE		NULL_GPIO//	//rmode ->CNF1


//----------------------------------------------------------------------------
// PRIVATE FUNCTION:
// Set registers to initial values
//----------------------------------------------------------------------------

static int avt6201_if_wait_for_ready(void)
{
    int cnt = 1000;
    int d = gpio_get_value(GPIO_HRDY);

    while (d == 0)
    {
        //mdelay(1);
        msleep(2);

        if (--cnt <= 0)         // Avoid endless loop
        {
            printk(">>>>>> wait_for_ready -- timeout! \n");
            return -1;
        }

        d = gpio_get_value(GPIO_HRDY);
    }

    return 0;
}


static int avt6201_if_cmd(unsigned ioctlcmd, avt6201_ioctl_cmd_params *params, int numparams)
{
	int i;

	if(avt6201_if_wait_for_ready() != 0){
		return -1;
	}

	mcu_ioctl(MCU_WRCMD, ioctlcmd);
	for (i = 0; i < numparams; i++) {
		mcu_ioctl(MCU_WRDATA, params->param[i]);
	}

	return 0;
}

static int avt6201_if_write_reg16(u16 Index, u16 Value)
{
	if(avt6201_if_wait_for_ready() != 0){
		return -1;
	}

	mcu_ioctl(MCU_WRCMD, WR_REG);
	mcu_ioctl(MCU_WRDATA, Index);
	mcu_ioctl(MCU_WRDATA, Value);

	return 0;
}


static void avt6201_fb_InitRegisters(void)
{
	unsigned i, cmd,j,numparams;
	avt6201_ioctl_cmd_params cmd_params;
	S1D_INSTANTIATE_REGISTERS(static,InitCmdArray);

	i = 0;
	while (i < sizeof(InitCmdArray)/sizeof(InitCmdArray[0]))
	{
		cmd = InitCmdArray[i++];
		numparams = InitCmdArray[i++];

		for (j = 0; j < numparams; j++)
		cmd_params.param[j] = InitCmdArray[i++];

		avt6201_if_cmd(cmd,&cmd_params,numparams);
	}
}

static void avt6201_if_init_gpio(void)
{
//	rockchip_mux_api_set(GPIOB_SPI0_MMC0_NAME, IOMUXA_GPIO0_B567);
	gpio_direction_output(GPIO_RESET_L, GPIO_HIGH);
//	rockchip_mux_api_set(GPIOG1_UART0_MMC1WPT_NAME, IOMUXA_GPIO1_C1);
	gpio_direction_input(GPIO_HRDY);
	
}

static int avt6201_if_set_reset(void)
{
	gpio_set_value(GPIO_RMODE, GPIO_HIGH);

	// reset pulse
	gpio_set_value(GPIO_RESET_L, GPIO_HIGH);
	mdelay(20);
	gpio_set_value(GPIO_RESET_L, GPIO_LOW);
	mdelay(6);
	gpio_set_value(GPIO_RESET_L, GPIO_HIGH);
	mdelay(10);

	if(avt6201_if_wait_for_ready() != 0){
		return -1;
	}
	return 0;
}


static int avt6201_if_display_test(void)
{
	avt6201_ioctl_cmd_params cmd_params;
	unsigned mode;
	unsigned cmd = UPD_FULL;
	unsigned int i;
	
	pr_info("%s:%d--->test..........\n", __FUNCTION__, __LINE__);
	mcu_ioctl(MCU_SETBYPASS, 1);

//	avt6201_if_write_reg16(0x330,0x84);              // LUT AutoSelect + P4N
	// Copy virtual framebuffer to hardware via indirect interface burst mode write
	avt6201_if_cmd(WAIT_DSPE_TRG,&cmd_params,0);
	if(S1D_DISPLAY_BPP == DISPLAY_8BPP){
		cmd_params.param[0] = (0x3<<4);
	}else if(S1D_DISPLAY_BPP == DISPLAY_4BPP){
		cmd_params.param[0] = (0x2<<4);
	}
	avt6201_if_cmd(LD_IMG,&cmd_params,1);
	cmd_params.param[0] = 0x154;
	avt6201_if_cmd(WR_REG,&cmd_params,1);

#ifdef AVT6201_RGB565_MODE
	for(i=0; i<BS60_INIT_VSIZE*BS60_INIT_HSIZE; i++) {
		mcu_ioctl(MCU_WRDATA, 0xFFFF);
	}
#else
	for(i=0; i<BS60_INIT_VSIZE*512; i++) {
		mcu_ioctl(MCU_WRDATA, 0xFFFF);
	}
#endif
	
	avt6201_if_cmd(LD_IMG_END,&cmd_params,0);
	if(S1D_DISPLAY_BPP == DISPLAY_8BPP){
		mode = WF_MODE_GC;
	}else{
		mode = WF_MODE_GU;
	}
	cmd_params.param[0] = (mode<<8);
	avt6201_if_cmd(cmd, &cmd_params,1);              // update all pixels
	avt6201_if_cmd(WAIT_DSPE_TRG,&cmd_params,0);
	avt6201_if_cmd(WAIT_DSPE_FREND,&cmd_params,0);

	mcu_ioctl(MCU_SETBYPASS, 0);

	return 0;
}

static int avt6201_if_init_blank(void)
{
	int i=0;
	avt6201_ioctl_cmd_params cmd_params;

	mcu_ioctl(MCU_SETBYPASS, 1);
#if 1
//while(1){
//	pr_info("%s:%d--->display test!\n", __FUNCTION__, __LINE__);
//	avt6201_if_write_reg16(0x330,0x84);              // LUT AutoSelect+P4N
	avt6201_if_cmd(WAIT_DSPE_TRG,&cmd_params,0);
	if(S1D_DISPLAY_BPP == DISPLAY_8BPP){
		cmd_params.param[0] = (0x3<<4);
	}else if(S1D_DISPLAY_BPP == DISPLAY_4BPP){
		cmd_params.param[0] = (0x2<<4);
	}
	avt6201_if_cmd(LD_IMG,&cmd_params,1);

	cmd_params.param[0] = 0x154;
	avt6201_if_cmd(WR_REG,&cmd_params,1);

#ifdef AVT6201_RGB565_MODE
	for(i=0; i<BS60_INIT_VSIZE*BS60_INIT_HSIZE; i++) {
		mcu_ioctl(MCU_WRDATA, 0xFFFF);
	}
#else
	for(i=0; i<BS60_INIT_VSIZE*BS60_INIT_HSIZE/2; i++) {
		mcu_ioctl(MCU_WRDATA, 0xFFFF);
	}
#endif

	avt6201_if_cmd(LD_IMG_END,&cmd_params,0);
	cmd_params.param[0] = ((WF_MODE_INIT<<8) |0x4000);
	avt6201_if_cmd(UPD_FULL,&cmd_params,1);         // update all pixels

	avt6201_if_cmd(WAIT_DSPE_TRG,&cmd_params,0);

	avt6201_if_cmd(WAIT_DSPE_FREND,&cmd_params,0);

//mdelay(5000);}
#endif

	mcu_ioctl(MCU_SETBYPASS, 0);
//	mdelay(1000);
#if 0
	//for test
	while(1){
		avt6201_if_display_test();
		mdelay(3000);}
#endif

    return 0;
}

static int avt6201_if_init(void)
{
	avt6201_if_init_gpio();
	avt6201_if_set_reset();

	mcu_ioctl(MCU_SETBYPASS, 1);

	avt6201_fb_InitRegisters();

#ifdef AVT6201_RGB565_MODE
	avt6201_if_write_reg16(0x0164, 0x1);
	avt6201_if_write_reg16(0x0140, 0x0880);//0x0880
#endif

	mcu_ioctl(MCU_SETBYPASS, 0);
//	mdelay(1000);

    return 0;
}

static int avt6201_if_lcd_init(void)
{
	avt6201_if_init();
	avt6201_if_init_blank();
	
	return 0;
}

int avt6201_if_standby(u8 enable)
{
    return 0;
}

int avt6201_if_refresh(u8 arg)
{
	avt6201_ioctl_cmd_params cmd_params;
	mcu_ioctl(MCU_SETBYPASS, 1);

	switch(arg){
	case REFRESH_PRE:   //DMA传送前准备
		avt6201_if_cmd(RUN_SYS, &cmd_params, 0);
		/*
		 * Set Reg[0x0A] big[12] to avoid avt6201 Wait READY timeout after resume from SLEEP mode
		 * CD Huang@20100312
		 */
/*		avt6201_if_write_reg16(0x0A,0x1503);
		*/
//		avt6201_if_write_reg16(0x330,0x84);              // LUT AutoSelect + P4N
		avt6201_if_cmd(WAIT_DSPE_TRG,&cmd_params,0);		
		if(S1D_DISPLAY_BPP == DISPLAY_8BPP){
			cmd_params.param[0] = (0x3<<4);
		}else if(S1D_DISPLAY_BPP == DISPLAY_4BPP){
			cmd_params.param[0] = (0x2<<4);
		}
		avt6201_if_cmd(LD_IMG,&cmd_params,1);
		cmd_params.param[0] = 0x154;
		avt6201_if_cmd(WR_REG,&cmd_params,1);
		//printk(">>>>>> lcd_refresh : REFRESH_PRE!\n");
		break;

	case REFRESH_END:   //DMA传送结束后
		{
			unsigned mode;
			unsigned cmd = UPD_PART;//UPD_FULL;

			if(avt6201_if_cmd(LD_IMG_END,&cmd_params,0) == -1){
				goto Reset_lcd;
			}
			if(S1D_DISPLAY_BPP == DISPLAY_8BPP){
				mode = WF_MODE_GC;
			}else{
				mode = WF_MODE_GU;
			}
			cmd_params.param[0] = (mode<<8);
			if(avt6201_if_cmd(cmd,&cmd_params,1) == -1){
				goto Reset_lcd;
			}
			if(avt6201_if_cmd(WAIT_DSPE_TRG,&cmd_params,0) == -1){
				goto Reset_lcd;
			}
			if(avt6201_if_cmd(WAIT_DSPE_FREND,&cmd_params,0) == -1){
				goto Reset_lcd;
			}
			if(avt6201_if_cmd(SLP, &cmd_params, 0) == -1){
				goto Reset_lcd;
			}
		}
		//printk(">>>>>> lcd_refresh : REFRESH_END!\n");
		break;

	default:
		break;
	}

	mcu_ioctl(MCU_SETBYPASS, 0);

	return 0;

Reset_lcd:
	printk(">>>>>> lcd_refresh failed : reset lcd!\n");
	avt6201_if_init();
	
	return 0;
}



void set_lcd_info(struct rk29fb_screen *screen, struct rk29lcd_info *lcd_info)
{
	/* screen type & face */
	screen->type = OUT_TYPE;
	screen->face = OUT_FACE;

	/* Screen size */
	screen->x_res = H_VD;
	screen->y_res = V_VD;

	/* Timing */
	screen->left_margin = H_BP;
	screen->right_margin = H_FP;
	screen->hsync_len = H_PW;
	screen->upper_margin = V_BP;
	screen->lower_margin = V_FP;
	screen->vsync_len = V_PW;
	screen->mcu_wrperiod = P_WR;

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
	screen->init = avt6201_if_lcd_init;
	screen->standby = avt6201_if_standby;
	screen->refresh = avt6201_if_refresh;
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

	return 0;
}
//wisky[e]


