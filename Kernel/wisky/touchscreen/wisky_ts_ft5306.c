/* 
 * FocalTech ft5306 TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 *
 *	note: only support mulititouch	Wenfs 2010-10-01
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <mach/gpio.h>
#include <linux/irq.h>
#include <mach/board.h>
#include <linux/hrtimer.h>
#include <linux/timer.h>
#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#if 0
#define SINGLETOUCHE_MODE
#else
#define FT5306_MAX_SUPPORT_POINT	5
#endif
#define FT5306_DUOD_PACKGE_BUFLEN		10

#define FT5306_I2C_SPEED 	(300*1000)
#define FT5306_REG_FIRMID         0xA6

#if 0
#define CUT_WIDTH 10
#define DRIVENO	15
#define SENSENO	10

#define DEVICE_ID_REG                 2
#define VERSION_ID_REG                3
#define FINGER01_REG                  0x7c
#define EVENT_STATUS                  0x79
#endif

#define PEN_DOWN 1
#define PEN_RELEASE 0
#define MicroTimeTInterupt	5000000// 100Hz - 10,000,000us

#if 0
#define REVERSE_Y	1
#define REVERSE_X	1
#else
#define REVERSE_Y	0
#define REVERSE_X	0
#endif

struct ts_ft5306 {
	struct input_dev	*input;
	char	phys[32];
	struct hrtimer timer;
	struct delayed_work	work;
	struct workqueue_struct *wq;
	struct i2c_client	*client;
	struct ft5306_platform_data *pdata;
	u16	 model;
	u8 finger;
	int reported_finger_count;
	bool pendown;
	bool status;
	int irq;
	int has_relative_report;
	int (*get_pendown_state)(void);
	void	(*clear_penirq)(void);
};
struct ts_ft5306 *ts;

//是否支持FT5306 开机自升级
//#define FT5306_FIRMWARE_UPGRADE	1

#if defined(FT5306_FIRMWARE_UPGRADE)
#define    FTS_PACKET_LENGTH        128
static u8 CTPM_FW[]=
{
//	#include "ft_app_5306.i"
};

typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;


/***********************************************************************
    [function]: 
		           callback:                send data to ctpm by i2c interface;
    [parameters]:
			    txdata[in]:              data buffer which is used to send data;
			    length[in]:              the length of the data buffer;
    [return]:
			    FTS_TRUE:              success;
			    FTS_FALSE:             fail;
************************************************************************/
static int fts_i2c_txdata(u8 *txdata, int length)
{
	int ret;

	struct i2c_msg msg;

      msg.addr = ts->client->addr;
      msg.flags = 0;
      msg.len = length;
      msg.buf = txdata;
	ret = i2c_transfer(ts->client->adapter, &msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

/***********************************************************************
    [function]: 
		           callback:               write data to ctpm by i2c interface;
    [parameters]:
			    buffer[in]:             data buffer;
			    length[in]:            the length of the data buffer;
    [return]:
			    FTS_TRUE:            success;
			    FTS_FALSE:           fail;
************************************************************************/
static bool  i2c_write_interface(u8* pbt_buf, int dw_lenth)
{
    int ret;
    ret=i2c_master_send(ts->client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk("[FT5306]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return false;
    }

    return true;
}

/***********************************************************************
    [function]: 
		           callback:                read register value ftom ctpm by i2c interface;
    [parameters]:
                        reg_name[in]:         the register which you want to write;
			    tx_buf[in]:              buffer which is contained of the writing value;
    [return]:
			    FTS_TRUE:              success;
			    FTS_FALSE:             fail;
************************************************************************/
static bool fts_register_write(u8 reg_name, u8* tx_buf)
{
	u8 write_cmd[2] = {0};

	write_cmd[0] = reg_name;
	write_cmd[1] = *tx_buf;

	/*call the write callback function*/
	return i2c_write_interface(write_cmd, 2);
}

/***********************************************************************
[function]: 
                      callback:         send a command to ctpm.
[parameters]:
			  btcmd[in]:       command code;
			  btPara1[in]:     parameter 1;    
			  btPara2[in]:     parameter 2;    
			  btPara3[in]:     parameter 3;    
                      num[in]:         the valid input parameter numbers, 
                                           if only command code needed and no 
                                           parameters followed,then the num is 1;    
[return]:
			  FTS_TRUE:      success;
			  FTS_FALSE:     io fail;
************************************************************************/
static bool cmd_write(u8 btcmd,u8 btPara1,u8 btPara2,u8 btPara3,u8 num)
{
    u8 write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(write_cmd, num);
}

/***********************************************************************
    [function]: 
		           callback:              read data from ctpm by i2c interface;
    [parameters]:
			    buffer[in]:            data buffer;
			    length[in]:           the length of the data buffer;
    [return]:
			    FTS_TRUE:            success;
			    FTS_FALSE:           fail;
************************************************************************/
static bool i2c_read_interface(u8* pbt_buf, int dw_lenth)
{
    int ret;
    
    ret=i2c_master_recv(ts->client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("[FT5306]i2c_read_interface error\n");
        return false;
    }
  
    return true;
}


/***********************************************************************
[function]: 
                      callback:         read a byte data  from ctpm;
[parameters]:
			  buffer[in]:       read buffer;
			  length[in]:      the size of read data;    
[return]:
			  FTS_TRUE:      success;
			  FTS_FALSE:     io fail;
************************************************************************/
static bool byte_read(u8* buffer, int length)
{
    return i2c_read_interface(buffer, length);
}

/***********************************************************************
[function]: 
                      callback:         write a byte data  to ctpm;
[parameters]:
			  buffer[in]:       write buffer;
			  length[in]:      the size of write data;    
[return]:
			  FTS_TRUE:      success;
			  FTS_FALSE:     io fail;
************************************************************************/
static bool byte_write(u8* buffer, int length)
{
    
    return i2c_write_interface(buffer, length);
}

/***********************************************************************
    [function]: 
		           callback:                 read register value ftom ctpm by i2c interface;
    [parameters]:
                        reg_name[in]:         the register which you want to read;
			    rx_buf[in]:              data buffer which is used to store register value;
			    rx_length[in]:          the length of the data buffer;
    [return]:
			    FTS_TRUE:              success;
			    FTS_FALSE:             fail;
************************************************************************/
static bool fts_register_read(u8 reg_name, u8* rx_buf, int rx_length)
{
	u8 read_cmd[2]= {0};
	u8 cmd_len 	= 0;

	read_cmd[0] = reg_name;
	cmd_len = 1;	

	/*send register addr*/
	if(!i2c_write_interface(&read_cmd[0], cmd_len))
	{
		return false;
	}

	/*call the read callback function to get the register value*/		
	if(!i2c_read_interface(rx_buf, rx_length))
	{
		return false;
	}
	return true;
}



/***********************************************************************
[function]: 
                        callback:          burn the FW to ctpm.
[parameters]:
			    pbt_buf[in]:     point to Head+FW ;
			    dw_lenth[in]:   the length of the FW + 6(the Head length);    
[return]:
			    ERR_OK:          no error;
			    ERR_MODE:      fail to switch to UPDATE mode;
			    ERR_READID:   read id fail;
			    ERR_ERASE:     erase chip fail;
			    ERR_STATUS:   status error;
			    ERR_ECC:        ecc error.
************************************************************************/
E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(u8* pbt_buf, int dw_lenth)
{
    u8  cmd,reg_val[2] = {0};
	u8  buffer[2] = {0};
    u8  packet_buf[FTS_PACKET_LENGTH + 6];
    u8  auc_i2c_write_buf[10];
    u8  bt_ecc;
	
    int  j,temp,lenght,i_ret,packet_number, i = 0;
    int  i_is_new_protocol = 0;
	

    /******write 0xaa to register 0xfc******/
    cmd=0xaa;
    fts_register_write(0xfc,&cmd);
    mdelay(50);
	
     /******write 0x55 to register 0xfc******/
    cmd=0x55;
    fts_register_write(0xfc,&cmd);
    printk("[FT5306] Step 1: Reset CTPM test\n");
   
    mdelay(10);   


    /*******Step 2:Enter upgrade mode ****/
    printk("\n[FT5306] Step 2:enter new update mode\n");
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = fts_i2c_txdata(auc_i2c_write_buf, 2);
        mdelay(5);
    }while(i_ret <= 0 && i < 10 );

    if (i > 1)
    {
        i_is_new_protocol = 1;
    }

    /********Step 3:check READ-ID********/        
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
    {
        printk("[FT5306] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        return ERR_READID;
        //i_is_new_protocol = 1;
    }
    

     /*********Step 4:erase app**********/
    if (i_is_new_protocol)
    {
        cmd_write(0x61,0x00,0x00,0x00,1);
    }
    else
    {
        cmd_write(0x60,0x00,0x00,0x00,1);
    }
    mdelay(1500);
    printk("[FT5306] Step 4: erase. \n");



    /*Step 5:write firmware(FW) to ctpm flash*/
    bt_ecc = 0;
    printk("[FT5306] Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8)(temp>>8);
        packet_buf[3] = (u8)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8)(lenght>>8);
        packet_buf[5] = (u8)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }
        
        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        mdelay(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk("[FT5306] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8)(temp>>8);
        packet_buf[3] = (u8)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8)(temp>>8);
        packet_buf[5] = (u8)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);    
        mdelay(20);
    }

    /***********send the last six byte**********/
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (u8)(temp>>8);
        packet_buf[3] = (u8)temp;
        temp =1;
        packet_buf[4] = (u8)(temp>>8);
        packet_buf[5] = (u8)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i]; 
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);  
        mdelay(20);
    }

    /********send the opration head************/
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("[FT5306] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        return ERR_ECC;
    }

    /*******Step 7: reset the new FW**********/
    cmd_write(0x07,0x00,0x00,0x00,1);
	mdelay(100);//100ms	
	fts_register_read(0xfc, buffer, 1);	
	if (buffer[0] == 1)
	{
	cmd=4;
	fts_register_write(0xfc, &cmd);
	mdelay(2500);//2500ms	
	 do	
	 {	
	 fts_register_read(0xfc, buffer, 1);	
	 mdelay(100);//100ms	
	 }while (buffer[0] != 1); 		   	
	}
    return ERR_OK;
}


/***********************************************************************/

int fts_ctpm_fw_upgrade_with_i_file(void)
{
   u8*     pbt_buf = 0;
   int i_ret;
    
   pbt_buf = CTPM_FW;
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));
   
   return i_ret;
}

/***********************************************************************/

unsigned char fts_ctpm_get_upg_ver(void)
{
    unsigned int ui_sz;
	
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
        return 0xff; 
 
}
#endif

static int ft5306_read_regs(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret; 
	ret = i2c_master_reg8_recv(client, reg, buf, len, FT5306_I2C_SPEED);
	return ret; 
}


/* set the it7260 registe,used i2c bus*/
static int ft5306_set_regs(struct i2c_client *client, u8 reg, u8 const buf[], unsigned short len)
{
	int ret; 
	ret = i2c_master_reg8_send(client, reg, buf, (int)len, FT5306_I2C_SPEED);
	return ret;
}

static inline  int  ft5306_read_values()
{
	u8 buf[32];
	
	int err = 0;
	u8 i=0;
	uint coorlen = 0;
	uint readxy_data,width;
	ts->finger=0;
	u16 xpos[FT5306_MAX_SUPPORT_POINT];
	u16 ypos[FT5306_MAX_SUPPORT_POINT];
	err= ft5306_read_regs(ts->client,0x00, buf, 31);
	ts->finger= buf[2] & 0x07;

	if(ts->finger== 0){
//		pr_err("coorlen = 0, no finger touch!\n");
		goto no_touch;
	}
    if(ts->finger>0) {
		for(i=0;i<ts->finger;i++){
#if(REVERSE_X)
			xpos[i] = 1024-((s16)(buf[3+i*6] & 0x0F)<<8 | (s16)buf[4+i*6]);
#else
			xpos[i] =(s16)(buf[3+i*6] & 0x0F)<<8 | (s16)buf[4+i*6];
#endif

#if(REVERSE_Y)
			ypos[i]  = 768-((s16)(buf[5+i*6] & 0x0F)<<8 | (s16)buf[6+i*6]);
#else
			ypos[i]  = (s16)(buf[5+i*6] & 0x0F)<<8 | (s16)buf[6+i*6];
#endif
			input_mt_slot(ts->input, i);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(ts->input, ABS_MT_POSITION_X,  xpos[i]);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,  ypos[i]);				
			pr_info("===x[%d] = %d,y[%d] = %d \n",i,xpos[i],i,ypos[i]);
   	 	}
    }
	
	ts->reported_finger_count = ts->finger;
	input_sync(ts->input);
	ts->pendown = PEN_DOWN;
	ts->finger=0;
	
	return PEN_DOWN;
	
no_touch:
		ts->pendown = PEN_RELEASE;
		for(i=0; i<FT5306_MAX_SUPPORT_POINT; i++){
			input_mt_slot(ts->input, i);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
		}
		input_sync(ts->input);	
		ts->finger=0;	
}


static void ft5306_work(struct work_struct *work)
{
	struct ts_ft5306 *ts = container_of(to_delayed_work(work), struct ts_ft5306, work);
	 ft5306_read_values(ts);
	enable_irq(ts->irq);
}

static irqreturn_t ft5306_irq(int irq, void *handle)
{
	struct ts_ft5306 *ts = handle;
	
	disable_irq_nosync(ts->irq);
	queue_delayed_work(ts->wq, &ts->work, 0);
	return IRQ_HANDLED;
}
static void ft5306_free_irq(struct ts_ft5306 *ts)
{
	free_irq(ts->irq, ts);
	if (cancel_delayed_work_sync(&ts->work)) {
		enable_irq(ts->irq);
	}
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5306_early_suspend(struct early_suspend *h)
{
	disable_irq(ts->irq);

	if (ts->pdata->ft5306_platform_sleep){
		ts->pdata->ft5306_platform_sleep();
	}
	pr_info(" FT5406 Touchscreen suspend!\n ");
}

static void ft5306_early_resume(struct early_suspend *h)
{
	if (ts->pdata->ft5306_platform_wakeup){
		ts->pdata->ft5306_platform_wakeup();
	}

	enable_irq(ts->irq);
	pr_info(" FT5406 Touchscreen resume!\n ");
}

static struct early_suspend searly_suspend_info = {
	.suspend = ft5306_early_suspend,
	.resume = ft5306_early_resume,
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
};

#endif

#ifdef CONFIG_PM
static int ft5306_suspend(struct i2c_client *client, pm_message_t mesg)
{

	return 0;
}

static int ft5306_resume(struct i2c_client *client)
{
	return 0;
}
#endif

static int __devinit ft5306_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5306_platform_data *pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err, setup_try = 0;
	unsigned char reg_value;
	unsigned char reg_version;
	
	pr_info("%s\n",__FUNCTION__);

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
		goto err_exit;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
		err = -EIO;
		goto err_exit;
	}

	ts = kzalloc(sizeof(struct ts_ft5306), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_1;
	}

	ts->client = client;
	ts->irq = client->irq;
	ts->input = input_dev;
	ts->status =0 ;
	ts->pendown = 0;
	ts->pdata = pdata;

	ts->wq = create_workqueue("ft5306_wq");
	INIT_DELAYED_WORK(&ts->work, ft5306_work);
	
	ts->model = pdata->model;

	snprintf(ts->phys, sizeof(ts->phys), "%s/input3", dev_name(&client->dev));

	input_dev->name = "ft5306 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

#if defined (SINGLETOUCHE_MODE)
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(BTN_2, input_dev->keybit);
	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev,ABS_X,0,WISKY_TOUCH_WIDTH,0,0);
	input_set_abs_params(input_dev,ABS_Y,0,WISKY_TOUCH_HEIGHT,0,0);
#else
	ts->has_relative_report = 0;

	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	__set_bit(EV_ABS, input_dev->evbit);	
	input_mt_init_slots(input_dev, FT5306_MAX_SUPPORT_POINT);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);	
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, WISKY_TOUCH_WIDTH, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, WISKY_TOUCH_HEIGHT, 0, 0);
#endif

	if (pdata->init_platform_hw)
		pdata->init_platform_hw();
	
	err = input_register_device(input_dev);
	if (err)
		goto err_2;
		
	i2c_set_clientdata(client, ts);


#if defined(FT5306_FIRMWARE_UPGRADE)//write firmware 
	/***wait CTP to bootup normally***/
	msleep(200); 
	ft5306_read_regs(client, FT5306_REG_FIRMID, &reg_version, 1);
	printk("[FT5306] firmware version = 0x%2x\n", reg_version);
	  if (fts_ctpm_get_upg_ver() != reg_version)  
	  {
		  printk("[FT5306] start upgrade new verison 0x%2x\n", fts_ctpm_get_upg_ver());
		  msleep(200);
		  err =  fts_ctpm_fw_upgrade_with_i_file();
		  if (err == 0)
		  {
			  printk("[FT5306] ugrade successfuly.\n");
			  msleep(300);
			  fts_register_read(FT5306_REG_FIRMID, &reg_value,1);
			  printk("FTS_DBG from old version 0x%2x to new version = 0x%2x\n", reg_version, reg_value);
		  }
		  else
		  {
			  printk("[FT5306]  ugrade fail err=%d, line = %d.\n",err, __LINE__);
		  }
		  msleep(4000);
	  }
#endif

	if (!ts->irq) {
		dev_dbg(&ts->client->dev, "no IRQ?\n");
		err = -ENODEV;
		goto err_3;
	}else{
		ts->irq = gpio_to_irq(ts->irq);
	}

	err = request_irq(ts->irq, ft5306_irq, IRQF_TRIGGER_FALLING, 
			client->dev.driver->name, ts);
	
	if (err < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ts->irq);
		goto err_1;
	}
	
	if (err < 0)
		goto err_free_irq;
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&searly_suspend_info);
#endif

	printk("FT5306 I2C Touchscreen init done!\n ");
	return 0;

 err_free_irq:
 	ft5306_free_irq(ts);
err_3:
	input_unregister_device(input_dev);
err_2:
	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();
 err_1:
	input_free_device(input_dev);
	kfree(ts);
err_exit:	
	printk("FT5306 I2C Touchscreen init Failed!\n ");
	return err;
}

static int __devexit ft5306_remove(struct i2c_client *client)
{
	struct ts_ft5306 *ts = i2c_get_clientdata(client);
	struct ft5306_platform_data *pdata = client->dev.platform_data;

#ifdef CONFIG_HAS_EARLYSUSPEND
        unregister_early_suspend(&searly_suspend_info);
#endif

	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();
	ft5306_free_irq(ts);
	input_unregister_device(ts->input);
	kfree(ts);

	return 0;
}

static struct i2c_device_id ft5306_idtable[] = {
	{ "ft5306", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ft5306_idtable);

static struct i2c_driver ft5306_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ft5306"
	},
	.id_table	= ft5306_idtable,
	.probe		= ft5306_probe,
	.remove		= __devexit_p(ft5306_remove),
#ifdef CONFIG_PM
	.suspend = ft5306_suspend,
	.resume = ft5306_resume,
#endif
};

static int __init ft5306_init(void)
{
	pr_info("ft5306 init...");
	i2c_add_driver(&ft5306_driver);
	return 0;
}

static void __exit ft5306_exit(void)
{
	return i2c_del_driver(&ft5306_driver);
}

/*
 * FT5306 touchscreen must init power up early, or the other I2C devices will not work.
 */
fs_initcall(ft5306_init);
module_exit(ft5306_exit);
MODULE_LICENSE("GPL");


