/*
 * 
 * Copyright (C) 2011 Goodix, Inc.
 * 
 * Author: Scott
 * Date: 2012.01.05
 */
 

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/input/mt.h>
#include <mach/board.h>
#include <mach/gpio.h>


#include "wisky_ts_gt8xx.h"

#define GT82X_I2C_SPEED 	(400*1000)

#define READ_TOUCH_ADDR_H   0x0F
#define READ_TOUCH_ADDR_L   0x40
#define READ_KEY_ADDR_H     0x0F
#define READ_KEY_ADDR_L     0x41
#define READ_COOR_ADDR_H    0x0F
#define READ_COOR_ADDR_L    0x42
#define RESOLUTION_LOC      71
#define TRIGGER_LOC         66

#define GOODIX_I2C_NAME "Goodix-TS"

static struct workqueue_struct *goodix_wq;
static const char *goodix_ts_name = "Goodix TouchScreen GT8XX";
static int googix_ts_type;

static s32 goodix_ts_remove(struct i2c_client *);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

#ifdef CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client*);
extern void uninit_wr_node(void);
#endif

#ifdef AUTO_UPDATE_GUITAR
extern s32 init_update_proc(struct goodix_ts_data *);
#else
static void guitar_reset(s32);
#endif

//*************************Touchkey Surpport Part*****************************
//#define HAVE_TOUCH_KEY
#ifdef HAVE_TOUCH_KEY
    const u16 touch_key_array[]={
                                      KEY_MENU,             //MENU
                                      KEY_BACK,             //BACK
                                      KEY_HOME,             //HOME
                                      KEY_SEARCH,           //SEARCH
                                     }; 
    #define MAX_KEY_NUM     (sizeof(touch_key_array)/sizeof(touch_key_array[0]))
#endif

struct goodix_i2c_rmi_platform_data {
    uint32_t version;    /* Use this entry for panels with */
    //reservation
};

#if 1
#define TOUCH_MAX_HEIGHT     WISKY_TOUCH_WIDTH
#define TOUCH_MAX_WIDTH      WISKY_TOUCH_HEIGHT
#else
#define AUTO_SET
u16 TOUCH_MAX_HEIGHT;
u16 TOUCH_MAX_WIDTH;
#endif

#if 0
//touch power up setup data
static u8 config_info[]=
    {
        0x0F,0x80,/*config address*/
/*F80*/		0x00,0x0F,0x01,0x10,0x02,0x11,0x03,0x12,
		0x04,0x13,0x05,0x14,0x06,0x15,0x07,0x16,
/*F90*/		0x08,0x17,0x09,0x18,0x0A,0x19,0x0B,0x1A,
		0x0C,0x1B,0x0D,0x1C,0x0E,0x1D,0x13,0x09,
/*FA0*/		0x12,0x08,0x11,0x07,0x10,0x06,0x0F,0x05,
		0x0E,0x04,0x0D,0x03,0x0C,0x02,0x0B,0x01,
/*FB0*/		0x0A,0x00,0x09,0x03,0x88,0x88,0x88,0x25,
		0x00,0x00,0x08,0x00,0x00,0x0E,0x48,0x34,
/*FC0*/		0x0C,0x03,0x00,0x05,0x00,0x03,0x00,0x04,
		0x00,0x3C,0x43,0x3E,0x46,0x00,0x00,0x05,
/*FD0*/		0x19,0x05,0x14,0x10,0x00,0x02,0x00,0x00,
		0x00,0x00,0x00,0x00,0x38,0x28,0x30,0x20,
/*FE0*/		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
    };
#else
//touch power up setup data
static u8 *config_info;
static int config_size = 0;

static u8 config_info_default[]=
{
	0x0F,0x80,/*config address*/
	#include GT8XX_CFG_DEFAULT
};
static u8 config_info_QiuTian[]=
{
	0x0F,0x80,/*config address*/
	#include GT8XX_CFG_QiuTian
};
static u8 config_info_PingBo[]=
{
	0x0F,0x80,/*config address*/
	#include GT8XX_CFG_PingBo
};
static u8 config_info_RuiShi[]=
{
	0x0F,0x80,/*config address*/
	#include GT8XX_CFG_RuiShi
};
static u8 config_info_NULL[]=
{
	0x0F,0x80,/*config address*/
	#include GT8XX_CFG_DEFAULT
};
#endif

/*******************************************************	
���ܣ�	
	��ȡ�ӻ�����
	ÿ��������������i2c_msg��ɣ���1����Ϣ���ڷ��ʹӻ���ַ��
	��2�����ڷ��Ͷ�ȡ��ַ��ȡ�����ݣ�ÿ����Ϣǰ������ʼ�ź�
������
	client:	i2c�豸�������豸��ַ
	buf[0]~buf[1]��	 ���ֽ�Ϊ��ȡ��ַ
	buf[2]~buf[len]�����ݻ�����
	len��	��ȡ���ݳ���
return��
	ִ����Ϣ��
*********************************************************/
/*Function as i2c_master_send */
static s32 i2c_read_bytes(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;

    //����д��ַ
    msgs[0].flags=!I2C_M_RD; //д��Ϣ
    msgs[0].addr=client->addr;
	msgs[0].scl_rate = GT82X_I2C_SPEED;
    msgs[0].len=2;
    msgs[0].buf=&buf[0];
    //��������
    msgs[1].flags=I2C_M_RD;//����Ϣ
    msgs[1].addr=client->addr;
	msgs[1].scl_rate = GT82X_I2C_SPEED;
    msgs[1].len=len - ADDR_LENGTH;
    msgs[1].buf=&buf[2];

    ret=i2c_transfer(client->adapter,msgs, 2);

    return ret;
}

/*******************************************************	
���ܣ�
	��ӻ�д����
������
	client:	i2c�豸�������豸��ַ
	buf[0]~buf[1]��	 ���ֽ�Ϊд��ַ
	buf[2]~buf[len]�����ݻ�����
	len��	���ݳ���	
return��
	ִ����Ϣ��
*******************************************************/
/*Function as i2c_master_send */
static s32 i2c_write_bytes(struct i2c_client *client,u8 *data,s32 len)
{
    struct i2c_msg msg;
    s32 ret=-1;

    //�����豸��ַ
    msg.flags=!I2C_M_RD;//д��Ϣ
    msg.addr=client->addr;
	msg.scl_rate = GT82X_I2C_SPEED;
    msg.len=len;
    msg.buf=data;        

    ret=i2c_transfer(client->adapter,&msg, 1);
	
    return ret;
}

/*******************************************************
���ܣ�
	����ǰ׺����
	
	ts:	client˽�����ݽṹ��
return��
    �ɹ�����1
*******************************************************/
static s32 i2c_pre_cmd(struct goodix_ts_data *ts)
{
    s32 ret;
    u8 pre_cmd_data[2]={0x0f, 0xff};

    ret=i2c_write_bytes(ts->client,pre_cmd_data,2);
    return ret;//*/
}

/*******************************************************
���ܣ�
	���ͺ�׺����
	
	ts:	client˽�����ݽṹ��
return��
    �ɹ�����1
*******************************************************/
static s32 i2c_end_cmd(struct goodix_ts_data *ts)
{
    s32 ret;
    u8 end_cmd_data[2]={0x80, 0x00};    

    ret=i2c_write_bytes(ts->client,end_cmd_data,2);
    return ret;//*/
}

/*******************************************************
���ܣ�
	Guitar��ʼ�����������ڷ���������Ϣ����ȡ�汾��Ϣ
������
	ts:	client˽�����ݽṹ��
return��
	ִ�н���룬0��ʾ����ִ��
*******************************************************/
s32 goodix_init_panel(struct goodix_ts_data *ts, u8 send)
{
    s32 ret = -1;

#if 0//def AUTO_SET
    TOUCH_MAX_WIDTH  = ((config_info[RESOLUTION_LOC] << 8)|config_info[RESOLUTION_LOC + 1]);
    TOUCH_MAX_HEIGHT = ((config_info[RESOLUTION_LOC + 2] << 8)|config_info[RESOLUTION_LOC + 3]);

    DEBUG_MSG("TOUCH_MAX_WIDTH  : %d\n", (s32)TOUCH_MAX_WIDTH);
    DEBUG_MSG("TOUCH_MAX_HEIGHT : %d\n", (s32)TOUCH_MAX_HEIGHT);
#else
    config_info[RESOLUTION_LOC]     = TOUCH_MAX_WIDTH >> 8;
    config_info[RESOLUTION_LOC + 1] = TOUCH_MAX_WIDTH & 0xff;
    config_info[RESOLUTION_LOC + 2] = TOUCH_MAX_HEIGHT >> 8;
    config_info[RESOLUTION_LOC + 3] = TOUCH_MAX_HEIGHT & 0xff;
#endif

    if (INT_TRIGGER == GT_IRQ_FALLING)
    {
        config_info[TRIGGER_LOC] &= 0xf7; 
    }
    else if (INT_TRIGGER == GT_IRQ_RISING)
    {
        config_info[TRIGGER_LOC] |= 0x08;
    }

    if (send)
    {
        ret=i2c_write_bytes(ts->client, config_info, config_size);
        if (ret <= 0)
        {
            return fail;
        }
        i2c_end_cmd(ts);
        msleep(10);
    }
    return success;
}

static s32 touch_num(u8 value, s32 max)
{
    s32 tmp = 0;

    while((tmp < max) && value)
    {
        if ((value & 0x01) == 1)
        {
            tmp++;
        }
        value = value >> 1;
    }

    return tmp;
}

/*******************************************************	
���ܣ�
	��������������
	���жϴ���������1���������ݣ�У����ٷ������
������
	ts:	client˽�����ݽṹ��
return��
    void
********************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
    u8 finger = 0;
    u8 chk_sum = 0;
    u8 key = 0;
    static u8 last_key = 0;
    u16 X_value;
    u16 Y_value;
    u32 count = 0;
    u32 position = 0;
    s32 ret = -1;
    s32 tmp = 0;
    s32 i;
    u8 *coor_point;
    u8 touch_data[2 + 2 + 5*MAX_FINGER_NUM + 1] = {READ_TOUCH_ADDR_H,READ_TOUCH_ADDR_L,0, 0};
    static u8 finger_last[MAX_FINGER_NUM+1]={0};        //�ϴδ�����������ָ����
    u8 finger_current[MAX_FINGER_NUM+1] = {0};        //��ǰ������������ָ����

    struct goodix_ts_data *ts = container_of(work, struct goodix_ts_data, work);
    
#ifndef INT_PORT
COORDINATE_POLL:
#endif
    if( tmp > 9)
    {
        dev_info(&(ts->client->dev), "Because of transfer error,touchscreen stop working.\n");
        goto XFER_ERROR ;
    }

    //���齫����һ���Զ�ȡ��
    ret=i2c_read_bytes(ts->client, touch_data,sizeof(touch_data)/sizeof(touch_data[0])); 
    i2c_end_cmd(ts);
    if(ret <= 0) 
    {
        dev_err(&(ts->client->dev),"I2C transfer error. Number:%d\n ", ret);
        ts->bad_data = 1;
        tmp ++;
#ifndef INT_PORT
        goto COORDINATE_POLL;
#else
        goto XFER_ERROR;
#endif
    }

    if(ts->bad_data)
    {
        //TODO:Is sending config once again (to reset the chip) useful?    
        ts->bad_data = 0;
        msleep(20);
    }

    if((touch_data[2]&0xC0)!=0x80)
    {
        goto DATA_NO_READY;        
    }

    key = touch_data[3]&0x0f; // 1, 2, 4, 8
    if (key == 0x0f)
    {
        if (fail == goodix_init_panel(ts, 1))
        {
/**/        DEBUG_COOR("Reload config failed!\n");
        }
        else
        {
            DEBUG_COOR("Reload config successfully!\n");
        }
        goto XFER_ERROR;
    }

    finger = (u8)touch_num(touch_data[2]&0x1f, MAX_FINGER_NUM);

/**/DEBUG_COOR("touch num:%x\n", finger);

    for (i = 1;i < MAX_FINGER_NUM + 1; i++)        
    {
        finger_current[i] = !!(touch_data[2] & (0x01<<(i-1)));
    }

#ifdef DEBUG_COORD
/**/for (i = 0; i < finger*5+4; i++)
/**/{  
/**/    DEBUG_COOR("%5x", touch_data[i]);
/**/}
/**/DEBUG_COOR("\n");
#endif 

    //����У���    
    coor_point = &touch_data[4];
    chk_sum = 0;
    for ( i = 0; i < 5*finger; i++)
    {
        chk_sum += coor_point[i];
/**/    DEBUG_COOR("%5x", coor_point[i]);
    }
/**/DEBUG_COOR("\ncheck sum:%x\n", chk_sum);
/**/DEBUG_COOR("check sum byte:%x\n", coor_point[5*finger]);
    if (chk_sum != coor_point[5*finger])
    {
        goto XFER_ERROR;
    }

    //��������//
    if (finger)
    {
        for(i = 0, position=1;position < MAX_FINGER_NUM+1; position++)
        {
            if(finger_current[position])
            {     
                X_value = coor_point[i] << 8;
                X_value = X_value | coor_point[i + 1];

                Y_value = coor_point[i + 2] << 8;
                Y_value = Y_value | coor_point[i + 3];
              	
				input_mt_slot(ts->input_dev, position - 1);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 1);
#if 0
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X,  X_value);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,  Y_value);
    /**/        DEBUG_COOR("X:%d\n", (s32)X_value);
    /**/        DEBUG_COOR("Y:%d\n", (s32)Y_value);
#else			
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X,  Y_value);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,  X_value);
    /**/        DEBUG_COOR("X:%d\n", (s32)Y_value);
    /**/        DEBUG_COOR("Y:%d\n", (s32)X_value);
#endif

                i += 5;
            }
        }
    }
    else
    {
		for(i=0; i<MAX_FINGER_NUM; i++){
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}
    }

#ifdef HAVE_TOUCH_KEY
#ifdef DEBUG_COORD
/**/for (i = 0; i < 4; i++)
/**/{
/**/    DEBUG_COOR("key:%4x   ", !!(key&(0x01<<i)));
/**/}
/**/DEBUG_COOR("\n");
#endif

    if((last_key != 0) || (key != 0))
    {
        for(count = 0; count < MAX_KEY_NUM; count++)
        {
            input_report_key(ts->input_dev, touch_key_array[count], !!(key&(0x01<<count)));    
        }
    }
    last_key = key;
#endif

    input_sync(ts->input_dev);
    
    for(position=1;position<MAX_FINGER_NUM+1; position++)
    {
        finger_last[position] = finger_current[position];
    }

DATA_NO_READY:
XFER_ERROR:
    if(ts->use_irq && ts->irq_is_disable == 1)
    {
        ts->irq_is_disable = 0;
        enable_irq(ts->irq);
    }
}

/*******************************************************	
���ܣ�
	��ʱ����Ӧ����
	�ɼ�ʱ�����������ȴ����������������У�֮�����¼�ʱ
������
	timer�����������ļ�ʱ��	
return��
	��ʱ������ģʽ��HRTIMER_NORESTART��ʾ����Ҫ�Զ�����
********************************************************/
#if 0
static enum hrtimer_restart goodix_ts_timer_func(struct hrtimer *timer)
{
    struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);

    queue_work(goodix_wq, &ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, (POLL_TIME+6)*1000000), HRTIMER_MODE_REL);

    return HRTIMER_NORESTART;
}
#endif

/*******************************************************	
���ܣ�
	�ж���Ӧ����
	���жϴ��������ȴ���������������
������
return��
    IRQ_HANDLED:interrupt was handled by this device
********************************************************/
static irqreturn_t goodix_ts_irq_handler(s32 irq, void *dev_id)
{
    struct goodix_ts_data *ts = (struct goodix_ts_data*)dev_id;

    if (ts->use_irq && (!ts->irq_is_disable))
    {
        disable_irq_nosync(ts->irq);
        ts->irq_is_disable = 1;
    }
    
    queue_work(goodix_wq, &ts->work);

    return IRQ_HANDLED;
}

/*******************************************************	
���ܣ�
	�����Դ������IC����˯�߻��份��
������
	on:	0��ʾʹ��˯�ߣ�1Ϊ����
return��
	�Ƿ����óɹ���successΪ�ɹ�
	�����룺-1Ϊi2c����-2ΪGPIO����-EINVALΪ����on����
********************************************************/
//#if defined(INT_PORT)
static s32 goodix_ts_power(struct goodix_ts_data * ts, s32 on)
{
    s32 ret = -1;

    u8 i2c_control_buf[3] = {0x0f,0xf2,0xc0};        //suspend cmd

    if(ts == NULL || !ts->use_irq)
        return -2;

    switch(on)
    {
    case 0:
        ret = i2c_write_bytes(ts->client, i2c_control_buf, 3);
        return ret;

    case 1:
		gpio_request(INT_PORT, NULL);
        gpio_direction_output(INT_PORT, 0);
        gpio_set_value(INT_PORT, 0);
        msleep(5);
        gpio_set_value(INT_PORT, 1);
        msleep(5);
        gpio_direction_input(INT_PORT);
        gpio_pull_updown(INT_PORT, 0);
		gpio_free(INT_PORT);
        msleep(10);
        return success;

    default:
        DEBUG_MSG(KERN_DEBUG "%s: Cant't support this command.", goodix_ts_name);
        return -EINVAL;
    }

}

static s32 init_input_dev(struct goodix_ts_data *ts)
{
    s32 i;
    s32 ret = 0;

    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL)
    {
        dev_dbg(&ts->client->dev,"goodix_ts_probe: Failed to allocate input device\n");
        return fail;
    }

#ifdef HAVE_TOUCH_KEY
    for(i = 0; i < MAX_KEY_NUM; i++)
    {
        input_set_capability(ts->input_dev, EV_KEY, touch_key_array[i]);
    }
#endif

    goodix_init_panel(ts, 0);

	ts->input_dev->name = "GT82X Touchscreen";
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
#ifdef GOODIX_MULTI_TOUCH
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	__set_bit(EV_ABS, ts->input_dev->evbit);	
	input_mt_init_slots(ts->input_dev, MAX_FINGER_NUM);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);	
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, TOUCH_MAX_HEIGHT, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TOUCH_MAX_WIDTH, 0, 0);
#else
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	input_set_abs_params(ts->input_dev,ABS_X,0,TOUCH_MAX_HEIGHT,0,0);
	input_set_abs_params(ts->input_dev,ABS_Y,0,TOUCH_MAX_WIDTH,0,0);
#endif    

    memcpy(ts->phys, "input/ts", 8);
    ts->input_dev->phys = ts->phys;
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor = 0xDEAD;
    ts->input_dev->id.product = 0xBEEF;
    ts->input_dev->id.version = 10427;    //screen firmware version

	switch(googix_ts_type){
		case TS_TYPE_QIUTIAN:
			ts->input_dev->name = "GT8XX_QiuTian";
			break;
		case TS_TYPE_PINGBO:
			ts->input_dev->name = "GT8XX_PingBo";
			break;
		case TS_TYPE_RUISHI:
			ts->input_dev->name = "GT8XX_RuiShi";
			break;
		default:
			ts->input_dev->name = "GT8XX_Default";
			break;
	}
	
    ret = input_register_device(ts->input_dev);
    if (ret) 
    {
        dev_err(&ts->client->dev,"Probe: Unable to register %s input device\n", ts->input_dev->name);
        input_free_device(ts->input_dev);
        return fail;
    }
    DEBUG_MSG("Register input device successfully!\n");

    return success;
}

#if 0
static s32 set_pins(struct goodix_ts_data *ts)
{
    s32 ret = -1;
    
    ts->client->irq=TS_INT;        //If not defined in client
    if (ts->client->irq)
    {
        ret = GPIO_REQUEST(INT_PORT, "TS_INT");    //Request IO
        if (ret < 0) 
        {
            dev_err(&ts->client->dev, "Failed to request GPIO:%d, ERRNO:%d\n",(s32)INT_PORT,ret);
            goto err_gpio_request_failed;
        }
        DEBUG_MSG("Request int port successfully!\n");
        
        gpio_direction_input(INT_PORT);
        gpio_pull_updown(INT_PORT, 0);
        GPIO_CFG_PIN(INT_PORT, INT_CFG);        //Set IO port function    

        ret = request_irq(ts->client->irq, goodix_ts_irq_handler, INT_TRIGGER,
                          ts->client->name, ts);
        if (ret != 0) 
        {
            dev_err(&ts->client->dev,"Cannot allocate ts INT!ERRNO:%d\n", ret);
            gpio_direction_input(INT_PORT);
            gpio_free(INT_PORT);
            goto err_gpio_request_failed;
        }
        else 
        {
            disable_irq(ts->client->irq);
            ts->use_irq = 1;
            ts->irq_is_disable = 1;
            dev_dbg(&ts->client->dev, "Reques EIRQ %d successed on GPIO:%d\n", TS_INT, INT_PORT);
        }
    }

err_gpio_request_failed:
    if (!ts->use_irq) 
    {
        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = goodix_ts_timer_func;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        DEBUG_MSG("Use timer!\n");
    }

    ret = GPIO_REQUEST(RESET_PORT, "TS_RESET");    //Request IO
    if (ret < 0) 
    {
        dev_err(&ts->client->dev, "Failed to request GPIO:%d, ERRNO:%d\n",(s32)RESET_PORT,ret);
    }
    else
    {
        ts->use_reset = 1;
        gpio_direction_input(RESET_PORT);
        gpio_pull_updown(RESET_PORT, 0);
    }

    dev_info(&ts->client->dev,"Start %s in %s mode\n", 
              ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

    return ts->use_irq;
}
#endif

#define READ_VERSION
#ifdef READ_VERSION
static s32 goodix_ts_version(struct goodix_ts_data *ts)
{
    u8 buf[8];

    buf[0] = 0x0f;
    buf[1] = 0x7d;
    
    i2c_read_bytes(ts->client, buf, 5);
    i2c_end_cmd(ts);

    NOTICE("PID:%02x, VID:%02x%02x\n", buf[2], buf[3], buf[4]);

    return success;
}
#endif

/*******************************************************	
���ܣ�
	������̽�⺯��
	��ע������ʱ����(Ҫ����ڶ�Ӧ��client)��
	����IO,�жϵ���Դ���룻�豸ע�᣻��������ʼ���ȹ���
������
	client�����������豸�ṹ��
	id���豸ID
return��
	ִ�н���룬0��ʾ����ִ��
********************************************************/
static s32 goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 ret = 0;
    s32 retry=0;
    struct goodix_ts_data *ts = NULL;
    struct gt82x_platform_data *pdata = NULL;
    
    DEBUG("Start to install Goodix Capacitive TouchScreen driver.\n");
    DEBUG("*DRIVER INFORMATION\n");
    DEBUG("**RELEASE DATE:%s.\n", RELEASE_DATE);
    DEBUG("**COMPILE TIME:%s, %s.\n", __DATE__, __TIME__);

    //Check I2C function
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
    {
        dev_err(&client->dev, "Must have I2C_FUNC_I2C.\n");
        ret = -ENODEV;
	goto error_0;
    }

    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL)
    {
        ret = -ENOMEM;
	goto error_0;
    }

	googix_ts_type = ts_check_type();
	pr_info("%s[%d]: touchscreen type check as googix_ts_type=%d\n", __FUNCTION__, __LINE__, googix_ts_type);
	switch(googix_ts_type){
		case TS_TYPE_QIUTIAN:
			config_info = config_info_QiuTian;
			config_size  = sizeof(config_info_QiuTian)/sizeof(config_info_QiuTian[0]);
			break;
		case TS_TYPE_PINGBO:
			config_info = config_info_PingBo;
			config_size  = sizeof(config_info_PingBo)/sizeof(config_info_PingBo[0]);
			break;
		case TS_TYPE_RUISHI:
			config_info = config_info_RuiShi;
			config_size  = sizeof(config_info_RuiShi)/sizeof(config_info_RuiShi[0]);
			break;
		default:
			pr_info("[GT8xx]No correct type check, use default firmware (GT8XX_CFG_DEFAULT)\n");
			config_info = config_info_default;
			config_size  = sizeof(config_info_default)/sizeof(config_info_default[0]);
			break;
	}
	
    INIT_WORK(&ts->work, goodix_ts_work_func);        //init work_struct
    ts->client = client;
    ts->power = goodix_ts_power;
    ts->bad_data = 0;
    ts->use_irq = 0;
    ts->use_reset = 0;
    ts->irq_is_disable = 0;
	ts->irq = ts->client->irq;
    i2c_set_clientdata(client, ts);
    pdata = client->dev.platform_data;

    if (fail == init_input_dev(ts))
    {
        ret = -EFAULT;
	goto error_1;
    }
	
#if 0
	set_pins(ts);
#else
	if (pdata->init_platform_hw){
		pdata->init_platform_hw();
	}
	
	if (!ts->irq) {
		dev_dbg(&ts->client->dev, "no IRQ?\n");
		ret = -ENODEV;
		goto error_2;
	}else{
		ts->irq = gpio_to_irq(ts->irq);
		ts->use_irq = 1;
	}

	ret = request_irq(ts->irq, goodix_ts_irq_handler, IRQF_TRIGGER_RISING, 
			client->dev.driver->name, ts);
	
	if (ret < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ts->irq);
		goto error_3;
	}else{
	        disable_irq(ts->client->irq);
	        ts->use_irq = 1;
	        ts->irq_is_disable = 1;
	}
#endif
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#ifdef CREATE_WR_NODE
	init_wr_node(client);
#endif

#ifdef AUTO_UPDATE_GUITAR
	if (0xff == init_update_proc(ts))
	{
		DEBUG_MSG("Need update!\n");
		ret = -EFAULT;
		goto error_4;
	}
#else
//    msleep(5);
//    guitar_reset(10);
#endif

    //Test I2C connection.    
#if 0
    DEBUG_MSG("Testing I2C connection...\n");
    for(retry = 0;retry < 3; retry++)
    while(1)            //For debug use!
    {
        ret = i2c_pre_cmd(ts);
        if (ret > 0)
            break;
        msleep(20);
    }
    if(ret <= 0)
    {
        dev_err(&client->dev, "Warnning: I2C communication might be ERROR!\n");
        DEBUG_MSG("I2C test failed. I2C addr:%x\n", client->addr);
        goodix_ts_remove(ts->client);
	ret = -FAILED;
	goto error_1;
    }
#endif

	//Send config
	for (retry = 0; retry < 3; retry++)
	{
		if (success == goodix_init_panel(ts, 1))
		{
			DEBUG_MSG("Initialize successfully!\n");
			break;
		}
	}
	if (retry >= 3)
	{
		ts->bad_data=1;
		DEBUG_MSG("Initialize failed!\n");
		ret = -EFAULT;
		goto error_4;
	}

	//Enable interrupt
	if(ts->use_irq && ts->irq_is_disable == 1)
	{
		ts->irq_is_disable = 0;
		enable_irq(ts->irq);
	}

#ifdef READ_VERSION
	goodix_ts_version(ts);
#endif

	pr_info("%s[%d]: GT828 touchscreen driver probe ok.\n", __FUNCTION__, __LINE__);
	return 0;

error_4:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

#ifdef CREATE_WR_NODE
	uninit_wr_node();
#endif

error_3:
	if (ts && ts->use_irq) {
		gpio_free(INT_PORT);
	}else if(ts){
		hrtimer_cancel(&ts->timer);
	}
error_2:
	if (pdata->exit_platform_hw){
		pdata->exit_platform_hw();
	}
error_1:
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	kfree(ts);
error_0:
	pr_info("%s[%d]: GT828 touchscreen driver probe failed!\n", __FUNCTION__, __LINE__);
	return ret;
}


/*******************************************************	
���ܣ�
	������Դ�ͷ�
������
	client���豸�ṹ��
return��
	ִ�н���룬success��ʾ����ִ��
********************************************************/
static s32 goodix_ts_remove(struct i2c_client *client)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(client);

    dev_notice(&client->dev,"The driver is removing...\n");
    
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ts->early_suspend);
#endif

#ifdef CREATE_WR_NODE
    uninit_wr_node();
#endif

    if (ts && ts->use_irq) 
    {
        free_irq(ts->irq, ts);
        //gpio_direction_input(INT_PORT);
        gpio_free(INT_PORT);
    }
    else if(ts)
        hrtimer_cancel(&ts->timer);

    if (ts && ts->use_reset)
    {
        //gpio_direction_input(RESET_PORT);
        gpio_free(RESET_PORT);
    }
    
    i2c_set_clientdata(client, NULL);
    input_unregister_device(ts->input_dev);
    input_free_device(ts->input_dev);
    kfree(ts);
    return success;
}

//ͣ���豸
static s32 goodix_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
    s32 ret;
    struct goodix_ts_data *ts = i2c_get_clientdata(client);

    if (ts->irq_is_disable == 2)
    {
        return 0;
    }

    if (ts->use_irq)
    {
        if (!ts->irq_is_disable)
        {
            disable_irq(ts->irq);
            ts->irq_is_disable = 1;
        }
    }
    else
    {
        hrtimer_cancel(&ts->timer);
    }
    
    if (ts->power) 
    {
        ret = ts->power(ts, 0);
        if (ret <= 0)
            DEBUG_MSG(KERN_ERR "goodix_ts_resume power off failed\n");
    }
    return 0;
}

static s32 goodix_ts_resume(struct i2c_client *client)
{
    s32 ret;
    struct goodix_ts_data *ts = i2c_get_clientdata(client);

    if (ts->irq_is_disable == 2)
    {
        return 0;
    }

    if (ts->power) 
    {
        ret = ts->power(ts, 1);
        if (ret <= 0)
            DEBUG_MSG(KERN_ERR "goodix_ts_resume power on failed\n");
    }

    if (ts->use_irq)
    {
        ts->irq_is_disable = 0;
        enable_irq(ts->irq);
    }
    else
    {
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }

    return success;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h)
{
    struct goodix_ts_data *ts;
    ts = container_of(h, struct goodix_ts_data, early_suspend);
    goodix_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void goodix_ts_late_resume(struct early_suspend *h)
{
    struct goodix_ts_data *ts;
    ts = container_of(h, struct goodix_ts_data, early_suspend);
    goodix_ts_resume(ts->client);
}
#endif
//******************************Begin of firmware update surpport*******************************

#ifndef AUTO_UPDATE_GUITAR
static void guitar_reset(s32 ms)
{
    gpio_direction_output(RESET_PORT, 0);
    gpio_set_value(RESET_PORT, GPIO_LOW);
    msleep(ms);

    gpio_direction_input(RESET_PORT);
    gpio_pull_updown(RESET_PORT, 0);

    msleep(20);
    return;
}
#endif


//�����ڸ������� �豸�����豸ID �б�
//only one client
static const struct i2c_device_id goodix_ts_id[] = {
    { GOODIX_I2C_NAME, 0 },
    { }
};

//�豸�����ṹ��
static struct i2c_driver goodix_ts_driver = {
    .probe      = goodix_ts_probe,
    .remove     = goodix_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = goodix_ts_suspend,
    .resume     = goodix_ts_resume,
#endif
    .id_table   = goodix_ts_id,
    .driver     = {
        .name   = GOODIX_I2C_NAME,
        .owner  = THIS_MODULE,
    },
};

/*******************************************************	
���ܣ�
	�������غ���
return��
	ִ�н���룬0��ʾ����ִ��
********************************************************/
static s32 __devinit goodix_ts_init(void)
{
    goodix_wq = create_workqueue("goodix_wq");        //create a work queue and worker thread
    if (!goodix_wq)
    {
        DEBUG_MSG(KERN_ALERT "creat workqueue faiked\n");
        return -ENOMEM;
    }
    return i2c_add_driver(&goodix_ts_driver);
}

/*******************************************************	
���ܣ�
	����ж�غ���
������
	client���豸�ṹ��
********************************************************/
static void __exit goodix_ts_exit(void)
{
    DEBUG_MSG(KERN_ALERT "Touchscreen driver of guitar exited.\n");
    i2c_del_driver(&goodix_ts_driver);
    if (goodix_wq)
        destroy_workqueue(goodix_wq);        //release our work queue
}

late_initcall(goodix_ts_init);                //����ʼ������
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("Goodix Touchscreen Driver");
MODULE_LICENSE("GPL");

