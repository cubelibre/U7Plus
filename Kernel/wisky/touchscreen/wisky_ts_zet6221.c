/* 
** this file is touchscreen model, you can modify this file for specieal use
**
** author: hean
** email: hean86@gmail.com
*/

#include <linux/irq.h>
#include <asm/mach/irq.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <linux/earlysuspend.h>
#include <linux/input/mt.h>
#include <linux/hrtimer.h>  
/*usually to modify the code*/
 
//#define WISKY_TS_DEBUG 1 //print the debug messages or not
#define SUPPORT_CHARGE_MODE 1
/* related touchscreen chip */
#define I2C_NAME_ZET6221 "zet6221_ts"
#define ts_name_zet6221 "zet6221_touchscreen"
#define ZET6221_UPGRADE_FW	 1
#define READ_ZET6221_TPINFO 1
#define TS_MODLE_MULTI_POINT_TOUCH
  
#ifdef TS_MODLE_MULTI_POINT_TOUCH
#define MAX_SUPPORT_POINTS 5
#else
#define MAX_SUPPORT_POINTS 1
#endif
#define ZET_KEY_NUM 0

#define READ_DATA_BYTES (4 * MAX_SUPPORT_POINTS + 3) //need to modify as your ic
//#define NEED_REG_ADDR_TO_READ 1
#ifdef NEED_REG_ADDR_TO_READ
#define REG_ADDR_LENGTH     2
#define READ_TOUCH_ADDR_H   0x07
#define READ_TOUCH_ADDR_L   0x21
const char ts_read_addr[REG_ADDR_LENGTH] = {READ_TOUCH_ADDR_H, READ_TOUCH_ADDR_L};
const char *ts_read_reg = (const char *)&ts_read_addr; //if not need, must be NULL
#else
#define REG_ADDR_LENGTH     0
const char  *ts_read_reg = NULL;
#endif
#define TS_I2C_SPEED 	(200*1000)//80*1000
#define TS_IRQ_MODE_ZET6221 GPIOLevelLow//wisky-lxh@20120908,解决触摸屏反应慢问题 
/* related touchscreen chip end */
/*usually to modify the code end*/

/*macro define */
#define TOUCHSCREEN_MAX_HEIGHT	(WISKY_TOUCH_HEIGHT)
#define TOUCHSCREEN_MAX_WIDTH		(WISKY_TOUCH_WIDTH)

#define false 0
#define true 1
    
#ifdef WISKY_TS_DEBUG
    #define Ts_debug(fmt, arg...) printk(KERN_INFO "<ts--debug> " fmt, ##arg)
#else
    #define Ts_debug(fmt, arg...)
#endif

#ifndef MAX_SUPPORT_POINTS
    #define MAX_SUPPORT_POINTS 1
#endif

#ifdef SUPPORT_CHARGE_MODE
extern int external_power_status;
extern int dc_adaptor_status;
int last_power_status = 0;//add for charge mode
#define POWER_STATUS_CHARGING 1
#define POWER_STATUS_NORMAL 0
#define MicroTimeTInterupt	(25000000)
#endif
#define SIZE_ARRAY(arr) (sizeof(arr) / sizeof(arr[0]))

/*macro define end*/

static int zet_ts_type;

/* struct define */

/* about update start...*/
static u16 ResolutionX = TOUCHSCREEN_MAX_WIDTH;
static u16 ResolutionY = TOUCHSCREEN_MAX_HEIGHT;
static u16 FingerNum = 0;
static u16 KeyNum = 0;
static u8 inChargerMode = 0;
static int bufLength = 0;	
static u8 pc[8];
#if (defined(ZET6221_UPGRADE_FW) || defined(READ_ZET6221_TPINFO))
static unsigned int FW_ZET6221_LEN = 0;
static unsigned char *fw_zet6221_config_p = NULL;

static unsigned char CTPM_FW_ZET6221_DEFAULT[] __initdata = 
{
	#include ZET6221_FIRMWARE_DEFAULT
};

static unsigned char CTPM_FW_ZET6221_QIUTIAN[] __initdata = 
{
    #include ZET6221_FIRMWARE_QiuTian
};

static unsigned char CTPM_FW_ZET6221_PINGBO[] __initdata = 
{
    #include ZET6221_FIRMWARE_PingBo
};

static unsigned char CTPM_FW_ZET6221_RUISHI[] __initdata = 
{
    #include ZET6221_FIRMWARE_RuiShi
};

static unsigned char CTPM_FW_ZET6221_HAOEN[] __initdata = 
{
    #include ZET6221_FIRMWARE_HaoEn
};

#include "zet6221_downloader.c"
#endif
/* about update end */

struct data_zet6221_ts {
	char phys[32];		
	unsigned char use_irq;
	struct i2c_client *client;
	struct input_dev *input_dev;
	unsigned int irq; //if used irq, it is interrupt no
	struct delayed_work  work;
    struct workqueue_struct *wq;
	#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
	#endif
	#ifdef SUPPORT_CHARGE_MODE
	struct hrtimer timer;
	struct work_struct work_charge;
	struct workqueue_struct *wq_charge;
	#endif
};

enum finger_state {
    FINGER_UP = 0,
    FINGER_DOWN,
    FINGER_INVALID,
    FINGER_STATE_MAX,
};

struct point_data {
    unsigned char id;       //point id
    enum finger_state state; // point state
    int point_x;   // x coordinate value
    int point_y;   // y coordinate value
    int point_p;   //pressure value
};

/* struct define end */

/*
** 本函数为i2c通讯的读操作，其中client为客户端， reg为读寄存器的地址，reglen为地址的长度
** buf为读到的数据存储的容器，len为所读数据的字节数。
** 注意：如果ic为主设备，不需要知道读寄存器的地址，可以把reg指设为NULL, reglen为0；
** 如果ic位从设备，需要知道寄存器的地址，并且要知道寄存器地址的字节数。
** 移植程序时，请根据自己的平台及ic来实现这个函数的功能, 本例是rk平台的实现
*/
static inline int 
ts_model_i2c_read_bytes(struct i2c_client *client, void *reg, int reglen, void *buf, int len)
{
    int ret;
    if (reg == NULL) {
        ret = i2c_master_normal_recv(client, buf, len, TS_I2C_SPEED);
    }
    else if (reglen == 1){
        ret = i2c_master_reg8_recv(client, *((char *)reg), (char *)buf, len, TS_I2C_SPEED);
    }
    else if (reglen == 2) {
        ret = i2c_master_reg16_recv(client, *((short *)reg), (short *)buf, len, TS_I2C_SPEED);
    }
    else {
        printk("%s:%d:arg error!\n", __func__, __LINE__);
        ret = -1;
    }
    return ((ret > 0) ? 0 : ret);
}

/*
** 本函数为i2c通讯的写操作，其中client为客户端， reg为读寄存器的地址，reglen为地址的长度
** buf为要写的数据存储的容器，len为写数据的字节数。
** 注意：如果ic为主设备，不需要知道读寄存器的地址，可以把reg指设为NULL, reglen为0；
** 如果ic为从设备，需要知道寄存器的地址，并且要知道寄存器地址的字节数。
** 移植程序时，请根据自己的平台及ic来实现这个函数的功能 本例是rk平台的实现
*/
static inline int 
ts_model_i2c_write_bytes(struct i2c_client *client, void *reg, int reglen,  void *buf, int len)
{
    int ret;
    if (reg == NULL) {
        ret = i2c_master_normal_send(client, buf, len, TS_I2C_SPEED);
    }
    else if (reglen == 1){
        ret = i2c_master_reg8_send(client, *((char *)reg), (char *)buf, len, TS_I2C_SPEED);
    }
    else if (reglen == 2) {
        ret = i2c_master_reg16_send(client, *((short*)reg), (short *)buf, len, TS_I2C_SPEED);
    }
    else {
        printk("%s:%d:arg error!\n", __func__, __LINE__);
        ret = -1;
    }
    return ((ret > 0) ? 0 : ret);
}

static inline int ts_model_i2c_test(struct i2c_client *client)
{
	int ret, retry;
	char test_data[1] = {0};
	
	retry = 0;
	//printk("%s:%d:start... retry = %d\n", __func__, __LINE__, retry);
	while (retry++ < 10) {
		ret = ts_model_i2c_write_bytes(client, NULL, 0, test_data, 1);
		if (ret == 0)
			break;
		printk("ret = %d\n", ret);
		msleep(5);
	}
	printk("%s:%d:end retry = %d\n", __func__, __LINE__, retry);
	return ret;
}

#ifdef SUPPORT_CHARGE_MODE
/*
 * add for write charge_mode_enable cmd 
 */

static void ts_write_charge_enable_cmd(struct data_zet6221_ts *ts)
{

	char ts_write_charge_cmd[1] = {0xb5}; 
	int ret = 0;
	Ts_debug("%s is running ==========",__FUNCTION__);
	ret = ts_model_i2c_write_bytes(ts->client, NULL, 0, (void *)(&ts_write_charge_cmd), 1);
	if (ret != 0) {
		printk("%s:%d:i2c write failed!\n", __func__, __LINE__);
	}
	msleep(50);
}

static void ts_write_charge_disable_cmd(struct data_zet6221_ts *ts)
{
	
	
	char ts_write_cmd[1] = {0xb6}; 
	int ret = 0;
	Ts_debug("%s is running ==========",__FUNCTION__);
	ret = ts_model_i2c_write_bytes(ts->client, NULL, 0, ts_write_cmd, 1);
	if (ret != 0) {
		printk("%s:%d:i2c write failed!\n", __func__, __LINE__);
	}
	msleep(50);
}

static void write_cmd_work(struct work_struct *work1)
{
	struct data_zet6221_ts *ts = container_of(work1, struct data_zet6221_ts, work_charge);
	Ts_debug("dc_adaptor_status:%d\n", dc_adaptor_status);
	if(dc_adaptor_status != last_power_status)
	{	
		if(dc_adaptor_status == POWER_STATUS_CHARGING) {
			ts_write_charge_enable_cmd(ts);
			
		}
		else if(dc_adaptor_status == POWER_STATUS_NORMAL) {
			ts_write_charge_disable_cmd(ts);
		}
		last_power_status = dc_adaptor_status;
		Ts_debug("ChargeChangeFlag = %d\n",last_power_status);
	}

}

static enum hrtimer_restart ts_timer(struct hrtimer *timer)
{	
	struct data_zet6221_ts *ts = container_of(timer, struct data_zet6221_ts, timer);
	queue_work(ts->wq_charge, &ts->work_charge);
	hrtimer_start(&ts->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}
/*
 * add for write charge_mode_enable cmd end
 */
#endif

#if defined(READ_ZET6221_TPINFO)
static inline int zet_read_tp_info(struct data_zet6221_ts *ts)
{
	int i;
	struct zet6221_ts_platform_data *pdata;
	pdata = ts->client->dev.platform_data;
	 
	pdata->zet6221_ts_reset();
	#define TRY_READ_TPINFO_CNT  5
	for (i = 0; i < TRY_READ_TPINFO_CNT; i++) {
		/* 如果获取TP信息失败，就赋默认值 */
		if (zet6221_ts_get_report_mode_t(ts->client) == 0) {
			ResolutionX = TOUCHSCREEN_MAX_WIDTH;
			ResolutionY = TOUCHSCREEN_MAX_HEIGHT;
			FingerNum = MAX_SUPPORT_POINTS;
			KeyNum = ZET_KEY_NUM;
			if(KeyNum == 0)
				bufLength  = 3 + 4 * FingerNum;
			else
				bufLength  = 3 + 4 * FingerNum + 1;
		}
		else if (zet6221_ts_version() != 0) {
			printk("the detected touchscreen: width = %d, height = %d\n", ResolutionX, ResolutionY);
			return 0;
		}
	}
	return -1;
}
#endif

#if (defined (ZET6221_UPGRADE_FW) || defined(READ_ZET6221_TPINFO))
static void detect_tp_zet6221(void)
{
	zet_ts_type = ts_check_type();
	if(TS_TYPE_ERROR == zet_ts_type) {
		printk("Zet6221 No correct type check, use default firmware\n");
	}
	switch(zet_ts_type){
		case TS_TYPE_QIUTIAN:
			fw_zet6221_config_p = &CTPM_FW_ZET6221_QIUTIAN;
			FW_ZET6221_LEN = SIZE_ARRAY(CTPM_FW_ZET6221_QIUTIAN);
			printk("ADC:CTPM_ZET6221_QIUTIAN, len = %d\n", FW_ZET6221_LEN);
			break;
		case TS_TYPE_PINGBO:
			fw_zet6221_config_p = &CTPM_FW_ZET6221_PINGBO;
			FW_ZET6221_LEN = SIZE_ARRAY(CTPM_FW_ZET6221_PINGBO);
			printk("ADC:CTPM_ZET6221_PINGBO, len = %d\n", FW_ZET6221_LEN);
			break;
		case TS_TYPE_RUISHI:
			fw_zet6221_config_p = &CTPM_FW_ZET6221_RUISHI;
			FW_ZET6221_LEN = SIZE_ARRAY(CTPM_FW_ZET6221_RUISHI);
			printk("ADC:CTPM_FW_ZET6221_RUISHI, len = %d\n", FW_ZET6221_LEN);
			break;
		case TS_TYPE_HAOEN:
			fw_zet6221_config_p = &CTPM_FW_ZET6221_HAOEN;
			FW_ZET6221_LEN = SIZE_ARRAY(CTPM_FW_ZET6221_HAOEN);
			printk("ADC:CTPM_FW_ZET6221_HAOEN, len = %d\n", FW_ZET6221_LEN);
			break;
		case TS_TYPE_NULL1:
			fw_zet6221_config_p = &CTPM_FW_ZET6221_DEFAULT;
			FW_ZET6221_LEN = SIZE_ARRAY(CTPM_FW_ZET6221_DEFAULT);
			printk("ADC:CTPM_FW_ZET6221_NULL1, len = %d\n", FW_ZET6221_LEN);
			break;
		default:
			fw_zet6221_config_p = &CTPM_FW_ZET6221_DEFAULT;
			FW_ZET6221_LEN = SIZE_ARRAY(CTPM_FW_ZET6221_DEFAULT);
			printk("ADC:CTPM_FW_ZET6221_Default, len = %d\n", FW_ZET6221_LEN);
			break;
	}
	
}
#endif

/*
** ic的初始化函数
** 注意：有些ic的固件支持在线升级功能，如果要实现这个功能，请在这个函数里调用
*/
static inline int init_chip_zet6221_hw(struct data_zet6221_ts *ts)
{  
	int ret;
	int i;
	struct zet6221_ts_platform_data *pdata;
	pdata = ts->client->dev.platform_data;
	/* 1. i2c test */
	ret = ts_model_i2c_test(ts->client);
	if (ret != 0) {
		printk("ts_model_i2c_test failured!\n");
		return -1;
	}
	pdata->zet6221_ts_reset();

	#if defined(ZET6221_UPGRADE_FW) || defined(READ_ZET6221_TPINFO)
	detect_tp_zet6221();
	#endif
	
	/* 2. ic upgrade */
	#ifdef ZET6221_UPGRADE_FW
	zet6221_downloader(ts->client, FW_ZET6221_LEN);
	#endif

	/* 3. read tp info */
	#if defined(READ_ZET6221_TPINFO)
	#define READ_TP_INFO_CNT 2
	for (i = 0; i < READ_TP_INFO_CNT; i++) {
		ret = zet_read_tp_info(ts);
		if (ret == 0) {
			printk("%s:%d: zet6221 read TP info success!\n", __func__, __LINE__);
			break;
		}
		else {
			printk("%s:%d: zet6221 read TP info failed, retry it cnt = %d!\n", __func__, __LINE__, i);
			#ifdef ZET6221_UPGRADE_FW
			//读取TP信息失败，重新升级ic
			zet6221_downloader(ts->client, FW_ZET6221_LEN);
			#endif
		}
	}
	#endif
    return 0;
}


/*
** 这个函数实现的功能是：
** 把数据处理成struct point_data结构的数据
** 注意：请根据你的ic，来实现这个函数
*/
static char finger_save; // save the last fingers state, for look
static unsigned char no_touch_cnt;
static inline int process_data_zet6221(unsigned char *src, int src_num, struct point_data *dst, int dst_num)
{
	int i;
	char finger_current;
	int action_valid_flag = 1; // zet6221_ts when no touch action, the ic report 5 interrupts
	
	finger_current = src[1];
	Ts_debug("%s: finger = %x, no_touch_cnt = %d\n", __func__, finger_current, (int)no_touch_cnt);
	if ((finger_current & (unsigned char)0xf8) == 0) {
		Ts_debug("no touch start...\n");
		no_touch_cnt++;
		if (no_touch_cnt > 2) {
			action_valid_flag = 1;
			no_touch_cnt = 0;
		}
		else {
			action_valid_flag = 0;
		}
	}
	else {
		no_touch_cnt = 0;
		action_valid_flag = 1;
	}
	
	for (i = 0; i < dst_num; i++) {
		if (finger_current & (0x80 >> i)) {
			dst[i].id = i;
			dst[i].point_x = ((src[4 * i + 3]  & 0xf0) << 4) | (src[4 * i + 3 + 1]);
			dst[i].point_y = ((src[4 * i + 3] & 0xf) << 8) | (src[4 * i + 3 + 2]);
			dst[i].point_p = src[4 * i + 3 + 3] & 0xf;
			dst[i].state = FINGER_DOWN;
		}
		else {
			dst[i].point_p = 0;
			if (((finger_current & (0x80 >> i)) ^ (finger_save & (0x80 >> i)))
					&& (action_valid_flag == 1)) {
				dst[i].id = i;
				dst[i].state = FINGER_UP;
			}
			else {
				dst[i].state = FINGER_INVALID;
			}
		}
	}
	if (action_valid_flag == 1) {
		finger_save = finger_current;
	}
    return 0;
}

/*
**注意： android 4.0 和 android 2.3略有不同，移植时请自行修改
*/
static inline void report_data_ts(struct data_zet6221_ts *ts, struct point_data *data, int num)
{
    int i;

    #if defined(TS_MODLE_MULTI_POINT_TOUCH)
    for (i = 0; i < num; i++) {
        if (data[i].state == FINGER_INVALID) {
            continue;
        }
        if (data[i].state == FINGER_DOWN) {
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, data[i].point_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, data[i].point_y);
			Ts_debug("i = %d, Id:%d, x:%d, y:%d\n", i, data[i].id, data[i].point_x, data[i].point_y);
		} 
		else {  //state is FINGER_UP
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
			Ts_debug("id:%d, i = %d\n", data[i].id, i);
	
		}
		data[i].state = FINGER_INVALID;
    }
    #else //sigle point touch
    if (data[0].state == FINGER_DOWN) {
        input_report_abs(ts->input_dev, ABS_X, data[0].point_x);
	    input_report_abs(ts->input_dev, ABS_Y, data[0].point_y);
        data[0].state = FINGER_INVALID;
    }
    #endif
    input_sync(ts->input_dev);
}

static inline int read_point_values_zet6221(struct data_zet6221_ts *ts)
{
	/*8位的寄存器用char型， 16位的用short型*/
    char raw_data[READ_DATA_BYTES] = {0};
    struct point_data value_data[MAX_SUPPORT_POINTS];
    int ret = -1;
	
	memset(value_data, 0, sizeof(value_data));
    /* 从寄存器里提取原始的数据 */
    ret = ts_model_i2c_read_bytes(ts->client, (void *)ts_read_reg, REG_ADDR_LENGTH, 
    						(void *)&raw_data, SIZE_ARRAY(raw_data));
    if (ret < 0) {
        Ts_debug("%s:%d:i2c_read_bytes error!\n", __func__, __LINE__);
        return ret;
    }
    
    /* 处理原始数据，得到我们需要的数据形式 */
    ret = process_data_zet6221((unsigned char *)&raw_data, SIZE_ARRAY(raw_data), 
    					(struct point_data *)&value_data, SIZE_ARRAY(value_data));
    if (ret < 0) {
        Ts_debug("%s:%d:the data invalid!\n",  __func__, __LINE__);
        return ret;
    }

    /* 上报数据 */
    report_data_ts(ts, (struct point_data *)&value_data, SIZE_ARRAY(value_data));
    return 0;
}

/*
** note: this function depends your touchscreen ic and the the way of touchscreen's work
*/
static void work_zet6221_ts(struct work_struct *work)
{
    struct data_zet6221_ts *ts = container_of(to_delayed_work(work), struct data_zet6221_ts, work);
    int ret;

    Ts_debug("%s is called!\n", __func__);
    ret = read_point_values_zet6221(ts);
	enable_irq(ts->irq);
}

/*
** note: this function depends your touchscreen ic and the the way of touchscreen's work
*/
static irqreturn_t irq_zet6221_ts(int irq, void *dev_id)
{
	struct data_zet6221_ts *ts = dev_id;

    Ts_debug("%s is called!\n", __func__);
    disable_irq_nosync(ts->irq);
    queue_delayed_work(ts->wq, &ts->work, msecs_to_jiffies(10));
    return IRQ_HANDLED;
}

/*
** note: this function depends your touchscreen ic and the the way of touchscreen's work
*/
static void free_irq_zet6221_ts(struct data_zet6221_ts *ts)
{
   	free_irq(ts->irq, ts);
	if (cancel_delayed_work_sync(&ts->work)) {
		/*
		 * Work was pending, therefore we need to enable
		 * IRQ here to balance the disable_irq() done in the
		 * interrupt handler.
		 */
		enable_irq(ts->irq);
	} 
}

#ifdef CONFIG_HAS_EARLYSUSPEND	
//#define PRINT_SUSPEND_INFO

/*
** note: this function depend the touchscreen ic and the platform
*/
static void ts_early_suspend_zet6221(struct early_suspend *h)
{
	struct zet6221_ts_platform_data *pdata;
	struct data_zet6221_ts *ts = container_of(h, struct data_zet6221_ts, early_suspend);
	pdata = ts->client->dev.platform_data;
	
    #ifdef PRINT_SUSPEND_INFO
	printk("%s is callled!\n", __func__);
    #endif
	#ifdef SUPPORT_CHARGE_MODE
	hrtimer_cancel(&ts->timer);
	cancel_work_sync(&ts->work_charge);
	#endif

	disable_irq(ts->irq);
	if (pdata->zet6221_ts_platform_sleep) {
		pdata->zet6221_ts_platform_sleep();
	}
	//printk("%s is end!\n", __func__);
}

/*
** note: this function depend the touchscreen ic and the platform
*/
static void ts_early_resume_zet6221(struct early_suspend *h)
{
    struct zet6221_ts_platform_data *pdata;
	struct data_zet6221_ts *ts = container_of(h, struct data_zet6221_ts, early_suspend);
	pdata = ts->client->dev.platform_data;
	
    #ifdef PRINT_SUSPEND_INFO
	printk("%s is callled!\n", __func__);
    #endif
	#ifdef SUPPORT_CHARGE_MODE
	hrtimer_start(&ts->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
	#endif

	if (pdata->zet6221_ts_platform_wakeup) {
		pdata->zet6221_ts_platform_wakeup();
	}
	enable_irq(ts->irq);
	#ifdef SUPPORT_CHARGE_MODE
	if(dc_adaptor_status/* == POWER_STATUS_CHARGING*/) {
		ts_write_charge_enable_cmd(ts);		
	}
	else {
		ts_write_charge_disable_cmd(ts);
	}
	//wakeup
	//ctp_wakeup();
	#endif

	//printk("%s is end!\n", __func__);
}

#endif


/*
** 设置输入设备的工作方式：能产生那类事件以及这类事件的哪些事件
** 注意：这个函数与android系统有关，本例子是RK平台android4.0上的代码
*/
static inline void setting_ts_input_dev_mode(struct input_dev *input_dev)
{
    __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	__set_bit(EV_ABS, input_dev->evbit);
    #if defined(TS_MODLE_MULTI_POINT_TOUCH)	
	input_mt_init_slots(input_dev, MAX_SUPPORT_POINTS);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,0, ResolutionX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ResolutionY, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	#else
	input_set_abs_params(input_dev,ABS_X,0,ResolutionX,0,0);
	input_set_abs_params(input_dev,ABS_Y,0,ResolutionY,0,0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);
    #endif

}

/*
** 根据需要，请自行实现这个函数的功能
*/
static inline int setting_zet6221_ts_work_mode(struct data_zet6221_ts *ts)
{
    ts->wq = create_workqueue("ts_wq");
    if (!ts->wq) {
        printk("%s:%d:create_workqueue_failed!\n", __func__, __LINE__);
        return -1;
    }
	INIT_DELAYED_WORK(&ts->work, work_zet6221_ts);
	#ifdef SUPPORT_CHARGE_MODE
	ts->wq_charge = create_singlethread_workqueue("ts_wq1"); // workqueue
	if (!ts->wq_charge) {
		printk("%s:%d:create_singlethread_workqueue_failed!\n", __func__, __LINE__);
		return -1;	
	}
	INIT_WORK(&ts->work_charge, write_cmd_work);
	flush_workqueue(ts->wq_charge);
	
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = ts_timer;
	hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	#endif

    return 0;
}

static inline void exit_zet6221_ts_work_mode(struct data_zet6221_ts *ts)
{
    if (ts->wq) {
        flush_workqueue(ts->wq);
       	destroy_workqueue(ts->wq);
    }
	#ifdef SUPPORT_CHARGE_MODE
	if (ts->wq_charge) {
		flush_workqueue(ts->wq_charge);
        destroy_workqueue(ts->wq_charge);
	}
	hrtimer_cancel(&ts->timer);
	#endif

}

static int  probe_zet6221_ts(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct data_zet6221_ts *ts;
    struct input_dev *input_dev;
    struct zet6221_ts_platform_data *pdata = client->dev.platform_data;
    int err = 0;
   
	printk("===================probe_zet6221_ts v1.0========================\n");
    /* 0. checkout */
	if (Ts_probe_success_flag) {
		printk("%s:%d: probe failed as the touchscreen had been probed successfully!\n",
				__func__, __LINE__);
		return -1;
	}
    if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk("%s:%d:i2c_check_functionality_failed!\n", __func__, __LINE__);
		return -EIO;
	}

	/* 1.alloc memory space:data_zet6221_ts,input_dev */
	ts = kzalloc(sizeof(struct data_zet6221_ts), GFP_KERNEL);
	if (!ts) {
        printk("%s:%d:alloc_ts_model_data_failed!\n", __func__, __LINE__);
		return ENOMEM;
	}
	ts->client = client;
	
    input_dev = input_allocate_device();
    if (!input_dev) {
		err = -ENOMEM;
        printk("%s:%d:alloc_input_dev_failed!\n", __func__, __LINE__);
		goto alloc_input_dev_failed;
    }
    ts->input_dev = input_dev;

	/* 2.related hadware setting */
    /* 2.1 related platform */
    if (pdata->init_platform_hw) {
		Ts_debug("init platform hw start...\n");
        err = pdata->init_platform_hw();
        if (err < 0) {
            printk("%s:%d:init_platform_hw failed!\n", __func__, __LINE__);
            goto init_platform_hw_failed;
        }
    }   
	
	msleep(200);
	/* 2.2 related the touchscreen ic */
	err = init_chip_zet6221_hw(ts);
	if (err < 0) {
		printk("%s:%d:init_chip_zet6221_hw failed!\n", __func__, __LINE__);
		goto init_chip_hw_failed;
	}

	/* 3.setting */
	/*3.1 setting the input device mode*/
	setting_ts_input_dev_mode(input_dev);
	/*3.3 setting the ic work mode */
	err = setting_zet6221_ts_work_mode(ts);
	if (err < 0) {
		printk("%s:%d:set_ts_work_mode_failed!\n", __func__, __LINE__);
		goto set_ts_work_mode_failed;
	}
	
	/* 3.4 other setting */
	ts->irq = client->irq;
	i2c_set_clientdata(client, ts);
	//snprintf(ts->phys, sizeof(ts->phys), "%s/input3", dev_name(&client->dev));
	//input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0xDEAD;
	input_dev->id.product = 0xBEEF;
	input_dev->id.version = 0x1105;	

	switch(zet_ts_type){
		case TS_TYPE_QIUTIAN:
			input_dev->name = "ZET6221_QiuTian";
			break;
		case TS_TYPE_PINGBO:
			input_dev->name = "ZET6221_PinBo";
			break;
		case TS_TYPE_RUISHI:
			input_dev->name = "ZET6221_RuiShi";
			break;
		default:
			input_dev->name = "ZET6221_Default";
			break;
	}
	
    /* 4.register input_dev */
    err = input_register_device(input_dev);
    if (err < 0) {
         printk("%s:%d:input_register_device failed!\n", __func__, __LINE__);
         goto input_register_device_failed;
    }

    /* 5. request irq */
    if (!ts->irq) {
		dev_dbg(&ts->client->dev, "no IRQ?\n");
		err = -ENODEV;
		goto irq_request_failed;
	} 
    else {
		ts->irq = gpio_to_irq(ts->irq);
	}
    //gpio_direction_input(TOUCHSCREEN_INT_PIN);
    err = request_irq(ts->irq, irq_zet6221_ts, TS_IRQ_MODE_ZET6221, client->dev.driver->name, ts);
	if (err < 0) {
        printk("%s:%d:request_irq failed:irq %d busy\n", __func__, __LINE__, ts->irq);
		goto irq_request_failed;
	}

    #ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	ts->early_suspend.suspend = ts_early_suspend_zet6221;
	ts->early_suspend.resume = ts_early_resume_zet6221;
	register_early_suspend(&ts->early_suspend);
    #endif
	
    Ts_probe_success_flag = 1;
    printk("========probe_zet6221_ts Ok==============\n");
	return 0;

irq_request_failed:

    input_unregister_device(input_dev);
input_register_device_failed:
    exit_zet6221_ts_work_mode(ts);
    i2c_set_clientdata(client, NULL);
set_ts_work_mode_failed:
init_chip_hw_failed:
    if (pdata->exit_platform_hw) {
		pdata->exit_platform_hw();
    }
init_platform_hw_failed:
	input_free_device(input_dev);
alloc_input_dev_failed:
    kfree(ts);
    printk("============probe_zet6221_ts failed!===========\n");
    return err;
}

static int  remove_zet6221_ts(struct i2c_client *client)
{
    struct data_zet6221_ts *ts = i2c_get_clientdata(client);
    struct zet6221_ts_platform_data *pdata = client->dev.platform_data;
    
    free_irq(ts->irq, ts);
    exit_zet6221_ts_work_mode(ts);
    i2c_set_clientdata(client, NULL);
    #ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend((&ts->early_suspend));
    #endif
    free_irq_zet6221_ts(ts);

    if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();

    input_unregister_device(ts->input_dev);
    input_free_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static const struct i2c_device_id idtable_zet6221_ts[] = {
	{ I2C_NAME_ZET6221, 0 },
	{ }
};

static struct i2c_driver driver_zet6221_ts = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= I2C_NAME_ZET6221
	},
	.id_table	= idtable_zet6221_ts,
	.probe		= probe_zet6221_ts,
	.remove		= remove_zet6221_ts,
};

static int __init init_zet6221_ts(void)
{
	int ret;
	ret = i2c_add_driver(&driver_zet6221_ts);
	return 0;
}

static void __exit exit_zet6221_ts(void)
{
	return i2c_del_driver(&driver_zet6221_ts);
}

/*
 * touchscreen must init power up early, or the other I2C devices will not work.
 */
late_initcall(init_zet6221_ts);
module_exit(exit_zet6221_ts);
MODULE_DESCRIPTION("model Touchscreen Driver@wisky-hean-20120524");
MODULE_LICENSE("GPL");
