#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/adc.h>


/*
* ANI2=0.33V R19=10K R20=1.5K 使用(秋田微)触摸屏
* ANI2=0.80V R19=10K R20=4.7K 使用(平波)触摸屏
* ANI2=1.36V R19=10K R20=12K 使用(瑞视光电)触摸屏
* ANI2=1.82V R19=10K R20=27K 使用(豪恩)触摸屏
* ANI2=2.12V R19=10K R20=56K 使用(XX)触摸屏
*/
#define ADC_VALUE_QIUTIAN	133
#define ADC_VALUE_PINGBO	335
#define ADC_VALUE_RUISHI		563
#define ADC_VALUE_HAOEN		750
#define ADC_VALUE_NULL1		860

#define ADC_DRIFT_VALUE	30

struct ts_check_data{
	int adc_channel;
	int adc_value;
	struct adc_client *adc_client;
	struct completion adc_complete;
};

struct ts_check_data my_ts_check;



//param:none
//return: negative number--no correct ts detect, positive number--ts type
//返回值参考请文件:wisky_ts_check.h 文件中的enum ts_type
int ts_check_type(void)
{
	int type = TS_TYPE_ERROR;
	
#if defined(TS_CHECK_ADC_CHANNEL)
	if(my_ts_check.adc_client){
		init_completion(&my_ts_check.adc_complete);
		adc_async_read(my_ts_check.adc_client);
		wait_for_completion_timeout(&my_ts_check.adc_complete, HZ*5);
	}

	if(my_ts_check.adc_value > (ADC_VALUE_QIUTIAN - ADC_DRIFT_VALUE)
	&&my_ts_check.adc_value < (ADC_VALUE_QIUTIAN + ADC_DRIFT_VALUE)){
	//(秋田微)
		type = TS_TYPE_QIUTIAN;
	}else if(my_ts_check.adc_value > (ADC_VALUE_PINGBO - ADC_DRIFT_VALUE)
	&&my_ts_check.adc_value < (ADC_VALUE_PINGBO + ADC_DRIFT_VALUE)){
	//(平波)
		type = TS_TYPE_PINGBO;
	}else if(my_ts_check.adc_value > (ADC_VALUE_RUISHI - ADC_DRIFT_VALUE)
	&&my_ts_check.adc_value < (ADC_VALUE_RUISHI + ADC_DRIFT_VALUE)){
	//(瑞视光电)
		type = TS_TYPE_RUISHI;
	}else if(my_ts_check.adc_value > (ADC_VALUE_HAOEN - ADC_DRIFT_VALUE)
	&&my_ts_check.adc_value < (ADC_VALUE_HAOEN + ADC_DRIFT_VALUE)){
	//(豪恩)
		type = TS_TYPE_HAOEN;
	}else if(my_ts_check.adc_value > (ADC_VALUE_NULL1 - ADC_DRIFT_VALUE)
	&&my_ts_check.adc_value < (ADC_VALUE_NULL1 + ADC_DRIFT_VALUE)){
	//未定义
		type = TS_TYPE_NULL1;
	}else{
	//如检测到未知类型，则
		type = TS_TYPE_ERROR;
	}
#endif
	
	return type;
}
EXPORT_SYMBOL(ts_check_type);

static void ts_check_callback(struct adc_client *client, void *client_param, int result)
{
	struct ts_check_data *ts_check = (struct ts_check_data *)client_param;

	pr_info("[TouchScreen Type Check] adc callback---------> ADC Result = %d\n", result);
	my_ts_check.adc_value = result;
	complete(&my_ts_check.adc_complete);
}

static int __init ts_check_init(void)
{
	#if defined(TS_CHECK_ADC_CHANNEL)
	if(TS_CHECK_ADC_CHANNEL >= 0){
		my_ts_check.adc_channel = TS_CHECK_ADC_CHANNEL;
		my_ts_check.adc_value = 0;
		my_ts_check.adc_client = adc_register(my_ts_check.adc_channel, ts_check_callback, (void*)&my_ts_check);
		if(!my_ts_check.adc_client){
			pr_err("[TS Check] adc client register error!\n");
			return -EFAULT;
		}
	}
	pr_info("[TS Check] TouchScreen type check by ADC[%d]\n", TS_CHECK_ADC_CHANNEL);
	#endif	
	
	return 0;
}

static void ts_check_exit(void)
{
	#if defined(TS_CHECK_ADC_CHANNEL)
	if(TS_CHECK_ADC_CHANNEL >= 0){
		if(my_ts_check.adc_client){
			adc_unregister(my_ts_check.adc_client);
		}
	}
	#endif
}


fs_initcall(ts_check_init);
module_exit(ts_check_exit);

MODULE_DESCRIPTION("Wisky TouchScreen Check");
MODULE_LICENSE("GPL");


