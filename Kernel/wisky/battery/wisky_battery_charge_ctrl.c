#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/power_supply.h>
#include <mach/iomux.h>
#include <mach/gpio.h>


int dc_adaptor_status = 0;//0---dc removed, 1---dc insert

extern int usb_otg_op_state;

#define SYSTEM_READY_TIME	4000//6000 jiffies(1 minute) after driver init, run thread
static unsigned long system_ready = 0;//use for thread run after system boot complete


struct battery_charge_tag{
	int init_flag;//0--not init, 1--init
	int dc_status;
	int charge_enable;
	//wisky libai 20120825
	//#define CHARGE_DISABLE	0
  //#define CHARGE_ENABLE	1
};

#define CHARGE_DISABLE	0
#define CHARGE_ENABLE	1
//end

struct battery_charge_tag battery_charge_data;

//STAT2(PIN1): Low--Charging, HIGH---Charge FULL or No charge
//return 1 if charge full, 0 if charging, else return negative if error
static int get_charge_status(void)
{
	int state1=-1, state2=-1;
	int charge_status = 0;
	
#if defined(BATTERY_CHARGE_STAT1_PIN) && defined(BATTERY_CHARGE_STAT2_PIN)
	if(BATTERY_CHARGE_STAT1_PIN != INVALID_GPIO && BATTERY_CHARGE_STAT2_PIN != INVALID_GPIO){
	//使用两个状态监测 脚STAT1 & STAT2
		state1 = gpio_get_value(BATTERY_CHARGE_STAT1_PIN);
		state2 = gpio_get_value(BATTERY_CHARGE_STAT2_PIN);
		if(GPIO_HIGH == state1 && GPIO_HIGH == state2){
			charge_status = POWER_SUPPLY_STATUS_DISCHARGING;//POWER_SUPPLY_STATUS_NOT_CHARGING;
		}else if(GPIO_HIGH == state1 && GPIO_LOW == state2){
			charge_status = POWER_SUPPLY_STATUS_CHARGING;
		}else if(GPIO_LOW == state1 && GPIO_HIGH == state2){
			charge_status = POWER_SUPPLY_STATUS_FULL;
		}
		WPRINTK("Read BAT Charge IC-->state1=%d, state2=%d\n", state1, state2);
		WPRINTK("BAT Charge IC-->charge_status=%d\n", charge_status);
	}else if(BATTERY_CHARGE_STAT2_PIN != INVALID_GPIO){
	//只使用一个状态监测脚 STAT2
		state2 = gpio_get_value(BATTERY_CHARGE_STAT2_PIN);
		if(GPIO_LOW == state2){
			//STAT2 low, charging
			charge_status = POWER_SUPPLY_STATUS_CHARGING;
		}else{
			//STAT2 high, charge full or not charge
			charge_status = POWER_SUPPLY_STATUS_FULL;
		}
		WPRINTK("Read BAT Charge IC-->state2=%d\n", state2);
		WPRINTK("BAT Charge IC-->charge_status=%d\n", charge_status);
	}
#elif defined(BATTERY_CHARGE_STAT2_PIN)
	 if(BATTERY_CHARGE_STAT2_PIN != INVALID_GPIO){
	//只使用一个状态监测脚 STAT2
		state2 = gpio_get_value(BATTERY_CHARGE_STAT2_PIN);
		if(GPIO_LOW == state2){
			//STAT2 low, charging
			charge_status = POWER_SUPPLY_STATUS_CHARGING;
		}else{
			//STAT2 high, charge full or not charge
			charge_status = POWER_SUPPLY_STATUS_FULL;
		}
		WPRINTK("Read BAT Charge IC-->state2=%d\n", state2);
		WPRINTK("BAT Charge IC-->charge_status=%d\n", charge_status);
	}
#endif
	return charge_status;
}

/*
 *return 1 if battery charge full, else return 0
 */
int is_battery_charge_full(void)
{
	int batt_full = 0;

	if((CHARGE_ENABLE == battery_charge_data.charge_enable) && (get_charge_status() == POWER_SUPPLY_STATUS_FULL)){
		batt_full = 1;
	}

	return batt_full;
}
EXPORT_SYMBOL(is_battery_charge_full);

//param:void
//return: 0---no dc adaptor present, 1--dc adaptor insert
int get_dc_status(void)
{
	unsigned int status = 0;

	if(DC_DETECT_PIN != INVALID_GPIO){
		if(gpio_get_value(DC_DETECT_PIN) == DC_DETECT_VALUE){
			if(1 == usb_otg_op_state){
				status = 0;
				WPRINTK("%s[%d]: (OTG in) DC remove\n", __FUNCTION__, __LINE__);
			}else{
				status = 1;
				WPRINTK("%s[%d]: DC insert\n", __FUNCTION__, __LINE__);
			}
		}else{
			status = 0;
			WPRINTK("%s[%d]: DC remove\n", __FUNCTION__, __LINE__);
		}
	}

	return status;
}
EXPORT_SYMBOL(get_dc_status);

static void set_charge_enable(int enable)
{
	if(enable > 0){
		WPRINTK("%s[%d]:------>Enable battery charge.\n", __FUNCTION__, __LINE__);
		battery_charge_data.charge_enable = CHARGE_ENABLE;
		#if defined(BATTERY_CHARGE_PWR_EN_PIN)
		if(BATTERY_CHARGE_PWR_EN_PIN != INVALID_GPIO){
			gpio_set_value(BATTERY_CHARGE_PWR_EN_PIN, BATTERY_CHARGE_PWR_EN_VALUE);
		}
		#endif
		
		if(BATTERY_CHARGE_EN_PIN != INVALID_GPIO){			
			gpio_set_value(BATTERY_CHARGE_EN_PIN, BATTERY_CHARGE_EN_VALUE);//enable battery charge
		}
	}else{
		WPRINTK("%s[%d]:------>Disable battery charge.\n", __FUNCTION__, __LINE__);
		battery_charge_data.charge_enable = CHARGE_DISABLE;
		#if defined(BATTERY_CHARGE_PWR_EN_PIN)
		if(BATTERY_CHARGE_PWR_EN_PIN != INVALID_GPIO){
			gpio_set_value(BATTERY_CHARGE_PWR_EN_PIN, !BATTERY_CHARGE_PWR_EN_VALUE);
		}
		#endif
		
		if(BATTERY_CHARGE_EN_PIN != INVALID_GPIO){
			gpio_set_value(BATTERY_CHARGE_EN_PIN, !BATTERY_CHARGE_EN_VALUE);//disable battery charge
		}	
	}
}

//battery charge current has 2 level, low, and high
#define BAT_CHARGE_LEVEL_MIN 		1	//about 400mA
#define BAT_CHARGE_LEVEL_MAX 		2	//about 1000mA
static void set_charge_level(int level)
{
	switch(level){
	case BAT_CHARGE_LEVEL_MIN:
		//if(CHARGE_CURRENT_CTL1_PIN != INVALID_GPIO){
		//	gpio_set_value(CHARGE_CURRENT_CTL1_PIN, !CHARGE_CURRENT_CTL1_ON_VALUE);
		//}
		
		if(CHARGE_CURRENT_CTL2_PIN != INVALID_GPIO){
			gpio_set_value(CHARGE_CURRENT_CTL2_PIN, !CHARGE_CURRENT_CTL2_ON_VALUE);
		}
		break;
	case BAT_CHARGE_LEVEL_MAX:
	default:
		//if(CHARGE_CURRENT_CTL1_PIN != INVALID_GPIO){
		//	gpio_set_value(CHARGE_CURRENT_CTL1_PIN, !CHARGE_CURRENT_CTL1_ON_VALUE);
		//}
		
		if(CHARGE_CURRENT_CTL2_PIN != INVALID_GPIO){
			gpio_set_value(CHARGE_CURRENT_CTL2_PIN, CHARGE_CURRENT_CTL2_ON_VALUE);
		}
		break;
	}
}

int battery_charge_control_init(void)
{
	set_charge_level(BAT_CHARGE_LEVEL_MIN);
	set_charge_enable(0);

	battery_charge_data.init_flag = 1;
	battery_charge_data.charge_enable = CHARGE_DISABLE;
	battery_charge_data.dc_status = get_dc_status();
	dc_adaptor_status = battery_charge_data.dc_status;
	system_ready = jiffies + SYSTEM_READY_TIME;
}
EXPORT_SYMBOL(battery_charge_control_init);

//param:void
//return: 0--not charge, 1--charging
int battery_charge_control_poll(void)
{
	int status = 0;

	if(battery_charge_data.init_flag != 1){
		pr_err("%s[%d]: should call battery_charge_control_init() in battery driver probe functio!\n", __FUNCTION__, __LINE__);
		goto exit;
	}
	
	if(time_before(jiffies, system_ready)){
		WPRINTK("%s[%d]:------>System not ready, Disable battery charge.\n", __FUNCTION__, __LINE__);
		set_charge_level(BAT_CHARGE_LEVEL_MIN);
		set_charge_enable(0);
		//before system run 1 minute
		goto exit;
	}
	
	if(get_dc_status()){
		battery_charge_data.dc_status = 1;
		set_charge_enable(1);
		set_charge_level(BAT_CHARGE_LEVEL_MAX);
		status = 1;
	}else{
		battery_charge_data.dc_status = 0;
		set_charge_enable(0);
		set_charge_level(BAT_CHARGE_LEVEL_MIN);
		status = 0;
	}
	dc_adaptor_status = battery_charge_data.dc_status;
	
exit:
	return status;
}
EXPORT_SYMBOL(battery_charge_control_poll);






