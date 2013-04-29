#include <mach/gpio.h>
#include <plat/key.h>

#define EV_ENCALL				KEY_F4
#define EV_MENU				KEY_F1

#define PRESS_LEV_LOW		1
#define PRESS_LEV_HIGH		0

static struct rk29_keys_button key_button[] = {
#if 0
	{
		.desc	= "menu",
		.code	= EV_MENU,
		.gpio	= RK30_PIN6_PA0,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "vol+",
		.code	= KEY_VOLUMEUP,
		.gpio	= RK30_PIN6_PA1,
		.active_low = PRESS_LEV_LOW,
	},

	{
		.desc	= "vol-",
		.code	= KEY_VOLUMEDOWN,
		.gpio	= RK30_PIN4_PC5,
		.active_low = PRESS_LEV_LOW,
	},
#endif
//wisky-lxh@20120731,board with axp202 don't use gpio power key,this gpio be used as RTC interrupt pin
#ifndef CONFIG_KP_AXP
	{
		.desc	= "play",
		.code = KEY_POWER,
		.gpio	= KEY_POWER_PIN,
		.active_low = PRESS_LEV_LOW,
		//.code_long_press = EV_ENCALL,
		.wakeup	= 1,
	},
#endif
//end-wisky-lxh
//----------ADC key---------------

	{
		.desc	= "vol+",
		.code = KEY_VOLUMEUP,
		.adc_value = KEY_VOLUMEUP_ADC_VALUE,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},

	{
		.desc	= "vol-",
		.code = KEY_VOLUMEDOWN,
		.adc_value = KEY_VOLUMEDOWN_ADC_VALUE,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},

#if 1
	{
		.desc	= "menu",
		.code = EV_MENU,
		.adc_value = KEY_MENU_ADC_VALUE,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	
	{
		.desc	= "home",
		.code = KEY_HOME,
		.adc_value = KEY_HOME_ADC_VALUE,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	
	{
		.desc	= "esc",
		.code = KEY_BACK,
		.adc_value = KEY_ESC_ADC_VALUE,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
#endif	
#if 0
	{
		.desc	= "camera",
		.code	= KEY_CAMERA,
		.adc_value	= 827,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
#endif
};
struct rk29_keys_platform_data rk29_keys_pdata = {
	.buttons = key_button,
	.nbuttons	= ARRAY_SIZE(key_button),
	.chn	= KEY_ADC_CHANNEL,  //chn: 0-7, if do not use ADC,set 'chn' -1
};

