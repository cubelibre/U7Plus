
#ifndef _CHARGE_H_
#define _CHARGE_H_


#define DELAY_TIME_MS  		500
#define FULL_SHOW_LOGO_TIME	(600*3)///3 minutes

#define BATT_LEVEL_POOR		5
#define BATT_LEVEL_FULL		100

enum {
	LOGO_BLANK = 0,
	LOGO_NUM1,
	LOGO_NUM2,
	LOGO_NUM3,
	LOGO_NUM4,
	LOGO_NUM5,
	LOGO_NUM6,
	LOGO_POOR,
	LOGO_FULL,
	LOGO_BOOTUP,
};



extern struct linux_logo logo_charge_blank_clut224;
extern struct linux_logo logo_charge0_clut224;
extern struct linux_logo logo_charge1_clut224;
extern struct linux_logo logo_charge2_clut224;
extern struct linux_logo logo_charge3_clut224;
extern struct linux_logo logo_charge4_clut224;
extern struct linux_logo logo_charge_poor_clut224;
extern struct linux_logo logo_charge_full_clut224;

extern void show_logo(int index);

#endif// _CHARGE_H_

