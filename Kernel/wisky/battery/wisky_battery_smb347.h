/*
 * Copyright (C) 2012 Wisky Ltd
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
 * LOG
 * ------------------------------------------------------------
 *
 */

#ifndef _WISKY_BATTERY_SMB347_H_
#define _WISKY_BATTERY_SMB347_H_

#define SMB347_NAME	"smb347"
#define SMB347_I2C_SCL_RATE		300*1000

#define SDM347_BUF_SIZE	2

//R:Read only
//W:Write only
#define SMB347_STAT_TIMER_CTRL	0x05
	#define STAT_OUTPUT_POLARITY_LOW	(0<<7)
	#define STAT_OUTPUT_POLARITY_HIGH	(1<<7)
	#define STAT_OUTPUT_MODE_CHARGING	(1<<6)
	#define STAT_OUTPUT_CTRL_ENABLE		(0<<5)
	#define STAT_OUTPUT_CTRL_DISABLE		(1<<5)
#define SMB347_OTHER_CTRLA	0x09
#define SMD347_STATUS_INT	0x0D

#define SMB347_CMD_REGA	0x30
	#define OTG_DISABLE	(0<<4)
	#define OTG_ENABLE	(1<<4)
#define SMB347_CMD_REGB	0x31
#define SMB347_CMD_REGC	0x33

#define SMB347_INT_STATUS_REGA	0x35
#define SMB347_INT_STATUS_REGB	0x36
#define SMB347_INT_STATUS_REGC	0x37
#define SMB347_INT_STATUS_REGD	0x38
#define SMB347_INT_STATUS_REGE	0x39
#define SMB347_INT_STATUS_REGF	0x3A
#define SMB347_STATUS_REGA	0x3B	//R
#define SMB347_STATUS_REGB	0x3C	//R
#define SMB347_STATUS_REGC	0x3D	//R
#define SMB347_STATUS_REGD	0x3E	//R
#define SMB347_STATUS_REGE	0x3F	//R


#endif //_WISKY_BATTERY_SMB347_H_