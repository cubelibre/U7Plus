/*
 * Definitions for dmard06 compass chip.
 */
#ifndef DMARD06_H
#define DMARD06_H

#include <linux/ioctl.h>

#define GSENSOR_MISC_DEV_NAME	"mma8452_daemon"//"gsensor"
#define GSENSOR_INPUT_DEV_NAME	"gsensor"
/* IOCTLs for G-Sensor library */
#define GSENSOR_IO				0xA1
#define GSENSOR_IOCTL_INIT		_IO(GSENSOR_IO, 0x01)
#define GSENSOR_IOCTL_RESET		_IO(GSENSOR_IO, 0x04)
#define GSENSOR_IOCTL_CLOSE		_IO(GSENSOR_IO, 0x02)
#define GSENSOR_IOCTL_START		_IO(GSENSOR_IO, 0x03)
#define GSENSOR_IOCTL_GETDATA	_IOR(GSENSOR_IO, 0x08, char[RBUFF_SIZE+1])
/* IOCTLs for APPs */
#define GSENSOR_IOCTL_SET_RATE		_IOW(GSENSOR_IO, 0x10, char)

/* Default register settings */
#define RBUFF_SIZE		12	/* Rx buffer size */

#define DMARD06_REG_T_OUT       0x40  //r
#define DMARD06_REG_X_OUT       0x41
#define DMARD06_REG_Y_OUT       0x42
#define DMARD06_REG_Z_OUT       0x43

#define DMARD06_REG_PM       0x44  //w  default0x27   poweroff is 07 
#define DMARD06_REG_INT      0x47
/*#define DMARD06_REG_TILT        0x3
#define DMARD06_REG_SRST        0x4
#define DMARD06_REG_SPCNT       0x5
#define DMARD06_REG_INTSU       0x6
#define DMARD06_REG_MODE        0x7
#define DMARD06_REG_SR          0x8
#define DMARD06_REG_PDET        0x9
#define DMARD06_REG_PD          0xa
*/

/*status*/
#define DMARD06_OPEN           1
#define DMARD06_CLOSE          0

#define DMARD06_IIC_ADDR 	    0x38  
#define DMARD06_REG_LEN         11
//HAL:*(9.80665 / 1000)
#define DMARD06_RANGE			1000000
#define DMARD06_PRECISION       6
#define DMARD06_BOUNDARY        (0x1 << (DMARD06_PRECISION - 1))
#define DMARD06_GRAVITY_STEP    15000
#define DMARD06_TOTAL_TIME      10

struct dmard06_data {
    char  status;
    char  curr_tate;
	struct input_dev *input_dev;
	struct i2c_client *client;
	struct work_struct work;
	struct delayed_work delaywork;	/*report second event*/
};

struct dmard06_axis {
	int x;
	int y;
	int z;
};
#endif

