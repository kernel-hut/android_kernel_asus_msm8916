/* 
 * Copyright (C) 2014 ASUSTek Inc.
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
 */
 
/***************************/
/* IR Sensor sysfs Module */
/**************************/
#ifndef __LINUX_IRSENSOR_SYSFS_H
#define __LINUX_IRSENSOR_SYSFS_H

/*Return value is NOT negative (>=0) if it is success reading from file(.nv)*/
extern int 		proximity_sysfs_read_high(void);
extern int 		proximity_sysfs_read_low(void);
/*Return value is TRUE if it is success writing value to file(.nv)*/
extern bool 	proximity_sysfs_write_high(int calvalue);
extern bool 	proximity_sysfs_write_low(int calvalue);
/*Return value is NOT negative (>=0) if it is success reading from file(.nv)*/
extern int 		light_sysfs_read_200lux(void);
extern int 		light_sysfs_read_1000lux(void);
/*Return value is TRUE if it is success writing value to file(.nv)*/
extern bool 	light_sysfs_write_200lux(int calvalue);
extern bool 	light_sysfs_write_1000lux(int calvalue);

#define LSENSOR_200LUX_CALIBRATION_FILE	"/factory/lsensor_200lux.nv"
#define LSENSOR_1000LUX_CALIBRATION_FILE	"/factory/lsensor_1000lux.nv"

#define PSENSOR_HI_CALIBRATION_FILE  		"/factory/psensor_hi.nv"
#define PSENSOR_LOW_CALIBRATION_FILE  	"/factory/psensor_low.nv"

#endif

