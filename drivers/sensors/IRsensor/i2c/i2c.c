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
#define MODULE_NAME	"i2c"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>

#include "../IRsensor.h"
#include "i2c.h"
#include "../hardware/hardware.h"

static struct i2c_driver* IRsensor_i2c_driver;

static struct i2c_driver IRsensor_i2c_driver_client = {
	.probe = IRsensor_probe,
	.remove = IRsensor_remove,
	.shutdown = IRsensor_shutdown,
	.suspend = IRsensor_suspend,
	.resume = IRsensor_resume,
};

int IRsensor_i2c_register(int hardware_source)
{
	int err = 0;	
	
	/* i2c Registration */	
	IRsensor_i2c_driver = &IRsensor_i2c_driver_client;
	IRsensor_i2c_driver = IRsensor_hw_setI2cDriver(hardware_source, IRsensor_i2c_driver);
	err = i2c_add_driver(IRsensor_i2c_driver);
	if ( err != 0 ) {
		err("i2c_driver_register fail, Error : %d\n",err);	
		return err;
	}		
	
	/*check the i2c probe*/
	if(IRsensor_check_probe() == false) {
		i2c_del_driver(IRsensor_i2c_driver);
		return -1;
	}
	
	return 0;
}

int IRsensor_i2c_unregister(void)
{		
	i2c_del_driver(IRsensor_i2c_driver);
	return 0;
}
