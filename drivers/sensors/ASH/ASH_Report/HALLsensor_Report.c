/* 
 * Copyright (C) 2015 ASUSTek Inc.
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

/********************************/
/* HALL Sensor Report Module */
/******************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/input/ASH.h>
#include "../ASH_log.h"

/******************/
/*Global Variables*/
/*****************/
static struct input_dev *input_dev_hall = NULL;

int HALLsensor_report_register(void)
{
	int ret = 0;

	input_dev_hall = input_allocate_device();     
	if(!input_dev_hall){
		err("Hall Sensor Failed to allocate input event device\n");
		return -ENOMEM;		
	}

	input_dev_hall->name = "ASUS Hallsensor";
	input_dev_hall->phys= "/dev/input/hall_indev";
	input_dev_hall->dev.parent= NULL;
	input_set_capability(input_dev_hall, EV_SW, SW_LID);

	ret = input_register_device(input_dev_hall);
	if (ret) {
		err("Hall Sensor Failed to register input event device\n");
		return -1;		
	}
		
	log("Hall Sensor Input Event registration Success!\n");
	return 0;
}
EXPORT_SYMBOL(HALLsensor_report_register);

void HALLsensor_report_unregister(void)
{	
	input_unregister_device(input_dev_hall);
	input_free_device(input_dev_hall);
}
EXPORT_SYMBOL(HALLsensor_report_unregister);

void hallsensor_report_lid(int lid)
{
	if(lid != HALLSENSOR_REPORT_LID_OPEN &&
		lid != HALLSENSOR_REPORT_LID_CLOSE) {
			err("Hall Sensor Detect Magnetic ERROR.\n");
	}
	
	input_report_switch(input_dev_hall, SW_LID, lid);
      input_sync(input_dev_hall);
}
EXPORT_SYMBOL(hallsensor_report_lid);
