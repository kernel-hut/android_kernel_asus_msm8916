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

/*****************************/
/* IR Sensor Report Module */
/***************************/
#define MODULE_NAME	"report"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include "report.h"
#include "../IRsensor.h"

/******************/
/*Global Variables*/
/*****************/
static struct input_dev *input_dev_als = NULL;
static struct input_dev *input_dev_ps = NULL;

int IRsensor_report_event_register(void)
{
	int ret = 0;
	
	/* Proximity Input event allocate */
	input_dev_ps = input_allocate_device();
	if (!input_dev_ps) {		
		err("Proximity Failed to allocate input event device\n");
		return -ENOMEM;
	}

	/* Set Proximity input device */
	input_dev_ps->name = "ASUS Proximitysensor";
	input_dev_ps->id.bustype = BUS_I2C;
	input_set_capability(input_dev_ps, EV_ABS, ABS_DISTANCE);
	__set_bit(EV_ABS, input_dev_ps->evbit);
	__set_bit(ABS_DISTANCE, input_dev_ps->absbit);
	input_set_abs_params(input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);
	//input_set_drvdata(input_dev_ps, g_ps_data);

	/* Register Proximity input device */
	ret = input_register_device(input_dev_ps);
	if (ret < 0) {
		err("Proximity Failed to register input event device\n");
		return -1;
	}

	/* Light Sensor Input event allocate */
	input_dev_als = input_allocate_device();
	if (!input_dev_als) {
		err("Light Sensor Failed to allocate input event device\n");
		return -ENOMEM;
	}

	/* Set Light Sensor input device */
	input_dev_als->name = "ASUS Lightsensor";
	input_dev_als->id.bustype = BUS_I2C;
	input_set_capability(input_dev_als, EV_ABS, ABS_MISC);
	__set_bit(EV_ABS, input_dev_als->evbit);
	__set_bit(ABS_MISC, input_dev_als->absbit);
	input_set_abs_params(input_dev_als, ABS_MISC, 0, 65535, 0, 0);
	//input_set_drvdata(input_dev_als, g_als_data);

	/* Register Light Sensor input device */
	ret = input_register_device(input_dev_als);
	if (ret < 0) {
		err("Light Sensor Failed to register input event device\n");
		return -1;
	}
	
	return 0;
}

void IRsensor_report_event_unregister(void)
{	
	input_unregister_device(input_dev_als);
	input_free_device(input_dev_als);
	input_unregister_device(input_dev_ps);
	input_free_device(input_dev_ps);	
}

void proximity_report_abs(int abs)
{

	if(abs == IRSENSOR_INT_PS_AWAY) {
		input_report_abs(input_dev_ps, ABS_DISTANCE, IRSENSOR_REPORT_PS_AWAY);
		input_sync(input_dev_ps);
		dbg("Proximity Detect Object Away.\n");
	} else if (abs == IRSENSOR_INT_PS_CLOSE) {
		input_report_abs(input_dev_ps, ABS_DISTANCE, IRSENSOR_REPORT_PS_CLOSE);
		input_sync(input_dev_ps);
		dbg("Proximity Detect Object Close.\n");
	} else {
		err("Proximity Detect Object ERROR.\n");
	}
}

void light_report_lux(int lux)
{
	input_report_abs(input_dev_als, ABS_MISC, lux);
	input_event(input_dev_als, EV_SYN, SYN_REPORT, 1);
	input_sync(input_dev_als);
}

