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
 
 /******************************/
/* IR Sensor Property Module */
/*****************************/
#define MODULE_NAME	"property"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/input/proximity_class.h>
#include "property.h"
#include "../IRsensor.h"

static int g_devMajor = 0;
static struct class *g_property_class = NULL;

/*******************/
/*Sensor Property */
/******************/
static struct device_attribute proximity_property_attrs[] = {
		/*read only*/
	__ATTR(vendor, 0444, IRsensor_show_vendor, NULL),
	__ATTR(version, 0444, IRsensor_show_version, NULL),
	__ATTR(module_number, 0444, IRsensor_show_module_number, NULL),
	__ATTR(proxm, 0444, proximity_show_adc, NULL),
	__ATTR(atd_status, 0444, proximity_show_atd_test, NULL),
	__ATTR(reg, 0444, IRsensor_show_allreg, NULL),
		/*read/write*/
	__ATTR(switch, 0664, proximity_show_switch_onoff, proximity_store_switch_onoff),
	__ATTR(hi_cal, 0664, proximity_show_calibration_hi, proximity_store_calibration_hi),
	__ATTR(low_cal, 0664, proximity_show_calibration_lo, proximity_store_calibration_lo),
	__ATTR(poll_mode, 0664, proximity_show_polling_mode, proximity_store_polling_mode),
	__ATTR(led_current, 0664, proximity_show_led_current, proximity_store_led_current),
	__ATTR(led_duty_ratio, 0664, proximity_show_led_duty_ratio, proximity_store_led_duty_ratio),
	__ATTR(persistence, 0664, proximity_show_persistence, proximity_store_persistence),
	__ATTR(integration, 0664, proximity_show_integration, proximity_store_integration),
};

static struct device_attribute light_property_attrs[] = {
	/*read only*/
	__ATTR(vendor, 0444, IRsensor_show_vendor, NULL),
	__ATTR(version, 0444, IRsensor_show_version, NULL),
	__ATTR(module_number, 0444, IRsensor_show_module_number, NULL),
	__ATTR(adc, 0444, light_show_adc, NULL),
	__ATTR(lux, 0444, light_show_lux, NULL),
	__ATTR(shift, 0444, light_show_shift, NULL),
	__ATTR(gain, 0444, light_show_gain, NULL),
	__ATTR(atd_status, 0444, light_show_atd_test, NULL),
	__ATTR(reg, 0444, IRsensor_show_allreg, NULL),
		/*read/write*/
	__ATTR(switch, 0664, light_show_switch_onoff, light_store_switch_onoff),
	__ATTR(200lux_cal, 0664, light_show_calibration_200lux, light_store_calibration_200lux),
	__ATTR(1000lux_cal, 0664, light_show_calibration_1000lux, light_store_calibration_1000lux),
	__ATTR(sensitivity, 0664, light_show_sensitivity, light_store_sensitivity),
	__ATTR(persistence, 0664, light_show_persistence, light_store_persistence),
	__ATTR(integration, 0664, light_show_integration, light_store_integration),
};	

static int property_class_open(struct inode * inode, struct file * file)
{
    return 0;
}

static const struct file_operations property_class_fops = {
    .owner      = THIS_MODULE,
    .open       = property_class_open,
};

static int create_property_class(void)
{
	int ret = 0;	
	dev_t dev;
	struct device *psensor_dev;
	struct device *lsensor_dev;
	int property_index;

	/*create character device*/
	g_devMajor = register_chrdev(0, "sensors", &property_class_fops);
	if (g_devMajor < 0) {
		printk("Property_class: could not get major number\n");
		return g_devMajor;
	}

	/* create sys/class */
	g_property_class = class_create(THIS_MODULE, "sensors");
	if (IS_ERR(g_property_class)) {
		return PTR_ERR(g_property_class);
	}		

	/* psensor device */
	dev = MKDEV(g_devMajor, 0);
	psensor_dev = device_create(g_property_class, NULL, dev, NULL, "%s", "psensor");
	if (IS_ERR(psensor_dev)) {
		ret = PTR_ERR(psensor_dev);
		return ret;
	}	
	for (property_index=0; property_index < ARRAY_SIZE(proximity_property_attrs); property_index++) {
		ret = device_create_file(psensor_dev, &proximity_property_attrs[property_index]);
		if (ret)
			return ret;
	}

	/*lsensor device*/
	dev = MKDEV(g_devMajor, 1);
	lsensor_dev = device_create(g_property_class, NULL, dev, NULL, "%s", "lsensor");
	if (IS_ERR(lsensor_dev)) {
		ret = PTR_ERR(lsensor_dev);
		return ret;
	}
	for (property_index=0; property_index < ARRAY_SIZE(light_property_attrs); property_index++) {
		ret = device_create_file(lsensor_dev, &light_property_attrs[property_index]);
		if (ret)
			return ret;
	}
	
	return 0;
}


int IRsensor_property_register(void)
{
	int ret = 0;
		
	ret = create_property_class();
	if (ret < 0)
		return ret;
	
	return 0;
}

int IRsensor_property_unregister(void)
{
	class_destroy(g_property_class);
	unregister_chrdev(0, "sensors");
	return 0;
}

