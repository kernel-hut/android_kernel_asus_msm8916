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
 
/**************************/
/* Hall Sensor Attribute */
/************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/input/ASH.h>
#include "../ASH_log.h"

HALLsensor_ATTR *g_hall_ATTR = NULL;
struct device *g_hallsensor_dev;

static ssize_t show_action_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	bool action_status = false;

	if(g_hall_ATTR->show_action_status == NULL) {
		err("show_action_status NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}

	action_status=g_hall_ATTR->show_action_status();
	
	return sprintf(buf, "%d\n", action_status);
}

static ssize_t show_hall_sensor_enable(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool bOn = false;
	
       if(g_hall_ATTR->show_hall_sensor_enable== NULL) {
		err("show_hall_sensor_enable NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}

	bOn = g_hall_ATTR->show_hall_sensor_enable();
	if(bOn)
		return sprintf(buf, "on\n");
	else
		return sprintf(buf, "off\n");	
}

static ssize_t store_hall_sensor_enable(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	bool bOn = false;
	
	if(g_hall_ATTR->store_hall_sensor_enable == NULL) {
		err("store_hall_sensor_enable NOT SUPPORT. \n");
		return count;
	}

	/*check input character*/
	if (0 == strncmp(buf, "off", 3))
		bOn = false;
	else if (0 == strncmp(buf, "on", 2)) 
		bOn = true;
	else
		return -EINVAL;

	if(g_hall_ATTR->store_hall_sensor_enable(bOn) < 0)
		return -EINVAL;
	log("Hall Sensor switch %s\n", bOn?"on":"off");	
	
	return count;
}

static ssize_t show_hall_sensor_debounce(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	if(g_hall_ATTR->show_hall_sensor_debounce == NULL) {
		err("show_hall_sensor_debounce NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	return sprintf(buf, "%d\n", g_hall_ATTR->show_hall_sensor_debounce());
}

static ssize_t store_hall_sensor_debounce(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	int debounce = 0;

	if(g_hall_ATTR->store_hall_sensor_debounce == NULL) {
		err("store_hall_sensor_debounce NOT SUPPORT. \n");
		return count;
	}
	
	sscanf(buf, "%d", &debounce);
	if(g_hall_ATTR->store_hall_sensor_debounce(debounce) < 0)
		return -EINVAL;
	log("Hall Sensor store debounce: %d\n", debounce);	
	
	return count;
}

static struct device_attribute hallsensor_property_attrs[] = {
	/*read only*/
	__ATTR(status, 0440, show_action_status, NULL),
	
	/*read/write*/
	__ATTR(switch, 0660, show_hall_sensor_enable, store_hall_sensor_enable),
	__ATTR(debounce, 0660, show_hall_sensor_debounce, store_hall_sensor_debounce),
};

int HALLsensor_ATTR_register(HALLsensor_ATTR *mATTR)
{
	int ret = 0;
	int ATTR_index;
		
	g_hall_ATTR = mATTR;

	/* psensor device */
	g_hallsensor_dev = ASH_ATTR_device_create(hallsensor);
	if (IS_ERR(g_hallsensor_dev) || g_hallsensor_dev == NULL) {
		ret = PTR_ERR(g_hallsensor_dev);
		err("HALLsensor_ATTR_register : hall sensor create ERROR.\n");
		return ret;
	}	
	for (ATTR_index=0; ATTR_index < ARRAY_SIZE(hallsensor_property_attrs); ATTR_index++) {
		ret = device_create_file(g_hallsensor_dev, &hallsensor_property_attrs[ATTR_index]);
		if (ret)
			return ret;
	}
	
	return 0;
}

int HALLsensor_ATTR_unregister(void)
{
	ASH_ATTR_device_remove(hallsensor);
	return 0;
}

int HALLsensor_ATTR_create(struct device_attribute *mhallsensor_attr)
{
	int ret = 0;
	if(mhallsensor_attr == NULL) {
		err("HALLsensor_ATTR_create : the device_attribute is NULL point. \n");
		return -EINVAL;
	}
	ret = device_create_file(g_hallsensor_dev, mhallsensor_attr);
	if (ret){		
		err("HALLsensor_ATTR_create : hall sensor create customize attribute ERROR. \n");
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(HALLsensor_ATTR_create);

