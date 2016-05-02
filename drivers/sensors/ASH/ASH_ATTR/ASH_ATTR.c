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
 
 /******************************/
/* Asus Sensor Hub Attribute */
/*****************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/input/ASH.h>
#include "../ASH_log.h"

static int g_devMajor = -1;
static struct class *g_property_class = NULL;

static int ASH_class_open(struct inode * inode, struct file * file)
{
    return 0;
}

static const struct file_operations ASH_class_fops = {
    .owner      = THIS_MODULE,
    .open       = ASH_class_open,
};

static int create_ASH_chrdev(void)
{
	/*create character device*/
	g_devMajor = register_chrdev(0, "ASH", &ASH_class_fops);
	if (g_devMajor < 0) {
		err("create_ASH_chrdev : could not get major number\n");
		return g_devMajor;
	}
	
	return 0;
}

static int create_ASH_class(void)
{
	/* create sys/class file node*/
	g_property_class = class_create(THIS_MODULE, "ASH");
	if (IS_ERR(g_property_class)) {
		err("create_ASH_class : could not get class\n");
		return PTR_ERR(g_property_class);
	}
	
	return 0;
}

/*
 Return NULL for error handling
*/
struct device *ASH_ATTR_device_create(ASH_type type)
{	
	int ret = 0;
	dev_t dev;
	struct device *sensor_dev=NULL;

	/*prepare character device and return for error handling*/
	if(g_devMajor < 0){
		ret=create_ASH_chrdev();
		if(ret < 0)
			return NULL;
	}	

	/*prepare sys/class file node and return for error handling*/
	if(g_property_class == NULL || IS_ERR(g_property_class) ){
		ret=create_ASH_class();
		if(ret < 0)
			return NULL;
	}

	switch(type){
		case psensor:
			dev = MKDEV(g_devMajor, psensor);
			sensor_dev = device_create(g_property_class, NULL, dev, NULL, "%s", "psensor");
			break;
		case lsensor:
			dev = MKDEV(g_devMajor, lsensor);
			sensor_dev = device_create(g_property_class, NULL, dev, NULL, "%s", "lsensor");
			break;
		case hallsensor:
			dev = MKDEV(g_devMajor, hallsensor);
			sensor_dev = device_create(g_property_class, NULL, dev, NULL, "%s", "hallsensor");
			break;
		default:
			err("ASH_Interface_device_create Type ERROR.(%d)\n", type);

	}
	
	if (IS_ERR(sensor_dev)) {
		ret = PTR_ERR(sensor_dev);
		err("sensor_dev pointer is ERROR.(%d)\n", ret);		
		return NULL;
	}	
	
	return sensor_dev;
}
EXPORT_SYMBOL(ASH_ATTR_device_create);

void ASH_ATTR_device_remove(ASH_type type)
{
	dev_t dev;
	
	switch(type){
		case psensor:
			dev = MKDEV(g_devMajor, psensor);
			device_destroy(g_property_class, dev);
			break;
		case lsensor:
			dev = MKDEV(g_devMajor, lsensor);
			device_destroy(g_property_class, dev);
			break;
		case hallsensor:
			dev = MKDEV(g_devMajor, hallsensor);
			device_destroy(g_property_class, dev);
			break;
		default:
			err("ASH_Interface_remove Type ERROR.(%d)\n", type);

	}

}
EXPORT_SYMBOL(ASH_ATTR_device_remove);
