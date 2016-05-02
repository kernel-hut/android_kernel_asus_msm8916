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
 
#ifndef __LINUX_IRSENSOR_H
#define __LINUX_IRSENSOR_H

#define DRIVER_NAME	"IRsensor"
#define DRIVER_VERSION "2.1"

/*************************/
/* Debug Switch System */
/************************/
#undef dbg
#ifdef IR_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s] "fmt,DRIVER_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif

#define log(fmt, args...) printk(KERN_INFO "[%s] "fmt,DRIVER_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s] "fmt,DRIVER_NAME,##args)

/****************************/
/* IR Sensor Configuration */
/**************************/
#define IRSENSOR_INT_PS_CLOSE 				(1)
#define IRSENSOR_INT_PS_AWAY     			(2) 
#define IRSENSOR_INT_PS_MASK				(3<< 0)
#define IRSENSOR_INT_ALS           				(4)
#define IRSENSOR_INT_ALS_MASK				(1<< 2)

#define LIGHT_CALVALUE_200LUX_DEFAULT	(200)
#define LIGHT_CALVALUE_1000LUX_DEFAULT	(1000)
#define LIGHT_CALVALUE_SHIFT_DEFAULT	(40)
#define LIGHT_GAIN_ACCURACY_CALVALUE	(100000)
#define LIGHT_CHANGE_LOW_SENSITIVITY 	(10)
#define LIGHT_CHANGE_MID_SENSITIVITY 	(5)
#define LIGHT_CHANGE_HI_SENSITIVITY 		(2)
#define LIGHT_CHANGE_FACTORY_SENSITIVITY (0)
#define LIGHT_CHANGE_MIN_SENSITIVITY 	(0)
#define LIGHT_MAX_LUX							(20000)
#define LIGHT_TURNON_DELAY_TIME			(250)

#define IRSENSOR_DEFAULT_VALUE				(-1)

#define LIGHT_LOG_THRESHOLD					(100)

/*******************************/
/* IR Sensor Driver Interface */
/*****************************/
/*i2c*/
#include <linux/i2c.h>
extern int IRsensor_probe(struct i2c_client *client, const struct i2c_device_id *id);
extern int IRsensor_remove(struct i2c_client *client);
extern void IRsensor_shutdown(struct i2c_client *client);
extern int IRsensor_suspend(struct i2c_client *client, pm_message_t mesg);
extern int IRsensor_resume(struct i2c_client *client);
extern bool IRsensor_check_probe(void);

/*gpio*/
#include <linux/irq.h>
extern irqreturn_t IRsensor_irq_handler(int irq, void *dev_id);

/*Calibration Interface*/
#include <linux/device.h>
extern ssize_t proximity_show_calibration_hi(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t proximity_store_calibration_hi(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t proximity_show_calibration_lo(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t proximity_store_calibration_lo(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t light_show_calibration_200lux(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t light_store_calibration_200lux(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t light_show_calibration_1000lux(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t light_store_calibration_1000lux(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t light_show_adc(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t proximity_show_adc (struct device *dev, 
	struct device_attribute *attr, char *buf);

/*ATD / HAL / Debug / Info Interface*/
extern ssize_t  IRsensor_show_allreg(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  IRsensor_show_vendor(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  IRsensor_show_module_number(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  IRsensor_show_version(struct device *dev, 
	struct device_attribute *attr, char *buf);

extern ssize_t  proximity_show_switch_onoff(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  proximity_store_switch_onoff(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t  proximity_show_atd_test(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  proximity_show_polling_mode(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  proximity_store_polling_mode(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t  proximity_show_led_current(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  proximity_store_led_current(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t  proximity_show_led_duty_ratio(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  proximity_store_led_duty_ratio(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t  proximity_show_persistence(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  proximity_store_persistence(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t  proximity_show_integration(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  proximity_store_integration(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count);

extern ssize_t  light_show_switch_onoff(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  light_store_switch_onoff(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t  light_show_atd_test(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  light_show_lux(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  light_show_shift(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  light_show_gain(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  light_show_sensitivity(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  light_store_sensitivity(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t  light_show_persistence(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  light_store_persistence(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t  light_show_integration(struct device *dev, 
	struct device_attribute *attr, char *buf);
extern ssize_t  light_store_integration(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count);

#endif
