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

 /************************/
/* IR Sensor Atrribute */
/**********************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/input/ASH.h>

#define BUF_SIZE	(10)
IRsensor_ATTR *g_IR_ATTR = NULL;
struct device *g_psensor_dev;
struct device *g_lsensor_dev;

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_ATTR"
#define SENSOR_TYPE_NAME		"IRsensor"


#undef dbg
#ifdef ASH_ATTR_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

ssize_t  ATT_IRsensor_show_vendor(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	if(strcmp(g_IR_ATTR->info_type->vendor, "") == 0) {
		err("Show vendor NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	return sprintf(buf, "%s\n", g_IR_ATTR->info_type->vendor);
}

ssize_t  ATT_IRsensor_show_module_number(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	if(strcmp(g_IR_ATTR->info_type->module_number, "") == 0) {
		err("Show module number NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	return sprintf(buf, "%s\n", g_IR_ATTR->info_type->module_number);
}

/**********************/
/*Calibration Function*/
/*********************/
ssize_t ATT_proximity_show_calibration_hi(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int calvalue;

	if(g_IR_ATTR->ATTR_Calibration->proximity_show_calibration_hi== NULL) {
		err("proximity_show_calibration_hi NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	calvalue = g_IR_ATTR->ATTR_Calibration->proximity_show_calibration_hi();
	dbg("Proximity show High Calibration: %d\n", calvalue);
	return sprintf(buf, "%d\n", calvalue);
}

ssize_t ATT_proximity_store_calibration_hi(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long calvalue;
	
	if(g_IR_ATTR->ATTR_Calibration->proximity_store_calibration_hi == NULL) {
		err("proximity_store_calibration_hi NOT SUPPORT. \n");
		return count;
	}
	
	if ((strict_strtoul(buf, 10, &calvalue) < 0))
		return -EINVAL;		
	if(calvalue < 0) {
		err("Proximity store High Calibration with NEGATIVE value. (%lu) \n", calvalue);
		return -EINVAL;	
	}	
	if(g_IR_ATTR->ATTR_Calibration->proximity_store_calibration_hi(calvalue) < 0)
		return -EINVAL;
	log("Proximity store High Calibration: %lu\n", calvalue);
	
	return count;
}

ssize_t ATT_proximity_show_calibration_lo(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int calvalue;

	if(g_IR_ATTR->ATTR_Calibration->proximity_show_calibration_lo == NULL) {
		err("proximity_show_calibration_lo NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	calvalue = g_IR_ATTR->ATTR_Calibration->proximity_show_calibration_lo();
	dbg("Proximity show Low Calibration: %d\n", calvalue);
	return sprintf(buf, "%d\n", calvalue);
}

ssize_t ATT_proximity_store_calibration_lo(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long calvalue;

	if(g_IR_ATTR->ATTR_Calibration->proximity_store_calibration_lo == NULL) {
		err("proximity_store_calibration_lo NOT SUPPORT. \n");
		return count;
	}
	
	if ((strict_strtoul(buf, 10, &calvalue) < 0))
		return -EINVAL;	
	if(calvalue < 0) {
		err("Proximity store Low Calibration with NEGATIVE value. (%lu) \n", calvalue);
		return -EINVAL;
	}	
	if(g_IR_ATTR->ATTR_Calibration->proximity_store_calibration_lo(calvalue) < 0)
		return -EINVAL;
	log("Proximity store Low Calibration: %lu\n", calvalue);
	
	return count;
}

ssize_t ATT_light_show_calibration_200lux(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int calvalue;

	if(g_IR_ATTR->ATTR_Calibration->light_show_calibration_200lux == NULL) {
		err("light_show_calibration_200lux NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	calvalue = g_IR_ATTR->ATTR_Calibration->light_show_calibration_200lux();
	dbg("Light Sensor show 200 lux Calibration: %d\n", calvalue);
	return sprintf(buf, "%d\n", calvalue);
}

ssize_t ATT_light_store_calibration_200lux(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long calvalue;

	if(g_IR_ATTR->ATTR_Calibration->light_store_calibration_200lux == NULL) {
		err("light_store_calibration_200lux NOT SUPPORT. \n");
		return count;
	}
	
	if ((strict_strtoul(buf, 10, &calvalue) < 0))
		return -EINVAL;	
	if(calvalue < 0) {
		err("Light Sensor store 200 lux Calibration with NEGATIVE value. (%lu) \n", calvalue);
		return -EINVAL;
	}
	if(g_IR_ATTR->ATTR_Calibration->light_store_calibration_200lux(calvalue) < 0)
		return -EINVAL;
	log("Light Sensor store 200 lux Calibration: %lu\n", calvalue);		
		
	return count;
}

ssize_t ATT_light_show_calibration_1000lux(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int calvalue;
	
	if(g_IR_ATTR->ATTR_Calibration->light_show_calibration_1000lux == NULL) {
		err("light_show_calibration_1000lux NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	calvalue = g_IR_ATTR->ATTR_Calibration->light_show_calibration_1000lux();
	dbg("Light Sensor show 1000 lux Calibration: %d\n", calvalue);
	return sprintf(buf, "%d\n", calvalue);
}	

ssize_t ATT_light_store_calibration_1000lux(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long calvalue;

	if(g_IR_ATTR->ATTR_Calibration->light_store_calibration_1000lux == NULL) {
		err("light_store_calibration_1000lux NOT SUPPORT. \n");
		return count;
	}
	
	if ((strict_strtoul(buf, 10, &calvalue) < 0))
		return -EINVAL;
	if(calvalue < 0) {
		err("Light Sensor store 1000 lux Calibration with NEGATIVE value. (%lu) \n", calvalue);
		return -EINVAL;
	}
	if(g_IR_ATTR->ATTR_Calibration->light_store_calibration_1000lux(calvalue) < 0)
		return -EINVAL;
	log("Light Sensor store 1000 lux Calibration: %lu\n", calvalue);	
			
	return count;
}

ssize_t  ATT_light_show_shift(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int shift = 0;
	if(g_IR_ATTR->ATTR_Calibration->light_show_shift == NULL) {
		err("light_show_shift NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	shift = g_IR_ATTR->ATTR_Calibration->light_show_shift();
	return sprintf(buf, "%d\n", shift);
}

ssize_t  ATT_light_show_gain(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int gainvalue = 0;
	if(g_IR_ATTR->ATTR_Calibration->light_show_gain == NULL) {
		err("light_show_gain NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	gainvalue = g_IR_ATTR->ATTR_Calibration->light_show_gain();
	return sprintf(buf, "%d.%05d\n", 
		gainvalue/LIGHT_GAIN_ACCURACY_CALVALUE, gainvalue%LIGHT_GAIN_ACCURACY_CALVALUE);
}

ssize_t ATT_light_show_adc(struct device *dev, struct device_attribute *attr, char *buf)
{
	int adc = 0;
	if(g_IR_ATTR->ATTR_Calibration->light_show_adc == NULL) {
		err("light_show_adc NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	adc = g_IR_ATTR->ATTR_Calibration->light_show_adc();
	return sprintf(buf, "%d\n", adc);
}

ssize_t ATT_proximity_show_adc(struct device *dev, struct device_attribute *attr, char *buf)
{
	int adc = 0;
	if(g_IR_ATTR->ATTR_Calibration->proximity_show_adc == NULL) {
		err("proximity_show_adc NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	adc = g_IR_ATTR->ATTR_Calibration->proximity_show_adc();
	return sprintf(buf, "%d\n", adc);
}

/******************/
/*BMMI Function*/
/****************/
ssize_t  ATT_proximity_show_atd_test(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool atd_test = false;
	if(g_IR_ATTR->ATTR_BMMI->proximity_show_atd_test== NULL) {
		err("proximity_show_atd_test NOT SUPPORT. \n");
		return sprintf(buf, "%d\n", atd_test);
	}
	atd_test = g_IR_ATTR->ATTR_BMMI->proximity_show_atd_test();
	return sprintf(buf, "%d\n", atd_test);	
}

ssize_t  ATT_light_show_atd_test(struct device *dev, struct device_attribute *attr, char *buf)
{
	bool atd_test = false;
	if(g_IR_ATTR->ATTR_BMMI->light_show_atd_test== NULL) {
		err("light_show_atd_test NOT SUPPORT. \n");
		return sprintf(buf, "%d\n", atd_test);
	}
	atd_test = g_IR_ATTR->ATTR_BMMI->light_show_atd_test();
	return sprintf(buf, "%d\n", atd_test);	
}

/*********************/
/*Hardware Function*/
/********************/
ssize_t  ATT_IRsensor_show_read_reg(struct device *dev, struct device_attribute *attr, char *buf)
{	
	int i2c_reg_addr = 0, i2c_reg_value = 0;
	
	if(g_IR_ATTR->ATTR_Hardware->IRsensor_show_reg== NULL) {
		err("IRsensor_store_reg NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}

	i2c_reg_addr = g_IR_ATTR->ATTR_Hardware->show_reg_addr;
	i2c_reg_value = g_IR_ATTR->ATTR_Hardware->IRsensor_show_reg(i2c_reg_addr);
	
	return sprintf(buf, "%d\n", i2c_reg_value);
}

ssize_t  ATT_IRsensor_store_read_reg(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i2c_reg_addr = 0;
	
	if(g_IR_ATTR->ATTR_Hardware->IRsensor_show_reg== NULL) {
		err("IRsensor_store_reg NOT SUPPORT. \n");
		return count;
	}

	sscanf(buf, "%x", &i2c_reg_addr);
	g_IR_ATTR->ATTR_Hardware->show_reg_addr=i2c_reg_addr;
	
	return count;
}
ssize_t  ATT_IRsensor_store_write_reg(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i2c_reg_addr = 0, i2c_reg_value = 0;
	
	if(g_IR_ATTR->ATTR_Hardware->IRsensor_store_reg== NULL) {
		err("IRsensor_store_reg NOT SUPPORT. \n");
		return count;
	}
	
	sscanf(buf, "%x %d", &i2c_reg_addr, &i2c_reg_value);
	
	if(g_IR_ATTR->ATTR_Hardware->IRsensor_store_reg(i2c_reg_addr, i2c_reg_value) < 0)
		return -EINVAL;
	log("IRsensor_store_reg, addr=%02X, value=%02X\n", i2c_reg_addr, i2c_reg_value);	
	
	return count;
}

/****************/
/*HAL Function*/
/***************/
ssize_t  ATT_proximity_show_switch_onoff(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool bOn;
	if(g_IR_ATTR->ATTR_HAL->proximity_show_switch_onoff== NULL) {
		err("proximity_show_switch_onoff NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	bOn = g_IR_ATTR->ATTR_HAL->proximity_show_switch_onoff();
	if(bOn)
		return sprintf(buf, "on\n");
	else
		return sprintf(buf, "off\n");	
}

ssize_t  ATT_proximity_store_switch_onoff(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	bool bOn;

	if(g_IR_ATTR->ATTR_HAL->proximity_store_switch_onoff == NULL) {
		err("proximity_store_switch_onoff NOT SUPPORT. \n");
		return count;
	}
	
	/*check input character*/
	if (0 == strncmp(buf, "off", 3))
		bOn = false;
	else if (0 == strncmp(buf, "on", 2)) 
		bOn = true;
	else
		return -EINVAL;
	
	if(g_IR_ATTR->ATTR_HAL->proximity_store_switch_onoff(bOn) < 0)
		return -EINVAL;
	log("Proximity switch %s\n", bOn?"on":"off");	
	
	return count;
}

ssize_t ATT_proximity_show_status(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool proximity_status;
	if(g_IR_ATTR->ATTR_HAL->proximity_show_status== NULL) {
		err("proximity_show_status NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	proximity_status = g_IR_ATTR->ATTR_HAL->proximity_show_status();
	if(proximity_status)
		return sprintf(buf, "close\n");
	else
		return sprintf(buf, "away\n");	
}

ssize_t  ATT_light_show_switch_onoff(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool bOn;
	if(g_IR_ATTR->ATTR_HAL->light_show_switch_onoff== NULL) {
		err("light_show_switch_onoff NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	bOn = g_IR_ATTR->ATTR_HAL->light_show_switch_onoff();
	if(bOn)
		return sprintf(buf, "on\n");
	else
		return sprintf(buf, "off\n");	
}

ssize_t  ATT_light_store_switch_onoff(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	bool bOn;

	if(g_IR_ATTR->ATTR_HAL->light_store_switch_onoff == NULL) {
		err("light_store_switch_onoff NOT SUPPORT. \n");
		return count;
	}
	
	/*check input character*/
	if (0 == strncmp(buf, "off", 3))
		bOn = false;
	else if (0 == strncmp(buf, "on", 2)) 
		bOn = true;
	else
		return -EINVAL;
	
	if(g_IR_ATTR->ATTR_HAL->light_store_switch_onoff(bOn) < 0)
		return -EINVAL;
	log("Light Sensor switch %s\n", bOn?"on":"off");	
	
	return count;
}

ssize_t ATT_light_show_lux(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int lux;
	if(g_IR_ATTR->ATTR_HAL->light_show_lux== NULL) {
		err("light_show_lux NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	lux = g_IR_ATTR->ATTR_HAL->light_show_lux();
	return sprintf(buf, "%d\n", lux);
}

/*********************/
/*Extension Function*/
/********************/
ssize_t  ATT_IRsensor_show_allreg(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool ret;
	
	if(g_IR_ATTR->ATTR_Extension->IRsensor_show_allreg== NULL) {
		err("IRsensor_show_allreg NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	ret=g_IR_ATTR->ATTR_Extension->IRsensor_show_allreg();
	return sprintf(buf, "%d\n", ret);
}

ssize_t  ATT_proximity_show_polling_mode(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	bool bOn;
	if(g_IR_ATTR->ATTR_Extension->proximity_show_polling_mode == NULL) {
		err("proximity_show_polling_mode NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	bOn = g_IR_ATTR->ATTR_Extension->proximity_show_polling_mode();
	if(bOn)
		return sprintf(buf, "on\n");
	else
		return sprintf(buf, "off\n");
}

ssize_t  ATT_proximity_store_polling_mode(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	bool bOn;

	if(g_IR_ATTR->ATTR_Extension->proximity_store_polling_mode== NULL) {
		err("proximity_store_polling_mode NOT SUPPORT. \n");
		return count;
	}
	
	/*check input character*/
	if (0 == strncmp(buf, "off", 3))
		bOn = false;
	else if (0 == strncmp(buf, "on", 2)) 
		bOn = true;
	else
		return -EINVAL;

	if(g_IR_ATTR->ATTR_Extension->proximity_store_polling_mode(bOn) < 0)
		return -EINVAL;
	log("Proximity polling mode %s\n", bOn?"on":"off");	
	
	return count;	
}

ssize_t  ATT_light_show_sensitivity(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int sensitivity = 0;
	if(g_IR_ATTR->ATTR_Extension->light_show_sensitivity == NULL) {
		err("light_show_sensitivity NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	sensitivity = g_IR_ATTR->ATTR_Extension->light_show_sensitivity();
	return sprintf(buf, "%d\n", sensitivity);
}

ssize_t  ATT_light_store_sensitivity(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long sensitivity;	

	if(g_IR_ATTR->ATTR_Extension->light_store_sensitivity == NULL) {
		err("light_store_sensitivity NOT SUPPORT. \n");
		return count;
	}
	
	if ((strict_strtoul(buf, 10, &sensitivity) < 0))
		return -EINVAL;
	if(sensitivity < 0) {
		err("Light Sensor store Sensitivity with NEGATIVE value. (%lu) \n", sensitivity);
		return -EINVAL;
	}
	if(g_IR_ATTR->ATTR_Extension->light_store_sensitivity(sensitivity) < 0)
		return -EINVAL;	
	log("Light Sensor store Sensitivity: %lu\n", sensitivity);
	
	return count;
}

ssize_t  ATT_light_show_log_threshold(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int log_threshold = 0;
	if(g_IR_ATTR->ATTR_Extension->light_show_log_threshold == NULL) {
		err("light_show_log_threshold NOT SUPPORT. \n");
		return sprintf(buf, "NOT SUPPORT\n");
	}
	
	log_threshold = g_IR_ATTR->ATTR_Extension->light_show_log_threshold();
	return sprintf(buf, "%d\n", log_threshold);
}

ssize_t  ATT_light_store_log_threshold(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long log_threshold;	

	if(g_IR_ATTR->ATTR_Extension->light_store_log_threshold == NULL) {
		err("light_store_log_threshold NOT SUPPORT. \n");
		return count;
	}
	
	if ((strict_strtoul(buf, 10, &log_threshold) < 0))
		return -EINVAL;
	if(log_threshold < 0) {
		err("Light Sensor store Log Threshold with NEGATIVE value. (%lu) \n", log_threshold);
		return -EINVAL;
	}
	if(g_IR_ATTR->ATTR_Extension->light_store_log_threshold(log_threshold) < 0)
		return -EINVAL;	
	log("Light Sensor store Log Threshold: %lu\n", log_threshold);
	
	return count;
}


static struct device_attribute proximity_property_attrs[] = {
	/*read only*/
	__ATTR(vendor, 0444, ATT_IRsensor_show_vendor, NULL),
	__ATTR(module_number, 0444, ATT_IRsensor_show_module_number, NULL),
	__ATTR(proxm, 0444, ATT_proximity_show_adc, NULL),
	__ATTR(atd_status, 0444, ATT_proximity_show_atd_test, NULL),
	__ATTR(proxm_status, 0444, ATT_proximity_show_status, NULL),	
	__ATTR(dump_reg, 0444, ATT_IRsensor_show_allreg, NULL),
	/*read/write*/
	__ATTR(switch, 0664, ATT_proximity_show_switch_onoff, ATT_proximity_store_switch_onoff),
	__ATTR(hi_cal, 0664, ATT_proximity_show_calibration_hi, ATT_proximity_store_calibration_hi),
	__ATTR(low_cal, 0664, ATT_proximity_show_calibration_lo, ATT_proximity_store_calibration_lo),
	__ATTR(poll_mode, 0664, ATT_proximity_show_polling_mode, ATT_proximity_store_polling_mode),
	__ATTR(read_reg, 0664, ATT_IRsensor_show_read_reg, ATT_IRsensor_store_read_reg),
	__ATTR(write_reg, 0220, NULL, ATT_IRsensor_store_write_reg),
};

static struct device_attribute light_property_attrs[] = {
	/*read only*/
	__ATTR(vendor, 0444, ATT_IRsensor_show_vendor, NULL),
	__ATTR(module_number, 0444, ATT_IRsensor_show_module_number, NULL),
	__ATTR(adc, 0444, ATT_light_show_adc, NULL),	
	__ATTR(shift, 0444, ATT_light_show_shift, NULL),
	__ATTR(gain, 0444, ATT_light_show_gain, NULL),
	__ATTR(atd_status, 0444, ATT_light_show_atd_test, NULL),
	__ATTR(lux, 0444, ATT_light_show_lux, NULL),
	__ATTR(dump_reg, 0444, ATT_IRsensor_show_allreg, NULL),
	/*read/write*/
	__ATTR(switch, 0664, ATT_light_show_switch_onoff, ATT_light_store_switch_onoff),
	__ATTR(200lux_cal, 0664, ATT_light_show_calibration_200lux, ATT_light_store_calibration_200lux),
	__ATTR(1000lux_cal, 0664, ATT_light_show_calibration_1000lux, ATT_light_store_calibration_1000lux),
	__ATTR(sensitivity, 0664, ATT_light_show_sensitivity, ATT_light_store_sensitivity),
	__ATTR(read_reg, 0664, ATT_IRsensor_show_read_reg, ATT_IRsensor_store_read_reg),
	__ATTR(write_reg, 0220, NULL, ATT_IRsensor_store_write_reg),
	__ATTR(log_threshold, 0664, ATT_light_show_log_threshold, ATT_light_store_log_threshold),
};

int IRsensor_ATTR_register(IRsensor_ATTR *mATTR)
{
	int ret = 0;
	int ATTR_index;
	
	g_IR_ATTR=mATTR;
	
	/* psensor device */
	g_psensor_dev = ASH_ATTR_device_create(psensor);
	if (IS_ERR(g_psensor_dev) || g_psensor_dev == NULL) {
		ret = PTR_ERR(g_psensor_dev);
		err("%s: psensor create ERROR.\n", __FUNCTION__);
		return ret;
	}	
	for (ATTR_index=0; ATTR_index < ARRAY_SIZE(proximity_property_attrs); ATTR_index++) {
		ret = device_create_file(g_psensor_dev, &proximity_property_attrs[ATTR_index]);
		if (ret)
			return ret;
	}

	/*lsensor device*/
	g_lsensor_dev = ASH_ATTR_device_create(lsensor);
	if (IS_ERR(g_lsensor_dev) || g_lsensor_dev == NULL) {
		err("%s: lsensor create ERROR.\n", __FUNCTION__);
		ret = PTR_ERR(g_lsensor_dev);
		return ret;
	}
	for (ATTR_index=0; ATTR_index < ARRAY_SIZE(light_property_attrs); ATTR_index++) {
		ret = device_create_file(g_lsensor_dev, &light_property_attrs[ATTR_index]);
		if (ret){
			return ret;
		}
	}
	
	return 0;
}
EXPORT_SYMBOL(IRsensor_ATTR_register);

int IRsensor_ATTR_unregister(void)
{
	ASH_ATTR_device_remove(psensor);
	ASH_ATTR_device_remove(lsensor);
	return 0;
}
EXPORT_SYMBOL(IRsensor_ATTR_unregister);

int psensor_ATTR_create(struct device_attribute *mpsensor_attr)
{
	int ret = 0;
	if(mpsensor_attr == NULL) {
		err("%s: the device_attribute is NULL point. \n", __FUNCTION__);
		return -EINVAL;
	}
	ret = device_create_file(g_psensor_dev, mpsensor_attr);
	if (ret){		
		err("%s: device_create_file ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(psensor_ATTR_create);

int lsensor_ATTR_create(struct device_attribute *mlsensor_attr)
{
	int ret = 0;
	if(mlsensor_attr == NULL) {
		err("%s: the device_attribute is NULL point. \n", __FUNCTION__);
		return -EINVAL;
	}
	ret = device_create_file(g_lsensor_dev, mlsensor_attr);
	if (ret){		
		err("%s: device_create_file ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(lsensor_ATTR_create);

