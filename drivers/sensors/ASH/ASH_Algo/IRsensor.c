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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/input/ASH.h>
#include "IRsensor.h"

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_ALGO"
#define SENSOR_TYPE_NAME		"IRsensor"

#undef dbg
#ifdef ASH_ALGO_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

/******************************/
/* IR Sensor Global Variables */
/*****************************/
static int ASUS_IR_SENSOR_IRQ;
static struct ASUS_light_sensor_data			*g_als_data;
static struct ASUS_proximity_sensor_data	*g_ps_data;
static struct IRsensor_hw						*IRsensor_hw_client;
static struct workqueue_struct 					*IRsensor_workqueue;
static struct workqueue_struct 					*IRsensor_delay_workqueue;
static struct mutex 								g_ir_lock;
static struct wake_lock 							g_ir_wake_lock;

static int g_als_last_lux = 0;

/***********************/
/* IR Sensor Functions*/
/**********************/
/*Device Layer Part*/
static int 	proximity_turn_onoff(bool bOn);
static int 	proximity_set_threshold(void);
static void proximity_polling_adc(struct work_struct *work);
static int 	light_turn_onoff(bool bOn);
static int 	light_get_lux(int adc);
static int 	light_get_shift(void);
static int 	light_get_accuracy_gain(void);
static void light_polling_lux(struct work_struct *work);

/*Interrupt Service Routine Part*/
static void IRsensor_ist(struct work_struct *work);

/* Export Functions */
bool proximity_check_status(void);

/*Initialization Part*/
static int init_data(void);

/*Work Queue*/
static 		DECLARE_WORK(IRsensor_ist_work, IRsensor_ist);
static 		DECLARE_DELAYED_WORK(proximity_polling_adc_work, proximity_polling_adc);
static 		DECLARE_DELAYED_WORK(light_polling_lux_work, light_polling_lux);

/*******************************/
/* ALS and PS data structure */
/******************************/
struct ASUS_light_sensor_data 
{	
	int g_als_calvalue_200lux;				/* Lightsensor 200lux calibration value(adc) */
	int g_als_calvalue_1000lux;				/* Lightsensor 1000lux calibration value(adc) */
	int g_als_calvalue_shift;					/* Lightsensor Shift calibration value */
	int g_als_change_sensitivity;			/* Lightsensor Change sensitivity */
	int g_als_log_threshold;					/* Lightsensor Log Print Threshold */

	bool HAL_switch_on;						/* this var. means if HAL is turning on als or not */
	bool Device_switch_on;					/* this var. means if als hw is turn on or not */
		
};

struct ASUS_proximity_sensor_data 
{	
	int g_ps_calvalue_lo;						/* Proximitysensor setting low calibration value(adc) */
	int g_ps_calvalue_hi;						/* Proximitysensor setting high calibration value(adc) */
	
	bool HAL_switch_on;						/* this var. means if HAL is turning on ps or not */
	bool Device_switch_on;					/* this var. means is turning on ps or not */	
	bool polling_mode;							/* Polling for adc of proximity */
};

/*=======================
 *|| I2c Stress Test Part ||
 *=======================
 */

#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>

#define I2C_TEST_Lsensor_FAIL (-1)
#define I2C_TEST_Psensor_FAIL (-1)

static int IRsensor_I2C_stress_test(struct i2c_client *client)
{
	int lnResult = I2C_TEST_PASS;	
	int ret = 0;
	int adc = 0;
	int low_threshold = 0;
	int high_threshold = 0;

	i2c_log_in_test_case("TestIRSensorI2C ++\n");

	/* Check Hardware Support First */
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold == NULL) {
		err("proximity_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold == NULL) {
		err("proximity_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mlsensor_hw->light_hw_get_adc == NULL) {
		err("light_hw_get_adc NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mlsensor_hw->light_hw_set_hi_threshold == NULL) {
		err("light_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mlsensor_hw->light_hw_set_lo_threshold == NULL) {
		err("light_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	/* Turn on Proximity and Light Sensor */
	if(!g_ps_data->Device_switch_on) {
		proximity_turn_onoff(true);
	}
	if(!g_als_data->Device_switch_on) {
		light_turn_onoff(true);
	}

	/* Proximity i2c read test */
	ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to get adc\n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}

	/* Proximity i2c write test */
	ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold(g_ps_data->g_ps_calvalue_hi);
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to set high threshold.\n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}
	ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold(g_ps_data->g_ps_calvalue_lo);
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to set low threshold. \n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}

	/* Light Sensor i2c read test */
	adc = IRsensor_hw_client->mlsensor_hw->light_hw_get_adc();
	if(adc < 0){
		i2c_log_in_test_case("IRsensor Light Sensor Fail to get adc\n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}
	/* Light Sensor Low Threshold */	
	low_threshold = adc * (100 - LIGHT_CHANGE_MID_SENSITIVITY) / 100;

	/* Light Sensor High Threshold */
	high_threshold = adc * (100 + LIGHT_CHANGE_MID_SENSITIVITY) / 100;	
	if (high_threshold > IRsensor_hw_client->mlsensor_hw->light_max_threshold)	
		high_threshold = IRsensor_hw_client->mlsensor_hw->light_max_threshold;	
	
	ret = IRsensor_hw_client->mlsensor_hw->light_hw_set_hi_threshold(high_threshold);
	if(ret < 0) {
		i2c_log_in_test_case("IRsensor Light Sensor Fail to set high threshold. \n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}
	
	ret = IRsensor_hw_client->mlsensor_hw->light_hw_set_lo_threshold(low_threshold);
	if(ret < 0) {
		i2c_log_in_test_case("IRsensor Light Sensor Fail to set low threshold. \n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}
	
	i2c_log_in_test_case("TestLSensorI2C --\n");
	return lnResult;
}

static struct i2c_test_case_info IRSensorTestCaseInfo[] =	{
	__I2C_STRESS_TEST_CASE_ATTR(IRsensor_I2C_stress_test),
};
#endif

/*====================
 *|| Device Layer Part ||
 *====================
 */ 
static int proximity_turn_onoff(bool bOn)
{
	int ret = 0;	

	/* Check Hardware Support First */
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff == NULL) {
		err("proximity_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	if (bOn == 1)	{	/* power on */
		/*Set Proximity Threshold*/
		ret = proximity_set_threshold();
		if (ret < 0) {		
			return ret;
		}
		/*set turn on register*/
		if(g_ps_data->Device_switch_on == false)
			IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);		
		/*enable IRQ only when proximity and light sensor is off*/
		if (g_ps_data->Device_switch_on == false && g_als_data->Device_switch_on == false) {
			dbg("[IRQ] Enable irq !! \n");
			enable_irq(ASUS_IR_SENSOR_IRQ);
		}
		/*change the Device Status*/
		g_ps_data->Device_switch_on = true;
		/*check the polling mode*/
		if(g_ps_data->polling_mode == true) {
			queue_delayed_work(IRsensor_delay_workqueue, &proximity_polling_adc_work, msecs_to_jiffies(1000));
			log("[Polling] Proximity polling adc START. \n");
		}		
	} else	{	/* power off */
		/*set turn off register*/
		if(g_ps_data->Device_switch_on == true)
			IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);
		/*disable IRQ only when proximity is on and light sensor is off*/
		if (g_ps_data->Device_switch_on == true && g_als_data->Device_switch_on == false) {
			dbg("[IRQ] Disable irq !! \n");
			disable_irq_nosync(ASUS_IR_SENSOR_IRQ);
		}
		/*change the Device Status*/
		g_ps_data->Device_switch_on = false;		
	}	
	
	return 0;
}

static int proximity_set_threshold(void)
{
	int ret = 0;	

	/* Check Hardware Support First */
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold == NULL) {
		err("proximity_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold == NULL) {
		err("proximity_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	/*Set Proximity High Threshold*/
	ret = psensor_sysfs_read_high();
	if(ret > 0) {
	    	g_ps_data->g_ps_calvalue_hi = ret;
		log("Proximity read High Calibration : %d\n", g_ps_data->g_ps_calvalue_hi);
	}else{
		log("Proximity read DEFAULT High Calibration : %d\n", g_ps_data->g_ps_calvalue_hi);
	}
	IRsensor_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold(g_ps_data->g_ps_calvalue_hi);
	
	/*Set Proximity Low Threshold*/
	ret = psensor_sysfs_read_low();	
	if(ret > 0) {
	    	g_ps_data->g_ps_calvalue_lo = ret;
		log("Proximity read Low Calibration : %d\n", g_ps_data->g_ps_calvalue_lo);
	}else{
		log("Proximity read DEFAULT Low Calibration : %d\n", g_ps_data->g_ps_calvalue_lo);
	}
	IRsensor_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold(g_ps_data->g_ps_calvalue_lo);

	return 0;
}

static void proximity_polling_adc(struct work_struct *work)
{
	int adc = 0;

	/* Check Hardware Support First */
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");		
	}
	
	adc= IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
	log("[Polling] Proximity get adc = %d\n", adc);
	
	if(g_ps_data->Device_switch_on == true)
		queue_delayed_work(IRsensor_delay_workqueue, &proximity_polling_adc_work, msecs_to_jiffies(1000));
	else
		log("[Polling] Proximity polling adc STOP. \n");
}

static int light_turn_onoff(bool bOn)
{

	/* Check Hardware Support First */
	if(IRsensor_hw_client->mlsensor_hw->light_hw_turn_onoff == NULL) {
		err("light_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mlsensor_hw->light_hw_set_hi_threshold == NULL) {
		err("light_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mlsensor_hw->light_hw_set_lo_threshold == NULL) {
		err("light_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	if (bOn == 1)	{	/* power on */	
			
		light_get_shift();
		log("[Cal] Light Sensor Set Shift calibration value : %d\n", g_als_data->g_als_calvalue_shift);

		if(g_als_data->Device_switch_on == false) {
			IRsensor_hw_client->mlsensor_hw->light_hw_set_hi_threshold(0);
			IRsensor_hw_client->mlsensor_hw->light_hw_set_lo_threshold(0);
			IRsensor_hw_client->mlsensor_hw->light_hw_turn_onoff(true);
		}
		/*enable IRQ only when proximity and light sensor is off*/
		if (g_ps_data->Device_switch_on == false && g_als_data->Device_switch_on == false) {
			dbg("[IRQ] Enable irq !! \n");
			enable_irq(ASUS_IR_SENSOR_IRQ);
		}
		g_als_data->Device_switch_on = true;		
	} else	{	/* power off */	
		if(g_als_data->Device_switch_on == true)
			IRsensor_hw_client->mlsensor_hw->light_hw_turn_onoff(false);
		/*disable IRQ only when proximity is off and light sensor is on*/
		if (g_ps_data->Device_switch_on == false && g_als_data->Device_switch_on == true) {
			dbg("[IRQ] Disable irq !! \n");
			disable_irq_nosync(ASUS_IR_SENSOR_IRQ);
		}
		g_als_data->Device_switch_on = false;			
	}
	
	return 0;
}

static int light_get_lux(int adc)
{
	int lux = 0;

	if(adc < 0) {
		err("Light Sensor get Lux ERROR. (adc < 0)\n");
		return 0;
	}	
	
	lux = (adc * 800/(g_als_data->g_als_calvalue_1000lux-g_als_data->g_als_calvalue_200lux)
			+ g_als_data->g_als_calvalue_shift);	
	
	if(lux > LIGHT_MAX_LUX)
		lux = LIGHT_MAX_LUX;
	if(adc < 10 || lux < 0)
		lux = 0;
	
	return lux;
}

static int 	light_get_shift(void)
{
	int ret = 0;
	
	/* Light Sensor Read Calibration*/
	ret = lsensor_sysfs_read_200lux();
	if(ret > 0 ) {
		g_als_data->g_als_calvalue_200lux = ret;
		log("Light Sensor read 200lux Calibration: Cal: %d\n", g_als_data->g_als_calvalue_200lux);
	}else{
		log("Light Sensor read DEFAULT 200lux Calibration: Cal: %d\n", g_als_data->g_als_calvalue_200lux);
	}
	
	ret = lsensor_sysfs_read_1000lux();
	if(ret > 0 ) {
		g_als_data->g_als_calvalue_1000lux = ret;
		log("Light Sensor read 1000lux Calibration: Cal: %d\n", g_als_data->g_als_calvalue_1000lux);
	}else{
		log("Light Sensor read DEFAULT 1000lux Calibration: Cal: %d\n", g_als_data->g_als_calvalue_1000lux);
	}
		
	g_als_data->g_als_calvalue_shift = (1000 - g_als_data->g_als_calvalue_1000lux*800/
				(g_als_data->g_als_calvalue_1000lux-g_als_data->g_als_calvalue_200lux));
	return g_als_data->g_als_calvalue_shift;
}

static int 	light_get_accuracy_gain(void)
{
	int ret = 0;
	int gainvalue = 0;

	/* Light Sensor Read Calibration*/
	ret = lsensor_sysfs_read_200lux();
	if(ret > 0 )
		g_als_data->g_als_calvalue_200lux = ret;
	ret = lsensor_sysfs_read_1000lux();
	if(ret > 0 )
		g_als_data->g_als_calvalue_1000lux = ret;
		
	gainvalue = (800*LIGHT_GAIN_ACCURACY_CALVALUE)/
				(g_als_data->g_als_calvalue_1000lux-g_als_data->g_als_calvalue_200lux);
	
	return gainvalue;
}

static void light_polling_lux(struct work_struct *work)
{
	int adc = 0;
	int lux = 0;

	/* Check Hardware Support First */
	if(IRsensor_hw_client->mlsensor_hw->light_hw_get_adc == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");		
	}
	
	/* Light Sensor Report the first real event*/			
	adc = IRsensor_hw_client->mlsensor_hw->light_hw_get_adc();
	lux = light_get_lux(adc);
	log("[Polling] Light Sensor Report lux : %d (adc = %d)\n", lux, adc);
	lsensor_report_lux(lux);	
}

/**********************/
/*IR sensor Info Type*/
/*********************/

static IRsensor_info_type mIRsensor_info_type = {{0}};
	
/**********************/
/*Calibration Function*/
/*********************/
int mproximity_show_calibration_hi(void)
{
	int calvalue;
	calvalue = psensor_sysfs_read_high();	
	dbg("Proximity show High Calibration: %d\n", calvalue);
	return calvalue;
}

int mproximity_store_calibration_hi(int calvalue)
{
	if(calvalue <= 0) {
		err("Proximity store High Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;	
	}
	log("Proximity store High Calibration: %d\n", calvalue);
	psensor_sysfs_write_high(calvalue);
	proximity_set_threshold();
	
	return 0;
}

int mproximity_show_calibration_lo(void)
{
	int calvalue;
	calvalue = psensor_sysfs_read_low();
	dbg("Proximity show Low Calibration: %d\n", calvalue);
	return calvalue;
}

int mproximity_store_calibration_lo(int calvalue)
{
	if(calvalue <= 0) {
		err("Proximity store Low Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Proximity store Low Calibration: %d\n", calvalue);
	psensor_sysfs_write_low(calvalue);
	proximity_set_threshold();	

	return 0;
}

int mlight_show_calibration_200lux(void)
{
	int calvalue;
	calvalue = lsensor_sysfs_read_200lux();	
	dbg("Light Sensor show 200 lux Calibration: %d\n", calvalue);
	return calvalue;
}

int mlight_store_calibration_200lux(int calvalue)
{
	int adc = 0;
	int lux = 0;
	
	if(calvalue <= 0) {
		err("Light Sensor store 200 lux Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Light Sensor store 200 lux Calibration: %d\n", calvalue);
	lsensor_sysfs_write_200lux(calvalue);
	g_als_data->g_als_calvalue_200lux = calvalue;
	light_get_shift();

	/* Light Sensor Report input event*/			
	adc = IRsensor_hw_client->mlsensor_hw->light_hw_get_adc();
	lux = light_get_lux(adc);
	log("[ISR] Light Sensor Report lux : %d (adc = %d)\n", lux, adc);
	lsensor_report_lux(lux);	

	return 0;
}

int mlight_show_calibration_1000lux(void)
{
	int calvalue;
	calvalue = lsensor_sysfs_read_1000lux();	
	dbg("Light Sensor show 1000 lux Calibration: %d\n", calvalue);
	return calvalue;
}	

int mlight_store_calibration_1000lux(int calvalue)
{
	int adc = 0;
	int lux = 0;
	
	if(calvalue <= 0) {
		err("Light Sensor store 1000 lux Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Light Sensor store 1000 lux Calibration: %d\n", calvalue);
	lsensor_sysfs_write_1000lux(calvalue);
	g_als_data->g_als_calvalue_1000lux = calvalue;
	light_get_shift();

	/* Light Sensor Report input event*/			
	adc = IRsensor_hw_client->mlsensor_hw->light_hw_get_adc();
	lux = light_get_lux(adc);
	log("[ISR] Light Sensor Report lux : %d (adc = %d)\n", lux, adc);
	lsensor_report_lux(lux);	
			
	return 0;
}

int mlight_show_shift(void)
{
	int shiftvalue;
	shiftvalue = light_get_shift();
	dbg("Light Sensor show Shift Calibration: %d\n", shiftvalue);
	return shiftvalue;
}

int mlight_show_gain(void)
{
	int gainvalue;
	gainvalue = light_get_accuracy_gain();
	dbg("Light Sensor show Gain Calibration: %d.%d\n",
		gainvalue/LIGHT_GAIN_ACCURACY_CALVALUE, gainvalue%LIGHT_GAIN_ACCURACY_CALVALUE);

	return gainvalue;
}

int mlight_show_adc(void)
{
	int adc = 0;
	if(IRsensor_hw_client->mlsensor_hw->light_hw_get_adc == NULL) {
		err("light_hw_get_adc NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_ir_lock);

	if (!g_als_data->HAL_switch_on) {
		light_turn_onoff(true);
	}
	
	msleep(LIGHT_TURNON_DELAY_TIME);
		
	adc = IRsensor_hw_client->mlsensor_hw->light_hw_get_adc();
	dbg("mlight_show_adc : %d \n", adc);

	if (!g_als_data->HAL_switch_on) {
		light_turn_onoff(false);
	}
	
	mutex_unlock(&g_ir_lock);
	return adc;
}

int mproximity_show_adc(void)
{
	int adc = 0;
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_ir_lock);
	
	if (!g_ps_data->HAL_switch_on) {
		proximity_turn_onoff(true);
	}

	msleep(PROXIMITY_TURNON_DELAY_TIME);
	
	adc = IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
	dbg("mproximity_show_adc : %d \n", adc);

	
	if (!g_ps_data->HAL_switch_on) {
		proximity_turn_onoff(false);
	}
	
	mutex_unlock(&g_ir_lock);
	
	return adc;
}

static IRsensor_ATTR_Calibration mIRsensor_ATTR_Calibration = {
	.proximity_show_calibration_hi = mproximity_show_calibration_hi,
	.proximity_store_calibration_hi = mproximity_store_calibration_hi,
	.proximity_show_calibration_lo = mproximity_show_calibration_lo,
	.proximity_store_calibration_lo = mproximity_store_calibration_lo,
	.light_show_calibration_200lux = mlight_show_calibration_200lux,
	.light_store_calibration_200lux = mlight_store_calibration_200lux,
	.light_show_calibration_1000lux = mlight_show_calibration_1000lux,
	.light_store_calibration_1000lux = mlight_store_calibration_1000lux,
	.light_show_shift = mlight_show_shift,
	.light_show_gain = mlight_show_gain,
	.light_show_adc = mlight_show_adc,
	.proximity_show_adc = mproximity_show_adc,
};

/******************/
/*BMMI Function*/
/****************/
bool mproximity_show_atd_test(void)
{
	int ret=0;
	int round=0;

	ret = IRsensor_hw_client->IRsensor_hw_check_ID();
	if(ret < 0){
		err("Proximity ATD test check ID ERROR\n");
		goto proximity_atd_test_fail;
	}
	
	ret = proximity_turn_onoff(true);
	if(ret < 0){
		err("Proximity ATD test turn on ERROR\n");
		goto proximity_atd_test_fail;
	}	
	
	for(;round<5; round++){
		ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
		if(ret < 0){
			err("Proximity ATD test get adc ERROR\n");
			goto proximity_atd_test_fail;
		}
		msleep(100);
	}	

	ret = proximity_turn_onoff(false);
	if(ret < 0){
		err("Proximity ATD test turn off ERROR\n");
		goto proximity_atd_test_fail;
	}

	return true;
proximity_atd_test_fail:
	return false;
}

bool mlight_show_atd_test(void)
{
	int ret=0;
	int round=0;

	ret = IRsensor_hw_client->IRsensor_hw_check_ID();
	if(ret < 0){
		err("Light Sensor ATD test check ID ERROR\n");
		goto light_atd_test_fail;
	}
	
	ret =light_turn_onoff(true);
	if(ret < 0){
		err("Light Sensor ATD test turn on ERROR\n");
		goto light_atd_test_fail;
	}	
	
	for(; round<5; round++){
		ret = IRsensor_hw_client->mlsensor_hw->light_hw_get_adc();
		if(ret < 0){
			err("Light Sensor ATD test get adc ERROR\n");
			goto light_atd_test_fail;
		}
		msleep(100);
	}	
	
	ret = light_turn_onoff(false);
	if(ret < 0){
		err("Light Sensor ATD test turn off ERROR\n");
		goto light_atd_test_fail;
	}

	return true;
light_atd_test_fail:
	return false;

}

static IRsensor_ATTR_BMMI mIRsensor_ATTR_BMMI = {
	.proximity_show_atd_test = mproximity_show_atd_test,
	.light_show_atd_test = mlight_show_atd_test,
};

/*********************/
/*Hardware Function*/
/********************/
int mIRsensor_show_reg(uint8_t addr)
{
	int value;
	if(IRsensor_hw_client->IRsensor_hw_get_register == NULL) {
		err("IRsensor_hw_get_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	value = IRsensor_hw_client->IRsensor_hw_get_register(addr);
	log("mIRsensor_show_reg, addr=%02X, value=%02X.\n", addr, value);
	return value;
}

int mIRsensor_store_reg(uint8_t addr, int value)
{	
	if(IRsensor_hw_client->IRsensor_hw_set_register == NULL) {
		err("IRsensor_hw_set_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	IRsensor_hw_client->IRsensor_hw_set_register(addr, value);
	log("mIRsensor_store_reg, addr=%02X, value=%02X.\n", addr, value);
	return 0;
}

static IRsensor_ATTR_Hardware mIRsensor_ATTR_Hardware = {
	.IRsensor_show_reg = mIRsensor_show_reg,
	.IRsensor_store_reg = mIRsensor_store_reg,
};

/****************/
/*HAL Function*/
/***************/
bool mproximity_show_switch_onoff(void)
{	
	return g_ps_data->Device_switch_on;
}

int mproximity_store_switch_onoff(bool bOn)
{
	mutex_lock(&g_ir_lock);
	dbg("Proximity switch = %d.\n", bOn);		
	if ((g_ps_data->Device_switch_on != bOn))	{						
		if (bOn == true)	{
			/* Turn on Proxomity */
			g_ps_data->HAL_switch_on = true;
			proximity_turn_onoff(true);
			/* send the init value */
			psensor_report_abs(IRSENSOR_REPORT_PS_AWAY);
			log("Proximity Report First Away abs.\n");
		} else	{
			/* Turn off Proxomity */
			g_ps_data->HAL_switch_on = false;				
			proximity_turn_onoff(false);				
		}			
	}
	mutex_unlock(&g_ir_lock);
	
	return 0;
}

bool mlight_show_switch_onoff(void)
{
	return g_als_data->Device_switch_on;
}

int mlight_store_switch_onoff(bool bOn)
{
	mutex_lock(&g_ir_lock);
	dbg("Light Sensor switch = %d.\n", bOn);		
	if ((g_als_data->Device_switch_on != bOn)) {					
		if (bOn == true)	{
			/* Turn on Light Sensor */
			g_als_data->HAL_switch_on = true;
			light_turn_onoff(true);
			
			/*light sensor polling the first real event after delayed time. */
			queue_delayed_work(IRsensor_delay_workqueue, &light_polling_lux_work, msecs_to_jiffies(LIGHT_TURNON_DELAY_TIME));
		} else	{
			/* Turn off Light Sensor */
			g_als_data->HAL_switch_on = false;				
			light_turn_onoff(false);
			/* Report lux=-1 when turn off */
			lsensor_report_lux(-1);
		}			
	}
	mutex_unlock(&g_ir_lock);

	return 0;
}

int mlight_show_lux(void)
{
	int adc = 0;
	int lux = 0;

	mutex_lock(&g_ir_lock);

	if (!g_als_data->HAL_switch_on) {
		light_turn_onoff(true);
	}
	
	msleep(LIGHT_TURNON_DELAY_TIME);
	
	adc = IRsensor_hw_client->mlsensor_hw->light_hw_get_adc();
	lux = light_get_lux(adc);
	dbg("mlight_show_lux : %d \n", lux);

	if (!g_als_data->HAL_switch_on) {
		light_turn_onoff(false);
	}
	
	mutex_unlock(&g_ir_lock);
	
	return lux;	
}

static IRsensor_ATTR_HAL mIRsensor_ATTR_HAL = {
	.proximity_show_switch_onoff = mproximity_show_switch_onoff,
	.proximity_store_switch_onoff = mproximity_store_switch_onoff,
	.proximity_show_status = proximity_check_status,
	.light_show_switch_onoff = mlight_show_switch_onoff,
	.light_store_switch_onoff = mlight_store_switch_onoff,
	.light_show_lux = mlight_show_lux,		
};

/*********************/
/*Extension Function*/
/********************/
bool mIRsensor_show_allreg(void)
{
	if(IRsensor_hw_client->IRsensor_hw_show_allreg == NULL) {
		err("IRsensor_hw_show_allreg NOT SUPPORT. \n");
		return false;
	}
	IRsensor_hw_client->IRsensor_hw_show_allreg();
	return true;
}

bool mproximity_show_polling_mode(void)
{
	return g_ps_data->polling_mode;
}

int mproximity_store_polling_mode(bool bOn)
{
	g_ps_data->polling_mode = bOn;	
	return 0;
}

int mlight_show_sensitivity(void)
{
	return g_als_data->g_als_change_sensitivity;
}

int mlight_store_sensitivity(int sensitivity)
{
	g_als_data->g_als_change_sensitivity = sensitivity;
	log("Light Sensor store Sensitivity: %d\n", sensitivity);	
	
	return 0;
}

int mlight_show_log_threshold(void)
{
	return g_als_data->g_als_log_threshold;
}

int mlight_store_log_threshold(int log_threshold)
{
	g_als_data->g_als_log_threshold = log_threshold;
	log("Light Sensor store Log Threshold: %d\n", log_threshold);	
	
	return 0;
}

static IRsensor_ATTR_Extension mATTR_Extension = {
	.IRsensor_show_allreg = mIRsensor_show_allreg,
	.proximity_show_polling_mode = mproximity_show_polling_mode,
	.proximity_store_polling_mode = mproximity_store_polling_mode,
	.light_show_sensitivity = mlight_show_sensitivity,
	.light_store_sensitivity = mlight_store_sensitivity,
	.light_show_log_threshold = mlight_show_log_threshold,
	.light_store_log_threshold = mlight_store_log_threshold,
};

static IRsensor_ATTR mIRsensor_ATTR = {
	.info_type = &mIRsensor_info_type,
	.ATTR_Calibration = &mIRsensor_ATTR_Calibration,
	.ATTR_BMMI = &mIRsensor_ATTR_BMMI,
	.ATTR_Hardware = &mIRsensor_ATTR_Hardware,
	.ATTR_HAL = &mIRsensor_ATTR_HAL,
	.ATTR_Extension = &mATTR_Extension,
};

/*================================
 *|| Interrupt Service Routine Part ||
 *================================
 */
static void proximity_work(int state)
{
	int adc = 0;

	/* Get Proximity adc value */
	adc= IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
	if(adc < 0){
		err("[ISR] Proximity get adc ERROR\n");		
	}

	/* Ignore the interrupt when Switch off */
	if(g_ps_data->HAL_switch_on == true)
	{
		/* Check proximity close or away. */
		if(IRSENSOR_INT_PS_AWAY == state) {
			log("[ISR] Proximity Detect Object Away. (adc = %d)\n", adc);		
			psensor_report_abs(IRSENSOR_REPORT_PS_AWAY);
		} else if (IRSENSOR_INT_PS_CLOSE == state) {
			log("[ISR] Proximity Detect Object Close. (adc = %d)\n", adc);		
			psensor_report_abs(IRSENSOR_REPORT_PS_CLOSE);
		} else {
			err("[ISR] Proximity Detect Object ERROR. (adc = %d)\n", adc);
		}
	}
	
}

static void light_work(void)
{	
	int low_threshold = 0;
	int high_threshold = 0;	
	int adc = 0;
	int lux = 0;
	int ret = 0;
	int light_change_sensitivity = 0;	
	int light_log_threshold = 0;

	/* Ignore the interrupt when Switch off */
	if(g_als_data->HAL_switch_on == true)
	{
		adc = IRsensor_hw_client->mlsensor_hw->light_hw_get_adc();
		dbg("[ISR] Light Sensor Get adc : %d\n", adc);

		/* Set the default sensitivity (3rd priority)*/
		if(adc >= g_als_data->g_als_calvalue_1000lux) {
			light_change_sensitivity = LIGHT_CHANGE_LOW_SENSITIVITY;
		} else if (adc <= g_als_data->g_als_calvalue_200lux) {
			light_change_sensitivity = LIGHT_CHANGE_HI_SENSITIVITY;
		} else {
			light_change_sensitivity = LIGHT_CHANGE_MID_SENSITIVITY;
		}

		/* Set the factory sensitivity (2nd priority) */
#ifdef ASUS_FACTORY_BUILD
		light_change_sensitivity = LIGHT_CHANGE_FACTORY_SENSITIVITY;
#endif

		/* Set the interface sensitivity (1st priority) */
		if(g_als_data->g_als_change_sensitivity >= 0)
			light_change_sensitivity = g_als_data->g_als_change_sensitivity;
		
		dbg("[ISR] Light Sensor Set Sensitivity. (light_change_sensitivity:%d)\n", light_change_sensitivity);	
		
		/* Light Sensor Low Threshold */	
		low_threshold = adc * (100 - light_change_sensitivity) / 100;

		/* Light Sensor High Threshold */
		high_threshold = adc * (100 + light_change_sensitivity) / 100;	
		if (high_threshold > IRsensor_hw_client->mlsensor_hw->light_max_threshold)	
			high_threshold = IRsensor_hw_client->mlsensor_hw->light_max_threshold;	
		
		ret = IRsensor_hw_client->mlsensor_hw->light_hw_set_hi_threshold(high_threshold);
		if(ret < 0) {
			err("[ISR] Light Sensor Set High Threshold ERROR. (High:%d)\n", high_threshold);
		}
		dbg("[ISR] Light Sensor Set High Threshold. (High:%d)\n", high_threshold);	

		ret = IRsensor_hw_client->mlsensor_hw->light_hw_set_lo_threshold(low_threshold);
		if(ret < 0) {
			err("[ISR] Light Sensor Set Low Threshold ERROR. (Low:%d)\n", low_threshold);
		}
		dbg("[ISR] Light Sensor Set Low Threshold. (Low:%d)\n", low_threshold);	
		
		/* Light Sensor Report input event*/
		lux = light_get_lux(adc);

		light_log_threshold = LIGHT_LOG_THRESHOLD;
		
		/* Set the interface log threshold (1st priority) */
		if(g_als_data->g_als_log_threshold >= 0)
			light_log_threshold = g_als_data->g_als_log_threshold;
		
		if(abs(g_als_last_lux - lux) > light_log_threshold)
			log("[ISR] Light Sensor Report lux : %d (adc = %d)\n", lux, adc);
			
		lsensor_report_lux(lux);	
		g_als_last_lux = lux;
	}
}

static void IRsensor_ist(struct work_struct *work)
{
	int ret = 0;	
	int irsensor_int_ps, irsensor_int_als;
	
mutex_lock(&g_ir_lock);

	dbg("IRsensor ist +++ \n");
	/* when driver probe, the interrupt MAY BE triggered */
	if(IRsensor_hw_client == NULL) {
		err("IRsensor_hw_client is NULL. \n");
		goto ist_err;
	}
	
	/* Read INT_FLAG will clean the interrupt */
	if(IRsensor_hw_client->IRsensor_hw_get_interrupt == NULL) {
		err("IRsensor_hw_get_interrupt NOT SUPPORT. \n");
		goto ist_err;
	}
	ret = IRsensor_hw_client->IRsensor_hw_get_interrupt();

	/* Check Proximity Interrupt */
	irsensor_int_ps = ret&IRSENSOR_INT_PS_MASK;
	if(irsensor_int_ps == IRSENSOR_INT_PS_CLOSE || irsensor_int_ps == IRSENSOR_INT_PS_AWAY) 
	{
		dbg("Proximity ist \n");
		if (irsensor_int_ps == IRSENSOR_INT_PS_AWAY)
			proximity_work(IRSENSOR_INT_PS_AWAY);
		
		if (irsensor_int_ps == IRSENSOR_INT_PS_CLOSE)
			proximity_work(IRSENSOR_INT_PS_CLOSE);	
	}

	/* Check Light Sensor Interrupt */
	irsensor_int_als = ret&IRSENSOR_INT_ALS_MASK;
	if (irsensor_int_als == IRSENSOR_INT_ALS) {
		dbg("Light Sensor ist \n");
		light_work();
	}
	dbg("IRsensor ist --- \n");
ist_err:
	wake_unlock(&g_ir_wake_lock);
	dbg("[IRQ] Enable irq !! \n");
	enable_irq(ASUS_IR_SENSOR_IRQ);
mutex_unlock(&g_ir_lock);

}

void IRsensor_irq_handler(void)
{
	dbg("[IRQ] Disable irq !! \n");
	disable_irq_nosync(ASUS_IR_SENSOR_IRQ);
	queue_work(IRsensor_workqueue, &IRsensor_ist_work);
	wake_lock(&g_ir_wake_lock);
}

static IRsensor_GPIO mIRsensor_GPIO = {
	.IRsensor_isr = IRsensor_irq_handler,
};


/*============================
 *|| For Proximity check status ||
 *============================
 */
bool proximity_check_status(void)
{	
	int adc_value = 0;
	bool status = false;	

	/* check probe status */
	if(IRsensor_hw_client == NULL)
		return status;

	mutex_lock(&g_ir_lock);
	
	if (!g_ps_data->HAL_switch_on) {
		proximity_turn_onoff(true);
	}
	
	msleep(PROXIMITY_TURNON_DELAY_TIME);

	adc_value = IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc();

	if (adc_value >= g_ps_data->g_ps_calvalue_hi) {
		status = true;
	}else{ 
		status = false;
	}
	log("proximity_check_status : %s \n", status?"Close":"Away");
	
	if (!g_ps_data->HAL_switch_on) {
		proximity_turn_onoff(false);
	}
	mutex_unlock(&g_ir_lock);

	return status;
}

EXPORT_SYMBOL(proximity_check_status);

/*====================
 *|| Initialization Part ||
 *====================
 */
static int init_data(void)
{
	int ret = 0;
	/* Reset ASUS_light_sensor_data */
	g_als_data = kmalloc(sizeof(struct ASUS_light_sensor_data), GFP_KERNEL);
	if (!g_als_data)	{
		err("g_als_data kmalloc ERROR\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	memset(g_als_data, 0, sizeof(struct ASUS_light_sensor_data));
	g_als_data->Device_switch_on = false;
	g_als_data->HAL_switch_on = 	false;	
	
	g_als_data->g_als_calvalue_200lux = 	IRsensor_hw_client->mlsensor_hw->light_200lux_default;
	g_als_data->g_als_calvalue_1000lux = 	IRsensor_hw_client->mlsensor_hw->light_1000lux_default;	
	g_als_data->g_als_calvalue_shift = 		IRSENSOR_DEFAULT_VALUE;
	g_als_data->g_als_change_sensitivity = IRSENSOR_DEFAULT_VALUE;
	g_als_data->g_als_log_threshold = 		IRSENSOR_DEFAULT_VALUE;
	
	/* Reset ASUS_proximity_sensor_data */
	g_ps_data = kmalloc(sizeof(struct ASUS_proximity_sensor_data), GFP_KERNEL);
	if (!g_ps_data) {
		err("g_ps_data kmalloc ERROR\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	memset(g_ps_data, 0, sizeof(struct ASUS_proximity_sensor_data));
	g_ps_data->Device_switch_on = 	false;
	g_ps_data->HAL_switch_on = 	false;	
	g_ps_data->polling_mode = 		false;
	
	g_ps_data->g_ps_calvalue_hi = IRsensor_hw_client->mpsensor_hw->proximity_hi_threshold_default;
	g_ps_data->g_ps_calvalue_lo = IRsensor_hw_client->mpsensor_hw->proximity_low_threshold_default;		

	return 0;
init_data_err:
	err("Init Data ERROR\n");
	return ret;
}
 
void mIRsensor_algo_probe(struct i2c_client *client)
{	
	int ret = 0;	
	log("Driver PROBE +++\n");

	/*check i2c client*/
	if (client == NULL) {
		err("i2c Client is NUll\n");
		goto probe_err;
	}	

	/*link driver data to i2c client*/
	strlcpy(client->name, SENSOR_TYPE_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, g_als_data);
	i2c_set_clientdata(client, g_ps_data);	

	/*set i2c Client for of_get_named_gpio*/
	ret = IRsensor_gpio_setI2cClient(client);
	if(ret < 0) {
		err("IRsensor_gpio_setI2cClient is ERROR\n");
		goto probe_err;
	}

	/* I2c stress test */
#ifdef CONFIG_I2C_STRESS_TEST	
	i2c_add_test_case(client, "IRSensorTest", ARRAY_AND_SIZE(IRSensorTestCaseInfo));	
#endif
	
	log("Driver PROBE ---\n");
	return ;
probe_err:
	err("Driver PROBE ERROR ---\n");
	return;

}

void IRsensor_algo_remove(void)
{
	log("Driver REMOVE +++\n");

	IRsensor_gpio_unregister(ASUS_IR_SENSOR_IRQ);

	log("Driver REMOVE ---\n");
	
	return;
}

void mIRsensor_algo_shutdown(void)
{
	log("Driver SHUTDOWN +++\n");

	/* Disable sensor */
	if (g_als_data->Device_switch_on)
		light_turn_onoff(false);
	if (g_ps_data->Device_switch_on)
		proximity_turn_onoff(false);	
	
	log("Driver SHUTDOWN ---\n");
	
	return;
}

void mIRsensor_algo_suspend(void)
{
	log("Driver SUSPEND +++\n");

	/* For keep Proximity can wake_up system */
	if (g_ps_data->Device_switch_on)
		enable_irq_wake(ASUS_IR_SENSOR_IRQ);

	/* For make sure Light sensor mush be switch off when system suspend */
	if (g_als_data->Device_switch_on)				
		light_turn_onoff(0);
	
	log("Driver SUSPEND ---\n");
	
	return;
}

void mIRsensor_algo_resume(void)
{
	log("Driver RESUME +++\n");

	if (g_als_data->Device_switch_on == 0 && g_als_data->HAL_switch_on == 1)
		light_turn_onoff(1);
	
	log("Driver RESUME ---\n");
	
	return;
}

static IRsensor_I2C mIRsensor_I2C = {
	.IRsensor_probe = mIRsensor_algo_probe,
	.IRsensor_remove = IRsensor_algo_remove,
	.IRsensor_shutdown = mIRsensor_algo_shutdown,
	.IRsensor_suspend = mIRsensor_algo_suspend,
	.IRsensor_resume = mIRsensor_algo_resume,
};

static int __init IRsensor_init(void)
{
	int ret = 0;
	log("Driver INIT +++\n");
	
	/* Work Queue */
	IRsensor_workqueue = create_singlethread_workqueue(SENSOR_TYPE_NAME"_wq");	
	IRsensor_delay_workqueue = create_singlethread_workqueue(SENSOR_TYPE_NAME"_delay_wq");	

	/* Initialize the Mutex */
	mutex_init(&g_ir_lock);	

	/* Initialize the wake lock */
	wake_lock_init(&g_ir_wake_lock, WAKE_LOCK_SUSPEND, "IRsensor_wake_lock");
	
	/* i2c Registration for probe/suspend/resume */				
	ret = IRsensor_i2c_register(&mIRsensor_I2C);
	if (ret < 0)
		goto init_err;
	
	/* Hardware Register Initialization */
	IRsensor_hw_client = IRsensor_hw_getHardware();
	if(IRsensor_hw_client == NULL)
		goto init_err;

	/* GPIO */
	ASUS_IR_SENSOR_IRQ = IRsensor_gpio_register(&mIRsensor_GPIO);
	if (ASUS_IR_SENSOR_IRQ < 0)
		goto init_err;	

	/* driver data structure initialize */
	ret = init_data();
	if (ret < 0)
		goto init_err;

	/* string copy the character of vendor and module number */
	strcpy(mIRsensor_ATTR.info_type->vendor, IRsensor_hw_client->vendor);
	strcpy(mIRsensor_ATTR.info_type->module_number, IRsensor_hw_client->module_number);
	
	/* Attribute */
	IRsensor_ATTR_register(&mIRsensor_ATTR);	
	if (ret < 0)
		goto init_err;
	
	/* Input Device */
	ret = IRsensor_report_register();
	if (ret < 0)
		goto init_err;	

	/*Turn off Proximity and Disable Proximity Interrupt */
	ret = proximity_turn_onoff(0);
	if (ret < 0) {		
		goto init_err;
	}	
	
	/*Turn off Light Sensor and Disable Light Sensor Interrupt*/
	ret = light_turn_onoff(0);
	if (ret < 0) {		
		goto init_err;
	}	
		
	log("Driver INIT ---\n");
	return 0;

init_err:
	err("Driver INIT ERROR ---\n");
	return ret;
}

static void __exit IRsensor_exit(void)
{
	log("Driver EXIT +++\n");

	/* i2c Unregistration */	
	IRsensor_i2c_unregister();

	IRsensor_report_unregister();
	IRsensor_ATTR_unregister();	
	
	wake_lock_destroy(&g_ir_wake_lock);
	mutex_destroy(&g_ir_lock);
	kfree(g_ps_data);
	kfree(g_als_data);

	destroy_workqueue(IRsensor_workqueue);
	destroy_workqueue(IRsensor_delay_workqueue);
	
	log("Driver EXIT ---\n");
}

module_init(IRsensor_init);
module_exit(IRsensor_exit);

MODULE_AUTHOR("sr_Huang <sr_Huang@asus.com>");
MODULE_DESCRIPTION("Proximity and Ambient Light Sensor");
MODULE_LICENSE("GPL");

