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
 
#ifndef __LINUX_ASH_H
#define __LINUX_ASH_H

/**
 * ASH_type - define the sensor types ASH supports.
 */
typedef enum{
	psensor = 0,
	lsensor,
	hallsensor,
}ASH_type;

/**
 * ASH_ATTR_device_create - create the sensor type attributes and return device struct,
 * 			which can used to create attributes. (/sys/class/sensors/)
 */
#include <linux/device.h>
#include <linux/fs.h>
extern struct device *ASH_ATTR_device_create(ASH_type type);

/**
 * ASH_ATTR_device_remove - remove the sensor type.
 */
extern void ASH_ATTR_device_remove(ASH_type type);

/**
 * HALLsensor_ATTR - for attributes  
 * @show_action_status : show the hall sensor status.
 * @show_hall_sensor_enable : true - HW on ; false - HW off.
 * @store_hall_sensor_enable : true - HW turn on ; false - HW turn off.
 * @show_hall_sensor_debounce : time in mini second(ms).
 * @store_hall_sensor_debounce : time in mini second(ms).
 */
typedef struct{
	bool (*show_action_status)(void);
	bool (*show_hall_sensor_enable)(void);
	int (*store_hall_sensor_enable)(bool bOn);
	int (*show_hall_sensor_debounce)(void);
	int (*store_hall_sensor_debounce)(int debounce);
}HALLsensor_ATTR;

/**
 * HALLsensor_ATTR_register - assign a hall sensor file node and create the attributes.
 * @status : 1 - No magnetic ; 0 - Magnetic
 * @switch : on - HW on ; off - HW off
 * @debounce : time in mini second(ms)
 *
 * The attributes will be created at /sys/class/sensors/hallsensor.
 */
extern int HALLsensor_ATTR_register(HALLsensor_ATTR *mATTR);

/**
 * HALLsensor_ATTR_unregister - remove the hall sensor attributes.
 */
extern int HALLsensor_ATTR_unregister(void);

/**
 * HALLsensor_ATTR_create - create hall sensor customize attributes.
 * @mpsensor_attr : the pointer of device_attribute, which you want to create attribute.
 */
#include <linux/device.h>
extern int HALLsensor_ATTR_create(struct device_attribute *mhallsensor_attr);

/**
 * IRsensor_info_type - define the IRsensor information.
 * @vendor : ASUS.
 * @version : driver version.
 * @module_number : hardware chip serial number.
 */
#define NAME_SIZE	(10)
typedef struct{
	char vendor[NAME_SIZE];
	char module_number[NAME_SIZE];
}IRsensor_info_type;

/**
 * @LIGHT_GAIN_ACCURACY_CALVALUE : the accuracy of light sensor gain value.
 */
#define LIGHT_GAIN_ACCURACY_CALVALUE	(100000)

/**
 * IRsensor_ATTR_Calibration - attributes for IRsensor calibration.
 */
typedef struct{	
	int (*proximity_show_calibration_hi)(void);
	int (*proximity_store_calibration_hi)(int calvalue);
	int (*proximity_show_calibration_lo)(void);
	int (*proximity_store_calibration_lo)(int calvalue);
	int (*light_show_calibration_200lux)(void);
	int (*light_store_calibration_200lux)(int calvalue);
	int (*light_show_calibration_1000lux)(void);
	int (*light_store_calibration_1000lux)(int calvalue);	
	int (*light_show_shift)(void);
	int (*light_show_gain)(void);
	int (*light_show_adc)(void);
	int (*proximity_show_adc)(void);
}IRsensor_ATTR_Calibration;

/**
 * IRsensor_ATTR_BMMI - attributes for IRsensor BMMI.
 */
typedef struct{	
	bool (*proximity_show_atd_test)(void);
	bool (*light_show_atd_test)(void);
}IRsensor_ATTR_BMMI;

/**
 * IRsensor_ATTR_Hardware - attributes for IRsensor hardware read/write.
 */
typedef struct{
	uint8_t show_reg_addr;
	int (*IRsensor_show_reg)(uint8_t addr);
	int (*IRsensor_store_reg)(uint8_t addr, int value);	
}IRsensor_ATTR_Hardware;

/**
 * IRsensor_ATTR_HAL - attributes for IRsensor HAL function.
 */
typedef struct{	
	bool (*proximity_show_switch_onoff)(void);
	int (*proximity_store_switch_onoff)(bool bOn);
	bool (*proximity_show_status)(void);
	bool (*light_show_switch_onoff)(void);
	int (*light_store_switch_onoff)(bool bOn);
	int (*light_show_lux)(void);		
}IRsensor_ATTR_HAL;

/**
 * IRsensor_ATTR_Extension - attributes for IRsensor extensive functions.
 */
typedef struct{	
	bool (*IRsensor_show_allreg)(void);
	bool (*proximity_show_polling_mode)(void);
	int (*proximity_store_polling_mode)(bool bOn);
	int (*light_show_sensitivity)(void);
	int (*light_store_sensitivity)(int sensitivity);
	int (*light_show_log_threshold)(void);
	int (*light_store_log_threshold)(int log_threshold);
}IRsensor_ATTR_Extension;

/**
 * IRsensor_ATTR - attributes for IRsensor.
 */
typedef struct{
	IRsensor_info_type 			*info_type;	
	IRsensor_ATTR_Calibration 	*ATTR_Calibration;
	IRsensor_ATTR_BMMI 		*ATTR_BMMI;
	IRsensor_ATTR_Hardware 	*ATTR_Hardware;
	IRsensor_ATTR_HAL 		*ATTR_HAL;
	IRsensor_ATTR_Extension 	*ATTR_Extension;
}IRsensor_ATTR;

/**
 * IRsensor_ATTR_register - assign a psensor/lsensor file node and create the attributes.
 *
 * The attributes will be created at /sys/class/sensors/psensor.
 * The attributes will be created at /sys/class/sensors/lsensor.
 */
extern int IRsensor_ATTR_register(IRsensor_ATTR *mATTR);

/**
 * IRsensor_ATTR_unregister - remove the psensor/lsensor attributes.
 */
extern int IRsensor_ATTR_unregister(void);

/**
 * psensor_ATTR_create - create psensor customize attributes.
 * @mpsensor_attr : the pointer of device_attribute, which you want to create attribute.
 */
#include <linux/device.h>
extern int psensor_ATTR_create(struct device_attribute *mpsensor_attr);

/**
 * lsensor_ATTR_create - create lsensor customize attributes.
 * @mlsensor_attr : the pointer of device_attribute, which you want to create attribute.
 */
extern int lsensor_ATTR_create(struct device_attribute *mlsensor_attr);

/**
 * Define the proximity report event values.
 * @IRSENSOR_REPORT_PS_CLOSE : report close event.
 * @IRSENSOR_REPORT_PS_AWAY : report away event.
 */
#define IRSENSOR_REPORT_PS_CLOSE 				(0)
#define IRSENSOR_REPORT_PS_AWAY     			(1) 

/**
 * IRsensor_report_register - before report psensor/lesensor event
 * you need to register first. This will create input device for IRsensor.
 */
extern int IRsensor_report_register(void);

/**
 * IRsensor_report_unregister - Remove the input device for IRsensor.
 */
extern void IRsensor_report_unregister(void);

/**
 * psensor_report_abs - report the proximity abs.
 * @ abs=IRSENSOR_REPORT_PS_CLOSE : report close event.
 * @ abs=IRSENSOR_REPORT_PS_AWAY : report away event.
 */
extern void psensor_report_abs(int abs);

/**
 * lsensor_report_lux - report the light sensor lux.
 */
extern void lsensor_report_lux(int lux);

/**
 * Define the hall sensor report event values.
 * @HALLSENSOR_REPORT_LID_OPEN : report open event.
 * @HALLSENSOR_REPORT_LID_CLOSE : report close event.
 */
#define HALLSENSOR_REPORT_LID_OPEN 			(0)
#define HALLSENSOR_REPORT_LID_CLOSE    		(1) 

/**
 * HALLsensor_report_register - before report hall sensor event
 * you need to register first. This will create input device for Hall sensor.
 */
extern int HALLsensor_report_register(void);

/**
 * HALLsensor_report_unregister - Remove the input device for Hall sensor.
 */
extern void HALLsensor_report_unregister(void);

/**
 * hallsensor_report_lid - report the hall sensor lid.
 */
extern void hallsensor_report_lid(int lid);

/**
 * psensor_sysfs_read_high
 * psensor_sysfs_read_low - kernel space read high/low calibration data.
 * 
 * Return value is NOT negative (>=0) if it is success reading from file(.nv)
 */
extern int 	psensor_sysfs_read_high(void);
extern int 	psensor_sysfs_read_low(void);

/**
 * psensor_sysfs_write_high
 * psensor_sysfs_write_low - kernel space write high/low calibration data.
 *
 * Return value is TRUE if it is success writing value to file(.nv)
 */
extern bool psensor_sysfs_write_high(int calvalue);
extern bool psensor_sysfs_write_low(int calvalue);

/**
 * lsensor_sysfs_read_200lux
 * lsensor_sysfs_read_1000lux - kernel space read 200lux/1000lux calibration data.
 *
 * Return value is NOT negative (>=0) if it is success reading from file(.nv)
 */
extern int 	lsensor_sysfs_read_200lux(void);
extern int 	lsensor_sysfs_read_1000lux(void);

/**
 * lsensor_sysfs_write_200lux
 * lsensor_sysfs_write_1000lux - kernel space write 200lux/1000lux calibration data.
 *
 * Return valu is TRUE if it is success writing value to file(.nv)
 */
extern bool lsensor_sysfs_write_200lux(int calvalue);
extern bool lsensor_sysfs_write_1000lux(int calvalue);

/**
 * Define the path of Light Sensor Calibration file.
 * @LSENSOR_200LUX_CALIBRATION_FILE : 200 Lux Calibration file.
 * @LSENSOR_1000LUX_CALIBRATION_FILE : 1000 Lux Calibration file.
 */
#define LSENSOR_200LUX_CALIBRATION_FILE	"/factory/lsensor_200lux.nv"
#define LSENSOR_1000LUX_CALIBRATION_FILE	"/factory/lsensor_1000lux.nv"

/**
 * Define the path of Proximity Sensor Calibration file.
 * @PSENSOR_HI_CALIBRATION_FILE : high calibration file.
 * @PSENSOR_LOW_CALIBRATION_FILE : low calibration file.
 */
#define PSENSOR_HI_CALIBRATION_FILE  		"/factory/psensor_hi.nv"
#define PSENSOR_LOW_CALIBRATION_FILE  	"/factory/psensor_low.nv"

/**
 * IRsensor_GPIO
 */
 typedef struct IRsensor_GPIO {
	void (*IRsensor_isr)(void);
}IRsensor_GPIO;

/**
 * IRsensor_gpio_register - register the GPIO setting and set the IRQ handler.
 * Return the IRQ.
 */
extern int IRsensor_gpio_register(IRsensor_GPIO *gpio_ist);

/**
 * IRsensor_gpio_unregister - unregister the GPIO setting.
 */
extern int IRsensor_gpio_unregister(int irq);

/**
 * IRsensor_gpio_setI2cClient - set the i2c_client for of_get_named_gpio.
 */
 #include <linux/i2c.h>
 extern int IRsensor_gpio_setI2cClient(struct i2c_client *client);

/**
 * HALLsensor_GPIO
 */
 typedef struct HALLsensor_GPIO {
	void (*HALLsensor_isr)(void);
}HALLsensor_GPIO;

/**
 * HALLsensor_gpio_register - register the GPIO setting and set the IRQ handler.
 * Return the IRQ.
 */
#include <linux/platform_device.h>
extern int HALLsensor_gpio_register(struct platform_device *pdev, HALLsensor_GPIO *gpio_ist);

/**
 * HALLsensor_gpio_unregister - unregister the GPIO setting.
 */
extern int HALLsensor_gpio_unregister(int irq);

/**
 * HALLsensor_gpio_value - return the gpio value.
 */
extern int HALLsensor_gpio_value(void);

/**
 * i2c_read_reg_u8 - 
 * i2c_write_reg_u8 - read/write i2c for 1 Byte (8bits). These are functions for i2c read/write.
 */
extern uint8_t i2c_read_reg_u8(struct i2c_client* client, u8 reg);
extern int 		i2c_write_reg_u8(struct i2c_client* client, u8 reg, uint8_t data);

/**
 * i2c_read_reg_u16 - 
 * i2c_write_reg_u16 - read/write i2c for 2 Byte (16bits). These are functions for i2c read/write.
 */
extern int i2c_read_reg_u16(struct i2c_client* client, u8 reg, uint8_t* data);
extern int i2c_write_reg_u16(struct i2c_client* client, u8 reg, uint8_t* data);

/**
 * IRsensor_I2C - We define functions for i2c driver.
 * @IRsensor_probe : It will be run when "i2c_add_driver" has been called.
 * @IRsensor_remove : Remove all Memory.
 * @IRsensor_shutdown : Turn off all sensors.
 * @IRsensor_suspend : Do what suspend should do.
 * @IRsensor_resume : Do what resume should do.
 */
typedef struct IRsensor_I2C {
	void (*IRsensor_probe)(struct i2c_client *client);
	void (*IRsensor_remove)(void);
	void (*IRsensor_shutdown)(void);
	void (*IRsensor_suspend)(void);
	void (*IRsensor_resume)(void);	
}IRsensor_I2C;

/**
 * IRsensor_i2c_register - struct IRsensor_I2C wrapped by i2c driver.
 */
extern int IRsensor_i2c_register(IRsensor_I2C *ir_i2c);

/**
 * IRsensor_i2c_unregister - i2c_del_driver.
 */
extern int IRsensor_i2c_unregister(void);

/**
 * HALLsensor_Platform - We define functions for platform driver.
 * @HALLsensor_probe : It will be run when "platform_driver_register" has been called.
 * @HALLsensor_suspend : Do what suspend should do.
 * @HALLsensor_resume : Do what resume should do.
 */
typedef struct HALLsensor_Platform {
	void (*HALLsensor_probe)(struct platform_device *pdev);
	void (*HALLsensor_suspend)(void);
	void (*HALLsensor_resume)(void);	
}HALLsensor_Platform;

/**
 * IRsensor_i2c_register - struct IRsensor_I2C wrapped by i2c driver.
 */
extern int HALLsensor_platform_register(HALLsensor_Platform *hall_platform);

/**
 * IRsensor_i2c_unregister - i2c_del_driver.
 */
extern int HALLsensor_platform_unregister(void);


/**
 * Define the return interrupt bits, which support proximity close/away and light sensor simultaneously.
 * @IRSENSOR_INT_PS_CLOSE : [1:0] (0,1)
 * @IRSENSOR_INT_PS_AWAY : [1:0] (1,0)
 * @IRSENSOR_INT_ALS : [2] (1)
 */
#define IRSENSOR_INT_PS_CLOSE 				(1)
#define IRSENSOR_INT_PS_AWAY     			(2) 
#define IRSENSOR_INT_PS_MASK				(3<< 0)
#define IRSENSOR_INT_ALS           				(4)
#define IRSENSOR_INT_ALS_MASK				(1<< 2)

/**
 * psensor_hw - the i2c control functions for proximity sensor.
 * @proximity_low_threshold_default : 
 * @proximity_hi_threshold_default : depends on each hardware situation, which impact on the CSC SMMI.
 * @proximity_hw_turn_onoff : Turn on the proximity sensor.
 * @proximity_hw_get_adc : get the count of proximity sensor.
 * @proximity_hw_set_hi_threshold :
 * @proximity_hw_set_lo_threshold : set proximity threshold which will trigger the interrupt.
 */
typedef struct psensor_hw {
	int proximity_low_threshold_default;	
	int proximity_hi_threshold_default;
	
	int (*proximity_hw_turn_onoff)(bool bOn);
	int (*proximity_hw_get_adc)(void);
	int (*proximity_hw_set_hi_threshold)(int hi_threshold);
	int (*proximity_hw_set_lo_threshold)(int low_threshold);
}psensor_hw;

/**
 * lsensor_hw - the i2c control functions for light sensor.
 * @light_max_threshold : the maximum count of light sensor.
 * @light_hw_turn_onoff : Turn on the light sensor.
 * @light_hw_get_adc : get the count of light sensor.
 * @light_hw_set_hi_threshold : 
 * @light_hw_set_lo_threshold : set light sensor threshold which will trigger the interrupt.
 */
typedef struct lsensor_hw {
	int light_max_threshold;
	int light_200lux_default;
	int light_1000lux_default;
	int (*light_hw_turn_onoff)(bool bOn);
	int (*light_hw_get_adc)(void);
	int (*light_hw_set_hi_threshold)(int hi_threshold);
	int (*light_hw_set_lo_threshold)(int low_threshold);
}lsensor_hw;

/**
 * IRsensor_hw - the i2c control functions for IR sensor including psensor and lsensor.
 */
 #include <linux/i2c.h>
typedef struct IRsensor_hw {	
	char vendor[NAME_SIZE];
	char module_number[NAME_SIZE];

	int (*IRsensor_hw_check_ID)(void);	
	int (*IRsensor_hw_init)(struct i2c_client* client);
	int (*IRsensor_hw_get_interrupt)(void);	
	int (*IRsensor_hw_show_allreg)(void);	
	int (*IRsensor_hw_set_register)(uint8_t reg, int value);
	int (*IRsensor_hw_get_register)(uint8_t reg);	

	psensor_hw	*mpsensor_hw;
	lsensor_hw		*mlsensor_hw;
}IRsensor_hw;

/**
 * IRsensor_hw_getHardware - Before this function, you SHOULD execute IRsensor_i2c_register first.
 */
extern IRsensor_hw* IRsensor_hw_getHardware(void);

#endif
