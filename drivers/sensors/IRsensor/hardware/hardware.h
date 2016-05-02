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

/*******************************/
/* IR Sensor Hardware Module */
/******************************/
#ifndef __LINUX_IRSENSOR_HARDWARE_H
#define __LINUX_IRSENSOR_HARDWARE_H

#define NAME_SIZE (20)
#define INT_NULL			(0)

enum hardware_source {
	IRsensor_hw_source_ap3425=0,	
	IRsensor_hw_source_cm36686,
	IRsensor_hw_source_max
};

typedef struct IRsensor_hw {	
	int (*IRsensor_hw_init)(struct i2c_client* client);
	int (*IRsensor_hw_show_allreg)(void);
	int (*IRsensor_hw_get_interrupt)(void);
	int (*IRsensor_hw_check_ID)(void);
	
	int (*proximity_hw_turn_onoff)(bool bOn);
	int (*proximity_hw_get_adc)(void);
	int (*proximity_hw_set_hi_threshold)(int hi_threshold);
	int (*proximity_hw_set_lo_threshold)(int low_threshold);
	int (*proximity_hw_set_led_current)(uint8_t led_current_reg);
	int (*proximity_hw_set_led_duty_ratio)(uint8_t led_duty_ratio_reg);
	int (*proximity_hw_set_persistence)(uint8_t persistence);
	int (*proximity_hw_set_integration)(uint8_t integration);

	int (*light_hw_turn_onoff)(bool bOn);
	int (*light_hw_get_adc)(void);
	int (*light_hw_set_hi_threshold)(int hi_threshold);
	int (*light_hw_set_lo_threshold)(int low_threshold);
	int (*light_hw_set_persistence)(uint8_t persistence);
	int (*light_hw_set_integration)(uint8_t integration);

	int proximity_low_threshold_default;	
	int proximity_hi_threshold_default;
	int proximity_max_led_current;
	int proximity_max_led_duty_ratio;
	int proximity_max_persistence;
	int proximity_max_integration;
	
	int light_max_threshold;
	int light_max_persistence;
	int light_max_integration;
	
	char vendor[NAME_SIZE];
	char module_number[NAME_SIZE];
}IRsensor_hw;

extern IRsensor_hw* IRsensor_hw_getHardware(int hardware_source);
extern struct i2c_driver* IRsensor_hw_setI2cDriver(int hardware_source, 
	struct i2c_driver* IRsensor_i2c_driver_client);

/*read/write i2c for 1 Byte (8bits)*/
extern uint8_t i2c_read_reg_u8(struct i2c_client* client, u8 reg);
extern int i2c_write_reg_u8(struct i2c_client* client, u8 reg, uint8_t data);

/*read/write i2c for 2 Byte (16bits)*/
extern int i2c_read_reg_u16(struct i2c_client* client, u8 reg, uint8_t* data);
extern int i2c_write_reg_u16(struct i2c_client* client, u8 reg, uint8_t* data);

#endif

