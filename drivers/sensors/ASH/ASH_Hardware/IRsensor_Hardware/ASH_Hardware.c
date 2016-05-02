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

/********************************/
/* IR Sensor Hardware Module */
/******************************/
#define MODULE_NAME	"hardware"
 
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include "hardware.h"
#include "../IRsensor.h"
#include "cm36686/cm36686.h"
#include "ap3425/ap3425.h"

/***********************/
/*CM36686 I2c Driver*/
/**********************/
static const struct i2c_device_id cm36686_i2c_id[] = {
	{"cm36686", 0},
	{}
};

static struct of_device_id cm36686_match_table[] = {
	{ .compatible = "qcom,cm36686",},
	{},
};

/**********************/
/*AP3425 I2c Driver */
/*********************/
static const struct i2c_device_id ap3425_i2c_id[] = {
	{"ap3425", 0},
	{}
};

static struct of_device_id ap3425_match_table[] = {
	{ .compatible = "qcom,ap3425",},
	{},
};

/**********************/
/*Interface Functions*/
/*********************/
IRsensor_hw* IRsensor_hw_getHardware(int hardware_source)
{
	IRsensor_hw* IRsensor_hw_client = NULL;
	switch(hardware_source) {
		case IRsensor_hw_source_cm36686:
			dbg("get hardware client : cm36686 \n");
			IRsensor_hw_client = IRsensor_hw_cm36686_getHardware();
			break;

		case IRsensor_hw_source_ap3425:
			dbg("get hardware client : ap3425 \n");		
			IRsensor_hw_client = IRsensor_hw_ap3425_getHardware();
			break;

		default:
			err("get hardware client ERROR : hardware enum=%d\n", hardware_source);
	}

	return 	IRsensor_hw_client;
}

struct i2c_driver* IRsensor_hw_setI2cDriver(int hardware_source, 
	struct i2c_driver* IRsensor_i2c_driver_client)
{
	switch(hardware_source) {
		case IRsensor_hw_source_cm36686:
			dbg("set i2c client : cm36686 \n");
			IRsensor_i2c_driver_client->driver.name = "cm36686";
			IRsensor_i2c_driver_client->driver.owner = THIS_MODULE;
			IRsensor_i2c_driver_client->driver.of_match_table = cm36686_match_table;
			IRsensor_i2c_driver_client->id_table = cm36686_i2c_id;
			break;

		case IRsensor_hw_source_ap3425:
			dbg("set i2c client : ap3425 \n");	
			IRsensor_i2c_driver_client->driver.name = "ap3425";
			IRsensor_i2c_driver_client->driver.owner = THIS_MODULE;
			IRsensor_i2c_driver_client->driver.of_match_table = ap3425_match_table;
			IRsensor_i2c_driver_client->id_table = ap3425_i2c_id;
			break;

		default:
			err("get hardware client ERROR : hardware enum=%d\n", hardware_source);
	}
	
	return IRsensor_i2c_driver_client;
}

/**************************/
/*i2c read/write function*/
/************************/
uint8_t i2c_read_reg_u8(struct i2c_client* client, u8 reg)
{
	uint8_t data =0 ;
	
	if(client == NULL) {
		err("i2c_read_reg_u8 ERROR. (i2c client is NULL)\n");
		return -1;
	}
	
	data = i2c_smbus_read_byte_data(client, reg);
	if (data < 0) {
		err("i2c_read_reg_u8 ERROR. (i2c_smbus_read_byte_data : 0x%02X)\n", reg);			
	}
	
	return data;
}

int i2c_write_reg_u8(struct i2c_client* client, u8 reg, uint8_t data)
{
	int ret = 0;

	if(client == NULL) {
		err("i2c_write_reg_u8 ERROR. (i2c client is NULL)\n");
		return -1;
	}
	
	ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0) {
		err("i2c_write_reg_u8 ERROR. (i2c_smbus_write_byte_data : 0x%02X)\n", reg);	
		return ret;
	}
	
	return 0;
}

int i2c_read_reg_u16(struct i2c_client* client, u8 reg, uint8_t* data)
{
	int ret = 0;	
	struct i2c_msg msg[] = {
		{
		    .addr = client->addr,
		    .flags = 0, //write
		    .len = 1,
		    .buf = &reg,
		},
		{
		    .addr = client->addr,
		    .flags = I2C_M_RD, //read
		    .len = 2,
		    .buf = data,
		}
	};

	if(client == NULL) {
		err("i2c_read_reg_u16 ERROR. (i2c client is NULL)\n");
		return -1;
	}
	
	if (!client->adapter) {
	    return -ENODEV;
	}
	memset(&data, 0, sizeof(data));
	
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	/*return 2 is expected.*/
	if (ret != ARRAY_SIZE(msg)) {
		err("i2c_read_reg_u16 ERROR. (i2c_transfer:0x%0X)\n", reg);
		return -1;
	}

	return 0; 
}

int i2c_write_reg_u16(struct i2c_client* client, u8 reg, uint8_t* data)
{
	int ret = 0;	
	int len = 3;
	uint8_t buf[len];
	struct i2c_msg msg;

	if(client == NULL) {
		err("i2c_write_reg_u16 ERROR. (i2c client is NULL)\n");
		return -1;
	}

	if(data == NULL) {
		err("i2c_write_reg_u16 ERROR. (data is NULL)\n");
		return -1;
	}	
	
	msg.addr = client->addr;
	msg.flags = 0; /*write*/
	msg.len = len;
	msg.buf = buf;

	if (!client->adapter)
		return -ENODEV;

	buf[0] = reg;
	memcpy(buf + 1, &data[0], sizeof(data[0]));
	memcpy(buf + 2, &data[1], sizeof(data[1]));

	ret = i2c_transfer(client->adapter, &msg, 1);
	/*return postive is expected.*/
	if(ret < 0){
		err("i2c_write_reg_u16 ERROR. (reg=0x%x, data_l=%d, data_h=%d, err = 0x%x)\n", reg, data[0], data[1], ret);
		return ret;
	}

    return 0; 
}