/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/export.h>
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define FLASH_NAME "qcom,led-flash"

#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define LM3642_DBG(fmt, args...) pr_err(fmt, ##args)
#else
#define LM3642_DBG(fmt, args...)
#endif


static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver lm3642_i2c_driver;

static struct mutex flash_lock;
static struct mutex brightness_lock;
static enum msm_camera_led_config_t brightness_state;
static int last_brightness_value = 0;
static unsigned long flash_start_time;

static struct msm_camera_i2c_reg_array lm3642_init_array[] = {
	{0x10, 0x00},//enable register 0 0 0 1 1 0 0 0
	{0xA0, 0x14},//torch brightness register 0 0 |0 1 0 | 1 0 0, 28.125mA 84.375mA, 140.625mA
	{0xB0, 0x28},//flash brightness register 0 0 1 0|1 0 0 0,  56.25mA 168.75mA 506.25mA
	{0xC0, 0x7F},//flash duration register   0 |1 1| 1 1 1 1 1, 32ms Boost Peak current limit is 3.2A, timeout is  1024ms
	{0xE0, 0x68},//configuration register 1,  0 1 1 0 1 0 0 0, Torch pin disable, FLEN pin disable, TX2 set to input, FLEN active high, 
	{0xF0, 0x00},//configuration register 2,  0 0 0 0 0 0 0 0, last two bit, NTC shutdown TX2 shutdown
};

static struct msm_camera_i2c_reg_array lm3642_off_array[] = {
	{0x10, 0x00}
};

static struct msm_camera_i2c_reg_array lm3642_release_array[] = {
	{0x10, 0x00}
};

static struct msm_camera_i2c_reg_array lm3642_low_array[] = {
	{0x10, 0x1A},//enable register 0 0 0 1 1 0 1 0, torch mode
};

static struct msm_camera_i2c_reg_array lm3642_low_array_led1[] = {
	{0x10, 0x0A},//enable register 0 0 0 0 1 0 1 0, torch mode
};

static struct msm_camera_i2c_reg_array lm3642_low_array_led2[] = {
	{0x10, 0x12},//enable register 0 0 0 1 0 0 1 0, torch mode
};

static struct msm_camera_i2c_reg_array lm3642_high_array[] = {
	{0x10, 0x1B},//enable register 0 0 0 1 1 0 1 1, flash mode
};

static struct msm_camera_i2c_reg_array lm3642_high_array_led1[] = {
	{0x10, 0x0B},//enable register 0 0 0 0 1 0 1 1, flash mode
};

static struct msm_camera_i2c_reg_array lm3642_high_array_led2[] = {
	{0x10, 0x13},//enable register 0 0 0 1 0 0 1 1, flash mode
};


static const struct of_device_id lm3642_i2c_trigger_dt_match[] = {
	{.compatible = "qcom,led-flash", .data = &fctrl},
	{}
};

MODULE_DEVICE_TABLE(of, lm3642_i2c_trigger_dt_match);
static const struct i2c_device_id lm3642_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};
/*
static void msm_led_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	pr_info("[S-DEBUG]%s:%d called\n", __func__, __LINE__);
	if (value > LED_OFF) {
		if(fctrl.func_tbl->flash_led_low)
			fctrl.func_tbl->flash_led_low(&fctrl);
	} else {
		if(fctrl.func_tbl->flash_led_off)
			fctrl.func_tbl->flash_led_off(&fctrl);
	}
};

static struct led_classdev msm_torch_led = {
	.name			= "torch-light",
	.brightness_set	= msm_led_torch_brightness_set,
	.brightness		= LED_OFF,
};

static int32_t msm_lm3642_torch_create_classdev(struct device *dev ,
				void *data)
{
	int rc;
	pr_info("[S-DEBUG]%s:%d called\n", __func__, __LINE__);
	msm_led_torch_brightness_set(&msm_torch_led, LED_OFF);
	rc = led_classdev_register(dev, &msm_torch_led);
	if (rc) {
		pr_err("Failed to register led dev. rc = %d\n", rc);
		return rc;
	}

	return 0;
};
*/
//#define DUMP_REG
#ifdef DUMP_REG
static uint32_t dump_reg[] =
{
	0x10,
	0xA0,
	0xB0,
	0xC0,
	0xD0,
	0xE0,
	0xF0,
	0x81,
	0x30,
	0x31,
	0x80,
	0xFF
};
static uint16_t reg_val[12];

static void dump_register(struct msm_led_flash_ctrl_t *fctrl, uint32_t * reg_addr, uint16_t * reg_val, int size)
{
	int rc = 0;
	int i;
	pr_info("czw dump_register size is %d\n",size);
	if(fctrl->flash_i2c_client)
	{
		for(i=0;i<size;i++)
		{
			rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(
			 fctrl->flash_i2c_client,
			 reg_addr[i],
			 &reg_val[i],
			 MSM_CAMERA_I2C_BYTE_DATA
			);
			if(rc < 0)
			{
				pr_err("czw read reg 0x%02X ERROR!\n",reg_addr[i]);
			}
			else
			{
				pr_info("czw read reg 0x%02X --> 0x%02X\n",reg_addr[i],reg_val[i]);
			}
		}
	}

}
#endif

#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>

#define I2C_TEST_LEDFLASH_FAIL (-1)

static int TestLEDFlashI2C(struct i2c_client *apClient)
{
	int lnResult = I2C_TEST_PASS;
	int rc = 0;
	uint16_t chipid = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	i2c_log_in_test_case("TestLEDFlashI2C ++\n");

	power_info = &(fctrl.flashdata->power_info);

	mutex_lock(&flash_lock);
	if(brightness_state == MSM_CAMERA_LED_RELEASE)//power off
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);
		msleep(1);
		pr_err("czw, flash full test");
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_read(
			fctrl.flash_i2c_client, fctrl.flashdata->slave_info->sensor_id_reg_addr ,
			&chipid, MSM_CAMERA_I2C_BYTE_DATA);

		if (rc < 0) {
			pr_err("%s: flash: read id failed\n", __func__);
			lnResult = I2C_TEST_LEDFLASH_FAIL;
		}

		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_LOW);
	}
	else //power on
	{
		pr_err("czw, flash read test");
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_read(
			fctrl.flash_i2c_client, fctrl.flashdata->slave_info->sensor_id_reg_addr ,
			&chipid, MSM_CAMERA_I2C_BYTE_DATA);

		if (rc < 0) {
			pr_err("%s: flash: read id failed\n", __func__);
			lnResult = I2C_TEST_LEDFLASH_FAIL;
		}
	}
	mutex_unlock(&flash_lock);

	i2c_log_in_test_case("TestLEDFlashI2C --\n");
	return lnResult;
}

static struct i2c_test_case_info LEDFlashTestCaseInfo[] =
{
     __I2C_STRESS_TEST_CASE_ATTR(TestLEDFlashI2C),
};
#endif


#ifdef TABLE_MAPPING
/*
#define FLASH_MAX_LEVEL 13//56.25x13=731.25  max 16

#define FLASH_DURATION_MAX_LEVEL 32

#define TORCH_MAX_LEVEL 2// 28.125x2=56.25   max 8

#define TORCH_LIGHT_MAX_LEVEL 4// 28.125x4=112.25 max 8
*/
typedef struct
{
	uint8_t flash_current;
	uint8_t flash_duration;
	uint8_t torch_current;
	uint8_t light_current;
}reg_map_table_t;

// 0~100, 0 means off
// index 0 ~ 100
static reg_map_table_t register_map_table[] =
{
	{ 0,  0, 0, 0},//for stub, 0 means off, no need use this table
	{ 0,  0, 0, 0},
	{ 0,  0, 0, 0},
	{ 0,  0, 0, 0},
	{ 0,  0, 0, 0},
	{ 0,  1, 0, 0},
	{ 0,  1, 0, 0},
	{ 0,  1, 0, 0},
	{ 1,  2, 0, 0},
	{ 1,  2, 0, 0},
	{ 1,  2, 0, 0},
	{ 1,  3, 0, 0},
	{ 1,  3, 0, 0},
	{ 1,  3, 0, 0},
	{ 1,  3, 0, 0},
	{ 1,  4, 0, 0},
	{ 2,  4, 0, 0},
	{ 2,  4, 0, 0},
	{ 2,  5, 0, 0},
	{ 2,  5, 0, 0},
	{ 2,  5, 0, 0},
	{ 2,  6, 0, 0},
	{ 2,  6, 0, 0},
	{ 2,  6, 0, 0},
	{ 3,  7, 0, 0},
	{ 3,  7, 0, 0},
	{ 3,  7, 0, 1},
	{ 3,  8, 0, 1},
	{ 3,  8, 0, 1},
	{ 3,  8, 0, 1},
	{ 3,  9, 0, 1},
	{ 3,  9, 0, 1},
	{ 4,  9, 0, 1},
	{ 4, 10, 0, 1},
	{ 4, 10, 0, 1},
	{ 4, 10, 0, 1},
	{ 4, 11, 0, 1},
	{ 4, 11, 0, 1},
	{ 4, 11, 0, 1},
	{ 4, 11, 0, 1},
	{ 5, 12, 0, 1},
	{ 5, 12, 0, 1},
	{ 5, 12, 0, 1},
	{ 5, 13, 0, 1},
	{ 5, 13, 0, 1},
	{ 5, 13, 0, 1},
	{ 5, 14, 0, 1},
	{ 5, 14, 0, 1},
	{ 6, 14, 0, 1},
	{ 6, 15, 0, 1},
	{ 6, 15, 0, 1},
	{ 6, 15, 1, 2},
	{ 6, 16, 1, 2},
	{ 6, 16, 1, 2},
	{ 6, 16, 1, 2},
	{ 6, 17, 1, 2},
	{ 7, 17, 1, 2},
	{ 7, 17, 1, 2},
	{ 7, 18, 1, 2},
	{ 7, 18, 1, 2},
	{ 7, 18, 1, 2},
	{ 7, 19, 1, 2},
	{ 7, 19, 1, 2},
	{ 7, 19, 1, 2},
	{ 8, 19, 1, 2},
	{ 8, 20, 1, 2},
	{ 8, 20, 1, 2},
	{ 8, 20, 1, 2},
	{ 8, 21, 1, 2},
	{ 8, 21, 1, 2},
	{ 8, 21, 1, 2},
	{ 8, 22, 1, 2},
	{ 9, 22, 1, 2},
	{ 9, 22, 1, 2},
	{ 9, 23, 1, 2},
	{ 9, 23, 1, 2},
	{ 9, 23, 1, 3},
	{ 9, 24, 1, 3},
	{ 9, 24, 1, 3},
	{ 9, 24, 1, 3},
	{10, 25, 1, 3},
	{10, 25, 1, 3},
	{10, 25, 1, 3},
	{10, 26, 1, 3},
	{10, 26, 1, 3},
	{10, 26, 1, 3},
	{10, 27, 1, 3},
	{11, 27, 1, 3},
	{11, 27, 1, 3},
	{11, 27, 1, 3},
	{11, 28, 1, 3},
	{11, 28, 1, 3},
	{11, 28, 1, 3},
	{11, 29, 1, 3},
	{12, 29, 1, 3},
	{12, 29, 1, 3},
	{12, 30, 1, 3},
	{12, 30, 1, 3},
	{12, 30, 1, 3},
	{12, 31, 1, 3},
	{12, 31, 1, 3},
};

int msm_flash_lm3642_light_current_setting(int strength1, int strength2)//it's torch, so no need set duration
{
	int rc = 0;
	uint8_t current_level1, current_level2;
	uint16_t current_reg_val;
	//strength 0~100
	//28.125mA, 3bit, 8 level
	//100 vs 8 , 25/2
	//int current_level1 = strength1*2/25;
	//int current_level2 = strength2*2/25;
	if(strength1>100)
		strength1=100;
	if(strength2>100)
		strength2=100;
	if(strength1<1)
		strength1=1;
	if(strength2<1)
		strength2=1;

	current_level1 = register_map_table[strength1].light_current;
	current_level2 = register_map_table[strength2].light_current;

	current_reg_val = (0b00<<6)|(current_level2<<3)|current_level1;
	mutex_lock(&flash_lock);
	if(fctrl.flash_i2c_client)
	{
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
				 fctrl.flash_i2c_client,
				 0xA0,//torch brightness register
				 current_reg_val,
				 MSM_CAMERA_I2C_BYTE_DATA
			);
		if(rc < 0)
		{
			pr_err("czw light_current_setting write 0xA0 reg ERROR!\n");
		}
	}
	else
	{
		pr_err("czw light_current_setting flash_i2c_client not usable ?!\n");
		rc = -1;
	}
	pr_err("czw light strength1 %d -> %d, strength2 %d -> %d, reg is 0x%02X\n",strength1,current_level1,strength2,current_level2,current_reg_val);
	mutex_unlock(&flash_lock);
	return rc;
}
int msm_flash_lm3642_flash_current_setting(int strength1, int strength2)
{
	int rc = 0;
	uint8_t current_level1,current_level2;
	uint16_t current_reg_val;
	//strength 0~100
	//56.25mA, 4 bit, 16 level
	//16 vs 100, 6.25  4/25
	//int current_level1 = strength1*4/25;
	//int current_level2 = strength2*4/25;
	if(strength1>100)
		strength1=100;
	if(strength2>100)
		strength2=100;
	if(strength1<1)
		strength1=1;
	if(strength2<1)
		strength2=1;



	current_level1 = register_map_table[strength1].flash_current;
	current_level2 = register_map_table[strength2].flash_current;

	current_reg_val = (current_level2<<4)|current_level1;//2|1;
	mutex_lock(&flash_lock);
	if(fctrl.flash_i2c_client)
	{
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
			 fctrl.flash_i2c_client,
			 0xB0,//flash brightness register
			 current_reg_val,
			 MSM_CAMERA_I2C_BYTE_DATA
			);
		if(rc < 0)
		{
			pr_err("czw flash_current_setting write 0xB0 reg ERROR!\n");
		}
	}
	else
	{
		pr_err("czw flash_current_setting flash_i2c_client not usable ?!\n");
		rc = -1;
	}
	pr_err("czw flash strength1 %d -> %d, strength2 %d -> %d, reg is 0x%02X\n",strength1,current_level1,strength2,current_level2,current_reg_val);
	mutex_unlock(&flash_lock);
	return rc;

}
EXPORT_SYMBOL(msm_flash_lm3642_flash_current_setting);

static int msm_flash_lm3642_flash_duration_setting(int duration)
{
	int rc = 0;
	uint8_t duration_level;
	uint16_t duration_reg_val;
	//32ms, 5 bit, 32 level
	//32 vs 100, 25/8
	//int duration_level = duration*4/25;

	if(duration>100)
		duration=100;
	if(duration<0)
		duration=0;

	duration_level = register_map_table[duration].flash_duration;

	duration_reg_val = (0b011<<5)|duration_level;
	mutex_lock(&flash_lock);
	if(fctrl.flash_i2c_client)
	{
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
				 fctrl.flash_i2c_client,
				 0xC0,//flash duration register
				 duration_reg_val,
				 MSM_CAMERA_I2C_BYTE_DATA
			);
		if(rc < 0)
		{
			pr_err("czw flash_duration_setting write 0xC0 ERROR!\n");
		}
	}
	else
	{
		pr_err("czw flash_duration_setting flash_i2c_client not usable ?!\n");
		rc = -1;
	}
	pr_info("flash duration_level %d ->%d, reg is 0x%02X\n",duration,duration_level,duration_reg_val);
	mutex_unlock(&flash_lock);
	return rc;
}

int msm_flash_lm3642_torch_current_setting(int strength1, int strength2)//it's torch, so no need set duration
{
	int rc = 0;
	uint8_t current_level1, current_level2;
	uint16_t current_reg_val;
	//strength 0~100
	//28.125mA, 3bit, 8 level
	//100 vs 8 , 25/2
	//int current_level1 = strength1*2/25;
	//int current_level2 = strength2*2/25;

	if(strength1>100)
		strength1=100;
	if(strength2>100)
		strength2=100;
	if(strength1<1)
		strength1=1;
	if(strength2<1)
		strength2=1;

	current_level1 = register_map_table[strength1].torch_current;
	current_level2 = register_map_table[strength2].torch_current;

	current_reg_val = (0b00<<6)|(current_level2<<3)|current_level1;
	mutex_lock(&flash_lock);
	if(fctrl.flash_i2c_client)
	{
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
				 fctrl.flash_i2c_client,
				 0xA0,//torch brightness register
				 current_reg_val,
				 MSM_CAMERA_I2C_BYTE_DATA
			);
		if(rc < 0)
		{
			pr_err("czw torch_current_setting write 0xA0 reg ERROR!\n");
		}
	}
	else
	{
		pr_err("czw torch_current_setting flash_i2c_client not usable ?!\n");
		rc = -1;
	}
	pr_err("czw torch strength1 %d -> %d, strength2 %d -> %d, reg is 0x%02X\n",strength1,current_level1,strength2,current_level2,current_reg_val);
	mutex_unlock(&flash_lock);
	return rc;
}
EXPORT_SYMBOL(msm_flash_lm3642_torch_current_setting);

#endif

#define MAX_FLASH_CURRENT 900
#define MAX_TORCH_CURRENT 225
#define MAX_FLASH_DURATION 1024

#define FLASH_INTERVAL 56
#define TORCH_INTERVAL 28
#define DURATION_INTERVAL 32

static uint8_t flash_current_to_reg_level(int set_current_val)
{
	if(set_current_val<FLASH_INTERVAL)
		set_current_val=FLASH_INTERVAL;
	else if(set_current_val>MAX_FLASH_CURRENT)
		set_current_val=MAX_FLASH_CURRENT;

	return (set_current_val+FLASH_INTERVAL/2)*4/225-1;
}

static uint8_t torch_current_to_reg_level(int set_current_val)
{
	if(set_current_val<TORCH_INTERVAL)
		set_current_val=TORCH_INTERVAL;
	else if(set_current_val>MAX_TORCH_CURRENT)
		set_current_val=MAX_TORCH_CURRENT;

	return (set_current_val+TORCH_INTERVAL/2)*8/225-1;
}

static uint8_t flash_duration_to_reg_level(int duration)
{
	if(duration<DURATION_INTERVAL)
		duration=DURATION_INTERVAL;
	else if(duration>MAX_FLASH_DURATION)
		duration=MAX_FLASH_DURATION;

	return (duration+DURATION_INTERVAL/2)/DURATION_INTERVAL-1;
}

int msm_flash_lm3642_flash_current_setting(int current1, int current2)
{
	int rc = 0;
	uint8_t current_level1,current_level2;
	uint16_t current_reg_val;

	current_level1 = flash_current_to_reg_level(current1);
	current_level2 = flash_current_to_reg_level(current2);

	current_reg_val = (current_level2<<4)|current_level1;//2|1;
	mutex_lock(&flash_lock);
	if(fctrl.flash_i2c_client)
	{
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
			 fctrl.flash_i2c_client,
			 0xB0,//flash brightness register
			 current_reg_val,
			 MSM_CAMERA_I2C_BYTE_DATA
			);
		if(rc < 0)
		{
			pr_err("czw flash_current_setting write 0xB0 reg ERROR!\n");
		}
	}
	else
	{
		pr_err("czw flash_current_setting flash_i2c_client not usable ?!\n");
		rc = -1;
	}
	pr_info("czw flash current1 %d -> %d, current2 %d -> %d, reg is 0x%02X\n",current1,current_level1,current2,current_level2,current_reg_val);
	mutex_unlock(&flash_lock);
	return rc;
}
EXPORT_SYMBOL(msm_flash_lm3642_flash_current_setting);

int msm_flash_lm3642_torch_current_setting(int current1, int current2)
{
	int rc = 0;
	uint8_t current_level1, current_level2;
	uint16_t current_reg_val;

	current_level1 = torch_current_to_reg_level(current1);
	current_level2 = torch_current_to_reg_level(current2);

	current_reg_val = (0b00<<6)|(current_level2<<3)|current_level1;
	mutex_lock(&flash_lock);
	if(fctrl.flash_i2c_client)
	{
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
				 fctrl.flash_i2c_client,
				 0xA0,//torch brightness register
				 current_reg_val,
				 MSM_CAMERA_I2C_BYTE_DATA
			);
		if(rc < 0)
		{
			pr_err("czw torch_current_setting write 0xA0 reg ERROR!\n");
		}
	}
	else
	{
		pr_err("czw torch_current_setting flash_i2c_client not usable ?!\n");
		rc = -1;
	}
	pr_info("czw torch current1 %d -> %d, current2 %d -> %d, reg is 0x%02X\n",current1,current_level1,current2,current_level2,current_reg_val);
	mutex_unlock(&flash_lock);
	return rc;
}
EXPORT_SYMBOL(msm_flash_lm3642_torch_current_setting);

static int msm_flash_lm3642_flash_duration_setting(int duration)
{
	int rc = 0;
	uint8_t duration_level;
	uint16_t duration_reg_val;

	duration_level = flash_duration_to_reg_level(duration);

	duration_reg_val = (0b011<<5)|duration_level;
	mutex_lock(&flash_lock);
	if(fctrl.flash_i2c_client)
	{
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
				 fctrl.flash_i2c_client,
				 0xC0,//flash duration register
				 duration_reg_val,
				 MSM_CAMERA_I2C_BYTE_DATA
			);
		if(rc < 0)
		{
			pr_err("czw flash_duration_setting write 0xC0 ERROR!\n");
		}
	}
	else
	{
		pr_err("czw flash_duration_setting flash_i2c_client not usable ?!\n");
		rc = -1;
	}
	pr_info("flash duration %d ->%d, reg is 0x%02X\n",duration,duration_level,duration_reg_val);
	mutex_unlock(&flash_lock);
	return rc;
}


void msm_flash_lm3642_get_last_flash_current(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	uint16_t read_val;
	uint32_t reg_addr = 0x81;
	unsigned char led1,led2;
	mutex_lock(&flash_lock);
	if(fctrl->flash_i2c_client)
	{
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(
			 fctrl->flash_i2c_client,
			 reg_addr,
			 &read_val,
			 MSM_CAMERA_I2C_BYTE_DATA
			);
		if(rc < 0)
		{
			pr_err("czw get_last_flash_current  ERROR!\n");
		}
		else
		{
			pr_info("czw last flash current val is 0x%02X\n",read_val);
			led1 = read_val&0x0F;
			led2 = (read_val>>4)&0x0F;
			pr_info("czw last flash current, led1 is %d, led2 is %d\n",led1,led2);
		}
	}
	mutex_unlock(&flash_lock);
}

//#define GPIO_CONTROL
int msm_flash_lm3642_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	mutex_lock(&flash_lock);
	pr_info("czw msm_flash_lm3642_led INIT!\n");
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

#ifdef GPIO_CONTROL
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
#endif
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);
	msleep(1);
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
		{
			pr_err("%s:%d failed\n", __func__, __LINE__);
			pr_err("czw msm_flash_lm3642_led INIT ERROR!\n");
		}
		else
		{
			brightness_state = MSM_CAMERA_LED_INIT;
		}
	}
	mutex_unlock(&flash_lock);
	return rc;
}

int msm_flash_lm3642_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	mutex_lock(&flash_lock);
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	pr_info("czw msm_flash_lm3642_led RELEASE!\n");

#ifdef GPIO_CONTROL
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_LOW);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
#endif
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->release_setting);
		if (rc < 0)
		{
			pr_err("%s:%d failed\n", __func__, __LINE__);
			pr_err("czw msm_flash_lm3642_led RELEASE ERROR!\n");
		}
		else
		{
			brightness_state = MSM_CAMERA_LED_RELEASE;
		}
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_LOW);

	mutex_unlock(&flash_lock);
	return rc;
}

int msm_flash_lm3642_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	mutex_lock(&flash_lock);
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	pr_info("czw msm_flash_lm3642_led OFF!\n");


	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
		{
			pr_err("czw msm_flash_lm3642_led OFF ERROR!\n");
			pr_err("%s:%d failed\n", __func__, __LINE__);//failed!
		}
		else
		{
			brightness_state = MSM_CAMERA_LED_OFF;
		}
	}
#ifdef GPIO_CONTROL
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_LOW);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
#endif
	mutex_unlock(&flash_lock);
	return rc;
}

int msm_flash_lm3642_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	mutex_lock(&flash_lock);
	pr_info("czw msm_flash_lm3642_led LOW!\n");
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
#ifdef GPIO_CONTROL
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
#endif
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {

		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
		fctrl->flash_i2c_client,
		fctrl->reg_setting->low_setting);

		if (rc < 0)
		{
			pr_err("%s:%d failed\n", __func__, __LINE__);
			pr_err("czw msm_flash_lm3642_led LOW ERROR!\n");
		}
	}
	mutex_unlock(&flash_lock);
	return rc;
}

int msm_flash_lm3642_led_low_led1(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	mutex_lock(&flash_lock);
	pr_info("czw msm_flash_lm3642_led LOW LED1!\n");
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
#ifdef GPIO_CONTROL
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
#endif
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {

		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
		fctrl->flash_i2c_client,
		fctrl->reg_setting->low_first_setting);

		if (rc < 0)
		{
			pr_err("%s:%d failed\n", __func__, __LINE__);
			pr_err("czw msm_flash_lm3642_led LOW LED1 ERROR!\n");
		}
		else
		{
			brightness_state = MSM_CAMERA_LED_LOW;
		}
	}
	mutex_unlock(&flash_lock);
	return rc;
}
int msm_flash_lm3642_led_low_led2(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	mutex_lock(&flash_lock);
	pr_info("czw msm_flash_lm3642_led LOW LED2!\n");
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
#ifdef GPIO_CONTROL
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
#endif
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {

		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
		fctrl->flash_i2c_client,
		fctrl->reg_setting->low_second_setting);

		if (rc < 0)
		{
			pr_err("%s:%d failed\n", __func__, __LINE__);
			pr_err("czw msm_flash_lm3642_led LOW LED2 ERROR!\n");
		}
	}
	mutex_unlock(&flash_lock);
	return rc;
}
int msm_flash_lm3642_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	struct timeval start_time;

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	mutex_lock(&flash_lock);

	pr_info("czw msm_flash_lm3642_led HIGH!\n");

	flashdata = fctrl->flashdata;

	power_info = &flashdata->power_info;
#ifdef GPIO_CONTROL
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
#endif
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {

		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);

		if (rc < 0)
		{
			pr_err("%s:%d failed\n", __func__, __LINE__);
			pr_err("czw msm_flash_lm3642_led HIGH ERROR!\n");
		}
	}
	do_gettimeofday(&start_time);
	flash_start_time = start_time.tv_sec*1000 + start_time.tv_usec/1000;

	mutex_unlock(&flash_lock);
	return rc;
}

int msm_flash_lm3642_led_high_led1(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	struct timeval start_time;

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	mutex_lock(&flash_lock);
	pr_info("czw msm_flash_lm3642_led HIGH LED1!\n");

	flashdata = fctrl->flashdata;

	power_info = &flashdata->power_info;
#ifdef GPIO_CONTROL
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
#endif
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {

		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_first_setting);

		if (rc < 0)
		{
			pr_err("%s:%d failed\n", __func__, __LINE__);
			pr_err("czw msm_flash_lm3642_led HIGH LED1 ERROR!\n");
		}
	}
	do_gettimeofday(&start_time);
	flash_start_time = start_time.tv_sec*1000 + start_time.tv_usec/1000;

	mutex_unlock(&flash_lock);
	return rc;
}
int msm_flash_lm3642_led_high_led2(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	struct timeval start_time;

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	mutex_lock(&flash_lock);
	pr_info("czw msm_flash_lm3642_led HIGH LED2!\n");

	flashdata = fctrl->flashdata;

	power_info = &flashdata->power_info;
#ifdef GPIO_CONTROL
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
#endif
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {

		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_second_setting);

		if (rc < 0)
		{
			pr_err("%s:%d failed\n", __func__, __LINE__);
			pr_err("czw msm_flash_lm3642_led HIGH LED2 ERROR!\n");
		}
	}
	do_gettimeofday(&start_time);
	flash_start_time = start_time.tv_sec*1000 + start_time.tv_usec/1000;

	mutex_unlock(&flash_lock);
	return rc;
}
//ASUS_BSP Sam +++ "Porting Flash"

int msm_flash_lm3642_led_control(int id, int mode)
{
	int rc = 0;

	if(id == 1)
	{
		if(mode == 0)
		{
			rc = msm_flash_lm3642_led_low_led1(&fctrl);
		}
		else if(mode == 1)
		{
			rc = msm_flash_lm3642_led_high_led1(&fctrl);
		}
	    else
	    {
			pr_err("msm_flash_lm3642_led_control invalid mode %d for LED1!\n",mode);
			rc = -1;
		}
	}
	else if(id == 2)
	{
		if(mode == 0)
		{
			rc = msm_flash_lm3642_led_low_led2(&fctrl);
		}
		else if(mode == 1)
		{
			rc = msm_flash_lm3642_led_high_led2(&fctrl);
		}
	    else
	    {
			pr_err("msm_flash_lm3642_led_control invalid mode %d for LED2!\n",mode);
			rc = -1;
		}
	}
	else
	{
		pr_err("msm_flash_lm3642_led_control invalid ID %d\n",id);
		rc = -1;
	}

	return rc;
}


#define        STATUS_PROC_FILE        "driver/flash_status"
static struct proc_dir_entry *status_proc_file;
static int ATD_status;
static int status_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%d\n", ATD_status);
    ATD_status = 0;
    return 0;
}

static int status_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, status_proc_read, NULL);
}


static const struct file_operations status_fops = {
       .owner = THIS_MODULE,
       .open = status_proc_open,
       .read = seq_read,
       //.write = status_proc_write,
       .llseek = seq_lseek,
       .release = single_release,
};

static void create_status_proc_file(void)
{
    status_proc_file = proc_create(STATUS_PROC_FILE, 0666, NULL, &status_fops);
    if (status_proc_file) {
       printk("[S-DEBUG]%s sucessed!\n", __func__);
    } else {
       printk("[S-DEBUG]%s failed!\n", __func__);
    }
}


/*#define      ASUS_FLASH_PROC_FILE    "driver/asus_flash"
static struct proc_dir_entry *asus_flash_proc_file;
static int asus_flash_value;*/

static ssize_t asus_flash_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{
       return 0;
}

static ssize_t asus_flash_store(struct file *dev, const char *buf, size_t count, loff_t *loff)
{
       int mode = -1, set_val = -1;
       int set_val1 = -1;

       sscanf(buf, "%d %d", &mode, &set_val);
       pr_info("[AsusFlash]flash mode=%d value=%d\n", mode, set_val);
       msm_flash_lm3642_led_init(&fctrl);

       if(mode == 0) {
               if (set_val < 0 || set_val > 250 || set_val == 1) {
                       ATD_status = 1;
                       msm_flash_lm3642_led_low_led1(&fctrl);
               } else if (set_val == 0 ) {
                       ATD_status = 1;
                       msm_flash_lm3642_led_release(&fctrl);
               } else if(0 < set_val && set_val < 251) {
                       printk(KERN_INFO "[AsusFlash] current now in 1~250");
               } else {
                       msm_flash_lm3642_led_release(&fctrl);
                       return -1;
               }
       } else if(mode == 1) {
               if (set_val == 1 || set_val < 0 || set_val > 1500) {
                       ATD_status = 1;
                       msm_flash_lm3642_led_high_led1(&fctrl);
               } else if (set_val == 0) {
                       ATD_status = 1;
                       msm_flash_lm3642_led_release(&fctrl);
               } else if (0 < set_val && set_val < 1501) {
                       printk(KERN_INFO "[AsusFlash] Flash current now in 1~1500");
               } else {
                       msm_flash_lm3642_led_release(&fctrl);
                       return -1;
               }
       }
       else if(mode == 2)//LED1 LED2 control
       {
		    sscanf(buf, "%d %d %d", &mode, &set_val,&set_val1);
		    msm_flash_lm3642_led_control(set_val,set_val1);
	   }
	   else if(mode == 3)//flash duration stress test
	   {
		   msm_flash_lm3642_flash_duration_setting(set_val);
		   msm_flash_lm3642_led_high(&fctrl);
	   }
	   else if(mode == 4)//flash strength stress test
	   {
		   sscanf(buf, "%d %d %d", &mode,&set_val, &set_val1);
		   msm_flash_lm3642_flash_current_setting(set_val,set_val1);
		   msm_flash_lm3642_led_high(&fctrl);
	   }
	   else if(mode == 5)//torch stress test
	   {
		   sscanf(buf, "%d %d %d", &mode,&set_val, &set_val1);
		   msm_flash_lm3642_torch_current_setting(set_val,set_val1);
		   msm_flash_lm3642_led_low(&fctrl);
	   }
	   else if(mode == 6)//torch LED2
	   {
		    sscanf(buf, "%d %d", &mode,&set_val);
		    msm_flash_lm3642_torch_current_setting(set_val,set_val);
		    msm_flash_lm3642_led_low_led2(&fctrl);
	   }
       else {
               return -1;
       }

       return count;
}


static ssize_t asus_led_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{
       return 0;
}

static ssize_t asus_led_store(struct file *dev, const char *buf, size_t count, loff_t *loff)
{
       int mode = -1, set_val = -1;

       sscanf(buf, "%d %d", &mode, &set_val);
       pr_info("[AsusFlash]flash mode=%d value=%d\n", mode, set_val);
       msm_flash_lm3642_led_init(&fctrl);

       if(mode == 0) {
               if (set_val < 0 || set_val > 250 || set_val == 1) {
                       ATD_status = 1;
                       msm_flash_lm3642_led_low_led2(&fctrl);
               } else if (set_val == 0 ) {
                       ATD_status = 1;
                       msm_flash_lm3642_led_release(&fctrl);
               } else if(0 < set_val && set_val < 251) {
                       printk(KERN_INFO "[AsusFlash] current now in 1~250");
               } else {
                       msm_flash_lm3642_led_release(&fctrl);
                       return -1;
               }

       } else if(mode == 1) {
               if (set_val == 1 || set_val < 0 || set_val > 1500) {
                       ATD_status = 1;
                       msm_flash_lm3642_led_high_led2(&fctrl);
               } else if (set_val == 0) {
                       ATD_status = 1;
                       msm_flash_lm3642_led_release(&fctrl);
               } else if (0 < set_val && set_val < 1501) {
                       printk(KERN_INFO "[AsusFlash] Flash current now in 1~1500");
               } else {
                       msm_flash_lm3642_led_release(&fctrl);
                       return -1;
               }
       }
       else {
               return -1;
       }
       return count;
}

static ssize_t asus_brightness_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{
	int n;
	char messages[8];

	//pr_info("asus_brightness_show POS = %lld\n",*ppos);

	if(*ppos == 0)
	{
		n = snprintf(messages, sizeof(messages),"%d\n%c", last_brightness_value,'\0');
		if(copy_to_user(buffer,messages,n))
		{
			pr_err("copy_to_user fail !!\n");
			return -EFAULT;
		}
		else
		{
			*ppos += n;
			//pr_info("asus_brightness_show RETURN %d\n",n);
			return n;
		}
	}
	else
	{
		//pr_info("asus_brightness_show RETURN 0\n");
		return 0;
	}
}

static ssize_t asus_brightness_store(struct file *dev, const char *buff, size_t count, loff_t *loff)
{
	int set_val = -1;
	int real_set_brightness = -1;
	int MAX_FLASHLIGHT_CURRENT = 112;
	int rc = 0;
	char messages[8];
	if (count > 8) {
		count = 8;
	}
	if (copy_from_user(messages, buff, count)) {
		pr_err("copy_from_user fail !!\n");
		return -EFAULT;
	}

	sscanf(messages, "%d",&set_val);


	if(set_val<=0)
	{
		real_set_brightness = 0;
	}
	else if(set_val > 100)
	{
		real_set_brightness = 100;
	}
	else
	{
		real_set_brightness = set_val;
	}

	//turn on and switch to third-party app, turn on, turn off, then switch back(from bar), val is same 99, etc, should not block this case.
	if(last_brightness_value == real_set_brightness && brightness_state != MSM_CAMERA_LED_RELEASE)
	{
		pr_info("czw, asus flash, real set val %d equal to last value, do nothing\n",real_set_brightness);
		return count;
	}

	mutex_lock(&brightness_lock);

	pr_info("#### czw, asus flash, real set val %d ####\n",real_set_brightness);

	last_brightness_value = real_set_brightness;

	if(brightness_state == MSM_CAMERA_LED_RELEASE)
	{
		rc = msm_flash_lm3642_led_init(&fctrl);
		if(rc < 0)
		{
			pr_err("asus flash, init failed!\n");
			msm_flash_lm3642_led_release(&fctrl);
			mutex_unlock(&brightness_lock);
			return rc;
		}
	}
	else
	{
		//pr_info("czw, asus flash, flash already init, Not do init\n");
	}


	if(real_set_brightness == 0)
	{
		if(fctrl.led_state == MSM_CAMERA_LED_INIT)
		{
			msm_flash_lm3642_led_off(&fctrl);//Camera is Open, just turn off LED, not power down IC
		}
		else
		{
			msm_flash_lm3642_led_release(&fctrl);
		}
	}
	else
	{

		msm_flash_lm3642_torch_current_setting(MAX_FLASHLIGHT_CURRENT*real_set_brightness/100,0);
		if(brightness_state == MSM_CAMERA_LED_INIT || brightness_state == MSM_CAMERA_LED_OFF)//first turn on
		{
			msm_flash_lm3642_led_low_led1(&fctrl);
		}
		else//change brightness without set 0 first
		{
			msm_flash_lm3642_led_off(&fctrl);
			msm_flash_lm3642_led_low_led1(&fctrl);
			pr_info("czw, asus flash, brightness state is %d,flash already on, turn off and turn on again to see new current",brightness_state);
		}
	}

	mutex_unlock(&brightness_lock);
	return count;
}

static ssize_t asus_flash_start_time_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{
	int n;
	char messages[32];// for 64bit unsigned integer

	//pr_info("asus_flash_start_time_show POS = %lld\n",*ppos);

	if(*ppos == 0)
	{
		n = snprintf(messages, sizeof(messages),"%lu\n%c", flash_start_time,'\0');

		if(copy_to_user(buffer,messages,n))
		{
			pr_err("copy_to_user fail !!\n");
			return -EFAULT;
		}
		else
		{
			*ppos += n;
			//pr_info("asus_flash_start_time_show RETURN %d\n",n);
			flash_start_time = 0;
			return n;
		}
	}
	else
	{
		//pr_info("asus_flash_start_time_show RETURN 0\n");
		return 0;
	}

}

static const struct file_operations asus_flash_proc_fops = {
       .read = asus_flash_show,
       .write = asus_flash_store,
};


static const struct file_operations asus_led_proc_fops = {
       .read = asus_led_show,
       .write = asus_led_store,
};

static const struct file_operations asus_brightness_proc_fops = {
       .read = asus_brightness_show,
       .write = asus_brightness_store,
};

static const struct file_operations asus_flash_start_time_proc_fops = {
       .read = asus_flash_start_time_show,
       .write = NULL,
};

//ASUS_BSP Sam --- "Porting Flash"
static int msm_flash_lm3642_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	/*For ASUS FLASH id+++*/
	struct proc_dir_entry* proc_entry_flash;
	struct proc_dir_entry* proc_entry_led;
	struct proc_dir_entry* proc_entry_brightness;
	struct proc_dir_entry* proc_entry_flash_time;
	void* dummy = NULL;
	uint16_t chipid = 0;
	/*For ASUS FLASH id---*/

	int rc = 0 ;
	int retry=0;
	pr_info("[S-DEBUG]%s:%d called\n", __func__, __LINE__);

	ATD_status = 0;
	create_status_proc_file();
	
	if (!id) {
		pr_err("[S-DEBUG]msm_flash_lm3642_i2c_probe: id is NULL");
		id = lm3642_i2c_id;
	}
	rc = msm_flash_i2c_probe(client, id);

	if (rc < 0) {
		pr_err("%s: msm_flash_i2c_probe failed\n", __func__);
		return rc;
	}

	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("[S-DEBUG]%s: request gpio failed\n", __func__);
		return rc;
	}

	pr_err("%s: msm_flash_i2c_probe torch:%d, flash:%d.\n", __func__,
		power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN],
		power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW]);

	/*For ASUS FLASH+++*/
	proc_entry_flash = proc_create_data("driver/asus_flash", 0666, NULL, &asus_flash_proc_fops, dummy);
	proc_set_user(proc_entry_flash, 1000, 1000);

	proc_entry_led = proc_create_data("driver/asus_flash2", 0666, NULL, &asus_led_proc_fops, dummy);
	proc_set_user(proc_entry_led, 1000, 1000);

	proc_entry_brightness = proc_create_data("driver/asus_flash_brightness", 0666, NULL, &asus_brightness_proc_fops, dummy);
	proc_set_user(proc_entry_brightness, 1000, 1000);

	proc_entry_flash_time = proc_create_data("driver/zenflash", 0666, NULL, &asus_flash_start_time_proc_fops, dummy);
	proc_set_user(proc_entry_flash_time, 1000, 1000);
	/*For ASUS FLASH---*/

	if (fctrl.pinctrl_info.use_pinctrl == true) {
		pr_err("[S-DEBUG]%s:%d PC:: flash pins setting to active state",
				__func__, __LINE__);
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_active);
		if (rc)
			pr_err("[S-DEBUG]%s:%d cannot set pin to active state",
					__func__, __LINE__);
	}


/*
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_CUSTOM1],
		GPIO_OUT_HIGH);
*/

	for(retry=0;retry<3;retry++)
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);

		msleep(1);

		pr_err("[S-DEBUG]%s:%d i2c_read register:0x%x",
						__func__, __LINE__, fctrl.flashdata->slave_info->sensor_id_reg_addr);

		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_read(
			fctrl.flash_i2c_client, fctrl.flashdata->slave_info->sensor_id_reg_addr ,
			&chipid, MSM_CAMERA_I2C_BYTE_DATA);

		pr_err("%s: read id: %x expected id %x:\n", __func__, chipid,
			fctrl.flashdata->slave_info->sensor_id);

		if(rc<0)
		{
			pr_err("%s: flash: read id failed, retry, count %d\n", __func__,retry);
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_RESET],
				GPIO_OUT_LOW);
			msleep(20);
		}
		else
		{
			break;
		}
	}

	if(rc<0)
	{
		pr_err("%s: flash: probe failed!",__func__);
		return rc;
	}

	brightness_state = MSM_CAMERA_LED_RELEASE;
	mutex_init(&flash_lock);
	mutex_init(&brightness_lock);
#ifdef GPIO_CONTROL
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_LOW);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
#endif

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_LOW);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
	//if (!rc)
		//msm_lm3642_torch_create_classdev(&(client->dev),NULL);
#ifdef CONFIG_I2C_STRESS_TEST
	pr_err("Camera LED flash add test case+\n");
	i2c_add_test_case(client, "camera_led_flash",ARRAY_AND_SIZE(LEDFlashTestCaseInfo));
	pr_err("Camera LED flash add test case-\n");
#endif
	rc = 0;
	return rc;
}

static int msm_flash_lm3642_i2c_remove(struct i2c_client *client)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	pr_info("[S-DEBUG]%s:%d called\n", __func__, __LINE__);
	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("[S-DEBUG]%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl.pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_suspend);
		if (rc)
			pr_err("[S-DEBUG]%s:%d cannot set pin to suspend state\n",
				__func__, __LINE__);
	}
	return rc;
}


static struct i2c_driver lm3642_i2c_driver = {
	.id_table = lm3642_i2c_id,
	.probe  = msm_flash_lm3642_i2c_probe,
	.remove = msm_flash_lm3642_i2c_remove,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3642_i2c_trigger_dt_match,
	},
};

/*
static int msm_flash_lm3642_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	pr_info("[S-DEBUG]%s:%d called\n", __func__, __LINE__);
	match = of_match_device(lm3642_i2c_trigger_dt_match, &pdev->dev);
	if (!match)
		return -EFAULT;
	return msm_flash_probe(pdev, match->data);
}

static struct platform_driver lm3642_platform_driver = {
	.probe = msm_flash_lm3642_platform_probe,
	.driver = {
		.name = "qcom,led-flash",
		.owner = THIS_MODULE,
		.of_match_table = lm3642_i2c_trigger_dt_match,
	},
};
*/

static int __init msm_flash_lm3642_init(void)
{
	int32_t rc = 0;
	pr_info("[S-DEBUG]%s:%d called\n", __func__, __LINE__);
	//rc = platform_driver_register(&lm3642_platform_driver);
	pr_info("[S-DEBUG]%s:%d rc %d\n", __func__, __LINE__, rc);
	//if (!rc)
		//return rc;
	return i2c_add_driver(&lm3642_i2c_driver);
}

static void __exit msm_flash_lm3642_exit(void)
{
	pr_info("[S-DEBUG]%s:%d called\n", __func__, __LINE__);
	//if (fctrl.pdev)
		//platform_driver_unregister(&lm3642_platform_driver);
	//else
		i2c_del_driver(&lm3642_i2c_driver);
	return;
}


static struct msm_camera_i2c_client lm3642_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting lm3642_init_setting = {
	.reg_setting = lm3642_init_array,
	.size = ARRAY_SIZE(lm3642_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_off_setting = {
	.reg_setting = lm3642_off_array,
	.size = ARRAY_SIZE(lm3642_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_release_setting = {
	.reg_setting = lm3642_release_array,
	.size = ARRAY_SIZE(lm3642_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_low_setting = {
	.reg_setting = lm3642_low_array,
	.size = ARRAY_SIZE(lm3642_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_low_setting_led1 = {
	.reg_setting = lm3642_low_array_led1,
	.size = ARRAY_SIZE(lm3642_low_array_led1),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_low_setting_led2 = {
	.reg_setting = lm3642_low_array_led2,
	.size = ARRAY_SIZE(lm3642_low_array_led2),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_high_setting = {
	.reg_setting = lm3642_high_array,
	.size = ARRAY_SIZE(lm3642_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_high_setting_led1 = {
	.reg_setting = lm3642_high_array_led1,
	.size = ARRAY_SIZE(lm3642_high_array_led1),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};
static struct msm_camera_i2c_reg_setting lm3642_high_setting_led2 = {
	.reg_setting = lm3642_high_array_led2,
	.size = ARRAY_SIZE(lm3642_high_array_led2),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};


static struct msm_led_flash_reg_t lm3642_regs = {
	.init_setting = &lm3642_init_setting,
	.off_setting = &lm3642_off_setting,
	.low_setting = &lm3642_low_setting,
	.low_first_setting = &lm3642_low_setting_led1,
	.low_second_setting = &lm3642_low_setting_led2,
	.high_setting = &lm3642_high_setting,
	.high_first_setting = &lm3642_high_setting_led1,
	.high_second_setting = &lm3642_high_setting_led2,
	.release_setting = &lm3642_release_setting,
};

static struct msm_flash_fn_t lm3642_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_lm3642_led_init,
	.flash_led_release = msm_flash_lm3642_led_release,
	.flash_led_off = msm_flash_lm3642_led_off,
	.flash_led_low = msm_flash_lm3642_led_low,
	.flash_led_low_first = msm_flash_lm3642_led_low_led1,
	.flash_led_low_second = msm_flash_lm3642_led_low_led2,
	.flash_led_high = msm_flash_lm3642_led_high,
	.flash_led_high_first = msm_flash_lm3642_led_high_led1,
	.flash_led_high_second = msm_flash_lm3642_led_high_led2
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &lm3642_i2c_client,
	.reg_setting = &lm3642_regs,
	.func_tbl = &lm3642_func_tbl,
};

/*subsys_initcall(msm_flash_i2c_add_driver);*/
module_init(msm_flash_lm3642_init);
module_exit(msm_flash_lm3642_exit);
MODULE_DESCRIPTION("lm3642 FLASH");
MODULE_LICENSE("GPL v2");
