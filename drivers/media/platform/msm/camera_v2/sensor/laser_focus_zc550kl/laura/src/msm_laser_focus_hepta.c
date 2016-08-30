/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include "msm_laser_focus.h"
#include "show_log.h"
#include "laura_debug.h"
#include "laura_interface.h"
#include "laura_factory_func.h"
#include "laura_shipping_func.h"
#include "laser_focus_hepta.h"
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#ifdef CONFIG_I2C_STRESS_TEST
#include <linux/i2c_testcase.h>
#endif
#define DO_CAL true
#define NO_CAL false
#define DO_MEASURE true
#define NO_MEASURE false

static int DMax = 400;
int ErrCode = 0;

struct msm_laser_focus_ctrl_t *laura_t = NULL;
#if I2C_DEVICE
static struct i2c_driver hepta_i2c_driver;
#endif
static bool camera_on_flag = false;

static bool calibration_flag = true;

static int laser_focus_enforce_ctrl = 0;

static bool load_calibration_data = false;

static int ATD_status;

static int client=0;

extern int Laser_Product;
extern int FirmWare;

//temply use
bool factory = true;
struct msm_laser_focus_ctrl_t *get_laura_ctrl(void){
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return laura_t;
}

bool OLI_device_invalid(void){
	return (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
			laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI);
}	

//extern bool device_invalid(void);


int Laser_Disable(enum msm_laser_focus_atd_device_trun_on_type val){

		int rc=0;
	
		mutex_ctrl(laura_t, MUTEX_LOCK);
		if(camera_on_flag){
              	LOG_Handler(LOG_DBG, "%s: Camera is running, do nothing!!\n ", __func__);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
              	return rc;
              }
		rc = dev_deinit(laura_t);
		power_down(laura_t);
		laura_t->device_state = val;
		load_calibration_data=false;
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		LOG_Handler(LOG_CDBG, "%s Deinit Device (%d)\n", __func__, laura_t->device_state);
		return rc;
}

int Laser_Enable(enum msm_laser_focus_atd_device_trun_on_type val){

		int rc=0;
		mutex_ctrl(laura_t, MUTEX_LOCK);
		if(camera_on_flag){
			LOG_Handler(LOG_DBG, "%s: Camera is running, do nothing!!\n ", __func__);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			return rc;
        }

		if (laura_t->device_state != MSM_LASER_FOCUS_DEVICE_OFF){
			rc = dev_deinit(laura_t);
			laura_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
		}

		rc = !power_up(laura_t) && dev_init(laura_t);
		if(rc)
		{
			LOG_Handler(LOG_ERR, "%s: power up or init failed!\n ", __func__);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			return rc;
		}
		if(val == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION)
			rc = Laura_device_power_up_init_interface(laura_t, DO_CAL, &calibration_flag, NO_MEASURE);
		else
			rc = Laura_device_power_up_init_interface(laura_t, NO_CAL, &calibration_flag, NO_MEASURE);			

		laura_t->device_state = val;
		load_calibration_data = (val == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION?true:false);

		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		LOG_Handler(LOG_CDBG, "%s Init Device (%d)\n", __func__, laura_t->device_state);
		return rc;
}

int Laser_Enable_by_Camera(enum msm_laser_focus_atd_device_trun_on_type val){

	int rc=0;
	mutex_ctrl(laura_t, MUTEX_LOCK);
	laura_t->device_state = val;
	rc = !power_up(laura_t) && dev_init(laura_t);
	if(rc)
	{
		LOG_Handler(LOG_ERR, "%s: power up or init failed!\n ", __func__);
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return rc;
	}
	rc = Laura_device_power_up_init_interface(laura_t, DO_CAL, &calibration_flag, NO_MEASURE);
	if (rc < 0)
	{
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return rc;
	}
	//?
	laura_t->device_state = val;
	load_calibration_data = true;
	camera_on_flag = true;
	mutex_ctrl(laura_t, MUTEX_UNLOCK);
	LOG_Handler(LOG_CDBG, "%s Init Device (%d)\n", __func__, laura_t->device_state);
	return rc;
}

int Laser_Disable_by_Camera(enum msm_laser_focus_atd_device_trun_on_type val){

	int rc=0;
	mutex_ctrl(laura_t, MUTEX_LOCK);
	rc = dev_deinit(laura_t);
	rc = power_down(laura_t);

	laura_t->device_state = val;
	load_calibration_data = false;
	camera_on_flag = false;
	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_CDBG, "%s Deinit Device (%d)\n", __func__, laura_t->device_state);
	return rc;
}

static ssize_t ATD_Laura_device_enable_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, rc = 0;
	char messages[8]="";

	LOG_Handler(LOG_DBG, "%s: Enter\n", __func__);
	
	len =(len > 8 ?8:len);
	if (copy_from_user(messages, buff, len)) {
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	if (laura_t->device_state == val)	{
		LOG_Handler(LOG_ERR, "%s Setting same command (%d) !!\n", __func__, val);
		return -EINVAL;
	}
	
	switch (val) {
		case MSM_LASER_FOCUS_DEVICE_OFF:
			rc = Laser_Disable(val);
			if (rc < 0)
				goto DEVICE_TURN_ON_ERROR;
			break;
			
		case MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION:
		case MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION:	
			rc = Laser_Enable(val);
			if (rc < 0)
				goto DEVICE_TURN_ON_ERROR;				
			break;
			
		case MSM_LASER_FOCUS_DEVICE_INIT_CCI:		
			rc = Laser_Enable_by_Camera(val);
			if (rc < 0)
				goto DEVICE_TURN_ON_ERROR;			
			break;
			
		case MSM_LASER_FOCUS_DEVICE_DEINIT_CCI:
			rc = Laser_Disable_by_Camera(val);
			if (rc < 0)
				goto DEVICE_TURN_ON_ERROR;			
			break;
			
		default:
			LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
			break;
	}

	LOG_Handler(LOG_DBG, "%s: command (%d) done\n",__func__,val);
	return len;
	
DEVICE_TURN_ON_ERROR:

	rc = dev_deinit(laura_t);
	if (rc < 0) 
		LOG_Handler(LOG_ERR, "%s Laura_deinit failed %d\n", __func__, __LINE__);

	laura_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
	
	LOG_Handler(LOG_DBG, "%s: Exit due to device trun on fail !!\n", __func__);
	return -EIO;
}

static int ATD_Laura_device_enable_read(struct seq_file *buf, void *v){
	seq_printf(buf, "%d\n", laura_t->device_state);
	return 0;
}

static int ATD_Laura_device_enable_open(struct inode *inode, struct  file *file){
	return single_open(file, ATD_Laura_device_enable_read, NULL);
}

static const struct file_operations ATD_laser_focus_device_enable_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_device_enable_open,
	.write = ATD_Laura_device_enable_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Laura_device_get_range_read(struct seq_file *buf, void *v)
{
	int Range = 0;
#if 0	
	struct timeval start, now;
#endif
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	mutex_ctrl(laura_t, MUTEX_LOCK);
#if 0
	start = get_current_time();
#endif
	if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		seq_printf(buf, "%d\n", 0);
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return -EBUSY;
	}

	if(laser_focus_enforce_ctrl != 0){
		seq_printf(buf, "%d\n", laser_focus_enforce_ctrl);
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return 0;
	}

	Range = Laura_device_read_range_interface(laura_t, load_calibration_data, &calibration_flag);

	if (Range >= OUT_OF_RANGE) {
		LOG_Handler(LOG_ERR, "%s: Read_range(%d) failed\n", __func__, Range);
		Range = OUT_OF_RANGE;
	}
	
	LOG_Handler(LOG_DBG, "%s : Get range (%d)  Device (%d)\n", __func__, Range , laura_t->device_state);

	seq_printf(buf, "%d\n", Range);

	mutex_ctrl(laura_t, MUTEX_UNLOCK);
#if 0
	now = get_current_time();
	LOG_Handler(LOG_DBG, "%d ms\n", (int) ((((now.tv_sec*1000000)+now.tv_usec)-((start.tv_sec*1000000)+start.tv_usec))));
#endif
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return 0;
}
 
static int ATD_Laura_device_get_range_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, ATD_Laura_device_get_range_read, NULL);
}

static const struct file_operations ATD_laser_focus_device_get_range_fos = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_device_get_range_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Laura_device_get_range_read_more_info(struct seq_file *buf, void *v)
{
	int RawRange = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	mutex_ctrl(laura_t, MUTEX_LOCK);

	if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		seq_printf(buf, "%d\n", 0);
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return -EBUSY;
	}

	if(laser_focus_enforce_ctrl != 0){
		seq_printf(buf, "%d#%d#%d\n", laser_focus_enforce_ctrl, DMax, ErrCode);
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return 0;
	}

	RawRange = (int) Laura_device_read_range_interface(laura_t, load_calibration_data, &calibration_flag);

	if (RawRange >= OUT_OF_RANGE) {
		LOG_Handler(LOG_ERR, "%s: Read_range(%d) failed\n", __func__, RawRange);
		RawRange = OUT_OF_RANGE;
	}
	
	LOG_Handler(LOG_DBG, "%s : Get range (%d)  Device (%d)\n", __func__, RawRange , laura_t->device_state);

	seq_printf(buf, "%d#%d#%d\n", RawRange, DMax, ErrCode);

	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return 0;
}
 
static int ATD_Laura_device_get_range_more_info_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, ATD_Laura_device_get_range_read_more_info, NULL);
}

static const struct file_operations ATD_laser_focus_device_get_range_more_info_fos = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_device_get_range_more_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t ATD_Laura_device_calibration_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, ret = 0;
	char messages[8]="";

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		return -EBUSY;
	}

	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buff, len)) {
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);

	LOG_Handler(LOG_DBG, "%s command : %d\n", __func__, val);
	
	switch (val) {
		case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			ret = Laura_device_calibration_interface(laura_t, load_calibration_data, &calibration_flag, val);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			if (ret < 0)
				return ret;
			break;
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			ret = Laura_device_calibration_interface(laura_t, load_calibration_data, &calibration_flag, val);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			if (ret < 0)
				return ret;
			break;
		case MSM_LASER_FOCUS_APPLY_INFINITY_CALIBRATION:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			ret = Laura_device_calibration_interface(laura_t, load_calibration_data, &calibration_flag, val);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			if (ret < 0)
				return ret;
			break;
		default:
			LOG_Handler(LOG_ERR, "%s command fail(%d) !!\n", __func__, val);
			return -EINVAL;
			break;
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return len;
}

static int ATD_Olivia_dummy_read(struct seq_file *buf, void *v)
{
	return 0;
}

static int ATD_Olivia_dummy_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, ATD_Olivia_dummy_read, NULL);
}

static ssize_t ATD_Olivia_device_calibration_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, ret = 0;
	char messages[8]="";
	LOG_Handler(LOG_CDBG, "%s: Enter\n", __func__);

	if (OLI_device_invalid()){
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		return -EBUSY;
	}
	len = (len>8?8:len);
	if (copy_from_user(messages, buff, len)){
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	LOG_Handler(LOG_DBG, "%s command : %d\n", __func__, val);	
	switch (val) {
		case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
		case MSM_LASER_FOCUS_APPLY_INFINITY_CALIBRATION:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			ret = Olivia_device_calibration_interface(laura_t, load_calibration_data, &calibration_flag, val);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			if (ret < 0)
				return ret;
			break;
		default:
			LOG_Handler(LOG_ERR, "%s command fail(%d) !!\n", __func__, val);
			return -EINVAL;
			break;
	}

	LOG_Handler(LOG_CDBG, "%s: Exit\n", __func__);	
	return len;
}


static const struct file_operations ATD_olivia_calibration_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Olivia_dummy_open,
	.write = ATD_Olivia_device_calibration_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations ATD_laser_focus_device_calibration_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_device_get_range_open,
	.write = ATD_Laura_device_calibration_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Laura_get_calibration_input_data_proc_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, Laura_get_calibration_input_data_interface, NULL);
}

static const struct file_operations ATD_Laura_get_calibration_input_data_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_get_calibration_input_data_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Olivia_get_calibration_input_data_proc_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, Olivia_get_calibration_input_data_interface, NULL);
}

static const struct file_operations ATD_Olivia_get_calibration_input_data_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Olivia_get_calibration_input_data_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Laura_I2C_status_check_proc_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	mutex_ctrl(laura_t, MUTEX_LOCK);
	ATD_status = dev_I2C_status_check(laura_t, MSM_CAMERA_I2C_WORD_DATA);
	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	seq_printf(buf, "%d\n", ATD_status);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
}

static int ATD_Laura_I2C_status_check_proc_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, ATD_Laura_I2C_status_check_proc_read, NULL);
}

static const struct file_operations ATD_I2C_status_check_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_I2C_status_check_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Laura_I2C_status_check_proc_read_for_camera(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	seq_printf(buf, "%d\n", ATD_status);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
}

static int ATD_Laura_I2C_status_check_proc_open_for_camera(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, ATD_Laura_I2C_status_check_proc_read_for_camera, NULL);
}

static const struct file_operations ATD_I2C_status_check_for_camera_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_I2C_status_check_proc_open_for_camera,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_register_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, dump_laura_register_read, NULL);
}

static const struct file_operations dump_laser_focus_register_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_register_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	//mutex_ctrl(laura_t, MUTEX_LOCK);
	laura_debug_dump(buf, v);
	//mutex_ctrl(laura_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}

static int dump_Laura_laser_focus_debug_register_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, dump_Laura_debug_register_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_register_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int Laura_laser_focus_enforce_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
}

static int Laura_laser_focus_enforce_open(struct inode *inode, struct file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, Laura_laser_focus_enforce_read, NULL);
}

static ssize_t Laura_laser_focus_enforce_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t rc;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	mutex_ctrl(laura_t, MUTEX_LOCK);
	rc = Laser_Focus_enforce(laura_t, buff, len, &laser_focus_enforce_ctrl);
	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;
}

static const struct file_operations laser_focus_enforce_fops = {
	.owner = THIS_MODULE,
	.open = Laura_laser_focus_enforce_open,
	.write = Laura_laser_focus_enforce_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int Laura_laser_focus_log_contorl_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
}

static int Laura_laser_focus_log_contorl_open(struct inode *inode, struct file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, Laura_laser_focus_log_contorl_read, NULL);
}

static ssize_t Laura_laser_focus_log_contorl_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t rc;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	mutex_ctrl(laura_t, MUTEX_LOCK);
	rc = Laser_Focus_log_contorl(buff, len);
	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;
}

static const struct file_operations laser_focus_log_contorl_fops = {
	.owner = THIS_MODULE,
	.open = Laura_laser_focus_log_contorl_open,
	.write = Laura_laser_focus_log_contorl_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*++++++++++CE Debug++++++++++*/
static int dump_Laura_debug_value1_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	/*
	if(Laser_Product == PRODUCT_OLIVIA)
		Olivia_Read_Calibration_Value_From_File(NULL, cal_data);		
	else
		Laura_Read_Calibration_Value_From_File(NULL, cal_data);
	*/
	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);		

	seq_printf(buf,"%d\n",cal_data[3]);	
	
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}

static int dump_Laura_laser_focus_debug_value1_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, dump_Laura_debug_value1_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value1_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value1_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_value2_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	/*
	if(Laser_Product == PRODUCT_OLIVIA)
		Olivia_Read_Calibration_Value_From_File(NULL, cal_data);		
	else
		Laura_Read_Calibration_Value_From_File(NULL, cal_data);
	*/
	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);	

        seq_printf(buf,"%d\n",cal_data[4]);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}

static int dump_Laura_laser_focus_debug_value2_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, dump_Laura_debug_value2_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value2_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value2_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_value3_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	/*
	if(Laser_Product == PRODUCT_OLIVIA)
		Olivia_Read_Calibration_Value_From_File(NULL, cal_data);		
	else
		Laura_Read_Calibration_Value_From_File(NULL, cal_data);
	*/
	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);	

        seq_printf(buf,"%d\n",cal_data[6]);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}

static int dump_Laura_laser_focus_debug_value3_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, dump_Laura_debug_value3_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value3_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value3_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_value4_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	/*
	if(Laser_Product == PRODUCT_OLIVIA)
		Olivia_Read_Calibration_Value_From_File(NULL, cal_data);		
	else
		Laura_Read_Calibration_Value_From_File(NULL, cal_data);
	*/
	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);	

        seq_printf(buf,"%d\n",cal_data[7]);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}

static int dump_Laura_laser_focus_debug_value4_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, dump_Laura_debug_value4_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value4_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value4_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_value5_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/*
	if(Laser_Product == PRODUCT_OLIVIA)
		Olivia_Read_Calibration_Value_From_File(NULL, cal_data);		
	else
		Laura_Read_Calibration_Value_From_File(NULL, cal_data);
	*/
	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);	

        seq_printf(buf,"%d\n",cal_data[8]);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}

static int dump_Laura_laser_focus_debug_value5_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, dump_Laura_debug_value5_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value5_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value5_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int dump_Laura_debug_value6_read(struct seq_file *buf, void *v)
{
	int16_t rc = 0, RawConfidence = 0, confidence_level = 0;;
	
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		return -EBUSY;
	}

	/* Read result confidence level */
	rc = CCI_I2C_RdWord(laura_t, 0x0A, &RawConfidence);
	if (rc < 0){
		return rc;
	}
	//RawConfidence = swap_data(RawConfidence);
	confidence_level = (RawConfidence&0x7fff)>>4;
	
	seq_printf(buf,"%d\n",confidence_level);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}

static int dump_Laura_laser_focus_debug_value6_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, dump_Laura_debug_value6_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value6_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value6_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

//ATD will tell how to impliment it
static int dump_Laura_value_check_read(struct seq_file *buf, void *v)
{
        LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	seq_printf(buf,"PASS\n");

        LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

        return 0;
}

static int dump_Laura_laser_focus_value_check_open(struct inode *inode, struct  file *file)
{
        LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
        LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
        return single_open(file, dump_Laura_value_check_read, NULL);
}

static const struct file_operations dump_laser_focus_value_check_fops = {
        .owner = THIS_MODULE,
        .open = dump_Laura_laser_focus_value_check_open,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};
/*----------CE Debug----------*/


static int Laura_laser_focus_set_K_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_DBG, "%s: Enter and Exit, calibration: %d\n", __func__, calibration_flag);
	seq_printf(buf,"%d",calibration_flag);
	return 0;
}
static int Laura_laser_focus_set_K_open(struct inode *inode, struct file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter and Exit\n", __func__);
	return single_open(file, Laura_laser_focus_set_K_read, NULL);
}

static ssize_t Laura_laser_focus_set_K_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t rc;
	int val;
	char messages[8]="";
	LOG_Handler(LOG_CDBG, "%s: Enter\n", __func__);

	len = (len>8?8:len);
	if (copy_from_user(messages, buff, len)){
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	
	switch(val){
		case 0:
			calibration_flag = false;
			break;
		case 1:
			calibration_flag = true;
			break;
		default:
			LOG_Handler(LOG_DBG, "command '%d' is not valid\n", val);
	}

	LOG_Handler(LOG_CDBG, "%s: Exit, calibration: %d\n", __func__, calibration_flag);
	return rc;
}

static const struct file_operations laser_focus_set_K_fops = {
	.owner = THIS_MODULE,
	.open = Laura_laser_focus_set_K_open,
	.write = Laura_laser_focus_set_K_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int Laura_laser_focus_product_family_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_DBG, "%s: Enter and Exit, Product Family: %d\n", __func__, Laser_Product);
	seq_printf(buf,"%d",Laser_Product);
	return 0;
}
static int Laura_laser_focus_product_family_open(struct inode *inode, struct file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter and Exit\n", __func__);
	return single_open(file, Laura_laser_focus_product_family_read, NULL);
}

static const struct file_operations laser_focus_product_family = {
	.owner = THIS_MODULE,
	.open = Laura_laser_focus_product_family_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


#define MODULE_NAME "LaserSensor"
#define ASUS_LASER_NAME_SIZE	32
#define ASUS_LASER_DATA_SIZE	4
#define OLIVIA_IOC_MAGIC                      ('W')
#define ASUS_LASER_SENSOR_MEASURE     _IOR(OLIVIA_IOC_MAGIC  , 0, unsigned int[ASUS_LASER_DATA_SIZE])
#define ASUS_LASER_SENSOR_GET_NAME	_IOR(OLIVIA_IOC_MAGIC  , 4, char[ASUS_LASER_NAME_SIZE])

int Olivia_get_measure(int* distance){
	int RawRange = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	mutex_ctrl(laura_t, MUTEX_LOCK);

	if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return -EBUSY;
	}

	if(laser_focus_enforce_ctrl != 0){
		*distance = laser_focus_enforce_ctrl;
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return 0;
	}

	RawRange = (int) Laura_device_read_range_interface(laura_t, load_calibration_data, &calibration_flag);

	if (RawRange < 0) {
		LOG_Handler(LOG_ERR, "%s: Read_range(%d) failed\n", __func__, RawRange);
		RawRange = 0;
	}
	
	LOG_Handler(LOG_DBG, "%s : Get range (%d)  Device (%d)\n", __func__, RawRange , laura_t->device_state);


	mutex_ctrl(laura_t, MUTEX_UNLOCK);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);	
	return 0;


}

static int Olivia_misc_open(struct inode *inode, struct file *file){

	client++;
	LOG_Handler(LOG_DBG,"%s: client enter(%d)\n", __func__, client);

	power_up(laura_t);

	//camera may open once at 70cm K
	if(factory)
		Laser_Enable(MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION);
	else
		Laser_Enable(MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION);
	
	return 0;
}

static int Olivia_misc_release(struct inode *inode, struct file *file)
{
	client--;
	LOG_Handler(LOG_DBG,"%s: client leave(%d)\n", __func__, client);


	if(client <=0){
		Laser_Disable(MSM_LASER_FOCUS_DEVICE_OFF);
		power_down(laura_t);
	}

	return 0;
}

static long Olivia_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

 	unsigned int dist[4] = {0,0,0,0};
	char name[ASUS_LASER_NAME_SIZE];
	int distance;
	int ret = 0;
	snprintf(name, ASUS_LASER_NAME_SIZE, MODULE_NAME);
	
	switch (cmd) {
		
		case ASUS_LASER_SENSOR_MEASURE:
			Olivia_get_measure(&distance);
			//__put_user(dist, (int* __user*)arg);
			dist[0] = distance;
			dist[1] = ErrCode;
			dist[2] = DMax;
			dist[3] = calibration_flag;
			ret = copy_to_user((int __user*)arg, dist, sizeof(dist));
			LOG_Handler(LOG_DBG, "%s: range data [%d,%d,%d,%d]\n"
								,__func__,distance,ErrCode,DMax,calibration_flag);
			break;
		case ASUS_LASER_SENSOR_GET_NAME:
			//__put_user(MODULE_NAME, (int __user*)arg);
			ret = copy_to_user((int __user*)arg, &name, sizeof(name));			
			break;
		default:
			LOG_Handler(LOG_ERR,"%s: ioctrl command is not valid\n", __func__);
	}
	return 0;

}
static struct file_operations Olivia_fops = {
  .owner = THIS_MODULE,
  .open = Olivia_misc_open,
  .release = Olivia_misc_release,
  .unlocked_ioctl = Olivia_misc_ioctl,
  .compat_ioctl = Olivia_misc_ioctl
};

struct miscdevice Olivia_misc = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = MODULE_NAME,
  .fops = &Olivia_fops
};

static int Olivia_misc_register(int Product)
{
	int rtn = 0;

	if(Product != PRODUCT_OLIVIA){
		LOG_Handler(LOG_CDBG, "LaserFocus is not supported, ProductFamily is not Olivia (%d)",Product);
		return rtn;
	}
	
	rtn = misc_register(&Olivia_misc);
	if (rtn < 0) {
		LOG_Handler(LOG_ERR,"Unable to register misc devices\n");
		misc_deregister(&Olivia_misc);
	}
	return rtn;
}



#define proc(file,mod,fop)	\
	LOG_Handler(LOG_DBG,"proc %s %s\n",file,proc_create(file,mod,NULL,fop)? "success":"fail");

static void Olivia_Create_proc(void)
{
	#if 1
	proc(STATUS_PROC_FILE, 0664, &ATD_I2C_status_check_fops);
	proc(STATUS_PROC_FILE_FOR_CAMERA, 0664, &ATD_I2C_status_check_for_camera_fops);
	proc(DEVICE_TURN_ON_FILE, 0664, &ATD_laser_focus_device_enable_fops);
	proc(DEVICE_GET_VALUE, 0664, &ATD_laser_focus_device_get_range_fos);
	proc(DEVICE_GET_VALUE_MORE_INFO, 0664, &ATD_laser_focus_device_get_range_more_info_fos);

if(Laser_Product == PRODUCT_OLIVIA){
	pr_err("czw, OLIVIA create calibration fops");
	proc(DEVICE_SET_CALIBRATION, 0664, &ATD_olivia_calibration_fops);
	proc(DEVICE_GET_CALIBRATION_INPUT_DATA, 0664, &ATD_Olivia_get_calibration_input_data_fops);
}
else{
	proc(DEVICE_SET_CALIBRATION, 0664, &ATD_laser_focus_device_calibration_fops);
	proc(DEVICE_GET_CALIBRATION_INPUT_DATA, 0664, &ATD_Laura_get_calibration_input_data_fops);
}
	
	proc(DEVICE_DUMP_REGISTER_VALUE, 0664, &dump_laser_focus_register_fops);
	proc(DEVICE_DUMP_DEBUG_VALUE, 0664, &dump_laser_focus_debug_register_fops);
	//proc(DEVICE_ENFORCE_FILE, 0664, &laser_focus_enforce_fops);
	proc(DEVICE_LOG_CTRL_FILE, 0664, &laser_focus_log_contorl_fops);

	//for ce
	proc(DEVICE_DEBUG_VALUE1, 0664, &dump_laser_focus_debug_value1_fops);
	proc(DEVICE_DEBUG_VALUE2, 0664, &dump_laser_focus_debug_value2_fops);
	proc(DEVICE_DEBUG_VALUE3, 0664, &dump_laser_focus_debug_value3_fops);
	proc(DEVICE_DEBUG_VALUE4, 0664, &dump_laser_focus_debug_value4_fops);
	proc(DEVICE_DEBUG_VALUE5, 0664, &dump_laser_focus_debug_value5_fops);	
	proc(DEVICE_DEBUG_VALUE6, 0664, &dump_laser_focus_debug_value6_fops);	
	proc(DEVICE_VALUE_CHECK, 0664, &dump_laser_focus_value_check_fops);
	//for ce

	//for dit
	proc(DEVICE_IOCTL_SET_K, 0664, &laser_focus_set_K_fops);
	proc(DEVICE_IOCTL_PRODUCT_FAMILY, 0444, &laser_focus_product_family);
	//for dit
	#endif
}

#if I2C_DEVICE
static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	//.i2c_write_seq = msm_camera_qup_i2c_write_seq,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
	//.i2c_util = msm_sensor_qup_i2c_util,
	.i2c_poll = msm_camera_qup_i2c_poll,
};
#else
static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll =  msm_camera_cci_i2c_poll,
};
#endif
static const struct v4l2_subdev_internal_ops msm_laser_focus_internal_ops;

static struct v4l2_subdev_core_ops msm_laser_focus_subdev_core_ops = {
	.ioctl = NULL,
};

static struct v4l2_subdev_ops msm_laser_focus_subdev_ops = {
	.core = &msm_laser_focus_subdev_core_ops,
};

static struct msm_camera_i2c_client msm_laser_focus_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

#if I2C_DEVICE
static const struct i2c_device_id hepta_i2c_id[] = {
	{"qcom,tof", (kernel_ulong_t)NULL},
	{ }
};
static const struct of_device_id hepta_i2c_dt_match[] = {
	{.compatible = "qcom,tof", .data = NULL},
	{}
};
MODULE_DEVICE_TABLE(of, hepta_i2c_dt_match);

static int hepta_i2c_remove(struct i2c_client *client)
{
		return 0;
}

#else
static const struct of_device_id msm_laser_focus_dt_match[] = {
	{.compatible = "qcom,tof", .data = NULL},
	{}
};
MODULE_DEVICE_TABLE(of, msm_laser_focus_dt_match);

static int match_Olivia(struct platform_device *pdev){

	const struct of_device_id *match;

	match = of_match_device(msm_laser_focus_dt_match, &pdev->dev);
	if (!match) {
		pr_err("device not match\n");
		return -EFAULT;
	}

	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}
	
	laura_t = kzalloc(sizeof(struct msm_laser_focus_ctrl_t),GFP_KERNEL);
	if (!laura_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	/* Set platform device handle */
	laura_t->pdev = pdev;

	LOG_Handler(LOG_FUN, "%s: done\n", __func__);
	return 0;
}
#endif



static int set_i2c_client(struct platform_device *pdev){

	/* Assign name for sub device */
	snprintf(laura_t->msm_sd.sd.name, sizeof(laura_t->msm_sd.sd.name),
			"%s", laura_t->sensordata->sensor_name);
			
//	snprintf(vl6180x_t->msm_sd.sd.name, sizeof(vl6180x_t->msm_sd.sd.name),
//		"%s", laura_t->i2c_driver->driver.name);
	laura_t->act_v4l2_subdev_ops = &msm_laser_focus_subdev_ops;

#if I2C_DEVICE
	/* Set device type as i2c device */
	laura_t->act_device_type = MSM_CAMERA_I2C_DEVICE;
#else
	/* Set device type as platform device */
	laura_t->act_device_type = MSM_CAMERA_PLATFORM_DEVICE;	
#endif
	laura_t->i2c_client = &msm_laser_focus_i2c_client;
	if (NULL == laura_t->i2c_client) {
		pr_err("%s i2c_client NULL\n",
			__func__);
		return -EFAULT;
	}
	if (!laura_t->i2c_client->i2c_func_tbl)
	{
#if I2C_DEVICE
		laura_t->i2c_client->i2c_func_tbl = &msm_sensor_qup_func_tbl;	
#else
		laura_t->i2c_client->i2c_func_tbl = &msm_sensor_cci_func_tbl;	
#endif
	}

#if !I2C_DEVICE
	laura_t->i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!laura_t->i2c_client->cci_client) {
		kfree(laura_t->vreg_cfg.cam_vreg);
		kfree(laura_t);
		pr_err("failed no memory\n");
		return -ENOMEM;
	}
#endif
	LOG_Handler(LOG_FUN, "%s: done\n", __func__);	
	return 0;	
}

#if !I2C_DEVICE
static void set_cci_client(struct platform_device *pdev){

	struct msm_camera_cci_client *cci_client = NULL;

	cci_client = laura_t->i2c_client->cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = laura_t->cci_master;
	if (laura_t->sensordata->slave_info->sensor_slave_addr)
		cci_client->sid = laura_t->sensordata->slave_info->sensor_slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = I2C_FAST_MODE;
	
	LOG_Handler(LOG_FUN, "%s: done\n", __func__);
}
#endif
static int set_subdev(struct platform_device *pdev){
	int rc = 0;
#if I2C_DEVICE
	v4l2_i2c_subdev_init(&laura_t->msm_sd.sd,laura_t->i2c_client->client,laura_t->act_v4l2_subdev_ops);
#else
	v4l2_subdev_init(&laura_t->msm_sd.sd, laura_t->act_v4l2_subdev_ops);
#endif
	v4l2_set_subdevdata(&laura_t->msm_sd.sd, laura_t);
	
	laura_t->msm_sd.sd.internal_ops = &msm_laser_focus_internal_ops;
	laura_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(laura_t->msm_sd.sd.name,
		ARRAY_SIZE(laura_t->msm_sd.sd.name), "msm_laser_focus");
	
	media_entity_init(&laura_t->msm_sd.sd.entity, 0, NULL, 0);
	laura_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	laura_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_LASER_FOCUS;
	laura_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	rc=msm_sd_register(&laura_t->msm_sd);

	return rc;
	LOG_Handler(LOG_FUN, "%s: done\n", __func__);
}

static void set_laser_config(struct platform_device *pdev){

	/* Init data struct */
	laura_t->laser_focus_state = LASER_FOCUS_POWER_DOWN;
	laura_t->laser_focus_cross_talk_offset_value = 0;
	laura_t->laser_focus_offset_value = 0;
	laura_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
	
	LOG_Handler(LOG_FUN, "%s: done\n", __func__);
}

static int Laura_Init_Chip_Status_On_Boot(struct msm_laser_focus_ctrl_t *dev_t){
	       int rc = 0, chip_status = 0;
	      
		     LOG_Handler(LOG_CDBG, "%s: Enter Init Chip Status\n", __func__);    
		
			      mutex_ctrl(laura_t, MUTEX_LOCK);

			      rc = !power_up(laura_t) && dev_init(laura_t);

			      rc = Laura_device_power_up_init_interface(laura_t, NO_CAL, &calibration_flag, NO_MEASURE);
			      chip_status = Laura_WaitDeviceStandby(dev_t);
			      if (rc < 0 || chip_status < 0){
			      LOG_Handler(LOG_ERR, "%s Device init fail !! (rc,status):(%d,%d)\n", __func__, rc, chip_status);
			     } else {
			    LOG_Handler(LOG_CDBG, "%s Init init success !! (rc,status):(%d,%d)\n", __func__, rc,chip_status);
			     }	   
			    dev_deinit(laura_t);
				power_down(laura_t);
			       mutex_ctrl(laura_t, MUTEX_UNLOCK);
			    
			    LOG_Handler(LOG_CDBG, "%s: Exit Init Chip Status\n", __func__);
			    
			    return rc;
}
#if I2C_DEVICE

#ifdef CONFIG_I2C_STRESS_TEST
#define I2C_TEST_LASERFOCUS_FAIL (-1)

static int TestLaserFocusI2C(struct i2c_client *apClient)
{
	int32_t rc ;

	i2c_log_in_test_case("TestLaserFocusI2C ++\n");
	mutex_ctrl(laura_t, MUTEX_LOCK);

	rc = dev_I2C_status_check(laura_t,MSM_CAMERA_I2C_WORD_DATA);
	if(rc==0)
		goto FAIL;
	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	i2c_log_in_test_case("TestLaserFocusI2C --\n");
	return I2C_TEST_PASS;
FAIL:
	mutex_ctrl(laura_t, MUTEX_UNLOCK);
	i2c_log_in_test_case("TestLaserFocusI2C failed --\n");
	return I2C_TEST_LASERFOCUS_FAIL;
}

static struct i2c_test_case_info LaserFocusTestCaseInfo[] =
{
     __I2C_STRESS_TEST_CASE_ATTR(TestLaserFocusI2C),
};
#endif

//ASUS_BSP Sam add i2c monitor +++
static int ASUS_Laura_I2C_status_check_proc_read(struct seq_file *buf, void *v)
{
	int i2c_status  = 0;
	if(ATD_status == 2)
		i2c_status = dev_I2C_status_check(laura_t,MSM_CAMERA_I2C_WORD_DATA);
	seq_printf(buf, "%d\n", i2c_status);
	return 0;
}

static int ASUS_Laura_I2C_status_check_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ASUS_Laura_I2C_status_check_proc_read, NULL);
}

static const struct file_operations ASUS_I2C_status_check_fops = {
	.owner = THIS_MODULE,
	.open = ASUS_Laura_I2C_status_check_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void Laura_create_proc_file_only_for_i2c(void)
{
	struct proc_dir_entry * i2c_status_file = proc_create("driver/LaserFocus_I2c_Status_Laura", 0664, NULL, &ASUS_I2C_status_check_fops);

	if (i2c_status_file) {
		printk("%s i2c_status_file sucessed!\n", __func__);
	} else {
		printk("%s i2c_status_file failed!\n", __func__);
	}
}

//ASUS_BSP Sam add i2c monitor ---
static int32_t hepta_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	int retry = 0;
	int check_result = 0;
	pr_err("czw, hepta_i2c_probe Enter\n");

	if (client == NULL) {
		LOG_Handler(LOG_ERR, "%s(): client is null\n",__func__);
		rc = -EINVAL;
		goto probe_failure;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		LOG_Handler(LOG_ERR, "%s(): i2c_check_functionality failed\n",__func__);
		goto probe_failure;
	}

	laura_t = kzalloc(sizeof(struct msm_laser_focus_ctrl_t),GFP_KERNEL);
	if (!laura_t) {
		LOG_Handler(LOG_ERR, "%s(): no memory!\n",__func__);
		rc = -ENOMEM;
		goto probe_failure;
	}

	rc = get_dtsi_data(client->dev.of_node, laura_t);
	if (rc < 0)
	{
		LOG_Handler(LOG_ERR, "%s(): get_dtsi_data failed!\n",__func__);
		goto Error_Early;
	}

	rc = set_i2c_client(NULL);
	if (rc < 0)
	{
		LOG_Handler(LOG_ERR, "%s(): set_i2c_client failed\n",__func__);
		goto Error_Early;
	}
	laura_t->i2c_client->client = client;
	laura_t->i2c_driver = &hepta_i2c_driver;
	pr_err("czw, line %d,i2c_client address is 0x%X\n",__LINE__,laura_t->i2c_client->client->addr & 0x7F);
	laura_t->i2c_client->client->addr = 0x98;
	pr_err("czw, line %d,i2c_client address is 0x%X\n",__LINE__,laura_t->i2c_client->client->addr & 0x7F);
	laura_t->sensordata->power_info.dev = &client->dev;
	
	set_laser_config(NULL);

	Laura_create_proc_file_only_for_i2c();//ASUS_BSP Sam add i2c monitor

#if 1
	/* Check I2C status */
	for(retry=0;retry<3;retry++)
	{
		check_result = dev_I2C_status_check(laura_t,MSM_CAMERA_I2C_WORD_DATA);
		if(check_result == 0)
		{
			LOG_Handler(LOG_ERR, "%s(): i2c check status failed, retry, count %d\n",__func__,retry);
			msleep(20);
		}
		else
		{
			break;
		}
	}
	if(check_result == 0)
	{
		rc = -EFAULT;
		goto Error_Early;
	}
#endif	

	/* Initialize sub device */

	/* Init mutex */
    mutex_ctrl(laura_t, MUTEX_ALLOCATE);
	mutex_ctrl(laura_t, MUTEX_INIT);

	rc = Laura_Init_Chip_Status_On_Boot(laura_t);
	if (rc < 0)
	{
		LOG_Handler(LOG_ERR, "%s(): Init_Chip_Status_On_Boot Failed!\n",__func__);
		goto Error_Middle;
	}
	rc = Olivia_misc_register(Laser_Product);
	if (rc < 0)
		goto Error_Middle;

	rc = set_subdev(NULL);//?
	if (rc < 0)
	{
		LOG_Handler(LOG_ERR, "%s(): set_subdev failed, rc = %d\n",__func__,rc);
		goto Error_Late;	
	}
	
	Olivia_Create_proc();
	ATD_status = 2;

#ifdef CONFIG_I2C_STRESS_TEST
	pr_err("Camera LaserFocus add test case+\n");
	i2c_add_test_case(client, "camera_laser_focus",ARRAY_AND_SIZE(LaserFocusTestCaseInfo));
	pr_err("Camera LaserFocus add test case-\n");
#endif
	rc = 0;
	LOG_Handler(LOG_CDBG, "%s: Probe Success\n", __func__);
	return rc;

Error_Late:
	misc_deregister(&Olivia_misc);
Error_Middle:
	mutex_ctrl(laura_t, MUTEX_DESTROY);
Error_Early:
	if(laura_t->sensordata)
	{
		if(laura_t->sensordata->slave_info)
			kfree(laura_t->sensordata->slave_info);
		if(laura_t->sensordata->power_info.gpio_conf)
			kfree(laura_t->sensordata->power_info.gpio_conf);
		kfree(laura_t->sensordata);
	}
	kfree(laura_t);
probe_failure:
	pr_err("%s(): failed, rc=%d\n",__func__,rc);
	
	return rc;
}
#else
static int32_t Olivia_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	//struct device_node *np = pdev->dev.of_node;
	LOG_Handler(LOG_CDBG, "%s: Probe Start\n", __func__);

	rc = match_Olivia(pdev);
	if(rc < 0)
		goto probe_failure;
		
	rc = get_dtsi_data(pdev->dev.of_node, laura_t);
	if (rc < 0) 
		goto probe_failure;

	rc = set_i2c_client(pdev);	
	if (rc < 0)
		goto probe_failure;
	
	set_cci_client(pdev);	
	set_subdev(pdev);
	set_laser_config(pdev);

	/* Check I2C status */
	if(dev_I2C_status_check(laura_t, MSM_CAMERA_I2C_WORD_DATA) == 0)
		goto probe_failure;

	/* Init mutex */
    mutex_ctrl(laura_t, MUTEX_ALLOCATE);
	mutex_ctrl(laura_t, MUTEX_INIT);

	Laura_Init_Chip_Status_On_Boot(laura_t);


	rc = Olivia_misc_register(Laser_Product);
	if (rc < 0)
		goto probe_failure;

	Olivia_Create_proc();
	ATD_status = 2;
	
	LOG_Handler(LOG_CDBG, "%s: Probe Success\n", __func__);
	return 0;
	
probe_failure:
	LOG_Handler(LOG_CDBG, "%s: Probe failed, rc = %d\n", __func__, rc);
	return rc;
}
#endif

#if 0
static int32_t Laura_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;//, i = 0;
	//uint32_t id_info[3];
	const struct of_device_id *match;
	struct msm_camera_cci_client *cci_client = NULL;
	
	//if(g_ASUS_laserID == 1)
                //LOG_Handler(LOG_ERR, "%s: It is VL6180x sensor, do nothing!!\n", __func__);
               // return rc;
        
	printk("Laura_platform_probe\n");
	LOG_Handler(LOG_CDBG, "%s: Probe Start\n", __func__);
	ATD_status = 0;
	
	match = of_match_device(msm_laser_focus_dt_match, &pdev->dev);
	if (!match) {
		pr_err("device not match\n");
		return -EFAULT;
	}

	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}
	laura_t = kzalloc(sizeof(struct msm_laser_focus_ctrl_t),
		GFP_KERNEL);
	if (!laura_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	/* Set platform device handle */
	laura_t->pdev = pdev;

	rc = get_dtsi_data(pdev->dev.of_node, laura_t);
	if (rc < 0) {
		pr_err("%s failed line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}
	/* Assign name for sub device */
	snprintf(laura_t->msm_sd.sd.name, sizeof(laura_t->msm_sd.sd.name),
			"%s", laura_t->sensordata->sensor_name);

	laura_t->act_v4l2_subdev_ops = &msm_laser_focus_subdev_ops;
	//laura_t->laser_focus_mutex = &msm_laser_focus_mutex;
	//laura_t->cam_name = pdev->id;

	/* Set device type as platform device */
	laura_t->act_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	laura_t->i2c_client = &msm_laser_focus_i2c_client;
	if (NULL == laura_t->i2c_client) {
		pr_err("%s i2c_client NULL\n",
			__func__);
		rc = -EFAULT;
		goto probe_failure;
	}
	if (!laura_t->i2c_client->i2c_func_tbl)
		laura_t->i2c_client->i2c_func_tbl = &msm_sensor_cci_func_tbl;

	laura_t->i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!laura_t->i2c_client->cci_client) {
		kfree(laura_t->vreg_cfg.cam_vreg);
		kfree(laura_t);
		pr_err("failed no memory\n");
		return -ENOMEM;
	}
	//laura_t->i2c_client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	cci_client = laura_t->i2c_client->cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = laura_t->cci_master;
	if (laura_t->sensordata->slave_info->sensor_slave_addr)
		cci_client->sid = laura_t->sensordata->slave_info->sensor_slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = I2C_FAST_MODE;
	v4l2_subdev_init(&laura_t->msm_sd.sd,
		laura_t->act_v4l2_subdev_ops);
	v4l2_set_subdevdata(&laura_t->msm_sd.sd, laura_t);
	laura_t->msm_sd.sd.internal_ops = &msm_laser_focus_internal_ops;
	laura_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(laura_t->msm_sd.sd.name,
		ARRAY_SIZE(laura_t->msm_sd.sd.name), "msm_laser_focus");
	media_entity_init(&laura_t->msm_sd.sd.entity, 0, NULL, 0);
	laura_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	//laura_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_LASER_FOCUS;
	laura_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&laura_t->msm_sd);
	laura_t->laser_focus_state = LASER_FOCUS_POWER_DOWN;

	/* Init data struct */
	laura_t->laser_focus_cross_talk_offset_value = 0;
	laura_t->laser_focus_offset_value = 0;
	laura_t->laser_focus_state = MSM_LASER_FOCUS_DEVICE_OFF;

	/* Check I2C status */
	if(dev_I2C_status_check(laura_t, MSM_CAMERA_I2C_WORD_DATA) == 0)
		goto probe_failure;

	/* Init mutex */
       mutex_ctrl(laura_t, MUTEX_ALLOCATE);
	mutex_ctrl(laura_t, MUTEX_INIT);

	ATD_status = 2;
	
	/* Create proc file */
	Olivia_Create_proc();
	LOG_Handler(LOG_CDBG, "%s: Probe Success\n", __func__);
	return 0;
probe_failure:
	LOG_Handler(LOG_CDBG, "%s: Probe failed\n", __func__);

	return rc;
}
#endif

#if I2C_DEVICE
static struct i2c_driver hepta_i2c_driver = {
	.id_table = hepta_i2c_id,
	.probe  = hepta_i2c_probe,
	.remove = hepta_i2c_remove,
	.driver = {
		.name = "qcom,tof",
		.owner = THIS_MODULE,
		.of_match_table = hepta_i2c_dt_match,
	},
};
#else
static struct platform_driver msm_laser_focus_platform_driver = {
	.driver = {
		.name = "qcom,tof",
		.owner = THIS_MODULE,
		.of_match_table = msm_laser_focus_dt_match,
	},
};
#endif
static int __init Laura_init_module(void)
{
	int32_t rc = 0;
	pr_err("czw, Laura_init_module Enter\n");
	LOG_Handler(LOG_DBG, "%s: Enter\n", __func__);
#if I2C_DEVICE
	rc = i2c_add_driver(&hepta_i2c_driver);
#else
	rc = platform_driver_probe(&msm_laser_focus_platform_driver, Olivia_platform_probe);
#endif
	LOG_Handler(LOG_DBG, "%s rc %d\n", __func__, rc);
	pr_err("czw, Laura_init_module Exit\n");
	return rc;
}
static void __exit Laura_driver_exit(void)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
#if I2C_DEVICE
	i2c_del_driver(&hepta_i2c_driver);
#else
	platform_driver_unregister(&msm_laser_focus_platform_driver);
#endif
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return;
}

module_init(Laura_init_module);
module_exit(Laura_driver_exit);
MODULE_DESCRIPTION("MSM LASER_FOCUS");
MODULE_LICENSE("GPL v2");
