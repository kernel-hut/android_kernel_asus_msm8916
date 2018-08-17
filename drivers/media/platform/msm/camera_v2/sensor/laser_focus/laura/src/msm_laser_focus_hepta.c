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

#define DO_CAL true
#define NO_CAL false
#define DO_MEASURE true
#define NO_MEASURE false

static int DMax = 400;
static int errorStatus = 0;

//DEFINE_MSM_MUTEX(msm_laser_focus_mutex);

//static struct v4l2_file_operations msm_laser_focus_v4l2_subdev_fops;

struct msm_laser_focus_ctrl_t *laura_t = NULL;
static bool camera_on_flag = false;

static bool calibration_flag = false;

static int laser_focus_enforce_ctrl = 0;

static bool load_calibration_data = false;

static int ATD_status;

struct msm_laser_focus_ctrl_t *get_laura_ctrl(void){
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return laura_t;
}

static int Laura_Init_Chip_Status_On_Boot(struct msm_laser_focus_ctrl_t *dev_t){
	int rc = 0, chip_status = 0;
	
	LOG_Handler(LOG_CDBG, "%s: Enter Init Chip Status\n", __func__);
	
	mutex_ctrl(laura_t, MUTEX_LOCK);
	
	rc = dev_init(laura_t);
	
	rc = Laura_device_power_up_init_interface(laura_t, NO_CAL, &calibration_flag, NO_MEASURE);
	chip_status = Laura_WaitDeviceStandby(dev_t);
	if (rc < 0 || chip_status < 0){
		LOG_Handler(LOG_ERR, "%s Device init fail !! (rc,status):(%d,%d)\n", __func__, rc, chip_status);
	} else	{
		LOG_Handler(LOG_CDBG, "%s Init init success !! (rc,status):(%d,%d)\n", __func__, rc, chip_status);
	}
	
	rc = dev_deinit(laura_t);
	
	mutex_ctrl(laura_t, MUTEX_UNLOCK);
	
	LOG_Handler(LOG_CDBG, "%s: Exit Init Chip Status\n", __func__);
	
	return rc;
}

static ssize_t ATD_Laura_device_enable_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, rc = 0;
	char messages[8];

	LOG_Handler(LOG_CDBG, "%s: Enter Power On\n", __func__);
	
	if (len > 8) {
		len = 8;
	}
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
			mutex_ctrl(laura_t, MUTEX_LOCK);
			if(camera_on_flag){
              		LOG_Handler(LOG_DBG, "%s: Camera is running, do nothing!!\n ", __func__);
				mutex_ctrl(laura_t, MUTEX_UNLOCK);
              			break;
              		}
			rc = dev_deinit(laura_t);
			//rc = power_down(laura_t);
			laura_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
			LOG_Handler(LOG_CDBG, "%s Power Off Device (%d)\n", __func__, laura_t->device_state);
			load_calibration_data=false;
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			break;
		case MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION:
			if(camera_on_flag){
              			LOG_Handler(LOG_DBG, "%s: Camera is running, do nothing!!\n ", __func__);
				break;
            		}
            		mutex_ctrl(laura_t, MUTEX_LOCK);
			if (laura_t->device_state != MSM_LASER_FOCUS_DEVICE_OFF)	{
				rc = dev_deinit(laura_t);
				//rc = power_down(laura_t);
				laura_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
			}
			//laura_t->device_state = MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION;
			//rc = power_up(laura_t);
			rc = dev_init(laura_t);
			rc = Laura_device_power_up_init_interface(laura_t, DO_CAL, &calibration_flag, NO_MEASURE);
			if (rc < 0)	{
				LOG_Handler(LOG_ERR, "%s Device trun on fail !!\n", __func__);
				laura_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				mutex_ctrl(laura_t, MUTEX_UNLOCK);
				goto DEVICE_TURN_ON_ERROR;
			} else	{
				laura_t->device_state = MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION;
				LOG_Handler(LOG_CDBG, "%s Init Device (%d)\n", __func__, laura_t->device_state);
			}
			load_calibration_data = true;
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			break;
		case MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION:
			if(camera_on_flag){
            			LOG_Handler(LOG_DBG, "%s: Camera is running, do nothing!!\n ", __func__);
            			break;
            		}
            		mutex_ctrl(laura_t, MUTEX_LOCK);
			if (laura_t->device_state != MSM_LASER_FOCUS_DEVICE_OFF)	{
				rc = dev_deinit(laura_t);
				//rc = power_down(laura_t);
				laura_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
			}
			//laura_t->device_state = MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION;
			//rc = power_up(laura_t);
			rc = dev_init(laura_t);
			rc = Laura_device_power_up_init_interface(laura_t, NO_CAL, &calibration_flag, NO_MEASURE);
			if (rc < 0)	{
				LOG_Handler(LOG_ERR, "%s Device trun on fail !!\n", __func__);
				laura_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				mutex_ctrl(laura_t, MUTEX_UNLOCK);
				goto DEVICE_TURN_ON_ERROR;
			} else	{
				laura_t->device_state = MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION;
				LOG_Handler(LOG_CDBG, "%s Init Device (%d)\n", __func__, laura_t->device_state);
			}
			load_calibration_data = false;
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			break;
		case MSM_LASER_FOCUS_DEVICE_INIT_CCI:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			laura_t->device_state = MSM_LASER_FOCUS_DEVICE_INIT_CCI;
			rc = dev_init(laura_t);
			rc = Laura_device_power_up_init_interface(laura_t, DO_CAL, &calibration_flag, NO_MEASURE);
			if (rc < 0)	{
				LOG_Handler(LOG_ERR, "%s Device turn on fail !!\n", __func__);
				laura_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				mutex_ctrl(laura_t, MUTEX_UNLOCK);
				return -EIO;
			} else	{
				laura_t->device_state = MSM_LASER_FOCUS_DEVICE_INIT_CCI;
				LOG_Handler(LOG_CDBG, "%s Init Device (%d)\n", __func__, laura_t->device_state);
			}
			load_calibration_data = true;
			camera_on_flag = true;
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			break;
		case MSM_LASER_FOCUS_DEVICE_DEINIT_CCI:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			rc = dev_deinit(laura_t);
			laura_t->device_state = MSM_LASER_FOCUS_DEVICE_DEINIT_CCI;
			LOG_Handler(LOG_CDBG, "%s Deinit Device (%d)\n", __func__, laura_t->device_state);
			load_calibration_data = false;
			camera_on_flag = false;
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			break;
		default:
			LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
			break;
		}

		LOG_Handler(LOG_FUN, "%s: Exit Power On(%d)\n", __func__, val);
	
		return len;
	
DEVICE_TURN_ON_ERROR:
	rc = dev_deinit(laura_t);
	if (rc < 0) {
		//kfree(laura_t);
		LOG_Handler(LOG_ERR, "%s Laura_deinit failed %d\n", __func__, __LINE__);
	}
	//rc = power_down(laura_t);
	//if (rc < 0) {
		//kfree(laura_t);
	//	LOG_Handler(LOG_ERR, "%s Laura_power_down failed %d\n", __func__, __LINE__);
	//}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return -EIO;
}

static int ATD_Laura_device_enable_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	seq_printf(buf, "%d\n", laura_t->device_state);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
}

static int ATD_Laura_device_enable_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
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
	int RawRange = 0;
#if 0	
	struct timeval start, now;
#endif
	LOG_Handler(LOG_FUN, "%s: Enter Read Range\n", __func__);

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

	RawRange = (int) Laura_device_read_range_interface(laura_t, load_calibration_data, &calibration_flag, &errorStatus);

	if (RawRange >= OUT_OF_RANGE) {
         RawRange = OUT_OF_RANGE;
         LOG_Handler(LOG_DBG, "%s: Reset distance from %d to %d", __func__, RawRange, OUT_OF_RANGE);
    }
	
	LOG_Handler(LOG_DBG, "%s : Get range (%d)  Device (%d)\n", __func__, RawRange , laura_t->device_state);

	seq_printf(buf, "%d\n", RawRange);

	mutex_ctrl(laura_t, MUTEX_UNLOCK);
#if 0
	now = get_current_time();
	LOG_Handler(LOG_DBG, "%d ms\n", (int) ((((now.tv_sec*1000000)+now.tv_usec)-((start.tv_sec*1000000)+start.tv_usec))));
#endif
	LOG_Handler(LOG_FUN, "%s: Exit Read Range\n", __func__);
	
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

	LOG_Handler(LOG_FUN, "%s: Enter Read Range More Info\n", __func__);

	mutex_ctrl(laura_t, MUTEX_LOCK);

	if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		seq_printf(buf, "%d\n", 0);
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return -EBUSY;
	}

	if(laser_focus_enforce_ctrl != 0){
		seq_printf(buf, "%d#%d#%d\n", laser_focus_enforce_ctrl, DMax, errorStatus);
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return 0;
	}

	errorStatus = 0;

	RawRange = (int) Laura_device_read_range_interface(laura_t, load_calibration_data, &calibration_flag, &errorStatus);

	if (RawRange >= OUT_OF_RANGE) {
		RawRange = OUT_OF_RANGE;
		LOG_Handler(LOG_DBG, "%s: Reset distance from %d to %d", __func__, RawRange, OUT_OF_RANGE);
	}

	LOG_Handler(LOG_DBG, "%s : Get range (%d)  Device (%d)\n", __func__, RawRange , laura_t->device_state);

	seq_printf(buf, "%d#%d#%d\n", RawRange, DMax, errorStatus);

	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_FUN, "%s: Exit Read Range More Info\n", __func__);
	
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
	char messages[8];

	LOG_Handler(LOG_CDBG, "%s: Enter Calibration\n", __func__);

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
			ret = Laura_device_calibration_interface(laura_t, load_calibration_data, &calibration_flag, LAURA_CALIBRATION_10_CONFIG);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			if (ret < 0)
				return ret;
			break;
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			ret = Laura_device_calibration_interface(laura_t, load_calibration_data, &calibration_flag, LAURA_CALIBRATION_40_CONFIG);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			if (ret < 0)
				return ret;
			break;
		case MSM_LASER_FOCUS_APPLY_INFINITY_CALIBRATION:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			ret = Laura_device_calibration_interface(laura_t, load_calibration_data, &calibration_flag, LAURA_CALIBRATION_INF_CONFIG);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			if (ret < 0)
				return ret;
			break;
		default:
			LOG_Handler(LOG_ERR, "%s command fail(%d) !!\n", __func__, val);
			return -EINVAL;
			break;
	}

	LOG_Handler(LOG_CDBG, "%s: Exit Calibration(%d)\n", __func__, val);
	
	return len;
}

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

static int ATD_Laura_I2C_status_check_proc_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_CDBG, "%s: Enter Status Check\n", __func__);
	
	mutex_ctrl(laura_t, MUTEX_LOCK);
	ATD_status = dev_I2C_status_check(laura_t, MSM_CAMERA_I2C_WORD_DATA);
	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	if(ATD_status==1){
		ATD_status=2;
	}

	seq_printf(buf, "%d\n", ATD_status);

	LOG_Handler(LOG_CDBG, "%s: Exit Status Check(%d)\n", __func__, ATD_status);
	
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
	LOG_Handler(LOG_CDBG, "%s: Enter\n", __func__);
	seq_printf(buf, "%d\n", ATD_status);
	LOG_Handler(LOG_CDBG, "%s: Exit\n", __func__);
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
	LOG_Handler(LOG_CDBG, "%s: Enter Debug Dump\n", __func__);
	
	//mutex_ctrl(laura_t, MUTEX_LOCK);
	laura_debug_dump(buf,v);
	//mutex_ctrl(laura_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_CDBG, "%s: Exit Debug Dump\n", __func__);

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

	LOG_Handler(LOG_CDBG, "%s: Enter Enforce\n", __func__);
	
	mutex_ctrl(laura_t, MUTEX_LOCK);
	rc = Laser_Focus_enforce(laura_t, buff, len, &laser_focus_enforce_ctrl);
	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_CDBG, "%s: Exit Enforce\n", __func__);

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

	LOG_Handler(LOG_CDBG, "%s: Enter Log Controller\n", __func__);
	
	mutex_ctrl(laura_t, MUTEX_LOCK);
	rc = Laser_Focus_log_contorl(buff, len);
	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_CDBG, "%s: Exit Log Controller\n", __func__);

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

/*++++++++++CSC Debug++++++++++*/
static int dump_Laura_debug_value1_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_LAURA_CALIBRATION_DATA];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	Laura_Read_Calibration_Value_From_File(NULL, cal_data);

	seq_printf(buf,"%d",cal_data[3]);	
	
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
	int16_t cal_data[SIZE_OF_LAURA_CALIBRATION_DATA];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	Laura_Read_Calibration_Value_From_File(NULL, cal_data);

    seq_printf(buf,"%d",cal_data[4]);

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
	int16_t cal_data[SIZE_OF_LAURA_CALIBRATION_DATA];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	Laura_Read_Calibration_Value_From_File(NULL, cal_data);

    seq_printf(buf,"%d",cal_data[6]);

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
	int16_t cal_data[SIZE_OF_LAURA_CALIBRATION_DATA];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	Laura_Read_Calibration_Value_From_File(NULL, cal_data);

    seq_printf(buf,"%d",cal_data[7]);

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
	int16_t cal_data[SIZE_OF_LAURA_CALIBRATION_DATA];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	Laura_Read_Calibration_Value_From_File(NULL, cal_data);

    seq_printf(buf,"%d",cal_data[8]);

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
	RawConfidence = swap_data(RawConfidence);
	confidence_level = (RawConfidence&0x7fff)>>4;
	
	seq_printf(buf,"%d",confidence_level);

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
/*----------CSC Debug----------*/


static void Laura_create_proc_file(void)
{
	LOG_Handler(LOG_CDBG, "%s: Enter Create Proc File\n", __func__);

	create_proc_file(STATUS_PROC_FILE, STATUS_PROC_FILE_MODE, NULL, &ATD_I2C_status_check_fops);
	create_proc_file(STATUS_PROC_FILE_FOR_CAMERA, STATUS_PROC_FILE_FOR_CAMERA_MODE, NULL, &ATD_I2C_status_check_for_camera_fops);
	create_proc_file(DEVICE_TURN_ON_FILE, DEVICE_TURN_ON_FILE_MODE, NULL, &ATD_laser_focus_device_enable_fops);
	create_proc_file(DEVICE_GET_VALUE, DEVICE_GET_VALUE_MODE, NULL, &ATD_laser_focus_device_get_range_fos);
	create_proc_file(DEVICE_GET_VALUE_MORE_INFO, DEVICE_GET_VALUE_MODE_MORE_INFO, NULL, &ATD_laser_focus_device_get_range_more_info_fos);
	create_proc_file(DEVICE_SET_CALIBRATION, DEVICE_SET_CALIBRATION_MODE, NULL, &ATD_laser_focus_device_calibration_fops);
	create_proc_file(DEVICE_GET_CALIBRATION_INPUT_DATA, DEVICE_GET_CALIBRATION_INPUT_DATA_MODE, NULL, &ATD_Laura_get_calibration_input_data_fops);
	create_proc_file(DEVICE_DUMP_REGISTER_VALUE, DEVICE_DUMP_REGISTER_VALUE_MODE, NULL, &dump_laser_focus_register_fops);
	create_proc_file(DEVICE_DUMP_DEBUG_VALUE, DEVICE_DUMP_DEBUG_VALUE_MODE, NULL, &dump_laser_focus_debug_register_fops);
	create_proc_file(DEVICE_ENFORCE_FILE, DEVICE_ENFORCE_MODE, NULL, &laser_focus_enforce_fops);
	create_proc_file(DEVICE_LOG_CTRL_FILE, DEVICE_LOG_CTRL_MODE, NULL, &laser_focus_log_contorl_fops);
	/*++++++++++CSC Debug++++++++++*/
	create_proc_file(DEVICE_DEBUG_VALUE1, DEVICE_DEBUG_VALUE_MODE, NULL, &dump_laser_focus_debug_value1_fops);
	create_proc_file(DEVICE_DEBUG_VALUE2, DEVICE_DEBUG_VALUE_MODE, NULL, &dump_laser_focus_debug_value2_fops);
	create_proc_file(DEVICE_DEBUG_VALUE3, DEVICE_DEBUG_VALUE_MODE, NULL, &dump_laser_focus_debug_value3_fops);
	create_proc_file(DEVICE_DEBUG_VALUE4, DEVICE_DEBUG_VALUE_MODE, NULL, &dump_laser_focus_debug_value4_fops);
	create_proc_file(DEVICE_DEBUG_VALUE5, DEVICE_DEBUG_VALUE_MODE, NULL, &dump_laser_focus_debug_value5_fops);
	create_proc_file(DEVICE_DEBUG_VALUE6, DEVICE_DEBUG_VALUE_MODE, NULL, &dump_laser_focus_debug_value6_fops);
	/*----------CSC Debug----------*/

	LOG_Handler(LOG_CDBG, "%s: Exit Create Proc File\n", __func__);
}

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

static const struct v4l2_subdev_internal_ops msm_laser_focus_internal_ops;

static struct v4l2_subdev_core_ops msm_laser_focus_subdev_core_ops = {
	.ioctl = NULL,
	//.s_power = msm_laser_focus_power,
};

static struct v4l2_subdev_ops msm_laser_focus_subdev_ops = {
	.core = &msm_laser_focus_subdev_core_ops,
};

static struct msm_camera_i2c_client msm_laser_focus_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};
static const struct of_device_id msm_laser_focus_dt_match[] = {
	{.compatible = "qcom,ois1", .data = NULL},
	{}
};

MODULE_DEVICE_TABLE(of, msm_laser_focus_dt_match);

static int32_t Laura_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;//, i = 0;
	//uint32_t id_info[3];
	const struct of_device_id *match;
	struct msm_camera_cci_client *cci_client = NULL;

	if(g_ASUS_laserID == 1){
                LOG_Handler(LOG_ERR, "%s: It is VL6180x sensor, do nothing!!\n", __func__);
                return rc;
        }

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
	laura_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_LASER_FOCUS;
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
	//mutex_ctrl(laura_t, MUTEX_ALLOCATE);
	mutex_ctrl(laura_t, MUTEX_INIT);

	ATD_status = 2;
	
	/* Create proc file */
	Laura_create_proc_file();
	
	Laura_Init_Chip_Status_On_Boot(laura_t);
	
	LOG_Handler(LOG_CDBG, "%s: Probe Success\n", __func__);
	return 0;
probe_failure:
	LOG_Handler(LOG_CDBG, "%s: Probe failed\n", __func__);

	return rc;
}

static struct platform_driver msm_laser_focus_platform_driver = {
	//.probe = Laura_platform_probe,
	.driver = {
		.name = "qcom,ois1",
		.owner = THIS_MODULE,
		.of_match_table = msm_laser_focus_dt_match,
	},
};

static int __init Laura_init_module(void)
{
	int32_t rc = 0;
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	rc = platform_driver_probe(&msm_laser_focus_platform_driver, Laura_platform_probe);
	LOG_Handler(LOG_DBG, "%s rc %d\n", __func__, rc);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;
}
static void __exit Laura_driver_exit(void)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	platform_driver_unregister(&msm_laser_focus_platform_driver);
	
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return;
}

module_init(Laura_init_module);
module_exit(Laura_driver_exit);
MODULE_DESCRIPTION("MSM LASER_FOCUS");
MODULE_LICENSE("GPL v2");
