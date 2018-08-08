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

 /***************************/
/* IR Sensor sysfs Module */
/**************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/input/ASH.h>

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_Sysfs"
#define SENSOR_TYPE_NAME		"IRsensor"

#undef dbg
#ifdef ASH_SYSFS_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

/***************************************/
/* Proximity read/write Calibration File*/
/**************************************/
int psensor_sysfs_read_high(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	
	
	fp = filp_open(PSENSOR_HI_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity read High Calibration open (%s) fail\n", PSENSOR_HI_CALIBRATION_FILE);
		return -ENOENT;	/*No such file or directory*/
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, buf, 6, &pos_lsts);
		buf[readlen] = '\0';		
	} else {
		err("Proximity read High Calibration strlen: f_op=NULL or op->read=NULL\n");
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);
	if(cal_val < 0) {
		err("Proximity read High Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Proximity read High Calibration : %d\n", cal_val);
	}
	
	return cal_val;
}
EXPORT_SYMBOL(psensor_sysfs_read_high);

int psensor_sysfs_read_low(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	
	
	fp = filp_open(PSENSOR_LOW_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity read Low Calibration open (%s) fail\n", PSENSOR_LOW_CALIBRATION_FILE);
		return -ENOENT;	/*No such file or directory*/
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, buf, 6, &pos_lsts);
		buf[readlen] = '\0';		
	} else {
		err("Proximity read Low Calibration strlen f_op=NULL or op->read=NULL\n");
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);	
	if(cal_val < 0) {
		err("Proximity read Low Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Proximity read Low Calibration : %d\n", cal_val);
	}	
	
	return cal_val;
}
EXPORT_SYMBOL(psensor_sysfs_read_low);

bool psensor_sysfs_write_high(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(PSENSOR_HI_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity write High Calibration open (%s) fail\n", PSENSOR_HI_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Proximity Hi-Calibration strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Proximity write High Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(psensor_sysfs_write_high);

bool psensor_sysfs_write_low(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(PSENSOR_LOW_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Proximity write Low Calibration open (%s) fail\n", PSENSOR_LOW_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Proximity Lo-Calibration strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Proximity write Low Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(psensor_sysfs_write_low);

/**************************************/
/* Light Sensor read/write Calibration*/
/*************************************/
int lsensor_sysfs_read_200lux(void)
{
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char buf[16];
	int cal_val = 0, readlen = 0;
	mm_segment_t old_fs;	

	fp = filp_open(LSENSOR_200LUX_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Light Sensor read 200lux Calibration open (%s) fail\n", LSENSOR_200LUX_CALIBRATION_FILE);
		return -ENOENT;	/*No such file or directory*/
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, buf, 16, &pos_lsts);
		buf[readlen] = '\0';
	} else {
		err("Light Sensor read 200lux Calibration f_op=NULL or op->read=NULL\n");
		return -ENXIO;	/*No such device or address*/
	}

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);	
	if(cal_val < 0) {
		err("Light Sensor read 200lux Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Light Sensor read 200lux Calibration: Cal: %d\n", cal_val);
	}	
	
	return cal_val;
}
EXPORT_SYMBOL(lsensor_sysfs_read_200lux);

int lsensor_sysfs_read_1000lux(void)
{
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char buf[16];
	int cal_val = 0, readlen = 0;
	mm_segment_t old_fs;	

	fp = filp_open(LSENSOR_1000LUX_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Light Sensor read 1000lux Calibration open (%s) fail\n", LSENSOR_1000LUX_CALIBRATION_FILE);
		return -ENOENT;	/*No such file or directory*/
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, buf, 16, &pos_lsts);
		buf[readlen] = '\0';
	} else {
		err("Light Sensor read 1000lux Calibration f_op=NULL or op->read=NULL\n");
		return -ENXIO;	/*No such device or address*/
	}

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);	
	if(cal_val < 0) {
		err("Light Sensor read 1000lux Calibration is FAIL. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		dbg("Light Sensor read 1000lux Calibration: Cal: %d\n", cal_val);
	}	
	
	return cal_val;
}
EXPORT_SYMBOL(lsensor_sysfs_read_1000lux);

bool lsensor_sysfs_write_200lux(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(LSENSOR_200LUX_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Light Sensor write 200lux Calibration open (%s) fail\n", LSENSOR_200LUX_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Light Sensor write 200lux Calibration strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Light Sensor write 200lux Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(lsensor_sysfs_write_200lux);

bool lsensor_sysfs_write_1000lux(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);
	
	fp = filp_open(LSENSOR_1000LUX_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		err("Light Sensor write 1000lux Calibration open (%s) fail\n", LSENSOR_1000LUX_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		err("Light Sensor write 1000lux Calibration strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	
	log("Light Sensor write 1000lux Calibration : %s\n", buf);
	
	return true;
}
EXPORT_SYMBOL(lsensor_sysfs_write_1000lux);

