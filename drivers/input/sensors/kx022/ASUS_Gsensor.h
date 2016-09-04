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

#ifndef __LINUX_ASUS_GSENSOR_H
#define __LINUX_ASUS_GSENSOR_H
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include "sysfs/Gsensor_sysfs.h"
#include "property/Gsensor_property.h"
#include "kionix_gsensor.h"
#include "motion_detection/Gsensor_motion_detection.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/*************************************************
 *           Use to Setting device state          *
 *************************************************/
#define KX022_DEVICE_DISABLE	0
#define KX022_ACC_ENABLE	1
#define KX022_ORI_ENABLE 	2
#define KX022_BOTH_ENABLE 	3

/*************************************************
 *   Use to early suspend/resume state   *
 *************************************************/
#define KX022_RESUME_DISABLE		0
#define KX022_RESUME_ENABLE 		1
#define KX022_RESUME_MISSDISABLE	2
#define KX022_RESUME_MISSENABLE	3

/*************************************************
 *               Use to output data rate            *
 *************************************************/
struct kionix_odr_table {
	unsigned int cutoff;
	u8 mask;
	int RES;
};

/*************************************************
 * 	kionix kernel driver data struture
 *************************************************/
struct ASUS_Gsensor_data	{
	struct i2c_client		*client;
	struct input_dev		*input_dev;
	struct input_dev		*input_dev_zen;
	struct mutex			lock;				/* For muxtex lock */
	unsigned int			last_poll_interval;
	atomic_t		enabled;
	u8		ctrl_reg1;
	u8		data_ctrl;
	u8		int_ctrl;
	int		suspend_resume_state;
	int		resume_enable;
	int		irq_status;
	int		irq;
	int		event_irq_status;
	int		event_irq;

/* ASUS_BSP +++ Peter_Lu For CTS verify sensor report rate test fail work around +++ */
	ktime_t	timestamp;
/* ASUS_BSP --- Peter_Lu */

	/* For Gsensor motion detection */
	struct delayed_work		flick_work;
	struct workqueue_struct	*flick_workqueue;
	/* Work used for activity/wake-up */
	struct delayed_work		moving_work;
	struct workqueue_struct	*moving_workqueue;

	/* For Setting Flick motion detect G-force */
	u8		ATH_ctrl;
	u8		zen_state;

#ifdef CONFIG_INPUT_KXTJ9_POLLED_MODE
	struct input_polled_dev *poll_dev;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND	
	struct early_suspend gsensor_early_suspendresume;
#endif
// for enable motion detect , added by cheng_kao 2014.02.12 ++
	u8 wufe_rate;
	u8 wufe_timer;
	u8 wufe_thres;
// for enable motion detect , added by cheng_kao 2014.02.12 --
// added by cheng_kao 2013.06.01  for sensors calibration ++
	int accel_cal_data[6];
	int accel_cal_offset[3];
	int accel_cal_sensitivity[3];
// added by cheng_kao 2013.06.01  for sensors calibration --
// for enable motion detect , added by cheng_kao 2014.02.12 ++
	int motion_detect_threshold_x;
	int motion_detect_threshold_y;
	int motion_detect_threshold_z;
	int motion_detect_timer;
	int chip_interrupt_mode;
// for enable motion detect , added by cheng_kao 2014.02.12 --
	struct calidata asus_gsensor_cali_data;
	int x_gain_pos_data;
	int y_gain_pos_data;
	int z_gain_pos_data;
	int x_gain_neg_data;
	int y_gain_neg_data;
	int z_gain_neg_data;
	int data_report_count;
};

/* Work used for activity/wake-up */
#if 0
struct Gsensor_work {
	struct delayed_work gsensor_delayed_work;
	int state;
};
struct Gsensor_work *gsensor_work_ptr;
#endif

extern ssize_t gsensor_chip_id_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_status_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_dump_reg(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_read_raw(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_r_en_mt(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_w_en_mt(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

extern ssize_t gsensor_show_message(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_set_message(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t gsensor_get_poll(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_set_poll(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t gsensor_enable_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t get_gsensor_data(struct device *dev, struct device_attribute *devattr, char *buf);
extern ssize_t read_gsensor_resolution(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t write_gsensor_resolution(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t reset_gsensor(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_get_flick_detect_force(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_set_flick_detect_force(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t init_gsensor_double_tap(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t init_gsensor_flip(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t init_gsensor_hands(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t init_gsensor_flick(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t init_gsensor_moving_detection(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t read_gsensor_double_tap(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t read_gsensor_flip(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t read_gsensor_hands(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t read_gsensor_flick(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t read_gsensor_moving_detection(struct device *dev, struct device_attribute *attr, char *buf);

/* ASUS_BSP +++ Peter_Lu "For CTS verify Zen-Motion flush test faul issue " */
extern ssize_t zenmotion_get_flush(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t zenmotion_set_flush(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
/* ASUS_BSP --- Peter_Lu */

#endif
