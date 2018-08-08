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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>
#include "../ASUS_Gsensor.h"
#include "Gsensor_property.h"

static DEVICE_ATTR(delay, 0660, gsensor_get_poll, gsensor_set_poll);
static DEVICE_ATTR(enable, 0660,gsensor_enable_show,gsensor_enable_store);
static DEVICE_ATTR(rawdata, S_IRUGO, get_gsensor_data, NULL);
static DEVICE_ATTR(Gsensor_resolution, 0660, read_gsensor_resolution, write_gsensor_resolution);
static DEVICE_ATTR(Gsensor_reset, 0660, reset_gsensor, NULL);
static DEVICE_ATTR(Gsensor_debug_message, 0660, gsensor_show_message, gsensor_set_message);

static DEVICE_ATTR(Gsensor_chip_id, 0440, gsensor_chip_id_show, NULL);
static DEVICE_ATTR(Gsensor_status, 0440, gsensor_status_show, NULL);
static DEVICE_ATTR(Gsensor_raw, 0440, gsensor_read_raw, NULL);
static DEVICE_ATTR(Gsensor_dump_reg, 0440, gsensor_dump_reg, NULL);

static DEVICE_ATTR(ZenMotion_set_flush, 0660, zenmotion_get_flush, zenmotion_set_flush);
static DEVICE_ATTR(ZenMotion_double_tap, 0660, read_gsensor_double_tap, init_gsensor_double_tap);
static DEVICE_ATTR(ZenMotion_flip, 0660, read_gsensor_flip, init_gsensor_flip);
static DEVICE_ATTR(ZenMotion_flick, 0660, read_gsensor_flick, init_gsensor_flick);
static DEVICE_ATTR(ZenMotion_flick_detection_force, 0664, read_gsensor_moving_detection, init_gsensor_moving_detection);
static DEVICE_ATTR(hands, 0660, read_gsensor_hands, init_gsensor_hands);
static DEVICE_ATTR(moving, 0660, read_gsensor_moving_detection, init_gsensor_moving_detection);

static struct attribute *kx022_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_rawdata.attr,

	&dev_attr_Gsensor_resolution.attr,
	&dev_attr_Gsensor_reset.attr,
	&dev_attr_Gsensor_chip_id.attr,
	&dev_attr_Gsensor_status.attr,
	&dev_attr_Gsensor_raw.attr,
	&dev_attr_Gsensor_dump_reg.attr,
	&dev_attr_Gsensor_debug_message.attr,

	&dev_attr_ZenMotion_set_flush.attr,
	&dev_attr_ZenMotion_double_tap.attr,
	&dev_attr_ZenMotion_flip.attr,
	&dev_attr_ZenMotion_flick_detection_force.attr,
	&dev_attr_ZenMotion_flick.attr,
	&dev_attr_hands.attr,
	&dev_attr_moving.attr,
	NULL
};

struct attribute_group kx022_attribute_group = {
	.attrs = kx022_attributes
};

