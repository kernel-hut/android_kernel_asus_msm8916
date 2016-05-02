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

#ifndef __LINUX_GSENSOR_SYSFS_H
#define __LINUX_GSENSOR_SYSFS_H

#define GSEN_CALI_FILE		"/factory/KXTF9_Calibration_v1.ini"
#define GSEN_CALI_FILE_V2	"/factory/KXTF9_Calibration_v2.ini"
#define GSEN_CALI_FILE_V3	"/factory/ASUS_Gsensor_Calibration.ini"

#define GSENSOR_MAX_RESOLUTION 8192

struct calidata	{
	int x_max;
	int x_min;
	int y_max;
	int y_min;
	int z_max;
	int z_min;
	int x_0g[4];
	int y_0g[4];
	int z_0g[4];
	bool valid;
	int version;

	int x_offset;
	int y_offset;
	int z_offset;
	float x_gain_pos;
	float x_gain_neg;
	float y_gain_pos;
	float y_gain_neg;
	float z_gain_pos;
	float z_gain_neg;
};

int Gsensor_sysfs_read_calibration_data(struct calidata *cali_data);
void init_cali_data (struct calidata *calidata);
int get_cali_data(char *filename, struct calidata *calidata);

#endif

