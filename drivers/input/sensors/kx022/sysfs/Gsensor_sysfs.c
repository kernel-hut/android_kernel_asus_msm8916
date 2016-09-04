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
#include <linux/fs.h>
#include <asm/uaccess.h>
#include "Gsensor_sysfs.h"

int Gsensor_sysfs_read_calibration_data(struct calidata *cali_data)
{
	int ret = 0;

	/* Get calibration data */
	ret = get_cali_data(GSEN_CALI_FILE_V3, cali_data);
	//if (ret < 0 || (g_ASUS_hwID >= ZE500KL_SR1 && cali_data->z_max == 0))	{
	if (ret < 0 || (cali_data->z_max == 0))	{
		printk("[Gsensor] New calibration file %s exists, try get old version data %s\n", GSEN_CALI_FILE_V3, GSEN_CALI_FILE_V2);
		ret = get_cali_data(GSEN_CALI_FILE_V2, cali_data);
	}
	if (ret < 0)	{
		printk("[Gsensor] Get calibration data fail!!!\n");
		cali_data->version = 2;
	}

	return 0;
}

void init_cali_data (struct calidata *calidata)
{
	calidata->valid = false;
	calidata->x_max = 0;
	calidata->x_min = 0;
	calidata->y_max = 0;
	calidata->y_min = 0;
	calidata->z_max = 0;
	calidata->z_min = 0;
	calidata->x_offset = 0;
	calidata->y_offset = 0;
	calidata->z_offset = 0;
	calidata->version = 0;
	return;
}

int get_cali_data(char *filename, struct calidata *calidata)
{
	int ret = 0;
	int buf[21] = {0};
	int readlen = 0;
	char file_buf[256] = "";
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	struct file *fp = NULL;

	fp = filp_open(filename, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (!IS_ERR_OR_NULL(fp))	{
		printk("[Gsensor] Open %s success\n",filename);

		old_fs = get_fs();
		set_fs(KERNEL_DS);

		if (fp->f_op != NULL && fp->f_op->read != NULL)	{
			pos_lsts = 0;
			readlen = fp->f_op->read(fp, file_buf, sizeof(file_buf), &pos_lsts);
			//file_buf[readlen] = '\0';
		} else {
			printk("[Gsensor] Read Calibration data : f_op=NULL or op->read=NULL\n");
			return -ENXIO;	/*No such device or address*/
		}
		set_fs(old_fs);
		filp_close(fp, NULL);

		sscanf(file_buf, "%d %d %d %d %d %d\n%d %d %d\n%d %d %d %d\n%d %d %d %d\n%d %d %d %d"	, &buf[0]
										, &buf[1]
										, &buf[2]
										, &buf[3]
										, &buf[4]
										, &buf[5]
										, &buf[6]
										, &buf[7]
										, &buf[8]
										, &buf[9]
										, &buf[10]
										, &buf[11]
										, &buf[12]
										, &buf[13]
										, &buf[14]
										, &buf[15]
										, &buf[16]
										, &buf[17]
										, &buf[18]
										, &buf[19]
										, &buf[20]);

		if(1){
			//axis_transform(calidata,buf);
			calidata->x_max = buf[0];
			calidata->x_min = buf[1];
			calidata->y_max = buf[2];
			calidata->y_min = buf[3];
			calidata->z_max = buf[4];
			calidata->z_min = buf[5];
			calidata->x_offset = buf[6];
			calidata->y_offset = buf[7];
			calidata->z_offset = buf[8];
			calidata->x_0g[0] = buf[9];
			calidata->x_0g[1] = buf[10];
			calidata->x_0g[2] = buf[11];
			calidata->x_0g[3] = buf[12];
			calidata->y_0g[0] = buf[13];
			calidata->y_0g[1] = buf[14];
			calidata->y_0g[2] = buf[15];
			calidata->y_0g[3] = buf[16];
			calidata->z_0g[0] = buf[17];
			calidata->z_0g[1] = buf[18];
			calidata->z_0g[2] = buf[19];
			calidata->z_0g[3] = buf[20];
			printk("[Gsensor] min max %d %d %d %d %d %d\n",calidata->x_max
															,calidata->x_min
															,calidata->y_max
															,calidata->y_min
															,calidata->z_max
															,calidata->z_min);
													
			printk("[Gsensor] Real X 0G offset %d %d %d %d\n"	,calidata->x_0g[0]
																,calidata->x_0g[1]
																,calidata->x_0g[2]
																,calidata->x_0g[3]);

			printk("[Gsensor] Real Y 0G offset %d %d %d %d\n"	,calidata->y_0g[0]
																,calidata->y_0g[1]
																,calidata->y_0g[2]
																,calidata->y_0g[3]);

			printk("[Gsensor] Real Z 0G offset %d %d %d %d\n"	,calidata->z_0g[0]
																,calidata->z_0g[1]
																,calidata->z_0g[2]
																,calidata->z_0g[3]);

			printk("[Gsensor]  0G offset %d %d %d\n"	,calidata->x_offset
													,calidata->y_offset
													,calidata->z_offset);
			calidata->valid = true;
		}else{
			printk("[Gsensor] Read calibration data error!!\n");
			calidata->valid = false;
		}
		
		if(!strcmp(filename, GSEN_CALI_FILE_V3))
			calidata->version = 3;
		else
			calidata->version = 2;
		
		ret = 0;
	}else{
		printk("[Gsensor] Open %s failed(%d)\n", filename, (int)(IS_ERR_OR_NULL(fp)));
		calidata->valid = false;
		ret = -EEXIST;
	}
	return ret;
}

