/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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
#ifndef _I2C_DEV_DBGFS_H
#define _I2C_DEV_DBGFS_H

#include <linux/debugfs.h>
#include <linux/i2c.h>

#ifdef CONFIG_DEBUG_FS
int i2c_dev_dfs_add_controller(struct i2c_client *ctrl);
int i2c_dev_dfs_del_controller(struct i2c_client *ctrl);
#else
static inline int i2c_dev_dfs_add_controller(struct i2c_client *ctrl)
{
	return 0;
}
static inline int i2c_dev_dfs_del_controller(struct i2c_client *ctrl)
{
	return 0;
}
#endif

struct dentry *i2c_dev_dfs_create_file(struct i2c_client *ctrl,
					const char *name, void *data,
					const struct file_operations *fops);

#endif /* _SPMI_DBGFS_H */
