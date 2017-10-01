/* Copyright (c) 2013-2015,2017 The Linux Foundation. All rights reserved.
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

#define SENSOR_DRIVER_I2C "camera"
/* Header file declaration */
#include "msm_sensor.h"
#include "msm_sd.h"
#include "camera.h"
#include "msm_cci.h"
#include "msm_camera_dt_util.h"

//ASUS_BSP Stimber_Hsueh "Implement for ATD interface" +++
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
//ASUS_BSP Stimber_Hsueh "Implement for ATD interface" ---

//ASUS_BSP Stimber_Hsueh +++
#include <media/v4l2-subdev.h>
#define TSB_OTP_DATA_SIZE		0x24
#define SENSOR_MAX_RETRIES      50 
#define REAR_MODULE_OTP_SIZE 482
static char rear_module_otp[REAR_MODULE_OTP_SIZE];
//ASUS_BSP Stimber_Hsueh ---
//ASUS_BSP+++ CR_000 Randy_Change@asus.com.tw [2015/3/5] Modify Begin
#define MODULE_OTP_BULK_SIZE 32
#define MODULE_OTP_BULK 3
#define MODULE_OTP_SIZE MODULE_OTP_BULK_SIZE*MODULE_OTP_BULK
#define MODULE_OTP_BUF_SIZE MODULE_OTP_SIZE*5+MODULE_OTP_BULK*sizeof('\n')*5+1
static  char front_module_otp[MODULE_OTP_BUF_SIZE];
static unsigned char g_front_otp_created = 0;
static void create_front_otp_proc_file(void);
//ASUS_BSP--- CR_000 Randy_Change@asus.com.tw [2015/3/5] Modify End
//ASUS_BSP PJ_Ma +++ "implement imx219 read OTP"
#define IMX219_ASUS_OTP_SIZE 96
//static u8 imx219_asus_otp_value[IMX219_ASUS_OTP_SIZE]={0};
#define IMX219_ASUS_OTP_START 0x3204
//#define IMX219_ASUS_OTP_MODULE_REG 0x320c
//ASUS_BSP PJ_Ma --- "implement imx219 read OTP"

/* Logging macro */
#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

#define SENSOR_MAX_MOUNTANGLE (360)

static struct v4l2_file_operations msm_sensor_v4l2_subdev_fops;

/* Static declaration */
static struct msm_sensor_ctrl_t *g_sctrl[MAX_CAMERAS];

//ASUS_BSP Stimber_Hsueh "Implement for ATD interface" +++
unsigned char g_camera_status[4] = {1, 1, 1,1};
unsigned char g_vga_status[2] = {1, 1};
int g_camera_id = MAX_CAMERAS;
static unsigned char g_camera_status_created = 0;
static unsigned char g_front_camera_status_created = 0;
int g_rear_camera_ref_cnt = -1;
int g_front_camera_ref_cnt = -1;
static void create_rear_status_proc_file(void);
static void create_front_status_proc_file(void);
static void remove_proc_file(void);
//ASUS_BSP Stimber_Hsueh "Implement for ATD interface" ---

//ASUS_BSP Stimber_Hsueh +++
static unsigned char g_rear_resolution_created = 0;
char *g_rear_resolution = "";
static void create_rear_resolution_proc_file(void);

static unsigned char g_front_resolution_created = 0;
char *g_front_resolution = "";
static void create_front_resolution_proc_file(void);
//ASUS_BSP Stimber_Hsueh ---

//ASUS_BSP Stimber_Hsueh +++
static unsigned char g_rear_module_created = 0;
char *g_rear_module = "";
static void create_rear_module_proc_file(void);

static unsigned char g_front_module_created = 0;
char *g_front_module = "";
static void create_front_module_proc_file(void);
//ASUS_BSP Stimber_Hsueh ---

//ASUS_BSP Stimber_Hsueh +++
static unsigned char g_rear_otp_created = 0;
static void create_rear_otp_proc_file(void);
static void tsb_otp_read(void);
//ASUS_BSP Stimber_Hsueh ---
static void imx219_otp_read(void); //ASUS_BSP PJ_Ma +++
static void create_gc2155_obvalue_proc_file(void);
static void mn34150_otp_read(void); //ASUS_BSP PJ_Ma +++

static int msm_sensor_platform_remove(struct platform_device *pdev)
{
	struct msm_sensor_ctrl_t  *s_ctrl;

	pr_err("%s: sensor FREE\n", __func__);

	s_ctrl = g_sctrl[pdev->id];
	if (!s_ctrl) {
		pr_err("%s: sensor device is NULL\n", __func__);
		return 0;
	}

	msm_sensor_free_sensor_data(s_ctrl);
	kfree(s_ctrl->msm_sensor_mutex);
	kfree(s_ctrl->sensor_i2c_client);
	kfree(s_ctrl);
	g_sctrl[pdev->id] = NULL;

	remove_proc_file();	//ASUS_BSP Stimber_Hsueh "Implement for ATD interface"
	return 0;
}


static const struct of_device_id msm_sensor_driver_dt_match[] = {
	{.compatible = "qcom,camera"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_sensor_driver_dt_match);

static struct platform_driver msm_sensor_platform_driver = {
	.driver = {
		.name = "qcom,camera",
		.owner = THIS_MODULE,
		.of_match_table = msm_sensor_driver_dt_match,
	},
	.remove = msm_sensor_platform_remove,
};

static struct v4l2_subdev_info msm_sensor_driver_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static int32_t msm_sensor_driver_create_i2c_v4l_subdev
			(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint32_t session_id = 0;
	struct i2c_client *client = s_ctrl->sensor_i2c_client->client;

	CDBG("%s %s I2c probe succeeded\n", __func__, client->name);
	rc = camera_init_v4l2(&client->dev, &session_id);
	if (rc < 0) {
		pr_err("failed: camera_init_i2c_v4l2 rc %d", rc);
		return rc;
	}
	CDBG("%s rc %d session_id %d\n", __func__, rc, session_id);
	snprintf(s_ctrl->msm_sd.sd.name,
		sizeof(s_ctrl->msm_sd.sd.name), "%s",
		s_ctrl->sensordata->sensor_name);
	v4l2_i2c_subdev_init(&s_ctrl->msm_sd.sd, client,
		s_ctrl->sensor_v4l2_subdev_ops);
	v4l2_set_subdevdata(&s_ctrl->msm_sd.sd, client);
	s_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&s_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	s_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	s_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_SENSOR;
	s_ctrl->msm_sd.sd.entity.name =	s_ctrl->msm_sd.sd.name;
	s_ctrl->sensordata->sensor_info->session_id = session_id;
	s_ctrl->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x3;
	rc = msm_sd_register(&s_ctrl->msm_sd);
	if (rc < 0) {
		pr_err("failed: msm_sd_register rc %d", rc);
		return rc;
	}
	CDBG("%s:%d\n", __func__, __LINE__);
	return rc;
}

static int32_t msm_sensor_driver_create_v4l_subdev
			(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint32_t session_id = 0;

	rc = camera_init_v4l2(&s_ctrl->pdev->dev, &session_id);
	if (rc < 0) {
		pr_err("failed: camera_init_v4l2 rc %d", rc);
		return rc;
	}
	CDBG("rc %d session_id %d", rc, session_id);
	s_ctrl->sensordata->sensor_info->session_id = session_id;

	/* Create /dev/v4l-subdevX device */
	v4l2_subdev_init(&s_ctrl->msm_sd.sd, s_ctrl->sensor_v4l2_subdev_ops);
	snprintf(s_ctrl->msm_sd.sd.name, sizeof(s_ctrl->msm_sd.sd.name), "%s",
		s_ctrl->sensordata->sensor_name);
	v4l2_set_subdevdata(&s_ctrl->msm_sd.sd, s_ctrl->pdev);
	s_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&s_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	s_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	s_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_SENSOR;
	s_ctrl->msm_sd.sd.entity.name = s_ctrl->msm_sd.sd.name;
	s_ctrl->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x3;
	rc = msm_sd_register(&s_ctrl->msm_sd);
	if (rc < 0) {
		pr_err("failed: msm_sd_register rc %d", rc);
		return rc;
	}
	msm_sensor_v4l2_subdev_fops = v4l2_subdev_fops;
#ifdef CONFIG_COMPAT
	msm_sensor_v4l2_subdev_fops.compat_ioctl32 =
		msm_sensor_subdev_fops_ioctl;
#endif
	s_ctrl->msm_sd.sd.devnode->fops =
		&msm_sensor_v4l2_subdev_fops;

	return rc;
}

static int32_t msm_sensor_fill_eeprom_subdevid_by_name(
				struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	const char *eeprom_name;
	struct device_node *src_node = NULL;
	uint32_t val = 0, count = 0, eeprom_name_len;
	int i;
	int32_t *eeprom_subdev_id;
	struct  msm_sensor_info_t *sensor_info;
	struct device_node *of_node = s_ctrl->of_node;
	const void *p;

	if (!s_ctrl->sensordata->eeprom_name || !of_node)
		return -EINVAL;

	eeprom_name_len = strlen(s_ctrl->sensordata->eeprom_name);
	if (eeprom_name_len >= MAX_SENSOR_NAME)
		return -EINVAL;

	sensor_info = s_ctrl->sensordata->sensor_info;
	eeprom_subdev_id = &sensor_info->subdev_id[SUB_MODULE_EEPROM];
	/*
	 * string for eeprom name is valid, set sudev id to -1
	 *  and try to found new id
	 */
	*eeprom_subdev_id = -1;

	if (0 == eeprom_name_len)
		return 0;

	CDBG("Try to find eeprom subdev for %s\n",
			s_ctrl->sensordata->eeprom_name);
	p = of_get_property(of_node, "qcom,eeprom-src", &count);
	if (!p || !count)
		return 0;

	count /= sizeof(uint32_t);
	for (i = 0; i < count; i++) {
		eeprom_name = NULL;
		src_node = of_parse_phandle(of_node, "qcom,eeprom-src", i);
		if (!src_node) {
			pr_err("eeprom src node NULL\n");
			continue;
		}
		rc = of_property_read_string(src_node, "qcom,eeprom-name",
			&eeprom_name);
		if (rc < 0) {
			pr_err("failed\n");
			of_node_put(src_node);
			continue;
		}
		if (strcmp(eeprom_name, s_ctrl->sensordata->eeprom_name))
			continue;

		rc = of_property_read_u32(src_node, "cell-index", &val);

		CDBG("%s qcom,eeprom cell index %d, rc %d\n", __func__,
			val, rc);
		if (rc < 0) {
			pr_err("failed\n");
			of_node_put(src_node);
			continue;
		}

		*eeprom_subdev_id = val;
		CDBG("Done. Eeprom subdevice id is %d\n", val);
		of_node_put(src_node);
		src_node = NULL;
		break;
	}

	return rc;
}

static int32_t msm_sensor_fill_actuator_subdevid_by_name(
				struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct device_node *src_node = NULL;
	uint32_t val = 0, actuator_name_len;
	int32_t *actuator_subdev_id;
	struct  msm_sensor_info_t *sensor_info;
	struct device_node *of_node = s_ctrl->of_node;

	if (!s_ctrl->sensordata->actuator_name || !of_node)
		return -EINVAL;

	actuator_name_len = strlen(s_ctrl->sensordata->actuator_name);
	if (actuator_name_len >= MAX_SENSOR_NAME)
		return -EINVAL;

	sensor_info = s_ctrl->sensordata->sensor_info;
	actuator_subdev_id = &sensor_info->subdev_id[SUB_MODULE_ACTUATOR];
	/*
	 * string for actuator name is valid, set sudev id to -1
	 * and try to found new id
	 */
	*actuator_subdev_id = -1;

	if (0 == actuator_name_len)
		return 0;

	src_node = of_parse_phandle(of_node, "qcom,actuator-src", 0);
	if (!src_node) {
		CDBG("%s:%d src_node NULL\n", __func__, __LINE__);
	} else {
		rc = of_property_read_u32(src_node, "cell-index", &val);
		CDBG("%s qcom,actuator cell index %d, rc %d\n", __func__,
			val, rc);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			return -EINVAL;
		}
		*actuator_subdev_id = val;
		of_node_put(src_node);
		src_node = NULL;
	}

	return rc;
}

static int32_t msm_sensor_fill_ois_subdevid_by_name(
				struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct device_node *src_node = NULL;
	uint32_t val = 0, ois_name_len;
	int32_t *ois_subdev_id;
	struct  msm_sensor_info_t *sensor_info;
	struct device_node *of_node = s_ctrl->of_node;

	if (!s_ctrl->sensordata->ois_name || !of_node)
		return -EINVAL;

	ois_name_len = strlen(s_ctrl->sensordata->ois_name);
	if (ois_name_len >= MAX_SENSOR_NAME)
		return -EINVAL;

	sensor_info = s_ctrl->sensordata->sensor_info;
	ois_subdev_id = &sensor_info->subdev_id[SUB_MODULE_OIS];
	/*
	 * string for ois name is valid, set sudev id to -1
	 * and try to found new id
	 */
	*ois_subdev_id = -1;

	if (0 == ois_name_len)
		return 0;

	src_node = of_parse_phandle(of_node, "qcom,ois-src", 0);
	if (!src_node) {
		CDBG("%s:%d src_node NULL\n", __func__, __LINE__);
	} else {
		rc = of_property_read_u32(src_node, "cell-index", &val);
		CDBG("%s qcom,ois cell index %d, rc %d\n", __func__,
			val, rc);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			return -EINVAL;
		}
		*ois_subdev_id = val;
		of_node_put(src_node);
		src_node = NULL;
	}

	return rc;
}

static int32_t msm_sensor_fill_slave_info_init_params(
	struct msm_camera_sensor_slave_info *slave_info,
	struct msm_sensor_info_t *sensor_info)
{
	struct msm_sensor_init_params *sensor_init_params;
	if (!slave_info ||  !sensor_info)
		return -EINVAL;

	if (!slave_info->is_init_params_valid)
		return 0;

	sensor_init_params = &slave_info->sensor_init_params;
	if (INVALID_CAMERA_B != sensor_init_params->position)
		sensor_info->position =
			sensor_init_params->position;

	if (SENSOR_MAX_MOUNTANGLE > sensor_init_params->sensor_mount_angle) {
		sensor_info->sensor_mount_angle =
			sensor_init_params->sensor_mount_angle;
		sensor_info->is_mount_angle_valid = 1;
	}

	if (CAMERA_MODE_INVALID != sensor_init_params->modes_supported)
		sensor_info->modes_supported =
			sensor_init_params->modes_supported;

	return 0;
}


static int32_t msm_sensor_validate_slave_info(
	struct msm_sensor_info_t *sensor_info)
{
	if (INVALID_CAMERA_B == sensor_info->position) {
		sensor_info->position = BACK_CAMERA_B;
		CDBG("%s:%d Set default sensor position\n",
			__func__, __LINE__);
	}
	if (CAMERA_MODE_INVALID == sensor_info->modes_supported) {
		sensor_info->modes_supported = CAMERA_MODE_2D_B;
		CDBG("%s:%d Set default sensor modes_supported\n",
			__func__, __LINE__);
	}
	if (SENSOR_MAX_MOUNTANGLE <= sensor_info->sensor_mount_angle) {
		sensor_info->sensor_mount_angle = 0;
		CDBG("%s:%d Set default sensor mount angle\n",
			__func__, __LINE__);
		sensor_info->is_mount_angle_valid = 1;
	}
	return 0;
}

#ifdef CONFIG_COMPAT
static int32_t msm_sensor_get_pw_settings_compat(
	struct msm_sensor_power_setting *ps,
	struct msm_sensor_power_setting *us_ps, uint32_t size)
{
	int32_t rc = 0, i = 0;
	struct msm_sensor_power_setting32 *ps32 =
		kzalloc(sizeof(*ps32) * size, GFP_KERNEL);

	if (!ps32) {
		pr_err("failed: no memory ps32");
		return -ENOMEM;
	}
	if (copy_from_user(ps32, (void *)us_ps, sizeof(*ps32) * size)) {
		pr_err("failed: copy_from_user");
		kfree(ps32);
		return -EFAULT;
	}
	for (i = 0; i < size; i++) {
		ps[i].config_val = ps32[i].config_val;
		ps[i].delay = ps32[i].delay;
		ps[i].seq_type = ps32[i].seq_type;
		ps[i].seq_val = ps32[i].seq_val;
	}
	kfree(ps32);
	return rc;
}
#endif

static int32_t msm_sensor_create_pd_settings(void *setting,
	struct msm_sensor_power_setting *pd, uint32_t size_down,
	struct msm_sensor_power_setting *pu)
{
	int32_t rc = 0;
	int c, end;
	struct msm_sensor_power_setting pd_tmp;

	pr_err("Generating power_down_setting");

#ifdef CONFIG_COMPAT
	if (is_compat_task()) {
		rc = msm_sensor_get_pw_settings_compat(
			pd, pu, size_down);
		if (rc < 0) {
			pr_err("failed");
			return -EFAULT;
		}
	} else
#endif
	{
		if (copy_from_user(pd, (void *)pu, sizeof(*pd) * size_down)) {
			pr_err("failed: copy_from_user");
			return -EFAULT;
		}
	}
	/* reverse */
	end = size_down - 1;
	for (c = 0; c < size_down/2; c++) {
		pd_tmp = pd[c];
		pd[c] = pd[end];
		pd[end] = pd_tmp;
		end--;
	}
	return rc;
}

static int32_t msm_sensor_get_power_down_settings(void *setting,
	struct msm_camera_sensor_slave_info *slave_info,
	struct msm_camera_power_ctrl_t *power_info)
{
	int32_t rc = 0;
	uint16_t size_down = 0;
	uint16_t i = 0;
	struct msm_sensor_power_setting *pd = NULL;

	/* DOWN */
	size_down = slave_info->power_setting_array.size_down;
	if (!size_down || size_down > MAX_POWER_CONFIG)
		size_down = slave_info->power_setting_array.size;
	/* Validate size_down */
	if (size_down > MAX_POWER_CONFIG) {
		pr_err("failed: invalid size_down %d", size_down);
		return -EINVAL;
	}
	/* Allocate memory for power down setting */
	pd = kzalloc(sizeof(*pd) * size_down, GFP_KERNEL);
	if (!pd) {
		pr_err("failed: no memory power_setting %pK", pd);
		return -EFAULT;
	}

	if (slave_info->power_setting_array.power_down_setting) {
#ifdef CONFIG_COMPAT
		if (is_compat_task()) {
			rc = msm_sensor_get_pw_settings_compat(
				pd, slave_info->power_setting_array.
				power_down_setting, size_down);
			if (rc < 0) {
				pr_err("failed");
				kfree(pd);
				return -EFAULT;
			}
		} else
#endif
		if (copy_from_user(pd, (void *)slave_info->power_setting_array.
				power_down_setting, sizeof(*pd) * size_down)) {
			pr_err("failed: copy_from_user");
			kfree(pd);
			return -EFAULT;
		}
	} else {

		rc = msm_sensor_create_pd_settings(setting, pd, size_down,
			slave_info->power_setting_array.power_setting);
		if (rc < 0) {
			pr_err("failed");
			kfree(pd);
			return -EFAULT;
		}
	}

	/* Fill power down setting and power down setting size */
	power_info->power_down_setting = pd;
	power_info->power_down_setting_size = size_down;

	/* Print power setting */
	for (i = 0; i < size_down; i++) {
		CDBG("DOWN seq_type %d seq_val %d config_val %ld delay %d",
			pd[i].seq_type, pd[i].seq_val,
			pd[i].config_val, pd[i].delay);
	}
	return rc;
}

static int32_t msm_sensor_get_power_up_settings(void *setting,
	struct msm_camera_sensor_slave_info *slave_info,
	struct msm_camera_power_ctrl_t *power_info)
{
	int32_t rc = 0;
	uint16_t size = 0;
	uint16_t i = 0;
	struct msm_sensor_power_setting *pu = NULL;

	size = slave_info->power_setting_array.size;

	/* Validate size */
	if ((size == 0) || (size > MAX_POWER_CONFIG)) {
		pr_err("failed: invalid power_setting size_up = %d\n", size);
		return -EINVAL;
	}

	/* Allocate memory for power up setting */
	pu = kzalloc(sizeof(*pu) * size, GFP_KERNEL);
	if (!pu) {
		pr_err("failed: no memory power_setting %pK", pu);
		return -ENOMEM;
	}

#ifdef CONFIG_COMPAT
	if (is_compat_task()) {
		rc = msm_sensor_get_pw_settings_compat(pu,
			slave_info->power_setting_array.
				power_setting, size);
		if (rc < 0) {
			pr_err("failed");
			kfree(pu);
			return -EFAULT;
		}
	} else
#endif
	{
		if (copy_from_user(pu,
			(void *)slave_info->power_setting_array.power_setting,
			sizeof(*pu) * size)) {
			pr_err("failed: copy_from_user");
			kfree(pu);
			return -EFAULT;
		}
	}

	/* Print power setting */
	for (i = 0; i < size; i++) {
		CDBG("UP seq_type %d seq_val %d config_val %ld delay %d",
			pu[i].seq_type, pu[i].seq_val,
			pu[i].config_val, pu[i].delay);
	}


	/* Fill power up setting and power up setting size */
	power_info->power_setting = pu;
	power_info->power_setting_size = size;

	return rc;
}

static int32_t msm_sensor_get_power_settings(void *setting,
	struct msm_camera_sensor_slave_info *slave_info,
	struct msm_camera_power_ctrl_t *power_info)
{
	int32_t rc = 0;

	rc = msm_sensor_get_power_up_settings(setting, slave_info, power_info);
	if (rc < 0) {
		pr_err("failed");
		return -EINVAL;
	}

	rc = msm_sensor_get_power_down_settings(setting, slave_info,
		power_info);
	if (rc < 0) {
		pr_err("failed");
		kfree(power_info->power_setting);
		return -EINVAL;
	}
	return rc;
}

static void msm_sensor_fill_sensor_info(struct msm_sensor_ctrl_t *s_ctrl,
	struct msm_sensor_info_t *sensor_info, char *entity_name)
{
	uint32_t i;

	if (!s_ctrl || !sensor_info) {
		pr_err("%s:failed\n", __func__);
		return;
	}

	strlcpy(sensor_info->sensor_name, s_ctrl->sensordata->sensor_name,
		MAX_SENSOR_NAME);

	sensor_info->session_id = s_ctrl->sensordata->sensor_info->session_id;

	s_ctrl->sensordata->sensor_info->subdev_id[SUB_MODULE_SENSOR] =
		s_ctrl->sensordata->sensor_info->session_id;
	for (i = 0; i < SUB_MODULE_MAX; i++) {
		sensor_info->subdev_id[i] =
			s_ctrl->sensordata->sensor_info->subdev_id[i];
		sensor_info->subdev_intf[i] =
			s_ctrl->sensordata->sensor_info->subdev_intf[i];
	}

	sensor_info->is_mount_angle_valid =
		s_ctrl->sensordata->sensor_info->is_mount_angle_valid;
	sensor_info->sensor_mount_angle =
		s_ctrl->sensordata->sensor_info->sensor_mount_angle;
	sensor_info->modes_supported =
		s_ctrl->sensordata->sensor_info->modes_supported;
	sensor_info->position =
		s_ctrl->sensordata->sensor_info->position;

	strlcpy(entity_name, s_ctrl->msm_sd.sd.entity.name, MAX_SENSOR_NAME);
}

/* static function definition */

//ASUS_BSP+++ CR_0 Randy_Change@asus.com.tw [2015/3/3] Modify Begin
struct msm_camera_i2c_read_config read_i2c_struct = {0x20,0x300b,MSM_CAMERA_I2C_WORD_DATA, 0};
//struct msm_camera_i2c_read_config read_otp = {0x20,0x7100,MSM_CAMERA_I2C_BYTE_DATA, 0};
static struct msm_camera_i2c_reg_array otp_reg_array[] = {
	{0x5002,0x0},// [3] disable OTP_DPC
		{0x3D88, 0x70},
		{0x3D89, 0x00},
		{0x3D8A, 0x70},
		{0x3D8B, MODULE_OTP_SIZE},
		{0x3D85, 0x06},
		{0x3D8C, 0x01},
		{0x3D8D, 0x00},
		{0x5002, 0x28},
		{0x0100, 0x01},
};

static  struct msm_camera_i2c_reg_setting otp_settings = {
  .reg_setting = otp_reg_array,
  .size = ARRAY_SIZE(otp_reg_array),
  .addr_type = MSM_CAMERA_I2C_WORD_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 0,
};
uint16_t read_i2c(struct msm_sensor_ctrl_t *s_ctrl, uint16_t addr, enum msm_camera_i2c_data_type data_type) {

	int32_t rc = 0;
	read_i2c_struct.reg_addr=addr;
	read_i2c_struct.data_type=data_type;
	read_i2c_struct.data=0xFF;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			read_i2c_struct.reg_addr,
			&read_i2c_struct.data, read_i2c_struct.data_type);



	if (rc< 0) {
		read_i2c_struct.data=0;
		pr_err("Randy read sensor data...fail = 0x%X\n", read_i2c_struct.data);
	}
//	else
//			pr_err("Randy read sensor data[0x%X] = 0x%X\n",read_i2c_struct.reg_addr, read_i2c_struct.data);

	return read_i2c_struct.data;
}


void PrintOutFormatOTP(char * out_otp_buf, char* in_i2c_buffer) {
	int i, j;
			memset(out_otp_buf, 0x0, MODULE_OTP_BUF_SIZE);
			for (j=0;j<MODULE_OTP_BULK;j++) {
				for (i=0;i<MODULE_OTP_BULK_SIZE;i++) {
					 char OTPBuff[6];
					sprintf(OTPBuff, "0x%02X ", 0xFF & in_i2c_buffer[i+j*MODULE_OTP_BULK_SIZE]);
					strcat(out_otp_buf, OTPBuff);		
					if ((i&0x7)==0x7) {
						strcat(out_otp_buf, "\n"); 	
					}		
				}
				strcat(out_otp_buf, "\n"); 	
			}
}

int read_front_otp(struct msm_sensor_ctrl_t *s_ctrl ) {
//  struct sensorb_cfg_data cfg;
 int   temp;
 int32_t rc = 0;
 //unsigned char bank_value[OV5670_FRONT_MODULE_OTP_SIZE];

 
	pr_err("Randy read sensor data...1\n");
	temp=read_i2c(s_ctrl, 0x300b, MSM_CAMERA_I2C_WORD_DATA);

	temp=read_i2c(s_ctrl, 0x5002, MSM_CAMERA_I2C_BYTE_DATA);
	otp_settings.reg_setting[0].reg_data=(0x00&0x08)|(temp&(~0x08));
	pr_err("Randy reading otp...Begin\n");
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(s_ctrl->sensor_i2c_client, &otp_settings);
	 if (rc < 0) {
		 pr_err("Randy write otp setting failed\n");
	 }

	msleep(1);

#if 0	
	pr_err("Randy Dump Writing OTP setting Begin\n");
	temp=read_i2c(s_ctrl, 0x3D80,MSM_CAMERA_I2C_BYTE_DATA);
	temp=read_i2c(s_ctrl, 0x3D81,MSM_CAMERA_I2C_BYTE_DATA);
	temp=read_i2c(s_ctrl, 0x3D8E,MSM_CAMERA_I2C_BYTE_DATA);
	temp=read_i2c(s_ctrl, 0x3D8F,MSM_CAMERA_I2C_BYTE_DATA);
	for (i=0;i<otp_settings.size;i++) {
		temp=read_i2c(s_ctrl, otp_settings.reg_setting[i].reg_addr,MSM_CAMERA_I2C_BYTE_DATA);
//		ALOGE("Randy read OTP settings[0x%X] = 0x%X",  otp_settings.reg_setting[i].reg_addr, temp);
	}
	pr_err("Randy Dump Writing OTP setting End\n");

	pr_err("Randy reading otp...Begin2\n");
	for (i=0;i<OV5670_FRONT_MODULE_OTP_SIZE;i++) {
		unsigned char OTPByte;
	//	unsigned char OTPBuff[6];
		OTPByte=read_i2c(s_ctrl, 0x7000+i,MSM_CAMERA_I2C_BYTE_DATA);
//		sprintf(OTPBuff, "0x%X ", 0xFF & OTPByte);
//		strcat(front_module_otp, OTPBuff);		
//		ALOGE("Randy read OTP[0x%X] = 0x%X", 0x7100+i, temp);
	bank_value[i]=OTPByte;

	}
{
//	memset(bank_value, 0, 72);
	snprintf(front_module_otp, sizeof(front_module_otp)
		, "0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n\n0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n\n0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n"
			, bank_value[0]&0xFF, bank_value[1]&0xFF, bank_value[2]&0xFF, bank_value[3]&0xFF, bank_value[4]&0xFF
			, bank_value[5]&0xFF, bank_value[6]&0xFF, bank_value[7]&0xFF, bank_value[8]&0xFF, bank_value[9]&0xFF
			, bank_value[10]&0xFF, bank_value[11]&0xFF, bank_value[12]&0xFF, bank_value[13]&0xFF, bank_value[14]&0xFF
			, bank_value[15]&0xFF, bank_value[16]&0xFF, bank_value[17]&0xFF, bank_value[18]&0xFF, bank_value[19]&0xFF
			, bank_value[20]&0xFF, bank_value[21]&0xFF, bank_value[22]&0xFF, bank_value[23]&0xFF
			, bank_value[24]&0xFF, bank_value[25]&0xFF, bank_value[26]&0xFF, bank_value[27]&0xFF, bank_value[28]&0xFF
			, bank_value[29]&0xFF, bank_value[30]&0xFF, bank_value[31]&0xFF, bank_value[32]&0xFF, bank_value[33]&0xFF
			, bank_value[34]&0xFF, bank_value[35]&0xFF, bank_value[36]&0xFF, bank_value[37]&0xFF, bank_value[38]&0xFF
			, bank_value[39]&0xFF, bank_value[40]&0xFF, bank_value[41]&0xFF, bank_value[42]&0xFF, bank_value[43]&0xFF
			, bank_value[44]&0xFF, bank_value[45]&0xFF, bank_value[46]&0xFF, bank_value[47]&0xFF
			, bank_value[48]&0xFF, bank_value[49]&0xFF, bank_value[50]&0xFF, bank_value[51]&0xFF, bank_value[52]&0xFF
			, bank_value[53]&0xFF, bank_value[54]&0xFF, bank_value[55]&0xFF, bank_value[56]&0xFF, bank_value[57]&0xFF
			, bank_value[58]&0xFF, bank_value[59]&0xFF, bank_value[60]&0xFF, bank_value[61]&0xFF, bank_value[62]&0xFF
			, bank_value[63]&0xFF, bank_value[64]&0xFF, bank_value[65]&0xFF, bank_value[66]&0xFF, bank_value[67]&0xFF
			, bank_value[68]&0xFF, bank_value[69]&0xFF, bank_value[70]&0xFF, bank_value[71]&0xFF);

}
#endif 
{	
	int i;
		char OTPByte[MODULE_OTP_SIZE];
		for (i=0;i<MODULE_OTP_SIZE;i++) OTPByte[i]=read_i2c(s_ctrl, 0x7010+i, MSM_CAMERA_I2C_BYTE_DATA);
		PrintOutFormatOTP(front_module_otp, OTPByte);
}
	pr_err("Randy all OTP=%s\n", front_module_otp);

	pr_err("Randy reading otp...End2\n");
	return 0;	
}
//ASUS_BSP--- CR_0 Randy_Change@asus.com.tw [2015/3/3] Modify End
int32_t msm_sensor_driver_is_special_support(
	struct msm_sensor_ctrl_t *s_ctrl,
	char* sensor_name)
{
	int32_t rc = FALSE;
	int32_t i = 0;
	struct msm_camera_sensor_board_info *sensordata = s_ctrl->sensordata;
	for (i = 0; i < sensordata->special_support_size; i++) {
		if (!strcmp(sensordata->special_support_sensors[i],
			sensor_name)) {
			rc = TRUE;
			break ;
		}
	}
	return rc;
}

int32_t msm_sensor_driver_probe(void *setting,
	struct msm_sensor_info_t *probed_info, char *entity_name)
{
	int32_t                              rc = 0;
	struct msm_sensor_ctrl_t            *s_ctrl = NULL;
	struct msm_camera_cci_client        *cci_client = NULL;
	struct msm_camera_sensor_slave_info *slave_info = NULL;
	struct msm_camera_slave_info        *camera_info = NULL;

	unsigned long                        mount_pos = 0;
	uint32_t                             is_yuv;

	/* Validate input parameters */
	if (!setting) {
		pr_err("failed: slave_info %pK", setting);
		return -EINVAL;
	}

	/* Allocate memory for slave info */
	slave_info = kzalloc(sizeof(*slave_info), GFP_KERNEL);
	if (!slave_info) {
		pr_err("failed: no memory slave_info %pK", slave_info);
		return -ENOMEM;
	}
#ifdef CONFIG_COMPAT
	if (is_compat_task()) {
		struct msm_camera_sensor_slave_info32 setting32;
		if (copy_from_user((void *)&setting32, setting,
			sizeof(setting32))) {
				pr_err("failed: copy_from_user");
				rc = -EFAULT;
				goto free_slave_info;
			}

		strlcpy(slave_info->actuator_name, setting32.actuator_name,
			sizeof(slave_info->actuator_name));

		strlcpy(slave_info->eeprom_name, setting32.eeprom_name,
			sizeof(slave_info->eeprom_name));

		strlcpy(slave_info->sensor_name, setting32.sensor_name,
			sizeof(slave_info->sensor_name));

		strlcpy(slave_info->ois_name, setting32.ois_name,
			sizeof(slave_info->ois_name));

		strlcpy(slave_info->flash_name, setting32.flash_name,
			sizeof(slave_info->flash_name));

		slave_info->addr_type = setting32.addr_type;
		slave_info->camera_id = setting32.camera_id;

		slave_info->i2c_freq_mode = setting32.i2c_freq_mode;
		slave_info->sensor_id_info = setting32.sensor_id_info;

		slave_info->slave_addr = setting32.slave_addr;
		slave_info->power_setting_array.size =
			setting32.power_setting_array.size;
		slave_info->power_setting_array.size_down =
			setting32.power_setting_array.size_down;
		slave_info->power_setting_array.size_down =
			setting32.power_setting_array.size_down;
		slave_info->power_setting_array.power_setting =
			compat_ptr(setting32.power_setting_array.power_setting);
		slave_info->power_setting_array.power_down_setting =
			compat_ptr(setting32.
				power_setting_array.power_down_setting);
		slave_info->is_init_params_valid =
			setting32.is_init_params_valid;
		slave_info->sensor_init_params = setting32.sensor_init_params;
		slave_info->is_flash_supported = setting32.is_flash_supported;
		slave_info->output_format = setting32.output_format;
	} else
#endif
	{
		if (copy_from_user(slave_info,
					(void *)setting, sizeof(*slave_info))) {
			pr_err("failed: copy_from_user");
			rc = -EFAULT;
			goto free_slave_info;
		}
	}

	/* Print slave info */
	CDBG("camera id %d", slave_info->camera_id);
	CDBG("slave_addr 0x%x", slave_info->slave_addr);
	CDBG("addr_type %d", slave_info->addr_type);
	CDBG("sensor_id_reg_addr 0x%x",
		slave_info->sensor_id_info.sensor_id_reg_addr);
	CDBG("sensor_id 0x%x", slave_info->sensor_id_info.sensor_id);
	CDBG("size %d", slave_info->power_setting_array.size);
	CDBG("size down %d", slave_info->power_setting_array.size_down);

	if (slave_info->is_init_params_valid) {
		CDBG("position %d",
			slave_info->sensor_init_params.position);
		CDBG("mount %d",
			slave_info->sensor_init_params.sensor_mount_angle);
	}

//ASUS_BSP Stimber_Hsueh "Implement for ATD interface" +++
	if(CAMERA_0 == slave_info->camera_id){
		g_camera_id = CAMERA_0;
		create_rear_status_proc_file();
		create_rear_resolution_proc_file();
		create_rear_module_proc_file();
	}else if(CAMERA_1 == slave_info->camera_id){
		g_camera_id = CAMERA_1;
		create_front_status_proc_file();
		create_front_resolution_proc_file();
		create_front_module_proc_file();
	}
//ASUS_BSP Stimber_Hsueh "Implement for ATD interface" ---

	/* Validate camera id */
	if (slave_info->camera_id >= MAX_CAMERAS) {
		pr_err("failed: invalid camera id %d max %d",
			slave_info->camera_id, MAX_CAMERAS);
		rc = -EINVAL;
		goto free_slave_info;
	}

	/* Extract s_ctrl from camera id */
	s_ctrl = g_sctrl[slave_info->camera_id];
	if (!s_ctrl) {
		pr_err("failed: s_ctrl %pK for camera_id %d", s_ctrl,
			slave_info->camera_id);
		rc = -EINVAL;
		goto free_slave_info;
	}

	CDBG("s_ctrl[%d] %pK", slave_info->camera_id, s_ctrl);

	if (s_ctrl->is_probe_succeed == 1) {
		/*
		 * Different sensor on this camera slot has been connected
		 * and probe already succeeded for that sensor. Ignore this
		 * probe
		 */
		if (slave_info->sensor_id_info.sensor_id ==
			s_ctrl->sensordata->cam_slave_info->
				sensor_id_info.sensor_id) {
			pr_err("slot%d: sensor id%d already probed\n",
				slave_info->camera_id,
				s_ctrl->sensordata->cam_slave_info->
					sensor_id_info.sensor_id);
			msm_sensor_fill_sensor_info(s_ctrl,
				probed_info, entity_name);
		} else
			pr_err("slot %d has some other sensor\n",
				slave_info->camera_id);

		rc = 0;
		goto free_slave_info;
	}

	if (s_ctrl->sensordata->special_support_size > 0) {
		if (!msm_sensor_driver_is_special_support(s_ctrl,
			slave_info->sensor_name)) {
			pr_err("%s:%s is not support on this board\n",
				__func__, slave_info->sensor_name);
			rc = 0;
			goto free_slave_info;
		}
	}

	rc = msm_sensor_get_power_settings(setting, slave_info,
		&s_ctrl->sensordata->power_info);
	if (rc < 0) {
		pr_err("failed");
		goto free_slave_info;
	}


	camera_info = kzalloc(sizeof(struct msm_camera_slave_info), GFP_KERNEL);
	if (!camera_info) {
		pr_err("failed: no memory slave_info %p", camera_info);
		goto free_power_settings;
	}

	s_ctrl->sensordata->slave_info = camera_info;

	/* Fill sensor slave info */
	camera_info->sensor_slave_addr = slave_info->slave_addr;
	camera_info->sensor_id_reg_addr =
		slave_info->sensor_id_info.sensor_id_reg_addr;
	camera_info->sensor_id = slave_info->sensor_id_info.sensor_id;

	/* Fill CCI master, slave address and CCI default params */
	if (!s_ctrl->sensor_i2c_client) {
		pr_err("failed: sensor_i2c_client %pK",
			s_ctrl->sensor_i2c_client);
		rc = -EINVAL;
		goto free_camera_info;
	}
	/* Fill sensor address type */
	s_ctrl->sensor_i2c_client->addr_type = slave_info->addr_type;
	if (s_ctrl->sensor_i2c_client->client)
		s_ctrl->sensor_i2c_client->client->addr =
			camera_info->sensor_slave_addr;

	cci_client = s_ctrl->sensor_i2c_client->cci_client;
	if (!cci_client) {
		pr_err("failed: cci_client %pK", cci_client);
		goto free_camera_info;
	}
	cci_client->cci_i2c_master = s_ctrl->cci_i2c_master;
	cci_client->sid = slave_info->slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = slave_info->i2c_freq_mode;

	/* Parse and fill vreg params for powerup settings */
	rc = msm_camera_fill_vreg_params(
		s_ctrl->sensordata->power_info.cam_vreg,
		s_ctrl->sensordata->power_info.num_vreg,
		s_ctrl->sensordata->power_info.power_setting,
		s_ctrl->sensordata->power_info.power_setting_size);
	if (rc < 0) {
		pr_err("failed: msm_camera_get_dt_power_setting_data rc %d",
			rc);
		goto free_camera_info;
	}

	/* Parse and fill vreg params for powerdown settings*/
	rc = msm_camera_fill_vreg_params(
		s_ctrl->sensordata->power_info.cam_vreg,
		s_ctrl->sensordata->power_info.num_vreg,
		s_ctrl->sensordata->power_info.power_down_setting,
		s_ctrl->sensordata->power_info.power_down_setting_size);
	if (rc < 0) {
		pr_err("failed: msm_camera_fill_vreg_params for PDOWN rc %d",
			rc);
		goto free_camera_info;
	}

	/* Update sensor, actuator and eeprom name in
	*  sensor control structure */
	s_ctrl->sensordata->sensor_name = slave_info->sensor_name;
	s_ctrl->sensordata->eeprom_name = slave_info->eeprom_name;
	s_ctrl->sensordata->actuator_name = slave_info->actuator_name;
	s_ctrl->sensordata->ois_name = slave_info->ois_name;
	/*
	 * Update eeporm subdevice Id by input eeprom name
	 */
	rc = msm_sensor_fill_eeprom_subdevid_by_name(s_ctrl);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto free_camera_info;
	}
	/*
	 * Update actuator subdevice Id by input actuator name
	 */
	rc = msm_sensor_fill_actuator_subdevid_by_name(s_ctrl);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto free_camera_info;
	}

	rc = msm_sensor_fill_ois_subdevid_by_name(s_ctrl);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto free_camera_info;
	}

	//ASUS_BSP bill_chen +++
	if(CAMERA_1 == slave_info->camera_id) {
		pr_err("%s: g_camera_resolution = %s, front_sensor_id = %x\n", __func__, g_rear_resolution, slave_info->sensor_id_info.sensor_id);
		if(strcmp(g_rear_resolution, "13M") == 0 &&
			slave_info->sensor_id_info.sensor_id == 0x2155){
			//g_vga_status[g_front_camera_ref_cnt] = 0;

			pr_err("13M is not match 2M");
			//goto free_camera_info;
		}else if(strcmp(g_rear_resolution, "8M") == 0 &&
			slave_info->sensor_id_info.sensor_id == 0x5670){
			//g_vga_status[g_front_camera_ref_cnt] = 0;

			pr_err("8M is not match 5M");
			//goto free_camera_info;
		}
	}
	//ASUS_BSP bill_chen ---

	/* Power up and probe sensor */
	rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
	if (rc < 0) {

		//ASUS_BSP Stimber_Hsueh "Implement for ATD interface" +++
		if(CAMERA_0 == slave_info->camera_id){
			g_camera_status[g_rear_camera_ref_cnt] = 0;
		}else if(CAMERA_1 == slave_info->camera_id){
			g_vga_status[g_front_camera_ref_cnt] = 0;
		}
		//ASUS_BSP Stimber_Hsueh "Implement for ATD interface" ---

		pr_err("%s power up failed", slave_info->sensor_name);
		goto free_camera_info;
	}

	pr_err("%s probe succeeded", slave_info->sensor_name);

	//ASUS_BSP Stimber_Hsueh +++
	if(strcmp(slave_info->sensor_name, "t4k37") == 0){
		g_rear_resolution = "13M";
		g_rear_module = "T4K37";

		tsb_otp_read();
		create_rear_otp_proc_file();
	}else if(strcmp(slave_info->sensor_name, "t4k35") == 0){
		g_rear_resolution = "8M";
		g_rear_module = "T4K35";

		tsb_otp_read();
		create_rear_otp_proc_file();
	}else if(strcmp(slave_info->sensor_name, "imx219_asus") == 0){
		g_rear_resolution = "8M";
		g_rear_module = "IMX219";
		imx219_otp_read();
		create_rear_otp_proc_file();
	}else if(strcmp(slave_info->sensor_name, "mn34150") == 0){
		g_rear_resolution = "13M";
		g_rear_module = "MN34150";
		mn34150_otp_read();
		create_rear_otp_proc_file();
	}else if(strcmp(slave_info->sensor_name, "ov5670_q5v41b") == 0){
		g_front_resolution = "5M";
		g_front_module = "OV5670";
		read_front_otp(s_ctrl);
		create_front_otp_proc_file();
	}else if(strcmp(slave_info->sensor_name, "gc2155") == 0){
		g_front_resolution = "2M";
		g_front_module = "GC2155";
		snprintf(front_module_otp, sizeof(front_module_otp),"0");
		create_front_otp_proc_file();
		create_gc2155_obvalue_proc_file();
	}
	//ASUS_BSP Stimber_Hsueh ---

	/*
	 * Update the subdevice id of flash-src based on availability in kernel.
	 */
	if (slave_info->is_flash_supported == 0) {
		s_ctrl->sensordata->sensor_info->
			subdev_id[SUB_MODULE_LED_FLASH] = -1;
	}

	/*
	 * Create /dev/videoX node, comment for now until dummy /dev/videoX
	 * node is created and used by HAL
	 */

	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		rc = msm_sensor_driver_create_v4l_subdev(s_ctrl);
	else
		rc = msm_sensor_driver_create_i2c_v4l_subdev(s_ctrl);
	if (rc < 0) {
		pr_err("failed: camera creat v4l2 rc %d", rc);
		goto camera_power_down;
	}

	/* Power down */
	s_ctrl->func_tbl->sensor_power_down(s_ctrl);

	rc = msm_sensor_fill_slave_info_init_params(
		slave_info,
		s_ctrl->sensordata->sensor_info);
	if (rc < 0) {
		pr_err("%s Fill slave info failed", slave_info->sensor_name);
		goto free_camera_info;
	}
	rc = msm_sensor_validate_slave_info(s_ctrl->sensordata->sensor_info);
	if (rc < 0) {
		pr_err("%s Validate slave info failed",
			slave_info->sensor_name);
		goto free_camera_info;
	}
	/* Update sensor mount angle and position in media entity flag */
	is_yuv = (slave_info->output_format == MSM_SENSOR_YCBCR) ? 1 : 0;
	mount_pos = is_yuv << 25 |
		(s_ctrl->sensordata->sensor_info->position << 16) |
		((s_ctrl->sensordata->
		sensor_info->sensor_mount_angle / 90) << 8);

	s_ctrl->msm_sd.sd.entity.flags = mount_pos | MEDIA_ENT_FL_DEFAULT;

	/*Save sensor info*/
	s_ctrl->sensordata->cam_slave_info = slave_info;

	msm_sensor_fill_sensor_info(s_ctrl, probed_info, entity_name);

	/*
	 * Set probe succeeded flag to 1 so that no other camera shall
	 * probed on this slot
	 */
	s_ctrl->is_probe_succeed = 1;
	return rc;

camera_power_down:
	s_ctrl->func_tbl->sensor_power_down(s_ctrl);
free_camera_info:
	kfree(camera_info);
free_power_settings:
	kfree(s_ctrl->sensordata->power_info.power_setting);
	kfree(s_ctrl->sensordata->power_info.power_down_setting);
free_slave_info:
	kfree(slave_info);
	return rc;
}

static int32_t msm_sensor_driver_get_gpio_data(
	struct msm_camera_sensor_board_info *sensordata,
	struct device_node *of_node)
{
	int32_t                      rc = 0, i = 0;
	struct msm_camera_gpio_conf *gconf = NULL;
	uint16_t                    *gpio_array = NULL;
	uint16_t                     gpio_array_size = 0;

	/* Validate input paramters */
	if (!sensordata || !of_node) {
		pr_err("failed: invalid params sensordata %pK of_node %pK",
			sensordata, of_node);
		return -EINVAL;
	}

	sensordata->power_info.gpio_conf = kzalloc(
			sizeof(struct msm_camera_gpio_conf), GFP_KERNEL);
	if (!sensordata->power_info.gpio_conf) {
		pr_err("failed");
		return -ENOMEM;
	}
	gconf = sensordata->power_info.gpio_conf;

	gpio_array_size = of_gpio_count(of_node);
	CDBG("gpio count %d", gpio_array_size);
	if (!gpio_array_size)
		return 0;

	gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size, GFP_KERNEL);
	if (!gpio_array) {
		pr_err("failed");
		goto FREE_GPIO_CONF;
	}
	for (i = 0; i < gpio_array_size; i++) {
		gpio_array[i] = of_get_gpio(of_node, i);
		CDBG("gpio_array[%d] = %d", i, gpio_array[i]);
	}

	rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf, gpio_array,
		gpio_array_size);
	if (rc < 0) {
		pr_err("failed");
		goto FREE_GPIO_CONF;
	}

	rc = msm_camera_init_gpio_pin_tbl(of_node, gconf, gpio_array,
		gpio_array_size);
	if (rc < 0) {
		pr_err("failed");
		goto FREE_GPIO_REQ_TBL;
	}

	kfree(gpio_array);
	return rc;

FREE_GPIO_REQ_TBL:
	kfree(sensordata->power_info.gpio_conf->cam_gpio_req_tbl);
FREE_GPIO_CONF:
	kfree(sensordata->power_info.gpio_conf);
	kfree(gpio_array);
	return rc;
}

static int32_t msm_sensor_driver_get_dt_data(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t                              rc = 0;
	struct msm_camera_sensor_board_info *sensordata = NULL;
	struct device_node                  *of_node = s_ctrl->of_node;
	uint32_t                             cell_id;
	int32_t                              i;

	s_ctrl->sensordata = kzalloc(sizeof(*sensordata), GFP_KERNEL);
	if (!s_ctrl->sensordata) {
		pr_err("failed: no memory");
		return -ENOMEM;
	}

	sensordata = s_ctrl->sensordata;

	/*
	 * Read cell index - this cell index will be the camera slot where
	 * this camera will be mounted
	 */
	rc = of_property_read_u32(of_node, "cell-index", &cell_id);
	if (rc < 0) {
		pr_err("failed: cell-index rc %d", rc);
		goto FREE_SENSOR_DATA;
	}
	s_ctrl->id = cell_id;

	/* Validate cell_id */
	if (cell_id >= MAX_CAMERAS) {
		pr_err("failed: invalid cell_id %d", cell_id);
		rc = -EINVAL;
		goto FREE_SENSOR_DATA;
	}

	/* Check whether g_sctrl is already filled for this cell_id */
	if (g_sctrl[cell_id]) {
		pr_err("failed: sctrl already filled for cell_id %d", cell_id);
		rc = -EINVAL;
		goto FREE_SENSOR_DATA;
	}

	sensordata->special_support_size =
		of_property_count_strings(of_node, "qcom,special-support-sensors");

	if (sensordata->special_support_size < 0)
		sensordata->special_support_size = 0;

	if (sensordata->special_support_size > MAX_SPECIAL_SUPPORT_SIZE) {
		pr_err("%s:support_size exceed max support size\n",__func__);
		sensordata->special_support_size = MAX_SPECIAL_SUPPORT_SIZE;
	}

	if (sensordata->special_support_size) {
		for( i = 0; i < sensordata->special_support_size; i++) {
			rc = of_property_read_string_index(of_node,
				"qcom,special-support-sensors", i,
				&(sensordata->special_support_sensors[i]));
			if(rc < 0 ) {
				/* if read sensor support names failed,
				*   set support all sensors, break;
				*/
				sensordata->special_support_size = 0;
				break ;
			}
			CDBG("%s special_support_sensors[%d] = %s\n", __func__,
				i, sensordata->special_support_sensors[i]);
		}
	}

	/* Read subdev info */
	rc = msm_sensor_get_sub_module_index(of_node, &sensordata->sensor_info);
	if (rc < 0) {
		pr_err("failed");
		goto FREE_SENSOR_DATA;
	}

	/* Read vreg information */
	rc = msm_camera_get_dt_vreg_data(of_node,
		&sensordata->power_info.cam_vreg,
		&sensordata->power_info.num_vreg);
	if (rc < 0) {
		pr_err("failed: msm_camera_get_dt_vreg_data rc %d", rc);
		goto FREE_SUB_MODULE_DATA;
	}

	/* Read gpio information */
	rc = msm_sensor_driver_get_gpio_data(sensordata, of_node);
	if (rc < 0) {
		pr_err("failed: msm_sensor_driver_get_gpio_data rc %d", rc);
		goto FREE_VREG_DATA;
	}

	/* Get CCI master */
	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&s_ctrl->cci_i2c_master);
	CDBG("qcom,cci-master %d, rc %d", s_ctrl->cci_i2c_master, rc);
	if (rc < 0) {
		/* Set default master 0 */
		s_ctrl->cci_i2c_master = MASTER_0;
		rc = 0;
	}

	/* Get mount angle */
	if (0 > of_property_read_u32(of_node, "qcom,mount-angle",
		&sensordata->sensor_info->sensor_mount_angle)) {
		/* Invalidate mount angle flag */
		sensordata->sensor_info->is_mount_angle_valid = 0;
		sensordata->sensor_info->sensor_mount_angle = 0;
	} else {
		sensordata->sensor_info->is_mount_angle_valid = 1;
	}
	CDBG("%s qcom,mount-angle %d\n", __func__,
		sensordata->sensor_info->sensor_mount_angle);
	if (0 > of_property_read_u32(of_node, "qcom,sensor-position",
		&sensordata->sensor_info->position)) {
		CDBG("%s:%d Invalid sensor position\n", __func__, __LINE__);
		sensordata->sensor_info->position = INVALID_CAMERA_B;
	}
	if (0 > of_property_read_u32(of_node, "qcom,sensor-mode",
		&sensordata->sensor_info->modes_supported)) {
		CDBG("%s:%d Invalid sensor mode supported\n",
			__func__, __LINE__);
		sensordata->sensor_info->modes_supported = CAMERA_MODE_INVALID;
	}
	/* Get vdd-cx regulator */
	/*Optional property, don't return error if absent */
	of_property_read_string(of_node, "qcom,vdd-cx-name",
		&sensordata->misc_regulator);
	CDBG("qcom,misc_regulator %s", sensordata->misc_regulator);

	s_ctrl->set_mclk_23880000 = of_property_read_bool(of_node,
						"qcom,mclk-23880000");

	CDBG("%s qcom,mclk-23880000 = %d\n", __func__,
		s_ctrl->set_mclk_23880000);

	return rc;

FREE_VREG_DATA:
	kfree(sensordata->power_info.cam_vreg);
FREE_SUB_MODULE_DATA:
	kfree(sensordata->sensor_info);
FREE_SENSOR_DATA:
	kfree(sensordata);
	return rc;
}

static int32_t msm_sensor_driver_parse(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t                   rc = 0;

	CDBG("Enter");
	/* Validate input parameters */


	/* Allocate memory for sensor_i2c_client */
	s_ctrl->sensor_i2c_client = kzalloc(sizeof(*s_ctrl->sensor_i2c_client),
		GFP_KERNEL);
	if (!s_ctrl->sensor_i2c_client) {
		pr_err("failed: no memory sensor_i2c_client %pK",
			s_ctrl->sensor_i2c_client);
		return -ENOMEM;
	}

	/* Allocate memory for mutex */
	s_ctrl->msm_sensor_mutex = kzalloc(sizeof(*s_ctrl->msm_sensor_mutex),
		GFP_KERNEL);
	if (!s_ctrl->msm_sensor_mutex) {
		pr_err("failed: no memory msm_sensor_mutex %pK",
			s_ctrl->msm_sensor_mutex);
		goto FREE_SENSOR_I2C_CLIENT;
	}

	/* Parse dt information and store in sensor control structure */
	rc = msm_sensor_driver_get_dt_data(s_ctrl);
	if (rc < 0) {
		pr_err("failed: rc %d", rc);
		goto FREE_MUTEX;
	}

	/* Initialize mutex */
	mutex_init(s_ctrl->msm_sensor_mutex);

	/* Initilize v4l2 subdev info */
	s_ctrl->sensor_v4l2_subdev_info = msm_sensor_driver_subdev_info;
	s_ctrl->sensor_v4l2_subdev_info_size =
		ARRAY_SIZE(msm_sensor_driver_subdev_info);

	/* Initialize default parameters */
	rc = msm_sensor_init_default_params(s_ctrl);
	if (rc < 0) {
		pr_err("failed: msm_sensor_init_default_params rc %d", rc);
		goto FREE_DT_DATA;
	}

	/* Store sensor control structure in static database */
	g_sctrl[s_ctrl->id] = s_ctrl;
	pr_err("g_sctrl[%d] %pK", s_ctrl->id, g_sctrl[s_ctrl->id]);

	return rc;

FREE_DT_DATA:
	kfree(s_ctrl->sensordata->power_info.gpio_conf->gpio_num_info);
	kfree(s_ctrl->sensordata->power_info.gpio_conf->cam_gpio_req_tbl);
	kfree(s_ctrl->sensordata->power_info.gpio_conf);
	kfree(s_ctrl->sensordata->power_info.cam_vreg);
	kfree(s_ctrl->sensordata);
FREE_MUTEX:
	kfree(s_ctrl->msm_sensor_mutex);
FREE_SENSOR_I2C_CLIENT:
	kfree(s_ctrl->sensor_i2c_client);
	return rc;
}

static int32_t msm_sensor_driver_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl = NULL;

	/* Create sensor control structure */
	s_ctrl = kzalloc(sizeof(*s_ctrl), GFP_KERNEL);
	if (!s_ctrl) {
		pr_err("failed: no memory s_ctrl %pK", s_ctrl);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, s_ctrl);

	/* Initialize sensor device type */
	s_ctrl->sensor_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	s_ctrl->of_node = pdev->dev.of_node;

	rc = msm_sensor_driver_parse(s_ctrl);
	if (rc < 0) {
		pr_err("failed: msm_sensor_driver_parse rc %d", rc);
		goto FREE_S_CTRL;
	}

	/* Fill platform device */
	pdev->id = s_ctrl->id;
	s_ctrl->pdev = pdev;

	/* Fill device in power info */
	s_ctrl->sensordata->power_info.dev = &pdev->dev;
	return rc;
FREE_S_CTRL:
	kfree(s_ctrl);
	return rc;
}

static int32_t msm_sensor_driver_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int32_t rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl;

	CDBG("\n\nEnter: msm_sensor_driver_i2c_probe");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s %s i2c_check_functionality failed\n",
			__func__, client->name);
		rc = -EFAULT;
		return rc;
	}

	/* Create sensor control structure */
	s_ctrl = kzalloc(sizeof(*s_ctrl), GFP_KERNEL);
	if (!s_ctrl) {
		pr_err("failed: no memory s_ctrl %pK", s_ctrl);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, s_ctrl);

	/* Initialize sensor device type */
	s_ctrl->sensor_device_type = MSM_CAMERA_I2C_DEVICE;
	s_ctrl->of_node = client->dev.of_node;

	rc = msm_sensor_driver_parse(s_ctrl);
	if (rc < 0) {
		pr_err("failed: msm_sensor_driver_parse rc %d", rc);
		goto FREE_S_CTRL;
	}

	if (s_ctrl->sensor_i2c_client != NULL) {
		s_ctrl->sensor_i2c_client->client = client;
		s_ctrl->sensordata->power_info.dev = &client->dev;

	}

	return rc;
FREE_S_CTRL:
	kfree(s_ctrl);
	return rc;
}

static int msm_sensor_driver_i2c_remove(struct i2c_client *client)
{
	struct msm_sensor_ctrl_t  *s_ctrl = i2c_get_clientdata(client);

	pr_err("%s: sensor FREE\n", __func__);

	if (!s_ctrl) {
		pr_err("%s: sensor device is NULL\n", __func__);
		return 0;
	}

	g_sctrl[s_ctrl->id] = NULL;
	msm_sensor_free_sensor_data(s_ctrl);
	kfree(s_ctrl->msm_sensor_mutex);
	kfree(s_ctrl->sensor_i2c_client);
	kfree(s_ctrl);

	return 0;
}

static const struct i2c_device_id i2c_id[] = {
	{SENSOR_DRIVER_I2C, (kernel_ulong_t)NULL},
	{ }
};

static struct i2c_driver msm_sensor_driver_i2c = {
	.id_table = i2c_id,
	.probe  = msm_sensor_driver_i2c_probe,
	.remove = msm_sensor_driver_i2c_remove,
	.driver = {
		.name = SENSOR_DRIVER_I2C,
	},
};

static int __init msm_sensor_driver_init(void)
{
	int32_t rc = 0;

	CDBG("Enter");
	rc = platform_driver_probe(&msm_sensor_platform_driver,
		msm_sensor_driver_platform_probe);
	if (!rc) {
		CDBG("probe success");
		return rc;
	} else {
		CDBG("probe i2c");
		rc = i2c_add_driver(&msm_sensor_driver_i2c);
	}

	return rc;
}


static void __exit msm_sensor_driver_exit(void)
{
	CDBG("Enter");
	platform_driver_unregister(&msm_sensor_platform_driver);
	i2c_del_driver(&msm_sensor_driver_i2c);
	return;
}

//ASUS_BSP Stimber_Hsueh "Implement for ATD interface" +++
#define	STATUS_REAR_PROC_FILE	"driver/camera_status"
#define	STATUS_FRONT_PROC_FILE	"driver/vga_status"

#define	RESOLUTION_REAR_PROC_FILE	"driver/GetRearCameraResolution"
#define	RESOLUTION_FRONT_PROC_FILE	"driver/GetFrontCameraResolution"


#define	MODULE_REAR_PROC_FILE	"driver/RearModule"
#define	MODULE_FRONT_PROC_FILE	"driver/FrontModule"


static struct proc_dir_entry *status_proc_file;

static int rear_status_proc_read(struct seq_file *buf, void *v)
{
	unsigned char status = 0;
	int i=0;
	for(i=0; i<=g_rear_camera_ref_cnt; i++){
		pr_info("Stimber proc read=%d\n",g_camera_status[i]);
		status |= g_camera_status[i];
	}
	
   	seq_printf(buf, "%d\n", status);				
    return 0;
}

static int front_status_proc_read(struct seq_file *buf, void *v)
{
	unsigned char status = 0;
	int i=0;
	for(i=0; i<=g_front_camera_ref_cnt; i++){
		pr_info("Stimber front proc read=%d\n",g_vga_status[i]);
		status |= g_vga_status[i];
	}

	seq_printf(buf, "%d\n", status);
    return 0;
}

static int rear_status_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rear_status_proc_read, NULL);
}

static int front_status_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, front_status_proc_read, NULL);
}

static const struct file_operations rear_status_fops = {
	.owner = THIS_MODULE,
	.open = rear_status_proc_open,
	.read = seq_read,
	//.write = status_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations front_status_fops = {
	.owner = THIS_MODULE,
	.open = front_status_proc_open,
	.read = seq_read,
	//.write = status_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_rear_status_proc_file(void)
{
    if(!g_camera_status_created) {   
        status_proc_file = proc_create(STATUS_REAR_PROC_FILE, 0666, NULL, &rear_status_fops);
		if(status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
			g_camera_status_created = 1;
	    } else {
			pr_err("%s failed!\n", __func__);
			g_camera_status_created = 0;
	    }  
    } else {  
        pr_info("File Exist!\n");  
    }  
	g_rear_camera_ref_cnt++;
}

static void create_front_status_proc_file(void)
{
	if(!g_front_camera_status_created) {
		status_proc_file = proc_create(STATUS_FRONT_PROC_FILE, 0666, NULL, &front_status_fops);
	
		if(status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
			g_front_camera_status_created = 1;
	    } else {
			pr_err("%s failed!\n", __func__);
			g_front_camera_status_created = 0;
	    }
	} else {
        pr_info("%s: File Exist!\n", __func__);
    }
	g_front_camera_ref_cnt++;
}

static void remove_proc_file(void)
{
    extern struct proc_dir_entry proc_root;
    pr_info("remove_proc_file\n");	
    remove_proc_entry(STATUS_REAR_PROC_FILE, &proc_root);
	remove_proc_entry(STATUS_FRONT_PROC_FILE, &proc_root);
	remove_proc_entry(RESOLUTION_REAR_PROC_FILE, &proc_root);
	remove_proc_entry(RESOLUTION_FRONT_PROC_FILE, &proc_root);
	remove_proc_entry(MODULE_REAR_PROC_FILE, &proc_root);
	remove_proc_entry(MODULE_FRONT_PROC_FILE, &proc_root);
	
	memset(g_camera_status, 0, sizeof(g_camera_status));
	g_rear_camera_ref_cnt = -1;
	g_front_camera_ref_cnt = -1;
	g_camera_status_created = 0;
	g_front_camera_status_created = 0;

	g_rear_resolution_created = 0;
	g_front_resolution_created = 0;
	g_rear_module_created = 0;
	g_front_module_created = 0;
}
//ASUS_BSP Stimber_Hsueh "Implement for ATD interface" ---

//ASUS_BSP Stimber_Hsueh +++
static int rear_resolution_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%s\n", g_rear_resolution);
				
    return 0;
}

static int rear_resolution_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rear_resolution_proc_read, NULL);
}

static const struct file_operations rear_resolution_fops = {
	.owner = THIS_MODULE,
	.open = rear_resolution_proc_open,
	.read = seq_read,
	//.write = status_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_rear_resolution_proc_file(void)
{	
	if(!g_rear_resolution_created) {   
        status_proc_file = proc_create(RESOLUTION_REAR_PROC_FILE, 0666, NULL, &rear_resolution_fops);
		if(status_proc_file) {
			CDBG("Stimber: %s sucessed!\n", __func__);
			g_rear_resolution_created = 1;
	    } else {
			pr_err("Stimber: %s failed!\n", __func__);
			g_rear_resolution_created = 0;
	    }  
    } else {  
        pr_info("File Exist!\n");  
    }  
}
//ASUS_BSP Stimber_Hsueh ---

//ASUS_BSP Stimber_Hsueh +++
static int front_resolution_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%s\n", g_front_resolution);
				
    return 0;
}

static int front_resolution_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, front_resolution_proc_read, NULL);
}

static const struct file_operations front_resolution_fops = {
	.owner = THIS_MODULE,
	.open = front_resolution_proc_open,
	.read = seq_read,
	//.write = status_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_front_resolution_proc_file(void)
{	
	if(!g_front_resolution_created) {   
        status_proc_file = proc_create(RESOLUTION_FRONT_PROC_FILE, 0666, NULL, &front_resolution_fops);
		if(status_proc_file) {
			CDBG("Stimber: %s sucessed!\n", __func__);
			g_front_resolution_created = 1;
	    } else {
			pr_err("Stimber: %s failed!\n", __func__);
			g_front_resolution_created = 0;
	    }  
    } else {  
        pr_info("File Exist!\n");  
    }  
}
//ASUS_BSP Stimber_Hsueh ---


//ASUS_BSP Stimber_Hsueh +++
static int rear_module_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%s\n", g_rear_module);
				
    return 0;
}

static int rear_module_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rear_module_proc_read, NULL);
}

static const struct file_operations rear_module_fops = {
	.owner = THIS_MODULE,
	.open = rear_module_proc_open,
	.read = seq_read,
	//.write = status_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_rear_module_proc_file(void)
{	
	if(!g_rear_module_created) {   
        status_proc_file = proc_create(MODULE_REAR_PROC_FILE, 0666, NULL, &rear_module_fops);
		if(status_proc_file) {
			CDBG("Stimber: %s sucessed!\n", __func__);
			g_rear_module_created = 1;
	    } else {
			pr_err("Stimber:%s failed!\n", __func__);
			g_rear_module_created = 0;
	    }  
    } else {  
        pr_info("File Exist!\n");  
    }  
}

static int front_module_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%s\n", g_front_module);
				
    return 0;
}

static int front_module_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, front_module_proc_read, NULL);
}

static const struct file_operations front_module_fops = {
	.owner = THIS_MODULE,
	.open = front_module_proc_open,
	.read = seq_read,
	//.write = status_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_front_module_proc_file(void)
{	
	if(!g_front_module_created) {   
        status_proc_file = proc_create(MODULE_FRONT_PROC_FILE, 0666, NULL, &front_module_fops);
		if(status_proc_file) {
			CDBG("Stimber: %s sucessed!\n", __func__);
			g_front_module_created = 1;
	    } else {
			pr_err("Stimber: %s failed!\n", __func__);
			g_front_module_created = 0;
	    }  
    } else {  
        pr_info("File Exist!\n");  
    }  
}
//ASUS_BSP Stimber_Hsueh ---

//ASUS_BSP Stimber_Hsueh +++
int sensor_read_reg(struct msm_sensor_ctrl_t  *s_ctrl, u16 addr, u16 *val)
{
	int err;
      
	err = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,addr,val,MSM_CAMERA_I2C_BYTE_DATA);
	CDBG("sensor_read_reg 0x%x\n",*val);
	if(err <0)
		return -EINVAL;	
	else return 0;
}

int sensor_write_reg(struct msm_sensor_ctrl_t  *s_ctrl, u16 addr, u16 val)
{
	int err;
	int retry = 0;
	do {
		err =s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,addr,val,MSM_CAMERA_I2C_BYTE_DATA);		

		if (err == 0)
			return 0;
		retry++;
		pr_err("Stimber : i2c transfer failed, retrying %x %x\n",
		       addr, val);
		msleep(1); 
	} while (retry <= SENSOR_MAX_RETRIES);

	if(err == 0) {
		pr_err("%s(%d): i2c_transfer error, but return 0!?\n", __FUNCTION__, __LINE__);
		err = 0xAAAA;
	}

	return err;
}

struct tsb_af_data {
	u16 af_inf_pos;
	u16 af_30cm_pos;
	u16 af_10cm_pos;
	u16 af_start_curr;
	u8 module_id;
	u8 vendor_id;
	u32 date_code;
	u32 sn_number;
	u32 pn_number;
	u16 cks;
};

static int __tsb_otp_read(void)
{		
	int ret;
	u8 i, check_count;
	u16 check_status, check_data1, check_data2;
	int page_num;
	u16 read_value[MODULE_OTP_BULK_SIZE];
	u8 page_cnt = 0;
	u16 bank_value[MODULE_OTP_SIZE];
	int j;

	struct msm_sensor_ctrl_t  *s_ctrl = g_sctrl[CAMERA_0];

	page_num = 2;
	ret = sensor_write_reg(s_ctrl, 0x3500, 0x01);
	if (ret) {
		pr_err("%s: failed to write 0x3500 = 0x01\n",
			 __func__);
		return ret;
	}

	for(j=0; j<=page_num; j++){
		check_count = 0;
		ret = sensor_write_reg(s_ctrl, 0x3502, j);
		usleep_range(300, 500);
		pr_info("Stimber: otp now read bank %d\n", j+1);
		if (ret) {
			pr_err("%s: failed to write 0x3502 = %d\n",
				 __func__, j);
			return ret;
		}
		ret = sensor_write_reg(s_ctrl, 0x3500, 0x81);
		if (ret) {
			pr_err("Stimber: %s: failed to write 0x3500 = 0x81\n",
				 __func__);
			return ret;
		}
		do {
			pr_debug("%s : check access status ...\n", __func__);
			sensor_read_reg(s_ctrl, 0x3500, &check_status);
			usleep_range(300, 500);
			check_count++;
		} while (((check_status & 0x80) != 0) && (check_count <= 10));

		sensor_read_reg(s_ctrl, 0x3504, &check_data1);
		usleep_range(300, 500);
		sensor_read_reg(s_ctrl, 0x3505, &check_data2);
		usleep_range(300, 500);

		sensor_read_reg(s_ctrl, 0x3546, &check_status);
		if ((check_status & 0x08) == 1) {
			printk("Stimber: %s : check error status fail\n", __func__);
			ret = sensor_write_reg(s_ctrl, 0x3500, 0x00);
			if (ret) {
				pr_err("%s: failed to write 0x3500 = 0x00\n",
					 __func__);
				return ret;
			}
			return -1;
		}

		for (i = 0; i < MODULE_OTP_BULK_SIZE; i++) {
			sensor_read_reg(s_ctrl, 0x3504+i, &read_value[i]);

			bank_value[i+page_cnt*MODULE_OTP_BULK_SIZE] = read_value[i];
		}	

		page_cnt++;
	}

	snprintf(rear_module_otp, sizeof(rear_module_otp)
		, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n"
			, bank_value[0]&0xFF, bank_value[1]&0xFF, bank_value[2]&0xFF, bank_value[3]&0xFF, bank_value[4]&0xFF
			, bank_value[5]&0xFF, bank_value[6]&0xFF, bank_value[7]&0xFF, bank_value[8]&0xFF, bank_value[9]&0xFF
			, bank_value[10]&0xFF, bank_value[11]&0xFF, bank_value[12]&0xFF, bank_value[13]&0xFF, bank_value[14]&0xFF
			, bank_value[15]&0xFF, bank_value[16]&0xFF, bank_value[17]&0xFF, bank_value[18]&0xFF, bank_value[19]&0xFF
			, bank_value[20]&0xFF, bank_value[21]&0xFF, bank_value[22]&0xFF, bank_value[23]&0xFF
			, bank_value[24]&0xFF, bank_value[25]&0xFF, bank_value[26]&0xFF, bank_value[27]&0xFF, bank_value[28]&0xFF
			, bank_value[29]&0xFF, bank_value[30]&0xFF, bank_value[31]&0xFF, bank_value[32]&0xFF, bank_value[33]&0xFF
			, bank_value[34]&0xFF, bank_value[35]&0xFF, bank_value[36]&0xFF, bank_value[37]&0xFF, bank_value[38]&0xFF
			, bank_value[39]&0xFF, bank_value[40]&0xFF, bank_value[41]&0xFF, bank_value[42]&0xFF, bank_value[43]&0xFF
			, bank_value[44]&0xFF, bank_value[45]&0xFF, bank_value[46]&0xFF, bank_value[47]&0xFF
			, bank_value[48]&0xFF, bank_value[49]&0xFF, bank_value[50]&0xFF, bank_value[51]&0xFF, bank_value[52]&0xFF
			, bank_value[53]&0xFF, bank_value[54]&0xFF, bank_value[55]&0xFF, bank_value[56]&0xFF, bank_value[57]&0xFF
			, bank_value[58]&0xFF, bank_value[59]&0xFF, bank_value[60]&0xFF, bank_value[61]&0xFF, bank_value[62]&0xFF
			, bank_value[63]&0xFF, bank_value[64]&0xFF, bank_value[65]&0xFF, bank_value[66]&0xFF, bank_value[67]&0xFF
			, bank_value[68]&0xFF, bank_value[69]&0xFF, bank_value[70]&0xFF, bank_value[71]&0xFF
			, bank_value[72]&0xFF, bank_value[73]&0xFF, bank_value[74]&0xFF, bank_value[75]&0xFF, bank_value[76]&0xFF
			, bank_value[77]&0xFF, bank_value[78]&0xFF, bank_value[79]&0xFF, bank_value[80]&0xFF, bank_value[81]&0xFF
			, bank_value[82]&0xFF, bank_value[83]&0xFF, bank_value[84]&0xFF, bank_value[85]&0xFF, bank_value[86]&0xFF
			, bank_value[87]&0xFF, bank_value[88]&0xFF, bank_value[89]&0xFF, bank_value[90]&0xFF, bank_value[91]&0xFF
			, bank_value[92]&0xFF, bank_value[93]&0xFF, bank_value[94]&0xFF, bank_value[95]&0xFF);

	pr_debug("Stimber: %s OTP value: %s\n", __func__, rear_module_otp);

	ret = sensor_write_reg(s_ctrl, 0x3500, 0x00);
	if (ret) {
		pr_err("%s: failed to write 0x3500 = 0x00\n",
			 __func__);
		return ret;
	}

	return 0;
}

static void tsb_otp_read(void)
{
	int ret;

	ret = __tsb_otp_read();
	
	if (ret) {
		pr_err("%s: sensor found no valid OTP data\n",
			  __func__);
	}
}

static int __imx219_otp_read(void){
	int i=0;
	u16 otp_start_addr;
	int page_num;
	u16 i2c_get_value;
	struct msm_sensor_ctrl_t  *s_ctrl = g_sctrl[CAMERA_0];
	u16 bank_value[IMX219_ASUS_OTP_SIZE];
	//static u32 otp_get_value;

	otp_start_addr=IMX219_ASUS_OTP_START;
	page_num=0;
	sensor_write_reg(s_ctrl,0x0100,0x00);//write standby mode
	//sensor_read_reg(imx219_asus_s_ctrl.sensor_i2c_client->client, 0x0100, (u16 *)&otp_get_value);
	//pr_err("init_read_otp  +0x0100++ otp_get_value =%x\n",otp_get_value);

	sensor_write_reg(s_ctrl,0x3302,0x02);//write OTP write clock
	//sensor_read_reg(imx219_asus_s_ctrl.sensor_i2c_client->client, 0x3302, (u16 *)&otp_get_value);
	//pr_err("init_read_otp  +0x3302++ otp_get_value =%x\n",otp_get_value);
	sensor_write_reg(s_ctrl,0x3303,0x58);//write OTP write clock
	//sensor_read_reg(imx219_asus_s_ctrl.sensor_i2c_client->client, 0x3303, (u16 *)&otp_get_value);
	//pr_err("init_read_otp  +0x3303++ otp_get_value =%x\n",otp_get_value);

	sensor_write_reg(s_ctrl,0x012A,0x18);//write INCK
	//sensor_read_reg(imx219_asus_s_ctrl.sensor_i2c_client->client, 0x012A, (u16 *)&otp_get_value);
	//pr_err("init_read_otp  +0x012A++ otp_get_value =%x\n",otp_get_value);
	sensor_write_reg(s_ctrl,0x012B,0x00);//write INCK
	//sensor_read_reg(imx219_asus_s_ctrl.sensor_i2c_client->client, 0x012B, (u16 *)&otp_get_value);
	//pr_err("init_read_otp  +0x012B++ otp_get_value =%x\n",otp_get_value);

	sensor_write_reg(s_ctrl,0x3300,0x08);//write ECC
	//sensor_read_reg(imx219_asus_s_ctrl.sensor_i2c_client->client, 0x3300, (u16 *)&otp_get_value);
	//pr_err("init_read_otp  +0x3300++ otp_get_value =%x\n",otp_get_value);
/*
	sensor_write_reg(s_ctrl,0x3200,1);
	pr_info("PJ: otp now read bank %d\n", page_num);
	sensor_write_reg(s_ctrl,0x3202,page_num);
	sensor_read_reg(s_ctrl, IMX219_ASUS_OTP_MODULE_REG, (u16 *)&i2c_get_value);
	pr_info("PJ: module =%x\n",i2c_get_value);
	if( i2c_get_value==0x0){
		page_num=1;	
		sensor_write_reg(s_ctrl,0x3200,1);
		pr_info("PJ: otp now read bank %d\n", page_num);
		sensor_write_reg(s_ctrl,0x3202,page_num);
		sensor_read_reg(s_ctrl, IMX219_ASUS_OTP_MODULE_REG, (u16 *)&i2c_get_value);
		pr_info("PJ: module =%x\n",i2c_get_value);
		if(i2c_get_value==0x0){
			page_num=0;
			sensor_write_reg(s_ctrl,0x3200,1);
			pr_info("PJ: otp now read bank %d\n", page_num);
			sensor_write_reg(s_ctrl,0x3202,page_num);
			sensor_read_reg(s_ctrl, IMX219_ASUS_OTP_MODULE_REG, (u16 *)&i2c_get_value);
			pr_info("PJ: module =%x\n",i2c_get_value);
			if(i2c_get_value==0x0) return -1;
		}
	}
*/
	for(i=0;i<IMX219_ASUS_OTP_SIZE;i++) {
		if (i % 32 == 0) {
			sensor_write_reg(s_ctrl,0x3200,1);
			pr_info("PJ: otp now read bank %d\n", page_num);
			sensor_write_reg(s_ctrl,0x3202,page_num);
			page_num++;
		}
		sensor_read_reg(s_ctrl, otp_start_addr+i%32, (u16 *)&i2c_get_value);
		//pr_info("imx219_otp_read  byte=%x addr=%x value=%x\n",i,otp_start_addr+i,i2c_get_value);
		bank_value[i]=i2c_get_value;
	}
/*
	for(i=0;i<IMX219_ASUS_OTP_SIZE;i++)
		printk("imx219_otp_read i=%d =%x\n",i,rear_module_otp[i]);
*/
	snprintf(rear_module_otp, sizeof(rear_module_otp)
	, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n"
			, bank_value[0]&0xFF, bank_value[1]&0xFF, bank_value[2]&0xFF, bank_value[3]&0xFF, bank_value[4]&0xFF
			, bank_value[5]&0xFF, bank_value[6]&0xFF, bank_value[7]&0xFF, bank_value[8]&0xFF, bank_value[9]&0xFF
			, bank_value[10]&0xFF, bank_value[11]&0xFF, bank_value[12]&0xFF, bank_value[13]&0xFF, bank_value[14]&0xFF
			, bank_value[15]&0xFF, bank_value[16]&0xFF, bank_value[17]&0xFF, bank_value[18]&0xFF, bank_value[19]&0xFF
			, bank_value[20]&0xFF, bank_value[21]&0xFF, bank_value[22]&0xFF, bank_value[23]&0xFF, bank_value[24]&0xFF
			, bank_value[25]&0xFF, bank_value[26]&0xFF, bank_value[27]&0xFF, bank_value[28]&0xFF, bank_value[29]&0xFF
			, bank_value[30]&0xFF, bank_value[31]&0xFF, bank_value[32]&0xFF, bank_value[33]&0xFF, bank_value[34]&0xFF
			, bank_value[35]&0xFF, bank_value[36]&0xFF, bank_value[37]&0xFF, bank_value[38]&0xFF, bank_value[39]&0xFF
			, bank_value[40]&0xFF, bank_value[41]&0xFF, bank_value[42]&0xFF, bank_value[43]&0xFF, bank_value[44]&0xFF
			, bank_value[45]&0xFF, bank_value[46]&0xFF, bank_value[47]&0xFF, bank_value[48]&0xFF, bank_value[49]&0xFF
			, bank_value[50]&0xFF, bank_value[51]&0xFF, bank_value[52]&0xFF, bank_value[53]&0xFF, bank_value[54]&0xFF
			, bank_value[55]&0xFF, bank_value[56]&0xFF, bank_value[57]&0xFF, bank_value[58]&0xFF, bank_value[59]&0xFF
			, bank_value[60]&0xFF, bank_value[61]&0xFF, bank_value[62]&0xFF, bank_value[63]&0xFF, bank_value[64]&0xFF
			, bank_value[65]&0xFF, bank_value[66]&0xFF, bank_value[67]&0xFF, bank_value[68]&0xFF, bank_value[69]&0xFF
			, bank_value[70]&0xFF, bank_value[71]&0xFF, bank_value[72]&0xFF, bank_value[73]&0xFF, bank_value[74]&0xFF
			, bank_value[75]&0xFF, bank_value[76]&0xFF, bank_value[77]&0xFF, bank_value[78]&0xFF, bank_value[79]&0xFF
			, bank_value[80]&0xFF, bank_value[81]&0xFF, bank_value[82]&0xFF, bank_value[83]&0xFF, bank_value[84]&0xFF
			, bank_value[85]&0xFF, bank_value[86]&0xFF, bank_value[87]&0xFF, bank_value[88]&0xFF, bank_value[89]&0xFF
			, bank_value[90]&0xFF, bank_value[91]&0xFF, bank_value[92]&0xFF, bank_value[93]&0xFF, bank_value[94]&0xFF
			, bank_value[95]&0xFF);

	pr_debug("PJ: %s OTP value: %s\n", __func__, rear_module_otp);

	return 0;
}

static void imx219_otp_read(void)
{
	int ret;

	ret = __imx219_otp_read();
	
	if (ret) {
		pr_err("%s: sensor found no valid OTP data\n",
			  __func__);
	}
}

static u16 PLLsetting1[9][2]={
	{ 0x300A, 0x09 },
	{ 0x300B, 0x03 },
	{ 0x0304, 0x00 },
	{ 0x0305, 0x02 },
	{ 0x0306, 0x00 },
	{ 0x0307, 0x9C },
	{ 0x3004, 0x01 },
	{ 0x3005, 0xDF },
	{ 0x3000, 0x03 },
};

static u16 PLLsetting2[3][2]={
	{ 0x3007, 0x40 },
	{ 0x3000, 0x43 },
	{ 0x300A, 0x08 },
};

int __mn34150_otp_read(void)
{
	int ret;
	u8 i;
	int page_num;
	u16 read_value[MODULE_OTP_BULK_SIZE];
	u8 page_cnt = 0;
	u16 bank_value[MODULE_OTP_SIZE];
	int j;


	struct msm_sensor_ctrl_t  *s_ctrl = g_sctrl[CAMERA_0];

	page_num = 2;
	//PLL setting 1
	for (j=0;j<sizeof(PLLsetting1)/(2*sizeof(u16));j++) {
		 ret = sensor_write_reg(s_ctrl, PLLsetting1[j][0], PLLsetting1[j][1]);
		if (ret) {
			pr_err("%s: failed to write 0x%x = 0x%x\n",
				 __func__,PLLsetting1[j][0], PLLsetting1[j][1]);
			return ret;
		}
	}
	usleep(85);
	//PLL setting 2
	for (j=0;j<sizeof(PLLsetting2)/(2*sizeof(u16));j++) {
		 ret = sensor_write_reg(s_ctrl, PLLsetting2[j][0], PLLsetting2[j][1]);
		if (ret) {
			pr_err("%s: failed to write 0x%x = 0x%x\n",
				 __func__,PLLsetting2[j][0], PLLsetting2[j][1]);
			return ret;
		}
	}
	usleep(1005);

	for(j=0; j<=page_num; j++){
		for (i = 0; i < MODULE_OTP_BULK_SIZE; i++) {
			sensor_read_reg(s_ctrl, 0x3481+i+page_cnt*MODULE_OTP_BULK_SIZE, &read_value[i]);
			bank_value[i+page_cnt*MODULE_OTP_BULK_SIZE] = read_value[i];
		}
		page_cnt++;
	}

	snprintf(rear_module_otp, sizeof(rear_module_otp)
		, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n"
			, bank_value[0]&0xFF, bank_value[1]&0xFF, bank_value[2]&0xFF, bank_value[3]&0xFF, bank_value[4]&0xFF
			, bank_value[5]&0xFF, bank_value[6]&0xFF, bank_value[7]&0xFF, bank_value[8]&0xFF, bank_value[9]&0xFF
			, bank_value[10]&0xFF, bank_value[11]&0xFF, bank_value[12]&0xFF, bank_value[13]&0xFF, bank_value[14]&0xFF
			, bank_value[15]&0xFF, bank_value[16]&0xFF, bank_value[17]&0xFF, bank_value[18]&0xFF, bank_value[19]&0xFF
			, bank_value[20]&0xFF, bank_value[21]&0xFF, bank_value[22]&0xFF, bank_value[23]&0xFF
			, bank_value[24]&0xFF, bank_value[25]&0xFF, bank_value[26]&0xFF, bank_value[27]&0xFF, bank_value[28]&0xFF
			, bank_value[29]&0xFF, bank_value[30]&0xFF, bank_value[31]&0xFF, bank_value[32]&0xFF, bank_value[33]&0xFF
			, bank_value[34]&0xFF, bank_value[35]&0xFF, bank_value[36]&0xFF, bank_value[37]&0xFF, bank_value[38]&0xFF
			, bank_value[39]&0xFF, bank_value[40]&0xFF, bank_value[41]&0xFF, bank_value[42]&0xFF, bank_value[43]&0xFF
			, bank_value[44]&0xFF, bank_value[45]&0xFF, bank_value[46]&0xFF, bank_value[47]&0xFF
			, bank_value[48]&0xFF, bank_value[49]&0xFF, bank_value[50]&0xFF, bank_value[51]&0xFF, bank_value[52]&0xFF
			, bank_value[53]&0xFF, bank_value[54]&0xFF, bank_value[55]&0xFF, bank_value[56]&0xFF, bank_value[57]&0xFF
			, bank_value[58]&0xFF, bank_value[59]&0xFF, bank_value[60]&0xFF, bank_value[61]&0xFF, bank_value[62]&0xFF
			, bank_value[63]&0xFF, bank_value[64]&0xFF, bank_value[65]&0xFF, bank_value[66]&0xFF, bank_value[67]&0xFF
			, bank_value[68]&0xFF, bank_value[69]&0xFF, bank_value[70]&0xFF, bank_value[71]&0xFF
			, bank_value[72]&0xFF, bank_value[73]&0xFF, bank_value[74]&0xFF, bank_value[75]&0xFF, bank_value[76]&0xFF
			, bank_value[77]&0xFF, bank_value[78]&0xFF, bank_value[79]&0xFF, bank_value[80]&0xFF, bank_value[81]&0xFF
			, bank_value[82]&0xFF, bank_value[83]&0xFF, bank_value[84]&0xFF, bank_value[85]&0xFF, bank_value[86]&0xFF
			, bank_value[87]&0xFF, bank_value[88]&0xFF, bank_value[89]&0xFF, bank_value[90]&0xFF, bank_value[91]&0xFF
			, bank_value[92]&0xFF, bank_value[93]&0xFF, bank_value[94]&0xFF, bank_value[95]&0xFF);

	pr_debug("PJ: %s OTP value: %s\n", __func__, rear_module_otp);

	return 0;
}

static void mn34150_otp_read(void)
{
	int ret;

	ret = __mn34150_otp_read();
	if (ret) {
		pr_err("%s: sensor found no valid OTP data\n",
			  __func__);
	}
}

#define	REAR_OTP_PROC_FILE	"driver/rear_otp"
static struct proc_dir_entry *otp_proc_file;

static int rear_otp_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%s\n", rear_module_otp);
    return 0;
}

static int rear_otp_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rear_otp_proc_read, NULL);
}

static const struct file_operations rear_otp_fops = {
	.owner = THIS_MODULE,
	.open = rear_otp_proc_open,
	.read = seq_read,
	//.write = otp_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_rear_otp_proc_file(void)
{
	if(!g_rear_otp_created) {
	    otp_proc_file = proc_create(REAR_OTP_PROC_FILE, 0666, NULL, &rear_otp_fops);
	    if (otp_proc_file) {
			CDBG("Stimber: %s sucessed!\n", __func__);
			g_rear_otp_created = 1;
	    } else {
			printk("Stimber: %s failed!\n", __func__);
			g_rear_otp_created = 0;
	    }
	} else {  
        pr_info("File Exist!\n");  
    }
}
//ASUS_BSP Stimber_Hsueh ---



//ASUS_BSP+++ CR_0 Randy_Change@asus.com.tw [2015/3/5] Modify Begin
#define	FRONT_OTP_PROC_FILE	"driver/front_otp"
static struct proc_dir_entry *otp_front_proc_file;

static int front_otp_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%s\n", front_module_otp);
    return 0;
}

static int front_otp_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, front_otp_proc_read, NULL);
}

static const struct file_operations front_otp_fops = {
	.owner = THIS_MODULE,
	.open = front_otp_proc_open,
	.read = seq_read,
	//.write = otp_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_front_otp_proc_file(void)
{
	if(!g_front_otp_created) {
	    otp_front_proc_file = proc_create(FRONT_OTP_PROC_FILE, 0666, NULL, &front_otp_fops);
	    if (otp_front_proc_file) {
			CDBG("Randy: %s sucessed!\n", __func__);
			g_front_otp_created = 1;
	    } else {
			printk("Randy: %s failed!\n", __func__);
			g_front_otp_created = 0;
	    }
	} else {  
        pr_info("File Exist!\n");  
    }
}
//ASUS_BSP--- CR_0 Randy_Change@asus.com.tw [2015/3/5] Modify End

//ASUS_BSP +++ bill_chen "Implement gc2155 control OB interface"
#define	GC2155_OBVALUE_PROC_FILE	"driver/gc2155_obvalue"
static struct proc_dir_entry *obvalue_gc2155_proc_file;

static ssize_t gc2155_obvalue_proc_wrtie(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int DarkMode, rc = 0;
	char messages[8];
	struct msm_sensor_ctrl_t *s_ctrl = g_sctrl[CAMERA_1];
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buff, len)) {
		printk("[OB] %s commond fail !!\n", __func__);
		return -EFAULT;
	}

	DarkMode = (int)simple_strtol(messages, NULL, 10);

    rc = sensor_write_reg(s_ctrl, 0xfe, 0x00);
    rc = sensor_write_reg(s_ctrl, 0x5c, DarkMode);

	printk("[OB] %s: val = %d", __func__, DarkMode);

	return len;
}

static int gc2155_obvalue_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%s\n", "0");
    return 0;
}

static int gc2155_obvalue_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, gc2155_obvalue_proc_read, NULL);
}

static const struct file_operations gc2155_obvalue_fops = {
	.owner = THIS_MODULE,
	.open = gc2155_obvalue_proc_open,
	//.read = seq_read,
	.write = gc2155_obvalue_proc_wrtie,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_gc2155_obvalue_proc_file(void)
{
	obvalue_gc2155_proc_file = proc_create(GC2155_OBVALUE_PROC_FILE, 0776, NULL, &gc2155_obvalue_fops);
	if (obvalue_gc2155_proc_file) {
		printk("[OB] %s sucessed!\n", __func__);
	} else {
		printk("[OB] %s failed!\n", __func__);
	}
}
//ASUS_BSP --- bill_chen "Implement gc2155 control OB interface"

module_init(msm_sensor_driver_init);
module_exit(msm_sensor_driver_exit);
MODULE_DESCRIPTION("msm_sensor_driver");
MODULE_LICENSE("GPL v2");
