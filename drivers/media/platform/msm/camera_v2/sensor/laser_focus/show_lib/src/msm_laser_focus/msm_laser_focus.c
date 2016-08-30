/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/

#include "msm_laser_focus.h"
#include "show_log.h"

static struct mutex _mutex;

/** @brief Check device verify number
*	
*	@param dev_t the laser focus controller
*
*/
int match_id(struct msm_laser_focus_ctrl_t *dev_t, int chip_id_size)
{
	int rc = 0;
	uint16_t chipid = 0;
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct msm_camera_slave_info *slave_info;
	const char *sensor_name;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if (!dev_t) {
		LOG_Handler(LOG_ERR, "%s: failed: %p\n", __func__, dev_t);
		return -EINVAL;
	}
	sensor_i2c_client = dev_t->i2c_client;
	slave_info = dev_t->sensordata->slave_info;
	sensor_name = dev_t->sensordata->sensor_name;

	/* Check sensor client, slave information and sensor name if NULL */
	if (!sensor_i2c_client || !slave_info || !sensor_name) {
		LOG_Handler(LOG_ERR, "%s: failed: %p %p %p\n", __func__, 
			sensor_i2c_client, slave_info, sensor_name);
		return -EINVAL;
	}

	/* Get verify number */
	rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
		sensor_i2c_client, slave_info->sensor_id_reg_addr, 
		&chipid, chip_id_size);
	if (rc < 0) {
		LOG_Handler(LOG_ERR, "%s: %s: read id failed\n", __func__, sensor_name);
		return rc;
	}
	LOG_Handler(LOG_DBG, "%s: read id: 0x%x expected id 0x%x\n", __func__, chipid, slave_info->sensor_id);

	/* Compare verify number */
	if (chipid != slave_info->sensor_id) {
		LOG_Handler(LOG_ERR, "%s: msm_sensor_match_id chip id doesnot match\n", __func__);
		return -ENODEV;
	}
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return rc;
}

/** @brief Voltage control
*	
*	@param dev_t the laser focus controller
*	@param config the configuration (>0:enable; 0:disable)
*
*/
int32_t vreg_control(struct msm_laser_focus_ctrl_t *dev_t, int config)
{
	int rc = 0, i, cnt;
	struct msm_laser_focus_vreg *vreg_cfg;
	
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	vreg_cfg = &dev_t->vreg_cfg;
	cnt = vreg_cfg->num_vreg;
	if (!cnt)
		return 0;

	if (cnt >= CAM_VREG_MAX) {
		LOG_Handler(LOG_ERR, "%s: failed %d cnt %d\n", __func__, __LINE__, cnt);
		return -EINVAL;
	}

	for (i = 0; i < cnt; i++) {
		rc = msm_camera_config_single_vreg(&(dev_t->pdev->dev),
			&vreg_cfg->cam_vreg[i], (struct regulator **)&vreg_cfg->data[i], config);
	}
	
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return rc;
}

/** @brief Power on component
*	
*	@param dev_t the laser focus controller
*
*/
int32_t power_up(struct msm_laser_focus_ctrl_t *dev_t)
{
	int rc = 0;
	
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Enable voltage */
	rc = vreg_control(dev_t, ENABLE_VREG);
	if (rc < 0) {
		LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
		return rc;
	}

	/* Set current status to power on state */
	dev_t->laser_focus_state = LASER_FOCUS_POWER_UP;

	/* Set GPIO vdig to high for sku4 */
	GPIO_Handler(dev_t, SENSOR_GPIO_VDIG, GPIO_HIGH);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return rc;
}

/** @brief Power off component
*	
*	@param dev_t the laser focus controller
*
*/
int32_t power_down(struct msm_laser_focus_ctrl_t *dev_t)
{
	int32_t rc = 0;
	
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Check device status */
	if (dev_t->laser_focus_state != LASER_FOCUS_POWER_DOWN) {

		/* Disable voltage */
		rc = vreg_control(dev_t, DISABLE_VREG);
		if (rc < 0) {
			LOG_Handler(LOG_ERR, "%s failed %d\n", __func__, __LINE__);
			return rc;
		}

		/* release memory for i2c table */
		kfree(dev_t->i2c_reg_tbl);
		dev_t->i2c_reg_tbl = NULL;
		dev_t->i2c_tbl_index = 0;
		/* Set current status to power off state*/
		dev_t->laser_focus_state = LASER_FOCUS_POWER_DOWN;
	}

	/* Set GPIO vdig to low for sku4 */
	GPIO_Handler(dev_t, SENSOR_GPIO_VDIG, GPIO_LOW);
	
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return rc;
}

/** @brief Initialize device
*	
*	@param dev_t the laser focus controller
*
*/
int dev_init(struct msm_laser_focus_ctrl_t *dev_t)
{
	int rc = 0;
	
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	if (!dev_t) {
		LOG_Handler(LOG_ERR, "%s: failed\n", __func__);
		return -EINVAL;
	}
	
	// CCI initialize
	if (dev_t->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = dev_t->i2c_client->i2c_func_tbl->i2c_util(dev_t->i2c_client, MSM_CCI_INIT);
		if (rc < 0)
			LOG_Handler(LOG_ERR, "%s: cci_init failed\n", __func__);
	}
	
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return rc;
}

/** @brief Deinitialize device
*	
*	@param dev_t the laser focus controller
*
*/
int dev_deinit(struct msm_laser_focus_ctrl_t *dev_t) 
{
	int rc = 0;
	
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	if (!dev_t) {
		LOG_Handler(LOG_ERR, "%s: failed\n", __func__);
		return -EINVAL;
	}
	// CCI deinitialize 
	if (dev_t->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = dev_t->i2c_client->i2c_func_tbl->i2c_util(dev_t->i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			LOG_Handler(LOG_ERR, "%s: cci_deinit failed\n", __func__);
		
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return rc;
}

/** @brief Check device status
*	
*	@param dev_t the laser focus controller
*
*/
int dev_I2C_status_check(struct msm_laser_focus_ctrl_t *dev_t, int chip_id_size){
	int32_t rc;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* VL6180x only */
	if(g_ASUS_laserID == 1){
		/* Power on device */
		rc = power_up(dev_t);
		if (rc < 0) {
			LOG_Handler(LOG_ERR, "%s: dev_power_up failed %d\n", __func__, __LINE__);
			return 0;
		}
	}
	
	/* Initialize device */
	dev_init(dev_t);
	if (rc < 0) {
		LOG_Handler(LOG_ERR, "%s: dev_init failed %d\n", __func__, __LINE__);
		return 0;
	}

	/* Check device ID */
	rc = match_id(dev_t, chip_id_size);
	if (rc < 0) {
		LOG_Handler(LOG_ERR, "%s: dev_match_id failed %d\n", __func__, __LINE__);
		
		/* If device id is not match, deinitialize and power off device*/
		rc = dev_deinit(dev_t);
		if (rc < 0) {
			LOG_Handler(LOG_ERR, "%s: dev_deinit failed %d\n", __func__, __LINE__);
		}
		rc = power_down(dev_t);
		if (rc < 0) {
			LOG_Handler(LOG_ERR, "%s: dev_power_down failed %d\n", __func__, __LINE__);
		}
		
		return 0;
	}
	/* Deinitialize device */
	rc = dev_deinit(dev_t);
	if (rc < 0) {
		LOG_Handler(LOG_ERR, "%s: dev_deinit failed %d\n", __func__, __LINE__);
		return 0;
	}

	/* VL6180x only */
	if(g_ASUS_laserID == 1){
		/* Power off device */
		rc = power_down(dev_t);
		if (rc < 0) {
			LOG_Handler(LOG_ERR, "%s: dev_power_down failed %d\n", __func__, __LINE__);
			return 0;
		}
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 1;
}

/** @brief Mutex controller handles mutex action
*	
*	@param dev_t the laser focus controller
*	@param ctrl the action of mutex
*
*/
int mutex_ctrl(struct msm_laser_focus_ctrl_t *dev_t, int ctrl)
{
	int rc = 0;
	
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	if(!dev_t){
		LOG_Handler(LOG_ERR, "%s: fail dev_t %p is NULL\n", __func__, dev_t);
		return -EFAULT;
	}

	switch(ctrl){
		/* Allocate mutex */
		/*case MUTEX_ALLOCATE:
			rc = _mutex_allocate(&dev_t->laser_focus_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_allocate\n", __func__);
				return rc;
			}
			break;
		*/
		/* Initialize mutex */
		case MUTEX_INIT:
			mutex_init(&_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_init\n", __func__);
				return rc;
			}
			break;
		/* Lock mutex */
		case MUTEX_LOCK:
			mutex_lock(&_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_lock\n", __func__);
				return rc;
			}
			break;
		/* Try lock mutex*/
		case MUTEX_TRYLOCK:
			mutex_trylock(&_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_trylock\n", __func__);
				return rc;
			}
			break;
		/* Unlock mutex */
		case MUTEX_UNLOCK:
			 mutex_unlock(&_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_unlock\n", __func__);
				return rc;
			}
			break;
		/* Destroy mutex */
		case MUTEX_DESTROY:
			mutex_destroy(&_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_destroy\n", __func__);
				return rc;
			}
			break;
		default:
			LOG_Handler(LOG_ERR, "%s: command fail !!\n", __func__);
			break;
	}
	
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;;
}

/** @brief Laser focus driver prob function
*	
*	@param pdev platform device information
*	@param dev_t the laser focus controller
*	@param msm_sensor_cci_func_tbl Cci function table
*	@param msm_laser_focus_internal_ops Laser focus operators
*	@param msm_laser_focus_subdev_ops Laser focus sub devices operators
*	@parammsm_laser_focus_i2c_client The i2c client information
*	@param laser_focus_dt_match[] the struct used for matching laser device
*	@param LF_status the prob stauts of laser sensor
*
*/
int32_t Laser_Focus_platform_probe(struct platform_device *pdev, struct msm_laser_focus_ctrl_t *dev_t,
										struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl,
										const struct v4l2_subdev_internal_ops msm_laser_focus_internal_ops,
										struct v4l2_subdev_ops msm_laser_focus_subdev_ops,
										struct msm_camera_i2c_client msm_laser_focus_i2c_client,
										const struct of_device_id laser_focus_dt_match[], int *LF_status){

	int32_t rc = 0;

	const struct of_device_id *match;
	struct msm_camera_cci_client *cci_client = NULL;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Initialize Laser Focus status */
	*LF_status = 0;

	/* Check if laser device matches an of_device_id list */
	match = of_match_device(laser_focus_dt_match, &pdev->dev);
	if (!match) {
		LOG_Handler(LOG_ERR, "%s: device not match\n", __func__);
		return -EFAULT;
	}
	if (!pdev->dev.of_node) {
		LOG_Handler(LOG_ERR, "%s: of_node NULL\n", __func__);
		return -EINVAL;
	}
#if 0
	/* Allocate memory */
	dev_t = kzalloc(sizeof(struct msm_laser_focus_ctrl_t), GFP_KERNEL);
	if (!dev_t) {
		LOG_Handler(LOG_ERR, "%s: failed no memory\n", __func__);
		return -ENOMEM;
	}
#endif	
	/* Set platform device handle */
	dev_t->pdev = pdev;

	/* Get dtsi data */
	rc = get_dtsi_data(pdev->dev.of_node, dev_t);
	if (rc < 0) {
		LOG_Handler(LOG_ERR, "%s: failed line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}
	
	/* Assign name for sub device */
	snprintf(dev_t->msm_sd.sd.name, sizeof(dev_t->msm_sd.sd.name), "%s", dev_t->sensordata->sensor_name);

	dev_t->act_v4l2_subdev_ops = &msm_laser_focus_subdev_ops;

	/* Set device type as platform device */
	dev_t->act_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	dev_t->i2c_client = &msm_laser_focus_i2c_client;
	if (NULL == dev_t->i2c_client) {
		LOG_Handler(LOG_ERR, "%s: i2c_client NULL\n", __func__);
		rc = -EFAULT;
		goto probe_failure;
	}
	if (!dev_t->i2c_client->i2c_func_tbl){
		dev_t->i2c_client->i2c_func_tbl = &msm_sensor_cci_func_tbl;
	}
	
	/* Set platform device handle */
	dev_t->i2c_client->cci_client = kzalloc(sizeof(struct msm_camera_cci_client), GFP_KERNEL);
	if (!dev_t->i2c_client->cci_client) {
		kfree(dev_t->vreg_cfg.cam_vreg);
		kfree(dev_t);
		LOG_Handler(LOG_ERR, "%s: failed no memory\n", __func__);
		return -ENOMEM;
	}

	/* Set device client information as platform device */
	//dev_t->i2c_client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	cci_client = dev_t->i2c_client->cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = dev_t->cci_master;
	if (dev_t->sensordata->slave_info->sensor_slave_addr){
		cci_client->sid = dev_t->sensordata->slave_info->sensor_slave_addr >> 1;
	}
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = I2C_FAST_MODE;
	v4l2_subdev_init(&dev_t->msm_sd.sd, dev_t->act_v4l2_subdev_ops);
	v4l2_set_subdevdata(&dev_t->msm_sd.sd, dev_t);
	dev_t->msm_sd.sd.internal_ops = &msm_laser_focus_internal_ops;
	dev_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(dev_t->msm_sd.sd.name, ARRAY_SIZE(dev_t->msm_sd.sd.name), "msm_laser_focus");
	media_entity_init(&dev_t->msm_sd.sd.entity, 0, NULL, 0);
	dev_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	dev_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_LASER_FOCUS;
	dev_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&dev_t->msm_sd);
	dev_t->laser_focus_state = LASER_FOCUS_POWER_DOWN;

	/* Init data struct */
	dev_t->laser_focus_cross_talk_offset_value = 0;
	dev_t->laser_focus_offset_value = 0;
	dev_t->laser_focus_state = MSM_LASER_FOCUS_DEVICE_OFF;

#if 0
	/* Check device stauts */
	if(dev_I2C_status_check(dev_t, MSM_CAMERA_I2C_WORD_DATA) == 0){
		goto probe_failure;
	}
#endif

	/* Init mutex */
       mutex_ctrl(dev_t, MUTEX_ALLOCATE);
	mutex_ctrl(dev_t, MUTEX_INIT);

	/* Set laser sensor status to ok */
	*LF_status = 1;

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
probe_failure:
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return rc;
}
