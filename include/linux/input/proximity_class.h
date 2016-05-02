/*
 * proximity class driver
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

#ifndef __LINUX_PROXIMITY_CLASS_H__
#define __LINUX_PROXIMITY_CLASS_H__

/*Ugly Design: New types on Eclair,sdev->id*/
#define SENSORS_ACCELERATION		(1)
#define SENSORS_MAGNETIC_FIELD		(2)
#define SENSORS_ORIENTATION		(3)
#define SENSORS_GYROSCOPE           	(4)
#define SENSORS_LIGHT			(5)
#define SENSOR_TYPE_PRESSURE            (6)
#define SENSORS_TEMPERATURE		(7)
#define SENSORS_PROXIMITY		(8)
#define SENSORS_TRICORDER		(9)
#define SENSORS_HALL			(10)
#define SENSORS_MASK			(11)

#define DEFAULT_ID		(0)

enum proximity_property {
	/* int */
	SENSORS_PROP_INTERVAL = 0,
	SENSORS_PROP_HI_THRESHOLD,
    SENSORS_PROP_LO_THRESHOLD,
	SENSORS_PROP_MODE,
	SENSORS_PROP_TEST_STEP,
	SENSORS_PROP_MAXRANGE,		/* read only */
	SENSORS_PROP_RESOLUTION,	/* read only */
	SENSORS_PROP_VERSION,		/* read only */
	SENSORS_PROP_CURRENT,		/* read only */
    SENSORS_PROP_CALIBRATION,
    SENSORS_PROP_ADC,           /* read only */
    SENSORS_PROP_K_ADC,         /* read only */
    SENSORS_PROP_LUX,           /* read only */
    SENSORS_PROP_ATD_STATUS,
    SENSORS_PROP_ATD_ADC,
	SENSORS_PROP_DBG,           /* Add for debug only */
	SENSORS_PROP_HI_CAL,
	SENSORS_PROP_LOW_CAL,
	SENSORS_PROP_200_CAL,
	SENSORS_PROP_1000_CAL,

	/* char */
	SENSORS_PROP_SWITCH,
	SENSORS_PROP_VENDOR,		/* read only */
	SENSORS_PROP_REGISTER,		/* read only */
};

#define PRO_STR_SIZE		20	
union proximity_propval {
	int intval;
	char strval[PRO_STR_SIZE];
};

struct proximity_class_dev {
	int		        minor;
	unsigned int	id;
	const char	    *name;
	size_t          num_properties;
	enum proximity_property *properties;
	int (*get_property)(struct proximity_class_dev *prxdev,
			    enum proximity_property property,
			    union proximity_propval *val);
	int (*put_property)(struct proximity_class_dev *prxdev,
			    enum proximity_property property,
			    union proximity_propval *val);

	struct file_operations	*fops;
	struct list_head list;
	struct device	*dev;
};

extern int proximity_dev_register(struct proximity_class_dev *prxdev);
extern int proximity_dev_unregister(struct proximity_class_dev *prxdev);
//extern int als_lux_report_event_register(struct input_dev *dev);
extern int set_camera_autobrightness_mode(int mode);
extern void als_lux_report_event(int lux);

#define DEFAULT_ALS_SHIFT			40
#define DEFAULT_ALS_GAIN			38

#define DEFAULT_MAX_ALS_LUX		10000

#define DEFAULT_PS_THRESHOLD_lo	10
#define DEFAULT_PS_THRESHOLD_hi	12

#define DEFAULT_ALS_200LUX_CALVALUE			200
#define DEFAULT_ALS_1000LUX_CALVALUE			1000

#define LSENSOR_CALIBRATION_ASUS_NV_FILE  "/data/asusdata/lsensor.nv"
#define LSENSOR_CALIBRATION_SHIFT_ASUS_NV_FILE  "/data/asusdata/lsensor_shift.nv"
#define LSENSOR_CALIBRATION_200LUX	"/data/asusdata/lsensor_200lux.nv"
#define LSENSOR_CALIBRATION_1000LUX	"/data/asusdata/lsensor_1000lux.nv"

#define PSENSOR_CALIBRATION_HI_ASUS_NV_FILE  "/data/asusdata/psensor_hi.nv"
#define PSENSOR_CALIBRATION_LO_ASUS_NV_FILE  "/data/asusdata/psensor_low.nv"
/* ASUS BSP Peter_Lu for Proximity sensor new setting and new calibration issue */
#define PSENSOR_CALIBRATION_CHECK_ASUS_NV_FILE  "/data/asusdata/Proximity_calibration_new_version.nv"
#endif /* __LINUX_PROXIMITY_CLASS_H__ */
