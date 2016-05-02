/*
 *  drivers/sensors/proxm_class.c
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
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/input.h>
#include "linux/input/proximity_class.h"

//.....................For Camera backlight issue...........................//
/*
enum camera_iso {
	NORMAL_AUTO_MODE = 0,
	MEDIUM_ISO_BRIGHTNESS,
	MAX_ISO_BRIGHTNESS,
};

static u8 camera_autobrightness_mode = 0;
static int max_light_level = 12;
static int light_map[12] = {0,50,100,200,300,450,700,850,1050,1250,1500,2200};
extern u16 g_cm36283_light;

int set_camera_autobrightness_mode(int mode)
{
	camera_autobrightness_mode = mode;
	if ( mode > NORMAL_AUTO_MODE )	{
		printk("[als] Set camera autobrightness mode : ");
		if ( mode == MEDIUM_ISO_BRIGHTNESS )	{
			printk("Medium brightness mode(%d)\n", camera_autobrightness_mode);
			// Send Medium brightness to system
			als_lux_report_event(g_cm36283_light);
		}
		else if ( mode == MAX_ISO_BRIGHTNESS )	{
			printk("Max brightness mode(%d)\n", camera_autobrightness_mode);
			// Send Max brightness to system
			als_lux_report_event(2200);
		}
		else	{
			printk("Other mode(%d)\n", camera_autobrightness_mode);
			printk("[als] Invalid value!!\n");
			return -EINVAL;
		}
	}	
	else if ( mode == NORMAL_AUTO_MODE )	{
		printk("[als] Set numal autobrightness mode(%d)\n", camera_autobrightness_mode);
		als_lux_report_event(g_cm36283_light);
	}
	else	{
		printk("[als] Set mode fail! (Invalid value!)\n");
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(set_camera_autobrightness_mode);
*/

/////////////////////////////////////////////////////////////////////////////////
// report Lux input event feature
//
#include <linux/slab.h>
struct input_dev *master_input_dev = NULL;

int als_lux_report_event_register(struct input_dev *dev)
{
	if (master_input_dev != NULL)	{
		printk("[als] Input_dev already registered");
		return -EBUSY;
	}
	
	if (!(master_input_dev = kmalloc(sizeof(struct input_dev), GFP_KERNEL)))
		return -ENOMEM;

	memset( master_input_dev, 0, sizeof(struct input_dev) );

	master_input_dev = dev;
	printk("[als] Finish register input_dev for lightsensoer : %s", dev->name);

	return 0;
}
EXPORT_SYMBOL(als_lux_report_event_register);

void als_lux_report_event(int lux)
{
	//int i, level = 0;
	printk("[als] ******* report lux = %d\n", lux);
//.....................For Camera backlight issue...........................//
/*
	if ( camera_autobrightness_mode > NORMAL_AUTO_MODE )	{
		if ( camera_autobrightness_mode == MAX_ISO_BRIGHTNESS )	{
			lux = 2200;
		}
		else if ( camera_autobrightness_mode == MEDIUM_ISO_BRIGHTNESS )	{
			for ( i = 0; i < max_light_level; i++ )	{
				if ( lux == light_map[i] )	{
					level = i;
					level = i/2 + 6;
					printk("[als][camera mode] Get Lux level : %d, Set : %d \n", i, level);
					lux = light_map[level];
					break;
				}
			}
		}
	}
	if ( camera_autobrightness_mode > NORMAL_AUTO_MODE )
		printk("[als][camera mode] ******* report lux = %d\n", lux);
*/
//........................................................................................//

	if (master_input_dev != NULL)	{
		input_report_abs(master_input_dev, ABS_MISC, lux);
		input_event(master_input_dev, EV_SYN, SYN_REPORT, 1);
		input_sync(master_input_dev);
	}
	else
		printk("[als] Without register input_dev for lightsensoer!!!\n");
		
}    
EXPORT_SYMBOL(als_lux_report_event);

/////////////////////////////////////////////////////////////////////////////////
// Sensoers kernel note port
//
struct class *g_classProximity = NULL;
static struct device_attribute proximity_attrs[];

int g_HAL_als_switch_on = 0;     //	For Lightsensor turn on/off global flag

static int g_nDevMajor = 0;
static atomic_t g_nDevCount;

static LIST_HEAD(g_proximity_class_list);
static DEFINE_MUTEX(g_mutexProximity);

static int create_proximity_class(void);

int old_sensors_id[SENSORS_MASK] = {
	DEFAULT_ID,
	(1 << 1),
	(1 << 3),
	(1 << 0),
	DEFAULT_ID,
	(1 << 4),
	DEFAULT_ID,
	(1 << 2),
	(1 << 5),
	(1 << 6),
	(1 << 7),
};


#define PROXIMITY_ATTR_RO(_name)						\
{									\
	.attr = { .name = #_name, .mode = 0444 },	\
	.show = proximity_show_property,					\
	.store = NULL,							\
}

#define PROXIMITY_ATTR(_name)						\
{									\
	.attr = { .name = #_name, .mode = 0664 },	\
	.show = proximity_show_property,					\
	.store = proximity_store_property,				\
}

static ssize_t proximity_show_property(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	ssize_t ret;
	struct proximity_class_dev *prxdev = dev_get_drvdata(dev);
	const ptrdiff_t off = attr - proximity_attrs;
	union proximity_propval value;

	ret = prxdev->get_property(prxdev, off, &value);
	if (ret < 0) {
		if (ret != -ENODEV)
			dev_err(dev, "driver failed to report `%s' property\n",
				attr->attr.name);
		return ret;
	}

	if (off <= SENSORS_PROP_DBG)
		return sprintf(buf, "%d", value.intval);
	else if (off == SENSORS_PROP_SWITCH)
		return sprintf(buf, "%s", (value.intval ? "on" : "off"));
	else
		return sprintf(buf, "%s", value.strval);
}

static ssize_t proximity_store_property(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	int ret;
	struct proximity_class_dev *prxdev = dev_get_drvdata(dev);
	const ptrdiff_t off = attr - proximity_attrs;
	union proximity_propval value;
	
	/* Add SENSORS_PROP_DBG for debug only */
	//if (off <= SENSORS_PROP_CURRENT)
	if (off <= SENSORS_PROP_DBG)
		value.intval = simple_strtol(buf,NULL,10);
	else if (off == SENSORS_PROP_SWITCH) {
		if (0 == strncmp(buf, "off", 3))
			value.intval = 0;
		else if (0 == strncmp(buf, "on", 2)) 
			value.intval = 1;
		else
			return -EINVAL;
	}else
		memcpy(value.strval, buf, 
		   sizeof(value.strval) > count ? count : sizeof(value.strval));

	return (ret = prxdev->put_property(prxdev, off, &value)) 
			? ret : count;
}

static struct device_attribute proximity_attrs[] = {
	PROXIMITY_ATTR(interval),
	PROXIMITY_ATTR(high_threshold),
	PROXIMITY_ATTR(low_threshold),
	PROXIMITY_ATTR(mode),
	PROXIMITY_ATTR(teststep),
	PROXIMITY_ATTR_RO(maxrange),
	PROXIMITY_ATTR_RO(resolution),
	PROXIMITY_ATTR_RO(version),
	PROXIMITY_ATTR_RO(current),
	PROXIMITY_ATTR(calvalue),
	PROXIMITY_ATTR_RO(adc),
	PROXIMITY_ATTR_RO(k_adc),
	PROXIMITY_ATTR_RO(lux),
	PROXIMITY_ATTR_RO(atd_status),
	PROXIMITY_ATTR_RO(atd_adc),    
	PROXIMITY_ATTR(dbg), /* Add SENSORS_PROP_DBG for debug only */
	PROXIMITY_ATTR(hi_cal),
	PROXIMITY_ATTR(low_cal),
	PROXIMITY_ATTR(200_cal),
	PROXIMITY_ATTR(1000_cal),

	PROXIMITY_ATTR(switch),
	PROXIMITY_ATTR_RO(vendor),
	PROXIMITY_ATTR_RO(adj), 	/* Adjust register value */
};	

static ssize_t proximity_class_show_oldid_attrs(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct proximity_class_dev *prxdev = dev_get_drvdata(dev);
        return sprintf(buf,"%x", old_sensors_id[prxdev->id]);
}

static ssize_t proximity_class_show_type_attrs(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct proximity_class_dev *prxdev = dev_get_drvdata(dev);
        return sprintf(buf,"%d", prxdev->id);
}

static ssize_t proximity_class_show_name_attrs(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct proximity_class_dev *prxdev = dev_get_drvdata(dev);
        return sprintf(buf,"%s\n", prxdev->name);
}

static struct device_attribute proximity_class_attrs[] = {
	__ATTR(id, 0444, proximity_class_show_oldid_attrs, NULL),
	__ATTR(type, 0444, proximity_class_show_type_attrs, NULL),
	__ATTR(name, 0444, proximity_class_show_name_attrs, NULL),
};

int proximity_dev_register(struct proximity_class_dev *prxdev)
{
	int i, j, ret = 0;
	dev_t dev;

	if (!prxdev->fops) {
		printk("proximity_dev_register: w/o fops!\n");
		return -EINVAL;
	}

	if (!g_classProximity) {
		ret = create_proximity_class();
		if (ret < 0)
			return ret;
	}

        if (IS_ERR(g_classProximity))
            return PTR_ERR(g_classProximity);

	INIT_LIST_HEAD(&prxdev->list);
	mutex_lock(&g_mutexProximity);

	prxdev->minor = atomic_read(&g_nDevCount);
	dev = MKDEV(g_nDevMajor, prxdev->minor);
	printk("[cm36283]devMajor=0x%x,minor=0x%x\n",dev,prxdev->minor);
	prxdev->dev = device_create(g_classProximity, NULL, dev, NULL, "%s", prxdev->name);
	if (IS_ERR(prxdev->dev)) {
		ret = PTR_ERR(prxdev->dev);
		goto dev_create_failed;
	}

	atomic_inc(&g_nDevCount);
	dev_set_drvdata(prxdev->dev, prxdev);

	if (!(prxdev->id < SENSORS_MASK))
		goto device_id_invalid; 

	for (i = 0; i < ARRAY_SIZE(proximity_class_attrs); i++) {
		ret = device_create_file(prxdev->dev, &proximity_class_attrs[i]);
		if (ret)
			goto base_attrs_create_failed;
	}

	for (j = 0; j < prxdev->num_properties; j++) {
		ret = device_create_file(prxdev->dev, &proximity_attrs[prxdev->properties[j]]);
		if (ret)
			goto attrs_create_failed;
	}

	list_add(&prxdev->list, &g_proximity_class_list);
	mutex_unlock(&g_mutexProximity);

	return ret;

attrs_create_failed:
	while (j--)
		device_remove_file(prxdev->dev, 
				&proximity_attrs[prxdev->properties[j]]);
base_attrs_create_failed:
	while (i--)
		device_remove_file(prxdev->dev, &proximity_class_attrs[i]);
device_id_invalid:
	device_destroy(g_classProximity, MKDEV(g_nDevMajor, prxdev->minor));
dev_create_failed:
	return ret;
}
EXPORT_SYMBOL_GPL(proximity_dev_register);

int proximity_dev_unregister(struct proximity_class_dev *prxdev)
{
	int i, j;
	if (list_empty(&prxdev->list))
		return -EINVAL;

	mutex_lock(&g_mutexProximity);
	list_del(&prxdev->list);

	for (i = 0; i < ARRAY_SIZE(proximity_class_attrs); i++)
		device_remove_file(prxdev->dev, &proximity_class_attrs[i]);

	for (j = 0; j < prxdev->num_properties; j++) {
		device_remove_file(prxdev->dev, &proximity_attrs[prxdev->properties[j]]);
	}

	dev_set_drvdata(prxdev->dev, NULL);
	device_destroy(g_classProximity, MKDEV(g_nDevMajor, prxdev->minor));

	mutex_unlock(&g_mutexProximity);
	return 0;
}
EXPORT_SYMBOL_GPL(proximity_dev_unregister);


static int proximity_class_open(struct inode * inode, struct file * file)
{
    int minor = iminor(inode);
    struct proximity_class_dev *prxdev;
    int err = -ENODEV;
    const struct file_operations *old_fops, *new_fops = NULL;

    mutex_lock(&g_mutexProximity);

    list_for_each_entry(prxdev, &g_proximity_class_list, list) {
        if (prxdev->minor == minor) {
            new_fops = fops_get(prxdev->fops);
            break;
        }
    }

    if (!new_fops) {
        mutex_unlock(&g_mutexProximity);
        request_module("char-major-%d-%d", g_nDevMajor, minor);
        mutex_lock(&g_mutexProximity);

        list_for_each_entry(prxdev, &g_proximity_class_list, list) {
            if (prxdev->minor == minor) {
                new_fops = fops_get(prxdev->fops);
                break;
            }
        }
        if (!new_fops)
            goto fail;
    }

    err = 0;
    old_fops = file->f_op;
    file->f_op = new_fops;
    if (file->f_op->open) {
        err=file->f_op->open(inode,file);
        if (err) {
            fops_put(file->f_op);
            file->f_op = fops_get(old_fops);
        }
    }
    fops_put(old_fops);
fail:
    mutex_unlock(&g_mutexProximity);
    return err;
}


static const struct file_operations proximity_class_fops = {
    .owner      = THIS_MODULE,
    .open       = proximity_class_open,
};

static int create_proximity_class(void)
{
	//if( g_ASUS_hwID < A91_SR5 )	{

		if (!g_nDevMajor) {
			g_nDevMajor = register_chrdev(0, "asus_sensors", &proximity_class_fops);
			if (g_nDevMajor < 0) {
				printk("Proximity_class: could not get major number\n");
				return g_nDevMajor;
			}
		}

		if (!g_classProximity) {
			g_classProximity = class_create(THIS_MODULE, "asus_sensors");
			if (IS_ERR(g_classProximity)) {
				return PTR_ERR(g_classProximity);
			}
			atomic_set(&g_nDevCount, 0);
		}
	//}

	return 0;
}

static int __init proximity_class_init(void)
{
	return create_proximity_class();
}

static void __exit proximity_class_exit(void)
{
	class_destroy(g_classProximity);
}

module_init(proximity_class_init);
module_exit(proximity_class_exit);

MODULE_DESCRIPTION("Proximity class driver");
MODULE_LICENSE("GPL");

