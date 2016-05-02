/* adux-1020.c - ADUX-1020 MS Gesture/Proximity sensor driver
 *
 * Copyright (C) 2013 ASUSTek Inc.
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

#include <linux/miscdevice.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/gpio.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/cpu.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/percpu.h>
#include <linux/notifier.h>
#include <linux/syscalls.h>
#include <linux/kallsyms.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/unistd.h> 
#include <asm/uaccess.h>
#include <asm/unistd.h>
#include <linux/linkage.h>
#include <linux/stringify.h>
#include <linux/errno.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/mman.h>
#include <linux/shm.h>
#include <linux/wait.h>
#include <linux/slab.h>

#include <linux/kthread.h>
#include <linux/poll.h>

#include "linux/input/cm36283.h"
#include "linux/input/proximity_class.h"
#include <linux/of_gpio.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h> 
#endif
#include <linux/pm.h>

#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>

#define adux1020_GPIO_PROXIMITY_INT			74
#define adux1020_GPIO_PROXIMITY_PWR_EN		3

#define ADUX1020_NAME							"ADUX1020"

struct adux1020_data {
	struct i2c_client				client ;
	struct input_dev				*input_dev;     /* Pointer to input device */
	struct delayed_work			i2c_poll_work;
	unsigned int					poll_interval_ms;   /* I2C polling period */
	unsigned int					event_threshold;    /* Change reqd to gen event */
	unsigned int					open_count;     /* Reference count */
	char							polling;            /* Polling flag */
};

struct adux1020_data  *g_adux1020_data;

struct i2c_client *adux1020_client = NULL;

static int g_proxm_dbg = 0;				/* Add for debug only */
static int g_interval = 10;

static int adux1020_proxmdev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int adux1020_proxmdev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int __devinit adux1020_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int adux1020_remove(struct i2c_client *client);
static int adux1020_turn_onoff_proxm(bool bOn);
static int ps_atd_get_status_and_adc(int *ps_status, int *ps_adc);
static int get_ps_adc_from_adux1020(void);

/////////////////////////////////////////////////////////////////////////////////////////
//---proximity sensor part---
//
static int g_ps_threshold_lo = DEFAULT_PS_THRESHOLD_lo;
static int g_ps_threshold_hi = DEFAULT_PS_THRESHOLD_hi;
static int g_proxm_switch_on			 = 0;		/*For check proxinity function on/off state*/
//static bool adux1020_proxim_state		 = 0;		/*For check proxinity device power on/off state*/
static bool pad_proxm_switch_on		 = 0;		/*For phone call on in pad issue*/

static int g_psData = 0;					//proximity data

/* adux1020 Proximity Register */
#define ADUX1020_TEST_PD			0x32
#define ADUX1020_OP_MODE			0x45	//0X000F
#define ADUX1020_INT_MASK			0x48	//0X00FF
#define ADUX1020_INT_STATUS		0x49
#define ADUX1020_READ_ADC			0x60
#define ADUX1020_READ_X1			0x68
#define ADUX1020_READ_Y1			0x69
#define ADUX1020_READ_X2			0x6A
#define ADUX1020_READ_Y2			0x6B

static enum proximity_property adux1020_proxmdev_properties[] = {
    /* int */
    SENSORS_PROP_INTERVAL,
    SENSORS_PROP_HI_THRESHOLD,
    SENSORS_PROP_LO_THRESHOLD,
    SENSORS_PROP_MAXRANGE,      /* read only */
    SENSORS_PROP_RESOLUTION,    /* read only */
    SENSORS_PROP_VERSION,       /* read only */
    SENSORS_PROP_CURRENT,       /* read only */
    SENSORS_PROP_DBG, /* Add for debug only */
    /* char */
    SENSORS_PROP_SWITCH,
    SENSORS_PROP_VENDOR,        /* read only */
    SENSORS_PROP_ADC,           /* adc raw data */
    SENSORS_PROP_ATD_STATUS     /* for atd mode only */
};

atomic_t adux1020_test_proxm_update;

static int proxmdev_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	printk("[adux1020][ps] proxmdev_dev_open.\n");

	if (file->f_flags & O_NONBLOCK)
		printk("[adux1020][ps] proxmdl_dev_open (O_NONBLOCK)\n");

	atomic_set(&adux1020_test_proxm_update, 0); //initialize atomic.

	ret = nonseekable_open(inode, file);

	return ret;
}

static struct file_operations proxmdev_fops = {
    .owner = THIS_MODULE,
    .open = proxmdev_open,
};

struct proximity_class_dev adux1020_test_proxmDev = {
	.id = SENSORS_PROXIMITY,
	.name = "psensor",
	.num_properties = ARRAY_SIZE(adux1020_proxmdev_properties),
	.properties = adux1020_proxmdev_properties,
	.get_property = adux1020_proxmdev_get_property,
	.put_property = adux1020_proxmdev_put_property,
	.fops = &proxmdev_fops
};

static int adux1020_proxmdev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	switch( property ) 
	{
		case SENSORS_PROP_HI_THRESHOLD:
			printk(DBGMSK_PRX_G4"[adux1020][ps] SENSORS_PROP_HI_THRESHOLD. (%d)\n", g_ps_threshold_hi);
			val->intval = g_ps_threshold_hi;
			break;

		case SENSORS_PROP_LO_THRESHOLD:
			printk(DBGMSK_PRX_G4"[adux1020][ps] SENSORS_PROP_LO_THRESHOLD. (%d)\n", g_ps_threshold_lo);
			val->intval = g_ps_threshold_lo;
			break;

		case SENSORS_PROP_INTERVAL:
			printk(DBGMSK_PRX_G4"[adux1020][ps] SENSORS_PROP_INTERVAL.\n");
			val->intval = g_interval;
			break;

		case SENSORS_PROP_MAXRANGE:
			printk(DBGMSK_PRX_G4"[adux1020][ps] SENSORS_PROP_MAXRANGE.\n");
			val->intval = 1; //keep it 1.0 for OMS.
			break;

		case SENSORS_PROP_RESOLUTION:
			printk(DBGMSK_PRX_G4"[adux1020][ps] SENSORS_PROP_RESOLUTION.\n");
			val->intval = 1;
			break;

		case SENSORS_PROP_VERSION:
			printk(DBGMSK_PRX_G4"[adux1020][ps] SENSORS_PROP_VERSION.\n");
			sprintf(val->strval, "Ver 0");
			break;

		case SENSORS_PROP_CURRENT:
			printk(DBGMSK_PRX_G4"[adux1020][ps] SENSORS_PROP_CURRENT.\n");
			val->intval = 30;
			break;

		case SENSORS_PROP_SWITCH:
			printk(DBGMSK_PRX_G4"[adux1020][ps] get switch = %d.\n", g_proxm_switch_on);
			val->intval = g_proxm_switch_on;
			break;

		case SENSORS_PROP_VENDOR:
			printk(DBGMSK_PRX_G4"[adux1020][ps] SENSORS_PROP_VENDOR.\n");
			sprintf(val->strval, "CAPELLA");
			break;

		/* Add for debug only */
		case SENSORS_PROP_DBG:
			printk(DBGMSK_PRX_G4"[adux1020][ps] dbg = %d.\n", g_proxm_dbg);
			val->intval = g_proxm_dbg;
			break;

		case SENSORS_PROP_ADC:
			{
				g_psData = get_ps_adc_from_adux1020();
				val->intval = g_psData;
				printk(DBGMSK_PRX_G4"[adux1020][ps] get adc property: %d\n", val->intval);
				break;
			}

		case SENSORS_PROP_ATD_STATUS:
			{
				int ps_status =0, ps_adc =0;

				ps_atd_get_status_and_adc(&ps_status, &ps_adc);
				val->intval = ps_status;
				printk(DBGMSK_PRX_G3"[adux1020][ps] get atd status: %d\n", val->intval);
				break;
			}

		default:
			printk(DBGMSK_PRX_G0"[adux1020][ps] default.\n");
			return -EINVAL;
	}
	return 0;
}

static int adux1020_proxmdev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	int ret = 0;
	//int err = 0;

	switch (property) 
	{
		case SENSORS_PROP_SWITCH:
			printk(DBGMSK_PRX_G4"[adux1020][ps] put SENSORS_PROP_SWITCH (%d,%d).\n", (val->intval), g_proxm_switch_on);

			if((g_proxm_switch_on != val->intval))	{
				if(val->intval==1)	{	//turn on PS
					ret = adux1020_turn_onoff_proxm(1);
					g_proxm_switch_on = 1;
					if ( ret == 0 )	{
						//send an init value
						input_report_abs(g_adux1020_data->input_dev, ABS_DISTANCE, 1);
						input_event(g_adux1020_data->input_dev, EV_SYN, SYN_REPORT, 1);
						input_sync(g_adux1020_data->input_dev);

						g_proxm_switch_on = 1;
						pad_proxm_switch_on = 0;
						printk(DBGMSK_PRX_G4"[adux1020][ps] proximity on.\n");
					}
				}else	{	//turn off PS if val->intval==0 or other
					g_proxm_switch_on = 0;
					pad_proxm_switch_on = 0;

					// disable PS or directly Power off adux1020
					ret = adux1020_turn_onoff_proxm(0);
					printk(DBGMSK_PRX_G4"[adux1020][ps] proximity off.\n");
				}
			}
			/*
			else if (g_bIsP01Attached)	{
				printk("[adux1020][ps] Phone on in pad (%d,%d).\n", (val->intval), pad_proxm_switch_on );
				pad_proxm_switch_on = val->intval;
			}
			*/
			break;

		case SENSORS_PROP_HI_THRESHOLD:
			printk(DBGMSK_PRX_G4"[adux1020][ps] config high THRESHOLD (%d).\n", val->intval);
			if(val->intval>=0 && val->intval<=255) {
				//if(g_proxm_switch_on==1) {
				if(g_ps_threshold_hi != val->intval) {
					g_ps_threshold_hi = val->intval;
					//ret = proxm_set_hi_threshold(g_ps_threshold_hi);
				}
			}
			else
				printk(DBGMSK_PRX_G0"[adux1020][ps] ERROR!!! OUT OF THRESHOLD (0~255)!!! \n");
			break;

		case SENSORS_PROP_LO_THRESHOLD:
			printk(DBGMSK_PRX_G4"[adux1020][ps] config low THRESHOLD (%d).\n", val->intval);
			if(val->intval>=0 && val->intval<=255) {
				//if(g_proxm_switch_on==1) {
				if(g_ps_threshold_lo != val->intval) {
					g_ps_threshold_lo = val->intval;
					//ret = proxm_set_lo_threshold(g_ps_threshold_lo);
				}
			}
			else
				printk(DBGMSK_PRX_G0"[adux1020][ps] ERROR!!! OUT OF THRESHOLD (0~255)!!! \n");
			break;

		case SENSORS_PROP_INTERVAL:
			if(1) {
				printk(DBGMSK_PRX_G4"[adux1020][ps] set interval (0x%x)\n", val->intval);
				//interval = val->intval;
			}
			else {
				printk(DBGMSK_PRX_G4"[adux1020][ps] config IT_PS (0x%x)\n", val->intval);
#if 0
				if( (val->intval>=0) && (val->intval<=3) ) {
					gpio_set_value(g_proxm_pwr_pin, 0);
					msleep(1);
					gpio_set_value(g_proxm_pwr_pin, 1);
					gpio_free(g_proxm_pwr_pin);
					ps_IT = val->intval;
					ret = adux1020_reset();
				}
#endif				
			}
			break;

		/* Add for debug only */
		case SENSORS_PROP_DBG:
			g_proxm_dbg = val->intval;
#if 0
			if ( g_proxm_dbg == 1 && g_proxm_switch_on == 1)
				queue_delayed_work(Proximity_test_wq, &Proximity_test_work, HZ * PROXIMITY_TEST_DELAY);
			else if ( g_proxm_dbg == 2 && g_proxm_switch_on == 1)
				queue_delayed_work(Proximity_test_wq, &Proximity_test_work, HZ * PROXIMITY_TEST_DELAY);
			f ( g_proxm_dbg == 0 )
				cancel_delayed_work(&Proximity_test_work);
#endif
			printk(DBGMSK_PRX_G4"[adux1020][ps] dbg = %d.\n", g_proxm_dbg);
			break;

		default:
			printk(DBGMSK_PRX_G0"[adux1020][ps] put default.\n");
			return -EINVAL;
	}
	return 0;
}

/*.........................................adux1020 attribute.........................................*/
/**
 * adux1020_als_read_reg - read data from adux1020
 *
 * Returns negative errno, else the number of messages executed.
 */
static int adux1020_als_read_reg( u8 reg, int len, void *data )
{
	int err = 0;
	struct i2c_msg msg[] = {
		{
			.addr = adux1020_client->addr,
			.flags = 0, //write
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = adux1020_client->addr,
			.flags = I2C_M_RD, //read
			.len = len,
			.buf = data,
		}
	};

	if (!adux1020_client->adapter)
		return -ENODEV;

	memset(&data, 0, sizeof(data));

	err = i2c_transfer(adux1020_client->adapter, msg, ARRAY_SIZE(msg));

	if(err < 0)	{
		printk(DBGMSK_PRX_G0"[adux1020] adux1020_als_read_reg err %d\n", err);
		return err;
	}else
		return 0;
}

/**
 * adux1020_als_write_reg - write an I2C message to adux1020
 *
 * Returns negative errno, else the number of messages executed.
 */
static int adux1020_als_write_reg( u8 reg, int data_l, int data_h )
{
	int err = 0;
	uint8_t buf[3];

	static struct i2c_msg msg;

	msg.addr = adux1020_client->addr;
	msg.flags = 0; //write
	msg.len = 3;
	msg.buf = buf;

	if (!adux1020_client->adapter)
		return -ENODEV;

	buf[0] = reg;

	memcpy(buf + 1, &data_l, sizeof(data_l));
	memcpy(buf + 2, &data_h, sizeof(data_h));

	err = i2c_transfer(adux1020_client->adapter, &msg, 1);

	if(err < 0)	{
		printk(DBGMSK_PRX_G0"[adux1020] adux1020_als_write_reg err: reg=0x%x, data_l=%d, data_h=%d, err = 0x%x\n", reg, data_l, data_h, err);
		return err;
	}else
		return 0;
	//	printk("[adux1020] reg=0x%x, data_l=%d, data_h=%d\n", reg, data_l, data_h);

}

static int adux1020_turn_onoff_proxm(bool bOn)
{
	int err = 0;
	uint16_t data_16 = 0;
	
	if(bOn == 1)	{	//power on
		printk(DBGMSK_PRX_G4"[adux1020][ps] sensor switch, turn on proximity sensor ++.\n");

		/* Set INT_MASK */
		data_16 = 0xFF00;
		err = adux1020_als_write_reg( ADUX1020_INT_MASK, data_16 & 0xFF , data_16 >> 8 );

		/* Set OP_MODE */
		data_16 = 0x0F00;
		err = adux1020_als_write_reg( ADUX1020_OP_MODE, data_16 & 0xFF , data_16 >> 8 );

		/* Set INT_STATUS */
		data_16 = 0xFF80;
		err = adux1020_als_write_reg( ADUX1020_INT_STATUS, data_16 & 0xFF , data_16 >> 8 );

		/* Set OP_MODE */
		data_16 = 0x1F00;
		err = adux1020_als_write_reg( ADUX1020_OP_MODE, data_16 & 0xFF , data_16 >> 8 );

		/* Set INT_MASK */
		data_16 = 0xF000;
		err = adux1020_als_write_reg( ADUX1020_INT_MASK, data_16 & 0xFF , data_16 >> 8 );

		/* Set OP_MODE */
		data_16 = 0x1100;
		err = adux1020_als_write_reg( ADUX1020_OP_MODE, data_16 & 0xFF , data_16 >> 8 );

		printk(DBGMSK_PRX_G4"[adux1020][ps] sensor switch, turn on proximity sensor --.\n");
	}
	else		//power off
	{
		printk(DBGMSK_PRX_G4"[adux1020][ps] turn off proximity sensor ++.\n");

		printk(DBGMSK_PRX_G4"[adux1020][ps] turn off proximity sensor --.\n");
	}
	return err;
}

static int ps_atd_get_status_and_adc(int *ps_status, int *ps_adc)
{
	int adc = 0;
	int status = 0;
	status = adux1020_turn_onoff_proxm(1);
	if (status == 0)	{
		adc = get_ps_adc_from_adux1020();
		status = adux1020_turn_onoff_proxm(g_proxm_switch_on);
	}
	if(status == 0)	{
		*ps_status = 1;
		*ps_adc = adc;
	}else	{
		*ps_status = 0;
		*ps_adc = 0;
	}
	printk(DBGMSK_PRX_G2"[adux1020][ps][ATD] Status:%d, adc:%d\n", *ps_status, *ps_adc);
	return status;
}

static int get_ps_adc_from_adux1020(void)
{
	int err = 0;
	int adc = 0;
	uint16_t data_16 = 0;
	uint8_t buff[2] = {0,0};

	/* Dead INT_STATUS */
	err = adux1020_als_read_reg( ADUX1020_INT_STATUS, 2, &buff);
	err = adux1020_als_read_reg( ADUX1020_INT_STATUS, 2, &buff);

	/* Set INT_STATUS */
	data_16 = 0x0300;
	err = adux1020_als_write_reg( ADUX1020_INT_STATUS, data_16 & 0xFF , data_16 >> 8 );

	/* Dead INT_STATUS */
	err = adux1020_als_read_reg( ADUX1020_INT_STATUS, 2, &buff);
	err = adux1020_als_read_reg( ADUX1020_INT_STATUS, 2, &buff);

	/* Set INT_STATUS */
	data_16 = 0x0302;
	err = adux1020_als_write_reg( ADUX1020_INT_STATUS, data_16 & 0xFF , data_16 >> 8 );

	/* Set TEST_PD */
	data_16 = 0x4F0F;
	err = adux1020_als_write_reg( ADUX1020_TEST_PD, data_16 & 0xFF , data_16 >> 8 );

	/* Get adux1020 adc DATA */
	err = adux1020_als_read_reg( ADUX1020_READ_X1, 2, &buff);

	adc = (u32)((buff[1] << 8) | buff[0]);

	printk("[adux1020][ps] Get adc : %d\n", adc );
	
	/* Get adux1020 adc DATA */
	err = adux1020_als_read_reg( ADUX1020_READ_Y1, 2, &buff);

	adc = (u32)((buff[1] << 8) | buff[0]);

	printk("[adux1020][ps] Get adc : %d\n", adc );

	/* Get adux1020 adc DATA */
	err = adux1020_als_read_reg( ADUX1020_READ_X2, 2, &buff);

	adc = (u32)((buff[1] << 8) | buff[0]);

	printk("[adux1020][ps] Get adc : %d\n", adc );

	/* Get adux1020 adc DATA */
	err = adux1020_als_read_reg( ADUX1020_READ_Y2, 2, &buff);

	adc = (u32)((buff[1] << 8) | buff[0]);

	printk("[adux1020][ps] Get adc : %d\n", adc );
	
	return adc;
}

/*.........................................I2C_driver sensoer init.........................................*/
static int adux1020_input_init(void)
{
	int ret = 0;
	struct input_dev *input_dev_ps = NULL;

	input_dev_ps = input_allocate_device();
	if (!input_dev_ps) {
		ret = -ENOMEM;
		printk("[adux1020]: Failed to allocate input_data device\n");
		goto error_1;
	}

	input_dev_ps->name = "adux1020_ps";
	input_dev_ps->id.bustype = BUS_I2C;
	input_set_capability(input_dev_ps, EV_ABS, ABS_DISTANCE);
	__set_bit(EV_ABS, input_dev_ps->evbit);
	__set_bit(ABS_DISTANCE, input_dev_ps->absbit);
	input_set_abs_params(input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);
	input_set_drvdata(input_dev_ps, g_adux1020_data);

	ret = input_register_device(input_dev_ps);
	if (ret < 0) {
		input_free_device(input_dev_ps);
		goto error_1;
	}
	g_adux1020_data->input_dev = input_dev_ps;
	g_adux1020_data->polling = 0;
	g_adux1020_data->poll_interval_ms = 100;
	g_adux1020_data->event_threshold = 1000;

error_1:
	return ret;
}

const struct i2c_device_id adux1020_id[] = {
	{"adux1020", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, adux1020_id);

static struct of_device_id adux1020_match_table[] = {
	{ .compatible = "ADI,adux1020",},
	{ },
};

static struct i2c_driver adux1020_driver = {
    .driver = {
        .name = "adux1020",
        .owner  = THIS_MODULE,
        .of_match_table = adux1020_match_table,
     },
    .probe = adux1020_probe,
    .remove = adux1020_remove,
    .id_table = adux1020_id,
};

static int __devinit adux1020_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	printk("[adux1020] adux1020_ps_conf_probe +.\n");

	if (client == NULL) {
		printk("[adux1020] Client is NUll.\n");
		ret =  -EFAULT;
		goto adux1020_ps_conf_probe_err;
	}

	if (!(g_adux1020_data = kmalloc(sizeof(struct adux1020_data), GFP_KERNEL))) {
		ret = -ENOMEM;
		goto adux1020_ps_conf_probe_err;
	}
	memset(g_adux1020_data, 0, sizeof(struct adux1020_data));

	adux1020_client = client;
	i2c_set_clientdata(adux1020_client, g_adux1020_data);
	adux1020_client->driver = &adux1020_driver;
	adux1020_client->flags = 1;
	strlcpy(adux1020_client->name, ADUX1020_NAME, I2C_NAME_SIZE);

	printk("[adux1020] Register input device...\n");
	ret = adux1020_input_init();
	if( ret != 0 )
		goto adux1020_ps_conf_probe_err;

	ret = proximity_dev_register(&adux1020_test_proxmDev);
	if (ret)
		printk("[adux1020] proxmdl create sysfile fail.\n");

	printk("[adux1020] adux1020_ps_conf_probe -.\n");
	return 0;

adux1020_ps_conf_probe_err:
	printk("[adux1020] adux1020_ps_conf_probe - (error).\n");
    //if (g_adux1020_data_ps != NULL) {
    //  kfree(g_adux1020_data_ps);
    //}

	return ret;
}

static int adux1020_remove(struct i2c_client *client)
{
	return 0;
}

/*.........................................Platform deiver init.........................................*/
static int adux1020_platform_probe( struct platform_device *pdev )
{
	int err = 0;
	printk("[adux1020] adux1020_platform_probe ++ \n");
	err = i2c_add_driver(&adux1020_driver);
	if ( err != 0 )
		printk("[adux1020] i2c_add_driver fail, Error : %d\n",err);
	printk("[adux1020] adux1020_platform_probe -- \n");
	return 0;
}

static int __devexit adux1020_platform_remove( struct platform_device *pdev )
{
	i2c_del_driver(&adux1020_driver);
	//proximity_dev_unregister(&g_proxmDev);
	return 0;
}

static struct of_device_id adux1020_of_match[] = {
	{ .compatible = "adux1020", },
	{ },
};
MODULE_DEVICE_TABLE(of, adux1020_of_match);

#ifdef CONFIG_PM_SLEEP
static int adux1020_suspend(struct device *dev)
{
	return 0;
}

static int adux1020_resume(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(adux1020_pm_ops, adux1020_suspend, adux1020_resume );

static struct platform_driver  adux1020_platform_driver = {
	.probe 	= adux1020_platform_probe,
     	.remove   = adux1020_platform_remove,

	.driver = {
		.name = "adux1020",
		.owner = THIS_MODULE,
		.pm = &adux1020_pm_ops,		
		.of_match_table = adux1020_of_match,
	},
};

static int __init adux1020_init(void)
{
	int err = 0;
	
	printk("[adux1020] adux1020_platform_init +.\n");

	err = platform_driver_register(&adux1020_platform_driver);
	if ( err != 0 )
		printk("[adux1020] adux1020_driver_register fail, Error : %d\n",err);

	printk("[adux1020] adux1020_platform_init -.\n");
	return err;
}


static void __exit adux1020_exit(void)
{
	printk(DBGMSK_PRX_G2"[adux1020] adux1020_platform_exit +.\n");
	platform_driver_unregister(&adux1020_platform_driver);
	printk(DBGMSK_PRX_G2"[adux1020] adux1020_platform_exit -.\n");
}

module_init(adux1020_init);
module_exit(adux1020_exit);

MODULE_AUTHOR("ASUS");
MODULE_DESCRIPTION("ADI ADUX-1020 MS Gesture/Proximity sensor");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

