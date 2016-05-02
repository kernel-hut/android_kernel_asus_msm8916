/* cm32180.c - CM32180 proximity/light sensor driver

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

#include "linux/input/al3010.h"
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>

#define CM32180_ALS_CONF_NAME			"cm32180_als_conf"
#define CM32180_ARA_NAME					"cm32180_ara"

#define CM32180_GPIO_PROXIMITY_INT			65
#define CM32180_GPIO_PROXIMITY_PWR_EN		3


struct i2c_client *cm32180_als_client = NULL;
struct i2c_client *cm32180_ara_client = NULL;

static struct workqueue_struct *cm32180_light_interrupt_workqueue;
static struct workqueue_struct *cm32180_light_debounce_workqueue;
static struct delayed_work cm32180_light_interrupt_delay_work;
static struct delayed_work cm32180_light_debounce_work;

struct cm32180_data {
	struct i2c_client		client ;
	struct input_dev		*input_dev;		// Pointer to input device

	unsigned int g_als_threshold_lo;		// Lightsensor setting low threshold(adc)
	unsigned int g_als_threshold_hi;		// Lightsensor setting high threshold(adc)
	u16 cm32180_light_adc;				// Original light adc value
	u16 cm32180_light_k_adc;			// The light adc value after calibration
	u16 g_last_cm32180_light;			// light Lux after filter
	u16 cm32180_light_lux;				// Final light Lux
	
	bool g_cm32180_HAL_als_switch_on;	//this var. means if HAL is turning on als or not
	bool g_cm32180_als_switch_on;		//this var. means if cm32180 als hw is turn on or not


	unsigned int g_ambient_dbg;			// Add for debug only
	unsigned int g_interval;

	struct delayed_work			i2c_poll_work;
	unsigned int					poll_interval_ms;   /* I2C polling period */
	unsigned int					event_threshold;    /* Change reqd to gen event */
	unsigned int					open_count;     /* Reference count */
	char							polling;            /* Polling flag */
};

extern int g_HAL_als_switch_on;		// For all lightsensor trun on/off global flag

static struct cm32180_data  *g_cm32180_data_as;

struct _cm32180_device {
	unsigned int cm32180_irq_gpio;		// Lightsensor gpio interrupt pin
	unsigned int irq;
}g_cm32180_device;

static int cm32180_max_light_level = 12;
static int cm32180_light_map[12] = {0,50,100,200,300,450,700,850,1050,1250,1500,2200} ;

//atomic_t ambient_update;

static int cm32180_ambientDev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int cm32180_ambientDev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int cm32180_ambientDev_open(struct inode *inode, struct file *file);
static int cm32180_als_conf_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int cm32180_ara_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int als_atd_get_status_and_adc(int *als_status, int *als_adc);
static int get_adc_calibrated_lux_from_cm32180(void);
static int get_als_adc_from_cm32180(void);
static int cm32180_turn_onoff_als(bool bOn);

/////////////////////////////////////////////////////////////////////////////////
// Pad mode reltaed
//
extern bool g_bIsP01Attached;

static struct work_struct cm32180_attached_Pad_work;

static void cm32180_lightsensor_attached_pad(struct work_struct *work)
{
	printk(DBGMSK_PRX_G2"[cm32180_als] lightsensor_attached_pad()++\n");

	if (g_HAL_als_switch_on)	{
		printk(DBGMSK_PRX_G2"[cm32180_als] lightsensor_attached_pad, checking if P01 is attached\n");
		printk(DBGMSK_PRX_G2"[cm32180_als] lightsensor_attached_pad, attached! switch to al3010 : %d lux\n"
						, g_cm32180_data_as->cm32180_light_lux);

		/*On going*/

		/*shut down cm32180_als*/
		cm32180_turn_onoff_als(0);
	}
	printk(DBGMSK_PRX_G2"[cm32180_als] lightsensor_attached_pad()--\n");
	return;
}

int cm32180_lightsensor_detached_pad(void)
{
	printk(DBGMSK_PRX_G2"[cm32180_als] cm32180_detached_pad()++\n");

	//if HAL still turned on the als, we switch back to cm32180
	if(g_HAL_als_switch_on)	{
		/* On going*/
		/*
		printk(DBGMSK_PRX_G2"[cm32180_als] lightsensor_detached_pad(), switch back to cm32180 : %d lux\n", g_cm32180_light);
		als_lux_report_event( g_cm32180_light);
		*/
		
		//turn on cm32180_als
		cm32180_turn_onoff_als(1);
	}
	else	{
		if ( g_cm32180_data_as->g_cm32180_als_switch_on )
			cm32180_turn_onoff_als(0);
		printk(DBGMSK_PRX_G2"[cm32180_als] lightsensor_detached_pad(), als is turned off\n");
	}
		
	printk(DBGMSK_PRX_G2"[cm32180_als] cm32180_detached_pad()--\n");
	return 0;
}

static int cm32180_lightsensor_mp_event(struct notifier_block *this, unsigned long event, void *ptr)
{

    switch (event) {
        case P01_ADD:
            printk("[cm32180_als][MicroP] Pad_ADD \r\n");
            queue_work(cm32180_light_interrupt_workqueue, &cm32180_attached_Pad_work);
            return NOTIFY_DONE;

        case P01_REMOVE:
            printk("[cm32180_als][MicroP] Pad_REMOVE \r\n");
            cm32180_lightsensor_detached_pad();
            return NOTIFY_DONE;

        default:
            return NOTIFY_DONE;
    }
}

static struct notifier_block cm32180_lightsensor_mp_notifier = {
        .notifier_call = cm32180_lightsensor_mp_event,
        .priority = CM36283_LIGHTSENSOR_MP_NOTIFY,
};

/////////////////////////////////////////////////////////////////////////////////
// CM32180 calibration port
//
//#define LSENSOR_CALIBRATION_ASUS_NV_FILE			"/data/asusdata/lsensor.nv"
//#define LSENSOR_CALIBRATION_SHIFT_ASUS_NV_FILE	"/data/asusdata/lsensor_shift.nv"

static u32 cm32180_light_calibration			= 10;
static u32 cm32180_light_shift_calibration		= 0;

static bool read_lightsensor_calibrationvalue(void)
{
	struct file *fp = NULL; 
	loff_t pos_lsts = 0;
	char readstr[8];
	int ori_val = 0, readlen =0;
	mm_segment_t old_fs;
	
	printk(DBGMSK_PRX_G3"[als] ++read_lsensor_calvalue open\n");

	fp = filp_open(LSENSOR_CALIBRATION_ASUS_NV_FILE,
						O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G3"[als] read_lsensor_calvalue open (%s) fail\n",
						LSENSOR_CALIBRATION_ASUS_NV_FILE);
		return false;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	
	if(fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, readstr, 6, &pos_lsts);
		readstr[readlen] = '\0';

		printk(DBGMSK_PRX_G3"[als] strlen:%s(%d)\n", readstr, strlen(readstr));
	}
	else
		printk("[als] read_lsensor_calvalue, f_op=NULL or op->read=NULL\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(readstr, "%d", &ori_val);

	//limit the calibration value range
	cm32180_light_calibration = ori_val;

	printk("[als] read_lsensor_calvalue: Ori: %d, Cal: %d\n", 
			ori_val, cm32180_light_calibration);

	printk("[als] --read_lsensor_calvalue open\n");
	return true;
}

static bool read_lightsensor_shiftvalue(void)
{
	struct file *fp = NULL; 
	loff_t pos_lsts = 0;
	char readstr[8];
	int ori_val = 0, readlen =0;
	mm_segment_t old_fs;

	printk(DBGMSK_PRX_G3"[als] ++read_lsensor_shift open\n");

	fp = filp_open(LSENSOR_CALIBRATION_SHIFT_ASUS_NV_FILE,
						O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);

	if(IS_ERR_OR_NULL(fp))	{
		printk(DBGMSK_PRX_G3"[als] read_lsensor_shift open (%s) fail\n",
						LSENSOR_CALIBRATION_SHIFT_ASUS_NV_FILE);
		return false;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if(fp->f_op != NULL && fp->f_op->read != NULL)	{
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, readstr, 6, &pos_lsts);
		readstr[readlen] = '\0';

		printk(DBGMSK_PRX_G3"[als] strlen:%s(%d)\n", readstr, strlen(readstr));
	}
	else
		printk(DBGMSK_PRX_G0"[als] read_lsensor_shift, f_op=NULL or op->read=NULL\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(readstr, "%d", &ori_val);

	//limit the calibration value range
	cm32180_light_shift_calibration = ori_val;

	printk("[als] read_lsensor_shift: Ori: %d, Cal: %d\n",
				ori_val,  cm32180_light_shift_calibration);

	printk("[als] --read_lsensor_calvalue open\n");
	return true;
}


/////////////////////////////////////////////////////////////////////////////////
//---ambient light sensor part---
//
/* CM32180 Als Register */
#define CM32180_ALS_CONF			0x00    //Setting cmd (integration time, interrupt setting and enable/disable, persistence) for als sensor.
#define CM32180_ALS_WH				0x01    //High interrupt threshold window setting
#define CM32180_ALS_WL				0x02    //Low interrupt threshold window setting
#define CM32180_ALS_DATA			0x04    //Get als data

/* CM32180 Als command */
/* For_ALS_CONF */
static u8 ALS_SM			 = 0x00; // ALS sensitivity setting					, bit 12,11	( 00:sensitivity x 1 , 01:x 2, 10:x 0.5 )
static u8 ALS_IT			 = 0x03;	// ALs intrgration time setting				, bit 7,6	( 2T, 500ms )
static u8 ALS_PERS		 = 0x03; // ALS interrupt persistence setting			, bit 5,4	( 8 times )
static u8 ALS_INT_EN_OFF	 = 0x00; // ALS interrupt enable/disable				, bit 1	( disable )
static u8 ALS_SD_OFF		 = 0x01; // ALS shut down setting					, bit 0	( disable )
static u8 ALS_INT_EN_ON	 = 0x01; // ALS interrupt enable					, bit 1	( enable )
static u8 ALS_SD_ON		 = 0x00; // ALS shut down setting					, bit 0	( enable )

static enum proximity_property cm32180_ambientDev_properties[] = {
    /* int */
    SENSORS_PROP_INTERVAL,
    SENSORS_PROP_HI_THRESHOLD,
    SENSORS_PROP_LO_THRESHOLD,
    SENSORS_PROP_MAXRANGE,      /* read only */
    SENSORS_PROP_RESOLUTION,    /* read only */
    SENSORS_PROP_VERSION,       /* read only */
    SENSORS_PROP_CURRENT,       /* read only */
    SENSORS_PROP_DBG,           /* Add for debug only */
    /* char */
    SENSORS_PROP_SWITCH,
    SENSORS_PROP_VENDOR,        /* read only */
    SENSORS_PROP_CALIBRATION,   /* Old_calibration value */
    SENSORS_PROP_ADC,           /* adc raw data */
    SENSORS_PROP_K_ADC,         /* adc raw data w/ calibrated */
    SENSORS_PROP_LUX,            /* lux data (calibrated) */
    SENSORS_PROP_ATD_STATUS,    /* for atd mode only */
    SENSORS_PROP_ATD_ADC,        /* for atd mode only */
};

static struct file_operations cm32180_ambientDev_fops = {
    .owner = THIS_MODULE,
    .open = cm32180_ambientDev_open,
};

struct proximity_class_dev cm32180_ambientDev = {
    .id = SENSORS_LIGHT,
    .name = "lsensor",
    .num_properties = ARRAY_SIZE(cm32180_ambientDev_properties),
    .properties = cm32180_ambientDev_properties,
    .get_property = cm32180_ambientDev_get_property,
    .put_property = cm32180_ambientDev_put_property,
    .fops = &cm32180_ambientDev_fops
};

static int cm32180_ambientDev_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	printk(DBGMSK_PRX_G3"[cm32180][als] ambientdl_dev_open \n");

	if (file->f_flags & O_NONBLOCK)
		printk(DBGMSK_PRX_G2"[cm32180][als] ambientdl_dev_open (O_NONBLOCK)\n");

//	atomic_set(&ambient_update, 0); //initialize atomic.

	ret = nonseekable_open(inode, file);
	return ret;
}

static int cm32180_ambientDev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	printk(DBGMSK_PRX_G3"[cm32180][als] ambientdl_get_property +.\n");

	switch( property ) 
	{
		case SENSORS_PROP_HI_THRESHOLD:
			printk(DBGMSK_PRX_G3"[cm32180][als] SENSORS_PROP_HI_THRESHOLD. (%d)\n",
						g_cm32180_data_as->g_als_threshold_hi);
			val->intval = g_cm32180_data_as->g_als_threshold_hi;
			break;

		case SENSORS_PROP_LO_THRESHOLD:
			printk(DBGMSK_PRX_G3"[cm32180][als] SENSORS_PROP_LO_THRESHOLD. (%d)\n",
						g_cm32180_data_as->g_als_threshold_lo);
			val->intval = g_cm32180_data_as->g_als_threshold_lo;
			break;

		case SENSORS_PROP_INTERVAL:
			printk(DBGMSK_PRX_G3"[cm32180][als] config GAIN_ALS by using \"echo XXX > /sys/class/sensors/sensor4/interval\" XXX is only 0~3. \n");
			printk(DBGMSK_PRX_G3"[cm32180][als] SENSORS_PROP_INTERVAL.\n");
			val->intval = g_cm32180_data_as->g_interval;
			break;

		case SENSORS_PROP_MAXRANGE:
			printk(DBGMSK_PRX_G3"[cm32180][als] SENSORS_PROP_MAXRANGE.\n");
			val->intval = 128;  
			break;

		case SENSORS_PROP_RESOLUTION:
			printk(DBGMSK_PRX_G3"[cm32180][als] SENSORS_PROP_RESOLUTION.\n");
			val->intval = 1;
			break;

		case SENSORS_PROP_VERSION:
			printk(DBGMSK_PRX_G3"[cm32180][als] SENSORS_PROP_VERSION.\n");
			val->intval = 1;
			break;

		case SENSORS_PROP_CURRENT:
			printk(DBGMSK_PRX_G3"[cm32180][als] SENSORS_PROP_CURRENT.\n");
			val->intval = 30;   
			break;

		case SENSORS_PROP_SWITCH:
			printk(DBGMSK_PRX_G3"[cm32180][als] get switch = %d.\n", g_cm32180_data_as->g_cm32180_HAL_als_switch_on);
			val->intval = g_cm32180_data_as->g_cm32180_HAL_als_switch_on;
			break;

		case SENSORS_PROP_VENDOR:
			printk(DBGMSK_PRX_G3"[cm32180][als] SENSORS_PROP_VENDOR.\n");
			sprintf(val->strval, "CAPELLA");
			break;

		/* Add for debug only */
		case SENSORS_PROP_DBG:
			val->intval = g_cm32180_data_as->g_ambient_dbg;
			printk(DBGMSK_PRX_G3"[cm32180][als] dbg = %d.\n", g_cm32180_data_as->g_ambient_dbg);
			break;
			
		case SENSORS_PROP_ADC:
			g_cm32180_data_as->cm32180_light_adc =  get_als_adc_from_cm32180();
			val->intval = g_cm32180_data_as->cm32180_light_adc;
			printk(DBGMSK_PRX_G3"[cm32180][als] get adc property: %d\n", val->intval);
			break;

		case SENSORS_PROP_K_ADC:
			val->intval = g_cm32180_data_as->cm32180_light_k_adc;
			printk(DBGMSK_PRX_G3"[cm32180][als] get k_adc property: %d\n", val->intval);\
			break;

		case SENSORS_PROP_LUX:
			val->intval = g_cm32180_data_as->cm32180_light_lux;
			printk(DBGMSK_PRX_G3"[cm32180][als] get lux property: %d\n", val->intval);
			break;
			
		case SENSORS_PROP_ATD_STATUS:
		{
			int als_status =0, als_adc =0;
			
			als_atd_get_status_and_adc(&als_status, &als_adc);
			val->intval = als_status;
			printk(DBGMSK_PRX_G3"[cm32180][als] get atd status: %d\n", val->intval);
			break;
		}

		case SENSORS_PROP_ATD_ADC:
		{
			int als_status =0, als_adc =0;

			als_atd_get_status_and_adc(&als_status, &als_adc);
			val->intval = als_adc;
			printk(DBGMSK_PRX_G3"[cm32180][als] get atd adc: %d\n", val->intval);
			break;
		}
		default:
			printk(DBGMSK_PRX_G0"[cm32180][als] default\n");
			return -EINVAL;
		}

	printk( DBGMSK_PRX_G3"[cm32180]: ambientdl_get_property -.\n");

	return 0;
}

static int cm32180_ambientDev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	static bool bFirst = true;
	static bool openfilp = true;

	switch (property)
	{
		case SENSORS_PROP_SWITCH:
			//read calibration value
			if(bFirst) {
				printk(DBGMSK_PRX_G3"[cm32180][als] put switch 1st read calvalue\n");
				openfilp = read_lightsensor_calibrationvalue();
				/*Surpport old calibration value*/
				if ( cm32180_light_calibration >= 1024 || openfilp == false )	{
					printk("[cm32180][als] Get old calvalue : %d or read calibrationvalue fail\n",
									cm32180_light_calibration );
					cm32180_light_calibration = 10;//38;
					cm32180_light_shift_calibration = 0;//40;
				}else
					read_lightsensor_shiftvalue();
				printk("[cm32180] Set calibration and shift: %d , %d\n",
								cm32180_light_calibration,
								cm32180_light_shift_calibration );
				bFirst = false;
			}
			
			printk(DBGMSK_PRX_G3"[cm32180][als] put SENSORS_PROP_SWITCH (%d,%d).\n", 
				(val->intval), g_cm32180_data_as->g_cm32180_HAL_als_switch_on);
			if ( g_HAL_als_switch_on != val->intval )	{
				if(val->intval > 0)
					g_HAL_als_switch_on = 1;
				else
					g_HAL_als_switch_on = 0;
			}
			g_cm32180_data_as->g_cm32180_HAL_als_switch_on = g_HAL_als_switch_on;
			if ( g_bIsP01Attached )	{
				printk(DBGMSK_PRX_G3"[cm32180][als] sensor switch, turn on/off al3010: %d\n",
						g_cm32180_data_as->g_cm32180_HAL_als_switch_on);
				set_als_power_state_of_P01(g_cm32180_data_as->g_cm32180_HAL_als_switch_on);
				if ( g_cm32180_data_as->g_cm32180_als_switch_on )
					cm32180_turn_onoff_als(0);
			}
			else	{
				printk(DBGMSK_PRX_G3"[cm36283][als] sensor switch, turn on/off cm36283: %d\n",
						g_cm32180_data_as->g_cm32180_HAL_als_switch_on);
				cm32180_turn_onoff_als(g_cm32180_data_as->g_cm32180_HAL_als_switch_on);
			}
			break;

		case SENSORS_PROP_LO_THRESHOLD:
			break;

		case SENSORS_PROP_HI_THRESHOLD:
			break;

		case SENSORS_PROP_INTERVAL:
			printk(DBGMSK_PRX_G3"[cm32180][als] put SENSORS_PROP_INTERVAL. %d\n", val->intval);
			if(val->intval < 100)
				g_cm32180_data_as->g_interval = 100;
			else
				g_cm32180_data_as->g_interval = val->intval;
			break;
			
		/* Add for debug only */
		case SENSORS_PROP_DBG:
			g_cm32180_data_as->g_ambient_dbg = val->intval;
			printk(DBGMSK_PRX_G3"[cm32180][als] dbg = %d.\n", g_cm32180_data_as->g_ambient_dbg);
			break;

		default:
			printk(DBGMSK_PRX_G0"[cm32180][als] put default.\n");
			return -EINVAL;
		}
	return 0;
}

/*.........................................cm32180 attribute.........................................*/
/**
 * cm32180_als_read_reg - read data from cm32180
 *
 * Returns negative errno, else the number of messages executed.
 */
static int cm32180_als_read_reg( u8 reg, int len, void *data )
{
	int err = 0;
	struct i2c_msg msg[] = {
		{
			.addr = cm32180_als_client->addr,
			.flags = 0, //write
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = cm32180_als_client->addr,
			.flags = I2C_M_RD, //read
			.len = len,
			.buf = data,
		}
	};

	if (!cm32180_als_client->adapter)
		return -ENODEV;

	memset(&data, 0, sizeof(data));

	err = i2c_transfer(cm32180_als_client->adapter, msg, ARRAY_SIZE(msg));

	if (err != ARRAY_SIZE(msg))
		printk(DBGMSK_PRX_G0"[cm32180] cm32180_als_read_reg err %d\n", err);

	return err; // return 2 is expected.
}

/**
 * cm32180_als_write_reg - write an I2C message to cm32180
 *
 * Returns negative errno, else the number of messages executed.
 */
static int cm32180_als_write_reg( u8 reg, int data_l, int data_h )
{
	int err = 0;
	uint8_t buf[3];

	static struct i2c_msg msg;

	msg.addr = cm32180_als_client->addr;
	msg.flags = 0; //write
	msg.len = 3;
	msg.buf = buf;

	if (!cm32180_als_client->adapter)
		return -ENODEV;

	buf[0] = reg;

	memcpy(buf + 1, &data_l, sizeof(data_l));
	memcpy(buf + 2, &data_h, sizeof(data_h));

	err = i2c_transfer(cm32180_als_client->adapter, &msg, 1);

	if(err < 0)
		printk(DBGMSK_PRX_G0"[cm32180] cm32180_als_write_reg err: reg=0x%x, data_l=%d, data_h=%d, err = 0x%x\n", reg, data_l, data_h, err);
	//else
	//	printk("[cm32180] reg=0x%x, data_l=%d, data_h=%d\n", reg, data_l, data_h);

	return err; // return postive is expected.
}

/**
 * cm32180_ara_read_reg - read data from cm32180
 *
 * Returns negative errno, else the number of messages executed.
 */
static int cm32180_ara_read_reg( void *data )
{
	int err = 0;
	struct i2c_msg msg[] = {
		{
			.addr = cm32180_ara_client->addr,
			.flags = I2C_M_RD, //read
			.len = 1,
			.buf = data,
		}
	};

	if (!cm32180_ara_client->adapter)
		return -ENODEV;

	memset(&data, 0, sizeof(data));

	err = i2c_transfer(cm32180_ara_client->adapter, msg, ARRAY_SIZE(msg));

	if (err != ARRAY_SIZE(msg))
		printk(DBGMSK_PRX_G0"[cm32180] cm32180_ara_read_reg err %d\n", err);

	return err; // return 1 is expected.
}

static irqreturn_t cm32180_als_interrupt_handler(int irq, void *dev_id)
{
	printk(DBGMSK_PRX_G3"[cm32180][isr] interrupt handler ++\n");
	queue_delayed_work(cm32180_light_interrupt_workqueue, &cm32180_light_interrupt_delay_work, 0 );
	printk(DBGMSK_PRX_G3"[cm32180][isr] interrupt handler --\n");
	
	return IRQ_HANDLED;
}

static void cm32180_als_debounce_work(struct work_struct *work)
{
	int err = 0;
	uint16_t data_16 = 0;
	/* Set 0x00 ALS CONF(Turn on) */
	data_16 = ( ALS_SM<<11 | ALS_IT<<6 |ALS_PERS<<4 | ALS_INT_EN_ON<<1 | ALS_SD_ON );
	err = cm32180_als_write_reg(CM32180_ALS_CONF, data_16 & 0xFF , data_16 >> 8);
	if(err < 0)
		printk("[cm32180][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM32180_ALS_CONF, err);
	else
		printk(DBGMSK_PRX_G3"[cm32180][als] set als_config(0x%x), data=0x%x\n", CM32180_ALS_CONF, data_16);
}

static void cm32180_light_interrupt_handler(struct work_struct *work)
{
	int err = 0;
	uint8_t data_8 = 0;
	
	/* Get ara INT data*/
	err = cm32180_ara_read_reg( &data_8 );
	printk("[cm32180][als] Get ARA, data=0x%x\n", data_8);
	
	get_adc_calibrated_lux_from_cm32180();

	/* Report Lux ans light filter */
	if( g_cm32180_data_as->cm32180_light_lux != g_cm32180_data_as->g_last_cm32180_light ) {
		g_cm32180_data_as->g_last_cm32180_light = g_cm32180_data_as->cm32180_light_lux;
		als_lux_report_event( g_cm32180_data_as->cm32180_light_lux );
	}

	/* Enable interrupt and trun on sensor */
	if ( g_cm32180_data_as->g_cm32180_als_switch_on == true )
		queue_delayed_work(cm32180_light_debounce_workqueue, &cm32180_light_debounce_work, HZ);
}

static int get_als_adc_from_cm32180(void)
{
	int adc = 0;
	int lsb = 0, msb = 0;
	uint8_t buff[2] = {0,0};
	uint16_t data_16 = 0;

	/* Set 0x00 ALS CONF(Shut down) */
	data_16 = ( ALS_SM<<11 | ALS_IT<<6 |ALS_PERS<<4 | ALS_INT_EN_OFF<<1 | ALS_SD_OFF );
	adc = cm32180_als_write_reg(CM32180_ALS_CONF, data_16 & 0xFF , data_16 >> 8);
	if(adc < 0) {
		printk("[cm32180][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM32180_ALS_CONF, adc);
		return -1;
	}else
		printk(DBGMSK_PRX_G3"[cm32180][als] set als_config(0x%x), data=0x%x\n", CM32180_ALS_CONF, data_16);

	/* Get 0x04 adc value ALS DATA */
	adc = cm32180_als_read_reg(CM32180_ALS_DATA, 2, &buff);
	if(adc < 0) {
		printk("[cm32180][als] (%s): err(0x%x)=%d\n",__FUNCTION__, CM32180_ALS_DATA, adc);
		return -1;
	}else	{
		lsb = buff[0];
		msb = buff[1];
		printk(DBGMSK_PRX_G3"[cm32180][als] Read als_config(0x%x), read-LoByte (%d). read-HiByte (%d)\n",
			CM32180_ALS_DATA ,lsb , msb);
	}

	adc = (u32)((msb << 8) | lsb) ;

	/* Set 0x00 ALS CONF(Turn on) */
	data_16 = ( ALS_SM<<11 | ALS_IT<<6 |ALS_PERS<<4 | ALS_INT_EN_ON<<1 | ALS_SD_ON );
	msb = cm32180_als_write_reg(CM32180_ALS_CONF, data_16 & 0xFF , data_16 >> 8);
	if(msb < 0) {
		printk("[cm32180][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM32180_ALS_CONF, msb);
		return -1;
	}else
		printk(DBGMSK_PRX_G3"[cm32180][als] set als_config(0x%x), data=0x%x\n", CM32180_ALS_CONF, data_16);
		
	printk("[cm32180][als] Get adc : %d\n", adc );

	return adc;
}

static int get_adc_calibrated_lux_from_cm32180(void)
{
	int err = 0, i = 0;
	int adc = 0, lsb = 0, msb = 0;
	uint8_t buff[2] = {0,0};
	uint16_t data_16 = 0;
	uint16_t tmp = g_cm32180_data_as->cm32180_light_adc;		// Old adc value

	/* Set 0x00 ALS CONF(Disable sensor) */
	data_16 = ( ALS_SM<<11 | ALS_IT<<6 |ALS_PERS<<4 | ALS_INT_EN_OFF<<1 | ALS_SD_OFF );
	err = cm32180_als_write_reg(CM32180_ALS_CONF, data_16 & 0xFF , data_16 >> 8);
	if(err < 0) {
		printk("[cm32180][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM32180_ALS_CONF, err);
		return -EIO;
	}else
		printk(DBGMSK_PRX_G3"[cm32180][als] set als_config(0x%x), data=0x%x\n", CM32180_ALS_CONF, data_16);

	/* Get 0x04 adc value ALS DATA */
	adc = cm32180_als_read_reg(CM32180_ALS_DATA, 2, &buff);
	if(adc < 0) {
		printk("[cm32180][als] (%s): err(0x%x)=%d\n",__FUNCTION__, CM32180_ALS_DATA, adc);
		g_cm32180_data_as->cm32180_light_adc = tmp;
	}else	{
		lsb	= buff[0];
		msb	= buff[1];
		g_cm32180_data_as->cm32180_light_adc = (u32)((msb << 8) | lsb);
		printk(DBGMSK_PRX_G3"[cm32180][als] Read als_config(0x%x), read-LoByte (%d). read-HiByte (%d)\n",
			CM32180_ALS_DATA ,lsb , msb);
	}

	/* Calibtation adc */
	adc = g_cm32180_data_as->cm32180_light_adc;     	// adc is the value that is returned from HW directly ( Original adb number )
	g_cm32180_data_as->cm32180_light_k_adc = 
		(u32)(adc * cm32180_light_calibration / 100  + cm32180_light_shift_calibration);     //apply calibration value ( Because the panel level, calibration number probably over 10000)

	printk("[cm32180][als] read adc: 0x%X, cal adc: 0x%X, file adc: 0x%X\n", adc
						, g_cm32180_data_as->cm32180_light_k_adc
						, g_cm32180_data_as->cm32180_light_adc);

	/* Get Lux */
	for( i = 1; i < cm32180_max_light_level; i++)	{
		if( g_cm32180_data_as->cm32180_light_k_adc < cm32180_light_map[i] ) {
			g_cm32180_data_as->cm32180_light_lux = cm32180_light_map[ i -1 ];
			break;
		}
		else if( g_cm32180_data_as->cm32180_light_k_adc > cm32180_light_map[cm32180_max_light_level - 1] )	{
			g_cm32180_data_as->cm32180_light_lux = cm32180_light_map[ cm32180_max_light_level -1 ];
			break;
		}
	}
	if ( g_cm32180_data_as->cm32180_light_lux > cm32180_light_map[ cm32180_max_light_level -1 ])
		g_cm32180_data_as->cm32180_light_lux = cm32180_light_map[ cm32180_max_light_level -1 ];
	printk(DBGMSK_PRX_G3"[cm32180][als] last=%d light=%d steps=%d\n",
							g_cm32180_data_as->g_last_cm32180_light,
							g_cm32180_data_as->cm32180_light_lux, cm32180_light_map[i] );
	
	/* Set 0x01,0x02 ALS threshold */
	if( g_cm32180_data_as->cm32180_light_lux != g_cm32180_data_as->g_last_cm32180_light ) {
		g_cm32180_data_as->g_als_threshold_hi =
					(u16)(g_cm32180_data_as->cm32180_light_adc*105/100);
		g_cm32180_data_as->g_als_threshold_lo =
					(u16)(g_cm32180_data_as->cm32180_light_adc*95/100);

		err = cm32180_als_write_reg(CM32180_ALS_WH, g_cm32180_data_as->g_als_threshold_hi & 0xFF,
													g_cm32180_data_as->g_als_threshold_hi >> 8);
		if(err < 0)
			printk("[cm32180][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM32180_ALS_WH, err);
		else
			printk(DBGMSK_PRX_G3"[cm32180][als] set als_config(0x%x), data=0x%x\n", CM32180_ALS_WH,
													g_cm32180_data_as->g_als_threshold_hi);

		err = cm32180_als_write_reg(CM32180_ALS_WL, g_cm32180_data_as->g_als_threshold_lo & 0xFF, 
													g_cm32180_data_as->g_als_threshold_lo >> 8);
		if(err < 0)
			printk("[cm32180][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM32180_ALS_WL, err);
		else
			printk(DBGMSK_PRX_G3"[cm32180][als] set als_config(0x%x), data=0x%x\n", CM32180_ALS_WL, 
													g_cm32180_data_as->g_als_threshold_lo);
	}

	return g_cm32180_data_as->cm32180_light_lux;
}

static int cm32180_turn_onoff_als(bool bOn)
{
	int err = 0;
	uint8_t data_8 = 0;
	uint16_t data_16 = 0;

	printk("[cm32180][als] turn on/off light sensor ++.\n");
	if ( g_cm32180_data_as->g_cm32180_als_switch_on != bOn )	{
	
		if ( bOn == 1 )	{			//Enable Lightsensor
			/* Clear ara INT*/
			err = cm32180_ara_read_reg( &data_8 );
			printk("[cm32180][als] Clean ARA, data=0x%x\n", data_8);

			/* Set 0x00 ALS CONF(Shut down) */
			data_16 = ( ALS_SM<<11 | ALS_IT<<6 |ALS_PERS<<4 | ALS_INT_EN_OFF<<1 | ALS_SD_OFF );
			err = cm32180_als_write_reg(CM32180_ALS_CONF, data_16 & 0xFF , data_16 >> 8);
			if(err < 0) {
				printk("[cm32180][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM32180_ALS_CONF, err);
				return -1;
			}else
				printk(DBGMSK_PRX_G3"[cm32180][als] set als_config(0x%x), data=0x%x\n", CM32180_ALS_CONF, data_16);

			/* Set 0x01,0x02 ALS threshold */
			data_16 = 0x00;
			err = cm32180_als_write_reg(CM32180_ALS_WH, data_16 & 0xFF , data_16 >> 8);
			if(err < 0) {
				printk("[cm32180][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM32180_ALS_WH, err);
				return -1;
			}else
				printk(DBGMSK_PRX_G3"[cm32180][als] set als_config(0x%x), data=0x%x\n", CM32180_ALS_WH, data_16);
			
			err = cm32180_als_write_reg(CM32180_ALS_WL, data_16 & 0xFF , data_16 >> 8);
			if(err < 0) {
				printk("[cm32180][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM32180_ALS_WL, err);
				return -1;
			}else
				printk(DBGMSK_PRX_G3"[cm32180][als] set als_config(0x%x), data=0x%x\n", CM32180_ALS_WL, data_16);

			/* Set 0x00 ALS CONF(Turn on) */
			data_16 = ( ALS_SM<<11 | ALS_IT<<6 |ALS_PERS<<4 | ALS_INT_EN_ON<<1 | ALS_SD_ON );
			err = cm32180_als_write_reg(CM32180_ALS_CONF, data_16 & 0xFF , data_16 >> 8);
			if(err < 0) {
				printk("[cm32180][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM32180_ALS_CONF, err);
				return -1;
			}else
				printk(DBGMSK_PRX_G3"[cm32180][als] set als_config(0x%x), data=0x%x\n", CM32180_ALS_CONF, data_16);

			g_cm32180_data_as->g_cm32180_als_switch_on = 1;

			printk("[cm32180][als] turn on light sensor --.\n");
		}
		else	{				//Disable Lightsensor
			printk("[cm32180][als] turn off light sensor ++.\n");

			/* Set 0x00 ALS CONF(Shut down) */
			data_16 = ( ALS_SM<<11 | ALS_IT<<6 |ALS_PERS<<4 | ALS_INT_EN_OFF<<1 | ALS_SD_OFF );
			err = cm32180_als_write_reg(CM32180_ALS_CONF, data_16 & 0xFF , data_16 >> 8);
			if(err < 0) {
				printk("[cm32180][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM32180_ALS_CONF, err);
				return -1;
			}else
				printk(DBGMSK_PRX_G3"[cm32180][als] set als_config(0x%x), data=0x%x\n",
										CM32180_ALS_CONF, data_16);

			g_cm32180_data_as->g_cm32180_als_switch_on = 0;

			printk("[cm32180][als] turn off light sensor --.\n");
		}
	}
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////
// CM32180 CTS & Factory and debug port
//
static int als_atd_get_status_and_adc(int *als_status, int *als_adc)
{
	int adc = 0;
	int status = 0;
	status = cm32180_turn_onoff_als(1);
	if (status == 0)	{
		adc = get_als_adc_from_cm32180();
		status = cm32180_turn_onoff_als(g_cm32180_data_as->g_cm32180_als_switch_on);
	}
	if(status == 0)	{
		*als_status = 1;
		*als_adc = adc;
	}else	{
		*als_status = 0;
		*als_adc = 0;
	}
	printk(DBGMSK_PRX_G2"[cm32180][als][ATD] Status:%d, adc:%d\n", *als_status, *als_adc);
	return status;
}

static int cm32180_show_adc( struct device *dev, struct device_attribute *attr, char *buf )
{
	int adc = 0;
	
	adc = get_als_adc_from_cm32180();
	
	printk("[cm32180][als] Get adc : %d\n", adc );
	
	return sprintf(buf, "%d\n", adc);
}
static DEVICE_ATTR(adc, S_IRWXU | S_IRWXG | S_IROTH, cm32180_show_adc, NULL );


#ifdef ASUS_FACTORY_BUILD
/* Set and write Factory calibration value */
static int a_als_calibration_lux = 80000;
static int a_als_low_calibration_adc = 0;
static int a_als_high_calibration_adc = 0;

static struct write_calvalue {
    struct work_struct write_calvalue_work;
    int calvalue;
} *cm32180_write_calvalue;

static struct write_shift {
    struct work_struct write_shift_work;
    int calvalue;
} *cm32180_write_shift;

static void write_lightsensor_calibrationvalue_work(struct work_struct *work)
{
	struct file *fp = NULL; 
	struct write_calvalue *this = NULL;
	loff_t pos_lsts = 0;
	char writestr[8];
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	this = container_of(work, struct write_calvalue, write_calvalue_work);

	fp = filp_open(LSENSOR_CALIBRATION_ASUS_NV_FILE, O_RDWR|O_CREAT|O_TRUNC, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk("[als] write_lsensor_calvalue open (%s) fail\n", LSENSOR_CALIBRATION_ASUS_NV_FILE);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk(DBGMSK_PRX_G3"[als] write_lsensor_calvalue = %d[%s(%d)]\n", 
		this->calvalue, writestr, strlen(writestr));

	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
		printk(DBGMSK_PRX_G0"[als] write_lsensor_calvalue fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}

static void write_lightsensor_shiftvalue_work(struct work_struct *work)
{
	struct file *fp = NULL; 
	struct write_shift *this = NULL;
	loff_t pos_lsts = 0;
	char writestr[8];
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	this = container_of(work, struct write_shift, write_shift_work);

	fp = filp_open(LSENSOR_CALIBRATION_SHIFT_ASUS_NV_FILE, O_RDWR|O_CREAT|O_TRUNC, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G0"[als] write_lsensor_shift open (%s) fail\n", LSENSOR_CALIBRATION_SHIFT_ASUS_NV_FILE);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk(DBGMSK_PRX_G3"[als] write_lsensor_shift = %d[%s(%d)]\n", 
		this->calvalue, writestr, strlen(writestr));

	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
		printk(DBGMSK_PRX_G0"[als] write_lsensor_shift fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}

static int als_show_calibration_200(struct device *dev, struct device_attribute *attr, char *buf)
{	
	read_lightsensor_calibrationvalue();
	
	printk(DBGMSK_PRX_G2"[als] Show_gait_calibration: %d\n", cm32180_light_calibration );
	
	return sprintf(buf, "%d\n", cm32180_light_calibration );
}

static int als_store_calibration_200(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	
	if ( (strict_strtoul(buf, 10, &val) < 0) )
		return -EINVAL;
	
	/*Get low brightness adc*/
	a_als_low_calibration_adc = (int)val;

	printk("[als] Get low calibration adc value : %d\n", a_als_low_calibration_adc );

	return a_als_low_calibration_adc;
	
}

static DEVICE_ATTR(calibration_200, S_IRWXU | S_IRWXG| S_IRWXO,
		   als_show_calibration_200, als_store_calibration_200);


static int als_show_calibration_1000(struct device *dev, struct device_attribute *attr, char *buf)
{	
	read_lightsensor_shiftvalue();

	printk(DBGMSK_PRX_G2"[als] Show_shift_calibration: %d\n", cm32180_light_shift_calibration );
	
	return sprintf(buf, "%d\n", cm32180_light_shift_calibration );
}

static int als_store_calibration_1000(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	
	if ( (strict_strtoul(buf, 10, &val) < 0) )
		return -EINVAL;
	
	/*Get low brightness adc*/
	a_als_high_calibration_adc = (int)val;

	printk("[als] Phone Lightsensoer Get Hight calibration adc value : %d\n", a_als_high_calibration_adc );

	/*Calibration operation*/
	cm32180_light_calibration = 
		a_als_calibration_lux / ( a_als_high_calibration_adc - a_als_low_calibration_adc );

	cm32180_light_shift_calibration = 
		1000 - ( a_als_high_calibration_adc*cm32180_light_calibration/100 );

	/*Write Calibration value*/
	cm32180_write_calvalue = kmalloc(sizeof(struct write_calvalue), GFP_KERNEL);

	INIT_WORK(&cm32180_write_calvalue->write_calvalue_work, write_lightsensor_calibrationvalue_work);

	cm32180_write_calvalue->calvalue = cm32180_light_calibration;

	queue_work(cm32180_light_interrupt_workqueue, &cm32180_write_calvalue->write_calvalue_work);
	
	/*Write shift value*/
	cm32180_write_shift = kmalloc(sizeof(struct write_shift), GFP_KERNEL);

	INIT_WORK(&cm32180_write_shift->write_shift_work, write_lightsensor_shiftvalue_work);

	cm32180_write_shift->calvalue = cm32180_light_shift_calibration;

	queue_work(cm32180_light_interrupt_workqueue, &cm32180_write_shift->write_shift_work);

	return a_als_high_calibration_adc;
}

static DEVICE_ATTR(calibration_1000, S_IRWXU | S_IRWXG | S_IRWXO,
		   als_show_calibration_1000, als_store_calibration_1000);
#endif

static struct attribute *cm32180_attributes[] = {
#ifdef ASUS_FACTORY_BUILD
	&dev_attr_calibration_200.attr,
	&dev_attr_calibration_1000.attr,
#endif
	&dev_attr_adc.attr,
	NULL
};

static const struct attribute_group cm32180_attr_group = {
	.name = "cm32180",
	.attrs = cm32180_attributes,
};

/*.........................................Sensoer init.........................................*/
static int cm32180_gpio_init(void)
{
	int rc = -EINVAL;
	printk("[cm32180][board]cm32180_gpio_init++\n");

	/* configure Phone Lightsensor interrupt gpio */
	rc = gpio_request(CM32180_GPIO_PROXIMITY_INT, "cm32180-irq");
	if (rc) {
		pr_err("%s: unable to request gpio %d (cm32180-irq)\n",__func__, CM32180_GPIO_PROXIMITY_INT);
		goto err;
	}

	rc = gpio_direction_input(CM32180_GPIO_PROXIMITY_INT);
	if (rc < 0) {
		pr_err("%s: unable to set the direction of gpio %d\n",__func__, CM32180_GPIO_PROXIMITY_INT);
		goto err;
	}

	/* configure Phone Light/Proximity sensor power_enable gpio */
	if ( !gpio_get_value(CM32180_GPIO_PROXIMITY_PWR_EN) )	{
		/* configure Phone Light/Proximity sensor power_enable gpio */
		rc = gpio_request(CM32180_GPIO_PROXIMITY_PWR_EN, "proxm_pwr_en");
		if (rc) {
			pr_err("%s: unable to request gpio %d (proxm_pwr_en)\n",__func__, CM32180_GPIO_PROXIMITY_PWR_EN);
			goto err;
		}

		rc = gpio_direction_output(CM32180_GPIO_PROXIMITY_PWR_EN, 1);
		if (rc < 0) {
			pr_err("%s: unable to set the direction of gpio %d\n",__func__, CM32180_GPIO_PROXIMITY_PWR_EN);
			goto err;
		}

		/* HW Power on Light/Proximity sensor  */
		gpio_set_value(CM32180_GPIO_PROXIMITY_PWR_EN, 1);

		printk("[cm32180][board] Power on Light/Proximity sensor\n");
	}
	printk("[cm32180][board]cm32180_gpio_init--\n");
	return 0;
err:
	gpio_free(CM32180_GPIO_PROXIMITY_PWR_EN);
	return rc;
}

static int init_cm32180(void)
{
	int ret = 0;
	printk("[cm32180]: init_cm32180 +.\n");
	
	/* Set workqueue*/
	cm32180_light_debounce_workqueue	= create_singlethread_workqueue("cm32180_als_debounce_wq");
	cm32180_light_interrupt_workqueue	= create_singlethread_workqueue("cm32180_als_interrupt_wq");
	INIT_DELAYED_WORK(&cm32180_light_debounce_work, cm32180_als_debounce_work);
	INIT_DELAYED_WORK(&cm32180_light_interrupt_delay_work, cm32180_light_interrupt_handler);
	INIT_WORK(&cm32180_attached_Pad_work, cm32180_lightsensor_attached_pad);

	// register irq handler after sensor power on in order to avoid unexpected irq error!
	g_cm32180_device.irq = gpio_to_irq( g_cm32180_device.cm32180_irq_gpio );
	printk("[cm32180]Reques EIRQ %d succesd on GPIO:%d\n",
						g_cm32180_device.irq,
						g_cm32180_device.cm32180_irq_gpio );

	if( g_cm32180_device.irq < 0 )
		printk("[cm32180] gpio_to_irq fail (g_cm32180_device.irq)irq=%d.\n", g_cm32180_device.irq);
	else		{
		printk("[cm32180] (g_cm32180_device.irq) irq=%d.\n", g_cm32180_device.irq);
		ret = request_irq(  g_cm32180_device.irq,
			cm32180_als_interrupt_handler,
				IRQF_TRIGGER_FALLING  | IRQF_TRIGGER_FALLING,
				"cm32180_INT",
				&cm32180_ara_client->dev );
	}
	
	if (ret < 0)
		printk("[cm32180] (g_cm32180_device.irq) request_irq() error %d.\n",ret);
	else	{
		printk("[cm32180] (g_cm32180_device.irq) request_irq ok.\n");
		disable_irq(g_cm32180_device.irq);
		msleep(5);
		enable_irq(g_cm32180_device.irq);
	}

	printk("[cm32180] init_cm32180 -.\n");
	return ret;
}

static int cm32180_input_init(void)
{
	int ret = 0;
	struct input_dev *input_dev_as = NULL;

	input_dev_as = input_allocate_device();
	if (!input_dev_as) {
		ret = -ENOMEM;
		printk("[cm32180]: Failed to allocate input_data device\n");
		goto error_1;
	}

	input_dev_as->name = "ASUS Lightsensor";
	input_dev_as->id.bustype = BUS_I2C;
	input_set_capability(input_dev_as, EV_ABS, ABS_MISC);
	__set_bit(EV_ABS, input_dev_as->evbit);
	__set_bit(ABS_MISC, input_dev_as->absbit);
	input_set_abs_params(input_dev_as, ABS_MISC, 0, 1048576, 0, 0);
	input_set_drvdata(input_dev_as, g_cm32180_data_as);

	ret = input_register_device(input_dev_as);
	if (ret < 0) {
		input_free_device(input_dev_as);
		goto error_1;
	}

	g_cm32180_data_as->input_dev = input_dev_as;
	g_cm32180_data_as->polling = 0;
	g_cm32180_data_as->poll_interval_ms = 100;
	g_cm32180_data_as->event_threshold = 1000;

	ret = als_lux_report_event_register(g_cm32180_data_as->input_dev);

error_1:
	return ret;
}

static int cm32180_reset(void)
{
	int ret = 0;
	
	printk("[cm32180] cm32180_reset +.\n");

	ret = cm32180_als_write_reg(CM32180_ALS_CONF, 0x01, 0x00);
	if(ret < 0) {
		printk("[cm32180][als] Reset(%s): err(0x%x)=%d\n",__FUNCTION__,CM32180_ALS_CONF, ret);
		return -1;
	}else
		printk(DBGMSK_PRX_G3"[cm32180][als] Reset als_config(0x%x)\n", CM32180_ALS_CONF);

	ret = cm32180_als_write_reg(CM32180_ALS_WH, 0x00, 0x00);
	if(ret < 0) {
		printk("[cm32180][als] Reset(%s): err(0x%x)=%d\n",__FUNCTION__,CM32180_ALS_WH, ret);
		return -1;
	}else
		printk(DBGMSK_PRX_G3"[cm32180][als] Reset als_config(0x%x)\n", CM32180_ALS_WH);

	ret = cm32180_als_write_reg(CM32180_ALS_WL, 0x00, 0x00);
	if(ret < 0) {
		printk("[cm32180][als] Reset(%s): err(0x%x)=%d\n",__FUNCTION__,CM32180_ALS_WL, ret);
		return -1;
	}else
		printk(DBGMSK_PRX_G3"[cm32180][als] Reset als_config(0x%x)\n", CM32180_ALS_WL);
	
	printk("[cm32180] cm32180_reset -.\n");

	return ret;
}

/*.........................................Ambient light sensoer.........................................*/
static const struct i2c_device_id cm32180_als_conf_id[] = {
	{ "cm32180_als_conf", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, cm32180_als_conf_id);

static struct of_device_id cm32180_als_conf_match_table[] = {
	{ .compatible = "capella,cm32180_als_conf",},
	{},
};

static struct i2c_driver cm32180_als_conf_driver = {
	.driver = {
		.name	= "cm32180_als_conf",
		.owner	= THIS_MODULE,
		.of_match_table = cm32180_als_conf_match_table,
	},
	.probe	= cm32180_als_conf_probe,
	.id_table = cm32180_als_conf_id,
};

static int __devinit cm32180_als_conf_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	printk("[cm32180] cm32180_als_conf_probe +.\n");
	if (client == NULL) {
		printk("[cm32180] Client is NUll.\n");
		ret =  -EFAULT;
		goto cm32180_als_probe_err;
	}

	if (!(g_cm32180_data_as = kmalloc(sizeof(struct cm32180_data), GFP_KERNEL))) {
		ret = -ENOMEM;
		goto cm32180_als_probe_err;
	}

	if (!(cm32180_als_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
		ret = -ENOMEM;
		goto cm32180_als_probe_err;
	}

	memset(g_cm32180_data_as, 0, sizeof(struct cm32180_data));

	cm32180_als_client = client;
	i2c_set_clientdata(cm32180_als_client, g_cm32180_data_as);
	cm32180_als_client->driver = &cm32180_als_conf_driver;
	cm32180_als_client->flags = 1;
	strlcpy(cm32180_als_client->name, CM32180_ALS_CONF_NAME, I2C_NAME_SIZE);

	printk("[cm32180] calling init_platform_hw\n");
	ret = cm32180_gpio_init();
	if (ret) {
		printk("hw init failed");
		goto cm32180_als_probe_err;
	}

	printk("[cm32180] Register input device...\n");
	if( cm32180_input_init() != 0 )
		goto cm32180_als_probe_err;

	ret = proximity_dev_register(&cm32180_ambientDev);
	if (ret)
		printk("[cm32180] ambientdl create sysfile fail.\n");

	ret = sysfs_create_group(&client->dev.kobj, &cm32180_attr_group);
	if (ret)
		goto cm32180_als_probe_err;

	ret = cm32180_reset();
	if (ret < 0)
		goto cm32180_als_probe_err;

	printk("[cm32180] cm32180_als_conf_probe -.\n");
	return 0;

cm32180_als_probe_err:
	printk("[cm32180] cm32180_als_conf_probe - (error).\n");
	//if (g_cm32180_data_as != NULL) {
	//  kfree(g_cm32180_data_as);
	//}
	return ret;
}

/*.......................................Alert Response interrupt.........................................*/
static int cm32180_suspend(struct i2c_client *client, pm_message_t mesg)
{
    return 0;
}

static int cm32180_resume(struct i2c_client *client)
{
    return 0;
}

static int cm32180_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id cm32180_ara_id[] = {
	{ "cm32180_ara", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, cm32180_ara_id);

static struct of_device_id cm32180_ara_match_table[] = {
	{ .compatible = "capella,cm32180_ara",},
	{},
};

static struct i2c_driver cm32180_ara_driver = {
	.driver = {
		.name	= "cm32180_ara",
		.owner	= THIS_MODULE,
		.of_match_table = cm32180_ara_match_table,
	},
	.suspend = cm32180_suspend,
	.resume	= cm32180_resume,
	.probe	= cm32180_ara_probe,
	.remove	= cm32180_remove,
	.id_table = cm32180_ara_id,
};

static int __devinit cm32180_ara_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int error = 0;

	printk("[cm32180] cm32180_ara_probe +.\n");
	
	if (client == NULL) {
		printk("[cm32180] Client is NUll.\n");
		error =  -EFAULT;
		goto cm32180_ara_probe_err;
	}

	if (!(cm32180_ara_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
		error = -ENOMEM;
		goto cm32180_ara_probe_err;
	}

	cm32180_ara_client = client;
	cm32180_ara_client->driver = &cm32180_ara_driver;
	cm32180_ara_client->flags = 1;
	strlcpy(cm32180_ara_client->name, CM32180_ARA_NAME, I2C_NAME_SIZE);

	/* Get data that is defined in board specific code. */
	g_cm32180_device.cm32180_irq_gpio = of_get_named_gpio_flags( 
    			client->dev.of_node, "cm32180,irq-gpio",0,NULL);
/*
	printk("[cm32180]calling init_platform_hw\n");
	error = cm32180_gpio_init();
	if (error) {
		printk("hw init failed");
		goto cm32180_ara_probe_err;
	}
*/
	error = init_cm32180();
	if( error < 0 )  {
		printk("[cm32180] init_cm32180() error.\n");
		goto cm32180_ara_probe_err;
	}

	/* Pad mode */
	register_microp_notifier(&cm32180_lightsensor_mp_notifier);
	notify_register_microp_notifier(&cm32180_lightsensor_mp_notifier, "cm36283");

	printk("[cm32180] cm32180_ara_probe -.\n");
	return 0;	
	
cm32180_ara_probe_err:
	printk("[cm32180] cm32180_ara_probe - (error).\n");
	return error;	
}


MODULE_AUTHOR("ASUS");
MODULE_DESCRIPTION("test version v1.0");

module_i2c_driver(cm32180_als_conf_driver);
module_i2c_driver(cm32180_ara_driver);
