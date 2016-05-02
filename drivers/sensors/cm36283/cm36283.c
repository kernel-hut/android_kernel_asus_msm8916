/* cm36283.c - CM36283 proximity/light sensor driver
 *
 * Copyright (C) 2012 ASUSTek Inc.
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
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
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

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h> 
#endif

#include "linux/input/al3010.h"
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>	//ASUS_BSP Lenter+

#define CM36283_DRV_NAME					"cm36283"
#define CM36283_GPIO_PROXIMITY_INT		74
#define CM36283_GPIO_PROXIMITY_PWR_EN	3

static int cm36283_irq_gpio = 0;

struct i2c_client *cm36283_client = NULL;
struct cm36283_data {
	struct i2c_client		client;
	struct input_dev		*input_dev;			/* Pointer to input device */
	struct mutex			lock;				/* For muxtex lock */

	struct delayed_work	i2c_poll_work;
	unsigned int			poll_interval_ms;	/* I2C polling period */
	unsigned int			event_threshold;	/* Change reqd to gen event */
	unsigned int			open_count;			/* Reference count */
	char					polling;				/* Polling flag */
};

struct _cm36283_device {
    int irq;
} g_cm36283_device;

struct cm36283_data  *g_cm36283_data_as;
struct cm36283_data  *g_cm36283_data_ps;

struct input_dev *this_input_dev_as = NULL;
struct input_dev *this_input_dev_ps = NULL;

static struct workqueue_struct *cm36283_workqueue;
static struct workqueue_struct *cm36283_delay_workqueue;
//[CR] Queue a work to write ATD self-detect file;

static struct work_struct cm36283_attached_Pad_work;
static struct work_struct cm36283_light_interrupt_work;
static struct work_struct cm36283_proximity_interrupt_work;
static struct delayed_work cm36283_light_interrupt_delay_work;

static int g_switch[2] = {0,0}; //switch on/off for proximity and light.

static u8 g_cm36283_int_state =1;
static int g_cm36283_on_fail=0;

static int g_proxm_dbg = 0; /* Add for debug only */
static int g_ambient_dbg = 0; /* Add for debug only */
static int g_interval = 100;
static int g_proxm_switch_on = 0;
static int pad_proxm_switch_on = 0;/*For phone call on in pad issue*/
static int g_ambient_suspended = 0;
static int g_cm36283_earlysuspend_int = 0;
static int g_cm36283_reset = 0;

u32 g_cm36283_light=0;
static u16 g_last_cm36283_light=0;
static int g_cm36283_light_first=1;

// adc is the value that is returned from HW directly ( Original adb number )
static u16 g_cm36283_light_adc = 0;
static int g_cm36283_light_k_adc = 0;

static int g_nProxm_Faraway = 1;	//means no object detected!

//INFOMATION:
//The level of autobrightness is defined in frameworks/base/core/res/res/values/config.xml
//Setting config_autoBrightnessLevels and config_autoBrightnessLcdBacklightValues.

//A80
static int g_max_light_level = 17;
static int g_cm36283_light_map[17] = {0,50,100,200,300,400,500,650,800,1000,1500,2000,3000,4000,5000,7000,10000};

//A68 EVB{0,100,200,400,600,800,1000,1200,1400,1800,2200,3000,3800,4600,5400,6500} ;
//A68 Old{ 0 , 50, 100, 200, 300, 400, 500, 600, 700, 900, 1100, 1400, 1700, 2100};

atomic_t ambient_update, proxm_update;

static int g_ps_threshold_lo = DEFAULT_PS_THRESHOLD_lo;
static int g_ps_threshold_hi = DEFAULT_PS_THRESHOLD_hi;
static int g_ioctl_enable = 1; //allow ioctl function.

static int g_als_threshold_lo = 0;
static int g_als_threshold_hi = 0;

static u16 g_psData = 0; //proximity data
static u16 g_prxVer = 0;
static u8 als_ps_SD = 0x1;

static u8 als_IT = 0x1; //0=80ms and 0.1 lux/step, 1=160ms and 0.05 lux/step, 2=320ms and 0.025 lux/step, 3=640ms and 0.0125 lux/step.
static u8 als_PERS = 3; //0~3. ALS interrupt persistence setting
static u8 als_INT_En = 1;   //enable als

static u8 ps_DUTY = 0;  //PS LED on/off duty ratio setting
static u8 ps_IT = 3;
static u8 ps_PERS = 3; //PS interrupt persistence setting
static u8 ps_ITB = 0;
static u8 ps_INT = 3;
static u8 ps_SM_PERS = 1;
static u8 ps_FOR = 0;
static u8 ps_FOR_Tri = 0;
static u8 ps_MS = 0;    //normal operation (0) / logical hi_low (1)

static int g_android_or_oms = 0; // 0 for android, 1 for oms.

/*
static int threshold_max_level = 40;
static int g_light_level[3][40] = { // for set Level
                    {400,1000,2000,4000,6000,10000,14000,18000,22000,30000,38000,46000,54000,65000},  //A68 SR1
                    {50,250,500,750,1000,1250,1500,1750,2000,2250,2500,2750,3000,3250,3500,4000,4500,5000,5500,6000,7000,7750,8500,9250,10500,12500,15000},  //A68 SR2
                    {50,100,150,200,350,450,550,650,750,850,1000,1350,1700,2000,2500,3000,3500,4000,4500,5000,6000,7000,8000,9000,10000,11000,12000,13000,14000,16000,
                    18000,20000,22000,24000,28000,31000,34000,37000,40000,50000}//A68 ER	adc*0.025
                };
*/
//A68 EVB : {0   ,  100,  200,  400, 600,  1000,  1400,  1800,  2200,  3000, 3800, 4600, 5400, 6500} 
//A68 SR1 : {400,1000,2000,4000,6000,10000,14000,18000,22000,30000,38000,46000,54000,65000}
//A68 SR2 : {10,50,100,200,400,600,800,1000,2000,4000,6000,10000,14000,18000,22000,30000,38000,46000,54000,65000}

//wait_queue_head_t cm36283_wq_head; /* Wait queue for the misc device */
DECLARE_WAIT_QUEUE_HEAD(ambient_wq_head);

static int cm36283_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int cm36283_remove(struct i2c_client *client);
static int cm36283_suspend(struct i2c_client *client, pm_message_t mesg);
static int cm36283_resume(struct i2c_client *client);
static int cm36283_reset(void);

static int cm36283_read_reg(struct i2c_client* client, u8 reg, int len, void *data);
static int cm36283_write_reg(u8 reg, int data_l, int data_h, int len);

//device attribute ++
static int proxmdev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int proxmdev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int proxmdev_open(struct inode *inode, struct file *file);
static ssize_t proxmdev_read(struct file *file, char __user * buffer, size_t size, loff_t * f_pos);
static unsigned int proxmdev_poll(struct file *filep, poll_table * wait);
static long proxmdev_ioctl(struct file *file, unsigned int cmd,unsigned long arg);
static int proxmdev_release(struct inode *inode, struct file *filp);

static int ambientDev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int ambientDev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int ambientDev_open(struct inode *inode, struct file *file);
static ssize_t ambientDev_read(struct file *file, char __user * buffer, size_t size, loff_t * f_pos);
static unsigned int ambientDev_poll(struct file *filep, poll_table * wait);
static int ambientDev_release(struct inode *inode, struct file *filp);
//device attribute --

static int proxm_set_hi_threshold(int value);
static int proxm_set_lo_threshold(int value);

static int cm36283_turn_onoff_proxm(bool bOn);
static int cm36283_turn_onoff_als(bool bOn);
static int get_adc_calibrated_lux_from_cm36283(void);

static void gpio_proximity_onoff(void);
static int atd_write_status_and_adc_2(int *als_sts, int *als_adc, int *ps_sts);
static int proximity_als_turn_on(int bOn);

//void als_lux_report_event(int);
extern int g_HAL_als_switch_on;		// For all lightsensor trun on/off global flag

//wake_lock for Proximity, make sure Proximity finish all of the work before suspend
#include <linux/wait.h>
#include <linux/wakelock.h>

static struct wake_lock proximity_wake_lock;
static int g_proxim_state = 0;

// for als / ps interrupt work ++
static struct work_struct cm36283_ISR_work;
static void light_interrupt_work(struct work_struct *work);
static void proximity_interrupt_work(struct work_struct *work);

/*static int g_lsb_thd[42] =   {0,0x32,0x64,0x96,0xc8,0x5e,0xc2,0x26,0x8a,0xee,0x52,0xe8,0x46,0xa4,0xd0,0xc4,0xb8,0xac,0xa0,0x94,0x88,0x70,0x58,0x40,0x28,0x10,0xf8,0xe0,0xc8,
						    0xb0,0x80,0x50,0x20,0xf0,0xc0,0x60,0x18,0xd0,0x88,0x40,0x50,0xff};
static int g_msb_thd[42] = {0,0x00,0x00,0x00,0x00,0x01,0x01,0x02,0x02,0x02,0x03,0x03,0x05,0x06,0x07,0x09,0x0b,0x0d,0x0f,0x11,0x13,0x17,0x1b,0x1f,0x23,0x27,0x2a,0x2e,0x32,
						    0x36,0x3e,0x46,0x4e,0x55,0x5d,0x6d,0x79,0x84,0x90,0x9c,0xc3,0xff};

static int g_thd[42] = {0,50,100,150,200,350,450,550,650,750,850,1000,1350,1700,2000,2500,3000,3500,4000,4500,5000,6000,7000,8000,9000,10000,11000,12000,13000,14000,16000,
						18000,20000,22000,24000,28000,31000,34000,37000,40000,50000,65535};
*/
/*A68 SR2*/
//{10,50,100,200,400,600,800,1000,2000,4000,6000,10000,14000,18000,22000,30000,38000,46000,54000,65000}  //A68 SR1
//static int g_lsb_thd[22] =   {0,0x0a,0x32,0x64,0xc8,0x90,0x58,0x20,0xe8,0xd0,0xa0,0x70,0x10,0xb0,0x50,0xf0,0x30,0x70,0xb0,0xf0,0xe8,0xff};
//static int g_msb_thd[22] = {0,0x00,0x00,0x00,0x00,0x01,0x02,0x03,0x03,0x07,0x0f,0x17,0x27,0x36,0x46,0x55,0x75,0x94,0xb3,0xd2,0xfd,0xff};

/*Old*/
//static int g_lsb_thd[30] =   {0,0x32,0xfa,0xf4,0xee,0xe8,0xe2,0xdc,0xd6,0xd0,0xca,0xc4,0xbe,0xb8,0xb2,
//						    0xac,0xa0,0x94,0x88,0x7c,0x70,0x58,0x46,0x34,0x22,0x04,0xd4,0x98,0xf0,0xff};
//static int g_msb_thd[30] = {0,0x00,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,
//						    0x0d,0x0f,0x11,0x13,0x15,0x17,0x1b,0x1e,0x21,0x24,0x29,0x30,0x3a,0x55,0xff};

//static int g_level = 0;     //als interrupt level
//--

// error handle ++
//static void light_proxim_error_reset(int handle, bool enable);
//static unsigned long jtimes0, jtimes1;
enum sensors {
        LIGHT = 0,
        PROXIMITY,
} sensor_mode;
//  --

// r/w calibration value ++
static bool read_lightsensor_calibrationvalue(void);
#ifdef ASUS_FACTORY_BUILD
static void write_lightsensor_calibrationvalue_work(struct work_struct *work);

static struct write_calvalue {
    struct work_struct write_calvalue_work;
    int calvalue;
} *cm36283_write_calvalue;
#endif

static bool read_lightsensor_shiftvalue(void);
#ifdef ASUS_FACTORY_BUILD
static void write_lightsensor_shiftvalue_work(struct work_struct *work);

static struct write_shift {
    struct work_struct write_shift_work;
    int calvalue;
} *cm36283_write_shift;
#endif
static bool read_prox_hi_calibrationvalue(void);
static bool read_prox_lo_calibrationvalue(void);
#ifdef ASUS_FACTORY_BUILD
static void write_prox_hi_calibrationvalue_work(struct work_struct *work);
static struct write_prox_hi {
    struct work_struct write_prox_hi_work;
    int calvalue;
} *cm36283_write_prox_hi;


static void write_prox_lo_calibrationvalue_work(struct work_struct *work);
static struct write_prox_lo {
    struct work_struct write_prox_lo_work;
    int calvalue;
} *cm36283_write_prox_lo;

#endif

#ifdef ASUS_FACTORY_BUILD
static int a_als_calibration_lux = 800;
static int a_als_low_calibration_adc = 0;
static int a_als_high_calibration_adc = 0;
static int a_ps_hi_calibration_adc = 0;
static int a_ps_lo_calibration_adc = 0;
#endif
static u32 g_cm36283_light_shift_calibration = 40;
static int g_cm36283_light_gain_calibration = 38000;
static int cm36283_als_calibration_accuracy = 100000;
/*
#define LSENSOR_CALIBRATION_ASUS_NV_FILE  "/data/asusdata/lsensor.nv"
#define LSENSOR_CALIBRATION_SHIFT_ASUS_NV_FILE  "/data/asusdata/lsensor_shift.nv"
#define PSENSOR_CALIBRATION_HI_ASUS_NV_FILE  "/data/asusdata/psensor_hi.nv"
#define PSENSOR_CALIBRATION_LO_ASUS_NV_FILE  "/data/asusdata/psensor_low.nv"
*/
// for r/w calibration value --

/////////////////////////////////////////////////////////////////////////////////////////
//P02 reltaed

static void cm36283_lightsensor_attached_pad(struct work_struct *work);
bool g_cm36283_als_switch_on = false;    //this var. means if cm36283 als hw is turn on or not

extern bool g_bIsP01Attached;
//extern bool g_al3010_switch_on;


/////////////////////////////////////////////////////////////////////////////////////////
//---proximity sensor part---
//

	/*For Proximity test*/
static struct delayed_work Proximity_test_work;
static struct workqueue_struct *Proximity_test_wq = NULL;
#define PROXIMITY_TEST_DELAY			1

static enum proximity_property proxmdev_properties[] = {
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

static struct file_operations proxmdev_fops = {
    .owner = THIS_MODULE,
    .open = proxmdev_open,
    .read = proxmdev_read,
    .poll = proxmdev_poll,
    .unlocked_ioctl = proxmdev_ioctl,
    .release = proxmdev_release
};

struct proximity_class_dev g_proxmDev = {
    .id = SENSORS_PROXIMITY,
    .name = "psensor",
    .num_properties = ARRAY_SIZE(proxmdev_properties),
    .properties = proxmdev_properties,
    .get_property = proxmdev_get_property,
    .put_property = proxmdev_put_property,
    .fops = &proxmdev_fops
};


static int proxmdev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
    //printk(DBGMSK_PRO_G0 "[proxmdl]: proxmdl_get_property +.\n");

    switch( property ) 
    {
        case SENSORS_PROP_HI_THRESHOLD:
            printk(DBGMSK_PRX_G4"[cm36283][ps] SENSORS_PROP_HI_THRESHOLD. (%d)\n", g_ps_threshold_hi);
            val->intval = g_ps_threshold_hi;
            break;

        case SENSORS_PROP_LO_THRESHOLD:
            printk(DBGMSK_PRX_G4"[cm36283][ps] SENSORS_PROP_LO_THRESHOLD. (%d)\n", g_ps_threshold_lo);
            val->intval = g_ps_threshold_lo;
            break;

        case SENSORS_PROP_INTERVAL:
            //printk("[proxmdl]: config IT_PS by using \"echo XXX > /sys/class/sensors/sensor3/interval\" XXX is only 0~3. \n");
            printk(DBGMSK_PRX_G4"[cm36283][ps] SENSORS_PROP_INTERVAL.\n");
            val->intval = g_interval;
            break;

        case SENSORS_PROP_MAXRANGE:
            printk(DBGMSK_PRX_G4"[cm36283][ps] SENSORS_PROP_MAXRANGE.\n");
            val->intval = 1; //keep it 1.0 for OMS.
            break;

        case SENSORS_PROP_RESOLUTION:
            printk(DBGMSK_PRX_G4"[cm36283][ps] SENSORS_PROP_RESOLUTION.\n");
            val->intval = 1;
            break;

        case SENSORS_PROP_VERSION:
            printk(DBGMSK_PRX_G4"[cm36283][ps] SENSORS_PROP_VERSION.\n");
          //  cm36283_read_reg(cm36283_client, CM36283_PS_DATA, &g_prxVer);
            printk(DBGMSK_PRX_G4"[cm36283][ps] pData=%d\n", g_prxVer);
            val->intval = g_prxVer; //1;
            break;

        case SENSORS_PROP_CURRENT:
            printk(DBGMSK_PRX_G4"[cm36283][ps] SENSORS_PROP_CURRENT.\n");
            val->intval = 30;   
            break;

        case SENSORS_PROP_SWITCH:
            //printk(DBGMSK_PRO_G0 "[proxmdl]: SENSORS_PROP_SWITCH (%d).\n", g_kxtf9_switch_on);
            printk(DBGMSK_PRX_G4"[cm36283][ps] get switch = %d.\n", g_proxm_switch_on);
            val->intval = g_proxm_switch_on;
            break;

        case SENSORS_PROP_VENDOR:
            printk(DBGMSK_PRX_G4"[cm36283][ps] SENSORS_PROP_VENDOR.\n");
            sprintf(val->strval, "CAPELLA");
            break;

        /* Add for debug only */
        case SENSORS_PROP_DBG:
            val->intval = g_proxm_dbg;
            printk(DBGMSK_PRX_G4"[cm36283][ps] dbg = %d.\n", g_proxm_dbg);
            printk(DBGMSK_PRX_G4"[cm36283][ps] ps_IT(%d), ps_PERS(%d), ps_INT(%d), ps_MS(%d)\n",ps_IT, ps_PERS, ps_INT ,ps_MS);
            break;

        case SENSORS_PROP_ADC:
            val->intval = g_psData;
            printk(DBGMSK_PRX_G4"[cm36283][ps] get adc property: %d\n", val->intval);
            break;

        case SENSORS_PROP_ATD_STATUS:
        {
            int als_sts =0, als_adc =0, ps_sts =0;

            atd_write_status_and_adc_2(&als_sts, &als_adc, &ps_sts);
            val->intval = ps_sts;
            printk(DBGMSK_PRX_G4"[cm36283][ps] get atd status: %d\n", val->intval);
            break;
        }
        default:
            printk(DBGMSK_PRX_G0"[cm36283][ps] default.\n");
            return -EINVAL;
    }

    //printk(DBGMSK_PRO_G0 "[proxmdl]: proxmdl_get_property -.\n");
    return 0;
}

static int cm36283_turn_onoff_proxm(bool bOn)
{
	int err = 0;
	unsigned char idata[2] = {0,0};

	g_cm36283_reset=0;

	if(bOn == 1)	{	//power on
		printk(DBGMSK_PRX_G4"[cm36283][ps] sensor switch, turn on proximity sensor ++.\n");

		//set ps_threshold 0x06_L, 0x06_H
		idata[0] = g_ps_threshold_lo;
		idata[1] = g_ps_threshold_hi;

		err = cm36283_write_reg(CM36283_PS_THD, idata[0], idata[1], 3);
		if(err < 0)
			printk("[cm36283][ps] (%s): err(0x%x)=%d\n",__FUNCTION__, CM36283_PS_THD, err);
		else
			printk(DBGMSK_PRX_G4"[cm36283][ps] set ps_threshold, pdata_l=%d, pdata_h=%d\n", idata[0], idata[1]);

		//set 0x03_L, 0x03_H	Enable 
		idata[0] = (ps_DUTY<<6 | ps_IT<<4 | ps_PERS<<2 | INIT_PS);
		idata[1] = (ps_ITB<<6 | ps_INT | INIT_PS);

		err = cm36283_write_reg(CM36283_PS_CONF, idata[0], idata[1], 3);
		if(err < 0) {
			printk("[cm36283][ps] (%s): err(0x%x)=%d\n",__FUNCTION__,CM36283_PS_CONF, err);
			return -1;
		}else
			printk(DBGMSK_PRX_G4"[cm36283][ps] set ps_config(0x%x), pdata_l=0x%x, pdata_h=0x%x\n", CM36283_PS_CONF, idata[0], idata[1]);

		//set 0x04_L, 0x04_H
		idata[0] = ((ps_SM_PERS<<4) | (ps_FOR<<3) | (ps_FOR_Tri<<2) | INIT_PS);
		idata[1] = ((ps_MS<<6) | INIT_PS);

		err = cm36283_write_reg(CM36283_PS_MODE, idata[0], idata[1], 3);
		if(err < 0)
			printk("[cm36283][ps] (%s): err(0x%x)=%d\n",__FUNCTION__, CM36283_PS_MODE, err);
		else
			printk(DBGMSK_PRX_G4"[cm36283][ps] set ps_config(0x%x), pdata_l=0x%x, pdata_h=0x%x\n", CM36283_PS_MODE, idata[0], idata[1]);

		//Get Distance when light sensor turn on.
		queue_work(cm36283_workqueue, &cm36283_proximity_interrupt_work);

		printk(DBGMSK_PRX_G4"[cm36283][ps] sensor switch, turn on proximity sensor --.\n");
	}
	else		//power off
	{
		printk(DBGMSK_PRX_G4"[cm36283][ps] turn off proximity sensor ++.\n");
		idata[0] = (INIT_PS | 0x1);
		idata[1] = INIT_PS;

		/*Turn off proximity*/
		err = cm36283_write_reg(CM36283_PS_CONF, idata[0], idata[1], 3);
		if(err < 0)
			printk("[cm36283][ps] (%s): err(0x%x)=%d\n",__FUNCTION__, CM36283_PS_CONF, err);
		else
			printk(DBGMSK_PRX_G4"[cm36283][ps] Set off, pdata_l=0x%x, pdata_h=0x%x\n", idata[0], idata[1]);
		
		/*Clean proximity Data*/
		err = cm36283_write_reg(CM36283_PS_DATA, 0x00, 0x00, 3);
		if(err < 0)
			printk("[cm36283][ps] (%s): err(0x%x)=%d\n",__FUNCTION__, CM36283_PS_DATA, err);
		else
			printk(DBGMSK_PRX_G4"[cm36283][ps] Clean PS_Data\n");

		printk(DBGMSK_PRX_G4"[cm36283][ps] turn off proximity sensor --.\n");
	}
	return 0;
}

static int proxmdev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
    int ret;
    static bool bFirst = true;
    static bool openfilp = true;

    switch (property) 
    {
        case SENSORS_PROP_SWITCH:
		printk(DBGMSK_PRX_G4"[cm36283][ps] put SENSORS_PROP_SWITCH (%d,%d).\n", (val->intval), g_proxm_switch_on);

		if(bFirst) {
			printk(DBGMSK_PRX_G3"[cm36283][ps] put switch 1st read calvalue\n");
			openfilp = read_prox_hi_calibrationvalue();			

			if (openfilp == false )	{
				printk("[cm36283][ps] Get old calvalue : thd_hi = %d thd_lo = %d or fail\n", g_ps_threshold_hi, g_ps_threshold_lo);
				g_ps_threshold_hi = DEFAULT_PS_THRESHOLD_hi;
				g_ps_threshold_lo = DEFAULT_PS_THRESHOLD_lo;
			}
			else
				read_prox_lo_calibrationvalue();

			printk("[cm36283] Set prox threshold hi and lo: %d , %d\n",g_ps_threshold_hi, g_ps_threshold_lo);
			bFirst = false;
		}
	
		if((g_proxm_switch_on != val->intval) && (!g_bIsP01Attached)) {
			mutex_lock(&g_cm36283_data_ps->lock);
			if(val->intval==1) { //turn on PS
				ret = cm36283_turn_onoff_proxm(1);
				if ( ret == 0 )	{
					g_proxm_switch_on = 1;
					pad_proxm_switch_on = 0;
					printk(DBGMSK_PRX_G4"[cm36283][ps] proximity on.\n");
				}
			}else	{	//turn off PS if val->intval==0 or other
				g_proxm_switch_on = 0;
				pad_proxm_switch_on = 0;

				// disable PS or directly Power off cm36283
				cm36283_turn_onoff_proxm(0);

				//reset the state or store the last state??
				g_nProxm_Faraway = 1;
				printk(DBGMSK_PRX_G4"[cm36283][ps] proximity off.\n");
			}
			mutex_unlock(&g_cm36283_data_ps->lock);
		}
		else if (g_bIsP01Attached)	{
			printk("[cm36283][ps] Phone on in pad (%d,%d).\n", (val->intval), pad_proxm_switch_on );
			pad_proxm_switch_on = val->intval;			
		}

		break;

        case SENSORS_PROP_HI_THRESHOLD:
            printk(DBGMSK_PRX_G4"[cm36283][ps] config high THRESHOLD (%d).\n", val->intval);
            if(val->intval>=0 && val->intval<=255) {
                //if(g_proxm_switch_on==1) {
                    if(g_ps_threshold_hi != val->intval) {
                        g_ps_threshold_hi = val->intval;
                        ret = proxm_set_hi_threshold(g_ps_threshold_hi);
                    }
            }
            else {
                printk(DBGMSK_PRX_G0"[cm36283][ps] ERROR!!! OUT OF THRESHOLD (0~255)!!! \n");
            }
            break;

        case SENSORS_PROP_LO_THRESHOLD:
            printk(DBGMSK_PRX_G4"[cm36283][ps] config low THRESHOLD (%d).\n", val->intval);
            if(val->intval>=0 && val->intval<=255) {
                //if(g_proxm_switch_on==1) {
                    if(g_ps_threshold_lo != val->intval) {
                        g_ps_threshold_lo = val->intval;
                        ret = proxm_set_lo_threshold(g_ps_threshold_lo);
                    }
            }
            else {
                printk(DBGMSK_PRX_G0"[cm36283][ps] ERROR!!! OUT OF THRESHOLD (0~255)!!! \n");
            }
            break;

        case SENSORS_PROP_INTERVAL:
            if(1) {
                printk(DBGMSK_PRX_G4"[cm36283][ps] set interval (0x%x)\n", val->intval);
                g_interval = val->intval;
            }
            else {
                printk(DBGMSK_PRX_G4"[cm36283][ps] config IT_PS (0x%x)\n", val->intval);
                if( (val->intval>=0) && (val->intval<=3) ) {
                    #if 0
                    gpio_set_value(g_proxm_pwr_pin, 0);
                    msleep(1);
                    gpio_set_value(g_proxm_pwr_pin, 1);
                    gpio_free(g_proxm_pwr_pin);
                    #endif
                    ps_IT = val->intval;
                    ret = cm36283_reset();
                }
            }
            break;

        /* Add for debug only */
        case SENSORS_PROP_DBG:
		g_proxm_dbg = val->intval;
		if ( g_proxm_dbg == 1 && g_proxm_switch_on == 1)
			queue_delayed_work(Proximity_test_wq, &Proximity_test_work, HZ * PROXIMITY_TEST_DELAY);
		else if ( g_proxm_dbg == 2 && g_proxm_switch_on == 1)
			queue_delayed_work(Proximity_test_wq, &Proximity_test_work, HZ * PROXIMITY_TEST_DELAY);
		if ( g_proxm_dbg == 0 )
			cancel_delayed_work(&Proximity_test_work);

		printk(DBGMSK_PRX_G4"[cm36283][ps] dbg = %d.\n", g_proxm_dbg);
		break;
        default:
		printk(DBGMSK_PRX_G0"[cm36283][ps] put default.\n");
		return -EINVAL;
    }

    return 0;

}

static int proxmdev_open(struct inode *inode, struct file *file)
{
    int ret = 0;

    printk(DBGMSK_PRX_G4"[cm36283][ps] proxmdev_dev_open.\n");

    if (file->f_flags & O_NONBLOCK) {
        printk(DBGMSK_PRX_G4"[cm36283][ps] proxmdl_dev_open (O_NONBLOCK)\n");
    }
    atomic_set(&proxm_update, 0); //initialize atomic.

    ret = nonseekable_open(inode, file);

    return ret;
}


static ssize_t proxmdev_read(struct file *file, char __user * buffer, size_t size, loff_t * f_pos)
{
    struct input_event event;
    int ret = 0;
	
    if(g_proxm_dbg==1)
        printk(DBGMSK_PRX_G4"[cm36283][ps] proxmdl_dev_read++\n");

    if (size < 1) {
        printk(DBGMSK_PRX_G0"[cm36283][ps] proxmdl_dev_read (size error!)\n");
        return -EINVAL;
    }

    if(g_android_or_oms==1) {

        while(1) {
            // I shall return EAGAIN when the request is O_NONBLOCK and the data is not ready yet.
            if (file->f_flags & O_NONBLOCK) {
                ret = -EAGAIN;
                printk(DBGMSK_PRX_G0"[cm36283][ps] proxmdl_dev_read (-EAGAIN)\n");
                return ret;
            }
			
            if(atomic_read(&proxm_update)) {
                // If there is update, don't sleep and just update.
                atomic_dec(&proxm_update);
                if(g_proxm_dbg==1)
			printk(DBGMSK_PRX_G0"[cm36283][ps] proxmdl_dev_read break!\n");
                break;
            }
            else {
                // If there is no avaialbe, just sleep and wait for wakeup or signal.
                if(g_proxm_dbg==1)
                    printk(DBGMSK_PRX_G4"[cm36283][ps] proxmdl_dev_read schedule++\n");
                schedule();
                if(g_proxm_dbg==1)
                    printk(DBGMSK_PRX_G4"[cm36823][ps] proxmdl_dev_read schedule--\n");
            }
        } //end of while
    }

    do_gettimeofday(&event.time);

    event.type = EV_ABS;
    event.code = ABS_DISTANCE;
    //jonathan: change the way to get intr. pin state
    //g_cm36283_proxm = gpio_get_value(g_proxm_int_pin);

    event.value = g_nProxm_Faraway ? 1: 0;

    /* make sure we are not going into copy_to_user() with
     * TASK_INTERRUPTIBLE state */
    set_current_state(TASK_RUNNING);
    if (copy_to_user(buffer, &event, sizeof(event))) {
        printk(DBGMSK_PRX_G0"[cm36283][ps] Copy proxm data Fail!\n");
        ret = -EFAULT;
    } 
    else  {
        ret = sizeof(event);
    }

    if(g_proxm_dbg) {
        cm36283_read_reg(cm36283_client, CM36283_PS_DATA, 2, &g_prxVer);
        printk(DBGMSK_PRX_G4"[cm36283][ps] proxm value 0x%x (%d)\n", event.value, g_prxVer);
    }

    //msleep(100); //100ms

    if(g_proxm_dbg==1)
        printk("[cm36283][ps] proxmdl_dev_read--\n");
    return ret;
}

static unsigned int proxmdev_poll(struct file *filep, poll_table * wait)
{
    if(g_proxm_dbg==1)
        printk(DBGMSK_PRX_G4"[cm36283][ps] proxmdl_dev_poll++\n");

    if(atomic_read(&proxm_update)) {
        if(g_proxm_dbg==1)
            printk(DBGMSK_PRX_G4"[cm36283][ps] proxmdl_dev_poll-- (POLLIN %d)\n",atomic_read(&proxm_update));
        return (POLLIN | POLLRDNORM);
    }

    if(g_proxm_dbg==1)
        printk(DBGMSK_PRX_G4"[cm36283][ps] proxmdl_dev_poll--\n");
    return 0;
}


static long proxmdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    void __user *data;
    int value=0;
    unsigned char idata[2] = {0,0};
    int err=0;
    int ret;

    switch (cmd) 
    {
        case PS_POLL_READ_GPIO_ON_OFF: //sensor on/off
            printk(DBGMSK_PRX_G4"[cm36283][ps] PS_POLL_READ_GPIO_ON_OFF %d\n", (int)arg);
            value = (int)arg;
            if(value == 1) {
                printk(DBGMSK_PRX_G4"[cm36283][ps] promxdl_dev_ioctl, SD_PS = 0.\n");
                //set 0x03_L, 0x03_H
                idata[0] = (ps_DUTY<<6 | ps_IT<<4 | ps_PERS<<2 | INIT_PS);
                idata[1] = (ps_ITB<<6 | ps_INT | INIT_PS);

                err = cm36283_write_reg(CM36283_PS_CONF, idata[0], idata[1], 3);
                if(err < 0) {
                    printk(DBGMSK_PRX_G0"[cm36283][ps] (%s): err=%d\n",__FUNCTION__, err);
                }
                else {
                    printk(DBGMSK_PRX_G4"[cm36283][ps] set ps_config(0x%x), pdata_l=0x%x, pdata_h=0x%x\n", CM36283_PS_CONF, idata[0], idata[1]);
                }

                //set 0x04_L, 0x04_H
                idata[0] = ((ps_SM_PERS<<4) | (ps_FOR<<3) | (ps_FOR_Tri<<2) | INIT_PS);
                idata[1] = ((ps_MS<<6) | INIT_PS);
    
                err = cm36283_write_reg(CM36283_PS_MODE, idata[0], idata[1], 3);
                if(err < 0) {
                    printk(DBGMSK_PRX_G0"[cm36283][ps] (%s): err=%d\n",__FUNCTION__, err);
                }
                else {
                    printk(DBGMSK_PRX_G4"[cm36283][ps] set ps_config(0x%x), pdata_l=0x%x, pdata_h=0x%x\n", CM36283_PS_MODE, idata[0], idata[1]);
                }
                g_proxm_switch_on = 1;
            }

            else {
                printk(DBGMSK_PRX_G4"[cm36283][ps] promxdl_dev_ioctl, SD_PS = 1.\n");
                g_proxm_switch_on = 0;
                //0x04_L 0x04_H
                idata[0] = (INIT_PS | 0x1);  //power off
                idata[1] = INIT_PS;

                err = cm36283_write_reg(CM36283_PS_CONF, idata[0], idata[1], 3);
                if(err < 0) {
                    printk(DBGMSK_PRX_G0"[cm36283][ps] (%s): err=%d\n",__FUNCTION__, err);
                }
                else {
                    printk(DBGMSK_PRX_G4"[cm36283][ps] clear ps_config, pdata_l=0x%x, pdata_h=0x%x\n", idata[0], idata[1]);
                }
            }
            break;

        case ATD_ASK_PR_STATUS: //get data
            printk(DBGMSK_PRX_G4"[cm36283][ps] ATD_ASK_PR_STATUS \n");
            data = (void __user *) arg;
            if (data == NULL) {
                printk(DBGMSK_PRX_G0"[cm36283][ps] null data!!! \n");
                ret = -EFAULT;
                break;
            }

            if (copy_from_user(&value, data, sizeof(int))) {
                printk(DBGMSK_PRX_G0"[cm36283][ps] copy_from_user fail (%d).\n", (-EFAULT));
                return -EFAULT;
            }
            else {
                printk(DBGMSK_PRX_G4"[cm36283][ps] value=%d \n", value);
            }

            if(value>=0 && value<=255) {
                if(g_ps_threshold_hi != value) {

                    printk(DBGMSK_PRX_G4"[cm36283][ps] promxdl_dev_ioctl, changing PS setting (%d)\n", value);
                    //set ps_hi_threshold 0x06_H
                    idata[0] = g_ps_threshold_lo;
                    idata[1] = value;
        
                    err = cm36283_write_reg(CM36283_PS_THD, idata[0], idata[1], 3);
                    if(err < 0) {
                        printk(DBGMSK_PRX_G0"[cm36283][ps] (%s): err=%d\n", __FUNCTION__, err);
                        ret = -EFAULT;
                    }
                    else {
                        g_ps_threshold_hi = value;
                    }

                }
                if(ret!=(-EFAULT)) {
                    g_cm36283_int_state = gpio_get_value(CM36283_GPIO_PROXIMITY_INT);
                    g_cm36283_int_state = g_cm36283_int_state ? 1 : 0;
                    if (copy_to_user( (void *)data, &g_cm36283_int_state, sizeof(g_cm36283_int_state))) {
                        printk(DBGMSK_PRX_G0"[cm36283][ps] promxdl_dev_ioctl, copy_to_user failed!\n");
                        ret = -EFAULT;
                    }
                }
            }
            else {
                printk(DBGMSK_PRX_G1"[cm36283][ps] promxdl_dev_ioctl, wrong PS setting!\n");
                ret = -EFAULT;
            }

            break;
        ///////////////////////////////////////////////////////////////
        case IOCTL_ENABLE_CTRL: //turn on/off sensor power
            printk(DBGMSK_PRX_G4"[cm36283][ps] IOCTL_ENABLE_CTRL %d\n", (int)arg);

            value = (int)arg;
            printk(DBGMSK_PRX_G4"[cm36283][ps] proxmdl_dev_ioctl, IOCTL_ENABLE_CTRL(%d)\n",value);
            g_ioctl_enable = value;
            break;

        case CM36283_ANDROID_OR_OMS:
            value = (int)arg;
            g_android_or_oms = value;
            printk(DBGMSK_PRX_G4"[cm36283][ps] proxmdl_dev_ioctl, g_android_or_oms=%d\n",g_android_or_oms);
            break;

        default:
            printk(DBGMSK_PRX_G4"[cm36283][ps] proxmdl_dev_ioctl, unknown cmd(0x%x)\n", cmd);
            break;
    }
    return ret;
}

static int proxmdev_release(struct inode *inode, struct file *filp)
{
    printk(DBGMSK_PRX_G4"[cm36283][ps] proxmdl_dev_release.\n");

    return 0;
}


/////////////////////////////////////////////////////////////////////////////////
//---ambient light sensor part---
//
static enum proximity_property ambientDev_properties[] = {
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

static struct file_operations ambientDev_fops = {
    .owner = THIS_MODULE,
    .open = ambientDev_open,
    .read = ambientDev_read,
    .poll = ambientDev_poll,
    //.ioctl = ambientdl_dev_ioctl,
    .release = ambientDev_release,
};

struct proximity_class_dev g_ambientDev = {
    .id = SENSORS_LIGHT,
    .name = "lsensor",
    .num_properties = ARRAY_SIZE(ambientDev_properties),
    .properties = ambientDev_properties,
    .get_property = ambientDev_get_property,
    .put_property = ambientDev_put_property,
    .fops = &ambientDev_fops
};


static int ambientDev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
    printk(DBGMSK_PRX_G3"[cm36283][als] ambientdl_get_property +.\n");

    switch( property ) 
    {
        case SENSORS_PROP_HI_THRESHOLD:
            printk(DBGMSK_PRX_G3"[cm36283][als] SENSORS_PROP_HI_THRESHOLD. (%d)\n", g_als_threshold_hi);
            val->intval = g_als_threshold_hi;
            break;

        case SENSORS_PROP_LO_THRESHOLD:
            printk(DBGMSK_PRX_G3"[cm36283][als] SENSORS_PROP_LO_THRESHOLD. (%d)\n", g_als_threshold_lo);
            val->intval = g_als_threshold_lo;
            break;

        case SENSORS_PROP_INTERVAL:
            printk(DBGMSK_PRX_G3"[cm36283][als] config GAIN_ALS by using \"echo XXX > /sys/class/sensors/sensor4/interval\" XXX is only 0~3. \n");
            printk(DBGMSK_PRX_G3"[cm36283][als] SENSORS_PROP_INTERVAL.\n");
            val->intval = g_interval;
            break;

        case SENSORS_PROP_MAXRANGE:
            printk(DBGMSK_PRX_G3"[cm36283][als] SENSORS_PROP_MAXRANGE.\n");
            val->intval = 128;  
            break;

        case SENSORS_PROP_RESOLUTION:
            printk(DBGMSK_PRX_G3"[cm36283][als] SENSORS_PROP_RESOLUTION.\n");
            val->intval = 1;
            break;

        case SENSORS_PROP_VERSION:
            printk(DBGMSK_PRX_G3"[cm36283][als] SENSORS_PROP_VERSION.\n");
            val->intval = 1;    
            break;

        case SENSORS_PROP_CURRENT:
            printk(DBGMSK_PRX_G3"[cm36283][als] SENSORS_PROP_CURRENT.\n");
            val->intval = 30;   
            break;

        case SENSORS_PROP_SWITCH:
            printk(DBGMSK_PRX_G3"[cm36283][als] get switch = %d.\n", g_HAL_als_switch_on);
            val->intval = g_HAL_als_switch_on;
            break;

        case SENSORS_PROP_VENDOR:
            printk(DBGMSK_PRX_G3"[cm36283][als] SENSORS_PROP_VENDOR.\n");
            sprintf(val->strval, "CAPELLA");
            break;

        /* Add for debug only */
        case SENSORS_PROP_DBG:
            val->intval = g_ambient_dbg;
            printk(DBGMSK_PRX_G3"[cm36283][als] dbg = %d.\n", g_ambient_dbg);
            printk(DBGMSK_PRX_G3"[cm36283][als] als_IT(%d),als_PERS(%d)\n", als_IT, als_PERS);
            break;
        case SENSORS_PROP_ADC:
            val->intval = g_cm36283_light_adc;
            printk(DBGMSK_PRX_G3"[cm36283][als] get adc property: %d\n", val->intval);
            break;
        case SENSORS_PROP_K_ADC:
            val->intval = g_cm36283_light_k_adc;
            printk(DBGMSK_PRX_G3"[cm36283][als] get k_adc property: %d\n", val->intval);
            break;
        case SENSORS_PROP_LUX:
            val->intval = g_cm36283_light;
            printk(DBGMSK_PRX_G3"[cm36283][als] get lux property: %d\n", val->intval);
            break;

       case SENSORS_PROP_ATD_STATUS:
        {
            int als_sts =0, als_adc =0, ps_sts =0;

            atd_write_status_and_adc_2(&als_sts, &als_adc, &ps_sts);
            val->intval = als_sts;
            printk(DBGMSK_PRX_G3"[cm36283][als] get atd status: %d\n", val->intval);
            break;
        }
        case SENSORS_PROP_ATD_ADC:
        {
            int als_sts =0, als_adc =0, ps_sts =0;

            atd_write_status_and_adc_2(&als_sts, &als_adc, &ps_sts);
            val->intval = als_adc;
            printk(DBGMSK_PRX_G3"[cm36283][als] get atd adc: %d\n", val->intval);
            break;
        }

       default:
            printk(DBGMSK_PRX_G0"[cm36283][als] default\n");
            return -EINVAL;
    }

    printk( DBGMSK_PRX_G3"[cm36283]: ambientdl_get_property -.\n");

    return 0;
}

static int cm36283_turn_onoff_als(bool bOn)
{
    struct i2c_msg msg[1];
    unsigned char data[2] = {0,0};
    int err = 0;

    printk(DBGMSK_PRX_G3"[cm36283][als]++Turn onoff ambient sensor\n");

   if(g_cm36283_als_switch_on != bOn || g_cm36283_reset==1) {

	g_cm36283_reset = 0;

        if(1 == bOn) //turn on ambient sensor.
        { 
            printk(DBGMSK_PRX_G3"[cm36283][als] Turn on ambient sensor\n");	
	    g_cm36283_light_first=1;

           if(g_ambient_suspended==1) {
                printk(DBGMSK_PRX_G3"[cm36283][als]senosr_switch_on: switch on later when resume.\n");
                g_ambient_suspended = 1;   //set 1 that resume func will turn on als
            }
            else {
                //set als_config 0x00_L
                msg->addr = cm36283_client->addr;
                msg->flags = 0;
                msg->len = 2; //two data byte
                msg->buf = data;
                data[0] = CM36283_ALS_CONF; //command code

                data[1] = ( (als_IT<<6) | (als_PERS<<2) | (als_INT_En << 1) | INIT_ALS ); //0.05 lux/step 

                err = i2c_transfer(cm36283_client->adapter, msg, ARRAY_SIZE(msg));

                if(err < 0) {
                    printk(DBGMSK_PRX_G0"[cm36283][als] (%s): reg=0x%x, data=0x%x, err=%d\n", __FUNCTION__, data[0], data[1], err);
		    g_cm36283_on_fail=1;
                }

                else    {
                    printk(DBGMSK_PRX_G3"[cm36283][als] set initial config, reg=0x%x, value=0x%x\n",data[0], data[1]);
		    g_cm36283_on_fail=0;
                }
    
                //set interrupt high threshold 0x01
                err = cm36283_write_reg(CM36283_ALS_THDH, (g_als_threshold_hi & 0xFF), 
                            ((g_als_threshold_hi>> 8 ) & 0xFF), 3);

                if(err < 0)
			printk("[cm36283][als] (%s): reg=0x%x, err=%d\n", __FUNCTION__, CM36283_ALS_THDH, err);
                else
			printk(DBGMSK_PRX_G3"[cm36283][als] set high threshold: ldata_l=%d, ldata_h=%d\n",
					(g_als_threshold_hi & 0xFF), ((g_als_threshold_hi>> 8 ) & 0xFF));

                //set interrupt low threshold 0x02
                err = cm36283_write_reg(CM36283_ALS_THDL, (g_als_threshold_lo & 0xFF),        
                            ((g_als_threshold_lo >> 8 ) & 0xFF), 3);

                if(err < 0)
			printk(DBGMSK_PRX_G0"[cm36283][als] (%s): reg=0x%x, err=%d\n",__FUNCTION__, CM36283_ALS_THDH, err);
                else
			printk(DBGMSK_PRX_G3"[cm36283][als] set high threshold: ldata_l=%d, ldata_h=%d\n",
					(g_als_threshold_lo & 0xFF), ((g_als_threshold_lo >> 8 ) & 0xFF));

		  //Get Lux when light sensor turn on.
		  //queue_work(cm36283_workqueue, &cm36283_light_interrupt_work);
		  queue_delayed_work(cm36283_delay_workqueue, &cm36283_light_interrupt_delay_work, 20);

                g_cm36283_als_switch_on = 1;
                printk("[cm36283][als] turn on\n");
            }

        } 
        else  //turn off
        {
        	printk(DBGMSK_PRX_G3"[cm36283][als] Turn OFF ambient sensor\n");
		err = waitqueue_active(&ambient_wq_head);
		printk(DBGMSK_PRX_G3"[cm3tu6283][als] ambient_wq_head(%d)\n", err);

		g_cm36283_light_first=1;

		if(g_ambient_suspended == 1) {
			printk(DBGMSK_PRX_G3"[cm36283][als] switch off was done in early_suspend.\n");
			g_cm36283_als_switch_on = 0;
		}
		else {
			g_cm36283_als_switch_on = 0;

			msg->addr = cm36283_client->addr;
			msg->flags = 0; //0 - write.
			msg->len = 2;
			msg->buf = data;
			data[0] = CM36283_ALS_CONF;
			data[1] = (INIT_ALS | 0x1); //shutdown ALS.

			err = i2c_transfer(cm36283_client->adapter, msg, ARRAY_SIZE(msg));

			if(err < 0)
				printk(DBGMSK_PRX_G0"[cm36283] (%s): reg=0x%x data=0x%x err=%d\n",__FUNCTION__ ,data[0], data[1], err);
			else
				printk(DBGMSK_PRX_G3"[cm36283] reg=0x%x, data=0x%x\n",data[0], data[1]);
			printk("[cm36283][als] turn off\n");
		}
		g_als_threshold_hi = 0;
		g_als_threshold_lo = 0;
        }
    }

    printk(DBGMSK_PRX_G3"[cm36283][als] --Turn on ambient sensor\n");
    return err;

}

static int ambientDev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
    static bool bFirst = true;
    static bool openfilp = true;

    switch (property) 
    {
        case SENSORS_PROP_SWITCH:
		printk(DBGMSK_PRX_G3"[cm36283][als] put SENSORS_PROP_SWITCH (%d,%d).\n", 
                    (val->intval), g_HAL_als_switch_on);

		//read calibration value
		if(bFirst) {
			printk(DBGMSK_PRX_G3"[cm36283][als] put switch 1st read calvalue\n");
			openfilp = read_lightsensor_calibrationvalue();
			/*Surpport old calibration value*/
			if ( g_cm36283_light_gain_calibration >= 10000000 || openfilp == false )	{
				printk("[cm36283][als] Get old calvalue : %d or fail\n",g_cm36283_light_gain_calibration );
				g_cm36283_light_gain_calibration = 38000;
				g_cm36283_light_shift_calibration = 40;
			}else
				read_lightsensor_shiftvalue();
			printk("[cm36283] Set calibration and shift: %d.%d , %d\n"
					, g_cm36283_light_gain_calibration/cm36283_als_calibration_accuracy
					, g_cm36283_light_gain_calibration%cm36283_als_calibration_accuracy
					, g_cm36283_light_shift_calibration );
			bFirst = false;
		}

		if(val->intval > 0)
			g_HAL_als_switch_on = 1;
		else	{
			g_HAL_als_switch_on = 0;
			
			// Clean dev/input/event, 
			// Make sure when sensor trun on sensoreventlistener can detect event have been changed
			als_lux_report_event( -1 );
		}

		if( g_bIsP01Attached )	{
			printk(DBGMSK_PRX_G3"[cm36283][als] sensor switch, turn on/off al3010: %d\n",
                         g_HAL_als_switch_on);

			set_als_power_state_of_P01(g_HAL_als_switch_on);
              	//cm36283_turn_onoff_proxm(0);
		}
		else		{			
              	printk(DBGMSK_PRX_G3"[cm36283][als] sensor switch, turn on/off cm36283: %d\n",
                         g_HAL_als_switch_on);

			cm36283_turn_onoff_als(g_HAL_als_switch_on);
              }
		break;

        case SENSORS_PROP_LO_THRESHOLD:
            /*
            printk(DBGMSK_PRO_G1 "[ambientdl]: config IT_ALS (0x%x)\n", val->intval);
            if(val->intval>=0 && val->intval<=3) {
                gpio_set_value(g_proxm_pwr_pin, 0);
                msleep(1);
                gpio_set_value(g_proxm_pwr_pin, 1);
                gpio_free(g_proxm_pwr_pin);
                als_IT = val->intval;
                ret = cm36283_reset();
            }
            */
            break;

        case SENSORS_PROP_HI_THRESHOLD:
        
            break;

        case SENSORS_PROP_INTERVAL:
            printk(DBGMSK_PRX_G3"[cm36283][als] put SENSORS_PROP_INTERVAL. %d\n", val->intval);
            if(val->intval < 100) {
                g_interval = 100;
            }
            else {
                g_interval = val->intval;
            }
            /*
            printk(DBGMSK_PRO_G1 "[ambientdl]: config GAIN_ALS (0x%x)\n", val->intval);
            //g_interval = val->intval;
            if(val->intval>=0 && val->intval<=3) {
                gpio_set_value(g_proxm_pwr_pin, 0);
                msleep(1);
                gpio_set_value(g_proxm_pwr_pin, 1);
                gpio_free(g_proxm_pwr_pin);
                als_GAIN = val->intval;
                ret = cm36283_reset();
            }
            */
            break;

        /* Add for debug only */
        case SENSORS_PROP_DBG:
            g_ambient_dbg = val->intval;
            printk(DBGMSK_PRX_G3"[cm36283][als] dbg = %d.\n", g_ambient_dbg);
            break;

        case SENSORS_PROP_CALIBRATION:

		/*Old version*/
/*            if(val->intval > 30720)
                g_cm36283_light_gain_calibration = 30720;
            else if(val->intval < 1024)
                g_cm36283_light_gain_calibration = 1024;
            else
*/
		g_cm36283_light_gain_calibration = val->intval;

            printk(DBGMSK_PRX_G3"[cm36283][als] calibration val x1000= %d\n", g_cm36283_light_gain_calibration);

	     /*
            cm36283_write_calvalue = kmalloc(sizeof(struct write_calvalue), GFP_KERNEL);

            // write calvalue work
            INIT_WORK(&cm36283_write_calvalue->write_calvalue_work, write_lightsensor_calibrationvalue_work);

            cm36283_write_calvalue -> calvalue = g_cm36283_light_gain_calibration;

            queue_work(cm36283_workqueue, &cm36283_write_calvalue->write_calvalue_work);
            */
            break;

        default:
            printk(DBGMSK_PRX_G0"[cm36283][als] put default.\n");
            return -EINVAL;
    }
    return 0;
}

static int ambientDev_open(struct inode *inode, struct file *file)
{
    int ret = 0;

    printk(DBGMSK_PRX_G3"[cm36283][als] ambientdl_dev_open \n");

    if (file->f_flags & O_NONBLOCK) {
        printk(DBGMSK_PRX_G2"[cm36283][als] ambientdl_dev_open (O_NONBLOCK)\n");
    }
    atomic_set(&ambient_update, 0); //initialize atomic.

    ret = nonseekable_open(inode, file);

    return ret;
}

static ssize_t ambientDev_read(struct file *file, char __user * buffer, size_t size, loff_t * f_pos)
{
    printk(DBGMSK_PRX_G3"[cm36283]: ambientDev_read\n");
    return 0;
}


static unsigned int ambientDev_poll(struct file *filep, poll_table * wait)
{
//  if(g_ambient_dbg==1)
//  printk("[cm36283]: ambientdl_dev_poll++\n");

    if(g_cm36283_als_switch_on==0) {
//      if(g_ambient_dbg==1)
        printk(DBGMSK_PRX_G0"[cm36283][als] ambientdl_dev_poll-- not ready\n");
        return 0;
    }

    poll_wait(filep, &ambient_wq_head, wait);
    
    if(atomic_read(&ambient_update)) {
//      if(g_ambient_dbg==1)
//      printk("[cm36283]: ambientdl_dev_poll-- (POLLIN %d)\n", atomic_read(&ambient_update));
        return (POLLIN | POLLRDNORM);
    }

//  if(g_ambient_dbg==1)
//  printk("[cm36283]: ambientdl_dev_poll--\n");
    return 0;
}

static int ambientDev_release(struct inode *inode, struct file *filp)
{
    printk(DBGMSK_PRX_G3"[cm36283][als] ambientdl_dev_release.\n");

    return 0;
}


/////////////////////////////////////////////////////////////////////////////////////////
//
//  Low layer driver part
//


//
// Support ATD light sensor test ++
//

int atd_read_P_L_sensor_adc(int *adc)
{
    int status = 0;
    int ret = 0;
    unsigned char alsdata[2] = {0,0};
    //    u16 pData=0; //proximity data.
    int lux = 0;
    u8 lobyte = 0;
    int idx = 0;

    printk(DBGMSK_PRX_G2"[cm36283][atd]readadc: trying to turn on lsensor\n");

    status = proximity_als_turn_on(1);

    if(0 == status) {
        //if(g_ambient_dbg==1)
        printk(DBGMSK_PRX_G0"[cm36283][atd]readadc: lsensor is not on\n");
        return status;
    }

    *adc = 0;

    for(idx = 0; idx < 5; idx++)
    {
        ret = cm36283_read_reg(cm36283_client, CM36283_ALS_DATA, 2, &alsdata); //get LSB data.

        if(ret != 2) {
            printk(DBGMSK_PRX_G0"[cm36283][atd]readadc: reg2 error! (cmd=0x%x)\n",CM36283_ALS_DATA);
            alsdata[0] = 0;
            alsdata[1] = 0;
            ret = -EIO;
        }
        else {
            lobyte = alsdata[0];

            printk(DBGMSK_PRX_G2"[cm36283][atd]readadc: read-LoByte: %d\n",(int)lobyte);
            printk(DBGMSK_PRX_G2"[cm36283][atd]readadc: read-HiByte: %d\n", (int)alsdata[1]);

            *adc = ((alsdata[1]<<8) | lobyte);
        }

        msleep(100);
    }

    lux = (*adc * 25) / 1000; // multiply (0.025*10000) to avoid float value...

    printk(DBGMSK_PRX_G2"[cm36283][atd]readadc steps=%d lux=%d\n", *adc, lux);

    proximity_als_turn_on(0);

    return status;
}

static int proximity_als_turn_on(int bOn)
{ 
    struct i2c_msg msg[1];
    unsigned char pdata[2] = {0,0};
    unsigned char data[2] = {0,0};
    int err = 0;
    int status = 0; // 0x1: light sensor is ok, 0x2: proximity is ok, 0x3: both are ok, 0x0: both failed

    printk(DBGMSK_PRX_G2"[cm36283]proximity_light_turn_on:%d\n", bOn);

    //set als_config 0x00_L
    msg->addr = cm36283_client->addr;
    msg->flags = 0;
    msg->len = 2; //two data byte
    msg->buf = data;
    data[0] = CM36283_ALS_CONF; //command code
    data[1] = ( (als_IT<<6) | (als_PERS<<2) | (als_INT_En << 1) | INIT_ALS ); //0.05 lux/step 

    if(bOn == 0) {
        data[1] |= als_ps_SD;
    }

    err = i2c_transfer(cm36283_client->adapter, msg, ARRAY_SIZE(msg));

    if(err < 0) {
        printk(DBGMSK_PRX_G0"[cm36283] (%s): reg=0x%x, data=0x%x, err=%d\n" ,__FUNCTION__, data[0], data[1], err);
    }
    else {
        printk(DBGMSK_PRX_G2"[cm36283] (%s): reg=0x%x, value=0x%x\n" ,__FUNCTION__, data[0], data[1]);
        status |= 0x1;  //als ok
    }
 
    //set ps_config 0x03_L 0x03_H
    pdata[0] = (ps_DUTY<<6 | ps_IT<<4 | ps_PERS<<2 | INIT_PS);
    pdata[1] = (ps_ITB<<6 | ps_INT | INIT_PS);

    if(bOn == 0) {
        pdata[0] |= als_ps_SD;
        pdata[1] |= (ps_ITB<<6 | INIT_PS);
    }

    err = cm36283_write_reg(CM36283_PS_CONF, pdata[0], pdata[1], 3);

    if(err < 0) {
        printk(DBGMSK_PRX_G0"[cm36283] (%s): err=%d\n",__FUNCTION__, err);
    }
    else {
        printk(DBGMSK_PRX_G2"[cm36283] (%s): set ps_config(0x%x), pdata_l=0x%x, pdata_h=0x%x\n",__FUNCTION__ , CM36283_PS_CONF, pdata[0], pdata[1]);
        status |= 0x02; //ps OK
    }

    printk(DBGMSK_PRX_G2"[cm36283]turn on/off, status:0x%x (bitwise)\n", status);

    return status;
}

static void gpio_proximity_onoff(void)
{
    printk("[cm36283][isr] GPIO PWR ON/OFF: g_HAL_als_switch_on=%d, g_proxm_switch_on=%d ++\n", g_HAL_als_switch_on, g_proxm_switch_on);
    gpio_set_value(CM36283_GPIO_PROXIMITY_PWR_EN, 0);
    msleep(1);
    gpio_set_value(CM36283_GPIO_PROXIMITY_PWR_EN, 1);	
    msleep(1);

    g_cm36283_reset = 1;

    if (g_HAL_als_switch_on == 1) 
	cm36283_turn_onoff_als(1);
    
    if(g_proxm_switch_on == 1) {
	cm36283_turn_onoff_proxm(1);
	queue_work(cm36283_workqueue, &cm36283_proximity_interrupt_work);
    }

    return;
}

static int atd_write_status_and_adc_2(int *als_sts, int *als_adc, int *ps_sts)
{
    int adc = 0;
    int status = 0; // 0x1: light sensor is ok, 0x2: proximity is ok, 0x3: both are ok, 0x0: both failed

    printk(DBGMSK_PRX_G2"[cm36283][atd]writests_2: started\n");

    status = atd_read_P_L_sensor_adc(&adc);

    if(status & 0x01) {
        *als_sts = 1;
        *als_adc = adc;
    }
    else {
        *als_sts = 0;
        *als_adc = 0;
    }

    if(status & 0x02) {
        *ps_sts = 1;
    }
    else {
        *ps_sts = 0;
    }

    printk(DBGMSK_PRX_G2"[cm36283][atd]writests_2: get adc, als_sts:%d, als_adc:%d, ps_sts:%d\n", 
            *als_sts, *als_adc, *ps_sts);

    return status;
}


//
// Support ATD light sensor test --
//

static bool read_lightsensor_calibrationvalue()
{
	struct file *fp = NULL; 
	loff_t pos_lsts = 0;
	char readstr[8];
	int ori_val = 0, readlen =0;
	mm_segment_t old_fs;
	
	printk(DBGMSK_PRX_G3"[cm36283] ++read_lsensor_calvalue open\n");

	fp = filp_open(LSENSOR_CALIBRATION_ASUS_NV_FILE, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	//fp = filp_open(pFile, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G3"[cm36283] read_lsensor_calvalue open (%s) fail\n", LSENSOR_CALIBRATION_ASUS_NV_FILE);
		return false;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	
	if(fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, readstr, 6, &pos_lsts);
		readstr[readlen] = '\0';

		printk(DBGMSK_PRX_G3"[cm36283] strlen:%s(%d)\n", readstr, strlen(readstr));
	}
	else
		printk(DBGMSK_PRX_G0"[cm36283] read_lsensor_calvalue, f_op=NULL or op->read=NULL\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(readstr, "%d", &ori_val);

	//limit the calibration value range
	g_cm36283_light_gain_calibration = ori_val;

	printk("[cm36283] read_lsensor_calvalue: Ori: %d, Cal: %d\n", 
			ori_val, g_cm36283_light_gain_calibration);

	printk("[cm36283] --read_lsensor_calvalue open\n");
	return true;
}

#ifdef ASUS_FACTORY_BUILD
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
		printk(DBGMSK_PRX_G0"[cm36283] write_lsensor_calvalue open (%s) fail\n", LSENSOR_CALIBRATION_ASUS_NV_FILE);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk(DBGMSK_PRX_G3"[cm36283] write_lsensor_calvalue = %d[%s(%d)]\n", 
		this->calvalue, writestr, strlen(writestr));

	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
		printk(DBGMSK_PRX_G0"[cm36283] write_lsensor_calvalue fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}
#endif

//Maggie+++ support proximity calibration

static bool read_prox_hi_calibrationvalue()
{
	struct file *fp = NULL; 
	loff_t pos_lsts = 0;
	char readstr[8];
	int ori_val = 0, readlen =0;
	mm_segment_t old_fs;
	
	printk(DBGMSK_PRX_G3"[cm36283] ++read_psensor_hi_calvalue open\n");

	fp = filp_open(PSENSOR_CALIBRATION_HI_ASUS_NV_FILE, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	//fp = filp_open(pFile, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G3"[cm36283] read_psensor_hi_calvalue open (%s) fail\n", PSENSOR_CALIBRATION_HI_ASUS_NV_FILE);
		return false;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	
	if(fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, readstr, 6, &pos_lsts);
		readstr[readlen] = '\0';

		printk(DBGMSK_PRX_G3"[cm36283] strlen:%s(%d)\n", readstr, strlen(readstr));
	}
	else
		printk(DBGMSK_PRX_G0"[cm36283] read_psensor_hi_calvalue, f_op=NULL or op->read=NULL\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(readstr, "%d", &ori_val);

	//limit the calibration value range
	g_ps_threshold_hi = ori_val;

	printk("[cm36283] read_psensor_hi_calvalues: Ori: %d, Cal: %d\n", 
			ori_val, g_ps_threshold_hi);

	printk("[cm36283] --read_psensor_hi_calvalue open\n");
	return true;
}

#ifdef ASUS_FACTORY_BUILD
static void write_prox_hi_calibrationvalue_work(struct work_struct *work)
{
	struct file *fp = NULL; 
	struct write_prox_hi *this = NULL;
	loff_t pos_lsts = 0;
	char writestr[8];
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	this = container_of(work, struct write_prox_hi, write_prox_hi_work);

	fp = filp_open(PSENSOR_CALIBRATION_HI_ASUS_NV_FILE, O_RDWR|O_CREAT|O_TRUNC, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G0"[cm36283] write_psensor_hi_calvalue open (%s) fail\n", PSENSOR_CALIBRATION_HI_ASUS_NV_FILE);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk(DBGMSK_PRX_G3"[cm36283] write_psensor_hi_calvalue = %d[%s(%d)]\n", 
		this->calvalue, writestr, strlen(writestr));

	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
		printk(DBGMSK_PRX_G0"[cm36283] write_psensor_hi_calvalue fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}

static int cm36283_show_calibration_prox_hi(struct device *dev, struct device_attribute *attr, char *buf)
{	
	read_prox_hi_calibrationvalue();
	
	printk(DBGMSK_PRX_G2"[cm36283] Show_prox_hi_calibration: %d\n", g_ps_threshold_hi);
	
	return sprintf(buf, "%d\n", g_ps_threshold_hi);
}

static int cm36283_store_calibration_prox_hi(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	
	if ( (strict_strtoul(buf, 10, &val) < 0) )
		return -EINVAL;
	
	/*Get hi threshold adc*/
	a_ps_hi_calibration_adc = (int)val;

	printk("[cm36283] Get calibration_prox_hi value : %d\n", a_ps_hi_calibration_adc );

	/*Write Calibration value*/
	cm36283_write_prox_hi = kmalloc(sizeof(struct write_calvalue), GFP_KERNEL);

	INIT_WORK(&cm36283_write_prox_hi->write_prox_hi_work, write_prox_hi_calibrationvalue_work);

	cm36283_write_prox_hi -> calvalue = a_ps_hi_calibration_adc;

	queue_work(cm36283_workqueue, &cm36283_write_prox_hi->write_prox_hi_work);

	return a_ps_hi_calibration_adc;
}

static DEVICE_ATTR(calibration_prox_hi, S_IRWXU | S_IRWXG| S_IRWXO,
		   cm36283_show_calibration_prox_hi, cm36283_store_calibration_prox_hi);
#endif

static bool read_prox_lo_calibrationvalue()
{
	struct file *fp = NULL; 
	loff_t pos_lsts = 0;
	char readstr[8];
	int ori_val = 0, readlen =0;
	mm_segment_t old_fs;
	
	printk(DBGMSK_PRX_G3"[cm36283] ++read_psensor_low_calvalue open\n");

	fp = filp_open(PSENSOR_CALIBRATION_LO_ASUS_NV_FILE, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	//fp = filp_open(pFile, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G3"[cm36283] read_psensor_low_calvalue open (%s) fail\n", PSENSOR_CALIBRATION_LO_ASUS_NV_FILE);
		return false;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	
	if(fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, readstr, 6, &pos_lsts);
		readstr[readlen] = '\0';

		printk(DBGMSK_PRX_G3"[cm36283] strlen:%s(%d)\n", readstr, strlen(readstr));
	}
	else
		printk(DBGMSK_PRX_G0"[cm36283] read_psensor_low_calvalue, f_op=NULL or op->read=NULL\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(readstr, "%d", &ori_val);

	//limit the calibration value range
	g_ps_threshold_lo = ori_val;

	printk("[cm36283] read_psensor_low_calvalue: Ori: %d, Cal: %d\n", 
			ori_val, g_ps_threshold_lo);

	printk("[cm36283] --read_psensor_low_calvalue open\n");
	return true;
}

#ifdef ASUS_FACTORY_BUILD
static void write_prox_lo_calibrationvalue_work(struct work_struct *work)
{
	struct file *fp = NULL; 
	struct write_prox_lo *this = NULL;
	loff_t pos_lsts = 0;
	char writestr[8];
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	this = container_of(work, struct write_prox_lo, write_prox_lo_work);

	fp = filp_open(PSENSOR_CALIBRATION_LO_ASUS_NV_FILE, O_RDWR|O_CREAT|O_TRUNC, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G0"[cm36283] write_psensor_low_calvalue open (%s) fail\n", PSENSOR_CALIBRATION_LO_ASUS_NV_FILE);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk(DBGMSK_PRX_G3"[cm36283] write_psensor_low_calvalue = %d[%s(%d)]\n", 
		this->calvalue, writestr, strlen(writestr));

	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
		printk(DBGMSK_PRX_G0"[cm36283] write_psensor_low_calvalue fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}


static int cm36283_show_calibration_prox_lo(struct device *dev, struct device_attribute *attr, char *buf)
{	
	read_prox_lo_calibrationvalue();

	printk(DBGMSK_PRX_G2"[cm36283] Show_prox_lo_calibration: %d\n", g_ps_threshold_lo);
	
	return sprintf(buf, "%d\n", g_ps_threshold_lo);
}

static int cm36283_store_calibration_prox_lo(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	
	if ( (strict_strtoul(buf, 10, &val) < 0) )
		return -EINVAL;
	
	/*Get hi threshold adc*/
	a_ps_lo_calibration_adc = (int)val;

	printk("[cm36283] Get calibration_prox_hi value : %d\n", a_ps_lo_calibration_adc );

	/*Write Calibration value*/
	cm36283_write_prox_lo = kmalloc(sizeof(struct write_calvalue), GFP_KERNEL);

	INIT_WORK(&cm36283_write_prox_lo->write_prox_lo_work, write_prox_lo_calibrationvalue_work);

	cm36283_write_prox_lo -> calvalue = a_ps_lo_calibration_adc;

	queue_work(cm36283_workqueue, &cm36283_write_prox_lo->write_prox_lo_work);

	return a_ps_lo_calibration_adc;
}

static DEVICE_ATTR(calibration_prox_lo, S_IRWXU | S_IRWXG | S_IRWXO,
		   cm36283_show_calibration_prox_lo, cm36283_store_calibration_prox_lo);
#endif
//Maggie ---

static int proxm_set_hi_threshold(int value)
{
    int err = 0;
    u8 data_l, data_h;

    printk(DBGMSK_PRX_G4"[cm36283][ps] set high threshold: %d\n", value);

    data_l = g_ps_threshold_lo;
    data_h = value;

    err = cm36283_write_reg(CM36283_PS_THD, data_l, data_h, 3);

    if(err < 0) {
        printk(DBGMSK_PRX_G0"[cm36283]: (%s): high_thd=%d err=%d\n", __FUNCTION__, data_h, err);
        printk(DBGMSK_PRX_G0"[cm36283]: WARNING! Please check sensor is power on!\n");
    }
    else {
        printk(DBGMSK_PRX_G4"[cm36283] (%s): set ps_threshold, data_l=%d, data_h=%d\n",__FUNCTION__, data_l, data_h);
    }

    return err;
}

static int proxm_set_lo_threshold(int value)
{
    int err = 0;
    u8 data_l, data_h;

    printk(DBGMSK_PRX_G4"[cm36283][ps] set low threshold: %d\n", value);

    data_l = value;
    data_h = g_ps_threshold_hi;
        
    err = cm36283_write_reg(CM36283_PS_THD, data_l, data_h, 3);

    if(err < 0) {
        printk(DBGMSK_PRX_G0"[cm36283]: (%s): low_thd=%d, err=%d\n", __FUNCTION__, data_l, err);
        printk(DBGMSK_PRX_G0"[cm36283]: WARNING! Please check sensor is power on!\n");
    }
    else {
        printk(DBGMSK_PRX_G4"[cm36283] (%s): set ps_threshold, data_l=%d, data_h=%d\n",__FUNCTION__, data_l, data_h);
    }

    return err;
}

static irqreturn_t als_proxm_interrupt_handler(int irq, void *dev_id)
{
	printk(DBGMSK_PRX_G2"[cm36283][isr] interrupt handler ++\n");

	queue_work(cm36283_workqueue, &cm36283_ISR_work);
	if (g_proxim_state == 1)
                wake_lock_timeout(&proximity_wake_lock, 1 * HZ);

	printk(DBGMSK_PRX_G2"[cm36283][isr] interrupt handler --\n");

	return IRQ_HANDLED;
}


static void cm36283_interrupt_handler(struct work_struct *work)
{
	int ret = 0;
	unsigned char buff[2] = {0,0};

	if ((g_proxm_switch_on == 1) || (g_HAL_als_switch_on == 1) || (g_ambient_suspended == 1))	{	//consider autosuspend in phone mode
		//qup_i2c_resume(proximity_dev);
		ret = cm36283_read_reg(cm36283_client, CM36283_INT_FLAGS, 2, &buff);
		if( (buff[1] & 0x30) != 0 || (buff[1] & 0x03) != 0 || (buff[1] & 0x40) != 0 ) {
			printk(DBGMSK_PRX_G2"[cm36283][isr] INT_FLAG =(0x%x)\n", buff[1]);
		}else if (g_ambient_suspended)	{
			printk("[cm36283][isr] Resume & clean interrupt INT_FLAG =(0x%x)\n", buff[1]);
		}else{
			printk("[cm36283][isr] INT_FLAG =(0x%x), i2c error retry\n",buff[1]);
			gpio_proximity_onoff();
		}
	}

	if((buff[1] & 0x30) != 0)
		queue_work(cm36283_workqueue, &cm36283_light_interrupt_work);
	else if ((buff[1] & 0x03) != 0) {
		if (g_ambient_suspended==1) {
			printk("[cm36283][isr] setting g_cm36283_earlysuspend_int = %d\n", g_cm36283_earlysuspend_int);
			g_cm36283_earlysuspend_int = 1;
		}
		queue_work(cm36283_workqueue, &cm36283_proximity_interrupt_work);
	}
	else if ((buff[1] & 0x40) != 0)
		printk(DBGMSK_PRX_G2"[cm36283][isr] PS1 entering protection mode\n");

	return;
}

static void light_interrupt_work(struct work_struct *work)
{
	int err = 0;

	get_adc_calibrated_lux_from_cm36283();

	g_als_threshold_hi = g_cm36283_light_adc*105/100;
	g_als_threshold_lo = g_cm36283_light_adc*95/100;

	//set interrupt high threshold
	err = cm36283_write_reg(CM36283_ALS_THDH, (g_als_threshold_hi & 0xFF), ((g_als_threshold_hi>> 8 ) & 0xFF), 3);

	if(err < 0)
		printk("[cm36283][als] (%s): reg=0x%x, err=%d\n",__FUNCTION__,CM36283_ALS_THDH ,err);
	else
		printk(DBGMSK_PRX_G3"[cm36283][als] high threshold: ldata_lo=%d, ldata_hi=%d\n",
					(g_als_threshold_hi & 0xFF), ((g_als_threshold_hi >> 8 ) & 0xFF) );


	//set interrupt low threshold
	err = cm36283_write_reg(CM36283_ALS_THDL, (g_als_threshold_lo & 0xFF), ((g_als_threshold_lo >> 8 )& 0xFF), 3);

	if(err < 0)
		printk("[cm36283][als] (%s): addr=0x%x, err=%d\n",__FUNCTION__, CM36283_ALS_THDH, err);
	else
		printk(DBGMSK_PRX_G3"[cm36283][als] low threshold: ldata_lo=%d, ldata_hi=%d\n",
					(g_als_threshold_lo & 0xFF), ((g_als_threshold_lo >> 8 )& 0xFF));

	if(g_cm36283_light != g_last_cm36283_light || g_cm36283_light_first) {
		g_last_cm36283_light = g_cm36283_light;
		als_lux_report_event( g_cm36283_light);
		g_cm36283_light_first=0;
	}

	if(g_proxm_switch_on==1) {
		if (g_ambient_suspended==1) {
			printk("[cm36283][isr] setting g_cm36283_earlysuspend_int = %d\n", g_cm36283_earlysuspend_int);
			g_cm36283_earlysuspend_int = 1;
		}
		queue_work(cm36283_workqueue, &cm36283_proximity_interrupt_work);
	}
	
	if(g_proxim_state == 1) 
		wake_unlock(&proximity_wake_lock);
	return;
}

static void proximity_interrupt_work(struct work_struct *work)
{
	int ret;
	unsigned char buff[2] = {0,0};

	mutex_lock(&g_cm36283_data_ps->lock);
	ret = cm36283_read_reg(cm36283_client, CM36283_PS_DATA, 2, &buff);
	g_psData = buff[0];

	printk(DBGMSK_PRX_G4"/----------------------------------------------------\n");
	printk(DBGMSK_PRX_G4"[cm36283][ps] PS_data = %d\n",g_psData);

	if(buff[0] >= g_ps_threshold_hi) {  //panel off
		input_report_abs(g_cm36283_data_ps->input_dev, ABS_DISTANCE, 0);
		input_sync(g_cm36283_data_ps->input_dev);
		g_proxim_state = 1;
		printk("[cm36283][ps] trigger panel off\n");
	}else	{
		input_report_abs(g_cm36283_data_ps->input_dev, ABS_DISTANCE, 1);
		input_sync(g_cm36283_data_ps->input_dev);
		wake_unlock(&proximity_wake_lock);
		g_proxim_state = 0;
		printk("[cm36283][ps] trigger panel on\n");
	}

#ifndef INPUT_EVENT_MODE
	atomic_inc(&proxm_update);
	printk(DBGMSK_PRX_G4"[cm36283][ps] proxm_interrupt_handler, state_change\n");
	printk(DBGMSK_PRX_G4"[cm36283][ps] proxm_interrupt_handler fire(%d)\n",atomic_read(&proxm_update));
	printk(DBGMSK_PRX_G4"/----------------------------------------------------\n");
#endif

	mutex_unlock(&g_cm36283_data_ps->lock);
	return;
}

static void Proximity_test_delayed_work(struct work_struct *work)
{
	int ret;
	unsigned char buff[2] = {0,0};

	if ( g_proxm_dbg == 2 )
		queue_work(cm36283_workqueue, &cm36283_proximity_interrupt_work);
	else if ( g_proxm_dbg == 1 )	{
		ret = cm36283_read_reg(cm36283_client, CM36283_PS_DATA, 2, &buff);
		printk(DBGMSK_PRX_G4"[cm36283][ps] PS_data = %d\n",buff[0]);
	}
	if ( g_proxm_dbg > 0 )
		queue_delayed_work(Proximity_test_wq, &Proximity_test_work, HZ * PROXIMITY_TEST_DELAY);
	else
		cancel_delayed_work(&Proximity_test_work);
}

static int get_adc_calibrated_lux_from_cm36283(void)
{
	unsigned char buff[2] = {0,0};
	int ret = 0;
	u16 tmp = g_cm36283_light_adc;
	u8 lobyte = 0;
	u8 hibyte = 0;

	//0x09 CM36283_ALS_DATA_REG
	ret = cm36283_read_reg(cm36283_client, CM36283_ALS_DATA, 2, &buff); //get ALS data.
	if(ret != 2) {
		printk("[cm36283][als] ambientdl_dev_read. error! (cmd=0x%x)\n",CM36283_ALS_DATA);
		g_cm36283_light_adc = tmp;
		ret = -EIO;
	}        else	{
		lobyte = buff[0];
		hibyte = buff[1];
		// adc is the value that is returned from HW directly ( Original adb number )
		g_cm36283_light_adc = ((hibyte<<8) | lobyte);
		printk(DBGMSK_PRX_G3"[cm36283][als] read-LoByte (%d). read-HiByte (%d)\n",lobyte, hibyte);
	}

	//apply calibration value
	g_cm36283_light_k_adc = (int)(g_cm36283_light_adc * g_cm36283_light_gain_calibration
		/ cm36283_als_calibration_accuracy  + g_cm36283_light_shift_calibration);


	/*Get Lux*/
	if ( g_cm36283_light_k_adc <= 0 )
		g_cm36283_light = 0;
	else
		g_cm36283_light = g_cm36283_light_k_adc;	

	if ( g_cm36283_light > g_cm36283_light_map[ g_max_light_level -1 ])
		g_cm36283_light = g_cm36283_light_map[ g_max_light_level -1 ];
	printk(DBGMSK_PRX_G3"[cm36283][als] read adc: %d, cal adc: %d,\n", g_cm36283_light_adc, g_cm36283_light_k_adc);
	printk(DBGMSK_PRX_G3"[cm36283][als] last Lux=%d, light=%d \n", g_last_cm36283_light, g_cm36283_light );

	return g_cm36283_light_adc;
}

/*
void als_lux_report_event(int lux)
{       
        printk("[cm36283][als] ******* report lux = %d\n",lux);

#ifndef INTERNAL_TEST
        input_report_abs(g_cm36283_data_as->input_dev, ABS_MISC, lux);
#else
      //  lux = ret;  //ret value is not 0 only under internal test 
        input_report_abs(g_cm36283_data_as->input_dev, ABS_MISC, get_adc_calibrated_lux_from_cm36283);
#endif
        input_event(g_cm36283_data_as->input_dev, EV_SYN, SYN_REPORT, 1);
        input_sync(g_cm36283_data_as->input_dev);

#ifndef INPUT_EVENT_MODE
        atomic_inc(&ambient_update);
        if(g_ambient_dbg==1)
		printk(DBGMSK_PRX_G3"[cm36283][als] ambient_poll_work fire(%d)\n",atomic_read(&ambient_update));
        wake_up_interruptible(&ambient_wq_head);
#endif
}    
EXPORT_SYMBOL(als_lux_report_event);
*/

/**
 * cm36283_read_reg - read data from cm36283
 * @i2c_client: context of i2c client of cm36283
 * @reg: the target register
 *
 * Returns negative errno, else the number of messages executed.
 */

static int cm36283_read_reg(struct i2c_client* client, u8 reg, int len, void *data)
{
    int err = 0;

    struct i2c_msg msg[] = {
        {
            .addr = cm36283_client->addr,
            .flags = 0, //write
            .len = 1,
            .buf = &reg,
        },
        {
            .addr = cm36283_client->addr,
            .flags = I2C_M_RD, //read
            .len = len,
            .buf = data,
        }
    };

    if (!client->adapter) {
        return -ENODEV;
    }

    memset(&data, 0, sizeof(data));

    err = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));

    if (err != ARRAY_SIZE(msg))
        printk(DBGMSK_PRX_G0"[cm36283] cm36283_read_reg err %d\n", err);

    return err; // return 2 is expected.
}

/**
 * cm36283_write_reg - write an I2C message to cm36283
 * @i2c_client: context of i2c client of cm36283
 * @reg: the target register
 * @val: the value will be wrote
 *
 * Returns negative errno, else the number of messages executed.
 */

static int cm36283_write_reg(u8 reg, int data_l, int data_h, int len)
{
	int err = 0;
	uint8_t buf[len];

	static struct i2c_msg msg;

	msg.addr = cm36283_client->addr;
	msg.flags = 0; //write
	msg.len = len;
	msg.buf = buf;

	if (!cm36283_client->adapter)
		return -ENODEV;

	buf[0] = reg;
	memcpy(buf + 1, &data_l, sizeof(data_l));
	memcpy(buf + 2, &data_h, sizeof(data_h));

	err = i2c_transfer(cm36283_client->adapter, &msg, 1);

	if(err < 0)
		printk(DBGMSK_PRX_G0"[cm36283] cm36283_write_reg err: reg=0x%x, data_l=%d, data_h=%d, err = 0x%x\n", reg, data_l, data_h, err);

	//else {
	//	printk("[cm36283] reg=0x%x, data_l=%d, data_h=%d\n", reg, data_l, data_h);
	//}
    return err; // return postive is expected.
}


/**
 * cm36283_reset - reset cm36283
 *
 * Returns negative errno, else the number of messages executed.
 */

static int cm36283_reset(void)
{
    int err = 0;
    u8 data_l = 0;
    u8 data_h = 0;

    printk(DBGMSK_PRX_G2"[cm36283] cm36283_reset ++\n");
   // printk("[cm36283]: proxm_int_pin=%d \n",gpio_get_value(g_proxm_int_pin));

    if (!cm36283_client->adapter) {
        return -ENODEV;
    }

    //0x00
    data_l = (u8) g_switch[1];
    data_l = ~(data_l);
    data_l = (data_l & 0x1);
    data_l = (INIT_ALS | data_l);
    data_l = ((als_IT<<6) | (als_PERS<<2) | data_l);

    data_h = (u8) g_switch[1];

    err = cm36283_write_reg(CM36283_ALS_CONF, data_l, data_h, 3);
    
    if(err < 0) {
        printk(DBGMSK_PRX_G0"[cm36283] (%s): reg=0x%x, config=0x%x, err=%d\n", __FUNCTION__, CM36283_ALS_CONF, data_l, err);
    }
    else {
        printk(DBGMSK_PRX_G2"[cm36283] (%s): reg=0x%x, config=0x%x \n", __FUNCTION__, CM36283_ALS_CONF, data_l);
    }

    //0x01
    err = cm36283_write_reg(CM36283_ALS_THDH, (g_als_threshold_hi & 0xFF), ((g_als_threshold_hi >> 8 )& 0xFF), 3);

    if(err < 0) {
        printk(DBGMSK_PRX_G0"[cm36283] (%s): reg=0x%x, err=%d\n", __FUNCTION__, CM36283_ALS_THDH ,err);
    }
    else {
        printk(DBGMSK_PRX_G2"[cm36283] (%s): als high threshold(0x%x): ldata_l=%d, ldata_h=%d\n", __FUNCTION__
                , CM36283_ALS_THDH , (g_als_threshold_hi & 0xFF), ((g_als_threshold_hi >> 8 )& 0xFF));
    }

    //0x02
    err = cm36283_write_reg(CM36283_ALS_THDL, (g_als_threshold_lo & 0xFF), ((g_als_threshold_lo >> 8 )& 0xFF), 3);

    if(err < 0) {
        printk(DBGMSK_PRX_G0"[cm36283] (%s): addr=0x%x, err=%d\n", __FUNCTION__, CM36283_ALS_THDH, err);
    }
    else {
        printk(DBGMSK_PRX_G2"[cm36283] (%s): als low threshold(0x%x): ldata_l=%d, ldata_h=%d\n", __FUNCTION__
                , CM36283_ALS_THDL, (g_als_threshold_lo & 0xFF), ((g_als_threshold_lo >> 8 )& 0xFF));
    }

    //0x03
    data_l = (u8) g_switch[0];
    data_l = ~(data_l);
    data_l = data_l & 0x01;
    data_l |= INIT_PS;
    data_l = (ps_DUTY<<6 | ps_IT<<4 | ps_PERS<<2 | data_l);
    data_h = (ps_ITB<<6 | INIT_PS);

    err = cm36283_write_reg(CM36283_PS_CONF, data_l, data_h, 3);

    if(err < 0) {
        printk(DBGMSK_PRX_G0"[cm36283] (%s): reg=0x%x, err=%d\n",__FUNCTION__, CM36283_PS_CONF, err);
    }
    else {
        printk(DBGMSK_PRX_G2"[cm36283] (%s): set ps_config(0x%x), data_l=0x%x, data_h=0x%x\n",__FUNCTION__, CM36283_PS_CONF, data_l, data_h);
    }
    
    //0x04
    data_l = ((ps_SM_PERS<<4) | (ps_FOR<<3) | (ps_FOR_Tri<<2) | INIT_PS);
    data_h = ((ps_MS<<6) | INIT_PS);
    
    err = cm36283_write_reg(CM36283_PS_MODE, data_l, data_h, 3);
    if(err < 0) {
        printk(DBGMSK_PRX_G0"[cm36283] (%s): err=%d\n",__FUNCTION__, err);
    }
    else {
        printk(DBGMSK_PRX_G2"[cm36283] (%s): set ps_config(0x%x), data_l=0x%x, data_h=0x%x\n",__FUNCTION__, CM36283_PS_MODE, data_l, data_h);
    }

    //0x06
    data_l = g_ps_threshold_lo;
    data_h = g_ps_threshold_hi;
        
    err = cm36283_write_reg(CM36283_PS_THD, data_l, data_h, 3);
    if(err < 0) {
        printk(DBGMSK_PRX_G0"[cm36283] (%s): err=%d\n",__FUNCTION__, err);
    }
    else {
        printk(DBGMSK_PRX_G2"[cm36283] (%s): set ps_threshold, data_l=%d, data_h=%d\n",__FUNCTION__, data_l, data_h);
    }

    printk(DBGMSK_PRX_G2"[cm36283] cm36283_reset --\n");
    return err;     //return 3 is expected
}

static bool read_lightsensor_shiftvalue()
{
    struct file *fp = NULL; 
    loff_t pos_lsts = 0;
    char readstr[8];
    int ori_val = 0, readlen =0;
    mm_segment_t old_fs;

    printk(DBGMSK_PRX_G3"[cm36283] ++read_lsensor_shift open\n");

    fp = filp_open(LSENSOR_CALIBRATION_SHIFT_ASUS_NV_FILE, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
    //fp = filp_open(pFile, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
    if(IS_ERR_OR_NULL(fp)) {
        printk(DBGMSK_PRX_G3"[cm36283] read_lsensor_shift open (%s) fail\n", LSENSOR_CALIBRATION_SHIFT_ASUS_NV_FILE);
        return false;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    if(fp->f_op != NULL && fp->f_op->read != NULL) {
        pos_lsts = 0;
        readlen = fp->f_op->read(fp, readstr, 6, &pos_lsts);
        readstr[readlen] = '\0';

        printk(DBGMSK_PRX_G3"[cm36283] strlen:%s(%d)\n", readstr, strlen(readstr));
    }
    else {
        printk(DBGMSK_PRX_G0"[cm36283] read_lsensor_shift, f_op=NULL or op->read=NULL\n");
    }

    set_fs(old_fs);
    filp_close(fp, NULL);

    sscanf(readstr, "%d", &ori_val);

    //limit the calibration value range
    g_cm36283_light_shift_calibration = ori_val;

    printk("[cm36283] read_lsensor_shift: Ori: %d, Cal: %d\n", 
            ori_val, g_cm36283_light_shift_calibration);

    printk("[cm36283] --read_lsensor_calvalue open\n");
    return true;
}

#ifdef ASUS_FACTORY_BUILD
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
		printk(DBGMSK_PRX_G0"[cm36283] write_lsensor_shift open (%s) fail\n", LSENSOR_CALIBRATION_SHIFT_ASUS_NV_FILE);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk(DBGMSK_PRX_G3"[cm36283] write_lsensor_shift = %d[%s(%d)]\n", 
		this->calvalue, writestr, strlen(writestr));

	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
		printk(DBGMSK_PRX_G0"[cm36283] write_lsensor_shift fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}

/* calibration */
static int cm36283_show_calibration_200(struct device *dev, struct device_attribute *attr, char *buf)
{	
	read_lightsensor_calibrationvalue();
	
	printk(DBGMSK_PRX_G5"[cm36283] Show_gait_calibration: %d.%d \n", 
					g_cm36283_light_gain_calibration/cm36283_als_calibration_accuracy,
					g_cm36283_light_gain_calibration%cm36283_als_calibration_accuracy );

	return sprintf(buf, "%d.%d\n"
				, g_cm36283_light_gain_calibration/cm36283_als_calibration_accuracy
				, g_cm36283_light_gain_calibration%cm36283_als_calibration_accuracy);
}

static int cm36283_store_calibration_200(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	
	if ( (strict_strtoul(buf, 10, &val) < 0) )
		return -EINVAL;
	
	/*Get low brightness adc*/
	a_als_low_calibration_adc = (int)val;

	printk("[cm36283] Get low calibration adc value : %d\n", a_als_low_calibration_adc );

	return a_als_low_calibration_adc;
	
}

static DEVICE_ATTR(calibration_200, S_IRWXU | S_IRWXG| S_IRWXO,
		   cm36283_show_calibration_200, cm36283_store_calibration_200);


static int cm36283_show_calibration_1000(struct device *dev, struct device_attribute *attr, char *buf)
{	
	read_lightsensor_shiftvalue();

	printk(DBGMSK_PRX_G2"[cm36283] Show_shift_calibration: %d\n", g_cm36283_light_shift_calibration );
	
	return sprintf(buf, "%d\n", g_cm36283_light_shift_calibration );
}

static int cm36283_store_calibration_1000(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	
	if ( (strict_strtoul(buf, 10, &val) < 0) )
		return -EINVAL;
	
	/*Get low brightness adc*/
	a_als_high_calibration_adc = (int)val;

	printk("[als_P01] al3010 Get Hight calibration adc value : %d\n", a_als_high_calibration_adc );

	/*Calibration operation*/
	g_cm36283_light_gain_calibration = 
		(a_als_calibration_lux*cm36283_als_calibration_accuracy) /
				( a_als_high_calibration_adc - a_als_low_calibration_adc );

	g_cm36283_light_shift_calibration = 
		1000 - ( a_als_high_calibration_adc*g_cm36283_light_gain_calibration/cm36283_als_calibration_accuracy );

	/*Write Calibration value*/
	cm36283_write_calvalue = kmalloc(sizeof(struct write_calvalue), GFP_KERNEL);

	INIT_WORK(&cm36283_write_calvalue->write_calvalue_work, write_lightsensor_calibrationvalue_work);

	cm36283_write_calvalue -> calvalue = g_cm36283_light_gain_calibration;

	queue_work(cm36283_workqueue, &cm36283_write_calvalue->write_calvalue_work);
	
	/*Write shift value*/
	cm36283_write_shift = kmalloc(sizeof(struct write_shift), GFP_KERNEL);

	INIT_WORK(&cm36283_write_shift->write_shift_work, write_lightsensor_shiftvalue_work);

	cm36283_write_shift -> calvalue = g_cm36283_light_shift_calibration;

	queue_work(cm36283_workqueue, &cm36283_write_shift->write_shift_work);

	return a_als_high_calibration_adc;
}

static DEVICE_ATTR(calibration_1000, S_IRWXU | S_IRWXG | S_IRWXO,
		   cm36283_show_calibration_1000, cm36283_store_calibration_1000);
#endif
/* adc */
static int cm36283_show_adc(struct device *dev, struct device_attribute *attr, char *buf)
{
	int adc = 0;
	unsigned char buff[2] = {0,0};
	int ret = 0;
	u8 lobyte = 0;
	u8 hibyte = 0;

	ret = cm36283_read_reg(cm36283_client, CM36283_ALS_DATA, 2, &buff); //get ALS data.
	if(ret != 2) {
		printk(DBGMSK_PRX_G3"[cm36283][als] ambientdl_dev_read. error! (cmd=0x%x)\n",CM36283_ALS_DATA);
		ret = -EIO;
	}        else	{
		lobyte = buff[0];
		hibyte = buff[1];
		adc = (int)((hibyte<<8) | lobyte);
		printk(DBGMSK_PRX_G3"[cm36283][als] read-LoByte (%d). read-HiByte (%d)\n",lobyte, hibyte);
	}

	return sprintf(buf, "%d\n", adc);
}

static DEVICE_ATTR(adc, S_IRWXU | S_IRWXG | S_IROTH, cm36283_show_adc, NULL);

static int cm36283_show_proxm (struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned char buff[2] = {0,0};

	ret = cm36283_read_reg(cm36283_client, CM36283_PS_DATA, 2, &buff);
	g_psData = buff[0];

	printk(DBGMSK_PRX_G4"/----------------------------------------------------\n");
	printk(DBGMSK_PRX_G4"[cm36283][ps] PS_data = %d\n",g_psData);
	printk(DBGMSK_PRX_G4"/----------------------------------------------------\n");	

	return sprintf(buf, "%d\n", g_psData);
}

static DEVICE_ATTR(proxm, S_IRWXU | S_IRWXG  | S_IROTH, cm36283_show_proxm, NULL);

static struct attribute *cm36283_attributes[] = {
#ifdef ASUS_FACTORY_BUILD
	&dev_attr_calibration_200.attr,
	&dev_attr_calibration_1000.attr,
	&dev_attr_calibration_prox_lo.attr,
	&dev_attr_calibration_prox_hi.attr,
#endif
	&dev_attr_adc.attr,
	&dev_attr_proxm.attr,
	NULL
};

static const struct attribute_group cm36283_attr_group = {
    .name = "cm36283",
	.attrs = cm36283_attributes,
};

#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>

#define I2C_TEST_Lsensor_FAIL (-1)
#define I2C_TEST_Psensor_FAIL (-1)

static int i2c_stresstest_turn_on_als(bool bOn)
{
	struct i2c_msg msg[1];
	unsigned char data[2] = {0,0};
	int err = 0;

	//bOn always on
	if(g_cm36283_als_switch_on != bOn) {
		//set als_config 0x00_L
		msg->addr = cm36283_client->addr;
		msg->flags = 0;
		msg->len = 2; //two data byte
		msg->buf = data;

		data[0] = CM36283_ALS_CONF; //command code
		data[1] = ( (als_IT<<6) | (als_PERS<<2) |INIT_ALS ); //turn on 0.1 lux/step without interrupt

		err = i2c_transfer(cm36283_client->adapter, msg, ARRAY_SIZE(msg));

		if(err < 0)
			return err;
		g_cm36283_als_switch_on = 1;
	}
	return 0;
}

static int i2c_stresstest_turn_on_proxm(bool bOn)
{
	int err = 0;
	unsigned char idata[2] = {0,0};

	if ( bOn == 1 )	{
		//set 0x03_L, 0x03_H
		idata[0] = (ps_DUTY<<6 | ps_IT<<4 | ps_PERS<<2 | INIT_PS);
		idata[1] = INIT_PS;	//turn on without interrupt

		err = cm36283_write_reg(CM36283_PS_CONF, idata[0], idata[1], 3);

		if(err < 0)
			return -1;
	}

	return 0;
}

static int TestCm36283SensorI2C (struct i2c_client *apClient)
{
	int lnResult = I2C_TEST_PASS;
	int err = 0;
	unsigned char buff[2] = {0,0};

	i2c_log_in_test_case("TestLSensorI2C ++\n");
	
	if( !g_bIsP01Attached )	{
		/* Light sensor */
		if (!g_HAL_als_switch_on)	{
			printk(DBGMSK_PRX_G3"[cm36283][als] Turn on Cm36283\n");
			err = i2c_stresstest_turn_on_als(g_HAL_als_switch_on);
			if(err < 0)	{
				i2c_log_in_test_case("Fail to turn on cm36283 lsensor\n");
				lnResult = I2C_TEST_Lsensor_FAIL;
				goto error_1;
			}
			err = cm36283_read_reg(cm36283_client, CM36283_ALS_DATA, 2, &buff);
			if(err < 0)	{
				i2c_log_in_test_case("Fail to read cm36283 lsensor data\n");
				lnResult = I2C_TEST_Lsensor_FAIL;
				goto error_1;
			}
			else	{
				err = cm36283_turn_onoff_als(0);
				if(err < 0)	{
					i2c_log_in_test_case("Fail to turn off cm36283 lsensor\n");
					lnResult = I2C_TEST_Lsensor_FAIL;
					goto error_1;
				}
			}
		}else	{
			err = cm36283_read_reg(cm36283_client, CM36283_ALS_DATA, 2, &buff);
			if(err < 0)	{
				i2c_log_in_test_case("Fail to read cm36283 lsensor data\n");
				lnResult = I2C_TEST_Lsensor_FAIL;
				goto error_1;
			}
			else	{
				err = cm36283_turn_onoff_als(0);
				if(err < 0)	{
					i2c_log_in_test_case("Fail to turn off cm36283 lsensor\n");
					lnResult = I2C_TEST_Lsensor_FAIL;
					goto error_1;
				}
			}
		}
		
		/* Proximity sensor */
		if ( g_proxm_switch_on )	{
			cm36283_turn_onoff_proxm(0);
			printk(DBGMSK_PRX_G3"[cm36283][ps] Turn on Cm36283\n");
			err = i2c_stresstest_turn_on_proxm(1);
			if(err < 0)	{
				i2c_log_in_test_case("Fail to turn on Psensor\n");
				lnResult = I2C_TEST_Psensor_FAIL;
				goto error_1;
			}
			err = cm36283_read_reg(cm36283_client, CM36283_PS_DATA, 2, &buff);
			if(err < 0)	{
				i2c_log_in_test_case("Fail to read data Psensor\n");
				lnResult = I2C_TEST_Psensor_FAIL;
				goto error_1;
			}
			else
				cm36283_turn_onoff_proxm(0);
		}else	{
			printk(DBGMSK_PRX_G3"[cm36283][ps] Turn on Cm36283\n");
			err = i2c_stresstest_turn_on_proxm(1);
			if(err < 0)	{
				i2c_log_in_test_case("Fail to turn on Psensor\n");
				lnResult = I2C_TEST_Psensor_FAIL;
				goto error_1;
			}
			err = cm36283_read_reg(cm36283_client, CM36283_PS_DATA, 2, &buff);
			if(err < 0)	{
				i2c_log_in_test_case("Fail to read data Psensor\n");
				lnResult = I2C_TEST_Psensor_FAIL;
				goto error_1;
			}
			else
				cm36283_turn_onoff_proxm(0);		
		}
	}
	i2c_log_in_test_case("TestLSensorI2C --\n");

error_1:
	return lnResult;
}

static struct i2c_test_case_info gLSensorTestCaseInfo[] =
{
     __I2C_STRESS_TEST_CASE_ATTR(TestCm36283SensorI2C),
};
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cm36283_early_suspend(struct early_suspend *handler)
{
	return;
}


static void cm36283_late_resume(struct early_suspend *handler)
{
	printk("[cm36283] ++cm36283_late_resume, g_cm36283_on_fail:%d\n", g_cm36283_on_fail);

	if(g_cm36283_on_fail==1) {
		if ( !g_bIsP01Attached )
			cm36283_turn_onoff_als(1);
		printk("[cm36283][als] late_resume: apply ALS interrupt mode\n");
	}

	printk(DBGMSK_PRX_G2"[cm36283]--cm36283_late_resume\n");
}


static struct early_suspend cm36283_early_suspend_desc = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = cm36283_early_suspend,
    .resume = cm36283_late_resume,
};
#endif


const struct i2c_device_id cm36283_id[] = {
    {"cm36283", 0},
    {}
};

MODULE_DEVICE_TABLE(i2c, cm36283_id);

static struct of_device_id cm36283_match_table[] = {
	{ .compatible = "capella,cm36283",},
	{ },
};

static struct i2c_driver cm36283_driver = {
    .driver = {
        .name = "cm36283",
        .owner  = THIS_MODULE,
        .of_match_table = cm36283_match_table,
     },
    .probe = cm36283_probe,
    .remove = cm36283_remove,
    .suspend = cm36283_suspend,
    .resume = cm36283_resume,
    .id_table = cm36283_id,
};

static int cm36283_gpio_init(void)
{
	int rc = -EINVAL;
	printk("[cm36283][board]cm36283_gpio_init++\n");

	/* configure Phone Lightsensor interrupt gpio */
	rc = gpio_request(CM36283_GPIO_PROXIMITY_INT, "cm36283-irq");
	if (rc) {
		pr_err("%s: unable to request gpio %d (cm36283-irq)\n",__func__, CM36283_GPIO_PROXIMITY_INT);
		goto err;
	}

	rc = gpio_direction_input(CM36283_GPIO_PROXIMITY_INT);
	if (rc < 0) {
		pr_err("%s: unable to set the direction of gpio %d\n",__func__, CM36283_GPIO_PROXIMITY_INT);
		goto err;
	}

	/* configure Phone Lightsensor power_enable gpio */
	rc = gpio_request(CM36283_GPIO_PROXIMITY_PWR_EN, "proxm_pwr_en");
	if (rc) {
		pr_err("%s: unable to request gpio %d (proxm_pwr_en)\n",__func__, CM36283_GPIO_PROXIMITY_PWR_EN);
		goto err;
	}

	rc = gpio_direction_output(CM36283_GPIO_PROXIMITY_PWR_EN, 1);
	if (rc < 0) {
		pr_err("%s: unable to set the direction of gpio %d\n",__func__, CM36283_GPIO_PROXIMITY_PWR_EN);
		goto err;
	}

	/* HW Power on cm36283 */
	gpio_set_value(CM36283_GPIO_PROXIMITY_PWR_EN, 1);

	printk("[cm36283][board]cm36283_gpio_init--\n");
	return 0;
err:
	gpio_free(CM36283_GPIO_PROXIMITY_PWR_EN);
	return rc;
}

static int init_cm36283(void)
{
	int ret = 0;
	printk(DBGMSK_PRX_G2"[cm36283]: init_cm36283 +.\n");

	g_proxm_switch_on = 0;
	g_HAL_als_switch_on = 0;
	g_cm36283_als_switch_on = 0;

	g_switch[0] = 0; //disable PS as default!
	g_switch[1] = 0; //disable ALS as default!
	ret = cm36283_reset(); // reset both sensor 

	Proximity_test_wq = create_workqueue("Proximity_test_work");
	cm36283_workqueue = create_singlethread_workqueue("cm36283_wq");
	cm36283_delay_workqueue = create_singlethread_workqueue("cm36283_delay_wq");

	INIT_WORK(&cm36283_ISR_work, cm36283_interrupt_handler);
	INIT_WORK(&cm36283_light_interrupt_work, light_interrupt_work);
	INIT_WORK(&cm36283_proximity_interrupt_work, proximity_interrupt_work);
	INIT_WORK(&cm36283_attached_Pad_work, cm36283_lightsensor_attached_pad);
	INIT_DELAYED_WORK( &Proximity_test_work, Proximity_test_delayed_work);
	INIT_DELAYED_WORK(&cm36283_light_interrupt_delay_work, light_interrupt_work);

	wake_lock_init(&proximity_wake_lock, WAKE_LOCK_SUSPEND, "proxm_wake_lock");

#ifndef INTERNAL_TEST // register irq handler after sensor power on in order to avoid unexpected irq error!
	g_cm36283_device.irq = gpio_to_irq( cm36283_irq_gpio );//(CM36283_GPIO_PROXIMITY_INT);
	printk("[cm36283]Reques EIRQ %d succesd on GPIO:%d\n",g_cm36283_device.irq,cm36283_irq_gpio);
	
	if( g_cm36283_device.irq < 0 )
		printk(DBGMSK_PRX_G0"[cm36283] gpio_to_irq fail (g_cm36283_device.irq)irq=%d.\n", g_cm36283_device.irq);
	else		{
		printk(DBGMSK_PRX_G2"[cm36283] (g_cm36283_device.irq) irq=%d.\n", g_cm36283_device.irq);

		ret = request_irq(  g_cm36283_device.irq,
                            als_proxm_interrupt_handler,
                            IRQF_TRIGGER_FALLING,
                            "cm36283_INT",
                            &cm36283_client->dev );
		if (ret < 0)
			printk(DBGMSK_PRX_G0"[cm36283] (g_cm36283_device.irq) request_irq() error %d.\n",ret);
		else
			printk(DBGMSK_PRX_G2"[cm36283] (g_cm36283_device.irq) request_irq ok.\n");
	}
#endif

	g_cm36283_data_as->polling = 0;
	g_cm36283_data_as->poll_interval_ms = 100;
	g_cm36283_data_as->event_threshold = 1000;

	g_cm36283_data_ps->polling = 0;
	g_cm36283_data_ps->poll_interval_ms = 100;
	g_cm36283_data_ps->event_threshold = 1000;

	printk(DBGMSK_PRX_G2"[cm36283] init_cm36283 -.\n");
	return 1;
}

static int cm36283_input_init(void)
{
	int ret = 0;
	struct input_dev *input_dev_as = NULL;
	struct input_dev *input_dev_ps = NULL;

	input_dev_as = input_allocate_device();
	input_dev_ps = input_allocate_device();
	if (!input_dev_as || !input_dev_ps) {
		ret = -ENOMEM;
		printk(DBGMSK_PRX_G0"[cm36283]: Failed to allocate input_data device\n");
		goto error_1;
	}

	input_dev_as->name = "ASUS Lightsensor";
	input_dev_as->id.bustype = BUS_I2C;
	input_set_capability(input_dev_as, EV_ABS, ABS_MISC);
	__set_bit(EV_ABS, input_dev_as->evbit);
	__set_bit(ABS_MISC, input_dev_as->absbit);
	input_set_abs_params(input_dev_as, ABS_MISC, 0, 1048576, 0, 0);
	input_set_drvdata(input_dev_as, g_cm36283_data_as);

	input_dev_ps->name = "ASUS Proximitysensor";
	input_dev_ps->id.bustype = BUS_I2C;
	input_set_capability(input_dev_ps, EV_ABS, ABS_DISTANCE);
	__set_bit(EV_ABS, input_dev_ps->evbit);
	__set_bit(ABS_DISTANCE, input_dev_ps->absbit);
	input_set_abs_params(input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);
	input_set_drvdata(input_dev_ps, g_cm36283_data_ps);

	ret = input_register_device(input_dev_as);
	if (ret < 0) {
		input_free_device(input_dev_as);
		goto error_1;
	}
	g_cm36283_data_as->input_dev = input_dev_as;

	ret = input_register_device(input_dev_ps);
	if (ret < 0) {
		input_free_device(input_dev_ps);
		goto error_1;
	}
	g_cm36283_data_ps->input_dev = input_dev_ps;

	this_input_dev_as = input_dev_as;
	this_input_dev_ps = input_dev_ps;
	
	ret = als_lux_report_event_register(g_cm36283_data_as->input_dev);

error_1:

	return ret;
}

static int cm36283_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int error = 0;

	printk(DBGMSK_PRX_G2"[cm36283] cm36283_probe +.\n");

	if (client == NULL) {
		printk(DBGMSK_PRX_G0"[cm36283] Client is NUll.\n");
		ret =  -EFAULT;
		goto cm36283_probe_err;
	}

	if (!(g_cm36283_data_as = kmalloc(sizeof(struct cm36283_data), GFP_KERNEL))) {
		ret = -ENOMEM;
		goto cm36283_probe_err;
	}
	memset(g_cm36283_data_as, 0, sizeof(struct cm36283_data));

	if (!(g_cm36283_data_ps = kmalloc(sizeof(struct cm36283_data), GFP_KERNEL))) {
		ret = -ENOMEM;
		goto cm36283_probe_err;
	}
	memset(g_cm36283_data_ps, 0, sizeof(struct cm36283_data));

	//store i2c client data structure
	cm36283_client = client;
	i2c_set_clientdata(cm36283_client, g_cm36283_data_as);
	i2c_set_clientdata(cm36283_client, g_cm36283_data_ps);
	mutex_init(&g_cm36283_data_as->lock);
	mutex_init(&g_cm36283_data_ps->lock);
	cm36283_client->driver = &cm36283_driver;
	cm36283_client->flags = 1;
	strlcpy(cm36283_client->name, CM36283_DRV_NAME, I2C_NAME_SIZE);

	/* Get data that is defined in board specific code. */
	cm36283_irq_gpio = of_get_named_gpio_flags(
			client->dev.of_node, "cm36283,irq-gpio",0,NULL);

	/* Init gpio */
	error = cm36283_gpio_init();
        if (error) {
            dev_err(&client->dev, "hw init failed");
            goto cm36283_probe_err;
        }

	printk(DBGMSK_PRX_G2"[cm36283] Register input device...\n");
	if( cm36283_input_init() != 0 ) {
		goto cm36283_probe_err;
	}

	ret = init_cm36283();
	if( ret <= 0 )  {
		printk(DBGMSK_PRX_G0"[cm36283] init_cm36283() error.\n");
		goto cm36283_probe_err;
	}

	ret = proximity_dev_register(&g_proxmDev);
	if (ret)
		printk(DBGMSK_PRX_G0"[cm36283] proxmdl create sysfile fail.\n");

	ret = proximity_dev_register(&g_ambientDev);
	if (ret)
		printk(DBGMSK_PRX_G2"[cm36283] ambientdl create sysfile fail.\n");
	
#ifdef CONFIG_I2C_STRESS_TEST
	printk("LSenor add test case+\n");
	i2c_add_test_case(client, "LightSensorTest",ARRAY_AND_SIZE(gLSensorTestCaseInfo));
	printk("LSensor add test case-\n");
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend( &cm36283_early_suspend_desc );
#endif

	/*For calibration issue*/
	printk(DBGMSK_PRX_G2"[cm36283]++cm36283_probe: create_group\n");
	/* register sysfs hooks */

	ret = sysfs_create_group(&client->dev.kobj, &cm36283_attr_group);
	if (ret)
		goto cm36283_probe_err;

	printk(DBGMSK_PRX_G2"[cm36283] cm36283_probe -.\n");
	return 0;

cm36283_probe_err:
	printk(DBGMSK_PRX_G0"[cm36283] cm36283_probe - (error).\n");
	//if (g_cm36283_data_ps != NULL) {
	//  kfree(g_cm36283_data_ps);
	//}
	return ret;
}


static int cm36283_remove(struct i2c_client *client)
{
    printk(DBGMSK_PRX_G2"[cm36283] cm36283_remove +.\n");

    /* free_irq(client->irq, NULL); */

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend( &cm36283_early_suspend_desc );
#endif

    printk(DBGMSK_PRX_G2"[cm36283] cm36283_remove -.\n");
    return 0;
}


static int cm36283_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int err = 1;
	struct i2c_msg msg[1];
	unsigned char data[2] = {0,0};

	printk("[cm36283] ++cm36283_suspend, psensor:%d, als:%d\n", g_proxm_switch_on, g_cm36283_als_switch_on);

	enable_irq_wake(g_cm36283_device.irq);

	if(g_cm36283_als_switch_on==1) {
		//In case upper layer doesn't switch off ambient before early_suspend.
		g_cm36283_als_switch_on = 0;
		g_ambient_suspended = 1;

		printk(DBGMSK_PRX_G2"[cm36283] cm36283_suspend, turn off ambient\n");

		//0x00_L
		msg->addr = cm36283_client->addr;
		msg->flags = 0;
		msg->len = 2;
		msg->buf = data;
		data[0] = CM36283_ALS_CONF;
		data[1] = (INIT_ALS | 0x1);

		err = i2c_transfer(cm36283_client->adapter, msg, ARRAY_SIZE(msg));

		if(err != 1)
			printk(DBGMSK_PRX_G0"[cm36283] (%s): reg=0x%x data=0x%x err=%d\n",__FUNCTION__ ,data[0], data[1], err);
		else
			printk(DBGMSK_PRX_G2"[cm36283] als_config=0x%x, data=0x%x\n",data[0], data[1]);
	}
	printk("[cm36283] --cm36283_suspend\n");
	return 0 ;
}


static int cm36283_resume(struct i2c_client *client)
{
	printk(DBGMSK_PRX_G2"[cm36283]++cm36283_resume, gpio 38 : %d\n",gpio_get_value(CM36283_GPIO_PROXIMITY_INT) );

	if( !g_bIsP01Attached /*&& g_proxm_switch_on && !gpio_get_value(CM36283_GPIO_PROXIMITY_INT)*/) {
		g_cm36283_earlysuspend_int = 0;
		queue_work(cm36283_workqueue, &cm36283_ISR_work);
		if (g_proxim_state == 1)
			wake_lock_timeout(&proximity_wake_lock, 1 * HZ);
	}

	printk("[cm36283][als] resume: g_ambient_suspended = %d first_light=%d\n",g_ambient_suspended, g_cm36283_light_first);
	if(g_ambient_suspended==1) {
		if ( !g_bIsP01Attached ) {
			g_ambient_suspended = 0;
			cm36283_turn_onoff_als(1);
		}

		g_ambient_suspended = 0;
		g_cm36283_als_switch_on = 1; //this flag is usually changed in put_property.
		printk("[cm36283][als] resume: apply ALS interrupt mode\n");
	}

	disable_irq_wake(g_cm36283_device.irq);

	g_cm36283_earlysuspend_int = 0;

	printk(DBGMSK_PRX_G2"[cm36283]--cm36283_resume\n");
	return 0 ;
}

/////////////////////////////////////////////////////////////////////////////////
// Pad mode reltaed
//
static void cm36283_lightsensor_attached_pad(struct work_struct *work)
{
	printk(DBGMSK_PRX_G2"[cm36283_als] lightsensor_attached_pad()++\n");

	//if HAL already turned on als, we switch to P01 al3010
	if(g_proxm_switch_on)
		cm36283_turn_onoff_proxm(0);

	if(g_HAL_als_switch_on) 
	{
		printk(DBGMSK_PRX_G2"[cm36283_als] lightsensor_attached_pad, checking if P01 is attached\n");

		//if(g_bIsP01Attached) {
			printk(DBGMSK_PRX_G2"[cm36283_als] lightsensor_attached_pad, attached! switch to al3010 : %d lux\n", g_cm36283_light);
			
			/*shut down cm36283_als*/
			cm36283_turn_onoff_als(0);
		//}else
			//printk("[cm36283_als] al3010_attached_P02 fail\n");
	}
	printk(DBGMSK_PRX_G2"[cm36283_als] lightsensor_attached_pad()--\n");

	return;
}

int cm36283_lightsensor_detached_pad(void)
{
	printk(DBGMSK_PRX_G2"[cm36283_als] Cm36283_detached_pad()++\n");

	//if HAL still turned on the als, we switch back to cm36283
	if(g_HAL_als_switch_on)	{
		printk(DBGMSK_PRX_G2"[cm36283_als] lightsensor_detached_pad(), switch back to cm36283 : %d lux\n", g_cm36283_light);
		als_lux_report_event( g_cm36283_light);
		
		//turn on cm36283_als
		cm36283_turn_onoff_als(1);
	}
	else
		printk(DBGMSK_PRX_G2"[cm36283_als] lightsensor_detached_pad(), als is turned off\n");

	//if phone call or ps still on, we switch back to cm36283
	if (g_proxm_switch_on)	{
		printk(DBGMSK_PRX_G2"[cm36283_proxm] ps_detached_pad(), phone call still on\n");
		cm36283_turn_onoff_proxm(1);
		queue_work(cm36283_workqueue, &cm36283_proximity_interrupt_work);
	}
	else if ( pad_proxm_switch_on && !g_proxm_switch_on)	{
		printk("[cm36283_proxm] ps_detached_pad(), phone call on in Pad\n");
		cm36283_turn_onoff_proxm(1);
		g_proxm_switch_on = 1;
		queue_work(cm36283_workqueue, &cm36283_proximity_interrupt_work);
	}
	else
		printk(DBGMSK_PRX_G2"[cm36283_als] ps is turned off\n");\
		
	printk(DBGMSK_PRX_G2"[cm36283_als] Cm36283_detached_pad()--\n");
	return 0;
}


static int cm36283_lightsensor_mp_event(struct notifier_block *this, unsigned long event, void *ptr)
{

    switch (event) {
        case P01_ADD:
            printk("[cm36283_als][MicroP] Pad_ADD \r\n");
            //Work to cm36283_lightsensor_attached_pad();
            queue_work(cm36283_workqueue, &cm36283_attached_Pad_work);
            return NOTIFY_DONE;

        case P01_REMOVE:
            printk("[cm36283_als][MicroP] Pad_REMOVE \r\n");
            cm36283_lightsensor_detached_pad();
            return NOTIFY_DONE;

        default:
            return NOTIFY_DONE;
    }
}

static struct notifier_block cm36283_lightsensor_mp_notifier = {
        .notifier_call = cm36283_lightsensor_mp_event,
        .priority = CM36283_LIGHTSENSOR_MP_NOTIFY,
};

static int cm36283_platform_probe( struct platform_device *pdev )
{
	int err = 0;
	printk("[cm36283] cm36283_platform_probe ++ \n");
	err = i2c_add_driver(&cm36283_driver);
	if ( err != 0 )
		printk("[cm36283] i2c_add_driver fail, Error : %d\n",err);
	printk("[cm36283] cm36283_platform_probe -- \n");
	return 0;
}

static int __devexit cm36283_platform_remove( struct platform_device *pdev )
{
	i2c_del_driver(&cm36283_driver);
	proximity_dev_unregister(&g_proxmDev);
	proximity_dev_unregister(&g_ambientDev);
	destroy_workqueue(cm36283_workqueue);

	return 0;
}

static int cm36283_platform_suspend_noirq( struct device *dev )
{
	printk("[cm36283][suspend_noirq] g_earlysuspend_int = %d\n",  g_cm36283_earlysuspend_int);
	if(g_cm36283_earlysuspend_int == 1) {
		g_cm36283_earlysuspend_int = 0;
		return -EBUSY;
	}

        return 0;
}

static int cm36283_platform_resume_noirq( struct device *dev )
{
        return 0;
}

static struct of_device_id cm36283_of_match[] = {
	{ .compatible = "cm36283", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);

static const struct dev_pm_ops cm36283_pm_ops = {
        .suspend_noirq  = cm36283_platform_suspend_noirq,
	.resume_noirq = cm36283_platform_resume_noirq,
};

static struct platform_driver  cm36283_platform_driver = {
	.probe 	= cm36283_platform_probe,
     	.remove   = cm36283_platform_remove,

	.driver = {
		.name = "cm36283",
		.owner = THIS_MODULE,
		.pm = &cm36283_pm_ops,		
		.of_match_table = cm36283_of_match,
	},
};

static int __init cm36283_init(void)
{
	int err = 0;
	if ( g_ASUS_hwID == A90_EVB0 )	{
		printk("[cm36283] cm36283_platform_init +.\n");

		err = platform_driver_register(&cm36283_platform_driver);
		if ( err != 0 )
			printk("[cm36283] platform_driver_register fail, Error : %d\n",err);

		register_microp_notifier(&cm36283_lightsensor_mp_notifier);
		notify_register_microp_notifier(&cm36283_lightsensor_mp_notifier, "cm36283"); //ASUS_BSP Lenter+

		printk("[cm36283] cm36283_platform_init -.\n");
	}
	return err;
}


static void __exit cm36283_exit(void)
{
	printk(DBGMSK_PRX_G2"[cm36283] cm36283_platform_exit +.\n");
	platform_driver_unregister(&cm36283_platform_driver);
	printk(DBGMSK_PRX_G2"[cm36283] cm36283_platform_exit -.\n");
}

module_init(cm36283_init);
module_exit(cm36283_exit);

MODULE_AUTHOR("ASUS");
MODULE_DESCRIPTION("CAPELLA CM36283 proximity sensor with ALS");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
