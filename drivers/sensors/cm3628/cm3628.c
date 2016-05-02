/* cm3628.c - CM3628 proximity/light sensor driver
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
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>
#endif

#define CM3628_PS_CONF_NAME			"cm3628_ps_conf"
#define CM3628_ALS_CONF_NAME			"cm3628_als_conf"
#define CM3628_ARA_NAME				"cm3628_ara"

#define CM3628_PS_MODE					0x00
#define CM3628_ALS_MODE				0x01
#define CM3628_ARA_MODE				0x02

/* For sensor reset & HW Reset issue */
#define CM3628_REG_INIT					0x00
static int g_cm3628_reset = 0;

#define CM3628_GPIO_PROXIMITY_INT			74
//#define CM3628_GPIO_PROXIMITY_PWR_EN		3

// ASUS_BSP +++ Peter_Lu "Set L18 sensor power source from standby mode to normal mode for keep sensor power source stable"
#include <linux/regulator/consumer.h>
static struct regulator *ldoa18_regulator;
// ASUS_BSP --Peter_Lu

struct i2c_client *cm3628_ps_conf_client = NULL;
struct i2c_client *cm3628_als_conf_client = NULL;
struct i2c_client *cm3628_ara_client = NULL;

static struct workqueue_struct *cm3628_workqueue;
static struct workqueue_struct *cm3628_delay_workqueue;

#ifdef CONFIG_EEPROM_NUVOTON
static struct work_struct cm3628_attached_Pad_work;
static void cm3628_lightsensor_attached_pad(struct work_struct *work);
#endif

static struct delayed_work cm3628_ISR_work;
static struct delayed_work cm3628_proximity_interrupt_delay_work;
static struct delayed_work cm3628_light_interrupt_delay_work;

struct ASUS_light_sensor_data {
	struct i2c_client				client ;
	struct input_dev				*input_dev;     /* Pointer to input device */

	int g_als_threshold_lo;				// Lightsensor setting low threshold(adc)
	int g_als_threshold_hi;				// Lightsensor setting high threshold(adc)
	int light_adc;							// Original light adc value
	int light_k_adc;						// The light adc value after calibration
	int last_light_lux;					// light Lux after filter
	int light_lux;							// Final light Lux
	
	bool HAL_als_switch_on;				//this var. means if HAL is turning on als or not
	bool Device_als_switch_on;			//this var. means if cm32180 als hw is turn on or not
	
	//struct delayed_work			i2c_poll_work;
	unsigned int					poll_interval_ms;   /* I2C polling period */
	unsigned int					event_threshold;    /* Change reqd to gen event */
	unsigned int					open_count;     /* Reference count */
	char							polling;            /* Polling flag */
};

struct ASUS_proximity_sensor_data {
	struct i2c_client				client ;
	struct input_dev				*input_dev;     /* Pointer to input device */

	int g_ps_threshold_lo;					// Proximitysensor setting low threshold(adc)
	int g_ps_threshold_hi;					// Proximitysensor setting high threshold(adc)
	bool proxm_detect_object_state;			//For check proximity near/far state

	bool HAL_ps_switch_on;					//this var. means if HAL is turning on ps or not
	bool Device_ps_switch_on;				//this var. means is turning on ps or not
	bool pad_proxm_switch_on;				//For phone call on in pad issue
	
	//struct delayed_work			i2c_poll_work;
	unsigned int					poll_interval_ms;   /* I2C polling period */
	unsigned int					event_threshold;    /* Change reqd to gen event */
	unsigned int					open_count;     /* Reference count */
	char							polling;            /* Polling flag */
};

struct _cm3628_device {
	int irq;
} g_cm3628_device;

struct ASUS_light_sensor_data  *g_cm3628_data_as;
struct ASUS_proximity_sensor_data  *g_cm3628_data_ps;
static int cm3628_irq_gpio = 0;

extern bool g_bIsP01Attached;				/* For check Pad mode state */
static int g_cm3628_earlysuspend_int = 0;	/* For suspend no_irq issue */

static int g_proxm_dbg = 0; /* Add for debug only */
static int g_ambient_dbg = 0; /* Add for debug only */
static int g_interval = 100;

static int cm3628_proxmdev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int cm3628_proxmdev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int cm3628_turn_onoff_proxm(bool bOn);
static int cm3628_ps_conf_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int cm3628_als_conf_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int cm3628_ara_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int get_ps_adc_from_cm3628(bool trigger_by_interrupt);
static int cm3628_ara_read_reg( struct i2c_client *client );
 static int cm3628_als_read_reg( struct i2c_client *client );
static int cm3628_ambientDev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int cm3628_ambientDev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int cm3628_turn_onoff_als(bool bOn);
static int get_als_adc_from_cm3628(bool trigger_by_interrupt);
static int cm3628_reg_reset(unsigned int mode);
static int get_calibrated_lux_from_cm3628_adc(int adc);

static bool read_lightsensor_shiftvalue(void);
static bool read_lightsensor_calibrationvalue(void);
static bool read_prox_hi_calibrationvalue(void);
static bool read_prox_lo_calibrationvalue(void);


/////////////////////////////////////////////////////////////////////////////////////////
//---proximity sensor setting part---
//
//static int g_ps_threshold_lo = DEFAULT_PS_THRESHOLD_lo;
//static int g_ps_threshold_hi = DEFAULT_PS_THRESHOLD_hi;
//static bool g_cm3628_ps_switch_on = 0;	/*For check proximity function on/off state*/
//static bool g_proxm_state = 0;			/*For check proximity near/far state*/
//static int g_proxm_switch_on = 0;			/*For check proxinity function on/off state*/
//static bool g_cm3628_proxim_state = 0;	/*For check proxinity device power on/off state*/
//static bool pad_proxm_switch_on = 0;		/*For phone call on in pad issue*/
//static u16 g_psData = 0; 					//proximity data

//wake_lock for Proximity, make sure Proximity finish all of the work before suspend
#include <linux/wait.h>
#include <linux/wakelock.h>
static struct wake_lock proximity_wake_lock;

/* CM3628 Proximity Register */
#define CM3628_PS_CONF1			0x00    //Setting cmd (Duty rate, integration time, interrupt and ps enable/disable, persistence) for proximity sensor.
#define CM3628_PS_THD				0x01    //write control cmd for proximity sensor.
#define CM3628_PS_CANC				0x02    //write control cmd for proximity sensor.
#define CM3628_PS_CONF2			0x03    //write control cmd for proximity sensor.

/* CM3628 Proximity command CONF1 */
static u8 PS_DR			 = 0x02;  //PS LED on/off duty ratio setting		, bit 7,6 ( 1/320 )
static u8 PS_IT			 = 0x02;	// PS interrupt intrgration time setting	, bit 5,4 ( 2T )
static u8 PS_PERS			 = 0x02; // PS interrupt persistence setting		, bit 3,2 ( 3 times )
static u8 PS_INT_EN		 = 0x00; // PS interrupt enable/disable			, bit 1   ( disable )
static u8 PS_INT_EN_ON	 = 0x01; // PS interrupt enable					, bit 1   ( enable )
static u8 PS_SD_OFF		 = 0x01; // PS shut down setting				, bit 0   ( disable )
static u8 PS_SD_ON		 = 0x00; // PS shut down setting				, bit 0   ( enable )

/* CM3628 Proximity command CONF2 */
static u8 PS_MS		 	= 0x00;	// PS interrupt or logic output mode			, bit 4       ( interrupt )
static u8 PS_DIR_INT		= 0x00;	// PS directional interrupt function 			, bit 2	( disable )
static u8 PS_SMART_PERS	= 0x01;	// PS Smart persistence function				, bit 1	( enable )
static u8 PS_HYS			= 0x01;	// PS detect threshold hysteresis window		, bit 0	( 2 steps )

/////////////////////////////////////////////////////////////////////////////////////////
//---light sensor setting part---
//
extern int g_HAL_als_switch_on;			// For all lightsensor trun on/off global flag
//static bool g_cm3628_als_switch_on = false;	/*For check Light sensor device power on/off state*/
static int g_ambient_suspended = 0;		/*For check Light sensor device suspend/resume state*/

static int g_max_light_level = 17;
static int g_cm3628_light_map[17] = {0,50,100,200,300,400,500,650,800,1000,1500,2000,3000,4000,5000,7000,10000};

/* CM3628 Als Register */
#define CM3628_ALS_CONF1			0x00    //Setting cmd (integration time, interrupt setting and enable/disable, persistence) for als sensor.
#define CM3628_ALS_CONF2			0x01    //Average mode and high sensitivity setting
#define CM3628_ALS_WH_M			0x02    //High interrupt threshold window setting
#define CM3628_ALS_WH_L			0x03    //High interrupt threshold window setting 
#define CM3628_ALS_WL_M			0x04    //Low interrupt threshold window setting
#define CM3628_ALS_WL_L			0x05    //Low interrupt threshold window setting

/* CM3628 Als command */
/* For_CONF1 */
static u8 ALS_IT			 = 0x00;	// ALs intrgration time setting					, bit 7,6 ( 50ms )
static u8 ALS_PERS		 = 0x02; // ALS interrupt persistence setting				, bit 5,4 ( 4 times )
static u8 ALS_THRES		 = 0x01; // ALS interrupt trigger Threshold machanism setting	, bit 2   ( 0:variance mode, 1:threshold mode )
static u8 ALS_INT_EN		 = 0x00; // ALS interrupt enable/disable					, bit 1   ( disable )
static u8 ALS_SD			 = 0x01; // ALS shut down setting						, bit 0   ( disable )
static u8 ALS_SD_NO		 = 0x00; // ALS shut down setting						, bit 0   ( enable )

static u8 ALS_INT_EN_OFF	 = 0x00; // ALS interrupt enable/disable					, bit 1   ( disable )
static u8 ALS_INT_EN_ON	 = 0x01; // ALS interrupt enable/disable					, bit 1   ( enable )
static u8 ALS_SD_OFF		 = 0x01; // ALS shut down setting						, bit 0   ( disable )
static u8 ALS_SD_ON		 = 0x00; // ALS shut down setting						, bit 0   ( enable )

/* For_CONF2 */
static u8 ALS_RESET		 = 0x01; // Reset ALS reading sequence					, bit 6       ( enable )
static u8 ALS_AV			 = 0x02; // ALS average number setting					, bit 5,4    ( 4 times )
static u8 ALS_HS			 = 0x01; // ALS normal/high sensitivity mode selection		, bit 3       ( normal )
static u8 ALS_THD			 = 0x02; // ALS variance mode thershold range setting		, bit 0,1,2  ( +/- 32 steps )

/////////////////////////////////////////////////////////////////////////////////////////
//---ATD test part---
//
static int proximity_als_turn_on(int bOn)
{ 
	int err = 0;
	int status = 0; // 0x1: light sensor is ok, 0x2: proximity is ok, 0x3: both are ok, 0x0: both failed

	printk(DBGMSK_PRX_G5"[cm3628] proximity and light sensor test, turn_on:%d\n", bOn);

	err = cm3628_turn_onoff_als(bOn);
	if(err < 0)
		printk(DBGMSK_PRX_G0"[cm3628] (%s): turn %s light sensor error!\n" ,__FUNCTION__, bOn ? "on" : "off");
	else	{
		printk(DBGMSK_PRX_G5"[cm3628] (%s): light sensor %s OK!\n" ,__FUNCTION__, bOn ? "on" : "off");
		status |= 0x1;  //als ok
	}

	err = cm3628_turn_onoff_proxm(bOn);
	if(err < 0)
		printk(DBGMSK_PRX_G0"[cm3628] (%s): turn %s proximity sensor error!\n",__FUNCTION__, bOn ? "on" : "off");
	else {
		printk(DBGMSK_PRX_G5"[cm3628] (%s): proximity sensor %s OK!\n",__FUNCTION__, bOn ? "on" : "off");
		status |= 0x02; //ps OK
	}

	printk(DBGMSK_PRX_G5"[cm3628]turn %s, status:0x%x (bitwise)\n", bOn ? "on" : "off", status);

	return status;
}

static int atd_read_P_L_sensor_adc(int *adc)
{
	int status = 0;
	//int lux = 0;
	int idx = 0;

	printk(DBGMSK_PRX_G5"[cm3628][atd]readadc: trying to turn on lsensor\n");
	status = proximity_als_turn_on(1);

	if(0 == status) {
		printk(DBGMSK_PRX_G2"[cm3628][atd]readadc: lsensor is not on\n");
		return status;
	}

	*adc = 0;

	for(idx = 0; idx < 5; idx++)
	{
		*adc = get_als_adc_from_cm3628( false );
		msleep(100);
	}

	//lux = (u32)((*adc) * g_cm3628_light_gain_calibration / 100  + g_cm3628_light_shift_calibration); 

	printk(DBGMSK_PRX_G5"[cm3628][atd]readadc steps=%d \n", *adc);//lux=%d\n", *adc, lux);
	proximity_als_turn_on(0);

	return status;
}

static int atd_write_status_and_adc_2(int *als_sts, int *als_adc, int *ps_sts)
{
	int adc = 0;
	int status = 0; // 0x1: light sensor is ok, 0x2: proximity is ok, 0x3: both are ok, 0x0: both failed

	printk(DBGMSK_PRX_G5"[cm3628][atd]writests_2: started\n");
	status = atd_read_P_L_sensor_adc(&adc);

	if(status & 0x01)	{
		*als_sts = 1;
		*als_adc = adc;
	}
	else	{
		*als_sts = 0;
		*als_adc = 0;
	}

	if(status & 0x02)	{
		*ps_sts = 1;
	}
	else	{
		*ps_sts = 0;
	}

	printk(DBGMSK_PRX_G5"[cm3628][atd]writests_2: get adc, als_sts:%d, als_adc:%d, ps_sts:%d\n", 
		*als_sts, *als_adc, *ps_sts);

	return status;
}

static u32 g_cm3628_light_shift_calibration = 40;
static int g_cm3628_light_gain_calibration = 38;
static int a_als_calibration_accuracy = 100000;

#ifdef ASUS_FACTORY_BUILD
static struct write_calvalue {
    struct work_struct write_calvalue_work;
    int calvalue;
} *cm3628_write_calvalue;

static struct write_shift {
    struct work_struct write_shift_work;
    int calvalue;
} *cm3628_write_shift;

static struct write_prox_hi {
    struct work_struct write_prox_hi_work;
    int calvalue;
} *cm3628_write_prox_hi;

static struct write_prox_lo {
    struct work_struct write_prox_lo_work;
    int calvalue;
} *cm3628_write_prox_lo;

static int a_als_calibration_lux = 800;
static int a_als_low_calibration_adc = 0;
static int a_als_high_calibration_adc = 0;
static int a_ps_hi_calibration_adc = 0;
static int a_ps_lo_calibration_adc = 0;
#endif

/* Support ATD light sensor calibration process */
static bool read_lightsensor_calibrationvalue()
{
	struct file *fp = NULL; 
	loff_t pos_lsts = 0;
	char readstr[16];
	int ori_val = 0, readlen =0;
	mm_segment_t old_fs;
	
	printk(DBGMSK_PRX_G5"[cm3628] ++read_lsensor_calvalue open\n");

	fp = filp_open(LSENSOR_CALIBRATION_ASUS_NV_FILE, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G5"[cm3628] read_lsensor_calvalue open (%s) fail\n", LSENSOR_CALIBRATION_ASUS_NV_FILE);
		return false;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	
	if(fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, readstr, 16, &pos_lsts);
		readstr[readlen] = '\0';

		printk(DBGMSK_PRX_G5"[cm3628] strlen:%s(%d)\n", readstr, strlen(readstr));
	}
	else
		printk(DBGMSK_PRX_G0"[cm3628] read_lsensor_calvalue, f_op=NULL or op->read=NULL\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(readstr, "%d", &ori_val);

	//limit the calibration value range
	g_cm3628_light_gain_calibration = ori_val;

	printk("[cm3628] read_lsensor_calvalue: Ori: %d, Cal: %d.%d\n", ori_val, 
				g_cm3628_light_gain_calibration/a_als_calibration_accuracy,
				g_cm3628_light_gain_calibration%a_als_calibration_accuracy);

	printk(DBGMSK_PRX_G5"[cm3628] --read_lsensor_calvalue open\n");
	return true;
}

static bool read_lightsensor_shiftvalue()
{
    struct file *fp = NULL; 
    loff_t pos_lsts = 0;
    char readstr[16];
    int ori_val = 0, readlen =0;
    mm_segment_t old_fs;

    printk(DBGMSK_PRX_G5"[cm3628] ++read_lsensor_shift open\n");

    fp = filp_open(LSENSOR_CALIBRATION_SHIFT_ASUS_NV_FILE, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
    if(IS_ERR_OR_NULL(fp)) {
        printk(DBGMSK_PRX_G5"[cm3628] read_lsensor_shift open (%s) fail\n", LSENSOR_CALIBRATION_SHIFT_ASUS_NV_FILE);
        return false;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    if(fp->f_op != NULL && fp->f_op->read != NULL) {
        pos_lsts = 0;
        readlen = fp->f_op->read(fp, readstr, 16, &pos_lsts);
        readstr[readlen] = '\0';

        printk(DBGMSK_PRX_G5"[cm3628] strlen:%s(%d)\n", readstr, strlen(readstr));
    }
    else {
        printk(DBGMSK_PRX_G0"[cm3628] read_lsensor_shift, f_op=NULL or op->read=NULL\n");
    }

    set_fs(old_fs);
    filp_close(fp, NULL);

    sscanf(readstr, "%d", &ori_val);

    //limit the calibration value range
    g_cm3628_light_shift_calibration = ori_val;

    printk("[cm3628] read_lsensor_shift: Ori: %d, Cal: %d\n", ori_val, g_cm3628_light_shift_calibration);

    printk(DBGMSK_PRX_G5"[cm3628] --read_lsensor_calvalue open\n");
    return true;
}

#ifdef ASUS_FACTORY_BUILD
static void write_lightsensor_calibrationvalue_work(struct work_struct *work);

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
		printk(DBGMSK_PRX_G0"[cm3628] write_lsensor_shift open (%s) fail\n", LSENSOR_CALIBRATION_SHIFT_ASUS_NV_FILE);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk(DBGMSK_PRX_G5"[cm3628] write_lsensor_shift = %d[%s(%d)]\n", 
		this->calvalue, writestr, strlen(writestr));

	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
		printk(DBGMSK_PRX_G0"[cm3628] write_lsensor_shift fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}

/*Light Sensor Calibration */
static int cm3628_show_calibration_200(struct device *dev, struct device_attribute *attr, char *buf)
{
	read_lightsensor_calibrationvalue();
	
	printk(DBGMSK_PRX_G5"[cm3628] Show_gait_calibration: %d.%d \n", 
					g_cm3628_light_gain_calibration/a_als_calibration_accuracy,
					g_cm3628_light_gain_calibration%a_als_calibration_accuracy );

	return sprintf(buf, "%d.%d\n"
				, g_cm3628_light_gain_calibration/a_als_calibration_accuracy
				, g_cm3628_light_gain_calibration%a_als_calibration_accuracy);
}

static ssize_t cm3628_store_calibration_200(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	
	if ( (kstrtoul(buf, 10, &val) < 0) )
		return -EINVAL;
	
	/*Get low brightness adc*/
	a_als_low_calibration_adc = (int)val;

	printk("[cm3628] Get low calibration adc value : %d\n", a_als_low_calibration_adc );

	return count;
	
}

static DEVICE_ATTR(calibration_200, S_IRWXU | S_IRWXG| S_IRWXO,
		   cm3628_show_calibration_200, cm3628_store_calibration_200);


static int cm3628_show_calibration_1000(struct device *dev, struct device_attribute *attr, char *buf)
{	
	read_lightsensor_shiftvalue();

	printk(DBGMSK_PRX_G5"[cm3628] Show_shift_calibration: %d\n", g_cm3628_light_shift_calibration );
	
	return sprintf(buf, "%d\n", g_cm3628_light_shift_calibration );
}

static ssize_t cm3628_store_calibration_1000(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	
	if ( (kstrtoul(buf, 10, &val) < 0) )
		return -EINVAL;
	
	/*Get low brightness adc*/
	a_als_high_calibration_adc = (int)val;

	printk("[cm3628] Get High calibration adc value : %d\n", a_als_high_calibration_adc );

	/*Calibration operation*/
	g_cm3628_light_gain_calibration = 
		(a_als_calibration_lux*a_als_calibration_accuracy) /
					( a_als_high_calibration_adc - a_als_low_calibration_adc );

	g_cm3628_light_shift_calibration = 
		1000 - ( a_als_high_calibration_adc*g_cm3628_light_gain_calibration/a_als_calibration_accuracy );

	/*Write Calibration value*/
	cm3628_write_calvalue = kmalloc(sizeof(struct write_calvalue), GFP_KERNEL);

	INIT_WORK(&cm3628_write_calvalue->write_calvalue_work, write_lightsensor_calibrationvalue_work);

	cm3628_write_calvalue ->calvalue = g_cm3628_light_gain_calibration;

	queue_work(cm3628_workqueue, &cm3628_write_calvalue->write_calvalue_work);
	
	/*Write shift value*/
	cm3628_write_shift = kmalloc(sizeof(struct write_shift), GFP_KERNEL);

	INIT_WORK(&cm3628_write_shift->write_shift_work, write_lightsensor_shiftvalue_work);

	cm3628_write_shift ->calvalue = g_cm3628_light_shift_calibration;

	queue_work(cm3628_workqueue, &cm3628_write_shift->write_shift_work);

	return count;
}

static DEVICE_ATTR(calibration_1000, S_IRWXU | S_IRWXG | S_IRWXO,
		   cm3628_show_calibration_1000, cm3628_store_calibration_1000);

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
		printk(DBGMSK_PRX_G0"[cm3628] write_lsensor_calvalue open (%s) fail\n", LSENSOR_CALIBRATION_ASUS_NV_FILE);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk(DBGMSK_PRX_G5"[cm3628] write_lsensor_calvalue = %d[%s(%d)]\n", 
		this->calvalue, writestr, strlen(writestr));

	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
		printk(DBGMSK_PRX_G0"[cm3628] write_lsensor_calvalue fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}
#endif

static bool read_prox_hi_calibrationvalue()
{
	struct file *fp = NULL; 
	loff_t pos_lsts = 0;
	char readstr[8];
	int ori_val = 0, readlen =0;
	mm_segment_t old_fs;
	
	printk(DBGMSK_PRX_G3"[cm3628] ++read_psensor_hi_calvalue open\n");

	fp = filp_open(PSENSOR_CALIBRATION_HI_ASUS_NV_FILE, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G3"[cm3628] read_psensor_hi_calvalue open (%s) fail\n", PSENSOR_CALIBRATION_HI_ASUS_NV_FILE);
		return false;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	
	if(fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, readstr, 6, &pos_lsts);
		readstr[readlen] = '\0';

		printk(DBGMSK_PRX_G3"[cm3628] strlen:%s(%d)\n", readstr, strlen(readstr));
	}
	else
		printk(DBGMSK_PRX_G0"[cm3628] read_psensor_hi_calvalue, f_op=NULL or op->read=NULL\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(readstr, "%d", &ori_val);

	//limit the calibration value range
	g_cm3628_data_ps->g_ps_threshold_hi = ori_val;

	printk("[cm3628] read_psensor_hi_calvalues: Ori: %d, Cal: %d\n", ori_val, g_cm3628_data_ps->g_ps_threshold_hi);
	printk("[cm3628] --read_psensor_hi_calvalue open\n");
	return true;
}

static bool read_prox_lo_calibrationvalue()
{
	struct file *fp = NULL; 
	loff_t pos_lsts = 0;
	char readstr[8];
	int ori_val = 0, readlen =0;
	mm_segment_t old_fs;

	printk(DBGMSK_PRX_G3"[cm3628] ++read_psensor_low_calvalue open\n");

	fp = filp_open(PSENSOR_CALIBRATION_LO_ASUS_NV_FILE, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	//fp = filp_open(pFile, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G3"[cm3628] read_psensor_low_calvalue open (%s) fail\n", PSENSOR_CALIBRATION_LO_ASUS_NV_FILE);
		return false;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if(fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, readstr, 6, &pos_lsts);
		readstr[readlen] = '\0';

		printk(DBGMSK_PRX_G3"[cm3628] strlen:%s(%d)\n", readstr, strlen(readstr));
	}
	else
		printk(DBGMSK_PRX_G0"[cm3628] read_psensor_low_calvalue, f_op=NULL or op->read=NULL\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(readstr, "%d", &ori_val);

	//limit the calibration value range
	g_cm3628_data_ps->g_ps_threshold_lo = ori_val;

	printk("[cm3628] read_psensor_low_calvalue: Ori: %d, Cal: %d\n", ori_val, g_cm3628_data_ps->g_ps_threshold_lo);

	printk("[cm3628] --read_psensor_low_calvalue open\n");
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
		printk(DBGMSK_PRX_G0"[cm3628] write_psensor_hi_calvalue open (%s) fail\n", PSENSOR_CALIBRATION_HI_ASUS_NV_FILE);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk(DBGMSK_PRX_G3"[cm3628] write_psensor_hi_calvalue = %d[%s(%d)]\n", 
		this->calvalue, writestr, strlen(writestr));

	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
		printk(DBGMSK_PRX_G0"[cm3628] write_psensor_hi_calvalue fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}

static int cm3628_show_calibration_prox_hi(struct device *dev, struct device_attribute *attr, char *buf)
{
	read_prox_hi_calibrationvalue();
	printk(DBGMSK_PRX_G2"[cm3628] Show_prox_hi_calibration: %d\n", g_cm3628_data_ps->g_ps_threshold_hi);
	return sprintf(buf, "%d\n", g_cm3628_data_ps->g_ps_threshold_hi);
}

static int cm3628_store_calibration_prox_hi(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if ( (strict_strtoul(buf, 10, &val) < 0) )
		return -EINVAL;

	/*Get hi threshold adc*/
	a_ps_hi_calibration_adc = (int)val;

	printk("[cm3628] Get calibration_prox_hi value : %d\n", a_ps_hi_calibration_adc );

	/*Write Calibration value*/
	cm3628_write_prox_hi = kmalloc(sizeof(struct write_calvalue), GFP_KERNEL);

	INIT_WORK(&cm3628_write_prox_hi->write_prox_hi_work, write_prox_hi_calibrationvalue_work);

	cm3628_write_prox_hi ->calvalue = a_ps_hi_calibration_adc;

	queue_work(cm3628_workqueue, &cm3628_write_prox_hi->write_prox_hi_work);

	return a_ps_hi_calibration_adc;
}

static DEVICE_ATTR(calibration_prox_hi, S_IRWXU | S_IRWXG| S_IRWXO,
		   cm3628_show_calibration_prox_hi, cm3628_store_calibration_prox_hi);

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
		printk(DBGMSK_PRX_G0"[cm3628] write_psensor_low_calvalue open (%s) fail\n", PSENSOR_CALIBRATION_LO_ASUS_NV_FILE);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk(DBGMSK_PRX_G3"[cm3628] write_psensor_low_calvalue = %d[%s(%d)]\n", 
		this->calvalue, writestr, strlen(writestr));

	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
		printk(DBGMSK_PRX_G0"[cm3628] write_psensor_low_calvalue fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}


static int cm3628_show_calibration_prox_lo(struct device *dev, struct device_attribute *attr, char *buf)
{	
	read_prox_lo_calibrationvalue();

	printk(DBGMSK_PRX_G2"[cm3628] Show_prox_lo_calibration: %d\n", g_cm3628_data_ps->g_ps_threshold_lo);
	
	return sprintf(buf, "%d\n", g_cm3628_data_ps->g_ps_threshold_lo);
}

static int cm3628_store_calibration_prox_lo(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	
	if ( (strict_strtoul(buf, 10, &val) < 0) )
		return -EINVAL;
	
	/*Get hi threshold adc*/
	a_ps_lo_calibration_adc = (int)val;

	printk("[cm3628] Get calibration_prox_hi value : %d\n", a_ps_lo_calibration_adc );

	/*Write Calibration value*/
	cm3628_write_prox_lo = kmalloc(sizeof(struct write_calvalue), GFP_KERNEL);

	INIT_WORK(&cm3628_write_prox_lo->write_prox_lo_work, write_prox_lo_calibrationvalue_work);

	cm3628_write_prox_lo ->calvalue = a_ps_lo_calibration_adc;

	queue_work(cm3628_workqueue, &cm3628_write_prox_lo->write_prox_lo_work);

	return a_ps_lo_calibration_adc;
}

static DEVICE_ATTR(calibration_prox_lo, S_IRWXU | S_IRWXG | S_IRWXO,
		   cm3628_show_calibration_prox_lo, cm3628_store_calibration_prox_lo);
#endif

/////////////////////////////////////////////////////////////////////////////////////////
//---proximity sensor part---
//

/**
 * cm3628_ps_read_reg - read data from cm3628
 * @i2c_client: context of i2c client of cm3628
 *
 * Returns negative errno, else the number of messages executed.
 */
static int cm3628_ps_read_reg( struct i2c_client *client )
{
	int err = 0;
	uint8_t data = 0;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = I2C_M_RD, //read
			.len = 1,
			.buf = &data,
		}
	};

	if (!cm3628_ps_conf_client->adapter)
		return -ENODEV;

	memset(&data, 0, sizeof(data));

	err = i2c_transfer(cm3628_ps_conf_client->adapter, msg, ARRAY_SIZE(msg));

	if (err != ARRAY_SIZE(msg))
		printk("[cm3628] cm3628_ps_read_reg err %d\n", err);

	printk("[cm3628] cm3628_ps_read_reg : %d\n", data);

	return (int)data; // return 2 is expected.
}

static enum proximity_property cm3628_proxmdev_properties[] = {
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

atomic_t cm3628_proxm_update;

static int proxmdev_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	printk("[cm3628][ps] proxmdev_dev_open.\n");

	if (file->f_flags & O_NONBLOCK)
		printk("[cm3628][ps] proxmdl_dev_open (O_NONBLOCK)\n");

	atomic_set(&cm3628_proxm_update, 0); //initialize atomic.

	ret = nonseekable_open(inode, file);

	return ret;
}

static struct file_operations proxmdev_fops = {
    .owner = THIS_MODULE,
    .open = proxmdev_open,
};

struct proximity_class_dev cm3628_proxmDev = {
	.id = SENSORS_PROXIMITY,
	.name = "psensor",
	.num_properties = ARRAY_SIZE(cm3628_proxmdev_properties),
	.properties = cm3628_proxmdev_properties,
	.get_property = cm3628_proxmdev_get_property,
	.put_property = cm3628_proxmdev_put_property,
	.fops = &proxmdev_fops
};

static int cm3628_proxmdev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	switch( property ) 
	{
		case SENSORS_PROP_HI_THRESHOLD:
			printk(DBGMSK_PRX_G4"[cm3628][ps] SENSORS_PROP_HI_THRESHOLD. (%d)\n", g_cm3628_data_ps->g_ps_threshold_hi);
			val->intval = g_cm3628_data_ps->g_ps_threshold_hi;
			break;

		case SENSORS_PROP_LO_THRESHOLD:
			printk(DBGMSK_PRX_G4"[cm3628][ps] SENSORS_PROP_LO_THRESHOLD. (%d)\n", g_cm3628_data_ps->g_ps_threshold_lo);
			val->intval = g_cm3628_data_ps->g_ps_threshold_lo;
			break;

		case SENSORS_PROP_INTERVAL:
			printk(DBGMSK_PRX_G4"[cm3628][ps] SENSORS_PROP_INTERVAL.\n");
			val->intval = g_interval;
			break;

		case SENSORS_PROP_MAXRANGE:
			printk(DBGMSK_PRX_G4"[cm3628][ps] SENSORS_PROP_MAXRANGE.\n");
			val->intval = 1; //keep it 1.0 for OMS.
			break;

		case SENSORS_PROP_RESOLUTION:
			printk(DBGMSK_PRX_G4"[cm3628][ps] SENSORS_PROP_RESOLUTION.\n");
			val->intval = 1;
			break;

		case SENSORS_PROP_VERSION:
			printk(DBGMSK_PRX_G4"[cm3628][ps] SENSORS_PROP_VERSION.\n");
			sprintf(val->strval, "Ver 0");
			break;

		case SENSORS_PROP_CURRENT:
			printk(DBGMSK_PRX_G4"[cm3628][ps] SENSORS_PROP_CURRENT.\n");
			val->intval = 30;
			break;

		case SENSORS_PROP_SWITCH:
			printk(DBGMSK_PRX_G4"[cm3628][ps] get switch = %d.\n", g_cm3628_data_ps->Device_ps_switch_on);
			val->intval = g_cm3628_data_ps->Device_ps_switch_on;
			break;

		case SENSORS_PROP_VENDOR:
			printk(DBGMSK_PRX_G4"[cm3628][ps] SENSORS_PROP_VENDOR.\n");
			sprintf(val->strval, "CAPELLA");
			break;

		/* Add for debug only */
		case SENSORS_PROP_DBG:
			printk(DBGMSK_PRX_G4"[cm3628][ps] dbg = %d.\n", g_proxm_dbg);
			val->intval = g_proxm_dbg;
			//printk(DBGMSK_PRX_G4"[cm36283][ps] ps_IT(%d), ps_PERS(%d), ps_INT(%d), ps_MS(%d)\n",ps_IT, ps_PERS, ps_INT ,ps_MS);
			break;

		case SENSORS_PROP_ADC:
			val->intval = 	get_ps_adc_from_cm3628( false );
			printk(DBGMSK_PRX_G4"[cm3628][ps] get adc property: %d\n", val->intval);
			break;

		case SENSORS_PROP_ATD_STATUS:
		{
			int als_sts =0, als_adc =0, ps_sts =0;

			atd_write_status_and_adc_2(&als_sts, &als_adc, &ps_sts);
			val->intval = ps_sts;
			printk(DBGMSK_PRX_G4"[cm3628][ps] get atd status: %d\n", val->intval);
			break;
		}

		default:
			printk(DBGMSK_PRX_G0"[cm3628][ps] default.\n");
			return -EINVAL;
	}
	return 0;
}

static int cm3628_proxmdev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	int ret = 0;
	int err = 0;
	static bool bFirst = true;
	static bool openfilp = true;

	switch (property) 
	{
		case SENSORS_PROP_SWITCH:
			printk(DBGMSK_PRX_G4"[cm3628][ps] put SENSORS_PROP_SWITCH (%d,%d).\n",
								(val->intval), g_cm3628_data_ps->Device_ps_switch_on);
			if(bFirst) {
				if (err < 0) 
					printk(DBGMSK_PRX_G0"[cm3628][ps] switch init error\n");
				else {
					printk(DBGMSK_PRX_G4"[cm3628][ps] put switch 1st read calvalue\n");
					openfilp = read_prox_hi_calibrationvalue();

					if (openfilp == false ) {
						printk(DBGMSK_PRX_G0"[cm3628][ps] Fail to open file. Get old calvalue : %d\n", 
									g_cm3628_data_ps->g_ps_threshold_hi);
						g_cm3628_data_ps->g_ps_threshold_hi = DEFAULT_PS_THRESHOLD_hi;
					}
					else
						read_prox_lo_calibrationvalue();

					printk("[cm3628] Set prox threshold : %d\n",g_cm3628_data_ps->g_ps_threshold_hi);
					bFirst = false;
				}
			}

			if( (g_cm3628_data_ps->Device_ps_switch_on!= val->intval) && (!g_bIsP01Attached) )	{
				if(val->intval==1)	{	//turn on PS
					ret = cm3628_turn_onoff_proxm(1);
					g_cm3628_data_ps->HAL_ps_switch_on = 1;
					if ( ret == 0 )	{
						//send an init value
						input_report_abs(g_cm3628_data_ps->input_dev, ABS_DISTANCE, 1);
						input_event(g_cm3628_data_ps->input_dev, EV_SYN, SYN_REPORT, 1);
						input_sync(g_cm3628_data_ps->input_dev);

						g_cm3628_data_ps->Device_ps_switch_on = 1;
						g_cm3628_data_ps->pad_proxm_switch_on = 0;
						printk(DBGMSK_PRX_G4"[cm3628][ps] proximity on.\n");
					}
				}else	{	//turn off PS if val->intval==0 or other
					g_cm3628_data_ps->HAL_ps_switch_on = 0;
					g_cm3628_data_ps->pad_proxm_switch_on = 0;

					// disable PS or directly Power off cm3628
					cm3628_turn_onoff_proxm(0);
					printk(DBGMSK_PRX_G4"[cm3628][ps] proximity off.\n");
				}
			}
			else if (g_bIsP01Attached)	{
				printk("[cm36283][ps] Phone on in pad (%d,%d).\n",
								(val->intval), g_cm3628_data_ps->pad_proxm_switch_on );
				g_cm3628_data_ps->pad_proxm_switch_on = val->intval;
			}
			break;

		case SENSORS_PROP_HI_THRESHOLD:
			printk(DBGMSK_PRX_G4"[cm3628][ps] config high THRESHOLD (%d).\n", val->intval);
			if(val->intval>=0 && val->intval<=255) {
				if(g_cm3628_data_ps->g_ps_threshold_hi != val->intval) {
					g_cm3628_data_ps->g_ps_threshold_hi = val->intval;
				}
			}
			else
				printk(DBGMSK_PRX_G0"[cm3628][ps] ERROR!!! OUT OF THRESHOLD (0~255)!!! \n");
			break;

		case SENSORS_PROP_LO_THRESHOLD:
			printk(DBGMSK_PRX_G4"[cm3628][ps] config low THRESHOLD (%d).\n", val->intval);
			if(val->intval>=0 && val->intval<=255) {
				if(g_cm3628_data_ps->g_ps_threshold_lo != val->intval) {
					g_cm3628_data_ps->g_ps_threshold_lo = val->intval;
				}
			}
			else
				printk(DBGMSK_PRX_G0"[cm3628][ps] ERROR!!! OUT OF THRESHOLD (0~255)!!! \n");
			break;

		case SENSORS_PROP_INTERVAL:
			if(1) {
				printk(DBGMSK_PRX_G4"[cm3628][ps] set interval (0x%x)\n", val->intval);
				//interval = val->intval;
			}
			else {
				printk(DBGMSK_PRX_G4"[cm3628][ps] config IT_PS (0x%x)\n", val->intval);
#if 0
				if( (val->intval>=0) && (val->intval<=3) ) {
					gpio_set_value(g_proxm_pwr_pin, 0);
					msleep(1);
					gpio_set_value(g_proxm_pwr_pin, 1);
					gpio_free(g_proxm_pwr_pin);
					ps_IT = val->intval;
					ret = cm36283_reset();
				}
#endif				
			}
			break;

		/* Add for debug only */
		case SENSORS_PROP_DBG:
			g_proxm_dbg = val->intval;
#if 0
			if ( g_proxm_dbg == 1 && g_cm3628_data_ps->Device_ps_switch_on == 1)
				queue_delayed_work(Proximity_test_wq, &Proximity_test_work, HZ * PROXIMITY_TEST_DELAY);
			else if ( g_proxm_dbg == 2 && g_cm3628_data_ps->Device_ps_switch_on == 1)
				queue_delayed_work(Proximity_test_wq, &Proximity_test_work, HZ * PROXIMITY_TEST_DELAY);
			f ( g_proxm_dbg == 0 )
				cancel_delayed_work(&Proximity_test_work);
#endif
			printk(DBGMSK_PRX_G4"[cm3628][ps] dbg = %d.\n", g_proxm_dbg);
			break;

		default:
			printk(DBGMSK_PRX_G0"[cm3628][ps] put default.\n");
			return -EINVAL;
	}
	return 0;
}

static int cm3628_turn_onoff_proxm(bool bOn)
{
	int err = 0;
	unsigned char data_16;

	if(bOn == 1)	{	//power on
		printk("[cm3628][ps] sensor switch, turn on proximity sensor ++.\n");

// ASUS_BSP +++ Peter_Lu "Set L18 sensor power source from standby mode to normal mode for keep sensor power source stable"
		if (ldoa18_regulator == NULL)	{
			ldoa18_regulator = regulator_get( &cm3628_ps_conf_client->dev, "8941_l18");
			if (IS_ERR(ldoa18_regulator)) {
				printk("[cm3628][ps] Get regulator ldoa18 fail!! \n");
				err = -EIO;
			}	else		{
				printk("[cm3628][ps] Get regulator ldoa18 success  \n");
				err = regulator_set_mode(ldoa18_regulator, REGULATOR_MODE_NORMAL);
				if (err < 0)
					printk("[cm3628][ps] Set regulator ldoa18 fail!! \n");
				else
					printk("[cm3628][ps] Set regulator ldoa18 to Normal mode success \n");
			}
		}else	{
			printk("[cm3628][ps] Regulator ldoa18 is exist \n");
			err = regulator_set_mode(ldoa18_regulator, REGULATOR_MODE_NORMAL);
			if (err < 0)
				printk("[cm3628][ps] Set regulator ldoa18 fail!! \n");
			else
				printk("[cm3628][ps] Set regulator ldoa18 to Normal mode success \n");
		}
// ASUS_BSP --- Peter_Lu

		//set ps_threshold  0x01
		err = i2c_smbus_write_byte_data( cm3628_ps_conf_client, CM3628_PS_THD,
								g_cm3628_data_ps->g_ps_threshold_hi );

		//Set 0x02
		err = i2c_smbus_write_byte_data( cm3628_ps_conf_client, CM3628_PS_CANC, 0x02 );

		//Set Proximity config_02 0x03
		data_16 = ( PS_MS<<4 | PS_DIR_INT<<2 | PS_SMART_PERS<<1 | PS_HYS);
		err = i2c_smbus_write_byte_data(cm3628_ps_conf_client, CM3628_PS_CONF2, data_16);
		if(err < 0) {
			printk("[cm3628][ps] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_PS_CONF2, err);
			return -1;
		}else
			printk(DBGMSK_PRX_G4"[cm3628][ps] set ps_config(0x%x), pdata=0x%x\n", CM3628_PS_CONF2, data_16);


		/* Enable PS & interrupt */
		//set Proximity config_01 0x00
		data_16 = ( PS_DR<<6 | PS_IT<<4 | PS_PERS<<2 | PS_INT_EN_ON<<1 | PS_SD_ON);
		err = i2c_smbus_write_byte_data( cm3628_ps_conf_client, CM3628_PS_CONF1, data_16 );
		
		if(err < 0) {
			printk("[cm3628][ps] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_PS_CONF1, err);
			return -1;
		}else
			printk("[cm3628][ps] set ps_config(0x%x), pdata=0x%x\n", CM3628_PS_CONF1, data_16);

		g_cm3628_data_ps->Device_ps_switch_on = 1;

		printk("[cm3628][ps]turn on proximity sensor --.\n");
	}
	else		//power off
	{
		printk("[cm3628][ps] sensor switch, turn off proximity sensor ++.\n");
		/* Disable PS & interrupt */
		//set Proximity config_01 0x00
		data_16 = ( PS_DR<<6 | PS_IT<<4 | PS_PERS<<2 | PS_INT_EN<<1 | PS_SD_OFF);

		err = i2c_smbus_write_byte_data( cm3628_ps_conf_client, CM3628_PS_CONF1, data_16 );
		if(err < 0) {
			printk("[cm3628][ps] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_PS_CONF1, err);
			return -1;
		}else
			printk("[cm3628][ps] set ps_config(0x%x), pdata=0x%x\n", CM3628_PS_CONF1, data_16);

		g_cm3628_data_ps->Device_ps_switch_on = 0;

// ASUS_BSP +++ Peter_Lu "Set L18 sensor power source from standby mode to normal mode for keep sensor power source stable"
		if (ldoa18_regulator == NULL)	{
			ldoa18_regulator = regulator_get( &cm3628_ps_conf_client->dev, "8941_l18");
			if (IS_ERR(ldoa18_regulator)) {
				printk("[cm3628][ps] Get regulator ldoa18 fail!! \n");
				err = -EIO;
			}	else		{
				printk("[cm3628][ps] Get regulator ldoa18 success  \n");
				err = regulator_set_mode(ldoa18_regulator, REGULATOR_MODE_STANDBY);
				if (err < 0)
					printk("[cm3628][ps] Set regulator ldoa18 fail!! \n");
				else
					printk("[cm3628][ps] Set regulator ldoa18 to Normal mode success \n");
			}
		}else	{
			printk("[cm3628][ps] Regulator ldoa18 is exist \n");
			err = regulator_set_mode(ldoa18_regulator, REGULATOR_MODE_STANDBY);
			if (err < 0)
				printk("[cm3628][ps] Set regulator ldoa18 fail!! \n");
			else
				printk("[cm3628][ps] Set regulator ldoa18 to Normal mode success \n");
		}
// ASUS_BSP --- Peter_Lu

		printk("[cm3628][ps] turn off proximity sensor --.\n");
	}
	return 0;
}

static int get_ps_adc_from_cm3628(bool trigger_by_interrupt)
{
	int data = 0;

	if (trigger_by_interrupt)
		cm3628_ara_read_reg( cm3628_ara_client );

	data = cm3628_ps_read_reg(cm3628_ps_conf_client);
	printk("[cm3628][ps] Read ps data : %d\n", data);

	return data;
}

/////////////////////////////////////////////////////////////////////////////////////////
//---cm3628 interrupt part---
//

/**
 * cm3628_ara_read_reg - read data from cm3628
 * @i2c_client: context of i2c client of cm3628
 *
 * Returns negative errno, else the number of messages executed.
 */
static int cm3628_ara_read_reg( struct i2c_client *client )
{
	int err = 0;
	uint8_t data = 0;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = I2C_M_RD, //read
			.len = 1,
			.buf = &data,
		}
	};

	if (!cm3628_ara_client->adapter)
		return -ENODEV;

	memset(&data, 0, sizeof(data));

	err = i2c_transfer(cm3628_ara_client->adapter, msg, ARRAY_SIZE(msg));

	if (err != ARRAY_SIZE(msg))
		printk("[cm3628] cm3628_ara_read_reg err %d\n", err);

	printk("[cm3628] cm3628_ara_read_reg : %d\n", data);

	return (int)data; // return 2 is expected.
}

static void cm3628_software_reset(void)		//hardware reset sensor in case of sensor stall
{
	printk("[cm3628][isr] sensor reset on/off: g_HAL_als_switch_on=%d, proxm_switch_on=%d ++ \n",
				g_HAL_als_switch_on, g_cm3628_data_ps->Device_ps_switch_on);
/*
	gpio_set_value(CM3628_GPIO_PROXIMITY_PWR_EN, 0);
	msleep(1);
	gpio_set_value(CM3628_GPIO_PROXIMITY_PWR_EN, 1);	
	msleep(5);
	cm3628_reg_reset(CM3628_ARA_MODE);
*/	
	cm3628_turn_onoff_als(0);
	cm3628_turn_onoff_proxm(0);
	cm3628_reg_reset(CM3628_ARA_MODE);
	
	g_cm3628_reset = 1;

	if (g_HAL_als_switch_on == 1) 
		cm3628_turn_onoff_als(1);
    
	if(g_cm3628_data_ps->Device_ps_switch_on == 1) {
		cm3628_turn_onoff_proxm(1);
		queue_delayed_work(cm3628_delay_workqueue, &cm3628_proximity_interrupt_delay_work, 0 );
	}

	g_cm3628_reset = 0;

	return;
}

static irqreturn_t cm3628_als_proxm_interrupt_handler(int irq, void *dev_id)
{	
	printk(DBGMSK_PRX_G2"[cm3628][isr] interrupt handler ++\n");

	queue_delayed_work(cm3628_workqueue, &cm3628_ISR_work, 0);
	if (g_cm3628_data_ps->proxm_detect_object_state == 1)	
		wake_lock_timeout(&proximity_wake_lock, 1 * HZ);

	printk(DBGMSK_PRX_G2"[cm3628][isr] interrupt handler --\n");

	return IRQ_HANDLED;
}

static void cm3628_interrupt_handler(struct work_struct *work)
{
	int ret = 0;
	
	if((g_cm3628_data_ps->Device_ps_switch_on == 1) ||(g_cm3628_data_as->Device_als_switch_on == 1)
			|| (g_ambient_suspended == 1)) {
			
		ret  = cm3628_ara_read_reg( cm3628_ara_client );
		printk("[cm3628][isr] INT_FLAG = (0x%x)\n", ret);
		if( ( ret>>1 ) == cm3628_als_conf_client->addr ) {			//8 bit dec for 0x60 (7-bit)
			printk("[cm3628][isr] als triggered\n");
			queue_delayed_work(cm3628_delay_workqueue, &cm3628_light_interrupt_delay_work, 0);
		}
		if( ( ret>>1 ) == cm3628_ps_conf_client->addr ) {		//8 bit dec for 0x61 (7-bit)
			printk("[cm3628][isr] ps triggered\n");
			if (g_ambient_suspended==1) {
				printk("[cm3628][isr] setting g_cm3628_earlysuspend_int = %d\n", g_cm3628_earlysuspend_int);
				g_cm3628_earlysuspend_int = 1;
			}
			queue_delayed_work(cm3628_delay_workqueue, &cm3628_proximity_interrupt_delay_work, 0);
		}
		
		if(ret < 0) {
			printk("[cm3628][isr] I2C error!INT_FLAG = (0x%x)\n", ret);
			printk("[cm3628][isr] Software reset triggered\n");
			cm3628_software_reset();
		}
		else
			printk("[cm3628][isr] No interrupt, INT_FLAG = (0x%x)\n", ret);
	}

	return;
}

static void cm3628_Proximity_work(struct work_struct *work)
{
	int adc = 0;
	adc = get_ps_adc_from_cm3628( false );

	printk(DBGMSK_PRX_G4"/----------------------------------------------------\n");
	printk(DBGMSK_PRX_G4"[cm3628][ps] PS_data = %d\n",adc);
	
	if( adc >= g_cm3628_data_ps->g_ps_threshold_hi) {  //panel off
		input_report_abs(g_cm3628_data_ps->input_dev, ABS_DISTANCE, 0);
		input_sync(g_cm3628_data_ps->input_dev);
		g_cm3628_data_ps->proxm_detect_object_state = 1;
		printk("[cm3628][ps] trigger panel off\n");
	}else	{
		input_report_abs(g_cm3628_data_ps->input_dev, ABS_DISTANCE, 1);
		input_sync(g_cm3628_data_ps->input_dev);
		wake_unlock(&proximity_wake_lock);
		g_cm3628_data_ps->proxm_detect_object_state = 0;
		printk("[cm3628][ps] trigger panel on\n");
	}

#ifndef INPUT_EVENT_MODE
	atomic_inc(&cm3628_proxm_update);
	printk(DBGMSK_PRX_G4"[cm3628][ps] proxm_interrupt_handler, state_change\n");
	printk(DBGMSK_PRX_G4"[cm3628][ps] proxm_interrupt_handler fire(%d)\n",atomic_read(&cm3628_proxm_update));
	printk(DBGMSK_PRX_G4"/----------------------------------------------------\n");
#endif
}

static void cm3628_light_work(struct work_struct *work)
{
	int err = 0;
	g_cm3628_data_as->light_adc = get_als_adc_from_cm3628( false );

	g_cm3628_data_as->g_als_threshold_hi = (g_cm3628_data_as->light_adc*105/100);
	g_cm3628_data_as->g_als_threshold_lo = (g_cm3628_data_as->light_adc*95/100);
	
	/* Set ALS high threshold*/
	err = i2c_smbus_write_byte_data( cm3628_als_conf_client, CM3628_ALS_WH_M,
							((g_cm3628_data_as->g_als_threshold_hi>> 8 ) & 0xFF) );
	err = i2c_smbus_write_byte_data( cm3628_als_conf_client, CM3628_ALS_WH_L,
							(g_cm3628_data_as->g_als_threshold_hi& 0xFF));
	if(err < 0) 
		printk("[cm3628][als] (%s):Set High threshold error=%d\n",__FUNCTION__, err);

	/* Set ALS low threshold*/
	err = i2c_smbus_write_byte_data( cm3628_als_conf_client, CM3628_ALS_WL_M,
							((g_cm3628_data_as->g_als_threshold_lo>> 8 ) & 0xFF)  );
	err = i2c_smbus_write_byte_data( cm3628_als_conf_client, CM3628_ALS_WL_L,
							(g_cm3628_data_as->g_als_threshold_lo& 0xFF) );
	if(err < 0) 
		printk("[cm3628][als] (%s):Set Low threshold error=%d\n",__FUNCTION__, err);

	
	get_calibrated_lux_from_cm3628_adc(g_cm3628_data_as->light_adc);

	printk(DBGMSK_PRX_G3"[cm3628][als] Setting threshold: adc=%d, g_als_threshold_hi=%d, g_als_threshold_lo=%d\n",
		g_cm3628_data_as->light_adc, g_cm3628_data_as->g_als_threshold_hi, g_cm3628_data_as->g_als_threshold_lo);
	
	als_lux_report_event( g_cm3628_data_as->light_lux );

	/* For Proximity interrupt issue */
	if(g_cm3628_data_ps->Device_ps_switch_on == 1) {
		if (g_ambient_suspended == 1) {
			printk("[cm36283][isr] setting g_cm36283_earlysuspend_int = %d\n", g_cm3628_earlysuspend_int);
			g_cm3628_earlysuspend_int = 1;
		}
		queue_delayed_work(cm3628_delay_workqueue, &cm3628_proximity_interrupt_delay_work, 0);
	}
	
	if(g_cm3628_data_ps->proxm_detect_object_state == 1) 
		wake_unlock(&proximity_wake_lock);
}

/////////////////////////////////////////////////////////////////////////////////////////
//---cm3628 attribute part---
//

/**
 * cm3628_als_read_reg - write an I2C message to cm3628
 * @i2c_client: context of i2c client of cm3628
 *
 * Returns negative errno, else the number of messages executed.
 */
 static int cm3628_als_read_reg( struct i2c_client *client )
{
	int err = 0;
	uint8_t data = 0;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = I2C_M_RD, //read
			.len = 1,
			.buf = &data,
		}
	};

	if (!cm3628_als_conf_client->adapter)
		return -ENODEV;

	memset(&data, 0, sizeof(data));

	err = i2c_transfer(cm3628_als_conf_client->adapter, msg, ARRAY_SIZE(msg));

	if (err != ARRAY_SIZE(msg))
		printk("[cm3628] cm3628_als_read_reg err %d\n", err);

	printk("[cm3628] cm3628_als_read_reg : %d\n", data);

	return (int)data; // return 2 is expected.
}

static enum proximity_property cm3628_ambientDev_properties[] = {
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

atomic_t cm3628_ambient_update;

static int ambientDev_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	printk(DBGMSK_PRX_G3"[cm3628][als] ambientdl_dev_open \n");

	if (file->f_flags & O_NONBLOCK)
		printk(DBGMSK_PRX_G2"[cm3628][als] ambientdl_dev_open (O_NONBLOCK)\n");

	atomic_set(&cm3628_ambient_update, 0); //initialize atomic.

	ret = nonseekable_open(inode, file);

	return ret;
}

static struct file_operations ambientDev_fops = {
	.owner = THIS_MODULE,
	.open = ambientDev_open,
};

struct proximity_class_dev cm3628_ambientDev = {
	.id = SENSORS_LIGHT,
	.name = "lsensor",
	.num_properties = ARRAY_SIZE(cm3628_ambientDev_properties),
	.properties = cm3628_ambientDev_properties,
	.get_property = cm3628_ambientDev_get_property,
	.put_property = cm3628_ambientDev_put_property,
	.fops = &ambientDev_fops
};

static int cm3628_ambientDev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	printk(DBGMSK_PRX_G3"[cm3628][als] ambientdl_get_property +.\n");

	switch( property )
	{
		case SENSORS_PROP_HI_THRESHOLD:
			printk(DBGMSK_PRX_G3"[cm3628][als] SENSORS_PROP_HI_THRESHOLD. (%d)\n", g_cm3628_data_as->g_als_threshold_hi);
			val->intval = g_cm3628_data_as->g_als_threshold_hi;
			break;

		case SENSORS_PROP_LO_THRESHOLD:
			printk(DBGMSK_PRX_G3"[cm3628][als] SENSORS_PROP_LO_THRESHOLD. (%d)\n", g_cm3628_data_as->g_als_threshold_lo);
			val->intval = g_cm3628_data_as->g_als_threshold_lo;
			break;

		case SENSORS_PROP_INTERVAL:
			printk(DBGMSK_PRX_G3"[cm3628][als] config GAIN_ALS by using \"echo XXX > /sys/class/sensors/sensor4/interval\" XXX is only 0~3. \n");
			printk(DBGMSK_PRX_G3"[cm3628][als] SENSORS_PROP_INTERVAL.\n");
			val->intval = g_interval;
			break;

		case SENSORS_PROP_MAXRANGE:
			printk(DBGMSK_PRX_G3"[cm3628][als] SENSORS_PROP_MAXRANGE.\n");
			val->intval = 128;  
			break;

		case SENSORS_PROP_RESOLUTION:
			printk(DBGMSK_PRX_G3"[cm3628][als] SENSORS_PROP_RESOLUTION.\n");
			val->intval = 1;
			break;

		case SENSORS_PROP_VERSION:
			printk(DBGMSK_PRX_G3"[cm3628][als] SENSORS_PROP_VERSION.\n");
			val->intval = 1;
			break;

		case SENSORS_PROP_CURRENT:
			printk(DBGMSK_PRX_G3"[cm3628][als] SENSORS_PROP_CURRENT.\n");
			val->intval = 30;
			break;

		case SENSORS_PROP_SWITCH:
			printk(DBGMSK_PRX_G3"[cm3628][als] get switch = %d.\n", g_HAL_als_switch_on);
			val->intval = g_HAL_als_switch_on;
			break;

		case SENSORS_PROP_VENDOR:
			printk(DBGMSK_PRX_G3"[cm3628][als] SENSORS_PROP_VENDOR.\n");
			sprintf(val->strval, "CAPELLA");
			break;

		/* Add for debug only */
		case SENSORS_PROP_DBG:
			val->intval = g_ambient_dbg;
			printk(DBGMSK_PRX_G3"[cm3628][als] dbg = %d.\n", g_ambient_dbg);
			break;

		case SENSORS_PROP_ADC:
			val->intval = get_als_adc_from_cm3628( false );
			printk(DBGMSK_PRX_G3"[cm3628][als] get adc property: %d\n", val->intval);
			break;
#if 0
		case SENSORS_PROP_K_ADC:
			val->intval = get_adc_calibrated_lux_from_cm3628(get_als_adc_from_cm3628( false ));
			printk(DBGMSK_PRX_G3"[cm3628][als] get k_adc property: %d\n", val->intval);
			break;

		case SENSORS_PROP_LUX:
			val->intval = g_lux_light;
			printk(DBGMSK_PRX_G3"[cm3628][als] get lux property: %d\n", val->intval);
			break;
#endif

		case SENSORS_PROP_ATD_STATUS:
		{
			int als_sts =0, als_adc =0, ps_sts =0;

			atd_write_status_and_adc_2(&als_sts, &als_adc, &ps_sts);
			val->intval = als_sts;
			printk(DBGMSK_PRX_G3"[cm3628][als] get atd status: %d\n", val->intval);
			break;
		}

		case SENSORS_PROP_ATD_ADC:
		{
			int als_sts =0, als_adc =0, ps_sts =0;
			atd_write_status_and_adc_2(&als_sts, &als_adc, &ps_sts);

			val->intval = als_adc;
			printk(DBGMSK_PRX_G3"[cm3628][als] get atd adc: %d\n", val->intval);
			break;
		}

		default:
			printk(DBGMSK_PRX_G0"[cm3628][als] default\n");

		return -EINVAL;

	}
	printk( DBGMSK_PRX_G3"[cm3628]: ambientdl_get_property -.\n");

	return 0;
}

static int cm3628_ambientDev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	static bool bFirst = true;
	static bool openfilp = true;

	switch (property) 
	{
		case SENSORS_PROP_SWITCH:
			printk(DBGMSK_PRX_G3"[cm3628][als] put SENSORS_PROP_SWITCH (%d,%d).\n",
				(val->intval), g_HAL_als_switch_on);

			//read calibration value
			if(bFirst) {
				printk(DBGMSK_PRX_G3"[cm3628][als] put switch 1st read calvalue\n");
				openfilp = read_lightsensor_calibrationvalue();

				/*Surpport old calibration value*/
				if ( g_cm3628_light_gain_calibration <= 0 || openfilp == false )
					printk(DBGMSK_PRX_G3"[cm3628][als] Get old calvalue or fail\n" );
				else
					read_lightsensor_shiftvalue();
				printk("[cm3628] Set calibration and shift: %d.%d , %d\n",
					g_cm3628_light_gain_calibration/a_als_calibration_accuracy,
					g_cm3628_light_gain_calibration%a_als_calibration_accuracy,
					g_cm3628_light_shift_calibration );
				bFirst = false;
			}

			if(val->intval > 0)	{
				g_HAL_als_switch_on = 1;
				g_cm3628_data_as->HAL_als_switch_on = 1;
			}
			else	{
				g_HAL_als_switch_on = 0;
				g_cm3628_data_as->Device_als_switch_on = 0;
			}

			if( g_bIsP01Attached )	{
				printk(DBGMSK_PRX_G3"[cm3628][als] sensor switch, turn on/off al3010: %d\n", g_HAL_als_switch_on);
				als_lux_report_event(0);
				set_als_power_state_of_P01(g_HAL_als_switch_on);
			}
			else	 {
				printk(DBGMSK_PRX_G3"[cm3628][als] sensor switch, turn on/off cm3628: %d\n", g_HAL_als_switch_on);
				cm3628_turn_onoff_als(g_HAL_als_switch_on);
			}
			break;

		case SENSORS_PROP_LO_THRESHOLD:
			break;

		case SENSORS_PROP_HI_THRESHOLD:        
			break;

		case SENSORS_PROP_INTERVAL:
			printk(DBGMSK_PRX_G3"[cm3628][als] put SENSORS_PROP_INTERVAL. %d\n", val->intval);
			if(val->intval < 100)
				g_interval = 100;
			else
				g_interval = val->intval;

			break;

		/* Add for debug only */
		case SENSORS_PROP_DBG:
			g_ambient_dbg = val->intval;
#if 0
			if ( g_ambient_dbg == 1 && g_cm3628_data_as->Device_als_switch_on == 1)
				queue_delayed_work(cm3628_debug_test_wq, &cm3628_als_test_work, HZ * DEBUG_TEST_DELAY);
			else if ( g_ambient_dbg == 2 && g_cm3628_data_as->Device_als_switch_on == 1)
				queue_delayed_work(cm3628_debug_test_wq, &cm3628_als_test_work, HZ * DEBUG_TEST_DELAY);
			else if ( g_ambient_dbg == 3 && g_cm3628_data_as->Device_als_switch_on == 1)
				set_camera_autobrightness_mode(g_ambient_dbg);			//For dbg camera backlight
			if ( g_ambient_dbg == 0 )
				cancel_delayed_work(&cm3628_als_test_work);
#endif
			printk(DBGMSK_PRX_G3"[cm3628][als] dbg = %d.\n", g_ambient_dbg);
			break;

		case SENSORS_PROP_CALIBRATION:
			g_cm3628_light_gain_calibration = val->intval;
			printk(DBGMSK_PRX_G3"[cm3628][als] Gain calibration val = %d.%d\n", 
							g_cm3628_light_gain_calibration/a_als_calibration_accuracy,
							g_cm3628_light_gain_calibration%a_als_calibration_accuracy);
			break;

		default:
			printk(DBGMSK_PRX_G0"[cm3628][als] put default.\n");
			return -EINVAL;
	}
	return 0;
}

static int cm3628_turn_onoff_als(bool bOn)
{
	uint8_t data_8 = 0;
	uint16_t data_16 = 0;
	int err = 0;

	printk(DBGMSK_PRX_G3"[cm3628][als]++Turn onoff ambient sensor\n");

	if (g_cm3628_data_as->Device_als_switch_on != bOn || g_cm3628_reset==1) {
		if ( bOn == 1 )	{
			printk(DBGMSK_PRX_G3"[cm3628][als] turn on light sensor ++.\n");

			/* Clear ara INT*/
			err = cm3628_ara_read_reg( cm3628_ara_client );
			printk(DBGMSK_PRX_G3"[cm3628][als] Clean ARA, data=0x%x\n", data_8);		

			/* Set 0x01 ALS CONF2 */
			data_16 = ( ALS_RESET<<6 | ALS_AV<<4 | ALS_HS<<3 | ALS_THD );
			err = i2c_smbus_write_byte_data(cm3628_als_conf_client, CM3628_ALS_CONF2, data_16 );
			if(err < 0) {
				printk("[cm3628][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_ALS_CONF2, err);
				return -1;
			}else
				printk(DBGMSK_PRX_G3"[cm3628][als] set als_config(0x%x), data=0x%x\n", CM3628_ALS_CONF2, data_16);

			/* Set 0x02,0x03 ALS High threshold */
			err = i2c_smbus_write_byte_data(cm3628_als_conf_client, CM3628_ALS_WH_M, 0x00 );
			if(err < 0) {
				printk("[cm3628][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_ALS_WH_M, err);
				return -1;
			}else
				printk(DBGMSK_PRX_G3"[cm3628][als] set als_config(0x%x), data=0x%x\n", CM3628_ALS_WH_M, data_16);

			err = i2c_smbus_write_byte_data(cm3628_als_conf_client, CM3628_ALS_WH_L, 0x00);
			if(err < 0) {
				printk("[cm3628][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_ALS_WH_L, err);
				return -1;
			}else
				printk(DBGMSK_PRX_G3"[cm3628][als] set als_config(0x%x), data=0x%x\n", CM3628_ALS_WH_L, data_16);

			/* Set 0x04,0x05 ALS Low threshold */
			err = i2c_smbus_write_byte_data(cm3628_als_conf_client, CM3628_ALS_WL_M, 0x00);
			if(err < 0) {
				printk("[cm3628][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_ALS_WL_M, err);
				return -1;
			}else
				printk(DBGMSK_PRX_G3"[cm3628][als] set als_config(0x%x), data=0x%x\n", CM3628_ALS_WL_M, data_16);

			err = i2c_smbus_write_byte_data(cm3628_als_conf_client, CM3628_ALS_WL_L, 0x00);
			if(err < 0) {
				printk("[cm3628][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_ALS_WL_L, err);
				return -1;
			}else
				printk(DBGMSK_PRX_G3"[cm3628][als] set als_config(0x%x), data=0x%x\n", CM3628_ALS_WL_L, data_16);
	
			/* Set 0x00 ALS CONF1 */
			/* Enable ALS & interrupt */
			data_16 = ( ALS_IT<<6 | ALS_PERS<<4 | ALS_THRES<<2 | ALS_INT_EN_ON<<1 | ALS_SD_ON );
			err = i2c_smbus_write_byte_data(cm3628_als_conf_client, CM3628_ALS_CONF1, data_16);
			if(err < 0) {
				printk("[cm3628][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_ALS_CONF1, err);
				return -1;
			}else
				printk(DBGMSK_PRX_G3"[cm3628][als] set als_config(0x%x), data=0x%x\n", CM3628_ALS_CONF1, data_16);

			g_cm3628_data_as->Device_als_switch_on = 1;

			//Immediately report lux after sensor switched on
			queue_delayed_work(cm3628_delay_workqueue, &cm3628_light_interrupt_delay_work, 20);
		
			printk("[cm3628][als] turn on light sensor --.\n");
		}
		else	{
			printk(DBGMSK_PRX_G3"[cm3628][als] turn off light sensor ++.\n");

			/* Set 0x00 ALS CONF1 */
			/* Disable ALS & interrupt */
			data_16 = ( ALS_IT<<6 | ALS_PERS<<4 | ALS_THRES<<2 | ALS_INT_EN_OFF<<1 | ALS_SD_OFF );
			err = i2c_smbus_write_byte_data(cm3628_als_conf_client, CM3628_ALS_CONF1, data_16);
			if(err < 0) {
				printk("[cm3628][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_ALS_CONF1, err);
				return -1;
			}else
				printk(DBGMSK_PRX_G3"[cm3628][als] set als_config(0x%x), data=0x%x\n", CM3628_ALS_CONF1, data_16);

			g_cm3628_data_as->Device_als_switch_on = 0;
		
			printk("[cm3628][als] turn off light sensor --.\n");
		}
	}
	return 0;
}

static int get_als_adc_from_cm3628(bool trigger_by_interrupt)
{
	int adc = 0;
	int lsb = 0, msb = 0;

	if (trigger_by_interrupt)
		cm3628_ara_read_reg( cm3628_ara_client );

	lsb = cm3628_als_read_reg( cm3628_als_conf_client );

	msb = cm3628_als_read_reg( cm3628_als_conf_client );

	adc = (u32)((msb << 8) | lsb) ;

	printk("[cm3628][als] read-HiByte (%d). read-LoByte (%d)\n", msb, lsb );
	printk("[cm3628][als] Get adc : %d\n", adc );

	return adc;
}

static int get_calibrated_lux_from_cm3628_adc(int adc)
{
	//int i = 0;
	uint32_t lux = 0;

	lux = (adc * g_cm3628_light_gain_calibration / a_als_calibration_accuracy)
			 + g_cm3628_light_shift_calibration;     //apply calibration value ( Because the panel level, calibration number probably over 10000)
	printk("[cm3628][als] Get lux : %d\n", lux );
	
	g_cm3628_data_as->light_k_adc = lux;

	/*Get Lux*/
	if ( adc < 10 ||g_cm3628_data_as->light_k_adc < 0 )
		g_cm3628_data_as->light_k_adc = 0;

	g_cm3628_data_as->light_lux = g_cm3628_data_as->light_k_adc;
/*
	for(i=1;i<g_max_light_level;i++) {
		if( lux < g_cm3628_light_map[i] ) {
			g_cm3628_data_as->light_lux = g_cm3628_light_map[ i -1 ];
			break;
		}
		else if( lux > g_cm3628_light_map[g_max_light_level - 1] )	{
			g_cm3628_data_as->light_lux = g_cm3628_light_map[ g_max_light_level -1 ];
			break;
		}
	}
*/
	if ( g_cm3628_data_as->light_lux > g_cm3628_light_map[ g_max_light_level -1 ])
		g_cm3628_data_as->light_lux = g_cm3628_light_map[ g_max_light_level -1 ];

	if( g_cm3628_data_as->light_lux != g_cm3628_data_as->last_light_lux )
		g_cm3628_data_as->last_light_lux = g_cm3628_data_as->light_lux;

/*
	if(g_cm36283_light != g_last_cm36283_light || g_cm36283_light_first) {
		g_last_cm36283_light = g_cm36283_light;
		als_lux_report_event( g_cm36283_light);
		g_cm3628_light_first=0;
	}
*/
	printk(DBGMSK_PRX_G3"[cm3628][als] adc=%d, k_adc=%d, lux=%d, last=%d\n", adc, lux, g_cm3628_data_as->light_lux, g_cm3628_data_as->last_light_lux);

	return lux;
}

/////////////////////////////////////////////////////////////////////////////////////////
//---cm3628_attributes part---
//
static int cm3628_store_adc(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int als_state = 0, err = 0;
	unsigned long val;
	unsigned char data;
	
	if ( (strict_strtoul(buf, 10, &val) < 0) )
		return -EINVAL;

	als_state = (int)val;

	if ( als_state == 1 )	{
		printk("[cm3628][als] turn on light sensor ++.\n");

		/* Set 0x01 ALS CONF2 */
		data = ( ALS_RESET<<6 | ALS_AV<<4 | ALS_HS<<3 | ALS_THD );
		err = i2c_smbus_write_byte_data( cm3628_als_conf_client, CM3628_ALS_CONF2, data );
		if(err < 0) {
			printk("[cm3628][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_PS_CONF2, err);
			return -1;
		}else
			printk("[cm3628][als] set als_config(0x%x), data=0x%x\n", CM3628_PS_CONF2, data);

		/* Set 0x02,0x03 ALS High threshold */
		err = i2c_smbus_write_byte_data( cm3628_als_conf_client, CM3628_ALS_WH_M, 0x00 );
		err = i2c_smbus_write_byte_data( cm3628_als_conf_client, CM3628_ALS_WH_L, 0x00 );
		if(err < 0) {
			printk("[cm3628][als] (%s):Set High threshold error=%d\n",__FUNCTION__, err);
			return -1;
		}

		/* Set 0x04,0x05 ALS Low threshold */
		err = i2c_smbus_write_byte_data( cm3628_als_conf_client, CM3628_ALS_WL_M, 0x00 );
		err = i2c_smbus_write_byte_data( cm3628_als_conf_client, CM3628_ALS_WL_L, 0x00 );
		if(err < 0) {
			printk("[cm3628][als] (%s):Set Low threshold error=%d\n",__FUNCTION__, err);
			return -1;
		}

		/* Set 0x00 ALS CONF1 */
		/* Enable ALS & interrupt */
		data = ( ALS_IT<<6 | ALS_PERS<<4 | ALS_THRES<<2 | ALS_INT_EN_ON<<1 | ALS_SD_NO );
		err = i2c_smbus_write_byte_data( cm3628_als_conf_client, CM3628_ALS_CONF1, data );
		if(err < 0) {
			printk("[cm3628][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_PS_CONF1, err);
			return -1;
		}else
			printk("[cm3628][als] set als_config(0x%x), data=0x%x\n", CM3628_PS_CONF1, data);

		g_cm3628_data_as->Device_als_switch_on = 1;
		
		printk("[cm3628][als] turn on light sensor --.\n");
	}
	else	{
		printk("[cm3628][als] turn off light sensor ++.\n");

		/* Set 0x00 ALS CONF1 */
		/* Disable ALS & interrupt */
		data = ( ALS_IT<<6 | ALS_PERS<<4 | ALS_THRES<<2 | ALS_INT_EN<<1 | ALS_SD );
		err = i2c_smbus_write_byte_data( cm3628_als_conf_client, CM3628_ALS_CONF1, data );
		if(err < 0) {
			printk("[cm3628][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_PS_CONF1, err);
			return -1;
		}else
			printk("[cm3628][als] set als_config(0x%x), data=0x%x\n", CM3628_PS_CONF1, data);

		g_cm3628_data_as->Device_als_switch_on = 0;
		
		printk("[cm3628][als] turn off light sensor --.\n");
	}

	return 1;
}

static int cm3628_show_adc(struct device *dev, struct device_attribute *attr, char *buf)
{
	int adc = 0;

	adc = get_als_adc_from_cm3628( false );

	return sprintf(buf, "%d\n", adc);
}

static DEVICE_ATTR(adc, S_IRWXU | S_IRWXG | S_IROTH, cm3628_show_adc, cm3628_store_adc);

static int cm3628_show_proxm (struct device *dev, struct device_attribute *attr, char *buf)
{
	int adc;

	adc = get_ps_adc_from_cm3628( false );
	printk("[cm3628][als] Get adc : %d\n", adc );

	return sprintf(buf, "%d\n", adc);
}

static DEVICE_ATTR(proxm, S_IRWXU | S_IRWXG  | S_IROTH, cm3628_show_proxm, NULL);

static struct attribute *cm3628_attributes[] = {
#ifdef ASUS_FACTORY_BUILD
	&dev_attr_calibration_200.attr,
	&dev_attr_calibration_1000.attr,
	&dev_attr_calibration_prox_lo.attr,
	&dev_attr_calibration_prox_hi.attr,
#endif
	&dev_attr_proxm.attr,
	&dev_attr_adc.attr,
	NULL
};

static const struct attribute_group cm3628_attr_group = {
    .name = "cm3628",
	.attrs = cm3628_attributes,
};

/*.........................................Sensoer init.........................................*/
static int cm3628_reg_reset(unsigned int mode)
{
	int err = 0;

	printk("[cm3628] cm3628_reg_reset ++ mode: 0x%x\n", mode);

	if (mode == CM3628_ALS_MODE) {
		cm3628_ara_read_reg( cm3628_ara_client );

		/* Set 0x00 ALS CONF1 */
		err = i2c_smbus_write_byte_data(cm3628_als_conf_client, CM3628_ALS_CONF1, (ALS_THRES<<2 | ALS_SD_OFF));
		if(err < 0) {
			printk("[cm3628][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_ALS_CONF1, err);
			return -1;
		}else
			printk(DBGMSK_PRX_G3"[cm3628][als] set als_config(0x%x), data=0x%x\n", CM3628_ALS_CONF1, (ALS_THRES<<2 | ALS_SD_OFF));

		/* Set 0x01 ALS CONF2 */
		err = i2c_smbus_write_byte_data(cm3628_als_conf_client, CM3628_ALS_CONF2, CM3628_REG_INIT );
		if(err < 0) {
			printk("[cm3628][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_ALS_CONF2, err);
			return -1;
		}else
			printk(DBGMSK_PRX_G3"[cm3628][als] set als_config(0x%x), data=0x%x\n", CM3628_ALS_CONF2, CM3628_REG_INIT);

		/* Set 0x02,0x03 ALS High threshold */
		err = i2c_smbus_write_byte_data(cm3628_als_conf_client, CM3628_ALS_WH_M, CM3628_REG_INIT );
		if(err < 0) {
			printk("[cm3628][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_ALS_WH_M, err);
			return -1;
		}else
			printk(DBGMSK_PRX_G3"[cm3628][als] set als_config(0x%x), data=0x%x\n", CM3628_ALS_WH_M, CM3628_REG_INIT);

		err = i2c_smbus_write_byte_data(cm3628_als_conf_client, CM3628_ALS_WH_L, CM3628_REG_INIT);
		if(err < 0) {
			printk("[cm3628][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_ALS_WH_L, err);
			return -1;
		}else
			printk(DBGMSK_PRX_G3"[cm3628][als] set als_config(0x%x), data=0x%x\n", CM3628_ALS_WH_L, CM3628_REG_INIT);

		/* Set 0x04,0x05 ALS Low threshold */
		err = i2c_smbus_write_byte_data(cm3628_als_conf_client, CM3628_ALS_WL_M, CM3628_REG_INIT);
		if(err < 0) {
			printk("[cm3628][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_ALS_WL_M, err);
			return -1;
		}else
			printk(DBGMSK_PRX_G3"[cm3628][als] set als_config(0x%x), data=0x%x\n", CM3628_ALS_WL_M, CM3628_REG_INIT);

		err = i2c_smbus_write_byte_data(cm3628_als_conf_client, CM3628_ALS_WL_L, CM3628_REG_INIT);
		if(err < 0) {
			printk("[cm3628][als] (%s): err(0x%x)=%d\n",__FUNCTION__,CM3628_ALS_WL_L, err);
			return -1;
		}else
			printk(DBGMSK_PRX_G3"[cm3628][als] set als_config(0x%x), data=0x%x\n", CM3628_ALS_WL_L, CM3628_REG_INIT);
	}
	if (mode == CM3628_PS_MODE) {
		cm3628_ara_read_reg( cm3628_ara_client );

		//set 0x00
		err = i2c_smbus_write_byte_data(cm3628_ps_conf_client, CM3628_PS_CONF1, PS_SD_OFF );
		if(err < 0) {
			printk("[cm3628][ps] (%s): err(0x%x)=%d\n",__FUNCTION__, CM3628_PS_CONF1, err);
			return -1;
		}else
			printk(DBGMSK_PRX_G4"[cm3628][ps] set ps_config(0x%x), pdata=0x%x\n", CM3628_PS_CONF1, PS_SD_OFF);

		//Set 0x01
		err = i2c_smbus_write_byte_data(cm3628_ps_conf_client, CM3628_PS_THD, g_cm3628_data_ps->g_ps_threshold_hi);
		if(err < 0) {
			printk("[cm3628][ps] (%s): err(0x%x)=%d\n",__FUNCTION__, CM3628_PS_THD, err);
			return -1;
		}else
			printk(DBGMSK_PRX_G4"[cm3628][ps] set ps_config(0x%x), pdata=0x%x\n", CM3628_PS_THD, g_cm3628_data_ps->g_ps_threshold_hi);

		//Set 0x02
		err = i2c_smbus_write_byte_data(cm3628_ps_conf_client, CM3628_PS_CANC, CM3628_REG_INIT);
		if(err < 0) {
			printk("[cm3628][ps] (%s): err(0x%x)=%d\n",__FUNCTION__, CM3628_PS_CANC, err);
			return -1;
		}else
			printk(DBGMSK_PRX_G4"[cm3628][ps] set ps_config(0x%x), pdata=0x%x\n", CM3628_PS_CANC, CM3628_REG_INIT);

		//Set 0x03
		err = i2c_smbus_write_byte_data(cm3628_ps_conf_client, CM3628_PS_CONF2, CM3628_REG_INIT);
		if(err < 0) {
			printk("[cm3628][ps] (%s): err(0x%x)=%d\n",__FUNCTION__, CM3628_PS_CONF2, err);
			return -1;
		}else
			printk(DBGMSK_PRX_G4"[cm3628][ps] set ps_config(0x%x), pdata=0x%x\n", CM3628_PS_CONF2, CM3628_REG_INIT);

// ASUS_BSP +++ Peter_Lu "Set L18 sensor power source from standby mode to normal mode for keep sensor power source stable"
		if (ldoa18_regulator == NULL)	{
			ldoa18_regulator = regulator_get( &cm3628_ps_conf_client->dev, "8941_l18");
			if (IS_ERR(ldoa18_regulator)) {
				printk("[cm3628][ps] Get regulator ldoa18 fail!! \n");
				err = -EIO;
			}	else		{
				printk("[cm3628][ps] Get regulator ldoa18 success  \n");
				err = regulator_set_mode(ldoa18_regulator, REGULATOR_MODE_STANDBY);
				if (err < 0)
					printk("[cm3628][ps] Set regulator ldoa18 fail!! \n");
				else
					printk("[cm3628][ps] Set regulator ldoa18 to Normal mode success \n");
			}
		}else	{
			printk("[cm3628][ps] Regulator ldoa18 is exist \n");
			err = regulator_set_mode(ldoa18_regulator, REGULATOR_MODE_STANDBY);
			if (err < 0)
				printk("[cm3628][ps] Set regulator ldoa18 fail!! \n");
			else
				printk("[cm3628][ps] Set regulator ldoa18 to Normal mode success \n");
		}
// ASUS_BSP --- Peter_Lu
		
	}
	if (mode == CM3628_ARA_MODE) {
		/*cm3628 initialization*/
		cm3628_ara_read_reg( cm3628_ara_client );
		cm3628_ara_read_reg( cm3628_ara_client );
		cm3628_ara_read_reg( cm3628_ara_client );
	}

	printk("[cm3628] cm3628_reg_reset --mode: 0x%x\n", mode);

	return err;
}

static int cm3628_gpio_init(void)
{
	int rc = -EINVAL;
	printk("[cm3628][board]cm3628_gpio_init++\n");

	/* configure Phone Lightsensor interrupt gpio */
	rc = gpio_request(CM3628_GPIO_PROXIMITY_INT, "cm3628-irq");
	if (rc) {
		pr_err("%s: unable to request gpio %d (cm3628-irq)\n",__func__, CM3628_GPIO_PROXIMITY_INT);
		goto err;
	}

	rc = gpio_direction_input(CM3628_GPIO_PROXIMITY_INT);
	if (rc < 0) {
		pr_err("%s: unable to set the direction of gpio %d\n",__func__, CM3628_GPIO_PROXIMITY_INT);
		goto err;
	}

	/* configure Phone Lightsensor power_enable gpio */
/*	
	rc = gpio_request(CM3628_GPIO_PROXIMITY_PWR_EN, "proxm_pwr_en");
	if (rc) {
		pr_err("%s: unable to request gpio %d (proxm_pwr_en)\n",__func__, CM3628_GPIO_PROXIMITY_PWR_EN);
		goto err;
	}

	rc = gpio_direction_output(CM3628_GPIO_PROXIMITY_PWR_EN, 1);
	if (rc < 0) {
		pr_err("%s: unable to set the direction of gpio %d\n",__func__, CM3628_GPIO_PROXIMITY_PWR_EN);
		goto err;
	}
*/
	/* Power on cm3628 */
/*	
	gpio_set_value(CM3628_GPIO_PROXIMITY_PWR_EN, 1);

	printk("[cm3628][board]cm3628_gpio_init--\n");
	return 0;
*/	
err:
	//gpio_free(CM3628_GPIO_PROXIMITY_PWR_EN);
	return rc;
}

static int cm3628_input_init(unsigned int mode)
{
	int ret = 0;
	struct input_dev *input_dev_as = NULL;
	struct input_dev *input_dev_ps = NULL;

	if ( mode == CM3628_ALS_MODE)	{
		input_dev_as = input_allocate_device();
		if (!input_dev_as) {
			ret = -ENOMEM;
			printk("[cm3628]: Failed to allocate input_data device\n");
			goto error_1;
		}

		input_dev_as->name = "ASUS Lightsensor";
		input_dev_as->id.bustype = BUS_I2C;
		input_set_capability(input_dev_as, EV_ABS, ABS_MISC);
		__set_bit(EV_ABS, input_dev_as->evbit);
		__set_bit(ABS_MISC, input_dev_as->absbit);
		input_set_abs_params(input_dev_as, ABS_MISC, 0, 1048576, 0, 0);
		input_set_drvdata(input_dev_as, g_cm3628_data_as);

		ret = input_register_device(input_dev_as);
		if (ret < 0) {
			input_free_device(input_dev_as);
			goto error_1;
		}
		g_cm3628_data_as->input_dev = input_dev_as;


		g_cm3628_data_as->polling = 0;
		g_cm3628_data_as->poll_interval_ms = 100;
		g_cm3628_data_as->event_threshold = 1000;

		/* Register light input event  */
		ret = als_lux_report_event_register(g_cm3628_data_as->input_dev);
	}

	if ( mode == CM3628_PS_MODE)	{
		input_dev_ps = input_allocate_device();
		if (!input_dev_ps) {
			ret = -ENOMEM;
			printk("[cm3628]: Failed to allocate input_data device\n");
			goto error_1;
		}

		input_dev_ps->name = "ASUS Proximitysensor";
		input_dev_ps->id.bustype = BUS_I2C;
		input_set_capability(input_dev_ps, EV_ABS, ABS_DISTANCE);
		__set_bit(EV_ABS, input_dev_ps->evbit);
		__set_bit(ABS_DISTANCE, input_dev_ps->absbit);
		input_set_abs_params(input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);
		input_set_drvdata(input_dev_ps, g_cm3628_data_ps);

		ret = input_register_device(input_dev_ps);
		if (ret < 0) {
			input_free_device(input_dev_ps);
			goto error_1;
		}
		g_cm3628_data_ps->input_dev = input_dev_ps;

		g_cm3628_data_ps->polling = 0;
		g_cm3628_data_ps->poll_interval_ms = 100;
		g_cm3628_data_ps->event_threshold = 1000;
	}

error_1:

	return ret;
}

static int init_cm3628(void)
{
	int ret = 0;
	printk("[cm3628]: init_cm3628 +.\n");

	//ret = cm36283_reset(); // reset both sensor

	/* Set workqueue*/
	cm3628_workqueue					= create_singlethread_workqueue("cm3628_wq");
	cm3628_delay_workqueue			= create_singlethread_workqueue("cm3628_delay_wq");

	INIT_DELAYED_WORK(&cm3628_ISR_work, cm3628_interrupt_handler);
	INIT_DELAYED_WORK(&cm3628_light_interrupt_delay_work, cm3628_light_work);
	INIT_DELAYED_WORK(&cm3628_proximity_interrupt_delay_work, cm3628_Proximity_work);

#ifdef CONFIG_EEPROM_NUVOTON
	INIT_WORK(&cm3628_attached_Pad_work, cm3628_lightsensor_attached_pad);
#endif
	
	wake_lock_init(&proximity_wake_lock, WAKE_LOCK_SUSPEND, "proxm_wake_lock");

	// register irq handler after sensor power on in order to avoid unexpected irq error!
	g_cm3628_device.irq = gpio_to_irq( cm3628_irq_gpio );//(CM3628_GPIO_PROXIMITY_INT);
	printk("[cm3628]Reques EIRQ %d succesd on GPIO:%d\n",g_cm3628_device.irq, cm3628_irq_gpio );
	
	if( g_cm3628_device.irq < 0 )
		printk("[cm3628] gpio_to_irq fail (g_cm3628_device.irq)irq=%d.\n", g_cm3628_device.irq);
	else		{
		printk("[cm3628] (g_cm3628_device.irq) irq=%d.\n", g_cm3628_device.irq);

		ret = request_irq(  g_cm3628_device.irq,
			cm3628_als_proxm_interrupt_handler,
			IRQF_TRIGGER_FALLING  | IRQF_TRIGGER_FALLING,
			"cm3628_INT",
			&cm3628_ara_client->dev );
		
		if (ret < 0)
			printk("[cm3628] (g_cm36283_device.irq) request_irq() error %d.\n",ret);
		else	{
			printk("[cm3628] (g_cm36283_device.irq) request_irq ok.\n");
			disable_irq(g_cm3628_device.irq);
			msleep(5);
			enable_irq(g_cm3628_device.irq);
		}
	}

	// Reset Sensor
	ret = cm3628_reg_reset(CM3628_ALS_MODE);
	if (ret < 0)
		printk("[cm3628] cm3628 ALS reg init error \n");
	
	ret = cm3628_reg_reset(CM3628_PS_MODE);
	if (ret < 0)
		printk("[cm3628] cm3628 PS reg init error \n");
	
	cm3628_reg_reset(CM3628_ARA_MODE);

	printk("[cm3628] init_cm3628 -.\n");
	return ret;
}

static int cm3628_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("[cm3628] ++cm3628_suspend, psensor:%d, als:%d\n",
		g_cm3628_data_ps->Device_ps_switch_on, g_cm3628_data_as->Device_als_switch_on);

	/* For keep Proximity can wake_up system */
	enable_irq_wake(g_cm3628_device.irq);

	/* For make sure Light sensor mush be switch off when system suspend */
	if(g_cm3628_data_as->Device_als_switch_on == 1)	{
		//In case upper layer doesn't switch off ambient before early_suspend.
		printk(DBGMSK_PRX_G2"[cm3628] cm3628_suspend, turn off ambient\n");

		/* Flag for check system still in suspend state */
		g_ambient_suspended = 1;
		cm3628_turn_onoff_als(0);
	}
	printk("[cm3628] --cm3628_suspend\n");

	return 0 ;
}

static int cm3628_resume(struct i2c_client *client)
{
	printk(DBGMSK_PRX_G2"[cm3628]++cm3628_resume, gpio %d : %d\n",
		CM3628_GPIO_PROXIMITY_INT,gpio_get_value(CM3628_GPIO_PROXIMITY_INT) );

	if( !g_bIsP01Attached /*&& g_cm3628_data_ps->Device_ps_switch_on && !gpio_get_value(GPIO_PROXIMITY_INT)*/) {
		g_cm3628_earlysuspend_int = 0;
		if (g_cm3628_data_ps->proxm_detect_object_state == 1)
			wake_lock_timeout(&proximity_wake_lock, 1 * HZ);
	}

	printk("[cm3628][als] resume: g_ambient_suspended = %d \n", g_ambient_suspended);
	if(g_ambient_suspended==1) {
		if ( !g_bIsP01Attached )
			cm3628_turn_onoff_als(1);	// Light sensor is always on.
		
		g_ambient_suspended = 0;
		printk("[cm3628][als] resume: apply ALS interrupt mode\n");
	}

	disable_irq_wake(g_cm3628_device.irq);

	g_cm3628_earlysuspend_int = 0;

	printk(DBGMSK_PRX_G2"[cm3628]--cm3628_resume\n");

	return 0 ;
}

static int cm3628_remove(struct i2c_client *client)
{
	return 0;
}

/*.........................................Probe Ambient light sensoer.........................................*/
static const struct i2c_device_id cm3628_als_conf_id[] = {
	{ "cm3628_als_conf", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, cm3628_als_conf_id);

static struct of_device_id cm3628_als_conf_match_table[] = {
	{ .compatible = "capella,cm3628_als_conf",},
	{},
};

static struct i2c_driver cm3628_als_conf_driver = {
	.driver = {
		.name	= "cm3628_als_conf",
		.owner	= THIS_MODULE,
		.of_match_table = cm3628_als_conf_match_table,
	},
	.probe	= cm3628_als_conf_probe,
	.id_table = cm3628_als_conf_id,
};

static int cm3628_als_conf_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	
	printk("[cm3628] cm3628_als_conf_probe +.\n");

	if (client == NULL) {
		printk("[cm3628] Client is NUll.\n");
		ret =  -EFAULT;
		goto cm3628_als_probe_err;
	}

	if (!(g_cm3628_data_as = kmalloc(sizeof(struct ASUS_light_sensor_data), GFP_KERNEL))) {
		ret = -ENOMEM;
		goto cm3628_als_probe_err;
	}
	memset(g_cm3628_data_as, 0, sizeof(struct ASUS_light_sensor_data));

	/* Reset ASUS_light_sensor_data*/
	g_cm3628_data_as->g_als_threshold_hi = 0;
	g_cm3628_data_as->g_als_threshold_lo = 0;
	g_cm3628_data_as->last_light_lux = 0;
	g_cm3628_data_as->light_adc = 0;
	g_cm3628_data_as->light_k_adc = 0;
	g_cm3628_data_as->Device_als_switch_on = 0;
	g_cm3628_data_as->HAL_als_switch_on= 0;

	cm3628_als_conf_client = client;
	i2c_set_clientdata(cm3628_als_conf_client, g_cm3628_data_as);
	cm3628_als_conf_client->driver = &cm3628_als_conf_driver;
	cm3628_als_conf_client->flags = 1;
	strlcpy(cm3628_als_conf_client->name, CM3628_ALS_CONF_NAME, I2C_NAME_SIZE);

	printk("[cm3628] Register input device...\n");
	if( cm3628_input_init(CM3628_ALS_MODE) != 0 )
		goto cm3628_als_probe_err;

	ret = sysfs_create_group(&client->dev.kobj, &cm3628_attr_group);
	if (ret)
		goto cm3628_als_probe_err;

	ret = proximity_dev_register(&cm3628_ambientDev);
	if (ret)
		printk("[cm3628] ambientdl create sysfile fail.\n");
	
	printk("[cm3628] cm3628_als_conf_probe -.\n");
	return 0;

cm3628_als_probe_err:
	printk("[cm3628] cm3628_als_conf_probe - (error).\n");
    //if (g_cm3628_data_as != NULL) {
    //  kfree(g_cm3628_data_as);
    //}

	return ret;
}

/*.........................................Probe Proximity sensoer.........................................*/
static const struct i2c_device_id cm3628_ps_conf_id[] = {
	{ "cm3628_ps_conf", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, cm3628_ps_conf_id);

static struct of_device_id cm3628_ps_conf_match_table[] = {
	{ .compatible = "capella,cm3628_ps_conf",},
	{},
};

static struct i2c_driver cm3628_ps_conf_driver = {
	.driver = {
		.name	= "cm3628_ps_conf",
		.owner	= THIS_MODULE,
		.of_match_table = cm3628_ps_conf_match_table,
	},
	.probe	= cm3628_ps_conf_probe,
	.id_table = cm3628_ps_conf_id,
};

static int cm3628_ps_conf_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	printk("[cm3628] cm3628_ps_conf_probe +.\n");

	if (client == NULL) {
		printk("[cm3628] Client is NUll.\n");
		ret =  -EFAULT;
		goto cm3628_ps_conf_probe_err;
	}

	if (!(g_cm3628_data_ps = kmalloc(sizeof(struct ASUS_proximity_sensor_data), GFP_KERNEL))) {
		ret = -ENOMEM;
		goto cm3628_ps_conf_probe_err;
	}
	memset(g_cm3628_data_ps, 0, sizeof(struct ASUS_proximity_sensor_data));

	g_cm3628_data_ps->g_ps_threshold_lo = DEFAULT_PS_THRESHOLD_lo;
	g_cm3628_data_ps->g_ps_threshold_hi = DEFAULT_PS_THRESHOLD_hi;

	cm3628_ps_conf_client = client;
	i2c_set_clientdata(cm3628_ps_conf_client, g_cm3628_data_ps);
	cm3628_ps_conf_client->driver = &cm3628_ps_conf_driver;
	cm3628_ps_conf_client->flags = 1;
	strlcpy(cm3628_ps_conf_client->name, CM3628_PS_CONF_NAME, I2C_NAME_SIZE);

	printk("[cm3628] Register input device...\n");
	if( cm3628_input_init(CM3628_PS_MODE) != 0 )
		goto cm3628_ps_conf_probe_err;

	ret = proximity_dev_register(&cm3628_proxmDev);
	if (ret)
		printk("[cm3628] proxmdl create sysfile fail.\n");

	printk("[cm3628] cm3628_ps_conf_probe -.\n");
	return 0;

cm3628_ps_conf_probe_err:
	printk("[cm3628] cm3628_ps_conf_probe - (error).\n");
    //if (g_cm3628_data_ps != NULL) {
    //  kfree(g_cm3628_data_ps);
    //}

	return ret;
}

/*.......................................Probe Alert Response interrupt.........................................*/
static const struct i2c_device_id cm3628_ara_id[] = {
	{ "cm3628_ara", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, cm3628_ara_id);

static struct of_device_id cm3628_ara_match_table[] = {
	{ .compatible = "capella,cm3628_ara",},
	{},
};

static struct i2c_driver cm3628_ara_driver = {
	.driver = {
		.name	= "cm3628_ara",
		.owner	= THIS_MODULE,
		.of_match_table = cm3628_ara_match_table,
	},
	.suspend = cm3628_suspend,
	.resume	= cm3628_resume,
	.probe	= cm3628_ara_probe,
	.remove	= cm3628_remove,
	.id_table = cm3628_ara_id,
};

static int cm3628_ara_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int error = 0;

	printk("[cm3628] cm3628_ara_probe +.\n");

	if (client == NULL) {
		printk("[cm3628] Client is NUll.\n");
		error =  -EFAULT;
		goto cm3628_ara_probe_err;
	}
	
	cm3628_ara_client = client;
	cm3628_ara_client->driver = &cm3628_ara_driver;
	cm3628_ara_client->flags = 1;
	strlcpy(cm3628_ara_client->name, CM3628_ARA_NAME, I2C_NAME_SIZE);
	
	/* Get data that is defined in board specific code. */
	cm3628_irq_gpio = of_get_named_gpio_flags( 
    			client->dev.of_node, "cm3628,irq-gpio",0,NULL);

	printk("[cm3628]calling init_platform_hw\n");
	error = cm3628_gpio_init();
	if (error) {
		printk("hw init failed");
		goto cm3628_ara_probe_err;
	}

	error = init_cm3628();
	if( error < 0 )  {
		printk("[cm3628] init_cm3628() error.\n");
		goto cm3628_ara_probe_err;
	}

	printk("[cm3628] cm3628_ara_probe -.\n");

	return 0;	
	
cm3628_ara_probe_err:
	printk("[cm3628] cm3628_ara_probe - (error).\n");

	return error;	
}

/////////////////////////////////////////////////////////////////////////////////
// Pad mode related
//
#ifdef CONFIG_EEPROM_NUVOTON
static void cm3628_lightsensor_attached_pad(struct work_struct *work)
{
	printk(DBGMSK_PRX_G2"[cm3628_als] lightsensor_attached_pad()++\n");

	//if HAL already turned on als, we switch to Pad light sensor
	if(g_cm3628_data_ps->Device_ps_switch_on)
		cm3628_turn_onoff_proxm(0);

	if(g_HAL_als_switch_on) 
	{
		printk(DBGMSK_PRX_G2"[cm3628_als] lightsensor_attached_pad, checking if pad is attached\n");
		printk(DBGMSK_PRX_G2"[cm3628_als] lightsensor_attached_pad, attached! switch to al3010 : %d lux\n",
			g_cm3628_data_as->light_lux);

		/*shut down cm3628_als*/
		cm3628_turn_onoff_als(0);
	}

	g_bIsP01Attached = true;
	printk(DBGMSK_PRX_G2"[cm3628_als] lightsensor_attached_pad()--\n");

	return;
}

int cm3628_lightsensor_detached_pad(void)
{
	printk(DBGMSK_PRX_G2"[cm3628_als] Cm36283_detached_pad()++\n");

	//if HAL still turned on the als, we switch back to phone light sensor
	if(g_HAL_als_switch_on)	{
		printk(DBGMSK_PRX_G2"[cm3628_als] lightsensor_detached_pad(), switch back to cm36283 : %d lux\n",
			g_cm3628_data_as->light_lux);

		//turn on cm36283_als
		cm3628_turn_onoff_als(1);
	}
	else
		printk(DBGMSK_PRX_G2"[cm3628_als] lightsensor_detached_pad(), als is turned off\n");

	//if phone call or ps still on, we switch back to cm36283
	if (g_cm3628_data_ps->Device_ps_switch_on)	{
		printk(DBGMSK_PRX_G2"[cm3628_proxm] ps_detached_pad(), phone call still on\n");
		cm3628_turn_onoff_proxm(1);
		queue_delayed_work(cm3628_delay_workqueue, &cm3628_proximity_interrupt_delay_work, 0);
	}

	else if ( g_cm3628_data_ps->pad_proxm_switch_on && !g_cm3628_data_ps->Device_ps_switch_on)	{
		printk("[cm3628_proxm] ps_detached_pad(), phone call on in Pad\n");
		cm3628_turn_onoff_proxm(1);
		g_cm3628_data_ps->Device_ps_switch_on = 1;
		queue_delayed_work(cm3628_delay_workqueue, &cm3628_proximity_interrupt_delay_work, 0);
	}
	else
		printk(DBGMSK_PRX_G2"[cm3628_als] ps is turned off\n");

	g_bIsP01Attached = false;
	printk(DBGMSK_PRX_G2"[cm3628_als] Cm3628_detached_pad()--\n");

	return 0;
}

static int cm3628_lightsensor_mp_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	switch (event) {
		case P01_ADD:
			printk("[cm3628_als][MicroP] Pad_ADD \r\n");
			//Work to cm36283_lightsensor_attached_pad();
			queue_work(cm3628_workqueue, &cm3628_attached_Pad_work);
			return NOTIFY_DONE;

		case P01_REMOVE:
			printk("[cm3628_als][MicroP] Pad_REMOVE \r\n");
			cm3628_lightsensor_detached_pad();
			return NOTIFY_DONE;
			
		default:
			return NOTIFY_DONE;
	}
}

static struct notifier_block cm3628_lightsensor_mp_notifier = {
        .notifier_call = cm3628_lightsensor_mp_event,
        .priority = CM36283_LIGHTSENSOR_MP_NOTIFY,
};
#endif

static int cm3628_platform_probe( struct platform_device *pdev )
{
	int err = 0;
	
	printk("[cm3628] cm3628_platform_probe ++ \n");

	err = i2c_add_driver(&cm3628_als_conf_driver);
	if ( err != 0 )
		printk("[cm3628] i2c_add_driver fail: cm3628_als_conf_driver, Error : %d\n",err);
	err = i2c_add_driver(&cm3628_ps_conf_driver);
	if ( err != 0 )
		printk("[cm3628] i2c_add_driver fail: cm3628_ps_conf_driver, Error : %d\n",err);
	err = i2c_add_driver(&cm3628_ara_driver);
	if ( err != 0 )
		printk("[cm3628] i2c_add_driver fail: cm3628_ara_driver, Error : %d\n",err);


	printk("[cm3628] cm3628_platform_probe -- \n");
	return 0;

}

static int __devexit cm3628_platform_remove( struct platform_device *pdev )
{
	i2c_del_driver(&cm3628_als_conf_driver);
	i2c_del_driver(&cm3628_ps_conf_driver);
	i2c_del_driver(&cm3628_ara_driver);

	proximity_dev_unregister(&cm3628_proxmDev);
	proximity_dev_unregister(&cm3628_ambientDev);

	destroy_workqueue(cm3628_workqueue);
	
	return 0;
}

static int cm3628_platform_suspend_noirq( struct device *dev )
{
	printk("[cm36283][suspend_noirq] g_earlysuspend_int = %d\n",  g_cm3628_earlysuspend_int);
	if(g_cm3628_earlysuspend_int == 1) {
		g_cm3628_earlysuspend_int = 0;
		return -EBUSY;
	}
        return 0;
}

static int cm3628_platform_resume_noirq( struct device *dev )
{
        return 0;
}

MODULE_DEVICE_TABLE(of, gpio_keys_of_match);

static const struct dev_pm_ops cm3628_pm_ops = {
	.suspend_noirq  = cm3628_platform_suspend_noirq,
	.resume_noirq = cm3628_platform_resume_noirq,
};

static struct platform_driver  cm3628_platform_driver = {
	.probe 	= cm3628_platform_probe,
	.remove   = cm3628_platform_remove,

	.driver = {
		.name = "cm3628",
		.owner = THIS_MODULE,
		.pm = &cm3628_pm_ops,
	},
};



static int __init cm3628_init(void)
{
	int err = 0;

	if( g_ASUS_hwID >= A91_SR1 && g_ASUS_hwID < A91_SR5 ) {

		printk("[cm3628] cm3628_platform_init +.\n");

		err = platform_driver_register(&cm3628_platform_driver);
		if ( err != 0 )
			printk("[cm3628] platform_driver_register fail, Error : %d\n",err);

#ifdef CONFIG_EEPROM_NUVOTON
		register_microp_notifier(&cm3628_lightsensor_mp_notifier);
		notify_register_microp_notifier(&cm3628_lightsensor_mp_notifier, "cm36283");
#endif

		printk("[cm3628] cm3628_platform_init -.\n");
	}
	return err;
}

static void __exit cm3628_exit(void)
{
	if(g_ASUS_hwID >= A91_SR1)	{
		printk(DBGMSK_PRX_G2"[cm3628] cm3628_platform_exit +.\n");
		platform_driver_unregister(&cm3628_platform_driver);
		printk(DBGMSK_PRX_G2"[cm3628] cm3628_platform_exit -.\n");
	}
}

module_init(cm3628_init);
module_exit(cm3628_exit);

MODULE_AUTHOR("ASUS");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAPELLA CM3628 ALS/Proximity sensor");
