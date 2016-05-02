/* stk3210.c - stk3210 proximity/light sensor driver
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

#include "linux/input/proximity_class.h"
#include "linux/input/STKdriver.h"
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <asm/intel-mid.h>
#ifdef CONFIG_EEPROM_PADSTATION
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>
#endif

#define STK3210_PS_CONF_NAME			"stk3210_ps_conf"
#define STK3210_ALS_CONF_NAME			"stk3210_als_conf"
#define STK3210_ARA_NAME				"stk3210_ara"

#define PS_MODE							0x00
#define ALS_MODE						0x01

/* For sensor reset & HW Reset issue */
#define STK3210_REG_INIT				0x00
static int asus_ir_sensor_irq_gpio;

#define STK_GPIO_PROXIMITY_INT			77

struct i2c_client *stk3210_client;

static struct workqueue_struct *stk3210_workqueue;
static struct work_struct stk3210_ISR_work;

static struct workqueue_struct *stk3210_delay_workqueue;
static struct delayed_work stk3210_light_proximity_delay_work;
/*
static struct delayed_work stk3210_proximity_interrupt_delay_work;
*/
struct ASUS_light_sensor_data {
	struct i2c_client client ;
	struct input_dev *input_dev;				/* Pointer to input device */

	int g_als_threshold_lo;					/* Lightsensor setting low threshold(adc) */
	int g_als_threshold_hi;					/* Lightsensor setting high threshold(adc) */
	int g_als_200lux_calvalue;					/* Lightsensor 200lux calibration value(adc) */
	int g_als_1000lux_calvalue;					/* Lightsensor 1000lux calibration value(adc) */
	int g_als_shift_calvalue;					/* Lightsensor Shift calibration value */

	int light_adc;								/* Original light adc value */
	int light_k_adc;							/* The light adc value after calibration */
	int last_light_lux;							/* light Lux after filter */
	int light_lux;								/* Final light Lux */

	bool HAL_als_switch_on;					/* this var. means if HAL is turning on als or not */
	bool Device_als_switch_on;				/* this var. means if als hw is turn on or not */
	bool als_in_pad_mode;					/* For phone plug in pad issue */

	unsigned int poll_interval_ms;				/* I2C polling period */
	unsigned int event_threshold;				/* Change reqd to gen event */
	unsigned int open_count;					/* Reference count */
	char	polling;								/* Polling flag */
};

struct ASUS_proximity_sensor_data {
	struct i2c_client client;
	struct input_dev *input_dev;				/* Pointer to input device */

	int g_ps_threshold_lo;					/* Proximitysensor setting low threshold(adc) */
	int g_ps_threshold_hi;					/* Proximitysensor setting high threshold(adc) */
	bool proxm_detect_object_state;			/* For check proximity near/far state */

	bool HAL_ps_switch_on;					/* this var. means if HAL is turning on ps or not */
	bool Device_ps_switch_on;				/* this var. means is turning on ps or not */
	bool ps_in_pad_mode;					/* For phone plug in pad issue */
	bool is_irq_enable;						/* For check irq disable/enable state */
	bool calibration_new_version;				/* For check proximity calibraition version( MP/PR or recalibration device ) */
	int irq_count;
	struct mutex lock;							/* For muxtex lock */

	unsigned int poll_interval_ms;				/* I2C polling period */
	unsigned int event_threshold;				/* Change reqd to gen event */
	unsigned int open_count;					/* Reference count */
	char polling;								/* Polling flag */
};

struct ASUS_light_sensor_data			*g_stk3210_data_as;
struct ASUS_proximity_sensor_data		*g_stk3210_data_ps;

static uint8_t org_flag_reg;

static int g_proxm_dbg;		/* Add for debug only */
static int g_ambient_dbg;		/* Add for debug only */
#define PS_retry_time 8		/* Add for debug only */

/************************************************************************************
 *---proximity sensor setting part---
 *
 *wake_lock for Proximity, make sure Proximity finish all of the work before suspend
 */
#include <linux/wait.h>
#include <linux/wakelock.h>
static struct wake_lock proximity_wake_lock;

/*
 * Proximity command CONF1
 */
static u8 PS_IT			 = 0x05;	/* PS refresh time setting					, bit 3~0 ( x16 2.96ms ) */
static u8 PS_GAIN			 = 0x02;	/* PS gain setting for PS sensitivity			, bit 5,4 ( X16 ) */
static u8 PS_IT_MR		 = 0x01;	/* PS refresh time setting					, bit 3~0 ( x1 0.391ms ) */
static u8 PS_GAIN_MR		 = 0x03;	/* PS gain setting for PS sensitivity			, bit 5,4 ( X64 ) */
static u8 PS_PERS			 = 0x00;	/* PS interrupt persistence setting				, bit 7,6 ( 1 times ) */
/* static u8 PS_PERS			 = 0x01;  PS interrupt persistence setting				, bit 7,6 ( 4 times ) */

static u8 IRLED_DT			 = 0x10; /* PS IRLED on/off duty ratio setting			, bit 5~0 ( 16/64 x PS_IT ) */
static u8 IRLED_IRDR		 = 0x01; /* PS IRLED driving current setting				, bit 7,6  ( 25mA ) */
static u8 IRLED_DT_MR		 = 0x3F; /* PS IRLED on/off duty ratio setting			, bit 5~0 ( 64/64 x PS_IT ) */
static u8 IRLED_IRDR_MR	 = 0x03; /* PS IRLED driving current setting				, bit 7,6  ( 100mA ) */


static u8 PS_INT_EN_OFF	 = 0x00; /* PS interrupt disable						, bit 2~0   ( disable ) */
static u8 PS_INT_EN_ON	 = 0x01; /* PS interrupt recommend mode enable		, bit 2~0   ( enable ) */
static u8 PS_INT_EN_ON_2	 = 0x03; /* PS interrupt out-of threshold mode enable	, bit 2~0   ( enable ) */

static u8 PS_SD_OFF		 = 0xFE; /* PS shut down setting						, bit 0   ( disable ) */
static u8 PS_SD_ON		 = 0x01; /* PS shut down setting						, bit 0   ( enable ) */

/*
 * Proximity command CONF2
 */
static u8 PS_MS		 	= 0x00;	/* PS interrupt or logic output mode			, bit 4       ( interrupt ) */
static u8 PS_DIR_INT		= 0x00;	/* PS directional interrupt function 			, bit 2	( disable ) */
static u8 PS_SMART_PERS	= 0x01;	/* PS Smart persistence function				, bit 1	( enable ) */
static u8 PS_HYS			= 0x01;	/* PS detect threshold hysteresis window		, bit 0	( 2 steps ) */

/************************************************************************************
 *---light sensor setting part---
 */

/*
 * ALS command CONF1
 */
static u8 ALS_IT			 = 0x0A;	/* ALs intrgration time setting				, bit 3~0 ( 94.84ms ) */
static u8 ALS_GAIN		 = 0x02;	/* ALs gain setting							, bit 5,4 ( X16 ) */
static u8 ALS_PERS		 = 0x00; /* ALS interrupt persistence setting			, bit 7,6 ( 1 times ) */

static u8 ALS_INT_EN_OFF	 = 0x00; /* ALS interrupt enable/disable				, bit 3   ( disable ) */
static u8 ALS_INT_EN_ON	 = 0x01; /* ALS interrupt enable/disable				, bit 3   ( enable ) */
static u8 INT_CTRL	 	 = 0x7F; /* Trigger interrupt by PS "or" ALS			, bit 7   ( enable ALS/PS interrupt trigger ) */

static u8 ALS_SD_OFF		 = 0xFD; /* ALS shut down setting					, bit 1   ( disable ) */
static u8 ALS_SD_ON		 = 0x02; /* ALS shut down setting					, bit 1   ( enable ) */

static u8 ALS_RESET		 = 0x01; /* Reset ALS reading sequence				, bit 6       ( enable ) */
static u8 ALS_AV			 = 0x02; /* ALS average number setting				, bit 5,4    ( 4 times ) */
static u8 ALS_HS			 = 0x01; /* ALS normal/high sensitivity mode selection	, bit 3       ( normal ) */
static u8 ALS_THD			 = 0x02; /* ALS variance mode thershold range setting	, bit 0,1,2  ( +/- 32 steps ) */

/************************************************************************************
 *---Function part---
 */
static int stk3210_proxmdev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int stk3210_proxmdev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int stk3210_turn_onoff_proxm(bool bOn);
static int get_ps_adc_from_stk3210(bool trigger_by_interrupt);
static int stk3210_ambientDev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int stk3210_ambientDev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int stk3210_turn_onoff_als(bool bOn);
static int get_als_adc_from_stk3210(void);
static int32_t stk3x1x_set_flag(uint8_t org_flag_reg, uint8_t clr);
static irqreturn_t stk3210_irq_handler(int irq, void *dev_id);
static void stk3210_Proximity_work(void);
static void stk3210_light_work(void);

/************************************************************************************
 *---ATD test & calibration part---
 */
static int32_t stk3x1x_software_reset(void)
{
	int32_t r;
	uint8_t w_reg;

	w_reg = 0x7F;
	r = i2c_smbus_write_byte_data(stk3210_client, STK_WAIT_REG, w_reg);
	if (r < 0)	{
		printk("[stk3210] %s: write i2c error, ret=%d\n", __func__, r);
		return r;
	}
	r = i2c_smbus_read_byte_data(stk3210_client, STK_WAIT_REG);
	if (w_reg != r)
		printk("[stk3210] %s: read-back value is not the same\n", __func__);

	r = i2c_smbus_write_byte_data(stk3210_client, STK_SW_RESET_REG, 0);
	if (r < 0)	{
		printk("[stk3210] %s: read error after reset\n", __func__);
		return r;
	}
	usleep_range(1000, 5000);
	return 0;
}

static int proximity_als_turn_on(int bOn)
{
	int err = 0;
	/* 0x1: light sensor is ok, 0x2: proximity is ok, 0x3: both are ok, 0x0: both failed */
	int status = 0;
	printk("[ALS/PS] proximity and light sensor test, turn_on:%d\n", bOn);

	err = stk3210_turn_onoff_als(bOn);
	if (err < 0)
		printk("[ALS/PS] (%s): turn %s light sensor error!\n", __FUNCTION__, bOn ? "on" : "off");
	else	{
		printk("[ALS/PS] (%s): light sensor %s OK!\n", __FUNCTION__, bOn ? "on" : "off");
		/* als ok */
		status |= 0x1;
	}

	err = stk3210_turn_onoff_proxm(bOn);
	if (err < 0)
		printk("[ALS/PS] (%s): turn %s proximity sensor error!\n", __FUNCTION__, bOn ? "on" : "off");
	else {
		printk("[ALS/PS] (%s): proximity sensor %s OK!\n", __FUNCTION__, bOn ? "on" : "off");
		/* ps ok */
		status |= 0x02;
	}

	printk("[ALS/PS]turn %s, status:0x%x (bitwise)\n", bOn ? "on" : "off", status);

	return status;
}

static int atd_read_P_L_sensor_adc(int *adc)
{
	int status = 0;
	int idx = 0;

	printk("[ALS/PS][atd]readadc: trying to turn on lsensor\n");
	status = proximity_als_turn_on(1);

	if (0 == status) {
		printk("[ALS/PS][atd]readadc: lsensor is not on\n");
		return status;
	}

	*adc = 0;

	for (idx = 0; idx < 5; idx++)	{
		*adc = get_ps_adc_from_stk3210(false);
		msleep(100);
	}

	printk("[ALS/PS][atd]readadc steps=%d \n", *adc);
	proximity_als_turn_on(0);

	return status;
}

static int atd_write_status_and_adc_2(int *als_sts, int *als_adc, int *ps_sts)
{
	int adc = 0;
	/* 0x1: light sensor is ok, 0x2: proximity is ok, 0x3: both are ok, 0x0: both failed */
	int status = 0;

	printk("[ALS/PS][atd]writests_2: started\n");
	status = atd_read_P_L_sensor_adc(&adc);

	if (status & 0x01)	{
		*als_sts = 1;
		*als_adc = adc;
	} else		{
		*als_sts = 0;
		*als_adc = 0;
	}

	if (status & 0x02)	{
		*ps_sts = 1;
	} else		{
		*ps_sts = 0;
	}

	printk("[ALS/PS][atd]writests_2: get adc, als_sts:%d, als_adc:%d, ps_sts:%d\n",
		*als_sts, *als_adc, *ps_sts);

	return status;
}

 static int stk3210_read_reg(u8 reg)
{
	int err = 0;
	uint8_t data = 0;

	struct i2c_msg msg[] = {
		{
			.addr = stk3210_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = stk3210_client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &data,
		}
	};

	if (!stk3210_client)
		return -ENODEV;

	memset(&data, 0, sizeof(data));

	err = i2c_transfer(stk3210_client->adapter, msg, ARRAY_SIZE(msg));

	if (err != ARRAY_SIZE(msg))
		printk("[stk3210] stk3210_read_reg err %d\n", err);

	/* printk("[stk3210] stk3210_read_reg : %d\n", data); */

	return (int)data;
}

void stk3210_Show_Allreg(void)
{
	uint8_t reg[23] = {0};
	int i = 0;
	for (i = 0; i < 23; i++)	{
		reg[i] = i2c_smbus_read_byte_data(stk3210_client, i);
		printk("[stk3210] STK_Show_Allreg STK_ReadReg(0x%X) = 0x%x\n", i, reg[i]);
	}
}

/* Defult shift = 40 */
static u32 g_stk3210_light_shift_calibration;
/* Defult gain = 38 */
static int g_stk3210_light_gain_calibration;
/* Defult accuracy = 100000 */
static int a_als_calibration_accuracy;

#ifdef ASUS_FACTORY_BUILD
static struct write_calvalue {
    struct work_struct write_200lux_calvalue_work;
    int calvalue;
} *stk3210_write_200lux_calvalue;

static struct write_shift {
    struct work_struct write_1000lux_calvalue_work;
    int calvalue;
} *stk3210_write_1000lux_calvalue;

static struct write_prox_hi {
    struct work_struct write_prox_hi_work;
    int calvalue;
} *stk3210_write_prox_hi;

static struct write_prox_lo {
    struct work_struct write_prox_lo_work;
    int calvalue;
} *stk3210_write_prox_lo;

static int a_als_calibration_lux;
static int a_als_low_calibration_adc;
static int a_als_high_calibration_adc;
static int a_ps_hi_calibration_adc;
static int a_ps_lo_calibration_adc;
#endif

/* Support ATD light sensor calibration process */
static bool read_lightsensor_200lux_calibrationvalue(void)
{
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char readstr[16];
	int ori_val = 0, readlen = 0;
	mm_segment_t old_fs;

	printk("[stk3210] ++read_lsensor_calvalue open\n");

	fp = filp_open(LSENSOR_CALIBRATION_200LUX, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		printk("[stk3210] read_lsensor_calvalue open (%s) fail\n", LSENSOR_CALIBRATION_200LUX);
		return false;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, readstr, 16, &pos_lsts);
		readstr[readlen] = '\0';

		printk("[stk3210] strlen:%s(%d)\n", readstr, strlen(readstr));
	} else
		printk("[stk3210] read_lsensor_calvalue, f_op=NULL or op->read=NULL\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(readstr, "%d", &ori_val);

	/* Set the 200lux calibration value */
	g_stk3210_data_as->g_als_200lux_calvalue = ori_val;

	printk("[stk3210] read_lsensor_200lux_calvalue: Ori: %d, Cal: %d\n", ori_val,
				g_stk3210_data_as->g_als_200lux_calvalue);

/*
	printk("[stk3210] read_lsensor_calvalue: Ori: %d, Cal: %d.%d\n", ori_val,
				g_stk3210_light_gain_calibration/a_als_calibration_accuracy,
				g_stk3210_light_gain_calibration%a_als_calibration_accuracy);
*/
	printk("[stk3210] --read_lsensor_calvalue open\n");
	return true;
}

static bool read_lightsensor_1000lux_calibrationvalue(void)
{
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char readstr[16];
	int ori_val = 0, readlen = 0;
	mm_segment_t old_fs;

	printk("[stk3210] ++read_lsensor_shift open\n");

	fp = filp_open(LSENSOR_CALIBRATION_1000LUX, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		printk("[stk3210] read_lsensor_shift open (%s) fail\n", LSENSOR_CALIBRATION_1000LUX);
		return false;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, readstr, 16, &pos_lsts);
		readstr[readlen] = '\0';

		printk("[stk3210] strlen:%s(%d)\n", readstr, strlen(readstr));
	} else
		printk("[stk3210] read_lsensor_shift, f_op=NULL or op->read=NULL\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(readstr, "%d", &ori_val);

	/* Set the 1000lux calibration value */
	g_stk3210_data_as->g_als_1000lux_calvalue = ori_val;

	printk("[stk3210] read_lsensor_1000lux_calvalue: Ori: %d, Cal: %d\n", ori_val,
				g_stk3210_data_as->g_als_1000lux_calvalue);
/*
	//limit the calibration value range
	g_stk3210_light_shift_calibration = ori_val;
	printk("[stk3210] read_lsensor_shift: Ori: %d, Cal: %d\n", ori_val, g_stk3210_light_shift_calibration);
*/
	printk("[stk3210] --read_lsensor_calvalue open\n");
	return true;
}

#ifdef ASUS_FACTORY_BUILD
static void write_lightsensor_200Lux_calibrationvalue_work(struct work_struct *work)
{
	struct file *fp = NULL;
	struct write_calvalue *this = NULL;
	loff_t pos_lsts = 0;
	char writestr[8];
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	this = container_of(work, struct write_calvalue, write_200lux_calvalue_work);

	fp = filp_open(LSENSOR_CALIBRATION_200LUX, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		printk("[stk3210] write_lsensor_calvalue open (%s) fail\n", LSENSOR_CALIBRATION_200LUX);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk("[stk3210] write_lsensor_calvalue = %d[%s(%d)]\n",
		this->calvalue, writestr, strlen(writestr));

	if (fp->f_op != NULL && fp->f_op->write != NULL)	{
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	} else
		printk("[stk3210] write_lsensor_calvalue fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}

static void write_lightsensor_1000Lux_calibrationvalue_work(struct work_struct *work)
{
	struct file *fp = NULL;
	struct write_shift *this = NULL;
	loff_t pos_lsts = 0;
	char writestr[8];
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	this = container_of(work, struct write_shift, write_1000lux_calvalue_work);


	fp = filp_open(LSENSOR_CALIBRATION_1000LUX, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		printk("[stk3210] write_lsensor_shift open (%s) fail\n", LSENSOR_CALIBRATION_1000LUX);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk("[stk3210] write_lsensor_shift = %d[%s(%d)]\n",
		this->calvalue, writestr, strlen(writestr));

	if (fp->f_op != NULL && fp->f_op->write != NULL)	{
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	} else
		printk("[stk3210] write_lsensor_shift fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}

/*Light Sensor Calibration */
static int stk3210_show_calibration_200(struct device *dev, struct device_attribute *attr, char *buf)
{
	read_lightsensor_200lux_calibrationvalue();
	read_lightsensor_1000lux_calibrationvalue();

	printk("[stk3210] Show_200lux_calibration_value : %d\n",
				g_stk3210_data_as->g_als_200lux_calvalue);

	printk("[stk3210] Show_org_gait_calibration: %d.%d",
					g_stk3210_light_gain_calibration/a_als_calibration_accuracy,
					g_stk3210_light_gain_calibration%a_als_calibration_accuracy);

	g_stk3210_light_gain_calibration =
		(a_als_calibration_lux*a_als_calibration_accuracy) /
					(g_stk3210_data_as->g_als_1000lux_calvalue - g_stk3210_data_as->g_als_200lux_calvalue);

	printk(", cal : %d.%d \n",
					g_stk3210_light_gain_calibration/a_als_calibration_accuracy,
					g_stk3210_light_gain_calibration%a_als_calibration_accuracy);

	return sprintf(buf, "%d.%d\n"
				, g_stk3210_light_gain_calibration/a_als_calibration_accuracy
				, g_stk3210_light_gain_calibration%a_als_calibration_accuracy);
}

static ssize_t stk3210_store_calibration_200(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if ((kstrtoul(buf, 10, &val) < 0))
		return -EINVAL;

	/* Get low brightness adc */
	a_als_low_calibration_adc = (int)val;

	printk("[stk3210] Get low calibration adc value : %d\n", a_als_low_calibration_adc);

	return count;
}

static DEVICE_ATTR(calibration_200, S_IRWXU | S_IRWXG | S_IRWXO,
		   stk3210_show_calibration_200, stk3210_store_calibration_200);

static int stk3210_show_calibration_1000(struct device *dev, struct device_attribute *attr, char *buf)
{
	read_lightsensor_200lux_calibrationvalue();
	read_lightsensor_1000lux_calibrationvalue();

	printk("[stk3210] Show_1000lux_calibration_value: %d\n", g_stk3210_data_as->g_als_1000lux_calvalue);

	printk("[stk3210] Show_org_shift_calibration: %d", g_stk3210_light_shift_calibration);

	g_stk3210_light_shift_calibration =
		1000 - (800*g_stk3210_data_as->g_als_1000lux_calvalue/
			(g_stk3210_data_as->g_als_1000lux_calvalue - g_stk3210_data_as->g_als_200lux_calvalue));

	printk(", cal : %d\n", g_stk3210_light_shift_calibration);

	return sprintf(buf, "%d\n", g_stk3210_light_shift_calibration);
}

static ssize_t stk3210_store_calibration_1000(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if ((kstrtoul(buf, 10, &val) < 0))
		return -EINVAL;

	/*Get low brightness adc*/
	a_als_high_calibration_adc = (int)val;

	printk("[stk3210] Get High calibration adc value : %d\n", a_als_high_calibration_adc);

	/*Calibration operation*/
	g_stk3210_light_gain_calibration =
		(a_als_calibration_lux*a_als_calibration_accuracy) /
					(a_als_high_calibration_adc - a_als_low_calibration_adc);

	g_stk3210_light_shift_calibration =
		1000 - (a_als_high_calibration_adc*g_stk3210_light_gain_calibration / a_als_calibration_accuracy);

	/*Write Calibration value*/
	stk3210_write_200lux_calvalue = kmalloc(sizeof(struct write_calvalue), GFP_KERNEL);

	INIT_WORK(&stk3210_write_200lux_calvalue->write_200lux_calvalue_work, write_lightsensor_200Lux_calibrationvalue_work);

	stk3210_write_200lux_calvalue->calvalue = a_als_low_calibration_adc;
	g_stk3210_data_as->g_als_200lux_calvalue = a_als_low_calibration_adc;

	queue_work(stk3210_workqueue, &stk3210_write_200lux_calvalue->write_200lux_calvalue_work);

	/*Write shift value*/
	stk3210_write_1000lux_calvalue = kmalloc(sizeof(struct write_shift), GFP_KERNEL);

	INIT_WORK(&stk3210_write_1000lux_calvalue->write_1000lux_calvalue_work, write_lightsensor_1000Lux_calibrationvalue_work);

	stk3210_write_1000lux_calvalue->calvalue = a_als_high_calibration_adc;
	g_stk3210_data_as->g_als_1000lux_calvalue = a_als_high_calibration_adc;

	queue_work(stk3210_workqueue, &stk3210_write_1000lux_calvalue->write_1000lux_calvalue_work);

	return count;
}

static DEVICE_ATTR(calibration_1000, S_IRWXU | S_IRWXG | S_IRWXO,
		   stk3210_show_calibration_1000, stk3210_store_calibration_1000);
#endif

/* ASUS BSP Peter_Lu for Proximity sensor new setting and new calibration issue +++ */
static bool read_prox_check_calibration_version(void)
{
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char readstr[8];
	int ori_val = 0, readlen = 0;
	mm_segment_t old_fs;

	printk("[stk3210] ++Read Check calibration version file open\n");

	fp = filp_open(PSENSOR_CALIBRATION_CHECK_ASUS_NV_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		printk("[stk3210] Read Check calibration version file open (%s) fail\n", PSENSOR_CALIBRATION_CHECK_ASUS_NV_FILE);
		printk("[stk3210] This device calibrationed by old version\n");
		return false;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, readstr, 6, &pos_lsts);
		readstr[readlen] = '\0';

		printk("[stk3210] strlen:%s(%d)\n", readstr, strlen(readstr));
	} else
		printk("[stk3210] Read Check calibration version file, f_op=NULL or op->read=NULL\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(readstr, "%d", &ori_val);

	printk("[stk3210] --Read Check calibration version file open\n");
	return true;
}
/* ASUS BSP Peter_Lu for Proximity sensor new setting and new calibration issue --- */

static bool read_prox_hi_calibrationvalue(void)
{
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char readstr[8];
	int ori_val = 0, readlen = 0;
	mm_segment_t old_fs;

	printk("[stk3210] ++read_psensor_hi_calvalue open\n");

	fp = filp_open(PSENSOR_CALIBRATION_HI_ASUS_NV_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		printk("[stk3210] read_psensor_hi_calvalue open (%s) fail\n", PSENSOR_CALIBRATION_HI_ASUS_NV_FILE);
		return false;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, readstr, 6, &pos_lsts);
		readstr[readlen] = '\0';

		printk("[stk3210] strlen:%s(%d)\n", readstr, strlen(readstr));
	} else
		printk("[stk3210] read_psensor_hi_calvalue, f_op=NULL or op->read=NULL\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(readstr, "%d", &ori_val);

	/* limit the calibration value range */
	g_stk3210_data_ps->g_ps_threshold_hi = ori_val;

	printk("[stk3210] read_psensor_hi_calvalues: Ori: %d, Cal: %d\n", ori_val, g_stk3210_data_ps->g_ps_threshold_hi);
	printk("[stk3210] --read_psensor_hi_calvalue open\n");
	return true;
}

static bool read_prox_lo_calibrationvalue(void)
{
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char readstr[8];
	int ori_val = 0, readlen = 0;
	mm_segment_t old_fs;

	printk("[stk3210] ++read_psensor_low_calvalue open\n");

	fp = filp_open(PSENSOR_CALIBRATION_LO_ASUS_NV_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		printk("[stk3210] read_psensor_low_calvalue open (%s) fail\n", PSENSOR_CALIBRATION_LO_ASUS_NV_FILE);
		return false;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, readstr, 6, &pos_lsts);
		readstr[readlen] = '\0';

		printk("[stk3210] strlen:%s(%d)\n", readstr, strlen(readstr));
	} else
		printk("[stk3210] read_psensor_low_calvalue, f_op=NULL or op->read=NULL\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(readstr, "%d", &ori_val);

	/* limit the calibration value range */
	g_stk3210_data_ps->g_ps_threshold_lo = ori_val;

	printk("[stk3210] read_psensor_low_calvalue: Ori: %d, Cal: %d\n", ori_val, g_stk3210_data_ps->g_ps_threshold_lo);
	printk("[stk3210] --read_psensor_low_calvalue open\n");
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

	fp = filp_open(PSENSOR_CALIBRATION_HI_ASUS_NV_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		printk("[stk3210] write_psensor_hi_calvalue open (%s) fail\n", PSENSOR_CALIBRATION_HI_ASUS_NV_FILE);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk("[stk3210] write_psensor_hi_calvalue = %d[%s(%d)]\n",
		this->calvalue, writestr, strlen(writestr));

	if (fp->f_op != NULL && fp->f_op->write != NULL)	{
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
		g_stk3210_data_ps->g_ps_threshold_hi = this->calvalue;
	} else
		printk("[stk3210] write_psensor_hi_calvalue fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

/* ASUS BSP Peter_Lu for Proximity sensor new setting and new calibration issue +++*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(PSENSOR_CALIBRATION_CHECK_ASUS_NV_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		printk("[stk3210] Check calibration version file open (%s) fail\n", PSENSOR_CALIBRATION_CHECK_ASUS_NV_FILE);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	if (fp->f_op != NULL && fp->f_op->write != NULL)	{
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	} else
		printk("[stk3210] write Check calibration version file fail !!\n");

	set_fs(old_fs);
	filp_close(fp, NULL);
/* ASUS BSP Peter_Lu for Proximity sensor new setting and new calibration issue ---*/

	return;
}

static int stk3210_show_calibration_prox_hi(struct device *dev, struct device_attribute *attr, char *buf)
{
	read_prox_hi_calibrationvalue();
	printk("[stk3210] Show_prox_hi_calibration: %d\n", g_stk3210_data_ps->g_ps_threshold_hi);
	return sprintf(buf, "%d\n", g_stk3210_data_ps->g_ps_threshold_hi);
}

static int stk3210_store_calibration_prox_hi(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if ((strict_strtoul(buf, 10, &val) < 0))
		return -EINVAL;

	/*Get hi threshold adc*/
	a_ps_hi_calibration_adc = (int)val;

	printk("[stk3210] Get calibration_prox_hi value : %d\n", a_ps_hi_calibration_adc);

	/*Write Calibration value*/
	stk3210_write_prox_hi = kmalloc(sizeof(struct write_calvalue), GFP_KERNEL);

	INIT_WORK(&stk3210_write_prox_hi->write_prox_hi_work, write_prox_hi_calibrationvalue_work);

	stk3210_write_prox_hi->calvalue = a_ps_hi_calibration_adc;

	queue_work(stk3210_workqueue, &stk3210_write_prox_hi->write_prox_hi_work);

	return a_ps_hi_calibration_adc;
}

static DEVICE_ATTR(calibration_prox_hi, S_IRWXU | S_IRWXG | S_IRWXO,
		   stk3210_show_calibration_prox_hi, stk3210_store_calibration_prox_hi);

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

	fp = filp_open(PSENSOR_CALIBRATION_LO_ASUS_NV_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		printk("[stk3210] write_psensor_low_calvalue open (%s) fail\n", PSENSOR_CALIBRATION_LO_ASUS_NV_FILE);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk("[stk3210] write_psensor_low_calvalue = %d[%s(%d)]\n",
		this->calvalue, writestr, strlen(writestr));

	if (fp->f_op != NULL && fp->f_op->write != NULL)	{
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
		g_stk3210_data_ps->g_ps_threshold_lo = this->calvalue;
	} else
		printk("[stk3210] write_psensor_low_calvalue fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}


static int stk3210_show_calibration_prox_lo(struct device *dev, struct device_attribute *attr, char *buf)
{
	read_prox_lo_calibrationvalue();
	printk("[stk3210] Show_prox_lo_calibration: %d\n", g_stk3210_data_ps->g_ps_threshold_lo);
	return sprintf(buf, "%d\n", g_stk3210_data_ps->g_ps_threshold_lo);
}

static int stk3210_store_calibration_prox_lo(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if ((strict_strtoul(buf, 10, &val) < 0))
		return -EINVAL;

	/*Get hi threshold adc*/
	a_ps_lo_calibration_adc = (int)val;

	printk("[stk3210] Get calibration_prox_hi value : %d\n", a_ps_lo_calibration_adc);

	/*Write Calibration value*/
	stk3210_write_prox_lo = kmalloc(sizeof(struct write_calvalue), GFP_KERNEL);

	INIT_WORK(&stk3210_write_prox_lo->write_prox_lo_work, write_prox_lo_calibrationvalue_work);

	stk3210_write_prox_lo->calvalue = a_ps_lo_calibration_adc;

	queue_work(stk3210_workqueue, &stk3210_write_prox_lo->write_prox_lo_work);

	return a_ps_lo_calibration_adc;
}

static DEVICE_ATTR(calibration_prox_lo, S_IRWXU | S_IRWXG | S_IRWXO,
		   stk3210_show_calibration_prox_lo, stk3210_store_calibration_prox_lo);
#endif

#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>
#include <linux/microp_api.h>

#define I2C_TEST_Lsensor_FAIL (-1)
#define I2C_TEST_Psensor_FAIL (-1)

static int IRSensor_I2C_stress_test(struct i2c_client *apClient)
{
	int lnResult = I2C_TEST_PASS;
	int err = 0;

	i2c_log_in_test_case("TestIRSensorI2C ++\n");

	if (!AX_MicroP_IsP01Connected())	{
		/* Light sensor */
		if (!g_stk3210_data_as->HAL_als_switch_on)	{
			err = stk3210_turn_onoff_als(1);
			if (err < 0)	{
				i2c_log_in_test_case("Fail to turn on IRsensor\n");
				lnResult = I2C_TEST_Lsensor_FAIL;
				goto error_1;
			}
			err = get_als_adc_from_stk3210();
			if (err < 0)	{
				i2c_log_in_test_case("Fail to read IRsensor data\n");
				lnResult = I2C_TEST_Lsensor_FAIL;
				goto error_1;
			} else		{
				err = stk3210_turn_onoff_als(0);
				if (err < 0)	{
					i2c_log_in_test_case("Fail to turn off IRsensor\n");
					lnResult = I2C_TEST_Lsensor_FAIL;
					goto error_1;
				}
			}
		} else	{
			err = get_als_adc_from_stk3210();
			if (err < 0)	{
				i2c_log_in_test_case("Fail to read IRsensor data\n");
				lnResult = I2C_TEST_Lsensor_FAIL;
				goto error_1;
			} else		{
				err = stk3210_turn_onoff_als(g_stk3210_data_as->HAL_als_switch_on);
				if (err < 0)	{
					i2c_log_in_test_case("Fail to turn off IRsensor\n");
					lnResult = I2C_TEST_Lsensor_FAIL;
					goto error_1;
				}
			}
		}

		/* Proximity sensor */
		if (!g_stk3210_data_ps->HAL_ps_switch_on)	{
			err = stk3210_turn_onoff_proxm(1);
			if (err < 0)	{
				i2c_log_in_test_case("Fail to turn on IRsensor\n");
				lnResult = I2C_TEST_Lsensor_FAIL;
				goto error_1;
			}
			err = get_ps_adc_from_stk3210(0);
			if (err < 0)	{
				i2c_log_in_test_case("Fail to read IRsensor data\n");
				lnResult = I2C_TEST_Lsensor_FAIL;
				goto error_1;
			} else		{
				err = stk3210_turn_onoff_proxm(0);
				if (err < 0)	{
					i2c_log_in_test_case("Fail to turn off IRsensor\n");
					lnResult = I2C_TEST_Lsensor_FAIL;
					goto error_1;
				}
			}
		} else	{
			err = get_ps_adc_from_stk3210(0);
			if (err < 0)	{
				i2c_log_in_test_case("Fail to read IRsensor data\n");
				lnResult = I2C_TEST_Lsensor_FAIL;
				goto error_1;
			} else		{
				err = stk3210_turn_onoff_proxm(g_stk3210_data_ps->HAL_ps_switch_on);
				if (err < 0)	{
					i2c_log_in_test_case("Fail to turn off IRsensor\n");
					lnResult = I2C_TEST_Lsensor_FAIL;
					goto error_1;
				}
			}
		}
	}
	i2c_log_in_test_case("TestLSensorI2C --\n");

error_1:
	return lnResult;
}

static struct i2c_test_case_info IRSensorTestCaseInfo[] =	{
	__I2C_STRESS_TEST_CASE_ATTR(IRSensor_I2C_stress_test),
};
#endif

/************************************************************************************
 *---proximity sensor part---
 */
static enum proximity_property stk3210_proxmdev_properties[] = {
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

atomic_t stk3210_proxm_update;

static int proxmdev_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	printk("[stk3210][ps] proxmdev_dev_open.\n");

	if (file->f_flags & O_NONBLOCK)
		printk("[stk3210][ps] proxmdl_dev_open (O_NONBLOCK)\n");

	/* initialize atomic. */
	atomic_set(&stk3210_proxm_update, 0);

	ret = nonseekable_open(inode, file);

	return ret;
}

static struct file_operations proxmdev_fops = {
    .owner = THIS_MODULE,
    .open = proxmdev_open,
};

struct proximity_class_dev stk3210_proxmDev = {
	.id = SENSORS_PROXIMITY,
	.name = "psensor",
	.num_properties = ARRAY_SIZE(stk3210_proxmdev_properties),
	.properties = stk3210_proxmdev_properties,
	.get_property = stk3210_proxmdev_get_property,
	.put_property = stk3210_proxmdev_put_property,
	.fops = &proxmdev_fops
};

static int stk3210_proxmdev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	switch (property)	{
	case SENSORS_PROP_HI_THRESHOLD:
		printk("[stk3210][ps] SENSORS_PROP_HI_THRESHOLD. (%d)\n", g_stk3210_data_ps->g_ps_threshold_hi);
		val->intval = g_stk3210_data_ps->g_ps_threshold_hi;
		break;

	case SENSORS_PROP_LO_THRESHOLD:
		printk("[stk3210][ps] SENSORS_PROP_LO_THRESHOLD. (%d)\n", g_stk3210_data_ps->g_ps_threshold_lo);
		val->intval = g_stk3210_data_ps->g_ps_threshold_lo;
		break;

	case SENSORS_PROP_INTERVAL:
		printk("[stk3210][ps] SENSORS_PROP_INTERVAL.\n");
		val->intval = 1;
		break;

	case SENSORS_PROP_MAXRANGE:
		printk("[stk3210][ps] SENSORS_PROP_MAXRANGE.\n");
		val->intval = 1;
		break;

	case SENSORS_PROP_RESOLUTION:
		printk("[stk3210][ps] SENSORS_PROP_RESOLUTION.\n");
		val->intval = 1;
		break;

	case SENSORS_PROP_VERSION:
		printk("[stk3210][ps] SENSORS_PROP_VERSION.\n");
		printk("[stk3210][ps] Ver 11.30.0.50\n");
		sprintf(val->strval, "Ver 11.30.0.50\r\n");
		val->intval = 1;
		break;

	case SENSORS_PROP_CURRENT:
		printk("[stk3210][ps] SENSORS_PROP_CURRENT.\n");
		val->intval = 30;
		break;

	case SENSORS_PROP_SWITCH:
		printk("[stk3210][ps] get switch = %d.\n", g_stk3210_data_ps->Device_ps_switch_on);
		val->intval = g_stk3210_data_ps->Device_ps_switch_on;
		break;

	case SENSORS_PROP_VENDOR:
		printk("[stk3210][ps] SENSORS_PROP_VENDOR.\n");
		sprintf(val->strval, "SensorTek");
		break;

	/* Add for debug only */
	case SENSORS_PROP_DBG:
		stk3210_Show_Allreg();
		printk("[stk3210][ps] dbg = %d.\n", 0);
		org_flag_reg = i2c_smbus_read_byte_data(stk3210_client, STK_FLAG_REG);
		printk("[stk3210][test] INT_FLAG = (0x%x)\n", org_flag_reg);
		val->intval = 0;
		break;

	case SENSORS_PROP_ADC:
		val->intval = get_ps_adc_from_stk3210(true);
		printk("[stk3210][ps] get adc property: %d\n", val->intval);
		break;

	case SENSORS_PROP_ATD_STATUS:
	{
		int als_sts = 0, als_adc = 0, ps_sts = 0;

		atd_write_status_and_adc_2(&als_sts, &als_adc, &ps_sts);
		val->intval = ps_sts;
		printk("[stk3210][ps] get atd status: %d\n", val->intval);
		break;
	}

	default:
		printk("[stk3210][ps] default.\n");
		return -EINVAL;
	}
	return 0;
}

static int stk3210_proxmdev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	int ret = 0;
	int err = 0;
	static bool bFirst = true;
	static bool openfilp = true;

	switch (property)	{
	case SENSORS_PROP_SWITCH:
		printk("[stk3210][ps] put SENSORS_PROP_SWITCH (%d,%d).\n",
							(val->intval), g_stk3210_data_ps->Device_ps_switch_on);

#ifdef ASUS_FACTORY_BUILD
		g_stk3210_data_ps->calibration_new_version = true;
#endif
#ifndef ASUS_FACTORY_BUILD
		/* ASUS BSP Peter_Lu for Proximity sensor new setting and new calibration issue+++ */
		g_stk3210_data_ps->calibration_new_version = read_prox_check_calibration_version();
		if (g_stk3210_data_ps->calibration_new_version)
			printk("[stk3210][ps] This device is calibrationed by new version\n");
		else
			printk("[stk3210][ps] This device is calibrationed by old version\n");
		/* ASUS BSP Peter_Lu for Proximity sensor new setting and new calibration issue--- */
#endif

		if (bFirst) {
			if (err < 0)
				printk("[stk3210][ps] switch init error\n");
			else {
				printk("[stk3210][ps] put switch 1st read calvalue\n");

				openfilp = read_prox_hi_calibrationvalue();

				if (openfilp == false) {
					printk("[stk3210][ps] Fail to open file. Get old calvalue : %d\n",
								g_stk3210_data_ps->g_ps_threshold_hi);
					g_stk3210_data_ps->g_ps_threshold_hi = DEFAULT_PS_THRESHOLD_hi;
					g_stk3210_data_ps->g_ps_threshold_lo = DEFAULT_PS_THRESHOLD_lo;
				} else
					read_prox_lo_calibrationvalue();

				printk("[stk3210] Set prox high threshold : %d\n", g_stk3210_data_ps->g_ps_threshold_hi);
				printk("[stk3210] Set prox low threshold : %d\n", g_stk3210_data_ps->g_ps_threshold_lo);

				/* Enable irq when trun on sensor uin first time */
/*
				if (g_stk3210_data_ps->is_irq_enable)	{
					printk("[stk3210][ps] Irq already enable, do nothing !! \n");
				} else	{
					printk("[stk3210][ps] Enable irq !! \n");
					enable_irq(asus_ir_sensor_irq_gpio);
					g_stk3210_data_ps->is_irq_enable = 1;
					g_stk3210_data_ps->irq_count++;
				}
*/
				bFirst = false;
			}
		}
		if ((g_stk3210_data_ps->Device_ps_switch_on != val->intval))	{
			mutex_lock(&g_stk3210_data_ps->lock);
			if (val->intval == 1)	{
				g_stk3210_data_ps->HAL_ps_switch_on = 1;
				if (!g_stk3210_data_ps->ps_in_pad_mode)	{
					/* turn on PS */
					ret = stk3210_turn_onoff_proxm(1);
					if (ret == 0)	{
						/* send an init value */
						input_report_abs(g_stk3210_data_ps->input_dev, ABS_DISTANCE, 1);
						input_sync(g_stk3210_data_ps->input_dev);
						printk("[stk3210][ps] proximity on.\n");
					}
				} else	{
					/* Make sure Psensor switch off in pad mode */
					if (g_stk3210_data_ps->Device_ps_switch_on)
						stk3210_turn_onoff_proxm(0);
					printk("[stk3210][ps] proximity on in Pad mode.\n");
				}
			} else	{
				/* turn off PS if val->intval==0 or other */
				g_stk3210_data_ps->HAL_ps_switch_on = 0;

				/* disable PS or directly Power off stk3210 */
				stk3210_turn_onoff_proxm(0);
				printk("[stk3210][ps] proximity off.\n");
			}
			mutex_unlock(&g_stk3210_data_ps->lock);
		}
/*
		else if (g_bIsP01Attached)	{
			printk("[stk32103][ps] Phone on in pad (%d,%d).\n",
							(val->intval), g_stk3210_data_ps->pad_proxm_switch_on);
			g_stk3210_data_ps->pad_proxm_switch_on = val->intval;
		}
*/
		break;

	case SENSORS_PROP_HI_THRESHOLD:
		printk("[stk3210][ps] config high THRESHOLD (%d).\n", val->intval);
		if (val->intval >= 0 && val->intval <= 255) {
			if (g_stk3210_data_ps->g_ps_threshold_hi != val->intval) {
				g_stk3210_data_ps->g_ps_threshold_hi = val->intval;
			}
		} else
			printk("[stk3210][ps] ERROR!!! OUT OF THRESHOLD (0~255)!!! \n");
		break;

	case SENSORS_PROP_LO_THRESHOLD:
		printk("[stk3210][ps] config low THRESHOLD (%d).\n", val->intval);
		if (val->intval >= 0 && val->intval <= 255) {
			if (g_stk3210_data_ps->g_ps_threshold_lo != val->intval) {
				g_stk3210_data_ps->g_ps_threshold_lo = val->intval;
			}
		} else
			printk("[stk3210][ps] ERROR!!! OUT OF THRESHOLD (0~255)!!! \n");
		break;

	case SENSORS_PROP_INTERVAL:
		if (1) {
			printk("[stk3210][ps] set interval (0x%x)\n", val->intval);
			/* interval = val->intval; */
		} else	{
			printk("[stk3210][ps] config IT_PS (0x%x)\n", val->intval);
#if 0
			if ((val->intval >= 0) && (val->intval <= 3)) {
				gpio_set_value(g_proxm_pwr_pin, 0);
				msleep(1);
				gpio_set_value(g_proxm_pwr_pin, 1);
				gpio_free(g_proxm_pwr_pin);
				ps_IT = val->intval;
				ret = stk3210_reset();
			}
#endif
		}
		break;

	/* Add for debug only */
	case SENSORS_PROP_DBG:
		g_proxm_dbg = val->intval;
		if (g_proxm_dbg == 1)	{
			printk("[stk3210][test] Get interrupt bit value(%d)\n", gpio_get_value(STK_GPIO_PROXIMITY_INT));
			printk("[stk3210][dbg] irq count (%d)\n", g_stk3210_data_ps->irq_count);
		} else if (g_proxm_dbg == 2)	{
			/* Software reset */
			ret = stk3x1x_software_reset();
			/* Clean interrupt */
			ret = i2c_smbus_write_byte_data(stk3210_client, STK_FLAG_REG, 0x00);
			printk("[stk3210][dbg] stk3210 Clean all STK_FLAG_REG!! \n");
			if (g_stk3210_data_ps->Device_ps_switch_on)	{
				input_report_abs(g_stk3210_data_ps->input_dev, ABS_DISTANCE, 1);
				input_sync(g_stk3210_data_ps->input_dev);
				stk3210_turn_onoff_proxm(1);
			}
			if (g_stk3210_data_as->Device_als_switch_on)
				stk3210_turn_onoff_als(1);
		} else if (g_proxm_dbg == 3)	{
			if (g_stk3210_data_ps->is_irq_enable && g_stk3210_data_ps->irq_count == 1)	{
				printk("[stk3210][dbg] Irq already Enable, do nothing !! \n");
			} else if (g_stk3210_data_ps->irq_count < 1)	{
				printk("[stk3210][dbg] Unbalance irq count (%d) !! \n", g_stk3210_data_ps->irq_count);
				printk("[stk3210][dbg] Try to enable irq !! \n");
				enable_irq(asus_ir_sensor_irq_gpio);
				g_stk3210_data_ps->is_irq_enable = 1;
				g_stk3210_data_ps->irq_count++;
				printk("[stk3210][dbg] irq count (%d)\n", g_stk3210_data_ps->irq_count);
			} else	{
				printk("[stk3210][dbg] Enable irq !! \n");
				enable_irq(asus_ir_sensor_irq_gpio);
				g_stk3210_data_ps->is_irq_enable = 1;
				g_stk3210_data_ps->irq_count++;
			}
		} else if (g_proxm_dbg == 4)	{
			if (g_stk3210_data_ps->is_irq_enable)	{
				printk("[stk3210][dbg] Disable irq !! \n");
				disable_irq_nosync(asus_ir_sensor_irq_gpio);
				g_stk3210_data_ps->is_irq_enable = 0;
				g_stk3210_data_ps->irq_count--;
			} else if (g_stk3210_data_ps->irq_count > 0)	{
				printk("[stk3210][dbg] Unbalance irq count (%d) !! \n", g_stk3210_data_ps->irq_count);
				printk("[stk3210][dbg] Try to disable irq !! \n");
				disable_irq_nosync(asus_ir_sensor_irq_gpio);
				g_stk3210_data_ps->is_irq_enable = 0;
				g_stk3210_data_ps->irq_count--;
				printk("[stk3210][dbg] irq count (%d)\n", g_stk3210_data_ps->irq_count);
			} else	{
				printk("[stk3210][dbg] Irq already disable, do nothing !! \n");
			}
		} else if (g_proxm_dbg == 5)	{
			printk("[stk3210][dbg] unrequest irq  !! \n");
			disable_irq(asus_ir_sensor_irq_gpio);
			g_stk3210_data_ps->is_irq_enable = 0;
			g_stk3210_data_ps->irq_count--;
			free_irq(asus_ir_sensor_irq_gpio, &stk3210_client->dev);
			gpio_free(STK_GPIO_PROXIMITY_INT);
			printk("[stk3210][dbg] gpio_free  !! \n");
			msleep(10);
			printk("[stk3210][dbg] Re-request gpio  !! \n");
			ret = gpio_request(STK_GPIO_PROXIMITY_INT, "ASUS_IRsensor-irq");
			if (ret)
				printk("[stk3210] Unable to request gpio ASUS_IRsensor-irq(%d)\n", asus_ir_sensor_irq_gpio);
			ret = gpio_direction_input(STK_GPIO_PROXIMITY_INT);
			if (ret < 0)
				printk("[stk3210] Unable to set the direction of gpio %d\n", asus_ir_sensor_irq_gpio);

			printk("[stk3210][dbg] Request irq  !! \n");
			ret = request_irq(asus_ir_sensor_irq_gpio,
					stk3210_irq_handler, IRQF_TRIGGER_LOW, "STK3210_INT",
					&stk3210_client->dev);

			if (ret < 0)
				printk("[stk3210][dbg] (g_stk3210_device.irq) request_irq() error %d.\n", ret);
			else	{
				printk("[stk3210][dbg] (g_stk3210_device.irq) request_irq ok.\n");
				disable_irq(asus_ir_sensor_irq_gpio);
				g_stk3210_data_ps->is_irq_enable = 0;
				g_stk3210_data_ps->irq_count--;
				msleep(10);
				enable_irq(asus_ir_sensor_irq_gpio);
				g_stk3210_data_ps->is_irq_enable = 1;
				g_stk3210_data_ps->irq_count++;
			}
		} else if (g_proxm_dbg == 6)	{
			printk("[stk3210][dbg] Disable irq !! \n");
			disable_irq_nosync(asus_ir_sensor_irq_gpio);
			g_stk3210_data_ps->is_irq_enable = 0;
			g_stk3210_data_ps->irq_count--;
			printk("[stk3210][dbg] irq count (%d)\n", g_stk3210_data_ps->irq_count);
		} else if (g_proxm_dbg == 7)	{
			queue_delayed_work(stk3210_delay_workqueue, &stk3210_light_proximity_delay_work, msecs_to_jiffies(1));
		} else if (g_proxm_dbg == 8)	{
			cancel_delayed_work(&stk3210_light_proximity_delay_work);
		} else
			printk("[stk3210][dbg] put default.\n");

		printk("[stk3210][dbg] dbg = %d.\n", g_proxm_dbg);
		break;

	default:
		printk("[stk3210][dbg] put default.\n");
		return -EINVAL;
	}
	return 0;
}

static int stk3210_turn_onoff_proxm(bool bOn)
{
	int err = 0;
	uint8_t power_state_data_8 = 0;
	uint8_t reg_data_8 = 0;

	printk("[stk3210][ps] sensor switch proximity sensor(%d)\n", bOn);

	power_state_data_8 = i2c_smbus_read_byte_data(stk3210_client, STK_STATE_REG);
	printk("[stk3210][ps] stk3210 read STK_STATE_REG : 0x%X\n", power_state_data_8);

	if (bOn == 1)	{	/* power on */
		/* Set PS config reg */
		/* ASUS BSP Peter_Lu for Proximity sensor new setting and new calibration issue +++ */
		if (g_stk3210_data_ps->calibration_new_version == true)
			reg_data_8 = (PS_PERS<<STK_PS_PRS_SHIFT | PS_GAIN_MR<<STK_PS_GAIN_SHIFT | PS_IT_MR);
		else
			reg_data_8 = (PS_PERS<<STK_PS_PRS_SHIFT | PS_GAIN<<STK_PS_GAIN_SHIFT | PS_IT);
		/* ASUS BSP Peter_Lu for Proximity sensor new setting and new calibration issue ---*/
		err = i2c_smbus_write_byte_data(stk3210_client, STK_PSCTRL_REG, reg_data_8);
		printk("[stk3210][ps] stk3210 Set STK_PSCTRL_REG : 0x%X\n", reg_data_8);

		/* Set IR LED config reg */
		/* ASUS BSP Peter_Lu for Proximity sensor new setting and new calibration issue +++ */
		if (g_stk3210_data_ps->calibration_new_version == true)
			reg_data_8 = (IRLED_IRDR_MR<<STK_LED_IRDR_SHIFT | IRLED_DT_MR);
		else
			reg_data_8 = (IRLED_IRDR<<STK_LED_IRDR_SHIFT | IRLED_DT);
		/* ASUS BSP Peter_Lu for Proximity sensor new setting and new calibration issue ---*/
		err = i2c_smbus_write_byte_data(stk3210_client, STK_LEDCTRL_REG, reg_data_8);
		printk("[stk3210][ps] stk3210 Set STK_LEDCTRL_REG : 0x%X\n", reg_data_8);

		/* Set PS threshold */
		err = i2c_smbus_write_byte_data(stk3210_client, STK_THDH1_PS_REG,
					((g_stk3210_data_ps->g_ps_threshold_hi>>8) & 0xFF));
		err = i2c_smbus_write_byte_data(stk3210_client, STK_THDH2_PS_REG,
					(g_stk3210_data_ps->g_ps_threshold_hi & 0xFF));
		if (err < 0)
			printk("[stk3210][ps] (%s):Set High threshold error=%d\n", __FUNCTION__, err);

		err = i2c_smbus_write_byte_data(stk3210_client, STK_THDL1_PS_REG,
					((g_stk3210_data_ps->g_ps_threshold_lo>>8) & 0xFF));
		err = i2c_smbus_write_byte_data(stk3210_client, STK_THDL2_PS_REG,
					(g_stk3210_data_ps->g_ps_threshold_lo & 0xFF));
		if (err < 0)
			printk("[stk3210][ps] (%s):Set Low threshold error=%d\n", __FUNCTION__, err);

		printk("[stk3210][ps] Setting g_ps_threshold_hi=%d, g_ps_threshold_lo=%d\n",
			g_stk3210_data_ps->g_ps_threshold_hi, g_stk3210_data_ps->g_ps_threshold_lo);

		/* Set interrupt mode (b0***,*001) */
		reg_data_8 = i2c_smbus_read_byte_data(stk3210_client, STK_INT_REG);
		if (!(reg_data_8 & PS_INT_EN_ON) || (reg_data_8>>STK_INT_CTRL_SHIFT))	{
			reg_data_8 = (reg_data_8 | PS_INT_EN_ON) & INT_CTRL;
			err = i2c_smbus_write_byte_data(stk3210_client, STK_INT_REG, reg_data_8);
		}
		printk("[stk3210][ps] stk3210 Set STK_INT_REG : 0x%X\n", reg_data_8);

		/* Clean interrupt */
		get_ps_adc_from_stk3210(0);
		org_flag_reg = i2c_smbus_read_byte_data(stk3210_client, STK_FLAG_REG);
		stk3x1x_set_flag(org_flag_reg, STK_FLG_PSINT_MASK);
/*
		usleep_range(1000, 2000);
		if (g_stk3210_data_ps->is_irq_enable)	{
			printk("[stk3210][ps] Irq already, do nothing !! \n");
		} else	{
			printk("[stk3210][ps] Enable irq !! \n");
			enable_irq(asus_ir_sensor_irq_gpio);
			g_stk3210_data_ps->is_irq_enable = 1;
			g_stk3210_data_ps->irq_count++;
		}
*/
		if (g_stk3210_data_ps->Device_ps_switch_on == false && g_stk3210_data_as->Device_als_switch_on == false) {
			printk("[stk3210][ps] Enable irq !! \n");
			enable_irq(asus_ir_sensor_irq_gpio);
		}

		/* Trun on PS */
		power_state_data_8 = power_state_data_8 | PS_SD_ON;
		err = i2c_smbus_write_byte_data(stk3210_client, STK_STATE_REG, power_state_data_8);
		printk("[stk3210][ps] stk3210 Set STK_STATE_REG : 0x%X\n", power_state_data_8);

		g_stk3210_data_ps->Device_ps_switch_on = true;
		printk("[stk3210][ps] sensor switch, turn on proximity sensor --.\n");

		queue_delayed_work(stk3210_delay_workqueue, &stk3210_light_proximity_delay_work, 0);
	} else	{
		printk("[stk3210][ps] sensor switch, turn off proximity sensor ++.\n");

		/* Clean interrupt */
		get_ps_adc_from_stk3210(0);
		org_flag_reg = i2c_smbus_read_byte_data(stk3210_client, STK_FLAG_REG);
		stk3x1x_set_flag(org_flag_reg, STK_FLG_PSINT_MASK);
/*
		if (g_stk3210_data_ps->is_irq_enable)	{
			if (g_stk3210_data_as->Device_als_switch_on != 1)	{
				printk("[stk3210][ps] Disable irq !! \n");
				disable_irq_nosync(asus_ir_sensor_irq_gpio);
				g_stk3210_data_ps->is_irq_enable = 0;
				g_stk3210_data_ps->irq_count--;
			} else
				printk("[stk3210][ps] ALS still turn on, without disable irq !! \n");
		} else
			printk("[stk3210][ps] Irq already disabled(%d) !! \n", g_stk3210_data_ps->is_irq_enable);
*/
		if (g_stk3210_data_ps->Device_ps_switch_on == true && g_stk3210_data_as->Device_als_switch_on == false) {
			printk("[stk3210][ps] Disable irq !! \n");
			disable_irq_nosync(asus_ir_sensor_irq_gpio);
		}

		/* Trun off PS */
		power_state_data_8 = power_state_data_8 & PS_SD_OFF;
		err = i2c_smbus_write_byte_data(stk3210_client, STK_STATE_REG, power_state_data_8);
		printk("[stk3210][ps] stk3210 Set STK_STATE_REG : 0x%X\n", power_state_data_8);

		g_stk3210_data_ps->Device_ps_switch_on = false;
		printk("[stk3210][ps] sensor switch, turn off proximity sensor --.\n");
	}

	printk("[stk3210][test] Get interrupt bit(%d)\n", gpio_get_value(STK_GPIO_PROXIMITY_INT));
	printk("[stk3210][ps] irq count (%d)\n", g_stk3210_data_ps->irq_count);
	return err;
}

static int get_ps_adc_from_stk3210(bool trigger_by_interrupt)
{
	int data = 0;
	int lsb = 0, msb = 0;

	if (trigger_by_interrupt)	{
		data = stk3210_read_reg(0x3E);
	}
	/* printk("[stk3210][ps] Read ps ID : 0x%X\n", data);*/

	if (g_stk3210_data_ps->Device_ps_switch_on == true)	{
		msb = i2c_smbus_read_byte_data(stk3210_client, STK_DATA1_PS_REG);
		if (msb < 0)	{
			printk("[stk3210][ps] Get msb fail (%d)\n", msb);
			return msb;
		}
		lsb = i2c_smbus_read_byte_data(stk3210_client, STK_DATA2_PS_REG);
		data = (u32)((msb << 8) | lsb) ;

		/* printk("[stk3210][ps] read-HiByte (%d). read-LoByte (%d)\n", msb, lsb);
		printk("[stk3210][ps] Get adc : %d\n", data); */
	} else
		printk("[stk3210][ps] Proximity sensor is off state\n");

	return data;
}

static int get_ps_environmaent_adc_from_stk3210(void)
{
	int32_t data = 0, tmp_data = 0;

	if (g_stk3210_data_ps->Device_ps_switch_on == true)	{
		tmp_data = i2c_smbus_read_word_data(stk3210_client, STK_ENVIRONMENT_DATA1_PS_REG);
		if (tmp_data < 0)	{
			printk("[stk3210][ps] Get environmaent data fail (%d)\n", tmp_data);
			return tmp_data;
		}
		data = (((tmp_data & 0x00FF) << 8) | ((tmp_data & 0xFF00) >> 8)) ;
		tmp_data = i2c_smbus_read_word_data(stk3210_client, STK_ENVIRONMENT_DATA2_PS_REG);
		data += (((tmp_data & 0x00FF) << 8) | ((tmp_data & 0xFF00) >> 8)) ;
	} else
		printk("[stk3210][ps_env] Proximity sensor is off state\n");

	return data;
}

static void stk3210_Proximity_detect_disrance_work(void)
{
	g_stk3210_data_ps->proxm_detect_object_state = (org_flag_reg & STK_FLG_NF_MASK) ? 1 : 0;

	printk("/----------------------------------------------------\n");

	/* if (adc >= g_stk3210_data_ps->g_ps_threshold_hi) {  //panel off */
	if (g_stk3210_data_ps->proxm_detect_object_state == 0) {
		input_report_abs(g_stk3210_data_ps->input_dev, ABS_DISTANCE, 0);
		input_sync(g_stk3210_data_ps->input_dev);
		/* g_stk3210_data_ps->proxm_detect_object_state = 1; */
		printk("[stk3210][ps] trigger panel off\n");
	} else	{
		input_report_abs(g_stk3210_data_ps->input_dev, ABS_DISTANCE, 1);
		input_sync(g_stk3210_data_ps->input_dev);
		wake_unlock(&proximity_wake_lock);
		/* g_stk3210_data_ps->proxm_detect_object_state = 0; */
		printk("[stk3210][ps] trigger panel on\n");
	}
}

/************************************************************************************
 *---Attribute part---
 */
static enum proximity_property stk3210_ambientDev_properties[] = {
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

atomic_t stk3210_ambient_update;

static int ambientDev_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	printk("[stk3210][als] ambientdl_dev_open \n");

	if (file->f_flags & O_NONBLOCK)
		printk("[stk3210][als] ambientdl_dev_open (O_NONBLOCK)\n");

	/* initialize atomic. */
	atomic_set(&stk3210_ambient_update, 0);

	ret = nonseekable_open(inode, file);

	return ret;
}

static struct file_operations ambientDev_fops = {
	.owner = THIS_MODULE,
	.open = ambientDev_open,
};

struct proximity_class_dev stk3210_ambientDev = {
	.id = SENSORS_LIGHT,
	.name = "lsensor",
	.num_properties = ARRAY_SIZE(stk3210_ambientDev_properties),
	.properties = stk3210_ambientDev_properties,
	.get_property = stk3210_ambientDev_get_property,
	.put_property = stk3210_ambientDev_put_property,
	.fops = &ambientDev_fops
};

static int stk3210_ambientDev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	printk("[stk3210][als] ambientdl_get_property +.\n");

	switch (property)	{
	case SENSORS_PROP_HI_THRESHOLD:
		printk("[stk3210][als] SENSORS_PROP_HI_THRESHOLD. (%d)\n", g_stk3210_data_as->g_als_threshold_hi);
		val->intval = g_stk3210_data_as->g_als_threshold_hi;
		break;

	case SENSORS_PROP_LO_THRESHOLD:
		printk("[stk3210][als] SENSORS_PROP_LO_THRESHOLD. (%d)\n", g_stk3210_data_as->g_als_threshold_lo);
		val->intval = g_stk3210_data_as->g_als_threshold_lo;
		break;

	case SENSORS_PROP_INTERVAL:
		printk("[stk3210][als] config GAIN_ALS by using \"echo XXX > /sys/class/sensors/sensor4/interval\" XXX is only 0~3. \n");
		printk("[stk3210][als] SENSORS_PROP_INTERVAL.\n");
		/*val->intval = g_interval;*/
		break;

	case SENSORS_PROP_MAXRANGE:
		printk("[stk3210][als] SENSORS_PROP_MAXRANGE.\n");
		val->intval = 128;
		break;

	case SENSORS_PROP_RESOLUTION:
		printk("[stk3210][als] SENSORS_PROP_RESOLUTION.\n");
		val->intval = 1;
		break;

	case SENSORS_PROP_VERSION:
		printk("[stk3210][als] SENSORS_PROP_VERSION.\n");
		val->intval = 1;
		break;

	case SENSORS_PROP_CURRENT:
		printk("[stk3210][als] SENSORS_PROP_CURRENT.\n");
		val->intval = 30;
		break;

	case SENSORS_PROP_SWITCH:
		printk("[stk3210][als] get switch = %d.\n", g_stk3210_data_as->HAL_als_switch_on);
		val->intval = g_stk3210_data_as->HAL_als_switch_on;
		break;

	case SENSORS_PROP_VENDOR:
		printk("[stk3210][als] SENSORS_PROP_VENDOR.\n");
		sprintf(val->strval, "Sensortek");
		break;

	/* Add for debug only */
	case SENSORS_PROP_DBG:
		stk3210_Show_Allreg();
		val->intval = g_ambient_dbg;
		printk("[stk3210][als] dbg = %d.\n", g_ambient_dbg);
		org_flag_reg = i2c_smbus_read_byte_data(stk3210_client, STK_FLAG_REG);
		printk("[stk3210][test] INT_FLAG = (0x%x)\n", org_flag_reg);
		break;

	case SENSORS_PROP_ADC:
		val->intval = get_als_adc_from_stk3210();
		printk("[stk3210][als] get adc property: %d\n", val->intval);
		break;

	case SENSORS_PROP_K_ADC:
		val->intval = g_stk3210_data_as->light_k_adc;
		printk("[stk3210][als] get k_adc property: %d\n", val->intval);
		break;

	case SENSORS_PROP_LUX:
		val->intval = g_stk3210_data_as->light_lux;
		printk("[stk3210][als] get lux property: %d\n", val->intval);
		break;

	case SENSORS_PROP_ATD_STATUS:
	{
		int als_sts = 0, als_adc = 0, ps_sts = 0;

		atd_write_status_and_adc_2(&als_sts, &als_adc, &ps_sts);
		val->intval = als_sts;
		printk("[stk3210][als] get atd status: %d\n", val->intval);
		break;
	}

	case SENSORS_PROP_ATD_ADC:
	{
		int als_sts = 0, als_adc = 0, ps_sts = 0;
		atd_write_status_and_adc_2(&als_sts, &als_adc, &ps_sts);

		val->intval = als_adc;
		printk("[stk3210][als] get atd adc: %d\n", val->intval);
		break;
	}

	default:
		printk("[stk3210][als] default\n");
		return -EINVAL;
	}
	printk("[stk3210]: ambientdl_get_property -.\n");

	return 0;
}

static int stk3210_ambientDev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	int ret = 0;
	static bool bFirst = true;
	static bool openfilp = true;

	switch (property)	{
	case SENSORS_PROP_SWITCH:
		printk("[stk3210][als] put SENSORS_PROP_SWITCH (%d,%d).\n",
			(val->intval), g_stk3210_data_as->HAL_als_switch_on);

		/* read calibration value */
		if (bFirst) {
			printk("[stk3210][als] put switch 1st read calvalue\n");

			openfilp = read_lightsensor_200lux_calibrationvalue();

			/* Surpport old calibration value */
			if (g_stk3210_data_as->g_als_200lux_calvalue <= 0 || openfilp == false)	{
				printk("[stk3210][als] Get old calvalue or fail\n");
				g_stk3210_data_as->g_als_200lux_calvalue = DEFAULT_ALS_200LUX_CALVALUE;
				g_stk3210_data_as->g_als_1000lux_calvalue = DEFAULT_ALS_1000LUX_CALVALUE;
			} else
				read_lightsensor_1000lux_calibrationvalue();
			printk("[stk3210] Get 200Lux and 1000Lux calibration value : %d, %d\n",
				g_stk3210_data_as->g_als_200lux_calvalue,
				g_stk3210_data_as->g_als_1000lux_calvalue);

			/* Enable irq when trun on sensor uin first time */
/*
			if (g_stk3210_data_ps->is_irq_enable)	{
				printk("[stk3210][als] Irq already, do nothing !! \n");
			} else	{
				printk("[stk3210][als] Enable irq !! \n");
				enable_irq(asus_ir_sensor_irq_gpio);
				g_stk3210_data_ps->is_irq_enable = 1;
				g_stk3210_data_ps->irq_count++;
			}
*/
			bFirst = false;
		}
		g_stk3210_data_as->g_als_shift_calvalue = (1000 - g_stk3210_data_as->g_als_1000lux_calvalue*800/
			(g_stk3210_data_as->g_als_1000lux_calvalue-g_stk3210_data_as->g_als_200lux_calvalue));
		printk("[stk3210] Set Shift calibration value : %d\n", g_stk3210_data_as->g_als_shift_calvalue);
		mutex_lock(&g_stk3210_data_ps->lock);
		if (val->intval > 0)	{
			g_stk3210_data_as->HAL_als_switch_on = 1;
			if (!g_stk3210_data_as->als_in_pad_mode)
				stk3210_turn_onoff_als(g_stk3210_data_as->HAL_als_switch_on);
			else	{
				/* Make sure Lsensor switch off in pad mode */
				if (g_stk3210_data_as->Device_als_switch_on)
					stk3210_turn_onoff_als(0);
				printk("[stk3210][als] Lightsensor on in Pad mode.\n");
			}
		} else	{
			g_stk3210_data_as->HAL_als_switch_on = 0;
			stk3210_turn_onoff_als(g_stk3210_data_as->HAL_als_switch_on);
		}
		mutex_unlock(&g_stk3210_data_ps->lock);
/*
		if (g_bIsP01Attached)	{
			printk("[stk3210][als] sensor switch, turn on/off al3010: %d\n", g_HAL_als_switch_on);
			als_lux_report_event(0);
			set_als_power_state_of_P01(g_HAL_als_switch_on);
		} else	 {
			printk("[stk3210][als] sensor switch, turn on/off stk3210: %d\n", g_HAL_als_switch_on);
			stk3210_turn_onoff_als(g_HAL_als_switch_on);
		}
*/
		break;

	case SENSORS_PROP_LO_THRESHOLD:
		break;

	case SENSORS_PROP_HI_THRESHOLD:
		break;

	case SENSORS_PROP_INTERVAL:
		break;

	/* Add for debug only */
	case SENSORS_PROP_DBG:
		g_ambient_dbg = val->intval;

		if (g_ambient_dbg == 1)	{
			printk("[stk3210][test] Get interrupt bit value(%d)\n", gpio_get_value(STK_GPIO_PROXIMITY_INT));
			printk("[stk3210][dbg] irq count (%d)\n", g_stk3210_data_ps->irq_count);
		} else if (g_ambient_dbg == 2)	{
			/* Software reset */
			ret = stk3x1x_software_reset();
			/* Clean interrupt */
			ret = i2c_smbus_write_byte_data(stk3210_client, STK_FLAG_REG, 0x00);
			printk("[stk3210][dbg] stk3210 Clean all STK_FLAG_REG!! \n");
			if (g_stk3210_data_ps->Device_ps_switch_on)	{
				input_report_abs(g_stk3210_data_ps->input_dev, ABS_DISTANCE, 1);
				input_sync(g_stk3210_data_ps->input_dev);
				stk3210_turn_onoff_proxm(1);
			}
			if (g_stk3210_data_as->Device_als_switch_on)
				stk3210_turn_onoff_als(1);
		} else if (g_ambient_dbg == 3)	{
			if (g_stk3210_data_ps->is_irq_enable && g_stk3210_data_ps->irq_count == 1)	{
				printk("[stk3210][dbg] Irq already, do nothing !! \n");
			} else if (g_stk3210_data_ps->irq_count < 1)	{
				printk("[stk3210][dbg] Unbalance irq count (%d) !! \n", g_stk3210_data_ps->irq_count);
				printk("[stk3210][dbg] Try to enable irq !! \n");
				enable_irq(asus_ir_sensor_irq_gpio);
				g_stk3210_data_ps->is_irq_enable = 1;
				g_stk3210_data_ps->irq_count++;
				printk("[stk3210][dbg] irq count (%d)\n", g_stk3210_data_ps->irq_count);
			} else	{
				printk("[stk3210][dbg] Enable irq !! \n");
				enable_irq(asus_ir_sensor_irq_gpio);
				g_stk3210_data_ps->is_irq_enable = 1;
				g_stk3210_data_ps->irq_count++;
			}
		} else if (g_ambient_dbg == 4)	{
			if (g_stk3210_data_ps->is_irq_enable)	{
				printk("[stk3210][dbg] Disable irq !! \n");
				disable_irq_nosync(asus_ir_sensor_irq_gpio);
				g_stk3210_data_ps->is_irq_enable = 0;
				g_stk3210_data_ps->irq_count--;
			} else if (g_stk3210_data_ps->irq_count > 0)	{
				printk("[stk3210][dbg] Unbalance irq count (%d) !! \n", g_stk3210_data_ps->irq_count);
				printk("[stk3210][dbg] Try to disable irq !! \n");
				disable_irq_nosync(asus_ir_sensor_irq_gpio);
				g_stk3210_data_ps->is_irq_enable = 0;
				g_stk3210_data_ps->irq_count--;
				printk("[stk3210][dbg] irq count (%d)\n", g_stk3210_data_ps->irq_count);
			} else	{
				printk("[stk3210][dbg] Irq already disable, do nothing !! \n");
			}
		} else if (g_ambient_dbg == 5)	{
			printk("[stk3210][dbg] unrequest irq  !! \n");
			disable_irq(asus_ir_sensor_irq_gpio);
			g_stk3210_data_ps->is_irq_enable = 0;
			g_stk3210_data_ps->irq_count--;
			free_irq(asus_ir_sensor_irq_gpio, &stk3210_client->dev);
			gpio_free(STK_GPIO_PROXIMITY_INT);
			printk("[stk3210][dbg] gpio_free  !! \n");
			msleep(10);
			printk("[stk3210][dbg] Re-request gpio  !! \n");
			ret = gpio_request(STK_GPIO_PROXIMITY_INT, "ASUS_IRsensor-irq");
			if (ret)
				printk("[stk3210] Unable to request gpio ASUS_IRsensor-irq(%d)\n", asus_ir_sensor_irq_gpio);
			ret = gpio_direction_input(STK_GPIO_PROXIMITY_INT);
			if (ret < 0)
				printk("[stk3210] Unable to set the direction of gpio %d\n", asus_ir_sensor_irq_gpio);

			printk("[stk3210][dbg] Request irq  !! \n");
			ret = request_irq(asus_ir_sensor_irq_gpio,
					stk3210_irq_handler, IRQF_TRIGGER_LOW, "STK3210_INT",
					&stk3210_client->dev);

			if (ret < 0)
				printk("[stk3210][dbg] (g_stk3210_device.irq) request_irq() error %d.\n", ret);
			else	{
				printk("[stk3210][dbg] (g_stk3210_device.irq) request_irq ok.\n");
				disable_irq(asus_ir_sensor_irq_gpio);
				g_stk3210_data_ps->is_irq_enable = 0;
				g_stk3210_data_ps->irq_count--;
				msleep(10);
				enable_irq(asus_ir_sensor_irq_gpio);
				g_stk3210_data_ps->is_irq_enable = 1;
				g_stk3210_data_ps->irq_count++;
			}
		} else
			printk("[stk3210][dbg] put default.\n");

		printk("[stk3210][dbg] dbg = %d.\n", g_ambient_dbg);
		break;

	case SENSORS_PROP_CALIBRATION:
/*
		g_stk3210_light_gain_calibration = val->intval;
		printk("[stk3210][als] Gain calibration val = %d.%d\n",
						g_stk3210_light_gain_calibration/a_als_calibration_accuracy,
						g_stk3210_light_gain_calibration%a_als_calibration_accuracy);
*/
		break;

	default:
		printk("[stk3210][als] put default.\n");
		return -EINVAL;
	}
	return 0;
}

static int stk3210_turn_onoff_als(bool bOn)
{
	int err = 0;
	uint8_t power_state_data_8 = 0;
	uint8_t reg_data_8 = 0;

	printk("[stk3210][als] sensor switch Light sensor(%d)\n", bOn);

	power_state_data_8 = i2c_smbus_read_byte_data(stk3210_client, STK_STATE_REG);
	printk("[stk3210] stk3210 read STK_STATE_REG : 0x%X\n", power_state_data_8);

	if (bOn == 1)	{	/*power on*/
		/* Set ALS contral register (0x6A) */
		reg_data_8 = (ALS_PERS<<STK_ALS_PRS_SHIFT | ALS_GAIN<<STK_ALS_GAIN_SHIFT | ALS_IT);
		err = i2c_smbus_write_byte_data(stk3210_client, STK_ALSCTRL_REG, reg_data_8);
		printk("[stk3210][als] stk3210 Set STK_ALSCTRL_REG : 0x%X\n", reg_data_8);

		/* Reset ALS threshold */
		err = i2c_smbus_write_byte_data(stk3210_client, STK_THDH1_ALS_REG, 0x00);
		err = i2c_smbus_write_byte_data(stk3210_client, STK_THDH2_ALS_REG, 0x00);
		err = i2c_smbus_write_byte_data(stk3210_client, STK_THDL1_ALS_REG, 0xFF);
		err = i2c_smbus_write_byte_data(stk3210_client, STK_THDL2_ALS_REG, 0xFF);

		/* Set interrupt mode (b0000,1***) */
		reg_data_8 = i2c_smbus_read_byte_data(stk3210_client, STK_INT_REG);
		if (!(reg_data_8>>STK_INT_ALS_SHIFT & ALS_INT_EN_ON) ||
							(reg_data_8>>STK_INT_CTRL_SHIFT))	{
			reg_data_8 = (reg_data_8 | ALS_INT_EN_ON<<STK_INT_ALS_SHIFT) & INT_CTRL;
			err = i2c_smbus_write_byte_data(stk3210_client, STK_INT_REG, reg_data_8);
		}
		printk("[stk3210][als] stk3210 Set STK_INT_REG : 0x%X\n", reg_data_8);

		/* Clean interrupt */
		get_als_adc_from_stk3210();
		org_flag_reg = i2c_smbus_read_byte_data(stk3210_client, STK_FLAG_REG);
		stk3x1x_set_flag(org_flag_reg, STK_FLG_ALSINT_MASK);
/*
		usleep_range(1000, 2000);
		if (g_stk3210_data_ps->is_irq_enable)	{
			printk("[stk3210][als] Irq already, do nothing !! \n");
		} else	{
			printk("[stk3210][als] Enable irq !! \n");
			enable_irq(asus_ir_sensor_irq_gpio);
			g_stk3210_data_ps->is_irq_enable = 1;
			g_stk3210_data_ps->irq_count++;
		}
*/
		if (g_stk3210_data_as->Device_als_switch_on == false && g_stk3210_data_ps->Device_ps_switch_on == false) {
			printk("[stk3210][ps] Enable irq !! \n");
			enable_irq(asus_ir_sensor_irq_gpio);
		}

		/* Trun on ALS */
		power_state_data_8 = power_state_data_8 | ALS_SD_ON;
		err = i2c_smbus_write_byte_data(stk3210_client, STK_STATE_REG, power_state_data_8);
		printk("[stk3210][als] stk3210 Set STK_STATE_REG : 0x%X\n", power_state_data_8);

		g_stk3210_data_as->Device_als_switch_on = true;
		printk("[stk3210][als] sensor switch, turn on Light sensor --.\n");

		queue_delayed_work(stk3210_delay_workqueue, &stk3210_light_proximity_delay_work, msecs_to_jiffies(400));
	} else	{
		printk("[stk3210][als] sensor switch, turn off Light sensor ++.\n");

		/* Clean interrupt */
		get_als_adc_from_stk3210();
		org_flag_reg = i2c_smbus_read_byte_data(stk3210_client, STK_FLAG_REG);
		stk3x1x_set_flag(org_flag_reg, STK_FLG_ALSINT_MASK);
/*
		if (g_stk3210_data_ps->is_irq_enable)	{
			if (g_stk3210_data_ps->Device_ps_switch_on != 1)	{
				printk("[stk3210][als] Disable irq !! \n");
				disable_irq_nosync(asus_ir_sensor_irq_gpio);
				g_stk3210_data_ps->is_irq_enable = 0;
				g_stk3210_data_ps->irq_count--;
			} else
				printk("[stk3210][als] PS still turn on, without disable irq !! \n");
		} else
			printk("[stk3210][als] Irq already disabled(%d) !! \n", g_stk3210_data_ps->is_irq_enable);
*/
		if (g_stk3210_data_as->Device_als_switch_on == true && g_stk3210_data_ps->Device_ps_switch_on == false) {
			printk("[stk3210][ps] Disable irq !! \n");
			disable_irq_nosync(asus_ir_sensor_irq_gpio);
		}

		/* Trun off ALS */
		power_state_data_8 = power_state_data_8 & ALS_SD_OFF;
		err = i2c_smbus_write_byte_data(stk3210_client, STK_STATE_REG, power_state_data_8);
		printk("[stk3210][als] stk3210 Set STK_STATE_REG : 0x%X\n", power_state_data_8);

		g_stk3210_data_as->Device_als_switch_on = false;
		printk("[stk3210][als] sensor switch, turn off Light sensor --.\n");

		als_lux_report_event(-1);
	}
	printk("[stk3210][test] Get interrupt bit(%d)\n", gpio_get_value(STK_GPIO_PROXIMITY_INT));
	printk("[stk3210][als] irq count (%d)\n", g_stk3210_data_ps->irq_count);
	return err;
}

static int get_als_adc_from_stk3210(void)
{
	int data = 0;
	int lsb = 0, msb = 0;

	if (g_stk3210_data_as->Device_als_switch_on == true)	{
		msb = i2c_smbus_read_byte_data(stk3210_client, STK_DATA1_ALS_REG);
		if (msb < 0)	{
			printk("[stk3210][als] Get msb fail (%d)\n", msb);
			return msb;
		}
		lsb = i2c_smbus_read_byte_data(stk3210_client, STK_DATA2_ALS_REG);
		data = (u32)((msb << 8) | lsb) ;

		/*printk("[stk3210][als] read-HiByte (%d). read-LoByte (%d)\n", msb, lsb);
		printk("[stk3210][als] Get adc : %d\n", data);*/
	} else
		printk("[stk3210][als] Light sensor is off state\n");

	return data;
}

static int get_calibrated_lux_from_stk3210_adc(int adc)
{
	int lux = 0;

	lux = (adc * 800/(g_stk3210_data_as->g_als_1000lux_calvalue-g_stk3210_data_as->g_als_200lux_calvalue)
			+ g_stk3210_data_as->g_als_shift_calvalue);

	g_stk3210_data_as->light_k_adc = lux;

	/*Get Lux*/
	if (adc < 10 || g_stk3210_data_as->light_k_adc < 0)
		g_stk3210_data_as->light_k_adc = 0;

	g_stk3210_data_as->light_lux = g_stk3210_data_as->light_k_adc;

	if (g_stk3210_data_as->light_lux > DEFAULT_MAX_ALS_LUX)
		g_stk3210_data_as->light_lux = DEFAULT_MAX_ALS_LUX;

	if (g_stk3210_data_as->light_lux != g_stk3210_data_as->last_light_lux)
		g_stk3210_data_as->last_light_lux = g_stk3210_data_as->light_lux;
/*
	printk(DBGMSK_PRX_G3"[stk3210][als] adc=%d, k_adc=%d, lux=%d, last=%d\n",
		adc, lux, g_stk3210_data_as->light_lux, g_stk3210_data_as->last_light_lux);
*/
	return lux;
}

/************************************************************************************
 *---stk3210 interrupt part---
 */
static int32_t stk3x1x_set_flag(uint8_t org_flag_reg, uint8_t clr)
{
	uint8_t w_flag;
	int ret;

	w_flag = org_flag_reg;
	w_flag &= (~clr);
	/* printk("[stk3210] stk3210 Set STK_FLAG_REG : 0x%X\n", w_flag); */
	ret = i2c_smbus_write_byte_data(stk3210_client, STK_FLAG_REG, w_flag);
	if (ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);

	return ret;
}

static int clean_and_try_to_reset_interrupt_error(void)
{
	stk3x1x_software_reset();
	printk("[stk3210][isr_reset] WARNING!!!! Trigger IRQ without sensor contral(0x%x)\n", org_flag_reg);
	/*
	 *Reset Wait config reg
	 *0x00 5.93ms default
	 *0x07 50ms for Sensortek suggest setting ( Delay time same with ALS operation time )
	 */
	i2c_smbus_write_byte_data(stk3210_client, STK_WAIT_REG, 0x07);

	/* Clean interrupt */
	i2c_smbus_write_byte_data(stk3210_client, STK_FLAG_REG, 0x00);
	printk("[stk3210][isr_reset] stk3210 Clean all STK_FLAG_REG!! \n");
	msleep(100);
	if (!gpio_get_value(STK_GPIO_PROXIMITY_INT))	{
		if (g_stk3210_data_ps->irq_count == 0)	{
			printk("[stk3210][isr_reset] Reser interrupt bit(%d) !!!!\n", gpio_get_value(STK_GPIO_PROXIMITY_INT));
		} else if (g_stk3210_data_ps->irq_count == -1)	{
			g_stk3210_data_ps->irq_count++;
			/* enable_irq(asus_ir_sensor_irq_gpio); */
			printk("[stk3210][isr_reset] Reser irq+ (%d)\n", g_stk3210_data_ps->irq_count);
		} else if (g_stk3210_data_ps->irq_count == 1)	{
			g_stk3210_data_ps->irq_count--;
			/* disable_irq(asus_ir_sensor_irq_gpio); */
			printk("[stk3210][isr_reset] Reser irq- (%d)\n", g_stk3210_data_ps->irq_count);
		} else
			printk("[stk3210][isr_reset] Unbalance irq  count (%d)\n", g_stk3210_data_ps->irq_count);
	} else	{
		if (g_stk3210_data_ps->irq_count == 0)	{
			printk("[stk3210][isr_reset] Reser interrupt bit(%d) successful !!!!\n", gpio_get_value(STK_GPIO_PROXIMITY_INT));
		} else if (g_stk3210_data_ps->irq_count == -1)	{
			g_stk3210_data_ps->irq_count++;
			/* enable_irq(asus_ir_sensor_irq_gpio);*/
			printk("[stk3210][isr_reset] Reser irq+ (%d)\n", g_stk3210_data_ps->irq_count);
		} else if (g_stk3210_data_ps->irq_count == 1)	{
			g_stk3210_data_ps->irq_count--;
			/* disable_irq(asus_ir_sensor_irq_gpio);*/
			printk("[stk3210][isr_reset] Reser irq- (%d)\n", g_stk3210_data_ps->irq_count);
		} else
			printk("[stk3210][isr_reset] Unbalance irq  count (%d)\n", g_stk3210_data_ps->irq_count);

	}
	if (g_stk3210_data_ps->Device_ps_switch_on)	{
		input_report_abs(g_stk3210_data_ps->input_dev, ABS_DISTANCE, 1);
		input_sync(g_stk3210_data_ps->input_dev);
		stk3210_turn_onoff_proxm(1);
	}
	if (g_stk3210_data_as->Device_als_switch_on)
		stk3210_turn_onoff_als(1);
	printk("[stk3210][isr_reset] (%d)---!! \n", g_stk3210_data_ps->irq_count);
	if (g_stk3210_data_ps->irq_count < 1)
		g_stk3210_data_ps->is_irq_enable = 0;
	return 0;
}

static irqreturn_t stk3210_irq_handler(int irq, void *dev_id)
{
	/* if (g_stk3210_data_ps->is_irq_enable)	{
		g_stk3210_data_ps->is_irq_enable = 0;
		printk("[stk3210][irq] Disable irq !! \n");
		disable_irq_nosync(asus_ir_sensor_irq_gpio);
		g_stk3210_data_ps->irq_count--;
	} else	{
		printk("[stk3210][irq] Irq already disable, do nothing(%d) !! \n", g_stk3210_data_ps->irq_count);
	}
	* */

	disable_irq_nosync(asus_ir_sensor_irq_gpio);
	queue_work(stk3210_workqueue, &stk3210_ISR_work);
	if (g_stk3210_data_ps->proxm_detect_object_state == 1)
		wake_lock_timeout(&proximity_wake_lock, 1 * HZ);

	return IRQ_HANDLED;
}

static void stk3210_als_proxm_interrupt_handler(struct work_struct *work)
{
	uint8_t disable_flag = 0;
	mutex_lock(&g_stk3210_data_ps->lock);
	printk("[stk3210][irq] interrupt handler ++\n");

	org_flag_reg = i2c_smbus_read_byte_data(stk3210_client, STK_FLAG_REG);
	uint8_t tmp_flag_reg = org_flag_reg;
	/* printk("[stk3210][isr] INT_FLAG = (0x%x)\n", org_flag_reg); */
	int stk3210_Proximity_Interrupt = 0;
	int stk3210_Light_Interrupt = 0;

	/* Check interrupt status */
	if ((org_flag_reg & 0x30) == 0 || (!g_stk3210_data_ps->Device_ps_switch_on && !g_stk3210_data_as->Device_als_switch_on))	{
		/*
		 *IRQ is triggered without sensor contral try to clean and reset sensor
		 */
		clean_and_try_to_reset_interrupt_error();
	}

	/* Proximity sensor port */
	if (org_flag_reg & STK_FLG_PSINT_MASK)	{
		/* stk3210_Proximity_work(); */
		/*
		queue_delayed_work(stk3210_delay_workqueue, &stk3210_proximity_interrupt_delay_work, 0);
		*/
		stk3210_Proximity_Interrupt = 1;
		disable_flag |= STK_FLG_PSINT_MASK;
	}

	/* Light sensor port */
	if (org_flag_reg & STK_FLG_ALSINT_MASK)	{
		/* stk3210_light_work(); */
		/*
		queue_delayed_work(stk3210_delay_workqueue, &stk3210_light_interrupt_delay_work, 0);
		*/
		stk3210_Light_Interrupt = 1;
		disable_flag |= STK_FLG_ALSINT_MASK;
	}

	/* Clean interrupt bit and enable irq */
	stk3x1x_set_flag(org_flag_reg, disable_flag);
	usleep_range(1000, 2000);

	org_flag_reg = i2c_smbus_read_byte_data(stk3210_client, STK_FLAG_REG);
	printk("[stk3210][isr] After clean INT_FLAG = (0x%x --> 0x%x)\n", tmp_flag_reg, org_flag_reg);
	/* Proximity sensor port */
	if (stk3210_Proximity_Interrupt == 1)	{
		stk3210_Proximity_Interrupt = 0;
		stk3210_Proximity_work();
		/*
		queue_delayed_work(stk3210_delay_workqueue, &stk3210_proximity_interrupt_delay_work, 0);
		*/
		disable_flag |= STK_FLG_PSINT_MASK;
	}

	/* Light sensor port */
	if (stk3210_Light_Interrupt == 1)	{
		stk3210_Light_Interrupt = 0;
		stk3210_light_work();
		/*
		queue_delayed_work(stk3210_delay_workqueue, &stk3210_light_interrupt_delay_work, 0);
		*/
		/* ASUS BSP Peter_Lu for Light/Proximity sensor new setting issue */
		if (g_stk3210_data_ps->Device_ps_switch_on)
			stk3210_Proximity_work();

		disable_flag |= STK_FLG_ALSINT_MASK;
	}

/*
	if (g_stk3210_data_ps->Device_ps_switch_on  || g_stk3210_data_as->Device_als_switch_on)	{

		if (g_stk3210_data_ps->is_irq_enable)	{
			if (g_stk3210_data_ps->irq_count != 1)
				printk("[stk3210][isr] Irq already Enable, do nothing(%d) !! \n", g_stk3210_data_ps->irq_count);
		} else	{
			printk("[stk3210][isr] Enable irq !! \n");
			g_stk3210_data_ps->is_irq_enable = 1;
			g_stk3210_data_ps->irq_count++;
			enable_irq(asus_ir_sensor_irq_gpio);
		}

	}
	else
		printk("[stk3210][isr] Without any sensor turn on,  Do nothing!!!! \n");
*/
	enable_irq(asus_ir_sensor_irq_gpio);

	/* printk("[stk3210][isr] irq count (%d)\n", g_stk3210_data_ps->irq_count); */
	printk("[stk3210][irq] interrupt handler --\n");
	mutex_unlock(&g_stk3210_data_ps->lock);
}

static void stk3210_Proximity_work(void)
{
	int adc = 0, environment_adc = 0, indx = 0;

	g_stk3210_data_ps->proxm_detect_object_state = (org_flag_reg & STK_FLG_NF_MASK) ? 1 : 0;

#if 0
	adc = get_ps_adc_from_stk3210(true);
	environment_adc = get_ps_environmaent_adc_from_stk3210();

	printk("/----------------------------------------------------\n");
	printk("[stk3210][ps] PS_data = %d, Environment_data = %d \n", adc, environment_adc);
#endif

	/* if (adc >= g_stk3210_data_ps->g_ps_threshold_hi) {  //panel off */
	if (g_stk3210_data_ps->proxm_detect_object_state == 0) {
		/* ASUS BSP Peter_Lu for Proximity sensor new setting and new calibration issue */
		if (g_stk3210_data_ps->calibration_new_version) {
			adc = get_ps_adc_from_stk3210(true);
			environment_adc = get_ps_environmaent_adc_from_stk3210();
			printk("[stk3210][ps] trigger panel off (%d, %d)\n", environment_adc, adc);
			input_report_abs(g_stk3210_data_ps->input_dev, ABS_DISTANCE, 0);
			input_sync(g_stk3210_data_ps->input_dev);
		/* For old calibration version */
		}	else	{
			for (indx = 1; indx <= PS_retry_time; indx++)	{
				adc = get_ps_adc_from_stk3210(true);
				environment_adc = get_ps_environmaent_adc_from_stk3210();
				/* Environment IR could effect PS Detect */
				if (environment_adc > g_stk3210_data_ps->g_ps_threshold_hi)	{
					if (environment_adc > 2400 || adc < g_stk3210_data_ps->g_ps_threshold_hi)	{
						printk("[stk3210][ps] Effect by environment IR(%d), abandon panel off event in %d.st(%d)!!\n", environment_adc, indx, adc);
						break;
					}
					if (adc > (2*g_stk3210_data_ps->g_ps_threshold_hi))	{
						printk("[stk3210][ps] Object close enough(%d), Trigger panel off (%d)\n", environment_adc, adc);
						input_report_abs(g_stk3210_data_ps->input_dev, ABS_DISTANCE, 0);
						input_sync(g_stk3210_data_ps->input_dev);
						break;
					}
				}
				/* Withour environment IR effect PS Detect */
				else	{
					if (adc > g_stk3210_data_ps->g_ps_threshold_hi || indx == 1)	{
						printk("[stk3210][ps] trigger panel off (%d, %d)\n", environment_adc, adc);
						input_report_abs(g_stk3210_data_ps->input_dev, ABS_DISTANCE, 0);
						input_sync(g_stk3210_data_ps->input_dev);
						break;
					}
				}
				msleep(15);
			}
		}
#if 0
		/* Environment IR could effect PS Detect */
		if (environment_adc > g_stk3210_data_ps->g_ps_threshold_hi)	{
			for (indx = 1; indx <= PS_retry_time; indx++)	{
				if (environment_adc > 2400 || adc < g_stk3210_data_ps->g_ps_threshold_hi)	{
					printk("[stk3210][ps] Check Ps environment Low data(%d) fail in %d.st\n", environment_adc, indx);
					printk("[stk3210][ps] Effect by environment IR, abandon panel off event!!\n");
					adc = 0;
					break;
				}
				msleep(15);
				adc = get_ps_adc_from_stk3210(true);
				environment_adc = get_ps_environmaent_adc_from_stk3210();
				if (environment_adc < g_stk3210_data_ps->g_ps_threshold_hi && adc > g_stk3210_data_ps->g_ps_threshold_hi)	{
					input_report_abs(g_stk3210_data_ps->input_dev, ABS_DISTANCE, 0);
					input_sync(g_stk3210_data_ps->input_dev);
					printk("[stk3210][ps] trigger panel off without Environment effect (%d)\n", environment_adc);
					break;
				}
				if (adc > (2*g_stk3210_data_ps->g_ps_threshold_hi))	{
					printk("[stk3210][ps] Object close enough, Trigger panel off (%d)\n", adc);
					input_report_abs(g_stk3210_data_ps->input_dev, ABS_DISTANCE, 0);
					input_sync(g_stk3210_data_ps->input_dev);
					break;
				}
				if (indx >= PS_retry_time)	{
					printk("[stk3210][ps] Retry finish(%d), Environment IR still effect PS (%d)\n", indx , environment_adc);
				}
			}
		}
		/* Withour environment IR effect PS Detect */
		else	{
			input_report_abs(g_stk3210_data_ps->input_dev, ABS_DISTANCE, 0);
			input_sync(g_stk3210_data_ps->input_dev);
			printk("[stk3210][ps] trigger panel off (%d)\n", environment_adc);
		}
#endif
	/* g_stk3210_data_ps->proxm_detect_object_state = 1; */
	} else	{
		input_report_abs(g_stk3210_data_ps->input_dev, ABS_DISTANCE, 1);
		input_sync(g_stk3210_data_ps->input_dev);
		adc = get_ps_adc_from_stk3210(true);
		environment_adc = get_ps_environmaent_adc_from_stk3210();
		wake_unlock(&proximity_wake_lock);
		/* g_stk3210_data_ps->proxm_detect_object_state = 0; */
		printk("[stk3210][ps] trigger panel on (%d, %d)\n", environment_adc, adc);
	}
}

static void stk3210_light_work(void)
{
	int err = 0;
	int threshold_range = 5;

	/* Get adc value */
	g_stk3210_data_as->light_adc = get_als_adc_from_stk3210();

	/* Set new ALS threshold value */
	if (g_stk3210_data_as->last_light_lux >= 1000)
		threshold_range = 10;
	else if (g_stk3210_data_as->last_light_lux >= 200 && g_stk3210_data_as->last_light_lux < 1000)
		threshold_range = 5;
	else
		threshold_range = 2;
	g_stk3210_data_as->g_als_threshold_hi = (g_stk3210_data_as->light_adc*(100 + threshold_range)/100);
	g_stk3210_data_as->g_als_threshold_lo = (g_stk3210_data_as->light_adc*(100 - threshold_range)/100);

	/* Set ALS threshold */
	err = i2c_smbus_write_byte_data(stk3210_client, STK_THDH1_ALS_REG,
					((g_stk3210_data_as->g_als_threshold_hi>>8) & 0xFF));
	err = i2c_smbus_write_byte_data(stk3210_client, STK_THDH2_ALS_REG,
					(g_stk3210_data_as->g_als_threshold_hi & 0xFF));
	if (err < 0)
		printk("[stk3210][als] (%s):Set High threshold error=%d\n", __FUNCTION__, err);

	err = i2c_smbus_write_byte_data(stk3210_client, STK_THDL1_ALS_REG,
					((g_stk3210_data_as->g_als_threshold_lo>>8) & 0xFF));
	err = i2c_smbus_write_byte_data(stk3210_client, STK_THDL2_ALS_REG,
					(g_stk3210_data_as->g_als_threshold_lo & 0xFF));
	if (err < 0)
		printk("[stk3210][als] (%s):Set Low threshold error=%d\n", __FUNCTION__, err);
/*
	printk(DBGMSK_PRX_G3"[stk3210][als] Setting threshold: adc=%d, g_als_threshold_hi=%d, g_als_threshold_lo=%d\n",
		g_stk3210_data_as->light_adc, g_stk3210_data_as->g_als_threshold_hi, g_stk3210_data_as->g_als_threshold_lo);
*/
	/* Get Lux */
	get_calibrated_lux_from_stk3210_adc(g_stk3210_data_as->light_adc);

	als_lux_report_event(g_stk3210_data_as->light_lux);
}

static void stk3210_light_proximity_delay_work_handler(struct work_struct *work)
{
	mutex_lock(&g_stk3210_data_ps->lock);
	if (g_stk3210_data_ps->Device_ps_switch_on && g_proxm_dbg == 7)	{
		printk("[stk3210][ps] PS_data = %d, PS_environmaent_data = %d\n",
			get_ps_adc_from_stk3210(true), get_ps_environmaent_adc_from_stk3210());
		queue_delayed_work(stk3210_delay_workqueue, &stk3210_light_proximity_delay_work, msecs_to_jiffies(1));
	}
	if (g_stk3210_data_ps->Device_ps_switch_on && g_proxm_dbg != 7)	{
		org_flag_reg = i2c_smbus_read_byte_data(stk3210_client, STK_FLAG_REG);
		stk3210_Proximity_work();
	}
	if (g_stk3210_data_as->Device_als_switch_on && g_proxm_dbg != 7)
		stk3210_light_work();
	mutex_unlock(&g_stk3210_data_ps->lock);
}

/************************************************************************************
 *---stk3210_attributes part---
 */
static int stk3210_show_adc(struct device *dev, struct device_attribute *attr, char *buf)
{
	int adc = 0;

	adc = get_als_adc_from_stk3210();

	return sprintf(buf, "%d\n", adc);
}

static DEVICE_ATTR(adc, S_IRWXU | S_IRWXG | S_IROTH, stk3210_show_adc, NULL);

static int stk3210_show_proxm (struct device *dev, struct device_attribute *attr, char *buf)
{
	int adc;

	adc = get_ps_adc_from_stk3210(false);

	return sprintf(buf, "%d\n", adc);
}

static DEVICE_ATTR(proxm, S_IRWXU | S_IRWXG  | S_IROTH, stk3210_show_proxm, NULL);

static struct attribute *stk3210_attributes[] = {
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

static const struct attribute_group stk3210_attr_group = {
    .name = "stk3210",
	.attrs = stk3210_attributes,
};

/************************************************************************************
 *---Pad mode part---
 */
#ifdef CONFIG_EEPROM_PADSTATION
static int stk3210_lightsensor_mp_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	switch (event) {
	case P01_ADD:
		printk("[stk3210][MicroP] Pad_ADD \r\n");
		/*
		* If HAL already turned on sensor, we switch to Pad light sensor
		* and switch off proximity sensor also return panel on event
		*/
		mutex_lock(&g_stk3210_data_ps->lock);
		g_stk3210_data_ps->ps_in_pad_mode = true;
		if (g_stk3210_data_ps->Device_ps_switch_on)	{
			/* send an panel on event to make sure Pad panel must trun on */
			input_report_abs(g_stk3210_data_ps->input_dev, ABS_DISTANCE, 1);
			input_sync(g_stk3210_data_ps->input_dev);
			stk3210_turn_onoff_proxm(0);
		}

		g_stk3210_data_as->als_in_pad_mode = true;
		if (g_stk3210_data_as->HAL_als_switch_on)
			stk3210_turn_onoff_als(0);
		mutex_unlock(&g_stk3210_data_ps->lock);

		return NOTIFY_DONE;

	case P01_REMOVE:
		printk("[stk3210][MicroP] Pad_REMOVE \r\n");
		mutex_lock(&g_stk3210_data_ps->lock);
		g_stk3210_data_ps->ps_in_pad_mode = false;
		if (g_stk3210_data_ps->Device_ps_switch_on)
			stk3210_turn_onoff_proxm(1);

		g_stk3210_data_as->als_in_pad_mode = false;
		if (g_stk3210_data_as->HAL_als_switch_on)
			stk3210_turn_onoff_als(1);
		mutex_unlock(&g_stk3210_data_ps->lock);

		return NOTIFY_DONE;
	default:
		return NOTIFY_DONE;
	}
}

static struct notifier_block stk3210_lightsensor_mp_notifier = {
	.notifier_call = stk3210_lightsensor_mp_event,
	.priority = PHONE_LIGHTSENSOR_MP_NOTIFY,
};
#endif

/************************************************************************************
 *---Init driver part---
 */
static int ir_sensor_input_init(unsigned int mode)
{
	int ret = 0;
	struct input_dev *input_dev_as = NULL;
	struct input_dev *input_dev_ps = NULL;

	if (mode == ALS_MODE)	{
		input_dev_as = input_allocate_device();
		if (!input_dev_as) {
			ret = -ENOMEM;
			printk("[ASUS_Sensoer]: Failed to allocate input_data device\n");
			goto error_1;
		}

		input_dev_as->name = "ASUS Lightsensor";
		input_dev_as->id.bustype = BUS_I2C;
		input_set_capability(input_dev_as, EV_ABS, ABS_MISC);
		__set_bit(EV_ABS, input_dev_as->evbit);
		__set_bit(ABS_MISC, input_dev_as->absbit);
		input_set_abs_params(input_dev_as, ABS_MISC, 0, 65535, 0, 0);
		input_set_drvdata(input_dev_as, g_stk3210_data_as);

		ret = input_register_device(input_dev_as);
		if (ret < 0) {
			input_free_device(input_dev_as);
			goto error_1;
		}
		g_stk3210_data_as->input_dev = input_dev_as;


		g_stk3210_data_as->polling = 0;
		g_stk3210_data_as->poll_interval_ms = 100;
		g_stk3210_data_as->event_threshold = 1000;

		/* Register light input event  */
		ret = als_lux_report_event_register(g_stk3210_data_as->input_dev);
	}

	if (mode == PS_MODE)	{
		input_dev_ps = input_allocate_device();
		if (!input_dev_ps) {
			ret = -ENOMEM;
			printk("[ASUS_Sensor]: Failed to allocate input_data device\n");
			goto error_1;
		}

		input_dev_ps->name = "ASUS Proximitysensor";
		input_dev_ps->id.bustype = BUS_I2C;
		input_set_capability(input_dev_ps, EV_ABS, ABS_DISTANCE);
		__set_bit(EV_ABS, input_dev_ps->evbit);
		__set_bit(ABS_DISTANCE, input_dev_ps->absbit);
		input_set_abs_params(input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);
		input_set_drvdata(input_dev_ps, g_stk3210_data_ps);

		ret = input_register_device(input_dev_ps);
		if (ret < 0) {
			input_free_device(input_dev_ps);
			goto error_1;
		}
		g_stk3210_data_ps->input_dev = input_dev_ps;

		g_stk3210_data_ps->polling = 0;
		g_stk3210_data_ps->poll_interval_ms = 100;
		g_stk3210_data_ps->event_threshold = 1000;
	}

error_1:

	return ret;
}

static int stk3210_reset(void)
{
	int err = 0;

	printk("[stk3210] SW Sensor Reset ++.\n");
	/* SW Reset sensor */
	stk3x1x_software_reset();

	/*Reset STK_STATE_REG
	 *0x05 only enable ps;0x02 only enable als;
	 *0x00 disable all &go into a low power standby mode
	 */
	err = i2c_smbus_write_byte_data(stk3210_client, STK_STATE_REG, 0x00);

	/*Reset PS config reg
	 *0x31 default PSCTRL 	 ps_persistance=1, ps_gain=64X, PS_IT=0.391ms
	 */
	err = i2c_smbus_write_byte_data(stk3210_client, STK_PSCTRL_REG, 0x31);

	/*Reset ALS config reg
	 *0x7b 1.5s
	 *0x39 94.72ms default
	 *0x38 ALSCTRL	als_persistance=1, als_gain=64X, ALS_IT=378.88*4=1515.52ms
	 */
	err = i2c_smbus_write_byte_data(stk3210_client, STK_ALSCTRL_REG, 0x6A);

	/*
	 *Reset IR LED config reg
	 *100mA IRDR, 64/64 LED duty
	 */
	err = i2c_smbus_write_byte_data(stk3210_client, STK_LEDCTRL_REG, (IRLED_IRDR<<STK_LED_IRDR_SHIFT | IRLED_DT));

	/*Reset Interrupt config reg
	 *INT 0x08=INT_ALS[3]=1'b1 only enable als
	 *wile(INT_ALS[3]=1'b1)
	 *0x0c=INT_PS[2:0] 3'b100 ,system pre-defined sequence (wk test yes);
	 *0x09=INT_PS[2:0] 3'b001,interrupt is issued while FLG_NF is toggled;
	 *0x0a=INT_PS[2:0] 3'b010/3'b011, FLAG Mode;
	 *0x0f=INT_PS[2:0] 3'b101/3'b110/3'b111 , issue continuous interrupt
	 */
	err = i2c_smbus_write_byte_data(stk3210_client, STK_INT_REG, 0x09);

	/*
	 *Reset Wait config reg
	 *0x00 5.93ms default
	 *0x07 50ms for Sensortek suggest setting ( Delay time same with ALS operation time )
	 */
	err = i2c_smbus_write_byte_data(stk3210_client, STK_WAIT_REG, 0x07);

	/* Reset PS threshold */
	err = i2c_smbus_write_byte_data(stk3210_client, STK_THDH1_PS_REG, 0xFF);
	err = i2c_smbus_write_byte_data(stk3210_client, STK_THDH2_PS_REG, 0xFF);
	err = i2c_smbus_write_byte_data(stk3210_client, STK_THDL1_PS_REG, 0x00);
	err = i2c_smbus_write_byte_data(stk3210_client, STK_THDL2_PS_REG, 0x00);

	/* Reset ALS threshold */
	err = i2c_smbus_write_byte_data(stk3210_client, STK_THDH1_ALS_REG, 0xFF);
	err = i2c_smbus_write_byte_data(stk3210_client, STK_THDH2_ALS_REG, 0xFF);
	err = i2c_smbus_write_byte_data(stk3210_client, STK_THDL1_ALS_REG, 0x00);
	err = i2c_smbus_write_byte_data(stk3210_client, STK_THDL2_ALS_REG, 0x00);

	/* Clean interrupt */
	err = i2c_smbus_write_byte_data(stk3210_client, STK_FLAG_REG, 0x00);

	g_proxm_dbg = 0;
	g_ambient_dbg = 0;

	g_stk3210_light_shift_calibration = 40;
	g_stk3210_light_gain_calibration = 38;
	a_als_calibration_accuracy = 100000;

#ifdef ASUS_FACTORY_BUILD
	a_als_calibration_lux = 800;
	a_als_low_calibration_adc = 0;
	a_als_high_calibration_adc = 0;
	a_ps_hi_calibration_adc = 0;
	a_ps_lo_calibration_adc = 0;
#endif
	return err;
}

static int init_stk3210(void)
{
	int ret = 0;
	printk("[stk3210]: init_stk3210 +.\n");

	/* reset both reg */
	ret = stk3210_reset();

	/* Set workqueue */
	stk3210_workqueue = create_singlethread_workqueue("stk3210_wq");
	INIT_WORK(&stk3210_ISR_work, stk3210_als_proxm_interrupt_handler);

	stk3210_delay_workqueue = create_singlethread_workqueue("stk3210_delay_wq");
	INIT_DELAYED_WORK(&stk3210_light_proximity_delay_work, stk3210_light_proximity_delay_work_handler);
/*
	INIT_DELAYED_WORK(&stk3210_proximity_interrupt_delay_work, stk3210_Proximity_work);
*/
/*
#ifdef CONFIG_EEPROM_NUVOTON
	INIT_WORK(&stk3210_attached_Pad_work, stk3210_lightsensor_attached_pad);
#endif
*/
	wake_lock_init(&proximity_wake_lock, WAKE_LOCK_SUSPEND, "proxm_wake_lock");

	/* register irq handler after sensor power on in order to avoid unexpected irq error! */
	asus_ir_sensor_irq_gpio = gpio_to_irq(STK_GPIO_PROXIMITY_INT);
	printk("[stk3210]Reques EIRQ %d succesd on GPIO:%d\n", asus_ir_sensor_irq_gpio, STK_GPIO_PROXIMITY_INT);

	if (asus_ir_sensor_irq_gpio < 0)
		printk("[stk3210] gpio_to_irq fail (g_stk3210_device.irq)irq=%d.\n", asus_ir_sensor_irq_gpio);
	else		{
		printk("[stk3210] (g_stk3210_device.irq) irq=%d.\n", asus_ir_sensor_irq_gpio);

		g_stk3210_data_ps->is_irq_enable = 1;
		ret = request_irq(asus_ir_sensor_irq_gpio,
			stk3210_irq_handler, IRQF_TRIGGER_LOW, "STK3210_INT",
			&stk3210_client->dev);

		if (ret < 0)
			printk("[stk3210] (g_stk3210_device.irq) request_irq() error %d.\n", ret);
		else		{
			printk("[stk3210] (g_stk3210_device.irq) request_irq ok.\n");
			disable_irq(asus_ir_sensor_irq_gpio);
			g_stk3210_data_ps->is_irq_enable = 0;
			/*
			msleep(5);
			enable_irq(asus_ir_sensor_irq_gpio);
			g_stk3210_data_ps->is_irq_enable = 1;
			*/
		}
	}

	printk("[stk3210] init_stk3210 -.\n");
	return ret;
}

static int stk3210_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	printk("[stk3210] stk3210_probe +.\n");

	if (client == NULL) {
		printk("[stk3210] Client is NUll.\n");
		ret =  -EFAULT;
		goto stk3210_probe_err;
	}

	/* Reset ASUS_light_sensor_data */
	g_stk3210_data_as = kmalloc(sizeof(struct ASUS_light_sensor_data), GFP_KERNEL);
	if (!g_stk3210_data_as)	{
		ret = -ENOMEM;
		goto stk3210_probe_err;
	}
	memset(g_stk3210_data_as, 0, sizeof(struct ASUS_light_sensor_data));

	g_stk3210_data_as->g_als_threshold_hi = 0;
	g_stk3210_data_as->g_als_threshold_lo = 0;
	g_stk3210_data_as->g_als_200lux_calvalue = DEFAULT_ALS_200LUX_CALVALUE;
	g_stk3210_data_as->g_als_1000lux_calvalue = DEFAULT_ALS_1000LUX_CALVALUE;
	g_stk3210_data_as->last_light_lux = 0;
	g_stk3210_data_as->light_adc = 0;
	g_stk3210_data_as->light_k_adc = 0;
	g_stk3210_data_as->Device_als_switch_on = 0;
	g_stk3210_data_as->HAL_als_switch_on = 0;
	g_stk3210_data_as->als_in_pad_mode = 0;

	/* Reset ASUS_proximity_sensor_data */
	g_stk3210_data_ps = kmalloc(sizeof(struct ASUS_proximity_sensor_data), GFP_KERNEL);
	if (!g_stk3210_data_ps) {
		ret = -ENOMEM;
		goto stk3210_probe_err;
	}
	memset(g_stk3210_data_ps, 0, sizeof(struct ASUS_proximity_sensor_data));

	g_stk3210_data_ps->Device_ps_switch_on = 0;
	g_stk3210_data_ps->g_ps_threshold_hi = DEFAULT_PS_THRESHOLD_hi;
	g_stk3210_data_ps->g_ps_threshold_lo = DEFAULT_PS_THRESHOLD_lo;
	g_stk3210_data_ps->HAL_ps_switch_on = 0;
	g_stk3210_data_ps->ps_in_pad_mode = 0;
	g_stk3210_data_ps->proxm_detect_object_state = 0;
	g_stk3210_data_ps->is_irq_enable = 0;
	g_stk3210_data_ps->calibration_new_version = 1;
	g_stk3210_data_ps->irq_count = 0;
	mutex_init(&g_stk3210_data_ps->lock);

	/* store i2c client data structure */
	stk3210_client = client;
	i2c_set_clientdata(stk3210_client, g_stk3210_data_as);
	i2c_set_clientdata(stk3210_client, g_stk3210_data_ps);

	/* configure Phone Lightsensor interrupt gpio */
	asus_ir_sensor_irq_gpio = get_gpio_by_name("ALS_INT#");
	printk("[stk3210] intr_gpio =%d(%d)\n", asus_ir_sensor_irq_gpio, get_gpio_by_name("ALS_INT#"));
	printk("[stk3210][test] Get interrupt bit value(%d)\n", gpio_get_value(asus_ir_sensor_irq_gpio));
	ret = gpio_request(asus_ir_sensor_irq_gpio, "ASUS_IRsensor-irq");
	if (ret) {
		printk("[stk3210] Unable to request gpio ASUS_IRsensor-irq(%d)\n", asus_ir_sensor_irq_gpio);
		goto stk3210_probe_err;
	}
	ret = gpio_direction_input(asus_ir_sensor_irq_gpio);
	if (ret < 0) {
		printk("[stk3210] Unable to set the direction of gpio %d\n", asus_ir_sensor_irq_gpio);
		goto stk3210_probe_err;
	}

	strlcpy(stk3210_client->name, "stk3210", I2C_NAME_SIZE);

	printk("[stk3210] Register input device...\n");
	if (ir_sensor_input_init(ALS_MODE) != 0) {
		goto stk3210_probe_err;
	}
	if (ir_sensor_input_init(PS_MODE) != 0) {
		goto stk3210_probe_err;
	}

	/* init HW, irq, wq */
	ret = init_stk3210();
	if (ret < 0)  {
		printk("[stk3210] init_stk3210() error(%d).\n", ret);
		goto stk3210_probe_err;
	}

	ret = proximity_dev_register(&stk3210_proxmDev);
	if (ret)
		printk("[stk3210] proxmdl create sysfile fail.\n");

	ret = proximity_dev_register(&stk3210_ambientDev);
	if (ret)
		printk("[stk3210] ambientdl create sysfile fail.\n");

	/* register sysfs hooks */
	ret = sysfs_create_group(&client->dev.kobj, &stk3210_attr_group);
	if (ret)
		goto stk3210_probe_err;

#ifdef CONFIG_EEPROM_PADSTATION
	register_microp_notifier(&stk3210_lightsensor_mp_notifier);
#endif

#ifdef CONFIG_I2C_STRESS_TEST
	printk("IRSenor add test case+\n");
	i2c_add_test_case(client, "IRSensorTest", ARRAY_AND_SIZE(IRSensorTestCaseInfo));
	printk("IRensor add test case-\n");
#endif

	printk("[stk3210] stk3210_probe -.\n");
	return 0;

stk3210_probe_err:
	printk("[stk3210] stk3210_probe - (error).\n");
	return ret;
}


static int stk3210_remove(struct i2c_client *client)
{
	return 0;
}

static int stk3210_shutdown(struct i2c_client *client)
{
	printk("[stk3210] stk3210_shutdown +++\n");
	/* Disable sensor */
	if (g_stk3210_data_as->Device_als_switch_on)
		stk3210_turn_onoff_als(0);
	if (g_stk3210_data_ps->Device_ps_switch_on)
		stk3210_turn_onoff_proxm(0);
	/* Disable irq */
	if (g_stk3210_data_ps->is_irq_enable)	{
		if (g_stk3210_data_as->Device_als_switch_on != 1)	{
			printk("[stk3210][shutdown] Disable irq !! \n");
			disable_irq_nosync(asus_ir_sensor_irq_gpio);
			g_stk3210_data_ps->is_irq_enable = 0;
			g_stk3210_data_ps->irq_count--;
		} else
			printk("[stk3210][shutdown] ALS still turn on, without disable irq !! \n");
	}
	printk("[stk3210][shutdown] irq count (%d)\n", g_stk3210_data_ps->irq_count);
	printk("[stk3210] stk3210_shutdown ---\n");
	return 0;
}

static int stk3210_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("[stk3210] stk3210_suspend, psensor:%d, als:%d\n",
		g_stk3210_data_ps->Device_ps_switch_on, g_stk3210_data_as->Device_als_switch_on);

	/* For keep Proximity can wake_up system */
	if (g_stk3210_data_ps->Device_ps_switch_on)
		enable_irq_wake(asus_ir_sensor_irq_gpio);

	/* For make sure Light sensor mush be switch off when system suspend */
	if (g_stk3210_data_as->Device_als_switch_on == 1)	{
		//In case upper layer doesn't switch off ambient before early_suspend.
		printk("[stk3210] stk3210_suspend, turn off ambient\n");
		stk3210_turn_onoff_als(0);
	}

	return 0;
}

static int stk3210_resume(struct i2c_client *client)
{
	printk("[stk3210] stk3210_resume, psensor:%d, als:%d, interrupt bit(%d)\n",
		g_stk3210_data_ps->Device_ps_switch_on, g_stk3210_data_as->Device_als_switch_on, gpio_get_value(STK_GPIO_PROXIMITY_INT));
	if (g_stk3210_data_as->Device_als_switch_on == 0 && g_stk3210_data_as->HAL_als_switch_on == 1)
		stk3210_turn_onoff_als(0);
	disable_irq_wake(asus_ir_sensor_irq_gpio);
	return 0;
}

/************************************************************************************
 *---I2C driver part---
 */
const struct i2c_device_id stk3210_id[] = {
    {"stk3210", 0},
    {}
};

MODULE_DEVICE_TABLE(i2c, stk3210_id);

/*
static struct of_device_id stk3210_match_table[] = {
	{ .compatible = "SensorTek,stk3210",},
	{ },
};
*/

static int stk3210_suspend_noirq(struct device *dev)
{
/*
	printk("[stk3210][suspend_noirq] g_earlysuspend_int = %d\n",  g_stk3210_earlysuspend_int);
	if (g_stk3210_earlysuspend_int == 1) {
		g_stk3210_earlysuspend_int = 0;
		return -EBUSY;
	}
*/
	return 0;
}

static int stk3210_resume_noirq(struct device *dev)
{
	return 0;
}

MODULE_DEVICE_TABLE(of, gpio_keys_of_match);

static const struct dev_pm_ops stk3210_pm_ops = {
	.suspend_noirq  = stk3210_suspend_noirq,
	.resume_noirq = stk3210_resume_noirq,
};

static struct i2c_driver stk3210_driver = {
	.driver = {
		.name = "stk3210",
		.owner  = THIS_MODULE,
		.pm = &stk3210_pm_ops,
		/*.of_match_table = stk3210_match_table,*/
	},
	.probe = stk3210_probe,
	.remove = stk3210_remove,
	.shutdown = stk3210_shutdown,
	.suspend = stk3210_suspend,
	.resume = stk3210_resume,
	.id_table = stk3210_id,
};

/************************************************************************************
 *---Platform driver part---
 */
static int stk3210_platform_probe(struct platform_device *pdev)
{
	int err = 0;
	printk("[stk3210] stk3210_platform_probe ++ \n");

	err = i2c_add_driver(&stk3210_driver);
	if (err != 0)
		printk("[stk3210] i2c_add_driver fail: stk3210_als_conf_driver, Error : %d\n", err);

	printk("[stk3210] stk3210_platform_probe -- \n");
	return 0;

}

static int stk3210_platform_remove(struct platform_device *pdev)
{
	i2c_del_driver(&stk3210_driver);

	proximity_dev_unregister(&stk3210_proxmDev);
	proximity_dev_unregister(&stk3210_ambientDev);
	destroy_workqueue(stk3210_workqueue);

	return 0;
}
/*
static int stk3210_platform_suspend_noirq(struct device *dev)
{
	printk("[stk3210][suspend_noirq] g_earlysuspend_int = %d\n",  g_stk3210_earlysuspend_int);
	if (g_stk3210_earlysuspend_int == 1) {
		g_stk3210_earlysuspend_int = 0;
		return -EBUSY;
	}
	return 0;
}

static int stk3210_platform_resume_noirq(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops stk3210_pm_ops = {
	.suspend_noirq  = stk3210_platform_suspend_noirq,
	.resume_noirq = stk3210_platform_resume_noirq,
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);
*/

static struct platform_driver  stk3210_platform_driver = {
	.probe 	= stk3210_platform_probe,
	.remove   = stk3210_platform_remove,

	.driver = {
		.name = "stk3210",
		.owner = THIS_MODULE,
		/*.pm = &stk3210_pm_ops,*/
	},
};

static int __init stk3210_init(void)
{
	int err = 0;

	printk("[stk3210] stk3210_platform_init +.\n");
	err = i2c_add_driver(&stk3210_driver);
	if (err != 0)
		printk("[stk3210] platform_driver_register fail, Error : %d\n", err);
#if 0
	err = platform_driver_register(&stk3210_platform_driver);
	if (err != 0)
		printk("[stk3210] platform_driver_register fail, Error : %d\n", err);
#endif
	printk("[stk3210] stk3210_platform_init -.\n");

	return err;
}

static void __exit stk3210_exit(void)
{
	printk("[stk3210] stk3210_platform_exit +.\n");
	platform_driver_unregister(&stk3210_platform_driver);
	printk("[stk3210] stk3210_platform_exit -.\n");
}

module_init(stk3210_init);
module_exit(stk3210_exit);

MODULE_AUTHOR("ASUS");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SensorTek STK3210 ALS/Proximity sensor");
