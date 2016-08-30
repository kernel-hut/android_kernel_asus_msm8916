/*
 * Copyright (C) 2011 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/sensors.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
//+++ ASUS_BSP Alian_Shen "add GSensor for ZC550KL"
//#include <linux/input/kxtj9.h>
#include <linux/kxtj9_zc550kl.h>
//--- ASUS_BSP Alian_Shen "add GSensor for ZC550KL"
#include <linux/input-polldev.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
//+++ ASUS_BSP Alian_Shen
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/time.h>  
//--- ASUS_BSP Alian_Shen

// ASUS_BSP +++ guochang_qiu "[ZC550KL][Sensor][NA][Spec] support I2C stress test"
#ifdef CONFIG_I2C_STRESS_TEST
#include <linux/i2c_testcase.h>
#endif
// ASUS_BSP --- Guochang_qiu "[ZC550KL][Sensor][NA][Spec] support I2C stress test"


#define ACCEL_INPUT_DEV_NAME    "accelerometer"
#define DEVICE_NAME     "kxtj9"
//+++ ASUS_BSP Alian_Shen "add function for 'shake-shake' "
#define SHAKE_INPUT_DEV_NAME  "shake"
//--- ASUS_BSP Alian_Shen "add function for 'shake-shake' "

#define G_MAX           8000
/* OUTPUT REGISTERS */
#define XOUT_L          0x06
#define WHO_AM_I        0x0F
/* CONTROL REGISTERS */
#define INT_REL         0x1A
#define CTRL_REG1       0x1B
#define INT_CTRL1       0x1E
#define DATA_CTRL       0x21
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF         0x7F

//+++ ASUS_BSP Alian_Shen "add shake-shake gesture "
#define KXTT9_INT_SOURCE_1                 0x16
#define KXTT9_INT_SOURCE_2                 0x17
#define KXTT9_CTL_REG_1                       0x1B
#define KXTT9_CTL_REG_2                       0x1D
#define KXTT9_INT_CTL_REG_1               0x1E
#define KXTT9_INT_CTL_REG_2               0x1F
#define KXTJ9_WAKEUP_TIMER                0x29
#define KXTJ9_WAKEUP_THRESHOLD       0x6A

#define KXTJ9_XNWU_BIT                        (1 << 5)
#define KXTJ9_XPWU_BIT                        (1 << 4)
#define KXTJ9_YNWU_BIT                        (1 << 3)
#define KXTJ9_YPWU_BIT                        (1 << 2)
#define KXTJ9_ZNWU_BIT                        (1 << 1)
#define KXTJ9_ZPWU_BIT                        (1 << 0)
//--- ASUS_BSP Alian_Shen "add shake-shake gesture "

#define PC1_ON          (1 << 7)
/* Data ready funtion enable bit: set during probe if using irq mode */
#define DRDYE           (1 << 5)
/* DATA CONTROL REGISTER BITS */
//+++ ASUS_BSP Alian_Shen
/*
#define ODR12_5F        0
#define ODR25F          1
#define ODR50F          2
#define ODR100F     3
#define ODR200F     4
#define ODR400F     5
#define ODR800F     6
*/
#define ODR0_781F   0x08
#define ODR1_563F   0x09
#define ODR3_125F   0x0A
#define ODR6_25F     0x0B
#define ODR12_5F     0
#define ODR25F         1
#define ODR50F         2
#define ODR100F       3
#define ODR200F       4
#define ODR400F       5
#define ODR800F       6
//--- ASUS_BSP Alian_Shen

/* INTERRUPT CONTROL REGISTER 1 BITS */
/* Set these during probe if using irq mode */
#define KXTJ9_IEL       (1 << 3)
#define KXTJ9_IEA       (1 << 4)
#define KXTJ9_IEN       (1 << 5)
/* INPUT_ABS CONSTANTS */
//#define FUZZ          3
//#define FLAT          3
#define FUZZ            0
#define FLAT            0

/* RESUME STATE INDICES */
#define RES_DATA_CTRL       0
#define RES_CTRL_REG1       1
#define RES_INT_CTRL1       2
#define RESUME_ENTRIES      3
/* POWER SUPPLY VOLTAGE RANGE */
#define KXTJ9_VDD_MIN_UV    2000000
#define KXTJ9_VDD_MAX_UV    3300000
#define KXTJ9_VIO_MIN_UV    1750000
#define KXTJ9_VIO_MAX_UV    1950000

/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */

static struct sensors_classdev sensors_cdev = {
    .name = "kxtj9-accel",
    .vendor = "Kionix",
    .version = 1,
    .handle = 0,
    .type = 1,
    .max_range = "39.2",
    .resolution = "0.01",
    .sensor_power = "0.2",
    .min_delay = 2000,  /* microsecond */
    .fifo_reserved_event_count = 0,
    .fifo_max_event_count = 0,
    .enabled = 0,
    .delay_msec = 200,  /* millisecond */
    .sensors_enable = NULL,
    .sensors_poll_delay = NULL,
};

static const struct {
    unsigned int cutoff;
    u8 mask;
} kxtj9_odr_table[] = {
/*
    { 3,    ODR800F },
    { 5,    ODR400F },
    { 10,   ODR200F },
    { 20,   ODR100F },
    { 40,   ODR50F  },
    { 80,   ODR25F  },
    { 0,    ODR12_5F},
*/
    //+++ ASUS_BSP Alian_Shen
     { 2,    ODR800F },
        { 3,    ODR400F },
        { 5,    ODR200F },
        { 10,  ODR100F },
        { 20,  ODR50F  },
        { 40,  ODR25F  },
        { 80,  ODR12_5F},
        { 160, ODR6_25F},
        { 320, ODR3_125F},
        { 640, ODR1_563F},
        { 1600,ODR0_781F},
    //--- ASUS_BSP Alian_Shen        
};

struct kxtj9_data {
    struct i2c_client *client;
    struct kxtj9_platform_data pdata;
    struct input_dev *input_dev;
#ifdef CONFIG_INPUT_KXTJ9_POLLED_MODE
    struct input_polled_dev *poll_dev;
#endif
    unsigned int last_poll_interval;
    bool    enable;
    u8 shift;
    u8 ctrl_reg1;
    u8 data_ctrl;
    u8 int_ctrl;
    bool    power_enabled;
    struct regulator *vdd;
    struct regulator *vio;
    struct sensors_classdev cdev;
//+++ ASUS Alian_Shen "add function for 'shake-shake'"
    struct input_dev *dev_interrupt;
//--- ASUS Alian_Shen "add function for 'shake-shake'"
};

//+++ ASUS_BSP Alian_Shen "add shake-shake gesture "
#define TIME_WAIT_SECOND_SHAKE_MS                800
#define TIME_MOTION_DETECT_POLL_TIME_ms     10
#define MOTION_DETECT_DATA_BUF_SIZE             ((TIME_WAIT_SECOND_SHAKE_MS/TIME_MOTION_DETECT_POLL_TIME_ms) << 2)
#define RDY_DATA_BUF_NUM                                  (2)
#define MAX_ORIENTATION_NUM                             3
#define FLICK_STAGE_1_REPEAT_MAX_NUM           2
#define FLICK_STAGE_2_REPEAT_MAX_NUM           13
#define FLICK_STAGE_3_REPEAT_MAX_NUM           2
#define FLICK_STAGE_4_REPEAT_MAX_NUM           13

#define FLICK_STAGE_Z_1_REPEAT_MAX_NUM        2
#define FLICK_STAGE_Z_2_REPEAT_MAX_NUM        12
#define FLICK_STAGE_Z_3_REPEAT_MAX_NUM        2
#define FLICK_STAGE_Z_4_REPEAT_MAX_NUM        11
#define Z_SHAKE_NUM                                              2

#define FLICK_G_DATA_MAX                                    1900
#define FLICK_STAGW_1_G_DATA_MAX                   900
#define FLICK_STAGW_2_G_DATA_MAX                   2048
#define FLICK_STAGW_3_G_DATA_MAX                   2048
#define FLICK_STAGW_4_G_DATA_MAX                   2048

#define FLICK_STAGW_Z_1_G_DATA_MAX               900
#define FLICK_STAGW_Z_2_G_DATA_MAX               1800
#define FLICK_STAGW_Z_3_G_DATA_MAX               2048
#define FLICK_STAGW_Z_4_G_DATA_MAX               1800


#define ORIEN_X                                                 0
#define ORIEN_Y                                                 1
#define ORIEN_Z                                                 2
static struct workqueue_struct *kxtj9_wakeup_work = NULL;
static struct workqueue_struct *kxtj9_read_work = NULL;
struct kxtj9_wake_up_work_t {
    struct kxtj9_data   *tj9;
    struct timer_list       timer;
    struct timer_list       poll_timer;
    struct timeval          tv1;
    int                           shake_num[3];
    int                           reverse_num[3];
    char                        orientation[3]; //- x y z
    struct work_struct   wakeup_work;
    int                          xyz;
    int                          flickDataNum;
} tj9_wake_up_data,  tj9_read_data;

struct XYZ_Data
{
    s16 x;
    s16 y;
    s16 z;
};

static char g_cFlickOrientationBuff[MAX_ORIENTATION_NUM][MOTION_DETECT_DATA_BUF_SIZE];
static s16 g_cFlickDataBuff[MOTION_DETECT_DATA_BUF_SIZE][3];
static s16 g_cFlickDataRdyBuff[RDY_DATA_BUF_NUM][3];

static struct kxtj9_data *g_kxtj9p = NULL;
//--- ASUS_BSP Alian_Shen "add shake-shake ,gesture "
static bool g_bIsStop = false;
static bool g_bIsFirstMotion = true;


static int kxtj9_i2c_read(struct kxtj9_data *tj9, u8 addr, u8 *data, int len)
{
    struct i2c_msg msgs[] = {
        {
            .addr = tj9->client->addr,
            .flags = tj9->client->flags,
            .len = 1,
            .buf = &addr,
        },
        {
            .addr = tj9->client->addr,
            .flags = tj9->client->flags | I2C_M_RD,
            .len = len,
            .buf = data,
        },
    };

    return i2c_transfer(tj9->client->adapter, msgs, 2);
}

static void kxtj9_report_acceleration_data(struct kxtj9_data *tj9)
{
    s16 acc_data[3]; /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
    s16 x, y, z;
    int err;
    static int iDataNum = 0;
    static int iDias = 0;
//+++ ASUS_BSP Alian_Shen "input: sensors: send boot time alone with sensor events"
    ktime_t timestamp;

    timestamp = ktime_get_boottime();
//--- ASUS_BSP Alian_Shen

    err = kxtj9_i2c_read(tj9, XOUT_L, (u8 *)acc_data, 6);
    if (err < 0)
        dev_err(&tj9->client->dev, "accelerometer data read failed\n");

    x = le16_to_cpu(acc_data[tj9->pdata.axis_map_x]);
    y = le16_to_cpu(acc_data[tj9->pdata.axis_map_y]);
    z = le16_to_cpu(acc_data[tj9->pdata.axis_map_z]);

    /* 8 bits output mode support */
    if (!(tj9->ctrl_reg1 & RES_12BIT)) {
        x <<= 4;
        y <<= 4;
        z <<= 4;
    }

    x >>= tj9->shift;
    y >>= tj9->shift;
    z >>= tj9->shift;   

    x += iDias;
    y += iDias;
    z += iDias;
    iDias = iDias ? 0 : 1;

    input_report_abs(tj9->input_dev, ABS_X, tj9->pdata.negate_x ? -x : x);
    input_report_abs(tj9->input_dev, ABS_Y, tj9->pdata.negate_y ? -y : y);
    input_report_abs(tj9->input_dev, ABS_Z, tj9->pdata.negate_z ? -z : z);
//+++ ASUS_BSP Alian_Shen "input: sensors: send boot time alone with sensor events"
    input_event(tj9->input_dev, EV_SYN, SYN_TIME_SEC,
        ktime_to_timespec(timestamp).tv_sec);
    input_event(tj9->input_dev, EV_SYN, SYN_TIME_NSEC,
        ktime_to_timespec(timestamp).tv_nsec);
//--- ASUS_BSP Alian_Shen
    input_sync(tj9->input_dev);

    x = tj9->pdata.negate_x ? -x : x;
    y = tj9->pdata.negate_y ? -y : y;
    z = tj9->pdata.negate_z ? -z : z;
    
    g_cFlickDataRdyBuff[iDataNum][ORIEN_X] = x;
    g_cFlickDataRdyBuff[iDataNum][ORIEN_Y] = y;
    g_cFlickDataRdyBuff[iDataNum][ORIEN_Z] = z;

    iDataNum++;
    if (iDataNum >= RDY_DATA_BUF_NUM)
        iDataNum = 0;

/*
    printk("[kxtj9] acc data: %d, %d, %d\n", 
        (tj9->pdata.negate_x ? -x : x),
        (tj9->pdata.negate_y ? -y : y),
        (tj9->pdata.negate_z ? -z : z) );
*/      
}


static void kxtj9_read_acceleration_data(struct kxtj9_data *tj9)
{
    s16 acc_data[3]; /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
    s16 x, y, z;
    int err;
    
    err = kxtj9_i2c_read(tj9, XOUT_L, (u8 *)acc_data, 6);
    if (err < 0)
        dev_err(&tj9->client->dev, "accelerometer data read failed\n");

    x = le16_to_cpu(acc_data[tj9->pdata.axis_map_x]);
    y = le16_to_cpu(acc_data[tj9->pdata.axis_map_y]);
    z = le16_to_cpu(acc_data[tj9->pdata.axis_map_z]);

    /* 8 bits output mode support */
    if (!(tj9->ctrl_reg1 & RES_12BIT)) {
        x <<= 4;
        y <<= 4;
        z <<= 4;
    }

    x >>= tj9->shift;
    y >>= tj9->shift;
    z >>= tj9->shift;

    
    x = (tj9->pdata.negate_x ? -x : x);
    y = (tj9->pdata.negate_y ? -y : y);
    z = (tj9->pdata.negate_z ? -z : z);

            
    g_cFlickDataBuff[tj9_read_data.flickDataNum][ORIEN_X]= x;
    g_cFlickDataBuff[tj9_read_data.flickDataNum][ORIEN_Y]= y;
    g_cFlickDataBuff[tj9_read_data.flickDataNum][ORIEN_Z]= z;   
    tj9_read_data.flickDataNum++;
    /*
    printk("[kxtj9] acc data: %d, %d, %d | x = %c y = %c z = %c\n", 
        (tj9->pdata.negate_x ? -x : x),
        (tj9->pdata.negate_y ? -y : y),
        (tj9->pdata.negate_z ? -z : z),

        ((tj9->pdata.negate_x ? -x : x) > 0 ? '+' : '-'), 
        ((tj9->pdata.negate_y ? -y : y) > 0 ? '+' : '-'), 
        ((tj9->pdata.negate_z ? -z : z) > 0 ? '+' : '-'));      
        */
}

static irqreturn_t kxtj9_isr(int irq, void *dev)
{
    struct kxtj9_data *tj9 = dev;
    int err;
    int int_src1_reg = 0;
    int int_src_reg = 0;
    //static char s_cLastOrientation = 'X';
    char orien = 'X';
    int    xyz = 0;
    

 /*
    int_src_reg = i2c_smbus_read_byte_data(tj9->client, KXTT9_CTL_REG_1);
    printk("[alian][kxtj9_isr] KXTT9_CTL_REG_1 = %X\n", int_src_reg);
    int_src_reg = i2c_smbus_read_byte_data(tj9->client, KXTT9_CTL_REG_2);
    printk("[alian][kxtj9_isr] KXTT9_CTL_REG_2 = %X\n", int_src_reg);
    int_src_reg = i2c_smbus_read_byte_data(tj9->client, KXTT9_INT_CTL_REG_1);
    printk("[alian][kxtj9_isr] KXTT9_INT_CTL_REG_1 = %X\n", int_src_reg);
    int_src_reg = i2c_smbus_read_byte_data(tj9->client, KXTT9_INT_CTL_REG_2);
    printk("[alian][kxtj9_isr] KXTT9_INT_CTL_REG_2 = %X\n", int_src_reg);
    int_src_reg = i2c_smbus_read_byte_data(tj9->client, KXTJ9_WAKEUP_TIMER);
    printk("[alian][kxtj9_isr] KXTJ9_WAKEUP_TIMER = %X\n", int_src_reg);
    int_src_reg = i2c_smbus_read_byte_data(tj9->client, KXTJ9_WAKEUP_THRESHOLD);
    printk("[alian][kxtj9_isr] KXTJ9_WAKEUP_THRESHOLD = %X\n\n", int_src_reg);  
    int_src_reg = i2c_smbus_read_byte_data(tj9->client, CTRL_REG1);
    printk("[alian][kxtj9_isr] CTRL_REG1 = %X\n\n", int_src_reg);   
    //int_src_reg = i2c_smbus_read_byte_data(tj9->client, DATA_CTRL);
    //printk("[alian][kxtj9_isr] DATA_CTRL = %X\n\n", int_src_reg); 
*/
    //int_src_reg = i2c_smbus_read_byte_data(tj9->client, KXTT9_CTL_REG_2);
    //printk("[alian][kxtj9_isr] KXTT9_CTL_REG_2 = %X\n\n", int_src_reg);   

    
    int_src1_reg = i2c_smbus_read_byte_data(tj9->client, KXTT9_INT_SOURCE_1);
    if (int_src1_reg & (0x1 << 1)) {    
        int_src_reg = i2c_smbus_read_byte_data(tj9->client, KXTT9_INT_SOURCE_2);

        printk("[kxtj9] motion detect interupt had happened\n");
        
        if (int_src_reg & KXTJ9_XNWU_BIT) {
            orien = '-';
            xyz = ORIEN_X;
            //printk("[kxtj9] xyz=%d, orien=%c\n", xyz, orien);
        }
        if (int_src_reg & KXTJ9_XPWU_BIT) { 
            orien = '+';
            xyz = ORIEN_X;
            //printk("[kxtj9] xyz=%d, orien=%c\n", xyz, orien);
        } 
        
        if (int_src_reg & KXTJ9_YNWU_BIT) {
            orien = '-';
            xyz = ORIEN_Y;
            //printk("[kxtj9] xyz=%d, orien=%c\n", xyz, orien);
        }
        if (int_src_reg & KXTJ9_YPWU_BIT) { 
            orien = '+';
            xyz = ORIEN_Y;
            //printk("[kxtj9] xyz=%d, orien=%c\n", xyz, orien);
        } 
        if (int_src_reg & KXTJ9_ZNWU_BIT) {
            orien = '-';
            xyz = ORIEN_Z;
            //printk("[kxtj9] xyz=%d, orien=%c\n", xyz, orien);
        }
        if (int_src_reg & KXTJ9_ZPWU_BIT) { 
            orien = '+';
            xyz = ORIEN_Z;
            //printk("[kxtj9] xyz=%d, orien=%c\n", xyz, orien);
        }
        //kxtj9_read_acceleration_data(tj9);
    //  printk("[kxtj9] end ------------\n");
        
        tj9_wake_up_data.orientation[xyz] = orien;  
        tj9_wake_up_data.shake_num[xyz]++;  
        g_cFlickOrientationBuff[xyz][tj9_wake_up_data.shake_num[xyz]-1] = orien;

        tj9_wake_up_data.xyz = xyz;
        //s_cLastOrientation = tj9_wake_up_data.orientation;

        //tj9_wake_up_data.shake_num++; 
        tj9_wake_up_data.tj9 = tj9;

        if (g_bIsFirstMotion) {
            queue_work(kxtj9_wakeup_work, &(tj9_wake_up_data.wakeup_work));
        
        }
    } 

    if (int_src1_reg & (0x1 << 4)) {
        /* data ready is the only possible interrupt type */
        kxtj9_report_acceleration_data(tj9);
    }

    /* data ready is the only possible interrupt type */
    //kxtj9_report_acceleration_data(tj9);

    err = i2c_smbus_read_byte_data(tj9->client, INT_REL);
    
    if (err < 0)
        dev_err(&tj9->client->dev,
            "error clearing interrupt status: %d\n", err);
    return IRQ_HANDLED;
}

static int kxtj9_update_g_range(struct kxtj9_data *tj9, u8 new_g_range)
{
    switch (new_g_range) {
    case KXTJ9_G_2G:
        tj9->shift = 4;
        break;
    case KXTJ9_G_4G:
        tj9->shift = 3;
        break;
    case KXTJ9_G_8G:
        tj9->shift = 2;
        break;
    default:
        return -EINVAL;
    }

    tj9->ctrl_reg1 &= 0xe7;
    tj9->ctrl_reg1 |= new_g_range;

    return 0;
}

static int kxtj9_update_odr(struct kxtj9_data *tj9, unsigned int poll_interval)
{
    int err;
    int i = 0;

    /* Use the lowest ODR that can support the requested poll interval */
    //+++ Alian_Shen
    /*
    for (i = 0; i < ARRAY_SIZE(kxtj9_odr_table); i++) {
        tj9->data_ctrl = kxtj9_odr_table[i].mask;       
        if (poll_interval < kxtj9_odr_table[i].cutoff)
            break;
    }
    */
    for (i = (ARRAY_SIZE(kxtj9_odr_table)-1);i>=0; i--) {
        tj9->data_ctrl = kxtj9_odr_table[i].mask;       
        if (poll_interval > kxtj9_odr_table[i].cutoff)
            break;
    }
    //--- Alian_Shen    

    printk("[alian][L%d]kxtj9_update_odr: poll_interval=%d, data_ctrl=%d, i=%d\n", __LINE__, poll_interval, tj9->data_ctrl, i);

    err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
    if (err < 0)
        return err;

    err = i2c_smbus_write_byte_data(tj9->client, DATA_CTRL, tj9->data_ctrl);
    if (err < 0)
        return err;

    err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
    if (err < 0)
        return err;

    return 0;
}

static int kxtj9_power_on(struct kxtj9_data *data, bool on)
{
    int rc = 0;

    if (!on && data->power_enabled) {
        rc = regulator_disable(data->vdd);
        if (rc) {
            dev_err(&data->client->dev,
                "Regulator vdd disable failed rc=%d\n", rc);
            return rc;
        }

        rc = regulator_disable(data->vio);
        if (rc) {
            dev_err(&data->client->dev,
                "Regulator vio disable failed rc=%d\n", rc);
            rc = regulator_enable(data->vdd);
            if (rc) {
                dev_err(&data->client->dev,
                    "Regulator vdd enable failed rc=%d\n",
                    rc);
            }
        }
        data->power_enabled = false;
    } else if (on && !data->power_enabled) {
        rc = regulator_enable(data->vdd);
        if (rc) {
            dev_err(&data->client->dev,
                "Regulator vdd enable failed rc=%d\n", rc);
            return rc;
        }

        rc = regulator_enable(data->vio);
        if (rc) {
            dev_err(&data->client->dev,
                "Regulator vio enable failed rc=%d\n", rc);
            regulator_disable(data->vdd);
        }
        data->power_enabled = true;
    } else {
        dev_warn(&data->client->dev,
                "Power on=%d. enabled=%d\n",
                on, data->power_enabled);
    }

    return rc;
}

static int kxtj9_power_init(struct kxtj9_data *data, bool on)
{
    int rc;

    if (!on) {
        if (regulator_count_voltages(data->vdd) > 0)
            regulator_set_voltage(data->vdd, 0, KXTJ9_VDD_MAX_UV);

        regulator_put(data->vdd);

        if (regulator_count_voltages(data->vio) > 0)
            regulator_set_voltage(data->vio, 0, KXTJ9_VIO_MAX_UV);

        regulator_put(data->vio);
    } else {
        data->vdd = regulator_get(&data->client->dev, "vdd");
        if (IS_ERR(data->vdd)) {
            rc = PTR_ERR(data->vdd);
            dev_err(&data->client->dev,
                "Regulator get failed vdd rc=%d\n", rc);
            return rc;
        }

        if (regulator_count_voltages(data->vdd) > 0) {
            rc = regulator_set_voltage(data->vdd, KXTJ9_VDD_MIN_UV,
                           KXTJ9_VDD_MAX_UV);
            if (rc) {
                dev_err(&data->client->dev,
                    "Regulator set failed vdd rc=%d\n",
                    rc);
                goto reg_vdd_put;
            }
        }

        data->vio = regulator_get(&data->client->dev, "vio");
        if (IS_ERR(data->vio)) {
            rc = PTR_ERR(data->vio);
            dev_err(&data->client->dev,
                "Regulator get failed vio rc=%d\n", rc);
            goto reg_vdd_set;
        }

        if (regulator_count_voltages(data->vio) > 0) {
            rc = regulator_set_voltage(data->vio, KXTJ9_VIO_MIN_UV,
                           KXTJ9_VIO_MAX_UV);
            if (rc) {
                dev_err(&data->client->dev,
                "Regulator set failed vio rc=%d\n", rc);
                goto reg_vio_put;
            }
        }
    }

    return 0;

reg_vio_put:
    regulator_put(data->vio);
reg_vdd_set:
    if (regulator_count_voltages(data->vdd) > 0)
        regulator_set_voltage(data->vdd, 0, KXTJ9_VDD_MAX_UV);
reg_vdd_put:
    regulator_put(data->vdd);
    return rc;
}
static int kxtj9_device_power_on(struct kxtj9_data *tj9)
{
    int err = 0;
    if (tj9->pdata.power_on) {
        err = tj9->pdata.power_on();
    } else {
        err = kxtj9_power_on(tj9, true);
        if (err) {
            dev_err(&tj9->client->dev, "power on failed");
            goto err_exit;
        }
        /* Use 80ms as vendor suggested. */
        msleep(80);
    }

err_exit:
    dev_dbg(&tj9->client->dev, "soft power on complete err=%d.\n", err);
    return err;
}

static void kxtj9_device_power_off(struct kxtj9_data *tj9)
{
    int err;

    tj9->ctrl_reg1 &= PC1_OFF;
    err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
    if (err < 0)
        dev_err(&tj9->client->dev, "soft power off failed\n");

    if (tj9->pdata.power_off)
        tj9->pdata.power_off();
    else
        kxtj9_power_on(tj9, false);

    dev_dbg(&tj9->client->dev, "soft power off complete.\n");
    return ;
}

static int kxtj9_enable(struct kxtj9_data *tj9)
{
    int err;

    err = kxtj9_device_power_on(tj9);
    if (err < 0)
        return err;

    /* ensure that PC1 is cleared before updating control registers */
    //+++ ASUS Alian_Shen 
    err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
    //tj9->ctrl_reg1 &= ~PC1_ON;
    //err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
    //+++ ASUS Alian_Shen 
    
    if (err < 0)
        return err;

    /* only write INT_CTRL_REG1 if in irq mode */
    if (tj9->client->irq) {
        err = i2c_smbus_write_byte_data(tj9->client,
                        INT_CTRL1, tj9->int_ctrl);
        if (err < 0)
            return err;
    }

    err = kxtj9_update_g_range(tj9, tj9->pdata.g_range);
    if (err < 0)
        return err;

    /* turn on outputs */
    tj9->ctrl_reg1 |= PC1_ON;
    err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
    if (err < 0)
        return err;

    err = kxtj9_update_odr(tj9, tj9->last_poll_interval);
    if (err < 0)
        return err;

    /* clear initial interrupt if in irq mode */
    if (tj9->client->irq) {
        err = i2c_smbus_read_byte_data(tj9->client, INT_REL);
        if (err < 0) {
            dev_err(&tj9->client->dev,
                "error clearing interrupt: %d\n", err);
            goto fail;
        }
    }

    return 0;

fail:
    kxtj9_device_power_off(tj9);
    return err;
}

static void kxtj9_disable(struct kxtj9_data *tj9)
{
    kxtj9_device_power_off(tj9);
}


static void kxtj9_init_input_device(struct kxtj9_data *tj9,
                          struct input_dev *input_dev)
{
    __set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
    input_set_abs_params(input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
    input_set_abs_params(input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

    input_dev->name = ACCEL_INPUT_DEV_NAME;
    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = &tj9->client->dev;
}

static int kxtj9_setup_input_device(struct kxtj9_data *tj9)
{
    struct input_dev *input_dev;
    int err;
//+++ ASUS_BSP Alian_Shen "add function for 'shake-shake' "
    struct input_dev *dev_interrupt;
//--- ASUS_BSP Alian_Shen "add function for 'shake-shake' "

    input_dev = input_allocate_device();
    if (!input_dev) {
        dev_err(&tj9->client->dev, "input device allocate failed\n");
        return -ENOMEM;
    }   

    tj9->input_dev = input_dev;

    input_set_drvdata(input_dev, tj9);

    kxtj9_init_input_device(tj9, input_dev);

    err = input_register_device(tj9->input_dev);
    if (err) {
        dev_err(&tj9->client->dev,
            "unable to register input polled device %s: %d\n",
            tj9->input_dev->name, err);
        input_free_device(tj9->input_dev);
        return err;
    }

//+++ ASUS_BSP Alian_Shen "add function for 'shake-shake' "
    dev_interrupt = input_allocate_device();
    if (!dev_interrupt) {
        input_free_device(tj9->input_dev);/*free the successful dev and return*/
        return -ENOMEM;
    }   

    tj9->dev_interrupt = dev_interrupt;
    
    input_set_drvdata(dev_interrupt, tj9);

    __set_bit(EV_REL, dev_interrupt->evbit);
    input_set_capability(dev_interrupt, EV_REL, REL_DIAL);

    dev_interrupt->name = SHAKE_INPUT_DEV_NAME;
    dev_interrupt->id.bustype = BUS_I2C;
    dev_interrupt->dev.parent = &tj9->client->dev;

    err = input_register_device(tj9->dev_interrupt);
    if (err) {
        dev_err(&tj9->client->dev,
            "unable to register input polled device %s: %d\n",
            tj9->dev_interrupt->name, err);
        input_free_device(tj9->input_dev);
        input_free_device(tj9->dev_interrupt);
        return err;
    }
//--- ASUS_BSP Alian_Shen "add function for 'shake-shake' "

    return 0;
}

static int kxtj9_enable_set(struct sensors_classdev *sensors_cdev,
                    unsigned int enabled)
{
    struct kxtj9_data *tj9 = container_of(sensors_cdev,
                    struct kxtj9_data, cdev);
    struct input_dev *input_dev = tj9->input_dev;

    printk("[kxtj9] enable(%d)\n", enabled);
    
    mutex_lock(&input_dev->mutex);

    if (enabled == 0) {
        disable_irq(tj9->client->irq);
        kxtj9_disable(tj9);
        tj9->enable = false;
    } else if (enabled == 1) {
        if (!kxtj9_enable(tj9)) {
            enable_irq(tj9->client->irq);
            tj9->enable = true;
        }
    } else {
        dev_err(&tj9->client->dev,
            "Invalid value of input, input=%d\n", enabled);
        mutex_unlock(&input_dev->mutex);
        return -EINVAL;
    }

    mutex_unlock(&input_dev->mutex);

    return 0;
}

static ssize_t kxtj9_enable_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kxtj9_data *tj9 = i2c_get_clientdata(client);

    return snprintf(buf, 4, "%d\n", tj9->enable);
}

static ssize_t kxtj9_enable_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kxtj9_data *tj9 = i2c_get_clientdata(client);
    unsigned long data;
    int error;

    error = kstrtoul(buf, 10, &data);
    if (error < 0)
        return error;

    error = kxtj9_enable_set(&tj9->cdev, data);
    if (error < 0)
        return error;
    return count;
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
            kxtj9_enable_show, kxtj9_enable_store);

/*
 * When IRQ mode is selected, we need to provide an interface to allow the user
 * to change the output data rate of the part.  For consistency, we are using
 * the set_poll method, which accepts a poll interval in milliseconds, and then
 * calls update_odr() while passing this value as an argument.  In IRQ mode, the
 * data outputs will not be read AT the requested poll interval, rather, the
 * lowest ODR that can support the requested interval.  The client application
 * will be responsible for retrieving data from the input node at the desired
 * interval.
 */
static int kxtj9_poll_delay_set(struct sensors_classdev *sensors_cdev,
                    unsigned int delay_msec)
{
    struct kxtj9_data *tj9 = container_of(sensors_cdev,
                    struct kxtj9_data, cdev);
    struct input_dev *input_dev = tj9->input_dev;

    printk("[kxtj9][L%d]kxtj9_poll_delay_set(delay=%u)\n", __LINE__, delay_msec);
    
    /* Lock the device to prevent races with open/close (and itself) */
    mutex_lock(&input_dev->mutex);

    if (tj9->enable)
        disable_irq(tj9->client->irq);

    tj9->last_poll_interval = max(delay_msec, tj9->pdata.min_interval);

    if (tj9->enable) {
        kxtj9_update_odr(tj9, tj9->last_poll_interval);
        enable_irq(tj9->client->irq);
    }
    mutex_unlock(&input_dev->mutex);

    return 0;
}

/* Returns currently selected poll interval (in ms) */
static ssize_t kxtj9_get_poll_delay(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kxtj9_data *tj9 = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", tj9->last_poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kxtj9_set_poll_delay(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kxtj9_data *tj9 = i2c_get_clientdata(client);
    unsigned int interval;
    int error;

    error = kstrtouint(buf, 10, &interval);
    if (error < 0)
        return error;

    //printk("[kxtj9][L%d] kxtj9_set_poll_delay: ap set = %d  we must set = %d", __LINE__,  interval, 90);
    error = kxtj9_poll_delay_set(&tj9->cdev, interval);
    //error = kxtj9_poll_delay_set(&tj9->cdev, 90); 
    if (error < 0)
        return error;

    error = i2c_smbus_read_byte_data(tj9->client, KXTT9_INT_SOURCE_1);
    printk("[alian][kxtj9_isr][L%d] KXTT9_INT_SOURCE_1 = %d", __LINE__, error);
    
    return count;
}

static DEVICE_ATTR(poll_delay, S_IRUGO|S_IWUSR|S_IWGRP,
            kxtj9_get_poll_delay, kxtj9_set_poll_delay);

static struct attribute *kxtj9_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_poll_delay.attr,
    NULL
};

static struct attribute_group kxtj9_attribute_group = {
    .attrs = kxtj9_attributes
};

#ifdef CONFIG_INPUT_KXTJ9_POLLED_MODE
static void kxtj9_poll(struct input_polled_dev *dev)
{
    struct kxtj9_data *tj9 = dev->private;
    unsigned int poll_interval = dev->poll_interval;

    kxtj9_report_acceleration_data(tj9);

    if (poll_interval != tj9->last_poll_interval) {
        kxtj9_update_odr(tj9, poll_interval);
        tj9->last_poll_interval = poll_interval;
    }
}

static void kxtj9_polled_input_open(struct input_polled_dev *dev)
{
    struct kxtj9_data *tj9 = dev->private;

    kxtj9_enable(tj9);
}

static void kxtj9_polled_input_close(struct input_polled_dev *dev)
{
    struct kxtj9_data *tj9 = dev->private;

    kxtj9_disable(tj9);
}

static int kxtj9_setup_polled_device(struct kxtj9_data *tj9)
{
    int err;
    struct input_polled_dev *poll_dev;
    poll_dev = input_allocate_polled_device();

    if (!poll_dev) {
        dev_err(&tj9->client->dev,
            "Failed to allocate polled device\n");
        return -ENOMEM;
    }

    tj9->poll_dev = poll_dev;
    tj9->input_dev = poll_dev->input;

    poll_dev->private = tj9;
    poll_dev->poll = kxtj9_poll;
    poll_dev->open = kxtj9_polled_input_open;
    poll_dev->close = kxtj9_polled_input_close;

    kxtj9_init_input_device(tj9, poll_dev->input);

    err = input_register_polled_device(poll_dev);
    if (err) {
        dev_err(&tj9->client->dev,
            "Unable to register polled device, err=%d\n", err);
        input_free_polled_device(poll_dev);
        return err;
    }

    return 0;
}

static void kxtj9_teardown_polled_device(struct kxtj9_data *tj9)
{
    input_unregister_polled_device(tj9->poll_dev);
    input_free_polled_device(tj9->poll_dev);
}

#else

static inline int kxtj9_setup_polled_device(struct kxtj9_data *tj9)
{
    return -ENOSYS;
}

static inline void kxtj9_teardown_polled_device(struct kxtj9_data *tj9)
{
}

#endif

//+++ ASUS_BSP Alian_Shen "add shake-shake gesture "
/**
      * \u8ba1\u7b97\u4e24\u4e2a\u65f6\u95f4\u7684\u95f4\u9694\uff0c\u5f97\u5230\u65f6\u95f4\u5dee
      * @param struct timeval* resule \u8fd4\u56de\u8ba1\u7b97\u51fa\u6765\u7684\u65f6\u95f4
      * @param struct timeval* x \u9700\u8981\u8ba1\u7b97\u7684\u524d\u4e00\u4e2a\u65f6\u95f4
      * @param struct timeval* y \u9700\u8981\u8ba1\u7b97\u7684\u540e\u4e00\u4e2a\u65f6\u95f4
      * return -1 failure ,0 success
  **/
/*
static int timeval_subtract(struct timeval* result, struct timeval* x, struct timeval* y)
  {    
        if ( x->tv_sec>y->tv_sec )
                  return -1;
    
        if ( (x->tv_sec==y->tv_sec) && (x->tv_usec>y->tv_usec) )
                  return -1;
    
        result->tv_sec = ( y->tv_sec-x->tv_sec );
        result->tv_usec = ( y->tv_usec-x->tv_usec );
    
        if (result->tv_usec<0)
        {
                  result->tv_sec--;
                  result->tv_usec+=1000000;
        }
    
        return 0;
  } 
*/

static void kxtj9_wakeup_work_handler(struct work_struct *p_work)
{
    int ret;
    int iNum = 0;
    struct kxtj9_wake_up_work_t *wakeup_work = container_of(p_work, struct kxtj9_wake_up_work_t, wakeup_work);

    for (iNum = 0; iNum < RDY_DATA_BUF_NUM; iNum++) {
        g_cFlickDataBuff[tj9_read_data.flickDataNum][ORIEN_X]= g_cFlickDataRdyBuff[iNum][ORIEN_X];
        g_cFlickDataBuff[tj9_read_data.flickDataNum][ORIEN_Y]= g_cFlickDataRdyBuff[iNum][ORIEN_Y];
        g_cFlickDataBuff[tj9_read_data.flickDataNum][ORIEN_Z]= g_cFlickDataRdyBuff[iNum][ORIEN_Z];

        tj9_read_data.flickDataNum++;
    }
    
    
    
        
    kxtj9_read_acceleration_data(wakeup_work->tj9);

    g_bIsFirstMotion = false;
    g_bIsStop           = false;
    ret = mod_timer(&(wakeup_work->timer), jiffies + msecs_to_jiffies(TIME_WAIT_SECOND_SHAKE_MS));  
    ret = mod_timer(&(wakeup_work->poll_timer), jiffies + msecs_to_jiffies(TIME_MOTION_DETECT_POLL_TIME_ms));       
}

static void kxtj9_read_work_handler(struct work_struct *p_work)
{
    struct kxtj9_wake_up_work_t *wakeup_work = container_of(p_work, struct kxtj9_wake_up_work_t, wakeup_work);
    kxtj9_read_acceleration_data(wakeup_work->tj9);
}

enum STEP_t {
    STEP_1 = 0,
    STEP_2,
    STEP_3,
    STEP_4,
    STEP_OK,
} g_tStage;

static const int g_repeatNumMax[]   = {
    FLICK_STAGE_1_REPEAT_MAX_NUM,
    FLICK_STAGE_2_REPEAT_MAX_NUM,
    FLICK_STAGE_3_REPEAT_MAX_NUM,
    FLICK_STAGE_4_REPEAT_MAX_NUM,
};  

static const int g_Z_repeatNumMax[] = {
    FLICK_STAGE_Z_1_REPEAT_MAX_NUM,
    FLICK_STAGE_Z_2_REPEAT_MAX_NUM,
    FLICK_STAGE_Z_3_REPEAT_MAX_NUM,
    FLICK_STAGE_Z_4_REPEAT_MAX_NUM,
};

static s16 g_data_G[] = {
    FLICK_STAGW_1_G_DATA_MAX,
    FLICK_STAGW_2_G_DATA_MAX,
    FLICK_STAGW_3_G_DATA_MAX,
    FLICK_STAGW_4_G_DATA_MAX,
};


static s16 g_Z_data_G[] = {
    FLICK_STAGW_Z_1_G_DATA_MAX,
    FLICK_STAGW_Z_2_G_DATA_MAX,
    FLICK_STAGW_Z_3_G_DATA_MAX,
    FLICK_STAGW_Z_4_G_DATA_MAX,
};

#define GET_STEP_DATA_MAX(step, isZ)                     (isZ ? g_Z_data_G[step] : g_data_G[step])

/*
//#define CHANGE_NEXT_STAGE(num, max, nowStage, nextstage, backStage)   \
//          ( ( ((num) >= g_repeatNumMax[nowStage]) && ((max) >= FLICK_G_DATA_MAX) )  \
//             ? (nextstage) : (backStage))
*/
#define CHANGE_NEXT_STAGE(num, max, nowStage, nextstage, backStage, zShakeNum)   \
            ( ( ((num) >=   ((zShakeNum > Z_SHAKE_NUM) ? g_Z_repeatNumMax[nowStage]: g_repeatNumMax[nowStage]) ) )  \
               ? (nextstage) : (backStage))

#define ADD_REPEAT_NUM(data, max, num)                             ( ((data) >= (max)) ? ((num)+1) : (num))                 
static enum STEP_t   flickCheckStage(s16 data, bool bIsFirst, bool * bIsErrp, bool bIsZ)
//static enum STEP_t   flickCheckStage(s16 data, bool bIsFirst)
{
    static char s_cOrien;
             char cOrien;
    //static int    s_iReverseNum = 0;
    static int    s_repeatNum = 0;
    static enum STEP_t  tStep = STEP_1;
    static s16   s_dataMax  = 0;
             s16   data_abs = 0;
    //bool bIsZ = tj9_wake_up_data.shake_num[2] > Z_SHAKE_NUM;

    *bIsErrp = false;

    if (bIsFirst) {
        s_cOrien = (data > 0) ? '+' : '-';
        s_repeatNum = 0;
        s_dataMax = 0;

        tStep = STEP_1;
    }

    //printk("flickCheckStage: step%d, data=%i\n", tStep+1, data);
    
    switch(tStep) {
        case STEP_1:
            if (data > 0) {
                data_abs = data;
                cOrien = '+';
            } else {
                data_abs = 0 - data;
                cOrien = '-';
            }
            s_dataMax = (s_dataMax > data_abs) ? s_dataMax : data_abs;
            
            if (cOrien == s_cOrien) {
                s_repeatNum = ADD_REPEAT_NUM(data_abs, GET_STEP_DATA_MAX(tStep, bIsZ), s_repeatNum);
            } else {
                //printk("flickCheckStage: step%d, data=%i, repeat=%d, max=%d\n", tStep+1, data, s_repeatNum, s_dataMax);

                tStep = CHANGE_NEXT_STAGE(s_repeatNum, s_dataMax, STEP_1, STEP_2, STEP_1, tj9_wake_up_data.shake_num[2]);
                s_dataMax = 0;
                s_repeatNum = 0;
                s_cOrien = cOrien;

                s_repeatNum = ADD_REPEAT_NUM(data_abs, GET_STEP_DATA_MAX(tStep, bIsZ), s_repeatNum);
            }
            break;
            
        case STEP_2:
            if (data > 0) {
                data_abs = data;
                cOrien = '+';
            } else {
                data_abs = 0 - data;
                cOrien = '-';
            }
            s_dataMax = (s_dataMax > data_abs) ? s_dataMax : data_abs;
            
            if (cOrien == s_cOrien) {
                s_repeatNum = ADD_REPEAT_NUM(data_abs, GET_STEP_DATA_MAX(tStep, bIsZ), s_repeatNum);
            } else {
                //printk("flickCheckStage: step%d, data=%i, repeat=%d, max=%d\n", tStep+1, data, s_repeatNum, s_dataMax);
                tStep = CHANGE_NEXT_STAGE(s_repeatNum, s_dataMax, STEP_2, STEP_3, STEP_1, tj9_wake_up_data.shake_num[2]);
                if (STEP_1 == tStep) {
                    *bIsErrp = true;
                }
                s_dataMax = 0;
                s_repeatNum = 0;
                s_cOrien = cOrien;

                s_repeatNum = ADD_REPEAT_NUM(data_abs, GET_STEP_DATA_MAX(tStep, bIsZ), s_repeatNum);
            }
            break;

        case STEP_3:
            if (data > 0) {
                data_abs = data;
                cOrien = '+';
            } else {
                data_abs = 0 - data;
                cOrien = '-';
            }
            s_dataMax = (s_dataMax > data_abs) ? s_dataMax : data_abs;
            
            if (cOrien == s_cOrien) {
                s_repeatNum = ADD_REPEAT_NUM(data_abs, GET_STEP_DATA_MAX(tStep, bIsZ), s_repeatNum);
            } else {
                //printk("flickCheckStage: step%d, data=%i, reeat=%d, max=%d\n", tStep+1, data, s_repeatNum, s_dataMax);
                tStep = CHANGE_NEXT_STAGE(s_repeatNum, s_dataMax, STEP_3, STEP_4, STEP_1, tj9_wake_up_data.shake_num[2]);
                s_dataMax = 0;
                s_repeatNum = 0;
                s_cOrien = cOrien;

                s_repeatNum = ADD_REPEAT_NUM(data_abs, GET_STEP_DATA_MAX(tStep, bIsZ), s_repeatNum);
            }
            break;

        case STEP_4:
            if (data > 0) {
                data_abs = data;
                cOrien = '+';
            } else {
                data_abs = 0 - data;
                cOrien = '-';
            }
            s_dataMax = (s_dataMax > data_abs) ? s_dataMax : data_abs;
            
            if (cOrien == s_cOrien) {
                //printk("flickCheckStage: step%d, data=%i, reeat=%d, max=%d\n", tStep+1, data, s_repeatNum, s_dataMax);

                s_repeatNum = ADD_REPEAT_NUM(data_abs, GET_STEP_DATA_MAX(tStep, bIsZ), s_repeatNum);
                
                tStep = CHANGE_NEXT_STAGE(s_repeatNum, s_dataMax, STEP_4, STEP_OK, STEP_4, tj9_wake_up_data.shake_num[2]);
                s_dataMax = 0;
            } else {
                //printk("flickCheckStage: step%d, data=%i, reeat=%d, max=%d\n", tStep+1, data, s_repeatNum, s_dataMax);
                //tStep = ((s_repeatNum >= FLICK_STAGE_3_REPEAT_MAX_NUM) && 
                //        (s_dataMax >= FLICK_G_DATA_MAX)) ? STEP_4 : STEP_1;
                tStep = CHANGE_NEXT_STAGE(s_repeatNum, s_dataMax, STEP_4, STEP_OK, STEP_1, tj9_wake_up_data.shake_num[2]);
                s_dataMax = 0;
                s_repeatNum = 0;
                s_cOrien = cOrien;

                s_repeatNum = ADD_REPEAT_NUM(data_abs, GET_STEP_DATA_MAX(tStep, bIsZ), s_repeatNum);
            }
            break;
            
        default:
            printk("[L%d]default: return to step1, data=%d\n", __LINE__, data);
            g_tStage = STEP_1;
            break;
    }

    return tStep;
}


static void kxtj9_poll_timer_callback(unsigned long data)
{
    struct kxtj9_data  *ktj9 = g_kxtj9p;
    if (!g_bIsStop) {
        tj9_read_data.tj9 = ktj9;
        queue_work(kxtj9_read_work, &(tj9_read_data.wakeup_work));
            
        mod_timer(&(tj9_wake_up_data.poll_timer), jiffies + msecs_to_jiffies(TIME_MOTION_DETECT_POLL_TIME_ms)); 
    }
}

static void kxtj9_timer_callback(unsigned long data)
{
    int i = 0, j =0;
    int xyz=0;
    int step2Pos = 0;
    bool bIsErr = false;
    bool bIsZ;
    int   stepErrNum = 0;
    enum STEP_t tStep;

    g_bIsStop = true;
    g_bIsFirstMotion = true;
        
    printk("kxtj9_timer_callback: flick data = %d\n",  tj9_read_data.flickDataNum);
    
    printk("[kxtj9] flick data: ");

    //+++  test
    /*
    for (i = 0; i < tj9_read_data.flickDataNum; i++) {
        printk("[kxtj9] x y z = %6i %6i %6i\n", g_cFlickDataBuff[i][ORIEN_X],
                                                              g_cFlickDataBuff[i][ORIEN_Y],
                                                              g_cFlickDataBuff[i][ORIEN_Z]);
    }
    */
    //---  test
    
    for(xyz = 0; xyz < 3; xyz++) {
        step2Pos = 0;
        bIsErr = false;
        stepErrNum = 0;
    
        if (ORIEN_Y == xyz)
            continue;
        if (ORIEN_Z == xyz)
            bIsZ =  tj9_wake_up_data.shake_num[2] > Z_SHAKE_NUM;
        else 
            bIsZ = false;
        
        for (i = 0, j =0; i < tj9_read_data.flickDataNum; i++, j++) {
            tStep = flickCheckStage(g_cFlickDataBuff[j][xyz], step2Pos == j, & bIsErr, bIsZ);
            //tStep = flickCheckStage(g_cFlickDataBuff[i][xyz], 0 == i);
            if (STEP_OK == tStep)
                break;  
            else if ((STEP_2 == tStep) && (!step2Pos))
                step2Pos = i;
            
            if (bIsErr) {
                bIsErr = false;
                stepErrNum++;
                if (stepErrNum > 1)
                    break;
                
                if (step2Pos > 0) {
                    j = step2Pos-1;
                    if (j >= tj9_read_data.flickDataNum - 4)
                        break;
                    step2Pos = 0;
                    i =  j;
                }

                //printk("[kxtj9] shake-step change: error, skip to the pos(%d)\n", step2Pos);
            }
        }
        //printk("[kxtj9] check stage for xyz=%d step=%d i = %d num=%d ++++++\n", xyz, tStep+1, i,  tj9_read_data.flickDataNum);
        if (STEP_OK == tStep)
            break;      
    }
    tj9_read_data.flickDataNum = 0;

    if (STEP_OK == tStep) {
        printk("[kxtj9] shake-shake has happened --------------\n");
        input_report_rel(tj9_wake_up_data.tj9->dev_interrupt, REL_DIAL, 2);
        input_sync(tj9_wake_up_data.tj9->dev_interrupt);
    }

        
    printk("[kxtj9] clear shake_num\n");
    //tj9_wake_up_data.shake_num = 0;
    tj9_wake_up_data.shake_num[0] = 0;
    tj9_wake_up_data.shake_num[1] = 0;
    tj9_wake_up_data.shake_num[2] = 0;

    tj9_wake_up_data.reverse_num[0] = 0;
    tj9_wake_up_data.reverse_num[1] = 0;
    tj9_wake_up_data.reverse_num[2] = 0;

}

static int kxtj9_EnWakeUpFunction(struct kxtj9_data *tj9)
{
    int retval = 0;

    //- initialize accelerometer in stand-by mode.
    retval = i2c_smbus_write_byte_data(tj9->client, KXTT9_CTL_REG_1, 0x00); 
    if (retval < 0) {
        dev_err(&tj9->client->dev, "read err int source\n");
        goto out;
    }

    //- initializekeep the accelerometer in stand-by mode, 
    //- to set the performance mode of the KXTJ2 to high current 12 bit resolution
    tj9->pdata.clt_reg1 = ENABLE_WUFE;
    tj9->ctrl_reg1 |= 0x42;
    retval = i2c_smbus_write_byte_data(tj9->client, KXTT9_CTL_REG_1, tj9->ctrl_reg1);   
    if (retval < 0) {
        dev_err(&tj9->client->dev, "read err int source\n");
        goto out;
    }

    //- set the Output Data Rate of the Wake Up function (motion detection) (OWUF) to 100 Hz
    //retval = i2c_smbus_write_byte_data(tj9->client, KXTT9_CTL_REG_2, 0x07);   
    retval = i2c_smbus_write_byte_data(tj9->client, KXTT9_CTL_REG_2, 0x06); //- 50HZ    
    if (retval < 0) {
        dev_err(&tj9->client->dev, "read err int source\n");
        goto out;
    }

    //- define the direction of detected motion for all positive and negative directions: 
    //- x positive (x+), x negative (x-),
    //retval = i2c_smbus_write_byte_data(tj9->client, KXTT9_INT_CTL_REG_2, 0x30);   
    retval = i2c_smbus_write_byte_data(tj9->client, KXTT9_INT_CTL_REG_2, 0x7F);// xyz   
    if (retval < 0) {
        dev_err(&tj9->client->dev, "read err int source\n");
        goto out;
    }

    //-set the time motion must be present before a wake-up interrupt is set to x second.
    //retval = i2c_smbus_write_byte_data(tj9->client, KXTJ9_WAKEUP_TIMER, 0x01);    
    retval = i2c_smbus_write_byte_data(tj9->client, KXTJ9_WAKEUP_TIMER, 0x02);  
    if (retval < 0) {
        dev_err(&tj9->client->dev, "read err int source\n");
        goto out;
    }

    //- set the level to 0.5g
    //retval = i2c_smbus_write_byte_data(tj9->client, KXTJ9_WAKEUP_THRESHOLD, 0x08); //- 0.5G   
    //retval = i2c_smbus_write_byte_data(tj9->client, KXTJ9_WAKEUP_THRESHOLD, 10); 
    //retval = i2c_smbus_write_byte_data(tj9->client, KXTJ9_WAKEUP_THRESHOLD, 11);  //- 0.7
    //- 0.8G
    //retval = i2c_smbus_write_byte_data(tj9->client, KXTJ9_WAKEUP_THRESHOLD, 12);  
    //retval = i2c_smbus_write_byte_data(tj9->client, KXTJ9_WAKEUP_THRESHOLD, 14);//- 0.9
    retval = i2c_smbus_write_byte_data(tj9->client, KXTJ9_WAKEUP_THRESHOLD, 16);//- 1G  
    //retval = i2c_smbus_write_byte_data(tj9->client, KXTJ9_WAKEUP_THRESHOLD, 24); //- 1.5G 
    //retval = i2c_smbus_write_byte_data(tj9->client, KXTJ9_WAKEUP_THRESHOLD, 40); //- 2.5G 
    if (retval < 0) {
        dev_err(&tj9->client->dev, "read err int source\n");
        goto out;
    }

    //- output the physical interrupt of the previously defined Wake-Up detect function. 
    //- This value will create an active high and latched interrupt.
    retval = i2c_smbus_write_byte_data(tj9->client, KXTT9_INT_CTL_REG_1, 0x30); 
    if (retval < 0) {
        dev_err(&tj9->client->dev, "read err int source\n");
        goto out;
    }

    //- set the accelerometer in operating mode with the previously defined settings.
    retval = i2c_smbus_write_byte_data(tj9->client, KXTT9_CTL_REG_1, 0xC2); 
    if (retval < 0) {
        dev_err(&tj9->client->dev, "read err int source\n");
        goto out;
    }

    tj9_wake_up_data.shake_num[ORIEN_X] = 0;
    tj9_wake_up_data.shake_num[ORIEN_Y] = 0;
    tj9_wake_up_data.shake_num[ORIEN_Z] = 0;
    
    tj9_wake_up_data.reverse_num[ORIEN_X] = 0;
    tj9_wake_up_data.reverse_num[ORIEN_Y] = 0;
    tj9_wake_up_data.reverse_num[ORIEN_Z] = 0;

    tj9_read_data.flickDataNum = 0;

    //- init timer
    setup_timer(&tj9_wake_up_data.timer, kxtj9_timer_callback, 0);
    setup_timer(&tj9_wake_up_data.poll_timer, kxtj9_poll_timer_callback, 0);
    
    kxtj9_wakeup_work = create_workqueue("kxtj9_wakeup_work");
    if (!kxtj9_wakeup_work) {
        printk("create workqueue for 'kxtj9_wakeup_work' ERROR \n");
        goto out;
    }
    INIT_WORK(& (tj9_wake_up_data.wakeup_work), kxtj9_wakeup_work_handler);

    kxtj9_read_work= create_workqueue("kxtj9_read_work");
    if (!kxtj9_read_work) {
        printk("create workqueue for 'kxtj9_wakeup_work' ERROR \n");
        goto out;
    }
    INIT_WORK(& (tj9_read_data.wakeup_work), kxtj9_read_work_handler);
    
    printk("[alian][kxtj91002] enable wake up OK\n");
    
out:
    return retval;
}

//--- ASUS_BSP Alian_Shen "add shake-shake gesture "

static int kxtj9_verify(struct kxtj9_data *tj9)
{
    int retval;

    retval = i2c_smbus_read_byte_data(tj9->client, WHO_AM_I);
    
    if (retval < 0) {
        dev_err(&tj9->client->dev, "read err int source\n");
        goto out;
    }

    //retval = (retval != 0x05 && retval != 0x07 && retval != 0x08)
    //      ? -EIO : 0;

    retval = (retval != 0x05 && retval != 0x07 && retval != 0x08 && retval != 0x09)
            ? -EIO : 0; 
out:
    return retval;
}
#ifdef CONFIG_OF
static int kxtj9_parse_dt(struct device *dev,
                struct kxtj9_platform_data *kxtj9_pdata)
{
    struct device_node *np = dev->of_node;
    u32 temp_val;
    int rc;

    rc = of_property_read_u32(np, "kionix,min-interval", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read min-interval\n");
        return rc;
    } else {
        kxtj9_pdata->min_interval = temp_val;
    }

    rc = of_property_read_u32(np, "kionix,init-interval", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read init-interval\n");
        return rc;
    } else {
        kxtj9_pdata->init_interval = temp_val;
    }

    rc = of_property_read_u32(np, "kionix,axis-map-x", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read axis-map_x\n");
        return rc;
    } else {
        kxtj9_pdata->axis_map_x = (u8)temp_val;
    }

    rc = of_property_read_u32(np, "kionix,axis-map-y", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read axis_map_y\n");
        return rc;
    } else {
        kxtj9_pdata->axis_map_y = (u8)temp_val;
    }

    rc = of_property_read_u32(np, "kionix,axis-map-z", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read axis-map-z\n");
        return rc;
    } else {
        kxtj9_pdata->axis_map_z = (u8)temp_val;
    }

    rc = of_property_read_u32(np, "kionix,g-range", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read g-range\n");
        return rc;
    } else {
        switch (temp_val) {
        case 2:
            kxtj9_pdata->g_range = KXTJ9_G_2G;
            break;
        case 4:
            kxtj9_pdata->g_range = KXTJ9_G_4G;
            break;
        case 8:
            kxtj9_pdata->g_range = KXTJ9_G_8G;
            break;
        default:
            kxtj9_pdata->g_range = KXTJ9_G_2G;
            break;
        }
    }

    kxtj9_pdata->negate_x = of_property_read_bool(np, "kionix,negate-x");

    kxtj9_pdata->negate_y = of_property_read_bool(np, "kionix,negate-y");

    kxtj9_pdata->negate_z = of_property_read_bool(np, "kionix,negate-z");

    if (of_property_read_bool(np, "kionix,res-12bit"))
        kxtj9_pdata->res_ctl = RES_12BIT;
    else
        kxtj9_pdata->res_ctl = RES_8BIT;

/*  

    //+++ ASUS Alian_Shen
    pdata->int_pin = of_get_named_gpio_flags(dt, "ap3426,irq-gpio",
                                0, &pdata->irq_flags);
                                */

    return 0;
}
#else
static int kxtj9_parse_dt(struct device *dev,
                struct kxtj9_platform_data *kxtj9_pdata)
{
    return -ENODEV;
}
#endif /* !CONFIG_OF */

// ASUS_BSP +++ guochang_qiu "[ZC550KL][Sensor][NA][Spec] support I2C stress test"
#ifdef CONFIG_I2C_STRESS_TEST
static int TestKXTJ2SensorI2C (struct i2c_client *apClient)
{
    int ret=0;
    int lnResult = I2C_TEST_PASS;
    struct kxtj9_data *tj9 = i2c_get_clientdata(apClient);

    i2c_log_in_test_case("%s  +\n",__func__);

    printk("[kxtj9] IIC stress test ++\n");

    if (tj9->enable) {
        ret = i2c_smbus_read_byte_data(tj9->client, CTRL_REG1);
        if (ret < 0) {
            i2c_log_in_test_case("%s: enable gsensor failed\n",__func__);
            lnResult = -1;
            goto exit;
        }
        msleep(10);
    } else {
        ret = kxtj9_enable(tj9);
        if(ret < 0){
            i2c_log_in_test_case("%s: enable gsensor failed\n",__func__);
            lnResult = -1;
            goto exit;
        }

        msleep(10);

        kxtj9_disable(tj9);
    }

    printk("[kxtj9] IIC stress test --\n"); 
    i2c_log_in_test_case("%s -\n",__func__);
exit:
    return lnResult;
}
static struct i2c_test_case_info gGSensorTestCaseInfo[] =
{
     __I2C_STRESS_TEST_CASE_ATTR(TestKXTJ2SensorI2C),
};
#endif
// ASUS_BSP --- Guochang_qiu "[ZC550KL][Sensor][NA][Spec] support I2C stress test"


static int kxtj9_probe(struct i2c_client *client,
                 const struct i2c_device_id *id)
{
    struct kxtj9_data *tj9;
    int err;

    if (!i2c_check_functionality(client->adapter,
                I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
        dev_err(&client->dev, "client is not i2c capable\n");
        return -ENXIO;
    }

    tj9 = kzalloc(sizeof(*tj9), GFP_KERNEL);
    if (!tj9) {
        dev_err(&client->dev,
            "failed to allocate memory for module data\n");
        return -ENOMEM;
    }   
    
    if (client->dev.of_node) {
        memset(&tj9->pdata, 0 , sizeof(tj9->pdata));
        err = kxtj9_parse_dt(&client->dev, &tj9->pdata);
        if (err) {
            dev_err(&client->dev,
                "Unable to parse platfrom data err=%d\n", err);
            return err;
        }
    } else {
        if (client->dev.platform_data)
            tj9->pdata = *(struct kxtj9_platform_data *)
                    client->dev.platform_data;
        else {
            dev_err(&client->dev,
                "platform data is NULL; exiting\n");
            return -EINVAL;
        }
    }   

    //+++ Alian_Shen
    //client->irq = gpio_to_irq(114);  
    //----
    
    tj9->client = client;
    tj9->power_enabled = false;

    if (tj9->pdata.init) {
        err = tj9->pdata.init();
        if (err < 0)
            goto err_free_mem;
    }
    
    err = kxtj9_power_init(tj9, true);
    if (err < 0) {
        dev_err(&tj9->client->dev, "power init failed! err=%d", err);
        goto err_pdata_exit;
    }
    
    err = kxtj9_device_power_on(tj9);
    if (err < 0) {
        dev_err(&client->dev, "power on failed! err=%d\n", err);
        goto err_power_deinit;
    }
    
    err = kxtj9_verify(tj9);
    if (err < 0) {
        dev_err(&client->dev, "device not recognized\n");
        goto err_power_off;
    }   

    i2c_set_clientdata(client, tj9);    

    //+++ ASUS Alian_Shen
    //tj9->ctrl_reg1 = tj9->pdata.res_ctl | tj9->pdata.g_range;
    tj9->ctrl_reg1 = tj9->pdata.res_ctl | tj9->pdata.g_range| tj9->pdata.clt_reg1;
    err = i2c_smbus_read_byte_data(tj9->client, KXTT9_CTL_REG_1);
    printk("[alian][kxtj9_isr][L%d] KXTT9_CTL_REG_1 = %X\n",__LINE__, err);
    //+++ ASUS Alian_Shen
    tj9->last_poll_interval = tj9->pdata.init_interval;

    tj9->cdev = sensors_cdev;
    /* The min_delay is used by userspace and the unit is microsecond. */
    tj9->cdev.min_delay = tj9->pdata.min_interval * 1000;
    tj9->cdev.delay_msec = tj9->pdata.init_interval;
    tj9->cdev.sensors_enable = kxtj9_enable_set;
    tj9->cdev.sensors_poll_delay = kxtj9_poll_delay_set;
    err = sensors_classdev_register(&client->dev, &tj9->cdev);
    if (err) {
        dev_err(&client->dev, "class device create failed: %d\n", err);
        goto err_power_off;
    }

    if (client->irq) {
        /* If in irq mode, populate INT_CTRL_REG1 and enable DRDY. */
        //tj9->int_ctrl |= KXTJ9_IEN | KXTJ9_IEA | KXTJ9_IEL;
        tj9->int_ctrl |= KXTJ9_IEN | KXTJ9_IEA;
        
        tj9->ctrl_reg1 |= DRDYE;    
        
        err = kxtj9_setup_input_device(tj9);
        if (err)
            goto err_power_off;

        err = request_threaded_irq(client->irq, NULL, kxtj9_isr,
                       IRQF_TRIGGER_RISING | IRQF_ONESHOT,
                       "kxtj9-irq", tj9);
        if (err) {
            dev_err(&client->dev, "request irq failed: %d\n", err);
            goto err_destroy_input;
        }

        //+++ ASUS Alian
        disable_irq(tj9->client->irq);
        //+++ ASUS Alian
                
        err = sysfs_create_group(&client->dev.kobj, &kxtj9_attribute_group);
        if (err) {
            dev_err(&client->dev, "sysfs create failed: %d\n", err);
            goto err_free_irq;
        }
        
        err = sysfs_create_group(&tj9->input_dev->dev.kobj, &kxtj9_attribute_group);
        if (err) {
            dev_err(&client->dev, "input sysfs create failed: %d\n", err);
            goto err_free_irq;
        }
//+++ ASUS_BSP Alian_Shen "add function for 'shake-shake'"
        err = sysfs_create_group(&tj9->dev_interrupt->dev.kobj, &kxtj9_attribute_group);
        if (err) {
            dev_err(&client->dev, "input sysfs create failed: %d\n", err);
            goto err_free_irq;
        }
//--- ASUS_BSP Alian_Shen "add function for 'shake-shake'"

    } else {
        err = kxtj9_setup_polled_device(tj9);
        if (err)
            goto err_power_off;
    }

    //+++ ASUS_BSP Alian_Shen "add shake-shake gesture "
    kxtj9_EnWakeUpFunction(tj9);
     g_kxtj9p = tj9;
    //--- ASUS_BSP Alian_Shen "add shake-shake gesture "

    // ASUS_BSP +++ guochang_qiu "[ZC550KL][Sensor][NA][Spec] support I2C stress test"
#ifdef CONFIG_I2C_STRESS_TEST
    i2c_add_test_case(client, "AccelSensorStressTest",ARRAY_AND_SIZE(gGSensorTestCaseInfo));        
#endif
// ASUS_BSP --- guochang_qiu "[ZC550KL][Sensor][NA][Spec] support I2C stress test"

    err = i2c_smbus_read_byte_data(tj9->client, KXTT9_CTL_REG_1);
    printk("[alian][kxtj9_isr] 1 KXTT9_CTL_REG_1 = %X\n", err);

    dev_dbg(&client->dev, "%s: kxtj9_probe OK.\n", __func__);
    kxtj9_device_power_off(tj9);
    
    return 0;

err_free_irq:
    free_irq(client->irq, tj9);
err_destroy_input:
    input_unregister_device(tj9->input_dev);
//+++ ASUS_BSP Alian_Shen "add shake-shake gesture "
    input_unregister_device(tj9->dev_interrupt);
//+++ ASUS_BSP Alian_Shen "add shake-shake gesture "
err_power_off:
    kxtj9_device_power_off(tj9);
err_power_deinit:
    kxtj9_power_init(tj9, false);
err_pdata_exit:
    if (tj9->pdata.exit)
        tj9->pdata.exit();
err_free_mem:
    kfree(tj9);

    dev_err(&client->dev, "%s: kxtj9_probe err=%d\n", __func__, err);
    return err;
}

static int kxtj9_remove(struct i2c_client *client)
{
    struct kxtj9_data *tj9 = i2c_get_clientdata(client);

    if (client->irq) {
        sysfs_remove_group(&client->dev.kobj, &kxtj9_attribute_group);
        free_irq(client->irq, tj9);
        input_unregister_device(tj9->input_dev);
    } else {
        kxtj9_teardown_polled_device(tj9);
    }

    //+++ ASUS_BSP Alian_Shen "add shake-shake gesture "
    if(kxtj9_wakeup_work)
            destroy_workqueue(kxtj9_wakeup_work);

    if(& tj9_wake_up_data.timer != NULL)
        del_timer(& tj9_wake_up_data.timer);
    //--- ASUS_BSP Alian_Shen "add shake-shake gesture "
    
    kxtj9_device_power_off(tj9);
    kxtj9_power_init(tj9, false);

    if (tj9->pdata.exit)
        tj9->pdata.exit();

    kfree(tj9);

    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int kxtj9_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kxtj9_data *tj9 = i2c_get_clientdata(client);
    struct input_dev *input_dev = tj9->input_dev;

    mutex_lock(&input_dev->mutex);

    if (input_dev->users && tj9->enable)
        kxtj9_disable(tj9);

    mutex_unlock(&input_dev->mutex);
    return 0;
}

static int kxtj9_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kxtj9_data *tj9 = i2c_get_clientdata(client);
    struct input_dev *input_dev = tj9->input_dev;
    int retval = 0;

    mutex_lock(&input_dev->mutex);

    if (input_dev->users && tj9->enable)
        kxtj9_enable(tj9);

    mutex_unlock(&input_dev->mutex);
    return retval;
}
#endif

static SIMPLE_DEV_PM_OPS(kxtj9_pm_ops, kxtj9_suspend, kxtj9_resume);

static const struct i2c_device_id kxtj9_id[] = {
    { DEVICE_NAME, 0 },
    { },
};

static struct of_device_id kxtj9_match_table[] = {
    { .compatible = "kionix,kxtj9_zc550KL", },
    { },
};


MODULE_DEVICE_TABLE(i2c, kxtj9_id);

static struct i2c_driver kxtj9_driver = {
    .driver = {
        .name   = DEVICE_NAME,
        .owner  = THIS_MODULE,
        .of_match_table = kxtj9_match_table,
        .pm = &kxtj9_pm_ops,
    },
    .probe      = kxtj9_probe,
    .remove     = kxtj9_remove,
    .id_table   = kxtj9_id,
};

module_i2c_driver(kxtj9_driver);

MODULE_DESCRIPTION("KXTJ9 accelerometer driver");
MODULE_AUTHOR("Chris Hudson <chudson@kionix.com>");
MODULE_LICENSE("GPL");
