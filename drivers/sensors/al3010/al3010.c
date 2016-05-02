#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>	//ASUS_BSP Lenter+
#include <linux/atomic.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/switch.h>
#include <linux/microp.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#include "linux/input/proximity_class.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h> 
#endif

#define AL3010_DRV_NAME	"al3010_light_sensor"
#define DRIVER_VERSION		"1.0"

#define P01_EVENT_NOTIFY_LIGHTSENSOR_NO_ERROR (0)
#define P01_EVENT_NOTIFY_LIGHTSENSOR_ERROR (-1)

#define AL3010_NUM_CACHABLE_REGS		9

#define	AL3010_ALS_COMMAND			0x04
#define	AL3010_RAN_MASK				0x70
#define	AL3010_RAN_SHIFT				(4)

#define AL3010_MODE_COMMAND			0x00
#define AL3010_MODE_SHIFT				(0)
#define AL3010_MODE_MASK				0x07

#define AL3010_POW_MASK				0x01

/*Power system mode(on/off) command*/
#define AL3010_POW_DOWN				0x00
#define AL3010_POW_UP					0x01
#define AL3010_POW_RESET				0x04
#define AL3010_GET_ADC_POW_DOWN		0x05
#define AL3010_POW_SHIFT				(0)

/*Adc register*/
#define	AL3010_ADC_LSB				0x0c
#define	AL3010_ADC_MSB				0x0d

/*INT register*/
#define AL3010_INT_STATUS				0x01

/*Configuration register*/
#define AL3010_INT_COMMAND			0x10
/*
#define AL3010_INT_SHIFT				0x00       
#define AL3010_INT_IF					0x03
#define AL3010_INT_MASK				0xff
*/

/*THD register*/
#define AL3010_LOW_THD_LSB			0x1A
#define AL3010_LOW_THD_MSB			0x1B
#define AL3010_HIGH_THD_LSB			0x1C
#define AL3010_HIGH_THD_MSB			0x1D

/*Interrupt Filter register command*/
static int ALS_IF = 0x02;	//8 conversion times

/*Ambient light detectable range command*/
static int ALS_Gain = 0x01;		//0~19452 Lux 	  :   resolution = 0.3
static int al3010_range[4] = {77806,19452,4863,1216};

#if 0
static u8 al3010_reg[AL3010_NUM_CACHABLE_REGS] = 
	{0x00,0x01,0x0c,0x0d,0x10,0x1a,0x1b,0x1c,0x1d};
#endif

struct al3010_data {
	struct i2c_client *client;
	struct mutex lock;
	struct input_dev   *input_dev;
	u8 reg_cache[AL3010_NUM_CACHABLE_REGS];
	u8 power_state_before_suspend;
	/* For set Threshold*/
	int adb;

	unsigned int					poll_interval_ms;   /* I2C polling period */
	unsigned int					event_threshold;    /* Change reqd to gen event */
	unsigned int					open_count;     /* Reference count */
	char							polling;            /* Polling flag */
};

struct al3010_data *g_al3010_data_as;
struct input_dev *this_input_dev_p02_als = NULL;

static struct workqueue_struct *al3010light_workqueue = NULL;
static struct delayed_work al3010_attached_P02_work;
static struct work_struct al3010_ISR_work;
static struct delayed_work al3010_ISR_delay_work;
bool al3010_interrupt_busy = false;	

static int level = 0;
static int TOTALMAPS = 17;
static int g_al3010_light_map[17] = {0,50,100,200,300,400,500,650,800,1000,1500,2000,3000,4000,5000,7000,10000};

/*
static int al3010_threshold_max_level = 40;
static int g_al3010_light_level[40] = 
    {50,100,150,200,350,450,550,650,750,850,1000,1350,1700,2000,2500,3000,3500,4000,4500,5000,6000,7000,8000,9000,10000,11000,12000,13000,14000,16000,18000,20000,22000,24000,28000,31000,34000,37000,40000,50000};

static int g_thd[42] = 
     {0,50,100,150,200,350,450,550,650,750,850,1000,1350,1700,2000,2500,3000,3500,4000,4500,5000,6000,7000,8000,9000,10000,11000,12000,13000,14000,16000,18000,20000,22000,24000,28000,31000,34000,37000,40000,50000,65535};
*/

bool g_bIsP01Attached = false;
static int g_AlsP01ProbeError = 0xff;

/* HW is turn on or not */
bool g_al3010_switch_on = false;

/* Flag for check sensor in suspend state */
static int g_al3010_switch_earlysuspend = 0;

/*For resume and debounce I2C issue*/
static struct delayed_work Al3010light_resume_work;
static struct delayed_work Al3010light_debounce_work;
static struct workqueue_struct *Al3010light_delay_workqueue = NULL;
bool g_al3010_suspend_switch_on = false;

//wake_lock for resume power on lightsensor
#include <linux/wait.h>
#include <linux/wakelock.h>
static struct wake_lock pad_lightsensoer_wake_lock;

/* For Lux report issue */
static int g_al3010_light = 0;
static int g_last_al3010_light = 0;
static int g_al3010_light_first=1;

/* For Calibration*/
static int p_als_threshold_lo = 0;
static int p_als_threshold_hi = 0;
static int a_als_calibration_accuracy = 10000;
extern unsigned int g_microp_ver;
#ifdef ASUS_FACTORY_BUILD
static int p_als_calibration_lux = 800;		//1000 - 200 Lux
static int p_als_low_calibration_adc = 0;
static int p_als_high_calibration_adc = 0;
#endif
static u32 g_al3010_light_calibration = 84;
static int g_al3010_light_shift_calibration = 35;


static int al3010_get_adc_value(struct i2c_client *client);
static int al3010_put_property(struct i2c_client *client);
static void mp_als_interrupt_handler(struct work_struct *work);
//static int al3010_init_client(struct i2c_client *client);

//void reportPadStationI2CFail(char *devname);;
//extern bool hdmi_exist(void);
extern int g_HAL_als_switch_on;		// For all lightsensor trun on/off global flag

// For Pad I2C suspend/resume issue
static int al3010_power_on_retry_time = 0;

static struct switch_dev ls_switch_dev ={ 
        .name = AL3010_DRV_NAME,
        .index = 0,
};

#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>

#define I2C_TEST_FAIL_SENSOR (-1)
int set_als_power_state_of_P01(int);

static int Test_Al3010_SensorI2C(struct i2c_client *apClient)
{
	int err = 0;
	int lnResult = I2C_TEST_PASS;

	i2c_log_in_test_case("Test_Al3010_SensorI2C++\n");
	
	if( g_AlsP01ProbeError == 0 && g_bIsP01Attached )	{
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(100));

		if (!g_HAL_als_switch_on)	{
			err = set_als_power_state_of_P01(1);
			if ( err < 0 )	{
				i2c_log_in_test_case("Fail to turn on al3010 lsensor\n");
				lnResult = I2C_TEST_FAIL_SENSOR;
			}
			err = al3010_get_adc_value(apClient);
			if ( err < 0 )	{
				i2c_log_in_test_case("Fail to read al3010 data\n");
				lnResult = I2C_TEST_FAIL_SENSOR;
			}
			else	{
				err = set_als_power_state_of_P01(0);
				if ( err < 0 )	{
					i2c_log_in_test_case("Fail to turn off al3010 lsensor\n");
					lnResult = I2C_TEST_FAIL_SENSOR;
				}
			}
		}
		else	{
			err = al3010_get_adc_value(apClient);
			if ( err < 0 )	{
				i2c_log_in_test_case("Fail to read al3010 data\n");
				lnResult = I2C_TEST_FAIL_SENSOR;
			}
			else	{
				err = set_als_power_state_of_P01(0);
				if ( err < 0 )	{
					i2c_log_in_test_case("Fail to turn off al3010 lsensor\n");
					lnResult = I2C_TEST_FAIL_SENSOR;
				}
			}
		}
	}else	{
		i2c_log_in_test_case("Fail to lsensor test\n");
		lnResult = I2C_TEST_FAIL_SENSOR;
	}

	i2c_log_in_test_case("Test_Al3010_SensorI2C--\n");

	return lnResult;
};

static struct i2c_test_case_info gSensorTestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(Test_Al3010_SensorI2C),
};
#endif
//ASUS_MERGE_END

/*
 * register access helpers
 */

static int __al3010_read_reg(struct i2c_client *client,
			       u32 reg, u8 mask, u8 shift)
{
	return (g_al3010_data_as->reg_cache[reg] & mask) >> shift;
}

static int __al3010_write_reg(struct i2c_client *client,
				u32 reg, u8 mask, u8 shift, u8 val)
{
	int ret = 0;
	u8 tmp;

	if (reg >= AL3010_NUM_CACHABLE_REGS)
		return -EINVAL;

	mutex_lock(&g_al3010_data_as->lock);

	tmp = g_al3010_data_as->reg_cache[reg];
	tmp &= ~mask;
	tmp |= val << shift;

	ret = i2c_smbus_write_byte_data(client, reg, tmp);
	if (!ret)
		g_al3010_data_as->reg_cache[reg] = tmp;

	mutex_unlock(&g_al3010_data_as->lock);
	return ret;
}

/*
 * internally used functions
 */

/* range */
static int al3010_get_range(struct i2c_client *client)
{
	int tmp;
	tmp = __al3010_read_reg(client, AL3010_ALS_COMMAND,
											AL3010_RAN_MASK, AL3010_RAN_SHIFT);;
	return al3010_range[tmp];
}

static int al3010_set_range(struct i2c_client *client, int range)
{
	//return __al3010_write_reg(client, AL3010_ALS_COMMAND, AL3010_RAN_MASK, AL3010_RAN_SHIFT, range);
	return i2c_smbus_write_byte_data(client, AL3010_INT_COMMAND,  range);
}

/* mode */
static int al3010_get_mode(struct i2c_client *client)
{
	return __al3010_read_reg(client, AL3010_MODE_COMMAND,
		AL3010_MODE_MASK, AL3010_MODE_SHIFT);
}

static int al3010_set_mode(struct i2c_client *client, int mode)
{
    if(AL3010_POW_UP == (mode & AL3010_POW_MASK)) {
        g_al3010_switch_on = true;
    }
    else if(AL3010_POW_DOWN == (mode & AL3010_POW_MASK)) {
        g_al3010_switch_on = false;
    }

	return __al3010_write_reg(client, AL3010_MODE_COMMAND,
		AL3010_MODE_MASK, AL3010_MODE_SHIFT, mode);
}

/* power_state */
static int al3010_set_power_state(struct i2c_client *client, int state)
{
	int ret = 0;
	//mutex_lock(&g_al3010_data_as->lock);
	
	if(AL3010_POW_UP == state)	{
		g_al3010_switch_on = true;
		// Enable microp lightsensor interrupt
		AX_MicroP_enablePinInterrupt( INTR_EN_ALS_INT, 1 );
	}
	else if(AL3010_POW_DOWN == state)	{
		g_al3010_switch_on = false;
		// Disable microp lightsensor interrupt
		AX_MicroP_enablePinInterrupt( INTR_EN_ALS_INT, 0 );
		g_last_al3010_light = -1;
	}
	g_al3010_light_first=1;

	printk(DBGMSK_PRX_G2"[als_P01] al3010_set_pwr_state: state:%d\n", g_al3010_switch_on);

	ret = i2c_smbus_write_byte_data(
		g_al3010_data_as->client, AL3010_MODE_COMMAND, g_al3010_switch_on);
	
	//mutex_unlock(&g_al3010_data_as->lock);
	
	return ret;
}

static int al3010_get_power_state(struct i2c_client *client)
{
	return i2c_smbus_read_byte_data(g_al3010_data_as->client, AL3010_MODE_COMMAND );
/*
	struct al3010_data *data = i2c_get_clientdata(client);
	u8 cmdreg = data->reg_cache[AL3010_MODE_COMMAND];
	return (cmdreg & AL3010_POW_MASK) >> AL3010_POW_SHIFT;
*/
}

int set_als_power_state_of_P01(int state)
{
	int ret = 0, indx;
	/* For resume check Pad power state */
	int microp_state = -1;
	
	printk("[als_P01]set_als_pwr_state: %d\n", state );
	if( !AX_MicroP_IsP01Connected() )	{
		printk("[als_P01]Without P03 plug in\n");
		return -1;		
	}
	
	if( g_al3010_switch_earlysuspend == 1 )	{
		g_al3010_suspend_switch_on = state;
		printk("[als_P01][als] Al3010 without resume, by pass; state:%d\n", g_al3010_switch_earlysuspend);
		return 0;
	}else
		g_al3010_suspend_switch_on = false;

	mutex_lock(&g_al3010_data_as->lock);
	wake_lock_timeout(&pad_lightsensoer_wake_lock, 2*HZ);
	al3010_interrupt_busy = true;

	/*Check microp state before Inital al3010 */
	if(!AX_MicroP_Is_3V3_ON())	{
		printk("[als_P01][als] Bus Suspended: Skip\r\n");
		microp_state = st_MICROP_Sleep;
	}
	else
		microp_state = AX_MicroP_getOPState();
	
	if(microp_state == st_MICROP_Active)	{
		al3010_power_on_retry_time = 0;
		printk(DBGMSK_PRX_G2"[al3010][als] Microp in Active mode\n");
	}
	else	{
		printk("[al3010][als] Microp not in Active mode(%d), retry %d\n", microp_state, al3010_power_on_retry_time);
		al3010_power_on_retry_time++;
		if ( al3010_power_on_retry_time < 10 )	{
			al3010_interrupt_busy = false;
			mutex_unlock(&g_al3010_data_as->lock);
			wake_unlock(&pad_lightsensoer_wake_lock);
			queue_delayed_work(Al3010light_delay_workqueue, &Al3010light_resume_work, 50 );
			return 0;
		}
		else
			printk("[als_P01] set_als_pwr_state retry fail!!(%d)\n", state);
	}
	
	/*Inital al3010*/
	for(indx = 0; indx<5; indx++) {
		ret = al3010_put_property(g_al3010_data_as->client);
		if(!ret)	{
			printk(DBGMSK_PRX_G2"[al3010][als] init al3010 success\n");
			break;
		}else	{
			printk("[al3010][als] init_client error retry = %d\n",indx);
			msleep( 10 );
		}
		if ( indx >= 4 ){
			al3010_interrupt_busy = false;
			mutex_unlock(&g_al3010_data_as->lock);
			wake_unlock(&pad_lightsensoer_wake_lock);
			return ret;
		}
	}

	/*Trun on/off al3010*/
	for(indx = 0; indx<5; indx++) {
		ret = al3010_set_power_state(
			g_al3010_data_as->client, state? AL3010_POW_UP:AL3010_POW_DOWN);
		if(!ret) {
			printk(DBGMSK_PRX_G2"[al3010][als] switch on al3010 success\n");
			break;
		}else
			printk("[al3010][als] i2c error retry = %d\n",indx);
		if (indx >= 4 ) {
			al3010_interrupt_busy = false;
			mutex_unlock(&g_al3010_data_as->lock);
			wake_unlock(&pad_lightsensoer_wake_lock);
			//reportPadStationI2CFail("al3010");
			return ret;
		}
	}
	
	/*Release interrupt trigger*/
	i2c_smbus_read_byte_data(g_al3010_data_as->client, AL3010_ADC_MSB);
	al3010_interrupt_busy = false;
	mutex_unlock(&g_al3010_data_as->lock);
	wake_unlock(&pad_lightsensoer_wake_lock);

	if (state == 1)
		printk(DBGMSK_PRX_G2"[al3010][als] P02 light sensor dev_open\n");
	else
		printk(DBGMSK_PRX_G2"[al3010][als] P02 light sensor dev_close\n");

	queue_delayed_work(Al3010light_delay_workqueue, &al3010_ISR_delay_work, 10);

	return ret;
}
EXPORT_SYMBOL(set_als_power_state_of_P01);

static int al3010_get_adc_value(struct i2c_client *client)
{
	int lsb, msb;
	int adc;
	printk(DBGMSK_PRX_G6"[als_P01]++al3010_get_adc_value \n");

	mutex_lock(&g_al3010_data_as->lock);

	msb = i2c_smbus_read_byte_data(client, AL3010_MODE_COMMAND);
	printk(DBGMSK_PRX_G6"[als_P01]al3010_get_adc_value: reg (0x00) = 0x%x\n", msb);
	msb = i2c_smbus_read_byte_data(client, AL3010_INT_COMMAND);
	printk(DBGMSK_PRX_G6"[als_P01]al3010_get_adc_value: reg (0x10) = 0x%x\n", msb);

	lsb = i2c_smbus_read_byte_data(client, AL3010_ADC_LSB);

	if (lsb < 0) {
		mutex_unlock(&g_al3010_data_as->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, AL3010_ADC_MSB);
	mutex_unlock(&g_al3010_data_as->lock);

	printk(DBGMSK_PRX_G6"[als_P01]****al3010_get_adc_value: msb=%d, lsb=%d\n", msb, lsb);

	if (msb < 0)
		return msb;

	adc = (u32)((msb << 8) | lsb) ;

#if 0
	/*Get Lux*/
	adc = g_al3010_light_shift_calibration + (adc * g_al3010_light_calibration/a_als_calibration_accuracy);
	for(int i = 1 ; i < TOTALMAPS ; i++) {
		if( adc < g_al3010_light_map[i] ) {
			adc = g_al3010_light_map[ i -1 ];
			break;
		}
		else if( adc > g_al3010_light_map[TOTALMAPS - 1] )	{
			adc = g_al3010_light_map[ TOTALMAPS -1 ];
			break;
		}
	}
	if( adc > g_al3010_light_map[TOTALMAPS - 1] )
		adc = g_al3010_light_map[TOTALMAPS - 1];
#endif
	return adc;
}



static int al3010_put_property(struct i2c_client *client)
{
	int status = 0;
	int lsb, msb;

	//mutex_lock(&g_al3010_data_as->lock);
	/*Set light detectable range*/
	status = i2c_smbus_write_byte_data(client, AL3010_INT_COMMAND,  ALS_IF | (ALS_Gain << 4 ));

	if (status < 0)  {
		printk(DBGMSK_PRX_G2"[als_P02] addr=0x%x, val=0x%x, ret=%d\n",AL3010_INT_COMMAND, ALS_IF | (ALS_Gain << 4 ), status);
		switch_set_state(&ls_switch_dev, P01_EVENT_NOTIFY_LIGHTSENSOR_ERROR);
		mutex_unlock(&g_al3010_data_as->lock);
		return status;
	}else
		printk(DBGMSK_PRX_G2"[als_P02] addr=0x%x, val=0x%x\n",AL3010_INT_COMMAND, ALS_IF  | (ALS_Gain << 4 ) );

	/*Set high/low threshold*/
	i2c_smbus_write_byte_data(client, AL3010_LOW_THD_LSB, 0x00);//g_thd[0] & 0xFF);
	lsb = i2c_smbus_read_byte_data(client, AL3010_LOW_THD_LSB);

	i2c_smbus_write_byte_data(client, AL3010_LOW_THD_MSB, 0x00);//(g_thd[0] >> 8 ) & 0xFF);
	msb = i2c_smbus_read_byte_data(client, AL3010_LOW_THD_MSB);

	printk(DBGMSK_PRX_G2"[als_P02]++ al3010_set_inital_low_threshold_value: msb=%d, lsb=%d\n", msb, lsb);

	/* Set High threshold*/
	i2c_smbus_write_byte_data(client, AL3010_HIGH_THD_LSB,  0x00);//g_thd[1] & 0xFF);
	lsb = i2c_smbus_read_byte_data(client, AL3010_HIGH_THD_LSB);

	i2c_smbus_write_byte_data(client, AL3010_HIGH_THD_MSB, 0x00);//(g_thd[1] >> 8 ) & 0xFF);
	msb = i2c_smbus_read_byte_data(client, AL3010_HIGH_THD_MSB);

	printk(DBGMSK_PRX_G2"[als_P02]-- al3010_set_inital_high_threshold_value: msb=%d, lsb=%d\n", msb, lsb);

	/*Check interrupt state (Read only)*/
	status = i2c_smbus_read_byte_data(client, AL3010_INT_STATUS);

	if (status == 0)
		printk(DBGMSK_PRX_G2"[al3010][als] P02 light sensor interrupt is cleared\n");
	else{
		printk(DBGMSK_PRX_G2"[al3010][als] P02 light sensor interrupt is triggered\n");
		/*Release interrupt trigger*/
		i2c_smbus_read_byte_data(g_al3010_data_as->client, AL3010_ADC_MSB);
	}
	//mutex_unlock(&g_al3010_data_as->lock);
  
        return 0;
}

static void mp_als_interrupt_delay_work(struct work_struct *work)
{
	int lsb = 0;
	int msb = 0;
	int indx = 0;
	int ret = 0;
	
	mutex_lock(&g_al3010_data_as->lock);

	/* Set Low threshold*/
	indx = (g_al3010_data_as->adb * 8)/10;
	i2c_smbus_write_byte_data(
			g_al3010_data_as->client, AL3010_LOW_THD_LSB, indx & 0xFF);//g_thd[level] & 0xFF);
	lsb = i2c_smbus_read_byte_data(
			g_al3010_data_as->client, AL3010_LOW_THD_LSB);

	i2c_smbus_write_byte_data(
			g_al3010_data_as->client, AL3010_LOW_THD_MSB, (indx >> 8 ) & 0xFF);
	msb = i2c_smbus_read_byte_data(
			g_al3010_data_as->client, AL3010_LOW_THD_MSB);

	p_als_threshold_lo = ((msb <<8) | lsb );

	printk(DBGMSK_PRX_G2"[als_P02]-- al3010_get_high_threshold_value: %d ,msb=%d, lsb=%d\n"
											,p_als_threshold_lo, msb, lsb);

	/* Set High threshold*/
	indx = (g_al3010_data_as->adb * 12)/10;
	i2c_smbus_write_byte_data(
			g_al3010_data_as->client, AL3010_HIGH_THD_LSB, indx & 0xFF);//g_thd[level + 1] & 0xFF);
	lsb = i2c_smbus_read_byte_data(
			g_al3010_data_as->client, AL3010_HIGH_THD_LSB);

	i2c_smbus_write_byte_data(
			g_al3010_data_as->client, AL3010_HIGH_THD_MSB, (indx >> 8 ) & 0xFF);//(g_thd[level + 1] >> 8 ) & 0xFF);
	msb = i2c_smbus_read_byte_data(
			g_al3010_data_as->client, AL3010_HIGH_THD_MSB);

	p_als_threshold_hi = ((msb <<8) | lsb );
	printk(DBGMSK_PRX_G2"[als_P02]++ al3010_get_low_threshold_value: %d ,msb=%d, lsb=%d\n"
											,p_als_threshold_hi, msb, lsb);

	/*Trun on al3010*/
	for(indx = 0; indx<5; indx++) {
		ret = i2c_smbus_write_byte_data(
			g_al3010_data_as->client, AL3010_MODE_COMMAND, AL3010_POW_UP);
		if(!ret) {
			printk(DBGMSK_PRX_G2"[al3010][als] switch on al3010 success\n");
			break;
		}else if ( indx >= 5 )
			printk("[al3010][als] %s: switch on al3010 fail\n",__FUNCTION__);
		else
			printk("[al3010][als] i2c error retry = %d\n",indx);
	}
	al3010_interrupt_busy = false;

	/*Check interrupt state (Read only)*/
	ret = i2c_smbus_read_byte_data(g_al3010_data_as->client, AL3010_INT_STATUS);

	if (ret == 0)
		printk(DBGMSK_PRX_G2"[al3010][als] P02 light sensor interrupt is cleared\n");
	else{
		printk(DBGMSK_PRX_G2"[al3010][als] P02 light sensor interrupt is triggered\n");
		/*Release interrupt trigger*/
		i2c_smbus_read_byte_data(g_al3010_data_as->client, AL3010_ADC_MSB);
	}
	mutex_unlock(&g_al3010_data_as->lock);
}

static void mp_als_interrupt_handler(struct work_struct *work)
{
	int lsb, msb;
	u16 adc = 0;
	int k_adc = 0;
	int ret = 0;
	level = 0;
	
	if( g_bIsP01Attached && g_al3010_switch_on ) {
		mutex_lock(&g_al3010_data_as->lock);
		al3010_interrupt_busy = true;

		/*Get one ALS data and power down*/
		ret = i2c_smbus_write_byte_data(
				g_al3010_data_as->client, AL3010_MODE_COMMAND, AL3010_GET_ADC_POW_DOWN);

		/*Read adc*/
		lsb = i2c_smbus_read_byte_data(g_al3010_data_as->client, AL3010_ADC_LSB);
		if (lsb < 0) {
			switch_set_state(&ls_switch_dev,P01_EVENT_NOTIFY_LIGHTSENSOR_ERROR);
			al3010_interrupt_busy = false;
			mutex_unlock(&g_al3010_data_as->lock);
			return;
		}

		msb = i2c_smbus_read_byte_data(g_al3010_data_as->client, AL3010_ADC_MSB);
		if (msb < 0) {
			al3010_interrupt_busy = false;
			switch_set_state(&ls_switch_dev,P01_EVENT_NOTIFY_LIGHTSENSOR_ERROR);
			mutex_unlock(&g_al3010_data_as->lock);
			return;
		}

		adc = (u32)((msb << 8) | lsb) ;

		/* For set threshold */
		g_al3010_data_as->adb = adc;
		
		mutex_unlock(&g_al3010_data_as->lock);
		
		printk(DBGMSK_PRX_G2"/********************************************************/\n");
		printk(DBGMSK_PRX_G2"[als_P02] al3010_get_raw_adc_value: %d\n", adc);

		/*Get threshold level*/
		/*
		for( i = 0 ; i < al3010_threshold_max_level ; i++) {
			if( adc < g_al3010_light_level[i] ) {
				level = i;
				break;
			}
			else if (adc > g_al3010_light_level[al3010_threshold_max_level - 1])	{
				level = al3010_threshold_max_level;
				break;
			}
		}
		*/

		/*Get Lux*/
		g_al3010_light = 0;
		if ( adc == 0 )
			k_adc = 0;
		else
			k_adc = g_al3010_light_shift_calibration + (adc * g_al3010_light_calibration/a_als_calibration_accuracy);
/*
		for( i=1 ; i < TOTALMAPS ; i++) {
			if( k_adc < g_al3010_light_map[i] ) {
				g_al3010_light = g_al3010_light_map[ i -1 ];
				break;
			}
			else if( k_adc > g_al3010_light_map[TOTALMAPS - 1] )	{
				g_al3010_light = g_al3010_light_map[ TOTALMAPS -1 ];
				break;
			}
		}
*/

		if ( k_adc > 0 )
			g_al3010_light = k_adc;
		else
			g_al3010_light = 0;
		
		if( g_al3010_light > g_al3010_light_map[TOTALMAPS - 1] )
			g_al3010_light = g_al3010_light_map[TOTALMAPS - 1];
		printk(DBGMSK_PRX_G2"[als_P02] level= %d, raw adc= %d, cal_adc= %d, lux = %d\n",level, adc, k_adc, g_al3010_light);

		/* Report Lux*/
		if(g_al3010_light != g_last_al3010_light || g_al3010_light_first) {
			g_last_al3010_light = g_al3010_light;
			als_lux_report_event( g_al3010_light);
			if (g_al3010_light_first)
				printk("[als_Pad][als] First light=%d \n", g_al3010_light);
			g_al3010_light_first = 0;
		}
		printk(DBGMSK_PRX_G3"[als_P02][als] last=%d light=%d \n", g_last_al3010_light, g_al3010_light);

		queue_delayed_work(Al3010light_delay_workqueue, &Al3010light_debounce_work, HZ);
#if 0
		mutex_lock(&g_al3010_data_as->lock);
		/* Set Low threshold*/
		i2c_smbus_write_byte_data(g_al3010_data_as->client, AL3010_LOW_THD_LSB, (g_thd[level] * CNT_RESOLUTION) & 0xFF);
		lsb = i2c_smbus_read_byte_data(g_al3010_data_as->client, AL3010_LOW_THD_LSB);

		i2c_smbus_write_byte_data(g_al3010_data_as->client, AL3010_LOW_THD_MSB, ((g_thd[level] * CNT_RESOLUTION) >> 8 ) & 0xFF);
		msb = i2c_smbus_read_byte_data(g_al3010_data_as->client, AL3010_LOW_THD_MSB);

		p_als_threshold_lo = ((msb <<8) | lsb )/ CNT_RESOLUTION;
		printk(DBGMSK_PRX_G2"[als_P02]-- al3010_get_high_threshold_value: %d ,msb=%d, lsb=%d\n"
											,p_als_threshold_lo, msb, lsb);

		/* Set High threshold*/
		i2c_smbus_write_byte_data(g_al3010_data_as->client, AL3010_HIGH_THD_LSB, (g_thd[level + 1] * CNT_RESOLUTION) & 0xFF);
		lsb = i2c_smbus_read_byte_data(g_al3010_data_as->client, AL3010_HIGH_THD_LSB);

		i2c_smbus_write_byte_data(g_al3010_data_as->client, AL3010_HIGH_THD_MSB, ((g_thd[level + 1] * CNT_RESOLUTION) >> 8 ) & 0xFF);
		msb = i2c_smbus_read_byte_data(g_al3010_data_as->client, AL3010_HIGH_THD_MSB);

		p_als_threshold_hi = ((msb <<8) | lsb )/ CNT_RESOLUTION;
		printk(DBGMSK_PRX_G2"[als_P02]++ al3010_get_low_threshold_value: %d ,msb=%d, lsb=%d\n"
											,p_als_threshold_hi, msb, lsb);
		/*Trun on al3010*/
		for(indx = 0; indx<5; indx++) {
			ret = i2c_smbus_write_byte_data(g_al3010_data_as->client, AL3010_MODE_COMMAND, AL3010_POW_UP);
			if(!ret) {
				printk(DBGMSK_PRX_G2"[al3010][als] switch on al3010 success\n");
				break;
			}else if ( indx >= 5 )
				printk("[al3010][als] %s: switch on al3010 fail\n",__FUNCTION__);
			else
				printk("[al3010][als] i2c error retry = %d\n",indx);
		}
		al3010_interrupt_busy = false;

		/*Check interrupt state (Read only)*/
		ret = i2c_smbus_read_byte_data(g_al3010_data_as->client, AL3010_INT_STATUS);

		if (ret == 0)
			printk(DBGMSK_PRX_G2"[al3010][als] P02 light sensor interrupt is cleared\n");
		else{
			printk(DBGMSK_PRX_G2"[al3010][als] P02 light sensor interrupt is triggered\n");
			/*Release interrupt trigger*/
			i2c_smbus_read_byte_data(g_al3010_data_as->client, AL3010_ADC_MSB);
		}
		
		mutex_unlock(&g_al3010_data_as->lock);
#endif
	}
}

/*
 * sysfs layer
 */

/* range */
static ssize_t al3010_show_range(struct device *dev, struct device_attribute *attr, char *buf)
{
	if ( g_AlsP01ProbeError == 0 && g_bIsP01Attached )	{
		struct i2c_client *client = to_i2c_client(dev);
		printk(DBGMSK_PRX_G2"[als_P01] al3010_show_range: %d\n", al3010_get_range(client));
		
		return sprintf(buf, "%i\n", al3010_get_range(client));
	}else
		return 0;
}

static ssize_t al3010_store_range(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if ( g_AlsP01ProbeError == 0 && g_bIsP01Attached )	{
		struct i2c_client *client = to_i2c_client(dev);
		unsigned long val;
		int ret;

		printk(DBGMSK_PRX_G2"[als_P01] al3010_store_range\n");

		if ((strict_strtoul(buf, 10, &val) < 0) || (val > 3))
			return -EINVAL;

		printk(DBGMSK_PRX_G2"[als_P01] al3010_store_range: %lu\n", val);
		ret = al3010_set_range(client, val);
		if (ret < 0)
			return ret;

		return count;
	}else
		return 0;
	
}

static DEVICE_ATTR(range, S_IWUSR | S_IRUGO,
		   al3010_show_range, al3010_store_range);

#ifdef ASUS_FACTORY_BUILD
/* calibration */
static int al3010_show_calibration_200(struct device *dev, struct device_attribute *attr, char *buf)
{
	int p0_calibration_data = 0;
	
	if ( g_bIsP01Attached )	{
		p0_calibration_data = AX_MicroP_readKDataOfLightSensor();

		p0_calibration_data = p0_calibration_data & 0xFFFFFFFF;
		
		printk(DBGMSK_PRX_G2"[als_P01] al3010_show_gait_calibration: %d.%d\n"
					, p0_calibration_data/a_als_calibration_accuracy
					, p0_calibration_data%a_als_calibration_accuracy);
		
		return sprintf(buf, "%d.%d\n"
					, p0_calibration_data/a_als_calibration_accuracy
					, p0_calibration_data%a_als_calibration_accuracy);

	}else	{
			printk("[als_P01] Without P03\n");
			return -1;
	}
}

static int al3010_store_calibration_200(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	p_als_low_calibration_adc = 0;

	if ( g_bIsP01Attached )	{
		printk(DBGMSK_PRX_G2"[als_P01] al3010_store_resolution\n");

		if ( (strict_strtoul(buf, 10, &val) < 0) )
			return -EINVAL;

		p_als_low_calibration_adc = (int)val;

		printk("[als_P01] al3010 Get low calibration adc value : %d\n", p_als_low_calibration_adc );

		return p_als_low_calibration_adc;
	}
	else	{
			printk("[als_P01] Without P03\n");
			return -1;
	}

	return p_als_low_calibration_adc;
}

static DEVICE_ATTR(calibration_200, S_IRWXU | S_IRWXG | S_IRWXO,
		   al3010_show_calibration_200, al3010_store_calibration_200);


static ssize_t al3010_show_calibration_1000(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint64_t p0_calibration_data = 0;
	u32 p0_shift_calibration_data = 0;
	
	if ( g_bIsP01Attached )	{
		p0_calibration_data = AX_MicroP_readKDataOfLightSensor();
		p0_shift_calibration_data = p0_calibration_data >> 32;

		printk(DBGMSK_PRX_G2"[als_P01] al3010_show_shift_calibration: %d\n", 
				p0_shift_calibration_data );
		return sprintf(buf, "%d\n", (int)(p0_shift_calibration_data) );
		
	}else	{
			printk("[als_P01] Without P03\n");
			return -1;
	}
}

static int al3010_store_calibration_1000(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	unsigned long val;
	uint64_t p0_calibration_data = 0;
	uint64_t p0_shift_calibration_team_data = 0;
	u32 p0_shift_calibration_data = 0;

	
	p_als_high_calibration_adc = 0;

	if ( g_bIsP01Attached )	{
		printk(DBGMSK_PRX_G2"[als_P01] al3010_calibration_final\n");
		
		if ( (strict_strtoul(buf, 10, &val) < 0) )
			return -EINVAL;
		
		p_als_high_calibration_adc = (int)val;

		printk("[als_P01] al3010 Get Hight calibration adc value : %d\n", p_als_high_calibration_adc );

		/*Calibration operation*/
		g_al3010_light_calibration = 
			p_als_calibration_lux*a_als_calibration_accuracy /
						( p_als_high_calibration_adc - p_als_low_calibration_adc );

		g_al3010_light_shift_calibration = 
			1000 - ( p_als_high_calibration_adc*g_al3010_light_calibration/a_als_calibration_accuracy);

		if ( g_al3010_light_calibration > 0xFFFFFFFF )
			g_al3010_light_calibration = a_als_calibration_accuracy;
		
		printk("[als_P01] al3010 Set shift calibration value : %d\n", g_al3010_light_shift_calibration);

		p0_shift_calibration_team_data = g_al3010_light_shift_calibration;

		p0_calibration_data = ((p0_shift_calibration_team_data << 32 )  | g_al3010_light_calibration) ;

		p0_shift_calibration_data = p0_calibration_data >> 32;

		printk("[als_P01] al3010 Set Pad calibration value : 0x%8x%8x\n", p0_shift_calibration_data, g_al3010_light_calibration );

		err = AX_MicroP_writeKDataOfLightSensor( p0_calibration_data );
		if ( err == 0 )	{
			printk(DBGMSK_PRX_G2"[als_P01] al3010 calibration success\n");
			return p_als_high_calibration_adc;			
		}
		else	{
			printk("[als_P01] al3010 calibration fail\n");
			return -1;
		}
	}
	else	{
		printk("[als_P01] Without Pad\n");
		return -1;
	}

	return p_als_high_calibration_adc;
}

static DEVICE_ATTR(calibration_1000, S_IRWXU | S_IRWXG | S_IRWXO,
		   al3010_show_calibration_1000, al3010_store_calibration_1000);
#endif


/* mode */
static ssize_t al3010_show_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	if ( g_AlsP01ProbeError == 0 && g_bIsP01Attached )	{
		struct i2c_client *client = to_i2c_client(dev);
		printk(DBGMSK_PRX_G2"[als_P01] al3010_show_mode: %d\n", al3010_get_mode(client));

		return sprintf(buf, "%d\n", al3010_get_mode(client));
	}else
		return 0;
}

static ssize_t al3010_store_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if ( g_AlsP01ProbeError == 0 && g_bIsP01Attached )	{
		struct i2c_client *client = to_i2c_client(dev);
		unsigned long val;
		int ret;
		printk(DBGMSK_PRX_G2"[als_P01] al3010_store_mode\n");

		if ((strict_strtoul(buf, 10, &val) < 0) || (val > 2))
			return -EINVAL;

		printk(DBGMSK_PRX_G2"[als_P01] al3010_store_mode: %lu\n", val);
		ret = al3010_set_mode(client, val);
		if (ret < 0)
			return ret;

		return count;
	}else
		return 0;
}

static DEVICE_ATTR(mode, S_IWUSR | S_IRUGO,
		   al3010_show_mode, al3010_store_mode);


/* power state */
static ssize_t al3010_show_power_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	if ( g_AlsP01ProbeError == 0 && g_bIsP01Attached )	{
		struct i2c_client *client = to_i2c_client(dev);
		printk(DBGMSK_PRX_G2"[als_P01] al3010_show_power_state: %d\n", al3010_get_power_state(client));
		return sprintf(buf, "%d\n", al3010_get_power_state(client));
	}else
		return 0;
}

static ssize_t al3010_store_power_state(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if ( g_AlsP01ProbeError == 0 && g_bIsP01Attached )	{
		unsigned long val;
		int ret;
		printk(DBGMSK_PRX_G2"[als_P01] al3010_store_power_state\n");

		if ((strict_strtoul(buf, 10, &val) < 0) || (val > 1))
			return -EINVAL;

		printk(DBGMSK_PRX_G2"[als_P01] al3010_store_power_state: %lu\n", val);
		ret = set_als_power_state_of_P01( val );

		return ret ? ret : count;
	}else
		return 0;
}

static DEVICE_ATTR(power_state, S_IRWXU | S_IRWXG | S_IROTH,
		   al3010_show_power_state, al3010_store_power_state);


/* adc */
static int al3010_show_lux(struct device *dev,	 struct device_attribute *attr, char *buf)
{
	if ( g_AlsP01ProbeError == 0 && g_bIsP01Attached )	{
		int lux = 0;
		struct i2c_client *client = to_i2c_client(dev);
		printk(DBGMSK_PRX_G2"[als_P01] al3010_show_lux\n");

		/* No LUX data if not operational */
		if (al3010_get_power_state(client) != 0x01)
			return -EBUSY;

		lux = al3010_get_adc_value(client);
		printk(DBGMSK_PRX_G2"[als_P01] al3010_show_lux: %d\n", lux );

		return sprintf(buf, "%d\n", lux);
	}else
		return 0;
}

static DEVICE_ATTR(lux, S_IRUSR | S_IRGRP| S_IROTH, al3010_show_lux, NULL);

static struct attribute *al3010_attributes[] = {
	&dev_attr_range.attr,
#ifdef ASUS_FACTORY_BUILD
	&dev_attr_calibration_200.attr,
	&dev_attr_calibration_1000.attr,
#endif
	&dev_attr_mode.attr,
	&dev_attr_power_state.attr,
	&dev_attr_lux.attr,
	NULL
};

static const struct attribute_group al3010_attr_group = {
    .name = "al3010",
	.attrs = al3010_attributes,
};

#if 0
static int al3010_init_client(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	int i;
	printk(DBGMSK_PRX_G2"[als_P01]++al3010_init_client\n");

	/* read all the registers once to fill the cache.
	* if one of the reads fails, we consider the init failed */
	for (i = 0; i < ARRAY_SIZE(data->reg_cache); i++) {
		int v = i2c_smbus_read_byte_data(client, al3010_reg[i]);
		if (v < 0)
			return -ENODEV;

		data->reg_cache[i] = v;
	}

	/* set defaults */
	al3010_set_range(client, 0);
	al3010_set_mode(client, 0);
	//al3010_set_power_state(client, 0);

	printk(DBGMSK_PRX_G2"[als_P01]--al3010_init_client\n");

	return 0;
}
#endif

static void al3010_late_resume_delayed_work(struct work_struct *work)
{
	printk("[als_P01] al3010_late_resume, resume ALS:%d\n", g_al3010_switch_earlysuspend);

#ifndef CONFIG_HAS_EARLYSUSPEND
	/*Release suspend flag for without late_resume feature*/
	g_al3010_switch_earlysuspend = 0;
#endif

	if (g_HAL_als_switch_on)
		set_als_power_state_of_P01(g_HAL_als_switch_on);
	else
		set_als_power_state_of_P01(g_al3010_suspend_switch_on || g_al3010_switch_on);

	/*Release interrupt trigger*/
	if(AX_MicroP_Is_3V3_ON())	{
		i2c_smbus_read_byte_data(g_al3010_data_as->client, AL3010_ADC_MSB);
		al3010_interrupt_busy = false;
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void al3010_early_suspend(struct early_suspend *handler)
{
	printk("[als_P01] ++al3010_early_suspend, als:%d\n", g_al3010_switch_on);
	g_al3010_switch_earlysuspend = 1;
	
	if(1 == g_al3010_switch_on) {
		printk(DBGMSK_PRX_G2"[als_P01] al3010_early_suspend, turn off ambient\n");
		set_als_power_state_of_P01(0);
		
	}

	//force report 0 lux
	als_lux_report_event(0);

	printk("[als_P01] --al3010_early_suspend\n");
}


static void al3010_late_resume(struct early_suspend *handler)
{
	printk("[als_P01] ++al3010_late_resume, als:%d\n", g_al3010_switch_on);

	if(1 == g_al3010_switch_earlysuspend) {
		printk(DBGMSK_PRX_G2"[als_P01] al3010_late_resume, P01 attached: %d\n", g_bIsP01Attached);

		if( g_bIsP01Attached && ( g_al3010_suspend_switch_on || g_al3010_switch_on )) {
			printk(DBGMSK_PRX_G2"[als_P01] al3010_late_resume, resume ALS +++\n");
			queue_delayed_work(Al3010light_delay_workqueue, &Al3010light_resume_work, 150 );
		}
	}

	g_al3010_switch_earlysuspend=0;
	printk("[als_P01]--al3010_late_resume\n");
}

static struct early_suspend al3010_early_suspend_desc = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = al3010_early_suspend,
    .resume = al3010_late_resume,
};
#endif

static int al3010_suspend(struct i2c_client *client, pm_message_t mesg)
{
#ifndef CONFIG_HAS_EARLYSUSPEND
	printk("[als_P01] ++al3010_suspend, als:%d\n", g_al3010_switch_on);
	/* Set Flag for mark sensor is in suspend state */
	g_al3010_switch_earlysuspend = 1;
	
	if(1 == g_al3010_switch_on) {
		printk(DBGMSK_PRX_G2"[als_P01] al3010_suspend, turn off ambient\n");
		set_als_power_state_of_P01(0);
		
	}

	if (delayed_work_pending(&Al3010light_resume_work)){
		printk("[als_P01] Resume work still runing, begin cancel resume work \r\n");
		cancel_delayed_work_sync(&Al3010light_resume_work);
		printk("[als_P01] Finish cancel work \r\n");
	}
	printk("[als_P01] --al3010_suspend\n");
#endif

    return 0;
}

static int al3010_resume(struct i2c_client *client)
{
#ifndef CONFIG_HAS_EARLYSUSPEND
	/*Release suspend flag for without late_resume feature*/
	g_al3010_switch_earlysuspend = 0;
#endif
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////
//---Pad feature part---
//
static void lightsensor_attached_pad(struct work_struct *work)
{
	uint64_t p0_calibration_data = 0;
	u32 p0_shift_calibration_data = 0;

	printk(DBGMSK_PRX_G2"[als_P01] lightsensor_attached_pad()++\n");	

	/*Get calibration data*/
	p0_calibration_data = AX_MicroP_readKDataOfLightSensor();
	
	if (g_microp_ver >= 0x906)	{
		p0_shift_calibration_data = p0_calibration_data >> 32;

		g_al3010_light_calibration = (p0_calibration_data & 0xFFFFFFFF);

		g_al3010_light_shift_calibration = (int)p0_shift_calibration_data;

		/*Check Calibration value*/
		if ( g_al3010_light_calibration > 0x7FFFFFFF  || g_al3010_light_calibration <= 0 )
			g_al3010_light_calibration = 85;

		if ( g_al3010_light_shift_calibration >= 65535  || g_al3010_light_shift_calibration <= -65535 )
			g_al3010_light_shift_calibration = 35;
	}
	else		// For old lightsensor K data
	{
		g_al3010_light_calibration = (p0_calibration_data & 0x0000ffff);

		if ( (0xf << 28 ) & p0_calibration_data )
			g_al3010_light_shift_calibration = (int)( (p0_calibration_data >> 16) | (0xffff << 16 ) );
		else
			g_al3010_light_shift_calibration = (int)(p0_calibration_data >> 16);

		/*Check Calibration value*/
		if ( g_al3010_light_calibration > 4095  || g_al3010_light_calibration <= 0 )
			g_al3010_light_calibration = 85;

		if ( g_al3010_light_shift_calibration >= 255  || g_al3010_light_shift_calibration <= -255 )
			g_al3010_light_shift_calibration = 35;

		g_al3010_light_calibration = g_al3010_light_calibration*100;
	}
	
		
	printk("[als_P01] al3010 set calibration and shift: %d.%d , %d \n",
					g_al3010_light_calibration/a_als_calibration_accuracy
					, g_al3010_light_calibration%a_als_calibration_accuracy
					, g_al3010_light_shift_calibration );
			
	g_bIsP01Attached = true;
	if ( g_AlsP01ProbeError != 0 )	{
		printk("[als_P01] Lightsensor add i2c error!\n");

		//report uevent if prob error
		printk("[als_P01] al3010 prob error, report uevent to framework\n");
		switch_set_state(&ls_switch_dev, P01_EVENT_NOTIFY_LIGHTSENSOR_ERROR);
		g_AlsP01ProbeError = -1;

		return;
	}

	if (g_HAL_als_switch_on) {
		g_al3010_suspend_switch_on = 0;

		/*Switch Phone light value to Pad*/
		//g_last_al3010_light = g_cm36283_light;
		
		/*Wait al3010 stable*/
		queue_delayed_work(Al3010light_delay_workqueue, &Al3010light_resume_work, 500 );
	}

	printk(DBGMSK_PRX_G2"[als_P01] lightsensor_attached_pad()--\n");

	return;
}
EXPORT_SYMBOL(lightsensor_attached_pad);

int lightsensor_detached_pad(void)
{
	printk(DBGMSK_PRX_G2"[als_P01] lightsensor_detached_pad()++\n");

	//turn al3010 off
	if( g_al3010_switch_on ) {
		set_als_power_state_of_P01(0);
		//g_cm36283_light = g_al3010_light;
		//printk(DBGMSK_PRX_G2"[als_P01] lightsensor_detached_pad, switch to cm36238 : %d lux\n", g_cm36283_light);
		g_al3010_switch_on = false;
	}

	g_bIsP01Attached = false;

	printk(DBGMSK_PRX_G2"[als_P01] lightsensor_detached_pad()--\n");

	return 0;
}
EXPORT_SYMBOL(lightsensor_detached_pad);

static int lightsensor_pad_mp_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	switch (event) {
		case P01_ADD:
			printk(DBGMSK_PRX_G2"[als_P01][MicroP] P01_ADD \r\n");                
			queue_delayed_work(al3010light_workqueue, &al3010_attached_P02_work, HZ );
			return NOTIFY_DONE;

		case P01_REMOVE:
			printk(DBGMSK_PRX_G2"[als_P01][MicroP] P01_REMOVE \r\n");
			lightsensor_detached_pad();
			return NOTIFY_DONE;

		case P01_LIGHT_SENSOR:
			if(AX_MicroP_getOPState() == st_MICROP_Active)	{
				printk("[als_P01][MicroP] P01_ISR \r\n");
				if (work_pending(&al3010_ISR_work)){
					printk("[als_P01] Begin cancel work \r\n");
					cancel_work_sync(&al3010_ISR_work);
					printk("[als_P01] Finish cancel work \r\n");
				}
				if ( !al3010_interrupt_busy )
					queue_work(al3010light_workqueue ,&al3010_ISR_work);
				else
					printk(DBGMSK_PRX_G2"[als_P01] Inerrupt busy \r\n");
			}
			return NOTIFY_DONE;
		default:
			return NOTIFY_DONE;
		}
}

static struct notifier_block lightsensor_pad_mp_notifier = {
       .notifier_call = lightsensor_pad_mp_event,
        .priority = AL3010_LIGHTSENSOR_MP_NOTIFY,
};

/*.......................................Al3010 prob port.........................................*/
static int al3010_input_init(void)
{
	int ret = 0;
	struct input_dev *input_dev_as = NULL;

	input_dev_as = input_allocate_device();
	if (!input_dev_as) {
		ret = -ENOMEM;
		printk("[al3010]: Failed to allocate input_data device\n");
		goto error_1;
	}

	input_dev_as->name = "ASUS Pad Lightsensor";
	input_dev_as->id.bustype = BUS_I2C;
	input_set_capability(input_dev_as, EV_ABS, ABS_MISC);
	__set_bit(EV_ABS, input_dev_as->evbit);
	__set_bit(ABS_MISC, input_dev_as->absbit);
	input_set_abs_params(input_dev_as, ABS_MISC, 0, 1048576, 0, 0);
	input_set_drvdata(input_dev_as, g_al3010_data_as);

	ret = input_register_device(input_dev_as);
	if (ret < 0) {
		input_free_device(input_dev_as);
		goto error_1;
	}

	g_al3010_data_as->input_dev = input_dev_as;
	g_al3010_data_as->polling = 0;
	g_al3010_data_as->poll_interval_ms = 100;
	g_al3010_data_as->event_threshold = 1000;

	if ( g_ASUS_hwID >= A91_SR5 )
		ret = als_lux_report_event_register(g_al3010_data_as->input_dev);

error_1:
	return ret;
}

static int __devinit al3010_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int err = 0;
	g_AlsP01ProbeError = -1;

	/*......................................init port......................................*/
	printk("[als_P01] al3010_init++\n");
	
	al3010light_workqueue = create_singlethread_workqueue("al3010light_wq");
	INIT_DELAYED_WORK(&al3010_attached_P02_work, lightsensor_attached_pad);
	INIT_WORK(&al3010_ISR_work, mp_als_interrupt_handler);

	/*For resume and debounce I2C issue*/
	Al3010light_delay_workqueue = create_singlethread_workqueue("Al3010light_delay_wq");
	INIT_DELAYED_WORK( &Al3010light_resume_work, al3010_late_resume_delayed_work);
	INIT_DELAYED_WORK( &Al3010light_debounce_work, mp_als_interrupt_delay_work);
	INIT_DELAYED_WORK(&al3010_ISR_delay_work, mp_als_interrupt_handler);
	wake_lock_init(&pad_lightsensoer_wake_lock, WAKE_LOCK_SUSPEND, "al3010_wake_lock");

	//Disable P01 attached temporarily for 1st ICS check-in
	register_microp_notifier(&lightsensor_pad_mp_notifier);
	notify_register_microp_notifier(&lightsensor_pad_mp_notifier, "al3010");

	printk(DBGMSK_PRX_G2"[als_P01] al3010_init--\n");

	/*......................................Driver prob port......................................*/
	printk(DBGMSK_PRX_G2"[als_P01]++al3010_probe\n");

	g_al3010_data_as = kzalloc(sizeof(struct al3010_data), GFP_KERNEL);
	if (!g_al3010_data_as)	{
		g_AlsP01ProbeError = -ENOMEM;
		return -ENOMEM;
	}

	// registered as switch device
	err = switch_dev_register(&ls_switch_dev);
	if (err < 0)
		goto exit_kfree;

   	// Check adapter supports everything 
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))	{
		g_AlsP01ProbeError = -EIO;
		goto exit_kfree;
	}

	//store i2c client structure
	g_al3010_data_as->client = client;
	i2c_set_clientdata(client, g_al3010_data_as);
	g_al3010_data_as->adb = 0;

	printk("[al3010] Register input device...\n");
	if( al3010_input_init() != 0 )
		goto exit_kfree;
	
	mutex_init(&g_al3010_data_as->lock);

	printk(DBGMSK_PRX_G2"[als_P01]++al3010_probe: create_group\n");
	/* register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &al3010_attr_group);
	if (err)
		goto exit_kfree;

	err = dev_info(&client->dev, "driver version %s enabled\n", DRIVER_VERSION);

	if (err)
		printk(DBGMSK_PRX_G2"[als_P01] ambientdl create sysfile fail.\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend( &al3010_early_suspend_desc );
#endif

#ifdef CONFIG_I2C_STRESS_TEST
       i2c_add_test_case(client, "Sensor_Al3010",ARRAY_AND_SIZE(gSensorTestCaseInfo));
#endif

	printk("[als_P01]--al3010_probe\n");

	g_AlsP01ProbeError = 0;
	return 0;

exit_kfree:
	g_AlsP01ProbeError = err;
	kfree(g_al3010_data_as);
	printk("[als_P01]--al3010_probe fail : %d\n", err);
	return err;
}

static int __devexit al3010_remove(struct i2c_client *client)
{
	destroy_workqueue(al3010light_workqueue);
	destroy_workqueue(Al3010light_delay_workqueue);
	
	sysfs_remove_group(&client->dev.kobj, &al3010_attr_group);
	al3010_set_power_state(client, 0);
	switch_dev_unregister(&ls_switch_dev);
	kfree(g_al3010_data_as);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend( &al3010_early_suspend_desc );
#endif
	return 0;
}

static const struct i2c_device_id al3010_id[] = {
	{ "al3010", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, al3010_id);

static struct of_device_id al3010_match_table[] = {
	{ .compatible = "dyna,al3010",},
	{},
};

static struct i2c_driver al3010_driver = {
	.driver = {
		.name	= AL3010_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = al3010_match_table,
	},
	.suspend = al3010_suspend,
	.resume	= al3010_resume,
	.probe	= al3010_probe,
	.remove	= __devexit_p(al3010_remove),
	.id_table = al3010_id,
};

MODULE_AUTHOR("ASUS");
MODULE_DESCRIPTION("version v1.1");

module_i2c_driver(al3010_driver);

