/**
 *\mainpage
 * ADUX1020 driver
 \n
 * @copyright 2013 Analog Devices Inc.
 \n
 * Licensed under the GPL version 3 or later.
 * \date	April-2013
 * \version	Driver 1.0
 * \version	Linux 3.0.15
 * \version	Android 4.0.4
*/
/**
@file adux1020.c
@brief ADUX1020	- Low level driver  Source 'C' File
*/
#include <linux/types.h>
#include <linux/workqueue.h>	/* workqueue related header */
#include <linux/kernel.h>	/* kernel related header */
#include <linux/err.h>		/* error related macro */
#include <linux/slab.h>		/* memory allocation related header */
#include <linux/device.h>	/* device related header */
#include <linux/input.h>	/* input related header */
#include <linux/delay.h>	/* sleep realted header */
#include <linux/irq.h>		/*  irq related functionality */
#include <linux/interrupt.h>	/* interrupt related functionality  */
#include <linux/sysfs.h>	/* sysfs related dunctionality */
#include <linux/uaccess.h>	/* filp open usage */
#include <linux/stat.h>		/* user mode permission */
#include <linux/init.h>		/* module initialization api */
#include <linux/module.h>	/* module functionality */
#include <linux/i2c.h>		/* i2c related functionality */
#include <linux/jiffies.h>	/* jiffies related macro and api*/
#include <linux/kthread.h>

#include <linux/of_gpio.h>
#include <linux/input/adux1020.h> /**< adux1020 chip related header*/

#include "linux/proximity_class.h"
#define ADUX1020_PROXIMITY_INT		74
#define PROXIMITY_PWR_EN				3
static int calibration_flag = 0;

/************* FILP - CONFIG **************************************/
/**
	@brief Macro to fetch configuration value
*/
#define		CONFIG(VALUE)			CONFIG_##VALUE##_VALUE
/**
	@brief General configuration value
*/
#define		CONFIG_GENERAL_VALUE            1
/**
	@brief Proximity configuration value
*/
#define		CONFIG_PROXIMITY_VALUE          2
/**
	@brief Gesture configuraiton value
*/
#define		CONFIG_GESTURE_VALUE            3
/**
	@brief Sample XYI configuration value
*/
#define		CONFIG_SAMPLE_XYI_VALUE         4
/**
	@brief Sample RAW configuration value
*/
#define		CONFIG_SAMPLE_RAW_VALUE         5
/*	@brief directory based on android*/
/**
	@brief General configuration path
*/
#define		GENERAL_CONFIG		"/data/misc/general_configuration"
/**
	@brief Proximity configuration path
*/
#define		PROXIMITY_CONFIG	"/data/misc/proximity_configuration"
/**
	@brief Gesture configuration path
*/
#define		GESTURE_CONFIG		"/data/misc/gesture_configuration"
/**
	@brief Sample XYI configuration path
*/
#define		SAMPLE_XYI_CONFIG	"/data/misc/sample_configuration"
/**
	@brief Sample RAW configuration path
*/
#define		SAMPLE_RAW_CONFIG	"/data/misc/sample_raw_configuration"
/**
	@brief Clock configuration path
*/
#define		CLOCK_CALIB_CONFIG	"/data/misc/clock_calibration"

/******************************************************************/

/**************** ADUX1020_CHIP_REG_ADDR ****************************/
/**
	@brief CHIP_ID VERSION Address.
*/
#define		ADUX_CHIP_ID_VERSION_ADDR	0x8
/**
	@brief Oscillator Calibration output register
*/
#define		ADUX_OSC_CAL_OUT		0xA
/**
	@brief Software Reset setting register
*/
#define		ADUX_SW_RESET			0xF
/**
	@brief ADUX1020 Oscillator set register 1
*/
#define		ADUX_OSCS_1			0x18
/**
	@brief ADUX1020 Oscillator set register 3
*/
#define		ADUX_OSCS_3			0x1A
/**
	@brief I2C control and speed mode register
*/
#define		ADUX_I2C_1			0x1E
/**
	@brief ADUX1020 FIFO threshold register
*/
#define		ADUX_I2C_2			0x1F
/**
	@brief ADUX1020 Proximity Type register
*/
#define		ADUX_PROX_TYPE			0x2F
/**
	@brief ADUX1020 calibration enable register
*/
#define		ADUX_TEST_MODES_1		0x30
/**
	@brief ADUX1020 power down control and Slave address
	configurable register
*/
#define		ADUX_TEST_MODES_3		0x32
/**
	@brief ADUX1020 operating mode register
*/
#define		ADUX_OP_MODE_ADDR		0x45
/**
	@brief ADUX1020 interrupt mask register
*/
#define		ADUX_INT_MASK_ADDR		0x48
/**
	@brief ADUX1020 interrupt status register
*/
#define		ADUX_INT_STATUS_ADDR		0x49
/**
	@brief ADUX1020 data buffer register
*/
#define		ADUX_DATA_BUFFER		0x60
/*******************************************************************/
/**************** ADUX1020 Driver **********************************/
/**
	@brief vendor name for class create
*/
#define		VENDOR				"ADUX1020"
/**
	@brief Gesture input device name
*/
#define		ADUX1020_INPUT1_NAME		"adux1020_gesture"
/**
	@brief Proximity1 input device name
*/
#define		ADUX1020_INPUT2_NAME		"adux1020_proximity1"
/**
	@brief Proximity2 input device name
*/
#define		ADUX1020_INPUT3_NAME		"adux1020_proximity2"
/**
	@brief Define number of times the key event to send
*/
#define		GEST_KEY_REPEAT_RANGE		1
/**
	@brief ADUX1020 fifo size mask
*/
#define		FIFO_SIZE			0x7F00
/**
	@brief Self Resetting set enable
*/
#define		SELF_RESETTING			0x8000
/**
	@brief Maximum retry count for retrying a register
*/
#define		MAX_RETRY			5
/*******************************************************************/
/**************** ADUX1020 CHIP REG VALUE **************************/
/**
	@brief Force Clock On enable value
*/
#define		FORCE_CLOCK_ON			0xF4F
/**
	@brief Force Clock Reset value
*/
#define		FORCE_CLOCK_RESET		0x40
/*******************************************************************/
/**************** I2C - POWER UP ***********************************/
/**
	@brief chip ID mask value
*/
#define		CHIP_ID_MASK			0x0FFF
/**
	@brief macro to get version number
*/
#define		GET_VERSION(value)		((value & 0xF000) >> 12)
/**
	@brief i2c speed mask value
*/
#define		I2C_SPEED_MASK			0x1000
/**
	@brief ADUX1020 software reset set value
*/
#define		SW_RESET_SET			0x0001
/*******************************************************************/
/**************** I2C - READ, WRITE ********************************/
/**
	@brief condition failed
*/
#define		FAILED				0
/**
	@brief condition success
*/
#define		SUCCESS				1
/**
	@brief size of register in bytes
*/
#define		ADUX1020_REG_SIZE_BYTES		2
/**
	@brief size of register in byte
*/
#define		ADUX1020_REG_SIZE_1BYTE		1
/**
	@brief retry count for loop
*/
#define		RETRY_CNT			5
/**
	@brief maximum data count for i2c write
*/
#define		MAX_DATA_CNT			8
/**
	@brief ADUX1020 big endian format
*/
#define		ADUX1020_BIG_ENDIAN		1
/**
	@brief ADUX1020 little endian format
*/
#define		ADUX1020_LITTLE_ENDIAN		2
/**
	@brief ADUX1020 address range limit
*/
#define		ADUX_ADDRESS_RANGE		127
/**
	@brief data size in adux1020 mentioned in bytes
*/
#define		DATA_SIZE			2
/**
	@brief word size in adux1020 mentioned in word(16bits)
*/
#define		WORD_SIZE			1
/**
	@brief message count used in i2c transfer method
*/
#define		MSG_EXECUTED			2
/******************************************************************/
/***************** ADUX1020 CLOCK *********************************/
#define		ABORT				0xE
/**
	@brief convert variable into absolute value
*/
#define		MATH_ABS(x)			((x <= -1) ? (x * -1) : x)
/**
	@brief macro to define increase trim value
*/
#define		INCREASE_TRIM			1
/**
	@brief macro to define decrease trim value
*/
#define		DECREASE_TRIM			2
/**
	@brief ADUX1020 oscillator calibration enable
*/
#define		OSC32M_CAL_ENABLE		0x20
/**
	@brief ADUX1020 oscillator calibration disable
*/
#define		OSC32M_CAL_DISABLE		0xFFDF
/******************************************************************/
/***************** OPERATING MODES ********************************/
/**
	@brief true conditions
*/
#define		TRUE				1
/**
	@brief false conditions
*/
#define		FALSE				-1

/**
	@brief OFF state used in proximity
*/
#define		OFF				0x0
/**
	@brief ON state used in proximity
*/
#define		ON				0x1
/**
	@brief UNKNOWN state used in proximity
*/
#define		UNKNOWN				0x2
/**
	@brief ADUX1020 Proximity operating mode value from user space
*/
#define		PROXIMITY			0x1
/**
	@brief ADUX1020 Proximity none operating mode value from user space
*/
#define		PROXIMITY_NONE			0x1
/**
	@brief ADUX1020 Proximity I operating mode value from user space
*/
#define		PROXIMITY_I			0x2
/**
	@brief ADUX1020 Force operating mode value from driver
*/
#define		FORCE				0xE
/**
	@brief ADUX1020 Idle operating mode value from user space
*/
#define		IDLE				0xF

/**
	@brief ADUX1020 Gesture operating mode value from user space
*/
#define		GESTURE				0x10
/**
	@brief ADUX1020 Sample operating mode value from user space
*/
#define		SAMPLE				0x20
/**
	@brief ADUX1020 Sample XYI operating mode value from user space
*/
#define		SAMPLE_XYI			0x21
/**
	@brief ADUX1020 Sample RAW operating mode value from user space
*/
#define		SAMPLE_RAW			0x20
/**
	@brief ADUX1020 Fifo operating mode value from user space
*/
#define		FIFO				0x30

/**
	@brief ADUX1020 Fifo prevent mask value
*/
#define		R_FIFO_PREVENT_MASK		(0xFFFF & ~(0x1 << 8))

/**
	@brief ADUX1020 Pack start mask value
*/
#define		R_PACK_START_MASK		(0xFFFF & ~(0x1 << 9))

/**
	@brief ADUX1020 Sample out mask value
*/
#define		R_SAMP_OUT_MASK			(0xFFFF & ~(0x1 << 12))

/**
	@brief ADUX1020 Fifo prevent enable macro
*/
#define		R_FIFO_PREVENT_EN(value)	(value << 8)

/**
	@brief ADUX1020 Pack start enable macro
*/
#define		R_PACK_START_EN(value)		(value << 9)

/**
	@brief ADUX1020 Sample out mode enable macro
*/
#define		R_SAMP_OUT_MODE(value)		(value << 12)

/**
	@brief ADUX1020 Read outdata flag enable macro
*/
#define		R_RDATA_FLAG_EN			(0x1 << 11)
/**
	@brief ADUX1020 Read back data out enable macro
*/
#define		R_RDOUT_MODE			(0x1 << 13)

/**
	@brief Off operating mode value to ADUX1020
*/
#define		OP_OFF_DATA_NONE		(0x0000)

/**
	@brief Idle operating mode value to ADUX1020
*/
#define		OP_IDLE_DATA_NONE		(0xF/*OP_MODE*/)

/**
	@brief Proximity none operating mode value to ADUX1020
*/
#define		OP_PROXIMITY_DATA_NONE		((0x1/*OP_MODE*/) |\
						(0x0/*DATA_MODE*/ << 4) |\
						R_RDATA_FLAG_EN | R_RDOUT_MODE)

/**
	@brief Proximity I operating mode value to ADUX1020
*/
#define		OP_PROXIMITY_DATA_I		((0x1/*OP_MODE*/) |\
						(0x1/*DATA_MODE*/ << 4) |\
						R_RDATA_FLAG_EN | R_RDOUT_MODE)

/**
	@brief Proximity XYI operating mode value to ADUX1020
*/
#define		OP_PROXIMITY_DATA_XYI		((0x1/*OP_MODE*/) |\
						(0x3/*DATA_MODE*/ << 4) |\
						R_RDATA_FLAG_EN | R_RDOUT_MODE)

/**
	@brief Gesture none operating mode value to ADUX1020
*/
#define		OP_GESTURE_DATA_NONE		((0x2/*OP_MODE*/) |\
						(0x2/*DATA_MODE*/ << 4) |\
						R_RDATA_FLAG_EN | R_RDOUT_MODE)

/**
	@brief Sample Raw operating mode value to ADUX1020
*/
#define		OP_SAMPLE_DATA_RAW		((0x8/*OP_MODE*/) |\
						(0x4/*DATA_MODE*/ << 4) |\
						R_RDATA_FLAG_EN | R_RDOUT_MODE)

/**
	@brief Sample XYI operating mode value to ADUX1020
*/
#define		OP_SAMPLE_DATA_XYI		((0x8/*OP_MODE*/) |\
						(0x3/*DATA_MODE*/ << 4) |\
						R_RDATA_FLAG_EN | R_RDOUT_MODE)

/**
	@brief Operating mode Interrupt mask value
*/
#define		OP_MODE_INT_MASK		0x00FF

/**
	@brief General mode Interrupt mask value
*/
#define		INTR_GENERAL			0x00FF
/**
	@brief Off mode Interrupt mask value
*/
#define		INTR_OFF			0x0000
/**
	@brief Idle mode Interrupt mask value
*/
#define		INTR_IDLE			0x0000
/**
	@brief Proximity Interrupt mask value
*/
#define		INTR_PROXIMITY			0x000F
/**
	@brief Gesture Interrupt mask value
*/
#define		INTR_GESTURE			0x0010
/**
	@brief Sample Interrutp mask value
*/
#define		INTR_SAMPLE			0x0020
/**
	@brief Watchdog Interrupt mask value
*/
#define		INTR_WATCHDOG			0x0040
/**
	@brief Fifo Interrupt mask value
*/
#define		INTR_FIFO			0x0080

/**
	@brief macro to fetch operating mode value from Operating
	mode from user space and corresponding data out mode.
*/
#define		SET_MODE_VALUE(OP_MODE, DATA_MODE)\
					OP_##OP_MODE##_DATA_##DATA_MODE

/**
	@brief macro to fetch Interrupt mask value from Operating
	mode from user space.
*/
#define		SET_INTR_MASK_VALUE(OP_MODE)	\
				((~(INTR_##OP_MODE)) & OP_MODE_INT_MASK)

/**
	@brief macro to fetch Interrupt status value from Operating
	mode from user space.
*/
#define		INTR_STATUS_MASK(OP_MODE)	(INTR_##OP_MODE)

/**
	@brief Proximity 1 mask value
*/
#define		PROXIMITY_1			0x3
/**
	@brief Proximity 2 mask value
*/
#define		PROXIMITY_2			0xC
/**
	@brief Proximity ON1 mask value
*/
#define		PROX_ON1			0x1
/**
	@brief Proximity OFF1 mask value
*/
#define		PROX_OFF1			0x2
/**
	@brief Proximity ON2 mask value
*/
#define		PROX_ON2			0x4
/**
	@brief Proximity OFF2 mask value
*/
#define		PROX_OFF2			0x8
/******************************************************************/
/**************** FILP ********************************************/
/**
	@brief Minimum loop to execute
*/
#define		MINI_SIZE			256
/******************************************************************/
/**************** EVENT/BUFFER SIZE *******************************/
/**
	@brief Proximity I buffer size
*/
#define		PROX_I_BUF_SIZE			1
/**
	@brief Gesture buffer size
*/
#define		GEST_BUF_SIZE			2
/**
	@brief Sample XYI buffer size
*/
#define		SAMPLE_XYI_BUF_SIZE		3
/**
	@brief Sample RAW buffer size
*/
#define		SAMPLE_RAW_BUF_SIZE		4
/**
	@brief Maximum buffer size
*/
#define		MAX_BUFFER			72
/******************************************************************/
/**************** DEBUG PRINT *************************************/

#define		ADUX_DBG_PRINT	1
/**
  @def ADUX_DBG_PRINT
  @brief ADUX_DBG_PRINT is defined during build to enable dbg print
 */
#ifdef ADUX_DBG_PRINT
#define ADUX1020_dbg(format, arg...)    printk(/*KERN_DEBUG*/ "ADUX1020 : "\
							format,	##arg)
#else
#define ADUX1020_dbg(format, arg...)    if (0)
#endif
/**
	@def ADUX_INFO_PRINT
	@brief ADUX_INFO_PRINT is defined during build to enable info print
 */
#ifdef ADUX_INFO_PRINT
#define ADUX1020_info(format, arg...)    printk(KERN_INFO "ADUX1020 : "\
							format, ##arg)
#else
#define ADUX1020_info(format, arg...)	if (0)
#endif
/**
	@def ADUX_ERR_PRINT
	@brief ADUX_ERR_PRINT is defined during build to enable err print
 */
#ifdef ADUX_ERR_PRINT
#define ADUX1020_err(format, arg...)    printk(KERN_ERR "ADUX1020 Error : "\
							format, ##arg)
#else
#define ADUX1020_err(format, arg...)	if (0)
#endif
/******************************************************************/

/**
	@brief ADUX1020 Indexes for Gesture sign
*/
typedef enum {
	G_SIGN_INVALID = 0,
	G_SIGN_WE,
	G_SIGN_EW,
	G_SIGN_SN,
	G_SIGN_NS,
	G_SIGN_WEH,
	G_SIGN_EWH,
	G_SIGN_SNH,
	G_SIGN_NSH,
	G_SIGN_WEF,
	G_SIGN_EWF,
	G_SIGN_SNF,
	G_SIGN_NSF,
	G_SIGN_HOVER = 0x1A,
	G_SIGN_LOCK
} ENUM_GESTURE_INDEX;

/**
	@brief ADUX1020 Keymap table
*/
struct adux1020_key_map {
	u32 Inp_KeyMap_Table[NUM_OF_GESTURE_VALUE];
	u16 Inp_KeyRepeat_Table[NUM_OF_GESTURE_VALUE];
	/*Linked list to be added in future to support multiple keymap */
};

/**
	@brief ADUX1020 Proximity state informations
*/
struct adux_prox_state {
	u8 proximity1;
	u8 proximity2;
};

/**
	@brief ADUX1020 data for gesture
*/
struct adux1020_gest_data {
	unsigned dirX:2;
	unsigned dirY:2;
	unsigned dirZ:1;
	unsigned dntcare:3;
	u8 gest_length;
};

/**
	@brief ADUX1020 Bus Operations
*/
struct adux_bus_ops {
	s32 (*read)(struct device *, u16 reg, u32 len, u16 *data, u32 endian);
	s32 (*write)(struct device *, u16 reg, u16 data, u32 endian);
};

/**
	@brief adux1020 main chip
*/
struct adux1020 {
	struct device *adux_dev;
	struct class *adux_class;
	struct device *adux_dev_attr;
	struct mutex mutex;	/*lock */

	struct adux_platform_data *pdata;
	struct adux_platform_data *runtime_pdata;
	struct adux_platform_data *default_pdata;

	struct input_dev *gest_input;
	struct input_dev *prox1_input;
	struct input_dev *prox2_input;
	struct adux1020_key_map *pt_adux1020_key_map;
	struct work_struct work;

	struct adux_prox_state prox_state;
	struct adux1020_gest_data adux_gest_data;

	struct task_struct *clk_thread;
	s32 irq;
	u32 mode;
	u32 endian;
	u32 KeyMapCode;
	u32 GestOutMode;

	u16 chip_id;
	u16 product;
	u16 version;
	u16 curr_intr;
	u16 fifosize;
	u16 fifosize_byte;
	u16 fifo_th;

	u16 r_fifo_prevent;
	u16 r_pack_start;
	u16 sample_out;

	u16 prox_type;
	u16 clk_status;

	u16 *pu16_databuff;
	/*sysfs register read write */
	u16 sysfs_I2C_regaddr;
	u16 sysfs_I2C_regval;
	u16 sysfslastreadval;

	s32 (*read)(struct device *, u16 reg, u32 len, u16 *data, u32 endian);
	s32 (*write)(struct device *, u16 reg, u16 data, u32 endian);
};

struct adux1020 *adux1020_data;
/**
	@brief ADUX1020 data buffer
*/
static u16 databuffer[MAX_BUFFER];
/**
	@struct adux1020_key_map
	@brief ADUX1020 key map structure
*/
struct adux1020_key_map st_adux1020keymap;
/*************** PLAT DATA ****************************************/
/**
	@struct adux_platform_data
	@brief Platform data for initialization if not provided by the user
*/
static struct adux_platform_data adux1020_default_platform_data = {
	.general_reg_cnt = 54,
	.general_regs = {
			 0x000c000f, 0x00101010, 0x0011004c, 0x00125f0c,
			 0x0013ada5, 0x00140080, 0x00150000, 0x00160600,
			 0x00170000, 0x00180089, 0x00190004, 0x001a428f,
			 0x001b0060, 0x001c2084, 0x001d0000, 0x001e0001,
			 0x001f0000, 0x00200320, 0x00210713, 0x00220320,
			 0x00230113, 0x00240000, 0x0025241f, 0x00262442,
			 0x00270022, 0x00280020, 0x00290300, 0x002a1770,
			 0x002b157c, 0x002c4268, 0x002d2710, 0x002e0000,
			 0x002f0000, 0x00300000, 0x00310000, 0x00320040,
			 0x00330000, 0x0034E400, 0x00388080, 0x00398080,
			 0x003a1e9c, 0x003b1ec8, 0x003c1ec9, 0x003d1f0a,
			 0x003e0000, 0x0040707d, 0x00411f2d, 0x00424000,
			 0x00430000, 0x0044000B, 0x00460033, 0x004800ef,
			 0x00490000, 0x0045000f,
			 },
	.proximity_reg_cnt = 27,
	.proximity_regs = {
			   0x00210713, 0x00220320, 0x00230113, 0x00240000,
			   0x0025241f, 0x00262442, 0x00270022, 0x00280020,
			   0x00290300, 0x002a1770, 0x002b157c, 0x002c4268,
			   0x002d2710, 0x002e0000, 0x002f0000, 0x00300000,
			   0x00310000, 0x00320040, 0x00330000, 0x0034E400,
			   0x00388080, 0x00398080, 0x003a1e9c, 0x003b1ec8,
			   0x003c1ec9, 0x003d1f0a, 0x003e0000, 0x0040707d,
			   0x00411f2d, 0x00424000, 0x00430000, 0x0044000B,
			   0x00460033, 0x00490000,
			   },
	.gesture_reg_cnt = 16,
	.gesture_regs = {
			 0x00210713, 0x00220320, 0x00230113, 0x00262442,
			 0x00280020, 0x003a1e9c, 0x003b1ec8, 0x003c1ec9,
			 0x003d1f0a, 0x003e0000, 0x0040707d, 0x00411f2d,
			 0x00424000, 0x00430000, 0x0044000B, 0x00460033,
			 },
	.sample_xyi_reg_cnt = 16,
	.sample_xyi_regs = {
			    0x00210713, 0x00220320, 0x00230113, 0x00262442,
			    0x00280020, 0x003a1e9c, 0x003b1ec8, 0x003c1ec9,
			    0x003d1f0a, 0x003e0000, 0x0040707d, 0x00411f2d,
			    0x00424000, 0x00430000, 0x0044000B, 0x00460033,
			    },
	.sample_raw_reg_cnt = 16,
	.sample_raw_regs = {
			    0x00210713, 0x00220320, 0x00230113, 0x00262442,
			    0x00280020, 0x003a1e9c, 0x003b1ec8, 0x003c1ec9,
			    0x003d1f0a, 0x003e0000, 0x0040707d, 0x00411f2d,
			    0x00424000, 0x00430000, 0x0044000B, 0x00460033,
			    },
};

/******************************************************************/
/*I2C_INLINE - START **********************************************/
/**
@brief This API is used as an wrapper to call the I2C read
*       @param pst_adux the ADUX device pointer
*	@param reg 8 bit address to be read
*       @return u16
*/
static inline u16
adux1020_read(struct adux1020 *pst_adux, u16 reg)
{
	u16 value;
	s32 ret;
	ret = pst_adux->read(pst_adux->adux_dev,
			     reg, WORD_SIZE, &value, pst_adux->endian);
	if (ret < FAILED)
		value = 0xFFFF;
	return value;
}

/**
@brief This API is used as an wrapper to call the multi I2C read
*       @param pst_adux the ADUX device pointer
*       @param first_reg 8 bit register address to be read
*	@param count Number of data to be read
*	@param buf Buffer to place the read data
*       @return int
*/
static inline int
adux1020_multi_read(struct adux1020 *pst_adux,
		    u16 first_reg, u16 count, u16 *buf)
{
	return pst_adux->read(pst_adux->adux_dev,
			      first_reg, count, buf, pst_adux->endian);
}

/**
@brief This API is used as an wrapper to call the I2C write
*	@param pst_adux the ADUX device pointer
*	@param reg 8 bit register address
*	@param val 16 bit value to be written
*       @return int
*/
static inline int
adux1020_write(struct adux1020 *pst_adux, u16 reg, u16 val)
{
	return pst_adux->write(pst_adux->adux_dev, reg, val, pst_adux->endian);
}

/*I2C_INLINE - END ************************************************/
/*FILP API - START ************************************************/
/**
@brief This API is used to start calibration from user
*       @param filename name of the file
*       @param pst_adux the ADUX device pointer
*	@param config
*       @return s32
*/
static s32
adux_filp_start_calib(s8 *filename, struct adux1020 *pst_adux, u16 config)
{
	struct adux_platform_data *config_reg = pst_adux->runtime_pdata;
	mm_segment_t old_fs;
	struct file *fpt_adux_filp = NULL;
	s32 value;
	s32 ret;
	s32 loop = MINI_SIZE;
	s32 count = 0;
	s32 err = 0;
	u16 addr;
	u16 data;
	u16 loop_cnt = 2;
	u16 feed_cnt = 0;
	u16 test_cnt = 0;
	u16 newline_cnt = 0;
	u32 len = 0;
	u8 buff[12];
	u8 *buf = NULL;
	s8 *temp_buf = NULL;
	s8 *temp_buf1 = NULL;
	loff_t pos = 0;

	memset(buff, '\0', sizeof(buff));

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fpt_adux_filp = filp_open(filename, O_RDONLY, 0666);
	if (IS_ERR(fpt_adux_filp)) {
		ADUX1020_dbg("unable to find de file %ld\n",
			      PTR_ERR(fpt_adux_filp));
		set_fs(old_fs);
		ret = /*PTR_ERR(fpt_adux_filp); */ -1;
		goto err_filp_open1;
	}

	ret = vfs_read(fpt_adux_filp, buff, 12 * sizeof(char), &pos);
	if (ret != 12 * sizeof(char)) {
		ret = -EIO;
		ADUX1020_dbg("No data or reached EOF\n");
		if (count == 0)
			goto err_filp_nodata;
	} else {
		buf = buff;
		while (loop_cnt != 0) {
			feed_cnt = 0;
			newline_cnt = 0;
			temp_buf = strsep((char **)&buf, "\n");
			newline_cnt++;
			temp_buf1 = temp_buf;
			temp_buf1 = strsep((char **)&temp_buf1, "\r");
			if (temp_buf1 != NULL) {
				temp_buf = temp_buf1;
				feed_cnt++;
			} else {
				temp_buf1 = NULL;
			}
			pos = 0;
			ADUX1020_info("TEMP READ - %s\n", buff);
			len += strlen(temp_buf) + feed_cnt + newline_cnt;
			if ((temp_buf[pos] == '0') &&
			    (temp_buf[pos + 1] == 'x' ||
			     temp_buf[pos + 1] == 'X')) {
				if (kstrtoul(&temp_buf[pos + 2],
					     16, (long unsigned int *)&value))
					value = 0;
			} else {
				if (kstrtoul(&temp_buf[pos],
					     10, (long unsigned int *)&value))
					value = 0;
			}
			loop = value;
			ADUX1020_info("Value = %d\n", value);
			loop_cnt--;
		}
		ADUX1020_info("Length = %d\n", len);

	}

	pos = len;

	ADUX1020_info("Check whether it is neccasry to take NEWline cnt\n");
	if ((buff[pos] == '0') &&
	    (buff[pos + 1] == 'x' || buff[pos + 1] == 'X')) {
		ADUX1020_info("Error\n");
	} else {
		pos = pos - newline_cnt - feed_cnt;
		test_cnt = 1;
	}

	do {
		ret = vfs_read(fpt_adux_filp, buff, 12 * sizeof(char), &pos);
		if (ret != 12 * sizeof(char)) {
			ret = -EIO;
			ADUX1020_info("No data or reached EOF\n");
			if (count == 0)
				goto err_filp_nodata;
			if (loop != 0)
				ADUX1020_info(" LAST VAL  - %s\n", buff);
			ret = count;
		}
		if (loop) {
			buf = buff;
			ADUX1020_info("READ - %s\n", buff);
			feed_cnt = 0;
			newline_cnt = 0;
			temp_buf = strsep((char **)&buf, "\n");
			newline_cnt++;
			temp_buf1 = temp_buf;
			temp_buf1 = strsep((char **)&temp_buf1, "\r");
			if (temp_buf1 != NULL) {
				temp_buf = temp_buf1;
				feed_cnt++;
			} else {
				temp_buf1 = NULL;
			}

			if (kstrtoul
			    (temp_buf, 16, (long unsigned int *)&value))
				value = 0;
			addr = (value >> 16) & 0xFFFF;
			data = value & 0xFFFF;
			ADUX1020_info("VALUE  %d\n", value);
			ADUX1020_info("ADDR  0x%04x\tDATA  0x%04x\n",
				      addr, data);
			switch (config) {
			case CONFIG(GENERAL):
				{
					config_reg->general_regs[count] =
					    (u32) value;
					config_reg->general_reg_cnt = count + 1;
				}
				break;
			case CONFIG(PROXIMITY):
				{
					config_reg->proximity_regs[count] =
					    (u32) value;
					config_reg->proximity_reg_cnt =
					    count + 1;
				}
				break;
			case CONFIG(GESTURE):
				{
					config_reg->gesture_regs[count] =
					    (u32) value;
					config_reg->gesture_reg_cnt = count + 1;
				}
				break;
			case CONFIG(SAMPLE_XYI):
				{
					config_reg->sample_xyi_regs[count] =
					    (u32) value;
					config_reg->sample_xyi_reg_cnt =
					    count + 1;
				}
				break;
			case CONFIG(SAMPLE_RAW):
				{
					config_reg->sample_raw_regs[count] =
					    (u32) value;
					config_reg->sample_raw_reg_cnt =
					    count + 1;
				}
				break;
			default:
				{
					ADUX1020_info("INVALID INPUT\n");
					err = 1;
				}
				break;
			}
			if (test_cnt != 1)
				pos = pos;
			else
				pos = pos - 1;
			count++;
		}
		loop--;
		if (err == 1)
			break;
	} while (loop > 0);

	if (err != 1) {
		/*print the number of count */
		ADUX1020_info("config cnt %d\n", count);
	} else {
		/*print error */
		ADUX1020_info("FAILED!!\n");
	}

	filp_close(fpt_adux_filp, NULL);

	set_fs(old_fs);
	return ret;
err_filp_nodata:
	return ret;
err_filp_open1:
	return ret;
}

/*FILP API - END **************************************************/
/*COMMON_API - START **********************************************/
/**	@fn cmd_parsing (const char *buf,
	unsigned short cnt, unsigned short *data)
*	@brief This API is used to convert the buf(string)
		into integer or hexadecimal value.
*	@param buf stores the pointer of the string
*	@param cnt store the number of value need to retrive
*	@param data store the pointer of the value derived from the string
*	@return void
*/
static void
cmd_parsing(const char *buf, unsigned short cnt, unsigned short *data)
{

	char **bp = (char **)&buf;
	char *token, minus, parsing_cnt = 0;
	int val;
	int pos;

	while ((token = strsep(bp, " "))) {
		pos = 0;
		minus = false;
		if ((char) token[pos] == '-') {
			minus = true;
			pos++;
		}
		if ((token[pos] == '0') &&
		    (token[pos + 1] == 'x' || token[pos + 1] == 'X')) {
			if (kstrtoul(&token[pos + 2],
				     16, (long unsigned int *)&val))
				val = 0;
			if (minus)
				val *= (-1);
		} else {
			if (kstrtoul(&token[pos],
				     10, (long unsigned int *)&val))
				val = 0;
			if (minus)
				val *= (-1);
		}

		if (parsing_cnt < cnt)
			*(data + parsing_cnt) = val;
		else
			break;
		parsing_cnt++;
	}
}

/*COMMON_API - END ************************************************/
/*FIFO BUFFER READ - START ****************************************/
/**
This function is used for  setting the force calibration of  the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return  s32 return value
*/
static s32
adux1020_set_force_clock(struct adux1020 *pst_adux)
{
	s32 ret;

	ADUX1020_info(" %s\n", __func__);
	/*Andre:  FORCE_CLOCK_ON - 0xF4F */
	ret = adux1020_write(pst_adux, ADUX_TEST_MODES_3, FORCE_CLOCK_ON);
	return ret;
}

/**
This function is used to reset the force clock of the ADUX1020 chip
@param pst_adux  pointer to ADUX1020 chip structure
@return s32 reset force clock status of the ADUX1020 chip
*/
static s32
adux1020_reset_force_clock(struct adux1020 *pst_adux)
{
	s32 ret;

	ADUX1020_info(" %s\n", __func__);
	ret = adux1020_write(pst_adux, ADUX_TEST_MODES_3, FORCE_CLOCK_RESET);
	return ret;
}

/**
This function is used to read the data of the fifo of the ADUX1020 chip
@param pst_adux  pointer to ADUX1020 chip structure
@return s32 fifo data of the ADUX1020 chip
*/
static s32
adux1020_fifo_data(struct adux1020 *pst_adux)
{
	s32 ret;
	u32 loopcnt = 0;

	if ( !calibration_flag )	{
		ADUX1020_info(" %s\n", __func__);
		ADUX1020_info("FIFO data of size %d to follow\n", pst_adux->fifosize);
	}
	/* Force Clock On */
	adux1020_set_force_clock(pst_adux);

	if (pst_adux->fifosize_byte) {
		ret = adux1020_multi_read(pst_adux,
					  ADUX_DATA_BUFFER,
					  (pst_adux->fifosize +
					   (1 & (pst_adux->fifosize_byte))),
					  pst_adux->pu16_databuff);
		if (ret < 0) {
			ADUX1020_err(" %s read error -%d\n", __func__, ret);
			return FAILED;
		}
		for (loopcnt = 0; loopcnt < pst_adux->fifosize; loopcnt++)
		{
			if ( !calibration_flag )	{
				ADUX1020_info("Data[%d] = 0x%x\n",loopcnt , pst_adux->pu16_databuff[loopcnt]);
			}
		}
		if ( !calibration_flag )
			ADUX1020_info("\n*******************\n");
	}

	/* Clock ctrl by Internal state machine */
	adux1020_reset_force_clock(pst_adux);

	return SUCCESS;
}

/*FIFO BUFFER READ - END ******************************************/
/*GESTURE ONCHIP ALGORITHM - START ********************************/
/**
This function is used for the Gesture Sign for the Gesture
utility with N-E-W-S data
@param pst_adux the ADUX device pointer.
@return Gesture index value
*/
static ENUM_GESTURE_INDEX
adux1020_get_gesture_sign(struct adux1020 *pst_adux)
{
	ENUM_GESTURE_INDEX gest_index;

	u8 DirX = pst_adux->adux_gest_data.dirX;
	u8 DirY = pst_adux->adux_gest_data.dirY;
	u8 DirZ = pst_adux->adux_gest_data.dirZ;

	/** need clarification */

	ADUX1020_info(" %s\n", __func__);
	ADUX1020_info("DirX = %d , DirY = %d , DirZ = %d\n", DirX, DirY, DirZ);
	if ((DirX == 1) && (DirY == 0) && (DirZ == 0)) {
		ADUX1020_info("GESTURE DIRECTION - East to West\n");
		gest_index = G_SIGN_EW;
	} else if ((DirX == 2) && (DirY == 0) && (DirZ == 0)) {
		ADUX1020_info("GESTURE DIRECTION - West to EAST\n");
		gest_index = G_SIGN_WE;
	} else if ((DirX == 0) && (DirY == 1) && (DirZ == 0)) {
		ADUX1020_info("GESTURE DIRECTION - South to North\n");
		gest_index = G_SIGN_SN;
	} else if ((DirX == 0) && (DirY == 2) && (DirZ == 0)) {
		ADUX1020_info("GESTURE DIRECTION - North to South\n");
		gest_index = G_SIGN_NS;
	} else if ((DirX == 0) && (DirY == 0) && (DirZ == 1)) {
		ADUX1020_info("GESTURE DIRECTION - HOVER\n");
		gest_index = G_SIGN_HOVER;
	} else {
		ADUX1020_info("GESTURE DIRECTION INVALID!!\n");
		gest_index = G_SIGN_INVALID;
	}

	ADUX1020_info("gest_index = %d\n", gest_index);
	return gest_index;
}

/*GESTURE ONCHIP ALGORITHM - END **********************************/
/*KEYCODE MAPPING - START *****************************************/
/**
This function is used in default allocation of the Keymap of the ADUX1020 chip
@param pst_adux the ADUX device pointer.
@return void
*/
static void
adux1020_default_keymap(struct adux1020 *pst_adux)
{
	s32 loop_i;
	ADUX1020_info(" %s\n", __func__);

	for (loop_i = 0; loop_i < NUM_OF_GESTURE_VALUE; loop_i++) {
		pst_adux->pt_adux1020_key_map->Inp_KeyMap_Table[loop_i] =
		    KEY_UNKNOWN;
		pst_adux->pt_adux1020_key_map->Inp_KeyRepeat_Table[loop_i] =
		    (unsigned short) GEST_KEY_REPEAT_RANGE;
	}

}

/**
This function is used to map the keycode to the gesture atoms.
@param pst_adux ADUX device pointer.
@return void
*/
static void
adux1020_keycodemapping(struct adux1020 *pst_adux)
{
	struct adux1020_key_map *pst_keymap = (pst_adux->pt_adux1020_key_map);
	/*unsigned short cnt = 0; */
	ADUX1020_info(" %s\n", __func__);

	adux1020_default_keymap(pst_adux);

	switch (pst_adux->KeyMapCode) {
	case 1:
		{
			pst_keymap->Inp_KeyMap_Table[1] = KEY_HOME;
			pst_keymap->Inp_KeyMap_Table[2] = KEY_MENU;
			pst_keymap->Inp_KeyMap_Table[3] = KEY_BACK;
			pst_keymap->Inp_KeyMap_Table[4] = KEY_SEARCH;
		}
		break;

	case 2:
		{
			pst_keymap->Inp_KeyMap_Table[1] = KEY_PLAYCD;
			pst_keymap->Inp_KeyMap_Table[2] = KEY_PAUSECD;
			pst_keymap->Inp_KeyMap_Table[3] = KEY_VOLUMEDOWN;
			pst_keymap->Inp_KeyMap_Table[4] = KEY_VOLUMEUP;
		}
		break;

	case 3:
		{
			pst_keymap->Inp_KeyMap_Table[1] = KEY_UP;
			pst_keymap->Inp_KeyMap_Table[2] = KEY_DOWN;
			pst_keymap->Inp_KeyMap_Table[3] = KEY_RIGHT;
			pst_keymap->Inp_KeyMap_Table[4] = KEY_LEFT;
		}
		break;
	case 4:
		{
			pst_keymap->Inp_KeyMap_Table[1] = KEY_POWER;
			pst_keymap->Inp_KeyMap_Table[2] = KEY_SLEEP;
			pst_keymap->Inp_KeyMap_Table[3] = KEY_WAKEUP;
			pst_keymap->Inp_KeyMap_Table[4] = KEY_COFFEE;
		}
		break;

	case 5:
		{
			pst_keymap->Inp_KeyMap_Table[1] = KEY_EQUAL;
			pst_keymap->Inp_KeyMap_Table[2] = KEY_MINUS;
			pst_keymap->Inp_KeyMap_Table[3] = KEY_RIGHT;
			pst_keymap->Inp_KeyMap_Table[4] = KEY_LEFT;
		}
		break;

	default:
		break;
	}
}

/*KEYCODE MAPPING - END *******************************************/
/*INPUT_DEVICE - START ********************************************/
/**
This function is used to send the Gesture proximity distance
	data throught event interface
@param prox_inputdev the ADUX input device pointer.
@param PROX_VALUE the proximity value of the Adux1020 chip.
@return void
*/
static void
adux1020_prox_dist_evt(struct input_dev *prox_inputdev, u32 PROX_VALUE)
{
	ADUX1020_info(" %s   proxval: %d\n", __func__, PROX_VALUE);
	input_event(prox_inputdev, EV_ABS, ABS_Z, PROX_VALUE);
	input_sync(prox_inputdev);
}

/**
This function is used to send Gesture proximity data throught event interface
@param prox_inputdev the ADUX input device pointer.
@return void
*/
static void
adux1020_proxi_evt(struct input_dev *prox_inputdev)
{
	struct adux1020 *pst_adux =
	    (struct adux1020 *)input_get_drvdata(prox_inputdev);
	u16 cnt_i;
	u16 cnt_j;

	ADUX1020_dbg(" %s\n", __func__);
	for (cnt_i = 0; cnt_i < pst_adux->fifosize; cnt_i += PROX_I_BUF_SIZE)
		for (cnt_j = 0; cnt_j < PROX_I_BUF_SIZE; cnt_j++) {
			input_event(prox_inputdev,
				    EV_MSC,
				    MSC_RAW,
				    (int) *(pst_adux->pu16_databuff +
					    cnt_j + cnt_i));
			input_sync(prox_inputdev);
		}
}

/**
This function is used for trigger the Gesture keyevent of the Adux1020 chip
@param pst_adux the ADUX device pointer.
@param gest_index the Gesture index value
@return void
*/
static void
adux1020_gest_keyevent(struct adux1020 *pst_adux, ENUM_GESTURE_INDEX gest_index)
{
	u32 key_value;
	u32 repeat;
	u16 cnt_i;
	ADUX1020_info(" %s\n", __func__);
	key_value = pst_adux->pt_adux1020_key_map->Inp_KeyMap_Table[gest_index];
	repeat = pst_adux->pt_adux1020_key_map->Inp_KeyRepeat_Table[gest_index];
	for (cnt_i = 0; cnt_i < repeat; cnt_i++) {
		input_event(pst_adux->gest_input, EV_KEY, key_value, 1);
		input_sync(pst_adux->gest_input);

		input_event(pst_adux->gest_input, EV_KEY, key_value, 0);
		input_sync(pst_adux->gest_input);
	}
}

/**
This function is used to send Gesture proximity data throught event interface
@param gest_inputdev the ADUX input device pointer.
@return void
*/
static void
adux1020_gest_inputevent(struct input_dev *gest_inputdev)
{
	struct adux1020 *pst_adux =
	    (struct adux1020 *)input_get_drvdata(gest_inputdev);
	u16 cnt_i;
	u16 cnt_j;

	ADUX1020_info(" %s\n", __func__);

	for (cnt_i = 0; cnt_i < pst_adux->fifosize; cnt_i += GEST_BUF_SIZE) {
		for (cnt_j = 0; cnt_j < GEST_BUF_SIZE; cnt_j++) {
			ADUX1020_info("Gesture data 0x%x\t",
				      (int) *(pst_adux->pu16_databuff + cnt_j +
					      cnt_i));

			input_event(gest_inputdev,
				    EV_MSC,
				    MSC_GESTURE,
				    (int) *(pst_adux->pu16_databuff +
					    cnt_j + cnt_i));
		}
		input_sync(gest_inputdev);
	}

}

/**
This function is used to send the Gesture keyevent data throught event interface
@param pst_adux the ADUX device pointer.
@return void
*/
static void
adux1020_gest_evt(struct adux1020 *pst_adux)
{
	u32 gest_index = 0;

	ADUX1020_info(" %s\n", __func__);
	if (pst_adux->GestOutMode == 1) {
		gest_index = adux1020_get_gesture_sign(pst_adux);
		adux1020_gest_keyevent(pst_adux, gest_index);
	} else {
		adux1020_gest_inputevent(pst_adux->gest_input);
	}

}

/**
This function is used to send the RAW data throught event interface
@param sample_inputdev ADUX input device pointer.
@return void
*/
static void
adux1020_sample_evt(struct input_dev *sample_inputdev)
{
	struct adux1020 *pst_adux =
	    (struct adux1020 *)input_get_drvdata(sample_inputdev);
	u16 cnt_i;
	u16 cnt_j;
	s32 event_value1 = 0;
	s32 event_value2 = 0;

	ADUX1020_info(" %s\n", __func__);
	switch (pst_adux->mode) {
	case SAMPLE_XYI:
		{
			for (cnt_i = 0;
			     cnt_i < (pst_adux->fifosize / SAMPLE_XYI_BUF_SIZE);
			     cnt_i++) {
				/* value in buffer in order of X | Y | I
				   event1 = X(b:16) | Y(b:16)
				   event2 = I(b:16)
				 */
				event_value1 = (*(pst_adux->pu16_databuff +\
				(cnt_i * SAMPLE_XYI_BUF_SIZE) + 0) << 16) |\
				*(pst_adux->pu16_databuff + (cnt_i *\
				SAMPLE_XYI_BUF_SIZE) + 1);

				event_value2 = *(pst_adux->pu16_databuff +
						 (cnt_i * SAMPLE_XYI_BUF_SIZE) +
						 2);

				input_event(sample_inputdev,
					    EV_MSC, MSC_RAW, event_value1);
				input_event(sample_inputdev,
					    EV_MSC, MSC_RAW, event_value2);
				input_sync(sample_inputdev);
				event_value1 = 0;
				event_value2 = 0;
			}
			if (pst_adux->fifosize % SAMPLE_XYI_BUF_SIZE) {
				ADUX1020_info("swp!! R_Da F %d, R_Da NF %d\n",\
				(pst_adux->fifosize % SAMPLE_XYI_BUF_SIZE),\
				SAMPLE_XYI_BUF_SIZE -\
				(pst_adux->fifosize % SAMPLE_XYI_BUF_SIZE));
			}
		}
		break;
	case SAMPLE_RAW:
		{
			ADUX1020_info("sample RAW evt\n");
			for (cnt_i = 0;
			     cnt_i < pst_adux->fifosize;
			     cnt_i += SAMPLE_RAW_BUF_SIZE) {
				for (cnt_j = 0;
				     cnt_j < SAMPLE_RAW_BUF_SIZE; cnt_j++) {
					if (cnt_j < 2) {
						event_value1 =
						    event_value1 << 16;
						event_value1 = event_value1 |
						    ((int)
						     *(pst_adux->pu16_databuff +
						       cnt_i + cnt_j));

					} else {
						event_value2 =
						    event_value2 << 16;
						event_value2 = event_value2 |
						    ((int)
						     *(pst_adux->pu16_databuff +
						       cnt_i + cnt_j));
					}
				}
				input_event(sample_inputdev,
					    EV_MSC, MSC_RAW, event_value1);
				input_event(sample_inputdev,
					    EV_MSC, MSC_RAW, event_value2);
				input_sync(sample_inputdev);
				event_value1 = 0;
				event_value2 = 0;
				ADUX1020_info("\n");
			}
		}
		break;
	default:
		ADUX1020_info("Invalid Sample mode\n");
		break;
	}
}

/**
This function is used for the input allocation of the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@param dev The device structure for the input device.
@param inputdev_name the name of the input device.
@return input_dev the input_dev device allocated
*/
static struct input_dev *
adux1020_inputdev_alloc(struct adux1020 *pst_adux,
			struct device *dev, s8 *inputdev_name)
{
	struct input_dev *pst_inputdev;
	s32 error;
	ADUX1020_info(" %s\n", __func__);
	/* Input device Registration for gesture gesture_raw and proximity */

	pst_inputdev = input_allocate_device();
	if (!pst_inputdev) {
		error = -ENOMEM;
		ADUX1020_err(" %s Error -%s\n", __func__, inputdev_name);
		return ERR_PTR(error);
	}

	pst_inputdev->id.bustype = BUS_I2C;
	pst_inputdev->id.product = pst_adux->product;
	pst_inputdev->id.version = pst_adux->version;
	pst_inputdev->name = inputdev_name;
	pst_inputdev->dev.parent = pst_adux->adux_dev;
	input_set_drvdata(pst_inputdev, pst_adux);

	return pst_inputdev;
}

/**
This function is used for the registering of the input capabilities and
registration of the input device of the ADUX1020 chip for gesture mode
@param pst_inputdev the input device pointer to open.
@return s32
*/
static s32
adux1020_gesture_inputdev_reg(struct input_dev *pst_inputdev)
{
	s32 error;
	s32 keyset = 0;

	ADUX1020_info(" %s\n", __func__);

	 /**************** register input device for gesture *****************/
	__set_bit(EV_MSC, pst_inputdev->evbit);
	__set_bit(EV_KEY, pst_inputdev->evbit);

	for (keyset = 0; keyset < 16; keyset++)
		__set_bit(keyset, pst_inputdev->keybit);	/* set Key bit*/
	for (keyset = 26; keyset < 31; keyset++)
		__set_bit(keyset, pst_inputdev->keybit);	/* set Key bit*/
	for (keyset = 51; keyset < 256; keyset++)
		__set_bit(keyset, pst_inputdev->keybit);	/* set Key bit*/
	__set_bit(MSC_GESTURE, pst_inputdev->mscbit);	/* set Raw bit */
	__set_bit(MSC_RAW, pst_inputdev->mscbit);	/* set Raw bit */

	error = input_register_device(pst_inputdev);
	if (error < 0) {
		ADUX1020_dbg(" %s registered failed", pst_inputdev->name);
		return error;
	}
	return 0;
}

/**
This function is used to deregister the input device of the ADUX1020 chip
@param pst_inputdev the input device pointer to unregister
@return void
*/
static void
adux1020_gesture_inputdev_unreg(struct input_dev *pst_inputdev)
{
	ADUX1020_info(" %s\n", __func__);
	input_unregister_device(pst_inputdev);
}

/**
This function is used to Register the input device
to proximity mode of the ADUX1020 chip
@param pst_inputdev the input device pointer to close.
@return s32
*/
static s32
adux1020_proximity_inputdev_reg(struct input_dev *pst_inputdev)
{
	s32 error;

	ADUX1020_info(" %s\n", __func__);
	/*************** register input device for proximity ****************/
	__set_bit(EV_ABS, pst_inputdev->evbit);
	__set_bit(EV_MSC, pst_inputdev->evbit);

	__set_bit(ABS_Z, pst_inputdev->absbit);
	__set_bit(MSC_RAW, pst_inputdev->mscbit);	/* set RAW bit */
	input_set_abs_params(pst_inputdev, ABS_Z, 0, 1, 0, 0);

	/* ASUS input */
	input_set_capability(pst_inputdev, EV_ABS, ABS_DISTANCE);
	__set_bit(EV_ABS, pst_inputdev->evbit);
	__set_bit(ABS_DISTANCE, pst_inputdev->absbit);
	input_set_abs_params(pst_inputdev, ABS_DISTANCE, 0, 1, 0, 0);

	error = input_register_device(pst_inputdev);
	if (error < 0) {
		ADUX1020_dbg(" %s registered failed", pst_inputdev->name);
		return error;
	}

	return 0;
}

/**
This function is used to unregister the input device
from proximity mode of the ADUX1020 chip
@param pst_inputdev the input device pointer to close.
@return void
*/
static void
adux1020_proximity_inputdev_unreg(struct input_dev *pst_inputdev)
{
	ADUX1020_info(" %s\n", __func__);
	input_unregister_device(pst_inputdev);
}

/*INPUT_DEVICE -END ***********************************************/

/*ADUX DRIVER - API **********************************************/
/**
This function is used to flush buffer the Adux1020 chip.
@param pst_adux ADUX device pointer.
@return void
*/
static void
adux1020_flush_buffer(struct adux1020 *pst_adux)
{
	ADUX1020_info(" %s\n", __func__);
	memset(pst_adux->pu16_databuff, 0, MAX_BUFFER);
}

/**
This function is used for  reset the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return void
*/
static void
adux1020_reset_device(struct adux1020 *pst_adux)
{
	u16 resetdevice;
	u16 retrycnt = MAX_RETRY;

	ADUX1020_info(" %s\n", __func__);
	/*/set ADUX SW RESET bit [0] */
	adux1020_write(pst_adux, ADUX_SW_RESET, (u16) SW_RESET_SET);
	do {
		/*wait for transition to get completed */
		/*      udelay_range(10000, 10000);//maha */
		msleep(20);
		resetdevice = adux1020_read(pst_adux, ADUX_SW_RESET);
		retrycnt--;
	} while (retrycnt && (resetdevice != 0x0));

}

/**
This function is used to set the I2c speed of the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return void
*/
static void
adux1020_set_i2c_speed(struct adux1020 *pst_adux)
{
	u16 i2cspeed;
	u16 val;

	ADUX1020_info(" %s\n", __func__);
	val = adux1020_read(pst_adux, ADUX_I2C_1);

	i2cspeed = (val & I2C_SPEED_MASK) >> 12;

	ADUX1020_info("I2c Speed 0x%x\n", i2cspeed);
	i2cspeed = val | I2C_SPEED_MASK;
	ADUX1020_info("%s write value 0x%x\n", __func__, i2cspeed);
	adux1020_write(pst_adux, ADUX_I2C_1, i2cspeed);

	val = adux1020_read(pst_adux, ADUX_I2C_1);
	i2cspeed = (val & I2C_SPEED_MASK) >> 12;
	ADUX1020_dbg("I2c Speed 0x%x\n", i2cspeed);
}

/**
This function is used for  read the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return void
*/
static void
adux1020_read_chip(struct adux1020 *pst_adux)
{
	u16 checkdevice = 0;
	ADUX1020_info(" %s\n", __func__);
	checkdevice = adux1020_read(pst_adux, ADUX_CHIP_ID_VERSION_ADDR);
	pst_adux->chip_id = checkdevice & CHIP_ID_MASK;
	pst_adux->version = GET_VERSION(checkdevice);
}

/**
This function is used to update the threshold for fifo of the ADUX1020 chip
@param pst_adux  pointer to ADUX1020 chip structure
@return u16 the updated threshold value
*/
static u16
adux1020_update_fifo_th(struct adux1020 *pst_adux)
{
	u16 fifo_th;

	ADUX1020_info(" %s\n", __func__);
	fifo_th = adux1020_read(pst_adux, ADUX_I2C_2);
	fifo_th = (fifo_th & 0x0F00) >> 8;
	return fifo_th;
}

/**
This function is used for clearing the interrupt status of the ADUX1020 chip
@param pst_adux pointer to  Adux1020 chip structure
@return s32
*/
static s32
adux1020_clr_intr_status(struct adux1020 *pst_adux)
{
	s32 ret;
	u16 clear;
	clear = pst_adux->curr_intr;
	ret = adux1020_write(pst_adux, ADUX_INT_STATUS_ADDR, clear);
	return ret;
}

/**
This function is used to read the interrupt status of the fifo
@param pst_adux  pointer to ADUX1020 chip structure
@return s32 interrupt status of the ADUX1020 chip
*/
static s32
adux1020_rd_intr_fifo(struct adux1020 *pst_adux)
{
	u16 status;
	ADUX1020_info("%s\n", __func__);

	status = adux1020_read(pst_adux, ADUX_INT_STATUS_ADDR);
	ADUX1020_info("Status - 0x%x\n", status);
	switch (pst_adux->mode) {
	case PROXIMITY_NONE:
	case PROXIMITY_I:
		pst_adux->curr_intr = status & INTR_STATUS_MASK(PROXIMITY);
		break;
	case GESTURE:
		pst_adux->curr_intr = status & INTR_STATUS_MASK(GESTURE);
		break;
	case SAMPLE_XYI:
	case SAMPLE_RAW:
		pst_adux->curr_intr = status & INTR_STATUS_MASK(SAMPLE);
		break;
	case IDLE:
	case OFF:
		pst_adux->curr_intr = status & INTR_STATUS_MASK(GENERAL);
		break;
	case FIFO:
		pst_adux->curr_intr = status & INTR_STATUS_MASK(FIFO);
		break;
	default:
		pst_adux->curr_intr = status & INTR_STATUS_MASK(WATCHDOG);
		break;
	};

	pst_adux->fifosize_byte = ((status & FIFO_SIZE) >> 8);
	pst_adux->fifosize =
	    (((status & FIFO_SIZE) >> 8) / ADUX1020_REG_SIZE_BYTES);

	if (pst_adux->fifosize >= 32)
		ADUX1020_info("Buffer overflow!!\n");


	ADUX1020_info("IS - 0x%x\tFS - 0x%x\n", pst_adux->curr_intr,
		      pst_adux->fifosize);
	return SUCCESS;
}

/**
This function is used for clearing the interrupt fifo of the ADUX1020 chip
@param pst_adux pointer to  Adux1020 chip structure
@return s32
*/
static s32
adux1020_clr_intr_fifo(struct adux1020 *pst_adux)
{
	s32 ret;
	u16 clear;
	u16 loopcnt;

	ADUX1020_info(" %s\n", __func__);
	do {
		adux1020_rd_intr_fifo(pst_adux);

		clear = (pst_adux->curr_intr) | SELF_RESETTING;

		ADUX1020_dbg("FIFO size during SWITCH %x\n",
			     pst_adux->fifosize);
		ADUX1020_dbg("FS byte %x\n", pst_adux->fifosize_byte);
		if (pst_adux->fifosize_byte) {
			adux1020_write(pst_adux,
				       ADUX_TEST_MODES_3, FORCE_CLOCK_ON);
			adux1020_multi_read(pst_adux,
					    ADUX_DATA_BUFFER,
					    (pst_adux->fifosize +
					     (1 & (pst_adux->fifosize_byte))),
					    pst_adux->pu16_databuff);
			for (loopcnt = 0; loopcnt < pst_adux->fifosize;
			     loopcnt++)
				ADUX1020_info("0x%x\n",
					      pst_adux->pu16_databuff[loopcnt]);

			adux1020_write(pst_adux,
				       ADUX_TEST_MODES_3, FORCE_CLOCK_RESET);
			ret = adux1020_write(pst_adux,
					     ADUX_INT_STATUS_ADDR, clear);
		}
	} while (pst_adux->fifosize != 0);
	adux1020_flush_buffer(pst_adux);
	return ret;
}

/**
This function is used to enter idle mode of the Adux1020
@param pst_adux  pointer to ADUX1020 chip structure
@return s32
*/
static s32
adux1020_enter_idle(struct adux1020 *pst_adux)
{
	s32 ret;
	u16 mode_val = 0;
	u16 mask_val = 0;
	ADUX1020_dbg("%s\n", __func__);

	mode_val = SET_MODE_VALUE(IDLE, NONE);
	mask_val = SET_INTR_MASK_VALUE(IDLE);
	ret = adux1020_write(pst_adux, ADUX_OP_MODE_ADDR, mode_val);

	if (ret < 0) {
		ADUX1020_err("%s write error -%d\n", __func__, ret);
		return FAILED;
	}
	ADUX1020_dbg("Address 0x%02x\tValue 0x%04x\n", ADUX_OP_MODE_ADDR, mode_val);

	pst_adux->mode = IDLE;

	ret = adux1020_write(pst_adux, ADUX_INT_MASK_ADDR, mask_val);

	if (ret < 0) {
		ADUX1020_err("%s write error -%d\n", __func__, ret);
		return FAILED;
	}
	ADUX1020_dbg("Address 0x%02x\tValue 0x%04x\n", ADUX_INT_MASK_ADDR, mask_val);

	/*  Clear IRQ Status */
	adux1020_clr_intr_fifo(pst_adux);

	return SUCCESS;
}

/**
 *@brief This API is used to multi write
 *@param pst_adux the device structure
 *@param mode the operational mode.
 *@return u16 short int value of the status.
*/
static u16
adux1020_mode_switching(struct adux1020 *pst_adux, u16 mode)
{
	s32 ret = 1;
	u16 mode_val = 0;
	u16 intr_mask_val = 0;

	disable_irq(pst_adux->irq);
	mutex_lock(&pst_adux->mutex);
	/*enter to idle mode to avoid unnecessary intr */
	adux1020_enter_idle(pst_adux);

	ADUX1020_dbg("adux1020_mode_switching\n");

	switch (mode) {
	case PROXIMITY_NONE:
		mode_val = SET_MODE_VALUE(PROXIMITY, NONE);
		intr_mask_val = SET_INTR_MASK_VALUE(PROXIMITY);
		pst_adux->prox_state.proximity1 = (u8) UNKNOWN;
		pst_adux->prox_state.proximity2 = (u8) UNKNOWN;
		break;
	case PROXIMITY_I:
		mode_val = SET_MODE_VALUE(PROXIMITY, I);
		intr_mask_val = SET_INTR_MASK_VALUE(PROXIMITY);
		pst_adux->prox_state.proximity1 = (u8) UNKNOWN;
		pst_adux->prox_state.proximity2 = (u8) UNKNOWN;
		ADUX1020_dbg("PROXIMITY_I\n");
		break;
	case GESTURE:
		mode_val = SET_MODE_VALUE(GESTURE, NONE);
		intr_mask_val = SET_INTR_MASK_VALUE(GESTURE);
		break;
#if 1
	case SAMPLE_XYI:
		mode_val = SET_MODE_VALUE(SAMPLE, XYI);
		intr_mask_val = SET_INTR_MASK_VALUE(SAMPLE);
		break;
#endif
	case SAMPLE_RAW:
		mode_val = SET_MODE_VALUE(SAMPLE, RAW);
		intr_mask_val = SET_INTR_MASK_VALUE(SAMPLE);
		break;
	case IDLE:
		mode_val = SET_MODE_VALUE(IDLE, NONE);
		intr_mask_val = SET_INTR_MASK_VALUE(IDLE);
		pst_adux->r_fifo_prevent = 0;
		pst_adux->r_pack_start = 0;
		pst_adux->sample_out = 0;
		break;
	default:
		ADUX1020_dbg("Invalid mode!!! - entered into idle\n");
		ret = 0;
		mode = IDLE;
		break;
	};
	ADUX1020_dbg("mode_val - 0x%04x\tIntr_mask_val - 0x%04x\n",
		      mode_val, intr_mask_val);
	if (pst_adux->r_fifo_prevent == 1)
		mode_val = mode_val | (1 << 8);
	if (pst_adux->r_pack_start == 1)
		mode_val = mode_val | (1 << 9);
	if (pst_adux->sample_out == 1)
		mode_val = mode_val | (1 << 12);

	ADUX1020_dbg("mode_val - 0x%04x\tIntr_mask_val - 0x%04x\n",
		      mode_val, intr_mask_val);
	if (ret) {
		adux1020_write(pst_adux, ADUX_INT_MASK_ADDR, intr_mask_val);
		adux1020_write(pst_adux, ADUX_OP_MODE_ADDR, mode_val);
	}
	pst_adux->mode = mode;
	mutex_unlock(&pst_adux->mutex);
	enable_irq(pst_adux->irq);
	return SUCCESS;
}

/**
@brief This API used to initialize the adux1020 structure
@param dev pointer to Device Structure of the adux1020 chip
@param devid pointer to i2c_device_id of the adux1020 chip
@param irq GPIO irq number
@param pst_adux pointer to Adux1020 Structure
@param pt_i2c_bus_ops pointer to adux_bus_ops Structure of the adux1020 chip
@return void
*/
static void
adux1020_struct_init(struct device *dev,
		     const struct i2c_device_id *devid,
		     unsigned irq,
		     struct adux1020 *pst_adux, const struct adux_bus_ops
		     *pt_i2c_bus_ops)
{
	ADUX1020_dbg(" %s\n", __func__);
	pst_adux->adux_dev = dev;
	pst_adux->product = 0x00;
	pst_adux->version = 0x00;
	pst_adux->adux_class = NULL;
	pst_adux->adux_dev_attr = NULL;
	pst_adux->irq = irq;
	pst_adux->read = pt_i2c_bus_ops->read;
	pst_adux->write = pt_i2c_bus_ops->write;
	/*  Assigning Default Values chk */
	pst_adux->mode = OFF;
	pst_adux->fifosize = 0;
	pst_adux->fifo_th = 0;
	pst_adux->pu16_databuff = databuffer;
	pst_adux->chip_id = CHIP_ID_MASK;
	/* it is assumed that the default value for proximity is OFF */
	pst_adux->prox_state.proximity1 = (u8) UNKNOWN;
	pst_adux->prox_state.proximity2 = (u8) UNKNOWN;
	/*dynamic memory allocation */
	pst_adux->runtime_pdata = kzalloc(sizeof(struct adux_platform_data),
					  GFP_KERNEL);
	if (pst_adux->pdata == NULL) {
		pst_adux->pdata = &adux1020_default_platform_data;
		ADUX1020_info("Default Configuration\n");
	} else {
		ADUX1020_info("Board File Configuration\n");
	}

	pst_adux->endian = ADUX1020_BIG_ENDIAN;
	pst_adux->GestOutMode = 1;	/*Key Event Mode */

	pst_adux->pt_adux1020_key_map = &st_adux1020keymap;

	pst_adux->r_fifo_prevent = 0;
	pst_adux->r_pack_start = 0;
	pst_adux->sample_out = 0;
	pst_adux->prox_type = 0;
}

/**
This function is used for  getting the power on initialization
status of  ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return  void
*/
static void
adux1020_power_init(struct adux1020 *pst_adux)
{
	ADUX1020_dbg(" %s\n", __func__);
	/* Read chip ID  and board revision details */
	adux1020_read_chip(pst_adux);
	ADUX1020_dbg(" CHIP ID 0x%x\tVERSION 0x%x\n", pst_adux->chip_id,
		     pst_adux->version);
	if (pst_adux->chip_id != CHIP_ID_MASK) {
		adux1020_reset_device(pst_adux);
		/*  Set I2C high speed mode (400K)  */
		adux1020_set_i2c_speed(pst_adux);
	}
}

/******************************************************************/
/*CLOCK RELATED - START *******************************************/
/**
This function is used for  enabling 32K oscillator
status of  ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return  void
*/
static void enable32kOsc(struct adux1020 *pst_adux)
{
	ADUX1020_dbg("%s\n", __func__);
	adux1020_write(pst_adux, 0x0032, 0x0B0B);
}

/**
This function is used for  disabling 32K oscillator
status of  ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return  void
*/
static void disable32kOsc(struct adux1020 *pst_adux)
{
	ADUX1020_dbg("%s\n", __func__);
	adux1020_write(pst_adux, 0x0032, 0x40);
} 

/**
This function is used to measure oscillator
status of  ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return unsigned short
*/
static unsigned short measureOsc(struct adux1020 *pst_adux)
{
	unsigned short readreg;
    unsigned short clockCount;
	
	ADUX1020_dbg("%s\n", __func__);
	
	readreg = adux1020_read(pst_adux, 0x0030);
    readreg |= 0x20;
    adux1020_write(pst_adux, 0x0030, readreg);

    //After retime, a pulse of 2-cycle @ 32KHz is generated.
    //!!! do we need to wait here???

    //Fast clock counts the clock numbers, and store data in 12-bit register 0x0A.
    clockCount = adux1020_read(pst_adux, 0x0A);
	ADUX1020_dbg("OSC_CAL_OUT - %d\n", clockCount);
	
	//Clear reg 0x30 bit[5] ¡°r_osc32m_cal_en¡± to 0.
    readreg = adux1020_read(pst_adux, 0x0030);
    readreg &= 0xFFDF;
    adux1020_write(pst_adux, 0x0030, readreg);
    return clockCount;
}

/**
This function is used for decrease Trim value for oscillator
status of  ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return  void
*/
static void decreaseTrimOsc(struct adux1020 *pst_adux, int step)
{
	unsigned short readreg = 0;
	unsigned short readregtmp = 0;
	
	readreg = adux1020_read(pst_adux, 0x1A);
	readregtmp = (unsigned short)(readreg & 0xFF);
	if (readregtmp > (step - 1))
	{
		readregtmp = (unsigned short)(readregtmp - step);
		readreg &= 0xFF00;
		readreg |= readregtmp;
		adux1020_write(pst_adux, 0x1A, readreg);
	}

}

/**
This function is used for increase Trim value for oscillator
status of  ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return  void
*/
static void increaseTrimOsc(struct adux1020 *pst_adux, int step)
{
	unsigned short readreg = 0;
	unsigned short readregtmp = 0;
	readreg = adux1020_read(pst_adux, 0x1A);
    readregtmp = (unsigned short)(readreg & 0xFF);
	if (readregtmp < (255 - step))
	{
		readregtmp = (unsigned short)(readregtmp + step);
		readreg &= 0xFF00;
		readreg |= readregtmp;
		adux1020_write(pst_adux, 0x1A, readreg);
	}

}

/**
This function is used for changing 32M frequency
status of  ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return  void
*/
static void change32MFreq(struct adux1020 *pst_adux, int dv)
{
	if (dv > 0) {
		//Too high frequency
		increaseTrimOsc(pst_adux, 1);
	} else {
		//Too low frequency
		decreaseTrimOsc(pst_adux, 1);
	}
}
/**
This function is used for 32K clock calibration of the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return  int
*/
static int adux1020_check32KClock(struct adux1020 *pst_adux)
{
	int clockFreq = 0;
	//int sampleTimeoutMS = 100;
    int timeTargetSec = 5;
    int timeTargetMS = timeTargetSec * 1000;
    int numSamples = 0;
	int numSamplesSec = 0;
	unsigned int temp_cnt = 0;
	
	unsigned short read_reg = 0;
	unsigned short intr_status = 0;
	
	unsigned long time_calib;
	
	/*switch to sample mode*/
	adux1020_write(pst_adux, 0x48, 0x70);
	adux1020_write(pst_adux, 0x45, 0x108);
			
		
		time_calib = jiffies;
	while (jiffies_to_msecs(jiffies - time_calib) < timeTargetMS) {
		if (pst_adux->clk_status == ABORT) {
			ADUX1020_dbg("ABORT - %s\n", __func__);
			goto check32KClock_abort;
		}
			read_reg = adux1020_read(pst_adux, 0x49);
		intr_status = (read_reg & 0xFF);
		//if (intr_status) {
			pst_adux->fifosize_byte = ((read_reg &\
							FIFO_SIZE) >> 8);
			pst_adux->fifosize = (((read_reg & FIFO_SIZE) >> 8) /\
						ADUX1020_REG_SIZE_BYTES);
			temp_cnt += pst_adux->fifosize;
		//	ADUX1020_info("count = %d\n", temp_cnt);
				adux1020_fifo_data(pst_adux); /*read FIFO*/
				adux1020_clr_intr_status(pst_adux);
		//}

		}
	adux1020_enter_idle(pst_adux);
	ADUX1020_dbg("temp_cnt = %d\n", temp_cnt);

	numSamples = temp_cnt / 4;
	ADUX1020_dbg("numSamples = %d\n", numSamples);

	//Calculate 32k Clock trim value
    numSamplesSec = numSamples / timeTargetSec;
	ADUX1020_dbg("numSamplesSec = %d\n", numSamplesSec);
    clockFreq = (32000 + (32000 * (numSamplesSec - 50) / 50));
    ADUX1020_dbg("Current Clock = %d Hz\n", clockFreq);
    return clockFreq;
check32KClock_abort:
	return -1;
}

/**
This function is used for 32M clock calibration of the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return int
*/
static int
adux1020_32M_calibration(struct adux1020 *pst_adux)
{
	int absdv;
    int dv;
    int prevabsdv;
    int prevdv;
    int targetValue;
	int trycnt = 18;
    unsigned short currentValue;
    unsigned short prevValue;

	enable32kOsc(pst_adux);
	targetValue = 2000;
	currentValue = measureOsc(pst_adux);
    prevValue = currentValue;

	absdv = MATH_ABS(targetValue - currentValue);
	dv = (targetValue - currentValue);
	prevabsdv = absdv;
	prevdv = dv;

	while (absdv != 0)
	{
		if (pst_adux->clk_status == ABORT) {
			ADUX1020_dbg("ABORT - %s\n", __func__);
			goto adux_32M_calibration_abort;
		}
		//Configure OS32M trim register to get register 0x0A close to 2000
		if (dv == 0) {
			//We are done.
			break;
		} else {
			change32MFreq(pst_adux, dv);
		}
		trycnt = 18;
		while (trycnt-- > 0) {
			if (pst_adux->clk_status == ABORT) {
				ADUX1020_dbg("ABORT - %s\n", __func__);
				goto adux_32M_calibration_abort;
			}
			msleep(50);
			currentValue = measureOsc(pst_adux);
			
			if ((prevValue == currentValue))
            {
				if (trycnt == 6)
				{
					change32MFreq(pst_adux, dv);
				}
				if (trycnt == 0)
				{
					//If no change - exit with error.
					disable32kOsc(pst_adux);
					return false;
				}
            } else {
				//OK. Exit the loop.
				trycnt = 0;
            }
		}
		absdv = MATH_ABS(targetValue - currentValue);
		dv = (targetValue - currentValue);
		if ((prevdv * dv) < 0)
		{
			//the target crossed
			if (absdv < prevabsdv)
			{
				//the best value is current
				//absdv = 0; below
		}
		else
			{
				//the best value is previous
				if (absdv != 0)
				{
				//Configure OS32M trim register to get register 0x0A close to 2000
					if (dv < 0)
					{
						//Too high frequency
						decreaseTrimOsc(pst_adux, 1);
					}
		else
					{
						//Too low frequency
						increaseTrimOsc(pst_adux, 1);
					}
				}
			}
			//we are done. Exit the loop.
			absdv = 0;
		}
		prevabsdv = absdv;
		prevdv = dv;
		if (prevValue == currentValue)
		{
			//If no change - exit.
			absdv = 0;
			disable32kOsc(pst_adux);
			return 0;
		}
		prevValue = currentValue;
		


	}

	disable32kOsc(pst_adux);
	return 1;
adux_32M_calibration_abort:
	disable32kOsc(pst_adux);
	return -1;

}

/**
This function is used for  clock calibration of the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return  void
*/
static void
adux1020_clock_calibration(struct adux1020 *pst_adux)
{
	/*variable required for 32K osc*/
	int clockFreq = 0;
	u16 prev_didt = 0;
	u16 prev_freq = 0;
	u16 prev_deci = 0;
	u16 read_reg = 0;
		
	int retry = 2;
	int diff = 0;
	int corr = 0;
	int trim = 0;
	int ret = 0;
	

	/*configure 32K oscillator*/
	/*step 1: get previous value*/
	prev_didt = adux1020_read(pst_adux, 0x28);
	prev_freq = adux1020_read(pst_adux, 0x40);
	prev_deci = adux1020_read(pst_adux, 0x46);
	/*step 2: set value for didt, freq, decimation*/
	adux1020_write(pst_adux, 0x28, 0);/*didt = 0*/
	adux1020_write(pst_adux, 0x40, 0x8068);/*freq = 0x8068*/
	adux1020_write(pst_adux, 0x46, 0);

	/*step 3: disable the interrupt*/
	disable_irq(pst_adux->irq);
	/*step 4: calculate 32k trim value*/
	clockFreq = adux1020_check32KClock(pst_adux);
	if (clockFreq < 0) {
		ADUX1020_dbg("ABORT - %s\n", __func__);
		goto adux_clock_calibration_abort;
}

	while ((retry-- > 0) && ((clockFreq > (32000 + 960)) || (clockFreq < (32000 - 960))))
    {
		//Calibrate 32M
        diff = clockFreq - 32000;
        ADUX1020_dbg("Diff Clock = %d\n", diff);
		
        corr = (int)(diff / 545);
        ADUX1020_dbg("Correction = %d\n", corr);
		/*read 32K trim value*/
		read_reg = adux1020_read(pst_adux, ADUX_OSCS_1);
		trim = (read_reg & 0x1F);
		
		ADUX1020_dbg("Old trim value = %d\n", trim);
		
		trim += corr;
        ADUX1020_dbg("New trim value = %d\n", trim);
		
		read_reg = (read_reg & 0xFFE0) | (trim & 0x1F);
		/*set trim value back to 32K*/
		adux1020_write(pst_adux, ADUX_OSCS_1, read_reg);
        
		clockFreq = adux1020_check32KClock(pst_adux);
		if (clockFreq < 0) {
			ADUX1020_dbg("ABORT - %s\n", __func__);
			goto adux_clock_calibration_abort;
		}
		
        ADUX1020_dbg("Current Clock = %d\n", clockFreq);
    }
    
	if ((clockFreq < (32000 + 960)) && (clockFreq > (32000 - 960))) {
                ret = adux1020_32M_calibration(pst_adux);
				if (ret <= -1) {
					ADUX1020_dbg("ABORT from 32M calib\n");
					goto adux_clock_calibration_abort;
				}
    }
	

	//ADUX1020_dbg("New 32K trim value = 0x%04x\n", read_reg);
adux_clock_calibration_abort:
	adux1020_write(pst_adux, 0x28, prev_didt);
	adux1020_write(pst_adux, 0x40, prev_freq);
	adux1020_write(pst_adux, 0x46, prev_deci);

	adux1020_enter_idle(pst_adux);
	enable_irq(pst_adux->irq);

}
/*CLOCK RELATED - END *********************************************/
/*CONFIG RELATED - START ******************************************/
/**
This function is used for general  configuration of the ADUX1020 Chipset
@param pst_adux  pointer to ADUX1020 chip structure
@return s32 SUCCESS /FAILED bassed on the outcome.
*/
static s32
adux1020_general_config_init(struct adux1020 *pst_adux)
{
	s32 ret = 0;
	s32 i = 0;
	u16 addr;
	u16 value = 0;

	struct adux_platform_data *config_reg;
	if (pst_adux->runtime_pdata->general_reg_cnt != 0)
		config_reg = pst_adux->runtime_pdata;
	else
		config_reg = pst_adux->pdata;
	/** configuration for general registers */
	ADUX1020_dbg(" %s\n", __func__);
	for (i = 0; i < config_reg->general_reg_cnt; i++) {
		addr = (u16) ((config_reg->general_regs[i] & 0xffff0000) >> 16);
		value = (u16) (config_reg->general_regs[i] & 0x0000ffff);
		if (!(addr == 0x45 || addr == 0x48 || addr == 0x49)) {
			ADUX1020_info("Writing 0x%02x with Value 0x%04x\n",
				      addr, value);
			adux1020_write(pst_adux, addr, value);
			if (ret < 0) {
				ADUX1020_info(" %s i2c wr err %d\n",
					      __func__, ret);
				return FAILED;
			}
		} else {
			ADUX1020_info("!!!!Writing 0x%02x with Value 0x%04x\n",\
					addr, value);
		}

	}
	for (i = 0; i < config_reg->general_reg_cnt; i++) {
		addr = (u16) ((config_reg->general_regs[i] & 0xffff0000) >> 16);
		value = adux1020_read(pst_adux, addr);
		ADUX1020_dbg("Address 0x%02x\tValue 0x%04x\n", addr, value);
	}
	return SUCCESS;
}

/**
This function is used for initialize the proximity configuration
	of the ADUX1020 Chip
@param pst_adux  pointer to ADUX1020 chip structure
@return s32 SUCCESS /FAILED bassed on the outcome.
*/
static s32
adux1020_proximity_config_init(struct adux1020 *pst_adux)
{
	s32 ret = 0;
	s32 i = 0;
	u16 addr;
	u16 value = 0;

	struct adux_platform_data *config_reg;
	if (pst_adux->runtime_pdata->proximity_reg_cnt != 0)
		config_reg = pst_adux->runtime_pdata;
	else
		config_reg = pst_adux->pdata;

	/** configuration for proximity  registers */
	ADUX1020_info(" %s\n", __func__);
	for (i = 0; i < config_reg->proximity_reg_cnt; i++) {
		addr = (u16) ((config_reg->proximity_regs[i] & 0xffff0000) >>
			      16);
		value = (u16) (config_reg->proximity_regs[i] & 0x0000ffff);
		if (!(addr == 0x45 || addr == 0x48 /*|| addr == 0x49*/)) {
			ADUX1020_info("Writing 0x%02x with Value 0x%04x\n",
				      addr, value);
			ret = adux1020_write(pst_adux, addr, value);
			if (ret < 0) {
				ADUX1020_info(" %s i2c wr err %d\n",
					      __func__, ret);
				return FAILED;
			}
		} else {
			ADUX1020_info("!!!!Writing 0x%02x with Value 0x%04x\n",
				      addr, value);
		}
	}
	/*addr = value = 0; */
	for (i = 0; i < config_reg->proximity_reg_cnt; i++) {
		addr = (u16) ((config_reg->proximity_regs[i] & 0xffff0000) >>
			      16);

		value = adux1020_read(pst_adux, addr);

		ADUX1020_dbg("Address 0x%02x\tValue 0x%04x\n", addr, value);
	}
	return SUCCESS;
}

/**
This function is used for general  gesture configuration / initialization
	of the ADUX1020 Chip
@param pst_adux  pointer to ADUX1020 chip structure
@return s32 SUCCESS /FAILED bassed on the outcome.
*/
static s32
adux1020_gesture_config_init(struct adux1020 *pst_adux)
{
	s32 ret = 0;
	s32 i = 0;
	u16 addr;
	u16 value = 0;

	struct adux_platform_data *config_reg;
	if (pst_adux->runtime_pdata->gesture_reg_cnt != 0)
		config_reg = pst_adux->runtime_pdata;
	else
		config_reg = pst_adux->pdata;
	/** configuration for Gesture Atom  registers */
	ADUX1020_dbg(" %s\n", __func__);
	for (i = 0; i < config_reg->gesture_reg_cnt; i++) {
		addr = (u16) ((config_reg->gesture_regs[i] & 0xffff0000) >> 16);
		value = (u16) (config_reg->gesture_regs[i] & 0x0000ffff);
		if (!(addr == 0x45 || addr == 0x48 || addr == 0x49)) {
			ADUX1020_dbg("Writing 0x%02x with Value 0x%04x\n",
				      addr, value);
			ret = adux1020_write(pst_adux, addr, value);
			if (ret < 0) {
				ADUX1020_dbg(" %s i2c wr err %d\n",
					      __func__, ret);
				/* return -1; */
				return FAILED;
			}
		} else {
			ADUX1020_dbg("!!!!Writing 0x%02x with Value 0x%04x\n",
				      addr, value);
		}
	}
	/*addr = value = 0; */

	for (i = 0; i < config_reg->gesture_reg_cnt; i++) {
		addr = (u16) ((config_reg->gesture_regs[i] & 0xffff0000) >> 16);
		value = adux1020_read(pst_adux, addr);
		ADUX1020_dbg("Address 0x%02x\tValue 0x%04x\n", addr, value);
	}
	return SUCCESS;
}

/**
This function is used sample configuration initialzation of the ADUX1020 Chip
@param pst_adux  pointer to ADUX1020 chip structure
@return s32 SUCCESS /FAILED bassed on the outcome.
*/
static s32
adux1020_sample_xyi_config_init(struct adux1020 *pst_adux)
{
	s32 ret = 0;
	s32 i = 0;
	u16 addr;
	u16 value = 0;

	struct adux_platform_data *config_reg;
	if (pst_adux->runtime_pdata->sample_xyi_reg_cnt != 0)
		config_reg = pst_adux->runtime_pdata;
	else
		config_reg = pst_adux->pdata;
	/** configuration forproximity  registers */
	ADUX1020_dbg(" %s\n", __func__);
	for (i = 0; i < config_reg->sample_xyi_reg_cnt; i++) {
		addr = (u16) ((config_reg->sample_xyi_regs[i] & 0xffff0000) >>
			      16);
		value = (u16) (config_reg->sample_xyi_regs[i] & 0x0000ffff);
		if (!(addr == 0x45 || addr == 0x48 || addr == 0x49)) {
			ADUX1020_dbg("Writing 0x%02x\twith Value 0x%04x\n",
				      addr, value);

			ret = adux1020_write(pst_adux, addr, value);
			if (ret < 0) {
				ADUX1020_dbg(" %s i2c wr err %d\n",
					      __func__, ret);
				/* return -1; */
				return FAILED;
			}
		} else {
			ADUX1020_dbg("!!!!Writing 0x%02x with Value 0x%04x\n",
				      addr, value);
		}
	}
	/*addr = value = 0; */

	for (i = 0; i < config_reg->sample_xyi_reg_cnt; i++) {
		addr = (u16) ((config_reg->sample_xyi_regs[i] & 0xffff0000) >>
			      16);
		value = adux1020_read(pst_adux, addr);
		ADUX1020_dbg("Address 0x%02x   Value 0x%04x\n", addr, value);
	}
	return SUCCESS;
}

/**
This function is used sample configuration initialzation of the ADUX1020 Chip
@param pst_adux  pointer to ADUX1020 chip structure
@return s32 SUCCESS /FAILED bassed on the outcome.
*/
static s32
adux1020_sample_raw_config_init(struct adux1020 *pst_adux)
{
	s32 ret = 0;
	s32 i = 0;
	u16 addr;
	u16 value = 0;

	struct adux_platform_data *config_reg;
	if (pst_adux->runtime_pdata->sample_raw_reg_cnt != 0)
		config_reg = pst_adux->runtime_pdata;
	else
		config_reg = pst_adux->pdata;
	/** configuration forproximity  registers */
	ADUX1020_dbg(" %s\n", __func__);
	for (i = 0; i < config_reg->sample_raw_reg_cnt; i++) {
		addr = (u16) ((config_reg->sample_raw_regs[i] & 0xffff0000) >>
			      16);
		value = (u16) (config_reg->sample_raw_regs[i] & 0x0000ffff);
		if (!(addr == 0x45 || addr == 0x48 || addr == 0x49)) {
			ADUX1020_dbg("Writing 0x%02x with Value 0x%04x\n",
				      addr, value);

			ret = adux1020_write(pst_adux, addr, value);
			if (ret < 0) {
				ADUX1020_dbg(" %s i2c wr err %d\n",
					      __func__, ret);
				return FAILED;
			}
		} else {
			ADUX1020_dbg("!!!!Writing 0x%02x with Value 0x%04x\n",
				      addr, value);
		}
	}
	/*addr = value = 0; */
	for (i = 0; i < config_reg->sample_raw_reg_cnt; i++) {
		addr = (u16) ((config_reg->sample_raw_regs[i] & 0xffff0000) >>
			      16);
		value = adux1020_read(pst_adux, addr);
		ADUX1020_dbg("Address 0x%02x\tValue 0x%04x\n", addr, value);
	}
	return SUCCESS;
}

/**
This function is used to configure the fifo and
	also for the updation of the same
@param pst_adux  pointer to ADUX1020 chip structure
@return s32
*/
static s32
adux1020_update_config_init(struct adux1020 *pst_adux)
{
	u16 read_reg;
	ADUX1020_dbg(" %s\n", __func__);
	pst_adux->fifo_th = adux1020_update_fifo_th(pst_adux);
	/*set value of proximity type during configuration*/
	read_reg = adux1020_read(pst_adux, ADUX_PROX_TYPE);
	ADUX1020_dbg("Prev val 0x2F - 0x%04x\n", read_reg);
	read_reg = (read_reg & 0x7FFF) | (pst_adux->prox_type << 15);
	ADUX1020_dbg("after set Prox_type 0x2F - 0x%04x\n", read_reg);
	adux1020_write(pst_adux, ADUX_PROX_TYPE, (u16) read_reg);
	return SUCCESS;
}

/**
This function is used for configuration initilization of the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@param config store the config mode number
@return  u16 return value
*/
static s32
adux1020_config_init(struct adux1020 *pst_adux, u16 config)
{
	s32 ret = 0;

	ADUX1020_dbg(" %s\n", __func__);
	switch (config) {
	case CONFIG(GENERAL):
		ret = adux1020_general_config_init(pst_adux);
		break;
	case CONFIG(PROXIMITY):
		ret = adux1020_proximity_config_init(pst_adux);
		break;
	case CONFIG(GESTURE):
		ret = adux1020_gesture_config_init(pst_adux);
		break;
	case CONFIG(SAMPLE_XYI):
		ret = adux1020_sample_xyi_config_init(pst_adux);
		break;
	case CONFIG(SAMPLE_RAW):
		ret = adux1020_sample_raw_config_init(pst_adux);
		break;
	default:
		ADUX1020_dbg("INVALID INPUT\n");
		break;
	}
	ret = adux1020_update_config_init(pst_adux);
	return ret;
}

/*CONFIG RELATED - END ********************************************/
/*ISR & WORKQUEUE - START *****************************************/
/**
This function is used to find the status of proximity and call proximity event
@param pst_adux the adux1020 chip structure.
@return void
*/
static void
adux1020_prox_status(struct adux1020 *pst_adux)
{
	ADUX1020_dbg("%s\n", __func__);

	ADUX1020_dbg("Intr status - 0x%04x\n",\
			pst_adux->curr_intr);
	if ((pst_adux->curr_intr & PROXIMITY_1) == PROX_ON1) {
		if (pst_adux->prox_state.proximity1 != (u8) ON) {
			/*ADUX1020_dbg("Intr status - 0x%04x\n",\
				pst_adux->curr_intr);*/
			pst_adux->prox_state.proximity1 = (u8) ON;
			adux1020_prox_dist_evt(pst_adux->prox1_input,\
						ON);

			/* ASUS report input event */
			input_report_abs(pst_adux->prox1_input, ABS_DISTANCE, 0);
			input_sync(pst_adux->prox1_input);
			printk("[adux1020][ps] trigger panel off\n");
			
		} else {
			/* no need to send event */
		}
	} else if ((pst_adux->curr_intr & PROXIMITY_1) == PROX_OFF1) {
		if (pst_adux->prox_state.proximity1 != (u8) OFF) {
			adux1020_prox_dist_evt(pst_adux->prox1_input,\
						OFF);

			/* ASUS report input event */
			input_report_abs(pst_adux->prox2_input, ABS_DISTANCE, 1);
			input_sync(pst_adux->prox2_input);
			printk("[adux1020][ps] trigger panel on\n");
			
			pst_adux->prox_state.proximity1 = (u8) OFF;
		} else {
			/* no need to send event */
		}
	}
	if ((pst_adux->curr_intr & PROXIMITY_2) == PROX_ON2) {
		if (pst_adux->prox_state.proximity2 != (u8) ON) {
			/*ADUX1020_dbg("Intr status - 0x%04x\n",\
				pst_adux->curr_intr);*/
			pst_adux->prox_state.proximity2 = (u8) ON;
			adux1020_prox_dist_evt(pst_adux->prox2_input,\
						ON);
			/* ASUS report input event */
			input_report_abs(pst_adux->prox2_input, ABS_DISTANCE, 0);
			input_sync(pst_adux->prox2_input);
			printk("[adux1020][ps] trigger panel off\n");
		
		} else {
			/* no need to send event */
		}
	} else if ((pst_adux->curr_intr & PROXIMITY_2) == PROX_OFF2) {
		if (pst_adux->prox_state.proximity2 != (u8) OFF) {
			adux1020_prox_dist_evt(pst_adux->prox2_input,\
						OFF);

			/* ASUS report input event */
			input_report_abs(pst_adux->prox2_input, ABS_DISTANCE, 1);
			input_sync(pst_adux->prox2_input);
			printk("[adux1020][ps] trigger panel on\n");
			
			pst_adux->prox_state.proximity2 = (u8) OFF;
		} else {
			/* no need to send event */
		}
	}
}

/**
This function is used to read the fifo data for
	proximity mode of the ADUX1020 chip
@param prox_inputdev  pointer to ADUX1020 chip structure
@return s32  number of data read from fifo of the ADUX1020 chip
*/
static s32
adux1020_proximity(struct input_dev *prox_inputdev)
{
	struct adux1020 *pst_adux =
	    (struct adux1020 *)input_get_drvdata(prox_inputdev);
	s32 ret = 0;

	ADUX1020_dbg(" %s\n", __func__);
	ret = adux1020_fifo_data(pst_adux);
	adux1020_proxi_evt(prox_inputdev);
	return ret;
}

/**
This function is used to read the fifo data for
	gesture mode of the ADUX1020 chip
@param pst_adux  pointer to ADUX1020 chip structure
@return s32
*/
static s32
adux1020_gesture(struct adux1020 *pst_adux)
{
	s32 ret = 0;
	ADUX1020_dbg(" %s\n", __func__);
	/*Read FIFO Data  */
	ret = adux1020_fifo_data(pst_adux);

	pst_adux->adux_gest_data.dirX = (u8) ((pst_adux->pu16_databuff[0] &
					       0x60) >> 5);
	pst_adux->adux_gest_data.dirY = (u8) ((pst_adux->pu16_databuff[0] &
					       0x18) >> 3);
	pst_adux->adux_gest_data.dirZ = (u8) ((pst_adux->pu16_databuff[0] &
					       0x80) >> 7);
	pst_adux->adux_gest_data.gest_length =\
		(u8) ((pst_adux->pu16_databuff[0] >> 8) & 0xFF);

	if (pst_adux->adux_gest_data.gest_length != 0xFF)
		adux1020_gest_evt(pst_adux);
	else
		ADUX1020_dbg("gest length - 0x%04x\n",\
			pst_adux->adux_gest_data.gest_length);
	return ret;
}

/**
This function is used to read the fifo data for sample mode of the ADUX1020 chip
@param pst_adux  pointer to ADUX1020 chip structure
@return s32  number of data read from fifo of the ADUX1020 chip
*/
static s32
adux1020_sample(struct adux1020 *pst_adux)
{
	s32 ret = 0;

	ADUX1020_info(" %s\n", __func__);
	/*Read FIFO Data  */
	ret = adux1020_fifo_data(pst_adux);
	adux1020_sample_evt(pst_adux->gest_input);
	return ret;
}

/**
This function is used to process the interrupts and
	is called from the Work queue
@param pst_adux the adux1020 chip structure.
@return void
*/
static void
adux1020_work(struct adux1020 *pst_adux)
{
	ADUX1020_info(" %s\n", __func__);
	mutex_lock(&pst_adux->mutex);
	disable_irq(pst_adux->irq);
	/*Read Intr status and update Fifo size to the adux structure */
	if (adux1020_rd_intr_fifo(pst_adux) != SUCCESS) {
		/*error */
		ADUX1020_dbg("error\n");
	}
	switch (pst_adux->mode) {
	case PROXIMITY_NONE:
		adux1020_prox_status(pst_adux);
		break;
	case PROXIMITY_I:
		{
			adux1020_prox_status(pst_adux);
			adux1020_proximity(pst_adux->prox2_input);
		}
		break;
	case GESTURE:
		{
			adux1020_gesture(pst_adux);
		}
		break;
	case SAMPLE_XYI:
	case SAMPLE_RAW:
		{
			adux1020_sample(pst_adux);
		}
		break;
	case FIFO:
		{
			ADUX1020_dbg(" INT_FIFO_FULL triggered\n");
			/*      FIFO mainted in driver need to reset the buffer,
			   if it is in event driven
			   If FIFO interrupt raised during in
			   EVENT DRIVEN model,
			   TBD->the buffer allocated for the data buffer
			   need to cleared and reallocated */
			/* TBD clear the data buffer if allocated. */
			adux1020_flush_buffer(pst_adux);
			/* chk whther need to reset the device */
		}
		break;
	case IDLE:
		{
			adux1020_fifo_data(pst_adux);
			adux1020_flush_buffer(pst_adux);
		}
		break;
	default:
		{
			ADUX1020_dbg("mode = 0x%x\n", pst_adux->mode);
			ADUX1020_dbg("curr_intr = 0x%x\n",
				      (u32) pst_adux->curr_intr);
			ADUX1020_dbg("INVALID!! INT triggered\n");
		}
		break;
	};
	/*  Clear IRQ Status - need condition for proximity */
	adux1020_clr_intr_status(pst_adux);
	enable_irq(pst_adux->irq);
    adux1020_clr_intr_status(pst_adux); //Mellow
	mutex_unlock(&pst_adux->mutex);
}

/**
This work function that is executed from the IRQ
@param work the work assigned.
@return void
*/
static void
adux1020_work_queue(struct work_struct *work)
{
	struct adux1020 *pst_adux = container_of(work,
						 struct adux1020,
						 work);

	ADUX1020_info(" %s\n", __func__);

	adux1020_work(pst_adux);

}

/**
This ISR function is the back end of the irs in the thread
	to handle the sleeping functions
@param irq the IRQ assigned.
@param dev_id the Device ID
@return irqreturn_t IRQ_HANDLED values
*/
static irqreturn_t
adux1020_isr(int irq, void *dev_id)
{
	struct adux1020 *pst_adux = dev_id;
	unsigned long flags;

	local_irq_save(flags);
	local_irq_disable();
	if (!work_pending(&pst_adux->work))
		schedule_work(&pst_adux->work);
	else
		ADUX1020_dbg("work pending!!\n");

	local_irq_restore(flags);
	return IRQ_HANDLED;
}

/*ISR & WORKQUEUE - END *******************************************/
/*SYSFS - START ***************************************************/
/**
This function is used for I2C read from the sysfs
	file system of the ADUX1020 Chip
This is called from the Sysfs functions
@param pst_adux  pointer to ADUX1020 chip structure
@return s16 the bytes of read data.
*/
static s16
adux1020_sysfs_I2C_rd(struct adux1020 *pst_adux)
{
	u16 value = 0;
/*	ADUX1020_dbg(" %s\n", __func__);*/
	/*adux1020_read return length on success and negative on failure */
	value = adux1020_read(pst_adux, pst_adux->sysfs_I2C_regaddr);
	return value;
}

/**
This function is used for I2C write from the sysfs
	file system of the ADUX1020 Chip
This is called from the Sysfs functions
@param pst_adux  pointer to ADUX1020 chip structure
@return s16 the bytes of written data.
*/
static s16
adux1020_sysfs_I2C_wr(struct adux1020 *pst_adux)
{
	s32 ret = 0;
	u16 value = pst_adux->sysfs_I2C_regval;

	ADUX1020_dbg(" %s\n", __func__);
	/*pst_adux->wrtie return length on success and return -error on error */
	ret = adux1020_write(pst_adux, pst_adux->sysfs_I2C_regaddr, value);
	if (ret < 0) {
		ADUX1020_dbg(" %s i2c read error %d\n", __func__, ret);
		return -1;
	} else {
		return value;
	}
}

/**
This function is used to show the mode of operation of the driver
*	@param adux_dev The device ID and Information structure(Linux Standard argument)
*	@param attr standard Linux Device attribute.
*	@param buf The buffer to store the read data.
*	@return ssize_t The count of the data sent to the user buffer.
*/

static ssize_t
adux1020_sysfs_mode_show(struct device *adux_dev,
			 struct device_attribute *attr, char *buf)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	ADUX1020_dbg("OP_MODE 0x%x\n\n", pst_adux->mode);
	return sprintf(buf, "0x%x", pst_adux->mode);
}

/**
This function is used to store the mode of operation of the driver
*	@param adux_dev The device ID and Information structure(Linux Standard argument)
*	@param attr standard linux device attributes.
*	@param buf The buffer to store the Read data
*	@param count The size of the Buffer.
*	@return ssize_t The Size of the User buffer.
*/
static ssize_t
adux1020_sysfs_mode_store(struct device *adux_dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	u16 mode = 0;
	u16 parse_data[2];

	memset(parse_data, 0, sizeof(unsigned short) * 2);
	cmd_parsing(buf, 1, parse_data);

	mode = parse_data[0];

	if ((mode != OFF) || (mode != FORCE)) {
		ADUX1020_dbg("OP_MODE 0x%x\n", mode);
		adux1020_mode_switching(pst_adux, mode);
	} else {
		ADUX1020_dbg(" Invalid mode !!!\n");
	}
	return count;
}

/**
This function is used to display the last read register value
*	@param adux_dev The device ID and Information structure(Linux Standard argument)
*	@param attr Standard linux device attributes.
*	@param buf The buffer to store the Read data,USER space buffer.
*	@return ssize_t The count of the data sent to the user buffer.
*/
static ssize_t
adux1020_sysfs_reg_read_show(struct device *adux_dev,
			     struct device_attribute *attr, char *buf)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	ADUX1020_dbg("Regval : 0x%4x\n", pst_adux->sysfslastreadval);
	return sprintf(buf, "0x%x", pst_adux->sysfslastreadval);
}

/**
This function is used to Read the register values of the ADUX1020 chip
*	@param adux_dev The device ID and Information structure(Linux Standard argument)
*	@param attr Standard linux device attributes.
*	@param buf The buffer to store the Read data
*	@param count The count of the data in the buffer provided from user.
*	@return ssize_t The count of the data received by default.
*/
static ssize_t
adux1020_sysfs_reg_read_store(struct device *adux_dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	unsigned short addr, cnt;
	unsigned short parse_data[4];
	unsigned short ret;

	memset(parse_data, 0, sizeof(unsigned short) * 4);
	cmd_parsing(buf, 2, parse_data);
	addr = parse_data[0];
	cnt = parse_data[1];

	mutex_lock(&pst_adux->mutex);

	pst_adux->sysfs_I2C_regaddr = addr;

	ret = adux1020_sysfs_I2C_rd(pst_adux);
	if (ret != (-1)) {
		ADUX1020_dbg(" RegRead_Store : addr = 0x%04X,"\
				"value = 0x%04X\n", addr, ret);
		pst_adux->sysfslastreadval = ret;
	} else {
		ADUX1020_dbg("%s Error\n", __func__);
		pst_adux->sysfslastreadval = (unsigned short) -1;
	}

	mutex_unlock(&pst_adux->mutex);
	return count;
}

/**
This function is used to write to a register in the ADUX 1020 / ADUX 1020i chip
*	@param adux_dev The device ID and Information structure(Linux Standard argument)
*	@param attr Standard linux device attributes.
*	@param buf The buffer to store the Read data
*	@param count The count of the data in the user space buffer.
*	@return ssize_t returns The count of the data received.
*/
static ssize_t
adux1020_sysfs_reg_write_store(struct device *adux_dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	unsigned short addr, cnt;
	unsigned short parse_data[6];
	unsigned short value[4];
	unsigned short ret;
	memset(parse_data, 0, sizeof(unsigned short) * 6);
	cmd_parsing(buf, 6, parse_data);

	addr = parse_data[0];
	cnt = parse_data[1];

	value[0] = parse_data[2];
	value[1] = parse_data[3];

	if (!(addr == 0x45 || addr == 0x48 || addr == 0x49)) {
		pst_adux->sysfs_I2C_regaddr = addr;
		pst_adux->sysfs_I2C_regval = value[0];

		mutex_lock(&pst_adux->mutex);

		ret = adux1020_sysfs_I2C_wr(pst_adux);

		ADUX1020_dbg(" RegWrite Store : reg = 0x%X,"\
				"value = 0x%X, ret : 0x%x\n",\
				pst_adux->sysfs_I2C_regaddr,\
				pst_adux->sysfs_I2C_regval, ret);
		if (addr == 0x2F)
			adux1020_update_config_init(pst_adux);
		mutex_unlock(&pst_adux->mutex);
	} else {

	}


	return count;
}

/**
This function is used to display the custome keymap in the ADUX 1020 / ADUX 1020i chip
*	@param dev The Device ID and Information structure(Linux Standard argument)
*	@param attr Standard linux device attributes.
*	@param buf The buffer to store the read data.
*	@return ssize_t The size of the written data to the user buffer.
*/
static ssize_t
adux1020_sysfs_mapkeycodes_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct adux1020 *pst_adux = dev_get_drvdata(dev);
	ADUX1020_dbg("KeyMapCode : %d\n", pst_adux->KeyMapCode);
	return sprintf(buf, "%d", pst_adux->KeyMapCode);
}

/**
This function is used to display the custome keymap in the ADUX 1020 / ADUX 1020i chip
*	@param dev The Device ID and Information structure(Linux Standard argument)
*	@param attr Standard linux device attributes.
*	@param buf The buffer to store the read data.
*	@param count The count value
*	@return ssize_t The size of the written data to the user buffer.
*/
static ssize_t
adux1020_sysfs_mapkeycodes_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned int KeyMapCode = 0;
	struct adux1020 *pst_adux = dev_get_drvdata(dev);

	sscanf(buf, "%d", &KeyMapCode);
	ADUX1020_dbg(" Keymap Stored = %d\n", KeyMapCode);
	pst_adux->KeyMapCode = KeyMapCode;
	adux1020_keycodemapping(pst_adux);
	return count;
}

/**
@brief This API used to get current Gesture Out Mode stored,
@note depending upon the u32_GestOutMode it display
@note zero(0 - Input Event )  and one(1 - Read Interface )
@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the adux1020
@param buf The buffer to store the Read data
@return ssize_t The Size of the Read Data, 0 if not Read
*/
static ssize_t
adux1020_sysfs_gestout_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct adux1020 *pst_adux = dev_get_drvdata(dev);

	ADUX1020_dbg("Gest Out Mode (0 - Read Interface,"\
			"1 - Input Event) is %d\n", pst_adux->GestOutMode);
	return sprintf(buf, "%d", pst_adux->GestOutMode);
}

/**
@brief This API used to set Gesture Out Mode stored, from userspace
\note This is used to store the register addres to write the data.
\note invoked by the used from the echo command in /../sysfs/<Device> region.
@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the adux1020
@param buf The buffer to store the Read data
@param count The number of bytes to write from the buffer
@return ssize_t The Size of the writen Data, 0 if not writen
*/
static ssize_t
adux1020_sysfs_gestout_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int GestOutMode = 0;
	struct adux1020 *pst_adux = dev_get_drvdata(dev);
	sscanf(buf, "%d", &GestOutMode);

	ADUX1020_dbg("Gesture Output Event = %d\n", GestOutMode);
	pst_adux->GestOutMode = GestOutMode;
	return count;
}

/**
@brief This API used to display the status of sysfs vendor
@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the adux1020
@param buf The buffer to store the Read data
@return ssize_t The Size of the Read Data.
*/
static ssize_t
adux1020_sysfs_vendor_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VENDOR);
}

/**
@brief This API used to display the status of sysfs product
@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the adux1020
@param buf The buffer to store the Read data
@return ssize_t The Size of the Read Data.
*/
static ssize_t
adux1020_sysfs_product_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct adux1020 *pst_adux = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pst_adux->product);
}

/**
This function is used to display the register's cofiguration of the mode
	at runtime in the ADUX 1020 / ADUX 1020i chip
*	@param adux_dev The Device ID and Information structure(Linux Standard argument)
*	@param attr Standard linux device attributes.
*	@param buf The buffer to store the read data
*	@return ssize_t The count of the data received.
*	@note configuration Supported
	General config - "1"
	proximity config - "2"
	gesture config - "3"
	sample XYI config - "4"
	sample RAW config - "5"
*/
static ssize_t
adux1020_sysfs_config_show(struct device *adux_dev,
			   struct device_attribute *attr, char *buf)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);

	struct adux_platform_data *config_reg = pst_adux->runtime_pdata;

	u32 cnt = 0;
	u32 len = 0;
	u16 config;

	config = config_reg->config;

	switch (config) {
	case CONFIG(GENERAL):
		{
			for (cnt = 0; cnt < config_reg->general_reg_cnt;
			     cnt++) {
				sprintf(buf + len, "0x%04x%04x",
					(config_reg->general_regs[cnt] &
					 0xFFFF0000) >> 16,
					(config_reg->general_regs[cnt] &
					 0xFFFF));
				len = strlen(buf);
			}
		}
		break;
	case CONFIG(PROXIMITY):
		{
			for (cnt = 0; cnt < config_reg->proximity_reg_cnt;
			     cnt++) {
				sprintf(buf + len, "0x%04x%04x ",
					(config_reg->proximity_regs[cnt] &
					 0xFFFF0000) >> 16,
					(config_reg->proximity_regs[cnt] &
					 0xFFFF));
				len = strlen(buf);
			}
		}
		break;
	case CONFIG(GESTURE):
		{
			for (cnt = 0; cnt < config_reg->gesture_reg_cnt;
			     cnt++) {
				sprintf(buf + len, "0x%04x%04x ",
					(config_reg->gesture_regs[cnt] &
					 0xFFFF0000) >> 16,
					(config_reg->gesture_regs[cnt] &
					 0xFFFF));
				len = strlen(buf);
			}
		}
		break;
	case CONFIG(SAMPLE_XYI):
		{
			for (cnt = 0; cnt < config_reg->sample_xyi_reg_cnt;
			     cnt++) {
				sprintf(buf + len, "0x%04x%04x ",
					(config_reg->sample_xyi_regs[cnt] &
					 0xFFFF0000) >> 16,
					(config_reg->sample_xyi_regs[cnt] &
					 0xFFFF));
				len = strlen(buf);
			}
		}
		break;
	case CONFIG(SAMPLE_RAW):
		{
			for (cnt = 0; cnt < config_reg->sample_xyi_reg_cnt;
			     cnt++) {
				sprintf(buf + len, "0x%04x%04x ",
					(config_reg->sample_raw_regs[cnt] &
					 0xFFFF0000) >> 16,
					(config_reg->sample_raw_regs[cnt] &
					 0xFFFF));
				len = strlen(buf);
			}
		}
		break;
	default:
		ADUX1020_dbg("INVALID INPUT\n");
		break;
	};
	return strlen(buf);
}

/**
This function is used to configure the registers based on the modes at runtime in the ADUX 1020 / ADUX 1020i chip
*	@param adux_dev The Device ID and Information structure(Linux Standard argument)
*	@param attr Standard linux device attributes.
*	@param buf The buffer to store the read data
*	@param count The count of the data in the user space buffer.
*	@return ssize_t The count of the data received..
*	@note Modes Supported
	General config - mode "1"
	proximity config - mode "2"
	gesture atom config - mode "3"
	gesture raw config - mode "4"
*/
static ssize_t
adux1020_sysfs_config_store(struct device *adux_dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	/* struct test_config_register config_reg[50]; */
	struct adux_platform_data *config_reg = pst_adux->runtime_pdata;

	long value = 0;
	u32 cnt = 0;
	u32 ret;
	u32 returnval;
	u32 err = 0;
	s8 *temp_buf = NULL;
	s8 *temp_buf1 = NULL;
	s8 *hex_prefix = "0x";
	u16 loop_cnt = 2;
	u16 config = 0;
	u32 reg_cnt = 0;
	u32 pos = 0;

	adux1020_enter_idle(pst_adux);

	while (loop_cnt != 0) {
		temp_buf = strsep((char **)&buf, "\n");
		temp_buf1 = temp_buf;
		temp_buf1 = strsep((char **)&temp_buf1, "\r");
		if (temp_buf1 != NULL)
			temp_buf = temp_buf1;
		else
			temp_buf1 = NULL;

		pos = 0;
		if ((temp_buf[pos] == '0') &&
		    (temp_buf[pos + 1] == 'x' || temp_buf[pos + 1] == 'X')) {
			if (kstrtoul(&temp_buf[pos + 2],
				     16, (long unsigned int *)&value))
				value = 0;
		} else {
			if (kstrtoul(&temp_buf[pos],
				     10, (long unsigned int *)&value))
				value = 0;
		}

		if (loop_cnt == 2) {
			config = (unsigned short) value;
			config_reg->config = config;
		} else if (loop_cnt == 1) {
			reg_cnt = (u32) value;
			switch (config_reg->config) {
			case CONFIG(GENERAL):	/* general config */
				config_reg->general_reg_cnt = (u32) value;
				break;
			case CONFIG(PROXIMITY):	/* proximity config */
				config_reg->proximity_reg_cnt = (u32) value;
				break;
			case CONFIG(GESTURE):
				config_reg->gesture_reg_cnt = (u32) value;
				break;
			case CONFIG(SAMPLE_XYI):
				config_reg->sample_xyi_reg_cnt = (u32) value;
				break;
			case CONFIG(SAMPLE_RAW):
				config_reg->sample_raw_reg_cnt = (u32) value;
				break;
			default:
				ADUX1020_dbg("INVALID CNT VALUE\n");
				ADUX1020_dbg("enter:\n"
					     "1 to load General config\n"
					     "2 to load Proximity config\n"
					     "3 to load Gesture config\n"
					     "4 to load Sample XYI config\n"
					     "5 to load Sample RAW config\n");
				break;
			}
		}
		loop_cnt--;
	}

	loop_cnt = 0;
	ADUX1020_dbg("CONFIG = %hu\nCONFIG_CNT = %d\n", config, reg_cnt);

	while ((temp_buf = strsep((char **)&buf, "\n")) && err != 1) {

		temp_buf1 = temp_buf;
		temp_buf1 = strsep((char **)&temp_buf1, "\r");
		if (temp_buf1 != NULL)
			temp_buf = temp_buf1;
		else
			temp_buf1 = NULL;

		ret = strnicmp(hex_prefix, temp_buf, 2);
		if (ret == 0)
			returnval = (int) kstrtoul(temp_buf, 0, &value);
		else
			returnval = (int) kstrtoul(temp_buf, 10, &value);
		switch (config) {
		case CONFIG(GENERAL):	/* general config */
			{
				config_reg->general_regs[cnt] = (u32) value;
				ADUX1020_dbg("address = 0x%x\tdata = 0x%x\n",
					      (int) (value >> 16),
					      (int) (value & 0xFFFF));
			}
			break;
		case CONFIG(PROXIMITY):
			{
				config_reg->proximity_regs[cnt] = (u32) value;
				ADUX1020_dbg("address = 0x%x\tdata = 0x%x\n",
					      (int) (value >> 16),
					      (int) (value & 0xFFFF));
			}
			break;
		case CONFIG(GESTURE):
			{
				config_reg->gesture_regs[cnt] = (u32) value;
				ADUX1020_dbg("address = 0x%x\tdata = 0x%x\n",
					      (int) (value >> 16),
					      (int) (value & 0xFFFF));
			}
			break;
		case CONFIG(SAMPLE_XYI):
			{
				config_reg->sample_xyi_regs[cnt] = (u32) value;
				ADUX1020_dbg("address = 0x%x\tdata = 0x%x\n",
					      (int) (value >> 16),
					      (int) (value & 0xFFFF));
			}
			break;
		case CONFIG(SAMPLE_RAW):
			{
				config_reg->sample_raw_regs[cnt] = (u32) value;
				ADUX1020_dbg("address = 0x%x\tdata = 0x%x\n",
					      (int) (value >> 16),
					      (int) (value & 0xFFFF));
			}
			break;
		default:
			{
				ADUX1020_dbg("INVALID INPUT\n");
				err = 1;
			}
			break;
		}
		loop_cnt++;
		cnt++;
		if (cnt > (reg_cnt - 1))
			break;
	}
	mutex_lock(&pst_adux->mutex);
	if (loop_cnt != 0 && err != 1) {
		ADUX1020_dbg(" START CONFIG\n");
		if (config == 1)
			adux1020_reset_device(pst_adux);
		ret = adux1020_config_init(pst_adux, config);
		/*adux1020_clock_calibration(pst_adux);*/
	} else {
		ADUX1020_dbg("CONFIG FAILED !!\n");
	}
	adux1020_enter_idle(pst_adux);
	mutex_unlock(&pst_adux->mutex);
	return count;
}

/**
@brief This API used to set the status of sysfs configuration
@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the adux1020
@param buf The buffer to store the Read data
@param count buffer count
@return ssize_t The Size of the Read Data.
*/
static ssize_t
adux1020_sysfs_config_filp_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct adux1020 *pst_adux = dev_get_drvdata(dev);
	u16 config = 0;
	u32 ret = 0;

	/**
	@see cmd_parsing
	*/
	cmd_parsing(buf, 1, &config);

	ADUX1020_dbg("CONFIG value frm USER = %hu\n", config);

	/*  IDLE, RESET,OFF Mode */
	adux1020_enter_idle(pst_adux);
	switch (config) {
	case CONFIG(GENERAL):
		ret = adux_filp_start_calib(GENERAL_CONFIG, pst_adux, config);
		if (ret == -1) {
			pst_adux->runtime_pdata->general_reg_cnt = 0;
			ADUX1020_dbg("invalid F - %s\n", GENERAL_CONFIG);
			ADUX1020_dbg("running stored general config\n");
		}
		adux1020_reset_device(pst_adux);
		adux1020_config_init(pst_adux, config);
		/*adux1020_clock_calibration(pst_adux);*/
		break;
	case CONFIG(PROXIMITY):
		ret = adux_filp_start_calib(PROXIMITY_CONFIG, pst_adux, config);
		if (ret == -1) {
			pst_adux->runtime_pdata->proximity_reg_cnt = 0;
			ADUX1020_dbg("FILE NOT FOUND - %s\n", PROXIMITY_CONFIG);
			ADUX1020_dbg("running stored proximity config\n");
		}
		adux1020_config_init(pst_adux, config);
		break;
	case CONFIG(GESTURE):
		ret = adux_filp_start_calib(GESTURE_CONFIG, pst_adux, config);
		if (ret == -1) {
			pst_adux->runtime_pdata->gesture_reg_cnt = 0;
			ADUX1020_dbg("FILE NOT FOUND - %s\n", GESTURE_CONFIG);
			ADUX1020_dbg("running stored gesture config\n");
		}
		adux1020_config_init(pst_adux, config);
		break;
	case CONFIG(SAMPLE_XYI):
		ret = adux_filp_start_calib(SAMPLE_XYI_CONFIG,
					    pst_adux, config);
		if (ret == -1) {
			pst_adux->runtime_pdata->sample_xyi_reg_cnt = 0;
			ADUX1020_dbg("FILE NOT FOUND - %s\n",
				     SAMPLE_XYI_CONFIG);
			ADUX1020_dbg("running stored sample XYI config\n");
		}
		adux1020_config_init(pst_adux, config);
		break;
	case CONFIG(SAMPLE_RAW):
		ret = adux_filp_start_calib(SAMPLE_RAW_CONFIG,
					    pst_adux, config);
		if (ret == -1) {
			pst_adux->runtime_pdata->sample_raw_reg_cnt = 0;
			ADUX1020_dbg("FILE NOT FOUND - %s\n",
				     SAMPLE_RAW_CONFIG);
			ADUX1020_dbg("running stored sample RAW config\n");
		}
		adux1020_config_init(pst_adux, config);
		break;
	default:
		ADUX1020_dbg("Invalid commad\n");
		ADUX1020_dbg("enter:\n1 to load General config\n"
			     "2 to load Proximity config\n"
			     "3 to load Gesture config\n"
			     "4 to load Sample XYI config\n"
			     "5 to load Sample RAW config\n");
		break;
	}
	adux1020_enter_idle(pst_adux);
	return count;
}

/**
This function is used to display the status of Fifo prevent bit value
*       @param adux_dev The device ID and Information structure(Linux Standard argument)
*       @param attr Standard linux device attributes.
*       @param buf The buffer to store the Read data,USER space buffer.
*       @return ssize_t The count of the data sent to the user buffer.
*/
static ssize_t
adux1020_sysfs_fifo_prevent_show(struct device *adux_dev,
				 struct device_attribute *attr, char *buf)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	ADUX1020_dbg("Fifo prevent - 0x%x,\n0 - disable, 1 - enable\n",
		     pst_adux->r_fifo_prevent);
	return sprintf(buf, "0x%x", pst_adux->r_fifo_prevent);
}

/**
This function is used to enable/disable the status of Fifo prevent bit value
*       @param adux_dev The device ID and Information structure(Linux Standard argument)
*       @param attr Standard linux device attributes.
*       @param buf The buffer to store the Read data
*       @param count The count of the data in the buffer provided from user.
*       @return ssize_t The count of the data received by default.
*/
static ssize_t
adux1020_sysfs_fifo_prevent_store(struct device *adux_dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	unsigned short read_reg;
	unsigned short parse_data[2];

	memset(parse_data, 0, sizeof(unsigned short) * 2);
	cmd_parsing(buf, 1, parse_data);

	if (parse_data[0] != 1)
		pst_adux->r_fifo_prevent = 0;
	else
		pst_adux->r_fifo_prevent = parse_data[0];

	mutex_lock(&pst_adux->mutex);
	read_reg = adux1020_read(pst_adux, ADUX_OP_MODE_ADDR);
	ADUX1020_dbg("OP - 0x%04x\n", read_reg);

	read_reg = (read_reg & R_FIFO_PREVENT_MASK) |\
			R_FIFO_PREVENT_EN(pst_adux->r_fifo_prevent);

	ADUX1020_dbg("after setting OP - 0x%04x\n", read_reg);
	adux1020_write(pst_adux, ADUX_OP_MODE_ADDR, (u16) read_reg);
	mutex_unlock(&pst_adux->mutex);
	return count;
}

/**
This function is used to display the status of Packet start bit value
*       @param adux_dev The device ID and Information structure(Linux Standard argument)
*       @param attr Standard linux device attributes.
*       @param buf The buffer to store the Read data,USER space buffer.
*       @return ssize_t The count of the data sent to the user buffer.
*/
static ssize_t
adux1020_sysfs_pack_start_show(struct device *adux_dev,
			       struct device_attribute *attr, char *buf)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	ADUX1020_dbg("Packet start enable - 0x%x,\n0 - disable, 1 - enable\n",
		     pst_adux->r_pack_start);
	return sprintf(buf, "0x%x", pst_adux->r_pack_start);
}

/**
This function is used to enable/disable the status of Packet start bit value
*       @param adux_dev The device ID and Information structure(Linux Standard argument)
*       @param attr Standard linux device attributes.
*       @param buf The buffer to store the Read data
*       @param count The count of the data in the buffer provided from user.
*       @return ssize_t The count of the data received by default.
*/
static ssize_t
adux1020_sysfs_pack_start_store(struct device *adux_dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	unsigned short read_reg;
	unsigned short parse_data[2];

	memset(parse_data, 0, sizeof(unsigned short) * 2);
	cmd_parsing(buf, 1, parse_data);
	if (parse_data[0] != 1)
		pst_adux->r_pack_start = 0;
	else
		pst_adux->r_pack_start = parse_data[0];

	mutex_lock(&pst_adux->mutex);
	read_reg = adux1020_read(pst_adux, ADUX_OP_MODE_ADDR);
	ADUX1020_dbg("OP - 0x%04x\n", read_reg);
	read_reg =
	    (read_reg & R_PACK_START_MASK) | R_PACK_START_EN(pst_adux->
							     r_pack_start);
	ADUX1020_dbg("after setting OP - 0x%04x\n", read_reg);
	adux1020_write(pst_adux, ADUX_OP_MODE_ADDR, (u16) read_reg);
	mutex_unlock(&pst_adux->mutex);
	return count;
}

/**
This function is used to display the status of Sample out start bit value
*       @param adux_dev The device ID and Information structure(Linux Standard argument)
*       @param attr Standard linux device attributes.
*       @param buf The buffer to store the Read data,USER space buffer.
*       @return ssize_t The count of the data sent to the user buffer.
*/
static ssize_t
adux1020_sysfs_sample_out_show(struct device *adux_dev,
			       struct device_attribute *attr, char *buf)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	ADUX1020_dbg
	    ("Sample out start enable - 0x%x,\n0 - disable, 1 - enable\n",
	     pst_adux->sample_out);
	return sprintf(buf, "0x%x", pst_adux->sample_out);
}

/**
This function is used to enable/disable the status of Sample out start bit value
*       @param adux_dev The device ID and Information structure(Linux Standard argument)
*       @param attr Standard linux device attributes.
*       @param buf The buffer to store the Read data
*       @param count The count of the data in the buffer provided from user.
*       @return ssize_t The count of the data received by default.
*/
static ssize_t
adux1020_sysfs_sample_out_store(struct device *adux_dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	unsigned short read_reg;
	unsigned short parse_data[2];

	memset(parse_data, 0, sizeof(unsigned short) * 2);
	cmd_parsing(buf, 1, parse_data);
	if (parse_data[0] != 1)
		pst_adux->sample_out = 0;
	else
		pst_adux->sample_out = 1;

	mutex_lock(&pst_adux->mutex);
	read_reg = adux1020_read(pst_adux, ADUX_OP_MODE_ADDR);
	ADUX1020_dbg("OP - 0x%04x\n", read_reg);
	read_reg =
	    (read_reg & R_SAMP_OUT_MASK) | R_SAMP_OUT_MODE(pst_adux->
							   sample_out);
	ADUX1020_dbg("after setting OP - 0x%04x\n", read_reg);
	adux1020_write(pst_adux, ADUX_OP_MODE_ADDR, (u16) read_reg);
	mutex_unlock(&pst_adux->mutex);
	return count;
}

/**
This function is used to display the status of proximity type
*       @param adux_dev The device ID and Information structure(Linux Standard argument)
*       @param attr Standard linux device attributes.
*       @param buf The buffer to store the Read data,USER space buffer.
*       @return ssize_t The count of the data sent to the user buffer.
*/
static ssize_t
adux1020_sysfs_prox_type_show(struct device *adux_dev,
			      struct device_attribute *attr, char *buf)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	ADUX1020_dbg("Prox type - 0x%x,\n0 - cross th, 1 - above/below th\n",
		     pst_adux->prox_type);
	return sprintf(buf, "0x%x", pst_adux->prox_type);
}
/**
This function is used to enable/disable the proximity type bit value
*       @param adux_dev The device ID and Information structure(Linux Standard argument)
*       @param attr Standard linux device attributes.
*       @param buf The buffer to store the Read data
*       @param count The count of the data in the buffer provided from user.
*       @return ssize_t The count of the data received by default.
*/
static ssize_t
adux1020_sysfs_prox_type_store(struct device *adux_dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	unsigned short read_reg;
	unsigned short parse_data[2];

	memset(parse_data, 0, sizeof(unsigned short) * 2);
	cmd_parsing(buf, 1, parse_data);
	if (parse_data[0] != 1)
		pst_adux->prox_type = 0;
	else
		pst_adux->prox_type = 1;

	mutex_lock(&pst_adux->mutex);
	read_reg = adux1020_read(pst_adux, ADUX_PROX_TYPE);
	ADUX1020_dbg("Prev val 0x2F - 0x%04x\n", read_reg);

	read_reg = (read_reg & 0x7FFF) | (pst_adux->prox_type << 15);
	ADUX1020_dbg("after set Prox_type 0x2F - 0x%04x\n", read_reg);
	adux1020_write(pst_adux, ADUX_PROX_TYPE, (u16) read_reg);
	mutex_unlock(&pst_adux->mutex);
	return count;
}

static int adux1020_show_adc( struct device *adux_dev, struct device_attribute *attr, char *buf )
{
	int adc = 0;
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);

	adc = adux1020_fifo_data(pst_adux);
	
	//printk("[adux1020][ps] Get 0x60 : 0x%x, 0x%d\n", adc, adc );
	
	return sprintf(buf, "%d\n", adc);
}
static DEVICE_ATTR(adc, S_IRWXU | S_IRWXG | S_IROTH, adux1020_show_adc, NULL );

/**
This function is used to display the status of clock calibration
*       @param adux_dev The device ID and Information structure(Linux Standard argument)
*       @param attr Standard linux device attributes.
*       @param buf The buffer to store the Read data,USER space buffer.
*       @return ssize_t The count of the data sent to the user buffer.
*/
static ssize_t
adux1020_sysfs_clock_calibration_show(struct device *adux_dev,
				      struct device_attribute *attr, char *buf)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	ADUX1020_dbg("clock calibration, 0xF - InProgress, 0 - completed\n");
	return sprintf(buf, "0x%x", pst_adux->clk_status);
}

/**
This function is used to start clock calibration
*       @param adux_dev The device ID and Information structure(Linux Standard argument)
*       @param attr Standard linux device attributes.
*       @param buf The buffer to store the Read data
*       @param count The count of the data in the buffer provided from user.
*       @return ssize_t The count of the data received by default.
*/
static ssize_t
adux1020_sysfs_clock_calibration_store(struct device *adux_dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	unsigned short parse_data[2];
	memset(parse_data, 0, sizeof(unsigned short) * 2);
	cmd_parsing(buf, 1, parse_data);
	if (parse_data[0] == 1 && pst_adux->clk_status == 0) {
		pst_adux->clk_status = 1;
		wake_up_process(pst_adux->clk_thread);
	} else if (parse_data[0] == 0xE && pst_adux->clk_status == 0xF) {
		pst_adux->clk_status = 0xE;
	} else {
		ADUX1020_dbg("clock calibration status 0x%x\n",\
				pst_adux->clk_status);
	}

	return count;
}
/*
  Sysfs device attributes list
 */
static DEVICE_ATTR(adux1020_config_filp,
		   S_IRWXUGO, NULL, adux1020_sysfs_config_filp_store);

static DEVICE_ATTR(adux1020_config,
		   S_IRWXUGO,
		   adux1020_sysfs_config_show, adux1020_sysfs_config_store);

static DEVICE_ATTR(adux1020_mode,
		   S_IRWXUGO,
		   adux1020_sysfs_mode_show, adux1020_sysfs_mode_store);

static DEVICE_ATTR(adux1020_reg_read,
		   S_IRWXUGO,
		   adux1020_sysfs_reg_read_show, adux1020_sysfs_reg_read_store);

static DEVICE_ATTR(adux1020_reg_write,
		   S_IRWXUGO, NULL, adux1020_sysfs_reg_write_store);

static DEVICE_ATTR(adux1020_mapkeycodes,
		   S_IRWXUGO,
		   adux1020_sysfs_mapkeycodes_show,
		   adux1020_sysfs_mapkeycodes_store);

static DEVICE_ATTR(adux1020_gestout,
		   S_IRWXUGO,
		   adux1020_sysfs_gestout_show, adux1020_sysfs_gestout_store);

static DEVICE_ATTR(adux1020_fifo_prevent,
		   S_IRWXUGO,
		   adux1020_sysfs_fifo_prevent_show,
		   adux1020_sysfs_fifo_prevent_store);

static DEVICE_ATTR(adux1020_pack_start,
		   S_IRWXUGO,
		   adux1020_sysfs_pack_start_show,
		   adux1020_sysfs_pack_start_store);

static DEVICE_ATTR(adux1020_sample_out,
		   S_IRWXUGO,
		   adux1020_sysfs_sample_out_show,
		   adux1020_sysfs_sample_out_store);

static DEVICE_ATTR(adux1020_proximity_type,
		   S_IRWXUGO,
		   adux1020_sysfs_prox_type_show,
		   adux1020_sysfs_prox_type_store);

static DEVICE_ATTR(adux1020_clock_calibration,
		   S_IRWXUGO,
		   adux1020_sysfs_clock_calibration_show,
		   adux1020_sysfs_clock_calibration_store);
static DEVICE_ATTR(vendor, S_IRWXUGO, adux1020_sysfs_vendor_show, NULL);

static DEVICE_ATTR(product, S_IRWXUGO, adux1020_sysfs_product_show, NULL);

/**
array of attributes
*/
static struct attribute *adux1020_attributes[] = {
	&dev_attr_adux1020_config_filp.attr,
	&dev_attr_adux1020_config.attr,
	&dev_attr_adux1020_reg_read.attr,
	&dev_attr_adux1020_reg_write.attr,
	&dev_attr_adux1020_gestout.attr,
	&dev_attr_adux1020_mapkeycodes.attr,
	&dev_attr_adux1020_mode.attr,
	&dev_attr_adux1020_fifo_prevent.attr,
	&dev_attr_adux1020_pack_start.attr,
	&dev_attr_adux1020_sample_out.attr,
	&dev_attr_adux1020_proximity_type.attr,
	&dev_attr_adux1020_clock_calibration.attr,
	&dev_attr_vendor.attr,
	&dev_attr_product.attr,
	&dev_attr_adc.attr,
	NULL
};

/**
attribute group
*/
static struct attribute_group adux1020_attr_group = {
	.attrs = adux1020_attributes
};

/*.........................................ASUS part................................................*/
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

atomic_t adux1020_proxm_update;

static int proxmdev_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	printk("[adux1020][ps] proxmdev_dev_open.\n");

	if (file->f_flags & O_NONBLOCK)
		printk("[adux1020][ps] proxmdl_dev_open (O_NONBLOCK)\n");

	atomic_set(&adux1020_proxm_update, 0); //initialize atomic.

	ret = nonseekable_open(inode, file);

	return ret;
}

static struct file_operations proxmdev_fops = {
    .owner = THIS_MODULE,
    .open = proxmdev_open,
};

static int adux1020_proxmdev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	switch( property ) 
	{
		case SENSORS_PROP_HI_THRESHOLD:
			break;

		case SENSORS_PROP_LO_THRESHOLD:
			break;

		case SENSORS_PROP_INTERVAL:
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
			break;

		case SENSORS_PROP_VENDOR:
			printk(DBGMSK_PRX_G4"[adux1020][ps] SENSORS_PROP_VENDOR.\n");
			sprintf(val->strval, "ADI");
			break;

		/* Add for debug only */
		case SENSORS_PROP_DBG:
			break;

		case SENSORS_PROP_ADC:
			break;

		case SENSORS_PROP_ATD_STATUS:
			printk(DBGMSK_PRX_G4"[adux1020][ps] get atd status: ongoing\n");
			break;

		default:
			printk(DBGMSK_PRX_G0"[adux1020][ps] default.\n");
			return -EINVAL;
	}
	return 0;
}

static int adux1020_proxmdev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	//int ret = 0;
	static bool bFirst = true;

	switch (property) 
	{
		case SENSORS_PROP_SWITCH:
			printk(DBGMSK_PRX_G4"[adux1020][ps] put SENSORS_PROP_SWITCH (%d).\n", (val->intval));
			if(bFirst) {
				printk("[adux1020] First run, begin calibration.\n");
				/*adux1020 initialization*/
				adux1020_clock_calibration(adux1020_data);
				printk("[adux1020] adux1020 initialization finish.\n");
				bFirst = false;
			}

			if(val->intval==1)		{	//turn on PS
				ADUX1020_dbg("OP_MODE 0x%x\n", PROXIMITY_I);
				adux1020_mode_switching(adux1020_data, PROXIMITY_I);
			}else	{	//turn off PS if val->intval==0 or other
				ADUX1020_dbg("OP_MODE 0x%x\n", IDLE);
				adux1020_mode_switching(adux1020_data, IDLE);
			}
			break;

		case SENSORS_PROP_HI_THRESHOLD:
			break;

		case SENSORS_PROP_LO_THRESHOLD:
			break;

		case SENSORS_PROP_INTERVAL:
			break;

		/* Add for debug only */
		case SENSORS_PROP_DBG:
			break;

		default:
			printk(DBGMSK_PRX_G0"[adux1020][ps] put default.\n");
			return -EINVAL;
	}
	return 0;
}

struct proximity_class_dev adux1020_proxmDev = {
	.id = SENSORS_PROXIMITY,
	.name = "psensor",
	.num_properties = ARRAY_SIZE(adux1020_proxmdev_properties),
	.properties = adux1020_proxmdev_properties,
	.get_property = adux1020_proxmdev_get_property,
	.put_property = adux1020_proxmdev_put_property,
	.fops = &proxmdev_fops
};
/*.........................................ASUS part................................................*/

/**
@brief This API used to create the adux1020 sysfs attribute link to
	common location.
@param adux_dev Device Structure to the adux1020 chip
@return int The status of sensor link created
*/
static int
adux1020_sensor_link_create(struct device *adux_dev)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	u32 ret = 0;

	ADUX1020_info(" %s\n", __func__);
	pst_adux->adux_class = class_create(THIS_MODULE, "ADUX_SENSOR");
	pst_adux->adux_dev_attr = device_create(pst_adux->adux_class,
						NULL, 0, NULL, "%s", VENDOR);
	ret = sysfs_create_link(&pst_adux->adux_dev_attr->kobj,
				&adux_dev->kobj, "attribute");
	if (ret < 0)
		goto err_attribute_destroy;
	ret = sysfs_create_link(&pst_adux->adux_dev_attr->kobj,
				&pst_adux->gest_input->dev.kobj,
				pst_adux->gest_input->name);
	if (ret < 0)
		goto err_gest_destroy;
	ret = sysfs_create_link(&pst_adux->adux_dev_attr->kobj,
				&pst_adux->prox1_input->dev.kobj,
				ADUX1020_INPUT2_NAME);
	if (ret < 0)
		goto err_prox1_destroy;
	ret = sysfs_create_link(&pst_adux->adux_dev_attr->kobj,
				&pst_adux->prox2_input->dev.kobj,
				ADUX1020_INPUT3_NAME);
	if (ret < 0)
		goto err_prox2_destroy;
	return SUCCESS;
err_prox2_destroy:
	ADUX1020_info(" unable to create link\n");
	sysfs_remove_link(&pst_adux->adux_dev_attr->kobj,
			  pst_adux->prox1_input->name);
err_prox1_destroy:
	sysfs_remove_link(&pst_adux->adux_dev_attr->kobj,
			  pst_adux->gest_input->name);
err_gest_destroy:
	sysfs_remove_link(&pst_adux->adux_dev_attr->kobj, "ADUX1020_attribute");
err_attribute_destroy:
	device_destroy(pst_adux->adux_class, 0);
	class_destroy(pst_adux->adux_class);
}

/**
@brief This API used to destroy the adux1020 sysfs attribute link present
	in the common location.
@param adux_dev Device Structure to the adux1020 chip
@return void
*/
static void
adux1020_sensor_link_destroy(struct device *adux_dev)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	ADUX1020_info(" %s\n", __func__);
	sysfs_remove_link(&pst_adux->adux_dev_attr->kobj,
			  pst_adux->prox2_input->name);

	sysfs_remove_link(&pst_adux->adux_dev_attr->kobj,
			  pst_adux->prox1_input->name);
	sysfs_remove_link(&pst_adux->adux_dev_attr->kobj,
			  pst_adux->gest_input->name);
	sysfs_remove_link(&pst_adux->adux_dev_attr->kobj, "ADUX1020_attribute");
	device_destroy(pst_adux->adux_class, 0);
	class_destroy(pst_adux->adux_class);
}

/**
@brief This API used to create the sysfs
@param adux_dev Device Structure to the adux1020
@return int The status of sysfs created
*/
static int
adux1020_sysfs_create(struct device *adux_dev)
{
	int s32_SysfsRet = 0;
	ADUX1020_info(" %s\n", __func__);
	s32_SysfsRet = sysfs_create_group(&adux_dev->kobj,
					  &adux1020_attr_group);
	adux1020_sensor_link_create(adux_dev);
	return s32_SysfsRet;
}

/**
@brief This API used to remove the sysfs
@param adux_dev Device Structure to the adux1020 chip
@return void
*/
static void
adux1020_sysfs_remove(struct device *adux_dev)
{
	ADUX1020_info(" %s\n", __func__);
	sysfs_remove_group(&adux_dev->kobj, &adux1020_attr_group);
	adux1020_sensor_link_destroy(adux_dev);
}

/*SYSFS - END *****************************************************/
/**
@brief This API is used to read clock calibrated value from user
*       @param filename name of the file
*       @param pst_adux the ADUX device pointer
*       @return s32
*/
int adux1020_read_clock(s8 *filename, struct adux1020 *pst_adux)
{
	struct file *fd_adux1020 = NULL;
	int ret;
	int val;

	unsigned int checksum;

	u16 i = 0;
	u16 j;
	u16 Lpos;
	u16 data[5][3];

	u16 count = 0;

	loff_t pos = 0;
	char read_buff[128];
	char **temp_buf = NULL;
	char *token = NULL;
	char *data_buf = NULL;


	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	memset(read_buff, '\0', sizeof(read_buff));

	fd_adux1020 = filp_open(filename, O_RDONLY, 0666);
	if (IS_ERR(fd_adux1020)) {
		ADUX1020_dbg(" Unable to Open the file %ld\n",\
		PTR_ERR(fd_adux1020));
		set_fs(old_fs);
		ret = -1;
		goto err_filp_open;
	}
	do {
		ret = vfs_read(fd_adux1020,\
				read_buff,\
				sizeof(read_buff),\
				&pos);
		if ((ret == -EIO) | (ret <= 4)) {
			ret = 0;
			ADUX1020_info("No data reached EOF\n");
			if (count == 0)
				goto err_filp_nodata;
		} else {
			ADUX1020_info("DATA: %s\n", read_buff);
			temp_buf = (char **)&read_buff;
			while ((token = strsep((char **)&temp_buf, "\n"))) {
				if (!strcmp("ACAC", token))
					ADUX1020_info("Start of Fs\n");
				if (!strcmp("DCDC", token)) {
					ADUX1020_info("End of Fs\n");
					ret = 1;
					val = 0;
					checksum = 0;
					/*calculate checksum*/
					for (j = 0; j <= (i-2); j++)
						val = val + data[j][1];
					while (val != 0) {
						checksum = (val % 10) +\
							checksum;
						val = val / 10;
					}
					if (checksum == data[i-1][0]) {
						ADUX1020_dbg("CS M\n");
						ret = 0;
						for (j = 1;\
							j < (i-1);\
							j++) {
							ADUX1020_dbg(\
						"write 0x%x in reg 0x%x\n",\
						data[j][1], data[j][0]);
							adux1020_write(\
								pst_adux,\
								data[j][0],\
								data[j][1]);
						}
					} else {
						ADUX1020_dbg("Chksum Fail\n");
						ret = -1;
					}

					goto err_filp_nodata;
				}
				/*next line is data*/
				j = 0;
				Lpos = 0;
				data[i][0] = 0;
				data[i][1] = 0;
				while ((data_buf = \
					strsep((char **)&token, " "))) {
					if ((data_buf[Lpos] == '0') &&\
					(data_buf[Lpos + 1] == 'x' ||\
					data_buf[Lpos + 1] == 'X')) {
						if ( kstrtoul(\
						&data_buf[Lpos + 2], 16,\
						(long unsigned int *)&val))
							val  = 0;
					} else {
						if ( kstrtoul(\
					&data_buf[Lpos], 16/*10*/,\
					(long unsigned int *)&val))
							val  = 0;
					}
					data[i][j] = val;
					j++;
				}
					ADUX1020_info("%x %x\n",\
							data[i][0],\
							data[i][1]);
					i++;
			}
		}
	} while ((ret > 4) | (ret != -EIO));

	filp_close(fd_adux1020, NULL);
	set_fs(old_fs);
	return 1;
err_filp_nodata:
	filp_close(fd_adux1020, NULL);
	set_fs(old_fs);
	return ret;
err_filp_open:
	set_fs(old_fs);
	return ret;
}
/**
  This is a function thread used for clock calibration.
  @param data This is the device structure of the i2c client
  @return int
 */
int adux1020_clk_calib_thread(void *data)
{
	struct adux1020 *pst_adux = data;
	u32 prv_mode = 0;
	int ret = 1;
	set_current_state(TASK_INTERRUPTIBLE);
	while (!kthread_should_stop()) {
		if (pst_adux->clk_status == 1) {
			prv_mode = pst_adux->mode;
			adux1020_mode_switching(pst_adux, IDLE);
			set_current_state(TASK_RUNNING);
			pst_adux->clk_status = 0xF;
			ADUX1020_dbg("clock calibration started\n");
			ret = adux1020_read_clock(CLOCK_CALIB_CONFIG, pst_adux);
			if (ret == -1) {
				ADUX1020_dbg("Running Clock calibration\n");
				adux1020_clock_calibration(pst_adux);
			}
			if (pst_adux->clk_status == ABORT)
				ADUX1020_dbg("Clock calibration aborted!!\n");
			else
				ADUX1020_dbg("clock calibration done\n");
			pst_adux->clk_status = 0x0;
			adux1020_mode_switching(pst_adux, prv_mode);
		} else {
			schedule();
		}
		set_current_state(TASK_INTERRUPTIBLE);
	}
	return 1;
}
/*
  This function is  used for Device probe. All initialization routines are handled here
  This is the callback function from the adux1020_i2c_probe
  @param dev This is the device structure of the i2c client
  @param devid ID to ADUX1020 chip
  @param irq GPIO irq number
  @param pt_i2c_bus_ops pointer to the adux_bus_ops structure
  @return ADUX1020 Device structure on Sucess.On failure -ENOMEM, -EINVAL ,etc.,
 */
static struct adux1020 *
adux1020_probe(struct device *dev,
	       const struct i2c_device_id *devid,
	       unsigned irq, const struct adux_bus_ops *pt_i2c_bus_ops)
{
	struct adux1020 *pst_adux;
	struct adux_platform_data *plat_data = dev->platform_data;
	s32 error = 0;

	ADUX1020_dbg(" %s\n", __func__);
	if (!irq) {
		dev_err(dev, "no IRQ?\n");
		error = -EINVAL;
		goto err_out;
	}
#ifdef ENDIAN
	if (!endian) {
		dev_err(dev, "no endian?\n");
		error = -EINVAL;
		goto err_out;
	}
#endif

	pst_adux = kzalloc(sizeof(struct adux1020), GFP_KERNEL);

	if (IS_ERR(pst_adux)) {
		error = -ENOMEM;	/* out of memory */
		goto err_free_mem;
	}
	/* adux1020 structure initialization is done here */
	pst_adux->pdata = plat_data;
	adux1020_struct_init(dev, devid, irq, pst_adux, pt_i2c_bus_ops);

	/* Power on initilization */
	adux1020_power_init(pst_adux);
	if (pst_adux->chip_id == CHIP_ID_MASK) {
		ADUX1020_dbg("adux1020 i2c slave device not found!!!\n");
		error = -ENXIO;	/*No such device or address */
		goto err_free_mem;
	}

	/* General configuration */
	adux_filp_start_calib(GENERAL_CONFIG, pst_adux, CONFIG(GENERAL));
	adux1020_config_init(pst_adux, CONFIG(GENERAL));
	//adux1020_clock_calibration(pst_adux);

	adux1020_enter_idle(pst_adux);

	dev_set_drvdata(dev, pst_adux);

	mutex_init(&pst_adux->mutex);

	INIT_WORK(&pst_adux->work, adux1020_work_queue);

	pst_adux->clk_thread = kthread_create(&adux1020_clk_calib_thread,
				    pst_adux,
				    "adux1020_clk_calibration");

	if (!IS_ERR(pst_adux->clk_thread))
		ADUX1020_dbg("Clock calibration thread created!!\n");
	else
		goto err_free_mem;
	/* Interrupt Handle Request */
	error = request_threaded_irq(pst_adux->irq,
				     NULL,
				     adux1020_isr,
				     IRQF_TRIGGER_RISING,
				     dev_name(dev), pst_adux);
	if (error) {
		ADUX1020_dbg(" irq %d busy?\n", pst_adux->irq);
		goto err_free_thread;
	}
	disable_irq_nosync(pst_adux->irq);

	/* Allocation of input devices */
	pst_adux->gest_input = adux1020_inputdev_alloc(pst_adux,
						       dev,
						       ADUX1020_INPUT1_NAME);
	if (!pst_adux->gest_input) {
		error = -ENOMEM;
		goto err_free_irq;
	}
	pst_adux->prox1_input = adux1020_inputdev_alloc(pst_adux,
							dev,
							"ASUS Proximitysensor");
	if (!pst_adux->prox1_input) {
		error = -ENOMEM;
		goto err_free_gesture_dev;
	}

	pst_adux->prox2_input = adux1020_inputdev_alloc(pst_adux,
							dev,
							"ASUS Proximitysensor");
	if (!pst_adux->prox2_input) {
		error = -ENOMEM;
		goto err_free_proximity1_dev;
	}
	/* Registering input device */
	error = adux1020_gesture_inputdev_reg(pst_adux->gest_input);
	if (error < 0)
		goto err_free_proximity2_dev;
	error = adux1020_proximity_inputdev_reg(pst_adux->prox1_input);
	if (error < 0)
		goto err_unregister_gesture_dev;

	error = adux1020_proximity_inputdev_reg(pst_adux->prox2_input);
	if (error < 0)
		goto err_unregister_proximity1_dev;
	error = adux1020_sysfs_create(pst_adux->adux_dev);
	if (error < 0) {
		adux1020_sysfs_remove(pst_adux->adux_dev);
		goto err_unregister_proximity2_dev;
	}

	pst_adux->KeyMapCode = 1;	/* MENU KEYS */

	adux1020_keycodemapping(pst_adux);

	error = proximity_dev_register(&adux1020_proxmDev);
	if (error)
		printk("[adux1020] proxmdl create sysfile fail.\n");

	adux1020_data = pst_adux;

	enable_irq(pst_adux->irq);
	return pst_adux;

err_unregister_proximity2_dev:
	adux1020_proximity_inputdev_unreg(pst_adux->prox2_input);
err_unregister_proximity1_dev:
	adux1020_proximity_inputdev_unreg(pst_adux->prox1_input);
err_unregister_gesture_dev:
	adux1020_gesture_inputdev_unreg(pst_adux->gest_input);
err_free_proximity2_dev:
	input_free_device(pst_adux->prox2_input);
err_free_proximity1_dev:
	input_free_device(pst_adux->prox1_input);
err_free_gesture_dev:
	input_free_device(pst_adux->gest_input);
err_free_irq:
	free_irq(pst_adux->irq, pst_adux);
err_free_thread:
	kthread_stop(pst_adux->clk_thread);
err_free_mem:
	kfree(pst_adux->runtime_pdata);
	kfree(pst_adux);
err_out:
	return ERR_PTR(error);

}

/**
  Used to remove device.
  This function is used to remove ADUX1020 from the system by unreg ADUX1020
  @param pst_adux The chip Structure
  @return void Nothing returned
 */
static void
adux1020_remove(struct adux1020 *pst_adux)
{
	/*adux1020 remove called */
	ADUX1020_info(" %s\n", __func__);
	adux1020_enter_idle(pst_adux);
	adux1020_sysfs_remove(pst_adux->adux_dev);
	kthread_stop(pst_adux->clk_thread);
	free_irq(pst_adux->irq, pst_adux);
	adux1020_gesture_inputdev_unreg(pst_adux->gest_input);
	adux1020_proximity_inputdev_unreg(pst_adux->prox1_input);
	adux1020_proximity_inputdev_unreg(pst_adux->prox2_input);
	kfree(pst_adux->runtime_pdata);
	kfree(pst_adux);
}

/**	@see adux1020_i2c_suspend
 *	@brief This API is used to suspend the device
 *	@param  *client the i2c_client structure
 *	@param message the power management message variable
 *	@return static s32
 */
static s32
adux1020_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
	ADUX1020_info(" %s\n", __func__);
	return 0;
}

/**	static s32 adux1020_i2c_resume (struct i2c_client *client)
 *	@brief This API is used to resume the device from sleep mode
 *	@param  *client the i2c_client structure
 *	@return static s32
 */
static s32
adux1020_i2c_resume(struct i2c_client *client)
{
	ADUX1020_info(" %s\n", __func__);
	return 0;
}

/**     @fn static s32 adux1020_i2c_read(struct device  *adux_dev,
  u16 reg,
  u32 len,
  u16 *data,
  u32 endian)
 *      @brief This API is used to multi read
 *      @param *adux_dev store the pointer of device
 *      @param reg store the register value
 *      @param len store the length of word to read
 *      @param *data store the data pointer
 *      @param endian store the type of endian
 *      @return static s32
 */
static s32
adux1020_i2c_read(struct device *adux_dev,
		  u16 reg, u32 len, u16 *data, u32 endian)
{
	struct i2c_client *client = to_i2c_client(adux_dev);
	struct i2c_msg msg[2];
	u8 block_data[2];
	s32 ret, icnt;
	u8 retrycnt = 0;
	u8 *_reg = (u8 *)&reg;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;

	if (_reg[1] < ADUX_ADDRESS_RANGE) {
		msg[0].len = ADUX1020_REG_SIZE_1BYTE;
		block_data[0] = *_reg;
	} else {
		msg[0].len = ADUX1020_REG_SIZE_BYTES;
		switch (endian) {
		default:
		case ADUX1020_BIG_ENDIAN:
			reg = cpu_to_be16(reg);
			memcpy(block_data, &reg, DATA_SIZE);
			break;
		case ADUX1020_LITTLE_ENDIAN:
			reg = cpu_to_le16(reg);
			memcpy(block_data, &reg, DATA_SIZE);
			break;
		}
	}

	msg[0].buf = (s8 *)&block_data;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = len * ADUX1020_REG_SIZE_BYTES;
	msg[1].buf = (s8 *)data;

	do {
		ret = i2c_transfer(client->adapter, msg, MSG_EXECUTED);
		if (ret != MSG_EXECUTED)
			retrycnt++;
	} while ((ret != MSG_EXECUTED) && (retrycnt <= RETRY_CNT));

	if (ret != MSG_EXECUTED) {
		dev_err(&client->dev, "I2C read err (%d) reg: 0x%X len: %d\n",
			ret, reg, len);
		return -EIO;
	}
	switch (endian) {
	default:
	case ADUX1020_BIG_ENDIAN:
		for (icnt = 0; icnt < len; icnt++)
			data[icnt] = be16_to_cpu(data[icnt]);
		break;
	case ADUX1020_LITTLE_ENDIAN:
		for (icnt = 0; icnt < len; icnt++)
			data[icnt] = le16_to_cpu(data[icnt]);
		break;
	}
	return len;
}

/**
This function is used for  i2c write of ADUX1020 chip
@param adux_dev  pointer to driver device structure
@param reg  register to read
@param data store the 16bits data to write
@param endian little or big endian
 *      @return static s32 len read length
*/
static s32
adux1020_i2c_write(struct device *adux_dev, u16 reg, u16 data, u32 endian)
{
	struct i2c_client *client = to_i2c_client(adux_dev);
	s32 ret;
	u8 block_data[MAX_DATA_CNT];
	u8 retrycnt = 0;
	u8 *_reg = (u8 *)&reg;
	u16 value = 0;
	u32 size = 0;

	if (_reg[1] < ADUX_ADDRESS_RANGE) {
		size = ADUX1020_REG_SIZE_1BYTE;
		block_data[0] = *_reg;

	} else {
		size = ADUX1020_REG_SIZE_BYTES;
		switch (endian) {
		default:
		case ADUX1020_BIG_ENDIAN:
			reg = cpu_to_be16(reg);
			memcpy(block_data, &reg, DATA_SIZE);
			break;
		case ADUX1020_LITTLE_ENDIAN:
			reg = cpu_to_le16(reg);
			memcpy(block_data, &reg, DATA_SIZE);
			break;
		}
	}

	switch (endian) {
	default:
	case ADUX1020_BIG_ENDIAN:
		value = cpu_to_be16(data);
		memcpy(&block_data[size], &value, DATA_SIZE);
		value = 0;
		break;
	case ADUX1020_LITTLE_ENDIAN:
		value = cpu_to_le16(data);
		memcpy(&block_data[size], &value, DATA_SIZE);
		value = 0;
		break;
	}

	do {
		ret = i2c_master_send(client, (s8 *)block_data,
				      (size/*address */ +
				       ADUX1020_REG_SIZE_BYTES/*data */));
		if (ret < 0)
			retrycnt++;
	} while ((ret < 0) && (retrycnt <= RETRY_CNT));

	if (ret < 0) {
		dev_err(&client->dev,
			"I2C write error reg: 0x%x len: ox%x\n",
			reg, WORD_SIZE);
		return ret;
	}
	return ADUX1020_REG_SIZE_BYTES;

}

/**     @see struct ad7166_bus_ops
 *      @note  Bus Operation Table
 */
static const struct adux_bus_ops adux1020_i2c_bus_ops = {
	.read = adux1020_i2c_read,
	.write = adux1020_i2c_write,
};

/**
Probes the Device based on ID using i2c.
@param client The Client to be probed
@param id the Client id
@return s32 0 on success ,PTR_ERR(chip) on error
@see adux1020_i2c_read
*/
static s32 __devinit
adux1020_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	/*struct adux1020 *pst_adux = NULL; */
	void *pst_adux = NULL;
	s32 error = 0;
	int adux1020_irq_gpio = 0;

	ADUX1020_info(" %s\n", __func__);
	/* check whether i2c support smbus protocol as we need to
	   set i2c bus mode during start up */
	if (!i2c_check_functionality(client->adapter,\
			I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "SMBUS Word Data not Supported\n");
		return -EIO;	/* I/O error */
	}
	adux1020_irq_gpio = of_get_named_gpio_flags( 
    			client->dev.of_node, "adux1020,irq-gpio",0,NULL);
	client->irq = gpio_to_irq( adux1020_irq_gpio );

	printk("[cm32180]Reques EIRQ %d succesd on GPIO:%d\n",
						client->irq,
						adux1020_irq_gpio );

	pst_adux = (struct adux1020 *)adux1020_probe(&(client->dev),
						      id,
						      client->irq,
						      &adux1020_i2c_bus_ops);
	if (IS_ERR(pst_adux)) {
		error = -ENOMEM;	/* out of memory */
		return error;
	}

	i2c_set_clientdata(client, (struct adux1020 *)pst_adux);
	return 0;
}

/**
This function is used to remove the I2C client for its existance in the system
@param client The Client to be removed
@return static s32 0 on success ,PTR_ERR(chip) on error
*/
static s32 __devexit
adux1020_i2c_remove(struct i2c_client *client)
{
	struct adux1020 *pst_adux = i2c_get_clientdata(client);
	ADUX1020_info(" %s\n", __func__);
	adux1020_remove(pst_adux);
	i2c_set_clientdata(client, NULL);
	return 0;
}

/**
  This table tell which framework it supported
  @brief the name has to get matched to the board configuration file setup
 */
static struct i2c_device_id adux_id[] = {
	{"adux1020", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, adux_id);

static struct of_device_id adux1020_match_table[] = {
	{ .compatible = "ADI,adux1020",},
	{ },
};

/**
  i2c operation structure
 */
struct i2c_driver adux1020_i2c_driver = {
	.driver = {
		   .name = "adux1020",
		   .owner = THIS_MODULE,
		   .of_match_table = adux1020_match_table,
		   },
	.probe = adux1020_i2c_probe,
	.remove = adux1020_i2c_remove,
	.suspend = adux1020_i2c_suspend,
	.resume = adux1020_i2c_resume,
	.id_table = adux_id,
};

/**
  The adux1020 module start here.
  s32 adux1020_start( void)
  @brief this is used to initiate I2C driver
@return s32
 */
static s32 __init
adux1020_start(void)
{
	s32 ret = 0;

	if ( g_ASUS_hwID == A86_SR3 )	{
		if ( !gpio_get_value(PROXIMITY_PWR_EN) )	{
			/* configure Phone Light/Proximity sensor power_enable gpio */
			ret = gpio_request(PROXIMITY_PWR_EN, "proxm_pwr_en");
			if (ret)
				pr_err("%s: unable to request gpio %d (proxm_pwr_en)\n",__func__, PROXIMITY_PWR_EN);

			ret = gpio_direction_output(PROXIMITY_PWR_EN, 1);
			if (ret < 0)
				pr_err("%s: unable to set the direction of gpio %d\n",__func__, PROXIMITY_PWR_EN);

			/* HW Power on Light/Proximity sensor  */
			gpio_set_value(PROXIMITY_PWR_EN, 1);

			printk("[Light/Proximity sensor][board] Power on Light/Proximity sensor\n");
		}
		
		/* configure Phone Lightsensor interrupt gpio */
		ret = gpio_request(ADUX1020_PROXIMITY_INT, "adux1020-irq");
		if (ret)
			pr_err("%s: unable to request gpio %d (adux1020-irq)\n",__func__, ADUX1020_PROXIMITY_INT);

		ret = gpio_direction_input(ADUX1020_PROXIMITY_INT);
		if (ret < 0)
			pr_err("%s: unable to set the direction of gpio %d\n",__func__, ADUX1020_PROXIMITY_INT);
		
		ADUX1020_info(" %s\n", __func__);
		ret = i2c_add_driver(&adux1020_i2c_driver);
		if (!ret) {
			/*print i2c register successfully */
			ADUX1020_info(" i2c register successfully\n");
		} else {
			/*print i2c register failed */
			ADUX1020_dbg(" i2c register failed\n");
		}
	}
	return ret;
}

/**
  The adux1020 module stop here.
  void adux1020_stop( void)
  @brief this is used to de register and exit I2C driver
@return void
 */
static void __exit
adux1020_stop(void)
{
	ADUX1020_dbg(" %s\n", __func__);
	i2c_del_driver(&adux1020_i2c_driver);
}

module_init(adux1020_start);
module_exit(adux1020_stop);
MODULE_DESCRIPTION("ANALOG DEVICES - ADUX1020");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("");
