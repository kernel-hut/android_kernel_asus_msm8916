/* Copyright (c) 2013-2015 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "SMB:%s: " fmt, __func__

#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/math64.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/completion.h>
#include <linux/pm_wakeup.h>

//asus report capacity frank ++++
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include "asus_battery.h"
#include <linux/wakelock.h>
#include <linux/pm_runtime.h>
#include <linux/alarmtimer.h>
//asus report capacity frank ----

#include <linux/reboot.h>

#define _SMB1360_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB1360_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB1360_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
				(RIGHT_BIT_POS))

/* Charger Registers */
#define CFG_BATT_CHG_REG		0x00
#define CHG_ITERM_MASK			SMB1360_MASK(2, 0)
#define CHG_ITERM_25MA			0x0
#define CHG_ITERM_200MA			0x7
#define RECHG_MV_MASK			SMB1360_MASK(6, 5)
#define RECHG_MV_SHIFT			5
#define OTG_CURRENT_MASK		SMB1360_MASK(4, 3)
#define OTG_CURRENT_SHIFT		3

#define CFG_BATT_CHG_ICL_REG		0x05
#define AC_INPUT_ICL_PIN_BIT		BIT(7)
#define AC_INPUT_PIN_HIGH_BIT		BIT(6)
#define RESET_STATE_USB_500		BIT(5)
#define INPUT_CURR_LIM_MASK		SMB1360_MASK(3, 0)
#define INPUT_CURR_LIM_300MA		0x0

#define CFG_GLITCH_FLT_REG		0x06
#define AICL_ENABLED_BIT		BIT(0)
#define INPUT_UV_GLITCH_FLT_20MS_BIT	BIT(7)

#define CFG_CHG_MISC_REG		0x7
#define CHG_EN_BY_PIN_BIT		BIT(7)
#define CHG_EN_ACTIVE_LOW_BIT		BIT(6)
#define PRE_TO_FAST_REQ_CMD_BIT		BIT(5)
#define CFG_BAT_OV_ENDS_CHG_CYC		BIT(4)
#define CHG_CURR_TERM_DIS_BIT		BIT(3)
#define CFG_AUTO_RECHG_DIS_BIT		BIT(2)
#define CFG_CHG_INHIBIT_EN_BIT		BIT(0)

#define CFG_CHG_FUNC_CTRL_REG		0x08
#define CHG_RECHG_THRESH_FG_SRC_BIT	BIT(1)

#define CFG_STAT_CTRL_REG		0x09
#define CHG_STAT_IRQ_ONLY_BIT		BIT(4)
#define CHG_TEMP_CHG_ERR_BLINK_BIT	BIT(3)
#define CHG_STAT_ACTIVE_HIGH_BIT	BIT(1)
#define CHG_STAT_DISABLE_BIT		BIT(0)

#define CFG_SFY_TIMER_CTRL_REG		0x0A
#define SAFETY_TIME_DISABLE_BIT		BIT(5)
#define SAFETY_TIME_MINUTES_SHIFT	2
#define SAFETY_TIME_MINUTES_MASK	SMB1360_MASK(3, 2)

#define CFG_BATT_MISSING_REG		0x0D
#define BATT_MISSING_SRC_THERM_BIT	BIT(1)

#define CFG_FG_BATT_CTRL_REG		0x0E
#define CFG_FG_OTP_BACK_UP_ENABLE	BIT(7)
#define BATT_ID_ENABLED_BIT		BIT(5)
#define CHG_BATT_ID_FAIL		BIT(4)
#define BATT_ID_FAIL_SELECT_PROFILE	BIT(3)
#define BATT_PROFILE_SELECT_MASK	SMB1360_MASK(3, 0)
#define BATT_PROFILEA_MASK		0x0
#define BATT_PROFILEB_MASK		0xF

#define IRQ_CFG_REG			0x0F
#define IRQ_BAT_HOT_COLD_HARD_BIT	BIT(7)
#define IRQ_BAT_HOT_COLD_SOFT_BIT	BIT(6)
#define IRQ_DCIN_UV_BIT			BIT(2)
#define IRQ_AICL_DONE_BIT		BIT(1)
#define IRQ_INTERNAL_TEMPERATURE_BIT	BIT(0)

#define IRQ2_CFG_REG			0x10
#define IRQ2_SAFETY_TIMER_BIT		BIT(7)
#define IRQ2_CHG_ERR_BIT		BIT(6)
#define IRQ2_CHG_PHASE_CHANGE_BIT	BIT(4)
#define IRQ2_POWER_OK_BIT		BIT(2)
#define IRQ2_BATT_MISSING_BIT		BIT(1)
#define IRQ2_VBAT_LOW_BIT		BIT(0)

#define IRQ3_CFG_REG			0x11
#define IRQ3_FG_ACCESS_OK_BIT		BIT(6)
#define IRQ3_SOC_CHANGE_BIT		BIT(4)
#define IRQ3_SOC_MIN_BIT		BIT(3)
#define IRQ3_SOC_MAX_BIT		BIT(2)
#define IRQ3_SOC_EMPTY_BIT		BIT(1)
#define IRQ3_SOC_FULL_BIT		BIT(0)

#define CHG_CURRENT_REG			0x13
#define FASTCHG_CURR_MASK		SMB1360_MASK(4, 2)
#define FASTCHG_CURR_SHIFT		2

#define CHG_CMP_CFG			0x14
#define JEITA_COMP_CURR_MASK		SMB1360_MASK(3, 0)
#define JEITA_COMP_EN_MASK		SMB1360_MASK(7, 4)
#define JEITA_COMP_EN_SHIFT		4
#define JEITA_COMP_EN_BIT		SMB1360_MASK(7, 4)
#define BATT_CHG_FLT_VTG_REG		0x15
#define VFLOAT_MASK			SMB1360_MASK(6, 0)
#define CFG_FVC_REG			0x16
#define FLT_VTG_COMP_MASK		SMB1360_MASK(6, 0)

#define SHDN_CTRL_REG			0x1A
#define SHDN_CMD_USE_BIT		BIT(1)
#define SHDN_CMD_POLARITY_BIT		BIT(2)

#define CURRENT_GAIN_LSB_REG		0x1D
#define CURRENT_GAIN_MSB_REG		0x1E

/* Command Registers */
#define CMD_I2C_REG			0x40
#define ALLOW_VOLATILE_BIT		BIT(6)
#define FG_ACCESS_ENABLED_BIT		BIT(5)
#define FG_RESET_BIT			BIT(4)
#define CYCLE_STRETCH_CLEAR_BIT		BIT(3)

#define CMD_IL_REG			0x41
#define USB_CTRL_MASK			SMB1360_MASK(1 , 0)
#define USB_100_BIT			0x01
#define USB_500_BIT			0x00
#define USB_AC_BIT			0x02
#define SHDN_CMD_BIT			BIT(7)

#define CMD_CHG_REG			0x42
#define CMD_CHG_EN			BIT(1)
#define CMD_OTG_EN_BIT			BIT(0)

/* Status Registers */
#define STATUS_1_REG			0x48
#define AICL_CURRENT_STATUS_MASK	SMB1360_MASK(6, 0)
#define AICL_LIMIT_1500MA		0xF
#define AICL_LIMIT_1000MA		0xA
#define STATUS_3_REG			0x4B
#define CHG_HOLD_OFF_BIT		BIT(3)
#define CHG_TYPE_MASK			SMB1360_MASK(2, 1)
#define CHG_TYPE_SHIFT			1
#define BATT_NOT_CHG_VAL		0x0
#define BATT_PRE_CHG_VAL		0x1
#define BATT_FAST_CHG_VAL		0x2
#define BATT_TAPER_CHG_VAL		0x3
#define CHG_EN_BIT			BIT(0)

#define STATUS_4_REG			0x4C
#define CYCLE_STRETCH_ACTIVE_BIT	BIT(5)

#define REVISION_CTRL_REG		0x4F
#define DEVICE_REV_MASK			SMB1360_MASK(3, 0)

/* IRQ Status Registers */
#define IRQ_A_REG			0x50
#define IRQ_A_HOT_HARD_BIT		BIT(6)
#define IRQ_A_COLD_HARD_BIT		BIT(4)
#define IRQ_A_HOT_SOFT_BIT		BIT(2)
#define IRQ_A_COLD_SOFT_BIT		BIT(0)

#define IRQ_B_REG			0x51
#define IRQ_B_BATT_TERMINAL_BIT		BIT(6)
#define IRQ_B_BATT_MISSING_BIT		BIT(4)

#define IRQ_C_REG			0x52
#define IRQ_C_CHG_TERM			BIT(0)

#define IRQ_D_REG			0x53
#define IRQ_E_REG			0x54
#define IRQ_E_USBIN_UV_BIT		BIT(0)

#define IRQ_F_REG			0x55

#define IRQ_G_REG			0x56

#define IRQ_H_REG			0x57
#define IRQ_I_REG			0x58
#define FG_ACCESS_ALLOWED_BIT		BIT(0)
#define BATT_ID_RESULT_BIT		SMB1360_MASK(6, 4)
#define BATT_ID_SHIFT			4

/* FG registers - IRQ config register */
#define SOC_MAX_REG			0x24
#define SOC_MIN_REG			0x25
#define VTG_EMPTY_REG			0x26
#define SOC_DELTA_REG			0x28
#define JEITA_SOFT_COLD_REG		0x29
#define JEITA_SOFT_HOT_REG		0x2A
#define VTG_MIN_REG			0x2B

/* FG SHADOW registers */
#define SHDW_FG_ESR_ACTUAL		0x20
#define SHDW_FG_BATT_STATUS		0x60
#define BATTERY_PROFILE_BIT		BIT(0)

#define SHDW_FG_MSYS_SOC		0x61
#define SHDW_FG_CAPACITY		0x62
#define SHDW_FG_VTG_NOW			0x69
#define SHDW_FG_CURR_NOW		0x6B
#define SHDW_FG_BATT_TEMP		0x6D

#define VOLTAGE_PREDICTED_REG		0x80
#define CC_TO_SOC_COEFF			0xBA
#define NOMINAL_CAPACITY_REG		0xBC
#define ACTUAL_CAPACITY_REG		0xBE
#define FG_AUTO_RECHARGE_SOC		0xD2
#define FG_SYS_CUTOFF_V_REG		0xD3
#define FG_CC_TO_CV_V_REG		0xD5
#define FG_ITERM_REG			0xD9
#define FG_THERM_C1_COEFF_REG		0xDB
#define FG_IBATT_STANDBY_REG		0xCF

#define FG_I2C_CFG_MASK			SMB1360_MASK(2, 1)
#define FG_CFG_I2C_ADDR			0x2
#define FG_PROFILE_A_ADDR		0x4
#define FG_PROFILE_B_ADDR		0x6

/* Constants */
#define CURRENT_100_MA			100
#define CURRENT_500_MA			500
#define MAX_8_BITS			255
#define JEITA_WORK_MS			3000

#define FG_RESET_THRESHOLD_MV		15
#define SMB1360_REV_1			0x01

#define SMB1360_POWERON_DELAY_MS	2000
#define SMB1360_FG_RESET_DELAY_MS	1500

#if defined(CONFIG_ASUS_ZC550KL8939_PROJECT)
#define THERMAL_DCP_LEVEL	5
#define THERMAL_SDP_LEVEL	0
enum usb_mode{
	USB_DCP_MODE,
	USB_SDP_MODE
};
#endif

#if defined(CONFIG_ASUS_ZC550KL8939_PROJECT)
#define ADAPTOR_10W_FACTORY_FILE		"/factory/10W"
#define COUNTRY_CODE_FACTORY_FILE	"/factory/COUNTRY"

struct delayed_work adaptor_10W_check_work;
struct wake_lock adaptor_10W_check_wake_lock;

#endif
bool runin_switch_control_off;
//asus report capacity frank ++++
int64_t battID =-1;
static struct smb1360_chip *smb1360_dev;
static struct alarm bat_alarm;
struct wake_lock bat_alarm_wake_lock;
struct delayed_work SetBatRTCWorker;
struct delayed_work battery_poll_data_work;

static DEFINE_SPINLOCK(bat_alarm_slock);
DEFINE_MUTEX(g_usb_state_lock);
DEFINE_MUTEX(g_charging_toggle_lock);
static bool g_charging_toggle_status = true;
static int g_usb_state = CABLE_OUT;
static struct timespec g_last_print_time;
#define REPORT_CAPACITY_POLLING_TIME (180)
static struct timespec plugin_time;
static struct timespec plugout_time;
//int usbin_check_count=0;
bool usbin_voltage_poor=false;
struct battery_info_reply {
u32 capacity;
int rawsoc;
int Rsoc;
u32 voltage_now;
int current_now;
char *temperature_negative_sign;
int temperature;
int temperature10;
u32 cable_status;
int status;
int fcc;
bool charging_toggle;
int AICL_status;
};
struct battery_info_reply batt_info;

#if defined(CONFIG_ASUS_ZC550KL8939_PROJECT)
extern void asus_otg_chargetype_set(bool sdp);
extern int g_CHG_mode;
static int bank_hot_lvl = 0;
extern uint32_t get_zc550kl_pcb_rev(void);
#endif

bool smb1360_is_charging(int usb_state);
bool smb1360_probe_success= false;
static enum alarmtimer_restart batAlarm_handler(struct alarm *alarm, ktime_t now)
{
	printk("[BAT][alarm]batAlarm triggered\n");
	return ALARMTIMER_NORESTART;
}
//asus report capacity frank ----
int asus_battery_update_status(void);
//asus bsp frank add smb1360_charging_toggle and runin_switch +++
int smb1360_charging_toggle(charging_toggle_level_t level, bool on, bool status_flag);
//asus bsp frank add smb1360_charging_toggle and runin_switch ---

int64_t check_battery_id(void);

#define	Battery_435V	0
#define	Battery_440V	1
#define	Battery_438V	2

//ASUS_BSP frank for ChargerDriver stress test +++
#ifdef CONFIG_I2C_STRESS_TEST
#include <linux/i2c_testcase.h>
#define I2C_TEST_SMB1360_FAIL (-1)
#endif
//ASUS_BSP frank for ChargerDriver stress test ---

struct wake_lock usbin_wake_lock;

int ac_current =1000;

extern char bl_cmd[2];
extern void set_tcon_cmd(char*, short);

u8 battery_type;
enum {
	WRKRND_FG_CONFIG_FAIL = BIT(0),
	WRKRND_BATT_DET_FAIL = BIT(1),
	WRKRND_USB100_FAIL = BIT(2),
	WRKRND_HARD_JEITA = BIT(3),
};

enum {
	USER	= BIT(0),
};

enum {
	PARALLEL_USER = BIT(0),
	PARALLEL_CURRENT = BIT(1),
	PARALLEL_JEITA_SOFT = BIT(2),
	PARALLEL_JEITA_HARD = BIT(3),
	PARALLEL_EOC = BIT(4),
};

enum fg_i2c_access_type {
	FG_ACCESS_CFG = 0x1,
	FG_ACCESS_PROFILE_A = 0x2,
	FG_ACCESS_PROFILE_B = 0x3
};

enum {
	BATTERY_PROFILE_A,
	BATTERY_PROFILE_B,
	BATTERY_PROFILE_MAX,
};

static int otg_curr_ma[] = {350, 550, 950, 1500};

struct otp_backup_pool {
	u8 reg_start;
	u8 reg_end;
	u8 start_now;
	u16 alg_bitmap;
	bool initialized;
	struct mutex lock;
};

enum otp_backup_alg {
	OTP_BACKUP_NOT_USE = 0,
	OTP_BACKUP_FG_USE,
	OTP_BACKUP_PROF_A_USE,
	OTP_BACKUP_PROF_B_USE,
};

struct smb1360_otg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

enum wakeup_src {
	WAKEUP_SRC_FG_ACCESS = 0,
	WAKEUP_SRC_JEITA_SOFT,
	WAKEUP_SRC_PARALLEL,
	WAKEUP_SRC_MIN_SOC,
	WAKEUP_SRC_EMPTY_SOC,
	WAKEUP_SRC_JEITA_HYSTERSIS,
	WAKEUP_SRC_MAX,
};
#define WAKEUP_SRC_MASK (~(~0 << WAKEUP_SRC_MAX))

struct smb1360_wakeup_source {
	struct wakeup_source source;
	unsigned long enabled_bitmap;
	spinlock_t ws_lock;
};

struct smb1360_chip {
	struct i2c_client		*client;
	struct device			*dev;
	u8				revision;
	u8				soft_hot_rt_stat;
	u8				soft_cold_rt_stat;
	struct delayed_work		jeita_work;
	struct delayed_work		delayed_init_work;
	unsigned short			default_i2c_addr;
	unsigned short			fg_i2c_addr;
	bool				pulsed_irq;
	struct completion		fg_mem_access_granted;

	/* wakeup source */
	struct smb1360_wakeup_source	smb1360_ws;

	/* configuration data - charger */
	int				fake_battery_soc;
	bool				batt_id_disabled;
	bool				charging_disabled;
	bool				recharge_disabled;
	bool				chg_inhibit_disabled;
	bool				iterm_disabled;
	bool				shdn_after_pwroff;
	bool				config_hard_thresholds;
	bool				soft_jeita_supported;
	bool				ov_ends_chg_cycle_disabled;
	int				iterm_ma;
	int				vfloat_mv;
	int				safety_time;
	int				resume_delta_mv;
	u32				default_batt_profile;
	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;
	int				otg_batt_curr_limit;
	bool				min_icl_usb100;
	int				cold_bat_decidegc;
	int				hot_bat_decidegc;
	int				cool_bat_decidegc;
	int				warm_bat_decidegc;
	int				cool_bat_mv;
	int				warm_bat_mv;
	int				cool_bat_ma;
	int				warm_bat_ma;
	int				soft_cold_thresh;
	int				soft_hot_thresh;

	/* parallel-chg params */
	int				fastchg_current;
	int				parallel_chg_disable_status;
	int				max_parallel_chg_current;
	bool				parallel_charging;

	/* configuration data - fg */
	int				soc_max;
	int				soc_min;
	int				delta_soc;
	int				voltage_min_mv;
	int				voltage_empty_mv;
	int				batt_capacity_mah;
	int				cc_soc_coeff;
	int				v_cutoff_mv;
	int				fg_iterm_ma;
	int				fg_ibatt_standby_ma;
	int				fg_thermistor_c1_coeff;
	int				fg_cc_to_cv_mv;
	int				fg_auto_recharge_soc;
	bool				empty_soc_disabled;
	int				fg_reset_threshold_mv;
	bool				fg_reset_at_pon;
	bool				rsense_10mohm;
	bool				otg_fet_present;
	bool				fet_gain_enabled;
	bool				otp_rslow_config;
	int				otg_fet_enable_gpio;
	bool				otp_jeita_hard_config;

	/* status tracking */
	int				voltage_now;
	int				current_now;
	int				resistance_now;
	int				temp_now;
	int				soc_now;
	int				fcc_mah;
	bool				usb_present;
	bool				batt_present;
	bool				batt_hot;
	bool				batt_cold;
	bool				batt_warm;
	bool				batt_cool;
	bool				batt_full;
	bool				resume_completed;
	bool				irq_waiting;
	bool				irq_disabled;
	bool				empty_soc;
	bool				awake_min_soc;
	int				workaround_flags;
	u8				irq_cfg_mask[3];
	int				usb_psy_ma;
	int				charging_disabled_status;
	u32				connected_rid;
	u32				profile_rid[BATTERY_PROFILE_MAX];

	u32				peek_poke_address;
	u32				fg_access_type;
	u32				fg_peek_poke_address;
	int				skip_writes;
	int				skip_reads;
	struct dentry			*debug_root;

	struct qpnp_vadc_chip		*vadc_dev;
	struct power_supply		*parallel_psy;
	struct power_supply		*usb_psy;
	struct power_supply		batt_psy;
	struct smb1360_otg_regulator	otg_vreg;
	struct mutex			irq_complete;
	struct mutex			charging_disable_lock;
	struct mutex			current_change_lock;
	struct mutex			read_write_lock;
	struct mutex			parallel_chg_lock;
	struct work_struct		parallel_work;
	struct mutex			otp_gain_lock;
	struct mutex			fg_access_request_lock;
	struct otp_backup_pool		otp_backup;
	u8				current_gain_otp_reg;
	bool				otp_hard_jeita_config;
	int				otp_cold_bat_decidegc;
	int				otp_hot_bat_decidegc;
	u8				hard_jeita_otp_reg;
	struct work_struct		jeita_hysteresis_work;
	int				cold_hysteresis;
	int				hot_hysteresis;

	int otg_enable_gpio;
	int ovp_enable_gpio;
	int boost_enable_gpio;
	uint32_t HW_ID;
};

static int chg_time[] = {
	192,
	384,
	768,
	1536,
};

static int input_current_limit[] = {
	300, 400, 450, 500, 600, 700, 800, 850, 900,
	950, 1000, 1100, 1200, 1300, 1400, 1500,
};

static int fastchg_current[] = {
	450, 600, 750, 900, 1050, 1200, 1350, 1500,
};

static void smb1360_stay_awake(struct smb1360_wakeup_source *source,
	enum wakeup_src wk_src)
{
	unsigned long flags;

	spin_lock_irqsave(&source->ws_lock, flags);

	if (!__test_and_set_bit(wk_src, &source->enabled_bitmap)) {
		__pm_stay_awake(&source->source);
		pr_debug("enabled source %s, wakeup_src %d\n",
			source->source.name, wk_src);
	}
	spin_unlock_irqrestore(&source->ws_lock, flags);
}

static void smb1360_relax(struct smb1360_wakeup_source *source,
	enum wakeup_src wk_src)
{
	unsigned long flags;

	spin_lock_irqsave(&source->ws_lock, flags);
	if (__test_and_clear_bit(wk_src, &source->enabled_bitmap) &&
		!(source->enabled_bitmap & WAKEUP_SRC_MASK)) {
		__pm_relax(&source->source);
		pr_debug("disabled source %s\n", source->source.name);
	}
	spin_unlock_irqrestore(&source->ws_lock, flags);

	pr_debug("relax source %s, wakeup_src %d\n",
		source->source.name, wk_src);
}

static void smb1360_wakeup_src_init(struct smb1360_chip *chip)
{
	spin_lock_init(&chip->smb1360_ws.ws_lock);
	wakeup_source_init(&chip->smb1360_ws.source, "smb1360");
}

extern void focal_usb_detection(bool plugin);		//ASUS BSP Freeman : notify touch cable in +++

static int is_between(int value, int left, int right)
{
	if (left >= right && left >= value && value >= right)
		return 1;
	if (left <= right && left <= value && value <= right)
		return 1;

	return 0;
}

static int bound(int val, int min, int max)
{
	if (val < min)
		return min;
	if (val > max)
		return max;

	return val;
}

static int __smb1360_read(struct smb1360_chip *chip, int reg,
				u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}
	pr_debug("Reading 0x%02x=0x%02x\n", reg, *val);

	return 0;
}

static int __smb1360_write(struct smb1360_chip *chip, int reg,
						u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	pr_debug("Writing 0x%02x=0x%02x\n", reg, val);
	return 0;
}

static int smb1360_read(struct smb1360_chip *chip, int reg,
				u8 *val)
{
	int rc;

	if (chip->skip_reads) {
		*val = 0;
		return 0;
	}
	mutex_lock(&chip->read_write_lock);
	rc = __smb1360_read(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb1360_write(struct smb1360_chip *chip, int reg,
						u8 val)
{
	int rc;

	if (chip->skip_writes)
		return 0;

	mutex_lock(&chip->read_write_lock);
	rc = __smb1360_write(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb1360_fg_read(struct smb1360_chip *chip, int reg,
				u8 *val)
{
	int rc;

	if (chip->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&chip->read_write_lock);
	chip->client->addr = chip->fg_i2c_addr;
	rc = __smb1360_read(chip, reg, val);
	chip->client->addr = chip->default_i2c_addr;
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb1360_fg_write(struct smb1360_chip *chip, int reg,
						u8 val)
{
	int rc;

	if (chip->skip_writes)
		return 0;

	mutex_lock(&chip->read_write_lock);
	chip->client->addr = chip->fg_i2c_addr;
	rc = __smb1360_write(chip, reg, val);
	chip->client->addr = chip->default_i2c_addr;
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb1360_read_bytes(struct smb1360_chip *chip, int reg,
						u8 *val, u8 bytes)
{
	s32 rc;

	if (chip->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&chip->read_write_lock);
	rc = i2c_smbus_read_i2c_block_data(chip->client, reg, bytes, val);
	if (rc < 0)
		dev_err(chip->dev,
			"i2c read fail: can't read %d bytes from %02x: %d\n",
							bytes, reg, rc);
	mutex_unlock(&chip->read_write_lock);

	return (rc < 0) ? rc : 0;
}

static int smb1360_write_bytes(struct smb1360_chip *chip, int reg,
						u8 *val, u8 bytes)
{
	s32 rc;

	if (chip->skip_writes) {
		*val = 0;
		return 0;
	}

	mutex_lock(&chip->read_write_lock);
	rc = i2c_smbus_write_i2c_block_data(chip->client, reg, bytes, val);
	if (rc < 0)
		dev_err(chip->dev,
			"i2c write fail: can't read %d bytes from %02x: %d\n",
							bytes, reg, rc);
	mutex_unlock(&chip->read_write_lock);

	return (rc < 0) ? rc : 0;
}

static int smb1360_masked_write(struct smb1360_chip *chip, int reg,
						u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	if (chip->skip_writes || chip->skip_reads)
		return 0;

	mutex_lock(&chip->read_write_lock);
	rc = __smb1360_read(chip, reg, &temp);
	if (rc < 0) {
		dev_err(chip->dev, "read failed: reg=%03X, rc=%d\n", reg, rc);
		goto out;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = __smb1360_write(chip, reg, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"write failed: reg=%03X, rc=%d\n", reg, rc);
	}
out:
	mutex_unlock(&chip->read_write_lock);
	return rc;
}

static int smb1360_select_fg_i2c_address(struct smb1360_chip *chip)
{
	unsigned short addr = chip->default_i2c_addr << 0x1;

	switch (chip->fg_access_type) {
	case FG_ACCESS_CFG:
		addr = (addr & ~FG_I2C_CFG_MASK) | FG_CFG_I2C_ADDR;
		break;
	case FG_ACCESS_PROFILE_A:
		addr = (addr & ~FG_I2C_CFG_MASK) | FG_PROFILE_A_ADDR;
		break;
	case FG_ACCESS_PROFILE_B:
		addr = (addr & ~FG_I2C_CFG_MASK) | FG_PROFILE_B_ADDR;
		break;
	default:
		pr_err("Invalid FG access type=%d\n", chip->fg_access_type);
		return -EINVAL;
	}

	chip->fg_i2c_addr = addr >> 0x1;
	pr_debug("FG_access_type=%d fg_i2c_addr=%x\n", chip->fg_access_type,
							chip->fg_i2c_addr);

	return 0;
}

#define EXPONENT_MASK		0xF800
#define MANTISSA_MASK		0x3FF
#define SIGN_MASK		0x400
#define EXPONENT_SHIFT		11
#define SIGN_SHIFT		10
#define MICRO_UNIT		1000000ULL
static int64_t float_decode(u16 reg)
{
	int64_t final_val, exponent_val, mantissa_val;
	int exponent, mantissa, n;
	bool sign;

	exponent = (reg & EXPONENT_MASK) >> EXPONENT_SHIFT;
	mantissa = (reg & MANTISSA_MASK);
	sign = !!(reg & SIGN_MASK);

	pr_debug("exponent=%d mantissa=%d sign=%d\n", exponent, mantissa, sign);

	mantissa_val = mantissa * MICRO_UNIT;

	n = exponent - 15;
	if (n < 0)
		exponent_val = MICRO_UNIT >> -n;
	else
		exponent_val = MICRO_UNIT << n;

	n = n - 10;
	if (n < 0)
		mantissa_val >>= -n;
	else
		mantissa_val <<= n;

	final_val = exponent_val + mantissa_val;

	if (sign)
		final_val *= -1;

	return final_val;
}

#define MAX_MANTISSA    (1023 * 1000000ULL)
unsigned int float_encode(int64_t float_val)
{
	int exponent = 0, sign = 0;
	unsigned int final_val = 0;

	if (float_val == 0)
		return 0;

	if (float_val < 0) {
		sign = 1;
		float_val = -float_val;
	}

	/* Reduce large mantissa until it fits into 10 bit */
	while (float_val >= MAX_MANTISSA) {
		exponent++;
		float_val >>= 1;
	}

	/* Increase small mantissa to improve precision */
	while (float_val < MAX_MANTISSA && exponent > -25) {
		exponent--;
		float_val <<= 1;
	}

	exponent = exponent + 25;

	/* Convert mantissa from micro-units to units */
	float_val = div_s64((float_val + MICRO_UNIT), (int)MICRO_UNIT);

	if (float_val == 1024) {
		exponent--;
		float_val <<= 1;
	}

	float_val -= 1024;

	/* Ensure that resulting number is within range */
	if (float_val > MANTISSA_MASK)
		float_val = MANTISSA_MASK;

	/* Convert to 5 bit exponent, 11 bit mantissa */
	final_val = (float_val & MANTISSA_MASK) | (sign << SIGN_SHIFT) |
		((exponent << EXPONENT_SHIFT) & EXPONENT_MASK);

	return final_val;
}

/* FG reset could only be done after FG access being granted */
static int smb1360_force_fg_reset(struct smb1360_chip *chip)
{
	int rc;

	rc = smb1360_masked_write(chip, CMD_I2C_REG, FG_RESET_BIT,
						FG_RESET_BIT);
	if (rc) {
		pr_err("Couldn't reset FG rc=%d\n", rc);
		return rc;
	}

	msleep(SMB1360_FG_RESET_DELAY_MS);

	rc = smb1360_masked_write(chip, CMD_I2C_REG, FG_RESET_BIT, 0);
	if (rc)
		pr_err("Couldn't un-reset FG rc=%d\n", rc);

	return rc;
}

/*
 * Requesting FG access relys on the FG_ACCESS_ALLOWED IRQ.
 * This function can only be called after interrupt handler
 * being installed successfully.
 */
#define SMB1360_FG_ACCESS_TIMEOUT_MS	5000
#define SMB1360_FG_ACCESS_RETRY_COUNT	3
static int smb1360_enable_fg_access(struct smb1360_chip *chip)
{
	int rc = 0;
	u8 reg, retry = SMB1360_FG_ACCESS_RETRY_COUNT;

	pr_debug("request FG memory access\n");
	/*
	 * read the ACCESS_ALLOW status bit firstly to
	 * check if the access was granted before
	 */
	mutex_lock(&chip->fg_access_request_lock);
	smb1360_stay_awake(&chip->smb1360_ws, WAKEUP_SRC_FG_ACCESS);
	rc = smb1360_read(chip, IRQ_I_REG, &reg);
	if (rc) {
		pr_err("Couldn't read IRQ_I_REG, rc=%d\n", rc);
		goto bail_i2c;
	} else if (reg & FG_ACCESS_ALLOWED_BIT) {
		pr_debug("FG access was granted\n");
		goto bail_i2c;
	}

	/* request FG access */
	rc = smb1360_masked_write(chip, CMD_I2C_REG, FG_ACCESS_ENABLED_BIT,
							FG_ACCESS_ENABLED_BIT);
	if (rc) {
		pr_err("Couldn't enable FG access rc=%d\n", rc);
		goto bail_i2c;
	}

	while (retry--) {
		rc = wait_for_completion_interruptible_timeout(
			&chip->fg_mem_access_granted,
			msecs_to_jiffies(SMB1360_FG_ACCESS_TIMEOUT_MS));
		if (rc <= 0)
			pr_debug("FG access timeout, retry: %d\n", retry);
		else
			break;
	}
	if (rc == 0) /* timed out */
		rc = -ETIMEDOUT;
	else if (rc > 0) /* completed */
		rc = 0;

	/* Clear the FG access bit if request failed */
	if (rc < 0) {
		rc = smb1360_masked_write(chip, CMD_I2C_REG,
				FG_ACCESS_ENABLED_BIT, 0);
		if (rc)
			pr_err("Couldn't disable FG access rc=%d\n", rc);
	}

bail_i2c:
	smb1360_relax(&chip->smb1360_ws, WAKEUP_SRC_FG_ACCESS);
	mutex_unlock(&chip->fg_access_request_lock);
	return rc;
}

static inline bool is_device_suspended(struct smb1360_chip *chip)
{
	return !chip->resume_completed;
}

static int smb1360_disable_fg_access(struct smb1360_chip *chip)
{
	int rc;

	rc = smb1360_masked_write(chip, CMD_I2C_REG, FG_ACCESS_ENABLED_BIT, 0);
	if (rc)
		pr_err("Couldn't disable FG access rc=%d\n", rc);

	INIT_COMPLETION(chip->fg_mem_access_granted);

	return rc;
}

static int smb1360_enable_volatile_writes(struct smb1360_chip *chip)
{
	int rc;

	rc = smb1360_masked_write(chip, CMD_I2C_REG,
		ALLOW_VOLATILE_BIT, ALLOW_VOLATILE_BIT);
	if (rc < 0)
		dev_err(chip->dev,
			"Couldn't set VOLATILE_W_PERM_BIT rc=%d\n", rc);

	return rc;
}

void smb1360_otp_backup_pool_init(struct smb1360_chip *chip)
{
	struct otp_backup_pool *pool = &chip->otp_backup;

	pool->reg_start = 0xE0;
	pool->reg_end = 0xEF;
	pool->start_now = pool->reg_start;
	mutex_init(&pool->lock);
}

static int smb1360_alloc_otp_backup_register(struct smb1360_chip *chip,
						u8 size, int usage)
{
	int rc = 0, i;
	u8 inv_pos;
	struct otp_backup_pool *pool = &chip->otp_backup;

	if (size % 2) {
		pr_err("Must be allocated with pairs\n");
		return -EINVAL;
	}

	mutex_lock(&pool->lock);
	if (pool->start_now + size > pool->reg_end) {
		pr_err("Allocation fail: start = 0x%x, size = %d\n",
						pool->start_now, size);
		mutex_unlock(&pool->lock);
		return -EBUSY;
	}
	rc = pool->start_now;
	inv_pos = pool->reg_end - pool->start_now + 1;
	for (i = 0; i < size; i = i + 2) {
		inv_pos -= i;
		pool->alg_bitmap |= usage << (inv_pos - 2);
	}
	pr_debug("Allocation success, start = 0x%x, size = %d, alg_bitmap = 0x%x\n",
						rc, size, pool->alg_bitmap);
	pool->start_now += size;
	mutex_unlock(&pool->lock);

	return rc;
}

#define OTP_BACKUP_WA_ALG_1	0xF0
#define OTP_BACKUP_WA_ALG_2	0xF1
static int smb1360_otp_backup_alg_update(struct smb1360_chip *chip)
{
	int rc = 0;
	struct otp_backup_pool *pool = &chip->otp_backup;

	mutex_lock(&pool->lock);
	rc = smb1360_fg_write(chip, OTP_BACKUP_WA_ALG_1,
			(u8)(pool->alg_bitmap >> 8));
	rc |= smb1360_fg_write(chip, OTP_BACKUP_WA_ALG_2,
			(u8)(pool->alg_bitmap));
	if (rc)
		pr_err("Write FG address F0/F1 failed, rc = %d\n", rc);
	mutex_unlock(&pool->lock);

	return rc;
}

#define TRIM_1C_REG		0x1C
#define CHECK_USB100_GOOD_BIT	BIT(6)
static bool is_usb100_broken(struct smb1360_chip *chip)
{
	int rc;
	u8 reg;

	rc = smb1360_read(chip, TRIM_1C_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read trim 1C reg rc = %d\n", rc);
		return rc;
	}
	return !!(reg & CHECK_USB100_GOOD_BIT);
}

static int read_revision(struct smb1360_chip *chip, u8 *revision)
{
	int rc;

	*revision = 0;
	rc = smb1360_read(chip, REVISION_CTRL_REG, revision);
	if (rc)
		dev_err(chip->dev, "Couldn't read REVISION_CTRL_REG rc=%d", rc);

	*revision &= DEVICE_REV_MASK;

	return rc;
}

#define MIN_FLOAT_MV		3460
#define MAX_FLOAT_MV		4730
#define VFLOAT_STEP_MV		10
static int smb1360_float_voltage_set(struct smb1360_chip *chip, int vfloat_mv)
{
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}

	pr_info("set float_voltage ==> %d \n",vfloat_mv);

	temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

	return smb1360_masked_write(chip, BATT_CHG_FLT_VTG_REG,
				VFLOAT_MASK, temp);
}

#define MIN_RECHG_MV		50
#define MAX_RECHG_MV		300
static int smb1360_recharge_threshold_set(struct smb1360_chip *chip,
							int resume_mv)
{
	u8 temp;

	if ((resume_mv < MIN_RECHG_MV) || (resume_mv > MAX_RECHG_MV)) {
		dev_err(chip->dev, "bad rechg_thrsh =%d asked to set\n",
							resume_mv);
		return -EINVAL;
	}

	temp = resume_mv / 100;

	return smb1360_masked_write(chip, CFG_BATT_CHG_REG,
		RECHG_MV_MASK, temp << RECHG_MV_SHIFT);
}

static int __smb1360_charging_disable(struct smb1360_chip *chip, bool disable)
{
	int rc;

	rc = smb1360_masked_write(chip, CMD_CHG_REG,
			CMD_CHG_EN, disable ? 0 : CMD_CHG_EN);
	if (rc < 0)
		pr_err("Couldn't set CHG_ENABLE_BIT disable=%d rc = %d\n",
							disable, rc);
	else
		pr_debug("CHG_EN status=%d\n", !disable);

	return rc;
}

static int smb1360_charging_disable(struct smb1360_chip *chip, int reason,
								int disable)
{
	int rc = 0;
	int disabled;

	mutex_lock(&chip->charging_disable_lock);

	disabled = chip->charging_disabled_status;

	pr_debug("reason=%d requested_disable=%d disabled_status=%d\n",
					reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (disabled)
		rc = __smb1360_charging_disable(chip, true);
	else
		rc = __smb1360_charging_disable(chip, false);

	if (rc)
		pr_err("Couldn't disable charging for reason=%d rc=%d\n",
							rc, reason);
	else
		chip->charging_disabled_status = disabled;

	mutex_unlock(&chip->charging_disable_lock);

	return rc;
}

static int smb1360_soft_jeita_comp_enable(struct smb1360_chip *chip,
								bool enable)
{
	int rc = 0;

	rc = smb1360_masked_write(chip, CHG_CMP_CFG, JEITA_COMP_EN_MASK,
					enable ? JEITA_COMP_EN_BIT : 0);
	if (rc)
		pr_err("Couldn't %s JEITA compensation\n", enable ?
						"enable" : "disable");

	return rc;
}

static enum power_supply_property smb1360_battery_properties[] = {
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_RESISTANCE,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
};

static int smb1360_get_prop_batt_present(struct smb1360_chip *chip)
{
	return chip->batt_present;
}

static int smb1360_get_prop_batt_status(struct smb1360_chip *chip)
{
	int rc;
	u8 reg = 0, chg_type;

	if (is_device_suspended(chip))
		return POWER_SUPPLY_STATUS_UNKNOWN;

	if (chip->batt_full)
		return POWER_SUPPLY_STATUS_FULL;

	rc = smb1360_read(chip, STATUS_3_REG, &reg);
	if (rc) {
		pr_err("Couldn't read STATUS_3_REG rc=%d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	pr_debug("STATUS_3_REG = %x\n", reg);

	if (reg & CHG_HOLD_OFF_BIT)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;

	if (chg_type == BATT_NOT_CHG_VAL)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	else
		return POWER_SUPPLY_STATUS_CHARGING;
}

static int smb1360_get_prop_charge_type(struct smb1360_chip *chip)
{
	int rc;
	u8 reg = 0;
	u8 chg_type;

	if (is_device_suspended(chip))
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	rc = smb1360_read(chip, STATUS_3_REG, &reg);
	if (rc) {
		pr_err("Couldn't read STATUS_3_REG rc=%d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	if (chg_type == BATT_NOT_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	else if ((chg_type == BATT_FAST_CHG_VAL) ||
			(chg_type == BATT_TAPER_CHG_VAL))
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (chg_type == BATT_PRE_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int smb1360_get_prop_batt_health(struct smb1360_chip *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->batt_hot)
		ret.intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->batt_cold)
		ret.intval = POWER_SUPPLY_HEALTH_COLD;
	else if (chip->batt_warm)
		ret.intval = POWER_SUPPLY_HEALTH_WARM;
	else if (chip->batt_cool)
		ret.intval = POWER_SUPPLY_HEALTH_COOL;
	else
		ret.intval = POWER_SUPPLY_HEALTH_GOOD;

	return ret.intval;
}

static int smb1360_get_prop_batt_capacity(struct smb1360_chip *chip)
{
	u8 reg;
	u32 temp = 0;
	int rc, soc = 0;

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	if (chip->empty_soc) {
		pr_debug("empty_soc\n");
		return 0;
	}

	if (is_device_suspended(chip))
		return chip->soc_now;

	rc = smb1360_read(chip, SHDW_FG_MSYS_SOC, &reg);
	if (rc) {
		pr_err("Failed to read FG_MSYS_SOC rc=%d\n", rc);
		return rc;
	}
	soc = (100 * reg) / MAX_8_BITS;

	temp = (100 * reg) % MAX_8_BITS;
	if (temp > (MAX_8_BITS / 2))
		soc += 1;

	pr_debug("msys_soc_reg=0x%02x, fg_soc=%d batt_full = %d\n", reg,
						soc, chip->batt_full);

	chip->soc_now = (chip->batt_full ? 100 : bound(soc, 0, 100));

	return chip->soc_now;
}

static int smb1360_get_prop_chg_full_design(struct smb1360_chip *chip)
{
	u8 reg[2];
	int rc, fcc_mah = 0;

	if (is_device_suspended(chip))
		return chip->fcc_mah;

	rc = smb1360_read_bytes(chip, SHDW_FG_CAPACITY, reg, 2);
	if (rc) {
		pr_err("Failed to read SHDW_FG_CAPACITY rc=%d\n", rc);
		return rc;
	}
	fcc_mah = (reg[1] << 8) | reg[0];

	pr_debug("reg[0]=0x%02x reg[1]=0x%02x fcc_mah=%d\n",
				reg[0], reg[1], fcc_mah);

	chip->fcc_mah = fcc_mah * 1000;

	return chip->fcc_mah;
}

static int smb1360_get_prop_batt_temp(struct smb1360_chip *chip)
{
	u8 reg[2];
	int rc, temp = 0;

	if (is_device_suspended(chip))
		return chip->temp_now;

	rc = smb1360_read_bytes(chip, SHDW_FG_BATT_TEMP, reg, 2);
	if (rc) {
		pr_err("Failed to read SHDW_FG_BATT_TEMP rc=%d\n", rc);
		return rc;
	}

	temp = (reg[1] << 8) | reg[0];
	temp = div_u64(temp * 625, 10000UL);	/* temperature in kelvin */
	temp = (temp - 273) * 10;		/* temperature in decideg */

	pr_debug("reg[0]=0x%02x reg[1]=0x%02x temperature=%d\n",
					reg[0], reg[1], temp);

	chip->temp_now = temp;

	return chip->temp_now;
}

static int smb1360_get_prop_voltage_now(struct smb1360_chip *chip)
{
	u8 reg[2];
	int rc, temp = 0;

	if (is_device_suspended(chip))
		return chip->voltage_now;

	rc = smb1360_read_bytes(chip, SHDW_FG_VTG_NOW, reg, 2);
	if (rc) {
		pr_err("Failed to read SHDW_FG_VTG_NOW rc=%d\n", rc);
		return rc;
	}

	temp = (reg[1] << 8) | reg[0];
	temp = div_u64(temp * 5000, 0x7FFF);

	pr_debug("reg[0]=0x%02x reg[1]=0x%02x voltage=%d\n",
				reg[0], reg[1], temp * 1000);

	chip->voltage_now = temp * 1000;

	return chip->voltage_now;
}

static int smb1360_get_prop_batt_resistance(struct smb1360_chip *chip)
{
	u8 reg[2];
	u16 temp;
	int rc;
	int64_t resistance;

	if (is_device_suspended(chip))
		return chip->resistance_now;

	rc = smb1360_read_bytes(chip, SHDW_FG_ESR_ACTUAL, reg, 2);
	if (rc) {
		pr_err("Failed to read FG_ESR_ACTUAL rc=%d\n", rc);
		return rc;
	}
	temp = (reg[1] << 8) | reg[0];

	resistance = float_decode(temp) * 2;

	pr_debug("reg=0x%02x resistance=%lld\n", temp, resistance);

	/* resistance in uohms */
	chip->resistance_now = resistance;

	return chip->resistance_now;
}

static int smb1360_get_prop_current_now(struct smb1360_chip *chip)
{
	u8 reg[2];
	int rc, temp = 0;

	if (is_device_suspended(chip))
		return chip->current_now;

	rc = smb1360_read_bytes(chip, SHDW_FG_CURR_NOW, reg, 2);
	if (rc) {
		pr_err("Failed to read SHDW_FG_CURR_NOW rc=%d\n", rc);
		return rc;
	}

	temp = ((s8)reg[1] << 8) | reg[0];
	temp = div_s64(temp * 2500, 0x7FFF);

	pr_debug("reg[0]=0x%02x reg[1]=0x%02x current=%d\n",
				reg[0], reg[1], temp * 1000);

	chip->current_now = temp * 1000;

	return chip->current_now;
}

static int smb1360_set_minimum_usb_current(struct smb1360_chip *chip)
{
	int rc = 0;

	if (chip->min_icl_usb100) {
		pr_info("USB min current set to 100mA\n");
		/* set input current limit to minimum (300mA) */
		rc = smb1360_masked_write(chip, CFG_BATT_CHG_ICL_REG,
						INPUT_CURR_LIM_MASK,
						INPUT_CURR_LIM_300MA);
		if (rc)
			pr_err("Couldn't set ICL mA rc=%d\n", rc);

		if (!(chip->workaround_flags & WRKRND_USB100_FAIL))
			rc = smb1360_masked_write(chip, CMD_IL_REG,
					USB_CTRL_MASK, USB_100_BIT);
			if (rc)
				pr_err("Couldn't configure for USB100 rc=%d\n",
								rc);
	} else {
		pr_info("USB min current set to 500mA\n");
		rc = smb1360_masked_write(chip, CMD_IL_REG,
				USB_CTRL_MASK, USB_500_BIT);
		if (rc)
			pr_err("Couldn't configure for USB100 rc=%d\n",
							rc);
	}

	return rc;
}

static struct power_supply *get_parallel_psy(struct smb1360_chip *chip)
{
	if (chip->parallel_psy)
		return chip->parallel_psy;
	chip->parallel_psy = power_supply_get_by_name("usb-parallel");
	if (!chip->parallel_psy)
		pr_debug("parallel charger not found\n");
	return chip->parallel_psy;
}

static int __smb1360_parallel_charger_enable(struct smb1360_chip *chip,
							bool enable)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };

	if (!parallel_psy)
		return 0;

	pval.intval = (enable ? (chip->max_parallel_chg_current * 1000) : 0);
	parallel_psy->set_property(parallel_psy,
		POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	pval.intval = (enable ? 1 : 0);
	parallel_psy->set_property(parallel_psy,
		POWER_SUPPLY_PROP_CHARGING_ENABLED, &pval);

	pr_info("Parallel-charger %s max_chg_current=%d\n",
		enable ? "enabled" : "disabled",
		enable ? (chip->max_parallel_chg_current * 1000) : 0);

	return 0;
}

static int smb1360_parallel_charger_enable(struct smb1360_chip *chip,
						int reason, bool enable)
{
	int disabled, *disabled_status;

	mutex_lock(&chip->parallel_chg_lock);

	disabled = chip->parallel_chg_disable_status;
	disabled_status = &chip->parallel_chg_disable_status;

	pr_info("reason=0x%x requested=%s disabled_status=0x%x\n",
			reason, enable ? "enable" : "disable", disabled);

	if (enable == true)
		disabled &= ~reason;
	else
		disabled |= reason;

	if (*disabled_status && !disabled)
		__smb1360_parallel_charger_enable(chip, true);

	if (!(*disabled_status) && disabled)
		__smb1360_parallel_charger_enable(chip, false);

	*disabled_status = disabled;

	pr_info("disabled_status = %x\n", *disabled_status);

	mutex_unlock(&chip->parallel_chg_lock);

	return 0;
}

static void smb1360_parallel_work(struct work_struct *work)
{
	u8 reg;
	int rc, i;
	struct smb1360_chip *chip = container_of(work,
				struct smb1360_chip, parallel_work);

	/* check the AICL settled value */
	rc = smb1360_read(chip, STATUS_1_REG, &reg);
	if (rc) {
		pr_debug("Unable to read AICL status rc=%d\n", rc);
		goto exit_work;
	}
	pr_debug("STATUS_1 (aicl status)=0x%x\n", reg);
	if ((reg & AICL_CURRENT_STATUS_MASK) == AICL_LIMIT_1000MA && runin_switch_control_off == false) {
		/* Strong Charger - Enable parallel path */
		/* find the new fastchg current */
		chip->fastchg_current += (chip->max_parallel_chg_current / 2);
		for (i = 0; i < ARRAY_SIZE(fastchg_current) - 1;  i++) {
			if (fastchg_current[i] >= chip->fastchg_current)
				break;
		}
		if (i == ARRAY_SIZE(fastchg_current))
			i--;

		rc = smb1360_masked_write(chip, CHG_CURRENT_REG,
			FASTCHG_CURR_MASK, i << FASTCHG_CURR_SHIFT);
		if (rc)
			pr_err("Couldn't set fastchg mA rc=%d\n", rc);

		pr_debug("fast-chg (parallel-mode) current set to = %d\n",
							fastchg_current[i]);

		smb1360_parallel_charger_enable(chip, PARALLEL_CURRENT, true);
	} else {
		/* Weak-charger - Disable parallel path */
		smb1360_parallel_charger_enable(chip, PARALLEL_CURRENT, false);
	}

exit_work:
	smb1360_relax(&chip->smb1360_ws, WAKEUP_SRC_PARALLEL);
}

static int smb1360_set_appropriate_usb_current(struct smb1360_chip *chip)
{
	int rc = 0, i, therm_ma, current_ma;
	int path_current = chip->usb_psy_ma;

	/*
	 * If battery is absent do not modify the current at all, these
	 * would be some appropriate values set by the bootloader or default
	 * configuration and since it is the only source of power we should
	 * not change it
	 */
	if (!chip->batt_present) {
		pr_info("ignoring current request since battery is absent\n");
		return 0;
	}
	pr_info("chip->therm_lvl_sel ====> %d \n",chip->therm_lvl_sel);
	if (chip->therm_lvl_sel > 0
			&& chip->therm_lvl_sel < (chip->thermal_levels - 1))
		/*
		 * consider thermal limit only when it is active and not at
		 * the highest level
		 */
		therm_ma = chip->thermal_mitigation[chip->therm_lvl_sel];
	else
		if(chip->therm_lvl_sel == (chip->thermal_levels - 1))
			therm_ma = CURRENT_100_MA;
		else
			therm_ma = path_current;
	if(chip->therm_lvl_sel == 2)
	{
		if(smb1360_get_prop_batt_capacity(chip) >= 15)
			therm_ma = 600;
		else
			if(smb1360_get_prop_batt_capacity(chip) >= 8 &&smb1360_get_prop_batt_capacity(chip) < 15)
				therm_ma = 700;
			else
				therm_ma = 1400;
	}
	else
		if(chip->therm_lvl_sel == 1)
			therm_ma = 1400;

	current_ma = min(therm_ma, path_current);

	if (chip->workaround_flags & WRKRND_HARD_JEITA) {
		if (chip->batt_warm)
			current_ma = min(current_ma, chip->warm_bat_ma);
		else if (chip->batt_cool)
			current_ma = min(current_ma, chip->cool_bat_ma);
	}

	if (current_ma <= 2) {
		/*
		 * SMB1360 does not support USB suspend -
		 * so set the current-limit to minimum in suspend.
		 */
		pr_info("current_ma=%d <= 2 set USB current to minimum\n",
								current_ma);
		rc = smb1360_set_minimum_usb_current(chip);
		if (rc < 0)
			pr_err("Couldn't to set minimum USB current rc = %d\n",
								rc);
		/* disable parallel charger */
		if (chip->parallel_charging)
			smb1360_parallel_charger_enable(chip,
					PARALLEL_CURRENT, false);

		return rc;
	}

	for (i = ARRAY_SIZE(input_current_limit) - 1; i >= 0; i--) {
		if (input_current_limit[i] <= current_ma)
			break;
	}
	if (i < 0) {
		pr_info("Couldn't find ICL mA rc=%d\n", rc);
		i = 0;
	}
	/* set input current limit */
	rc = smb1360_masked_write(chip, CFG_BATT_CHG_ICL_REG,
					INPUT_CURR_LIM_MASK, i);
	if (rc)
		pr_err("Couldn't set ICL mA rc=%d\n", rc);

	pr_info("ICL set to = %d,current_ma = %d\n", input_current_limit[i],current_ma);

	if ((current_ma <= CURRENT_100_MA) &&
		((chip->workaround_flags & WRKRND_USB100_FAIL) ||
				!chip->min_icl_usb100)) {
		pr_info("usb100 not supported: usb100_wrkrnd=%d min_icl_100=%d\n",
			!!(chip->workaround_flags & WRKRND_USB100_FAIL),
						chip->min_icl_usb100);
		current_ma = CURRENT_500_MA;
	}

	if (current_ma <= CURRENT_100_MA) {
		/* USB 100 */
		rc = smb1360_masked_write(chip, CMD_IL_REG,
				USB_CTRL_MASK, USB_100_BIT);
		if (rc)
			pr_err("Couldn't configure for USB100 rc=%d\n", rc);
		pr_info("Setting USB 100\n");
	//} else if (current_ma <= CURRENT_500_MA) {
		/* USB 500 */
	//	rc = smb1360_masked_write(chip, CMD_IL_REG,
	//			USB_CTRL_MASK, USB_500_BIT);
	//	if (rc)
	//		pr_err("Couldn't configure for USB500 rc=%d\n", rc);
	//	pr_info("Setting USB 500\n");
	} else {
		/* USB AC */
		//if (chip->rsense_10mohm)
		//	current_ma /= 2;

		for (i = ARRAY_SIZE(fastchg_current) - 1; i >= 0; i--) {
			if (fastchg_current[i] <= current_ma)
				break;
		}
		if (i < 0) {
			pr_info("Couldn't find fastchg mA rc=%d\n", rc);
			i = 0;
		}

		chip->fastchg_current = fastchg_current[i];

		/* set fastchg limit */
		rc = smb1360_masked_write(chip, CHG_CURRENT_REG,
			FASTCHG_CURR_MASK, i << FASTCHG_CURR_SHIFT);
		if (rc)
			pr_err("Couldn't set fastchg mA rc=%d\n", rc);

		/*
		 * To move to a new (higher) input-current setting,
		 * first set USB500 and then USBAC. This makes sure
		 * that the new ICL setting takes affect.
		 */
		rc = smb1360_masked_write(chip, CMD_IL_REG,
				USB_CTRL_MASK, USB_500_BIT);
		if (rc)
			pr_err("Couldn't configure for USB500 rc=%d\n", rc);

		rc = smb1360_masked_write(chip, CMD_IL_REG,
				USB_CTRL_MASK, USB_AC_BIT);
		if (rc)
			pr_err("Couldn't configure for USB AC rc=%d\n", rc);

		pr_info("fast-chg current set to = %d\n", fastchg_current[i]);
	}

	return rc;
}

static int smb1360_set_jeita_comp_curr(struct smb1360_chip *chip,
							int current_ma)
{
	int i;
	int rc = 0;

	for (i = ARRAY_SIZE(fastchg_current) - 1; i >= 0; i--) {
		if (fastchg_current[i] <= current_ma)
			break;
	}
	if (i < 0) {
		pr_debug("Couldn't find fastchg_current %dmA\n", current_ma);
		i = 0;
	}

	rc = smb1360_masked_write(chip, CHG_CMP_CFG,
			JEITA_COMP_CURR_MASK, i);
	if (rc)
		pr_err("Couldn't configure for Icomp, rc = %d\n", rc);

	return rc;
}

#define TEMP_THRE_SET(x) ((x + 300) / 10)
#define TEMP_THRE_GET(x) ((x * 10) - 300)
static int smb1360_set_soft_jeita_threshold(struct smb1360_chip *chip,
					int cold_threshold, int hot_threshold)
{
	int rc = 0;

	rc = smb1360_write(chip, JEITA_SOFT_COLD_REG,
				TEMP_THRE_SET(cold_threshold));
	if (rc) {
		pr_err("Couldn't set soft cold threshold, rc = %d\n", rc);
		return rc;
	} else {
		chip->soft_cold_thresh = cold_threshold;
	}

	rc = smb1360_write(chip, JEITA_SOFT_HOT_REG,
				TEMP_THRE_SET(hot_threshold));
	if (rc) {
		pr_err("Couldn't set soft hot threshold, rc = %d\n", rc);
		return rc;
	} else {
		chip->soft_hot_thresh = hot_threshold;
	}

	return rc;
}

static int smb1360_get_soft_jeita_threshold(struct smb1360_chip *chip,
				int *cold_threshold, int *hot_threshold)
{
	int rc = 0;
	u8 value;

	rc = smb1360_read(chip, JEITA_SOFT_COLD_REG, &value);
	if (rc) {
		pr_err("Couldn't get soft cold threshold, rc = %d\n", rc);
		return rc;
	}
	*cold_threshold = TEMP_THRE_GET(value);

	rc = smb1360_read(chip, JEITA_SOFT_HOT_REG, &value);
	if (rc) {
		pr_err("Couldn't get soft hot threshold, rc = %d\n", rc);
		return rc;
	}
	*hot_threshold = TEMP_THRE_GET(value);

	return rc;
}

#define OTP_HARD_COLD_REG_ADDR	0x12
#define OTP_HARD_HOT_REG_ADDR	0x13
static int smb1360_set_otp_hard_jeita_threshold(struct smb1360_chip *chip,
				int cold_threshold, int hot_threshold)
{
	int rc = 0, i;
	u8 reg[4] = { 0 };
	u8 otp_reg = 0;
	int temp_code;

	if (cold_threshold > chip->cool_bat_decidegc ||
		chip->cool_bat_decidegc >= chip->warm_bat_decidegc ||
		chip->warm_bat_decidegc > hot_threshold) {
		pr_err("cold:%d, cool:%d, warm:%d, hot:%d should be ordered in size\n",
			cold_threshold, chip->cool_bat_decidegc,
			chip->warm_bat_decidegc, hot_threshold);
		return -EINVAL;
	}
	pr_debug("cold:%d, cool:%d, warm:%d, hot:%d\n",
			cold_threshold, chip->cool_bat_decidegc,
			chip->warm_bat_decidegc, hot_threshold);
	if (!chip->hard_jeita_otp_reg) {
		otp_reg = smb1360_alloc_otp_backup_register(chip,
				ARRAY_SIZE(reg), OTP_BACKUP_FG_USE);
		if (otp_reg <= 0) {
			pr_err("OTP reg allocation failed for hard JEITA\n");
			return otp_reg;
		}

		chip->hard_jeita_otp_reg = otp_reg;
	} else {
		otp_reg = chip->hard_jeita_otp_reg;
	}
	pr_debug("hard_jeita_otp_reg = 0x%x\n", chip->hard_jeita_otp_reg);

	reg[0] = (u8)OTP_HARD_HOT_REG_ADDR;
	temp_code = TEMP_THRE_SET(hot_threshold);
	if (temp_code < 0) {
		pr_err("hard hot temp encode failed\n");
		return temp_code;
	}
	reg[1] = (u8)temp_code;
	reg[2] = (u8)OTP_HARD_COLD_REG_ADDR;
	temp_code = TEMP_THRE_SET(cold_threshold);
	if (temp_code < 0) {
		pr_err("hard cold temp encode failed\n");
		return temp_code;
	}
	reg[3] = (u8)temp_code;

	rc = smb1360_enable_fg_access(chip);
	if (rc) {
		pr_err("Couldn't request FG access rc = %d\n", rc);
		return rc;
	}
	chip->fg_access_type = FG_ACCESS_CFG;

	rc = smb1360_select_fg_i2c_address(chip);
	if (rc) {
		pr_err("Unable to set FG access I2C address\n");
		goto restore_fg;
	}

	for (i = 0; i < ARRAY_SIZE(reg); i++) {
		rc = smb1360_fg_write(chip, (otp_reg + i), reg[i]);
		if (rc) {
			pr_err("Write FG address 0x%x: 0x%x failed, rc = %d\n",
					otp_reg + i, reg[i], rc);
			goto restore_fg;
		}
		pr_debug("Write FG addr=0x%x, value=0x%x\n",
					otp_reg + i, reg[i]);
	}
	rc = smb1360_otp_backup_alg_update(chip);
	if (rc) {
		pr_err("Update OTP backup algorithm failed\n");
		goto restore_fg;
	}

	rc = smb1360_masked_write(chip, CFG_FG_BATT_CTRL_REG,
			CFG_FG_OTP_BACK_UP_ENABLE, CFG_FG_OTP_BACK_UP_ENABLE);
	if (rc) {
		pr_err("Write reg 0x0E failed, rc = %d\n", rc);
		goto restore_fg;
	}

restore_fg:
	rc = smb1360_disable_fg_access(chip);
	if (rc) {
		pr_err("Couldn't disable FG access rc = %d\n", rc);
		return rc;
	}

	return rc;
}

static int smb1360_hard_jeita_otp_init(struct smb1360_chip *chip)
{
	int rc = 0;

	if (!chip->otp_hard_jeita_config)
		return rc;

	rc = smb1360_set_otp_hard_jeita_threshold(chip,
		chip->otp_cold_bat_decidegc, chip->otp_hot_bat_decidegc);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set OTP hard jeita threshold,rc = %d\n", rc);
		return rc;
	}

	return rc;
}

#if defined(CONFIG_ASUS_ZC550KL8939_PROJECT)
static int reverse_current_control_thermal(struct smb1360_chip *chip, int lvl_sel)
{
	int local_lvl = -1;
	pr_info("lvl_sel = %d\n", lvl_sel);

	if(local_lvl == lvl_sel)
		return 0;
	bank_hot_lvl = lvl_sel;
	local_lvl = lvl_sel;
	if(local_lvl == THERMAL_DCP_LEVEL)
		asus_otg_chargetype_set(USB_SDP_MODE);
	else if(local_lvl == THERMAL_SDP_LEVEL)
		asus_otg_chargetype_set(USB_DCP_MODE);

	return 0;
}
#endif

static int smb1360_system_temp_level_set(struct smb1360_chip *chip,
							int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;
	pr_info("start; lvl_sel = %d\n", lvl_sel);
	if (!chip->thermal_mitigation) {
		pr_err("Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		pr_err("Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}
#if defined(CONFIG_ASUS_ZC550KL8939_PROJECT)
	if((chip->HW_ID == ZC550KL_8939_ER || chip->HW_ID == ZC550KL_8939_PR) && (g_CHG_mode == 2)){
		rc = reverse_current_control_thermal(chip, lvl_sel);
		pr_info("in otg mode!\n");
		return 0;
	}
#endif
	if (lvl_sel >= chip->thermal_levels) {
		pr_err("Unsupported level selected %d forcing %d\n", lvl_sel,
				chip->thermal_levels - 1);
		lvl_sel = chip->thermal_levels - 1;
	}

	if (lvl_sel == chip->therm_lvl_sel)
		return 0;

	mutex_lock(&chip->current_change_lock);
	prev_therm_lvl = chip->therm_lvl_sel;
	chip->therm_lvl_sel = lvl_sel;
	if (chip->therm_lvl_sel == (chip->thermal_levels - 1)) {
		rc = smb1360_set_minimum_usb_current(chip);
		if (rc)
			pr_err("Couldn't set USB current to minimum rc = %d\n",
							rc);
	} else {
		rc = smb1360_set_appropriate_usb_current(chip);
		if (rc)
			pr_err("Couldn't set USB current rc = %d\n", rc);
	}

	mutex_unlock(&chip->current_change_lock);
	return rc;
}

static int smb1360_battery_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct smb1360_chip *chip = container_of(psy,
				struct smb1360_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		smb1360_charging_disable(chip, USER, !val->intval);
		if (chip->parallel_charging)
			smb1360_parallel_charger_enable(chip,
				PARALLEL_USER, val->intval);
		power_supply_changed(&chip->batt_psy);
		power_supply_changed(chip->usb_psy);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		pr_info("fake_soc set to %d\n", chip->fake_battery_soc);
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		smb1360_system_temp_level_set(chip, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smb1360_battery_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int smb1360_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb1360_chip *chip = container_of(psy,
				struct smb1360_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = smb1360_get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = smb1360_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = asus_battery_update_status();
		//val->intval = smb1360_get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !chip->charging_disabled_status;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = smb1360_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = smb1360_get_prop_batt_capacity(chip);
		if(val->intval == 0 && smb1360_get_prop_batt_temp(chip) < 0 && g_ASUS_bootmode == USER_MODE)
		{
			val->intval = 50;
		}
/*#if defined(CONFIG_ASUS_ZC550KL8939_PROJECT)
		if(g_CHG_mode == 2 && (chip->HW_ID == ZC550KL_8939_ER || chip->HW_ID == ZC550KL_8939_PR)
			&& val->intval <= 40)
			asus_otg_chargetype_set(USB_SDP_MODE);
#endif*/
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = smb1360_get_prop_chg_full_design(chip);
		val->intval = 5000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = smb1360_get_prop_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = smb1360_get_prop_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_RESISTANCE:
		val->intval = smb1360_get_prop_batt_resistance(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = smb1360_get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
#if defined(CONFIG_ASUS_ZC550KL8939_PROJECT)
		if(g_CHG_mode == 2){
			val->intval = bank_hot_lvl;
			break;
		}
#endif
		val->intval = chip->therm_lvl_sel;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void smb1360_external_power_changed(struct power_supply *psy)
{
	struct smb1360_chip *chip = container_of(psy,
				struct smb1360_chip, batt_psy);
	union power_supply_propval prop = {0,};
	int rc, current_limit = 0;

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc < 0)
		dev_err(chip->dev,
			"could not read USB current_max property, rc=%d\n", rc);
	else
		current_limit = prop.intval / 1000;

	pr_debug("current_limit = %d\n", current_limit);
/*+++BSP frank add asus_set_charger +++  remove set_appropriate_usb_current*/
/*	if (chip->usb_psy_ma != current_limit) {
		mutex_lock(&chip->current_change_lock);
		chip->usb_psy_ma = current_limit;
		rc = smb1360_set_appropriate_usb_current(chip);
		if (rc < 0)
			pr_err("Couldn't set usb current rc = %d\n", rc);
		mutex_unlock(&chip->current_change_lock);
	}
*/
/*---BSP frank add asus_set_charger ---*/
	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_ONLINE, &prop);
	if (rc < 0)
		pr_err("could not read USB ONLINE property, rc=%d\n", rc);

	/* update online property */
	rc = 0;
	if (chip->usb_present && !chip->charging_disabled_status
					//&& chip->usb_psy_ma != 0) {
					&& g_usb_state == USB_IN) {
		if (prop.intval == 0)
			rc = power_supply_set_online(chip->usb_psy, true);
	} else {
		if (prop.intval == 1)
			rc = power_supply_set_online(chip->usb_psy, false);
	}
	if (rc < 0)
		pr_err("could not set usb online, rc=%d\n", rc);
}

static int hot_hard_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_debug("rt_stat = 0x%02x\n", rt_stat);
	chip->batt_hot = !!rt_stat;

	if (chip->parallel_charging) {
		pr_debug("%s parallel-charging\n", chip->batt_hot ?
					"Disable" : "Enable");
		smb1360_parallel_charger_enable(chip,
				PARALLEL_JEITA_HARD, !chip->batt_hot);
	}
	if (chip->hot_hysteresis) {
		smb1360_stay_awake(&chip->smb1360_ws,
			WAKEUP_SRC_JEITA_HYSTERSIS);
		schedule_work(&chip->jeita_hysteresis_work);
	}

	return 0;
}

static int cold_hard_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_debug("rt_stat = 0x%02x\n", rt_stat);
	chip->batt_cold = !!rt_stat;

	if (chip->parallel_charging) {
		pr_debug("%s parallel-charging\n", chip->batt_cold ?
					"Disable" : "Enable");
		smb1360_parallel_charger_enable(chip,
				PARALLEL_JEITA_HARD, !chip->batt_cold);
	}
	if (chip->cold_hysteresis) {
		smb1360_stay_awake(&chip->smb1360_ws,
			WAKEUP_SRC_JEITA_HYSTERSIS);
		schedule_work(&chip->jeita_hysteresis_work);
	}

	return 0;
}

static void smb1360_jeita_hysteresis_work(struct work_struct *work)
{
	int rc = 0;
	int hard_hot, hard_cold;
	struct smb1360_chip *chip = container_of(work,
			struct smb1360_chip, jeita_hysteresis_work);

	/* disable hard JEITA IRQ first */
	rc = smb1360_masked_write(chip, IRQ_CFG_REG,
			IRQ_BAT_HOT_COLD_HARD_BIT, 0);
	if (rc) {
		pr_err("disable hard JEITA IRQ failed, rc = %d\n", rc);
		goto exit_worker;
	}
	hard_hot = chip->otp_hot_bat_decidegc;
	hard_cold = chip->otp_cold_bat_decidegc;
	if (chip->batt_hot)
		hard_hot -= chip->hot_hysteresis;
	else if (chip->batt_cold)
		hard_cold += chip->cold_hysteresis;

	rc = smb1360_set_otp_hard_jeita_threshold(chip, hard_cold, hard_hot);
	if (rc) {
		pr_err("set hard JEITA threshold failed\n");
		goto exit_worker;
	}
	pr_debug("hard cold: %d, hard hot: %d reprogramed\n",
					hard_cold, hard_hot);
	/* enable hard JEITA IRQ at the end */
	rc = smb1360_masked_write(chip, IRQ_CFG_REG,
		IRQ_BAT_HOT_COLD_HARD_BIT, IRQ_BAT_HOT_COLD_HARD_BIT);
	if (rc)
		pr_err("enable hard JEITA IRQ failed\n");
exit_worker:
	smb1360_relax(&chip->smb1360_ws, WAKEUP_SRC_JEITA_HYSTERSIS);
}

/*
 * This worker thread should only be called when WRKRND_HARD_JEITA
 * is set.
 * It is needed to re-program JEITA soft thresholds, compensate
 * target voltage and charging current manually.
 * The function is required as JEITA hard thresholds can't be programmed.
*/
static void smb1360_jeita_work_fn(struct work_struct *work)
{
	int temp;
	int rc = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct smb1360_chip *chip = container_of(dwork, struct smb1360_chip,
							jeita_work);
	temp = smb1360_get_prop_batt_temp(chip);

	if (temp > chip->hot_bat_decidegc) {
		/* battery status is hot, only config thresholds */
		rc = smb1360_set_soft_jeita_threshold(chip,
			chip->warm_bat_decidegc, chip->hot_bat_decidegc);
		if (rc) {
			dev_err(chip->dev, "Couldn't set jeita threshold\n");
			goto end;
		}
	} else if (temp > chip->warm_bat_decidegc ||
		(temp == chip->warm_bat_decidegc && !!chip->soft_hot_rt_stat)) {
		/* battery status is warm, do compensation manually */
		chip->batt_warm = true;
		chip->batt_cool = false;
		rc = smb1360_float_voltage_set(chip, chip->warm_bat_mv);
		if (rc) {
			dev_err(chip->dev, "Couldn't set float voltage\n");
			goto end;
		}
		rc = smb1360_set_appropriate_usb_current(chip);
		if (rc)
			pr_err("Couldn't set USB current\n");
		rc = smb1360_set_soft_jeita_threshold(chip,
			chip->warm_bat_decidegc, chip->hot_bat_decidegc);
		if (rc) {
			dev_err(chip->dev, "Couldn't set jeita threshold\n");
			goto end;
		}
	} else if (temp > chip->cool_bat_decidegc ||
		(temp == chip->cool_bat_decidegc && !chip->soft_cold_rt_stat)) {
		/* battery status is good, do the normal charging */
		chip->batt_warm = false;
		chip->batt_cool = false;
		rc = smb1360_float_voltage_set(chip, chip->vfloat_mv);
		if (rc) {
			dev_err(chip->dev, "Couldn't set float voltage\n");
			goto end;
		}
		rc = smb1360_set_appropriate_usb_current(chip);
		if (rc)
			pr_err("Couldn't set USB current\n");
		rc = smb1360_set_soft_jeita_threshold(chip,
			chip->cool_bat_decidegc, chip->warm_bat_decidegc);
		if (rc) {
			dev_err(chip->dev, "Couldn't set jeita threshold\n");
			goto end;
		}
	} else if (temp > chip->cold_bat_decidegc) {
		/* battery status is cool, do compensation manually */
		chip->batt_cool = true;
		chip->batt_warm = false;
		rc = smb1360_float_voltage_set(chip, chip->cool_bat_mv);
		if (rc) {
			dev_err(chip->dev, "Couldn't set float voltage\n");
			goto end;
		}
		rc = smb1360_set_soft_jeita_threshold(chip,
			chip->cold_bat_decidegc, chip->cool_bat_decidegc);
		if (rc) {
			dev_err(chip->dev, "Couldn't set jeita threshold\n");
			goto end;
		}
	} else {
		/* battery status is cold, only config thresholds */
		rc = smb1360_set_soft_jeita_threshold(chip,
			chip->cold_bat_decidegc, chip->cool_bat_decidegc);
		if (rc) {
			dev_err(chip->dev, "Couldn't set jeita threshold\n");
			goto end;
		}
	}

	pr_debug("warm %d, cool %d, soft_cold_rt_sts %d, soft_hot_rt_sts %d, jeita supported %d, threshold_now %d %d\n",
		chip->batt_warm, chip->batt_cool, !!chip->soft_cold_rt_stat,
		!!chip->soft_hot_rt_stat, chip->soft_jeita_supported,
		chip->soft_cold_thresh, chip->soft_hot_thresh);
end:
	smb1360_relax(&chip->smb1360_ws, WAKEUP_SRC_JEITA_SOFT);
}

static int hot_soft_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	chip->soft_hot_rt_stat = rt_stat;
	pr_debug("rt_stat = 0x%02x\n", rt_stat);
	if (!chip->config_hard_thresholds)
		chip->batt_warm = !!rt_stat;

	if (chip->workaround_flags & WRKRND_HARD_JEITA) {
		cancel_delayed_work_sync(&chip->jeita_work);
		schedule_delayed_work(&chip->jeita_work,
					msecs_to_jiffies(JEITA_WORK_MS));
		smb1360_stay_awake(&chip->smb1360_ws,
			WAKEUP_SRC_JEITA_SOFT);
	}

	if (chip->parallel_charging) {
		pr_debug("%s parallel-charging\n", chip->batt_warm ?
					"Disable" : "Enable");
		smb1360_parallel_charger_enable(chip,
				PARALLEL_JEITA_SOFT, !chip->batt_warm);
	}
	return 0;
}

static int cold_soft_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	chip->soft_cold_rt_stat = rt_stat;
	pr_debug("rt_stat = 0x%02x\n", rt_stat);
	if (!chip->config_hard_thresholds)
		chip->batt_cool = !!rt_stat;

	if (chip->workaround_flags & WRKRND_HARD_JEITA) {
		cancel_delayed_work_sync(&chip->jeita_work);
		schedule_delayed_work(&chip->jeita_work,
					msecs_to_jiffies(JEITA_WORK_MS));
		smb1360_stay_awake(&chip->smb1360_ws,
			WAKEUP_SRC_JEITA_SOFT);
	}

	if (chip->parallel_charging) {
		pr_debug("%s parallel-charging\n", chip->batt_cool ?
					"Disable" : "Enable");
		smb1360_parallel_charger_enable(chip,
				PARALLEL_JEITA_SOFT, !chip->batt_cool);
	}

	return 0;
}

static int battery_missing_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_debug("rt_stat = 0x%02x\n", rt_stat);
	chip->batt_present = !rt_stat;
	return 0;
}

static int vbat_low_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_debug("vbat low\n");

	return 0;
}

static int chg_hot_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_warn_ratelimited("chg hot\n");
	return 0;
}

static int chg_term_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_debug("rt_stat = 0x%02x\n", rt_stat);
	chip->batt_full = !!rt_stat;

	if (chip->parallel_charging) {
		pr_debug("%s parallel-charging\n", chip->batt_full ?
					"Disable" : "Enable");
		smb1360_parallel_charger_enable(chip,
				PARALLEL_EOC, !chip->batt_full);
	}

	return 0;
}

static int chg_fastchg_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_debug("rt_stat = 0x%02x\n", rt_stat);

	return 0;
}

static int usbin_uv_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	bool usb_present = !rt_stat;
	static long time_cal = 0;
	pr_info("chip->usb_present = %d usb_present = %d\n",
				chip->usb_present, usb_present);
	if (chip->usb_present && !usb_present) {
		/* USB removed */
		chip->usb_present = usb_present;
		power_supply_set_present(chip->usb_psy, usb_present);
		focal_usb_detection(false);
		plugout_time = current_kernel_time();
	}

	if (!chip->usb_present && usb_present) {
		/* USB inserted */
		chip->usb_present = usb_present;
		power_supply_set_present(chip->usb_psy, usb_present);
		focal_usb_detection(true);
		plugin_time = current_kernel_time();
		time_cal = plugin_time.tv_sec - plugout_time.tv_sec;
		time_cal = time_cal * 1000000000 + (plugin_time.tv_nsec - plugout_time.tv_nsec);
		pr_info("time_cal ====> %ld\n",time_cal);
		if ( time_cal <= 100000000 ){
			//usbin_check_count ++;
			//if(usbin_check_count >= 3)
			//{
					usbin_voltage_poor = true;
					pr_info("usbin voltage is poor!\n");
			//}
		}
		else
		{
			//usbin_check_count = 0;
			usbin_voltage_poor = false;
		}
	}

	if(smb1360_probe_success == true)
		wake_lock_timeout(&usbin_wake_lock, 3*HZ);
	else
		pr_info("usbin_wake_lock = NULL \n");

	return 0;
}

static int aicl_done_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	bool aicl_done = !!rt_stat;

	pr_debug("AICL done=%d\n", aicl_done);

	if (chip->parallel_charging && aicl_done) {
		cancel_work_sync(&chip->parallel_work);
		smb1360_stay_awake(&chip->smb1360_ws, WAKEUP_SRC_PARALLEL);
		schedule_work(&chip->parallel_work);
	}

	return 0;
}

static int chg_inhibit_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	/*
	 * charger is inserted when the battery voltage is high
	 * so h/w won't start charging just yet. Treat this as
	 * battery full
	 */
	pr_debug("rt_stat = 0x%02x\n", rt_stat);
	chip->batt_full = !!rt_stat;
	return 0;
}

static int delta_soc_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_debug("SOC changed! - rt_stat = 0x%02x\n", rt_stat);

	return 0;
}

static int min_soc_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_debug("SOC dropped below min SOC, rt_stat = 0x%02x\n", rt_stat);

	if (chip->awake_min_soc)
		rt_stat ? smb1360_stay_awake(&chip->smb1360_ws,
				WAKEUP_SRC_MIN_SOC) :
			smb1360_relax(&chip->smb1360_ws,
				WAKEUP_SRC_MIN_SOC);

	return 0;
}

static int empty_soc_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_debug("SOC empty! rt_stat = 0x%02x\n", rt_stat);

	if (!chip->empty_soc_disabled) {
		if (rt_stat) {
			chip->empty_soc = true;
			smb1360_stay_awake(&chip->smb1360_ws,
				WAKEUP_SRC_EMPTY_SOC);
			pr_warn_ratelimited("SOC is 0\n");
		} else {
			chip->empty_soc = false;
			smb1360_relax(&chip->smb1360_ws,
				WAKEUP_SRC_EMPTY_SOC);
		}
	}

	return 0;
}

static int full_soc_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	if (rt_stat)
		pr_debug("SOC is 100\n");

	return 0;
}

static int fg_access_allowed_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_debug("stat=%d\n", !!rt_stat);

	if (rt_stat & FG_ACCESS_ALLOWED_BIT) {
		pr_debug("FG access granted\n");
		complete_all(&chip->fg_mem_access_granted);
	}

	return 0;
}

static int batt_id_complete_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_debug("batt_id = %x\n", (rt_stat & BATT_ID_RESULT_BIT)
						>> BATT_ID_SHIFT);

	return 0;
}

static int smb1360_adjust_current_gain(struct smb1360_chip *chip,
						int gain_factor)
{
	int i, rc;
	int64_t current_gain, new_current_gain;
	u16 reg_value1 = 0, reg_value2 = 0;
	u8 reg[4] = {0x1D, 0x00, 0x1E, 0x00};
	u8 otp_reg = 0;

	if (!chip->current_gain_otp_reg) {
		otp_reg = smb1360_alloc_otp_backup_register(chip,
				ARRAY_SIZE(reg), OTP_BACKUP_FG_USE);
		if (otp_reg <= 0) {
			pr_err("OTP reg allocation fail for adjusting current gain\n");
			return otp_reg;
		} else {
			chip->current_gain_otp_reg = otp_reg;
		}
	} else {
		otp_reg = chip->current_gain_otp_reg;
	}
	pr_debug("current_gain_otp_reg = 0x%x\n", chip->current_gain_otp_reg);

	if (gain_factor) {
		rc = smb1360_fg_read(chip, CURRENT_GAIN_LSB_REG, &reg[1]);
		if (rc) {
			pr_err("Unable to set FG access I2C address rc=%d\n",
									rc);
			return rc;
		}

		rc = smb1360_fg_read(chip, CURRENT_GAIN_MSB_REG, &reg[3]);
		if (rc) {
			pr_err("Unable to set FG access I2C address rc=%d\n",
									rc);
			return rc;
		}

		reg_value1 = (reg[3] << 8) | reg[1];
		current_gain = float_decode(reg_value1);
		new_current_gain = MICRO_UNIT  + (gain_factor * current_gain);
		reg_value2 = float_encode(new_current_gain);
		reg[1] = reg_value2 & 0xFF;
		reg[3] = (reg_value2 & 0xFF00) >> 8;
		pr_debug("current_gain_reg=0x%x current_gain_decoded=%lld new_current_gain_decoded=%lld new_current_gain_reg=0x%x\n",
			reg_value1, current_gain, new_current_gain, reg_value2);

		for (i = 0; i < ARRAY_SIZE(reg); i++) {
			pr_debug("Writing reg_add=%x value=%x\n",
					otp_reg + i, reg[i]);

			rc = smb1360_fg_write(chip, (otp_reg + i), reg[i]);
			if (rc) {
				pr_err("Write FG address 0x%x failed, rc = %d\n",
							otp_reg + i, rc);
				return rc;
			}
		}
		rc = smb1360_otp_backup_alg_update(chip);
		if (rc) {
			pr_err("Update OTP backup algorithm failed\n");
			return rc;
		}
	} else {
		pr_debug("Disabling gain correction\n");
		rc = smb1360_fg_write(chip, 0xF0, 0x00);
		if (rc) {
			pr_err("Write fg address 0x%x failed, rc = %d\n",
								0xF0, rc);
			return rc;
		}
	}

	return 0;
}
static int smb1360_otp_rslow_cfg(struct smb1360_chip *chip)
{
	int rc, i;
	u8 address, data;
	u8 reg_val_mapping_standalone[][2] = {
			{0xE0, 0x54},
			{0xE1, 0x18},
			{0xE2, 0x55},
			{0xE3, 0x6A},
			{0xE4, 0x56},
			{0xE5, 0x4C},
			{0xE6, 0x57},
			{0xE7, 0x6A},
			{0xF0, 0xAA},
			{0xF1, 0x00},
	};
	u8 reg_val_mapping_w_rsense[][2] = {
			{0xE4, 0x54},
			{0xE5, 0x18},
			{0xE6, 0x55},
			{0xE7, 0x6A},
			{0xE8, 0x56},
			{0xE9, 0x4C},
			{0xEA, 0x57},
			{0xEB, 0x6A},
			{0xF0, 0x5A},
			{0xF1, 0xA0},
	};

	for (i = 0; i < ARRAY_SIZE(reg_val_mapping_standalone); i++) {

		if (chip->rsense_10mohm || chip->otg_fet_present) {
			address = reg_val_mapping_w_rsense[i][0];
			data = reg_val_mapping_w_rsense[i][1];
		} else {
			address = reg_val_mapping_standalone[i][0];
			data = reg_val_mapping_standalone[i][1];
		}

		pr_debug("Writing reg_add=%x value=%x\n", address, data);

		rc = smb1360_fg_write(chip, address, data);
		if (rc) {
			pr_err("Write fg address 0x%x failed, rc = %d\n",
						address, rc);
			return rc;
		}
	}

	return 0;
}

static int smb1360_otp_jeita_hard_config(struct smb1360_chip *chip)
{
	int rc, i;
	u8 address, data;
	u8 reg_val_mapping_standalone[][2] = {
			{0xE0, 0x13},
			{0xE1, 0x5A}, /* hot @ 60C */
			{0xF0, 0x40},
	};

	u8 reg_val_mapping_w_rsense[][2] = {
			{0xE4, 0x13},
			{0xE5, 0x5A}, /* hoy @ 60C */
			{0xF0, 0x54}
	};

	u8 reg_val_mapping_w_rslow[][2] = {
			{0xE8, 0x13},
			{0xE9, 0x5A}, /* hoy @ 60C */
			{0xF1, 0x40}
	};

	u8 reg_val_mapping_w_rsense_rslow[][2] = {
			{0xEC, 0x13},
			{0xED, 0x5A},
			{0xF1, 0xA4},
	};

	pr_debug("Setting hard-jeita rsense=%d otg_fet_gain=%d\n",
				chip->rsense_10mohm, chip->otg_fet_present);

	for (i = 0; i < ARRAY_SIZE(reg_val_mapping_standalone); i++) {

		if (!chip->rsense_10mohm && !chip->otg_fet_present) {
			address = reg_val_mapping_standalone[i][0];
			data = reg_val_mapping_standalone[i][1];
		} else if (chip->rsense_10mohm && chip->otg_fet_present) {
			address = reg_val_mapping_w_rsense_rslow[i][0];
			data = reg_val_mapping_w_rsense_rslow[i][1];
		} else if (chip->rsense_10mohm && !chip->otg_fet_present) {
			address = reg_val_mapping_w_rsense[i][0];
			data = reg_val_mapping_w_rsense[i][1];
		} else if (!chip->rsense_10mohm && chip->otg_fet_present) {
			address = reg_val_mapping_w_rslow[i][0];
			data = reg_val_mapping_w_rslow[i][1];
		}

		pr_debug("Writing reg_add=%x value=%x\n", address, data);

		rc = smb1360_fg_write(chip, address, data);
		if (rc) {
			pr_err("Write fg address 0x%x failed, rc = %d\n",
						address, rc);
			return rc;
		}
	}

	return 0;
}

static int smb1360_otp_gain_config(struct smb1360_chip *chip, int gain_factor)
{
	int rc = 0;

	rc = smb1360_enable_fg_access(chip);
	if (rc) {
		pr_err("Couldn't request FG access rc = %d\n", rc);
		return rc;
	}
	chip->fg_access_type = FG_ACCESS_CFG;

	rc = smb1360_select_fg_i2c_address(chip);
	if (rc) {
		pr_err("Unable to set FG access I2C address\n");
		goto restore_fg;
	}

	if (chip->rsense_10mohm || chip->otg_fet_present) {
		rc = smb1360_adjust_current_gain(chip, gain_factor);
		if (rc) {
			pr_err("Unable to modify current gain rc=%d\n", rc);
			goto restore_fg;
		}
	}
	if (chip->otp_rslow_config) {
		rc = smb1360_otp_rslow_cfg(chip);
		if (rc) {
			pr_err("Unable configure OTP for rlsow rc=%d\n", rc);
			goto restore_fg;
		}
	}

	if (chip->otp_jeita_hard_config) {
		rc = smb1360_otp_jeita_hard_config(chip);
		if (rc) {
			pr_err("Unable configure OTP for jeita rc=%d\n", rc);
			goto restore_fg;
		}
	}

	rc = smb1360_masked_write(chip, CFG_FG_BATT_CTRL_REG,
			CFG_FG_OTP_BACK_UP_ENABLE, CFG_FG_OTP_BACK_UP_ENABLE);
	if (rc) {
		pr_err("Write reg 0x0E failed, rc = %d\n", rc);
		goto restore_fg;
	}

restore_fg:
	rc = smb1360_disable_fg_access(chip);
	if (rc) {
		pr_err("Couldn't disable FG access rc = %d\n", rc);
		return rc;
	}

	return rc;
}

static int smb1360_otg_disable(struct smb1360_chip *chip)
{
	int rc;

	rc = smb1360_masked_write(chip, CMD_CHG_REG, CMD_OTG_EN_BIT, 0);
	if (rc) {
		pr_err("Couldn't disable OTG mode rc=%d\n", rc);
		return rc;
	}

	mutex_lock(&chip->otp_gain_lock);
	/* Disable current gain configuration */
	if (chip->otg_fet_present && chip->fet_gain_enabled) {
		/* Disable FET */
		gpio_set_value(chip->otg_fet_enable_gpio, 1);
		rc = smb1360_otp_gain_config(chip, 0);
		if (rc < 0)
			pr_err("Couldn't config OTP gain config rc=%d\n", rc);
		else
			chip->fet_gain_enabled = false;
	}
	mutex_unlock(&chip->otp_gain_lock);

	return rc;
}

static int otg_fail_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	int rc;

	pr_debug("OTG Failed stat=%d\n", rt_stat);
	rc = smb1360_otg_disable(chip);
	if (rc)
		pr_err("Couldn't disable OTG mode rc=%d\n", rc);

	return 0;
}

static int otg_oc_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	int rc;

	pr_debug("OTG over-current stat=%d\n", rt_stat);
	rc = smb1360_otg_disable(chip);
	if (rc)
		pr_err("Couldn't disable OTG mode rc=%d\n", rc);

	return 0;
}

struct smb_irq_info {
	const char		*name;
	int			(*smb_irq)(struct smb1360_chip *chip,
							u8 rt_stat);
	int			high;
	int			low;
};

struct irq_handler_info {
	u8			stat_reg;
	u8			val;
	u8			prev_val;
	struct smb_irq_info	irq_info[4];
};

static struct irq_handler_info handlers[] = {
	{IRQ_A_REG, 0, 0,
		{
			{
				.name		= "cold_soft",
				.smb_irq	= cold_soft_handler,
			},
			{
				.name		= "hot_soft",
				.smb_irq	= hot_soft_handler,
			},
			{
				.name		= "cold_hard",
				.smb_irq	= cold_hard_handler,
			},
			{
				.name		= "hot_hard",
				.smb_irq	= hot_hard_handler,
			},
		},
	},
	{IRQ_B_REG, 0, 0,
		{
			{
				.name		= "chg_hot",
				.smb_irq	= chg_hot_handler,
			},
			{
				.name		= "vbat_low",
				.smb_irq	= vbat_low_handler,
			},
			{
				.name		= "battery_missing",
				.smb_irq	= battery_missing_handler,
			},
			{
				.name		= "battery_missing",
				.smb_irq	= battery_missing_handler,
			},
		},
	},
	{IRQ_C_REG, 0, 0,
		{
			{
				.name		= "chg_term",
				.smb_irq	= chg_term_handler,
			},
			{
				.name		= "taper",
			},
			{
				.name		= "recharge",
			},
			{
				.name		= "fast_chg",
				.smb_irq	= chg_fastchg_handler,
			},
		},
	},
	{IRQ_D_REG, 0, 0,
		{
			{
				.name		= "prechg_timeout",
			},
			{
				.name		= "safety_timeout",
			},
			{
				.name		= "aicl_done",
				.smb_irq	= aicl_done_handler,
			},
			{
				.name		= "battery_ov",
			},
		},
	},
	{IRQ_E_REG, 0, 0,
		{
			{
				.name		= "usbin_uv",
				.smb_irq	= usbin_uv_handler,
			},
			{
				.name		= "usbin_ov",
			},
			{
				.name		= "unused",
			},
			{
				.name		= "chg_inhibit",
				.smb_irq	= chg_inhibit_handler,
			},
		},
	},
	{IRQ_F_REG, 0, 0,
		{
			{
				.name		= "power_ok",
			},
			{
				.name		= "unused",
			},
			{
				.name		= "otg_fail",
				.smb_irq	= otg_fail_handler,
			},
			{
				.name		= "otg_oc",
				.smb_irq	= otg_oc_handler,
			},
		},
	},
	{IRQ_G_REG, 0, 0,
		{
			{
				.name		= "delta_soc",
				.smb_irq	= delta_soc_handler,
			},
			{
				.name		= "chg_error",
			},
			{
				.name		= "wd_timeout",
			},
			{
				.name		= "unused",
			},
		},
	},
	{IRQ_H_REG, 0, 0,
		{
			{
				.name		= "min_soc",
				.smb_irq	= min_soc_handler,
			},
			{
				.name		= "max_soc",
			},
			{
				.name		= "empty_soc",
				.smb_irq	= empty_soc_handler,
			},
			{
				.name		= "full_soc",
				.smb_irq	= full_soc_handler,
			},
		},
	},
	{IRQ_I_REG, 0, 0,
		{
			{
				.name		= "fg_access_allowed",
				.smb_irq	= fg_access_allowed_handler,
			},
			{
				.name		= "fg_data_recovery",
			},
			{
				.name		= "batt_id_complete",
				.smb_irq	= batt_id_complete_handler,
			},
		},
	},
};

#define IRQ_LATCHED_MASK	0x02
#define IRQ_STATUS_MASK		0x01
#define BATT_ID_LATCHED_MASK	0x08
#define BATT_ID_STATUS_MASK	0x07
#define BITS_PER_IRQ		2
static irqreturn_t smb1360_stat_handler(int irq, void *dev_id)
{
	struct smb1360_chip *chip = dev_id;
	int i, j;
	u8 triggered;
	u8 changed;
	u8 rt_stat, prev_rt_stat, irq_latched_mask, irq_status_mask;
	int rc;
	int handler_count = 0;

	mutex_lock(&chip->irq_complete);
	chip->irq_waiting = true;
	if (!chip->resume_completed) {
		dev_dbg(chip->dev, "IRQ triggered before device-resume\n");
		if (!chip->irq_disabled) {
			disable_irq_nosync(irq);
			chip->irq_disabled = true;
		}
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	chip->irq_waiting = false;

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		rc = smb1360_read(chip, handlers[i].stat_reg,
					&handlers[i].val);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't read %d rc = %d\n",
					handlers[i].stat_reg, rc);
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
			if (handlers[i].stat_reg == IRQ_I_REG && j == 2) {
				irq_latched_mask = BATT_ID_LATCHED_MASK;
				irq_status_mask = BATT_ID_STATUS_MASK;
			} else {
				irq_latched_mask = IRQ_LATCHED_MASK;
				irq_status_mask = IRQ_STATUS_MASK;
			}
			triggered = handlers[i].val
			       & (irq_latched_mask << (j * BITS_PER_IRQ));
			rt_stat = handlers[i].val
				& (irq_status_mask << (j * BITS_PER_IRQ));
			prev_rt_stat = handlers[i].prev_val
				& (irq_status_mask << (j * BITS_PER_IRQ));
			changed = prev_rt_stat ^ rt_stat;

			if (triggered || changed)
				rt_stat ? handlers[i].irq_info[j].high++ :
						handlers[i].irq_info[j].low++;

			if ((triggered || changed)
				&& handlers[i].irq_info[j].smb_irq != NULL) {
				handler_count++;
				rc = handlers[i].irq_info[j].smb_irq(chip,
								rt_stat);
				if (rc < 0)
					dev_err(chip->dev,
						"Couldn't handle %d irq for reg 0x%02x rc = %d\n",
						j, handlers[i].stat_reg, rc);
			}
		}
		handlers[i].prev_val = handlers[i].val;
	}

	pr_debug("handler count = %d\n", handler_count);
	if (handler_count)
		power_supply_changed(&chip->batt_psy);

	mutex_unlock(&chip->irq_complete);

	return IRQ_HANDLED;
}

static int show_irq_count(struct seq_file *m, void *data)
{
	int i, j, total = 0;

	for (i = 0; i < ARRAY_SIZE(handlers); i++)
		for (j = 0; j < 4; j++) {
			if (!handlers[i].irq_info[j].name)
				continue;
			seq_printf(m, "%s=%d\t(high=%d low=%d)\n",
						handlers[i].irq_info[j].name,
						handlers[i].irq_info[j].high
						+ handlers[i].irq_info[j].low,
						handlers[i].irq_info[j].high,
						handlers[i].irq_info[j].low);
			total += (handlers[i].irq_info[j].high
					+ handlers[i].irq_info[j].low);
		}

	seq_printf(m, "\n\tTotal = %d\n", total);

	return 0;
}

static int irq_count_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1360_chip *chip = inode->i_private;

	return single_open(file, show_irq_count, chip);
}

static const struct file_operations irq_count_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= irq_count_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int get_reg(void *data, u64 *val)
{
	struct smb1360_chip *chip = data;
	int rc;
	u8 temp;

	rc = smb1360_read(chip, chip->peek_poke_address, &temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read reg %x rc = %d\n",
			chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	struct smb1360_chip *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = smb1360_write(chip, chip->peek_poke_address, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't write 0x%02x to 0x%02x rc= %d\n",
			chip->peek_poke_address, temp, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");

static int fg_get_reg(void *data, u64 *val)
{
	struct smb1360_chip *chip = data;
	int rc;
	u8 temp;

	rc = smb1360_select_fg_i2c_address(chip);
	if (rc) {
		pr_err("Unable to set FG access I2C address\n");
		return -EINVAL;
	}

	rc = smb1360_fg_read(chip, chip->fg_peek_poke_address, &temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read reg %x rc = %d\n",
			chip->fg_peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int fg_set_reg(void *data, u64 val)
{
	struct smb1360_chip *chip = data;
	int rc;
	u8 temp;

	rc = smb1360_select_fg_i2c_address(chip);
	if (rc) {
		pr_err("Unable to set FG access I2C address\n");
		return -EINVAL;
	}

	temp = (u8) val;
	rc = smb1360_fg_write(chip, chip->fg_peek_poke_address, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't write 0x%02x to 0x%02x rc= %d\n",
			chip->fg_peek_poke_address, temp, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fg_poke_poke_debug_ops, fg_get_reg,
				fg_set_reg, "0x%02llx\n");

#define LAST_CNFG_REG	0x17
static int show_cnfg_regs(struct seq_file *m, void *data)
{
	struct smb1360_chip *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb1360_read(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1360_chip *chip = inode->i_private;

	return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cnfg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_CMD_REG	0x40
#define LAST_CMD_REG	0x42
static int show_cmd_regs(struct seq_file *m, void *data)
{
	struct smb1360_chip *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb1360_read(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cmd_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1360_chip *chip = inode->i_private;

	return single_open(file, show_cmd_regs, chip);
}

static const struct file_operations cmd_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cmd_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_STATUS_REG	0x48
#define LAST_STATUS_REG		0x4B
static int show_status_regs(struct seq_file *m, void *data)
{
	struct smb1360_chip *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = smb1360_read(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int status_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1360_chip *chip = inode->i_private;

	return single_open(file, show_status_regs, chip);
}

static const struct file_operations status_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= status_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_IRQ_REG		0x50
#define LAST_IRQ_REG		0x58
static int show_irq_stat_regs(struct seq_file *m, void *data)
{
	struct smb1360_chip *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_IRQ_REG; addr <= LAST_IRQ_REG; addr++) {
		rc = smb1360_read(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int irq_stat_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1360_chip *chip = inode->i_private;

	return single_open(file, show_irq_stat_regs, chip);
}

static const struct file_operations irq_stat_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= irq_stat_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int data_8(u8 *reg)
{
	return reg[0];
}
static int data_16(u8 *reg)
{
	return (reg[1] << 8) | reg[0];
}
static int data_24(u8 *reg)
{
	return  (reg[2] << 16) | (reg[1] << 8) | reg[0];
}
static int data_28(u8 *reg)
{
	return  ((reg[3] & 0xF) << 24) | (reg[2] << 16) |
					(reg[1] << 8) | reg[0];
}
static int data_32(u8 *reg)
{
	return  (reg[3]  << 24) | (reg[2] << 16) |
				(reg[1] << 8) | reg[0];
}

struct fg_regs {
	int index;
	int length;
	char *param_name;
	int (*calc_func) (u8 *);
};

static struct fg_regs fg_scratch_pad[] = {
	{0, 2, "v_current_predicted", data_16},
	{2, 2, "v_cutoff_predicted", data_16},
	{4, 2, "v_full_predicted", data_16},
	{6, 2, "ocv_estimate", data_16},
	{8, 2, "rslow_drop", data_16},
	{10, 2, "voltage_old", data_16},
	{12, 2, "current_old", data_16},
	{14, 4, "current_average_full", data_32},
	{18, 2, "temperature", data_16},
	{20, 2, "temp_last_track", data_16},
	{22, 2, "ESR_nominal", data_16},
	{26, 2, "Rslow", data_16},
	{28, 2, "counter_imptr", data_16},
	{30, 2, "counter_pulse", data_16},
	{32, 1, "IRQ_delta_prev", data_8},
	{33, 1, "cap_learning_counter", data_8},
	{34, 4, "Vact_int_error", data_32},
	{38, 3, "SOC_cutoff", data_24},
	{41, 3, "SOC_full", data_24},
	{44, 3, "SOC_auto_rechrge_temp", data_24},
	{47, 3, "Battery_SOC", data_24},
	{50, 4, "CC_SOC", data_28},
	{54, 2, "SOC_filtered", data_16},
	{56, 2, "SOC_Monotonic", data_16},
	{58, 2, "CC_SOC_coeff", data_16},
	{60, 2, "nominal_capacity", data_16},
	{62, 2, "actual_capacity", data_16},
	{68, 1, "temperature_counter", data_8},
	{69, 3, "Vbatt_filtered", data_24},
	{72, 3, "Ibatt_filtered", data_24},
	{75, 2, "Current_CC_shadow", data_16},
	{79, 2, "Ibatt_standby", data_16},
	{82, 1, "Auto_recharge_SOC_threshold", data_8},
	{83, 2, "System_cutoff_voltage", data_16},
	{85, 2, "System_CC_to_CV_voltage", data_16},
	{87, 2, "System_term_current", data_16},
	{89, 2, "System_fake_term_current", data_16},
	{91, 2, "thermistor_c1_coeff", data_16},
};

static struct fg_regs fg_cfg[] = {
	{0, 2, "ESR_actual", data_16},
	{4, 1, "IRQ_SOC_max", data_8},
	{5, 1, "IRQ_SOC_min", data_8},
	{6, 1, "IRQ_volt_empty", data_8},
	{7, 1, "Temp_external", data_8},
	{8, 1, "IRQ_delta_threshold", data_8},
	{9, 1, "JIETA_soft_cold", data_8},
	{10, 1, "JIETA_soft_hot", data_8},
	{11, 1, "IRQ_volt_min", data_8},
	{14, 2, "ESR_sys_replace", data_16},
};

static struct fg_regs fg_shdw[] = {
	{0, 1, "Latest_battery_info", data_8},
	{1, 1, "Latest_Msys_SOC", data_8},
	{2, 2, "Battery_capacity", data_16},
	{4, 2, "Rslow_drop", data_16},
	{6, 1, "Latest_SOC", data_8},
	{7, 1, "Latest_Cutoff_SOC", data_8},
	{8, 1, "Latest_full_SOC", data_8},
	{9, 2, "Voltage_shadow", data_16},
	{11, 2, "Current_shadow", data_16},
	{13, 2, "Latest_temperature", data_16},
	{15, 1, "Latest_system_sbits", data_8},
};

#define FIRST_FG_CFG_REG		0x20
#define LAST_FG_CFG_REG			0x2F
#define FIRST_FG_SHDW_REG		0x60
#define LAST_FG_SHDW_REG		0x6F
#define FG_SCRATCH_PAD_MAX		93
#define FG_SCRATCH_PAD_BASE_REG		0x80
#define SMB1360_I2C_READ_LENGTH		32

static int smb1360_check_cycle_stretch(struct smb1360_chip *chip)
{
	int rc = 0;
	u8 reg;

	rc = smb1360_read(chip, STATUS_4_REG, &reg);
	if (rc) {
		pr_err("Unable to read status regiseter\n");
	} else if (reg & CYCLE_STRETCH_ACTIVE_BIT) {
		/* clear cycle stretch */
		rc = smb1360_masked_write(chip, CMD_I2C_REG,
			CYCLE_STRETCH_CLEAR_BIT, CYCLE_STRETCH_CLEAR_BIT);
		if (rc)
			pr_err("Unable to clear cycle stretch\n");
	}

	return rc;
}

static int show_fg_regs(struct seq_file *m, void *data)
{
	struct smb1360_chip *chip = m->private;
	int rc, i , j, rem_length;
	u8 reg[FG_SCRATCH_PAD_MAX];

	rc = smb1360_check_cycle_stretch(chip);
	if (rc)
		pr_err("Unable to check cycle-stretch\n");

	rc = smb1360_enable_fg_access(chip);
	if (rc) {
		pr_err("Couldn't request FG access rc=%d\n", rc);
		return rc;
	}

	for (i = 0; i < (FG_SCRATCH_PAD_MAX / SMB1360_I2C_READ_LENGTH); i++) {
		j = i * SMB1360_I2C_READ_LENGTH;
		rc = smb1360_read_bytes(chip, FG_SCRATCH_PAD_BASE_REG + j,
					&reg[j], SMB1360_I2C_READ_LENGTH);
		if (rc) {
			pr_err("Couldn't read scratch registers rc=%d\n", rc);
			break;
		}
	}

	j = i * SMB1360_I2C_READ_LENGTH;
	rem_length = (FG_SCRATCH_PAD_MAX % SMB1360_I2C_READ_LENGTH);
	if (rem_length) {
		rc = smb1360_read_bytes(chip, FG_SCRATCH_PAD_BASE_REG + j,
						&reg[j], rem_length);
		if (rc)
			pr_err("Couldn't read scratch registers rc=%d\n", rc);
	}

	rc = smb1360_disable_fg_access(chip);
	if (rc) {
		pr_err("Couldn't disable FG access rc=%d\n", rc);
		return rc;
	}

	rc = smb1360_check_cycle_stretch(chip);
	if (rc)
		pr_err("Unable to check cycle-stretch\n");


	seq_puts(m, "FG scratch-pad registers\n");
	for (i = 0; i < ARRAY_SIZE(fg_scratch_pad); i++)
		seq_printf(m, "\t%s = %x\n", fg_scratch_pad[i].param_name,
		fg_scratch_pad[i].calc_func(&reg[fg_scratch_pad[i].index]));

	rem_length = LAST_FG_CFG_REG - FIRST_FG_CFG_REG + 1;
	rc = smb1360_read_bytes(chip, FIRST_FG_CFG_REG,
					&reg[0], rem_length);
	if (rc)
		pr_err("Couldn't read config registers rc=%d\n", rc);

	seq_puts(m, "FG config registers\n");
	for (i = 0; i < ARRAY_SIZE(fg_cfg); i++)
		seq_printf(m, "\t%s = %x\n", fg_cfg[i].param_name,
				fg_cfg[i].calc_func(&reg[fg_cfg[i].index]));

	rem_length = LAST_FG_SHDW_REG - FIRST_FG_SHDW_REG + 1;
	rc = smb1360_read_bytes(chip, FIRST_FG_SHDW_REG,
					&reg[0], rem_length);
	if (rc)
		pr_err("Couldn't read shadow registers rc=%d\n", rc);

	seq_puts(m, "FG shadow registers\n");
	for (i = 0; i < ARRAY_SIZE(fg_shdw); i++)
		seq_printf(m, "\t%s = %x\n", fg_shdw[i].param_name,
				fg_shdw[i].calc_func(&reg[fg_shdw[i].index]));

	return rc;
}

static int fg_regs_open(struct inode *inode, struct file *file)
{
	struct smb1360_chip *chip = inode->i_private;

	return single_open(file, show_fg_regs, chip);
}

static const struct file_operations fg_regs_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= fg_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int smb1360_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb1360_chip *chip = rdev_get_drvdata(rdev);

	rc = smb1360_masked_write(chip, CMD_CHG_REG, CMD_OTG_EN_BIT,
						CMD_OTG_EN_BIT);
	if (rc) {
		pr_err("Couldn't enable  OTG mode rc=%d\n", rc);
		return rc;
	}

	pr_debug("OTG mode enabled\n");
	/* Enable current gain configuration */
	mutex_lock(&chip->otp_gain_lock);
	if (chip->otg_fet_present) {
		/* Enable FET */
		gpio_set_value(chip->otg_fet_enable_gpio, 0);
		rc = smb1360_otp_gain_config(chip, 3);
		if (rc < 0)
			pr_err("Couldn't config OTP gain config rc=%d\n", rc);
		else
			chip->fet_gain_enabled = true;
	}
	mutex_unlock(&chip->otp_gain_lock);

	return rc;
}

static int smb1360_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb1360_chip *chip = rdev_get_drvdata(rdev);

	rc = smb1360_otg_disable(chip);
	if (rc)
		pr_err("Couldn't disable OTG regulator rc=%d\n", rc);

	pr_debug("OTG mode disabled\n");
	return rc;
}

static int smb1360_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	u8 reg = 0;
	int rc = 0;
	struct smb1360_chip *chip = rdev_get_drvdata(rdev);

	rc = smb1360_read(chip, CMD_CHG_REG, &reg);
	if (rc) {
		pr_err("Couldn't read OTG enable bit rc=%d\n", rc);
		return rc;
	}

	return  (reg & CMD_OTG_EN_BIT) ? 1 : 0;
}

struct regulator_ops smb1360_otg_reg_ops = {
	.enable		= smb1360_otg_regulator_enable,
	.disable	= smb1360_otg_regulator_disable,
	.is_enabled	= smb1360_otg_regulator_is_enable,
};

static int smb1360_regulator_init(struct smb1360_chip *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smb1360_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
					&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}
int check_CSIR_function(void);

static int smb1360_check_batt_profile(struct smb1360_chip *chip)
{
	int rc, i, timeout = 50;
	u8 reg = 0, loaded_profile, new_profile = 0, bid_mask;

	/*if (!chip->connected_rid) {
		pr_debug("Skip batt-profile loading connected_rid=%d\n",
						chip->connected_rid);
		return 0;
	}*/

	rc = smb1360_read(chip, SHDW_FG_BATT_STATUS, &reg);
	if (rc) {
		pr_err("Couldn't read FG_BATT_STATUS rc=%d\n", rc);
		return rc;
	}
	loaded_profile = !!(reg & BATTERY_PROFILE_BIT) ?
			BATTERY_PROFILE_B : BATTERY_PROFILE_A;

	pr_info("fg_batt_status=%x loaded_profile=%d\n", reg, loaded_profile);

	for (i = 0; i < BATTERY_PROFILE_MAX; i++) {
		pr_debug("profile=%d profile_rid=%d connected_rid=%d\n", i,
						chip->profile_rid[i],
						chip->connected_rid);
		if (abs(chip->profile_rid[i] - chip->connected_rid) <
				(div_u64(chip->connected_rid, 10)))
			break;
	}

	/*if (i == BATTERY_PROFILE_MAX) {
		pr_err("None of the battery-profiles match the connected-RID\n");
		return 0;
	} else {
		if (i == loaded_profile) {
			pr_debug("Loaded Profile-RID == connected-RID\n");
			return 0;
		} else {
			new_profile = (loaded_profile == BATTERY_PROFILE_A) ?
					BATTERY_PROFILE_B : BATTERY_PROFILE_A;
			bid_mask = (new_profile == BATTERY_PROFILE_A) ?
					BATT_PROFILEA_MASK : BATT_PROFILEB_MASK;
			pr_info("Loaded Profile-RID != connected-RID, switch-profile old_profile=%d new_profile=%d\n",
						loaded_profile, new_profile);
		}
	}*/
	if(loaded_profile == 0)
	{
		if(check_CSIR_function() == 0x2334 )
		{
			pr_info("loaded profile is profile A ,skip ! \n");
			return 0;
		}
		else
		{
			pr_info("must load battery profile B! \n");
			bid_mask = BATT_PROFILEB_MASK;
		}
	}
	else
	{	if(check_CSIR_function() == 0x2334 )
		{
			pr_info("must load battery profile A! \n");
			bid_mask = BATT_PROFILEA_MASK;
		}
		else
		{
			pr_info("loaded profile is profileB ,skip ! \n");
			return 0;
		}
	}
	/* set the BID mask */
	rc = smb1360_masked_write(chip, CFG_FG_BATT_CTRL_REG,
				BATT_PROFILE_SELECT_MASK, bid_mask);
	if (rc) {
		pr_err("Couldn't reset battery-profile rc=%d\n", rc);
		return rc;
	}

	rc = smb1360_enable_fg_access(chip);
	if (rc) {
		pr_err("FG access timed-out, rc = %d\n", rc);
		return rc;
	}
	/* delay after handshaking for profile-switch to continue */
	msleep(1500);

	rc = smb1360_force_fg_reset(chip);
	if (rc) {
		pr_err("Couldn't reset FG rc=%d\n", rc);
		goto restore_fg;
	}

	rc = smb1360_disable_fg_access(chip);
	if (rc) {
		pr_err("disable FG access failed, rc = %d\n", rc);
		return rc;
	}

	timeout = 10;
	while (timeout) {
		/* delay for profile to change */
		msleep(500);
		rc = smb1360_read(chip, SHDW_FG_BATT_STATUS, &reg);
		if (rc) {
			pr_err("Could't read FG_BATT_STATUS rc=%d\n", rc);
			return rc;
		}

		reg = !!(reg & BATTERY_PROFILE_BIT);
		if (reg == new_profile) {
			pr_info("New profile=%d loaded\n", new_profile);
			break;
		}
		timeout--;
	}

	if (!timeout) {
		pr_err("New profile could not be loaded\n");
		return -EBUSY;
	}

	return 0;

restore_fg:
	smb1360_disable_fg_access(chip);
	return rc;
}

#define UPDATE_IRQ_STAT(irq_reg, value) \
		handlers[irq_reg - IRQ_A_REG].prev_val = value;

static int determine_initial_status(struct smb1360_chip *chip)
{
	int rc;
	u8 reg = 0;

	/*
	 * It is okay to read the IRQ status as the irq's are
	 * not registered yet.
	 */
	chip->batt_present = true;
	rc = smb1360_read(chip, IRQ_B_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read IRQ_B_REG rc = %d\n", rc);
		return rc;
	}
	UPDATE_IRQ_STAT(IRQ_B_REG, reg);

	if (reg & IRQ_B_BATT_TERMINAL_BIT || reg & IRQ_B_BATT_MISSING_BIT)
		chip->batt_present = false;

	rc = smb1360_read(chip, IRQ_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_C_REG rc = %d\n", rc);
		return rc;
	}
	UPDATE_IRQ_STAT(IRQ_C_REG, reg);

	if (reg & IRQ_C_CHG_TERM)
		chip->batt_full = true;

	rc = smb1360_read(chip, IRQ_A_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read irq A rc = %d\n", rc);
		return rc;
	}
	UPDATE_IRQ_STAT(IRQ_A_REG, reg);

	if (chip->workaround_flags & WRKRND_HARD_JEITA) {
		schedule_delayed_work(&chip->jeita_work, 0);
	} else {
		if (reg & IRQ_A_HOT_HARD_BIT)
			chip->batt_hot = true;
		if (reg & IRQ_A_COLD_HARD_BIT)
			chip->batt_cold = true;
		if (!chip->config_hard_thresholds) {
			if (reg & IRQ_A_HOT_SOFT_BIT)
				chip->batt_warm = true;
			if (reg & IRQ_A_COLD_SOFT_BIT)
				chip->batt_cool = true;
		}
	}

	rc = smb1360_read(chip, IRQ_E_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read irq E rc = %d\n", rc);
		return rc;
	}
	UPDATE_IRQ_STAT(IRQ_E_REG, reg);

	chip->usb_present = (reg & IRQ_E_USBIN_UV_BIT) ? false : true;
	power_supply_set_present(chip->usb_psy, chip->usb_present);

	if(chip->usb_present == 1)
		focal_usb_detection(true);
	else
	{
		focal_usb_detection(false);
	}

	if(chip->usb_present==0 && g_Charger_mode)
	{
        bl_cmd[1] = 0x0;
        set_tcon_cmd(bl_cmd, ARRAY_SIZE(bl_cmd));
		//kernel_power_off();
		orderly_poweroff(true);
	}

	return 0;
}

static int smb1360_fg_config(struct smb1360_chip *chip)
{
	int rc = 0, temp, fcc_mah;
	u8 reg = 0, reg2[2];

	if (chip->fg_reset_at_pon) {
		int v_predicted, v_now;

		rc = smb1360_enable_fg_access(chip);
		if (rc) {
			pr_err("Couldn't enable FG access rc=%d\n", rc);
			return rc;
		}

		rc = smb1360_read_bytes(chip, VOLTAGE_PREDICTED_REG, reg2, 2);
		if (rc) {
			pr_err("Failed to read VOLTAGE_PREDICTED rc=%d\n", rc);
			goto disable_fg_reset;
		}
		v_predicted = (reg2[1] << 8) | reg2[0];
		v_predicted = div_u64(v_predicted * 5000, 0x7FFF);

		rc = smb1360_read_bytes(chip, SHDW_FG_VTG_NOW, reg2, 2);
		if (rc) {
			pr_err("Failed to read SHDW_FG_VTG_NOW rc=%d\n", rc);
			goto disable_fg_reset;
		}
		v_now = (reg2[1] << 8) | reg2[0];
		v_now = div_u64(v_now * 5000, 0x7FFF);

		pr_debug("v_predicted=%d v_now=%d reset_threshold=%d\n",
			v_predicted, v_now, chip->fg_reset_threshold_mv);

		/*
		 * Reset FG if the predicted voltage is off wrt
		 * the real-time voltage.
		 */
		temp = abs(v_predicted - v_now);
		if (temp >= chip->fg_reset_threshold_mv) {
			pr_info("Reseting FG - v_delta=%d threshold=%d\n",
					temp, chip->fg_reset_threshold_mv);
			/* delay for the FG access to settle */
			msleep(1500);
			rc = smb1360_force_fg_reset(chip);
			if (rc) {
				pr_err("Couldn't reset FG rc=%d\n", rc);
				goto disable_fg_reset;
			}
		}
disable_fg_reset:
		smb1360_disable_fg_access(chip);
	}

	/*
	 * The below IRQ thresholds are not accessible in REV_1
	 * of SMB1360.
	 */
	if (!(chip->workaround_flags & WRKRND_FG_CONFIG_FAIL)) {
		if (chip->delta_soc != -EINVAL) {
			reg = abs(((chip->delta_soc * MAX_8_BITS) / 100) - 1);
			pr_debug("delta_soc=%d reg=%x\n", chip->delta_soc, reg);
			rc = smb1360_write(chip, SOC_DELTA_REG, reg);
			if (rc) {
				dev_err(chip->dev, "Couldn't write to SOC_DELTA_REG rc=%d\n",
						rc);
				return rc;
			}
		}

		if (chip->soc_min != -EINVAL) {
			if (is_between(chip->soc_min, 0, 100)) {
				reg = DIV_ROUND_UP(chip->soc_min * MAX_8_BITS,
									100);
				pr_debug("soc_min=%d reg=%x\n",
						chip->soc_min, reg);
				rc = smb1360_write(chip, SOC_MIN_REG, reg);
				if (rc) {
					dev_err(chip->dev, "Couldn't write to SOC_MIN_REG rc=%d\n",
							rc);
					return rc;
				}
			}
		}

		if (chip->soc_max != -EINVAL) {
			if (is_between(chip->soc_max, 0, 100)) {
				reg = DIV_ROUND_UP(chip->soc_max * MAX_8_BITS,
									100);
				pr_debug("soc_max=%d reg=%x\n",
						chip->soc_max, reg);
				rc = smb1360_write(chip, SOC_MAX_REG, reg);
				if (rc) {
					dev_err(chip->dev, "Couldn't write to SOC_MAX_REG rc=%d\n",
							rc);
					return rc;
				}
			}
		}

		if (chip->voltage_min_mv != -EINVAL) {
			temp = (chip->voltage_min_mv - 2500) * MAX_8_BITS;
			reg = DIV_ROUND_UP(temp, 2500);
			pr_debug("voltage_min=%d reg=%x\n",
					chip->voltage_min_mv, reg);
			rc = smb1360_write(chip, VTG_MIN_REG, reg);
			if (rc) {
				dev_err(chip->dev, "Couldn't write to VTG_MIN_REG rc=%d\n",
							rc);
				return rc;
			}
		}

		if (chip->voltage_empty_mv != -EINVAL) {
			temp = (chip->voltage_empty_mv - 2500) * MAX_8_BITS;
			reg = DIV_ROUND_UP(temp, 2500);
			pr_debug("voltage_empty=%d reg=%x\n",
					chip->voltage_empty_mv, reg);
			rc = smb1360_write(chip, VTG_EMPTY_REG, reg);
			if (rc) {
				dev_err(chip->dev, "Couldn't write to VTG_EMPTY_REG rc=%d\n",
							rc);
				return rc;
			}
		}
	}

	/* scratch-pad register config */
	if (chip->batt_capacity_mah != -EINVAL
		|| chip->v_cutoff_mv != -EINVAL
		|| chip->fg_iterm_ma != -EINVAL
		|| chip->fg_ibatt_standby_ma != -EINVAL
		|| chip->fg_thermistor_c1_coeff != -EINVAL
		|| chip->fg_cc_to_cv_mv != -EINVAL
		|| chip->fg_auto_recharge_soc != -EINVAL) {

		rc = smb1360_enable_fg_access(chip);
		if (rc) {
			pr_err("Couldn't enable FG access rc=%d\n", rc);
			return rc;
		}

		/* Update battery capacity */
		if (chip->batt_capacity_mah != -EINVAL) {
			rc = smb1360_read_bytes(chip, ACTUAL_CAPACITY_REG,
								reg2, 2);
			if (rc) {
				pr_err("Failed to read ACTUAL CAPACITY rc=%d\n",
									rc);
				goto disable_fg;
			}
			fcc_mah = (reg2[1] << 8) | reg2[0];
			if (fcc_mah == chip->batt_capacity_mah) {
				pr_debug("battery capacity correct\n");
			} else {
				/* Update the battery capacity */
				reg2[1] =
					(chip->batt_capacity_mah & 0xFF00) >> 8;
				reg2[0] = (chip->batt_capacity_mah & 0xFF);
				rc = smb1360_write_bytes(chip,
					ACTUAL_CAPACITY_REG, reg2, 2);
				if (rc) {
					pr_err("Couldn't write batt-capacity rc=%d\n",
									rc);
					goto disable_fg;
				}
				rc = smb1360_write_bytes(chip,
					NOMINAL_CAPACITY_REG, reg2, 2);
				if (rc) {
					pr_err("Couldn't write batt-capacity rc=%d\n",
									rc);
					goto disable_fg;
				}

				/* Update CC to SOC COEFF */
				if (chip->cc_soc_coeff != -EINVAL) {
					reg2[1] =
					(chip->cc_soc_coeff & 0xFF00) >> 8;
					reg2[0] = (chip->cc_soc_coeff & 0xFF);
					rc = smb1360_write_bytes(chip,
						CC_TO_SOC_COEFF, reg2, 2);
					if (rc) {
						pr_err("Couldn't write cc_soc_coeff rc=%d\n",
									rc);
						goto disable_fg;
					}
				}
			}
		}

		/* Update cutoff voltage for SOC = 0 */
		if (chip->v_cutoff_mv != -EINVAL) {
			temp = (u16) div_u64(chip->v_cutoff_mv * 0x7FFF, 5000);
			reg2[1] = (temp & 0xFF00) >> 8;
			reg2[0] = temp & 0xFF;
			rc = smb1360_write_bytes(chip, FG_SYS_CUTOFF_V_REG,
								reg2, 2);
			if (rc) {
				pr_err("Couldn't write cutoff_mv rc=%d\n", rc);
				goto disable_fg;
			}
		}

		/*
		 * Update FG iterm for SOC = 100, this value is always assumed
		 * to be -ve
		 */
		if (chip->fg_iterm_ma != -EINVAL) {
			int iterm = chip->fg_iterm_ma * -1;
			temp = (s16) div_s64(iterm * 0x7FFF, 2500);
			reg2[1] = (temp & 0xFF00) >> 8;
			reg2[0] = temp & 0xFF;
			rc = smb1360_write_bytes(chip, FG_ITERM_REG,
							reg2, 2);
			if (rc) {
				pr_err("Couldn't write fg_iterm rc=%d\n", rc);
				goto disable_fg;
			}
		}

		/*
		 * Update FG iterm standby for SOC = 0, this value is always
		 * assumed to be +ve
		 */
		if (chip->fg_ibatt_standby_ma != -EINVAL) {
			int iterm = chip->fg_ibatt_standby_ma;
			temp = (u16) div_u64(iterm * 0x7FFF, 2500);
			reg2[1] = (temp & 0xFF00) >> 8;
			reg2[0] = temp & 0xFF;
			rc = smb1360_write_bytes(chip, FG_IBATT_STANDBY_REG,
								reg2, 2);
			if (rc) {
				pr_err("Couldn't write fg_iterm rc=%d\n", rc);
				goto disable_fg;
			}
		}

		/* Update CC_to_CV voltage threshold */
		if (chip->fg_cc_to_cv_mv != -EINVAL) {
			temp = (u16) div_u64(chip->fg_cc_to_cv_mv * 0x7FFF,
								5000);
			reg2[1] = (temp & 0xFF00) >> 8;
			reg2[0] = temp & 0xFF;
			rc = smb1360_write_bytes(chip, FG_CC_TO_CV_V_REG,
								reg2, 2);
			if (rc) {
				pr_err("Couldn't write cc_to_cv_mv rc=%d\n",
								rc);
				goto disable_fg;
			}
		}

		/* Update the thermistor c1 coefficient */
		if (chip->fg_thermistor_c1_coeff != -EINVAL) {
			reg2[1] = (chip->fg_thermistor_c1_coeff & 0xFF00) >> 8;
			reg2[0] = (chip->fg_thermistor_c1_coeff & 0xFF);
			rc = smb1360_write_bytes(chip, FG_THERM_C1_COEFF_REG,
								reg2, 2);
			if (rc) {
				pr_err("Couldn't write thermistor_c1_coeff rc=%d\n",
							rc);
				goto disable_fg;
			}
		}

		/* Update SoC based resume charging threshold */
		if (chip->fg_auto_recharge_soc != -EINVAL) {
			rc = smb1360_masked_write(chip, CFG_CHG_FUNC_CTRL_REG,
						CHG_RECHG_THRESH_FG_SRC_BIT,
						CHG_RECHG_THRESH_FG_SRC_BIT);
			if (rc) {
				dev_err(chip->dev, "Couldn't write to CFG_CHG_FUNC_CTRL_REG rc=%d\n",
									rc);
				goto disable_fg;
			}

			reg = DIV_ROUND_UP(chip->fg_auto_recharge_soc *
							MAX_8_BITS, 100);
			pr_debug("fg_auto_recharge_soc=%d reg=%x\n",
					chip->fg_auto_recharge_soc, reg);
			rc = smb1360_write(chip, FG_AUTO_RECHARGE_SOC, reg);
			if (rc) {
				dev_err(chip->dev, "Couldn't write to FG_AUTO_RECHARGE_SOC rc=%d\n",
									rc);
				goto disable_fg;
			}
		}

disable_fg:
		/* disable FG access */
		smb1360_disable_fg_access(chip);
	}

	return rc;
}

static void smb1360_check_feature_support(struct smb1360_chip *chip)
{

	if (is_usb100_broken(chip)) {
		pr_info("USB100 is not supported\n");
		chip->workaround_flags |= WRKRND_USB100_FAIL;
	}

	/*
	 * FG Configuration
	 *
	 * The REV_1 of the chip does not allow access to
	 * FG config registers (20-2FH). Set the workaround flag.
	 * Also, the battery detection does not work when the DCIN is absent,
	 * add a workaround flag for it.
	*/
	if (chip->revision == SMB1360_REV_1) {
		pr_debug("FG config and Battery detection is not supported\n");
		chip->workaround_flags |=
			WRKRND_FG_CONFIG_FAIL | WRKRND_BATT_DET_FAIL;
	}
}

static int smb1360_enable(struct smb1360_chip *chip, bool enable)
{
	int rc = 0;
	u8 val = 0, shdn_cmd_polar;

	rc = smb1360_read(chip, SHDN_CTRL_REG, &val);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read 0x1A reg rc = %d\n", rc);
		return rc;
	}

	/* Ignore if a CMD based shutdown is not enabled */
	if (!(val & SHDN_CMD_USE_BIT)) {
		pr_debug("SMB not configured for CMD based shutdown\n");
		return 0;
	}

	shdn_cmd_polar = !!(val & SHDN_CMD_POLARITY_BIT);
	val = (shdn_cmd_polar ^ enable) ? SHDN_CMD_BIT : 0;

	pr_debug("enable=%d shdn_polarity=%d value=%d\n", enable,
						shdn_cmd_polar, val);

	rc = smb1360_masked_write(chip, CMD_IL_REG, SHDN_CMD_BIT, val);
	if (rc < 0)
		pr_err("Couldn't shutdown smb1360 rc = %d\n", rc);

	return rc;
}

static inline int smb1360_poweroff(struct smb1360_chip *chip)
{
	pr_debug("power off smb1360\n");
	return smb1360_enable(chip, false);
}

static inline int smb1360_poweron(struct smb1360_chip *chip)
{
	pr_debug("power on smb1360\n");
	return smb1360_enable(chip, true);
}

static int smb1360_jeita_init(struct smb1360_chip *chip)
{
	int rc = 0;
	int temp;

	if (chip->config_hard_thresholds) {
		if (chip->soft_jeita_supported) {
			chip->workaround_flags |= WRKRND_HARD_JEITA;
			rc = smb1360_set_soft_jeita_threshold(chip,
			chip->cool_bat_decidegc, chip->warm_bat_decidegc);
			if (rc) {
				dev_err(chip->dev,
					"Couldn't set jeita threshold\n");
				return rc;
			}
		} else {
			rc = smb1360_set_soft_jeita_threshold(chip,
			chip->cold_bat_decidegc, chip->hot_bat_decidegc);
			if (rc) {
				dev_err(chip->dev,
					"Couldn't set jeita threshold\n");
				return rc;
			}
		}
	} else {
		if (chip->soft_jeita_supported) {
			temp = min(chip->warm_bat_ma, chip->cool_bat_ma);
			rc = smb1360_set_jeita_comp_curr(chip, temp);
			if (rc) {
				dev_err(chip->dev, "Couldn't set comp current\n");
				return rc;
			}

			temp = (chip->vfloat_mv - chip->warm_bat_mv) / 10;
			rc = smb1360_masked_write(chip, CFG_FVC_REG,
					FLT_VTG_COMP_MASK, temp);
			if (rc < 0) {
				dev_err(chip->dev, "Couldn't set VFLT compensation = %d",
									rc);
				return rc;
			}

			rc = smb1360_set_soft_jeita_threshold(chip,
			chip->cool_bat_decidegc, chip->warm_bat_decidegc);
			if (rc) {
				dev_err(chip->dev,
					"Couldn't set jeita threshold\n");
				return rc;
			}

			rc = smb1360_soft_jeita_comp_enable(chip, true);
			if (rc) {
				dev_err(chip->dev, "Couldn't enable jeita\n");
				return rc;
			}
		}
	}

	return rc;
}

static int smb1360_otp_gain_init(struct smb1360_chip *chip)
{
	int rc = 0, gain_factor;
	bool otp_gain_config = false;

	if (chip->rsense_10mohm) {
		gain_factor = 2;
		otp_gain_config = true;
	}

	if (chip->otp_jeita_hard_config)
		otp_gain_config = true;

	mutex_lock(&chip->otp_gain_lock);
	if (chip->otg_fet_present) {
		/*
		 * Reset current gain to the default value if OTG
		 * is not enabled
		 */
		if (!chip->fet_gain_enabled) {
			otp_gain_config = true;
			gain_factor = 0;
		}
	}

	if (otp_gain_config) {
		rc = smb1360_otp_gain_config(chip, gain_factor);
		if (rc < 0)
			pr_err("Couldn't config OTP gain rc=%d\n", rc);
	}
	mutex_unlock(&chip->otp_gain_lock);

	return rc;
}

static int smb1360_hw_init(struct smb1360_chip *chip)
{
	int rc;
	int i;
	u8 reg, mask;

	smb1360_check_feature_support(chip);

	rc = smb1360_enable_volatile_writes(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't configure for volatile rc = %d\n",
				rc);
		return rc;
	}

	/* Bring SMB1360 out of shutdown, if it was enabled by default */
	rc = smb1360_poweron(chip);
	if (rc < 0) {
		pr_err("smb1360 power on failed\n");
		return rc;
	} else {
		/*
		 * A 2 seconds delay is mandatory after bringing the chip out
		 * of shutdown. This guarantees that FG is in a proper state.
		 */
		schedule_delayed_work(&chip->delayed_init_work,
				msecs_to_jiffies(SMB1360_POWERON_DELAY_MS));
	}
	/*
	 * set chg en by cmd register, set chg en by writing bit 1,
	 * enable auto pre to fast
	 */
	rc = smb1360_masked_write(chip, CFG_CHG_MISC_REG,
					CHG_EN_BY_PIN_BIT
					| CHG_EN_ACTIVE_LOW_BIT
					| PRE_TO_FAST_REQ_CMD_BIT,
					0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set CFG_CHG_MISC_REG rc=%d\n", rc);
		return rc;
	}

	/* USB/AC pin settings */
	rc = smb1360_masked_write(chip, CFG_BATT_CHG_ICL_REG,
					AC_INPUT_ICL_PIN_BIT
					| AC_INPUT_PIN_HIGH_BIT
					| RESET_STATE_USB_500,
					AC_INPUT_PIN_HIGH_BIT
					| RESET_STATE_USB_500);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set CFG_BATT_CHG_ICL_REG rc=%d\n",
				rc);
		return rc;
	}

	/* AICL enable and set input-uv glitch flt to 20ms*/
	reg = AICL_ENABLED_BIT | INPUT_UV_GLITCH_FLT_20MS_BIT;
	rc = smb1360_masked_write(chip, CFG_GLITCH_FLT_REG, reg, reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set CFG_GLITCH_FLT_REG rc=%d\n",
				rc);
		return rc;
	}

	/* set the float voltage */
	if (chip->vfloat_mv != -EINVAL) {
		rc = smb1360_float_voltage_set(chip, chip->vfloat_mv);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
	}

	/* set iterm */
	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled) {
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");
			return -EINVAL;
		} else {
			if (chip->rsense_10mohm)
				chip->iterm_ma /= 2;

			if (chip->iterm_ma < 25)
				reg = CHG_ITERM_25MA;
			else if (chip->iterm_ma > 200)
				reg = CHG_ITERM_200MA;
			else
				reg = DIV_ROUND_UP(chip->iterm_ma, 25) - 1;

			rc = smb1360_masked_write(chip, CFG_BATT_CHG_REG,
						CHG_ITERM_MASK, reg);
			if (rc) {
				dev_err(chip->dev,
					"Couldn't set iterm rc = %d\n", rc);
				return rc;
			}

			rc = smb1360_masked_write(chip, CFG_CHG_MISC_REG,
					CHG_CURR_TERM_DIS_BIT, 0);
			if (rc) {
				dev_err(chip->dev,
					"Couldn't enable iterm rc = %d\n", rc);
				return rc;
			}
		}
	} else  if (chip->iterm_disabled) {
		rc = smb1360_masked_write(chip, CFG_CHG_MISC_REG,
						CHG_CURR_TERM_DIS_BIT,
						CHG_CURR_TERM_DIS_BIT);
		if (rc) {
			dev_err(chip->dev, "Couldn't set iterm rc = %d\n",
								rc);
			return rc;
		}
	}

	/* set the safety time voltage */
	if (chip->safety_time != -EINVAL) {
		if (chip->safety_time == 0) {
			/* safety timer disabled */
			rc = smb1360_masked_write(chip, CFG_SFY_TIMER_CTRL_REG,
			SAFETY_TIME_DISABLE_BIT, SAFETY_TIME_DISABLE_BIT);
			if (rc < 0) {
				dev_err(chip->dev,
				"Couldn't disable safety timer rc = %d\n",
								rc);
				return rc;
			}
		} else {
			for (i = 0; i < ARRAY_SIZE(chg_time); i++) {
				if (chip->safety_time <= chg_time[i]) {
					reg = i << SAFETY_TIME_MINUTES_SHIFT;
					break;
				}
			}
			rc = smb1360_masked_write(chip, CFG_SFY_TIMER_CTRL_REG,
			SAFETY_TIME_DISABLE_BIT | SAFETY_TIME_MINUTES_MASK,
								reg);
			if (rc < 0) {
				dev_err(chip->dev,
					"Couldn't set safety timer rc = %d\n",
									rc);
				return rc;
			}
		}
	}

	/* configure resume threshold, auto recharge and charge inhibit */
	if (chip->resume_delta_mv != -EINVAL) {
		if (chip->recharge_disabled && chip->chg_inhibit_disabled) {
			dev_err(chip->dev, "Error: Both recharge_disabled and recharge_mv set\n");
			return -EINVAL;
		} else {
			rc = smb1360_recharge_threshold_set(chip,
						chip->resume_delta_mv);
			if (rc) {
				dev_err(chip->dev,
					"Couldn't set rechg thresh rc = %d\n",
									rc);
				return rc;
			}
		}
	}

	rc = smb1360_masked_write(chip, CFG_CHG_MISC_REG,
					CFG_AUTO_RECHG_DIS_BIT,
					chip->recharge_disabled ?
					CFG_AUTO_RECHG_DIS_BIT : 0);
	if (rc) {
		dev_err(chip->dev, "Couldn't set rechg-cfg rc = %d\n", rc);
		return rc;
	}
	rc = smb1360_masked_write(chip, CFG_CHG_MISC_REG,
					CFG_CHG_INHIBIT_EN_BIT,
					chip->chg_inhibit_disabled ?
					0 : CFG_CHG_INHIBIT_EN_BIT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set chg_inhibit rc = %d\n", rc);
		return rc;
	}

	rc = smb1360_masked_write(chip, CFG_CHG_MISC_REG,
					CFG_BAT_OV_ENDS_CHG_CYC,
					chip->ov_ends_chg_cycle_disabled ?
					0 : CFG_BAT_OV_ENDS_CHG_CYC);
	if (rc) {
		dev_err(chip->dev, "Couldn't set bat_ov_ends_charge rc = %d\n"
									, rc);
		return rc;
	}

	/* battery missing detection */
	rc = smb1360_masked_write(chip, CFG_BATT_MISSING_REG,
				BATT_MISSING_SRC_THERM_BIT,
				BATT_MISSING_SRC_THERM_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set batt_missing config = %d\n",
									rc);
		return rc;
	}

	rc = smb1360_jeita_init(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't init jeita, rc = %d\n", rc);
		return rc;
	}

	/* interrupt enabling - active low */
	if (chip->client->irq) {
		mask = CHG_STAT_IRQ_ONLY_BIT
			| CHG_STAT_ACTIVE_HIGH_BIT
			| CHG_STAT_DISABLE_BIT
			| CHG_TEMP_CHG_ERR_BLINK_BIT;

		if (!chip->pulsed_irq)
			reg = CHG_STAT_IRQ_ONLY_BIT;
		else
			reg = CHG_TEMP_CHG_ERR_BLINK_BIT;
		rc = smb1360_masked_write(chip, CFG_STAT_CTRL_REG, mask, reg);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set irq config rc = %d\n",
					rc);
			return rc;
		}

		/* enabling only interesting interrupts */
		rc = smb1360_write(chip, IRQ_CFG_REG,
				IRQ_BAT_HOT_COLD_HARD_BIT
				| IRQ_BAT_HOT_COLD_SOFT_BIT
				| IRQ_INTERNAL_TEMPERATURE_BIT
				| IRQ_DCIN_UV_BIT
				| IRQ_AICL_DONE_BIT);
		if (rc) {
			dev_err(chip->dev, "Couldn't set irq1 config rc = %d\n",
					rc);
			return rc;
		}

		rc = smb1360_write(chip, IRQ2_CFG_REG,
				IRQ2_SAFETY_TIMER_BIT
				| IRQ2_CHG_ERR_BIT
				| IRQ2_CHG_PHASE_CHANGE_BIT
				| IRQ2_POWER_OK_BIT
				| IRQ2_BATT_MISSING_BIT
				| IRQ2_VBAT_LOW_BIT);
		if (rc) {
			dev_err(chip->dev, "Couldn't set irq2 config rc = %d\n",
					rc);
			return rc;
		}

		rc = smb1360_write(chip, IRQ3_CFG_REG,
				IRQ3_FG_ACCESS_OK_BIT
				| IRQ3_SOC_CHANGE_BIT
				| IRQ3_SOC_MIN_BIT
				| IRQ3_SOC_MAX_BIT
				| IRQ3_SOC_EMPTY_BIT
				| IRQ3_SOC_FULL_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set irq3 enable rc = %d\n",
					rc);
			return rc;
		}
	}

	/* batt-id configuration */
	if (chip->batt_id_disabled) {
		mask = BATT_ID_ENABLED_BIT | CHG_BATT_ID_FAIL;
		reg = CHG_BATT_ID_FAIL;
		rc = smb1360_masked_write(chip, CFG_FG_BATT_CTRL_REG,
						mask, reg);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set batt_id_reg rc = %d\n",
					rc);
			return rc;
		}
	}

	/* USB OTG current limit configuration */
	if (chip->otg_batt_curr_limit != -EINVAL) {
		for (i = 0; i < ARRAY_SIZE(otg_curr_ma); i++) {
			if (otg_curr_ma[i] >= chip->otg_batt_curr_limit)
				break;
		}

		if (i == ARRAY_SIZE(otg_curr_ma))
			i = i - 1;

		rc = smb1360_masked_write(chip, CFG_BATT_CHG_REG,
						OTG_CURRENT_MASK,
					i << OTG_CURRENT_SHIFT);
		if (rc)
			pr_err("Couldn't set OTG current limit, rc = %d\n", rc);
	}

	rc = smb1360_charging_disable(chip, USER, !!chip->charging_disabled);
	if (rc)
		dev_err(chip->dev, "Couldn't '%s' charging rc = %d\n",
			chip->charging_disabled ? "disable" : "enable", rc);

	if (chip->parallel_charging) {
		rc = smb1360_parallel_charger_enable(chip, PARALLEL_USER,
						!chip->charging_disabled);
		if (rc)
			dev_err(chip->dev, "Couldn't '%s' parallel-charging rc = %d\n",
			chip->charging_disabled ? "disable" : "enable", rc);
	}

	return rc;
}

static int smb1360_delayed_hw_init(struct smb1360_chip *chip)
{
	int rc;

	pr_debug("delayed hw init start!\n");

	if (chip->otp_hard_jeita_config) {
		rc = smb1360_hard_jeita_otp_init(chip);
		if (rc) {
			pr_err("Unable to change the OTP hard jeita, rc=%d\n",
				rc);
			return rc;
		}
	}
	rc = smb1360_check_batt_profile(chip);
	if (rc) {
		pr_err("Unable to modify battery profile, rc=%d\n", rc);
		return rc;
	}

	rc = smb1360_otp_gain_init(chip);
	if (rc) {
		pr_err("Unable to config otp gain, rc=%d\n", rc);
		return rc;
	}

	rc = smb1360_fg_config(chip);
	if (rc) {
		pr_err("Couldn't configure FG rc=%d\n", rc);
		return rc;
	}

	rc = smb1360_check_cycle_stretch(chip);
	if (rc) {
		pr_err("Unable to check cycle-stretch\n");
		return rc;
	}

	pr_debug("delayed hw init complete!\n");
	return rc;
}

static void smb1360_delayed_init_work_fn(struct work_struct *work)
{
	int rc = 0;
	struct smb1360_chip *chip = container_of(work, struct smb1360_chip,
						delayed_init_work.work);

	rc = smb1360_delayed_hw_init(chip);

	if (!rc) {
		/*
		 * If the delayed hw init successfully, update battery
		 * power_supply to make sure the correct SoC reported
		 * timely.
		 */
		power_supply_changed(&chip->batt_psy);
	} else if (rc == -ETIMEDOUT) {
		/*
		 * If the delayed hw init failed causing by waiting for
		 * FG access timed-out, force a FG reset and queue the
		 * worker again to retry the initialization.
		 */
		pr_debug("delayed hw init timed-out, retry!");
		rc = smb1360_force_fg_reset(chip);
		if (rc) {
			pr_err("couldn't reset FG, rc = %d\n", rc);
			return;
		}
		schedule_delayed_work(&chip->delayed_init_work, 0);
	} else {
		pr_err("delayed hw init failed, rc=%d\n", rc);
	}
}

static int smb_parse_batt_id(struct smb1360_chip *chip)
{
	int rc = 0, rpull = 0, vref = 0;
	int64_t denom, batt_id_uv;
	struct device_node *node = chip->dev->of_node;
	struct qpnp_vadc_result result;

	chip->vadc_dev = qpnp_get_vadc(chip->dev, "smb1360");
	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		if (rc == -EPROBE_DEFER)
			pr_err("vadc not found - defer rc=%d\n", rc);
		else
			pr_err("vadc property missing, rc=%d\n", rc);

		return rc;
	}

	rc = of_property_read_u32(node, "qcom,profile-a-rid-kohm",
						&chip->profile_rid[0]);
	if (rc < 0) {
		pr_err("Couldn't read profile-a-rid-kohm rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(node, "qcom,profile-b-rid-kohm",
						&chip->profile_rid[1]);
	if (rc < 0) {
		pr_err("Couldn't read profile-b-rid-kohm rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(node, "qcom,batt-id-vref-uv", &vref);
	if (rc < 0) {
		pr_err("Couldn't read batt-id-vref-uv rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(node, "qcom,batt-id-rpullup-kohm", &rpull);
	if (rc < 0) {
		pr_err("Couldn't read batt-id-rpullup-kohm rc=%d\n", rc);
		return rc;
	}

	/* read battery ID */
	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX2_BAT_ID, &result);
	if (rc) {
		pr_err("error reading batt id channel = %d, rc = %d\n",
					LR_MUX2_BAT_ID, rc);
		return rc;
	}
	batt_id_uv = result.physical;

	if (batt_id_uv == 0) {
		/* vadc not correct or batt id line grounded, report 0 kohms */
		pr_err("batt_id_uv = 0, batt-id grounded using same profile\n");
		return 0;
	}

	denom = div64_s64(vref * 1000000LL, batt_id_uv) - 1000000LL;
	if (denom == 0) {
		/* batt id connector might be open, return 0 kohms */
		return 0;
	}
	chip->connected_rid = div64_s64(rpull * 1000000LL + denom/2, denom);

	pr_debug("batt_id_voltage = %lld, connected_rid = %d\n",
			batt_id_uv, chip->connected_rid);

	return 0;
}

/*
 * Note the below:
 * 1. if both qcom,soft-jeita-supported and qcom,config-hard-thresholds
 * are not defined, SMB continues with default OTP configuration.
 * 2. if both are enabled, the hard thresholds are modified.
 * 3. if only qcom,config-hard-thresholds is defined, the soft JEITA is disabled
 * 4. if only qcom,soft-jeita-supported is defined, the soft JEITA thresholds
 * are modified.
 */
static int smb1360_parse_jeita_params(struct smb1360_chip *chip)
{
	int rc = 0;
	struct device_node *node = chip->dev->of_node;
	int temp[2];

	if (of_property_read_bool(node, "qcom,config-hard-thresholds")) {
		rc = of_property_read_u32(node,
			"qcom,cold-bat-decidegc", &chip->cold_bat_decidegc);
		if (rc) {
			pr_err("cold_bat_decidegc property error, rc = %d\n",
								rc);
			return -EINVAL;
		}

		rc = of_property_read_u32(node,
			"qcom,hot-bat-decidegc", &chip->hot_bat_decidegc);
		if (rc) {
			pr_err("hot_bat_decidegc property error, rc = %d\n",
								rc);
			return -EINVAL;
		}

		chip->config_hard_thresholds = true;
		pr_info("config_hard_thresholds = %d, cold_bat_decidegc = %d, hot_bat_decidegc = %d\n",
			chip->config_hard_thresholds, chip->cold_bat_decidegc,
			chip->hot_bat_decidegc);
	} else if (of_property_read_bool(node, "qcom,otp-hard-jeita-config")) {
		rc = of_property_read_u32(node, "qcom,otp-cold-bat-decidegc",
					&chip->otp_cold_bat_decidegc);
		if (rc) {
			pr_err("otp-cold-bat-decidegc property error, rc = %d\n",
								rc);
			return -EINVAL;
		}

		rc = of_property_read_u32(node, "qcom,otp-hot-bat-decidegc",
					&chip->otp_hot_bat_decidegc);

		if (rc) {
			pr_err("otp-hot-bat-decidegc property error, rc = %d\n",
								rc);
			return -EINVAL;
		}

		chip->otp_hard_jeita_config = true;
		rc = of_property_read_u32_array(node,
				"qcom,otp-hard-jeita-hysteresis", temp, 2);
		if (rc) {
			if (rc != -EINVAL) {
				pr_err("read otp-hard-jeita-hysteresis failed, rc = %d\n",
					rc);
				return rc;
			}
		} else {
			chip->cold_hysteresis = temp[0];
			chip->hot_hysteresis = temp[1];
		}

		pr_debug("otp_hard_jeita_config = %d, otp_cold_bat_decidegc = %d\n"
			"otp_hot_bat_decidegc = %d, cold_hysteresis = %d\n"
			"hot_hysteresis = %d\n",
			chip->otp_hard_jeita_config,
			chip->otp_cold_bat_decidegc,
			chip->otp_hot_bat_decidegc, chip->cold_hysteresis,
			chip->hot_hysteresis);
	}

	if (of_property_read_bool(node, "qcom,soft-jeita-supported")) {
		rc = of_property_read_u32(node, "qcom,warm-bat-decidegc",
						&chip->warm_bat_decidegc);
		if (rc) {
			pr_err("warm_bat_decidegc property error, rc = %d\n",
								rc);
			return -EINVAL;
		}

		rc = of_property_read_u32(node, "qcom,cool-bat-decidegc",
						&chip->cool_bat_decidegc);
		if (rc) {
			pr_err("cool_bat_decidegc property error, rc = %d\n",
								rc);
			return -EINVAL;
		}
		rc = of_property_read_u32(node, "qcom,cool-bat-mv",
						&chip->cool_bat_mv);
		if (rc) {
			pr_err("cool_bat_mv property error, rc = %d\n", rc);
			return -EINVAL;
		}

		if(battery_type == Battery_438V)
			chip->cool_bat_mv = 4380;
		else
			chip->cool_bat_mv = 4350;

		rc = of_property_read_u32(node, "qcom,warm-bat-mv",
						&chip->warm_bat_mv);
		if (rc) {
			pr_err("warm_bat_mv property error, rc = %d\n", rc);
			return -EINVAL;
		}

		rc = of_property_read_u32(node, "qcom,cool-bat-ma",
						&chip->cool_bat_ma);
		if (rc) {
			pr_err("cool_bat_ma property error, rc = %d\n", rc);
			return -EINVAL;
		}

		rc = of_property_read_u32(node, "qcom,warm-bat-ma",
						&chip->warm_bat_ma);

		if (rc) {
			pr_err("warm_bat_ma property error, rc = %d\n", rc);
			return -EINVAL;
		}

		chip->soft_jeita_supported = true;
	} else {
		/*
		 * If no soft JEITA configuration required from devicetree,
		 * read the default soft JEITA setting for hard JEITA
		 * configuration sanity check.
		 */
		rc = smb1360_get_soft_jeita_threshold(chip,
				&chip->cool_bat_decidegc,
				&chip->warm_bat_decidegc);
		if (rc) {
			pr_err("get default soft JEITA threshold failed, rc=%d\n",
							rc);
			return rc;
		}
	}

	pr_debug("soft-jeita-enabled = %d, warm-bat-decidegc = %d, cool-bat-decidegc = %d, cool-bat-mv = %d, warm-bat-mv = %d, cool-bat-ma = %d, warm-bat-ma = %d\n",
		chip->soft_jeita_supported, chip->warm_bat_decidegc,
		chip->cool_bat_decidegc, chip->cool_bat_mv, chip->warm_bat_mv,
		chip->cool_bat_ma, chip->warm_bat_ma);

	return rc;
}

#define MAX_PARALLEL_CURRENT		540
static int smb1360_parse_parallel_charging_params(struct smb1360_chip *chip)
{
	struct device_node *node = chip->dev->of_node;

	if (of_property_read_bool(node, "qcom,parallel-charging-enabled")) {

		if (!chip->rsense_10mohm) {
			pr_err("10mohm-rsense configuration not enabled - parallel-charging disabled\n");
			return 0;
		}

		//chip->parallel_charging = true;
		// set parallel charging default disable for india 5W
		chip->parallel_charging = false;

		chip->max_parallel_chg_current = MAX_PARALLEL_CURRENT;
		of_property_read_u32(node, "qcom,max-parallel-current-ma",
					&chip->max_parallel_chg_current);

		pr_info("Max parallel charger current = %dma\n",
				chip->max_parallel_chg_current);

		/* mark the parallel-charger as disabled */
		chip->parallel_chg_disable_status |= PARALLEL_CURRENT;
	}

	return 0;
}

static int smb_parse_dt(struct smb1360_chip *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}
/**
	chip->HW_ID = of_property_read_bool(node,"ZC550KL-8939-ER-device");
**/
	chip->HW_ID = get_zc550kl_pcb_rev();
	pr_info("load on HW_ID=%d\n", chip->HW_ID);

	chip->rsense_10mohm = of_property_read_bool(node, "qcom,rsense-10mhom");
	chip->otp_rslow_config = of_property_read_bool(node,
						"qcom,otp-rslow-cfg");
	chip->otp_jeita_hard_config = of_property_read_bool(node,
						"qcom,otp-jeita-hard");
	if (of_property_read_bool(node, "qcom,batt-profile-select")) {
		rc = smb_parse_batt_id(chip);
		if (rc < 0) {
			if (rc != -EPROBE_DEFER)
				pr_err("Unable to parse batt-id rc=%d\n", rc);
			return rc;
		}
	}

	chip->otg_fet_present = of_property_read_bool(node,
						"qcom,otg-fet-present");
	if (chip->otg_fet_present) {
		chip->otg_fet_enable_gpio = of_get_named_gpio(node,
						"qcom,otg-fet-enable-gpio", 0);
		if (!gpio_is_valid(chip->otg_fet_enable_gpio)) {
			if (chip->otg_fet_enable_gpio != -EPROBE_DEFER)
				pr_err("Unable to get OTG FET enable gpio=%d\n",
						chip->otg_fet_enable_gpio);
			return chip->otg_fet_enable_gpio;
		} else {
			/* Configure OTG FET control gpio */
			rc = devm_gpio_request_one(chip->dev,
					chip->otg_fet_enable_gpio,
					GPIOF_OPEN_DRAIN | GPIOF_INIT_HIGH,
					"smb1360_otg_fet_gpio");
			if (rc) {
				pr_err("Unable to request gpio rc=%d\n", rc);
				return rc;
			}
		}
	}

	chip->pulsed_irq = of_property_read_bool(node, "qcom,stat-pulsed-irq");

	rc = of_property_read_u32(node, "qcom,float-voltage-mv",
						&chip->vfloat_mv);
	if (rc < 0)
		chip->vfloat_mv = -EINVAL;

	if(battery_type == Battery_438V)
		chip->vfloat_mv = 4380;
	else
		chip->vfloat_mv = 4350;

	rc = of_property_read_u32(node, "qcom,charging-timeout",
						&chip->safety_time);
	if (rc < 0)
		chip->safety_time = -EINVAL;

	if (!rc && (chip->safety_time > chg_time[ARRAY_SIZE(chg_time) - 1])) {
		dev_err(chip->dev, "Bad charging-timeout %d\n",
						chip->safety_time);
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,recharge-thresh-mv",
						&chip->resume_delta_mv);
	if (rc < 0)
		chip->resume_delta_mv = -EINVAL;

	chip->recharge_disabled = of_property_read_bool(node,
						"qcom,recharge-disabled");

	rc = of_property_read_u32(node, "qcom,iterm-ma", &chip->iterm_ma);
	if (rc < 0)
		chip->iterm_ma = -EINVAL;

	if(battery_type == Battery_438V)
		chip->iterm_ma = 200;
	else
		chip->iterm_ma = 150;

	chip->iterm_disabled = of_property_read_bool(node,
						"qcom,iterm-disabled");

	chip->chg_inhibit_disabled = of_property_read_bool(node,
						"qcom,chg-inhibit-disabled");

	chip->charging_disabled = of_property_read_bool(node,
						"qcom,charging-disabled");

	chip->batt_id_disabled = of_property_read_bool(node,
						"qcom,batt-id-disabled");

	chip->shdn_after_pwroff = of_property_read_bool(node,
						"qcom,shdn-after-pwroff");

	chip->min_icl_usb100 = of_property_read_bool(node,
						"qcom,min-icl-100ma");

	chip->ov_ends_chg_cycle_disabled = of_property_read_bool(node,
					"qcom,disable-ov-ends-chg-cycle");

	rc = smb1360_parse_parallel_charging_params(chip);
	if (rc) {
		pr_err("Couldn't parse parallel charginng params rc=%d\n", rc);
		return rc;
	}

	if (of_find_property(node, "qcom,thermal-mitigation",
					&chip->thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
					chip->thermal_levels,
						GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			pr_err("thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
				"qcom,thermal-mitigation",
				chip->thermal_mitigation, chip->thermal_levels);
		if (rc) {
			pr_err("Couldn't read threm limits rc = %d\n", rc);
			return rc;
		}
	}

	rc = smb1360_parse_jeita_params(chip);
	if (rc < 0) {
		pr_err("Couldn't parse jeita params, rc = %d\n", rc);
		return rc;
	}

	/* fg params */
	chip->empty_soc_disabled = of_property_read_bool(node,
						"qcom,empty-soc-disabled");

	rc = of_property_read_u32(node, "qcom,fg-delta-soc", &chip->delta_soc);
	if (rc < 0)
		chip->delta_soc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-soc-max", &chip->soc_max);
	if (rc < 0)
		chip->soc_max = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-soc-min", &chip->soc_min);
	if (rc < 0)
		chip->soc_min = -EINVAL;

	chip->awake_min_soc = of_property_read_bool(node,
					"qcom,awake-min-soc");

	rc = of_property_read_u32(node, "qcom,fg-voltage-min-mv",
					&chip->voltage_min_mv);
	if (rc < 0)
		chip->voltage_min_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-voltage-empty-mv",
					&chip->voltage_empty_mv);
	if (rc < 0)
		chip->voltage_empty_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-batt-capacity-mah",
					&chip->batt_capacity_mah);
	if (rc < 0)
		chip->batt_capacity_mah = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-cc-soc-coeff",
					&chip->cc_soc_coeff);
	if (rc < 0)
		chip->cc_soc_coeff = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-cutoff-voltage-mv",
						&chip->v_cutoff_mv);
	if (rc < 0)
		chip->v_cutoff_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-iterm-ma",
					&chip->fg_iterm_ma);
	if (rc < 0)
		chip->fg_iterm_ma = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-ibatt-standby-ma",
					&chip->fg_ibatt_standby_ma);
	if (rc < 0)
		chip->fg_ibatt_standby_ma = -EINVAL;

	rc = of_property_read_u32(node, "qcom,thermistor-c1-coeff",
					&chip->fg_thermistor_c1_coeff);
	if (rc < 0)
		chip->fg_thermistor_c1_coeff = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-cc-to-cv-mv",
					&chip->fg_cc_to_cv_mv);
	if (rc < 0)
		chip->fg_cc_to_cv_mv = -EINVAL;

	if(battery_type == Battery_438V)
               chip->fg_cc_to_cv_mv = 4360;
       else
               chip->fg_cc_to_cv_mv = 4330;

	rc = of_property_read_u32(node, "qcom,otg-batt-curr-limit",
					&chip->otg_batt_curr_limit);
	if (rc < 0)
		chip->otg_batt_curr_limit = -EINVAL;

	{
#ifdef CONFIG_ASUS_ZC550KL8916_PROJECT

		int new_pcb;

		new_pcb= of_get_named_gpio(chip->dev->of_node, "new_pcb_gpio", 0);

		if (new_pcb < 0)
			pr_err("new_pcb_gpio is not available\n");
		else {
			printk(" new_pcb_gpio ====> %d\n", gpio_get_value(new_pcb));
		}

		if(gpio_get_value(new_pcb) == 0)
			chip->otg_batt_curr_limit = 950;

#endif
	}

	rc = of_property_read_u32(node, "qcom,fg-auto-recharge-soc",
					&chip->fg_auto_recharge_soc);
	if (rc < 0)
		chip->fg_auto_recharge_soc = -EINVAL;

	if (of_property_read_bool(node, "qcom,fg-reset-at-pon")) {
		chip->fg_reset_at_pon = true;
		rc = of_property_read_u32(node, "qcom,fg-reset-thresold-mv",
						&chip->fg_reset_threshold_mv);
		if (rc) {
			pr_debug("FG reset voltage threshold not specified using 50mV\n");
			chip->fg_reset_threshold_mv = FG_RESET_THRESHOLD_MV;
		}
	}

	return 0;
}
//asus report capacity frank ++++

static inline struct power_supply *get_psy_battery(void)
{
	struct class_dev_iter iter;
	struct device *dev;
	static struct power_supply *pst;

	class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
	while ((dev = class_dev_iter_next(&iter))) {
		pst = (struct power_supply *)dev_get_drvdata(dev);
		if (pst->type == POWER_SUPPLY_TYPE_BATTERY) {
			class_dev_iter_exit(&iter);
			return pst;
		}
	}
	class_dev_iter_exit(&iter);

	return NULL;
}
extern bool get_sw_charging_toggle(void)
{
	bool ret;

	mutex_lock(&g_charging_toggle_lock);
	ret = g_charging_toggle_status;
	mutex_unlock(&g_charging_toggle_lock);

	return ret;
}
extern int get_charger_type(void)
{
	int ret;

	if (!smb1360_dev) {
		pr_err("Warning: smb358_dev is null due to probe function has error\n");
		return -1;
	}
	ret = g_usb_state;
	return ret;
}
int asus_battery_update_status(void)
{
	int status = POWER_SUPPLY_STATUS_UNKNOWN;

	if (smb1360_dev->usb_present == 1 &&smb1360_get_prop_batt_capacity(smb1360_dev) ==100)
	{
		status = POWER_SUPPLY_STATUS_FULL;
	}
	else if (smb1360_dev->usb_present == 0 &&smb1360_get_prop_batt_capacity(smb1360_dev) ==100)
	{
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	else
		status = smb1360_get_prop_batt_status(smb1360_dev);

	return status;
}
int get_AICL_status(void)
{
	int rc;
	u8 reg = 0;

	rc = smb1360_read(smb1360_dev, FIRST_STATUS_REG, &reg);
	if (rc) {
		pr_err("Couldn't read FIRST_STATUS_REG rc=%d\n", rc);
		return 0;
	}

	return reg;
}
static int asus_update_all(void)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;
	int batt_temp;
	int charging_status;

	psy = get_psy_battery();
	if (!psy) {
		BAT_DBG_E("%s: can't get battery psy!\n", __FUNCTION__);
		return -EINVAL;
	}
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (!ret) {
		batt_info.capacity = val.intval;
	} else{
		BAT_DBG_E("%s: can't get capacity, ret = %d!\n", __FUNCTION__, ret);
		batt_info.capacity = ret;
	}
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret) {
		batt_info.voltage_now = val.intval /1000;
	} else{
		BAT_DBG_E("%s: can't get voltage_now, ret = %d!\n", __FUNCTION__, ret);
		batt_info.voltage_now = ret;
	}
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (!ret) {
		batt_info.current_now = val.intval / 1000;
	} else{
		BAT_DBG_E("%s: can't get current_now, ret = %d!\n", __FUNCTION__, ret);
		batt_info.current_now = ret;
	}
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (!ret) {
		batt_temp = val.intval;
		if (batt_temp >= 10 || batt_temp <= -10) {
			batt_info.temperature10 = batt_temp / 10;
			batt_info.temperature = batt_temp - (batt_info.temperature10 * 10);
			if (batt_info.temperature < 0) {
				batt_info.temperature = -batt_info.temperature;
			}
			batt_info.temperature_negative_sign = "";
		} else {
			batt_info.temperature10 = 0;
			batt_info.temperature = batt_temp < 0 ? -batt_temp : batt_temp;
			if (batt_temp >= 0) {
				batt_info.temperature_negative_sign = "";
			}
		}
	} else{
		BAT_DBG_E("%s: can't get bat_temp, ret = %d!\n", __FUNCTION__, ret);
		batt_temp = ret;
	}

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_STATUS, &val);
	if (!ret) {
		batt_info.status = val.intval;
	} else{
		BAT_DBG_E("%s: can't get charging_status, ret = %d!\n", __FUNCTION__, ret);
		charging_status = ret;
	}
	//ret = get_bms_calculated_soc(&batt_info.rawsoc);
	//if (ret < 0) {
		//BAT_DBG_E("%s: can't get raw bms soc, ret = %d\n", __FUNCTION__, ret);
	//}
	batt_info.rawsoc = smb1360_get_prop_batt_capacity(smb1360_dev);
	batt_info.cable_status = get_charger_type();
	batt_info.charging_toggle = get_sw_charging_toggle();
	//batt_info.fcc = get_vm_bms_fcc();
	batt_info.Rsoc =smb1360_get_prop_batt_capacity(smb1360_dev);
	batt_info.AICL_status = get_AICL_status();
	return 0;
}
static int asus_print_all(void)
{
	int ret;
	//char chargerReg[128] = "";
	char battInfo[256];
	/*  UNKN means UNKOWN, CHRG mean CHARGING, DISC means DISCHARGING,
	    NOTC means NOT CHARGING
	*/
	char batt_status_str[5][5] = {
		"UNKN",
		"CHRG",
		"DISC",
		"NOTC",
		"FULL"
	};
	char cable_status_str[8][5] = {
		"USB",
		"AC",
		"NO",
		"EN5V",
		"DS5V",
		"PAD",
		"UNKN",
		"SE1",
	};
	ret = asus_update_all();
	if (ret < 0) {
		BAT_DBG_E("%s: can't update battery info!\n", __FUNCTION__);
		return -1;
	}
	//ChargerRegDump(chargerReg, 128);
	battID = check_battery_id();
	snprintf(battInfo, sizeof(battInfo), "report Capacity ==>%d, FCC:%dmAh, BMS:%d, V:%dmV, Cur:%dmA, ",
		batt_info.capacity,
		batt_info.fcc,
		batt_info.capacity,
		batt_info.voltage_now,
		batt_info.current_now);
	snprintf(battInfo, sizeof(battInfo), "%sTemp:%s%d.%dC, Cable:%d(%s), Status:%s, Charging:%s, Rsoc:%d, BATID:%lld, TLevel:%d, ",
		battInfo,
		batt_info.temperature_negative_sign,
		batt_info.temperature10,
		batt_info.temperature,
		batt_info.cable_status,
		cable_status_str[batt_info.cable_status],
		batt_status_str[batt_info.status],
		batt_info.charging_toggle ? "ON" : "OFF",
		batt_info.Rsoc,
		battID,
		smb1360_dev->therm_lvl_sel);
	snprintf(battInfo, sizeof(battInfo), "%sAICL register(0x48):0x%X\n",
		battInfo,
		batt_info.AICL_status);
	ASUSEvtlog("[BAT][Ser]%s", battInfo);
	BAT_DBG("%s", battInfo);
	g_last_print_time = current_kernel_time();
	return 0;
}
void asus_polling_battery_data_work(int time)
{
	cancel_delayed_work(&battery_poll_data_work);
	schedule_delayed_work(&battery_poll_data_work, time * HZ);
}
void asus_polling_data(struct work_struct *dat)
{
	int ret;
	int usb_state;
	usb_state = g_usb_state;
	ret = asus_print_all();

	if(smb1360_dev->therm_lvl_sel==2)
		smb1360_set_appropriate_usb_current(smb1360_dev);

	/*BSP david: if report capacity fail, do it after 5s*/
	if (!ret) {
		if(batt_info.capacity > 15)
			asus_polling_battery_data_work(180);
		else
			asus_polling_battery_data_work(60);
		if (!smb1360_is_charging(usb_state)) {
			schedule_delayed_work(&SetBatRTCWorker, 0);
		}
	} else{
		printk("%s: gauge not ready yet, delay 5s!\n", __FUNCTION__);
		asus_polling_battery_data_work(5);
	}
}

static inline int get_battery_rsoc(int *rsoc)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (!ret)
		*rsoc = val.intval;

	return ret;
}
void BatTriggeredSetRTC(struct work_struct *dat)
{
	unsigned long batflags;
	struct timespec new_batAlarm_time;
	struct timespec mtNow;
	int RTCSetInterval = 180;
	int cableType;
	int soc;
	int ret;

	if (!smb1360_dev) {
		BAT_DBG("%s: driver not ready yet!\n", __FUNCTION__);
		return;
	}

	mtNow = current_kernel_time();
	new_batAlarm_time.tv_sec = 0;
	new_batAlarm_time.tv_nsec = 0;

	cableType = get_charger_type();

	if (smb1360_is_charging(cableType)) {
		RTCSetInterval = 60;
	} else{
		ret = get_battery_rsoc(&soc);
		if (!ret) {
			if (soc <= 1) {
				RTCSetInterval = 300;
			} else{
				RTCSetInterval = soc * 450;
				if (RTCSetInterval < 450) {
					RTCSetInterval = 450;
				}
				if (RTCSetInterval > 36000) {
					RTCSetInterval = 36000;
				}
			}
		} else{
			BAT_DBG_E("%s: NO SOC INFO! Set minimum alarm time!\n", __FUNCTION__);
			RTCSetInterval = 300;
		}
	}
	new_batAlarm_time.tv_sec = mtNow.tv_sec + RTCSetInterval;
	BAT_DBG("%s: alarm start after %ds\n", __FUNCTION__, RTCSetInterval);
	spin_lock_irqsave(&bat_alarm_slock, batflags);
	alarm_start(&bat_alarm,
		timespec_to_ktime(new_batAlarm_time));
	spin_unlock_irqrestore(&bat_alarm_slock, batflags);
}
bool smb1360_is_charging(int usb_state)
{
	if (usb_state == USB_IN || usb_state == AC_IN ||usb_state == UNKNOWN_IN || usb_state == SE1_IN || usb_state == PAD_SUPPLY) {
		return true;
	} else{
		return false;
	}
}
//asus report capacity frank ----
/*+++BSP frank AC power supply+++*/
static char *supply_list[] = {
	"battery",
};

static enum power_supply_property asus_power_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};


static int smb1360_power_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	int usb_state;
	usb_state = g_usb_state;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = (usb_state == AC_IN || usb_state == UNKNOWN_IN || usb_state == SE1_IN) ? 1 : 0;
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}
static struct power_supply smb1360_power_supplies[] = {
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = asus_power_properties,
		.num_properties = ARRAY_SIZE(asus_power_properties),
		.get_property = smb1360_power_get_property,
	},
};
static int smb1360_register_power_supply(struct device *dev)
{
	int ret;
	ret = power_supply_register(dev, &smb1360_power_supplies[0]);
	if (ret) {
		BAT_DBG_E("Fail to register power supply AC\n");
		goto batt_err_reg_fail_ac;
	}
	return 0;

batt_err_reg_fail_ac:
	power_supply_unregister(&smb1360_power_supplies[0]);
	return ret;
}
/*---BSP frank AC power supply---*/
/*+++BSP frank add asus_set_charger +++*/
int set_otg_battery_current_limit(int value)
{
	int i,rc;
	for (i = 0; i < ARRAY_SIZE(otg_curr_ma); i++) {
			if (otg_curr_ma[i] >= value)
				break;
		}

	if (i == ARRAY_SIZE(otg_curr_ma))
		i = i - 1;

	pr_info("set otg battery current limit ====> %d \n",otg_curr_ma[i]);

	rc = smb1360_masked_write(smb1360_dev, CFG_BATT_CHG_REG,
						OTG_CURRENT_MASK,
					i << OTG_CURRENT_SHIFT);
	if (rc)
	{
		pr_err("Couldn't set OTG current limit, rc = %d\n", rc);
		return rc;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(set_otg_battery_current_limit);
static int set_otg(int toggle)
{
	int ret;

	if (!smb1360_dev) {
		pr_info("Warning: smb1360_dev is null due to probe function has error\n");
		return 1;
	}

	if(smb1360_dev->HW_ID == ZC550KL_8939_ER || smb1360_dev->HW_ID == ZC550KL_8939_PR)
	{
		pr_info("start to set otg ====> %d \n",toggle);

		if(toggle)
		{
			smb1360_charging_toggle(FLAGS, false, true);

			ret = gpio_direction_output(smb1360_dev->ovp_enable_gpio, 1);
			if (ret < 0) {
				printk("%s: set direction of ovp_enable_gpio  fail!\n", __FUNCTION__);
				return ret;
			}
		}
		else
		{
			ret = gpio_direction_output(smb1360_dev->ovp_enable_gpio, 0);
			if (ret < 0) {
				printk("%s: set direction of ovp_enable_gpio  fail!\n", __FUNCTION__);
				return ret;
			}
			smb1360_charging_toggle(FLAGS, true, true);
		}

		pr_info("set otg successfully ====> %d \n",toggle);
	}
	else
	{
		dev_err(smb1360_dev->dev, "%s++\n", __FUNCTION__);

		if(toggle)
		{
			ret = smb1360_otg_regulator_enable(smb1360_dev->otg_vreg.rdev);
			if (ret < 0)
			{
				printk("otg enable faill \n");
				return ret;
			}
		}
		else
		{
			ret = smb1360_otg_regulator_disable(smb1360_dev->otg_vreg.rdev);
			if (ret < 0)
			{
				printk("otg disable faill \n");
				return ret;
			}
		}

		dev_err(smb1360_dev->dev, "%s--\n", __FUNCTION__);
	}

	return 0;
}

static void smb_config_max_current(int usb_state)
{

	BAT_DBG("%s: usb_state = %d\n", __FUNCTION__, usb_state);
	/* USB Mode Detection (by SOC) */
	if (usb_state == AC_IN||usb_state == SE1_IN)
	{
		smb1360_dev->usb_psy_ma = ac_current;
		smb1360_set_appropriate_usb_current(smb1360_dev);
	} else if (usb_state == USB_IN)
	{
		/*do nothing when USB_IN, using default setting 500mA*/
		if(usbin_voltage_poor == true)
			smb1360_dev->usb_psy_ma = 100;
		else
			smb1360_dev->usb_psy_ma = 500;

		smb1360_set_appropriate_usb_current(smb1360_dev);
	} else if (usb_state == UNKNOWN_IN)
	{
		/*do nothing when UNKNOWN_IN, using default setting 500mA*/
		if(usbin_voltage_poor == true)
			smb1360_dev->usb_psy_ma = 100;
		else
			smb1360_dev->usb_psy_ma = 500;
		smb1360_set_appropriate_usb_current(smb1360_dev);
	}
	else if (usb_state == CABLE_OUT)
	{
		/*set 100mA when cable out*/
		smb1360_dev->usb_psy_ma = 100;
		smb1360_set_appropriate_usb_current(smb1360_dev);
	}

}

static void smb1360_config_max_current(int usb_state)
{
	int rc;
	//if (!smb1360_is_charging(usb_state))
	//	return;

	if (!smb1360_dev) {
		pr_err("%s: smb1360_dev is null due to driver probed isn't ready\n",
			__FUNCTION__);

		return;
	}
	rc = smb1360_enable_volatile_writes(smb1360_dev);
	if (rc < 0) {
		dev_err(smb1360_dev->dev, "Couldn't configure for volatile rc = %d\n",
				rc);
	}

	//smb358_pre_config();
	smb_config_max_current(usb_state);
}

int asus_set_Charger(int usb_state)
{
	int ret = 0;
	struct power_supply *psy;

	char usb_status_str[8][11] = {
		"USB_IN",
		"AC_IN",
		"CABLE_OUT",
		"ENABLE_5V",
		"DISABLE_5V",
		"PAD_SUPPLY",
		"UNKNOWN_IN",
		"SE1_IN",
	};
	//if( usb_state != CABLE_OUT )
		g_usb_state = usb_state;
	if (usb_state >= 0 && usb_state <= 7) {
		BAT_DBG("%s:%s\n", __FUNCTION__, usb_status_str[usb_state]);
	}

	/*BSP david: update power_supply after cable type changed*/
	psy = get_psy_battery();
	if (psy) {
		power_supply_changed(psy);
	} else{
		pr_err("%s: fail to request power supply changed\n", __FUNCTION__);
	}
	if (smb1360_dev) {
		power_supply_changed(&smb1360_power_supplies[0]);

		//under 02h[7]=0,  prevent cable_plug & reboot when suspend adapter
		//ret = Suspend_Adapter(0);
		//if(ret){
		//	pr_err("%s: Set adapter to normal mode fail!\n",
		//		__FUNCTION__);
		//}
	}
	switch (usb_state) {
	case AC_IN:
		if (smb1360_dev) {
			mutex_lock(&g_usb_state_lock);
			/*BSP david : do initial setting & config current*/
			smb1360_config_max_current(g_usb_state);
			mutex_unlock(&g_usb_state_lock);
		}
		break;
	case USB_IN:
		power_supply_set_online(smb1360_dev->usb_psy, 1);
	case UNKNOWN_IN:
	case SE1_IN:
		/* BSP Clay: AC debounce policy */
		if (smb1360_dev) {
			mutex_lock(&g_usb_state_lock);
			/*BSP david : do initial setting & config current*/
			smb1360_config_max_current(usb_state);
			mutex_unlock(&g_usb_state_lock);
			/*BSP david : do JEITA*/

		}
		break;
	case CABLE_OUT:
		if (smb1360_dev) {
			power_supply_set_online(smb1360_dev->usb_psy, 0);
			mutex_lock(&g_usb_state_lock);
			/*BSP david : do initial setting & config current*/
			smb1360_config_max_current(g_usb_state);
			mutex_unlock(&g_usb_state_lock);
			pr_info("cable out !\n");

		}
		break;
	case ENABLE_5V:
		if (smb1360_dev) {
			/* BSP Clay: AC debounce policy */
			ret = set_otg(1);
		}
		break;
	case DISABLE_5V:
		if (smb1360_dev) {
			/* BSP Clay: AC debounce policy */
			ret = set_otg(0);
		}
		break;
	default:
		BAT_DBG(" %s: not support usb state = %d\n", __FUNCTION__, usb_state);
		ret = 1;
	}
	return ret;
}
/*---BSP frank add asus_set_charger ---*/
//asus bsp frank add smb1360_charging_toggle  and runin_switch +++
int smb1360_charging_toggle(charging_toggle_level_t level, bool on, bool status_flag)
{
	int ret = 0;
	static int old_toggle = true;
	int result_toggle;
	bool reject = false;
	static charging_toggle_level_t old_lvl = JEITA;
	char *level_str[] = {
		"BALANCE",
		"JEITA",
		"FLAGS",
	};

	if (!smb1360_dev) {
		pr_info("Warning: smb1360_dev is null "
			"due to probe function has error\n");
		return 1;
	}


	result_toggle = on;

	/* do charging or not? */
	if (level != FLAGS) {
		if (on) {
			/* want to start charging? */
			if (level == JEITA) {
				if (!old_toggle) {
					/* want to restart charging? */
					if (old_lvl != JEITA) {
						/* reject the request! someone stop charging before */
						result_toggle = false;
						reject = true;
					}
				}
			} else if (level == BALANCE) {
				if (!old_toggle) {
					/* want to restart charging? */
					if (old_lvl != BALANCE) {
						/* reject the request! someone stop charging before */
						result_toggle = false;
						reject = true;
					}
				}
			} else {
				/* what the hell are you? */
			}
		} else {
			/* want to stop charging? just do it! */
		}
	} else {
		/* it's the highest level. just do it! */
	}

	BAT_DBG("%s: old_lvl:%s, request_lv:%s, old_toggle:%s, request_toggle:%s, result:%s, status_flag:%s\n",
		__FUNCTION__,
		level_str[old_lvl],
		level_str[level],
		old_toggle ? "ON" : "OFF",
		on ? "ON" : "OFF",
		result_toggle ? "ON" : "OFF",
		status_flag ? "ON" : "OFF");

	/* level value assignment */
	if (!reject) {
		old_lvl = level;
	}

	smb1360_charging_disable(smb1360_dev,USER,!result_toggle);



	old_toggle = result_toggle;
	if (status_flag) {
		mutex_lock(&g_charging_toggle_lock);
		g_charging_toggle_status = result_toggle;
		mutex_unlock(&g_charging_toggle_lock);
	}

	return ret;
}
static int runin_switch_on;
static int runin_switch_off;
static ssize_t runin_switch_on_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)

{
	runin_switch_control_off = false;
	smb1360_charging_toggle(FLAGS, true, true);
   	return sprintf(buf, "%d\n", runin_switch_on);

}
static ssize_t runin_switch_off_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)

{
	runin_switch_control_off = true;
	smb1360_charging_toggle(FLAGS, false, true);
   	return sprintf(buf, "%d\n", runin_switch_off);

}
static ssize_t runin_switch_on_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)

{

   	sscanf(buf, "%du", &runin_switch_on);

  			 return count;

}
static ssize_t runin_switch_off_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)

{

   	sscanf(buf, "%du", &runin_switch_off);

  			 return count;

}
static struct kobj_attribute runin_switch_on_attribute = __ATTR(runin_switch_on, 0666, runin_switch_on_show, runin_switch_on_store);
static struct kobj_attribute runin_switch_off_attribute = __ATTR(runin_switch_off, 0666, runin_switch_off_show, runin_switch_off_store);
//asus bsp frank add smb1360_charging_toggle and runin_switch ---
//int is_usb_chg_plugged_in(void)
//{
//	int rc;
//	rc = smb1360_dev->usb_present;
//	return rc;
//}
//EXPORT_SYMBOL_GPL(is_usb_chg_plugged_in);

int check_CSIR_function(void)
{
	u8 CSIR_first,CSIR_last;
	int rc,CSIR_Version;
	rc = smb1360_check_cycle_stretch(smb1360_dev);
	if (rc)
		pr_err("Unable to check cycle-stretch\n");

	rc = smb1360_enable_fg_access(smb1360_dev);
	if (rc) {
		pr_err("Couldn't request FG access rc=%d\n", rc);
	}
	smb1360_dev->fg_access_type = FG_ACCESS_CFG;
	rc = smb1360_select_fg_i2c_address(smb1360_dev);
	if (rc) {
		pr_err("Unable to set FG access I2C address\n");
	}

	//printk("++++ fg_i2c_addr = %X \n",smb1360_dev->fg_i2c_addr);
	rc = smb1360_fg_read(smb1360_dev, 0xD2, &CSIR_first);

	rc = smb1360_fg_read(smb1360_dev, 0xD3, &CSIR_last);

	//printk("++++ CSIR version = %X%X\n",CSIR_first,CSIR_last);

	CSIR_Version = (CSIR_first<<8 ) + CSIR_last;

	printk("++++ CSIR version = %X \n",CSIR_Version);

	rc = smb1360_disable_fg_access(smb1360_dev);
	if (rc) {
		pr_err("Couldn't disable FG access rc=%d\n", rc);
	}
	return CSIR_Version;
}
#define	CSIR_Version_PROC_FILE	"CSIR_Version"
static int CSIR_Version_proc_read(struct seq_file *buf, void *v)
{
	int ret = -1;
	ret = check_CSIR_function();
	seq_printf(buf, "%X\n", ret);
	return 0;
}
static int CSIR_Version_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, CSIR_Version_proc_read, NULL);
}

static const struct file_operations CSIR_Version_fops = {
	.owner = THIS_MODULE,
	.open = CSIR_Version_proc_open,
	.read = seq_read,
};
static void create_CSIR_Version_proc_file(void)
{
	struct proc_dir_entry *CSIR_Version_proc_file = proc_create(CSIR_Version_PROC_FILE, 0666, NULL, &CSIR_Version_fops);

	if (CSIR_Version_proc_file) {
		printk("[BAT][CHG][SMB][Proc]CSIR_Version create ok!\n");
	} else{
		printk("[BAT][CHG][SMB][Proc]CSIR_Version create failed!\n");
	}
	return;
}
int64_t check_battery_id(void)
{
	struct qpnp_vadc_result result;
	int rc=-1;
	uint64_t batt_id_uv;
	smb1360_dev->vadc_dev = qpnp_get_vadc(smb1360_dev->dev, "smb1360");
	/* read battery ID */
	rc = qpnp_vadc_read(smb1360_dev->vadc_dev, LR_MUX2_BAT_ID, &result);
	if (rc) {
		pr_err("error reading batt id channel = %d, rc = %d\n",
					LR_MUX2_BAT_ID, rc);
		return rc;
	}
	//denom = div64_s64(1800000 * 1000000LL, batt_id_uv) - 1000000LL;
	//connectd_R = div64_s64(100 * 1000000LL + denom/2, denom);
	batt_id_uv = result.physical;
	pr_info("battery id ====> %llu \n",batt_id_uv);
	return batt_id_uv;
}
#define	battery_id_PROC_FILE	"battery_id"
static int battery_id_proc_read(struct seq_file *buf, void *v)
{
	int64_t ret = -1;
	ret = check_battery_id();
	seq_printf(buf, "%llu\n", ret);
	return 0;
}
static int battery_id_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, battery_id_proc_read, NULL);
}

static const struct file_operations battery_id_fops = {
	.owner = THIS_MODULE,
	.open = battery_id_proc_open,
	.read = seq_read,
};
static void create_battery_id_proc_file(void)
{
	struct proc_dir_entry *battery_id_proc_file = proc_create(battery_id_PROC_FILE, 0666, NULL, &battery_id_fops);

	if (battery_id_proc_file) {
		printk("[BAT][CHG][SMB][Proc]battID_read create ok!\n");
	} else{
		printk("[BAT][CHG][SMB][Proc]battID_read create failed!\n");
	}
	return;
}

#define	ac_current_PROC_FILE	"driver/ac_current"
static int ac_current_proc_read(struct seq_file *buf, void *v)
{
	int ret = -1;
	ret = ac_current;
	seq_printf(buf, "%d\n", ret);
	return 0;
}
static int ac_current_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, ac_current_proc_read, NULL);
}
static ssize_t ac_current_proc_write(struct file *filp, const char __user *buff,
	size_t len, loff_t *data)
{
	int val;

	char messages[256];

	if (len > 256)
	{
		len = 256;
	}

	if (copy_from_user(messages, buff, len))
	{
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);

	ac_current = val;

	printk("[smb] set ac current ====> %d\n",val);

	return len;
}


static const struct file_operations ac_current_fops = {
	.owner = THIS_MODULE,
	.open = ac_current_proc_open,
	.write = ac_current_proc_write,
	.read = seq_read,
};
static void create_ac_current_proc_file(void)
{
	struct proc_dir_entry *ac_current_proc_file = proc_create(ac_current_PROC_FILE, 0666, NULL, &ac_current_fops);

	if (ac_current_proc_file) {
		printk("[BAT][CHG][SMB][Proc]create_ac_current_proc_file create ok!\n");
	} else{
		printk("[BAT][CHG][SMB][Proc]create_ac_current_proc_file create failed!\n");
	}
	return;
}

//ASUS_BSP frank for ChargerDriver stress test +++
#ifdef CONFIG_I2C_STRESS_TEST
static int TestSmb1360ChargerReadWrite(struct i2c_client *client)
{
	int status;
	u8 reg;
	i2c_log_in_test_case("[BAT][CHG][SMB][Test]Test Smb1360Charger start\n");

	status = smb1360_read(smb1360_dev, CFG_BATT_CHG_REG, &reg);
	if (status < 0) {
		i2c_log_in_test_case("[BAT][CHG][SMB][Test]Test Smb1360Charger end: status = %d\n", status);
		return I2C_TEST_SMB1360_FAIL;
	}
	status = smb1360_masked_write(smb1360_dev, CMD_I2C_REG,
		ALLOW_VOLATILE_BIT, ALLOW_VOLATILE_BIT);
	if (status < 0) {
		i2c_log_in_test_case("[BAT][CHG][SMB][Test]Test Smb1360Charger end: status = %d\n", status);
		return I2C_TEST_SMB1360_FAIL;
	}

	i2c_log_in_test_case("[BAT][CHG][SMB][Test]Test Smb1360Charger end: data = %x\n", reg);
	return I2C_TEST_PASS;
};
static struct i2c_test_case_info gChargerTestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(TestSmb1360ChargerReadWrite),
};
static void smb1360_i2c_stress_test(void)
{
	BAT_DBG("%s\n", __FUNCTION__);
	i2c_add_test_case(smb1360_dev->client, "Smb1360Charger", ARRAY_AND_SIZE(gChargerTestCaseInfo));
}
#endif
//ASUS_BSP frank for ChargerDriver stress test ---
static void check_default_boost_gpio(struct smb1360_chip *chip, const char *default_name,
									const char *gpio_name, int *gpio_num, int enable)
{
	int ret;
 	struct pinctrl *key_pinctrl;
 	struct pinctrl_state *set_state;

	if(!chip || !default_name || !gpio_name)
		pr_err("chip default_name or gpio_name is null!\n");

	key_pinctrl = devm_pinctrl_get(chip->dev);

	set_state = pinctrl_lookup_state(key_pinctrl, default_name);

	if (IS_ERR(set_state))
	{
		pr_err("cannot get phy pinctrl %s\n", default_name);
	}
	else
	{
		ret = pinctrl_select_state(key_pinctrl, set_state);
		printk("%s: %s pinctrl_select_state = %d\n", __FUNCTION__, default_name, ret);
	}

	*gpio_num = of_get_named_gpio(chip->dev->of_node, gpio_name, 0);
	if(*gpio_num < 0)
		printk("otg enable gpio is not available !\n");

	if ((!gpio_is_valid(*gpio_num))) {
		printk("%s: otg_enable_gpio is not valid!\n", __FUNCTION__);
	}
	ret = gpio_request(*gpio_num, gpio_name);
	if (ret < 0) {
		printk("%s: request otg_enable_gpio  fail!\n", __FUNCTION__);
	}

	ret = gpio_direction_output(*gpio_num, enable);
	if (ret < 0) {
		printk("%s: set direction of otg_enable_gpio  fail!\n", __FUNCTION__);
	}

	printk("%s ====> %d \n", gpio_name, gpio_get_value(*gpio_num));

}


void check_otg_ovp_gpio_value(struct smb1360_chip *chip)
{

	// 1. set otg enable gpio
	if(chip->HW_ID == ZC550KL_8939_ER)
		check_default_boost_gpio(chip, "otg_enable_default", "otg_enable_gpio", &chip->otg_enable_gpio, 1);
	else if(chip->HW_ID == ZC550KL_8939_PR)
		check_default_boost_gpio(chip, "boost_enable_default", "boost_enable_gpio", &chip->boost_enable_gpio, 0);

 	// 2. set ovp enable gpio
	check_default_boost_gpio(chip, "ovp_enable_default", "ovp_enable_gpio", &chip->ovp_enable_gpio, 0);

#if 0
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;

	// 1. set otg enable gpio
	key_pinctrl = devm_pinctrl_get(chip->dev);

	set_state = pinctrl_lookup_state(key_pinctrl, "otg_enable_default");

	if (IS_ERR(set_state))
	{
		pr_err("cannot get phy pinctrl otg_enable_default\n");
	}
	else
	{
		ret = pinctrl_select_state(key_pinctrl, set_state);
		printk("%s: otg_enable_default pinctrl_select_state = %d\n", __FUNCTION__, ret);
	}

	chip->otg_enable_gpio = of_get_named_gpio(chip->dev->of_node, "otg_enable_gpio", 0);
	if(chip->otg_enable_gpio < 0)
		printk("otg enable gpio is not available !\n");

	if ((!gpio_is_valid(chip->otg_enable_gpio))) {
		printk("%s: otg_enable_gpio is not valid!\n", __FUNCTION__);
	}
	ret = gpio_request(chip->otg_enable_gpio,"otg_enable_gpio");
	if (ret < 0) {
		printk("%s: request otg_enable_gpio  fail!\n", __FUNCTION__);
	}

	ret = gpio_direction_output(chip->otg_enable_gpio, 1);
	if (ret < 0) {
		printk("%s: set direction of otg_enable_gpio  fail!\n", __FUNCTION__);
	}

	printk("otg_enable_gpio ====> %d \n",gpio_get_value(chip->otg_enable_gpio));

	// 2. set ovp enable gpio
	set_state = pinctrl_lookup_state(key_pinctrl, "ovp_enable_default");
	if (IS_ERR(set_state))
	{
		pr_err("cannot get phy pinctrl ovp_enable_default\n");
	}
	else
	{
		ret = pinctrl_select_state(key_pinctrl, set_state);
		printk("%s: ovp_enable_default pinctrl_select_state = %d\n", __FUNCTION__, ret);
	}

	chip->ovp_enable_gpio = of_get_named_gpio(chip->dev->of_node, "ovp_enable_gpio", 0);
	if(chip->otg_enable_gpio < 0)
		printk("ovp enable gpio is not available !\n");

	if ((!gpio_is_valid(chip->ovp_enable_gpio))) {
		printk("%s: ovp_enable_gpio is not valid!\n", __FUNCTION__);
	}
	ret = gpio_request(chip->ovp_enable_gpio,"ovp_enable_gpio");
	if (ret < 0) {
		printk("%s: request ovp_enable_gpio  fail!\n", __FUNCTION__);
	}

	ret = gpio_direction_output(chip->ovp_enable_gpio, 0);
	if (ret < 0) {
		printk("%s: set direction of ovp_enable_gpio  fail!\n", __FUNCTION__);
	}

	printk("ovp_enable_gpio ====> %d \n",gpio_get_value(chip->ovp_enable_gpio));
#endif

}
#if defined(CONFIG_ASUS_ZC550KL8939_PROJECT)

int AICL_enable(bool value)
{
	int rc;
	if(value == true)
	{
		rc = smb1360_masked_write(smb1360_dev, CFG_GLITCH_FLT_REG, BIT(0), BIT(0));
		if (rc < 0) {
			dev_err(smb1360_dev->dev, "Couldn't enable AICL rc=%d\n",
				rc);
			return rc;
		}
	}
	else
	{
		rc = smb1360_masked_write(smb1360_dev, CFG_GLITCH_FLT_REG, BIT(0), 0);
		if (rc < 0) {
			dev_err(smb1360_dev->dev, "Couldn't disable AICL rc=%d\n",
				rc);
			return rc;
		}
	}
	return 0;
}

int Sysfs_read_file(char *filename, char *buf, size_t size)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;

	//int cal_val = 0;
	int readlen = 0;

	strcpy(buf,"0");

	/* open file */
	fp = filp_open(filename, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_info(" File open (%s) fail\n", filename);
		return -1;
	}

	/*For purpose that can use read/write system call*/

	/* Save addr_limit of the current process */
	old_fs = get_fs();
	/* Set addr_limit of the current process to that of kernel */
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, buf, size, &pos_lsts);
		buf[readlen] = '\0';
	} else {
		pr_info(" File (%s) strlen f_op=NULL or op->read=NULL\n", filename);
		return -2;
	}
	/* Set addr_limit of the current process back to its own */
	set_fs(old_fs);

	/* close file */
	filp_close(fp, NULL);

	//sscanf(country_code_buf, "%s", &cal_val);

	pr_info("buf ====> %s \n",buf);

	return 0;
}

void asus_adaptor_10W_check_work(int time)
{
	wake_lock(&adaptor_10W_check_wake_lock);
	schedule_delayed_work(&adaptor_10W_check_work, time * HZ);
}
int adaptor_10W_check_count;
void adaptor_10W_check(struct work_struct *dat)
{
	char read_buf[8];
	if(Sysfs_read_file(COUNTRY_CODE_FACTORY_FILE,read_buf,2) == 0)
	{
		if(strcmp(read_buf,"IN") == 0)
		{
			if(Sysfs_read_file(ADAPTOR_10W_FACTORY_FILE,read_buf,2) < 0)
			{
				pr_info("ADAPTOR_10W_FACTORY_FILE not exsit, set 5W ! \n");
				//AICL_enable(false);
				//mdelay(100);
				//AICL_enable(true);
			}
			else
			{
				pr_info("ADAPTOR_10W_FACTORY_FILE exsit, set 10W ! \n");
				smb1360_dev->parallel_charging = true;
				AICL_enable(false);
				mdelay(100);
				AICL_enable(true);
			}
		}
		else
		{
			pr_info("other country, not India ! set 10W setting \n");
			smb1360_dev->parallel_charging = true;
			AICL_enable(false);
			mdelay(100);
			AICL_enable(true);
		}
		wake_unlock(&adaptor_10W_check_wake_lock);
	}
	else
	{
		pr_info("can not read country code \n");
		adaptor_10W_check_count++;

		if(adaptor_10W_check_count < 20)
		{
			schedule_delayed_work(&adaptor_10W_check_work, 2 * HZ);
		}
		else
		{
			pr_info("adaptor_10W_check_count >= 20\n");
			wake_unlock(&adaptor_10W_check_wake_lock);
		}
	}
}
#endif
static int smb1360_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	u8 reg;
	int rc;
	struct smb1360_chip *chip;
	struct power_supply *usb_psy;
	pr_info("start ! \n");
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		//dev_dbg(&client->dev, "USB supply not found; defer probe\n");
		pr_info( "USB supply not found; defer probe\n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->resume_completed = true;
	chip->client = client;
	chip->dev = &client->dev;
	chip->usb_psy = usb_psy;
	chip->fake_battery_soc = -EINVAL;

	/*+++BSP frank AC power supply+++*/
	rc = smb1360_register_power_supply(&client->dev);
	if (rc < 0)
		return rc;
	/*---BSP frank AC power supply---*/

	mutex_init(&chip->read_write_lock);
	mutex_init(&chip->parallel_chg_lock);
	mutex_init(&chip->otp_gain_lock);
	mutex_init(&chip->fg_access_request_lock);
	INIT_DELAYED_WORK(&chip->jeita_work, smb1360_jeita_work_fn);
	INIT_DELAYED_WORK(&chip->delayed_init_work,
			smb1360_delayed_init_work_fn);
	init_completion(&chip->fg_mem_access_granted);
	smb1360_wakeup_src_init(chip);

//asus report capacity frank ++++
	smb1360_dev = chip;
	wake_lock_init(&bat_alarm_wake_lock, WAKE_LOCK_SUSPEND, "bat_alarm_wake");
	alarm_init(&bat_alarm, ALARM_REALTIME, batAlarm_handler);
	INIT_DELAYED_WORK(&SetBatRTCWorker, BatTriggeredSetRTC);
	INIT_DELAYED_WORK(&battery_poll_data_work, asus_polling_data);
//asus report capacity frank ----

#if defined(CONFIG_ASUS_ZC550KL8939_PROJECT)
	INIT_DELAYED_WORK(&adaptor_10W_check_work, adaptor_10W_check);
#endif

	/* probe the device to check if its actually connected */
	rc = smb1360_read(chip, CFG_BATT_CHG_REG, &reg);
	if (rc) {
		pr_err("Failed to detect SMB1360, device may be absent\n");
		return -ENODEV;
	}

	rc = read_revision(chip, &chip->revision);
	if (rc)
		dev_err(chip->dev, "Couldn't read revision rc = %d\n", rc);


	if( check_battery_id() < 700000 && check_battery_id()>500000)
	{
		chip->vfloat_mv = 4380;
		battery_type = Battery_438V;
	}
	else
	{
		chip->vfloat_mv = 4350;
		battery_type = Battery_435V;
	}
	pr_info("battery type ====> %d \n",battery_type);

	rc = smb_parse_dt(chip);
	if (rc < 0) {
		dev_err(&client->dev, "Unable to parse DT nodes\n");
		return rc;
	}
	device_init_wakeup(chip->dev, 1);
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->irq_complete);
	mutex_init(&chip->charging_disable_lock);
	mutex_init(&chip->current_change_lock);
	chip->default_i2c_addr = client->addr;
	INIT_WORK(&chip->parallel_work, smb1360_parallel_work);
	if (chip->cold_hysteresis || chip->hot_hysteresis)
		INIT_WORK(&chip->jeita_hysteresis_work,
				smb1360_jeita_hysteresis_work);

	pr_debug("default_i2c_addr=%x\n", chip->default_i2c_addr);
	smb1360_otp_backup_pool_init(chip);

	if(chip->HW_ID == ZC550KL_8939_ER || chip->HW_ID == ZC550KL_8939_PR)
		check_otg_ovp_gpio_value(chip);

	rc = smb1360_hw_init(chip);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to intialize hardware rc = %d\n", rc);
		return rc;
	}

	rc = smb1360_regulator_init(chip);
	if  (rc) {
		dev_err(&client->dev,
			"Couldn't initialize smb349 ragulator rc=%d\n", rc);
		return rc;
	}

	rc = determine_initial_status(chip);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to determine init status rc = %d\n", rc);
		goto fail_hw_init;
	}

	chip->batt_psy.name		= "battery";
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= smb1360_battery_get_property;
	chip->batt_psy.set_property	= smb1360_battery_set_property;
	chip->batt_psy.properties	= smb1360_battery_properties;
	chip->batt_psy.num_properties  = ARRAY_SIZE(smb1360_battery_properties);
	chip->batt_psy.external_power_changed = smb1360_external_power_changed;
	chip->batt_psy.property_is_writeable = smb1360_battery_is_writeable;

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to register batt_psy rc = %d\n", rc);
		goto fail_hw_init;
	}

	/* STAT irq configuration */
	if (client->irq) {
		rc = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				smb1360_stat_handler, IRQF_ONESHOT,
				"smb1360_stat_irq", chip);
		if (rc < 0) {
			dev_err(&client->dev,
				"request_irq for irq=%d  failed rc = %d\n",
				client->irq, rc);
			goto unregister_batt_psy;
		}
		enable_irq_wake(client->irq);
	}

	chip->debug_root = debugfs_create_dir("smb1360", NULL);
	if (!chip->debug_root)
		dev_err(chip->dev, "Couldn't create debug dir\n");

	if (chip->debug_root) {
		struct dentry *ent;

		ent = debugfs_create_file("config_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cnfg_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create cnfg debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("status_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &status_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create status debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("irq_status", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &irq_stat_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create irq_stat debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("cmd_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cmd_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create cmd debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("fg_regs",
				S_IFREG | S_IRUGO, chip->debug_root, chip,
					  &fg_regs_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create fg_scratch_pad debug file rc = %d\n",
				rc);

		ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->peek_poke_address));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create address debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &poke_poke_debug_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);

		ent = debugfs_create_x32("fg_address",
					S_IFREG | S_IWUSR | S_IRUGO,
					chip->debug_root,
					&(chip->fg_peek_poke_address));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create address debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("fg_data",
					S_IFREG | S_IWUSR | S_IRUGO,
					chip->debug_root, chip,
					&fg_poke_poke_debug_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);

		ent = debugfs_create_x32("fg_access_type",
					S_IFREG | S_IWUSR | S_IRUGO,
					chip->debug_root,
					&(chip->fg_access_type));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);

		ent = debugfs_create_x32("skip_writes",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->skip_writes));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);

		ent = debugfs_create_x32("skip_reads",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->skip_reads));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("irq_count", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &irq_count_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create count debug file rc = %d\n",
				rc);
	}

	dev_info(chip->dev, "SMB1360 revision=0x%x probe success! batt=%d usb=%d soc=%d\n",
			chip->revision,
			smb1360_get_prop_batt_present(chip),
			chip->usb_present,
			smb1360_get_prop_batt_capacity(chip));

	//asus report capacity frank ++++
	asus_polling_battery_data_work(0);
	//asus report capacity frank ----

#if defined(CONFIG_ASUS_ZC550KL8939_PROJECT)
	wake_lock_init(&adaptor_10W_check_wake_lock, WAKE_LOCK_SUSPEND, "adaptor_10W_check");
	adaptor_10W_check_count = 0;
	asus_adaptor_10W_check_work(1);
#endif

	//asus bsp frank add smb1360_charging_toggle and runin_switch +++
	{
		int retval;


   		retval = sysfs_create_file(&chip->dev->kobj, &runin_switch_on_attribute.attr);

   		if (retval)

      			kobject_put(&chip->dev->kobj);

		retval = sysfs_create_file(&chip->dev->kobj, &runin_switch_off_attribute.attr);

   		if (retval)

      			kobject_put(&chip->dev->kobj);

	}
	//asus bsp frank add smb1360_charging_toggle and runin_switch ---
	create_CSIR_Version_proc_file();
	create_battery_id_proc_file();

	create_ac_current_proc_file();

//ASUS_BSP frank for ChargerDriver stress test +++
#ifdef CONFIG_I2C_STRESS_TEST
      smb1360_i2c_stress_test();
#endif
//ASUS_BSP frank for ChargerDriver stress test ---
	wake_lock_init(&usbin_wake_lock, WAKE_LOCK_SUSPEND, "usbin_wake");

	smb1360_probe_success = true;

	return 0;

unregister_batt_psy:
	power_supply_unregister(&chip->batt_psy);
fail_hw_init:
	regulator_unregister(chip->otg_vreg.rdev);
	power_supply_unregister(&smb1360_power_supplies[0]);
	return rc;
}

static int smb1360_remove(struct i2c_client *client)
{
	struct smb1360_chip *chip = i2c_get_clientdata(client);

	if (gpio_is_valid(chip->otg_enable_gpio))
		gpio_free(chip->otg_enable_gpio);

	if (gpio_is_valid(chip->ovp_enable_gpio))
		gpio_free(chip->ovp_enable_gpio);

	if (gpio_is_valid(chip->boost_enable_gpio))
		gpio_free(chip->boost_enable_gpio);

	regulator_unregister(chip->otg_vreg.rdev);
	power_supply_unregister(&chip->batt_psy);
	mutex_destroy(&chip->charging_disable_lock);
	mutex_destroy(&chip->current_change_lock);
	mutex_destroy(&chip->read_write_lock);
	mutex_destroy(&chip->irq_complete);
	mutex_destroy(&chip->otp_gain_lock);
	mutex_destroy(&chip->fg_access_request_lock);
	debugfs_remove_recursive(chip->debug_root);
	wake_lock_destroy(&usbin_wake_lock);
#if defined(CONFIG_ASUS_ZC550KL8939_PROJECT)
	wake_lock_destroy(&adaptor_10W_check_wake_lock);
#endif
	return 0;
}

static int smb1360_suspend(struct device *dev)
{
	int i, rc;
	struct i2c_client *client = to_i2c_client(dev);
	struct smb1360_chip *chip = i2c_get_clientdata(client);

	/* Save the current IRQ config */
	for (i = 0; i < 3; i++) {
		rc = smb1360_read(chip, IRQ_CFG_REG + i,
					&chip->irq_cfg_mask[i]);
		if (rc)
			pr_err("Couldn't save irq cfg regs rc=%d\n", rc);
	}

	/* enable only important IRQs */
	rc = smb1360_write(chip, IRQ_CFG_REG, IRQ_DCIN_UV_BIT
						| IRQ_AICL_DONE_BIT
						| IRQ_BAT_HOT_COLD_SOFT_BIT
						| IRQ_BAT_HOT_COLD_HARD_BIT);
	if (rc < 0)
		pr_err("Couldn't set irq_cfg rc=%d\n", rc);

	rc = smb1360_write(chip, IRQ2_CFG_REG, IRQ2_BATT_MISSING_BIT
						| IRQ2_VBAT_LOW_BIT
						| IRQ2_POWER_OK_BIT);
	if (rc < 0)
		pr_err("Couldn't set irq2_cfg rc=%d\n", rc);

	rc = smb1360_write(chip, IRQ3_CFG_REG, IRQ3_SOC_FULL_BIT
					| IRQ3_SOC_MIN_BIT
					| IRQ3_SOC_EMPTY_BIT);
	if (rc < 0)
		pr_err("Couldn't set irq3_cfg rc=%d\n", rc);

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = false;
	mutex_unlock(&chip->irq_complete);

	return 0;
}

static int smb1360_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb1360_chip *chip = i2c_get_clientdata(client);

	if (chip->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int smb1360_resume(struct device *dev)
{
	int i, rc;
	struct i2c_client *client = to_i2c_client(dev);
	struct smb1360_chip *chip = i2c_get_clientdata(client);
	//asus report capacity frank ++++
	struct timespec mtNow;
	//asus report capacity frank ----

	//int usb_state = g_usb_state;
	BAT_DBG("%s\n", __FUNCTION__);

	/* Restore the IRQ config */
	for (i = 0; i < 3; i++) {
		rc = smb1360_write(chip, IRQ_CFG_REG + i,
					chip->irq_cfg_mask[i]);
		if (rc)
			pr_err("Couldn't restore irq cfg regs rc=%d\n", rc);
	}

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = true;

	//asus report capacity frank ++++
	mtNow = current_kernel_time();
	/*BSP david: if not update for more than 180s, do report capacity*/
	if (mtNow.tv_sec - g_last_print_time.tv_sec >= REPORT_CAPACITY_POLLING_TIME) {
		asus_polling_battery_data_work(0);
	}
	//asus report capacity frank ----

	if (chip->irq_waiting) {
		chip->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&chip->irq_complete);
		smb1360_stat_handler(client->irq, chip);
	} else {
		mutex_unlock(&chip->irq_complete);
	}

	power_supply_changed(&chip->batt_psy);

	return 0;
}

static void smb1360_shutdown(struct i2c_client *client)
{
	int rc;
	struct smb1360_chip *chip = i2c_get_clientdata(client);

	rc = smb1360_otg_disable(chip);
	if (rc)
		pr_err("Couldn't disable OTG mode rc=%d\n", rc);

	if (chip->shdn_after_pwroff) {
		rc = smb1360_poweroff(chip);
		if (rc)
			pr_err("Couldn't shutdown smb1360, rc = %d\n", rc);
		pr_info("smb1360 power off\n");
	}
}

static const struct dev_pm_ops smb1360_pm_ops = {
	.resume		= smb1360_resume,
	.suspend_noirq	= smb1360_suspend_noirq,
	.suspend	= smb1360_suspend,
};

static struct of_device_id smb1360_match_table[] = {
	{ .compatible = "qcom,smb1360-chg-fg",},
	{ },
};

static const struct i2c_device_id smb1360_id[] = {
	{"smb1360-chg-fg", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb1360_id);

static struct i2c_driver smb1360_driver = {
	.driver		= {
		.name		= "smb1360-chg-fg",
		.owner		= THIS_MODULE,
		.of_match_table	= smb1360_match_table,
		.pm		= &smb1360_pm_ops,
	},
	.probe		= smb1360_probe,
	.remove		= smb1360_remove,
	.shutdown	= smb1360_shutdown,
	.id_table	= smb1360_id,
};
int __init
smb1360_driver_init(void)
{
	return i2c_add_driver(&smb1360_driver);
}
late_initcall(smb1360_driver_init);
//module_i2c_driver(smb1360_driver);
static void __exit
smb1360_driver_exit(void)
{
	i2c_del_driver(&smb1360_driver);
}
module_exit(smb1360_driver_exit);
MODULE_DESCRIPTION("SMB1360 Charger and Fuel Gauge");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:smb1360-chg-fg");
