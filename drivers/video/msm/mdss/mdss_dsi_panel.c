/* Copyright (c) 2012-2016, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>
#include <linux/string.h>

#include "mdss_dsi.h"
#include "mdss_livedisplay.h"

#define DT_CMD_HDR 6

/* NT35596 panel specific status variables */
#define NT35596_BUF_3_STATUS 0x02
#define NT35596_BUF_4_STATUS 0x40
#define NT35596_BUF_5_STATUS 0x80
#define NT35596_MAX_ERR_CNT 2

#define MIN_REFRESH_RATE 48
#define DEFAULT_MDP_TRANSFER_TIME 14000

//ASUS_BSP: Louis +++
#include <linux/msm_mdp.h>
#include "mdss_panel.h"


#ifndef CONFIG_ASUS_ZC550KL_PROJECT
extern void rt4532_suspend(void);
extern void rt4532_resume(void);
extern void ftxxxx_ts_suspend(void);
extern void ftxxxx_ts_resume(void);
#else
//ASUS_BSP:Freeman +++
extern void ftxxxx_ts_suspend(void);
extern void ftxxxx_ts_resume(void);
//ASUS_BSP:Freeman ---
#endif

extern struct mdss_panel_data *g_mdss_pdata;
#ifndef CONFIG_ASUS_ZC550KL_PROJECT
extern char lcd_unique_id[64];
#else

extern int g_CHG_mode;
#endif
void set_tcon_cmd(char *cmd, short len);
void get_tcon_cmd(char cmd, int rlen);
static struct mutex cmd_mutex;
char bl_cmd[2] = {0x51, 0x64};
#ifndef CONFIG_ASUS_ZC550KL_PROJECT
char dimming_cmd[2] = {0x53, 0x2C};

#endif
#ifndef CONFIG_ASUS_ZC550KL_PROJECT
#ifndef ASUS_FACTORY_BUILD
char cabc_mode[2] = {0x55, Still_MODE};
#else
char cabc_mode[2] = {0x55, OFF_MODE};
#endif
#else
char cabc_mode[2] = {0x55, Still_MODE};

#endif

#ifndef CONFIG_ASUS_ZC550KL_PROJECT
static struct panel_list supp_panels[] = {
	{"AUO", ZE500KL_LCD_AUO},
	{"TM", ZE500KL_LCD_TIANMA},
	{"BOE", ZE500KL_LCD_BOE},
	{"HSD", A500_HSD},
};
//ASUS_BSP: Louis ---

//Bernard, dynamic calibration backlight +++
#endif
#include <linux/syscalls.h>
#include <linux/proc_fs.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#ifndef CONFIG_ASUS_ZC550KL_PROJECT
#define BL_DEFAULT 450
#define BL_BOUNDARY_VAL 12
#define LCD_CALIBRATION_PATH "/factory/lcd_calibration.ini"
#define BL_CALIBRATION_PATH "/data/data/bl_adjust"
#define SHIFT_MASK 23

#define ON "on"
uint8_t g_lcd_uniqueId_crc = -1;
int g_asus_lcdID_verify = 0;
int g_calibrated_bl = -1;
int g_actual_bl = -1;
int g_adjust_bl = 0;
int g_dcs_bl_value = 0;

static mm_segment_t oldfs;

static void initKernelEnv(void)
{
    oldfs = get_fs();
    set_fs(KERNEL_DS);
}

static void deinitKernelEnv(void)
{
    set_fs(oldfs);
}

typedef void (*lcd_func)( const char *msg, int index);

struct lcd_command{
    char *id;
    int len;
    const lcd_func func;
};

bool lcd_adjust_backlight(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5)
{
    unsigned int readstr;
    unsigned int asus_lcdID, raw_data, mantissa;
    int mantissa_val=0;
    int exp, i;
    int readlen =0;
    struct inode *inode;
    struct file *fp = NULL;
    unsigned char *buf = NULL;

    sscanf(lcd_unique_id, "%x", &readstr);
    asus_lcdID = ((readstr>>24) ^ 0x55 ^ (readstr>>16) ^ (readstr>>8) ^ (readstr)) & 0xff;
    raw_data = (byte2<<24) | (byte3<<16) | (byte4<<8) | byte5;

    printk("[Display] Panel Unique ID from Phone: 0x%x\n", asus_lcdID);
    printk("[Display] Panel Unique ID from File : 0x%x\n", byte1);

    exp = ((raw_data & 0x7F800000) >> SHIFT_MASK);
    exp = exp - 127;

    mantissa = raw_data & 0x7FFFFF;
    for(i=0; i<=SHIFT_MASK; i++) {    /*bit 23 = 1/2, bit 22 = 1/4*/
        if ((mantissa >> (SHIFT_MASK - i)) & 0x01)
            mantissa_val += 100000/(0x01 << i);
    }

    g_actual_bl = (0x01 << exp) + ((0x01 << exp) * mantissa_val / 100000);
    printk("[Display] Actual Panel Backlight value = %d \n", g_actual_bl);

    if (asus_lcdID == byte1) {
        initKernelEnv();
        fp = filp_open(BL_CALIBRATION_PATH, O_RDONLY, 0);
        if (!IS_ERR_OR_NULL(fp)) {
            inode = fp->f_dentry->d_inode;
            buf = kmalloc(inode->i_size, GFP_KERNEL);

            if (fp->f_op != NULL && fp->f_op->read != NULL) {
                readlen = fp->f_op->read(fp, buf, inode->i_size, &(fp->f_pos));
                buf[readlen] = '\0';
            }
            sscanf(buf, "%d", &g_adjust_bl);
            printk("[Display] Dynamic calibration Backlight value = %d\n", g_adjust_bl);

            g_asus_lcdID_verify = 1;
            deinitKernelEnv();
            filp_close(fp, NULL);
            kfree(buf);
        } else {
			g_adjust_bl = BL_DEFAULT;
            pr_err("[Display] Dynamic calibration Backlight disable\n");
            return false;
        }
    } else
        return false;

    return true;
}

bool lcd_read_calibration_file(void)
{
    int ret;
    int readlen =0;
    off_t fsize;
    struct file *fp = NULL;
    struct inode *inode;
    uint8_t *buf = NULL;

    initKernelEnv();

    fp = filp_open(LCD_CALIBRATION_PATH, O_RDONLY, 0);
    if (IS_ERR_OR_NULL(fp)) {
        pr_err("[Display] Read (%s) failed\n", LCD_CALIBRATION_PATH);
        return false;
    }

    inode = fp->f_dentry->d_inode;
    fsize = inode->i_size;
    buf = kmalloc(fsize, GFP_KERNEL);

    if (fp->f_op != NULL && fp->f_op->read != NULL) {
        readlen = fp->f_op->read(fp, buf, fsize, &(fp->f_pos));
        if (readlen < 0) {
            DEV_ERR("[Display] Read (%s) error\n", LCD_CALIBRATION_PATH);
            deinitKernelEnv();
            filp_close(fp, NULL);
            kfree(buf);
            return false;
        }
    } else
        pr_err("[Display] Read (%s) f_op=NULL or op->read=NULL\n", LCD_CALIBRATION_PATH);

    deinitKernelEnv();
    filp_close(fp, NULL);

    g_calibrated_bl = (buf[60]<<8) + buf[61];
    printk("[Display] Panel Calibrated Backlight: %d\n", g_calibrated_bl);
	g_lcd_uniqueId_crc = buf[5];

    ret = lcd_adjust_backlight(buf[5], buf[55], buf[54], buf[53], buf[52]);
    if(!ret)
        g_asus_lcdID_verify = 0;

    kfree(buf);
    return true;
}

void lcd_bl_calibration(const char *msg, int index)
{
    int ret;
    ret = lcd_read_calibration_file();
    if(!ret)
        pr_err("[Display] Dynamic Calibration Backlight disable\n");
}

struct lcd_command lcd_cmd_tb[] = {
    {ON, sizeof(ON), lcd_bl_calibration},		/*do calibration*/
    //{BL, sizeof(BL), lcd_read_bl},		/*read backlight*/
};

static void lcd_write(char *msg)
{
    int i = 0;
    for(; i<ARRAY_SIZE(lcd_cmd_tb);i++){
        if(strncmp(msg, lcd_cmd_tb[i].id, lcd_cmd_tb[i].len-1)==0){
            lcd_cmd_tb[i].func(msg, lcd_cmd_tb[i].len-1);
            return;
        }
    }
    pr_err("### error Command no found in  %s ###\n", __FUNCTION__);
    return;
}

static ssize_t lcd_info_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
    char messages[256];

    memset(messages, 0, sizeof(messages));

    if (len > 256)
        len = 256;
    if (copy_from_user(messages, buff, len))
        return -EFAULT;

    pr_info("[Dislay] ### %s ###\n", __func__);

    lcd_write(messages);

    return len;
}

static ssize_t lcd_info_read(struct file *file, char __user *buf,
                             size_t count, loff_t *ppos)
{
    int len = 0;
    ssize_t ret = 0;
    char *buff;

    buff = kmalloc(512, GFP_KERNEL);
    if (!buff)
        return -ENOMEM;

    len += sprintf(buff, "===================\nPanel Vendor: %s\nUnique ID: %s\nUnique ID after CRC: %x\nSW Calibration Enable: %s\nCalibrated Value: %d\nActual Backlight: %d\nAdjust Backlight: %d\nCABC Mode: %d\n===================\n", supp_panels[g_asus_lcdID].name, lcd_unique_id, g_lcd_uniqueId_crc, g_asus_lcdID_verify ? "Y" : "N", g_calibrated_bl, g_actual_bl, g_adjust_bl, cabc_mode[1]);
    ret = simple_read_from_buffer(buf, count, ppos, buff, len);
    kfree(buff);

    return ret;
}

static struct file_operations lcd_info_proc_ops = {
    .write = lcd_info_write,
    .read = lcd_info_read,
};
//ASUS BSP Bernard, dynamic calibration backlight ---
#endif
DEFINE_LED_TRIGGER(bl_led_trigger);

void mdss_dsi_panel_pwm_cfg(struct mdss_dsi_ctrl_pdata *ctrl)
{
	if (ctrl->pwm_pmi)
		return;

	ctrl->pwm_bl = pwm_request(ctrl->pwm_lpg_chan, "lcd-bklt");
	if (ctrl->pwm_bl == NULL || IS_ERR(ctrl->pwm_bl)) {
		pr_err("%s: Error: lpg_chan=%d pwm request failed",
				__func__, ctrl->pwm_lpg_chan);
	}
	ctrl->pwm_enabled = 0;
}

static void mdss_dsi_panel_bklt_pwm(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	int ret;
	u32 duty;
	u32 period_ns;

	if (ctrl->pwm_bl == NULL) {
		pr_err("%s: no PWM\n", __func__);
		return;
	}

	if (level == 0) {
		if (ctrl->pwm_enabled) {
			ret = pwm_config_us(ctrl->pwm_bl, level,
					ctrl->pwm_period);
			if (ret)
				pr_err("%s: pwm_config_us() failed err=%d.\n",
						__func__, ret);
			pwm_disable(ctrl->pwm_bl);
		}
		ctrl->pwm_enabled = 0;
		return;
	}

	duty = level * ctrl->pwm_period;
	duty /= ctrl->bklt_max;

	pr_debug("%s: bklt_ctrl=%d pwm_period=%d pwm_gpio=%d pwm_lpg_chan=%d\n",
			__func__, ctrl->bklt_ctrl, ctrl->pwm_period,
				ctrl->pwm_pmic_gpio, ctrl->pwm_lpg_chan);

	pr_debug("%s: ndx=%d level=%d duty=%d\n", __func__,
					ctrl->ndx, level, duty);

	if (ctrl->pwm_period >= USEC_PER_SEC) {
		ret = pwm_config_us(ctrl->pwm_bl, duty, ctrl->pwm_period);
		if (ret) {
			pr_err("%s: pwm_config_us() failed err=%d.\n",
					__func__, ret);
			return;
		}
	} else {
		period_ns = ctrl->pwm_period * NSEC_PER_USEC;
		ret = pwm_config(ctrl->pwm_bl,
				level * period_ns / ctrl->bklt_max,
				period_ns);
		if (ret) {
			pr_err("%s: pwm_config() failed err=%d.\n",
					__func__, ret);
			return;
		}
	}

	if (!ctrl->pwm_enabled) {
		ret = pwm_enable(ctrl->pwm_bl);
		if (ret)
			pr_err("%s: pwm_enable() failed err=%d\n", __func__,
				ret);
		ctrl->pwm_enabled = 1;
	}
}

static char dcs_cmd[2] = {0x54, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc dcs_read_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(dcs_cmd)},
	dcs_cmd
};

u32 mdss_dsi_panel_cmd_read(struct mdss_dsi_ctrl_pdata *ctrl, char cmd0,
		char cmd1, void (*fxn)(int), char *rbuf, int len)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;

	pinfo = &(ctrl->panel_data.panel_info);
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			return -EINVAL;
	}

	dcs_cmd[0] = cmd0;
	dcs_cmd[1] = cmd1;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = len;
	cmdreq.rbuf = rbuf;
	cmdreq.cb = fxn; /* call back */
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	/*
	 * blocked here, until call back called
	 */

	return 0;
}

void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;

	pinfo = &(ctrl->panel_data.panel_info);
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			return;
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;

	/*Panel ON/Off commands should be sent in DSI Low Power Mode*/
	if (pcmds->link_state == DSI_LP_MODE)
		cmdreq.flags  |= CMD_REQ_LP_MODE;
	else if (pcmds->link_state == DSI_HS_MODE)
		cmdreq.flags |= CMD_REQ_HS_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

extern int display_commit_cnt; //ASUS BSP Bernard: frame commit count

static void mdss_dsi_panel_bklt_dcs(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	struct mdss_panel_info *pinfo;
#ifndef CONFIG_ASUS_ZC550KL_PROJECT
	//ASUS BSP Bernard +++
    int tmp_level = 0;
#ifndef ASUS_FACTORY_BUILD
	static bool g_bl_first_bootup = true;
    int rc;
#endif
#else

	int calc_res = 100000;
	int phone_min_value =12;
	int phone_max_value =255 ;
	int phone_min_index = 16;
	int phone_max_index = 255;
	int tmp_level = 0;

#endif
#ifndef CONFIG_ASUS_ZC550KL_PROJECT
    if (level == 0) {
        printk("[BL] Set %s brightness (%d) return func.\n", supp_panels[g_asus_lcdID].name, level);
        return;
    }
	//ASUS BSP Bernard ---

#else
	int div = ((phone_max_index-phone_min_index)*calc_res/(phone_max_value-phone_min_value));

	int shift = phone_min_index -phone_min_value*calc_res/div;
	tmp_level = level;

#endif
	pinfo = &(ctrl->panel_data.panel_info);
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			return;
	}
#ifndef CONFIG_ASUS_ZC550KL_PROJECT
    //ASUS BSP Bernard +++
#ifndef ASUS_FACTORY_BUILD
    if (g_bl_first_bootup) {
        rc = lcd_read_calibration_file();
        if(!rc)
            pr_err("%s: Backlight calibration read failed\n", __func__);
        g_bl_first_bootup = false;
    }
#endif

    //ASUS BSP Bernard ---

    pr_debug("%s: level=%d\n", __func__, level);
    tmp_level = level;

#else
	if (level == 0) {
		//printk("[BL] %s turn off Phone backlight\n",__func__);
		level = 0;
		}
	else if (level >= phone_max_value)
			level = phone_max_index;
	else if (level > 0 && level <= phone_min_value)
		level = phone_min_index;
	else if (level > phone_min_value && level < phone_max_value)
		{
			level = level*calc_res/div + shift;
			if(level >= phone_max_index)
				level = phone_max_index;
		}

#endif
#ifndef CONFIG_ASUS_ZC550KL_PROJECT
#ifndef ASUS_FACTORY_BUILD
    if ((g_adjust_bl < g_calibrated_bl) && (g_adjust_bl > 0))
        level = level * g_adjust_bl / g_calibrated_bl;
#endif

    if (level < BL_BOUNDARY_VAL)
        level = BL_BOUNDARY_VAL;

    printk("[BL] Set %s brightness (Actual:%d) (Adjust:%d)\n", supp_panels[g_asus_lcdID].name, tmp_level, level);

    bl_cmd[1] = level;
	if(display_commit_cnt <= 4)
        set_tcon_cmd(bl_cmd, ARRAY_SIZE(bl_cmd));

    if (tmp_level >= BL_BOUNDARY_VAL)
        g_dcs_bl_value = level;
#else
//	printk("[Hyde][BL] Setbrightness (Actual:%d) (Adjust:%d)\n", level, tmp_level);
	bl_cmd[1] = level;
	set_tcon_cmd(bl_cmd, ARRAY_SIZE(bl_cmd));
#endif
}

static int mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		rc = gpio_request(ctrl_pdata->disp_en_gpio,
						"disp_enable");
		if (rc) {
			pr_err("request disp_en gpio failed, rc=%d\n",
				       rc);
			goto disp_en_gpio_err;
		}
	}
	rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
	if (rc) {
		pr_err("request reset gpio failed, rc=%d\n",
			rc);
		goto rst_gpio_err;
	}
	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
		rc = gpio_request(ctrl_pdata->bklt_en_gpio,
						"bklt_enable");
		if (rc) {
			pr_err("request bklt gpio failed, rc=%d\n",
				       rc);
			goto bklt_en_gpio_err;
		}
	}
	if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
		rc = gpio_request(ctrl_pdata->mode_gpio, "panel_mode");
		if (rc) {
			pr_err("request panel mode gpio failed,rc=%d\n",
								rc);
			goto mode_gpio_err;
		}
	}
	return rc;

mode_gpio_err:
	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
		gpio_free(ctrl_pdata->bklt_en_gpio);
bklt_en_gpio_err:
	gpio_free(ctrl_pdata->rst_gpio);
rst_gpio_err:
	if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
		gpio_free(ctrl_pdata->disp_en_gpio);
disp_en_gpio_err:
	return rc;
}

int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int i, rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	pr_debug("%s: enable = %d\n", __func__, enable);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
		rc = mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}
		if (!pinfo->cont_splash_enabled) {
			if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
				gpio_set_value((ctrl_pdata->disp_en_gpio), 1);

			for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				gpio_set_value((ctrl_pdata->rst_gpio),
					pdata->panel_info.rst_seq[i]);
				if (pdata->panel_info.rst_seq[++i])
					usleep(pinfo->rst_seq[i] * 1000);
			}

			if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
				gpio_set_value((ctrl_pdata->bklt_en_gpio), 1);
		}

		if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
			if (pinfo->mode_gpio_state == MODE_GPIO_HIGH)
				gpio_set_value((ctrl_pdata->mode_gpio), 1);
			else if (pinfo->mode_gpio_state == MODE_GPIO_LOW)
				gpio_set_value((ctrl_pdata->mode_gpio), 0);
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
			gpio_set_value((ctrl_pdata->bklt_en_gpio), 0);
			gpio_free(ctrl_pdata->bklt_en_gpio);
		}
		if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
			gpio_free(ctrl_pdata->disp_en_gpio);
		}
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
		gpio_free(ctrl_pdata->rst_gpio);
		if (gpio_is_valid(ctrl_pdata->mode_gpio))
			gpio_free(ctrl_pdata->mode_gpio);
	}
	return rc;
}

/**
 * mdss_dsi_roi_merge() -  merge two roi into single roi
 *
 * Function used by partial update with only one dsi intf take 2A/2B
 * (column/page) dcs commands.
 */
static int mdss_dsi_roi_merge(struct mdss_dsi_ctrl_pdata *ctrl,
					struct mdss_rect *roi)
{
	struct mdss_panel_info *l_pinfo;
	struct mdss_rect *l_roi;
	struct mdss_rect *r_roi;
	struct mdss_dsi_ctrl_pdata *other = NULL;
	int ans = 0;

	if (ctrl->ndx == DSI_CTRL_LEFT) {
		other = mdss_dsi_get_ctrl_by_index(DSI_CTRL_RIGHT);
		if (!other)
			return ans;
		l_pinfo = &(ctrl->panel_data.panel_info);
		l_roi = &(ctrl->panel_data.panel_info.roi);
		r_roi = &(other->panel_data.panel_info.roi);
	} else  {
		other = mdss_dsi_get_ctrl_by_index(DSI_CTRL_LEFT);
		if (!other)
			return ans;
		l_pinfo = &(other->panel_data.panel_info);
		l_roi = &(other->panel_data.panel_info.roi);
		r_roi = &(ctrl->panel_data.panel_info.roi);
	}

	if (l_roi->w == 0 && l_roi->h == 0) {
		/* right only */
		*roi = *r_roi;
		roi->x += l_pinfo->xres;/* add left full width to x-offset */
	} else {
		/* left only and left+righ */
		*roi = *l_roi;
		roi->w +=  r_roi->w; /* add right width */
		ans = 1;
	}

	return ans;
}

static char caset[] = {0x2a, 0x00, 0x00, 0x03, 0x00};	/* DTYPE_DCS_LWRITE */
static char paset[] = {0x2b, 0x00, 0x00, 0x05, 0x00};	/* DTYPE_DCS_LWRITE */

/* pack into one frame before sent */
static struct dsi_cmd_desc set_col_page_addr_cmd[] = {
	{{DTYPE_DCS_LWRITE, 0, 0, 0, 1, sizeof(caset)}, caset},	/* packed */
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(paset)}, paset},
};

static void mdss_dsi_send_col_page_addr(struct mdss_dsi_ctrl_pdata *ctrl,
					struct mdss_rect *roi)
{
	struct dcs_cmd_req cmdreq;

	roi = &ctrl->roi;

	caset[1] = (((roi->x) & 0xFF00) >> 8);
	caset[2] = (((roi->x) & 0xFF));
	caset[3] = (((roi->x - 1 + roi->w) & 0xFF00) >> 8);
	caset[4] = (((roi->x - 1 + roi->w) & 0xFF));
	set_col_page_addr_cmd[0].payload = caset;

	paset[1] = (((roi->y) & 0xFF00) >> 8);
	paset[2] = (((roi->y) & 0xFF));
	paset[3] = (((roi->y - 1 + roi->h) & 0xFF00) >> 8);
	paset[4] = (((roi->y - 1 + roi->h) & 0xFF));
	set_col_page_addr_cmd[1].payload = paset;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds_cnt = 2;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL | CMD_REQ_UNICAST;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	cmdreq.cmds = set_col_page_addr_cmd;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static int mdss_dsi_set_col_page_addr(struct mdss_panel_data *pdata)
{
	struct mdss_panel_info *pinfo;
	struct mdss_rect roi;
	struct mdss_rect *p_roi;
	struct mdss_rect *c_roi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_dsi_ctrl_pdata *other = NULL;
	int left_or_both = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &pdata->panel_info;
	p_roi = &pinfo->roi;

	/*
	 * to avoid keep sending same col_page info to panel,
	 * if roi_merge enabled, the roi of left ctrl is used
	 * to compare against new merged roi and saved new
	 * merged roi to it after comparing.
	 * if roi_merge disabled, then the calling ctrl's roi
	 * and pinfo's roi are used to compare.
	 */
	if (pinfo->partial_update_roi_merge) {
		left_or_both = mdss_dsi_roi_merge(ctrl, &roi);
		other = mdss_dsi_get_ctrl_by_index(DSI_CTRL_LEFT);
		c_roi = &other->roi;
	} else {
		c_roi = &ctrl->roi;
		roi = *p_roi;
	}

	/* roi had changed, do col_page update */
	if (!mdss_rect_cmp(c_roi, &roi)) {
		pr_debug("%s: ndx=%d x=%d y=%d w=%d h=%d\n",
				__func__, ctrl->ndx, p_roi->x,
				p_roi->y, p_roi->w, p_roi->h);

		*c_roi = roi; /* keep to ctrl */
		if (c_roi->w == 0 || c_roi->h == 0) {
			/* no new frame update */
			pr_debug("%s: ctrl=%d, no partial roi set\n",
						__func__, ctrl->ndx);
			return 0;
		}

		if (pinfo->dcs_cmd_by_left) {
			if (left_or_both && ctrl->ndx == DSI_CTRL_RIGHT) {
				/* 2A/2B sent by left already */
				return 0;
			}
		}

		if (!mdss_dsi_sync_wait_enable(ctrl)) {
			if (pinfo->dcs_cmd_by_left)
				ctrl = mdss_dsi_get_ctrl_by_index(
							DSI_CTRL_LEFT);
			mdss_dsi_send_col_page_addr(ctrl, &roi);
		} else {
			/*
			 * when sync_wait_broadcast enabled,
			 * need trigger at right ctrl to
			 * start both dcs cmd transmission
			 */
			other = mdss_dsi_get_other_ctrl(ctrl);
			if (!other)
				goto end;

			if (mdss_dsi_is_left_ctrl(ctrl)) {
				mdss_dsi_send_col_page_addr(ctrl, &ctrl->roi);
				mdss_dsi_send_col_page_addr(other, &other->roi);
			} else {
				mdss_dsi_send_col_page_addr(other, &other->roi);
				mdss_dsi_send_col_page_addr(ctrl, &ctrl->roi);
			}
		}
	}

end:
	return 0;
}

static void mdss_dsi_panel_switch_mode(struct mdss_panel_data *pdata,
							int mode)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mipi_panel_info *mipi;
	struct dsi_panel_cmds *pcmds;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	mipi  = &pdata->panel_info.mipi;

	if (!mipi->dynamic_switch_enabled)
		return;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (mode == DSI_CMD_MODE)
		pcmds = &ctrl_pdata->video2cmd;
	else
		pcmds = &ctrl_pdata->cmd2video;

	mdss_dsi_panel_cmds_send(ctrl_pdata, pcmds);

	return;
}

static void mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata,
							u32 bl_level)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_dsi_ctrl_pdata *sctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	/*
	 * Some backlight controllers specify a minimum duty cycle
	 * for the backlight brightness. If the brightness is less
	 * than it, the controller can malfunction.
	 */

	pr_debug("[Display] %s: bl_level=%d\n" ,__func__, bl_level);	/*ASUS_BSP: Louis ++ */

#ifndef CONFIG_ASUS_ZC550KL_PROJECT

	if ((bl_level < pdata->panel_info.bl_min) && (bl_level != 0))
		bl_level = pdata->panel_info.bl_min;
#endif

	switch (ctrl_pdata->bklt_ctrl) {
	case BL_WLED:
		led_trigger_event(bl_led_trigger, bl_level);
		break;
	case BL_PWM:
		mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
		break;
	case BL_DCS_CMD:
		if (!mdss_dsi_sync_wait_enable(ctrl_pdata)) {
			mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
			break;
		}
		/*
		 * DCS commands to update backlight are usually sent at
		 * the same time to both the controllers. However, if
		 * sync_wait is enabled, we need to ensure that the
		 * dcs commands are first sent to the non-trigger
		 * controller so that when the commands are triggered,
		 * both controllers receive it at the same time.
		 */
		sctrl = mdss_dsi_get_other_ctrl(ctrl_pdata);
		if (mdss_dsi_sync_wait_trigger(ctrl_pdata)) {
			if (sctrl)
				mdss_dsi_panel_bklt_dcs(sctrl, bl_level);
			mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
		} else {
			mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
			if (sctrl)
				mdss_dsi_panel_bklt_dcs(sctrl, bl_level);
		}
		break;
	default:
		pr_err("%s: Unknown bl_ctrl configuration\n",
			__func__);
		break;
	}
}

#ifdef CONFIG_QPNP_VM_BMS_SUSPEND_PREDICT
void bms_modify_soc_early_suspend(void);
void bms_modify_soc_late_resume(void);
#endif

static int mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%pK ndx=%d\n", __func__, ctrl, ctrl->ndx);
	printk("[Display] %s: ++\n", __func__);

	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			goto end;
	}

	if (ctrl->on_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->on_cmds);
#ifndef CONFIG_ASUS_ZC550KL_PROJECT
    set_tcon_cmd(cabc_mode, ARRAY_SIZE(cabc_mode)); //ASUS_BSP: Louis+++, restore cabc level
	
	rt4532_resume();/*Austin+++*/

	ftxxxx_ts_resume();/*Jacob+++*/

#ifdef CONFIG_QPNP_VM_BMS_SUSPEND_PREDICT
	bms_modify_soc_late_resume();
#endif
#else

	ftxxxx_ts_resume();		//Freeman +++

#endif

end:
	pinfo->blank_state = MDSS_PANEL_BLANK_UNBLANK;
	pr_debug("%s:-\n", __func__);
	printk("[Display] %s: --\n", __func__);
	return 0;
}

static int mdss_dsi_post_panel_on(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	struct dsi_panel_cmds *on_cmds;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%pK ndx=%d\n", __func__, ctrl, ctrl->ndx);

	pinfo = &pdata->panel_info;
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			goto end;
	}

	on_cmds = &ctrl->post_panel_on_cmds;

	pr_debug("%s: ctrl=%pK cmd_cnt=%d\n", __func__, ctrl, on_cmds->cmd_cnt);

	if (on_cmds->cmd_cnt) {
		msleep(50);	/* wait for 3 vsync passed */
		mdss_dsi_panel_cmds_send(ctrl, on_cmds);
	}

end:
	pr_debug("%s:-\n", __func__);
	return 0;
}

static int mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%pK ndx=%d\n", __func__, ctrl, ctrl->ndx);
	printk("[Display] %s: ++\n", __func__);
#ifndef CONFIG_ASUS_ZC550KL_PROJECT

	rt4532_suspend();/*Austin+++*/

	ftxxxx_ts_suspend();/*jacob+++*/

#ifdef CONFIG_QPNP_VM_BMS_SUSPEND_PREDICT
	bms_modify_soc_early_suspend();
#endif
#else

	ftxxxx_ts_suspend();		//Freeman +++
#endif

	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			goto end;
	}

	if (ctrl->off_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds);

end:
	pinfo->blank_state = MDSS_PANEL_BLANK_BLANK;
	pr_debug("%s:-\n", __func__);
	printk("[Display] %s: --\n", __func__);
	return 0;
}

static int mdss_dsi_panel_low_power_config(struct mdss_panel_data *pdata,
	int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%pK ndx=%d enable=%d\n", __func__, ctrl, ctrl->ndx,
		enable);

	/* Any panel specific low power commands/config */
	if (enable)
		pinfo->blank_state = MDSS_PANEL_BLANK_LOW_POWER;
	else
		pinfo->blank_state = MDSS_PANEL_BLANK_UNBLANK;

	pr_debug("%s:-\n", __func__);
	return 0;
}

//Louis, Panel debug control interface +++
#define LCD_REGISTER_RW         "driver/panel_reg_rw"

#ifndef CONFIG_ASUS_ZC550KL_PROJECT
#define DUMP_LCD_REGISTER       "driver/panel_reg"
#define DUMP_CALIBRATION_INFO   "driver/panel_info"
#define PANEL_REG_FILE       	"/data/data/ze500kl_panel_reg.txt"
#define ZE500KL_LCD_ID			"lcd_id"
#define ZE500KL_LCD_UNIQUE_ID   "lcd_unique_id"

// read lcd unique id for factory +++
static char otm1284a_Id_page1[] = {0x00, 0x00};
static char otm1284a_Id_page2[] = {0xFF, 0x12, 0x84, 0x01};
static char otm1284a_Id_page3[] = {0x00, 0x80};
static char otm1284a_Id_page4[] = {0xFF, 0x12, 0x84};
static char otm1284a_Id1_cmd[] = {0x00, 0xD1};

static struct dsi_cmd_desc tm_id_page_cmd[] = {
    {{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(otm1284a_Id_page1)}, otm1284a_Id_page1},
	{{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(otm1284a_Id_page2)}, otm1284a_Id_page2},
	{{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(otm1284a_Id_page3)}, otm1284a_Id_page3},
	{{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(otm1284a_Id_page4)}, otm1284a_Id_page4}
};

static struct dsi_cmd_desc tm_id1_cmd = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(otm1284a_Id1_cmd)},
	otm1284a_Id1_cmd
};

static char read_otm1284a_id_cmd[] = {0xF4, 0x00};
static struct dsi_cmd_desc read_tm_id_start_cmd = {
    {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(read_otm1284a_id_cmd)},
	read_otm1284a_id_cmd
};

static u32 dump_TM_chip_uniqueId(void)
{
    struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
    struct dcs_cmd_req cmdreq;
    char *rbuffer;

	ctrl_pdata = container_of(g_mdss_pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	mutex_lock(&cmd_mutex);

	cmdreq.cmds = tm_id_page_cmd;
	cmdreq.cmds_cnt = ARRAY_SIZE(tm_id_page_cmd);
	cmdreq.flags = CMD_CLK_CTRL | CMD_REQ_COMMIT;
	cmdreq.rlen = 0;;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	cmdreq.cmds = &tm_id1_cmd;
	cmdreq.cmds_cnt = 1;
	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	memset(&cmdreq, 0, sizeof(cmdreq));
	rbuffer = kmalloc(sizeof(ctrl_pdata->rx_buf.len), GFP_KERNEL);

	cmdreq.cmds = &read_tm_id_start_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = 4;	//read back 4 byte reg from F4D1
	cmdreq.rbuf = rbuffer;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);
	printk("[Display] TM id: 0x%x, 0x%x, 0x%x, 0x%x\n", *(cmdreq.rbuf), *(cmdreq.rbuf+1), 
								*(cmdreq.rbuf+2), *(cmdreq.rbuf+3));

	kfree(rbuffer);
	mutex_unlock(&cmd_mutex);

    return 0;
}

static ssize_t unique_id_proc_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
    char messages[256];

    memset(messages, 0, sizeof(messages));

    if (len > 256)
        len = 256;
    if (copy_from_user(messages, buff, len))
        return -EFAULT;

    initKernelEnv();

	if (strncmp(messages, "tm", 1) == 0)
		dump_TM_chip_uniqueId();


    deinitKernelEnv(); 
    return len;
}

static ssize_t unique_id_proc_read(struct file *file, char __user *buf,
                    size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kmalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%s\n", lcd_unique_id);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct file_operations lcd_uniqueID_proc_ops = {
    .read = unique_id_proc_read,
	.write = unique_id_proc_write,
};
// read lcd unique id for factory ---

// read lcd id info +++
static ssize_t id_proc_read(struct file *file, char __user *buf,
                    size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kmalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%x\n", g_asus_lcdID);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct file_operations lcd_id_proc_ops = {
    .read = id_proc_read,
};
// read lcd id info ---
#endif

static void dump_register_cb(int len) {
}

#define PANEL_CMD 0

void set_tcon_cmd(char *cmd, short len)
{
    struct dcs_cmd_req cmdreq;
    struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
    struct dsi_cmd_desc tcon_cmd = {
        {DTYPE_DCS_WRITE1, 1, 0, 0, 1, len}, cmd};
    int i=0;

    if(len > 2)
        tcon_cmd.dchdr.dtype = DTYPE_GEN_LWRITE;

    for(i=0; i<len && PANEL_CMD; i++)
         pr_info("[Display] cmd%d (0x%02x)\n", i, cmd[i]);

    ctrl_pdata = container_of(g_mdss_pdata, struct mdss_dsi_ctrl_pdata,
                panel_data);

    mutex_lock(&cmd_mutex);

    if (g_mdss_pdata->panel_info.panel_power_state == MDSS_PANEL_POWER_ON) {
        pr_info("[Display] write parameter: 0x%02x = 0x%02x\n", cmd[0], cmd[1]);
        memset(&cmdreq, 0, sizeof(cmdreq));
        cmdreq.cmds = &tcon_cmd;
        cmdreq.cmds_cnt = 1;
        cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
        cmdreq.rlen = 0;
        cmdreq.cb = NULL;
        mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);
    } else {
        pr_err("[Display] write parameter failed\n");
    }
    mutex_unlock(&cmd_mutex);
}
EXPORT_SYMBOL(set_tcon_cmd);

void get_tcon_cmd(char cmd, int rlen)
{
    struct dcs_cmd_req cmdreq;
    struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
    char des_cmd[2] = {cmd, 0x00};
    struct dsi_cmd_desc tcon_cmd = {
        {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(des_cmd)}, des_cmd};
    char *rbuffer;
    int i = 0;

    if(PANEL_CMD)
        pr_info("[Display] cmd (0x%02x)\n", des_cmd[0]);

    ctrl_pdata = container_of(g_mdss_pdata, struct mdss_dsi_ctrl_pdata,
                panel_data);

    mutex_lock(&cmd_mutex);

    if (g_mdss_pdata->panel_info.panel_power_state == MDSS_PANEL_POWER_ON) {
        memset(&cmdreq, 0, sizeof(cmdreq));
        rbuffer = kmalloc(sizeof(ctrl_pdata->rx_buf.len), GFP_KERNEL);

        cmdreq.cmds = &tcon_cmd;
        cmdreq.cmds_cnt = 1;
        cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
        cmdreq.rlen = rlen;   //read back rlen byte
        cmdreq.rbuf = rbuffer;
        cmdreq.cb = dump_register_cb;  //fxn; /* call back */
        mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);
        for(i=0; i<rlen; i++)
            pr_info("[Display] read parameter: 0x%02x = 0x%02x\n", des_cmd[0], *(cmdreq.rbuf+i));
    } else {
        pr_err("[Display] read parameter failed\n");
    }

    mutex_unlock(&cmd_mutex);
}


#ifndef CONFIG_ASUS_ZC550KL_PROJECT
//////////////////  dump panel reg  //////////////////
static char dsi_num_err_r[2] = {0x05, 0x00};
static char power_mode_r[2] = {0x0A, 0x00};
static char addr_mode_r[2] = {0x0B, 0x00};
static char pixel_mode_r[2] = {0x0C, 0x00};
static char display_mode_r[2] = {0x0D, 0x00};
static char signal_mode_r[2] = {0x0E, 0x00};
static char self_diagnostic_r[2] = {0x0F, 0x00};
static char memory_start_r[2] = {0x2E, 0x00};
static char bl_value_r[2] = {0x52, 0x00}; 
static char init_cabc1_r[2] = {0x54, 0x00};
static char IE_CABC_r[2] = {0x56, 0x00}; 
static char CABC_min_bl_r[2] = {0x5F, 0x0};
static char id1[2] = {0xDA, 0x0};
static char id2[2] = {0xDB, 0x0};
static char id3[2] = {0xDC, 0x0};

static struct dsi_cmd_desc dump_panel_reg_cmd[] = {
    { {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(dsi_num_err_r)}, dsi_num_err_r },
    { {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(power_mode_r)}, power_mode_r },
    { {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(addr_mode_r)}, addr_mode_r },
    { {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(pixel_mode_r)}, pixel_mode_r },
    { {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(display_mode_r)}, display_mode_r },
    { {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(signal_mode_r)}, signal_mode_r },
    { {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(self_diagnostic_r)}, self_diagnostic_r },
    { {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(memory_start_r)}, memory_start_r },
    { {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(bl_value_r)}, bl_value_r},
    { {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(init_cabc1_r)}, init_cabc1_r },
    { {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(IE_CABC_r)}, IE_CABC_r},
    { {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(CABC_min_bl_r)}, CABC_min_bl_r },
    { {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(id1)}, id1},
    { {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(id2)}, id2},
    { {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(id3)}, id3},
};

static bool write_lcd_reg_val(char *dump_result)
{
    struct file *fp = NULL; 
    loff_t pos_lsts = 0;
    mm_segment_t old_fs;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    fp = filp_open(PANEL_REG_FILE, O_RDWR|O_CREAT|O_APPEND, 0644);
    if(IS_ERR_OR_NULL(fp)) {
        printk("[Display] write lcd register open (%s) fail\n", PANEL_REG_FILE);
        return false;
    }
    fp->f_op->write(fp, dump_result, strlen(dump_result), &pos_lsts);

    pos_lsts = 0;
    set_fs(old_fs);
    filp_close(fp, NULL);

    return true;
}

static ssize_t dump_asus_panel_register(char *dump_result)
{
    struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
    struct dcs_cmd_req cmdreq;
    int i;
    char addr;
    char *rbuffer, buf[512];
	int len = 0;

    ctrl_pdata = container_of(g_mdss_pdata, struct mdss_dsi_ctrl_pdata,
                panel_data);

    mutex_lock(&cmd_mutex);

	if (g_mdss_pdata->panel_info.panel_power_state == MDSS_PANEL_POWER_ON) {
		for (i=0; i < ARRAY_SIZE(dump_panel_reg_cmd); i++) {
			memset(&cmdreq, 0, sizeof(cmdreq));
			rbuffer = kmalloc(sizeof(ctrl_pdata->rx_buf.len), GFP_KERNEL);
	
			cmdreq.cmds = &dump_panel_reg_cmd[i];
			cmdreq.cmds_cnt = 1;
			cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
			cmdreq.rlen = 3;
			cmdreq.rbuf = rbuffer;
			cmdreq.cb = dump_register_cb;
			mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);
	
			addr = (char) (dump_panel_reg_cmd[i].payload)[0];
			len += sprintf(buf, "addr: 0x%x = (0x%x)\n", addr, *(cmdreq.rbuf));
			strcat(dump_result, buf);

			kfree(rbuffer);
		}
	} else {
		printk("[Display] dump fail due to panel_power_state=%d\n",
 														g_mdss_pdata->panel_info.panel_power_state);
	}
	//printk("%s\n", dump_result);

    mutex_unlock(&cmd_mutex);

	return len;
}
//////////////////  dump panel reg  //////////////////

static ssize_t lcd_reg_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
    char messages[256];
	char reg_buf[512] = {0};

    memset(messages, 0, sizeof(messages));

    if (len > 256)
        len = 256;
    if (copy_from_user(messages, buff, len))
        return -EFAULT;

    initKernelEnv();

    if (strncmp(messages, "on", 2) == 0) {
        dump_asus_panel_register(reg_buf);
		write_lcd_reg_val(reg_buf);
    }

    deinitKernelEnv(); 
    return len;
}

static ssize_t lcd_reg_read(struct file *file, char __user *buf,
                    size_t count, loff_t *ppos)
{
	ssize_t ret = 0, len = 0;
	char buff[512] = {0};

	len = dump_asus_panel_register(buff);
	ret = simple_read_from_buffer(buf, count, ppos, &buff, len);

	return ret;
}

static struct file_operations lcd_reg_proc_ops = {
	.write = lcd_reg_write,
	.read = lcd_reg_read,
};
//Louis, Panel debug control interface ---
#endif

#define MIN_LEN 2
#define MAX_LEN 4

static ssize_t lcd_reg_rw(struct file *filp, const char *buff, size_t len, loff_t *off)
{
    char *messages, *tmp, *cur;
    char *token, *token_par;
    char *put_cmd;
    bool flag = 0;
    int *store;
    int i = 0, cnt = 0, cmd_cnt = 0;
    int ret = 0;
    uint8_t str_len = 0;

    messages = (char*) kmalloc(len*sizeof(char), GFP_KERNEL);
    if(!messages)
        return -EFAULT;

    tmp = (char*) kmalloc(len*sizeof(char), GFP_KERNEL);
    memset(tmp, 0, len*sizeof(char));
    store =  (int*) kmalloc((len/MIN_LEN)*sizeof(int), GFP_KERNEL);
    put_cmd = (char*) kmalloc((len/MIN_LEN)*sizeof(char), GFP_KERNEL);

    if (copy_from_user(messages, buff, len)) {
        ret = -1;
        goto error;
    }
    cur = messages;
    *(cur+len-1) = '\0';

    pr_info("[Display] %s +++\n", __func__);

    if (strncmp(cur, "w", 1) == 0) //write
        flag = true;
    else if(strncmp(cur, "r", 1) == 0) //read
        flag = false;
    else {
        ret = -1;
        goto error;
    }

    while ((token = strsep(&cur, "wr")) != NULL) {
        str_len = strlen(token);

        if(str_len > 0) { /*filter zero length*/
            if(!(strncmp(token, ",", 1) == 0) || (str_len < MAX_LEN)) {
                ret = -1;
                goto error;
            }

            memset(store, 0, (len/MIN_LEN)*sizeof(int));
            memset(put_cmd, 0, (len/MIN_LEN)*sizeof(char));
            cmd_cnt++;

            while ((token_par = strsep(&token, ",")) != NULL) {
                if(strlen(token_par) > MIN_LEN) {
                    ret = -1;
                    goto error;
                }
                if(strlen(token_par)) {
                    sscanf(token_par, "%x", &(store[cnt]));
                    cnt++;
                }
            }

            for(i=0; i<cnt; i++)
                put_cmd[i] = store[i]&0xff;

            if(flag) {
                pr_info("[Display] write panel command\n");
                set_tcon_cmd(put_cmd, cnt);
            }
            else {
                pr_info("[Display] read panel command\n");
                get_tcon_cmd(put_cmd[0], store[1]);
            }

            if(cur != NULL) {
                if (*(tmp+str_len) == 'w') 
                    flag = true;
                else if (*(tmp+str_len) == 'r')
                    flag = false;
            }
            cnt = 0;
        }

        memset(tmp, 0, len*sizeof(char));

        if(cur != NULL)
            strcpy(tmp, cur);
    }

    if(cmd_cnt == 0) {
        ret = -1;
        goto error;
    }

    ret = len;

error:
    pr_err("[Display] %s(%d)  --\n", __func__, ret);
    kfree(messages);
    kfree(tmp);
    kfree(store);
    kfree(put_cmd);
    return ret;

}

static struct file_operations lcd_reg_rw_ops = {
    .write = lcd_reg_rw,
};
//Louis, Panel debug control interface ---

static void mdss_dsi_parse_lane_swap(struct device_node *np, char *dlane_swap)
{
	const char *data;

	*dlane_swap = DSI_LANE_MAP_0123;
	data = of_get_property(np, "qcom,mdss-dsi-lane-map", NULL);
	if (data) {
		if (!strcmp(data, "lane_map_3012"))
			*dlane_swap = DSI_LANE_MAP_3012;
		else if (!strcmp(data, "lane_map_2301"))
			*dlane_swap = DSI_LANE_MAP_2301;
		else if (!strcmp(data, "lane_map_1230"))
			*dlane_swap = DSI_LANE_MAP_1230;
		else if (!strcmp(data, "lane_map_0321"))
			*dlane_swap = DSI_LANE_MAP_0321;
		else if (!strcmp(data, "lane_map_1032"))
			*dlane_swap = DSI_LANE_MAP_1032;
		else if (!strcmp(data, "lane_map_2103"))
			*dlane_swap = DSI_LANE_MAP_2103;
		else if (!strcmp(data, "lane_map_3210"))
			*dlane_swap = DSI_LANE_MAP_3210;
	}
}

static void mdss_dsi_parse_trigger(struct device_node *np, char *trigger,
		char *trigger_key)
{
	const char *data;

	*trigger = DSI_CMD_TRIGGER_SW;
	data = of_get_property(np, trigger_key, NULL);
	if (data) {
		if (!strcmp(data, "none"))
			*trigger = DSI_CMD_TRIGGER_NONE;
		else if (!strcmp(data, "trigger_te"))
			*trigger = DSI_CMD_TRIGGER_TE;
		else if (!strcmp(data, "trigger_sw_seof"))
			*trigger = DSI_CMD_TRIGGER_SW_SEOF;
		else if (!strcmp(data, "trigger_sw_te"))
			*trigger = DSI_CMD_TRIGGER_SW_TE;
	}
}


int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len;
	char *buf, *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;

	data = of_get_property(np, cmd_key, &blen);
	if (!data) {
		pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	buf = kzalloc(sizeof(char) * blen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, data, blen);

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len >= sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			goto exit_free;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		goto exit_free;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds)
		goto exit_free;

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	/*Set default link state to LP Mode*/
	pcmds->link_state = DSI_LP_MODE;

	if (link_key) {
		data = of_get_property(np, link_key, NULL);
		if (data && !strcmp(data, "dsi_hs_mode"))
			pcmds->link_state = DSI_HS_MODE;
		else
			pcmds->link_state = DSI_LP_MODE;
	}

	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;

exit_free:
	kfree(buf);
	return -ENOMEM;
}


int mdss_panel_get_dst_fmt(u32 bpp, char mipi_mode, u32 pixel_packing,
				char *dst_format)
{
	int rc = 0;
	switch (bpp) {
	case 3:
		*dst_format = DSI_CMD_DST_FORMAT_RGB111;
		break;
	case 8:
		*dst_format = DSI_CMD_DST_FORMAT_RGB332;
		break;
	case 12:
		*dst_format = DSI_CMD_DST_FORMAT_RGB444;
		break;
	case 16:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB565;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		}
		break;
	case 18:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB666;
			break;
		default:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		}
		break;
	case 24:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB888;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}


static int mdss_dsi_parse_fbc_params(struct device_node *np,
				struct mdss_panel_info *panel_info)
{
	int rc, fbc_enabled = 0;
	u32 tmp;

	fbc_enabled = of_property_read_bool(np,	"qcom,mdss-dsi-fbc-enable");
	if (fbc_enabled) {
		pr_debug("%s:%d FBC panel enabled.\n", __func__, __LINE__);
		panel_info->fbc.enabled = 1;
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bpp", &tmp);
		panel_info->fbc.target_bpp =	(!rc ? tmp : panel_info->bpp);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-packing",
				&tmp);
		panel_info->fbc.comp_mode = (!rc ? tmp : 0);
		panel_info->fbc.qerr_enable = of_property_read_bool(np,
			"qcom,mdss-dsi-fbc-quant-error");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bias", &tmp);
		panel_info->fbc.cd_bias = (!rc ? tmp : 0);
		panel_info->fbc.pat_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-pat-mode");
		panel_info->fbc.vlc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-vlc-mode");
		panel_info->fbc.bflc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-bflc-mode");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-h-line-budget",
				&tmp);
		panel_info->fbc.line_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-budget-ctrl",
				&tmp);
		panel_info->fbc.block_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-block-budget",
				&tmp);
		panel_info->fbc.block_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossless-threshold", &tmp);
		panel_info->fbc.lossless_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-threshold", &tmp);
		panel_info->fbc.lossy_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-rgb-threshold",
				&tmp);
		panel_info->fbc.lossy_rgb_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-mode-idx", &tmp);
		panel_info->fbc.lossy_mode_idx = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-slice-height", &tmp);
		panel_info->fbc.slice_height = (!rc ? tmp : 0);
		panel_info->fbc.pred_mode = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-2d-pred-mode");
		panel_info->fbc.enc_mode = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-ver2-mode");
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-max-pred-err", &tmp);
		panel_info->fbc.max_pred_err = (!rc ? tmp : 0);
	} else {
		pr_debug("%s:%d Panel does not support FBC.\n",
				__func__, __LINE__);
		panel_info->fbc.enabled = 0;
		panel_info->fbc.target_bpp =
			panel_info->bpp;
	}
	return 0;
}

static void mdss_panel_parse_te_params(struct device_node *np,
				       struct mdss_panel_info *panel_info)
{

	u32 tmp;
	int rc = 0;
	/*
	 * TE default: dsi byte clock calculated base on 70 fps;
	 * around 14 ms to complete a kickoff cycle if te disabled;
	 * vclk_line base on 60 fps; write is faster than read;
	 * init == start == rdptr;
	 */
	panel_info->te.tear_check_en =
		!of_property_read_bool(np, "qcom,mdss-tear-check-disable");
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-cfg-height", &tmp);
	panel_info->te.sync_cfg_height = (!rc ? tmp : 0xfff0);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-init-val", &tmp);
	panel_info->te.vsync_init_val = (!rc ? tmp : panel_info->yres);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-threshold-start", &tmp);
	panel_info->te.sync_threshold_start = (!rc ? tmp : 4);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-threshold-continue", &tmp);
	panel_info->te.sync_threshold_continue = (!rc ? tmp : 4);
	rc = of_property_read_u32(np, "qcom,mdss-tear-check-start-pos", &tmp);
	panel_info->te.start_pos = (!rc ? tmp : panel_info->yres);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-rd-ptr-trigger-intr", &tmp);
	panel_info->te.rd_ptr_irq = (!rc ? tmp : panel_info->yres + 1);
	rc = of_property_read_u32(np, "qcom,mdss-tear-check-frame-rate", &tmp);
	panel_info->te.refx100 = (!rc ? tmp : 6000);
}


static int mdss_dsi_parse_reset_seq(struct device_node *np,
		u32 rst_seq[MDSS_DSI_RST_SEQ_LEN], u32 *rst_len,
		const char *name)
{
	int num = 0, i;
	int rc;
	struct property *data;
	u32 tmp[MDSS_DSI_RST_SEQ_LEN];
	*rst_len = 0;
	data = of_find_property(np, name, &num);
	num /= sizeof(u32);
	if (!data || !num || num > MDSS_DSI_RST_SEQ_LEN || num % 2) {
		pr_debug("%s:%d, error reading %s, length found = %d\n",
			__func__, __LINE__, name, num);
	} else {
		rc = of_property_read_u32_array(np, name, tmp, num);
		if (rc)
			pr_debug("%s:%d, error reading %s, rc = %d\n",
				__func__, __LINE__, name, rc);
		else {
			for (i = 0; i < num; ++i)
				rst_seq[i] = tmp[i];
			*rst_len = num;
		}
	}
	return 0;
}

static int mdss_dsi_gen_read_status(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	if (ctrl_pdata->status_buf.data[0] !=
					ctrl_pdata->status_value) {
		pr_err("%s: Read back value from panel is incorrect\n",
							__func__);
		return -EINVAL;
	} else {
		return 1;
	}
}

static int mdss_dsi_nt35596_read_status(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	if (ctrl_pdata->status_buf.data[0] !=
					ctrl_pdata->status_value) {
		ctrl_pdata->status_error_count = 0;
		pr_err("%s: Read back value from panel is incorrect\n",
							__func__);
		return -EINVAL;
	} else {
		if (ctrl_pdata->status_buf.data[3] != NT35596_BUF_3_STATUS) {
			ctrl_pdata->status_error_count = 0;
		} else {
			if ((ctrl_pdata->status_buf.data[4] ==
				NT35596_BUF_4_STATUS) ||
				(ctrl_pdata->status_buf.data[5] ==
				NT35596_BUF_5_STATUS))
				ctrl_pdata->status_error_count = 0;
			else
				ctrl_pdata->status_error_count++;
			if (ctrl_pdata->status_error_count >=
					NT35596_MAX_ERR_CNT) {
				ctrl_pdata->status_error_count = 0;
				pr_err("%s: Read value bad. Error_cnt = %i\n",
					 __func__,
					ctrl_pdata->status_error_count);
				return -EINVAL;
			}
		}
		return 1;
	}
}

static void mdss_dsi_parse_roi_alignment(struct device_node *np,
		struct mdss_panel_info *pinfo)
{
	int len = 0;
	u32 value[6];
	struct property *data;
	data = of_find_property(np, "qcom,panel-roi-alignment", &len);
	len /= sizeof(u32);
	if (!data || (len != 6)) {
		pr_debug("%s: Panel roi alignment not found", __func__);
	} else {
		int rc = of_property_read_u32_array(np,
				"qcom,panel-roi-alignment", value, len);
		if (rc)
			pr_debug("%s: Error reading panel roi alignment values",
					__func__);
		else {
			pinfo->xstart_pix_align = value[0];
			pinfo->width_pix_align = value[1];
			pinfo->ystart_pix_align = value[2];
			pinfo->height_pix_align = value[3];
			pinfo->min_width = value[4];
			pinfo->min_height = value[5];
		}

		pr_debug("%s: ROI alignment: [%d, %d, %d, %d, %d, %d]",
				__func__, pinfo->xstart_pix_align,
				pinfo->width_pix_align, pinfo->ystart_pix_align,
				pinfo->height_pix_align, pinfo->min_width,
				pinfo->min_height);
	}
}

static int mdss_dsi_parse_panel_features(struct device_node *np,
	struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct mdss_panel_info *pinfo;

	if (!np || !ctrl) {
		pr_err("%s: Invalid arguments\n", __func__);
		return -ENODEV;
	}

	pinfo = &ctrl->panel_data.panel_info;

	pinfo->cont_splash_enabled = of_property_read_bool(np,
		"qcom,cont-splash-enabled");

	if (pinfo->mipi.mode == DSI_CMD_MODE) {
		pinfo->partial_update_enabled = of_property_read_bool(np,
				"qcom,partial-update-enabled");
		pr_info("%s: partial_update_enabled=%d\n", __func__,
					pinfo->partial_update_enabled);
		if (pinfo->partial_update_enabled) {
			ctrl->set_col_page_addr = mdss_dsi_set_col_page_addr;
			pinfo->partial_update_roi_merge =
					of_property_read_bool(np,
					"qcom,partial-update-roi-merge");
		}

		pinfo->dcs_cmd_by_left = of_property_read_bool(np,
						"qcom,dcs-cmd-by-left");
	}

	pinfo->ulps_feature_enabled = of_property_read_bool(np,
		"qcom,ulps-enabled");
	pr_info("%s: ulps feature %s\n", __func__,
		(pinfo->ulps_feature_enabled ? "enabled" : "disabled"));
	pinfo->esd_check_enabled = of_property_read_bool(np,
		"qcom,esd-check-enabled");

	pinfo->ulps_suspend_enabled = of_property_read_bool(np,
		"qcom,suspend-ulps-enabled");
	pr_info("%s: ulps during suspend feature %s", __func__,
		(pinfo->ulps_suspend_enabled ? "enabled" : "disabled"));

	pinfo->mipi.dynamic_switch_enabled = of_property_read_bool(np,
		"qcom,dynamic-mode-switch-enabled");

	if (pinfo->mipi.dynamic_switch_enabled) {
		mdss_dsi_parse_dcs_cmds(np, &ctrl->video2cmd,
			"qcom,video-to-cmd-mode-switch-commands", NULL);

		mdss_dsi_parse_dcs_cmds(np, &ctrl->cmd2video,
			"qcom,cmd-to-video-mode-switch-commands", NULL);

		if (!ctrl->video2cmd.cmd_cnt || !ctrl->cmd2video.cmd_cnt) {
			pr_warn("No commands specified for dynamic switch\n");
			pinfo->mipi.dynamic_switch_enabled = 0;
		}
	}

	pr_info("%s: dynamic switch feature enabled: %d\n", __func__,
		pinfo->mipi.dynamic_switch_enabled);
	pinfo->panel_ack_disabled = of_property_read_bool(np,
				"qcom,panel-ack-disabled");

	if (pinfo->panel_ack_disabled && pinfo->esd_check_enabled) {
		pr_warn("ESD should not be enabled if panel ACK is disabled\n");
		pinfo->esd_check_enabled = false;
	}

	if (ctrl->disp_en_gpio <= 0) {
		ctrl->disp_en_gpio = of_get_named_gpio(
			np,
			"qcom,5v-boost-gpio", 0);

		if (!gpio_is_valid(ctrl->disp_en_gpio))
			pr_err("%s:%d, Disp_en gpio not specified\n",
					__func__, __LINE__);
	}

	return 0;
}

static void mdss_dsi_parse_panel_horizintal_line_idle(struct device_node *np,
	struct mdss_dsi_ctrl_pdata *ctrl)
{
	const u32 *src;
	int i, len, cnt;
	struct panel_horizontal_idle *kp;

	if (!np || !ctrl) {
		pr_err("%s: Invalid arguments\n", __func__);
		return;
	}

	src = of_get_property(np, "qcom,mdss-dsi-hor-line-idle", &len);
	if (!src || len == 0)
		return;

	cnt = len % 3; /* 3 fields per entry */
	if (cnt) {
		pr_err("%s: invalid horizontal idle len=%d\n", __func__, len);
		return;
	}

	cnt = len / sizeof(u32);

	kp = kzalloc(sizeof(*kp) * (cnt / 3), GFP_KERNEL);
	if (kp == NULL) {
		pr_err("%s: No memory\n", __func__);
		return;
	}

	ctrl->line_idle = kp;
	for (i = 0; i < cnt; i += 3) {
		kp->min = be32_to_cpu(src[i]);
		kp->max = be32_to_cpu(src[i+1]);
		kp->idle = be32_to_cpu(src[i+2]);
		kp++;
		ctrl->horizontal_idle_cnt++;
	}

	pr_debug("%s: horizontal_idle_cnt=%d\n", __func__,
				ctrl->horizontal_idle_cnt);
}

static int mdss_dsi_set_refresh_rate_range(struct device_node *pan_node,
		struct mdss_panel_info *pinfo)
{
	int rc = 0;
	rc = of_property_read_u32(pan_node,
			"qcom,mdss-dsi-min-refresh-rate",
			&pinfo->min_fps);
	if (rc) {
		pr_warn("%s:%d, Unable to read min refresh rate\n",
				__func__, __LINE__);

		/*
		 * Since min refresh rate is not specified when dynamic
		 * fps is enabled, using minimum as 30
		 */
		pinfo->min_fps = MIN_REFRESH_RATE;
		rc = 0;
	}

	rc = of_property_read_u32(pan_node,
			"qcom,mdss-dsi-max-refresh-rate",
			&pinfo->max_fps);
	if (rc) {
		pr_warn("%s:%d, Unable to read max refresh rate\n",
				__func__, __LINE__);

		/*
		 * Since max refresh rate was not specified when dynamic
		 * fps is enabled, using the default panel refresh rate
		 * as max refresh rate supported.
		 */
		pinfo->max_fps = pinfo->mipi.frame_rate;
		rc = 0;
	}

	pr_info("dyn_fps: min = %d, max = %d\n",
			pinfo->min_fps, pinfo->max_fps);
	return rc;
}

static void mdss_dsi_parse_dfps_config(struct device_node *pan_node,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	const char *data;
	bool dynamic_fps;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	dynamic_fps = of_property_read_bool(pan_node,
			"qcom,mdss-dsi-pan-enable-dynamic-fps");

	if (!dynamic_fps)
		return;

	pinfo->dynamic_fps = true;
	data = of_get_property(pan_node, "qcom,mdss-dsi-pan-fps-update", NULL);
	if (data) {
		if (!strcmp(data, "dfps_suspend_resume_mode")) {
			pinfo->dfps_update = DFPS_SUSPEND_RESUME_MODE;
			pr_debug("dfps mode: suspend/resume\n");
		} else if (!strcmp(data, "dfps_immediate_clk_mode")) {
			pinfo->dfps_update = DFPS_IMMEDIATE_CLK_UPDATE_MODE;
			pr_debug("dfps mode: Immediate clk\n");
		} else if (!strcmp(data, "dfps_immediate_porch_mode")) {
			pinfo->dfps_update = DFPS_IMMEDIATE_PORCH_UPDATE_MODE;
			pr_debug("dfps mode: Immediate porch\n");
		} else {
			pinfo->dfps_update = DFPS_SUSPEND_RESUME_MODE;
			pr_debug("default dfps mode: suspend/resume\n");
		}
		mdss_dsi_set_refresh_rate_range(pan_node, pinfo);
	} else {
		pinfo->dynamic_fps = false;
		pr_debug("dfps update mode not configured: disable\n");
	}
	pinfo->new_fps = pinfo->mipi.frame_rate;

	return;
}

void mdss_dsi_unregister_bl_settings(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	if (ctrl_pdata->bklt_ctrl == BL_WLED)
		led_trigger_unregister_simple(bl_led_trigger);
}


/**
 * get_mdss_dsi_even_lane_clam_mask() - Computest DSI lane 0 & 2 clamps mask
 * @dlane_swap: dsi_lane_map_type
 * @lane_id: DSI lane (DSI_LANE_0)/(DSI_LANE_2)
 *
 * Return DSI lane 0 & 2 clamp mask based on lane swap configuration.
 * Clamp Bit for Physical lanes
 *      Lane Num        Bit     Mask
 *      Lane0           Bit 7   0x80
 *      Lane1           Bit 5   0x20
 *      Lane2           Bit 3   0x08
 *      Lane3           Bit 2   0x02
 */
u32 get_mdss_dsi_even_lane_clam_mask(char dlane_swap,
				     enum dsi_lane_ids lane_id)
{
	u32 lane0_mask = 0;
	u32 lane2_mask = 0;

	switch (dlane_swap) {
	case DSI_LANE_MAP_0123:
	case DSI_LANE_MAP_0321:
		lane0_mask = 0x80;
		lane2_mask = 0x08;
		break;
	case DSI_LANE_MAP_3012:
	case DSI_LANE_MAP_1032:
		lane0_mask = 0x20;
		lane2_mask = 0x02;
		break;
	case DSI_LANE_MAP_2301:
	case DSI_LANE_MAP_2103:
		lane0_mask = 0x08;
		lane2_mask = 0x80;
		break;
	case DSI_LANE_MAP_1230:
	case DSI_LANE_MAP_3210:
		lane0_mask = 0x02;
		lane2_mask = 0x20;
		break;
	default:
		lane0_mask = 0x00;
		lane2_mask = 0x00;
		break;
	}
	if (lane_id == DSI_LANE_0)
		return lane0_mask;
	else if (lane_id == DSI_LANE_2)
		return lane2_mask;
	else
		return 0;
}

/**
 * get_mdss_dsi_odd_lane_clam_mask() - Computest DSI lane 1 & 3 clamps mask
 * @dlane_swap: dsi_lane_map_type
 * @lane_id: DSI lane (DSI_LANE_1)/(DSI_LANE_3)
 *
 * Return DSI lane 1 & 3 clamp mask based on lane swap configuration.
 */
u32 get_mdss_dsi_odd_lane_clam_mask(char dlane_swap,
					enum dsi_lane_ids lane_id)
{
	u32 lane1_mask = 0;
	u32 lane3_mask = 0;

	switch (dlane_swap) {
	case DSI_LANE_MAP_0123:
	case DSI_LANE_MAP_2103:
		lane1_mask = 0x20;
		lane3_mask = 0x02;
		break;
	case DSI_LANE_MAP_3012:
	case DSI_LANE_MAP_3210:
		lane1_mask = 0x08;
		lane3_mask = 0x80;
		break;
	case DSI_LANE_MAP_2301:
	case DSI_LANE_MAP_0321:
		lane1_mask = 0x02;
		lane3_mask = 0x20;
		break;
	case DSI_LANE_MAP_1230:
	case DSI_LANE_MAP_1032:
		lane1_mask = 0x80;
		lane3_mask = 0x08;
		break;
	default:
		lane1_mask = 0x00;
		lane3_mask = 0x00;
		break;
	}
	if (lane_id == DSI_LANE_1)
		return lane1_mask;
	else if (lane_id == DSI_LANE_3)
		return lane3_mask;
	else
		return 0;
}
static void mdss_dsi_set_lane_clamp_mask(struct mipi_panel_info *mipi)
{
	u32 mask = 0;

	if (mipi->data_lane0)
		mask = get_mdss_dsi_even_lane_clam_mask(mipi->dlane_swap,
					DSI_LANE_0);
	if (mipi->data_lane1)
		mask |= get_mdss_dsi_odd_lane_clam_mask(mipi->dlane_swap,
					DSI_LANE_1);
	if (mipi->data_lane2)
		mask |= get_mdss_dsi_even_lane_clam_mask(mipi->dlane_swap,
					DSI_LANE_2);
	if (mipi->data_lane3)
		mask |= get_mdss_dsi_odd_lane_clam_mask(mipi->dlane_swap,
					DSI_LANE_3);

	mipi->phy_lane_clamp_mask = mask;
}


static int mdss_panel_parse_dt(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	u32 tmp;
	int rc, i, len;
	const char *data;
	static const char *pdest;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-width", &tmp);
	if (rc) {
		pr_err("%s:%d, panel width not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pinfo->xres = (!rc ? tmp : 640);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-height", &tmp);
	if (rc) {
		pr_err("%s:%d, panel height not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pinfo->yres = (!rc ? tmp : 480);

	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-width-dimension", &tmp);
	pinfo->physical_width = (!rc ? tmp : 0);
	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-height-dimension", &tmp);
	pinfo->physical_height = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-left-border", &tmp);
	pinfo->lcdc.border_left = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-right-border", &tmp);
	if (!rc)
		pinfo->lcdc.border_right = tmp;

	pinfo->lcdc.xres_pad = (pinfo->lcdc.border_left +
				pinfo->lcdc.border_right);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-top-border", &tmp);
	pinfo->lcdc.border_top = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-bottom-border", &tmp);
	if (!rc)
		pinfo->lcdc.border_bottom = tmp;

	pinfo->lcdc.yres_pad = (pinfo->lcdc.border_top +
				pinfo->lcdc.border_bottom);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-bpp", &tmp);
	if (rc) {
		pr_err("%s:%d, bpp not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	pinfo->bpp = (!rc ? tmp : 24);
	pinfo->mipi.mode = DSI_VIDEO_MODE;
	data = of_get_property(np, "qcom,mdss-dsi-panel-type", NULL);
	if (data && !strncmp(data, "dsi_cmd_mode", 12))
		pinfo->mipi.mode = DSI_CMD_MODE;
	tmp = 0;
	data = of_get_property(np, "qcom,mdss-dsi-pixel-packing", NULL);
	if (data && !strcmp(data, "loose"))
		pinfo->mipi.pixel_packing = 1;
	else
		pinfo->mipi.pixel_packing = 0;
	rc = mdss_panel_get_dst_fmt(pinfo->bpp,
		pinfo->mipi.mode, pinfo->mipi.pixel_packing,
		&(pinfo->mipi.dst_format));
	if (rc) {
		pr_debug("%s: problem determining dst format. Set Default\n",
			__func__);
		pinfo->mipi.dst_format =
			DSI_VIDEO_DST_FORMAT_RGB888;
	}
	pdest = of_get_property(np,
		"qcom,mdss-dsi-panel-destination", NULL);

	if (pdest) {
		if (strlen(pdest) != 9) {
			pr_err("%s: Unknown pdest specified\n", __func__);
			return -EINVAL;
		}
		if (!strcmp(pdest, "display_1"))
			pinfo->pdest = DISPLAY_1;
		else if (!strcmp(pdest, "display_2"))
			pinfo->pdest = DISPLAY_2;
		else {
			pr_debug("%s: incorrect pdest. Set Default\n",
				__func__);
			pinfo->pdest = DISPLAY_1;
		}
	} else {
		pr_debug("%s: pdest not specified. Set Default\n",
				__func__);
		pinfo->pdest = DISPLAY_1;
	}
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-front-porch", &tmp);
	pinfo->lcdc.h_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-back-porch", &tmp);
	pinfo->lcdc.h_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-pulse-width", &tmp);
	pinfo->lcdc.h_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-sync-skew", &tmp);
	pinfo->lcdc.hsync_skew = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-back-porch", &tmp);
	pinfo->lcdc.v_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-front-porch", &tmp);
	pinfo->lcdc.v_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-pulse-width", &tmp);
	pinfo->lcdc.v_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-underflow-color", &tmp);
	pinfo->lcdc.underflow_clr = (!rc ? tmp : 0xff);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-border-color", &tmp);
	pinfo->lcdc.border_clr = (!rc ? tmp : 0);
	data = of_get_property(np, "qcom,mdss-dsi-panel-orientation", NULL);
	if (data) {
		pr_debug("panel orientation is %s\n", data);
		if (!strcmp(data, "180"))
			pinfo->panel_orientation = MDP_ROT_180;
		else if (!strcmp(data, "hflip"))
			pinfo->panel_orientation = MDP_FLIP_LR;
		else if (!strcmp(data, "vflip"))
			pinfo->panel_orientation = MDP_FLIP_UD;
	}

	ctrl_pdata->bklt_ctrl = UNKNOWN_CTRL;
	data = of_get_property(np, "qcom,mdss-dsi-bl-pmic-control-type", NULL);
	if (data) {
		if (!strncmp(data, "bl_ctrl_wled", 12)) {
			led_trigger_register_simple("bkl-trigger",
				&bl_led_trigger);
			pr_debug("%s: SUCCESS-> WLED TRIGGER register\n",
				__func__);
			ctrl_pdata->bklt_ctrl = BL_WLED;
		} else if (!strncmp(data, "bl_ctrl_pwm", 11)) {
			ctrl_pdata->bklt_ctrl = BL_PWM;
			ctrl_pdata->pwm_pmi = of_property_read_bool(np,
					"qcom,mdss-dsi-bl-pwm-pmi");
			rc = of_property_read_u32(np,
				"qcom,mdss-dsi-bl-pmic-pwm-frequency", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, panel pwm_period\n",
						__func__, __LINE__);
				return -EINVAL;
			}
			ctrl_pdata->pwm_period = tmp;
			if (ctrl_pdata->pwm_pmi) {
				ctrl_pdata->pwm_bl = of_pwm_get(np, NULL);
				if (IS_ERR(ctrl_pdata->pwm_bl)) {
					pr_err("%s: Error, pwm device\n",
								__func__);
					ctrl_pdata->pwm_bl = NULL;
					return -EINVAL;
				}
			} else {
				rc = of_property_read_u32(np,
					"qcom,mdss-dsi-bl-pmic-bank-select",
								 &tmp);
				if (rc) {
					pr_err("%s:%d, Error, lpg channel\n",
							__func__, __LINE__);
					return -EINVAL;
				}
				ctrl_pdata->pwm_lpg_chan = tmp;
				tmp = of_get_named_gpio(np,
					"qcom,mdss-dsi-pwm-gpio", 0);
				ctrl_pdata->pwm_pmic_gpio = tmp;
				pr_debug("%s: Configured PWM bklt ctrl\n",
								 __func__);
			}
		} else if (!strncmp(data, "bl_ctrl_dcs", 11)) {
			ctrl_pdata->bklt_ctrl = BL_DCS_CMD;
			pr_debug("%s: Configured DCS_CMD bklt ctrl\n",
								__func__);
		}
	}
	rc = of_property_read_u32(np, "qcom,mdss-brightness-max-level", &tmp);
	pinfo->brightness_max = (!rc ? tmp : MDSS_MAX_BL_BRIGHTNESS);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-min-level", &tmp);
	pinfo->bl_min = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-max-level", &tmp);
	pinfo->bl_max = (!rc ? tmp : 255);
	ctrl_pdata->bklt_max = pinfo->bl_max;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-interleave-mode", &tmp);
	pinfo->mipi.interleave_mode = (!rc ? tmp : 0);

	pinfo->mipi.vsync_enable = of_property_read_bool(np,
		"qcom,mdss-dsi-te-check-enable");
	pinfo->mipi.hw_vsync_mode = of_property_read_bool(np,
		"qcom,mdss-dsi-te-using-te-pin");

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-h-sync-pulse", &tmp);
	pinfo->mipi.pulse_mode_hsa_he = (!rc ? tmp : false);

	pinfo->mipi.hfp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hfp-power-mode");
	pinfo->mipi.hsa_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hsa-power-mode");
	pinfo->mipi.hbp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hbp-power-mode");
	pinfo->mipi.last_line_interleave_en = of_property_read_bool(np,
		"qcom,mdss-dsi-last-line-interleave");
	pinfo->mipi.bllp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-bllp-power-mode");
	pinfo->mipi.eof_bllp_power_stop = of_property_read_bool(
		np, "qcom,mdss-dsi-bllp-eof-power-mode");
	pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_PULSE;
	data = of_get_property(np, "qcom,mdss-dsi-traffic-mode", NULL);
	if (data) {
		if (!strcmp(data, "non_burst_sync_event"))
			pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
		else if (!strcmp(data, "burst_mode"))
			pinfo->mipi.traffic_mode = DSI_BURST_MODE;
	}
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-dcs-command", &tmp);
	pinfo->mipi.insert_dcs_cmd =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-wr-mem-continue", &tmp);
	pinfo->mipi.wr_mem_continue =
			(!rc ? tmp : 0x3c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-wr-mem-start", &tmp);
	pinfo->mipi.wr_mem_start =
			(!rc ? tmp : 0x2c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-pin-select", &tmp);
	pinfo->mipi.te_sel =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-virtual-channel-id", &tmp);
	pinfo->mipi.vc = (!rc ? tmp : 0);
	pinfo->mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	data = of_get_property(np, "qcom,mdss-dsi-color-order", NULL);
	if (data) {
		if (!strcmp(data, "rgb_swap_rbg"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_RBG;
		else if (!strcmp(data, "rgb_swap_bgr"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BGR;
		else if (!strcmp(data, "rgb_swap_brg"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BRG;
		else if (!strcmp(data, "rgb_swap_grb"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_GRB;
		else if (!strcmp(data, "rgb_swap_gbr"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_GBR;
	}
	pinfo->mipi.data_lane0 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-0-state");
	pinfo->mipi.data_lane1 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-1-state");
	pinfo->mipi.data_lane2 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-2-state");
	pinfo->mipi.data_lane3 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-3-state");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-pre", &tmp);
	pinfo->mipi.t_clk_pre = (!rc ? tmp : 0x24);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-post", &tmp);
	pinfo->mipi.t_clk_post = (!rc ? tmp : 0x03);

	pinfo->mipi.rx_eot_ignore = of_property_read_bool(np,
		"qcom,mdss-dsi-rx-eot-ignore");
	pinfo->mipi.tx_eot_append = of_property_read_bool(np,
		"qcom,mdss-dsi-tx-eot-append");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-stream", &tmp);
	pinfo->mipi.stream = (!rc ? tmp : 0);

	data = of_get_property(np, "qcom,mdss-dsi-panel-mode-gpio-state", NULL);
	if (data) {
		if (!strcmp(data, "high"))
			pinfo->mode_gpio_state = MODE_GPIO_HIGH;
		else if (!strcmp(data, "low"))
			pinfo->mode_gpio_state = MODE_GPIO_LOW;
	} else {
		pinfo->mode_gpio_state = MODE_GPIO_NOT_VALID;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-framerate", &tmp);
	pinfo->mipi.frame_rate = (!rc ? tmp : 60);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-clockrate", &tmp);
	pinfo->clk_rate = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-mdp-transfer-time-us", &tmp);
	pinfo->mdp_transfer_time_us = (!rc ? tmp : DEFAULT_MDP_TRANSFER_TIME);
	data = of_get_property(np, "qcom,mdss-dsi-panel-timings", &len);
	if ((!data) || (len != 12)) {
		pr_err("%s:%d, Unable to read Phy timing settings",
		       __func__, __LINE__);
		goto error;
	}
	for (i = 0; i < len; i++)
		pinfo->mipi.dsi_phy_db.timing[i] = data[i];

	pinfo->mipi.lp11_init = of_property_read_bool(np,
					"qcom,mdss-dsi-lp11-init");
	rc = of_property_read_u32(np, "qcom,mdss-dsi-init-delay-us", &tmp);
	pinfo->mipi.init_delay = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-post-init-delay", &tmp);
	pinfo->mipi.post_init_delay = (!rc ? tmp : 0);

	mdss_dsi_parse_roi_alignment(np, pinfo);

	mdss_dsi_parse_trigger(np, &(pinfo->mipi.mdp_trigger),
		"qcom,mdss-dsi-mdp-trigger");

	mdss_dsi_parse_trigger(np, &(pinfo->mipi.dma_trigger),
		"qcom,mdss-dsi-dma-trigger");

	mdss_dsi_parse_lane_swap(np, &(pinfo->mipi.dlane_swap));

	mdss_dsi_parse_fbc_params(np, pinfo);

	mdss_panel_parse_te_params(np, pinfo);

	mdss_dsi_parse_reset_seq(np, pinfo->rst_seq, &(pinfo->rst_seq_len),
		"qcom,mdss-dsi-reset-sequence");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->on_cmds,
		"qcom,mdss-dsi-on-command", "qcom,mdss-dsi-on-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->post_panel_on_cmds,
		"qcom,mdss-dsi-post-panel-on-command", NULL);

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->off_cmds,
		"qcom,mdss-dsi-off-command", "qcom,mdss-dsi-off-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->status_cmds,
			"qcom,mdss-dsi-panel-status-command",
				"qcom,mdss-dsi-panel-status-command-state");
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-status-value", &tmp);
	ctrl_pdata->status_value = (!rc ? tmp : 0);


	ctrl_pdata->status_mode = ESD_MAX;
	rc = of_property_read_string(np,
				"qcom,mdss-dsi-panel-status-check-mode", &data);
	if (!rc) {
		if (!strcmp(data, "bta_check")) {
			ctrl_pdata->status_mode = ESD_BTA;
		} else if (!strcmp(data, "reg_read")) {
			ctrl_pdata->status_mode = ESD_REG;
			ctrl_pdata->status_cmds_rlen = 1;
			ctrl_pdata->check_read_status =
						mdss_dsi_gen_read_status;
		} else if (!strcmp(data, "reg_read_nt35596")) {
			ctrl_pdata->status_mode = ESD_REG_NT35596;
			ctrl_pdata->status_error_count = 0;
			ctrl_pdata->status_cmds_rlen = 8;
			ctrl_pdata->check_read_status =
						mdss_dsi_nt35596_read_status;
		} else if (!strcmp(data, "te_signal_check")) {
			if (pinfo->mipi.mode == DSI_CMD_MODE)
				ctrl_pdata->status_mode = ESD_TE;
			else
				pr_err("TE-ESD not valid for video mode\n");
		}
	}

	pinfo->mipi.force_clk_lane_hs = of_property_read_bool(np,
		"qcom,mdss-dsi-force-clock-lane-hs");

	pinfo->mipi.always_on = of_property_read_bool(np,
		"qcom,mdss-dsi-always-on");

	rc = mdss_dsi_parse_panel_features(np, ctrl_pdata);
	if (rc) {
		pr_err("%s: failed to parse panel features\n", __func__);
		goto error;
	}

	mdss_dsi_parse_panel_horizintal_line_idle(np, ctrl_pdata);

	mdss_dsi_parse_dfps_config(np, ctrl_pdata);

	mdss_livedisplay_parse_dt(np, pinfo);

	return 0;

error:
	return -EINVAL;
}

int mdss_dsi_panel_init(struct device_node *node,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	bool cmd_cfg_cont_splash)
{
	int rc = 0;
	static const char *panel_name;
	struct mdss_panel_info *pinfo;

	if (!node || !ctrl_pdata) {
		pr_err("%s: Invalid arguments\n", __func__);
		return -ENODEV;
	}

	pinfo = &ctrl_pdata->panel_data.panel_info;

	pr_debug("%s:%d\n", __func__, __LINE__);
	pinfo->panel_name[0] = '\0';
	panel_name = of_get_property(node, "qcom,mdss-dsi-panel-name", NULL);
	if (!panel_name) {
		pr_info("%s:%d, Panel name not specified\n",
						__func__, __LINE__);
	} else {
		pr_info("%s: Panel Name = %s\n", __func__, panel_name);
		strlcpy(&pinfo->panel_name[0], panel_name, MDSS_MAX_PANEL_LEN);
	}
	rc = mdss_panel_parse_dt(node, ctrl_pdata);
	if (rc) {
		pr_err("%s:%d panel dt parse failed\n", __func__, __LINE__);
		return rc;
	}

	mdss_dsi_set_lane_clamp_mask(&pinfo->mipi);
	if (!cmd_cfg_cont_splash)
		pinfo->cont_splash_enabled = false;
	pr_info("%s: Continuous splash %s\n", __func__,
		pinfo->cont_splash_enabled ? "enabled" : "disabled");

	pinfo->dynamic_switch_pending = false;
	pinfo->is_lpm_mode = false;
	pinfo->esd_rdy = false;
	ctrl_pdata->on = mdss_dsi_panel_on;
	ctrl_pdata->post_panel_on = mdss_dsi_post_panel_on;
	ctrl_pdata->off = mdss_dsi_panel_off;
	ctrl_pdata->low_power_config = mdss_dsi_panel_low_power_config;
	ctrl_pdata->panel_data.set_backlight = mdss_dsi_panel_bl_ctrl;
	ctrl_pdata->switch_mode = mdss_dsi_panel_switch_mode;

	//ASUS_BSP: Louis ++
	mutex_init(&cmd_mutex);
#ifndef CONFIG_ASUS_ZC550KL_PROJECT
	proc_create(ZE500KL_LCD_UNIQUE_ID, 0444, NULL, &lcd_uniqueID_proc_ops);
	proc_create(ZE500KL_LCD_ID, 0444, NULL, &lcd_id_proc_ops);
#endif
	proc_create(LCD_REGISTER_RW, 0640, NULL, &lcd_reg_rw_ops);
#ifndef CONFIG_ASUS_ZC550KL_PROJECT
	proc_create(DUMP_LCD_REGISTER, 0644, NULL, &lcd_reg_proc_ops);
	//ASUS_BSP: Louis --

    //ASUS BSP Bernard +++
    proc_create(DUMP_CALIBRATION_INFO, 0666, NULL, &lcd_info_proc_ops);
    //ASUS BSP Bernard ---
#endif
	return 0;
}
