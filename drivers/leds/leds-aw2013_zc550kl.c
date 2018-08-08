/*
 * leds-aw2013
 *
 * The AW2013 is a programmable LED controller that can drive 7
 * separate lines either by holding them low, or by pulsing them
 * with modulated width.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/leds-aw2013_zc550kl.h>
#include <linux/of.h>

#include "i2c-dev-dbgfs.h"


#define TIMECODES 8
#define FULL_ON_CODES 6
#define DELAY_CODES 9
#define DFAULT_FADE_ON_TIME 130
#define DFAULT_FADE_OFF_TIME 130
#define DFAULT_DELAY_TIME 0

static int fade_on_time_codes[TIMECODES] = {
	130, 260, 520, 1040, 2080, 4160, 8320, 16640
};

static int full_on_time_codes[FULL_ON_CODES] = {
	130, 260, 520, 1040, 2080, 4160
};

static int fade_off_time_codes[TIMECODES] = {
	130, 260, 520, 1040, 2080, 4160, 8320, 16640
};

static int full_off_time_codes[TIMECODES] = {
	130, 260, 520, 1040, 2080, 4160, 8320, 16640
};

static int delay_time_codes[DELAY_CODES] = {
	0, 130, 260, 520, 1040, 2080, 4160, 8320, 16640
};


//define adderss of  registers
#define	AW2013_REG_RSTR_ADDR	0x00
#define	AW2013_REG_GCR_ADDR	0x01
#define	AW2013_REG_ISR_ADDR	0x02

#define	AW2013_REG_LCTR_ADDR	0x30
#define	AW2013_REG_LCFG1_ADDR	0x31
#define	AW2013_REG_LCFG2_ADDR	0x32
#define	AW2013_REG_LCFG3_ADDR	0x33
#define	AW2013_REG_PWM1_ADDR	0x34
#define	AW2013_REG_PWM2_ADDR	0x35
#define	AW2013_REG_PWM3_ADDR	0x36
#define	AW2013_REG_LED0T0_ADDR	0x37
#define	AW2013_REG_LED0T1_ADDR	0x38
#define	AW2013_REG_LED0T2_ADDR	0x39
#define	AW2013_REG_LED1T0_ADDR	0x3A
#define	AW2013_REG_LED1T1_ADDR	0x3B
#define	AW2013_REG_LED1T2_ADDR	0x3C
#define	AW2013_REG_LED2T0_ADDR	0x3D
#define	AW2013_REG_LED2T1_ADDR	0x3E
#define	AW2013_REG_LED2T2_ADDR	0x3F

#define	AW2013_REG_IADR_ADDR	0x77

static u8 sts_reg_addr[3] = {
	AW2013_REG_RSTR_ADDR,
	AW2013_REG_GCR_ADDR,
	AW2013_REG_ISR_ADDR,
};

static u8 ctrl_reg_addr[16] = {
	AW2013_REG_LCTR_ADDR,
	AW2013_REG_LCFG1_ADDR,
	AW2013_REG_LCFG2_ADDR,
	AW2013_REG_LCFG3_ADDR,
	AW2013_REG_PWM1_ADDR,
	AW2013_REG_PWM2_ADDR,
	AW2013_REG_PWM3_ADDR,
	AW2013_REG_LED0T0_ADDR,
	AW2013_REG_LED0T1_ADDR,
	AW2013_REG_LED0T2_ADDR,
	AW2013_REG_LED1T0_ADDR,
	AW2013_REG_LED1T1_ADDR,
	AW2013_REG_LED1T2_ADDR,
	AW2013_REG_LED2T0_ADDR,
	AW2013_REG_LED2T1_ADDR,
	AW2013_REG_LED2T2_ADDR,
};

#define AW2013_STS_REG_CNT 0x03
#define AW2013_CTRL_REG_CNT 0x10

#define	LCFG_INDEX_BASE 0x01		//the begin index of AW2013_REG_LCFG[x] in all ctrl registers
#define	PWM_INDEX_BASE 0x04		//the begin index of AW2013_REG_PWM[x] in all ctrl registers
#define	TIME_INDEX_BASE 0x07		//the begin index of AW2013_REG_LED0T0 in all ctrl registers

#define NUM_LEDS 2
struct aw2013_chip {
	u8		work_sts;

	u8		AW2013_REG_RSTR;
	u8		AW2013_REG_GCR;
	u8		AW2013_REG_ISR;

	u8		AW2013_REG_LCTR;
	u8		AW2013_REG_LCFG[3];
	u8		AW2013_REG_PWM[3];
	u8		AW2013_REG_LED0T0;
	u8		AW2013_REG_LED0T1;
	u8		AW2013_REG_LED0T2;
	u8		AW2013_REG_LED1T0;
	u8		AW2013_REG_LED1T1;
	u8		AW2013_REG_LED1T2;
	u8		AW2013_REG_LED2T0;
	u8		AW2013_REG_LED2T1;
	u8		AW2013_REG_LED2T2;

	u8		AW2013_REG_IADR;

	u8		sts_reg_set;
	u8		addr_reg_set;
	u16		ctrl_reg_set;

	struct i2c_client	*client;
	struct work_struct	work;
	spinlock_t		lock;

	struct aw2013_led {
		struct aw2013_chip	*chip;
		struct led_classdev	led_cdev;
		int			num;
		int			ontime, offtime;
		int			on_dflt, off_dflt;
		int			blink;	/* Set if hardware-blinking */
	} leds[NUM_LEDS];
};

static const struct i2c_device_id aw2013_id[] = {
	{ "aw2013" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw2013_id);


static int convert_times_to_code(int msec, int covert_array[], int array_size)
{
	int i;
	int down_level_code, up_level_code;
	int down_diff, up_diff;

	for (i = 0; i < array_size; i++) {
		int t = covert_array[i];
		if (t <= msec) {
			down_level_code = i;
			continue;
		}
		if (t > msec) {
			up_level_code = i;
			break;
		}
	}

	down_diff = msec - covert_array[down_level_code];
	up_diff = msec - covert_array[up_level_code];

	return down_diff>up_diff?up_level_code:down_level_code;
}

/* Write all needed register of aw2013 */
static void aw2013_work(struct work_struct *work)
{
	struct aw2013_chip *aw = container_of(work, struct aw2013_chip,	work);
	struct i2c_client *cl = aw->client;

	int sts_set, ctrl_set, addr_set, r;

	u8 sts_file[AW2013_STS_REG_CNT];
	u8 ctrl_file[AW2013_CTRL_REG_CNT];
	u8 addr_file;

	spin_lock_irq(&aw->lock);

	sts_set = aw->sts_reg_set;
	ctrl_set = aw->ctrl_reg_set;
	addr_set = aw->addr_reg_set;

	memcpy(sts_file, &(aw->AW2013_REG_RSTR), AW2013_STS_REG_CNT);
	memcpy(ctrl_file, &(aw->AW2013_REG_LCTR), AW2013_CTRL_REG_CNT);
	addr_file = aw->AW2013_REG_IADR;

	aw->sts_reg_set = 0;
	aw->ctrl_reg_set = 0;
	aw->addr_reg_set = 0;

	spin_unlock_irq(&aw->lock);

	if ( addr_set ) {
		i2c_smbus_write_byte_data(cl, AW2013_REG_IADR_ADDR, addr_file);
	}

	for (r = 0; r < AW2013_STS_REG_CNT; r++) {
		if (sts_set & (1<<r)) {
			i2c_smbus_write_byte_data(cl, sts_reg_addr[r], sts_file[r]);
		}
	}

	for (r = 0; r < AW2013_CTRL_REG_CNT; r++) {
		if (ctrl_set & (1<<r)) {
			i2c_smbus_write_byte_data(cl, ctrl_reg_addr[r], ctrl_file[r]);
		}
	}
}

static int register_prepare(struct aw2013_led *led)
{
	u8 level = 0xFF & (led->led_cdev.brightness);
	u8 t0,t1,t2,t3,t4;
	int mask;
	struct aw2013_chip *aw = led->chip;

	if (level == 0) {
		//turn led off
		mask = (1 << led->num);
		aw->AW2013_REG_LCTR &= ~mask;
		aw->ctrl_reg_set |= 0x01;

		//check if all led off
		aw->work_sts &= ~mask;
		if (0 == aw->work_sts) {
			aw->AW2013_REG_GCR &= 0xFE;
			aw->sts_reg_set |= 0x02;
		}
		return 0;
	} 


	//first, we only set the brightness
	//set led enable
	mask = (1 << led->num);
	aw->AW2013_REG_LCTR |= mask;

	//set max current && PWM,red led 5mA current,green led 15mA current
	aw->AW2013_REG_LCFG[led->num] = (led->num==1)?0x01:0x03;
	aw->AW2013_REG_PWM[led->num] = level;

	//check if it is first led work
	aw->work_sts |= mask;
	if (aw->work_sts != 0) {
		aw->AW2013_REG_GCR |= 0x01;
		aw->sts_reg_set |= 0x02;
	}

	//set register dirty flag
	mask = 1;
	mask |= (1<< ( led->num + LCFG_INDEX_BASE) );
	mask |= (1<< ( led->num + PWM_INDEX_BASE) );

	if (led->ontime != 0 && led->offtime != 0) {
	//then, we check need blink
		//set mode
		aw->AW2013_REG_LCFG[led->num] |= 0x10;

		t1 = 0x00|convert_times_to_code(DFAULT_FADE_ON_TIME,
			fade_on_time_codes,TIMECODES);
		t2 = 0x00|convert_times_to_code(led->ontime,
			full_on_time_codes, FULL_ON_CODES);
		t3 = 0x00|convert_times_to_code(DFAULT_FADE_ON_TIME,
			fade_off_time_codes,TIMECODES);
		t4 = 0x00|convert_times_to_code(led->offtime,
			full_off_time_codes, TIMECODES);
		t0 = 0x00|convert_times_to_code(DFAULT_DELAY_TIME,
			delay_time_codes, DELAY_CODES);

		//set time to all leds
		aw->AW2013_REG_LED0T0 = (t1<<4)|t2;
		aw->AW2013_REG_LED0T1 = (t3<<4)|t4;
		aw->AW2013_REG_LED0T2 = (t0<<4)|0x00;
		aw->AW2013_REG_LED1T0 = (t1<<4)|t2;
		aw->AW2013_REG_LED1T1 = (t3<<4)|t4;
		aw->AW2013_REG_LED1T2 = (t0<<4)|0x00;
		aw->AW2013_REG_LED2T0 = (t1<<4)|t2;
		aw->AW2013_REG_LED2T1 = (t3<<4)|t4;
		aw->AW2013_REG_LED2T2 = (t0<<4)|0x00;

		//only set register dirty flag to right led
		mask |= (1<< ( led->num + LCFG_INDEX_BASE) );
		mask |= (1<< ( led->num*3 + TIME_INDEX_BASE) );
		mask |= (1<< ( led->num*3 + TIME_INDEX_BASE +1) );
		mask |= (1<< ( led->num*3 + TIME_INDEX_BASE +2) );
	}
	aw->ctrl_reg_set |= mask;
	return 0;
}


static int led_assign(struct aw2013_led *led)
{
	struct aw2013_chip *aw = led->chip;
	int err;
	unsigned long flags;

	spin_lock_irqsave(&aw->lock, flags);

	err = register_prepare(led);

	spin_unlock_irqrestore(&aw->lock, flags);

	if (aw->sts_reg_set|aw->addr_reg_set|aw->ctrl_reg_set)
		schedule_work(&aw->work);
	return err;
}

static void aw2013_brightness_set(struct led_classdev *led_cdev,
				   enum led_brightness brightness)
{
	struct aw2013_led *led = container_of(led_cdev, struct aw2013_led,
							led_cdev);
	led->led_cdev.brightness = brightness;
	led->ontime = 0;
	led->offtime = 0;
	led_assign(led);
}

static int aw2013_blink_set(struct led_classdev *led_cdev,
			     unsigned long *delay_on,
			     unsigned long *delay_off)
{
	struct aw2013_led *led = container_of(led_cdev, struct aw2013_led,
							led_cdev);

	if (*delay_on == 0) {
		led->ontime = 512;
	}

	if (*delay_off == 0) {
		led->offtime = 2536;
	}

	if (led->led_cdev.brightness == LED_OFF) {
		led->led_cdev.brightness = LED_FULL;
	}

	led_assign(led);
	return 0;
}

#ifdef CONFIG_OF
static struct aw2013_platform_data *
aw2013_led_dt_init(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node, *child;
	struct aw2013_platform_data *pdata;
	struct led_info *aw_leds;
	int count;

	count = of_get_child_count(np);
	if (!count || count > NUM_LEDS)
		return ERR_PTR(-ENODEV);

	aw_leds = devm_kzalloc(&client->dev,
			sizeof(struct led_info) * count, GFP_KERNEL);
	if (!aw_leds)
		return ERR_PTR(-ENOMEM);

	for_each_child_of_node(np, child) {
		struct led_info led;
		u32 idx;
		int ret;

		led.name = of_get_property(child, "linux,name", NULL) ? : child->name;
		led.default_trigger = of_get_property(child, "linux,default-trigger", NULL);

		ret = of_property_read_u32(child, "idx", &idx);
		if (ret != 0)
			continue;

		aw_leds[idx] = led;
	}
	pdata = devm_kzalloc(&client->dev,
			sizeof(struct aw2013_platform_data), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->leds.leds = aw_leds;
	pdata->leds.num_leds = count;

	return pdata;
}
#else
static struct aw2013_platform_data *
aw2013_led_dt_init(struct i2c_client *client)
{
	return -EINVAL;
}
#endif /* !CONFIG_OF */

static ssize_t blink_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned long blinking;
	unsigned long delay_on=0;
	unsigned long delay_off=0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led = container_of(led_cdev, struct aw2013_led, led_cdev);
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &blinking);
	if (ret)
		return ret;

	if (led_cdev->blink_set && led_cdev->brightness_set && led!=NULL) {
		printk("[aw2013],blink store turn on, blinking = %lu\n",blinking);
		if (blinking == 0) {
			led_cdev->brightness_set(led_cdev,LED_OFF);
		} else {
			delay_on = led->ontime;
			delay_off = led->offtime;
			led_cdev->blink_set(led_cdev,&delay_on,&delay_off);
		}
	}
	return count;
}

static ssize_t on_ms_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned long on_ms;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led = container_of(led_cdev, struct aw2013_led, led_cdev);
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &on_ms);
	if (ret)
		return ret;

	if (led!=NULL) {
		printk("[aw2013],on_ms_store, on_ms = %lu\n",on_ms);
		led->ontime = on_ms;
	}
	return count;
}

static ssize_t off_ms_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned long off_ms;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led = container_of(led_cdev, struct aw2013_led, led_cdev);
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &off_ms);
	if (ret)
		return ret;

	if (led!=NULL) {
		printk("[aw2013],off_ms_store, on_ms = %lu\n",off_ms);
		led->offtime = off_ms;
	}
	return count;
}

static DEVICE_ATTR(blink, 0664, NULL, blink_store);
static DEVICE_ATTR(on_ms, 0664, NULL, on_ms_store);
static DEVICE_ATTR(off_ms, 0664, NULL, off_ms_store);


static struct attribute *blink_attrs[] = {
	&dev_attr_blink.attr,
	&dev_attr_on_ms.attr,
	&dev_attr_off_ms.attr,
	NULL
};

static const struct attribute_group blink_attr_group = {
	.attrs = blink_attrs,
};

static int aw2013_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct aw2013_chip *aw;
	struct i2c_adapter *adapter;
	struct aw2013_platform_data *pdata;
	int err;
	int i = 0;

	printk("[aw2013] %s\n", __func__);

	adapter = to_i2c_adapter(client->dev.parent);
	pdata = client->dev.platform_data;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	if (!pdata || pdata->leds.num_leds != NUM_LEDS) {
		pdata = aw2013_led_dt_init(client);
		if (IS_ERR(pdata)) {
			dev_err(&client->dev, "Need %d entries in platform-data list\n",
				NUM_LEDS);
			return PTR_ERR(pdata);
		}
	}
	aw = devm_kzalloc(&client->dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;

	aw->client = client;

	INIT_WORK(&aw->work, aw2013_work);
	spin_lock_init(&aw->lock);
	i2c_set_clientdata(client, aw);

	for (i = 0; i < NUM_LEDS; i++) {
		struct aw2013_led *l = aw->leds + i;

		l->chip = aw;
		l->num = i;
		if (pdata->leds.leds[i].name && !pdata->leds.leds[i].flags) {
			l->led_cdev.name = pdata->leds.leds[i].name;
			l->led_cdev.default_trigger
				= pdata->leds.leds[i].default_trigger;
			l->led_cdev.brightness_set = aw2013_brightness_set;
			l->led_cdev.blink_set = aw2013_blink_set;
			err = led_classdev_register(&client->dev, &l->led_cdev);
			if (err < 0)
				goto exit;

			err = sysfs_create_group(&l->led_cdev.dev->kobj,	&blink_attr_group);
			if (err < 0)
				printk("[aw2013],%s set blink sysfs failed",__func__);
		}
	}

#ifdef CONFIG_DEBUG_FS
i2c_dev_dfs_add_controller(client);
#endif

	//force reset all registers
	//aw->AW2013_REG_RSTR = 0x55;
	aw->AW2013_REG_IADR = 0x45;
	//aw->sts_reg_set = 0x01;
	//schedule_work(&aw->work);

	return 0;
exit:
	while (i--) {
		if (aw->leds[i].led_cdev.name)
			led_classdev_unregister(&aw->leds[i].led_cdev);
	}
	return err;
}

static int aw2013_remove(struct i2c_client *client)
{
	int i;
	struct aw2013_chip *aw = i2c_get_clientdata(client);
	struct aw2013_led *aw_leds = aw->leds;

	for (i = 0; i < NUM_LEDS; i++) {
		if (aw_leds[i].led_cdev.name)
			led_classdev_unregister(&aw_leds[i].led_cdev);
	}
	cancel_work_sync(&aw->work);

	return 0;
}

static struct of_device_id aw2013_match_table[] = {
	{ .compatible = "awinic,aw2013",},
	{ },
};

static struct i2c_driver aw2013_driver = {
	.driver   = {
		.name    = "aw2013",
		.owner   = THIS_MODULE,
		.of_match_table = aw2013_match_table,
	},
	.probe    = aw2013_probe,
	.remove   = aw2013_remove,
	.id_table = aw2013_id,
};

static int __init aw2013_init(void)
{
	return i2c_add_driver(&aw2013_driver);
}

static void __exit aw2013_exit(void)
{
	i2c_del_driver(&aw2013_driver);
}

module_init(aw2013_init);
module_exit(aw2013_exit);

MODULE_AUTHOR("jeff_gu@asus.com>");
MODULE_DESCRIPTION("AW2013 LED driver");
MODULE_LICENSE("GPL v2");
