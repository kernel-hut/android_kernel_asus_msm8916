/* 
 * Copyright (C) 2015 ASUSTek Inc.
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

 /***************************/
/* IR Sensor GPIO Module */
/**************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/input/ASH.h>

#define IR_INTEL_NAME 	"ALS_INT#"
#define IR_QCOM_NAME 	"qcom,psals-gpio"
#define IR_IRQ_NAME		"IR_SENSOR_IRQ"
#define IR_INT_NAME		"IR_SENSOR_INT"

static int ASUS_IR_SENSOR_GPIO;
static IRsensor_GPIO * mIRsensor_GPIO;
static struct i2c_client *g_i2c_client;

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_GPIO"
#define SENSOR_TYPE_NAME		"IRsensor"

#undef dbg
#ifdef ASH_GPIO_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

static irqreturn_t IRsensor_irq_handler(int irq, void *dev_id);

#ifdef GPIO_INTEL
#include <asm/intel-mid.h>
#endif

#ifdef GPIO_QCOM
#include <linux/of_gpio.h>
#define GPIO_LOOKUP_STATE	"psals_gpio_high"

static void set_pinctrl(struct i2c_client *client)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	
	key_pinctrl = devm_pinctrl_get(&client->dev);
	set_state = pinctrl_lookup_state(key_pinctrl, GPIO_LOOKUP_STATE);
	ret = pinctrl_select_state(key_pinctrl, set_state);
	if(ret < 0)
		err("%s: pinctrl_select_state ERROR(%d).\n", __FUNCTION__, ret);
}
#endif
static int init_irq (void)
{
	int ret = 0;
	int irq = 0;

	/* GPIO to IRQ */
	irq = gpio_to_irq(ASUS_IR_SENSOR_GPIO);
	if (irq < 0) {
		err("%s: gpio_to_irq ERROR(%d). \n", __FUNCTION__, irq);
		return irq;
	}else {
		log("gpio_to_irq IRQ %d successed on GPIO:%d\n", irq, ASUS_IR_SENSOR_GPIO);
	}

	/*Request IRQ*/	
	#ifdef GPIO_INTEL
	ret = request_irq(irq,IRsensor_irq_handler, IRQF_TRIGGER_LOW, IR_INT_NAME, NULL);
	#endif
	
	#ifdef GPIO_QCOM
	ret = request_threaded_irq(irq, NULL, IRsensor_irq_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT, IR_INT_NAME, NULL);
	#endif
	
	if (ret < 0) {
		err("%s: request_irq/request_threaded_irq ERROR(%d).\n", __FUNCTION__, ret);
		return ret;
	}else {		
		dbg("Disable irq !! \n");
		disable_irq(irq);
	}

	return irq;
}

irqreturn_t IRsensor_irq_handler(int irq, void *dev_id)
{
	mIRsensor_GPIO->IRsensor_isr();
	return IRQ_HANDLED;
}

int IRsensor_gpio_register(IRsensor_GPIO *gpio_ist)
{
	int ret = 0;
	int irq = 0;

	mIRsensor_GPIO = gpio_ist;
	
	/* GPIO */
	#ifdef GPIO_INTEL
	log("Intel GPIO \n");
	ASUS_IR_SENSOR_GPIO = get_gpio_by_name(IR_INTEL_NAME);
	#endif
	
	#ifdef GPIO_QCOM
	log("Qcom GPIO \n");
	if(g_i2c_client == NULL){
		err("g_i2c_client is NULL, Please set I2c Client first. \n");
		return -1;
	}		
	set_pinctrl(g_i2c_client);
	ASUS_IR_SENSOR_GPIO = of_get_named_gpio(g_i2c_client->dev.of_node, IR_QCOM_NAME, 0);	
	#endif
		
	dbg("GPIO =%d(%d)\n", ASUS_IR_SENSOR_GPIO, gpio_get_value(ASUS_IR_SENSOR_GPIO));	
	/* GPIO Request */
	ret = gpio_request(ASUS_IR_SENSOR_GPIO, IR_IRQ_NAME);
	if (ret) {
		err("%s: gpio_request ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}
	/* GPIO Direction */
	ret = gpio_direction_input(ASUS_IR_SENSOR_GPIO);
	if (ret < 0) {
		err("%s: gpio_direction_input ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}
	/*IRQ*/
	irq = init_irq();
	
	return irq;

}
EXPORT_SYMBOL(IRsensor_gpio_register);


int IRsensor_gpio_unregister(int irq)
{
	free_irq(irq, NULL);
	gpio_free(ASUS_IR_SENSOR_GPIO);
	return 0;
}
EXPORT_SYMBOL(IRsensor_gpio_unregister);

int IRsensor_gpio_setI2cClient(struct i2c_client *client)
{
	if(client != NULL){
		g_i2c_client=client;
		dbg("IRsensor_gpio_getI2cClient Success. \n");
	}else{
		err("%s: i2c_client is NULL. \n", __FUNCTION__);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(IRsensor_gpio_setI2cClient);