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
#include "../ASH_log.h"

#define IR_INTEL_NAME 	"ALS_INT#"
#define IR_QCOM_NAME 	"qcom,alsp-gpio"
#define IR_IRQ_NAME		"IR_SENSOR_IRQ"
#define IR_INT_NAME		"IR_SENSOR_INT"

static int ASUS_IR_SENSOR_GPIO;
static IRsensor_GPIO * mIRsensor_GPIO;

static irqreturn_t IRsensor_irq_handler(int irq, void *dev_id);

#ifdef GPIO_INTEL
#include <asm/intel-mid.h>
#endif

#ifdef GPIO_QCOM
#include <linux/of_gpio.h>
#define GPIO_LOOKUP_STATE	"cm36686_gpio_high"

static void set_pinctrl(struct i2c_client *client)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	
	key_pinctrl = devm_pinctrl_get(&client->dev);
	set_state = pinctrl_lookup_state(key_pinctrl, GPIO_LOOKUP_STATE);
	ret = pinctrl_select_state(key_pinctrl, set_state);
	log("%s: pinctrl_select_state = %d\n", __FUNCTION__, ret);
}
#endif
static int init_irq (void)
{
	int ret = 0;
	int irq = 0;

	/* GPIO to IRQ */
	irq = gpio_to_irq(ASUS_IR_SENSOR_GPIO);
	if (irq < 0) {
		err("gpio_to_irq ERROR, irq=%d.\n", irq);
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
		err("request_irq() ERROR %d.\n", ret);
		return ret;
	}else {		
		log("Disable irq !! \n");
		disable_irq(irq);
	}

	return irq;
}

irqreturn_t IRsensor_irq_handler(int irq, void *dev_id)
{
	mIRsensor_GPIO->IRsensor_isr();
	return IRQ_HANDLED;
}

int IRsensor_gpio_register(struct i2c_client *client, IRsensor_GPIO *gpio_ist)
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
	set_pinctrl(client);
	ASUS_IR_SENSOR_GPIO = of_get_named_gpio(client->dev.of_node, IR_QCOM_NAME, 0);	
	#endif
		
	log("[GPIO] GPIO =%d(%d)\n", ASUS_IR_SENSOR_GPIO, gpio_get_value(ASUS_IR_SENSOR_GPIO));	
	/* GPIO Request */
	ret = gpio_request(ASUS_IR_SENSOR_GPIO, IR_IRQ_NAME);
	if (ret) {
		err("Unable to request gpio %s(%d)\n", IR_IRQ_NAME, ASUS_IR_SENSOR_GPIO);
		return ret;
	}
	/* GPIO Direction */
	ret = gpio_direction_input(ASUS_IR_SENSOR_GPIO);
	if (ret < 0) {
		err("Unable to set the direction of gpio %d\n", ASUS_IR_SENSOR_GPIO);
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
