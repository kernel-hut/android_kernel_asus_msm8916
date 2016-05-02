/* 
 * Copyright (C) 2014 ASUSTek Inc.
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
#ifndef __LINUX_IRSENSOR_GPIO_H
#define __LINUX_IRSENSOR_GPIO_H

#define GPIO_NAME 	"ALS_INT#"
#define GPIO_QCOM_NAME 	"qcom,alsp-gpio"
#define IRQ_Name		"IR_SENSOR_IRQ"
#define INT_NAME		"IR_SENSOR_INT"

#include <linux/i2c.h>
extern int IRsensor_gpio_register(struct i2c_client *client);
extern int IRsensor_gpio_unregister(int irq);

#endif

