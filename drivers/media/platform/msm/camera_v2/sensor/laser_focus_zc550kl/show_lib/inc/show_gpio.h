/*
*
* Author:		Jheng-Siou, Cai
* Time:		2015-05
*
*/

#ifndef __LINUX_SHOW_SENSOR_GPIO_H
#define __LINUX_SHOW_SENSOR_GPIO_H

#include "msm_laser_focus.h"
#include <linux/of_gpio.h>

/* GPIO digital signal */
#define GPIO_LOW	0
#define GPIO_HIGH	1

#define GPIO_OUT_LOW          (0 << 1)
#define GPIO_OUT_HIGH         (1 << 1)

/* Set GPIO to high */
int GPIO_UP(struct msm_camera_power_ctrl_t *power_info, int gpio_num);
/* Set GPIO to low */
int GPIO_DOWN(struct msm_camera_power_ctrl_t *power_info, int gpio_num);
/* Handle GPIO */
int GPIO_Handler(struct msm_laser_focus_ctrl_t *a_ctrl, int gpio_num, int ctrl);

#endif
