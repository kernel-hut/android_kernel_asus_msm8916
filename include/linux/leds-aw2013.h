/*
 * AW2013 LED chip driver.
 */

#ifndef __LINUX_AW2013_H
#define __LINUX_AW2013_H
#include <linux/leds.h>

struct aw2013_platform_data {
	struct led_platform_data leds;
};

#define	AW2013_MAKE_GPIO 1
#endif /* __LINUX_AW2013_H*/
