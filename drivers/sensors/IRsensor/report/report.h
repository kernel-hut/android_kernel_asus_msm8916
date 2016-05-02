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
 
/*****************************/
/* IR Sensor Report Module */
/***************************/
#ifndef __LINUX_IRSENSOR_REPORT_H
#define __LINUX_IRSENSOR_REPORT_H

#define IRSENSOR_REPORT_PS_CLOSE 				(0)
#define IRSENSOR_REPORT_PS_AWAY     			(1) 

extern int IRsensor_report_event_register(void);
extern void IRsensor_report_event_unregister(void);

extern void proximity_report_abs(int abs);
extern void light_report_lux(int lux);

#endif