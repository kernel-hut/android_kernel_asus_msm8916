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
#ifndef __LINUX_GSENSOR_MOTION_DETECTION_H
#define __LINUX_GSENSOR_MOTION_DETECTION_H

/*************************************************
 *                  ZenMotion state                    *
 *************************************************/
#define DTAP_STATE				(1 << 0)
#define HANDS_STATE				(1 << 1)
#define FLIP_STATE				(1 << 2)
#define FLICK_STATE				(1 << 3)
#define MOVING_STATE			(1 << 4)

#define FLICK_CHECK			2

#define FLICK_TWICE			3
#define HANDS_UP				4
#define HANDS_DOWN			5
#define FACE_UP 				6
#define FACE_DOWN 			7
#define DOUBLE_TAP 			8

#define ACTIVITY_STATE_MOVING			10
#define ACTIVITY_STATE_NOT_MOVING		11

#define FUNSTAT_ON		1
#define FUNSTAT_OFF		0

/*************************************************
 *                      Event state                        *
 *************************************************/
#define KX022_DOUBLETAP_EVENT	0x08
#define KX022_FLIP_EVENT			0x01	/* For HANDS UP/HANDS DOWN */
#define KX022_WUF_EVENT			0x02	/* For WUFS */

/*************************************************
 *   Motion detect sensortifity( For Flick )   *
 *************************************************/
#define KX022_FLICK_DETECT_1G_GFORCE		0b00010000
#define KX022_FLICK_DETECT_075G_GFORCE		0b00001100
#define KX022_FLICK_DETECT_050G_GFORCE		0b00001000

/**************************************************************
 *   Wakeup/Motion setting( For Moving detection )   *
 **************************************************************/
#define WUFE_WUFC_VAL				0b00001000
#define WUFE_ATH_VAL				0b00000100
#define CNTL3_3HZ_OWUF_VAL			0b00000010

#define WUFE_INTERRUPT_WAIT_TIME_MS	(15*1000)
#define FLICK_INTERRUPT_WAIT_TIME_MS	(1*1000)

#endif

