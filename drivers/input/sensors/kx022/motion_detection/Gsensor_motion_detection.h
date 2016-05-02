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

#define FLICK_CHECK				2

/*************************************************
 *                      Event state                        *
 *************************************************/
#define KX022_DOUBLETAP_EVENT	0x08
#define KX022_FLIP_EVENT			0x01	/* For HANDS UP/HANDS DOWN */
#define KX022_WUF_EVENT			0x02	/* For WUFS */

#endif

