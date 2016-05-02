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

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME	"ASH"

#undef dbg
#ifdef ASH_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s] "fmt,MODULE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s] "fmt,MODULE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s] "fmt,MODULE_NAME,##args)
