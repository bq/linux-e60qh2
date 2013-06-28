/* drivers/input/touchscreen/zforce.c
 *
 * Copyright (C) 2010 Barnes & Noble, Inc.
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

#ifndef _LINUX_ZFORCE_H
#define _LINUX_ZFORCE_H

struct zforce_ts_platdata {
	int gpio_int;
	int gpio_rst;

	unsigned int x_max;
	unsigned int y_max;
};

#endif /* _LINUX_ZFORCE_H */
