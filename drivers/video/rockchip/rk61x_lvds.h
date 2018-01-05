/*
 * Header file for rockchip rk616/rk618 lvds driver
 *
 * Copyright (C) 2013-2015 Rockchip Electronics Co., Ltd.
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

#ifndef __RK61X_LVDS__
#define __RK61X_LVDS__
#include <linux/mfd/rk61x.h>
#include <linux/rockchip_screen.h>

/*
 * Data Structure Definition
 */
struct rk61x_lvds {
	bool sys_state;
	struct mfd_rk61x *rk61x;
	struct rockchip_screen screen;
};

#endif
