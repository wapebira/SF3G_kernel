/*
 ****************************************************************
 *
 * Copyright (C) 2014-2015 Fuzhou Rockchip Electronics Co., Ltd

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 ******************************************************************
 */
 #ifndef _CIF_ISP20_RK_VERSION_H_
#define _CIF_ISP20_RK_VERSION_H_
#include <linux/version.h>

 /*
*       CIF DRIVER VERSION NOTE
*
*v1.0.0:
*     1.  Buffer_queue list which is operated in cif_isp20_mi_frame_end is not protected when in interrupt context,  add spin_lock;
*v1.0.1:
*     1.  fix the problem that kernel someimes panic when cif write register
*         via add the spin_lock in cif_iowrite32_verify;
*v1.0.2:
*     1.  Increase the processing that turn the flashlight off
*         when system shutdown.
*v1.0.3:
*     1.  Revert intel patch: sofia3gr_camera_cif_driver20150528.tgz. Because it may cause Rendorthread fault addr ;
*v1.0.4:
*     1.  modify cif driver for Compatibling with a variety of cameras ;
*v1.0.5:
*     1.  modify cif driver for Compatibling with a variety of cameras. Customers only modify dts file ;
*v1.0.6:
*     1.  fix the problem that zoom ratio is not same when switch zsl between normal. ;
*/

#define CONFIG_CIFDRIVER_VERSION KERNEL_VERSION(1, 0, 6)

#endif