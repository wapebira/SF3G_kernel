/*
 *
 * Copyright (C) 2014-2015 Intel Mobile Communications GmbH
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

#ifndef _LINUX_ROCKCHIP_ION_H
#define _LINUX_ROCKCHIP_ION_H

#ifdef __KERNEL__
#include "../../drivers/staging/android/ion/ion.h"
#else
#include <linux/ion.h>
#endif

static struct ion_phys_data {
	ion_user_handle_t handle;
	unsigned long phys;
	unsigned long size;
};

#define ION_IOC_ROCKCHIP_MAGIC 'R'

/* Get phys addr of the handle specified. */
#define ION_IOC_GET_PHYS	_IOWR(ION_IOC_ROCKCHIP_MAGIC, 0, \
						struct ion_phys_data)
/* Get share object of the fd specified. */
#define ION_IOC_ALLOC_SECURE	_IOWR(ION_IOC_ROCKCHIP_MAGIC, 1, \
				struct ion_phys_data)

/* Free secure region alloced. */
#define ION_IOC_FREE_SECURE	_IOWR(ION_IOC_ROCKCHIP_MAGIC, 2, \
				struct ion_phys_data)

struct ion_client *rockchip_ion_client_create(const char *name);

int rk_ion_handler_init(struct device_node *node,
	struct ion_device *idev, struct ion_platform_data *pdata);

void rk_ion_handler_exit(void);

#endif
