/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifndef _IMC_IDI_GNSS_OF_H
#define _IMC_IDI_GNSS_OF_H

#include "imc_idi_gnss.h"

#ifdef CONFIG_OF
struct imc_idi_gnss_platdata *imc_idi_gnss_of_get_platdata(
				struct idi_peripheral_device *p_device);
#endif

#endif /* _IMC_IDI_GNSS_OF_H */
