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
#ifndef _IMC_IDI_GNSS_PM_H
#define _IMC_IDI_GNSS_PM_H

#ifdef CONFIG_PINCTRL_SINGLE
int gnss_set_pinctrl_state(struct device *dev, struct pinctrl_state *state);
#endif

#ifdef NATIVE_PLATFORM_DEVICE_PM
int gnss_pm_add_class(void);
#endif

#endif /* _IMC_IDI_GNSS_PM_H */
