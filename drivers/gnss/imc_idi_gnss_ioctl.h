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

#ifndef _IMC_IDI_GNSS_IOCTL_H
#define _IMC_IDI_GNSS_IOCTL_H

#define GNSS_MAGIC_IOC		'g'

/* CHIP_ID versions */
#define CHIP_ID_ES1             0x5700
#define CHIP_ID_ES1_1           0x5710
#define CHIP_ID_ES1_1_EXP       0x5711
#define CHIP_ID_ES2             0x5720
#define CHIP_ID_ES2_1		0x5721

/* IDI Power state definition */
enum idi_devices_power_state {D0, D0I0, D0I1, D0I2, D0I3, D3};

/* Frequency Manager feature values: */
enum fmr_freq {
	FMR_FRQ_DEF_DCLK_832_PCLK_1248, /* D0   */
	FMR_FRQ_DCLK_96_PCLK_1248,      /* D0I0 */
	FMR_FRQ_DCLK_832_PCLK_104,      /* D0I1 */
	FMR_FRQ_DCLK_96_PCLK_96,        /* D0I2
									   */
};
/* IOCTL get the CHIP ID: see definition of valid chip-ids up */
#define GNSS_IOC_GET_CHIPID		_IOR(GNSS_MAGIC_IOC, 0, unsigned int *)

/* IOCTL get the current IDI power state : see defs of valid power states up */
#define GNSS_IOC_GET_PWR_STATE	_IOR(GNSS_MAGIC_IOC, 1, unsigned int *)
/* IOCTL set the IDI power state */
#define GNSS_IOC_SET_PWR_STATE	_IOW(GNSS_MAGIC_IOC, 2, unsigned int)

/* IOCTL get the current frequency setting */
#define GNSS_IOC_GET_FRQ		_IOR(GNSS_MAGIC_IOC, 3, unsigned int *)
/* IOCTL set new frequency */
#define GNSS_IOC_SET_FRQ		_IOW(GNSS_MAGIC_IOC, 4, unsigned int)

#ifdef GNSS_DCLK_FTR
/* IOCTL get the current DCLK setting */
#define GNSS_IOC_GET_DCLCK		_IOR(GNSS_MAGIC_IOC, 5, unsigned int *)
/* IOCTL set new DCLK */
#define GNSS_IOC_SET_DCLCK		_IOW(GNSS_MAGIC_IOC, 6, unsigned int)
#endif /* GNSS_DCLK_FTR */
/* IOCTL wait for BB release after sleep */
#define GNSS_IOC_WAIT_FOR_BB_RELEASE	_IO(GNSS_MAGIC_IOC, 7)
#endif
