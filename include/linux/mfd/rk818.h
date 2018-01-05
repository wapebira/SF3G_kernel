/*
 * Copyright (c) 2014, Fuzhou Rockchip Electronics Co., Ltd
 * Author: zhangqing <zhangqing@rock-chips.com>
 * Copyright (C) 2014-2015 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef __LINUX_REGULATOR_rk818_H
#define __LINUX_REGULATOR_rk818_H

#include <linux/regulator/machine.h>
#include <linux/wakelock.h>
#include <linux/power_supply.h>

#if defined(CONFIG_DEBUG_FS)
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#define RK818_DCDC1			0

#define RK818_LDO1			4

#define RK818_SECONDS_REG		0x00
#define RK818_MINUTES_REG		0x01
#define RK818_HOURS_REG			0x02
#define RK818_DAYS_REG			0x03
#define RK818_MONTHS_REG		0x04
#define RK818_YEARS_REG			0x05
#define RK818_WEEKS_REG			0x06
#define RK818_ALARM_SECONDS_REG		0x08
#define RK818_ALARM_MINUTES_REG		0x09
#define RK818_ALARM_HOURS_REG		0x0a
#define RK818_ALARM_DAYS_REG		0x0b
#define RK818_ALARM_MONTHS_REG		0x0c
#define RK818_ALARM_YEARS_REG		0x0d
#define RK818_RTC_CTRL_REG		0x10
#define RK818_RTC_STATUS_REG		0x11
#define RK818_RTC_INT_REG		0x12
#define RK818_RTC_COMP_LSB_REG		0x13
#define RK818_RTC_COMP_MSB_REG		0x14
#define RK818_CHIP_NAME_MSB_REG		0x17
#define RK818_CHIP_NAME_LSB_REG		0x18
#define RK818_CLK32OUT_REG		0x20
#define RK818_VB_MON_REG		0x21
#define RK818_THERMAL_REG		0x22
#define RK818_DCDC_EN_REG		0x23
#define RK818_LDO_EN_REG		0x24
#define RK818_SLEEP_SET_OFF_REG1	0x25
#define RK818_SLEEP_SET_OFF_REG2	0x26
#define RK818_DCDC_UV_STS_REG		0x27
#define RK818_DCDC_UV_ACT_REG		0x28
#define RK818_LDO_UV_STS_REG		0x29
#define RK818_LDO_UV_ACT_REG		0x2a
#define RK818_DCDC_PG_REG		0x2b
#define RK818_LDO_PG_REG		0x2c
#define RK818_VOUT_MON_TDB_REG		0x2d
#define RK818_BUCK1_CONFIG_REG		0x2e
#define RK818_BUCK1_ON_REG		0x2f
#define RK818_BUCK1_SLP_REG		0x30
#define RK818_BUCK2_CONFIG_REG		0x32
#define RK818_BUCK2_ON_REG		0x33
#define RK818_BUCK2_SLP_REG		0x34
#define RK818_BUCK3_CONFIG_REG		0x36
#define RK818_BUCK4_CONFIG_REG		0x37
#define RK818_BUCK4_ON_REG		0x38
#define RK818_BUCK4_SLP_VSEL_REG	0x39
#define RK818_BOOST_CONFIG_REG		0x3a
#define RK818_LDO1_ON_VSEL_REG		0x3b
#define RK818_LDO1_SLP_VSEL_REG		0x3c
#define RK818_LDO2_ON_VSEL_REG		0x3d
#define RK818_LDO2_SLP_VSEL_REG		0x3e
#define RK818_LDO3_ON_VSEL_REG		0x3f
#define RK818_LDO3_SLP_VSEL_REG		0x40
#define RK818_LDO4_ON_VSEL_REG		0x41
#define RK818_LDO4_SLP_VSEL_REG		0x42
#define RK818_LDO5_ON_VSEL_REG		0x43
#define RK818_LDO5_SLP_VSEL_REG		0x44
#define RK818_LDO6_ON_VSEL_REG		0x45
#define RK818_LDO6_SLP_VSEL_REG		0x46
#define RK818_LDO7_ON_VSEL_REG		0x47
#define RK818_LDO7_SLP_VSEL_REG		0x48
#define RK818_LDO8_ON_VSEL_REG		0x49
#define RK818_LDO8_SLP_VSEL_REG		0x4a
#define RK818_BOOST_LDO9_ON_VSEL_REG	0x54
#define RK818_BOOST_LDO9_SLP_VSEL_REG	0x55
#define RK818_DEVCTRL_REG		0x4b
#define RK818_INT_STS_REG1		0x4c
#define RK818_INT_STS_MSK_REG1		0x4d
#define RK818_INT_STS_REG2		0x4e
#define RK818_INT_STS_MSK_REG2		0x4f
#define RK818_IO_POL_REG		0x50
#define CHRG_COMP_REG1                  0x99
#define CHRG_COMP_REG2                  0x9A
#define RK818_SUP_STS_REG		0xA0
#define USB_CTRL_REG                    0xA1
#define CHRG_CTRL_REG1                  0xA3
#define CHRG_CTRL_REG2                  0xA4
#define CHRG_CTRL_REG3                  0xA5
#define BAT_CTRL_REG                    0xA6
#define BAT_HTS_TS1_REG                 0xA8
#define BAT_LTS_TS1_REG                 0xA9
#define BAT_HTS_TS2_REG                 0xAA
#define BAT_LTS_TS2_REG                 0xAB

#define TS_CTRL_REG                     0xAC
#define ADC_CTRL_REG                    0xAD

#define RK818_ON_SOURCE_REG		0xAE
#define RK818_OFF_SOURCE_REG		0xAF

#define GGCON                           0xB0
#define GGSTS                           0xB1
#define FRAME_SMP_INTERV_REG            0xB2
#define AUTO_SLP_CUR_THR_REG            0xB3

#define GASCNT_CAL_REG3                 0xB4
#define GASCNT_CAL_REG2                 0xB5
#define GASCNT_CAL_REG1                 0xB6
#define GASCNT_CAL_REG0                 0xB7
#define GASCNT3                         0xB8
#define GASCNT2                         0xB9
#define GASCNT1                         0xBA
#define GASCNT0                         0xBB

#define BAT_CUR_AVG_REGH                0xBC
#define BAT_CUR_AVG_REGL                0xBD
#define TS1_ADC_REGH                    0xBE
#define TS1_ADC_REGL                    0xBF
#define TS2_ADC_REGH                    0xC0
#define TS2_ADC_REGL                    0xC1

#define BAT_OCV_REGH                    0xC2
#define BAT_OCV_REGL                    0xC3
#define BAT_VOL_REGH                    0xC4
#define BAT_VOL_REGL                    0xC5

#define RELAX_ENTRY_THRES_REGH          0xC6
#define RELAX_ENTRY_THRES_REGL          0xC7
#define RELAX_EXIT_THRES_REGH           0xC8
#define RELAX_EXIT_THRES_REGL           0xC9

#define RELAX_VOL1_REGH                 0xCA
#define RELAX_VOL1_REGL                 0xCB
#define RELAX_VOL2_REGH                 0xCC
#define RELAX_VOL2_REGL                 0xCD

#define BAT_CUR_R_CALC_REGH             0xCE
#define BAT_CUR_R_CALC_REGL             0xCF
#define BAT_VOL_R_CALC_REGH             0xD0
#define BAT_VOL_R_CALC_REGL             0xD1

#define CAL_OFFSET_REGH                 0xD2
#define CAL_OFFSET_REGL                 0xD3

#define NON_ACT_TIMER_CNT_REG           0xD4

#define VCALIB0_REGH                    0xD5
#define VCALIB0_REGL                    0xD6
#define VCALIB1_REGH                    0xD7
#define VCALIB1_REGL                    0xD8

#define IOFFSET_REGH                    0xDD
#define IOFFSET_REGL                    0xDE
/*0xE0 ~0xF2  data register,*/
#define  SOC_REG                        0xE0

#define  REMAIN_CAP_REG3                0xE1
#define  REMAIN_CAP_REG2                0xE2
#define  REMAIN_CAP_REG1                0xE3
#define  REMAIN_CAP_REG0                0xE4

#define UPDAT_LEVE_REG                  0xE5

#define  NEW_FCC_REG3                   0xE6
#define  NEW_FCC_REG2                   0xE7
#define  NEW_FCC_REG1                   0xE8
#define  NEW_FCC_REG0                   0xE9

#define NON_ACT_TIMER_CNT_REG_SAVE      0xEA
#define SOFT_SHTD_REG			0xEB

#define UBT_INIT_SOC_REG                0xEC
#define UBT_INIT_TEMP_SOC_REG           0xED
#define UBT_INIT_BRANCH                 0xEE
#define UBT_PWRON_SOC_REG               0xEF
#define SYS_RUN_MAGIC                   0xF0
#define RK818_DATA18_REG_SYS_DEBUG_COPY	0xf1
#define RK818_DATA19_REG_SYS_DEBUG	0xf2

#define SYS_REBOOT_SHUTDOWN_MAGIC	0x00

#define SYS_DEBUG_SUSPEND		(1<<0)
#define SYS_DEBUG_RESUME		(1<<1)
#define SYS_DEBUG_REBOOT		(1<<2)
#define SYS_DEBUG_SHUTDOWN		(1<<3)

/* IRQ Definitions */
#define RK818_IRQ_VOUT_LO		0
#define RK818_IRQ_VB_LO			1
#define RK818_IRQ_PWRON			2
#define RK818_IRQ_PWRON_LP		3
#define RK818_IRQ_HOTDIE		4
#define RK818_IRQ_RTC_ALARM		5
#define RK818_IRQ_RTC_PERIOD		6
#define RK818_IRQ_USB_OV		7
#define RK818_IRQ_PLUG_IN		8
#define RK818_IRQ_PLUG_OUT		9
#define RK818_IRQ_CHG_OK		10
#define RK818_IRQ_CHG_TE		11
#define RK818_IRQ_CHG_TS1		12
#define RK818_IRQ_TS2			13
#define RK818_IRQ_CHG_CVTLIM		14
#define RK818_IRQ_DISCHG_ILIM		6

#define RK818_NUM_IRQ			16

#define rk818_NUM_REGULATORS		14
struct rk818;

#define RK818_VBAT_LOW_2V8		0x00
#define RK818_VBAT_LOW_2V9		0x01
#define RK818_VBAT_LOW_3V0		0x02
#define RK818_VBAT_LOW_3V1		0x03
#define RK818_VBAT_LOW_3V2		0x04
#define RK818_VBAT_LOW_3V3		0x05
#define RK818_VBAT_LOW_3V4		0x06
#define RK818_VBAT_LOW_3V5		0x07
#define VBAT_LOW_VOL_MASK		(0x07 << 0)
#define EN_VABT_LOW_SHUT_DOWN		(0x00 << 4)
#define EN_VBAT_LOW_IRQ			(0x1 << 4)
#define VBAT_LOW_ACT_MASK		(0x1 << 4)

#define RK818_CHIP_ID_NUM 0x8181
#define RK819_CHIP_ID_NUM 0x8191
#define PMIC_CHIP_RK818 0x818
#define PMIC_CHIP_RK819 0x819
#define PMIC_CHIP_UNKNOWN 0

#define RK818_PWRON_LP_ACT_MASK		(0x1 << 6)
#define RK818_PWRON_LP_OFF_TIME_MASK	(0x3 << 4)
#define RK818_PWRON_LP_ACT_SHUTDOWN	(0x0 << 6)
#define RK818_PWRON_LP_OFF_TIME_6S	(0x0 << 4)

#define RK818_OTG_EN_MASK		(0x1 << 7)

enum {
	VBUS_OFF = 0,
	VBUS_ON,
};

struct rk818 {
	int bat_test_mode;
	struct device *dev;
	int chip_id;
	struct rt_mutex io_lock;	/*protect rk818 bit set and clear*/
	struct i2c_client *i2c;
	struct wake_lock irq_wake;
	/*irq_lock used in rk818 lock and sync unclock functions*/
	struct rt_mutex irq_lock;
	int irq_base;
	int irq_num;
	int chip_irq;
	u32 irq_mask;
	struct irq_domain *irq_domain;
	int (*read)(struct rk818 *rk818, u8 reg, int size, void *dest);
	int (*write)(struct rk818 *rk818, u8 reg, int size, void *src);
	int pmic_sleep_gpio;	/* */
	bool pmic_sleep;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct workqueue_struct *wq;
	struct delayed_work otg_delay_work;
	struct usb_phy *otg_handle;
	struct notifier_block otg_nb;
	int otg_en_boost;
	int vbus;
	int vbus_state_prev;
	int notify_usb_det;
	struct dentry *debugfs_dir;
#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device_pm_platdata *pm_platdata;
#endif
};

int rk818_irq_init(struct rk818 *rk818);
int rk818_i2c_read(struct rk818 *rk818, u8 reg, int count, u8 *dest);
int rk818_i2c_write(struct rk818 *rk818, u8 reg, int count, u8 src);
int rk818_set_bits(struct rk818 *rk818, u8 reg, u8 mask, u8 val);
int rk818_clear_bits(struct rk818 *rk818, u8 reg, u8 mask);
u8 rk818_reg_read(struct rk818 *rk818, u8 reg);
int rk818_reg_write(struct rk818 *rk818, u8 reg, u8 val);
int rk818_bulk_read(struct rk818 *rk818, u8 reg, int count, u8 *buf);
int rk818_bulk_write(struct rk818 *rk818, u8 reg, int count, u8 *buf);
void rk818_device_shutdown(void);
#endif
