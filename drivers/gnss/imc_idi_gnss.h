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
#ifndef _IMC_IDI_GNSS_H
#define _IMC_IDI_GNSS_H

#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/serial_core.h>
#include <linux/tty_flip.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_tty.h>
#include <linux/platform_data/serial_xgold.h>

#include "imc_idi_gnss_regs.h"
#include "imc_idi_gnss_ioctl.h"

#ifdef CONFIG_WAKELOCK
#ifdef CONFIG_HAS_WAKELOCK
/*none*/
#else
#define CONFIG_HAS_WAKELOCK
#endif
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#else
#include <linux/idi/idi_of.h>
#endif

#ifdef CONFIG_PINCTRL
#include <linux/pinctrl/consumer.h>
#endif

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#else
#define gnss_wake_lock_set(x, y)
#endif

#define TX_BUFFER_SIZE			2048
#define MAX_GNSS_TX_SIZE		0x8000

#ifdef GNSS_FORCE_DEBUG
#ifdef dev_dbg
#undef dev_dbg
#endif
#define dev_dbg(dev, format, arg...) printk(format, ##arg)

#ifdef dev_info
#undef dev_info
#endif
#define dev_info(dev, format, arg...) printk(format, ##arg)

#ifdef dev_warn
#undef dev_warn
#endif
#define dev_warn(dev, format, arg...) printk(format, ##arg)

#ifdef pr_debug
#undef pr_debug
#endif
#define pr_debug printk
#endif

/* If no virtual pm driver, register callbacks to handle pm state locally */
#ifdef CONFIG_PLATFORM_DEVICE_PM
#if (defined(CONFIG_VPOWER_FE) || defined(CONFIG_PLATFORM_DEVICE_PM_VIRT))
#define VIRTUAL_PLATFORM_DEVICE_PM
#else
#define NATIVE_PLATFORM_DEVICE_PM
#endif
#endif

#define IMC_IDI_GNSS_ENTER		pr_debug("--> %s\n", __func__)
#define IMC_IDI_GNSS_EXIT		pr_debug("<-- %s\n", __func__)

#define GNSS_GNSS_RES_NAME		"gnss"
#define GNSS_RX_RES_NAME		"rx fifo"
#define GNSS_TX_RES_NAME		"tx fifo"
#define GNSS_REG_RES_NAME		"register"
#define GNSS_SCU_RES_NAME		"scu"

#define GNSS_IRQ_WAKEUP_NAME		"gnss_wakeup"
#define GNSS_IRQ_NOTIFICATION_NAME	"gnss_notification"
#define GNSS_IRQ_ERROR_NAME		"gnss_error"

#define SERIAL_GNSS_NAME		"ttyGNSS"
#define SERIAL_GNSS_MAJOR		4
#define SERIAL_GNSS_MINOR		208

#define GNSS_BB_DATA_CLK_NAME		"gps_bb_data"
#define GNSS_BB_CLK_NAME		"gps_bb"
#define GNSS_BB_EXT_UART_CLK_NAME	"gps_ext_uart"
#define GNSS_BB_RTC_CLK_NAME		"gps_rtc"
#define GNSS_REF_REQ_CLK_NAME		"gps_ref_req"
#define GNSS_BB_REQ_CLK_NAME		"gps_bb_req"
#define GNSS_RF_REQ_CLK_NAME		"gps_rf_req"
#define GNSS_SYS_REQ_CLK_NAME		"gps_sys_req"
#define GNSS_AUTO_PU_PHS1_CLK_NAME	"gps_auto_pu_phs1"
#define GNSS_AUTO_PU_PHS4_CLK_NAME	"gps_auto_pu_phs4"
#define GNSS_AUTO_PU_PHS5_CLK_NAME	"gps_auto_pu_phs5"

struct imc_idi_gnss_platdata {
#ifdef NATIVE_PLATFORM_DEVICE_PM
	struct clk *gps_bb_data;
	struct clk *gps_bb_clk;
	struct clk *ext_uart_clk;
	struct clk *rtc;
	struct clk *gps_ref_clk_req;
	struct clk *gps_bb_clk_req;
	struct clk *gps_rf_clk_req;
	struct clk *gps_sys_clk_req;
	struct clk *gnss_pmu_sys_clk_req;
	struct clk *gps_auto_pu_phs1;
	struct clk *gps_auto_pu_phs4;
	struct clk *gps_auto_pu_phs5;
#endif
#ifdef CONFIG_PINCTRL
#ifdef CONFIG_PINCTRL_SINGLE
	struct pinctrl *pinctrl;
	/* tcxo */
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	/* ext_lna */
	struct pinctrl_state *ext_lna_pins_default;
	struct pinctrl_state *ext_lna_pins_inactive;
	/* fta */
	struct pinctrl_state *fta_pins_default;
	struct pinctrl_state *fta_pins_inactive;
	/* fta_pos */
	struct pinctrl_state *fta_pos_pins_default;
	struct pinctrl_state *fta_pos_pins_inactive;
#else
	/* tcxo */
	const char *pinctrl_name;
	const char *tcxo_pinctrl_name;
#endif
#endif
};

struct idi_gnss_port {
	const struct uart_ops *uxp_ops;
	int state;
	struct idi_peripheral_device *p_dev;
	phys_addr_t scu_io;
	phys_addr_t gnss_io;
	unsigned wk_irq_stat;
	unsigned notify_irq_stat;
	unsigned error_irq_stat;
	unsigned scu_chip_id;
	bool idi_channel_initialized;
#ifdef GNSS_DCLK_FTR
	unsigned cgu_gps_phx_control;
#endif
	int gps_state;
	bool bb_clk_state;
	unsigned int idi_power_state;
	unsigned int freq;
	struct idi_channel_config rx_ch_config;
	struct idi_channel_config tx_ch_config;
	spinlock_t hw_lock;
	unsigned char tx_bounce[TX_BUFFER_SIZE] ____cacheline_aligned;
	struct work_struct gnss_work_tx;
	atomic_t tx_trans_in_process;
	atomic_t rx_trans_in_process;
	struct semaphore pm_sem;
#ifdef CONFIG_HAS_WAKELOCK
	/* Android PM support */
	struct wake_lock gnss_wake_lock;
#endif
};

#endif /* _IMC_IDI_GNSS_H */
