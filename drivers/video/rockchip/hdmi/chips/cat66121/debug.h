/*
 * HDMI support
 *
 * Copyright (C) 2013 ITE Tech. Inc.
 * Author: Jau-Chih.Tseng@ite.com.tw
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _DEBUG_H_
#define _DEBUG_H_

#ifdef CONFIG_RK_HDMI_DEBUG
#define debug_message 1
#else
#define debug_message 0
#endif

/* #pragma message("debug.h") */

#if debug_message
#define hdmi_edid_error(fmt, ...) \
	printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define HDMITX_DEBUG_PRINTF(fmt, ...)  printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define HDCP_DEBUG_PRINTF(fmt, ...)  printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define EDID_DEBUG_PRINTF(fmt, ...)  printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define HDMITX_DEBUG_INFO(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
#else
    #define HDMITX_DEBUG_PRINTF(fmt, ...)  printk(pr_fmt(fmt), ##__VA_ARGS__)
    /* #define HDMITX_DEBUG_PRINTF(fmt, ...) */
    #define HDCP_DEBUG_PRINTF(fmt, ...)
    #define EDID_DEBUG_PRINTF(fmt, ...)
    #define HDMITX_DEBUG_INFO(fmt, ...)
#endif

#if (debug_message & (1 << 1))
    #define HDMITX_DEBUG_PRINTF1(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define HDCP_DEBUG_PRINTF1(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define EDID_DEBUG_PRINTF1(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
#else
    #define HDMITX_DEBUG_PRINTF1(fmt, ...)
    #define HDCP_DEBUG_PRINTF1(fmt, ...)
    #define EDID_DEBUG_PRINTF1(fmt, ...)
#endif

#if (debug_message & (1 << 2))
    #define HDMITX_DEBUG_PRINTF2(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define HDCP_DEBUG_PRINTF2(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define EDID_DEBUG_PRINTF2(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
#else
    #define HDMITX_DEBUG_PRINTF2(fmt, ...)
    #define HDCP_DEBUG_PRINTF2(fmt, ...)
    #define EDID_DEBUG_PRINTF2(fmt, ...)
#endif

#if (debug_message & (1 << 3))
    #define HDMITX_DEBUG_PRINTF3(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define HDCP_DEBUG_PRINTF3(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define EDID_DEBUG_PRINTF3(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
#else
    #define HDMITX_DEBUG_PRINTF3(fmt, ...)
    #define HDCP_DEBUG_PRINTF3(fmt, ...)
    #define EDID_DEBUG_PRINTF3(fmt, ...)
#endif

#if (debug_message & (1 << 4))
    #define HDMITX_DEBUG_PRINTF4(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define HDCP_DEBUG_PRINTF4(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define EDID_DEBUG_PRINTF4(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
#else
    #define HDMITX_DEBUG_PRINTF4(fmt, ...)
    #define HDCP_DEBUG_PRINTF4(fmt, ...)
    #define EDID_DEBUG_PRINTF4(fmt, ...)
#endif

#if (debug_message & (1 << 5))
    #define HDMITX_DEBUG_PRINTF5(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define HDCP_DEBUG_PRINTF5(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define EDID_DEBUG_PRINTF5(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
#else
    #define HDMITX_DEBUG_PRINTF5(fmt, ...)
    #define HDCP_DEBUG_PRINTF5(fmt, ...)
    #define EDID_DEBUG_PRINTF5(fmt, ...)
#endif

#if (debug_message & (1 << 6))
    #define HDMITX_DEBUG_PRINTF6(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define HDCP_DEBUG_PRINTF6(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define EDID_DEBUG_PRINTF6(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
#else
    #define HDMITX_DEBUG_PRINTF6(fmt, ...)
    #define HDCP_DEBUG_PRINTF6(fmt, ...)
    #define EDID_DEBUG_PRINTF6(fmt, ...)
#endif

#if (debug_message & (1 << 7))
    #define HDMITX_DEBUG_PRINTF7(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define HDCP_DEBUG_PRINTF7(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
    #define EDID_DEBUG_PRINTF7(fmt, ...) printk(pr_fmt(fmt), ##__VA_ARGS__)
#else
    #define HDMITX_DEBUG_PRINTF7(fmt, ...)
    #define HDCP_DEBUG_PRINTF7(fmt, ...)
    #define EDID_DEBUG_PRINTF7(fmt, ...)
#endif
#endif/* _DEBUG_H_ */
