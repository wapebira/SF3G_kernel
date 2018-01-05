/*
 * Copyright (C) 2012-2015 Rockchip Electronics Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __CAT66121_HDMI_H__
#define __CAT66121_HDMI_H__
#include "../../rk_hdmi.h"

#if defined(CONFIG_HDMI_SOURCE_LCDC1)
#define HDMI_SOURCE_DEFAULT HDMI_SOURCE_LCDC1
#else
#define HDMI_SOURCE_DEFAULT HDMI_SOURCE_LCDC0
#endif

struct cat66121_hdmi_pdata {
	int gpio;
	struct i2c_client *client;
	struct delayed_work delay_work;
	struct workqueue_struct *workqueue;
	int plug_status;
};

extern struct cat66121_hdmi_pdata *cat66121_hdmi;

int cat66121_detect_device(void);
int cat66121_hdmi_sys_init(struct hdmi *hdmi_drv);
void cat66121_hdmi_interrupt(struct hdmi *hdmi_drv);
int cat66121_hdmi_sys_detect_hpd(struct hdmi *hdmi_drv);
int cat66121_hdmi_sys_insert(struct hdmi *hdmi_drv);
int cat66121_hdmi_sys_remove(struct hdmi *hdmi_drv);
int cat66121_hdmi_sys_read_edid(struct hdmi *hdmi_drv,
				int block, unsigned char *buff);
int cat66121_hdmi_sys_config_video(struct hdmi *hdmi_drv,
				   struct hdmi_video_para *vpara);
int cat66121_hdmi_sys_config_audio(struct hdmi *hdmi_drv,
				   struct hdmi_audio *audio);
void cat66121_hdmi_sys_enalbe_output(struct hdmi *hdmi_drv, int enable);
int cat66121_hdmi_register_hdcp_callbacks(void (*hdcp_cb)(void),
					  void (*hdcp_irq_cb)(int status),
					  int (*hdcp_power_on_cb)(void),
					  void (*hdcp_power_off_cb)(void));
struct rockchip_vop_driver *get_vop_drv(char *name);
void hdmi_register_display_sysfs(struct hdmi *hdmi, struct device *parent);
void hdmi_unregister_display_sysfs(struct hdmi *hdmi);
#endif
