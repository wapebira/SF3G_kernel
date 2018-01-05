/*
 * rockchip screen driver
 *
 * Copyright (C) 2014-2015 Rockchip Electronics Co., Ltd.
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
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/leds.h>
#include <linux/of_gpio.h>
#include <linux/rockchip_screen.h>
#include <linux/rockchip_fb.h>
#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include "hdmi/rk_hdmi.h"

#define NODE_DISPLAY_PANEL	"display-panel0"

#define PROP_DISPLAY_GPIORST    "intel,display-gpio-reset"
#define PROP_DISPLAY_GPIOVH     "intel,display-gpio-vhigh"
#define PROP_DISPLAY_GPIOVL     "intel,display-gpio-vlow"

#define GPIO_LIST_POWER_ON      "gpio-power-on"
#define GPIO_LIST_POWER_OFF     "gpio-power-off"

#define PROP_DISPLAY_GPIOTYPE   "intel,gpio-type"
#define PROP_DISPLAY_GPIOVALUE  "intel,gpio-value-delay"

#define COMPAT_DISPLAY_SCALER	"rockchip,display-scaler"
#define PROP_SCL_HDMI_RES	"rockchip,scl-hdmi-res"
#define PROP_SCL_PLL_CFG	"rockchip,scl-pll-cfg"
#define PROP_SCL_FRAC		"rockchip,scl-frac"
#define PROP_SCL_VST		"rockchip,scl-vst"
#define PROP_SCL_HST		"rockchip,scl-hst"
#define PROP_SCL_VIF_VST	"rockchip,scl-vif-vst"
#define PROP_SCL_VIF_HST	"rockchip,scl-vif-hst"

static struct rockchip_screen *sfa_screen;

size_t get_fb_size(void)
{
	size_t size = 0;
	u32 xres = 0;
	u32 yres = 0;

	if (unlikely(!sfa_screen))
		return 0;

	xres = sfa_screen->mode.xres;
	yres = sfa_screen->mode.yres;

	xres = ALIGN_N_TIMES(xres, 32);

	/* three buffer as default */
	size = (xres * yres << 2) * 3;
	return ALIGN(size, SZ_1M);
}

int rockchip_get_prmry_screen(struct rockchip_screen *screen)
{
	if (unlikely(!sfa_screen) || unlikely(!screen))
		return -1;

	memcpy(screen, sfa_screen, sizeof(struct rockchip_screen));
	return 0;
}

int rockchip_set_prmry_screen(struct rockchip_screen *screen)
{
	if (unlikely(!sfa_screen) || unlikely(!screen))
		return -1;

	sfa_screen->vop_id = screen->vop_id;
	sfa_screen->screen_id = screen->screen_id;

	if (!sfa_screen->sscreen_set)
		sfa_screen->sscreen_set = screen->sscreen_set;
	return 0;
}

static void rockchip_screen_set_gpiolist(struct rockchip_screen *screen,
					 struct display_pwr_gpio *gpios)
{
	struct display_pwr_gpio *gpio;

	list_for_each_entry(gpio, &gpios->list, list) {
		switch (gpio->type) {
		case DISPLAY_GPIO_VHIGH:
			if (!screen->gpio_vhigh)
				break;

			gpio_direction_output(screen->gpio_vhigh, gpio->value);
			break;

		case DISPLAY_GPIO_VLOW:
			if (!screen->gpio_vlow)
				break;

			gpio_direction_output(screen->gpio_vlow, gpio->value);
			break;

		case DISPLAY_GPIO_RESET:
			if (!screen->gpio_reset)
				break;

			gpio_direction_output(screen->gpio_reset, gpio->value);
			break;
		}

		if (gpio->delay)
			mdelay(gpio->delay);
	}
}

static void rockchip_screen_power_on(struct rockchip_screen *screen)
{
	if (screen->gpios_power_on)
		rockchip_screen_set_gpiolist(screen, screen->gpios_power_on);
}

static void rockchip_screen_power_off(struct rockchip_screen *screen)
{
	if (screen->gpios_power_off)
		rockchip_screen_set_gpiolist(screen, screen->gpios_power_off);
}

/*
 * display power control parse from dts
 */

static int rockchip_screen_parse_display_gpio(struct device_node *n,
					      struct display_pwr_gpio *gpio)
{
	const char *string;
	int array[2];
	int ret;

	gpio->name = n->name;
	ret = of_property_read_string(n, PROP_DISPLAY_GPIOTYPE, &string);
	if (ret) {
		pr_err("%s: Get %s failed\n", __func__, PROP_DISPLAY_GPIOTYPE);
		return ret;
	} else if (!strcmp("vhigh", string)) {
		gpio->type = DISPLAY_GPIO_VHIGH;
	} else if (!strcmp("vlow", string)) {
		gpio->type = DISPLAY_GPIO_VLOW;
	} else if (!strcmp("reset", string)) {
		gpio->type = DISPLAY_GPIO_RESET;
	}

	ret = of_property_read_u32_array(n, PROP_DISPLAY_GPIOVALUE, array, 2);
	if (ret) {
		pr_err("%s: Get %s failed\n", __func__, PROP_DISPLAY_GPIOVALUE);
		return ret;
	}

	gpio->value = array[0];
	gpio->delay = array[1];

	return 0;
}

static int
rockchip_screen_parse_display_gpiolist(struct rockchip_screen *screen,
				       struct device_node *n,
				       struct display_pwr_gpio **gpiolist)
{
	struct device_node *child;
	struct display_pwr_gpio *gpio;

	*gpiolist = devm_kzalloc(screen->dev, sizeof(struct display_pwr_gpio),
				 GFP_KERNEL);
	if (!*gpiolist) {
		pr_err("%s: Can't alloc gpio table\n", __func__);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&(*gpiolist)->list);
	for_each_child_of_node(n, child) {
		gpio = devm_kzalloc(screen->dev, sizeof(*gpio), GFP_KERNEL);
		if (!gpio) {
			pr_err("%s: Allocation of display gpio failed\n",
			       __func__);
			return -EINVAL;
		}

		if (!rockchip_screen_parse_display_gpio(child, gpio))
			list_add_tail(&gpio->list, &(*gpiolist)->list);
		else
			devm_kfree(screen->dev, gpio);
	}

	return 0;
}

static int rockchip_screen_parse_gpio(struct rockchip_screen *screen)
{
	struct device_node *np = screen->dev->of_node;
	enum of_gpio_flags gpio_flags;
	unsigned long flags;
	int ret;

	if (!np) {
		pr_err("%s: Can't find screen matching node\n", __func__);
		return -EINVAL;
	}

	screen->gpio_vhigh = of_get_named_gpio_flags(np,
			PROP_DISPLAY_GPIOVH, 0, &gpio_flags);
	if (gpio_is_valid(screen->gpio_vhigh)) {
		if (support_loader_display()) {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_LOW;
			else
				flags = GPIOF_OUT_INIT_HIGH;
		} else {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_HIGH;
			else
				flags = GPIOF_OUT_INIT_LOW;
		}
		ret = gpio_request_one(screen->gpio_vhigh, flags, "disp_vhigh");
		if (ret) {
			pr_err("%s: request display high power gpio fail: %d\n",
			       __func__, ret);
			screen->gpio_vhigh = 0;
		}
	} else {
		screen->gpio_vhigh = 0;
	}

	screen->gpio_vlow = of_get_named_gpio_flags(np,
			PROP_DISPLAY_GPIOVL, 0, &gpio_flags);
	if (gpio_is_valid(screen->gpio_vlow)) {
		if (support_loader_display()) {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_LOW;
			else
				flags = GPIOF_OUT_INIT_HIGH;
		} else {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_HIGH;
			else
				flags = GPIOF_OUT_INIT_LOW;
		}
		ret = gpio_request_one(screen->gpio_vlow, flags, "disp_vlow");
		if (ret) {
			pr_err("%s: request display low power gpio fail: %d\n",
			       __func__, ret);
			screen->gpio_vlow = 0;
		}
	} else {
		screen->gpio_vlow = 0;
	}

	screen->gpio_reset = of_get_named_gpio_flags(np,
			PROP_DISPLAY_GPIORST, 0, &gpio_flags);
	if (gpio_is_valid(screen->gpio_reset)) {
		if (support_loader_display()) {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_LOW;
			else
				flags = GPIOF_OUT_INIT_HIGH;
		} else {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_HIGH;
			else
				flags = GPIOF_OUT_INIT_LOW;
		}
		ret = gpio_request_one(screen->gpio_reset, flags, "disp_rst");
		if (ret) {
			pr_err("%s: request display reset gpio fail: %d\n",
			       __func__, ret);
			screen->gpio_reset = 0;
		}
	} else {
		screen->gpio_reset = 0;
	}

	return 0;
}

#ifdef CONFIG_MFD_RK61X
static int rockchip_screen_parse_scaler(struct device_node *np,
					struct display_scaler_param *scaler)
{
	const char *string;
	u32 val;
	int ret;

	ret = of_property_read_string(np, PROP_SCL_HDMI_RES, &string);
	if (ret) {
		pr_err("%s: Get %s failed\n", __func__, PROP_SCL_HDMI_RES);
		return ret;
	}
	scaler->hdmi_vic = hdmi_get_vic_by_res_name(string);

	if (!of_property_read_u32(np, PROP_SCL_PLL_CFG, &val))
		scaler->pll_cfg_val = val;

	if (!of_property_read_u32(np, PROP_SCL_FRAC, &val))
		scaler->frac = val;

	if (!of_property_read_u32(np, PROP_SCL_VST, &val))
		scaler->scl_vst = val;

	if (!of_property_read_u32(np, PROP_SCL_HST, &val))
		scaler->scl_hst = val;

	if (!of_property_read_u32(np, PROP_SCL_VIF_VST, &val))
		scaler->vif_vst = val;

	if (!of_property_read_u32(np, PROP_SCL_VIF_HST, &val))
		scaler->vif_hst = val;

	return 0;
}

static int rockchip_screen_parse_scaler_list(struct device_node *np,
					     struct rockchip_screen *screen)
{
	struct device_node *child;
	struct device_node *scl_node;
	struct display_scaler_param *scl_para;

	scl_node = of_find_compatible_node(np, NULL, COMPAT_DISPLAY_SCALER);
	if (!scl_node) {
		pr_debug("%s: Can't find display-scaler matching node\n",
			 __func__);
		return -EINVAL;
	}

	INIT_LIST_HEAD(&screen->scaler_list_head);
	for_each_child_of_node(scl_node, child) {
		scl_para = devm_kzalloc(screen->dev, sizeof(*scl_para),
					GFP_KERNEL);
		if (!scl_para) {
			pr_err("%s: Alloc display scaler param failed\n",
			       __func__);
			return -ENOMEM;
		}

		if (!rockchip_screen_parse_scaler(child, scl_para))
			list_add_tail(&scl_para->list,
				      &screen->scaler_list_head);
		else
			devm_kfree(screen->dev, scl_para);
	}
	return 0;
}

static int rockchip_screen_get_scaler_param(struct rockchip_screen *screen,
					    u8 hdmi_resolution)
{
	struct display_scaler_param *scaler = NULL;

	if (list_empty(&screen->scaler_list_head))
		return 0;

	list_for_each_entry(scaler, &screen->scaler_list_head, list) {
		if (scaler->hdmi_vic == hdmi_resolution)
			break;
	}

	if (scaler) {
		screen->pll_cfg_val = scaler->pll_cfg_val;
		screen->frac = scaler->frac;
		screen->scl_vst = scaler->scl_vst;
		screen->scl_hst = scaler->scl_hst;
		screen->vif_vst = scaler->vif_vst;
		screen->vif_hst = scaler->vif_hst;
	} else {
		pr_err("%s: Can't find scaler parameter for hdmi vic=%d\n",
		       __func__, hdmi_resolution);
	}

	return 0;
}
#else
#define rockchip_screen_get_scaler_param	NULL
#endif

static struct of_device_id backlight_dt_match[] = {
	{	.compatible = "pwm-backlight",
		.data = (void *)PWM_BACKLIGHT,
	},
	{	.compatible = "intel,agold620-led",
		.data = (void *)LED_BACKLIGHT,
	},
	{},
};

static struct display_backlight *
rockchip_screen_get_backlight(struct rockchip_screen *screen)
{
	struct device_node *backlight_node;
	struct display_backlight *backlight = NULL;

	backlight_node =
		of_parse_phandle(screen->dev->of_node, "backlight", 0);

	if (backlight_node) {
		const struct of_device_id *match;

		match = of_match_node(backlight_dt_match, backlight_node);
		backlight = devm_kzalloc(screen->dev, sizeof(*backlight),
					 GFP_KERNEL);
		if (!backlight)
			return 0;

		backlight->type = (int)match->data;

		if (backlight->type == PWM_BACKLIGHT)
			backlight->u.pwm_bl =
				of_find_backlight_by_node(backlight_node);
		else if (backlight->type == LED_BACKLIGHT)
			backlight->u.led_bl =
				of_find_led_classdev_by_node(backlight_node);

		of_node_put(backlight_node);

		if (!backlight->u.pwm_bl && !backlight->u.led_bl) {
			devm_kfree(screen->dev, backlight);
			backlight = NULL;
			dev_info(screen->dev, "No find screen backlight device\n");
		}
	} else {
		dev_info(screen->dev, "No find screen backlight device node\n");
	}

	return backlight;
}

static int
rockchip_screen_backlight_control(struct rockchip_screen *screen, bool enable)
{
	struct backlight_device *pwm_bl;
	struct led_classdev *led_bl;

	if (!screen->backlight) {
		screen->backlight = rockchip_screen_get_backlight(screen);
		if (unlikely(!screen->backlight))
			return -ENODEV;
	}

	if (screen->backlight->type == PWM_BACKLIGHT) {
		pwm_bl = screen->backlight->u.pwm_bl;
		pwm_bl->props.fb_blank =
			enable ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN;

		backlight_update_status(screen->backlight->u.pwm_bl);
	} else if (screen->backlight->type == LED_BACKLIGHT) {
		led_bl = screen->backlight->u.led_bl;

		if (enable)
			led_classdev_resume(led_bl);
		else
			led_classdev_suspend(led_bl);
	}

	return 0;
}

static int rockchip_disp_pwr_ctr_parse_dt(struct device_node *np,
					  struct rockchip_screen *screen)
{
	int ret = 0;
	struct device_node *panel_np;
	struct device_node *child;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	screen->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(screen->pm_platdata)) {
		dev_err(screen->dev, "Error during device state pm init.\n");
		screen->pm_platdata = NULL;
		return -EINVAL;
	}
#endif

	if (screen->type != SCREEN_MIPI) {
		rockchip_screen_parse_gpio(screen);

		panel_np  = of_get_child_by_name(np, NODE_DISPLAY_PANEL);
		if (!panel_np) {
			pr_err("%s: Can't find display-panel0 matching node\n",
			       __func__);
			return -EINVAL;
		}

		for_each_child_of_node(panel_np, child) {
			if (!strcmp(child->name, GPIO_LIST_POWER_ON)) {
				ret = rockchip_screen_parse_display_gpiolist(
						screen, child,
						&screen->gpios_power_on);
			} else if (!strcmp(child->name, GPIO_LIST_POWER_OFF)) {
				ret = rockchip_screen_parse_display_gpiolist(
						screen, child,
						&screen->gpios_power_off);
			}

			if (ret)
				pr_info("%s: Node %s parsing failed %d\n",
					__func__, child->name, ret);
		}
	}

	return 0;
}

int rockchip_disp_pwr_enable(struct rockchip_screen *screen)
{
	int ret = 0;

	if (unlikely(!screen))
		return -ENODEV;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (screen->pm_platdata) {
		ret = device_state_pm_set_state_by_name(
			screen->dev, screen->pm_platdata->pm_state_D0_name);
		if (ret < 0) {
			dev_err(screen->dev, "Error while setting the pm class\n");
			return ret;
		}
	}
#endif

	if (!support_loader_display() && screen->type != SCREEN_MIPI) {
		if (screen->power_on)
			screen->power_on(screen);
	}

	if (screen->dev->pins && screen->dev->pins->default_state)
		pinctrl_select_state(screen->dev->pins->p,
				     screen->dev->pins->default_state);

	return 0;
}

int rockchip_disp_pwr_disable(struct rockchip_screen *screen)
{
	int ret = 0;

	if (unlikely(!screen))
		return -ENODEV;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (screen->pm_platdata) {
		ret = device_state_pm_set_state_by_name(
			screen->dev, screen->pm_platdata->pm_state_D3_name);

		if (ret < 0) {
			dev_err(screen->dev, "could not set PM state: %s\n",
				screen->pm_platdata->pm_state_D3_name);
			return ret;
		}
	}
#endif

	if (screen->type != SCREEN_MIPI) {
		if (screen->power_off)
			screen->power_off(screen);
	}

	if (screen->dev->pins && screen->dev->pins->sleep_state)
		pinctrl_select_state(screen->dev->pins->p,
				     screen->dev->pins->sleep_state);

	return 0;
}

static int rockchip_fb_videomode_from_timing(const struct display_timing *dt,
					  struct rockchip_screen *screen)
{
	screen->mode.pixclock = dt->pixelclock.typ;
	screen->mode.left_margin = dt->hback_porch.typ;
	screen->mode.right_margin = dt->hfront_porch.typ;
	screen->mode.xres = dt->hactive.typ;
	screen->mode.hsync_len = dt->hsync_len.typ;
	screen->mode.upper_margin = dt->vback_porch.typ;
	screen->mode.lower_margin = dt->vfront_porch.typ;
	screen->mode.yres = dt->vactive.typ;
	screen->mode.vsync_len = dt->vsync_len.typ;
	screen->type = dt->screen_type;
	screen->lvds_format = dt->lvds_format;
	screen->face = dt->face;
	screen->color_mode = dt->color_mode;
	screen->dsp_lut = dt->dsp_lut;
	screen->width = dt->width;
	screen->height = dt->height;

	if (dt->flags & DISPLAY_FLAGS_PIXDATA_POSEDGE)
		screen->pin_dclk = 1;
	else
		screen->pin_dclk = 0;
	if (dt->flags & DISPLAY_FLAGS_HSYNC_HIGH)
		screen->pin_hsync = 1;
	else
		screen->pin_hsync = 0;
	if (dt->flags & DISPLAY_FLAGS_VSYNC_HIGH)
		screen->pin_vsync = 1;
	else
		screen->pin_vsync = 0;
	if (dt->flags & DISPLAY_FLAGS_DE_HIGH)
		screen->pin_den = 1;
	else
		screen->pin_den = 0;

	return 0;
}

static int rockchip_prase_timing_dt(struct device_node *np,
				 struct rockchip_screen *screen)
{
	struct display_timings *disp_timing;
	struct display_timing *dt;

	disp_timing = of_get_display_timings(np);
	if (!disp_timing) {
		pr_err("parse display timing err\n");
		return -EINVAL;
	}
	dt = display_timings_get(disp_timing, disp_timing->native_mode);
	if (dt)
		rockchip_fb_videomode_from_timing(dt, screen);
	return 0;
}

static int rockchip_screen_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "Missing device tree node.\n");
		return -EINVAL;
	}
	sfa_screen = devm_kzalloc(&pdev->dev,
				  sizeof(struct rockchip_screen), GFP_KERNEL);
	if (!sfa_screen) {
		dev_err(&pdev->dev, "kmalloc for rockchip screen fail!\n");
		return -ENOMEM;
	}

	sfa_screen->dev =  &pdev->dev;
	ret = rockchip_prase_timing_dt(np, sfa_screen);
	rockchip_disp_pwr_ctr_parse_dt(np, sfa_screen);

#ifdef CONFIG_MFD_RK61X
	rockchip_screen_parse_scaler_list(np, sfa_screen);
#endif

	sfa_screen->sscreen_get = rockchip_screen_get_scaler_param;
	sfa_screen->power_on = rockchip_screen_power_on;
	sfa_screen->power_off = rockchip_screen_power_off;
	sfa_screen->backlight_ctrl = rockchip_screen_backlight_control;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (sfa_screen->pm_platdata) {
		ret = device_state_pm_set_class(&pdev->dev,
				sfa_screen->pm_platdata->pm_user_name);
		if (ret < 0) {
			dev_err(&pdev->dev, "ERROR while LVDS initialize its PM state!\n");
			kfree(sfa_screen->pm_platdata);
			sfa_screen->pm_platdata = NULL;
		}
	}
#endif

	dev_info(&pdev->dev, "rockchip screen probe %s\n",
		 ret ? "failed" : "success");
	return ret;
}

static const struct of_device_id rockchip_screen_dt_ids[] = {
	{.compatible = "rockchip,screen",},
	{}
};

struct platform_driver rockchip_screen_driver = {
	.probe = rockchip_screen_probe,
	.driver = {
		   .name = "rockchip-screen",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(rockchip_screen_dt_ids),
		   },
};
