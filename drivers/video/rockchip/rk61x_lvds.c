/*
 * rockchip rk616/rk618 lvds driver
 *
 * Copyright (C) 2013-2015 Rockchip Electronics Co., Ltd.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "rk61x_lvds.h"

struct rk61x_lvds *g_lvds;

static int rk61x_lvds_cfg(struct mfd_rk61x *rk61x,
			  struct rockchip_screen *screen)
{
	struct rk61x_route *route = &rk61x->route;
	u32 val = 0;
	int ret;
	int odd = (screen->mode.left_margin & 0x01) ? 0 : 1;

	if (!route->lvds_en) { /* lvds port is not used ,power down lvds */
		val = LVDS_PLL_PWR_DN(1) | LVDS_CBG_PWR_EN(0) |
			LVDS_CH0_PWR_EN(0) | LVDS_CH1_PWR_EN(0) |
			LVDS_CH0TTL_EN(0) | LVDS_CH1TTL_EN(0);
		ret = rk61x->write_dev(rk61x, CRU_LVDS_CON0, &val);

		/* set lcd1 port for output as RGB interface */
		if (!route->lcd1_input) {
			val = LCD1_INPUT_EN(0);
			ret = rk61x->write_dev(rk61x, CRU_IO_CON0, &val);
		}
	} else {
		if (route->lvds_mode) { /* lvds mode */
			val = LVDS_CH0TTL_EN(0) | LVDS_CH1TTL_EN(0) |
				LVDS_PLL_PWR_DN(0) | LVDS_CBG_PWR_EN(1) |
				LVDS_DCLK_INV(1) | /*LVDS_MSB_SEL(1) |*/
				LVDS_OUT_FORMAT(screen->lvds_format);
			if (route->lvds_ch_nr == 2) { /* dual lvds channel */
				val |= LVDS_CH_SEL(1) | /* sel channel0 and 1 */
					LVDS_CH0_PWR_EN(1) |
					LVDS_CH1_PWR_EN(1) |
					LVDS_HBP_ODD(odd);
				ret = rk61x->write_dev(rk61x, CRU_LVDS_CON0,
						       &val);

				rk61x_dbg(rk61x->dev, "rk61x use dual lvds channel\n");
			} else { /* single lvds channel */
				/* use channel 0 */
				val |= LVDS_CH_SEL(0) |
					LVDS_CH0_PWR_EN(1) |
					LVDS_CH1_PWR_EN(0);
				ret = rk61x->write_dev(rk61x, CRU_LVDS_CON0,
						       &val);

				rk61x_dbg(rk61x->dev, "rk61x use single lvds channel\n");
			}
		} else { /* mux lvds port to RGB mode */
			val = LVDS_PLL_PWR_DN(1) | LVDS_CBG_PWR_EN(0) |
				LVDS_CH0_PWR_EN(0) | LVDS_CH1_PWR_EN(0) |
				LVDS_CH0TTL_EN(1) | LVDS_CH1TTL_EN(1);
			ret = rk61x->write_dev(rk61x, CRU_LVDS_CON0, &val);

			val = LVDS_OUT_EN(0);
			ret = rk61x->write_dev(rk61x, CRU_IO_CON0, &val);
			rk61x_dbg(rk61x->dev, "rk61x use RGB output\n");
		}
	}

	return 0;
}

/*
 * function: rk61x_scaler_set_param()
 * @screen: dst screen info
 * @enable:
 *	0: bypass; 1: scaler
 */
int rk61x_scaler_set_param(struct rockchip_screen *screen, bool enable)
{
	int ret;
	struct mfd_rk61x *rk61x = g_lvds->rk61x;

	if (!rk61x) {
		pr_err("%s:mfd rk61x is null!\n", __func__);
		return -1;
	}

	ret = rk61x_display_router_cfg(rk61x, screen, enable);
	ret = rk61x_lvds_cfg(rk61x, screen);
	return ret;
}

static int rk61x_lvds_disable(void)
{
	struct rk61x_lvds *lvds = g_lvds;
	struct mfd_rk61x *rk61x;
	u32 val = 0;
	int ret = 0;

	if (unlikely(!lvds) || !lvds->sys_state)
		return 0;

	rk61x = lvds->rk61x;
	val = LVDS_PLL_PWR_DN(1) | LVDS_CBG_PWR_EN(0) |
		LVDS_CH0_PWR_EN(0) | LVDS_CH1_PWR_EN(0);
	ret = rk61x->write_dev(rk61x, CRU_LVDS_CON0, &val);

	val = LCD1_INPUT_EN(0);
	ret = rk61x->write_dev(rk61x, CRU_IO_CON0, &val);

	lvds->sys_state = false;
	return ret;
}

static int rk61x_lvds_enable(void)
{
	struct rk61x_lvds *lvds = g_lvds;
	struct mfd_rk61x *rk61x;
	struct rockchip_screen *screen = &lvds->screen;

	if (unlikely(!lvds) || lvds->sys_state)
		return 0;

	rk61x = lvds->rk61x;
	rk61x_display_router_cfg(rk61x, screen, 0);
	rk61x_lvds_cfg(rk61x, screen);

	lvds->sys_state = true;
	return 0;
}

static int rk61x_lvds_get_property(int prop_id)
{
	return PROP_EXTER_CONNECT;
}

static struct rockchip_fb_trsm_ops trsm_lvds_ops = {
	.enable = rk61x_lvds_enable,
	.disable = rk61x_lvds_disable,
	.get_property = rk61x_lvds_get_property,
};

static int rk61x_lvds_probe(struct platform_device *pdev)
{
	struct rk61x_lvds *lvds = NULL;
	struct mfd_rk61x *rk61x = NULL;
	int ret = 0;

	lvds = devm_kzalloc(&pdev->dev, sizeof(*lvds), GFP_KERNEL);
	if (!lvds) {
		dev_err(&pdev->dev, "alloc for struct rk61x_lvds fail\n");
		return  -ENOMEM;
	}

	rk61x = dev_get_drvdata(pdev->dev.parent);
	if (!rk61x) {
		dev_err(&pdev->dev, "null mfd device rk61x!\n");
		return -ENODEV;
	}

	g_lvds = lvds;
	lvds->rk61x = rk61x;

	rockchip_get_prmry_screen(&lvds->screen);
	if ((lvds->screen.type != SCREEN_RGB) &&
	    (lvds->screen.type != SCREEN_LVDS)) {
		dev_err(&pdev->dev, "screen is not lvds/rgb!\n");
		ret = -EINVAL;
		goto err_free_mem;
	}

	lvds->screen.sscreen_set = rk61x_scaler_set_param;
	rockchip_fb_trsm_ops_register(&trsm_lvds_ops, SCREEN_LVDS);
	if (support_loader_display())
		lvds->sys_state = true;

	dev_info(&pdev->dev, "rk61x lvds probe success!\n");
	return 0;

err_free_mem:
	devm_kfree(&pdev->dev, lvds);
	lvds = NULL;
	return ret;
}

static int rk61x_lvds_remove(struct platform_device *pdev)
{
	return 0;
}

static void rk61x_lvds_shutdown(struct platform_device *pdev)
{
}

struct platform_driver rk61x_lvds_driver = {
	.driver		= {
		.name	= "rk61x-lvds",
		.owner	= THIS_MODULE,
	},
	.probe		= rk61x_lvds_probe,
	.remove		= rk61x_lvds_remove,
	.shutdown	= rk61x_lvds_shutdown,
};
