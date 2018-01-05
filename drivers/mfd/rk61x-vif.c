/*
 * video interface driver of rockchip rk616/rk618
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
#include <linux/slab.h>
#include <linux/mfd/rk61x.h>

#define CONFIG_ONE_LCDC_DUAL_OUTPUT_INF 1
#define GET_VIF_SUSPEND_STATE(x, id) ((x >> id) & 1)

/* rk61x video interface config */

int rk61x_vif_disable(struct mfd_rk61x *rk61x, int id)
{
	u32 val = 0;
	int ret = 0;

	if (GET_VIF_SUSPEND_STATE(rk61x->vif_suspend, id)) {
		rk61x_dbg(rk61x->dev, "%s: vif%d suspend_state=%d\n",
			  __func__, id, rk61x->vif_suspend);
		return 0;
	}

	if (id == 0) {	/* video interface 0 */
		val = VIF_EN(0);	/* disable vif0 */
		ret = rk61x->write_dev(rk61x, VIF0_REG0, &val);
	} else {		/* video interface 1 */
		val = VIF_EN(0);	/* disable vif1 */
		ret = rk61x->write_dev(rk61x, VIF1_REG0, &val);
	}

	msleep(21);

	if (id == 0) {	/* video interface 0 */
		val = VIF0_CLK_GATE(1); /* gating vif0 */
		ret = rk61x->write_dev(rk61x, CRU_CLKSEL2_CON, &val);
	} else {		/* video interface 1 */
		val = VIF1_CLK_GATE(1); /* gating vif1 */
		ret = rk61x->write_dev(rk61x, CRU_CLKSEL2_CON, &val);
	}

	rk61x->vif_suspend |= (1 << id);
	rk61x_dbg(rk61x->dev, "rk61x vif%d disable\n", id);
	return 0;
}

int rk61x_vif_enable(struct mfd_rk61x *rk61x, int id)
{
	u32 val = 0;
	u32 offset = 0;
	int ret;

	if (!GET_VIF_SUSPEND_STATE(rk61x->vif_suspend, id)) {
		rk61x_dbg(rk61x->dev, "%s: vif%d suspend_state=%d\n",
			  __func__, id, rk61x->vif_suspend);
		return 0;
	}

	if (id == 0) {
		val = VIF0_CLK_BYPASS(0) | VIF0_CLK_GATE(0);
		offset = 0;
	} else {
		val = VIF1_CLK_BYPASS(0) | VIF1_CLK_GATE(0);
		offset = 0x18;
	}

	ret = rk61x->write_dev(rk61x, CRU_CLKSEL2_CON, &val);

	val = 0;
	/* disable ddr mode,enable VIF */
	val |= VIF_DDR_CLK_EN(0) | VIF_DDR_PHASEN_EN(0) |
		VIF_DDR_MODE_EN(0) | VIF_EN(1);
	ret = rk61x->write_dev(rk61x, VIF0_REG0 + offset, &val);

	rk61x->vif_suspend &= ~(1 << id);
	rk61x_dbg(rk61x->dev, "rk61x vif%d enable\n", id);

	return 0;
}

static __maybe_unused int rk61x_vif_bypass(struct mfd_rk61x *rk61x, int id)
{
	u32 val = 0;
	int ret;

	if (id == 0)
		val = VIF0_CLK_BYPASS(1);
	else
		val = VIF1_CLK_BYPASS(1);

	ret = rk61x->write_dev(rk61x, CRU_CLKSEL2_CON, &val);

	rk61x_dbg(rk61x->dev, "rk61x vif%d bypass\n", id);
	return 0;
}

int rk61x_vif_cfg(struct mfd_rk61x *rk61x,
		  struct rockchip_screen *screen, int id)
{
	int ret = 0;
	u32 val = 0;
	int offset = 0;
	int pll_id;
	bool pll_use_mclk12m = false;

	if (id == 0) { /* video interface 0 */
		if (!rk61x->route.vif0_en) {
			rk61x_vif_disable(rk61x, id);
			return 0;
		}
		offset = 0;
		pll_id = rk61x->route.vif0_clk_sel;
	} else { /* video interface 1 */
		if (!rk61x->route.vif1_en) {
			rk61x_vif_disable(rk61x, id);
			return 0;
		}
		offset = 0x18;
		pll_id = rk61x->route.vif1_clk_sel;
	}

	pll_use_mclk12m = (pll_id == MCLK_12M) ? true : false;
	if (pll_use_mclk12m)
		rk61x_mclk_set_rate(rk61x->mclk, 12000000);

	if (!screen) {
		dev_err(rk61x->dev, "%s:screen is null.........\n", __func__);
		return -EINVAL;
	}

	rk61x_vif_disable(rk61x, id);
	if ((screen->mode.xres == 1920) && (screen->mode.yres == 1080)) {
		if (pll_use_mclk12m)
			rk61x_pll_set_rate(rk61x, pll_id, 0x028853de, 0);
		else
			rk61x_pll_set_rate(rk61x, pll_id, 0x02bf5276, 0);

		val = VIF_FRAME_HST(0xc1) | VIF_FRAME_VST(0x01);
	} else if ((screen->mode.xres == 1280) && (screen->mode.yres == 720)) {
		if (pll_use_mclk12m)
			rk61x_pll_set_rate(rk61x, pll_id, 0x0288418c, 0);
		else
			rk61x_pll_set_rate(rk61x, pll_id, 0x1422014, 0);

		val = VIF_FRAME_HST(0xc1) | VIF_FRAME_VST(0x01);
	} else if (screen->mode.xres == 720) {
		if (pll_use_mclk12m)
			rk61x_pll_set_rate(rk61x, pll_id, 0x0306510e, 0);
		else
			rk61x_pll_set_rate(rk61x, pll_id, 0x1c13015, 0);

		val = VIF_FRAME_HST(0x1) | VIF_FRAME_VST(0x01);
	}

	ret = rk61x->write_dev(rk61x, VIF0_REG1 + offset, &val);

	val = VIF_HS_END(screen->mode.hsync_len) |
	      VIF_HTOTAL(screen->mode.hsync_len + screen->mode.left_margin +
			 screen->mode.right_margin + screen->mode.xres);
	ret = rk61x->write_dev(rk61x, VIF0_REG2 + offset, &val);

	val = VIF_HACT_END(screen->mode.hsync_len + screen->mode.left_margin +
			   screen->mode.xres) |
	      VIF_HACT_ST(screen->mode.hsync_len + screen->mode.left_margin);
	ret = rk61x->write_dev(rk61x, VIF0_REG3 + offset, &val);

	val = VIF_VS_END(screen->mode.vsync_len) |
	      VIF_VTOTAL(screen->mode.vsync_len + screen->mode.upper_margin +
			 screen->mode.lower_margin + screen->mode.yres);
	ret = rk61x->write_dev(rk61x, VIF0_REG4 + offset, &val);

	val = VIF_VACT_END(screen->mode.vsync_len + screen->mode.upper_margin +
			   screen->mode.yres) |
	      VIF_VACT_ST(screen->mode.vsync_len + screen->mode.upper_margin);
	ret = rk61x->write_dev(rk61x, VIF0_REG5 + offset, &val);

	if (id == 0) {
		val = VIF0_SYNC_EN(1);
		rk61x->write_dev(rk61x, CRU_IO_CON0, &val);
	} else {
		val = VIF1_SYNC_EN(1);
		rk61x->write_dev(rk61x, CRU_IO_CON0, &val);
	}
	rk61x_vif_enable(rk61x, id);

	return ret;
}

static int rk61x_scaler_disable(struct mfd_rk61x *rk61x)
{
	u32 val = 0;
	int ret;

	val |= SCL_EN(0);	/* disable scaler */
	ret = rk61x->write_dev(rk61x, SCL_REG0, &val);
	rk61x_dbg(rk61x->dev, "rk61x scaler disable\n");
	return 0;
}

int rk61x_scaler_cfg(struct mfd_rk61x *rk61x, struct rockchip_screen *screen)
{
	u32 scl_hor_mode, scl_ver_mode;
	u32 scl_v_factor, scl_h_factor;
	u32 scl_reg_value;
	u32 dst_frame_hst, dst_frame_vst;
	u32 dst_vact_st;
	u32 dsp_htotal, dsp_hs_end, dsp_hact_st, dsp_hact_end;
	u32 dsp_vtotal, dsp_vs_end, dsp_vact_st, dsp_vact_end;
	u32 dsp_hbor_end, dsp_hbor_st, dsp_vbor_end, dsp_vbor_st;
	u32 src_w, src_h, src_htotal, dst_w, dst_h, src_vact_st;
	u16 bor_right = 0;
	u16 bor_left = 0;
	u16 bor_up = 0;
	u16 bor_down = 0;
	u8 hor_down_mode = 0;	/* 1:average; 0:bilinear */
	u8 ver_down_mode = 0;
	u8 bic_coe_sel = 2;
	struct rockchip_screen *src;
	struct rockchip_screen *dst;
	int pll_id;
	struct rk61x_route *route = &rk61x->route;

	if (!route->scl_en) {
		rk61x_scaler_disable(rk61x);
		return 0;
	}

	dst = screen;
	if (!dst) {
		dev_err(rk61x->dev, "%s:screen is null!\n", __func__);
		return -EINVAL;
	}

	if (route->scl_bypass) {
		src = dst;
		dst->pll_cfg_val = 0x01422014;
		dst->frac = 0;
	} else {
		src = screen->ext_screen;
	}

	if (route->sclk_sel == SCLK_SEL_PLL0)
		pll_id = 0;
	else
		pll_id = 1;

	rk61x_scaler_disable(rk61x);
	rk61x_pll_set_rate(rk61x, pll_id, dst->pll_cfg_val, dst->frac);
	dst_frame_vst = dst->scl_vst;
	dst_frame_hst = dst->scl_hst;

	src_htotal = src->mode.hsync_len + src->mode.left_margin +
			src->mode.xres + src->mode.right_margin;
	src_vact_st = src->mode.vsync_len + src->mode.upper_margin;
	dst_vact_st = dst->mode.vsync_len + dst->mode.upper_margin;

	dsp_htotal = dst->mode.hsync_len + dst->mode.left_margin +
			dst->mode.xres + dst->mode.right_margin;
	dsp_hs_end = dst->mode.hsync_len;

	dsp_vtotal = dst->mode.vsync_len + dst->mode.upper_margin +
			dst->mode.yres + dst->mode.lower_margin;
	dsp_vs_end = dst->mode.vsync_len;

	dsp_hbor_end =
		dst->mode.hsync_len + dst->mode.left_margin + dst->mode.xres;
	dsp_hbor_st = dst->mode.hsync_len + dst->mode.left_margin;
	dsp_vbor_end =
		dst->mode.vsync_len + dst->mode.upper_margin + dst->mode.yres;
	dsp_vbor_st = dst_vact_st;

	dsp_hact_st = dsp_hbor_st + bor_left;
	dsp_hact_end = dsp_hbor_end - bor_right;
	dsp_vact_st = dsp_vbor_st + bor_up;
	dsp_vact_end = dsp_vbor_end - bor_down;

	src_w = src->mode.xres;
	src_h = src->mode.yres;
	dst_w = dsp_hact_end - dsp_hact_st;
	dst_h = dsp_vact_end - dsp_vact_st;

	if (src_w > dst_w) { /* hor scale mode: 0:no_scl 1:scl_up 2:scl_down */
		scl_hor_mode = 0x2; /* scl_down */
		if (hor_down_mode == 0) { /* bilinear */
			if ((src_w - 1) / (dst_w - 1) > 2)
				scl_h_factor =
					((src_w - 1) << 14) / (dst_w - 1);
			else
				scl_h_factor =
					((src_w - 2) << 14) / (dst_w - 1);
		} else { /* average */
			scl_h_factor = (dst_w << 16) / (src_w - 1);
		}
	} else if (src_w == dst_w) {
		scl_hor_mode = 0x0; /* no_Scl */
		scl_h_factor = 0x0;
	} else {
		scl_hor_mode = 0x1; /* scl_up */
		scl_h_factor = ((src_w - 1) << 16) / (dst_w - 1);
	}

	/* ver scale mode: 0: no_scl 1: scl_up 2: scl_down */
	if (src_h > dst_h) {
		scl_ver_mode = 0x2; /* scl_down */
		/* bilinearhor_down_mode,u8 ver_down_mode */
		if (ver_down_mode == 0) {
			if ((src_h - 1) / (dst_h - 1) > 2)
				scl_v_factor =
					((src_h - 1) << 14) / (dst_h - 1);
			else
				scl_v_factor =
					((src_h - 2) << 14) / (dst_h - 1);
		} else {
			scl_v_factor = (dst_h << 16) / (src_h - 1);
		}
	} else if (src_h == dst_h) {
		scl_ver_mode = 0x0; /* no_Scl */
		scl_v_factor = 0x0;
	} else {
		scl_ver_mode = 0x1; /* scl_up */
		scl_v_factor = ((src_h - 1) << 16) / (dst_h - 1);
	}

	/* factor register1 */
	scl_reg_value = SCL_V_FACTOR(scl_v_factor) | SCL_H_FACTOR(scl_h_factor);
	rk61x->write_dev(rk61x, SCL_REG1, &scl_reg_value);
	/* dsp_frame register2 */
	scl_reg_value = DSP_FRAME_VST(dst_frame_vst) |
			DSP_FRAME_HST(dst_frame_hst);
	rk61x->write_dev(rk61x, SCL_REG2, &scl_reg_value);
	/* dsp_h register3 */
	scl_reg_value = DSP_HS_END(dsp_hs_end) | DSP_HTOTAL(dsp_htotal);
	rk61x->write_dev(rk61x, SCL_REG3, &scl_reg_value);
	/* dsp_hact register4 */
	scl_reg_value = DSP_HACT_END(dsp_hact_end) | DSP_HACT_ST(dsp_hact_st);
	rk61x->write_dev(rk61x, SCL_REG4, &scl_reg_value);
	/* dsp_v register5 */
	scl_reg_value = DSP_VS_END(dsp_vs_end) | DSP_VTOTAL(dsp_vtotal);
	rk61x->write_dev(rk61x, SCL_REG5, &scl_reg_value);
	/* dsp_vact register6 */
	scl_reg_value = DSP_VACT_END(dsp_vact_end) | DSP_VACT_ST(dsp_vact_st);
	rk61x->write_dev(rk61x, SCL_REG6, &scl_reg_value);
	/* hbor register7 */
	scl_reg_value = DSP_HBOR_END(dsp_hbor_end) | DSP_HBOR_ST(dsp_hbor_st);
	rk61x->write_dev(rk61x, SCL_REG7, &scl_reg_value);
	/* vbor register8 */
	scl_reg_value = DSP_VBOR_END(dsp_vbor_end) | DSP_VBOR_ST(dsp_vbor_st);
	rk61x->write_dev(rk61x, SCL_REG8, &scl_reg_value);
	/* control register0 */
	scl_reg_value = SCL_EN(1) | SCL_HOR_MODE(scl_hor_mode) |
			SCL_VER_MODE(scl_ver_mode) |
			SCL_BIC_COE_SEL(bic_coe_sel) |
			SCL_HOR_DOWN_MODE(hor_down_mode) |
			SCL_VER_DOWN_MODE(ver_down_mode);
	rk61x->write_dev(rk61x, SCL_REG0, &scl_reg_value);

	rk61x_dbg(rk61x->dev, "rk61x scaler enable\n");

	return 0;
}

static int rk61x_dual_input_cfg(struct mfd_rk61x *rk61x,
				struct rockchip_screen *screen,
				bool enable)
{
	struct rk61x_platform_data *pdata = rk61x->pdata;
	struct rk61x_route *route = &rk61x->route;

	route->vif0_bypass = 1;
	route->vif0_en = 0;
	route->vif0_clk_sel = VIF_CLKIN_SEL_PLL0;
	route->pll0_clk_sel = LCD0_DCLK;

	if (pdata->pll_clk_sel == MCLK_12M)
		route->pll1_clk_sel = MCLK_12M;
	else
		route->pll1_clk_sel = LCD1_DCLK;

	route->vif1_clk_sel = VIF_CLKIN_SEL_PLL1;
	route->hdmi_sel = HDMI_IN_SEL_VIF1;
	route->hdmi_clk_sel = HDMI_CLK_SEL_VIF1;
	if (enable) { /* hdmi plug in */
		route->vif1_bypass = 0;
		route->vif1_en = 1;
	} else { /* hdmi plug out */
		route->vif1_bypass = 1;
		route->vif1_en = 0;
	}

	route->sclin_sel = SCL_SEL_VIF0; /* from vif0 */
	route->scl_en = 0; /* dual lcdc, scaler not needed */
	route->dither_sel = DITHER_SEL_VIF0;
	route->lcd1_input = 1;

	if (screen->type == SCREEN_RGB) {
		route->lvds_en = 1;
		route->lvds_mode = RGB; /* rgb output */
	} else if (screen->type == SCREEN_LVDS) {
		route->lvds_en = 1;
		route->lvds_mode = LVDS;
		route->lvds_ch_nr = pdata->lvds_ch_nr;
	} else if (screen->type == SCREEN_MIPI) {
		route->lvds_en = 0;
	}

	return 0;
}

static int rk61x_lcd0_input_lcd1_unused_cfg(struct mfd_rk61x *rk61x,
					    struct rockchip_screen *screen,
					    bool enable)
{
	struct rk61x_platform_data *pdata = rk61x->pdata;
	struct rk61x_route *route = &rk61x->route;

	if (enable) { /* hdmi plug in */
		route->vif0_bypass = 0;
		route->vif0_en = 1;
		route->vif0_clk_sel = VIF_CLKIN_SEL_PLL0;
		route->sclin_sel = SCL_SEL_VIF0;
		route->scl_en = 1;
		route->sclk_sel = SCLK_SEL_PLL1;
		route->dither_sel = DITHER_SEL_SCL;
		route->hdmi_sel = HDMI_IN_SEL_VIF0;
		route->hdmi_clk_sel = HDMI_CLK_SEL_VIF0;
	} else {
		route->vif0_bypass = 1;
		route->vif0_en = 0;
		route->sclin_sel = SCL_SEL_VIF0;
		route->scl_en = 0;
		route->dither_sel = DITHER_SEL_VIF0;
		route->hdmi_sel = HDMI_IN_SEL_VIF0;
	}
	route->pll1_clk_sel = LCD0_DCLK;

	if (pdata->pll_clk_sel == MCLK_12M)
		route->pll0_clk_sel = MCLK_12M;
	else
		route->pll0_clk_sel = LCD0_DCLK;

	route->vif1_bypass = 1;
	route->vif1_en = 0;
	route->lcd1_input  = 0;

	if (screen->type == SCREEN_RGB) {
		route->lvds_en = 1;
		route->lvds_mode = RGB;
	} else if (screen->type == SCREEN_LVDS) {
		route->lvds_en = 1;
		route->lvds_mode = LVDS;
		route->lvds_ch_nr = pdata->lvds_ch_nr;
	} else if (screen->type == SCREEN_MIPI) {
		route->lvds_en = 0;
	}

	return 0;
}

static int rk61x_lcd0_input_lcd1_output_cfg(struct mfd_rk61x *rk61x,
					    struct rockchip_screen *screen,
					    bool enable)
{
	struct rk61x_platform_data *pdata = rk61x->pdata;
	struct rk61x_route *route = &rk61x->route;

	if (enable) {
		route->vif0_bypass = 0;
		route->vif0_en = 1;
		route->vif0_clk_sel = VIF_CLKIN_SEL_PLL0;
		route->sclin_sel = SCL_SEL_VIF0;
		route->scl_en = 1;
		route->sclk_sel = SCLK_SEL_PLL1;
		route->dither_sel = DITHER_SEL_SCL;
		route->hdmi_sel = HDMI_IN_SEL_VIF0;
		route->hdmi_clk_sel = HDMI_CLK_SEL_VIF0;
	} else {
		route->vif0_bypass = 1;
		route->vif0_en = 0;
		route->sclin_sel = SCL_SEL_VIF0;
		route->scl_en = 0;
		route->dither_sel = DITHER_SEL_VIF0;
		route->hdmi_sel = HDMI_IN_SEL_VIF0;
		route->hdmi_clk_sel = HDMI_CLK_SEL_VIF1;
	}

	route->pll1_clk_sel = LCD0_DCLK;

	if (pdata->pll_clk_sel == MCLK_12M)
		route->pll0_clk_sel = MCLK_12M;
	else
		route->pll0_clk_sel = LCD0_DCLK;

	route->vif1_bypass = 1;
	route->vif1_en = 0;
	route->lcd1_input = 0; /* lcd1 as out put */
	route->lvds_en	= 0;

	return 0;
}

static int rk61x_lcd0_unused_lcd1_input_cfg(struct mfd_rk61x *rk61x,
					    struct rockchip_screen *screen,
					    bool enable)
{
	struct rk61x_platform_data *pdata = rk61x->pdata;
	struct rk61x_route *route = &rk61x->route;

	route->pll0_clk_sel = LCD1_DCLK;

	if (pdata->pll_clk_sel == MCLK_12M)
		route->pll1_clk_sel = MCLK_12M;
	else
		route->pll1_clk_sel = LCD1_DCLK;

	route->vif0_bypass = 1;
	route->vif0_en     = 0;
	if (enable) {
		route->vif1_bypass = 0;
		route->vif1_en = 1;
		route->scl_bypass = 0;
	} else {
		route->vif1_bypass = 1;
		route->vif1_en = 0;
		route->scl_bypass = 1; /* 1:1 scaler */
	}

	route->vif1_clk_sel = VIF_CLKIN_SEL_PLL1;
	route->sclin_sel = SCL_SEL_VIF1; /* from vif1 */
	route->scl_en = 1;
	route->sclk_sel = SCLK_SEL_PLL0;

	route->dither_sel = DITHER_SEL_SCL;
	route->hdmi_sel = HDMI_IN_SEL_VIF1; /* from vif1 */
	route->hdmi_clk_sel = HDMI_CLK_SEL_VIF1;
	route->lcd1_input  = 1;

	if (screen->type == SCREEN_RGB) {
		route->lvds_en = 1;
		route->lvds_mode = RGB; /* rgb output */
	} else if (screen->type == SCREEN_LVDS) {
		route->lvds_en = 1;
		route->lvds_mode = LVDS;
		route->lvds_ch_nr = pdata->lvds_ch_nr;
	} else if (screen->type == SCREEN_MIPI) {
		route->lvds_en = 0;
	}

	return 0;
}

int rk61x_set_router(struct mfd_rk61x *rk61x,
		     struct rockchip_screen *screen, bool enable)
{
	struct rk61x_platform_data *pdata = rk61x->pdata;
	int ret;

	if ((pdata->lcd0_func == INPUT) && (pdata->lcd1_func == INPUT)) {
		ret = rk61x_dual_input_cfg(rk61x, screen, enable);
		rk61x_dbg(rk61x->dev, "rk61x use dual input for dual display!\n");
	} else if ((pdata->lcd0_func == INPUT) &&
		   (pdata->lcd1_func == UNUSED)) {
		ret = rk61x_lcd0_input_lcd1_unused_cfg(rk61x, screen, enable);
		rk61x_dbg(rk61x->dev,
			  "rk61x use lcd0 as input and lvds/rgb "
			  "port as output for dual display\n");
	} else if ((pdata->lcd0_func == INPUT) &&
		   (pdata->lcd1_func == OUTPUT)) {
		ret = rk61x_lcd0_input_lcd1_output_cfg(rk61x, screen, enable);
		rk61x_dbg(rk61x->dev,
			  "rk61x use lcd0 as input and lcd1 as "
			  "output for dual display\n");
	} else if ((pdata->lcd0_func == UNUSED) &&
		   (pdata->lcd1_func == INPUT)) {
		ret = rk61x_lcd0_unused_lcd1_input_cfg(rk61x, screen, enable);
		rk61x_dbg(rk61x->dev,
			  "rk61x use lcd1 as input and lvds/rgb as "
			  "output for dual display\n");
	} else {
		dev_err(rk61x->dev,
			"invalid configration,please check your "
			"rk61x_platform_data setting in your board file!\n");
		return -EINVAL;
	}

	return ret;
}

static int rk61x_router_cfg(struct mfd_rk61x *rk61x)
{
	u32 val;
	int ret;
	struct rk61x_route *route = &rk61x->route;

	/*
	 * pll1 clk from lcdc1_dclk, pll0 clk from lcdc0_dclk,
	 * mux_lcdx = lcdx_clk
	 */
	val = PLL0_CLK_SEL(route->pll0_clk_sel) |
		PLL1_CLK_SEL(route->pll1_clk_sel);
	ret = rk61x->write_dev(rk61x, CRU_CLKSEL0_CON, &val);

	val = SCLK_SEL(route->sclk_sel);
	ret = rk61x->write_dev(rk61x, CRU_CLKSEL1_CON, &val);

	val = SCL_IN_SEL(route->sclin_sel) |
		DITHER_IN_SEL(route->dither_sel) |
		HDMI_IN_SEL(route->hdmi_sel) |
		VIF1_CLK_BYPASS(route->vif1_bypass) |
		VIF0_CLK_BYPASS(route->vif0_bypass) |
		VIF1_CLKIN_SEL(route->vif1_clk_sel) |
		VIF0_CLKIN_SEL(route->vif0_clk_sel);
	ret = rk61x->write_dev(rk61x, CRU_CLKSEL2_CON, &val);

	val = HDMI_CLK_SEL(route->hdmi_clk_sel);
	ret = rk61x->write_dev(rk61x, CRU_CFGMISC_CON, &val);

	return ret;
}

static int rk61x_dither_cfg(struct mfd_rk61x *rk61x,
			    struct rockchip_screen *screen, bool enable)
{
	u32 val = 0;
	int ret = 0;

	if (screen->type != SCREEN_RGB) /* if RGB screen , not invert D_CLK */
		val = FRC_DCLK_INV(1);

	/* enable frc dither if the screen is not 24bit */
	if ((screen->face != OUT_P888) && enable)
		val |= FRC_DITHER_EN(1);
	else
		val |= FRC_DITHER_EN(0);
	ret = rk61x->write_dev(rk61x, FRC_REG, &val);

	return 0;
}

int rk61x_display_router_cfg(struct mfd_rk61x *rk61x,
			     struct rockchip_screen *screen, bool enable)
{
	int ret;
	struct rockchip_screen *hdmi_screen = screen->ext_screen;

	ret = rk61x_set_router(rk61x, screen, enable);
	if (ret < 0)
		return ret;
	ret = rk61x_router_cfg(rk61x);

	ret = rk61x_vif_cfg(rk61x, hdmi_screen, 0); /* cfg vif0 */
	ret = rk61x_vif_cfg(rk61x, hdmi_screen, 1); /* cfg vif1 */

	ret = rk61x_scaler_cfg(rk61x, screen);
	ret = rk61x_dither_cfg(rk61x, screen, enable);
	return 0;
}

int rk61x_set_vif(struct mfd_rk61x *rk61x,
		  struct rockchip_screen *screen, bool connect)
{
	struct rk61x_platform_data *pdata;

	if (!rk61x) {
		pr_err("%s:mfd rk61x is null!\n", __func__);
		return -1;
	}

	pdata = rk61x->pdata;

	if (!connect) {
		rk61x_vif_disable(rk61x, 0);
		rk61x_vif_disable(rk61x, 1);
		rk61x_mclk_set_rate(rk61x->mclk, 11289600);
		return 0;
	}

#if defined(CONFIG_ONE_LCDC_DUAL_OUTPUT_INF)
	return 0;
#else
	if ((pdata->lcd0_func == INPUT) && (pdata->lcd1_func == INPUT)) {
		rk61x_dual_input_cfg(rk61x, screen, connect);
		rk61x_dbg(rk61x->dev, "rk61x use dual input for dual display!\n");
	} else if ((pdata->lcd0_func == INPUT) &&
		   (pdata->lcd1_func == UNUSED)) {
		rk61x_lcd0_input_lcd1_unused_cfg(rk61x, screen, connect);
		rk61x_dbg(rk61x->dev, "rk61x use lcd0 input for hdmi display!\n");
	}

	rk61x_router_cfg(rk61x);
	rk61x_vif_cfg(rk61x, screen, 0);
	rk61x_vif_cfg(rk61x, screen, 1);
	rk61x_scaler_disable(rk61x);
#endif

	return 0;
}
