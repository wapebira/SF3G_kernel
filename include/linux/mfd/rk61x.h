/*
 * include/linux/mfd/rk61x.h
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
 *
 */

#ifndef _RK61X_H_
#define _RK61X_H_

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/rockchip_fb.h>
#include <linux/clk.h>
#include <linux/delay.h>

#if defined(CONFIG_RK61X_DEBUG)
#define rk61x_dbg(dev, format, arg...)		\
	dev_info(dev , format , ## arg)
#else
#define rk61x_dbg(dev, format, arg...)	do {} while (0)
#endif

#define MASK	16

#define VIF0_REG0		0x0000
#define VIF1_REG0		0x0018
#define VIF_DDR_CLK_EN(x)	((((x) & 1) << 3) | (1 << (3 + MASK)))
/* negative edge first en */
#define VIF_DDR_PHASEN_EN(x)	((((x) & 1) << 2) | (1 << (2 + MASK)))
#define VIF_DDR_MODE_EN(x)	((((x) & 1) << 1) | (1 << (1 + MASK)))
#define VIF_EN(x)		((((x) & 1) << 0) | (1 << (0 + MASK)))

#define VIF0_REG1		0x0004
#define VIF1_REG1		0x001C
#define VIF_FRAME_VST(x)	(((x) & 0xfff) << 16)
#define VIF_FRAME_HST(x)	(((x) & 0xfff) << 0)

#define VIF0_REG2		0x0008
#define VIF1_REG2		0x0020
#define VIF_HS_END(x)		(((x) & 0xfff) << 16)
#define VIF_HTOTAL(x)		(((x) & 0xfff) << 0)

#define VIF0_REG3		0x000C
#define VIF1_REG3		0x0024
#define VIF_HACT_END(x)		(((x) & 0xfff) << 16)
#define VIF_HACT_ST(x)		(((x) & 0xfff) << 0)

#define VIF0_REG4		0x0010
#define VIF1_REG4		0x0028
#define VIF_VS_END(x)		(((x) & 0xfff) << 16)
#define VIF_VTOTAL(x)		(((x) & 0xfff) << 0)

#define VIF0_REG5		0x0014
#define VIF1_REG5		0x002C
#define VIF_VACT_END(x)		(((x) & 0xfff) << 16)
#define VIF_VACT_ST(x)		(((x) & 0xfff) << 0)

#define SCL_REG0		0x0030
#define SCL_VER_DOWN_MODE(x)	((((x) & 1) << 8) | (1 << (8 + MASK)))
#define SCL_HOR_DOWN_MODE(x)	((((x) & 1) << 7) | (1 << (7 + MASK)))
#define SCL_BIC_COE_SEL(x)	((((x) & 3) << 5) | (3 << (5 + MASK)))
#define SCL_VER_MODE(x)		((((x) & 3) << 3) | (3 << (3 + MASK)))
#define SCL_HOR_MODE(x)		((((x) & 3) << 1) | (3 << (1 + MASK)))
#define SCL_EN(x)		((((x) & 1) << 0) | (1 << (0 + MASK)))

#define SCL_REG1		0x0034
#define SCL_V_FACTOR(x)		(((x) & 0xffff) << 16)
#define SCL_H_FACTOR(x)		(((x) & 0xffff) << 0)

#define SCL_REG2		0x0038
#define DSP_FRAME_VST(x)	(((x) & 0xfff) << 16)
#define DSP_FRAME_HST(x)	(((x) & 0xfff) << 0)

#define SCL_REG3		0x003C
#define DSP_HS_END(x)		(((x) & 0xfff) << 16)
#define DSP_HTOTAL(x)		(((x) & 0xfff) << 0)

#define SCL_REG4		0x0040
#define DSP_HACT_END(x)		(((x) & 0xfff) << 16)
#define DSP_HACT_ST(x)		(((x) & 0xfff) << 0)

#define SCL_REG5		0x0044
#define DSP_VS_END(x)		(((x) & 0xfff) << 16)
#define DSP_VTOTAL(x)		(((x) & 0xfff) << 0)

#define SCL_REG6		0x0048
#define DSP_VACT_END(x)		(((x) & 0xfff) << 16)
#define DSP_VACT_ST(x)		(((x) & 0xfff) << 0)

#define SCL_REG7		0x004C
#define DSP_HBOR_END(x)		(((x) & 0xfff) << 16)
#define DSP_HBOR_ST(x)		(((x) & 0xfff) << 0)

#define SCL_REG8		0x0050
#define DSP_VBOR_END(x)		(((x) & 0xfff) << 16)
#define DSP_VBOR_ST(x)		(((x) & 0xfff) << 0)

#define FRC_REG			0x0054
#define FRC_DEN_INV(x)		((((x) & 1) << 6) | (1 << (6 + MASK)))
#define FRC_SYNC_INV(x)		((((x) & 1) << 5) | (1 << (5 + MASK)))
#define FRC_DCLK_INV(x)		((((x) & 1) << 4) | (1 << (4 + MASK)))
#define FRC_OUT_ZERO(x)		((((x) & 1) << 3) | (1 << (3 + MASK)))
#define FRC_RGB18_MODE(x)	((((x) & 1) << 2) | (1 << (2 + MASK)))
#define FRC_HIFRC_MODE(x)	((((x) & 1) << 1) | (1 << (1 + MASK)))
#define FRC_DITHER_EN(x)	((((x) & 1) << 0) | (1 << (0 + MASK)))

enum pll_clk_sel {
	LCD0_DCLK = 0,
	LCD1_DCLK,
	MCLK_12M, /* sel source from system clock, it is 26M for sofia3gr */
};

#define CRU_CLKSEL0_CON		0x0058
#define PLL1_CLK_SEL(x)		((((x) & 3) << 8) | (3 << (8 + MASK)))
#define PLL0_CLK_SEL(x)		((((x) & 3) << 6) | (3 << (6 + MASK)))
#define LCD1_CLK_DIV(x)		((((x) & 7) << 3) | (7 << (3 + MASK)))
#define LCD0_CLK_DIV(x)		((((x) & 7) << 0) | (7 << (0 + MASK)))

enum sclk_sel_pll {
	SCLK_SEL_PLL0 = 0,
	SCLK_SEL_PLL1,
};

enum codec_mclk_sel {
	CODEC_MCLK_SEL_PLL0 = 0,
	CODEC_MCLK_SEL_PLL1,
	CODEC_MCLK_SEL_12M,
};

#define CRU_CLKSEL1_CON		0x005C
#define LCDC_CLK_GATE(x)	((((x) & 1) << 12) | (1 << (12 + MASK)))
#define LCDC1_CLK_GATE(x)	((((x) & 1) << 11) | (1 << (11 + MASK)))
#define MIPI_CLK_GATE(x)	((((x) & 1) << 10) | (1 << (10 + MASK)))
#define LVDS_CLK_GATE(x)	((((x) & 1) << 9) | (1 << (9 + MASK)))
#define HDMI_CLK_GATE(x)	((((x) & 1) << 8) | (1 << (8 + MASK)))
#define SCL_CLK_DIV(x)		((((x) & 7) << 5) | (7 << (5 + MASK)))
#define SCL_CLK_GATE(x)		((((x) & 1) << 4) | (1 << (4 + MASK)))
#define SCLK_SEL(x)		((((x) & 1) << 3) | (1 << (3 + MASK)))
#define CODEC_CLK_GATE(x)	((((x) & 1) << 2) | (1 << (2 + MASK)))
#define CODEC_MCLK_SEL(x)	((((x) & 3) << 0) | (3 << (0 + MASK)))

#define CRU_CODEC_DIV		0x0060

enum scl_sel_vif {
	SCL_SEL_VIF0 = 0,
	SCL_SEL_VIF1,
};

enum dither_sel_vif {
	DITHER_SEL_VIF0 = 0,
	DITHER_SEL_SCL,
};

/* hdmi data in select */
enum hdmi_in_sel {
	HDMI_IN_SEL_VIF1 = 0,
	HDMI_IN_SEL_SCL,
	HDMI_IN_SEL_VIF0,
};

enum vif_clkin_sel_pll {
	VIF_CLKIN_SEL_PLL0 = 0,
	VIF_CLKIN_SEL_PLL1,
};

#define CRU_CLKSEL2_CON		0x0064
#define SCL_IN_SEL(x)		((((x) & 1) << 15) | (1 << (15 + MASK)))
#define DITHER_IN_SEL(x)	((((x) & 1) << 14) | (1 << (14 + MASK)))
#define HDMI_IN_SEL(x)		((((x) & 3) << 12) | (3 << (12 + MASK)))
#define VIF1_CLK_DIV(x)		((((x) & 7) << 9) | (7 << (9 + MASK)))
#define VIF1_CLK_GATE(x)	((((x) & 1) << 8) | (1 << (8 + MASK)))
#define VIF1_CLK_BYPASS(x)	((((x) & 1) << 7) | (1 << (7 + MASK)))
#define VIF1_CLKIN_SEL(x)	((((x) & 1) << 6) | (1 << (6 + MASK)))
#define VIF0_CLK_DIV(x)		((((x) & 7) << 3) | (7 << (3 + MASK)))
#define VIF0_CLK_GATE(x)	((((x) & 1) << 2) | (1 << (2 + MASK)))
#define VIF0_CLK_BYPASS(x)	((((x) & 1) << 1) | (1 << (1 + MASK)))
#define VIF0_CLKIN_SEL(x)	((((x) & 1) << 0) | (1 << (0 + MASK)))

#define CRU_PLL0_CON0		0x0068
#define PLL0_BYPASS(x)		((((x) & 1) << 15) | (1 << (15 + MASK)))
#define PLL0_POSTDIV1(x)	((((x) & 7) << 12) | (7 << (12 + MASK)))
#define PLL0_FBDIV(x)		((((x) & 0xfff) << 0) | (0xfff << (0 + MASK)))

#define CRU_PLL0_CON1		0x006C
#define PLL0_LOCK		(1 << 15)		/* only read */
#define PLL0_PWR_DN(x)		((((x) & 1) << 10) | (1 << (10 + MASK)))
#define PLL0_DIV_MODE(x)	((((x) & 1) << 9) | (1 << (9 + MASK)))
#define PLL0_POSTDIV2(x)	((((x) & 7) << 6) | (7 << (6 + MASK)))
#define PLL0_REFDIV(x)		((((x) & 0x3f) << 0) | (0x3f << (0 + MASK)))

#define CRU_PLL0_CON2		0x0070
#define PLL0_FOUT4_PWR_DN	(1 << 27)
#define PLL0_FOUTVCO_PWR_DN	(1 << 26)
#define PLL0_POSTDIV_PWR_DN	(1 << 25)
#define PLL0_DAC_PWR_DN		(1 << 24)
#define PLL0_FRAC(x)		(((x) & 0xffffff) << 0)

#define CRU_PLL1_CON0		0x0074
#define PLL1_BYPASS(x)		((((x) & 1) << 15) | (1 << (15 + MASK)))
#define PLL1_POSTDIV1(x)	((((x) & 7) << 12) | (7 << (12 + MASK)))
#define PLL1_FBDIV(x)		((((x) & 0xfff) << 0) | (0xfff << (0 + MASK)))

#define CRU_PLL1_CON1		0x0078
#define PLL1_LOCK		(1 << 15)		/* only read */
#define PLL1_PWR_DN		((((x) & 1) << 10) | (1 << (10 + MASK)))
#define PLL1_DIV_MODE		((((x) & 1) << 9) | (1 << (9 + MASK)))
#define PLL1_POSTDIV2(x)	((((x) & 7) << 6) | (7 << (6 + MASK)))
#define PLL1_REFDIV(x)		((((x) & 0x3f) << 0) | (0x3f << (0 + MASK)))

#define CRU_PLL1_CON2		0x007C
#define PLL1_FOUT4_PWR_DN	(1 << 27)
#define PLL1_FOUTVCO_PWR_DN	(1 << 26)
#define PLL1_POSTDIV_PWR_DN	(1 << 25)
#define PLL1_DAC_PWR_DN		(1 << 24)
#define PLL1_FRAC(x)		(((x) & 0xffffff) << 0)

#define CRU_I2C_CON0		0x0080

#define CRU_LVDS_CON0		0x0084
#define LVDS_HBP_ODD(x)		((((x) & 1) << 14) | (1 << (14 + MASK)))
#define LVDS_DCLK_INV(x)	((((x) & 1) << 13) | (1 << (13 + MASK)))
#define LVDS_CH1_LOAD(x)	((((x) & 1) << 12) | (1 << (12 + MASK)))
#define LVDS_CH0_LOAD(x)	((((x) & 1) << 11) | (1 << (11 + MASK)))
#define LVDS_CH1TTL_EN(x)	((((x) & 1) << 10) | (1 << (10 + MASK)))
#define LVDS_CH0TTL_EN(x)	((((x) & 1) << 9) | (1 << (9 + MASK)))
#define LVDS_CH1_PWR_EN(x)	((((x) & 1) << 8) | (1 << (8 + MASK)))
#define LVDS_CH0_PWR_EN(x)	((((x) & 1) << 7) | (1 << (7 + MASK)))
#define LVDS_CBG_PWR_EN(x)	((((x) & 1) << 6) | (1 << (6 + MASK)))
#define LVDS_PLL_PWR_DN(x)	((((x) & 1) << 5) | (1 << (5 + MASK)))
#define LVDS_START_CH_SEL(x)	((((x) & 1) << 4) | (1 << (4 + MASK)))
#define LVDS_CH_SEL(x)		((((x) & 1) << 3) | (1 << (3 + MASK)))
#define LVDS_MSB_SEL(x)		((((x) & 1) << 2) | (1 << (2 + MASK)))
#define LVDS_OUT_FORMAT(x)	((((x) & 3) << 0) | (3 << (0 + MASK)))

#define CRU_IO_CON0		0x0088
#define VIF1_SYNC_EN(x)		((((x) & 1) << 15) | (1 << (15 + MASK)))
#define VIF0_SYNC_EN(x)		((((x) & 1) << 14) | (1 << (14 + MASK)))
#define I2S1_OUT_DISABLE(x)	((((x) & 1) << 13) | (1 << (13 + MASK)))
#define I2S0_OUT_DISABLE(x)	((((x) & 1) << 12) | (1 << (12 + MASK)))
#define LVDS_OUT_EN(x)		((((x) & 1) << 11) | (1 << (11 + MASK)))
#define LCD1_INPUT_EN(x)	((((x) & 1) << 10) | (1 << (10 + MASK)))
#define LVDS_RGBIO_PD_DISABLE(x)	((((x) & 1) << 9) | (1 << (9 + MASK)))
#define LCD1_IO_PD_DISABLE(x)	((((x) & 1) << 8) | (1 << (8 + MASK)))
#define LCD0_IO_PD_DISABLE(x)	((((x) & 1) << 7) | (1 << (7 + MASK)))
#define HDMI_IO_PU_DISABLE(x)	((((x) & 1) << 6) | (1 << (6 + MASK)))
#define SPDIF_IO_PD_DISABLE(x)	((((x) & 1) << 5) | (1 << (5 + MASK)))
#define I2S1_PD_DISABLE(x)	((((x) & 1) << 4) | (1 << (4 + MASK)))
#define I2S0_PD_DISABLE(x)	((((x) & 1) << 3) | (1 << (3 + MASK)))
#define I2C_PU_DISABLE(x)	((((x) & 1) << 2) | (1 << (2 + MASK)))
#define INT_IO_PU(x)		((((x) & 1) << 1) | (1 << (1 + MASK)))
#define CLKIN_PU(x)		((((x) & 1) << 0) | (1 << (0 + MASK)))

#define CRU_IO_CON1		0x008C
/* shmitt input enable */
#define LVDS_RGBIO_SI_EN(x)	((((x) & 1) << 9) | (1 << (9 + MASK)))
#define LCD1_SI_EN(x)		((((x) & 1) << 8) | (1 << (8 + MASK)))
#define LCD0_SI_EN(x)		((((x) & 1) << 7) | (1 << (7 + MASK)))
#define HDMI_SI_EN(x)		((((x) & 1) << 6) | (1 << (6 + MASK)))
#define SPDIF_SI_EN(x)		((((x) & 1) << 5) | (1 << (5 + MASK)))
#define I2S1_SI_EN(x)		((((x) & 1) << 4) | (1 << (4 + MASK)))
#define I2S0_SI_EN(x)		((((x) & 1) << 3) | (1 << (3 + MASK)))
#define I2C_SI_EN(x)		((((x) & 1) << 2) | (1 << (2 + MASK)))
#define INT_SI_EN(x)		((((x) & 1) << 1) | (1 << (1 + MASK)))
#define CLKIN_SI_EN(x)		((((x) & 1) << 0) | (1 << (0 + MASK)))

#define CRU_PCM2IS2_CON0	0x0090
#define CRU_PCM2IS2_CON1	0x0094
#define CRU_PCM2IS2_CON2	0x0098

enum hdmi_clk_sel {
	HDMI_CLK_SEL_VIF1 = 0,
	HDMI_CLK_SEL_SCL,
	HDMI_CLK_SEL_VIF0,
};

#define CRU_CFGMISC_CON		0x009C
#define HDMI_CLK_SEL(x)		((((x) & 3) << 12) | (3 << (12 + MASK)))

/*
 * the function of lcd ports(lcd0,lcd1),
 * the lcd0 only can be used as input or unused
 * the lcd1 can used as input or output or unused
 */
enum lcd_port_func {
	UNUSED,
	INPUT,
	OUTPUT,
};

enum lvds_mode {
	RGB,
	LVDS,
};

struct rk61x_platform_data {
	int (*power_init)(void);
	int (*power_deinit)(void);
	enum lcd_port_func lcd0_func;
	enum lcd_port_func lcd1_func;
	int lvds_ch_nr;	/* the number of used  lvds channel */
	int hdmi_irq;
	int pll_clk_sel;
};

struct rk61x_route {
	u8 vif0_bypass;
	u8 vif0_en;
	u8 vif0_clk_sel;
	u8 vif1_bypass;
	u8 vif1_en;
	u8 vif1_clk_sel;
	u8 sclin_sel;
	u8 scl_en;
	u8 scl_bypass;
	u8 dither_sel;
	u8 hdmi_sel;
	u8 hdmi_clk_sel;
	u8 pll0_clk_sel;
	u8 pll1_clk_sel;
	u8 sclk_sel;
	u8 lcd1_input;
	u8 lvds_en;
	enum lvds_mode lvds_mode; /* RGB or LVDS */
	int lvds_ch_nr; /* the number of used  lvds channel */
};

struct mfd_rk61x {
	struct mutex reg_lock; /* muxtex lock for register config */
	struct device *dev;
	unsigned int irq_base;
	struct rk61x_platform_data *pdata;
	struct rk61x_route route; /* display path router */
	struct i2c_client *client;
	struct clk *mclk;
	u64 pll0_rate;
	u64 pll1_rate;
	unsigned int vif_suspend;
	struct dentry *debugfs_dir;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device_pm_platdata *pm_platdata;
#endif

	int (*read_dev)(struct mfd_rk61x *rk61x, u16 reg, u32 *pval);
	int (*write_dev)(struct mfd_rk61x *rk61x, u16 reg, u32 *pval);
	int (*write_dev_bits)(struct mfd_rk61x *rk61x, u16 reg,
			      u32 mask, u32 *pval);
	int (*write_bulk)(struct mfd_rk61x *rk61x, u16 reg,
			  int count, u32 *pval);
};

int rk61x_set_vif(struct mfd_rk61x *rk61x,
		  struct rockchip_screen *screen, bool connect);
int rk61x_display_router_cfg(struct mfd_rk61x *rk61x,
			     struct rockchip_screen *screen, bool enable);
void rk61x_mclk_set_rate(struct clk *mclk, unsigned long rate);
int rk61x_pll_set_rate(struct mfd_rk61x *rk61x, int id, u32 cfg_val, u32 frac);
int rk61x_pll_pwr_down(struct mfd_rk61x *rk61x, int id);

#endif
