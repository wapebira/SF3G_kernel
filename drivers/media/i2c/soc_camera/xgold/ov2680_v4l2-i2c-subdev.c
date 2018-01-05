/*
 * ov2680 sensor driver
 *
 * Copyright (C) 2012-2014 Intel Mobile Communications GmbH
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Note:
 *    09/25/2014: new implementation using v4l2-subdev
 *                        instead of v4l2-int-device.
 */

#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-core.h>
#include <linux/slab.h>
#include "ov_camera_module.h"

#define OV2680_DRIVER_NAME "ov2680"

#define OV2680_FETCH_LSB_GAIN(VAL) (VAL & 0x00FF)       /* gain[7:0] */
#define OV2680_FETCH_MSB_GAIN(VAL) ((VAL >> 8) & 0x7)	/* gain[10:8] */
#define OV2680_AEC_PK_LONG_GAIN_HIGH_REG 0x350a	/* Bit 8 */
#define OV2680_AEC_PK_LONG_GAIN_LOW_REG	 0x350b	/* Bits 0 -7 */

#define OV2680_AEC_PK_LONG_EXPO_3RD_REG 0x3500	/* Exposure Bits 16-19 */
#define OV2680_AEC_PK_LONG_EXPO_2ND_REG 0x3501	/* Exposure Bits 8-15 */
#define OV2680_AEC_PK_LONG_EXPO_1ST_REG 0x3502	/* Exposure Bits 0-7 */

#define OV2680_AEC_GROUP_UPDATE_ADDRESS 0x3208
#define OV2680_AEC_GROUP_UPDATE_START_DATA 0x00
#define OV2680_AEC_GROUP_UPDATE_END_DATA 0x10
#define OV2680_AEC_GROUP_UPDATE_END_LAUNCH 0xA0

#define OV2680_FETCH_3RD_BYTE_EXP(VAL) ((VAL >> 12) & 0xF)	/* 4 Bits */
#define OV2680_FETCH_2ND_BYTE_EXP(VAL) ((VAL >> 4) & 0xFF)	/* 8 Bits */
#define OV2680_FETCH_1ST_BYTE_EXP(VAL) ((VAL & 0x0F)<<4)	/* 4 Bits */

#define OV2680_PIDH_ADDR     0x300A
#define OV2680_PIDL_ADDR     0x300B

/* High byte of product ID */
#define OV2680_PIDH_MAGIC 0x26
/* Low byte of product ID  */
#define OV2680_PIDL_MAGIC 0x80

#define OV2680_EXT_CLK 26000000
#define OV2680_PLL_PREDIV0_REG 0x3088
#define OV2680_PLL_PREDIV_REG  0x3080
#define OV2680_PLL_MUL_HIGH_REG 0x3081
#define OV2680_PLL_MUL_LOW_REG 0x3082
#define OV2680_PLL_SPDIV_REG 0x3086
#define OV2680_PLL_DIVSYS_REG 0x3084
#define OV2680_TIMING_VTS_HIGH_REG 0x380e
#define OV2680_TIMING_VTS_LOW_REG 0x380f
#define OV2680_TIMING_HTS_HIGH_REG 0x380c
#define OV2680_TIMING_HTS_LOW_REG 0x380d
#define OV2680_COARSE_INTG_TIME_MIN 16
#define OV2680_COARSE_INTG_TIME_MAX_MARGIN 4
#define OV2680_TIMING_X_INC		0x3814
#define OV2680_TIMING_Y_INC		0x3815
#define OV2680_HORIZONTAL_START_HIGH_REG 0x3800
#define OV2680_HORIZONTAL_START_LOW_REG 0x3801
#define OV2680_VERTICAL_START_HIGH_REG 0x3802
#define OV2680_VERTICAL_START_LOW_REG 0x3803
#define OV2680_HORIZONTAL_END_HIGH_REG 0x3804
#define OV2680_HORIZONTAL_END_LOW_REG 0x3805
#define OV2680_VERTICAL_END_HIGH_REG 0x3806
#define OV2680_VERTICAL_END_LOW_REG 0x3807
#define OV2680_HORIZONTAL_OUTPUT_SIZE_HIGH_REG 0x3808
#define OV2680_HORIZONTAL_OUTPUT_SIZE_LOW_REG 0x3809
#define OV2680_VERTICAL_OUTPUT_SIZE_HIGH_REG 0x380a
#define OV2680_VERTICAL_OUTPUT_SIZE_LOW_REG 0x380b
#define OV2680_H_WIN_OFF_HIGH_REG 0x3810
#define OV2680_H_WIN_OFF_LOW_REG 0x3811
#define OV2680_V_WIN_OFF_HIGH_REG 0x3812
#define OV2680_V_WIN_OFF_LOW_REG 0x3813

/* ======================================================================== */
/* Base sensor configs */
/* ======================================================================== */
/* MCLK:26MHz  1600x1200  28fps   mipi 1lane   663Mbps/lane */
static const struct ov_camera_module_reg ov2680_init_tab_1600_1200_28fps[] = {
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x0103, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3002, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3016, 0x1c},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3018, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3020, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3080, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3082, 0x33},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3084, 0x09},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3085, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3086, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3501, 0xc3},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3502, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3503, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x350b, 0x80},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3600, 0xb4},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3603, 0x35},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3604, 0x24},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3605, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3620, 0x24},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3621, 0x37},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3622, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3628, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3701, 0x64},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3705, 0x3c},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x370c, 0x50},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x370d, 0xc0},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3718, 0x80},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3720, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3721, 0x09},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3722, 0x06},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3723, 0x59},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3738, 0x99},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x370a, 0x21},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3717, 0x58},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3781, 0x80},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3784, 0x0c},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3789, 0x60},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3800, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3801, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3802, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3803, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3804, 0x06},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3805, 0x4f},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3806, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3807, 0xbf},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3808, 0x06},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3809, 0x40},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x380a, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x380b, 0xb0},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x380c, 0x06},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x380d, 0xa4},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x380e, 0x05},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x380f, 0x6a},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3810, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3811, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3812, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3813, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3814, 0x11},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3815, 0x11},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3819, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3820, 0xc0},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3821, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x4000, 0x81},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x4001, 0x40},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x4008, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x4009, 0x09},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x4602, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x481f, 0x36},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x4825, 0x36},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x4837, 0x18},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5002, 0x30},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5080, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5081, 0x41},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5780, 0x3e},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5781, 0x0f},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5782, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5783, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5784, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5785, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5786, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5787, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5788, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5789, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x578a, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x578b, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x578c, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x578d, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x578e, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x578f, 0x0c},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5790, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5791, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5792, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5793, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5794, 0x03},
};
/* MCLK:26MHz  800x600  30fps   mipi 1lane   663Mbps/lane */
static const struct ov_camera_module_reg ov2680_init_tab_800_600_30fps[] = {

{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x0103, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3002, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3016, 0x1c},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3018, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3020, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3080, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3082, 0x33},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3084, 0x09},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3085, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3086, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3501, 0x95},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3502, 0x70},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3503, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x350b, 0x80},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3600, 0xb4},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3603, 0x35},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3604, 0x24},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3605, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3620, 0x26},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3621, 0x37},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3622, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3628, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3701, 0x64},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3705, 0x3c},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x370c, 0x50},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x370d, 0xc0},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3718, 0x88},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3720, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3721, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3722, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3723, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3738, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x370a, 0x23},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3717, 0x58},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3781, 0x80},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3784, 0x0c},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3789, 0x60},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3800, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3801, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3802, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3803, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3804, 0x06},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3805, 0x4f},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3806, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3807, 0xbf},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3808, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3809, 0x20},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x380a, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x380b, 0x58},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x380c, 0x06},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x380d, 0xac},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x380e, 0x05},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x380f, 0x14},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3810, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3811, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3812, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3813, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3814, 0x31},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3815, 0x31},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3819, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3820, 0xc2},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x3821, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x4000, 0x81},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x4001, 0x40},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x4008, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x4009, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x4602, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x481f, 0x36},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x4825, 0x36},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x4837, 0x18},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5002, 0x30},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5080, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5081, 0x41},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5780, 0x3e},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5781, 0x0f},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5782, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5783, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5784, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5785, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5786, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5787, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5788, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5789, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x578a, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x578b, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x578c, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x578d, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x578e, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x578f, 0x0c},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5790, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5791, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5792, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5793, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA,  0x5794, 0x03},

};

/* ======================================================================== */

static struct ov_camera_module_config ov2680_configs[] = {
	/*{
		.name = "800x600_30fps",
		.frm_fmt = {
			.width = 800,
			.height = 600,
			.code = V4L2_MBUS_FMT_SBGGR10_1X10
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 30
			}
		},
		.auto_exp_enabled = false,
		.auto_gain_enabled = false,
		.auto_wb_enabled = false,
		.reg_table = (void *)ov2680_init_tab_800_600_30fps,
		.reg_table_num_entries =
			sizeof(ov2680_init_tab_800_600_30fps)
			/
			sizeof(ov2680_init_tab_800_600_30fps[0]),
	    .v_blanking_time_us = 17949
	},*/
	{
		.name = "1600x1200_28fps",
		.frm_fmt = {
			.width = 1600,
			.height = 1200,
			.code = V4L2_MBUS_FMT_SBGGR10_1X10
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 28
			}
		},
		.auto_exp_enabled = false,
		.auto_gain_enabled = false,
		.auto_wb_enabled = false,
		.reg_table = (void *)ov2680_init_tab_1600_1200_28fps,
		.reg_table_num_entries =
			sizeof(ov2680_init_tab_1600_1200_28fps)
			/
			sizeof(ov2680_init_tab_1600_1200_28fps[0]),
	    .v_blanking_time_us = 4000
	},
};

/*--------------------------------------------------------------------------*/
static int OV2680_g_VTS(struct ov_camera_module *cam_mod, u32 *vts)
{
	u32 msb, lsb;
	int ret;

	ret = ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_TIMING_VTS_HIGH_REG,
		&msb);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_TIMING_VTS_LOW_REG,
		&lsb);
	if (IS_ERR_VALUE(ret))
		goto err;

	*vts = (msb << 8) | lsb;

	return 0;
err:
	ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}
static int OV2680_auto_adjust_fps(struct ov_camera_module *cam_mod,
	u32 exp_time)
{
	int ret;
	u32 vts;

	if ((cam_mod->exp_config.exp_time + OV2680_COARSE_INTG_TIME_MAX_MARGIN)
		> cam_mod->vts_min)
		vts = cam_mod->exp_config.exp_time+OV2680_COARSE_INTG_TIME_MAX_MARGIN;
	else
		vts = cam_mod->vts_min;

	ret = ov_camera_module_write_reg(cam_mod,
		OV2680_TIMING_VTS_LOW_REG,
		vts & 0xFF);
	ret |= ov_camera_module_write_reg(cam_mod,
		OV2680_TIMING_VTS_HIGH_REG,
		(vts >> 8) & 0xFF);

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_err(cam_mod,
				"failed with error (%d)\n", ret);
	else
		ov_camera_module_pr_debug(cam_mod,
					  "updated vts = %d,vts_min=%d\n", vts, cam_mod->vts_min);

	return ret;
}
static int ov2680_write_aec(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod,
		"exp_time = %d, gain = %d, flash_mode = %d\n",
		cam_mod->exp_config.exp_time,
		cam_mod->exp_config.gain,
		cam_mod->exp_config.flash_mode);

	/* if the sensor is already streaming, write to shadow registers,
		if the sensor is in SW standby, write to active registers,
		if the sensor is off/registers are not writeable, do nothing */
	if ((cam_mod->state == OV_CAMERA_MODULE_SW_STANDBY) ||
		(cam_mod->state == OV_CAMERA_MODULE_STREAMING)) {
		u32 a_gain = cam_mod->exp_config.gain;
		u32 exp_time = cam_mod->exp_config.exp_time;
		if (cam_mod->state == OV_CAMERA_MODULE_STREAMING)
			ret = ov_camera_module_write_reg(cam_mod,
				OV2680_AEC_GROUP_UPDATE_ADDRESS,
				OV2680_AEC_GROUP_UPDATE_START_DATA);
		if (!IS_ERR_VALUE(ret) && cam_mod->auto_adjust_fps)
			ret = OV2680_auto_adjust_fps(cam_mod, cam_mod->exp_config.exp_time);
		ret |= ov_camera_module_write_reg(cam_mod,
			OV2680_AEC_PK_LONG_GAIN_HIGH_REG,
			OV2680_FETCH_MSB_GAIN(a_gain));
		ret |= ov_camera_module_write_reg(cam_mod,
			OV2680_AEC_PK_LONG_GAIN_LOW_REG,
			OV2680_FETCH_LSB_GAIN(a_gain));
		ret = ov_camera_module_write_reg(cam_mod,
			OV2680_AEC_PK_LONG_EXPO_3RD_REG,
			OV2680_FETCH_3RD_BYTE_EXP(exp_time));
		ret |= ov_camera_module_write_reg(cam_mod,
			OV2680_AEC_PK_LONG_EXPO_2ND_REG,
			OV2680_FETCH_2ND_BYTE_EXP(exp_time));
		ret |= ov_camera_module_write_reg(cam_mod,
			OV2680_AEC_PK_LONG_EXPO_1ST_REG,
			OV2680_FETCH_1ST_BYTE_EXP(exp_time));
		if (cam_mod->state == OV_CAMERA_MODULE_STREAMING) {
			ret = ov_camera_module_write_reg(cam_mod,
				OV2680_AEC_GROUP_UPDATE_ADDRESS,
				OV2680_AEC_GROUP_UPDATE_END_DATA);
			ret = ov_camera_module_write_reg(cam_mod,
				OV2680_AEC_GROUP_UPDATE_ADDRESS,
				OV2680_AEC_GROUP_UPDATE_END_LAUNCH);
		}
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

static int ov2680_g_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_FLASH_LED_MODE:
		/* nothing to be done here */
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov2680_g_timings(struct ov_camera_module *cam_mod,
	struct ov_camera_module_timings *timings)
{
	int ret = 0;
	u32 reg_val;
	u32 win_off;

	if (IS_ERR_OR_NULL(cam_mod->active_config))
		goto err;

	/*VTS*/
	/*
	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_TIMING_VTS_HIGH_REG,
		&reg_val)))
		goto err;

	timings->frame_length_lines = reg_val <<  8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_TIMING_VTS_LOW_REG,
		&reg_val)))
		goto err;

	timings->frame_length_lines |= reg_val;
	*/
	timings->frame_length_lines = 0x56a;

	/*HTS*/
	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_TIMING_HTS_HIGH_REG,
		&reg_val)))
		goto err;

	timings->line_length_pck = reg_val << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_TIMING_HTS_LOW_REG,
		&reg_val)))
		goto err;

	timings->line_length_pck |= reg_val;

	timings->coarse_integration_time_min = OV2680_COARSE_INTG_TIME_MIN;
	timings->coarse_integration_time_max_margin =
		OV2680_COARSE_INTG_TIME_MAX_MARGIN;

	/* OV Sensor do not use fine integration time. */
	timings->fine_integration_time_min = 0;
	timings->fine_integration_time_max_margin = 0;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_TIMING_X_INC,
		&reg_val)))
		goto err;

	timings->binning_factor_x = ((reg_val >> 4) + 1) / 2;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_TIMING_Y_INC,
		&reg_val)))
		goto err;

	timings->binning_factor_y = ((reg_val >> 4) + 1) / 2;

	/* Get the cropping and output resolution to ISP for this mode. */
	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_HORIZONTAL_START_HIGH_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_start = reg_val << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_HORIZONTAL_START_LOW_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_start |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_VERTICAL_START_HIGH_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_start = reg_val << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_VERTICAL_START_LOW_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_start |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_HORIZONTAL_END_HIGH_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_end = reg_val << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_HORIZONTAL_END_LOW_REG,
		&reg_val)))
		goto err;

	timings->crop_horizontal_end |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_VERTICAL_END_HIGH_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_end = reg_val << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_VERTICAL_END_LOW_REG,
		&reg_val)))
		goto err;

	timings->crop_vertical_end |= reg_val;

	/* The sensor can do windowing within the cropped array.
	Take this into the cropping size reported. */
	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_H_WIN_OFF_HIGH_REG,
		&reg_val)))
		goto err;

	win_off = (reg_val & 0xf) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_H_WIN_OFF_LOW_REG,
		&reg_val)))
		goto err;

	win_off |= (reg_val & 0xff);

	timings->crop_horizontal_start += win_off;
	timings->crop_horizontal_end -= win_off;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_V_WIN_OFF_HIGH_REG,
		&reg_val)))
		goto err;

	win_off = (reg_val & 0xf) << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_V_WIN_OFF_LOW_REG,
		&reg_val)))
		goto err;

	win_off |= (reg_val & 0xff);

	timings->crop_vertical_start += win_off;
	timings->crop_vertical_end -= win_off;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_HORIZONTAL_OUTPUT_SIZE_HIGH_REG,
		&reg_val)))
		goto err;

	timings->sensor_output_width = reg_val << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_HORIZONTAL_OUTPUT_SIZE_LOW_REG,
		&reg_val)))
		goto err;

	timings->sensor_output_width |= reg_val;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_VERTICAL_OUTPUT_SIZE_HIGH_REG,
		&reg_val)))
		goto err;

	timings->sensor_output_height = reg_val << 8;

	if (IS_ERR_VALUE(ov_camera_module_read_reg_table(
		cam_mod,
		OV2680_VERTICAL_OUTPUT_SIZE_LOW_REG,
		&reg_val)))
		goto err;

	timings->sensor_output_height |= reg_val;

	timings->vt_pix_clk_freq_hz = cam_mod->frm_intrvl.interval.denominator
					* timings->frame_length_lines
					* timings->line_length_pck;
	return ret;
err:
	ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov2680_s_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
		ret = ov2680_write_aec(cam_mod);
		break;
	case V4L2_CID_FLASH_LED_MODE:
		/* nothing to be done here */
		break;
	case V4L2_CID_FOCUS_ABSOLUTE:
		/* todo*/
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov2680_s_ext_ctrls(struct ov_camera_module *cam_mod,
				 struct ov_camera_module_ext_ctrls *ctrls)
{
	int ret = 0;

	/* Handles only exposure and gain together special case. */
	if (ctrls->count == 1)
		ret = ov2680_s_ctrl(cam_mod, ctrls->ctrls[0].id);
	else if (ctrls->count == 2 &&
		((ctrls->ctrls[0].id == V4L2_CID_GAIN &&
		ctrls->ctrls[1].id == V4L2_CID_EXPOSURE) ||
		(ctrls->ctrls[1].id == V4L2_CID_GAIN &&
		ctrls->ctrls[0].id == V4L2_CID_EXPOSURE)))
		ret = ov2680_write_aec(cam_mod);
	else
		ret = -EINVAL;

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);

	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov2680_start_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "active config=%s\n", cam_mod->active_config->name);

	ret = OV2680_g_VTS(cam_mod, &cam_mod->vts_min);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = ov2680_write_aec(cam_mod);
	if (IS_ERR_VALUE(ret))
		goto err;
	if (IS_ERR_VALUE(ov_camera_module_write_reg(cam_mod, 0x0100, 1)))
		goto err;

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n",
		ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov2680_stop_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret = ov_camera_module_write_reg(cam_mod, 0x0100, 0);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov2680_check_camera_id(struct ov_camera_module *cam_mod)
{
	u32 pidh, pidl;
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret |= ov_camera_module_read_reg(cam_mod, 1, OV2680_PIDH_ADDR, &pidh);
	ret |= ov_camera_module_read_reg(cam_mod, 1, OV2680_PIDL_ADDR, &pidl);
	if (IS_ERR_VALUE(ret)) {
		ov_camera_module_pr_err(cam_mod,
			"register read failed, camera module powered off?\n");
		goto err;
	}

	if ((pidh == OV2680_PIDH_MAGIC) && (pidl == OV2680_PIDL_MAGIC))
		ov_camera_module_pr_debug(cam_mod,
			"successfully detected camera ID 0x%02x%02x\n",
			pidh, pidl);
	else {
		ov_camera_module_pr_err(cam_mod,
			"wrong camera ID, expected 0x%02x%02x, detected 0x%02x%02x\n",
			OV2680_PIDH_MAGIC, OV2680_PIDL_MAGIC, pidh, pidl);
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}


/* ======================================================================== */
int ov_camera_2680_module_s_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{

	return 0;
}

/* ======================================================================== */

int ov_camera_2680_module_s_ext_ctrls(
	struct v4l2_subdev *sd,
	struct v4l2_ext_controls *ctrls)
{

	return 0;
}


long ov_camera_2680_module_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd,
	void *arg)
{
	return 0;
}


/* ======================================================================== */
/* This part is platform dependent */
/* ======================================================================== */

static struct v4l2_subdev_core_ops ov2680_camera_module_core_ops = {
	.g_ctrl = ov_camera_module_g_ctrl,
	.s_ctrl = ov_camera_module_s_ctrl,
	.s_ext_ctrls = ov_camera_module_s_ext_ctrls,
	.s_power = ov_camera_module_s_power,
	.ioctl = ov_camera_module_ioctl
};

static struct v4l2_subdev_video_ops ov2680_camera_module_video_ops = {
	.enum_frameintervals = ov_camera_module_enum_frameintervals,
	.s_mbus_fmt = ov_camera_module_s_fmt,
	.g_mbus_fmt = ov_camera_module_g_fmt,
	.try_mbus_fmt = ov_camera_module_try_fmt,
	.s_frame_interval = ov_camera_module_s_frame_interval,
	.s_stream = ov_camera_module_s_stream
};

static struct v4l2_subdev_ops ov2680_camera_module_ops = {
	.core = &ov2680_camera_module_core_ops,
	.video = &ov2680_camera_module_video_ops
};

static struct ov_camera_module ov2680;
static struct ov_camera_module ov2680_1;
static int num_cameras;

static struct ov_camera_module_custom_config ov2680_custom_config = {
	.start_streaming = ov2680_start_streaming,
	.stop_streaming = ov2680_stop_streaming,
	.s_ctrl = ov2680_s_ctrl,
	.g_ctrl = ov2680_g_ctrl,
	.s_ext_ctrls = ov2680_s_ext_ctrls,
	.g_timings = ov2680_g_timings,
	.check_camera_id = ov2680_check_camera_id,
	.configs = ov2680_configs,
	.num_configs = sizeof(ov2680_configs) / sizeof(ov2680_configs[0]),
	.power_up_delays_ms = {5, 30, 0}
};

static int ov2680_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct ov_camera_module *tmp_ov2680 = NULL;

	dev_info(&client->dev, "probing...\n");

	tmp_ov2680 = (num_cameras == 0) ? &ov2680 : &ov2680_1;
	v4l2_i2c_subdev_init(&tmp_ov2680->sd, client,
				&ov2680_camera_module_ops);
	ret = ov_camera_module_init(tmp_ov2680,
			&ov2680_custom_config);
	if (IS_ERR_VALUE(ret))
		goto err;
	num_cameras++;
	dev_info(&client->dev, "probing successful\n");
	return 0;
err:
	dev_err(&client->dev, "probing failed with error (%d)\n", ret);
	ov_camera_module_release(&ov2680);
	return ret;
}

/* ======================================================================== */

static int ov2680_remove(
	struct i2c_client *client)
{
	struct ov_camera_module *cam_mod = i2c_get_clientdata(client);

	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	ov_camera_module_release(cam_mod);

	dev_info(&client->dev, "removed\n");
	return 0;
}

static const struct i2c_device_id ov2680_id[] = {
	{ OV2680_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id ov2680_of_match[] = {
	{.compatible = "omnivision," OV2680_DRIVER_NAME "-v4l2-i2c-subdev",},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov2680_id);

static struct i2c_driver ov2680_i2c_driver = {
	.driver = {
		.name = OV2680_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ov2680_of_match
	},
	.probe = ov2680_probe,
	.remove = ov2680_remove,
	.id_table = ov2680_id,
};

module_i2c_driver(ov2680_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for ov2680");
MODULE_AUTHOR("Eike Grimpe");
MODULE_LICENSE("GPL");

