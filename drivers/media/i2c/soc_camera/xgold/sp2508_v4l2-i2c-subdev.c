/*
 * drivers/media/i2c/soc_camera/xgold/sp2508.c
 *
 * sp2508 sensor driver
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
 *    11/14/2014: new implementation using v4l2-subdev
 *                        instead of v4l2-int-device.
 */

#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-core.h>
#include <linux/slab.h>
#include "sp_camera_module.h"

#define SP2508_DRIVER_NAME "sp2508"


#define SP2508_INTEGRATION_TIME_MARGIN         4

#define SP2508_HORIZONTAL_START_HIGH_REG       		0x3b
#define SP2508_HORIZONTAL_START_LOW_REG        		0x3c
#define SP2508_VERTICAL_START_HIGH_REG           		0x37
#define SP2508_VERTICAL_START_LOW_REG          	 	0x38


#define SP2508_HORIZONTAL_OUTPUT_SIZE_HIGH_REG 		0x3d
#define SP2508_HORIZONTAL_OUTPUT_SIZE_LOW_REG  		0x3e
#define SP2508_VERTICAL_OUTPUT_SIZE_HIGH_REG   		0x39
#define SP2508_VERTICAL_OUTPUT_SIZE_LOW_REG    		0x3a



#define SP2508_PIDH_ADDR                       0x02
#define SP2508_PIDL_ADDR                       0x03

/* High byte of product ID */
#define SP2508_PIDH_MAGIC 0x25
/* Low byte of product ID  */
#define SP2508_PIDL_MAGIC 0x08

static struct sp_camera_module sp2508;

#define SP2508_FLIP_REG                      0x3f


#define SP2508_SW_STREAM				0xac
#define SP2508_START_STREAMING			0x01
#define SP2508_STOP_STREAMING			0x00


/* ======================================================================== */
/* Base sensor configs */
/* ======================================================================== */

/* ======================================================================== */
static const struct sp_camera_module_reg sp2508_init_tab_1616_1216_15fps_vfifo[] = {
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xfd,0x00},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x35,0x20}, //pll bias
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x2f,0x0c}, //pll clk 57.6M  19.2m mclk
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x1c,0x03}, //pull down parallel pad
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xfd,0x01},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x03,0x01}, //exp time, 3 base
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x04,0x37},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x06,0xa0}, //vblank //0x10
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x24,0xf0}, //pga gain 10x
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x01,0x01}, //enable reg write
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x2b,0xc4}, //readout vref
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x2e,0x20}, //dclk delay
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x79,0x42}, //p39 p40
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x85,0x0f}, //p51
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x09,0x01}, //hblank
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x0a,0x40},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x21,0xef}, //pcp tx 4.05v
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x25,0xf2}, //reg dac 2.7v, enable bl_en,vbl 1.4v
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x26,0x00}, //vref2 1v, disable ramp driver
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x2a,0xea}, //bypass dac res, adc range 0.745, vreg counter 0.9
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x2c,0xf0}, //high 8bit, pldo 2.7v
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x8a,0x55}, //pixel bias 2uA
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x8b,0x55}, 
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x19,0xf3}, //icom1 1.7u, icom2 0.6u 
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x11,0x30}, //rst num
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xd0,0x01}, //boost2 enable
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xd1,0x01}, //boost2 start point h'1do
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xd2,0xd0},  
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x55,0x10},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x58,0x30},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x5d,0x15},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x5e,0x05},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x64,0x40},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x65,0x00},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x66,0x66},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x67,0x00},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x68,0x68},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x72,0x70},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xfb,0x25},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xf0,0x08}, //offset
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xf1,0x08},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xf2,0x08},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xf3,0x08},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xfd,0x01}, //mipi
	//{SP_CAMERA_MODULE_REG_TYPE_DATA,0xcc,0x01},//mipi 信号稳定
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xb3,0x00},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x93,0x01},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x9d,0x17},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xc5,0x01},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xc6,0x00},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xb1,0x01},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x8e,0x06},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x8f,0x50},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x90,0x04},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x91,0xc0},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x92,0x01},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xa1,0x05},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xaa,0x00},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xfd,0x02},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x37,0x00},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x38,0x00}, //v-start
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x39,0x04}, 
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x3a,0xb0},//v-value  //1216
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x3b,0x00}, 
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x3c,0x00},//h-start
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x3d,0x03},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x3e,0x20}, //h-value 的一半//1616
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0xfd,0x01},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x8e,0x06},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x8f,0x40}, //1616
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x90,0x04},
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x91,0xb0}, //1216
	{SP_CAMERA_MODULE_REG_TYPE_DATA,0x3f,0x03},	 
};

static struct sp_camera_module_config sp2508_configs[] = {
	{
		.name    = "1600x1200_15fps",
		.frm_fmt = {
			.width  = 1600,
			.height = 1200,
			.code   = V4L2_MBUS_FMT_SRGGB10_1X10
		},
		.frm_intrvl = {
			.interval = {
				.numerator     = 1,
				.denominator   = 20   //15 fps
			}
		},
		.auto_exp_enabled      = false,
		.auto_gain_enabled     = false,
		.auto_wb_enabled       = false,
		.reg_table             = (void *)sp2508_init_tab_1616_1216_15fps_vfifo,
		.reg_table_num_entries =
			sizeof(sp2508_init_tab_1616_1216_15fps_vfifo)
			/
			sizeof(sp2508_init_tab_1616_1216_15fps_vfifo[0]),
		.v_blanking_time_us    = 5227 /*empirically measured time*///1227  //Vblank
	},
};

static int sp2508_write_aec(struct sp_camera_module *cam_mod)
{
	int ret      = 0;
	u32 a_gain   = 0;
	u32 exp_time = 0;
	u8 tmp,tmp1;
	u16 tmp2;

	sp_camera_module_pr_debug(cam_mod,
		"exp_time = %d, gain = %d, flash_mode = %d\n",
		cam_mod->exp_config.exp_time,
		cam_mod->exp_config.gain,
		cam_mod->exp_config.flash_mode);

	/* if the sensor is already streaming, write to shadow registers,
		if the sensor is in SW standby, write to active registers,
		if the sensor is off/registers are not writeable, do nothing */
		
	if ((cam_mod->state == SP_CAMERA_MODULE_SW_STANDBY) ||
		(cam_mod->state == SP_CAMERA_MODULE_STREAMING)) {
		a_gain   = cam_mod->exp_config.gain;
		exp_time = cam_mod->exp_config.exp_time;

		sp_camera_module_write_reg(cam_mod, 0xfd, 0x01);

		tmp = exp_time;
		tmp1 = exp_time >> 8;
		sp_camera_module_write_reg(cam_mod, 0x03, tmp1);
		sp_camera_module_write_reg(cam_mod, 0x04, tmp);
	
		tmp = a_gain < 0x10?0x10:a_gain>0xa0?0xa0:a_gain;

		sp_camera_module_write_reg(cam_mod, 0x24, tmp);
		sp_camera_module_write_reg(cam_mod, 0x01, 0x01);
		
	}

	if (IS_ERR_VALUE(ret)) {
		sp_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	}
	return ret;
}

static int sp2508_flip(struct sp_camera_module *cam_mod)
{
	int ret = -EAGAIN;

	if (cam_mod->state == SP_CAMERA_MODULE_SW_STANDBY &&
			cam_mod->update_config == true) 
	{

		u8 reg_val;

		sp_camera_module_write_reg(cam_mod, 0xfd, 0x01);

		ret = sp_camera_module_read_reg(cam_mod, 1,
						SP2508_FLIP_REG, &reg_val);

		if (cam_mod->vflip)
			;
		if (cam_mod->hflip)
			;
	}
	else
		ret = 0;


	if (IS_ERR_VALUE(ret))
		sp_camera_module_pr_err(cam_mod,
					"failed with error (%d)\n", ret);

	return ret;
}

static int sp2508_g_ctrl(struct sp_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	sp_camera_module_pr_debug(cam_mod, "\n");

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

	if (IS_ERR_VALUE(ret)) {
		sp_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);
	}
	return ret;
}

static int sp2508_g_timings(struct sp_camera_module *cam_mod,
	struct sp_camera_module_timings *timings)
{
	int ret            = 0;
	u8 reg_val        = 0;

	if (IS_ERR_OR_NULL(cam_mod->active_config)) {
		goto err;
	}

	timings->vt_pix_clk_freq_hz = 78000000;
	timings->line_length_pck = 2316;
	timings->frame_length_lines = 1241+ 144;
	
	timings->coarse_integration_time_min        = 1;
	timings->coarse_integration_time_max_margin = SP2508_INTEGRATION_TIME_MARGIN;

	/* SP Sensor do not use fine integration time. */
	timings->fine_integration_time_min          = 0;
	timings->fine_integration_time_max_margin   = 0;
	
	timings->binning_factor_x = 1;
	timings->binning_factor_y = 1;

	 sp_camera_module_write_reg(cam_mod, 0xfd, 0x02);

	/* Get the cropping and output resolution to ISP for this mode. */

	sp_camera_module_read_reg(cam_mod, 1, SP2508_HORIZONTAL_START_HIGH_REG, &reg_val);
	timings->crop_horizontal_start = reg_val << 8;

	sp_camera_module_read_reg(cam_mod, 1, SP2508_HORIZONTAL_START_LOW_REG, &reg_val);
	timings->crop_horizontal_start |= reg_val;

	sp_camera_module_read_reg(cam_mod, 1, SP2508_VERTICAL_START_HIGH_REG, &reg_val);
	timings->crop_vertical_start = reg_val << 8;

	sp_camera_module_read_reg(cam_mod, 1, SP2508_VERTICAL_START_LOW_REG, &reg_val);
	timings->crop_vertical_start |= reg_val;


	sp_camera_module_read_reg(cam_mod, 1, SP2508_HORIZONTAL_OUTPUT_SIZE_HIGH_REG, &reg_val);
	timings->sensor_output_width = reg_val << 8;

	sp_camera_module_read_reg(cam_mod, 1, SP2508_HORIZONTAL_OUTPUT_SIZE_LOW_REG, &reg_val);
	timings->sensor_output_width |= reg_val;
	timings->sensor_output_width *= 2;

	timings->sensor_output_width = 1600;


	sp_camera_module_read_reg(cam_mod, 1, SP2508_VERTICAL_OUTPUT_SIZE_HIGH_REG, &reg_val);
	timings->sensor_output_height = reg_val << 8;

	sp_camera_module_read_reg(cam_mod, 1, SP2508_VERTICAL_OUTPUT_SIZE_LOW_REG, &reg_val);
	timings->sensor_output_height |= reg_val;

	timings->sensor_output_height = 1200;

	if(timings->binning_factor_x ==1 )
	{
		timings->crop_horizontal_end = timings->sensor_output_width+ timings->crop_horizontal_start-1;
		timings->crop_vertical_end = timings->sensor_output_height + timings->crop_vertical_start-1;
	}
	else if(timings->binning_factor_x ==1 ) //binning
	{
		timings->crop_horizontal_end = timings->sensor_output_width *2+ timings->crop_horizontal_start-1+16;
		timings->crop_vertical_end = timings->sensor_output_height*2 + timings->crop_vertical_start-1+4;

	}

	sp_camera_module_pr_debug(cam_mod, "sp2508_g_timings %d,%d,%d,%d,%d,%d\n", 
	    timings->crop_horizontal_start,timings->crop_vertical_start, 
	    timings->sensor_output_width,timings->sensor_output_height,
	    timings->crop_horizontal_end,timings->crop_vertical_end);

	return ret;

err:
	sp_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

static int sp2508_s_ctrl(struct sp_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	sp_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
		case V4L2_CID_GAIN:
		case V4L2_CID_EXPOSURE:
			ret = sp2508_write_aec(cam_mod);
			break;
		case V4L2_CID_FLASH_LED_MODE:
			/* nothing to be done here */
			break;
		case V4L2_CID_HFLIP:
		case V4L2_CID_VFLIP:
			ret = sp2508_flip(cam_mod);
			break;
		default:
			ret = -EINVAL;
			break;
	}

	if (IS_ERR_VALUE(ret)) {
		sp_camera_module_pr_debug(cam_mod,
			"failed with error (%d) 0x%x\n", ret, ctrl_id);
	}
	return ret;
}

static int sp2508_s_ext_ctrls(struct sp_camera_module *cam_mod,
				 struct sp_camera_module_ext_ctrls *ctrls)
{
	int ret = 0;

	/* Handles only exposure and gain together special case. */
	if (ctrls->count == 1) {
		ret = sp2508_s_ctrl(cam_mod, ctrls->ctrls[0].id);
	} else if (ctrls->count == 2 &&
		((ctrls->ctrls[0].id == V4L2_CID_GAIN &&
		ctrls->ctrls[1].id == V4L2_CID_EXPOSURE) ||
		(ctrls->ctrls[1].id == V4L2_CID_GAIN &&
		ctrls->ctrls[0].id == V4L2_CID_EXPOSURE))) {
		ret = sp2508_write_aec(cam_mod);
	} else {
		ret = -EINVAL;
	}

	if (IS_ERR_VALUE(ret)) {
		sp_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);
	}
	return ret;
}

static int sp2508_start_streaming(struct sp_camera_module *cam_mod)
{
	int ret = 0;
	int i=0;
	int j =0;

	sp_camera_module_pr_debug(cam_mod, "\n");

	/*apply otp data*/

	ret = sp2508_write_aec(cam_mod);
	if (IS_ERR_VALUE(ret)) {
		goto err;
	}

	ret = sp2508_flip(cam_mod);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = sp_camera_module_write_reg(cam_mod, 0xfd, 0x01);
    //ret = sp_camera_module_write_reg(cam_mod, 0xb1, 0x01);
	ret = sp_camera_module_write_reg(cam_mod, SP2508_SW_STREAM, SP2508_START_STREAMING);
    //ret = sp_camera_module_write_reg(cam_mod, 0xe7, 0x03);
    //ret = sp_camera_module_write_reg(cam_mod, 0xe7, 0x00);

	udelay(10000);

	if (IS_ERR_VALUE(ret)) {
		goto err;
	}

	return 0;

err:
	sp_camera_module_pr_err(cam_mod, "failed with error (%d)\n",
		ret);
	return ret;
}

static int sp2508_stop_streaming(struct sp_camera_module *cam_mod)
{
	int ret = 0;

	sp_camera_module_pr_debug(cam_mod, "\n");
	
	ret = sp_camera_module_write_reg(cam_mod, 0xfd, 0x01);
    //ret = sp_camera_module_write_reg(cam_mod, 0xb1, 0x00);
	//sp_camera_module_write_reg(cam_mod, 0xe7, 0x03);
	//sp_camera_module_write_reg(cam_mod, 0xe7, 0x00);

	ret =  sp_camera_module_write_reg(cam_mod, SP2508_SW_STREAM, SP2508_STOP_STREAMING);
	if (IS_ERR_VALUE(ret)) {
		goto err;
	}
	return 0;

err:
	sp_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

static int sp2508_check_camera_id(struct sp_camera_module *cam_mod)
{
	u8 pidh, pidl;
	int ret = 0;

	sp_camera_module_pr_debug(cam_mod, "\n");

	ret |= sp_camera_module_write_reg(cam_mod, 0xfd, 0x00);
	ret |= sp_camera_module_read_reg(cam_mod, 1, SP2508_PIDH_ADDR, &pidh);
	ret |= sp_camera_module_read_reg(cam_mod, 1, SP2508_PIDL_ADDR, &pidl);
	
	if (IS_ERR_VALUE(ret)) {
		sp_camera_module_pr_err(cam_mod,
			"register read failed, camera module powered off?\n");
		goto err;
	}

	if ((pidh == SP2508_PIDH_MAGIC) && (pidl == SP2508_PIDL_MAGIC)) {
		sp_camera_module_pr_debug(cam_mod,
				"successfully detected camera ID 0x%02x%02x\n",
			pidh, pidl);
	} else {
		sp_camera_module_pr_err(cam_mod,
				"wrong camera ID, expected 0x%02x%02x, detected 0x%02x%02x\n",
			SP2508_PIDH_MAGIC, SP2508_PIDL_MAGIC, pidh, pidl);
		ret = -EINVAL;
		goto err;
	}

	return 0;

err:
	sp_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/* ======================================================================== */
/* This part is platform dependent */
/* ======================================================================== */

static struct v4l2_subdev_core_ops sp2508_camera_module_core_ops = {
	.g_ctrl              = sp_camera_module_g_ctrl,
	.s_ctrl              = sp_camera_module_s_ctrl,
	.s_ext_ctrls         = sp_camera_module_s_ext_ctrls,
	.s_power             = sp_camera_module_s_power,
	.ioctl               = sp_camera_module_ioctl
};

static struct v4l2_subdev_video_ops sp2508_camera_module_video_ops = {
	.enum_frameintervals = sp_camera_module_enum_frameintervals,
	.s_mbus_fmt          = sp_camera_module_s_fmt,
	.g_mbus_fmt          = sp_camera_module_g_fmt,
	.try_mbus_fmt        = sp_camera_module_try_fmt,
	.s_frame_interval    = sp_camera_module_s_frame_interval,
	.s_stream            = sp_camera_module_s_stream
};

static struct v4l2_subdev_ops sp2508_camera_module_ops = {
	.core                = &sp2508_camera_module_core_ops,
	.video               = &sp2508_camera_module_video_ops
};

static struct sp_camera_module_custom_config sp2508_custom_config = {
	.start_streaming    = sp2508_start_streaming,
	.stop_streaming     = sp2508_stop_streaming,
	.s_ctrl             = sp2508_s_ctrl,
	.s_ext_ctrls        = sp2508_s_ext_ctrls,
	.g_ctrl             = sp2508_g_ctrl,
	.g_timings          = sp2508_g_timings,
	.check_camera_id    = sp2508_check_camera_id,
	.configs            = sp2508_configs,
	.num_configs        = sizeof(sp2508_configs) / sizeof(sp2508_configs[0]),
	.power_up_delays_ms = {5, 20, 0}
};

static int __init sp2508_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;

	dev_info(&client->dev, "probing...\n");

	v4l2_i2c_subdev_init(&sp2508.sd, client, &sp2508_camera_module_ops);

	ret = sp_camera_module_init(&sp2508,
			&sp2508_custom_config);
	if (IS_ERR_VALUE(ret)) {
		goto err;
	}

	sp_camera_module_s_power(&sp2508.sd, 1);

	dev_info(&client->dev, "get otp data for r2a...\n");

	sp_camera_module_s_power(&sp2508.sd, 0);

	dev_info(&client->dev, "probing successful\n");
	return 0;

err:
	dev_err(&client->dev, "probing failed with error (%d)\n", ret);
	return ret;
}

/* ======================================================================== */

static int __exit sp2508_remove(
	struct i2c_client *client)
{
	struct sp_camera_module *cam_mod = i2c_get_clientdata(client);

	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter) {
		return -ENODEV;	/* our client isn't attached */
	}

	sp_camera_module_release(cam_mod);


	dev_info(&client->dev, "removed\n");

	return 0;
}

static const struct i2c_device_id sp2508_id[] = {
	{ SP2508_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id sp2508_of_match[] = {
	{.compatible = "superpix," SP2508_DRIVER_NAME "-v4l2-i2c-subdev",},
	{},
};

MODULE_DEVICE_TABLE(i2c, sp2508_id);

static struct i2c_driver sp2508_i2c_driver = {
	.driver = {
		.name           = SP2508_DRIVER_NAME,
		.owner          = THIS_MODULE,
		.of_match_table = sp2508_of_match
	},
	.probe              = sp2508_probe,
	.remove             = __exit_p(sp2508_remove),
	.id_table           = sp2508_id,
};

module_i2c_driver(sp2508_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for sp2508");
MODULE_AUTHOR("Kunkka Lu");
MODULE_LICENSE("GPL");
