/*
 * drivers/media/i2c/soc_camera/xgold/lm3642.c
 *
 * lm3642 flash controller driver
 *
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Note:
 *    23/01/2015: initial version
 */

#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-core.h>

#define LM3642_DRIVER_NAME "lm3642"

/* registers */
#define REG_ENABLE		0x0A
#define REG_FLASH_CUR   0x09
#define REG_FLASH_TOUT	0x08
#define REG_FLAG		0x0b

/*REG_ENABLE*/
/*
b00:standby
b01:indicator
b10:torch
b11:flash
*/
#define REG_ENABLE_MODE_BIT_SHIFT       0x0
#define REG_ENABLE_MODE_MASK            0x03

#define REG_ENABLE_TORCH_PIN_BIT_SHIFT  0x4
#define REG_ENABLE_TORCH_PIN_MASK       0x10

#define REG_ENABLE_FLASH_PIN_BIT_SHIFT  0x5
#define REG_ENABLE_FLASH_PIN_MASK       0x20

/*REG_FLASH_CUR*/
/*
FlashCurrent
0000=93.75mA
0001=187.5mA
0010=281.25mA
0011=375mA
0100=468.75mA
0101=562.5mA
0110=656.25mA
0111=750mA
1000=843.75mA
1001=937.5mA
1010=1031.25mA
1011=1125mA
1100=1218.75mA
1101=1312.5mA
1110=1406.25mA
1111=1500mA(default)
*/
#define REG_FLASH_CUR_BIT_SHIFT         0x0
#define REG_FLASH_CUR_MASK              0x0f

/*
TorchCurrent(LM3642LT)
000=48.4mA(default)
001=93.74mA(46.87mA)
010=140.63mA(70.315mA)
011=187.5mA(93.25mA)
100=234.38mA(117.19mA)
101=281.25mA(140.625mA)
110=328.13mA(164.075mA)
111=375mA(187.5mA)
*/
#define REG_TORCH_CUR_BIT_SHIFT         0x4
#define REG_TORCH_CUR_MASK              0xf0

/*REG_FLASH_TOUT*/
/*
000:100ms
001:200ms
010:300ms(default)
011:400ms
100:500ms
101:600ms
110:700ms
111:800ms
*/
#define REG_FLASH_TOUT_BIT_SHIFT         0x0
#define REG_FLASH_TOUT_MASK              0x07

static const char *fl_cur_string[] = {
"93.75mA",
"187.5mA",
"281.25mA",
"375mA",
"468.75mA",
"562.5mA",
"656.25mA",
"750mA",
"843.75mA",
"937.5mA",
"1031.25mA",
"1125mA",
"1218.75mA",
"1312.5mA",
"1406.25mA",
"1500mA"
};

static const char *fl_tout_string[] = {
"100ms",
"200ms",
"300ms",
"400ms",
"500ms",
"600ms",
"700ms",
"800ms"
};

/* ======================================================================== */

struct lm3642_dev {
		struct v4l2_subdev sd;
		enum v4l2_flash_led_mode mode;
		u16 fl_timeout;
		u16 fl_intensity;
		u16 torch_intensity;
};

/* ======================================================================== */

int lm3642_write_msg(
	struct i2c_client *client,
	u8 msb, u8 lsb)
{
	int ret = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retries;

	if (!client->adapter) {
		dev_err(&client->dev, "client->adapter NULL\n");
		return -ENODEV;
	}

	for (retries = 0; retries < 5; retries++) {
		msg->addr = client->addr;
		msg->flags = 0;
		msg->len = 2;
		msg->buf = data;

		data[0] = msb;
		data[1] = lsb;
		ret = i2c_transfer(client->adapter, msg, 1);
		usleep_range(50, 51);

		if (ret == 1)
			return 0;

		dev_dbg(&client->dev,
			"retrying I2C... %d\n", retries);
		retries++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
	}
	dev_err(&client->dev,
		"i2c write to failed with error %d\n", ret);
	return ret;
}

/* ======================================================================== */

static int lm3642_g_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct lm3642_dev *dev = container_of(sd, struct lm3642_dev, sd);

	switch (ctrl->id) {
	case V4L2_CID_FLASH_LED_MODE:
		ctrl->value = dev->mode;
		break;
	case V4L2_CID_FLASH_TIMEOUT:
		ctrl->value = dev->fl_timeout;
		break;
	case V4L2_CID_FLASH_INTENSITY:
		ctrl->value = dev->fl_intensity;
		break;
	case V4L2_CID_FLASH_TORCH_INTENSITY:
		ctrl->value = dev->torch_intensity;
		break;
	case V4L2_CID_FLASH_FAULT:
		break;
	default:
		ret = -EINVAL;
		dev_err(&client->dev,
			"not support ctrl %d\n", ctrl->id);
		break;
	}

	return ret;
}

/* ======================================================================== */

static int lm3642_s_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	int ret = 0;
	u8 lsb = 0;
	u8 msb = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct lm3642_dev *dev = container_of(sd, struct lm3642_dev, sd);

	switch (ctrl->id) {
	case V4L2_CID_FLASH_LED_MODE:
		if ((ctrl->value) == V4L2_FLASH_LED_MODE_NONE) {
			/*stand by*/
			msb = REG_ENABLE;
			lsb = REG_ENABLE_MODE_MASK & (0x0 << REG_ENABLE_MODE_BIT_SHIFT);
			dev->mode = V4L2_FLASH_LED_MODE_NONE;
			dev_dbg(&client->dev, "mode V4L2_FLASH_LED_MODE_NONE\n");
		} else if ((ctrl->value) == V4L2_FLASH_LED_MODE_FLASH) {
			/* for test: set max fl timeout and max  INTENSITY */
			struct v4l2_control ctrl;

			ctrl.id = V4L2_CID_FLASH_TIMEOUT;
			ctrl.value = 0x7;

			lm3642_s_ctrl(sd, &ctrl);
			ctrl.id = V4L2_CID_FLASH_INTENSITY;
			ctrl.value = 0xf;
			lm3642_s_ctrl(sd, &ctrl);

			msb = REG_ENABLE;
			lsb = (REG_ENABLE_MODE_MASK & (0x3 << REG_ENABLE_MODE_BIT_SHIFT))
					|(REG_ENABLE_FLASH_PIN_MASK & (0x1 << REG_ENABLE_FLASH_PIN_BIT_SHIFT));
			dev->mode = V4L2_FLASH_LED_MODE_FLASH;
			dev_dbg(&client->dev, "mode V4L2_FLASH_LED_MODE_FLASH\n");
		} else if ((ctrl->value) == V4L2_FLASH_LED_MODE_TORCH) {
			msb = REG_ENABLE;
			lsb = (REG_ENABLE_MODE_MASK & (0x2 << REG_ENABLE_MODE_BIT_SHIFT))
					|(REG_ENABLE_TORCH_PIN_MASK & (0x1 << REG_ENABLE_TORCH_PIN_BIT_SHIFT));
			dev->mode = V4L2_FLASH_LED_MODE_TORCH;
			dev_dbg(&client->dev, "mode V4L2_FLASH_LED_MODE_TORCH\n");
		} else {
			ret = -EINVAL;
			dev_dbg(&client->dev,
				"not support ctrl %d\n", ctrl->id);
		}
		break;
	case V4L2_CID_FLASH_TIMEOUT:
		if ((ctrl->value) > 0x7) {
			ret = -EINVAL;
			dev_err(&client->dev,
				"wrong flash timout %d\n", ctrl->value);
		} else {
			msb = REG_FLASH_TOUT;
			lsb = REG_FLASH_TOUT_MASK & ((u8)(ctrl->value) << REG_FLASH_TOUT_BIT_SHIFT);
			dev->fl_timeout = ctrl->value;
			dev_dbg(&client->dev,
				"V4L2_CID_FLASH_TIMEOUT %s\n", fl_tout_string[ctrl->value]);
		}
		break;
	case V4L2_CID_FLASH_INTENSITY:
		if ((ctrl->value) > 0xf) {
			ret = -EINVAL;
			dev_err(&client->dev,
				"wrong flash brightness %d\n", ctrl->value);
		} else {
			msb = REG_FLASH_CUR;
			lsb = REG_FLASH_CUR_MASK & ((u8)(ctrl->value) << REG_FLASH_CUR_BIT_SHIFT);
			dev->fl_intensity = ctrl->value;
			dev_dbg(&client->dev,
				"V4L2_CID_FLASH_INTENSITY %s\n", fl_cur_string[ctrl->value]);
		}
		break;
	case V4L2_CID_FLASH_TORCH_INTENSITY:
		break;
	default:
		ret = -EINVAL;
		dev_err(&client->dev,
			"not support ctrl %d\n", ctrl->id);
		break;
	}

	if (ret != 0) {
		dev_err(&client->dev,
			"failed with error %d\n", ret);
	} else {
		ret = lm3642_write_msg(client, msb, lsb);
	}
	return ret;
}

/* ======================================================================== */
static struct v4l2_subdev_core_ops lm3642_core_ops = {
	.g_ctrl = lm3642_g_ctrl,
	.s_ctrl = lm3642_s_ctrl,
};

static struct v4l2_subdev_ops lm3642_ops = {
	.core = &lm3642_core_ops,
};

static int lm3642_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct lm3642_dev *dev;

	dev_info(&client->dev, "probing...\n");

	dev = devm_kzalloc(&client->dev, sizeof(struct lm3642_dev), GFP_KERNEL);
	if (dev == NULL)
		return -ENOMEM;
	v4l2_i2c_subdev_init(&dev->sd, client, &lm3642_ops);
	dev_info(&client->dev, "probing successful\n");
	return 0;
}

/* ======================================================================== */

static int __exit lm3642_remove(
	struct i2c_client *client)
{
	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	dev_info(&client->dev, "removed\n");
	return 0;
}

static const struct i2c_device_id lm3642_id[] = {
	{ LM3642_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id lm3642_of_match[] = {
	{.compatible = "TI,lm3642-v4l2-i2c-subdev"}
};

MODULE_DEVICE_TABLE(i2c, lm3642_id);

static struct i2c_driver lm3642_i2c_driver = {
	.driver = {
		.name = LM3642_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3642_of_match
	},
	.probe = lm3642_probe,
	.remove = __exit_p(lm3642_remove),
	.id_table = lm3642_id,
};

module_i2c_driver(lm3642_i2c_driver);

MODULE_DESCRIPTION("LM3642 flash ligher controller driver");
MODULE_AUTHOR("zyc");
MODULE_LICENSE("GPL");
