/*
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 * Copyright (C) 2014 Intel Mobile Communications GmbH
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
 *
 *	note: only support mulititouch	Wenfs 2010-10-01
 */

#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/i2c/ft5x06_ts.h>

#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/input/mt.h>
//#include "ft5x06_ex_fun.h"

#define CONFIG_FT5X0X_MULTITOUCH 1
#define CFG_MAX_TOUCH_POINTS	5
static struct i2c_client *this_client;


struct ts_event {
	u16 x1;
	u16 y1;
	u16 x2;
	u16 y2;
	u16 x3;
	u16 y3;
	u16 x4;
	u16 y4;
	u16 x5;
	u16 y5;
	u16 pressure;
	s16 touch_ID1;
	s16 touch_ID2;
	s16 touch_ID3;
	s16 touch_ID4;
	s16 touch_ID5;
	u8 touch_point;
#if 1
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:
					0 -- down; 1-- contact; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
#endif
};

 typedef enum
 {
	   ERR_OK,
	   ERR_MODE,
	   ERR_READID,
	   ERR_ERASE,
	   ERR_STATUS,
	   ERR_ECC,
	   ERR_DL_ERASE_FAIL,
	   ERR_DL_PROGRAM_FAIL,
	   ERR_DL_VERIFY_FAIL
 }E_UPGRADE_ERR_TYPE;


struct ft5x0x_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	struct delayed_work pen_event_work;
	struct workqueue_struct *ts_workqueue;
	spinlock_t btn_lock;
	spinlock_t en_lock;
	int btn_active;
	int enable;
};

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
struct device_state_pm_state ft5x0x_pm_states[] = {
	{.name = "disable", }, /* D3 */
	{.name = "enable", }, /* D0 */
};

#define FT5X0X_STATE_D0		1
#define FT5X0X_STATE_D3		0

/* Touchscreen PM states & class */
static int ft5x0x_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	struct ft5x0x_ts_platform_data *ft5x06_pdata = dev->platform_data;
	int id = device_state_pm_get_state_id(dev, state->name);
	int ret = 0;

	switch (id) {
	case FT5X0X_STATE_D0:
		if (ft5x06_pdata->power)
			ret = regulator_enable(ft5x06_pdata->power);
		if (ret)
			return ret;
		mdelay(50);

		if (ft5x06_pdata->power2)
			ret = regulator_enable(ft5x06_pdata->power2);
		if (ret)
			return ret;
		mdelay(50);

		break;

	case FT5X0X_STATE_D3:
		if (ft5x06_pdata->power)
			regulator_disable(ft5x06_pdata->power);

		if (ft5x06_pdata->power2)
			regulator_disable(ft5x06_pdata->power2);

		break;

	default:
		return id;
	}

	return 0;
}

static struct device_state_pm_state *ft5x0x_get_initial_state(
		struct device *dev)
{
	return &ft5x0x_pm_states[FT5X0X_STATE_D3];
}

static struct device_state_pm_ops ft5x0x_pm_ops = {
	.set_state = ft5x0x_set_pm_state,
	.get_initial_state = ft5x0x_get_initial_state,
};

DECLARE_DEVICE_STATE_PM_CLASS(ft5x0x);
#endif

#ifdef CONFIG_OF

#define OF_FT5X0X_SUPPLY	"i2c"
#define OF_FT5X0X_SUPPLY_A	"i2ca"
#define OF_FT5X0X_PIN_RESET	"intel,ts-gpio-reset"

static struct ft5x0x_ts_platform_data *ft5x0x_ts_of_get_platdata(
		struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct ft5x0x_ts_platform_data *ft5x06_pdata;
	struct regulator *pow_reg;
	int ret;

	ft5x06_pdata = devm_kzalloc(&client->dev,
			sizeof(*ft5x06_pdata), GFP_KERNEL);
	if (!ft5x06_pdata)
		return ERR_PTR(-ENOMEM);

	/* regulator */
	pow_reg = regulator_get(&client->dev, OF_FT5X0X_SUPPLY);
	ft5x06_pdata->power = pow_reg;
	if (IS_ERR(pow_reg)) {
		dev_warn(&client->dev, "%s can't get %s-supply handle\n",
			np->name, OF_FT5X0X_SUPPLY);
		ft5x06_pdata->power = NULL;
	}

	pow_reg = regulator_get(&client->dev, OF_FT5X0X_SUPPLY_A);
	ft5x06_pdata->power2 = pow_reg;
	if (IS_ERR(pow_reg)) {
		dev_warn(&client->dev, "%s can't get %s-supply handle\n",
			np->name, OF_FT5X0X_SUPPLY_A);
		ft5x06_pdata->power2 = NULL;
	}

	/* pinctrl */
	ft5x06_pdata->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(ft5x06_pdata->pinctrl)) {
		ret = PTR_ERR(ft5x06_pdata->pinctrl);
		goto out;
	}

	ft5x06_pdata->pins_default = pinctrl_lookup_state(
			ft5x06_pdata->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(ft5x06_pdata->pins_default))
		dev_err(&client->dev, "could not get default pinstate\n");

	ft5x06_pdata->pins_sleep = pinctrl_lookup_state(
			ft5x06_pdata->pinctrl, PINCTRL_STATE_SLEEP);
	if (IS_ERR(ft5x06_pdata->pins_sleep))
		dev_err(&client->dev, "could not get sleep pinstate\n");

	ft5x06_pdata->pins_inactive = pinctrl_lookup_state(
			ft5x06_pdata->pinctrl, "inactive");
	if (IS_ERR(ft5x06_pdata->pins_inactive))
		dev_err(&client->dev, "could not get inactive pinstate\n");

	/* gpio reset */
	ft5x06_pdata->reset_pin = of_get_named_gpio_flags(client->dev.of_node,
			OF_FT5X0X_PIN_RESET, 0, NULL);
	if (ft5x06_pdata->reset_pin <= 0) {
		dev_err(&client->dev,
			"error getting gpio for %s\n", OF_FT5X0X_PIN_RESET);
		ret = -EINVAL;
		goto out;
	}

	/* interrupt mode */
	if (of_property_read_bool(np, "intel,polling-mode"))
		ft5x06_pdata->polling_mode = true;

	/* pm */
	ft5x06_pdata->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(ft5x06_pdata->pm_platdata)) {
		dev_warn(&client->dev, "Error during device state pm init\n");
		ret = PTR_ERR(ft5x06_pdata->pm_platdata);
		goto out;
	}

	return ft5x06_pdata;

out:
	return ERR_PTR(ret);
}
#endif

static inline int ft5x0x_set_pinctrl_state(struct device *dev,
		struct pinctrl_state *state)
{
	struct ft5x0x_ts_platform_data *pdata = dev_get_platdata(dev);
	int ret = 0;

	if (!IS_ERR(state)) {
		ret = pinctrl_select_state(pdata->pinctrl, state);
		if (ret)
			dev_err(dev, "%d:could not set pins\n", __LINE__);
	}

	return ret;
}

static int ft5x0x_ts_power_off(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);
	struct ft5x0x_ts_platform_data *ft5x06_pdata =
		client->dev.platform_data;
	unsigned long flags = 0;
	int ret = 0;

	spin_lock_irqsave(&ft5x0x_ts->en_lock, flags);
	if (!ft5x0x_ts->enable)
		goto out;

	disable_irq_nosync(client->irq);
	cancel_delayed_work_sync(&ft5x0x_ts->pen_event_work);
	if (ft5x06_pdata->pins_sleep)
		ft5x0x_set_pinctrl_state(&client->dev,
				ft5x06_pdata->pins_sleep);

	if (ft5x06_pdata->pm_platdata &&
			ft5x06_pdata->pm_platdata->pm_state_D3_name) {
		ret = device_state_pm_set_state_by_name(&client->dev,
				ft5x06_pdata->pm_platdata->pm_state_D3_name);
	}

	if (!ret)
		ft5x0x_ts->enable = 0;

out:
	spin_unlock_irqrestore(&ft5x0x_ts->en_lock, flags);

	return ret;
}

static int ft5x0x_ts_power_on(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);
	struct ft5x0x_ts_platform_data *ft5x06_pdata =
		client->dev.platform_data;
	unsigned long flags = 0;
	int ret = 0;

	spin_lock_irqsave(&ft5x0x_ts->en_lock, flags);
	if (ft5x0x_ts->enable)
		goto out;

	if (ft5x06_pdata->pins_default)
		ret = ft5x0x_set_pinctrl_state(&client->dev,
					ft5x06_pdata->pins_default);

	if (ft5x06_pdata->pm_platdata &&
			ft5x06_pdata->pm_platdata->pm_state_D0_name)
		ret = device_state_pm_set_state_by_name(&client->dev,
			ft5x06_pdata->pm_platdata->pm_state_D0_name);

	if (!ret) {
		enable_irq(client->irq);
		ft5x0x_ts->enable++;
	}

out:
	spin_unlock_irqrestore(&ft5x0x_ts->en_lock, flags);

	return ret;
}

/*
 * SysFS support
 */

static ssize_t ft5x0x_ts_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);

	dev_dbg(dev, "%s\n", __func__);

	return sprintf(buf, "%d\n", ft5x0x_ts->enable);
}

static ssize_t ft5x0x_ts_store_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (ft5x0x_ts->enable == val)
		return count;

	switch (val) {
	case 0:
		ft5x0x_ts_power_off(client);
		break;
	case 1:
		ft5x0x_ts_power_on(client);
		break;
	default:
		dev_err(&client->dev, "%s: unsuppored value %ld\n",
				__func__, val);
		return count;
	}

	dev_info(dev, "%s\n", (val == 1) ? "enable" : "disable");

	return count;
}

static DEVICE_ATTR(enable, 0777,
		ft5x0x_ts_show_enable,
		ft5x0x_ts_store_enable);

static struct attribute *ft5x0x_ts_attributes[] = {
	&dev_attr_enable.attr,
	NULL
};

static const struct attribute_group ft5x0x_ts_attr_group = {
	.attrs = ft5x0x_ts_attributes,
};

static int ft5x0x_i2c_rxdata(struct i2c_client *client, char *buf, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = buf,
		 },
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	return ret;
}

static int ft5x0x_set_u8(const struct i2c_client *client, u8 addr, u8 value)
{
	int ret = i2c_smbus_write_byte_data(client, addr, value);
	if (ret < 0)
		pr_err("%s: 0x%08x failed\n", __func__, value);
	else
		pr_debug("sent byte [0x%02x] @0x%x\n", value, addr);
	return ret;
}

static u16 ft5x0x_read_u16(const struct i2c_client *client, u8 addr)
{
	u16 value = i2c_smbus_read_word_data(client, addr);
	if (value < 0) {
		pr_err("%s: read @0x%x failed\n", __func__, addr);
	} else {
		value = swab16(value);
		pr_debug("[0x%X] = %04X\n", addr, value);
	}
	return value;
}

static u8 ft5x0x_read_u8(const struct i2c_client *client, u8 addr)
{
	u8 value = i2c_smbus_read_byte_data(client, addr);
	if (value < 0) {
		pr_err("%s: read @0x%x failed\n", __func__, addr);
		return value;
	}
	pr_debug("[0x%X] = %02X\n", addr, value);
	return value;
}


static void ft5x0x_ts_release(struct i2c_client *client)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(client);

	pr_debug("ft5x0x_ts_release\n");
#ifdef CONFIG_FT5X0X_MULTITOUCH
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 0);
#else
	input_report_abs(data->input_dev, ABS_PRESSURE, 0);
	input_report_key(data->input_dev, BTN_TOUCH, 0);
#endif
	input_sync(data->input_dev);
}

static void ft5x0x_ts_inactivate(struct i2c_client *client)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(client);
	unsigned long flags = 0;

	dev_dbg(&client->dev, "%s\n", __func__);
	spin_lock_irqsave(&data->btn_lock, flags);
	if (data->btn_active) {
		input_event(data->input_dev, EV_KEY, KEY_BACK, 0);
		input_event(data->input_dev, EV_KEY, KEY_MENU, 0);
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		data->btn_active = 0;
	}
	spin_unlock_irqrestore(&data->btn_lock, flags);
	input_mt_sync(data->input_dev);
	input_sync(data->input_dev);
}

#define X_POS_MIN	0
#define X_POS_MAX	1200
#define Y_POS_MIN	0
#define Y_POS_MAX	1920

static u16 ft5x0x_convert_y(u16 value)
{
	//return (1024 - value) * 1920 /1024 ;//(value - Y_POS_MIN) * SCREEN_MAX_Y / (Y_POS_MAX - Y_POS_MIN);
	return value;
}

static u16 ft5x0x_convert_x(u16 value)
{
	return value ;//* 1200 /600;//(value - X_POS_MIN) * SCREEN_MAX_X / (X_POS_MAX - X_POS_MIN);
}

static bool ft5x0x_check_position(u16 x_val, u16 y_val)
{
	//if (x_val < X_POS_MIN || x_val > X_POS_MAX)
	//	return false;
	//if (y_val < Y_POS_MIN || y_val > Y_POS_MAX)
	//	return false;

	return true;
}

static bool ft5x0x_is_button(u16 x, u16 y)
{
	if (y == 900 && (x == 60 || x == 80 || x == 400 || x == 420))
		return true;

	return false;
}

static int ft5x0x_read_data(struct i2c_client *client)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(client);
	struct ts_event *event = &data->event;
	u8 buf[32] = { 0 };
	int ret = -1;
	int status = 0;

#ifdef CONFIG_FT5X0X_MULTITOUCH
	ret = ft5x0x_i2c_rxdata(client, buf, 31);
#else
	ret = ft5x0x_i2c_rxdata(client, buf, 7);
#endif
	if (ret < 0) {
		dev_err(&client->dev,
			"%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07;

	/* at least one touch point is detected.
	   Otherwise, in case of release, get position of latest active touch */
	event->x1 = (s16) (buf[3] & 0x0F) << 8 | (s16) buf[4];
	event->y1 = (s16) (buf[5] & 0x0F) << 8 | (s16) buf[6];

	if (event->touch_point == 0) {
		pr_debug("touch_point == 0 now inactivate\n");
		ft5x0x_ts_inactivate(client);
		return 1;
	}
#ifdef CONFIG_FT5X0X_MULTITOUCH
	switch (event->touch_point) {
	case 5:
		event->x5 = (s16) (buf[0x1b] & 0x0F) << 8 | (s16) buf[0x1c];
		event->y5 = (s16) (buf[0x1d] & 0x0F) << 8 | (s16) buf[0x1e];
		status = (s16) ((buf[0x1b] & 0xc0) >> 6);
		event->touch_ID5 = (s16) (buf[0x1D] & 0xF0) >> 4;
		if (status == 1)
			ft5x0x_ts_release(client);
	case 4:
		event->x4 = (s16) (buf[0x15] & 0x0F) << 8 | (s16) buf[0x16];
		event->y4 = (s16) (buf[0x17] & 0x0F) << 8 | (s16) buf[0x18];
		status = (s16) ((buf[0x15] & 0xc0) >> 6);
		event->touch_ID4 = (s16) (buf[0x17] & 0xF0) >> 4;
		if (status == 1)
			ft5x0x_ts_release(client);
	case 3:
		event->x3 = (s16) (buf[0x0f] & 0x0F) << 8 | (s16) buf[0x10];
		event->y3 = (s16) (buf[0x11] & 0x0F) << 8 | (s16) buf[0x12];
		status = (s16) ((buf[0x0f] & 0xc0) >> 6);
		event->touch_ID3 = (s16) (buf[0x11] & 0xF0) >> 4;
		if (status == 1)
			ft5x0x_ts_release(client);
	case 2:
		event->x2 = (s16) (buf[9] & 0x0F) << 8 | (s16) buf[10];
		event->y2 = (s16) (buf[11] & 0x0F) << 8 | (s16) buf[12];
		status = (s16) ((buf[0x9] & 0xc0) >> 6);
		event->touch_ID2 = (s16) (buf[0x0b] & 0xF0) >> 4;
		if (status == 1)
			ft5x0x_ts_release(client);
	case 1:
		event->x1 = (s16) (buf[3] & 0x0F) << 8 | (s16) buf[4];
		event->y1 = (s16) (buf[5] & 0x0F) << 8 | (s16) buf[6];
		status = (s16) ((buf[0x3] & 0xc0) >> 6);
		event->touch_ID1 = (s16) (buf[0x05] & 0xF0) >> 4;
		if (status == 1)
			ft5x0x_ts_release(client);
		break;
	default:
		pr_err("Unexpected touch point\n");
		return -1;
	}
#else
	if (event->touch_point == 1) {
		event->x1 = (s16) (buf[3] & 0x0F) << 8 | (s16) buf[4];
		event->y1 = (s16) (buf[5] & 0x0F) << 8 | (s16) buf[6];
	}
#endif
	event->pressure = 200;

	dev_dbg(&client->dev, "%s: 1:%d %d 2:%d %d\n", __func__,
		event->x1, event->y1, event->x2, event->y2);

	return 0;
}

static void ft5x0x_report_value(struct i2c_client *client)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(client);
	struct ts_event *event = &data->event;
#ifdef CONFIG_FT5X0X_MULTITOUCH
	unsigned long flags;
	int nbreport = 0;
#endif

	dev_dbg(&client->dev, "%s\n", __func__);
#ifdef CONFIG_FT5X0X_MULTITOUCH
	switch (event->touch_point) {
	case 5:
		if (ft5x0x_check_position(event->x5, event->y5)) {
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
					 event->touch_ID5);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 ft5x0x_convert_x(event->x5));
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 ft5x0x_convert_y(event->y5));
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR,
					 1);
			input_mt_sync(data->input_dev);
			dev_dbg(&client->dev, "=== x5 = %d, y5 = %d ===\n",
					event->x5, event->y5);
			nbreport++;
		}
	case 4:
		if (ft5x0x_check_position(event->x4, event->y4)) {
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
					 event->touch_ID4);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 ft5x0x_convert_x(event->x4));
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 ft5x0x_convert_y(event->y4));
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR,
					 1);
			input_mt_sync(data->input_dev);
			dev_dbg(&client->dev, "=== x4 = %d, y4 = %d ===\n",
					event->x4, event->y4);
			nbreport++;
		}
	case 3:
		if (ft5x0x_check_position(event->x3, event->y3)) {
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
					 event->touch_ID3);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 ft5x0x_convert_x(event->x3));
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 ft5x0x_convert_y(event->y3));
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR,
					 1);
			input_mt_sync(data->input_dev);
			dev_dbg(&client->dev, "=== x3 = %d, y3 = %d ===\n",
					event->x3, event->y3);
			nbreport++;
		}
	case 2:
		if (ft5x0x_check_position(event->x2, event->y2)) {
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
					 event->touch_ID2);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 ft5x0x_convert_x(event->x2));
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 ft5x0x_convert_y(event->y2));
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR,
					 1);
			input_mt_sync(data->input_dev);
			dev_dbg(&client->dev, "=== x2 = %d, y2 = %d ===\n",
					event->x2, event->y2);
			nbreport++;
		}
	case 1:
		if (ft5x0x_is_button(event->x1, event->y1)) {
			spin_lock_irqsave(&data->btn_lock, flags);
			if (event->x1 == 60 || event->x1 == 80) {
				input_event(data->input_dev, EV_KEY, KEY_BACK,
					    1);
				input_report_key(data->input_dev, BTN_TOUCH, 1);
				data->btn_active++;
				nbreport++;
			} else if (event->x1 == 400 || event->x1 == 420) {
				input_event(data->input_dev, EV_KEY, KEY_MENU,
					    1);
				input_report_key(data->input_dev, BTN_TOUCH, 1);
				data->btn_active++;
				nbreport++;
			}
			spin_unlock_irqrestore(&data->btn_lock, flags);
			//dev_dbg(&client->dev, "*** x1 = %d, y1 = %d ***\n",
			printk( "*** x1 = %d, y1 = %d ***\n",
					event->x1, event->y1);
		} else if (ft5x0x_check_position(event->x1, event->y1)) {
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
					 event->touch_ID1);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 ft5x0x_convert_x(event->x1));
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 ft5x0x_convert_y(event->y1));
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR,
					 1);
			input_mt_sync(data->input_dev);
			dev_dbg(&client->dev, "=== x1 = %d, y1 = %d ===\n",
					ft5x0x_convert_x(event->x1),  ft5x0x_convert_y(event->y1));
			nbreport++;
		}
	default:
		dev_dbg(&client->dev, "touch_point default\n");
		break;
	}

	if (nbreport)
		input_sync(data->input_dev);

#else /* CONFIG_FT5X0X_MULTITOUCH */
	if (event->touch_point == 1) {
		input_report_key(data->input_dev, BTN_TOUCH, 1);
		input_report_abs(data->input_dev, ABS_X, /*SCREEN_MAX_X - */
				 event->x1);
		input_report_abs(data->input_dev, ABS_Y, event->y1);
		input_report_abs(data->input_dev, ABS_PRESSURE,
				 !!event->pressure);
		input_sync(data->input_dev);
		dev_dbg(&client->dev, "imc-ts: x:%d y:%d p:%d\n",
			event->x1, event->y1, event->pressure);
	}
	input_sync(data->input_dev);
#endif /* CONFIG_FT5X0X_MULTITOUCH */

	dev_dbg(&client->dev,
		"1:(%d, %d) 2:(%d, %d) 3:(%d, %d) 4:(%d, %d) 5:(%d, %d)\n",
		event->x1, event->y1, event->x2, event->y2, event->x3,
		event->y3, event->x4, event->y4, event->x5, event->y5);
} /* end ft5x0x_report_value */


# if 1

#define PRESS_MAX	0xFF
#define FT_PRESS	0x80


#define FT_MAX_ID	0x0F
#define FT_TOUCH_STEP	6
#define FT_TOUCH_X_H_POS		3
#define FT_TOUCH_X_L_POS		4
#define FT_TOUCH_Y_H_POS		5
#define FT_TOUCH_Y_L_POS		6
#define FT_TOUCH_EVENT_POS		3
#define FT_TOUCH_ID_POS			5

#define POINT_READ_BUF	(3 + FT_TOUCH_STEP * CFG_MAX_TOUCH_POINTS)

/*register address*/
#define FT5x0x_REG_FW_VER		0xA6
#define FT5x0x_REG_VENDOR_ID            0xA8
#define FT5x0x_REG_POINT_RATE	0x88
#define FT5X0X_REG_THGROUP	0x80


/*Read touch point information when the interrupt  is asserted.*/
static int ft5x0x_read_Touchdata(struct ft5x0x_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;
	
//	ret = ft5x0x_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	ret = ft5x0x_i2c_rxdata(this_client,buf,POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));
	
	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
			event->au16_x[i] =
		    (s16) (buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) << 8 | (s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
	//event->au16_x[i]= data->x_max - (event->au16_x[i]);
		event->au16_y[i] =
		    (s16) (buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) << 8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
	//event->au16_y[i]= data->y_max - (event->au16_y[i]);	
	event->au8_touch_event[i] =
		buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
		    (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		#if 0
		printk("id=%d event=%d x=%d y=%d\n", event->au8_finger_id[i],
			event->au8_touch_event[i], event->au16_x[i], event->au16_y[i]);
		#endif
	}

	event->pressure = FT_PRESS;	

	return 0;
}

/*
*report the point information
*/
static void ft5x0x_report_value2(struct ft5x0x_ts_data *data)
{
	struct ts_event *event = &data->event;
	int i;
	int uppoint = 0;

	/*protocol B*/	
	for (i = 0; i < event->touch_point; i++)
	{
		input_mt_slot(data->input_dev, event->au8_finger_id[i]);
		
		if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
		{
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,
				true);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					ft5x0x_convert_x(event->au16_x[i]));
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					ft5x0x_convert_y(event->au16_y[i]));
		 // printk("Report:  x=%d , y=%d\n",event->au16_x[i],event->au16_y[i]);
		 //  printk("                                                Report:  x=%d , y=%d\n",ft5x0x_convert_x(event->au16_x[i]),ft5x0x_convert_y(event->au16_y[i]));
		}
		else
		{
			uppoint++;
			input_mt_slot(data->input_dev,event->au8_finger_id[i]);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,
				false);
		}
	}
	if(event->touch_point == uppoint)
		input_report_key(data->input_dev, BTN_TOUCH, 0);
	else
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
	input_sync(data->input_dev);


}

#endif
static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct ft5x0x_ts_platform_data *ft5x0x_pdata;
	struct i2c_client *client;
	int ret;

	ft5x0x_ts = container_of(to_delayed_work(work),
			struct ft5x0x_ts_data, pen_event_work);
	client = ft5x0x_ts->client;
	ft5x0x_pdata = client->dev.platform_data;

	dev_dbg(&client->dev, "Enter %s\n", __func__);
	ret = ft5x0x_read_data(client);
	//ret = ft5x0x_read_Touchdata(ft5x0x_ts);
	if (ret == 0)
		ft5x0x_report_value(client);
			//ft5x0x_report_value2(ft5x0x_ts);
	else if (ret < 0)
		dev_err(&client->dev, "data package read error %d\n", ret);

	if (ft5x0x_pdata->polling_mode == true &&
			ft5x0x_ts->event.touch_point > 0)
		schedule_delayed_work(&ft5x0x_ts->pen_event_work,
				msecs_to_jiffies(12));
}

static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;

	pr_debug("Enter %s\n", __func__);
	schedule_delayed_work(&ft5x0x_ts->pen_event_work, 0);

	return IRQ_HANDLED;
}

static int ft5x06_ts_reset(struct i2c_client *client, bool show)
{
	struct ft5x0x_ts_platform_data *ft5x06_pdata =
		client->dev.platform_data;
	u8 mode;

	mdelay(100);
	gpio_set_value(ft5x06_pdata->reset_pin, 0);
	mdelay(50);
	gpio_set_value(ft5x06_pdata->reset_pin, 1);
	mdelay(100);

	ft5x0x_set_u8(client, FT5X0X_REG_DEVICE_MODE, 0);
	ft5x0x_set_u8(client, FT5X0X_REG_PMODE, 0x00);
	ft5x0x_set_u8(client, FT5X0X_REG_THGROUP, 0x10);
	ft5x0x_set_u8(client, FT5X0X_REG_PERIODACTIVE, 14);

	/* set interrupt mode */
	mode = (ft5x06_pdata->polling_mode == true) ? 0x0 : 0x1;
	ft5x0x_set_u8(client, FT5X0X_REG_MODE, mode);

	if (!show)
		goto out;

	dev_info(&client->dev, "LIB VER : 0x%04x\n",
			ft5x0x_read_u16(client,	FT5X0X_REG_LIB_VERSION_H));
	dev_info(&client->dev, "FIRMID  : 0x%02x\n",
			ft5x0x_read_u8(client, FT5X0X_REG_FIRMID));
	dev_info(&client->dev, "ID      : 0x%02x\n",
			ft5x0x_read_u8(client, FT5X0X_REG_FT5201ID));
	dev_info(&client->dev, "Touch threshold is : %d\n",
			ft5x0x_read_u8(client, FT5X0X_REG_THGROUP) * 4);
	dev_info(&client->dev, "Report rate is : %dHz\n",
			ft5x0x_read_u8(client, FT5X0X_REG_PERIODACTIVE) * 10);
	dev_info(&client->dev, "%s mode\n",
			(mode == 0) ? "Polling" : "Trigger");

out:
	return 0;
}

#ifdef CONFIG_PM
static int ft5x0x_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	dev_dbg(&client->dev, "%s\n", __func__);
	return ft5x0x_ts_power_off(client);
}

static int ft5x0x_ts_resume(struct i2c_client *client)
{
	int ret;

	dev_dbg(&client->dev, "%s\n", __func__);
	ret = ft5x0x_ts_power_on(client);
	if (ret) {
		dev_err(&client->dev, "%s: Error during power on\n",
				__func__);
		return ret;
	}

	return ft5x06_ts_reset(client, false);
}
#endif /* CONFIG_PM */

#define    FTS_PACKET_LENGTH        128
#define FTS_FALSE                   0x0
#define FTS_TRUE                    0x1
typedef unsigned char         FTS_BYTE;
#define FTS_NULL                    0x0



static bool  i2c_write_interface(u8* pbt_buf, int dw_lenth);
static bool i2c_read_interface(u8* pbt_buf, int dw_lenth);

static unsigned char CTPM_FW[]={
	//#include "ft_app.i"
	//#include "FT5416_BYD4_ID0x59_T7SR_Ver0x21_V21_D01_20141225_app.i"
	//#include "FT5416_ID0xA0_T7SR_Ver0x20_V21_D01_20141225_app.i"
	};


/***********************************************************************
  [function]: 
callback:         write a byte data  to ctpm;
[parameters]:
buffer[in]:       write buffer;
length[in]:      the size of write data;    
[return]:
FTS_TRUE:      success;
FTS_FALSE:     io fail;
 ************************************************************************/
static bool byte_write(u8* buffer, int length)
{

	return i2c_write_interface(buffer, length);
}



/***********************************************************************
  [function]: 
callback:         send a command to ctpm.
[parameters]:
btcmd[in]:       command code;
btPara1[in]:     parameter 1;    
btPara2[in]:     parameter 2;    
btPara3[in]:     parameter 3;    
num[in]:         the valid input parameter numbers, 
if only command code needed and no 
parameters followed,then the num is 1;    
[return]:
FTS_TRUE:      success;
FTS_FALSE:     io fail;
 ************************************************************************/
static bool cmd_write(u8 btcmd,u8 btPara1,u8 btPara2,u8 btPara3,u8 num)
{
	u8 write_cmd[4] = {0};

	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;
	return i2c_write_interface(write_cmd, num);
}



/***********************************************************************
  [function]: 
callback:         read a byte data  from ctpm;
[parameters]:
buffer[in]:       read buffer;
length[in]:      the size of read data;    
[return]:
FTS_TRUE:      success;
FTS_FALSE:     io fail;
 ************************************************************************/
static bool byte_read(u8* buffer, int length)
{
	return i2c_read_interface(buffer, length);
}



/***********************************************************************
  [function]: 
callback:              read data from ctpm by i2c interface;
[parameters]:
buffer[in]:            data buffer;
length[in]:           the length of the data buffer;
[return]:
FTS_TRUE:            success;
FTS_FALSE:           fail;
 ************************************************************************/
static bool i2c_read_interface(u8* pbt_buf, int dw_lenth)
{
	int ret;

	ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

	if(ret<=0)
	{
		printk("[TSP]i2c_read_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}



/***********************************************************************
  [function]: 
callback:               write data to ctpm by i2c interface;
[parameters]:
buffer[in]:             data buffer;
length[in]:            the length of the data buffer;
[return]:
FTS_TRUE:            success;
FTS_FALSE:           fail;
 ************************************************************************/
static bool  i2c_write_interface(u8* pbt_buf, int dw_lenth)
{
	int ret;
	ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
	if(ret<=0)
	{
		printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
		return FTS_FALSE;
	}

	return FTS_TRUE;
}



/***********************************************************************
  [function]: 
callback:                 read register value ftom ctpm by i2c interface;
[parameters]:
reg_name[in]:         the register which you want to read;
rx_buf[in]:              data buffer which is used to store register value;
rx_length[in]:          the length of the data buffer;
[return]:
FTS_TRUE:              success;
FTS_FALSE:             fail;
 ************************************************************************/
static bool fts_register_read(u8 reg_name, u8* rx_buf, int rx_length)
{
	u8 read_cmd[2]= {0};
	u8 cmd_len 	= 0;

	read_cmd[0] = reg_name;
	cmd_len = 1;	

	/*send register addr*/
	if(!i2c_write_interface(&read_cmd[0], cmd_len))
	{
		return FTS_FALSE;
	}

	/*call the read callback function to get the register value*/		
	if(!i2c_read_interface(rx_buf, rx_length))
	{
		return FTS_FALSE;
	}
	return FTS_TRUE;
}




/***********************************************************************
  [function]: 
callback:                read register value ftom ctpm by i2c interface;
[parameters]:
reg_name[in]:         the register which you want to write;
tx_buf[in]:              buffer which is contained of the writing value;
[return]:
FTS_TRUE:              success;
FTS_FALSE:             fail;
 ************************************************************************/
static bool fts_register_write(u8 reg_name, u8* tx_buf)
{
	u8 write_cmd[2] = {0};

	write_cmd[0] = reg_name;
	write_cmd[1] = *tx_buf;

	/*call the write callback function*/
	return i2c_write_interface(write_cmd, 2);
}


/***********************************************************************
  [function]: 
callback:                send data to ctpm by i2c interface;
[parameters]:
txdata[in]:              data buffer which is used to send data;
length[in]:              the length of the data buffer;
[return]:
FTS_TRUE:              success;
FTS_FALSE:             fail;
 ************************************************************************/
static int fts_i2c_txdata(u8 *txdata, int length)
{
	int ret;

	struct i2c_msg msg;

	msg.addr = this_client->addr;
	msg.flags = 0;
	msg.len = length;
	msg.buf = txdata;
//	msg.scl_rate = FT5X0X_I2C_SPEED;
	
	ret = i2c_transfer(this_client->adapter, &msg, 1);
	
	if(ret == 0){
		pr_err("msg %s line:%d i2c write error: %d\n", __func__, __LINE__,ret);
		return -EBUSY;
	}else if(ret < 0){
		pr_err("msg %s line:%d i2c write error: %d\n", __func__, __LINE__,ret);
		return ret;
	}

	return ret;
}



/***********************************************************************
  [function]: 
callback:          burn the FW to ctpm.
[parameters]:
pbt_buf[in]:     point to Head+FW ;
dw_lenth[in]:   the length of the FW + 6(the Head length);    
[return]:
ERR_OK:          no error;
ERR_MODE:      fail to switch to UPDATE mode;
ERR_READID:   read id fail;
ERR_ERASE:     erase chip fail;
ERR_STATUS:   status error;
ERR_ECC:        ecc error.
 ************************************************************************/
E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(u8* pbt_buf, int dw_lenth)
{
	u8  cmd,reg_val[2] = {0};
	u8  packet_buf[FTS_PACKET_LENGTH + 6];
	u8  auc_i2c_write_buf[10];
	u8  bt_ecc;

	int  j,temp,lenght,i_ret,packet_number, i = 0;
	int  i_is_new_protocol = 0;


	/******write 0xaa to register 0xfc******/
	cmd=0xaa;
	fts_register_write(0xfc,&cmd);
	mdelay(50);

	/******write 0x55 to register 0xfc******/
	cmd=0x55;
	fts_register_write(0xfc,&cmd);
	printk("[TSP] Step 1: Reset CTPM test\n");

	mdelay(10);   


	/*******Step 2:Enter upgrade mode ****/
	printk("\n[TSP] Step 2:enter new update mode\n");
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do
	{
		i ++;
		i_ret = fts_i2c_txdata(auc_i2c_write_buf, 2);
		mdelay(5);
	}while(i_ret <= 0 && i < 10 );

	if (i > 1)
	{
		i_is_new_protocol = 1;
	}

	/********Step 3:check READ-ID********/        
	cmd_write(0x90,0x00,0x00,0x00,4);
	byte_read(reg_val,2);
	if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
	{
		printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	}
	else
	{
		printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x  failed\n",reg_val[0],reg_val[1]);
		return ERR_READID;
		//i_is_new_protocol = 1;
	}


	/*********Step 4:erase app**********/
	if (i_is_new_protocol)
	{
		cmd_write(0x61,0x00,0x00,0x00,1);
	}
	else
	{
		cmd_write(0x60,0x00,0x00,0x00,1);
	}
	mdelay(1500);
	printk("[TSP] Step 4: erase. \n");



	/*Step 5:write firmware(FW) to ctpm flash*/
	bt_ecc = 0;
	printk("[TSP] Step 5: start upgrade. \n");
	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j=0;j<packet_number;j++)
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (FTS_BYTE)(lenght>>8);
		packet_buf[5] = (FTS_BYTE)lenght;

		for (i=0;i<FTS_PACKET_LENGTH;i++)
		{
			packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
			bt_ecc ^= packet_buf[6+i];
		}

		byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
		mdelay(FTS_PACKET_LENGTH/6 + 1);
		if ((j * FTS_PACKET_LENGTH % 1024) == 0)
		{
			printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;

		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (FTS_BYTE)(temp>>8);
		packet_buf[5] = (FTS_BYTE)temp;

		for (i=0;i<temp;i++)
		{
			packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
			bt_ecc ^= packet_buf[6+i];
		}

		byte_write(&packet_buf[0],temp+6);    
		mdelay(20);
	}

	/***********send the last six byte**********/
	for (i = 0; i<6; i++)
	{
		temp = 0x6ffa + i;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;
		temp =1;
		packet_buf[4] = (FTS_BYTE)(temp>>8);
		packet_buf[5] = (FTS_BYTE)temp;
		packet_buf[6] = pbt_buf[ dw_lenth + i]; 
		bt_ecc ^= packet_buf[6];

		byte_write(&packet_buf[0],7);  
		mdelay(20);
	}

	/********send the opration head************/
	cmd_write(0xcc,0x00,0x00,0x00,1);
	byte_read(reg_val,1);
	printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
	if(reg_val[0] != bt_ecc)
	{
		return ERR_ECC;
	}

	/*******Step 7: reset the new FW**********/
	cmd_write(0x07,0x00,0x00,0x00,1);

	return ERR_OK;
}

int fts_ctpm_fw_upgrade_with_i_file(void)
{
	u8*     pbt_buf = FTS_NULL;
	int i_ret;

	pbt_buf = CTPM_FW;
	i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));

	return i_ret;
}

unsigned char fts_ctpm_get_upg_ver(void)
{
	unsigned int ui_sz;

	ui_sz = sizeof(CTPM_FW);
	if (ui_sz > 2)
	{
		return CTPM_FW[ui_sz - 2];
	}
	else
		return 0xff; 

}

static int i2c_master_reg8_recv(const struct i2c_client *client, const char reg, char *buf, int count, int scl_rate)
{
		struct i2c_adapter *adap=client->adapter;
		struct i2c_msg msgs[2];
		int ret;
		char reg_buf = reg;

		msgs[0].addr = client->addr;
		msgs[0].flags = client->flags;
		msgs[0].len = 1;
		msgs[0].buf = &reg_buf;
		//msgs[0].scl_rate = scl_rate;

		msgs[1].addr = client->addr;
		msgs[1].flags = client->flags | I2C_M_RD;
		msgs[1].len = count;
		msgs[1].buf = (char *)buf;
		//msgs[1].scl_rate = scl_rate;

		ret = i2c_transfer(adap, msgs, 2);

		return (ret == 2)? count : ret;
}


static int ft5x0x_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	struct ft5x0x_ts_platform_data *ft5x06_pdata;
	int err = 0;
	unsigned char reg_value;	
	unsigned char reg_version;
	
	this_client=client;
	
	dev_dbg(&client->dev, "EDT FT5X06 touchscreen driver probing\n");

#ifdef CONFIG_OF
	ft5x06_pdata = client->dev.platform_data =
		ft5x0x_ts_of_get_platdata(client);
	if (IS_ERR(ft5x06_pdata)) {
		err = PTR_ERR(ft5x06_pdata);
		return err;
	}
#else
	ft5x06_pdata = client->dev.platform_data;
#endif

	ft5x0x_set_pinctrl_state(&client->dev, ft5x06_pdata->pins_default);

	if (ft5x06_pdata->pm_platdata) {
		err = device_state_pm_set_class(&client->dev,
			ft5x06_pdata->pm_platdata->pm_user_name);
		if (err) {
			dev_err(&client->dev,
				"Error while setting the pm class\n");
			goto exit_pm_class;
		}

		err = device_state_pm_set_state_by_name(&client->dev,
				ft5x06_pdata->pm_platdata->pm_state_D0_name);
		if (err)
			goto exit_pm_class;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	if (gpio_is_valid(ft5x06_pdata->reset_pin)) {
		/* this pulls reset down, enabling the low active reset */
		err = gpio_request(ft5x06_pdata->reset_pin,
				"edt-ft5x06 reset");
		if (err) {
			dev_err(&client->dev,
				"Failed to request GPIO %d as reset pin, error %d\n",
				ft5x06_pdata->reset_pin, err);
			goto exit_reset_failed;
		}
	} else {
		err = -EINVAL;
		dev_err(&client->dev,
			"invalid GPIO %d as reset pin\n",
			ft5x06_pdata->reset_pin);
		goto exit_reset_failed;
	}

	err = ft5x06_ts_reset(client, true);
	if (err) {
		dev_err(&client->dev, "reset failed\n");
		goto exit_reset_failed;
	}

	dev_dbg(&client->dev, "%s: kzalloc\n", __func__);
	ft5x0x_ts = devm_kzalloc(&client->dev, sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	dev_dbg(&client->dev, "%s: i2c_set_clientdata\n", __func__);
	i2c_set_clientdata(client, ft5x0x_ts);
	ft5x0x_ts->client = client;

	dev_dbg(&client->dev, "%s: INIT_DELAYED_WORK\n", __func__);
	INIT_DELAYED_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);

	dev_dbg(&client->dev, "%s: create_singlethread_workqueue\n", __func__);
	ft5x0x_ts->ts_workqueue =
	    create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		dev_err(&client->dev, "singlethread error = %d\n", err);
		goto exit_create_singlethread;
	}

	spin_lock_init(&ft5x0x_ts->btn_lock);
	spin_lock_init(&ft5x0x_ts->en_lock);
	ft5x0x_ts->btn_active = 0;
	ft5x0x_ts->enable = 1;


	
#if 0
	
	msleep(200);
	
	this_client = client;
	err = i2c_master_reg8_recv(this_client, FT5X0X_REG_FIRMID, &reg_version, 1, 200*1000);
	if (err < 0) {
		printk("[LAIBAO] Device not found\n");
		goto out;
	}
	
	if (fts_ctpm_get_upg_ver() != reg_version)	
	{
		printk("[LAIBAO] start upgrade new verison 0x%2x\n", fts_ctpm_get_upg_ver());
		msleep(200);
		err =  fts_ctpm_fw_upgrade_with_i_file();
		if (err == 0)
		{
			printk("[LAIBAO] ugrade successfuly.\n");
			msleep(300);
			fts_register_read(FT5X0X_REG_FIRMID, &reg_value,1);
			printk("FTS_DBG from old version 0x%2x to new version = 0x%2x\n", reg_version, reg_value);
		}
		else
		{
			printk("[LAIBAO]  ugrade fail err=%d, line = %d.\n",
					err, __LINE__);
		}
		msleep(4000);
	}
#endif
	


	err = request_irq(client->irq, ft5x0x_ts_interrupt, IRQF_DISABLED,
			"ft5x0x_ts", ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	dev_dbg(&client->dev, "%s: input_allocate_device\n", __func__);
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft5x0x_ts->input_dev = input_dev;

#ifdef CONFIG_FT5X0X_MULTITOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 5, 0, 0);

	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_MENU, input_dev->keybit);

	printk("== func = %s ,line = %d, SCREEN_MAX_X =%d , SCREEN_MAX_Y = %d\n",
			__FUNCTION__,__LINE__,SCREEN_MAX_X,SCREEN_MAX_Y);
#else
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, PRESS_MAX, 0, 0);
#endif

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_dev->name = FT5X0X_NAME;
	dev_dbg(&client->dev, "%s: TS_MAX_X_COORD %d TS_MAX_Y_COORD %d\n",
		input_dev->name, SCREEN_MAX_X, SCREEN_MAX_Y);
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"%s: failed to register input device: %s\n",
			__func__, dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &ft5x0x_ts_attr_group);
	if (err)
		goto exit_sysfs_failed;

	dev_dbg(&client->dev, "%s: probe over %d, client->irq=%d\n",
			__func__, err, client->irq);
	goto out;

exit_sysfs_failed:
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(client->irq, ft5x0x_ts);
exit_irq_request_failed:
	cancel_delayed_work(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
exit_create_singlethread:
	i2c_set_clientdata(client, NULL);
exit_alloc_data_failed:
exit_reset_failed:
exit_check_functionality_failed:
exit_pm_class:
out:
	return err;
}

static int ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);
	struct ft5x0x_ts_platform_data *pdata =
		dev_get_platdata(&client->dev);

	dev_dbg(&client->dev, "%s\n", __func__);
	ft5x0x_set_pinctrl_state(&client->dev, pdata->pins_inactive);
	free_irq(client->irq, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_dev);
	kfree(ft5x0x_ts);
	cancel_delayed_work(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{"ft5x06", 0}, {}
};

MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.probe = ft5x0x_ts_probe,
	.remove = ft5x0x_ts_remove,
	.id_table = ft5x0x_ts_id,
	.suspend = ft5x0x_ts_suspend,
	.resume = ft5x0x_ts_resume,
	.driver = {
		.name = FT5X0X_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ft5x06_ts_init(void)
{
#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	int ret = device_state_pm_add_class(&ft5x0x_pm_class);
	if (ret)
		return ret;
#endif
	return i2c_add_driver(&ft5x0x_ts_driver);
}

static void __exit ft5x06_ts_exit(void)
{
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x06_ts_init);
module_exit(ft5x06_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
