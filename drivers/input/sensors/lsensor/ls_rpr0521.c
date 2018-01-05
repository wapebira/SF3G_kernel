/* drivers/input/sensors/lsensor/ls_rpr0521.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: Bin Yang <yangbin@rock-chips.com>
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
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/sensor-dev.h>

#define APS_TAG                  "[ALS/PS] "
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#if 0
#define APS_DBG(fmt, args...)    printk(APS_TAG fmt, ##args)
#else
#define APS_DBG(fmt, args...)
#endif

/*RPR-0521  Register Address*/
#define RPR0521_SYSTEM_CTRL		0x40
#define RPR0521_MODE_CTRL		0x41
#define RPR0521_ALS_PS_CTRL		0x42
#define RPR0521_PS_CTRL			0x43
#define RPR0521_PS_DATA_LSB		0x44
#define RPR0521_PS_DATA_MSB		0x45
#define RPR0521_ALS_DATA0_LSB	0x46
#define RPR0521_ALS_DATA0_MSB	0x47
#define RPR0521_ALS_DATA1_LSB	0x48
#define RPR0521_ALS_DATA1_MSB	0x49
#define RPR0521_INTERRUPT		0x4A
#define RPR0521_PS_TH_LSB		0x4B
#define RPR0521_PS_TH_MSB		0x4C
#define RPR0521_PS_TL_LSB		0x4D
#define RPR0521_PS_TL_MSB		0x4E
#define RPR0521_ALS_TH_LSB		0x4F
#define RPR0521_ALS_TH_MSB		0x50
#define RPR0521_ALS_TL_LSB		0x51
#define RPR0521_ALS_TL_MSB		0x52
#define RPR0521_PS_OFFSET_LSB	0x53
#define RPR0521_PS_OFFSET_MSB	0x54
#define RPR0521_WHO_AM_I		0x92

#define RPR0521_DEVICE_ID		0xE0
#define RPR0521_ALS_EN			(1 << 7)
#define RPR0521_PS_EN			(1 << 6)

uint16_t g_rpr0521_levels[9] = {0, 10, 160, 225, 320, 640, 1280, 2600, 0xffff};

int get_rpr0521_level(uint16_t data)
{
	unsigned char index = 0;

	if (data <= 10)
		index = 0;
	else if (data <= 160)
		index = 1;
	else if (data <= 225)
		index = 2;
	else if (data <= 320)
		index = 3;
	else if (data <= 640)
		index = 4;
	else if (data <= 1280)
		index = 5;
	else if (data <= 2600)
		index = 6;
	else
		index = 7;

	return index;
}

static int rpr0521_enable_als(struct i2c_client *client, int enable, int rate)
{
	long res = 0;
	int status = 0;
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);

	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
	if (enable) {
		status = RPR0521_ALS_EN;
		sensor->ops->ctrl_data |= status;
		APS_DBG("lsensor rpr0521 power on\n");
	} else {
		status = ~RPR0521_ALS_EN;
		sensor->ops->ctrl_data &= status;
		APS_DBG("lsensor rpr0521 power off\n");
	}
	res = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if (res) {
		APS_ERR("rpr0521_enable_als fail\n");
		return res;
	}

	return 0;
}

static int rpr0521_init_client(struct i2c_client *client)
{
	int res = 0;
	unsigned char reg_data = 0;

	reg_data = sensor_read_reg(client, RPR0521_MODE_CTRL);
	reg_data = reg_data & 0xC0;
	reg_data = reg_data | 0x05;		/*ALS:100ms PS:50ms*/
	res = sensor_write_reg(client, RPR0521_MODE_CTRL, reg_data);
	if (res)
		goto EXIT_ERR;
	/*ALS DATA0 Gain:x1, ALS DATA1 Gain:x1; LED CURRENT:100mA*/
	res = sensor_write_reg(client, RPR0521_ALS_PS_CTRL, 0x02);
	if (res)
		goto EXIT_ERR;

	res = sensor_write_reg(client, RPR0521_INTERRUPT, 0x23);
	if (res)
		goto EXIT_ERR;

	rpr0521_enable_als(client, 0, 0);

	return 0;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

static int light_report_value(struct input_dev *input, unsigned char index)
{
	input_report_abs(input, ABS_MISC, index);
	input_sync(input);

	return index;
}

static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *)i2c_get_clientdata(client);
	int ret = 0;
	unsigned char als_value_low = 0;
	unsigned char als_value_high = 0;
	unsigned char value = 0;
	unsigned short data0_value = 0;
	unsigned short data1_value = 0;
	unsigned short als_value = 0;
	uint8_t als_level;
	int index = 0;

	/*get adc channel 0 value*/
	als_value_low = sensor_read_reg(client, RPR0521_ALS_DATA0_LSB);
	if (als_value_low < 0)
		return als_value_low;
	als_value_high = sensor_read_reg(client, RPR0521_ALS_DATA0_MSB);
	if (als_value_high < 0)
		return als_value_high;
	data0_value = als_value_low | (als_value_high << 8);

	/*get adc channel 1 value*/
	als_value_low = sensor_read_reg(client, RPR0521_ALS_DATA1_LSB);
	if (als_value_low < 0)
		return als_value_low;
	als_value_high = sensor_read_reg(client, RPR0521_ALS_DATA1_MSB);
	if (als_value_high < 0)
		return als_value_high;
	data1_value = als_value_low | (als_value_high << 8);

	APS_DBG("data0_value=%d, data1_value=%d\n", data0_value, data1_value);

	als_value = (data0_value + data1_value)/2;
	als_level = get_rpr0521_level(als_value);
	index = light_report_value(sensor->input_dev, als_level);

	sensor_write_word_reg(client, RPR0521_ALS_TH_LSB, g_rpr0521_levels[als_level + 1]);
	sensor_write_word_reg(client, RPR0521_ALS_TL_LSB, g_rpr0521_levels[als_level]);

	/*read sensor intterupt status register*/
	if (sensor->pdata->irq_enable) {
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		if (!(value & 0x40)) {
			APS_DBG("not lsensor int value =0x%2x!!!\n", value);
			return ret;
		}
	}

	return ret;
}

struct sensor_operate light_rpr0521_ops = {
	.name			= "ls_rpr0521",
	.type			= SENSOR_TYPE_LIGHT,		/*sensor type and it should be correct*/
	.id_i2c			= LIGHT_ID_RPR0521,			/*i2c id number*/
	.read_reg		= RPR0521_ALS_DATA0_LSB,	/*read data*/
	.read_len		= 2,						/*data length*/
	.id_reg			= RPR0521_WHO_AM_I,			/*read device id from this register*/
	.id_data		= RPR0521_DEVICE_ID,		/*device id*/
	.precision		= 16,						/*16bits*/
	.ctrl_reg		= RPR0521_MODE_CTRL,		/*enable or disable*/
	.int_status_reg	= RPR0521_INTERRUPT,		/*intterupt status register*/
	.range			= {100, 65535},				/*range*/
	.brightness		= {10, 255},				/* brightness*/
	.trig			= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,
	.active			= rpr0521_enable_als,
	.init			= rpr0521_init_client,
	.report			= sensor_report_value,
};

/*function name should not be changed*/
static struct sensor_operate *light_get_ops(void)
{
	return &light_rpr0521_ops;
}

static int __init light_rpr0521_init(void)
{
	struct sensor_operate *ops = light_get_ops();
	int result = 0;
	int type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, light_get_ops);
	return result;
}

static void __exit light_rpr0521_exit(void)
{
	struct sensor_operate *ops = light_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, light_get_ops);
}

module_init(light_rpr0521_init);
module_exit(light_rpr0521_exit);
