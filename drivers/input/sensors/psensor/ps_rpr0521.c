/* drivers/input/sensors/psensor/ps_rpr0521.c
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
#include <linux/proc_fs.h>

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

static u16 ps_thd_val_high = 0xFFFF;
static u16 ps_thd_val_low = 0x0;

static int ps_rpr0521_parse_dt(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	uint32_t temp_val;
	int rc;

	rc = of_property_read_u32(np, "ps_threshold_low", &temp_val);
	if (rc) {
		printk("Unable to read Ps_threshold_low\n");
		return rc;
	} else {
		ps_thd_val_low = (u16)temp_val;
	}

	rc = of_property_read_u32(np, "ps_threshold_high", &temp_val);
	if (rc)	{
		printk("Unable to read ps_threshold_high\n");
		return rc;
	} else {
		ps_thd_val_high = (u16)temp_val;
	}

	APS_DBG("%s: ps_threshold_low = %d\n", __func__, ps_thd_val_low);
	APS_DBG("%s: ps_threshold_high = %d\n", __func__, ps_thd_val_high);
	return 0;
}

static int rpr0521_enable_ps(struct i2c_client *client, int enable, int rate)
{
	long res = 0;
	int status = 0;
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);

	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
	if (enable) {
		status = RPR0521_PS_EN;
		sensor->ops->ctrl_data |= status;
		APS_DBG("psensor rpr0521 power on\n");
	} else {
		status = ~RPR0521_PS_EN;
		sensor->ops->ctrl_data &= status;
		APS_DBG("psensor rpr0521 power off\n");
	}

	res = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if (res) {
		APS_ERR("rpr0521_enable_ps fail\n");
		return res;
	}
	if (enable) {
		input_report_abs(sensor->input_dev, ABS_DISTANCE, 1);
		input_sync(sensor->input_dev);
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

	res = sensor_write_reg(client, RPR0521_PS_CTRL, 0x02);
	if (res)
		goto EXIT_ERR;

	res = sensor_write_reg(client, RPR0521_INTERRUPT, 0x23);
	if (res)
		goto EXIT_ERR;

	ps_rpr0521_parse_dt(client);

	/*initialization  Proximity Interrupt Threshold Registers*/
	res = sensor_write_reg(client, RPR0521_PS_TH_LSB, (u8)(ps_thd_val_high & 0x00FF));
	if (res)
		goto EXIT_ERR;
	res = sensor_write_reg(client, RPR0521_PS_TH_MSB, (u8)(ps_thd_val_high & 0xFF00) >> 8);
	if (res)
		goto EXIT_ERR;
	res = sensor_write_reg(client, RPR0521_PS_TL_LSB, (u8)(ps_thd_val_low & 0x00FF));
	if (res)
		goto EXIT_ERR;
	res = sensor_write_reg(client, RPR0521_PS_TL_MSB, (u8)((ps_thd_val_low & 0xFF00) >> 8));
	if (res)
		goto EXIT_ERR;

	rpr0521_enable_ps(client, 0, 0);

	return 0;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);
	int ret = 0;
	u16 ps_data = 0;
	u8 buffer[2] = {0};
	unsigned char value = 0;
	int ps_report_val = 0;
	static int val_temp = 1;
	static int intr_flag_value = 0;

	/*sensor->ops->read_len = 2*/
	if (sensor->ops->read_len < 2) {
		printk("%s:len is error,len=%d\n", __func__, sensor->ops->read_len);
		return -1;
	}
	memset(buffer, 0, 2);

	do {
		*buffer = sensor->ops->read_reg;
		ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
		if (ret < 0)
			return ret;
	} while (0);

	ps_data = ((buffer[1] << 8) & 0xff00) + (buffer[0] & 0xFF);
	APS_DBG("psensor rpr-0521 PS data : %d\n", ps_data);

	if (ps_data > ps_thd_val_high) {
		ps_report_val = 0;
		val_temp = 0;
		intr_flag_value = 1;
	} else if (ps_data < ps_thd_val_low) {
		ps_report_val = 1;
		val_temp = 1;
		intr_flag_value = 0;
	} else {
	    ps_report_val = val_temp;
	}
	input_report_abs(sensor->input_dev, ABS_DISTANCE, ps_report_val);
	input_sync(sensor->input_dev);

	if (sensor->pdata->irq_enable) {
		if (intr_flag_value) {
			ret = sensor_write_reg(client, RPR0521_PS_TL_LSB, (u8)(ps_thd_val_low & 0x00FF));
			if (ret)
				return ret;
			ret = sensor_write_reg(client, RPR0521_PS_TL_MSB, (u8)((ps_thd_val_low) & 0xFF00) >> 8);
			if (ret)
				return ret;
			ret = sensor_write_reg(client, RPR0521_PS_TH_LSB, (u8)(0x00FF));
			if (ret)
				return ret;
			ret = sensor_write_reg(client, RPR0521_PS_TH_MSB, (u8)((0xFF00) >> 8));
			if (ret)
				return ret;
		} else {
			ret = sensor_write_reg(client, RPR0521_PS_TL_LSB, (u8)(0 & 0x00FF));
			if (ret)
				return ret;
			ret = sensor_write_reg(client, RPR0521_PS_TL_MSB, (u8)((0 & 0xFF00) >> 8));
			if (ret)
				return ret;
			ret = sensor_write_reg(client, RPR0521_PS_TH_LSB, (u8)(ps_thd_val_high & 0x00FF));
			if (ret)
				return ret;
			ret = sensor_write_reg(client, RPR0521_PS_TH_MSB, (u8)((ps_thd_val_high & 0xFF00) >> 8));
			if (ret)
				return ret;
		}
	}
    /*read sensor intterupt status register*/
	if (sensor->pdata->irq_enable) {
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		if (!(value & 0x80)) {
			APS_DBG("not psensor int value =0x%2x!!!\n", value);
			return ret;
		}
	}

	return ret;
}

struct sensor_operate proximity_rpr0521_ops = {
	.name			= "ps_rpr0521",
	.type			= SENSOR_TYPE_PROXIMITY,	/*sensor type and it should be correct*/
	.id_i2c			= PROXIMITY_ID_RPR0521,		/*i2c id number*/
	.read_reg		= RPR0521_PS_DATA_LSB,		/*read data*/
	.read_len		= 2,						/*data length*/
	.id_reg			= RPR0521_WHO_AM_I,			/*read device id from this register*/
	.id_data		= RPR0521_DEVICE_ID,		/*device id*/
	.precision		= 16,						/*16bits*/
	.ctrl_reg		= RPR0521_MODE_CTRL,		/*enable or disable*/
	.int_status_reg	= RPR0521_INTERRUPT,		/*intterupt status register*/
	.range			= {0, 1},					/*range*/
	.trig			= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED | IRQF_NO_SUSPEND,
	.active			= rpr0521_enable_ps,
	.init			= rpr0521_init_client,
	.report			= sensor_report_value,
};

static struct sensor_operate *proximity_get_ops(void)
{
	return &proximity_rpr0521_ops;
}

static int __init proximity_rpr0521_init(void)
{
	struct sensor_operate *ops = proximity_get_ops();
	int result = 0;
	int type = ops->type;

	result = sensor_register_slave(type, NULL, NULL, proximity_get_ops);
	return result;
}

static void __exit proximity_rpr0521_exit(void)
{
	struct sensor_operate *ops = proximity_get_ops();
	int type = ops->type;

	sensor_unregister_slave(type, NULL, NULL, proximity_get_ops);
}

module_init(proximity_rpr0521_init);
module_exit(proximity_rpr0521_exit);
