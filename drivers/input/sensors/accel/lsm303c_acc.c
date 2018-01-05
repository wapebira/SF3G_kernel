/* drivers/input/sensors/access/kxtik.c
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
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/sensor-dev.h>

#define LSM303D_WHO_AM_I		(0x0F)

/* full scale setting - register & mask */
#define LSM303D_ACT_DURA		(0x1F)
#define LSM303D_CTRL_REG1		(0x20)
#define LSM303D_CTRL_REG2		(0x21)
#define LSM303D_CTRL_REG3		(0x22)
#define LSM303D_CTRL_REG4		(0x23)
#define LSM303D_CTRL_REG5		(0x24)
#define LSM303D_CTRL_REG6		(0x25)
#define LSM303D_CTRL_REG7		(0x26)
#define LSM303D_STATUS_REG		(0x27)
#define LSM303D_OUT_X_L			(0x28)
#define LSM303D_OUT_X_H			(0x29)
#define LSM303D_OUT_Y_L			(0x2a)
#define LSM303D_OUT_Y_H			(0x2b)
#define LSM303D_OUT_Z_L			(0x2c)
#define LSM303D_OUT_Z_H			(0x2d)
#define LSM303C_FIFO_CTRL_REG		(0x2E)
#define LSM303D_FIFO_SRC_REG		(0X2F)

#define LSM303C_IG_CFG1			(0x30)
#define LSM303C_IG_SRC1			(0x31)
#define LSM303C_IG_THSX			(0x32)
#define LSM303C_IG_THSY			(0x33)
#define LSM303C_IG_THSZ			(0x34)
#define LSM303C_IG_DURATION1	(0x35)

#define LSM303C_DEVID_A			(0x41)
#define LSM303C_ACC_DISABLE		(0x08)

#define LSM303D_RANGE			2000000

/* LSM303D */
#define LSM303D_PRECISION		16
#define LSM303D_BOUNDARY			(0x1 << (LSM303D_PRECISION - 1))
#define LSM303D_GRAVITY_STEP		(LSM303D_RANGE / LSM303D_BOUNDARY)

#define ODR10				0x10  /* 10Hz output data rate */
#define ODR50				0x20  /* 50Hz output data rate */
#define ODR100				0x30  /* 100Hz output data rate */
#define ODR200				0x40  /* 200Hz output data rate */
#define ODR400				0x50  /* 400Hz output data rate */
#define ODR800				0x60  /* 800Hz output data rate */

struct sensor_reg_data {
	char reg;
	char data;
};

/****************operate according to sensor chip:start************/
static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *)i2c_get_clientdata(client);
	int result = 0;
	int status = 0;

	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);

	sensor->ops->ctrl_data |= ODR100;/*100HZ*/

	if (!enable) {
		status = LSM303C_ACC_DISABLE;
		sensor->ops->ctrl_data |= status;
	} else {
		status = ~LSM303C_ACC_DISABLE;
		sensor->ops->ctrl_data &= status;
	}

	DBG("%s:reg=0x%x,reg_ctrl=0x%x,enable=%d\n", __func__, sensor->ops->ctrl_reg, sensor->ops->ctrl_data, enable);
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if (result)
		printk("%s:fail to active sensor\n", __func__);

	return result;
}

static int sensor_init(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *)i2c_get_clientdata(client);
	int result = 0;
	int i;

	struct sensor_reg_data reg_data[] = {
		{LSM303D_CTRL_REG1, 0x07},
		{LSM303D_CTRL_REG2, 0x00},
		{LSM303D_CTRL_REG3, 0x08},
		{LSM303D_CTRL_REG4, 0x04},
		{LSM303D_CTRL_REG5, 0x00},
		{LSM303D_CTRL_REG6, 0x00},
		{LSM303D_CTRL_REG7, 0x0C},
		{LSM303C_FIFO_CTRL_REG, 0x00},
		{LSM303C_IG_CFG1, 0xFF},
		{LSM303C_IG_THSX, 0x7F},
		{LSM303C_IG_THSY, 0x7F},
		{LSM303C_IG_THSZ, 0x7F},
		{LSM303C_IG_DURATION1, 0x7F},
	};

	result = sensor->ops->active(client, 0, 0);
	if (result) {
		printk("%s:line=%d,error\n", __func__, __LINE__);
		return result;
	}
	sensor->status_cur = SENSOR_OFF;

	for (i = 0; i < (sizeof(reg_data)/sizeof(struct sensor_reg_data)); i++) {
		result = sensor_write_reg(client, reg_data[i].reg, reg_data[i].data);
		if (result) {
			printk("%s:line=%d,i=%d,error\n", __func__, __LINE__, i);
			return result;
		}
	}

	if (sensor->pdata->irq_enable) {
		result = sensor_write_reg(client, LSM303D_CTRL_REG3, 0x08);
		if (result) {
			printk("%s:line=%d,error\n", __func__, __LINE__);
			return result;
		}
		i = sensor_read_reg(client, LSM303D_CTRL_REG7);
		result = sensor_write_reg(client, LSM303D_CTRL_REG7, (i|0x04));
		if (result) {
			printk("%s:line=%d,error\n", __func__, __LINE__);
			return result;
		}
	}
	return result;
}

static int sensor_convert_data(struct i2c_client *client, unsigned char high_byte, unsigned char low_byte)
{
	s64 result;
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *)i2c_get_clientdata(client);

	switch (sensor->devid) {
	case LSM303C_DEVID_A:
		result = ((int)high_byte << 8) | (int)low_byte;
		if (result < LSM303D_BOUNDARY)
			result = result * LSM303D_GRAVITY_STEP;
		else
			result = ~(((~result & (0x7fff>>(16-LSM303D_PRECISION))) + 1)
					* LSM303D_GRAVITY_STEP) + 1;
		break;

	default:
		printk(KERN_ERR "%s: devid wasn't set correctly\n", __func__);
		return -EFAULT;
	}
	return (int)result;
}

static int gsensor_report_value(struct i2c_client *client, struct sensor_axis *axis)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);

	/* Report acceleration sensor information */
	input_report_abs(sensor->input_dev, ABS_X, axis->x);
	input_report_abs(sensor->input_dev, ABS_Y, axis->y);
	input_report_abs(sensor->input_dev, ABS_Z, axis->z);
	input_sync(sensor->input_dev);
	DBG("Gsensor x==%d  y==%d z==%d\n", axis->x, axis->y, axis->z);

	return 0;
}

#define GSENSOR_MIN  10
static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
			(struct sensor_private_data *)i2c_get_clientdata(client);
	struct sensor_platform_data *pdata = sensor->pdata;
	int ret = 0;
	int x, y, z;
	struct sensor_axis axis;
	unsigned char buffer[6] = {0};
	char value = 0;

	if (sensor->ops->read_len < 6) {
		printk("%s:lenth is error,len=%d\n", __func__, sensor->ops->read_len);
		return -1;
	}

	memset(buffer, 0, 6);

	value = sensor_read_reg(client, LSM303D_STATUS_REG);
	if ((value & 0x0f) == 0) {
		printk("%s:line=%d,value=0x%x,data is not ready\n", __func__, __LINE__, value);
		return -1;
	}
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	do {
		*buffer = sensor->ops->read_reg;
		ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
		if (ret < 0)
			return ret;
	} while (0);

	/*this gsensor need 6 bytes buffer*/
	x = sensor_convert_data(sensor->client, buffer[1], buffer[0]);
	y = sensor_convert_data(sensor->client, buffer[3], buffer[2]);
	z = sensor_convert_data(sensor->client, buffer[5], buffer[4]);

	axis.x = (pdata->orientation[0])*x + (pdata->orientation[1])*y + (pdata->orientation[2])*z;
	axis.y = (pdata->orientation[3])*x + (pdata->orientation[4])*y + (pdata->orientation[5])*z;
	axis.z = (pdata->orientation[6])*x + (pdata->orientation[7])*y + (pdata->orientation[8])*z;

	DBG("%s: axis = %d  %d  %d \n", __func__, axis.x, axis.y, axis.z);

	if ((abs(sensor->axis.x - axis.x) > GSENSOR_MIN) || (abs(sensor->axis.y - axis.y) > GSENSOR_MIN) || (abs(sensor->axis.z - axis.z) > GSENSOR_MIN)) {
		gsensor_report_value(client, &axis);
		mutex_lock(&sensor->data_mutex);
		sensor->axis = axis;
		mutex_unlock(&sensor->data_mutex);
	}

	if ((sensor->pdata->irq_enable) && (sensor->ops->int_status_reg >= 0)) {
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		DBG("%s:sensor int status :0x%x\n", __func__, value);
	}

	return ret;
}

struct sensor_operate gsensor_lsm303c_ops = {
	.name			= "lsm303c_acc",
	.type			= SENSOR_TYPE_ACCEL,			/*sensor type and it should be correct*/
	.id_i2c			= ACCEL_ID_LSM303C,				/*i2c id number*/
	.read_reg		= (LSM303D_OUT_X_L | 0x80),		/*read data*/
	.read_len		= 6,							/*data length*/
	.id_reg			= LSM303D_WHO_AM_I,				/*read device id from this register*/
	.id_data		= LSM303C_DEVID_A,				/*device id*/
	.precision		= LSM303D_PRECISION,			/*16 bits*/
	.ctrl_reg		= LSM303D_CTRL_REG1,			/*enable or disable*/
	.int_status_reg	= LSM303C_IG_SRC1,				/*intterupt status register*/
	.range			= {-LSM303D_RANGE, LSM303D_RANGE},/*range*/
	.trig			= (IRQF_TRIGGER_LOW|IRQF_ONESHOT),
	.active			= sensor_active,
	.init			= sensor_init,
	.report			= sensor_report_value,
};

/****************operate according to sensor chip:end************/
/*function name should not be changed*/
static struct sensor_operate *gsensor_get_ops(void)
{
	return &gsensor_lsm303c_ops;
}

static int __init gsensor_lsm303c_init(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int result = 0;
	int type = ops->type;

	result = sensor_register_slave(type, NULL, NULL, gsensor_get_ops);
	return result;
}

static void __exit gsensor_lsm303c_exit(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int type = ops->type;

	sensor_unregister_slave(type, NULL, NULL, gsensor_get_ops);
}

module_init(gsensor_lsm303c_init);
module_exit(gsensor_lsm303c_exit);
