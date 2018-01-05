/* drivers/input/sensors/access/mpu6880_gyro.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: ouenhui <oeh@rock-chips.com>
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
#include <linux/mpu6880.h>

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *)i2c_get_clientdata(client);
	int result = 0;
	int status = 0;
	u8  pwrm1  = 0;

	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
	pwrm1 = sensor_read_reg(client, MPU6880_PWR_MGMT_1);
	DBG("%s:reg=0x%x,reg_ctrl=0x%x,pwrm1=0x%x,enable=%d\n", __func__, sensor->ops->ctrl_reg, sensor->ops->ctrl_data, pwrm1, enable);
	/*register setting according to chip datasheet*/
	if (!enable) {
		status = BIT_GYRO_STBY;
		sensor->ops->ctrl_data |= status;
		if ((sensor->ops->ctrl_data & BIT_ACCEL_STBY) == BIT_ACCEL_STBY)
			pwrm1 |= MPU6880_PWRM1_SLEEP;
	} else {
		status = ~BIT_GYRO_STBY;
		sensor->ops->ctrl_data &= status;
		pwrm1 &= ~MPU6880_PWRM1_SLEEP;
	}
	DBG("%s:reg=0x%x,reg_ctrl=0x%x,pwrm1=0x%x,enable=%d\n", __func__, sensor->ops->ctrl_reg, sensor->ops->ctrl_data, pwrm1, enable);
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if (result)
		printk("%s:fail to active sensor\n", __func__);
	result = sensor_write_reg(client, MPU6880_PWR_MGMT_1, pwrm1);
	if (result)
		printk("%s:fail to set pwrm1\n", __func__);
	return result;
}

static int sensor_init(struct i2c_client *client)
{
	int ret;
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *)i2c_get_clientdata(client);

	/*client->addr = sensor->ops->slave_addr;*/
	ret = sensor->ops->active(client, 0, 0);
	if (ret) {
		printk("%s:line=%d,error\n", __func__, __LINE__);
		return ret;
	}
	msleep(20);
	sensor->status_cur = SENSOR_OFF;
	ret = sensor_write_reg(client, MPU6880_GYRO_CONFIG, 0x18);	/*config gyro for 2000dps*/
	if (ret) {
		printk("Set Gyroscope Configuration 0x1B error,ret: %d!\n", ret);
		return ret;
	}
	return ret;
}

static int gyro_report_value(struct i2c_client *client, struct sensor_axis *axis)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *)i2c_get_clientdata(client);

	/* Report acceleration sensor information */
	input_report_rel(sensor->input_dev, ABS_RX, axis->x);
	input_report_rel(sensor->input_dev, ABS_RY, axis->y);
	input_report_rel(sensor->input_dev, ABS_RZ, axis->z);
	input_sync(sensor->input_dev);
	DBG("Gyro sensor x==%d  y==%d z==%d\n", axis->x, axis->y, axis->z);
	return 0;
}

static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);
	struct sensor_platform_data *pdata = sensor->pdata;
	int ret = 0;
	short x, y, z;
	struct sensor_axis axis;
	u8 buffer[6] = {0};
	char value = 0;

	if (sensor->ops->read_len < 6) {
		printk("%s:len is error,len=%d\n", __func__, sensor->ops->read_len);
		return -1;
	}
	memset(buffer, 0, 6);
	do {
		*buffer = sensor->ops->read_reg;
		ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
		if (ret < 0)
			return ret;
	} while (0);

	x = ((buffer[0] << 8) & 0xFF00) + (buffer[1] & 0xFF);
	y = ((buffer[2] << 8) & 0xFF00) + (buffer[3] & 0xFF);
	z = ((buffer[4] << 8) & 0xFF00) + (buffer[5] & 0xFF);

	axis.x = (pdata->orientation[0])*x + (pdata->orientation[1])*y + (pdata->orientation[2])*z;
	axis.y = (pdata->orientation[3])*x + (pdata->orientation[4])*y + (pdata->orientation[5])*z;
	axis.z = (pdata->orientation[6])*x + (pdata->orientation[7])*y + (pdata->orientation[8])*z;

	/*filter gyro data*/
	if ((abs(axis.x) > pdata->x_min) || (abs(axis.y) > pdata->y_min) || (abs(axis.z) > pdata->z_min)) {
		gyro_report_value(client, &axis);

		mutex_lock(&sensor->data_mutex);
		sensor->axis = axis;
		mutex_unlock(&sensor->data_mutex);
	}
	/*read sensor intterupt status register*/
	if ((sensor->pdata->irq_enable) && (sensor->ops->int_status_reg >= 0)) {
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		DBG("%s:gyro sensor int status :0x%x\n", __func__, value);
	}
	return ret;
}

struct sensor_operate gyro_mpu6880_ops = {
	.name				= "mpu6880_gyro",
	.type				= SENSOR_TYPE_GYROSCOPE,			/*sensor type and it should be correct*/
	.id_i2c				= GYRO_ID_MPU6880,					/*i2c id number*/
	.read_reg			= MPU6880_GYRO_XOUT_H,				/*read data*/
	.read_len			= 6,								/*data length*/
	.id_reg				= MPU6880_WHOAMI,					/*read device id from this register*/
	.id_data			= MPU6880_DEVICE_ID,				/*device id*/
	.precision			= MPU6880_PRECISION,				/*16 bit*/
	.ctrl_reg			= MPU6880_PWR_MGMT_2,				/*enable or disable */
	.int_status_reg		= MPU6880_INT_STATUS,				/*intterupt status register*/
	.range				= {-MPU6880_RANGE, MPU6880_RANGE},	/*range*/
	.trig				= IRQF_TRIGGER_HIGH | IRQF_ONESHOT | IRQF_SHARED,
	.active				= sensor_active,
	.init				= sensor_init,
	.report				= sensor_report_value,
};

/****************operate according to sensor chip:end************/

/*function name should not be changed*/
static struct sensor_operate *gyro_get_ops(void)
{
	return &gyro_mpu6880_ops;
}

static int __init gyro_mpu6880_init(void)
{
	struct sensor_operate *ops = gyro_get_ops();
	int result = 0;
	int type = ops->type;

	result = sensor_register_slave(type, NULL, NULL, gyro_get_ops);
	return result;
}

static void __exit gyro_mpu6880_exit(void)
{
	struct sensor_operate *ops = gyro_get_ops();
	int type = ops->type;

	sensor_unregister_slave(type, NULL, NULL, gyro_get_ops);
}

/*register mpu6880_gyro after mpu6880_acc*/
device_initcall_sync(gyro_mpu6880_init);  /*module_init  device_initcall_sync*/
module_exit(gyro_mpu6880_exit);
