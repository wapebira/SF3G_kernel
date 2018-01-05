/*
 * kernel/drivers/input/sensors/compass/akm8963_x86.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: luowei <lw@rock-chips.com>
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
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/sensor-dev.h>

#define SENSOR_DATA_SIZE	8
#define YPR_DATA_SIZE		26
#define RWBUF_SIZE		16

#define ACC_DATA_FLAG		0
#define MAG_DATA_FLAG		1
#define ORI_DATA_FLAG		1
#define AKM_NUM_SENSORS		5

#define ACC_DATA_READY		(1 << (ACC_DATA_FLAG))
#define MAG_DATA_READY		(1 << (MAG_DATA_FLAG))
#define ORI_DATA_READY		(1 << (ORI_DATA_FLAG))

/*! \name AK8963 constant definition
 \anchor AK8963_Def
 Constant definitions of the AK8963.*/
#define AK8963_MEASUREMENT_TIME_US	10000

/*! \name AK8963 operation mode
 \anchor AK8963_Mode
 Defines an operation mode of the AK8963.*/
/*! @{*/
#define AK8963_MODE_SNG_MEASURE	0x01
#define	AK8963_MODE_SELF_TEST	0x08
#define	AK8963_MODE_FUSE_ACCESS	0x0F
#define	AK8963_MODE_POWERDOWN	0x00

/*! @}*/

/*! \name AK8963 register address
\anchor AK8963_REG
Defines a register address of the AK8963.*/
/*! @{*/
#define AK8963_REG_WIA		0x00
#define AK8963_REG_INFO		0x01
#define AK8963_REG_ST1		0x02
#define AK8963_REG_HXL		0x03
#define AK8963_REG_HXH		0x04
#define AK8963_REG_HYL		0x05
#define AK8963_REG_HYH		0x06
#define AK8963_REG_HZL		0x07
#define AK8963_REG_HZH		0x08
#define AK8963_REG_ST2		0x09
#define AK8963_REG_CNTL1	0x0A
#define AK8963_REG_CNTL2	0x0B
#define AK8963_REG_ASTC		0x0C
#define AK8963_REG_TS1		0x0D
#define AK8963_REG_TS2		0x0E
#define AK8963_REG_I2CDIS	0x0F
/*! @}*/

/*! \name AK8963 fuse-rom address
\anchor AK8963_FUSE
Defines a read-only address of the fuse ROM of the AK8963.*/
/*! @{*/
#define AK8963_FUSE_ASAX	0x10
#define AK8963_FUSE_ASAY	0x11
#define AK8963_FUSE_ASAZ	0x12
/*! @}*/
#define AK8963_INFO_SIZE	2
#define AK8963_CONF_SIZE	3
#define AK8963_INFO_DATA	(0x03 << 3)

#define AKM_SENSOR_INFO_SIZE        AK8963_INFO_SIZE
#define AKM_SENSOR_CONF_SIZE        AK8963_CONF_SIZE
#define AKM_SENSOR_DATA_SIZE        9

/* IOCTLs for Msensor misc. device library */
#define MSENSOR				0x83

#define ECS_IOCTL_GET_INFO              _IOR(MSENSOR, 0x27, \
	unsigned char[AKM_SENSOR_INFO_SIZE])
#define ECS_IOCTL_GET_CONF              _IOR(MSENSOR, 0x28, \
	unsigned char[AKM_SENSOR_CONF_SIZE])
#define ECS_IOCTL_SET_YPR_09911         _IOW(MSENSOR, 0x29, int[YPR_DATA_SIZE])
#define ECS_IOCTL_GET_DELAY_09911       _IOR(MSENSOR, 0x30, int64_t[3])
#define	ECS_IOCTL_GET_LAYOUT_09911      _IOR(MSENSOR, 0x31, char)
/* IOCTLs for AKM library */
#define ECS_IOCTL_WRITE                 _IOW(MSENSOR, 0x0b, char*)
#define ECS_IOCTL_READ                  _IOWR(MSENSOR, 0x0c, char*)
/* NOT used in AK8975 */
#define ECS_IOCTL_RESET                 _IO(MSENSOR, 0x0d)
#define ECS_IOCTL_SET_MODE              _IOW(MSENSOR, 0x0e, short)
#define ECS_IOCTL_GET_DATA              _IOR(MSENSOR, 0x0f, \
	char[AKM_SENSOR_DATA_SIZE])
#define ECS_IOCTL_SET_YPR               _IOW(MSENSOR, 0x10, short[12])
#define ECS_IOCTL_GET_OPEN_STATUS       _IOR(MSENSOR, 0x11, int)
#define ECS_IOCTL_GET_CLOSE_STATUS      _IOR(MSENSOR, 0x12, int)
#define ECS_IOCTL_GET_OSENSOR_STATUS    _IOR(MSENSOR, 0x13, int)
#define ECS_IOCTL_GET_DELAY             _IOR(MSENSOR, 0x14, short)
#define ECS_IOCTL_GET_PROJECT_NAME      _IOR(MSENSOR, 0x15, char[64])
#define ECS_IOCTL_GET_MATRIX            _IOR(MSENSOR, 0x16, short [4][3][3])
#define	ECS_IOCTL_GET_LAYOUT            _IOR(MSENSOR, 0x17, int[3])
#define ECS_IOCTL_GET_OUTBIT            _IOR(MSENSOR, 0x23, char)
#define ECS_IOCTL_GET_ACCEL             _IOR(MSENSOR, 0x24, short[3])

#define AK8963_DEVICE_ID		0x48
static struct i2c_client *this_client;
static struct miscdevice compass_dev_device;

static int g_akm_rbuf[YPR_DATA_SIZE];
static char g_sensor_info[AK8963_INFO_SIZE];
static char g_sensor_conf[AK8963_CONF_SIZE];

/****************operate according to sensor chip:start************/
static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *)i2c_get_clientdata(client);
	int result = 0;

	/*register setting according to chip datasheet*/
	if (enable)
		sensor->ops->ctrl_data = AK8963_MODE_SNG_MEASURE;
	else
		sensor->ops->ctrl_data = AK8963_MODE_POWERDOWN;

	DBG("%s:reg=0x%x,reg_ctrl=0x%x,enable=%d\n",
	    __func__,
	    sensor->ops->ctrl_reg,
	    sensor->ops->ctrl_data,
	    enable);
	result = sensor_write_reg(client,
				  sensor->ops->ctrl_reg,
				  sensor->ops->ctrl_data);
	if (result)
		dev_err(&client->dev, "%s:fail to active sensor\n", __func__);

	return result;
}

static int sensor_init(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *)i2c_get_clientdata(client);
	int result = 0;
	char info = 0;

	this_client = client;
	result = sensor->ops->active(client, 0, 0);
	if (result) {
		dev_err(&client->dev, "%s:line=%d,error\n", __func__, __LINE__);
		return result;
	}

	sensor->status_cur = SENSOR_OFF;
	info = sensor_read_reg(client, AK8963_REG_INFO);
	if ((info & (0x0f<<3)) != AK8963_INFO_DATA) {
		dev_err(&client->dev, "%s:info=0x%x,it is not %s\n",
			__func__,
			info,
			sensor->ops->name);
			return -1;
	}

	result = misc_register(&compass_dev_device);
	if (result < 0) {
		dev_err(&client->dev, "%s:fail to register misc device %s\n",
			__func__,
			compass_dev_device.name);
		result = -1;
	}
	g_sensor_info[0] = AK8963_REG_WIA;
	result = sensor_rx_data(client, g_sensor_info, AK8963_INFO_SIZE);
	if (result)	{
		dev_err(&client->dev, "%s:line=%d,error\n", __func__, __LINE__);
		return result;
	}

	g_sensor_conf[0] = AK8963_FUSE_ASAX;
	result = sensor_rx_data(client, g_sensor_conf, AK8963_CONF_SIZE);
	if (result)	{
		dev_err(&client->dev, "%s:line=%d,error\n", __func__, __LINE__);
		return result;
	}

	DBG("%s:status_cur=%d\n", __func__, sensor->status_cur);
	return result;
}

static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);
	char buffer[8] = {0};
	unsigned char *stat;
	unsigned char *stat2;
	int ret = 0;
	char value = 0;
	int i;

	if (sensor->ops->read_len < 8) {
		dev_err(&client->dev, "%s:Readlenth is error,len=%d\n",
			__func__,
			sensor->ops->read_len);
		return -1;
	}
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	*buffer = sensor->ops->read_reg;
	ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
	if (ret < 0)
		return ret;

	stat = &buffer[0];
	stat2 = &buffer[7];

	/*
	 * ST : data ready -
	 * Measurement has been completed and data is ready to be read.
	 */
	if ((*stat & 0x01) != 0x01) {
		DBG(KERN_ERR "%s:ST is not set\n", __func__);
		return -1;
	}

	/*
	 * ST2 : data error -
	 * occurs when data read is started outside of a readable period;
	 * data read would not be correct.
	 * Valid in continuous measurement mode only.
	 * In single measurement mode this error should not occour but we
	 * stil account for it and return an error, since the data would be
	 * corrupted.
	 * DERR bit is self-clearing when ST2 register is read.
	 */
	if (*stat2 & 0x04) {
		DBG(KERN_ERR "%s:compass data error\n", __func__);
		return -2;
	}

	/*
	 * ST2 : overflow -
	 * the sum of the absolute values of all axis |X|+|Y|+|Z| < 2400uT.
	 * This is likely to happen in presence of an external magnetic
	 * disturbance; it indicates, the sensor data is incorrect and should
	 * be ignored.
	 * An error is returned.
	 * HOFL bit clears when a new measurement starts.
	 */
	if (*stat2 & 0x08) {
		DBG(KERN_ERR "%s:compass data overflow\n", __func__);
		return -3;
	}

	mutex_lock(&sensor->data_mutex);
	memcpy(sensor->sensor_data, buffer, sensor->ops->read_len);
	mutex_unlock(&sensor->data_mutex);
	DBG("%s:", __func__);
	for (i = 0; i < sensor->ops->read_len; i++)
		DBG("0x%x,", buffer[i]);
	DBG("\n");

	if ((sensor->pdata->irq_enable) && (sensor->ops->int_status_reg >= 0)) {
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		DBG("%s:sensor int status :0x%x\n", __func__, value);
	}

	/*trigger next measurement */
	ret = sensor_write_reg(client,
			       sensor->ops->ctrl_reg,
			       sensor->ops->ctrl_data);
	if (ret) {
		dev_err(&client->dev, "%s:fail to set ctrl_data:0x%x\n",
			__func__,
			sensor->ops->ctrl_data);
		return ret;
	}

	return ret;
}

static void compass_set_YPR(int *rbuf)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *)i2c_get_clientdata(this_client);

	/* No events are reported */
	if (!rbuf[0]) {
		dev_err(&sensor->client->dev,
			"%s:Don't waste a time.",
			__func__);
		return;
	}
	DBG("%s:buf[0]=0x%x\n", __func__, rbuf[0]);

	/* Report magnetic sensor information */
	if (atomic_read(&sensor->flags.m_flag) && (rbuf[0] & ORI_DATA_READY)) {
		input_report_abs(sensor->input_dev, ABS_RX, rbuf[13]);
		input_report_abs(sensor->input_dev, ABS_RY, rbuf[14]);
		input_report_abs(sensor->input_dev, ABS_RZ, rbuf[15]);
		input_report_abs(sensor->input_dev, ABS_RUDDER, rbuf[4]);
		DBG("%s:m_flag:x=%d,y=%d,z=%d,RUDDER=%d\n",
		    __func__,
		    rbuf[13],
		    rbuf[14],
		    rbuf[15],
		    rbuf[4]);
	}

	/* Report acceleration sensor information */
	if (atomic_read(&sensor->flags.a_flag) && (rbuf[0] & ACC_DATA_READY)) {
		input_report_abs(sensor->input_dev, ABS_X, rbuf[1]);
		input_report_abs(sensor->input_dev, ABS_Y, rbuf[2]);
		input_report_abs(sensor->input_dev, ABS_Z, rbuf[3]);
		input_report_abs(sensor->input_dev, ABS_WHEEL, rbuf[4]);
		DBG("%s:a_flag:x=%d,y=%d,z=%d,WHEEL=%d\n",
		    __func__,
		    rbuf[1],
		    rbuf[2],
		    rbuf[3],
		    rbuf[4]);
	}

	/* Report magnetic vector information */
	if (atomic_read(&sensor->flags.mv_flag) && (rbuf[0] & MAG_DATA_READY)) {
		input_report_abs(sensor->input_dev, ABS_HAT0X, rbuf[5]);
		input_report_abs(sensor->input_dev, ABS_HAT0Y, rbuf[6]);
		input_report_abs(sensor->input_dev, ABS_BRAKE, rbuf[7]);
		input_report_abs(sensor->input_dev, ABS_HAT1X, rbuf[8]);
		DBG("%s:mv_flag:x=%d,y=%d,z=%d,status=%d\n",
		    __func__,
		    rbuf[5],
		    rbuf[6],
		    rbuf[7],
		    rbuf[8]);
	}
	input_sync(sensor->input_dev);
	memcpy(g_akm_rbuf, rbuf, YPR_DATA_SIZE);
}

static int compass_dev_open(struct inode *inode, struct file *file)
{
	int result = 0;

	DBG("%s\n", __func__);

	return result;
}

static int compass_dev_release(struct inode *inode, struct file *file)
{
	int result = 0;

	DBG("%s\n", __func__);

	return result;
}

static int compass_akm_set_mode(struct i2c_client *client, char mode)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(this_client);
	int result = 0;

	switch (mode & 0x0f) {
	case AK8963_MODE_SNG_MEASURE:
	case AK8963_MODE_SELF_TEST:
	case AK8963_MODE_FUSE_ACCESS:
		if (sensor->status_cur == SENSOR_OFF) {
			if (!sensor->pdata->irq_enable)
				schedule_delayed_work(&sensor->delaywork,
						      msecs_to_jiffies(
						 sensor->pdata->poll_delay_ms));
			sensor->status_cur = SENSOR_ON;
		}
		break;

	case AK8963_MODE_POWERDOWN:
		if (sensor->status_cur == SENSOR_ON) {
			if (!sensor->pdata->irq_enable)
				cancel_delayed_work_sync(&sensor->delaywork);
			sensor->status_cur = SENSOR_OFF;
		}
		break;
	}

	switch (mode & 0x0f) {
	case AK8963_MODE_SNG_MEASURE:
		result = sensor_write_reg(client, sensor->ops->ctrl_reg, mode);
		if (result)
			dev_err(&client->dev,
				"%s:i2c error,mode=%d\n",
				__func__,
				mode);
		break;
	case AK8963_MODE_SELF_TEST:
		result = sensor_write_reg(client, sensor->ops->ctrl_reg, mode);
		if (result)
			dev_err(&client->dev,
				"%s:i2c error,mode=%d\n",
				__func__,
				mode);
		break;
	case AK8963_MODE_FUSE_ACCESS:
		result = sensor_write_reg(client, sensor->ops->ctrl_reg, mode);
		if (result)
			dev_err(&client->dev,
				"%s:i2c error,mode=%d\n",
				__func__,
				mode);
		break;
	case AK8963_MODE_POWERDOWN:
		/* Set powerdown mode */
		result = sensor_write_reg(client,
					  sensor->ops->ctrl_reg,
					  AK8963_MODE_POWERDOWN);
		if (result)
			dev_err(&client->dev,
				"%s:i2c error,mode=%d\n",
				__func__, mode);
		break;
	default:
		dev_err(&client->dev, "%s: Unknown mode(%d)", __func__, mode);
		result = -EINVAL;
		break;
	}
	DBG("%s:mode=0x%x\n", __func__, mode);
	return result;
}

static int compass_akm_reset(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(this_client);
	int result = 0;

	if (sensor->pdata->reset_pin > 0) {
		gpio_direction_output(sensor->pdata->reset_pin, GPIO_LOW);
		gpio_direction_output(sensor->pdata->reset_pin, GPIO_HIGH);
	} else {
		/* Set measure mode */
		result = sensor_write_reg(client,
					  AK8963_REG_CNTL2,
					  AK8963_MODE_SNG_MEASURE);
		if (result)
			dev_err(&client->dev,
				"%s:fail to Set measure mode\n",
				__func__);
	}

	return result;
}

static int compass_akm_get_openstatus(void)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(this_client);
	wait_event_interruptible(sensor->flags.open_wq,
				 (atomic_read(&sensor->flags.open_flag) != 0));
	return atomic_read(&sensor->flags.open_flag);
}

static int compass_akm_get_closestatus(void)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(this_client);
	wait_event_interruptible(sensor->flags.open_wq,
				 (atomic_read(&sensor->flags.open_flag) <= 0));
	return atomic_read(&sensor->flags.open_flag);
}

/* ioctl - I/O control */
static long compass_dev_ioctl(struct file *file,
			      unsigned int cmd,
			      unsigned long arg)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(this_client);
	struct i2c_client *client = this_client;
	void __user *argp = (void __user *)arg;
	int result = 0;
	/*struct akm_platform_data compass;*/

	/* NOTE: In this function the size of "char" should be 1-byte. */
	char compass_data[SENSOR_DATA_SIZE];	/* for GETDATA */
	char rwbuf[RWBUF_SIZE];/* for READ/WRITE */
	char mode;				/* for SET_MODE*/
	int value[YPR_DATA_SIZE];			/* for SET_YPR */
	int status;				/* for OPEN/CLOSE_STATUS */
	int ret = -1;				/* Return value. */

	int16_t acc_buf[3];/* for GET_ACCEL */
	int64_t delay[AKM_NUM_SENSORS];/* for GET_DELAY */

	char layout;		/* for GET_LAYOUT */
	char outbit;		/* for GET_OUTBIT */

	switch (cmd) {
	case ECS_IOCTL_WRITE:
	case ECS_IOCTL_READ:
		if (argp == NULL)
			return -EINVAL;
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case ECS_IOCTL_SET_MODE:
		if (argp == NULL)
			return -EINVAL;
		if (copy_from_user(&mode, argp, sizeof(mode)))
			return -EFAULT;
		break;
	case ECS_IOCTL_SET_YPR:
		if (argp == NULL)
			return -EINVAL;
		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GET_DATA:
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
	case ECS_IOCTL_GET_DELAY:
	case ECS_IOCTL_GET_LAYOUT:
	case ECS_IOCTL_GET_OUTBIT:
	case ECS_IOCTL_GET_ACCEL:
		/* Just check buffer pointer */
		if (argp == NULL) {
			dev_err(&sensor->client->dev,
				"%s:invalid argument\n",
				__func__);
			return -EINVAL;
		}
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_WRITE:
		DBG("%s:ECS_IOCTL_WRITE start\n", __func__);
		mutex_lock(&sensor->operation_mutex);
		if ((rwbuf[0] < 2) || (rwbuf[0] > (RWBUF_SIZE-1))) {
			mutex_unlock(&sensor->operation_mutex);
			return -EINVAL;
		}
		ret = sensor_tx_data(client, &rwbuf[1], rwbuf[0]);
		if (ret < 0) {
			mutex_unlock(&sensor->operation_mutex);
			dev_err(&sensor->client->dev,
				"%s:fait to tx data\n",
				__func__);
			return ret;
		}
		mutex_unlock(&sensor->operation_mutex);
		break;
	case ECS_IOCTL_READ:
		DBG("%s:ECS_IOCTL_READ start\n", __func__);
		mutex_lock(&sensor->operation_mutex);
		if ((rwbuf[0] < 1) || (rwbuf[0] > (RWBUF_SIZE-1))) {
			mutex_unlock(&sensor->operation_mutex);
			dev_err(&sensor->client->dev,
				"%s:data is error\n",
				__func__);
			return -EINVAL;
		}
		ret = sensor_rx_data(client, &rwbuf[1], rwbuf[0]);
		if (ret < 0) {
			mutex_unlock(&sensor->operation_mutex);
			dev_err(&sensor->client->dev,
				"%s:fait to rx data\n",
				__func__);
			return ret;
		}
		mutex_unlock(&sensor->operation_mutex);
		break;
	case ECS_IOCTL_SET_MODE:
		DBG("%s:ECS_IOCTL_SET_MODE start\n", __func__);
		mutex_lock(&sensor->operation_mutex);
		if (sensor->ops->ctrl_data != mode) {
			ret = compass_akm_set_mode(client, mode);
			if (ret < 0) {
				dev_err(&sensor->client->dev,
					"%s:fait to set mode\n",
					__func__);
				mutex_unlock(&sensor->operation_mutex);
				return ret;
			}
			sensor->ops->ctrl_data = mode;
		}
		mutex_unlock(&sensor->operation_mutex);
		break;
	case ECS_IOCTL_GET_DATA:
			DBG("%s:ECS_IOCTL_GETDATA start\n", __func__);
			mutex_lock(&sensor->data_mutex);
			/*get data from buffer*/
			memcpy(compass_data,
			       sensor->sensor_data,
			       SENSOR_DATA_SIZE);
			mutex_unlock(&sensor->data_mutex);
			break;
	case ECS_IOCTL_SET_YPR:
			DBG("%s:ECS_IOCTL_SET_YPR start\n", __func__);
			mutex_lock(&sensor->data_mutex);
			compass_set_YPR(value);
			mutex_unlock(&sensor->data_mutex);
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
		status = compass_akm_get_openstatus();
		DBG("%s:openstatus=%d\n", __func__, status);
		break;
	case ECS_IOCTL_GET_CLOSE_STATUS:
		status = compass_akm_get_closestatus();
		DBG("%s:closestatus=%d\n", __func__, status);
		break;
	case ECS_IOCTL_GET_DELAY:
		DBG("%s:ECS_IOCTL_GET_DELAY start\n", __func__);
		mutex_lock(&sensor->operation_mutex);
		delay[0] = sensor->flags.delay;
		delay[1] = sensor->flags.delay;
		delay[2] = sensor->flags.delay;
		delay[3] = sensor->flags.delay;
		delay[4] = sensor->flags.delay;
		mutex_unlock(&sensor->operation_mutex);
		break;
	case ECS_IOCTL_GET_LAYOUT:
		DBG("%s:ECS_IOCTL_GET_LAYOUT start\n", __func__);
		layout = sensor->pdata->layout;
		break;
	case ECS_IOCTL_GET_OUTBIT:
		DBG("%s:ECS_IOCTL_GET_OUTBIT start\n", __func__);
		outbit = 1; /*sensor->pdata->outbit;*/
		break;
	case ECS_IOCTL_RESET:
		DBG("%s:ECS_IOCTL_RESET start\n", __func__);
		ret = compass_akm_reset(client);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_GET_ACCEL:
		DBG("%s:ECS_IOCTL_GET_ACCEL start,no accel data\n", __func__);
		mutex_lock(&sensor->operation_mutex);
		acc_buf[0] = g_akm_rbuf[6];
		acc_buf[1] = g_akm_rbuf[7];
		acc_buf[2] = g_akm_rbuf[8];
		mutex_unlock(&sensor->operation_mutex);
		break;
	case ECS_IOCTL_GET_INFO:
		DBG("%sECS_IOCTL_GET_INFO\n", __func__);
		ret = copy_to_user(argp, g_sensor_info, sizeof(g_sensor_info));
		if (ret < 0) {
			dev_err(&sensor->client->dev,
				"%s:error,ret=%d\n",
				__func__,
				ret);
			return ret;
		}
		break;
	case ECS_IOCTL_GET_CONF:
		DBG("%s:ECS_IOCTL_GET_ACCEL start,no accel data\n", __func__);
		ret = copy_to_user(argp, g_sensor_conf, sizeof(g_sensor_conf));
		if (ret < 0) {
			dev_err(&sensor->client->dev,
				"%s:error,ret=%d\n",
				__func__,
				ret);
			return ret;
		}
		break;

	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		if (copy_to_user(argp, &rwbuf, rwbuf[0]+1))
			return -EFAULT;
		break;
	case ECS_IOCTL_GET_DATA:
		if (copy_to_user(argp, &compass_data, sizeof(compass_data)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
		if (copy_to_user(argp, &status, sizeof(status)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GET_DELAY:
		if (copy_to_user(argp, &delay, sizeof(delay)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GET_LAYOUT:
		if (copy_to_user(argp, &layout, sizeof(layout))) {
			dev_err(&sensor->client->dev,
				"%s:error:%d\n",
				__func__,
				__LINE__);
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_OUTBIT:
		if (copy_to_user(argp, &outbit, sizeof(outbit))) {
			dev_err(&sensor->client->dev,
				"%s:error:%d\n",
				__func__,
				__LINE__);
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_ACCEL:
		if (copy_to_user(argp, &acc_buf, sizeof(acc_buf))) {
			dev_err(&sensor->client->dev,
				"%s:error:%d\n",
				__func__,
				__LINE__);
			return -EFAULT;
		}
		break;
	default:
		break;
	}

	return result;
}

static struct file_operations compass_dev_fops = {
	.owner = THIS_MODULE,
	.open = compass_dev_open,
	.release = compass_dev_release,
	.unlocked_ioctl = compass_dev_ioctl,
};

static struct miscdevice compass_dev_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "akm8963_dev",
	.fops = &compass_dev_fops,
};

struct sensor_operate compass_akm8963_ops = {
	.name			= "akm8963",
	.type			= SENSOR_TYPE_COMPASS,/*it is important*/
	.id_i2c			= COMPASS_ID_AK8963,
	.read_reg		= AK8963_REG_ST1,/*read data*/
	.read_len		= SENSOR_DATA_SIZE,/*data length*/
	.id_reg			= AK8963_REG_WIA,/*read id*/
	.id_data		= AK8963_DEVICE_ID,
	.precision		= 8,/*12 bits*/
	.ctrl_reg		= AK8963_REG_CNTL1,/*enable or disable*/
	.int_status_reg	= SENSOR_UNKNOW_DATA,/*not exist*/
	.range			= {-0xffff, 0xffff},
	/*if LEVEL interrupt then IRQF_ONESHOT*/
	.trig			= IRQF_TRIGGER_RISING,
	.active			= sensor_active,
	.init			= sensor_init,
	.report			= sensor_report_value,
	.misc_dev		= NULL,/*private misc support*/
};

/****************operate according to sensor chip:end************/
static struct sensor_operate *compass_get_ops(void)
{
	return &compass_akm8963_ops;
}

static int __init compass_akm8963_init(void)
{
	struct sensor_operate *ops = compass_get_ops();
	int result = 0;
	int type = ops->type;

	result = sensor_register_slave(type, NULL, NULL, compass_get_ops);

	return result;
}

static void __exit compass_akm8963_exit(void)
{
	struct sensor_operate *ops = compass_get_ops();
	int type = ops->type;

	sensor_unregister_slave(type, NULL, NULL, compass_get_ops);
}

module_init(compass_akm8963_init);
module_exit(compass_akm8963_exit);
