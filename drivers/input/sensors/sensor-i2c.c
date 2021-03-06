/* drivers/input/sensors/sensor-i2c.c - sensor i2c handle
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

#define SENSOR_I2C_RATE 200*1000


static int sensor_i2c_write(struct i2c_adapter *i2c_adap,
			    unsigned char address,
			    unsigned int len, unsigned char const *data)
{
	struct i2c_msg msgs[1];
	int res;

	if (!data || !i2c_adap) {
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return -EINVAL;
	}

	msgs[0].addr = address;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = (unsigned char *)data;
	msgs[0].len = len;
	//msgs[0].scl_rate = SENSOR_I2C_RATE;

	res = i2c_transfer(i2c_adap, msgs, 1);
	if (res == 1)
		return 0;
	else if(res == 0)
		return -EBUSY;
	else
		return res;

}

static int senosr_i2c_read(struct i2c_client *client, struct i2c_adapter *i2c_adap,
			   unsigned char address, unsigned char reg,
			   unsigned int len, unsigned char *data)
{
	struct sensor_private_data *sensor = (struct sensor_private_data *)i2c_get_clientdata(client);
	struct i2c_msg msgs[2];
	int res;

	if (!data || !i2c_adap) {
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return -EINVAL;
	}

	msgs[0].addr = address;
	if ((sensor->ops->id_i2c == LIGHT_ID_CM36682) ||
		(sensor->ops->id_i2c == PROXIMITY_ID_CM36682) ||
		(sensor->ops->id_i2c == ACCEL_ID_MMA845X)) {
		msgs[0].flags = I2C_M_NOSTART;/* i2c read , no stop*/	/* write */
	} else {
		msgs[0].flags = 0;	/* write */
	}
	msgs[0].buf = &reg;
	msgs[0].len = 1;
	//msgs[0].scl_rate = SENSOR_I2C_RATE;
	
	msgs[1].addr = address;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = data;
	msgs[1].len = len;
	//msgs[1].scl_rate = SENSOR_I2C_RATE;	

	res = i2c_transfer(i2c_adap, msgs, 2);
	if (res == 2)
		return 0;
	else if(res == 0)
		return -EBUSY;
	else
		return res;

}


int sensor_rx_data(struct i2c_client *client, unsigned char *rxData, int length)
{
	//struct sensor_private_data* sensor = 
	//	(struct sensor_private_data *)i2c_get_clientdata(client);
	int ret = 0;
	unsigned char reg = rxData[0];

	ret = senosr_i2c_read(client, client->adapter, client->addr, reg, length, rxData);
	
	/*DBG("=2014-7-11-9-30===addr=0x%x,len=%d,client->addr=0x%x",reg,length,client->addr);
	for(i=0; i<length; i++)
		DBG("0x%x,",rxData[i]);
	DBG("\n");*/
	return ret;
}
EXPORT_SYMBOL(sensor_rx_data);

int sensor_tx_data(struct i2c_client *client, char *txData, int length)
{
	//struct sensor_private_data* sensor = 
		//(struct sensor_private_data *)i2c_get_clientdata(client);
	int i = 0;
	int ret = 0;

	DBG("addr=0x%x,len=%d,txdata:",txData[0],length);
	for(i=1; i<length; i++)
		DBG("0x%x,",txData[i]);
	DBG("\n");
	ret = sensor_i2c_write(client->adapter, client->addr, length, txData);
	return ret;

}
EXPORT_SYMBOL(sensor_tx_data);

int sensor_write_reg(struct i2c_client *client, int addr, int value)
{
	char buffer[2];
	int ret = 0;
	struct sensor_private_data* sensor = 
		(struct sensor_private_data *)i2c_get_clientdata(client);
	
	mutex_lock(&sensor->i2c_mutex);	
	buffer[0] = addr;
	buffer[1] = value;
	ret = sensor_tx_data(client, &buffer[0], 2);	
	mutex_unlock(&sensor->i2c_mutex);	
	return ret;
}
EXPORT_SYMBOL(sensor_write_reg);

int sensor_read_reg(struct i2c_client *client, int addr)
{
	unsigned char tmp[1] = {0};
	int ret = 0;	
	struct sensor_private_data* sensor = 
		(struct sensor_private_data *)i2c_get_clientdata(client);
	
	mutex_lock(&sensor->i2c_mutex);	
	tmp[0] = addr;
	ret = sensor_rx_data(client, tmp, 1);
	mutex_unlock(&sensor->i2c_mutex);
	
	return tmp[0];
}

EXPORT_SYMBOL(sensor_read_reg);

int sensor_read_word_reg(struct i2c_client *client, uint8_t addr)
{
	uint8_t data[2] = {0};
	uint16_t value = 0;
	data[0] = addr;
	sensor_rx_data(client, data, 2);
	value = (data[1] << 8) | data[0] ;
	/*printk("data[0]=[0x%x], data[1]=[0x%x]\n", data[0],data[1]);*/
	return value;
}
EXPORT_SYMBOL(sensor_read_word_reg);

int sensor_write_word_reg(struct i2c_client *client, int addr, int value)
{
	uint8_t buffer[3];
	int ret = 0;
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);

	mutex_lock(&sensor->i2c_mutex);
	buffer[0] = addr;
	buffer[1] = (uint8_t)(value & 0xff);
	buffer[2] = (uint8_t)((value & 0xff00) >> 8);
	ret = sensor_tx_data(client, &buffer[0], 3);
	mutex_unlock(&sensor->i2c_mutex);
	return ret;
}
EXPORT_SYMBOL(sensor_write_word_reg);

static int i2c_master_normal_recv(const struct i2c_client *client, char *buf, int count, int scl_rate)
 {
     struct i2c_adapter *adap=client->adapter;
     struct i2c_msg msg;
    int ret;
 
    msg.addr = client->addr;
    msg.flags = client->flags | I2C_M_RD;
	msg.len = count;
	msg.buf = (char *)buf;
	//msg.scl_rate = scl_rate;
	ret = i2c_transfer(adap, &msg, 1);

		 return (ret == 1) ? count : ret;
}

static int i2c_master_normal_send(const struct i2c_client *client, const char *buf, int count, int scl_rate)
{
	int ret;
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg; 

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = count;
	msg.buf = (char *)buf;
	//msg.scl_rate = scl_rate;

	ret = i2c_transfer(adap, &msg, 1);
	return (ret == 1) ? count : ret;
}

int sensor_tx_data_normal(struct i2c_client *client, char *buf, int num)
{
	int ret = 0;
	ret = i2c_master_normal_send(client, buf, num, SENSOR_I2C_RATE);
	
	return (ret == num) ? 0 : ret;
}
EXPORT_SYMBOL(sensor_tx_data_normal);


int sensor_rx_data_normal(struct i2c_client *client, char *buf, int num)
{
	int ret = 0;
	ret = i2c_master_normal_recv(client, buf, num, SENSOR_I2C_RATE);
	
	return (ret == num) ? 0 : ret;
}

EXPORT_SYMBOL(sensor_rx_data_normal);


int sensor_write_reg_normal(struct i2c_client *client, char value)
{
	char buffer[2];
	int ret = 0;
	struct sensor_private_data* sensor = 
		(struct sensor_private_data *)i2c_get_clientdata(client);
	
	mutex_lock(&sensor->i2c_mutex);	
	buffer[0] = value;
	ret = sensor_tx_data_normal(client, &buffer[0], 1);	
	mutex_unlock(&sensor->i2c_mutex);	
	return ret;
}
EXPORT_SYMBOL(sensor_write_reg_normal);

int sensor_read_reg_normal(struct i2c_client *client)
{
	char tmp[1] = {0};
	int ret = 0;	
	struct sensor_private_data* sensor = 
		(struct sensor_private_data *)i2c_get_clientdata(client);
	
	mutex_lock(&sensor->i2c_mutex);	
	ret = sensor_rx_data_normal(client, tmp, 1);
	mutex_unlock(&sensor->i2c_mutex);
	
	return tmp[0];
}

EXPORT_SYMBOL(sensor_read_reg_normal);

