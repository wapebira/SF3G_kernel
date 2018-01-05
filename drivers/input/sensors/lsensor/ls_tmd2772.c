/* drivers/input/sensors/access/kxtik.c
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
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/sensor-dev.h>

#include "ls_tmd2772.h"

#define APS_TAG                  "[ALS/PS] "
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#if 0
#define APS_DBG(fmt, args...)    printk(APS_TAG fmt, ##args)              
#else
#define APS_DBG(fmt, args...)
#endif

int TMD2772_CMM_PPCOUNT_VALUE = 0x0B;
int ZOOM_TIME = 4;
int TMD2772_CMM_CONTROL_VALUE = 0x24;

static u16 als_deb_end = 0;
static bool als_deb_on = 1;

static u16 als_modulus = 0;

typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

static int tmd2772_enable_als(struct i2c_client *client, int enable, int rate)
{ 
	long res = 0;
	int read_value;
	int status = 0;
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	read_value = sensor_read_reg(client, TMD2772_CMM_ENABLE);

	if (enable) {
		if (sensor->pdata->irq_enable)
			status = 0x1B;
		else
			status = 0x0B;
		read_value |= status;
		als_deb_on = 1;
		als_deb_end = jiffies + 50 / (1000 / HZ);
		APS_DBG("lsensor tmd2772 power on\n");
	} else {
		if (read_value & 0x04)
			status = ~0x12;
		else
			status = ~0x1B;
		read_value &= status;

		als_deb_on = 0;
		APS_DBG("lsensor tmd2772 power off\n");
	}

	res = sensor_write_reg(client, TMD2772_CMM_ENABLE, read_value);
	if (res)	{
		APS_ERR("tmd2772_enable_ls fail\n");
		return res;
	}

	return 0;
}

static int tmd2772_check_and_clear_intr(struct i2c_client *client) 
{
	int res, value;
	u8 reg;

	value = sensor_read_reg(client, TMD2772_CMM_STATUS);
	if (value < 0)
		goto EXIT_ERR;
	if (0 != (value & 0x10)) {
		reg = (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_SPL_FN | 0x06);
		res = i2c_master_send(client, &reg, 0x1);
		if(res <= 0)
			goto EXIT_ERR;
	}
	return 0;

EXIT_ERR:
	APS_ERR("tmd2772_check_and_clear_intr fail\n");
	return 1;
}

static int tmd2772_init_client(struct i2c_client *client)
{
	int res = 0;

	/*client->addr = sensor->ops->slave_addr;*/

	/*res = sensor_write_reg(client, TMD2772_CMM_ENABLE, 0x00);
	if(res < 0)
		goto EXIT_ERR;*/

	res = sensor_write_reg(client, TMD2772_CMM_ATIME, 0xF6);
	if (res)
		goto EXIT_ERR;

	res = sensor_write_reg(client, TMD2772_CMM_PTIME, 0xFF);
	if (res)
		goto EXIT_ERR;

	res = sensor_write_reg(client, TMD2772_CMM_WTIME, 0xFC);
	if (res)
		goto EXIT_ERR;

	res = sensor_write_reg(client, TMD2772_CMM_CONFIG, 0x04);
	if (res)
		goto EXIT_ERR;

	res = sensor_write_reg(client, TMD2772_CMM_PPCOUNT, TMD2772_CMM_PPCOUNT_VALUE);
	if (res)
		goto EXIT_ERR;

	res = sensor_write_reg(client, TMD2772_CMM_CONTROL, TMD2772_CMM_CONTROL_VALUE);
	if (res < 0)
		goto EXIT_ERR;

	als_modulus = (400 * 100 * ZOOM_TIME) / (1 * 150);

	tmd2772_enable_als(client, 0, 0);

	return TMD2772_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

int tmd2772_read_als(struct i2c_client *client, u16 *data)
{ 
	u16 c0_value, c1_value;	 
	u32 c0_nf, c1_nf;
	int als_value_low, als_value_high;
	u16 atio;
	int res = 0;

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	als_value_low = sensor_read_reg(client, TMD2772_CMM_C0DATA_L);
	if (als_value_low < 0)
		goto EXIT_ERR;

	als_value_high = sensor_read_reg(client, TMD2772_CMM_C0DATA_H);
	if (als_value_high < 0)
		goto EXIT_ERR;

	c0_value = als_value_low | (als_value_high << 8);
	c0_nf = als_modulus * c0_value;

	APS_DBG("c0_value=%d, c0_nf=%d, als_modulus=%d\n", c0_value, c0_nf, als_modulus);

	//get adc channel 1 value
	als_value_low = sensor_read_reg(client, TMD2772_CMM_C1DATA_L);
	if(als_value_low < 0)
		goto EXIT_ERR;

	als_value_high = sensor_read_reg(client, TMD2772_CMM_C1DATA_H);
	if (als_value_high < 0)
		goto EXIT_ERR;

	c1_value = als_value_low | (als_value_high << 8);
	c1_nf = als_modulus * c1_value;

	APS_DBG("c1_value=%d, c1_nf=%d, als_modulus=%d\n", c1_value, c1_nf, als_modulus);

	if ((c0_value > c1_value) && (c0_value < 50000)) {
		atio =  (c1_nf * 100)  / c0_nf;

		APS_DBG("atio = %d\n", atio);
		if (atio < 30) {
			*data = (13 * c0_nf  - 24 * c1_nf) /10000;
		} else if (atio >= 30 && atio < 38) { 
			*data = (16 * c0_nf  - 35 * c1_nf) /10000;
		} else if (atio >= 38 && atio < 45) { 
			*data = (9 * c0_nf  - 17 * c1_nf)  /10000;
		} else if (atio >= 45 && atio < 54) { 
			*data = (6 * c0_nf  - 10 * c1_nf)  /10000;
		} else
			*data = 0;
	} else if (c0_value > 50000) {
		*data = 65535;
	} else if (c0_value == 0) {
		*data = 0;
	} else if (c0_value == 1 && c0_value == c1_value) {
		*data = 1;
	} else {
		APS_DBG("als_value is invalid!!\n");
		return -1;
	}

	APS_DBG("als_value_lux = %d\n", *data);
	return 0;

EXIT_ERR:
	APS_ERR("tmd2772_read_als fail\n");
	return res;
}

int tmd2772_read_als_ch0(struct i2c_client *client, u16 *data)
{
	u16 c0_value;
	u8 als_value_low, als_value_high;
	int res = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	als_value_low = sensor_read_reg(client, TMD2772_CMM_C0DATA_L);
	if(als_value_low < 0)
	{
		goto EXIT_ERR;
	}

	als_value_high = sensor_read_reg(client, TMD2772_CMM_C0DATA_H);
	if(als_value_high < 0)
	{
		goto EXIT_ERR;
	}

	c0_value = als_value_low | (als_value_high<<8);
	*data = c0_value;

	return 0;

EXIT_ERR:
	APS_ERR("tmd2772_read_als_ch0 fail\n");
	return res;
}

/*
static void tmd2772_eint_work(struct work_struct *work)
{
	int err;
	hwm_sensor_data sensor_data;
//	u8 buffer[1];
//	u8 reg_value[1];
	u8 databuf[2];
	int res = 0;

	if((err = tmd2772_check_intr(obj->client)))
	{
		APS_ERR("tmd2772_eint_work check intrs: %d\n", err);
	}
	else
	{
		//get raw data
		tmd2772_read_ps(obj->client, &obj->ps);
		//mdelay(160);
		tmd2772_read_als_ch0(obj->client, &obj->als);
		APS_DBG("tmd2772_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		//printk("tmd2772_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		sensor_data.values[0] = tmd2772_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;			
//singal interrupt function add
#if 1
		if(intr_flag_value){
				//printk("yucong interrupt value ps will < 750");
				databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;	
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;	
				databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;	
				databuf[1] = (u8)(0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH; 
				databuf[1] = (u8)((0xFF00) >> 8);;
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		} else {
				//printk("yucong interrupt value ps will > 900");
				databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;	
				databuf[1] = (u8)(0 & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;	
				databuf[1] = (u8)((0 & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;	
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH; 
				databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
#endif
		//let up layer to know
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
		  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	}
	tmd2772_clear_intr(obj->client);
	mt_eint_unmask(CUST_EINT_ALS_NUM);      
}*/

static int light_report_value(struct input_dev *input, int data)
{
	unsigned char index = 0;
	if (data <= 0) {
		index = 0;goto report;
	} else if(data <= 1) {
		index = 1;goto report;
	} else if(data <= 10) {
		index = 2;goto report;
	} else if(data <= 50) {
		index = 3;goto report;
	} else if(data <= 150) {
		index = 4;goto report;
	} else if(data <= 400) {
		index = 5;goto report;
	} else if(data <= 1000) {
		index = 6;goto report;
	} else {
		index = 7;goto report;
	}

report:
	input_report_abs(input, ABS_MISC, index);
	input_sync(input);

	return index;
}

static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int index = 0, result = 0;
	u16 d[2], data = 0;
	int int_status = 0;

	/*psensor  trigger return */
	int_status = sensor_read_reg(client, sensor->ops->int_status_reg);
	if (0 == (int_status & 0x10)) {
		APS_DBG("not lsensor int!!!\n");
		return int_status;
	}

	/*clear psensor interrupt*/
	result = tmd2772_check_and_clear_intr(client);
	/*if  (result)
		printk(KERN_ERR "%s clear interrupt error, result = %d\n",
					sensor->ops->name, result);
		return result;
	*/

	tmd2772_read_als(client, d);
	tmd2772_read_als(client, (d+1));
	data = (d[1] > d[0]) ? d[0] : d[1];
	index = light_report_value(sensor->input_dev, data);
	APS_DBG("%s:%s result=0x%x,index=%d\n", __func__, sensor->ops->name, data, index);

	if ((sensor->pdata->irq_enable) &&  (sensor->ops->int_status_reg >= 0))	//read sensor intterupt status register
	{
		result = sensor_read_reg(client, sensor->ops->int_status_reg);
		if (result)
			printk("%s:lsensor init status:=0x%x\n",	__func__, result);
	}
	return result;
}

struct sensor_operate light_tmd2772_ops = {
	.name			= "ls_tmd2772",
	.type			= SENSOR_TYPE_LIGHT,	/*sensor type and it should be correct*/
	.id_i2c			= LIGHT_ID_TMD2772,		/*i2c id number*/
	.read_reg		= SENSOR_UNKNOW_DATA,	/*read data*/
	.read_len		= 2,					/*data length*/
	.id_reg			= TMD2773_ID_REF,		/*read device id from this register*/
	.id_data		= TMD2773_DEVICE_ID,	/*device id*/
	.precision		= 8,					/*8 bits*/
	.ctrl_reg		= SENSOR_UNKNOW_DATA,	/*enable or disable*/
	.int_status_reg	= TMD2772_CMM_STATUS,	/*intterupt status register*/
	.range			= {100, 65535},			/*range*/
	.brightness		= {10, 255},			/* brightness*/
	.trig			= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,
	.active			= tmd2772_enable_als,
	.init			= tmd2772_init_client,
	.report			= sensor_report_value,
};

/*function name should not be changed*/
static struct sensor_operate *light_get_ops(void)
{
	return &light_tmd2772_ops;
}

static int __init light_tmd2772_init(void)
{
	struct sensor_operate *ops = light_get_ops();
	int result = 0;
	int type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, light_get_ops);
	return result;
}

static void __exit light_tmd2772_exit(void)
{
	struct sensor_operate *ops = light_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, light_get_ops);
}

module_init(light_tmd2772_init);
module_exit(light_tmd2772_exit);
