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
#include <linux/proc_fs.h>


#include "ps_tmd2772.h"

#define APS_TAG                  "[ALS/PS] "
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#if 0
#define APS_DBG(fmt, args...)    printk(APS_TAG fmt, ##args)              
#else
#define APS_DBG(fmt, args...)
#endif   

#define TMD2772_CMM_PPCOUNT_VALUE 0x0B
#define  ZOOM_TIME 4
#define  TMD2772_CMM_CONTROL_VALUE 0x24


extern int tp_ps_enable(struct i2c_client *client);//tmd2772_enable_ps,	
extern int tp_ps_init(struct i2c_client *client);
extern int tp_ps_report(struct i2c_client *client);
 struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static int intr_flag_value = 0;
static int ps_deb_on = 0;
static int ps_deb_end = 0;
static u16 ps_thd_val_high = 0xFFFF;
static u16 ps_thd_val_low = 0x0;

typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;
int tmd2772_read_ps(struct i2c_client *client, u16 *data);

static int ps_tmd2772_parse_dt(struct i2c_client *client)
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

static int tmd2772_enable_ps(struct i2c_client *client, int enable, int rate)
{
	long res = 0;
	int read_value = 0;
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
			status = 0x2D;
		else
			status = 0x0D;
		read_value |= status;
		ps_deb_on = 1;
		ps_deb_end = jiffies + 10 /(1000 / HZ);
		APS_DBG("psensor tmd2772 power on\n");
	} else {
		if (read_value & 0x02)
			status = ~0x24;
		else
			status = ~0x2D;
		read_value &= status;
		APS_DBG("psensor tmd2772 power off\n");
	}

	res = sensor_write_reg(client, TMD2772_CMM_ENABLE, read_value);
	if (res) {
		APS_ERR("tmd2772_enable_ps fail\n");
		return res;
	}
	if(enable) {
		input_report_abs(sensor->input_dev, ABS_DISTANCE, 1);
		input_sync(sensor->input_dev);
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
	if (value & 0x20) {
		reg = (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_SPL_FN | 0x05);
		res = i2c_master_send(client, &reg, 0x1);
		if(res <= 0)
			goto EXIT_ERR;
	}
	return 0;

EXIT_ERR:
	APS_ERR("tmd2772_check_and_clear_intr fail\n");
	return 1;
}

struct i2c_client *g_client;

static int tmd2772_init_client(struct i2c_client *client)
{
	int res = 0;
	g_client = client;

	/*client->addr = sensor->ops->slave_addr;*/

	/*res = sensor_write_reg(client, TMD2772_CMM_ENABLE, 0x00);
	if(res <= 0)
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

	res = sensor_write_reg(client, TMD2772_CMM_PERSISTENCE, 0x20);
	if (res)
		goto EXIT_ERR;

	res = sensor_write_reg(client, TMD2772_CMM_CONFIG, 0x04);
	if (res)
		goto EXIT_ERR;

	res = sensor_write_reg(client, TMD2772_CMM_PPCOUNT, TMD2772_CMM_PPCOUNT_VALUE);
	if (res)
		goto EXIT_ERR;

	res = sensor_write_reg(client, TMD2772_CMM_CONTROL, TMD2772_CMM_CONTROL_VALUE);
	if (res)
		goto EXIT_ERR;

	ps_tmd2772_parse_dt(client);

	/*initialization  Proximity Interrupt Threshold Registers*/
	res = sensor_write_reg(client, TMD2772_CMM_INT_LOW_THD_LOW, (u8)(ps_thd_val_low & 0x00FF));
	if (res)
		goto EXIT_ERR;
	res = sensor_write_reg(client, TMD2772_CMM_INT_LOW_THD_HIGH, (u8)(ps_thd_val_low & 0xFF00) >> 8);
	if (res)
		goto EXIT_ERR;
	res = sensor_write_reg(client, TMD2772_CMM_INT_HIGH_THD_LOW, (u8)(ps_thd_val_high & 0x00FF));
	if (res)
		goto EXIT_ERR;
	res = sensor_write_reg(client, TMD2772_CMM_INT_HIGH_THD_HIGH, (u8)((ps_thd_val_high & 0xFF00) >> 8));
	if (res)
		goto EXIT_ERR;

	tmd2772_enable_ps(client, 0, 0);

	return TMD2772_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

int tmd2772_read_ps(struct i2c_client *client, u16 *data)
{
	int ps_value_low, ps_value_high;
	int res = 0;

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	ps_value_low = sensor_read_reg(client, TMD2772_CMM_PDATA_L);
	if (ps_value_low < 0)
		goto EXIT_ERR;

	ps_value_high = sensor_read_reg(client, TMD2772_CMM_PDATA_H);
	if (ps_value_high < 0)
		goto EXIT_ERR;

	*data = ps_value_low | (ps_value_high << 8);
	APS_DBG("ps_data=%d, low:%d  high:%d \n", *data, ps_value_low, ps_value_high);
	return 0;    

EXIT_ERR:
	APS_ERR("tmd2772_read_ps fail\n");
	return res;
}

static int tmd2772_get_ps_value(u16 ps)
{
	int val;
	int invalid = 0;
	static int val_temp=1;
	int type = -1;

	APS_DBG("PS raw data:  %05d =>\n", ps);
	APS_DBG("ps_thd_val_high = %d ,ps_thd_val_low =%d\n", ps_thd_val_high, ps_thd_val_low);
	if (ps > ps_thd_val_high) {
		val = 0;
		val_temp = 0;
		intr_flag_value = 1;
		type = 3;
	} else if (ps < ps_thd_val_low) {
		val = 1;
		val_temp = 1;
		intr_flag_value = 0;
		type = 4;
	} else
	    val = val_temp;

	if (1 == ps_deb_on) {
		unsigned long endt = ps_deb_end;

		if (time_after(jiffies, endt))
			ps_deb_on = 0;
		
		if  (1 == ps_deb_on) {
			invalid = 1;
			type = 6;
		}
	}

	if (!invalid) {
		APS_DBG("PS:  %05d => %05d,   type = %d \n", ps, val, type);
		return val;
	} else {
		APS_DBG("wrong PS:  %05d => %05d,   type = %d \n", ps, val, type);
		return -1;
	}
}

static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int value = 0, res = 0;
	u16 index = 0;

	/*lsensor  trigger return */
	if ((sensor->pdata->irq_enable) &&	(sensor->ops->int_status_reg >= 0))
	{
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		if (0 == (value & 0x20)) {
			APS_DBG("not psensor int value =0x%0x!!!\n", value);
			return res;
		}
	}

	tmd2772_read_ps(client, &index);
	value = tmd2772_get_ps_value(index);
	input_report_abs(sensor->input_dev, ABS_DISTANCE, value);
	input_sync(sensor->input_dev);
	APS_DBG("%s:%s result=0x%x\n", __func__, sensor->ops->name, value);

	if(sensor->pdata->irq_enable)
	{
		if (intr_flag_value) {
			res = sensor_write_reg(client, TMD2772_CMM_INT_LOW_THD_LOW, (u8)(ps_thd_val_low & 0x00FF));
			if (res)
				return res;
			res = sensor_write_reg(client, TMD2772_CMM_INT_LOW_THD_HIGH, (u8)((ps_thd_val_low) & 0xFF00) >> 8);
			if (res)
				return res;
			res = sensor_write_reg(client, TMD2772_CMM_INT_HIGH_THD_LOW, (u8)(0x00FF));
			if (res)
				return res;
			res = sensor_write_reg(client, TMD2772_CMM_INT_HIGH_THD_HIGH, (u8)((0xFF00) >> 8));
			if (res)
				return res;
		} else {
			res = sensor_write_reg(client, TMD2772_CMM_INT_LOW_THD_LOW, (u8)(0 & 0x00FF));
			if (res)
				return res;
			res = sensor_write_reg(client, TMD2772_CMM_INT_LOW_THD_HIGH, (u8)((0 & 0xFF00) >> 8));
			if (res)
				return res;
			res = sensor_write_reg(client, TMD2772_CMM_INT_HIGH_THD_LOW, (u8)(ps_thd_val_high & 0x00FF));
			if (res)
				return res;
			res = sensor_write_reg(client, TMD2772_CMM_INT_HIGH_THD_HIGH, (u8)((ps_thd_val_high & 0xFF00) >> 8));
			if (res)
				return res;
		}
	}
	/*clear psensor interrupt*/
	res = tmd2772_check_and_clear_intr(client);
	if (res != 0) {
		printk("%s clear interrupt error res = %d\n", sensor->ops->name, res);
		return res;
	}
	return res;
}

struct sensor_operate proximity_tmd2772_ops = {
	.name				= "ps_tmd2772",
	.type				= SENSOR_TYPE_PROXIMITY,	//sensor type and it should be correct
	.id_i2c				= PROXIMITY_ID_TMD2772,		//i2c id number
	.read_reg			= SENSOR_UNKNOW_DATA,			//read data
	.read_len			= 2,				//data length
	.id_reg				= TMD2773_ID_REF,		/*read device id from this register*/
	.id_data			= TMD2773_DEVICE_ID,	/*device id*/
	.precision			= 8,				//8 bits
	.ctrl_reg 			= SENSOR_UNKNOW_DATA,			//enable or disable 
	.int_status_reg		= TMD2772_CMM_STATUS,	/*intterupt status register*/
	.range				= {0,1},			//range
	.trig				= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED | IRQF_NO_SUSPEND,
	.active				= tp_ps_enable,//tmd2772_enable_ps,	
	.init				= tp_ps_init,
	.report				= tp_ps_report,
};

//function name should not be changed
static struct sensor_operate *proximity_get_ops(void)
{
	return &proximity_tmd2772_ops;
}

static ssize_t xxx_proc_write(struct file *file, const char __user *buffer,
			   size_t count, loff_t *data)
{
	char c;
	int rc;
	int enable = 0;
	rc = get_user(c, buffer);
	enable = c - '0';
	printk("enable = %d",enable);
	tmd2772_enable_ps(g_client,enable,0);
}


static const struct file_operations xxx_proc_fops = {
	.owner		= THIS_MODULE, 
	.write		= xxx_proc_write,
};

static int __init proximity_tmd2772_init(void)
{
	struct sensor_operate *ops = proximity_get_ops();
	int result = 0;
	int type = ops->type;
	struct proc_dir_entry *xxx_proc_entry;

	result = sensor_register_slave(type, NULL, NULL, proximity_get_ops);

	xxx_proc_entry = proc_create("driver/xxx_dbg", 0660, NULL, &xxx_proc_fops); 
	return result;
}

static void __exit proximity_tmd2772_exit(void)
{
	struct sensor_operate *ops = proximity_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, proximity_get_ops);
}

module_init(proximity_tmd2772_init);
module_exit(proximity_tmd2772_exit);
