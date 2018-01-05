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



#define CM36283_I2C_NAME "cm36283"

/* Define Slave Address*/
#define	CM36283_slave_add	0xC0>>1

#define ALS_CALIBRATED		0x6E9F
#define PS_CALIBRATED		  0x509F

/*Define Command Code*/
#define		ALS_CONF		  0x00
#define		ALS_THDH  	  0x01
#define		ALS_THDL	    0x02
#define		PS_CONF1      0x03
#define		PS_CONF3      0x04
#define		PS_CANC       0x05
#define		PS_THD        0x06
#define		RESERVED      0x07

#define		PS_DATA       0x08
#define		ALS_DATA      0x09
#define		RESERVED2     0x0A
#define		INT_FLAG      0x0B
#define		ID_REG        0x0C

/*cm36283*/
/*for ALS CONF command*/
#define CM36283_ALS_IT_80ms 	(0 << 6)
#define CM36283_ALS_IT_160ms 	(1 << 6)
#define CM36283_ALS_IT_320ms 	(2 << 6)
#define CM36283_ALS_IT_640ms 	(3 << 6)
#define CM36283_ALS_GAIN_1 		(0 << 2)
#define CM36283_ALS_GAIN_2 		(1 << 2)
#define CM36283_ALS_GAIN_4 		(2 << 2)
#define CM36283_ALS_GAIN_8 		(3 << 2)
#define CM36283_ALS_INT_EN	 	(1 << 1) /*enable/disable Interrupt*/
#define CM36283_ALS_INT_MASK	0xFFFD
#define CM36283_ALS_SD			  (1 << 0) /*enable/disable ALS func, 1:disable , 0: enable*/
#define CM36283_ALS_SD_MASK		0xFFFE

/*for PS CONF1 command*/
#define CM36283_PS_ITB_1_2	 (0 << 14)
#define CM36283_PS_ITB_1     (1 << 14)
#define CM36283_PS_ITB_2     (2 << 14)
#define CM36283_PS_ITB_4     (3 << 14)
#define CM36283_PS_INT_OFF	       (0 << 8) /*enable/disable Interrupt*/
#define CM36283_PS_INT_IN          (1 << 8)
#define CM36283_PS_INT_OUT         (2 << 8)
#define CM36283_PS_INT_IN_AND_OUT  (3 << 8)

#define CM36283_PS_INT_MASK   0xFCFF

#define CM36283_PS_DR_1_40   (0 << 6)
#define CM36283_PS_DR_1_80   (1 << 6)
#define CM36283_PS_DR_1_160  (2 << 6)
#define CM36283_PS_DR_1_320  (3 << 6)
#define CM36283_PS_IT_1T 	   (0 << 4)
#define CM36283_PS_IT_1_3T   (1 << 4)
#define CM36283_PS_IT_1_6T 	 (2 << 4)
#define CM36283_PS_IT_2T 		 (3 << 4)
#define CM36283_PS_PERS_1 	 (0 << 2)
#define CM36283_PS_PERS_2 	 (1 << 2)
#define CM36283_PS_PERS_3 	 (2 << 2)
#define CM36283_PS_PERS_4 	 (3 << 2)
#define CM36283_PS_RES_1     (1 << 1)
#define CM36283_PS_SD	       (1 << 0)/*enable/disable PS func, 1:disable , 0: enable*/
#define CM36283_PS_SD_MASK	 0xFFFE

/*for PS CONF3 command*/
#define CM36283_PS_MS_NORMAL        (0 << 14)
#define CM36283_PS_MS_LOGIC_ENABLE  (1 << 14)
#define CM36283_PS_PROL_63 	     (0 << 12)
#define CM36283_PS_PROL_127      (1 << 12)
#define CM36283_PS_PROL_191 	   (2 << 12)
#define CM36283_PS_PROL_255 		 (3 << 12)
#define CM36283_PS_SMART_PERS_ENABLE  (1 << 4)
#define CM36283_PS_ACTIVE_FORCE_MODE  (1 << 3)
#define CM36283_PS_ACTIVE_FORCE_TRIG  (1 << 2)

/*for INT FLAG*/
#define INT_FLAG_PS_SPFLAG           (1<<14)
#define INT_FLAG_ALS_IF_L            (1<<13)
#define INT_FLAG_ALS_IF_H            (1<<12)
#define INT_FLAG_PS_IF_CLOSE         (1<<9)
#define INT_FLAG_PS_IF_AWAY          (1<<8)  

#define LS_PWR_ON		BIT(0)
#define PS_PWR_ON		BIT(1)

uint16_t ls_cmd=0x04;

/****************operate according to sensor chip:start************/

uint16_t g_als_levels[9] = {0,10, 160, 225, 320, 640, 1280, 2600,0xffff};
int get_als_level(uint16_t data)
{
	unsigned char index = 0;
	
	if(data <= 10){
		index = 0;
	}
	else if(data <= 160){
		index = 1;
	}
	else if(data <= 225){
		index = 2;
	}
	else if(data <= 320){
		index = 3;
	}
	else if(data <= 640){
		index = 4;
	}
	else if(data <= 1280){
		index = 5;
	}
	else if(data <= 2600){
		index = 6;
	}
	else{
		index = 7;
	}

	return index;
}
static int ls_cm36682_parse_dt(struct i2c_client *client)
{	
	struct device_node *np = client->dev.of_node;
	uint32_t temp_val;
	int rc;

	rc = of_property_read_u32(np, "capella,ls_cmd", &temp_val);	
	if (rc) 
	{		
		DBG("Unable to read ls_cmd\n");		
		return rc;	
	} 
	else 
	{		
		ls_cmd = (uint16_t)temp_val;
	}

	DBG("%s: ls_cmd = 0x%x\n", __func__, ls_cmd);
	
	return 0;
}

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;	
	
	if (enable) 
	{
		sensor->ops->ctrl_data = ls_cmd;	
		sensor->ops->ctrl_data &= CM36283_ALS_SD_MASK;
		sensor->ops->ctrl_data |= CM36283_ALS_INT_EN;

		sensor_write_word_reg(client, ALS_THDH,640);
		sensor_write_word_reg(client, ALS_THDL,320);	
		sensor_read_word_reg(client, INT_FLAG);//clear interrupt flag
	}
	else
	{
		sensor->ops->ctrl_data |= CM36283_ALS_SD;
		sensor->ops->ctrl_data &= CM36283_ALS_INT_MASK;
	}

	DBG("%s:reg=0x%x,reg_ctrl=0x%x,enable=%d\n",__func__,sensor->ops->ctrl_reg, sensor->ops->ctrl_data, enable);

	result = sensor_write_word_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if(result)
		printk("%s:fail to active sensor\n",__func__);
	
	//msleep(10);  
		
	return result;
}


static int sensor_init(struct i2c_client *client)
{	
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;

	ls_cm36682_parse_dt(client);
	
	sensor->status_cur = SENSOR_OFF;
	/*must disable l-sensor interrupt befrore IST create*//*disable ALS func*/
	ls_cmd &= CM36283_ALS_INT_MASK;
	ls_cmd |= CM36283_ALS_SD;
	result =sensor_write_word_reg(client, ALS_CONF, ls_cmd);
	DBG("sensor_init write ALS_CONF client->addr=0x%x,result= %d\n",client->addr,result);
	//msleep(100);  
	
	return result;
}


static int light_report_value(struct input_dev *input, int data)
{
	unsigned char index = 0;
	
	index = get_als_level(data);

	input_report_abs(input, ABS_MISC, index);
	input_sync(input);
	
	return index;
}


static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	unsigned short value = 0;
	int index = 0;
	//unsigned short conf,thdh,thdl;
	uint8_t als_level;

	// 1. close als interrupt
	/*sensor->ops->ctrl_data &= CM36283_ALS_INT_MASK;
	result =sensor_write_word_reg(client, ALS_CONF, sensor->ops->ctrl_data);*/

	// 2. read als data
	value=sensor_read_word_reg(client, ALS_DATA);

	// 3. send  input to system
	index = light_report_value(sensor->input_dev, value);	
	input_sync(sensor->input_dev);

	// 4. calc new thd range
	als_level = get_als_level(value);
	sensor_write_word_reg(client, ALS_THDH, g_als_levels[als_level + 1]);
	sensor_write_word_reg(client, ALS_THDL, g_als_levels[als_level]);
		
	//conf = sensor_read_word_reg(client, ALS_CONF);
	//thdh = sensor_read_word_reg(client, ALS_THDH);
	//thdl = sensor_read_word_reg(client, ALS_THDL);
	//uint16_t psvalue=sensor_read_word_reg(client, PS_DATA);
	
	//DBG("conf = 0x%x,value = %d, psvalue = 0x%x, thdh = %d, thdl = %d\n", conf, value, psvalue, thdh, thdl);

	// 5. open als interrupt
	/*sensor->ops->ctrl_data |= CM36283_ALS_INT_EN;
	result =sensor_write_word_reg(client, ALS_CONF, sensor->ops->ctrl_data);	*/
	return result;
}


struct sensor_operate light_cm36682_ops = {
	.name				= "ls_cm36682",
	.type				= SENSOR_TYPE_LIGHT,	//sensor type and it should be correct
	.id_i2c				= LIGHT_ID_CM36682,	//i2c id number
	.read_reg			= ALS_DATA,	//read data
	.read_len			= 2,			//data length
	.id_reg				= ID_REG,	//read device id from this register
	.id_data 			= 0x83,	//device id
	.precision			= 8,			//8 bits
	.ctrl_reg 			= ALS_CONF,	//enable or disable 
	.int_status_reg 		= SENSOR_UNKNOW_DATA,	//intterupt status register
	.range				= {100,65535},		//range
	.brightness         		= {10,255},             // brightness
	.trig				= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,		
	.active				= sensor_active,	
	.init				= sensor_init,
	.report				= sensor_report_value,
};

/****************operate according to sensor chip:end************/

//function name should not be changed
static struct sensor_operate *light_get_ops(void)
{
	return &light_cm36682_ops;
}


static int __init light_cm36682_init(void)
{
	struct sensor_operate *ops = light_get_ops();
	int result = 0;
	int type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, light_get_ops);
	return result;
}

static void __exit light_cm36682_exit(void)
{
	struct sensor_operate *ops = light_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, light_get_ops);
}


module_init(light_cm36682_init);
module_exit(light_cm36682_exit);


