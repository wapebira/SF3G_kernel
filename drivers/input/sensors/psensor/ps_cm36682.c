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


uint16_t ps_conf1_val=0x0006;//it = 2; itb = 0; ?y¡¤?¨º¡À??¡ê??y¡¤?¡À??¨º
uint16_t ps_conf3_val=0x3010;

uint8_t ps_close_thd_set = 0x0f;//0xaf;//close
uint8_t ps_away_thd_set = 0x09;//0xae;//away


/****************operate according to sensor chip:start************/

static int ps_cm36682_parse_dt(struct i2c_client *client)
{	
	struct device_node *np = client->dev.of_node;
	uint32_t temp_val;
	int rc;

	rc = of_property_read_u32(np, "capella,ps_close_thd_set", &temp_val);	
	if (rc) 
	{		
		DBG("Unable to read ps_close_thd_set\n");		
		return rc;	
	} 
	else 
	{		
		ps_close_thd_set = (uint8_t)temp_val;
	}
	rc = of_property_read_u32(np, "capella,ps_away_thd_set", &temp_val);
	if (rc) 
	{
		DBG("Unable to read ps_away_thd_set\n");
		return rc;	
	}
	else 
	{		
		ps_away_thd_set = (uint8_t)temp_val; 
	}
	rc = of_property_read_u32(np, "capella,ps_conf1_val", &temp_val);	
	if (rc) 
	{		
		DBG("Unable to read ps_conf1_val\n");		
		return rc;	
	} 
	else 
	{		
		ps_conf1_val = (uint16_t)temp_val;
	}
	rc = of_property_read_u32(np, "capella,ps_conf3_val", &temp_val);
	if (rc) 
	{
		DBG("Unable to read ps_conf3_val\n");
		return rc;	
	}
	else 
	{		
		ps_conf3_val = (uint16_t)temp_val; 
	}
	
	DBG("%s: ps_close_thd_set = 0x%x\n", __func__, ps_close_thd_set);
	DBG("%s: ps_away_thd_set = 0x%x\n", __func__, ps_away_thd_set);
	DBG("%s: ps_conf1_val = 0x%x\n", __func__, ps_conf1_val);
	DBG("%s: ps_conf3_val = 0x%x\n", __func__, ps_conf3_val);
	
	return 0;
}

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	uint8_t value = 0;

	if(enable)
	{	
		sensor->ops->ctrl_data = ps_conf1_val;	
		sensor->ops->ctrl_data &= CM36283_PS_SD_MASK;
		sensor->ops->ctrl_data |= CM36283_PS_INT_IN_AND_OUT; 

	    sensor_write_word_reg(client, PS_THD,
				((ps_close_thd_set) << 8) | ps_away_thd_set);
		sensor_read_word_reg(client, INT_FLAG);//clear interrupt flag
	}
	else
	{
		sensor->ops->ctrl_data |= CM36283_PS_SD;
		sensor->ops->ctrl_data &= CM36283_PS_INT_MASK;
	}

	//DBG("%s:reg=0x%x,reg_ctrl=0x%x,enable=%d\n",__func__,sensor->ops->ctrl_reg, sensor->ops->ctrl_data, enable);

	result = sensor_write_word_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if(enable) {
		input_report_abs(sensor->input_dev, ABS_DISTANCE, 1);
		input_sync(sensor->input_dev);
	}
	if(result)
		printk("%s:fail to active sensor\n",__func__);
	//msleep(100);  
	return result;

}


static int sensor_init(struct i2c_client *client)
{	
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;

	/*ps_cm36682 i2c addr in dts is dummy, correct here.*/
	client->addr = 0x60;
	ps_cm36682_parse_dt(client);
	
	sensor->status_cur = SENSOR_OFF;
	/*must disable l-sensor interrupt befrore IST create*//*disable ALS func*/
	ps_conf1_val |= CM36283_PS_SD;
	ps_conf1_val &= CM36283_PS_INT_MASK;
	result =sensor_write_word_reg(client, PS_CONF1, ps_conf1_val);
	sensor_write_word_reg(client, PS_CONF3, ps_conf3_val);
	sensor_write_word_reg(client, PS_THD,
			(ps_close_thd_set << 8) | ps_away_thd_set);
	
	DBG("ps_cm36682 sensor_init: result=%d \n",result);
	//msleep(100);
	return result;
}

static int get_stable_ps_adc_value(struct sensor_private_data *sensor)
{	
	uint32_t value[3] = {0, 0, 0};
	uint32_t mid_value = 0;
	uint8_t i = 0;

	while(i++ < 3)
	{
		value[i] = sensor_read_word_reg(sensor->client, sensor->ops->read_reg);
		mid_value += value[i];
	}

	return (mid_value / 3);
}

static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	uint8_t value = 0;	
	uint16_t intflag = 0;
	struct sensor_private_data *lsensor;

	// polling
	if(!sensor->pdata->irq_enable)
	{
		// psensor
		value = get_stable_ps_adc_value(sensor);	
		input_report_abs(sensor->input_dev, ABS_DISTANCE, (value>= ps_close_thd_set)?0:1);
		input_sync(sensor->input_dev);		
			
		// light sensor
	/*	lsensor = g_sensor[SENSOR_TYPE_LIGHT];
		if((lsensor) && (lsensor->i2c_id) && (LIGHT_ID_CM36682 == (int)(lsensor->i2c_id->driver_data)))
		{
			lsensor->ops->report(lsensor->client);
		}*/
		return 0;
	}

	// irq
	if(sensor->pdata->irq_enable)
	{
		if(sensor->ops->int_status_reg)
		{
			intflag = sensor_read_word_reg(client, sensor->ops->int_status_reg);
		}
	}

	//psensor irq
	if((intflag & INT_FLAG_PS_IF_AWAY) || (intflag & INT_FLAG_PS_IF_CLOSE))
	{
		// 1. close ps interrupt
		//sensor->ops->ctrl_data &= CM36283_PS_INT_MASK;
		//result =sensor_write_word_reg(client, PS_CONF1, sensor->ops->ctrl_data);

		// 2. read ps value
		value = get_stable_ps_adc_value(sensor);	
		input_report_abs(sensor->input_dev, ABS_DISTANCE, (value>= ps_close_thd_set)?0:1);
		input_sync(sensor->input_dev);

		DBG("intflag = 0x%x, value = 0x%x\n", intflag, value);
		// 3. open ps interrupt
		//sensor->ops->ctrl_data |= CM36283_PS_INT_IN_AND_OUT; 
		//result =sensor_write_word_reg(client, PS_CONF1, sensor->ops->ctrl_data);

	}
	//light sensor irq
/*	if((intflag & INT_FLAG_ALS_IF_H) || (intflag & INT_FLAG_ALS_IF_L))
	{
		// find the light sensor, 1 chip 2 parallel function.
		lsensor = g_sensor[SENSOR_TYPE_LIGHT];
		if((lsensor) && (lsensor->i2c_id) && (LIGHT_ID_CM36682 == (int)(lsensor->i2c_id->driver_data)))
		{
			lsensor->ops->report(lsensor->client);
		}
	}*/
	return result;
}


struct sensor_operate proximity_cm36682_ops = {
	.name				= "ps_cm36682",
	.type				= SENSOR_TYPE_PROXIMITY,	//sensor type and it should be correct
	.id_i2c				= PROXIMITY_ID_CM36682,	//i2c id number
	.read_reg			= PS_DATA,	//read data
	.read_len			= 2,			//data length
	.id_reg				= ID_REG,	//read device id from this register
	.id_data 			= 0x83,	//device id
	.precision			= 8,			//8 bits
	.ctrl_reg 			= PS_CONF1,	//enable or disable 
	.int_status_reg 		= INT_FLAG,	//intterupt status register
	.range				= {0,1},			//range
	.trig				= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED | IRQF_NO_SUSPEND,
	.active				= sensor_active,	
	.init				= sensor_init,
	.report				= sensor_report_value,
};

/****************operate according to sensor chip:end************/

//function name should not be changed
static struct sensor_operate *proximity_get_ops(void)
{
	return &proximity_cm36682_ops;
}


static int __init proximity_cm36682_init(void)
{
	struct sensor_operate *ops = proximity_get_ops();
	int result = 0;
	int type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, proximity_get_ops);
	return result;
}

static void __exit proximity_cm36682_exit(void)
{
	struct sensor_operate *ops = proximity_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, proximity_get_ops);
}


module_init(proximity_cm36682_init);
module_exit(proximity_cm36682_exit);


