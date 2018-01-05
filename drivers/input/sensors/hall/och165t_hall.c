/* drivers/input/sensors/hall/och165t_hall.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
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
#include <linux/platform_device.h>
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


#define CM3232_CLOSE	0x01


#define CM3232_ADDR_COM 0
#define CM3232_ADDR_DATA 50	

#define CM3232_DRV_NAME "cm3232"
//command code
#define COMMAND_CTRL 		0
#define COMMAND_ALS_DATA 	50 		//ALS: 15:8 MSB 8bits data
						//7:0 LSB 8bits data

struct och165t_data {
	struct work_struct work;
	int irq_pin;
	int irq; 
};

static struct och165t_data *och165t;

extern  void xgold_send_power_key(int state);
extern  void xgold_send_wakeup_key(void); 
/****************operate according to sensor chip:start************/

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	if(enable)
	{
		sensor->status_cur = SENSOR_ON;
	}
	else
	{
		sensor->status_cur = SENSOR_OFF;
	}
	return 0;
}

static int sensor_init(struct i2c_client *client)
{	
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	
	result = sensor->ops->active(client,0,0);
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
	
	sensor->status_cur = SENSOR_OFF;
	return result;
}


static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	struct sensor_platform_data *pdata = sensor->pdata;
	int gpio_value = 0;
	gpio_value = gpio_get_value(pdata->irq_pin);
	if(gpio_value == 0)
	{		
		//send power key to sleep
		xgold_send_power_key(1);
		xgold_send_power_key(0);
	}
	else
	{
		//xgold_send_power_key(1);
		//xgold_send_power_key(0);
		xgold_send_wakeup_key(); // wake up the system
	}
	return 0;
}


static struct sensor_operate hall_och165t_ops = {
	.name				= "och165t",
	.type				= SENSOR_TYPE_HALL,	//sensor type and it should be correct
	.id_i2c				= HALL_ID_OCH165T,	//i2c id number
	.read_reg			= SENSOR_UNKNOW_DATA,	//read data
	.read_len			= 2,			//data length
	.id_reg				= SENSOR_UNKNOW_DATA,	//read device id from this register
	.id_data 			= SENSOR_UNKNOW_DATA,	//device id
	.precision			= 8,			//8 bits
	.ctrl_reg 			= SENSOR_UNKNOW_DATA,	//enable or disable 
	.int_status_reg 		= SENSOR_UNKNOW_DATA,	//intterupt status register
	.range				= {100,65535},		//range
	.brightness         		= {10,255},             // brightness
	.trig				= SENSOR_UNKNOW_DATA,		
	.active				= sensor_active,	
	.init				= sensor_init,
	.report				= sensor_report_value,
};

/****************operate according to sensor chip:end************/

//function name should not be changed
static struct sensor_operate *hall_get_ops(void)
{
	return &hall_och165t_ops;
}

static void och165t_work_func(struct work_struct *work)
{
	int gpio_value = 0;
	pr_info("%s %d\n",__func__,__LINE__);
	gpio_value = gpio_get_value(och165t->irq_pin);
	if(gpio_value == 0)
	{		
		//send power key to sleep
		xgold_send_power_key(1);
		xgold_send_power_key(0);
	}
	else
	{
		//xgold_send_power_key(1);
		//xgold_send_power_key(0);
		xgold_send_wakeup_key(); // wake up the system
	}

	//enable_irq_wake(och165t->irq);
	return ;
}

static irqreturn_t hall_och165t_interrupt(int irq, void *dev_id)
{
	struct och165t_data *data = dev_id;
	pr_debug("%s %d\n",__func__,__LINE__);
	//disable_irq_wake(irq);
	schedule_work(&data->work);
	return IRQ_HANDLED;
}

static int och165t_hall_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	unsigned long irq_flags;
	int err;
	
	pr_info("och165t_hall_probe enter\n");
	
	/* Allocate memory for driver data */
	och165t = kzalloc(sizeof(struct och165t_data), GFP_KERNEL);
	if (!och165t) {
		printk(KERN_ERR "och165t och165t_hall_probe: memory allocation failed.\n");
		err = -ENOMEM;
		goto exit;
	}
	
	INIT_WORK(&och165t->work, och165t_work_func);

	och165t->irq_pin = of_get_named_gpio_flags(np, "irq-gpio", 0,(enum of_gpio_flags *)&irq_flags);
	if (!och165t->irq_pin) {
		pr_debug("no IRQ?\n");
		goto exit;
	}else {
		och165t->irq = gpio_to_irq(och165t->irq_pin);
		pr_info("pin %d, irq %d\n", och165t->irq_pin, och165t->irq);
	}
	#if 0
	err = gpio_request(och165t->irq_pin, "hall_och165t"); 
	if (err < 0) { 
		dev_err(&pdev->dev, "failed to request GPIO, error %d\n", err); 
		goto exit; 
	}

	err = gpio_direction_input(och165t->irq_pin);
	if (err) {
		dev_err(&pdev->dev, "failed to set GPIO direction, error %d\n", err); 
		goto exit; 
	}
	#endif 
	
	/* IRQ */
	err = request_irq(och165t->irq, hall_och165t_interrupt, IRQ_TYPE_EDGE_BOTH, "hall_och165t", och165t);
	if (err < 0) {
		printk(KERN_ERR "AKM8975 akm8975_probe: request irq failed\n");
		goto exit;
	}
	//enable_irq_wake(och165t->irq);
	pr_info("och165t_hall_probe OK\n");
	return  0;
exit:
	pr_err("och165t_hall_probe failed\n");
	return -1;
	
}
static const struct of_device_id och165t_hall_match[] = {
	{ .compatible = "hall_och165t" }, 
	{ /* Sentinel */ } 
}; 

static struct platform_driver och165t_hall_driver = {
	.probe		= och165t_hall_probe,
	.driver		= {
		.name	= "och165t",
		.owner	= THIS_MODULE,
		.of_match_table	= och165t_hall_match,
	},
};
static int hall_och165t_init(void)
{
	int result = 0;
	result = platform_driver_register(&och165t_hall_driver);
	return result;
}

static void hall_och165t_exit(void)
{
	platform_driver_unregister(&och165t_hall_driver);
}

module_init(hall_och165t_init);
module_exit(hall_och165t_exit);


