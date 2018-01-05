/*
 * Copyright (C) 2012-2014 FuZhou ROCKCHIP, Inc.
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

#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/wakelock.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <sofia/vmm_pmic.h>

#define VMM_SHARED_I2C_SLAVE_ADDR_7_BIT_MASK	0x7f
#define VMM_SHARED_I2C_SLAVE_ADDR_10_BIT_MASK	0x3ff

enum {
	I2C_CON_MOD_TX = 0,
	I2C_CON_MOD_RX
};

struct vmm_shared_i2c {
	struct i2c_adapter	adap;
	struct device		*dev;
	unsigned int		mode;
	unsigned int            suspended:1;
};

static int vmm_shared_i2c_set_master(struct vmm_shared_i2c *i2c, struct i2c_msg *msgs, int num)
{
	if (num == 1) {
		if (!(msgs[0].flags & I2C_M_RD)) {
			i2c->mode = I2C_CON_MOD_TX;
		} else {
			dev_err(i2c->dev, "This case(num = 1,read action) has not been supported.\n");
			return -EIO;
		}
	} else if (num == 2) {
		if(msgs[0].addr != msgs[1].addr)
			return -EIO;
		if (!(msgs[0].flags & I2C_M_RD) && (msgs[1].flags & I2C_M_RD)) {
			i2c->mode = I2C_CON_MOD_RX;
		} else
			return -EIO;
	} else {
		dev_err(i2c->dev, "This case(num > 2) has not been supported now\n");
		return -EIO;
	}

	return 0;
}

static inline int vmm_shared_i2c_invalid_address(const struct i2c_msg *msg)
{
	return (msg[0].addr > VMM_SHARED_I2C_SLAVE_ADDR_7_BIT_MASK);
}

static int vmm_shared_i2c_xfer(struct i2c_adapter *adap,
			struct i2c_msg *msgs, int num)
{
	struct vmm_shared_i2c *i2c = i2c_get_adapdata(adap);
	uint32_t vmm_reg_address = 0;
	int ret = 0;

	if (i2c->suspended) {
		dev_err(&adap->dev, "vmm shared i2c bus is not initialized.\n");
		return -EIO;
	}

	if (vmm_shared_i2c_invalid_address(msgs)) {
		dev_err(i2c->dev, "Invalid address.\n");
		ret = -EINVAL;
		return ret;
	}

	if (vmm_shared_i2c_set_master(i2c, msgs, num))
		return ret;

	vmm_reg_address = msgs[0].addr << 24 | msgs[0].buf[0];
	if (i2c->mode ==  I2C_CON_MOD_TX) {
		if(msgs[0].len < 2)
			return -EINVAL;
		ret = vmm_pmic_reg_write_by_range(vmm_reg_address, &(msgs[0].buf[1]), msgs[0].len - 1);
	} else if (i2c->mode == I2C_CON_MOD_RX){
		ret = vmm_pmic_reg_read_by_range(vmm_reg_address, msgs[1].buf, msgs[1].len);
	} else
		ret = -EINVAL;

	if (!ret)
		ret = num;
	else
		dev_err(i2c->dev, "vmm shared i2c transfer for reg 0x%x failed\n", msgs[0].buf[0]);

	return ret;
}

/* declare our i2c functionality */
static u32 vmm_shared_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_PROTOCOL_MANGLING;
}

/* i2c bus registration info */
static const struct i2c_algorithm vmm_shared_i2c_algorithm = {
	.master_xfer		= vmm_shared_i2c_xfer,
	.functionality		= vmm_shared_i2c_func,
};

/* vmm_shared_i2c_probe
 *
 * called by the bus driver when a suitable device is found
*/
static int vmm_shared_i2c_probe(struct platform_device *pdev)
{
	struct vmm_shared_i2c *i2c = NULL;
	struct device_node *np = pdev->dev.of_node;
	int ret;
	char id;

	if (!np) {
		dev_err(&pdev->dev, "Missing device tree node.\n");
		return -EINVAL;
	}

	i2c = devm_kzalloc(&pdev->dev, sizeof(struct vmm_shared_i2c), GFP_KERNEL);
	if (!i2c) {
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	id = *(np->name + strlen(np->name) - 1) - '0';
	WARN_ON(id < 0 || id > 9);

	/* Initialize algo data */	
	strcpy(i2c->adap.name, dev_name(&pdev->dev));
	i2c->dev = &pdev->dev;
	i2c->adap.owner = THIS_MODULE;
	i2c->adap.algo = &vmm_shared_i2c_algorithm;
	i2c->adap.class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	i2c->adap.retries = 2;
	i2c->adap.dev.of_node = np;
	i2c->adap.dev.parent = &pdev->dev;
	i2c->adap.nr = id;
	i2c_set_adapdata(&i2c->adap, i2c);

	/* setup info block for the i2c core */
	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add adapter\n");
		return ret;
	}
	platform_set_drvdata(pdev, i2c);
	return ret;
}

/* vmm_shared_i2c_remove
 *
 * called when device is removed from the bus
*/
static int vmm_shared_i2c_remove(struct platform_device *pdev)
{
	struct vmm_shared_i2c *i2c = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c->adap);
	return 0;
}

#ifdef CONFIG_PM
static int vmm_shared_i2c_suspend_noirq(struct device *dev)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct vmm_shared_i2c *i2c = platform_get_drvdata(pdev);

        i2c->suspended = 1;

        return 0;
}

static int vmm_shared_i2c_resume_noirq(struct device *dev)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct vmm_shared_i2c *i2c = platform_get_drvdata(pdev);

        i2c->suspended = 0;

        return 0;
}

static const struct dev_pm_ops vmm_shared_i2c_dev_pm_ops = {
        .suspend_noirq = vmm_shared_i2c_suspend_noirq,
        .resume_noirq = vmm_shared_i2c_resume_noirq,
};

#define VMM_SHARED_I2C_PM_OPS (&vmm_shared_i2c_dev_pm_ops)
#else
#define VMM_SHARED_I2C_PM_OPS NULL
#endif


static const struct of_device_id vmm_shared_i2c_of_match[] = {
	{ .compatible = "sofia,vmm-shared-i2c", .data = NULL, },
	{},
};
MODULE_DEVICE_TABLE(of, vmm_shared_i2c_of_match);

static struct platform_driver vmm_shared_i2c_driver = {
	.probe		= vmm_shared_i2c_probe,
	.remove		= vmm_shared_i2c_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "vmm_shared_i2c",
		.pm     = VMM_SHARED_I2C_PM_OPS,
		.of_match_table	= of_match_ptr(vmm_shared_i2c_of_match),
	},
};

static int __init vmm_shared_i2c_init_driver(void)
{
	return platform_driver_register(&vmm_shared_i2c_driver);
}

subsys_initcall(vmm_shared_i2c_init_driver);
