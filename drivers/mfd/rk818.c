/*
 * Regulator driver for rk818 PMIC chip for rk31xx
 *
 * Based on rk818.c that is work by zhangqing<zhangqing@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/regulator/driver.h>
#include <linux/mfd/rk818.h>
#include <linux/mfd/core.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <sofia/vmm_pmic.h>
#include <linux/usb/phy.h>
#include <linux/usb/phy-intel.h>

#define PM_CONTROL

struct rk818 *g_rk818;

static struct mfd_cell rk818s[] = {
	{
	 .name = "rk818-rtc",
	 },

	{
	 .name = "rk818-battery",
	 },
};

#define RK818_VMM_I2C_ADDR (0x1c << 24)
#define PARAMETER_NO_USE 1

int rk818_i2c_read(struct rk818 *rk818, u8 reg, int count, u8 *dest)
{
	struct i2c_client *i2c = rk818->i2c;
	struct i2c_adapter *adap;
	struct i2c_msg msgs[2];
	int ret = -1;

	if (!i2c)
		return -ENODEV;

	if (count != 1)
		return -EIO;

	adap = i2c->adapter;

	msgs[0].addr = i2c->addr;
	msgs[0].buf = &reg;
	msgs[0].flags = 0;
	msgs[0].len = 1;

	msgs[1].buf = dest;
	msgs[1].addr = i2c->addr;
	msgs[1].flags =  I2C_M_RD;
	msgs[1].len = count;

	ret = i2c_transfer(adap, msgs, 2);
	if (ret != 2)
		ret = -1;

	return ret;
}
EXPORT_SYMBOL_GPL(rk818_i2c_read);

int rk818_i2c_write(struct rk818 *rk818, u8 reg, int count, u8 src)
{
	struct i2c_client *i2c = rk818->i2c;
	struct i2c_adapter *adap;
	struct i2c_msg msg;
	char tx_buf[2];
	int ret = -1;

	if (!i2c)
		return -ENODEV;
	if (count != 1)
		return -EIO;

	adap = i2c->adapter;
	tx_buf[0] = reg;
	tx_buf[1] = src;

	msg.addr = i2c->addr;
	msg.buf = &tx_buf[0];
	msg.len = 1 + 1;
	msg.flags = i2c->flags;

	ret = i2c_transfer(adap, &msg, 1);
	if (ret != 1)
		ret = -1;

	return ret;
}
EXPORT_SYMBOL_GPL(rk818_i2c_write);

u8 rk818_reg_read(struct rk818 *rk818, u8 reg)
{
	u8 val = 0;
	int ret;

	rt_mutex_lock(&rk818->io_lock);

	ret = rk818_i2c_read(rk818, reg, 1, &val);

	if (ret < 0)
		dev_err(rk818->dev, "Read reg 0x%x failed\n", reg);
	else
		dev_dbg(rk818->dev, "Read reg 0x%02x -> 0x%02x\n",
			reg, val);

	rt_mutex_unlock(&rk818->io_lock);

	return val;
}
EXPORT_SYMBOL_GPL(rk818_reg_read);

int rk818_reg_write(struct rk818 *rk818, u8 reg, u8 val)
{
	int ret = 0;

	rt_mutex_lock(&rk818->io_lock);

	ret = rk818_i2c_write(rk818, reg, 1, val);
	if (ret < 0)
		dev_err(rk818->dev, "Write reg 0x%x failed\n", reg);
	else
		dev_dbg(rk818->dev, "Write reg 0x%02x -> 0x%02x\n", reg, val);

	rt_mutex_unlock(&rk818->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(rk818_reg_write);

int rk818_set_bits(struct rk818 *rk818, u8 reg, u8 mask, u8 val)
{
	u8 tmp;
	int ret;

	rt_mutex_lock(&rk818->io_lock);

	ret = rk818_i2c_read(rk818, reg, 1, &tmp);
	if (ret < 0) {
		dev_err(rk818->dev, "Set bits read reg %x failed\n", reg);
		goto out;
	}
	tmp = (tmp & ~mask) | val;
	ret = rk818_i2c_write(rk818, reg, 1, tmp);
	if (ret < 0)
		dev_err(rk818->dev, "Set bits write reg %x failed\n", reg);

out:
	rt_mutex_unlock(&rk818->io_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(rk818_set_bits);

int rk818_clear_bits(struct rk818 *rk818, u8 reg, u8 mask)
{
	u8 data;
	int ret;

	rt_mutex_lock(&rk818->io_lock);

	ret = rk818_i2c_read(rk818, reg, 1, &data);
	if (ret < 0) {
		dev_err(rk818->dev, "Clear bits read reg %x failed\n", reg);
		goto out;
	}
	data &= ~mask;
	ret = rk818_i2c_write(rk818, reg, 1, data);
	if (ret < 0)
		dev_err(rk818->dev, "Clear bits write reg %x failed\n", reg);

out:
	rt_mutex_unlock(&rk818->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(rk818_clear_bits);

static inline int rk818_set_pinctrl_state(struct pinctrl *pinctrl,
					  struct pinctrl_state *state)
{
	int ret = 0;

	if (!IS_ERR(state))
		ret = pinctrl_select_state(pinctrl, state);

	return ret;
}

#if 1
static ssize_t rk818_test_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t n)
{
	u32 getdata[8];
	u16 reg;
	u8 data;
	int ret;
	char cmd;
	const char *buftmp = buf;
	struct rk818 *rk818 = g_rk818;
	/**
	* W Addr(8Bit) reg(8Bit) data0(8Bit) data1(8Bit) data2(8Bit) data3(8Bit)
	* data can be less than 4 byte
	* R reg(8Bit)
	* C gpio_name(poweron/powerhold/sleep/boot0/boot1) value(H/L)
	*/
	reg = (u16)(getdata[0] & 0xff);
	if (strncmp(buf, "start", 5) == 0) {
		dev_dbg(rk818->dev, "start\n");
	} else if (strncmp(buf, "stop", 4 == 0)) {
		dev_dbg(rk818->dev, "stop\n");
	} else {
		ret = sscanf(buftmp, "%c ", &cmd);
		dev_info(rk818->dev, "------zhangqing: get cmd = %c\n", cmd);
		switch (cmd) {
		case 'w':
			ret = sscanf(buftmp, "%c %x %x ",
				     &cmd, &getdata[0], &getdata[1]);
			reg = (u16)(getdata[0] & 0xff);
			data = (u8)(getdata[1] & 0xff);
			dev_info(rk818->dev, "get value = %x\n", data);

			rk818_i2c_write(rk818, reg, 1, data);
			rk818_i2c_read(rk818, reg, 1, &data);
			dev_info(rk818->dev, "%x   %x\n", getdata[1], data);

			break;

		case 'r':
			ret = sscanf(buftmp, "%c %x ", &cmd, &getdata[0]);
			dev_info(rk818->dev,
				 "CMD : %c %x\n", cmd, getdata[0]);

			reg = (u16)(getdata[0] & 0xff);
			rk818_i2c_read(rk818, reg, 1, &data);
			dev_info(rk818->dev, "%x %x\n", getdata[0], data);

			break;

		case 't':
			ret = sscanf(buftmp, "%c %x ", &cmd, &getdata[0]);
			dev_info(rk818->dev,
				 "CMD : %c %x\n", cmd, getdata[0]);
			if (getdata[0] == 1) {
				rk818->bat_test_mode = 1;
				dev_info(rk818->dev,
					 "You open battery test mode, capacity will always report fixed value\n"
					);
			} else{
				rk818->bat_test_mode = 0;
				dev_info(rk818->dev, "close battery test mode\n");
			}
			break;

		default:
			dev_info(rk818->dev, "Unknown command\n");
			break;
		}
	}
	return n;
}

static ssize_t rk818_test_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	buf = "hello";
	return sprintf(s, "%s\n", buf);
}

static struct kobject *rk818_kobj;
struct rk818_attribute {
	struct attribute attr;
	 ssize_t (*show)(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf);
	 ssize_t (*store)(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t n);
};

static struct rk818_attribute rk818_attrs[] = {
	/*node_name   permision   show_func   store_func */
	__ATTR(rk818_test, S_IRUGO | S_IWUSR, rk818_test_show,
	       rk818_test_store),
};
#endif

#ifdef CONFIG_OF
static struct of_device_id rk818_of_match[] = {
	{.compatible = "rockchip,rk818"},
	{},
};

MODULE_DEVICE_TABLE(of, rk818_of_match);
#endif

#ifdef CONFIG_OF
static int rk818_parse_dt(struct rk818 *rk818)
{
	struct device_node *rk818_pmic_np;
	int ret = 0;

	rk818_pmic_np = of_node_get(rk818->dev->of_node);
	if (!rk818_pmic_np) {
		dev_err(rk818->dev, "could not find pmic sub-node\n");
		return -ENODEV;
	}
	rk818->pinctrl = devm_pinctrl_get(rk818->dev);
	if (IS_ERR(rk818->pinctrl)) {
		PTR_ERR(rk818->pinctrl);
		dev_err(rk818->dev, "could not get pinctrl\n");
	}
	rk818->pins_default = pinctrl_lookup_state(
		rk818->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(rk818->pins_default))
		dev_err(rk818->dev, "could not get default pinstate\n");
	rk818->pins_sleep = pinctrl_lookup_state(
			rk818->pinctrl, PINCTRL_STATE_SLEEP);
	if (IS_ERR(rk818->pins_sleep))
		dev_err(rk818->dev, "could not get sleep pinstate\n");
	rk818->pins_inactive = pinctrl_lookup_state(
			rk818->pinctrl, "inactive");
	if (IS_ERR(rk818->pins_inactive))
		dev_err(rk818->dev, "could not get inactive pinstate\n");

	of_property_read_u32(rk818_pmic_np, "notify_usb_det",
			     &rk818->notify_usb_det);
	rk818->chip_irq = irq_of_parse_and_map(rk818_pmic_np, 0);
	dev_info(rk818->dev, "rk818 irq number: %d, notify_usb_det: %d\n",
		 rk818->chip_irq, rk818->notify_usb_det);

#ifdef CONFIG_PLATFORM_DEVICE_PM
	rk818->pm_platdata = of_device_state_pm_setup(rk818_pmic_np);
	if (IS_ERR(rk818->pm_platdata)) {
		dev_err(rk818->dev, "Error during device state pm init\n");
		return -ENOMEM;
	}
	ret = device_state_pm_set_class(rk818->dev,
					rk818->pm_platdata->pm_user_name);
#endif

	return ret;
}

#else
static int rk818_parse_dt(struct i2c_client *i2c)
{
	return NULL;
}
#endif

static int rk818_otg_enable(struct rk818 *rk818)
{
	int ret = 0;

	dev_info(rk818->dev, "rk818 otg en set pm state D0\n");
#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (rk818->pm_platdata) {
		ret = device_state_pm_set_state_by_name(
			rk818->dev, rk818->pm_platdata->pm_state_D0_name);
		if (ret < 0) {
			dev_err(rk818->dev, "Error while setting pm state D0\n");
			return ret;
		}
	}
#endif

	return 0;
}

static int rk818_otg_disable(struct rk818 *rk818)
{
	int ret = 0;

	dev_info(rk818->dev, "rk818 otg en set pm state D3\n");
#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (rk818->pm_platdata) {
		ret = device_state_pm_set_state_by_name(
			rk818->dev, rk818->pm_platdata->pm_state_D3_name);
		if (ret < 0) {
			dev_err(rk818->dev, "Error while setting pm state D3\n");
			return ret;
		}
	}
#endif

	return 0;
}

void rk818_device_shutdown(void)
{
	int ret;
	struct rk818 *rk818 = g_rk818;
	u8 val;

	dev_info(rk818->dev, "%s\n", __func__);

	val = SYS_REBOOT_SHUTDOWN_MAGIC | SYS_DEBUG_SHUTDOWN;
	ret = rk818_reg_write(rk818, RK818_DATA19_REG_SYS_DEBUG, val);
	if (ret < 0)
		dev_err(rk818->dev, "Write DATA19: %x ERR\n", val);

	dev_info(rk818->dev, "DATA19_REG_SYS_DEBUG = %x\n", val);

	/*close rtc int when power off*/
	ret = rk818_set_bits(rk818, RK818_INT_STS_MSK_REG1,
			     (0x3 << 5), (0x3 << 5));
	ret = rk818_clear_bits(rk818, RK818_RTC_INT_REG, (0x3 << 2));
	ret = rk818_reg_read(rk818, RK818_DEVCTRL_REG);
	ret = rk818_set_bits(rk818, RK818_DEVCTRL_REG, (0x1 << 0), (0x1 << 0));
	/*ret = rk818_set_bits(rk818, RK818_DEVCTRL_REG,(0x1<<4),(0x1<<4));*/
	if (ret < 0)
		dev_err(rk818->dev, "rk818 power off error!\n");
}
EXPORT_SYMBOL_GPL(rk818_device_shutdown);

__weak void rk818_device_suspend(void)
{
}

__weak void rk818_device_resume(void)
{
}

#ifdef CONFIG_PM
static int rk818_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct rk818 *rk818 = i2c_get_clientdata(client);
	u8 val;

	val = rk818_reg_read(rk818, RK818_DATA19_REG_SYS_DEBUG);
	val = (val & SYS_RUN_MAGIC) | SYS_DEBUG_SUSPEND;
	ret = rk818_reg_write(rk818, RK818_DATA19_REG_SYS_DEBUG, val);
	if (ret < 0) {
		dev_err(rk818->dev, "Write DATA19: %x ERR\n", val);
		return ret;
	}
	dev_info(rk818->dev, "DATA19_REG_SYS_DEBUG = %x\n", val);

	rk818_device_suspend();
	ret = rk818_set_pinctrl_state(rk818->pinctrl, rk818->pins_sleep);
	if (ret < 0) {
		dev_err(rk818->dev, "rk818_set_pinctrl_state sleep error\n");
		return -EBUSY;
	}
	cancel_delayed_work_sync(&rk818->otg_delay_work);
	return 0;
}

static int rk818_resume(struct i2c_client *client)
{
	int ret;
	struct rk818 *rk818 = i2c_get_clientdata(client);
	u8 val;

	ret = rk818_set_pinctrl_state(rk818->pinctrl, rk818->pins_default);
	if (ret < 0) {
		dev_err(rk818->dev, "rk818_set_pinctrl_state default error\n");
		return -EBUSY;
	}
	rk818_device_resume();

	val = rk818_reg_read(rk818, RK818_DATA19_REG_SYS_DEBUG);
	val = (val & SYS_RUN_MAGIC) | SYS_DEBUG_RESUME;
	ret = rk818_reg_write(rk818, RK818_DATA19_REG_SYS_DEBUG, val);
	if (ret < 0) {
		dev_err(rk818->dev, "Write DATA19: %x ERR\n", val);
		return ret;
	}
	dev_info(rk818->dev, "DATA19_REG_SYS_DEBUG = %x\n", val);
	return 0;
}
#else
static int rk818_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int rk818_resume(struct i2c_client *client)
{
	return 0;
}
#endif

static int rk818_pre_init(struct rk818 *rk818)
{
	int ret;
	u8 val;

	dev_info(rk818->dev, "ON_SOURCE:0x%02x OFF_SOURCE:0x%02x\n",
		 rk818_reg_read(rk818, RK818_ON_SOURCE_REG),
		 rk818_reg_read(rk818, RK818_OFF_SOURCE_REG));

	/*close charger when usb low then 3.4V*/
	ret = rk818_set_bits(rk818, 0xa1, (0x7 << 4), (0x7 << 4));
	/*no action when vref*/
	ret = rk818_set_bits(rk818, 0x52, (0x1 << 1), (0x1 << 1));

	/****************set vbat low **********/
	val = rk818_reg_read(rk818, RK818_VB_MON_REG);
	val &= (~(VBAT_LOW_VOL_MASK | VBAT_LOW_ACT_MASK));
	val |= (RK818_VBAT_LOW_3V5 | EN_VBAT_LOW_IRQ);
	ret = rk818_reg_write(rk818, RK818_VB_MON_REG, val);
	if (ret < 0) {
		dev_err(rk818->dev, "Unable to write RK818_VB_MON_REG reg\n");
		return ret;
	}
	/**************************************/

	/**********mask int****************/
	val = rk818_reg_read(rk818, RK818_INT_STS_MSK_REG1);
	val |= (0x1 << 0);	/*mask vout_lo_int*/
	ret = rk818_reg_write(rk818, RK818_INT_STS_MSK_REG1, val);
	if (ret < 0) {
		dev_err(rk818->dev, "Unable to write RK818_INT_STS_MSK_REG1 reg\n");
		return ret;
	}
	/**********enable clkout2****************/
	ret = rk818_reg_write(rk818, RK818_CLK32OUT_REG, 0x01);
	if (ret < 0) {
		dev_err(rk818->dev, "Unable to write RK818_CLK32OUT_REG reg\n");
		return ret;
	}

	/*open rtc int when power on*/
	ret = rk818_clear_bits(rk818, RK818_INT_STS_MSK_REG1, (0x3 << 5));
	ret = rk818_set_bits(rk818, RK818_RTC_INT_REG, (0x1 << 3), (0x1 << 3));
	ret = rk818_set_bits(rk818, CHRG_CTRL_REG3, 1 << 6, 1 << 6);

	val = rk818_reg_read(rk818, RK818_DEVCTRL_REG);
	if (val & RK818_PWRON_LP_ACT_MASK) {
		dev_err(rk818->dev, "ERROR: long press action is reset, clear\n");
		ret = rk818_set_bits(rk818, RK818_DEVCTRL_REG,
				     RK818_PWRON_LP_ACT_MASK |
				     RK818_PWRON_LP_OFF_TIME_MASK,
				     RK818_PWRON_LP_ACT_SHUTDOWN |
				     RK818_PWRON_LP_OFF_TIME_6S);
	}

	val = rk818_reg_read(rk818, RK818_DATA19_REG_SYS_DEBUG);
	ret = rk818_reg_write(rk818, RK818_DATA18_REG_SYS_DEBUG_COPY, val);
	if (ret < 0) {
		dev_err(rk818->dev, "Write DATA18_REG_SYS_DEBUG_COPY ERR\n");
		return ret;
	}
	ret = rk818_reg_write(rk818, RK818_DATA19_REG_SYS_DEBUG, SYS_RUN_MAGIC);
	if (ret < 0) {
		dev_err(rk818->dev, "Write DATA19_REG_SYS_DEBUG ERR\n");
		return ret;
	}
	dev_info(rk818->dev, "DATA18 = %x, DATA19 = %x\n", val, SYS_RUN_MAGIC);

	return ret;
}

static int rk81x_get_chip_id(struct rk818 *rk818)
{
	u8 val;
	int id = 0;

	val = rk818_reg_read(rk818, RK818_CHIP_NAME_MSB_REG);
	id |= val << 8;
	val = rk818_reg_read(rk818, RK818_CHIP_NAME_LSB_REG);
	id |= val;

	if (id == RK818_CHIP_ID_NUM) {
		rk818->chip_id = PMIC_CHIP_RK818;
		dev_info(rk818->dev, "RK81X chip id is rk818\n");
	} else if (id == RK819_CHIP_ID_NUM) {
		rk818->chip_id = PMIC_CHIP_RK819;
		dev_info(rk818->dev, "RK81X chip id is rk819\n");
	} else {
		rk818->chip_id = PMIC_CHIP_UNKNOWN;
		dev_err(rk818->dev, "RK81X chip id is unknown\n");
	}

	return rk818->chip_id;
}

static irqreturn_t rk81x_usb_plug_in(int irq, void *di)
{
	struct rk818 *rk818 = (struct rk818 *)di;

	rk818->vbus_state_prev = rk818->vbus;
	rk818->vbus = VBUS_ON;

	if ((rk818->vbus != rk818->vbus_state_prev) &&
	    rk818->notify_usb_det) {
		if (rk818->otg_handle) {
			atomic_notifier_call_chain(&rk818->otg_handle->notifier,
						   USB_EVENT_VBUS,
						   &rk818->vbus);
		}
		rk818->vbus_state_prev = rk818->vbus;
	}
	return IRQ_HANDLED;
}

static irqreturn_t rk81x_usb_plug_out(int irq, void *di)
{
	struct rk818 *rk818 = (struct rk818 *)di;

	rk818->vbus_state_prev = rk818->vbus;
	rk818->vbus = VBUS_OFF;

	if ((rk818->vbus != rk818->vbus_state_prev) &&
	    rk818->notify_usb_det) {
		if (rk818->otg_handle) {
			atomic_notifier_call_chain(&rk818->otg_handle->notifier,
						   USB_EVENT_VBUS,
						   &rk818->vbus);
		}
		rk818->vbus_state_prev = rk818->vbus;
	}

	return IRQ_HANDLED;
}

static int rk81x_usb_plug_irq_init(struct rk818 *rk818)
{
	int plug_in_irq, plug_out_irq;
	int ret;

	plug_in_irq	= rk818->irq_base + RK818_IRQ_PLUG_IN;
	plug_out_irq	= rk818->irq_base + RK818_IRQ_PLUG_OUT;

	ret = request_threaded_irq(plug_in_irq, NULL, rk81x_usb_plug_in,
				   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				   "rk81x_usb_plug_in", rk818);
	if (ret != 0)
		dev_err(rk818->dev, "plug_in_irq request failed!\n");

	ret = request_threaded_irq(plug_out_irq, NULL, rk81x_usb_plug_out,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "rk81x_usb_plug_out", rk818);
	if (ret != 0)
		dev_err(rk818->dev, "plug_out_irq request failed!\n");

	return ret;
}

static void rk81x_otg_notifier_handler_work(struct work_struct *work)
{
	struct rk818 *rk818 = container_of(work,
					struct rk818, otg_delay_work.work);

	if (rk818->otg_en_boost) {
		dev_info(rk818->dev, "rk81x enable otg, disable charge\n");
		rk818_set_bits(rk818, RK818_INT_STS_MSK_REG2,
			       3 << 0, 3 << 0);
		rk818_otg_enable(rk818);
		/* otg plugin, set sleep otg_en and boost on. */
		rk818_clear_bits(rk818, RK818_SLEEP_SET_OFF_REG1,
				 1 << 7 | 1 << 4);
	} else {
		dev_info(rk818->dev, "rk81x disable otg, enable charge\n");
		rk818_otg_disable(rk818);
		/* otg plugout, set sleep otg_en and boost off. */
		rk818_set_bits(rk818, RK818_SLEEP_SET_OFF_REG1,
			       1 << 7 | 1 << 4, 1 << 7 | 1 << 4);
		msleep(50);
		rk818_clear_bits(rk818, RK818_INT_STS_MSK_REG2,
				 3 << 0);
	}
}

static int rk81x_otg_notifier_handler(struct notifier_block *nb,
				      unsigned long event, void *data)
{
	struct rk818 *rk818 = container_of(nb,
					struct rk818, otg_nb);

	if (!data)
		return NOTIFY_BAD;

	switch (event) {
	case INTEL_USB_DRV_VBUS:
		rk818->otg_en_boost = *((bool *)data);
		queue_delayed_work(rk818->wq, &rk818->otg_delay_work,
				   msecs_to_jiffies(50));
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static int rk818_handle_usb_init(struct rk818 *rk818)
{
	u8 sup_sts_reg;
	int ret;

	sup_sts_reg = rk818_reg_read(rk818, RK818_SUP_STS_REG);

	rk818->otg_handle = usb_get_phy(USB_PHY_TYPE_USB2);
	if (IS_ERR_OR_NULL(rk818->otg_handle)) {
		dev_err(rk818->dev, "get usb phy failed\n");
		return PTR_ERR(rk818->otg_handle);
	}

	rk818->vbus = VBUS_OFF;
	rk818->vbus_state_prev = VBUS_OFF;
	if (sup_sts_reg & 0x02) {
		rk818->vbus = VBUS_ON;
		if (rk818->otg_handle && rk818->notify_usb_det) {
			atomic_notifier_call_chain(&rk818->otg_handle->notifier,
						   USB_EVENT_VBUS,
						   &rk818->vbus);
		}
		rk818->vbus_state_prev = VBUS_ON;
	}
	rk818->otg_nb.notifier_call = rk81x_otg_notifier_handler;
	ret = usb_register_notifier(rk818->otg_handle, &rk818->otg_nb);
	if (ret)
		dev_err(rk818->dev, "registr OTG notification failed\n");

	return ret;
}

#if defined(CONFIG_DEBUG_FS)
static int rk81x_debugfs_reg_show(struct seq_file *s, void *v)
{
	int i = 0;
	int val;
	struct rk818 *rk818 = s->private;

	for (i = 0; i < 0xff; i++) {
		val = rk818_reg_read(rk818, i);
		seq_printf(s, "0x%02x : 0x%02x\n", i, val);
	}

	return 0;
}

static ssize_t rk81x_debugfs_reg_write(struct file *file,
				       const char __user *buf,
				       size_t count, loff_t *ppos)
{
	struct rk818 *rk818 = ((struct seq_file *)file->private_data)->private;
	int reg, val;
	char kbuf[16];

	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;

	kbuf[count] = '\0';
	if (sscanf(kbuf, " %x %x", &reg, &val) != 2)
		return -EINVAL;
	rk818_reg_write(rk818, reg, val);

	return count;
}

static int rk81x_debugfs_reg_open(struct inode *inode, struct file *file)
{
	struct rk818 *rk818 = inode->i_private;

	return single_open(file, rk81x_debugfs_reg_show, rk818);
}

static const struct file_operations rk81x_reg_fops = {
	.owner          = THIS_MODULE,
	.open           = rk81x_debugfs_reg_open,
	.read           = seq_read,
	.write		= rk81x_debugfs_reg_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};
#endif

static int rk818_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct rk818 *rk818;
	const struct of_device_id *match;
	int ret = 0, i = 0;

	if (client->dev.of_node) {
		match = of_match_device(rk818_of_match, &client->dev);
		if (!match) {
			pr_err("Failed to find matching dt id\n");
			return -EINVAL;
		}
	}

	rk818 = devm_kzalloc(&client->dev, sizeof(struct rk818), GFP_KERNEL);
	if (rk818 == NULL) {
		ret = -ENOMEM;
		goto err;
	}
	rk818->dev = &client->dev;
	rk818->i2c = client;
	i2c_set_clientdata(client, rk818);

	rt_mutex_init(&rk818->io_lock);

	if (rk81x_get_chip_id(rk818) == PMIC_CHIP_UNKNOWN)
		goto err;

	ret = rk818_pre_init(rk818);
	if (ret < 0) {
		dev_err(rk818->dev, "The rk818_pre_init failed %d\n", ret);
		goto err;
	}
	if (rk818->dev->of_node)
		rk818_parse_dt(rk818);

	ret = rk818_set_pinctrl_state(rk818->pinctrl, rk818->pins_default);
	if (ret < 0)
		goto err;

	ret = rk818_irq_init(rk818);
	if (ret < 0)
		goto err;

	rk818->wq = create_singlethread_workqueue("rk81x_otg_work");
	INIT_DELAYED_WORK(&rk818->otg_delay_work,
			  rk81x_otg_notifier_handler_work);
	rk81x_usb_plug_irq_init(rk818);
	rk818_handle_usb_init(rk818);

	ret = mfd_add_devices(rk818->dev, -1,
			      rk818s, ARRAY_SIZE(rk818s), NULL, 0, NULL);

	g_rk818 = rk818;
	if (!pm_power_off)
		pm_power_off = rk818_device_shutdown;

	rk818_kobj = kobject_create_and_add("rk818", NULL);
	if (!rk818_kobj)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(rk818_attrs); i++) {
		ret = sysfs_create_file(rk818_kobj, &rk818_attrs[i].attr);
		if (ret != 0) {
			dev_err(rk818->dev, "create index %d error\n", i);
			return ret;
		}
	}

#if defined(CONFIG_DEBUG_FS)
	rk818->debugfs_dir = debugfs_create_dir("rk81x", NULL);
	if (IS_ERR(rk818->debugfs_dir))
		dev_err(rk818->dev, "failed to create debugfs dir\n");
	else
		debugfs_create_file("reg", S_IRUSR, rk818->debugfs_dir,
				    rk818, &rk81x_reg_fops);
#endif

	return 0;

err:
	mfd_remove_devices(rk818->dev);
	return ret;
}

static int rk818_i2c_remove(struct i2c_client *client)
{
	int ret;
	struct rk818 *rk818 = i2c_get_clientdata(client);

	ret = rk818_set_pinctrl_state(rk818->pinctrl, rk818->pins_inactive);
	if (ret < 0) {
		dev_err(rk818->dev, "rk818_set_pinctrl_state inactive error\n");
		return -EBUSY;
	}
	cancel_delayed_work_sync(&rk818->otg_delay_work);
	i2c_set_clientdata(client, NULL);
	kfree(rk818);
	return 0;
}

static void rk818_shutdown(struct i2c_client *client)
{
	int ret;
	u8 val;
	struct rk818 *rk818 = i2c_get_clientdata(client);

	rk818_otg_disable(rk818);

	val = SYS_REBOOT_SHUTDOWN_MAGIC | SYS_DEBUG_REBOOT;
	ret = rk818_reg_write(rk818, RK818_DATA19_REG_SYS_DEBUG, val);
	if (ret < 0)
		dev_err(rk818->dev, "Write DATA19: %x ERR\n", val);

	dev_info(rk818->dev, "DATA19_REG_SYS_DEBUG = %x\n", val);
}

static const struct i2c_device_id rk818_i2c_id[] = {
	{ "rk818", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, rk818_i2c_id);

static struct i2c_driver rk818_i2c_driver = {
	.driver = {
		.name = "rk818",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rk818_of_match),
	},
	.probe    = rk818_probe,
	.remove   = rk818_i2c_remove,
	.id_table = rk818_i2c_id,
	#ifdef CONFIG_PM
	.suspend	= rk818_suspend,
	.resume		= rk818_resume,
	#endif
	.shutdown = rk818_shutdown,
};

static int __init rk818_module_init(void)
{
	int ret;

	ret = i2c_add_driver(&rk818_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register I2C driver: %d\n", ret);
	return ret;
}

subsys_initcall_sync(rk818_module_init);

static void __exit rk818_module_exit(void)
{
	i2c_del_driver(&rk818_i2c_driver);
}

module_exit(rk818_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhangqing <zhangqing@rock-chips.com>");
MODULE_DESCRIPTION("rk818 PMIC driver");
