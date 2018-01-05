/*
 * mfd core driver for rockchip rk616/rk618
 *
 * Copyright (C) 2013-2015 Rockchip Electronics Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/slab.h>
#include <linux/mfd/rk61x.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#if defined(CONFIG_DEBUG_FS)
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#ifndef MHZ
#define MHZ (1000*1000)
#endif

static struct mfd_cell rk61x_devs[] = {
	{
		.name = "rk61x-lvds",
		.id = 0,
	},
	{
		.name = "rk61x-codec",
		.id = 1,
	},
	{
		.name = "rk61x-hdmi",
		.id = 2,
	},
	{
		.name = "rk61x-mipi",
		.id = 3,
	},
};

void rk61x_mclk_set_rate(struct clk *mclk, unsigned long rate)
{
	if (mclk)
		clk_set_rate(mclk, rate);
}

static int rk61x_i2c_read_reg(struct mfd_rk61x *rk61x, u16 reg, u32 *pval)
{
	struct i2c_client *client = rk61x->client;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msgs[2];
	int ret;
	char reg_buf[2];

	memcpy(reg_buf, &reg, 2);

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags | I2C_M_NOSTART;
	msgs[0].len = 2;
	msgs[0].buf = reg_buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags | I2C_M_RD;
	msgs[1].len = 4;
	msgs[1].buf = (char *)pval;

	ret = i2c_transfer(adap, msgs, 2);

	if (ret != 2) {
		dev_err(&client->dev, "I2c read Fail ret=%d\n", ret);
		return ret;
	}

	return (ret == 2) ? 4 : ret;
}

static int rk61x_i2c_write_reg(struct mfd_rk61x *rk61x, u16 reg, u32 *pval)
{
	struct i2c_client *client = rk61x->client;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret;
	char *tx_buf = NULL;

	tx_buf = (char *)devm_kzalloc(rk61x->dev, 6, GFP_KERNEL);
	if (!tx_buf)
		return -ENOMEM;

	memcpy(tx_buf, &reg, 2);
	memcpy(tx_buf + 2, (char *)pval, 4);

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = 6;
	msg.buf = (char *)tx_buf;

	ret = i2c_transfer(adap, &msg, 1);
	devm_kfree(rk61x->dev, tx_buf);

	if (ret != 1) {
		dev_err(&client->dev, "I2c write Fail ret=%d\n", ret);
		return ret;
	}
	return (ret == 1) ? 4 : ret;
}

static int rk61x_i2c_write_bits(struct mfd_rk61x *rk61x, u16 reg,
				u32 mask, u32 *pval)
{
	struct i2c_client *client = rk61x->client;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret;
	u32 reg_val;
	char *tx_buf = NULL;

	tx_buf = (char *)devm_kzalloc(rk61x->dev, 6, GFP_KERNEL);
	if (!tx_buf)
		return -ENOMEM;

	mutex_lock(&rk61x->reg_lock);
	rk61x->read_dev(rk61x, reg, &reg_val);
	reg_val &= ~mask;
	*pval &= mask;
	reg_val |= *pval;
	*pval = reg_val;
	memcpy(tx_buf, &reg, 2);
	memcpy(tx_buf + 2, (char *)pval, 4);

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = 6;
	msg.buf = (char *)tx_buf;

	ret = i2c_transfer(adap, &msg, 1);
	devm_kfree(rk61x->dev, tx_buf);
	mutex_unlock(&rk61x->reg_lock);

	return (ret == 1) ? 4 : ret;
}

static int rk61x_i2c_bulk_write(struct mfd_rk61x *rk61x, u16 reg,
				int count, u32 *pval)
{
	const struct i2c_client *client = rk61x->client;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret;
	char *tx_buf = NULL;

	tx_buf = (char *)devm_kzalloc(rk61x->dev, (count << 2) + 2, GFP_KERNEL);
	if (!tx_buf)
		return -ENOMEM;

	memcpy(tx_buf, &reg, 2);
	memcpy(tx_buf + 2, (char *)pval, count << 2);

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = (count<<2) + 2;
	msg.buf = (char *)tx_buf;

	ret = i2c_transfer(adap, &msg, 1);
	devm_kfree(rk61x->dev, tx_buf);

	return (ret == 1) ? count : ret;
}

#if defined(CONFIG_DEBUG_FS)

static int rk61x_reg_show(struct seq_file *s, void *v)
{
	int i = 0;
	u32 val = 0;
	struct mfd_rk61x *rk61x = s->private;

	if (!rk61x) {
		dev_err(rk61x->dev, "no mfd rk61x!\n");
		return 0;
	}

	for (i = 0; i <= CRU_CFGMISC_CON; i += 4) {
		rk61x->read_dev(rk61x, i, &val);
		if (i % 16 == 0)
			seq_printf(s, "\n0x%04x:", i);
		seq_printf(s, " %08x", val);
	}
	seq_puts(s, "\n");

	return 0;
}

static ssize_t rk61x_reg_write(struct file *file, const char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct mfd_rk61x *rk61x = file->f_path.dentry->d_inode->i_private;
	u32 reg;
	u32 val;
	char kbuf[25];
	int ret = 0;

	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;

	ret = sscanf(kbuf, "%x%x", &reg, &val);
	rk61x->write_dev(rk61x, reg, &val);
	return count;
}

static int rk61x_reg_open(struct inode *inode, struct file *file)
{
	struct mfd_rk61x *rk61x = inode->i_private;

	return single_open(file, rk61x_reg_show, rk61x);
}

static const struct file_operations rk61x_reg_fops = {
	.owner		= THIS_MODULE,
	.open		= rk61x_reg_open,
	.read		= seq_read,
	.write          = rk61x_reg_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

static u32 rk61x_clk_gcd(u32 numerator, u32 denominator)
{
	u32 a, b;

	if (!numerator || !denominator)
		return 0;

	if (numerator > denominator) {
		a = numerator;
		b = denominator;
	} else {
		a = denominator;
		b = numerator;
	}

	while (b != 0) {
		int r = b;

		b = a % b;
		a = r;
	}

	return a;
}

static __maybe_unused int
rk61x_pll_par_calc(u32 fin_hz, u32 fout_hz, u32 *refdiv, u32 *fbdiv,
		   u32 *postdiv1, u32 *postdiv2, u32 *frac)
{
	/* FIXME set postdiv1/2 always 1 */
	u32 gcd;
	u64 fin_64, frac_64;
	u32 f_frac;

	if (!fin_hz || !fout_hz)
		return -EINVAL;

	if (fin_hz / MHZ * MHZ == fin_hz && fout_hz / MHZ * MHZ == fout_hz) {
		fin_hz /= MHZ;
		fout_hz /= MHZ;
		gcd = rk61x_clk_gcd(fin_hz, fout_hz);
		*refdiv = fin_hz / gcd;
		*fbdiv = fout_hz / gcd;
		*postdiv1 = 1;
		*postdiv2 = 1;

		*frac = 0;
	}  else {
		gcd = rk61x_clk_gcd(fin_hz / MHZ, fout_hz / MHZ);
		*refdiv = fin_hz / MHZ / gcd;
		*fbdiv = fout_hz / MHZ / gcd;
		*postdiv1 = 1;
		*postdiv2 = 1;

		*frac = 0;

		f_frac = (fout_hz % MHZ);
		fin_64 = fin_hz;
		do_div(fin_64, (u64)*refdiv);
		frac_64 = (u64)f_frac << 24;
		do_div(frac_64, fin_64);
		*frac = (u32)frac_64;
		pr_info("frac_64=%llx, frac=%u\n", frac_64, *frac);
	}
	pr_info("fin=%u,fout=%u,gcd=%u,refdiv=%u,fbdiv=%u,postdiv1=%u,postdiv2=%u,frac=%u\n",
		fin_hz, fout_hz, gcd, *refdiv, *fbdiv, *postdiv1,
		*postdiv2, *frac);
	return 0;
}

static int rk61x_pll_wait_lock(struct mfd_rk61x *rk61x, int id)
{
	u32 delay = 10;
	u32 val = 0;
	int ret;
	int offset;

	if (id == 0)	/* PLL0 */
		offset = 0;
	else		/* PLL1 */
		offset = 0x0c;

	while (delay >= 1) {
		ret = rk61x->read_dev(rk61x, CRU_PLL0_CON1 + offset, &val);
		if (val & PLL0_LOCK) {
			rk61x_dbg(rk61x->dev, "PLL%d locked\n", id);
			break;
		}
		mdelay(1);
		delay--;
	}

	if (delay == 0)
		dev_err(rk61x->dev, "rk61x wait PLL%d lock time out!\n", id);

	return 0;
}

int rk61x_pll_pwr_down(struct mfd_rk61x *rk61x, int id)
{
	u32 val = 0;
	int ret;
	int offset;

	if (id == 0)	/* PLL0 */
		offset = 0;
	else		/* PLL1 */
		offset = 0x0c;

	val = PLL0_PWR_DN(1);
	ret = rk61x->write_dev(rk61x, CRU_PLL0_CON1 + offset, &val);

	return 0;
}

int rk61x_pll_set_rate(struct mfd_rk61x *rk61x, int id, u32 cfg_val, u32 frac)
{
	u32 val = 0;
	int ret;
	int offset;
	u16 con0 = cfg_val & 0xffff;
	u16 con1 = (cfg_val >> 16) & 0xffff;
	u32 fbdiv = con0 & 0xfff;
	u32 postdiv1 = (con0 >> 12) & 0x7;
	u32 refdiv = con1 & 0x3f;
	u32 postdiv2 = (con1 >> 6) & 0x7;
	u8 mode = !frac;

	if (id == 0) {	/* PLL0 */
		if (((rk61x->pll0_rate >> 32) == cfg_val) &&
		    ((rk61x->pll0_rate & 0xffffffff) == frac)) {
			/* return 0; */
		}
		rk61x->pll0_rate = ((u64)cfg_val << 32) | frac;
		offset = 0;
	} else {		/* PLL1 */
		if (((rk61x->pll1_rate >> 32) == cfg_val) &&
		    ((rk61x->pll1_rate & 0xffffffff) == frac)) {
			/* return 0; */
		}
		rk61x->pll1_rate = ((u64)cfg_val << 32) | frac;
		offset = 0x0c;
	}

	val = PLL0_PWR_DN(1);
	ret = rk61x->write_dev(rk61x, CRU_PLL0_CON1 + offset, &val);

	ret = rk61x->read_dev(rk61x, CRU_PLL0_CON2 + offset, &val);
	val &= 0xff000000;
	if (frac)
		val |= PLL0_FRAC(frac);
	else
		val |= 0x800000;	/* default value */
	ret = rk61x->write_dev(rk61x, CRU_PLL0_CON2 + offset, &val);

	val = PLL0_POSTDIV1(postdiv1) | PLL0_FBDIV(fbdiv) | PLL0_BYPASS(0);
	ret = rk61x->write_dev(rk61x, CRU_PLL0_CON0 + offset, &val);

	val = PLL0_DIV_MODE(mode) | PLL0_POSTDIV2(postdiv2) |
		PLL0_REFDIV(refdiv);
	ret = rk61x->write_dev(rk61x, CRU_PLL0_CON1 + offset, &val);

	val = PLL0_PWR_DN(0);
	ret = rk61x->write_dev(rk61x, CRU_PLL0_CON1 + offset, &val);

	rk61x_pll_wait_lock(rk61x, id);

	return 0;
}

/*
 * rk61x_clk_common_init()
 * default clk patch settiing:
 *	CLKIN-------->CODEC
 *	LCD_DCLK0--->PLL0--->Dither--->LVDS/MIPI
 *	LCD_DCLK1--->PLL1--->HDMI
 */

static int rk61x_clk_common_init(struct mfd_rk61x *rk61x)
{
	u32 val = 0;
	int ret;

	/*
	 * pll1 clk from lcdc1_dclk,pll0 clk from lcdc0_dclk,mux_lcdx = lcdx_clk
	 */
	val = PLL1_CLK_SEL(LCD1_DCLK) | PLL0_CLK_SEL(LCD0_DCLK) |
		LCD1_CLK_DIV(0) | LCD0_CLK_DIV(0);
	ret = rk61x->write_dev(rk61x, CRU_CLKSEL0_CON, &val);

	val = SCLK_SEL(SCLK_SEL_PLL1) | CODEC_MCLK_SEL(CODEC_MCLK_SEL_12M);
	ret = rk61x->write_dev(rk61x, CRU_CLKSEL1_CON, &val);

	val = 0; /* codec mck = clkin */
	ret = rk61x->write_dev(rk61x, CRU_CODEC_DIV, &val);

	/* bypass pll0 */
	val = PLL0_BYPASS(1);
	ret = rk61x->write_dev(rk61x, CRU_PLL0_CON0, &val);
	/* power down pll0 */
	val = PLL0_PWR_DN(1);
	ret = rk61x->write_dev(rk61x, CRU_PLL0_CON1, &val);

	val = PLL1_BYPASS(1);
	ret = rk61x->write_dev(rk61x, CRU_PLL1_CON0, &val);

	return 0;
}

#if defined(CONFIG_PM)
static int rk61x_core_suspend(struct device *dev)
{
	struct mfd_rk61x *rk61x = dev_get_drvdata(dev);
	int ret = 0;

	if (dev->pins->p && dev->pins->sleep_state)
		pinctrl_select_state(dev->pins->p, dev->pins->sleep_state);

	ret = device_state_pm_set_state_by_name(
			dev, rk61x->pm_platdata->pm_state_D3_name);
	if (ret < 0) {
		dev_err(dev, "%s: Error while setting the pm class\n",
			__func__);
		return ret;
	}

	return 0;
}

static int rk61x_core_resume(struct device *dev)
{
	struct mfd_rk61x *rk61x = dev_get_drvdata(dev);
	int ret = 0;

	ret = device_state_pm_set_state_by_name(
			dev, rk61x->pm_platdata->pm_state_D0_name);
	if (ret < 0) {
		dev_err(dev, "%s: Error while setting the pm class\n",
			__func__);
		return ret;
	}

	if (dev->pins->p && dev->pins->default_state)
		pinctrl_select_state(dev->pins->p, dev->pins->default_state);

	rk61x_clk_common_init(rk61x);
	return 0;
}
#else
#define rk61x_core_suspend	NULL
#define rk61x_core_resume		NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops rk61x_core_pm = {
	.suspend = rk61x_core_suspend,
	.resume = rk61x_core_resume,
};

#ifdef CONFIG_OF
static struct rk61x_platform_data *rk61x_parse_dt(struct mfd_rk61x *rk61x)
{
	struct rk61x_platform_data *pdata = NULL;
	struct device_node *rk61x_np = rk61x->dev->of_node;
	int val = 0, gpio = 0;

	if (!rk61x_np) {
		dev_err(rk61x->dev, "could not find rk61x node\n");
		return NULL;
	}

	pdata = devm_kzalloc(rk61x->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	if (!of_property_read_u32(rk61x_np, "rk61x,lcd0_func", &val))
		pdata->lcd0_func = val;
	else
		pdata->lcd0_func = INPUT; /* lcd0 input default */

	if (!of_property_read_u32(rk61x_np, "rk61x,lcd1_func", &val))
		pdata->lcd1_func = val;
	else
		pdata->lcd1_func = UNUSED; /* lcd1 unused default */

	if (!of_property_read_u32(rk61x_np, "rk61x,lvds_ch_nr", &val))
		pdata->lvds_ch_nr = val;
	else
		pdata->lvds_ch_nr = 1; /* single chanel default */

	if (!of_property_read_u32(rk61x_np, "rk61x,pll_clk_sel", &val))
		pdata->pll_clk_sel = val;
	else
		pdata->pll_clk_sel = MCLK_12M; /* from system clock default */

	gpio = of_get_named_gpio(rk61x_np, "rk61x,hdmi_irq_gpio", 0);
	if (!gpio_is_valid(gpio))
		dev_info(rk61x->dev, "invalid hdmi_irq_gpio: %d\n", gpio);
	pdata->hdmi_irq = gpio;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	rk61x->pm_platdata = of_device_state_pm_setup(rk61x_np);
	if (IS_ERR(rk61x->pm_platdata)) {
		dev_info(rk61x->dev, "Error during device state pm init.\n");
		rk61x->pm_platdata = NULL;
	}
#endif

	/* TODO Daisen >>pwr gpio wait to add */

	return pdata;
}
#else
static struct rk61x_platform_data *rk61x_parse_dt(struct mfd_rk61x *rk61x)
{
	return NULL;
}
#endif

#if defined(CONFIG_OF)
static const struct of_device_id rk61x_dt_ids[] = {
	{.compatible = "rockchip,rk61x",},
	{}
};
MODULE_DEVICE_TABLE(of, rk61x_dt_ids);
#endif

static int rk61x_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int ret;
	struct mfd_rk61x *rk61x = NULL;

	if (client->dev.of_node) {
		if (!of_match_device(rk61x_dt_ids, &client->dev)) {
			dev_err(&client->dev, "Failed to find matching dt id\n");
			return -EINVAL;
		}
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Must have I2C_FUNC_I2C.\n");
		ret = -ENODEV;
	}
	rk61x = devm_kzalloc(&client->dev, sizeof(*rk61x), GFP_KERNEL);
	if (rk61x == NULL)
		ret = -ENOMEM;

	rk61x->dev = &client->dev;
	rk61x->pdata = rk61x_parse_dt(rk61x);
	rk61x->client = client;
	i2c_set_clientdata(client, rk61x);
	dev_set_drvdata(rk61x->dev, rk61x);
	mutex_init(&rk61x->reg_lock);

#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (rk61x->pm_platdata) {
		ret = device_state_pm_set_class(
			rk61x->dev, rk61x->pm_platdata->pm_user_name);
		if (ret < 0) {
			dev_err(rk61x->dev,
				"ERROR while rk61x initialize its PM state!\n");
			kfree(rk61x->pm_platdata);
			rk61x->pm_platdata = NULL;
			goto err_out;
		}

		/* power on rk61x system clock */
		ret = device_state_pm_set_state_by_name(
			rk61x->dev, rk61x->pm_platdata->pm_state_D0_name);
		if (ret < 0) {
			dev_err(rk61x->dev, "Error while setting the pm class\n");
			goto err_out;
		}
	}
#endif

	if (rk61x->pdata->power_init)
		rk61x->pdata->power_init();

	rk61x->read_dev = rk61x_i2c_read_reg;
	rk61x->write_dev = rk61x_i2c_write_reg;
	rk61x->write_dev_bits = rk61x_i2c_write_bits;
	rk61x->write_bulk = rk61x_i2c_bulk_write;

#if defined(CONFIG_DEBUG_FS)
	rk61x->debugfs_dir = debugfs_create_dir("rk61x", NULL);
	if (IS_ERR(rk61x->debugfs_dir))
		dev_err(rk61x->dev, "failed to create debugfs dir for rk61x!\n");
	else
		debugfs_create_file("core", S_IRUSR, rk61x->debugfs_dir,
				    rk61x, &rk61x_reg_fops);
#endif

	rk61x_clk_common_init(rk61x);
	ret = mfd_add_devices(rk61x->dev, -1, rk61x_devs,
			      ARRAY_SIZE(rk61x_devs), NULL,
			      rk61x->irq_base, NULL);
	dev_info(rk61x->dev, "rk61x core probe success!\n");
	return 0;

err_out:
	devm_kfree(&client->dev, rk61x->pdata);
	devm_kfree(&client->dev, rk61x);
	return ret;
}

static int rk61x_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static void rk61x_core_shutdown(struct i2c_client *client)
{
	struct mfd_rk61x *rk61x = i2c_get_clientdata(client);

	if (rk61x->pdata->power_deinit)
		rk61x->pdata->power_deinit();
}

static const struct i2c_device_id id_table[] = {
	{"rk61x", 0 },
	{ }
};

static struct i2c_driver rk61x_i2c_driver  = {
	.driver = {
		.name = "rk61x",
		.owner = THIS_MODULE,
		.pm = &rk61x_core_pm,
		.of_match_table = of_match_ptr(rk61x_dt_ids),
	},
	.probe = &rk61x_i2c_probe,
	.remove = &rk61x_i2c_remove,
	.shutdown = &rk61x_core_shutdown,
	.id_table = id_table,
};

static int __init rk61x_module_init(void)
{
	return i2c_add_driver(&rk61x_i2c_driver);
}

static void __exit rk61x_module_exit(void)
{
	i2c_del_driver(&rk61x_i2c_driver);
}

subsys_initcall_sync(rk61x_module_init);
module_exit(rk61x_module_exit);
