/*
 * ag620_thermal.c - AG620 thermal protection driver.
 *
 * Copyright (C) 2014-2015 Intel Mobile Communications GmbH
 * Copyright (C) 2014-2015 Rockchip Electronics Co., Ltd.
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
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>
#include <linux/wakelock.h>

struct ag620tp_state_data {
	struct delayed_work dw;
	struct wakeup_source *ws;
	struct iio_channel *iio_client;
};

static struct ag620tp_state_data ag620tp_data;

static void ag620tp_delayed_work(struct work_struct *work)
{
	int pmic_temp = 0;
	int ret = 0;

	__pm_stay_awake(ag620tp_data.ws);
	iio_read_channel_raw(ag620tp_data.iio_client, &pmic_temp);
	__pm_relax(ag620tp_data.ws);

	pr_info("%s pmic_temp=%d\n",__func__, pmic_temp);

	mod_delayed_work(system_freezable_wq, &ag620tp_data.dw,
			 msecs_to_jiffies(1000));
}

static int ag620tp_probe(struct platform_device *dev)
{
	struct device *pdev = &dev->dev;
	const char *channel_name;
	int ret = 0;

	ret = of_property_read_string(pdev->of_node, "intel,sensor-names",
				      &channel_name);
	if (ret)
		goto err;

	ag620tp_data.iio_client = iio_channel_get(NULL, channel_name);
	if (IS_ERR(ag620tp_data.iio_client)) {
		dev_err(pdev, "iio channel get error\n");
		goto err;
	}

	ag620tp_data.ws = wakeup_source_register("Ag620-thermal-ws");

	INIT_DELAYED_WORK(&ag620tp_data.dw, ag620tp_delayed_work);
	mod_delayed_work(system_freezable_wq, &ag620tp_data.dw,
			 msecs_to_jiffies(5000));

err:
	return ret;
}

static int ag620tp_remove(struct platform_device *dev)
{
	wakeup_source_unregister(ag620tp_data.ws);
	iio_channel_release(ag620tp_data.iio_client);
	cancel_delayed_work(&ag620tp_data.dw);
	return 0;
}

static int ag620tp_suspend(struct device *dev)
{
	return 0;
}

static int ag620tp_resume(struct device *dev)
{
	return 0;
}

static const struct of_device_id ag620tp_id_table[] = {
	{ .compatible = "intel,ag620_thermal" },
	{}
};
MODULE_DEVICE_TABLE(of, ag620tp_id_table);

const struct dev_pm_ops ag620tp_pm = {
	.suspend = ag620tp_suspend,
	.resume  = ag620tp_resume,
};

static struct platform_driver ag620tp_driver = {
	.driver  = {
		.name  = "ag620_thermal",
		.owner = THIS_MODULE,
		.pm    = &ag620tp_pm,
		.of_match_table =
			of_match_ptr(ag620tp_id_table),
	},
	.probe  = ag620tp_probe,
	.remove = ag620tp_remove,
};

static int __init ag620tp_init(void)
{
   return platform_driver_register(&ag620tp_driver);
}
late_initcall(ag620tp_init);

static void __exit ag620tp_exit(void)
{
   return platform_driver_unregister(&ag620tp_driver);
}
module_exit(ag620tp_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Agold620 thermal protection");
