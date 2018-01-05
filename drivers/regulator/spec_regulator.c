/*
 * Regulator driver for special power control
 *
 * Based on spec_regular.c that is work by chenshunqing<csq@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/of_device.h>

struct special_pow_ctrl {
	struct device *dev;
	struct workqueue_struct *wq;
	struct delayed_work spec_pow_delay_work;
	int delay_time;
#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device_pm_platdata *pm_platdata;
#endif
};

static struct of_device_id spec_pow_of_match[] = {
	{.compatible = "rockchip,spec_pow_ctrl"},
	{},
};

int spec_pow_enable(struct special_pow_ctrl *spec)
{
#ifdef CONFIG_PLATFORM_DEVICE_PM
	int ret;

	if (spec->pm_platdata) {
		ret = device_state_pm_set_state_by_name(
			spec->dev, spec->pm_platdata->pm_state_D0_name);
		if (ret < 0) {
			dev_err(spec->dev, "Error while setting pm state D0\n");
			return -1;
		}
	}
#endif
	return 0;
}

int spec_pow_disable(struct special_pow_ctrl *spec)
{
#ifdef CONFIG_PLATFORM_DEVICE_PM
	int ret;

	if (spec->pm_platdata) {
		ret = device_state_pm_set_state_by_name(
			spec->dev, spec->pm_platdata->pm_state_D3_name);
		if (ret < 0) {
			dev_err(spec->dev, "Error while setting pm state D3\n");
			return -1;
		}
	}
#endif
	return 0;
}

static void spec_pow_ctrl_handler_work(struct work_struct *work)
{
	struct special_pow_ctrl *spec = container_of(work,
						     struct special_pow_ctrl,
						     spec_pow_delay_work.work);

	spec_pow_disable(spec);
}

static int spec_pow_parse_dt(struct special_pow_ctrl *spec)
{
	struct device_node *spec_pmic_np;

	spec_pmic_np = of_node_get(spec->dev->of_node);
	if (!spec_pmic_np) {
		dev_err(spec->dev,
			"could not find special power contorl node\n");
		return -ENODEV;
	}
	of_property_read_u32(spec_pmic_np, "pow_disable_delay_ms",
			     &spec->delay_time);

	return 0;
}

static int spec_pow_ctrl_probe(struct platform_device *pdev)
{
	struct special_pow_ctrl *spec;
	struct device_node *special_pmic_np;
	const struct of_device_id *match;
	int ret = 0;

	if (pdev->dev.of_node) {
		match = of_match_device(spec_pow_of_match, &pdev->dev);
		if (!match) {
			pr_err("Failed to find matching dt id\n");
			return -EINVAL;
		}
	}
	spec = devm_kzalloc(&pdev->dev, sizeof(struct special_pow_ctrl),
			    GFP_KERNEL);
	if (spec == NULL)
		return -ENOMEM;

	spec->dev = &pdev->dev;
	platform_set_drvdata(pdev, spec);
	spec_pow_parse_dt(spec);

#ifdef CONFIG_PLATFORM_DEVICE_PM
	special_pmic_np = of_node_get(spec->dev->of_node);
	spec->pm_platdata = of_device_state_pm_setup(special_pmic_np);
	if (IS_ERR(spec->pm_platdata)) {
		dev_err(spec->dev, "Error during device state pm init\n");
		return -ENOMEM;
	}
	ret = device_state_pm_set_class(spec->dev,
					spec->pm_platdata->pm_user_name);
#endif
	spec_pow_enable(spec);
	spec->wq = create_singlethread_workqueue("spec_pow_ctrl_work");
	INIT_DELAYED_WORK(&spec->spec_pow_delay_work,
			  spec_pow_ctrl_handler_work);
	queue_delayed_work(spec->wq, &spec->spec_pow_delay_work,
			   msecs_to_jiffies(spec->delay_time));

	return 0;
}

static int spec_pow_ctrl_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct special_pow_ctrl *spec;

	spec = platform_get_drvdata(pdev);
	cancel_delayed_work_sync(&spec->spec_pow_delay_work);
	spec_pow_disable(spec);

	return 0;
}

static int spec_pow_ctrl_remove(struct platform_device *pdev)
{
	struct special_pow_ctrl *spec;

	spec = platform_get_drvdata(pdev);
	cancel_delayed_work_sync(&spec->spec_pow_delay_work);
	return 0;
}

static void spec_pow_ctrl_shutdown(struct platform_device *pdev)
{
	struct special_pow_ctrl *spec;

	spec = platform_get_drvdata(pdev);
	cancel_delayed_work_sync(&spec->spec_pow_delay_work);
}

static struct platform_driver spec_pow_crrl_driver = {
	.driver = {
		.name = "spec_pow_ctrl",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(spec_pow_of_match),
	},
	.probe    = spec_pow_ctrl_probe,
	.suspend = spec_pow_ctrl_suspend,
	.remove   = spec_pow_ctrl_remove,
	.shutdown = spec_pow_ctrl_shutdown,
};

static int __init spec_pow_ctrl_module_init(void)
{
	int ret;

	ret = platform_driver_register(&spec_pow_crrl_driver);
	if (ret != 0)
		pr_err("Failed to register I2C driver: %d\n", ret);
	return ret;
}

late_initcall_sync(spec_pow_ctrl_module_init);

static void __exit spec_pow_ctrl_module_exit(void)
{
	platform_driver_unregister(&spec_pow_crrl_driver);
}

module_exit(spec_pow_ctrl_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("chenshunqing <csq@rock-chips.com>");
MODULE_DESCRIPTION("special power control driver");
