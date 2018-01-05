/******************************************************************/
/* Copyright (C) 2014-2015 Fuzhou Rockchip Electronics Co., Ltd   */
/*******************************************************************
File    :   sofia3gr_dev.c
Desc    :
Author  :   ZYF
Date    :   2015-05-15
Notes   :
Revision 1.00  2015/05/15 ZYF
Init file.
********************************************************************/
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/blkpg.h>
#include <linux/spinlock.h>
#include <linux/hdreg.h>
#include <linux/init.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/pinctrl/consumer.h>
#include "sofia3gr_ftl.h"
#include <linux/dma-mapping.h>

struct nandc_info {
	void __iomem	*reg_base;
	int				irq;
	int				clk_rate;
	struct clk		*clk;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
#ifdef CONFIG_PLATFORM_DEVICE_PM
	struct device_pm_platdata *pm_platdata;
#endif
};

static struct nandc_info *nfi;
static struct device *n_dev;

void nand_malloc_dma_buf(int size , void **dma_addr, void **cpu_addr)
{
	*cpu_addr = dmam_alloc_coherent(n_dev, size,
				(dma_addr_t *)dma_addr, GFP_KERNEL);
}

void *nand_page_address(const struct page *page)
{
	return page_address(page);
}

static int nand_suspend(struct platform_device *pdev, pm_message_t state)
{
	dev_err(n_dev, "nand_suspend\n");
	nand_dev_suspend();
#ifdef CONFIG_PLATFORM_DEVICE_PM
	device_state_pm_set_state_by_name(&pdev->dev,
					  nfi->pm_platdata->pm_state_D3_name);
#endif
	return 0;
}

static int nand_resume(struct platform_device *pdev)
{
	dev_err(n_dev, "nand_resume\n");
#ifdef CONFIG_PLATFORM_DEVICE_PM
	device_state_pm_set_state_by_name(&pdev->dev,
					  nfi->pm_platdata->pm_state_D2_name);
#endif
	nand_dev_resume();
	return 0;
}

static void nand_shutdown(struct platform_device *pdev)
{
	dev_err(n_dev, "nand_shutdown...\n");
	nand_dev_shutdown();
	dev_err(n_dev, "nand_shutdown:OK\n");
}

static int nand_probe(struct platform_device *pdev)
{
	int ret = -1;
	int irq;
	struct resource *mem;
	void __iomem *membase;

	n_dev = &pdev->dev;
	nfi = devm_kzalloc(&pdev->dev, sizeof(struct nandc_info), GFP_KERNEL);
	gp_nandc_info = nfi;
	if (!nfi)
		return -ENOMEM;
	pdev->dev.platform_data = nfi;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	membase = devm_request_and_ioremap(&pdev->dev, mem);
	if (membase == 0) {
		dev_err(n_dev, "no reg resource?\n");
		return -1;
	}
	irq = platform_get_irq(pdev, 0);
	dev_info(n_dev, "nand_probe %x %x %d\n", (int)mem,
		 (int)membase, irq);
	if (irq < 0) {
		dev_err(n_dev, "no irq resource?\n");
		return irq;
	}
	nfi->irq = irq;
	nfi->reg_base = membase;
	nfi->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(gp_nandc_info->pinctrl)) {
		dev_err(n_dev, "unable to get pinctrl for sdhci\n");
		goto err_end;
	}
	nfi->pins_default =
		pinctrl_lookup_state(nfi->pinctrl,
				     PINCTRL_STATE_DEFAULT);
	nfi->pins_sleep =
		pinctrl_lookup_state(nfi->pinctrl,
				     PINCTRL_STATE_SLEEP);
	nfi->pins_inactive =
		pinctrl_lookup_state(nfi->pinctrl, "inactive");
	ret = pinctrl_select_state(nfi->pinctrl,
				   nfi->pins_default);

#ifdef CONFIG_PLATFORM_DEVICE_PM
	nfi->pm_platdata =
		of_device_state_pm_setup(pdev->dev.of_node);
	if (IS_ERR(nfi->pm_platdata)) {
		dev_err(n_dev, "Error during device state pm init.\n");
		goto err_end;
	}
	ret = device_state_pm_set_class(&pdev->dev,
					nfi->pm_platdata->pm_user_name);
	if (ret) {
		dev_err(n_dev, "Error while setting the pm clock ctrl class\n");
		return -1;
	}
	device_state_pm_set_state_by_name(&pdev->dev,
					  nfi->pm_platdata->pm_state_D3_name);
	device_state_pm_set_state_by_name(&pdev->dev,
					  nfi->pm_platdata->pm_state_D2_name);
	nfi->clk_rate = 156;
#endif
	ret = init_nand_blk_dev(&pdev->dev, (unsigned int)membase);
	if (ret)
		dev_err(n_dev, "init_nand_blk_dev:ret = %x\n", ret);
err_end:
	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id of_rk_nandc_match[] = {
	{ .compatible = "intel,nand" },
	{ .compatible = "rockchip,nand" },
	{ /* Sentinel */ }
};
#endif

static struct platform_driver nand_driver = {
	.probe		= nand_probe,
	.suspend	= nand_suspend,
	.resume		= nand_resume,
	.shutdown   = nand_shutdown,
	.driver		= {
	    .name	= "nand",
#ifdef CONFIG_OF
		.of_match_table = of_rk_nandc_match,
#endif
		.owner	= THIS_MODULE,
	},
};

static int __init nand_init(void)
{
	int ret;

	n_dev = NULL;
	ret = platform_driver_register(&nand_driver);
	if (ret)
		dev_err(n_dev, "nand_driver:ret = %x\n", ret);
	return ret;
}

static void __exit nand_exit(void)
{
	;
}

module_init(nand_init);
module_exit(nand_exit);

MODULE_LICENSE("Proprietary");
MODULE_AUTHOR("Fuzhou Rockchip Electronics");
MODULE_DESCRIPTION("NAND FTL Block Interface");
