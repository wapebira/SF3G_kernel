#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb/phy.h>
#include <linux/usb/phy-intel.h>
#include <linux/of_irq.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_interface.h>

#define CHRG_CTL		0x0A00
#define CHRG_STAT		0x0A04
#define CHRG_CTL_WR		0x0A08
#define C0_FS			0x1038

#define CDETIO			(0x3 << 29)
#define CDETDT			(0xf << 12)
#define CDETSRC			(7 << 7)
#define CHGDET			(2 << 7)

#define CDETUP			BIT(31)
#define CDETDIR			BIT(11)
#define CDETSENS		BIT(10)
#define CDETLV			BIT(6)
#define CDETEN			BIT(5)
#define WR_WS			BIT(0)
#define CDET			BIT(8)

#define CHECK_CHGDET_DELAY	0

enum {
	VBUS_OFF = 0,
	VBUS_ON,
};

struct sofia_vbus_det {
	struct device *dev;
	struct idi_peripheral_device *idev;
	struct work_struct chgdet_work;
	struct delayed_work check_chgdet_status_work;
	void __iomem *base;
	struct usb_phy *usb_phy;
	int vbus_irq;
	int vbus;
};

static void ag620_chgdet_work(struct work_struct *work)
{
	int ret;
	u32 chg_ctl_reg, c0_fs_reg;
	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	struct sofia_vbus_det *vbus_det = container_of(work,
		struct sofia_vbus_det, chgdet_work);

	pm_state_en =
		idi_peripheral_device_pm_get_state_handler(
						vbus_det->idev, "enable");
	if (pm_state_en == NULL) {
		pr_err("Unable to get handler for PM state 'enable'!\n");
		return;
	}

	pm_state_dis =
		idi_peripheral_device_pm_get_state_handler(
						vbus_det->idev, "disable");
	if (pm_state_dis == NULL) {
		pr_err("Unable to get handler for PM state 'disable'!\n");
		return;
	}

	ret = idi_set_power_state(vbus_det->idev, pm_state_en, true);

	if (ret) {
		pr_err("setting PM state '%s' failed!\n", pm_state_en->name);
		return;
	}

	chg_ctl_reg = ioread32(vbus_det->base + CHRG_CTL);
	c0_fs_reg = ioread32(vbus_det->base + C0_FS);

	dev_info(vbus_det->dev, "chg_ctl_reg = 0x%08x, c0_fs_reg = 0x%08x\n",
		 chg_ctl_reg, c0_fs_reg);

	if (c0_fs_reg & CDET) {
		vbus_det->vbus = VBUS_OFF;
		chg_ctl_reg &= ~CDETDIR;
	} else {
		vbus_det->vbus = VBUS_ON;
		chg_ctl_reg |= CDETDIR;
	}

	iowrite32(chg_ctl_reg, vbus_det->base + CHRG_CTL);
	iowrite32(WR_WS, vbus_det->base + CHRG_CTL_WR);
	iowrite32(WR_WS, vbus_det->base + CHRG_CTL_WR);

	ret = idi_set_power_state(vbus_det->idev, pm_state_dis, false);

	if (ret) {
		pr_err("setting PM state '%s' failed!\n", pm_state_dis->name);
		return;
	}

	atomic_notifier_call_chain(&vbus_det->usb_phy->notifier,
				   USB_EVENT_VBUS, &vbus_det->vbus);
}

static void ag620_check_chgdet_status_work(struct work_struct *work)
{
	int ret;
	u32 chg_ctl_reg, c0_fs_reg;
	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	struct sofia_vbus_det *vbus_det = container_of(work,
		struct sofia_vbus_det, check_chgdet_status_work.work);

	pm_state_en =
		idi_peripheral_device_pm_get_state_handler(
						vbus_det->idev, "enable");
	if (pm_state_en == NULL) {
		pr_err("Unable to get handler for PM state 'enable'!\n");
		return;
	}

	pm_state_dis =
		idi_peripheral_device_pm_get_state_handler(
						vbus_det->idev, "disable");
	if (pm_state_dis == NULL) {
		pr_err("Unable to get handler for PM state 'disable'!\n");
		return;
	}

	ret = idi_set_power_state(vbus_det->idev, pm_state_en, true);

	if (ret) {
		pr_err("setting PM state '%s' failed!\n", pm_state_en->name);
		return;
	}

	c0_fs_reg = ioread32(vbus_det->base + C0_FS);
	chg_ctl_reg = ioread32(vbus_det->base + CHRG_CTL);

	dev_info(vbus_det->dev, "chg_ctl_reg_1 = 0x%08x, c0_fs_reg_1 = 0x%08x\n",
		 chg_ctl_reg, c0_fs_reg);

	if (c0_fs_reg & CDET) {
		iowrite32(chg_ctl_reg & ~CDETDIR, vbus_det->base + CHRG_CTL);
		iowrite32(WR_WS, vbus_det->base + CHRG_CTL_WR);
		iowrite32(WR_WS, vbus_det->base + CHRG_CTL_WR);
		vbus_det->vbus = VBUS_OFF;
		atomic_notifier_call_chain(&vbus_det->usb_phy->notifier,
					   USB_EVENT_VBUS, &vbus_det->vbus);
	} else if (!(c0_fs_reg & CDET) &&
		   (vbus_det->vbus == VBUS_OFF)) {
		iowrite32(chg_ctl_reg | CDETDIR, vbus_det->base + CHRG_CTL);
		iowrite32(WR_WS, vbus_det->base + CHRG_CTL_WR);
		iowrite32(WR_WS, vbus_det->base + CHRG_CTL_WR);

		vbus_det->vbus = VBUS_ON;
		atomic_notifier_call_chain(&vbus_det->usb_phy->notifier,
					   USB_EVENT_VBUS, &vbus_det->vbus);
	}

	ret = idi_set_power_state(vbus_det->idev, pm_state_dis, false);

	if (ret)
		pr_err("setting PM state '%s' failed!\n", pm_state_dis->name);
}

static irqreturn_t vbus_irq_isr(int irq, void *dev)
{
	struct sofia_vbus_det *vbus_det = (struct sofia_vbus_det *)dev;

	schedule_work(&vbus_det->chgdet_work);

	return IRQ_HANDLED;
}

static int sofia_vbus_cfg_pmu_regs(struct sofia_vbus_det *vbus_det)
{
	u32 regval;
	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	int ret;

	if (!vbus_det->base || !vbus_det->idev)
		return -EINVAL;

	pm_state_en =
		idi_peripheral_device_pm_get_state_handler(
						vbus_det->idev, "enable");
	if (pm_state_en == NULL) {
		pr_err("Unable to get handler for PM state 'enable'!\n");
		return -EINVAL;
	}

	pm_state_dis =
		idi_peripheral_device_pm_get_state_handler(
						vbus_det->idev, "disable");
	if (pm_state_dis == NULL) {
		pr_err("Unable to get handler for PM state 'disable'!\n");
		return -EINVAL;
	}

	pr_info("Getting PM state handlers: OK\n");

	ret = idi_set_power_state(vbus_det->idev, pm_state_en, true);

	if (ret) {
		pr_err("setting PM state '%s' failed!\n", pm_state_en->name);
		return -EIO;
	}

	regval = ioread32(vbus_det->base + CHRG_CTL);
	regval &= ~CDETIO;
	regval &= ~CDETDIR;
	regval |= CDETSENS;
	regval &= ~(CDETSRC);
	regval |= CHGDET;
	regval &= ~CDETLV;
	regval |= CDETEN;
	iowrite32(regval, vbus_det->base + CHRG_CTL);
	/* need to write twice */
	iowrite32(WR_WS, vbus_det->base + CHRG_CTL_WR);
	iowrite32(WR_WS, vbus_det->base + CHRG_CTL_WR);
	ret = idi_set_power_state(vbus_det->idev, pm_state_dis, false);

	if (ret)
		pr_err("setting PM state '%s' failed!\n", pm_state_dis->name);

	return 0;
}

static int sofia_vbus_det_probe(struct idi_peripheral_device *idev,
				const struct idi_device_id *id)
{
	struct sofia_vbus_det *vbus_det;
	struct resource *res;
	int ret;

	vbus_det = devm_kzalloc(&idev->device, sizeof(*vbus_det), GFP_KERNEL);
	if (!vbus_det)
		return -ENOMEM;

	vbus_det->dev = &idev->device;

	vbus_det->usb_phy = usb_get_phy(USB_PHY_TYPE_USB2);
	if (IS_ERR_OR_NULL(vbus_det->usb_phy)) {
		dev_err(vbus_det->dev, "get usb phy failed\n");
		return PTR_ERR(vbus_det->usb_phy);
	}

	res = idi_get_resource_byname(&idev->resources, IORESOURCE_MEM,
				      "registers");
	if (!res) {
		dev_err(vbus_det->dev, "get PMU's Charger registers resources failed!\n");
		return -EINVAL;
	}

	vbus_det->base = devm_ioremap_resource(&idev->device, res);
	if (!vbus_det->base) {
		dev_err(vbus_det->dev, "mapping PMU's Charger registers failed!\n");
		return -EINVAL;
	}

	vbus_det->idev = idev;

	ret = idi_device_pm_set_class(idev);
	if (ret) {
		dev_err(vbus_det->dev, "Unable to register for generic pm class\n");
		return ret;
	}

	sofia_vbus_cfg_pmu_regs(vbus_det);
	vbus_det->vbus_irq = irq_of_parse_and_map(idev->device.of_node, 0);
	if (!vbus_det->vbus_irq) {
		dev_err(vbus_det->dev, "failed to map vbus irq\n");
		return -ENXIO;
	}

	INIT_WORK(&vbus_det->chgdet_work, ag620_chgdet_work);
	INIT_DELAYED_WORK(&vbus_det->check_chgdet_status_work,
			  ag620_check_chgdet_status_work);

	ret = request_irq(vbus_det->vbus_irq, vbus_irq_isr,
			  IRQF_NO_SUSPEND, "vbus_irq", vbus_det);
	if (ret != 0) {
		pr_err("Failed to register @PMU for GHGINT irq! ret=%d", ret);
		return ret;
	}

	schedule_delayed_work(&vbus_det->check_chgdet_status_work,
			      CHECK_CHGDET_DELAY);

	dev_set_drvdata(&idev->device, vbus_det);

	dev_info(vbus_det->dev, "sofia charger detect driver probe\n");

	return ret;
}

static int sofia_vbus_det_remove(struct idi_peripheral_device *ididev)
{
	return 0;
}

static void sofia_vbus_det_shutdown(struct idi_peripheral_device *ididev)
{
	u32 regval;
	int ret;
	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	struct sofia_vbus_det *vbus_det;

	vbus_det = dev_get_drvdata(&ididev->device);

	if (!vbus_det->base || !vbus_det->idev)
		return;

	free_irq(vbus_det->vbus_irq, vbus_det);

	pm_state_en =
		idi_peripheral_device_pm_get_state_handler(
						vbus_det->idev, "enable");
	if (pm_state_en == NULL) {
		pr_err("Unable to get handler for PM state 'enable'!\n");
		return;
	}

	pm_state_dis =
		idi_peripheral_device_pm_get_state_handler(
						vbus_det->idev, "disable");
	if (pm_state_dis == NULL) {
		pr_err("Unable to get handler for PM state 'disable'!\n");
		return;
	}

	pr_info("Getting PM state handlers: OK\n");

	ret = idi_set_power_state(vbus_det->idev, pm_state_en, true);

	if (ret) {
		pr_err("setting PM state '%s' failed!\n", pm_state_en->name);
		return;
	}

	regval = ioread32(vbus_det->base + CHRG_CTL);
	regval &= ~CDETSENS;
	iowrite32(regval, vbus_det->base + CHRG_CTL);
	/* need to write twice */
	iowrite32(WR_WS, vbus_det->base + CHRG_CTL_WR);
	iowrite32(WR_WS, vbus_det->base + CHRG_CTL_WR);
	ret = idi_set_power_state(vbus_det->idev, pm_state_dis, false);

	if (ret)
		pr_err("setting PM state '%s' failed!\n", pm_state_dis->name);

	return;
}

static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_CHG,
	},

	{ /* end: all zeroes */},
};

static struct idi_peripheral_driver ag620_vbus_idi_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "ag620-vbus-det",
	},
	.p_type = IDI_CHG,
	.id_table = idi_ids,
	.probe  = sofia_vbus_det_probe,
	.remove = sofia_vbus_det_remove,
	.shutdown = sofia_vbus_det_shutdown,
};

static int sofia_vdet_probe(struct platform_device *pdev)
{
	return idi_register_peripheral_driver(&ag620_vbus_idi_driver);
}

static struct of_device_id sofia_vdet_of_match[] = {
	{ .compatible = "intel,ag620-vbus-det", },
	{ },
};

static struct platform_driver sofia_vdet_drv = {
	.driver = {
		.name = "ag620-vbus-det",
		.owner = THIS_MODULE,
		.of_match_table = sofia_vdet_of_match,
	},
	.probe = sofia_vdet_probe,
};

static int __init sofia_vdet_init(void)
{
	return platform_driver_register(&sofia_vdet_drv);
}

late_initcall(sofia_vdet_init);

static void __exit sofia_vdet_exit(void)
{
	platform_driver_unregister(&sofia_vdet_drv);
}
module_exit(sofia_vdet_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("yxj <yxj@rock-chips.com>");
MODULE_DESCRIPTION("ag620 usb detect driver");
MODULE_DEVICE_TABLE(of, sofia_vdet_of_match);
