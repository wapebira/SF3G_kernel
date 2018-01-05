#include <linux/module.h>
#include <linux/usb/phy.h>
#include <linux/usb/phy-intel.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>

#define IS_BATTERY(psy) (psy->type == POWER_SUPPLY_TYPE_BATTERY)
#define CHRG_POWER_OFF_THRESD		3300

struct sofia_chrg_det {
	struct device *dev;
	struct usb_phy *usb_phy;
	struct notifier_block usb_nb;
	struct power_supply *bat_psy;
	struct power_supply psy;
	struct wake_lock suspend_lock;
	struct workqueue_struct *wq;
	struct delayed_work	usb_delay_work;

	int cur;
	int online;
	int otg_en_boost;
	int event;
};

static int chrg_get_online(enum power_supply_charger_event evt)
{
	int online = 0;

	switch (evt) {
	case POWER_SUPPLY_CHARGER_EVENT_CONNECT:
		online = 1;
		break;
	case POWER_SUPPLY_CHARGER_EVENT_DISCONNECT:
		online = 0;
		break;
	default:
		online = 0;
		break;
	}

	return online;
}

static struct power_supply *get_battery_supplied_by_list(void)
{
	struct class_dev_iter iter;
	struct device *dev;
	struct power_supply *pst;

	/* Identify chargers which are supplying power to the battery */
	class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
	while ((dev = class_dev_iter_next(&iter))) {
		pst = (struct power_supply *)dev_get_drvdata(dev);
		if (IS_BATTERY(pst))
			break;
	}
	class_dev_iter_exit(&iter);

	return pst;
}

static int chrg_set_battery_status_property(struct power_supply *psy_battery,
					    enum power_supply_property psp,
					    int prop_val)
{
	union power_supply_propval val;

	val.intval = prop_val;
	if (psy_battery->set_property)
		return psy_battery->set_property(psy_battery, psp, &val);

	return 0;
}

static void chrg_usb_notifier_delayed_work(struct work_struct *work)
{
	struct sofia_chrg_det *chrg_det = container_of(work,
			struct sofia_chrg_det, usb_delay_work.work);
	struct power_supply *battery_lst;
	int battery_status;

	switch (chrg_det->event) {
	case USB_EVENT_CHARGER:
		if (chrg_det->online && chrg_det->cur >= 500) {
			battery_status = POWER_SUPPLY_STATUS_CHARGING;
			battery_lst = get_battery_supplied_by_list();
			chrg_set_battery_status_property(battery_lst,
							 POWER_SUPPLY_PROP_STATUS,
							 battery_status);
			power_supply_changed(&chrg_det->psy);
		} else if (chrg_det->online == 0 && chrg_det->cur == 0) {
			battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			battery_lst = get_battery_supplied_by_list();
			chrg_set_battery_status_property(battery_lst,
							 POWER_SUPPLY_PROP_STATUS,
							 battery_status);
			power_supply_changed(&chrg_det->psy);
			wake_unlock(&chrg_det->suspend_lock);
		}
		break;
	case INTEL_USB_DRV_VBUS:
		power_supply_changed(&chrg_det->psy);
		break;
	default:
		break;
	}
}

static int chrg_usb_notifier_handler(struct notifier_block *nb,
				     unsigned long event, void *data)
{
	struct sofia_chrg_det *chrg_det;
	struct power_supply_cable_props *cable_props;
	struct power_supply *battery_lst;
	int battery_status;

	if (!data)
		return NOTIFY_BAD;
	chrg_det = container_of(nb, struct sofia_chrg_det, usb_nb);

	switch (event) {
	case USB_EVENT_CHARGER:
		cable_props = (struct power_supply_cable_props *)data;
		chrg_det->cur = cable_props->ma;
		chrg_det->event = USB_EVENT_CHARGER;

		if (chrg_det->cur >= 100 || chrg_det->cur == 0)
			chrg_det->online = chrg_get_online(cable_props->chrg_evt);

		if(chrg_det->online && chrg_det->cur >= 500){
			wake_lock(&chrg_det->suspend_lock);
		}
		queue_delayed_work(chrg_det->wq, &chrg_det->usb_delay_work,
				   msecs_to_jiffies(50));
		break;
	case INTEL_USB_DRV_VBUS:
		chrg_det->event = INTEL_USB_DRV_VBUS;
		chrg_det->otg_en_boost = *((bool *)data);
		queue_delayed_work(chrg_det->wq, &chrg_det->usb_delay_work,
				   msecs_to_jiffies(50));
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static int usb_charger_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct sofia_chrg_det *chrg_det;
	union power_supply_propval prop;
	struct device_node *np;
	struct power_supply *bat_psy = NULL;

	chrg_det = container_of(psy, struct sofia_chrg_det, psy);
	bat_psy = chrg_det->bat_psy;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (!chrg_det->bat_psy) {
			np = chrg_det->dev->of_node;
			chrg_det->bat_psy = power_supply_get_by_phandle(np,
									"sofia,fuel_gauge");
			bat_psy = chrg_det->bat_psy;
		}

		if (bat_psy) {
			bat_psy->get_property(bat_psy,
					      POWER_SUPPLY_PROP_VOLTAGE_NOW,
					      &prop);
			if (prop.intval < CHRG_POWER_OFF_THRESD)
				val->intval = 0;
			else
				val->intval = chrg_det->online;
		} else {
			val->intval = chrg_det->online;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chrg_det->cur;
		break;
	case POWER_SUPPLY_PROP_USB_OTG:
		val->intval = chrg_det->otg_en_boost;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property usb_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static char *chrg_supplied_to[] = {
	"battery",
};

static struct power_supply_throttle chrg_dummy_throttle_states[] = {
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
	},
};

static int sofia_chrg_det_probe(struct platform_device *pdev)
{
	struct sofia_chrg_det *chrg_det;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	chrg_det = devm_kzalloc(&pdev->dev, sizeof(*chrg_det), GFP_KERNEL);
	if (!chrg_det)
		return -ENOMEM;

	chrg_det->dev = &pdev->dev;

	chrg_det->usb_phy = usb_get_phy(USB_PHY_TYPE_USB2);
	if (IS_ERR_OR_NULL(chrg_det->usb_phy)) {
		dev_err(chrg_det->dev, "get usb phy failed\n");
		return PTR_ERR(chrg_det->usb_phy);
	}

	chrg_det->bat_psy = power_supply_get_by_phandle(np, "sofia,fuel_gauge");
	chrg_det->usb_nb.notifier_call = chrg_usb_notifier_handler;
	ret = usb_register_notifier(chrg_det->usb_phy, &chrg_det->usb_nb);
	if (ret) {
		dev_err(chrg_det->dev, "registr usb notification failed\n");
		return ret;
	}

		/* Set up the wake lock to prevent suspend when charging. */
	wake_lock_init(&chrg_det->suspend_lock,
			WAKE_LOCK_SUSPEND,
			"chrg_wake_lock");

	chrg_det->psy.name		= "chrg_det";
	chrg_det->psy.type		= POWER_SUPPLY_TYPE_USB;
	chrg_det->psy.properties	= usb_power_props;
	chrg_det->psy.num_properties	= ARRAY_SIZE(usb_power_props);
	chrg_det->psy.get_property	= usb_charger_get_property;
	chrg_det->psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
					POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	chrg_det->psy.supplied_to = chrg_supplied_to;
	chrg_det->psy.num_supplicants = ARRAY_SIZE(chrg_supplied_to);
	chrg_det->psy.throttle_states = chrg_dummy_throttle_states;
	chrg_det->psy.num_throttle_states =
				ARRAY_SIZE(chrg_dummy_throttle_states);
	chrg_det->wq = alloc_ordered_workqueue("%s", WQ_MEM_RECLAIM | WQ_FREEZABLE,
					       "chrg-work");
	INIT_DELAYED_WORK(&chrg_det->usb_delay_work,
			  chrg_usb_notifier_delayed_work);
	ret = power_supply_register(chrg_det->dev, &chrg_det->psy);
	if (ret)
		dev_err(chrg_det->dev, "register chrg power supply failed\n");

	dev_info(chrg_det->dev, "sofia charger detect driver probe\n");

	return ret;
}

static int sofia_chrg_det_remove(struct platform_device *pdev)
{
	struct sofia_chrg_det *chrg_det;
	chrg_det = devm_kzalloc(&pdev->dev, sizeof(*chrg_det), GFP_KERNEL);
	wake_lock_destroy(&chrg_det->suspend_lock);
	cancel_delayed_work_sync(&chrg_det->usb_delay_work);
	return 0;
}

static const struct of_device_id sofia_chrg_det_of_ids[] = {
		{.compatible = "sofia,chrg_det",},
		{},
};

static struct platform_driver sofia_chrg_det_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "sofia_chrg_det",
		.pm = NULL,
		.of_match_table = sofia_chrg_det_of_ids,
	},
	.probe  = sofia_chrg_det_probe,
	.remove = sofia_chrg_det_remove,
};

module_platform_driver(sofia_chrg_det_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("yxj <yxj@rock-chips.com>");
MODULE_DESCRIPTION("charger detect driver");

