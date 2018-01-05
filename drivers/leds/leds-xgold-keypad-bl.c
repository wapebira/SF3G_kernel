/*
* Copyright (C) 2013 Intel Mobile Communications GmbH
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define TRUE 1
#define FALSE 0

#define LED_CURSINK	0x0

/* Scalling of intensity */
#define DRIVER_MAX_INTENSITY    20
#define ANDROID_MAX_INTENSITY   255
#define SCALING_INTENSITY(val) \
		((DRIVER_MAX_INTENSITY * val)/ANDROID_MAX_INTENSITY)

/* The ON and OFF values LED BL*/

#define SCU_KPD_LED_UP		0x101
#define SCU_KPD_LED_DOWN	0x100

#define XGOLD_LED_KPD_MODULE_NAME "leds-xgold-keypad-bl"

struct xgold_led_kpd_device {
	int (*init)(struct device *dev);
	void (*exit)(void);
	int (*set_clk)(struct device *dev, bool on);
	int (*set_keypad_backlight)(struct device *dev, void *mmio_base,
					 spinlock_t *lock, int intensity);
	struct platform_device *pdev;
	void __iomem *mmio_base;
	struct led_classdev key_bl_cdev;
	struct work_struct work;
	enum led_brightness key_bl_brightness;
	spinlock_t lock;
};

static void kpd_bl_write32(void *mmio_base, u16 offset, u32 val)
{
	iowrite32(val, (char *)mmio_base + offset);
}

static int xgold_kpd_set_clk(struct device *dev,
					bool on)
{
	return 0;
}

static int xgold_set_keypad_backlight(struct device *dev,
			void *mmio_base, spinlock_t *lock,
			int intensity)
{
	int val ;
	unsigned long flags;

	intensity = SCALING_INTENSITY(intensity);
    pr_info("%s, intensity:%d. mmio_base=0x%08p\n", __func__, intensity, mmio_base);
	if (intensity) {
		val = (SCU_KPD_LED_UP | intensity << 1);
		spin_lock_irqsave(lock, flags);
		kpd_bl_write32(mmio_base, LED_CURSINK, val);
		spin_unlock_irqrestore(lock, flags);
	} else {
		spin_lock_irqsave(lock, flags);
		kpd_bl_write32(mmio_base, LED_CURSINK, SCU_KPD_LED_DOWN);
		spin_unlock_irqrestore(lock, flags);
	}
	return 0;
}

static int xgold_kpd_bl_init(struct device *dev)
{
	return 0;
}

static void xgold_kpd_bl_exit(void)
{
	return;
}
static void xgold_led_kpd_work(struct work_struct *work)
{
	struct xgold_led_kpd_device *kpd_bl =
			container_of(work, struct xgold_led_kpd_device, work);
	static int power_on = FALSE;
	int ret = 0;

	if (kpd_bl->key_bl_brightness) {
		if (power_on == FALSE) {
			if (kpd_bl->set_clk) {
				ret = kpd_bl->set_clk(
					kpd_bl->key_bl_cdev.dev,
							TRUE);
				if (ret) {
					dev_err(kpd_bl->key_bl_cdev.dev,
					 "%s Set Clk Failed\n", __func__);
					return;
				}
			}
			power_on = TRUE;
		}
		if (kpd_bl->set_keypad_backlight) {
			ret = kpd_bl->set_keypad_backlight(
						kpd_bl->key_bl_cdev.dev,
						kpd_bl->mmio_base,
						&kpd_bl->lock,
						kpd_bl->key_bl_brightness);
			if (ret) {
				dev_err(kpd_bl->key_bl_cdev.dev,
				"%s Set key_bl_cdev Failed\n", __func__);
				return;
			}
		}
	} else {
		if (kpd_bl->set_keypad_backlight) {
			ret = kpd_bl->set_keypad_backlight(
						kpd_bl->key_bl_cdev.dev,
						kpd_bl->mmio_base,
						&kpd_bl->lock,
						kpd_bl->key_bl_brightness);
			if (ret) {
				dev_err(kpd_bl->key_bl_cdev.dev,
				"%s Set key_bl_cdev Failed\n", __func__);
				return;
			}
		}
		if (power_on == TRUE) {
			if (kpd_bl->set_clk) {
				ret = kpd_bl->set_clk(
						kpd_bl->key_bl_cdev.dev,
						FALSE);
				if (ret) {
					dev_err(kpd_bl->key_bl_cdev.dev,
					"%s Set Clk Failed\n", __func__);
					return;
				}
			}
			power_on = FALSE;
		}
	}
}

static void xgold_key_brightness_set(struct led_classdev *led_cdev,
					enum led_brightness brightness)
{
	struct xgold_led_kpd_device *kpd_bl =
	 container_of(led_cdev, struct xgold_led_kpd_device, key_bl_cdev);
	unsigned long flags;
    pr_info("%s, brightness:%d.\n", __func__, brightness);

	spin_lock_irqsave(&kpd_bl->lock, flags);
	kpd_bl->key_bl_brightness = brightness;
	schedule_work(&kpd_bl->work);
	spin_unlock_irqrestore(&kpd_bl->lock, flags);
	return ;
}

static int xgold_led_kpd_probe(struct platform_device *pdev)
{
	int ret = 0;
    unsigned int init_level = 0;
	struct xgold_led_kpd_device *kpd_bl;
	struct device_node *nbl;

    pr_info("%s 11.\n", __func__);
	kpd_bl = kzalloc(sizeof(struct xgold_led_kpd_device), GFP_KERNEL);
	if (!kpd_bl) {
		dev_err(&pdev->dev,
			"not enough memory for driver data\n");
		return -ENOMEM;
	}

    pr_info("%s 12.\n", __func__);
	nbl = pdev->dev.of_node;

	kpd_bl->init = xgold_kpd_bl_init;
	kpd_bl->exit = xgold_kpd_bl_exit;
	kpd_bl->set_clk = xgold_kpd_set_clk;
	kpd_bl->set_keypad_backlight= xgold_set_keypad_backlight;
	kpd_bl->mmio_base = of_iomap(nbl, 0);
	if (kpd_bl->mmio_base == NULL) {
		dev_err(&pdev->dev,
			" I/O remap failed\n");
		ret = -EINVAL;
		goto failed_free_mem;
	}
    pr_info("%s 13.\n", __func__);

	of_property_read_u32(nbl, "default-brightness-level", &init_level);

	kpd_bl->pdev = pdev;
	kpd_bl->key_bl_brightness = LED_OFF;
	INIT_WORK(&kpd_bl->work, xgold_led_kpd_work);
	spin_lock_init(&kpd_bl->lock);

	kpd_bl->key_bl_cdev.name = "keyboard-backlight";
	kpd_bl->key_bl_cdev.brightness = LED_HALF;
	kpd_bl->key_bl_cdev.brightness_set = xgold_key_brightness_set;
	ret = led_classdev_register(&pdev->dev, &kpd_bl->key_bl_cdev);
	if (ret != 0) {
		dev_err(&pdev->dev,
			" unable to register with Leds class\n");
		ret = -EINVAL;
		goto failed_unmap;
	}

    pr_info("%s 14.\n", __func__);
	if (kpd_bl->init) {
		ret = kpd_bl->init(kpd_bl->key_bl_cdev.dev);
		if (ret < 0) {
			dev_err(&pdev->dev, "Plat Init Failed: %s\n",
					kpd_bl->key_bl_cdev.name);
			ret = -EINVAL;
			goto failed_unregister_led_class;
		}
        kpd_bl->set_keypad_backlight(&pdev->dev, kpd_bl->mmio_base, &kpd_bl->lock, init_level);
	}
    pr_info("%s 15.\n", __func__);
	dev_set_drvdata(&pdev->dev, kpd_bl);
	return 0;

failed_unregister_led_class:
	led_classdev_unregister(&kpd_bl->key_bl_cdev);
failed_unmap:
	iounmap(kpd_bl->mmio_base);
failed_free_mem:
	kfree(kpd_bl);

	return ret;
}

static int xgold_led_kpd_remove(struct platform_device *pdev)
{
	struct xgold_led_kpd_device *kpd_bl = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	cancel_work_sync(&kpd_bl->work);
	led_classdev_unregister(&kpd_bl->key_bl_cdev);
	kfree(kpd_bl);
	return 0;
}


static struct of_device_id xgold_led_kbd_of_match[] = {
	{.compatible = "intel,keypad-bl",},
	{},
};

static struct platform_driver xgold_led_kpd_driver = {
	.driver		= {
		.name	= XGOLD_LED_KPD_MODULE_NAME,
		.owner	= THIS_MODULE,
        .of_match_table = xgold_led_kbd_of_match,
	},
	.probe		= xgold_led_kpd_probe,
	.remove		= xgold_led_kpd_remove,
};

static int xgold_led_kpd_init(void)
{
	pr_info("xgold keypad bl init\n");
	return platform_driver_register(&xgold_led_kpd_driver);
}

static void xgold_led_kpd__exit(void)
{
	pr_info("xgold keypad bl exit\n");
	platform_driver_unregister(&xgold_led_kpd_driver);

}

module_init(xgold_led_kpd_init);
module_exit(xgold_led_kpd__exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("xgold keypad bl Driver");
MODULE_DEVICE_TABLE(of, xgold_led_kbd_of_match);
