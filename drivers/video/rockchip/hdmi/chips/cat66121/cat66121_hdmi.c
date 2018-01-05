/*
 * Copyright (C) 2012-2015 Rockchip Electronics Co., Ltd.
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
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#if defined(CONFIG_DEBUG_FS)
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif
#include <linux/of_gpio.h>
#include <linux/of_device.h>

#include "cat66121_hdmi.h"
#include "cat66121_hdmi_hw.h"

#define HDMI_POLL_MDELAY	50/* 100 */
#define INVALID_GPIO -1
struct cat66121_hdmi_pdata *cat66121_hdmi = NULL;
struct hdmi *hdmi = NULL;

static void cat66121_irq_work_func(struct work_struct *work);

static int rk_hdmi_fb_event_notify(struct notifier_block *self,
				   unsigned long action, void *data)
{
	struct fb_event *event = data;
	int blank_mode = *((int *)event->data);

	if (!hdmi)
		return NOTIFY_OK;

	if (action == FB_EARLY_EVENT_BLANK) {
		switch (blank_mode) {
		case FB_BLANK_UNBLANK:
			break;
		default:
			hdmi_dbg(hdmi->dev, "hdmi enter early suspend pwr %d state %d\n",
				 hdmi->pwr_mode, hdmi->state);
			flush_delayed_work(&hdmi->delay_work);
			mutex_lock(&hdmi->enable_mutex);
			hdmi->suspend = 1;
			if (!hdmi->enable) {
				mutex_unlock(&hdmi->enable_mutex);
				return 0;
			}

			if (hdmi->irq != INVALID_GPIO)
				disable_irq(hdmi->irq);

			mutex_unlock(&hdmi->enable_mutex);
			hdmi->command = HDMI_CONFIG_ENABLE;
			init_completion(&hdmi->complete);
			hdmi->wait = 1;
			queue_delayed_work(hdmi->workqueue,
					   &hdmi->delay_work, 0);
			wait_for_completion_interruptible_timeout(&hdmi->complete,
							msecs_to_jiffies(5000));
			flush_delayed_work(&hdmi->delay_work);
			break;
		}
	} else if (action == FB_EVENT_BLANK) {
		switch (blank_mode) {
		case FB_BLANK_UNBLANK:
			hdmi_dbg(hdmi->dev, "hdmi exit early resume\n");
			mutex_lock(&hdmi->enable_mutex);

			hdmi->suspend = 0;
			if (hdmi->irq == INVALID_GPIO)
				queue_delayed_work(cat66121_hdmi->workqueue,
						   &cat66121_hdmi->delay_work,
						   HDMI_POLL_MDELAY);
			else if (hdmi->enable)
				enable_irq(hdmi->irq);

			queue_delayed_work(hdmi->workqueue, &hdmi->delay_work,
					   msecs_to_jiffies(10));
			mutex_unlock(&hdmi->enable_mutex);
			break;
		default:
			break;
		}
	}
	return NOTIFY_OK;
}

static struct notifier_block rk_hdmi_fb_notifier = {
	.notifier_call = rk_hdmi_fb_event_notify,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hdmi_early_suspend(struct early_suspend *h)
{
	hdmi_dbg(hdmi->dev, "hdmi enter early suspend pwr %d state %d\n",
		 hdmi->pwr_mode, hdmi->state);
	flush_delayed_work(&hdmi->delay_work);
	mutex_lock(&hdmi->enable_mutex);
	hdmi->suspend = 1;
	if (!hdmi->enable) {
		mutex_unlock(&hdmi->enable_mutex);
		return;
	}

	if (hdmi->irq != INVALID_GPIO)
		disable_irq(hdmi->irq);

	mutex_unlock(&hdmi->enable_mutex);
	hdmi->command = HDMI_CONFIG_ENABLE;
	init_completion(&hdmi->complete);
	hdmi->wait = 1;
	queue_delayed_work(hdmi->workqueue, &hdmi->delay_work, 0);
	wait_for_completion_interruptible_timeout(&hdmi->complete,
						  msecs_to_jiffies(5000));
	flush_delayed_work(&hdmi->delay_work);
}

static void hdmi_early_resume(struct early_suspend *h)
{
	hdmi_dbg(hdmi->dev, "hdmi exit early resume\n");
	mutex_lock(&hdmi->enable_mutex);

	hdmi->suspend = 0;
	if (hdmi->irq == INVALID_GPIO)
		queue_delayed_work(cat66121_hdmi->workqueue,
				   &cat66121_hdmi->delay_work,
				   HDMI_POLL_MDELAY);
	else if (hdmi->enable)
		enable_irq(hdmi->irq);

	queue_delayed_work(hdmi->workqueue, &hdmi->delay_work,
			   msecs_to_jiffies(10));
	mutex_unlock(&hdmi->enable_mutex);
}
#endif

static void cat66121_irq_work_func(struct work_struct *work)
{
	if (hdmi->suspend == 0) {
		if (hdmi->enable == 1) {
			cat66121_hdmi_interrupt(hdmi);
			if (hdmi->hdcp_irq_cb)
				hdmi->hdcp_irq_cb(0);
		}
		if (!gpio_is_valid(hdmi->irq))
			queue_delayed_work(cat66121_hdmi->workqueue,
					   &cat66121_hdmi->delay_work,
					   HDMI_POLL_MDELAY);
	}
}

static irqreturn_t cat66121_thread_interrupt(int irq, void *dev_id)
{
	cat66121_irq_work_func(NULL);
	msleep(HDMI_POLL_MDELAY);
	hdmi_dbg(hdmi->dev, "%s irq=%d\n", __func__, irq);

	return IRQ_HANDLED;
}

#define I2C_WRITE_FLAG (!I2C_M_RD)
#define I2C_READ_FLAG I2C_M_RD
#define ADDR_LENGTH	   2

#if defined(CONFIG_DEBUG_FS)
static int hdmi_read_p0_reg(struct i2c_client *client, char reg, char *val)
{
	struct i2c_msg msgs[2];
	int ret = 0;
	int retries = 0;
	char buf[2];

	memset(buf, 0xAA, 2);
	buf[0] = reg;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];
	/* msgs[0].scl_rate = 300 * 1000;	// for Rockchip, etc. */

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = 1;
	msgs[1].buf   = &buf[0];
	/* msgs[1].scl_rate = 300 * 1000; */

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}
	if (ret != 2) {
		dev_err(&client->dev, "I2c Fail ret=%d\n", ret);
		return ret;
	}

	*val = buf[0];

	return 0;
}

static int hdmi_write_p0_reg(struct i2c_client *client, char reg, char *val)
{
	struct i2c_msg msg;
	int ret = 0;
	int retries = 0;
	char buf[2];

	memset(buf, 0xAA, 2);
	/* msgs[1].scl_rate = 300 * 1000; */
	buf[0] = reg;
	buf[1] = *val;

	msg.addr	= client->addr;
	msg.flags = !I2C_M_RD;
	msg.len		= 2;
	msg.buf		= buf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}

	return 0;
}

static int hdmi_reg_show(struct seq_file *s, void *v)
{
	int i;
	unsigned char val;
	struct i2c_client *client = cat66121_hdmi->client;

	for (i = 0; i < 256; i++) {
		hdmi_read_p0_reg(client, i,  &val);
		if (i % 16 == 0)
			seq_printf(s, "\n>>>hdmi_hdmi %x:", i);
		seq_printf(s, " %02x", val);
	}

	return 0;
}

static ssize_t hdmi_reg_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	struct i2c_client *client = NULL;
	u32 reg, val;
	char kbuf[25];
	int ret;

	client = cat66121_hdmi->client;

	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;
	ret = sscanf(kbuf, "%x%x", &reg, &val);
	if (ret < 0)
		return -EINVAL;

	dev_info(&cat66121_hdmi->client->dev, "%s %d reg=%x val=%x\n",
		 __func__, __LINE__, reg, val);
	if (reg == 0xff) {
		hdmi->vic = val;
		if (!hdmi->hotplug)
			return 0;
		hdmi->command = HDMI_CONFIG_VIDEO;
		init_completion(&hdmi->complete);
		hdmi->wait = 1;
		queue_delayed_work(hdmi->workqueue, &hdmi->delay_work, 0);
		wait_for_completion_interruptible_timeout(&hdmi->complete,
							  msecs_to_jiffies
							  (10000));
	}
	hdmi_write_p0_reg(client, reg, (u8 *)&val);

	return count;
}

static int hdmi_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, hdmi_reg_show, hdmi);
}

static const struct file_operations hdmi_reg_fops = {
	.owner		= THIS_MODULE,
	.open		= hdmi_reg_open,
	.read		= seq_read,
	.write		  = hdmi_reg_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#endif

static int rk_hdmi_drv_init(struct hdmi *hdmi_drv)
{
	int ret = 0;

	hdmi_drv->vop = get_vop_drv("vop0");
	if (!hdmi_drv->vop) {
		dev_err(hdmi_drv->dev, "can not connect to video source vop\n");
		ret = -ENXIO;
		return ret;
	}

#ifdef SUPPORT_HDCP
	hdmi_drv->irq = INVALID_GPIO;
#endif

	hdmi_sys_init(hdmi_drv);
	hdmi_drv->xscale = 100;
	hdmi_drv->yscale = 100;
	hdmi_drv->insert = cat66121_hdmi_sys_insert;
	hdmi_drv->remove = cat66121_hdmi_sys_remove;
	hdmi_drv->control_output = cat66121_hdmi_sys_enalbe_output;
	hdmi_drv->config_video = cat66121_hdmi_sys_config_video;
	hdmi_drv->config_audio = cat66121_hdmi_sys_config_audio;
	hdmi_drv->detect_hotplug = cat66121_hdmi_sys_detect_hpd;
	hdmi_drv->read_edid = cat66121_hdmi_sys_read_edid;

	#ifdef CONFIG_HAS_EARLYSUSPEND
	hdmi_drv->early_suspend.suspend = hdmi_early_suspend;
	hdmi_drv->early_suspend.resume = hdmi_early_resume;
	hdmi_drv->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 10;
	register_early_suspend(&hdmi_drv->early_suspend);
	#endif

	hdmi_register_display_sysfs(hdmi_drv, NULL);

	#ifdef CONFIG_SWITCH
	hdmi_drv->switch_hdmi.name = "hdmi";
	switch_dev_register(&hdmi_drv->switch_hdmi);
	#endif

	spin_lock_init(&hdmi_drv->irq_lock);
	mutex_init(&hdmi_drv->enable_mutex);
	hdmi_drv_register(hdmi_drv);

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id cat66121_dt_ids[] = {
	{.compatible = "ite,cat66121", },
	{}
};

MODULE_DEVICE_TABLE(of, cat66121_dt_ids);
#endif

static int cat66121_hdmi_i2c_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int rc = 0;
	int ret;

	dev_info(&client->dev, "%s, line=%d\n", __func__, __LINE__);

	if (client->dev.of_node) {
		if (!of_match_device(cat66121_dt_ids, &client->dev)) {
			dev_err(&client->dev,
				"Failed to find matching dt id\n");
			return -EINVAL;
		}
	}

	cat66121_hdmi = devm_kzalloc(&client->dev, sizeof(*cat66121_hdmi),
				     GFP_KERNEL);
	if (!cat66121_hdmi)
		return -ENOMEM;
	cat66121_hdmi->client = client;
	i2c_set_clientdata(client, cat66121_hdmi);

	hdmi = devm_kmalloc(&client->dev, sizeof(struct hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;
	memset(hdmi, 0, sizeof(struct hdmi));
	hdmi->dev = &client->dev;

	hdmi->pins = devm_kzalloc(hdmi->dev, sizeof(*hdmi->pins),
				  GFP_KERNEL);
	if (!hdmi->pins)
		return -ENOMEM;

	hdmi->pins->p = devm_pinctrl_get(&client->dev);
	if (IS_ERR(hdmi->pins->p)) {
		dev_err(hdmi->dev, "Can not get pinctrl.\n");
		return IS_ERR(hdmi->pins->p);
	}

	hdmi->pins->default_state = pinctrl_lookup_state(
		hdmi->pins->p, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(hdmi->pins->default_state))
		dev_err(hdmi->dev, "could not get default pinctrl state\n");
	hdmi->pins->sleep_state = pinctrl_lookup_state(
		hdmi->pins->p, PINCTRL_STATE_SLEEP);
	if (IS_ERR(hdmi->pins->sleep_state))
		dev_err(hdmi->dev, "could not get sleep pinctrl state\n");

	hdmi->dif = pinctrl_lookup_state(hdmi->pins->p, "dif");
	if (IS_ERR(hdmi->dif))
		dev_err(hdmi->dev, "could not get dif pinctrl state\n");

	ret = pinctrl_select_state(hdmi->pins->p, hdmi->dif);
	if (ret < 0) {
		dev_err(hdmi->dev, "Error while setting PIN dif state\n");
		goto err_request_vop;
	}

	ret = pinctrl_select_state(hdmi->pins->p, hdmi->pins->default_state);
	if (ret < 0) {
		dev_err(hdmi->dev, "Error while setting PIN default state\n");
		goto err_request_vop;
	}

	if (cat66121_detect_device() != 1) {
		dev_err(hdmi->dev, "can't find it66121 device.\n");
		rc = -ENXIO;
		goto err_request_vop;
	}

	cat66121_hdmi->plug_status = -1;
	rk_hdmi_drv_init(hdmi);
	cat66121_hdmi_sys_init(hdmi);

	hdmi->workqueue = create_singlethread_workqueue("hdmi");
	INIT_DELAYED_WORK(&hdmi->delay_work, hdmi_work);
	hdmi->irq = INVALID_GPIO;

	if (gpio_is_valid(hdmi->irq)) {
		rc = gpio_request(hdmi->irq, "hdmi gpio");
		if (rc < 0) {
			dev_err(&client->dev,
				"fail to request gpio %d\n", hdmi->irq);
			goto err_request_vop;
		}

		cat66121_hdmi->gpio = hdmi->irq;
		gpio_direction_input(hdmi->irq);
		hdmi->irq = gpio_to_irq(hdmi->irq);
		if (hdmi->irq <= 0) {
			dev_err(hdmi->dev,
				"failed to get hdmi irq resource (%d).\n",
				hdmi->irq);
			goto err_request_irq;
		}
		rc = request_threaded_irq(hdmi->irq, NULL ,
					  cat66121_thread_interrupt,
					  IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					  dev_name(&client->dev), hdmi);
		if (rc < 0)  {
			dev_err(&client->dev, "fail to request hdmi irq\n");
			goto err_request_irq;
		}
	} else {
		cat66121_hdmi->workqueue =
			create_singlethread_workqueue("cat66121 irq");
		INIT_DELAYED_WORK(&cat66121_hdmi->delay_work,
				  cat66121_irq_work_func);
		cat66121_irq_work_func(NULL);
	}

#if defined(CONFIG_DEBUG_FS)
	{
		struct dentry *debugfs_dir =
			debugfs_create_dir("it66121", NULL);

		if (IS_ERR(debugfs_dir))
			dev_err(&client->dev,
				"failed to create debugfs dir for it66121!\n");
		else
			debugfs_create_file("hdmi", S_IRUSR, debugfs_dir,
					    hdmi, &hdmi_reg_fops);
	}
#endif

	fb_register_client(&rk_hdmi_fb_notifier);

	dev_info(&client->dev, "cat66121 hdmi i2c probe ok\n");

	return 0;

err_request_irq:
	gpio_free(hdmi->irq);
err_request_vop:
	dev_err(&client->dev, "cat66121 hdmi probe error\n");
	return rc;
}

static int cat66121_hdmi_i2c_remove(struct i2c_client *client)
{
	hdmi_dbg(hdmi->dev, "%s\n", __func__);
	if (hdmi) {
		mutex_lock(&hdmi->enable_mutex);
		if (!hdmi->suspend && hdmi->enable && hdmi->irq)
			disable_irq(hdmi->irq);
		mutex_unlock(&hdmi->enable_mutex);
		if (hdmi->irq)
			free_irq(hdmi->irq, NULL);
		flush_workqueue(hdmi->workqueue);
		destroy_workqueue(hdmi->workqueue);
		#ifdef CONFIG_SWITCH
		switch_dev_unregister(&hdmi->switch_hdmi);
		#endif
		hdmi_unregister_display_sysfs(hdmi);
		#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&hdmi->early_suspend);
		#endif
		fb_destroy_modelist(&hdmi->edid.modelist);
		kfree(hdmi->edid.audio);
		if (hdmi->edid.specs)
			kfree(hdmi->edid.specs->modedb);
		kfree(hdmi->edid.specs);
		kfree(hdmi);
		hdmi = NULL;
	}
	return 0;
}

static void cat66121_hdmi_i2c_shutdown(struct i2c_client *client)
{
	if (hdmi) {
		#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&hdmi->early_suspend);
		#endif
	}
	dev_info(&client->dev, "hdmi shut down.\n");
}

static const struct i2c_device_id cat66121_hdmi_id[] = {
	{ "cat66121_hdmi", 0 },
	{ }
};

struct i2c_driver cat66121_hdmi_i2c_driver = {
	.driver = {
		.name  = "cat66121_hdmi",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cat66121_dt_ids),
	},
	.probe	  = cat66121_hdmi_i2c_probe,
	.remove	 = cat66121_hdmi_i2c_remove,
	.shutdown	= cat66121_hdmi_i2c_shutdown,
	.id_table	= cat66121_hdmi_id,
};
