/**
 * @file mali_platform.c
 * Platform specific Mali driver functions for a default platform
 */
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/gfp.h>
#include <linux/fs.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/regulator/driver.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/cpufreq.h>
#include <linux/of.h>

#include "mali_kernel_common.h"
#include "mali_osk.h"
#include "arm_core_scaling.h"
#include "mali_platform.h"

static int mali_core_scaling_enable;

u32 mali_group_error;

struct device *mali_dev;

static ssize_t show_dvfs_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", mali_dvfs_is_enabled(dev));
}

static ssize_t set_dvfs_enable(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	unsigned long enable;
	ssize_t ret;

	ret = kstrtoul(buf, 0, &enable);
	if (ret)
		return ret;

	if (enable == 1)
		mali_dvfs_enable(dev);
	else if (enable == 0)
		mali_dvfs_disable(dev);
	else
		return -EINVAL;

	return count;
}

static ssize_t show_utilisation(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", mali_dvfs_utilisation(dev));
}

static int error_count_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", mali_group_error);
}

static ssize_t set_pm_states(struct device *dev,
			     struct device_attribute *attr, const char *buf,
			     size_t count)
{
	struct mali_platform_drv_data *drv_data = dev_get_drvdata(dev);
	unsigned long set_state;
	ssize_t ret;

	ret = kstrtoul(buf, 0, &set_state);
	if (ret)
		return ret;

	if (set_state > GPU_MAX_PM_STATE)
		return -EINVAL;

	dev_info(dev, "mali setting power state: %ld[%s]\n", set_state,
		 drv_data->pm_states[set_state]->name);

	ret = device_state_pm_set_state(dev, drv_data->pm_states[set_state]);
	if (ret) {
		dev_err(dev, "mali sys set_pm_states error %d\n", ret);
		return ret;
	}
	drv_data->curr_pm_state = set_state;

	return count;
}

static int read_pm_states(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct mali_platform_drv_data *drv_data = dev_get_drvdata(dev);

	return sprintf(buf, "mali current power state: %d[%s]\n",
		       drv_data->curr_pm_state,
		       drv_data->pm_states[drv_data->curr_pm_state]->name);
}
static ssize_t set_pm_max_states(struct device *dev,
			     struct device_attribute *attr, const char *buf,
			     size_t count)
{
	struct mali_platform_drv_data *drv_data = dev_get_drvdata(dev);
	unsigned long set_state;
	ssize_t ret;

	ret = kstrtoul(buf, 0, &set_state);
	if (ret)
		return ret;

	if (set_state > GPU_MAX_PM_STATE)
		return -EINVAL;

	if (set_state == 0)
		return -EINVAL;

	drv_data->max_pm_state = set_state;
	drv_data->resume_pm_state = set_state;

	/*
	 * If GPU dvfs disabled, I don't think need to change the GPU clk frequency,
	 * even thought GPU thermal control start to work, just save the param
	 */
	if (!mali_dvfs_is_enabled(dev))
		return count;

	dev_info(dev, "mali setting max pm_state: %ld[%s]\n", set_state,
		 drv_data->pm_states[set_state]->name);

	ret = device_state_pm_set_state(dev, drv_data->pm_states[set_state]);
	if (ret) {
		dev_err(dev, "mali sys set_pm_states error %d\n", ret);
		return ret;
	}

	return count;
}

static int read_pm_max_states(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct mali_platform_drv_data *drv_data = dev_get_drvdata(dev);

	if (drv_data->max_pm_state == 0)
		return sprintf(buf, "mali max pm_state not set\n");
	else
		return sprintf(buf, "mali max pm_state: %d[%s]\n",
			       drv_data->max_pm_state,
			       drv_data->pm_states[drv_data->max_pm_state]->name);
}

DEVICE_ATTR(dvfs_enable, S_IRUGO | S_IWUSR, show_dvfs_enable, set_dvfs_enable);
DEVICE_ATTR(utilisation, S_IRUGO, show_utilisation, NULL);
DEVICE_ATTR(error_count, 0644, error_count_show, NULL);
DEVICE_ATTR(pm_state, S_IRUGO | S_IWUSR, read_pm_states, set_pm_states);
DEVICE_ATTR(pm_max_state, S_IRUGO | S_IWUSR, read_pm_max_states, set_pm_max_states);

static struct attribute *mali_sysfs_entries[] = {
	&dev_attr_dvfs_enable.attr,
	&dev_attr_utilisation.attr,
	&dev_attr_error_count.attr,
	&dev_attr_pm_state.attr,
	&dev_attr_pm_max_state.attr,
	NULL,
};

static const struct attribute_group mali_attr_group = {
	.attrs	= mali_sysfs_entries,
};

static int mali_create_sysfs(struct device *dev)
{
	int ret;

	ret = sysfs_create_group(&dev->kobj, &mali_attr_group);
	if (ret)
		dev_err(dev, "create sysfs group error, %d\n", ret);

	return ret;
}

void mali_remove_sysfs(struct device *dev)
{
	sysfs_remove_group(&dev->kobj, &mali_attr_group);
}

const char *pm_state_name[] = {
	"disable",
	"low_perf",
	"mid_perf",
	"high_perf",
#if defined(GPU_USE_ULTRA_HIGH_PERF)
	"ultra_high_perf"
#endif
};

#if !defined(CONFIG_PLATFORM_DEVICE_PM_VIRT)

/* PM states & class */
static struct device_state_pm_state gpu_pm_states[] = {
	{ .name = "disable",   }, /* D3 */
	{ .name = "low_perf",  }, /* D0i3 */
	{ .name = "mid_perf",  }, /* D0i2 */
	{ .name = "high_perf", }, /* D0 */
/*	{ .name = "ultra_high_perf", },*/ /* D0 */
};

static int mali_clock_init(struct device *dev)
{
	int ret;

	struct mali_platform_drv_data *drv_data = dev_get_drvdata(dev);

	drv_data->clk_kernel = devm_clk_get(dev, "clk_kernel");
	if (IS_ERR(drv_data->clk_kernel)) {
		ret = PTR_ERR(drv_data->clk_kernel);
		dev_err(dev, "get clk_kernel failed, %d\n", ret);
		return ret;
	}

	drv_data->clk_ahb = devm_clk_get(dev, "clk_ahb");
	if (IS_ERR(drv_data->clk_ahb)) {
		ret = PTR_ERR(drv_data->clk_ahb);
		dev_err(dev, "prepare clk_ahb failed, %d\n", ret);
		return ret;
	}

	return 0;
}

static int mali_regulator_init(struct device *dev)
{
	struct mali_platform_drv_data *drv_data = dev_get_drvdata(dev);
	int ret;

	drv_data->regulator = devm_regulator_get(dev, "gpu");
	if (IS_ERR(drv_data->regulator)) {
		ret = PTR_ERR(drv_data->regulator);
		dev_err(dev, "init regulator failed, %d\n", ret);
		return ret;
	}

	return 0;
}

static int mali_platform_set_pm_state_by_num(struct device *dev, int state_num)
{
	struct mali_platform_drv_data *drv_data = dev_get_drvdata(dev);
	int ret;

	if (MALI_PLF_PM_STATE_D3) {
		clk_disable_unprepare(drv_data->clk_kernel);
		clk_disable_unprepare(drv_data->clk_ahb);

		ret = regulator_disable(drv_data->regulator);
		if (ret) {
			dev_err(dev, "regulator disable error\n");
			return ret;
		}
	} else {
		ret = regulator_enable(drv_data->regulator);
		if (ret) {
			dev_err(dev, "regulator enable error\n");
			return ret;
		}

		ret = clk_prepare_enable(drv_data->clk_kernel);
		if (ret) {
			dev_err(dev, "clk_kernel enable error\n");
			return ret;
		}

		ret = clk_prepare_enable(drv_data->clk_ahb);
		if (ret) {
			dev_err(dev, "clk_ahb enable error\n");
			return ret;
		}
	}

	return 0;
}

static int mali_platform_get_pm_state_id(char *name)
{
	int id;

	for (id = MALI_PLF_PM_STATE_D3; id < GPU_NUM_PM_STATES; id++) {
		if (!strcmp(name, gpu_pm_states[id].name))
			return id;
	}

	return GPU_NUM_PM_STATES;
}

static struct device_state_pm_state *mali_platform_get_initial_state(
		struct device *dev)
{
	return &gpu_pm_states[MALI_PLF_PM_STATE_D3];
}

static int mali_platform_set_pm_state(struct device *dev,
				      struct device_state_pm_state *state)
{
	int state_num;
	int ret = 0;

	state_num = mali_platform_get_pm_state_id(state->name);

	dev_info(dev, "set pm state (%d) %s\n", state_num, state->name);

	ret = mali_platform_set_pm_state_by_num(dev, state_num);
	if (ret)
		dev_err(dev, "Unable to set pm state (%d)%s\n",
			state_num, state->name);

	return ret;
}

static struct device_state_pm_ops gpu_pm_ops = {
	.set_state = mali_platform_set_pm_state,
	.get_initial_state = mali_platform_get_initial_state,
};

DECLARE_DEVICE_STATE_PM_CLASS(gpu);

static int mali_platform_native_init(struct device *dev)
{
	int ret;

	ret = device_state_pm_add_class(&gpu_pm_class);
	if (ret)
		return ret;

	ret = mali_regulator_init(dev);
	if (ret)
		return ret;

	ret = mali_clock_init(dev);
	if (ret)
		return ret;
}

#endif

static int mali_platform_pm_state_init(struct device *dev)
{
	struct mali_platform_drv_data *drv_data = dev_get_drvdata(dev);
	int ret;
	int i;
	unsigned int init_state;

	if (GPU_NUM_PM_STATES != ARRAY_SIZE(pm_state_name)) {
		dev_err(dev, "GPU_NUM_PM_STATES(%d) != array_size(pm_state_name)(%d)\n",
			GPU_NUM_PM_STATES, ARRAY_SIZE(pm_state_name));
		return -EINVAL;
	}

	drv_data->pm_platdata = of_device_state_pm_setup(dev->of_node);
	if (IS_ERR(drv_data->pm_platdata)) {
		ret = PTR_ERR(drv_data->pm_platdata);
		dev_err(dev, "pm state setup error %d\n", ret);
		return ret;
	}

	ret = device_state_pm_set_class(dev,
					drv_data->pm_platdata->pm_user_name);
	if (ret) {
		dev_err(dev, "pm state set class error %d\n", ret);
		return ret;
	}

	for (i = 0; i < GPU_NUM_PM_STATES; i++) {
		drv_data->pm_states[i] = device_state_pm_get_state_handler(dev,
				pm_state_name[i]);
		if (!drv_data->pm_states[i]) {
			dev_err(dev, "unable to get pm state handle\n");
			return -EINVAL;
		}
		dev_info(dev, "drv_data->pm_states[%d].name = %s\n",
			 i, drv_data->pm_states[i]->name);
	}

	drv_data->once_glb = 1;
	drv_data->curr_pm_state = GPU_MAX_PM_STATE;
	drv_data->resume_pm_state = GPU_MAX_PM_STATE;
	drv_data->max_pm_state = 0;
	init_state = drv_data->curr_pm_state;

	dev_info(dev, "mali pm_state init %d[%s]\n", drv_data->curr_pm_state,
		 drv_data->pm_states[drv_data->curr_pm_state]->name);

	ret = device_state_pm_set_state(dev, drv_data->pm_states[init_state]);
	if (ret) {
		dev_err(dev, "pm_state init error (%d)\n", ret);
		return ret;
	}

	return 0;
}

_mali_osk_errcode_t mali_platform_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mali_platform_drv_data *mali_drv_data;
	int ret;

	mali_drv_data = devm_kzalloc(dev, sizeof(*mali_drv_data), GFP_KERNEL);
	if (!mali_drv_data)
		return _MALI_OSK_ERR_NOMEM;

	dev_set_drvdata(dev, mali_drv_data);

	mali_drv_data->dev = dev;

	mali_dev = dev;

#if !defined(CONFIG_PLATFORM_DEVICE_PM_VIRT)
	ret = mali_platform_native_init(dev);
	if (ret)
		return ret;
#endif

	ret = mali_platform_pm_state_init(dev);
	if (ret)
		return ret;

	ret = mali_create_sysfs(dev);
	if (ret)
		return ret;

	ret = mali_dvfs_init(dev);
	if (ret)
		goto remove_sysfs;

	mali_core_scaling_enable = 0;

	dev_info(dev, "%s, success\n", __func__);

	return _MALI_OSK_ERR_OK;
remove_sysfs:
	mali_remove_sysfs(dev);

	return _MALI_OSK_ERR_FAULT;
}

_mali_osk_errcode_t mali_platform_deinit(struct platform_device *pdev)
{
	mali_core_scaling_term();

	return 0;
}

void mali_gpu_utilization_handler(struct mali_gpu_utilization_data *data)
{
	if (data->utilization_pp > 256)
		return;

	if (mali_core_scaling_enable)
		mali_core_scaling_update(data);

	dev_dbg(mali_dev, "utilization:%d\r\n", data->utilization_pp);

	mali_dvfs_event(mali_dev, data->utilization_pp);
}
