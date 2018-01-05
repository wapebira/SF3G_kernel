/*
 * Rockchip SoC Mali-450 DVFS driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

#include "mali_platform.h"
#include "mali_dvfs.h"

#define work_to_dvfs(w) container_of(w, struct mali_dvfs, work)
#define dvfs_to_drv_data(dvfs) \
	container_of(dvfs, struct mali_platform_drv_data, dvfs)

static void mali_dvfs_event_proc(struct work_struct *w)
{
	struct mali_dvfs *dvfs = work_to_dvfs(w);
	struct mali_platform_drv_data *drv_data = dvfs_to_drv_data(dvfs);
	unsigned int utilisation = dvfs->utilisation;
	int ret;
	unsigned int prev_pm_state;
	unsigned int new_state;

	dev_dbg(drv_data->dev, "utilisation = %d\n", dvfs->utilisation);

	prev_pm_state = drv_data->curr_pm_state;

	if (utilisation > GPU_THROTTLE_UP_THRESHOLD &&
	    drv_data->curr_pm_state < GPU_MAX_PM_STATE)
		drv_data->curr_pm_state += 1;
	else if (drv_data->curr_pm_state > GPU_MIN_PM_STATE &&
		 utilisation < GPU_THROTTLE_DOWN_THRESHOLD)
		drv_data->curr_pm_state -= 1;
	else
		return;

	if (drv_data->max_pm_state != 0 &&
	    drv_data->curr_pm_state > drv_data->max_pm_state)
	    drv_data->curr_pm_state = drv_data->max_pm_state;

	new_state = drv_data->curr_pm_state;

	ret = device_state_pm_set_state(drv_data->dev,
					drv_data->pm_states[new_state]);
	if (ret) {
		drv_data->curr_pm_state = prev_pm_state;
		dev_err(drv_data->dev, "device set pm state error (%d)\n", ret);
		return;
	}

	if (drv_data->curr_pm_state > prev_pm_state) {
		if ((drv_data->once_glb) &&
		    (prev_pm_state == GPU_MIN_PM_STATE)) {
			xgold_noc_qos_set("GPU");
			drv_data->once_glb--;
		}
	}
}

bool mali_dvfs_is_enabled(struct device *dev)
{
	struct mali_platform_drv_data *drv_data = dev_get_drvdata(dev);
	struct mali_dvfs *dvfs = &drv_data->dvfs;

	return dvfs->enabled;
}

void mali_dvfs_enable(struct device *dev)
{
	struct mali_platform_drv_data *drv_data = dev_get_drvdata(dev);
	struct mali_dvfs *dvfs = &drv_data->dvfs;

	dvfs->enabled = true;
}

void mali_dvfs_disable(struct device *dev)
{
	struct mali_platform_drv_data *drv_data = dev_get_drvdata(dev);
	struct mali_dvfs *dvfs = &drv_data->dvfs;
	unsigned int new_state;
	int ret;

	dvfs->enabled = false;
	cancel_work_sync(&dvfs->work);

	new_state = drv_data->resume_pm_state;
	ret = device_state_pm_set_state(drv_data->dev,
					drv_data->pm_states[new_state]);
	if (ret) {
		dev_err(drv_data->dev,
			"mali_dvfs_disable, device set pm state error(%d)\n", ret);
		return;
	}

	drv_data->curr_pm_state = new_state;
}

unsigned int mali_dvfs_utilisation(struct device *dev)
{
	struct mali_platform_drv_data *drv_data = dev_get_drvdata(dev);
	struct mali_dvfs *dvfs = &drv_data->dvfs;

	return dvfs->utilisation;
}

int mali_dvfs_event(struct device *dev, u32 utilisation)
{
	struct mali_platform_drv_data *drv_data = dev_get_drvdata(dev);
	struct mali_dvfs *dvfs = &drv_data->dvfs;

	dvfs->utilisation = utilisation;

	if (dvfs->enabled)
		schedule_work(&dvfs->work);

	return MALI_TRUE;
}

int mali_dvfs_init(struct device *dev)
{
	struct mali_platform_drv_data *drv_data = dev_get_drvdata(dev);
	struct mali_dvfs *dvfs = &drv_data->dvfs;

	INIT_WORK(&dvfs->work, mali_dvfs_event_proc);
	dvfs->enabled = true;

	return 0;
}

void mali_dvfs_term(struct device *dev)
{
	struct mali_platform_drv_data *drv_data = dev_get_drvdata(dev);
	struct mali_dvfs *dvfs = &drv_data->dvfs;

	dvfs->enabled = false;
	cancel_work_sync(&dvfs->work);
}
