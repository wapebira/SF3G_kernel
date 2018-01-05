/*
 * Rockchip SoC Mali-450 DVFS driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */
#include <linux/workqueue.h>
#include <linux/types.h>
#ifndef _MALI_DVFS_H_
#define _MALI_DVFS_H_

struct device;

struct mali_dvfs {
	struct work_struct work;
	unsigned int utilisation;
	bool enabled;
};

#define GPU_THROTTLE_UP_THRESHOLD 166 /*65%*/
#define GPU_THROTTLE_DOWN_THRESHOLD 76 /*30%*/

int mali_dvfs_init(struct device *dev);
void mali_dvfs_term(struct device *dev);
void mali_set_dvfs(struct device *dev, bool enable);
bool mali_dvfs_is_enabled(struct device *dev);
void mali_dvfs_enable(struct device *dev);
void mali_dvfs_disable(struct device *dev);
unsigned int mali_dvfs_utilisation(struct device *dev);
int mali_dvfs_event(struct device *dev, u32 utilisation);
#endif		/*_MALI_DVFS_H_*/
