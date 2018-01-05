#ifndef __MALI_PLATFORM_H__
#define __MALI_PLATFORM_H__

#include "mali_dvfs.h"
#include "mali_osk.h"
#include <linux/mali/mali_utgard.h>
#include <linux/xgold_noc.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief description of power change reasons
 */
enum mali_power_mode_tag {
	MALI_POWER_MODE_ON,           /**< Power Mali on */
	MALI_POWER_MODE_LIGHT_SLEEP,  /**< Mali has been idle for a short time, or runtime PM suspend */
	MALI_POWER_MODE_DEEP_SLEEP,   /**< Mali has been idle for a long time, or OS suspend */
};

struct mali_fv {
	unsigned long freq;
	unsigned long volt;
};

#define GPU_USE_ULTRA_HIGH_PERF 1

#if defined(GPU_USE_ULTRA_HIGH_PERF)
#define GPU_NUM_PM_STATES 5
#else
#define GPU_NUM_PM_STATES 4
#endif /* defined(GPU_USE_ULTRA_HIGH_PERF) */

#define GPU_MIN_PM_STATE 1
#define GPU_MAX_PM_STATE (GPU_NUM_PM_STATES - 1)
#define GPU_INITIAL_PM_STATE GPU_MAX_PM_STATE

/* PM states index */
#define MALI_PLF_PM_STATE_D3	0
#define MALI_PLF_PM_STATE_D0	GPU_MAX_PM_STATE

struct mali_platform_drv_data {
#if !defined(CONFIG_PLATFORM_DEVICE_PM_VIRT)
	struct clk *clk_kernel;
	struct clk *clk_ahb;
	struct regulator *regulator;
#endif
	struct mali_dvfs dvfs;
	struct device *dev;
	struct device_state_pm_state *pm_states[GPU_NUM_PM_STATES];
	unsigned int curr_pm_state;
	unsigned int resume_pm_state;
	unsigned int max_pm_state;
	struct device_pm_platdata *pm_platdata;
	unsigned int once_glb;
};

/** @brief Platform specific setup and initialisation of MALI
 *
 * This is called from the entrypoint of the driver to initialize the platform
 *
 * @return _MALI_OSK_ERR_OK on success otherwise, a suitable _mali_osk_errcode_t error.
 */
_mali_osk_errcode_t mali_platform_init(struct platform_device *pdev);

/** @brief Platform specific deinitialisation of MALI
 *
 * This is called on the exit of the driver to terminate the platform
 *
 * @return _MALI_OSK_ERR_OK on success otherwise, a suitable _mali_osk_errcode_t error.
 */
_mali_osk_errcode_t mali_platform_deinit(struct platform_device *pdev);

/** @brief Platform specific handling of GPU utilization data
 *
 * When GPU utilization data is enabled, this function will be
 * periodically called.
 *
 * @param utilization The workload utilization of the Mali GPU. 0 = no utilization, 256 = full utilization.
 */
void mali_gpu_utilization_handler(struct mali_gpu_utilization_data *data);

#ifdef __cplusplus
}
#endif
#endif
