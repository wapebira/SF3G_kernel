/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include "imc_idi_gnss.h"

/*****************************************************************************
 *	name:		gnss_set_pinctrl_state
 *	description:
 *	in params:
 *	out params:
 *	return val:
 ****************************************************************************/
#ifdef CONFIG_PINCTRL_SINGLE
int gnss_set_pinctrl_state(struct device *dev, struct pinctrl_state *state)
{
	int ret = 0;
	struct xgold_usif_platdata *uxp_platdata = dev_get_platdata(dev);
	struct imc_idi_gnss_platdata *platdata =
		xgold_port_platdata_priv(uxp_platdata);


	IMC_IDI_GNSS_ENTER;

	if (!platdata) {
		dev_err(dev, "Unable to retrieve usif platform data\n");
		return -EINVAL;
	}

	if (!IS_ERR_OR_NULL(state)) {
		ret = pinctrl_select_state(platdata->pinctrl, state);
		if (ret)
			dev_err(dev, "%d:could not set pins\n", __LINE__);
	}
	IMC_IDI_GNSS_EXIT;

	return ret;
}
#endif /* CONFIG_PINCTRL_SINGLE */

#ifdef NATIVE_PLATFORM_DEVICE_PM
/*****************************************************************************
 *	name:		gnss_disable_clocks
 *	description:
 *	in params:	struct imc_idi_gnss_platdata *platdata
 *	out params:	none
 *	return val:
 ****************************************************************************/
static void gnss_disable_clocks(struct imc_idi_gnss_platdata *platdata)
{

	IMC_IDI_GNSS_ENTER;

	if (platdata->gps_bb_data) {
		clk_disable(platdata->gps_bb_data);
		clk_unprepare(platdata->gps_bb_data);
	}

	if (platdata->gps_bb_clk) {
		clk_disable(platdata->gps_bb_clk);
		clk_unprepare(platdata->gps_bb_clk);
	}

	if (platdata->ext_uart_clk) {
		clk_disable(platdata->ext_uart_clk);
		clk_unprepare(platdata->ext_uart_clk);
	}

	if (platdata->rtc) {
		clk_disable(platdata->rtc);
		clk_unprepare(platdata->rtc);
	}

	if (platdata->gps_ref_clk_req) {
		clk_disable(platdata->gps_ref_clk_req);
		clk_unprepare(platdata->gps_ref_clk_req);
	}

	if (platdata->gps_bb_clk_req) {
		clk_disable(platdata->gps_bb_clk_req);
		clk_unprepare(platdata->gps_bb_clk_req);
	}

	if (platdata->gps_rf_clk_req) {
		clk_disable(platdata->gps_rf_clk_req);
		clk_unprepare(platdata->gps_rf_clk_req);
	}

	if (platdata->gps_sys_clk_req) {
		clk_disable(platdata->gps_sys_clk_req);
		clk_unprepare(platdata->gps_sys_clk_req);
	}

	if (platdata->gps_auto_pu_phs1) {
		clk_disable(platdata->gps_auto_pu_phs1);
		clk_unprepare(platdata->gps_auto_pu_phs1);
	}

	if (platdata->gps_auto_pu_phs4) {
		clk_disable(platdata->gps_auto_pu_phs4);
		clk_unprepare(platdata->gps_auto_pu_phs4);
	}

	if (platdata->gps_auto_pu_phs5) {
		clk_disable(platdata->gps_auto_pu_phs5);
		clk_unprepare(platdata->gps_auto_pu_phs5);
	}

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		gnss_enable_clocks
 *	description:
 *	in params:	struct imc_idi_gnss_platdata *platdata
 *	out params:	none
 *	return val:
 ****************************************************************************/
static int gnss_enable_clocks(struct imc_idi_gnss_platdata *platdata)
{
	int ret;

	IMC_IDI_GNSS_ENTER;

	if (platdata->gps_bb_data) {
		ret = clk_prepare(platdata->gps_bb_data);
		if (ret)
			return ret;
		ret = clk_enable(platdata->gps_bb_data);
		if (ret)
			return ret;
	}

	if (platdata->gps_bb_clk) {
		ret = clk_prepare(platdata->gps_bb_clk);
		if (ret)
			return ret;
		ret = clk_enable(platdata->gps_bb_clk);
		if (ret)
			return ret;
	}

	if (platdata->ext_uart_clk) {
		ret = clk_prepare(platdata->ext_uart_clk);
		if (ret)
			return ret;
		ret = clk_enable(platdata->ext_uart_clk);
		if (ret)
			return ret;
	}

	if (platdata->rtc) {
		ret = clk_prepare(platdata->rtc);
		if (ret)
			return ret;
		ret = clk_enable(platdata->rtc);
		if (ret)
			return ret;
	}

	if (platdata->gps_ref_clk_req) {
		ret = clk_prepare(platdata->gps_ref_clk_req);
		if (ret)
			return ret;
		ret = clk_enable(platdata->gps_ref_clk_req);
		if (ret)
			return ret;
	}

	if (platdata->gps_bb_clk_req) {
		ret = clk_prepare(platdata->gps_bb_clk_req);
		if (ret)
			return ret;
		ret = clk_enable(platdata->gps_bb_clk_req);
		if (ret)
			return ret;
	}

	if (platdata->gps_rf_clk_req) {
		ret = clk_prepare(platdata->gps_rf_clk_req);
		if (ret)
			return ret;
		ret = clk_enable(platdata->gps_rf_clk_req);
		if (ret)
			return ret;
	}

	if (platdata->gps_sys_clk_req) {
		ret = clk_prepare(platdata->gps_sys_clk_req);
		if (ret)
			return ret;
		ret = clk_enable(platdata->gps_sys_clk_req);
		if (ret)
			return ret;
	}

	if (platdata->gps_auto_pu_phs1) {
		ret = clk_prepare(platdata->gps_auto_pu_phs1);
		if (ret)
			return ret;
		ret = clk_enable(platdata->gps_auto_pu_phs1);
		if (ret)
			return ret;
	}

	if (platdata->gps_auto_pu_phs4) {
		ret = clk_prepare(platdata->gps_auto_pu_phs4);
		if (ret)
			return ret;
		ret = clk_enable(platdata->gps_auto_pu_phs4);
		if (ret)
			return ret;
	}

	if (platdata->gps_auto_pu_phs5) {
		ret = clk_prepare(platdata->gps_auto_pu_phs5);
		if (ret)
			return ret;
		ret = clk_enable(platdata->gps_auto_pu_phs5);
		if (ret)
			return ret;
	}

	IMC_IDI_GNSS_EXIT;
	return 0;
}

#define GNSS_PM_STATE_D0	"enable_def_dclk_832_pclk_1248"
#define GNSS_PM_STATE_D0I0	"enable_dclk_96_pclk_1248"
#define GNSS_PM_STATE_D0I1	"enable_dclk_832_pclk_104"
#define GNSS_PM_STATE_D0I2	"enable_dclk_96_pclk_96"
#define GNSS_PM_STATE_D0I3	"idle"
#define GNSS_PM_STATE_D3	"disable"

static int gnss_set_pm_state(struct device *dev,
			struct device_state_pm_state *state)
{
	int ret = 0;
	struct xgold_usif_platdata *uxp_platdata = dev_get_platdata(dev);
	struct imc_idi_gnss_platdata *platdata =
		xgold_port_platdata_priv(uxp_platdata);

	IMC_IDI_GNSS_ENTER;

	if (!strcmp(state->name, GNSS_PM_STATE_D0))
		ret = gnss_enable_clocks(platdata);
	else if (!strcmp(state->name, GNSS_PM_STATE_D0I0))
		ret = gnss_enable_clocks(platdata);
	else if (!strcmp(state->name, GNSS_PM_STATE_D0I1))
		ret = gnss_enable_clocks(platdata);
	else if (!strcmp(state->name, GNSS_PM_STATE_D0I2))
		ret = gnss_enable_clocks(platdata);
	else if (!strcmp(state->name, GNSS_PM_STATE_D0I3))
		ret = gnss_enable_clocks(platdata);
	else if (!strcmp(state->name, GNSS_PM_STATE_D3))
		gnss_disable_clocks(platdata);

	IMC_IDI_GNSS_EXIT;

	return ret;
}

static struct device_state_pm_state *gnss_get_initial_pm_state(
		struct device *_dev)
{
	IMC_IDI_GNSS_ENTER;

	return device_state_pm_get_state_handler(_dev, GNSS_PM_STATE_D3);

	IMC_IDI_GNSS_EXIT;
}

static struct device_state_pm_ops gnss_pm_ops = {
	.set_state = gnss_set_pm_state,
	.get_initial_state = gnss_get_initial_pm_state,
};

/* IDI PM states & class */

static struct device_state_pm_state gnss_pm_states[] = {
	{ .name = GNSS_PM_STATE_D0 },
	{ .name = GNSS_PM_STATE_D0I0 },
	{ .name = GNSS_PM_STATE_D0I1 },
	{ .name = GNSS_PM_STATE_D0I2 },
	{ .name = GNSS_PM_STATE_D0I3 },
	{ .name = GNSS_PM_STATE_D3 },
};

DECLARE_DEVICE_STATE_PM_CLASS(gnss);

int gnss_pm_add_class(void)
{
	int ret = 0;

	IMC_IDI_GNSS_ENTER;

	ret = device_state_pm_add_class(&gnss_pm_class);
	if (ret)
		pr_err("Error while adding GNSS pm class\n");

	IMC_IDI_GNSS_EXIT;

	return ret;
}

#endif /* NATIVE_PLATFORM_DEVICE_PM */
