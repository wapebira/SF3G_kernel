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
#include "imc_idi_gnss_of.h"

#ifdef CONFIG_OF
/*****************************************************************************
 *	name:		imc_idi_gnss_of_get_platdata
 *	description:
 *	in params:	struct idi_peripheral_device *p_device
 *	out params:
 *	return val:
 ****************************************************************************/
struct imc_idi_gnss_platdata *imc_idi_gnss_of_get_platdata(
				struct idi_peripheral_device *p_device)
{
	struct device_node *np = p_device->device.of_node;
	struct device *dev = &p_device->device;
	struct xgold_usif_platdata *uxp_platdata;
	struct imc_idi_gnss_platdata *platdata;
#if defined(CONFIG_PINCTRL) && !defined(CONFIG_PINCTRL_SINGLE)
	struct device_node *npinctrl;
#endif

	if (!dev || !np)
		return ERR_PTR(-EINVAL);

	uxp_platdata = dev_get_platdata(dev);
	if (!uxp_platdata)
		return ERR_PTR(-ENODEV);

	platdata = xgold_port_platdata_priv(uxp_platdata);

#ifdef NATIVE_PLATFORM_DEVICE_PM
	platdata->gps_bb_data = of_clk_get_by_name(np, GNSS_BB_DATA_CLK_NAME);
	if (IS_ERR(platdata->gps_bb_data)) {
		dev_warn(dev, "%s clock is not available\n",
			 GNSS_BB_DATA_CLK_NAME);
		platdata->gps_bb_data = NULL;
	}

	platdata->gps_bb_clk = of_clk_get_by_name(np, GNSS_BB_CLK_NAME);
	if (IS_ERR(platdata->gps_bb_clk)) {
		dev_warn(dev, "%s clock is not available\n", GNSS_BB_CLK_NAME);
		platdata->gps_bb_clk = NULL;
	}

	platdata->ext_uart_clk =
	of_clk_get_by_name(np, GNSS_BB_EXT_UART_CLK_NAME);
	if (IS_ERR(platdata->ext_uart_clk)) {
		dev_warn(dev, "%s clock is not available\n",
			 GNSS_BB_EXT_UART_CLK_NAME);
		platdata->ext_uart_clk = NULL;
	}

	platdata->rtc = of_clk_get_by_name(np, GNSS_BB_RTC_CLK_NAME);
	if (IS_ERR(platdata->rtc)) {
		dev_warn(dev, "%s clock is not available\n",
			 GNSS_BB_RTC_CLK_NAME);
		platdata->rtc = NULL;
	}

	platdata->gps_ref_clk_req =
	of_clk_get_by_name(np, GNSS_REF_REQ_CLK_NAME);
	if (IS_ERR(platdata->gps_ref_clk_req)) {
		dev_warn(dev, "%s clock is not available\n",
			 GNSS_REF_REQ_CLK_NAME);
		platdata->gps_ref_clk_req = NULL;
	}

	platdata->gps_bb_clk_req = of_clk_get_by_name(np, GNSS_BB_REQ_CLK_NAME);
	if (IS_ERR(platdata->gps_bb_clk_req)) {
		dev_warn(dev, "%s clock is not available\n",
			 GNSS_BB_REQ_CLK_NAME);
		platdata->gps_bb_clk_req = NULL;
	}

	platdata->gps_rf_clk_req = of_clk_get_by_name(np, GNSS_RF_REQ_CLK_NAME);
	if (IS_ERR(platdata->gps_rf_clk_req)) {
		dev_warn(dev, "%s clock is not available\n",
			 GNSS_RF_REQ_CLK_NAME);
		platdata->gps_rf_clk_req = NULL;
	}

	platdata->gps_sys_clk_req =
	of_clk_get_by_name(np, GNSS_SYS_REQ_CLK_NAME);
	if (IS_ERR(platdata->gps_sys_clk_req)) {
		dev_warn(dev, "%s clock is not available\n",
			 GNSS_SYS_REQ_CLK_NAME);
		platdata->gps_sys_clk_req = NULL;
	}

	platdata->gps_auto_pu_phs1 =
	of_clk_get_by_name(np, GNSS_AUTO_PU_PHS1_CLK_NAME);
	if (IS_ERR(platdata->gps_auto_pu_phs1)) {
		dev_warn(dev, "%s clock is not available\n",
			 GNSS_AUTO_PU_PHS1_CLK_NAME);
		platdata->gps_auto_pu_phs1 = NULL;
	}

	platdata->gps_auto_pu_phs4 =
	of_clk_get_by_name(np, GNSS_AUTO_PU_PHS4_CLK_NAME);
	if (IS_ERR(platdata->gps_auto_pu_phs4)) {
		dev_warn(dev, "%s clock is not available\n",
			 GNSS_AUTO_PU_PHS4_CLK_NAME);
		platdata->gps_auto_pu_phs4 = NULL;
	}

	platdata->gps_auto_pu_phs5 =
	of_clk_get_by_name(np, GNSS_AUTO_PU_PHS5_CLK_NAME);
	if (IS_ERR(platdata->gps_auto_pu_phs5)) {
		dev_warn(dev, "%s clock is not available\n",
			 GNSS_AUTO_PU_PHS5_CLK_NAME);
		platdata->gps_auto_pu_phs5 = NULL;
	}
#else
	if (p_device->pm_platdata) {
		if (p_device->pm_platdata->pm_user_name)
			dev_dbg(dev, "pm_user_name = %s\n",
				p_device->pm_platdata->pm_user_name);
		if (p_device->pm_platdata->pm_class_name)
			dev_dbg(dev, "pm_class_name = %s\n",
				p_device->pm_platdata->pm_class_name);
		if (p_device->pm_platdata->pm_state_D0_name)
			dev_dbg(dev, "pm_state_D0_name = %s\n",
				p_device->pm_platdata->pm_state_D0_name);
		if (p_device->pm_platdata->pm_state_D0i0_name)
			dev_dbg(dev, "pm_state_D0i0_name = %s\n",
				p_device->pm_platdata->pm_state_D0i0_name);
		if (p_device->pm_platdata->pm_state_D0i1_name)
			dev_dbg(dev, "pm_state_D0i1_name = %s\n",
				p_device->pm_platdata->pm_state_D0i0_name);
		if (p_device->pm_platdata->pm_state_D0i0_name)
			dev_dbg(dev, "pm_state_D0i2_name = %s\n",
				p_device->pm_platdata->pm_state_D0i2_name);
		if (p_device->pm_platdata->pm_state_D0i3_name)
			dev_dbg(dev, "pm_state_D0i3_name = %s\n",
				p_device->pm_platdata->pm_state_D0i3_name);
		if (p_device->pm_platdata->pm_state_D3_name)
			dev_dbg(dev, "pm_state_D3_name = %s\n",
				p_device->pm_platdata->pm_state_D3_name);
	} else {
		dev_err(dev, "no pm_platdata\n");
	}

#endif /* NATIVE_PLATFORM_DEVICE_PM */

#ifdef CONFIG_PINCTRL
#ifdef CONFIG_PINCTRL_SINGLE
	dev_dbg(dev, "working with new pinctrl API\n");
	platdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(platdata->pinctrl))
		goto skip_pinctrl;

	/* tcxo */
	platdata->pins_default = pinctrl_lookup_state(platdata->pinctrl,
			PINCTRL_STATE_DEFAULT);
	if (IS_ERR(platdata->pins_default))
		dev_err(dev, "could not get default pinstate\n");

	platdata->pins_sleep = pinctrl_lookup_state(platdata->pinctrl,
			PINCTRL_STATE_SLEEP);
	if (IS_ERR(platdata->pins_sleep))
		dev_err(dev, "could not get sleep pinstate\n");

	platdata->pins_inactive = pinctrl_lookup_state(platdata->pinctrl,
			"inactive");
	if (IS_ERR(platdata->pins_inactive))
		dev_err(dev, "could not get inactive pinstate\n");

	/* ext_lna */
	platdata->ext_lna_pins_default = pinctrl_lookup_state(platdata->pinctrl,
			"default_lna");
	if (IS_ERR(platdata->ext_lna_pins_default))
		dev_err(dev, "could not get default_lna pinstate\n");

	 platdata->ext_lna_pins_inactive =
		 pinctrl_lookup_state(platdata->pinctrl,"inactive_lna");
	 if (IS_ERR(platdata->ext_lna_pins_inactive))
		 dev_err(dev, "could not get inactive_lna pinstate\n");

	 /* fta */
	 platdata->fta_pins_default = pinctrl_lookup_state(platdata->pinctrl,
			 "default_fta");
	 if (IS_ERR(platdata->fta_pins_default))
		 dev_err(dev, "could not get default_fta pinstate\n");

	 platdata->fta_pins_inactive = pinctrl_lookup_state(platdata->pinctrl,
			 "inactive_fta");
	 if (IS_ERR(platdata->fta_pins_inactive))
		 dev_err(dev, "could not get inactive_fta pinstate\n");

	 /* pos_fta */
	 platdata->fta_pos_pins_default =
		 pinctrl_lookup_state(platdata->pinctrl, "gnss_default");
	 if (IS_ERR(platdata->fta_pos_pins_default))
		 dev_err(dev, "could not get gnss_default pinstate\n");

	 platdata->fta_pos_pins_inactive =
		 pinctrl_lookup_state(platdata->pinctrl, "gnss_inactive");
	 if (IS_ERR(platdata->fta_pos_pins_inactive))
		 dev_err(dev, "could not get gnss_inactive pinstate\n");


#else
	dev_dbg(dev, "working with old pinctrl API\n");
	npinctrl = of_parse_phandle(np, "intel,gnss,pinctrl", 0);
	if (!npinctrl) {
		dev_err(dev, "No GNSS Pinctrl associated\n");
		goto no_pinctrl;
	}

	platdata->pinctrl_name = npinctrl->name;
	if (of_property_read_string(np, "intel,gnss,pinctrl,tcxo",
				&platdata->tcxo_pinctrl_name)) {
		dev_err(dev, "Unable to read TCXO pinctrl name");
		goto no_pinctrl;
	}
#endif
#endif /* CONFIG_PINCTRL */
skip_pinctrl:
	return platdata;

#if defined(CONFIG_PINCTRL) && !defined(CONFIG_PINCTRL_SINGLE)
no_pinctrl:
	/*TODO: Call clk_put */
	if (platdata->gps_bb_data)
		clk_put(platdata->gps_bb_data);

	if (platdata->gps_bb_clk)
		clk_put(platdata->gps_bb_clk);

	if (platdata->ext_uart_clk)
		clk_put(platdata->ext_uart_clk);

	if (platdata->rtc)
		clk_put(platdata->rtc);

	if (platdata->gps_ref_clk_req)
		clk_put(platdata->gps_ref_clk_req);

	if (platdata->gps_bb_clk_req)
		clk_put(platdata->gps_bb_clk_req);

	if (platdata->gps_rf_clk_req)
		clk_put(platdata->gps_rf_clk_req);

	if (platdata->gps_sys_clk_req)
		clk_put(platdata->gps_sys_clk_req);

	if (platdata->gps_auto_pu_phs1)
		clk_put(platdata->gps_auto_pu_phs1);

	if (platdata->gps_auto_pu_phs4)
		clk_put(platdata->gps_auto_pu_phs4);

	if (platdata->gps_auto_pu_phs5)
		clk_put(platdata->gps_auto_pu_phs5);

	return NULL;
#endif
}
#endif /* CONFIG_OF */

