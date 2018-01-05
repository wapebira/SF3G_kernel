/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2013 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <ilw@linux.intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2014 Intel Mobile Communications GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/reset.h>

#ifdef CONFIG_OF
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/idi/idi_ids.h>
#else
#include <plat/reset_ids.h>
#endif

#include "iwl-drv.h"
#include "iwl-config.h"
#include "iwl-prph.h"
#include "iwl-trans.h"
#include "iwl-fh.h"
#include "iwl-csr.h"
#include "iwl-io.h"
#include "iwl-op-mode.h"
#include "iwl-debug.h"
#include "shared.h"
#include "idi_irq_csr.h"
#include "idi_al.h"
#include "idi_internal.h"
#include "idi_utils.h"
#include "idi_tx.h"
#include "idi_tx_policy.h"
#include "idi_constants.h"

/* if not running compatible al_emulation - ignore changes */
#ifndef IWL_IDI_INTA
#define iwl_inta_em_clean_irq_called(trans) 0
#define iwl_inta_em_read_called(trans) 0
#else
/* for simulation of irq-register. will be replaced with target access */
#include "iwl-em-intr.h"
#endif

/* entries names when working with Open Firmware enabled */
#define IWL_IDI_NAME "IWL_IDI"
#define IWL_IDI_AL_MEM "wlan"
#define IWL_IDI_IRQ_NAME "wlan_irq"

#ifdef CONFIG_OF
#define WIFI_CLK_KERNEL_NAME "wlan_clk_req"
#define WIFI_CLK_RTC_NAME "rtc_clk_req"

/* there aren't power states in CV Linux */
enum idi_devices_power_state {D0, D0I0, D0I1, D0I2, D0I3, D3};

static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_WLAN,
	},

	{ /* end: all zeroes */},
};
#endif

#define IWL_IDI_IS_Q_PAN(q) (((q) > 3) && ((q) < 8))

#define ADDR_IN_AL_MSK (0x80000000)
#define IS_AL_ADDR(ofs) ((ofs) & (ADDR_IN_AL_MSK))

static const struct iwl_trans_ops trans_ops_idi;

static int iwl_trans_idi_al_read(struct iwl_trans *trans, u32 ofs)
{
	unsigned long flags;

	if (iwl_trans_grab_nic_access(trans, false, &flags)) {
		int ret = idi_al_read(trans, ofs);
		iwl_trans_release_nic_access(trans, &flags);
		return ret;
	}

	return -EBUSY;
}

static int iwl_trans_idi_al_write(struct iwl_trans *trans, u32 ofs, u32 val)
{
	unsigned long flags;

	if (iwl_trans_grab_nic_access(trans, false, &flags)) {
		idi_al_write(trans, ofs, val);
		iwl_trans_release_nic_access(trans, &flags);
		return 0;
	}

	return -EBUSY;
}

static u32 iwl_trans_idi_read_prph(struct iwl_trans *trans, u32 ofs)
{
	return idi_al_read_lmac_prph(trans, ofs);
}

static void iwl_trans_idi_write_prph(struct iwl_trans *trans, u32 ofs, u32 val)
{
	idi_al_write_lmac_prph(trans, ofs, val);
}

static void iwl_trans_idi_write8(struct iwl_trans *trans, u32 ofs, u8 val)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);

	if (WARN_ON_ONCE(trans_idi->power_state != PM_ENABLE))
		return;

	if (IS_AL_ADDR(ofs)) { /* AL address */
		idi_al_write8(trans, (ofs & ~ADDR_IN_AL_MSK), val);
	} else { /* LMAC address */
		IWL_ERR(trans, "write8 (LMAC address) is not implemented\n");
		/* TODO: write to LMAC */
	}
}

static void iwl_trans_idi_write32(struct iwl_trans *trans, u32 ofs, u32 val)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);

	if (WARN_ON_ONCE(trans_idi->power_state != PM_ENABLE))
		return;

	/* Usually this fucntion is used to access LMAC registers.
	 * Added support to AL registers; differentiation using MS bit.
	 * This is temporal solution to enable SV tools */
	if (IS_AL_ADDR(ofs)) { /* AL address */
		idi_al_write(trans, (ofs & ~ADDR_IN_AL_MSK), val);
	} else { /* LMAC address */
		idi_al_write_lmac(trans, ofs, val);
	}

}

static u32 iwl_trans_idi_read32(struct iwl_trans *trans, u32 ofs)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);

	if (WARN_ON_ONCE(trans_idi->power_state != PM_ENABLE))
		return -ENAVAIL;

	/* Usually this fucntion is used to access LMAC registers.
	 * Added support to AL registers; differentiation using MS bit.
	 * This is temporal solution to enable SV tools */
	if (IS_AL_ADDR(ofs)) { /* AL address */
		return idi_al_read(trans, (ofs & ~ADDR_IN_AL_MSK));
	} else { /* LMAC address */
		return idi_al_read_lmac(trans, ofs);
	}
}

static const char *get_csr_string(int cmd)
{
#define IWL_CMD(x) case x: return #x
	switch (cmd) {
	IWL_CMD(IDI_INTA_CSR_W1C);
	IWL_CMD(IDI_INTA_CSR_MASK);
	IWL_CMD(IDI_FH_INT_CSR_W1C);
	IWL_CMD(IDI_HW_REV_CSR);
	IWL_CMD(GP_LMAC_INIT_RST_ERR_STT_W1C);
	default:
		return "UNKNOWN";
	}
#undef IWL_CMD
}

void iwl_idi_dump_csr(struct iwl_trans *trans)
{
	int i;
	unsigned long flags;
	static const u32 csr_tbl[] = {
		IDI_INTA_CSR_W1C,
		IDI_INTA_CSR_MASK,
		IDI_FH_INT_CSR_W1C,
		IDI_HW_REV_CSR,
		GP_LMAC_INIT_RST_ERR_STT_W1C,
	};

	if (!iwl_trans_grab_nic_access(trans, false, &flags)) {
		IWL_ERR(trans, "Failed to wake up NIC\n");
		return;
	}

	IWL_ERR(trans, "CSR values:\n");
	for (i = 0; i <  ARRAY_SIZE(csr_tbl); i++) {
		IWL_ERR(trans, "  %25s: 0X%08x\n",
			get_csr_string(csr_tbl[i]),
			idi_al_read(trans, csr_tbl[i]));
	}

	iwl_trans_release_nic_access(trans, &flags);
}

#define SCU_RSTMODS 0xe6a00100
#define SCU_RSTMODS_WLAN_IPRST 0x100
#define SPCU_WLAN_POWER 0xE6402024

static void iwl_idi_config_st_arbiter(void)
{
	/* ABB_CguDTClkCtrl - set system trace and mtm clock souce as PLLA */
	iowrite32(0x30303, (void __iomem *)0xe640110c);

	/* ABB_SpcuMemPower - set the power of the trace memory */
	iowrite32(0x1, (void __iomem *)0xe640201c);

	/* set MIPI2 PCL, PCL_26 - PCL_30 */
	iowrite32(0x10, (void __iomem *)0xe6300274);
	iowrite32(0x10, (void __iomem *)0xe6300278);
	iowrite32(0x10, (void __iomem *)0xe630027c);
	iowrite32(0x10, (void __iomem *)0xe6300280);
	iowrite32(0x10, (void __iomem *)0xe6300284);

	/* ARB0_CNF - enable generic arbiter */
	iowrite32(0xc0000000, (void __iomem *)0xe6700108);

	/* enable WLAN arbiter */
	iowrite32(0x80000006, (void __iomem *)0xe6700140);

#ifdef IWL_MIPI_IDI
	/* enable IDI arbiter for all channels - this code is
	 * needed in case we'd like to look on IDI bus logs
	 * via MIPI
	 */
	iowrite32(0xB0000004, (void __iomem *)0xe6700124);
	iowrite32(0xC0000000, (void __iomem *)0xe6700128);
#endif
}

static void iwl_idi_enable_lauterbach(void)
{
	/* ABB_CguLFME_AP_Register*/
	iowrite32(0x0, (void __iomem *)0xE6401170);

	/* ABB_CguLFME_CP_Register*/
	iowrite32(0x0, (void __iomem *)0xE6401174);

	/* ABB_CGU_Debug_Trace_Clock_Control_Register*/
	iowrite32(0x00030202, (void __iomem *)0xE640110C);
}

static int iwl_idi_reset(struct idi_peripheral_device *pdev)
{
	struct iwl_idi_platdata *platdata = dev_get_platdata(&pdev->device);

	if (!platdata || !platdata->core_rst)
		return -EINVAL;

	return reset_control_reset(platdata->core_rst);
}

static int iwl_idi_set_power_state(struct iwl_trans *trans,
				   enum iwl_idi_power_state state,
				   bool reset)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	int ret = 0;

	if (trans_idi->power_state == state)
		return 0;

	switch (state) {
	case PM_ENABLE:
		ret = idi_set_power_state(trans_idi->pdev,
					  trans_idi->idi_pm_handlers[state],
					  true);
		if (ret)
			break;

		if (reset) {
			ret = iwl_idi_reset(trans_idi->pdev);
			if (ret)
				break;
		}
		mdelay(1);

		if (IWL_IDI_LHP_FLAGS & IWL_IDI_DBG_JTAG)
			iwl_idi_enable_lauterbach();

		if (IWL_IDI_LHP_FLAGS & IWL_IDI_DBG_ST_ARBITER)
			iwl_idi_config_st_arbiter();

		break;
	case PM_IDLE:
		WARN_ON(trans_idi->power_state != PM_ENABLE);

		ret = idi_set_power_state(trans_idi->pdev,
					  trans_idi->idi_pm_handlers[state],
					  false);
		break;
	case PM_DISABLE:
		ret = idi_set_power_state(trans_idi->pdev,
					  trans_idi->idi_pm_handlers[state],
					  false);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	WARN_ONCE(ret != 0, "power state ret: %d", ret);
	if (ret == 0)
		trans_idi->power_state = state;
	return ret;
}

static void iwl_idi_irq_handle_error(struct iwl_trans *trans)
{
	struct iwl_trans_slv *trans_slv = IWL_TRANS_GET_SLV_TRANS(trans);

#if 0
	iwl_idi_dump_csr(trans);
	iwl_dump_fh(trans, NULL);
#endif

	/* FIXME: add local_bh_disable(); when moving to threaded irq */
	iwl_trans_fw_error(trans);

	clear_bit(STATUS_SYNC_HCMD_ACTIVE, &trans->status);
	wake_up(&trans_slv->wait_command_queue);
}

static inline void iwl_idi_disable_interrupts(struct iwl_trans *trans)
{
	unsigned long flags;
	clear_bit(STATUS_INT_ENABLED, &trans->status);

	if (!iwl_trans_grab_nic_access(trans, false, &flags)) {
		IWL_ERR(trans, "Failed to wake up NIC\n");
		return;
	}

	idi_al_write(trans, WLAN_IRQ_O_MASK_REG, WLAN_IRQ_O_MASK_IRQS);

	/* acknowledge/clear/reset any interrupts still pending
	 * from uCode or flow handler (Rx/Tx DMA) */
	idi_al_write(trans, IDI_INTA_CSR_W1C, 0xffffffff);
	idi_al_write(trans, IDI_FH_INT_CSR_W1C, 0xffffffff);

	iwl_trans_release_nic_access(trans, &flags);

	IWL_DEBUG_ISR(trans, "Disabled interrupts\n");
}

static inline void iwl_idi_clear_irq_register(struct iwl_trans *trans)
{
	unsigned long flags;

	if (!iwl_trans_grab_nic_access(trans, false, &flags)) {
		IWL_ERR(trans, "Failed to wake up NIC\n");
		return;
	}

	idi_al_write(trans, WLAN_IRQ_O_MASK_REG, WLAN_IRQ_O_MASK_IRQS);
	idi_al_write(trans, WLAN_IRQ_O_REG, WLAN_IRQ_O_REG_CLEAR_MSK);
	idi_al_write(trans, WLAN_IRQ_O_MASK_REG, WLAN_IRQ_O_ENABLE_IRQS);

	iwl_trans_release_nic_access(trans, &flags);
}

static void iwl_idi_enable_wlan_irqs(struct iwl_trans *trans)
{
	unsigned long flags;

	IWL_DEBUG_ISR(trans, "Enabling interrupts\n");
	set_bit(STATUS_INT_ENABLED, &trans->status);

	if (!iwl_trans_grab_nic_access(trans, false, &flags)) {
		IWL_ERR(trans, "Failed to wake up NIC\n");
		return;
	}

	/* unmask interrupts from the AL layer */
	idi_al_write(trans, WLAN_IRQ_O_MASK_REG, WLAN_IRQ_O_ENABLE_IRQS);

	/* enable INTA interrupts */
	idi_al_write(trans, IDI_IRQ_VIA6_LEGACY_INTA_MSK_REG, 0xffffffff);
	idi_al_write(trans, IDI_INTA_CSR_MASK, IDI_CSR_INI_SET_MASK);
	/* Rx interrupts from the FH are not enabled: In LhP we use an
	 * interrupt from the IDI bus for that purpose. */
	idi_al_write(trans, FH_INT_CSR_MASK,
		     IDI_CSR_FH_INT_TX_MASK | IDI_CSR_FH_INT_BIT_ERR);

	iwl_trans_release_nic_access(trans, &flags);
}

static void idi_irq_inta_handler(struct iwl_trans *trans, u32 inta)
{
	/* Note: Some bits in CSR_INT are "OR" of bits in CSR_FH_INT_STATUS */

	/* Service all interrupt bits. */
	if (inta & IDI_CSR_INTA_BIT_HW_ERR) {
		IWL_ERR(trans, "Hardware error detected.\n");
		iwl_trans_idi_al_write(trans, IDI_FH_INT_CSR_W1C,
				       IDI_CSR_FH_INT_BIT_ERR);
		iwl_idi_disable_interrupts(trans);
		iwl_idi_irq_handle_error(trans);
	}

	if (inta & IDI_CSR_INTA_BIT_CT_KILL) {
		/* Chip got too hot and stopped itself */
		IWL_ERR(trans, "Microcode CT kill error detected.\n");
	}

	if (inta & IDI_CSR_INTA_BIT_SW_ERR) {
		/* Error detected by uCode */
		IWL_ERR(trans, "uCode SW error detected: 0x%x\n", inta);
		iwl_idi_irq_handle_error(trans);
	}

	if (inta & IDI_CSR_INTA_BIT_ALIVE) {
		/* Alive interrupt */
		IWL_DEBUG_ISR(trans, "Alive interrupt\n");

		/*
		 * for some reason, at this point the INTA_CSR_MASK is
		 * overridden with 0x7f, leading to some interrupts
		 * (e.g. SYSASSERT) being masked. workaround it by
		 * writing the correct mask again.
		 */
		iwl_trans_idi_al_write(trans, IDI_INTA_CSR_MASK,
				       IDI_CSR_INI_SET_MASK);
	}

	if (inta & IDI_CSR_INTA_BIT_WAKEUP) {
		/* uCode wakes up after power-down sleep */
		IWL_DEBUG_ISR(trans, "Wakeup interrupt\n");

		/* no need to wake up since in this case the bus is idle and
		 * the wake up was done in irq_thread
		 */
	}

	if (inta & IDI_CSR_INTA_BIT_FH_TX) {
		/* This "Tx" DMA channel is used only for loading uCode */
		IWL_DEBUG_ISR(trans, "uCode load interrupt\n");

		iwl_idi_load_fw_wake_wq(trans);
		iwl_trans_idi_al_write(trans, IDI_FH_INT_CSR_W1C,
					IDI_CSR_FH_INT_TX_MASK);
	}

	/* Ack */
	iwl_trans_idi_al_write(trans, IDI_INTA_CSR_W1C, 0xffffffff);
}

static void iwl_idi_irq_process(struct iwl_trans *trans)
{
	int called_grp, called;

	/* Read IRQ group (level 2) */
	called_grp = iwl_trans_idi_al_read(trans, IDI_IRQ_STT_IDI_CLK_REG);
	IWL_DEBUG_ISR(trans, "called group: 0x%x\n", called_grp);

	if (called_grp & IDI_IRQ_STT_READY_CODE_LOAD_BIT) {
		/* VIA0 status register not contains further information,
		 * no need to read it. */
		IWL_DEBUG_ISR(trans,
			      "received READY_FOR_CODE_LOAD interrupt\n");
	}

	if (called_grp & IDI_IRQ_STT_AL_DEV_ERR_BIT) {
		called = iwl_trans_idi_al_read(trans,
					       IDI_IRQ_VIA1_AL_DEV_ERR_STT_REG);
		IWL_DEBUG_ISR(trans,
			      "received interrupt in group 1 (DEVICE ERROR): 0x%x\n",
			      called);
	}

	if (called_grp & IDI_IRQ_STT_LMAC_DEV_ERR_BIT) {
		called = iwl_trans_idi_al_read(trans,
					IDI_IRQ_VIA2_LMAC_DEV_ERR_STT_REG);
		IWL_DEBUG_ISR(trans,
			      "received interrupt in group 2 (LMAC ERROR): 0x%x\n",
			      called);
	}

	if (called_grp & IDI_IRQ_STT_HOST_WKUP_BIT) {
		called = iwl_trans_idi_al_read(trans,
					       IDI_IRQ_VIA3_HOST_WKUP_STT_REG);
		IWL_DEBUG_ISR(trans,
			      "received interrupt in group 3 (WKUP STT): 0x%x\n",
			      called);
	}

	if (called_grp & IDI_IRQ_STT_AL_GEN_BIT) {
		called = iwl_trans_idi_al_read(trans,
					       IDI_IRQ_VIA4_AL_GEN_STT_REG);
		IWL_DEBUG_ISR(trans,
			      "received interrupt in group 4 (AL GEN STT): 0x%x\n",
			      called);
	}

	if (called_grp & IDI_IRQ_STT_LMAC_GEN_BIT) {
		called = iwl_trans_idi_al_read(trans,
					       IDI_IRQ_VIA5_LMAC_GEN_STT_REG);
		IWL_DEBUG_ISR(trans,
			      "received interrupt in group 5 (LMAC GEN STT): 0x%x\n",
			      called);
	}

	if (called_grp & IDI_IRQ_STT_LEGACY_INTA_BIT) {
		/* VIA6 status register contains the same information as the
		 * IDI_INTA register, so no need to read it. */
		called = iwl_trans_idi_al_read(trans, IDI_INTA_CSR_W1C);
		IWL_DEBUG_ISR(trans,
			      "received interrupt in group 6. IDI_INTA: 0x%x\n",
			      called);
		idi_irq_inta_handler(trans, called);
	}

	if (called_grp & IDI_IRQ_STT_AL_INTERRUPT_BIT) {
		called = iwl_trans_idi_al_read(trans,
					       IDI_IRQ_VIA7_AL_INTER_STT_REG);
		IWL_DEBUG_ISR(trans,
			      "received interrupt in group 7 (AL INTR): 0x%x\n",
			      called);
	}

	if (called_grp & IDI_IRQ_STT_DMA_BIT) {
		called = iwl_trans_idi_al_read(trans, IDI_IRQ_VIA8_DMA_STT_REG);
		IWL_DEBUG_ISR(trans,
			      "received interrupt in group 8 (DMA): 0x%x\n",
			      called);
	}

	iwl_idi_clear_irq_register(trans);
}

static irqreturn_t idi_irq_thread(int irq, void *data)
{
	struct iwl_trans *trans = (struct iwl_trans *)data;
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	int ret;

	IWL_DEBUG_ISR(trans, "IRQ thread was called\n");

	mutex_lock(&trans_idi->irq_mutex);

	if (trans_idi->power_state == PM_IDLE) {
		/* enable bus in order to be able to access registers,
		 * even before the D0i3 wake up completes
		 */
		ret = iwl_idi_set_power_state(trans, PM_ENABLE, false);
		if (WARN_ON_ONCE(ret)) {
			mutex_unlock(&trans_idi->irq_mutex);
			return IRQ_HANDLED;
		}
	}

	/*
	 * If the trans is idle or going idle (i.e. after the fw
	 * entered d0i3, but before we handled the response and
	 * set the bus to idle) - get out of d0i3.
	 */
	if (test_bit(STATUS_TRANS_GOING_IDLE, &trans->status) ||
	    test_bit(STATUS_TRANS_IDLE, &trans->status)) {
		/*
		 * Get out of D0i3 for a grace-period. The FW will
		 * immediately send us RX once we're in D0. No need
		 * to hold the ref. Need to do it here since we enabled the bus.
		 */
		iwl_trans_slv_ref(trans);
		iwl_trans_slv_unref(trans);
	}

	iwl_idi_irq_process(trans);
	mutex_unlock(&trans_idi->irq_mutex);

	return IRQ_HANDLED;
}

static irqreturn_t idi_irq_handler(int irq, void *data)
{
	struct iwl_trans *trans = (struct iwl_trans *)data;

	/* indicate wakeup event if the trans is idle */
	if (test_bit(STATUS_TRANS_GOING_IDLE, &trans->status) ||
	    test_bit(STATUS_TRANS_IDLE, &trans->status))
		pm_wakeup_event(trans->dev, 0);

	/* when an interrupt is received the bus can be in idle state and
	 * set_power_state cannot be called form atomic context, So, move
	 * interrupt processing to a thread.
	 */
	return IRQ_WAKE_THREAD;
}

static int iwl_idi_request_irqs(struct iwl_trans *trans)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	struct idi_resource *idi_res = &trans_idi->pdev->resources;
	struct resource *res;
	int ret;

	res = idi_get_resource_byname(idi_res, IORESOURCE_IRQ,
				      IWL_IDI_IRQ_NAME);
	if (!res) {
		IWL_ERR(trans, "Failed to get resource %s\n", IWL_IDI_IRQ_NAME);
		return -EINVAL;
	}

	ret = request_threaded_irq(res->start, idi_irq_handler, idi_irq_thread,
				   IRQF_SHARED | IRQF_NO_SUSPEND,
				   IWL_IDI_IRQ_NAME, trans);
	if (ret) {
		IWL_ERR(trans, "Failed in request_threaded_irq\n");
		return -EINVAL;
	}

	return 0;
}

static void iwl_idi_free_irqs(struct iwl_trans *trans)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	struct idi_resource *idi_res = &trans_idi->pdev->resources;
	struct resource *res;

	res = idi_get_resource_byname(idi_res, IORESOURCE_IRQ,
				      IWL_IDI_IRQ_NAME);
	if (!res) {
		IWL_ERR(trans, "Failed to get resource %s\n", IWL_IDI_IRQ_NAME);
		return;
	}

	free_irq(res->start, trans);
}

static void __iomem *
iwl_idi_request_io_byname(struct idi_peripheral_device *pdev,
			  const char *name, bool request)
{
	void __iomem *io;
	struct resource *res;
	struct idi_resource *idi_res = &pdev->resources;

	if (!name)
		return NULL;

	res = idi_get_resource_byname(idi_res, IORESOURCE_MEM, name);
	if (!res)
		return NULL;

	if (request && (!request_mem_region(res->start, resource_size(res),
					    dev_name(&pdev->device))))
		return NULL;

	io = ioremap(res->start, resource_size(res));
	if (!io && request)
		release_mem_region(res->start, resource_size(res));

	return io;
}

static void iwl_idi_release_io_byname(struct idi_peripheral_device *pdev,
				      const char *name, void __iomem *io,
				      bool release)
{
	struct resource *res;
	struct idi_resource *idi_res = &pdev->resources;

	if (!name)
		return;

	res = idi_get_resource_byname(idi_res, IORESOURCE_MEM, name);

	if (!res)
		return;

	iounmap(io);
	if (release)
		release_mem_region(res->start, resource_size(res));
}

#ifdef CONFIG_OF
static struct iwl_idi_platdata *iwl_idi_get_platdata(struct iwl_trans *trans)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	struct idi_peripheral_device *pdev = trans_idi->pdev;
	struct device_node *np = pdev->device.of_node;
	struct iwl_idi_platdata *platdata;
	struct device *dev = &pdev->device;

	platdata = kzalloc(sizeof(*platdata), GFP_KERNEL);
	if (!platdata) {
		IWL_ERR(trans, "Failed to allocate platform data\n");
		return NULL;
	}

	platdata->wifi_clk = of_clk_get_by_name(np, WIFI_CLK_KERNEL_NAME);
	if (IS_ERR(platdata->wifi_clk)) {
		IWL_ERR(trans, "No Wifi clock available\n");
		platdata->wifi_clk = NULL;
	}

	platdata->rtc_clk = of_clk_get_by_name(np, WIFI_CLK_RTC_NAME);
	if (IS_ERR(platdata->rtc_clk)) {
		IWL_ERR(trans, "No RTC clock available\n");
		platdata->rtc_clk = NULL;
	}

	platdata->pmu_rst = reset_control_get(dev, "pmu");
	platdata->core_rst = reset_control_get(dev, "core");

	pdev->device.platform_data = platdata;

	return platdata;
}
#else
static struct iwl_idi_platdata *iwl_idi_get_platdata(struct iwl_trans *trans)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	struct idi_peripheral_device *pdev = trans_idi->pdev;
	struct iwl_idi_platdata *platdata;

	platdata = kzalloc(sizeof(*platdata), GFP_KERNEL);
	if (!platdata) {
		IWL_ERR(trans, "Failed to allocate platform data\n");
		return NULL;
	}

	/* not needed when working w/o OF */
	platdata->wifi_clk = NULL;
	platdata->rtc_clk = NULL;
	platdata->pmu_rst = NULL;
	platdata->core_rst = reset_control_get(&pdev->device,
					       XMM_RST_ID_WLANIPRST);

	pdev->device.platform_data = platdata;

	return platdata;
}
#endif /* CONFIG_OF */

static int iwl_idi_alloc_platform_data(struct iwl_trans *trans)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	struct idi_peripheral_device *pdev = trans_idi->pdev;
	const char __maybe_unused *dev_pm_user_name;
	int ret;

	if (!iwl_idi_get_platdata(trans)) {
		IWL_ERR(trans, "Failed to get platform data\n");
		return -EINVAL;
	}

	/* I/O mapping all the AL memory */
	trans_idi->al_hw_base =
		iwl_idi_request_io_byname(pdev, IWL_IDI_AL_MEM, false);
	if (!trans_idi->al_hw_base) {
		IWL_ERR(trans, "Failed to request io for %s\n", IWL_IDI_AL_MEM);
		return -EINVAL;
	}

	ret = iwl_idi_request_irqs(trans);
	if (ret)
		goto error_irq;

#ifndef CONFIG_OF
#ifdef CONFIG_PLATFORM_DEVICE_PM
	dev_pm_user_name = idi_dev_pm_user_name(pdev);
	if (dev_pm_user_name) {
		ret = device_state_pm_set_class(&pdev->device,
						dev_pm_user_name);
		if (ret) {
			IWL_ERR(trans, "WLAN device PM registration failed\n");
			goto error_pm_class;
		}
	}
#endif
#else
	ret = idi_device_pm_set_class(pdev);
	if (ret) {
		IWL_ERR(trans, "WLAN device PM registration failed\n");
		goto error_pm_class;
	}
#endif

	trans_idi->idi_pm_handlers[PM_ENABLE] =
		(void *)idi_peripheral_device_pm_get_state_handler(pdev,
								   "enable");
	trans_idi->idi_pm_handlers[PM_DISABLE] =
		(void *)idi_peripheral_device_pm_get_state_handler(pdev,
								   "disable");
	trans_idi->idi_pm_handlers[PM_IDLE] =
		(void *)idi_peripheral_device_pm_get_state_handler(pdev,
								   "idle");
	return 0;

#ifdef CONFIG_PLATFORM_DEVICE_PM
error_pm_class:
	iwl_idi_free_irqs(trans);
#endif

error_irq:
	iwl_idi_release_io_byname(pdev, IWL_IDI_AL_MEM,
				  trans_idi->al_hw_base, true);
	return -EINVAL;
}

static struct iwl_trans *iwl_trans_idi_alloc(void *pdev_void,
					     const void *ent_void,
					     const struct iwl_cfg *cfg)
{
	struct iwl_trans *iwl_trans = NULL;
	struct iwl_trans_idi *trans_idi = NULL;
	struct idi_peripheral_device __maybe_unused *pdev =
				(struct idi_peripheral_device *)pdev_void;
	int ret = 0;

	/* Allocate IDI transport */
	iwl_trans = kzalloc(sizeof(struct iwl_trans) +
			sizeof(struct iwl_trans_slv) +
			sizeof(struct iwl_trans_idi), GFP_KERNEL);
	if (iwl_trans == NULL)
		return NULL;

	dev_set_drvdata(&pdev->device, iwl_trans);
	iwl_trans->dev = &pdev->device;
	trans_idi = IWL_TRANS_GET_IDI_TRANS(iwl_trans);
	iwl_trans->ops = &trans_ops_idi;
	iwl_trans->cfg = cfg;
	trans_idi->trans = iwl_trans;
	trans_idi->trans_tx.trans_idi = trans_idi;
	trans_idi->trans_rx.trans_idi = trans_idi;
	trans_idi->pdev = pdev;

	trans_lockdep_init(iwl_trans);

	snprintf(iwl_trans->dev_cmd_pool_name,
		 sizeof(iwl_trans->dev_cmd_pool_name),
		 "iwl_cmd_pool_IDI:%s", dev_name(iwl_trans->dev));

	/* TODO: add a few bytes for the prefix needed in IDI */
	iwl_trans->dev_cmd_headroom = IWL_IDI_TXBU_HEADROOM_SIZE;
	iwl_trans->dev_cmd_pool =
		kmem_cache_create(iwl_trans->dev_cmd_pool_name,
				  sizeof(struct iwl_device_cmd)
				  + iwl_trans->dev_cmd_headroom,
				  sizeof(void *),
				  SLAB_HWCACHE_ALIGN,
				  NULL);

	if (!iwl_trans->dev_cmd_pool)
		goto error;

	spin_lock_init(&trans_idi->reg_lock);
	mutex_init(&trans_idi->irq_mutex);

	ret = iwl_idi_alloc_platform_data(iwl_trans);
	if (ret)
		goto error;

	device_set_wakeup_capable(&pdev->device, true);

	return iwl_trans;
error:
	if (iwl_trans->dev_cmd_pool)
		kmem_cache_destroy(iwl_trans->dev_cmd_pool);
	kfree(iwl_trans);
	return NULL;
}

static int iwl_trans_idi_start_hw(struct iwl_trans *trans)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	int ret;

	clear_bit(TRANS_IDI_STATUS_HW_ACTIVE, &trans_idi->state);

	ret = iwl_idi_rx_set_channel_config(trans);
	if (ret) {
		IWL_ERR(trans, "Failed to set RX channel config, err %#x\n",
			ret);
		return ret;
	}

	ret = iwl_idi_tx_set_channel_config(trans);
	if (ret) {
		IWL_ERR(trans, "Failed to set TX channel config, error %#x\n",
			ret);
		return ret;
	}

	iwl_idi_set_power_state(trans, PM_ENABLE, true);

	ret = idi_al_init(trans);
	if (ret) {
		IWL_ERR(trans, "Failed to init AL layer, error %#x\n", ret);
		IWL_ERR(trans, "iwl_idi_set_power_state(trans, PM_DISABLE, false)\n");
		iwl_idi_set_power_state(trans, PM_DISABLE, false);
		return ret;
	}
	iwl_idi_enable_wlan_irqs(trans);

	set_bit(TRANS_IDI_STATUS_HW_ACTIVE, &trans_idi->state);

	return 0;
}

static int iwl_trans_idi_start_fw_dbg(struct iwl_trans *trans,
				      const struct fw_img *fw,
				      bool run_in_rfkill,
				      u32 fw_dbg_flags)
{
	int ret;

	ret = iwl_idi_rx_init(trans);
	if (ret) {
		IWL_ERR(trans, "Unable to init nic - rx\n");
		goto err;
	}

	ret = iwl_idi_tx_init(trans);
	if (ret) {
		IWL_ERR(trans, "Unable to init nic - tx\n");
		goto err;
	}

	ret = iwl_idi_load_given_ucode(trans, fw, fw_dbg_flags);
	if (ret) {
		IWL_ERR(trans, "Unable to init nic - fw\n");
		goto err;
	}

	set_bit(STATUS_DEVICE_ENABLED, &trans->status);
	return 0;

err:
	set_bit(STATUS_FW_ERROR, &trans->status);
	return ret;
}

static int iwl_trans_idi_start_fw(struct iwl_trans *trans,
				  const struct fw_img *fw,
				  bool run_in_rfkill)
{
	return iwl_trans_idi_start_fw_dbg(trans, fw, run_in_rfkill, 0);
}

static void iwl_trans_idi_stop_device(struct iwl_trans *trans)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);

	if (!test_and_clear_bit(TRANS_IDI_STATUS_HW_ACTIVE, &trans_idi->state))
		return;

	mutex_lock(&trans_idi->irq_mutex);
	iwl_idi_disable_interrupts(trans);
	mutex_unlock(&trans_idi->irq_mutex);

	/* stop all Tx SW tasks */
	if (test_bit(STATUS_DEVICE_ENABLED, &trans->status))
		iwl_slv_tx_stop(trans);

	iwl_idi_set_power_state(trans, PM_DISABLE, false);

	/* flash IDI RX channel */
	idi_peripheral_flush(trans_idi->pdev, IDI_PRIMARY_CHANNEL);

	/* flash IDI TX channels */
	idi_peripheral_flush(trans_idi->pdev,
			     IDI_PRIMARY_CHANNEL | IDI_TX_CHANNEL);
	idi_peripheral_flush(trans_idi->pdev,
			     IDI_SECONDARY_CHANNEL | IDI_TX_CHANNEL);

	/* now we can be sure that no new data from the bus will
	 * arrive and the only transactions that can be received are the
	 * result of the flush
	 */
	if (test_and_clear_bit(STATUS_DEVICE_ENABLED, &trans->status) ||
	    test_and_clear_bit(STATUS_FW_ERROR, &trans->status)) {
		iwl_idi_rx_stop(trans);
		iwl_idi_tx_stop(trans);
		iwl_idi_rx_free(trans);
		iwl_idi_tx_free(trans);
	}

	/* clear status bits */
	clear_bit(STATUS_SYNC_HCMD_ACTIVE, &trans->status);
	clear_bit(STATUS_INT_ENABLED, &trans->status);
}

static void iwl_trans_idi_d3_suspend(struct iwl_trans *trans, bool test)
{
	IWL_ERR(trans, "IDI d3 suspend is not implemented\n");
	/* TODO: */
}

static int iwl_trans_idi_d3_resume(struct iwl_trans *trans,
				   enum iwl_d3_status *status,
				   bool test)
{
	*status = IWL_D3_STATUS_RESET;
	IWL_ERR(trans, "IDI d3 resume is not implemented\n");
	/* TODO: */
	return 0;
}

static inline void iwl_idi_txq_set_inactive(struct iwl_trans *trans,
					    u16 txq_id)
{
	/* Simply stop the queue, but don't change any configuration;
	 * the SCD_ACT_EN bit is the write-enable mask for the ACTIVE bit. */
	iwl_write_prph(trans,
		       SCD_QUEUE_STATUS_BITS(txq_id),
		       (0 << SCD_QUEUE_STTS_REG_POS_ACTIVE) |
		       (1 << SCD_QUEUE_STTS_REG_POS_SCD_ACT_EN));
}

static int iwl_idi_txq_set_ratid_map(struct iwl_trans *trans, u16 ra_tid,
				     u16 txq_id)
{
	struct iwl_idi_trans_tx *trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);
	u32 tbl_dw_addr;
	u32 tbl_dw;
	u16 scd_q2ratid;

	scd_q2ratid = ra_tid & SCD_QUEUE_RA_TID_MAP_RATID_MSK;
	tbl_dw_addr = trans_tx->scd_base_addr +
		SCD_TRANS_TBL_OFFSET_QUEUE(txq_id);

	tbl_dw = iwl_trans_read_mem32(trans, tbl_dw_addr);

	if (txq_id & 0x1)
		tbl_dw = (scd_q2ratid << 16) | (tbl_dw & 0x0000FFFF);
	else
		tbl_dw = scd_q2ratid | (tbl_dw & 0xFFFF0000);

	iwl_trans_write_mem32(trans, tbl_dw_addr, tbl_dw);

	return 0;
}

static void iwl_trans_idi_txq_enable(struct iwl_trans *trans,
				     int txq_id, int fifo, int sta_id, int tid,
				     int frame_limit, u16 ssn)
{
	struct iwl_trans_slv *trans_slv = IWL_TRANS_GET_SLV_TRANS(trans);
	struct iwl_idi_trans_tx *trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);
	u32 val;
	unsigned long flags;

	/* Stop this Tx queue before configuring it */
	iwl_idi_txq_set_inactive(trans, txq_id);

	/* Set this queue as a chain-building queue unless it is CMD queue */
	if (txq_id != trans_slv->cmd_queue)
		iwl_set_bits_prph(trans, SCD_QUEUECHAIN_SEL, BIT(txq_id));

	/* If this queue is mapped to a certain station: it is an AGG queue */
	if (sta_id >= 0) {
		u16 ra_tid = BUILD_RAxTID(sta_id, tid);

		/* Map receiver-address / traffic-ID to this queue */
		iwl_idi_txq_set_ratid_map(trans, ra_tid, txq_id);

		/* enable aggregations for the queue */
		iwl_set_bits_prph(trans, SCD_AGGR_SEL, BIT(txq_id));
	} else {
		/*
		 * disable aggregations for the queue, this will also make the
		 * ra_tid mapping configuration irrelevant since it is now a
		 * non-AGG queue.
		 */
		iwl_clear_bits_prph(trans, SCD_AGGR_SEL, BIT(txq_id));
	}

	trans_tx->next_tfd_index[txq_id] = ssn & (TFD_QUEUE_SIZE_MAX - 1);
	iwl_trans_slv_tx_set_ssn(trans, txq_id, ssn);

	if (!iwl_trans_grab_nic_access(trans, false, &flags)) {
		IWL_ERR(trans, "Failed to wake up NIC\n");
		return;
	}

	/* Configure the shadow register of the SCD write pointer with the
	 * first TFD at index corresponding to start sequence number.
	 * Note that this write should be protected by wake up mechanism */

	idi_al_write(trans, AMFH_TG1_WR_BASE_ADDR + HBUS_TARG_WRPTR,
		     (ssn & (TFD_QUEUE_SIZE_MAX - 1)) | (txq_id << 8));

	iwl_trans_idi_write_prph(trans, SCD_QUEUE_RDPTR(txq_id), ssn);

	val = 0;
	idi_al_write_lmac_mem(trans, trans_tx->scd_base_addr +
			      SCD_CONTEXT_QUEUE_OFFSET(txq_id),
			      &val, 1);

	val = ((frame_limit << SCD_QUEUE_CTX_REG2_WIN_SIZE_POS) &
	       SCD_QUEUE_CTX_REG2_WIN_SIZE_MSK) |
	      ((frame_limit << SCD_QUEUE_CTX_REG2_FRAME_LIMIT_POS) &
	       SCD_QUEUE_CTX_REG2_FRAME_LIMIT_MSK);
	idi_al_write_lmac_mem(trans, trans_tx->scd_base_addr +
			      SCD_CONTEXT_QUEUE_OFFSET(txq_id) + sizeof(u32),
			      &val, 1);

	iwl_trans_idi_write_prph(trans, SCD_QUEUE_STATUS_BITS(txq_id),
				 (1 << SCD_QUEUE_STTS_REG_POS_ACTIVE) |
				 (fifo << SCD_QUEUE_STTS_REG_POS_TXF) |
				 (1 << SCD_QUEUE_STTS_REG_POS_WSL) |
				 SCD_QUEUE_STTS_REG_MSK);

	iwl_trans_release_nic_access(trans, &flags);

	IWL_DEBUG_TX_QUEUES(trans, "Activate queue %d on FIFO %d wr ptr: %d\n",
			    txq_id, fifo, ssn & 0xff);
}

static void iwl_trans_idi_txq_disable(struct iwl_trans *trans, int txq_id)
{
	struct iwl_idi_trans_tx *trans_tx;
	u32 stts_addr;
	static const u32 zero_data[4] = {};

	trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);

	iwl_idi_txq_set_inactive(trans, txq_id);

	stts_addr = trans_tx->scd_base_addr +
		SCD_TX_STTS_QUEUE_OFFSET(txq_id);
	iwl_trans_write_mem(trans, stts_addr, (void *)zero_data,
			    ARRAY_SIZE(zero_data));

	/* FIXME: any other configs? */

	iwl_slv_free_data_queue(trans, txq_id);

	IWL_DEBUG_TX_QUEUES(trans, "Deactivate queue %d\n", txq_id);
}

static void iwl_idi_tx_start(struct iwl_trans *trans, u32 scd_base_addr)
{
	u8 i, chnl;
	u32 reg_val;
	struct iwl_trans_slv *trans_slv = IWL_TRANS_GET_SLV_TRANS(trans);
	struct iwl_idi_trans_tx *trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);
	unsigned long flags;
	__le32 q_addr;

	if (!iwl_trans_grab_nic_access(trans, false, &flags)) {
		IWL_ERR(trans, "Failed to wake up NIC\n");
		return;
	}

	q_addr = IDI_AL_SFDB_VTFD_BASE_ADDR;
	for (i = 0; i < IDI_TX_MAX_Q_COUNT; i++) {
		iwl_trans_idi_write32(trans, FH_MEM_CBBC_QUEUE(i), q_addr >> 8);
		q_addr += IDI_AL_SFDB_VTFD_SIZE;
	}

	trans_tx->scd_base_addr = iwl_trans_idi_read_prph(trans,
							  SCD_SRAM_BASE_ADDR);

	WARN_ON(scd_base_addr != 0 &&
		scd_base_addr != trans_tx->scd_base_addr);

	iwl_trans_idi_write_prph(trans, SCD_DRAM_BASE_ADDR,
				 IDI_AL_SFDB_VIRT_BC_BASE_ADDR >> 10);

	/* Set entries count in queue to 32 TFDs */
	iwl_trans_idi_write_prph(trans, SCD_CB_SIZE, 6);

	for (chnl = 0; chnl < FH_TCSR_CHNL_NUM; chnl++)
		iwl_trans_idi_write32(trans, FH_TCSR_CHNL_TX_CONFIG_REG(chnl),
				FH_TCSR_TX_CONFIG_REG_VAL_MSG_MODE_TXF |
				FH_TCSR_TX_CONFIG_REG_VAL_DMA_CREDIT_ENABLE |
				FH_TCSR_TX_CONFIG_REG_VAL_CIRQ_HOST_NOINT |
				FH_TCSR_TX_CONFIG_REG_VAL_CIRQ_RTC_NOINT |
				FH_TCSR_TX_CONFIG_REG_VAL_DMA_CHNL_ENABLE);

	/* Update FH chicken bits */
	reg_val = iwl_trans_idi_read32(trans, FH_TX_CHICKEN_BITS_REG);
	iwl_trans_idi_write32(trans, FH_TX_CHICKEN_BITS_REG,
			      reg_val | FH_TX_CHICKEN_BITS_SCD_AUTO_RETRY_EN);

	/* configure TX max pending requests to one, to prevent
	 * fast response bug. This also disables snoop-mode, which
	 * only relevant to PCIe. */
	iwl_trans_idi_write32(trans, FH_TSSR_TX_MSG_CONFIG_REG, 0x1);

	/* write zero to SCD_AGGREGATION_EN */
	iwl_trans_idi_write_prph(trans, SCD_AGGR_SEL, 0);

	/* enable all transmit queue interrupts */
	iwl_trans_idi_write_prph(trans, SCD_INTERRUPT_MASK,
				 IWL_MASK(0, IDI_TX_MAX_Q_COUNT));

	iwl_trans_idi_write_prph(trans, SCD_ACTIVE, 0x1);

	/* enable all TX FIFOs */
	iwl_trans_idi_write_prph(trans, SCD_TXFACT, IWL_MASK(0, 7));

	iwl_trans_release_nic_access(trans, &flags);

	iwl_trans_ac_txq_enable(trans, trans_slv->cmd_queue,
				trans_slv->cmd_fifo);
	IWL_DEBUG_TX(trans, "Tx Started\n");
}

static void iwl_trans_idi_fw_alive(struct iwl_trans *trans, u32 scd_base_addr)
{
	IWL_DEBUG_FW(trans, "Got Alive.\n");
	iwl_idi_tx_start(trans, scd_base_addr);
}

static void iwl_trans_idi_free(struct iwl_trans *trans)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	struct idi_peripheral_device *pdev = trans_idi->pdev;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	/* remove pm device if it was registered */
	if (idi_dev_pm_user_name(pdev))
		device_state_pm_remove_device(&pdev->device);
#endif

	iwl_idi_free_irqs(trans);

	if (pdev && pdev->device.platform_data) {
		struct iwl_idi_platdata *platdata =
			(struct iwl_idi_platdata *)pdev->device.platform_data;

		if (platdata->core_rst)
			reset_control_put(platdata->core_rst);
		if (platdata->pmu_rst)
			reset_control_put(platdata->pmu_rst);
		kfree(platdata);
	}

	kmem_cache_destroy(trans->dev_cmd_pool);
	kfree(trans);
}

static int iwl_trans_idi_wait_tx_queue_empty(struct iwl_trans *trans)
{
	IWL_ERR(trans, "IDI wait_tx_queue_empty is not implemented\n");
	/* TODO: */
	return 0;
}

static int iwl_trans_idi_grab_bus(struct iwl_trans *trans)
{
	int ret;
	IWL_DEBUG_RPM(trans, "grab bus\n");

	ret = iwl_idi_set_power_state(trans, PM_ENABLE, false);
	if (ret)
		return ret;

	return iwl_idi_rx_resume(trans);
}

static int iwl_trans_idi_release_bus(struct iwl_trans *trans)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	int ret;

	IWL_DEBUG_RPM(trans, "release bus\n");

	ret = iwl_idi_rx_suspend(trans);
	if (ret)
		return ret;

	mutex_lock(&trans_idi->irq_mutex);
	ret = iwl_idi_set_power_state(trans, PM_IDLE, false);
	mutex_unlock(&trans_idi->irq_mutex);

	return ret;
}

static void iwl_trans_idi_configure(struct iwl_trans *trans,
			      const struct iwl_trans_config *trans_cfg)
{
	struct iwl_trans_slv *trans_slv = IWL_TRANS_GET_SLV_TRANS(trans);

	iwl_slv_set_reclaim_cmds(trans_slv, trans_cfg);
	iwl_slv_set_rx_page_order(trans_slv, trans_cfg);

	trans_slv->cmd_queue = trans_cfg->cmd_queue;
	trans_slv->cmd_fifo = trans_cfg->cmd_fifo;
	trans_slv->command_names = trans_cfg->command_names;
	trans_slv->bc_table_dword = trans_cfg->bc_table_dword;

	trans_slv->config.max_queues_num = IDI_TX_MAX_Q_COUNT;
	trans_slv->config.queue_size = IDI_TX_TFD_PER_Q;
	trans_slv->config.tb_size = IDI_TX_PAYLOAD_PAGE_SIZE;
	/* One more TXC is needed for the TFD, hence the maximum
	 * number of data descriptors is TXBU capacity minus one. */
	trans_slv->config.max_data_desc_count =
			IWL_IDI_MAX_TXC_COUNT_IN_TXBU - 1;
	trans_slv->config.hcmd_headroom = IWL_IDI_TXBU_HEADROOM_SIZE;
	trans_slv->config.policy_trigger = iwl_idi_tx_policy_trigger;
	trans_slv->config.clean_dtu = iwl_idi_tx_clean_txbu;
	trans_slv->config.free_dtu_mem = iwl_idi_tx_free_txbu_mem;
	trans_slv->config.calc_desc_num = iwl_idi_tx_calc_txcs_num;
	trans_slv->config.grab_bus = iwl_trans_idi_grab_bus;
	trans_slv->config.release_bus = iwl_trans_idi_release_bus;
	trans_slv->config.rx_dma_idle = iwl_idi_rx_dma_idle;
}

static bool
iwl_trans_idi_grab_nic_access(struct iwl_trans *trans, bool silent,
			      unsigned long *flags)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	int ret;
	spin_lock_irqsave(&trans_idi->reg_lock, *flags);
	ret = idi_al_request_access(trans, silent);

	if (unlikely(ret < 0)) {
		/* TODO: reset NIC (?) */
		if (!silent) {
			spin_unlock_irqrestore(&trans_idi->reg_lock, *flags);
			return false;
		}
	}

	/*
	 * Fool sparse by faking we release the lock - sparse will
	 * track nic_access anyway.
	 */
	__release(&trans_idi->reg_lock);

	return true;
}

static void
iwl_trans_idi_release_nic_access(struct iwl_trans *trans, unsigned long *flags)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);

	lockdep_assert_held(&trans_idi->reg_lock);

	/*
	 * Fool sparse by faking we acquiring the lock - sparse will
	 * track nic_access anyway.
	 */
	__acquire(&trans_idi->reg_lock);

	idi_al_release_access(trans);

	/*
	 * we need this write to be performed before any other writes
	 * scheduled on different CPUs (after we drop reg_lock).
	 */
	mmiowb();
	spin_unlock_irqrestore(&trans_idi->reg_lock, *flags);
}

static int iwl_trans_idi_read_mem(struct iwl_trans *trans, u32 addr, void *buf,
				  int dwords)
{
	unsigned long flags;

	if (iwl_trans_grab_nic_access(trans, false, &flags)) {
		int ret = idi_al_read_lmac_mem(trans, addr, buf, dwords);

		iwl_trans_release_nic_access(trans, &flags);
		return ret;
	}

	return -EBUSY;
}

static int iwl_trans_idi_write_mem(struct iwl_trans *trans, u32 addr,
				   const void *buf, int dwords)
{
	unsigned long flags;

	if (iwl_trans_grab_nic_access(trans, false, &flags)) {
		idi_al_write_lmac_mem(trans, addr, buf, dwords);
		iwl_trans_release_nic_access(trans, &flags);
		return 0;
	}

	return -EBUSY;
}

static void iwl_trans_idi_set_bits_mask(struct iwl_trans *trans,
					u32 reg, u32 mask, u32 value)
{
	unsigned long flags;
	u32 v;

	if (!iwl_trans_grab_nic_access(trans, false, &flags)) {
		IWL_ERR(trans, "Failed to wake up NIC\n");
		return;
	}

#ifdef CONFIG_IWLWIFI_DEBUG
	WARN_ON_ONCE(value & ~mask);
#endif
	v = iwl_read32(trans, reg);
	v &= ~mask;
	v |= value;
	iwl_write32(trans, reg, v);

	iwl_trans_release_nic_access(trans, &flags);
}

#ifdef CPTCFG_IWLWIFI_DEBUGFS
/* create and remove of files */
#define DEBUGFS_ADD_FILE(name, parent, mode) do {			\
	if (!debugfs_create_file(#name, mode, parent, trans,		\
				 &iwl_dbgfs_##name##_ops))		\
		goto err;						\
} while (0)

#define DEBUGFS_READ_WRITE_FILE_OPS(name)				\
static const struct file_operations iwl_dbgfs_##name##_ops = {		\
	.write = iwl_dbgfs_##name##_write,				\
	.read = iwl_dbgfs_##name##_read,				\
	.open = simple_open,						\
	.llseek = generic_file_llseek,					\
};

#define DEBUGFS_WRITE_FILE_OPS(name)				\
static const struct file_operations iwl_dbgfs_##name##_ops = {		\
	.write = iwl_dbgfs_##name##_write,				\
	.open = simple_open,						\
	.llseek = generic_file_llseek,					\
};


static ssize_t iwl_dbgfs_idi_bus_power_state_read(struct file *file,
						  char __user *user_buf,
						  size_t count, loff_t *ppos)
{
	struct iwl_trans *trans = file->private_data;
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	int pos = 0;
	char buf[200];

#define PRINT_POWER_STATE(state)					\
		(pos += sprintf(buf + pos, #state " (%d) %s\n", state,	\
				state == trans_idi->power_state ?	\
				"<- current state" : ""))

	PRINT_POWER_STATE(PM_ENABLE);
	PRINT_POWER_STATE(PM_IDLE);
	PRINT_POWER_STATE(PM_DISABLE);
#undef PRINT_POWER_STATE

	return simple_read_from_buffer(user_buf, count, ppos, buf, pos);
}

static ssize_t iwl_dbgfs_idi_bus_power_state_write(struct file *file,
						   const char __user *user_buf,
						   size_t count, loff_t *ppos)
{
	struct iwl_trans *trans = file->private_data;
	unsigned long power_state;
	int ret;

	ret = kstrtoul_from_user(user_buf, count, 0, &power_state);
	if (ret < 0)
		return -EINVAL;

	IWL_INFO(trans, "setting IDI power state to %ld\n", power_state);
	iwl_idi_set_power_state(trans, power_state, false);

	return count;
}

DEBUGFS_READ_WRITE_FILE_OPS(idi_bus_power_state);

static ssize_t iwl_dbgfs_inta_csr_write(struct file *file,
					const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct iwl_trans *trans = file->private_data;
	unsigned long value, flags;
	int ret;

	ret = kstrtoul_from_user(user_buf, count, 0, &value);
	if (ret < 0)
		return -EINVAL;

	if (!iwl_trans_grab_nic_access(trans, false, &flags)) {
		IWL_ERR(trans, "Failed to wake up NIC\n");
		return -EINVAL;
	}

	idi_al_write(trans, IDI_INTA_CSR_W1S, value);

	iwl_trans_release_nic_access(trans, &flags);
	return count;
}

static ssize_t iwl_dbgfs_al_read_write(struct file *file,
				       const char __user *user_buf,
				       size_t count, loff_t *ppos)
{
	struct iwl_trans *trans = file->private_data;
	struct iwl_idi_trans_tx *trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);
	char buf[64];
	int buf_size;
	unsigned int value;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, sizeof(buf) -  1);
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	if (sscanf(buf, "%x", &value) != 1)
		return -EINVAL;
	trans_tx->al_reg = value;

	return count;
}

static ssize_t iwl_dbgfs_al_read_read(struct file *file, char __user *user_buf,
				      size_t count, loff_t *ppos)
{
	struct iwl_trans *trans = file->private_data;
	struct iwl_idi_trans_tx *trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);
	char buf[256];
	int pos = 0;
	const size_t bufsz = sizeof(buf);
	unsigned long flags;

	if (!iwl_trans_grab_nic_access(trans, false, &flags)) {
		IWL_ERR(trans, "Failed to wake up NIC\n");
		return -EINVAL;
	}

	pos += scnprintf(buf + pos, bufsz - pos, "AL: 0x%x\n",
			 idi_al_read(trans, trans_tx->al_reg));

	iwl_trans_release_nic_access(trans, &flags);

	return simple_read_from_buffer(user_buf, count, ppos, buf, pos);
}

static ssize_t iwl_dbgfs_al_write_write(struct file *file,
					const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct iwl_trans *trans = file->private_data;
	char buf[64];
	int buf_size;
	unsigned int reg, value;
	unsigned long flags;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, sizeof(buf) -  1);
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	if (sscanf(buf, "%x=%x", &reg, &value) != 2)
		return -EINVAL;

	if (!iwl_trans_grab_nic_access(trans, false, &flags)) {
		IWL_ERR(trans, "Failed to wake up NIC\n");
		return -EINVAL;
	}

	idi_al_write(trans, reg, value);
	iwl_trans_release_nic_access(trans, &flags);

	return count;
}

DEBUGFS_READ_WRITE_FILE_OPS(al_read);
DEBUGFS_WRITE_FILE_OPS(al_write);
DEBUGFS_WRITE_FILE_OPS(inta_csr);

static int iwl_trans_idi_dbgfs_register(struct iwl_trans *trans,
					struct dentry *dir)
{
	DEBUGFS_ADD_FILE(idi_bus_power_state, dir, S_IWUSR | S_IRUSR);
	DEBUGFS_ADD_FILE(inta_csr, dir, S_IWUSR);
	DEBUGFS_ADD_FILE(al_read, dir, S_IRUSR | S_IWUSR);
	DEBUGFS_ADD_FILE(al_write, dir, S_IWUSR);

	return 0;

err:
	IWL_ERR(trans, "failed to create the trans debugfs entry\n");
	return -ENOMEM;
}

#else
static int iwl_trans_idi_dbgfs_register(struct iwl_trans *trans,
					struct dentry *dir)
{ return 0; }

#endif /*CPTCFG_IWLWIFI_DEBUGFS */

static const struct iwl_trans_ops trans_ops_idi = {
	.start_hw = iwl_trans_idi_start_hw,
	.fw_alive = iwl_trans_idi_fw_alive,
	.start_fw = iwl_trans_idi_start_fw,
#if IS_ENABLED(CPTCFG_IWLXVT)
	.start_fw_dbg = iwl_trans_idi_start_fw_dbg,
#endif
	.stop_device = iwl_trans_idi_stop_device,

	.d3_suspend = iwl_trans_idi_d3_suspend,
	.d3_resume = iwl_trans_idi_d3_resume,

	.send_cmd = iwl_trans_slv_send_cmd,

	.tx = iwl_trans_slv_tx_data_send,
	.reclaim = iwl_trans_slv_tx_data_reclaim,

	/* no aggregation on IDI at this stage */
	.txq_enable = iwl_trans_idi_txq_enable,
	.txq_disable = iwl_trans_idi_txq_disable,

	.dbgfs_register = iwl_trans_idi_dbgfs_register,
	.wait_tx_queue_empty = iwl_trans_idi_wait_tx_queue_empty,

	.write8 = iwl_trans_idi_write8,
	.write32 = iwl_trans_idi_write32,
	.read32 = iwl_trans_idi_read32,
	.read_prph = iwl_trans_idi_read_prph,
	.write_prph = iwl_trans_idi_write_prph,
	.read_mem = iwl_trans_idi_read_mem,
	.write_mem = iwl_trans_idi_write_mem,
	.configure = iwl_trans_idi_configure,
	.grab_nic_access = iwl_trans_idi_grab_nic_access,
	.release_nic_access = iwl_trans_idi_release_nic_access,
	.set_bits_mask = iwl_trans_idi_set_bits_mask,
	.ref = iwl_trans_slv_ref,
	.unref = iwl_trans_slv_unref,
};

static int iwl_idi_probe(struct idi_peripheral_device *pdev,
			 const struct idi_device_id *id)
{
	struct iwl_trans *iwl_trans;
	struct iwl_trans_idi *trans_idi;
	int ret;
	const struct iwl_cfg *cfg = &iwl7999_bgn_cfg;

	iwl_trans = iwl_trans_idi_alloc(pdev, id, cfg);
	if (iwl_trans == NULL)
		return -ENOMEM;

	IWL_INFO(iwl_trans, "Selected bus type = IDI\n");

	trans_idi = IWL_TRANS_GET_IDI_TRANS(iwl_trans);
	trans_idi->drv = iwl_drv_start(iwl_trans, cfg);

	trans_idi->power_state = PM_DISABLE;

	if (IS_ERR(trans_idi->drv)) {
		ret = PTR_ERR(trans_idi->drv);
		goto out_free_trans;
	}

	/* register transport layer debugfs here */
	ret = iwl_trans_slv_dbgfs_register(iwl_trans, iwl_trans->dbgfs_dir);
	if (ret)
		goto out_free_drv;

	if (iwl_ucode_alloc_dma_init(iwl_trans)) {
		printk("\n\n ***** Init uCode memory dma fail.\n");
	}

	return 0;

out_free_drv:
	iwl_drv_stop(trans_idi->drv);
out_free_trans:
	iwl_trans_idi_free(iwl_trans);
	dev_set_drvdata(&pdev->device, NULL);
	return ret;
}

static int iwl_idi_remove(struct idi_peripheral_device *pdev)
{
	struct iwl_trans *trans = dev_get_drvdata(&pdev->device);
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);

	iwl_drv_stop(trans_idi->drv);
	iwl_idi_release_io_byname(pdev, IWL_IDI_AL_MEM,
				  trans_idi->al_hw_base, false);
	iwl_trans_idi_free(trans);
	dev_set_drvdata(&pdev->device, NULL);

	return 0;
}

/* Called from idi_bus_shutdown during kernel_restart flow */
static void iwl_idi_shutdown(struct idi_peripheral_device *pdev)
{
	if (!pdev || !dev_get_drvdata(&pdev->device))
		return;

	iwl_idi_remove(pdev);
}

static int iwl_idi_suspend(struct idi_peripheral_device *pdev,
			   pm_message_t mesg)
{
	struct iwl_trans *trans = dev_get_drvdata(&pdev->device);
	struct iwl_trans_idi *trans_idi;

	if (!trans)
		goto exit;

	trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);

	/* Should never happen. We hold a wakelock while the IDI bus is used */
	if (WARN_ON(trans_idi->power_state == PM_ENABLE))
		return -EINVAL;
exit:
	return 0;
}

static int iwl_idi_resume(struct idi_peripheral_device *pdev)
{
	struct iwl_trans *trans = dev_get_drvdata(&pdev->device);
	struct iwl_trans_idi *trans_idi;

	if (!trans)
		goto exit;

	trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);

	WARN_ON(trans_idi->power_state == PM_ENABLE);
exit:
	/* Nothing to do here - bus is currently not taken */
	return 0;
}

static struct idi_peripheral_driver iwl_idi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = IWL_IDI_NAME,
	},
	.p_type = IDI_WLAN,
#ifdef CONFIG_OF
	.id_table = idi_ids,
#endif
	.probe  = iwl_idi_probe,
	.remove = iwl_idi_remove,
	.shutdown = iwl_idi_shutdown,
	.suspend = iwl_idi_suspend,
	.resume = iwl_idi_resume,
};

int iwl_idi_register_driver(void)
{
	return idi_register_peripheral_driver(&iwl_idi_driver);
}

void iwl_idi_unregister_driver(void)
{
	idi_unregister_peripheral_driver(&iwl_idi_driver);
}
