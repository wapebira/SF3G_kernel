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

#ifndef __iwl_trans_idi_h__
#define __iwl_trans_idi_h__

#include "iwl-trans.h"
#include "shared.h"
#include "idi_rx.h"
#include "idi_tx.h"

#define IWL_IDI_SG_LIST_END BIT(0)
#define IWL_IDI_SG_LIST_INT BIT(1)
#define IWL_IDI_SG_LIST_END_INT (IWL_IDI_SG_LIST_INT | IWL_IDI_SG_LIST_END)

/* define mask for irq's */
#ifndef IWL_IDI_LLS_INT
#define IWL_IDI_LLS_INT 0X1
#define IWL_IDI_HW_ERR_INT 0X2
#define IWL_IDI_SW_ERR_INT 0X4
#define IWL_IDI_FH_TX_INT 0X8
#endif

#define IWL_IDI_DBG_ST_ARBITER BIT(0)
#define IWL_IDI_DBG_GPIO BIT(1)
#define IWL_IDI_DBG_RX_TASKLET BIT(3)
#define IWL_IDI_DBG_JTAG BIT(4)

enum iwl_idi_power_state {PM_ENABLE, PM_IDLE, PM_DISABLE, PM_MAX};

struct iwl_idi_platdata {
	struct clk *wifi_clk;
	struct clk *rtc_clk;
	struct reset_control *core_rst;
	struct reset_control *pmu_rst;
};

#define TRANS_IDI_STATUS_HW_ACTIVE 0

struct iwl_trans_idi {
	struct iwl_drv *drv;
	struct iwl_trans *trans;
	struct iwl_idi_trans_rx trans_rx;
	struct iwl_idi_trans_tx trans_tx;
	struct idi_peripheral_device *pdev;
	void __iomem *al_hw_base;

	/* protect hw registers */
	spinlock_t reg_lock;

	struct mutex irq_mutex;

	/* pending irqs */
	atomic_t irq_data;

	/* IDI power state the driver has requested */
	enum iwl_idi_power_state power_state;
	void *idi_pm_handlers[PM_MAX];
	unsigned long state;
};

#define IWL_TRANS_GET_IDI_TRANS(_iwl_trans) \
	((struct iwl_trans_idi *)\
		((IWL_TRANS_GET_SLV_TRANS(_iwl_trans))->bus_specific))

#define IWL_TRANS_GET_IDI_TRANS_RX(_iwl_trans) \
	((struct iwl_idi_trans_rx *)\
		&((IWL_TRANS_GET_IDI_TRANS(_iwl_trans))->trans_rx))

#define IWL_TRANS_GET_IDI_TRANS_TX(_iwl_trans) \
	((struct iwl_idi_trans_tx *)\
		&((IWL_TRANS_GET_IDI_TRANS(_iwl_trans))->trans_tx))

#define IWL_TRANS_SLV_GET_IDI_TRANS(_slv_trans)\
	((struct iwl_trans_idi *)((_slv_trans)->bus_specific))

#define IWL_TRANS_SLV_GET_IDI_TRANS_RX(_slv_trans)\
	((struct iwl_idi_trans_rx *)\
		&((IWL_TRANS_SLV_GET_IDI_TRANS(_slv_trans))->trans_rx))

#define IWL_TRANS_SLV_GET_IDI_TRANS_TX(_slv_trans)\
	((struct iwl_idi_trans_tx *)\
		&((IWL_TRANS_SLV_GET_IDI_TRANS(_slv_trans))->trans_tx))

#define IWL_TRANS_RX_GET_TRANS(_iwl_trans_rx) \
	((struct iwl_trans *)((struct iwl_trans_idi *)\
				(_iwl_trans_rx)->trans_idi)->trans)

#define IWL_TRANS_TX_GET_TRANS(_iwl_trans_tx) \
	((struct iwl_trans *)((struct iwl_trans_idi *)\
				(_iwl_trans_tx)->trans_idi)->trans)

/* Rx handlers */
int iwl_idi_rx_init(struct iwl_trans *trans);
void iwl_idi_rx_stop(struct iwl_trans *trans);
void iwl_idi_rx_free(struct iwl_trans *trans);
int iwl_idi_rx_resume(struct iwl_trans *trans);
int iwl_idi_rx_suspend(struct iwl_trans *trans);
void iwl_idi_rx_dma_expect_idle(struct iwl_trans *trans, bool start);
void iwl_idi_rx_dma_idle(struct iwl_trans *trans);

/* Tx handlers */
int iwl_idi_tx_init(struct iwl_trans *trans);
void iwl_idi_tx_free(struct iwl_trans *trans);
int iwl_idi_tx_data_send(struct iwl_trans *trans, struct sk_buff *skb,
			 struct iwl_device_cmd *dev_cmd, int txq_id);
void iwl_idi_tx_cancel_cmd(struct iwl_trans *trans, int cmd_idx);

#endif

