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

#include "iwl-io.h"
#include "idi_internal.h"
#include "idi_tx.h"
#include "idi_tx_policy.h"
#include "idi_al.h"

#define IDI_CMD_QUOTA 2
#define IWL_IDI_TX_POLICY_WQ_FLAGS (WQ_HIGHPRI | WQ_UNBOUND | WQ_NON_REENTRANT)
#define IDI_CMD_TFD_QUOTA 1
#define IDI_CMD_TB_QUOTA 3

static inline
bool iwl_idi_tx_bus_agg_full(struct iwl_idi_trans_tx *trans_tx, u8 idx)
{
	return trans_tx->txbus_added_policy[idx] >=
		trans_tx->sg_list_meta[idx].max_txbus;
}

static inline
bool iwl_idi_tx_bus_agg_empty(struct iwl_idi_trans_tx *trans_tx, u8 idx)
{
	return trans_tx->txbus_added_policy[idx] == 0;
}

/**
 * iwl_idi_tx_policy_check_alloc - check according to available
 * and required resources that allocation request can be satisfied.
 * Since policy is only an algorithm and doesn't manages or holds any
 * resources, it should receive all the data required for the decision.
 * This function is usually called by transport.
 * @trans - the transport
 * @avail_tfds - TFDs available in transport
 * @avail_tbs - TBs available in transport
 * @req_tfds - how much TFDs are requested
 * @req_tbs - how much TBs are requested
 * Retuns true if the resources can be allocated, false otherwise.
 */
bool iwl_idi_tx_policy_check_alloc(struct iwl_trans_slv *trans_slv,
				   u8 txq_id,
				   int avail_tfds, int avail_tbs,
				   int req_tfds, int req_tbs)
{
	if (txq_id != trans_slv->cmd_queue) {
		avail_tfds -= IDI_CMD_TFD_QUOTA;
		avail_tbs -= IDI_CMD_TB_QUOTA;
	}

	if ((req_tfds <= avail_tfds) && (req_tbs <= avail_tbs))
		return true;

	return false;
}

/*
 * if there are no aggragations to channel, clears loaded bit. else
 *- calls for dma arm.
 */
static void
iwl_idi_tx_policy_dma_arm_chan(struct iwl_idi_trans_tx *trans_tx, u8 idx)
{
	int ret;
	struct iwl_trans *trans = IWL_TRANS_TX_GET_TRANS(trans_tx);

	if ((iwl_idi_tx_bus_agg_empty(trans_tx, idx))) {
		clear_bit(idx, &trans_tx->sg_list_loaded);
		wake_up(&trans_tx->stop_waitq);
		return;

	} else {
		/* if reached this point, bus aggregation should be sent */
		iwl_idi_tx_close_sg_list(trans_tx, idx);
		ret = iwl_idi_tx_dma_arm(trans_tx, idx);
		if (unlikely(ret)) {
			IWL_ERR(trans,
				"[%d] %s: iwl_idi_tx_dma_arm failed, ret %d\n",
				get_current()->tgid, __func__, ret);
			return;
		}

		IWL_DEBUG_TX(trans, "TXBU size %d, ret %d",
			     trans_tx->txbus_added_policy[idx], ret);
		trans_tx->txbus_added_policy[idx] = 0;
	}
}

void iwl_idi_tx_policy_trigger(struct work_struct *data)
{
	struct iwl_trans_slv *trans_slv =
			container_of(data, struct iwl_trans_slv,
				     policy_trigger);
	struct iwl_trans *trans = IWL_TRANS_SLV_GET_IWL_TRANS(trans_slv);
	struct iwl_idi_trans_tx *trans_tx =
		IWL_TRANS_SLV_GET_IDI_TRANS_TX(trans_slv);
	u8 idx = IWL_IDI_CHAN_TO_SG_IDX(IDI_PRIMARY_CHANNEL);
	int ret, qidx;

	if ((test_and_set_bit(idx, &trans_tx->sg_list_loaded)))
			return;

	/* don't continue to the next queue if no resources */
	while (!iwl_idi_tx_bus_agg_full(trans_tx, idx)) {
		qidx = iwl_slv_get_next_queue(trans_slv);
		if (qidx < 0)
			break;

		ret = iwl_idi_tx_add_burst(trans_slv, idx, qidx);
		IWL_DEBUG_TX(trans, "add_burst return %d\n", ret);

		if (ret < 0)
			break;

		trans_tx->txbus_added_policy[idx]++;
	}

	iwl_idi_tx_policy_dma_arm_chan(trans_tx, idx);
}
