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

#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/gfp.h>

#include "iwl-io.h"
#include "iwl-op-mode.h"
#include "iwl-trans.h"
#include "iwl-csr.h"
#include "iwl-fh.h"
#include "shared.h"
#include "idi_internal.h"
#include "idi_utils.h"
#include "idi_hals.h"
#include "idi_constants.h"

#define IWL_IDI_RX_RES_NAME "rx fifo"

#define IDI_RX_LL_DONE(next) (((next) & cpu_to_le32(IWL_IDI_SG_LIST_END)) || \
			      ((next) & cpu_to_le32(IWL_IDI_SG_LIST_INT)))

static void iwl_idi_rx_process_pages(struct work_struct *data);
static void iwl_idi_rx_sg_process(struct iwl_idi_trans_rx *trans_rx);
static void iwl_idi_rx_handler_kick0(struct iwl_idi_trans_rx *trans_rx);
static void iwl_idi_rx_handler_data(struct iwl_idi_trans_rx *trans_rx);
static void iwl_idi_rx_sm_move(struct iwl_idi_trans_rx *trans_rx);

static void iwl_idi_rx_complete(struct idi_transaction *transaction)
{
	struct iwl_trans *iwl_trans =
		dev_get_drvdata(&transaction->peripheral->device);
	struct iwl_idi_trans_rx *trans_rx =
		IWL_TRANS_GET_IDI_TRANS_RX(iwl_trans);

	if (transaction->status == IDI_STATUS_FLUSH) {
		clear_bit(IWL_IDI_RX_SM_MISC_DMA_ARMED, &trans_rx->sm.misc);
		wake_up(&trans_rx->stop_waitq);
		return;
	}

	if (WARN_ON_ONCE(transaction->status != IDI_STATUS_COMPLETE))
		return;

	set_bit(IWL_IDI_RX_SM_MISC_PROCESSING, &trans_rx->sm.misc);

	if (trans_rx->sm.state == IWL_IDI_RX_STOPPED)
		goto out;

	if (trans_rx->sm.state == IWL_IDI_RX_KICK0)
		iwl_idi_rx_handler_kick0(trans_rx);
	else
		iwl_idi_rx_handler_data(trans_rx);

out:
	clear_bit(IWL_IDI_RX_SM_MISC_PROCESSING, &trans_rx->sm.misc);
	wake_up(&trans_rx->stop_waitq);
}

int iwl_idi_rx_set_channel_config(struct iwl_trans *trans)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	struct idi_peripheral_device *pdev = trans_idi->pdev;
	struct idi_channel_config conf = {0};
	struct idi_resource *idi_res;
	struct resource *res;
	int ret;

	idi_res = &pdev->resources;

	res = idi_get_resource_byname(idi_res, IORESOURCE_MEM,
				      IWL_IDI_RX_RES_NAME);
	if (!res) {
		IWL_ERR(trans, "failed to get Rx resource\n");
		return -EINVAL;
	}

	conf.tx_or_rx = 0;
	conf.priority = IDI_NORMAL_PRIORITY;
	conf.channel_opts = IDI_PRIMARY_CHANNEL;
	conf.dst_addr = res->start;
	conf.hw_fifo_size = resource_size(res);

	ret = idi_set_channel_config(pdev, &conf);
	if (ret)
		IWL_ERR(trans, "failed setting channel config for Rx, err %d\n",
			ret);

	return ret;
}

static int iwl_idi_rx_alloc_transaction(struct iwl_idi_trans_rx *trans_rx)
{
	struct iwl_trans *trans = IWL_TRANS_RX_GET_TRANS(trans_rx);

	trans_rx->transaction = idi_alloc_transaction(GFP_KERNEL);
	if (!trans_rx->transaction) {
		IWL_ERR(trans, "%s failed to allocate IDI transaction\n",
			__func__);
		return -ENOMEM;
	}

	trans_rx->transaction->complete = iwl_idi_rx_complete;
	trans_rx->transaction->idi_xfer.channel_opts = IDI_PRIMARY_CHANNEL;

	return 0;
}

static inline
struct page *iwl_idi_alloc_pages(gfp_t mask, u32 page_order)
{
	if (page_order > 0)
		mask |= __GFP_COMP;
	return alloc_pages(mask, page_order);
}

static
struct iwl_rx_mem_buffer *iwl_idi_alloc_rxb(struct iwl_idi_trans_rx *trans_rx,
					    gfp_t flags)
{
	struct iwl_trans *trans = IWL_TRANS_RX_GET_TRANS(trans_rx);
	struct iwl_trans_slv *trans_slv = IWL_TRANS_GET_SLV_TRANS(trans);

	dma_addr_t aligned_addr;
	struct iwl_rx_mem_buffer *rxb;

	rxb = kmem_cache_alloc(trans_rx->rxb_pool, flags);
	if (unlikely(!rxb))
		return NULL;

	rxb->page = iwl_idi_alloc_pages(flags, trans_slv->rx_page_order);
	if (!rxb->page) {
		kmem_cache_free(trans_rx->rxb_pool, rxb);
		return NULL;
	}

	rxb->page_dma = dma_map_page(trans->dev,
				     rxb->page, 0,
				     PAGE_SIZE << trans_slv->rx_page_order,
				     DMA_FROM_DEVICE);
	if (dma_mapping_error(trans->dev, rxb->page_dma)) {
		IWL_ERR(trans, "Failed to map DMA page.\n");
		__free_pages(rxb->page, trans_slv->rx_page_order);
		kmem_cache_free(trans_rx->rxb_pool, rxb);
		return NULL;
	}

	aligned_addr = ALIGN(rxb->page_dma, 4);
	WARN_ON(aligned_addr != rxb->page_dma);

	return rxb;
}

static void iwl_idi_rx_free_rxb(struct iwl_idi_trans_rx *trans_rx,
			struct iwl_rx_mem_buffer *rxb)
{
	struct iwl_trans *trans = IWL_TRANS_RX_GET_TRANS(trans_rx);
	struct iwl_trans_slv *trans_slv = IWL_TRANS_GET_SLV_TRANS(trans);

	if (rxb == NULL)
		return;

	if (rxb->page) {
		dma_unmap_page(trans->dev,
			       rxb->page_dma,
			       PAGE_SIZE << trans_slv->rx_page_order,
			       DMA_FROM_DEVICE);
		__free_pages(rxb->page, trans_slv->rx_page_order);
	}
	kmem_cache_free(trans_rx->rxb_pool, rxb);
}

static inline void iwl_idi_rx_free_fifo_rxbs(struct iwl_idi_trans_rx *trans_rx,
				      struct kfifo *fifo)
{
	int count;
	struct iwl_rx_mem_buffer *rxb;

	while ((count = kfifo_out(fifo, (void *)&rxb, sizeof(void *))) != 0)
		iwl_idi_rx_free_rxb(trans_rx, rxb);
}

static inline void
iwl_idi_rx_free_fifo_and_rxbs(struct iwl_idi_trans_rx *trans_rx,
			      struct kfifo *fifo)
{
	iwl_idi_rx_free_fifo_rxbs(trans_rx, fifo);
	kfifo_free(fifo);
}

static
struct iwl_rx_mem_buffer *
iwl_idi_rx_page_pool_get(struct iwl_idi_trans_rx *trans_rx)
{
	struct iwl_rx_mem_buffer *rxb;
	int ret;

	ret = kfifo_out(&trans_rx->page_fifo, (void *)&rxb, sizeof(void *));
	if (!ret)
		rxb = NULL;

	if ((kfifo_len(&trans_rx->page_fifo) <
		IWL_IDI_RX_BG_POOL_TRSHLD*sizeof(void *))) {
		queue_work(trans_rx->wq, &trans_rx->page_refill_wrk);
	}

	/* try to allocate with GFP_ATOMIC in case of temporary empty pool */
	if (rxb == NULL)
		rxb = iwl_idi_alloc_rxb(trans_rx, GFP_ATOMIC);

	return rxb;
}

/**
 * iwl_idi_rx_page_pool_alloc - allocate page pool
 * @trans_rx - rx transport
 * Returns 0 only if succeeded to allocate fifo.
 * Otherwise returns -ENOMEM and frees all the allocated resources.
 */
static int iwl_idi_rx_page_pool_alloc(struct iwl_idi_trans_rx *trans_rx)
{
	struct iwl_trans *trans = IWL_TRANS_RX_GET_TRANS(trans_rx);

	if (kfifo_alloc(&trans_rx->page_fifo,
			IWL_IDI_RX_BG_POOL_SIZE * sizeof(void *),
			GFP_KERNEL)) {
		IWL_ERR(trans, "kfifo_alloc failed");
		return -ENOMEM;
	}
	kfifo_reset(&trans_rx->page_fifo);

	return 0;
}

static int iwl_idi_rx_page_pool_refill(struct iwl_idi_trans_rx *trans_rx)
{
	struct iwl_rx_mem_buffer *rxb;
	struct iwl_trans *trans = IWL_TRANS_RX_GET_TRANS(trans_rx);

	while (kfifo_len(&trans_rx->page_fifo) <
			 IWL_IDI_RX_BG_POOL_SIZE*sizeof(void *)) {
		rxb = iwl_idi_alloc_rxb(trans_rx, GFP_KERNEL);
		if (!rxb) {
			IWL_WARN(trans, "refill bg page pool failed. len: %d",
				 kfifo_len(&trans_rx->page_fifo));
			/* We don't free the filled rxbs. A partial refill
			 * might also be ok. */
			return -ENOMEM;
		}

		kfifo_in(&trans_rx->page_fifo, (void *)&rxb, sizeof(void *));
	}

	return 0;
}

static void iwl_idi_rx_page_pool_refill_work(struct work_struct *data)
{
	struct iwl_idi_trans_rx *trans_rx =
		container_of(data, struct iwl_idi_trans_rx, page_refill_wrk);

	iwl_idi_rx_page_pool_refill(trans_rx);
}

static
struct iwl_idi_sg_list_holder *iwl_rx_get_sg_list(struct list_head *sg_lists)
{
	struct iwl_idi_sg_list_holder *cur_sg_list;

	cur_sg_list = list_first_entry(sg_lists, struct iwl_idi_sg_list_holder,
				       list);
	list_del(&cur_sg_list->list);

	return cur_sg_list;
}

static void iwl_idi_rx_free_sg_list_data(struct iwl_trans *trans,
			struct iwl_idi_sg_list_holder *sg_list_holder)
{
	struct iwl_idi_trans_rx *trans_rx = IWL_TRANS_GET_IDI_TRANS_RX(trans);
	int i;

	if (sg_list_holder == NULL)
		return;

	for (i = 0; i < SG_LIST_MAX_SIZE; i++)
		iwl_idi_rx_free_rxb(trans_rx, sg_list_holder->addr_map[i]);

	iwl_idi_free_sg_list(trans, &sg_list_holder->sg_list);
	kfree(sg_list_holder);
}

static void iwl_idi_rx_free_sg_lists(struct iwl_trans *trans)
{
	struct iwl_idi_trans_rx *trans_rx = IWL_TRANS_GET_IDI_TRANS_RX(trans);
	struct iwl_idi_sg_list_holder *cur_sg_list, *tmp;

	if (unlikely(trans_rx == NULL))
		return;

	list_for_each_entry_safe(cur_sg_list, tmp, &trans_rx->free_sg_lists,
				 list)
		iwl_idi_rx_free_sg_list_data(trans, cur_sg_list);

	list_for_each_entry_safe(cur_sg_list, tmp, &trans_rx->used_sg_lists,
				 list)
		iwl_idi_rx_free_sg_list_data(trans, cur_sg_list);

	iwl_idi_rx_free_sg_list_data(trans, trans_rx->armed_sg_list);
	iwl_idi_free_sg_list(trans, &trans_rx->sg_list_kick0);
}

int iwl_idi_rx_suspend(struct iwl_trans *trans)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	struct iwl_idi_trans_rx *trans_rx = IWL_TRANS_GET_IDI_TRANS_RX(trans);
	int ret;

	WARN_ON_ONCE(!test_bit(IWL_IDI_RX_SM_MISC_DMA_IDLE, &trans_rx->sm.misc));

	/* flush IDI RX channel */
	ret = idi_peripheral_flush(trans_idi->pdev, IDI_PRIMARY_CHANNEL);
	if (ret) {
		IWL_ERR(trans, "Failed to set flush Rx on suspend, err %x\n",
			ret);
		return ret;
	}

	return 0;
}

int iwl_idi_rx_resume(struct iwl_trans *trans)
{
	struct iwl_idi_trans_rx *trans_rx = IWL_TRANS_GET_IDI_TRANS_RX(trans);
	int ret;

	ret = iwl_idi_rx_set_channel_config(trans);
	if (ret) {
		IWL_ERR(trans, "Failed to set RX channel config, err %x\n",
			ret);
		return ret;
	}

	WARN_ON_ONCE(test_bit(IWL_IDI_RX_SM_MISC_DMA_EXPECT_IDLE,
			      &trans_rx->sm.misc));
	clear_bit(IWL_IDI_RX_SM_MISC_DMA_IDLE, &trans_rx->sm.misc);
	iwl_idi_rx_sm_move(trans_rx);

	return 0;
}

/**
* iwl_idi_rx_stop - should be called from stop_device
*/
void iwl_idi_rx_stop(struct iwl_trans *trans)
{
	struct iwl_idi_trans_rx *trans_rx =
			IWL_TRANS_GET_IDI_TRANS_RX(trans);

	if (!wait_event_timeout(trans_rx->stop_waitq,
				!test_bit(IWL_IDI_RX_SM_MISC_DMA_ARMED,
					  &trans_rx->sm.misc) &&
				!test_bit(IWL_IDI_RX_SM_MISC_PROCESSING,
					  &trans_rx->sm.misc), 2*HZ)) {
		WARN_ONCE(1, "Failed to stop Rx");
		return;
	}

	flush_workqueue(trans_rx->wq);
	trans_rx->sm.state = IWL_IDI_RX_STOPPED;
}

/**
* iwl_idi_rx_free - implements rx_free callback.
*
*/
void iwl_idi_rx_free(struct iwl_trans *trans)
{
	struct iwl_idi_trans_rx *trans_rx;

	/* sanity */
	BUG_ON(trans == NULL);
	trans_rx = IWL_TRANS_GET_IDI_TRANS_RX(trans);
	BUG_ON(trans_rx == NULL);
	if (trans_rx->trans_idi == NULL)
		return;

	if (trans_rx->sm.state != IWL_IDI_RX_STOPPED) {
		IWL_WARN(trans,
			 "An attempt to free Rx transport when not STOPPED\n");
		return;
	}

	cancel_work_sync(&trans_rx->page_proc_wrk);
	cancel_work_sync(&trans_rx->page_refill_wrk);
	iwl_idi_rx_free_sg_lists(trans);
	iwl_idi_rx_free_fifo_and_rxbs(trans_rx, &trans_rx->ready_pages_fifo);
	iwl_idi_rx_free_fifo_and_rxbs(trans_rx, &trans_rx->page_fifo);

	if (trans_rx->rxb_pool) {
		kmem_cache_destroy(trans_rx->rxb_pool);
		trans_rx->rxb_pool = NULL;
	}

	if (trans_rx->wq) {
		destroy_workqueue(trans_rx->wq);
		trans_rx->wq = NULL;
	}

	if (trans_rx->transaction)
		idi_free_transaction(trans_rx->transaction);
}

static void iwl_idi_rx_sg_list_reset_kick0(struct iwl_idi_trans_rx *trans_rx)
{
	struct idi_sg_desc *virt;

	virt = (struct idi_sg_desc *)trans_rx->sg_list_kick0.addr;

	virt->next = cpu_to_le32(IWL_IDI_SG_LIST_END | IWL_IDI_SG_LIST_INT);
	virt->base = cpu_to_le32(trans_rx->sg_list_kick0.dma +
				 sizeof(struct idi_sg_desc));
	virt->size = cpu_to_le32(IWL_IDI_RX_SIG_SIZE);

}

/**
* iwl_idi_rx_sg_list_reset - reset all the pointers in the given SG list
* The same SG list should not be reset simultaneously by several threds,
* thus the sync is needed only when accessing free mem buffers list.
*
* @idx - the index of the SG list
*/
static void iwl_idi_rx_sg_list_reset(struct iwl_idi_trans_rx *trans_rx,
				struct iwl_idi_sg_list_holder *sg_list_hld)
{
	struct iwl_trans *trans = IWL_TRANS_RX_GET_TRANS(trans_rx);
	struct iwl_rx_mem_buffer *rxb;
	struct idi_sg_desc *cur_virt;
	dma_addr_t cur_dma;
	int i;

	cur_virt = (struct idi_sg_desc *)sg_list_hld->sg_list.addr;
	cur_dma = sg_list_hld->sg_list.dma;

	for (i = 0; i < SG_LIST_MAX_SIZE; i++) {
		/* this is the point where DMA finished writing. It can happen
		* only in the first DMA descriptor out of the 4 that compose
		* the logical one. Note that it cannot be the last dma
		* descriptor because of the loop stop condition.
		*/
		if (((cur_virt->next & cpu_to_le32(IWL_IDI_SG_LIST_END)) ||
		     (cur_virt->next & cpu_to_le32(IWL_IDI_SG_LIST_INT)))) {
			cur_virt->next = cpu_to_le32(cur_dma +
					sizeof(struct idi_sg_desc));
			goto sg_list_ready;
		}

		rxb = iwl_idi_rx_page_pool_get(trans_rx);
		if (!rxb) {
			IWL_WARN(trans,
			"Failed to reset SG list - no free mem bufs\n");
			return;
		}

		/* configure 1st descriptor */
		cur_virt->next = cpu_to_le32(cur_dma +
					     sizeof(struct idi_sg_desc));
		cur_virt->base = cpu_to_le32(cur_dma +
					     sizeof(struct idi_sg_desc) +
					     offsetof(struct idi_sg_desc,
					     size));
		cur_virt->size = cpu_to_le32(sizeof(u32));
		cur_virt->pad = 0;

		/* configure 2nd descriptor */
		cur_virt++;
		cur_dma += sizeof(struct idi_sg_desc);

		cur_virt->next = cpu_to_le32(cur_dma +
					     sizeof(struct idi_sg_desc));
		cur_virt->base = cpu_to_le32(rxb->page_dma);

		sg_list_hld->addr_map[i] = rxb;
		cur_virt->size = 0;

		/* configure 3rd descriptor */
		cur_virt++;
		cur_dma += sizeof(struct idi_sg_desc);

		cur_virt->next = cpu_to_le32(cur_dma +
					     sizeof(struct idi_sg_desc));
		cur_virt->base = cpu_to_le32(cur_dma +
					     sizeof(struct idi_sg_desc) +
					     offsetof(struct idi_sg_desc,
					     size));
		cur_virt->size = cpu_to_le32(sizeof(u32));
		cur_virt->pad = 0;

		/* configure 4th descriptor */
		cur_virt++;
		cur_dma += sizeof(struct idi_sg_desc);

		cur_virt->next = cpu_to_le32(cur_dma +
					     sizeof(struct idi_sg_desc));
		cur_virt->base = cpu_to_le32(cur_dma +
					     offsetof(struct idi_sg_desc, pad));
		cur_virt->size = 0;
		cur_virt->pad = 0;

		/* move to the next logical descriptor */
		cur_virt++;
		cur_dma += sizeof(struct idi_sg_desc);
	}

	/* reach here only if already processed 16x4 descriptors;
	 * two additional descriptors are inteinded to read last 4 bytes
	 * signature (RX_DUMMY_SIGNATURE)
	 */
	memset(cur_virt, 0, 2*sizeof(struct idi_sg_desc));
	cur_virt->next = cpu_to_le32(cur_dma + sizeof(struct idi_sg_desc));
	/* put the signature in the pad field of teh second descriptor */
	cur_virt->base = cpu_to_le32(cur_dma + sizeof(struct idi_sg_desc) +
				     offsetof(struct idi_sg_desc, pad));
	/* the size of the signature is 4 bytes */
	cur_virt->size = 4;

sg_list_ready:
	/* from this moment the list can be used again */
	list_add(&sg_list_hld->list, &trans_rx->free_sg_lists);
}

static int iwl_idi_rx_prepare_sg_ll(struct iwl_trans *trans)
{
	struct iwl_idi_trans_rx *trans_rx = IWL_TRANS_GET_IDI_TRANS_RX(trans);
	size_t sg_byte_size;
	int i, ret;

	/*
	 * initialize data SG lists, allocate two additional physical
	 * descriptors to keep end-of-data notification.
	 */
	sg_byte_size = (SG_LIST_MAX_SIZE*SG_LIST_DMA_DESC_NUM + 2)*
		sizeof(struct idi_sg_desc);
	for (i = 0; i < SG_LIST_NUM; i++) {
		struct iwl_idi_sg_list_holder *cur_sg_list;

		cur_sg_list = kmalloc(sizeof(struct iwl_idi_sg_list_holder),
				      GFP_KERNEL);
		if (!cur_sg_list)
			goto error;

		ret = iwl_idi_alloc_sg_list(trans, &cur_sg_list->sg_list,
					    sg_byte_size, DMA_BIDIRECTIONAL);
		if (ret) {
			IWL_WARN(trans,
				 "Rx init failed - not enough memory.");
			kfree(cur_sg_list);
			goto error;
		}

		memset(cur_sg_list->addr_map, 0,
		       SG_LIST_MAX_SIZE*sizeof(void *));
		iwl_idi_rx_sg_list_reset(trans_rx, cur_sg_list);
	}

	/* initilaize kick0 SG list
	 * The memory for receiving signature data is appended
	 * at the end of SG list.
	 */
	sg_byte_size = sizeof(struct idi_sg_desc) +
		       sizeof(u16) + IWL_IDI_RX_SIG_SIZE;
	ret = iwl_idi_alloc_sg_list(trans, &trans_rx->sg_list_kick0,
				    sg_byte_size, DMA_BIDIRECTIONAL);
	if (ret) {
		IWL_WARN(trans, "Rx init failed - not enough memory.");
		goto error;
	}
	iwl_idi_rx_sg_list_reset_kick0(trans_rx);

	return 0;

error:
	iwl_idi_rx_free_sg_lists(trans);
	return -ENOMEM;
}

static int iwl_idi_rx_alloc_data(struct iwl_trans *trans)
{
	int ret;
	struct iwl_idi_trans_rx *trans_rx = IWL_TRANS_GET_IDI_TRANS_RX(trans);

	trans_rx->rxb_pool = kmem_cache_create("iwl_idi_rxb",
					       sizeof(struct iwl_rx_mem_buffer),
					       sizeof(void *), 0, NULL);
	if (!trans_rx->rxb_pool) {
		IWL_ERR(trans, "IDI Rx init failed\n");
		return -ENOMEM;
	}

	if (kfifo_alloc(&trans_rx->ready_pages_fifo,
			IWL_IDI_RX_READY_PAGES_MAX_NUM * sizeof(void *),
			GFP_KERNEL)) {
		IWL_ERR(trans, "alloc ready_pages_fifo failed");
		goto err_free_cache;
	}
	kfifo_reset(&trans_rx->ready_pages_fifo);

	ret = iwl_idi_rx_page_pool_alloc(trans_rx);
	if (ret)
		goto err_free_fifo;

	ret = iwl_idi_rx_page_pool_refill(trans_rx);
	if (ret)
		goto err_free_mem_rsrc;

	ret = iwl_idi_rx_prepare_sg_ll(trans);
	if (ret)
		goto err_free_mem_rsrc;

	ret = iwl_idi_rx_alloc_transaction(trans_rx);
	if (ret) {
		IWL_ERR(trans, "Rx init failed - IDI transaction.");
		goto err_free_all;
	}

	init_waitqueue_head(&trans_rx->stop_waitq);

	return 0;

err_free_all:
	iwl_idi_rx_free_sg_lists(trans);
err_free_mem_rsrc:
	iwl_idi_rx_free_fifo_and_rxbs(trans_rx, &trans_rx->page_fifo);
err_free_fifo:
	kfifo_free(&trans_rx->ready_pages_fifo);
err_free_cache:
	kmem_cache_destroy(trans_rx->rxb_pool);
	return -ENOMEM;
}

static void iwl_idi_rx_hw_init(struct iwl_trans *trans)
{
	/* Stop FH Rx DMA */
	iwl_write_direct32(trans, FH_MEM_RCSR_CHNL0_CONFIG_REG, 0);

	/* reset and flush pointers */
	iwl_write_direct32(trans, FH_MEM_RCSR_CHNL0_RBDCB_WPTR, 0);
	iwl_write_direct32(trans, FH_MEM_RCSR_CHNL0_FLUSH_RB_REQ, 0);
	iwl_write_direct32(trans, FH_RSCSR_CHNL0_RDPTR, 0);

	/* Reset driver's Rx queue write index */
	iwl_write_direct32(trans, FH_RSCSR_CHNL0_RBDCB_WPTR_REG, 0);

	/* Tell device where to find RBD circular buffer in AL SRAM */
	iwl_write_direct32(trans, FH_RSCSR_CHNL0_RBDCB_BASE_REG, 0);

	/* Tell device where in DRAM to update its Rx status */
	iwl_write_direct32(trans, FH_RSCSR_CHNL0_STTS_WPTR_REG,
			   (0xcafebabe >> 4));

	/* Enable FH Rx DMA.
	 * RX_QUEUE_SIZE_LOG means 256 RBDs */
	iwl_write_direct32(trans, FH_MEM_RCSR_CHNL0_CONFIG_REG,
			   FH_RCSR_RX_CONFIG_CHNL_EN_ENABLE_VAL |
			   FH_RCSR_CHNL0_RX_CONFIG_IRQ_DEST_INT_HOST_VAL |
			   FH_RCSR_RX_CONFIG_REG_VAL_RB_SIZE_8K |
			   (RX_QUEUE_SIZE_LOG <<
			    FH_RCSR_RX_CONFIG_RBDCB_SIZE_POS)|
			   (RX_RB_TIMEOUT <<
			    FH_RCSR_RX_CONFIG_REG_IRQ_RBTH_POS));

	iwl_write_direct32(trans, FH_RSCSR_CHNL0_RBDCB_WPTR_REG, 8);
}

int iwl_idi_rx_init(struct iwl_trans *trans)
{
	struct iwl_idi_trans_rx *trans_rx;
	int ret;

	BUG_ON(trans == NULL);
	BUG_ON(trans->trans_specific == NULL);

	trans_rx = IWL_TRANS_GET_IDI_TRANS_RX(trans);
	trans_rx->trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);

	INIT_LIST_HEAD(&trans_rx->free_sg_lists);
	INIT_LIST_HEAD(&trans_rx->used_sg_lists);

	trans_rx->wq = alloc_workqueue("idi_rx_wq",
				       WQ_UNBOUND | WQ_NON_REENTRANT |
				       WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!trans_rx->wq)
		return -ENOMEM;

	INIT_WORK(&trans_rx->page_refill_wrk,
		  iwl_idi_rx_page_pool_refill_work);
	INIT_WORK(&trans_rx->page_proc_wrk, iwl_idi_rx_process_pages);

	trans_rx->sm.state = IWL_IDI_RX_STOPPED;
	clear_bit(IWL_IDI_RX_SM_MISC_DMA_ARMED, &trans_rx->sm.misc);
	clear_bit(IWL_IDI_RX_SM_MISC_DMA_EXPECT_IDLE, &trans_rx->sm.misc);
	clear_bit(IWL_IDI_RX_SM_MISC_DMA_IDLE, &trans_rx->sm.misc);
	clear_bit(IWL_IDI_RX_SM_MISC_PROCESSING, &trans_rx->sm.misc);

	IWL_INFO(trans, "IDI Rx config: SGs %d, pages %d\n",
		 SG_LIST_NUM, IWL_IDI_RX_BG_POOL_SIZE);

	if (IWL_IDI_LHP_FLAGS & IWL_IDI_DBG_RX_TASKLET)
		IWL_INFO(trans, "Rx is configured in tasklet mode\n");

	ret = iwl_idi_rx_alloc_data(trans);
	if (ret)
		goto err_clean_wq;

	iwl_idi_rx_hw_init(trans);

	clear_bit(IWL_IDI_RX_SM_MISC_DMA_ARMED, &trans_rx->sm.misc);

	iwl_idi_rx_sm_move(trans_rx);

	IWL_INFO(trans, "%s: finished.\n", __func__);
	return 0;

err_clean_wq:
	destroy_workqueue(trans_rx->wq);
	trans_rx->wq = NULL;

	return ret;
}

static inline bool iwl_idi_rx_sm_idle(struct iwl_idi_trans_rx *trans_rx)
{
	return !test_bit(IWL_IDI_RX_SM_MISC_DMA_ARMED, &trans_rx->sm.misc);
}

static int iwl_idi_rx_verify(struct iwl_idi_trans_rx *trans_rx,
			     struct iwl_idi_dma_ptr *sg_list_kick0)
{
	struct idi_sg_desc *sg_desc;
	u32 *payload_data;
	struct iwl_trans *trans =
		IWL_TRANS_RX_GET_TRANS(trans_rx);

	sg_desc = (struct idi_sg_desc *)sg_list_kick0->addr;
	if (WARN(sg_desc->size != sizeof(u32),
		 "KICK0 payload size is wrong: %d instead of %d\n",
		 sg_desc->size, sizeof(u32)))
		return -EINVAL;

	payload_data = (u32 *)phys_to_virt(sg_desc->base);
	if (payload_data[0] != RX_KCK0_SIGNATURE) {
		IWL_DEBUG_INFO(trans,
			"KICK0 siganture is not valid: 0x%x instead of 0x%x\n",
			payload_data[0], RX_KCK0_SIGNATURE);
		return -EINVAL;
	}

	return 0;
}

static void iwl_idi_rx_handler_kick0(struct iwl_idi_trans_rx *trans_rx)
{
	int ret;
	struct iwl_trans *trans =
		IWL_TRANS_RX_GET_TRANS(trans_rx);

	ret = iwl_idi_rx_verify(trans_rx, &trans_rx->sg_list_kick0);
	if (ret) {
		/* we probably got some spurious data, move to the stopped
		 * state to ignore it */
		IWL_DEBUG_INFO(trans, "KICK0 verify signature failed\n");
		trans_rx->sm.state = IWL_IDI_RX_STOPPED;
	}

	clear_bit(IWL_IDI_RX_SM_MISC_DMA_ARMED, &trans_rx->sm.misc);
	iwl_idi_rx_sm_move(trans_rx);
}
static void iwl_idi_rx_handler_data(struct iwl_idi_trans_rx *trans_rx)
{
	struct iwl_trans *trans = IWL_TRANS_RX_GET_TRANS(trans_rx);

	/* move the armed S/G list to used */
	if (WARN_ON(NULL == trans_rx->armed_sg_list))
		return;

	iwl_idi_dma_sync_for_cpu(trans, &trans_rx->armed_sg_list->sg_list);

	list_add_tail(&trans_rx->armed_sg_list->list,
		      &trans_rx->used_sg_lists);
	trans_rx->armed_sg_list = NULL;
	clear_bit(IWL_IDI_RX_SM_MISC_DMA_ARMED, &trans_rx->sm.misc);

	if (!test_bit(IWL_IDI_RX_SM_MISC_DMA_EXPECT_IDLE, &trans_rx->sm.misc))
		iwl_idi_rx_sm_move(trans_rx);

	/* schedule processing of the received sg list*/
	iwl_idi_rx_sg_process(trans_rx);
}

static void iwl_idi_rx_process_one_page(struct iwl_idi_trans_rx *trans_rx,
			struct iwl_rx_mem_buffer *rxb)
{
	struct iwl_trans *trans = IWL_TRANS_RX_GET_TRANS(trans_rx);
	struct iwl_trans_slv *trans_slv = IWL_TRANS_GET_SLV_TRANS(trans);
	struct iwl_rx_cmd_buffer rxcb;
	int ret;

	dma_unmap_page(trans->dev,
		       rxb->page_dma,
		       PAGE_SIZE << trans_slv->rx_page_order,
		       DMA_FROM_DEVICE);

	rxcb._page = rxb->page;
	/*
	 * IDI currently doesn't support multiple frames in one
	 * rxb, so just resetting rxcb here.
	 */
	rxcb._offset = 0;
	rxcb._page_stolen = false;
	rxcb._rx_page_order = trans_slv->rx_page_order;
	rxcb.truesize = PAGE_SIZE << trans_slv->rx_page_order;

	ret = iwl_slv_rx_handle_dispatch(trans, &rxcb);
	if (ret) {
		IWL_ERR(trans, "Invalid Rx detected, restarting FW.\n");

		/* free the memory before resetting FW */
		if (rxb->page != NULL)
			__free_pages(rxb->page, trans_slv->rx_page_order);
		kmem_cache_free(trans_rx->rxb_pool, rxb);

		iwl_trans_fw_error(trans);
		return;
	}

	if (rxcb._page_stolen) {
		__free_pages(rxb->page,
			     trans_slv->rx_page_order);
		rxb->page = NULL;
	}

	if (rxb->page != NULL)
		__free_pages(rxb->page, trans_slv->rx_page_order);

	kmem_cache_free(trans_rx->rxb_pool, rxb);
}

static void iwl_idi_rx_process_pages(struct work_struct *data)
{
	struct iwl_idi_trans_rx *trans_rx =
		container_of(data, struct iwl_idi_trans_rx, page_proc_wrk);
	struct iwl_trans *trans = IWL_TRANS_RX_GET_TRANS(trans_rx);
	struct iwl_rx_mem_buffer *rxb;
	int count;

	/* Rx will be processed in the tasklet */
	if (test_bit(IWL_IDI_RX_SM_MISC_DMA_EXPECT_IDLE, &trans_rx->sm.misc))
		return;

	while ((count = kfifo_out(&trans_rx->ready_pages_fifo,
				(void *)&rxb, sizeof(void *))) != 0) {
		if (WARN_ON(rxb == NULL)) {
			/* FIXME: define behaviour */
			IWL_ERR(trans, "Invalid rxb in ready_pages_fifo\n");
			continue;
		}
		iwl_idi_rx_process_one_page(trans_rx, rxb);
	}
}

/**
 * iwl_idi_rx_verify_pad() - verify pad value between logical rx descriptors.
 * The pad value should be 0 after each middle logical descriptor and
 * RX_BRST_SIGNATURE after the last descriptor in LL.
 */
static int iwl_idi_rx_verify_pad(struct iwl_trans *trans, u32 next, u32 pad)
{
	if (IDI_RX_LL_DONE(next)) {
		if (pad != RX_BRST_SIGNATURE) {
			IWL_ERR(trans, "invalid last rx pad, val = 0x%x\n",
				pad);
			return -EINVAL;
		}
	} else if (pad) {
		IWL_ERR(trans, "invalid rx pad, val = 0x%x\n", pad);
		return -EINVAL;
	}

	return 0;
}

/**
 * iwl_idi_rx_sg_process - processing of the Rx SG list.
 *
 * FIXME: the current implementation still supports having several
 * used SG LLs, which could happen when it was in a separate tasklet.
 * Now, this function runs in the context of the interrupt taslet,
 * thus only one used SG LL is possible. Need to consider moving
 * to a simpler implementation.
 */
static void iwl_idi_rx_sg_process(struct iwl_idi_trans_rx *trans_rx)
{
	struct idi_sg_desc *sg_virt;
	struct iwl_trans *trans = IWL_TRANS_RX_GET_TRANS(trans_rx);
	struct iwl_idi_sg_list_holder *cur_sg_list;
	struct list_head processed_lists;
	struct iwl_rx_mem_buffer *rxb;
	int i;
	u32 pad;
	bool expect_idle = test_bit(IWL_IDI_RX_SM_MISC_DMA_EXPECT_IDLE,
				    &trans_rx->sm.misc);

	INIT_LIST_HEAD(&processed_lists);

	/*
	 * Go over all SG lists that are back from DMA and still weren't
	 * processed. The situation when number of such lists is more
	 * than 1 could arise when the Rx interrupt with a new SG list
	 * arrives during the processing of the previous one.
	 */
	while (!list_empty(&trans_rx->used_sg_lists)) {
		cur_sg_list = iwl_rx_get_sg_list(&trans_rx->used_sg_lists);

		sg_virt = (struct idi_sg_desc *)
					cur_sg_list->sg_list.addr;

		/* push all pages from SG LL to fifo */
		for (i = 0; i < SG_LIST_MAX_SIZE; i++) {
			/* This indicates the end of the received data. */
			if (IDI_RX_LL_DONE(sg_virt->next))
				break;

			/* skip next, base and size field up to the pad */
			sg_virt += 3;
			pad = sg_virt->pad;

			/* skip to the next descriptor */
			sg_virt++;

			if (iwl_idi_rx_verify_pad(trans, sg_virt->next, pad)) {
				iwl_trans_fw_error(trans);
				return;
			}

			if (expect_idle ||
			    (IWL_IDI_LHP_FLAGS & IWL_IDI_DBG_RX_TASKLET)) {
				rxb = cur_sg_list->addr_map[i];
				iwl_idi_rx_process_one_page(trans_rx, rxb);
				cur_sg_list->addr_map[i] = NULL;
				continue;
			}

			if (kfifo_is_full(&trans_rx->ready_pages_fifo)) {
				IWL_ERR(trans, "Full ready_pages_fifo\n");
				/* FIXME: is it enough to discard this page? */
				iwl_idi_rx_free_rxb(trans_rx,
						    cur_sg_list->addr_map[i]);
				cur_sg_list->addr_map[i] = NULL;
				continue;
			}
			kfifo_in(&trans_rx->ready_pages_fifo,
				 (void *)&cur_sg_list->addr_map[i],
				 sizeof(void *));

			cur_sg_list->addr_map[i] = NULL;
		}

		if (!expect_idle &&
		    !(IWL_IDI_LHP_FLAGS & IWL_IDI_DBG_RX_TASKLET))
			queue_work(trans_rx->wq, &trans_rx->page_proc_wrk);

		/* optimization: when DMA is idle reset S/G list now */
		if (iwl_idi_rx_sm_idle(trans_rx)) {
			iwl_idi_rx_sg_list_reset(trans_rx, cur_sg_list);
			iwl_idi_rx_sm_move(trans_rx);
		} else {
			list_add_tail(&cur_sg_list->list, &processed_lists);
		}
	}

	while (!list_empty(&processed_lists)) {
		cur_sg_list = iwl_rx_get_sg_list(&processed_lists);

		iwl_idi_rx_sg_list_reset(trans_rx, cur_sg_list);
		iwl_idi_rx_sm_move(trans_rx);
	}

	/*
	* An optimization which moves the state machine in case this run of
	* the tasklet didn't processed any SG list.
	*/
	if (iwl_idi_rx_sm_idle(trans_rx))
		iwl_idi_rx_sm_move(trans_rx);
}

/**
* iwl_idi_rx_dma_arm - set a new Rx SG list to DMA a kick.
*
*/
static int iwl_idi_rx_dma_arm(struct iwl_idi_trans_rx *trans_rx)
{
	struct iwl_trans *trans = IWL_TRANS_RX_GET_TRANS(trans_rx);
	struct iwl_idi_sg_list_holder *cur_sg_list;
	struct iwl_idi_dma_ptr *sg_list = NULL;
	int ret;

	if (trans_rx->sm.state == IWL_IDI_RX_KICK0) {
		sg_list = &trans_rx->sg_list_kick0;
	} else if (!list_empty(&trans_rx->free_sg_lists)) {
		cur_sg_list =
			iwl_rx_get_sg_list(&trans_rx->free_sg_lists);
		trans_rx->armed_sg_list = cur_sg_list;
		sg_list = &cur_sg_list->sg_list;
	}

	if (sg_list == NULL) {
		IWL_ERR(trans, "No free S/G list.");
		return -ENOMEM;
	}

	iwl_idi_dma_sync_for_device(trans, sg_list);

	/* The API of IDI driver will be updated, should be phys addr */
	trans_rx->transaction->idi_xfer.desc = (void *)sg_list->dma;
	ret = idi_async_read(trans_rx->trans_idi->pdev, trans_rx->transaction);
	if (ret) {
		IWL_ERR(trans, "IDI async read failed.");
		return ret;
	}

	return 0;
}

void iwl_idi_rx_dma_expect_idle(struct iwl_trans *trans, bool start)
{
	struct iwl_idi_trans_rx *trans_rx = IWL_TRANS_GET_IDI_TRANS_RX(trans);

	if (start) {
		set_bit(IWL_IDI_RX_SM_MISC_DMA_EXPECT_IDLE,
			&trans_rx->sm.misc);
		/*
		 * make sure the next Rx will be processed directly from the
		 * tasklet.
		 */
		flush_work(&trans_rx->page_proc_wrk);
	} else {
		clear_bit(IWL_IDI_RX_SM_MISC_DMA_EXPECT_IDLE,
			  &trans_rx->sm.misc);
	}
}

void iwl_idi_rx_dma_idle(struct iwl_trans *trans)
{
	struct iwl_idi_trans_rx *trans_rx = IWL_TRANS_GET_IDI_TRANS_RX(trans);

	WARN_ON_ONCE(!test_bit(IWL_IDI_RX_SM_MISC_DMA_EXPECT_IDLE,
			       &trans_rx->sm.misc));

	set_bit(IWL_IDI_RX_SM_MISC_DMA_IDLE, &trans_rx->sm.misc);
	clear_bit(IWL_IDI_RX_SM_MISC_DMA_EXPECT_IDLE, &trans_rx->sm.misc);
}

static void iwl_idi_rx_sm_move(struct iwl_idi_trans_rx *trans_rx)
{
	struct iwl_trans *trans = IWL_TRANS_RX_GET_TRANS(trans_rx);

	if (test_bit(IWL_IDI_RX_SM_MISC_DMA_IDLE, &trans_rx->sm.misc)) {
		IWL_DEBUG_RPM(trans, "Avoiding Rx DMA re-arm\n");
		/* shouldn't happen when we're in the middle */
		WARN_ON_ONCE(trans_rx->sm.state == IWL_IDI_RX_KICK0);
		return;
	}

	if (test_and_set_bit(IWL_IDI_RX_SM_MISC_DMA_ARMED, &trans_rx->sm.misc))
		return;

	switch (trans_rx->sm.state) {
	case IWL_IDI_RX_STOPPED:
		if (test_bit(IWL_IDI_RX_SM_MISC_KICK0_IS_ON,
			     &trans_rx->sm.misc)) {
			trans_rx->sm.state = IWL_IDI_RX_KICK0;
			iwl_idi_rx_dma_arm(trans_rx);
		} else if (!list_empty(&trans_rx->free_sg_lists)) {
			trans_rx->sm.state = IWL_IDI_RX_DATA;
			iwl_idi_rx_dma_arm(trans_rx);
		} else {
			IWL_DEBUG_RX(trans, "moving to no SG list state.");
			trans_rx->sm.state = IWL_IDI_RX_NO_SG_LIST;
			clear_bit(IWL_IDI_RX_SM_MISC_DMA_ARMED,
				  &trans_rx->sm.misc);
		}
		break;
	case IWL_IDI_RX_KICK0:
		if (!list_empty(&trans_rx->free_sg_lists)) {
			trans_rx->sm.state = IWL_IDI_RX_DATA;
			iwl_idi_rx_dma_arm(trans_rx);
		} else {
			IWL_DEBUG_RX(trans, "moving to no SG list state.");
			trans_rx->sm.state = IWL_IDI_RX_NO_SG_LIST;
			clear_bit(IWL_IDI_RX_SM_MISC_DMA_ARMED,
				  &trans_rx->sm.misc);
		}
		break;
	case IWL_IDI_RX_DATA:
		if (test_bit(IWL_IDI_RX_SM_MISC_KICK0_IS_ON,
			     &trans_rx->sm.misc)) {
			trans_rx->sm.state = IWL_IDI_RX_KICK0;
			iwl_idi_rx_dma_arm(trans_rx);
		} else if (!list_empty(&trans_rx->free_sg_lists)) {
			/* no kick0 - remain in the current state */
			iwl_idi_rx_dma_arm(trans_rx);
		} else {
			IWL_DEBUG_RX(trans, "moving to no SG list state.");
			trans_rx->sm.state = IWL_IDI_RX_NO_SG_LIST;
			clear_bit(IWL_IDI_RX_SM_MISC_DMA_ARMED,
				  &trans_rx->sm.misc);
		}
		break;
	case IWL_IDI_RX_NO_SG_LIST:
		if (!list_empty(&trans_rx->free_sg_lists)) {
			IWL_DEBUG_RX(trans, "moving out of no SG list state.");
			trans_rx->sm.state = IWL_IDI_RX_DATA;
			iwl_idi_rx_dma_arm(trans_rx);
		}
		break;
	default:
		IWL_ERR(trans, "IDI Rx SM reached invalid state.\n");
		break;
	}
}
