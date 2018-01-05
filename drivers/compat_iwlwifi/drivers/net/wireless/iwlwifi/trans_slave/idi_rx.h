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

#ifndef __iwl_trans_rx_idi_h__
#define __iwl_trans_rx_idi_h__

#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/idi/idi_interface.h>
#include <linux/atomic.h>
#include <linux/kfifo.h>

#include "idi_utils.h"

#define SG_LIST_MAX_SIZE 16	/* number of logical descriptors (4 entries) */
#define SG_LIST_DMA_DESC_NUM 4	/* number of desc in one logical desc */
#define SG_LIST_NUM 3		/* default number of SG LLs */

/* the size of the signature data for the sync (kick0) phase, in bytes */
#define IWL_IDI_RX_SIG_SIZE 4

#define IWL_IDI_RX_BG_POOL_SIZE 512
#define IWL_IDI_RX_BG_POOL_TRSHLD 400
#define IWL_IDI_RX_READY_PAGES_MAX_NUM 2048

/**
* the state machine of the Rx
*/
enum iwl_trans_idi_rx_state {
	IWL_IDI_RX_STOPPED,	/* idi rx transport is stopped */
	IWL_IDI_RX_KICK0,	/* kick0 */
	IWL_IDI_RX_DATA,	/* waiting for real rx data */
	IWL_IDI_RX_NO_SG_LIST	/* all sg lists are waiting/in dma */
};

#define IWL_IDI_RX_SM_MISC_KICK0_IS_ON 0
#define IWL_IDI_RX_SM_MISC_DMA_ARMED 1
#define IWL_IDI_RX_SM_MISC_DMA_EXPECT_IDLE 2
#define IWL_IDI_RX_SM_MISC_DMA_IDLE 3
#define IWL_IDI_RX_SM_MISC_PROCESSING 4

struct iwl_idi_rx_sm {
	enum iwl_trans_idi_rx_state state;
	unsigned long misc;
};

/**
* struct iwl_idi_trans_rx_stats - statistics for tuning the module
* @used_sg_lists: the number of sg lists ever used
* @sg_list_filled: the maximum number of entries used for each sg list
*/
struct iwl_idi_trans_rx_stats {
	u8 used_sg_lists;
	u16 sg_list_filled[SG_LIST_NUM];
};

struct iwl_idi_sg_list_holder {
	struct iwl_idi_dma_ptr sg_list;
	struct iwl_rx_mem_buffer *addr_map[SG_LIST_MAX_SIZE];
	struct list_head list;
};

/*
TODO: consider using first 4 bytes of each page as the pointer to the page
TODO: instead of using mem_buf_pool and addr_map.
*/

/**
 * struct iwl_idi_trans_rx
 * @trans_idi:pointer to the general IDI transport
 * @free_sg_lists: SG LLs that are ready to use
 * @used_sg_lists: SG LLs that with received pages
 * @rxb_pool: pool of RXBs with allocated pages
 * @armed_sg_list: pointer to SG LL currently in DMA
 * @sg_list_kick0: the SG list for reading signature
 * @sm:the state machine of rx transport
 * @page_fifo:fifo that holds pages allocated in a background worker
 * @ready_pages_fifo: received pages to be processed
 * @page_proc_wrk: worker for passing received pages up
 * @page_refill_wrk:worker which allocates pages in the background
 * @wq: workqueue for rx transport sole use
 */
struct iwl_idi_trans_rx {
	struct iwl_trans_idi *trans_idi;
	struct list_head free_sg_lists;
	struct list_head used_sg_lists;
	struct kmem_cache *rxb_pool;
	struct iwl_idi_sg_list_holder *armed_sg_list;
	struct iwl_idi_dma_ptr sg_list_kick0;
	struct idi_transaction *transaction;
	struct iwl_idi_rx_sm sm;
	struct kfifo page_fifo;
	struct kfifo ready_pages_fifo;
	struct work_struct page_proc_wrk;
	struct work_struct page_refill_wrk;
	struct workqueue_struct *wq;
	wait_queue_head_t stop_waitq;

#ifdef CPTCFG_IWLWIFI_DEBUGFS
	struct iwl_idi_trans_rx_stats stats;
#endif
};

int iwl_idi_rx_set_channel_config(struct iwl_trans *trans);

#endif
