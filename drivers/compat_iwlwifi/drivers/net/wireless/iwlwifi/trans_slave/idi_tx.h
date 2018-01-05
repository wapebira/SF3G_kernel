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

#ifndef __iwl_trans_tx_idi_h__
#define __iwl_trans_tx_idi_h__

#include <linux/skbuff.h>
#include <linux/idi/idi_interface.h>

#include "shared.h"
#include "idi_al.h"
#include "idi_utils.h"
#include "idi_tx_policy.h"

/* convert IDI channel to SG List index in the array */
#define IWL_IDI_CHAN_TO_SG_IDX(chan) (((chan) & IDI_PRIMARY_CHANNEL) ? 0 : 1)
#define IWL_IDI_SG_IDX_TO_CHAN(idx) (((idx) == 0) ?\
			IDI_PRIMARY_CHANNEL : IDI_SECONDARY_CHANNEL)

/* the size of SG list f- descriptors number */
#define IWL_IDI_TX_CH_SG_SIZE 16
#define IWL_IDI_TX_SG_FRAGS_MAX \
	((IWL_MAX_CMD_TBS_PER_TFD + 1) * IWL_IDI_TX_CH_SG_SIZE)
#define IWL_IDI_TXBU_SIGNATURE 0xDEA1

/* AL TB (tx buffer) size in bytes */
#define IWL_IDI_TXBU_HEADROOM_SIZE 512
#define IWL_IDI_PAYLOAD_PAGE_SIZE_MASK (IDI_TX_PAYLOAD_PAGE_SIZE - 1)
#define IWL_IDI_MAX_TXC_COUNT_IN_TXBU 31

/* The number of used IDI bus channels, could be
 * 1  - only primary IDI Tx channels is used
 * 2  - both primary and secondary IDI Tx channels are used
 */
#define IWL_IDI_TX_CHAN_NUM 1

/* handling flags in txbu header */
#define IWL_IDI_TXBU_TXSC_BIT BIT(5)
#define IWL_IDI_TXBU_INTEN_BIT BIT(6)
#define IWL_IDI_TXBU_EX_BIT BIT(7)

/* uCode dma size */
#define IWL_SRAM_SIZE_FOR_FW (30 * 1024)

/**
 * struct iwl_idi_txbu_header - IDI Tx Burst header
 * @signature: Used for stream synchronization purposes
 * @seq_num: Used as a uniqe identifier for eacg of the TXBUs
 * @txc_count:	bits[0:4]: The number of TXCs in this burst
 *		bits[5-7]: Reserved.
 *
 * @queue:	bits[0:4]: Extended TXBU mode only.
 *		Select which queue to update in the SCHD block; used to
 *		calculate the entry in the LUT and BC tables.
 *		bits[5-7]: Reserved.
 *
 * @tfd_index:	Extended TXBU mode only.
 *		Calculate the next TFD index value that is written to the
 *		SCHD_WR_PTR.
 *
 * @flags:	bits[0-4]: Reserved.
 *		bit[5]:	txsc - TXC size control. enable/disable TXC size
 *			checking.
 *		bit[6]:	inten - Interrupt control. post TXBU Interrupt control
 *			at the end of the TXBU processing.
 *		bit[7]:	ex - extended: control the TXBU type (basic\extended).
 *
 * @lut_value:	Extended TXBU mode only. The byte count value that will
 *		be written to the SFDB_LUT[queue][tfd_index]
 * @reserved.
 * @byte_cnt_value:Extended TXBU mode only. The byte count value that will
 *		   be written to the SFDB_BC[queue][tfd_index]
 */
struct iwl_idi_txbu_header {
	__le16 signature;
	__le16 seq_num;
	u8 txc_count;
	u8 queue;
	u8 tfd_index;
	u8 flags;
	u8 lut_value;
	u8 reserved;
	__le16 byte_cnt_value;
} __packed;

/* handling destination address and flags in TXC header */
#define IWL_IDI_TXC_TFDI_BIT BIT(30)
#define IWL_IDI_TXC_REGA_BIT BIT(31)
#define IWL_IDI_TXC_DEST_MASK 0x000fffff

/**
 * struct iwl_idi_txc_header - IDI Tx Chunks header
 * @pb:			bits[0:1]: Pad before data.
 *			bits[2-7]: Reserved.
 * @pa:			bits[0:1]: Pad after data.
 *			bits[2-7]: Reserved.
 * @len:			TXC data length.
 * @dest_and_flags:	bits[0-19]: Destination address.
 *			bits[20-29]: Reserved.
 *			bit[30]: TFDI - Compressed TFD info (debug).
 *			bit[31]: REGA - Register Access (debug).
 */
struct iwl_idi_txc_header {
	u8 pb;
	u8 pa;
	__le16 len;
	__le32 dest_and_flags;
} __packed;

struct iwl_idi_tx_map_data {
	DEFINE_DMA_UNMAP_ADDR(mapping);
	DEFINE_DMA_UNMAP_LEN(len);
};

/**
 * struct iwl_idi_tx_reclaim_info - holds meta data used for reclaiming
 * memory blocks.
 * @map_data: an array of physical addresses and lengths of memory blocks;
 *	      blocksthe array size is according to the max number of possible
 *	      separate blocks. Since data uses 2 and host command uses
 *	      IWL_MAX_CMD_TBS_PER_TFD, this number is used. One additional
 *	      entry is for the case that all host command blocks are
 *	      NOCOPY, we need extra block for the headroom.
 * @txbu: pointer to the start of txbu header (inside the headroom)
 * @blocks_count: number of separate memory blocks used
 */
struct iwl_idi_tx_reclaim_info {
	struct iwl_idi_tx_map_data map_data[IWL_MAX_CMD_TBS_PER_TFD + 1];
	struct iwl_idi_txbu_header *txbu;
	u8 blocks_count;
};

/**
 * struct iwl_idi_tx_sg_meta - auxiliary data for maintaining an S/G list
 * @max_txbus: the max number of messages that this S/G list can carry
 * @used_count: the idx at which a new msg should be added
 */
struct iwl_idi_tx_sg_meta {
	int max_txbus;
	int used_count;
};

/**
 * struct iwl_idi_trans_tx - IDI Tx transport
 * @trans_idi: pointer to the general IDI transport
 * @sg_list: array of S/G lists for DBB DMA
 * @sg_list_meta: meta data for each S/G list
 * @reclaim_info_pool: the pool of iwl_idi_tx_reclaim_info for dynamic alloc
 * @txbus_added_policy: used by the quota reservation mechanism
 * @tfd_pool: pool of TFDs in AL SRAM
 * @tb_pool: pool of TBs in AL SRAM
 * @sg_list_loaded: indicates there's sg list loaded in dma
 * @mem_rsrc_lock: sync access to tb_pool, tfd_pool and sg_list.
 * @cmd_fifo: index of command fifo
 * @txbu_seq_num: serial sequence number for txbus.
 * @next_tfd_index: array of the next free tfd index for each queue
 * @ucode_write_waitq: wait queue for uCode load
 * @ucode_write_complete: indicates that the ucode has been downloaded.
 * @fw_load_refcount: counts responses to fw chunk load (FH, IDI)
 */
struct iwl_idi_trans_tx {
	struct iwl_trans_idi *trans_idi;
	struct iwl_trans_slv_tx slv_tx;
	struct iwl_idi_dma_ptr sg_list[IWL_IDI_TX_CHAN_NUM];
	struct iwl_idi_tx_sg_meta sg_list_meta[IWL_IDI_TX_CHAN_NUM];
	struct idi_transaction *transaction[IWL_IDI_TX_CHAN_NUM];
	struct kmem_cache *reclaim_info_pool;
	u8 txbus_added_policy[IWL_IDI_TX_CHAN_NUM];
	unsigned long sg_list_loaded;
	u32 scd_base_addr;
	u16 txbu_seq_num;
	u8 next_tfd_index[IDI_TX_MAX_Q_COUNT+1];
	wait_queue_head_t ucode_write_waitq;
	bool ucode_write_complete;
	unsigned int al_reg;
	wait_queue_head_t stop_waitq;
	atomic_t fw_load_refcount;
};

void iwl_idi_load_fw_wake_wq(struct iwl_trans *trans);
void iwl_idi_tx_close_sg_list(struct iwl_idi_trans_tx *trans_tx, u8 idx);
int iwl_idi_tx_add_burst(struct iwl_trans_slv *trans_slv, u8 idx, u8 txq_id);
int iwl_idi_tx_dma_arm(struct iwl_idi_trans_tx *trans_tx, u8 idx);
void iwl_idi_tx_stop(struct iwl_trans *trans);
int iwl_idi_load_given_ucode(struct iwl_trans *trans, const struct fw_img *img,
			     u32 fw_dbg_flags);
void iwl_idi_tx_calc_txcs_num(struct iwl_trans *trans,
			      struct iwl_slv_tx_chunk_info *chunk_info);
void iwl_idi_tx_free_txbu_mem(struct iwl_trans *trans, void **data);
void iwl_idi_tx_clean_txbu(struct iwl_trans *trans, void *data);

int iwl_idi_tx_set_channel_config(struct iwl_trans *trans);

#endif
