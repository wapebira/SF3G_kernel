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
#include "iwl-fh.h"
#include "iwl-prph.h"
#include "iwl-op-mode.h"
#include "iwl-agn-hw.h"
/* FIXME: remove this include after AMPDU support is added */
#include "iwl-modparams.h"

#include "idi_al.h"
#include "idi_host_csr.h"
#include "idi_internal.h"
#include "idi_utils.h"
#include "idi_tx.h"
#include "idi_tx_policy.h"
#include "shared.h"
#ifdef CPTCFG_IWLWIFI_DEVICE_TESTMODE
#include "iwl-dnt-cfg.h"
#endif
#include "idi_constants.h"

/* the LLS determine the SCD write-ptr by the tfd_idx, hence it shold be
 * warped to the TFD queue size */
#define INC_TFD_IDX(_idx) ((_idx) = ((_idx) + 1) & (TFD_QUEUE_SIZE_MAX - 1))

/* FIXME: which part of the SRAM can be used in FW download? */
//#define IWL_SRAM_SIZE_FOR_FW (30 * 1024)
#define IWL_IDI_TXBU_CAPACITY (IWL_IDI_MAX_TXC_COUNT_IN_TXBU *\
			       IDI_TX_PAYLOAD_PAGE_SIZE)

/* the FH config for FW load is a fixed procedure with 6 registers involved */
#define IWL_IDI_NUM_FH_REG_TO_CONFIG 6
#define IWL_IDI_FH_REG_SIZE sizeof(u32)

#define IWL_IDI_TXH_RES_NAME	"txh fifo"
#define IWL_IDI_TXL_RES_NAME	"txl fifo"

/* An address used by LhP FW to enable usage of all FIFOs memory
 * for ADC sampling debug */
#define IWL_LHP_ADC_SAMP_DBG 0x8179fc

#define MOD_WITHOUT_ZERO(_x, _mod) ((_x) % (_mod) == 0 ?\
				   (_mod) : ((_x) % (_mod)))

/* DBB DMA requires that the address and the total length of each chunk
 * will be aligned to 4. The LLS requires that the total len for each TXC
 * will be also aligned to 4 */
#define PADDING_BEFORE_ADDR(_addr) ((unsigned long)(_addr) & 0x3)
#define LEN_WITH_PADDINGS(_len, _pb) ALIGN((_len) + (_pb), 4)
#define ALIGN_ADDR_BACKWARDS(_addr) ((_addr) - PADDING_BEFORE_ADDR(_addr))

static void iwl_idi_tx_fix_sg_list(struct iwl_trans *trans, u32 idx,
				   bool is_fw_download);

/**
 * iwl_idi_tx_dma_arm - start IDI dma transaction
 * @trans_tx: tx transport
 * @idx: transaction index, determined by channel
 */
int iwl_idi_tx_dma_arm(struct iwl_idi_trans_tx *trans_tx, u8 idx)
{
	struct iwl_trans *trans = IWL_TRANS_TX_GET_TRANS(trans_tx);
	int ret = 0;

	iwl_idi_dma_sync_for_device(trans, &trans_tx->sg_list[idx]);

	ret = idi_async_write(trans_tx->trans_idi->pdev,
			      trans_tx->transaction[idx]);

	if (ret) {
		IWL_ERR(trans, "idi_async_write returns with error, ret = %d\n",
			ret);
		goto error;
	}

error:
	/* FIXME: Need to define error handling */
	return ret;
}

static inline u16 iwl_idi_get_txbu_hdr_size(u8 txcs_num)
{
	return sizeof(struct iwl_idi_txbu_header) +
		txcs_num * sizeof(struct iwl_idi_txc_header);
}

static inline u16 iwl_idi_tx_get_txbu_hdr_size(u8 txcs_num)
{
	return iwl_idi_get_txbu_hdr_size(txcs_num) +
		sizeof(struct iwl_idi_tfd);
}

static inline struct iwl_idi_txc_header *iwl_idi_tx_get_txcs_from_txbu(
					struct iwl_idi_txbu_header *txbu)
{
	return (struct iwl_idi_txc_header *)((u8 *)txbu +
			sizeof(struct iwl_idi_txbu_header));
}

static inline struct iwl_idi_tfd *iwl_idi_tx_get_tfd_from_txbu(
					struct iwl_idi_txbu_header *txbu,
					u8 txcs_num)
{
	return (struct iwl_idi_tfd *)
		((u8 *)txbu + sizeof(struct iwl_idi_txbu_header) +
		txcs_num * sizeof(struct iwl_idi_txc_header));
}

static inline int iwl_idi_set_sg_desc(struct iwl_idi_trans_tx *trans_tx,
				      u8 idx, int sg_idx, dma_addr_t phys_addr,
				      u32 len)
{
	struct idi_sg_desc *sg_desc;

	sg_desc = (struct idi_sg_desc *)trans_tx->sg_list[idx].addr;

	sg_desc[sg_idx].base = cpu_to_le32(phys_addr);
	sg_desc[sg_idx].size = cpu_to_le32(len);
	return 0;
}

static int iwl_idi_build_fw_chunk(struct iwl_idi_trans_tx *trans_tx,
				  struct iwl_idi_dma_ptr *hdrs,
				  u32 byte_cnt,
				  dma_addr_t phy_addr,
				  int *hdr_pos)
{
	struct iwl_idi_txbu_header *txbu;
	struct iwl_idi_txc_header *txcs;
	dma_addr_t dma_addr;
	u8 txcs_num, j, leftover_txcs;
	u16 txc_len, seq_num, leftover_bytes;
	u32 sram_dest, txbu_total_len;
	int txcs_in_chunk, txbus_num;
	int i, sg_idx;

	u8 idx = IWL_IDI_CHAN_TO_SG_IDX(IDI_PRIMARY_CHANNEL);

	txcs_in_chunk = DIV_ROUND_UP(byte_cnt, IDI_TX_PAYLOAD_PAGE_SIZE);
	leftover_bytes = byte_cnt % IDI_TX_PAYLOAD_PAGE_SIZE;

	txbus_num = DIV_ROUND_UP(txcs_in_chunk, IWL_IDI_MAX_TXC_COUNT_IN_TXBU);
	leftover_txcs = txcs_in_chunk % IWL_IDI_MAX_TXC_COUNT_IN_TXBU;

	sg_idx = trans_tx->sg_list_meta[idx].used_count;
	seq_num = trans_tx->txbu_seq_num;

	sram_dest = IDI_AL_SFDB_BASE_ADDR;
	*hdr_pos = 0;

	for (i = 0; i < txbus_num; i++) {
		txbu = (struct iwl_idi_txbu_header *)((u8 *)hdrs->addr +
						      *hdr_pos);
		memset(txbu, 0, sizeof(*txbu));
		txcs = iwl_idi_tx_get_txcs_from_txbu(txbu);
		txcs_num = (i == (txbus_num - 1) && leftover_txcs != 0) ?
			leftover_txcs : IWL_IDI_MAX_TXC_COUNT_IN_TXBU;
		txbu_total_len = 0;
		txc_len = IDI_TX_PAYLOAD_PAGE_SIZE;

		for (j = 0; j < txcs_num; j++) {
			/* image sections are aligned, no need to add pa, pb */
			txcs->dest_and_flags = cpu_to_le32(sram_dest);
			if (i == (txbus_num - 1) && j == (txcs_num - 1) &&
			    leftover_bytes != 0) {
				/* len of the last TXC in the last TXBU */
				txc_len = leftover_bytes;
			}
			txcs->len = cpu_to_le16(txc_len);
			sram_dest += txc_len;
			txbu_total_len += txc_len;
			txcs++;
		}

		txbu->signature = cpu_to_le16(IWL_IDI_TXBU_SIGNATURE);
		txbu->txc_count = txcs_num;
		txbu->seq_num = cpu_to_le16(seq_num++);

		/* first descriptor for headers, second for data */
		dma_addr = (dma_addr_t)(hdrs->dma + *hdr_pos);

		iwl_idi_set_sg_desc(trans_tx, idx, sg_idx++, dma_addr,
				    iwl_idi_get_txbu_hdr_size(txcs_num));
		iwl_idi_set_sg_desc(trans_tx, idx, sg_idx++, phy_addr,
				    txbu_total_len);

		/* check if S/G ll is full */
		if (WARN_ON(sg_idx > IWL_IDI_TX_SG_FRAGS_MAX))
			return -ENOSR;

		phy_addr += txbu_total_len;
		*hdr_pos += iwl_idi_get_txbu_hdr_size(txcs_num);
	}

	trans_tx->sg_list_meta[idx].used_count = sg_idx;
	trans_tx->txbu_seq_num = seq_num;

	return 0;
}

/**
 * iwl_idi_set_fh_txc() - configure TXC for FH configuration
 * @txc: address of the TXC
 * @dest: the local FH address
 */
static inline void iwl_idi_set_fh_txc(struct iwl_idi_txc_header *txc, u32 dest)
{
	/* FH addresses are accessible through AMFH TG1 WR */
	txc->dest_and_flags = cpu_to_le32(AMFH_TG1_WR_BASE_ADDR + dest);
	txc->len = cpu_to_le16(IWL_IDI_FH_REG_SIZE);
}

static int iwl_idi_load_fw_config_fh(struct iwl_idi_trans_tx *trans_tx,
				     struct iwl_idi_dma_ptr *headers,
				     u32 byte_cnt, u32 dst_addr, int *hdr_pos)
{
	struct iwl_idi_txbu_header *txbu;
	struct iwl_idi_txc_header *txcs;
	dma_addr_t dma_addr;

	int sg_idx;
	u8 idx = IWL_IDI_CHAN_TO_SG_IDX(IDI_PRIMARY_CHANNEL);
	u32 phy_addr, len;
	__le32 *data;

	/* the source address in S&F to read FW chunk from */
	phy_addr = IDI_AL_SFDB_BASE_ADDR;

	txbu = (struct iwl_idi_txbu_header *)((u8 *)headers->addr + *hdr_pos);
	memset(txbu, 0, sizeof(*txbu));
	txbu->signature = cpu_to_le16(IWL_IDI_TXBU_SIGNATURE);
	txbu->txc_count = IWL_IDI_NUM_FH_REG_TO_CONFIG;
	txbu->seq_num = cpu_to_le16(trans_tx->txbu_seq_num);

	trans_tx->txbu_seq_num++;

	txcs = iwl_idi_tx_get_txcs_from_txbu(txbu);
	data = (__le32 *)(txcs + txbu->txc_count); /* FH registers content */

	iwl_idi_set_fh_txc(txcs++, FH_TCSR_CHNL_TX_CONFIG_REG(FH_SRVC_CHNL));
	data[0] = cpu_to_le32(FH_TCSR_TX_CONFIG_REG_VAL_DMA_CHNL_PAUSE);

	iwl_idi_set_fh_txc(txcs++, FH_SRVC_CHNL_SRAM_ADDR_REG(FH_SRVC_CHNL));
	data[1] = cpu_to_le32(dst_addr);

	/* phy_addr is the address inside device's SRAM, so it's u32 */
	iwl_idi_set_fh_txc(txcs++, FH_TFDIB_CTRL0_REG(FH_SRVC_CHNL));
	data[2] = cpu_to_le32(phy_addr & FH_MEM_TFDIB_DRAM_ADDR_LSB_MSK);

	/* it's always 0 as long as we use an address inside SRAM */
	iwl_idi_set_fh_txc(txcs++, FH_TFDIB_CTRL1_REG(FH_SRVC_CHNL));
	data[3] = cpu_to_le32((iwl_get_dma_hi_addr(phy_addr)
		   << FH_MEM_TFDIB_REG1_ADDR_BITSHIFT) | byte_cnt);

	iwl_idi_set_fh_txc(txcs++, FH_TCSR_CHNL_TX_BUF_STS_REG(FH_SRVC_CHNL));
	data[4] = cpu_to_le32(1 << FH_TCSR_CHNL_TX_BUF_STS_REG_POS_TB_NUM |
		  1 << FH_TCSR_CHNL_TX_BUF_STS_REG_POS_TB_IDX |
		  FH_TCSR_CHNL_TX_BUF_STS_REG_VAL_TFDB_VALID);

	iwl_idi_set_fh_txc(txcs, FH_TCSR_CHNL_TX_CONFIG_REG(FH_SRVC_CHNL));
	data[5] = cpu_to_le32(FH_TCSR_TX_CONFIG_REG_VAL_DMA_CHNL_ENABLE |
		  FH_TCSR_TX_CONFIG_REG_VAL_DMA_CREDIT_DISABLE |
		  FH_TCSR_TX_CONFIG_REG_VAL_CIRQ_HOST_ENDTFD);

	len = iwl_idi_get_txbu_hdr_size(IWL_IDI_NUM_FH_REG_TO_CONFIG) +
		IWL_IDI_NUM_FH_REG_TO_CONFIG * IWL_IDI_FH_REG_SIZE;

	sg_idx = trans_tx->sg_list_meta[idx].used_count;
	/* check if S/G ll is full */
	if (WARN_ON(sg_idx >= IWL_IDI_TX_SG_FRAGS_MAX))
		return -ENOSR;

	dma_addr = (dma_addr_t)(headers->dma + *hdr_pos);
	iwl_idi_set_sg_desc(trans_tx, idx, sg_idx, dma_addr, len);

	trans_tx->sg_list_meta[idx].used_count++;
	return 0;
}

static int iwl_idi_load_fw_chunk(struct iwl_trans *trans,
				 u32 dst_addr,
				 struct iwl_idi_dma_ptr *headers,
				 struct iwl_idi_dma_ptr *buf, u32 byte_cnt)
{
	struct iwl_idi_trans_tx *trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);
	struct idi_sg_desc *sg_desc;
	int sg_idx, hdr_pos;
	int ret, time_left;
	u8 idx = IWL_IDI_CHAN_TO_SG_IDX(IDI_PRIMARY_CHANNEL);

	/* Build burst's TXBUs */
	ret = iwl_idi_build_fw_chunk(trans_tx, headers, byte_cnt, buf->dma,
				     &hdr_pos);
	if (ret)
		return ret;

	ret = iwl_idi_load_fw_config_fh(trans_tx, headers, byte_cnt, dst_addr,
					&hdr_pos);
	if (ret)
		return ret;

	/* seal burst and arm DMA */
	trans_tx->ucode_write_complete = false;

	sg_desc = (struct idi_sg_desc *)trans_tx->sg_list[idx].addr;
	sg_idx = trans_tx->sg_list_meta[idx].used_count;
	sg_desc[sg_idx - 1].next |= cpu_to_le32(IWL_IDI_SG_LIST_END |
						IWL_IDI_SG_LIST_INT);

	/* init refcount to 2 since in order to complete the chunk load,
	 * need to get responses from both FH and IDI
	 */
	atomic_set(&trans_tx->fw_load_refcount, 2);

	iwl_idi_dma_sync_for_device(trans, headers);
	iwl_idi_dma_sync_for_device(trans, buf);

	ret = iwl_idi_tx_dma_arm(trans_tx, idx);
	if (ret) {
		IWL_ERR(trans, "%s: iwl_idi_tx_dma_arm failed, ret %d\n",
			__func__, ret);
		return ret;
	}

	IWL_DEBUG_FW(trans, "FW chunk being loaded.\n");

	time_left = wait_event_timeout(trans_tx->ucode_write_waitq,
				       trans_tx->ucode_write_complete, 5 * HZ);

	if (!time_left) {
		IWL_ERR(trans, "Could not load FW chunk\n");
		return -ETIMEDOUT;
	}

	iwl_idi_tx_fix_sg_list(trans, idx, true);

	return 0;
}

/* Debug functionality for LhP BU - enable GPIO to SW (LMAC trace) */
static void iwl_idi_gpio_enable(struct iwl_trans *trans)
{
	#define PCL_0 0xe630020c
	#define PCL_1 0xe6300210
	#define PCL_2 0xe6300214
	#define PCL_3 0xe6300218
	#define PCL_4 0xe630021c
	#define PCL_5 0xe6300220

	iowrite32(0x66, (void __iomem *)PCL_0);
	iowrite32(0x66, (void __iomem *)PCL_1);
	iowrite32(0x66, (void __iomem *)PCL_2);
	iowrite32(0x66, (void __iomem *)PCL_3);
	iowrite32(0x66, (void __iomem *)PCL_4);
	iowrite32(0x66, (void __iomem *)PCL_5);

	idi_al_write(trans, HIDI_GPIO_SEL_AL, 0x2);
}

/**
 * FW download is done using LLS, with TXBUs in basic mode. This function
 * splits the FW image to chunks that can fit in the AL SRAM and constructs
 * TXBUs which describes them. FH configuration is done on the same burst,
 * by adding an additional TXBU which configures the service channel of the
 * FH to copy the data to the LMAC SRAM. Upon completion, the FH fires
 * interrupt which indicates that all HW components are done and the current
 * chunk has arrived to its position in LMAC SRAM.
 * @trans - the transport
 * @fw_img - the firmware image to load
 * @fw_debug_flags - firmware debug flags
 */
int iwl_idi_load_given_ucode(struct iwl_trans *trans, const struct fw_img *img,
			     u32 fw_dbg_flags)
{
	struct iwl_idi_dma_ptr chunk_headers, buf;
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	unsigned long flags;
	int i, j, chunks_num;
	int max_txbu_hdr_size, txbus_per_chunk;
	int ret = 0;
	u8 *cur_data;
	u32 dst_addr, byte_cnt;
	u32 leftover;
	bool fw_loaded = false;

	max_txbu_hdr_size =
		iwl_idi_get_txbu_hdr_size(IWL_IDI_MAX_TXC_COUNT_IN_TXBU);
	txbus_per_chunk = DIV_ROUND_UP(IWL_SRAM_SIZE_FOR_FW,
				       IWL_IDI_TXBU_CAPACITY);

	/* DMA-able buffer to hold current FW chunk */
	ret = iwl_idi_alloc_dma_mem(trans, &buf, IWL_SRAM_SIZE_FOR_FW,
				    DMA_TO_DEVICE);
	if (ret)
		return ret;


	/* Allocate room for TXBU headers and FH configuration data of each
	 * chunk. There are "txbus_per_chunk" data TXBUs, plus one for FH
	 * configuration. In addition, this memory will also hold the content
	 * of FH configuration TXBU (registers values).
	 */
	chunk_headers.size = max_txbu_hdr_size * (txbus_per_chunk + 1) +
		IWL_IDI_NUM_FH_REG_TO_CONFIG * IWL_IDI_FH_REG_SIZE;

	ret = iwl_idi_alloc_dma_mem(trans, &chunk_headers, chunk_headers.size,
				    DMA_TO_DEVICE);
	if (ret)
		goto free;

	/* set the max number of FH's pending read requests to host to 1 */
	spin_lock_irqsave(&trans_idi->reg_lock, flags);
	idi_al_write_lmac(trans, FH_TSSR_TX_MSG_CONFIG_REG, 0x1);
	spin_unlock_irqrestore(&trans_idi->reg_lock, flags);

	if (!iwl_trans_grab_nic_access(trans, false, &flags)) {
		IWL_ERR(trans, "Failed to wake up NIC\n");
		return -EBUSY;
	}

	if (IWL_IDI_LHP_FLAGS & IWL_IDI_DBG_GPIO)
		iwl_idi_gpio_enable(trans);

	idi_al_write(trans, AMFH_TG1_BUS_WAIT_EN_REG, AMFH_TG1_BUS_STALL);
	idi_al_write(trans, AMFH_TG2_BUS_WAIT_EN_REG, AMFH_TG2_BUS_STALL);

	iwl_trans_release_nic_access(trans, &flags);

	for (i = 0; i < IWL_UCODE_SECTION_MAX; i++) {
		if (!img->sec[i].data ||
		    img->sec[i].offset == CPU1_CPU2_SEPARATOR_SECTION) {
			IWL_DEBUG_FW(trans,
				     "data not valid or empty section (%d)\n",
				     i);
			break;
		}

		cur_data = (u8 *)img->sec[i].data;
		dst_addr = img->sec[i].offset;

		if (ALIGN(img->sec[i].len, 4) != img->sec[i].len) {
			ret = -EFAULT;
			goto unstall_bus;
		}

		chunks_num = DIV_ROUND_UP(img->sec[i].len,
					  IWL_SRAM_SIZE_FOR_FW);
		leftover = img->sec[i].len % IWL_SRAM_SIZE_FOR_FW;

		byte_cnt = IWL_SRAM_SIZE_FOR_FW;
		for (j = 0; j < chunks_num; j++) {
			IWL_DEBUG_FW(trans, "chunk %d, section %d\n", j, i);

			memset(chunk_headers.addr, 0, chunk_headers.size);

			if (leftover && (j == (chunks_num - 1)))
				byte_cnt = leftover;
			memcpy(buf.addr, cur_data, byte_cnt);

			ret = iwl_idi_load_fw_chunk(trans, dst_addr,
						    &chunk_headers, &buf,
						    byte_cnt);
			if (ret)
				goto unstall_bus;

			cur_data += byte_cnt;
			dst_addr += byte_cnt;
		}
	}

	fw_loaded = true;

#if IS_ENABLED(CPTCFG_IWLXVT)
	if (fw_dbg_flags & IWL_XVT_DBG_ADC_SAMP_TEST) {
		u32 val;

		if (!iwl_trans_grab_nic_access(trans, false, &flags)) {
			IWL_ERR(trans, "Failed to wake up NIC\n");
			ret = -EBUSY;
			goto free;
		}

		val = (fw_dbg_flags & IWL_XVT_DBG_ADC_SAMP_SYNC_RX) ? 2 : 1;
		idi_al_write_lmac_mem(trans, IWL_LHP_ADC_SAMP_DBG, &val, 1);
		iwl_trans_release_nic_access(trans, &flags);
	}
#endif
#ifdef CPTCFG_IWLWIFI_DEVICE_TESTMODE
	iwl_dnt_configure(trans, img);
#endif

unstall_bus:
	if (!iwl_trans_grab_nic_access(trans, false, &flags)) {
		IWL_ERR(trans, "Failed to wake up NIC\n");
		return -EBUSY;
	}

	/* release LMAC ARC from reset */
	if (fw_loaded)
		idi_al_write(trans, POWER_CFG_W1C_REG,
			     POWER_CFG_ARC_REST_REQ_MSK);

	idi_al_write(trans, AMFH_TG1_BUS_WAIT_EN_REG, AMFH_TG1_BUS_IGNORE);
	idi_al_write(trans, AMFH_TG2_BUS_WAIT_EN_REG, AMFH_TG2_BUS_IGNORE);

	iwl_trans_release_nic_access(trans, &flags);
free:

	iwl_idi_free_dma_mem(trans, &buf);
	iwl_idi_free_dma_mem(trans, &chunk_headers);

	return ret;
}

void iwl_idi_load_fw_wake_wq(struct iwl_trans *trans)
{
	struct iwl_idi_trans_tx *trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);

	if (!atomic_dec_return(&trans_tx->fw_load_refcount)) {
		trans_tx->ucode_write_complete = true;
		wake_up(&trans_tx->ucode_write_waitq);
	}
}

static void iwl_idi_tx_complete(struct idi_transaction *transaction)
{
	struct iwl_trans *iwl_trans =
		dev_get_drvdata(&transaction->peripheral->device);
	struct iwl_idi_trans_tx *trans_tx =
		IWL_TRANS_GET_IDI_TRANS_TX(iwl_trans);
	struct iwl_trans_slv *trans_slv =
		IWL_TRANS_GET_SLV_TRANS(iwl_trans);
	u32 idx = IWL_IDI_CHAN_TO_SG_IDX(transaction->idi_xfer.channel_opts);

	if (transaction->status == IDI_STATUS_FLUSH) {
		clear_bit(idx, &trans_tx->sg_list_loaded);
		wake_up(&trans_tx->stop_waitq);
		return;
	}

	if (iwl_trans->state == IWL_TRANS_NO_FW) {
		iwl_idi_load_fw_wake_wq(iwl_trans);
		return;
	}

	if (WARN_ON_ONCE(transaction->status != IDI_STATUS_COMPLETE))
		return;

	spin_lock_bh(&trans_tx->slv_tx.mem_rsrc_lock);

	iwl_idi_dma_sync_for_cpu(iwl_trans, &trans_tx->sg_list[idx]);

	iwl_idi_tx_fix_sg_list(iwl_trans, idx, false);
	clear_bit(idx, &trans_tx->sg_list_loaded);
	wake_up(&trans_tx->stop_waitq);

	spin_unlock_bh(&trans_tx->slv_tx.mem_rsrc_lock);

	queue_work(trans_slv->policy_wq, &trans_slv->policy_trigger);
}

void iwl_idi_tx_stop(struct iwl_trans *trans)
{
	struct iwl_idi_trans_tx *trans_tx =
			IWL_TRANS_GET_IDI_TRANS_TX(trans);

	if (!wait_event_timeout(trans_tx->stop_waitq, !trans_tx->sg_list_loaded,
				2*HZ))
		WARN_ONCE(1, "Failed to stop Tx");
}

int iwl_idi_tx_set_channel_config(struct iwl_trans *trans)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	struct idi_peripheral_device *pdev = trans_idi->pdev;
	struct idi_channel_config conf = {0};
	struct idi_resource *idi_res;
	struct resource *res;
	int ret, i;
	struct {
		const char *name;
		unsigned channel;
	} iwl_idi_tx_ch_map[] = {
		/* secondary channel can be added here if needed */
		{ .name = IWL_IDI_TXL_RES_NAME, .channel = IDI_PRIMARY_CHANNEL},
	};

	idi_res = &pdev->resources;

	for (i = 0; i < ARRAY_SIZE(iwl_idi_tx_ch_map); i++) {
		const char *ch_name = iwl_idi_tx_ch_map[i].name;
		unsigned ch_id = iwl_idi_tx_ch_map[i].channel;

		res = idi_get_resource_byname(idi_res, IORESOURCE_MEM, ch_name);

		if (!res) {
			IWL_ERR(trans, "failed to get resource %s\n", ch_name);
			return -EINVAL;
		}

		conf.tx_or_rx = 1;
		conf.priority = IDI_NORMAL_PRIORITY;
		conf.channel_opts = IDI_TX_EARLY_IRQ | ch_id;

		conf.dst_addr = res->start;
		conf.hw_fifo_size = resource_size(res);

		ret = idi_set_channel_config(pdev, &conf);
		if (ret) {
			IWL_ERR(trans,
				"failed in set channel config for %s, err %d\n",
				ch_name, ret);
			return ret;
		}
	}

	return 0;
}

static int iwl_idi_tx_alloc_transaction(struct iwl_idi_trans_tx *trans_tx,
					int idx)
{
	struct iwl_trans *trans = IWL_TRANS_TX_GET_TRANS(trans_tx);
	struct idi_resource *idi_res;
	struct idi_transaction *idi_trans;
	int chan = IWL_IDI_SG_IDX_TO_CHAN(idx);

	idi_res = &trans_tx->trans_idi->pdev->resources;

	idi_trans = idi_alloc_transaction(GFP_KERNEL);
	if (!idi_trans) {
		IWL_ERR(trans, "%s failed to allocate IDI transaction\n",
			__func__);
		return -ENOMEM;
	}

	idi_trans->complete = iwl_idi_tx_complete;
	idi_trans->idi_xfer.channel_opts = IDI_TX_EARLY_IRQ;
	idi_trans->idi_xfer.channel_opts |= chan;

	/* The API of IDI driver will be updated, should be phys addr */
	idi_trans->idi_xfer.desc = (void *)trans_tx->sg_list[idx].dma;
	trans_tx->transaction[idx] = idi_trans;
	return 0;
}

static void iwl_idi_tx_free_channel(struct iwl_idi_trans_tx *trans_tx, int idx)
{
	struct iwl_trans *trans = IWL_TRANS_TX_GET_TRANS(trans_tx);

	iwl_idi_free_sg_list(trans, &trans_tx->sg_list[idx]);
	if (trans_tx->transaction[idx]) {
		idi_free_transaction(trans_tx->transaction[idx]);
		trans_tx->transaction[idx] = NULL;
	}
}

/**
 * iwl_idi_tx_sg_init - initialize list of sg descriptors for a given channel.
 */
static int iwl_idi_tx_init_sg_data(struct iwl_idi_trans_tx *trans_tx, int idx)
{
	struct iwl_trans *trans = IWL_TRANS_TX_GET_TRANS(trans_tx);
	size_t sg_byte_size;
	int i, ret;
	struct idi_sg_desc *cur_virt;
	dma_addr_t cur_dma;

	/* Data Tx always requires 2 DMA descriptors per frame and HCmd needs
	 * at most IWL_MAX_CMD_TBS_PER_TFD + 1 descriptors. Each bus aggregation
	 * could contain up to sg_max_txbus elements. Allocating now the maximal
	 * needed size.
	 * FIXME: In case of separate channel for hcmds only - need to adjust
	 * the size accordingly.
	 */
	sg_byte_size = IWL_IDI_TX_SG_FRAGS_MAX * sizeof(struct idi_sg_desc);
	ret = iwl_idi_alloc_sg_list(trans, &trans_tx->sg_list[idx],
				    sg_byte_size, DMA_TO_DEVICE);
	if (unlikely(ret)) {
		IWL_WARN(trans, "%s (%d): alloc_sg_list failed, ret %d\n",
			 __func__, __LINE__, ret);
		return ret;
	}
	IWL_INFO(trans, "%s: sg_list offset %d\n", __func__,
		 trans_tx->sg_list[idx].align_offset);

	trans_tx->sg_list_meta[idx].max_txbus = IWL_IDI_TX_CH_SG_SIZE;
	trans_tx->sg_list_meta[idx].used_count = 0;
	clear_bit(idx, &trans_tx->sg_list_loaded);

	cur_virt = (struct idi_sg_desc *)trans_tx->sg_list[idx].addr;
	cur_dma = trans_tx->sg_list[idx].dma;

	/* set up next of each descriptor to point on the
	 * following descriptor */
	for (i = 0; i < IWL_IDI_TX_SG_FRAGS_MAX; i++) {
		cur_virt->next = cpu_to_le32(cur_dma +
					sizeof(struct idi_sg_desc));
		cur_dma += sizeof(struct idi_sg_desc);
		cur_virt++;
	}

	return 0;
}

static int iwl_idi_tx_chan_setup(struct iwl_idi_trans_tx *trans_tx)
{
	struct iwl_trans *trans = IWL_TRANS_TX_GET_TRANS(trans_tx);
	int i, j, ret;

	for (i = 0; i < IWL_IDI_TX_CHAN_NUM; i++) {
		ret = iwl_idi_tx_init_sg_data(trans_tx, i);
		if (unlikely(ret))
			goto error;

		ret = iwl_idi_tx_alloc_transaction(trans_tx, i);
		if (unlikely(ret))
			goto error;
	}

	return 0;

error:
	for (j = 0; j <= i; j++)
		iwl_idi_tx_free_channel(trans_tx, i);

	IWL_ERR(trans, "%s failed, ret %d\n", __func__, ret);
	return ret;
}

void iwl_idi_tx_free_txbu_mem(struct iwl_trans *trans, void **data)
{
	struct iwl_idi_trans_tx *trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);
	struct iwl_idi_tx_reclaim_info *reclaim_info = (void *)*data;
	int i;

	if (!reclaim_info)
		return;

	for (i = 0; i < reclaim_info->blocks_count; i++) {
		if (dma_unmap_len(&reclaim_info->map_data[i], len) > 0) {
			dma_unmap_single(trans->dev,
					 dma_unmap_addr(
						&reclaim_info->map_data[i],
						mapping),
					 dma_unmap_len(
						&reclaim_info->map_data[i],
						len),
					 DMA_TO_DEVICE);
		} else {
			/* mapping fragments is done sequentially */
			break;
		}
	}

	kmem_cache_free(trans_tx->reclaim_info_pool, reclaim_info);
	reclaim_info = NULL;
}

void iwl_idi_tx_clean_txbu(struct iwl_trans *trans, void *data)
{
	struct iwl_idi_trans_tx *trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);
	struct iwl_idi_tx_reclaim_info *reclaim_info =
		(struct iwl_idi_tx_reclaim_info *)data;
	struct iwl_idi_tfd *tfd;
	int i, ret;

	if (!reclaim_info)
		return;

	tfd = iwl_idi_tx_get_tfd_from_txbu(reclaim_info->txbu,
					   reclaim_info->txbu->txc_count);
	spin_lock_bh(&trans_tx->slv_tx.mem_rsrc_lock);
	for (i = 0; i < tfd->num_tbs; i++) {
		ret = iwl_slv_al_mem_pool_free(&trans_tx->slv_tx,
					       &trans_tx->slv_tx.tb_pool,
					       tfd->tbs[i].tb_idx);
		if (ret)
			IWL_WARN(trans, "%s failed to free tb pool resources\n",
				 __func__);
	}

	ret = iwl_slv_al_mem_pool_free(&trans_tx->slv_tx,
				       &trans_tx->slv_tx.tfd_pool,
				       reclaim_info->txbu->lut_value);
	if (ret)
		IWL_WARN(trans, "%s failed to free tfd pool resources\n",
			 __func__);
	spin_unlock_bh(&trans_tx->slv_tx.mem_rsrc_lock);

	iwl_idi_tx_free_txbu_mem(trans, (void **)&reclaim_info);
}

/**
* iwl_idi_tx_init - initialize IDI transport Tx (called from start device).
*/
int iwl_idi_tx_init(struct iwl_trans *trans)
{
	struct iwl_idi_trans_tx *trans_tx;
	int ret;

	/* ensure TB size is power of 2 */
	BUILD_BUG_ON_NOT_POWER_OF_2(IDI_TX_PAYLOAD_PAGE_SIZE);
	/* ensure Tx queue size is power of 2 */
	BUILD_BUG_ON_NOT_POWER_OF_2(IDI_TX_TFD_PER_Q);

	BUG_ON(trans == NULL);
	BUG_ON(trans->trans_specific == NULL);

	trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);
	trans_tx->trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);

	ret = iwl_slv_init(trans);
	if (ret)
		goto error;

	/* Turn off all SCD Tx DMA fifos */
	iwl_write_prph(trans, SCD_TXFACT, 0);

	spin_lock_init(&trans_tx->slv_tx.mem_rsrc_lock);

	/* initialize policy data, it doesn't run anything yet */
	memset(trans_tx->txbus_added_policy, 0,
	       sizeof(trans_tx->txbus_added_policy));

	ret = iwl_idi_tx_chan_setup(trans_tx);
	if (ret)
		goto error_free;

	ret = iwl_slv_al_mem_pool_init(&trans_tx->slv_tx.tfd_pool,
				       IDI_TX_TFD_PER_Q);
	if (ret)
		goto error_free;

	ret = iwl_slv_al_mem_pool_init(&trans_tx->slv_tx.tb_pool,
				       IDI_TX_TBS_IN_POOL);
	if (ret)
		goto error_free;

	trans_tx->reclaim_info_pool =
		kmem_cache_create("iwl_idi_reclaim_info",
				  sizeof(struct iwl_idi_tx_reclaim_info),
				  sizeof(void *), 0, NULL);
	if (unlikely(!trans_tx->reclaim_info_pool)) {
		ret = -ENOMEM;
		goto error_free;
	}

	trans_tx->txbu_seq_num = 0;
	memset(trans_tx->next_tfd_index, 0, sizeof(trans_tx->next_tfd_index));

	init_waitqueue_head(&trans_tx->ucode_write_waitq);
	init_waitqueue_head(&trans_tx->stop_waitq);

	return 0;

error_free:
	/* free all resources;
	 * it can handle the case of zeroed but not initialized resource
	 */

	/* FIXME: verify all un-allocated resources are zeroed */
	iwl_idi_tx_free(trans);

error:
	IWL_ERR(trans, "%s failed, ret %d\n", __func__, ret);
	return ret;
}

/**
 * iwl_idi_tx_fix_sg_list - resets descriptors of a S/G list up to used_count
 * @trans - the transport
 * @idx - the index of the required S/G list in the array of sg lists.
 * @is_fw_download - whether this function called from load ucode flow
 */
static void iwl_idi_tx_fix_sg_list(struct iwl_trans *trans, u32 idx,
				   bool is_fw_download)
{
	struct iwl_idi_trans_tx *trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);
	struct idi_sg_desc *sg_desc;
	u32 next;
	int i;

	if (!is_fw_download)
		lockdep_assert_held(&trans_tx->slv_tx.mem_rsrc_lock);

	sg_desc = (struct idi_sg_desc *)trans_tx->sg_list[idx].addr;
	next = trans_tx->sg_list[idx].dma;

	for (i = 0; i < trans_tx->sg_list_meta[idx].used_count; i++) {
		sg_desc[i].base = 0;
		sg_desc[i].size = 0;

		/* reset the next, even though it shouldn't be changed by
		 * IDI DMA except the last used one.
		 */
		if (i < IWL_IDI_TX_SG_FRAGS_MAX - 1) {
			next += sizeof(struct idi_sg_desc);
			sg_desc[i].next = cpu_to_le32(next);
		} else {
			/* set also INT in the last desc to prevent
			 * a possiblity of a stuck list.
			 */
			sg_desc[i].next = cpu_to_le32(IWL_IDI_SG_LIST_END);

			if (!is_fw_download)
				sg_desc[i].next |=
					cpu_to_le32(IWL_IDI_SG_LIST_INT);
		}
	}

	trans_tx->sg_list_meta[idx].used_count = 0;
}

/**
* iwl_idi_tx_free - free all the resources, assumes tx is stopped.
*/
void iwl_idi_tx_free(struct iwl_trans *trans)
{
	struct iwl_idi_trans_tx *trans_tx;
	int i;

	BUG_ON(trans == NULL);

	trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);
	BUG_ON(trans_tx == NULL);

	if (trans_tx->trans_idi == NULL)
		return;

	iwl_slv_free(trans);

	if (trans_tx->reclaim_info_pool) {
		kmem_cache_destroy(trans_tx->reclaim_info_pool);
		trans_tx->reclaim_info_pool = NULL;
	}

	for (i = 0; i < IWL_IDI_TX_CHAN_NUM; i++)
		iwl_idi_tx_free_channel(trans_tx, i);
}

void iwl_idi_tx_close_sg_list(struct iwl_idi_trans_tx *trans_tx, u8 idx)
{
	struct idi_sg_desc *sg_desc;

	if (unlikely(!trans_tx->sg_list_meta[idx].used_count))
		return;

	sg_desc = (struct idi_sg_desc *)trans_tx->sg_list[idx].addr;
	sg_desc[trans_tx->sg_list_meta[idx].used_count - 1].next =
			cpu_to_le32(IWL_IDI_SG_LIST_END|IWL_IDI_SG_LIST_INT);
}

static void
iwl_idi_tx_free_resources(struct iwl_idi_trans_tx *trans_tx,
			  u8 * const pool_idx_set, u8 tbs_num)
{
	int i;

	spin_lock_bh(&trans_tx->slv_tx.mem_rsrc_lock);

	iwl_slv_al_mem_pool_free(&trans_tx->slv_tx,
				 &trans_tx->slv_tx.tfd_pool,
				 pool_idx_set[0]);

	for (i = 1; i <= tbs_num; i++)
		iwl_slv_al_mem_pool_free(&trans_tx->slv_tx,
					 &trans_tx->slv_tx.tb_pool,
					 pool_idx_set[i]);
	spin_unlock_bh(&trans_tx->slv_tx.mem_rsrc_lock);
}

static int
iwl_idi_tx_get_resources(struct iwl_idi_trans_tx *trans_tx, u8 txq_id,
			 u8 * const pool_idx_set, u8 txcs_num)
{
	struct iwl_trans *trans = IWL_TRANS_TX_GET_TRANS(trans_tx);
	struct iwl_trans_slv *trans_slv = IWL_TRANS_GET_SLV_TRANS(trans);

	int free_tfds, free_tbs, i, ret;
	int idx;

	i = 0;
	spin_lock_bh(&trans_tx->slv_tx.mem_rsrc_lock);

	/* check resources availability */
	free_tbs = iwl_slv_al_mem_pool_free_count(&trans_tx->slv_tx,
						  &trans_tx->slv_tx.tb_pool);
	free_tfds = iwl_slv_al_mem_pool_free_count(&trans_tx->slv_tx,
						   &trans_tx->slv_tx.tfd_pool);
	if (!iwl_idi_tx_policy_check_alloc(trans_slv, txq_id, free_tfds,
					   free_tbs, 1, txcs_num)) {
		IWL_DEBUG_TX(trans, "%s failed, no free SRAM.\n", __func__);
		ret = -ENOMEM;
		goto error;
	}

	/* allocate TFD, store it in the first entry */
	idx = iwl_slv_al_mem_pool_alloc(&trans_tx->slv_tx,
					&trans_tx->slv_tx.tfd_pool);
	if (idx < 0) {
		ret = -ENOMEM;
		goto error;
	}

	/* TFD should be stored at index 0 */
	pool_idx_set[0] = idx;

	/* allocate TBs for each TXC (index 0 is reserved for TFD) */
	for (i = 1; i <= txcs_num; i++) {
		idx = iwl_slv_al_mem_pool_alloc(&trans_tx->slv_tx,
						&trans_tx->slv_tx.tb_pool);
		if (idx < 0) {
			ret = -ENOMEM;
			goto error;
		}
		pool_idx_set[i] = idx;
	}

	spin_unlock_bh(&trans_tx->slv_tx.mem_rsrc_lock);
	return 0;

error:
	spin_unlock_bh(&trans_tx->slv_tx.mem_rsrc_lock);

	if (i > 0)
		/* If arrived here, TFD was allocated - thus always freeing
		 * entry at idx 0*/
		iwl_idi_tx_free_resources(trans_tx, pool_idx_set, i);

	return ret;
}

static int iwl_idi_tx_attach_tfd(struct iwl_idi_trans_tx *trans_tx,
				 u8 * const pool_idx_set,
				 struct iwl_idi_txc_header *txc,
				 struct iwl_idi_tfd *tfd,
				 u16 len)
{
	u32 dest;

	/* first entry is the TFD */
	tfd->tbs[tfd->num_tbs].tb_idx = pool_idx_set[tfd->num_tbs + 1];

	dest = IDI_AL_SFDB_PAYLOAD_MEM_ADDR +
	       tfd->tbs[tfd->num_tbs].tb_idx * IDI_TX_PAYLOAD_PAGE_SIZE;

	if (WARN_ON(dest & ~IWL_IDI_TXC_DEST_MASK))
		return -EINVAL;

	txc->dest_and_flags = cpu_to_le32(dest);
	txc->len = cpu_to_le16(len);

	tfd->tbs[tfd->num_tbs].tb_len = cpu_to_le16(len);
	tfd->num_tbs++;

	return 0;
}

static int
iwl_idi_tx_add_frag_to_sg(struct iwl_trans *trans,
			  struct iwl_idi_tx_reclaim_info *reclaim_info,
			  u8 *addr, u16 len, u8 idx)
{
	struct iwl_idi_trans_tx *trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);
	struct idi_sg_desc *sg_desc;
	dma_addr_t phys_addr;
	int sg_idx;

	phys_addr = dma_map_single(trans->dev, addr, len, DMA_TO_DEVICE);

	if (unlikely(dma_mapping_error(trans->dev, phys_addr))) {
		IWL_ERR(trans, "%s (%d): dma_map_single failed.\n",
			__func__, __LINE__);
		return -ENOMEM;
	}
	sg_idx = trans_tx->sg_list_meta[idx].used_count;

	sg_desc = &((struct idi_sg_desc *)
		   trans_tx->sg_list[idx].addr)[sg_idx];

	trans_tx->sg_list_meta[idx].used_count++;

	sg_desc->base = cpu_to_le32(phys_addr);
	sg_desc->size = cpu_to_le32(len);

	dma_unmap_addr_set(&reclaim_info->map_data[reclaim_info->blocks_count],
			   mapping, phys_addr);
	dma_unmap_len_set(&reclaim_info->map_data[reclaim_info->blocks_count],
			  len, len);
	reclaim_info->blocks_count++;
	return 0;
}

/**
 * iwl_idi_tx_set_txc_chunk - set TXC and TFD entry for a single continuous
 * memory chunk.
 * @trans: IDI transport
 * @idx: channel
 * @tfd: a ponter to tfd which will be updtated
 * @txcs: array of txcs, starting at the first free TXC
 * @chunk_info: addr/len/txcs num for the chunk
 * @pool_idx_set: for each TXC(TFD), idx of the allcoated TB(TFD) in the pool
 *
 * Returns length including alignments if success, negative error otherwise.
 */
static int
iwl_idi_tx_set_txc_chunk(struct iwl_trans *trans, u8 idx,
			 struct iwl_idi_tfd *tfd,
			 struct iwl_idi_txc_header *txcs,
			 struct iwl_slv_tx_chunk_info *const chunk_info,
			 u8 *const pool_idx_set)
{
	struct iwl_trans_slv *trans_slv = IWL_TRANS_GET_SLV_TRANS(trans);
	struct iwl_idi_trans_tx *trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);
	u8 pb, pa;
	int total_len, cur_len, i, ret;

	/* DBB DMA requires that chunk addr and len will be aligned to 4 */
	pb = PADDING_BEFORE_ADDR(chunk_info->addr);
	total_len = LEN_WITH_PADDINGS(chunk_info->len, pb);
	pa = total_len - chunk_info->len - pb;

	/* Padding is added only at the start or at the end of the mem chunk.
	 * We add pb (padding before) to the first TXC and pa (padding after)
	 * to the last TXC. In all other TXCs, if any, pa=pb=0. */
	txcs[0].pb = pb;
	txcs[chunk_info->desc_num - 1].pa = pa;

	for (i = 0; i < chunk_info->desc_num ; i++) {
		/* The total length of each TXC (pa + txc_len + pb) should be
		 * aligned to 4 (LLS requirements).
		 * The default is to occupy a full TB, which we assume to
		 * be aligned. */
		cur_len = trans_slv->config.tb_size;
		if (i == 0) {
			/* The first TXC includes pb.
			 * If there is only one TXC, it will hold the whole
			 * data and will be aligned from the definition of
			 * pa and pb.
			 * Otherwise, len = (tb_size - pb) would maximize
			* the len and fill the alignment requirment. */
			if (chunk_info->desc_num == 1)
				cur_len = chunk_info->len;
			else
				cur_len -= pb;
		} else if (i == chunk_info->desc_num - 1) {
			/* The last TXC includes pa and the remainder of
			 * the data. The alignment requirment is met since
			 * total_len(= chunk_len + pa + pb) is aligned to 4. */
			cur_len = MOD_WITHOUT_ZERO(chunk_info->len + pb,
						   trans_slv->config.tb_size);
		}

		ret = iwl_idi_tx_attach_tfd(trans_tx, pool_idx_set, &txcs[i],
					    tfd, cur_len);
		if (ret) {
			IWL_ERR(trans, "%s: failed to attach TXC %d to TFD\n",
				__func__, i);
			return ret;
		}
	}

	return total_len;
}

/**
 * iwl_idi_set_txbu_and_tfd - fill TXBU header and TXC describing the TFD
 */
static
int iwl_idi_set_txbu_and_tfd(struct iwl_trans *trans,
			     struct iwl_idi_txbu_header *txbu,
			     struct iwl_idi_txc_header *txcs,
			     u8 txq_id,
			     struct iwl_slv_tx_dtu_meta *txbu_meta,
			     u8 *const pool_idx_set)
{
	struct iwl_idi_trans_tx *trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);
	u32 dest;

	/* fill in TXBU */
	txbu->signature = cpu_to_le16(IWL_IDI_TXBU_SIGNATURE);

	/* Note, txbu->lut_value was allocated at the start */

	/* FIXME: wrap around behavior should be defined */
	txbu->lut_value = pool_idx_set[0];
	txbu->seq_num = cpu_to_le16(trans_tx->txbu_seq_num);
	txbu->txc_count = txbu_meta->total_desc_num;
	txbu->flags = IWL_IDI_TXBU_EX_BIT;
	txbu->tfd_index = trans_tx->next_tfd_index[txq_id];
	txbu->queue = txq_id;
	txbu->byte_cnt_value = txbu_meta->scd_byte_cnt;

	/* TXC for compressed TFD */
	dest = IDI_AL_SFDB_TFD_POOL_BASE_ADDR +
	       txbu->lut_value * sizeof(struct iwl_idi_tfd);
	if (unlikely(dest & ~IWL_IDI_TXC_DEST_MASK)) {
		spin_lock_bh(&trans_tx->slv_tx.mem_rsrc_lock);
		iwl_slv_al_mem_pool_free(&trans_tx->slv_tx,
					 &trans_tx->slv_tx.tfd_pool,
					 txbu->lut_value);
		spin_unlock_bh(&trans_tx->slv_tx.mem_rsrc_lock);
		/* FIXME: free resources */
		IWL_ERR(trans, "%s failed, invalid destination 0x%x\n",
			__func__, dest);
		return -EINVAL;
	}

	/* set the first TXC which always references TFD */
	txcs[0].dest_and_flags = cpu_to_le32(dest);
	txcs[0].len = cpu_to_le16(sizeof(struct iwl_idi_tfd));

	return 0;
}

static
int iwl_idi_tx_process_txbu_info(struct iwl_idi_trans_tx *trans_tx,
				 u8 idx, u8 txq_id,
				 struct iwl_slv_tx_dtu_meta *txbu_meta,
				 struct iwl_idi_tx_reclaim_info *reclaim_info,
				 u8 * const pool_idx_set)
{
	struct iwl_trans *trans = IWL_TRANS_TX_GET_TRANS(trans_tx);
	struct iwl_idi_txbu_header *txbu;
	struct iwl_idi_txc_header *txcs;
	struct iwl_idi_tfd *tfd;
	u8 *sg_addr[IWL_MAX_CMD_TBS_PER_TFD + 1];
	u32 sg_len[IWL_MAX_CMD_TBS_PER_TFD + 1];
	u16 total_hdr_size;
	u8 i, pb;
	int ret;

	/* set up txbu headers and align txbu address */
	pb = PADDING_BEFORE_ADDR(txbu_meta->chunk_info[0].addr);
	total_hdr_size =
		iwl_idi_tx_get_txbu_hdr_size(txbu_meta->total_desc_num) + pb;
	txbu = (struct iwl_idi_txbu_header *)
			(txbu_meta->chunk_info[0].addr - total_hdr_size);

	memset(txbu, 0, total_hdr_size);

	txcs = iwl_idi_tx_get_txcs_from_txbu(txbu);
	tfd = iwl_idi_tx_get_tfd_from_txbu(txbu, txbu_meta->total_desc_num);

	reclaim_info->txbu = txbu;

	/* set up txbu header and the txc for tfd */
	ret = iwl_idi_set_txbu_and_tfd(trans, txbu, txcs, txq_id,
				       txbu_meta, pool_idx_set);
	if (ret) {
		IWL_ERR(trans, "%s failed to set txbu and txc-tfd\n", __func__);
		goto error_free;
	}

	/* skip the first TXC which is always refers to the TFD */
	txcs++;
	for (i = 0; i < txbu_meta->chunks_num; i++) {
		sg_len[i] = iwl_idi_tx_set_txc_chunk(trans, idx, tfd, txcs,
						     &txbu_meta->chunk_info[i],
						     pool_idx_set);
		if (sg_len[i] < 0)
			goto error_free;
		if (i == 0) {
			/* for the first chunk need to map header as well */
			sg_len[i] += total_hdr_size;
			sg_addr[i] = (u8 *)txbu;
		} else {
			sg_addr[i] = ALIGN_ADDR_BACKWARDS(
				txbu_meta->chunk_info[i].addr);
		}

		/* move to the first free txc */
		txcs += txbu_meta->chunk_info[i].desc_num;
	}

	/* chunk mapping (cache flush) can be done only when all the data is
	 * in place, thus the separate loop
	 */
	for (i = 0; i < txbu_meta->chunks_num; i++) {
		ret = iwl_idi_tx_add_frag_to_sg(trans, reclaim_info,
						sg_addr[i], sg_len[i], idx);
		if (ret) {
			IWL_ERR(trans, "%s failed to set sg descriptor\n",
				__func__);
			goto error_free;
		}
	}

	trans_tx->txbu_seq_num++;

	/* increment index w.r.t. wrapping at queue size */
	INC_TFD_IDX(trans_tx->next_tfd_index[txq_id]);

	return 0;

error_free:
	/* FIXME */
	iwl_idi_tx_clean_txbu(trans, reclaim_info);
	return ret;
}

/**
 * iwl_idi_tx_calc_txcs_num() - calculate number of TXCs for a chunk.
 * Besides IDI bus alignment requirements, LLS TXC needs (pb + len + pa)
 * to be aligned to 4. The reason for this is that IDIG can move only
 * dwords, so if LLS requests it to bring less than 4 bytes, it will still
 * bring a dword aligned chunk.
 * @trans: transport
 * @chunk_info: chunk description
 */
void iwl_idi_tx_calc_txcs_num(struct iwl_trans *trans,
			      struct iwl_slv_tx_chunk_info *chunk_info)
{
	struct iwl_trans_slv *trans_slv = IWL_TRANS_GET_SLV_TRANS(trans);
	u32 total_len;
	u8 pb;

	/*
	 * When chunk fits one TB, it means that it requires only one TXC
	 * which implies that this TXC is a separate S/G entry - thus
	 * pb + len + pa is aligned to 4 by the requirement of the DBB DMA.
	 */
	if (chunk_info->len < trans_slv->config.tb_size) {
		chunk_info->desc_num = 1;
		return;
	}

	pb = PADDING_BEFORE_ADDR(chunk_info->addr);
	total_len = LEN_WITH_PADDINGS(chunk_info->len, pb);

	chunk_info->desc_num = DIV_ROUND_UP(total_len,
					    trans_slv->config.tb_size);
}

/**
 * iwl_idi_tx_add_burst - single entry point for policy. Send cmd/data to device
 * and move from waiting to sent queue if sent with success.
 * @trans_tx: the transport
 * @idx: channel to send
 * @txq_id: the queue
 * Returns 0 if sent, error code otherwise.
 */
int iwl_idi_tx_add_burst(struct iwl_trans_slv *trans_slv, u8 idx, u8 txq_id)
{
	struct iwl_idi_trans_tx *trans_tx =
		IWL_TRANS_SLV_GET_IDI_TRANS_TX(trans_slv);
	struct iwl_trans *trans = IWL_TRANS_SLV_GET_IWL_TRANS(trans_slv);
	bool rx_dma_expect_idle = false;

	u8 pool_idx_set[IWL_IDI_MAX_TXC_COUNT_IN_TXBU + 1];
	struct iwl_slv_txq_entry *txq_entry;
	int ret;

	txq_entry = iwl_slv_txq_pop_entry(trans_slv, txq_id);
	if (!txq_entry)
		return -ENOBUFS;

	if (WARN_ON(txq_entry->dtu_meta.total_desc_num >
		    trans_slv->config.max_data_desc_count)) {
		IWL_ERR(trans, "%s failed, wrong desc count %d\n",
			__func__, txq_entry->dtu_meta.total_desc_num);
		return -EINVAL;
	}

	/* pass txcs_num not including the one for TFD */
	ret = iwl_idi_tx_get_resources(trans_tx, txq_id,
				       pool_idx_set,
				       txq_entry->dtu_meta.total_desc_num);
	/* if no resources, just return - it's valid state */
	if (ret)
		goto error_pushback;

	txq_entry->reclaim_info =
		kmem_cache_alloc(trans_tx->reclaim_info_pool, GFP_ATOMIC);
	if (unlikely(!txq_entry->reclaim_info)) {
		ret = -ENOMEM;
		goto error_free_rsc;
	}

	/* In IDI TXBU, the first txc is describing the TFD.
	 * total_desc_num counts only the number of descriptors
	 * needed for data, hence we need to add one */
	txq_entry->dtu_meta.total_desc_num++;

	memset(txq_entry->reclaim_info, 0,
	       sizeof(struct iwl_idi_tx_reclaim_info));

	if (trans_slv->cmd_queue == txq_id) {
		struct iwl_slv_tx_cmd_entry *cmd_entry;

		IWL_SLV_TXQ_GET_ENTRY(txq_entry, cmd_entry);
		/* we need to stop the RxDMA if this cmd will idle the bus */
		if (!(d0i3_debug & IWL_D0I3_DBG_KEEP_BUS) &&
		    cmd_entry->hcmd_meta.flags & CMD_MAKE_TRANS_IDLE) {
			iwl_idi_rx_dma_expect_idle(trans, true);
			rx_dma_expect_idle = true;
		}
	}

	ret = iwl_idi_tx_process_txbu_info(trans_tx, idx, txq_id,
					   &txq_entry->dtu_meta,
					   (struct iwl_idi_tx_reclaim_info *)
						txq_entry->reclaim_info,
					   pool_idx_set);
	if (ret)
		/* process_txbu_info free all resoures in case of failure */
		goto error_pushback;


	/* move this entry from waiting to sent only if succeeded to process */
	iwl_slv_txq_add_to_sent(trans_slv, txq_id, txq_entry);

	return 0;

error_free_rsc:
	iwl_idi_tx_free_resources(trans_tx,
				  pool_idx_set,
				  txq_entry->dtu_meta.total_desc_num);

error_pushback:
	if (rx_dma_expect_idle)
		iwl_idi_rx_dma_expect_idle(trans, false);

	iwl_slv_txq_pushback_entry(trans_slv, txq_id, txq_entry);
	return ret;
}
