/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2007 - 2014 Intel Corporation. All rights reserved.
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
 * Copyright(c) 2005 - 2014 Intel Corporation. All rights reserved.
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
#include <linux/bug.h>

#include "iwl-debug.h"
#include "iwl-csr.h"
#include "iwl-fh.h"

#include "idi_hals.h"
#include "idi_internal.h"
#include "idi_host_csr.h"
#include "idi_tx.h"
#include "idi_rx.h"
#include "idi_irq_csr.h"

#define AL_STABILIZATION_TIMEOUT 500 /* microseconds */
#define IDI_POLL_INTERVAL 1000	/* microseconds */

#define IDI_AL_READY_TIMEOUT 500000
#define IDI_POLLING_TIMEOUT 250000

void idi_al_write8(struct iwl_trans *trans, u32 ofs, u8 val)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);

	lockdep_assert_held(&trans_idi->reg_lock);
	iowrite8(val, (void *)(trans_idi->al_hw_base + ofs));
	udelay(1);
}

/*
 * write a u32 to a AL register or to an offset in AL's SRAM
 */
void idi_al_write(struct iwl_trans *trans, u32 ofs, u32 val)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);

	lockdep_assert_held(&trans_idi->reg_lock);
	iowrite32(val, (void *)(trans_idi->al_hw_base + ofs));
	udelay(1);
}

/*
 * read a u32 from a AL register or from an offset in AL's SRAM
 */
u32 idi_al_read(struct iwl_trans *trans, u32 ofs)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);

	if (ofs != READY_FOR_CODE_LOAD_STT_REG)
		lockdep_assert_held(&trans_idi->reg_lock);

	udelay(1);
	return ioread32((void *)(trans_idi->al_hw_base + ofs));
}

int idi_poll_al_bit(struct iwl_trans *trans, u32 addr, u32 mask, int timeout)
{
	int t = 0;

	do {
		if ((idi_al_read(trans, addr) & mask) == mask)
			return t;
		udelay(IDI_POLL_INTERVAL);
		t += IDI_POLL_INTERVAL;
	} while (t < timeout);

	return -ETIMEDOUT;
}

static int idi_al_check_tg_idle(struct iwl_trans *trans)
{
	int ret;

	udelay(1);
	ret = idi_poll_al_bit(trans, AMFH_STT_REG, AMFH_STT_REG_TG1_MSK,
			      IDI_POLLING_TIMEOUT);
	if (WARN_ON_ONCE(ret < 0)) {
		IWL_ERR(trans, "Error waiting for AMFH_STT_REG_TG1_MSK\n");
		return -EBUSY;
	}

	return 0;
}

/*
 * write AL's SRAM in DWORDs. If %buf is %NULL, then the memory
 * will be zeroed.
 * dwords is the size of the buffer.
 */
static void idi_al_write_mem(struct iwl_trans *trans, u32 addr,
			     u32 *buf, int dwords)
{
	int i, offs;

	for (i = 0; i < dwords; i++) {
		offs = addr + i * sizeof(u32);
		idi_al_write(trans, offs, buf ? buf[i] : 0);
	}
}

void idi_al_release_access(struct iwl_trans *trans)
{
	idi_al_write(trans, TG_ACCESS_WKUP_REG, 0);
	udelay(1);
}

int idi_al_request_access(struct iwl_trans *trans, bool silent)
{
	struct iwl_trans_idi *trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	u32 val;
	int ret;

	if (WARN_ON_ONCE(trans_idi->power_state != PM_ENABLE))
		return -EINVAL;

	/* wake up WLAN */
	idi_al_write(trans, TG_ACCESS_WKUP_REG, TG_WKUP_REQ_MSK);
	ret = idi_poll_al_bit(trans, TG_ACCESS_WKUP_REG, TG_WKUP_ALLOWED_MSK,
			      IDI_POLLING_TIMEOUT);
	WARN_ONCE(ret > 25000,
		  "NIC wake up time is longer than expected: %d ms\n",
		  ret/1000);
	if (ret < 0) {
		if (!silent) {
			val = idi_al_read(trans, TG_ACCESS_WKUP_REG);
			WARN_ONCE(1,
				  "Timeout waiting for hardware access (TG_ACCESS_WKUP 0x%08x)\n",
				  val);
		}
		return -EBUSY;
	}

	/* check that TG is idle */
	ret = idi_poll_al_bit(trans, AMFH_STT_REG, AMFH_STT_REG_TG1_MSK,
			      IDI_POLLING_TIMEOUT);
	if (ret < 0) {
		idi_al_release_access(trans);
		if (!silent) {
			val = idi_al_read(trans, AMFH_STT_REG);
			WARN_ONCE(1,
				  "Timeout waiting for TG to be idle (AMFH_STT_REG 0x%08x)\n",
				  val);
		}
		return -EBUSY;
	}
	return 0;
}

void idi_al_write_lmac8(struct iwl_trans *trans, u32 ofs, u8 val)
{
	/* TODO: */
}

int __idi_al_map_csr_addr(u32 ofs)
{
	/*
	 * FIXME: Most of the CSR registers in IDI have different names from
	 * the PCIe names, and no proper mapping is to be found.
	 * Hence, this mapping is tentative, prone to errors and\or
	 * incomplete.
	 */
	switch (ofs) {
	case CSR_INT:
		return IDI_INTA_CSR_W1C;
	case CSR_INT_MASK:
		return IDI_INTA_CSR_MASK;
	case CSR_FH_INT_STATUS:
		/* Unsure. match by address. */
		return IDI_FH_INT_CSR_W1C;
	case CSR_HW_REV:
		return IDI_HW_REV_CSR;
	case CSR_RESET:
		return IDI_CSR_RESET;
	default:
		WARN(1, "Unrecoginzed CSR address: 0x%x\n", ofs);
		return -EINVAL;
	}
}

u32 idi_al_read_fh(struct iwl_trans *trans, u32 ofs)
{
	struct iwl_trans_idi *trans_idi;
	int ret;

	trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	lockdep_assert_held(&trans_idi->reg_lock);

	ret = idi_al_check_tg_idle(trans);
	if (ret < 0)
		return ret;

	idi_al_read(trans, AMFH_TG1_RD_BASE_ADDR + ofs);

	/* wait until the data is ready and read it */
	ret = idi_poll_al_bit(trans, IDI_INDIRECT_DATA_4RD_RDY_REG,
			      IDI_INDIRECT_RD_DATA_IS_READY_MSK,
			      IDI_POLLING_TIMEOUT);
	if (ret < 0) {
		WARN_ONCE(1, "Timeout waiting for reading from FH\n");
		/* OPEN: what to do in case of failure? */
		return ret;
	}
	return idi_al_read(trans, IDI_INDIRECT_DATA_4RD);
}

void idi_al_write_lmac(struct iwl_trans *trans, u32 ofs, u32 val)
{
	struct iwl_trans_idi *trans_idi;
	int ret, addr;

	trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	lockdep_assert_held(&trans_idi->reg_lock);

	/*
	 * There are only two areas in the LMAC that are accessible
	 * directly: FH registers and CSRs.
	 * FH registers accessible via the AMFH, thus we have to verify
	 * that there is power to the nic (reg_lock should be locked).
	 * CSRs: some of the CSRs have eqivalent registers in AL. Others doesn't
	 * exist or inaccessible for the host.
	 */
	if ((ofs >= FH_MEM_LOWER_BOUND) && (ofs < FH_MEM_UPPER_BOUND)) {
		addr = AMFH_TG1_WR_BASE_ADDR + ofs;

		ret = idi_al_check_tg_idle(trans);
		if (ret < 0)
			return;
	} else { /* ofs is CSR */
		addr = __idi_al_map_csr_addr(ofs);
		if (addr < 0)
			return;
	}
	idi_al_write(trans, addr, val);
}

u32 idi_al_read_lmac(struct iwl_trans *trans, u32 ofs)
{
	struct iwl_trans_idi *trans_idi;
	int addr;

	/*
	 * There are only two areas in the LMAC that are accessible
	 * directly: FH registers and CSRs.
	 * FH registers accessible via the AMFH, thus we have to verify
	 * that there is power to the nic (reg_lock should be locked).
	 * CSRs: some of the CSRs have eqivalent registers in AL. Others doesn't
	 * exist or inaccessible for the host. no need to grab nic access.
	 */

	trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	lockdep_assert_held(&trans_idi->reg_lock);

	if ((ofs >= FH_MEM_LOWER_BOUND) && (ofs < FH_MEM_UPPER_BOUND)) {
		return idi_al_read_fh(trans, ofs);
	} else { /* ofs is CSR */
		addr = __idi_al_map_csr_addr(ofs);
		if (addr < 0)
			return addr;
		return idi_al_read(trans, addr);
	}
}

void idi_al_write_lmac_prph(struct iwl_trans *trans, u32 ofs, u32 val)
{
	struct iwl_trans_idi *trans_idi;
	int ret;

	trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	lockdep_assert_held(&trans_idi->reg_lock);

	ret = idi_al_check_tg_idle(trans);
	if (ret < 0)
		return;

	ofs = (ofs & 0x000FFFFF) | (3 << 24);
	idi_al_write(trans, AMFH_TG1_WR_BASE_ADDR + HBUS_TARG_PRPH_WADDR, ofs);
	idi_al_write(trans, AMFH_TG1_WR_BASE_ADDR + HBUS_TARG_PRPH_WDAT, val);
}

u32 idi_al_read_lmac_prph(struct iwl_trans *trans, u32 ofs)
{
	struct iwl_trans_idi *trans_idi;
	int ret;

	trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	lockdep_assert_held(&trans_idi->reg_lock);

	ret = idi_al_check_tg_idle(trans);
	if (ret < 0)
		return ret;

	ofs = (ofs & 0x000FFFFF) | (3 << 24);
	/* write the address to read, and read it to triger the AMFH */
	idi_al_write(trans, AMFH_TG1_WR_BASE_ADDR + HBUS_TARG_PRPH_RADDR, ofs);
	idi_al_read(trans, AMFH_TG1_RD_BASE_ADDR + HBUS_TARG_PRPH_RDAT);

	/* wait until the data is ready and read it */
	ret = idi_poll_al_bit(trans, IDI_INDIRECT_DATA_4RD_RDY_REG,
			      IDI_INDIRECT_RD_DATA_IS_READY_MSK,
			      IDI_POLLING_TIMEOUT);
	if (ret < 0) {
		WARN_ONCE(1, "Timeout waiting for TG prph read\n");
		/* OPEN: what to do in case of failure? */
		return ret;
	}
	return idi_al_read(trans, IDI_INDIRECT_DATA_4RD);
}

int idi_al_read_lmac_mem(struct iwl_trans *trans, u32 addr, void *buf,
			 int dwords)
{
	struct iwl_trans_idi *trans_idi;
	int offs, ret;
	u32 *vals = buf;

	trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	lockdep_assert_held(&trans_idi->reg_lock);

	ret = idi_al_check_tg_idle(trans);
	if (ret < 0)
		return ret;

	/* write the address to read */
	idi_al_write(trans, AMFH_TG1_WR_BASE_ADDR + HBUS_TARG_MEM_RADDR, addr);

	/* FIXME: verify auto increment as in PCIe */
	for (offs = 0; offs < dwords; offs++) {
		/* read to triger the AMFH */
		idi_al_read(trans, AMFH_TG1_RD_BASE_ADDR + HBUS_TARG_MEM_RDAT);

		/* wait until the data is ready and read it */
		ret = idi_poll_al_bit(trans, IDI_INDIRECT_DATA_4RD_RDY_REG,
				      IDI_INDIRECT_RD_DATA_IS_READY_MSK,
				      IDI_POLLING_TIMEOUT);
		if (ret < 0) {
			WARN_ONCE(1, "Timeout waiting for AL SRAM read\n");
			/* OPEN: what to do in case of failure? */
			return ret;
		}
		vals[offs] = idi_al_read(trans, IDI_INDIRECT_DATA_4RD);
	}

	return ret;
}

void idi_al_write_lmac_mem(struct iwl_trans *trans, u32 addr, const void *buf,
			   int dwords)
{
	struct iwl_trans_idi *trans_idi;
	int offs, ret;
	const u32 *vals = buf;

	trans_idi = IWL_TRANS_GET_IDI_TRANS(trans);
	lockdep_assert_held(&trans_idi->reg_lock);

	ret = idi_al_check_tg_idle(trans);
	if (ret < 0)
		return;

	idi_al_write(trans, AMFH_TG1_WR_BASE_ADDR + HBUS_TARG_MEM_WADDR, addr);

	/* FIXME: verify auto increment as in PCIe */
	for (offs = 0; offs < dwords; offs++)
		idi_al_write(trans, AMFH_TG1_WR_BASE_ADDR + HBUS_TARG_MEM_WDAT,
			     vals ? vals[offs] : 0);
}

static void idi_al_enable_lls_err_intr(struct iwl_trans *trans)
{
	/* TODO: */
}

static void idi_al_init_lls(struct iwl_trans *trans)
{
	u32 ll_addr;

	/* configure LL_PTR for TXH DMA channel (channel #1) */
	ll_addr = LLS_TXH_ADDR + LLS_LL_OFFSET;
	idi_al_write(trans, DMA_CH1_LL_PTR, ll_addr);
	idi_al_write(trans, CH1_START_BD_ADDR, ll_addr | CH_START_BD_ADDR_MSK);
	idi_al_write(trans, DMA_CH1_CMD, BUILD_DMA_LL_CMD(DMA_CMD_NOP, 0));

	/* configure LL_PTR for TXL DMA channel (channel #2) */
	ll_addr = LLS_TXL_ADDR + LLS_LL_OFFSET;
	idi_al_write(trans, DMA_CH2_LL_PTR, ll_addr);
	idi_al_write(trans, CH2_START_BD_ADDR, ll_addr | CH_START_BD_ADDR_MSK);
	idi_al_write(trans, DMA_CH2_CMD, BUILD_DMA_LL_CMD(DMA_CMD_NOP, 0));

	/* configure watchdogs for both channels */
	idi_al_write(trans, TXL_WDT_KEYPASS_ADDR, WDT_KEYPASS_VALUE);
	idi_al_write(trans, TXL_WDT_THRESHOLD_ADDR, TXL_WDT_THRESHOLD_VALUE);
	/* enable TXL WDT interrupt */
	idi_al_write(trans, TXL_WDT_MASK_ADDR, 0x1);
	/* disable configuration by writing incorrect keypass */
	idi_al_write(trans, TXL_WDT_KEYPASS_ADDR, 0x0);

	idi_al_write(trans, TXH_WDT_KEYPASS_ADDR, WDT_KEYPASS_VALUE);
	idi_al_write(trans, TXH_WDT_THRESHOLD_ADDR, TXH_WDT_THRESHOLD_VALUE);
	/* enable TXH WDT interrupt */
	idi_al_write(trans, TXH_WDT_MASK_ADDR, 0x1);
	/* disable configuration by writing incorrect keypass */
	idi_al_write(trans, TXH_WDT_KEYPASS_ADDR, 0x0);

	idi_al_enable_lls_err_intr(trans);
}

static void idi_al_init_sfdb(struct iwl_trans *trans)
{
	int i;

	/* verify that the SRAM allocation is valid */
	BUILD_BUG_ON((RXB_OFFSET + SIZEOF_CONST_SRAM_AREAS) >
		     IDI_AL_SFDB_SRAM_SIZE);

	BUILD_BUG_ON(IDI_AL_SFDB_TFD_POOL_OFFSET &
		     ~IDI_TFD_DB_OFFSET_REG_MSK);

	BUILD_BUG_ON(IDI_AL_SFDB_BC_TABLE_OFFSET &
		     ~IDI_BC_TABLE_OFFSET_REG_MSK);

	BUILD_BUG_ON(IDI_AL_SFDB_PAYLOAD_MEM_OFFSET &
		     ~IDI_PAYLOAD_OFFSET_REG_MSK);

	/* initialize SFDB LUT with invalid (NULL) values */
	for (i = 0; i < IDI_AL_SFDB_LUT_SIZE; i += sizeof(u32))
		idi_al_write(trans, IDI_AL_SFDB_LUT_BASE_ADDR + i, 0xFFFFFFFF);

	idi_al_write(trans, IDI_TFD_DB_OFFSET_REG, IDI_AL_SFDB_TFD_POOL_OFFSET);
	idi_al_write(trans, IDI_BC_TABLE_OFFSET_REG,
		     IDI_AL_SFDB_BC_TABLE_OFFSET);
	idi_al_write(trans, IDI_PAYLOAD_OFFSET_REG,
		     IDI_AL_SFDB_PAYLOAD_MEM_OFFSET);

	BUILD_BUG_ON((IDI_TX_PAYLOAD_PAGE_SIZE != 512) &&
		     (IDI_TX_PAYLOAD_PAGE_SIZE != 256));

	idi_al_write(trans, IDI_PAYLOAD_SIZE_REG, IDI_PAYLOAD_SIZE_REG_VALUE);
}

static void idi_al_init_rx_ll(struct iwl_trans *trans)
{
	struct dma_ll_entry rx_kck0_ll[6];
	struct dma_ll_entry rx_kck1_ll[6];
	struct dma_ll_entry rx_next_ll[3];
	struct dma_ll_entry rx_last_ll[6];
	u32 rx_next_trailer[2];
	u32 rx_last_trailer[4];

	int overridden_cmd, count_pos;

	/* RX KCK0 LL initialization */
	/* configure IDIG to transfer kck0 signature to the RX FIFO */
	rx_kck0_ll[0].src = DWORD_SIZE_W_LAST_ADDR;
	rx_kck0_ll[0].dst = IDIG_RXF_LOAD_DATA_REG;
	rx_kck0_ll[0].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, 4);

	rx_kck0_ll[1].src = KCK0_SIGNATURE_ADDR;
	rx_kck0_ll[1].dst = IDIG_RX_FIFO;
	rx_kck0_ll[1].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, 4);

	/* enable WDT */
	rx_kck0_ll[2].src = LLS_WDT_KEY_VALUE_ADDR;
	rx_kck0_ll[2].dst = RX_WDT_KEYPASS_ADDR;
	rx_kck0_ll[2].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, 4);

	rx_kck0_ll[3].src = LLS_WDT_ACTIVE_VALUE_ADDR;
	rx_kck0_ll[3].dst = RX_WDT_ABORT_ADDR;
	rx_kck0_ll[3].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, 4);

	/* Here we need to disable writing to the watchdog by writing
	 * incorrect key.
	 * we used AMFH_RESUME_ENABLE beacse this address will always holds 1,
	 * which is different from the valid signature */
	rx_kck0_ll[4].src = AMFH_RESUME_ENABLE_ADDR;
	rx_kck0_ll[4].dst = RX_WDT_KEYPASS_ADDR;
	rx_kck0_ll[4].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, 4);

	rx_kck0_ll[5].src = AMFH_RESUME_ENABLE_ADDR;
	rx_kck0_ll[5].dst = AMFH_RX_RESUME_REG;
	rx_kck0_ll[5].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE_STOP, 4);


	/* build RX KCK1 LL */
	/* first, the IDIG should move the length of the buffer to the
	 * RX_FIFO (4 bytes). */
	rx_kck1_ll[0].src = DWORD_SIZE_W_LAST_ADDR;
	rx_kck1_ll[0].dst = IDIG_RXF_LOAD_DATA_REG;
	rx_kck1_ll[0].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, 4);

	rx_kck1_ll[1].src = LENBUF_ADDR;
	rx_kck1_ll[1].dst = IDIG_RX_FIFO;
	rx_kck1_ll[1].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, 4);

	/* The third descriptor in the LL configures the lenght of the
	 * fourth descriptor (the actual data).
	 * Hence, the dst address of the second desc is the dma_cmd field
	 * of the 4th one. */
	overridden_cmd = RX_KCK1_LL_ADDR + (4 * sizeof(struct dma_ll_entry)) +
			 offsetof(struct dma_ll_entry, dma_cmd);
	count_pos = (DMA_CNT_POS / 8);

	rx_kck1_ll[2].src = LENBUF_ADDR;
	/* need to override only the count field of the dma_cmd */
	rx_kck1_ll[2].dst = overridden_cmd + count_pos;
	rx_kck1_ll[2].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, SIZEOF_DMA_CNT);

	/* Now, the IDIG should move the actual data to the RX_FIFO */
	rx_kck1_ll[3].src = LENBUF_ADDR;
	rx_kck1_ll[3].dst = IDIG_RXF_LOAD_DATA_REG;
	rx_kck1_ll[3].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, 4);

	rx_kck1_ll[4].src = RXB_ADDR;
	rx_kck1_ll[4].dst = IDIG_RX_FIFO;
	/* count is configured by the second descriptor, 0 is a filler */
	rx_kck1_ll[4].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, 0);

	rx_kck1_ll[5].src = AMFH_RESUME_ENABLE_ADDR;
	rx_kck1_ll[5].dst = AMFH_RX_RESUME_REG;
	rx_kck1_ll[5].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE_STOP, 4);


	/* RX NEXT LL initialization */
	/* the IDIG should move the 'next trailer' to RX_FIFO (8 Bytes) */
	rx_next_ll[0].src = TWO_DWORD_SIZE_W_LAST_ADDR;
	rx_next_ll[0].dst = IDIG_RXF_LOAD_DATA_REG;
	rx_next_ll[0].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, 4);

	rx_next_ll[1].src = RX_NEXT_TRAILER_ADDR;
	rx_next_ll[1].dst = IDIG_RX_FIFO;
	rx_next_ll[1].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, 8);

	rx_next_ll[2].src = AMFH_RESUME_ENABLE_ADDR;
	rx_next_ll[2].dst = AMFH_RX_RESUME_REG;
	rx_next_ll[2].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE_STOP, 4);


	/* RX LAST LL initialization */
	/* configure IDIG to transfer the 'last trailer' to the
	 * RX_FIFO (16 Bytes) */
	rx_last_ll[0].src = FOUR_DWORD_SIZE_W_LAST_ADDR;
	rx_last_ll[0].dst = IDIG_RXF_LOAD_DATA_REG;
	rx_last_ll[0].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, 4);

	rx_last_ll[1].src = RX_LAST_TRAILER_ADDR;
	rx_last_ll[1].dst = IDIG_RX_FIFO;
	rx_last_ll[1].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, 16);

	/* disable WDT */
	rx_last_ll[2].src = LLS_WDT_KEY_VALUE_ADDR;
	rx_last_ll[2].dst = RX_WDT_KEYPASS_ADDR;
	rx_last_ll[2].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, 4);

	rx_last_ll[3].src = LLS_WDT_ABORT_VALUE_ADDR;
	rx_last_ll[3].dst = RX_WDT_ABORT_ADDR;
	rx_last_ll[3].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, 4);

	/* Here we need to disable writing to the watchdog by writing
	 * incorrect key.
	 * we used AMFH_RESUME_ENABLE beacse this address will always hold 1,
	 * which is different from the valid signature */
	rx_last_ll[4].src = AMFH_RESUME_ENABLE_ADDR;
	rx_last_ll[4].dst = RX_WDT_KEYPASS_ADDR;
	rx_last_ll[4].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE, 4);

	rx_last_ll[5].src = AMFH_RESUME_ENABLE_ADDR;
	rx_last_ll[5].dst = AMFH_RX_RESUME_REG;
	rx_last_ll[5].dma_cmd = BUILD_DMA_LL_CMD(DMA_CMD_MOVE_STOP, 4);

	/* 'next trailer' and 'last trailer' initialization */
	rx_next_trailer[0] = RX_NEXT_INDICATOR;
	rx_next_trailer[1] = 0; /* dummy */

	rx_last_trailer[0] = RX_LAST_INDICATOR;
	rx_last_trailer[1] = RX_BRST_SIGNATURE;
	rx_last_trailer[2] = RX_DMA_HALT_CMD;
	rx_last_trailer[3] = RX_DUMMY_SIGNATURE;

	/* Write all to memory */
	idi_al_write_mem(trans, RX_NEXT_TRAILER_ADDR, (u32 *)rx_next_trailer,
			 sizeof(rx_next_trailer) / sizeof(u32));
	idi_al_write_mem(trans, RX_LAST_TRAILER_ADDR, (u32 *)rx_last_trailer,
			 sizeof(rx_last_trailer) / sizeof(u32));
	idi_al_write_mem(trans, RX_KCK0_LL_ADDR, (u32 *)rx_kck0_ll,
			 sizeof(rx_kck0_ll) / sizeof(u32));
	idi_al_write_mem(trans, RX_KCK1_LL_ADDR, (u32 *)rx_kck1_ll,
			 sizeof(rx_kck1_ll) / sizeof(u32));
	idi_al_write_mem(trans, RX_NEXT_LL_ADDR, (u32 *)rx_next_ll,
			 sizeof(rx_next_ll) / sizeof(u32));
	idi_al_write_mem(trans, RX_LAST_LL_ADDR, (u32 *)rx_last_ll,
			 sizeof(rx_last_ll) / sizeof(u32));
}

static void idi_al_init_amfh(struct iwl_trans *trans)
{
	idi_al_write(trans, AMFH_RESUME_ENABLE_ADDR, 0x1);
	idi_al_write(trans, KCK0_SIGNATURE_ADDR, RX_KCK0_SIGNATURE);

	idi_al_write(trans, DWORD_SIZE_ADDR, 0x4);
	idi_al_write(trans, TWO_DWORD_SIZE_ADDR, 0x8);
	idi_al_write(trans, FOUR_DWORD_SIZE_ADDR, 0x10);

	idi_al_write(trans, DWORD_SIZE_W_LAST_ADDR, (0x4 |
		     IDIG_RXF_LOAD_DATA_LAST_MSK));
	idi_al_write(trans, TWO_DWORD_SIZE_W_LAST_ADDR, (0x8 |
		     IDIG_RXF_LOAD_DATA_LAST_MSK));
	idi_al_write(trans, FOUR_DWORD_SIZE_W_LAST_ADDR, (0x10 |
		     IDIG_RXF_LOAD_DATA_LAST_MSK));

	idi_al_init_rx_ll(trans);

	idi_al_write(trans, AMFH_RX_KCK0_EN_REG, 0);
	idi_al_write(trans, AMFH_RXB_SIZE_REG, AMFH_RXB_SIZE_8K);

	idi_al_write(trans, AMFH_EN_REG, AMFH_EN_REG_MSK);

	idi_al_write(trans, AMFH_RX_KCK1_EN_REG, AMFH_RX_KCK1_EN_REG_MSK);

	idi_al_write(trans, AMFH_RX_CMPLT1_EN_REG, AMFH_RX_CMPLT1_EN_REG_MSK);
	idi_al_write(trans, AMFH_RX_CMPLT2_EN_REG, AMFH_RX_CMPLT2_EN_REG_MSK);

	idi_al_write(trans, AMFH_RX_GKCK_EN_REG, AMFH_RX_GKCK_EN_REG_KCK1_MSK |
					  AMFH_RX_GKCK_EN_REG_COMPL1_MSK |
					  AMFH_RX_GKCK_EN_REG_COMPL2_MSK);

	idi_al_write(trans, AMFH_TG1_BUS_WAIT_EN_REG, AMFH_TG1_BUS_IGNORE);
	idi_al_write(trans, AMFH_TG2_BUS_WAIT_EN_REG, AMFH_TG2_BUS_IGNORE);

	/* write to the AMFH the number of data buffers allowed in RX link
	 * list (equivalent to the number of frames in the S/G list) */
	BUILD_BUG_ON((SG_LIST_MAX_SIZE & ~AMFH_RX_MAX_FRM_COUNT_REG_MSK));
	idi_al_write(trans, AMFH_RX_MAX_FRM_COUNT_REG, SG_LIST_MAX_SIZE);

	idi_al_write(trans, AMFH_RXB_BASE_ADDR_REG, RXB_ADDR);
	idi_al_write(trans, AMFH_RX_LENBUF_ADDR_REG, LENBUF_ADDR);

	BUILD_BUG_ON(DMA_CH0_LL_PTR & ~AMFH_RX_KCK0_ADDR_REG_MSK);
	BUILD_BUG_ON(DMA_CH0_LL_PTR & ~AMFH_RX_KCK1_ADDR_REG_MSK);
	BUILD_BUG_ON(DMA_CH0_LL_PTR & ~AMFH_RX_CMPLT1_ADDR_REG_MSK);
	BUILD_BUG_ON(DMA_CH0_LL_PTR & ~AMFH_RX_CMPLT2_ADDR_REG_MSK);
	BUILD_BUG_ON(DMA_CH0_LL_PTR & ~AMFH_RX_GKCK_ADDR_REG_MSK);

	idi_al_write(trans, AMFH_RX_KCK0_ADDR_REG, DMA_CH0_LL_PTR);
	idi_al_write(trans, AMFH_RX_KCK0_DATA_REG, RX_KCK0_LL_ADDR);

	idi_al_write(trans, AMFH_RX_KCK1_ADDR_REG, DMA_CH0_LL_PTR);
	idi_al_write(trans, AMFH_RX_KCK1_DATA_REG, RX_KCK1_LL_ADDR);

	idi_al_write(trans, AMFH_RX_CMPLT1_ADDR_REG, DMA_CH0_LL_PTR);
	idi_al_write(trans, AMFH_RX_CMPLT1_DATA_REG, RX_NEXT_LL_ADDR);

	idi_al_write(trans, AMFH_RX_CMPLT2_ADDR_REG, DMA_CH0_LL_PTR);
	idi_al_write(trans, AMFH_RX_CMPLT2_DATA_REG, RX_LAST_LL_ADDR);

	idi_al_write(trans, AMFH_RX_GKCK_ADDR_REG, DMA_CH0_CMD);
	idi_al_write(trans, AMFH_RX_GKCK_DATA_REG, (DMA_CH_EN_MSK |
						    DMA_LL_EN_MSK));
}

/*
 * Init AL yDMA to mode SGM (Scatter-Gather mode)
 */
static void idi_al_init_ydma(struct iwl_trans *trans)
{
	int size_of_dma_desc;

	size_of_dma_desc = sizeof(struct dma_ll_entry) / sizeof(__le32);
	/* set global control register. Note that '0' means DREQ is active */
	idi_al_write(trans, DMA_CONTROL, size_of_dma_desc << DMA_CONTROL_DSCLGT_POS);

	/* for channels 0-2 (RX, TX-high, TX-low), enable DREQ and
	 * DACK and set polarity. The rest of the channels are not used
	 * in the operational flow. */
	idi_al_write(trans, DMA_CHANNELS_3210_CONTROL,
		     DMA_CH0_DREQEN_MSK |
		     DMA_CH0_DACKP_MSK |
		     DMA_CH0_DRQP_MSK |
		     DMA_CH1_DREQEN_MSK |
		     DMA_CH1_DACKP_MSK |
		     DMA_CH1_DRQP_MSK |
		     DMA_CH2_DREQEN_MSK |
		     DMA_CH2_DACKP_MSK |
		     DMA_CH2_DRQP_MSK);

	idi_al_write(trans, DMA_CHANNELS_654_CONTROL, 0);
}

/* TODO: Clock gates are an HW feature which saves power.
 * This function should be called here in the operational flow,
 * but it wasn't tested yet during the HW BU. */
static void __maybe_unused idi_al_activate_clock_gates(struct iwl_trans *trans)
{
	/* Enable clock gates feature by writing 0x0 to relevant registers */
	idi_al_write(trans, SW_CLK_BYPASS, 0);
	idi_al_write(trans, HIDI_CTL_SW_CLK_BYPASS_AL, 0);
	idi_al_write(trans, IDI_CSR_SW_CLK_BYPASS, 0);
}

/*
 * for debug purposes: polling READY_FOR_CODE_LOAD instead of
 * waiting for an interrupt.
 */
static int __maybe_unused idi_al_wait_al_ready_poll(struct iwl_trans *trans)
{
	return idi_poll_al_bit(trans, READY_FOR_CODE_LOAD_STT_REG,
			       READY_FOR_CODE_LOAD_MSK, IDI_AL_READY_TIMEOUT);
}

/*
 * Wait for ready for code load indication from APMG.
 * TODO: remove __maybe_unused flag.
 */
static int __maybe_unused idi_al_wait_al_ready_intr(struct iwl_trans *trans)
{
	struct iwl_idi_trans_tx *trans_tx;
	int time_left;

	trans_tx = IWL_TRANS_GET_IDI_TRANS_TX(trans);

	/* Enable 'ready_for_code_load' interrupt*/
	trans_tx->ucode_write_complete = false;

	idi_al_write(trans, READY_FOR_CODE_LOAD_INT_MASK,
		     READY_FOR_CODE_LOAD_INT_MASK_BIT);

	time_left = wait_event_timeout(trans_tx->ucode_write_waitq,
				       trans_tx->ucode_write_complete, 5 * HZ);
	if (!time_left) {
		IWL_ERR(trans,
			"READY_FOR_CODE_LOAD interrupt has not been received\n");
		return -ETIMEDOUT;
	}

	/* to be on the safe side: mask 'ready_for_code_load' interrupt */
	idi_al_write(trans, READY_FOR_CODE_LOAD_INT_MASK, 0);

	return time_left;
}

static void idi_al_enable_tg_read(struct iwl_trans *trans)
{
	/* In order to preform target access reading, need to configure the
	AMFH where to write the data */
	BUILD_BUG_ON((IDI_INDIRECT_DATA_4RD & ~AMFH_TG1_RD_RSP_ADDR_MSK));

	idi_al_write(trans, AMFH_TG1_RD_RSP_ADDR, IDI_INDIRECT_DATA_4RD);
}

/*
 * idi_al_init - Init the AL layer.
 */
int idi_al_init(struct iwl_trans *trans)
{
	unsigned long flags;
	int t;

	/* Wait for indication that AL registers ready to be written */
	/* TODO: while the HW bring up we are using the polling-based option;
	 * the operational flow should use the interrupt-based one.
	 * (idi_al_wait_al_ready_intr) */
	t = idi_al_wait_al_ready_poll(trans);
	if (t < 0) {
		WARN_ONCE(1, "Timeout waiting for HW to be ready\n");
		return t;
	}

	if (!iwl_trans_grab_nic_access(trans, false, &flags)) {
		IWL_ERR(trans, "Failed to wake up NIC\n");
		return -EBUSY;
	}

	/* mask all AL interrupts. */
	/* idi_al_write(trans, WLAN_IRQ_O_MASK, 0); */

	idi_al_enable_tg_read(trans);

	/* Clock gates are an HW feature which saves power.
	 * This function should be called here in the operational flow,
	 * but it wasn't tested yet during the HW BU. */
	/* idi_al_activate_clock_gates(trans); */

	idi_al_init_ydma(trans);

	idi_al_init_sfdb(trans);

	idi_al_init_amfh(trans);

	idi_al_init_lls(trans);

	iwl_trans_release_nic_access(trans, &flags);

	return 0;
}
