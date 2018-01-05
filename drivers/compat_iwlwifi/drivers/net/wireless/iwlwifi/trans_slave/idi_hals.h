/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2007 - 2014 Intel Corporation. All rights reserved.
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

#ifndef __iwl_idi_hals_h__
#define __iwl_idi_hals_h__

#include "idi_al.h"

/*
 * Direct-access block in SFDB address space:
 *	1) LUT database		|\
 *	2) TFD database		| related defines are in idi_al.h
 *	3) byte count table	|/
 *	4) payload memory (TBs)	|/
 *	5) RXB  - controlled by the LMAC via AMFH (4K)
 *	6) misc - mainly AMFH configuration values
 *	7) reserved for debuf buffers (2K)
 *	8) LLS buffers (0.5K)
 * Note that the size of parts 5-8 in constant, while 1-4 is up to
 * configuration, with the limitation that all parts together can not
 * exceed SFDB_SRAM_SIZE
 */

#define RXB_SIZE	(0x1000)
#define RXB_OFFSET		(ALIGN((IDI_AL_SFDB_PAYLOAD_MEM_OFFSET +\
					IDI_AL_SFDB_PAYLOAD_MEM_SIZE), 4))
#define RXB_ADDR		(IDI_AL_SFDB_BASE_ADDR + RXB_OFFSET)

/* MISC: AMFH and RX-related data in SFBD_SRAM */
#define SFDB_MISC_BASE_ADDR	(RXB_ADDR + RXB_SIZE)

#define RX_KCK1_LL_ADDR			(SFDB_MISC_BASE_ADDR)
#define RX_KCK0_LL_ADDR			(SFDB_MISC_BASE_ADDR + 0x80)
#define RX_NEXT_LL_ADDR			(SFDB_MISC_BASE_ADDR + 0x100)
#define RX_LAST_LL_ADDR			(SFDB_MISC_BASE_ADDR + 0x180)

#define DWORD_SIZE_ADDR			(SFDB_MISC_BASE_ADDR + 0x200)
#define TWO_DWORD_SIZE_ADDR		(SFDB_MISC_BASE_ADDR + 0x204)
#define FOUR_DWORD_SIZE_ADDR		(SFDB_MISC_BASE_ADDR + 0x208)

#define DWORD_SIZE_W_LAST_ADDR		(SFDB_MISC_BASE_ADDR + 0x20c)
#define TWO_DWORD_SIZE_W_LAST_ADDR	(SFDB_MISC_BASE_ADDR + 0x210)
#define FOUR_DWORD_SIZE_W_LAST_ADDR	(SFDB_MISC_BASE_ADDR + 0x214)

#define AMFH_RESUME_ENABLE_ADDR		(SFDB_MISC_BASE_ADDR + 0x218)
#define KCK0_SIGNATURE_ADDR		(SFDB_MISC_BASE_ADDR + 0x21c)
#define LENBUF_ADDR			(SFDB_MISC_BASE_ADDR + 0x220)
#define RX_NEXT_TRAILER_ADDR		(SFDB_MISC_BASE_ADDR + 0x280)
#define RX_LAST_TRAILER_ADDR		(SFDB_MISC_BASE_ADDR + 0x2c0)
#define SFDB_MISC_SIZE	(0x300)

#ifdef CPTCFG_LHP_DBGM
#define DEBUG_BUF_SIZE	(0x800)
#else
#define DEBUG_BUF_SIZE  (0)
#endif

/* LLS internal buffers used to copy data from the stream, TXL_HEADERS_MEM
 * at 0x8BE00 and TXH_HEADERS_MEM at 0x8BF00 each of size 256B according to LLS
 * MAS.
 */
#define LLS_BUFS_SIZE	(0x200)

/* Probably the 0x100 bytes at address 0x8BD00 are used for FW_MISC_PATCH, need
 * to verify that
 */
#define SFDB_SIZE_RESERVED (0x100)

#define SIZEOF_CONST_SRAM_AREAS (RXB_SIZE + SFDB_MISC_SIZE +\
				 SFDB_SIZE_RESERVED +\
				 DEBUG_BUF_SIZE + LLS_BUFS_SIZE)

/* AL addresses outside the SFDB_SRAM */

/* IDIG address space */
#define IDIG_BASE_ADDR		(0x20000)

#define IDIG_TXL_FIFO		(IDIG_BASE_ADDR + 0x00000)
#define IDIG_TXH_FIFO		(IDIG_BASE_ADDR + 0x08000)
#define IDIG_RX_FIFO		(IDIG_BASE_ADDR + 0x10000)

/* LLS address space */
#define LLS_BASE_ADDR		(0x40000)

#define LLS_TXL_ADDR		(LLS_BASE_ADDR + 0x0)
#define LLS_TXH_ADDR		(LLS_BASE_ADDR + 0x200)
#define LLS_LL_OFFSET		(0x60)

#define LLS_WDT_ABORT_VALUE_ADDR	(LLS_BASE_ADDR + 0x40)
#define LLS_WDT_ACTIVE_VALUE_ADDR	(LLS_BASE_ADDR + 0x44)
#define LLS_WDT_KEY_VALUE_ADDR		(LLS_BASE_ADDR + 0x48)

/* Watchdogs address space */
#define TXL_WDT_BASE_ADDR	(0x72000)
#define TXH_WDT_BASE_ADDR	(0x73000)
#define RX_WDT_BASE_ADDR	(0x74000)

#define WDT_KEYPASS_VALUE	(0x1a2b3c4d)
/* FIXME: determine resonable threshold values for the watch dogs.
 * The counter frequency is 115Mhz, hence a cycle time equals to ~8.7 nsec.
 * The initial value of the counter is 0xffffffff and it descending until
 * THRESHOLD_VALUE (then it fires an interrupt).
 * 0xfffffde8 equlas to ((0xffffffff - 0xfffffde8) * 8.7nsec) = 4.6 usec.
 */
#define TXL_WDT_THRESHOLD_VALUE	(0xfffffde8)
#define TXH_WDT_THRESHOLD_VALUE	(0xfffffde8)

#define WDT_ABORT_OFFSET	(0x0000)
#define WDT_MASK_OFFSET		(0x0004)
#define WDT_THRESHOLD_OFFSET	(0x0010)
#define WDT_KEYPASS_OFFSET	(0x0014)

#define TXL_WDT_ABORT_ADDR	(TXL_WDT_BASE_ADDR + WDT_ABORT_OFFSET)
#define TXL_WDT_MASK_ADDR	(TXL_WDT_BASE_ADDR + WDT_MASK_OFFSET)
#define TXL_WDT_THRESHOLD_ADDR	(TXL_WDT_BASE_ADDR + WDT_THRESHOLD_OFFSET)
#define TXL_WDT_KEYPASS_ADDR	(TXL_WDT_BASE_ADDR + WDT_KEYPASS_OFFSET)

#define TXH_WDT_ABORT_ADDR	(TXH_WDT_BASE_ADDR + WDT_ABORT_OFFSET)
#define TXH_WDT_MASK_ADDR	(TXH_WDT_BASE_ADDR + WDT_MASK_OFFSET)
#define TXH_WDT_THRESHOLD_ADDR	(TXH_WDT_BASE_ADDR + WDT_THRESHOLD_OFFSET)
#define TXH_WDT_KEYPASS_ADDR	(TXH_WDT_BASE_ADDR + WDT_KEYPASS_OFFSET)

#define RX_WDT_ABORT_ADDR	(RX_WDT_BASE_ADDR + WDT_ABORT_OFFSET)
#define RX_WDT_MASK_ADDR	(RX_WDT_BASE_ADDR + WDT_MASK_OFFSET)
#define RX_WDT_THRESHOLD_ADDR	(RX_WDT_BASE_ADDR + WDT_THRESHOLD_OFFSET)
#define RX_WDT_KEYPASS_ADDR	(RX_WDT_BASE_ADDR + WDT_KEYPASS_OFFSET)

/* Other defines */
#define RX_NEXT_INDICATOR	(0x4)
#define RX_LAST_INDICATOR	(0x8)
#define RX_BRST_SIGNATURE	(0x900ddeed)
#define RX_DMA_HALT_CMD		(0x3)
#define RX_DUMMY_SIGNATURE	(0xabbecafe)

struct dma_ll_entry {
	u32 src;
	u32 dst;
	u32 dma_cmd;
} __packed;

#define __BUILD_DMA_CMD(thcnt, warp, intr, eot, ch_en, ll_en, cmd, cnt) \
	(((thcnt) << DMA_THCNT_POS) |\
	((warp) << DMA_WRAP_EN_POS) |\
	((intr) << DMA_INT_EN_POS) |\
	((eot) << DMA_EOT_EN_POS) |\
	((ch_en) << DMA_CH_EN_POS) |\
	((ll_en) << DMA_LL_EN_POS) |\
	((cmd) << DMA_CMD_POS) |\
	((cnt) << DMA_CNT_POS))

#define BUILD_DMA_LL_CMD(cmd, cnt) \
	__BUILD_DMA_CMD(DMA_CMD_THCNT_16B, 0,\
			0, 0, 1, 1, (cmd), (cnt))

#endif /* __iwl_idi_hals_h__ */
