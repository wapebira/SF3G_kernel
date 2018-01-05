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

#ifndef __iwl_idi_al_h__
#define __iwl_idi_al_h__

#include "iwl-trans.h"
#include "idi_host_csr.h"

#define IDI_TX_MAX_Q_COUNT		(16)
#define IDI_TX_TFD_PER_Q		(32)
#define IWL_IDI_MAX_TB_COUNT_IN_TFD	(20)

/* Possible values for IDI_TX_PAYLOAD_PAGE_SIZE: 256 or 512 */
#define IDI_TX_PAYLOAD_PAGE_SIZE	(256)
#define IDI_PAYLOAD_SIZE_REG_VALUE	(IDI_PAYLOAD_SIZE_256)

/**
 * struct iwl_idi_tb - definition of compressed TB for SFDB.
 * @tb_idx: index in the SFDB TB pool, not a real address.
 * @tb_len: the length of the memory used
 */
struct iwl_idi_tb {
	u8 tb_idx;
	__le16 tb_len;
} __packed;

struct iwl_idi_tfd {
	u8 __reserved[3];
	u8 num_tbs;
	struct iwl_idi_tb tbs[IWL_IDI_MAX_TB_COUNT_IN_TFD];
} __packed;

#define IDI_TX_TBS_IN_POOL		(155)
#define IDI_TX_TFDS_IN_POOL		(32)

/* SFDB (AL SRAM) address space */
#define IDI_AL_SFDB_BASE_ADDR		(0x80000)
#define IDI_AL_SFDB_SRAM_SIZE		(0xc000)

/* TX-related structs in the SFDB */
#define IDI_AL_SFDB_LUT_OFFSET		(0x0)
/* each TFD has 1 byte in the LUT, therefore: */
#define IDI_AL_SFDB_LUT_SIZE		(IDI_TX_MAX_Q_COUNT * IDI_TX_TFD_PER_Q)
#define IDI_AL_SFDB_LUT_BASE_ADDR	(IDI_AL_SFDB_BASE_ADDR +\
					 IDI_AL_SFDB_LUT_OFFSET)

#define IDI_AL_SFDB_TFD_POOL_OFFSET	(ALIGN((IDI_AL_SFDB_LUT_OFFSET +\
						IDI_AL_SFDB_LUT_SIZE), 4))
#define IDI_AL_SFDB_TFD_POOL_SIZE	(IDI_TX_TFDS_IN_POOL *\
					 sizeof(struct iwl_idi_tfd))
#define IDI_AL_SFDB_TFD_POOL_BASE_ADDR	(IDI_AL_SFDB_BASE_ADDR +\
					 IDI_AL_SFDB_TFD_POOL_OFFSET)

#define IDI_AL_SFDB_BC_TABLE_OFFSET	(ALIGN((IDI_AL_SFDB_TFD_POOL_OFFSET +\
						IDI_AL_SFDB_TFD_POOL_SIZE), 4))
/* needs 2 bytes for each TFD in each queue*/
#define IDI_AL_SFDB_BC_TABLE_SIZE	(IDI_TX_MAX_Q_COUNT * 2 *\
					 IDI_TX_TFD_PER_Q)
#define IDI_AL_SFDB_BC_TABLE_BASE_ADDR	(IDI_AL_SFDB_BASE_ADDR +\
					 IDI_AL_SFDB_BC_TABLE_OFFSET)

#define IDI_AL_SFDB_PAYLOAD_MEM_OFFSET	(ALIGN((IDI_AL_SFDB_BC_TABLE_OFFSET +\
						IDI_AL_SFDB_BC_TABLE_SIZE), 4))
#define IDI_AL_SFDB_PAYLOAD_MEM_SIZE	(IDI_TX_PAYLOAD_PAGE_SIZE *\
					 IDI_TX_TBS_IN_POOL)
#define IDI_AL_SFDB_PAYLOAD_MEM_ADDR	(IDI_AL_SFDB_BASE_ADDR +\
					 IDI_AL_SFDB_PAYLOAD_MEM_OFFSET)

#define MEMORY_MAP_BC_BASE_ADDRESS	(0)
#define IDI_AL_SFDB_VTFD_BASE_ADDR	(0xc0000)
#define IDI_AL_SFDB_VTFD_SIZE		(0x1000)
#define IDI_AL_SFDB_VIRT_BC_BASE_ADDR	(0xe0000)

/* AMFH address space */
#define AMFH_BASE_ADDR	(0x60000)

#define AMFH_TG1_WR_BASE_ADDR	(AMFH_BASE_ADDR)
#define AMFH_TG1_RD_BASE_ADDR	(AMFH_BASE_ADDR + 0x2000)
#define AMFH_TG2_WR_BASE_ADDR	(AMFH_BASE_ADDR + 0x4000)
#define AMFH_TG2_RD_BASE_ADDR	(AMFH_BASE_ADDR + 0x6000)

/* other defines */
#define RX_KCK0_SIGNATURE	(0x0b00cafe)

void idi_al_write8(struct iwl_trans *trans, u32 ofs, u8 val);
void idi_al_write(struct iwl_trans *trans, u32 ofs, u32 val);
u32 idi_al_read(struct iwl_trans *trans, u32 ofs);

void idi_al_write_lmac(struct iwl_trans *trans, u32 ofs, u32 val);
u32 idi_al_read_lmac(struct iwl_trans *trans, u32 ofs);

int idi_al_request_access(struct iwl_trans *trans, bool silent);
void idi_al_release_access(struct iwl_trans *trans);

void idi_al_write_lmac_prph(struct iwl_trans *trans, u32 ofs, u32 val);
u32 idi_al_read_lmac_prph(struct iwl_trans *trans, u32 ofs);

int idi_al_read_lmac_mem(struct iwl_trans *trans, u32 addr, void *buf,
			 int dwords);
void idi_al_write_lmac_mem(struct iwl_trans *trans, u32 addr, const void *buf,
			   int dwords);

int idi_al_init(struct iwl_trans *trans);

#endif
