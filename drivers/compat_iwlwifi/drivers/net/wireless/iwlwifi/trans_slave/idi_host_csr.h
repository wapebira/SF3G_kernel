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

#ifndef __IDI_HOST_CSR_REGS__H__
#define __IDI_HOST_CSR_REGS__H__

/*
 * Register : IDI_RXF_LOAD_DATA
 * RX task size
 * [15:0]	IDI_RXF_LOAD_DATA_TASK_SIZE	RW	task size
 * [16:16]	IDI_RXF_LOAD_DATA_LAST_TASK	RW	last task
 * [31:17]	Reserved.
 */
#define IDIG_RXF_LOAD_DATA_REG			(0x38138)

#define IDIG_RXF_LOAD_DATA_SIZE_POS		(0)
#define IDIG_RXF_LOAD_DATA_LAST_POS		(16)

#define IDIG_RXF_LOAD_DATA_SIZE_MSK		(0xffff)
#define IDIG_RXF_LOAD_DATA_LAST_MSK		(0x10000)

/* Register:  POWER_CFG_W1S
 * bit[0:0]: hpet arcrstrq - ARC reset request
 * ARC reset request from the driver.
 * The main purpose of this bit is to release LMAC ARC from reset after uCode
 * load during boot flow. It also causes a reset of the whole arc logic.
 *		1 assert reset.
 *		0 no reset request.
 * The driver will use this bit throughout the process of ucode load.
 * When the driver releases this bit, we move to active (D0a) state.
*/
#define POWER_CFG_W1S_REG		(0x39350)
#define POWER_CFG_W1C_REG		(0x39354)

#define POWER_CFG_ARC_REST_REQ_MSK		(0x1)

/*
 * Register : TG_ACCESS_WKUP
 * [0:0]	TARGET_WKUP	RW	Indication that driver requests target
 *					access. Affects APMG only when in
 *					power-save.
 * [1:1]	TG_ALLOWED	RO	set by HW when the HOST is allowed to
 *					perform target access to the AL domain.
 * [31:2] Reserved
 */
#define TG_ACCESS_WKUP_REG		(0x3935c)

#define TG_WKUP_REQ_POS			(0)
#define TG_WKUP_ALLOWED_POS		(1)

#define TG_WKUP_REQ_MSK			(0x1)
#define TG_WKUP_ALLOWED_MSK		(0x2)

/*
 * Register : HIDI_CTL_SW_CLK_BYPASS_IDI
 * bypass the HW dynamic clock gates.
 * this register is not gated, the other registers are under the gated clock.
 * [0:0]	IDI_CSR_SW_CLK_BYPASS	RW
 */
#define IDI_CSR_SW_CLK_BYPASS		(0x393ac)
#define IDI_CSR_SW_CLK_BYPASS_MSK	(0x1)

/*
 * Register : IDI_INDIRECT_DATA_4RD
 * indirect data. RO.
 */
#define IDI_INDIRECT_DATA_4RD			(0x39800)

/*
 * Register : IDI_INDIRECT_DATA_4RD_RDY
 * indirect data is ready
 * [0:0]	DATA_IS_READY	RO
 */
#define IDI_INDIRECT_DATA_4RD_RDY_REG		(0x39804)
#define IDI_INDIRECT_RD_DATA_IS_READY_POS	(0)
#define IDI_INDIRECT_RD_DATA_IS_READY_MSK	(0x1)

/*
 * Register : AMFH_EN
 * Enable the AMFH FSM's to exit IDLE state.
 * [0:0]	EN	RW
 */
#define AMFH_EN_REG		(0x6c000)

#define AMFH_EN_REG_POS		(0)
#define AMFH_EN_REG_MSK		(0x1)

/*
 * Register : AMFH_STT
 * Returns FSM's to IDLE. Please do not write to this register.
 * [0:0]	AMFH_STT_TX	RO	TX FSM at IDLE.
 * [1:1]	AMFH_STT_RX	RO	RX FSM at IDLE.
 * [2:2]	AMFH_STT_TG1	RO	TG1 FSM at IDLE.
 * [3:3]	AMFH_STT_TG2	RO	TG2 FSM at IDLE.
 * [4:4]	AMFH_STT_MI	RO	MI FSM to IDLE.
 * [5:5]	AMFH_STT_DBG	RO	DBG FSM at IDLE.
 *
 * NOTE: Bit[2] of AMFH_STT is equal to bit[0] of AMFH_TG_DRV_IDLE.
 */
#define AMFH_STT_REG		(0x6c008)

#define AMFH_STT_TX_POS		(0)
#define AMFH_STT_REG_RX_POS	(1)
#define AMFH_STT_REG_TG1_POS	(2)
#define AMFH_STT_REG_TG2_POS	(3)
#define AMFH_STT_REG_MI_POS	(4)
#define AMFH_STT_REG_DBG_POS	(5)

#define AMFH_STT_REG_TX_MSK	(0x1)
#define AMFH_STT_REG_RX_MSK	(0x2)
#define AMFH_STT_REG_TG1_MSK	(0x4)
#define AMFH_STT_REG_TG2_MSK	(0x8)
#define AMFH_STT_REG_MI_MSK	(0x10)
#define AMFH_STT_REG_DBG_MSK	(0x20)

/*
 * Register : AMFH_RX_LENBUF_ADDR
 * AMFH writea to LENBUF address the Length of kick1 transaction (Frame
 * Payload). This writes occurs before KICK1 event.
 * Need to be written by SW when AMFH is not enabled.
 * [31:0]	LENBUF_ADDRESS	RW
 */
#define AMFH_RX_LENBUF_ADDR_REG		(0x6c020)

/*
 * Register : AMFH_RX_KCK0_EN
 * 0 - disable.
 * 1 - enable.
 * AMFH will fire KICK0 event after an entire frame (or burst) was copied to
 * the RXBUFF and the length header and control trailer was written to the
 * buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [0:0]	KCK0_ENABLE	RW
 * [31:1]	Reserved.
 */
#define AMFH_RX_KCK0_EN_REG		(0x6c024)

#define AMFH_RX_KCK0_EN_REG_POS		(0)
#define AMFH_RX_KCK0_EN_REG_MSK		(0x1)

/*
 * Register : AMFH_RX_KCK0_DATA
 * AMFH will fire KICK0 event after an entire frame (or burst) was copied to
 * the RXBUFF and the length header and control trailer was written to the
 * buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [31:0]	KCK0_DATA	RW
 */
#define AMFH_RX_KCK0_DATA_REG		(0x6c028)

/*
 * Register : AMFH_RX_KCK0_ADDR
 * AMFH will fire KICK0 event after an entire frame (or burst) was copied to
 * the RXBUFF and the length header and control trailer was written to the
 * buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [19:0]	KCK0_ADDRESS	RW
 * [31:20] Reserved.
 */
#define AMFH_RX_KCK0_ADDR_REG		(0x6c02c)

#define AMFH_RX_KCK0_ADDR_REG_POS	(0)
#define AMFH_RX_KCK0_ADDR_REG_MSK	(0xfffff)

/*
 * Register : AMFH_RX_KCK1_EN
 * 0- disable.
 * 1 - enable.
 * AMFH will fire kick1 event after an entire frame (or burst) was copied to
 * the RXBUFF and the length header and control trailer was written to the
 * buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [0:0]	KCK1_ENABLE	RW
 * [31:1]	Reserved.
 */
#define AMFH_RX_KCK1_EN_REG		(0x6c030)

#define AMFH_RX_KCK1_EN_REG_POS		(0)
#define AMFH_RX_KCK1_EN_REG_MSK		(0x1)

/*
 * Register : AMFH_RX_KCK1_DATA
 * AMFH will fire kick1 event after an entire frame (or burst) was copied to
 * the RXBUFF and the length header and control trailer was written to the
 * buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [31:0]	KCK1_DATA	RW
 */
#define AMFH_RX_KCK1_DATA_REG		(0x6c034)

/*
 * Register : AMFH_RX_KCK1_ADDR
 * AMFH will fire kick1 event after an entire frame (or burst) was copied to
 * the RXBUFF and the length header and control trailer was written to the
 * buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [19:0]	KCK1_ADDRESS	RW
 * [31:20] Reserved.
 */
#define AMFH_RX_KCK1_ADDR_REG		(0x6c038)

#define AMFH_RX_KCK1_ADDR_REG_POS	(0)
#define AMFH_RX_KCK1_ADDR_REG_MSK	(0xfffff)

/*
 * Register : AMFH_RX_KCK2_EN
 * 0- disable.
 * 1 - enable.
 * AMFH will fire kick2 event after an entire frame (or burst) was copied to
 * the RXBUFF and the length header and control trailer was written to the
 * buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [0:0]	KCK2_ENABLE	RW
 * [31:1]	Reserved.
 */
#define AMFH_RX_KCK2_EN_REG		(0x6c03c)

#define AMFH_RX_KCK2_EN_REG_POS		(0)
#define AMFH_RX_KCK2_EN_REG_MSK		(0x1)

/*
 * Register : AMFH_RX_KCK2_DATA
 * AMFH will fire kick2 event after an entire frame (or burst) was copied to
 * the RXBUFF and the length header and control trailer was written to the
 * buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [31:0]	KCK2_DATA	RW
 */
#define AMFH_RX_KCK2_DATA_REG		(0x6c040)

/*
 * Register : AMFH_RX_KCK2_ADDR
 * AMFH will fire kick2 event after an entire frame (or burst) was copied to
 * the RXBUFF and the length header and control trailer was written to the
 * buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [19:0]	KCK2_ADDRESS	RW
 * [31:20] Reserved.
 */
#define AMFH_RX_KCK2_ADDR_REG		(0x6c044)

#define AMFH_RX_KCK2_ADDR_REG_POS	(0)
#define AMFH_RX_KCK2_ADDR_REG_MSK	(0xfffff)

/*
 * Register : AMFH_RX_CMPLT1_EN
 * 0 - disable.
 * 1 - enable.
 * AMFH will fire complete1 event after an entire frame (or burst) was copied
 * to the RXBUFF and the length header and control trailer was written to the
 * buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [0:0]	CMPLT1_ENABLE	RW
 * [31:1]	Reserved.
 */
#define AMFH_RX_CMPLT1_EN_REG		(0x6c048)

#define AMFH_RX_CMPLT1_EN_REG_POS	(0)
#define AMFH_RX_CMPLT1_EN_REG_MSK	(0x1)

/*
 * Register : AMFH_RX_CMPLT1_DATA
 * AMFH will fire complete1 event after an entire frame (or burst) was copied
 * to the RXBUFF and the length header and control trailer was written to the
 * buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [31:0]	CMPLT1_DATA	RW
 */
#define AMFH_RX_CMPLT1_DATA_REG		(0x6c04c)

/*
 * Register : AMFH_RX_CMPLT1_ADDR
 * AMFH will fire complete1 event after an entire frame (or burst) was copied
 * to the RXBUFF and the length header and control trailer was written to the
 * buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [19:0]	CMPLT1_ADDRESS	RW
 * [31:20] Reserved.
 */
#define AMFH_RX_CMPLT1_ADDR_REG		(0x6c050)
#define AMFH_RX_CMPLT1_ADDR_REG_POS	(0)
#define AMFH_RX_CMPLT1_ADDR_REG_MSK	(0xfffff)

/*
 * Register : AMFH_RX_CMPLT2_EN
 * 0 - disable.
 * 1 - enable.
 * AMFH will fire complete2 event after an entire frame (or burst) was copied
 * to the RXBUFF and the length header and control trailer was written to the
 * buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [0:0]	CMPLT2_ENABLE	RW
 * [31:1]	Reserved.
 */
#define AMFH_RX_CMPLT2_EN_REG		(0x6c054)
#define AMFH_RX_CMPLT2_EN_REG_POS	(0)
#define AMFH_RX_CMPLT2_EN_REG_MSK	(0x1)

/*
 * Register : AMFH_RX_CMPLT2_DATA
 * AMFH will fire complete2 event after an entire frame (or burst) was copied
 * to the RXBUFF and the length header and control trailer was written to the
 * buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [31:0]	CMPLT2_DATA	RW
 */
#define AMFH_RX_CMPLT2_DATA_REG		(0x6c058)

/*
 * Register : AMFH_RX_CMPLT2_ADDR
 * AMFH will fire complete2 event after an entire frame (or burst) was copied
 * to the RXBUFF and the length header and control trailer was written to the
 * buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [19:0]	CMPLT2_ADDRESS	RW
 * [31:20] Reserved.
 */
#define AMFH_RX_CMPLT2_ADDR_REG		(0x6c05c)
#define AMFH_RX_CMPLT2_ADDR_REG_POS	(0)
#define AMFH_RX_CMPLT2_ADDR_REG_MSK	(0xfffff)

/*
 * Register : AMFH_RX_GKCK_EN
 * 0- disable.
 * 1 - enable.
 * AMFH will fire global_kick event after an entire frame (or burst) was
 * copied to the RXBUFF and the length header and control trailer was
 * written to the buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [0:0]	ENABLE_GKCK_AFTER_KCK0		RW
 * [1:1]	ENABLE_GKCK_AFTER_KCK1		RW
 * [2:2]	ENABLE_GKCK_AFTER_KCK2		RW
 * [3:3]	ENABLE_GKCK_AFTER_COMPLETE1	RW
 * [4:4]	ENABLE_GKCK_AFTER_COMPLETE2	RW
 * [31:5] Reserved.
 */
#define AMFH_RX_GKCK_EN_REG		(0x6c060)

#define AMFH_RX_GKCK_EN_REG_KCK0_POS	(0)
#define AMFH_RX_GKCK_EN_REG_KCK1_POS	(1)
#define AMFH_RX_GKCK_EN_REG_KCK2_POS	(2)
#define AMFH_RX_GKCK_EN_REG_COMPL1_POS	(3)
#define AMFH_RX_GKCK_EN_REG_COMPL2_POS	(4)

#define AMFH_RX_GKCK_EN_REG_KCK0_MSK	(0x1)
#define AMFH_RX_GKCK_EN_REG_KCK1_MSK	(0x2)
#define AMFH_RX_GKCK_EN_REG_KCK2_MSK	(0x4)
#define AMFH_RX_GKCK_EN_REG_COMPL1_MSK	(0x8)
#define AMFH_RX_GKCK_EN_REG_COMPL2_MSK	(0x10)

/*
 * Register : AMFH_RX_GKCK_DATA
 * AMFH will fire global_kick event after an entire frame (or burst) was
 * copied to the RXBUFF and the length header and control trailer was
 * written to the buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [31:0]	AMFH_RX_GKCK_DATA_GKCK_DATA	RW
 */
#define AMFH_RX_GKCK_DATA_REG		(0x6c064)

#define AMFH_RX_GKCK_DATA_REG_POS	(0)
#define AMFH_RX_GKCK_DATA_REG_MSK	(0xffffffff)

/*
 * Register : AMFH_RX_GKCK_ADDR
 * AMFH will fire global_kick event after an entire frame (or burst) was
 * copied to the RXBUFF and the length header and control trailer was
 * written to the buffer.
 * Need to be written by SW when AMFH is not enabled.
 * [19:0]  AMFH_RX_GKCK_ADDR_GKCK_ADDRESS	RW
 * [31:20] Reserved.
 */
#define AMFH_RX_GKCK_ADDR_REG		(0x6c068)

#define AMFH_RX_GKCK_ADDR_REG_POS	(0)
#define AMFH_RX_GKCK_ADDR_REG_MSK	(0xfffff)

/*
 * Register : AMFH_RX_RESUME
 * Resume command after STOP or HOLD event.
 * FW set bit to '1'; HW set the bit to '0'.
 * [0:0]  AMFH_RX_RESUME	RW
 * [31:2] Reserved.
 */
#define AMFH_RX_RESUME_REG		(0x6c06c)

#define AMFH_RX_RESUME_REG_POS		(0)
#define AMFH_RX_RESUME_REG_MSK		(0x1)

/*
 * Register : AMFH_RX_MAX_FRM_COUNT
 * Max #frames that a BURST can contain.
 * '0x0': illegal value.
 * Need to be written by SW when AMFH is not enabled.
 * [6:0]	MAX_FRAMES	RW
 */
#define AMFH_RX_MAX_FRM_COUNT_REG	(0x6c070)

#define AMFH_RX_MAX_FRM_COUNT_REG_POS	(0)
#define AMFH_RX_MAX_FRM_COUNT_REG_MSK	(0x7f)

/*
 * Register : AMFH_RXB_BASE_ADDR
 * Address of word0 of the RX-BUFFER.
 * Need to be written by SW when AMFH is not enabled.
 * [19:0]	AMFH_RXB_BASE_ADDR_RXB_BASE_ADDRESS	RW
 * [31:20]	Reserved
 */
#define AMFH_RXB_BASE_ADDR_REG		(0x6c080)

#define AMFH_RXB_BASE_ADDR_REG_POS	(0)
#define AMFH_RXB_BASE_ADDR_REG_MSK	(0xfffff)

/*
 * Register : AMFH_RXB_SIZE
 * Number of bytes that the RXB contains.
 * Need to be written by SW when AMFH is not enabled.
 * 0 - not allowed.
 * 1 - 4KB (the default).
 * 2 - 8KB.
 * 3 - 12KB.
 * 4 - 16KB.
 * [2:0]	RXB size	RW
 * [31:3]	Reserved
 */
#define AMFH_RXB_SIZE_REG		(0x6C084)

#define AMFH_RXB_SIZE_4K		(0x1)
#define AMFH_RXB_SIZE_8K		(0x2)
#define AMFH_RXB_SIZE_12K		(0x3)
#define AMFH_RXB_SIZE_16K		(0x4)

/*
 * Register : AMFH_TG1_RD_RSP_ADDR
 * DRIVER LMAC channel.
 * The Address that the AMFH will write the read response to.
 * Fields:
 *	[19:0]	RSP address.
 *	[20:31]	Reserved.
 */
#define AMFH_TG1_RD_RSP_ADDR		(0x6c0a4)

#define AMFH_TG1_RD_RSP_ADDR_MSK	(0xfffff)

/*
 * Register : AMFH_TG1_BUS_WAIT_EN
 * Enables AHB wait when accessing to busy TG1 channel.
 * '0' - ignores AHB access to the TG when TG busy.
 * '1' - stall AHB bus, if there is AHB access to TG when TG channel is busy.
 * [0:0]	TG1_BUS_WAIT_EN	RW
 * [31:1]	Reserved.
 */
#define AMFH_TG1_BUS_WAIT_EN_REG	(0x6c0c0)

#define AMFH_TG1_BUS_IGNORE		(0x0)
#define AMFH_TG1_BUS_STALL		(0x1)

/*
 * Register : AMFH_TG2_BUS_WAIT_EN
 * Enables AHB wait when accessing to busy TG2 channel.
 * '0' - ignores AHB access to the TG when TG busy.
 * '1' - stall AHB bus, if there is AHB access to TG when TG channel is busy.
 * [0:0]	TG2_BUS_WAIT_EN	RW
 * [31:1]	Reserved.
 */
#define AMFH_TG2_BUS_WAIT_EN_REG	(0x6c0c4)

#define AMFH_TG2_BUS_IGNORE		(0x0)
#define AMFH_TG2_BUS_STALL		(0x1)

/*
 * Register : SW_CLK_BYPASS
 * bypass the HW dynamic clock gates
 * [0:0]	SW_CLK_BYPASS_AMFH_TX	RW
 * [1:1]	SW_CLK_BYPASS_AMFH_RX	RW
 * [2:2]	SW_CLK_BYPASS_AMFH_TG1	RW
 * [3:3]	SW_CLK_BYPASS_AMFH_TG2	RW
 * [4:4]	SW_CLK_BYPASS_AMFH_MI	RW
 * [5:5]	SW_CLK_BYPASS_AMFH_DBG	RW
 * [6:6]	SW_CLK_BYPASS_AMFH_CSR	RW
 * [7:7]	SW_CLK_BYPASS_DMA	RW
 * [8:8]	SW_CLK_BYPASS_CFG	RW
 * [9:9]	SW_CLK_BYPASS_VIA	RW
 * [10:10]	SW_CLK_BYPASS_SFDB	RW
 */
#define SW_CLK_BYPASS			(0x7000c)

#define SW_CLK_BYPASS_AMFH_TX_MSK	(0x1)
#define SW_CLK_BYPASS_AMFH_RX_MSK	(0x2)
#define SW_CLK_BYPASS_AMFH_TG1_MSK	(0x4)
#define SW_CLK_BYPASS_AMFH_TG2_MSK	(0x8)
#define SW_CLK_BYPASS_AMFH_MI_MSK	(0x10)
#define SW_CLK_BYPASS_AMFH_DBG_MSK	(0x20)
#define SW_CLK_BYPASS_AMFH_CSR_MSK	(0x40)
#define SW_CLK_BYPASS_DMA_MSK		(0x80)
#define SW_CLK_BYPASS_CFG_MSK		(0x100)
#define SW_CLK_BYPASS_VIA_MSK		(0x200)
#define SW_CLK_BYPASS_SFDB_MSK		(0x400)

#define SW_CLK_BYPASS_AMFH_TX_POS	(0)
#define SW_CLK_BYPASS_AMFH_RX_POS	(1)
#define SW_CLK_BYPASS_AMFH_TG1_POS	(2)
#define SW_CLK_BYPASS_AMFH_TG2_POS	(3)
#define SW_CLK_BYPASS_AMFH_MI_POS	(4)
#define SW_CLK_BYPASS_AMFH_DBG_POS	(5)
#define SW_CLK_BYPASS_AMFH_CSR_POS	(6)
#define SW_CLK_BYPASS_DMA_POS		(7)
#define SW_CLK_BYPASS_CFG_POS		(8)
#define SW_CLK_BYPASS_VIA_POS		(9)
#define SW_CLK_BYPASS_SFDB_POS		(10)

/*
 * Register : DMA_CONTROL
 * controls the DMA and contains global DMA status.
 * [6:1]	 auto_padding	dummy field for padding
 * [8:7]	 Reserved
 * [9:9]	 INTEN	RW
 *			Interrupt Enable - Global Interrupt enable.
 *			0 - Disables interrupt generation from the DMA
 *			    Controller.
 *			1 - Enables interrupt generation from the DMA
 *			    Controller
 * [10:10] LOCKEN	RW
 *			Lock Enable - Enable generation of Lock
 *			transaction on AHB
 *			0 - Disable locking the AHB
 *			1 - The DMA will lock the AHB.
 * [15:11] DSCLGT	RW
 *			Descriptor Length - this register determine
 *			descriptor size in SGM, usual its 3 words.
 * [16:16] DMAINT	RW
 *			DMA Interrupt - denote the Interrupt Pending
 *			status of the DMA Controller. This bit is not set
 *			if the INTEN bit in the DMA Control register is a zero.
 *			This bit clears when there are no interrupt pending.
 *			0 - DMA Controller does not have an interrupt pending.
 *			1 - DMA Controller has an interrupt pending.
 * [23:17] Reserved	RO
 * [24:24] CH0_DREQSW	RW	Channel 0 Dreq software.
 *				this bit is active when CH0DREQEN is not enable
 *				(equal to zero).
 *				0 - DREQ 0 receive '1' value and its active
 *				1 - DREQ 0 receive '0' value and its not active
 * [25:25] CH1_DREQSW	RW	Channel 1 Dreq software.
 *				this bit is active when CH1DREQEN is not enable
 *				(equal to zero).
 *				0 - DREQ 1 receive '1' value and its active
 *				1 - DREQ 1 receive '0' value and its not active
 * [26:26] CH2_DREQSW	RW	Channel 2 Dreq software.
 *				this bit is active when CH2DREQEN is not enable
 *				(equal to zero).
 *				0 - DREQ 2 receive '1' value and its active
 *				1 - DREQ 2 receive '0' value and its not active
 * [27:27] CH3_DREQSW	RW	Channel 3 Dreq software.
 *				this bit is active when CH3DREQEN is not enable
 *				(equal to zero).
 *				0 - DREQ 3 receive '1' value and its active
 *				1 - DREQ 3 receive '0' value and its not active
 * [28:28] CH4_DREQSW	RW	Channel 4 Dreq software.
 *				this bit is active when CH4DREQEN is not enable
 *				(equal to zero).
 *				0 - DREQ 4 receive '1' value and its active
 *				1 - DREQ 4 receive '0' value and its not active
 * [29:29] CH5_DREQSW	RW	Channel 5 Dreq software.
 *				this bit is active when CH5DREQEN is not enable
 *				(equal to zero).
 *				0 - DREQ 5 receive '1' value and its active
 *				1 - DREQ 5 receive '0' value and its not active
 * [30:30] CH6_DREQSW	RW	Channel 6 Dreq software.
 *				this bit is active when CH6DREQEN is not enable
 *				(equal to zero).
 *				0 - DREQ 6 receive '1' value and its active
 *				1 - DREQ 6 receive '0' value and its not active
 * [31:31] Reserved	RO
 */
#define DMA_CONTROL			(0x71000)
#define DMA_CONTROL_INTEN_MSK		(0x200)
#define DMA_CONTROL_LOCKEN_MSK		(0x400)
#define DMA_CONTROL_DSCLGT_MSK		(0xf800)
#define DMA_CONTROL_DMAINT_MSK		(0x10000)
#define DMA_CONTROL_CH0_DREQSW_MSK	(0x1000000)
#define DMA_CONTROL_CH1_DREQSW_MSK	(0x2000000)
#define DMA_CONTROL_CH2_DREQSW_MSK	(0x4000000)
#define DMA_CONTROL_CH3_DREQSW_MSK	(0x8000000)
#define DMA_CONTROL_CH4_DREQSW_MSK	(0x10000000)
#define DMA_CONTROL_CH5_DREQSW_MSK	(0x20000000)
#define DMA_CONTROL_CH6_DREQSW_MSK	(0x40000000)

#define DMA_CONTROL_INTEN_POS		(9)
#define DMA_CONTROL_LOCKEN_POS		(10)
#define DMA_CONTROL_DSCLGT_POS		(11)
#define DMA_CONTROL_DMAINT_POS		(16)
#define DMA_CONTROL_CH0_DREQSW_POS	(24)
#define DMA_CONTROL_CH1_DREQSW_POS	(25)
#define DMA_CONTROL_CH2_DREQSW_POS	(26)
#define DMA_CONTROL_CH3_DREQSW_POS	(27)
#define DMA_CONTROL_CH4_DREQSW_POS	(28)
#define DMA_CONTROL_CH5_DREQSW_POS	(29)
#define DMA_CONTROL_CH6_DREQSW_POS	(30)

/*
 * Register : DMA_CHANNELS_3210_CONTROL
 * This is the control and status for the individual DMA channels 0-3
 * [1:0] Reserved	RO
 * [2:2]	CH0DREQWAIT_EN	 RW
 *		Determine if DMA Channel0 wait for dreq before loading
 *		new descriptor or not.
 *		0 - disable wait for dreq function and all the descriptors
 *		    (1st or in a link list) will be loaded like en211 and
 *		maybe wait for dreq. It depends on DMA internal arbiter.
 *		1 - enable wait for dreq function and all the descriptors
 *		(1st or in a link list) will loaded only if dreq signal active.
 * [3:3]	CH0_DREQ_EN	 RW
 *		Channel 0 DRQ Enable Global control for channel 0 DRQ.
 *			0 channels DRQ is disabled.
 *			1 channels DRQ is enabled.
 * [4:4]	CH0_DACK_P	 RW
 *		Channel 0 DACK Polarity Signal Polarity for channel 0 DACK.
 *			0 channels DACK is Active Low.
 *			1 channels DACK is Active high.
 * [5:5]	CH0_DRQP	 RW
 *		Channel 0 DRQ Polarity Signal Polarity for channel 0 DRQ.
 *			0 channels DRQ is Active Low.
 *			1 channels DRQ is Active high.
 * [6:6]	CH0_BUSY	 RW
 *		DMA Channel 0 Busy Denotes the busy status of channel 0.
 *		This bit is read only.
 *			0 Channel is not running.
 *			1 Channel is busy.
 * [9:7]	Reserved.
 * [10:10]	CH1_DREQ_WAIT_EN	 RW
 *		Determine if DMA Channel1 wait for dreq before loading
 *		new descriptor or not.
 *		0 - disable wait for dreq function and all the descriptors
 *		(1st or in a link list) will be loaded like en211 and maybe
 *		wait for dreq. It depends on DMA internal arbiter.
 *		1 - enable wait for dreq function and all the descriptors
 *		    (1st or in a link list) will loaded only if
 *		    dreq signal active.
 * [11:11]	CH1_DREQ_EN	 RW
 *		Channel 1 DRQ Enable Global control for channel 1 DRQ.
 *			0 channels DRQ is disabled.
 *			1 channels DRQ is enabled.
 * [12:12]	CH1_DACK_P	 RW
 *		Channel 1 DACK Polarity Signal Polarity for channel 1 DACK.
 *			0 channels DACK is Active Low.
 *			1 channels DACK is Active high.
 * [13:13]	CH1_DRQ_P	 RW
 *		Channel 1 DRQ Polarity Signal Polarity for channel 1 DRQ.
 *			0 channels DRQ is Active Low.
 *			1 channels DRQ is Active high.
 * [14:14]	CH1_BUSY	 RW
 *		DMA Channel 1 Busy Denotes the busy status of channel 1.
 *		This bit is read only.
 *			0 Channel is not running.
 *			1 Channel is busy.
 * [17:15]	Reserved.
 * [18:18]	CH2_DREQ_WAIT_EN	 RW
 *		Determine if DMA Channel2 wait for dreq before loading
 *		new descriptor or not.
 *		0 - disable wait for dreq function and all the descriptors
 *		    (1st or in a link list) will be loaded like en211 and maybe
 *		    wait for dreq. It depends on DMA internal arbiter.
 *		1 - enable wait for dreq function and all the descriptors
 *		    (1st or in a link list) will loaded only if dreq
 *		    signal active.
 * [19:19]	CH2_DREQ_EN	 RW
 *		Channel 2 DRQ Enable Global control for channel 2 DRQ.
 *			0 channels DRQ is disabled.
 *			1 channels DRQ is enabled.
 * [20:20]	CH2_DACK_P	 RW
 *		Channel 2 DACK Polarity Signal Polarity for channel 2 DACK.
 *			0 channels DACK is Active Low.
 *			1 channels DACK is Active high.
 * [21:21]	CH2_DRQ_P	 RW
 *		Channel 2 DRQ Polarity Signal Polarity for channel 2 DRQ.
 *			0 channels DRQ is Active Low.
 *			1 channels DRQ is Active high.
 * [22:22]	CH2_BUSY	 RW
 *		DMA Channel 2 Busy Denotes the busy status of channel 2.
 *		This bit is read only.
 *			0 Channel is not running.
 *			1 Channel is busy.
 * [26:26]	CH3_DREQ_WAIT_EN	 RW
 *		Determine if DMA Channel3 wait for dreq before loading
 *		new descriptor or not.
 *		0 - disable wait for dreq function and all the descriptors
 *		    (1st or in a link list) will be loaded like en211 and maybe
 *		    wait for dreq. It depends on DMA internal arbiter.
 *		1 - enable wait for dreq function and all the descriptors
 *		    (1st or in a link list) will loaded only if dreq
 *		    signal active.
 * [27:27]	CH3_DREQ_EN	 RW
 *		Channel 3 DRQ Enable Global control for channel 3 DRQ.
 *			0 channels DRQ is disabled.
 *			0 channels DRQ is enabled.
 * [28:28]	CH3_DACK_P	 RW
 *		Channel 3 DACK Polarity Signal Polarity for channel 3 DACK.
 *			0 channels DACK is Active Low.
 *			1 channels DACK is Active high.
 * [29:29]	CH3_DRQ_P	 RW
 *		Channel 3 DRQ Polarity Signal Polarity for channel 3 DRQ.
 *			0 channels DRQ is Active Low.
 *			1 channels DRQ is Active high.
 * [30:30]	CH3_BUSY	 RW
 *		DMA Channel 3 Busy Denotes the busy status of channel 3.
 *		This bit is read only.
 *			0 Channel is not running.
 *			1 Channel is busy.
 */
#define DMA_CHANNELS_3210_CONTROL	(0x71004)

#define DMA_CH0_DREQWAIT_EN_POS		(2)
#define DMA_CH0_DREQEN_POS		(3)
#define DMA_CH0_DACKP_POS		(4)
#define DMA_CH0_DRQP_POS		(5)
#define DMA_CH0_BUSY_POS		(6)

#define DMA_CH0_DREQWAIT_EN_MSK		(0x4)
#define DMA_CH0_DREQEN_MSK		(0x8)
#define DMA_CH0_DACKP_MSK		(0x10)
#define DMA_CH0_DRQP_MSK		(0x20)
#define DMA_CH0_BUSY_MSK		(0x40)

#define DMA_CH1_DREQWAIT_EN_POS		(10)
#define DMA_CH1_DREQEN_POS		(11)
#define DMA_CH1_DACKP_POS		(12)
#define DMA_CH1_DRQP_POS		(13)
#define DMA_CH1_BUSY_POS		(14)

#define DMA_CH1_DREQWAIT_EN_MSK		(0x400)
#define DMA_CH1_DREQEN_MSK		(0x800)
#define DMA_CH1_DACKP_MSK		(0x1000)
#define DMA_CH1_DRQP_MSK		(0x2000)
#define DMA_CH1_BUSY_MSK		(0x4000)

#define DMA_CH2_DREQWAIT_EN_POS		(18)
#define DMA_CH2_DREQEN_POS		(19)
#define DMA_CH2_DACKP_POS		(20)
#define DMA_CH2_DRQP_POS		(21)
#define DMA_CH2_BUSY_POS		(22)

#define DMA_CH2_DREQWAIT_EN_MSK		(0x40000)
#define DMA_CH2_DREQEN_MSK		(0x80000)
#define DMA_CH2_DACKP_MSK		(0x100000)
#define DMA_CH2_DRQP_MSK		(0x200000)
#define DMA_CH2_BUSY_MSK		(0x400000)

#define DMA_CH3_DREQWAIT_EN_POS		(26)
#define DMA_CH3_DREQEN_POS		(27)
#define DMA_CH3_DACKP_POS		(28)
#define DMA_CH3_DRQP_POS		(29)
#define DMA_CH3_BUSY_POS		(30)

#define DMA_CH3_DREQWAIT_EN_MSK		(0x4000000)
#define DMA_CH3_DREQEN_MSK		(0x8000000)
#define DMA_CH3_DACKP_MSK		(0x10000000)
#define DMA_CH3_DRQP_MSK		(0x20000000)
#define DMA_CH3_BUSY_MSK		(0x40000000)

/*
 * Register : DMA_CHANNELS_654_CONTROL
 * This is the control and status for the individual DMA channels 4-7
 * [1:0] Reserved	RO
 * [2:2]	CH4_DREQ_WAIT_EN	 RW
 *		Determine if DMA Channel4 wait for dreq before loading
 *		new descriptor or not.
 *		0 - disable wait for dreq function and all the descriptors
 *		(1st or in alink list) will be loaded like en211 and maybe
 *		wait for dreq. It depends on DMA internal arbiter.
 *		1 - enable wait for dreq function and all the descriptors
 *		(1st or in alink list) will loaded only if dreq signal active.
 * [3:3]	CH4_DREQ_EN	 RW
 *		Channel 0 DRQ Enable Global control for channel 4 DRQ.
 *			0 channels DRQ is disabled.
 *			1 channels DRQ is enabled.
 * [4:4]	CH4_DACKP	 RW
 *		Channel 4 DACK Polarity Signal Polarity for channel 4 DACK.
 *			0 channels DACK is Active Low.
 *			1 channels DACK is Active high.
 * [5:5]	CH5_DRQ_P	 RW
 *		Channel 4 DRQ Polarity Signal Polarity for channel 4 DRQ.
 *			0 channels DRQ is Active Low.
 *			1 channels DRQ is Active high.
 * [6:6]	CH4_BUSY	 RW
 *		DMA Channel 4 Busy Denotes the busy status of channel 4.
 *		This bit is read only.
 *			0 Channel is not running.
 *			1 Channel is busy.
 * [9:7]	Reserved.
 * [10:10]	CH5_DREQ_WAIT_EN	 RW
 *		Determine if DMA Channel5 wait for dreq before loading
 *		new descriptor or not.
 *			0 - disable wait for dreq function and all the
 *			    descriptors (1st or in a link list) will be loaded
 *			    like en211 and maybe wait for dreq.
 *			    It depends on DMA internal arbiter.
 *			1 - enable wait for dreq function and all the
 *			    descriptors (1st or in a link list) will loaded
 *			    only if dreq signal active.
 * [11:11]	CH5_DREQ_EN	 RW
 *		Channel 5 DRQ Enable Global control for channel 5 DRQ.
 *			0 channels DRQ is disabled.
 *			1 channels DRQ is enabled.
 * [12:12]	CH5_DACK_P	 RW
 *		Channel 5 DACK Polarity Signal Polarity for channel 5 DACK.
 *			0 channels DACK is Active Low.
 *			1 channels DACK is Active high.
 * [13:13]	CH5_DRQ_P	 RW
 *		Channel 1 DRQ Polarity Signal Polarity for channel 5 DRQ.
 *			0 channels DRQ is Active Low.
 *			1 channels DRQ is Active high.
 * [14:14]	CH5_BUSY	 RW
 *		DMA Channel 5 Busy Denotes the busy status of channel 5.
 *		This bit is read only.
 *			0 Channel is not running.
 *			1 Channel is busy.
 * [17:15]	Reserved.
 * [18:18]	CH6_DREQ_WAIT_EN	 RW
 *		Determine if DMA Channel6 wait for dreq before loading
 *		new descriptor or not.
 *		0 disable wait for dreq function and all the descriptors
 *		(1st or in a link list) will be loaded like en211
 *		and maybe wait for dreq.
 *		It depends on DMA internal arbiter.
 *		1 - enable wait for dreq function and all the descriptors
 *		    (1st or in a
 *		link list) will loaded only if dreq signal active.
 * [19:19]	CH6_DREQ_EN	 RW
 *		Channel 6 DRQ Enable Global control for channel 6 DRQ.
 *			0 channels DRQ is disabled.
 *			1 channels DRQ is enabled.
 * [20:20]	CH6_DACK_P	 RW
 *		Channel 6 DACK Polarity Signal Polarity for channel 6 DACK.
 *			0 channels DACK is Active Low.
 *			1 channels DACK is Active high.
 * [21:21]	CH6_DRQ_P	 RW
 *		Channel 6 DRQ Polarity Signal Polarity for channel 6 DRQ.
 *			0 channels DRQ is Active Low.
 *			1 channels DRQ is Active high.
 * [22:22]	CH6_BUSY	 RW
 *		DMA Channel 6 Busy Denotes the busy status of channel 6.
 *		This bit is read only.
 *			0 Channel is not running.
 *			1 Channel is busy.
 */

#define DMA_CHANNELS_654_CONTROL	(0x71008)

#define DMA_CH4_DREQWAIT_EN_POS		(2)
#define DMA_CH4_DREQEN_POS		(3)
#define DMA_CH4_DACKP_POS		(4)
#define DMA_CH4_DRQP_POS		(5)
#define DMA_CH4_BUSY_POS		(6)

#define DMA_CH4_DREQWAIT_EN_MSK		(0x4)
#define DMA_CH4_DREQEN_MSK		(0x8)
#define DMA_CH4_DACKP_MSK		(0x10)
#define DMA_CH4_DRQP_MSK		(0x20)
#define DMA_CH4_BUSY_MSK		(0x40)

#define DMA_CH5_DREQWAIT_EN_POS		(10)
#define DMA_CH5_DREQEN_POS		(11)
#define DMA_CH5_DACKP_POS		(12)
#define DMA_CH5_DRQP_POS		(13)
#define DMA_CH5_BUSY_POS		(14)

#define DMA_CH5_DREQWAIT_EN_MSK		(0x400)
#define DMA_CH5_DREQEN_MSK		(0x800)
#define DMA_CH5_DACKP_MSK		(0x1000)
#define DMA_CH5_DRQP_MSK		(0x2000)
#define DMA_CH5_BUSY_MSK		(0x4000)

#define DMA_CH6_DREQWAIT_EN_POS		(18)
#define DMA_CH6_DREQEN_POS		(19)
#define DMA_CH6_DACKP_POS		(20)
#define DMA_CH6_DRQP_POS		(21)
#define DMA_CH6_BUSY_POS		(22)

#define DMA_CH6_DREQWAIT_EN_MSK		(0x40000)
#define DMA_CH6_DREQEN_MSK		(0x80000)
#define DMA_CH6_DACKP_MSK		(0x100000)
#define DMA_CH6_DRQP_MSK		(0x200000)
#define DMA_CH6_BUSY_MSK		(0x400000)

/*
 * Registers : DMA_CH0_SRC_ADDR - DMA_CH6_SRC_ADDR
 * Source Address of the transfer on AHB. Implemented as 32 bits
 * increment register.
 * [31:0]	DMA_CHx_SRC_ADDR   RW	dma will transfer the data
 *					from this address to the
 *					specified destination address.
 */
#define DMA_CH0_SRC_ADDR		(0x71010)
#define DMA_CH1_SRC_ADDR		(0x71020)
#define DMA_CH2_SRC_ADDR		(0x71030)
#define DMA_CH3_SRC_ADDR		(0x71040)
#define DMA_CH4_SRC_ADDR		(0x71050)
#define DMA_CH5_SRC_ADDR		(0x71060)
#define DMA_CH6_SRC_ADDR		(0x71070)


/*
 * Registers : DMA_CH0_DST_ADDR - DMA_CH6_DST_ADDR
 * Destination Address of the transfer on AHB. Implemented as 32 bits
 * increment register.
 * [31:0]	DMA_CHx_DST_ADDR   RW	dma will transfer the data
 *					to this address from the
 *					specified source address.
 */
#define DMA_CH0_DST_ADDR		(0x71014)
#define DMA_CH1_DST_ADDR		(0x71024)
#define DMA_CH2_DST_ADDR		(0x71034)
#define DMA_CH3_DST_ADDR		(0x71044)
#define DMA_CH4_DST_ADDR		(0x71054)
#define DMA_CH5_DST_ADDR		(0x71064)
#define DMA_CH6_DST_ADDR		(0x71074)


/*
 * Registers : DMA_CH0_CMD - DMA_CH6_CMD
 * controls the transfer count and the channel configuration.
 * [5:0]	DMA_CHx_CMD_THCNT	RW
 *	Throttle Count Determine how many cycle the DMA arbiter wait until
 *	tack the GNT from current channel:
 *		000001- 16 bytes.
 *		000010 - 32 bytes.
 *		000011 - 48 bytes.
 *		000100 64 bytes.
 *		111110 992 bytes.
 *		111111 1008 bytes.
 * [6:6]	DMA_CHx_CMD_CHBDWRAPEN	RW
 *Determine if DMA Channel wrap function for link list pointer is active or not
 *		1'b0 - disable the wrap function and work regular.
 *		1'b1 - enable the wrap function and DMA jump to ChxStartBdaddr
 * [7:7]	DMA_CHx_CMD_CHBD_INTEN	RW
 *	Determine if DMA Channel get out interrupt in Scatter Gather
 *	mode (SGM).
 *		1b0 - disable the interrupt in SGM.
 *		1b1 enable the interrupt in SGM when DMA finish to handle the
 *		current buffer descriptor.
 * [8:8]	DMA_CHx_CMD_CHBD_EOTEN	RW
 *	Determine if DMA Channel get out EOT in Scatter Gather mode beside at
 *	end of link list.
 *		1b0 - EOT will be valid only at the end of link list
 *		1b1 - enable the EOT in SGM when DMA finish to handle the
 *		current buffer descriptor , and also EOT will be valid at the
 *		end of link list.
 * [9:9]	Reserved.
 * [10:10] DMA_CHx_CMD_CHEN	RW
 *	   Channel Enable - Enable DMA channel to do the programmed transfer.
 *	   Write:
 *		0 - Channel is not enabled
 *		1 - A 0 to 1 transition turns the channel ON to doeither a
 *		SGM or DM transfer depending on the value of LLEN.
 *	   Read:
 *		0 - The channel is not doing a transfer or has completed
 *		its transfer.
 *		1 - The channel has been enabled to do a transfer
 *	Note: This is bit is set to arm the DMA channel. The channel will
 *	immediately assert a request to the control engine to do a transfer.
 *	However, if the channel is using a DRQ/DACK resource then the request
 *	will be made only when the DRQ is asserted. In SGM mode the descriptor
 *	will be fetched but the transfer will not being till the
 *	DRQ is asserted.
 *	This bit is self clearing. It will be cleared at the end of
 *	the transfer.
 *	Abort A 1 to 0 transition turns the channel OFF to do abort function.
 *	When the channel receive grant from the arbiter its will do one cycle
 *	of 16 bytes ,and will terminate the descriptor.
 *	When its terminate the descriptor the ch_en bit will change to 0 and
 *	the software can reprogramming the channel.
 * [11:11] DMA_CHx_CMD_LLEN	RW
 *	   Link List Enable Scatter Gather Mode (SGM) enable.
 *		0- Program Channel not to do a SGM transfer (DM).
 *		1- Program Channel to do a SGM transfer.
 * [12:12] Reserved.
 * [15:13] DMA_CHx_CMD_DMA_CMD	RW
 *	   DMA Command support in 3 sort of command:
 *		000 NOP - No data movement, used to fetches the descriptor
 *		    in SGM.
 *		010 MOVE, Move data in SGM.
 *		011 MOVE&STOP, move data and stop after its, used in DM and
 *		    SGM in the last descriptor.
 * [31:16] DMA_CHx_CMD_CH_CNT	RW
 *	   DMA transfer size in bytes, when programming this filed in zero
 *	   value, its empty descriptor and the other filed need to be valid
 *	   like the regular mode
 */
#define DMA_CH0_CMD			(0x71018)
#define DMA_CH1_CMD			(0x71028)
#define DMA_CH2_CMD			(0x71038)
#define DMA_CH3_CMD			(0x71048)
#define DMA_CH4_CMD			(0x71058)
#define DMA_CH5_CMD			(0x71068)
#define DMA_CH6_CMD			(0x71078)

#define DMA_THCNT_POS		(0)
#define DMA_WRAP_EN_POS		(6)
#define DMA_INT_EN_POS		(7)
#define DMA_EOT_EN_POS		(8)
#define DMA_CH_EN_POS		(10)
#define DMA_LL_EN_POS		(11)
#define DMA_CMD_POS		(13)
#define DMA_CNT_POS		(16)

#define SIZEOF_DMA_CNT		(2)	/* size in bytes of the count field */

#define DMA_THCNT_MSK		(0x3f)
#define DMA_WRAP_EN_MSK		(0x40)
#define DMA_INT_EN_MSK		(0x80)
#define DMA_EOT_EN_MSK		(0x100)
#define DMA_CH_EN_MSK		(0x400)
#define DMA_LL_EN_MSK		(0x800)
#define DMA_CMD_MSK		(0xe000)
#define DMA_CNT_MSK		(0xffff0000)

#define DMA_CMD_THCNT_16B	(0x1)
#define DMA_CMD_THCNT_32B	(0x2)
#define DMA_CMD_THCNT_48B	(0x3)
#define DMA_CMD_THCNT_64B	(0x4)
#define DMA_CMD_THCNT_992B	(0x3E)
#define DMA_CMD_THCNT_1008B	(0x3F)

#define DMA_CMD_NOP		(0x0)
#define DMA_CMD_MOVE		(0x2)
#define DMA_CMD_MOVE_STOP	(0x3)

/*
 * Registers : DMA_CH0_LL_PTR - DMA_CH6_LL_PTR
 * Pointer to the link list strcture of channel x
 * [31:0]	DMA_CHx_LL_PTR_LL_PTR_ADDRESS	RW
 */
#define DMA_CH0_LL_PTR		(0x7101c)
#define DMA_CH1_LL_PTR		(0x7102c)
#define DMA_CH2_LL_PTR		(0x7103c)
#define DMA_CH3_LL_PTR		(0x7104c)
#define DMA_CH4_LL_PTR		(0x7105c)
#define DMA_CH5_LL_PTR		(0x7106c)
#define DMA_CH6_LL_PTR		(0x7107c)


/*
 * Register : CH0STARTBDADDR - CH6STARTBDADDR
 * Wrap around register
 * [31:2]	START_ADDRESS	RW
 * [1:0]	Reserved
 * The address in which the DMA will jump to if the current command is MOVE
 * and WRAP.
 */
#define CH0_START_BD_ADDR	(0x71100)
#define CH1_START_BD_ADDR	(0x71104)
#define CH2_START_BD_ADDR	(0x71108)
#define CH3_START_BD_ADDR	(0x7110c)
#define CH4_START_BD_ADDR	(0x71110)
#define CH5_START_BD_ADDR	(0x71114)
#define CH6_START_BD_ADDR	(0x71118)

#define CH_START_BD_ADDR_POS	(2)
#define CH_START_BD_ADDR_MSK	(0xfffffffc)

/*
 * Register : HW_REV_CSR
 * HW declares its own SKU and revision.
 * [1:0] Reserved, previosly Dash (0-3) until RP.
 * [3:2] HW_REV_CSR_STEP	RO
 *	Step (A-D) A -00, B -01 ..
 * [15:4] HW_REV_CSR_MAC_TYPE	RO
 *	  MAC type e.g. 0xC-Jackson Peak/Canyon Peak w/BT,
 *	  0x10-Marble Peak 1/Canyon Peak.
 */
#define IDI_HW_REV_CSR		(0x76028)

#define IDI_HW_REV_CSR_STEP_POS		(2)
#define IDI_HW_REV_CSR_MAC_TYPE_POS	(4)

#define IDI_HW_REV_CSR_STEP_MSK		(0xc)
#define IDI_HW_REV_CSR_MAC_TYPE_MSK	(0xfff0)

/*
 * Register : GP_LMAC_INIT_RST_ERR_STT_W1C
 * fields:
 * [0:0] LMAC FW section has been copied
 * [1:1] LMAC FW initialized and ready (alive notification)
 * [2:2] LMAC watch dog timer level interrupt
 * [6:3] LMAC device error reason
 * [31:7] Reserved.
 */
#define GP_LMAC_INIT_RST_ERR_STT_W1C	(0x76140)

/*
 * Register : HIDI_CTL_SW_CLK_BYPASS_AL
 * bypass the HW dynamic clock gates
 * this register is not gated, the other registers are under the gated clock.
 * fields:
 * [0:0] AL_CSR_SW_CLK_BYPASS	RW
 * [1:1] VIA_SW_CLK_BYPASS	RW
 */
#define HIDI_CTL_SW_CLK_BYPASS_AL	(0x761c4)

/*
 * Register: HIDI_GPIO_SEL_AL
 * [1:0] Select GPIO out source
 * 0x0 - TPMG
 * 0x1 - APMG
 * 0x2 - LMPM
 * 0x3 - MONITOR
 */
#define HIDI_GPIO_SEL_AL		(0x761c8)

#define AL_CSR_SW_CLK_BYPASS_MSK	(0x1)
#define VIA_SW_CLK_BYPASS_MSK		(0x2)

#define AL_CSR_SW_CLK_BYPASSPOS		(0)
#define VIA_SW_CLK_BYPASS_POS		(1)

/*
 * Register : TFD_DB_OFFSET
 * the TFD DB offset from the beginning of the SRAM.
 * Should be DWORD aligned.
 * [15:0]  TFD_DB_OFFSET_TFD_DB_OFFSET	RW
 * [31:16] Reserved
 */
#define IDI_TFD_DB_OFFSET_REG		(0xbfc00)

#define IDI_TFD_DB_OFFSET_REG_MSK	(0xffff)

/*
 * Register : BC_TABLE_OFFSET
 * the BC table offset from the beginning of the sram.
 * Should be DWORD aligned.
 * [15:0]  BC_TABLE_OFFSET_BC_TABLE_OFFSET	RW
 * [31:16] Reserved
 */
#define IDI_BC_TABLE_OFFSET_REG		(0xbfc04)

#define IDI_BC_TABLE_OFFSET_REG_MSK	(0xffff)

/*
 * Register : PAYLOAD_OFFSET
 * the payload offset from the beginning of the sram.
 * Should be DWORD aligned.
 * [15:0]  PAYLOAD_OFFSET_PAYLOAD_OFFSET	RW
 * [31:16] Reserved
 */
#define IDI_PAYLOAD_OFFSET_REG		(0xbfc08)

#define IDI_PAYLOAD_OFFSET_REG_MSK	(0xffff)

/*
 * Register : PAYLOAD_SIZE
 * '0' - 256 page bytes.
 * '1' - 512 page bytes.
 * [0:0] PAYLOAD_SIZE_PAYLOAD_SIZE	RW
 */
#define IDI_PAYLOAD_SIZE_REG		(0xbfc0c)

#define IDI_PAYLOAD_SIZE_256		(0x0)
#define IDI_PAYLOAD_SIZE_512		(0x1)

#endif /* __IDI_HOST_CSR_REGS__H__ */
