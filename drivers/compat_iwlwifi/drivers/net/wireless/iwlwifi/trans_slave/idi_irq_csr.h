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

#ifndef __IDI_IRQ_CSR_REGS__H__
#define __IDI_IRQ_CSR_REGS__H__

/*
 * Register : WLAN_IRQ_O
 * IRQ to driver. Should be sync in APMG to always-on clock.
 * write '1' to clear this register.
 *WLAN_IRQ_O	[0:0]	RW1C
 *
 */
#define WLAN_IRQ_O_REG			(0x761a4)
#define WLAN_IRQ_O_REG_CLEAR_MSK	(0x1)

/*
 * Register : WLAN_IRQ_O_MASK
 * mask to IRQ to driver. Should be sync in APMG to always-on clock.
 * '0' - mask enable.
 *WLAN_IRQ_O_MASK	[0:0]	RW	wlan_irq_o_mask
 *
 */
#define WLAN_IRQ_O_MASK_REG		(0x761a8)

#define WLAN_IRQ_O_MASK_IRQS	(0x0)
#define WLAN_IRQ_O_ENABLE_IRQS	(0x1)

/*
 * Register : IDI_IRQ_STT_IDI_CLK (HIDI_APMG_DRV_IRQ_STATUS_IDI_CLK)
 * Status of each interrupt group (of 9 groups).
 * This register is in IDI clock (as opposed to MAC clock).
 *
 * [0:0]	IDI_IRQ_STT_READY_CODE_LOAD_BIT	Ready for code load interrupt
 *						status.
 * [1:1]	IDI_IRQ_STT_AL_DEV_ERR_BIT	AL device error interrupt
 *						status.
 * [2:2]	IDI_IRQ_STT_LMAC_DEV_ERR_BIT	LMAC device error interrupt
 *						status.
 * [3:3]	IDI_IRQ_STT_HOST_WKUP_BIT	Host wake-up interrupt status.
 * [4:4]	IDI_IRQ_STT_AL_GEN_BIT		AL general interrupt status.
 * [5:5]	IDI_IRQ_STT_LMAC_GEN_BIT	LMAC general interrupt status.
 * [6:6]	IDI_IRQ_STT_LEGACY_INTA_BIT	Legacy INTA interrupt status.
 * [7:7]	IDI_IRQ_STT_AL_INTERRUPT_BIT	AL interrupts.
 * [8:8]	IDI_IRQ_STT_DMA_BIT		DMA interrupts.
 * [9:31]	Reserved.
 */
#define IDI_IRQ_STT_IDI_CLK_REG		(0x393a0)

#define IDI_IRQ_STT_READY_CODE_LOAD_BIT	BIT(0)
#define IDI_IRQ_STT_AL_DEV_ERR_BIT	BIT(1)
#define IDI_IRQ_STT_LMAC_DEV_ERR_BIT	BIT(2)
#define IDI_IRQ_STT_HOST_WKUP_BIT	BIT(3)
#define IDI_IRQ_STT_AL_GEN_BIT		BIT(4)
#define IDI_IRQ_STT_LMAC_GEN_BIT	BIT(5)
#define IDI_IRQ_STT_LEGACY_INTA_BIT	BIT(6)
#define IDI_IRQ_STT_AL_INTERRUPT_BIT	BIT(7)
#define IDI_IRQ_STT_DMA_BIT		BIT(8)

#define IDI_IRQ_VIA0_READY_CODE_LOAD_MSK_REG	(0x77000)
#define IDI_IRQ_VIA1_AL_DEV_ERR_MSK_REG		(0x77010)
#define IDI_IRQ_VIA2_LMAC_DEV_ERR_MSK_REG	(0x77020)
#define IDI_IRQ_VIA3_HOST_WKUP_MSK_REG		(0x77030)
#define IDI_IRQ_VIA4_AL_GEN_MSK_REG		(0x77040)
#define IDI_IRQ_VIA5_LMAC_GEN_MSK_REG		(0x77050)
#define IDI_IRQ_VIA6_LEGACY_INTA_MSK_REG	(0x77060)
#define IDI_IRQ_VIA7_AL_INTERRUPT_MSK_REG	(0x77070)
#define IDI_IRQ_VIA8_DMA_MSK_REG		(0x77080)

#define IDI_IRQ_VIA0_READY_CODE_LOAD_STT_REG	(0x77200)
#define IDI_IRQ_VIA1_AL_DEV_ERR_STT_REG		(0x77210)
#define IDI_IRQ_VIA2_LMAC_DEV_ERR_STT_REG	(0x77220)
#define IDI_IRQ_VIA3_HOST_WKUP_STT_REG		(0x77230)
#define IDI_IRQ_VIA4_AL_GEN_STT_REG		(0x77240)
#define IDI_IRQ_VIA5_LMAC_GEN_STT_REG		(0x77250)
#define IDI_IRQ_VIA6_LEGACY_INTA_STT_REG	(0x77260)
#define IDI_IRQ_VIA7_AL_INTER_STT_REG	(0x77270)
#define IDI_IRQ_VIA8_DMA_STT_REG		(0x77280)


/*
 * Register : READY_FOR_CODE_LOAD_STT
 *
 * [0:0]	RO	Status of READY_FOR_CODE_LOAD_INT.
 *			This interrupt is cleared via the registers in the
 *			AL clock
 */
#define READY_FOR_CODE_LOAD_STT_REG	(0x393b4)

#define READY_FOR_CODE_LOAD_MSK		(0x1)

/*
 * Register : READY_FOR_CODE_LOAD_INT_MASK
 *
 * [0:0]	RW	mask READY_FOR_CODE_LOAD_INT
 */
#define READY_FOR_CODE_LOAD_INT_MASK		(0x393bc)
#define READY_FOR_CODE_LOAD_INT_MASK_BIT	(0x1)

/*
 * Register : READY_FOR_CODE_LOAD_INT
 * write '1' to clear this register.
 * [0:0] READY_FOR_CODE_LOAD_INT	set by HW when apmg_hpe_cd1rdy is
 *					'1'. Cleared by SW.
 */
#define READY_FOR_CODE_LOAD_INT		(0x7619c)

/*
 * Register : INTA_CSR_W1C
 * UCODE uses this register to assert an interrupt to the host driver.
 * (Offset 008 h). Field comments taken from the address map as is.
 * [5:0]	INTA_CSR_W1S_INTRPT_GEN_5_0	RW
 *		Interrupt general
 * [6:6]	INTA_CSR_W1S_CT_KILL	RO
 *		CT KILL APMG indicate CT kill was asserted.
 *		m it prevent HOST driver access PCIEX CSR,
 *		0- No isolation
 * [7:7]	INTA_CSR_W1S_RF_KILL	RO
 *		RF Kill Asserted on a change in GP_CTRL_CSR_AD
 * [24:24]	INTA_CSR_W1S_INTRPT_GEN_24	RW
 *		Interrupt general
 * [25:25]	INTA_CSR_W1S_INTRPT_GEN_25	RW
 *		Interrupt general
 * [26:26]	INTA_CSR_W1S_HW_SCHEDL	RO
 *		HW schedule (Nevo interrupt) for details see Nevo EAS chapter
 * [27:27]	INTA_CSR_W1S_TX_INTRPT_FH_ST	RO
 *		indicate a TX interrupt was asserted in FH_INT_Status register
 * [29:29]	INTA_CSR_W1S_ERR_TO_ASRT	RO
 *		asserted by HW if Error handling event happened.
 *		Driver should check the error source clean the error
 *		source then clean this bit FH only !!!
 * [31:31]	INTA_CSR_W1S_RX_FRM_IN_DRAM	RO
 *		indicate a RX frame is in DRAM
 */
#define IDI_INTA_CSR_W1C			(0x76208)

/*
 * Register: INTA_CSR_W1S
 * The driver can use this register to simulate a wakeup interrupt from
 * the FW.
 */
#define IDI_INTA_CSR_W1S			(0x76008)

/*
 * Register : IDI_INTA_CSR_MASK (INTA_MASK_CSR_MASK)
 * Control which bit of INTA register can force assertion of
 * INT (PCI interrupt).
 * 1b-Interrupt is enabled
 * 0b- Interrupt is masked
 */
#define IDI_INTA_CSR_MASK	(0x7600c)

/*
 * Register : FH_INT_CSR_W1C
 * UCODE uses this register to assert an interrupt to the host driver.
 */
#define IDI_FH_INT_CSR_W1C	(0x76200)
#define FH_INT_CSR_MASK		(0x76204)

/* interrupt flags in INTA, set by uCode or hardware,
 * acknowledged (reset) by host writing "1" to flagged bits.
 * BIT(31) - Rx DMA, cmd responses, FH_INT[17:16]
 * BIT(29) - DMA hardware error FH_INT[31]
 * BIT(27) - Tx DMA FH_INT[1:0]
 * BIT(26) - TXQ pointer advanced
 * BIT(25) - uCode error
 * BIT(6) - Critical temp (chip too hot) rfkill
 * BIT(3) - Rx, command responses
 * BIT(1) - NIC controller waking up (pwr mgmt)
 * BIT(0) - uCode interrupts once it initializes
 */
#define IDI_CSR_INTA_BIT_FH_RX		BIT(31)
#define IDI_CSR_INTA_BIT_HW_ERR		BIT(29)
#define IDI_CSR_INTA_BIT_FH_TX		BIT(27)
#define IDI_CSR_INTA_BIT_SCD		BIT(26)
#define IDI_CSR_INTA_BIT_SW_ERR		BIT(25)
#define IDI_CSR_INTA_BIT_CT_KILL	BIT(6)
#define IDI_CSR_INTA_BIT_SW_RX		BIT(3)
#define IDI_CSR_INTA_BIT_WAKEUP		BIT(1)
#define IDI_CSR_INTA_BIT_ALIVE		BIT(0)

/* Rx interrupts from the FH are not enabled: In LhP we use an
 * interrupt from the IDI bus for that purpose. */
#define IDI_CSR_INI_SET_MASK	(IDI_CSR_INTA_BIT_HW_ERR  | \
				 IDI_CSR_INTA_BIT_FH_TX   | \
				 IDI_CSR_INTA_BIT_SW_ERR  | \
				 IDI_CSR_INTA_BIT_SW_RX   | \
				 IDI_CSR_INTA_BIT_WAKEUP  | \
				 IDI_CSR_INTA_BIT_ALIVE)

/* interrupt flags in FH (flow handler) */
#define IDI_CSR_FH_INT_BIT_ERR		BIT(31) /* Error */
#define IDI_CSR_FH_INT_BIT_HI_PRIOR	BIT(30) /* High priority Rx,
						   bypass coalescing */
#define IDI_CSR_FH_INT_BIT_RX_CHNL1	BIT(17) /* Rx channel 1 */
#define IDI_CSR_FH_INT_BIT_RX_CHNL0	BIT(16) /* Rx channel 0 */
#define IDI_CSR_FH_INT_BIT_TX_CHNL1	BIT(1)  /* Tx channel 1 */
#define IDI_CSR_FH_INT_BIT_TX_CHNL0	BIT(0)  /* Tx channel 0 */

#define IDI_CSR_FH_INT_RX_MASK	(IDI_CSR_FH_INT_BIT_HI_PRIOR | \
				IDI_CSR_FH_INT_BIT_RX_CHNL1 | \
				IDI_CSR_FH_INT_BIT_RX_CHNL0)

#define IDI_CSR_FH_INT_TX_MASK	(IDI_CSR_FH_INT_BIT_TX_CHNL1 | \
				IDI_CSR_FH_INT_BIT_TX_CHNL0)

/*
 * Register : IDI_CSR_RESET
 * xvt uses this register to exit from reset.
 */
#define IDI_CSR_RESET  (0x76220)

#endif /* __IDI_IRQ_CSR_REGS__H__ */
