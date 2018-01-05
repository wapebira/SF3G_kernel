/*
 * Rockchip IOMMU driver
 * 
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifdef CONFIG_ROCKCHIP_IOMMU_DEBUG
#define DEBUG
#endif

#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/memblock.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <linux/of.h>
#include <linux/rockchip_iovmm.h>
#include <linux/device.h>
#include "rockchip-iommu.h"

#ifdef CONFIG_X86
#include <asm/cacheflush.h>
#endif

struct rk_iommu_domain {
	struct list_head clients; /* list of iommu_drvdata.node */
	u32 *pgtable; /* lv1 page table, 4KB */
	spinlock_t lock; /* lock for this structure */
	spinlock_t pgtablelock; /* lock for modifying page table @ pgtable */
};

/*312x vop mmu can't not be read*/
#define RK_MMU_VOP_WORKAROUND 0x01
/*ISP mmu can't not be reset*/
#define RK_MMU_ISP_WORKAROUND 0x02
/*HEVC and VPU share the AHB interface*/
#define RK_MMU_VPU_HEVC_WORKAROUND 0x04

/** MMU register offsets */
#define RK_MMU_DTE_ADDR     0x00    /* Directory table address */
#define RK_MMU_STATUS       0x04
#define RK_MMU_COMMAND      0x08
#define RK_MMU_PAGE_FAULT_ADDR  0x0C    /* IOVA of last page fault */
#define RK_MMU_ZAP_ONE_LINE 0x10    /* Shootdown one IOTLB entry */
#define RK_MMU_INT_RAWSTAT  0x14    /* IRQ status ignoring mask */
#define RK_MMU_INT_CLEAR    0x18    /* Acknowledge and re-arm irq */
#define RK_MMU_INT_MASK     0x1C    /* IRQ enable */
#define RK_MMU_INT_STATUS   0x20    /* IRQ status after masking */
#define RK_MMU_AUTO_GATING  0x24

#define DTE_ADDR_DUMMY      0xCAFEBABE
#define FORCE_RESET_TIMEOUT 100 /* ms */

/* RK_MMU_STATUS fields */
#define RK_MMU_STATUS_PAGING_ENABLED       BIT(0)
#define RK_MMU_STATUS_PAGE_FAULT_ACTIVE    BIT(1)
#define RK_MMU_STATUS_STALL_ACTIVE         BIT(2)
#define RK_MMU_STATUS_IDLE                 BIT(3)
#define RK_MMU_STATUS_REPLAY_BUFFER_EMPTY  BIT(4)
#define RK_MMU_STATUS_PAGE_FAULT_IS_WRITE  BIT(5)
#define RK_MMU_STATUS_STALL_NOT_ACTIVE     BIT(31)

/* RK_MMU_COMMAND command values */
#define RK_MMU_CMD_ENABLE_PAGING    0  /* Enable memory translation */
#define RK_MMU_CMD_DISABLE_PAGING   1  /* Disable memory translation */
#define RK_MMU_CMD_ENABLE_STALL     2  /* Stall paging to allow other cmds */
#define RK_MMU_CMD_DISABLE_STALL    3  /* Stop stall re-enables paging */
#define RK_MMU_CMD_ZAP_CACHE        4  /* Shoot down entire IOTLB */
#define RK_MMU_CMD_PAGE_FAULT_DONE  5  /* Clear page fault */
#define RK_MMU_CMD_FORCE_RESET      6  /* Reset all registers */

/* RK_MMU_INT_* register fields */
#define RK_MMU_IRQ_PAGE_FAULT    0x01  /* page fault */
#define RK_MMU_IRQ_BUS_ERROR     0x02  /* bus read error */
#define RK_MMU_IRQ_MASK          (RK_MMU_IRQ_PAGE_FAULT | RK_MMU_IRQ_BUS_ERROR)

#define NUM_DT_ENTRIES 1024
#define NUM_PT_ENTRIES 1024

#define SPAGE_ORDER 12
#define SPAGE_SIZE (1 << SPAGE_ORDER)

 /*
  * Support mapping any size that fits in one page table:
  *   4 KiB to 4 MiB
  */
#define RK_IOMMU_PGSIZE_BITMAP 0x007ff000

#define IOMMU_REG_POLL_COUNT_FAST 1000

static inline void rk_table_flush(u32 *va, unsigned int count)
{

#ifdef CONFIG_X86
	clflush_cache_range((void *)va, count * sizeof(unsigned int));
#elif defined(CONFIG_ARM)
	dmac_flush_range(va, va + count);
	outer_flush_range(virt_to_phys(va), virt_to_phys(va + count));
#elif defined(CONFIG_ARM64)
	__dma_flush_range(va, va + count);
#endif
}

/**
 * Inspired by _wait_for in intel_drv.h
 * This is NOT safe for use in interrupt context.
 *
 * Note that it's important that we check the condition again after having
 * timed out, since the timeout could be due to preemption or similar and
 * we've never had a chance to check the condition before the timeout.
 */
#define rk_wait_for(COND, MS) ({ \
	unsigned long timeout__ = jiffies + msecs_to_jiffies(MS) + 1;	\
	int ret__ = 0;							\
	while (!(COND)) {						\
		if (time_after(jiffies, timeout__)) {			\
			ret__ = (COND) ? 0 : -ETIMEDOUT;		\
			break;						\
		}							\
		usleep_range(50, 100);					\
	}								\
	ret__;								\
})
/*
 * The Rockchip iommu uses a 2-level page table.
 * The first level is the "Directory Table" (DT).
 * The DT consists of 1024 4-byte Directory Table Entries (DTEs), each pointing
 * to a "Page Table".
 * The second level is the 1024 Page Tables (PT).
 * Each PT consists of 1024 4-byte Page Table Entries (PTEs), each pointing to
 * a 4 KB page of physical memory.
 *
 * The DT and each PT fits in a single 4 KB page (4-bytes * 1024 entries).
 * Each iommu device has a MMU_DTE_ADDR register that contains the physical
 * address of the start of the DT page.
 *
 * The structure of the page table is as follows:
 *
 *		     DT
 * MMU_DTE_ADDR -> +-----+
 *		   |	 |
 *		   +-----+     PT
 *		   | DTE | -> +-----+
 *		   +-----+    |     |	  Memory
 *		   |	 |    +-----+	  Page
 *		   |	 |    | PTE | -> +-----+
 *		   +-----+    +-----+	 |     |
 *			      |     |	 |     |
 *			      |     |	 |     |
 *			      +-----+	 |     |
 *					 |     |
 *					 |     |
 *					 +-----+
 */

/*
 * Each DTE has a PT address and a valid bit:
 * +---------------------+-----------+-+
 * | PT address 	 | Reserved  |V|
 * +---------------------+-----------+-+
 *  31:12 - PT address (PTs always starts on a 4 KB boundary)
 *  11: 1 - Reserved
 *	0 - 1 if PT @ PT address is valid
 */

#define RK_DTE_PT_ADDRESS_MASK    0xfffff000
#define RK_DTE_PT_VALID           BIT(0)

static inline phys_addr_t rk_dte_pt_address(u32 dte)
{
	return (phys_addr_t)dte & RK_DTE_PT_ADDRESS_MASK;
}

static inline bool rk_dte_is_pt_valid(u32 dte)
{
	return dte & RK_DTE_PT_VALID;
}

static u32 rk_mk_dte(u32 *pt)
{
	phys_addr_t pt_phys = virt_to_phys(pt);
	return (pt_phys & RK_DTE_PT_ADDRESS_MASK) | RK_DTE_PT_VALID;
}

/*
 * Each PTE has a Page address, some flags and a valid bit:
 * +---------------------+---+-------+-+
 * | Page address        |Rsv| Flags |V|
 * +---------------------+---+-------+-+
 *  31:12 - Page address (Pages always start on a 4 KB boundary)
 *  11: 9 - Reserved
 *   8: 1 - Flags
 *      8 - Read allocate - allocate cache space on read misses
 *      7 - Read cache - enable cache & prefetch of data
 *      6 - Write buffer - enable delaying writes on their way to memory
 *      5 - Write allocate - allocate cache space on write misses
 *      4 - Write cache - different writes can be merged together
 *      3 - Override cache attributes
 *          if 1, bits 4-8 control cache attributes
 *          if 0, the system bus defaults are used
 *      2 - Writable
 *      1 - Readable
 *      0 - 1 if Page @ Page address is valid
 */
#define RK_PTE_PAGE_ADDRESS_MASK  0xfffff000
#define RK_PTE_PAGE_FLAGS_MASK    0x000001fe
#define RK_PTE_PAGE_WRITABLE      BIT(2)
#define RK_PTE_PAGE_READABLE      BIT(1)
#define RK_PTE_PAGE_VALID         BIT(0)

static inline phys_addr_t rk_pte_page_address(u32 pte)
{
	return (phys_addr_t)pte & RK_PTE_PAGE_ADDRESS_MASK;
}

static inline bool rk_pte_is_page_valid(u32 pte)
{
	return pte & RK_PTE_PAGE_VALID;
}

static u32 rk_mk_pte(phys_addr_t page, int prot)
{
	u32 flags = 0;

	flags |= (prot & IOMMU_READ) ? RK_PTE_PAGE_READABLE : 0;
	flags |= (prot & IOMMU_WRITE) ? RK_PTE_PAGE_WRITABLE : 0;

	page &= RK_PTE_PAGE_ADDRESS_MASK;
	return page | flags | RK_PTE_PAGE_VALID;
}

static u32 rk_mk_pte_invalid(u32 pte)
{
	return pte & ~RK_PTE_PAGE_VALID;
}

/*
 * iova (IOMMU Virtual Address) format
 *  31       22.21       12.11          0
 * +-----------+-----------+-------------+
 * | DTE index | PTE index | Page offset |
 * +-----------+-----------+-------------+
 *  31:22 - DTE index   - index of DTE in DT
 *  21:12 - PTE index   - index of PTE in PT @ DTE.pt_address
 *  11: 0 - Page offset - offset into page @ PTE.page_address
 */
#define RK_IOVA_DTE_MASK    0xffc00000
#define RK_IOVA_DTE_SHIFT   22
#define RK_IOVA_PTE_MASK    0x003ff000
#define RK_IOVA_PTE_SHIFT   12
#define RK_IOVA_PAGE_MASK   0x00000fff
#define RK_IOVA_PAGE_SHIFT  0

static u32 rk_iova_dte_index(dma_addr_t iova)
{
	return (u32)(iova & RK_IOVA_DTE_MASK) >> RK_IOVA_DTE_SHIFT;
}

static u32 rk_iova_pte_index(dma_addr_t iova)
{
	return (u32)(iova & RK_IOVA_PTE_MASK) >> RK_IOVA_PTE_SHIFT;
}

static u32 rk_iova_page_offset(dma_addr_t iova)
{
	return (u32)(iova & RK_IOVA_PAGE_MASK) >> RK_IOVA_PAGE_SHIFT;
}

static u32 rk_iommu_read(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static void rk_iommu_write(void __iomem *base, u32 offset, u32 value)
{
	writel(value, base + offset);
}

static void rk_iommu_command(void __iomem *base, u32 command)
{
	writel(command, base + RK_MMU_COMMAND);
}

static bool rk_iommu_is_stall_active(void __iomem *base)
{
	return rk_iommu_read(base, RK_MMU_STATUS) &
			     RK_MMU_STATUS_STALL_ACTIVE;
}

static bool rk_iommu_is_paging_enabled(void __iomem *base)
{
	return rk_iommu_read(base, RK_MMU_STATUS) &
			     RK_MMU_STATUS_PAGING_ENABLED;
}

static int rk_iommu_enable_stall(void __iomem *base)
{
	int status;
	int i;

	if (rk_iommu_is_stall_active(base))
		return 0;

	/* Stall can only be enabled if paging is enabled */
	if (!rk_iommu_is_paging_enabled(base))
		return 0;

	status = rk_iommu_read(base, RK_MMU_STATUS);

	if (status & RK_MMU_STATUS_PAGE_FAULT_ACTIVE) {
		pr_info("Page fault, enable stall failed, mmu status is 0x%08x\n",
			 status);
		return -1;
	}

	for (i = 0; i < IOMMU_REG_POLL_COUNT_FAST; i++) {
		rk_iommu_command(base, RK_MMU_CMD_ENABLE_STALL);

		if (rk_iommu_is_stall_active(base))
			break;
	}

	if (i >= IOMMU_REG_POLL_COUNT_FAST) {
		pr_err("Enable stall request timed out, status: %#08x\n",
			rk_iommu_read(base, RK_MMU_STATUS));
		return -1;
	}

	return 0;
}

static int rk_iommu_disable_stall(void __iomem *base)
{
	int i;
	int status = rk_iommu_read(base, RK_MMU_STATUS);

	if (!rk_iommu_is_stall_active(base))
		return 0;

	if (status & RK_MMU_STATUS_PAGE_FAULT_ACTIVE) {
		pr_info("Page fault, disable stall failed, mmu status is 0x%08x\n",
			 status);
		return -1;
	}

	if (!rk_iommu_is_paging_enabled(base))
			return 0;

	for (i = 0; i < IOMMU_REG_POLL_COUNT_FAST; i++) {
		rk_iommu_command(base, RK_MMU_CMD_DISABLE_STALL);

		if (!rk_iommu_is_stall_active(base))
			break;
	}

	if (i >= IOMMU_REG_POLL_COUNT_FAST) {
		pr_err("Disable stall request timed out, status: %#08x\n",
		       rk_iommu_read(base, RK_MMU_STATUS));
		return -1;
	}

	return 0;
}

static int rk_iommu_enable_paging(struct iommu_drvdata *data,
				void __iomem *base)
{
	int i;

	if (data->mmu_flag & RK_MMU_VOP_WORKAROUND) {
		rk_iommu_command(base, RK_MMU_CMD_ENABLE_PAGING);
		return 0;
	}

	if (rk_iommu_is_paging_enabled(base))
		return 0;

	for (i = 0; i < IOMMU_REG_POLL_COUNT_FAST; i++) {
		rk_iommu_command(base, RK_MMU_CMD_ENABLE_PAGING);

		if (rk_iommu_is_paging_enabled(base))
			break;
	}

	if (i >= IOMMU_REG_POLL_COUNT_FAST) {
		pr_err("Enable paging request timed out, status: %#08x\n",
		       rk_iommu_read(base, RK_MMU_STATUS));
		return -1;
	}

	return 0;
}

static int rk_iommu_disable_paging(struct iommu_drvdata *data,
				void __iomem *base)
{
	int i;

	if (data->mmu_flag & RK_MMU_VOP_WORKAROUND) {
		rk_iommu_command(base, RK_MMU_CMD_DISABLE_PAGING);
		return 0;
	}
	if (!rk_iommu_is_paging_enabled(base))
		return 0;

	for (i = 0; i < IOMMU_REG_POLL_COUNT_FAST; i++) {
		rk_iommu_command(base, RK_MMU_CMD_DISABLE_PAGING);

		if (!rk_iommu_is_paging_enabled(base))
			break;
	}

	if (i >= IOMMU_REG_POLL_COUNT_FAST) {
		pr_err("Disable paging request timed out, status: %#08x\n",
		       rk_iommu_read(base, RK_MMU_STATUS));
		return -1;
	}

	return 0;
}

static int rk_iommu_force_reset(struct iommu_drvdata *data,
				void __iomem *base)
{
	int i;
	u32 dte_addr;

	/*
	 * Check if register DTE_ADDR is working by writing DTE_ADDR_DUMMY
	 * and verifying that upper 5 nybbles are read back.
	 */
	if (data->mmu_flag & RK_MMU_ISP_WORKAROUND)
		return 0;

	if (data->mmu_flag & RK_MMU_VOP_WORKAROUND) {
		rk_iommu_command(base, RK_MMU_CMD_FORCE_RESET);
		return 0;
	}

	rk_iommu_write(base, RK_MMU_DTE_ADDR, DTE_ADDR_DUMMY);

	dte_addr = rk_iommu_read(base, RK_MMU_DTE_ADDR);
	if (dte_addr != (DTE_ADDR_DUMMY & RK_DTE_PT_ADDRESS_MASK)) {
		pr_err("Error during raw reset. MMU_DTE_ADDR is not functioning\n");
		return -EFAULT;
	}

	for (i = 0; i < IOMMU_REG_POLL_COUNT_FAST; i++) {
		rk_iommu_command(base, RK_MMU_CMD_FORCE_RESET);

		if (rk_iommu_read(base, RK_MMU_DTE_ADDR) == 0x00000000)
			break;
	}

	if (i >= IOMMU_REG_POLL_COUNT_FAST) {
		pr_err("FORCE_RESET command timed out\n");
		return -1;
	}

	return 0;
}

static phys_addr_t rk_iommu_iova_to_phys(struct iommu_domain *domain,
					 dma_addr_t iova)
{
	struct rk_iommu_domain *rk_domain = domain->priv;
	unsigned long flags;
	phys_addr_t pt_phys, phys = 0;
	u32 dte, pte;
	u32 *page_table;

	spin_lock_irqsave(&rk_domain->pgtablelock, flags);

	dte = rk_domain->pgtable[rk_iova_dte_index(iova)];
	if (!rk_dte_is_pt_valid(dte)) {
		pr_err("Cant't find iova %pad relative pte\n", &iova);
		goto out;
	}

	pt_phys = rk_dte_pt_address(dte);
	page_table = (u32 *)phys_to_virt(pt_phys);
	pte = page_table[rk_iova_pte_index(iova)];
	if (!rk_pte_is_page_valid(pte))
		goto out;

	phys = rk_pte_page_address(pte) + rk_iova_page_offset(iova);
out:
	spin_unlock_irqrestore(&rk_domain->pgtablelock, flags);

	return phys;
}

static u32 *rk_dte_get_page_table(struct rk_iommu_domain *rk_domain,
				  dma_addr_t iova)
{
	u32 *page_table, *dte_addr;
	u32 dte;
	phys_addr_t pt_phys;

	assert_spin_locked(&rk_domain->pgtablelock);

	dte_addr = &rk_domain->pgtable[rk_iova_dte_index(iova)];
	dte = *dte_addr;
	if (rk_dte_is_pt_valid(dte))
		goto done;

	page_table = (u32 *)__get_free_pages(GFP_ATOMIC | __GFP_ZERO, 0);
	if (!page_table)
		return ERR_PTR(-ENOMEM);

	dte = rk_mk_dte(page_table);
	*dte_addr = dte;

	rk_table_flush(page_table, NUM_PT_ENTRIES);
	rk_table_flush(dte_addr, 1);

done:
	pt_phys = rk_dte_pt_address(dte);
	return (u32 *)phys_to_virt(pt_phys);
}

static bool rk_set_iommu_active(struct iommu_drvdata *data)
{
	/* return true if the IOMMU was not active previously
	   and it needs to be initialized */
	return ++data->activations == 1;
}

static bool rk_set_iommu_inactive(struct iommu_drvdata *data)
{
	/* return true if the IOMMU is needed to be disabled */
	BUG_ON(data->activations < 1);
	return --data->activations == 0;
}

static bool rk_is_iommu_active(struct iommu_drvdata *data)
{
	return data->activations > 0;
}

static void rk_iommu_page_fault_done(void __iomem *base,
				const char *dbgname)
{
	rk_iommu_command(base,RK_MMU_CMD_PAGE_FAULT_DONE);
	pr_info("%s: Leaving page fault mode\n", dbgname);
}

static int rk_iommu_zap_tlb_without_stall(void __iomem *base)
{
	rk_iommu_command(base,RK_MMU_CMD_ZAP_CACHE);

	return 0;
}
#if 0
static int rk_iommu_zap_tlb(struct iommu_drvdata *data,
				void __iomem *base)
{
	int ret;

	if (!(data->mmu_flag & RK_MMU_VOP_WORKAROUND)) {
		ret = rk_iommu_enable_stall(base);

		if (ret) {
			dev_err(data->iommu_dev,
				"enable stall when zap tlb failed\n");
			return -1;
		}
	}

	rk_iommu_command(base,RK_MMU_CMD_ZAP_CACHE);

	if (!(data->mmu_flag & RK_MMU_VOP_WORKAROUND))
		rk_iommu_disable_stall(base);

	return 0;
}
#endif
static void dump_pagetbl(dma_addr_t fault_address, u32 addr_dte)
{
	u32 dte_index, pte_index, page_offset;
	u32 mmu_dte_addr;
	phys_addr_t mmu_dte_addr_phys, dte_addr_phys;
	u32 *dte_addr;
	u32 dte;
	phys_addr_t pte_addr_phys = 0;
	u32 *pte_addr = NULL;
	u32 pte = 0;
	phys_addr_t page_addr_phys = 0;
	u32 page_flags = 0;

	dte_index = rk_iova_dte_index(fault_address);
	pte_index = rk_iova_pte_index(fault_address);
	page_offset = rk_iova_page_offset(fault_address);

	mmu_dte_addr = addr_dte;
	mmu_dte_addr_phys = (phys_addr_t)mmu_dte_addr;

	dte_addr_phys = mmu_dte_addr_phys + (4 * dte_index);
	dte_addr = phys_to_virt(dte_addr_phys);
	dte = *dte_addr;

	if (!rk_dte_is_pt_valid(dte))
		goto print_it;

	pte_addr_phys = rk_dte_pt_address(dte) + (pte_index * 4);
	pte_addr = phys_to_virt(pte_addr_phys);
	pte = *pte_addr;

	if (!rk_pte_is_page_valid(pte))
		goto print_it;

	page_addr_phys = rk_pte_page_address(pte) + page_offset;
	page_flags = pte & RK_PTE_PAGE_FLAGS_MASK;

print_it:
	pr_err("fault_address = %pad: dte_index: 0x%03x pte_index: 0x%03x page_offset: 0x%03x\n",
		&fault_address, dte_index, pte_index, page_offset);
	pr_err("mmu_dte_addr: %pa dte@%pa: %#08x valid: %u pte@%pa: %#08x valid: %u page@%pa flags: %#03x\n",
		&mmu_dte_addr_phys, &dte_addr_phys, dte,
		rk_dte_is_pt_valid(dte), &pte_addr_phys, pte,
		rk_pte_is_page_valid(pte), &page_addr_phys, page_flags);
}

static irqreturn_t rockchip_iommu_irq(int irq, void *dev_id)
{
	/* SYSMMU is in blocked when interrupt occurred. */
	struct iommu_drvdata *data = dev_id;
	u32 int_status;
	u32 rawstat;
	dma_addr_t fault_address;
	int i;
	u32 reg_status;

	if (!rk_is_iommu_active(data))
		return IRQ_HANDLED;

	for (i = 0; i < data->num_mem; i++) {
		int_status = rk_iommu_read(data->res_bases[i], RK_MMU_INT_STATUS);
		if (int_status == 0)
			continue;

		rawstat = rk_iommu_read(data->res_bases[i], RK_MMU_INT_RAWSTAT);

		reg_status = rk_iommu_read(data->res_bases[i], RK_MMU_STATUS);

		dev_info(data->iommu_dev, "1.rawstat = 0x%08x,int_status = 0x%08x,reg_status = 0x%08x\n",
			 rawstat, int_status, reg_status);

		if (rawstat & RK_MMU_IRQ_PAGE_FAULT) {
			u32 dte;
			int flags;

			fault_address = rk_iommu_read(data->res_bases[i],
						RK_MMU_PAGE_FAULT_ADDR);

			dte = rk_iommu_read(data->res_bases[i], RK_MMU_DTE_ADDR);

			flags = (int_status & 32) ? 1 : 0;

			dev_err(data->iommu_dev, "Page fault detected at %pad from bus id %d of type %s on %s\n",
				&fault_address, (int_status >> 6) & 0x1F,
				(flags == 1) ? "write" : "read", data->dbgname);

			dump_pagetbl(fault_address, dte);

			if (data->domain)
				report_iommu_fault(data->domain, data->iommu_dev,
						   fault_address, flags);
			if (data->fault_handler)
				data->fault_handler(data->iommu_dev,
						    IOMMU_PAGEFAULT,
						    dte, fault_address, 1);

			rk_iommu_page_fault_done(data->res_bases[i],
						 data->dbgname);
		}

		if (rawstat & RK_MMU_IRQ_BUS_ERROR) {
			dev_err(data->iommu_dev, "bus error occured at %pad\n",
				&fault_address);
		}

		if (rawstat & ~(RK_MMU_IRQ_PAGE_FAULT | RK_MMU_IRQ_BUS_ERROR))
			dev_err(data->iommu_dev, "unexpected int_status: %#08x\n\n",
				rawstat);

		rk_iommu_write(data->res_bases[i], RK_MMU_INT_CLEAR, rawstat);

		int_status = rk_iommu_read(data->res_bases[i], RK_MMU_INT_STATUS);

		rawstat = rk_iommu_read(data->res_bases[i], RK_MMU_INT_RAWSTAT);

		reg_status = rk_iommu_read(data->res_bases[i], RK_MMU_STATUS);

		dev_info(data->iommu_dev, "2.rawstat = 0x%08x,int_status = 0x%08x,reg_status = 0x%08x\n",
			 rawstat, int_status, reg_status);

		rk_iommu_zap_tlb_without_stall(data->res_bases[i]);
	}

	return IRQ_HANDLED;
}

int rockchip_iommu_tlb_invalidate_global(struct device *dev)
{
	int ret;
	struct iommu_drvdata *data = dev_get_drvdata(dev->archdata.iommu);

	if (rk_is_iommu_active(data)) {
		int i;

		for (i = 0; i < data->num_mem; i++) {
			ret = rk_iommu_zap_tlb_without_stall(data->res_bases[i]);
			if (ret)
				dev_err(dev->archdata.iommu, "(%s) %s failed\n",
					data->dbgname, __func__);
			return ret;
		}
	} else {
		dev_dbg(data->iommu_dev,
			 "(%s) Disabled. Global, Skipping invalidating TLB.\n",
			 data->dbgname);
	}

	return 0;
}

int rockchip_iommu_tlb_invalidate(struct device *dev)
{
	struct iommu_drvdata *data = dev_get_drvdata(dev->archdata.iommu);
	int i;
	int ret;

	if (data->mmu_flag & RK_MMU_VPU_HEVC_WORKAROUND) {
		dev_dbg(data->iommu_dev, "skip vpu/hevc invalidate TLB\n");
	}

	if (rk_is_iommu_active(data)) {
		for (i = 0; i < data->num_mem; i++) {
			ret = rk_iommu_zap_tlb_without_stall(data->res_bases[i]);
			if (ret) {
				dev_err(data->iommu_dev, "%s,failed\n", __func__);
				return ret;
			}
		}
	} else {
		dev_dbg(data->iommu_dev,
			 "(%s) Disabled. Skipping invalidating TLB.\n",
			 data->dbgname);
	}

	return 0;
}

static size_t rk_iommu_unmap_iova(struct rk_iommu_domain *rk_domain,
				  u32 *pte_addr, dma_addr_t iova, size_t size)
{
	unsigned int pte_count;
	unsigned int pte_total = size / SPAGE_SIZE;

	assert_spin_locked(&rk_domain->pgtablelock);

	for (pte_count = 0; pte_count < pte_total; pte_count++) {
		u32 pte = pte_addr[pte_count];
		if (!rk_pte_is_page_valid(pte))
			break;

		pte_addr[pte_count] = rk_mk_pte_invalid(pte);
	}

	rk_table_flush(pte_addr, pte_count);

	return pte_count * SPAGE_SIZE;
}
static int rk_iommu_map_iova(struct rk_iommu_domain *rk_domain, u32 *pte_addr,
			     dma_addr_t iova, phys_addr_t paddr, size_t size,
			     int prot)
{
	unsigned int pte_count;
	unsigned int pte_total = size / SPAGE_SIZE;
	phys_addr_t page_phys;

	assert_spin_locked(&rk_domain->pgtablelock);

	for (pte_count = 0; pte_count < pte_total; pte_count++) {
		u32 pte = pte_addr[pte_count];

		if (rk_pte_is_page_valid(pte))
			goto unwind;

		pte_addr[pte_count] = rk_mk_pte(paddr, prot);
		paddr += SPAGE_SIZE;
	}

	rk_table_flush(pte_addr, pte_count);

	return 0;

unwind:
	/* Unmap the range of iovas that we just mapped */
	rk_iommu_unmap_iova(rk_domain, pte_addr, iova, pte_count * SPAGE_SIZE);

	iova += pte_count * SPAGE_SIZE;
	page_phys = rk_pte_page_address(pte_addr[pte_count]);
	pr_err("iova: %pad already mapped to %pa cannot remap to phys: %pa prot: %#x\n",
	       &iova, &page_phys, &paddr, prot);

	return -EADDRINUSE;
}
static size_t rk_iommu_unmap(struct iommu_domain *domain,
				   unsigned long iova, size_t size)
{
	struct rk_iommu_domain *rk_domain = domain->priv;
	unsigned long flags;
	u32 dte;
	phys_addr_t pt_phys;
	u32 *pte_addr;
	size_t unmap_size;

	spin_lock_irqsave(&rk_domain->pgtablelock, flags);

	dte = rk_domain->pgtable[rk_iova_dte_index(iova)];
	if (!rk_dte_is_pt_valid(dte)) {
		spin_unlock_irqrestore(&rk_domain->pgtablelock, flags);
		return SPAGE_SIZE;
	}

	pt_phys = rk_dte_pt_address(dte);
	pte_addr = (u32 *)phys_to_virt(pt_phys) + rk_iova_pte_index(iova);
	unmap_size = rk_iommu_unmap_iova(rk_domain, pte_addr, iova, size);

	spin_unlock_irqrestore(&rk_domain->pgtablelock, flags);

	pr_debug("Unmap iova 0x%08lx/0x%x bytes\n", iova, SPAGE_SIZE);

	return unmap_size;
}

static int rk_iommu_map(struct iommu_domain *domain, unsigned long iova,
				phys_addr_t paddr, size_t size, int prot)
{
	struct rk_iommu_domain *rk_domain = domain->priv;
	u32 *page_table, *pte_addr;
	unsigned long flags;
	int ret;

	prot |= IOMMU_READ;
	prot |= IOMMU_WRITE;

	BUG_ON(rk_domain->pgtable == NULL);

	spin_lock_irqsave(&rk_domain->pgtablelock, flags);
	page_table = rk_dte_get_page_table(rk_domain, iova);
	if (IS_ERR(page_table)) {
		spin_unlock_irqrestore(&rk_domain->pgtablelock, flags);
		pr_err("failed to get page table\n");
		return PTR_ERR(page_table);
	}

	pte_addr = &page_table[rk_iova_pte_index(iova)];
	ret = rk_iommu_map_iova(rk_domain, pte_addr, iova, paddr, size, prot);

	spin_unlock_irqrestore(&rk_domain->pgtablelock, flags);

	return ret;
}

static int rk_iommu_attach_device(struct iommu_domain *domain,
				struct device *dev)
{
	struct iommu_drvdata *data = dev_get_drvdata(dev->archdata.iommu);
	struct rk_iommu_domain *rk_domain = domain->priv;
	unsigned long flags;
	int ret;
	int i;
	phys_addr_t dte_addr;

	if (!rk_set_iommu_active(data)) {
		dev_info(data->iommu_dev, "(%s) Already enabled\n",
			 data->dbgname);
		return 0;
	}

	for (i = 0; i < data->num_irq; i++) {
		if (data->mmu_flag & RK_MMU_VOP_WORKAROUND) {
			dev_info(data->iommu_dev, "skip request vop mmu irq\n");
			continue;
		}
		if (data->irq_requested[i])
			continue;
		ret = devm_request_irq(data->iommu_dev, data->irq_array[i],
				       rockchip_iommu_irq, IRQF_SHARED,
				       dev_name(data->iommu_dev), data);
		if (ret) {
			dev_err(data->iommu_dev, "Unabled to register interrupt handler\n");
			return ret;
		}
		data->irq_requested[i] = true;
	}

	for (i = 0; i < data->num_mem; i++) {
		void __iomem *base = data->res_bases[i];
		dte_addr = virt_to_phys(rk_domain->pgtable);

		if (!(data->mmu_flag & RK_MMU_VOP_WORKAROUND)) {
			ret = rk_iommu_enable_stall(data->res_bases[i]);
			if (ret) {
				dev_err(data->iommu_dev,
					"(%s)Enable stall failed\n",
					data->dbgname);
				return ret;
			}
		}

		ret = rk_iommu_force_reset(data, data->res_bases[i]);
		if (ret)
			return ret;

		rk_iommu_write(base, RK_MMU_DTE_ADDR, dte_addr);
		rk_iommu_command(base, RK_MMU_CMD_ZAP_CACHE);
		if (data->mmu_flag & RK_MMU_VOP_WORKAROUND)
			rk_iommu_write(base, RK_MMU_INT_MASK, 0);
		else
			rk_iommu_write(base, RK_MMU_INT_MASK, RK_MMU_IRQ_MASK);

		ret = rk_iommu_enable_paging(data, base);
		if (ret) {
			dev_err(data->iommu_dev, "(%s)Enable paging failed\n",
				data->dbgname);
			return ret;
		}
		dev_dbg(data->iommu_dev, "(%s) Enabled\n", data->dbgname);

		if (!(data->mmu_flag & RK_MMU_VOP_WORKAROUND))
			rk_iommu_disable_stall(base);
	}

	spin_lock_irqsave(&rk_domain->lock, flags);
	data->domain = domain;
	list_add_tail(&data->node, &rk_domain->clients);
	spin_unlock_irqrestore(&rk_domain->lock, flags);

	if (!(data->mmu_flag & RK_MMU_VPU_HEVC_WORKAROUND))
		dev_info(data->iommu_dev, "Attached IOMMU with pgtable 0x%08x\n",
			 (u32)virt_to_phys(rk_domain->pgtable));

	return 0;
}

static void rk_iommu_detach_device(struct iommu_domain *domain,
				struct device *dev)
{
	struct iommu_drvdata *data = dev_get_drvdata(dev->archdata.iommu);
	struct rk_iommu_domain *rk_domain = domain->priv;
	struct list_head *pos;
	unsigned long flags;
	bool found = false;
	int i;

	if (!rk_set_iommu_inactive(data)) {
		dev_info(data->iommu_dev, "(%s)%d times left to be disabled\n",
			 data->dbgname, data->activations);
		return;
	}

	spin_lock_irqsave(&rk_domain->lock, flags);
	list_for_each(pos, &rk_domain->clients) {
		if (list_entry(pos, struct iommu_drvdata, node) == data) {
			found = true;
			break;
		}
	}
	if (!found) {
		spin_unlock_irqrestore(&rk_domain->lock, flags);
		dev_err(data->iommu_dev,
			"(%s) can't find relative iommu node,detach failed\n",
			data->dbgname);
		return;
	}

	data->domain = NULL;
	list_del_init(&data->node);
	spin_unlock_irqrestore(&rk_domain->lock, flags);

	for (i = 0; i < data->num_mem; i++) {
		if (!(data->mmu_flag & RK_MMU_VOP_WORKAROUND))
			rk_iommu_enable_stall(data->res_bases[i]);
		rk_iommu_disable_paging(data, data->res_bases[i]);
		rk_iommu_write(data->res_bases[i], RK_MMU_INT_MASK, 0);
		rk_iommu_write(data->res_bases[i], RK_MMU_DTE_ADDR, 0);
		if (!(data->mmu_flag & RK_MMU_VOP_WORKAROUND))
			rk_iommu_disable_stall(data->res_bases[i]);
	}

	dev_dbg(data->iommu_dev, " disabled\n");

	if (!(data->mmu_flag & RK_MMU_VPU_HEVC_WORKAROUND))
		dev_info(data->iommu_dev,"Detached IOMMU with pgtable 0x%08x\n",
			 (u32)virt_to_phys(rk_domain->pgtable));
}

static int rk_iommu_domain_init(struct iommu_domain *domain)
{
	struct rk_iommu_domain *rk_domain;

	rk_domain = kzalloc(sizeof(*rk_domain), GFP_KERNEL);
	if (!rk_domain)
		return -ENOMEM;
	/*
	 * Rockchip iommu use a 2 level pagetable,
	 * level1 and leve2 both have 1024 entries,each entry  has 4 bytes,
	 * so alloc a page size for each pagetable
	 */
	rk_domain->pgtable = (u32 *)get_zeroed_page(GFP_KERNEL);
	if (!rk_domain->pgtable)
		goto err_pgtable;

	rk_table_flush(rk_domain->pgtable, NUM_DT_ENTRIES);

	spin_lock_init(&rk_domain->lock);
	spin_lock_init(&rk_domain->pgtablelock);
	INIT_LIST_HEAD(&rk_domain->clients);

	domain->priv = rk_domain;
	return 0;

err_pgtable:
	kfree(rk_domain);
	return -ENOMEM;
}

static void rk_iommu_domain_destroy(struct iommu_domain *domain)
{
	struct rk_iommu_domain *rk_domain = domain->priv;
	int i;

	WARN_ON(!list_empty(&rk_domain->clients));

	for (i = 0; i < NUM_DT_ENTRIES; i++) {
		u32 dte = rk_domain->pgtable[i];
		if (rk_dte_is_pt_valid(dte)) {
			phys_addr_t pt_phys = rk_dte_pt_address(dte);
			free_pages((unsigned long)phys_to_virt(pt_phys), 0);
		}
	}

	free_pages((unsigned long)rk_domain->pgtable, 0);
	kfree(domain->priv);
	domain->priv = NULL;
}

static struct iommu_ops rk_iommu_ops = {
	.domain_init = rk_iommu_domain_init,
	.domain_destroy = rk_iommu_domain_destroy,
	.attach_dev = rk_iommu_attach_device,
	.detach_dev = rk_iommu_detach_device,
	.map = rk_iommu_map,
	.unmap = rk_iommu_unmap,
	.iova_to_phys = rk_iommu_iova_to_phys,
	.pgsize_bitmap = RK_IOMMU_PGSIZE_BITMAP,
};

static int  rk_get_iommu_resource(struct platform_device *pdev,
					     unsigned int type)
{
	int num = 0;
	int i;

	for (i = 0; i < pdev->num_resources; i++) {
		struct resource *r = &pdev->resource[i];
		if (type == resource_type(r))
			num++;
	}

	return num;
}

static int rk_iommu_probe(struct platform_device *pdev)
{
	int i, ret;
	struct device *dev;
	struct iommu_drvdata *data;

	dev = &pdev->dev;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dev_set_drvdata(dev, data);

	if (!pdev->dev.of_node) {
		dev_err(dev, "need dts node\n");
		return -ENOENT;
	}

	of_property_read_string(pdev->dev.of_node, "dbgname", &(data->dbgname));
	if (!data->dbgname) {
		dev_err(dev, "need dbgname in dts\n");
		return -ENOENT;
	}

	dev_info(dev,"(%s) Enter\n", data->dbgname);

	data->num_mem = rk_get_iommu_resource(pdev, IORESOURCE_MEM);
	if (0 == data->num_mem) {
		dev_err(dev, "can't find iommu memory resource\n");
		return -ENOMEM;
	}
	dev_dbg(dev,"data->num_mem=%d\n", data->num_mem);

	data->num_irq = rk_get_iommu_resource(pdev, IORESOURCE_IRQ);
	if (0 == data->num_irq) {
		dev_err(dev,"can't find iommu irq resource\n");
		return -ENOMEM;
	}
	dev_dbg(dev,"data->num_irq=%d\n", data->num_irq);
	if (data->num_irq >= ARRAY_SIZE(data->irq_array)) {
		dev_err(dev, "invalid num of irq %d\n", data->num_irq);
		return -EINVAL;
	}

	data->res_bases = devm_kmalloc_array(dev, data->num_mem,
				sizeof(*data->res_bases), GFP_KERNEL);
	if (data->res_bases == NULL)
		return -ENOMEM;

	for (i = 0; i < data->num_mem; i++) {
		struct resource *res;

		res = platform_get_resource(pdev, IORESOURCE_MEM, i);

		data->res_bases[i] = devm_ioremap(dev,res->start,
						  resource_size(res));
		if (IS_ERR(data->res_bases[i]))
			return PTR_ERR(data->res_bases[i]);

		dev_dbg(dev,"%pa ioremap to 0x%p\n", &res->start,
			data->res_bases[i]);
	}

#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
	if (strstr(data->dbgname, "vop") &&
		(soc_is_rk3128() || soc_is_rk3126()))
		data->mmu_flag |= RK_MMU_VOP_WORKAROUND;
#endif
	if (strstr(data->dbgname, "isp"))
		data->mmu_flag |= RK_MMU_ISP_WORKAROUND;
	if (strstr(data->dbgname, "vpu") || strstr(data->dbgname, "hevc"))
		data->mmu_flag |= RK_MMU_VPU_HEVC_WORKAROUND;

	dev_dbg(dev, "data->mmu_flag = 0x%08lx\n", data->mmu_flag);

	for (i = 0; i < data->num_irq; i++) {
		if (data->mmu_flag & RK_MMU_VOP_WORKAROUND) {
			dev_info(dev, "skip request vop mmu irq\n");
			continue;
		}

		data->irq_array[i] = platform_get_irq(pdev, i);
		dev_dbg(dev, "data->irq_array[%d] = %d\n", i, data->irq_array[i]);
	}

	ret = rockchip_init_iovmm(dev, &data->vmm);
	if (ret)
		return ret;

	data->iommu_dev = dev;
	INIT_LIST_HEAD(&data->node);

	dev_info(dev,"(%s) Initialized\n", data->dbgname);

	return 0;
}

static int rk_iommu_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id iommu_dt_ids[] = {
	{ .compatible = IEP_IOMMU_COMPATIBLE_NAME},
	{ .compatible = VIP_IOMMU_COMPATIBLE_NAME},
	{ .compatible = VOPB_IOMMU_COMPATIBLE_NAME},
	{ .compatible = VOPL_IOMMU_COMPATIBLE_NAME},
	{ .compatible = HEVC_IOMMU_COMPATIBLE_NAME},
	{ .compatible = VPU_IOMMU_COMPATIBLE_NAME},
	{ .compatible = ISP_IOMMU_COMPATIBLE_NAME},
	{ .compatible = VOP_IOMMU_COMPATIBLE_NAME},
	{ /* end */ }
};

MODULE_DEVICE_TABLE(of, iommu_dt_ids);
#endif

static struct platform_driver rk_iommu_driver = {
	.probe = rk_iommu_probe,
	.remove = rk_iommu_remove,
	.driver = {
		   .name = "rk_iommu",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(iommu_dt_ids),
	},
};

static int __init rk_iommu_init_driver(void)
{
	int ret;

	ret = bus_set_iommu(&platform_bus_type, &rk_iommu_ops);
	if (ret)
		return ret;

	return platform_driver_register(&rk_iommu_driver);
}

static void __exit rk_iommu_exit(void)
{
	platform_driver_unregister(&rk_iommu_driver);
}

core_initcall(rk_iommu_init_driver);
module_exit(rk_iommu_exit);
