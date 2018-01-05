/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/version.h>
#include "imc_idi_gnss.h"
#include "imc_idi_gnss_regs.h"
#include "imc_idi_gnss_ioctl.h"
#include "imc_idi_gnss_of.h"
#include "imc_idi_gnss_pm.h"
#include <linux/idi/idi_device_pm.h>

#define ABB_CGU_GPS_PHX_CONTROL		((void __iomem *) 0xE6401188)
#define ABB_SCU_PCL15			((void __iomem *) 0xE6300248)

static void gnss_tx_transaction_complete(struct idi_transaction *);
static void idi_gnss_stop_tx(struct uart_port *port);
static int gnss_set_idi_power_state(struct idi_peripheral_device *p_device,
		int state);

static DECLARE_WAIT_QUEUE_HEAD(ioctl_wq);
static DECLARE_WAIT_QUEUE_HEAD(gps_on_wq);
static DECLARE_WAIT_QUEUE_HEAD(bb_clk_wq);

enum {
	GPS_UNINITIALIZED	= 0,
	GPS_POWER_OFF		= 1,
	GPS_POWER_ON		= 2,
	GPS_DEEP_COMA		= 3,
} gnss_gps_state;

static struct gnss_pm_state_entry {
	const char *name;
	struct device_state_pm_state *handler;
} gnss_pm_state_array[] = {
	/* D0	*/{ .name = "enable_def_dclk_832_pclk_1248"},
	/* D0I0 */{ .name = "enable_dclk_96_pclk_1248"},
	/* D0I1	*/{ .name = "enable_dclk_832_pclk_104"},
	/* D0I2	*/{ .name = "enable_dclk_96_pclk_96"},
	/* D0I3	*/{ .name = "idle"},
	/* D3	*/{ .name = "disable"},
};

/*****************************************************************************
 *	name:		wakeup_gnss
 *	description:	toggel SLEEP_INT bit to wake up the GNSS core
 *	in params:	struct idi_gnss_port *p_gnss
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static inline void wakeup_gnss(struct idi_gnss_port *p_gnss)
{
	phys_addr_t gnss = p_gnss->gnss_io;
	struct device *dev = &p_gnss->p_dev->device;
	unsigned scu_gnss_ctl1;

	dev_dbg(dev, "Waking GNSS core, toggle SLEEP_INT\n");
	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_GNSS_CTL1(gnss), &scu_gnss_ctl1))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_GNSS_CTL1(gnss));
	scu_gnss_ctl1 |= SCU_GNSS_CTL1_SLEEP_INT_MASK;
	if (idi_client_iowrite(p_gnss->p_dev, (unsigned)SCU_GNSS_CTL1(gnss), scu_gnss_ctl1))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_GNSS_CTL1(gnss));
	udelay(20);
	scu_gnss_ctl1 &= ~SCU_GNSS_CTL1_SLEEP_INT_MASK;
	if (idi_client_iowrite(p_gnss->p_dev, (unsigned)SCU_GNSS_CTL1(gnss), scu_gnss_ctl1))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_GNSS_CTL1(gnss));
}

static inline struct idi_gnss_port *to_gnss_port(struct uart_port *port)
{
	return xgold_port_priv(to_usif_port(port));
}

static inline struct idi_gnss_port *idi_dev_to_gnss_port(
		struct idi_peripheral_device *p_dev) {
	struct device *dev = &p_dev->device;
	struct uart_usif_xgold_port *uxp = dev_get_drvdata(dev);
	struct idi_gnss_port *g_port = xgold_port_priv(uxp);

	return g_port;
}

static inline struct imc_idi_gnss_platdata *idi_dev_to_gnss_platdata(
			struct idi_peripheral_device *p_dev)
{
	struct device *dev = &p_dev->device;
	struct xgold_usif_platdata *uxp_platdata = dev_get_platdata(dev);
	struct imc_idi_gnss_platdata *platdata;

	platdata = xgold_port_platdata_priv(uxp_platdata);

	return platdata;
}

/*****************************************************************************
 *	name:		gnss_pm_sem_down
 *	description:	sem_down gnss wrapper
 *	in params:	struct idi_gnss_port *p_gnss
 *			const char *func - func name for debug
 *			bool use_to - 0 use down_interruptible, 1 - use
 *			down_timeout
 *	out params:	none
 *	return val:	-EINTR on interrupted
 *			-ETIME on timeout
 *			0 - success
 ****************************************************************************/

#define GNSS_SEMDOWN_TO (2*HZ)

static int gnss_pm_sem_down(struct idi_gnss_port *p_gnss, const char *func,
		bool use_to)
{
	struct device *dev = &p_gnss->p_dev->device;
	int ret;

	dev_dbg(dev, "%s: locking pm_sem\n", func);
	if (!use_to) {
		if (down_interruptible(&p_gnss->pm_sem)) {
			dev_err(dev, "%s: interrupted while locking pm_sem\n",
					func);
			return -EINTR;
		}
	} else {
		ret = down_timeout(&p_gnss->pm_sem, GNSS_SEMDOWN_TO);
		if (ret < 0)
			goto sem_down_err;
	}
	dev_dbg(dev, "%s: locked pm_sem. count %d\n", func,
			p_gnss->pm_sem.count);
	return 0;

sem_down_err:
	if (ret < 0) {
		if (ret == -ETIME)
			dev_err(dev, "%s: timeout locking sem\n", func);
		else
			dev_err(dev, "%s: interrupted locking pm_sem\n", func);
	}
	return ret;
}

/*****************************************************************************
 *	name:		gnss_pm_sem_up
 *	description:	sem_up gnss wrapper
 *	in params:	struct idi_gnss_port *p_gnss
 *			const char *func - func name for debug
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void gnss_pm_sem_up(struct idi_gnss_port *p_gnss, const char *func)
{
	struct device *dev = &p_gnss->p_dev->device;

	dev_dbg(dev, "%s: freeing pm_sem\n", func);
	up(&p_gnss->pm_sem);
	dev_dbg(dev, "%s: sem count %d\n", func, p_gnss->pm_sem.count);
}

/*****************************************************************************
 *	name:		freq_to_power_state
 *	description:	convert frequency to power state
 *	in params:	int freq
 *	out params:	none
 *	return val:	int power_state (D0, D0I0 ...)
 ****************************************************************************/
static inline int freq_to_power_state(int freq)
{
	int power_state = -1;

	IMC_IDI_GNSS_ENTER;

	switch (freq) {
	case FMR_FRQ_DEF_DCLK_832_PCLK_1248:
		power_state = D0;
		break;
	case FMR_FRQ_DCLK_96_PCLK_1248:
		power_state = D0I0;
		break;
	case FMR_FRQ_DCLK_832_PCLK_104:
		power_state = D0I1;
		break;
	case FMR_FRQ_DCLK_96_PCLK_96:
		power_state = D0I2;
		break;
	default:
		break;
	}

	IMC_IDI_GNSS_EXIT;

	return power_state;
}

static inline int power_state_to_freq(int power_state)
{
	int freq = -1;

	IMC_IDI_GNSS_ENTER;

	switch (power_state) {
	case D0:
		freq = FMR_FRQ_DEF_DCLK_832_PCLK_1248;
		break;
	case D0I0:
		freq = FMR_FRQ_DCLK_96_PCLK_1248;
		break;
	case D0I1:
		freq = FMR_FRQ_DCLK_832_PCLK_104;
		break;
	case D0I2:
		freq = FMR_FRQ_DCLK_96_PCLK_96;
		break;
	default:
		break;
	}

	IMC_IDI_GNSS_EXIT;

	return power_state;
}


/*****************************************************************************
 *	name:		idi_gnss_wkp_isr
 *	description:	handle any irq from C5 IRQ register by mask clear it and
 *			invoke the bhthread to take care of it
 *	in params:	int irq
 *			void *dev_id
 *	out params:	none
 *	return val:	IRQ_WAKE_THREAD
 ****************************************************************************/
static irqreturn_t idi_gnss_wkp_isr(int irq, void *dev_id)
{
	struct idi_gnss_port *p_gnss = (struct idi_gnss_port *) dev_id;
	struct device *dev = &p_gnss->p_dev->device;
	phys_addr_t gnss = p_gnss->gnss_io;
	unsigned irq_mask, irq_stat, irq_clr, reg, scu_gnss_stat1;

	IMC_IDI_GNSS_ENTER;

	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_C5_IRQSM(gnss), &irq_mask))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_C5_IRQSM(gnss));

	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_C5_IRQSS(gnss), &irq_stat))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_C5_IRQSS(gnss));
	irq_clr = 0;
	p_gnss->wk_irq_stat = irq_stat & irq_mask;

	dev_dbg(dev, "%s: mask %x, stat %x, stat thread %x\n",
			 __func__, irq_mask, irq_stat, p_gnss->wk_irq_stat);

	irq_stat = p_gnss->wk_irq_stat;

	if (SCU_C5_IRQSS_GPS_EN_LDO_BB(irq_stat)) {
		/* indicate powerup or exiting deep coma */
		dev_dbg(dev, "GPS_EN_LDO_BB irq\n");

	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_GNSS_STAT1(gnss), &scu_gnss_stat1))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_GNSS_STAT1(gnss));

		if (!SCU_GNSS_STAT1_GPS_EN_LDO_BB(scu_gnss_stat1)) {
			dev_dbg(dev, "GNSS is in DEEP_COMA\n");
			p_gnss->gps_state = GPS_DEEP_COMA;
		}

		irq_mask &= ~(SCU_C5_IRQSM_GPS_EN_LDO_BB_MASK);
		irq_clr |= SCU_C5_IRQSC_GPS_EN_LDO_BB_CLEAR_INTERRUPT;
	}

	if (SCU_C5_IRQSS_GPS_TO_ARM_0(irq_stat)) {
		/* indicate gnss firmware have something to send */
		dev_dbg(dev, "GPS_TO_ARM_0 irq\n");
		irq_mask &= ~(SCU_C5_IRQSM_GPS_TO_ARM_0_MASK);
		irq_clr |= SCU_C5_IRQSC_GPS_TO_ARM_0_CLEAR_INTERRUPT;
	}

	if (SCU_C5_IRQSS_GNSS_GPIO_WAKE(irq_stat)) {
		/* gnss firmware want to wake the host */
		dev_dbg(dev, "GNSS_GPIO_WAKE irq\n");
		irq_mask &= ~(SCU_C5_IRQSS_GNSS_GPIO_WAKE_MASK);
		irq_clr |= SCU_C5_IRQSC_GNSS_GPIO_WAKE_CLEAR_INTERRUPT;
	}

	/* mask the irqs */
	if (idi_client_iowrite(p_gnss->p_dev, (unsigned)SCU_C5_IRQSM(gnss), irq_mask))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C5_IRQSM(gnss));

	/* clear irq & zero clear bits */
	if (idi_client_iowrite(p_gnss->p_dev, (unsigned)SCU_C5_IRQSC(gnss), irq_clr))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C5_IRQSC(gnss));

	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_C5_IRQSC(gnss), &reg))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_C5_IRQSC(gnss));

	if (idi_client_iowrite(p_gnss->p_dev, (unsigned)SCU_C5_IRQSC(gnss), 0))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C5_IRQSC(gnss));

	IMC_IDI_GNSS_EXIT;

	return IRQ_WAKE_THREAD;
}

/*****************************************************************************
 *	name:		idi_gnss_start_hw
 *	description:	Take out from RESET state the GNSS core if we came from
 *			GPS_POWER_OFF) and start the DSP
 *	in params:	struct idi_gnss_port *p_gnss
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void idi_gnss_start_hw(struct idi_gnss_port *p_gnss)
{
	phys_addr_t gnss = p_gnss->gnss_io;
	struct device *dev = &p_gnss->p_dev->device;
	unsigned scu_gnss_ctl1;
	unsigned long flags;

	IMC_IDI_GNSS_ENTER;

	spin_lock_irqsave(&p_gnss->hw_lock, flags);

	if ((p_gnss->gps_state == GPS_POWER_OFF) ||
			(p_gnss->gps_state == GPS_UNINITIALIZED)) {
		dev_dbg(dev, "gps state is off. take out of reset\n");

		/* take out of reset and enable the following: */
		if (idi_client_ioread(p_gnss->p_dev,
				(unsigned)SCU_GNSS_CTL1(gnss), &scu_gnss_ctl1))
			dev_err(dev, "%s: idi read error %x\n", __func__, SCU_GNSS_CTL1(gnss));

		scu_gnss_ctl1 |= (SCU_GNSS_CTL1_NPORST_NORST
			| SCU_GNSS_CTL1_PMU_RSTN_NORST
			| SCU_GNSS_CTL1_BB_RSTN_NORST
			| SCU_GNSS_CTL1_WDG_RSTN_NORST);
		scu_gnss_ctl1 &= ~(SCU_GNSS_CTL1_POWERONB_OFF);
		if (idi_client_iowrite(p_gnss->p_dev, (unsigned)SCU_GNSS_CTL1(gnss), scu_gnss_ctl1))
			dev_err(dev, "%s: idi write error %x\n", __func__, SCU_GNSS_CTL1(gnss));

		dev_dbg(dev, "%s: Setting GNSS_CTL1 to 0x%08X\n",
				__func__, scu_gnss_ctl1);
		spin_unlock_irqrestore(&p_gnss->hw_lock, flags);
		mdelay(10);
		spin_lock_irqsave(&p_gnss->hw_lock, flags);
	}

	/* 3. start the DSP */
	if (idi_client_ioread(p_gnss->p_dev,
			(unsigned)SCU_GNSS_CTL1(gnss), &scu_gnss_ctl1))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_GNSS_CTL1(gnss));
	scu_gnss_ctl1 |= SCU_GNSS_CTL1_DSP_START_MASK;
	if (idi_client_iowrite(p_gnss->p_dev, (unsigned)SCU_GNSS_CTL1(gnss), scu_gnss_ctl1))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_GNSS_CTL1(gnss));
	dev_dbg(dev, "%s: Setting GNSS_CTL1 to 0x%08X\n", __func__, scu_gnss_ctl1);

	/* modify the current state */
	dev_dbg(dev, "gps state is now on\n");
	p_gnss->gps_state = GPS_POWER_ON;
	if (waitqueue_active(&gps_on_wq)){
		dev_dbg(dev, "wakeup gps_on_wq\n");
		wake_up_interruptible(&gps_on_wq);
	}

	spin_unlock_irqrestore(&p_gnss->hw_lock, flags);

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		idi_gnss_stop_hw
 *	description:	Put the GNSS core in RESET state and release IDI bus
 *	in params:	struct idi_gnss_port *p_gnss
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void idi_gnss_stop_hw(struct idi_gnss_port *p_gnss)
{
	phys_addr_t gnss = p_gnss->gnss_io;
	struct device *dev = &p_gnss->p_dev->device;
	unsigned long flags;
	unsigned scu_gnss_ctl1;
	bool arm_to_gps_is_up = false;

	IMC_IDI_GNSS_ENTER;

	spin_lock_irqsave(&p_gnss->hw_lock, flags);

	if (idi_client_ioread(p_gnss->p_dev,
			(unsigned)SCU_GNSS_CTL1(gnss), &scu_gnss_ctl1))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_GNSS_CTL1(gnss));

	if (scu_gnss_ctl1 & SCU_GNSS_CTL1_ARM_TO_GPS_0_MASK)
		arm_to_gps_is_up = true;

	/* stop DSP */
	scu_gnss_ctl1 &= ~(SCU_GNSS_CTL1_DSP_START_START);
	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_GNSS_CTL1(gnss), scu_gnss_ctl1))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_GNSS_CTL1(gnss));
	dev_dbg(dev, "%s: Setting GNSS_CTL1 to 0x%08X\n", __func__,
			scu_gnss_ctl1);

	udelay(100);

	/* put GNSS in reset and disable the following: */
	scu_gnss_ctl1 &= ~(SCU_GNSS_CTL1_NPORST_NORST |
			SCU_GNSS_CTL1_PMU_RSTN_NORST |
			SCU_GNSS_CTL1_BB_RSTN_NORST |
			SCU_GNSS_CTL1_WDG_RSTN_NORST |
			SCU_GNSS_CTL1_ARM_TO_GPS_0_MASK);
	scu_gnss_ctl1 |= (SCU_GNSS_CTL1_POWERONB_OFF);

	if (idi_client_iowrite(p_gnss->p_dev,
			(unsigned)SCU_GNSS_CTL1(gnss), scu_gnss_ctl1))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_GNSS_CTL1(gnss));
	dev_dbg(dev, "%s: Setting GNSS_CTL1 to 0x%08X\n", __func__,
			scu_gnss_ctl1);

	/* modify the current state */
	dev_dbg(dev, "gps state is now GPS_POWER_OFF\n");
	p_gnss->gps_state = GPS_POWER_OFF;

	if(arm_to_gps_is_up && (p_gnss->pm_sem.count == 0)){
		/* avoid deadlock .. the rx msgs will
		 * not arrive in such case. */
		dev_dbg(dev, "releasing pm_sem\n");
		gnss_pm_sem_up(p_gnss, __func__);
	}

	spin_unlock_irqrestore(&p_gnss->hw_lock, flags);

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		idi_gnss_wkp_isr_bh
 *	description:	bh_thread that handles any irq from C5 IRQ register.
 *	in params:	int irq, void *dev_id
 *	out params:	none
 *	return val:	IRQ_HANDLED
 *****************************************************************************/
static irqreturn_t idi_gnss_wkp_isr_bh(int irq, void *dev_id)
{
	struct idi_gnss_port *p_gnss = (struct idi_gnss_port *) dev_id;
	struct device *dev = &p_gnss->p_dev->device;
	struct idi_peripheral_device *p_device = p_gnss->p_dev;
	struct uart_usif_xgold_port *uxp = dev_get_drvdata(dev);
	struct uart_port *port = &uxp->port;
	struct tty_port *t_port = &port->state->port;
	struct tty_struct *tty = t_port->tty;
	phys_addr_t gnss = p_gnss->gnss_io;
	unsigned irq_mask, irq_stat, scu_gnss_ctl1, scu_gnss_stat1;
	unsigned long flags;
	bool wake_bb = false;

	IMC_IDI_GNSS_ENTER;

	/* get the irq state & mask */
	spin_lock_irqsave(&p_gnss->hw_lock, flags);
	irq_stat = p_gnss->wk_irq_stat;

	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_C5_IRQSM(gnss), &irq_mask))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_C5_IRQSM(gnss));

	dev_dbg(dev, "%s: stat %x\n", __func__, irq_stat);

	if (SCU_C5_IRQSS_GPS_EN_LDO_BB(irq_stat)) {
		/* check GPS_EN_LDO_BB bit */
		if (idi_client_ioread(p_gnss->p_dev,
				(unsigned)SCU_GNSS_STAT1(gnss), &scu_gnss_stat1))
			dev_err(dev, "%s: idi read error %x\n", __func__, SCU_GNSS_STAT1(gnss));
		if (SCU_GNSS_STAT1_GPS_EN_LDO_BB(scu_gnss_stat1)) {
			dev_dbg(dev, "GNSS is out of RESET/DEEP_COMA\n");
			/* activate IDI bus */
			spin_unlock_irqrestore(&p_gnss->hw_lock, flags);
			gnss_pm_sem_down(p_gnss, __func__, false);
			gnss_set_idi_power_state(p_device,
				freq_to_power_state(p_gnss->freq));
			/* start h/w */
			idi_gnss_start_hw(p_gnss);
			gnss_pm_sem_up(p_gnss, __func__);
			spin_lock_irqsave(&p_gnss->hw_lock, flags);

		} else {
			/* we go to deep-come state, stop dsp */
			dev_dbg(dev, "stopping DSP\n");
			if (idi_client_ioread(p_gnss->p_dev,
				(unsigned)SCU_GNSS_CTL1(gnss), &scu_gnss_ctl1))
				dev_err(dev, "%s: idi read error %x\n", __func__, SCU_GNSS_CTL1(gnss));
			scu_gnss_ctl1 &= ~SCU_GNSS_CTL1_DSP_START_MASK;
			if (idi_client_iowrite(p_gnss->p_dev,
				(unsigned)SCU_GNSS_CTL1(gnss), scu_gnss_ctl1))
				dev_err(dev, "%s: idi write error %x\n", __func__, SCU_GNSS_CTL1(gnss));
			dev_dbg(dev, "%s: Setting GNSS_CTL1 to 0x%08X\n",
					__func__, scu_gnss_ctl1);

			if ((atomic_read(&p_gnss->tx_trans_in_process) > 0) ||
				(atomic_read(&p_gnss->rx_trans_in_process) > 0)) {
				/* tx/rx in process .. stay up */
				dev_dbg(dev, "TX/RX in progress.\n");
				wake_bb = true;
			}
			if (SCU_GNSS_CTL1_ARM_TO_GPS_0(scu_gnss_ctl1)) {
				/* disable ARM_TO_GPS ...*/
				dev_dbg(dev, "Set ARM_TO_GPS_0 0\n");
				scu_gnss_ctl1 &=
					~SCU_GNSS_CTL1_ARM_TO_GPS_0_MASK;
				if (idi_client_iowrite(p_gnss->p_dev, (unsigned)SCU_GNSS_CTL1(gnss), scu_gnss_ctl1))
					dev_err(dev, "%s: idi write error %x\n", __func__, SCU_GNSS_CTL1(gnss));
				if (p_gnss->pm_sem.count == 0) {
					/* avoid deadlock .. the rx msgs will
					 * not arrive in such case. */
					dev_dbg(dev, "releasing pm_sem\n");
					gnss_pm_sem_up(p_gnss, __func__);
				}
			} 

			if(wake_bb) {
				wakeup_gnss(p_gnss);
			} else {
				/* deactivate the IDI bus */
				spin_unlock_irqrestore(&p_gnss->hw_lock, flags);
				gnss_pm_sem_down(p_gnss, __func__, false);
				gnss_set_idi_power_state(p_device, D3);
				gnss_pm_sem_up(p_gnss, __func__);
				spin_lock_irqsave(&p_gnss->hw_lock, flags);

				if (tty->hw_stopped) {
					dev_dbg(dev, "tty->hw_stopped to 0\n");
					tty->hw_stopped = 0;
				}
			}
		}

		p_gnss->wk_irq_stat &= ~SCU_C5_IRQSM_GPS_EN_LDO_BB_MASK;
		/* unmask the IRQ */
		irq_mask |= SCU_C5_IRQSM_GPS_EN_LDO_BB_MASK;
	}

	if (SCU_C5_IRQSS_GPS_TO_ARM_0(irq_stat)) {
		dev_dbg(dev, "handling GPS_TO_ARM_0\n");

		spin_unlock_irqrestore(&p_gnss->hw_lock, flags);
		/* activate IDI bus */
		gnss_pm_sem_down(p_gnss, __func__, false);
		gnss_set_idi_power_state(p_device, freq_to_power_state(p_gnss->freq));
		spin_lock_irqsave(&p_gnss->hw_lock, flags);

		/* check if GNSS is sleeping */
		if (idi_client_ioread(p_gnss->p_dev,
			(unsigned)SCU_GNSS_STAT1(gnss), &scu_gnss_stat1))
			dev_err(dev, "%s: idi read error %x\n", __func__, SCU_GNSS_STAT1(gnss));

		if ((p_gnss->gps_state != GPS_POWER_ON) ||
			(p_gnss->bb_clk_state == false) ||
			!SCU_GNSS_STAT1_GPS_TO_ARM_0(scu_gnss_stat1)) {
			/* if GNSS is sleeping/request canceled 
			 * we cannot expect RX msgs will come. ignore */
			dev_warn(dev, "GNSS is a sleep/GPS_TO_ARM is 0. ignore irq\n");
			gnss_pm_sem_up(p_gnss, __func__);
		}
		else {
			/* reply this irq with ARM_TO_GPS_0 */
			dev_dbg(dev, "setting ARM_TO_GPS_0 to 1\n");
			if (idi_client_ioread(p_gnss->p_dev,
				(unsigned)SCU_GNSS_CTL1(gnss), &scu_gnss_ctl1))
				dev_err(dev, "%s: idi read error %x\n", __func__, SCU_GNSS_CTL1(gnss));
			scu_gnss_ctl1 |= SCU_GNSS_CTL1_ARM_TO_GPS_0_MASK;
			if (idi_client_iowrite(p_gnss->p_dev,
					(unsigned)SCU_GNSS_CTL1(gnss), scu_gnss_ctl1))
				dev_err(dev, "%s: idi write error %x\n", __func__, SCU_GNSS_CTL1(gnss));
			dev_dbg(dev, "%s: Setting GNSS_CTL1 to 0x%08X\n",
					__func__, scu_gnss_ctl1);
		}

		p_gnss->wk_irq_stat &= ~SCU_C5_IRQSS_GPS_TO_ARM_0_MASK;

		/* unmask the irq */
		irq_mask |= SCU_C5_IRQSM_GPS_TO_ARM_0_MASK;
	}

	if (SCU_C5_IRQSS_GNSS_GPIO_WAKE(irq_stat)) {
		dev_dbg(dev, "handling GNSS_GPIO_WAKE\n");

		p_gnss->wk_irq_stat &= ~SCU_C5_IRQSM_GNSS_GPIO_WAKE_MASK;
		/* unmask the irq */
		irq_mask |= SCU_C5_IRQSM_GNSS_GPIO_WAKE_MASK;
	}

	/* unmask handled irqs */
	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_C5_IRQSM(gnss), irq_mask))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C5_IRQSM(gnss));

	IMC_IDI_GNSS_EXIT;

	spin_unlock_irqrestore(&p_gnss->hw_lock, flags);

	return IRQ_HANDLED;
}

/*****************************************************************************
 *	name:		idi_gnss_notification_isr
 *	description:	handle any irq from C7 IRQ register by mask clear it
 *			and invoke the bhthread to take care of it
 *	in params:	int irq
 *			void *dev_id
 *	out params:	none
 *	return val:	IRQ_WAKE_THREAD
 ****************************************************************************/
static irqreturn_t idi_gnss_notification_isr(int irq, void *dev_id)
{
	struct idi_gnss_port *p_gnss = (struct idi_gnss_port *) dev_id;
	struct device *dev = &p_gnss->p_dev->device;
	phys_addr_t gnss = p_gnss->gnss_io;
	unsigned irq_mask, irq_stat, reg, scu_gnss_stat1, scu_gnss_ctl1;

	IMC_IDI_GNSS_ENTER;

	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_C7_IRQSM(gnss), &irq_mask))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_C7_IRQSM(gnss));
	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_C7_IRQSS(gnss), &irq_stat))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_C7_IRQSS(gnss));

	p_gnss->notify_irq_stat = irq_stat & irq_mask;

	dev_dbg(dev, "%s: mask %x, stat %x, stat thread %x\n", __func__,
			irq_mask, irq_stat, p_gnss->notify_irq_stat);
	irq_stat = p_gnss->notify_irq_stat;

	if (SCU_C7_IRQSM_GPS_BB_CLK_REQ(irq_stat)){
		dev_dbg(dev, "GPS_BB_CLK_REQ\n");
		if (idi_client_ioread(p_gnss->p_dev,
			(unsigned)SCU_GNSS_STAT1(gnss), &scu_gnss_stat1))
			dev_err(dev, "%s: idi read error %x\n", __func__, SCU_GNSS_STAT1(gnss));
		if (SCU_GNSS_STAT1_GPS_BB_CLK_REQ(scu_gnss_stat1)){
			dev_dbg(dev, "bb_clk_state is on\n");
			p_gnss->bb_clk_state = true;
		}
		else {
			dev_dbg(dev, "bb_clk_state is off\n");
			p_gnss->bb_clk_state = false;

			if (idi_client_ioread(p_gnss->p_dev,
					(unsigned)SCU_GNSS_CTL1(gnss), &scu_gnss_ctl1))
				dev_err(dev, "%s: idi read error %x\n", __func__, SCU_GNSS_CTL1(gnss));

			if (scu_gnss_ctl1 & SCU_GNSS_CTL1_ARM_TO_GPS_0_MASK) {
				/* remove the ACK bit so the f/w could send the next msg */
				scu_gnss_ctl1 &= ~SCU_GNSS_CTL1_ARM_TO_GPS_0_MASK;
				if (idi_client_iowrite(p_gnss->p_dev,
						(unsigned)SCU_GNSS_CTL1(gnss), scu_gnss_ctl1))
					dev_err(dev, "%s: idi write error %x\n", __func__, SCU_GNSS_CTL1(gnss));
				dev_dbg(dev, "%s: Setting GNSS_CTL1 to 0x%08X\n", __func__, scu_gnss_ctl1);
				if (p_gnss->pm_sem.count == 0) {
					/* avoid deadlock .. the rx msgs will
					 * not arrive in such case. */
					dev_dbg(dev, "releasing pm_sem\n");
					gnss_pm_sem_up(p_gnss, __func__);
				}
			}

		}
		if (waitqueue_active(&bb_clk_wq)){
			dev_dbg(dev, "wakeup bb_clk_wq\n");
			wake_up_interruptible(&bb_clk_wq);
		}
	}

	/* clear irq & zero clear bits */
	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_C7_IRQSC(gnss), irq_stat))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C7_IRQSC(gnss));

	if (idi_client_ioread(p_gnss->p_dev,
			(unsigned)SCU_C7_IRQSC(gnss), &reg))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_C7_IRQSC(gnss));

	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_C7_IRQSC(gnss), 0))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C7_IRQSC(gnss));

	p_gnss->notify_irq_stat = 0;

	IMC_IDI_GNSS_EXIT;

	return IRQ_HANDLED;
}

/*****************************************************************************
 *	name:		idi_gnss_wkp_isr
 *	description:	handle any irq from C6 IRQ register by mask clear it and
 *			invoke the bhthread to take care of it
 *	in params:	int irq
 *			void *dev_id
 *	out params:	none
 *	return val:	IRQ_WAKE_THREAD
 ****************************************************************************/
static irqreturn_t idi_gnss_error_isr(int irq, void *dev_id)
{
	struct idi_gnss_port *p_gnss = (struct idi_gnss_port *) dev_id;
	struct device *dev = &p_gnss->p_dev->device;
	phys_addr_t gnss = p_gnss->gnss_io;
	unsigned irq_mask, irq_stat, reg;

	IMC_IDI_GNSS_ENTER;

	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_C6_IRQSM(gnss), &irq_mask))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_C6_IRQSM(gnss));

	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_C6_IRQSS(gnss), &irq_stat))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_C6_IRQSS(gnss));

	p_gnss->error_irq_stat = irq_stat & irq_mask;

	dev_dbg(dev, "%s: mask %x, stat %x, stat thread %x\n",
			 __func__, irq_mask, irq_stat,
			 p_gnss->error_irq_stat);

	irq_stat = p_gnss->error_irq_stat;

	/* Mask the interrupts */
	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_C6_IRQSM(gnss), &irq_mask))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_C6_IRQSM(gnss));

	irq_mask &= ~irq_stat;

	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_C6_IRQSM(gnss), irq_mask))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C6_IRQSM(gnss));

	/* Clear the interrupts */

	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_C6_IRQSC(gnss), irq_stat))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C6_IRQSC(gnss));

	/* Dummy read */
	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_C6_IRQSC(gnss), &reg))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_C6_IRQSC(gnss));

	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_C6_IRQSC(gnss), 0))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C6_IRQSC(gnss));

	IMC_IDI_GNSS_EXIT;
	return IRQ_WAKE_THREAD;
}

/*****************************************************************************
 *	name:		idi_gnss_error_isr_bh
 *	description:	bh_thread that handles any irq from C6 IRQ register.
 *	in params:	int irq
 *			void *dev_id
 *	out params:	none
 *	return val:	IRQ_HANDLED
 ****************************************************************************/
static irqreturn_t idi_gnss_error_isr_bh(int irq, void *dev_id)
{
	struct idi_gnss_port *p_gnss = (struct idi_gnss_port *) dev_id;
	struct device *dev = &p_gnss->p_dev->device;
	phys_addr_t gnss = p_gnss->gnss_io;
	unsigned irq_mask, irq_stat;
	unsigned long flags;

	spin_lock_irqsave(&p_gnss->hw_lock, flags);

	IMC_IDI_GNSS_ENTER;

	irq_stat = p_gnss->error_irq_stat;

	dev_dbg(dev, "%s: stat %x\n", __func__, irq_stat);

	p_gnss->error_irq_stat = 0;

	/* Unmask the interrupts */

	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_C6_IRQSM(gnss), &irq_mask))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_C6_IRQSM(gnss));

	irq_mask |= irq_stat;
	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_C6_IRQSM(gnss), irq_mask))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C6_IRQSM(gnss));

	spin_unlock_irqrestore(&p_gnss->hw_lock, flags);

	IMC_IDI_GNSS_EXIT;

	return IRQ_HANDLED;
}

/*
 *This code has been duplicated twice (see gnss_dev_pm)
 *Should be a common helper code
 */
static const struct imc_idi_irq_desc {
	const char *name;
	irq_handler_t handler;
	irq_handler_t handler_bh;
	} irq_desc[] = {
	{ .name = GNSS_IRQ_WAKEUP_NAME,
		.handler = idi_gnss_wkp_isr,
		.handler_bh = idi_gnss_wkp_isr_bh},
	{ .name = GNSS_IRQ_NOTIFICATION_NAME,
		.handler = idi_gnss_notification_isr},
	{ .name = GNSS_IRQ_ERROR_NAME,
		.handler = idi_gnss_error_isr,
		.handler_bh = idi_gnss_error_isr_bh},
};

/*****************************************************************************
 *	name:		imc_idi_request_irqs
 *	description:	set IRQ handlers
 *	in params:	struct idi_peripheral_device *p_device
 *	out params:
 *	return val:
 ****************************************************************************/
static int imc_idi_request_irqs(struct idi_peripheral_device *p_device)
{
	int ret = 0;
	int i = 0;
	struct idi_resource *idi_res = &p_device->resources;
	struct idi_gnss_port *p_gnss = idi_dev_to_gnss_port(p_device);

	for (i = 0; i < ARRAY_SIZE(irq_desc); i++) {
		struct resource *res;
		const char *name = irq_desc[i].name;
		irq_handler_t handler = irq_desc[i].handler;
		irq_handler_t handler_bh = irq_desc[i].handler_bh;
		res = idi_get_resource_byname(idi_res, IORESOURCE_IRQ, name);
		if (!res) {
			ret = -EINVAL;
			goto fail_request_irq;
		}

		ret = request_threaded_irq(res->start, handler, handler_bh,
						IRQF_SHARED, name, p_gnss);
		if (ret) {
			dev_err(&p_device->device,
				"Error while requesting %d irq node\n",
							res->start);
			ret = -EINVAL;
			goto fail_request_irq;
		}
	}

	return 0;

fail_request_irq:
	for (i--; i > (-1); i--) {
		struct resource *res;
		const char *name = irq_desc[i].name;

		if (0 > i)
			BUG();

		res = idi_get_resource_byname(idi_res, IORESOURCE_IRQ, name);

		/* this should not happen */
		if (!res)
			BUG();

		free_irq(res->start, p_gnss);
	}
	return ret;
}

/*****************************************************************************
 *	name:		imc_idi_free_irqs
 *	description:	disable and release IRQ
 *	in params:	struct idi_peripheral_device *p_device
 *	out params:
 *	return val:
 ****************************************************************************/
static void imc_idi_free_irqs(struct idi_peripheral_device *p_device)
{
	int i;
	struct idi_resource *idi_res = &p_device->resources;
	struct resource *res;
	struct idi_gnss_port *p_gnss =
			idi_dev_to_gnss_port(p_device);

	for (i = 0; i < ARRAY_SIZE(irq_desc); i++) {
		const char *name = irq_desc[i].name;

		res = idi_get_resource_byname(idi_res, IORESOURCE_IRQ, name);
		if (res)
			free_irq(res->start, p_gnss);
	}
}

/*****************************************************************************
 *	name:		gnss_set_sp_pur_bits
 *	description:	set the correct value to SP_PUR bits per AGOLD ver. this
 *			is a w/a until platfrom power driver will do the actual
 *			setting of SP_PUR.
 *	in params:	struct idi_gnss_port *p_gnss
 *			bool enable - only relevant for ES2.0
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void gnss_set_sp_pur_bits(struct idi_gnss_port *p_gnss, bool enable)
{
	phys_addr_t scu = p_gnss->scu_io;
	unsigned scu_sp_pur;
	unsigned long flags;
	struct device *dev = &p_gnss->p_dev->device;

	IMC_IDI_GNSS_ENTER;

	spin_lock_irqsave(&p_gnss->hw_lock, flags);

	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_SP_PUR(scu), &scu_sp_pur))
		dev_err(dev, "%s: idi read error %x\n", __func__, (unsigned)SCU_SP_PUR(scu));

	switch (p_gnss->scu_chip_id) {
	case CHIP_ID_ES1:
		scu_sp_pur |= SCU_SP_PUR_AP_TCXO_REQ_REQ;
		scu_sp_pur &= ~SCU_SP_PUR_EXT_TCXO_LDO;
		scu_sp_pur |= SCU_SP_PUR_GNSS_BLANK_PCL;
		if (idi_client_iowrite(p_gnss->p_dev,
			(unsigned)ABB_SCU_PCL15, 0x3A101))
			dev_err(dev, "%s: idi write error %x\n", __func__, (unsigned)ABB_SCU_PCL15);
#ifdef GNSS_DCLK_FTR
		if (idi_client_iowrite(p_gnss->p_dev,
			(unsigned)ABB_CGU_GPS_PHX_CONTROL, p_gnss->cgu_gps_phx_control))
			dev_err(dev, "%s: idi write error %x\n", __func__, ABB_CGU_GPS_PHX_CONTROL);
#endif /* GNSS_DCLK_FTR */
		break;
	case CHIP_ID_ES1_1:
	case CHIP_ID_ES1_1_EXP:
		scu_sp_pur |= SCU_SP_PUR_AP_TCXO_REQ_REQ;
		scu_sp_pur &= ~SCU_SP_PUR_EXT_TCXO_LDO;
#ifdef GNSS_DCLK_FTR
		if (idi_client_iowrite(p_gnss->p_dev,
			(unsigned)ABB_CGU_GPS_PHX_CONTROL, p_gnss->cgu_gps_phx_control))
			dev_err(dev, "%s: idi write error %x\n", __func__, ABB_CGU_GPS_PHX_CONTROL);
#endif /*GNSS_DCLK_FTR */
		break;
	case CHIP_ID_ES2:
	case CHIP_ID_ES2_1:
		scu_sp_pur |= SCU_SP_PUR_EXT_TCXO_LDO;
		if (enable)
			scu_sp_pur |= SCU_SP_PUR_AP_TCXO_REQ_REQ;
		else
			scu_sp_pur &= ~SCU_SP_PUR_AP_TCXO_REQ_REQ;
		break;
	default:
		dev_err(dev, "No such CHIPID 0x%04x.\n", p_gnss->scu_chip_id);
		break;
	}

	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_SP_PUR(scu), scu_sp_pur))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_SP_PUR(scu));

	spin_unlock_irqrestore(&p_gnss->hw_lock, flags);

	dev_dbg(dev, "CHIPID 0x%04x scu_sp_pur = 0x%08X\n",
			p_gnss->scu_chip_id,
			scu_sp_pur);

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		gnss_es2_tcxo_setting
 *	description:	configure tcxo origin for ES2.0.
 *	in params:	struct idi_gnss_port *p_gnss
 *			bool digital: 1-digital, 0-analog.
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void gnss_es2_tcxo_setting(struct idi_gnss_port *p_gnss, bool digital)
{
	phys_addr_t gnss = p_gnss->gnss_io;
	unsigned scu_gnss_ctl1;
	unsigned long flags;
	struct device *dev = &p_gnss->p_dev->device;

	IMC_IDI_GNSS_ENTER;

	if ((p_gnss->scu_chip_id != CHIP_ID_ES2) &&
		(p_gnss->scu_chip_id != CHIP_ID_ES2_1)) {
		IMC_IDI_GNSS_EXIT;
		return;
	}

	spin_lock_irqsave(&p_gnss->hw_lock, flags);

	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_GNSS_CTL1(gnss), &scu_gnss_ctl1))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_GNSS_CTL1(gnss));
	if (digital)
		scu_gnss_ctl1 |= (SCU_GNSS_CTL1_ANA_PATH_ON_RF_EN |
				SCU_GNSS_CTL1_ANA_PATH_ON_DIG_EN);
	else
		scu_gnss_ctl1 &= ~(SCU_GNSS_CTL1_ANA_PATH_ON_RF_EN |
				SCU_GNSS_CTL1_ANA_PATH_ON_DIG_EN);
	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_GNSS_CTL1(gnss), scu_gnss_ctl1))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_GNSS_CTL1(gnss));

	dev_dbg(dev, "%s: Setting GNSS_CTL1 to 0x%08X\n", __func__,
			scu_gnss_ctl1);
	spin_unlock_irqrestore(&p_gnss->hw_lock, flags);

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		gnss_init_hw
 *	description:	init the required peripherals: clear, configure and
 *			enable IRQs, set SP_PUR bits and initialize h/w if
 *			ready.
 *	in params:
 *	out params:
 *	return val:
 ****************************************************************************/
static int gnss_init_hw(struct idi_gnss_port *p_gnss)
{
	phys_addr_t scu = p_gnss->scu_io;
	phys_addr_t gnss = p_gnss->gnss_io;
	unsigned irq_mask, irq_stat, scu_gnss_stat1;
	unsigned scu_tes0, scu_len0;
	struct device *dev = &p_gnss->p_dev->device;
	unsigned long flags;

	IMC_IDI_GNSS_ENTER;

	spin_lock_irqsave(&p_gnss->hw_lock, flags);

	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_GNSS_STAT1(gnss), &scu_gnss_stat1))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_GNSS_STAT1(gnss));

	if (SCU_GNSS_STAT1_GPS_BB_CLK_REQ(scu_gnss_stat1)){
		dev_dbg(dev, "bb_clk_state is on\n");
		p_gnss->bb_clk_state = true;
	}
	else {
		dev_dbg(dev, "bb_clk_state is off\n");
		p_gnss->bb_clk_state = false;
	}

	if (waitqueue_active(&bb_clk_wq)){
		dev_dbg(dev, "wakeup bb_clk_wq\n");
		wake_up_interruptible(&bb_clk_wq);
	}

	/* clear any pending irq */

	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_C5_IRQSS(gnss), &irq_stat))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_C5_IRQSS(gnss));

	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_C5_IRQSC(gnss), irq_stat))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C5_IRQSC(gnss));

	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_C5_IRQSC(gnss), 0))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C5_IRQSC(gnss));


	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_C6_IRQSS(gnss), &irq_stat))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_C6_IRQSS(gnss));

	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_C6_IRQSC(gnss), irq_stat))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C6_IRQSC(gnss));

	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_C6_IRQSC(gnss), 0))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C6_IRQSC(gnss));


	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_C7_IRQSS(gnss), &irq_stat))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_C7_IRQSS(gnss));

	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_C7_IRQSC(gnss), irq_stat))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C7_IRQSC(gnss));

	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_C7_IRQSC(gnss), 0))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C7_IRQSC(gnss));

#define NO_IRQ_TRIGGERED		0x0
#define IRQ_TRIGGERED_ON_RISING_EDGE	0x1
#define IRQ_TRIGGERED_ON_FALLING_EDGE	0x2
#define IRQ_TRIGGERED_ON_BOTH_EDGES	0x3
	/* set SCU_C5_IRQSM_GPS_EN_LDO_BB irq to be trigger on both edges */
	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_TES0(gnss), &scu_tes0))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_TES0(gnss));

	scu_tes0 &= ~SCU_TES0_GPS_EN_LDO_BB_MASK;
	scu_tes0 |=
		(IRQ_TRIGGERED_ON_BOTH_EDGES << SCU_TES0_GPS_EN_LDO_BB_OFFSET)|
		(IRQ_TRIGGERED_ON_BOTH_EDGES << SCU_TES0_GPS_BB_CLK_REQ_OFFSET);

	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_TES0(gnss), scu_tes0))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_TES0(gnss));
	/* disable Level sensitivity for SCU_LEN0_GPS_EN_LDO_BB (i.e. 0x00) */
	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_LEN0(gnss), &scu_len0))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_LEN0(gnss));

	scu_len0 &= ~(SCU_LEN0_GPS_EN_LDO_BB_MASK | SCU_LEN0_GPS_BB_CLK_REQ_MASK);

	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_LEN0(gnss), scu_len0))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_LEN0(gnss));

	/* enable relevant irqs */
	irq_mask = SCU_C5_IRQSM_GPS_EN_LDO_BB_ENABLED
		| SCU_C5_IRQSM_GPS_TO_ARM_0_ENABLED
		| SCU_C5_IRQSM_GNSS_GPIO_WAKE_ENABLED;
	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_C5_IRQSM(gnss), irq_mask))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C5_IRQSM(gnss));

	irq_mask = SCU_C6_IRQSM_WDG_EVENT_RST_ENABLED;
	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_C6_IRQSM(gnss), irq_mask))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C6_IRQSM(gnss));

	irq_mask = SCU_C7_IRQSM_GPS_DBG_ENABLED
		| SCU_C7_IRQSM_GPS_TCXO_CLK_REQ_ENABLED
		| SCU_C7_IRQSM_GPS_RF_CLK_REQ_ENABLED
		| SCU_C7_IRQSM_GPS_BB_CLK_REQ_ENABLED
		| SCU_C7_IRQSM_GPS_TO_ARM_1_ENABLED;
	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_C7_IRQSM(gnss), irq_mask))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C7_IRQSM(gnss));

	/* enable irqs at top level */
	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_IMSC(scu), &irq_mask))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_IMSC(scu));

	irq_mask |= (BIT(5) | BIT(6) | BIT(7));
	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned) SCU_IMSC(scu), irq_mask))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_IMSC(scu));

	/* if h/w ready (LDO==1) and LDO_BB irq haven't been raised and handled
	 * start h/w here. don't wait for irq */
	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_GNSS_STAT1(gnss), &scu_gnss_stat1))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_GNSS_STAT1(gnss));
	spin_unlock_irqrestore(&p_gnss->hw_lock, flags);
	if ((p_gnss->gps_state != GPS_POWER_ON) &&
			SCU_GNSS_STAT1_GPS_EN_LDO_BB(scu_gnss_stat1)) {
		dev_dbg(dev, "EN_LDO_BB already ON. goto idi_gnss_start_hw\n");
		idi_gnss_start_hw(p_gnss);
	} else {
		dev_dbg(dev, "EN_LDO_BB is OFF .. waiting for IRQ\n");
	}

	IMC_IDI_GNSS_EXIT;

	return 0;
}

/*****************************************************************************
 *	name:		gnss_deinit_hw
 *	description:	disable GNSS IRQ and put core in RESET
 *	in params:	struct idi_gnss_port *p_gnss
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void gnss_deinit_hw(struct idi_gnss_port *p_gnss)
{
	unsigned irq_mask = 0;
	unsigned long flags;
	phys_addr_t scu = p_gnss->scu_io;
	phys_addr_t gnss = p_gnss->gnss_io;
	struct device *dev = &p_gnss->p_dev->device;

	spin_lock_irqsave(&p_gnss->hw_lock, flags);

	IMC_IDI_GNSS_ENTER;

	/* disable irqs at top level */
	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_IMSC(scu), &irq_mask))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_IMSC(scu));

	irq_mask &= ~((BIT(5) | BIT(6) | BIT(7)));

	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned) SCU_IMSC(scu), irq_mask))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_IMSC(scu));

	/* disable all GNSS irqs */
	irq_mask = 0;
	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned) SCU_C5_IRQSM(gnss), irq_mask))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C5_IRQSM(gnss));
	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned) SCU_C6_IRQSM(gnss), irq_mask))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C6_IRQSM(gnss));
	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned) SCU_C7_IRQSM(gnss), irq_mask))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_C7_IRQSM(gnss));

	spin_unlock_irqrestore(&p_gnss->hw_lock, flags);

	/* put GNSS core in RESET state */
	idi_gnss_stop_hw(p_gnss);

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		gnss_idi_start_tx
 *	description:	write the transaction size to USIF TPS register to start
 *			the DMA process. This function will be invoked as a
 *			callback function from IDI (see gnss_prepare_tx)
 *	in params:	struct idi_transaction *trans
 *	out params:	none
 *	return val:	none
 *****************************************************************************/
static void gnss_idi_start_tx(struct idi_transaction *trans)
{
	struct idi_peripheral_device *p_device = trans->peripheral;
	struct device *dev = &p_device->device;
	struct uart_usif_xgold_port *uxp = dev_get_drvdata(dev);
	struct idi_xfer *xfer = &trans->idi_xfer;

	IMC_IDI_GNSS_ENTER;

	dev_dbg(dev, "Starting idi TX xfer of %d bytes\n", xfer->size);
	uxp->write_tps(&uxp->port, xfer->size);

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		gnss_idi_tx
 *	description:	start IDI tx transaction
 *	in params:	struct uart_port *port
 *	out params:	none
 *	return val:	none
 *****************************************************************************/
static int gnss_idi_tx(struct uart_port *port)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);
	struct idi_peripheral_device *p_device = p_gnss->p_dev;
	struct device *dev = &p_device->device;
	struct tty_struct *tty = port->state->port.tty;
	struct idi_transaction *trans;
	int ret;

	IMC_IDI_GNSS_ENTER;

	if (tty->hw_stopped) {
		dev_err(dev, "%s: tty hw_stopped ... exiting\n", __func__);
		IMC_IDI_GNSS_EXIT;
		return 0;
	}

	trans = idi_alloc_transaction(GFP_ATOMIC);
	if (!trans) {
		dev_err(dev, "idi_alloc_transaction FAILED\n");
		IMC_IDI_GNSS_EXIT;
		return -ENOMEM;
	}

	trans->idi_xfer.channel_opts = IDI_PRIMARY_CHANNEL;
	trans->complete = gnss_tx_transaction_complete;
	ret = idi_tty_prepare_tx_xfer(p_device, trans, port,
				(unsigned *) p_gnss->tx_bounce,
				TX_BUFFER_SIZE, MAX_GNSS_TX_SIZE);
	if (ret == 0) {
		dev_dbg(dev, "idi_tty_prepare_tx_xfer return 0.");
		dev_dbg(dev, "freeing TX transaction 0x%x\n", (unsigned)trans);
		idi_free_transaction(trans);

		IMC_IDI_GNSS_EXIT;
		return 0;
	}

	dev_dbg(dev, "Submitting TX transaction %x of size 0x%x\n", (unsigned)trans,
			trans->idi_xfer.size);
	if (idi_async_write(p_device, trans)) {
		dev_err(dev, "idi_async_write returns with error!\n");
		idi_free_transaction(trans);
		IMC_IDI_GNSS_EXIT;
		return -EINVAL;
	}

	atomic_inc(&p_gnss->tx_trans_in_process);

	IMC_IDI_GNSS_EXIT;
	return trans->idi_xfer.size;
}

/*****************************************************************************
 *	name:		gnss_tx_transaction_complete
 *	description:	idi bus callback that indicates the TX transaction was
 *			completed
 *	in params:	struct idi_transaction *trans
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void gnss_tx_transaction_complete(struct idi_transaction *trans)
{
	struct idi_peripheral_device *p_device = trans->peripheral;
	struct device *dev = &p_device->device;
	struct uart_usif_xgold_port *uxp = dev_get_drvdata(dev);
	struct uart_port *port = &uxp->port;
	struct idi_gnss_port *p_gnss = to_gnss_port(port);
	struct circ_buf *xmit = &port->state->xmit;
	unsigned long flags;
	int ret;

	IMC_IDI_GNSS_ENTER;

	dev_dbg(dev, "TX trans %x of size %x completed, status %d\n", (unsigned)trans,
		trans->idi_xfer.size, trans->status);

	idi_tty_complete_tx_xfer(p_device, trans, port);

	atomic_dec(&p_gnss->tx_trans_in_process);

	spin_lock_irqsave(&port->lock, flags);

	if(uart_circ_empty(xmit)) {
		idi_gnss_stop_tx(port);
		if (waitqueue_active(&ioctl_wq)){
			dev_dbg(dev, "uart buf empty, wakeup ioctl_wq\n");
			wake_up_interruptible(&ioctl_wq);
		}
		goto free_tx_trans;
	}

	if (atomic_read(&p_gnss->tx_trans_in_process) > 0) {
		dev_warn(dev, "%s: tx in process [%d]... exiting\n",__func__,
			atomic_read(&p_gnss->tx_trans_in_process));
		idi_gnss_stop_tx(port);
		goto free_tx_trans;
	}

	if((p_gnss->gps_state != GPS_POWER_ON) ||
		(p_gnss->bb_clk_state != true)) {
		dev_warn(dev, "%s: GNSS BB is sleeping\n", __func__);
		wakeup_gnss(p_gnss);
		idi_gnss_stop_tx(port);
		goto free_tx_trans;
	}

	if (trans->status != IDI_STATUS_FLUSH) {
		ret = gnss_idi_tx(port);
		if ((ret > 0) && (p_gnss->uxp_ops->start_tx))
			p_gnss->uxp_ops->start_tx(port);
		else
			idi_gnss_stop_tx(port);
	}

free_tx_trans:
	idi_free_transaction(trans);

	if (atomic_read(&p_gnss->tx_trans_in_process) == 0)
		gnss_pm_sem_up(p_gnss, __func__);

	spin_unlock_irqrestore(&port->lock, flags);

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		gnss_rx_transaction_complete
 *	description:	idi bus callback to indicate when a rx transaction was
 *			completed
 *	in params:	struct idi_transaction *trans
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void gnss_rx_transaction_complete(struct idi_transaction *trans)
{
	struct idi_peripheral_device *p_device = trans->peripheral;
	struct device *dev = &trans->peripheral->device;
	struct uart_usif_xgold_port *uxp = dev_get_drvdata(dev);
	struct uart_port *port = &uxp->port;
	struct idi_gnss_port *p_gnss = to_gnss_port(port);
	struct tty_struct *tty = port->state->port.tty;
	phys_addr_t gnss = p_gnss->gnss_io;
	unsigned scu_gnss_ctl1;
	unsigned long flags;
	bool release_pm_sem = false;
	int ret;

	IMC_IDI_GNSS_ENTER;

	dev_dbg(dev, "RX trans %x of size 0x%x completed\n", (unsigned)trans,
			trans->idi_xfer.size);

	/* push the rx data to the tty subsystem */
	ret = idi_push_xfer_to_tty(p_device, &trans->idi_xfer, tty);
	if (ret > 0)
		port->icount.rx += ret;

	idi_free_transaction(trans);

	spin_lock_irqsave(&p_gnss->hw_lock, flags);
	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_GNSS_CTL1(gnss), &scu_gnss_ctl1))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_GNSS_CTL1(gnss));
	if (scu_gnss_ctl1 & SCU_GNSS_CTL1_ARM_TO_GPS_0_MASK) {
		/* remove the ACK bit so the f/w could send the next msg */
		scu_gnss_ctl1 &= ~SCU_GNSS_CTL1_ARM_TO_GPS_0_MASK;
		if (idi_client_iowrite(p_gnss->p_dev,
				(unsigned)SCU_GNSS_CTL1(gnss), scu_gnss_ctl1))
			dev_err(dev, "%s: idi write error %x\n", __func__, SCU_GNSS_CTL1(gnss));
		dev_dbg(dev, "%s: Setting GNSS_CTL1 to 0x%08X\n", __func__, scu_gnss_ctl1);
		release_pm_sem = true;
	}
	spin_unlock_irqrestore(&p_gnss->hw_lock, flags);

	atomic_dec(&p_gnss->rx_trans_in_process);

	/* If no more rx transaction release sem */
	if (release_pm_sem && (atomic_read(&p_gnss->rx_trans_in_process) == 0))
		gnss_pm_sem_up(p_gnss, __func__);

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		gnss_eop_rx
 *	description:	IDI callback that will be invoked when the USIF will
 *			signal that it has rx msgs to send to IDI
 *	in params:	void *data
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void gnss_eop_rx(void *data)
{
	struct idi_gnss_port *p_gnss = data;
	struct device *dev = &p_gnss->p_dev->device;
	struct idi_transaction *trans;
	struct idi_xfer *xfer;
	unsigned payload = 0;
	struct uart_usif_xgold_port *uxp = dev_get_drvdata(dev);

	IMC_IDI_GNSS_ENTER;

	atomic_inc(&p_gnss->rx_trans_in_process);

	if ((uxp == NULL) || (uxp->read_rps == NULL))
		goto gnss_eop_rx_error;

	payload = uxp->read_rps(&uxp->port);
	trans = idi_alloc_transaction(GFP_ATOMIC);
	if (!trans) {
		dev_err(dev, "idi_alloc_transaction failed\n");
		goto gnss_eop_rx_error;
	}

	xfer = &trans->idi_xfer;
	xfer->size = payload;
	xfer->channel_opts = IDI_PRIMARY_CHANNEL;
	trans->complete = gnss_rx_transaction_complete;

	dev_dbg(dev, "Submitting RX trans %x, %d bytes, complete %x to q\n",
				(unsigned)trans, payload, (unsigned)trans->complete);
	if (idi_async_read(p_gnss->p_dev, trans)) {
		dev_err(dev, "Error while submitting IDI read action !\n");
		goto gnss_eop_rx_error;
	}

	IMC_IDI_GNSS_EXIT;

	return;

gnss_eop_rx_error:
	atomic_dec(&p_gnss->rx_trans_in_process);

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		gnss_prepare_rx
 *	description:	prepare the IDI bus for GNSS rx operation
 *	in params:	struct idi_gnss_port *p_gnss
 *	out params:	none
 *	return val:	int ret. 0 - for success
 ****************************************************************************/
static int gnss_prepare_rx(struct idi_gnss_port *p_gnss)
{
	struct device *dev = &p_gnss->p_dev->device;
	struct idi_channel_config *rx_conf = &p_gnss->rx_ch_config;
	struct idi_peripheral_device *p_device = p_gnss->p_dev;
	struct idi_resource *idi_res = &p_device->resources;
	struct xgold_usif_platdata *pdata = dev_get_platdata(dev);
	struct resource *res;
	unsigned ret = 0;

	IMC_IDI_GNSS_ENTER;

	if (pdata == NULL) {
		dev_err(dev, "no xgold_usif_platdata\n");
		return -EINVAL;
	}

	res = idi_get_resource_byname(idi_res,
			IORESOURCE_MEM,
			GNSS_RX_RES_NAME);
	if (res == NULL) {
		dev_err(dev, "failed to get res name %s\n", GNSS_RX_RES_NAME);
		return -EINVAL;
	}

	rx_conf->tx_or_rx = 0;
	rx_conf->priority = IDI_NORMAL_PRIORITY;
	rx_conf->channel_opts = IDI_PRIMARY_CHANNEL;
	rx_conf->size = pdata->rx_buffer_size;
	rx_conf->end_of_packet = gnss_eop_rx;
	rx_conf->private_data = p_gnss;

	if (!rx_conf->cpu_base)
		rx_conf->cpu_base = kmalloc(rx_conf->size, GFP_KERNEL | GFP_DMA);


	if (!rx_conf->cpu_base) {
		dev_err(dev, "Unable to allocate RX coherent buffer\n");
		return -ENOMEM;
	}
	rx_conf->base = dma_map_single(NULL, rx_conf->cpu_base,
					rx_conf->size, DMA_FROM_DEVICE);

	if (!rx_conf->base) {
		dev_err(dev, "Unable to DMA-map RX buffer \n");
		kfree(rx_conf->cpu_base);
		return -ENOMEM;
	}

	rx_conf->hw_fifo_size = resource_size(res);
	rx_conf->dst_addr = res->start;

	ret = idi_set_channel_config(p_device, rx_conf);
	if (ret) {
		dev_err(dev, "Unable to set IDI read channel configuration\n");
		goto fail_rx_config;
	}

	IMC_IDI_GNSS_EXIT;

	return 0;

fail_rx_config:
	dma_unmap_single(NULL, rx_conf->base, rx_conf->size, DMA_FROM_DEVICE);
	kfree(rx_conf->cpu_base);
	IMC_IDI_GNSS_EXIT;

	return ret;
}

/*****************************************************************************
 *	name:		gnss_prepare_tx
 *	description:	prepare the IDI bus for GNSS rx operation
 *	in params:	struct idi_gnss_port *p_gnss
 *	out params:	none
 *	return val:	int ret. 0 - for success
 ****************************************************************************/
static int gnss_prepare_tx(struct idi_gnss_port *p_gnss)
{
	struct device *dev = &p_gnss->p_dev->device;
	struct idi_channel_config *conf = &p_gnss->tx_ch_config;
	struct idi_peripheral_device *p_device = p_gnss->p_dev;
	struct idi_resource *idi_res = &p_device->resources;
	struct resource *res;
	struct idi_transaction *trans;
	struct idi_xfer *xfer;
	int ret = 0;

	IMC_IDI_GNSS_ENTER;

	res = idi_get_resource_byname(idi_res,
			IORESOURCE_MEM,
			GNSS_TX_RES_NAME);
	if (!res) {
		dev_err(dev, "failed to get res %s\n", GNSS_TX_RES_NAME);
		return -EINVAL;
	}

	trans = idi_alloc_transaction(GFP_KERNEL);
	if (!trans) {
		dev_err(dev, "failed to alloc tx trans");
		return -ENOMEM;
	}

	conf->tx_or_rx = 1;
	conf->priority = IDI_NORMAL_PRIORITY;
	conf->channel_opts = IDI_PRIMARY_CHANNEL | IDI_TX_CHANNEL;
	conf->size = 0;
	conf->cpu_base = NULL;
	conf->base = 0;
	conf->dst_addr = res->start;
	conf->hw_fifo_size = resource_size(res);
	conf->start_tx = gnss_idi_start_tx;

	xfer = &trans->idi_xfer;
	xfer->base = conf->base;
	xfer->cpu_base = 0;
	xfer->size = 0;
	xfer->dst_addr = conf->dst_addr;
	xfer->channel_opts = IDI_PRIMARY_CHANNEL;

	trans->complete = gnss_tx_transaction_complete;

	ret = idi_set_channel_config(p_device, conf);

	IMC_IDI_GNSS_EXIT;

	return ret;
}

#ifdef CONFIG_HAS_WAKELOCK
/*****************************************************************************
 *	name:		gnss_wake_lock_set
 *	description:	wake_lock wrapper incase it supported by the kernel
 *	in params:	struct idi_peripheral_device *p_device
 *			bool lock
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void gnss_wake_lock_set(struct idi_peripheral_device *p_device,
		bool lock)
{
	struct device *dev = &p_device->device;
	struct idi_gnss_port *p_gnss = idi_dev_to_gnss_port(p_device);

	if (lock) {
		dev_dbg(dev, "check if gnss_wake_lock is locked...\n");
		if (!wake_lock_active(&p_gnss->gnss_wake_lock)) {
			dev_dbg(dev, "not locked .. locking it\n");
			wake_lock(&p_gnss->gnss_wake_lock);
		} else {
			dev_dbg(dev, "already locked\n");
		}
	} else {
		dev_dbg(dev, "checking if gnss_wake_lock is locked ...\n");
		if (wake_lock_active(&p_gnss->gnss_wake_lock)) {
			dev_dbg(dev, "locked .. un-locking it\n");
			wake_unlock(&p_gnss->gnss_wake_lock);
		} else {
			dev_dbg(dev, "already un-locked\n");
		}
	}
}
#endif

/*****************************************************************************
 *	name:		gnss_set_idi_power_state
 *	description:	Per GNSS core state we modify the IDI bus power state.
 *	in params:	struct idi_peripheral_device *p_dev
 *			gnss_gps_state state
 *	out params:	none
 *	return val:	int ret (0 for success)
 ****************************************************************************/
static struct gnss_power_setting {
	bool idi_req;
	bool wake_lock_req;
	struct pinctrl_state *tcxo_pin_req;
	bool sp_pur_active;
	bool es2_tcxo_dig;
	bool gnss_hp_sw_req;
} gnss_power_setting_array[] = {
	/*D0 D0I0 D0I1 D0I2*/
	{
		.idi_req = true,
		.wake_lock_req = true,
		.tcxo_pin_req = NULL,
		.sp_pur_active = true,
		.es2_tcxo_dig = true,
		.gnss_hp_sw_req = true,
	},
	/*D0I3*/
	{
		.idi_req = false,
		.wake_lock_req = true,
		.tcxo_pin_req = NULL,
		.sp_pur_active = false,
		.es2_tcxo_dig = true,
		.gnss_hp_sw_req = true,
	},
	/*D3*/
	{
		.idi_req = false,
		.wake_lock_req = false,
		.tcxo_pin_req = NULL,
		.sp_pur_active = false,
		.es2_tcxo_dig = false,
		.gnss_hp_sw_req = false,
	},
};

static int gnss_set_idi_power_state(struct idi_peripheral_device *p_device,
		int state)
{
	int ret = 0;
	unsigned long flags;
	struct device *dev = &p_device->device;
	struct idi_gnss_port *p_gnss = idi_dev_to_gnss_port(p_device);
	struct gnss_power_setting *p_setting = NULL;
	unsigned scu_gnss_ctl1;
	phys_addr_t gnss = p_gnss->gnss_io;
	struct idi_channel_config *rx_conf = &p_gnss->rx_ch_config;
	struct idi_channel_config *tx_conf = &p_gnss->tx_ch_config;
	static bool reconfig_idi_channle = false;

	IMC_IDI_GNSS_ENTER;

	dev_dbg(dev, "Setting IDI power to %d\n", state);

	/* TODO: make sure there are no transactions */
	if (atomic_read(&p_gnss->tx_trans_in_process) > 0)
		dev_warn(dev, "Changing PM state while tx trans in process\n");

	/* Change the IDI power state if not equal to current state */
	if (p_gnss->idi_power_state == state) {
		dev_dbg(dev, "IDI bus is already in power state %d\n", state);
		goto idi_set_power_success;
	}

	switch (state) {
	case D0:
	case D0I0:
	case D0I1:
	case D0I2:
		if (p_gnss->idi_channel_initialized && reconfig_idi_channle) {
			dev_dbg(dev, "required to reconfigure idi channle\n");
			ret = idi_set_channel_config(p_device, rx_conf);
			if (ret) {
				dev_err(dev, "re-config idi rx failed\n");
				goto idi_set_power_failed;
			}
			ret = idi_set_channel_config(p_device, tx_conf);
			if (ret) {
				dev_err(dev, "re-config idi tx failed\n");
				goto idi_set_power_failed;
			}
			reconfig_idi_channle = false;
		}
		p_setting = &gnss_power_setting_array[0];
		break;
	case D0I3:
		p_setting = &gnss_power_setting_array[1];
		break;
	case D3:
		reconfig_idi_channle = true;
		p_setting = &gnss_power_setting_array[2];
		break;
	}

	/* set proper volt request (1.0/1.2) */
	spin_lock_irqsave(&p_gnss->hw_lock, flags);

	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_GNSS_CTL1(gnss), &scu_gnss_ctl1))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_GNSS_CTL1(gnss));
	if (p_setting->gnss_hp_sw_req)
		scu_gnss_ctl1 |= SCU_GNSS_CTL1_GNSS_HP_SW_REQ_V_HIGH;
	else
		scu_gnss_ctl1 &= ~(SCU_GNSS_CTL1_GNSS_HP_SW_REQ_V_HIGH);
	if (idi_client_iowrite(p_gnss->p_dev,
		(unsigned)SCU_GNSS_CTL1(gnss), scu_gnss_ctl1))
		dev_err(dev, "%s: idi write error %x\n", __func__, SCU_GNSS_CTL1(gnss));

	dev_dbg(dev, "%s: Setting GNSS_CTL1 to 0x%08X\n", __func__,
			scu_gnss_ctl1);
	spin_unlock_irqrestore(&p_gnss->hw_lock, flags);

	if ((p_gnss->scu_chip_id == CHIP_ID_ES2) || (p_gnss->scu_chip_id == CHIP_ID_ES2_1)) {
		gnss_set_sp_pur_bits(p_gnss, p_setting->sp_pur_active);
		gnss_es2_tcxo_setting(p_gnss, p_setting->es2_tcxo_dig);
	}

	ret = gnss_set_pinctrl_state(dev, p_setting->tcxo_pin_req);
	if (ret)
		goto idi_set_power_failed;

	ret = idi_set_power_state(p_device,
			(void *)gnss_pm_state_array[state].handler,
			p_setting->idi_req);
	if (ret)
			goto idi_set_power_failed;

	if ((state != D3) && (state != D0I3))
		p_gnss->freq = power_state_to_freq(state);

	gnss_wake_lock_set(p_device, p_setting->wake_lock_req);

idi_set_power_success:
	spin_lock_irqsave(&p_gnss->hw_lock, flags);
	p_gnss->idi_power_state = state;
	spin_unlock_irqrestore(&p_gnss->hw_lock, flags);

	IMC_IDI_GNSS_EXIT;
	return 0;

idi_set_power_failed:
	dev_err(dev, "failed to set power state %d\n", state);
	IMC_IDI_GNSS_EXIT;
	return ret;
}

/*****************************************************************************
 *	name:		idi_gnss_tx_empty
 *	description:
 *	in params:	struct uart_port *port
 *	out params:	none
 *	return val:
 ****************************************************************************/
static unsigned int idi_gnss_tx_empty(struct uart_port *port)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);
	unsigned int ret = 0;

	IMC_IDI_GNSS_ENTER;

	if (p_gnss->uxp_ops->tx_empty)
		ret = p_gnss->uxp_ops->tx_empty(port);

	IMC_IDI_GNSS_EXIT;

	return ret;
}

/*****************************************************************************
 *	name:		idi_gnss_set_mctrl
 *	description:
 *	in params:	struct uart_port *port
 *			unsigned int mctrl
 *	out params:	none
 *	return val:
 ****************************************************************************/
static void idi_gnss_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);

	IMC_IDI_GNSS_ENTER;

	if ((p_gnss->gps_state == GPS_POWER_ON) &&
			(p_gnss->uxp_ops->set_mctrl))
		p_gnss->uxp_ops->set_mctrl(port, mctrl);

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		idi_gnss_get_mctrl
 *	description:
 *	in params:	struct uart_port *port
 *	out params:	none
 *	return val:
 ****************************************************************************/
static unsigned int idi_gnss_get_mctrl(struct uart_port *port)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);
	unsigned int ret = 0;

	IMC_IDI_GNSS_ENTER;

	if (p_gnss->uxp_ops->get_mctrl)
		ret = p_gnss->uxp_ops->get_mctrl(port);

	IMC_IDI_GNSS_EXIT;

	return ret;
}

/*****************************************************************************
 *	name:		idi_gnss_stop_tx
 *	description:	config the USIF to stop TX operation
 *	in params:	struct uart_port *port
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void idi_gnss_stop_tx(struct uart_port *port)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);
	struct device *dev = &p_gnss->p_dev->device;
	struct tty_port *t_port = &port->state->port;
	struct tty_struct *tty = t_port->tty;

	IMC_IDI_GNSS_ENTER;

	/* TODO: check if this is necessary */
	if (atomic_read(&p_gnss->tx_trans_in_process) > 0)
		dev_warn(dev, "trying to stop TX while tx in process [%d]\n",
			atomic_read(&p_gnss->tx_trans_in_process));

	if (p_gnss->uxp_ops->stop_tx)
		 p_gnss->uxp_ops->stop_tx(port);

	if ((p_gnss->gps_state == GPS_DEEP_COMA) && (tty->hw_stopped)) {
		dev_dbg(dev, "GPS is in DEEP_COMA. set tty->hw_stopped to 0\n");
		tty->hw_stopped = 0;
	}

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		gnss_work_fn_tx
 *	description:	This function is the callback func for gnss_work_tx work
 *			struct. it do the actual work for idi_gnss_start_tx.
 *			we do it here since idi_gnss_start_tx is called from
 *			uninterrupable context.
 *	in params:	struct uart_port *port
 *	out params:	none
 *	return val:
 ****************************************************************************/
static void gnss_work_fn_tx(struct work_struct *work)
{
	struct idi_gnss_port *p_gnss =
		container_of(work, struct idi_gnss_port, gnss_work_tx);
	struct device *dev = &p_gnss->p_dev->device;
	struct uart_usif_xgold_port *uxp = dev_get_drvdata(dev);
	struct uart_port *port = &uxp->port;
	struct circ_buf *xmit = &port->state->xmit;
	struct idi_peripheral_device *p_device = p_gnss->p_dev;
	unsigned long flags;
	int ret;

	IMC_IDI_GNSS_ENTER;

	if (atomic_read(&p_gnss->tx_trans_in_process) > 0) {
		dev_warn(dev, "tx in process [%d]... exiting\n",
			atomic_read(&p_gnss->tx_trans_in_process));
		IMC_IDI_GNSS_EXIT;
		return;
	}

	gnss_pm_sem_down(p_gnss, __func__, false);

	spin_lock_irqsave(&p_gnss->hw_lock, flags);
	if (uart_circ_empty(xmit)){
		gnss_pm_sem_up(p_gnss, __func__);
		spin_unlock_irqrestore(&p_gnss->hw_lock, flags);
		dev_dbg(dev, "%s: no new tx trans to send\n", __func__);
		IMC_IDI_GNSS_EXIT;
		return;
	}

	// make sure BB is on
	if ((p_gnss->gps_state != GPS_POWER_ON) ||
		(p_gnss->bb_clk_state == false)){
		dev_dbg(dev, "GNSS is sleeping, wake it up and exit\n");
		wakeup_gnss(p_gnss);
		gnss_pm_sem_up(p_gnss, __func__);
		IMC_IDI_GNSS_EXIT;
		spin_unlock_irqrestore(&p_gnss->hw_lock, flags);
		return;
	}

	/* acivate IDI bus */
	if ((p_gnss->idi_power_state == D3) ||
		(p_gnss->idi_power_state == D0I3)) {
		/* Activate IDI bus */
		spin_unlock_irqrestore(&p_gnss->hw_lock, flags);
		gnss_set_idi_power_state(p_device,
			freq_to_power_state(p_gnss->freq));
		spin_lock_irqsave(&p_gnss->hw_lock, flags);
	}

	ret = gnss_idi_tx(port);
	if ((ret > 0) && (p_gnss->uxp_ops->start_tx))
		p_gnss->uxp_ops->start_tx(port);

	if (ret <= 0) {
		dev_dbg(dev, "%s: no new tx trans were sent\n", __func__);
		/* unlock PM */
		gnss_pm_sem_up(p_gnss, __func__);
	}

	spin_unlock_irqrestore(&p_gnss->hw_lock, flags);

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		idi_gnss_start_tx
 *	description:	triggers a TX operation
 *	in params:	struct uart_port *port
 *	out params:	none
 *	return val:
 ****************************************************************************/
static void idi_gnss_start_tx(struct uart_port *port)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);
	struct device *dev = &p_gnss->p_dev->device;

	IMC_IDI_GNSS_ENTER;

	if (!work_pending(&p_gnss->gnss_work_tx)) {
		dev_dbg(dev, "scheduling TX work\n");
		schedule_work(&p_gnss->gnss_work_tx);
	} else {
		dev_dbg(dev, "work is still pendindg\n");
	}

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		idi_gnss_stop_rx
 *	description:
 *	in params:	struct uart_port *port
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void idi_gnss_stop_rx(struct uart_port *port)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);

	IMC_IDI_GNSS_ENTER;

	if (p_gnss->uxp_ops->stop_rx)
		p_gnss->uxp_ops->stop_rx(port);

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		idi_gnss_enable_ms
 *	description:
 *	in params:	struct uart_port *port
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void idi_gnss_enable_ms(struct uart_port *port)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);

	IMC_IDI_GNSS_ENTER;

	if (p_gnss->uxp_ops->enable_ms)
		p_gnss->uxp_ops->enable_ms(port);

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		idi_gnss_startup
 *	description:
 *	in params:	struct uart_port *port
 *	out params:	none
 *	return val:
 ****************************************************************************/
static int idi_gnss_startup(struct uart_port *port)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);
	struct idi_peripheral_device *p_device = p_gnss->p_dev;
	struct device *dev = &p_gnss->p_dev->device;
	struct xgold_usif_platdata *uxp_platdata = dev_get_platdata(dev);
	struct imc_idi_gnss_platdata *platdata =
		xgold_port_platdata_priv(uxp_platdata);
	struct pinctrl_state *state = NULL;
	int ret = 0;

	IMC_IDI_GNSS_ENTER;

	state = platdata->ext_lna_pins_default;
	gnss_set_pinctrl_state(dev, state);

	state = platdata->fta_pins_default;
	gnss_set_pinctrl_state(dev, state);

	state = platdata->fta_pos_pins_default;
	gnss_set_pinctrl_state(dev, state);

	/* prepare IDI bus for GNSS RX operation */
	ret = gnss_prepare_rx(p_gnss);
	if (ret) {
		dev_err(dev, "Unable to setup RX operations\n");
		goto fail_prepare_rx;
	}

	/* prepare IDI bug for GNSS TX operation */
	ret = gnss_prepare_tx(p_gnss);
	if (ret) {
		dev_err(dev, "Unable to setup TX operations\n");
		goto fail_prepare_tx;
	}

	p_gnss->idi_channel_initialized = true;

	/* init the GNSS core h/w */
	ret = gnss_init_hw(p_gnss);
	if (ret) {
		dev_err(dev, "Unable to initialize GNSS hardware\n");
		goto fail_init_hw;
	}

	ret = imc_idi_request_irqs(p_device);
	if (ret) {
		dev_err(dev, "Unable to request GNSS IRQs\n");
		goto fail_request_irqs;
	}

	if (p_gnss->uxp_ops->pm)
		p_gnss->uxp_ops->pm(port, 0, 0);

	if (p_gnss->uxp_ops->startup) {
		ret = p_gnss->uxp_ops->startup(port);
		if (ret) {
			dev_err(dev, "Unable to initialize USIF hardware\n");
			goto fail_usif_startup;
		}
	}

	IMC_IDI_GNSS_EXIT;
	return ret;

fail_usif_startup:
fail_request_irqs:
fail_init_hw:
fail_prepare_tx:
fail_prepare_rx:

	IMC_IDI_GNSS_EXIT;
	return ret;
}

/*****************************************************************************
 *	name:		idi_gnss_shutdown
 *	description:
 *	in params:	struct uart_port *port
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void idi_gnss_shutdown(struct uart_port *port)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);
	struct idi_peripheral_device *p_device = p_gnss->p_dev;
	struct idi_channel_config *rx_conf = &p_gnss->rx_ch_config;
	struct device *dev = &p_gnss->p_dev->device;
	struct xgold_usif_platdata *uxp_platdata = dev_get_platdata(dev);
	struct imc_idi_gnss_platdata *platdata =
		xgold_port_platdata_priv(uxp_platdata);
	struct pinctrl_state *state = NULL;

	IMC_IDI_GNSS_ENTER;

	dev_dbg(dev, "wating for tx work to complete\n");
	cancel_work_sync(&p_gnss->gnss_work_tx);
	dev_dbg(dev, "tx work completed\n");

	while(atomic_read(&p_gnss->rx_trans_in_process) > 0) {
		dev_warn(dev, "shutting down while rx in progress\n");
		udelay(1000);
	}

	state = platdata->ext_lna_pins_inactive;
	gnss_set_pinctrl_state(dev, state);

	state = platdata->fta_pins_inactive;
	gnss_set_pinctrl_state(dev, state);

	state = platdata->fta_pos_pins_inactive;
	gnss_set_pinctrl_state(dev, state);

	/* shutdown USIF */
	if (p_gnss->uxp_ops->shutdown)
		p_gnss->uxp_ops->shutdown(port);

	/* stop GNSS h/w */
	gnss_deinit_hw(p_gnss);

	/* disable GNSS irqs */
	imc_idi_free_irqs(p_device);

	/* release any IDI resources if any taken */
	idi_peripheral_flush(p_device, p_gnss->rx_ch_config.channel_opts);
	idi_peripheral_flush(p_device, p_gnss->tx_ch_config.channel_opts);

	atomic_set(&p_gnss->tx_trans_in_process, 0);
	atomic_set(&p_gnss->rx_trans_in_process, 0);

	/* free RX DMA buffer */
	dma_unmap_single(NULL, rx_conf->base, rx_conf->size, DMA_FROM_DEVICE);
	kfree(rx_conf->cpu_base);
	rx_conf->cpu_base = NULL;

	p_gnss->idi_channel_initialized = false;

	/* lock PM */
	gnss_pm_sem_down(p_gnss, __func__, true);

	/* De-Activate the IDI bus */
	if (gnss_set_idi_power_state(p_device, D3))
		dev_err(dev, "Unable to setup IDI power state\n");

	/* unlock PM */
	gnss_pm_sem_up(p_gnss, __func__);

	gnss_wake_lock_set(p_device, false);

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		idi_gnss_set_termios
 *	description:
 *	in params:	struct uart_port *port
 *			struct ktermios *new
 *			struct ktermios *old
 *	out params:	none
 *	return val:
 ****************************************************************************/
static void idi_gnss_set_termios(struct uart_port *port, struct ktermios *new,
		struct ktermios *old)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);

	IMC_IDI_GNSS_ENTER;

	if (p_gnss->uxp_ops->set_termios)
		p_gnss->uxp_ops->set_termios(port, new, old);

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		idi_gnss_pm
 *	description:	this callback is invoked before any other callback from
 *			the TTY serial subsystem. this is way we set the power
 *			state to D0 here. if we call any uxp_ops while power
 *			state != D0 well get a crash.
 *	in params:	struct uart_port *port
 *			unsigned int state
 *			unsigned int oldstate
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void idi_gnss_pm(struct uart_port *port, unsigned int state,
		unsigned int oldstate)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);

	IMC_IDI_GNSS_ENTER;

	/* lock PM */
	gnss_pm_sem_down(p_gnss, __func__, false);

	if (state == 0)
		gnss_set_idi_power_state(p_gnss->p_dev,
				freq_to_power_state(p_gnss->freq));

	if ((p_gnss->gps_state == GPS_POWER_ON) && (p_gnss->uxp_ops->pm))
		p_gnss->uxp_ops->pm(port, state, oldstate);

	if (state == 3)
		gnss_set_idi_power_state(p_gnss->p_dev, D3);

	/* unlock PM */
	gnss_pm_sem_up(p_gnss, __func__);

	IMC_IDI_GNSS_EXIT;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
/****************************************************************************
 *	name:		idi_gnss_set_wake
 *	description:
 *	in params:	struct uart_port *port
 *			unsigned int state
 *	out params:	none
 *	return val:
 ****************************************************************************/
static int idi_gnss_set_wake(struct uart_port *port, unsigned int state)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);
	int ret = 0;

	IMC_IDI_GNSS_ENTER;

	if (p_gnss->uxp_ops->set_wake)
		ret = p_gnss->uxp_ops->set_wake(port, state);

	IMC_IDI_GNSS_EXIT;

	return ret;
}
#else
/****************************************************************************
 *	name:		idi_gnss_wake_peer
 *	description:
 *	in params:	struct uart_port *port
 *	out params:	none
 *	return val:
 ****************************************************************************/
static void idi_gnss_wake_peer(struct uart_port *port)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);

	IMC_IDI_GNSS_ENTER;

	if (p_gnss->uxp_ops->wake_peer)
		p_gnss->uxp_ops->wake_peer(port);

	IMC_IDI_GNSS_EXIT;
}
#endif
/****************************************************************************
 *	name:		idi_gnss_type
 *	description:	Return a string describing the type of the port
 *	in params:	struct uart_port *port
 *	out params:	none
 *	return val:	const char *
 ****************************************************************************/
static const char *idi_gnss_type(struct uart_port *port)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);

	IMC_IDI_GNSS_ENTER;

	if (p_gnss->uxp_ops->type)
		return p_gnss->uxp_ops->type(port);

	IMC_IDI_GNSS_EXIT;

	return "IDI_GNSS";
}

/****************************************************************************
 *	name:		idi_gnss_release_port
 *	description:	Release IO and memory resources used by the port.
 *			This includes iounmap if necessary.
 *	in params:	struct uart_port *port
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void idi_gnss_release_port(struct uart_port *port)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);

	IMC_IDI_GNSS_ENTER;

	if (p_gnss->uxp_ops->release_port)
		p_gnss->uxp_ops->release_port(port);

	IMC_IDI_GNSS_EXIT;
}

/****************************************************************************
 *	name:		idi_gnss_request_port
 *	description:
 *	in params:	struct uart_port *port
 *	out params:	none
 *	return val:
 ****************************************************************************/
static int idi_gnss_request_port(struct uart_port *port)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);
	int ret = 0;

	IMC_IDI_GNSS_ENTER;

	if (p_gnss->uxp_ops->request_port(port))
		ret = p_gnss->uxp_ops->request_port(port);

	IMC_IDI_GNSS_EXIT;

	return ret;
}

/****************************************************************************
 *	name:		idi_gnss_config_port
 *	description:
 *	in params:	struct uart_port *port
 *			int flags
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void idi_gnss_config_port(struct uart_port *port, int flags)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);

	IMC_IDI_GNSS_ENTER;

	if (p_gnss->uxp_ops->config_port)
		p_gnss->uxp_ops->config_port(port, flags);

	IMC_IDI_GNSS_EXIT;
}

/****************************************************************************
 *	name:		idi_gnss_verify_port
 *	description:
 *	in params:	struct uart_port *port
 *			struct serial_struct *ser
 *	out params:	none
 *	return val:
 ****************************************************************************/
static int idi_gnss_verify_port(struct uart_port *port,
		struct serial_struct *ser)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);
	int ret = 0;

	IMC_IDI_GNSS_ENTER;

	if (p_gnss->uxp_ops->verify_port)
		ret = p_gnss->uxp_ops->verify_port(port, ser);

	IMC_IDI_GNSS_EXIT;

	return ret;
}


/****************************************************************************
 *	name:		idi_gnss_set_freq
 *	description:	modify clock frequency. modify only when
 *			GPS_BB_CLK_REQ is down.
 *	in params:	unsigned int freq
 *			idi_gnss_port *p_gnss
 *	out params:
 *	return val:	0 for success, else error
 ****************************************************************************/
static int idi_gnss_set_freq(struct idi_gnss_port *p_gnss, unsigned int freq)
{
	struct device *dev = &p_gnss->p_dev->device;
	unsigned int new_state = D0;
	int ret;

	IMC_IDI_GNSS_ENTER;

	dev_dbg(dev, "Setting freq to %d\n", freq);

	ret = wait_event_interruptible_timeout(bb_clk_wq, (p_gnss->bb_clk_state == false), HZ/2);
	if(ret <= 0) {
		if(ret == 0)
			dev_warn(dev, "timeout waiting for GPS_BB_CLK_REQ become 0\n");
		else
			dev_dbg(dev, "interrupt recv while waiting for GPS_BB_CLK_REQ become 0\n");
		return -EAGAIN;
	}

	new_state = freq_to_power_state(freq);
	if (new_state < 0) {
		dev_err(dev, "freq %d is not supported\n", freq);
		goto set_freq_err;
	}
	ret = gnss_set_idi_power_state(p_gnss->p_dev, new_state);

set_freq_err:
	IMC_IDI_GNSS_EXIT;

	return ret;
}

/*****************************************************************************
 *	name:		idi_gnss_ioctl
 *	description:
 *	in params:	struct uart_port *port
 *			unsigned int cmd, unsigned
 *			long arg
 *	out params:
 *	return val:
 *****************************************************************************/
static int idi_gnss_ioctl(struct uart_port *port, unsigned int cmd,
		unsigned long arg)
{
	struct idi_gnss_port *p_gnss = to_gnss_port(port);
	struct device *dev = &p_gnss->p_dev->device;
	int ret = -ENOIOCTLCMD;
	struct circ_buf *xmit = &port->state->xmit;

	IMC_IDI_GNSS_ENTER;

	switch (cmd) {
	case GNSS_IOC_GET_CHIPID:
		dev_dbg(dev, "GNSS_IOC_GET_CHIPID\n");
		ret = copy_to_user((unsigned int *)arg,
				&p_gnss->scu_chip_id,
				sizeof(unsigned int));
		break;

	case GNSS_IOC_GET_PWR_STATE:
		dev_dbg(dev, "GNSS_IOC_GET_PWR_STATE\n");
		ret = copy_to_user((unsigned int *)arg,
				&p_gnss->idi_power_state,
				sizeof(unsigned int));
		break;

	case GNSS_IOC_SET_PWR_STATE:
		dev_dbg(dev, "GNSS_IOC_SET_PWR_STATE. arg = 0x%x\n",
				(unsigned int)arg);
		ret = wait_event_interruptible_timeout(ioctl_wq, uart_circ_empty(xmit), HZ/2);
		if(ret <= 0) {
			if(ret == 0)
				dev_warn(dev, "%s: timeout waiting for uart buf_empty\n", __func__);
			else
				dev_dbg(dev, "interrupt recv while waiting for uart buf_empty\n");
			return -EAGAIN;
		}

		ret = gnss_pm_sem_down(p_gnss, __func__, true);
		if (ret < 0)
			return ret;
		ret = gnss_set_idi_power_state(p_gnss->p_dev, arg);
		gnss_pm_sem_up(p_gnss, __func__);
		break;

	case GNSS_IOC_GET_FRQ:
		dev_dbg(dev, "GNSS_IOC_GET_FRQ\n");
		ret = copy_to_user((unsigned int *)arg, &p_gnss->freq,
				sizeof(unsigned int));
		break;

	case GNSS_IOC_SET_FRQ:
		dev_dbg(dev, "GNSS_IOC_SET_FRQ. arg = 0x%x\n",
				(unsigned int)arg);
		ret = gnss_pm_sem_down(p_gnss, __func__, true);
		if (ret < 0) {
			gnss_pm_sem_up(p_gnss, __func__);
			return ret;
		}
		ret = idi_gnss_set_freq(p_gnss, (unsigned int)arg);
		gnss_pm_sem_up(p_gnss, __func__);
		break;
#ifdef GNSS_DCLK_FTR
	case GNSS_IOC_GET_DCLCK:
		dev_dbg(dev, "GNSS_IOC_GET_DCLCK\n");
		ret = copy_to_user((unsigned int *)arg,
				&p_gnss->cgu_gps_phx_control,
				sizeof(unsigned int));
		break;

	case GNSS_IOC_SET_DCLCK:
		dev_dbg(dev, "GNSS_IOC_SET_DCLCK. arg = 0x%x\n",
				(unsigned int)arg);
		p_gnss->cgu_gps_phx_control = arg;
		if (idi_client_iowrite(p_gnss->p_dev,
			(unsigned)ABB_CGU_GPS_PHX_CONTROL, p_gnss->cgu_gps_phx_control))
			dev_err(dev, "%s: idi write error %x\n", __func__, ABB_CGU_GPS_PHX_CONTROL);

		ret = 0;
		break;
#endif /* GNSS_DCLK_FTR */

	case GNSS_IOC_WAIT_FOR_BB_RELEASE:
		dev_dbg(dev, "GNSS_IOC_WAIT_FOR_BB_RELEASE\n");
		ret = wait_event_interruptible_timeout(bb_clk_wq, (p_gnss->bb_clk_state == false), HZ/2);
		if(ret <= 0) {
			if(ret == 0)
				dev_warn(dev, "timeout waiting for GPS_BB_CLK_REQ become 0\n");
			else
				dev_dbg(dev, "interrupt recv while waiting for GPS_BB_CLK_REQ become 0\n");
			return -EAGAIN;
		}
		ret = 0;
		break;

	default:
		dev_dbg(dev, "GNSS_IOC_DEFAULT\n");
		if (p_gnss->uxp_ops->ioctl)
			ret = p_gnss->uxp_ops->ioctl(port, cmd, arg);
		break;
	}

	IMC_IDI_GNSS_EXIT;

	return ret;
}

static struct uart_ops idi_gnss_ops = {
	.tx_empty = idi_gnss_tx_empty,
	.set_mctrl = idi_gnss_set_mctrl,
	.get_mctrl = idi_gnss_get_mctrl,
	.stop_tx = idi_gnss_stop_tx,
	.start_tx = idi_gnss_start_tx,
	.stop_rx = idi_gnss_stop_rx,
	.enable_ms = idi_gnss_enable_ms,
	.startup = idi_gnss_startup,
	.shutdown = idi_gnss_shutdown,
	.set_termios = idi_gnss_set_termios,
	.type = idi_gnss_type,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
	.set_wake = idi_gnss_set_wake,
#else
	.wake_peer = idi_gnss_wake_peer,
#endif
	.ioctl = idi_gnss_ioctl,
	.release_port = idi_gnss_release_port,
	.request_port = idi_gnss_request_port,
	.config_port = idi_gnss_config_port,
	.verify_port = idi_gnss_verify_port,
	.pm = idi_gnss_pm,
};

static struct uart_driver gnss_reg = {
	.owner = THIS_MODULE,
	.driver_name = SERIAL_GNSS_NAME,
	.dev_name = SERIAL_GNSS_NAME,
	.major = SERIAL_GNSS_MAJOR,
	.minor = SERIAL_GNSS_MINOR,
	.nr = 1,
};

#ifdef ENABLE_LB
/*****************************************************************************
 *	name:		enable_lauterbuch
 *	description:	Workaround for XMM6321: enable Lauterbuch
 *	in params:	none
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void enable_lauterbuch(void)
{
	/* ABB_CguLFME_AP_Register */
	if (idi_client_iowrite(p_gnss->p_dev, (unsigned)0xE6401170, 0x0)
		dev_err(dev, "%s: idi write error %x\n", __func__, 0xE6401170);

	/* ABB_CguLFME_CP_Register */
	if (idi_client_iowrite(p_gnss->p_dev, (unsigned)0xE6401174, 0x0)
		dev_err(dev, "%s: idi write error %x\n", __func__, 0xE6401174);

	/* ABB_CGU_Debug_Trace_Clock_Control_Register */
	if (idi_client_iowrite(p_gnss->p_dev, (unsigned)0xE640110C, 0x00030202)
		dev_err(dev, "%s: idi write error %x\n", __func__, 0xE640110C);
}
#endif /* ENABLE_LB */

void imc_idi_gnss_populate_state_array(struct idi_peripheral_device *p_device)
{
	struct device *dev = &p_device->device;
	int i;

	IMC_IDI_GNSS_ENTER;

	for (i = 0; i < ARRAY_SIZE(gnss_pm_state_array); i++) {
		if (gnss_pm_state_array[i].name) {
			gnss_pm_state_array[i].handler =
				idi_peripheral_device_pm_get_state_handler(
					p_device, gnss_pm_state_array[i].name);
			if (gnss_pm_state_array[i].handler)
				dev_dbg(dev, "registered PM handler for %s\n",
					gnss_pm_state_array[i].name);
			else
				dev_err(dev, "fail to find PM handler for %s\n",
					gnss_pm_state_array[i].name);
		}
	}

	IMC_IDI_GNSS_EXIT;
}

/*****************************************************************************
 *	name:		imc_idi_gnss_probe
 *	description:
 *	in params:
 *	out params:
 *	return val:
 ****************************************************************************/
static int imc_idi_gnss_probe(struct idi_peripheral_device *p_device,
				const struct idi_device_id *id)
{
	int ret = 0;
	struct device *dev = &p_device->device;
	struct uart_usif_xgold_port *uxp;
#ifdef CONFIG_OF
	struct imc_idi_gnss_platdata *gnss_platdata;
#endif
	struct idi_gnss_port *p_gnss;
	struct resource *res;
	struct idi_resource *idi_res = &p_device->resources;

	IMC_IDI_GNSS_ENTER;

#ifdef ENABLE_LB
	/* enable Lauterbuch */
	enable_lauterbuch();
#endif

	dev_dbg(&p_device->device, "IMC IDI GNSS driver probe\n");

	/* Allocate the usif port, and gnss port embedded in the usif port */
	uxp = xgold_usif_add_port(dev,
				&gnss_reg,
				sizeof(struct idi_gnss_port),
				sizeof(struct imc_idi_gnss_platdata));

	if (IS_ERR(uxp)) {
		dev_err(dev, "xgold_usif_add_port failed\n");
		IMC_IDI_GNSS_EXIT;
		return -EINVAL;
	}

	dev_set_drvdata(dev, uxp);

	/* configure our gnss port */
	p_gnss = xgold_port_priv(uxp);

	/* set the GPS state */
	p_gnss->gps_state = GPS_UNINITIALIZED;
		
	p_gnss->idi_channel_initialized = false;

	/* Set the current IDI power state */
	p_gnss->idi_power_state = D3;

	/* Set default freq */
	p_gnss->freq = FMR_FRQ_DEF_DCLK_832_PCLK_1248;

	p_gnss->p_dev = p_device;
	p_gnss->uxp_ops = uxp->port.ops;
	uxp->port.ops = &idi_gnss_ops;
	spin_lock_init(&p_gnss->hw_lock);
	INIT_WORK(&p_gnss->gnss_work_tx, gnss_work_fn_tx);
	sema_init(&p_gnss->pm_sem, 1);

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&p_gnss->gnss_wake_lock, WAKE_LOCK_SUSPEND,
			"gnss_wakelock");
#endif

#ifdef CONFIG_OF
	gnss_platdata = imc_idi_gnss_of_get_platdata(p_device);
	if (IS_ERR(gnss_platdata)) {
		ret = -EINVAL;
		dev_err(dev, "imc_idi_gnss_of_get_platdata failed\n");
		goto fail_get_platdata;
	}
#else
	if (!dev_get_platdata(dev)) {
		ret = -EINVAL;
		dev_err(dev, "dev_get_platdata failed\n");
		goto fail_get_platdata;
	}
#endif

#ifdef CONFIG_PLATFORM_DEVICE_PM
	ret = device_state_pm_set_class(&p_device->device,
			p_device->pm_platdata->pm_user_name);
	if (ret) {
		dev_err(dev, "GNSS device PM registration failed\n");
		goto fail_get_platdata;
	} else
		dev_dbg(dev, "GNSS device PM registration success\n");
#endif

	imc_idi_gnss_populate_state_array(p_device);

	res = idi_get_resource_byname(idi_res,
			IORESOURCE_MEM,
			GNSS_SCU_RES_NAME);
	if (!res) {
		dev_err(dev, "Getting resource %s failed !\n",
				GNSS_SCU_RES_NAME);
		ret = -EPERM;
		goto fail_request_scu;
	}
	p_gnss->scu_io = res->start;

	res = idi_get_resource_byname(idi_res,
			IORESOURCE_MEM,
			GNSS_GNSS_RES_NAME);
	if (!res) {
		dev_err(dev, "Get resource %s failed !\n", GNSS_GNSS_RES_NAME);
		ret = -EPERM;
		goto fail_request_gnss;
	}
	p_gnss->gnss_io = res->start;

	dev_dbg(&p_device->device, "IO remapping: gnss %x\n", p_gnss->gnss_io);

#ifdef CONFIG_PINCTRL_SINGLE
	gnss_power_setting_array[0].tcxo_pin_req = gnss_platdata->pins_default;
	gnss_power_setting_array[1].tcxo_pin_req = gnss_platdata->pins_default;
	gnss_power_setting_array[2].tcxo_pin_req = gnss_platdata->pins_sleep;
#endif
	/* put GNSS in RESET state. this is a must since soft reset (reboot)
	 * does NOT reset the GNSS BB. need to be done here */

	gnss_set_idi_power_state(p_device, D0);

	idi_gnss_stop_hw(p_gnss);

	/* TODO: replace with API */
	/* Get the current CHIP-ID */
	if (idi_client_ioread(p_gnss->p_dev,
		(unsigned)SCU_CHIP_ID(p_gnss->scu_io), &p_gnss->scu_chip_id))
		dev_err(dev, "%s: idi read error %x\n", __func__, SCU_CHIP_ID(p_gnss->scu_io));

	dev_dbg(dev, "Detected CHIP-ID 0x%04X\n", p_gnss->scu_chip_id);

	gnss_set_idi_power_state(p_device, D3);

	/* set the GPS_BB_CLK state */
	p_gnss->bb_clk_state = false;

#ifdef GNSS_DCLK_FTR
	/* Set the default CGU_GPS_PHX_CONTROL value */
	p_gnss->cgu_gps_phx_control = 0xC13;
#endif /* GNSS_DCLK_FTR */

	atomic_set(&p_gnss->tx_trans_in_process, 0);
	atomic_set(&p_gnss->rx_trans_in_process, 0);

	ret = uart_add_one_port(&gnss_reg, &uxp->port);
	if (!ret) {
		IMC_IDI_GNSS_EXIT;
		return 0;
	}

fail_request_gnss:

fail_request_scu:

fail_get_platdata:
	xgold_usif_remove_port(uxp);
	dev_set_drvdata(dev, NULL);

	IMC_IDI_GNSS_EXIT;

	return ret;
}

/*****************************************************************************
 *	name:		imc_idi_gnss_remove
 *	description:
 *	in params:
 *	out params:
 *	return val:
 ****************************************************************************/
static int imc_idi_gnss_remove(struct idi_peripheral_device *p_device)
{
	int ret = 0;
	struct device *dev = &p_device->device;
	struct uart_usif_xgold_port *uxp = dev_get_drvdata(dev);
	struct idi_gnss_port *p_gnss = xgold_port_priv(uxp);

	IMC_IDI_GNSS_ENTER;

	ret = uart_remove_one_port(&gnss_reg, &uxp->port);
	if (ret)
		dev_err(dev, "uart_remove_one_port failed with %d\n",
				ret);

	ret = device_state_pm_remove_device(&p_device->device);
	if (ret)
		dev_err(dev, "device_state_pm_remove_device failed with %d\n",
				ret);

	xgold_usif_remove_port(uxp);

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&p_gnss->gnss_wake_lock);
#endif
	dev_set_drvdata(dev, NULL);


	kfree(p_gnss);

	IMC_IDI_GNSS_EXIT;

	return ret;
}

#ifdef CONFIG_PM
/*****************************************************************************
 *	name:		imc_idi_gnss_suspend
 *	description:
 *	in params:
 *	out params:
 *	return val:
 ****************************************************************************/
static int imc_idi_gnss_suspend(struct idi_peripheral_device *p_device,
				pm_message_t mesg)
{
	struct device *dev = &p_device->device;
	struct uart_usif_xgold_port *uxp = dev_get_drvdata(dev);

	IMC_IDI_GNSS_ENTER;

	if (uxp->pm_ops && uxp->pm_ops->suspend)
		return uxp->pm_ops->suspend(dev);

	IMC_IDI_GNSS_EXIT;

	return 0;
}

/*****************************************************************************
 *	name:		imc_idi_gnss_resume
 *	description:
 *	in params:
 *	out params:
 *	return val:
 *****************************************************************************/
static int imc_idi_gnss_resume(struct idi_peripheral_device *p_device)
{
	struct device *dev = &p_device->device;
	struct uart_usif_xgold_port *uxp = dev_get_drvdata(dev);

	IMC_IDI_GNSS_ENTER;

	if (uxp->pm_ops && uxp->pm_ops->resume)
		return uxp->pm_ops->resume(dev);

	IMC_IDI_GNSS_EXIT;

	return 0;
}
#endif

static int imc_idi_gnss_dev_pm_suspend(struct device *dev)
{
	pr_debug("GNSS entering suspend\n");
	return 0;
}

static int imc_idi_gnss_dev_pm_resume(struct device *dev)
{
	struct uart_usif_xgold_port *uxp = dev_get_drvdata(dev);
	struct idi_gnss_port *p_gnss = xgold_port_priv(uxp);

	pr_debug("GNSS resuming\n");

	gnss_prepare_rx(p_gnss);
	return 0;
}

const struct dev_pm_ops imc_idi_gnss_dev_pm_ops = {
	.suspend = imc_idi_gnss_dev_pm_suspend,
	.resume = imc_idi_gnss_dev_pm_resume,
};

static struct idi_peripheral_driver imc_idi_gnss_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SERIAL_GNSS_NAME,
		.pm = &imc_idi_gnss_dev_pm_ops,
	 },
	.p_type = IDI_GNSS,
	.probe = imc_idi_gnss_probe,
	.remove = imc_idi_gnss_remove,
#ifdef CONFIG_PM
	.suspend = imc_idi_gnss_suspend,
	.resume = imc_idi_gnss_resume,
#endif
};

/*****************************************************************************
 *	name:		imc_idi_gnss_init
 *	description:	Linux driver init
 *	in params:	none
 *	out params:	none
 *	return val:	int ret
 ****************************************************************************/
static int __init imc_idi_gnss_init(void)
{
	int ret = 0;

	IMC_IDI_GNSS_ENTER;

	pr_debug("IMC IDI GNSS driver initialization\n");

#ifdef NATIVE_PLATFORM_DEVICE_PM
	ret = gnss_pm_add_class();
	if (ret) {
		pr_err("Error while adding BTIF pm class\n");
		return ret;
	}
#endif
	ret = uart_register_driver(&gnss_reg);
	if (ret) {
		IMC_IDI_GNSS_EXIT;
		pr_err("uart_register_driver failed\n");
		return -EINVAL;
	}

	ret = idi_register_peripheral_driver(&imc_idi_gnss_driver);
	if (ret) {
		pr_err("idi_register_peripheral_driver failed\n");
		uart_unregister_driver(&gnss_reg);
	}

	IMC_IDI_GNSS_EXIT;

	return ret;
}

/*****************************************************************************
 *	name:		imc_idi_gnss_exit
 *	description:	Linux driver exit
 *	in params:	none
 *	out params:	none
 *	return val:	none
 ****************************************************************************/
static void __exit imc_idi_gnss_exit(void)
{
	IMC_IDI_GNSS_ENTER;

	idi_unregister_peripheral_driver(&imc_idi_gnss_driver);
	uart_unregister_driver(&gnss_reg);

	IMC_IDI_GNSS_EXIT;
}

module_init(imc_idi_gnss_init);
module_exit(imc_idi_gnss_exit);
MODULE_DESCRIPTION("GNSS AGOLD port driver");
MODULE_LICENSE("GPL");
