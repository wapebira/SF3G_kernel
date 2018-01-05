/*
 * rk818-irq.c
 *
 * Author: zhangqing <zhangqing@rock-chips.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/bug.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mfd/rk818.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#include <linux/irqdomain.h>
#include <linux/regmap.h>

static inline int irq_to_rk818_irq(struct rk818 *rk818, int irq)
{
	return irq - rk818->irq_base;
}

/*
 * This is a threaded IRQ handler so can access I2C/SPI.  Since all
 * interrupts are clear on read the IRQ line will be reasserted and
 * the physical IRQ will be handled again if another interrupt is
 * asserted while we run - in the normal course of events this is a
 * rare occurrence so we save I2C/SPI reads.  We're also assuming that
 * it's rare to get lots of interrupts firing simultaneously so try to
 * minimise I/O.
 */
static irqreturn_t rk818_irq(int irq, void *irq_data)
{
	struct rk818 *rk818 = irq_data;
	u32 irq_sts;
	u8 pwr_irq_sts = 0;
	u8 reg;
	int i;

	wake_lock(&rk818->irq_wake);
	rk818_i2c_read(rk818, RK818_INT_STS_REG1, 1, &reg);
	irq_sts = reg;
	rk818_i2c_read(rk818, RK818_INT_STS_REG2, 1, &reg);
	irq_sts |= reg << 8;
	if (rk818->chip_id == PMIC_CHIP_RK819)
		rk818_i2c_read(rk818, RK818_LDO8_ON_VSEL_REG, 1, &pwr_irq_sts);

	pr_info("irq_sts = 0x%2x, pwr_irq_sts = 0x%2x\n",
		irq_sts, pwr_irq_sts&0x60);

	if (!irq_sts) {
		wake_unlock(&rk818->irq_wake);
		pr_err("rk818 no child irq founded, wake unlock, return\n");
		return IRQ_NONE;
	}

	for (i = 0; i < rk818->irq_num; i++) {
		if (!(irq_sts & (1 << i)))
			continue;

		handle_nested_irq(rk818->irq_base + i);
	}

	/* Write the STS register back to clear IRQs we handled */
	reg = irq_sts & 0xFF;
	irq_sts >>= 8;
	rk818_i2c_write(rk818, RK818_INT_STS_REG1, 1, reg);
	reg = irq_sts & 0xFF;
	rk818_i2c_write(rk818, RK818_INT_STS_REG2, 1, reg);
	if ((rk818->chip_id == PMIC_CHIP_RK819) && (pwr_irq_sts&0x60)) {
		rk818_set_bits(rk818, RK818_LDO8_ON_VSEL_REG,
			       (pwr_irq_sts&0x60), (pwr_irq_sts&0x60));
	}
	wake_unlock(&rk818->irq_wake);

	return IRQ_HANDLED;
}

static void rk818_irq_lock(struct irq_data *data)
{
	struct rk818 *rk818 = irq_data_get_irq_chip_data(data);

	rt_mutex_lock(&rk818->irq_lock);
}

static void rk818_irq_sync_unlock(struct irq_data *data)
{
	struct rk818 *rk818 = irq_data_get_irq_chip_data(data);
	u32 reg_mask;
	u8 reg;

	rk818_i2c_read(rk818, RK818_INT_STS_MSK_REG1, 1, &reg);
	reg_mask = reg;
	rk818_i2c_read(rk818, RK818_INT_STS_MSK_REG2, 1, &reg);
	reg_mask |= reg << 8;

	if (rk818->irq_mask != reg_mask) {
		reg = rk818->irq_mask & 0xff;
		/*rk818_i2c_write(rk818, RK818_INT_STS_MSK_REG1, 1, reg);*/
		reg = rk818->irq_mask >> 8 & 0xff;
		/*rk818_i2c_write(rk818, RK818_INT_STS_MSK_REG2, 1, reg);*/
	}
	rt_mutex_unlock(&rk818->irq_lock);
}

static void rk818_irq_enable(struct irq_data *data)
{
	struct rk818 *rk818 = irq_data_get_irq_chip_data(data);

	rk818->irq_mask &= ~(1 << irq_to_rk818_irq(rk818, data->irq));
}

static void rk818_irq_disable(struct irq_data *data)
{
	struct rk818 *rk818 = irq_data_get_irq_chip_data(data);

	rk818->irq_mask |= (1 << irq_to_rk818_irq(rk818, data->irq));
}

#ifdef CONFIG_PM_SLEEP
static int rk818_irq_set_wake(struct irq_data *data, unsigned int enable)
{
	struct rk818 *rk818 = irq_data_get_irq_chip_data(data);

	return irq_set_irq_wake(rk818->chip_irq, enable);
}
#else
#define rk818_irq_set_wake NULL
#endif

static struct irq_chip rk818_irq_chip = {
	.name = "rk818",
	.irq_bus_lock = rk818_irq_lock,
	.irq_bus_sync_unlock = rk818_irq_sync_unlock,
	.irq_disable = rk818_irq_disable,
	.irq_enable = rk818_irq_enable,
	.irq_set_wake = rk818_irq_set_wake,
};

static void rk81x_init_irq_mask(struct rk818 *rk818)
{
	rk818_i2c_write(rk818, RK818_INT_STS_MSK_REG1, 1, 0x8f);
	rk818_i2c_write(rk818, RK818_INT_STS_MSK_REG2, 1, 0xfc);
	if (rk818->chip_id == PMIC_CHIP_RK819) {
		/* rk819 must mask 4ah, PWRON_RISE_INT_IM, PWRON_FALL_INT_IM*/
		rk818_set_bits(rk818, RK818_LDO8_SLP_VSEL_REG,
			       (0x3 << 5), (0x3 << 5));
	}
}

int rk818_irq_init(struct rk818 *rk818)
{
	int ret, flags = 0, cur_irq = 0;
	u8 reg;

	/* Clear unattended interrupts */
	rk818_i2c_read(rk818, RK818_INT_STS_REG1, 1, &reg);
	rk818_i2c_write(rk818, RK818_INT_STS_REG1, 1, reg);
	rk818_i2c_read(rk818, RK818_INT_STS_REG2, 1, &reg);
	rk818_i2c_write(rk818, RK818_INT_STS_REG2, 1, reg);
	rk818_i2c_read(rk818, RK818_RTC_STATUS_REG, 1, &reg);
	/*clear alarm and timer interrupt*/
	rk818_i2c_write(rk818, RK818_RTC_STATUS_REG, 1, reg);
	if (rk818->chip_id == PMIC_CHIP_RK819) {
		rk818_i2c_read(rk818, RK818_LDO8_ON_VSEL_REG, 1, &reg);
		rk818_set_bits(rk818, RK818_LDO8_ON_VSEL_REG,
			       (reg & 0x60), (reg & 0x60));
	}

	rk81x_init_irq_mask(rk818);

	/* Mask top level interrupts */
	rk818->irq_mask = 0xFFFFFF;
	rt_mutex_init(&rk818->irq_lock);
	wake_lock_init(&rk818->irq_wake, WAKE_LOCK_SUSPEND, "rk818_irq_wake");
	rk818->irq_num = RK818_NUM_IRQ;

	rk818->irq_base = irq_alloc_descs(-1, 0, rk818->irq_num, 0);
	if (rk818->irq_base < 0) {
		pr_err("Failed to allocate IRQs: %d\n", rk818->irq_base);
		rk818->irq_base = 0;
	}

	/* Register with genirq */
	for (cur_irq = rk818->irq_base;
	     cur_irq < rk818->irq_num + rk818->irq_base; cur_irq++) {
		irq_set_parent(cur_irq, rk818->chip_irq);
		irq_set_chip_data(cur_irq, rk818);
		irq_set_chip_and_handler(cur_irq, &rk818_irq_chip,
					 handle_edge_irq);
		irq_set_nested_thread(cur_irq, 1);

		/* ARM needs us to explicitly flag the IRQ as valid
		 * and will set them noprobe when we do so. */
#ifdef CONFIG_ARM
		set_irq_flags(cur_irq, IRQF_VALID);
#else
		irq_set_noprobe(cur_irq);
#endif
	}
	ret = request_threaded_irq(rk818->chip_irq, NULL, rk818_irq,
				   flags | IRQF_ONESHOT,
				   "rk818", rk818);

	enable_irq_wake(rk818->chip_irq);
	if (ret != 0)
		dev_err(rk818->dev, "Failed to request IRQ: %d\n", ret);

	return ret;
}

int rk818_irq_exit(struct rk818 *rk818)
{
	if (rk818->chip_irq)
		free_irq(rk818->chip_irq, rk818);
	return 0;
}
