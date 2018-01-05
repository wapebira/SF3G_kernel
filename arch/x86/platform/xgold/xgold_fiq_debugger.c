/*
 * arch/x86/platform/xgold/xgold_fiq_debugger.c
 *
 * Serial Debugger Interface for Rockchip
 * Copyright (C) 2015 ROCKCHIP, Inc.
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 * Copyright (C) 2008 Google, Inc.
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

#include <stdarg.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/serial_reg.h>
#include <linux/slab.h>
#include <linux/stacktrace.h>
#include <linux/uaccess.h>
#include <linux/kfifo.h>
#include <linux/kthread.h>
#include <linux/sched/rt.h>
#include <../drivers/staging/android/fiq_debugger/fiq_debugger.h>
#include <../drivers/tty/serial/xgold_usif.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/clk.h>
#include "xgold_fiq_debugger.h"
#include <linux/delay.h>
#include <linux/platform_device_pm.h>

struct xgold_fiq_debugger {
	int id;
	struct fiq_debugger_pdata pdata;
	void __iomem *debug_port_base;
	bool break_seen;
#ifdef CONFIG_XGOLD_CONSOLE_THREAD
	struct task_struct *console_task;
#endif
};

static int usif_suspend;
static void __iomem *io_base;
static struct device_pm_platdata *pm_platdata;
static struct platform_device *fiq_debugger_pdev;

static int debug_port_init(struct platform_device *pdev)
{
	/* Reconfigure USIF termios
	 *	So far, let's assume the bootloader is doing it
	*/
	iowrite32(0xAA6, USIF_CLC(io_base));
	iowrite32(1, USIF_CLC_CNT(io_base));
	/* set up other parameters, like parity ...*/
	iowrite32(0x10806, USIF_MODE_CFG(io_base));
	iowrite32(0x8, USIF_PRTC_CFG(io_base));
	iowrite32(0x0, USIF_IMSC(io_base));
	iowrite32(0xFFFFFFFF, USIF_ICR(io_base));
	/* set up baudrate */
	iowrite32(0x5, USIF_BC_CFG(io_base));
	iowrite32(0x1d00037, USIF_FDIV_CFG(io_base));
	iowrite32(0x2222, USIF_FIFO_CFG(io_base));
	iowrite32(0xAA5, USIF_CLC(io_base));

	iowrite32(0x51, USIF_FIFO_CTRL(io_base));
	iowrite32(0x3, USIF_IMSC(io_base));

	return 0;
}

static int debug_irq_mask(struct platform_device *pdev)
{
	iowrite32(0x0, USIF_IMSC(io_base));
	return 0;
}

static int debug_irq_unmask(struct platform_device *pdev)
{
	iowrite32(0x3, USIF_IMSC(io_base));
	return 0;
}

static int debug_getc(struct platform_device *pdev)
{
	unsigned int temp;

	if (usif_suspend)
		return -1;
	if (ioread32(USIF_FIFO_STAT(io_base)) & 0x000000FF) {
		temp = ioread32(USIF_RXD(io_base));
		iowrite32(0xFFFFFFFF, USIF_ICR(io_base));
		if (temp == 0)
			return FIQ_DEBUGGER_BREAK;
		else
			return temp;
	}

	iowrite32(0xFFFFFFFF, USIF_ICR(io_base));
	return FIQ_DEBUGGER_NO_CHAR;
}

static void debug_putc(struct platform_device *pdev, unsigned int c)
{
	unsigned int val = 0, count = 20;

	while (count--) {  /*fifo stage 32*/
		if (usif_suspend)
			return;
		val = USIF_FIFO_STAT_TXFFS(ioread32(USIF_FIFO_STAT(io_base)));
		if (val > 0x1f)
			udelay(50);
		else
			break;
	}
	iowrite32(c, USIF_TXD(io_base));
}

static void debug_flush(struct platform_device *pdev)
{
	while (!usif_suspend &&
	       USIF_FIFO_STAT_TXFFS(ioread32(USIF_FIFO_STAT(io_base)))) {
		barrier();
	}
}

#ifdef CONFIG_XGOLD_CONSOLE_THREAD
#include <linux/sizes.h>
#define FIFO_SIZE SZ_512K
static DEFINE_KFIFO(fifo, unsigned char, FIFO_SIZE);
static bool console_thread_stop;

static int console_thread(void *data)
{
	struct platform_device *pdev = data;
	struct xgold_fiq_debugger *t;
	unsigned char c;

	t = container_of(dev_get_platdata(&pdev->dev), typeof(*t), pdata);

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
		if (kthread_should_stop())
			break;
		set_current_state(TASK_RUNNING);
		while (!console_thread_stop &&
		       !usif_suspend && kfifo_get(&fifo, &c))
			debug_putc(pdev, c);
		if (!console_thread_stop)
			debug_flush(pdev);
	}

	return 0;
}

static void console_write(struct platform_device *pdev,
			  const char *s, unsigned int count)
{
	unsigned int fifo_count = FIFO_SIZE;
	unsigned char c, r = '\r';
	struct xgold_fiq_debugger *t;

	t = container_of(dev_get_platdata(&pdev->dev), typeof(*t), pdata);

	if (console_thread_stop ||
	    oops_in_progress ||
	    system_state == SYSTEM_HALT ||
	    system_state == SYSTEM_POWER_OFF ||
	    system_state == SYSTEM_RESTART) {
		if (!console_thread_stop) {
			console_thread_stop = true;
			smp_wmb();  /*why*/
			debug_flush(pdev);
			while (fifo_count-- && kfifo_get(&fifo, &c))
				debug_putc(pdev, c);
		}
		while (count--) {
			if (*s == '\n')
				debug_putc(pdev, r);
			debug_putc(pdev, *s++);
		}
		debug_flush(pdev);
	} else {
		while (count--) {
			if (*s == '\n')
				kfifo_put(&fifo, r);
			kfifo_put(&fifo, *s++);
		}
		if (!usif_suspend)
			wake_up_process(t->console_task);
	}
}
#endif

static int debug_uart_dev_suspend(struct platform_device *pdev)
{
	debug_irq_mask(pdev);
	usif_suspend = 1;
	if (pm_platdata)
		device_state_pm_set_state_by_name(&pdev->dev,
				pm_platdata->pm_state_D3_name);
	return 0;
}

static int debug_uart_dev_resume(struct platform_device *pdev)
{
	if (pm_platdata)
		device_state_pm_set_state_by_name(&pdev->dev,
				pm_platdata->pm_state_D0_name);
	debug_port_init(pdev);
	usif_suspend = 0;
	return 0;
}

static int fiq_debugger_pm_init(struct platform_device *pdev)
{
	struct device_node *np = of_find_node_by_name(NULL, "usif2");

	pm_platdata = of_device_state_pm_setup(np);

	if (IS_ERR(pm_platdata))
		pm_platdata = NULL;

	if (pm_platdata)
		platform_device_pm_set_class(pdev, pm_platdata->pm_user_name);

	return 0;
}

static int xgold_fiq_debugger_id;

void xgold_serial_debug_init(void __iomem *base,
			     int irq, int signal_irq, int wakeup_irq)
{
	struct xgold_fiq_debugger *t = NULL;
	struct platform_device *pdev = NULL;
	struct resource *res = NULL;
	int res_count = 0;

	if (!base) {
		pr_err("Invalid fiq debugger uart base\n");
		return;
	}

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return;

	t->pdata.uart_init = debug_port_init;
	t->pdata.uart_getc = debug_getc;
	t->pdata.uart_putc = debug_putc;
#ifndef CONFIG_XGOLD_CONSOLE_THREAD
	t->pdata.uart_flush = debug_flush;
#endif
	t->pdata.irq_unmask = debug_irq_unmask;
	t->pdata.irq_mask = debug_irq_mask;
	t->pdata.uart_dev_suspend = debug_uart_dev_suspend;
	t->pdata.uart_dev_resume = debug_uart_dev_resume;

	t->pdata.force_irq = NULL;
	t->debug_port_base = base;
	io_base = base;
	usif_suspend = 0;

	res = kzalloc(sizeof(*res) * 3, GFP_KERNEL);
	if (!res)
		goto out2;

	pdev = kzalloc(sizeof(*pdev), GFP_KERNEL);
	if (!pdev)
		goto out3;

	if (irq > 0) {
		res[0].flags = IORESOURCE_IRQ;
		res[0].start = irq;
		res[0].end = irq;
		res[0].name = "uart_irq";   /*fiq*/
		res_count++;
	}

	if (signal_irq > 0) {
		res[1].flags = IORESOURCE_IRQ;
		res[1].start = signal_irq;
		res[1].end = signal_irq;
		res[1].name = "signal";
		res_count++;
	}

	if (wakeup_irq > 0) {
		res[2].flags = IORESOURCE_IRQ;
		res[2].start = wakeup_irq;
		res[2].end = wakeup_irq;
		res[2].name = "wakeup";
		res_count++;
	}

#ifdef CONFIG_XGOLD_CONSOLE_THREAD
	t->console_task = kthread_create(console_thread, pdev, "kconsole");
	if (!IS_ERR(t->console_task))
		t->pdata.console_write = console_write;
#endif

	pdev->name = "fiq_debugger";
	pdev->id = xgold_fiq_debugger_id++;
	pdev->dev.platform_data = &t->pdata;
	pdev->resource = res;
	pdev->num_resources = res_count;
	fiq_debugger_pdev = pdev;

	if (platform_device_register(pdev)) {
		pr_err("Failed to register fiq debugger\n");
		goto out4;
	}
	return;

out4:
	kfree(pdev);
out3:
	kfree(res);
out2:
	kfree(t);
}

static const struct of_device_id ids[] __initconst = {
	{ .compatible = "rockchip,fiq-debugger" },
	{}
};

static int __init xgold_fiq_debugger_init(void)
{
	void __iomem *base;
	struct device_node *np;
	unsigned int serial_id;
	u32 irq, signal_irq = 0, wake_irq = 0;

	np = of_find_matching_node(NULL, ids);

	if (!np) {
		pr_err("fiq-debugger is missing in device tree!\n");
		return -ENODEV;
	}

	if (!of_device_is_available(np)) {
		pr_err("fiq-debugger is disabled in device tree\n");
		return -ENODEV;
	}

	if (of_property_read_u32(np, "rockchip,serial-id", &serial_id))
		return -EINVAL;

	if (of_property_read_u32(np, "rockchip,signal-irq", &signal_irq))
		signal_irq = -1;

	if (of_property_read_u32(np, "rockchip,wake-irq", &wake_irq))
		wake_irq = -1;

	np = NULL;
	if (serial_id == 1)
		np = of_find_node_by_name(np, "usif1");
	else
		np = of_find_node_by_name(np, "usif2");

	irq = irq_of_parse_and_map(np, 0);

	if (!irq)
		return -EINVAL;

	base = of_iomap(np, 0);

	if (base)
		xgold_serial_debug_init(base, irq, signal_irq, wake_irq);

	return 0;
}

postcore_initcall_sync(xgold_fiq_debugger_init);

static int __init xgold_fiq_debugger_pm_init(void)
{
	if (fiq_debugger_pdev)
		fiq_debugger_pm_init(fiq_debugger_pdev);
	return 0;
}

late_initcall_sync(xgold_fiq_debugger_pm_init);
