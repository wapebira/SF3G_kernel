#ifndef __PLAT_XGOLD_FIQ_DEBUGGER_H
#define __PLAT_XGOLD_FIQ_DEBUGGER_H

#ifdef CONFIG_FIQ_DEBUGGER
void xgold_serial_debug_init(void __iomem *base,
			     int irq, int signal_irq, int wakeup_irq);
#else
static inline void
xgold_serial_debug_init(void __iomem *base,
			int irq, int signal_irq, int wakeup_irq)
{
}
#endif

#endif
