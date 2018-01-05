#ifndef _SP_LINUX_IRQ_HANDLER_H_
#define _SP_LINUX_IRQ_HANDLER_H_ 1

void handle_irq_wakeup_i(int cpu, int irq_num, const char *irq_name, unsigned long long tsc);
int init_irq_map(void);
void destroy_irq_map(void);

#endif // _SP_LINUX_IRQ_HANDLER_H_
