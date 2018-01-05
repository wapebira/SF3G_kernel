#ifndef _ASM_X86_IDLE_H
#define _ASM_X86_IDLE_H

#if defined CONFIG_X86_64 || defined CONFIG_X86_INTEL_SOFIA
void enter_idle(void);
void exit_idle(void);
#else /* !CONFIG_X86_64 */
static inline void enter_idle(void) { }
static inline void exit_idle(void) { }
static inline void __exit_idle(void) { }
#endif /* CONFIG_X86_64 */

void amd_e400_remove_cpu(int cpu);

#endif /* _ASM_X86_IDLE_H */
