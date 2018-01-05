/*
 * xgold.h: Intel XGOLD platform specific setup code
 *
 * (C) Copyright 2013 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _ASM_X86_XGOLD_H
#define _ASM_X86_XGOLD_H

/* Chip REV */
#define CHIP_REV_SF_3GR_SOC_ES1 (0x00)
#define CHIP_REV_SF_3GR_SOC_ES2 (0x10)

extern int xgold_get_chip_rev(void);

extern struct console early_xgold_console;
extern void xgold_early_console_init(unsigned long base);
#ifdef CONFIG_X86_INTEL_XGOLD_PSTORE_RAM
extern void __init pstore_ram_reserve_memory(void);
#else
static inline void pstore_ram_reserve_memory(void) { }
#endif

extern bool xgold_platform_needs_broadcast_timer(void);
#endif
