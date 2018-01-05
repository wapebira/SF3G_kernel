#ifndef LINUX_BACKPORTS_PRIVATE_H
#define LINUX_BACKPORTS_PRIVATE_H

#include <linux/version.h>

#ifdef CPTCFG_BACKPORT_BUILD_DMA_SHARED_BUFFER
int __init dma_buf_init(void);
void __exit dma_buf_deinit(void);
#else
static inline int __init dma_buf_init(void) { return 0; }
static inline void __exit dma_buf_deinit(void) { }
#endif

//#ifdef CPTCFG_BACKPORT_BUILD_WANT_DEV_COREDUMP
#if 1
int devcoredump_init(void);
void devcoredump_exit(void);
#else
static inline int devcoredump_init(void)
{ return 0; }
static inline void devcoredump_exit(void)
{}
#endif

#endif /* LINUX_BACKPORTS_PRIVATE_H */
