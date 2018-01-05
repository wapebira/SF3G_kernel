/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* Some vectors may be platform dependent such as EINT
 * Move those vectors from irq-vectors to there :g!/EXI/d
 * use #ifdef MY_PLATFORM in case of shared IRQ
 * */

#ifndef _IRQ_VECTORS_PLF_H
#define _IRQ_VECTORS_PLF_H

#define EXI0 52
#define EXI1 71
#define EXI2 72
#define EXI3 53
#define EXI4 54
#define EXI5 55
#define EXI6 0
#define EXI7 56
#define EXI8 57
#define EXI9 0
#define EXI10 59
#define EXI11 60
#define EXI12 61
#define EXI13 0
#define EXI14 63
#define EXI15 0

#endif /*_IRQ_VECTORS_PLF_H */
