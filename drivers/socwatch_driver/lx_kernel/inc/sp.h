#ifndef _SP_H_
#define _SP_H_ 1

#include "pw_structs.h"
#include "pw_version.h"

#include "sofia/mv_svc_hypercalls.h"
#define STM_base 0xE4300000
#define STM_size 0X40
#define STM_TIM0_offset 0x20
#define STM_CAP_offset 0x32

// miscellaneous defines
#define CPU() (raw_smp_processor_id())
#define GET_BOOL_STRING(b) ( (b) ? "TRUE" : "FALSE" )

/*
 * 64bit Compare-and-swap.
 */
#define CAS64(p, o, n) cmpxchg64((p), (o), (n)) == (o)

typedef struct PWCollector_msg PWCollector_msg_t;

#endif // _SP_H_
