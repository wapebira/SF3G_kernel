#ifndef _SP_LINUX_WAKEUP_HANDLER_H_
#define _SP_LINUX_WAKEUP_HANDLER_H_ 1

#define LINUX_MESSAGE_CPU_MSB 0x80

#include "pw_structs.h"

int register_with_tracepoints(void);
int unregister_with_tracepoints(void);
void print_tracepoint_activity(void);
int check_if_tracepoints_enabled(void);
int mapSTM (void);
void unmapSTM (void);
unsigned long long readSTM(void);

#endif // _SP_LINUX_WAKEUP_HANDLER_H_
