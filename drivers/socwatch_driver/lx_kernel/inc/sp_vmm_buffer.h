#ifndef _SP_BUFFER_H_
#define _SP_BUFFER_H_ 1

//#include "sofia/vmm_platform_service.h"

#define BUFFER_SIZE 16

bool vmm_is_buffer_ready(int *cpu);
int vmm_init_buffers(int num_cpus);
void vmm_destroy_buffers(void);
void vmm_reset_buffers (void);



#endif // _SP_BUFFER_H_
