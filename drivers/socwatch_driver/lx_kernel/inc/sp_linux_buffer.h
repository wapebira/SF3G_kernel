#ifndef _SP_LINUX_BUFFER_H_
#define _SP_LINUX_BUFFER_H_ 1

#include "sp.h"

// miscellaneous defines
#define CIRCULAR_INC(index, mask) ( ( (index) + 1) & (mask) )

#define PW_SEG_DATA_SIZE 32760
#define PW_SEG_SIZE_BYTES ( PW_SEG_DATA_SIZE + 2 * sizeof(u32) ) // save room for bytes_written and is_full fields
#define PW_DATA_BUFFER_SIZE (PW_SEG_SIZE_BYTES)
#define PW_OUTPUT_BUFFER_SIZE (PW_DATA_BUFFER_SIZE * NUM_SEGS_PER_BUFFER)

#define NUM_SEGS_PER_BUFFER 2 /* MUST be POW-of-2 */
#define NUM_SEGS_PER_BUFFER_MASK 1 /* MUST be (NUM_SEGS_PER_BUFFER - 1) */
#define SEG_SIZE (NUM_SAMPLES_PER_SEG * sizeof(PWCollector_sample_t))

typedef struct pw_data_buffer pw_data_buffer_t;
typedef struct pw_output_buffer pw_output_buffer_t;

/*
 * Output buffer data structures.
 */
struct pw_data_buffer {
    int bytes_written;
    int is_full;
    char buffer[1];
};


struct pw_output_buffer {
    // actual data buffers
    pw_data_buffer_t *buffers[NUM_SEGS_PER_BUFFER];
    // indix of the segment buffer in use
    int buff_index;
    int produced_samples;
    int dropped_samples;
    int last_seg_read;
    unsigned long free_pages;
    unsigned long mem_alloc_size;
} ____cacheline_aligned_in_smp;


int pw_init_per_cpu_buffers(void);
void pw_destroy_per_cpu_buffers(void);
void pw_reset_per_cpu_buffers(void);
int pw_produce_generic_msg(struct PWCollector_msg *msg, bool allow_wakeup);
int pw_produce_generic_msg(PWCollector_msg_t *msg, bool allow_wakeup);
bool is_linux_kernel_buffer_ready (bool flush_now, pw_data_buffer_t** pSegment);

#endif // _SP_LINIX_BUFFER_H_
