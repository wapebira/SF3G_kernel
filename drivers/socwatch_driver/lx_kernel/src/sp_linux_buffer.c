#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include "sp_linux_buffer.h"
#include "sp_linux_wakeup_handler.h"
#include "sofia/mv_svc_hypercalls.h"

#define SPACE_AVAIL(seg) ( (seg)->is_full ? 0 : (PW_SEG_DATA_SIZE - (seg)->bytes_written) )
#define GET_OUTPUT_BUFFER(cpu) &per_cpu_output_buffers[(cpu)]

#define for_each_segment(i) for (i=0; i<NUM_SEGS_PER_BUFFER; ++i)

extern int num_CPUs;

int pw_last_cpu_read = -1;
pw_output_buffer_t *per_cpu_output_buffers = NULL;
extern wait_queue_head_t read_queue;

#if 0
int irq_msgs = 0;
int proc_maps = 0;
int cstate_msgs = 0;
int cmm_msgs = 0;
int posix_msgs = 0;
int cpu0_msgs = 0;
int cpu1_msgs = 0;
#endif

pw_data_buffer_t inline *pw_get_next_available_segment_i(pw_output_buffer_t *buffer, int size)
{
  int seg;
  int buff_index = buffer->buff_index;

  for_each_segment(seg) {
    buff_index = CIRCULAR_INC(buff_index, NUM_SEGS_PER_BUFFER_MASK);
    if (SPACE_AVAIL(buffer->buffers[buff_index]) >= size) {
      buffer->buff_index = buff_index;
      return buffer->buffers[buff_index];
    }
  }
  return NULL;
};


static pw_data_buffer_t *get_producer_seg_i(size_t size, int *cpu, int *write_index, bool *should_wakeup, bool *did_drop_sample)
{
  pw_data_buffer_t *seg = NULL;
  pw_output_buffer_t *buffer;
  unsigned long flags = 0;
  int my_cpu = raw_smp_processor_id();
  int buff_index;

  local_irq_save(flags);
  {
    *cpu = my_cpu;
    buffer = GET_OUTPUT_BUFFER(my_cpu);
//  printk(KERN_INFO "sofia-proto: get_producer_seg_i() cpu %d using buffer 0x%p\n", my_cpu, buffer);
    buff_index = buffer->buff_index;
    if (buff_index < 0 || buff_index >= NUM_SEGS_PER_BUFFER) {
      printk(KERN_INFO "sofia-proto: ERROR: can't find seg; cpu=%d, buff_index=%d\n", my_cpu, buff_index);
      seg = NULL;
      goto prod_seg_done;
    }
    seg = buffer->buffers[buff_index];

//  printk(KERN_INFO "sofia-proto: get_producer_seg_i() using seg 0x%p and buff index %d\n", seg, buff_index);

    if (unlikely(SPACE_AVAIL(seg) < size)) {
      seg->is_full = 1;
      *should_wakeup = true;
      seg = pw_get_next_available_segment_i(buffer, size);
      if (seg == NULL) {
        /*
         * We couldn't find a non-full segment.
         */
//      buffer->dropped_samples++;
//        *did_drop_sample = true;
        goto prod_seg_done;
      }
    }
    *write_index = seg->bytes_written;
    seg->bytes_written += size;

//  buffer->produced_samples++;
  }
prod_seg_done:
  // put_cpu();
  local_irq_restore(flags);
  return seg;
};


//int pw_produce_generic_msg(struct PWCollector_msg *msg, bool allow_wakeup)
int pw_produce_generic_msg(PWCollector_msg_t *msg, bool allow_wakeup)
{
  bool should_wakeup = false;
  bool did_drop_sample = false;
  int cpu;
//  bool did_switch_buffer = false;
  int size = 0;
  pw_data_buffer_t *seg = NULL;
  char *dst = NULL;
  int write_index = 0;
//  int segment;
//  pw_output_buffer_t *buffer;

  if (!msg) {
    printk(KERN_INFO "sofia-proto: ERROR: CANNOT produce a NULL msg!\n");
    return -1;
  }

  size = msg->data_len + PW_MSG_HEADER_SIZE;

#if 0
  if (msg->data_type == IRQ_MAP && irq_msgs <= 4) {
    printk(KERN_INFO "sofia-proto: pw_produce_generic_msg() IRQ_MAP - TSC=0x%llx, len=%d\n", msg->tsc, size);
    irq_msgs++;
  }
  else if (msg->data_type == PROC_MAP && proc_maps <= 3) {
    printk(KERN_INFO "sofia-proto: pw_produce_generic_msg() PROC_MAP - TSC=0x%llx, len=%d\n", msg->tsc, size);
    proc_maps++;
  }
  else if (msg->data_type == C_STATE && cstate_msgs <= 3) {
    printk(KERN_INFO "sofia-proto: pw_produce_generic_msg() C_STATE - TSC=0x%llx, len=%d\n", msg->tsc, size);
    cstate_msgs++;
  }
  else if (msg->data_type == TSC_POSIX_MONO_SYNC && posix_msgs <= 3) {
    printk(KERN_INFO "sofia-proto: pw_produce_generic_msg() TSC_POSIX_MONO_SYNC - TSC=0x%llx, len=%d\n", msg->tsc, size);
    posix_msgs++;
  }
  else if (msg->data_type == C_MULTI_MSG && cmm_msgs <= 3) {
    printk(KERN_INFO "sofia-proto: pw_produce_generic_msg() C_MULTI_MSG - TSC=0x%llx, len=%d\n", msg->tsc, size);
    cmm_msgs++;
  }
//  printk(KERN_INFO "sofia-proto: cpu [%d] producing msg: size = %d\n", raw_smp_processor_id(), size);
#endif
  seg = get_producer_seg_i(size, &cpu, &write_index, &should_wakeup, &did_drop_sample);
//  printk(KERN_INFO "sofia-proto: will use seg 0x%p\n", seg);

  if (likely(seg)) {
    dst = &seg->buffer[write_index];
#if 0
    if (cpu == 0 && cpu0_msgs < 3) {
      printk(KERN_INFO "sofia-proto: pw_produce_generic_msg() cpu=%d, seg=0x%p, write_index=0x%x, dst=0x%p, bytes_written=%d\n", cpu, seg, write_index, dst, seg->bytes_written);
      cpu0_msgs++;
    }
    if (cpu == 1 && cpu1_msgs < 3) {
      printk(KERN_INFO "sofia-proto: pw_produce_generic_msg() cpu=%d, seg=0x%p, write_index=0x%x, dst=0x%p\n", cpu, seg, write_index, dst);
      cpu1_msgs++;
    }
#endif
    memcpy (dst, msg, PW_MSG_HEADER_SIZE);
//    *((struct PWCollector_msg *)dst) = *msg;
    dst += PW_MSG_HEADER_SIZE;
    memcpy(dst, (void *)((unsigned long)msg->p_data), msg->data_len);
  } else {
    printk(KERN_INFO "sofia-proto: ERROR: trying to copy data into NULL segment\n");
  }

  if (unlikely(should_wakeup && allow_wakeup && waitqueue_active(&read_queue))) {
//        set_bit(cpu, &reader_map); // we're guaranteed this won't get reordered!
//        smp_mb(); // TODO: do we really need this?
        // printk(KERN_INFO "[%d]: has full seg!\n", cpu);
//        printk(KERN_INFO "sofia-proto: cpu [%d]: has full seg! waking up device_read()\n", cpu);
    wake_up_interruptible(&read_queue);
  }
//  printk(KERN_INFO "sofia-proto: pw_produce_generic_msg()");
#if 0
  for (cpu=0; cpu<num_CPUs; cpu++) {
    buffer = GET_OUTPUT_BUFFER(cpu);

    for_each_segment(segment) {
      seg = buffer->buffers[segment];
      printk(KERN_INFO "sofia-proto: pw_produce_generic_msg() cpu=%d, seg=0x%p : bytes_written=0x%x\n", cpu, seg, buffer->buffers[segment]->bytes_written);
    }
  }
#endif
  return 0;
};


void pw_destroy_per_cpu_buffers(void)
{
  int cpu;
  pw_output_buffer_t *buffer;

  if (per_cpu_output_buffers != NULL) {
    // for_each_possible_cpu(cpu) {
    for (cpu=0; cpu<num_CPUs; cpu++) {
      buffer = &per_cpu_output_buffers[cpu];
      if (buffer->free_pages != 0) {
        // printk(KERN_INFO "sofia-proto: freeing cpu %d's linux kernel buffer\n", cpu);
        free_pages(buffer->free_pages, get_order(buffer->mem_alloc_size));
        buffer->free_pages = 0;
      }
    }
    // printk(KERN_INFO "sofia-proto: freeing the per_cpu_output_buffer struct\n");
    kfree(per_cpu_output_buffers);
    per_cpu_output_buffers = NULL;
  }
};


int pw_init_per_cpu_buffers(void)
{
    int cpu, seg, order;
    char *buff = NULL;
    pw_output_buffer_t *buffer;

    // printk (KERN_INFO "sofia-proto: allocating the per_cpu_output_buffer struct\n");
    per_cpu_output_buffers = (pw_output_buffer_t *)kmalloc(sizeof(pw_output_buffer_t) * num_CPUs, GFP_KERNEL | __GFP_ZERO);
    if (per_cpu_output_buffers == NULL) {
        printk(KERN_INFO "sofia-proto: ERROR allocating space for per-cpu output buffers!\n");
        pw_destroy_per_cpu_buffers();
        return -ENOMEM;
    }
    // printk(KERN_INFO "sofia-proto: pw_init_per_cpu_buffers() per_cpu_output_buffers = 0x%p\n", per_cpu_output_buffers);

    for (cpu=0; cpu<num_CPUs; cpu++) {
        buffer = &per_cpu_output_buffers[cpu];
        buffer->mem_alloc_size = PW_OUTPUT_BUFFER_SIZE;
        order = get_order(buffer->mem_alloc_size);
        // printk(KERN_INFO "sofia-proto: allocating cpu %d buffer, order=%d\n", cpu, order);
        buffer->free_pages = __get_free_pages(GFP_KERNEL, order);
        if (buffer->free_pages == 0) {
            printk(KERN_INFO "sofia-proto: ERROR allocating pages for buffer [%d]!\n", cpu);
            pw_destroy_per_cpu_buffers();
            return -ENOMEM;
        }
        buff = (char *)buffer->free_pages;
        for_each_segment(seg) {
            buffer->buffers[seg] = (pw_data_buffer_t *)buff;
            buffer->buffers[seg]->bytes_written = 0;
            buffer->buffers[seg]->is_full = 0;
            // printk(KERN_INFO "sofia-proto: pw_init_per_cpu_buffers() cpu %d, segment %d, buffer=0x%p\n", cpu, seg, buff);
            buff += PW_DATA_BUFFER_SIZE;
        }
    }
    return 0;
};


void pw_reset_per_cpu_buffers(void)
{
  int cpu, seg;
  pw_output_buffer_t *buffer;

  for (cpu=0; cpu<num_CPUs; cpu++) {
    buffer = GET_OUTPUT_BUFFER(cpu);
    buffer->buff_index = 0;
    buffer->last_seg_read = -1;

    for_each_segment(seg) {
      buffer->buffers[seg]->bytes_written = 0;
      buffer->buffers[seg]->is_full = 0;
    }
  }
    pw_last_cpu_read = -1;
#if 0
  irq_msgs = 0;
  proc_maps = 0;
  cstate_msgs = 0;
  cmm_msgs = 0;
  posix_msgs = 0;
  cpu0_msgs = 0;
  cpu1_msgs = 0;
#endif
};

// if flush_now is false, find a full segment and return info about it via the pSegment field
// if flush_now is true, find a segment that has any data in it and return info about it via the pSegment field
bool is_linux_kernel_buffer_ready (bool flush_now, pw_data_buffer_t** pSegment)
{
  int cpu, seg;
  pw_output_buffer_t *buffer;

  for (cpu=0; cpu<num_CPUs; cpu++) {
    if (++pw_last_cpu_read >= num_CPUs) {
      pw_last_cpu_read = 0;
    }
    buffer = GET_OUTPUT_BUFFER(pw_last_cpu_read);
    for_each_segment(seg) {
      if (++buffer->last_seg_read >= NUM_SEGS_PER_BUFFER) {
        buffer->last_seg_read = 0;
      }
      smp_mb();
      if (buffer->buffers[buffer->last_seg_read]->is_full || (flush_now && buffer->buffers[buffer->last_seg_read]->bytes_written > 0)) {
        // return the segment containing the data to be returned to user mode
        *pSegment = buffer->buffers[buffer->last_seg_read];
        return true;
      }
    }
  }
  *pSegment = NULL;
  return false;
};
















