#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <asm/io.h>
#include "sp_vmm_buffer.h"
//#include "spioctl.h"
#include "sofia/pal_shared_data.h"
#include "sofia/mv_svc_hypercalls.h"

#define BUFFER_SIZE 16

struct socwatch_buffer_info* pPhysBufferInfo = NULL;
struct socwatch_buffer_info* pBufferInfo = NULL;


// TODO check if vmm delivers injected interrupts on stop to deliver partially filled buffers
// cpu is an OUT
bool vmm_is_buffer_ready(int * cpu)
{
  int i;
  // check each buffer_status value for SOCWATCH_BUFFER_VALID,
  //  if found return true and the cpu index representing which buffer is read to be copied
  if (pBufferInfo != NULL) {
    for (i=0; i<pBufferInfo->num_buffers_allocated; i++) {
      // this code assumes the vmm decides when the buffer is ready to consume - when
      //  it's full or when the collection has been stopped and the buffer is partially filled
      if (pBufferInfo->buffer_status[i] == SOCWATCH_BUFFER_VALID) {
        *cpu = i;
        return true;
      }
    }
  }
  else {
    printk(KERN_INFO "sofia-proto: vmm_is_buffer_ready: ERROR: pBufferInfo is NULL!\n");
    return false;
  }

  // didn't find any ready buffers
  return false;
};

int vmm_init_buffers(int num_cpus)
{
  int i;

  // allocate the structure that holds information about the buffers passed to the vmm
  pBufferInfo = (struct socwatch_buffer_info*)kmalloc(sizeof(struct socwatch_buffer_info), GFP_KERNEL);
  if (pBufferInfo == NULL) {
    printk(KERN_INFO "sofia-proto: failed to allocate struct socwatch_buffer_info structure\n");
    return -ENOMEM;
  }
  // printk(KERN_INFO "sofia-proto: allocated struct socwatch_buffer_info structure\n");

  // num_buffers_allocated should be twice the number of cpus in the system
  pBufferInfo->num_buffers_allocated = num_cpus*2;
  pBufferInfo->buffer_length = BUFFER_SIZE;
  for(i = 0; i < pBufferInfo->num_buffers_allocated; i++) {
    pBufferInfo->buffer_start[i] = (uint64_t)(unsigned long)kmalloc(pBufferInfo->buffer_length * 1024, GFP_KERNEL);
    if (pBufferInfo->buffer_start[i] == 0) {
      printk(KERN_INFO "sofia-proto: failed to allocate buffer for sharing with vmm\n");
      return -ENOMEM;
    }
    // set buffer status to 'consumed' to indicate the vmm can use buffer immediately
    pBufferInfo->buffer_status[i] = SOCWATCH_BUFFER_CONSUMED;
    // printk(KERN_INFO "sofia-proto: pBufferInfo->buffer_start[%d] = 0x%p\n", i, (void *)(unsigned long)pBufferInfo->buffer_start[i]);
  }
  // printk(KERN_INFO "sofia-proto: done allocating data buffers.\n");

  // convert virtual buffer addresses to physical buffer addresses - the vmm requires physical addresses
  for(i = 0; i < pBufferInfo->num_buffers_allocated; i++) {
    pBufferInfo->buffer_start[i] = (uint64_t)(unsigned long)virt_to_phys((void *)(unsigned long)pBufferInfo->buffer_start[i]);
    if (pBufferInfo->buffer_start[i] == 0) {
      printk(KERN_INFO "sofia-proto: failed to convert data buffer %d start address to physical address\n", i);
      // not sure what error to return but let's start with -EIO
      return -EIO;
    }
  }
  pPhysBufferInfo = (struct socwatch_buffer_info *)virt_to_phys(pBufferInfo);
  if (pPhysBufferInfo == NULL) {
    printk(KERN_INFO "sofia-proto: failed to convert socwatch_buffer_info struct to physical memory address\n");
    return -EIO;
  }

  // printk(KERN_INFO "sofia-proto: pBufferInfo=0x%p, pPhysBufferInfo=0x%p\n", pBufferInfo, pPhysBufferInfo);

  return 0;
};

void vmm_destroy_buffers()
{
  int i;

  if (pBufferInfo) {
    for(i = 0; i < pBufferInfo->num_buffers_allocated; i++) {
      // convert back to virtual memory of necessary
      pBufferInfo->buffer_start[i] = (uint64_t)(unsigned long)phys_to_virt((unsigned long)pBufferInfo->buffer_start[i]);
      if (pBufferInfo->buffer_start[i] != 0)
        kfree((void *)(unsigned long)pBufferInfo->buffer_start[i]);
    }
    if (pBufferInfo)
      kfree(pBufferInfo);
  }
};

void vmm_reset_buffers ()
{
  int i;

  if (pBufferInfo) {
    for(i = 0; i < pBufferInfo->num_buffers_allocated; i++) {
       // set buffer status to 'consumed' to indicate the vmm can use buffer immediately
       pBufferInfo->buffer_status[i] = SOCWATCH_BUFFER_CONSUMED;
    }
  }
};




