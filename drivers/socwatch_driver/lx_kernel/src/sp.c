#define MOD_AUTHOR "Robert Knight"
#define MOD_DESC "prototype delivery of sofia VMM buffers"


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
#include "sp.h"
#include "sp_vmm_buffer.h"
#include "sp_linux_buffer.h"
#include "sp_linux_wakeup_handler.h"
#include "sp_linux_timer_handler.h"
#include "spioctl.h"
/*
 * There is a build error because a typedef is being used in vmm_platform_service.h
 * SoCWatch doesn't use the typedef.
 * But adding this header as a work-around to prevent a build error of the socwatch driver
 * because of the typedef mentioned above.
 */
#include "sofia/pal_shared_data.h"
#include "sofia/mv_svc_hypercalls.h"

#define BUFFER_SIZE 16
#define SOCWATCH_IRQ 120

static int sp_dev_major_num = -1;
static dev_t sp_dev;
static struct cdev *sp_cdev;
static struct class *sp_class = NULL;

int num_CPUs = 0;
bool flush_now;

extern struct socwatch_buffer_info* pPhysBufferInfo;
extern struct socwatch_buffer_info* pBufferInfo;
wait_queue_head_t read_queue;

static int enable_tracepoints = 0;

// the following code is used for debugging
// static int num_interrupts = 0;

// The device_read condition is checked upon executing the wait_event_interruptable function
// 3 situations exist: a) only  vmm buffer is ready, b) only a linux buffer is ready, c) both a
//  vmm buffer and linux buffer are ready
//  In a), only the vmm_buf_rdy variable is set, that branch is taken, and a single buffer is returned
//  In b), only the linux_buf_rdy variable is set, that branch is taken, and a single buffer is returned
//  In c), which can occur during the collection and probably will occur at the end of a collection as soon
//   as the flush_now variable is set, the first buffer found is returned to user mode following the same
//   process as a) or b). Usermode will immediatly call read again since read returned a non-zero result,
//   and the wait_event_interruptable condition will be checked again and find another buffer read.
//   This loop will continue until no buffers are found to have data which will cause this function
//   to return 0. Usermode code should end the user mode loop when 0 is returned.
//   To force the end of collection return of partially filled buffers, call wake_up_interruptible(&read_queue)
//   on collection stop.
// Deallocation of ring0 buffers during the end of collection flushing of data to user mode is avoided
//  by allocating and deallocating all buffers during driver load and unload
ssize_t device_read(struct file *file,	/* see include/linux/fs.h   */
			   char __user * buffer,	/* buffer to be
							 * filled with data */
			   size_t length,	/* length of the buffer     */
			   loff_t * offset)
{
  unsigned int to_copy;
  unsigned long uncopied;
  uint64_t cur_buf;
  int cpu;
  int wei_ret;
  int ret;
  bool vmm_buf_rdy=false, linux_buf_rdy=false;
  pw_data_buffer_t *data_buffer;

  // printk(KERN_INFO "sofia-proto: device_read function waiting, user buf %p, size %zu bytes, flush_now=%d\n", buffer, length, flush_now);

  if (flush_now) {
    vmm_buf_rdy = vmm_is_buffer_ready(&cpu);
    linux_buf_rdy = is_linux_kernel_buffer_ready (flush_now, &data_buffer);
  }
  else {
    wei_ret = wait_event_interruptible(read_queue, (vmm_buf_rdy=vmm_is_buffer_ready(&cpu)) || (linux_buf_rdy=is_linux_kernel_buffer_ready (flush_now, &data_buffer)) );
    if (wei_ret != 0) {
      printk(KERN_INFO "sofia-proto: device_read() wait_event_interruptible failed\n");
      return -1;
    }
  }

  // printk(KERN_INFO "sofia-proto: device_read()- flush_now=%d, vmm_buf_rdy=%d, linux_buf_rdy=%d\n", flush_now, vmm_buf_rdy, linux_buf_rdy);

  if (vmm_buf_rdy) {
    cur_buf = (uint64_t)(unsigned long)phys_to_virt((unsigned long)pBufferInfo->buffer_delivered[cpu]);
    to_copy = pBufferInfo->buffer_data_size[cpu];

    // printk(KERN_INFO "sofia-proto: device_read-received vmm buffer for core %d at %p, with %d bytes\n", cpu, (void *)(unsigned long)cur_buf, to_copy);

    if (to_copy > length) {
      printk(KERN_INFO "sofia-proto: user buffer is too small\n");
      return -1;
    }

    uncopied = copy_to_user(buffer, (void *)(unsigned long)cur_buf, to_copy);
    *offset += to_copy-uncopied;

    if (uncopied) {
      printk(KERN_INFO "sofia-proto: device_read() failed to copy %ld bytes\n", uncopied);
      // copy_to_user returns an unsigned
      return -1;
    }
    else {
      // Mark the buffer empty
      pBufferInfo->buffer_status[cpu] = SOCWATCH_BUFFER_CONSUMED;
      pBufferInfo->buffer_data_size[cpu] = 0;
    }

    // printk(KERN_INFO "sofia-proto: device_read() copied %d bytes to usermode\n", to_copy);
    return to_copy;
  }

  if (linux_buf_rdy) {
    // printk(KERN_INFO "sofia-proto: device_read-received linux buffer at %p, with %d bytes\n", (void *)data_buffer, data_buffer->bytes_written);
    if (data_buffer->bytes_written > length) {
      printk(KERN_INFO "sofia-proto: user buffer is too small\n");
      return -1;
    }
    uncopied = copy_to_user(buffer, data_buffer->buffer, data_buffer->bytes_written);
    // add 2*sizeof(int) to the data_buffer start address to skip past the buffer header data
//    uncopied = copy_to_user(buffer, (void *)(data_buffer + 2*(sizeof(int))), data_buffer->bytes_written);
    *offset += data_buffer->bytes_written-uncopied;

    if (uncopied) {
      printk(KERN_INFO "sofia-proto: device_read() failed to copy %ld bytes\n", uncopied);
      // return 0 to force the usermode code to stop 'reading'
      return -1;
    }
    else {
      //printk(KERN_INFO "sofia-proto: device_read() copied %d bytes to usermode\n", data_buffer->bytes_written);
      ret = data_buffer->bytes_written;
      // Mark the buffer empty
      data_buffer->bytes_written = 0;
      data_buffer->is_full = 0;
      return ret;
    }
  }

  // nothing  to copy, return 0
  return 0;
};



static irqreturn_t socwatch_irq_handler(int irq, void *dev_id)
{
  // num_interrupts++;
  // printk(KERN_INFO "sofia-proto: total injected interrupts %d\n", num_interrupts);

  wake_up_interruptible(&read_queue);

  return IRQ_HANDLED;
}


static long
spdrv_Configure(int config_bitmap)
{
//  unsigned long long tsc=readSTM();
//  printk(KERN_INFO "sofia-proto: configure() readSTM=%llu\n", tsc);

  flush_now = false;
  vmm_reset_buffers();
  pw_reset_per_cpu_buffers();
  // enable all available metrics with the first parameter for now
  // TODO get requested metrics from user mode and set corresponding bits
  mv_svc_socwatch_config(config_bitmap, ((struct socwatch_buffer_info *)pPhysBufferInfo));
//  mv_svc_socwatch_config(0xFF, ((struct socwatch_buffer_info *)pPhysBufferInfo));
//  mv_svc_socwatch_config(0x1, ((struct socwatch_buffer_info *)pPhysBufferInfo));   // cstate only data collected
//  mv_svc_socwatch_config(0x20, ((struct socwatch_buffer_info *)pPhysBufferInfo));  // vm-switch only data collected

  return 0;
}

static long
spdrv_Stop(void)
{
  mv_svc_socwatch_run_control(0);
  // stop he linux kernel from generaing data
  if (enable_tracepoints) {
    unregister_with_tracepoints();
    enable_tracepoints = 0;
  }
  // set the flag that indictes partially filled linux buffers should be
  //  returned to user mode via the device_read function

  //unsigned long long tsc=readSTM();
  //printk(KERN_INFO "sofia-proto: stop() readSTM=%llu\n", tsc);

  flush_now = true;

  // printk(KERN_INFO "sofia-proto: collection stopped\n");
  // force the device_read function to check if any buffers are partially filled with data
  wake_up_interruptible(&read_queue);
  delete_all_non_kernel_timers();
  // print_tracepoint_activity();
  // num_interrupts = 0;

  return 0;
}

static long
spdrv_Start(int config_bitmap)
{
  int ret;
  //unsigned long long tsc=readSTM();
  //printk(KERN_INFO "sofia-proto: start() readSTM=%llu\n", tsc);

  if (config_bitmap) {
    ret = check_if_tracepoints_enabled();
    if (!ret) {
      register_with_tracepoints();
      enable_tracepoints = 1;
    }
  }

  mv_svc_socwatch_run_control(1);

  return 0;
}

long
spdrv_GetVersion(struct sp_driver_version_info *return_version, int size)
{
    struct sp_driver_version_info local_version;

    if (!return_version) {
        printk(KERN_INFO "sofia-proto: spdrv_GetVersion() ERROR: NULL return_version_struct pointer!\n");
        return -1;
    }

    local_version.major = SPDRV_VERSION_MAJOR;
    local_version.minor = SPDRV_VERSION_MINOR;
    local_version.other = SPDRV_VERSION_OTHER;

    /*
     * Copy everything back to user address space.
     */
    return copy_to_user(return_version, &local_version, size); // returns number of bytes that could NOT be copiled
};

static long handle_ioctl(unsigned int num, unsigned long ioctl_param)
{
  long status = 0;
  struct spdrv_ioctl_arg* ioctl_args = (struct spdrv_ioctl_arg*)ioctl_param;
  int size, config_bitmap;
  struct sp_driver_version_info * version;

    // printk(KERN_INFO "sofia-proto: handling ioctl %d\n", num);

    switch (num) {
        case DRV_OPERATION_START:
            // printk(KERN_INFO "sofia-proto: start collection\n");
            config_bitmap = ioctl_args->in_len;  // use the in_len field as a bitmap for configuring the Linux kernel collection
            status = spdrv_Start(config_bitmap);
            break;

        case DRV_OPERATION_STOP:
            // printk(KERN_INFO "sofia-proto: stop collection\n");
            status = spdrv_Stop();
            break;

        case DRV_OPERATION_CONFIGURE:
            // printk(KERN_INFO "sofia-proto: configure\n");
            config_bitmap = ioctl_args->in_len;  // use the in_len field as a bitmap for configuring the VMM collection
                                                 // the bits are passed directly to the VMM's configure VMCall function
            status = spdrv_Configure(config_bitmap);
            break;
        case DRV_OPERATION_VERSION:
            // printk(KERN_INFO "sofia-proto: version\n");
            size = ioctl_args->out_len;
//            printk(KERN_INFO "sofia-proto: version size = 0x%x bytes\n", size);
            version = (struct sp_driver_version_info *)ioctl_args->out_arg;
//            printk(KERN_INFO "sofia-proto: input version is %d.%d.%d\n", version->major, version->minor, version->other);
            status = spdrv_GetVersion(version, size);
//            printk(KERN_INFO "sofia-proto: output version is %d.%d.%d\n", version->major, version->minor, version->other);
            break;
    }
  return status;
}

static long device_unlocked_ioctl(struct file *filep, unsigned int ioctl_num, unsigned long ioctl_param)
{
  long  status = 0;
  handle_ioctl(_IOC_NR(ioctl_num), ioctl_param);
  return status;
};


#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
long device_compat_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_param)
{
  handle_ioctl(_IOC_NR(ioctl_num), ioctl_param);
  return 0;
};
#endif // COMPAT && x64


static int device_open(struct inode *inode, struct file *file)
{
  // printk(KERN_INFO "sofia-proto:  driver has been opened\n");

  flush_now = false;
  pw_reset_per_cpu_buffers();
  vmm_reset_buffers();

  return 0;
}

struct file_operations Fops = {
    .open = &device_open,
    .read = &device_read,
    .unlocked_ioctl = &device_unlocked_ioctl,
#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
    .compat_ioctl = &device_compat_ioctl,
#endif // COMPAT && x64
};


static int __init initSP( void )
{
  int initSTM, error;
  struct device *dev;

  initSTM = mapSTM();
  if (initSTM) {
    printk(KERN_INFO "sofia-proto: unable to ioremap the STM clock\n");
    error = -EIO;
    goto cleanup_return_error;
  }

  num_CPUs = num_possible_cpus();

  // init all of the data buffers
  error = vmm_init_buffers(num_CPUs);
  if (error) {
    printk(KERN_INFO "sofia-proto: Error failed to allocate vmm buffers\n");
    goto cleanup_return_error;
  }
  error = pw_init_per_cpu_buffers();
  if (error) {
    printk(KERN_INFO "sofia-proto: Error failed to allocate linux kernel buffers\n");
    goto cleanup_return_error;
  }

  // create the char device "sp"
  alloc_chrdev_region(&sp_dev, 0, 1, "sp");
  sp_dev_major_num = MAJOR(sp_dev);
  sp_class = class_create(THIS_MODULE, "sp");
  if(IS_ERR(sp_class)) {
    error = PTR_ERR(sp_class);
    printk(KERN_INFO "sofia-proto: Error registering sp class\n");
    goto cleanup_return_error;
  }

  dev = device_create(sp_class, NULL, sp_dev, NULL, "sp");
  if (dev == NULL) {
    error = PTR_ERR(dev);
    printk(KERN_INFO "sofia-proto: Error during call to device_create\n");
    goto cleanup_return_error;
  }

  sp_cdev = cdev_alloc();
  if (sp_cdev == NULL) {
    error = -ENOMEM;
    printk(KERN_INFO "sofia-proto: Error allocating character device\n");
    goto cleanup_return_error;
  }
  sp_cdev->owner = THIS_MODULE;
  sp_cdev->ops = &Fops;
  if( cdev_add(sp_cdev, sp_dev, 1) < 0 )  {
    error = -1;
    printk(KERN_INFO "sofia-proto: Error registering device driver\n");
    goto cleanup_return_error;
  }

  // printk(KERN_INFO "sofia-proto: registering sp class\n");
  // initialize a work queue to be used for signalling when data is ready to copy to usermode
  init_waitqueue_head(&read_queue);

  // register IRQ handler
////  printk(KERN_INFO "sofia-proto: request socwatch irq\n");
  error = request_irq(SOCWATCH_IRQ,
                      socwatch_irq_handler,
                      0,
                      "socwatch",
                      NULL);
  if (error) {
    error = -EBUSY;
    printk(KERN_INFO "sofia-proto: Error request_irq() failed during driver init\n");
    goto cleanup_return_error;
  }

  return 0;

  cleanup_return_error:
  // deregister irq
  free_irq(SOCWATCH_IRQ, NULL);
  // release char device
  unregister_chrdev(sp_dev_major_num, "sp");
  device_destroy(sp_class, sp_dev);
  class_destroy(sp_class);
  unregister_chrdev_region(sp_dev, 1);
  cdev_del(sp_cdev);

  vmm_destroy_buffers();
  pw_destroy_per_cpu_buffers();
  unmapSTM();
  return error;
}

static void __exit exitSP( void )
{
  // deregister irq
  free_irq(SOCWATCH_IRQ, NULL);
  unmapSTM();

  vmm_destroy_buffers();
  pw_destroy_per_cpu_buffers();
  // release char device
  unregister_chrdev(sp_dev_major_num, "sp");
  device_destroy(sp_class, sp_dev);
  class_destroy(sp_class);
  unregister_chrdev_region(sp_dev, 1);
  cdev_del(sp_cdev);

//  printk(KERN_INFO "sofia-proto: unregistering sp class\n");

  return;
}

module_init(initSP);
module_exit(exitSP);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(MOD_AUTHOR);
MODULE_DESCRIPTION(MOD_DESC);
