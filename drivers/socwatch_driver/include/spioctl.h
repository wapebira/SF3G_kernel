
/*
 * Structure to return version information.
 */
struct sp_driver_version_info {
    int major;
    int minor;
    int other;
};

struct spdrv_ioctl_arg {
    int in_len;
    int out_len;
    const char *in_arg;
    char *out_arg;
};

#define SP_IOC_MAGIC          99
#define DRV_OPERATION_START       1
#define DRV_OPERATION_STOP        2
#define DRV_OPERATION_CONFIGURE   3
#define DRV_OPERATION_VERSION     4
#define SPDRV_IOCTL_START     _IO (SP_IOC_MAGIC,  DRV_OPERATION_START)
#define SPDRV_IOCTL_STOP      _IO (SP_IOC_MAGIC,  DRV_OPERATION_STOP)
#define SPDRV_IOCTL_CONFIGURE _IO (SP_IOC_MAGIC,  DRV_OPERATION_CONFIGURE)
#define SPDRV_IOCTL_VERSION   _IOR (SP_IOC_MAGIC,  DRV_OPERATION_VERSION, struct sp_driver_version_info *)

#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
  #include <linux/compat.h>

  #define SPDRV_IOCTL_START     _IO (SP_IOC_MAGIC,  DRV_OPERATION_START)
  #define SPDRV_IOCTL_STOP      _IO (SP_IOC_MAGIC,  DRV_OPERATION_STOP)
  #define SPDRV_IOCTL_CONFIGURE _IO (SP_IOC_MAGIC,  DRV_OPERATION_CONFIGURE)
  #define SPDRV_IOCTL_VERSION   _IOR (SP_IOC_MAGIC,  DRV_OPERATION_VERSION, compat_uptr_t)
#endif // COMPAT && x64
