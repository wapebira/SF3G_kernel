#include <asm/io.h>
#include <asm/cputime.h>
#include <linux/interrupt.h>
#include <trace/events/timer.h>
#include <trace/events/power.h>
#include <trace/events/irq.h>
#include <trace/events/sched.h>
struct pool_workqueue; // Get rid of warnings regarding trace_workqueue
#include <trace/events/workqueue.h>
#include <trace/events/syscalls.h>
#include <linux/types.h>
#include <linux/hash.h>
#include <linux/slab.h>

#include "sp.h"
#include "sp_linux_buffer.h"
#include "sp_linux_wakeup_handler.h"
#include "sp_linux_timer_handler.h"
#include "sp_linux_irq_handler.h"


/*
  * Names for SOFTIRQs.
  * These are taken from "include/linux/interrupt.h"
  */
static const char *pw_softirq_to_name[] = {"HI_SOFTIRQ", "TIMER_SOFTIRQ", "NET_TX_SOFTIRQ", "NET_RX_SOFTIRQ", "BLOCK_SOFTIRQ", "BLOCK_IOPOLL_SOFTIRQ", "TA    SKLET_SOFTIRQ", "SCHED_SOFTIRQ", "HRTIMER_SOFTIRQ", "RCU_SOFTIRQ"};


/*
 * Per-cpu structure holding wakeup event causes, tscs
 * etc. Set by the first non-{TPS, TPE} event to occur
 * after a processor wakes up.
 */
struct wakeup_event {
  unsigned long long event_tsc; // TSC at which the event occurred
  unsigned long long event_val; // Event value -- domain-specific
  int init_cpu; // CPU on which a timer was initialized; valid ONLY for wakeups caused by timers!
  int event_type; // one of c_break_type_t enum values
  pid_t event_tid, event_pid;
  int first_sample_collected; // initialized to 0, set to 1 after the first sample is collected
};

/*
 * Used to record which wakeup event occured first.
 * Reset on every TPS.
 */
static DEFINE_PER_CPU_SHARED_ALIGNED(struct wakeup_event, wakeup_event_counter) = {0, 0, -1, PW_BREAK_TYPE_U, -1, -1, 0};

static atomic_t ci_epoch = ATOMIC_INIT(0);

#define record_wakeup_cause(tsc, type, value, cpu, pid, tid) do { \
    struct wakeup_event *wu_event = &get_cpu_var(wakeup_event_counter); \
    bool is_first_wakeup_event = CAS64(&wu_event->event_tsc, 0, (tsc)); \
    if (is_first_wakeup_event) { \
        wu_event->event_val = (value); \
        wu_event->init_cpu = (cpu); \
        wu_event->event_type = (type); \
        wu_event->event_tid = (tid); \
        wu_event->event_pid = (pid); \
    } \
    put_cpu_var(wakeup_event_counter); \
} while(0)

void * pSTM;
int stmReads = 0;

int tracepoints_fired_timer_init;
int tracepoints_fired_hrtimer_init;
int tracepoints_fired_itimer_state;
int tracepoints_fired_timer;
int tracepoints_fired_hrtimer;
int tracepoints_fired_hrtimer_expire_exit;
int tracepoints_fired_irq;
int tracepoints_fired_softirq;
int tracepoints_fired_sched;
int tracepoints_fired_workqueue;
int tracepoints_fired_cpu_idle;
int tracepoints_fired_sched_process_fork;
int tracepoints_fired_sched_process_exit;
int irq_map_samples_produced;
int proc_map_samples_produced;
int sched_samples_produced;
int tps_samples_produced;


// sofia 3G STM base is 0xE4300000, size is 0x40
// mapSTM returns 0 if it worked, non-zero if it failed
int mapSTM (void)
{
  int ret = 0;
  pSTM = ioremap(STM_base, STM_size);
  // printk(KERN_INFO "sofia-proto mapSTM() pSTM=%p\n", pSTM);
  if (pSTM == NULL)
    return 1;
  return ret;
};

void unmapSTM (void)
{
  iounmap((void*)STM_base);
  stmReads = 0;
};

unsigned long long readSTM(void)
{
//  return 0;

  unsigned int lower32, lower32_2, upper24;
  void *addressl = pSTM + STM_TIM0_offset;
  void *addressh = pSTM + STM_CAP_offset;

  lower32 = ioread32(addressl);
  upper24 = ioread32(addressh);
  lower32_2 = ioread32(addressl);
  upper24 &= 0x0000ffff;
  if (stmReads <= 10) {
    // printk(KERN_INFO "sofia-proto readSTM() upper24=0x%x, lower32=0x%x, lower32_2=0x%x\n", upper24, lower32, lower32_2);
    stmReads++;
  }
  if (lower32 <= lower32_2)
    return (((unsigned long long) upper24 << 32) | lower32);
  else {
    // handle overflow
    upper24 = ioread32(addressh);
    upper24 &= 0x0000ffff;
    return (((unsigned long long) upper24 << 32) | lower32_2);
  }

};


int inc_ci_epoch_i(void)
{
  int retVal = -1;
  /*
   * From "Documentation/memory-barriers.txt": "atomic_inc_return()"
   * has IMPLICIT BARRIERS -- no need to add explicit barriers
   * here!
   */
  retVal = atomic_inc_return(&ci_epoch);
  return retVal;
};

int read_ci_epoch_i(void)
{
  /*
   * Make sure TPS updates have propagated
   */
  smp_mb();
  return atomic_read(&ci_epoch);
};

static void probe_hrtimer_init(void *ignore, struct hrtimer *timer, clockid_t clockid, enum hrtimer_mode mode)
{
  tracepoints_fired_hrtimer_init++;
  timer_init(timer);
};

static void probe_timer_init(void *ignore, struct timer_list *timer)
{
  tracepoints_fired_timer_init++;
  timer_init(timer);
};

static void probe_itimer_state(void *ignore, int which, const struct itimerval *const value, cputime_t expires)
{
  struct hrtimer *timer = &current->signal->real_timer;

  tracepoints_fired_itimer_state++;
  timer_init(timer);
};


static void probe_timer_expire_entry(void *ignore, struct timer_list *t)
{
  pid_t return_pid;
  int return_init_cpu;
  unsigned long long sample_tsc = 0;
  pid_t tid = TIMER_START_PID(t);

  tracepoints_fired_timer++;
  timer_expire(t, tid, &return_pid, &return_init_cpu);

  sample_tsc = readSTM();
  record_wakeup_cause(sample_tsc, PW_BREAK_TYPE_T, 0, return_init_cpu, return_pid, tid);

  return;
};

static void probe_hrtimer_expire_entry(void *ignore, struct hrtimer *hrt, ktime_t *now)
{
  pid_t return_pid;
  int return_init_cpu;
  unsigned long long sample_tsc = 0;
  pid_t tid = TIMER_START_PID(hrt);

  tracepoints_fired_hrtimer++;
  timer_expire(hrt, tid, &return_pid, &return_init_cpu);

  sample_tsc = readSTM();
  record_wakeup_cause(sample_tsc, PW_BREAK_TYPE_T, 0, return_init_cpu, return_pid, tid);
  return;
};


static void probe_hrtimer_expire_exit(void *ignore, struct hrtimer *hrt)
{
  tracepoints_fired_hrtimer_expire_exit++;
  if(!IS_INTERVAL_TIMER(hrt)) {
    timer_delete((unsigned long)hrt, TIMER_START_PID(hrt));
  }
  return;
};


static void irq_common(int irq_num, const char *irq_name)
{
  unsigned long long sample_tsc = 0;

  bool was_hit =  __get_cpu_var(wakeup_event_counter).event_tsc == 0;

  sample_tsc = readSTM();
  record_wakeup_cause(sample_tsc, PW_BREAK_TYPE_I, irq_num, -1, -1, -1);
  if (was_hit)
    handle_irq_wakeup_i(CPU(), irq_num, irq_name, sample_tsc);
};

static void probe_irq_handler_entry(void *ignore, int irq, struct irqaction *action)
{
  const char *name = action->name;
  tracepoints_fired_irq++;

  irq_common(irq, name);
  return;
};

static void probe_softirq_entry(void *ignore, unsigned int vec_nr)
{
  int irq = (int)vec_nr;
  const char *name;

  tracepoints_fired_softirq++;

  name = pw_softirq_to_name[irq];
  irq_common(irq, name);

  return;
};

static void probe_sched_wakeup(void *ignore, struct task_struct *task, int success)
{
  int target_cpu = task_cpu(task), source_cpu = CPU();
  struct PWCollector_msg output_sample;
  event_sample_t event_sample;
  unsigned long long sample_tsc = 0;

  tracepoints_fired_sched++;

  // "Self-sched" samples are "don't care".
  if (target_cpu != source_cpu) {
    sample_tsc = readSTM();
    output_sample.cpuidx = source_cpu | LINUX_MESSAGE_CPU_MSB;
    output_sample.tsc = sample_tsc;

    event_sample.data[0] = source_cpu;
    event_sample.data[1] = target_cpu;

    event_sample.data[2] = read_ci_epoch_i();

    output_sample.data_type = SCHED_SAMPLE;
    output_sample.data_len = sizeof(event_sample);
    output_sample.p_data = (unsigned long long)((unsigned long)&event_sample);

    pw_produce_generic_msg(&output_sample, false); // "false" ==> don't wake any sleeping readers (required from scheduling context)
    sched_samples_produced++;
  }

  return;
};

static void probe_workqueue_execute_start(void *ignore, struct work_struct *work)
{
  unsigned long long tsc = readSTM();
  tracepoints_fired_workqueue++;
  record_wakeup_cause(tsc, PW_BREAK_TYPE_W, 0, -1, -1, -1);

  return;
};

static void probe_cpu_idle(void *ignore, unsigned int state, unsigned int cpu_id)
{
  int epoch;
  int first_sample;
  struct wakeup_event *wu_event;
  unsigned long long tsc; // TSC at which the event occurred
  unsigned long long val; // domain-specific
  int init_cpu; // valid ONLY for wakeups caused by timers!
  int type; // one of c_break_type_t enum values
  pid_t tid, pid;
  struct timespec ts;
  tsc_posix_sync_msg_t tsc_msg;
  unsigned long long tmp_tsc = 0;
  unsigned long long tmp_nsecs = 0;
  unsigned long long sample_tsc = 0;
  PWCollector_msg_t sample;
  int cpu = raw_smp_processor_id();
  c_multi_msg_t cm;

  if (state == PWR_EVENT_EXIT) {
    return;
  }

  tracepoints_fired_cpu_idle++;

  sample_tsc= readSTM();
  wu_event = &get_cpu_var(wakeup_event_counter);
  first_sample = wu_event->first_sample_collected;
  type = wu_event->event_type;
  val = wu_event->event_val;
  tsc = wu_event->event_tsc;
  pid = wu_event->event_pid;
  tid = wu_event->event_tid;
  init_cpu = wu_event->init_cpu;

  wu_event->event_tsc = 0; // reset for the next wakeup event.
  wu_event->first_sample_collected = 1;
  put_cpu_var(wakeup_event_counter);

  if (unlikely(first_sample == 0)) {
    ktime_get_ts(&ts);
    tmp_tsc= readSTM();
    tmp_nsecs = (unsigned long long)ts.tv_sec * 1000000000ULL + (unsigned long long)ts.tv_nsec;
    tsc_msg.tsc_val = tmp_tsc;
    tsc_msg.posix_mono_val = tmp_nsecs;

    sample.tsc = sample_tsc;
    sample.cpuidx = cpu | LINUX_MESSAGE_CPU_MSB;
    sample.data_type = TSC_POSIX_MONO_SYNC;
    sample.data_len = sizeof(tsc_msg);
    sample.p_data = (unsigned long long)((unsigned long)&tsc_msg);

    pw_produce_generic_msg(&sample, true);
    // printk(KERN_INFO "sofia-proto probe_cpu_idle() cpu %d: SENT POSIX_TIME_SYNC\n", cpu);
//    pw_pr_debug(KERN_INFO "[%d]: tsc = %llu posix mono = %llu\n", cpu, tmp_tsc, tmp_nsecs);
  }

  cm.mperf = 0;
  cm.wakeup_tsc = tsc;
  cm.wakeup_data = val;
  cm.wakeup_pid = pid;
  cm.wakeup_tid = tid;
  cm.wakeup_type = type;
  cm.timer_init_cpu = init_cpu;
  cm.req_state = -1;
  cm.num_msrs = 0;
  cm.data[0] = 0;

  // We're entering a new TPS "epoch".
  //  Increment our counter.
  epoch = inc_ci_epoch_i();
  cm.tps_epoch  = epoch;

  sample.cpuidx = cpu | LINUX_MESSAGE_CPU_MSB;
  sample.tsc = sample_tsc;
  sample.data_type = C_STATE;
  sample.data_len = sizeof(c_multi_msg_t);
  sample.p_data = (unsigned long long)((unsigned long)&cm);

  pw_produce_generic_msg(&sample, true);

  /*
   * Reset the "first-hit" variable.
   */
  __get_cpu_var(wakeup_event_counter).event_tsc = 0;

  tps_samples_produced++;

  return;
};

#if 0
static void probe_sys_exit(void *ignore, struct pt_regs *regs, long ret)
{
  long id = syscall_get_nr(current, regs);
  pid_t tid = current->pid;
  pid_t pid = current->tgid;

  tracepoints_fired_sys_exit++;
    DO_PER_CPU_OVERHEAD_FUNC(sys_exit_helper_i, id, tid, pid);

    if(id == __NR_execve && IS_COLLECTING()){
        u64 tsc;

        tscval(&tsc);
        OUTPUT(3, KERN_INFO "[%d]: EXECVE ENTER! TID = %d, NAME = %.20s\n", CPU(), TID(), NAME());
        produce_r_sample(CPU(), tsc, PW_PROC_EXEC, TID(), PID(), NAME());
    }
current->comm
};
#endif


static void probe_sched_process_exit(void *ignore, struct task_struct *task)
{
  pid_t tid = task->pid;
  pid_t pid = task->tgid;
  const char *name = task->comm;
  unsigned long long sample_tsc = 0;

  tracepoints_fired_sched_process_exit++;
  delete_timers_for_tid(tid);
  sample_tsc= readSTM();
  produce_r_sample(raw_smp_processor_id(), sample_tsc, PW_PROC_EXIT, tid, pid, name);
};

static void probe_sched_process_fork(void *ignore, struct task_struct *parent, struct task_struct *child)
{
  const char *cname = child->comm;
  pid_t ctid = child->pid;
  pid_t cpid = child->tgid;
  unsigned long long sample_tsc = 0;

  tracepoints_fired_sched_process_fork++;

  sample_tsc= readSTM();
  produce_r_sample(raw_smp_processor_id(), sample_tsc, PW_PROC_FORK, ctid, cpid, cname);
};

void print_tracepoint_activity(void)
{
  printk(KERN_INFO "sofia-proto: TRACEPOINT ACTIVITY\n");
  printk(KERN_INFO "sofia-proto: %d timer_init tracepoints have fired\n", tracepoints_fired_timer_init);
  printk(KERN_INFO "sofia-proto: %d hrtimer_init tracepoints have fired\n", tracepoints_fired_hrtimer_init);
  printk(KERN_INFO "sofia-proto: %d itimer_state tracepoints have fired\n", tracepoints_fired_itimer_state);
  printk(KERN_INFO "sofia-proto: %d timer_expire tracepoints have fired\n", tracepoints_fired_timer);
  printk(KERN_INFO "sofia-proto: %d hrtimer_expire tracepoints have fired\n", tracepoints_fired_hrtimer);
  printk(KERN_INFO "sofia-proto: %d hrtimer_expire_exit tracepoints have fired\n", tracepoints_fired_hrtimer_expire_exit);
  printk(KERN_INFO "sofia-proto: %d irq tracepoints have fired\n", tracepoints_fired_irq);
  printk(KERN_INFO "sofia-proto: %d softirq tracepoints have fired\n", tracepoints_fired_softirq);
  printk(KERN_INFO "sofia-proto: %d sched tracepoints have fired\n", tracepoints_fired_sched);
  printk(KERN_INFO "sofia-proto: %d workqueue tracepoints have fired\n", tracepoints_fired_workqueue);
  printk(KERN_INFO "sofia-proto: %d cpu_idle tracepoints have fired\n", tracepoints_fired_cpu_idle);
  printk(KERN_INFO "sofia-proto: %d sched_process_fork tracepoints have fired\n", tracepoints_fired_sched_process_fork);
  printk(KERN_INFO "sofia-proto: %d sched_process_exit tracepoints have fired\n", tracepoints_fired_sched_process_exit);
//  printk(KERN_INFO "sofia-proto: %d sys_exit tracepoints have fired\n", tracepoints_fired_sys_exit);

  printk(KERN_INFO "sofia-proto: %d irq_map msgs produced\n", irq_map_samples_produced);
  printk(KERN_INFO "sofia-proto: %d proc_map msgs produced\n", proc_map_samples_produced);
  printk(KERN_INFO "sofia-proto: %d sched wakeup msgs produced\n", sched_samples_produced);
  printk(KERN_INFO "sofia-proto: %d tps msgs produced\n", tps_samples_produced);
};

int unregister_with_tracepoints(void)
{
  unregister_trace_timer_expire_entry(probe_timer_expire_entry, NULL);
  unregister_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry, NULL);
  unregister_trace_hrtimer_expire_exit(probe_hrtimer_expire_exit, NULL);
  unregister_trace_hrtimer_init(probe_hrtimer_init, NULL);
  unregister_trace_timer_init(probe_timer_init, NULL);
  unregister_trace_itimer_state(probe_itimer_state, NULL);
  unregister_trace_irq_handler_entry(probe_irq_handler_entry, NULL);
  unregister_trace_softirq_entry(probe_softirq_entry, NULL);
  unregister_trace_sched_wakeup(probe_sched_wakeup, NULL);
  unregister_trace_workqueue_execute_start(probe_workqueue_execute_start, NULL);
  unregister_trace_sched_process_exit(probe_sched_process_exit, NULL);
  unregister_trace_sched_process_fork(probe_sched_process_fork, NULL);
  unregister_trace_cpu_idle(probe_cpu_idle, NULL);

//  unregister_trace_sys_exit(probe_sys_exit, NULL);

  tracepoint_synchronize_unregister();
  destroy_timer_map();
  destroy_irq_map();
  return 0;
};

#define TRACEPOINT_FAILED_TIMER_EXPIRE 0x1
#define TRACEPOINT_FAILED_HRTIMER_EXPIRE 0x2
#define TRACEPOINT_FAILED_IRQ_ENTRY 0x4
#define TRACEPOINT_FAILED_SOFTIRQ_ENTRY 0x8
#define TRACEPOINT_FAILED_SCHED_WAKEUP 0x10
#define TRACEPOINT_FAILED_WORKQUEUE_START 0x20
#define TRACEPOINT_FAILED_SCHED_PROCESS_FORK 0x40
#define TRACEPOINT_FAILED_SCHED_PROCESS_EXIT 0x80
#define TRACEPOINT_FAILED_SYS_EXIT 0x100
#define TRACEPOINT_FAILED_TIMER_INIT 0x200
#define TRACEPOINT_FAILED_HRTIMER_INIT 0x400
#define TRACEPOINT_FAILED_ITIMER_STATE 0x800
#define TRACEPOINT_FAILED_HRTIMER_EXPIRE_EXIT 0x1000


int register_with_tracepoints(void)
{
  int ret = 0;
  int cpu;
  int break_tracepoint_registration = 0;

  // debug code
  tracepoints_fired_timer = 0;
  tracepoints_fired_hrtimer = 0;
  tracepoints_fired_irq = 0;
  tracepoints_fired_softirq = 0;
  tracepoints_fired_sched = 0;
  tracepoints_fired_workqueue = 0;
  tracepoints_fired_cpu_idle = 0;

  irq_map_samples_produced = 0;
  proc_map_samples_produced = 0;
  sched_samples_produced = 0;
  tps_samples_produced = 0;

  // reset the cpu_idle epoch
  atomic_set(&ci_epoch, 0);
  init_irq_map();
  init_timer_map();
  for_each_online_cpu(cpu) {
    per_cpu(wakeup_event_counter, cpu).event_tsc = 0;
    per_cpu(wakeup_event_counter, cpu).first_sample_collected = 0;
  }

  ret = register_trace_timer_expire_entry(probe_timer_expire_entry, NULL);
  if (ret)
    break_tracepoint_registration |= TRACEPOINT_FAILED_TIMER_EXPIRE;
  ret = register_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry, NULL);
  if (ret)
    break_tracepoint_registration |= TRACEPOINT_FAILED_HRTIMER_EXPIRE;
  ret = register_trace_hrtimer_expire_exit(probe_hrtimer_expire_exit, NULL);
  if (ret)
    break_tracepoint_registration |= TRACEPOINT_FAILED_HRTIMER_EXPIRE_EXIT;
  ret = register_trace_hrtimer_init(probe_hrtimer_init, NULL);
  if (ret)
    break_tracepoint_registration |= TRACEPOINT_FAILED_HRTIMER_INIT;
  ret = register_trace_timer_init(probe_timer_init, NULL);
  if (ret)
    break_tracepoint_registration |= TRACEPOINT_FAILED_TIMER_INIT;
  ret = register_trace_itimer_state(probe_itimer_state, NULL);
  if (ret)
    break_tracepoint_registration |= TRACEPOINT_FAILED_ITIMER_STATE;
  ret = register_trace_irq_handler_entry(probe_irq_handler_entry, NULL);
  if (ret)
    break_tracepoint_registration |= TRACEPOINT_FAILED_IRQ_ENTRY;
  ret = register_trace_softirq_entry(probe_softirq_entry, NULL);
  if (ret)
    break_tracepoint_registration |= TRACEPOINT_FAILED_SOFTIRQ_ENTRY;
  ret = register_trace_sched_wakeup(probe_sched_wakeup, NULL);
  if (ret)
    break_tracepoint_registration |= TRACEPOINT_FAILED_SCHED_WAKEUP;
  ret = register_trace_workqueue_execute_start(probe_workqueue_execute_start, NULL);
  if (ret)
    break_tracepoint_registration |= TRACEPOINT_FAILED_WORKQUEUE_START;
  ret = register_trace_sched_process_exit(probe_sched_process_exit, NULL);
  if (ret)
    break_tracepoint_registration |= TRACEPOINT_FAILED_SCHED_PROCESS_EXIT;
  ret = register_trace_sched_process_fork(probe_sched_process_fork, NULL);
  if (ret)
    break_tracepoint_registration |= TRACEPOINT_FAILED_SCHED_PROCESS_FORK;
////  ret = register_trace_sys_exit(probe_sys_exit, NULL);
////  if (ret)
////    break_tracepoint_registration |= TRACEPOINT_FAILED_SYS_EXIT;


  if (break_tracepoint_registration)
    printk(KERN_INFO "sofia-proto: WARNING; failed to register one or more cstate break tracepoints [0x%x]\n",break_tracepoint_registration );

  ret = register_trace_cpu_idle(probe_cpu_idle, NULL);
  if (ret)
    printk(KERN_INFO "sofia-proto: ERROR: failed to register with the cpu_idle tracepoint\n");
  return ret;
};


int check_if_tracepoints_enabled(void)
{
#ifdef CONFIG_TRACEPOINTS
    // printk (KERN_INFO "sofia-proto: Tracepoints are enabled!\n");
#else
    // printk(KERN_INFO "sofia-proto: ERROR: kernel tracepoints are not enabled - bummer\n");
    return 1;
#endif
   return 0;
};


