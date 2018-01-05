#ifndef _SP_LINUX_TIMER_HANDLER_H_
#define _SP_LINUX_TIMER_HANDLER_H_ 1

#ifdef CONFIG_TIMER_STATS
  #define TIMER_START_PID(t) ( (t)->start_pid )
  #define TIMER_START_COMM(t) ( (t)->start_comm )
#else
  #define TIMER_START_PID(t) (-1)
  #define TIMER_START_COMM(t) ( "UNKNOWN" )
#endif


  /*
   * Macro to determine if the given
   * high resolution timer is periodic.
   */
#define IS_INTERVAL_TIMER(hrt) ({                                       \
             bool __tmp = false;                                         \
             pid_t pid = TIMER_START_PID(hrt);                           \
             ktime_t rem_k = hrtimer_expires_remaining(hrt);             \
             s64 remaining = rem_k.tv64;                                 \
             /* We first account for timers that */                      \
             /* are explicitly re-enqueued. For these */                 \
             /* we check the amount of time 'remaining' */               \
             /* for the timer i.e.  how much time until */               \
             /* the timer expires. If this is POSITIVE ==> */            \
             /* the timer will be re-enqueued onto the */                \
             /* timer list and is therefore PERIODIC */                  \
             if(remaining > 0){                                          \
                 __tmp = true;                                           \
             } else {                                                      \
                 /* Next, check for 'itimers' -- these INTERVAL TIMERS are */ \
                 /* different in that they're only re-enqueued when their */ \
                 /* signal (i.e. SIGALRM) is DELIVERED. Accordingly, we */ \
                 /* CANNOT check the 'remaining' time for these timers. Instead, */ \
                 /* we compare them to an individual task's 'REAL_TIMER' address.*/ \
                 /* N.B.: Call to 'pid_task(...)' influenced by SEP driver code */ \
                 struct task_struct *tsk = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID); \
                 __tmp = (tsk && ( (hrt) == &tsk->signal->real_timer));  \
             }                                                           \
             __tmp; })

int init_timer_map(void);
void timer_init(void *timer_addr);
void timer_expire(void *timer_addr, pid_t tid, pid_t *return_pid, int *return_init_cpu);
int timer_delete(unsigned long timer_addr, pid_t tid);
void produce_r_sample(int cpu, unsigned long long tsc, r_sample_type_t type, pid_t tid, pid_t pid, const char *name);
void delete_timers_for_tid(pid_t tid);
void delete_all_non_kernel_timers(void);
void destroy_timer_map(void);


#endif // _SP_LINUX_TIMER_HANDLER_H_
