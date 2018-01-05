#include <asm/io.h>
#include <asm/cputime.h>
#include <linux/interrupt.h>
#include <trace/events/timer.h>
#include <linux/types.h>
#include <linux/hash.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/sched.h>

#include "sp.h"
#include "sp_linux_buffer.h"
#include "sp_linux_wakeup_handler.h"
#include "sp_linux_timer_handler.h"

extern int proc_map_samples_produced;
/*
 * Data structure definitions.
 */

typedef struct tnode tnode_t;
struct tnode{
  struct hlist_node list;
  unsigned long timer_addr;
  pid_t pid;
  pid_t tid;
  int init_cpu;
  unsigned int is_root_timer;
};

typedef struct hnode hnode_t;
struct hnode{
    struct hlist_head head;
};

typedef struct tblock tblock_t;
struct tblock{
    struct tnode *data;
    tblock_t *next;
};

typedef struct per_cpu_mem per_cpu_mem_t;
struct per_cpu_mem{
    tblock_t *block_list;
    hnode_t free_list_head;
};

static DEFINE_PER_CPU(per_cpu_mem_t, per_cpu_mem_vars);
#define GET_MEM_VARS(cpu) &per_cpu(per_cpu_mem_vars, (cpu))
#define GET_MY_MEM_VARS(cpu) &__get_cpu_var(per_cpu_mem_vars)


#define TID() (current->pid)
#define PID() (current->tgid)

#define LOCK(l) {                               \
    unsigned long _tmp_l_flags;                 \
    spin_lock_irqsave(&(l), _tmp_l_flags);

#define UNLOCK(l)                               \
    spin_unlock_irqrestore(&(l), _tmp_l_flags); \
    }

#define NUM_MAP_BUCKETS_BITS 9
#define NUM_MAP_BUCKETS (1UL << NUM_MAP_BUCKETS_BITS)

// 32 locks for the hash table
#define HASH_LOCK_BITS 5
#define NUM_HASH_LOCKS (1UL << HASH_LOCK_BITS)
#define HASH_LOCK_MASK (NUM_HASH_LOCKS - 1)

#define HASH_LOCK(i) LOCK(hash_locks[(i) & HASH_LOCK_MASK])
#define HASH_UNLOCK(i) UNLOCK(hash_locks[(i) & HASH_LOCK_MASK])

#define NUM_TIMER_NODES_PER_BLOCK 20

#define TIMER_HASH_FUNC(a) hash_ptr((void *)a, NUM_MAP_BUCKETS_BITS)

/*
 * Macro to add newly allocated timer
 * nodes to individual free lists.
 */
#define LINK_FREE_TNODE_ENTRIES(nodes, size, free_head) do{             \
        int i=0;                                                        \
        for(i=0; i<(size); ++i){                                        \
            tnode_t *__node = &((nodes)[i]);                            \
            hlist_add_head(&__node->list, &((free_head)->head));        \
        }                                                               \
    }while(0)


#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0)
    #define PW_HLIST_FOR_EACH_ENTRY(tpos, pos, head, member) hlist_for_each_entry(tpos, pos, head, member)
    #define PW_HLIST_FOR_EACH_ENTRY_SAFE(tpos, pos, n, head, member) hlist_for_each_entry_safe(tpos, pos, n, head, member)
    #define PW_HLIST_FOR_EACH_ENTRY_RCU(tpos, pos, head, member) hlist_for_each_entry_rcu(tpos, pos, head, member)
#else // >= 3.9.0
    #define PW_HLIST_FOR_EACH_ENTRY(tpos, pos, head, member) pos = NULL; hlist_for_each_entry(tpos, head, member)
    #define PW_HLIST_FOR_EACH_ENTRY_SAFE(tpos, pos, n, head, member) pos = NULL; hlist_for_each_entry_safe(tpos, n, head, member)
    #define PW_HLIST_FOR_EACH_ENTRY_RCU(tpos, pos, head, member) pos = NULL; hlist_for_each_entry_rcu(tpos, head, member)
#endif


static struct hnode timer_map[NUM_MAP_BUCKETS];

static spinlock_t hash_locks[NUM_HASH_LOCKS];

static tblock_t *allocate_new_timer_block(struct hnode *free_head)
{
  tblock_t *block = kmalloc(sizeof(tblock_t), GFP_ATOMIC);
  if(!block) {
    return NULL;
  }
  block->data = kmalloc(sizeof(tnode_t) * NUM_TIMER_NODES_PER_BLOCK, GFP_ATOMIC);
  if(!block->data) {
    kfree(block);
    return NULL;
  }
  memset(block->data, 0, sizeof(tnode_t) * NUM_TIMER_NODES_PER_BLOCK);
  if(free_head){
    LINK_FREE_TNODE_ENTRIES(block->data, NUM_TIMER_NODES_PER_BLOCK, free_head);
  }
  block->next = NULL;
  return block;
};


static int init_tnode_i(tnode_t *node, unsigned long timer_addr, pid_t tid, pid_t pid, int init_cpu, unsigned int root)
{
  if (!node) {
    return -1;
  }

  node->timer_addr = timer_addr;
  node->tid = tid;
  node->pid = pid;
  node->init_cpu = init_cpu;
  node->is_root_timer = root;

  /*
   * Ensure everyone sees this...
   */
  smp_mb();

  return 0;
};


static tnode_t *get_next_free_tnode_i(unsigned long timer_addr, pid_t tid, pid_t pid, int init_cpu, unsigned int root)
{
  per_cpu_mem_t *pcpu_mem = GET_MY_MEM_VARS();
  struct hnode *free_head = &pcpu_mem->free_list_head;
  struct hlist_head *head = &free_head->head;

  if(hlist_empty(head)){
    tblock_t *block = allocate_new_timer_block(free_head);
    if(block) {
      block->next = pcpu_mem->block_list;
      pcpu_mem->block_list = block;
    }
//    printk(KERN_INFO "sofia-proto: get_next_free_tnode_i() cpu %d: allocated a new timer block!\n", CPU());
  }

  if(!hlist_empty(head)) {
    struct tnode *node = hlist_entry(head->first, struct tnode, list);
    hlist_del(&node->list);
    if (init_tnode_i(node, timer_addr, tid, pid, init_cpu, root)) {
      hlist_add_head(&node->list, head);
      return NULL;
    }
    return node;
  }
  return NULL;
};



static void timer_insert(unsigned long timer_addr, pid_t tid, pid_t pid, int init_cpu, unsigned int root)
{
  int idx = TIMER_HASH_FUNC(timer_addr);
  struct hlist_node *curr = NULL;
  struct hlist_head *head = NULL;
  struct tnode *node = NULL, *new_node = NULL;
  bool found = false;

  HASH_LOCK(idx);
  {
    head = &timer_map[idx].head;

    PW_HLIST_FOR_EACH_ENTRY(node, curr, head, list) {
      if(node->timer_addr == timer_addr){
        /*
         * Update-in-place.
         */
//      OUTPUT(3, KERN_INFO "Timer %p UPDATING IN PLACE! Node = %p, Trace = %p\n", (void *)timer_addr, node, node->trace);
        init_tnode_i(node, timer_addr, tid, pid, init_cpu, root);
        found = true;
        break;
      }
    }

    if(!found) {
      /*
       * Insert a new entry here.
       */
      new_node = get_next_free_tnode_i(timer_addr, tid, pid, init_cpu, root);
      if(likely(new_node)) {
        hlist_add_head(&new_node->list, &timer_map[idx].head);
      } else { // !new_node
        printk(KERN_INFO "sofia-proto: timer_insert() ERROR: could NOT allocate new timer node!\n");
      }
    }
  }
  HASH_UNLOCK(idx);

  return;
};





void timer_init(void *timer_addr)
{
  pid_t tid = TID();
  pid_t pid = PID();
  unsigned int root = 0;
  int init_cpu = raw_smp_processor_id();

  if (tid == 0)
    root = 1;

  timer_insert ((unsigned long)timer_addr, tid, pid, init_cpu, root);
}


static tnode_t *timer_find(unsigned long timer_addr, pid_t tid)
{
  int idx = TIMER_HASH_FUNC(timer_addr);
  tnode_t *node = NULL, *retVal = NULL;
  struct hlist_node *curr = NULL;
  struct hlist_head *head = NULL;

  HASH_LOCK(idx);
  {
    head = &timer_map[idx].head;

    PW_HLIST_FOR_EACH_ENTRY(node, curr, head, list) {
      if(node->timer_addr == timer_addr && (node->tid == tid || tid < 0)) {
        retVal = node;
        break;
      }
    }
  }
  HASH_UNLOCK(idx);

  return retVal;
};

void timer_expire(void *timer_addr, pid_t tid, pid_t *return_pid, int *return_init_cpu)
{
  pid_t pid;
  tnode_t *entry = NULL;
  int init_cpu = -1;
  bool found = false;

  if ((return_pid == NULL) || (return_init_cpu == NULL)) {
    printk(KERN_INFO "sofia-proto: timer_expire() BAD input parameters!\n");
    return;
  }

//  printk(KERN_INFO "sofia-proto: timer_expire() starting\n");

  if ( (entry = (tnode_t *)timer_find((unsigned long)timer_addr, tid))) {
    pid = entry->pid;
    init_cpu = entry->init_cpu;
    found = true;
//    printk(KERN_INFO "sofia-proto: timer_expire() found timer pid=0x%x, timer %p\n", pid, timer_addr);
  } else {
    /*
     * Couldn't find timer entry -- PID defaults to TID.
     */
    pid = tid;
//    printk(KERN_INFO "sofia-proto: timer_expire() pid=0x%x: timer %p NOT found in list!\n", pid, timer_addr);
    // TODO ask GU about setting the pid to 0, why?
  }

  if (!found) {
    if (tid < 0) {
      /*
       * UPDATE: this is also possible if
       * the kernel wasn't compiled with the
       * 'CONFIG_TIMER_STATS' option set.
      */
//      printk(KERN_INFO "sofia-proto: NEGATIVE tid in timer_expire, is kernel compiled with CONFIG_TIMER_STATS enabled?\n");
    }
  } else {
    /*
     * OK, found the entry. But timers fired
     * because of 'TIMER_SOFTIRQ' will have
     * tid == -1. Guard against that
     * by checking the 'tid' value. If < 0
     * then replace with entry->tid
     */
     if(tid < 0) {
       tid = entry->tid;
     }
  }
  *return_pid = pid;
  *return_init_cpu = init_cpu;
};



static void timer_destroy(struct tnode *node)
{
  per_cpu_mem_t *pcpu_mem = GET_MY_MEM_VARS();
  struct hnode *free_head = &pcpu_mem->free_list_head;

  if (!node) {
    return;
  }

//  printk(KERN_INFO "sofia-proto: timer_destroy() destroying %p\n", node);
  hlist_add_head(&node->list, &((free_head)->head));
};

#if 0
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
            }else{                                                      \
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
#endif

int timer_delete(unsigned long timer_addr, pid_t tid)
{
  int idx = TIMER_HASH_FUNC(timer_addr);
  tnode_t *node = NULL, *found_node = NULL;
  struct hlist_node *curr = NULL, *next = NULL;
  struct hlist_head *head = NULL;
  int retVal = -1;

  HASH_LOCK(idx);
  {
    head = &timer_map[idx].head;

    PW_HLIST_FOR_EACH_ENTRY_SAFE(node, curr, next, head, list) {
      if(node->timer_addr == timer_addr) {
        if (node->tid != tid) {
          printk(KERN_INFO "sofia-proto: timer_delete(), WARNING: stale timer tid value? node tid = %d, task tid = %d\n", node->tid, tid);
        }
        hlist_del(&node->list);
        found_node = node;
        retVal = 0;
//        printk(KERN_INFO "sofia-proto: timer_delete(), tid=%d: TIMER_DELETE FOUND HRT = %p\n", tid, (void *)timer_addr);
        break;
      }
    }
  }
  HASH_UNLOCK(idx);

  if(found_node){
    timer_destroy(found_node);
  }

  return retVal;
};


void delete_all_non_kernel_timers(void)
{
  struct tnode *node = NULL;
  struct hlist_node *curr = NULL, *next = NULL;
  int i, num_timers = 0;

  for(i=0; i<NUM_MAP_BUCKETS; ++i) {
    HASH_LOCK(i);
    {
      PW_HLIST_FOR_EACH_ENTRY_SAFE(node, curr, next, &timer_map[i].head, list) {
        if (node->is_root_timer == 0) {
          ++num_timers;
          hlist_del(&node->list);
          timer_destroy(node);
        }
      }
    }
    HASH_UNLOCK(i);
  }
};

void delete_timers_for_tid(pid_t tid)
{
  struct tnode *node = NULL;
  struct hlist_node *curr = NULL, *next = NULL;
  int i, num_timers = 0;

  for(i=0; i<NUM_MAP_BUCKETS; ++i)
  {
    HASH_LOCK(i);
    {
      PW_HLIST_FOR_EACH_ENTRY_SAFE(node, curr, next, &timer_map[i].head, list) {
        if(node->is_root_timer == 0 && node->tid == tid) {
          ++num_timers;
          hlist_del(&node->list);
          timer_destroy(node);
        }
      }
    }
    HASH_UNLOCK(i);
  }

  // printk(KERN_INFO "sofia-proto: delete_timers_for_tid() tid=%d: # timers deleted = %d\n", tid, num_timers);
};


/*
 * Insert a PROC_MAP sample into a (per-cpu) output buffer.
 */
void produce_r_sample(int cpu, unsigned long long tsc, r_sample_type_t type, pid_t tid, pid_t pid, const char *name)
{
  struct PWCollector_msg sample;
  r_sample_t r_sample;

  sample.cpuidx = cpu | LINUX_MESSAGE_CPU_MSB;
  sample.tsc = tsc;

  r_sample.type = type;
  r_sample.tid = tid;
  r_sample.pid = pid;
  memcpy(r_sample.proc_name, name, PW_MAX_PROC_NAME_SIZE); // dst, src

  sample.data_type = PROC_MAP;
  sample.data_len = sizeof(r_sample);
  sample.p_data = (u64)((unsigned long)&r_sample);

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
  // printk(KERN_INFO "sofia-proto: produce_r_sample() sending sample to buffer; pid=0x%x, name=%s\n", pid, name);
  pw_produce_generic_msg(&sample, true); // "true" ==> wakeup sleeping readers, if required
  proc_map_samples_produced++;
};


void destroy_timer_map(void)
{
  /*
   * NOP: nothing to free here -- timer nodes
   * are freed when their corresponding
   * (per-cpu) blocks are freed.
   */
};

int init_timer_map(void)
{
  int i;

  for(i=0; i<NUM_MAP_BUCKETS; ++i) {
    INIT_HLIST_HEAD(&timer_map[i].head);
  }

  for (i=0; i<NUM_HASH_LOCKS; ++i) {
    spin_lock_init(&hash_locks[i]);
  }

  return 0;
};
