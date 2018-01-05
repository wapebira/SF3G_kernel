#include <asm/io.h>
#include <asm/cputime.h>
#include <linux/interrupt.h>
#include <trace/events/timer.h>
#include <trace/events/power.h>
#include <trace/events/irq.h>
#include <trace/events/sched.h>
struct pool_workqueue; // Get rid of warnings regarding trace_workqueue
#include <trace/events/workqueue.h>
#include <linux/types.h>
#include <linux/hash.h>
#include <linux/slab.h>

#include "sp.h"
#include "sp_linux_buffer.h"
#include "sp_linux_wakeup_handler.h"
#include "sp_linux_irq_handler.h"

extern int irq_map_samples_produced;

// 32 locks for the hash table
#define HASH_LOCK_BITS 5
#define NUM_HASH_LOCKS (1UL << HASH_LOCK_BITS)
#define HASH_LOCK_MASK (NUM_HASH_LOCKS - 1)

#define HASH_LOCK(i) LOCK(hash_locks[(i) & HASH_LOCK_MASK])
#define HASH_UNLOCK(i) UNLOCK(hash_locks[(i) & HASH_LOCK_MASK])


#define NUM_IRQ_MAP_BITS 6
#define NUM_IRQ_MAP_BUCKETS (1UL << NUM_IRQ_MAP_BITS)
#define IRQ_MAP_HASH_MASK (NUM_IRQ_MAP_BITS - 1)
#define IRQ_MAP_HASH_FUNC(a) hash_long((u32)a, NUM_IRQ_MAP_BITS)
#define IRQ_LOCK_MASK HASH_LOCK_MASK
#define IRQ_LOCK(i) LOCK(irq_map_locks[(i) & IRQ_LOCK_MASK])
#define IRQ_UNLOCK(i) UNLOCK(irq_map_locks[(i) & IRQ_LOCK_MASK])

#define PW_IRQ_DEV_NAME_LEN 100

typedef struct irq_hash_node irq_hash_node_t;
struct irq_hash_node{
    struct hlist_head head;
};

static irq_hash_node_t irq_map[NUM_IRQ_MAP_BUCKETS];


#define PW_HLIST_FOR_EACH_ENTRY(tpos, pos, head, member) pos = NULL; hlist_for_each_entry(tpos, head, member)
#define PW_HLIST_FOR_EACH_ENTRY_SAFE(tpos, pos, n, head, member) pos = NULL; hlist_for_each_entry_safe(tpos, n, head, member)
#define PW_HLIST_FOR_EACH_ENTRY_RCU(tpos, pos, head, member) pos = NULL; hlist_for_each_entry_rcu(tpos, head, member)

#define IS_BIT_SET(bit,map) ( test_bit( (bit), (map) ) != 0 )
#define SET_BIT(bit,map) ( test_and_set_bit( (bit), (map) ) )
#define PWR_CPU_BITMAP(node) ( (node)->cpu_bitmap )

/*
 * Size of each 'bucket' for a 'cpu_bitmap'
 */
#define NUM_BITS_PER_BUCKET (sizeof(unsigned long) * 8)
/*
 * Num 'buckets' for each 'cpu_bitmap' in the
 * 'irq_node' struct.
 */
#define NUM_BITMAP_BUCKETS ( (num_possible_cpus() / NUM_BITS_PER_BUCKET) + 1 )

#define LOCK(l) {                               \
    unsigned long _tmp_l_flags;                 \
    spin_lock_irqsave(&(l), _tmp_l_flags);

#define UNLOCK(l)                               \
    spin_unlock_irqrestore(&(l), _tmp_l_flags); \
    }

static spinlock_t irq_map_locks[NUM_HASH_LOCKS];


/*
 * Used to indicate whether
 * a new IRQ mapping was
 * created.
 */
typedef enum {
        OK_IRQ_MAPPING_EXISTS,
        OK_NEW_IRQ_MAPPING_CREATED,
        ERROR_IRQ_MAPPING
} irq_mapping_types_t;

typedef struct irq_node irq_node_t;
struct irq_node{
    struct hlist_node list;
    struct rcu_head rcu;
    int irq;
    char *name;
    /*
     * We send IRQ # <-> DEV name
     * mappings to Ring-3 ONCE PER
     * CPU. We need a bitmap to let
     * us know which cpus have
     * already had this info sent.
     *
     * FOR NOW, WE ASSUME A MAX OF 64 CPUS!
     * (This assumption is enforced in
     * 'init_data_structures()')
     */
    unsigned long *cpu_bitmap;
};

static void irq_destroy_callback(struct rcu_head *head)
{
  struct irq_node *node = container_of(head, struct irq_node, rcu);

  if (!node) {
    return;
  }

  if(node->name) {
    kfree(node->name);
    node->name = NULL;
  }
  if (node->cpu_bitmap) {
    kfree(node->cpu_bitmap);
    node->cpu_bitmap = NULL;
  }
  kfree(node);
};


//static int init_irq_map(void)
int init_irq_map(void)
{
  int i=0;

  for(i=0; i<NUM_IRQ_MAP_BUCKETS; ++i){
    INIT_HLIST_HEAD(&irq_map[i].head);
  }

  /*
   * Init locks
   */
  for(i=0; i<NUM_HASH_LOCKS; ++i){
      spin_lock_init(&irq_map_locks[i]);
  }

//    total_num_irq_mappings = 0;

  return 0;
};

void destroy_irq_map(void)
{
  int i;

  for(i=0; i<NUM_IRQ_MAP_BUCKETS; ++i){
    struct hlist_head *head = &irq_map[i].head;
    while(!hlist_empty(head)){
      struct irq_node *node = hlist_entry(head->first, struct irq_node, list);
      if (!node) {
        continue;
      }
      hlist_del(&node->list);
      irq_destroy_callback(&node->rcu);
    }
  }

//    if(irq_mappings_list){
//        pw_kfree(irq_mappings_list);
//        irq_mappings_list = NULL;
//    }
};


/*
 * Check if the given IRQ # <-> DEV Name mapping exists and, if
 * it does, whether this mapping was sent for the given 'cpu'
 * (We need to send each such mapping ONCE PER CPU to ensure it is
 * received BEFORE a corresponding IRQ C-state wakeup).
 */
static bool find_irq_node_i(int cpu, int irq_num, const char *irq_name, int *index, bool *was_mapping_sent)
{
  irq_node_t *node = NULL;
  struct hlist_node *curr = NULL;
  int idx = IRQ_MAP_HASH_FUNC(irq_num);

  *index = idx;

  rcu_read_lock();

  PW_HLIST_FOR_EACH_ENTRY_RCU (node, curr, &irq_map[idx].head, list) {
    if(node->irq == irq_num) {
      /*
       * OK, so the maping exists. But each
       * such mapping must be sent ONCE PER
       * CPU to Ring-3 -- have we done so
       * for this cpu?
       */
       *was_mapping_sent = (IS_BIT_SET(cpu, PWR_CPU_BITMAP(node))) ? true : false;
       rcu_read_unlock();
       return true;
    }
  }
  rcu_read_unlock();
  return false;
};


static irq_node_t *get_next_free_irq_node_i(int cpu, int irq_num, const char *irq_name)
{
  irq_node_t *node = kmalloc(sizeof(irq_node_t), GFP_ATOMIC);

  if (likely(node)) {
    memset(node, 0, sizeof(irq_node_t));
    node->irq = irq_num;
    /*
     * Set current CPU bitmap.
     */
    node->cpu_bitmap = kmalloc(sizeof(unsigned long) * NUM_BITMAP_BUCKETS, GFP_ATOMIC);
    if (unlikely(!node->cpu_bitmap)) {
      printk(KERN_INFO "sofia-proto: ERROR: could NOT allocate a bitmap for the new irq_node!\n");
      kfree(node);
      return NULL;
    }
    memset(node->cpu_bitmap, 0, sizeof(unsigned long) * NUM_BITMAP_BUCKETS);
    SET_BIT(cpu, PWR_CPU_BITMAP(node));

    INIT_HLIST_NODE(&node->list);

    if( !(node->name = kstrdup(irq_name, GFP_ATOMIC))){
      printk(KERN_INFO "sofia-proto: ERROR: could NOT kstrdup irq device name: %s\n", irq_name);
      kfree(node->cpu_bitmap);
      kfree(node);
      node = NULL;
    }
  } else {
    printk(KERN_INFO "sofia-proto: ERROR: could NOT allocate new irq node!\n");
  }

  return node;
};


/*
 * Check to see if a given IRQ # <-> DEV Name mapping exists
 * in our list of such mappings and, if it does, whether this
 * mapping has been sent to Ring-3. Take appropriate actions
 * if any of these conditions is not met.
 */
static irq_mapping_types_t irq_insert(int cpu, int irq_num, const char *irq_name)
{
  irq_node_t *node = NULL;
  int idx = -1;
  bool found_mapping = false, mapping_sent = false;

  if (!irq_name) {
    printk(KERN_INFO "sofia-proto: ERROR: irq_insert() NULL IRQ name?!\n");
    return ERROR_IRQ_MAPPING;
  }
  /*
   * Protocol:
   * (a) if mapping FOUND: return "OK_IRQ_MAPPING_EXISTS"
   * (b) if new mapping CREATED: return "OK_NEW_IRQ_MAPPING_CREATED"
   * (c) if ERROR: return "ERROR_IRQ_MAPPING"
   */

  found_mapping = find_irq_node_i(cpu, irq_num, irq_name, &idx, &mapping_sent);
  if(found_mapping && mapping_sent){
    /*
     * OK, mapping exists AND we've already
     * sent the mapping for this CPU -- nothing
     * more to do.
     */
     return OK_IRQ_MAPPING_EXISTS;
  }

  /*
   * Either this mapping didn't exist at all,
   * or the mapping wasn't sent for this CPU.
   * In either case, because we're using RCU,
   * we'll have to allocate a new node.
   */
  node = get_next_free_irq_node_i(cpu, irq_num, irq_name);

  if(unlikely(node == NULL))
    return ERROR_IRQ_MAPPING;

  IRQ_LOCK(idx);
  {
    /*
     * It is *THEORETICALLY* possible that
     * a different CPU added this IRQ entry
     * to the 'irq_map'. For now, disregard
     * the possiblility (at worst we'll have
     * multiple entries with the same mapping,
     * which is OK).
     */
    bool found = false;
    irq_node_t *old_node = NULL;
    struct hlist_node *curr = NULL;
    if(found_mapping){
      PW_HLIST_FOR_EACH_ENTRY(old_node, curr, &irq_map[idx].head, list) {
        if(old_node->irq == irq_num) {
          /*
           * Found older entry -- copy the 'cpu_bitmap'
           * field over to the new entry (no need to set this
           * CPU's entry -- 'get_next_free_irq_node_i() has
           * already done that. Instead, do a BITWISE OR of
           * the old and new bitmaps)...
           */
          // printk (KERN_INFO "sofia-proto: [%d]: IRQ = %d, OLD bitmap = %lu\n", cpu, irq_num, *(old_node->cpu_bitmap));
          /*
           * UPDATE: new 'bitmap' scheme -- copy over the older
           * bitmap array...
           */
          memcpy(node->cpu_bitmap, old_node->cpu_bitmap, sizeof(unsigned long) * NUM_BITMAP_BUCKETS); // dst, src
          /*
           * ...then set the current CPU's pos in the 'bitmap'
           */
          SET_BIT(cpu, node->cpu_bitmap);
          /*
           * ...and then replace the old node with
           * the new one.
           */
          hlist_replace_rcu(&old_node->list, &node->list);
          call_rcu(&old_node->rcu, &irq_destroy_callback);
          /*
           * OK -- everything done.
           */
          found = true;
          break;
        }
      }
      if(!found)
        printk("sofia-proto: ERROR: irq_insert() CPU = %d, IRQ = %d, mapping_found but not found!\n", cpu, irq_num);
    } else {
      hlist_add_head_rcu(&node->list, &irq_map[idx].head);
      /*
       * We've added a new mapping.
       */
//      ++total_num_irq_mappings;
    }
  }
  IRQ_UNLOCK(idx);
  /*
   * Tell caller that this mapping
   * should be sent to Ring-3.
   */
  return OK_NEW_IRQ_MAPPING_CREATED;
};


static inline void produce_i_sample(int cpu, int num, u64 tsc, const char *name)
{
    struct PWCollector_msg sample;
    i_sample_t i_sample;

    sample.cpuidx = cpu | LINUX_MESSAGE_CPU_MSB;
    sample.tsc = tsc;

    i_sample.irq_num = num;
    memcpy(i_sample.irq_name, name, PW_IRQ_DEV_NAME_LEN); // dst, src
    // printk(KERN_INFO "sofia-proto: produce_i_sample() generated irq_map sample for device %s using irq %d\n", name, num);
    sample.data_type = IRQ_MAP;
    sample.data_len = sizeof(i_sample);
    sample.p_data = (u64)((unsigned long)&i_sample);

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    pw_produce_generic_msg(&sample, true); // "true" ==> wakeup sleeping readers, if required
};


void handle_irq_wakeup_i(int cpu, int irq_num, const char *irq_name, unsigned long long tsc)
{
  int create_i_sample_result;


  create_i_sample_result = irq_insert(cpu, irq_num, irq_name);
  if (create_i_sample_result == OK_NEW_IRQ_MAPPING_CREATED) {
    /*
     * Send mapping info to Ring-3.
     */
    produce_i_sample(cpu, irq_num, tsc, irq_name);
    irq_map_samples_produced++;
  } else if(create_i_sample_result == ERROR_IRQ_MAPPING) {
    printk("sofia-proto: handle_irq_wakeup_i ERROR: could NOT insert [%d,%s] into irq list!\n", irq_num, irq_name);
  }
};

