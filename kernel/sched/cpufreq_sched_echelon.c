#include "sched.h"

#include <linux/sched/cpufreq.h>
#include <trace/events/power.h>
#include <linux/sched/sysctl.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>  // This header fully defines sched_param
#include <linux/sched/rt.h>
#include <linux/sched/clock.h>
#include <linux/rcupdate.h>
#include <linux/thermal.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/arch_topology.h>

#ifndef CPUFREQ_GOV_START
#define CPUFREQ_GOV_START 0
#define CPUFREQ_GOV_STOP  1
#define CPUFREQ_GOV_LIMITS 2
#endif

#define ECHELON_UP_THRESHOLD       70
#define ECHELON_DOWN_THRESHOLD     40
#define ECHELON_THERMAL_LIMIT      95000
#define ECHELON_MIN_EMU_FREQ       1200000 /* kHz */
#define ECHELON_RATE_LIMIT_NS      (20 * NSEC_PER_MSEC)
#define ECHELON_BOOST_DURATION_MS  200

static const char *emulator_procs[] = {
    "yuzu", "aethersx2", "ppsspp", "dolphin-emu", "vita3k"
};

struct echelon_policy {
    struct cpufreq_policy *policy;
    struct task_struct *thread;
    struct kthread_worker worker;
    struct kthread_work work;
    struct thermal_zone_device *tz;

    unsigned int last_freq;
    u64 last_update;
    bool emulator_active;
    u64 rate_limit_ns;
    raw_spinlock_t update_lock;

    unsigned int up_threshold;
    unsigned int down_threshold;
    unsigned int min_emu_freq;
    unsigned int thermal_limit;

    u64 last_boost_time;
    bool touch_boosted;
};

static bool is_emulator_active(void)
{
    struct task_struct *task;
    rcu_read_lock(); // Corrected function name
    for_each_process(task) {
        int i;
        for (i = 0; i < ARRAY_SIZE(emulator_procs); i++) {
            if (strnstr(task->comm, emulator_procs[i], TASK_COMM_LEN)) {
                rcu_read_unlock(); // Corrected function name
                return true;
            }
        }
    }
    rcu_read_unlock(); // Corrected function name
    return false;
}

static unsigned int get_safe_temp(void)
{
    struct thermal_zone_device *tz;
    int temp = 0;

    tz = thermal_zone_get_zone_by_name("cpu-thermal");
    if (!IS_ERR(tz))
        thermal_zone_get_temp(tz, &temp);

    return temp;
}

static void log_policy_state(struct echelon_policy *ep, unsigned int target)
{
    pr_debug("[sched_echelon] CPU%d: freq=%u kHz, emu=%d, touch=%d, temp=%u\n",
             ep->policy->cpu, target, ep->emulator_active, ep->touch_boosted, get_safe_temp());
}

static void echelon_freq_work(struct kthread_work *work)
{
    struct echelon_policy *ep = container_of(work, struct echelon_policy, work);
    struct cpufreq_policy *policy = ep->policy;
    unsigned int util = topology_get_cpu_scale(NULL, policy->cpu);
    unsigned int max = policy->cpuinfo.max_freq;
    unsigned int min = policy->cpuinfo.min_freq;
    unsigned int target = policy->cur;
    unsigned int load;
    u64 now = sched_clock();

    raw_spin_lock(&ep->update_lock);

    if (now - ep->last_update < ep->rate_limit_ns) {
        raw_spin_unlock(&ep->update_lock);
        return;
    }

    ep->emulator_active = is_emulator_active();

    if (ep->emulator_active) {
        target = max;
        if (target < ep->min_emu_freq)
            target = ep->min_emu_freq;
    } else {
        if (ep->touch_boosted && time_before64(now, ep->last_boost_time + (ECHELON_BOOST_DURATION_MS * NSEC_PER_MSEC)))
            target = max;
        else {
            ep->touch_boosted = false;
            load = (util > 0) ? (policy->cur * 100 / util) : 0;

            if (load > ep->up_threshold)
                target = max;
            else if (load < ep->down_threshold)
                target = min;
        }
    }

    if (get_safe_temp() > ep->thermal_limit)
        target = min;

    target = clamp_val(target, min, max);

    if (target != ep->last_freq) {
        if (cpufreq_driver_fast_switch(policy, target))
            policy->cur = target;
        ep->last_freq = target;
    }

    log_policy_state(ep, target);
    ep->last_update = now;
    raw_spin_unlock(&ep->update_lock);
}

static int echelon_governor_start(struct cpufreq_policy *policy)
{
    struct echelon_policy *ep;
    struct sched_param param = {.sched_priority = MAX_RT_PRIO / 2};
    
    cpufreq_enable_fast_switch(policy);

    ep = kzalloc(sizeof(*ep), GFP_KERNEL);
    if (!ep)
        return -ENOMEM;

    ep->policy = policy;
    ep->last_freq = policy->cur;
    ep->last_update = 0;
    ep->rate_limit_ns = ECHELON_RATE_LIMIT_NS;
    ep->up_threshold = ECHELON_UP_THRESHOLD;
    ep->down_threshold = ECHELON_DOWN_THRESHOLD;
    ep->min_emu_freq = ECHELON_MIN_EMU_FREQ;
    ep->thermal_limit = ECHELON_THERMAL_LIMIT;
    ep->last_boost_time = 0;
    ep->touch_boosted = false;

    raw_spin_lock_init(&ep->update_lock);
    kthread_init_worker(&ep->worker);
    kthread_init_work(&ep->work, echelon_freq_work);

    ep->thread = kthread_create(kthread_worker_fn, &ep->worker, "echelon/%d", policy->cpu);
    if (IS_ERR(ep->thread)) {
        kfree(ep);
        return PTR_ERR(ep->thread);
    }

    sched_setscheduler_nocheck(ep->thread, SCHED_FIFO, &param);
    kthread_bind(ep->thread, policy->cpu);
    wake_up_process(ep->thread);
    kthread_queue_work(&ep->worker, &ep->work);

    return 0;
}

static void echelon_governor_stop(struct cpufreq_policy *policy)
{
    struct echelon_policy *ep = policy->governor_data; // Use governor_data
    if (!ep)
        return;

    kthread_cancel_work_sync(&ep->work);
    kthread_stop(ep->thread);
    kfree(ep);
    policy->governor_data = NULL; // Use governor_data
}

static struct cpufreq_governor sched_echelon_gov = {
    .name = "sched_echelon",
    .owner = THIS_MODULE,
    .start = echelon_governor_start, // Use start directly instead of event_handler
    .stop = echelon_governor_stop,   // Use stop directly instead of event_handler
};

static int __init sched_echelon_init(void)
{
    return cpufreq_register_governor(&sched_echelon_gov);
}
module_init(sched_echelon_init);

static void __exit sched_echelon_exit(void)
{
    cpufreq_unregister_governor(&sched_echelon_gov);
}
module_exit(sched_echelon_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("The_Anomalist");
MODULE_DESCRIPTION("sched_echelon: Full-featured CPU governor with emulation boost, thermal cap, sched hints, and touch boost");

