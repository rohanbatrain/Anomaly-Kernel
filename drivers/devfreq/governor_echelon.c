/*
 *  linux/drivers/devfreq/governor_echelon.c
 *
 *  Copyright (C) 2025 The_Anomalist
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/devfreq.h>
#include <linux/math64.h>
#include <linux/ktime.h>
#include <linux/thermal.h>  /* For temperature-based scaling */
#include <linux/sched/clock.h>
#include <dt-bindings/regulator/qcom,rpmh-regulator-levels.h>
#include <dt-bindings/regulator/qcom,rpmh-regulator-levels.h>
#include <dt-bindings/soc/qcom,rpmh-rsc.h>
#include <dt-bindings/clock/qcom,gpucc-kona.h>
#include "governor.h"

/* Echelon: Gaming-optimized DevFreq Governor for Adreno 650 */
#define ECHELON_UPTHRESHOLD       (75)  /* 75% load for scaling up */
#define ECHELON_DOWNTHRESHOLD     (30)  /* 30% load for scaling down */
#define ECHELON_DOWNSCALE_FACTOR  (50)  /* Frequency reduction factor on idle */
#define ECHELON_SCALE_TIMEOUT     (100) /* Timeout for scaling in ms */
#define THERMAL_ZONE_NAME         "thermal_zone0" /* Assuming thermal zone0 */

/* OPP Table */
static const unsigned long gpu_opp_freqs[] = {
    944000000, 835000000, 720000000, 640000000,
    525000000, 490000000, 400000000, 305000000,
    150000000
};

/* Data structure to hold Echelon governor settings */
struct devfreq_echelon_data {
    unsigned int upthreshold;
    unsigned int downthreshold;
    unsigned int downscale_factor;
    unsigned int max_scale_step;
    ktime_t last_update_time; /* Time of last frequency update */
};

/* Helper function to find the closest OPP */
static unsigned long find_closest_opp(unsigned long target_freq)
{
    unsigned long closest = gpu_opp_freqs[0];
    int i;

    for (i = 1; i < ARRAY_SIZE(gpu_opp_freqs); i++) {
        if (abs(target_freq - gpu_opp_freqs[i]) < abs(target_freq - closest)) {
            closest = gpu_opp_freqs[i];
        }
    }
    return closest;
}

/* Frequency scaling function */
static int devfreq_echelon_func(struct devfreq *df, unsigned long *freq)
{
    int err;
    struct devfreq_dev_status *stat;
    struct devfreq_echelon_data *data = df->data;
    unsigned long max = (df->max_freq) ? df->max_freq : gpu_opp_freqs[0];
    unsigned long min = (df->min_freq) ? df->min_freq : gpu_opp_freqs[ARRAY_SIZE(gpu_opp_freqs) - 1];

    unsigned int upthreshold = ECHELON_UPTHRESHOLD;
    unsigned int downthreshold = ECHELON_DOWNTHRESHOLD;
    unsigned int downscale_factor = ECHELON_DOWNSCALE_FACTOR;
    unsigned long predicted_frequency = *freq;

    struct thermal_zone_device *tz = NULL;

    err = devfreq_update_stats(df);
    if (err)
        return err;

    stat = &df->last_status;

    if (data) {
        if (data->upthreshold)
            upthreshold = data->upthreshold;
        if (data->downthreshold)
            downthreshold = data->downthreshold;
        if (data->downscale_factor)
            downscale_factor = data->downscale_factor;
    }

    if (stat->total_time == 0) {
        *freq = max;
        return 0;
    }

    if (stat->busy_time * 100 > stat->total_time * upthreshold) {
        predicted_frequency = max;
    } else if (stat->busy_time * 100 < stat->total_time * downthreshold) {
        predicted_frequency = min;
    }

    predicted_frequency = find_closest_opp(predicted_frequency);

    if (predicted_frequency != *freq) {
        *freq = predicted_frequency;
    }

    if (*freq < min)
        *freq = min;
    if (*freq > max)
        *freq = max;

    tz = thermal_zone_get_zone_by_name(THERMAL_ZONE_NAME);
    if (!IS_ERR(tz)) {
        int temp;
        err = thermal_zone_get_temp(tz, &temp);
        if (err == 0 && temp > 95000) {
            *freq = min;
        }
    }

    return 0;
}

/* Event handler for Echelon governor */
static int devfreq_echelon_handler(struct devfreq *devfreq,
                                   unsigned int event, void *data)
{
    switch (event) {
    case DEVFREQ_GOV_START:
        devfreq_monitor_start(devfreq);
        break;

    case DEVFREQ_GOV_STOP:
        devfreq_monitor_stop(devfreq);
        break;

    case DEVFREQ_GOV_INTERVAL:
        devfreq_interval_update(devfreq, (unsigned int *)data);
        break;

    case DEVFREQ_GOV_SUSPEND:
        devfreq_monitor_suspend(devfreq);
        break;

    case DEVFREQ_GOV_RESUME:
        devfreq_monitor_resume(devfreq);
        break;

    default:
        break;
    }

    return 0;
}

static struct devfreq_governor devfreq_echelon = {
    .name = "Echelon",
    .get_target_freq = devfreq_echelon_func,
    .event_handler = devfreq_echelon_handler,
};

static int __init devfreq_echelon_init(void)
{
    return devfreq_add_governor(&devfreq_echelon);
}
subsys_initcall(devfreq_echelon_init);

static void __exit devfreq_echelon_exit(void)
{
    int ret;

    ret = devfreq_remove_governor(&devfreq_echelon);
    if (ret)
        pr_err("%s: failed remove governor %d\n", __func__, ret);
}
module_exit(devfreq_echelon_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("The_Anomalist");
MODULE_DESCRIPTION("Echelon: A high-performance devfreq governor for Adreno 650 GPU, optimized for gaming.");
