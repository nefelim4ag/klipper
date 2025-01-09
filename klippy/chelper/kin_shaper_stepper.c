// Stepper input shapers to minimize motion oscillation
//
// Copyright (C) 2025  Timofey Titovets <nefelim4ag@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h>      // sqrt, exp
#include <stddef.h>    // offsetof
#include <stdlib.h>    // malloc
#include <string.h>    // memset
#include "compiler.h"  // __visible
#include "stepcompress.h" // struct stepcompress
#include "trapq.h"     // struct move
#include "pyhelper.h"  // errorf
#include "stdio.h"

struct tracker {
    uint16_t mscnt;
    uint8_t msteps;
    uint8_t mscnt_min;
    uint16_t mperiod;
    uint16_t period;
    float amplitude;
    float f[1024];
};

static struct tracker steppers[64];

int __visible
stepper_set_shaper_params(uint32_t oid, int mscnt, int msteps, int period, int offset, int amplitude)
{
    if (oid >= 16) {
        errorf("stepper shaper oid %d > MAX", oid);
        return -1;
    }

    if (period > 1024 || period % 256 != 0) {
        errorf("stepper shaper period (%i) is not 0, 256, 512, 1024", period);
        return -1;
    }

    steppers[oid].mscnt = mscnt;
    steppers[oid].msteps = msteps;
    steppers[oid].mscnt_min = msteps / 2;
    steppers[oid].period = period;

    FILE *file;
    char path[128];
    sprintf(path, "/tmp/stepper_shaper_%u.txt", oid);
    file = fopen(path, "w");

    int N = period / msteps;
    fprintf(file, "offset: %i -> %i\n", offset, offset / msteps);
    offset = offset / msteps;
    fprintf(file, "period: %i -> %i\n", period, N);
    float amp = (float)(amplitude) / 100;
    for (int i = 0; i < N; i++) {
        // Normalize i to [-1, 1]
        float x = (float)(i - offset) / (N / 2);
        steppers[oid].f[i] = 1.0f - amp/2 + amp * (-(x * x) + 1.0f);
        fprintf(file, "f[%i]: %f\n", i, steppers[oid].f[i]);
    }

    // Initialize after table computation finished
    steppers[oid].mperiod = period / msteps;
    fclose(file);

    return 0;
}

int __visible
stepper_get_shaper_mscnt(uint32_t oid)
{
    if (oid >= 16) {
        errorf("lin shaper oid %u > MAX", oid);
        return -1;
    }

    return steppers[oid].mscnt;
}

static uint32_t
diff(uint32_t a, uint32_t b)
{
    return (a >= b) ? (a - b) : (UINT32_MAX - b + a + 1);
}

static void
adjust_absolute_timings(uint32_t oid, uint32_t *timings)
{
    struct tracker *stepper = &steppers[oid];
    int mperiod = stepper->mperiod;
    uint32_t total_time = diff(timings[mperiod - 1], timings[0]);
    uint32_t original_intervals[stepper->mperiod];
    uint32_t adjusted_intervals[stepper->mperiod];
    uint32_t adjusted_total = 0;
    float normalization;
    int N = stepper->mperiod;

    FILE *file;
    char path[128];
    sprintf(path, "/tmp/adjust_timings_%u.txt", oid);
    file = fopen(path, "a");
    fprintf(file, "---\n");

    // Shift pulses around 'mid-point'
    for (int i = 1; i < N; i++) {
        original_intervals[i] = diff(timings[i], timings[i - 1]);
        adjusted_intervals[i] = original_intervals[i] * stepper->f[i];
        fprintf(file, "orig: %u, adj: %u\n", original_intervals[i], adjusted_intervals[i]);
        adjusted_total += adjusted_intervals[i];
    }
    fprintf(file, "adjusted_total: %u, total_time: %u\n", adjusted_total, total_time);
    // This should never happen
    if (adjusted_total < total_time / 2) {
        goto out;
    }
    normalization = (float)total_time / (float)adjusted_total;
    fprintf(file, "normalization: %f\n", normalization);
    if (total_time == 0)
        goto out;
    for (int i = 1; i < N; i++) {
        adjusted_intervals[i] *= normalization;
        // Update original timings
        fprintf(file, "timings_abs[%i]: %u, ", i, timings[i]);
        timings[i] = timings[i - 1] + adjusted_intervals[i];
        fprintf(file, "adj: %u\n", timings[i]);
    }
out:
    fclose(file);
}

uint32_t *stepper_shaper(uint32_t oid, uint32_t *queue_pos, uint32_t *queue_next, uint64_t last_step_clock, uint64_t move_clock, int sdir)
{
    uint32_t *qlast = queue_next;
    uint32_t *pos = queue_pos;
    struct tracker *stepper = &steppers[oid];
    int msteps = (sdir) ? -stepper->msteps : stepper->msteps;
    if (last_step_clock >= move_clock)
        return pos;
    if (stepper->mperiod == 0)
        return qlast;
    // FILE *file;
    // char path[128];
    // sprintf(path, "/tmp/queue_flush_%u.txt", oid);
    // file = fopen(path, "a");
    // fprintf(file, "mscnt: %u -> ", stepper->mscnt);
    // uint32_t rot = 0;
    while (pos < qlast && last_step_clock < move_clock) {
        stepper->mscnt = (stepper->mscnt + msteps) % 1024;
        last_step_clock += diff(*(pos+1), *pos);
        if (qlast - pos > stepper->mperiod) {
            if (stepper->mscnt % stepper->period == stepper->mscnt_min)
                adjust_absolute_timings(oid, pos);
        }
        pos++;
        // rot++;
    }
    // fprintf(file, " -> %u\n", stepper->mscnt);
    // fprintf(file, "Observed steps: %u\n", rot);
    // if (pos >= qlast) {
    //     fprintf(file, "Ended by pos(%u) >= qlast(%u):\n", *pos, *qlast);
    // }
    // if (last_step_clock >= move_clock) {
    //     fprintf(file, "Ended by last_step_clock(%lu) >= move_clock(%lu):\n", last_step_clock, move_clock);
    // }
    // fclose(file);
    return pos;
}

void stepper_shaper_far(uint32_t oid, int sdir) {
    // FILE *file;
    // char path[128];
    // sprintf(path, "/tmp/queue_flush_%u.txt", oid);
    // file = fopen(path, "a");
    struct tracker *stepper = &steppers[oid];
    if (stepper->mperiod == 0)
        return;
    int msteps = (sdir) ? -stepper->msteps : stepper->msteps;
    stepper->mscnt = (stepper->mscnt + msteps) % 1024;
    // fprintf(file, "Observed steps +1\n");
    // fclose(file);
}
