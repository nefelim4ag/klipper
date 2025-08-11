// Stepper pulse schedule compression
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

// The goal of this code is to take a series of scheduled stepper
// pulse times and compress them into a handful of commands that can
// be efficiently transmitted and executed on a microcontroller (mcu).
// The mcu accepts step pulse commands that take interval, count, and
// add parameters such that 'count' pulses occur, with each step event
// calculating the next step event time using:
//  next_wake_time = last_wake_time + interval; interval += add
// This code is written in C (instead of python) for processing
// efficiency - the repetitive integer math is vastly faster in C.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdint.h> // uint32_t
#include <stdio.h> // fprintf
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // DIV_ROUND_UP
#include "pyhelper.h" // errorf
#include "serialqueue.h" // struct queue_message
#include "stepcompress.h" // stepcompress_alloc

#define CHECK_LINES 1
#define QUEUE_START_SIZE 1024
// Most flat sections would bounce around +-1
// We limited by the max_error which is mcu_freq * t
// That mean rougly we would encode 0.5 * max_err, max_err + 1.5 flat lines
// 550_000_000 * 0.000_025 = 13750
// So let's limit memory/cpu complexity
#define MAX_STEPS_PER_MSG (16 * 1024)

struct stepcompress {
    // Buffer management
    uint32_t *queue, *queue_end, *queue_pos, *queue_next;
    // Internal tracking
    uint32_t max_error;
    double mcu_time_offset, mcu_freq, last_step_print_time;
    // Message generation
    uint64_t last_step_clock;
    struct list_head msg_queue;
    uint32_t oid;
    int32_t queue_step_msgtag, set_next_step_dir_msgtag;
    int sdir, invert_sdir;
    // Step+dir+step filter
    uint64_t next_step_clock;
    int next_step_dir;
    // History tracking
    int64_t last_position;
    struct list_head history_list;
    // Compression state
    uint32_t last_interval;
    int16_t last_add;
    uint16_t last_count;
};

struct step_move {
    uint32_t interval;
    uint16_t count;
    int16_t add;
};

struct history_steps {
    struct list_node node;
    uint64_t first_clock, last_clock;
    int64_t start_position;
    int step_count, interval, add;
};


/****************************************************************
 * Step compression
 ****************************************************************/

struct points {
    int32_t minp, maxp;
};

struct lls_state {
    double S_k;  // sum k
    double S_k2; // sum k^2
    double S_y;  // sum y_k
    double S_ky; // sum k * y_k
    double S_w;  // sum weights
    uint32_t n;
};

// Given a requested step time, return the minimum and maximum
// acceptable times
static inline struct points
minmax_point(struct stepcompress *sc, const uint32_t *pos)
{
    uint32_t lsc = sc->last_step_clock, point = *pos - lsc;
    uint32_t prevpoint = pos > sc->queue_pos ? *(pos-1) - lsc : 0;
    uint32_t max_error = (point - prevpoint) / 2;
    if (max_error > sc->max_error)
        max_error = sc->max_error;
    return (struct points){ point - max_error, point };
}

// Add a point to LLS calculation
static void
lls_add_point(struct lls_state *state, uint32_t k, uint32_t interval, double w) {
    int64_t kd = k;
    int64_t yd = interval;
    state->S_w  += w;
    state->S_k  += w * kd;
    state->S_k2 += w * kd*kd;
    state->S_y  += w * yd;
    state->S_ky += w * kd * yd;
    state->n++;
}

// Remove a point from LLS calculation
static void
lls_remove_point(struct lls_state *state, uint32_t k, uint32_t interval, double w) {
    uint32_t kd = k;
    uint32_t yd = interval;
    state->S_w  -= w;
    state->S_k  -= w * kd;
    state->S_k2 -= w * kd*kd;
    state->S_y  -= w * yd;
    state->S_ky -= w * kd * yd;
    state->n--;
}

struct ia_pair {
    uint32_t I_hi, I_lo;
    int32_t A_hi, A_lo;
};

static inline struct ia_pair
lls_solve(const struct lls_state *state) {
    struct ia_pair ret;
    double Sw = state->S_w;
    double n = state->n;
    double k_mean = (double)(state->S_k) / Sw;
    double y_mean = (double)state->S_y / Sw;
    // Sum of (k-kmean)^2
    double Sxx = state->S_k2 - Sw * k_mean * k_mean;
    // Sum of (k-kmean)*(y - ymean) == Σ(k-kmean)*y
    double Sxy = (double)state->S_ky - Sw * k_mean * y_mean;
    if (Sxx == 0.0) {
        ret.A_lo = 0;
        ret.A_hi = 0;
    } else {
        ret.A_lo = floor(Sxy / Sxx);
        ret.A_hi = ceil(Sxy / Sxx);
    }
    ret.I_lo = floor(y_mean - (ret.A_lo) * k_mean);
    ret.I_hi = ceil(y_mean - (ret.A_hi) * k_mean);
    return ret;
}

static int call_id = 0;
// Returns 0 on success if hit min interval -1, if max +1
static int
validate_move(struct stepcompress *sc, uint32_t I, int16_t A, uint16_t C)
{
    // Predict
    uint32_t p = 0;
    if (I == 0)
        return -1;
    // Cast the line and fast check the end
    p = I * C + A * C * (C - 1) / 2;
    struct points point = minmax_point(sc, sc->queue_pos + (C-1));
    if (p < point.minp) {
        // fprintf(stderr, "call_id:\t%i| C: %d bounds: %d < %d \n", call_id, C, p, point.minp);
        return -1;
    }
    if (p > point.maxp) {
        // fprintf(stderr, "call_id:\t%i| C: %d bounds: %d > %d\n", call_id, C, p, point.maxp);
        return +1;
    }
    p = 0;

    for (int i = 0; i < C; i++) {
        point = minmax_point(sc, sc->queue_pos + i);
        p += I;
        if (p < point.minp) {
            // fprintf(stderr, "call_id:\t%i| bounds: %d < %d \n", call_id, p, point.minp);
            return -1;
        }
        if (p > point.maxp) {
            // fprintf(stderr, "call_id:\t%i| bounds: %d > %d\n", call_id, p, point.maxp);
            return +1;
        }
        I += A;
        if (I > 0x80000000)
            return -1;
    }
    return 0;
}

// Find a 'step_move' that covers a series of step times
// Fit quadric function
static struct step_move
compress_bisect_add(struct stepcompress *sc)
{
    call_id++;
    uint32_t steps = 0xffff;
    uint32_t *qlast = sc->queue_next;
    if (qlast > sc->queue_pos + steps)
        qlast = sc->queue_pos + steps;
    steps = qlast - sc->queue_pos;
    // Cache intervals
    uint32_t cI[steps];

    const uint32_t *pos = sc->queue_pos;
    const uint32_t lsc = sc->last_step_clock;
    struct step_move move = {
        .interval = pos[0] - lsc,
        .count = 1,
        .add = 0,
    };
    cI[0] = pos[0] - lsc;
    uint32_t I_max = cI[0];
    uint32_t max_error = (cI[0] + 0) / 2;
    if (max_error > sc->max_error)
        max_error = sc->max_error;
    uint32_t I_min = I_max - max_error;
    // bias towards last interval
    // Large hack to allow LLS dump overshoots
    {
        uint64_t sum = sc->last_interval;
        sum += move.interval;
        uint32_t p = sum/2;
        // fprintf(stderr, "call_id:\t%i| i: %d/%d,  p: %d < %d < %d, a: %d\n", call_id, i, steps, point.minp, p, point.maxp, pos[i] - lsc);
        if (p >= I_min && p <= I_max) {
            move.interval = p;
            cI[0] = p;
        }
    }

    if (steps == 1)
        goto out;

    // Try perfect fit 2 points
    {
        int32_t add = 0, minadd = -0x8000, maxadd = 0x7fff;
        cI[1] = pos[1] - pos[0];
        add = (int32_t)(cI[1]) - (int32_t)(cI[0]);
        // knot
        if (add < minadd || add > maxadd) {
            // fprintf(stderr, "call_id:\t%i| knot, add: %i\n", call_id, add);
            goto out;
        }
        move.count = 2;
        move.add = add;
    }

    // Greedy linear expansion
    // Cast straight line
    if (1){
        // uint32_t p = 0;
        // int best_count = 2;
        // uint32_t best_interval = move.interval;
        uint32_t left = I_min;
        uint32_t right = I_max;
        // Binary search pass
        while (left <= right) {
            uint32_t mid = left + (right - left) / 2;
            uint32_t I = mid;
            uint32_t p = I;
            int c = 1;
            for (; c < steps; c++) {
                struct points point = minmax_point(sc, pos + c);
                p += I;
                if (p < point.minp) {
                    // fprintf(stderr, "call_id:\t%i| bounds: %d < %d \n", call_id, p, point.minp);
                    left = mid + 1;
                    break;
                }
                if (p > point.maxp) {
                    // fprintf(stderr, "call_id:\t%i| bounds: %d > %d\n", call_id, p, point.maxp);
                    right = mid - 1;
                    break;
                }
            }
            // fprintf(stderr, "call_id:\t%i| wtf: %i/%d\n", call_id, c, steps);
            if (c > move.count) {
                move.count = c;
                move.add = 0;
                move.interval = I;
            }
            if (c == steps)
                break;
        }
        // fprintf(stderr, "call_id:\t%i| wtf: I_min: %d, left: %d, right: %d\n", call_id, I_min, left, right);
    }

    // Original addfactor is a = x*(x - 1) / 2
    // a * 2 = x * (x - 1)
    // a * 2 = x^2 - x
    // So, quadratic, so, max count for a at least +-1 add is ~
    // x = (1 + sqrt(1 + 4 * 2 * a)) / 2
    uint32_t no_add = (1 + sqrt( 1 + 4 * 2 * sc->max_error)) / 2;
    if (move.count <= 2)
        goto out;

    // Duct tape linear overshoots
    if (move.count > no_add) {
        int i = move.count-1;
        cI[i] = pos[i] - pos[i-1]; i--;
        for (; i > move.count / 4 * 3; i--) {
            cI[i] = pos[i] - pos[i-1];
        }
        i = move.count - 1;
        // Window size 4
        for (; i > move.count / 4 * 3 + 4; i--) {
            float add_avg = 0;
            for (int k = i; k > i - 4; k--) {
                int add = (int)cI[k] - (int)cI[k-1];
                add_avg += add;
            }
            add_avg = (add_avg) / 4;
            if (fabs(add_avg) > 0.5)
                move.count--;
            else
                break;
        }


    }
    if (move.count > no_add || move.count <= 2)
        goto out;

    // Detect incline
    // {
    //     uint32_t I = move.interval;
    //     int16_t A = move.add;
    //     uint16_t C = move.count;
    //     uint32_t last_v = I * C + A * C * (C - 1) / 2;
    //     struct points last_p = minmax_point(sc, pos + (C - 1));
    //     uint32_t start_p_avg = (I_min + I_max) / 2;
    //     uint32_t end_p_avg = ((uint64_t)last_p.minp + (uint64_t)last_p.maxp) / 2;
    //     // Interval value line is `/` our line is --
    //     if (I > start_p_avg && last_v < end_p_avg) {
    //         A++;
    //         last_v = I * C + A * C * (C - 1) / 2;
    //         // fprintf(stderr, "call_id:\t%i| wtf: I: %d V: %d, V_max: %d\n", call_id, I, last_v, last_p.maxp);
    //         while (last_v > last_p.maxp) {
    //             // fprintf(stderr, "call_id:\t%i| wtf: C: %i, A: %i\n", call_id, C, A);
    //             C--;
    //             last_p = minmax_point(sc, pos + (C - 1));
    //             last_v = I * C + A * C * (C - 1) / 2;
    //         }
    //     // Interval line is `\` our line is --
    //     } else if (I < start_p_avg && last_v > end_p_avg) {
    //         A--;
    //         last_v = I * C + A * C * (C - 1) / 2;
    //         // fprintf(stderr, "call_id:\t%i| wtf: I: %d V: %d, V_min: %d\n", call_id, I, last_v, last_p.minp);
    //         while (last_v < last_p.minp) {
    //             // fprintf(stderr, "call_id:\t%i| wtf: C: %i, A: %i\n", call_id, C, A);
    //             C--;
    //             last_p = minmax_point(sc, pos + (C - 1));
    //             last_v = I * C + A * C * (C - 1) / 2;
    //         }
    //     }

    //     // Detect break points
    //     uint32_t interval = I;
    //     uint32_t p = 0;
    //     for (int i = 0; i < C; i++) {
    //         struct points point = minmax_point(sc, sc->queue_pos + i);
    //         p += interval;
    //         // fprintf(stderr, "call_id:\t%i| bounds: %d < %d \n", call_id, p, point.minp);
    //         // fprintf(stderr, "call_id:\t%i| bounds: %d > %d\n", call_id, p, point.maxp);
    //         if (p < point.minp) {
    //             // fprintf(stderr, "call_id:\t%i| i: %i bounds: %d < %d \n", call_id, i, p, point.minp);
    //             C = i;
    //             break;
    //         }
    //         if (p > point.maxp) {
    //             // fprintf(stderr, "call_id:\t%i| i: %i bounds: %d > %d\n", call_id, i, p, point.maxp);
    //             C = i;
    //             break;
    //         }
    //         interval += A;
    //     }
    //     move.count = C;
    //     // move.interval = I;
    //     move.add = A;


    // }

    // return move;

    // Linear Least Squares
    // Math magic, I would say
    struct lls_state S = {
      .S_k = 0,
      .S_k2 = 0,
      .S_y = 0,
      .S_ky = 0,
      .n = 0,
    };

    uint32_t i = 0;
    // Preseed
    {
        struct points p = minmax_point(sc, pos + i);
        double width = p.maxp - p.minp;
        // cW[i] = 12.0 / (width*width);
        // w = 1 / width;
        lls_add_point(&S, i, cI[i], 1); i++;
        p = minmax_point(sc, pos + i);
        width = p.maxp - p.minp;
        // cW[i] = 12.0 / (width*width);
        lls_add_point(&S, i, cI[i], 1); i++;
    }
    // Fast exponential search
    // Allow refit
    move.count = 2;
    uint32_t last_valid = 2;
    for (int k = 32; k < steps; k = k*2) {
        for (; i < k; i++) {
            cI[i] = pos[i] - pos[i-1];
            struct points p = minmax_point(sc, pos + i);
            double width = p.maxp - p.minp;
            // cW[i] = 12.0 / (width*width);
            lls_add_point(&S, i, cI[i], 1);
        }

        struct ia_pair fit = lls_solve(&S);
        uint32_t I_fit = fit.I_lo;
        int16_t A_fit = fit.A_lo;
        int ret = validate_move(sc, fit.I_lo, fit.A_lo, S.n);
        if (ret != 0) {
            // neiborhood search
            ret = validate_move(sc, fit.I_hi, fit.A_hi, S.n);
            if (ret != 0)
                break;
            I_fit = fit.I_hi;
            A_fit = fit.A_hi;
        }
        if (S.n > move.count) {
            move.count = S.n;
            move.interval = I_fit;
            move.add = A_fit;
        }
        last_valid = S.n;
    }

    // I always forget that calculations
    // addfactor = count * (count - 1) / 2;
    // predicted = lsc + interval * count + add * addfactor;
    uint32_t left = last_valid;
    // Exponential search fail around there
    uint32_t right = no_add;
    // Binary search pass
    while (left <= right) {
        uint32_t mid = left + (right - left) / 2;
        for (; i < mid; i++) {
            lls_add_point(&S, i, cI[i], 1);
        }
        for (; i > mid; i--) {
            lls_remove_point(&S, i-1, cI[i-1], 1);
        }

        struct ia_pair fit = lls_solve(&S);
        int I_fit = fit.I_lo;
        int A_fit = fit.A_lo;
        int ret = validate_move(sc, fit.I_lo, fit.A_lo, S.n);
        if (ret != 0) {
            // neighborhood search
            ret = validate_move(sc, fit.I_hi, fit.A_hi, S.n);
            if (ret != 0) {
                right = mid - 1;
                continue;
            }
            I_fit = fit.I_hi;
            A_fit = fit.A_hi;
        }

        // Match, try further
        left = mid + 1;
        if (S.n > move.count) {
            move.count = S.n;
            move.add = A_fit;
            move.interval = I_fit;
        }
    }

    // Duct tape for LLS overshoots
    if (move.count > no_add) {
        int i = move.count-1;
        cI[i] = pos[i] - pos[i-1]; i--;
        for (; i > move.count / 4 * 3; i--) {
            cI[i] = pos[i] - pos[i-1];
        }
        i = move.count - 1;
        // Window size 4
        for (; i > move.count / 4 * 3 + 4; i--) {
            float add_avg = 0;
            for (int k = i; k > i - 4; k--) {
                int add = (int)cI[k] - (int)cI[k-1];
                add_avg += add;
            }
            add_avg = (add_avg) / 4;
            if (fabs(add_avg) > 1 + abs(move.add))
                move.count--;
            else
                break;
        }


    }

out:
    sc->last_interval = move.interval;
    sc->last_add = move.add;
    sc->last_count = move.count;
    return move;
}


/****************************************************************
 * Step compress checking
 ****************************************************************/

// Verify that a given 'step_move' matches the actual step times
static int
check_line(struct stepcompress *sc, struct step_move move)
{
    if (!CHECK_LINES)
        return 0;
    if (!move.count || (!move.interval && !move.add && move.count > 1)
        || move.interval >= 0x80000000) {
        errorf("stepcompress o=%d i=%d c=%d a=%d: Invalid sequence"
               , sc->oid, move.interval, move.count, move.add);
        return ERROR_RET;
    }
    uint32_t interval = move.interval, p = 0;
    uint16_t i;
    for (i=0; i<move.count; i++) {
        struct points point = minmax_point(sc, sc->queue_pos + i);
        p += interval;
        if (p < point.minp || p > point.maxp) {
            errorf("stepcompress o=%d i=%d c=%d a=%d: Point %d: %d not in %d:%d"
                   , sc->oid, move.interval, move.count, move.add
                   , i+1, p, point.minp, point.maxp);
            return ERROR_RET;
        }
        if (interval >= 0x80000000) {
            errorf("stepcompress o=%d i=%d c=%d a=%d:"
                   " Point %d: interval overflow %d"
                   , sc->oid, move.interval, move.count, move.add
                   , i+1, interval);
            return ERROR_RET;
        }
        interval += move.add;
    }
    return 0;
}


/****************************************************************
 * Step compress interface
 ****************************************************************/

// Allocate a new 'stepcompress' object
struct stepcompress * __visible
stepcompress_alloc(uint32_t oid)
{
    struct stepcompress *sc = malloc(sizeof(*sc));
    memset(sc, 0, sizeof(*sc));
    list_init(&sc->msg_queue);
    list_init(&sc->history_list);
    sc->oid = oid;
    sc->sdir = -1;
    return sc;
}

// Fill message id information
void __visible
stepcompress_fill(struct stepcompress *sc, uint32_t max_error
                  , int32_t queue_step_msgtag, int32_t set_next_step_dir_msgtag)
{
    sc->max_error = max_error;
    sc->queue_step_msgtag = queue_step_msgtag;
    sc->set_next_step_dir_msgtag = set_next_step_dir_msgtag;
}

// Set the inverted stepper direction flag
void __visible
stepcompress_set_invert_sdir(struct stepcompress *sc, uint32_t invert_sdir)
{
    invert_sdir = !!invert_sdir;
    if (invert_sdir != sc->invert_sdir) {
        sc->invert_sdir = invert_sdir;
        if (sc->sdir >= 0)
            sc->sdir ^= 1;
    }
}

// Helper to free items from the history_list
static void
free_history(struct stepcompress *sc, uint64_t end_clock)
{
    while (!list_empty(&sc->history_list)) {
        struct history_steps *hs = list_last_entry(
            &sc->history_list, struct history_steps, node);
        if (hs->last_clock > end_clock)
            break;
        list_del(&hs->node);
        free(hs);
    }
}

// Expire the stepcompress history older than the given clock
static void
stepcompress_history_expire(struct stepcompress *sc, uint64_t end_clock)
{
    free_history(sc, end_clock);
}

// Free memory associated with a 'stepcompress' object
void __visible
stepcompress_free(struct stepcompress *sc)
{
    if (!sc)
        return;
    free(sc->queue);
    message_queue_free(&sc->msg_queue);
    free_history(sc, UINT64_MAX);
    free(sc);
}

uint32_t
stepcompress_get_oid(struct stepcompress *sc)
{
    return sc->oid;
}

int
stepcompress_get_step_dir(struct stepcompress *sc)
{
    return sc->next_step_dir;
}

// Determine the "print time" of the last_step_clock
static void
calc_last_step_print_time(struct stepcompress *sc)
{
    double lsc = sc->last_step_clock;
    sc->last_step_print_time = sc->mcu_time_offset + (lsc - .5) / sc->mcu_freq;
}

// Set the conversion rate of 'print_time' to mcu clock
static void
stepcompress_set_time(struct stepcompress *sc
                      , double time_offset, double mcu_freq)
{
    sc->mcu_time_offset = time_offset;
    sc->mcu_freq = mcu_freq;
    calc_last_step_print_time(sc);
}

// Maximium clock delta between messages in the queue
#define CLOCK_DIFF_MAX (3<<28)

// Helper to create a queue_step command from a 'struct step_move'
static void
add_move(struct stepcompress *sc, uint64_t first_clock, struct step_move *move)
{
    int32_t addfactor = move->count*(move->count-1)/2;
    uint32_t ticks = move->add*addfactor + move->interval*(move->count-1);
    uint64_t last_clock = first_clock + ticks;

    // Create and queue a queue_step command
    uint32_t msg[5] = {
        sc->queue_step_msgtag, sc->oid, move->interval, move->count, move->add
    };
    struct queue_message *qm = message_alloc_and_encode(msg, 5);
    qm->min_clock = qm->req_clock = sc->last_step_clock;
    if (move->count == 1 && first_clock >= sc->last_step_clock + CLOCK_DIFF_MAX)
        qm->req_clock = first_clock;
    list_add_tail(&qm->node, &sc->msg_queue);
    sc->last_step_clock = last_clock;

    // Create and store move in history tracking
    struct history_steps *hs = malloc(sizeof(*hs));
    hs->first_clock = first_clock;
    hs->last_clock = last_clock;
    hs->start_position = sc->last_position;
    hs->interval = move->interval;
    hs->add = move->add;
    hs->step_count = sc->sdir ? move->count : -move->count;
    sc->last_position += hs->step_count;
    list_add_head(&hs->node, &sc->history_list);
}

// Convert previously scheduled steps into commands for the mcu
static int
queue_flush(struct stepcompress *sc, uint64_t move_clock)
{
    if (sc->queue_pos >= sc->queue_next)
        return 0;
    while (sc->last_step_clock < move_clock) {
        struct step_move move = compress_bisect_add(sc);
        int ret = check_line(sc, move);
        if (ret)
            return ret;

        add_move(sc, sc->last_step_clock + move.interval, &move);

        if (sc->queue_pos + move.count >= sc->queue_next) {
            sc->queue_pos = sc->queue_next = sc->queue;
            break;
        }
        sc->queue_pos += move.count;
    }
    calc_last_step_print_time(sc);
    return 0;
}

// Generate a queue_step for a step far in the future from the last step
static int
stepcompress_flush_far(struct stepcompress *sc, uint64_t abs_step_clock)
{
    struct step_move move = { abs_step_clock - sc->last_step_clock, 1, 0 };
    add_move(sc, abs_step_clock, &move);
    calc_last_step_print_time(sc);
    return 0;
}

// Send the set_next_step_dir command
static int
set_next_step_dir(struct stepcompress *sc, int sdir)
{
    if (sc->sdir == sdir)
        return 0;
    int ret = queue_flush(sc, UINT64_MAX);
    if (ret)
        return ret;
    sc->sdir = sdir;
    uint32_t msg[3] = {
        sc->set_next_step_dir_msgtag, sc->oid, sdir ^ sc->invert_sdir
    };
    struct queue_message *qm = message_alloc_and_encode(msg, 3);
    qm->req_clock = sc->last_step_clock;
    list_add_tail(&qm->node, &sc->msg_queue);
    return 0;
}

// Slow path for queue_append() - handle next step far in future
static int
queue_append_far(struct stepcompress *sc)
{
    uint64_t step_clock = sc->next_step_clock;
    sc->next_step_clock = 0;
    int ret = queue_flush(sc, step_clock - CLOCK_DIFF_MAX + 1);
    if (ret)
        return ret;
    if (step_clock >= sc->last_step_clock + CLOCK_DIFF_MAX)
        return stepcompress_flush_far(sc, step_clock);
    *sc->queue_next++ = step_clock;
    return 0;
}

// Slow path for queue_append() - expand the internal queue storage
static int
queue_append_extend(struct stepcompress *sc)
{
    if (sc->queue_next - sc->queue_pos > 65535 + 2000) {
        // No point in keeping more than 64K steps in memory
        uint32_t flush = (*(sc->queue_next-65535)
                          - (uint32_t)sc->last_step_clock);
        int ret = queue_flush(sc, sc->last_step_clock + flush);
        if (ret)
            return ret;
    }

    if (sc->queue_next >= sc->queue_end) {
        // Make room in the queue
        int in_use = sc->queue_next - sc->queue_pos;
        if (sc->queue_pos > sc->queue) {
            // Shuffle the internal queue to avoid having to allocate more ram
            memmove(sc->queue, sc->queue_pos, in_use * sizeof(*sc->queue));
        } else {
            // Expand the internal queue of step times
            int alloc = sc->queue_end - sc->queue;
            if (!alloc)
                alloc = QUEUE_START_SIZE;
            while (in_use >= alloc)
                alloc *= 2;
            sc->queue = realloc(sc->queue, alloc * sizeof(*sc->queue));
            sc->queue_end = sc->queue + alloc;
        }
        sc->queue_pos = sc->queue;
        sc->queue_next = sc->queue + in_use;
    }

    *sc->queue_next++ = sc->next_step_clock;
    sc->next_step_clock = 0;
    return 0;
}

// Add a step time to the queue (flushing the queue if needed)
static int
queue_append(struct stepcompress *sc)
{
    if (unlikely(sc->next_step_dir != sc->sdir)) {
        int ret = set_next_step_dir(sc, sc->next_step_dir);
        if (ret)
            return ret;
    }
    if (unlikely(sc->next_step_clock >= sc->last_step_clock + CLOCK_DIFF_MAX))
        return queue_append_far(sc);
    if (unlikely(sc->queue_next >= sc->queue_end))
        return queue_append_extend(sc);
    *sc->queue_next++ = sc->next_step_clock;
    sc->next_step_clock = 0;
    return 0;
}

#define SDS_FILTER_TIME .000750

// Add next step time
int
stepcompress_append(struct stepcompress *sc, int sdir
                    , double print_time, double step_time)
{
    // Calculate step clock
    double offset = print_time - sc->last_step_print_time;
    double rel_sc = (step_time + offset) * sc->mcu_freq;
    uint64_t step_clock = sc->last_step_clock + (uint64_t)rel_sc;
    // Flush previous pending step (if any)
    if (sc->next_step_clock) {
        if (unlikely(sdir != sc->next_step_dir)) {
            double diff = (int64_t)(step_clock - sc->next_step_clock);
            if (diff < SDS_FILTER_TIME * sc->mcu_freq) {
                // Rollback last step to avoid rapid step+dir+step
                sc->next_step_clock = 0;
                sc->next_step_dir = sdir;
                return 0;
            }
        }
        int ret = queue_append(sc);
        if (ret)
            return ret;
    }
    // Store this step as the next pending step
    sc->next_step_clock = step_clock;
    sc->next_step_dir = sdir;
    return 0;
}

// Commit next pending step (ie, do not allow a rollback)
int
stepcompress_commit(struct stepcompress *sc)
{
    if (sc->next_step_clock)
        return queue_append(sc);
    return 0;
}

// Flush pending steps
static int
stepcompress_flush(struct stepcompress *sc, uint64_t move_clock)
{
    if (sc->next_step_clock && move_clock >= sc->next_step_clock) {
        int ret = queue_append(sc);
        if (ret)
            return ret;
    }
    return queue_flush(sc, move_clock);
}

// Reset the internal state of the stepcompress object
int __visible
stepcompress_reset(struct stepcompress *sc, uint64_t last_step_clock)
{
    int ret = stepcompress_flush(sc, UINT64_MAX);
    if (ret)
        return ret;
    sc->last_step_clock = last_step_clock;
    sc->sdir = -1;
    calc_last_step_print_time(sc);
    return 0;
}

// Set last_position in the stepcompress object
int __visible
stepcompress_set_last_position(struct stepcompress *sc, uint64_t clock
                               , int64_t last_position)
{
    int ret = stepcompress_flush(sc, UINT64_MAX);
    if (ret)
        return ret;
    sc->last_position = last_position;

    // Add a marker to the history list
    struct history_steps *hs = malloc(sizeof(*hs));
    memset(hs, 0, sizeof(*hs));
    hs->first_clock = hs->last_clock = clock;
    hs->start_position = last_position;
    list_add_head(&hs->node, &sc->history_list);
    return 0;
}

// Search history of moves to find a past position at a given clock
int64_t __visible
stepcompress_find_past_position(struct stepcompress *sc, uint64_t clock)
{
    int64_t last_position = sc->last_position;
    struct history_steps *hs;
    list_for_each_entry(hs, &sc->history_list, node) {
        if (clock < hs->first_clock) {
            last_position = hs->start_position;
            continue;
        }
        if (clock >= hs->last_clock)
            return hs->start_position + hs->step_count;
        int32_t interval = hs->interval, add = hs->add;
        int32_t ticks = (int32_t)(clock - hs->first_clock) + interval, offset;
        if (!add) {
            offset = ticks / interval;
        } else {
            // Solve for "count" using quadratic formula
            double a = .5 * add, b = interval - .5 * add, c = -ticks;
            offset = (sqrt(b*b - 4*a*c) - b) / (2. * a);
        }
        if (hs->step_count < 0)
            return hs->start_position - offset;
        return hs->start_position + offset;
    }
    return last_position;
}

// Queue an mcu command to go out in order with stepper commands
int __visible
stepcompress_queue_msg(struct stepcompress *sc, uint32_t *data, int len)
{
    int ret = stepcompress_flush(sc, UINT64_MAX);
    if (ret)
        return ret;

    struct queue_message *qm = message_alloc_and_encode(data, len);
    qm->req_clock = sc->last_step_clock;
    list_add_tail(&qm->node, &sc->msg_queue);
    return 0;
}

// Queue an mcu command that will consume space in the mcu move queue
int __visible
stepcompress_queue_mq_msg(struct stepcompress *sc, uint64_t req_clock
                          , uint32_t *data, int len)
{
    int ret = stepcompress_flush(sc, UINT64_MAX);
    if (ret)
        return ret;

    struct queue_message *qm = message_alloc_and_encode(data, len);
    qm->min_clock = qm->req_clock = req_clock;
    list_add_tail(&qm->node, &sc->msg_queue);
    return 0;
}

// Return history of queue_step commands
int __visible
stepcompress_extract_old(struct stepcompress *sc, struct pull_history_steps *p
                         , int max, uint64_t start_clock, uint64_t end_clock)
{
    int res = 0;
    struct history_steps *hs;
    list_for_each_entry(hs, &sc->history_list, node) {
        if (start_clock >= hs->last_clock || res >= max)
            break;
        if (end_clock <= hs->first_clock)
            continue;
        p->first_clock = hs->first_clock;
        p->last_clock = hs->last_clock;
        p->start_position = hs->start_position;
        p->step_count = hs->step_count;
        p->interval = hs->interval;
        p->add = hs->add;
        p++;
        res++;
    }
    return res;
}


/****************************************************************
 * Step compress synchronization
 ****************************************************************/

// The steppersync object is used to synchronize the output of mcu
// step commands.  The mcu can only queue a limited number of step
// commands - this code tracks when items on the mcu step queue become
// free so that new commands can be transmitted.  It also ensures the
// mcu step queue is ordered between steppers so that no stepper
// starves the other steppers of space in the mcu step queue.

struct steppersync {
    // Serial port
    struct serialqueue *sq;
    struct command_queue *cq;
    // Storage for associated stepcompress objects
    struct stepcompress **sc_list;
    int sc_num;
    // Storage for list of pending move clocks
    uint64_t *move_clocks;
    int num_move_clocks;
};

// Allocate a new 'steppersync' object
struct steppersync * __visible
steppersync_alloc(struct serialqueue *sq, struct stepcompress **sc_list
                  , int sc_num, int move_num)
{
    struct steppersync *ss = malloc(sizeof(*ss));
    memset(ss, 0, sizeof(*ss));
    ss->sq = sq;
    ss->cq = serialqueue_alloc_commandqueue();

    ss->sc_list = malloc(sizeof(*sc_list)*sc_num);
    memcpy(ss->sc_list, sc_list, sizeof(*sc_list)*sc_num);
    ss->sc_num = sc_num;

    ss->move_clocks = malloc(sizeof(*ss->move_clocks)*move_num);
    memset(ss->move_clocks, 0, sizeof(*ss->move_clocks)*move_num);
    ss->num_move_clocks = move_num;

    return ss;
}

// Free memory associated with a 'steppersync' object
void __visible
steppersync_free(struct steppersync *ss)
{
    if (!ss)
        return;
    free(ss->sc_list);
    free(ss->move_clocks);
    serialqueue_free_commandqueue(ss->cq);
    free(ss);
}

// Set the conversion rate of 'print_time' to mcu clock
void __visible
steppersync_set_time(struct steppersync *ss, double time_offset
                     , double mcu_freq)
{
    int i;
    for (i=0; i<ss->sc_num; i++) {
        struct stepcompress *sc = ss->sc_list[i];
        stepcompress_set_time(sc, time_offset, mcu_freq);
    }
}

// Expire the stepcompress history before the given clock time
static void
steppersync_history_expire(struct steppersync *ss, uint64_t end_clock)
{
    int i;
    for (i = 0; i < ss->sc_num; i++)
    {
        struct stepcompress *sc = ss->sc_list[i];
        stepcompress_history_expire(sc, end_clock);
    }
}

// Implement a binary heap algorithm to track when the next available
// 'struct move' in the mcu will be available
static void
heap_replace(struct steppersync *ss, uint64_t req_clock)
{
    uint64_t *mc = ss->move_clocks;
    int nmc = ss->num_move_clocks, pos = 0;
    for (;;) {
        int child1_pos = 2*pos+1, child2_pos = 2*pos+2;
        uint64_t child2_clock = child2_pos < nmc ? mc[child2_pos] : UINT64_MAX;
        uint64_t child1_clock = child1_pos < nmc ? mc[child1_pos] : UINT64_MAX;
        if (req_clock <= child1_clock && req_clock <= child2_clock) {
            mc[pos] = req_clock;
            break;
        }
        if (child1_clock < child2_clock) {
            mc[pos] = child1_clock;
            pos = child1_pos;
        } else {
            mc[pos] = child2_clock;
            pos = child2_pos;
        }
    }
}

// Find and transmit any scheduled steps prior to the given 'move_clock'
int __visible
steppersync_flush(struct steppersync *ss, uint64_t move_clock
                  , uint64_t clear_history_clock)
{
    // Flush each stepcompress to the specified move_clock
    int i;
    for (i=0; i<ss->sc_num; i++) {
        int ret = stepcompress_flush(ss->sc_list[i], move_clock);
        if (ret)
            return ret;
    }

    // Order commands by the reqclock of each pending command
    struct list_head msgs;
    list_init(&msgs);
    for (;;) {
        // Find message with lowest reqclock
        uint64_t req_clock = MAX_CLOCK;
        struct queue_message *qm = NULL;
        for (i=0; i<ss->sc_num; i++) {
            struct stepcompress *sc = ss->sc_list[i];
            if (!list_empty(&sc->msg_queue)) {
                struct queue_message *m = list_first_entry(
                    &sc->msg_queue, struct queue_message, node);
                if (m->req_clock < req_clock) {
                    qm = m;
                    req_clock = m->req_clock;
                }
            }
        }
        if (!qm || (qm->min_clock && req_clock > move_clock))
            break;

        uint64_t next_avail = ss->move_clocks[0];
        if (qm->min_clock)
            // The qm->min_clock field is overloaded to indicate that
            // the command uses the 'move queue' and to store the time
            // that move queue item becomes available.
            heap_replace(ss, qm->min_clock);
        // Reset the min_clock to its normal meaning (minimum transmit time)
        qm->min_clock = next_avail;

        // Batch this command
        list_del(&qm->node);
        list_add_tail(&qm->node, &msgs);
    }

    // Transmit commands
    if (!list_empty(&msgs))
        serialqueue_send_batch(ss->sq, ss->cq, &msgs);

    steppersync_history_expire(ss, clear_history_clock);
    return 0;
}
