// Stepper position correction for motor torque ripple
//
// Copyright (C) 2026  Maksim Bolgov <maksim8024@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // cos
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // struct move

#define MAX_HARMONIC 4
// Time span used to calculate the stepper velocity of the kinematics
// without a velocity callback
#define VELOCITY_TIME 0.0005
// Square of the velocity in mm/s below which the direction is blended
#define DIRECTION_SMOOTH_V2 4.
// Speed range above max_speed, relative to it, in which the correction fades
#define FADE_SPEED_RATIO .1

struct stepper_ripple {
    struct stepper_kinematics sk;
    struct stepper_kinematics *orig_sk, *kin_sk;
    double origin, inv_period;
    int count;
    // Speed model of all the harmonics, an f0 of 0 keeps the low speed
    // correction at all speeds, a max_speed of 0 does not fade it
    double f0, inv_q, delay, max_speed;
    // Cos and sin of the positive and the negative direction at low and
    // high speed, the offset is added without the speed model
    double low[MAX_HARMONIC][4], high[MAX_HARMONIC][4];
    double offset[MAX_HARMONIC][4];
};

// Return the move and the move time of a time that may be outside of the move
static struct move *
stepper_ripple_find_move(struct move *m, double *move_time)
{
    while (*move_time < 0.) {
        m = list_prev_entry(m, node);
        *move_time += m->move_t;
    }
    while (*move_time > m->move_t) {
        *move_time -= m->move_t;
        m = list_next_entry(m, node);
    }
    return m;
}

// Return the stepper velocity before input shaping
static double
stepper_ripple_velocity(struct stepper_ripple *rs, struct move *m
                        , double move_time)
{
    struct stepper_kinematics *ks = rs->kin_sk;
    if (ks->calc_velocity_cb)
        return ks->calc_velocity_cb(ks, m, move_time);
    // Average velocity around the time
    double t1 = move_time - VELOCITY_TIME, t2 = move_time + VELOCITY_TIME;
    struct move *m1 = stepper_ripple_find_move(m, &t1);
    struct move *m2 = stepper_ripple_find_move(m, &t2);
    return ((ks->calc_position_cb(ks, m2, t2)
             - ks->calc_position_cb(ks, m1, t1)) / (2. * VELOCITY_TIME));
}

// Return the toolhead speed along the axes that move the stepper, it is
// zero when the stepper is not active
static double
stepper_ripple_head_speed(struct stepper_ripple *rs, struct move *m
                          , double move_time)
{
    int af = rs->sk.active_flags;
    double rx = af & AF_X ? m->axes_r.x : 0.;
    double ry = af & AF_Y ? m->axes_r.y : 0.;
    double rz = af & AF_Z ? m->axes_r.z : 0.;
    return (fabs(move_get_velocity(m, move_time))
            * sqrt(rx*rx + ry*ry + rz*rz));
}

// Correction coefficients of harmonic index 'i' at a speed, the cos and
// sin of the positive and the negative direction
static void
stepper_ripple_coeffs(struct stepper_ripple *rs, int i, double speed
                      , double *c)
{
    double *lo = rs->low[i], *hi = rs->high[i], *of = rs->offset[i];
    double f = (i + 1) * speed * rs->inv_period;
    // Response h of f0^2 / (f0^2 - f^2 + j*f*f0/q)
    double h_re = 1., h_im = 0.;
    if (rs->f0 > 0.) {
        double x = f / rs->f0, re = 1. - x * x, im = x * rs->inv_q;
        double inv = 1. / (re * re + im * im);
        h_re = re * inv;
        h_im = -im * inv;
    }
    double phase = -2. * M_PI * f * rs->delay;
    double dc = cos(phase), ds = sin(phase);
    int d;
    for (d = 0; d < 4; d += 2) {
        // high + (low - high) * h, then delayed
        double ar = lo[d] - hi[d], ai = lo[d+1] - hi[d+1];
        double cr = hi[d] + ar * h_re - ai * h_im;
        double ci = hi[d+1] + ar * h_im + ai * h_re;
        c[d] = cr * dc - ci * ds + of[d];
        c[d+1] = cr * ds + ci * dc + of[d+1];
    }
}

// Calculate the position correction at a given stepper position
static double
stepper_ripple_offset(struct stepper_ripple *rs, double pos, double velocity
                      , double head_speed)
{
    double speed = fabs(velocity);
    double head_v2 = head_speed * head_speed;
    double fade = head_v2 / (head_v2 + DIRECTION_SMOOTH_V2);
    if (rs->max_speed > 0. && speed > rs->max_speed) {
        double x = (speed / rs->max_speed - 1.) / FADE_SPEED_RATIO;
        if (x >= 1.)
            return 0.;
        fade *= .5 * (1. + cos(M_PI * x));
    }
    double direction = velocity / sqrt(velocity * velocity
                                       + DIRECTION_SMOOTH_V2);
    double cycles = (pos - rs->origin) * rs->inv_period;
    double phase = 2. * M_PI * (cycles - floor(cycles));
    double c1 = cos(phase), s1 = sin(phase), ck = c1, sk = s1, res = 0.;
    int i;
    for (i = 0; i < rs->count; i++) {
        // Blend the directions, only near zero speed both matter
        double c[4], w = .5 * (1. + direction);
        stepper_ripple_coeffs(rs, i, speed, c);
        double a = w * c[0] + (1. - w) * c[2];
        double b = w * c[1] + (1. - w) * c[3];
        res += a * ck - b * sk;
        // cos and sin of (i + 2) * phase
        double ck_next = ck * c1 - sk * s1;
        sk = sk * c1 + ck * s1;
        ck = ck_next;
    }
    return fade * res;
}

static double
stepper_ripple_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    struct stepper_ripple *rs = container_of(sk, struct stepper_ripple, sk);
    double pos = rs->orig_sk->calc_position_cb(rs->orig_sk, m, move_time);
    if (!rs->count)
        return pos;
    double velocity = stepper_ripple_velocity(rs, m, move_time);
    double head_speed = stepper_ripple_head_speed(rs, m, move_time);
    return pos + stepper_ripple_offset(rs, pos, velocity, head_speed);
}

// A callback that forwards post_cb call to the original kinematics
static void
stepper_ripple_commanded_pos_post_fixup(struct stepper_kinematics *sk)
{
    struct stepper_ripple *rs = container_of(sk, struct stepper_ripple, sk);
    rs->orig_sk->commanded_pos = sk->commanded_pos;
    rs->orig_sk->post_cb(rs->orig_sk);
    sk->commanded_pos = rs->orig_sk->commanded_pos;
}

// Copy the step generation settings of the wrapped kinematics
void __visible
stepper_ripple_update_sk(struct stepper_kinematics *sk)
{
    struct stepper_ripple *rs = container_of(sk, struct stepper_ripple, sk);
    struct stepper_kinematics *orig_sk = rs->orig_sk;
    rs->sk.active_flags = orig_sk->active_flags;
    rs->sk.gen_steps_pre_active = fmax(orig_sk->gen_steps_pre_active
                                       , VELOCITY_TIME);
    rs->sk.gen_steps_post_active = fmax(orig_sk->gen_steps_post_active
                                        , VELOCITY_TIME);
}

// Wrap 'orig_sk', the velocity is calculated with 'kin_sk', the kinematics
// without input shaping
int __visible
stepper_ripple_set_sk(struct stepper_kinematics *sk
                      , struct stepper_kinematics *orig_sk
                      , struct stepper_kinematics *kin_sk)
{
    struct stepper_ripple *rs = container_of(sk, struct stepper_ripple, sk);
    if (!orig_sk->calc_position_cb || !kin_sk->calc_position_cb)
        return -1;
    rs->orig_sk = orig_sk;
    rs->kin_sk = kin_sk;
    rs->sk.calc_position_cb = stepper_ripple_calc_position;
    stepper_ripple_update_sk(sk);
    rs->sk.commanded_pos = orig_sk->commanded_pos;
    rs->sk.last_flush_time = orig_sk->last_flush_time;
    rs->sk.last_move_time = orig_sk->last_move_time;
    if (orig_sk->post_cb)
        rs->sk.post_cb = stepper_ripple_commanded_pos_post_fixup;
    return 0;
}

void __visible
stepper_ripple_set_phase(struct stepper_kinematics *sk, double origin
                         , double period)
{
    struct stepper_ripple *rs = container_of(sk, struct stepper_ripple, sk);
    rs->origin = origin;
    rs->inv_period = 1. / period;
}

// Set the speed model, an f0 of 0 disables it
int __visible
stepper_ripple_set_model(struct stepper_kinematics *sk, double f0, double q
                         , double delay, double max_speed)
{
    struct stepper_ripple *rs = container_of(sk, struct stepper_ripple, sk);
    if (f0 < 0. || (f0 > 0. && q <= 0.) || max_speed < 0.)
        return -1;
    rs->f0 = f0;
    rs->inv_q = f0 > 0. ? 1. / q : 0.;
    rs->delay = delay;
    rs->max_speed = max_speed;
    return 0;
}

// Set the correction of harmonic 'k' in mm, each array holds the cos and
// sin of the positive and then of the negative direction
int __visible
stepper_ripple_set_harmonic(struct stepper_kinematics *sk, int k
                            , double low[], double high[], double offset[])
{
    struct stepper_ripple *rs = container_of(sk, struct stepper_ripple, sk);
    if (k < 1 || k > MAX_HARMONIC)
        return -1;
    int h = k - 1, c, count = 0;
    for (c = 0; c < 4; c++) {
        rs->low[h][c] = low[c];
        rs->high[h][c] = high[c];
        rs->offset[h][c] = offset[c];
    }
    for (h = 0; h < MAX_HARMONIC; h++)
        for (c = 0; c < 4; c++)
            if (rs->low[h][c] != 0. || rs->high[h][c] != 0.
                || rs->offset[h][c] != 0.)
                count = h + 1;
    rs->count = count;
    return 0;
}

struct stepper_kinematics * __visible
stepper_ripple_alloc(void)
{
    struct stepper_ripple *rs = malloc(sizeof(*rs));
    memset(rs, 0, sizeof(*rs));
    rs->inv_period = 1.;
    return &rs->sk;
}
