// Stepper positional lag compensation

#include <math.h>      // sqrt, exp
#include <stddef.h>    // offsetof
#include <stdlib.h>    // malloc
#include <string.h>    // memset
#include "compiler.h"  // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h"     // struct move
#include "stdio.h"

struct stepper_shaper {
    struct stepper_kinematics sk;
    struct stepper_kinematics *orig_sk;
    // StealthChop compensation
    double motor_constant;
    double phase_dist;
    double fstep_dist;
};

#define M_2PI (2 * M_PI)
#define MIN_SPEED 0.000000001

void __visible
stealthchop_shaper(struct stepper_kinematics *sk, double L, double Ohm,
                   double rt_dist, int fsteps_per_rev)
{
    struct stepper_shaper *ss = container_of(sk, struct stepper_shaper, sk);
    ss->motor_constant = L / Ohm;
    int phase_rev_per_rev = fsteps_per_rev / 4;
    ss->fstep_dist = rt_dist / fsteps_per_rev;
    ss->phase_dist = rt_dist / phase_rev_per_rev;
    double omega_min = M_2PI * MIN_SPEED;
    double max_tlag = atan(omega_min * ss->motor_constant)/omega_min;
    ss->sk.gen_steps_pre_active = max_tlag;
    ss->sk.gen_steps_post_active = max_tlag;
}

double __visible
stepper_shaper_get_step_generation_window(struct stepper_kinematics *sk)
{
    struct stepper_shaper *ss = container_of(sk, struct stepper_shaper, sk);
    return ss->sk.gen_steps_pre_active > ss->sk.gen_steps_post_active
               ? ss->sk.gen_steps_pre_active
               : ss->sk.gen_steps_post_active;
}

static inline double
get_axis_position_across_moves(struct stepper_kinematics* sk, struct move* m,
                               double time)
{
    while (time < 0.) {
        m = list_prev_entry(m, node);
        time += m->move_t;
    }
    while (time > m->move_t) {
        time -= m->move_t;
        m = list_next_entry(m, node);
    }
    return sk->calc_position_cb(sk, m, time);
}

static double
guess_speed_smooth(struct stepper_shaper* ss, struct move* m,
                   double move_time, double cur)
{
    double h = ss->sk.gen_steps_pre_active / 2;
    struct stepper_kinematics* sk = ss->orig_sk;
    // sample positions at t±h and t±2h
    double p_m2 = get_axis_position_across_moves(sk, m, move_time - 2 * h);
    double p_m1 = get_axis_position_across_moves(sk, m, move_time - h);
    double p_p1 = get_axis_position_across_moves(sk, m, move_time + h);
    double p_p2 = get_axis_position_across_moves(sk, m, move_time + 2 * h);

    // compute the 5-point derivative/5 point stencil
    double speed = (p_m2 - 8.0 * p_m1 + 8.0 * p_p1 - p_p2) / (12.0 * h);

    return speed;
}


static double
shaper_calc_position(struct stepper_kinematics *sk, struct move *m,
                     double move_time)
{
    struct stepper_shaper *ss = container_of(sk, struct stepper_shaper, sk);
    double res = ss->orig_sk->calc_position_cb(ss->orig_sk, m, move_time);
    if (ss->motor_constant == .0)
        return res;
    double speed = guess_speed_smooth(ss, m, move_time, res);

    double phase_speed = speed / ss->phase_dist;
    double omega = M_2PI * phase_speed;
    double rads = atan(omega * ss->motor_constant);
    double dist_lag = ss->fstep_dist * sin(rads);
    double res_cor = res + dist_lag;
    return res_cor;
}

static void
commanded_pos_post_fixup(struct stepper_kinematics *sk)
{
    struct stepper_shaper *ss = container_of(sk, struct stepper_shaper, sk);
    ss->orig_sk->commanded_pos = sk->commanded_pos;
    ss->orig_sk->last_flush_time = sk->last_flush_time;
    ss->orig_sk->last_move_time = sk->last_move_time;
    if (ss->orig_sk->post_cb) {
        ss->orig_sk->post_cb(ss->orig_sk);
        sk->commanded_pos = ss->orig_sk->commanded_pos;
    }
}

int __visible
stepper_shaper_set_sk(struct stepper_kinematics *sk,
                      struct stepper_kinematics *orig_sk)
{
    struct stepper_shaper *ss = container_of(sk, struct stepper_shaper, sk);
    ss->sk.calc_position_cb = shaper_calc_position;
    ss->sk.active_flags = orig_sk->active_flags;
    ss->orig_sk = orig_sk;
    ss->sk.commanded_pos = orig_sk->commanded_pos;
    ss->sk.last_flush_time = orig_sk->last_flush_time;
    ss->sk.last_move_time = orig_sk->last_move_time;
    ss->sk.post_cb = commanded_pos_post_fixup;
    return 0;
}

struct stepper_kinematics * __visible
stepper_shaper_alloc(void)
{
    struct stepper_shaper *ss = malloc(sizeof(*ss));
    memset(ss, 0, sizeof(*ss));
    ss->sk.gen_steps_pre_active = .0;
    ss->sk.gen_steps_post_active = .0;
    ss->motor_constant = .0;
    return &ss->sk;
}
