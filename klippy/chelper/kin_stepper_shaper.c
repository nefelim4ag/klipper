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
#define USec 0.000001
#define MIN_SPEED 0.000000001

void __visible
stealthchop_shaper(struct stepper_kinematics *sk, double L, double Ohm
                    , double rt_dist, int fsteps_per_rev)
{
    struct stepper_shaper *ss = container_of(sk, struct stepper_shaper, sk);
    ss->motor_constant = L / Ohm;
    int phase_rev_per_rev = fsteps_per_rev / 4;
    ss->fstep_dist = rt_dist / fsteps_per_rev;
    ss->phase_dist = rt_dist / phase_rev_per_rev;
    double omega_min = M_2PI * MIN_SPEED;
    double max_tlag = atan(omega_min * ss->motor_constant)/omega_min;
    ss->sk.gen_steps_pre_active = max_tlag;
}

double __visible
stepper_shaper_get_step_generation_window(struct stepper_kinematics *sk)
{
    struct stepper_shaper *ss = container_of(sk, struct stepper_shaper, sk);
    return ss->sk.gen_steps_pre_active > ss->sk.gen_steps_post_active
               ? ss->sk.gen_steps_pre_active
               : ss->sk.gen_steps_post_active;
}

static double
guess_speed(struct stepper_shaper *ss, struct move *m, double move_time)
{
    double cur = ss->orig_sk->calc_position_cb(ss->orig_sk, m, move_time);
    double next = ss->orig_sk->calc_position_cb(ss->orig_sk, m, move_time + USec);
    double diff = next - cur;
    double speed = diff / USec;
    // // Full stop probably have the same max lag.
    // // Avoid time/positional jumps
    // if (speed == .0)
    //     speed = MIN_SPEED;
    return speed;
}

static double
shaper_calc_position(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    struct stepper_shaper *ss = container_of(sk, struct stepper_shaper, sk);
    double res = ss->orig_sk->calc_position_cb(ss->orig_sk, m, move_time);
    if (ss->motor_constant == .0)
        return res;
    double speed = guess_speed(ss, m, move_time);
    double phase_speed = speed / ss->phase_dist;
    double omega = M_2PI * phase_speed;
    double rads = atan(omega * ss->motor_constant);
    double degrees = rads * 180 / M_PI;
    // better fit encoder data at lower speeds
    double dist_lag = ss->fstep_dist * sin(degrees * M_PI / 180);
    // double tlag = rads / omega;
    // double dist_lag_2 = speed * tlag;
    if (speed == .0)
        dist_lag = .0;
    // double move_time_past = move_time - tlag;
    // // search move
    // while (likely(move_time_past < 0.)){
    //     m = list_prev_entry(m, node);
    //     move_time_past += m->move_t;
    // }
    // double new_res = ss->orig_sk->calc_position_cb(ss->orig_sk, m, move_time_past);
    // double guess_dist = res - new_res;
    // fprintf(stderr, "old: %f speed: %f, tlag: %f-> move_time: %f\n", move_time, speed, tlag, move_time_past);
    // fprintf(stderr, "speed: %f, pos_corrected: %f, dlag: %f\n", move_time, res - dist_lag, dist_lag);
    // fprintf(stderr, "compensation distance: %f, guess: %f, guess 2: %f\n", guess_dist, dist_lag, dist_lag_2);
    return res + dist_lag;
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
