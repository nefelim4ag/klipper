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
};

void __visible
stealthchop_shaper(struct stepper_kinematics *sk, double L, double Ohm)
{
    struct stepper_shaper *ss = container_of(sk, struct stepper_shaper, sk);
    ss->motor_constant = L / Ohm;

}

double __visible
stepper_shaper_get_step_generation_window(struct stepper_kinematics *sk)
{
    struct stepper_shaper *ss = container_of(sk, struct stepper_shaper, sk);
    return ss->sk.gen_steps_pre_active > ss->sk.gen_steps_post_active
               ? ss->sk.gen_steps_pre_active
               : ss->sk.gen_steps_post_active;
}

inline double
move_get_speed(struct move *m, double move_time)
{
    return m->start_v + 2 * m->half_accel * move_time;
}

static double
shaper_calc_position(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    struct stepper_shaper *ss = container_of(sk, struct stepper_shaper, sk);
    double cur = ss->orig_sk->calc_position_cb(ss->orig_sk, m, move_time);
    double next = ss->orig_sk->calc_position_cb(ss->orig_sk, m, move_time + 0.00001);
    double diff = next - cur;
    double speed = diff / 0.000001;
    double speed2 = move_get_speed(m, move_time);
    fprintf(stderr, "diff: %f, speed: %f, speed2: %f\n", diff, speed, speed2);
    if (ss->motor_constant == .0)
        return ss->orig_sk->calc_position_cb(ss->orig_sk, m, move_time);

}

static void
commanded_pos_post_fixup(struct stepper_kinematics *sk)
{
    struct stepper_shaper *ss = container_of(sk, struct stepper_shaper, sk);
    ss->orig_sk->commanded_pos = sk->commanded_pos;
    ss->orig_sk->post_cb(ss->orig_sk);
    sk->commanded_pos = ss->orig_sk->commanded_pos;
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
    if (orig_sk->post_cb)
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
