// Idex dual carriage kinematics
//
// Copyright (C) 2023  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h>  // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "integrate.h" // calc_smoothed_velocity
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // struct move

#define ZERO_SMOOTH_T 0.0
#define DUMMY_T 500.0

struct dual_carriage_stepper {
    struct stepper_kinematics sk;
    struct stepper_kinematics *orig_sk;
    struct move m;
    double x_scale, x_offs, y_scale, y_offs;
};

double
dual_carriage_calc_position(struct stepper_kinematics *sk, struct move *m
                            , double move_time)
{
    struct dual_carriage_stepper *dc = container_of(
            sk, struct dual_carriage_stepper, sk);
    struct coord pos = move_get_coord(m, move_time);
    dc->m.start_pos.x = pos.x * dc->x_scale + dc->x_offs;
    dc->m.start_pos.y = pos.y * dc->y_scale + dc->y_offs;
    dc->m.start_pos.z = pos.z;
    dc->m.start_v = 0.;
    return dc->orig_sk->calc_position_cb(dc->orig_sk, &dc->m, DUMMY_T);
}

double
dual_carriage_calc_velocity(struct stepper_kinematics *sk, struct move *m
                            , double move_time, double hst)
{
    struct dual_carriage_stepper *dc = container_of(
            sk, struct dual_carriage_stepper, sk);
    if (!hst) {
        dc->m.axes_r.x = m->axes_r.x * dc->x_scale;
        dc->m.axes_r.y = m->axes_r.y * dc->y_scale;
        dc->m.axes_r.z = m->axes_r.z;
        dc->m.start_v = move_get_velocity(m, move_time) * sqrt(
                dc->m.axes_r.x * dc->m.axes_r.x +
                dc->m.axes_r.y * dc->m.axes_r.y +
                dc->m.axes_r.z * dc->m.axes_r.z);
        return dc->orig_sk->calc_smoothed_velocity_cb(
                dc->orig_sk, &dc->m, DUMMY_T, ZERO_SMOOTH_T);
    }
    double v_x = calc_smoothed_velocity(m, 'x', move_time, hst) * dc->x_scale;
    double v_y = calc_smoothed_velocity(m, 'y', move_time, hst) * dc->y_scale;
    double v_z = calc_smoothed_velocity(m, 'z', move_time, hst);
    double v_nrm = sqrt(v_x * v_x + v_y * v_y + v_z * v_z);
    if (v_nrm < 1e-10) {
        memset(&dc->m.axes_r, 0, sizeof(dc->m.axes_r));
        dc->m.start_v = 0.;
    } else {
        double v_recipr = 1. / v_nrm;
        dc->m.start_v = v_nrm;
        dc->m.axes_r.x = v_x * v_recipr;
        dc->m.axes_r.y = v_y * v_recipr;
        dc->m.axes_r.z = v_z * v_recipr;
    }
    // Velocity smoothing was already applied
    return dc->orig_sk->calc_smoothed_velocity_cb(
            dc->orig_sk, &dc->m, DUMMY_T, ZERO_SMOOTH_T);
}

void __visible
dual_carriage_set_sk(struct stepper_kinematics *sk
                     , struct stepper_kinematics *orig_sk)
{
    struct dual_carriage_stepper *dc = container_of(
            sk, struct dual_carriage_stepper, sk);
    dc->sk.calc_position_cb = dual_carriage_calc_position;
    if (orig_sk->calc_smoothed_velocity_cb)
        dc->sk.calc_smoothed_velocity_cb = dual_carriage_calc_velocity;
    dc->sk.active_flags = orig_sk->active_flags;
    dc->orig_sk = orig_sk;
}

int __visible
dual_carriage_set_transform(struct stepper_kinematics *sk, char axis
                            , double scale, double offs)
{
    struct dual_carriage_stepper *dc = container_of(
            sk, struct dual_carriage_stepper, sk);
    if (axis == 'x') {
        dc->x_scale = scale;
        dc->x_offs = offs;
        if (!scale)
            dc->sk.active_flags &= ~AF_X;
        else if (scale && dc->orig_sk->active_flags & AF_X)
            dc->sk.active_flags |= AF_X;
        return 0;
    }
    if (axis == 'y') {
        dc->y_scale = scale;
        dc->y_offs = offs;
        if (!scale)
            dc->sk.active_flags &= ~AF_Y;
        else if (scale && dc->orig_sk->active_flags & AF_Y)
            dc->sk.active_flags |= AF_Y;
        return 0;
    }
    return -1;
}

struct stepper_kinematics * __visible
dual_carriage_alloc(void)
{
    struct dual_carriage_stepper *dc = malloc(sizeof(*dc));
    memset(dc, 0, sizeof(*dc));
    dc->m.move_t = 2. * DUMMY_T;
    dc->m.axes_r.x = 1.;
    dc->m.axes_r.y = 1.;
    dc->x_scale = dc->y_scale = 1.0;
    return &dc->sk;
}
