// Cartesian kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "integrate.h" // calc_smoothed_velocity
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

static double
cart_stepper_x_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).x;
}

static double
cart_stepper_x_calc_velocity(struct stepper_kinematics *sk, struct move *m
                             , double move_time, double hst)
{
    return calc_smoothed_velocity(m, 'x', move_time, hst);
}

static double
cart_stepper_y_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).y;
}

static double
cart_stepper_y_calc_velocity(struct stepper_kinematics *sk, struct move *m
                             , double move_time, double hst)
{
    return calc_smoothed_velocity(m, 'y', move_time, hst);
}

static double
cart_stepper_z_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).z;
}

static double
cart_stepper_z_calc_velocity(struct stepper_kinematics *sk, struct move *m
                             , double move_time, double hst)
{
    return calc_smoothed_velocity(m, 'z', move_time, hst);
}

struct stepper_kinematics * __visible
cartesian_stepper_alloc(char axis)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    if (axis == 'x') {
        sk->calc_position_cb = cart_stepper_x_calc_position;
        sk->calc_smoothed_velocity_cb = cart_stepper_x_calc_velocity;
        sk->active_flags = AF_X;
    } else if (axis == 'y') {
        sk->calc_position_cb = cart_stepper_y_calc_position;
        sk->calc_smoothed_velocity_cb = cart_stepper_y_calc_velocity;
        sk->active_flags = AF_Y;
    } else if (axis == 'z') {
        sk->calc_position_cb = cart_stepper_z_calc_position;
        sk->calc_smoothed_velocity_cb = cart_stepper_z_calc_velocity;
        sk->active_flags = AF_Z;
    }
    return sk;
}
