// Independent center-aware Polar stepper allocator for Klipper.
// Distributed under the GNU GPLv3 license.
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "itersolve.h"
#include "trapq.h"

// Kept local so this plugin has no runtime dependency on non-exported symbols
// in Klipper's c_helper.so.
static struct coord
plugin_move_get_coord(struct move *m, double move_time)
{
    double move_dist = (m->start_v + m->half_accel * move_time) * move_time;
    return (struct coord) {
        .x = m->start_pos.x + m->axes_r.x * move_dist,
        .y = m->start_pos.y + m->axes_r.y * move_dist,
        .z = m->start_pos.z + m->axes_r.z * move_dist };
}

static double
polar_center_angle_calc_position(struct stepper_kinematics *sk, struct move *m,
                                 double move_time)
{
    struct coord c = plugin_move_get_coord(m, move_time);
    if (hypot(c.x, c.y) <= 1.e-9) {
        c = m->start_pos;
        if (hypot(c.x, c.y) <= 1.e-9)
            return sk->commanded_pos;
    }
    double angle = atan2(c.y, c.x);
    return sk->commanded_pos + remainder(angle - sk->commanded_pos, 2. * M_PI);
}

struct stepper_kinematics *
polar_center_stepper_alloc(void)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    if (!sk)
        return NULL;
    memset(sk, 0, sizeof(*sk));
    sk->calc_position_cb = polar_center_angle_calc_position;
    sk->active_flags = AF_X | AF_Y;
    return sk;
}

// Native XY arc parameterized by true arc length carried in virtual X.
struct arc_stepper {
    struct stepper_kinematics sk;
    double cx, cy, radius, angle, inverse_radius, radial_bias, angular_bias;
};

static struct coord
arc_coord(struct stepper_kinematics *sk, struct move *m, double time)
{
    struct arc_stepper *a = (void *)sk;
    double phi = a->angle + plugin_move_get_coord(m, time).x*a->inverse_radius;
    return (struct coord){.x=a->cx+a->radius*cos(phi),
                          .y=a->cy+a->radius*sin(phi)};
}

static double
arc_radius(struct stepper_kinematics *sk, struct move *m, double time)
{
    struct coord c = arc_coord(sk, m, time);
    return hypot(c.x, c.y) + ((struct arc_stepper *)sk)->radial_bias;
}

static double
arc_unwrapped_angle(struct arc_stepper *a, double phi)
{
    double center_r = hypot(a->cx, a->cy);
    double center_angle = atan2(a->cy, a->cx);
    if (a->radius > center_r)
        // Positive real correction: no atan2 branch discontinuity.
        return phi + atan2(a->cy*cos(phi)-a->cx*sin(phi),
                           a->radius+a->cx*cos(phi)+a->cy*sin(phi));
    if (a->radius < center_r)
        return center_angle + atan2(a->radius*sin(phi-center_angle),
                                    center_r+a->radius*cos(phi-center_angle));
    // Tangent circles are accepted only for sweeps avoiding the origin.
    return .5*(phi+center_angle)
        + (cos(.5*(a->angle-center_angle)) < 0. ? M_PI : 0.);
}

static double
arc_angle(struct stepper_kinematics *sk, struct move *m, double time)
{
    struct arc_stepper *a = (void *)sk;
    double phi = a->angle + plugin_move_get_coord(m, time).x*a->inverse_radius;
    // commanded_pos is only updated at the end of an itersolve phase.
    // Full-circle cruises therefore need analytic winding, not nearest atan2.
    return arc_unwrapped_angle(a, phi) + a->angular_bias;
}

struct stepper_kinematics *
polar_arc_stepper_alloc(char type, double cx, double cy, double radius,
                       double angle, double inverse_radius, double radial_bias,
                       double angular_bias)
{
    struct arc_stepper *a = calloc(1, sizeof(*a));
    if (!a)
        return NULL;
    a->cx=cx; a->cy=cy; a->radius=radius; a->angle=angle;
    a->inverse_radius=inverse_radius; a->radial_bias=radial_bias;
    a->angular_bias = angular_bias
        + atan2(cy+radius*sin(angle), cx+radius*cos(angle))
        - arc_unwrapped_angle(a, angle);
    a->sk.calc_position_cb = type == 'r' ? arc_radius : arc_angle;
    a->sk.active_flags = AF_X;
    return &a->sk;
}
