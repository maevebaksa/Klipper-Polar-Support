// Exercise the actual C callbacks, including endpoint and idle moves.
#include "../csrc/polar_center.c"

static double upstream_angle(struct stepper_kinematics *sk, struct move *m,
                             double move_time)
{
    struct coord c = plugin_move_get_coord(m, move_time);
    double angle = atan2(c.y, c.x);
    if (angle - sk->commanded_pos > M_PI)
        angle -= 2. * M_PI;
    else if (angle - sk->commanded_pos < -M_PI)
        angle += 2. * M_PI;
    return angle;
}

double sample_angle(int patched, double commanded, double sx, double sy,
                    double dx, double dy, double t)
{
    struct stepper_kinematics sk = { .commanded_pos = commanded };
    struct move m = { .start_pos = { .x = sx, .y = sy },
                      .axes_r = { .x = dx, .y = dy }, .start_v = 1. };
    if (patched)
        return polar_center_angle_calc_position(&sk, &m, t);
    return upstream_angle(&sk, &m, t);
}

double sample_arc(int angular, double commanded, double cx, double cy,
                  double radius, double angle, double inverse_radius,
                  double radial_bias, double angular_bias, double distance)
{
    struct arc_stepper as = {
        .sk = { .commanded_pos = commanded },
        .cx = cx, .cy = cy, .radius = radius, .angle = angle,
        .inverse_radius = inverse_radius,
        .radial_bias = radial_bias, .angular_bias = angular_bias
    };
    struct move m = { .axes_r = { .x = 1. }, .start_v = 1. };
    as.angular_bias += atan2(cy+radius*sin(angle), cx+radius*cos(angle))
                       - arc_unwrapped_angle(&as, angle);
    if (angular)
        return arc_angle(&as.sk, &m, distance);
    return arc_radius(&as.sk, &m, distance);
}
