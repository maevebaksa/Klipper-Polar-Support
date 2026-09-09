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
