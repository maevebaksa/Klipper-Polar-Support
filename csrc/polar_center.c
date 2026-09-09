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
