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

// Persistent curve timeline shared by the radial, bed and Z solvers.
// Registration/pruning occur between Klipper's synchronous step-generation
// calls. Callbacks only read the timeline; only the bed initializes winding.
struct path_curve {
    double p[15]; // time, at, ct, dt, sv, cv, accel, cx, cy, R, phi, sweep, z0, z1, L
    double valid_until, angle_bias;
    int angle_ready;
};
struct path_context {
    struct path_curve *curves;
    int count, capacity;
};
struct path_stepper {
    struct stepper_kinematics sk;
    struct path_context *context;
    char type;
};
void *polar_path_alloc(void) { return calloc(1, sizeof(struct path_context)); }
void polar_path_clear(struct path_context *ctx) { ctx->count = 0; }
void polar_path_free(struct path_context *ctx) {
    if (ctx) { free(ctx->curves); free(ctx); }
}
void polar_path_close(struct path_context *ctx, double time) {
    if (ctx->count && ctx->curves[ctx->count-1].valid_until > time)
        ctx->curves[ctx->count-1].valid_until = time;
}
int polar_path_append(struct path_context *ctx, double *params) {
    if (ctx->count == ctx->capacity) {
        int size = ctx->capacity ? 2*ctx->capacity : 64;
        void *buffer = realloc(ctx->curves, size*sizeof(struct path_curve));
        if (!buffer) return -1;
        ctx->curves = buffer; ctx->capacity = size;
    }
    polar_path_close(ctx, params[0]);
    struct path_curve *c = &ctx->curves[ctx->count++];
    memset(c, 0, sizeof(*c));
    memcpy(c->p, params, sizeof(c->p));
    c->valid_until = INFINITY;
    return 0;
}
void polar_path_prune(struct path_context *ctx, double before) {
    int n=0;
    while (n<ctx->count && ctx->curves[n].valid_until < before) n++;
    if (n) {
        memmove(ctx->curves, ctx->curves+n, (ctx->count-n)*sizeof(struct path_curve));
        ctx->count -= n;
    }
}
static struct path_curve *path_find(struct path_context *ctx, double time) {
    int lo=0, hi=ctx->count;
    while (lo<hi) { int mid=(lo+hi)/2;
        if (ctx->curves[mid].p[0] <= time) lo=mid+1; else hi=mid; }
    if (!lo || time >= ctx->curves[lo-1].valid_until) return NULL;
    return &ctx->curves[lo-1];
}
static double path_distance(struct path_curve *c, double time, double *velocity) {
    double *p=c->p, t=fmax(0.,time-p[0]), s=0.;
    if (t<p[1]) { *velocity=p[4]+p[6]*t; return p[4]*t+.5*p[6]*t*t; }
    s=.5*(p[4]+p[5])*p[1]; t-=p[1];
    if (t<p[2]) { *velocity=p[5]; return s+p[5]*t; }
    s+=p[5]*p[2]; t-=p[2];
    if (t<p[3]) { *velocity=p[5]-p[6]*t; return s+p[5]*t-.5*p[6]*t*t; }
    *velocity=0.; return p[14];
}
int polar_path_query(struct path_context *ctx, double time, double *out) {
    struct path_curve *c=path_find(ctx,time);
    if (!c) return 0;
    double velocity, *p=c->p, f=path_distance(c,time,&velocity)/p[14];
    double phi=p[10]+p[11]*f;
    out[0]=p[7]+p[9]*cos(phi); out[1]=p[8]+p[9]*sin(phi);
    out[2]=p[12]+(p[13]-p[12])*f; out[3]=velocity;
    return 1;
}
static double path_calc(struct stepper_kinematics *sk, struct move *m, double t) {
    struct path_stepper *ps=(void *)sk;
    double time=m->print_time+t;
    struct path_curve *c=path_find(ps->context,time);
    if (c) {
        double v, *p=c->p, f=path_distance(c,time,&v)/p[14];
        double phi=p[10]+p[11]*f;
        if (ps->type=='z') return p[12]+(p[13]-p[12])*f;
        if (ps->type=='r') return hypot(p[7]+p[9]*cos(phi),p[8]+p[9]*sin(phi));
        struct arc_stepper a={.cx=p[7],.cy=p[8],.radius=p[9],
                              .angle=p[10]+p[11]*.5};
        // Tangent-circle pieces never cross the zero of their half-angle cosine.
        if (fabs(a.radius-hypot(a.cx,a.cy)) < 1.e-10)
            a.radius=hypot(a.cx,a.cy);
        if (!c->angle_ready) {
            double initial=arc_unwrapped_angle(&a,p[10]);
            c->angle_bias=nearbyint((sk->commanded_pos-initial)/(2.*M_PI))*2.*M_PI;
            c->angle_ready=1;
        }
        return arc_unwrapped_angle(&a,phi)+c->angle_bias;
    }
    if (ps->type=='a') return polar_center_angle_calc_position(sk,m,t);
    struct coord p=plugin_move_get_coord(m,t);
    return ps->type=='z' ? p.z : hypot(p.x,p.y);
}
struct stepper_kinematics *polar_path_stepper_alloc(struct path_context *ctx, char type) {
    struct path_stepper *ps=calloc(1,sizeof(*ps));
    if (!ps) return NULL;
    ps->context=ctx; ps->type=type;
    ps->sk.calc_position_cb=path_calc;
    ps->sk.active_flags=AF_X|AF_Y|AF_Z;
    return &ps->sk;
}
