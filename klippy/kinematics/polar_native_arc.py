"""Opt-in native XY arcs with a stopped trapezoid per arc.

Active transforms and arcs through the polar origin retain Klipper's segmented
path. Arc boundaries stop; this is not a general curve-blending planner.
"""
import math
import chelper
from toolhead import Move


def arc_geometry(start, end, offset, clockwise):
    cx, cy = start[0]+offset[0], start[1]+offset[1]
    radius = math.hypot(*offset)
    end_radius = math.hypot(end[0]-cx, end[1]-cy)
    if not radius or abs(radius-end_radius) > 1.e-6:
        return None
    angle = math.atan2(start[1]-cy, start[0]-cx)
    end_angle = math.atan2(end[1]-cy, end[0]-cx)
    sweep = ((angle-end_angle) if clockwise else (end_angle-angle)) % (2.*math.pi)
    if start[:2] == end[:2]:
        sweep = 2.*math.pi
    if not sweep:
        return None
    signed = -sweep if clockwise else sweep
    def inside(a):
        return ((angle-a) if clockwise else (a-angle)) % (2.*math.pi) <= sweep+1.e-12
    radii = [math.hypot(*start[:2]), math.hypot(*end[:2])]
    far_angle = math.atan2(cy, cx)
    for a in (far_angle, far_angle+math.pi):
        if inside(a):
            radii.append(math.hypot(cx+radius*math.cos(a), cy+radius*math.sin(a)))
    return cx, cy, radius, angle, signed, min(radii), max(radii)


class NativeArcs:
    def __init__(self, kin, config):
        self.kin, self.printer, self.th = kin, kin.printer, kin.toolhead
        self.count = self.fallbacks = 0
        self.arcs = self.printer.load_object(config, 'gcode_arcs')
        self.original = self.arcs.planArc
        self.arcs.planArc = self.plan
        self.trapq = kin.motion_queuing.allocate_trapq()

    def transform_supported(self, gm):
        if gm.move_transform is None:
            return True
        mesh = self.printer.lookup_object('bed_mesh', None)
        return (gm.move_transform is mesh and mesh is not None
                and type(mesh).__module__ == 'extras.bed_mesh'
                and mesh.z_mesh is None and mesh.fade_target == 0.)

    def plan(self, current, target, offset, clockwise, gcmd, absolute_e, alpha, beta, helical):
        gm = self.arcs.gcode_move
        if ((alpha, beta, helical) != (0, 1, 2) or current[2] != target[2]
                or len(self.th.extra_axes) != 1 or not self.transform_supported(gm)):
            return self.fallback(current, target, offset, clockwise, gcmd, absolute_e, alpha, beta, helical)
        start = self.th.get_position()
        end = list(start)
        for i in range(3):
            end[i] = target[i] + gm.base_position[i]
        e = gcmd.get_float('E', None)
        if e is not None:
            end[3] = e*gm.extrude_factor + (gm.base_position[3] if absolute_e else start[3])
        feed = gcmd.get_float('F', None, above=0.)
        speed = gm.speed if feed is None else feed*gm.speed_factor
        if not all(math.isfinite(v) for v in start+end+list(offset)+[speed]) or speed <= 0.:
            raise gcmd.error('Invalid native arc coordinate or speed')
        geometry = arc_geometry(start, end, offset, clockwise)
        if geometry is None or geometry[5] <= 1.e-6:
            return self.fallback(current, target, offset, clockwise, gcmd, absolute_e, alpha, beta, helical)
        self.execute(start, end, speed, geometry)
        gm.last_position[:] = end
        gm.speed = speed
        self.count += 1

    def fallback(self, *args):
        self.fallbacks += 1
        return self.original(*args)

    def execute(self, start, end, speed, geometry):
        kin, th = self.kin, self.th
        cx, cy, radius, angle, sweep, rmin, rmax = geometry
        length = radius*abs(sweep)
        # Virtual X carries actual arc length. The extruder sees true E/length
        # and a genuine XY move even when full-circle endpoints coincide.
        virtual_end = list(start)
        virtual_end[0] += length
        virtual_end[3] = end[3]
        move = Move(th, start, virtual_end, speed)
        kin._check_bounds(Move(th, start, end, speed))
        if rmax*rmax > kin.limit_xy2+1.e-9:
            raise self.printer.command_error('Native arc exceeds radial travel limit')
        if len(kin.rails[0].get_steppers()) != 1:
            raise self.printer.command_error('Native arcs require a single radial stepper')
        # Conservative analytic motor derivatives for arbitrary offset circles.
        # Centered circles have constant arm position and dtheta/ds = 1/R.
        if cx == 0. and cy == 0.:
            radial_slope = radial_curvature = angular_curvature = 0.
            angular_slope = 1./radius
        else:
            radial_slope = 1.
            radial_curvature = 1./rmin + 1./radius
            angular_slope = 1./rmin
            angular_curvature = 1./(radius*rmin) + 2./rmin**2
        vmax = kin.v_rad_max/angular_slope
        amax = min(th.max_accel/2., kin.angular_accel/(2.*angular_slope))
        # Normal and tangential acceleration share the Cartesian budget.
        vmax = min(vmax, math.sqrt(th.max_accel*radius/2.))
        if radial_slope:
            vmax = min(vmax, kin.radial_velocity/radial_slope)
            amax = min(amax, kin.radial_accel/(2.*radial_slope))
        if radial_curvature:
            vmax = min(vmax, math.sqrt(kin.radial_accel/(2.*radial_curvature)))
        if angular_curvature:
            vmax = min(vmax, math.sqrt(kin.angular_accel/(2.*angular_curvature)))
        move.limit_speed(vmax, amax)
        for i, extra in enumerate(th.extra_axes):
            if move.axes_d[i+3]:
                extra.check_move(move, i+3)
        # Same single-move minimum-cruise cap used by native look-ahead.
        cruise_v2 = min(move.max_cruise_v2, move.delta_v2/2., move.mcr_delta_v2/2.)
        move.set_junction(0., cruise_v2, 0.)
        th.flush_step_generation()
        arm = kin.rails[0].get_steppers()[0]
        ffi, lib = chelper.get_ffi()
        bias_r = arm.get_commanded_position()-math.hypot(*start[:2])
        bias_a = math.remainder(kin.bed.get_commanded_position()-math.atan2(start[1],start[0]), 2.*math.pi)
        temporary = []
        for kind in (b'r', b'a'):
            raw = kin.plugin_lib.polar_arc_stepper_alloc(kind,cx,cy,radius,angle,
                math.copysign(1./radius,sweep),bias_r,bias_a)
            if raw == kin.plugin_ffi.NULL:
                raise self.printer.command_error('Unable to allocate native arc solver')
            address = int(kin.plugin_ffi.cast('uintptr_t',raw))
            temporary.append(ffi.gc(ffi.cast('struct stepper_kinematics *',address),lib.free))
        previous = []
        try:
            for stepper, solver in zip((arm,kin.bed),temporary):
                previous.append((stepper,stepper.set_stepper_kinematics(solver),stepper.set_trapq(self.trapq)))
                stepper.set_position((0.,0.,start[2]))
            when = th.get_last_move_time()
            duration = move.accel_t+move.cruise_t+move.decel_t
            kin.trapq_append(self.trapq,when,move.accel_t,move.cruise_t,move.decel_t,
                0.,0.,start[2],1.,0.,0.,move.start_v,move.cruise_v,move.accel)
            for i, extra in enumerate(th.extra_axes):
                if move.axes_d[i+3]:
                    extra.process_move(when,move,i+3)
            kin.motion_queuing.note_mcu_movequeue_activity(when+duration)
            th.dwell(duration)
            th.flush_step_generation()
            actual_r, actual_a = arm.get_commanded_position(),kin.bed.get_commanded_position()
        except Exception:
            self.printer.invoke_shutdown('polar_center native arc transition failed')
            raise
        finally:
            for stepper, solver, trapq in previous:
                stepper.set_trapq(trapq)
                stepper.set_stepper_kinematics(solver)
            kin.motion_queuing.wipe_trapq(self.trapq)
        arm.set_position((actual_r,0.,start[2]))
        kin.bed.set_position((math.cos(actual_a),math.sin(actual_a),start[2]))
        lib.trapq_set_position(th.trapq,th.print_time,*end[:3])
        th.commanded_pos[:] = end
        kin.angle = math.atan2(end[1],end[0])
        kin._center_extruding = False
