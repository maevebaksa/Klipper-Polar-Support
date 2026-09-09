# Opt-in polar kinematics with stopped center reorientation.
# Copyright (C) 2026
# Distributed under the GNU GPLv3 license.
import math
from pathlib import Path
import cffi
import chelper
import runpy
from . import polar
from extras.force_move import calc_move_time

# Numerical snapping only: 0.0000001 mm, not a printable center exclusion.
CENTER_EPS = 1.e-7


def wrap_angle(angle):
    return (angle + math.pi) % (2. * math.pi) - math.pi


def split_parameters(start, end, slow_radius):
    """Straight-line fractions: exact center, or geometric near-center bands."""
    if math.hypot(*start[:2]) <= CENTER_EPS or math.hypot(*end[:2]) <= CENTER_EPS:
        return [], False
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if length <= CENTER_EPS:
        return [], False
    ux, uy = dx / length, dy / length
    along = start[0] * ux + start[1] * uy
    height = abs(start[0] * uy - start[1] * ux)
    closest_t = -along / length
    if height <= CENTER_EPS:
        if 0. < closest_t < 1.:
            return [closest_t], True
        return [], False
    if height >= slow_radius:
        return [], False
    # Bands double in width away from the singularity. This preserves the
    # line while avoiding a whole long move at the speed of its closest point.
    points = [closest_t]
    band = height
    extent = max(abs(along), abs(along + length))
    while band < extent:
        points.extend(((-band - along) / length, (band - along) / length))
        band *= 2.
    return sorted(set(t for t in points if 0. < t < 1.)), False


def derivative_bounds(start, end, move_length):
    """Bounds per mm of XYZ path: |theta'|, |theta''|, |r''|, |r'|."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    xy_length = math.hypot(dx, dy)
    if not xy_length or not move_length:
        return 0., 0., 0., 0.
    q = xy_length / move_length
    h = abs(start[0] * dy - start[1] * dx) / move_length
    rmin = polar.distance_to_center(start, end)
    # Exactly radial center legs have constant angle and linear radius.
    if (math.hypot(*start[:2]) == 0.
        or math.hypot(*end[:2]) == 0. or h == 0.):
        return 0., 0., 0., q
    if rmin <= 0.:
        raise ValueError("Unsplit polar center crossing")
    return h / rmin**2, 2. * h * q / rmin**3, h*h / rmin**3, q


class PolarCenterKinematics(polar.PolarKinematics):
    def __init__(self, toolhead, config):
        plugin_root = Path(__file__).resolve().parents[2]
        try:
            runpy.run_path(str(plugin_root / 'compatibility.py'))['check_installed'](
                Path(chelper.__file__).resolve().parents[2])
        except (OSError, ValueError, RuntimeError) as exc:
            raise config.error(str(exc))
        super().__init__(toolhead, config)
        self.toolhead = toolhead
        self.printer = config.get_printer()
        self._homing = False
        self.angle = 0.
        self.center_rotations = 0
        if config.has_section('input_shaper'):
            raise config.error("polar_center does not support [input_shaper]; "
                               "remove that section for this experimental profile")
        if self.v_rad_max <= 0.:
            raise config.error("polar_center requires max_angular_velocity > 0")
        self.angular_accel = config.getfloat('max_angular_accel', above=0.)
        self.radial_velocity = config.getfloat(
            'max_radial_velocity', self.max_velocity, above=0.)
        self.radial_accel = config.getfloat(
            'max_radial_accel', self.max_accel, above=0.)
        self.slow_radius = config.getfloat('polar_slow_radius', 5., above=0.)
        if self.rails[0].get_range()[0] != 0.:
            raise config.error("polar_center requires arm position_min: 0")
        self.bed = self.steppers[0]
        ffi, lib = chelper.get_ffi()
        library = Path(__file__).resolve().parents[2] / 'build/polar_center.so'
        if not library.is_file():
            raise config.error("Klipper-Polar-Support is not built; rerun install.sh")
        # Keep the independent FFI/library alive for the lifetime of this
        # kinematics object. Cast the allocation into Klipper's main FFI so
        # no tracked Klipper C or Python source needs modification.
        self.plugin_ffi = cffi.FFI()
        self.plugin_ffi.cdef(
            'struct stepper_kinematics; '
            'struct stepper_kinematics *polar_center_stepper_alloc(void);')
        self.plugin_lib = self.plugin_ffi.dlopen(str(library))
        raw_sk = self.plugin_lib.polar_center_stepper_alloc()
        if raw_sk == self.plugin_ffi.NULL:
            raise config.error("Unable to allocate polar_center solver")
        address = int(self.plugin_ffi.cast('uintptr_t', raw_sk))
        center_sk = ffi.gc(ffi.cast('struct stepper_kinematics *', address),
                           lib.free)
        self.bed.set_stepper_kinematics(center_sk)
        self.rotation_sk = ffi.gc(lib.cartesian_stepper_alloc(b'x'), lib.free)
        self.motion_queuing = toolhead.motion_queuing
        self.rotation_trapq = self.motion_queuing.allocate_trapq()
        self.trapq_append = self.motion_queuing.lookup_trapq_append()
        # Toolhead entry point, below G-code/bed-mesh transforms. Normal
        # manual moves and segmented G2/G3 moves use the same path. Homing's
        # drip moves retain their native endstop synchronization.
        self._native_move = toolhead.move
        toolhead.move = self.move

    def set_position(self, newpos, homing_axes):
        super().set_position(newpos, homing_axes)
        if math.hypot(*newpos[:2]) > CENTER_EPS:
            self.angle = math.atan2(newpos[1], newpos[0])

    def home(self, homing_state):
        self._homing = True
        try:
            super().home(homing_state)
        finally:
            self._homing = False
            self.angle = self.bed.get_commanded_position()

    def _check_bounds(self, move):
        end = move.end_pos
        if end[0]**2 + end[1]**2 > self.limit_xy2:
            if self.limit_xy2 < 0.:
                raise move.move_error("Must home axis first")
            raise move.move_error()
        if move.axes_d[2]:
            if self.limit_z[0] > self.limit_z[1]:
                raise move.move_error("Must home axis first")
            if not self.limit_z[0] <= end[2] <= self.limit_z[1]:
                raise move.move_error()
            ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * ratio,
                             self.max_z_accel * ratio)

    def check_move(self, move):
        self._check_bounds(move)
        # Exact stops at non-collinear junctions: Cartesian corner velocity
        # alone cannot bound the polar motors' instantaneous velocity jump.
        move.junction_deviation = 0.
        if not (move.axes_d[0] or move.axes_d[1]):
            return
        fractions, crossing = split_parameters(
            move.start_pos, move.end_pos, self.slow_radius)
        if crossing and not self._homing:
            raise move.move_error("Center crossing bypassed polar_center planner")
        f, g, curvature, q = derivative_bounds(
            move.start_pos, move.end_pos, move.move_d)
        vmax = self.radial_velocity / q
        amax = self.radial_accel / q
        if f:
            vmax = min(vmax, self.v_rad_max / f)
            # |theta_ddot| <= f*a + g*v^2, splitting its budget equally.
            amax = min(amax, self.angular_accel / (2. * f))
        if g:
            vmax = min(vmax, math.sqrt(self.angular_accel / (2. * g)))
        if curvature:
            # |r_ddot| <= q*a + curvature*v^2.
            amax = min(amax, self.radial_accel / (2. * q))
            vmax = min(vmax, math.sqrt(self.radial_accel / (2. * curvature)))
        move.limit_speed(vmax, amax)

    def _preflight(self, start, end, speed):
        # Run the actual toolhead Move and extra-axis (including extruder)
        # validators on every leg before queueing any part of the request.
        from toolhead import Move
        move = Move(self.toolhead, start, end, speed)
        if not move.move_d:
            return
        if move.is_kinematic_move:
            self.check_move(move)
        for index, axis in enumerate(self.toolhead.extra_axes):
            if move.axes_d[index + 3]:
                axis.check_move(move, index + 3)

    def _rotate_at_center(self, target):
        toolhead = self.toolhead
        pos = toolhead.get_position()
        if pos[0] != 0. or pos[1] != 0. or self.limit_xy2 < 0.:
            raise self.printer.command_error("Bed reorientation requires homed center")
        # End all preceding XYZ/E motion and pressure-advance step generation
        # before borrowing the bed stepper for a rotary trapezoid.
        toolhead.flush_step_generation()
        current = self.bed.get_commanded_position()
        delta = wrap_angle(target - current)
        if abs(delta) < 1.e-12:
            self.angle = target
            return
        previous_sk = self.bed.set_stepper_kinematics(self.rotation_sk)
        previous_trapq = self.bed.set_trapq(self.rotation_trapq)
        try:
            self.bed.set_position((current, 0., 0.))
            direction, accel_t, cruise_t, velocity = calc_move_time(
                delta, self.v_rad_max, self.angular_accel)
            start_time = toolhead.get_last_move_time()
            duration = 2. * accel_t + cruise_t
            self.trapq_append(self.rotation_trapq, start_time,
                              accel_t, cruise_t, accel_t,
                              current, 0., 0., direction, 0., 0.,
                              0., velocity, self.angular_accel)
            self.motion_queuing.note_mcu_movequeue_activity(start_time + duration)
            toolhead.dwell(duration)
            toolhead.flush_step_generation()
            actual_angle = self.bed.get_commanded_position()
        except Exception:
            # A failed queue transition makes position uncertain. Do not
            # resume accepting moves after catching a G-code error.
            self.printer.invoke_shutdown("polar_center rotary transition failed")
            raise
        finally:
            self.bed.set_trapq(previous_trapq)
            self.bed.set_stepper_kinematics(previous_sk)
            self.motion_queuing.wipe_trapq(self.rotation_trapq)
        # Preserve quantized physical position through the solver switch.
        # Re-labelling the bed to its ideal target would accumulate rounding
        # error over repeated rotations with a nonintegral steps/revolution.
        self.bed.set_position((math.cos(actual_angle), math.sin(actual_angle), 0.))
        self.angle = target
        self.center_rotations += 1

    def move(self, newpos, speed):
        if self._homing:
            return self._native_move(newpos, speed)
        if not math.isfinite(speed) or speed <= 0.:
            raise self.printer.command_error("Invalid polar move speed")
        end = list(newpos)
        if not all(math.isfinite(v) for v in end):
            raise self.printer.command_error("Nonfinite polar move coordinate")
        start = self.toolhead.get_position()
        if math.hypot(*end[:2]) <= CENTER_EPS:
            end[0] = end[1] = 0.
        # Validate the original unsplit move first for bounds and E safety.
        from toolhead import Move
        whole = Move(self.toolhead, start, end, speed)
        if whole.is_kinematic_move:
            self._check_bounds(whole)
        for index, axis in enumerate(self.toolhead.extra_axes):
            if whole.axes_d[index + 3]:
                axis.check_move(whole, index + 3)
        fractions, crossing = split_parameters(start, end, self.slow_radius)
        legs = []
        for t in fractions:
            middle = [a + t * (b - a) for a, b in zip(start, end)]
            if crossing:
                middle[0] = middle[1] = 0.
            legs.append(middle)
        legs.append(end)
        prev = start
        for leg in legs:
            self._preflight(prev, leg, speed)
            prev = leg
        for leg in legs:
            current = self.toolhead.get_position()
            at_center = current[0] == 0. and current[1] == 0.
            leaving = leg[0] != 0. or leg[1] != 0.
            if at_center and leaving:
                target = math.atan2(leg[1], leg[0])
                if abs(wrap_angle(target - self.angle)) > 1.e-12:
                    self._rotate_at_center(target)
            self._native_move(leg, speed)
            if leaving:
                self.angle = math.atan2(leg[1], leg[0])
            elif current[0] != 0. or current[1] != 0.:
                # An inward leg stops before a later departure/reorientation.
                self.angle = math.atan2(current[1], current[0])
                self.toolhead.limit_next_junction_speed(0.)

    def get_status(self, eventtime):
        result = super().get_status(eventtime)
        result['polar_center_rotations'] = self.center_rotations
        return result


def load_kinematics(toolhead, config):
    return PolarCenterKinematics(toolhead, config)
