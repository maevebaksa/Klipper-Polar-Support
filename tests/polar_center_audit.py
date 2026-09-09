# Test-only Klipper extra. Never installed by the user-facing installer.
import logging
import math


class Audit:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler('klippy:ready', self.ready)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('POLAR_TEST_ANCHOR', self.anchor)
        gcode.register_command('POLAR_TEST_POSITION', self.position)
        gcode.register_command('POLAR_TEST_REJECT', self.reject)
        self.checks = 0

    def ready(self):
        self.th = self.printer.lookup_object('toolhead')
        self.kin = self.th.get_kinematics()
        original_append = self.th.trapq_append
        self.trajectory_checks = 0
        def append(tq, start_time, accel_t, cruise_t, decel_t,
                   x, y, z, ux, uy, uz, start_v, cruise_v, accel):
            # Sample the actual trapezoids submitted to the C queue, rather
            # than only the proposed limits in Python check_move().
            phases = [(accel_t, accel), (cruise_t, 0.), (decel_t, -accel)]
            velocity = start_v
            for duration, a in phases:
                if not duration:
                    continue
                for i in range(101):
                    t = duration*i/100
                    dist = velocity*t + .5*a*t*t
                    px, py = x+ux*dist, y+uy*dist
                    r = math.hypot(px, py)
                    if r <= 1.e-7:
                        continue
                    v = velocity+a*t
                    cross, dot = px*uy-py*ux, px*ux+py*uy
                    omega = abs(cross*v/r**2)
                    alpha = abs(cross*a/r**2 - 2*cross*dot*v*v/r**4)
                    rv = abs(dot*v/r)
                    ra = abs(dot*a/r + cross**2*v*v/r**3)
                    for name, actual, limit in [
                        ('angular velocity', omega, self.kin.v_rad_max),
                        ('angular acceleration', alpha, self.kin.angular_accel),
                        ('radial velocity', rv, self.kin.radial_velocity),
                        ('radial acceleration', ra, self.kin.radial_accel)]:
                        if actual > limit*(1+1.e-5)+1.e-6:
                            raise self.printer.command_error(
                                '%s exceeds bound: %g > %g' % (name, actual, limit))
                    self.trajectory_checks += 1
                distance = velocity*duration + .5*a*duration*duration
                x += ux*distance
                y += uy*distance
                z += uz*distance
                velocity += a*duration
            # Preserve original arguments after inspecting local positions.
            return original_append(tq, start_time, accel_t, cruise_t, decel_t,
                                   *append.original_position,
                                   ux, uy, uz, start_v, cruise_v, accel)
        def checked_append(*args):
            append.original_position = args[5:8]
            return append(*args)
        self.th.trapq_append = checked_append
        original = self.kin._rotate_at_center
        def rotate(target):
            self.th.flush_step_generation()
            steppers = self.kin.get_steppers()[1:]
            steppers += [self.th.get_extruder().extruder_stepper.stepper]
            before = [s.get_mcu_position() for s in steppers]
            original(target)
            after = [s.get_mcu_position() for s in steppers]
            if before != after:
                raise self.printer.command_error('XYZ/E moved during center rotation')
            self.checks += 1
            logging.info('POLAR_AUDIT rotation stationary XYZ/E PASS')
        self.kin._rotate_at_center = rotate

    def anchor(self, gcmd):
        self.th.flush_step_generation()
        self.anchor_steps = self.kin.bed.get_mcu_position()
        self.anchor_angle = math.atan2(*self.th.get_position()[1::-1])

    def position(self, gcmd):
        self.th.flush_step_generation()
        p = self.th.get_position()
        for i, name in enumerate('XYZE'):
            target = gcmd.get_float(name, p[i])
            if abs(p[i]-target) > 1.e-6:
                raise gcmd.error('Logical '+name+' endpoint mismatch')
        if p[0] or p[1]:
            expected = math.atan2(p[1], p[0])
            actual = ((self.kin.bed.get_mcu_position()-self.anchor_steps)
                      * self.kin.bed.get_step_dist() + self.anchor_angle)
            error = (actual-expected+math.pi)%(2*math.pi)-math.pi
            if abs(error) > 1.1*self.kin.bed.get_step_dist():
                raise gcmd.error('Physical bed angle mismatch %.9f rad' % error)
        arm = self.kin.rails[0].get_steppers()[0]
        if abs(arm.get_commanded_position()-math.hypot(*p[:2])) > arm.get_step_dist():
            raise gcmd.error('Physical radial position mismatch')
        self.checks += 1
        logging.info('POLAR_AUDIT endpoint PASS checks=%d', self.checks)
        logging.info('POLAR_AUDIT trajectory samples=%d', self.trajectory_checks)

    def reject(self, gcmd):
        self.th.flush_step_generation()
        before = self.th.get_position()
        counts = [s.get_mcu_position() for s in self.kin.get_steppers()]
        end = list(before)
        end[0] = gcmd.get_float('X', end[0])
        end[2] = gcmd.get_float('Z', end[2])
        end[3] = gcmd.get_float('E', end[3])
        try:
            self.th.move(end, 10.)
        except self.printer.command_error:
            pass
        else:
            raise gcmd.error('Invalid request was accepted')
        self.th.flush_step_generation()
        if before != self.th.get_position():
            raise gcmd.error('Invalid request changed commanded position')
        if counts != [s.get_mcu_position() for s in self.kin.get_steppers()]:
            raise gcmd.error('Invalid request generated steps')
        self.checks += 1
        logging.info('POLAR_AUDIT rejected request atomicity PASS')


def load_config(config):
    return Audit(config)
