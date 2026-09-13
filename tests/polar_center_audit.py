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
        gcode.register_command('POLAR_TEST_FEATURES', self.features)
        gcode.register_command('POLAR_TEST_ARC_REJECT', self.arc_reject)
        gcode.register_command('POLAR_TEST_MESH', self.mesh)
        gcode.register_command('POLAR_TEST_CONTINUOUS', self.continuous)
        self.checks = 0
        self.native_history = []

    def ready(self):
        self.th = self.printer.lookup_object('toolhead')
        self.kin = self.th.get_kinematics()
        original_append = self.th.trapq_append
        self.trajectory_checks = 0
        self.junction_checks = 0
        self.moving_junctions = 0
        self.previous_trapezoid = None
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
            support = getattr(self.kin, 'arc_support', None)
            if support and support.emitting and hasattr(support.emitting[0], 'geometry'):
                return original_append(*args)
            _, start_time, at, ct, dt, x, y, z, ux, uy, uz, sv, cv, accel = args
            old = self.previous_trapezoid
            if old is not None and abs(start_time-old[0]) < 1.e-6:
                _, old_pos, old_u, old_v = old
                if math.dist(old_pos, (x, y, z)) < 1.e-6:
                    radius = math.hypot(x, y)
                    dvx, dvy = ux*sv-old_u[0]*old_v, uy*sv-old_u[1]*old_v
                    if radius <= 1.e-7:
                        if abs(sv) > 1.e-6 or abs(old_v) > 1.e-6:
                            raise self.printer.command_error('Nonzero center junction speed')
                    else:
                        dr = abs((x*dvx+y*dvy)/radius)
                        da = abs((x*dvy-y*dvx)/radius**2)
                        if dr > self.kin.radial_velocity_change+1.e-6:
                            raise self.printer.command_error('Radial corner jump exceeds limit')
                        if da > self.kin.angular_velocity_change+1.e-6:
                            raise self.printer.command_error('Angular corner jump exceeds limit')
                    self.junction_checks += 1
                    self.moving_junctions += sv > 1.e-6
            distance = .5*(sv+cv)*at + cv*ct + cv*dt - .5*accel*dt*dt
            self.previous_trapezoid = (start_time+at+ct+dt,
                (x+ux*distance, y+uy*distance, z+uz*distance), (ux,uy,uz), cv-accel*dt)
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
        turn = self.kin._center_turn
        def checked_turn(target, retract):
            before = self.th.get_position()
            turn(target, retract)
            if before != self.th.get_position():
                raise self.printer.command_error('Center retract changed logical E')
            if retract:
                logging.info('POLAR_AUDIT retraction PASS')
        self.kin._center_turn = checked_turn
        if self.kin.native_arcs:
            self.install_arc_audit()

    def install_arc_audit(self):
        support = self.kin.arc_support
        original = support.record_curve
        self.native_junctions = 0
        def record(when, move):
            cx,cy,radius,angle,sweep,_,_ = move.geometry
            q = radius*abs(sweep)/move.move_d
            direction = math.copysign(q,sweep)
            s, velocity, samples = 0.,move.start_v,0
            for duration,a in [(move.accel_t,move.accel), (move.cruise_t,0.), (move.decel_t,-move.accel)]:
                if not duration: continue
                for i in range(201):
                    t = duration*i/200.
                    phi = angle+direction*(s+velocity*t+.5*a*t*t)/radius
                    x,y = cx+radius*math.cos(phi),cy+radius*math.sin(phi)
                    dx,dy = -direction*math.sin(phi),direction*math.cos(phi)
                    ddx,ddy = -q*q*math.cos(phi)/radius,-q*q*math.sin(phi)/radius
                    r = math.hypot(x,y)
                    if r < 1.e-5: continue
                    dot,cross = x*dx+y*dy,x*dy-y*dx
                    rp,ap = dot/r,cross/r**2
                    rpp = (dx*dx+dy*dy+x*ddx+y*ddy)/r-dot**2/r**3
                    app = (x*ddy-y*ddx)/r**2-2.*cross*dot/r**4
                    v = velocity+a*t
                    for name,actual,limit in [
                        ('radial velocity',abs(rp*v),self.kin.radial_velocity),
                        ('angular velocity',abs(ap*v),self.kin.v_rad_max),
                        ('radial acceleration',abs(rp*a+rpp*v*v),self.kin.radial_accel),
                        ('angular acceleration',abs(ap*a+app*v*v),self.kin.angular_accel),
                        ('Cartesian acceleration',math.hypot(a,q*q*v*v/radius),self.th.max_accel),
                        ('Z velocity',abs(move.start_tangent[2]*v),self.kin.max_z_velocity),
                        ('Z acceleration',abs(move.start_tangent[2]*a),self.kin.max_z_accel)]:
                        if actual > limit*(1+1.e-4)+1.e-5:
                            raise self.printer.command_error('Native arc '+name+' exceeds bound: %g > %g' % (actual,limit))
                    samples += 1
                s += velocity*duration+.5*a*duration**2
                velocity += a*duration
            old = self.previous_trapezoid
            if old is not None and abs(when-old[0]) < 1.e-6 and math.dist(old[1],move.start_pos[:3]) < 1.e-6:
                x,y = move.start_pos[:2]
                r = math.hypot(x,y)
                vx = move.start_tangent[0]*move.start_v-old[2][0]*old[3]
                vy = move.start_tangent[1]*move.start_v-old[2][1]*old[3]
                if r < 1.e-6:
                    if move.start_v > 1.e-6 or old[3] > 1.e-6:
                        raise self.printer.command_error('Nonzero native center junction')
                elif (abs((x*vx+y*vy)/r) > self.kin.radial_velocity_change+1.e-6
                      or abs((x*vy-y*vx)/r**2) > self.kin.angular_velocity_change+1.e-6):
                    raise self.printer.command_error('Native junction motor jump exceeded')
                if move.start_v > 1.e-6: self.native_junctions += 1
            endtime = when+move.accel_t+move.cruise_t+move.decel_t
            self.previous_trapezoid = (endtime,move.end_pos[:3],move.end_tangent,move.end_v)
            original(when,move)
            self.native_history.append((when,move))
            logging.info('POLAR_AUDIT native arc PASS samples=%d moving_junctions=%d',samples,self.native_junctions)
        support.record_curve = record

    def anchor(self, gcmd):
        self.th.flush_step_generation()
        self.anchor_steps = self.kin.bed.get_mcu_position()
        self.anchor_angle = math.atan2(*self.th.get_position()[1::-1])
        extruder = self.th.get_extruder().extruder_stepper.stepper
        self.anchor_e_steps = extruder.get_mcu_position()
        self.anchor_e = self.th.get_position()[3]
        self.anchor_z_steps = self.kin.rails[1].get_steppers()[0].get_mcu_position()
        self.anchor_z = self.th.get_position()[2]
        self.anchor_r_steps = self.kin.rails[0].get_steppers()[0].get_mcu_position()
        self.anchor_r = math.hypot(*self.th.get_position()[:2])
        self.native_history = []

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
        extruder = self.th.get_extruder().extruder_stepper.stepper
        actual_e = self.anchor_e + (extruder.get_mcu_position()-self.anchor_e_steps)*extruder.get_step_dist()
        if abs(actual_e-p[3]) > 1.1*extruder.get_step_dist():
            raise gcmd.error('Physical extrusion mismatch: %g vs %g' % (actual_e,p[3]))
        zstep = self.kin.rails[1].get_steppers()[0]
        actual_z = self.anchor_z+(zstep.get_mcu_position()-self.anchor_z_steps)*zstep.get_step_dist()
        if abs(actual_z-p[2]) > 1.1*zstep.get_step_dist():
            raise gcmd.error('Physical Z endpoint mismatch')
        self.check_physical_curves(gcmd)
        self.checks += 1
        logging.info('POLAR_AUDIT endpoint PASS checks=%d', self.checks)
        logging.info('POLAR_AUDIT trajectory samples=%d', self.trajectory_checks)
        logging.info('POLAR_AUDIT junction checks=%d moving=%d',
                     self.junction_checks, self.moving_junctions)

    def check_physical_curves(self, gcmd):
        checked = 0
        now = self.th.print_time
        arm = self.kin.rails[0].get_steppers()[0]
        zstep = self.kin.rails[1].get_steppers()[0]
        for when,move in self.native_history:
            duration = move.accel_t+move.cruise_t+move.decel_t
            if when < now-20.: continue
            for i in range(1,20):
                t = duration*i/20.
                at,ct = move.accel_t,move.cruise_t
                if t <= at: s = move.start_v*t+.5*move.accel*t*t
                elif t <= at+ct: s = .5*(move.start_v+move.cruise_v)*at+move.cruise_v*(t-at)
                else:
                    dt=t-at-ct
                    s = .5*(move.start_v+move.cruise_v)*at+move.cruise_v*ct+move.cruise_v*dt-.5*move.accel*dt*dt
                cx,cy,r,a,sweep,_,_ = move.geometry
                f = s/move.move_d
                x,y = cx+r*math.cos(a+sweep*f),cy+r*math.sin(a+sweep*f)
                expected_r,expected_a = math.hypot(x,y),math.atan2(y,x)
                actual_r = self.anchor_r+(arm.get_past_mcu_position(when+t)-self.anchor_r_steps)*arm.get_step_dist()
                actual_a = self.anchor_angle+(self.kin.bed.get_past_mcu_position(when+t)-self.anchor_steps)*self.kin.bed.get_step_dist()
                expected_z = move.start_pos[2]+f*(move.end_pos[2]-move.start_pos[2])
                actual_z = self.anchor_z+(zstep.get_past_mcu_position(when+t)-self.anchor_z_steps)*zstep.get_step_dist()
                if abs(actual_r-expected_r) > 1.2*arm.get_step_dist():
                    raise gcmd.error('Physical curved radial trajectory mismatch')
                if expected_r > 1.e-6 and abs(math.remainder(actual_a-expected_a,2*math.pi)) > 1.2*self.kin.bed.get_step_dist():
                    raise gcmd.error('Physical curved angular trajectory mismatch')
                if abs(actual_z-expected_z) > 1.2*zstep.get_step_dist():
                    raise gcmd.error('Physical curved Z trajectory mismatch')
                checked += 1
        self.native_history = []
        if checked: logging.info('POLAR_AUDIT physical curves PASS samples=%d',checked)

    def mesh(self, gcmd):
        self.th.flush_step_generation()
        from .bed_mesh import ZMesh
        params = dict(min_x=-80.,max_x=80.,min_y=-80.,max_y=80.,
            x_count=3,y_count=3,mesh_x_pps=0,mesh_y_pps=0,algo='direct',tension=.2)
        zmesh = ZMesh(params,'test')
        zmesh.build_mesh([[-.3,-.1,.1],[-.2,0.,.2],[-.1,.1,.3]])
        self.printer.lookup_object('bed_mesh').set_mesh(zmesh)

    def continuous(self, gcmd):
        self.th.flush_step_generation()
        support = self.kin.arc_support
        if self.native_junctions < 6 or support.fitted < 75 or support.mesh_spans < 2:
            raise gcmd.error('Continuous feature check failed: moving=%d fitted=%d mesh=%d' % (self.native_junctions,support.fitted,support.mesh_spans))
        logging.info('POLAR_AUDIT continuous features PASS moving=%d fitted=%d mesh=%d', self.native_junctions,support.fitted,support.mesh_spans)

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

    def features(self, gcmd):
        kin = self.kin
        if kin.center_retractions < 120 or kin.arc_support.count != 44 or kin.arc_support.fallbacks != 0:
            raise gcmd.error('Unexpected native feature counters: %s' % kin.get_status(0))
        logging.info('POLAR_AUDIT feature state PASS')

    def arc_reject(self, gcmd):
        self.th.flush_step_generation()
        before = self.th.get_position()
        steppers = self.kin.get_steppers()+[self.th.get_extruder().extruder_stepper.stepper]
        counts = [s.get_mcu_position() for s in steppers]
        # Valid endpoint, but full circle exceeds radial bounds; then unsafe E.
        for params in [{'I': '-80', 'J': '0', 'F': '600'},
                       {'I': '20', 'J': '0', 'E': '100000', 'F': '600'}]:
            try:
                command = self.printer.lookup_object('gcode').create_gcode_command('G3', 'G3', params)
                self.kin.arc_support.arcs.cmd_G3(command)
            except self.printer.command_error:
                pass
            else:
                raise gcmd.error('Invalid native arc accepted')
            self.th.flush_step_generation()
            if before != self.th.get_position() or counts != [s.get_mcu_position() for s in steppers]:
                raise gcmd.error('Invalid native arc queued partial motion')
        logging.info('POLAR_AUDIT native arc rejection atomicity PASS')


def load_config(config):
    return Audit(config)
