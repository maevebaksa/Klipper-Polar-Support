"""Persistent native XY curves in Klipper's normal look-ahead queue.

Supports helical Z, bounded endpoint repair, mesh height approximation and
optional bounded G1 fitting. Exact origin passages still stop and reorient.
"""
import collections
import math
import chelper
from toolhead import Move
from .polar_center import CENTER_EPS, wrap_angle


def arc_geometry(start, end, offset, clockwise, tolerance=1.e-6):
    cx, cy = start[0]+offset[0], start[1]+offset[1]
    radius = math.hypot(*offset)
    end_radius = math.hypot(end[0]-cx, end[1]-cy)
    if not radius:
        return None
    dx, dy = end[0]-start[0], end[1]-start[1]
    chord2 = dx*dx+dy*dy
    if chord2 and abs(radius-end_radius) > 1.e-12:
        # Project the nominal center onto the perpendicular chord bisector.
        # Both endpoints stay exact. Center shift + radius change bounds the
        # change of corresponding points over the entire nominal circle.
        shift = (end_radius**2-radius**2)/(2.*chord2)
        nx, ny = cx+shift*dx, cy+shift*dy
        nr = math.hypot(start[0]-nx, start[1]-ny)
        if math.hypot(nx-cx,ny-cy)+abs(nr-radius) > tolerance:
            return None
        cx, cy, radius = nx, ny, nr
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
    far_angle = math.atan2(cy,cx)
    for a in (far_angle,far_angle+math.pi):
        if inside(a):
            radii.append(math.hypot(cx+radius*math.cos(a),cy+radius*math.sin(a)))
    return cx,cy,radius,angle,signed,min(radii),max(radii)


def center_fraction(geometry):
    cx,cy,r,a,sweep,_,_ = geometry
    if abs(math.hypot(cx,cy)-r) > 1.e-10:
        return None
    origin = math.atan2(-cy,-cx)
    span = ((a-origin) if sweep < 0 else (origin-a)) % (2.*math.pi)
    f = span/abs(sweep)
    return f if 1.e-10 < f < 1.-1.e-10 else None


def mesh_gradient(mesh):
    table = mesh.mesh_matrix
    gx = max(abs(b-a)/mesh.mesh_x_dist for row in table for a,b in zip(row,row[1:]))
    gy = max(abs(b-a)/mesh.mesh_y_dist for ra,rb in zip(table,table[1:]) for a,b in zip(ra,rb))
    return math.hypot(gx,gy)


def nominal_mesh_z(mesh, position):
    if mesh.z_mesh is None:
        return position[2]-mesh.fade_target
    correction = mesh.z_mesh.calc_z(*position[:2])
    low = position[2]-correction
    if low+mesh.tool_offset <= mesh.fade_start:
        return low
    high = position[2]-mesh.fade_target
    if high+mesh.tool_offset >= mesh.fade_end:
        return high
    k = (correction-mesh.fade_target)/mesh.fade_dist
    return (position[2]-mesh.fade_target-k*(mesh.fade_end-mesh.tool_offset))/(1.-k)


class ArcMove(Move):
    def __init__(self, support, start, end, speed, geometry):
        self.geometry = geometry
        cx,cy,r,angle,sweep,rmin,rmax = geometry
        flat = r*abs(sweep)
        length = math.hypot(flat,end[2]-start[2])
        virtual_end = list(start)
        virtual_end[0] += length
        virtual_end[3:] = end[3:]
        super().__init__(support.th,start,virtual_end,speed)
        self.start_pos, self.end_pos = tuple(start), tuple(end)
        self.requested_speed = speed
        # Skeleton trapq coordinates end at the real endpoint. Full circles
        # need an active XY flag even though their endpoints coincide.
        self.axes_d[:3] = [end[i]-start[i] for i in range(3)]
        self.axes_r[:3] = [v/length for v in self.axes_d[:3]]
        if not (self.axes_d[0] or self.axes_d[1]):
            self.axes_d[0] = length
            self.axes_r[0] = 1.
        q = flat/length
        direction = math.copysign(q,sweep)
        z_slope = (end[2]-start[2])/length
        self.start_tangent = [-direction*math.sin(angle),direction*math.cos(angle),z_slope]+self.axes_r[3:]
        self.end_tangent = [-direction*math.sin(angle+sweep),direction*math.cos(angle+sweep),z_slope]+self.axes_r[3:]
        kin = support.kin
        kin._check_bounds(self)
        if rmax*rmax > kin.limit_xy2+1.e-8:
            raise self.move_error('Native arc exceeds radial travel limit')
        tangent_circle = abs(r-math.hypot(cx,cy)) <= 1.e-10
        if cx == 0. and cy == 0.:
            rp = rpp = app = 0.
            ap = q/r
        elif tangent_circle:
            rp, rpp, ap, app = q, q*q/(2.*r), q/(2.*r), 0.
        else:
            if rmin <= CENTER_EPS:
                raise self.move_error('Native arc approaches unresolved polar singularity')
            rp, rpp = q, q*q*(1./rmin+1./r)
            ap, app = q/rmin, q*q*(1./(r*rmin)+2./rmin**2)
        vmax = kin.v_rad_max/ap
        amax = min(support.th.max_accel/2.,kin.angular_accel/((2. if app else 1.)*ap))
        vmax = min(vmax,math.sqrt(support.th.max_accel*r/(2.*q*q)))
        if rp:
            vmax = min(vmax,kin.radial_velocity/rp)
            amax = min(amax,kin.radial_accel/((2. if rpp else 1.)*rp))
        if rpp:
            vmax = min(vmax,math.sqrt(kin.radial_accel/(2.*rpp)))
        if app:
            vmax = min(vmax,math.sqrt(kin.angular_accel/(2.*app)))
        self.limit_speed(vmax,amax)
        kin.decorate_junction(self)
        for i,extra in enumerate(support.th.extra_axes):
            if self.axes_d[i+3]:
                extra.check_move(self,i+3)


class NativeArcs:
    def __init__(self, kin, config):
        self.kin,self.printer,self.th = kin,kin.printer,kin.toolhead
        self.count = self.fallbacks = self.fitted = self.mesh_spans = 0
        self.tolerance = config.getfloat('polar_arc_tolerance',.02,above=0.,maxval=.1)
        self.mesh_tolerance = config.getfloat('polar_mesh_tolerance',.005,above=0.,maxval=.05)
        self.auto_fit = config.getboolean('polar_auto_arcs',False)
        self.fit_tolerance = config.getfloat('polar_fit_tolerance',.01,above=0.,maxval=.05)
        self.arcs = self.printer.load_object(config,'gcode_arcs')
        self.original = self.arcs.planArc
        self.arcs.planArc = self.plan
        ffi = self.ffi = kin.plugin_ffi
        lib = self.lib = kin.plugin_lib
        ffi.cdef('''struct path_context;
          struct path_context *polar_path_alloc(void);
          void polar_path_clear(struct path_context *);
          void polar_path_free(struct path_context *);
          void polar_path_close(struct path_context *, double);
          int polar_path_append(struct path_context *, double *);
          void polar_path_prune(struct path_context *, double);
          int polar_path_query(struct path_context *, double, double *);
          struct stepper_kinematics *polar_path_stepper_alloc(struct path_context *, char);''')
        ctx = lib.polar_path_alloc()
        if ctx == ffi.NULL:
            raise config.error('Unable to allocate polar curve timeline')
        self.context = ffi.gc(ctx,lib.polar_path_free)
        mainffi,mainlib = chelper.get_ffi()
        groups = [(kin.bed,b'a')]+[(s,b'r') for s in kin.rails[0].get_steppers()]+[(s,b'z') for s in kin.rails[1].get_steppers()]
        for stepper,kind in groups:
            raw = lib.polar_path_stepper_alloc(ctx,kind)
            if raw == ffi.NULL:
                raise config.error('Unable to allocate persistent polar solver')
            address = int(ffi.cast('uintptr_t',raw))
            solver = mainffi.gc(mainffi.cast('struct stepper_kinematics *',address),mainlib.free)
            stepper.set_stepper_kinematics(solver)
        self.emitting = collections.deque()
        self.original_flush = self.th.lookahead.flush
        self.th.lookahead.flush = self.flush_lookahead
        self.original_append = self.th.trapq_append
        self.th.trapq_append = self.append
        self.original_add = self.th.lookahead.add_move
        self.th.lookahead.add_move = self.add_move
        kin.motion_queuing.register_flush_callback(self.prune)
        self.printer.register_event_handler('toolhead:set_position',self.clear)
        self.printer.register_event_handler('klippy:ready',self.ready)

    def ready(self):
        report = self.printer.lookup_object('motion_report',None)
        if report is not None and 'toolhead' in report.dtrapqs:
            trap = report.dtrapqs['toolhead']
            original = trap.get_trapq_position
            def position(time):
                out = self.ffi.new('double[4]')
                if self.lib.polar_path_query(self.context,time,out):
                    return tuple(out[i] for i in range(3)),out[3]
                return original(time)
            trap.get_trapq_position = position

    def clear(self, *args):
        self.lib.polar_path_clear(self.context)

    def prune(self, flush_time, step_gen_time):
        # Retain motion-report history; never discard ungenerated curves.
        mq = self.kin.motion_queuing
        before = min(mq.last_flush_time,mq.last_step_gen_time)-30.
        self.lib.polar_path_prune(self.context,before)

    def flush_lookahead(self, *args, **kwargs):
        moves = self.original_flush(*args,**kwargs)
        self.emitting.extend(m for m in moves if m.is_kinematic_move)
        return moves

    def append(self, *args):
        move = self.emitting.popleft() if self.emitting else None
        when = args[1]
        self.lib.polar_path_close(self.context,when)
        if isinstance(move,ArcMove):
            self.record_curve(when,move)
        return self.original_append(*args)

    def record_curve(self, when, move):
        cx,cy,r,a,sweep,_,_ = move.geometry
        params = [when,move.accel_t,move.cruise_t,move.decel_t,
                  move.start_v,move.cruise_v,move.accel,cx,cy,r,a,sweep,
                  move.start_pos[2],move.end_pos[2],move.move_d]
        if self.lib.polar_path_append(self.context,self.ffi.new('double[]',params)):
            self.printer.invoke_shutdown('Unable to queue native curve geometry')
            raise self.printer.command_error('Native curve allocation failed')

    def add_move(self, move):
        queue = self.th.lookahead.queue
        candidate = None
        if self.auto_fit and queue and not self.kin._homing:
            candidate = self.fit_pair(queue[-1],move)
        if candidate is not None:
            queue.pop()
            self.fitted += 1
            move = candidate
        return self.original_add(move)

    def fit_pair(self, first, second):
        if isinstance(first,ArcMove) or isinstance(second,ArcMove):
            return None
        if (not first.is_kinematic_move or not second.is_kinematic_move
            or first.end_pos != second.start_pos
            or len(first.axes_d) != 4 or first.timing_callbacks or second.timing_callbacks
            or first.next_junction_v2 != 999999999.9
            or first.axes_d[3] <= 0. or second.axes_d[3] <= 0.
            or first.move_d < .01 or second.move_d < .01
            or first.requested_speed != second.requested_speed):
            return None
        transform = self.arcs.gcode_move.move_transform
        if transform is not None and not self.transform_supported(self.arcs.gcode_move):
            return None
        a,b,c = first.start_pos,first.end_pos,second.end_pos
        if transform is None:
            heights = [p[2] for p in (a,b,c)]
        else:
            heights = [nominal_mesh_z(transform,p) for p in (a,b,c)]
        if max(heights)-min(heights) > 1.e-6:
            return None
        dx,dy,ex,ey = b[0]-a[0],b[1]-a[1],c[0]-a[0],c[1]-a[1]
        det = 2.*(dx*ey-dy*ex)
        if abs(det) < 1.e-10:
            return None
        u,v = (dx*dx+dy*dy),(ex*ex+ey*ey)
        offset = ((ey*u-dy*v)/det,(dx*v-ex*u)/det)
        geometry = arc_geometry(a,c,offset,det < 0.)
        if geometry is None or abs(geometry[4]) > math.pi/2. or geometry[5] <= self.kin.slow_radius:
            return None
        if transform is not None and transform.z_mesh is not None:
            # Constant nominal Z makes fade constant. The same global height
            # Lipschitz bound used by explicit arcs certifies a single fit.
            factor = transform.get_z_factor(sum(heights)/3.)
            height_error = factor*mesh_gradient(transform.z_mesh)*geometry[2]*abs(geometry[4])/2.
            if height_error > self.mesh_tolerance:
                return None
        mid = arc_geometry(a,b,offset,det < 0.)
        fraction = abs(mid[4]/geometry[4])
        if not 0. < fraction < 1.:
            return None
        sag = geometry[2]*max(1.-math.cos(mid[4]/2.),1.-math.cos((geometry[4]-mid[4])/2.))
        if sag > self.fit_tolerance:
            return None
        if abs(a[3]+fraction*(c[3]-a[3])-b[3]) > 1.e-5:
            return None
        try:
            return ArcMove(self,a,c,first.requested_speed,geometry)
        except self.printer.command_error:
            return None

    def transform_supported(self, gm):
        mesh = self.printer.lookup_object('bed_mesh',None)
        return (gm.move_transform is None or
                (gm.move_transform is mesh and mesh is not None
                 and type(mesh).__module__ == 'extras.bed_mesh'))

    def plan(self,current,target,offset,clockwise,gcmd,absolute_e,alpha,beta,helical):
        gm = self.arcs.gcode_move
        if (alpha,beta,helical) != (0,1,2) or not self.transform_supported(gm):
            return self.fallback(current,target,offset,clockwise,gcmd,absolute_e,alpha,beta,helical)
        start = list(gm.last_position)
        end = list(start)
        for i in range(3):
            end[i] = target[i]+gm.base_position[i]
        e = gcmd.get_float('E',None)
        if e is not None:
            end[3] = e*gm.extrude_factor+(gm.base_position[3] if absolute_e else start[3])
        feed = gcmd.get_float('F',None,above=0.)
        speed = gm.speed if feed is None else feed*gm.speed_factor
        if not all(math.isfinite(v) for v in start+end+list(offset)+[speed]) or speed <= 0.:
            raise gcmd.error('Invalid native arc coordinate or speed')
        geometry = arc_geometry(start,end,offset,clockwise,self.tolerance)
        if geometry is None:
            raise gcmd.error('Arc endpoints cannot be repaired within polar_arc_tolerance')
        if geometry[2]*abs(geometry[4]) <= 1.e-6:
            return self.fallback(current,target,offset,clockwise,gcmd,absolute_e,alpha,beta,helical)
        self.execute(start,end,speed,geometry)
        gm.last_position[:] = end
        gm.speed = speed
        mesh = gm.move_transform
        if mesh is not None:
            mesh.last_position[:] = end
        self.count += 1

    def fallback(self,*args):
        self.fallbacks += 1
        return self.original(*args)

    def prepare(self,start,end,speed,geometry):
        cx,cy,r,angle,sweep,_,_ = geometry
        mesh = self.arcs.gcode_move.move_transform
        splits = [0.,1.]
        crossing = center_fraction(geometry)
        if crossing is not None:
            splits.append(crossing)
        n = 1
        if mesh is not None and mesh.z_mesh is not None:
            # Global Lipschitz bound on height correction. Linear interpolation
            # error <= correction_slope * XY_arc_length / 2, including cell and
            # fade boundaries. XY remains an exact circle in every span.
            zmesh = mesh.z_mesh
            correction_slope = mesh_gradient(zmesh)
            if mesh.fade_end != mesh.FADE_DISABLE:
                low,high = zmesh.get_z_range()
                correction_slope += max(abs(low-mesh.fade_target),abs(high-mesh.fade_target))*abs(end[2]-start[2])/(r*abs(sweep)*mesh.fade_dist)
            n = max(1,math.ceil(r*abs(sweep)*correction_slope/(2.*self.mesh_tolerance)))
            if n > 4096:
                raise self.printer.command_error('Native mesh arc exceeds 4096 spans; check mesh or tolerance')
            splits += [i/n for i in range(1,n)]
        if crossing is not None:
            # Coalesce a numerically coincident mesh split with the exact
            # origin split; never turn a zero-length pair into a full circle.
            splits = [f for f in splits if f in (0.,1.) or abs(f-crossing) > 1.e-10]
            splits.append(crossing)
        splits = sorted(set(splits))
        points = []
        for f in splits:
            p = [v+f*(w-v) for v,w in zip(start,end)]
            phi = angle+sweep*f
            p[0],p[1] = cx+r*math.cos(phi),cy+r*math.sin(phi)
            if f == 0.: p[:2] = start[:2]
            if f == 1.: p[:2] = end[:2]
            if f == crossing or (f in (0.,1.) and math.hypot(*p[:2]) <= CENTER_EPS):
                p[0]=p[1]=0.
            if mesh is not None:
                factor = mesh.get_z_factor(p[2])
                p[2] += mesh.fade_target + (factor*(mesh.z_mesh.calc_z(*p[:2])-mesh.fade_target) if mesh.z_mesh is not None else 0.)
            points.append(p)
        points[0] = self.th.get_position()
        moves = []
        for a,b in zip(points,points[1:]):
            g = arc_geometry(a,b,[cx-a[0],cy-a[1]],sweep < 0.)
            if g is None:
                raise self.printer.command_error('Unable to split native arc consistently')
            moves.append(ArcMove(self,a,b,speed,g))
        incoming = self.kin._center_extruding
        angle = self.kin.angle
        for move in moves:
            if move.start_pos[0] == move.start_pos[1] == 0.:
                target = math.atan2(move.start_tangent[1],move.start_tangent[0])
                if abs(wrap_angle(target-angle)) > 1.e-12 and self.kin._want_center_retract(move.start_pos,move.end_pos,incoming):
                    pulled,restored = self.kin._retract_pair(move.start_pos)
                    self.kin._preflight(move.start_pos,pulled,self.kin.center_retract_speed)
                    self.kin._preflight(pulled,restored,self.kin.center_unretract_speed)
            incoming = move.end_pos[0] == move.end_pos[1] == 0. and move.end_pos[3] > move.start_pos[3]
            angle = math.atan2(-move.end_tangent[1],-move.end_tangent[0]) if incoming else math.atan2(move.end_pos[1],move.end_pos[0])
        return moves,n

    def execute(self,start,end,speed,geometry):
        moves,n = self.prepare(start,end,speed,geometry)
        if n > 1: self.mesh_spans += len(moves)
        for move in moves:
            kin,th = self.kin,self.th
            if move.start_pos[0] == move.start_pos[1] == 0.:
                target = math.atan2(move.start_tangent[1],move.start_tangent[0])
                if abs(wrap_angle(target-kin.angle)) > 1.e-12:
                    kin._center_turn(target,kin._want_center_retract(move.start_pos,move.end_pos,kin._center_extruding))
            th.commanded_pos[:] = move.end_pos
            want_flush = th.lookahead.add_move(move)
            centered = move.end_pos[0] == move.end_pos[1] == 0.
            if centered:
                th.limit_next_junction_speed(0.)
                kin.angle = math.atan2(-move.end_tangent[1],-move.end_tangent[0])
            else:
                kin.angle = math.atan2(move.end_pos[1],move.end_pos[0])
            kin._center_extruding = centered and move.end_pos[3] > move.start_pos[3]
            if want_flush: th._process_lookahead(lazy=True)
            if th.print_time > th.need_check_pause: th._check_pause()
