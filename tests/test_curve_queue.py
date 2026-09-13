import math
from types import SimpleNamespace
import unittest
from kinematics.polar_center import PolarCenterKinematics
from kinematics.polar_native_arc import NativeArcs, ArcMove, arc_geometry, nominal_mesh_z
from toolhead import Move


def fixture():
    th = SimpleNamespace(max_velocity=40.,max_accel=300.,junction_deviation=.01,
                         mcr_pseudo_accel=150.,extra_axes=[],printer=SimpleNamespace(command_error=RuntimeError))
    kin = object.__new__(PolarCenterKinematics)
    kin.printer = th.printer
    th.printer.lookup_object = lambda *args: None
    kin.center_retract_length = 0.
    kin._homing = False
    kin.limit_xy2,kin.limit_z = 10000.,(0.,150.)
    kin.max_z_velocity,kin.max_z_accel = 3.,30.
    kin.radial_velocity,kin.radial_accel = 40.,300.
    kin.v_rad_max,kin.angular_accel = 1.,2.
    kin.radial_velocity_change,kin.angular_velocity_change = 1.,.02
    kin.slow_radius,kin.angle,kin._center_extruding = 5.,0.,False
    support = object.__new__(NativeArcs)
    support.th,support.kin,support.printer = th,kin,th.printer
    support.fit_tolerance,support.mesh_tolerance = .01,.005
    support.arcs = SimpleNamespace(gcode_move=SimpleNamespace(move_transform=None))
    return support


class CurveQueueTests(unittest.TestCase):
    def test_tangent_arcs_keep_speed_and_preserve_extruder_junction_limit(self):
        support = fixture()
        support.th.extra_axes = [SimpleNamespace(check_move=lambda *a: None,calc_junction=lambda *a: .04)]
        a,b,c = [30,0,10,0],[0,30,10,.1],[-30,0,10,.2]
        first = ArcMove(support,a,b,20,arc_geometry(a,b,[-30,0],False))
        second = ArcMove(support,b,c,20,arc_geometry(b,c,[0,-30],False))
        prior_axes = first.axes_r[:]
        second.calc_junction(first)
        self.assertGreater(second.max_start_v2,0.)
        self.assertLessEqual(second.max_start_v2,.04)
        self.assertEqual(first.axes_r,prior_axes)

    def test_arc_to_line_uses_end_tangent(self):
        support = fixture()
        a,b,c = [30,0,10,0],[0,30,10,0],[-10,30,10,0]
        first = ArcMove(support,a,b,20,arc_geometry(a,b,[-30,0],False))
        second = Move(support.th,b,c,20)
        support.kin.check_move(second)
        second.calc_junction(first)
        self.assertGreater(second.max_start_v2,1.)

    def test_rounded_circle_preserves_endpoints_and_bounds_repair(self):
        start,end = [30,0],[.001,30.001]
        g = arc_geometry(start,end,[-30,0],False,.02)
        self.assertIsNotNone(g)
        cx,cy,r = g[:3]
        self.assertAlmostEqual(math.hypot(start[0]-cx,start[1]-cy),r)
        self.assertAlmostEqual(math.hypot(end[0]-cx,end[1]-cy),r)
        self.assertLessEqual(math.hypot(cx,cy)+abs(r-30),.02)
        self.assertIsNone(arc_geometry(start,[0,32],[-30,0],False,.02))

    def test_helical_move_uses_3d_length_and_z_limits(self):
        support = fixture()
        a,b = [30,0,10,0],[30,0,30,1]
        move = ArcMove(support,a,b,40,arc_geometry(a,b,[-30,0],False))
        self.assertAlmostEqual(move.move_d,math.hypot(60*math.pi,20))
        self.assertLessEqual(abs(move.start_tangent[2])*math.sqrt(move.max_cruise_v2),3.)
        self.assertLessEqual(abs(move.start_tangent[2])*move.accel,30.)

    def test_g1_fit_preserves_e_and_rejects_large_deviation(self):
        support = fixture()
        def candidate(angle):
            a,b,c = [[30*math.cos(i*angle),30*math.sin(i*angle),10,i*.01] for i in range(3)]
            first,second = Move(support.th,a,b,20),Move(support.th,b,c,20)
            support.kin.check_move(first)
            support.kin.check_move(second)
            return support.fit_pair(first,second),a,c
        move,a,c = candidate(.03)
        self.assertIsNotNone(move)
        self.assertEqual(move.start_pos,tuple(a))
        self.assertEqual(move.end_pos,tuple(c))
        move,_,_ = candidate(.4)
        self.assertIsNone(move)

    def test_mesh_error_is_bounded_through_fade_and_cell_edges(self):
        from extras.bed_mesh import ZMesh
        support = fixture()
        params = dict(min_x=-40.,max_x=40.,min_y=-40.,max_y=40.,x_count=3,y_count=3,
                      mesh_x_pps=0,mesh_y_pps=0,algo='direct',tension=.2)
        zm = ZMesh(params,'unit')
        zm.build_mesh([[.3,-.1,.2],[-.2,.1,-.15],[.1,-.3,.2]])
        def factor(z): return min(1.,max(0.,(10.-z)/9.))
        mesh = SimpleNamespace(z_mesh=zm,fade_target=.05,fade_end=10.,fade_dist=9.,
                               FADE_DISABLE=0x7fffffff,get_z_factor=factor)
        support.arcs.gcode_move.move_transform = mesh
        a,b = [30,0,.5,0],[30,0,12,.1]
        height = lambda x,y,z: z+.05+factor(z)*(zm.calc_z(x,y)-.05)
        support.th.get_position = lambda: [30,0,height(30,0,.5),0]
        moves,n = support.prepare(a,b,20,arc_geometry(a,b,[-30,0],False))
        self.assertGreater(n,1)
        for index,move in enumerate(moves):
            for i in range(21):
                f = i/20.
                total = (index+f)/n
                x,y = 30*math.cos(2*math.pi*total),30*math.sin(2*math.pi*total)
                z = move.start_pos[2]+f*(move.end_pos[2]-move.start_pos[2])
                expected = height(x,y,.5+11.5*total)
                self.assertLessEqual(abs(z-expected),support.mesh_tolerance+1.e-10)

    def test_mesh_inverse_with_fade_target_and_tool_offset(self):
        zm = SimpleNamespace(calc_z=lambda x,y: .2+.001*x-.002*y)
        mesh = SimpleNamespace(z_mesh=zm,fade_target=.05,fade_start=1.,fade_end=10.,
                               fade_dist=9.,tool_offset=.4)
        for x,y in [(0,0),(30,-20),(-50,10)]:
            for z in [-1.,.6,.9,3.,8.,9.6,12.]:
                f = min(1.,max(0.,(mesh.fade_end-z-mesh.tool_offset)/mesh.fade_dist))
                actual = z+mesh.fade_target+f*(zm.calc_z(x,y)-mesh.fade_target)
                self.assertAlmostEqual(nominal_mesh_z(mesh,[x,y,actual]),z)

    def test_mesh_origin_split_does_not_create_an_extra_full_circle(self):
        from extras.bed_mesh import ZMesh
        support = fixture()
        params = dict(min_x=-40.,max_x=40.,min_y=-40.,max_y=40.,x_count=3,y_count=3,
                      mesh_x_pps=0,mesh_y_pps=0,algo='direct',tension=.2)
        zm = ZMesh(params,'origin')
        zm.build_mesh([[-.1,0.,.1],[-.1,0.,.1],[-.1,0.,.1]])
        mesh = SimpleNamespace(z_mesh=zm,fade_target=0.,fade_end=0x7fffffff,
                               FADE_DISABLE=0x7fffffff,get_z_factor=lambda z: 1.)
        support.arcs.gcode_move.move_transform = mesh
        for phi in [.3,.7,1.2,-2.9]:
            a = [20*math.cos(phi),20*math.sin(phi),.5,0.]
            support.th.get_position = lambda: a[:2]+[.5+zm.calc_z(*a[:2]),0.]
            offset = [-a[0]/2.,-a[1]/2.]
            moves,n = support.prepare(a,a,20.,arc_geometry(a,a,offset,False))
            self.assertAlmostEqual(sum(abs(m.geometry[4]) for m in moves),2.*math.pi)
            self.assertEqual(sum(m.end_pos[0] == m.end_pos[1] == 0. for m in moves),1)
