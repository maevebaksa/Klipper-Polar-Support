import math
import random
from types import SimpleNamespace
import unittest
from kinematics.polar_center import PolarCenterKinematics, derivative_bounds
from kinematics.polar_native_arc import arc_geometry, NativeArcs
import test_geometry


class ExactBoundTests(unittest.TestCase):
    def test_angular_acceleration_extremum_is_inside_segment(self):
        f, g, k, rp = derivative_bounds([-10, 2], [10, 2], 20)
        self.assertAlmostEqual(f, .5)
        self.assertAlmostEqual(g, 9./(8.*math.sqrt(3.)*4.))
        self.assertAlmostEqual(k, .5)
        self.assertAlmostEqual(rp, 10./math.sqrt(104.))
        self.assertLess(g, .5)  # Previous bound was 2/h^2.

    def test_short_tangential_move_uses_actual_radial_slope(self):
        *_, rp = derivative_bounds([-.01, 20], [.01, 20], .02)
        self.assertLess(rp, .001)


class RetractionTests(unittest.TestCase):
    def setUp(self):
        self.kin = object.__new__(PolarCenterKinematics)
        self.firmware = SimpleNamespace(is_retracted=False)
        self.kin.printer = SimpleNamespace(lookup_object=lambda *a: self.firmware)
        self.kin.center_retract_length = .3

    def test_only_continuing_extrusion_retracts(self):
        k = self.kin
        self.assertTrue(k._want_center_retract([0,0,0,1], [10,0,0,2], True))
        for incoming, e in [(False,2), (True,1), (True,0)]:
            self.assertFalse(k._want_center_retract([0,0,0,1], [10,0,0,e], incoming))
        self.firmware.is_retracted = True
        self.assertFalse(k._want_center_retract([0,0,0,1], [10,0,0,2], True))
        self.firmware.is_retracted = False
        k.center_retract_length = 0
        self.assertFalse(k._want_center_retract([0,0,0,1], [10,0,0,2], True))

    def test_pair_preserves_coordinates_and_net_extrusion(self):
        start = [0,0,12,7]
        pulled, restored = self.kin._retract_pair(start)
        self.assertEqual(pulled, [0,0,12,6.7])
        self.assertEqual(restored, start)
        self.assertIsNot(restored, start)


class ArcGeometryTests(unittest.TestCase):
    def test_full_circles_have_correct_direction_and_bounds(self):
        for cw in [True, False]:
            arc = arc_geometry([30,0], [30,0], [-30,0], cw)
            self.assertEqual(arc[4], (-1 if cw else 1)*2*math.pi)
            self.assertAlmostEqual(arc[5], 30)
            self.assertAlmostEqual(arc[6], 30)
        arc = arc_geometry([20,0], [20,0], [-10,0], False)
        self.assertLess(arc[5], 1.e-12)

    def test_swept_bounds_include_interior_extrema(self):
        rng = random.Random(708)
        for _ in range(300):
            cx, cy = rng.uniform(-40,40), rng.uniform(-40,40)
            r, a, sweep = rng.uniform(.1,30), rng.uniform(-math.pi,math.pi), rng.uniform(-6,6)
            start = [cx+r*math.cos(a), cy+r*math.sin(a)]
            end = [cx+r*math.cos(a+sweep), cy+r*math.sin(a+sweep)]
            arc = arc_geometry(start, end, [cx-start[0],cy-start[1]], sweep < 0)
            for i in range(101):
                p = a+sweep*i/100
                value = math.hypot(cx+r*math.cos(p),cy+r*math.sin(p))
                self.assertGreaterEqual(value+1.e-10, arc[5])
                self.assertLessEqual(value-1.e-10, arc[6])

    def test_inconsistent_circle_falls_back(self):
        self.assertIsNone(arc_geometry([10,0], [0,10.01], [-10,0], False))

    def test_transform_gate_never_bypasses_active_mesh_or_custom_transform(self):
        support = object.__new__(NativeArcs)
        Mesh = type('BedMesh', (), {'__module__': 'extras.bed_mesh'})
        mesh = Mesh()
        mesh.z_mesh, mesh.fade_target = None, 0.
        support.printer = SimpleNamespace(lookup_object=lambda *a: mesh)
        gm = SimpleNamespace(move_transform=mesh)
        self.assertTrue(support.transform_supported(gm))
        mesh.z_mesh = object()
        self.assertTrue(support.transform_supported(gm))
        mesh.z_mesh, mesh.fade_target = None, .1
        self.assertTrue(support.transform_supported(gm))
        gm.move_transform = SimpleNamespace()
        self.assertFalse(support.transform_supported(gm))
        gm.move_transform = None
        self.assertTrue(support.transform_supported(gm))


class NativeSolverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        test_geometry.CompiledSolverTests.setUpClass.__func__(cls)
        import ctypes
        cls.arc = cls.lib.sample_arc
        cls.arc.argtypes = [ctypes.c_int]+[ctypes.c_double]*9
        cls.arc.restype = ctypes.c_double

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_continuous_winding_even_before_commanded_position_updates(self):
        # itersolve updates commanded_pos only at the end of a queue phase.
        for direction in [-1,1]:
            for cx,cy,r,angle,span in [(0,0,30,0,2*math.pi),
                (3,4,20,2,2*math.pi), (-20,5,4,0,2*math.pi),
                (10,0,10,0,.9*math.pi)]:
                prev = None
                for i in range(401):
                    s = r*span*i/400
                    phi = angle+direction*s/r
                    x,y = cx+r*math.cos(phi), cy+r*math.sin(phi)
                    theta = self.arc(1,0,cx,cy,r,angle,direction/r,.003,.0001,s)
                    radius = self.arc(0,0,cx,cy,r,angle,direction/r,.003,.0001,s)
                    self.assertAlmostEqual(radius-.003,math.hypot(x,y))
                    self.assertAlmostEqual(math.remainder(theta-.0001-math.atan2(y,x),2*math.pi),0.)
                    if prev is not None:
                        self.assertLess(abs(theta-prev), .1)
                    prev = theta
