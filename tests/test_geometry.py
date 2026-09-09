"""Run with the patched checkout on PYTHONPATH; standard-library unittest."""
import ctypes
import math
import os
from pathlib import Path
import random
import subprocess
import tempfile
import unittest
from kinematics.polar_center import (
    split_parameters, derivative_bounds, wrap_angle, PolarCenterKinematics)


class GeometryTests(unittest.TestCase):
    def test_center_crossings_all_quadrants(self):
        for angle in [0., .3, math.pi/2, math.pi, -math.pi/2, 2.9]:
            u, v = math.cos(angle), math.sin(angle)
            start, end = [30*u, 30*v, 10, 2], [-20*u, -20*v, 15, 7]
            ts, crossing = split_parameters(start, end, 5.)
            self.assertTrue(crossing)
            self.assertAlmostEqual(ts[0], .6)
            middle = [a + ts[0]*(b-a) for a, b in zip(start, end)]
            self.assertAlmostEqual(middle[2], 13.)
            self.assertAlmostEqual(middle[3], 5.)

    def test_endpoints_and_pure_z_are_not_internal_crossings(self):
        for a, b in [([20, 0], [0, 0]), ([0, 0], [-20, 0]),
                     ([0, 0], [0, 0]), ([20, 0], [10, 0])]:
            self.assertEqual(split_parameters(a, b, 5.), ([], False))
        for offset in [1.e-6, .001, .01, .1, 1., 5.]:
            self.assertEqual(split_parameters([30, offset], [0, 0], 5.),
                             ([], False))

    def test_near_center_is_not_detoured_or_dropped(self):
        for offset in [1.e-6, .001, .01, .1, 1.]:
            start, end = [-40, offset, 10, 0], [40, offset, 14, 8]
            ts, crossing = split_parameters(start, end, 5.)
            self.assertFalse(crossing)
            self.assertIn(.5, ts)
            self.assertLess(len(ts), 60)
            last, total_e = start, 0.
            for t in ts + [1.]:
                point = [a + t*(b-a) for a, b in zip(start, end)]
                self.assertEqual(point[1], offset)
                total_e += point[3] - last[3]
                last = point
            self.assertAlmostEqual(total_e, 8.)

    def test_derivative_bounds_against_sampled_exact_derivatives(self):
        rng = random.Random(731)
        for _ in range(400):
            start = [rng.uniform(-50, 50) for _ in range(3)]
            end = [rng.uniform(-50, 50) for _ in range(3)]
            ds = [b-a for a, b in zip(start, end)]
            length = math.sqrt(sum(d*d for d in ds))
            ux, uy = ds[0]/length, ds[1]/length
            f, g, k, q = derivative_bounds(start, end, length)
            for i in range(101):
                x, y = [start[j] + i/100*ds[j] for j in range(2)]
                r = math.hypot(x, y)
                cross, dot = x*uy-y*ux, x*ux+y*uy
                self.assertLessEqual(abs(cross/r**2), f*(1+1.e-10)+1.e-12)
                self.assertLessEqual(abs(-2*cross*dot/r**4), g*(1+1.e-10)+1.e-12)
                self.assertLessEqual(cross**2/r**3, k*(1+1.e-10)+1.e-12)
                self.assertLessEqual(abs(dot/r), q+1.e-12)

    def test_radial_and_vertical_derivatives(self):
        self.assertEqual(derivative_bounds([0, 0], [10, 0], 10), (0, 0, 0, 1))
        self.assertEqual(derivative_bounds([0, -10], [0, 0], 10), (0, 0, 0, 1))
        self.assertEqual(derivative_bounds([0, 0], [0, 0], 10), (0, 0, 0, 0))

    def test_shortest_rotation(self):
        self.assertAlmostEqual(wrap_angle(math.radians(-179-179)), math.radians(2))
        self.assertLessEqual(abs(wrap_angle(math.pi)), math.pi)


class CompiledSolverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.libpath = Path(cls.temp.name)/'angle.so'
        root = Path(os.environ['KLIPPER_PATH'])
        helper = root/'klippy/chelper'
        subprocess.run(['gcc', '-shared', '-fPIC', '-O2', '-Wall', '-Werror',
                        '-I'+str(helper), str(Path(__file__).with_name('angle_harness.c')),
                        '-lm', '-o', str(cls.libpath)], check=True)
        cls.lib = ctypes.CDLL(str(cls.libpath))
        cls.sample = cls.lib.sample_angle
        cls.sample.argtypes = [ctypes.c_int] + [ctypes.c_double]*6
        cls.sample.restype = ctypes.c_double

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_reproduces_upstream_origin_jump(self):
        old = self.sample(0, math.pi/2, 0, 10, 0, -1, 10)
        fixed = self.sample(1, math.pi/2, 0, 10, 0, -1, 10)
        self.assertAlmostEqual(old, 0.)
        self.assertAlmostEqual(fixed, math.pi/2)

    def test_all_inward_endpoints_hold_angle(self):
        for a in [-3, -2, -1, 0, 1, 2, 3]:
            x, y = math.cos(a), math.sin(a)
            for t in [0, .5, .99999999999, 1]:
                result = self.sample(1, a, x, y, -x, -y, t)
                self.assertAlmostEqual(result, a)

    def test_center_idle_and_z_only_hold_angle(self):
        for a in [-3, -1, 0, 1, 3]:
            self.assertEqual(self.sample(1, a, 0, 0, 0, 0, 1), a)

    def test_branch_cut_is_continuous(self):
        result = self.sample(1, math.pi-.001, -1, -.001, 0, 0, 0)
        self.assertAlmostEqual(result, math.pi+math.atan(.001))


if __name__ == '__main__':
    unittest.main()
