"""Check junction limits independently of the move-splitting implementation."""
import math
import random
import unittest
from types import SimpleNamespace
from kinematics.polar_center import corner_speed_limit, PolarCenterKinematics
from toolhead import Move


class CornerTests(unittest.TestCase):
    def test_collinear_moves_keep_speed(self):
        self.assertTrue(math.isinf(corner_speed_limit([20, 0], [1, 0], [1, 0], 1, .02)))

    def test_center_always_stops(self):
        self.assertEqual(corner_speed_limit([0, 0], [1, 0], [1, 0], 1, .02), 0)

    def test_radial_and_angular_limits(self):
        self.assertAlmostEqual(corner_speed_limit([20, 0], [1, 0], [-1, 0], 1, .02), .5)
        self.assertAlmostEqual(corner_speed_limit([20, 0], [0, 1], [0, -1], 1, .02), .2)
        self.assertAlmostEqual(corner_speed_limit([1, 0], [0, 1], [0, -1], 1, .02), .01)

    def test_zero_change_limit_is_respected(self):
        self.assertEqual(corner_speed_limit([20, 0], [1, 0], [0, 1], 0, .02), 0)
        self.assertEqual(corner_speed_limit([20, 0], [1, 0], [0, 1], 1, 0), 0)

    def test_random_junction_motor_velocities(self):
        rng = random.Random(84)
        for _ in range(4000):
            r = 10**rng.uniform(-5, 2)
            angle = rng.uniform(-math.pi, math.pi)
            x, y = r*math.cos(angle), r*math.sin(angle)
            vectors = []
            for _ in range(2):
                v = [rng.uniform(-1, 1) for _ in range(3)]
                length = math.sqrt(sum(c*c for c in v))
                vectors.append([c/length for c in v])
            limit = corner_speed_limit([x, y], *vectors, 1, .02)
            velocities = [(limit*(x*v[0]+y*v[1])/r,
                           limit*(x*v[1]-y*v[0])/r**2) for v in vectors]
            self.assertLessEqual(abs(velocities[1][0]-velocities[0][0]), 1+1.e-9)
            self.assertLessEqual(abs(velocities[1][1]-velocities[0][1]), .02+1.e-9)

    def test_native_and_extruder_limits_are_preserved(self):
        # Use real Klipper Move objects and its calc_junction implementation.
        extra = SimpleNamespace(calc_junction=lambda *args: .04)
        th = SimpleNamespace(junction_deviation=.01, max_velocity=50,
                             max_accel=300, mcr_pseudo_accel=150, extra_axes=[extra])
        kin = PolarCenterKinematics.__new__(PolarCenterKinematics)
        kin._homing = False
        kin.limit_xy2 = 10000
        kin.limit_z = (0, 150)
        kin.slow_radius = 5
        kin.radial_velocity = 40
        kin.radial_accel = 300
        kin.v_rad_max = 1
        kin.angular_accel = 2
        kin.radial_velocity_change = 1
        kin.angular_velocity_change = .02
        prev = Move(th, [20, -1, 10, 0], [20, 0, 10, .1], 20)
        move = Move(th, [20, 0, 10, .1], [20.01, 1, 10, .2], 20)
        kin.check_move(prev)
        kin.check_move(move)
        move.calc_junction(prev)
        self.assertGreater(move.max_start_v2, 0)
        self.assertLessEqual(move.max_start_v2, .04)
        self.assertLessEqual(move.max_mcr_start_v2, move.max_start_v2)

