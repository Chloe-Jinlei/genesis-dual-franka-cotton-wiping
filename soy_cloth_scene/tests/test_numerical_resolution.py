"""CPU regressions for erased PBD gravity; no Genesis scene or GPU is built."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from numerical_resolution import PBDGravityResolutionError, validate_pbd_gravity_resolution


def reconstructed_free_fall(dtype, h, seconds, initial_velocity=0.0):
    """Native PBD free-particle arithmetic, with no constraints or contacts."""
    z, v, step, gravity = map(dtype, (.764, initial_velocity, h, -9.81))
    for _ in range(round(seconds/h)):
        previous = z
        v = dtype(v + gravity*step)
        z = dtype(z + v*step)
        v = dtype((z-previous)/step)
    return float(z), float(v)


class GravityResolutionTests(unittest.TestCase):
    def test_rejects_observed_float32_combinations_with_diagnostics(self):
        for height in (.76, 1.1):
            with self.subTest(height=height):
                with self.assertRaises(PBDGravityResolutionError) as error:
                    validate_pbd_gravity_resolution(.002, 40, "32", height)
                m = error.exception.metrics
                self.assertEqual(m["h"], .00005)
                self.assertAlmostEqual(m["gravity_step"], 2.4525e-8)
                self.assertLessEqual(m["ratio"], .5)
                self.assertIn("velocity reconstruction can erase gravity", str(error.exception))

    def test_accepts_double_precision_or_resolvable_substep(self):
        for height in (.76, 1.1):
            for dt, precision in ((.002, "64"), (.004, 32)):
                with self.subTest(height=height, precision=precision):
                    m = validate_pbd_gravity_resolution(dt, 40, precision, height)
                    self.assertGreater(m["ratio"], .5)
                    self.assertEqual(m["h"], dt/40)

    def test_height_is_world_coordinate_not_table_clearance(self):
        # The 100 us step passes at 1.1 m but not at 2.1 m, whose ULP doubles.
        low = validate_pbd_gravity_resolution(.004, 40, 32, 1.1)
        with self.assertRaises(PBDGravityResolutionError) as error:
            validate_pbd_gravity_resolution(.004, 40, 32, 2.1)
        self.assertEqual(error.exception.metrics["ulp"], 2*low["ulp"])
        self.assertEqual(error.exception.metrics["ratio"], low["ratio"]/2)

    def test_float32_reconstruction_erases_static_and_airborne_gravity(self):
        z, v = reconstructed_free_fall(np.float32, .00005, .25)
        self.assertEqual(z, float(np.float32(.764)))
        self.assertEqual(v, 0.)
        z, v = reconstructed_free_fall(np.float32, .00005, .25, .5)
        self.assertGreater(z, .88)
        self.assertGreater(v, .49)  # Physical gravity should already make v negative.

    def test_double_precision_reconstruction_matches_semi_implicit_fall(self):
        h, duration, initial_v = .00005, .25, .5
        z, v = reconstructed_free_fall(np.float64, h, duration, initial_v)
        expected_z = .764+initial_v*duration-9.81*duration*(duration+h)/2
        self.assertAlmostEqual(z, expected_z, delta=1e-8)
        self.assertAlmostEqual(v, initial_v-9.81*duration, delta=1e-8)

    def test_larger_float32_step_resolves_fall_without_claiming_accuracy(self):
        validate_pbd_gravity_resolution(.002, 20, 32, 1.1)
        z, v = reconstructed_free_fall(np.float32, .0001, .1)
        self.assertLess(z, .73)
        self.assertLess(v, -.5)
        # A passed minimum check does not remove appreciable roundoff bias.
        self.assertGreater(abs(z-(.764-9.81*.1*(.1+.0001)/2)), .001)

    def test_invalid_inputs_fail_explicitly(self):
        base = dict(dt=.002, substeps=40, precision=64, max_height=1.1)
        bad = {"dt": (0, -1, math.nan, math.inf, True),
               "substeps": (0, -1, 2.5, True), "precision": (16, "single", True),
               "max_height": (0, -1, math.nan, math.inf, True)}
        for key, values in bad.items():
            for value in values:
                with self.subTest(key=key, value=value):
                    with self.assertRaises(ValueError):
                        validate_pbd_gravity_resolution(**{**base, key: value})


if __name__ == "__main__":
    unittest.main()
