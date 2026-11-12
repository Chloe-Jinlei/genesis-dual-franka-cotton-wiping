"""Mechanical invariants of free-set rigid-motion residual damping; CPU only."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cotton_deformation_damping import damp_deformation_reference


class DeformationDampingTests(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(23)
        self.p = rng.normal(size=(20, 3))*.05+[.2, -.1, 1.]
        self.v = rng.normal(size=(20, 3))*.04
        self.m = rng.uniform(1e-6, 8e-6, 20)
        self.free = np.ones(20, dtype=bool)

    def damp(self, **changes):
        args = dict(points=self.p, velocities=self.v, masses=self.m, free=self.free,
                    rate_per_s=4., dt=.025)
        args.update(changes)
        return damp_deformation_reference(**args)

    @staticmethod
    def momentum(p, v, m):
        return np.sum(m[:, None]*v, axis=0), np.sum(np.cross(p, m[:, None]*v), axis=0)

    def test_free_set_linear_and_angular_momentum_preserved_and_energy_exactly_decays(self):
        out, d = self.damp()
        np.testing.assert_allclose(self.momentum(self.p, out, self.m),
                                   self.momentum(self.p, self.v, self.m), atol=2e-21, rtol=0)
        self.assertAlmostEqual(d['deformation_energy_after_J']/d['deformation_energy_before_J'],
                               np.exp(-2*4*.025), places=13)
        loss = d['deformation_energy_before_J']-d['deformation_energy_after_J']
        self.assertAlmostEqual(d['kinetic_energy_before_J']-d['kinetic_energy_after_J'], loss, delta=1e-21)

    def test_translation_rotation_and_rigid_swing_of_free_subset_are_preserved(self):
        flags = self.free.copy(); flags[[0, 4, 8]] = False
        rigid = [.2, -.3, .1]+np.cross([1.2, -.9, .7], self.p-self.p.mean(0))
        rigid[~flags] = [10., -3., 8.]  # Anchors do not bias free-set fit.
        for free in (self.free, flags):
            velocity = rigid if np.any(~free) else [.2, -.3, .1]+np.cross([1.2, -.9, .7], self.p)
            out, d = self.damp(velocities=velocity, free=free, dt=1.)
            np.testing.assert_allclose(out, velocity, atol=5e-16, rtol=0)
            self.assertLess(d['deformation_energy_before_J'], 1e-31)

    def test_fixed_velocities_unchanged_without_support_work_or_impulse(self):
        flags = self.free.copy(); flags[[2, 7, 14]] = False
        velocity = self.v.copy(); velocity[~flags] = [100., -120., 14.]
        out, d = self.damp(velocities=velocity, free=flags)
        np.testing.assert_array_equal(out[~flags], velocity[~flags])
        np.testing.assert_allclose(self.momentum(self.p[flags], out[flags], self.m[flags]),
                                   self.momentum(self.p[flags], velocity[flags], self.m[flags]), atol=2e-21, rtol=0)
        self.assertEqual(d['anchor_work_J'], 0.)
        np.testing.assert_array_equal(d['support_impulse_kg_m_s'], 0.)

    def test_heterogeneous_wet_masses_change_fit_and_conserve_current_wet_momentum(self):
        dry, _ = self.damp()
        wet = self.m.copy(); wet[:7] *= 7.
        out, d = self.damp(masses=wet)
        self.assertGreater(np.linalg.norm(out-dry), .001)
        np.testing.assert_allclose(self.momentum(self.p, out, wet), self.momentum(self.p, self.v, wet), atol=5e-21, rtol=0)
        self.assertAlmostEqual(d['free_mass_kg'], wet.sum())
        self.assertLessEqual(d['kinetic_energy_after_J'], d['kinetic_energy_before_J'])

    def test_world_translation_invariance_uses_centered_inertia(self):
        baseline, d = self.damp()
        translated, t = self.damp(points=self.p+[1e6, -3e6, 2e6])
        np.testing.assert_allclose(translated, baseline, atol=1e-10, rtol=0)
        np.testing.assert_allclose(t['rigid_angular_velocity_rad_s'], d['rigid_angular_velocity_rad_s'], atol=2e-9, rtol=0)

    def test_planar_collinear_coincident_and_single_free_point_are_safe(self):
        for rank, points in ((3, self.p*np.array([1, 1, 0])),
                             (2, np.column_stack((np.arange(20)*.003, np.zeros((20, 2))))),
                             (0, np.full((20, 3), .76))):
            with self.subTest(rank=rank):
                out, d = self.damp(points=points)
                self.assertEqual(d['inertia_rank'], rank)
                self.assertTrue(np.isfinite(out).all())
                np.testing.assert_allclose(self.momentum(points, out, self.m),
                                           self.momentum(points, self.v, self.m), atol=2e-21, rtol=0)
                self.assertLessEqual(d['kinetic_energy_after_J'], d['kinetic_energy_before_J'])
        flags = np.zeros(20, dtype=bool); flags[8] = True
        out, d = self.damp(free=flags)
        np.testing.assert_array_equal(out, self.v)
        self.assertEqual(d['inertia_rank'], 0)

    def test_zero_rate_all_fixed_and_empty_are_noops_without_input_mutation(self):
        p, v, m, f = self.p.copy(), self.v.copy(), self.m.copy(), self.free.copy()
        for changes in ({'rate_per_s': 0.}, {'free': np.zeros(20, dtype=bool)}):
            out, _ = self.damp(**changes)
            np.testing.assert_array_equal(out, self.v)
        for actual, before in zip((self.p, self.v, self.m, self.free), (p, v, m, f)):
            np.testing.assert_array_equal(actual, before)
        out, d = self.damp(points=np.empty((0, 3)), velocities=np.empty((0, 3)),
                           masses=np.empty(0), free=np.empty(0, dtype=bool))
        self.assertEqual(out.shape, (0, 3)); self.assertEqual(d['free_count'], 0)

    def test_large_step_is_finite_and_repeated_steps_have_exponential_semigroup(self):
        out, d = self.damp(dt=1e20)
        self.assertTrue(np.isfinite(out).all())
        self.assertLess(d['deformation_energy_after_J'], d['deformation_energy_before_J']*1e-28)
        half, _ = self.damp(dt=.0125)
        two, _ = self.damp(velocities=half, dt=.0125)
        whole, _ = self.damp(dt=.025)
        np.testing.assert_allclose(two, whole, atol=3e-17, rtol=0)

    def test_float32_final_velocity_cast_does_not_erase_substep_damping(self):
        out, d = self.damp(dt=1e-4, velocities=self.v.astype(np.float32))
        rounded = out.astype(np.float32).astype(np.float64)
        measured_loss = .5*np.sum(self.m[:, None]*(self.v.astype(np.float32).astype(float)**2-rounded**2))
        expected_loss = d['kinetic_energy_before_J']-d['kinetic_energy_after_J']
        self.assertGreater(measured_loss, 0.)
        self.assertLess(abs(measured_loss/expected_loss-1), 1e-4)

    def test_invalid_parameters_and_states_rejected(self):
        for options in ({'rate_per_s': -1.}, {'rate_per_s': np.nan}, {'dt': 0.}, {'dt': np.inf},
                        {'free': np.ones(20, dtype=int)}, {'masses': np.zeros(20)},
                        {'points': np.ones((19, 3))}, {'velocities': np.full((20, 3), np.nan)}):
            with self.subTest(options=list(options)), self.assertRaises(ValueError):
                self.damp(**options)


if __name__ == '__main__':
    unittest.main()
