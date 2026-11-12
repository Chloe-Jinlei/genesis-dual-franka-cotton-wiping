"""Mechanical invariants of the optional velocity damping, without Genesis/GPU."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cotton_bending import hinge_angle_gradient
from cotton_bending_damping import BendingDampingReference, damp_hinge_reference


class BendingDampingTests(unittest.TestCase):
    def setUp(self):
        self.p = np.array([[0, 0, 0], [.003, 0, 0], [.0007, .002, .0008],
                           [.0018, -.0023, -.0001]])+[.2, -.1, .76]
        self.mass = np.array([2., 3., 4., 5.])*1e-6
        self.free = np.ones(4, dtype=bool)
        self.v = np.random.default_rng(7).normal(size=(4, 3))*.08
        self.k, self.zeta, self.dt = 2e-4, .35, 1e-4

    def damp(self, velocities=None, masses=None, free=None, **kwargs):
        options = dict(stiffness=self.k, damping_ratio=self.zeta, dt=self.dt)
        options.update(kwargs)
        return damp_hinge_reference(self.p, self.v if velocities is None else velocities,
                                    self.mass if masses is None else masses,
                                    self.free if free is None else free, **options)

    @staticmethod
    def momentum(points, velocities, masses):
        linear = np.sum(masses[:, None]*velocities, axis=0)
        angular = np.sum(np.cross(points, masses[:, None]*velocities), axis=0)
        return linear, angular

    def test_single_hinge_rate_has_implicit_not_explicit_decay(self):
        changed, stats = self.damp()
        _, g = hinge_angle_gradient(self.p)
        W = np.sum(g*g/self.mass[:, None])
        rate = np.sum(g*self.v)
        c = 2*self.zeta*np.sqrt(self.k/W)
        expected = rate/(1+self.dt*c*W)
        self.assertAlmostEqual(np.sum(g*changed), expected, places=11)
        self.assertLess(stats["angular_mode_kinetic_energy_after_J"],
                        stats["angular_mode_kinetic_energy_before_J"])
        self.assertAlmostEqual(stats["passivity_balance_error_J"], 0., delta=1e-21)

    def test_free_hinge_preserves_linear_and_angular_momentum_and_loses_energy(self):
        changed, stats = self.damp()
        before = self.momentum(self.p, self.v, self.mass)
        after = self.momentum(self.p, changed, self.mass)
        np.testing.assert_allclose(after, before, rtol=0, atol=1e-21)
        self.assertLess(stats["kinetic_energy_change_free_J"], 0.)
        self.assertGreater(stats["dissipated_energy_J"], 0.)
        np.testing.assert_array_equal(stats["support_impulse_on_free_kg_m_s"], 0.)

    def test_rigid_translation_and_rotation_unchanged_even_with_moving_anchors(self):
        rigid = [.13, -.09, .07]+np.cross([1.2, -.7, .4], self.p-self.p.mean(0))
        for flags in (self.free, np.array([False, True, False, True])):
            with self.subTest(free=flags.tolist()):
                changed, stats = self.damp(velocities=rigid, free=flags)
                np.testing.assert_allclose(changed, rigid, rtol=0, atol=1e-15)
                self.assertLess(stats["dissipated_energy_J"], 1e-30)

    def test_stationary_anchors_stay_fixed_and_account_for_support_impulse(self):
        flags = np.array([False, True, True, False])
        velocity = self.v.copy(); velocity[~flags] = 0.
        changed, stats = self.damp(velocities=velocity, free=flags)
        np.testing.assert_array_equal(changed[~flags], velocity[~flags])
        before = self.momentum(self.p[flags], velocity[flags], self.mass[flags])
        after = self.momentum(self.p[flags], changed[flags], self.mass[flags])
        np.testing.assert_allclose(after[0]-before[0], stats["support_impulse_on_free_kg_m_s"], atol=1e-21)
        np.testing.assert_allclose(after[1]-before[1], stats["support_angular_impulse_on_free_kg_m2_s"], atol=1e-21)
        self.assertEqual(stats["anchor_work_J"], 0.)
        self.assertLess(stats["kinetic_energy_change_free_J"], 0.)

    def test_moving_anchor_can_supply_energy_but_passivity_includes_its_work(self):
        flags = np.array([False, True, True, True])
        velocity = np.zeros((4, 3)); velocity[0, 2] = .2
        changed, stats = self.damp(velocities=velocity, free=flags)
        np.testing.assert_array_equal(changed[~flags], velocity[~flags])
        self.assertGreater(stats["kinetic_energy_change_free_J"], 0.)
        self.assertGreater(stats["anchor_work_J"], stats["kinetic_energy_change_free_J"])
        self.assertGreater(stats["dissipated_energy_J"], 0.)
        self.assertAlmostEqual(stats["passivity_balance_error_J"], 0., delta=1e-21)

    def test_current_wet_mass_changes_effective_rate_without_caching_dry_inverse(self):
        dry, d = self.damp()
        wet_mass = 4*self.mass
        wet, w = self.damp(masses=wet_mass)
        self.assertAlmostEqual(w["angular_inverse_mass"], d["angular_inverse_mass"]/4)
        # Same ratio/stiffness: a heavier hinge has lower natural frequency,
        # hence less rate removal over the same physical time step.
        self.assertLess(w["fraction_removed"], d["fraction_removed"])
        self.assertGreater(np.linalg.norm(wet-dry), 1e-5)
        np.testing.assert_allclose(self.momentum(self.p, wet, wet_mass),
                                   self.momentum(self.p, self.v, wet_mass), rtol=0, atol=1e-20)

    def test_large_step_and_strong_damping_remain_finite_and_passive(self):
        changed, stats = self.damp(dt=1e6, damping_ratio=1e6)
        self.assertTrue(np.isfinite(changed).all())
        self.assertLess(abs(stats["angular_speed_after"]), abs(stats["angular_speed_before"])*1e-10)
        self.assertLess(stats["kinetic_energy_change_free_J"], 0.)
        self.assertAlmostEqual(stats["passivity_balance_error_J"], 0., delta=1e-21)

    def test_disabled_all_prescribed_and_collapsed_hinges_are_noops(self):
        for changed, stats in (self.damp(damping_ratio=0.), self.damp(free=np.zeros(4, dtype=bool))):
            np.testing.assert_array_equal(changed, self.v)
            self.assertFalse(stats["applied"])
        collapsed = self.p.copy(); collapsed[2] = collapsed[0]
        changed, stats = damp_hinge_reference(collapsed, self.v, self.mass, self.free,
                                             stiffness=self.k, damping_ratio=.5, dt=self.dt)
        np.testing.assert_array_equal(changed, self.v)
        self.assertEqual(stats["skip_reason"], "collapsed_hinge")

    def test_colored_mesh_pass_is_passive_and_never_changes_elastic_geometry(self):
        x, y = np.meshgrid(np.linspace(0, .009, 4), np.linspace(0, .006, 3))
        points = np.column_stack((x.ravel(), y.ravel(), np.zeros(12)))
        faces = []
        for j in range(2):
            for i in range(3):
                a = 4*j+i
                faces.extend(((a, a+1, a+4), (a+1, a+5, a+4)))
        model = BendingDampingReference(points, faces, damping_ratio=.2)
        for start, stop in zip(model.color_bounds[:-1], model.color_bounds[1:]):
            ids = model.hinges[start:stop].ravel()
            self.assertEqual(len(ids), len(np.unique(ids)))
        points[:, 2] = .0006*np.sin(points[:, 0]*400)*np.cos(points[:, 1]*300)
        positions_before = points.copy()
        velocity = np.random.default_rng(8).normal(size=(12, 3))*.05
        mass = np.linspace(1e-6, 7e-6, 12)
        energy_before = sum(.5*k*hinge_angle_gradient(points[h])[0]**2
                            for h, k in zip(model.hinges, model.stiffness))
        changed, stats = model.apply(points, velocity, mass, np.ones(12, dtype=bool), self.dt)
        np.testing.assert_array_equal(points, positions_before)
        energy_after = sum(.5*k*hinge_angle_gradient(points[h])[0]**2
                           for h, k in zip(model.hinges, model.stiffness))
        self.assertEqual(energy_after, energy_before)
        self.assertLess(stats["kinetic_energy_change_free_J"], 0.)
        np.testing.assert_allclose(self.momentum(points, changed, mass),
                                   self.momentum(points, velocity, mass), rtol=0, atol=1e-20)
        self.assertAlmostEqual(stats["passivity_balance_error_J"], 0., delta=1e-21)

    def test_invalid_mass_flags_or_parameters_are_rejected(self):
        for options in ({"damping_ratio": -.1}, {"damping_ratio": float("nan")},
                        {"dt": 0.}, {"stiffness": -1.}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.damp(**options)
        with self.assertRaises(ValueError):
            self.damp(masses=np.array([0, 1, 1, 1]))
        with self.assertRaises(ValueError):
            self.damp(free=np.ones(4, dtype=int))


if __name__ == "__main__":
    unittest.main()
