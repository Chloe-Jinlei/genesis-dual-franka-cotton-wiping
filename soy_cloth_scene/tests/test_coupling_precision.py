"""Particle setter and wet-mass precision contract, with CPU native-IO stubs.

The stub models Genesis public setters: input arrays are converted to the
configured native float dtype, and position writes clear velocity. This tests
precision boundaries and ledgers, not GPU contact or SPH numerical stability.
"""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cloth_liquid_coupling import ClothLiquidCoupling


class NativeField:
    def __init__(self, value):
        self.value = np.asarray(value).copy()
        self.dtype = self.value.dtype

    def to_numpy(self):
        return self.value.copy()

    def from_numpy(self, value):
        self.value = np.asarray(value, dtype=self.dtype).copy()


class NativeEntity:
    def __init__(self, positions, mass, dtype, free=True):
        self.dtype = np.dtype(dtype)
        self.positions = np.asarray(positions, dtype=self.dtype).copy()
        self.velocities = np.zeros_like(self.positions)
        self.n_particles = len(self.positions)
        self.particle_start = 0
        self.particle_size = .003
        self.mesh = SimpleNamespace(faces=np.asarray([[0, 1, 2]], np.int64))
        self.active = np.ones(self.n_particles, dtype=bool)
        self.solver = SimpleNamespace(
            particles_info=SimpleNamespace(mass=NativeField(np.full(self.n_particles, mass, dtype))),
            particles=SimpleNamespace(free=NativeField(np.full((self.n_particles, 1), free, bool))))

    def get_particles_pos(self):
        return self.positions.copy()

    def get_particles_vel(self):
        return self.velocities.copy()

    def get_particles_active(self):
        return self.active.copy()

    def set_particles_pos(self, value):
        self.positions = np.asarray(value, dtype=self.dtype).copy()
        self.velocities[:] = 0.

    def set_particles_vel(self, value):
        self.velocities = np.asarray(value, dtype=self.dtype).copy()

    def set_particles_active(self, value):
        self.active = np.asarray(value, dtype=bool).copy()


def pair(dtype=np.float64, *, free=True, distance=.002, **options):
    origin = np.array([.123456789123, -.234567891234, .760000001234])
    cloth = NativeEntity(origin+[[0, 0, 0], [.01, 0, 0], [0, .01, 0]],
                         .0001000000000123, dtype, free=free)
    soy = NativeEntity([origin+[.003, .003, distance]], .0000010000000123, dtype)
    soy.velocities[:] = [0.0123456789123, -.0234567891234, 0.]
    cloth.velocities[:] = soy.velocities[0]
    config = dict(absorption_flux_kg_m2_s=100., minimum_contact_time_s=0., drag_rate_s=0.)
    config.update(options)
    coupling = ClothLiquidCoupling(SimpleNamespace(n_envs=0), cloth, soy, config)
    return cloth, soy, coupling


class CouplingPrecisionTests(unittest.TestCase):
    def test_fp64_contact_velocity_update_does_not_quantize_through_fp32(self):
        cloth, soy, c = pair(minimum_contact_time_s=100.)
        expected_c, expected_s = cloth.velocities.copy(), soy.velocities.copy()
        c.step(.004)
        self.assertEqual(c.metrics()['last_contact_count'], 1)
        self.assertEqual(c.metrics()['absorbed_particle_count'], 0)
        np.testing.assert_array_equal(cloth.velocities, expected_c)
        np.testing.assert_array_equal(soy.velocities, expected_s)
        self.assertEqual(cloth.velocities.dtype, np.dtype(np.float64))

    def test_fp64_projection_retains_fixed_cloth_positions_and_in_plane_liquid_coordinates(self):
        cloth, soy, c = pair(free=False, distance=.0007, minimum_contact_time_s=100.)
        cloth.velocities[:] = soy.velocities[:] = 0.
        expected_c, expected_xy = cloth.positions.copy(), soy.positions[:, :2].copy()
        c.step(.004)
        self.assertGreater(c.metrics()['geometric_projection_count'], 0)
        np.testing.assert_array_equal(cloth.positions, expected_c)
        np.testing.assert_allclose(soy.positions[:, :2], expected_xy, atol=5e-17, rtol=0)
        self.assertGreater(soy.positions[0, 2], c._initial_soy_pos[0, 2])

    def test_fp64_absorbed_parcel_advection_and_velocity_preserve_material_interpolation(self):
        cloth, soy, c = pair()
        c.step(.004)
        self.assertEqual(len(c.bound_particle_indices), 1)
        cloth.positions += [.023456789123, -.034567891234, .123456789123]
        cloth.velocities[:] = [[.012345678912, -.023456789123, .003456789123],
                               [-.043456789123, .053456789123, .006456789123],
                               [.073456789123, -.083456789123, .009456789123]]
        ids = c.bound_particle_indices
        expected_p = np.einsum('ni,nij->nj', c._bound_weights[ids], cloth.positions[c.faces[c._bound_face[ids]]])
        expected_v = np.einsum('ni,nij->nj', c._bound_weights[ids], cloth.velocities[c.faces[c._bound_face[ids]]])
        expected_cv = cloth.velocities.copy()
        c.step(.004)
        np.testing.assert_array_equal(soy.positions[ids], expected_p)
        np.testing.assert_array_equal(soy.velocities[ids], expected_v)
        np.testing.assert_array_equal(cloth.velocities, expected_cv)
        self.assertFalse(soy.active.any())

    def test_fp64_reset_restores_exact_initial_liquid_state_and_dry_mass(self):
        cloth, soy, c = pair()
        positions, velocities = soy.positions.copy(), soy.velocities.copy()
        dry = cloth.solver.particles_info.mass.to_numpy()
        c.step(.004)
        self.assertEqual(c.metrics()['absorbed_particle_count'], 1)
        c.reset()
        np.testing.assert_array_equal(soy.positions, positions)
        np.testing.assert_array_equal(soy.velocities, velocities)
        np.testing.assert_array_equal(cloth.solver.particles_info.mass.to_numpy(), dry)
        self.assertTrue(soy.active.all())

    def test_fp64_wet_mass_and_matching_ledger_restore_preserve_native_precision(self):
        cloth, soy, c = pair(wicking={'enabled': True, 'diffusivity_m2_s': 5e-6})
        for _ in range(20):
            c.step(.004)
        self.assertEqual(c.metrics()['absorbed_particle_count'], 1)
        state = c.capture_state()
        native_p, native_v = soy.positions.copy(), soy.velocities.copy()
        expected = c.dry_vertex_mass+c.vertex_absorbed_mass
        np.testing.assert_array_equal(cloth.solver.particles_info.mass.to_numpy(), expected)
        self.assertEqual(cloth.solver.particles_info.mass.to_numpy().dtype, np.dtype(np.float64))
        self.assertLess(abs(c.metrics()['cloth_loading_error_kg']), 1e-19)
        c.reset()
        soy.set_particles_pos(native_p); soy.set_particles_vel(native_v)
        c.restore_state(state)
        np.testing.assert_array_equal(cloth.solver.particles_info.mass.to_numpy(), expected)
        np.testing.assert_array_equal(c.vertex_absorbed_mass, state['vertex_absorbed_mass'])
        self.assertLess(abs(c.metrics()['mass_balance_error_kg']), 1e-19)
        self.assertLess(abs(c.metrics()['vertex_reservoir_error_kg']), 1e-19)
        self.assertFalse(soy.active.any())

    def test_fp32_native_setters_still_choose_fp32_and_keep_ledger_conservative(self):
        cloth, soy, c = pair(dtype=np.float32)
        initial_p, initial_v = soy.positions.copy(), soy.velocities.copy()
        c.step(.004)
        self.assertEqual(c.metrics()['absorbed_particle_count'], 1)
        for array in (cloth.positions, cloth.velocities, soy.positions, soy.velocities,
                      cloth.solver.particles_info.mass.to_numpy()):
            self.assertEqual(array.dtype, np.dtype(np.float32))
        self.assertLess(abs(c.metrics()['mass_balance_error_kg']), 1e-19)
        self.assertLess(abs(c.metrics()['cloth_loading_error_kg']), 1e-10)
        c.reset()
        np.testing.assert_array_equal(soy.positions, initial_p)
        np.testing.assert_array_equal(soy.velocities, initial_v)


if __name__ == '__main__':
    unittest.main()
