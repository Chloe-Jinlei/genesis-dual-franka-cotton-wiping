"""CPU regression tests for the conservative coupling operator, without a GPU scene.

Fake fields reproduce the relevant Genesis IO contract, including the position
setter clearing velocity. These tests complement the full native scene probe;
they do not assert that a NumPy stub establishes native solver stability.
"""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cloth_liquid_coupling import ClothLiquidCoupling, closest_points_on_triangles


class Field:
    def __init__(self, value):
        self.value = np.array(value)

    def to_numpy(self):
        return self.value.copy()

    def from_numpy(self, value):
        self.value = np.array(value)


class Entity:
    def __init__(self, positions, mass, faces=None, free=True):
        self.positions = np.array(positions, dtype=float)
        self.velocities = np.zeros_like(self.positions)
        self.active = np.ones(len(positions), dtype=bool)
        self.n_particles = len(positions)
        self.particle_start = 0
        self.particle_size = 0.003
        self.solver = SimpleNamespace(
            particles_info=SimpleNamespace(mass=Field(np.full(len(positions), mass, np.float32))),
            particles=SimpleNamespace(free=Field(np.full((len(positions), 1), free))))
        self.mesh = SimpleNamespace(faces=np.array(faces if faces is not None else [[0, 1, 2]], np.int64))

    def get_particles_pos(self):
        return self.positions.copy()

    def get_particles_vel(self):
        return self.velocities.copy()

    def get_particles_active(self):
        return self.active.copy()

    def set_particles_pos(self, positions):
        self.positions = positions.copy()
        self.velocities[:] = 0

    def set_particles_vel(self, velocities):
        self.velocities = velocities.copy()

    def set_particles_active(self, active):
        self.active = active.copy()


def make_pair(n=20, free=False, **settings):
    cloth = Entity([[0, 0, 0], [0.01, 0, 0], [0, 0.01, 0]], 1e-4, [[0, 1, 2]], free=free)
    liquid = Entity(np.tile([0.003, 0.003, 0.002], (n, 1)), 1e-6)
    coupling = ClothLiquidCoupling(SimpleNamespace(n_envs=0), cloth, liquid,
        {"absorption_flux_kg_m2_s": 1.0, "minimum_contact_time_s": 0.035, **settings})
    return cloth, liquid, coupling


def advance(coupling, steps):
    for _ in range(steps):
        coupling.step(0.004)


class CouplingModelTests(unittest.TestCase):
    def test_exact_triangle_projection_and_degenerate_edge(self):
        triangles = np.array([[[0, 0, 0], [1, 0, 0], [0, 1, 0]]]*4)
        points = np.array([[.2, .3, 2], [2, 0, 0], [-1, -1, 0], [.7, .7, 0]])
        closest, weights, _ = closest_points_on_triangles(points, triangles)
        np.testing.assert_allclose(closest, [[.2, .3, 0], [1, 0, 0], [0, 0, 0], [.5, .5, 0]])
        np.testing.assert_allclose(weights.sum(axis=1), 1)
        closest, weights, distance = closest_points_on_triangles([[.3, 1, 0]], [[[0, 0, 0], [1, 0, 0], [2, 0, 0]]])
        np.testing.assert_allclose(closest, [[.3, 0, 0]])
        np.testing.assert_allclose(distance, [1])

    def test_no_remote_capture_or_dry_flux_banking(self):
        cloth, liquid, coupling = make_pair()
        liquid.positions[:, 2] = .1
        advance(coupling, 100)
        self.assertEqual(coupling.metrics()["absorbed_particle_count"], 0)
        self.assertFalse(coupling._flux_credit.any())
        np.testing.assert_allclose(coupling._read_mass(cloth, coupling._cloth_slice), coupling.dry_vertex_mass)

    def test_interrupted_contact_resets_dwell(self):
        _, liquid, coupling = make_pair(absorption_flux_kg_m2_s=100)
        advance(coupling, 5)
        self.assertEqual(coupling.metrics()["absorbed_particle_count"], 0)
        liquid.positions[:, 2] = .1
        advance(coupling, 1)
        self.assertFalse(coupling._dwell.any())
        liquid.positions[:, 2] = .002
        advance(coupling, 5)
        self.assertEqual(coupling.metrics()["absorbed_particle_count"], 0)

    def test_absorption_has_finite_flux_and_conserves_mass(self):
        _, liquid, coupling = make_pair(n=100, minimum_contact_time_s=0)
        advance(coupling, 1)
        self.assertEqual(coupling.metrics()["absorbed_particle_count"], 0)
        advance(coupling, 99)
        result = coupling.metrics()
        self.assertGreater(result["absorbed_particle_count"], 0)
        self.assertAlmostEqual(result["mass_balance_error_kg"], 0, delta=1e-14)
        self.assertAlmostEqual(result["cloth_loading_error_kg"], 0, delta=1e-9)
        self.assertAlmostEqual(result["vertex_reservoir_error_kg"], 0, delta=1e-14)
        self.assertEqual(np.count_nonzero(~liquid.active), len(coupling.bound_particle_indices))
        self.assertEqual(len(np.unique(coupling.bound_particle_indices)), len(coupling.bound_particle_indices))
        bound_rate = coupling.config["absorption_flux_kg_m2_s"]*coupling.vertex_reference_area.sum()*.4
        self.assertLessEqual(result["absorbed_mass_kg"], bound_rate+1e-14)

    def test_bound_parcels_follow_deforming_triangles(self):
        cloth, liquid, coupling = make_pair()
        advance(coupling, 100)
        self.assertGreater(len(coupling.bound_particle_indices), 0)
        # Tilt and stretch the sheet; attachment must preserve material triangle
        # weights, not a horizontal rectangle or world-space pickup location.
        cloth.positions = np.array([[1, 2, 3], [1.02, 2, 3.01], [1, 2.02, 3.03]])
        advance(coupling, 1)
        ids = coupling.bound_particle_indices
        expected = coupling._bound_weights[ids]@cloth.positions
        np.testing.assert_allclose(coupling.bound_positions, expected)
        np.testing.assert_allclose(liquid.positions[ids], expected, atol=2e-7)

    def test_local_capacity_leaves_unabsorbed_liquid(self):
        _, _, coupling = make_pair(n=100, capacity_kg_m2=.1)
        advance(coupling, 500)
        self.assertGreater(coupling.metrics()["active_liquid_count"], 0)
        self.assertTrue(np.all(coupling.vertex_absorbed_mass <= coupling.vertex_capacity+1e-14))
        self.assertLessEqual(coupling.metrics()["maximum_saturation"], 1+1e-12)

    def test_free_cloth_operator_conserves_linear_momentum(self):
        _, liquid, coupling = make_pair(n=4, free=True, absorption_flux_kg_m2_s=50)
        liquid.velocities[:] = [1, 2, 3]
        advance(coupling, 20)
        result = coupling.metrics()
        self.assertGreater(result["absorbed_particle_count"], 0)
        self.assertLess(result["maximum_operator_momentum_residual_norm_kg_m_s"], 1e-12)
        np.testing.assert_allclose(result["support_impulse_on_system_kg_m_s"], 0)

    def test_fixed_support_reaction_and_zero_capacity_drag(self):
        cloth, liquid, coupling = make_pair(n=5, capacity_kg_m2=0)
        liquid.velocities[:] = [1, 2, 0]
        initial = (coupling.liquid_particle_mass[:, None]*liquid.velocities).sum(axis=0)
        advance(coupling, 40)
        result = coupling.metrics()
        self.assertEqual(result["absorbed_particle_count"], 0)
        self.assertLess(np.linalg.norm(liquid.velocities[0]), np.sqrt(5))
        np.testing.assert_allclose(cloth.velocities, 0)
        final = (coupling.liquid_particle_mass[:, None]*liquid.velocities).sum(axis=0)
        np.testing.assert_allclose(final-initial, result["support_impulse_on_system_kg_m_s"], atol=1e-12)

    def test_reset_and_matching_checkpoint_restore(self):
        cloth, liquid, coupling = make_pair()
        advance(coupling, 100)
        checkpoint = coupling.capture_state()
        count = len(coupling.bound_particle_indices)
        self.assertGreater(count, 0)
        coupling.reset()
        self.assertTrue(liquid.active.all())
        self.assertEqual(coupling.metrics()["absorbed_mass_kg"], 0)
        self.assertFalse(coupling._dwell.any())
        np.testing.assert_allclose(coupling._read_mass(cloth, coupling._cloth_slice), coupling.dry_vertex_mass)
        coupling.restore_state(checkpoint)
        self.assertEqual(len(coupling.bound_particle_indices), count)
        self.assertAlmostEqual(coupling.metrics()["mass_balance_error_kg"], 0, delta=1e-14)

    def test_inside_bounding_box_but_outside_triangle_cannot_absorb(self):
        _, liquid, coupling = make_pair(absorption_flux_kg_m2_s=100)
        liquid.positions[:] = [.009, .009, .002]
        advance(coupling, 100)
        self.assertEqual(coupling.metrics()["last_contact_count"], 0)
        self.assertEqual(coupling.metrics()["absorbed_particle_count"], 0)

    def test_mass_write_preserves_other_pbd_entities(self):
        cloth, liquid, _ = make_pair()
        cloth.particle_start = 1
        cloth.solver.particles_info.mass = Field(np.array([17, 1e-4, 1e-4, 1e-4, 23], np.float32))
        cloth.solver.particles.free = Field(np.zeros((5, 1), bool))
        coupling = ClothLiquidCoupling(SimpleNamespace(n_envs=0), cloth, liquid, {})
        advance(coupling, 100)
        self.assertGreater(coupling.metrics()["absorbed_particle_count"], 0)
        native = cloth.solver.particles_info.mass.to_numpy()
        np.testing.assert_array_equal(native[[0, 4]], [17, 23])
        coupling.reset()
        np.testing.assert_allclose(cloth.solver.particles_info.mass.to_numpy(), [17, 1e-4, 1e-4, 1e-4, 23])

    def test_invalid_configuration_and_batched_scene_rejected(self):
        cloth, liquid, _ = make_pair()
        with self.assertRaises(ValueError):
            ClothLiquidCoupling(SimpleNamespace(n_envs=2), cloth, liquid, {})
        with self.assertRaises(ValueError):
            make_pair(capacity_kg_m2=-1)
        with self.assertRaises(ValueError):
            make_pair(collision_distance_m=.01)


if __name__ == "__main__":
    unittest.main()
