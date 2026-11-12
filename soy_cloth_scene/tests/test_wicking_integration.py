"""Verify pore transport is reflected in native IO and checkpoint ledgers."""
import unittest
import numpy as np
from test_coupling_model import make_pair, advance


class WickingIntegrationTests(unittest.TestCase):
    def test_transport_updates_native_wet_mass_without_remote_capture(self):
        cloth, liquid, coupling = make_pair(n=12, free=True,
            wicking={"enabled": True, "diffusivity_m2_s": 1e-6})
        liquid.positions[-1, 2] = .1
        cloth.velocities[:] = [[.02, .01, 0], [-.01, .02, 0], [0, -.01, 0]]
        advance(coupling, 200)
        metrics = coupling.metrics()
        self.assertTrue(liquid.active[-1])
        self.assertEqual(metrics["absorbed_particle_count"], 11)
        self.assertGreater(metrics["wicking_parcel_moves"], 0)
        self.assertLess(metrics["maximum_operator_momentum_residual_norm_kg_m_s"], 1e-10)
        self.assertAlmostEqual(metrics["mass_balance_error_kg"], 0, delta=1e-14)
        self.assertAlmostEqual(metrics["vertex_reservoir_error_kg"], 0, delta=1e-14)
        np.testing.assert_allclose(coupling._read_mass(cloth, coupling._cloth_slice),
            coupling.dry_vertex_mass+coupling.vertex_absorbed_mass, atol=1e-10, rtol=1e-6)
        np.testing.assert_allclose(liquid.positions[coupling.bound_particle_indices],
                                   coupling.bound_positions, atol=1e-8)
        self.assertTrue(np.all(coupling.vertex_absorbed_mass <= coupling.vertex_capacity+1e-14))

    def test_reset_and_restore_transport_state(self):
        _, _, coupling = make_pair(n=12, free=True, wicking={"enabled": True})
        advance(coupling, 150)
        state = coupling.capture_state()
        before = coupling.metrics()
        self.assertGreater(before["wicking_parcel_moves"], 0)
        coupling.reset()
        self.assertEqual(coupling.metrics()["wicking_parcel_moves"], 0)
        coupling.restore_state(state)
        self.assertEqual(coupling.metrics()["wicking_parcel_moves"], before["wicking_parcel_moves"])
        np.testing.assert_array_equal(coupling.vertex_absorbed_mass, state["vertex_absorbed_mass"])
        np.testing.assert_array_equal(coupling._bound_weights, state["_bound_weights"])
        advance(coupling, 1)
        self.assertLess(abs(coupling.metrics()["cloth_loading_error_kg"]), 1e-9)


if __name__ == "__main__":
    unittest.main()
