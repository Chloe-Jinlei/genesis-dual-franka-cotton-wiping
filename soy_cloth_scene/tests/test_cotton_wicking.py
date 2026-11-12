"""CPU conservation/locality regressions for optional porous-sheet transport."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cotton_wicking import CottonWicking


class WickingTests(unittest.TestCase):
    def make(self, *, enabled=True, config=None, faces=None, vertices=None,
             weights=None, masses=None, capacity=None):
        vertices = np.array([[0, 0, 0], [.001, 0, 0], [0, .001, 0], [.001, .001, 0]], dtype=float) if vertices is None else np.asarray(vertices, float)
        faces = np.array([[0, 1, 2], [1, 3, 2]], np.int64) if faces is None else np.asarray(faces, np.int64)
        weights = np.array([[1/3, 1/3, 1/3]]*3, dtype=float) if weights is None else np.asarray(weights, float)
        masses = np.ones(len(weights), dtype=float) if masses is None else np.asarray(masses, float)
        capacity = np.ones(len(vertices)) if capacity is None else np.asarray(capacity, float)
        model = CottonWicking(vertices, faces, masses, capacity, {"enabled": enabled, **(config or {})})
        bf = np.zeros(len(weights), np.int64)
        wet = np.zeros(len(vertices))
        np.add.at(wet, faces[bf].ravel(), (masses[:, None]*weights).ravel())
        dry = np.ones(len(vertices))
        vel = np.arange(len(vertices)*3, dtype=float).reshape(-1, 3)/10
        free = np.ones(len(vertices), bool)
        state = [bf, weights, wet, dry, vel, free]
        return model, state

    def check_ledger(self, model, state):
        bf, bw, wet, dry, vel, free = state
        reconstructed = np.zeros(len(wet))
        bound = np.flatnonzero(bf >= 0)
        np.add.at(reconstructed, model.faces[bf[bound]].ravel(),
                  (model.parcel_mass[bound, None]*bw[bound]).ravel())
        np.testing.assert_allclose(wet, reconstructed, atol=1e-12)
        self.assertAlmostEqual(wet.sum(), model.parcel_mass[bound].sum(), places=12)
        self.assertTrue(np.all(wet >= -1e-14))
        self.assertTrue(np.all(wet <= model.capacity+1e-14))
        self.assertTrue(np.all(bw[bound] >= -1e-12))
        np.testing.assert_allclose(bw[bound].sum(1), 1, atol=1e-12)

    def test_disabled_is_identity(self):
        model, state = self.make(enabled=False)
        before = [a.copy() for a in state]
        report = model.step(.01, *state)
        self.assertFalse(report["enabled"])
        for old, new in zip(before, state):
            np.testing.assert_array_equal(old, new)

    def test_flat_saturated_face_reaches_adjacent_dry_vertex(self):
        model, state = self.make()
        initial = state[2].copy()
        hops = 0
        for _ in range(200):
            report = model.step(.002, *state)
            hops += report["neighbor_hops"]
            self.check_ledger(model, state)
            self.assertLessEqual(report["saturation_energy_change_kg"], 1e-14)
            self.assertLess(np.linalg.norm(report["momentum_residual_kg_m_s"]), 1e-12)
            if hops:
                break
        self.assertGreater(hops, 0)
        self.assertGreater(state[2][3], 0)
        self.assertLess(state[2][0], initial[0])
        self.assertTrue(np.any(state[0] == 1))

    def test_finite_hop_rate_not_instantaneous_or_global(self):
        model, state = self.make()
        model.step(.002, *state)
        np.testing.assert_array_equal(state[0], 0)
        self.assertEqual(state[2][3], 0)
        self.assertTrue(np.any(model.capture_state()["hop_credit"] > 0))

    def test_no_transfer_to_spatially_coincident_disconnected_sheet(self):
        vertices = [[0, 0, 0], [.001, 0, 0], [0, .001, 0]]*2
        model, state = self.make(vertices=vertices, faces=[[0, 1, 2], [3, 4, 5]])
        before = [a.copy() for a in state]
        for _ in range(30):
            model.step(1, *state)
        np.testing.assert_array_equal(state[2][3:], 0)
        np.testing.assert_array_equal(state[0], before[0])
        self.check_ledger(model, state)

    def test_fixed_vertex_reaction_and_free_momentum_balance(self):
        model, state = self.make(weights=[[.65, .25, .1]], masses=[.4],
                                 config={"diffusivity_m2_s": 1e-5})
        state[-1][1] = False
        fixed_before = state[-2][1].copy()
        initial_momentum = ((state[3]+state[2])[:, None]*state[4]).sum(0)
        support = np.zeros(3)
        moves = 0
        for _ in range(10):
            report = model.step(.002, *state)
            support += report["support_impulse_on_system_kg_m_s"]
            moves += report["moved_parcels"]
            self.check_ledger(model, state)
        self.assertGreater(moves, 0)
        np.testing.assert_array_equal(state[-2][1], fixed_before)
        final_momentum = ((state[3]+state[2])[:, None]*state[4]).sum(0)
        np.testing.assert_allclose(final_momentum-initial_momentum, support, atol=1e-12)

    def test_extreme_dt_and_diffusivity_stay_local_and_within_capacity(self):
        model, state = self.make(weights=[[.98, .01, .01]], masses=[.9],
            config={"diffusivity_m2_s": 10., "maximum_material_step_m": .00005,
                    "maximum_neighbor_hop_m": .0001})
        old_face = state[0].copy()
        report = model.step(100., *state)
        self.assertLessEqual(report["maximum_material_distance_m"], .000100000001)
        self.assertGreater(report["step_limited_parcels"], 0)
        for before, after in zip(old_face, state[0]):
            self.assertTrue(after == before or after in model.neighbors[before])
        self.check_ledger(model, state)

    def test_credit_checkpoint_restores_identical_next_step_and_reset(self):
        model, state = self.make()
        for _ in range(30):
            model.step(.002, *state)
        saved = model.capture_state()
        arrays = [a.copy() for a in state]
        first = model.step(.002, *state)
        result = [a.copy() for a in state]
        model.restore_state(saved)
        for old, dest in zip(arrays, state):
            dest[:] = old
        second = model.step(.002, *state)
        self.assertEqual(first, second)
        for a, b in zip(result, state):
            np.testing.assert_array_equal(a, b)
        model.reset()
        np.testing.assert_array_equal(model.capture_state()["hop_credit"], 0)
        np.testing.assert_array_equal(model.capture_state()["hop_target"], -1)

    def test_rejects_inconsistent_or_overfull_ledger(self):
        model, state = self.make()
        state[2][3] = .1
        with self.assertRaisesRegex(ValueError, "ledgers disagree"):
            model.step(.002, *state)
        state[2][0] = 1.1
        with self.assertRaisesRegex(ValueError, "capacity"):
            model.step(.002, *state)


if __name__ == "__main__":
    unittest.main()
