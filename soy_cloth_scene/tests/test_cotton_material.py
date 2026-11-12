"""CPU checks of cotton mesh weights, damping and build-warmup integration.

The small field/decorator stubs execute the adapter's actual Python kernels;
they do not compile Quadrants or validate native GPU scheduling/contact.
"""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cotton_material import _prepare_cloth_material_data, install_cotton_material


class Vector(np.ndarray):
    def __new__(cls, values):
        return np.asarray(values, dtype=float).view(cls)

    def norm(self):
        return np.linalg.norm(self)


class Field:
    def __init__(self, rows):
        self.rows = rows

    def __getitem__(self, index):
        return self.rows[index[0] if isinstance(index, tuple) else index]

    def __getattr__(self, name):
        return SimpleNamespace(to_numpy=lambda: np.array([getattr(row, name) for row in self.rows]))


class ClothMaterial:
    rho = 0.18
    stretch_compliance = 1e-7
    stretch_relaxation = 0.3


class CottonMaterialTests(unittest.TestCase):
    def setUp(self):
        self.points = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]])
        self.faces = np.array([[0, 1, 2]], dtype=np.int32)
        self.events = []
        # Sentinel particles outside the cloth slice catch unwanted writes.
        rows = [SimpleNamespace(pos=Vector([9, 9, 9]), vel=Vector([4, 5, 6]), free=True)]
        rows += [SimpleNamespace(pos=Vector(p), vel=Vector([0, 0, 0]), free=True) for p in self.points]
        rows += [SimpleNamespace(pos=Vector([8, 8, 8]), vel=Vector([7, 8, 9]), free=True)]
        info = [SimpleNamespace(mass=float(i + 1), air_resistance=-7.) for i in range(5)]
        self.solver = SimpleNamespace(
            particles=Field(rows), particles_info=Field(info), _B=1,
            _max_stretch_solver_iterations=10, _substep_dt=0.02,
            edges_info=Field([SimpleNamespace(stretch_compliance=1e-7, stretch_relaxation=.3)
                              for _ in range(3)]),
            _kernel_store_initial_pos=lambda f: self.events.append(("native_store", f)),
        )
        self.cloth = SimpleNamespace(
            solver=self.solver, material=ClothMaterial(), n_particles=3,
            particle_start=1, _edge_start=0, n_edges=3,
            mesh=SimpleNamespace(verts=self.points, faces=self.faces),
            edges=np.array([[0, 1], [0, 2], [1, 2]]),
        )
        self.solver.entities = [self.cloth]
        self.scene = SimpleNamespace(is_built=False, pbd_solver=self.solver)
        self.membrane_calls, self.bending_calls = [], []
        self.membrane = SimpleNamespace(description={"model": "test directional membrane"})
        self.bending = SimpleNamespace(description={"model": "test dihedral bending"},
                                       reset=lambda: self.events.append(("reset_bending",)))

        def membrane(*args, **kwargs):
            self.membrane_calls.append((args, kwargs))
            return self.membrane

        def bending(*args, **kwargs):
            self.bending_calls.append((args, kwargs))
            return self.bending

        fake_gs = SimpleNamespace(device="cpu", tc_int=np.int32, tc_float=np.float64,
                                  materials=SimpleNamespace(PBD=SimpleNamespace(Cloth=ClothMaterial)))
        fake_qd = SimpleNamespace(data_oriented=lambda cls: cls, kernel=lambda f: f,
                                  static=lambda value: value, exp=np.exp,
                                  types=SimpleNamespace(ndarray=lambda: object))
        fake_torch = SimpleNamespace(as_tensor=lambda array, device, dtype: np.asarray(array, dtype=dtype))
        self.modules = patch.dict(sys.modules, {
            "genesis": fake_gs, "quadrants": fake_qd, "torch": fake_torch,
            "cotton_membrane": SimpleNamespace(install_cotton_membrane=membrane),
            "cotton_bending": SimpleNamespace(install_cotton_bending=bending),
        })
        self.modules.start()
        self.addCleanup(self.modules.stop)
        version_patch = patch("cotton_material.version", return_value="1.4.3")
        version_patch.start()
        self.addCleanup(version_patch.stop)

    def install(self, **config):
        return install_cotton_material(self.scene, self.cloth, config)

    def test_residual_damping_replaces_edge_damping_during_native_warmup(self):
        residual = SimpleNamespace(description={"model": "rigid motion residual"},
                                   apply=lambda: self.events.append(("damp_deformation",)))
        calls = []
        def install(scene, cloth, *, rate_per_s):
            calls.append((scene, cloth, rate_per_s))
            return residual
        with patch.dict(sys.modules, {"cotton_deformation_damping": SimpleNamespace(
                install_cotton_deformation_damping=install)}):
            adapter = self.install(internal_damping_model="rigid_motion_residual")
        adapter.damp_edges = lambda *a: self.fail("Residual mode must replace axial damping")
        self.scene.is_built = True
        self.solver._kernel_store_initial_pos(0)
        self.assertEqual(self.events, [("native_store", 0), ("reset_bending",),
                                      ("damp_deformation",)])
        self.assertEqual(calls, [(self.scene, self.cloth, 4.)])
        self.assertEqual(adapter.description["internal_damping"], residual.description)

    def test_unknown_internal_damping_model_rejected(self):
        with self.assertRaisesRegex(ValueError, "internal_damping_model"):
            self.install(internal_damping_model="typo")

    def test_barycentric_area_preserves_total_and_edge_colors_are_disjoint(self):
        points = np.array([[0., 0., 0.], [2., 0., 0.], [0., 1., 0.], [0., -2., 0.]])
        data = _prepare_cloth_material_data(points, np.array([[0, 1, 2], [1, 0, 3]]), {})
        np.testing.assert_allclose(data["particle_area"], [1, 1, 1/3, 2/3])
        self.assertAlmostEqual(data["air_coefficients"].sum(), .5 * 1.225 * 3.)
        self.assertEqual(data["maximum_degree"], 3)
        np.testing.assert_allclose(data["edge_rates"], 4/3)
        for lo, hi in zip(data["color_bounds"][:-1], data["color_bounds"][1:]):
            endpoints = data["edges"][lo:hi].ravel()
            self.assertEqual(len(endpoints), len(np.unique(endpoints)))
        self.assertEqual(len(data["edges"]), 5)

    def test_air_coefficient_sum_does_not_grow_when_mesh_is_refined(self):
        coarse = _prepare_cloth_material_data(self.points, self.faces, {})
        refined_points = np.concatenate([self.points, [[.5, 0, 0], [.5, .5, 0], [0, .5, 0]]])
        refined_faces = np.array([[0, 3, 5], [3, 1, 4], [5, 4, 2], [3, 4, 5]])
        fine = _prepare_cloth_material_data(refined_points, refined_faces, {})
        self.assertAlmostEqual(coarse["air_coefficients"].sum(), fine["air_coefficients"].sum())
        self.assertLess(fine["air_coefficients"].max(), coarse["air_coefficients"].max())

    def test_warmup_configures_air_and_resets_bending_at_every_substep(self):
        adapter = self.install()
        with self.assertRaises(RuntimeError):
            adapter.configure()
        before_mass = self.solver.particles_info.mass.to_numpy().copy()
        self.scene.is_built = True
        self.solver._kernel_store_initial_pos(0)
        self.solver._kernel_store_initial_pos(1)
        self.assertEqual(self.events, [("native_store", 0), ("reset_bending",),
                                       ("native_store", 1), ("reset_bending",)])
        np.testing.assert_array_equal(before_mass, self.solver.particles_info.mass.to_numpy())
        coefficients = self.solver.particles_info.air_resistance.to_numpy()
        np.testing.assert_allclose(coefficients[1:4], .5 * 1.225 * .5 / 3)
        np.testing.assert_array_equal(coefficients[[0, 4]], [-7., -7.])
        self.assertTrue(adapter.configured)
        self.assertIs(adapter.membrane, self.membrane)
        self.assertIs(adapter.bending, self.bending)
        self.assertEqual(self.membrane_calls[0][1], {
            "warp_limit": .02, "weft_limit": .02, "bias_limit": .10, "iterations": 24})
        self.assertEqual(self.bending_calls[0][1], {"rigidity": 5e-5, "relaxation": .5})
        json.dumps(adapter.description, allow_nan=False)
        adapter.configure()
        self.assertEqual(len(self.events), 4)  # Post-build configure is idempotent.

    def test_internal_damping_reads_changed_wet_mass_and_preserves_momentum(self):
        adapter = self.install(linear_drag_per_s=0.)
        # Change native mass after installation: an initially dry cache is wrong.
        self.solver.particles_info[2].mass = 17.
        for particle, velocity in zip(self.solver.particles.rows[1:4], [[2, 0, 1], [-1, 3, 1], [0, -2, 1]]):
            particle.vel = Vector(velocity)
        masses = self.solver.particles_info.mass.to_numpy()[1:4, None]
        before = self.solver.particles.vel.to_numpy()[1:4].copy()
        adapter.damp_edges(adapter.edges, adapter.rates)
        after = self.solver.particles.vel.to_numpy()[1:4]
        np.testing.assert_allclose((masses * before).sum(axis=0), (masses * after).sum(axis=0), atol=1e-13)
        self.assertLess(float((masses * after**2).sum()), float((masses * before**2).sum()))
        np.testing.assert_array_equal(after[:, 2], before[:, 2])

    def test_internal_damping_preserves_rigid_translation_and_rotation(self):
        adapter = self.install()
        for particle in self.solver.particles.rows[1:4]:
            particle.vel = Vector([2, -3, 1]) + np.cross([0, 0, 7], particle.pos)
        before = self.solver.particles.vel.to_numpy().copy()
        adapter.damp_edges(adapter.edges, adapter.rates)
        np.testing.assert_allclose(self.solver.particles.vel.to_numpy(), before, atol=1e-14)

    def test_linear_drag_is_cloth_only_and_attached_velocities_are_unchanged(self):
        adapter = self.install()
        for particle in self.solver.particles.rows:
            particle.vel = Vector([2, -3, 4])
        self.solver.particles[2, 0].free = False
        before = self.solver.particles.vel.to_numpy().copy()
        adapter.apply_linear_drag()
        after = self.solver.particles.vel.to_numpy()
        np.testing.assert_allclose(after[[1, 3]], before[[1, 3]] * np.exp(-.02))
        np.testing.assert_array_equal(after[[0, 2, 4]], before[[0, 2, 4]])
        adapter.damp_edges(adapter.edges, adapter.rates)
        np.testing.assert_array_equal(self.solver.particles.vel.to_numpy()[[0, 2, 4]], before[[0, 2, 4]])

    def test_invalid_settings_or_mesh_rejected_before_installing_hooks(self):
        for config in ({"internal_damping_per_s": -1}, {"linear_drag_per_s": float("nan")},
                       {"strain_limit_iterations": 1.5}, {"strain_limit_iterations": True},
                       {"bending_relaxation": 1.01}):
            with self.subTest(config=config), self.assertRaises(ValueError):
                self.install(**config)
        self.cloth.edges = np.array([[0, 1]])
        with self.assertRaises(ValueError):
            self.install()
        self.assertFalse(self.membrane_calls)
        self.assertFalse(self.bending_calls)


class CottonBendingScheduleTests(unittest.TestCase):
    def test_four_colored_sweeps_reduce_positive_rigidity_checkerboard_energy(self):
        # This is the observed regression: the former simultaneous hinge sum
        # amplifies a 1 um grid-scale wrinkle even without contact or gravity.
        # Use production topology/coloring with identical positive B and .5
        # relaxation, changing only the constraint schedule in the CPU probe.
        from probe_pdf_bending_stability import run_diagnostic
        report = run_diagnostic()
        self.assertEqual(report["input"]["bending_rigidity_N_m"], 5e-5)
        self.assertEqual(report["input"]["sweeps"], 4)
        self.assertTrue(report["disjoint_hinge_colors"]["all_color_nodes_disjoint"])
        self.assertGreater(report["previous_simultaneous_jacobi"]["bending_energy_ratio"], 1.)
        self.assertLess(report["disjoint_hinge_colors"]["bending_energy_ratio"], 1.)
        self.assertTrue(all(report["checks"].values()))


if __name__ == "__main__":
    unittest.main()
