"""CPU mesh/material mathematics and real import checks; no solver build/GPU."""
import tempfile
import unittest
from pathlib import Path

import numpy as np

from cotton_bending import build_hinges, hinge_angle_gradient, disjoint_hinge_colors
from cotton_membrane import edge_constraints
from cotton_mesh import rectangular_cotton_mesh, existing_cotton_mesh, _validate_mesh


class MaterialGeometryTests(unittest.TestCase):
    def test_regular_grid_keeps_bulk_yarn_directions(self):
        points, faces = rectangular_cotton_mesh(center=(.1, -.23, .764))
        self.assertEqual(points.shape, (5795, 3))
        self.assertEqual(faces.shape, (11280, 3))
        np.testing.assert_allclose(np.ptp(points, axis=0), [.28, .18, 0], atol=1e-14)
        edges, rest, maximum, directions, bounds = edge_constraints(points, faces)
        np.testing.assert_array_equal(np.bincount(directions), [5734, 5700, 5640])
        np.testing.assert_allclose(maximum[directions < 2]/rest[directions < 2], 1.02)
        np.testing.assert_allclose(maximum[directions == 2]/rest[directions == 2], 1.10)
        for begin, end in zip(bounds[:-1], bounds[1:]):
            self.assertEqual(len(np.unique(edges[begin:end])), 2*(end-begin))
        self.assertLessEqual(rest[directions < 2].max(), .003+1e-14)

    def test_mass_area_and_alternating_diagonals(self):
        points, faces = rectangular_cotton_mesh(size=(.012, .006), spacing=.003)
        _, _, _, area, density = _validate_mesh(points, faces, .18)
        self.assertAlmostEqual(area, .012*.006, places=15)
        masses = np.full(len(points), area*density/len(points))
        self.assertAlmostEqual(masses.sum(), .012*.006*.18, places=15)
        edges, _, _, directions, _ = edge_constraints(points, faces)
        delta = points[edges[:, 1]]-points[edges[:, 0]]
        slopes = np.sign(delta[directions == 2, 0]*delta[directions == 2, 1])
        self.assertEqual(set(slopes), {-1., 1.})

    def test_bending_hinges_have_geometry_scaled_rigidity(self):
        points, faces = rectangular_cotton_mesh(size=(.012, .006), spacing=.003)
        ids, rest, compliance = build_hinges(points, faces, 5e-5)
        self.assertGreater(len(ids), 0)
        np.testing.assert_allclose(rest, 0, atol=1e-14)
        scaled = build_hinges(points*3, faces, 5e-5)[2]
        np.testing.assert_allclose(compliance, scaled, rtol=1e-12)
        np.testing.assert_allclose(build_hinges(points, faces, 1e-4)[2], compliance/2)

    def test_bending_gradient_and_rigid_motion_invariants(self):
        points = np.array([[0., 0., 0.], [.01, 0., 0.], [.003, .01, .002], [.007, -.01, -.004]])
        angle, gradient = hinge_angle_gradient(points)
        numerical = np.empty((4, 3))
        eps = 1e-7
        for i in range(4):
            for j in range(3):
                plus, minus = points.copy(), points.copy()
                plus[i, j] += eps
                minus[i, j] -= eps
                delta = hinge_angle_gradient(plus)[0]-hinge_angle_gradient(minus)[0]
                numerical[i, j] = np.arctan2(np.sin(delta), np.cos(delta))/(2*eps)
        np.testing.assert_allclose(gradient, numerical, atol=1e-6, rtol=1e-7)
        np.testing.assert_allclose(gradient.sum(0), 0, atol=1e-12)
        np.testing.assert_allclose(np.cross(points, gradient).sum(0), 0, atol=1e-12)
        self.assertTrue(np.isfinite(angle))

    def test_bending_color_partition_is_disjoint_and_complete(self):
        points, faces = rectangular_cotton_mesh()
        hinges, rest, compliance = build_hinges(points, faces, 5e-5)
        order, bounds = disjoint_hinge_colors(hinges, len(points))
        np.testing.assert_array_equal(np.sort(order), np.arange(len(hinges)))
        self.assertEqual(bounds[0], 0)
        self.assertEqual(bounds[-1], len(hinges))
        self.assertEqual(len(bounds)-1, 17)
        for begin, end in zip(bounds[:-1], bounds[1:]):
            self.assertLess(begin, end)
            self.assertEqual(len(np.unique(hinges[order[begin:end]])), 4*(end-begin))
        # Sorting the topology and its constitutive data together is lossless.
        inverse = np.argsort(order)
        np.testing.assert_array_equal(hinges[order][inverse], hinges)
        np.testing.assert_array_equal(rest[order][inverse], rest)
        np.testing.assert_array_equal(compliance[order][inverse], compliance)

    def test_bending_colors_reject_degenerate_or_foreign_nodes(self):
        for hinges in (np.array([[0, 1, 2, 2]]), np.array([[0, 1, 2, 4]]),
                       np.array([[0., 1., 2., 3.]]), np.array([[0, 1, 2]])):
            with self.assertRaises(ValueError):
                disjoint_hinge_colors(hinges, 4)
        order, bounds = disjoint_hinge_colors(np.empty((0, 4), dtype=int), 0)
        self.assertEqual(len(order), 0)
        self.assertEqual(bounds, [0])

    def test_hinge_mass_weighting_preserves_free_momentum_and_fixed_nodes(self):
        points = np.array([[0., 0., 0.], [.01, 0., 0.],
                           [.003, .01, .002], [.007, -.01, -.004]])
        angle, gradient = hinge_angle_gradient(points)
        # Nonuniform masses represent wet cloth. Translation and angular
        # impulse balance must survive inverse-mass weighting.
        mass = np.array([1., 2., 4., 8.])*1e-6
        inverse_mass = 1./mass
        alpha = (1./1.5e-4)/(.0001**2)
        delta_lambda = -.5*angle/(np.sum(inverse_mass[:, None]*gradient**2)+alpha)
        correction = inverse_mass[:, None]*delta_lambda*gradient
        np.testing.assert_allclose((mass[:, None]*correction).sum(0), 0, atol=1e-22)
        np.testing.assert_allclose(np.cross(points, mass[:, None]*correction).sum(0), 0, atol=1e-23)
        before = .5*angle**2/(1./1.5e-4)
        after = .5*hinge_angle_gradient(points+correction)[0]**2/(1./1.5e-4)
        self.assertLess(after, before)
        inverse_mass[[0, 2]] = 0
        delta_lambda = -.5*angle/(np.sum(inverse_mass[:, None]*gradient**2)+alpha)
        correction = inverse_mass[:, None]*delta_lambda*gradient
        np.testing.assert_array_equal(correction[[0, 2]], 0)
        self.assertLess(abs(hinge_angle_gradient(points+correction)[0]), abs(angle))

    def test_invalid_mesh_inputs_fail_early(self):
        for spacing in (0, -1, np.nan, True):
            with self.assertRaises(ValueError):
                rectangular_cotton_mesh(spacing=spacing)
        points, faces = rectangular_cotton_mesh(size=(.012, .006), spacing=.003)
        reversed_face = faces.copy()
        reversed_face[0] = reversed_face[0, ::-1]
        with self.assertRaises(ValueError):
            _validate_mesh(points, reversed_face, .18)


class NativeClothImportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import genesis as gs
        cls.gs = gs
        gs.init(backend=gs.cpu, logging_level="warning", precision="32", seed=7)
        cls.temp = tempfile.TemporaryDirectory(prefix="cotton-mesh-cpu-import-")

    @classmethod
    def tearDownClass(cls):
        cls.gs.destroy()
        cls.temp.cleanup()

    def make_scene(self):
        return self.gs.Scene(show_viewer=False,
            sim_options=self.gs.options.SimOptions(dt=.002, substeps=20),
            pbd_options=self.gs.options.PBDOptions(particle_size=.003))

    def add_cloth(self, scene, world_points, faces, visual_points, morph_pos):
        import trimesh
        path = Path(self.temp.name)/f"cloth-{len(list(Path(self.temp.name).iterdir()))}.obj"
        trimesh.Trimesh(vertices=visual_points,
            faces=np.array([[0, 1, 2], [0, 2, 3]]), process=False).export(path)
        with existing_cotton_mesh(world_points, faces, .18):
            return scene.add_entity(material=self.gs.materials.PBD.Cloth(rho=.18),
                morph=self.gs.morphs.Mesh(file=str(path), pos=morph_pos,
                    file_meshes_are_zup=True, decimate=False, convexify=False),
                surface=self.gs.surfaces.Default())

    def test_translation_visual_once_physical_never_retranslated(self):
        import genesis.engine.solvers.pbd_solver as solver_module
        original = solver_module.PBD2DEntity.sample
        center = np.array([.12, -.23, .764])
        points, faces = rectangular_cotton_mesh(size=(.012, .006), center=center)
        corners = np.array([[-.006, -.003, 0], [.006, -.003, 0], [.006, .003, 0], [-.006, .003, 0]])
        scene = self.make_scene()
        cloth = self.add_cloth(scene, points, faces, corners, center)
        np.testing.assert_allclose(cloth._particles, points, atol=5e-8, rtol=0)
        np.testing.assert_allclose(cloth._vverts, corners+center, atol=5e-8, rtol=0)
        np.testing.assert_allclose(cloth._init_particles_offset.cpu().numpy(), points-center, atol=5e-8, rtol=0)
        np.testing.assert_array_equal(cloth.mesh.faces, faces)
        self.assertAlmostEqual(cloth._particle_mass*cloth.n_particles, .012*.006*.18, places=15)
        self.assertIs(solver_module.PBD2DEntity.sample, original)
        self.assertFalse(scene.is_built)

    def test_identity_world_visual_supported(self):
        center = np.array([.12, -.23, .764])
        points, faces = rectangular_cotton_mesh(size=(.012, .006), center=center)
        corners = np.array([[-.006, -.003, 0], [.006, -.003, 0], [.006, .003, 0], [-.006, .003, 0]])+center
        scene = self.make_scene()
        cloth = self.add_cloth(scene, points, faces, corners, (0., 0., 0.))
        np.testing.assert_allclose(cloth._particles, points, atol=5e-8, rtol=0)
        np.testing.assert_allclose(cloth._vverts, corners, atol=5e-8, rtol=0)

    def test_material_hooks_install_and_restore_without_build(self):
        from cotton_membrane import install_cotton_membrane
        from cotton_bending import install_cotton_bending
        points, faces = rectangular_cotton_mesh(size=(.012, .006))
        corners = np.array([[-.006, -.003, 0], [.006, -.003, 0], [.006, .003, 0], [-.006, .003, 0]])
        scene = self.make_scene()
        cloth = self.add_cloth(scene, points, faces, corners, (0., 0., 0.))
        original_stretch, original_bending = cloth.solver._kernel_solve_stretch, cloth.solver._kernel_solve_bending
        membrane = install_cotton_membrane(scene, cloth, points, faces)
        bending = install_cotton_bending(scene, cloth, points, faces)
        self.assertEqual(membrane.description["sweeps_per_native_stretch_call"], 24)
        self.assertEqual(bending.description["bending_rigidity_N_m"], 5e-5)
        self.assertEqual(bending.description["interior_hinges"], cloth.n_inner_edges)
        self.assertIn("Gauss-Seidel", bending.description["solver"])
        self.assertGreater(bending.description["colors"], 1)
        self.assertEqual(membrane.description["edge_counts"], {"warp": 12, "weft": 10, "bias": 8})
        with self.assertRaisesRegex(RuntimeError, "initialized PBD particle fields"):
            bending.reset()
        bending.close()
        membrane.close()
        self.assertEqual(cloth.solver._kernel_solve_stretch, original_stretch)
        self.assertEqual(cloth.solver._kernel_solve_bending, original_bending)
        self.assertFalse(scene.is_built)

    def test_double_translation_rejected_and_hook_restored(self):
        import genesis.engine.solvers.pbd_solver as solver_module
        original = solver_module.PBD2DEntity.sample
        center = np.array([.12, -.23, .764])
        points, faces = rectangular_cotton_mesh(size=(.012, .006), center=center)
        corners = np.array([[-.006, -.003, 0], [.006, -.003, 0], [.006, .003, 0], [-.006, .003, 0]])+center
        with self.assertRaisesRegex(Exception, "different bounds"):
            self.add_cloth(self.make_scene(), points, faces, corners, center)
        self.assertIs(solver_module.PBD2DEntity.sample, original)


if __name__ == "__main__":
    unittest.main()
