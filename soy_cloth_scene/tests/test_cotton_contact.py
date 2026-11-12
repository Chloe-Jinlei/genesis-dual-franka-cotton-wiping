"""Thickness-adapter guards; native contact behaviour is tested in the scene.

These CPU tests deliberately stub Genesis: they verify configuration boundaries
and preservation, not native particle collision or kernel recompilation.
"""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cotton_contact import configure_thin_cloth_contact


class ClothMaterial:
    pass


class Solver:
    def __init__(self):
        self._particle_size = 0.006
        self.entities = []

    @property
    def particle_size(self):
        return self._particle_size

    def add_entity(self, entity):
        self.entities.append(entity)


class ThinContactTests(unittest.TestCase):
    def setUp(self):
        self.solver = Solver()
        self.cloth = SimpleNamespace(
            material=ClothMaterial(), solver=self.solver, particle_size=0.006,
            n_particles=1543, n_edges=4494, _mesh=object(), _mass=0.02016,
        )
        self.solver.entities.append(self.cloth)
        self.scene = SimpleNamespace(is_built=False, pbd_solver=self.solver)
        fake = SimpleNamespace(materials=SimpleNamespace(PBD=SimpleNamespace(Cloth=ClothMaterial)))
        self.module_patch = patch.dict(sys.modules, {"genesis": fake})
        self.module_patch.start()
        self.addCleanup(self.module_patch.stop)

    def test_radius_changes_without_resampling_or_changing_mass(self):
        before = dict(vars(self.cloth))
        report = configure_thin_cloth_contact(self.scene, self.cloth, 0.00075)
        self.assertEqual(self.solver.particle_size, 0.0015)
        self.assertEqual(self.cloth.collision_radius_m, 0.00075)
        for key, value in before.items():
            self.assertEqual(getattr(self.cloth, key), value)
        self.assertEqual(report["self_contact_distance_m"], 0.0015)
        self.assertEqual(report["mesh_spacing_m"], 0.006)

    def test_later_pbd_addition_cannot_resample_at_contact_diameter(self):
        configure_thin_cloth_contact(self.scene, self.cloth, 0.00075)
        with self.assertRaises(RuntimeError):
            self.solver.add_entity(object())
        self.assertEqual(len(self.solver.entities), 1)

    def test_configuration_after_build_rejected(self):
        self.scene.is_built = True
        with self.assertRaises(RuntimeError):
            configure_thin_cloth_contact(self.scene, self.cloth, 0.00075)
        self.assertEqual(self.solver.particle_size, 0.006)

    def test_multiple_pbd_entities_rejected(self):
        self.solver.entities.append(object())
        with self.assertRaises(ValueError):
            configure_thin_cloth_contact(self.scene, self.cloth, 0.00075)

    def test_invalid_or_larger_radius_rejected(self):
        for radius in (-1, 0, float("nan"), float("inf"), 0.0031):
            with self.subTest(radius=radius), self.assertRaises(ValueError):
                configure_thin_cloth_contact(self.scene, self.cloth, radius)
        self.assertEqual(self.solver.particle_size, 0.006)

    def test_idempotence_and_conflicting_radius_guard(self):
        first = configure_thin_cloth_contact(self.scene, self.cloth, 0.00075)
        self.assertEqual(first, configure_thin_cloth_contact(self.scene, self.cloth, 0.00075))
        with self.assertRaises(RuntimeError):
            configure_thin_cloth_contact(self.scene, self.cloth, 0.0005)


if __name__ == "__main__":
    unittest.main()
