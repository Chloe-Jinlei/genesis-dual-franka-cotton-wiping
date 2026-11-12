"""CPU checks for material compatibility before checkpoint state mutation."""
import copy
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_checkpoint import (FORMAT, FORMAT_VERSION, _material_profile_fingerprint,
                              _signature, _validate_saved_signature)


@dataclass
class Layout:
    n_envs: int = 0


class CheckpointMaterialProfileTests(unittest.TestCase):
    def setUp(self):
        p = np.array([[0., 0., 0.], [.01, 0., 0.], [0., .01, 0.]])
        self.scene = SimpleNamespace(_desc_digest="unchanged-native-digest",
                                      _layout=Layout(), entities=[])
        material = SimpleNamespace(model_dump=lambda: {"rho": .18, "stretch_compliance": 1e-7})
        cloth = SimpleNamespace(n_particles=3, solver=SimpleNamespace(particle_radius=.00075), material=material)
        soy = SimpleNamespace(n_particles=1, material=SimpleNamespace(model_dump=lambda: {"rho": 1100.}))
        coupling = SimpleNamespace(_initial_cloth_pos=p, _initial_soy_pos=np.zeros((1, 3)),
            faces=np.array([[0, 1, 2]], np.int64), liquid_particle_mass=np.array([7.04e-6]),
            dry_vertex_mass=np.full(3, 3e-6), config={"capacity_kg_m2": 1.08})
        self.handles = {"cloth": cloth, "soy": soy, "coupling": coupling,
            "material_profile": {"cloth_config": {"areal_density": .18},
                "model": {"bending_rigidity_nm": 5e-5, "warp_max_extension": .02,
                          "weft_max_extension": .02, "bias_max_extension": .10,
                          "internal_damping_per_s": 4.}}}

    def record(self):
        return {"format": FORMAT, "format_version": FORMAT_VERSION,
                "signature": _signature(self.scene, self.handles)}

    def test_same_new_schema_and_material_is_compatible(self):
        saved = self.record()
        _validate_saved_signature(saved, self.scene, self.handles)
        reversed_profile = {key: self.handles["material_profile"][key]
                            for key in reversed(self.handles["material_profile"])}
        self.assertEqual(_material_profile_fingerprint(reversed_profile),
                         _material_profile_fingerprint(self.handles["material_profile"]))

    def test_same_mesh_and_native_material_reject_changed_adapter_physics(self):
        saved = self.record()
        for key, value in (("bending_rigidity_nm", 1e-4), ("warp_max_extension", .03),
                           ("bias_max_extension", .08), ("internal_damping_per_s", 6.)):
            changed = dict(self.handles)
            changed["material_profile"] = copy.deepcopy(self.handles["material_profile"])
            changed["material_profile"]["model"][key] = value
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "material profile"):
                _validate_saved_signature(saved, self.scene, changed)

    def test_legacy_schema_and_missing_material_profile_are_rejected(self):
        saved = self.record()
        old = dict(saved, format_version=1)
        with self.assertRaisesRegex(ValueError, "schema"):
            _validate_saved_signature(old, self.scene, self.handles)
        missing = dict(self.handles)
        missing.pop("material_profile")
        with self.assertRaisesRegex(ValueError, "material profile"):
            _validate_saved_signature(saved, self.scene, missing)

    def test_profile_requires_finite_descriptions_and_no_live_objects(self):
        with self.assertRaises(ValueError):
            _material_profile_fingerprint({"cloth_config": {}})
        profile = copy.deepcopy(self.handles["material_profile"])
        profile["model"]["internal_damping_per_s"] = float("nan")
        with self.assertRaises(ValueError):
            _material_profile_fingerprint(profile)
        profile["model"] = lambda: None
        with self.assertRaises(TypeError):
            _material_profile_fingerprint(profile)


if __name__ == "__main__":
    unittest.main()
