"""Guard against confusing drape/rotation or overlapping folds with stretch."""
import sys
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cloth_metrics import ClothMetrics, acceptance


class MetricsTests(unittest.TestCase):
    limits = {"warp": .02, "weft": .02, "bias": .10}

    @staticmethod
    def grid():
        return np.array([[0, 0, 0], [.01, 0, 0], [0, .01, 0], [.01, .01, 0]])

    @staticmethod
    def stages(strain):
        return {"04_cloth_lifted": strain, "12_wet_cloth_lifted": strain,
                "08_after_pressing": {"projection_fraction_of_initial": 1.,
                                      "near_table_fraction_of_initial": 1.}}

    def test_rigid_rotation_is_not_stretch(self):
        p = np.array([[0, 0, 0], [.1, 0, 0], [0, .1, 0], [.1, .1, 0]])
        m = ClothMetrics(p, [[0, 1, 2], [1, 3, 2]])
        changed = Rotation.from_euler("xyz", [.4, .6, -.2]).apply(p)+[.7, -.3, .8]
        self.assertLess(abs(m.strain(changed)["edge_extension_max"]), 1e-12)
        self.assertAlmostEqual(m.strain(1.1*p)["edge_extension_max"], .1)

    def test_overlapping_fold_counts_projection_once(self):
        p = np.array([[0, 0, 0], [.1, 0, 0], [0, .1, 0], [.1, .1, 0], [.2, 0, 0], [.2, .1, 0]])
        m = ClothMetrics(p, [[0, 1, 2], [1, 3, 2], [1, 4, 3], [4, 5, 3]])
        folded = p.copy()
        folded[[4, 5], 0] = 0
        self.assertLess(abs(m.strain(folded)["edge_extension_max"]), 1e-12)
        self.assertAlmostEqual(m.footprint(folded)/m.reference_footprint, .5, delta=.01)

    def test_float32_footprint_is_unchanged_by_vertical_rigid_translation(self):
        # World-space offsets and decimal grid edges reproduce the live-scene
        # float32 boundary issue without depending on a saved simulation.
        x, y = np.meshgrid(np.linspace(-.014, .014, 3),
                           np.linspace(-.239, -.221, 3))
        p = np.column_stack((x.ravel(), y.ravel(), np.full(9, .766))).astype(np.float32)
        faces = []
        for j in range(2):
            for i in range(2):
                a = j*3+i
                faces.extend(((a, a+1, a+3), (a+1, a+4, a+3)))
        m = ClothMetrics(p, faces)
        translated = p.copy()
        translated[:, 2] -= .005
        self.assertGreater(m.reference_footprint, 0)
        for snapshot in (p, translated):
            with self.subTest(height=float(snapshot[0, 2])):
                self.assertEqual(m.footprint(snapshot), m.reference_footprint)
                self.assertEqual(m.footprint(snapshot), m.footprint(snapshot.astype(np.float64)))
                measurement = m.measure(snapshot, table_height=.76)
                self.assertEqual(measurement["projection_fraction_of_initial"], 1.)
                self.assertEqual(measurement["near_table_fraction_of_initial"], 1.)

    def test_material_directions_remain_fixed_during_world_rotation(self):
        p = self.grid()
        m = ClothMetrics(p, [[0, 1, 2], [1, 3, 2]], self.limits)
        changed = p.copy(); changed[:, 0] *= 1.015
        first = m.strain(changed)
        rotated = Rotation.from_euler("xyz", [.4, .6, -.2]).apply(changed)+[.7, -.3, .8]
        second = m.strain(rotated)
        self.assertEqual([first[f"{d}_edge_count"] for d in self.limits], [2, 2, 1])
        self.assertAlmostEqual(first["warp_edge_extension_max"], .015)
        self.assertAlmostEqual(first["weft_edge_extension_max"], 0.)
        for d in self.limits:
            self.assertAlmostEqual(first[f"{d}_edge_extension_max"], second[f"{d}_edge_extension_max"])

    def test_bias_shear_uses_10pct_limit_instead_of_legacy_all_edge_5pct(self):
        p = self.grid(); m = ClothMetrics(p, [[0, 1, 2], [1, 3, 2]], self.limits)
        shear = 1.08**2-1
        changed = p.copy(); changed[:, 0] -= shear*p[:, 1]; changed[:, 1] *= np.sqrt(1-shear**2)
        measured = m.strain(changed)
        self.assertAlmostEqual(measured["bias_edge_extension_max"], .08)
        self.assertAlmostEqual(measured["warp_edge_extension_max"], 0.)
        self.assertAlmostEqual(measured["weft_edge_extension_max"], 0.)
        self.assertTrue(all(acceptance(self.stages(measured), self.limits).values()))
        self.assertFalse(acceptance(self.stages(measured))["dry_and_wet_lift_peak_extension_below_5pct"])

    def test_directional_residual_is_explicit_and_bounded(self):
        p = self.grid(); m = ClothMetrics(p, [[0, 1, 2], [1, 3, 2]], self.limits)
        changed = p.copy(); changed[:, 0] *= 1.0219
        measured = m.strain(changed)
        self.assertAlmostEqual(measured["warp_bound_excess_max"], .0019)
        self.assertEqual(measured["warp_edges_over_limit"], 2)
        self.assertEqual(measured["warp_edges_over_limit_plus_residual"], 0)
        self.assertEqual(measured["directional_residual_tolerance"], .002)
        self.assertTrue(all(acceptance(self.stages(measured), self.limits).values()))
        changed[:, 0] = p[:, 0]*1.0221
        measured = m.strain(changed)
        self.assertEqual(measured["warp_edges_over_limit_plus_residual"], 2)
        self.assertFalse(acceptance(self.stages(measured), self.limits)[
            "dry_and_wet_lift_warp_extension_within_limit_and_residual"])

    def test_directional_acceptance_rejects_wrong_or_missing_profile_evidence(self):
        p = self.grid(); faces = [[0, 1, 2], [1, 3, 2]]
        old = ClothMetrics(p, faces).strain(p)
        checks = acceptance(self.stages(old), self.limits)
        self.assertFalse(checks["dry_and_wet_lift_warp_extension_within_limit_and_residual"])
        wrong = ClothMetrics(p, faces, self.limits, residual_tolerance=.003).strain(p)
        self.assertFalse(acceptance(self.stages(wrong), self.limits)[
            "dry_and_wet_lift_bias_extension_within_limit_and_residual"])
        with self.assertRaises(ValueError):
            ClothMetrics(p, faces, {"warp": .02, "bias": .1})
        with self.assertRaises(ValueError):
            ClothMetrics(p, faces, self.limits, residual_tolerance=float("nan"))


if __name__ == "__main__":
    unittest.main()
