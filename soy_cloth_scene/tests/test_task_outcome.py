"""Outcome classification using the preserved real 17/19 report plus failures."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from task_outcome import STRICT_MATERIAL_CHECKS, TASK_CHECKS, classify_task_outcome


class TaskOutcomeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # This is the recorded native run, not a synthetic 17/19 lookalike.
        cls.history_path = ROOT/"output/pdf_cotton_pickup_feedback/robot_wipe_validation.json"
        cls.history = json.loads(cls.history_path.read_text())

    def clean_report(self):
        report = deepcopy(self.history)
        report["checks"] = {name: True for name in TASK_CHECKS + STRICT_MATERIAL_CHECKS}
        report["passed"], report["failure"] = True, None
        return report

    def test_real_seventeen_of_nineteen_preserves_strict_failure_and_marks_task_complete(self):
        report = deepcopy(self.history)
        before = deepcopy(report)
        self.assertEqual(len(report["checks"]), 19)
        self.assertEqual(sum(v is True for v in report["checks"].values()), 17)
        self.assertIsNone(report["failure"])
        result = classify_task_outcome(report)
        self.assertIs(result["passed"], False)
        self.assertIs(result["task_passed"], True)
        self.assertIs(result["strict_material_passed"], False)
        self.assertEqual({w["check"] for w in result["material_warnings"]},
                         set(STRICT_MATERIAL_CHECKS[:2]))
        self.assertEqual(result["task_blockers"], [])
        self.assertEqual(report, before)
        self.assertEqual(result["checks"], report["checks"])
        self.assertIsNot(result["checks"], report["checks"])
        # The returned copy cannot accidentally mutate the retained evidence.
        result["checks"]["mass_conserved"] = False
        self.assertIs(report["checks"]["mass_conserved"], True)

    def test_all_recorded_checks_pass(self):
        result = classify_task_outcome(self.clean_report())
        self.assertTrue(result["passed"] and result["task_passed"] and result["strict_material_passed"])
        self.assertEqual(result["material_warnings"], [])
        self.assertEqual(result["task_blockers"], [])

    def test_every_nonmaterial_failure_including_collision_mass_coverage_binding_blocks_task(self):
        for name in TASK_CHECKS:
            with self.subTest(check=name):
                report = self.clean_report()
                report["checks"][name], report["passed"] = False, False
                result = classify_task_outcome(report)
                self.assertFalse(result["task_passed"])
                self.assertTrue(result["strict_material_passed"])
                self.assertEqual(result["material_warnings"], [])
                self.assertIn("failed task check: " + name, result["task_blockers"])

    def test_each_missing_expected_check_including_a_material_check_blocks_success(self):
        for name in TASK_CHECKS + STRICT_MATERIAL_CHECKS:
            with self.subTest(check=name):
                report = self.clean_report()
                del report["checks"][name]
                result = classify_task_outcome(report)
                self.assertFalse(result["task_passed"])
                self.assertFalse(result["strict_material_passed"])
                self.assertIn("missing expected check: " + name, result["task_blockers"])

    def test_unknown_failure_is_not_exempt_even_if_named_material_or_warp(self):
        for name in ("new_contact_check", "new_material_warp_check"):
            report = self.clean_report()
            report["checks"][name], report["passed"] = False, False
            self.assertFalse(classify_task_outcome(report)["task_passed"])
            report["checks"][name], report["passed"] = True, True
            self.assertTrue(classify_task_outcome(report)["task_passed"])

    def test_invalid_check_values_are_not_coerced_to_bools(self):
        for name in (TASK_CHECKS[0], STRICT_MATERIAL_CHECKS[0], "new_check"):
            for invalid in (1, 0, "true", "false", None, [], {}):
                with self.subTest(check=name, invalid=invalid):
                    report = self.clean_report()
                    report["checks"][name], report["passed"] = invalid, False
                    result = classify_task_outcome(report)
                    self.assertFalse(result["task_passed"])
                    self.assertFalse(result["strict_material_passed"])

    def test_runtime_failure_blocks_both_success_flags(self):
        report = self.clean_report()
        report["failure"], report["passed"] = "Free liquid penetrated table", False
        result = classify_task_outcome(report)
        self.assertFalse(result["task_passed"])
        self.assertFalse(result["strict_material_passed"])

    def test_missing_malformed_metadata_and_inconsistent_raw_passed_fail_closed(self):
        for key in ("passed", "failure", "checks"):
            report = self.clean_report()
            del report[key]
            self.assertFalse(classify_task_outcome(report)["task_passed"])
        for key, invalid in (("passed", 1), ("passed", None), ("passed", False),
                             ("failure", False), ("failure", []), ("checks", [])):
            report = self.clean_report()
            report[key] = invalid
            self.assertFalse(classify_task_outcome(report)["task_passed"])
        report = deepcopy(self.history)
        report["passed"] = True  # Cannot rewrite the original 17/19 outcome.
        self.assertFalse(classify_task_outcome(report)["task_passed"])
        with self.assertRaises(TypeError):
            classify_task_outcome(None)


if __name__ == "__main__":
    unittest.main()
