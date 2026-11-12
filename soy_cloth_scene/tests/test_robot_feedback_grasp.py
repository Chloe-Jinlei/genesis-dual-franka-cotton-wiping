"""CPU control-flow regression for the production grasp feedback methods.

The actual RobotWipe.grasp/close_grasp_with_feedback/fingers methods execute.
Only physical I/O is replaced: an independently specified measured-opening gate
can accept one hand before the other or reject every contact. These checks do
not validate native force closure, cloth geometry, or GPU dynamics.
"""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_wipe import RobotWipe, SIDES


class Robot:
    def __init__(self, side, q):
        self.side, self.measured = side, q.copy()
        self.kp_calls, self.kv_calls = [], []

    def get_dofs_position(self, indices):
        return self.measured[np.asarray(indices)].copy()

    def get_contacts(self):
        return {}

    def set_dofs_kp(self, values, indices):
        self.kp_calls.append((np.asarray(indices).copy(), np.asarray(values).copy()))

    def set_dofs_kv(self, values, indices):
        self.kv_calls.append((np.asarray(indices).copy(), np.asarray(values).copy()))


class FeedbackHarness(RobotWipe):
    """Exercise production decisions; record the commanded actuator trajectory."""
    def __init__(self, thresholds, *, initial=None, feedback=True, wet=True):
        self.dt, self.frame = .01, 0
        self.cfg = {"robot_task": {"wet_grasp_feedback_closure": feedback,
                                  "contact_first_grasp": False,
                                  "maximum_fixed_pair_extension": .02}}
        self.wet_grasp_active = wet
        self.attached, self.events, self.snapshots = {}, [], []
        self.trajectory, self.holds, self.gate_calls, self.successes = [], [], [], []
        initial = initial or {s: [.00035, .00035] for s in SIDES}
        # Distinct arm coordinates detect an accidental change of nonfinger DOFs.
        self.q = {s: np.r_[np.arange(7)*.1 + k, initial[s]] for k, s in enumerate(SIDES)}
        self.initial_q = {s: q.copy() for s, q in self.q.items()}
        self.robots = {s: Robot(s, q) for s, q in self.q.items()}
        self.points = np.array([[0., 0., 0.], [.002, 0., 0.]])
        self.h = {"robots": self.robots, "robot_dofs": {s: np.arange(9) for s in SIDES},
                  "cloth": SimpleNamespace(get_particles_pos=lambda: self.points.copy())}
        self.fabric = SimpleNamespace(reference=self.points.copy())
        self.preferred = {s: np.arange(2) for s in SIDES}
        self.thresholds = thresholds
        # Measured finger positions deliberately differ from commanded targets.
        self.deflection = np.array([.00002, .00003])
        for robot in self.robots.values():
            robot.measured[7:] += self.deflection

    def tcp(self, side):
        return np.zeros(3)

    def event(self, phase, **values):
        self.events.append({"phase": phase, **values})

    def snapshot(self, name):
        self.snapshots.append(name)

    def step(self, qs=None, velocities=None):
        if qs is not None:
            self.q = {s: np.asarray(qs[s]).copy() for s in SIDES}
        for side in SIDES:
            # The fake sensor models a small closure deflection. For attached
            # hands it reports the held opening without imposing further load.
            self.robots[side].measured = self.q[side].copy()
            if side not in self.attached:
                self.robots[side].measured[7:] += self.deflection
        self.trajectory.append({"frame": self.frame, "motion_command": qs is not None,
                                "q": {s: q.copy() for s, q in self.q.items()}})
        self.frame += 1

    def hold(self, seconds):
        self.holds.append(seconds)
        for _ in range(round(seconds/self.dt)):
            self.step()

    def contact_gate(self, robot, cloth, indices, **kwargs):
        """Accept only measured support in the fixture's independently set band."""
        threshold = self.thresholds[robot.side]
        measured = robot.measured[7:].copy()
        passes = threshold is not None and bool(np.max(measured) <= threshold)
        self.gate_calls.append({"side": robot.side, "frame": self.frame,
                                "measured": measured, "command": self.q[robot.side][7:].copy(),
                                "kwargs": kwargs, "passed": passes})
        if passes:
            self.successes.append({"side": robot.side, "frame": self.frame,
                                   "measured": measured.copy()})
        return {"attached": passes, "reasons": [] if passes else ["actual fixture contact absent"]}

    def run_grasp(self, *, required=True, close=True):
        with patch.dict(sys.modules, {"robot_grasp": SimpleNamespace(attach_grasp=self.contact_gate)}):
            return self.grasp("test wet grasp", self.preferred, required=required, close=close)


class FeedbackGraspTests(unittest.TestCase):
    def assert_arm_targets_unchanged(self, task):
        for frame in task.trajectory:
            for side in SIDES:
                np.testing.assert_array_equal(frame["q"][side][:7], task.initial_q[side][:7])

    def test_early_hand_holds_measured_width_while_other_hand_closes(self):
        task = FeedbackHarness({"left": .00029, "right": .00009})
        self.assertTrue(task.run_grasp())
        left = next(x for x in task.successes if x["side"] == "left")
        right = next(x for x in task.successes if x["side"] == "right")
        self.assertLess(left["frame"], right["frame"])
        left_call = next(x for x in task.gate_calls if x["side"] == "left" and x["passed"])
        self.assertGreater(np.linalg.norm(left["measured"]-left_call["command"]), 1e-5)
        after = [f for f in task.trajectory if f["frame"] >= left["frame"]]
        self.assertTrue(after)
        for frame in after:
            np.testing.assert_array_equal(frame["q"]["left"][7:], left["measured"])
        self.assertGreater(after[0]["q"]["right"][7], after[-1]["q"]["right"][7])
        self.assertEqual(sum(c["side"] == "left" and c["passed"] for c in task.gate_calls), 1)
        for side in SIDES:
            self.assertEqual(len(task.robots[side].kp_calls), 1)
            np.testing.assert_array_equal(task.robots[side].kp_calls[0][1], [10000., 10000.])
            np.testing.assert_array_equal(task.robots[side].kv_calls[0][1], [100., 100.])
        self.assert_arm_targets_unchanged(task)

    def test_each_success_stops_immediately_without_finishing_zero_closure(self):
        task = FeedbackHarness({"left": .00029, "right": .00009})
        self.assertTrue(task.run_grasp())
        last_success = max(x["frame"] for x in task.successes)
        self.assertFalse(any(f["motion_command"] for f in task.trajectory
                             if f["frame"] >= last_success))
        for success in task.successes:
            np.testing.assert_array_equal(task.q[success["side"]][7:], success["measured"])
            self.assertTrue(np.all(task.q[success["side"]][7:] > 0))
        self.assertEqual(task.holds.count(.05), 3)
        self.assertEqual(task.holds.count(.2), 1)
        self.assertEqual(len(task.successes), 2)
        for call in task.gate_calls:
            self.assertEqual(call["kwargs"]["max_fixed_pair_extension"], .02)
            self.assertTrue(call["kwargs"]["preserve_contact_geometry"])
            self.assertEqual(call["kwargs"]["gate_mode"], "bilateral_patch")
            np.testing.assert_array_equal(call["kwargs"]["commanded_finger_opening"], call["command"])

    def test_initial_contact_needs_no_closure_motion(self):
        task = FeedbackHarness({"left": .0004, "right": .0004})
        self.assertTrue(task.run_grasp())
        self.assertFalse(any(f["motion_command"] for f in task.trajectory))
        self.assertEqual(len(task.gate_calls), 2)
        self.assertEqual(task.holds, [.2])

    def test_optional_failure_stops_at_zero_without_forcing_attachment(self):
        task = FeedbackHarness({"left": None, "right": None},
                               initial={"left": [0., .00025], "right": [.00015, .00005]})
        before = task.points.copy()
        self.assertFalse(task.run_grasp(required=False))
        self.assertEqual(task.attached, {})
        self.assertEqual(task.successes, [])
        self.assertEqual(task.snapshots, [])
        self.assertEqual(task.holds.count(.05), 3)
        for side in SIDES:
            np.testing.assert_array_equal(task.q[side][7:], [0., 0.])
        for frame in task.trajectory:
            for side in SIDES:
                self.assertTrue(np.all(frame["q"][side][7:] >= 0))
        np.testing.assert_array_equal(task.points, before)
        self.assert_arm_targets_unchanged(task)

    def test_required_failure_reports_unattached_hand_and_preserves_success(self):
        task = FeedbackHarness({"left": .00029, "right": None})
        with self.assertRaisesRegex(RuntimeError, "Cloth grasp contact failed for right"):
            task.run_grasp(required=True)
        self.assertEqual(set(task.attached), {"left"})
        self.assertEqual(task.snapshots, ["grasp_failure"])
        np.testing.assert_array_equal(task.q["right"][7:], [0., 0.])
        success = task.successes[0]
        for frame in task.trajectory:
            if frame["frame"] >= success["frame"]:
                np.testing.assert_array_equal(frame["q"]["left"][7:], success["measured"])
        self.assertFalse(any(c["passed"] for c in task.gate_calls if c["side"] == "right"))

    def test_required_total_failure_remains_failure_at_initial_zero_targets(self):
        task = FeedbackHarness({"left": None, "right": None},
                               initial={s: [0., 0.] for s in SIDES})
        with self.assertRaisesRegex(RuntimeError, "Cloth grasp contact failed for left, right"):
            task.run_grasp()
        self.assertEqual(task.trajectory, [])
        self.assertEqual(task.attached, {})
        self.assertEqual(task.snapshots, ["grasp_failure"])

    def test_close_false_checks_contact_without_recursive_feedback_motion(self):
        task = FeedbackHarness({"left": None, "right": None})
        self.assertFalse(task.run_grasp(close=False, required=False))
        self.assertEqual(task.trajectory, [])
        self.assertEqual(len(task.gate_calls), 2)

    def test_feedback_is_opt_in_and_does_not_change_dry_prefix(self):
        for wet, feedback in [(True, False), (False, True)]:
            with self.subTest(wet=wet, feedback=feedback):
                task = FeedbackHarness({"left": None, "right": None}, wet=wet, feedback=feedback)
                self.assertFalse(task.run_grasp(required=False))
                self.assertEqual(len(task.gate_calls), 2)
                self.assertNotIn(.05, task.holds)
                self.assertEqual(sum(f["motion_command"] for f in task.trajectory), 80)
                self.assertFalse(any("close with contact feedback" in e["phase"] for e in task.events))

    def test_dry_feedback_preserves_contact_geometry_and_measured_aperture(self):
        task = FeedbackHarness({"left": .00029, "right": .00009}, wet=False)
        task.cfg["robot_task"]["dry_grasp_feedback_closure"] = True
        self.assertTrue(task.run_grasp())
        self.assertEqual(len(task.successes), 2)
        for call in task.gate_calls:
            self.assertTrue(call["kwargs"]["preserve_contact_geometry"])
            self.assertEqual(call["kwargs"]["max_fixed_pair_extension"], .02)
        for success in task.successes:
            np.testing.assert_array_equal(task.q[success["side"]][7:], success["measured"])
            self.assertTrue(np.all(task.q[success["side"]][7:] > 0.))
            self.assertEqual(len(task.robots[success["side"]].kp_calls), 1)
        self.assert_arm_targets_unchanged(task)


if __name__ == "__main__":
    unittest.main()
