"""Real controller scheduling/resume checks; no native physics or GPU."""
import ast
from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(Path(__file__).resolve().parent))
import robot_wipe
from robot_geometry import SIDES,grasp_offsets
from test_robot_press_resume import TraceTask,CheckpointStop,runtime_substitutes
from test_robot_reposition import RepositionTrace,substitutes


PLAN={"after_cycle":7,"translation_xy_m":[0.,.045]}


def make_task(schedule=True):
    task=TraceTask(absorb_cycle=8,maximum=14)
    task.press_cycles=task.completed_ups=6
    task.cfg["robot_task"]["wipe_translation_xy_m"]=[0.,.03]
    task.cfg["robot_task"]["wipe_endpoint_hold_s"]=.75
    task.current_cloth_center_xy=np.array([-.06,.04])
    task.current_cloth_yaw_rad=.11087071501577896
    if schedule:task.cfg["robot_task"]["wipe_stroke_schedule"]=[deepcopy(PLAN)]
    return task


class WipeStrokeScheduleTests(unittest.TestCase):
    def test_default_matches_archived_physical_commands_and_event_order(self):
        archive=ROOT/"output/cotton_fp64_layout_tracking/run/source_at_run/robot_wipe.py"
        cls=next(n for n in ast.parse(archive.read_text()).body
                 if isinstance(n,ast.ClassDef) and n.name=="RobotWipe")
        old=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=="press_cloth")
        namespace={"np":np,"arr":robot_wipe.arr,"SIDES":SIDES}
        exec(compile(ast.Module(body=[old],type_ignores=[]),str(archive),"exec"),namespace)
        baseline=make_task(False);current=make_task(False)
        with runtime_substitutes(baseline):namespace["press_cloth"](baseline)
        with runtime_substitutes(current):current.press_cloth()
        self.assertEqual(current.actions,baseline.actions)
        self.assertEqual(current.events,baseline.events)
        self.assertEqual(current.rows,baseline.rows)
        self.assertEqual(current.frame,baseline.frame)
        self.assertEqual(current.executed_wipe_stroke_ids,[])

    def test_selection_occurs_after_complete_seventh_cycle_and_preserves_physics_and_layout(self):
        task=make_task();original=deepcopy(task.cfg)
        baseline=make_task(False)
        with runtime_substitutes(task):task.press_cloth()
        with runtime_substitutes(baseline):baseline.press_cloth()
        self.assertEqual(task.cfg,original)
        self.assertEqual(task.frame,baseline.frame)
        self.assertEqual(len(task.actions),len(baseline.actions))
        self.assertEqual(task.executed_wipe_stroke_ids,["wipe_stroke_1"])
        self.assertEqual(len(task.wipe_stroke_history),1)
        record=task.wipe_stroke_history[0]
        self.assertEqual(record["after_completed_cycle"],7)
        self.assertEqual(record["previous_translation_xy_m"],[0.,.03])
        self.assertEqual(record["metrics"]["active_liquid_count"],3)
        self.assertEqual(record["current_center_xy_m"],[-.06,.04])
        self.assertEqual(record["current_yaw_rad"],.11087071501577896)
        moves={a[1]:a for a in task.actions if a[0]=="move"}
        span=np.linalg.norm(grasp_offsets(task.cfg)["right"]-grasp_offsets(task.cfg)["left"])
        for cycle,stroke in ((7,[0.,.03]),(8,[0.,.045])):
            targets=task.layout_targets(task.current_cloth_center_xy+stroke,
                                        task.cfg["robot_task"]["pinch_tcp_z_m"])
            outward=moves[f"08 Press cycle {cycle}: wipe outward"]
            for side in SIDES:np.testing.assert_allclose(outward[3][side],targets[side])
            self.assertAlmostEqual(np.linalg.norm(np.asarray(outward[3]["right"])-outward[3]["left"]),span)
        selection=task.saved["after_wipe_stroke_1.pkl"]
        seventh=task.saved["after_press_cycle_7.pkl"]
        self.assertEqual(selection["action_count"],seventh["action_count"])
        self.assertEqual(selection["task"]["frame"],seventh["task"]["frame"])
        self.assertEqual(selection["task"]["resume_stage"],"pressing")

    def test_resume_immediately_after_selection_does_not_repeat_or_lose_cumulative_state(self):
        complete=make_task()
        with runtime_substitutes(complete):complete.press_cloth()
        prefix=make_task()
        with runtime_substitutes(prefix,"after_wipe_stroke_1.pkl"),self.assertRaises(CheckpointStop):
            prefix.press_cloth()
        resumed=make_task()
        resumed.restore(prefix.saved["after_wipe_stroke_1.pkl"])
        before_q=deepcopy(resumed.q)
        np.testing.assert_array_equal(resumed.current_wipe_translation_xy,[0.,.045])
        for side in SIDES:np.testing.assert_array_equal(before_q[side],prefix.q[side])
        with runtime_substitutes(resumed):resumed.press_cloth()
        self.assertEqual(prefix.actions+resumed.actions,complete.actions)
        self.assertEqual(resumed.actions[0][1],"08 Press cycle 8: down")
        self.assertEqual(resumed.frame,complete.frame)
        self.assertEqual(resumed.rows,complete.rows)
        self.assertEqual(resumed.press_contacts,complete.press_contacts)
        self.assertEqual(resumed.wipe_stroke_history,complete.wipe_stroke_history)
        self.assertNotIn("after_wipe_stroke_1.pkl",resumed.saved)
        for side in SIDES:np.testing.assert_array_equal(resumed.q[side],complete.q[side])

    def test_bad_schedule_rejected_before_motion_and_old_checkpoint_defaults(self):
        invalid=[None,{},[None],[{"after_cycle":-1,"translation_xy_m":[0.,.04]}],
                 [{"after_cycle":True,"translation_xy_m":[0.,.04]}],
                 [{"after_cycle":7.0,"translation_xy_m":[0.,.04]}],
                 [{"after_cycle":7,"translation_xy_m":[0.]}],
                 [{"after_cycle":7,"translation_xy_m":[0.,float('nan')]}],
                 [PLAN,PLAN],
                 [PLAN,{"after_cycle":2,"translation_xy_m":[0.,.04]}]]
        for schedule in invalid:
            with self.subTest(schedule=schedule):
                task=make_task();task.cfg["robot_task"]["wipe_stroke_schedule"]=schedule
                with runtime_substitutes(task),self.assertRaises((ValueError,TypeError)):task.press_cloth()
                self.assertEqual(task.actions,[])
        old=make_task(False);old.ensure_press_control_state()
        self.assertEqual(old.executed_wipe_stroke_ids,[])
        self.assertEqual(old.wipe_stroke_history,[])

    def test_changed_removed_or_inconsistent_executed_plan_rejected_and_append_allowed(self):
        task=make_task()
        with runtime_substitutes(task):task.press_cloth()
        for edit in ([],[{"after_cycle":7,"translation_xy_m":[0.,.05]}]):
            altered=deepcopy(task.cfg);altered["robot_task"]["wipe_stroke_schedule"]=edit
            old_cfg=task.cfg;task.cfg=altered
            with self.assertRaisesRegex(ValueError,"changed or was removed"):task.wipe_stroke_schedule()
            task.cfg=old_cfg
        task.cfg["robot_task"]["wipe_stroke_schedule"].append(
            {"after_cycle":9,"translation_xy_m":[.005,.045]})
        self.assertEqual(len(task.wipe_stroke_schedule()),2)
        task.executed_wipe_stroke_ids=[]
        with self.assertRaisesRegex(ValueError,"IDs differ"):task.wipe_stroke_schedule()

    def test_no_new_selection_after_absorption_or_exhausted_cap(self):
        for absorbed in (True,False):
            task=make_task();task.press_cycles=task.completed_ups=8
            task.absorb_cycle=8 if absorbed else 9
            task.cfg["robot_task"]["maximum_press_cycles"]=8
            if absorbed:
                with runtime_substitutes(task):task.press_cloth()
            else:
                with runtime_substitutes(task),self.assertRaisesRegex(RuntimeError,"active particles remain"):
                    task.press_cloth()
            self.assertEqual(task.actions,[])
            self.assertEqual(task.executed_wipe_stroke_ids,[])

    def test_due_stroke_applies_after_reposition_and_uses_its_current_world_layout(self):
        task=RepositionTrace()
        task.cfg["robot_task"]["wipe_stroke_schedule"]=[
            {"after_cycle":2,"translation_xy_m":[0.,.045]}]
        with substitutes(task):task.press_cloth()
        self.assertEqual(task.wipe_stroke_history[0]["previous_translation_xy_m"],[0.,.01])
        np.testing.assert_allclose(task.wipe_stroke_history[0]["current_center_xy_m"],[0.,-.005],atol=1e-16)
        self.assertEqual(task.executed_reposition_ids,["reposition_1"])
        self.assertEqual(task.executed_wipe_stroke_ids,["wipe_stroke_1"])

    def test_promoted_config_composes_existing_repositions_layout_and_tail_stroke(self):
        # Load the actual promoted config, undoing the isolated baseline's
        # optional-feature defaults. Resume the completed-cycle-7 boundary;
        # fake physics/contact observations do not claim native verification.
        from test_cloth_layout_tracking import LayoutTrace,placed
        task=LayoutTrace()
        task.cfg=json.loads((ROOT/"config_robot_wipe.json").read_text())
        expected=json.loads((ROOT/"output/cotton_fp64_tail_wipe/config.json").read_text())
        # The archived native run predates contact-first open descent. Keep
        # this test focused on promoted schedules/layout, not whole-file
        # equality or a claim that the new grasp has native validation.
        for key in ("cloth","soy","simulation","coupling"):
            self.assertEqual(task.cfg[key],expected[key])
        for key in ("cloth_reposition_schedule","wipe_stroke_schedule",
                    "wipe_endpoint_hold_s","wet_grasp_track_layout_from_cycle"):
            self.assertEqual(task.cfg["robot_task"][key],expected["robot_task"][key])
        self.assertTrue(task.cfg["robot_task"]["contact_first_grasp"])
        self.assertEqual(task.cfg["robots"]["finger_opening"],.010)
        for key in ("pregrasp_finger_opening_m","wet_pregrasp_finger_opening_m"):
            self.assertEqual(task.cfg["robot_task"][key],.010)
        # Actual contact-first decisions have separate sensor/geometry tests;
        # this fake action trace deliberately retains its legacy grasp stub.
        task.cfg["robot_task"]["contact_first_grasp"]=False
        self.assertEqual(task.cfg["robot_task"]["maximum_press_cycles"],14)
        original=deepcopy(task.cfg)
        task.absorb_cycle=8
        task.press_cycles=task.completed_ups=7
        task.current_cloth_center_xy=np.array([-.06,.04])
        task.current_cloth_yaw_rad=.11
        task.current_wipe_translation_xy=np.array([0.,.03])
        task.positions=placed(task.fabric.reference,[-.06,.04],.11)
        plans=task.reposition_schedule()
        self.assertEqual(len(plans),2)
        task.executed_reposition_ids=[p["id"] for p in plans]
        task.reposition_history=[{"plan":deepcopy(p)} for p in plans]
        with substitutes(task):
            task.press_cloth()
            task.finish_pickup()
        self.assertEqual(task.cfg,original)
        self.assertTrue(task.completed)
        self.assertEqual(task.press_cycles,8)
        self.assertEqual(task.executed_wipe_stroke_ids,["wipe_stroke_1"])
        self.assertEqual(task.executed_reposition_ids,["reposition_1","reposition_2"])
        self.assertFalse(any("Reposition" in str(action) for action in task.actions))
        outward=next(a for a in task.actions if a[:2]==("move","08 Press cycle 8: wipe outward"))
        expected_targets=task.layout_targets([-.06,.085],task.cfg["robot_task"]["pinch_tcp_z_m"])
        for side in SIDES:np.testing.assert_allclose(outward[3][side],expected_targets[side])
        self.assertIn(("hold",.75),task.actions)
        self.assertEqual(len(task.layout_tracking_history),1)
        self.assertIn("after_press_cycle_8.pkl",task.saved)
        self.assertIn("after_wet_lift.pkl",task.saved)


if __name__=="__main__":unittest.main()
