"""Optional cloth-area change exercises real controller decisions, no GPU."""
import ast
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(Path(__file__).resolve().parent))
import robot_wipe
from robot_geometry import grasp_offsets,grasp_targets
from test_robot_press_resume import TraceTask,CheckpointStop


PLAN={"after_cycle":2,"center_offset_xy_m":[0.,-.045],"wipe_translation_xy_m":[0.,.01]}


class RepositionTrace(TraceTask):
    def __init__(self,absorb_cycle=4,maximum=4,schedule=True,fail_attempts=0):
        super().__init__(absorb_cycle,maximum)
        if schedule:self.cfg["robot_task"]["cloth_reposition_schedule"]=[deepcopy(PLAN)]
        self.servo=[]
        self.h["robots"]={s:SimpleNamespace(
            set_dofs_kp=lambda values,indices,s=s:self.servo.append((s,"kp",np.asarray(values).tolist())),
            set_dofs_kv=lambda values,indices,s=s:self.servo.append((s,"kv",np.asarray(values).tolist())))
            for s in robot_wipe.SIDES}
        self.h["robot_dofs"]={s:np.arange(9) for s in robot_wipe.SIDES}
        self.h["coupling"].bound_particle_indices=np.array([0,1])
        self.h["coupling"].bound_positions=np.zeros((2,3))
        self.completed=False
        self.fail_attempts=fail_attempts
        self.grasp_attempts=[]

    def tcp(self,side):return self.q[side][:3].copy()

    def metrics(self):
        return {**super().metrics(),"active_liquid_mass_kg":.003,
                "absorbed_mass_kg":.007,"total_liquid_mass_kg":.01}

    def grasp(self,phase,preferred,**kwargs):
        self.grasp_attempts.append((phase,deepcopy(kwargs),self.wet_grasp_active))
        self.actions.append(("grasp",phase))
        if self.fail_attempts:
            self.fail_attempts-=1
            return False
        self.attached={s:{"attached":True,"same_material_corner":True} for s in robot_wipe.SIDES}
        return True

    def detach(self):
        self.actions.append(("detach",))
        # Execute the production gain reset; substitute only native detach.
        robot_wipe.RobotWipe.detach(self)

    def finish_pickup(self):robot_wipe.RobotWipe.finish_pickup(self)


def substitutes(task,stop_after=None,pads=True):
    return patch.dict(sys.modules,{
        "robot_grasp":SimpleNamespace(
            pad_contact_report=lambda robot,cloth:{"contact_count":2,"both_fingers_contact":pads},
            detach=lambda cloth:None),
        "robot_checkpoint":SimpleNamespace(save_checkpoint=task.checkpoint_writer(stop_after)),
    })


class RepositionTests(unittest.TestCase):
    def test_due_plan_runs_once_between_cycles_with_real_sequence_constant_span_and_fixed_physics(self):
        task=RepositionTrace()
        before_cfg=deepcopy(task.cfg)
        with substitutes(task):task.press_cloth()
        self.assertFalse(task.completed)  # Area refresh is never task completion.
        self.assertEqual(task.executed_reposition_ids,["reposition_1"])
        self.assertEqual(len(task.reposition_history),1)
        self.assertEqual(task.cfg,before_cfg)  # Includes cloth, soy, coupling and simulation.
        np.testing.assert_allclose(task.current_cloth_center_xy,[0.,-.005],atol=1e-16)
        np.testing.assert_array_equal(task.current_wipe_translation_xy,[0.,.01])
        moves=[a for a in task.actions if a[0]=="move" and a[1].startswith("08 Reposition 1:")]
        self.assertEqual([m[1].split(": ")[1] for m in moves],[
            "Lower grippers to regrasp wet cloth","Lift same diagonal corners",
            "Translate suspended cloth","Lay shifted cloth"])
        self.assertEqual([m[2] for m in moves],[.8,2.,2.5,2.2])
        self.assertLess(next(i for i,a in enumerate(task.actions) if a[:2]==("move","08 Press cycle 2: up")),
                        next(i for i,a in enumerate(task.actions) if "Reposition 1:" in str(a)))
        self.assertLess(next(i for i,a in enumerate(task.actions) if a[0]=="fingers" and "Release shifted" in a[1]),
                        next(i for i,a in enumerate(task.actions) if a[:2]==("move","08 Press cycle 3: down")))
        offsets=grasp_offsets(task.cfg)
        expected_span=np.linalg.norm(offsets["right"]-offsets["left"])
        for action in task.actions:
            if action[0]=="move":
                p=action[3]
                self.assertAlmostEqual(np.linalg.norm(np.asarray(p["right"])-p["left"]),expected_span,places=13)
        outward=next(a for a in task.actions if a[:2]==("move","08 Press cycle 3: wipe outward"))
        for side in robot_wipe.SIDES:
            np.testing.assert_allclose(outward[3][side][:2],np.array([0.,.005])+offsets[side],atol=1e-15)
        record=task.reposition_history[0]
        self.assertTrue(record["previously_bound_parcels_preserved"])
        self.assertEqual(record["metrics_before"]["active_liquid_mass_kg"],.003)
        self.assertEqual(record["metrics_after"]["absorbed_mass_kg"],.007)
        for side in robot_wipe.SIDES:
            self.assertIn((side,"kp",[2000.,2000.]),task.servo)
            self.assertIn((side,"kv",[40.,40.]),task.servo)
        self.assertIn("after_reposition_1.pkl",task.saved)

    def test_completed_reposition_checkpoint_resumes_new_targets_without_regrasp_or_second_shift(self):
        full=RepositionTrace()
        with substitutes(full):full.press_cloth()
        prefix=RepositionTrace()
        with substitutes(prefix,"after_reposition_1.pkl"),self.assertRaises(CheckpointStop):
            prefix.press_cloth()
        saved=prefix.saved["after_reposition_1.pkl"]
        self.assertEqual(saved["task"]["resume_stage"],"pressing")
        self.assertEqual(saved["task"]["press_cycles"],2)
        resumed=RepositionTrace()
        resumed.restore(saved)
        with substitutes(resumed):resumed.press_cloth()
        self.assertEqual(prefix.actions+resumed.actions,full.actions)
        self.assertEqual(resumed.frame,full.frame)
        self.assertEqual(resumed.rows,full.rows)
        self.assertEqual(resumed.press_contacts,full.press_contacts)
        self.assertEqual(resumed.actions[0][1],"08 Press cycle 3: down")
        self.assertEqual(resumed.grasp_attempts,[])
        self.assertEqual(resumed.executed_reposition_ids,["reposition_1"])
        self.assertEqual(len(resumed.reposition_history),1)

    def test_old_checkpoint_defaults_and_final_lift_follow_shifted_center(self):
        task=RepositionTrace()
        # Emulate pre-feature checkpoints: fields have never existed.
        task.ensure_press_control_state()
        np.testing.assert_array_equal(task.current_cloth_center_xy,task.cfg["soy"]["position_xy"])
        self.assertEqual(task.executed_reposition_ids,[])
        task.current_cloth_center_xy=np.array([0.,-.005])
        with substitutes(task):task.finish_pickup()
        self.assertTrue(task.completed)
        lowered=next(a for a in task.actions if a[:2]==("move","09 Lower grippers to regrasp wet cloth"))
        raised=next(a for a in task.actions if a[:2]==("move","11 Lift wet cloth and absorbed soy together"))
        for a,z in ((lowered,task.cfg["robot_task"]["final_pinch_tcp_z_m"]),(raised,1.02)):
            target=grasp_targets(task.cfg,[0.,-.005],z)
            for side in robot_wipe.SIDES:np.testing.assert_allclose(a[3][side],target[side])

    def test_default_final_actions_and_fallbacks_match_archived_finish_exactly(self):
        archive=ROOT/"output/cotton_fp64_wide_wipe/run/source_at_run/robot_wipe.py"
        tree=ast.parse(archive.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=="RobotWipe")
        old=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=="finish_pickup")
        namespace={"np":np,"arr":robot_wipe.arr,"SIDES":robot_wipe.SIDES}
        exec(compile(ast.Module(body=[old],type_ignores=[]),str(archive),"exec"),namespace)
        for fail_attempts in (0,1,2):
            oldtask=RepositionTrace(schedule=False,fail_attempts=fail_attempts)
            newtask=RepositionTrace(schedule=False,fail_attempts=fail_attempts)
            with substitutes(oldtask):namespace["finish_pickup"](oldtask)
            with substitutes(newtask):newtask.finish_pickup()
            self.assertEqual(newtask.actions,oldtask.actions)
            self.assertEqual(newtask.events,oldtask.events)
            self.assertEqual(newtask.frame,oldtask.frame)
            self.assertEqual(newtask.servo,oldtask.servo)

    def test_contact_failure_prevents_lift_and_does_not_mark_plan_executed(self):
        task=RepositionTrace()
        task.press_cycles=task.completed_ups=2
        with substitutes(task,pads=False),self.assertRaisesRegex(RuntimeError,"actual bilateral pad support"):
            task.press_cloth()
        self.assertFalse(any(a[0]=="move" and "Lift same" in a[1] for a in task.actions))
        self.assertEqual(task.executed_reposition_ids,[])
        self.assertNotIn("after_reposition_1.pkl",task.saved)

    def test_no_reposition_when_absorbed_or_cap_exhausted_and_changed_executed_plan_rejected(self):
        absorbed=RepositionTrace(absorb_cycle=2)
        absorbed.press_cycles=absorbed.completed_ups=2
        with substitutes(absorbed):absorbed.press_cloth()
        self.assertEqual(absorbed.actions,[])
        capped=RepositionTrace(absorb_cycle=5,maximum=2)
        capped.press_cycles=capped.completed_ups=2
        with substitutes(capped),self.assertRaisesRegex(RuntimeError,"Puddle not fully absorbed"):
            capped.press_cloth()
        self.assertEqual(capped.actions,[])
        task=RepositionTrace()
        with substitutes(task):task.press_cloth()
        task.cfg["robot_task"]["cloth_reposition_schedule"][0]["center_offset_xy_m"]=[0.,-.06]
        with self.assertRaisesRegex(ValueError,"executed reposition plan changed"):
            task.reposition_schedule()


if __name__=="__main__":unittest.main()
