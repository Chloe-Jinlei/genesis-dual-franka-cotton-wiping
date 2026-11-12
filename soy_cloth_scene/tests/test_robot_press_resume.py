"""CPU continuation tests of the actual press controller and resume dispatch.

Physics, rendering and checkpoint I/O are substitutes. Motion calls retain
their targets/durations and a deterministic state so uninterrupted and resumed
execution can be compared. The archived pre-refactor task is an independent
action-order oracle; these tests do not claim native checkpoint fidelity.
"""
import ast
from contextlib import redirect_stdout
from copy import deepcopy
from io import StringIO
import json
from pathlib import Path
import re
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import robot_wipe


class CheckpointStop(Exception):
    pass


class TraceTask(robot_wipe.RobotWipe):
    def __init__(self, absorb_cycle=3, maximum=3):
        self.cfg=json.loads((ROOT/"config_robot_wipe.json").read_text())
        # This fixture exercises the original press/resume contract. Optional
        # production defaults must be enabled explicitly by their own tests.
        self.cfg["robot_task"].update(cloth_reposition_schedule=[],
                                     wipe_stroke_schedule=[],
                                     wipe_endpoint_hold_s=0.,
                                     contact_first_grasp=False,
                                     wet_grasp_track_layout_from_cycle=None)
        self.cfg["robot_task"]["maximum_press_cycles"]=maximum
        self.dt=self.cfg["simulation"]["dt"]
        self.q={s:np.arange(9,dtype=float)+k for k,s in enumerate(robot_wipe.SIDES)}
        self.initial_q=deepcopy(self.q)
        self.frame,self.phase=0,"Initialize"
        self.rows,self.events,self.actions=[],[],[]
        self.attached,self.initial_grasp_material,self.fabric_stages={},{},{}
        self.press_cycles,self.press_contacts=0,[]
        self.initial_mass=1.23
        self.max_mass_error=self.max_loading_error=self.max_saturation=0.
        self.max_tcp_error=self.max_ik_error=0.
        self.minimum_cloth_z=.76
        self.rigid_collision_samples=0
        self.max_edge_extension=self.max_p99_extension=0.
        self.out=Path("unused-output")
        self.scene=object()
        self.completed_ups=0
        self.absorb_cycle=absorb_cycle
        self.h={"robots":{s:object() for s in robot_wipe.SIDES},
                "cloth":SimpleNamespace(get_particles_pos=lambda:np.zeros((2,3))),
                "coupling":SimpleNamespace(metrics=self.metrics)}
        self.saved={}
        self.finished=False

    def metrics(self):
        return {"active_liquid_count":0 if self.completed_ups>=self.absorb_cycle else 3}

    def advance(self,seconds):
        self.frame+=round(seconds/self.dt)
        self.rows.append({"time_s":self.frame*self.dt,"phase":self.phase})
        self.max_mass_error=max(self.max_mass_error,self.frame*1e-15)
        self.max_loading_error=max(self.max_loading_error,self.frame*2e-15)
        self.max_tcp_error=max(self.max_tcp_error,self.frame*1e-10)
        self.rigid_collision_samples+=1  # Distinct cumulative sentinel, not physics.

    def event(self,phase,**values):
        self.phase=phase
        self.events.append({"phase":phase,"frame":self.frame,**values})

    def move(self,phase,targets,duration,**kwargs):
        self.event(phase)
        self.actions.append(("move",phase,duration,{s:np.asarray(p).tolist() for s,p in targets.items()}))
        for s in robot_wipe.SIDES:self.q[s][:3]=targets[s]
        self.advance(duration+.20)
        if re.fullmatch(r"08 Press cycle \d+: up",phase):self.completed_ups+=1

    def fingers(self,phase,opening,duration=.6):
        self.event(phase)
        self.actions.append(("fingers",phase,opening,duration))
        for q in self.q.values():q[-2:]=opening
        self.advance(duration+.25)

    def hold(self,seconds):
        self.actions.append(("hold",seconds))
        self.advance(seconds)

    def grasp(self,phase,preferred,**kwargs):
        self.actions.append(("grasp",phase))
        self.attached={s:{"attached":True} for s in robot_wipe.SIDES}
        return True

    def corner_candidates(self):return {s:np.array([0]) for s in robot_wipe.SIDES}
    def detach(self):
        self.actions.append(("detach",))
        self.attached={}
    def snapshot(self,name):self.fabric_stages[name]={"frame":self.frame}
    def finish_pickup(self):
        self.actions.append(("finish_pickup",))
        self.finished=True

    def checkpoint_writer(self,stop_after=None):
        def save(path,scene,handles,state):
            name=Path(path).name
            self.saved[name]={"task":deepcopy(state),"native_completed_ups":self.completed_ups,
                              "action_count":len(self.actions)}
            if name==stop_after:raise CheckpointStop(name)
            return path
        return save

    def restore(self,record):
        for key,value in deepcopy(record["task"]).items():setattr(self,key,value)
        self.completed_ups=record["native_completed_ups"]


def runtime_substitutes(task,stop_after=None):
    return patch.dict(sys.modules,{
        "robot_grasp":SimpleNamespace(pad_contact_report=lambda robot,cloth:{"contact_count":1}),
        "robot_checkpoint":SimpleNamespace(save_checkpoint=task.checkpoint_writer(stop_after)),
    })


class PressResumeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        archive=ROOT/"output/cotton_fp64_task/source_at_run/robot_wipe.py"
        tree=ast.parse(archive.read_text())
        old_class=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=="RobotWipe")
        method=next(n for n in old_class.body if isinstance(n,ast.FunctionDef) and n.name=="task")
        namespace={"np":np,"arr":robot_wipe.arr,"SIDES":robot_wipe.SIDES}
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(archive),"exec"),namespace)
        cls.original_task=staticmethod(namespace["task"])

    def test_fresh_action_targets_durations_order_and_steps_match_archived_task(self):
        for absorb_cycle in (0,1,3):
            with self.subTest(absorb_cycle=absorb_cycle):
                old=TraceTask(absorb_cycle)
                with runtime_substitutes(old):self.original_task(old)
                new=TraceTask(absorb_cycle)
                with runtime_substitutes(new):new.task()
                self.assertEqual(new.actions,old.actions)
                self.assertEqual(new.frame,old.frame)
                self.assertEqual(new.press_cycles,old.press_cycles)
                self.assertEqual(new.rows,old.rows)
                self.assertEqual(new.events,old.events)
                self.assertTrue(new.finished)
                before=new.saved["before_pressing.pkl"]
                self.assertEqual(before["task"]["resume_stage"],"pressing")
                self.assertEqual(before["task"]["press_cycles"],0)
                self.assertEqual(new.actions[before["action_count"]-1][0],"fingers")
                self.assertEqual(new.actions[before["action_count"]-1][1],"07 Release cloth flat on the table")
                for cycle in range(1,new.press_cycles+1):
                    saved=new.saved[f"after_press_cycle_{cycle}.pkl"]
                    self.assertEqual(saved["task"]["press_cycles"],cycle)
                    self.assertEqual(saved["task"]["resume_stage"],"pressing")
                    self.assertEqual(new.actions[saved["action_count"]-1],("hold",.15))
                    self.assertEqual(saved["native_completed_ups"],cycle)

    def test_resumed_cycle_does_not_replay_motion_or_drop_targets_and_cumulative_state(self):
        full=TraceTask()
        with runtime_substitutes(full):full.task()
        prefix=TraceTask()
        with runtime_substitutes(prefix,"after_press_cycle_1.pkl"), self.assertRaises(CheckpointStop):
            prefix.task()
        saved=prefix.saved["after_press_cycle_1.pkl"]
        resumed=TraceTask()
        resumed.restore(saved)
        for s in robot_wipe.SIDES:
            np.testing.assert_array_equal(resumed.q[s],prefix.q[s])
            np.testing.assert_array_equal(resumed.initial_q[s],prefix.initial_q[s])
        for field in ("frame","rows","press_contacts","max_mass_error","max_loading_error",
                      "max_tcp_error","rigid_collision_samples","initial_mass","fabric_stages"):
            self.assertEqual(getattr(resumed,field),getattr(prefix,field))
        with runtime_substitutes(resumed):
            resumed.press_cloth()
            resumed.finish_pickup()
        self.assertEqual(prefix.actions+resumed.actions,full.actions)
        self.assertEqual(resumed.frame,full.frame)
        self.assertEqual(resumed.rows,full.rows)
        self.assertEqual(resumed.press_contacts,full.press_contacts)
        self.assertEqual(resumed.rigid_collision_samples,full.rigid_collision_samples)
        self.assertEqual(resumed.max_mass_error,full.max_mass_error)
        self.assertEqual(resumed.actions[0][1],"08 Press cycle 2: down")
        self.assertNotIn("after_press_cycle_1.pkl",resumed.saved)
        for s in robot_wipe.SIDES:np.testing.assert_array_equal(resumed.q[s],full.q[s])

    def test_completed_absorption_resume_adds_no_physics_and_one_cycle_still_requires_two(self):
        for completed in (1,2):
            task=TraceTask(absorb_cycle=completed)
            task.press_cycles=task.completed_ups=completed
            frame=task.frame
            with runtime_substitutes(task):task.press_cloth()
            self.assertEqual(task.press_cycles,2)
            if completed==2:
                self.assertEqual(task.actions,[])
                self.assertEqual(task.frame,frame)
            else:
                self.assertEqual(task.actions[0][1],"08 Press cycle 2: down")
                self.assertFalse(any("wipe" in str(action) for action in task.actions))
            self.assertEqual(task.resume_stage,"before_final_pickup")

    def test_before_pressing_restore_starts_cycle_one_without_carry_lay_or_release_replay(self):
        full=TraceTask()
        with runtime_substitutes(full):full.task()
        prefix=TraceTask()
        with runtime_substitutes(prefix,"before_pressing.pkl"), self.assertRaises(CheckpointStop):
            prefix.task()
        resumed=TraceTask()
        resumed.restore(prefix.saved["before_pressing.pkl"])
        self.assertEqual(resumed.attached,{})
        self.assertEqual(resumed.press_cycles,0)
        self.assertEqual(resumed.press_contacts,[])
        with runtime_substitutes(resumed):
            resumed.press_cloth()
            resumed.finish_pickup()
        self.assertEqual(resumed.actions[0][1],"08 Press cycle 1: down")
        self.assertEqual(prefix.actions+resumed.actions,full.actions)
        self.assertEqual(resumed.frame,full.frame)
        self.assertEqual(resumed.rows,full.rows)

    def test_remaining_liquid_at_cap_still_fails_and_cannot_gain_extra_resume_cycles(self):
        task=TraceTask(absorb_cycle=4,maximum=3)
        with runtime_substitutes(task), self.assertRaisesRegex(RuntimeError,"3 active particles remain"):
            task.press_cloth()
        self.assertEqual(task.press_cycles,3)
        self.assertIn("after_press_cycle_3.pkl",task.saved)
        self.assertNotIn("before_final_pickup.pkl",task.saved)
        resumed=TraceTask(absorb_cycle=4,maximum=3)
        resumed.restore(task.saved["after_press_cycle_3.pkl"])
        before=resumed.frame
        with runtime_substitutes(resumed), self.assertRaisesRegex(RuntimeError,"3 active particles remain"):
            resumed.press_cloth()
        self.assertEqual(resumed.actions,[])
        self.assertEqual(resumed.frame,before)

    def test_bad_completed_count_is_rejected_before_any_action(self):
        for count in (-1,4,.5,True):
            task=TraceTask()
            task.press_cycles=count
            with self.subTest(count=count), runtime_substitutes(task), self.assertRaises(ValueError):
                task.press_cloth()
            self.assertEqual(task.actions,[])

    def test_optional_endpoint_hold_only_occurs_after_active_outward_motion_before_sampling(self):
        baseline=TraceTask(absorb_cycle=1)
        with runtime_substitutes(baseline):baseline.press_cloth()
        task=TraceTask(absorb_cycle=1)
        task.cfg["robot_task"]["wipe_endpoint_hold_s"]=.75
        samples=[]
        def sample():
            samples.append((task.phase,task.frame))
            return np.zeros((2,3))
        task.h["cloth"].get_particles_pos=sample
        with runtime_substitutes(task):task.press_cloth()
        self.assertEqual(len(samples),2)  # Second cycle has no active liquid.
        self.assertEqual(samples[1][0],"08 Press cycle 1: wipe outward")
        self.assertEqual(samples[1][1]-samples[0][1],round((1.6+.20+.75)/task.dt))
        self.assertEqual(task.actions.count(("hold",.75)),1)
        self.assertEqual([a for a in task.actions if a!=("hold",.75)],baseline.actions)
        self.assertEqual(task.frame-baseline.frame,round(.75/task.dt))

    def test_invalid_endpoint_hold_is_rejected_before_motion(self):
        for value in (-.1,float("nan"),float("inf")):
            task=TraceTask()
            task.cfg["robot_task"]["wipe_endpoint_hold_s"]=value
            with self.subTest(value=value), runtime_substitutes(task), self.assertRaises(ValueError):
                task.press_cloth()
            self.assertEqual(task.actions,[])


class ResumeDispatchTests(unittest.TestCase):
    def test_exact_stage_dispatch_and_unknown_stage_rejection(self):
        expected={"after_dry_lift":[("task",True)],"pressing":[("press",),("finish",)],
                  "before_final_pickup":[("finish",)]}
        for stage in (*expected,"after_pressing_typo","audit_only_complete",None):
            with self.subTest(stage=stage), TemporaryDirectory() as folder:
                calls=[]
                restored={"q":{"left":np.arange(9),"right":np.arange(9)+10},
                          "frame":123,"press_cycles":2,"rows":[{"kept":True}]}
                if stage is not None:restored["resume_stage"]=stage
                class FakeTask:
                    writer=None
                    def __init__(self,*unused):pass
                    def check_state(self):
                        np.testing.assert_array_equal(self.q["right"],restored["q"]["right"])
                        assert self.frame==123 and self.press_cycles==2 and self.rows==[{"kept":True}]
                    def task(self,resume_after_lift=False):self.check_state();calls.append(("task",resume_after_lift))
                    def press_cloth(self):self.check_state();calls.append(("press",))
                    def finish_pickup(self):self.check_state();calls.append(("finish",))
                    def snapshot(self,name):calls.append(("snapshot",name))
                    def report(self,failure):return {"passed":failure is None}
                args=SimpleNamespace(output=Path(folder),backend="cpu",headless=True,width=1,height=1,
                                     liquid_view="particles",resume_robot_checkpoint=Path("trusted.pkl"),
                                     hold_open=False,steps=None)
                with patch.dict(sys.modules,{
                    "genesis":SimpleNamespace(destroy=lambda:calls.append(("destroy",))),
                    "scene":SimpleNamespace(build_scene=lambda *a,**k:(object(),{})),
                    "robot_checkpoint":SimpleNamespace(load_checkpoint=lambda *a:deepcopy(restored)),
                }), patch.object(robot_wipe,"RobotWipe",FakeTask), redirect_stdout(StringIO()):
                    if stage in expected:robot_wipe.run(args,{})
                    else:
                        with self.assertRaisesRegex(ValueError,"Unknown robot checkpoint resume stage"):
                            robot_wipe.run(args,{})
                if stage in expected:self.assertEqual(calls,expected[stage]+[("destroy",)])
                else:self.assertEqual(calls,[("snapshot","failure_state"),("destroy",)])


if __name__=="__main__":
    unittest.main()
