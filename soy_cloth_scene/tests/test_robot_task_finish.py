"""Exercise production run termination/viewer/cleanup without a physics build."""
from contextlib import redirect_stdout
from copy import deepcopy
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import json
import sys
import time
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import robot_wipe
from task_outcome import classify_task_outcome


class EndBranchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.history=json.loads((ROOT/"output/pdf_cotton_pickup_feedback/robot_wipe_validation.json").read_text())

    def execute(self, base_report, *, error=None, steps=None):
        """Run real outer lifecycle; only scene/task physical I/O are fakes."""
        calls=[]
        captured={}
        class Viewer:
            alive=True
            def is_alive(self):
                calls.append("viewer_alive")
                return self.alive
            def update(self):
                calls.append("viewer_update")
                self.alive=False
        scene=SimpleNamespace(viewer=Viewer())
        class Writer:
            def close(self):calls.append("writer_close")
        class Task:
            def __init__(self,*unused):
                self.writer=Writer()
            def task(self):
                calls.append("task")
                if error is not None:raise error
            def snapshot(self,name):calls.append("snapshot:"+name)
            def report(self,failure):
                calls.append("report")
                captured["failure"]=failure
                report=deepcopy(base_report)
                if failure is not None:
                    report["failure"],report["passed"]=failure,False
                    if "task_passed" in report:report=classify_task_outcome(report)
                captured["report"]=report
                return report
        def build_scene(*args,**kwargs):
            calls.append("build")
            return scene,{}
        stdout=StringIO()
        caught=None
        with TemporaryDirectory() as folder:
            args=SimpleNamespace(output=Path(folder)/"run",headless=False,backend="cpu",
                                 width=100,height=100,liquid_view="particles",
                                 resume_robot_checkpoint=None,steps=steps,hold_open=True)
            with patch.dict(sys.modules,{"genesis":SimpleNamespace(destroy=lambda:calls.append("destroy")),
                                         "scene":SimpleNamespace(build_scene=build_scene)}), \
                 patch.object(robot_wipe,"RobotWipe",Task), \
                 patch.object(robot_wipe.time,"sleep",lambda seconds:calls.append("sleep")), \
                 redirect_stdout(stdout):
                try:robot_wipe.run(args,{"robot_task":{}})
                except Exception as exc:caught=exc
        self.assertEqual(calls.count("writer_close"),1)
        self.assertEqual(calls.count("destroy"),1)
        self.assertEqual(calls[-1],"destroy")
        self.assertLess(calls.index("report"),calls.index("writer_close"))
        self.assertLess(calls.index("writer_close"),calls.index("destroy"))
        return caught,calls,captured,stdout.getvalue()

    def test_material_only_warning_keeps_raw_failure_and_allows_viewer(self):
        report=classify_task_outcome(self.history)
        caught,calls,captured,text=self.execute(report)
        self.assertIsNone(caught)
        self.assertIn("viewer_update",calls)
        self.assertIs(captured["report"]["passed"],False)
        self.assertTrue(captured["report"]["task_passed"])
        self.assertIn("task completed with directional material diagnostic warnings",text)
        self.assertIn("warp_extension_within_limit",text)
        self.assertIn("original passed result and all strict checks are retained",text)

    def test_mass_or_collision_failure_still_raises_and_does_not_open_viewer(self):
        for key in ("mass_conserved","no_arm_table_or_arm_arm_collision"):
            with self.subTest(check=key):
                report=deepcopy(self.history)
                report["checks"][key]=False
                caught,calls,_,text=self.execute(classify_task_outcome(report))
                self.assertIsInstance(caught,RuntimeError)
                self.assertEqual(str(caught),"Robot task validation failed")
                self.assertNotIn("viewer_update",calls)
                self.assertNotIn("task completed",text)

    def test_runtime_exception_preserves_original_error_and_cleans_up(self):
        original=ValueError("Native liquid state nonfinite")
        caught,calls,captured,text=self.execute(classify_task_outcome(self.history),error=original)
        self.assertIs(caught,original)
        self.assertEqual(captured["failure"],str(original))
        self.assertFalse(captured["report"]["task_passed"])
        self.assertIn("snapshot:failure_state",calls)
        self.assertNotIn("viewer_update",calls)
        self.assertNotIn("task completed",text)

    def test_legacy_report_falls_back_to_original_passed(self):
        for passed in (True,False):
            with self.subTest(passed=passed):
                caught,calls,_,text=self.execute({"passed":passed,"checks":{},"failure":None})
                self.assertEqual(caught is None,passed)
                self.assertEqual("viewer_update" in calls,passed)
                self.assertNotIn("material diagnostic warnings",text)

    def test_step_limited_run_retains_existing_nonthrowing_viewer_behavior(self):
        caught,calls,captured,text=self.execute(classify_task_outcome(self.history),
                                               error=robot_wipe.StepLimit(),steps=25)
        self.assertIsNone(caught)
        self.assertIn("step limit or viewer closed",captured["failure"])
        self.assertFalse(captured["report"]["task_passed"])
        self.assertIn("viewer_update",calls)
        self.assertNotIn("task completed",text)

    def test_production_report_adds_classification_only_in_pdf_directional_mode(self):
        # Run the real report writer, substituting only measured physics I/O.
        # Historical stage measurements supply the actual directional failures.
        for directional in (True,False):
            with self.subTest(directional=directional), TemporaryDirectory() as folder:
                task=robot_wipe.RobotWipe.__new__(robot_wipe.RobotWipe)
                task.out=Path(folder)
                task.direction_limits={"warp":.02,"weft":.02,"bias":.10} if directional else None
                task.cfg={"table":{"height":.76}}
                task.fabric_stages=deepcopy(self.history["fabric_stage_measurements"])
                cp=np.array([[0.,0.,.9],[.1,0.,.9]])
                link=SimpleNamespace(get_pos=lambda:np.zeros(3),get_quat=lambda:np.array([1.,0.,0.,0.]))
                robot=SimpleNamespace(get_dofs_position=lambda ids:np.full(len(ids),.3),
                                      get_link=lambda name:link)
                metrics=deepcopy(self.history["final_metrics"])
                task.h={"coupling":SimpleNamespace(metrics=lambda:metrics,
                            bound_positions=cp.copy(),vertex_absorbed_mass=np.zeros(2)),
                        "cloth":SimpleNamespace(get_particles_pos=lambda:cp),
                        "soy":SimpleNamespace(get_particles_pos=lambda:cp,
                                              get_particles_active=lambda:np.zeros(2,dtype=bool)),
                        "robots":{side:robot for side in robot_wipe.SIDES},
                        "robot_dofs":{side:np.arange(9) for side in robot_wipe.SIDES},
                        "grasp_adapter":{}}
                task.attached={side:{"attached_indices":[k],"fixed_subset_selection":{
                    "fixed_point_pair_max_extension":0.}} for k,side in enumerate(robot_wipe.SIDES)}
                task.initial_grasp_material=deepcopy(task.attached)
                task.q={side:np.zeros(9) for side in robot_wipe.SIDES}
                task.initial_q=deepcopy(task.q)
                task.bound_before_lift=cp-np.array([0.,0.,.1])
                task.completed=True
                task.max_mass_error=task.max_loading_error=task.max_saturation=0.
                task.max_tcp_error=task.max_ik_error=0.
                task.rigid_collision_samples=0
                task.minimum_cloth_z=.76
                task.press_contacts=[{side:{"contact_count":1} for side in robot_wipe.SIDES}]
                task.frame,task.dt,task.started=100,.002,time.monotonic()
                task.args=SimpleNamespace(resume_robot_checkpoint=None)
                task.initial_mass=.01
                task.max_edge_extension=task.max_p99_extension=0.
                task.rows=[]
                fake_grasp=SimpleNamespace(_pad_distance=lambda points,link:np.zeros(len(points)),
                                           cloth_contact_radius=lambda cloth:.00075)
                with patch.dict(sys.modules,{"robot_grasp":fake_grasp}), redirect_stdout(StringIO()):
                    report=task.report()
                disk=json.loads((Path(folder)/"robot_wipe_validation.json").read_text())
                self.assertEqual(disk,report)
                self.assertEqual("task_passed" in report,directional)
                if directional:
                    self.assertFalse(report["passed"])
                    self.assertTrue(report["task_passed"])
                    self.assertFalse(report["strict_material_passed"])
                    self.assertEqual(len(report["material_warnings"]),2)


if __name__=="__main__":
    unittest.main()
