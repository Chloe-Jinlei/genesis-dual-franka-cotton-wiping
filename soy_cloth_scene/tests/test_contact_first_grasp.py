"""CPU tests of the real contact-first controller with measured I/O fixtures.

Production approach/report/grasp/feedback methods run. The authored Panda pad
distance function is real but diagnostic only. Native full-finger contact is an
explicit I/O fixture, as are link frames and actuator readings. These tests
prove command ordering/refusal, not native SDF contact or force closure.
"""
from contextlib import contextmanager
from copy import deepcopy
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(Path(__file__).resolve().parent))
import robot_grasp
from robot_wipe import RobotWipe,SIDES
from test_robot_feedback_grasp import FeedbackHarness


class StopAfterDryLift(Exception):
    pass


class ApproachHarness(FeedbackHarness):
    """Commanded arm movement and actual link pose are independent sensors."""
    def __init__(self, *, initial=None, thresholds=None, wet=False):
        super().__init__(thresholds or {"left":.00145,"right":.00095},
                         initial=initial or {s:[.010,.010] for s in SIDES},wet=wet)
        self.cfg=json.loads((ROOT/"config_robot_wipe.json").read_text())
        self.cfg["robot_task"].update(contact_first_grasp=True,
            pregrasp_finger_opening_m=.010,wet_pregrasp_finger_opening_m=.010,
            dry_grasp_feedback_closure=True,wet_grasp_feedback_closure=True)
        self.center=np.asarray(self.cfg["cloth"]["position_xy"],dtype=float)
        self.current_cloth_yaw_rad=0.
        self.positions=self.layout_targets(self.center,.96)
        self.command_positions=deepcopy(self.positions)
        self.planner=SimpleNamespace(fk=lambda side,q:(self.command_positions[side].copy(),None))
        self.tracking_errors={s:np.zeros(3) for s in SIDES}
        self.actions=[]
        self.native_contact_override={s:None for s in SIDES}
        self.native_queries=[]
        self.points=np.array([np.r_[self.positions[s][:2]+[dx,0.],.76075]
                              for s in SIDES for dx in (-.010,.010)])
        self.fabric=SimpleNamespace(reference=self.points.copy())
        self.h["cloth"]=SimpleNamespace(get_particles_pos=lambda:self.points.copy(),
            collision_radius_m=.00075,particle_size=.003)
        self.preferred=self.corner_candidates()
        for side,robot in self.robots.items():
            robot.approach_fixture=self
            robot.get_link=lambda name,s=side:self.finger_link(s,name)

    def native_contact_report(self,robot,indices):
        """Explicit fake SDF observation, independent of the real pad boxes.

        Unless overridden, our sensor fixture reports one finger touching when
        the measured TCP is within a prescribed height above the supplied
        material patch. This is a controller-input scenario, not a native
        Panda collision estimate or an alternate production contact query.
        """
        side=robot.side
        ids=np.asarray(indices,dtype=int)
        self.native_queries.append({"side":side,"indices":ids.copy()})
        sensed=(bool(len(ids)) and bool(self.tcp(side)[2]<=np.max(self.points[ids,2])+.00935))
        override=self.native_contact_override[side]
        contact=sensed if override is None else bool(override and len(ids))
        return {"any_finger_contact":contact,
            "fingers":{"left_finger":{"contact":contact},"right_finger":{"contact":False}},
            "evidence_type":"ASSUMED CPU native-contact I/O fixture; no engine query executed"}

    def finger_link(self,side,name):
        index={"left_finger":0,"right_finger":1}[name]
        sign=-1 if index==0 else 1
        # A known measured pose fixture: the central authored box's bottom is
        # 8.5 mm below TCP. Cloth distance is 1.15 mm at z=.7704, .95 mm at
        # .7702 and .75 mm at .7700. These exact pad distances are diagnostic;
        # the independent full-finger contact I/O fixture controls readiness.
        def pos():
            opening=self.robots[side].measured[7+index]
            return self.positions[side]+[sign*opening,-.0055,-.0445]
        return SimpleNamespace(get_pos=pos,get_quat=lambda:np.array([1.,0.,0.,0.]))

    def tcp(self,side):
        return self.positions[side].copy()

    def move(self,phase,targets,duration,**kwargs):
        self.actions.append({"kind":"move","phase":phase,"frame":self.frame,
            "targets":deepcopy(targets),"opening":{s:self.q[s][7:].copy() for s in SIDES},
            "orientation":kwargs.get("orientation")})
        self.event(phase)
        for side in SIDES:
            self.command_positions[side]=np.asarray(targets[side]).copy()
            self.positions[side]=np.asarray(targets[side])+self.tracking_errors[side]

    def step(self,qs=None,velocities=None):
        if qs is not None and any(np.any(qs[s][7:]<self.q[s][7:]-1e-12) for s in SIDES):
            self.actions.append({"kind":"close","frame":self.frame,
                "tcp":{s:self.tcp(s) for s in SIDES}})
        super().step(qs,velocities)

    def save_task_checkpoint(self,name,stage):
        if name=="after_dry_lift.pkl":raise StopAfterDryLift()
        raise AssertionError("Fixture should stop immediately after the dry lift")

    @contextmanager
    def io(self):
        with patch.object(robot_grasp,"attach_grasp",side_effect=self.contact_gate), \
             patch.object(robot_grasp,"pad_contact_report",return_value={
                 "both_fingers_contact":True,"fixture_assumed_support":True}):
            yield

    def run_grasp(self, *, required=True, close=True):
        with self.io():
            return self.grasp("test contact-first grasp",self.preferred,
                              required=required,close=close)


class NativeContactFixtureTests(unittest.TestCase):
    def setUp(self):
        # This deliberately replaces only the new physical observation API.
        # It neither calls nor claims to validate a native Genesis SDF kernel.
        self.contact_query=patch.dict(sys.modules,{"grasp_approach_contact":SimpleNamespace(
            native_finger_contact_report=lambda robot,cloth,indices:
                robot.approach_fixture.native_contact_report(robot,indices))})
        self.contact_query.start()
        self.addCleanup(self.contact_query.stop)


class ContactFirstApproachTests(NativeContactFixtureTests):
    def test_native_contact_fixture_requires_open_refinement_before_any_closing(self):
        task=ApproachHarness()
        before=task.points.copy()
        preferred=task.open_grasp_approach("approach",task.center,.7704,.8)
        moves=[a for a in task.actions if a["kind"]=="move"]
        np.testing.assert_allclose([a["targets"]["left"][2] for a in moves],
                                   [.7704,.7702,.7700],atol=1e-15)
        self.assertFalse(any(a["kind"]=="close" for a in task.actions))
        for move in moves:
            for side in SIDES:np.testing.assert_array_equal(move["opening"][side],[.010,.010])
        for side in SIDES:np.testing.assert_array_equal(preferred[side],task.preferred[side])
        reports=[e["approach"] for e in task.events if "approach" in e]
        self.assertEqual([r["ready"] for r in reports],[False,False,True])
        self.assertAlmostEqual(reports[-1]["contact_shell_distance_m"],.0008)
        self.assertEqual(task.gate_calls,[])  # Arrival is not an attachment.
        self.assertEqual(task.attached,{})
        self.assertTrue(task.run_grasp())
        closes=[a for a in task.actions if a["kind"]=="close"]
        self.assertTrue(closes)
        for close in closes:
            for side in SIDES:self.assertAlmostEqual(close["tcp"][side][2],.7700)
        np.testing.assert_array_equal(task.points,before)

    def test_approach_never_narrows_a_wider_existing_aperture(self):
        task=ApproachHarness(initial={"left":[.012,.013],"right":[.009,.011]})
        initial={s:q[7:].copy() for s,q in task.q.items()}
        task.open_grasp_approach("approach",task.center,.7704,.8)
        self.assertFalse(any(a["kind"]=="close" for a in task.actions))
        for frame in task.trajectory:
            for side in SIDES:self.assertTrue(np.all(frame["q"][side][7:]>=initial[side]))
        for side in SIDES:np.testing.assert_array_equal(task.q[side][7:],[.013,.013])
        firstmove=next(a for a in task.actions if a["kind"]=="move")
        for side in SIDES:np.testing.assert_array_equal(firstmove["opening"][side],[.013,.013])

    def test_already_open_does_not_issue_any_finger_motion(self):
        task=ApproachHarness()
        task.open_grasp_approach("approach",task.center,.7704,.8)
        self.assertFalse(any(f["motion_command"] for f in task.trajectory))
        self.assertTrue(any("keep fingers open" in e["phase"] for e in task.events))

    def test_tracking_error_refuses_close_even_when_actual_pad_proximity_is_good(self):
        task=ApproachHarness()
        task.positions=task.layout_targets(task.center,.7700)
        task.grasp_approach_targets=task.layout_targets(task.center,.7700)
        task.grasp_approach_targets["right"][0]+=.00061
        report=task.grasp_approach_report(task.preferred)
        self.assertFalse(report["ready"])
        right=report["hands"]["right"]
        self.assertGreater(right["target_error_m"],.0006)
        self.assertTrue(all(d<=.0008 for d in right["material_corner_pad_distances_m"].values()))
        with self.assertRaisesRegex(RuntimeError,"closure blocked"):
            task.run_grasp(required=False)
        self.assertEqual(task.trajectory,[])
        self.assertEqual(task.gate_calls,[])

    def test_arrival_is_rechecked_at_grasp_entry_not_only_during_approach(self):
        task=ApproachHarness()
        task.open_grasp_approach("approach",task.center,.7704,.8)
        task.positions["right"][2]+=.001
        frames=len(task.trajectory)
        with self.assertRaisesRegex(RuntimeError,"closure blocked"):task.run_grasp()
        self.assertEqual(len(task.trajectory),frames)
        self.assertEqual(task.gate_calls,[])

    def test_native_query_uses_supplied_original_corner_and_rejects_empty_patch(self):
        task=ApproachHarness()
        task.positions=task.layout_targets(task.center,.7700)
        task.grasp_approach_targets=deepcopy(task.positions)
        self.assertTrue(task.grasp_approach_report(task.preferred)["ready"])
        for call in task.native_queries:
            np.testing.assert_array_equal(call["indices"],task.preferred[call["side"]])
        # One finger's contact can authorize closure, not attachment; requiring
        # both pads here would recreate the downward-compression regression.
        missing=deepcopy(task.preferred);missing["right"]=missing["right"][:1]
        self.assertTrue(task.grasp_approach_report(missing)["ready"])
        empty=deepcopy(task.preferred);empty["right"]=np.array([],dtype=int)
        self.assertFalse(task.grasp_approach_report(empty)["ready"])
        self.assertTrue(task.grasp_approach_report(task.preferred)["ready"])

    def test_one_full_finger_contact_per_hand_allows_closure_but_not_attachment(self):
        task=ApproachHarness(thresholds={"left":None,"right":None})
        task.native_contact_override={s:True for s in SIDES}
        task.open_grasp_approach("approach",task.center,.7704,.8)
        moves=[a for a in task.actions if a["kind"]=="move"]
        self.assertEqual(len(moves),1)
        report=task.grasp_approach_report(task.preferred)
        self.assertTrue(report["ready"])
        for hand in report["hands"].values():
            self.assertTrue(hand["native_finger_contact"]["fingers"]["left_finger"]["contact"])
            self.assertFalse(hand["native_finger_contact"]["fingers"]["right_finger"]["contact"])
            self.assertTrue(all(d>report["contact_shell_distance_m"]
                                for d in hand["material_corner_pad_distances_m"].values()))
        self.assertEqual(task.attached,{})
        self.assertFalse(task.run_grasp(required=False))
        self.assertTrue(any(a["kind"]=="close" for a in task.actions))
        self.assertEqual(task.attached,{})
        self.assertFalse(any(c["passed"] for c in task.gate_calls))

    def test_each_hand_needs_native_contact_even_when_all_pad_boxes_are_near(self):
        for override in ({"left":True,"right":False},{"left":False,"right":False}):
            with self.subTest(native_contact=override):
                task=ApproachHarness()
                task.positions=task.layout_targets(task.center,.7700)
                task.grasp_approach_targets=deepcopy(task.positions)
                task.native_contact_override=override
                report=task.grasp_approach_report(task.preferred)
                self.assertFalse(report["ready"])
                for hand in report["hands"].values():
                    self.assertTrue(all(d<=report["contact_shell_distance_m"]
                                        for d in hand["material_corner_pad_distances_m"].values()))
                with self.assertRaisesRegex(RuntimeError,"closure blocked"):
                    task.run_grasp(required=False)
                self.assertEqual(task.trajectory,[])
                self.assertEqual(task.attached,{})
                self.assertEqual(task.gate_calls,[])

    def test_missing_target_and_nonfinite_actual_pose_cannot_pass(self):
        task=ApproachHarness()
        task.positions=task.layout_targets(task.center,.7700)
        self.assertFalse(task.grasp_approach_report(task.preferred)["ready"])
        task.grasp_approach_targets=deepcopy(task.positions)
        task.positions["left"][0]=np.nan
        self.assertFalse(task.grasp_approach_report(task.preferred)["ready"])

    def test_failed_approach_stops_at_floor_with_open_fingers_and_no_gate(self):
        task=ApproachHarness()
        task.points[:,2]-=.01
        with self.assertRaisesRegex(RuntimeError,"refusing to close or lift"):
            task.open_grasp_approach("approach",task.center,.7704,.8)
        moves=[a for a in task.actions if a["kind"]=="move"]
        self.assertEqual(len(moves),4)
        self.assertGreaterEqual(min(a["targets"]["left"][2] for a in moves),.7698)
        self.assertAlmostEqual(moves[-1]["targets"]["left"][2],.7698)
        self.assertEqual(task.snapshots,["open_approach_failed"])
        self.assertFalse(any(a["kind"]=="close" for a in task.actions))
        self.assertEqual(task.gate_calls,[])
        self.assertEqual(task.attached,{})

    def test_approach_never_reopens_an_attached_hand(self):
        task=ApproachHarness();task.attached["left"]={"attached":True}
        with self.assertRaisesRegex(RuntimeError,"both hands to be released"):
            task.open_grasp_approach("approach",task.center,.7704,.8)
        self.assertEqual(task.actions,[])
        self.assertEqual(task.trajectory,[])

    def test_actual_dry_task_descends_open_then_closes_and_only_then_lifts(self):
        task=ApproachHarness()
        with task.io(),self.assertRaises(StopAfterDryLift):task.task()
        firstclose=next(i for i,a in enumerate(task.actions) if a["kind"]=="close")
        refinements=[i for i,a in enumerate(task.actions)
                     if a["kind"]=="move" and "lower open fingers" in a["phase"]]
        lift=next(i for i,a in enumerate(task.actions)
                  if a["kind"]=="move" and a["phase"].startswith("04 Lift"))
        self.assertTrue(refinements)
        self.assertLess(max(refinements),firstclose)
        self.assertLess(firstclose,lift)
        self.assertEqual(set(task.attached),set(SIDES))
        self.assertFalse(any("Set narrow opening" in e["phase"] for e in task.events))

    def test_shared_wet_grasp_for_final_and_reposition_refines_before_closing(self):
        for prefix in (None,"08 Reposition 2"):
            with self.subTest(prefix=prefix):
                task=ApproachHarness(wet=True)
                with task.io():task.grasp_wet_corners(task.center,phase_prefix=prefix)
                firstclose=next(i for i,a in enumerate(task.actions) if a["kind"]=="close")
                moves=[(i,a) for i,a in enumerate(task.actions) if a["kind"]=="move"]
                self.assertEqual(len(moves),3)
                self.assertTrue(all(i<firstclose for i,a in moves))
                self.assertTrue(all(a["orientation"] is not None for i,a in moves))
                self.assertEqual(set(task.attached),set(SIDES))
                self.assertFalse(any("Preshape above" in e["phase"] for e in task.events))


class ContactFirstFeedbackTests(NativeContactFixtureTests):
    def ready_task(self,**kwargs):
        task=ApproachHarness(**kwargs)
        task.positions=task.layout_targets(task.center,.7700)
        task.grasp_approach_targets=deepcopy(task.positions)
        return task

    def test_coarse_then_fine_preserves_early_hand_measured_width(self):
        task=self.ready_task()
        self.assertTrue(task.run_grasp())
        for side in SIDES:
            commands=np.array([c["command"] for c in task.gate_calls if c["side"]==side])
            # Independent expected control contract, evaluated at gate times.
            np.testing.assert_allclose(commands[:9,0],np.arange(10,1,-1)*.001,atol=1e-15)
            np.testing.assert_allclose(np.diff(commands[8:,0]),-.0001,atol=1e-15)
            self.assertGreater(commands[-1,0],0.)
        left=next(s for s in task.successes if s["side"]=="left")
        right=next(s for s in task.successes if s["side"]=="right")
        self.assertLess(left["frame"],right["frame"])
        for frame in task.trajectory:
            if frame["frame"]>=left["frame"]:
                np.testing.assert_array_equal(frame["q"]["left"][7:],left["measured"])
            for side in SIDES:
                np.testing.assert_array_equal(frame["q"][side][:7],task.initial_q[side][:7])
        self.assertFalse(any(f["motion_command"] for f in task.trajectory if f["frame"]>=right["frame"]))
        self.assertLess(task.frame*task.dt,4.)  # Visible 10 mm aperture does not take 10 s.

    def test_coarse_boundary_is_per_finger_and_never_skips_past_two_mm(self):
        task=self.ready_task(initial={s:[.0034,.00205] for s in SIDES},
                             thresholds={"left":None,"right":None})
        self.assertFalse(task.run_grasp(required=False))
        calls=[c["command"] for c in task.gate_calls if c["side"]=="left"]
        np.testing.assert_allclose(calls[:3],[[.0034,.00205],[.0024,.002],[.002,.0019]],atol=1e-15)
        self.assertTrue(all(np.all(q>=0.) for q in calls))
        self.assertEqual(task.attached,{})

    def test_full_closure_still_requires_real_gate_and_preserves_required_failure(self):
        for required in (False,True):
            with self.subTest(required=required):
                task=self.ready_task(thresholds={"left":None,"right":None})
                if required:
                    with self.assertRaisesRegex(RuntimeError,"Cloth grasp contact failed for left, right"):
                        task.run_grasp(required=True)
                    self.assertEqual(task.snapshots,["grasp_failure"])
                else:self.assertFalse(task.run_grasp(required=False))
                self.assertEqual(task.attached,{})
                for side in SIDES:np.testing.assert_array_equal(task.q[side][7:],[0.,0.])
                self.assertLessEqual(task.holds.count(.05),29)


class DryReliefHarness(ApproachHarness):
    """Left material support appears only after the measured physical raise.

    The external contact fixture rejects left at every aperture until its TCP
    actually moves up by 1 mm. This tests production retry decisions without
    claiming the correction necessarily fixes a real compressed cloth corner.
    """
    def __init__(self, *, support_after_relief=True):
        super().__init__(thresholds={"left":None,"right":.00145})
        self.support_after_relief=support_after_relief

    def contact_gate(self,robot,cloth,indices,**kwargs):
        if robot.side=="left":
            relieved=self.tcp("left")[2]>=.771-1e-12
            self.thresholds["left"]=(.0002 if relieved and self.support_after_relief else None)
        return super().contact_gate(robot,cloth,indices,**kwargs)


class DryContactReliefTests(NativeContactFixtureTests):
    def test_successful_initial_grasp_adds_no_relief_or_extra_motion(self):
        task=ApproachHarness()
        preferred=task.open_grasp_approach("approach",task.center,.7704,.8)
        previous_moves=[a for a in task.actions if a["kind"]=="move"]
        with task.io():task.grasp_dry_corners(preferred)
        self.assertEqual(len([a for a in task.actions if a["kind"]=="move"]),len(previous_moves))
        self.assertFalse(any("Ease compressed" in e["phase"] for e in task.events))
        self.assertEqual(set(task.attached),set(SIDES))

    def test_only_failed_hand_rises_and_holds_its_measured_opening(self):
        task=DryReliefHarness()
        preferred=task.open_grasp_approach("approach",task.center,.7704,.8)
        before={s:task.tcp(s) for s in SIDES}
        unchanged_cfg=deepcopy(task.cfg)
        unchanged_material=task.points.copy()
        with task.io():task.grasp_dry_corners(preferred)
        relief=[a for a in task.actions if a["kind"]=="move" and "Ease compressed" in a["phase"]]
        self.assertEqual(len(relief),1)
        relief=relief[0]
        np.testing.assert_allclose(relief["targets"]["left"],before["left"]+[0.,0.,.001],atol=1e-15)
        np.testing.assert_array_equal(relief["targets"]["right"],before["right"])
        failed=next(c for c in reversed(task.gate_calls)
                    if c["side"]=="left" and not c["passed"])
        np.testing.assert_array_equal(failed["command"],[0.,0.])
        self.assertTrue(np.all(failed["measured"]>0.))
        np.testing.assert_array_equal(relief["opening"]["left"],failed["measured"])
        right=next(s for s in task.successes if s["side"]=="right")
        np.testing.assert_array_equal(relief["opening"]["right"],right["measured"])
        left=next(s for s in task.successes if s["side"]=="left")
        for frame in task.trajectory:
            if relief["frame"]<=frame["frame"]<left["frame"]:
                np.testing.assert_array_equal(frame["q"]["left"][7:],failed["measured"])
                np.testing.assert_array_equal(frame["q"]["right"][7:],right["measured"])
        self.assertEqual(set(task.attached),set(SIDES))
        self.assertEqual(task.cfg,unchanged_cfg)
        np.testing.assert_array_equal(task.points,unchanged_material)
        for call in task.gate_calls:
            self.assertEqual(call["kwargs"]["max_fixed_pair_extension"],.02)
            self.assertEqual(call["kwargs"]["gate_mode"],"bilateral_patch")

    def test_failed_relief_preserves_strict_failure_and_blocks_main_dry_lift(self):
        task=DryReliefHarness(support_after_relief=False)
        with task.io(),self.assertRaisesRegex(RuntimeError,"Cloth grasp contact failed for left"):
            task.task()
        self.assertEqual(set(task.attached),{"right"})
        self.assertEqual(task.snapshots[-1],"grasp_failure")
        self.assertNotIn("03_grasped",task.snapshots)
        moves=[a for a in task.actions if a["kind"]=="move"]
        self.assertEqual(sum("Ease compressed" in a["phase"] for a in moves),1)
        self.assertFalse(any(a["phase"].startswith(("04 Lift","05 Carry")) for a in moves))
        self.assertFalse(any(s["side"]=="left" for s in task.successes))
        relief=next(a for a in moves if "Ease compressed" in a["phase"])
        self.assertFalse(any(a["kind"]=="close" and a["frame"]>=relief["frame"] for a in task.actions))


if __name__=="__main__":unittest.main()
