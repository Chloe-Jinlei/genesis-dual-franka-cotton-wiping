"""Rigid material observation, guarded approach, and continuous robot targets."""
from copy import deepcopy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(Path(__file__).resolve().parent))
from cloth_layout import fit_planar_cloth_layout
from robot_geometry import DOWN,grasp_offsets
from robot_wipe import RobotWipe
from test_robot_reposition import RepositionTrace,substitutes


def sheet():
    x,y=np.meshgrid(np.linspace(-.14,.14,15),np.linspace(-.32,-.14,11))
    return np.c_[x.ravel(),y.ravel(),np.full(x.size,.766)]


def placed(reference,center=(-.058,-.003),yaw=.11):
    rotation=Rotation.from_euler("z",yaw).as_matrix()[:2,:2]
    return np.c_[(reference[:,:2]-[0.,-.23])@rotation.T+center,
                 np.full(len(reference),.76075)]


class LayoutTrace(RepositionTrace):
    def __init__(self,fail_attempts=0):
        super().__init__(absorb_cycle=4,maximum=4,schedule=False,fail_attempts=fail_attempts)
        self.cfg["robot_task"]["wet_grasp_track_layout_from_cycle"]=3
        self.press_cycles=self.completed_ups=3
        self.fabric=SimpleNamespace(reference=sheet())
        self.positions=placed(self.fabric.reference)
        self.h["cloth"].get_particles_pos=lambda:self.positions.copy()
        self.orientations=[]
        self.ensure_press_control_state()
        self.current_cloth_center_xy=np.array([-.06,-.005])

    def move(self,phase,targets,duration,**kwargs):
        orientation=kwargs.get("orientation",DOWN)
        self.orientations.append((phase,orientation.as_matrix().copy()))
        super().move(phase,targets,duration,**kwargs)


class PlanarFitTests(unittest.TestCase):
    def test_recovers_known_translation_yaw_without_scale_or_mutation(self):
        ref=sheet();pos=placed(ref);oldref=ref.copy();oldpos=pos.copy()
        fit=fit_planar_cloth_layout(ref,pos,[0.,-.23],.76)
        np.testing.assert_allclose(fit["center_xy_m"],[-.058,-.003],atol=1e-14)
        self.assertAlmostEqual(fit["yaw_rad"],.11,places=13)
        self.assertEqual(fit["scale"],1.)
        self.assertAlmostEqual(np.linalg.det(fit["rotation_matrix_xy"]),1.,places=13)
        self.assertLess(fit["max_residual_m"],1e-13)
        np.testing.assert_array_equal(ref,oldref);np.testing.assert_array_equal(pos,oldpos)

    def test_rejects_folds_elevation_nonfinite_reflection_scale_and_degeneracy(self):
        ref=sheet();good=placed(ref);cases={}
        a=good.copy();a[0,2]+=.006;cases["fold"]=a
        a=good.copy();a[:,2]+=.02;cases["elevated"]=a
        a=good.copy();a[0,0]=np.nan;cases["nonfinite"]=a
        a=good.copy();a[:,0]*=-1;cases["reflection"]=a
        a=good.copy();a[:,:2]*=1.2;cases["scale"]=a
        a=good.copy();a[:,1]=0;cases["line"]=a
        for name,pos in cases.items():
            with self.subTest(name=name),self.assertRaises(ValueError):
                fit_planar_cloth_layout(ref,pos,[0.,-.23],.76)


class TrackingControllerTests(unittest.TestCase):
    def test_disabled_and_before_trigger_add_no_motion_or_measurement_and_old_state_defaults(self):
        task=LayoutTrace();task.press_cycles=2;task.positions[:]=np.nan
        self.assertFalse(task.prepare_wet_grasp_layout())
        self.assertEqual(task.actions,[]);self.assertEqual(task.layout_tracking_history,[])
        task.press_cycles=3;del task.cfg["robot_task"]["wet_grasp_track_layout_from_cycle"]
        del task.current_cloth_yaw_rad;del task.layout_tracking_history
        self.assertFalse(task.prepare_wet_grasp_layout())
        self.assertEqual(task.current_cloth_yaw_rad,0.)
        self.assertEqual(task.layout_tracking_history,[])

    def test_invalid_or_already_attached_sheet_stops_before_any_motion(self):
        for attached in (True,False):
            task=LayoutTrace()
            if attached:task.attached={"left":{"attached":True}}
            else:task.positions[0,2]+=.02
            with self.assertRaises((RuntimeError,ValueError)):task.prepare_wet_grasp_layout()
            self.assertEqual(task.actions,[]);self.assertEqual(task.layout_tracking_history,[])

    def test_shared_layout_preserves_span_original_material_and_all_fallback_targets(self):
        task=LayoutTrace(fail_attempts=2);reference=task.fabric.reference.copy()
        old_candidates=RobotWipe.corner_candidates(task)
        plan={"id":"reposition_2","after_cycle":3,"center_offset_xy_m":[-.06,0.],"wipe_translation_xy_m":[0.,.03]}
        with substitutes(task):task.reposition_cloth(plan)
        self.assertEqual(task.actions[0][1],"08 Reposition 2: Raise empty grippers before layout alignment")
        self.assertEqual(task.actions[1][1],"08 Reposition 2: Align empty grippers with measured cloth layout")
        self.assertAlmostEqual(task.actions[0][3]["left"][2],.80)
        rotation=Rotation.from_euler("z",.11).as_matrix()[:2,:2]
        offsets=grasp_offsets(task.cfg)
        span=np.linalg.norm(offsets["right"]-offsets["left"])
        for action in task.actions:
            if action[0]!="move":continue
            targets=action[3]
            self.assertAlmostEqual(np.linalg.norm(np.array(targets["right"])-targets["left"]),span,places=12)
            if "Raise empty" not in action[1]:
                np.testing.assert_allclose(np.array(targets["right"][:2])-targets["left"][:2],
                                           rotation@(offsets["right"]-offsets["left"]),atol=1e-13)
        for phase,matrix in task.orientations:
            expected=DOWN if "Raise empty" in phase else Rotation.from_euler("z",.11)*DOWN
            np.testing.assert_allclose(matrix,expected.as_matrix(),atol=1e-13)
        for phrase in ("Lower grippers","Refine wet-cloth","Ease wet-cloth","Lift same"):
            action=next(a for a in task.actions if a[0]=="move" and phrase in a[1])
            for side in offsets:
                np.testing.assert_allclose(action[3][side][:2],[-.058,-.003]+rotation@offsets[side],atol=1e-13)
        np.testing.assert_array_equal(task.fabric.reference,reference)
        for side in offsets:np.testing.assert_array_equal(RobotWipe.corner_candidates(task)[side],old_candidates[side])
        saved=task.saved['after_reposition_2.pkl']
        self.assertAlmostEqual(saved['task']['current_cloth_yaw_rad'],.11)
        self.assertEqual(len(saved['task']['layout_tracking_history']),1)
        resumed=LayoutTrace();resumed.restore(saved)
        self.assertAlmostEqual(resumed.current_cloth_yaw_rad,.11)
        np.testing.assert_allclose(resumed.current_cloth_center_xy,[-.06,.04],atol=1e-13)
        resumed.positions=placed(resumed.fabric.reference,[-.06,.04])
        with substitutes(resumed):resumed.press_cloth()
        outward=next(a for a in resumed.actions if a[0]=="move" and "wipe outward" in a[1])
        for side in offsets:
            np.testing.assert_allclose(outward[3][side][:2],[-.06,.07]+rotation@offsets[side],atol=1e-13)

    def test_final_pickup_remeasures_before_capturing_center_and_keeps_it_during_lift(self):
        task=LayoutTrace();task.positions=placed(task.fabric.reference,[-.057,.042],.13)
        with substitutes(task):task.finish_pickup()
        rotation=Rotation.from_euler("z",.13).as_matrix()[:2,:2]
        for phrase in ("Lower grippers","Lift wet cloth"):
            action=next(a for a in task.actions if a[0]=="move" and phrase in a[1])
            for side,offset in grasp_offsets(task.cfg).items():
                np.testing.assert_allclose(action[3][side][:2],[-.057,.042]+rotation@offset,atol=1e-13)
        self.assertTrue(task.completed)


if __name__=="__main__":unittest.main()
