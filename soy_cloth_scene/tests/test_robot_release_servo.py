"""Check production release restores real actuator gains before finger motion."""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_wipe import RobotWipe, SIDES


class MeasuredRobot:
    def __init__(self, side, calls):
        self.side,self.calls=side,calls
        self.kp=np.full(11,123.)
        self.kv=np.full(11,7.)
        self.kp[[8,10]]=10000.
        self.kv[[8,10]]=100.

    def set_dofs_kp(self, values, indices):
        self.calls.append((self.side,"kp"))
        self.kp[np.asarray(indices)]=values

    def set_dofs_kv(self, values, indices):
        self.calls.append((self.side,"kv"))
        self.kv[np.asarray(indices)]=values


class ReleaseServoTests(unittest.TestCase):
    def release(self, attached, robot_config=None):
        calls=[]
        task=RobotWipe.__new__(RobotWipe)
        task.cfg={} if robot_config is None else {"robots":robot_config}
        task.attached={side:{"attached":True} for side in attached}
        task.q={side:np.r_[np.arange(7),.0012,.0014] for side in SIDES}
        targets={side:q.copy() for side,q in task.q.items()}
        cloth=object()
        task.h={"cloth":cloth,"robots":{side:MeasuredRobot(side,calls) for side in SIDES},
                "robot_dofs":{side:np.array([0,1,2,3,4,5,6,8,10]) for side in SIDES}}
        def detach(actual_cloth):
            self.assertIs(actual_cloth,cloth)
            calls.append(("native","detach"))
            # Native detach happens before actuator updates and clears every
            # existing cloth attachment, whether one or both arms acquired it.
            self.assertEqual(set(task.attached),set(attached))
        events=[]
        def event(phase,**values):
            calls.append(("event",phase))
            events.append((phase,values))
        task.event=event
        with patch.dict(sys.modules,{"robot_grasp":SimpleNamespace(detach=detach)}):
            task.detach()
        self.assertEqual(task.attached,{})
        self.assertEqual(calls[:5],[("native","detach"),("left","kp"),("left","kv"),
                                   ("right","kp"),("right","kv")])
        self.assertEqual(len(calls),6)
        self.assertIn("release and pressing",events[0][0])
        expected_kp=(robot_config or {}).get("finger_position_gain",2000.)
        expected_kv=(robot_config or {}).get("finger_velocity_gain",40.)
        self.assertEqual(events[0][1],{"finger_position_gain":expected_kp,
                                      "finger_velocity_gain":expected_kv})
        for side,robot in task.h["robots"].items():
            np.testing.assert_array_equal(robot.kp[[8,10]],[expected_kp,expected_kp])
            np.testing.assert_array_equal(robot.kv[[8,10]],[expected_kv,expected_kv])
            np.testing.assert_array_equal(robot.kp[:7],np.full(7,123.))
            np.testing.assert_array_equal(robot.kv[:7],np.full(7,7.))
            np.testing.assert_array_equal(task.q[side],targets[side])

    def test_two_attached_hands_restore_configured_gains_and_leave_arm_targets_alone(self):
        self.release(SIDES,{"finger_position_gain":1750.,"finger_velocity_gain":35.})

    def test_one_attached_hand_still_restores_both_robots_before_release(self):
        self.release(("right",),{"finger_position_gain":2000.,"finger_velocity_gain":40.})

    def test_missing_robot_gain_settings_restore_documented_defaults(self):
        self.release(SIDES)


if __name__=="__main__":
    unittest.main()
