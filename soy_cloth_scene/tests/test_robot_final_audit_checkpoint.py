"""Terminal native evidence is saved after the last snapshot, without motion.

Only physical I/O and the native serializer are substituted. The controller's
finish sequence, task-state capture, and public best-effort save wrapper run.
"""
from contextlib import redirect_stdout
from copy import deepcopy
from io import StringIO
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(Path(__file__).resolve().parent))
import robot_checkpoint
from test_robot_reposition import RepositionTrace


class FinalAuditCheckpointTests(unittest.TestCase):
    def execute_finish(self, folder, *, fail_save=False):
        task=RepositionTrace(schedule=False)
        task.out=Path(folder)
        task.h["coupling"].bound_positions=np.array([[.01,.02,.77],[.03,.04,.78]])
        snapshots=[]
        saved=[]
        original_snapshot=task.snapshot
        def snapshot(name):
            original_snapshot(name)
            snapshots.append((name,task.frame,deepcopy(task.fabric_stages)))
        task.snapshot=snapshot
        def serialize(path,scene,handles,state):
            # Check what is presented to the real checkpoint serializer, before
            # returning its successful path; no fake solver build is involved.
            saved.append({"path":Path(path),"frame":task.frame,
                          "actions":deepcopy(task.actions),"state":deepcopy(state)})
            if fail_save:raise OSError("audit disk full")
            Path(path).write_bytes(b"test serializer completed")
            return Path(path).resolve()
        fake_grasp=SimpleNamespace(pad_contact_report=lambda robot,cloth:
            {"contact_count":2,"both_fingers_contact":True})
        stdout=StringIO()
        with patch.dict(sys.modules,{"robot_grasp":fake_grasp}), \
             patch.object(robot_checkpoint,"_save_checkpoint",side_effect=serialize), \
             redirect_stdout(stdout):
            task.finish_pickup()
        return task,snapshots,saved,stdout.getvalue()

    def test_saved_after_final_snapshot_with_terminal_ledger_metadata_and_no_step(self):
        with TemporaryDirectory() as folder:
            task,snapshots,saved,text=self.execute_finish(folder)
            self.assertEqual(len(saved),1)
            record=saved[0]
            self.assertEqual(record["path"].name,"after_wet_lift.pkl")
            self.assertTrue(record["path"].exists())
            self.assertEqual(snapshots[-1][0],"12_wet_cloth_lifted")
            self.assertEqual(record["frame"],snapshots[-1][1])
            self.assertEqual(record["frame"],task.frame)
            self.assertEqual(record["actions"],task.actions)
            state=record["state"]
            self.assertEqual(state["resume_stage"],"audit_only_complete")
            self.assertTrue(state["completed"])
            self.assertEqual(state["fabric_stages"],snapshots[-1][2])
            self.assertEqual(state["final_pad_contacts"],task.final_pad_contacts)
            np.testing.assert_array_equal(state["bound_before_lift"],task.bound_before_lift)
            self.assertEqual(set(state["attached"]),{"left","right"})
            self.assertTrue(task.final_audit_checkpoint["saved"])
            self.assertEqual(task.final_audit_checkpoint["path"],str(record["path"].resolve()))
            self.assertNotIn("Save skipped",text)

    def test_save_failure_is_explicit_and_does_not_claim_an_audit_file_or_change_motion(self):
        with TemporaryDirectory() as good,TemporaryDirectory() as bad:
            success,*_=self.execute_finish(good)
            failed,snapshots,saved,text=self.execute_finish(bad,fail_save=True)
            self.assertEqual(failed.actions,success.actions)
            self.assertEqual(failed.events,success.events)
            self.assertEqual(failed.frame,success.frame)
            self.assertTrue(failed.completed)  # Physical outcome and optional evidence are distinct.
            self.assertFalse(failed.final_audit_checkpoint["saved"])
            self.assertIsNone(failed.final_audit_checkpoint["path"])
            self.assertFalse((Path(bad)/"after_wet_lift.pkl").exists())
            self.assertIn("Save skipped: OSError: audit disk full",text)
            self.assertEqual(len(saved),1)
            self.assertEqual(snapshots[-1][0],"12_wet_cloth_lifted")


if __name__=="__main__":unittest.main()
