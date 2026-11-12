"""Native contact-first grasp diagnostic; no physics or checkpoint bypass.

Fresh mode executes RobotWipe.task through its saved dry-lift checkpoint.
Resume mode accepts before_final_pickup, or a dry before_open_descent or
before_grasp_closure state. Dry continuations retain the production contact
checks and execute the remaining approach/closure, lift, and hold stages.
Every 25 outer steps and each controller event records actual/commanded finger
positions, actual TCP, and attachment state. This is a stage diagnostic, not a
new whole-task or material acceptance result.
"""
import argparse
import csv
import hashlib
import json
import math
import shutil
import traceback
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import numpy as np


ROOT = Path(__file__).resolve().parent
SIDES = ("left", "right")


class DryLiftReached(Exception):
    pass


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def archive_sources(output, config_path, config, resume):
    archive = output / "source_at_run"
    archive.mkdir()
    digests = {}
    for source in sorted(ROOT.glob("*.py")):
        destination = archive / source.name
        shutil.copyfile(source, destination)
        digests[source.name] = sha(destination)
    shutil.copyfile(config_path, archive / "input_config.json")
    (output / "config_used.json").write_text(json.dumps(config, indent=2)+"\n")
    manifest = {"created_utc": datetime.now(timezone.utc).isoformat(),
                "source_root": str(ROOT), "source_sha256": digests,
                "config_source": str(config_path), "config_sha256": sha(config_path),
                "resume_checkpoint": str(resume) if resume else None,
                "resume_checkpoint_sha256": sha(resume) if resume else None,
                "scope": __doc__}
    (output / "source_manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    return manifest


class ContactTrace:
    def __init__(self, task, output):
        from robot_wipe import arr
        self.task, self.arr, self.output = task, arr, output
        self.stream = (output / "contact_trace.csv").open("w", newline="")
        self.writer = None
        self.count = 0
        self.events = []
        self.original_step, self.original_event = task.step, task.event
        task.step, task.event = self.step, self.event

    def sample(self, reason):
        task = self.task
        row = {"frame": int(task.frame), "time_s": float(task.frame*task.dt),
               "phase": task.phase, "sample_reason": reason,
               "probe_event_index": len(self.events)-1}
        for side in SIDES:
            robot = task.h["robots"][side]
            actual = self.arr(robot.get_dofs_position(task.h["robot_dofs"][side][-2:])).reshape(-1)
            command = np.asarray(task.q[side][7:]).reshape(-1)
            for finger in range(2):
                row[f"{side}_actual_finger_{finger}_m"] = float(actual[finger])
                row[f"{side}_command_finger_{finger}_m"] = float(command[finger])
            row[side+"_actual_finger_joint_sum_m"] = float(actual.sum())
            row[side+"_command_finger_joint_sum_m"] = float(command.sum())
            for axis, value in zip("xyz", task.tcp(side)):
                row[f"{side}_tcp_{axis}_m"] = float(value)
            attached = task.attached.get(side, {})
            row[side+"_attached"] = bool(attached.get("attached", False))
            row[side+"_bound_node_count"] = len(attached.get("attached_indices", []))
        if self.writer is None:
            self.writer = csv.DictWriter(self.stream, fieldnames=list(row))
            self.writer.writeheader()
        self.writer.writerow(row)
        self.stream.flush()
        self.count += 1

    def step(self, *args, **kwargs):
        result = self.original_step(*args, **kwargs)
        if self.task.frame % 25 == 0:
            self.sample("periodic_25_steps")
        return result

    def event(self, phase, **values):
        result = self.original_event(phase, **values)
        self.events.append(self.task.events[-1])
        (self.output / "probe_events.json").write_text(json.dumps(self.events, indent=2)+"\n")
        self.sample("controller_event")
        return result

    def close(self):
        self.task.step, self.task.event = self.original_step, self.original_event
        self.stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config_robot_wipe.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resume", type=Path,
                        help="Trusted before_final_pickup, dry before_open_descent, or dry before_grasp_closure checkpoint")
    parser.add_argument("--hold", type=float, default=.5, help="Additional stationary hold after the requested stage (seconds)")
    args = parser.parse_args()
    if not math.isfinite(args.hold) or args.hold < 0:
        parser.error("--hold must be finite and nonnegative")
    output, config_path = args.output.resolve(), args.config.resolve()
    resume = args.resume.resolve() if args.resume else None
    if output.exists() and any(output.iterdir()):
        parser.error("--output must be empty to preserve prior evidence")
    output.mkdir(parents=True, exist_ok=True)
    config = json.loads(config_path.read_text())
    manifest = archive_sources(output, config_path, config, resume)
    scene = task = trace = None
    failure = None
    errors = []
    stage_reached = False
    reached_time = None
    report = None
    start_frame = None
    mode = "resume_stage_unvalidated" if resume else "fresh_dry_pickup"
    resume_stage = None
    failure_checkpoint = None
    try:
        from scene import build_scene
        from robot_wipe import RobotWipe
        scene, handles = build_scene(config, headless=True, width=960, height=720)
        options = SimpleNamespace(headless=True, record=True, steps=0,
                                  resume_robot_checkpoint=resume)
        task = RobotWipe(scene, handles, config, options, output)
        from grasp_approach_contact import native_finger_contact_report
        preflight = {side:native_finger_contact_report(handles["robots"][side],handles["cloth"],
                     task.corner_candidates()[side]) for side in SIDES}
        (output / "native_contact_preflight.json").write_text(json.dumps(preflight,indent=2)+"\n")
        if resume:
            from robot_checkpoint import load_checkpoint
            state = load_checkpoint(resume, scene, handles)
            resume_stage = state.get("resume_stage")
            if resume_stage not in ("before_final_pickup", "before_open_descent", "before_grasp_closure"):
                raise ValueError("Contact-first resume accepts only before_final_pickup, dry before_open_descent, or dry before_grasp_closure")
            if resume_stage in ("before_open_descent", "before_grasp_closure") and state.get("initial_grasp_material"):
                raise ValueError("Dry approach/closure resume is supported only before the initial dry grasp")
            for name, value in state.items():
                setattr(task, name, value)
            mode = {"before_grasp_closure": "dry_closure_resume",
                    "before_open_descent": "dry_approach_resume",
                    "before_final_pickup": "wet_final_pickup"}[resume_stage]
            if resume_stage == "before_grasp_closure":
                targets = next((event["targets_m"] for event in reversed(state.get("events", []))
                                if "targets_m" in event), None)
                if not isinstance(targets, dict) or any(side not in targets for side in SIDES):
                    raise ValueError("Dry closure checkpoint lacks the recorded approach targets")
                task.grasp_approach_targets = {side: np.asarray(targets[side], dtype=float)
                                               for side in SIDES}
                if any(value.shape != (3,) or not np.isfinite(value).all()
                       for value in task.grasp_approach_targets.values()):
                    raise ValueError("Invalid recorded approach targets in dry closure checkpoint")
        start_frame = int(task.frame)
        trace = ContactTrace(task, output)
        trace.sample("initial_or_restored_state")
        original_grasp = task.grasp
        closure_saved = False

        def save_before_closure(phase, preferred, *, close=True, required=True):
            nonlocal closure_saved
            if close and not closure_saved:
                task.save_task_checkpoint("before_grasp_closure.pkl", "before_grasp_closure")
                closure_saved = True
            return original_grasp(phase, preferred, close=close, required=required)

        task.grasp = save_before_closure
        original_open_approach = task.open_grasp_approach
        descent_saved = False

        def save_before_open_descent(phase, center, pinch_z, duration, *, wet=False):
            nonlocal descent_saved
            if not wet and not task.initial_grasp_material and not descent_saved:
                task.save_task_checkpoint("before_open_descent.pkl", "before_open_descent")
                descent_saved = True
            return original_open_approach(phase, center, pinch_z, duration, wet=wet)

        task.open_grasp_approach = save_before_open_descent
        if resume_stage in ("before_open_descent", "before_grasp_closure"):
            cotton = np.asarray(config["cloth"]["position_xy"], dtype=float)
            if resume_stage == "before_open_descent":
                pinch_z = config["robot_task"].get("pinch_tcp_z_m", .771)
                preferred = task.open_grasp_approach("02 Approach diagonal corners", cotton, pinch_z, 2.)
            else:
                preferred = task.corner_candidates()
                arrival = task.grasp_approach_report(preferred)
                task.event("Probe restored dry closure: measured open-finger arrival",
                           approach=arrival, restored_from_checkpoint=True)
                if not arrival["ready"]:
                    raise RuntimeError("Restored open-finger approach is not ready; refusing closure or lift")
            task.grasp_dry_corners(preferred)
            task.initial_grasp_material = deepcopy(task.attached)
            task.snapshot("03_grasped")
            task.move("04 Lift cloth using both robot hands", task.layout_targets(cotton, .96), 2.)
            task.snapshot("04_cloth_lifted")
            task.save_task_checkpoint("after_dry_lift.pkl", "after_dry_lift")
            stage_reached = True
        elif resume_stage == "before_final_pickup":
            task.finish_pickup()
            stage_reached = bool(task.completed)
        else:
            original_save = task.save_task_checkpoint

            def stop_after_dry(name, stage):
                saved = original_save(name, stage)
                if stage == "after_dry_lift":
                    raise DryLiftReached()
                return saved

            task.save_task_checkpoint = stop_after_dry
            try:
                task.task()
            except DryLiftReached:
                stage_reached = True
            finally:
                task.save_task_checkpoint = original_save
            if not stage_reached:
                raise RuntimeError("Fresh task returned without the dry-lift checkpoint hook")
        reached_time = task.frame*task.dt
        task.event("Probe: stationary hold after requested pickup stage", hold_seconds=args.hold)
        task.hold(args.hold)
        task.snapshot("contact_first_probe_final")
        trace.sample("final_state")
    except BaseException as exc:
        failure = {"type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc()}
        (output / "exception.json").write_text(json.dumps(failure, indent=2)+"\n")
        if task is not None:
            try:
                task.save_task_checkpoint("failed_probe_state.pkl", "audit_only_failure")
                failure_checkpoint = str(output / "failed_probe_state.pkl")
            except BaseException as checkpoint_error:
                errors.append({"operation": "failure_audit_checkpoint", "error": repr(checkpoint_error)})
            try:
                task.snapshot("contact_first_probe_failure")
            except BaseException as capture_error:
                errors.append({"operation": "failure_snapshot", "error": repr(capture_error)})
    finally:
        if task is not None:
            try:
                report = task.report(None if failure is None else failure["type"]+": "+failure["message"])
            except BaseException as report_error:
                errors.append({"operation": "task_report", "error": repr(report_error),
                               "traceback": traceback.format_exc()})
            finally:
                if task.writer is not None:
                    try:
                        task.writer.close()
                    except BaseException as writer_error:
                        errors.append({"operation": "close_video", "error": repr(writer_error)})
                    task.writer = None
        if trace is not None:
            try:
                trace.close()
            except BaseException as trace_error:
                errors.append({"operation": "close_trace", "error": repr(trace_error)})
        try:
            import genesis as gs
            gs.destroy()
        except BaseException as cleanup_error:
            errors.append({"operation": "genesis_destroy", "error": repr(cleanup_error)})
        summary = {"scope": __doc__, "mode": mode,
                   "resumed_from": str(resume) if resume else None,
                   "resume_stage": resume_stage,
                   "source_manifest": str(output / "source_manifest.json"),
                   "checkpoint_material_validation_bypassed": False,
                   "requested_stage_reached": stage_reached, "stage_reached_time_s": reached_time,
                   "additional_hold_s": args.hold, "start_frame": start_frame,
                   "final_frame": int(task.frame) if task is not None else None,
                   "trace_rows": trace.count if trace is not None else 0,
                   "periodic_sample_stride": 25, "event_boundary_samples_also_recorded": True,
                   "exception": failure, "report_or_cleanup_errors": errors,
                   "failure_audit_checkpoint": failure_checkpoint,
                   "failure_checkpoint_is_resumable": False,
                   "native_report_written": report is not None,
                   "complete_wiping_task_claimed": False,
                   "automatic_contact_first_acceptance": "Not evaluated: inspect trace and actual arrival/confirmation events.",
                   "video_dimensions": [960, 720]}
        (output / "probe_report.json").write_text(json.dumps(summary, indent=2)+"\n")
        print(json.dumps(summary, indent=2), flush=True)
    if failure is not None or errors:
        raise SystemExit(130 if failure and failure["type"] == "KeyboardInterrupt" else 1)


if __name__ == "__main__":
    main()
