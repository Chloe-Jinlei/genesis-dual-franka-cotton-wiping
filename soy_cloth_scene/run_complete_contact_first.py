"""Record a fresh native wiping task or a same-physics checkpoint continuation.

Uses the production task/continuation methods without changing control or physics.
Optional post-completion hold is separately labelled and checkpointed at its
own time. A continuation is explicitly labelled; no particle edits or threshold overrides.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import traceback
from pathlib import Path
from types import SimpleNamespace

from probe_contact_first import ContactTrace, archive_sources

ROOT = Path(__file__).resolve().parent


class FullTrace(ContactTrace):
    """Every outer-step finger/TCP samples and periodic actual pad geometry."""

    def __init__(self, task, output):
        super().__init__(task, output)
        self.pad_stream = (output / "pad_support_trace.jsonl").open("w")
        self.pad_samples = 0

    def pad_sample(self, reason):
        if not self.task.attached:
            return
        from robot_grasp import pad_contact_report
        row = {"frame": self.task.frame, "time_s": self.task.frame*self.task.dt,
               "phase": self.task.phase, "reason": reason,
               "attached": list(self.task.attached),
               "contacts": {side: pad_contact_report(self.task.h["robots"][side],
                                                     self.task.h["cloth"])
                            for side in ("left", "right")}}
        self.pad_stream.write(json.dumps(row)+"\n")
        self.pad_stream.flush()
        self.pad_samples += 1

    def step(self, *args, **kwargs):
        result = self.original_step(*args, **kwargs)
        self.sample("every_outer_step")
        if self.task.frame % 50 == 0:
            self.pad_sample("periodic_50_steps")
        return result

    def event(self, phase, **values):
        result = super().event(phase, **values)
        self.pad_sample("controller_event")
        return result

    def close(self):
        self.pad_stream.close()
        super().close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT/"config_robot_wipe.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resume", type=Path,
                        help="Trusted same-physics task checkpoint at a supported complete stage")
    parser.add_argument("--hold", type=float, default=4.,
                        help="Extra stationary observation after production task completion")
    args = parser.parse_args()
    if not math.isfinite(args.hold) or args.hold < 0:
        parser.error("--hold must be finite and nonnegative")
    output, config_path = args.output.resolve(), args.config.resolve()
    resume = args.resume.resolve() if args.resume else None
    if output.exists() and any(output.iterdir()):
        parser.error("--output must be empty; prior evidence is never overwritten")
    output.mkdir(parents=True, exist_ok=True)
    config = json.loads(config_path.read_text())
    manifest = archive_sources(output, config_path, config, resume)
    manifest["scope"] = __doc__
    manifest["fresh_uninterrupted_run"] = resume is None
    manifest["trace_outer_step_stride"] = 1
    manifest["extra_completed_task_hold_s"] = args.hold
    from importlib.util import find_spec
    genesis_root = Path(find_spec("genesis").origin).parent
    panda = genesis_root/"assets/xml/franka_emika_panda/panda.xml"
    if not panda.exists():
        panda = genesis_root/"assets/xml/franka_sim/panda.xml"
    manifest["panda_xml"] = {"path": str(panda),
        "sha256": hashlib.sha256(panda.read_bytes()).hexdigest() if panda.exists() else None}
    (output/"source_manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    scene = task = trace = None
    failure = None
    errors = []
    report = None
    task_end_time = None
    extra_hold_completed = False
    start_frame = 0
    resume_stage = None
    try:
        from scene import build_scene
        from robot_wipe import RobotWipe
        scene, handles = build_scene(config, headless=True, width=1280, height=900)
        options = SimpleNamespace(headless=True, record=True, steps=0,
                                  resume_robot_checkpoint=resume)
        task = RobotWipe(scene, handles, config, options, output)
        if resume is not None:
            from robot_checkpoint import load_checkpoint
            state = load_checkpoint(resume, scene, handles)
            resume_stage = state.get("resume_stage")
            if resume_stage not in ("after_dry_lift", "pressing", "before_final_pickup"):
                raise ValueError("Unsupported production checkpoint stage: "+str(resume_stage))
            for name, value in state.items():
                setattr(task, name, value)
            start_frame = int(task.frame)
            # Save restored native arrays before any integration for independent
            # comparison with the source checkpoint. Only the audit stage label differs.
            task.save_task_checkpoint("resume_loaded.pkl", "audit_only_resume_loaded")
            task.resume_stage = resume_stage
            (output/"resume_restore_evidence.json").write_text(json.dumps({
                "checkpoint_path": str(resume),
                "checkpoint_sha256": hashlib.sha256(resume.read_bytes()).hexdigest(),
                "restored_array_checkpoint": "resume_loaded.pkl",
                "restored_array_checkpoint_sha256": hashlib.sha256(
                    (output/"resume_loaded.pkl").read_bytes()).hexdigest(),
                "restored_frame": start_frame, "restored_time_s": start_frame*task.dt,
                "original_resume_stage": resume_stage, "steps_executed_after_load": 0,
                "material_validation_bypassed": False}, indent=2)+"\n")
        trace = FullTrace(task, output)
        trace.sample("restored_initial_state" if resume else "fresh_initial_state")
        from grasp_approach_contact import native_finger_contact_report
        preflight = {side: native_finger_contact_report(handles["robots"][side],
                     handles["cloth"], task.corner_candidates()[side])
                     for side in ("left", "right")}
        (output/"native_contact_preflight.json").write_text(json.dumps(preflight, indent=2)+"\n")
        if resume_stage == "after_dry_lift":
            task.task(resume_after_lift=True)
        elif resume_stage == "pressing":
            task.press_cloth()
            task.finish_pickup()
        elif resume_stage == "before_final_pickup":
            task.finish_pickup()
        else:
            task.task()
        task_end_time = task.frame*task.dt
        if not task.completed:
            raise RuntimeError("Production task returned without completing")
        trace.pad_sample("production_completed")
        if args.hold:
            task.event("13 Observe stationary wet cloth after complete task", extra_hold_s=args.hold)
            task.hold(args.hold)
            task.snapshot("13_after_hold")
            task.save_task_checkpoint("after_complete_hold.pkl", "audit_only_complete")
        extra_hold_completed = True
        trace.pad_sample("final_state")
    except BaseException as exc:
        failure = f"{type(exc).__name__}: {exc}"
        (output/"failure_traceback.txt").write_text(traceback.format_exc())
        if task is not None:
            try:
                task.snapshot("failure_state")
                task.save_task_checkpoint("failed_full_run.pkl", "audit_only_failure")
            except BaseException as snapshot_exc:
                errors.append("failure evidence: "+repr(snapshot_exc))
    finally:
        if task is not None:
            try:
                report = task.report(failure)
            except BaseException as report_exc:
                errors.append("report: "+repr(report_exc))
                (output/"report_error_traceback.txt").write_text(traceback.format_exc())
            finally:
                if task.writer is not None:
                    try:
                        task.writer.close()
                    except BaseException as writer_exc:
                        errors.append("video close: "+repr(writer_exc))
                    task.writer = None
        if trace is not None:
            try:
                trace.close()
            except BaseException as trace_exc:
                errors.append("trace close: "+repr(trace_exc))
        try:
            import genesis as gs
            if scene is not None:
                gs.destroy()
        except BaseException as destroy_exc:
            errors.append("destroy: "+repr(destroy_exc))
        summary = {"scope": __doc__, "fresh_uninterrupted_run": resume is None,
            "resumed_from": str(resume) if resume else None, "start_frame": start_frame,
            "resume_stage": resume_stage,
            "final_frame": task.frame if task else None,
            "production_task_completed": bool(task and task.completed),
            "production_task_end_time_s": task_end_time,
            "additional_hold_s": args.hold,
            "additional_hold_completed": extra_hold_completed,
            "final_time_s": task.frame*task.dt if task else None,
            "failure": failure, "cleanup_or_report_errors": errors,
            "trace_outer_step_stride": 1, "trace_rows": trace.count if trace else 0,
            "pad_support_sample_stride_s": 50*config["simulation"]["dt"],
            "pad_support_rows": trace.pad_samples if trace else 0,
            "raw_report_written": report is not None,
            "raw_task_passed": report.get("task_passed") if report else None,
            "raw_all_checks_passed": report.get("passed") if report else None,
            "independent_contact_and_material_acceptance": "Not asserted; audit saved states separately.",
            "video_dimensions": [1280, 900],
            "video_first_frame_time_s": ((start_frame//task.stride+1)*task.stride*task.dt
                                          if task else None),
            "video_frame_stride": task.stride if task else None}
        (output/"full_run_summary.json").write_text(json.dumps(summary, indent=2)+"\n")
        print(json.dumps(summary, indent=2), flush=True)
    if failure or errors or not report or report.get("task_passed", report.get("passed")) is not True:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
