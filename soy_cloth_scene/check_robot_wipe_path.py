"""Audit the actual RobotWipe command generator offline, without Genesis.

Calls the controller's original task/move/fingers/hold methods. Only dynamics,
rendering, contact gates and absorption are stubbed. For contact-first control,
full-finger shell arrival is explicitly assumed only at the lowest permitted
commanded TCP height, 0.7698 m. This deliberately exercises all four open
descent attempts, not the height where native contact actually occurs.
At least one coupled finger geometry per hand is assumed to touch the original
cloth corner; the unchanged bilateral pad/material grasp gate is separate and
is assumed to succeed only at stated narrow jaw openings. No force is measured.
Absorption succeeds after the requested maximum press cycle. Every unique
commanded state is checked against original Panda MJCF collision geometry at
the configured outer-step interval.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import pickle
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from robot_geometry import Planner, SIDES, grasp_offsets
from robot_wipe import RobotWipe


ASSUMED_FULL_FINGER_CONTACT_TARGET_Z_M = .7698


class CommandTrace(RobotWipe):
    def __init__(self, config):
        self.cfg = config
        self.dt = config["simulation"]["dt"]
        self.planner = Planner(config)
        self.q = {side: np.r_[config["robots"]["home"], [config["robots"]["finger_opening"]]*2]
                  for side in SIDES}
        self.frame, self.phase = 0, "Initialize"
        self.initial_q = {s: q.copy() for s, q in self.q.items()}
        self.rows = []
        self.initial_mass = 0.0
        self.max_mass_error = self.max_loading_error = self.max_saturation = 0.0
        self.max_tcp_error = 0.0
        self.minimum_cloth_z = float("inf")
        self.rigid_collision_samples = 0
        self.out = Path(".")
        self.scene = SimpleNamespace()
        self.max_ik_error = 0.0
        self.completed = False
        self.attached = {}
        self.press_contacts = []
        from cotton_mesh import rectangular_cotton_mesh
        reference, _ = rectangular_cotton_mesh(
            size=config["cloth"].get("size_m", (.28, .18)),
            spacing=config["cloth"]["particle_size"],
            center=(*config["cloth"]["position_xy"], config["table"]["height"]+
                    config["cloth"].get("initial_clearance", .006)))
        self.fabric = SimpleNamespace(reference=reference)
        self.max_edge_extension = self.max_p99_extension = 0.0
        self.fabric_stages = {}
        self.initial_grasp_material = {}
        self.trace = []
        self.events = []
        self.servo_commands = []
        self.measured_finger_positions = {
            "left": np.array([.0003, .0003]),
            "right": np.array([.0011, .0011]),
        }
        self.dof_measurements = []
        self.approach_assumptions = []
        self.grasp_assumptions = []
        self.layout_assumptions = []
        self.press_cycles = 0
        self._offline_cloth_positions=reference.copy()
        self._offline_cloth_positions[:,2]=config["table"]["height"]+config["cloth"].get("collision_radius_m",.00075)
        self._offline_measured_layout=False
        self.h = {"robots": {side: SimpleNamespace(
                      get_contacts=lambda: {},
                      get_dofs_position=lambda indices=None, side=side:
                          self._measured_dofs(side, indices),
                      set_dofs_kp=lambda values, indices, side=side:
                          self._record_servo(side, "kp", values, indices),
                      set_dofs_kv=lambda values, indices, side=side:
                          self._record_servo(side, "kv", values, indices)) for side in SIDES},
                  "robot_dofs": {side: np.arange(9) for side in SIDES},
                  "cloth": SimpleNamespace(get_particles_pos=lambda: self._offline_cloth_positions.copy()),
                  "coupling": SimpleNamespace(metrics=self._fake_metrics,
                                              bound_particle_indices=np.empty(0,dtype=int),
                                              bound_positions=np.zeros((1, 3)))}

    def _measured_dofs(self, side, indices):
        # Deliberately differs from the zero closing target, so the trace can
        # detect whether actual measured pinch width is retained during lifts.
        measured = self.q[side].copy()
        measured[-2:] = self.measured_finger_positions[side]
        selected = np.arange(len(measured)) if indices is None else np.asarray(indices)
        result = measured[selected].copy()
        self.dof_measurements.append({"side": side, "dof_indices": selected.tolist(),
            "values": result.tolist(), "time_s": self.frame*self.dt, "assumed_offline": True})
        return result

    def _record_servo(self, side, parameter, values, indices):
        # API-compatible no-op: record the controller call, simulate no forces.
        self.servo_commands.append({"side": side, "parameter": parameter,
            "values": np.asarray(values).tolist(), "dof_indices": np.asarray(indices).tolist(),
            "time_s": self.frame*self.dt, "assumed_offline": True})

    def _fake_metrics(self):
        final_cycle = self.cfg["robot_task"].get("maximum_press_cycles", 14)
        complete = self.phase == f"08 Press cycle {final_cycle}: up" or self.press_cycles >= final_cycle
        return {"active_liquid_count": 0 if complete else 1}

    def reposition_cloth(self, plan):
        super().reposition_cloth(plan)
        self._assume_flat_cloth_layout(self.current_cloth_center_xy,
                                       "after assumed transport: "+plan["id"])

    def _assume_flat_cloth_layout(self, center, reason):
        # A CPU command trace has no cloth dynamics. Keep the unchanged real
        # layout fitter callable, while clearly separating these synthetic
        # observation points from native measured cloth / contact evidence.
        from scipy.spatial.transform import Rotation
        xy=self.fabric.reference[:,:2]-self.cfg["cloth"]["position_xy"]
        yaw=getattr(self,"current_cloth_yaw_rad",0.)
        rotation=Rotation.from_euler("z",yaw).as_matrix()[:2,:2]
        self._offline_cloth_positions=np.c_[xy@rotation.T+center,
            np.full(len(xy),self.cfg["table"]["height"]+
                    self.cfg["cloth"].get("collision_radius_m",.00075))]
        self.layout_assumptions.append({"time_s":self.frame*self.dt,
            "reason":reason,"center_xy_m":np.asarray(center).tolist(),
            "yaw_rad":yaw,"assumed_offline":True,"native_measurement":False})

    def event(self, phase, **values):
        self.phase = phase
        self.events.append({"phase": phase, "time_s": self.frame*self.dt, **values})

    def step(self, qs=None, velocities=None):
        if qs is not None:
            self.q = {side: np.asarray(qs[side]).copy() for side in SIDES}
        self.trace.append({"frame": self.frame, "time_s": self.frame*self.dt,
                           "phase": self.phase, "q": {s: q.copy() for s, q in self.q.items()},
                           "both_grasped": len(self.attached) == 2,
                           "velocity": None if velocities is None else
                           {s: np.asarray(v).copy() for s, v in velocities.items()}})
        self.frame += 1

    def snapshot(self, name):
        if name == "06_laid_over_soy":
            self._assume_flat_cloth_layout(np.asarray(self.cfg["soy"]["position_xy"]),
                                           "initial assumed lay over soy")

    def tcp(self, side):
        return self.planner.fk(side, self.q[side][:7])[0]

    def detach(self):
        self.attached = {}

    def corner_candidates(self):
        # Material/contact validation belongs to the native run; this audit
        # only follows the controller's real joint-command branches.
        return super().corner_candidates()

    def grasp_approach_report(self, preferred):
        """Exercise real open-descent refinements using an explicit assumption.

        FK target error is computed, but full-finger SDF contact is NOT queried.
        Delay the contact assumption until .7698 m to cover every allowed open
        descent attempt, even if real full-finger contact occurs higher. This
        is a conservative command-path branch, not recommended press depth or
        evidence that cloth tolerates it. The immediate pre-closure recheck
        uses the same at-least-one-finger-per-hand assumption, not all pad boxes.
        """
        targets=getattr(self,"grasp_approach_targets",{})
        hands={}
        for side in SIDES:
            tcp=self.tcp(side)
            target=targets.get(side)
            error=float(np.linalg.norm(tcp-target)) if target is not None else float("inf")
            assumed_shell=bool(target is not None and
                               target[2]<=ASSUMED_FULL_FINGER_CONTACT_TARGET_Z_M+1e-10)
            hands[side]={"ready":bool(error<=.0006 and assumed_shell),
                "tcp_m":tcp.tolist(),"target_tcp_m":None if target is None else target.tolist(),
                "computed_nominal_fk_target_error_m":error,
                "shell_contact_assumed":assumed_shell,
                "any_full_finger_contact_assumed":assumed_shell,
                "commanded_finger_positions_m":self.q[side][7:].tolist()}
        report={"ready":all(r["ready"] for r in hands.values()),
                "assumed_offline":True,"native_measurement":False,
                "contact_force_measured":False,
                "assumed_shell_contact_maximum_target_tcp_z_m":ASSUMED_FULL_FINGER_CONTACT_TARGET_Z_M,
                "assumption":"At least one complete coupled finger collision geometry per hand is assumed to contact its original material corner at the lowest permitted target height. No actual SDF or force is queried, and bilateral pad/material attachment remains a separate assumption.",
                "hands":hands}
        self.approach_assumptions.append({"time_s":self.frame*self.dt,
                                        "phase":self.phase,**deepcopy(report)})
        return report

    def grasp(self, phase, preferred, *, close=True, required=True):
        # close=False does not emit its phase before calling attach_grasp.
        # Record the requested branch without replacing controller behavior.
        self._offline_grasp_phase = phase
        return super().grasp(phase, preferred, close=close, required=required)


def trace_commands(config, resume_task_checkpoint=None):
    """Run the real controller with explicit contact/physics assumptions."""
    trace = CommandTrace(config)
    if resume_task_checkpoint is not None:
        # Trusted task-generated file; no scene or GPU is initialized.
        with Path(resume_task_checkpoint).open("rb") as stream:
            checkpoint=pickle.load(stream)
        for key,value in deepcopy(checkpoint["task_state"]).items():
            setattr(trace,key,value)
        pbd=checkpoint["particle_fields"]["pbd"]
        trace.fabric.reference=np.asarray(pbd["particles_info"]["pos_rest"]).reshape(-1,3).copy()
        trace._offline_cloth_positions=np.asarray(pbd["particles"]["pos"]).reshape(-1,3).copy()
        trace._offline_measured_layout=True
    trace._audit_initial_press_cycles=trace.press_cycles
    from robot_checkpoint import _metadata
    checkpoint_records = []
    def assumed_save(path, scene, handles, state):
        # Copy at save time, just as the real helper does: later grasps mutate
        # the controller's attachment dictionary in place.
        saved = _metadata(state)
        checkpoint_records.append({"filename": Path(path).name,
            "resume_stage": saved.get("resume_stage"),
            "command_frame": saved["frame"], "time_s": saved["frame"]*trace.dt,
            "attached_sides": sorted(saved.get("attached", {})),
            "plain_task_state_validated": True})
        return None
    # Keep the original grasp method's actual closing command and duration.
    # Only its runtime contact measurements are replaced for offline planning.
    # Legacy mode forces its bounded closed-height corrections and relief
    # branch. Contact-first mode instead assumes successful contact only after
    # the real open-descent refinement and narrow-opening feedback sweep.
    # Do not force a legacy failed initial grasp into the new required branch.
    def assumed_contact(*args, **kwargs):
        requested = trace._offline_grasp_phase
        is_right = args[0] is trace.h["robots"]["right"]
        side = "right" if is_right else "left"
        if config["robot_task"].get("contact_first_grasp",False):
            opening=np.asarray(kwargs["commanded_finger_opening"])
            threshold=trace.measured_finger_positions[side]
            attached=bool(np.all(opening<=threshold+1e-10))
            report={"attached":attached,"assumed_offline":True,"native_measurement":False,
                    "assumption":"Unchanged bilateral contact/material gates are assumed to pass at this narrow opening; no cloth or pad contact was evaluated.",
                    "assumed_attachment_openings_m":threshold.tolist(),
                    "commanded_finger_openings_m":opening.tolist()}
            trace.grasp_assumptions.append({"time_s":trace.frame*trace.dt,
                "phase":requested,"side":side,**deepcopy(report)})
            return report
        force_reposition_correction=trace._offline_measured_layout and "Reposition" in requested and (
            requested.endswith("Pinch the same wet diagonal corners") or
            requested.endswith("Confirm both wet-cloth contacts"))
        attached = requested != "03 Pinch opposite diagonal corners" and not (
            is_right and requested in ("10 Pinch the same wet diagonal corners",
                                       "10 Confirm both wet-cloth contacts")) and not (
            is_right and force_reposition_correction)
        return {"attached": attached, "assumed_offline": True}
    with patch("robot_grasp.attach_grasp", side_effect=assumed_contact), \
         patch("robot_grasp.pad_contact_report", return_value={
             "measured_offline": False, "assumed_offline": True, "both_fingers_contact": True}), \
         patch("robot_checkpoint.save_checkpoint", side_effect=assumed_save):
        if resume_task_checkpoint is None:
            trace.task()
        elif trace.resume_stage=="pressing":
            trace.press_cloth();trace.finish_pickup()
        elif trace.resume_stage=="after_dry_lift":
            trace.task(resume_after_lift=True)
        elif trace.resume_stage=="before_final_pickup":
            trace.finish_pickup()
        else:
            raise ValueError("Unsupported offline resume stage: "+str(trace.resume_stage))
    return trace, checkpoint_records


def audit(config, resume_task_checkpoint=None):
    source_paths=[Path(__file__).resolve(),Path(__file__).with_name("robot_wipe.py"),
                  Path(__file__).with_name("robot_geometry.py"),Path(__file__).with_name("robot_grasp.py"),
                  Path(__file__).with_name("grasp_approach_contact.py")]
    source_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
    trace, checkpoint_records = trace_commands(config,resume_task_checkpoint)
    planner = trace.planner
    minimum = {}
    phases = {}
    collisions = []
    evaluations = 0
    last_q = None
    max_velocity = np.zeros(7)
    min_limit_margin = float("inf")
    finger_values = set()
    held_spans = []
    wipe_spans = []
    relief_spans = []
    open_descent_frames = []
    closing_before_ready = []
    previous_state = None
    for state in trace.trace:
        q_flat = np.concatenate([state["q"][s] for s in SIDES])
        finger_values.update(round(float(x), 8) for s in SIDES for x in state["q"][s][7:])
        if state["velocity"] is not None:
            for side in SIDES:
                max_velocity = np.maximum(max_velocity, np.abs(state["velocity"][side][:7]))
        is_wiping = "wipe outward" in state["phase"] or "wipe return" in state["phase"]
        is_relief = state["phase"] == "10 Ease wet-cloth fold above table"
        is_open_descent=("descend with open fingers" in state["phase"] or
                         "lower open fingers to cloth contact" in state["phase"])
        if is_open_descent:
            open_descent_frames.append(state)
            if previous_state is not None and any(
                    np.any(state["q"][s][7:]<previous_state["q"][s][7:]-1e-10) for s in SIDES):
                closing_before_ready.append({"phase":state["phase"],"time_s":state["time_s"]})
        previous_state=state
        if state["both_grasped"] or is_wiping or is_relief:
            positions = {s: planner.fk(s, state["q"][s][:7])[0] for s in SIDES}
            span = float(np.linalg.norm(positions["right"]-positions["left"]))
            if state["both_grasped"]:
                held_spans.append(span)
            if is_wiping:
                wipe_spans.append(span)
            if is_relief:
                relief_spans.append(span)
        for side in SIDES:
            q = state["q"][side][:7]
            limits = planner.limits[side]
            min_limit_margin = min(min_limit_margin, float(np.min(np.r_[q-limits[:, 0], limits[:, 1]-q])))
        if last_q is not None and np.array_equal(q_flat, last_q):
            continue
        last_q = q_flat.copy()
        # Check exact left/right finger positions, including both closure ramps.
        for side in SIDES:
            for finger in (1, 2):
                planner.d.qpos[planner.m.joint(f"{side}_finger_joint{finger}").qposadr[0]] = state["q"][side][6+finger]
        distances = planner.clearance({s: state["q"][s][:7] for s in SIDES})
        evaluations += 1
        phase_minimum = phases.setdefault(state["phase"], {})
        for category, detail in distances.items():
            evidence = {**detail, "time_s": state["time_s"], "phase": state["phase"]}
            if category not in minimum or detail["distance"] < minimum[category]["distance"]:
                minimum[category] = evidence
            phase_minimum[category] = min(phase_minimum.get(category, float("inf")), detail["distance"])
            if detail["distance"] < -1e-6 and len(collisions) < 30:
                collisions.append({"category": category, **evidence})
    offsets = grasp_offsets(config)
    authored_span = float(np.linalg.norm(offsets["right"]-offsets["left"]))
    span_error = max((abs(span-authored_span) for span in held_spans), default=0.0)
    outward_error = max((max(0.0, span-authored_span) for span in held_spans), default=0.0)
    wipe_span_error = max((abs(span-authored_span) for span in wipe_spans), default=0.0)
    relief_span_error = max((abs(span-authored_span) for span in relief_spans), default=0.0)
    first_correction = any("Refine diagonal-corner" in event["phase"] for event in trace.events)
    final_correction = any("Refine wet-cloth" in event["phase"] for event in trace.events)
    fold_relief = any(event["phase"] == "10 Ease wet-cloth fold above table" for event in trace.events)
    dry_relief = any(event["phase"] == "03 Ease compressed cloth corner above table" for event in trace.events)
    contact_first=bool(config["robot_task"].get("contact_first_grasp",False))
    first_open_correction=any(e["phase"].startswith("02 Approach diagonal corners: lower open fingers") for e in trace.events)
    final_open_correction=any(e["phase"].startswith("09 Approach wet diagonal corners: lower open fingers") for e in trace.events)
    closure_rechecks=[e for e in trace.events if e["phase"].endswith("check cloth contact before closing")]
    contact_branch_ok=(bool(closure_rechecks) and all(e["approach"]["ready"] for e in closure_rechecks)
                       and bool(open_descent_frames) and not closing_before_ready
                       and (first_open_correction or resume_task_checkpoint is not None)
                       and final_open_correction)
    legacy_branch_ok=(first_correction or resume_task_checkpoint is not None) and final_correction and fold_relief
    sources_unchanged=all(hashlib.sha256(p.read_bytes()).hexdigest()==source_hashes[str(p)] for p in source_paths)
    return {"passed": not collisions and min_limit_margin > 0 and span_error < .001
                      and outward_error < 1e-6 and wipe_span_error < .001
                      and relief_span_error < 1e-6
                      and (contact_branch_ok if contact_first else legacy_branch_ok)
                      and trace.completed and sources_unchanged,
            "resume_task_checkpoint":str(resume_task_checkpoint) if resume_task_checkpoint else None,
            "controller": "RobotWipe.task / move / fingers / hold; original command generators",
            "scope":"Offline nominal joint-command trajectory only; no native cloth contact, grasp, absorption or task success is established.",
            "source_sha256":source_hashes,
            "source_files_unchanged_during_audit":sources_unchanged,
            "configuration":deepcopy(config),
            "contact_first_grasp":contact_first,
            "contact_first_audit_branch":"Conservative deepest allowed open descent; delay assumed full-finger shell contact until 0.7698 m. This is not a measured or recommended contact height.",
            "assumed_full_finger_contact_target_z_m":ASSUMED_FULL_FINGER_CONTACT_TARGET_Z_M,
            "contact_first_branch_checks_passed":contact_branch_ok if contact_first else None,
            "open_shell_contact_assumptions":trace.approach_assumptions,
            "grasp_gate_assumptions":trace.grasp_assumptions,
            "cloth_layout_assumptions":trace.layout_assumptions,
            "open_descent_command_frames":len(open_descent_frames),
            "open_descent_finger_range_m":[min(float(np.min(f["q"][s][7:])) for f in open_descent_frames for s in SIDES),
                                            max(float(np.max(f["q"][s][7:])) for f in open_descent_frames for s in SIDES)] if open_descent_frames else None,
            "finger_closing_during_open_descent":closing_before_ready,
            "initial_open_height_refinement_checked":first_open_correction,
            "final_open_height_refinement_checked":final_open_correction,
            "immediate_preclosure_rechecks_assumed_offline":closure_rechecks,
            "legacy_closed_height_and_relief_branches_required":not contact_first,
            "task_checkpoints_observed": checkpoint_records,
            "servo_commands_assumed_offline": trace.servo_commands,
            "dof_measurements_assumed_offline": trace.dof_measurements,
            "final_pad_contacts_assumed_offline": trace.final_pad_contacts,
            "nominal_task_complete": trace.completed,
            "executed_reposition_ids_assumed_offline": getattr(trace,"executed_reposition_ids",[]),
            "reposition_history_assumed_offline": getattr(trace,"reposition_history",[]),
            "layout_tracking_history_assumed_offline":getattr(trace,"layout_tracking_history",[]),
            "current_cloth_yaw_rad_assumed_offline":getattr(trace,"current_cloth_yaw_rad",0.),
            "initial_height_correction_checked": first_correction,
            "final_height_correction_checked": final_correction,
            "fold_relief_branch_checked": fold_relief,
            "failed_dry_hand_relief_branch_checked":dry_relief,
            "failed_dry_hand_relief_scope":"Not forced by this audit. It checks the nominal successful dry feedback grasp; no claim is made about the optional one-hand 1 mm relief path or its physical grasp outcome.",
            "fold_relief_command_frames": len(relief_spans),
            "fold_relief_tcp_spacing_range_m": [min(relief_spans), max(relief_spans)] if relief_spans else None,
            "maximum_fold_relief_tcp_spacing_error_m": relief_span_error,
            "command_frames": len(trace.trace), "unique_states_checked": evaluations,
            "command_step_seconds": trace.dt, "total_duration_seconds": len(trace.trace)*trace.dt,
            "start_time_seconds":trace.trace[0]["time_s"] if trace.trace else trace.frame*trace.dt,
            "end_time_seconds":trace.frame*trace.dt,
            "press_cycles_checked": trace.press_cycles,
            "press_cycles_already_completed":trace._audit_initial_press_cycles,
            "new_press_cycles_checked":trace.press_cycles-trace._audit_initial_press_cycles,
            "outward_wipe_strokes_checked": sum("wipe outward" in e["phase"] for e in trace.events),
            "return_wipe_strokes_checked": sum("wipe return" in e["phase"] for e in trace.events),
            "max_ik_position_error_m": trace.max_ik_error,
            "max_feedforward_joint_speed_rad_s": max_velocity.tolist(),
            "minimum_joint_limit_margin_rad": min_limit_margin,
            "grasp_offsets_xy_m": {s: value.tolist() for s, value in offsets.items()},
            "authored_material_point_spacing_m": authored_span,
            "held_tcp_spacing_range_m": [min(held_spans), max(held_spans)] if held_spans else None,
            "maximum_held_tcp_spacing_error_m": span_error,
            "maximum_commanded_outward_spacing_error_m": outward_error,
            "wipe_tcp_spacing_range_m": [min(wipe_spans), max(wipe_spans)] if wipe_spans else None,
            "maximum_wipe_tcp_spacing_error_m": wipe_span_error,
            "finger_joint_extrema_m": [min(finger_values), max(finger_values)],
            "minimum_clearance_m": minimum, "phase_minimum_clearance_m": phases,
            "collisions": collisions, "events": trace.events,
            "limitations": ["Offline nominal command path; tracking errors and cloth dynamics are not included.",
                            "Conservative full-finger shell arrival at target TCP <=0.7698 m and its immediate recheck assume at least one actual finger geometry per hand; no SDF query or contact-force measurement is performed offline.",
                            "The complete finger collision geometry used for arrival is distinct from the unchanged bilateral pad/material attachment gate, whose narrow-opening success is separately assumed.",
                            "Successful physical contact gates are assumed only to reach later path phases.",
                            "Fresh cloth layouts are assumed flat at commanded centers; a resumed trace uses measured checkpoint cloth for its first fit. After transport, later/final fits assume rigidly carried cloth. Native execution remeasures actual cloth.",
                            "Contact-first audit covers its successful feedback-closure path; neither optional failed-dry-hand 1 mm relief nor wet fold relief is forced or validated by this result.",
                            "All maximum press cycles are checked, even if runtime absorption finishes earlier.",
                            "TCP spacing does not measure actual cloth strain or attached-node material spacing.",
                            "Original MJCF convex geometry may differ from Genesis decomposition."]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config_robot_wipe.json"))
    parser.add_argument("--output", type=Path, default=Path(__file__).parent/"output/robot_wipe_path_clearance.json")
    parser.add_argument("--resume-task-checkpoint",type=Path,
                        help="Trusted local checkpoint: audit saved joints and measured cloth, without GPU")
    args = parser.parse_args()
    result = audit(json.loads(args.config.read_text()),args.resume_task_checkpoint)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps({k: result[k] for k in ("passed", "command_frames", "unique_states_checked",
          "total_duration_seconds", "press_cycles_checked", "max_ik_position_error_m",
          "max_feedforward_joint_speed_rad_s", "minimum_joint_limit_margin_rad", "minimum_clearance_m",
          "collisions")}, indent=2))
    raise SystemExit(0 if result["passed"] else 1)
