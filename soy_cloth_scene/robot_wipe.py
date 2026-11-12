"""Dual Franka feedback-controlled cloth pick/carry/press/pick task.

Robot joints use Genesis PD dynamics. Only local cloth nodes actually touching
both finger pads can receive a no-slip link constraint. Interior cloth, free
liquid, finite absorption reservoirs and wet mass remain simulated.
"""
from __future__ import annotations
from copy import deepcopy
import csv
import json
import time
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation, Slerp

SIDES = ("left", "right")

def arr(value):
    return value.detach().cpu().numpy() if hasattr(value, "detach") else np.asarray(value)

def smooth(u):
    return u*u*u*(10+u*(-15+6*u))

class StepLimit(Exception):
    pass

class RobotWipe:
    def __init__(self, scene, handles, config, args, output):
        from robot_geometry import Planner
        self.scene, self.h, self.cfg, self.args, self.out = scene, handles, config, args, output
        self.planner = Planner(config)
        self.dt = config["simulation"]["dt"]
        self.q = {s:arr(handles["robots"][s].get_dofs_position(handles["robot_dofs"][s])).copy() for s in SIDES}
        self.initial_q = {s:q.copy() for s,q in self.q.items()}
        self.frame, self.phase, self.rows, self.events = 0, "Initialize", [], []
        self.started = time.monotonic()
        self.attached = {}
        self.initial_mass = handles["coupling"].metrics()["initial_liquid_mass_kg"]
        self.max_mass_error = self.max_loading_error = self.max_saturation = 0.
        self.max_tcp_error = self.max_ik_error = 0.
        self.minimum_cloth_z = float("inf")
        self.rigid_collision_samples = 0
        self.press_contacts = []
        self.press_cycles = 0
        self.completed = False
        from cloth_metrics import ClothMetrics
        self.direction_limits = ({direction: config["cloth"][direction+"_max_extension"]
                                  for direction in ("warp", "weft", "bias")}
                                 if config["cloth"].get("material_profile") == "plush_cotton_pdf" else None)
        self.fabric = ClothMetrics(arr(handles["cloth"].get_particles_pos()), handles["coupling"].faces,
                                   direction_limits=self.direction_limits)
        self.max_edge_extension = self.max_p99_extension = 0.
        self.fabric_stages = {}
        self.initial_grasp_material = {}
        self.writer = None
        self.stride = max(1, round(1/(15*self.dt)))
        if args.record:
            import imageio.v2 as imageio
            self.writer = imageio.get_writer(output/"dual_arm_wipe.mp4", fps=1/(self.stride*self.dt), quality=8, macro_block_size=2)
        self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 21)
        self.smallfont = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 17)

    def tcp(self, side):
        from robot_grasp import tcp_position
        return np.asarray(tcp_position(self.h["robots"][side]))

    def event(self, phase, **values):
        self.phase = phase
        record = {"phase":phase, "time_s":self.frame*self.dt, **values}
        self.events.append(record)
        print(json.dumps(record), flush=True)
        (self.out/"events.json").write_text(json.dumps(self.events, indent=2)+"\n")

    def step(self, qs=None, velocities=None):
        if self.args.steps and self.frame >= self.args.steps:
            raise StepLimit()
        if not self.args.headless and not self.scene.viewer.is_alive():
            raise StepLimit()
        if qs is not None:
            self.q = {s:np.asarray(qs[s]).copy() for s in SIDES}
        for s in SIDES:
            robot = self.h["robots"][s]
            if velocities is None:
                robot.control_dofs_position(self.q[s], self.h["robot_dofs"][s])
            else:
                robot.control_dofs_position_velocity(self.q[s], velocities[s], self.h["robot_dofs"][s])
        self.scene.step()
        self.frame += 1
        if self.frame % 25 == 0:
            m = self.h["coupling"].metrics()
            cp = arr(self.h["cloth"].get_particles_pos())
            if not np.isfinite(cp).all():
                raise RuntimeError("Nonfinite cloth state")
            if cp[:,2].max() > self.h.get("numerical_resolution", {}).get("max_height", float("inf")):
                raise RuntimeError("Cloth left the validated height range")
            active = arr(self.h["soy"].get_particles_active()).astype(bool)
            liquid = arr(self.h["soy"].get_particles_pos())[active]
            if not np.isfinite(liquid).all() or (len(liquid) and liquid[:,2].min()<self.cfg["table"]["height"]-.002):
                raise RuntimeError("Free liquid became nonfinite or penetrated the table")
            self.minimum_cloth_z = min(self.minimum_cloth_z, float(cp[:,2].min()))
            strain = self.fabric.strain(cp)
            self.max_edge_extension = max(self.max_edge_extension, strain["edge_extension_max"])
            self.max_p99_extension = max(self.max_p99_extension, strain["edge_extension_p99"])
            self.max_mass_error = max(self.max_mass_error, abs(m["mass_balance_error_kg"]))
            self.max_loading_error = max(self.max_loading_error, abs(m["cloth_loading_error_kg"]))
            self.max_saturation = max(self.max_saturation, m["maximum_saturation"])
            errors = {s:float(np.linalg.norm(self.planner.fk(s,self.q[s][:7])[0]-self.tcp(s))) for s in SIDES}
            self.max_tcp_error = max(self.max_tcp_error, *errors.values())
            pairs = [(self.h["robots"][s],self.h["table"]) for s in SIDES]
            pairs += [(self.h["robots"]["left"],self.h["robots"]["right"])]
            self.rigid_collision_samples += sum(len(a.get_contacts(with_entity=b)["geom_a"])>0 for a,b in pairs)
            self.rows.append({"time_s":self.frame*self.dt, "phase":self.phase,
                "free_liquid_g":1000*m["active_liquid_mass_kg"], "absorbed_g":1000*m["absorbed_mass_kg"],
                "active_particles":m["active_liquid_count"], "max_saturation":m["maximum_saturation"],
                "cloth_min_z_m":float(cp[:,2].min()), "left_tcp_error_m":errors["left"], "right_tcp_error_m":errors["right"], **strain})
            if self.max_mass_error > 1e-7 or self.max_saturation > 1.000001:
                raise RuntimeError("Absorption conservation/capacity check failed")
            if self.frame % 250 == 0:
                print(f"robot t={self.frame*self.dt:.1f}s phase={self.phase} free={m['active_liquid_mass_kg']*1000:.3f}g tcp={max(errors.values())*1000:.2f}mm wall={time.monotonic()-self.started:.1f}s", flush=True)
        if self.writer is not None and self.frame % self.stride == 0:
            self.writer.append_data(self.render_frame())

    def render_frame(self):
        rgb,*_ = self.h["cameras"]["overview"].render()
        frame=Image.fromarray(np.asarray(rgb))
        # An inset shows cloth contact while the main view keeps both arms visible.
        rgb,*_ = self.h["cameras"]["detail"].render()
        w,h = frame.size
        inset=Image.fromarray(np.asarray(rgb)).resize((round(w*.36),round(h*.36)))
        frame.paste(inset,(w-inset.width-10,h-inset.height-10))
        draw=ImageDraw.Draw(frame)
        draw.rectangle((0,0,w,76),fill=(22,29,38))
        draw.text((16,10), self.phase, font=self.font,fill="white")
        m=self.h["coupling"].metrics()
        draw.text((16,43),f"t={self.frame*self.dt:.1f}s   Table: {m['active_liquid_mass_kg']*1000:.2f} g   In cloth: {m['absorbed_mass_kg']*1000:.2f} g",font=self.smallfont,fill=(214,224,234))
        return np.asarray(frame)

    def snapshot(self, name):
        cp = arr(self.h["cloth"].get_particles_pos())
        self.fabric_stages[name] = self.fabric.measure(cp, self.cfg["table"]["height"])
        (self.out/"cloth_shape_measurements.json").write_text(json.dumps(self.fabric_stages, indent=2)+"\n")
        np.savez_compressed(self.out/f"{name}_cloth.npz", positions=cp,
                           reference=self.fabric.reference, faces=self.fabric.faces)
        Image.fromarray(self.render_frame()).save(self.out/f"{name}.png")
        rgb,*_=self.h["cameras"]["detail"].render()
        Image.fromarray(np.asarray(rgb)).save(self.out/f"{name}_detail.png")
        rgb,*_=self.h["cameras"]["top"].render()
        Image.fromarray(np.asarray(rgb)).save(self.out/f"{name}_top.png")

    def hold(self, seconds):
        for _ in range(round(seconds/self.dt)): self.step()

    def fingers(self, phase, opening, duration=.6):
        self.event(phase)
        starts={s:self.q[s].copy() for s in SIDES}
        n=round(duration/self.dt)
        for i in range(n):
            u=smooth((i+1)/n)
            qs={s:np.r_[starts[s][:7],starts[s][7:]+u*(opening-starts[s][7:])] for s in SIDES}
            self.step(qs)
        self.hold(.25)

    def move(self, phase, targets, duration, *, orientation=None):
        from robot_geometry import DOWN
        desired_rotation=DOWN if orientation is None else orientation
        self.event(phase, targets_m={s:np.asarray(p).tolist() for s,p in targets.items()})
        starts={s:self.planner.fk(s,self.q[s][:7]) for s in SIDES}
        slerp={s:Slerp([0,1],Rotation.concatenate([starts[s][1],desired_rotation])) for s in SIDES}
        knots=max(2,round(duration/.08))
        paths={s:[self.q[s][:7].copy()] for s in SIDES}
        for k in range(1,knots+1):
            u=smooth(k/knots)
            for s in SIDES:
                p=starts[s][0]+u*(np.asarray(targets[s])-starts[s][0])
                q,err,ang=self.planner.solve(s,p,slerp[s](u),paths[s][-1])
                self.max_ik_error=max(self.max_ik_error,err)
                if err>.001 or ang>.02:
                    raise RuntimeError(f"Unreachable {phase} {s}: {err:.4f}m / {ang:.4f}rad")
                paths[s].append(q)
        n=round(duration/self.dt)
        for i in range(n):
            coordinate=(i+1)*knots/n
            a=min(int(coordinate),knots-1); frac=coordinate-a
            qs={s:np.r_[(1-frac)*paths[s][a]+frac*paths[s][a+1],self.q[s][7:]] for s in SIDES}
            velocities={s:np.r_[(paths[s][a+1]-paths[s][a])/(duration/knots),0,0] for s in SIDES}
            self.step(qs,velocities)
        self.hold(.20)

    def grasp(self, phase, preferred, *, close=True, required=True):
        from robot_grasp import attach_grasp
        if close and self.cfg["robot_task"].get("contact_first_grasp", False):
            arrival = self.grasp_approach_report(preferred)
            self.event(phase+": check cloth contact before closing", approach=arrival)
            if not arrival["ready"]:
                raise RuntimeError("Gripper closure blocked: open fingers have not reached the cloth corners")
        wet = getattr(self,"wet_grasp_active",False)
        feedback = self.cfg["robot_task"].get(
            "wet_grasp_feedback_closure" if wet else "dry_grasp_feedback_closure", False)
        preserve_pinch = wet or feedback
        if close and feedback:
            return self.close_grasp_with_feedback(phase, preferred, required=required)
        if close:
            self.fingers(phase,0.0,.8)
        reports={}
        for s in SIDES:
            if s in self.attached:
                reports[s]=self.attached[s]
                continue
            # Re-evaluate actual local material after closure: cloth can bunch
            # or slide a little while the fingers approach. The geometric gate
            # still requires both actual pads, not merely a commanded target.
            cp=arr(self.h["cloth"].get_particles_pos())
            near=np.flatnonzero(np.linalg.norm(cp-self.tcp(s),axis=1)<.025)
            # Select the same small material corner throughout the task. A
            # spatially nearby fold from another part of the sheet is not a
            # valid substitute for that corner.
            near=np.intersect1d(near,preferred[s])
            report=attach_grasp(self.h["robots"][s],self.h["cloth"],near,
                gate_mode="bilateral_patch", commanded_finger_opening=self.q[s][7:].copy(),
                reference_positions=self.fabric.reference,
                max_fixed_pair_extension=self.cfg["robot_task"].get("maximum_fixed_pair_extension",.02),
                minimum_bound_per_finger=self.cfg["robot_task"].get("minimum_bound_per_finger",1),
                attachment_mode="hand",
                preserve_contact_geometry=preserve_pinch)
            contacts=self.h["robots"][s].get_contacts()
            report["native_rigid_contacts"]={k:arr(v).tolist() for k,v in contacts.items()}
            reports[s]=report
            if report.get("attached",False):
                self.attached[s]=report
                if preserve_pinch:
                    # Preserve the measured pinch width after establishing the
                    # ideal no-slip hand constraint. Continuing to command zero
                    # would sweep moving finger solids into those fixed points.
                    finger_indices=self.h["robot_dofs"][s][-2:]
                    measured=arr(self.h["robots"][s].get_dofs_position(finger_indices))
                    self.q[s][7:]=measured
                    prefix = "wet" if wet else "dry"
                    hold_kp=self.cfg["robot_task"].get(prefix+"_grasp_hold_position_gain",10000.)
                    hold_kv=self.cfg["robot_task"].get(prefix+"_grasp_hold_velocity_gain",100.)
                    self.h["robots"][s].set_dofs_kp(np.full(2,hold_kp),finger_indices)
                    self.h["robots"][s].set_dofs_kv(np.full(2,hold_kv),finger_indices)
                    report["held_finger_position_targets_m"]=measured.tolist()
                    report["hold_finger_servo"]={"position_gain":hold_kp,"velocity_gain":hold_kv}
        failed=[s for s in SIDES if not reports[s].get("attached",False)]
        if failed:
            self.event("Grasp contact requires adjustment", contacts=reports)
            if required:
                self.snapshot("grasp_failure")
                raise RuntimeError("Cloth grasp contact failed for "+", ".join(failed))
            return False
        self.event(phase+" confirmed", contacts=reports)
        self.hold(.2)
        return True

    def close_grasp_with_feedback(self, phase, preferred, *, required):
        """Stop each jaw pair at its first existing contact/material-safe grasp.

        The former open-loop zero-opening command could compress adjacent yarn
        rows before the gate was checked. Small physical servo increments let
        the unchanged gate inspect intermediate contacts. An attached hand
        keeps its measured opening while the other hand continues closing.
        No cloth positions or contact/strain thresholds are modified.
        """
        self.event(phase+": close with contact feedback")
        if self.grasp(phase, preferred, close=False, required=False):
            return True
        increment = 0.0001
        coarse = self.cfg["robot_task"].get("contact_first_grasp", False)
        remaining = max(float(np.max(self.q[s][7:])) for s in SIDES if s not in self.attached)
        for _ in range(int(np.ceil(max(remaining,0.)/increment))):
            starts = {s:self.q[s].copy() for s in SIDES}
            targets = {}
            for s in SIDES:
                opening = starts[s][7:]
                if s in self.attached:
                    targets[s] = opening.copy()
                elif coarse:
                    # This is a slow, physical contact sweep at cloth height,
                    # not an assumed empty gap. Inspect the unchanged grasp
                    # gate after every step; retain fine closure below 2 mm.
                    targets[s] = np.where(opening > .002+1e-10,
                        np.maximum(opening-.001,.002), np.maximum(opening-increment,0.))
                else:
                    targets[s] = np.maximum(opening-increment,0.)
            n = max(1,round(.05/self.dt))
            for i in range(n):
                u = smooth((i+1)/n)
                self.step({s:np.r_[starts[s][:7],starts[s][7:]+u*(targets[s]-starts[s][7:])]
                           for s in SIDES})
            self.hold(.05)
            if self.grasp(phase, preferred, close=False, required=False):
                return True
            if all(s in self.attached or np.max(self.q[s][7:]) <= 0. for s in SIDES):
                break
        return self.grasp(phase, preferred, close=False, required=required)

    def grasp_approach_report(self, preferred):
        """Measured arrival/contact-shell evidence, never a grasp certificate."""
        from robot_grasp import _pad_distance, cloth_contact_radius
        from grasp_approach_contact import native_finger_contact_report
        targets = getattr(self, "grasp_approach_targets", {})
        cp = arr(self.h["cloth"].get_particles_pos())
        threshold = cloth_contact_radius(self.h["cloth"])+.00005
        reports = {}
        for side in SIDES:
            if side in self.attached:
                reports[side] = {"ready": True, "already_attached": True}
                continue
            robot = self.h["robots"][side]
            points = cp[preferred[side]]
            distances = {name: float(_pad_distance(points, robot.get_link(name)).min())
                         if len(points) else float("inf")
                         for name in ("left_finger", "right_finger")}
            native_contact = native_finger_contact_report(robot,self.h["cloth"],preferred[side])
            tcp = self.tcp(side)
            error = (float(np.linalg.norm(tcp-targets[side]))
                     if side in targets else float("inf"))
            reports[side] = {"ready": bool(error <= .0006 and native_contact["any_finger_contact"]),
                "tcp_m": tcp.tolist(), "target_error_m": error,
                "material_corner_pad_distances_m": distances,
                "native_finger_contact": native_contact,
                "measured_finger_positions_m": arr(robot.get_dofs_position(
                    self.h["robot_dofs"][side][-2:])).tolist(),
                "commanded_finger_positions_m": self.q[side][7:].tolist()}
        return {"ready": all(r["ready"] for r in reports.values()),
                "contact_shell_distance_m": threshold,
                "arrival_contact_source": "actual coupled finger collision SDF; at least one finger per hand",
                "hands": reports}

    def open_grasp_approach(self, phase, center, pinch_z, duration, *, wet=False):
        """Keep an open aperture until actual pads reach original cloth corners."""
        if self.attached:
            raise RuntimeError("Open approach requires both hands to be released")
        key = "wet_pregrasp_finger_opening_m" if wet else "pregrasp_finger_opening_m"
        requested = float(self.cfg["robot_task"].get(key, .010))
        # Never pre-close a wider hand in the air. The production initial pose
        # and release aperture already match this visible 20 mm distal gap.
        opening = max(requested, *(float(np.max(self.q[s][7:])) for s in SIDES))
        if any(np.min(self.q[s][7:]) < opening-1e-9 for s in SIDES):
            self.fingers(phase+": open fingers before descent", opening, .5)
        else:
            self.event(phase+": keep fingers open", opening_per_finger_m=opening)
        preferred = self.corner_candidates()
        mover = self.move_layout if wet else self.move
        for attempt in range(4):
            z = max(.7698, pinch_z-attempt*.0002)
            self.grasp_approach_targets = self.layout_targets(center,z)
            mover(phase+(": descend with open fingers" if attempt == 0 else
                         ": lower open fingers to cloth contact"),
                  self.grasp_approach_targets, duration if attempt == 0 else .4)
            self.hold(.15)
            report = self.grasp_approach_report(preferred)
            self.event(phase+": measured open-finger arrival", approach=report)
            if report["ready"]:
                return preferred
        self.snapshot("open_approach_failed")
        raise RuntimeError("Open fingers did not reach both material corners; refusing to close or lift")

    def grasp_dry_corners(self, preferred):
        if self.grasp("03 Pinch opposite diagonal corners",preferred,required=False):
            return
        # A corner gathered at tabletop height can wedge under a finger.
        # Ease only an unsuccessful hand by 1 mm, keeping a successful hand
        # stationary and its measured pinch width unchanged. This is physical
        # contact adjustment; the transport lift still needs both strict gates.
        targets={side:self.planner.fk(side,self.q[side][:7])[0].copy() for side in SIDES}
        for side in SIDES:
            if side not in self.attached:
                indices=self.h["robot_dofs"][side][-2:]
                self.q[side][7:]=arr(self.h["robots"][side].get_dofs_position(indices))
                targets[side][2]+=.001
        self.move("03 Ease compressed cloth corner above table",targets,.6)
        self.hold(.3)
        self.grasp("03 Confirm relieved diagonal-corner contacts",preferred,close=False)

    def detach(self):
        from robot_grasp import detach
        detach(self.h["cloth"])
        self.attached={}
        # Holding a detected pinch uses a stiff servo. Once all cloth anchors
        # are released, restore the configured normal actuator gains before
        # opening the fingers or pressing/wiping the cloth on the table.
        robot_cfg=self.cfg.get("robots",{})
        kp=robot_cfg.get("finger_position_gain",2000.)
        kv=robot_cfg.get("finger_velocity_gain",40.)
        for side in SIDES:
            indices=self.h["robot_dofs"][side][-2:]
            self.h["robots"][side].set_dofs_kp(np.full(2,kp),indices)
            self.h["robots"][side].set_dofs_kv(np.full(2,kv),indices)
        self.event("07 Restore normal finger servo for release and pressing",
                   finger_position_gain=kp,finger_velocity_gain=kv)

    def corner_candidates(self):
        from robot_geometry import grasp_targets
        targets = grasp_targets(self.cfg, np.asarray(self.cfg["cloth"]["position_xy"]), 0.)
        radius = self.cfg["robot_task"].get("material_patch_radius_m", .022)
        return {s:np.flatnonzero(np.linalg.norm(self.fabric.reference[:,:2]-p[:2], axis=1)<=radius)
                for s,p in targets.items()}

    def save_task_checkpoint(self, name, resume_stage):
        from robot_checkpoint import save_checkpoint
        self.ensure_press_control_state()
        self.resume_stage=resume_stage
        fields=("q","initial_q","frame","phase","rows","events","initial_mass",
            "max_mass_error","max_loading_error","max_saturation","max_tcp_error","max_ik_error",
            "minimum_cloth_z","rigid_collision_samples","press_contacts",
            "max_edge_extension","max_p99_extension","fabric_stages","initial_grasp_material",
            "attached","resume_stage","current_cloth_center_xy","current_wipe_translation_xy",
            "executed_reposition_ids","reposition_history","current_cloth_yaw_rad","layout_tracking_history",
            "executed_wipe_stroke_ids","wipe_stroke_history")
        state={k:getattr(self,k) for k in fields}
        state["press_cycles"]=getattr(self,"press_cycles",0)
        if resume_stage=="audit_only_complete":
            # This terminal snapshot is evidence for independent ledger audits,
            # not an additional supported controller resume stage.
            state.update(completed=self.completed,
                         final_pad_contacts=deepcopy(self.final_pad_contacts),
                         bound_before_lift=self.bound_before_lift.copy())
        return save_checkpoint(self.out/name,self.scene,self.h,state)

    def ensure_press_control_state(self):
        """Restore-compatible targets, separate from the unchanged soy layout.

        The center describes the commanded same-corner target layout, not a
        measured deforming-cloth centroid. Reposition events also record the
        actual TCP positions. Old checkpoints omit these fields intentionally.
        """
        defaults={"current_cloth_center_xy":self.cfg["soy"]["position_xy"],
                  "current_wipe_translation_xy":self.cfg["robot_task"].get("wipe_translation_xy_m",[-.04,0.])}
        for name,default in defaults.items():
            value=np.asarray(getattr(self,name,default),dtype=float)
            if value.shape!=(2,) or not np.isfinite(value).all():
                raise ValueError("Invalid press target state: "+name)
            setattr(self,name,value.copy())
        if not hasattr(self,"executed_reposition_ids"):
            self.executed_reposition_ids=[]
        if not hasattr(self,"reposition_history"):
            self.reposition_history=[]
        if not hasattr(self,"executed_wipe_stroke_ids"):
            self.executed_wipe_stroke_ids=[]
        if not hasattr(self,"wipe_stroke_history"):
            self.wipe_stroke_history=[]
        self.current_cloth_yaw_rad=float(getattr(self,"current_cloth_yaw_rad",0.))
        if not hasattr(self,"layout_tracking_history"):
            self.layout_tracking_history=[]
        if not np.isfinite(self.current_cloth_yaw_rad) or not isinstance(self.layout_tracking_history,list):
            raise ValueError("Invalid planar cloth layout in checkpoint")
        if (not isinstance(self.executed_reposition_ids,list)
                or any(not isinstance(i,str) for i in self.executed_reposition_ids)
                or len(set(self.executed_reposition_ids))!=len(self.executed_reposition_ids)
                or not isinstance(self.reposition_history,list)):
            raise ValueError("Invalid reposition history in checkpoint")
        if (not isinstance(self.executed_wipe_stroke_ids,list)
                or any(not isinstance(i,str) for i in self.executed_wipe_stroke_ids)
                or len(set(self.executed_wipe_stroke_ids))!=len(self.executed_wipe_stroke_ids)
                or not isinstance(self.wipe_stroke_history,list)):
            raise ValueError("Invalid wipe stroke history in checkpoint")

    def layout_targets(self, center, z):
        """Rotate world TCP offsets, leaving authored material candidates intact."""
        from robot_geometry import grasp_targets
        targets=grasp_targets(self.cfg,center,z)
        yaw=getattr(self,"current_cloth_yaw_rad",0.)
        if yaw:
            rotation=Rotation.from_euler("z",yaw).as_matrix()[:2,:2]
            center=np.asarray(center,dtype=float)
            targets={s:np.r_[center+rotation@(p[:2]-center),p[2]] for s,p in targets.items()}
        return targets

    def layout_orientation(self):
        from robot_geometry import DOWN
        yaw=getattr(self,"current_cloth_yaw_rad",0.)
        return Rotation.from_euler("z",yaw)*DOWN if yaw else DOWN

    def move_layout(self, phase, targets, duration):
        self.move(phase,targets,duration,orientation=self.layout_orientation())

    def prepare_wet_grasp_layout(self, *, phase_prefix=None):
        """Observe a flat unbound sheet, then align empty hands above it."""
        self.ensure_press_control_state()
        start=self.cfg["robot_task"].get("wet_grasp_track_layout_from_cycle")
        if start is None or start is False:
            return False
        if type(start) is not int or start<0:
            raise ValueError("wet_grasp_track_layout_from_cycle must be a nonnegative integer or disabled")
        if self.press_cycles<start:
            return False
        if self.attached:
            raise RuntimeError("Cannot refit a cloth layout while any hand is attached")
        from cloth_layout import fit_planar_cloth_layout
        from robot_geometry import FINGERTIP_BELOW_TCP
        fit=fit_planar_cloth_layout(self.fabric.reference,arr(self.h["cloth"].get_particles_pos()),
                                   self.cfg["cloth"]["position_xy"],self.cfg["table"]["height"])
        old_center=self.current_cloth_center_xy.copy();old_yaw=self.current_cloth_yaw_rad
        safe_z=max(self.cfg["table"]["height"]+.04,
                   fit["cloth_z_range_m"][1]+FINGERTIP_BELOW_TCP+.01)
        if safe_z>self.cfg["table"]["height"]+.09:
            raise ValueError("Planar layout alignment exceeds the bounded approach height")
        def phase(text):return "09 "+text if phase_prefix is None else phase_prefix+": "+text
        self.event(phase("Measure current planar material layout"),fit=fit,
                   previous_center_xy_m=old_center.tolist(),previous_yaw_rad=old_yaw)
        # Raising first avoids sweeping the fingertips sideways through cloth.
        self.move_layout(phase("Raise empty grippers before layout alignment"),
                         self.layout_targets(old_center,safe_z),.8)
        self.current_cloth_center_xy=np.asarray(fit["center_xy_m"],dtype=float)
        self.current_cloth_yaw_rad=fit["yaw_rad"]
        self.move_layout(phase("Align empty grippers with measured cloth layout"),
                         self.layout_targets(self.current_cloth_center_xy,safe_z),1.)
        record={"time_s":self.frame*self.dt,"after_completed_cycle":self.press_cycles,
                "previous_center_xy_m":old_center.tolist(),"previous_yaw_rad":old_yaw,
                "fit":fit,"alignment_tcp_height_m":safe_z,
                "measured_aligned_tcp_m":{s:self.tcp(s).tolist() for s in SIDES}}
        self.layout_tracking_history.append(record)
        self.event(phase("Planar layout aligned with empty grippers"),**record)
        return True

    def reposition_schedule(self):
        schedule=self.cfg["robot_task"].get("cloth_reposition_schedule",[])
        if not isinstance(schedule,list):
            raise ValueError("cloth_reposition_schedule must be a list")
        plans=[]
        for index,item in enumerate(schedule):
            if not isinstance(item,dict):
                raise ValueError("Each cloth reposition plan must be a mapping")
            after=item.get("after_cycle")
            if type(after) is not int or after<0:
                raise ValueError("Reposition after_cycle must be a nonnegative integer")
            vectors={}
            for key in ("center_offset_xy_m","wipe_translation_xy_m"):
                value=np.asarray(item.get(key),dtype=float)
                if value.shape!=(2,) or not np.isfinite(value).all():
                    raise ValueError("Reposition "+key+" must be a finite XY pair")
                vectors[key]=value.tolist()
            plans.append({"id":f"reposition_{index+1}","after_cycle":after,**vectors})
        # Appending a new plan is allowed after resume. Rewriting an already
        # completed list entry would make its saved identity ambiguous.
        history={entry["plan"]["id"]:entry["plan"] for entry in self.reposition_history}
        for plan in plans:
            if plan["id"] in self.executed_reposition_ids and history.get(plan["id"])!=plan:
                raise ValueError("An executed reposition plan changed: "+plan["id"])
        return plans

    def execute_due_repositions(self, plans):
        changed=False
        for plan in plans:
            if (plan["id"] in self.executed_reposition_ids
                    or plan["after_cycle"]>self.press_cycles):
                continue
            if not self.h["coupling"].metrics()["active_liquid_count"]:
                break
            self.reposition_cloth(plan)
            changed=True
        return changed

    def wipe_stroke_schedule(self):
        """Validate explicit world-XY stroke changes at completed-cycle boundaries."""
        schedule=self.cfg["robot_task"].get("wipe_stroke_schedule",[])
        if not isinstance(schedule,list):
            raise ValueError("wipe_stroke_schedule must be a list")
        plans=[]
        previous_after=-1
        for index,item in enumerate(schedule):
            if not isinstance(item,dict):
                raise ValueError("Each wipe stroke plan must be a mapping")
            after=item.get("after_cycle")
            if type(after) is not int or after<0 or after<=previous_after:
                raise ValueError("Wipe stroke after_cycle must be strictly increasing nonnegative integers")
            stroke=np.asarray(item.get("translation_xy_m"),dtype=float)
            if stroke.shape!=(2,) or not np.isfinite(stroke).all():
                raise ValueError("Wipe stroke translation_xy_m must be a finite XY pair")
            plans.append({"id":f"wipe_stroke_{index+1}","after_cycle":after,
                          "translation_xy_m":stroke.tolist()})
            previous_after=after
        history={}
        for entry in self.wipe_stroke_history:
            if not isinstance(entry,dict) or not isinstance(entry.get("plan"),dict):
                raise ValueError("Invalid wipe stroke history entry in checkpoint")
            plan=entry["plan"]
            if not isinstance(plan.get("id"),str) or plan["id"] in history:
                raise ValueError("Invalid wipe stroke plan identity in checkpoint")
            history[plan["id"]]=plan
        if set(history)!=set(self.executed_wipe_stroke_ids):
            raise ValueError("Wipe stroke execution IDs differ from saved history")
        configured={plan["id"]:plan for plan in plans}
        for plan_id in self.executed_wipe_stroke_ids:
            if configured.get(plan_id)!=history[plan_id]:
                raise ValueError("An executed wipe stroke plan changed or was removed: "+plan_id)
        return plans

    def execute_due_wipe_strokes(self, plans):
        """Select future robot targets only; never move cloth or advance physics."""
        for plan in plans:
            if (plan["id"] in self.executed_wipe_stroke_ids
                    or plan["after_cycle"]>self.press_cycles):
                continue
            metrics=deepcopy(self.h["coupling"].metrics())
            if not metrics["active_liquid_count"]:
                break
            previous=self.current_wipe_translation_xy.copy()
            self.current_wipe_translation_xy=np.asarray(plan["translation_xy_m"],dtype=float)
            record={"plan":deepcopy(plan),"after_completed_cycle":self.press_cycles,
                    "time_s":self.frame*self.dt,"previous_translation_xy_m":previous.tolist(),
                    "current_center_xy_m":self.current_cloth_center_xy.tolist(),
                    "current_yaw_rad":self.current_cloth_yaw_rad,"metrics":metrics}
            self.executed_wipe_stroke_ids.append(plan["id"])
            self.wipe_stroke_history.append(record)
            self.event("08 Select scheduled world-XY wipe stroke",**record)
            self.save_task_checkpoint("after_"+plan["id"]+".pkl","pressing")

    def reposition_cloth(self, plan):
        """Physically grasp, lift, shift, lay and release the same material corners."""
        coupling=self.h["coupling"]
        before_metrics=deepcopy(coupling.metrics())
        bound_before=np.asarray(coupling.bound_particle_indices,dtype=int).copy()
        prefix="08 "+plan["id"].replace("_"," ").title()
        self.prepare_wet_grasp_layout(phase_prefix=prefix)
        previous=self.current_cloth_center_xy.copy()
        destination=np.asarray(self.cfg["soy"]["position_xy"],dtype=float)+plan["center_offset_xy_m"]
        self.event(prefix+": begin physical cloth-area change",plan=deepcopy(plan),
                   previous_center_xy_m=previous.tolist(),target_center_xy_m=destination.tolist(),
                   metrics_before=before_metrics)
        pad_contacts=self.grasp_wet_corners(previous,phase_prefix=prefix)
        grasp_evidence=deepcopy(self.attached)
        self.move_layout(prefix+": Lift same diagonal corners",self.layout_targets(previous,.96),2.)
        lifted_tcp={s:self.tcp(s).tolist() for s in SIDES}
        self.move_layout(prefix+": Translate suspended cloth",self.layout_targets(destination,.96),2.5)
        pinch_z=self.cfg["robot_task"].get("pinch_tcp_z_m",.771)
        self.move_layout(prefix+": Lay shifted cloth",self.layout_targets(destination,pinch_z),2.2)
        self.hold(.3)
        laid_tcp={s:self.tcp(s).tolist() for s in SIDES}
        self.detach()
        self.fingers(prefix+": Release shifted cloth",.010,.5)
        after_metrics=deepcopy(coupling.metrics())
        bound_after=np.asarray(coupling.bound_particle_indices,dtype=int)
        preserved=bool(np.isin(bound_before,bound_after).all())
        record={"plan":deepcopy(plan),"after_completed_cycle":self.press_cycles,
                "time_s":self.frame*self.dt,"previous_center_xy_m":previous.tolist(),
                "target_center_xy_m":destination.tolist(),"metrics_before":before_metrics,
                "layout_yaw_rad":self.current_cloth_yaw_rad,
                "metrics_after":after_metrics,"bound_before_count":int(len(bound_before)),
                "bound_after_count":int(len(bound_after)),"previously_bound_parcels_preserved":preserved,
                "pad_contacts_before_lift":pad_contacts,"grasp_material":grasp_evidence,
                "measured_lifted_tcp_m":lifted_tcp,"measured_laid_tcp_m":laid_tcp}
        self.event(prefix+": physical cloth-area change measured",**record)
        if not preserved:
            raise RuntimeError("Previously absorbed parcels lost their bound identity during reposition")
        self.current_cloth_center_xy=destination.copy()
        self.current_wipe_translation_xy=np.asarray(plan["wipe_translation_xy_m"],dtype=float)
        self.executed_reposition_ids.append(plan["id"])
        self.reposition_history.append(record)
        self.snapshot("08_"+plan["id"]+"_laid")
        self.save_task_checkpoint("after_"+plan["id"]+".pkl","pressing")

    def task(self, resume_after_lift=False):
        cfg=self.cfg["robot_task"]
        cotton=np.asarray(self.cfg["cloth"]["position_xy"])
        soy=np.asarray(self.cfg["soy"]["position_xy"])
        pinch_z=cfg.get("pinch_tcp_z_m",.771)
        def targets(center,z):
            return self.layout_targets(center,z)
        if not resume_after_lift:
            self.event("01 Approach cloth with both arms")
            self.hold(.5)
            self.snapshot("01_initial")
            self.move("01 Approach cloth with both arms",targets(cotton,.96),2.5)
            if cfg.get("contact_first_grasp", False):
                preferred=self.open_grasp_approach("02 Approach diagonal corners",cotton,pinch_z,2.)
                self.grasp_dry_corners(preferred)
            else:
                self.fingers("02 Set narrow opening above diagonal corners",cfg.get("pregrasp_finger_opening_m",.008),.5)
                self.move("02 Lower grippers to diagonal corners",targets(cotton,pinch_z),2.0)
                self.hold(.3)
                preferred=self.corner_candidates()
                if not self.grasp("03 Pinch opposite diagonal corners",preferred,required=False):
                    self.move("03 Refine diagonal-corner grasp height",targets(cotton,max(.7698,pinch_z-.0002)),.5)
                    self.hold(.2)
                    self.grasp("03 Confirm both diagonal-corner contacts",preferred,close=False)
            self.initial_grasp_material={s:dict(report) for s,report in self.attached.items()}
            self.snapshot("03_grasped")
            self.move("04 Lift cloth using both robot hands",targets(cotton,.96),2.)
            self.snapshot("04_cloth_lifted")
            self.save_task_checkpoint("after_dry_lift.pkl","after_dry_lift")
        self.move("05 Carry cloth above soy sauce",targets(soy,.96),2.5)
        self.move("06 Lay cloth over the puddle",targets(soy,pinch_z),2.2)
        self.hold(.3)
        self.snapshot("06_laid_over_soy")
        self.detach()
        self.fingers("07 Release cloth flat on the table",.010,.5)
        self.save_task_checkpoint("before_pressing.pkl","pressing")
        self.press_cloth()
        self.finish_pickup()

    def press_cloth(self):
        """Continue at the next complete press cycle without replaying motion.

        Checkpoints are taken only after the return/up/hold sequence finishes.
        ``press_cycles`` therefore counts fully completed cycles, including
        after resume; a partial cycle never advances this counter.
        """
        cfg=self.cfg["robot_task"]
        self.ensure_press_control_state()
        plans=self.reposition_schedule()
        stroke_plans=self.wipe_stroke_schedule()
        pinch_z=cfg.get("pinch_tcp_z_m",.771)
        def targets(center,z):
            return self.layout_targets(center,z)
        endpoint_hold=float(cfg.get("wipe_endpoint_hold_s",0.))
        if not np.isfinite(endpoint_hold) or endpoint_hold<0:
            raise ValueError("Wipe endpoint hold must be finite and nonnegative")
        completed=getattr(self,"press_cycles",0)
        maximum=cfg.get("maximum_press_cycles",14)
        if isinstance(completed,bool) or not isinstance(completed,(int,np.integer)) or not 0<=completed<=maximum:
            raise ValueError("Invalid completed press-cycle count in checkpoint")
        self.press_cycles=int(completed)
        # Native finger/cloth friction provides a short table-parallel wipe.
        # Moving fresh material over residual liquid avoids repeatedly loading
        # only the already saturated central triangles. Both TCPs receive the
        # same translation, preserving the authored diagonal spacing.
        self.event("08 Press diagonal corners and wipe across the puddle")
        already_absorbed=completed>=2 and self.h["coupling"].metrics()["active_liquid_count"]==0
        for cycles in (() if already_absorbed else range(completed+1,maximum+1)):
            repositioned=self.execute_due_repositions(plans)
            if repositioned and self.press_cycles>=2 and not self.h["coupling"].metrics()["active_liquid_count"]:
                break
            self.execute_due_wipe_strokes(stroke_plans)
            center=self.current_cloth_center_xy.copy()
            self.move_layout(f"08 Press cycle {cycles}: down",targets(center,pinch_z),.45)
            self.hold(.35)
            from robot_grasp import pad_contact_report
            self.press_contacts.append({s:pad_contact_report(self.h["robots"][s],self.h["cloth"]) for s in SIDES})
            if self.h["coupling"].metrics()["active_liquid_count"]:
                stroke=self.current_wipe_translation_xy
                before=arr(self.h["cloth"].get_particles_pos()).copy()
                self.move_layout(f"08 Press cycle {cycles}: wipe outward",targets(center+stroke,pinch_z),1.6)
                if endpoint_hold>0:
                    self.hold(endpoint_hold)
                after=arr(self.h["cloth"].get_particles_pos()).copy()
                self.event(f"08 Press cycle {cycles}: measured wipe", median_material_translation_m=np.median(after-before,axis=0).tolist())
                self.move_layout(f"08 Press cycle {cycles}: wipe return",targets(center,pinch_z),1.6)
            self.move_layout(f"08 Press cycle {cycles}: up",targets(center,pinch_z+.005),.45)
            self.hold(.15)
            m=self.h["coupling"].metrics()
            self.press_cycles=cycles
            self.save_task_checkpoint(f"after_press_cycle_{cycles}.pkl","pressing")
            if m["active_liquid_count"]==0 and cycles>=2:
                break
        m=self.h["coupling"].metrics()
        self.event("Absorption measurement", metrics=m)
        self.snapshot("08_after_pressing")
        if m["active_liquid_count"]!=0:
            raise RuntimeError(f"Puddle not fully absorbed: {m['active_liquid_count']} active particles remain")
        self.save_task_checkpoint("before_final_pickup.pkl","before_final_pickup")

    def grasp_wet_corners(self, center, *, phase_prefix=None):
        """Common unchanged wet-grasp servo, feedback and actual-contact gates."""
        cfg=self.cfg["robot_task"]
        self.wet_grasp_active=True
        center=np.asarray(center,dtype=float)
        def phase(original):
            return original if phase_prefix is None else phase_prefix+": "+original.split(" ",1)[1]
        # Close with finite servo force, then hold the actual measured opening
        # after each contact-gated grasp rather than squeezing through a fold.
        wet_kp=cfg.get("wet_grasp_finger_position_gain",2000.)
        wet_kv=cfg.get("wet_grasp_finger_velocity_gain",40.)
        for side in SIDES:
            indices=self.h["robot_dofs"][side][-2:]
            self.h["robots"][side].set_dofs_kp(np.full(2,wet_kp),indices)
            self.h["robots"][side].set_dofs_kv(np.full(2,wet_kv),indices)
        self.event(phase("09 Set wet-cloth finger servo"),finger_position_gain=wet_kp,finger_velocity_gain=wet_kv)
        pinch_z=cfg.get("final_pinch_tcp_z_m",.7705)
        def targets(center,z):
            return self.layout_targets(center,z)
        contact_first=cfg.get("contact_first_grasp",False)
        if contact_first:
            preferred=self.open_grasp_approach(phase("09 Approach wet diagonal corners"),
                                               center,pinch_z,.8,wet=True)
        else:
            self.fingers(phase("09 Preshape above wet diagonal corners"),cfg.get("wet_pregrasp_finger_opening_m",.006),.5)
            self.move_layout(phase("09 Lower grippers to regrasp wet cloth"),targets(center,pinch_z),.8)
            preferred=self.corner_candidates()
        if not self.grasp(phase("10 Pinch the same wet diagonal corners"),preferred,required=False):
            # A flattened sheet can lie slightly below the first grasp's fold.
            # Use one bounded 0.2mm physical correction; never widen the gate.
            lower=(float(self.grasp_approach_targets["left"][2]) if contact_first
                   else max(.7698,pinch_z-.0002))
            if not contact_first:
                self.move_layout(phase("10 Refine wet-cloth grasp height"),targets(center,lower),.4)
                self.hold(.15)
            if not self.grasp(phase("10 Confirm both wet-cloth contacts"),preferred,close=False,required=False):
                # A folded corner can wedge between the finger underside and
                # table, holding the jaws open. Ease the closed fingers upward
                # by 1 mm before checking the same contact/strain gate again.
                # This is actual robot motion, with no cloth-state edits.
                self.move_layout(phase("10 Ease wet-cloth fold above table"),targets(center,lower+.001),.6)
                self.hold(.3)
                self.grasp(phase("10 Confirm relieved wet-cloth contacts"),preferred,close=False)
        from robot_grasp import pad_contact_report
        contacts={s:pad_contact_report(self.h["robots"][s],self.h["cloth"]) for s in SIDES}
        self.event(phase("10 Fresh final finger-pad contact evidence"),contacts=contacts)
        if not all(r["both_fingers_contact"] for r in contacts.values()):
            label="Final" if phase_prefix is None else "Reposition"
            raise RuntimeError(label+" wet-cloth grasp lost actual bilateral pad support")
        return contacts

    def finish_pickup(self):
        cfg=self.cfg["robot_task"]
        self.ensure_press_control_state()
        self.prepare_wet_grasp_layout()
        center=self.current_cloth_center_xy.copy()
        self.final_pad_contacts=self.grasp_wet_corners(center)
        self.bound_before_lift=self.h["coupling"].bound_positions.copy()
        self.move_layout("11 Lift wet cloth and absorbed soy together",self.layout_targets(center,1.02),cfg.get("wet_lift_duration_s",3.5))
        self.hold(cfg.get("wet_lift_settle_s",1.))
        self.completed=True
        self.event("12 Complete - table liquid fully picked up")
        self.snapshot("12_wet_cloth_lifted")
        checkpoint=self.save_task_checkpoint("after_wet_lift.pkl","audit_only_complete")
        self.final_audit_checkpoint={"saved":checkpoint is not None,
            "path":str(checkpoint) if checkpoint is not None else None,
            "resume_stage":"audit_only_complete",
            "purpose":"Independent final native particle-mass and absorbed-parcel ledger audit; not resumable."}

    def report(self, failure=None):
        m=self.h["coupling"].metrics()
        cp=arr(self.h["cloth"].get_particles_pos())
        from robot_grasp import _pad_distance, cloth_contact_radius
        gripper_state={}
        for side in SIDES:
            robot=self.h["robots"][side]
            ids=np.asarray(self.attached.get(side,{}).get("attached_indices",[]),dtype=int)
            gripper_state[side]={
                "finger_joint_positions_m":arr(robot.get_dofs_position(self.h["robot_dofs"][side][-2:])).tolist(),
                "finger_position_targets_m":self.q[side][7:].tolist(),
                "links":{name:{"position_m":arr(robot.get_link(name).get_pos()).tolist(),
                                "quaternion_wxyz":arr(robot.get_link(name).get_quat()).tolist()}
                         for name in ("hand","left_finger","right_finger")},
                "bound_indices":ids.tolist(),
                "bound_particle_pad_shell_clearance_m":{
                    name:(_pad_distance(cp[ids],robot.get_link(name))-cloth_contact_radius(self.h["cloth"])).tolist()
                    for name in ("left_finger","right_finger")}}
        rise=float(np.median(self.h["coupling"].bound_positions[:,2]-self.bound_before_lift[:,2])) if hasattr(self,"bound_before_lift") else 0.
        movement={s:float(np.max(np.abs(arr(self.h["robots"][s].get_dofs_position(self.h["robot_dofs"][s]))[:7]-self.initial_q[s][:7]))) for s in SIDES}
        checks={"complete_action_sequence":self.completed, "all_liquid_absorbed":m["active_liquid_count"]==0,
            "both_robot_arms_moved":all(v>.2 for v in movement.values()),
            "two_contact_gated_final_grasps":len(self.attached)==2,
            "wet_cloth_clear_of_table":float(cp[:,2].min())>self.cfg["table"]["height"]+.06,
            "absorbed_liquid_carried_up":rise>.06,
            "mass_conserved":self.max_mass_error<1e-7, "wet_mass_in_native_cloth":self.max_loading_error<1e-7,
            "finite_local_capacity":self.max_saturation<=1.000001,
            "robot_tracking_within_5mm":self.max_tcp_error<.005,
            "no_arm_table_or_arm_arm_collision":self.rigid_collision_samples==0,
            "both_arms_pressed_cloth":bool(self.press_contacts) and all(any(p[s]["contact_count"]>0 for p in self.press_contacts) for s in SIDES),
            "no_cloth_through_table":self.minimum_cloth_z>=self.cfg["table"]["height"]-.002}
        from cloth_metrics import acceptance
        checks.update(acceptance(self.fabric_stages, direction_limits=self.direction_limits))
        checks["initial_and_final_grasps_preserve_material_lengths"] = bool(self.initial_grasp_material) and len(self.attached)==2 and all(
            report.get("fixed_subset_selection",{}).get("fixed_point_pair_max_extension",float("inf"))<=.02
            for report in [*self.initial_grasp_material.values(),*self.attached.values()])
        out={"passed":all(checks.values()) and failure is None, "checks":checks, "failure":failure,
            "simulated_seconds":self.frame*self.dt, "wall_seconds":time.monotonic()-self.started,
            "resumed_from":str(self.args.resume_robot_checkpoint) if self.args.resume_robot_checkpoint else None,
            "initial_liquid_g":self.initial_mass*1000,"absorbed_liquid_g":m["absorbed_mass_kg"]*1000,
            "active_liquid_particles":m["active_liquid_count"], "median_carried_liquid_rise_m":rise,
            "minimum_final_cloth_height_m":float(cp[:,2].min()), "press_cycles":getattr(self,"press_cycles",0),
            "max_tcp_tracking_error_m":self.max_tcp_error,"max_ik_position_error_m":self.max_ik_error,
            "max_mass_error_kg":self.max_mass_error,"robot_joint_change_rad":movement,
            "press_contact_evidence":self.press_contacts,"rigid_collision_samples":self.rigid_collision_samples,
            "grasp_model":self.h["grasp_adapter"],"final_metrics":m,
            "cloth_contact_model":self.h.get("cloth_contact_model"),
            "material_profile":self.h.get("material_profile"),
            "numerical_resolution":self.h.get("numerical_resolution"),
            "fabric_stage_measurements":self.fabric_stages,
            "maximum_sampled_edge_extension":self.max_edge_extension,
            "maximum_sampled_p99_edge_extension":self.max_p99_extension,
            "initial_grasp_material":self.initial_grasp_material,
            "final_grasp_material":self.attached,
            "final_pad_contacts":getattr(self,"final_pad_contacts",{}),
            "final_audit_checkpoint":getattr(self,"final_audit_checkpoint",None),
            "final_gripper_state":gripper_state,
            "cloth_repositions":getattr(self,"reposition_history",[]),
            "current_cloth_center_target_xy_m":(self.current_cloth_center_xy.tolist()
                if hasattr(self,"current_cloth_center_xy") else None),
            "current_wipe_translation_xy_m":(self.current_wipe_translation_xy.tolist()
                if hasattr(self,"current_wipe_translation_xy") else None),
            "executed_reposition_ids":getattr(self,"executed_reposition_ids",[]),
            "executed_wipe_stroke_ids":getattr(self,"executed_wipe_stroke_ids",[]),
            "wipe_stroke_history":getattr(self,"wipe_stroke_history",[]),
            "current_cloth_yaw_rad":getattr(self,"current_cloth_yaw_rad",0.),
            "layout_tracking_history":getattr(self,"layout_tracking_history",[]),
            "limitations":["Contact-gated no-slip grasp constraint; grip load is not returned to the robot and force closure is not validated.","Uncalibrated PDF-referenced cotton approximation and soy parameters; finite local absorption with no resolved molecular wicking or desorption."]}
        if self.direction_limits is not None:
            from task_outcome import classify_task_outcome
            out=classify_task_outcome(out)
        (self.out/"robot_wipe_validation.json").write_text(json.dumps(out,indent=2)+"\n")
        if self.rows:
            with (self.out/"mass_and_motion.csv").open("w") as f:
                w=csv.DictWriter(f,fieldnames=self.rows[0].keys());w.writeheader();w.writerows(self.rows)
        np.savez_compressed(self.out/"final_state.npz",cloth=cp,liquid=arr(self.h["soy"].get_particles_pos()),
            active=arr(self.h["soy"].get_particles_active()),bound_liquid=self.h["coupling"].bound_positions,
            absorbed_vertex_mass=self.h["coupling"].vertex_absorbed_mass)
        print(json.dumps(out,indent=2),flush=True)
        return out

def run(args, config):
    import genesis as gs
    from scene import build_scene
    config["robot_task"]={**config.get("robot_task",{}),"enabled":True}
    output=args.output.resolve()
    if output.name=="output":output=output/"robot_wipe"
    output.mkdir(parents=True,exist_ok=True)
    (output/"config_used.json").write_text(json.dumps(config,indent=2)+"\n")
    scene,handles=build_scene(config,headless=args.headless,backend=args.backend,width=args.width,height=args.height,liquid_view=args.liquid_view)
    task=RobotWipe(scene,handles,config,args,output)
    failure=None
    try:
        try:
            if args.resume_robot_checkpoint:
                from robot_checkpoint import load_checkpoint
                state=load_checkpoint(args.resume_robot_checkpoint,scene,handles)
                for key,value in state.items():setattr(task,key,value)
                stage=getattr(task,"resume_stage",None)
                if stage=="after_dry_lift":
                    task.task(resume_after_lift=True)
                elif stage=="pressing":
                    task.press_cloth()
                    task.finish_pickup()
                elif stage=="before_final_pickup":
                    task.finish_pickup()
                else:
                    raise ValueError(f"Unknown robot checkpoint resume stage: {stage!r}")
            else:
                task.task()
        except StepLimit:failure="Run stopped before completion (step limit or viewer closed)."
        except Exception as exc:
            failure=str(exc)
            task.snapshot("failure_state")
            raise
        finally:
            report=task.report(failure)
            if task.writer is not None:task.writer.close();task.writer=None
        task_passed=report.get("task_passed",report["passed"]) is True
        if not task_passed and not args.steps:
            raise RuntimeError("Robot task validation failed")
        if task_passed and report.get("material_warnings"):
            print("Robot task completed with directional material diagnostic warnings; "
                  "the original passed result and all strict checks are retained. "
                  "Warning checks: "+", ".join(w["check"] for w in report["material_warnings"]),
                  flush=True)
        if args.hold_open and not args.headless:
            while scene.viewer.is_alive():
                scene.viewer.update()
                time.sleep(.02)
    finally:
        gs.destroy()
