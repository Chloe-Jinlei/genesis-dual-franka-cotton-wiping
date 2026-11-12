"""Dual Franka feedback-controlled cloth pick/carry/press/pick task.

Robot joints use Genesis PD dynamics. Only local cloth nodes actually touching
both finger pads can receive a no-slip link constraint. Interior cloth, free
liquid, finite absorption reservoirs and wet mass remain simulated.
"""
from __future__ import annotations
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
        self.completed = False
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
            active = arr(self.h["soy"].get_particles_active()).astype(bool)
            liquid = arr(self.h["soy"].get_particles_pos())[active]
            if not np.isfinite(liquid).all() or (len(liquid) and liquid[:,2].min()<self.cfg["table"]["height"]-.002):
                raise RuntimeError("Free liquid became nonfinite or penetrated the table")
            self.minimum_cloth_z = min(self.minimum_cloth_z, float(cp[:,2].min()))
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
                "cloth_min_z_m":float(cp[:,2].min()), "left_tcp_error_m":errors["left"], "right_tcp_error_m":errors["right"]})
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
        Image.fromarray(self.render_frame()).save(self.out/f"{name}.png")
        rgb,*_=self.h["cameras"]["detail"].render()
        Image.fromarray(np.asarray(rgb)).save(self.out/f"{name}_detail.png")

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
            report=attach_grasp(self.h["robots"][s],self.h["cloth"],near,
                gate_mode="bilateral_patch", commanded_finger_opening=0.0)
            contacts=self.h["robots"][s].get_contacts()
            report["native_rigid_contacts"]={k:arr(v).tolist() for k,v in contacts.items()}
            reports[s]=report
            if report.get("attached",False):
                self.attached[s]=report
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

    def detach(self):
        from robot_grasp import detach
        detach(self.h["cloth"])
        self.attached={}

    def task(self):
        cfg=self.cfg["robot_task"]
        cotton=np.asarray(self.cfg["cloth"]["position_xy"])
        soy=np.asarray(self.cfg["soy"]["position_xy"])
        half=cfg.get("grasp_half_span_m",.13)
        pinch_z=cfg.get("pinch_tcp_z_m",.771)
        def targets(center,z):
            return {s:np.r_[center+np.array([(-1 if s=="left" else 1)*half,0]),z] for s in SIDES}
        self.event("01 Approach cloth with both arms")
        self.hold(.5)
        self.snapshot("01_initial")
        self.move("01 Approach cloth with both arms",targets(cotton,.96),2.5)
        self.move("02 Lower open grippers to cloth edges",targets(cotton,pinch_z),2.0)
        self.hold(.3)
        cp=arr(self.h["cloth"].get_particles_pos())
        preferred={s:np.flatnonzero((np.abs(cp[:,0]-p[0])<.015)&(np.abs(cp[:,1]-p[1])<.018)) for s,p in targets(cotton,pinch_z).items()}
        self.grasp("03 Pinch both cloth edges",preferred)
        self.snapshot("03_grasped")
        self.move("04 Lift cloth using both robot hands",targets(cotton,.96),2.)
        self.snapshot("04_cloth_lifted")
        self.move("05 Carry cloth above soy sauce",targets(soy,.96),2.5)
        self.move("06 Lay cloth over the puddle",targets(soy,pinch_z),2.2)
        self.hold(.3)
        self.snapshot("06_laid_over_soy")
        self.detach()
        self.fingers("07 Release cloth flat on the table",.010,.5)
        # Repeated low-amplitude vertical movement applies finger contact along
        # both edges. The free interior contacts and absorbs the small puddle.
        self.event("08 Press both edges and move up/down")
        cycles=0
        for cycles in range(1,cfg.get("maximum_press_cycles",14)+1):
            self.move(f"08 Press cycle {cycles}: down",targets(soy,pinch_z),.45)
            self.hold(.35)
            from robot_grasp import pad_contact_report
            self.press_contacts.append({s:pad_contact_report(self.h["robots"][s],self.h["cloth"]) for s in SIDES})
            self.move(f"08 Press cycle {cycles}: up",targets(soy,pinch_z+.005),.45)
            self.hold(.15)
            m=self.h["coupling"].metrics()
            if m["active_liquid_count"]==0 and cycles>=2:
                break
        self.press_cycles=cycles
        m=self.h["coupling"].metrics()
        self.event("Absorption measurement", metrics=m)
        self.snapshot("08_after_pressing")
        if m["active_liquid_count"]!=0:
            raise RuntimeError(f"Puddle not fully absorbed: {m['active_liquid_count']} active particles remain")
        from robot_checkpoint import save_checkpoint
        fields=("q","initial_q","frame","phase","rows","events","initial_mass",
            "max_mass_error","max_loading_error","max_saturation","max_tcp_error","max_ik_error",
            "minimum_cloth_z","rigid_collision_samples","press_contacts","press_cycles")
        save_checkpoint(self.out/"before_final_pickup.pkl",self.scene,self.h,{k:getattr(self,k) for k in fields})
        self.finish_pickup()

    def finish_pickup(self):
        cfg=self.cfg["robot_task"]
        soy=np.asarray(self.cfg["soy"]["position_xy"])
        half=cfg.get("grasp_half_span_m",.13)
        pinch_z=cfg.get("final_pinch_tcp_z_m",.7705)
        def targets(center,z):
            return {s:np.r_[center+np.array([(-1 if s=="left" else 1)*half,0]),z] for s in SIDES}
        self.move("09 Lower grippers to regrasp wet cloth",targets(soy,pinch_z),.8)
        cp=arr(self.h["cloth"].get_particles_pos())
        preferred={s:np.flatnonzero((np.abs(cp[:,0]-p[0])<.018)&(np.abs(cp[:,1]-p[1])<.02)) for s,p in targets(soy,pinch_z).items()}
        if not self.grasp("10 Pinch both wet cloth edges",preferred,required=False):
            # A flattened sheet can lie slightly below the first grasp's fold.
            # Use one bounded 0.5mm physical correction; never widen the gate.
            lower=max(.770,pinch_z-.0005)
            self.move("10 Refine wet-cloth grasp height",targets(soy,lower),.4)
            self.hold(.15)
            self.grasp("10 Confirm both wet-cloth contacts",preferred,close=False)
        self.bound_before_lift=self.h["coupling"].bound_positions.copy()
        self.move("11 Lift wet cloth and absorbed soy together",targets(soy,1.02),2.5)
        self.hold(.5)
        self.completed=True
        self.event("12 Complete - table liquid fully picked up")
        self.snapshot("12_wet_cloth_lifted")

    def report(self, failure=None):
        m=self.h["coupling"].metrics()
        cp=arr(self.h["cloth"].get_particles_pos())
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
            "limitations":["Contact-gated no-slip grasp constraint; grip load is not returned to the robot and force closure is not validated.","Uncalibrated thick-cotton and soy parameters; finite local absorption with no molecular wicking or desorption."]}
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
                task.finish_pickup()
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
        if not report["passed"] and not args.steps:
            raise RuntimeError("Robot task validation failed")
        if args.hold_open and not args.headless:
            while scene.viewer.is_alive():
                scene.viewer.update()
                time.sleep(.02)
    finally:
        gs.destroy()
