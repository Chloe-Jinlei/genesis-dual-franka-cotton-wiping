"""Dual Franka tabletop with native SPH liquid and a free PBD cotton cloth.

All dimensions are metres. Material parameters are illustrative, not calibrated.
The optional local cloth/liquid adapter transfers conserved liquid mass into a
finite-capacity cloth reservoir. It is an operator-split porous-sheet model.
"""
from __future__ import annotations

import argparse
from importlib.metadata import version
import json
import logging
from pathlib import Path
import time

import numpy as np
from PIL import Image

from generate_assets import ensure_assets

HERE = Path(__file__).resolve().parent


def numpy(value):
    return value.detach().cpu().numpy()


def build_scene(config, *, headless=False, backend="cuda", width=1280, height=900,
                liquid_view="recon"):
    """Return scene plus named control handles for subsequent robot experiments."""
    import genesis as gs
    import torch

    if backend == "auto":
        backend = "cuda" if torch.cuda.is_available() else "cpu"
    if backend == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable; use --backend cpu.")
    gs.init(backend=gs.cuda if backend == "cuda" else gs.cpu, seed=config["seed"],
            precision="32", logging_level=logging.WARNING)
    assets = ensure_assets()
    sim, fluid, cotton = (config[k] for k in ("simulation", "soy", "cloth"))
    overview = config["cameras"]["overview"]
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=sim["dt"], substeps=sim["substeps"]),
        rigid_options=gs.options.RigidOptions(enable_self_collision=True),
        sph_options=gs.options.SPHOptions(
            particle_size=fluid["particle_size"], pressure_solver=fluid["pressure_solver"],
            lower_bound=(-1.2, -0.9, 0.0), upper_bound=(1.2, 0.9, 2.1),
            hash_grid_res=(0.3, 0.3, 0.3),
            max_divergence_solver_iterations=30, max_density_solver_iterations=40),
        pbd_options=gs.options.PBDOptions(
            particle_size=cotton["particle_size"],
            max_stretch_solver_iterations=8, max_bending_solver_iterations=4,
            hash_grid_res=(0.3, 0.3, 0.3),
            lower_bound=(-1.2, -0.9, 0.0), upper_bound=(1.2, 0.9, 2.1)),
        viewer_options=gs.options.ViewerOptions(
            res=(width, height), camera_pos=overview["position"],
            camera_lookat=overview["target"], camera_fov=overview["fov"],
            refresh_rate=30, realtime_factor=1.0 if not headless else None),
        vis_options=gs.options.VisOptions(
            ambient_light=(0.4, 0.4, 0.4), background_color=(0.12, 0.15, 0.19),
            show_world_frame=False, shadow=True,
            lights=[{"type": "directional", "dir": (0.2, 0.5, -1),
                     "color": (1.0, 0.97, 0.92), "intensity": 3.0},
                    {"type": "directional", "dir": (-0.5, -0.3, -1),
                     "color": (0.88, 0.94, 1.0), "intensity": 1.8}]),
        show_viewer=not headless,
    )
    rigid = gs.materials.Rigid(friction=0.65, needs_coup=True,
                               coup_friction=0.65 if config.get("robot_task", {}).get("enabled") else 0.1,
                               coup_softness=0.0003)

    def box(name, position, size, color, coupling=True):
        return scene.add_entity(name=name,
            morph=gs.morphs.Box(pos=position, size=size, fixed=True),
            material=rigid if coupling else gs.materials.Rigid(needs_coup=False),
            surface=gs.surfaces.Default(color=color, roughness=0.65))

    scene.add_entity(name="floor", morph=gs.morphs.Plane(), material=rigid,
                     surface=gs.surfaces.Default(color=(0.22, 0.25, 0.29, 1)))
    sx, sy, sz = config["table"]["size"]
    th = config["table"]["height"]
    tabletop = box("tabletop", (0, 0, th-sz/2), (sx, sy, sz), (0.68, 0.53, 0.35, 1))
    for i, x in enumerate((-sx/2+0.08, sx/2-0.08)):
        for j, y in enumerate((-sy/2+0.08, sy/2-0.08)):
            box(f"table_leg_{i}_{j}", (x, y, (th-sz)/2), (0.065, 0.065, th-sz),
                (0.22, 0.25, 0.28, 1))
    robots = {}
    for side in ("left", "right"):
        bp = config["robots"][f"{side}_base"]
        yaw = config["robots"][f"{side}_yaw"]
        box(f"{side}_mount", (bp[0], bp[1], (th+bp[2])/2),
            (0.19, 0.19, bp[2]-th), (0.19, 0.22, 0.25, 1))
        robots[side] = scene.add_entity(name=f"{side}_franka",
            morph=gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml", pos=bp,
                quat=(np.cos(yaw/2), 0, 0, np.sin(yaw/2))),
            material=gs.materials.Rigid(needs_coup=True, coup_friction=0.6,
                coup_softness=0.0005, gravity_compensation=1.0))

    cloth = scene.add_entity(name="rectangular_cotton",
        morph=gs.morphs.Mesh(file=str(assets["cloth_mesh"]),
            pos=(*cotton["position_xy"], th+cotton["initial_clearance"]),
            file_meshes_are_zup=True, decimate=False, convexify=False),
        material=gs.materials.PBD.Cloth(rho=cotton["areal_density"],
            static_friction=cotton["friction"], kinetic_friction=cotton["friction"],
            stretch_compliance=cotton["stretch_compliance"],
            bending_compliance=cotton["bending_compliance"]),
        surface=gs.surfaces.Default(
            diffuse_texture=gs.textures.ImageTexture(image_path=str(assets["cloth_texture"])),
            vis_mode="visual", roughness=0.95, double_sided=True))
    soy = scene.add_entity(name="soy_sauce",
        morph=gs.morphs.Mesh(file=str(assets["soy_mesh"]),
            pos=(*fluid["position_xy"], th+fluid["initial_clearance"]),
            scale=(*fluid.get("xy_scale", [1, 1]), fluid["depth_scale"]), file_meshes_are_zup=True,
            decimate=False, convexify=False),
        material=gs.materials.SPH.Liquid(rho=fluid["density"], mu=fluid["viscosity"],
            gamma=fluid["surface_tension"], sampler="regular"),
        surface=gs.surfaces.Default(color=fluid["color"], roughness=0.15,
            vis_mode=liquid_view, recon_backend="splashsurf"))
    cameras = {name: scene.add_camera(res=(width, height), GUI=False,
        pos=cam["position"], lookat=cam["target"], fov=cam["fov"])
        for name, cam in config["cameras"].items()}
    grasp_adapter = None
    if config.get("robot_task", {}).get("enabled", False):
        from robot_grasp import install_grasp_adapter, install_rigid_pbd_friction
        install_rigid_pbd_friction(scene)
        grasp_adapter = install_grasp_adapter(scene)
    print("Building dual Franka / SPH soy / PBD cloth scene...", flush=True)
    scene.build()
    print(f"Built: {soy.n_particles} liquid particles, {cloth.n_particles} cloth particles.", flush=True)
    home = np.array([*config["robots"]["home"], *([config["robots"]["finger_opening"]]*2)])
    dofs = {}
    for side, robot in robots.items():
        indices = [robot.get_joint(n).dofs_idx_local[0] for n in
                   [f"joint{i}" for i in range(1, 8)]+["finger_joint1", "finger_joint2"]]
        dofs[side] = indices
        robot.set_dofs_position(home, dofs_idx_local=indices)
        finger_kp = 800 if grasp_adapter else 100
        finger_kv = 20 if grasp_adapter else 10
        robot.set_dofs_kp(np.array([4500,4500,3500,3500,2000,2000,2000,finger_kp,finger_kp]), indices)
        robot.set_dofs_kv(np.array([450,450,350,350,200,200,200,finger_kv,finger_kv]), indices)
        limit = np.array([87,87,87,87,12,12,12,50,50])
        robot.set_dofs_force_range(-limit, limit, indices)
        robot.control_dofs_position(home, indices)
    handles = {"robots": robots, "robot_dofs": dofs, "home": home,
                   "table": tabletop, "cloth": cloth, "soy": soy,
                   "cameras": cameras, "backend": backend, "initial_state": scene.get_state(),
                   "grasp_adapter": grasp_adapter}
    if config.get("coupling", {}).get("enabled", True):
        install_cloth_liquid_coupling(scene, handles, config)
    return scene, handles


def install_cloth_liquid_coupling(scene, handles, config):
    """Instance-local bridge: native physics -> porous coupling -> visualization.

    The installed engine has no SPH/PBD pair. This adapter runs once per outer
    step, without editing Genesis or changing other scenes. Gradients and
    batched environments are deliberately unsupported by this CPU bridge.
    """
    from cloth_liquid_coupling import ClothLiquidCoupling
    from wet_visual import WetClothVisual

    coupling = ClothLiquidCoupling(scene, handles["cloth"], handles["soy"], config)
    wet = WetClothVisual(scene, handles["cloth"])
    handles.update(coupling=coupling, wet_visual=wet)
    native_step = scene.sim.step
    native_reset = scene.reset

    def step(in_backward=False):
        if in_backward:
            raise NotImplementedError("Porous-sheet coupling does not support differentiation.")
        native_step(in_backward=False)
        coupling.step(config["simulation"]["dt"])
        wet.update(coupling.vertex_saturation)

    def reset(state=None, envs_idx=None):
        if envs_idx is not None:
            raise NotImplementedError("Porous-sheet reset supports one environment.")
        if handles.get("grasp_adapter"):
            from robot_grasp import detach, restore_grasp_state
            detach(handles["cloth"])
        if isinstance(state, dict) and "physics" in state and "absorption" in state:
            native_reset(state["physics"])
            coupling.restore_state(state["absorption"])
            if handles.get("grasp_adapter") and "grasp" in state:
                restore_grasp_state(handles["cloth"], state["grasp"])
            wet.update(coupling.vertex_saturation)
        elif state is None or state is handles["initial_state"]:
            native_reset(handles["initial_state"])
            coupling.reset()
            wet.reset()
            for side, robot in handles["robots"].items():
                robot.control_dofs_position(handles["home"], handles["robot_dofs"][side])
        else:
            raise ValueError("Use capture_coupled_state() to preserve both physics and absorbed liquid.")

    scene.sim.step = step
    scene.reset = reset


def capture_coupled_state(scene, handles):
    """A checkpoint includes reservoir state and wet masses, not only particles."""
    state = {"physics": scene.get_state(), "absorption": handles["coupling"].capture_state()}
    if handles.get("grasp_adapter"):
        from robot_grasp import capture_grasp_state
        state["grasp"] = capture_grasp_state(handles["cloth"])
    return state


def reset_scene(scene, handles):
    """Reset robot/particle state and restore the initial arm position targets."""
    scene.reset(handles["initial_state"])
    for side, robot in handles["robots"].items():
        robot.control_dofs_position(handles["home"], handles["robot_dofs"][side])


def save_views(handles, output, prefix=""):
    for name, camera in handles["cameras"].items():
        rgb, *_ = camera.render()
        Image.fromarray(np.asarray(rgb)).save(output / f"{prefix}{name}.png")


def validate(handles, config, initial_soy, steps):
    all_liquid, cloth = (numpy(handles[k].get_particles_pos()) for k in ("soy", "cloth"))
    active = numpy(handles["soy"].get_particles_active()).astype(bool)
    liquid = all_liquid[active]
    th = config["table"]["height"]
    errors = []
    for label, positions in (("liquid", liquid), ("cloth", cloth)):
        if len(positions) == 0:
            continue
        if not np.isfinite(positions).all():
            errors.append(f"{label}: nonfinite particle position")
        if positions[:, 2].min() < th-0.002:
            errors.append(f"{label}: penetrated tabletop")
        if positions[:, 2].max() > th+0.05:
            errors.append(f"{label}: unstable height above tabletop")
        if np.any(np.abs(positions[:, :2]) > np.array(config["table"]["size"][:2])/2):
            errors.append(f"{label}: left tabletop")
    liquid_velocity = numpy(handles["soy"].get_particles_vel())[active]
    cloth_velocity = numpy(handles["cloth"].get_particles_vel())
    if not np.isfinite(liquid_velocity).all() or not np.isfinite(cloth_velocity).all():
        errors.append("Nonfinite particle velocity")
    if not 500 < len(all_liquid) < 50000:
        errors.append("Unexpected liquid particle count")
    if len(liquid) and np.max(np.linalg.norm(liquid-initial_soy[active], axis=1)) < 0.0001 and steps > 10:
        errors.append("Liquid particles did not respond to gravity")
    robot_error = {}
    for side, robot in handles["robots"].items():
        q = numpy(robot.get_dofs_position(handles["robot_dofs"][side]))
        robot_error[side] = float(np.max(np.abs(q-handles["home"])))
        if not np.isfinite(q).all() or robot_error[side] > 0.05:
            errors.append(f"{side} arm failed to hold home pose")
    absorption = handles["coupling"].metrics() if "coupling" in handles else None
    if absorption is not None:
        for key in ("mass_balance_error_kg", "cloth_loading_error_kg", "vertex_reservoir_error_kg"):
            if abs(absorption[key]) > 1e-7:
                errors.append(f"Coupling {key} exceeds tolerance")
        if absorption["maximum_saturation"] > 1.0+1e-6:
            errors.append("Cloth absorption exceeds local capacity")
    report = {
        "passed": not errors, "errors": errors, "genesis_version": version("genesis-world"),
        "backend": handles["backend"], "steps": steps,
        "simulated_seconds": steps*config["simulation"]["dt"],
        "models": {"soy": "SPH.Liquid / "+config["soy"]["pressure_solver"], "cloth": "PBD.Cloth",
                   "robots": "two independent articulated Franka Panda, 9 DOFs each"},
        "liquid_particles": len(all_liquid), "active_liquid_particles": len(liquid), "cloth_particles": len(cloth),
        "liquid_bounds_m": [liquid.min(axis=0).tolist(), liquid.max(axis=0).tolist()] if len(liquid) else None,
        "cloth_bounds_m": [cloth.min(axis=0).tolist(), cloth.max(axis=0).tolist()],
        "liquid_displacement_max_m": float(np.linalg.norm(all_liquid-initial_soy, axis=1).max()),
        "liquid_displacement_mean_m": float(np.linalg.norm(all_liquid-initial_soy, axis=1).mean()),
        "liquid_planar_displacement_mean_m": float(np.linalg.norm((all_liquid-initial_soy)[:, :2], axis=1).mean()),
        "liquid_speed_max_m_s": float(np.linalg.norm(liquid_velocity, axis=1).max()) if len(liquid) else 0.0,
        "cloth_speed_max_m_s": float(np.linalg.norm(cloth_velocity, axis=1).max()),
        "robot_home_error_max_rad_or_m": robot_error,
        "limitations": ["Illustrative soy and cotton parameters; not measured or calibrated.",
                        "Native rigid-SPH and rigid-PBD contacts enabled.",
                        "SPH-PBD uses an outer-step local porous-sheet adapter; parameters are uncalibrated.",
                        "Static mode uses native PBD-rigid contacts; robot mode adds scene-local Coulomb friction.",
                        "Static mode holds home pose; --robot-wipe runs the full dual-arm task."],
    }
    if absorption is not None:
        report["absorption"] = absorption
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--backend", choices=("cuda", "cpu", "auto"), default="cuda")
    parser.add_argument("--steps", type=int, help="Finite steps; live viewer stays open when omitted.")
    parser.add_argument("--output", type=Path, default=HERE/"output")
    parser.add_argument("--config", type=Path, default=HERE/"config.json")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=900)
    parser.add_argument("--liquid-view", choices=("recon", "particle"), default="recon")
    parser.add_argument("--record", action="store_true", help="Save a 30fps simulation clip.")
    parser.add_argument("--check-reset", action="store_true", help="Verify reset after a finite run.")
    parser.add_argument("--flow-probe", action="store_true", help="After settling, apply a 0.08m/s lateral liquid impulse.")
    parser.add_argument("--demo-wipe", action="store_true", help="Controlled cloth-fixture sweep/lift test, not a robot grasp policy.")
    parser.add_argument("--robot-wipe", action="store_true", help="Run dual-arm grasp, carry, lay, press, regrasp and lift.")
    parser.add_argument("--hold-open", action="store_true", help="Keep the viewer open after the robot task.")
    parser.add_argument("--resume-robot-checkpoint", type=Path, help="Resume a trusted local robot task checkpoint.")
    args = parser.parse_args()
    if args.robot_wipe and args.config == HERE/"config.json":
        args.config = HERE/"config_robot_wipe.json"
    config = json.loads(args.config.read_text())
    if args.robot_wipe:
        from robot_wipe import run
        return run(args, config)
    if args.demo_wipe:
        from wiping_demo import run
        return run(args, config)
    if args.steps is not None and args.steps < 1:
        parser.error("--steps must be positive")
    if args.record and args.steps is None:
        parser.error("--record requires --steps")
    if args.check_reset and args.steps is None:
        parser.error("--check-reset requires --steps")
    if args.flow_probe and (args.steps is None or args.steps < config["simulation"]["settle_steps"]+100):
        parser.error("--flow-probe requires at least settle_steps + 100 finite steps")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    (output/"config_used.json").write_text(json.dumps(config, indent=2)+"\n")
    scene, handles = build_scene(config, headless=args.headless, backend=args.backend,
        width=args.width, height=args.height, liquid_view=args.liquid_view)
    initial_soy = numpy(handles["soy"].get_particles_pos()).copy()
    initial_cloth = numpy(handles["cloth"].get_particles_pos()).copy()
    steps = args.steps if args.steps is not None else (500 if args.headless else None)
    report_step = min(config["simulation"]["settle_steps"], steps or 10**9)
    frame_stride = max(1, round(1/(30*config["simulation"]["dt"])))
    video_writer = None
    if args.record:
        import imageio.v2 as imageio
        video_writer = imageio.get_writer(output/"liquid_motion.mp4",
            fps=1/(frame_stride*config["simulation"]["dt"]), quality=8, macro_block_size=2)
    frame = 0
    probe_initial = None
    start = time.monotonic()
    try:
        while steps is None or frame < steps:
            if not args.headless and not scene.viewer.is_alive():
                break
            scene.step()
            frame += 1
            if args.record and frame % frame_stride == 0:
                rgb, *_ = handles["cameras"]["detail"].render()
                video_writer.append_data(np.asarray(rgb))
            if frame == report_step:
                save_views(handles, output)
                report = validate(handles, config, initial_soy, frame)
                (output/"validation.json").write_text(json.dumps(report, indent=2)+"\n")
                np.savez_compressed(output/"settled_state.npz",
                    soy=numpy(handles["soy"].get_particles_pos()),
                    cloth=numpy(handles["cloth"].get_particles_pos()),
                    **{f"{side}_qpos": numpy(robot.get_qpos()) for side, robot in handles["robots"].items()})
                print(json.dumps(report, indent=2), flush=True)
                if not report["passed"]:
                    raise RuntimeError("Scene validation failed: "+"; ".join(report["errors"]))
                if args.flow_probe:
                    probe_initial = numpy(handles["soy"].get_particles_pos()).copy()
                    handles["soy"].set_particles_vel(np.tile([0.08, 0, 0], (len(probe_initial), 1)))
            if frame % 250 == 0:
                print(f"step={frame}, simulated={frame*config['simulation']['dt']:.2f}s, wall={time.monotonic()-start:.1f}s", flush=True)
        if frame:
            save_views(handles, output, "final_")
            report = validate(handles, config, initial_soy, frame)
            if probe_initial is not None:
                final = numpy(handles["soy"].get_particles_pos())
                report["flow_probe"] = {"initial_velocity_m_s": [0.08,0,0],
                    "centroid_displacement_m": (final.mean(0)-probe_initial.mean(0)).tolist(),
                    "max_particle_displacement_m": float(np.linalg.norm(final-probe_initial, axis=1).max())}
                probe_passed = bool(np.isfinite(final).all() and
                    report["flow_probe"]["centroid_displacement_m"][0] > 0.0005)
                report["flow_probe"]["passed"] = probe_passed
                if not probe_passed:
                    report["passed"] = False
                    report["errors"].append("Liquid failed to move in response to lateral impulse")
            if args.check_reset:
                reset_scene(scene, handles)
                reset_error = max(
                    float(np.max(np.abs(numpy(handles["soy"].get_particles_pos())-initial_soy))),
                    float(np.max(np.abs(numpy(handles["cloth"].get_particles_pos())-initial_cloth))),
                    *(float(np.max(np.abs(numpy(robot.get_dofs_position(handles["robot_dofs"][side]))-handles["home"])))
                      for side, robot in handles["robots"].items()))
                report["reset_check"] = {"passed": reset_error < 1e-6, "max_error": reset_error}
                if reset_error >= 1e-6:
                    report["passed"] = False
                    report["errors"].append("Reset failed to restore initial positions")
            (output/"final_validation.json").write_text(json.dumps(report, indent=2)+"\n")
            if not report["passed"]:
                raise RuntimeError("Final validation failed: "+"; ".join(report["errors"]))
        print(f"Outputs: {output}", flush=True)
    finally:
        if video_writer is not None:
            video_writer.close()
        import genesis as gs
        gs.destroy()


if __name__ == "__main__":
    main()
