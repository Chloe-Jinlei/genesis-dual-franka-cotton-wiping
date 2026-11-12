"""Controlled edge-fixture sweep/lift validation of cloth/liquid coupling.

The two Frankas stay at home. A prescribed laboratory fixture moves two cloth
edge strips; the interior stays deformable. This isolates the coupling from a
future robot-grasp/controller and is labeled as such in every saved report.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image


def _array(tensor):
    return tensor.detach().cpu().numpy()


def _smooth(u):
    u = np.clip(u, 0.0, 1.0)
    return u*u*(3.0-2.0*u)


def trajectory(t):
    return np.array([-0.44*_smooth((t-1.0)/5.0), 0.0,
                     0.12*_smooth((t-8.0)/2.0)], dtype=np.float32)


class EdgeFixture:
    def __init__(self, cloth):
        self.cloth = cloth
        self.reference = None
        self.indices = None

    def apply(self, t, dt):
        if t < 1.0-1e-8:
            return
        if self.reference is None:
            self.reference = _array(self.cloth.get_particles_pos()).copy()
            x = self.reference[:, 0]
            self.indices = np.flatnonzero((x < x.min()+0.004) | (x > x.max()-0.004))
            self.cloth.fix_particles(self.indices)
        a, b = trajectory(t), trajectory(t+dt)
        self.cloth.set_particles_pos(self.reference[self.indices]+a,
                                     particles_idx_local=self.indices)
        self.cloth.set_particles_vel(np.tile((b-a)/dt, (len(self.indices), 1)),
                                     particles_idx_local=self.indices)


def run(args, config):
    import genesis as gs
    import imageio.v2 as imageio
    from scene import build_scene, reset_scene, save_views

    if not config.get("coupling", {}).get("enabled", True):
        raise ValueError("The wipe demonstration requires coupling.enabled=true.")
    output = args.output.resolve()
    if output.name == "output":
        output = output/"wipe_demo"
    output.mkdir(parents=True, exist_ok=True)
    (output/"config_used.json").write_text(json.dumps(config, indent=2)+"\n")
    scene, handles = build_scene(config, headless=args.headless, backend=args.backend,
        width=args.width, height=args.height, liquid_view=args.liquid_view)
    cloth, soy, coupling = (handles[k] for k in ("cloth", "soy", "coupling"))
    fixture = EdgeFixture(cloth)
    liquid_masses = soy.solver.particles_info.mass.to_numpy()[soy.particle_start:soy.particle_end].astype(float)
    dry_masses = cloth.solver.particles_info.mass.to_numpy()[cloth.particle_start:cloth.particle_end].astype(float)
    initial_mass = float(liquid_masses.sum())
    initial_soy = _array(soy.get_particles_pos()).copy()
    initial_cloth = _array(cloth.get_particles_pos()).copy()
    dt = config["simulation"]["dt"]
    steps = args.steps or round(12.0/dt)
    stride = max(1, round(1/(30*dt)))
    writer = imageio.get_writer(output/"wipe_fixture_demo.mp4",
        fps=1/(stride*dt), quality=8, macro_block_size=2) if args.record else None
    rows = []
    checks = {}
    max_mass_error = 0.0
    max_loading_error = 0.0
    max_saturation = 0.0
    before_lift = None
    started = time.monotonic()
    actual_steps = 0
    try:
        for i in range(steps):
            if not args.headless and not scene.viewer.is_alive():
                break
            fixture.apply(i*dt, dt)
            scene.step()
            actual_steps = i+1
            t = actual_steps*dt
            active = _array(soy.get_particles_active()).astype(bool)
            absorbed = float(coupling.vertex_absorbed_mass.sum())
            free_mass = float(liquid_masses[active].sum())
            mass_error = abs(free_mass+absorbed-initial_mass)
            native_mass = cloth.solver.particles_info.mass.to_numpy()[cloth.particle_start:cloth.particle_end].astype(float)
            loading_error = abs(float(native_mass.sum()-dry_masses.sum())-absorbed)
            max_mass_error = max(max_mass_error, mass_error)
            max_loading_error = max(max_loading_error, loading_error)
            max_saturation = max(max_saturation, float(coupling.vertex_saturation.max()))
            if actual_steps % 25 == 0:
                positions = _array(soy.get_particles_pos())
                free_z = float(positions[active, 2].min()) if active.any() else None
                rows.append({"time_s": t, "free_liquid_g": free_mass*1000,
                    "absorbed_liquid_g": absorbed*1000, "mass_error_kg": mass_error,
                    "max_saturation": float(coupling.vertex_saturation.max()),
                    "free_liquid_min_z_m": free_z})
                if not np.isfinite(positions).all() or not np.isfinite(_array(cloth.get_particles_pos())).all():
                    raise RuntimeError("Nonfinite cloth/liquid state during wipe test")
            if abs(t-1.0) < dt/2:
                checks["no_remote_absorption"] = absorbed == 0.0
                save_views(handles, output, "before_")
            if abs(t-8.0) < dt/2:
                before_lift = {"absorbed_kg": absorbed,
                    "bound_ids": coupling.bound_particle_indices.copy(),
                    "bound_positions": coupling.bound_positions.copy()}
                save_views(handles, output, "after_sweep_")
            if writer is not None and actual_steps % stride == 0:
                rgb, *_ = handles["cameras"]["detail"].render()
                writer.append_data(np.asarray(rgb))
            if actual_steps % 250 == 0:
                print(f"wipe t={t:.1f}s, absorbed={absorbed*1000:.2f}g, free={free_mass*1000:.2f}g, wall={time.monotonic()-started:.1f}s", flush=True)
        save_views(handles, output, "lifted_")
        active = _array(soy.get_particles_active()).astype(bool)
        bound_ids = coupling.bound_particle_indices
        bound_positions = coupling.bound_positions
        absorbed = float(coupling.vertex_absorbed_mass.sum())
        checks.update({"completed_sweep_and_lift": actual_steps*dt >= 12.0-1e-8,
            "absorbed_at_least_5g": absorbed > 0.005,
            "mass_conserved": max_mass_error < 1e-7,
            "wet_mass_in_cloth_dynamics": max_loading_error < 1e-7,
            "finite_local_capacity": max_saturation <= 1.0+1e-6,
            "no_duplicate_bound_particles": len(np.unique(bound_ids)) == len(bound_ids)})
        if before_lift is not None and len(before_lift["bound_ids"]):
            _, old, new = np.intersect1d(before_lift["bound_ids"], bound_ids, return_indices=True)
            rise = bound_positions[new, 2]-before_lift["bound_positions"][old, 2]
            checks["absorbed_liquid_carried_up"] = len(old) == len(before_lift["bound_ids"]) and float(np.median(rise)) > 0.04
            carried_rise = float(np.median(rise))
        else:
            checks["absorbed_liquid_carried_up"] = False
            carried_rise = 0.0
        state = {"active_liquid": _array(soy.get_particles_pos())[active],
                 "cloth": _array(cloth.get_particles_pos()),
                 "bound_liquid": bound_positions,
                 "bound_ids": bound_ids,
                 "cloth_absorbed_mass": coupling.vertex_absorbed_mass,
                 "cloth_saturation": coupling.vertex_saturation}
        np.savez_compressed(output/"final_coupled_state.npz", **state)
        final_metrics = coupling.metrics()
        reset_scene(scene, handles)
        reset_error = max(float(np.abs(_array(soy.get_particles_pos())-initial_soy).max()),
                          float(np.abs(_array(cloth.get_particles_pos())-initial_cloth).max()))
        reset_mass = cloth.solver.particles_info.mass.to_numpy()[cloth.particle_start:cloth.particle_end]
        checks["complete_reset"] = bool(reset_error < 1e-6 and
            np.allclose(reset_mass, dry_masses, atol=1e-10, rtol=0) and
            coupling.vertex_absorbed_mass.sum() == 0 and
            _array(soy.get_particles_active()).all())
        report = {"passed": bool(all(checks.values())), "checks": checks,
            "demo_type": "Prescribed cloth edge fixture; robot arms stay at home. Not an autonomous robot wiping controller.",
            "simulated_seconds": actual_steps*dt, "initial_liquid_g": initial_mass*1000,
            "absorbed_liquid_g": absorbed*1000,
            "picked_up_fraction": absorbed/initial_mass,
            "max_mass_balance_error_kg": max_mass_error,
            "max_native_cloth_loading_error_kg": max_loading_error,
            "median_carried_liquid_rise_m": carried_rise,
            "fixture_nodes": 0 if fixture.indices is None else len(fixture.indices),
            "free_cloth_nodes": cloth.n_particles-(0 if fixture.indices is None else len(fixture.indices)),
            "final_coupling_metrics": final_metrics,
            "limitations": ["Uncalibrated outer-step porous-sheet model.",
                "No evaporation, wringing, desorption, molecular diffusion, or dissolved-stain chemistry.",
                "No autonomous robot grasp or wiping controller."]}
        (output/"wipe_validation.json").write_text(json.dumps(report, indent=2)+"\n")
        if rows:
            with (output/"mass_history.csv").open("w", newline="") as stream:
                out = csv.DictWriter(stream, fieldnames=rows[0].keys())
                out.writeheader()
                out.writerows(rows)
        print(json.dumps(report, indent=2), flush=True)
        if not report["passed"]:
            raise RuntimeError("Wipe validation failed: "+", ".join(k for k,v in checks.items() if not v))
    finally:
        if writer is not None:
            writer.close()
        gs.destroy()
