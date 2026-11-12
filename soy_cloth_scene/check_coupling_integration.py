"""Native GPU edge cases: empty liquid render, wet checkpoints, and dry reset.

Uses an explicitly high-capacity, fast-absorption test configuration, not the
scene's default material settings. The everyday finite-capacity wipe is tested
separately by wiping_demo.py.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from PIL import Image

from scene import build_scene, capture_coupled_state, numpy, reset_scene

HERE = Path(__file__).resolve().parent


def main():
    import genesis as gs
    cfg = json.loads((HERE/"config.json").read_text())
    cfg["cloth"]["position_xy"] = cfg["soy"]["position_xy"].copy()
    cfg["cloth"]["initial_clearance"] = 0.003
    cfg["coupling"].update(capacity_kg_m2=100.0, absorption_flux_kg_m2_s=1e6,
                           minimum_contact_time_s=0.0)
    output = HERE/"output"/"coupling_edge_checks"
    output.mkdir(parents=True, exist_ok=True)
    (output/"test_config.json").write_text(json.dumps(cfg, indent=2)+"\n")
    scene, h = build_scene(cfg, headless=True, width=800, height=600)
    cloth, soy, coupling = (h[k] for k in ("cloth", "soy", "coupling"))
    checks = {}
    try:
        cloth.fix_particles()
        for step in range(100):
            scene.step()
            if not numpy(soy.get_particles_active()).any():
                break
        checks["all_liquid_transferred_in_high_capacity_variant"] = bool(not numpy(soy.get_particles_active()).any())
        scene.step()
        scene.step()
        rgb, *_ = h["cameras"]["detail"].render()
        checks["empty_free_liquid_render"] = bool(np.asarray(rgb).shape == (600,800,3))
        Image.fromarray(np.asarray(rgb)).save(output/"all_absorbed.png")
        checkpoint = capture_coupled_state(scene, h)
        bound0 = coupling.bound_positions.copy()
        cloth0 = numpy(cloth.get_particles_pos()).copy()
        loaded_mass0 = cloth.solver.particles_info.mass.to_numpy().copy()
        cloth.set_particles_pos(cloth0+[0,0,0.10])
        cloth.set_particles_vel(np.zeros_like(cloth0))
        scene.step()
        lift_error = float(np.max(np.abs(coupling.bound_positions-bound0-[0,0,0.10])))
        checks["entire_reservoir_follows_cloth"] = lift_error < 1e-5
        scene.reset(checkpoint)
        checks["wet_checkpoint_restores_positions"] = bool(np.allclose(coupling.bound_positions, bound0, atol=1e-6, rtol=0))
        checks["wet_checkpoint_restores_mass"] = bool(np.allclose(cloth.solver.particles_info.mass.to_numpy(), loaded_mass0, atol=1e-10, rtol=0))
        checks["wet_checkpoint_restores_inactive_liquid"] = bool(not numpy(soy.get_particles_active()).any())
        scene.step()
        metrics = coupling.metrics()
        checks["native_mass_conserved"] = abs(metrics["mass_balance_error_kg"]) < 1e-7
        checks["native_wet_loading_matches_reservoir"] = abs(metrics["cloth_loading_error_kg"]) < 1e-7
        reset_scene(scene, h)
        checks["reset_reactivates_all_liquid"] = bool(numpy(soy.get_particles_active()).all())
        checks["reset_clears_reservoir_and_stain"] = bool(coupling.vertex_absorbed_mass.sum() == 0 and h["wet_visual"].visual_saturation.max() == 0)
        dry = cloth.solver.particles_info.mass.to_numpy()[cloth.particle_start:cloth.particle_end]
        checks["reset_restores_dry_mass"] = bool(np.allclose(dry, coupling.dry_vertex_mass, atol=1e-10, rtol=0))
        report = {"passed": bool(all(checks.values())), "checks": checks,
                  "test_variant": "High capacity and high flux for edge-case testing only",
                  "capture_steps": step+1, "lift_attachment_error_m": lift_error,
                  "captured_metrics": metrics}
        (output/"validation.json").write_text(json.dumps(report,indent=2)+"\n")
        print(json.dumps(report,indent=2), flush=True)
        if not report["passed"]:
            raise RuntimeError("Native coupling edge checks failed")
    finally:
        gs.destroy()


if __name__ == "__main__":
    main()
