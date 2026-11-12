"""Short contact diagnostic, with arms INITIALIZED above the cloth corners.

This deliberately skips the home approach and is not a full-task result/video.
Only initial robot state is set; lowering, closure and lift use the controller.
"""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scene import build_scene
from robot_wipe import RobotWipe
from robot_geometry import grasp_targets


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("config_robot_wipe.json"))
    parser.add_argument("--output", type=Path, default=Path("output/corner_pick_probe"))
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    args.output.mkdir(parents=True, exist_ok=True)
    scene, h = build_scene(cfg, headless=True, width=800, height=600)
    options = SimpleNamespace(headless=True, record=False, steps=0, resume_robot_checkpoint=None)
    task = RobotWipe(scene, h, cfg, options, args.output)
    center = np.asarray(cfg["cloth"]["position_xy"])
    for side, target in grasp_targets(cfg, center, .79).items():
        q, error, _ = task.planner.solve(side, target)
        if error > 1e-5:
            raise RuntimeError("Probe initialization is unreachable")
        task.q[side] = np.r_[q, [.008, .008]]
        h["robots"][side].set_dofs_position(task.q[side], h["robot_dofs"][side])
        h["robots"][side].control_dofs_position(task.q[side], h["robot_dofs"][side])
    task.hold(.4)
    task.move("Probe: lower to diagonal corners", grasp_targets(cfg, center, .770), 1.1)
    task.hold(.2)
    preferred = task.corner_candidates()
    success = task.grasp("Probe: actual bilateral pinch", preferred, required=False)
    if not success:
        task.move("Probe: refine grasp height", grasp_targets(cfg, center, .7698), .4)
        task.hold(.2)
        success = task.grasp("Probe: contact recheck", preferred, close=False, required=False)
    task.snapshot("pinch")
    if success:
        task.move("Probe: lift with both corners", grasp_targets(cfg, center, .9), 1.2)
        task.hold(.3)
        task.snapshot("lift")
    report = {"scope": "Short native corner-pick diagnostic; robot starts above cloth, not a full task.",
              "grasped": success, "contact_model": h["cloth_contact_model"],
              "material_profile": h.get("material_profile"),
              "liquid_and_mass": h["coupling"].metrics(),
              "stages": task.fabric_stages, "contacts": task.attached}
    (args.output/"probe_report.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report), flush=True)
    import genesis as gs
    gs.destroy()


if __name__ == "__main__":
    main()
