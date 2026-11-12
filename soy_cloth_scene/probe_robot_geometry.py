"""CPU-only trajectory feasibility check; never initializes Genesis."""
import argparse
import json
from pathlib import Path

import numpy as np

from robot_geometry import Planner, SIDES, recommended_waypoints


def probe(config=None, samples=25):
    planner = Planner(config)
    seeds = {side: planner.home.copy() for side in SIDES}
    stages = []
    for name, targets in recommended_waypoints(planner.config["table"]["height"]).items():
        path = planner.plan_cartesian(seeds, targets, samples=samples)
        minimum = {}
        max_joint_step = 0.0
        for index, entry in enumerate(path):
            clearance = planner.clearance(entry["q"], finger_opening=.003 if name in ("grasp", "lay") else .04)
            for key, value in clearance.items():
                if key not in minimum or value["distance"] < minimum[key]["distance"]:
                    minimum[key] = value
            if index:
                max_joint_step = max(max_joint_step, *(float(np.max(np.abs(entry["q"][s]-path[index-1]["q"][s])))
                                                      for s in SIDES))
        seeds = {side: path[-1]["q"][side].copy() for side in SIDES}
        stages.append({"name": name, "targets": {s: v.tolist() for s, v in targets.items()},
                       "q": {s: v.tolist() for s, v in seeds.items()},
                       "max_tcp_error_m": max(e for item in path for e in item["position_error"].values()),
                       "max_rotation_error_rad": max(e for item in path for e in item["angle_error"].values()),
                       "max_joint_sample_step_rad": max_joint_step,
                       "minimum_clearance_m": minimum})
    return {"passed": all(v["distance"] >= 0 for stage in stages
                           for v in stage["minimum_clearance_m"].values()),
            "samples_per_stage": samples, "stages": stages,
            "limitations": ["Offline original MJCF convex-geometry checks; Genesis runtime tracking still required.",
                            "Cloth/liquid contacts intentionally excluded from obstacle checks."]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--samples", type=int, default=25)
    args = parser.parse_args()
    result = probe(json.loads(args.config.read_text()) if args.config else None, args.samples)
    payload = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload)
    print(payload)
    raise SystemExit(0 if result["passed"] else 1)
