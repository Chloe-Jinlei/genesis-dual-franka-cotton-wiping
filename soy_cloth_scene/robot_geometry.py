"""Offline seven-joint Panda FK, IK and coarse dual-arm clearance checks.

The model is compiled by MuJoCo only; this module does not initialize Genesis.
TCP is the centre of the original Panda fingertip pads, hand-local (0,0,.1029).
Collision distances use the original MJCF convex meshes, so final validation
must also inspect the running Genesis contacts and measured TCP tracking.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

HERE = Path(__file__).resolve().parent
ASSET = HERE.parent / "genesis-world/genesis/assets/xml/franka_emika_panda"
TCP = np.array([0.0, 0.0, 0.1029])
DOWN = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])
PAD_BELOW_TCP = 0.0085
# The original finger collision mesh projects farther than its box-shaped pad.
FINGERTIP_BELOW_TCP = 0.009358
SIDES = ("left", "right")


def grasp_offsets(config):
    """Return authored cloth-local XY grasp points, without an outward stretch.

    The cloth asset is 0.28 x 0.18 m. Diagonal points such as left=(-.13,.08),
    right=(.13,-.08) remain 10 mm inside its corners. The same offsets must be
    used during grasp, carry and lay; actual attached material points still need
    strain measurements because their local offsets can differ from the TCPs.
    """
    task = config.get("robot_task", {})
    half = float(task.get("grasp_half_span_m", .13))
    authored = task.get("grasp_offsets_xy_m", {"left": [-half, 0.], "right": [half, 0.]})
    if not isinstance(authored, dict) or set(authored) != set(SIDES):
        raise ValueError("grasp_offsets_xy_m must contain left and right XY points")
    offsets = {side: np.asarray(authored[side], dtype=float).copy() for side in SIDES}
    if any(point.shape != (2,) or not np.isfinite(point).all() for point in offsets.values()):
        raise ValueError("Each grasp offset must be one finite XY point")
    if any(np.any(np.abs(point) > np.array([.14, .09])) for point in offsets.values()):
        raise ValueError("Grasp offsets must lie on the unchanged 0.28 x 0.18 m cloth")
    if np.linalg.norm(offsets["right"]-offsets["left"]) < 1e-6:
        raise ValueError("The two cloth grasp points must be distinct")
    return offsets


def grasp_targets(config, center, z):
    """Translate both fixed cloth-local grasp points to a world XY centre."""
    center = np.asarray(center, dtype=float)
    if center.shape != (2,) or not np.isfinite(center).all() or not np.isfinite(z):
        raise ValueError("Grasp target requires a finite XY centre and height")
    return {side: np.r_[center+offset, float(z)]
            for side, offset in grasp_offsets(config).items()}


def make_model(config):
    root = ET.parse(ASSET / "panda.xml").getroot()
    root.find("compiler").set("meshdir", str(ASSET / "assets"))
    for name in ("actuator", "keyframe", "tendon", "equality"):
        node = root.find(name)
        if node is not None:
            root.remove(node)
    world = root.find("worldbody")
    template = world.find("body")
    world.remove(template)
    for side in SIDES:
        body = copy.deepcopy(template)
        for element in body.iter():
            if "name" in element.attrib:
                element.set("name", side + "_" + element.get("name"))
        yaw = config["robots"][side + "_yaw"]
        base = config["robots"][side + "_base"]
        body.set("pos", " ".join(map(str, base)))
        body.set("quat", f"{math.cos(yaw/2)} 0 0 {math.sin(yaw/2)}")
        world.append(body)
        th = config["table"]["height"]
        ET.SubElement(world, "geom", name=side + "_mount", type="box",
            pos=f"{base[0]} {base[1]} {(th+base[2])/2}",
            size=f".095 .095 {(base[2]-th)/2}")
    sx, sy, sz = config["table"]["size"]
    th = config["table"]["height"]
    ET.SubElement(world, "geom", name="table", type="box",
        pos=f"0 0 {th-sz/2}", size=f"{sx/2} {sy/2} {sz/2}")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    return model, mujoco.MjData(model)


class Planner:
    def __init__(self, config=None):
        self.config = config if config is not None else json.loads((HERE / "config.json").read_text())
        self.home = np.asarray(self.config["robots"]["home"], dtype=float)
        self.m, self.d = make_model(self.config)
        self.qindices = {s: np.array([self.m.joint(f"{s}_joint{i}").qposadr[0]
                                     for i in range(1, 8)]) for s in SIDES}
        self.bodies = {s: self.m.body(s + "_hand").id for s in SIDES}
        self.limits = {s: self.m.jnt_range[[self.m.joint(f"{s}_joint{i}").id
                                          for i in range(1, 8)]].copy() for s in SIDES}
        self.geoms = {s: [i for i in range(self.m.ngeom) if self.m.geom_contype[i]
                         and self.body_name(i).startswith(s + "_")
                         and not self.body_name(i).endswith("link0")] for s in SIDES}
        self.obstacles = [self.m.geom(n).id for n in ("table", "left_mount", "right_mount")]
        self.self_pairs = {}
        for side, geoms in self.geoms.items():
            pairs = []
            for k, first in enumerate(geoms):
                for second in geoms[k+1:]:
                    a, b = [int(self.m.body_weldid[int(self.m.geom_bodyid[g])])
                            for g in (first, second)]
                    if all(self.body_name(g).endswith("_finger") for g in (first, second)):
                        continue
                    if a == b or int(self.m.body_weldid[int(self.m.body_parentid[a])]) == b \
                            or int(self.m.body_weldid[int(self.m.body_parentid[b])]) == a:
                        continue
                    pairs.append((first, second))
            self.self_pairs[side] = pairs
            self.d.qpos[self.qindices[side]] = self.home
        self.set_fingers(0.04)
        mujoco.mj_forward(self.m, self.d)

    def body_name(self, geom):
        return self.m.body(int(self.m.geom_bodyid[geom])).name

    def set_fingers(self, opening):
        """Set each finger joint displacement (half the nominal jaw opening)."""
        for side in SIDES:
            value = opening[side] if isinstance(opening, dict) else opening
            for finger in (1, 2):
                self.d.qpos[self.m.joint(f"{side}_finger_joint{finger}").qposadr[0]] = value

    def fk(self, side, q):
        self.d.qpos[self.qindices[side]] = np.asarray(q)
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)
        body = self.bodies[side]
        matrix = self.d.xmat[body].reshape(3, 3)
        return self.d.xpos[body].copy() + matrix @ TCP, Rotation.from_matrix(matrix)

    def solve(self, side, p, r=DOWN, seed=None, *, max_nfev=180):
        """Return (seven joint angles, TCP error in m, rotation error in rad)."""
        p = np.asarray(p, dtype=float)
        seed = self.home.copy() if seed is None else np.asarray(seed, dtype=float)
        if not isinstance(r, Rotation):
            raise TypeError("r must be scipy.spatial.transform.Rotation")

        def residual(q):
            fp, fr = self.fk(side, q)
            return np.r_[5.0 * (fp-p), (r*fr.inv()).as_rotvec(), 0.0002*(q-seed)]

        limits = self.limits[side]
        solution = least_squares(residual,
            np.clip(seed, limits[:, 0]+1e-6, limits[:, 1]-1e-6),
            bounds=(limits[:, 0]+1e-6, limits[:, 1]-1e-6),
            ftol=1e-10, gtol=1e-9, xtol=1e-10, max_nfev=max_nfev)
        fp, fr = self.fk(side, solution.x)
        return solution.x.copy(), float(np.linalg.norm(fp-p)), float((r*fr.inv()).magnitude())

    def clearance(self, qs, finger_opening=None):
        """Return minimum signed distances; negative means mesh penetration."""
        for side, q in qs.items():
            self.d.qpos[self.qindices[side]] = q
        if finger_opening is not None:
            self.set_fingers(finger_opening)
        mujoco.mj_forward(self.m, self.d)

        def minimum(pairs):
            distance, first, second = min((float(mujoco.mj_geomDistance(
                self.m, self.d, first, second, 0.20, None)), first, second)
                for first, second in pairs)
            return {"distance": distance, "bodies": [self.body_name(first), self.body_name(second)],
                    "geoms": [self.m.geom(first).name, self.m.geom(second).name]}

        out = {}
        for side in qs:
            out[side + "_self"] = minimum(self.self_pairs[side])
            out[side + "_table"] = minimum((g, self.obstacles[0]) for g in self.geoms[side])
            out[side + "_mounts"] = minimum((g, obstacle) for g in self.geoms[side]
                                               for obstacle in self.obstacles[1:])
        out["arms"] = minimum((a, b) for a in self.geoms["left"] for b in self.geoms["right"])
        return out

    def plan_cartesian(self, start_qs, targets, *, rotations=None, samples=30,
                       position_tolerance=0.0005, angle_tolerance=0.002):
        """Sequential warm-start IK for straight Cartesian paths with Slerp.

        Return samples including endpoints: {fraction, q, tcp, position_error,
        angle_error}. Does not silently accept unreachable targets. Collision
        checks are separate because the caller knows allowed grasp contacts.
        """
        if samples < 2:
            raise ValueError("samples must be at least two")
        seeds = {s: np.asarray(q, dtype=float).copy() for s, q in start_qs.items()}
        start = {s: self.fk(s, seeds[s]) for s in targets}
        rotations = {s: DOWN for s in targets} if rotations is None else rotations
        interpolators = {s: Slerp([0, 1], Rotation.from_quat(
            [start[s][1].as_quat(), rotations[s].as_quat()])) for s in targets}
        result = []
        for fraction in np.linspace(0, 1, samples):
            entry = {"fraction": float(fraction), "q": {}, "tcp": {},
                     "position_error": {}, "angle_error": {}}
            for side, target in targets.items():
                pos = (1-fraction)*start[side][0] + fraction*np.asarray(target)
                q, error, angle = self.solve(side, pos, interpolators[side](fraction), seeds[side])
                if error > position_tolerance or angle > angle_tolerance:
                    raise ValueError(f"Unreachable {side} Cartesian sample {fraction:.3f}: "
                                     f"position={error:.6f}m, rotation={angle:.6f}rad")
                seeds[side] = q
                entry["q"][side] = q.copy()
                entry["tcp"][side] = pos.copy()
                entry["position_error"][side] = error
                entry["angle_error"][side] = angle
            result.append(entry)
        return result


def recommended_waypoints(table_height=0.76, pinch_height=0.012):
    """Default paired TCP targets for side-edge pick, transfer and final lift."""
    def pair(y, z):
        return {"left": np.array([-0.13, y, z]), "right": np.array([0.13, y, z])}
    return {
        "pregrasp": pair(-0.23, table_height+0.18),
        "grasp": pair(-0.23, table_height+pinch_height),
        "lift": pair(-0.23, table_height+0.18),
        "transfer": pair(0.04, table_height+0.18),
        "lay": pair(0.04, table_height+pinch_height),
        "final_lift": pair(0.04, table_height+0.24),
    }
