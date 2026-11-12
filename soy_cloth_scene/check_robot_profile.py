"""CPU capacity/flux feasibility for the smaller robot-wiping puddle.

Reproduces Genesis's regular box sampling + signed-distance rejection and its
PBD material-aligned grid (or legacy remeshing). Exercises the real coupling operator against IO stubs, with
a stationary sheet. This is NOT a substitute for native SPH/PBD or robot tests.
No Genesis scene is built and no GPU runtime is initialized.
"""
from __future__ import annotations

import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import igl
import numpy as np
import pymeshlab
import trimesh

from cloth_liquid_coupling import ClothLiquidCoupling

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE/"tests"))
from test_coupling_model import Entity


def main():
    config = json.loads((HERE/"config_robot_wipe.json").read_text())
    fluid, cotton = config["soy"], config["cloth"]
    mesh = trimesh.load(HERE/"assets/soy_puddle.obj", process=False)
    mesh.vertices *= np.array([*fluid["xy_scale"], fluid["depth_scale"]])
    spacing = fluid["particle_size"]
    lower, upper = mesh.bounds
    axes = [np.linspace(lower[k]+spacing/2, upper[k]-spacing/2,
                        max(1, round((upper[k]-lower[k])/spacing))) for k in range(3)]
    liquid_positions = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
    signed_distance, *_ = igl.signed_distance(liquid_positions, mesh.vertices, mesh.faces)
    liquid_positions = liquid_positions[signed_distance < 0]
    cloth_mesh = trimesh.load(HERE/"assets/cotton_cloth.obj", process=False)
    if cotton.get("physical_mesh") == "material_aligned_grid":
        from cotton_mesh import rectangular_cotton_mesh
        cloth_positions, faces = rectangular_cotton_mesh(
            size=cotton.get("size_m", [.28, .18]), spacing=cotton["particle_size"])
    else:
        mesher = pymeshlab.MeshSet()
        mesher.add_mesh(pymeshlab.Mesh(vertex_matrix=cloth_mesh.vertices, face_matrix=cloth_mesh.faces))
        mesher.meshing_isotropic_explicit_remeshing(targetlen=pymeshlab.PureValue(cotton["particle_size"]))
        physics_mesh = mesher.current_mesh()
        cloth_positions, faces = physics_mesh.vertex_matrix(), physics_mesh.face_matrix()
    table_height = config["table"]["height"]
    # This diagnostic deliberately starts an already laid sheet at contact
    # height, not the production sheet's 6 mm initial release clearance.
    cloth_positions[:, 2] = table_height+cotton.get("collision_radius_m",cotton["particle_size"]/2)
    liquid_positions[:, 2] += table_height+fluid["initial_clearance"]
    dry_mass = cloth_mesh.area*cotton["areal_density"]
    cloth = Entity(cloth_positions, dry_mass/len(cloth_positions), faces, free=False)
    cloth.particle_size = cotton["particle_size"]
    cloth.collision_radius_m = cotton.get("collision_radius_m", cotton["particle_size"]/2)
    liquid = Entity(liquid_positions, .8*spacing**3*fluid["density"])
    liquid.particle_size = spacing
    coupling = ClothLiquidCoupling(SimpleNamespace(n_envs=0), cloth, liquid, config)
    ids, face_ids, weights, _ = coupling._contacts(liquid_positions, cloth_positions, np.arange(len(liquid_positions)))
    required_mass = np.zeros(len(cloth_positions))
    np.add.at(required_mass, faces[face_ids].ravel(),
              (coupling.liquid_particle_mass[ids, None]*weights).ravel())
    initial_required_saturation = required_mass/coupling.vertex_capacity
    start = time.perf_counter()
    history = []
    for step in range(2500):
        coupling.step(config["simulation"]["dt"])
        if step % 250 == 249:
            history.append([(step+1)*config["simulation"]["dt"], int(liquid.active.sum())])
        if not liquid.active.any():
            break
    report = {
        "scope": "CPU fixed already-laid sheet centered over puddle, using configured physical topology and contact radius; no native dynamics or robot claims",
        "material_profile": cotton.get("material_profile", "legacy"),
        "liquid_particles": len(liquid_positions),
        "liquid_mass_g": coupling.initial_liquid_mass*1000,
        "cloth_particles": len(cloth_positions),
        "local_required_saturation_max": float(initial_required_saturation.max()),
        "contact_count_initial": len(ids),
        "all_absorbed": bool(not liquid.active.any()),
        "capture_seconds": (step+1)*config["simulation"]["dt"],
        "wall_seconds": time.perf_counter()-start,
        "history": history,
        "metrics": coupling.metrics(),
    }
    (HERE/"output/robot_profile_feasibility.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))
    if not report["all_absorbed"]:
        raise RuntimeError("Local capacity/flux profile failed to collect the full static puddle")


if __name__ == "__main__":
    main()
