"""Independent native cloth strain and convergence benchmark.

The benchmark fixes two small adjacent corner patches, lets the sheet hang
under gravity, then increases their separation by 1% and holds. It does not
modify the robot scene, material configuration or liquid coupler. Snapshot
analysis requires the same native rest topology used by the saved scene.
"""
from __future__ import annotations
import argparse
import json
import logging
from pathlib import Path
import time
import numpy as np

HERE = Path(__file__).resolve().parent


def strain_metrics(positions, edges, lengths):
    strain = np.linalg.norm(positions[edges[:, 1]]-positions[edges[:, 0]], axis=1)/lengths-1.0
    return {
        "edge_count": len(edges),
        "tensile_strain_p50": float(np.quantile(strain, .5)),
        "tensile_strain_p95": float(np.quantile(strain, .95)),
        "tensile_strain_p99": float(np.quantile(strain, .99)),
        "tensile_strain_max": float(strain.max()),
        "minimum_strain": float(strain.min()),
        "edges_over_2pct": int((strain > .02).sum()),
        "edges_over_5pct": int((strain > .05).sum()),
        "edges_over_10pct": int((strain > .10).sum()),
    }


def surface_strain_metrics(positions, reference, faces):
    """Material X/Y extension and shear; avoids equating folding with strain.

    Reference is the planar undeformed material. X/Y are illustrative warp/weft
    directions, not a measured weave calibration. This diagnostic does not alter
    native isotropic edge constraints or add an anisotropic constitutive law.
    """
    rest_tri = reference[faces]
    current_tri = positions[faces]
    dm = np.stack((rest_tri[:, 1, :2]-rest_tri[:, 0, :2],
                   rest_tri[:, 2, :2]-rest_tri[:, 0, :2]), axis=2)
    ds = np.stack((current_tri[:, 1]-current_tri[:, 0],
                   current_tri[:, 2]-current_tri[:, 0]), axis=2)
    deformation = ds @ np.linalg.inv(dm)
    lengths = np.linalg.norm(deformation, axis=1)
    extension = lengths-1.
    cosine = np.einsum("ni,ni->n", deformation[:, :, 0], deformation[:, :, 1])
    cosine /= np.maximum(lengths[:, 0]*lengths[:, 1], 1e-15)
    shear = 90.-np.degrees(np.arccos(np.clip(cosine, -1., 1.)))
    return {"warp_x_extension_p99":float(np.quantile(extension[:,0], .99)),
            "warp_x_extension_max":float(extension[:,0].max()),
            "weft_y_extension_p99":float(np.quantile(extension[:,1], .99)),
            "weft_y_extension_max":float(extension[:,1].max()),
            "absolute_shear_angle_deg_p99":float(np.quantile(np.abs(shear), .99)),
            "absolute_shear_angle_deg_max":float(np.max(np.abs(shear)))}


def analyze_snapshots():
    reference = np.load(HERE/"output/material_rest_mesh.npz")
    report = {}
    for folder in (HERE/"output").iterdir():
        path = folder/"final_state.npz"
        if not path.is_file():
            continue
        data = np.load(path)
        if "cloth" not in data or data["cloth"].shape != reference["positions"].shape:
            continue
        report[folder.name] = strain_metrics(data["cloth"], reference["edges"], reference["rest_lengths"])
        report[folder.name]["material_directions"] = surface_strain_metrics(data["cloth"], reference["positions"], reference["faces"])
    (HERE/"output/material_snapshot_strain.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2), flush=True)


def benchmark(iterations, backend="cuda", compliance=1e-7, seconds=1.2, bending=1e-5):
    import genesis as gs
    gs.init(backend=gs.cuda if backend == "cuda" else gs.cpu, seed=17,
            precision="32", logging_level=logging.WARNING)
    scene = gs.Scene(sim_options=gs.options.SimOptions(dt=.004, substeps=40),
        pbd_options=gs.options.PBDOptions(particle_size=.006,
            max_stretch_solver_iterations=iterations, max_bending_solver_iterations=4,
            lower_bound=(-1,-1,0), upper_bound=(1,1,2), hash_grid_res=(.3,.3,.3)),
        show_viewer=False)
    cloth = scene.add_entity(morph=gs.morphs.Mesh(file=str(HERE/"assets/cotton_cloth.obj"),
        pos=(0,0,1.0), file_meshes_are_zup=True, decimate=False, convexify=False),
        material=gs.materials.PBD.Cloth(rho=.6, stretch_compliance=compliance,
            bending_compliance=bending, stretch_relaxation=.3, bending_relaxation=.1),
        surface=gs.surfaces.Default())
    scene.build()
    rest = cloth.get_particles_pos().detach().cpu().numpy().copy()
    edges, lengths = np.asarray(cloth.edges), np.asarray(cloth._edges_len_rest)
    # Fix genuinely small material patches, not every node near an open jaw.
    corner_ids=[]
    for x in (-.14,.14):
        distance=np.linalg.norm(rest[:,:2]-np.array([x,-.09]),axis=1)
        corner_ids.append(np.flatnonzero(distance<.008))
    fixed=np.unique(np.concatenate(corner_ids))
    cloth.fix_particles(particles_idx_local=fixed)
    start=time.monotonic(); rows=[]
    initial_sep=np.mean(rest[corner_ids[1],0])-np.mean(rest[corner_ids[0],0])
    for frame in range(round(seconds/.004)):
        t=(frame+1)*.004
        # First 0.5 s gravity settle, then a smooth 1% separation increase.
        u=np.clip((t-.5)/.3,0,1);u=u*u*(3-2*u)
        targets=rest[fixed].copy()
        targets[:,0]+=np.sign(targets[:,0])*.005*initial_sep*u
        cloth.set_particles_pos(targets,particles_idx_local=fixed)
        cloth.set_particles_vel(np.zeros_like(targets),particles_idx_local=fixed)
        scene.step()
        if frame%25==24:
            p=cloth.get_particles_pos().detach().cpu().numpy()
            row={"time_s":t,**strain_metrics(p,edges,lengths)}
            rows.append(row)
    final=cloth.get_particles_pos().detach().cpu().numpy()
    all_edges_fixed=np.isin(edges,fixed).all(axis=1)
    report={"backend":backend,"stretch_iterations":iterations,"stretch_compliance_m_N":compliance,
        "stretch_relaxation":.3,"bending_compliance":bending,"bending_iterations":4,
        "substeps":40,"dt_s":.004,"particle_count":cloth.n_particles,"fixed_node_count":len(fixed),
        "areal_density_kg_m2":.6,"loading":"uniform 30.24 g cloth mass: conservative proxy for current wet sheet",
        "grasp":"two small material corner patches; 1 percent increase in separation after gravity settling",
        "seconds":seconds,"wall_seconds":time.monotonic()-start,
        "sag_max_m":float(1.-final[:,2].min()),"sag_median_m":float(1.-np.median(final[:,2])),
        "final":strain_metrics(final,edges,lengths),
        "material_directions":surface_strain_metrics(final,rest,np.asarray(cloth.mesh.faces)),
        "free_or_mixed_edges_final":strain_metrics(final,edges[~all_edges_fixed],lengths[~all_edges_fixed]),
        "history":rows}
    suffix="" if bending==1e-5 else f"_bend{bending:g}"
    out=HERE/f"output/material_benchmark_{iterations}_{compliance:g}{suffix}.json"
    out.write_text(json.dumps(report,indent=2)+"\n")
    np.savez_compressed(HERE/f"output/material_benchmark_{iterations}_{compliance:g}{suffix}.npz",cloth=final,rest=rest,edges=edges,faces=np.asarray(cloth.mesh.faces),rest_lengths=lengths,fixed_indices=fixed)
    print(json.dumps(report,indent=2),flush=True)
    gs.destroy()


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshots",action="store_true")
    parser.add_argument("--iterations",type=int,default=8)
    parser.add_argument("--backend",choices=("cpu","cuda"),default="cuda")
    parser.add_argument("--compliance",type=float,default=1e-7)
    parser.add_argument("--seconds",type=float,default=1.2)
    parser.add_argument("--bending",type=float,default=1e-5)
    args=parser.parse_args()
    if args.snapshots: analyze_snapshots()
    else: benchmark(args.iterations,args.backend,args.compliance,args.seconds,args.bending)
