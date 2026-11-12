"""Matched native hanging-cloth diagnostic, independent of robot motion.

The cloth starts flat at 0.96 m, with two adjacent nodes fixed at each of the
same diagonal material corners used by the robot. Gravity releases the other
nodes. This deliberately excites drape/bending modes; it is not a robot grasp
or a full wiping run. No positions or velocities are reset after initialization.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from scene import build_scene
from cloth_metrics import ClothMetrics


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--config', type=Path, default=Path('config_robot_wipe.json'))
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=4.)
    p.add_argument('--record', action='store_true')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cfg = json.loads(args.config.read_text())
    cfg['cloth']['initial_clearance'] = .20
    (args.output/'config_used.json').write_text(json.dumps(cfg, indent=2)+'\n')
    scene, h = build_scene(cfg, headless=True, width=800, height=600)
    cloth = h['cloth']
    ref = cloth.get_particles_pos().cpu().numpy().copy()
    faces = h['coupling'].faces
    metric = ClothMetrics(ref, faces, direction_limits={
        k:cfg['cloth'][k+'_max_extension'] for k in ('warp','weft','bias')})
    anchors = []
    for offset in cfg['robot_task']['grasp_offsets_xy_m'].values():
        xy = np.asarray(cfg['cloth']['position_xy']) + offset
        first = int(np.argmin(np.linalg.norm(ref[:,:2]-xy, axis=1)))
        same_row = np.flatnonzero(np.abs(ref[:,1]-ref[first,1]) < 1e-7)
        other = same_row[same_row != first]
        second = int(other[np.argmin(np.linalg.norm(ref[other]-ref[first], axis=1))])
        anchors += [first, second]
    cloth.fix_particles(anchors)
    free = np.ones(len(ref), dtype=bool);free[anchors] = False
    rows, positions, velocities = [], [], []
    writer = None
    if args.record:
        import imageio.v2 as imageio
        writer = imageio.get_writer(args.output/'hanging_probe.mp4', fps=25, quality=8,
                                    macro_block_size=2)
    dt = cfg['simulation']['dt']
    sample_stride, image_stride = round(.02/dt), round(.04/dt)
    try:
        for step in range(round(args.seconds/dt)+1):
            if step:
                scene.step()
            if step % sample_stride == 0:
                x = cloth.get_particles_pos().cpu().numpy().copy()
                v = cloth.get_particles_vel().cpu().numpy().copy()
                mass = cloth.solver.particles_info.mass.to_numpy()[
                    cloth.particle_start:cloth.particle_start+cloth.n_particles]
                speed = np.linalg.norm(v[free], axis=1)
                rows.append({'time_s':step*dt, 'centroid_z_m':float(x[:,2].mean()),
                             'minimum_z_m':float(x[:,2].min()),
                             'speed_rms_m_s':float(np.sqrt(np.mean(speed**2))),
                             'speed_p95_m_s':float(np.percentile(speed,95)),
                             'kinetic_energy_J':float(.5*np.sum(mass[:,None]*v.astype(float)**2)),
                             'anchor_displacement_max_m':float(np.linalg.norm(x[anchors]-ref[anchors],axis=1).max()),
                             **metric.strain(x)})
                positions.append(x);velocities.append(v)
                if step % round(.5/dt) == 0:
                    print(json.dumps({k:rows[-1][k] for k in ('time_s','minimum_z_m','speed_rms_m_s','kinetic_energy_J')}),flush=True)
            if writer is not None and step % image_stride == 0:
                rgb,*_ = h['cameras']['detail'].render()
                writer.append_data(np.asarray(rgb))
        with (args.output/'motion.csv').open('w') as f:
            w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
        np.savez_compressed(args.output/'trajectory.npz',reference=ref,faces=faces,
                            times=np.asarray([r['time_s'] for r in rows]),positions=positions,
                            velocities=velocities,anchors=anchors)
        summaries={}
        for start in (0.,1.,2.,3.):
            selected=[r for r in rows if start<=r['time_s']<start+1.]
            if selected:
                summaries[f'{start:g}-{start+1:g}s']={
                    'centroid_z_peak_to_peak_m':float(np.ptp([r['centroid_z_m'] for r in selected])),
                    'minimum_z_peak_to_peak_m':float(np.ptp([r['minimum_z_m'] for r in selected])),
                    'mean_speed_rms_m_s':float(np.mean([r['speed_rms_m_s'] for r in selected])),
                    'mean_kinetic_energy_J':float(np.mean([r['kinetic_energy_J'] for r in selected]))}
        source_files=['probe_cotton_hang.py','cotton_material.py','cotton_bending.py',
                      'cotton_membrane.py','robot_grasp.py','scene.py',
                      'cotton_velocity_update.py','cotton_bending_damping.py']
        report={'scope':__doc__,'material_profile':h['material_profile'],'duration_s':args.seconds,
                'anchors':anchors,'windows':summaries,'final':rows[-1],
                'source_sha256':{name:hashlib.sha256(Path(name).read_bytes()).hexdigest()
                                 for name in source_files if Path(name).exists()}}
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    finally:
        if writer is not None:writer.close()
        import genesis as gs
        gs.destroy()


if __name__=='__main__':main()
