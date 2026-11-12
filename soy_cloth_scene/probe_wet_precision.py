"""Short native absorption/precision regression, independent of robot motion.

Place the unchanged cloth above the configured puddle at initialization, then
let gravity and the ordinary absorption/wicking hooks run. This is not a wipe
or a successful robot task. No particle states are imposed after build.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

from scene import build_scene, save_views


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=.5)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    cfg=json.loads(args.config.read_text())
    cfg['cloth']['position_xy']=list(cfg['soy']['position_xy'])
    (args.output/'config_used.json').write_text(json.dumps(cfg,indent=2)+'\n')
    files=['probe_wet_precision.py','cloth_liquid_coupling.py','cotton_wicking.py',
           'cotton_material.py','cotton_deformation_damping.py','scene.py']
    sources={name:hashlib.sha256(Path(name).read_bytes()).hexdigest()
             for name in files if Path(name).exists()}
    archive=args.output/'source_at_run';archive.mkdir(exist_ok=True)
    for name in sources:shutil.copyfile(name,archive/name)
    scene,h=build_scene(cfg,headless=True,width=800,height=600)
    rows=[];coupling=h['coupling'];cloth=h['cloth']
    dry=coupling.dry_vertex_mass.sum()
    try:
        for step in range(round(args.seconds/cfg['simulation']['dt'])):
            scene.step()
            if (step+1)%25==0:
                row={'time_s':(step+1)*cfg['simulation']['dt'],**coupling.metrics()}
                rows.append(row)
                print(json.dumps({k:row[k] for k in ('time_s','absorbed_mass_kg','active_liquid_count')}),flush=True)
        x=cloth.get_particles_pos().cpu().numpy()
        v=cloth.get_particles_vel().cpu().numpy()
        masses=cloth.solver.particles_info.mass.to_numpy()
        m=coupling.metrics()
        damping=h['cotton_material'].deformation_damping
        diagnostic=damping.diagnostics()
        expected_dtype=np.dtype('float64' if str(cfg['simulation'].get('precision','32'))=='64' else 'float32')
        bound=coupling.bound_particle_indices
        lp=h['soy'].get_particles_pos().cpu().numpy()
        bound_error=(float(np.max(np.linalg.norm(lp[bound]-coupling.bound_positions,axis=1)))
                     if len(bound) else float('inf'))
        checks={
            'finite_cloth':bool(np.isfinite(x).all() and np.isfinite(v).all()),
            'native_dtype_preserved':bool(x.dtype==expected_dtype and v.dtype==expected_dtype and masses.dtype==expected_dtype),
            'some_liquid_absorbed':bool(m['absorbed_mass_kg']>1e-6),
            'total_mass_conserved':bool(abs(m['mass_balance_error_kg'])<1e-7),
            'wet_mass_matches_ledger':bool(abs(float(masses.sum())-dry-m['absorbed_mass_kg'])<1e-7),
            'capacity_respected':bool(float(coupling.vertex_saturation.max())<=1.000001),
            # The diagnostic is from the last native substep, before the
            # current outer-step transfer; permit at most that actual transfer.
            'damping_reads_wet_mass':bool(diagnostic['free_mass_kg']>dry+1e-6),
            'bound_parcels_follow_cloth_at_native_precision':bool(
                bound_error < (1e-12 if expected_dtype==np.dtype('float64') else 2e-7)),
        }
        report={'scope':__doc__,'passed':all(checks.values()),'checks':checks,
                'source_sha256':sources,'rows':rows,'final_metrics':m,
                'native_position_dtype':str(x.dtype),'native_velocity_dtype':str(v.dtype),
                'native_mass_dtype':str(masses.dtype),'dry_mass_kg':float(dry),
                'native_wet_mass_kg':float(masses.sum()),
                'maximum_bound_parcel_advection_error_m':bound_error,
                'last_substep_internal_damping':diagnostic,
                'max_coordinate_difference_from_float32_m':float(np.max(np.abs(x-x.astype(np.float32).astype(x.dtype))))}
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        np.savez_compressed(args.output/'wet_state.npz',positions=x,velocities=v,
                            native_mass=masses,absorbed_mass=coupling.vertex_absorbed_mass)
        save_views(h,args.output)
        if not report['passed']:raise RuntimeError('Native wet precision regression failed')
    finally:
        import genesis as gs
        gs.destroy()


if __name__=='__main__':main()
