"""Native robot dry pickup followed by a measured four-second stationary hold.

This is a dry-grasp/settling diagnostic, not the complete wiping task. Optional
resume is restricted to an identical-profile dry-lift checkpoint; it never
bypasses checkpoint material validation. Fresh mode uses the normal task's
approach, diagonal grasp and lift, then stops before transporting to the soy.
"""
import argparse
import csv
import hashlib
import json
import shutil
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from scene import build_scene
from robot_wipe import RobotWipe, arr


class DryLiftReached(Exception):pass


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--config',type=Path,default=Path('config_robot_wipe.json'))
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--resume',type=Path)
    p.add_argument('--hold',type=float,default=4.)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    cfg=json.loads(args.config.read_text())
    (args.output/'config_used.json').write_text(json.dumps(cfg,indent=2)+'\n')
    files=['probe_robot_settle.py','scene.py','robot_wipe.py','robot_grasp.py',
           'cotton_material.py','cotton_bending.py','cotton_velocity_update.py',
           'cotton_bending_damping.py','cotton_deformation_damping.py','cotton_membrane.py']
    sources={f:hashlib.sha256(Path(f).read_bytes()).hexdigest() for f in files if Path(f).exists()}
    archive=args.output/'source_at_run';archive.mkdir(exist_ok=True)
    for name in sources:shutil.copyfile(name,archive/name)
    scene,h=build_scene(cfg,headless=True,width=960,height=720)
    opt=SimpleNamespace(headless=True,record=True,steps=0,resume_robot_checkpoint=args.resume)
    task=RobotWipe(scene,h,cfg,opt,args.output)
    rows,positions,velocities=[],[],[]
    try:
        if args.resume:
            from robot_checkpoint import load_checkpoint
            state=load_checkpoint(args.resume,scene,h)
            if state['resume_stage']!='after_dry_lift':
                raise ValueError('Settling probe only accepts dry-lift checkpoints')
            for k,v in state.items():setattr(task,k,v)
        else:
            save=task.save_task_checkpoint
            def stop_after_dry(name,stage):
                result=save(name,stage)
                if stage=='after_dry_lift':raise DryLiftReached()
                return result
            task.save_task_checkpoint=stop_after_dry
            try:task.task()
            except DryLiftReached:pass
        task.event('Hold still: cotton settling measurement')
        start_frame=task.frame
        start_tcp={s:task.tcp(s).copy() for s in ('left','right')}
        start_x=arr(h['cloth'].get_particles_pos()).copy()
        free=h['cloth'].solver.particles.free.to_numpy()[:,0].astype(bool)
        for step in range(round(args.hold/task.dt)+1):
            if step:task.step()
            if step % round(.01/task.dt)==0:
                x=arr(h['cloth'].get_particles_pos()).copy()
                v=arr(h['cloth'].get_particles_vel()).copy()
                m=h['cloth'].solver.particles_info.mass.to_numpy().astype(float)
                vf=v[free].astype(float);mf=m[free]
                com=np.average(x.astype(float),axis=0,weights=m)
                vcom=np.average(vf,axis=0,weights=mf)
                kinetic=.5*np.sum(mf[:,None]*vf**2)
                row={'hold_time_s':step*task.dt,'scene_time_s':task.frame*task.dt,
                     'com_x_m':float(com[0]),'com_y_m':float(com[1]),'com_z_m':float(com[2]),
                     'minimum_z_m':float(x[:,2].min()),'minimum_node_id':int(x[:,2].argmin()),
                     'free_velocity_rms_m_s':float(np.sqrt(np.average(np.sum(vf**2,axis=1),weights=mf))),
                     'kinetic_energy_J':float(kinetic),
                     'nontranslational_kinetic_energy_J':float(kinetic-.5*mf.sum()*(vcom@vcom)),
                     'free_speed_p95_m_s':float(np.percentile(np.linalg.norm(vf,axis=1),95))}
                for s in ('left','right'):
                    tcp=task.tcp(s)
                    row[s+'_tcp_drift_m']=float(np.linalg.norm(tcp-start_tcp[s]))
                    for axis,value in zip('xyz',tcp):row[s+'_tcp_'+axis+'_m']=float(value)
                rows.append(row);positions.append(x);velocities.append(v)
                if step % round(.5/task.dt)==0:print(json.dumps(row),flush=True)
        task.snapshot('after_stationary_hold')
        # Restore the original checkpoint method after the fresh-run stage hook.
        RobotWipe.save_task_checkpoint(task,'after_settling_hold.pkl','after_dry_lift')
        with (args.output/'settling.csv').open('w') as f:
            w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
        np.savez_compressed(args.output/'settling_trajectory.npz',positions=positions,
                            velocities=velocities,reference=task.fabric.reference,
                            faces=task.fabric.faces,free=free,times=np.asarray([r['hold_time_s'] for r in rows]))
        windows={}
        for i in range(int(args.hold)):
            ids=[k for k,r in enumerate(rows) if i<=r['hold_time_s']<i+1]
            r=[rows[k] for k in ids];x=np.asarray(positions)[ids]
            extents=np.linalg.norm(np.ptp(x[:,free,:],axis=0),axis=1)
            windows[str(i)+'-'+str(i+1)+'s']={
                'mean_free_velocity_rms_m_s':float(np.mean([a['free_velocity_rms_m_s'] for a in r])),
                'mean_kinetic_energy_J':float(np.mean([a['kinetic_energy_J'] for a in r])),
                'com_z_peak_to_peak_m':float(np.ptp([a['com_z_m'] for a in r])),
                'node_motion_extent_p95_m':float(np.percentile(extents,95)),
                'maximum_tcp_drift_m':max(a[s+'_tcp_drift_m'] for a in r for s in ('left','right'))}
        report={'scope':__doc__,'resumed_from':str(args.resume) if args.resume else None,
                'hold_start_s':start_frame*task.dt,'hold_duration_s':args.hold,
                'material_profile':h['material_profile'],'windows':windows,
                'liquid_and_mass':h['coupling'].metrics(),'grasp_contacts':task.attached,
                'source_sha256':sources,'final_shape':task.fabric_stages['after_stationary_hold']}
        material=getattr(h['cloth'].solver,'_cotton_material_adapter',None)
        residual=getattr(material,'deformation_damping',None)
        if residual is not None:
            report['last_substep_internal_damping']=residual.diagnostics()
        (args.output/'settling_report.json').write_text(json.dumps(report,indent=2)+'\n')
    finally:
        if task.writer is not None:task.writer.close();task.writer=None
        import genesis as gs
        gs.destroy()


if __name__=='__main__':main()
