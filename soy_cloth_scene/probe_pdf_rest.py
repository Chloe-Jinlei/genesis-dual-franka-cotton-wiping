"""Short native rest stability diagnostic; no robot motion or hidden pose fixes."""
import argparse,json
from pathlib import Path
import numpy as np
from scene import build_scene,save_views
from cloth_metrics import ClothMetrics

p=argparse.ArgumentParser()
p.add_argument('--output',type=Path,default=Path('output/pdf_cotton_rest_probe'))
p.add_argument('--disable-bending',action='store_true')
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
cfg=json.loads(Path('config_robot_wipe.json').read_text())
scene,h=build_scene(cfg,headless=True,width=800,height=600)
if a.disable_bending:
    h['cotton_material'].bending.solve=lambda f:None
    h['cloth'].solver._kernel_solve_bending=h['cotton_material'].bending.solve
rest=h['cloth'].get_particles_pos().cpu().numpy().copy()
metric=ClothMetrics(rest,h['coupling'].faces,direction_limits={k:cfg['cloth'][k+'_max_extension'] for k in ('warp','weft','bias')})
rows=[]
for i in range(250):
    scene.step()
    if (i+1)%10==0:
        x=h['cloth'].get_particles_pos().cpu().numpy()
        row={'time_s':(i+1)*.002,'min_z':float(x[:,2].min()),'max_z':float(x[:,2].max()),**metric.strain(x)}
        rows.append(row)
        print(json.dumps(row),flush=True)
np.savez_compressed(a.output/'rest_end.npz',reference=rest,positions=x,faces=h['coupling'].faces)
(a.output/'report.json').write_text(json.dumps({'diagnostic_only_bending_disabled':a.disable_bending,'material_profile':h['material_profile'],'rows':rows},indent=2))
save_views(h,a.output)
import genesis as gs
gs.destroy()
