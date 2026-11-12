"""CPU-only reproduction of world-position velocity quantization in PBD.

No Genesis initialization, scene modification, or GPU execution. The scalar
free-particle case isolates arithmetic and does not model contact or cloth.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def simulate_free_decay(mode, dtype=np.float32, *, x0=.8, v0=.1, h=.0001,
                        damping_per_s=1., duration=1.):
    if mode not in ("native", "post_drag", "incremental", "incremental_compensated"):
        raise ValueError("Unknown arithmetic comparison mode")
    x,v=dtype(x0),dtype(v0);step=dtype(h);decay=dtype(np.exp(-damping_per_s*h))
    residual=dtype(0.)
    for _ in range(round(duration/h)):
        initial=x
        if mode!="post_drag":
            v=dtype(v*decay)
        predicted_velocity=v
        displacement=dtype(v*step)
        if mode=="incremental_compensated":
            displacement=dtype(displacement+residual)
        predicted_position=dtype(initial+displacement)
        if mode=="incremental_compensated":
            residual=dtype(displacement-dtype(predicted_position-initial))
        x=predicted_position  # No position constraint in this isolating probe.
        if mode in ("incremental", "incremental_compensated"):
            v=dtype(predicted_velocity+dtype(dtype(x-predicted_position)/step))
        else:
            v=dtype(dtype(x-initial)/step)
        if mode=="post_drag":
            v=dtype(v*decay)
    ideal_v=v0*np.exp(-damping_per_s*duration)
    ideal_x=x0+(v0/damping_per_s)*(1-np.exp(-damping_per_s*duration)) if damping_per_s else x0+v0*duration
    return {"mode":mode,"precision":np.dtype(dtype).name,"initial_position_m":x0,
            "initial_velocity_m_s":v0,"substep_dt_s":h,"duration_s":duration,
            "damping_per_s":damping_per_s,"final_position_m":float(x),
            "final_velocity_m_s":float(v),"continuous_ideal_position_m":ideal_x,
            "continuous_ideal_velocity_m_s":ideal_v,
            "velocity_error_m_s":float(v)-ideal_v,"position_error_m":float(x)-ideal_x}


def run_diagnostic():
    comparison=[simulate_free_decay(mode,dtype)
                for dtype in (np.float32,np.float64)
                for mode in ("native","post_drag","incremental")]
    quantization=[]
    h=.0001;rate=1.
    for z in (.76,.95,1.02,1.25):
        ulp=float(np.spacing(np.float32(z)))
        quantization.append({"world_coordinate_m":z,"float32_ulp_m":ulp,
            "velocity_quantum_m_s":ulp/h,
            "approximate_preintegration_drag_rounding_deadzone_m_s":ulp/(2*h*(1-np.exp(-rate*h)))})
    # A pure normal velocity variation has no component along flat XY edges,
    # while the dihedral rate is nonzero. Axial edge damping cannot remove it.
    from cotton_bending import hinge_angle_gradient
    hinge=np.array([[0.,0.,0.],[.003,0.,0.],[0.,.003,0.],[.003,-.003,0.]])
    velocity=np.zeros((4,3));velocity[2,2]=.1
    pairs=np.array([[0,1],[1,2],[2,0],[1,3],[3,0]])
    edges=hinge[pairs[:,1]]-hinge[pairs[:,0]]
    relative=velocity[pairs[:,1]]-velocity[pairs[:,0]]
    axial=np.sum(relative*edges,axis=1)/np.linalg.norm(edges,axis=1)
    _,gradient=hinge_angle_gradient(hinge)
    return {"scope":"CPU scalar arithmetic and local hinge diagnostic, no native/GPU simulation",
      "comparison":comparison,"world_coordinate_resolution":quantization,
      "normal_mode_not_damped_by_axial_impulses":{
        "maximum_axial_relative_speed_m_s":float(np.max(np.abs(axial))),
        "dihedral_angular_rate_rad_s":float(np.sum(gradient*velocity))},
      "residual_position_quantization_example":[simulate_free_decay(mode,np.float32,v0=.0001,damping_per_s=0.)
        for mode in ("incremental","incremental_compensated")],
      "interpretation":[
        "The native world-position difference loses small damping updates even without forces, constraints or robot motion.",
        "Moving the same drag after reconstruction alone does not fix the repeated rounding fixed point.",
        "Prediction plus constraint displacement preserves integrated velocity; in exact arithmetic it equals original PBD.",
        "The proposed production adapter does not use the compensated-position example. Sub-ULP physical displacement may still fail to accumulate.",
        "A persistent position-compensation field would need explicit reset/teleport/checkpoint/contact handling and is a separate change.",
        "These isolated results identify a possible mechanism; a settled-held cloth trace is needed to quantify its share of actual oscillation."
      ],"source_sha256":{name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
         for name in ("cotton_velocity_update.py","cotton_material.py","cotton_bending.py","probe_pbd_velocity_resolution.py")}}


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--output",default="output/pbd_velocity_resolution_diagnostic.json")
    args=parser.parse_args();path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
    report=run_diagnostic();path.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({"output":str(path.resolve()),"comparison":report["comparison"]},indent=2))
