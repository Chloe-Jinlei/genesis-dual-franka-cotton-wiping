"""Measure a flat cloth's planar rigid pose without editing its state.

This observation is a grasp target estimate, not a contact or material gate.
It deliberately rejects folded, elevated or nonrigid sheets rather than
inventing a planar layout. Reference coordinates retain material identity.
"""
from __future__ import annotations

import numpy as np


def fit_planar_cloth_layout(reference, positions, reference_center_xy, table_height):
    reference=np.asarray(reference,dtype=float)
    positions=np.asarray(positions,dtype=float)
    center=np.asarray(reference_center_xy,dtype=float)
    if (reference.ndim!=2 or reference.shape[1:]!=(3,) or len(reference)<3
            or positions.shape!=reference.shape or center.shape!=(2,)
            or not np.isfinite(reference).all() or not np.isfinite(positions).all()
            or not np.isfinite(center).all() or not np.isfinite(table_height)):
        raise ValueError("Planar cloth tracking requires matching finite XYZ particles and an XY reference center")
    x=reference[:,:2];y=positions[:,:2]
    x0=x-x.mean(0);y0=y-y.mean(0)
    # Both dimensions must have physical extent: a line has no unique yaw.
    if min(np.linalg.svd(x0,compute_uv=False)[-1],np.linalg.svd(y0,compute_uv=False)[-1])<1e-5:
        raise ValueError("Planar cloth tracking rejects a degenerate material layout")
    zmin,zmax=float(positions[:,2].min()),float(positions[:,2].max())
    if zmax-zmin>.005 or zmin<table_height-.002 or zmax>table_height+.010:
        raise ValueError("Planar cloth tracking requires a flat sheet near the table")
    u,_,vt=np.linalg.svd(x0.T@y0)
    rotation=vt.T@u.T
    if np.linalg.det(rotation)<0:
        vt[-1]*=-1
        rotation=vt.T@u.T
    fitted_center=y.mean(0)+(center-x.mean(0))@rotation.T
    errors=np.linalg.norm((x-center)@rotation.T+fitted_center-y,axis=1)
    rms=float(np.sqrt(np.mean(errors**2)));maximum=float(errors.max())
    if rms>.001 or maximum>.003:
        raise ValueError("Planar cloth tracking rejects a nonrigid or reflected XY layout")
    return {"center_xy_m":fitted_center.tolist(),
            "yaw_rad":float(np.arctan2(rotation[1,0],rotation[0,0])),
            "rotation_matrix_xy":rotation.tolist(),"scale":1.,
            "rms_residual_m":rms,"p99_residual_m":float(np.quantile(errors,.99)),
            "max_residual_m":maximum,"cloth_z_range_m":[zmin,zmax],
            "guards":{"maximum_z_span_m":.005,"minimum_z_relative_to_table_m":-.002,
                      "maximum_z_relative_to_table_m":.010,"maximum_xy_rms_m":.001,
                      "maximum_xy_residual_m":.003},
            "method":"Proper 2D Kabsch rigid fit, no scale; original material reference unchanged"}
