"""Read-only arrival evidence using the native rigid/PBD collision SDF.

The five analytic pad boxes used by the *attachment* gate do not describe the
entire Panda finger collision mesh. An open finger can touch cloth with its
distal mesh before either pad box reaches the sheet. This module queries the
same SDF and current geometry poses as the Genesis 1.4.3 PBD coupler, restricted
to the requested original material particles and the two real finger links.

This is current geometric shell contact, not a measured force, a certificate
of bilateral support, or permission to attach/lift. The controller must still
check actual TCP arrival and its unchanged material/attachment gates. The
small tolerance accounts for the separated post-projection query; it is not
an enlarged physical collision radius. No particle, actuator, contact, solver,
or SDF state is written. Only a private NumPy result array is filled.
"""
from importlib.metadata import version
from weakref import WeakKeyDictionary

import numpy as np


_SCENE_QUERY_KERNELS = WeakKeyDictionary()
_FINGERS = ("left_finger", "right_finger")


def _numpy(value):
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def _query_kernel_for_scene(scene):
    """Cache code outside the scene; never attach/mutate simulation fields."""
    kernel = _SCENE_QUERY_KERNELS.get(scene)
    if kernel is None:
        import genesis as gs
        import quadrants as qd
        import genesis.utils.sdf as sdf
        from genesis.utils import array_class

        @qd.kernel
        def query(
            points: qd.types.ndarray(dtype=gs.qd_float, ndim=2),
            geom_indices: qd.types.ndarray(dtype=gs.qd_int, ndim=1),
            distances: qd.types.ndarray(dtype=gs.qd_float, ndim=2),
            geoms_state: array_class.GeomsState,
            geoms_info: array_class.GeomsInfo,
            sdf_info: array_class.SDFInfo,
        ):
            for i, j in qd.ndrange(points.shape[0], geom_indices.shape[0]):
                point = gs.qd_vec3([points[i, 0], points[i, 1], points[i, 2]])
                distances[i, j] = sdf.sdf_func_world(
                    geom_indices[j], 0, point, geoms_state, geoms_info, sdf_info)

        kernel = query
        _SCENE_QUERY_KERNELS[scene] = kernel
    return kernel


def _distance_summary(distances, indices, radius, tolerance):
    """JSON-ready evidence, retaining original cloth indices (not row IDs)."""
    if not len(indices):
        return {"minimum_sdf_distance_m": None, "minimum_shell_clearance_m": None,
                "minimum_distance_particle_index": None, "contact_indices": [],
                "contact_count": 0, "strict_shell_contact_indices": [],
                "strict_shell_contact_count": 0}
    row = int(np.argmin(distances))
    contact = indices[distances <= radius+tolerance]
    strict = indices[distances <= radius]
    return {"minimum_sdf_distance_m": float(distances[row]),
            "minimum_shell_clearance_m": float(distances[row]-radius),
            "minimum_distance_particle_index": int(indices[row]),
            "contact_indices": contact.tolist(), "contact_count": int(len(contact)),
            "strict_shell_contact_indices": strict.tolist(),
            "strict_shell_contact_count": int(len(strict))}


def native_finger_contact_report(robot, cloth, indices, tolerance_m=5e-5):
    """Query actual full-finger SDF against active specified cloth spheres.

    ``indices`` are entity-local *material* particle IDs, e.g. the unchanged
    reference-corner candidates. Only a single, built non-batched scene is
    supported. All coupled geometries belonging to each real finger link are
    included, using native current geom transforms rather than commanded FK.
    An empty candidate set returns no contact. Invalid data fails explicitly.
    """
    if version("genesis-world") != "1.4.3":
        raise RuntimeError("Native finger SDF query requires Genesis 1.4.3")
    if (isinstance(tolerance_m, (bool, np.bool_)) or not np.isscalar(tolerance_m)
            or not np.isfinite(tolerance_m) or tolerance_m < 0):
        raise ValueError("Contact query tolerance must be finite and nonnegative")
    tolerance = float(tolerance_m)
    scene = robot.scene
    if cloth.scene is not scene or not scene.is_built:
        raise ValueError("Robot and cloth must belong to the same built scene")
    rigid = robot.solver
    if rigid.n_envs != 0:
        raise ValueError("Native finger SDF query supports one non-batched scene")
    if not getattr(scene.sim.coupler, "_rigid_pbd", False):
        raise ValueError("The native rigid/PBD coupling must be active")
    native_sdf = rigid.collider._sdf
    if not native_sdf.is_active:
        raise ValueError("Native rigid/PBD SDF must already be active; query does not initialize it")

    raw_indices = _numpy(indices)
    if raw_indices.ndim != 1 or (raw_indices.size and raw_indices.dtype.kind not in "iu"):
        raise ValueError("Material particle indices must be a one-dimensional integer array")
    selected = np.unique(raw_indices.astype(np.int64, copy=False))
    if selected.size and (selected[0] < 0 or selected[-1] >= cloth.n_particles):
        raise ValueError("Material particle index is outside this cloth")
    positions = _numpy(cloth.get_particles_pos())
    active = _numpy(cloth.get_particles_active()).reshape(-1).astype(bool)
    if positions.shape != (cloth.n_particles, 3) or active.shape != (cloth.n_particles,):
        raise ValueError("Unexpected native cloth particle layout")
    queried = selected[active[selected]]
    if not np.isfinite(positions[queried]).all():
        raise ValueError("Cloth contact candidate coordinates must be finite")

    # This is the actual radius used by LegacyCoupler's depth calculation.
    radius = float(cloth.solver.particle_size)/2
    authored_radius = float(getattr(cloth, "collision_radius_m", radius))
    if not np.isfinite(radius) or radius <= 0 or abs(authored_radius-radius) > 1e-12:
        raise ValueError("Cloth shell radius disagrees with the native PBD coupler")

    geometries = {}
    all_geoms = []
    for name in _FINGERS:
        link = robot.get_link(name)
        geoms = [geom for geom in link.geoms if geom.needs_coup]
        if not geoms:
            raise ValueError("Finger link has no coupled collision geometry: "+name)
        geometries[name] = (link, geoms)
        all_geoms.extend(geoms)

    import genesis as gs
    points = np.ascontiguousarray(positions[queried], dtype=gs.np_float)
    geom_ids = np.asarray([g.idx for g in all_geoms], dtype=gs.np_int)
    distances = np.empty((len(queried), len(all_geoms)), dtype=gs.np_float)
    if len(queried):
        _query_kernel_for_scene(scene)(
            points, geom_ids, distances, rigid.dyn_state.geoms,
            rigid.dyn_info.geoms, native_sdf._sdf_info)
        if not np.isfinite(distances).all():
            raise RuntimeError("Native SDF query returned nonfinite distances")

    fingers = {}
    column = 0
    for name, (link, geoms) in geometries.items():
        per_geom = []
        for geom in geoms:
            per_geom.append({"geom_index": int(geom.idx), "geom_type": str(geom.type),
                             **_distance_summary(distances[:, column], queried, radius, tolerance)})
            column += 1
        minimum = distances[:, column-len(geoms):column].min(axis=1)
        summary = _distance_summary(minimum, queried, radius, tolerance)
        fingers[name] = {"link_index": int(link.idx), "coupled_geom_count": len(geoms),
                         "contact": bool(summary["contact_count"]),
                         **summary, "geometries": per_geom}
    return {"evidence_type": "Actual current full-finger native SDF against requested active PBD material spheres",
            "query_implementation": "Genesis 1.4.3 sdf.sdf_func_world, identical to LegacyCoupler/FrictionCoupler",
            "read_only": True, "native_measurement": True,
            "contact_force_measured": False, "attachment_gate_evaluated": False,
            "collision_radius_m": radius, "tolerance_m": tolerance,
            "contact_distance_threshold_m": radius+tolerance,
            "requested_indices": selected.tolist(), "queried_active_indices": queried.tolist(),
            "inactive_indices_excluded": selected[~active[selected]].tolist(),
            "fingers": fingers,
            "any_finger_contact": any(f["contact"] for f in fingers.values()),
            "both_fingers_contact": all(f["contact"] for f in fingers.values()),
            "limitations": "SDF shell proximity at the current state, with stated tolerance, is not contact force or force closure. Actual TCP arrival and the original bilateral pad/material attachment gate remain separate requirements."}
