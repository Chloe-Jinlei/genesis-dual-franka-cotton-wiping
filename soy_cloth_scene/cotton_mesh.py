"""Preserve a regular material-axis cloth mesh in local Genesis 1.4.3.

Derived from the cloth-only branch of the bear scene's ``existing_pbd_mesh``.
The supplied physical points are already world coordinates. The original visual
OBJ keeps its texture and receives the native morph placement exactly once.
This bypasses PBD remeshing only during one ``scene.add_entity`` call; no
installed Genesis file or entity class identity is changed. Do not construct
another PBD cloth concurrently while the context is active.
"""
from __future__ import annotations

from contextlib import contextmanager
from importlib.metadata import version
import math

import numpy as np


def rectangular_cotton_mesh(size=(.28, .18), spacing=.003, center=(0., 0., 0.)):
    """Return world points and consistently wound alternating-diagonal faces.

    Each axis has ``ceil(size / spacing) + 1`` nodes, so spacing is an upper
    bound along each yarn direction. Diagonal edges are naturally longer.
    The default has 95 x 61 = 5795 nodes and preserves the 0.28 x 0.18 m outline.
    """
    size, center = np.asarray(size, dtype=float), np.asarray(center, dtype=float)
    if size.shape != (2,) or not np.isfinite(size).all() or np.any(size <= 0):
        raise ValueError("Cotton size must contain two finite positive lengths")
    if center.shape != (3,) or not np.isfinite(center).all():
        raise ValueError("Cotton center must contain three finite coordinates")
    if isinstance(spacing, (bool, np.bool_)) or not np.isscalar(spacing):
        raise ValueError("Cotton spacing must be one finite positive length")
    spacing = float(spacing)
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError("Cotton spacing must be one finite positive length")
    nx, ny = np.ceil(size / spacing).astype(int) + 1
    xx, yy = np.meshgrid(np.linspace(-size[0]/2, size[0]/2, nx),
                         np.linspace(-size[1]/2, size[1]/2, ny), indexing="xy")
    points = np.column_stack((xx.ravel(), yy.ravel(), np.zeros(nx*ny))) + center
    faces = []
    for j in range(ny-1):
        for i in range(nx-1):
            a, b = j*nx+i, j*nx+i+1
            c, d = (j+1)*nx+i+1, (j+1)*nx+i
            if (i+j) % 2:
                faces.extend(((a, b, d), (b, c, d)))
            else:
                faces.extend(((a, b, c), (a, c, d)))
    return np.ascontiguousarray(points), np.asarray(faces, dtype=np.int32)


def _validate_mesh(points, faces, areal_density):
    points, faces = np.asarray(points, dtype=float), np.asarray(faces)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 4 or not np.isfinite(points).all():
        raise ValueError("Cotton points must be finite N by 3 world coordinates")
    if (faces.ndim != 2 or faces.shape[1] != 3 or len(faces) < 2
            or not np.issubdtype(faces.dtype, np.integer)
            or faces.min() < 0 or faces.max() >= len(points)):
        raise ValueError("Cotton faces must be integer triangles referencing the points")
    density = float(areal_density)
    if not math.isfinite(density) or density <= 0:
        raise ValueError("Cotton areal density must be finite and positive")
    if np.ptp(points[:, 2]) > 1e-7 or np.any(np.ptp(points[:, :2], axis=0) <= 0):
        raise ValueError("Cotton rest mesh must be horizontal in world XY")
    cross = np.cross(points[faces[:, 1]]-points[faces[:, 0]],
                     points[faces[:, 2]]-points[faces[:, 0]])
    if np.any(cross[:, 2] <= 1e-16):
        raise ValueError("Cotton faces must be nondegenerate and consistently +Z wound")
    edges, counts = np.unique(np.sort(np.concatenate((faces[:, [0, 1]],
        faces[:, [1, 2]], faces[:, [2, 0]])), axis=1), axis=0, return_counts=True)
    if np.any(counts > 2):
        raise ValueError("Cotton mesh must be manifold")
    if len(np.unique(faces)) != len(points):
        raise ValueError("Cotton mesh must not contain unused particles")
    area = float(np.linalg.norm(cross, axis=1).sum()/2)
    return (np.ascontiguousarray(points.copy()), np.ascontiguousarray(faces, dtype=np.int32),
            np.ascontiguousarray(edges, dtype=np.int32), area, density)


_active_context = False


@contextmanager
def existing_cotton_mesh(points, faces, areal_density):
    """Preserve supplied physical topology during one PBD.Cloth import.

    Call after ``gs.init`` and surround only the cloth's ``scene.add_entity``.
    The visual morph may translate a local OBJ; ``points`` must already contain
    that world translation. Alternatively an already-world OBJ may use identity
    placement. Rotations are rejected because material axes are world XY.
    Matching visual/physical bounds are checked to detect accidental double
    translations. Equal node masses sum to rest surface area * areal density.
    """
    import genesis as gs
    import genesis.engine.solvers.pbd_solver as solver_module
    import genesis.utils.geom as gu
    import trimesh

    global _active_context
    if version("genesis-world") != "1.4.3":
        raise RuntimeError("Review the private cloth importer before another Genesis version")
    if _active_context:
        raise RuntimeError("Cotton mesh import contexts must not overlap or nest")
    points, faces, edges, area, density = _validate_mesh(points, faces, areal_density)
    entity_class = solver_module.PBD2DEntity
    previous_sample = entity_class.sample
    used = False

    def sample(entity):
        nonlocal used
        if used:
            raise RuntimeError("Use one existing_cotton_mesh context per cloth entity")
        used = True
        if not math.isclose(float(entity.material.rho), density, rel_tol=1e-8, abs_tol=1e-12):
            raise ValueError("Material rho must equal the supplied cotton areal density")
        # PBDTetEntity.sample normally performs this exact placement before
        # remeshing. Its _vmesh is still asset-local when this hook is entered.
        pos, quat = gu.transform_pos_quat_by_trans_quat(
            np.asarray(entity.morph.offset_pos, dtype=gs.np_float),
            np.asarray(entity.morph.offset_quat, dtype=gs.np_float),
            np.asarray(entity.morph.pos, dtype=gs.np_float),
            np.asarray(entity.morph.quat, dtype=gs.np_float))
        if np.linalg.norm(quat[1:]) > 1e-7 or abs(abs(float(quat[0]))-1.) > 1e-7:
            raise ValueError("Cotton material axes require a translation-only visual morph")
        entity._sampled_pos = np.asarray(pos, dtype=gs.np_float)
        entity._vmesh.apply_transform(gu.trans_quat_to_T(pos, quat))
        entity._vverts = np.asarray(entity._vmesh.verts, dtype=gs.np_float)
        entity._vfaces = np.asarray(entity._vmesh.faces, dtype=gs.np_int)
        visual_bounds = np.array([entity._vverts.min(0), entity._vverts.max(0)])
        physical_bounds = np.array([points.min(0), points.max(0)])
        if not np.allclose(visual_bounds, physical_bounds, atol=1e-6, rtol=0):
            raise ValueError("Visual morph and world cotton points have different bounds; check translation/scale")
        if not math.isclose(float(entity._vmesh.area), area, rel_tol=1e-5, abs_tol=1e-9):
            raise ValueError("Visual and physical cotton rest areas do not match")

        entity._particles = np.asarray(points, dtype=gs.np_float).copy()
        entity._init_particles_offset = gs.tensor(entity._particles)-gs.tensor(entity._sampled_pos)
        entity._mass = area*density
        entity._n_particles = len(points)
        entity._particle_mass = entity._mass/entity._n_particles
        uv = (points[:, :2]-points[:, :2].min(0))/np.ptp(points[:, :2], axis=0)
        visual = trimesh.visual.texture.TextureVisuals(uv=uv)
        physical = trimesh.Trimesh(vertices=entity._particles, faces=faces, visual=visual, process=False)
        entity._mesh = gs.Mesh.from_trimesh(physical, surface=entity.surface)
        entity._edges = edges.astype(gs.np_int, copy=True)
        adjacency, shared = trimesh.graph.face_adjacency(mesh=entity._mesh.trimesh, return_edges=True)
        opposite1 = faces[adjacency[:, 0]].sum(1)-shared.sum(1)
        opposite2 = faces[adjacency[:, 1]].sum(1)-shared.sum(1)
        entity._inner_edges = np.column_stack((shared, opposite1, opposite2)).astype(gs.np_int)
        entity._edges_len_rest = np.linalg.norm(entity._particles[edges[:, 0]]-entity._particles[edges[:, 1]], axis=1)
        entity._inner_edges_len_rest = np.linalg.norm(entity._particles[opposite1]-entity._particles[opposite2], axis=1)
        entity._cotton_mesh_description = {
            "model": "preserved XY material mesh with alternating triangle diagonals",
            "physical_points_are_world_coordinates": True,
            "visual_morph_transform_applied_once": True,
            "physical_points_transformed_again": False,
            "particle_count": len(points), "face_count": len(faces),
            "rest_area_m2": area, "areal_density_kg_m2": density,
            "total_dry_mass_kg": entity._mass,
            "equal_particle_mass_kg": entity._particle_mass,
            "installed_genesis_modified": False,
        }

    _active_context = True
    entity_class.sample = sample
    try:
        yield
    finally:
        entity_class.sample = previous_sample
        _active_context = False
