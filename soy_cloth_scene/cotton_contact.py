"""Separate a single cloth's contact thickness from its sampled mesh spacing.

Genesis 1.4.3 samples/remeshes PBD cloth in ``add_entity``.  Its entity keeps
that spacing, whereas the solver's ``_particle_size`` subsequently determines
both rigid contact radius (half the value) and particle self-contact distance
(the full value).  Set the latter only after sampling and before kernel build.
This changes this scene instance only; no Genesis source/class is modified.

This is still particle contact, not continuous triangle contact or CCD.  A
1.5 mm shell sampled at 6 mm can miss inter-sample/fold contacts.  Reducing the
shell thickness resolves an otherwise impossible table/finger clearance; it
does not by itself make cloth contact resolution independent of mesh density.
"""
from __future__ import annotations

import math
from types import MethodType


def configure_thin_cloth_contact(scene, cloth, collision_radius_m: float) -> dict:
    """Configure one already sampled PBD.Cloth, before ``scene.build()``.

    Call after all entities have been added.  Future PBD additions are rejected
    because they would otherwise be sampled at the new contact diameter. Other
    solvers (SPH, rigid, etc.) are unaffected. ``cloth.particle_size`` retains
    the sampling spacing; ``cloth.collision_radius_m`` exposes shell radius to
    local grasp/contact/coupling helpers. Return JSON-serializable diagnostics.
    """
    import genesis as gs

    radius = float(collision_radius_m)
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError("Cloth collision_radius_m must be finite and positive.")
    if scene.is_built:
        raise RuntimeError("Configure cloth contact thickness before scene.build().")
    solver = scene.pbd_solver
    if len(solver.entities) != 1 or solver.entities[0] is not cloth:
        raise ValueError("Thin cloth contact requires exactly one PBD entity: this cloth.")
    if not isinstance(cloth.material, gs.materials.PBD.Cloth):
        raise TypeError("Thin cloth contact only supports PBD.Cloth material.")
    if cloth.solver is not solver:
        raise ValueError("Cloth does not belong to this scene's PBD solver.")
    spacing = float(cloth.particle_size)
    if not math.isfinite(spacing) or spacing <= 0 or radius > spacing / 2:
        raise ValueError("Contact radius must be at most half the sampled mesh spacing.")
    if cloth.n_particles <= 0 or cloth.n_edges <= 0 or not hasattr(cloth, "_mesh"):
        raise RuntimeError("Cloth mesh must already be sampled before setting thickness.")
    previous = getattr(solver, "_thin_cloth_contact_report", None)
    if previous is not None:
        if previous["collision_radius_m"] != radius:
            raise RuntimeError("Cloth contact thickness has already been configured.")
        return dict(previous)

    # The hash grid retains its original, larger cell size: it remains a valid
    # broad phase for the smaller contacts. dist_scale is a PBD-liquid-only
    # kernel parameter; no PBD liquid is supported by this single-cloth adapter.
    # Cloth topology, rest lengths, mass, visual mesh and interpolation weights
    # are already sampled and are deliberately left untouched.
    original_solver_spacing = float(solver.particle_size)
    solver._particle_size = 2 * radius
    cloth.collision_radius_m = radius

    def reject_later_pbd_entity(_solver, *args, **kwargs):
        raise RuntimeError(
            "Add all PBD entities before configuring thin cloth contact; "
            "this adapter supports one cloth only."
        )

    solver.add_entity = MethodType(reject_later_pbd_entity, solver)
    report = {
        "model": "single-cloth particle shell, scene-local radius override",
        "mesh_spacing_m": spacing,
        "original_solver_particle_size_m": original_solver_spacing,
        "collision_radius_m": radius,
        "self_contact_distance_m": 2 * radius,
        "cloth_particles": int(cloth.n_particles),
        "cloth_edges": int(cloth.n_edges),
        "continuous_triangle_collision": False,
    }
    solver._thin_cloth_contact_report = report
    return dict(report)


# An explicit install name is convenient alongside the other scene adapters.
install_thin_cloth_contact = configure_thin_cloth_contact
