"""Scene-local cotton response, adapted from the supplied material reference.

Native areal-density masses and edge stretch remain in use. Directional strain
limits and physical dihedral bending are installed by the companion adapters.
This module adds internal damping, rest-area-weighted native air drag, and
linear drag restricted to this cloth. Internal damping can use the historical
edge-axial model or an explicitly selected rigid-motion residual model. The latter
damps bending motion as well, and is a nonlocal engineering approximation.
The last restriction deliberately
differs from the PDF's full-scene drag: SPH and rigid bodies are not changed.

All impulses read current native mass/free fields, including absorbed liquid.
Only immutable mesh data are cached; bending multipliers reset each substep.
These are engineering constitutive approximations, not measured fabric data.
"""

from importlib.metadata import version

import numpy as np


def _unique_edges(faces):
    return np.unique(np.sort(np.concatenate([
        faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]],
    ]), axis=1), axis=0)


def _edge_colors(edges, n_particles):
    """Return an ordering and boundaries with disjoint vertices per color."""
    used = [set() for _ in range(n_particles)]
    groups = []
    for index, (a, b) in enumerate(edges):
        unavailable = used[a] | used[b]
        color = 0
        while color in unavailable:
            color += 1
        if color == len(groups):
            groups.append([])
        groups[color].append(index)
        used[a].add(color)
        used[b].add(color)
    order = np.asarray([index for group in groups for index in group], dtype=np.int32)
    bounds = np.cumsum([0] + [len(group) for group in groups]).tolist()
    return order, bounds


def _prepare_cloth_material_data(points, faces, ccfg):
    """Validate physical rest topology and prepare immutable, CPU mesh data."""
    points = np.asarray(points, dtype=np.float64)
    raw_faces = np.asarray(faces)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
        raise ValueError("Cotton requires an (N, 3) physical rest-position array.")
    if not np.isfinite(points).all():
        raise ValueError("Cotton rest positions must be finite.")
    if raw_faces.ndim != 2 or raw_faces.shape[1] != 3 or not len(raw_faces):
        raise ValueError("Cotton requires a nonempty triangle mesh.")
    if not np.issubdtype(raw_faces.dtype, np.integer):
        raise ValueError("Cotton face indices must be integers.")
    if raw_faces.min() < 0 or raw_faces.max() >= len(points):
        raise ValueError("Cotton face indices are outside the physical vertex array.")
    faces = raw_faces.astype(np.int32)
    parameters = {
        "internal_damping_per_s": float(ccfg.get("internal_damping_per_s", 4.0)),
        "linear_drag_per_s": float(ccfg.get("linear_drag_per_s", 1.0)),
        "air_density_kg_m3": float(ccfg.get("air_density_kg_m3", 1.225)),
        "air_drag_coefficient": float(ccfg.get("air_drag_coefficient", 1.0)),
        "warp_max_extension": float(ccfg.get("warp_max_extension", 0.02)),
        "weft_max_extension": float(ccfg.get("weft_max_extension", 0.02)),
        "bias_max_extension": float(ccfg.get("bias_max_extension", 0.10)),
        "bending_rigidity_nm": float(ccfg.get("bending_rigidity_nm", 5e-5)),
        "bending_relaxation": float(ccfg.get("bending_relaxation", 0.5)),
    }
    if not all(np.isfinite(value) and value >= 0 for value in parameters.values()):
        raise ValueError("Cotton material settings must be finite and nonnegative.")
    if parameters["bending_rigidity_nm"] <= 0:
        raise ValueError("Cotton bending rigidity must be positive.")
    if not 0 < parameters["bending_relaxation"] <= 1:
        raise ValueError("Cotton bending relaxation must be in (0, 1].")
    iterations = ccfg.get("strain_limit_iterations", 24)
    if isinstance(iterations, (bool, np.bool_)) or not isinstance(iterations, (int, np.integer)) or iterations < 1:
        raise ValueError("Cotton strain-limit iterations must be a positive integer.")
    parameters["strain_limit_iterations"] = int(iterations)
    face_area = 0.5 * np.linalg.norm(np.cross(
        points[faces[:, 1]] - points[faces[:, 0]],
        points[faces[:, 2]] - points[faces[:, 0]],
    ), axis=1)
    if np.any(face_area <= 1e-18):
        raise ValueError("Cotton physical rest triangles must be nondegenerate.")
    particle_area = np.zeros(len(points), dtype=np.float64)
    np.add.at(particle_area, faces.ravel(), np.repeat(face_area / 3.0, 3))
    if np.any(particle_area <= 0):
        raise ValueError("Cotton physical mesh cannot contain isolated vertices.")
    edges = _unique_edges(faces)
    degree = np.bincount(edges.ravel(), minlength=len(points))
    maximum_degree = int(degree.max())
    order, bounds = _edge_colors(edges, len(points))
    rate = parameters["internal_damping_per_s"] / maximum_degree
    return {
        "points": points,
        "faces": faces,
        "edges": edges[order],
        "color_bounds": bounds,
        "maximum_degree": maximum_degree,
        "edge_rates": np.full(len(edges), rate, dtype=np.float64),
        "particle_area": particle_area,
        "air_coefficients": 0.5 * parameters["air_density_kg_m3"]
                            * parameters["air_drag_coefficient"] * particle_area,
        "parameters": parameters,
    }


def _range(values):
    values = np.asarray(values)
    return {"minimum": float(values.min()), "maximum": float(values.max())}


def install_cotton_material(scene, cloth, ccfg):
    """Install before build; ``configure()`` also runs at the first warmup step.

    The returned adapter exposes ``membrane``, ``bending`` and a JSON-compatible
    ``description``. Install before any other wrapper of store_initial that must
    execute outside this material update. No global force field is registered.
    """
    import genesis as gs
    import quadrants as qd
    import torch
    from cotton_bending import install_cotton_bending
    from cotton_membrane import install_cotton_membrane

    if version("genesis-world") != "1.4.3" or scene.is_built:
        raise RuntimeError("Install the Genesis 1.4.3 cotton adapter before scene.build().")
    solver = cloth.solver
    if solver is not scene.pbd_solver:
        raise ValueError("Cotton must belong to this scene's PBD solver.")
    if len(solver.entities) != 1 or solver.entities[0] is not cloth:
        raise ValueError("Cotton material adapters require this cloth as the only PBD entity.")
    if not isinstance(cloth.material, gs.materials.PBD.Cloth):
        raise TypeError("Cotton material adapters require PBD.Cloth.")
    if getattr(solver, "_cotton_material_adapter", None) is not None:
        raise RuntimeError("Cotton material response is already installed on this solver.")

    # gs.Mesh.verts is the physical mesh; visual/render vertices are different.
    data = _prepare_cloth_material_data(cloth.mesh.verts, cloth.mesh.faces, ccfg)
    if len(data["points"]) != cloth.n_particles:
        raise ValueError("Cotton physical mesh does not match its native particle count.")
    native_edges = np.unique(np.sort(np.asarray(cloth.edges, dtype=np.int32), axis=1), axis=0)
    if not np.array_equal(native_edges, _unique_edges(data["faces"])):
        raise ValueError("Cotton native edges do not match the physical rest mesh.")
    p = data["parameters"]
    membrane = install_cotton_membrane(
        scene, cloth, data["points"], data["faces"],
        warp_limit=p["warp_max_extension"], weft_limit=p["weft_max_extension"],
        bias_limit=p["bias_max_extension"], iterations=p["strain_limit_iterations"],
    )
    bending = install_cotton_bending(
        scene, cloth, data["points"], data["faces"],
        rigidity=p["bending_rigidity_nm"], relaxation=p["bending_relaxation"],
    )
    velocity_update = None
    if ccfg.get("preserve_integrated_velocity", False):
        from cotton_velocity_update import install_cotton_velocity_update
        velocity_update = install_cotton_velocity_update(scene, cloth)
    bending_damping = None
    bending_damping_ratio = float(ccfg.get("bending_damping_ratio", 0.))
    if not np.isfinite(bending_damping_ratio) or bending_damping_ratio < 0:
        raise ValueError("Cotton bending damping ratio must be finite and nonnegative.")
    if bending_damping_ratio > 0:
        from cotton_bending_damping import install_cotton_bending_damping
        bending_damping = install_cotton_bending_damping(
            scene, cloth, data["points"], data["faces"],
            rigidity=p["bending_rigidity_nm"], damping_ratio=bending_damping_ratio)
    internal_model = ccfg.get("internal_damping_model", "edge_axial")
    if internal_model not in ("edge_axial", "rigid_motion_residual"):
        raise ValueError("Unknown cotton internal_damping_model")
    deformation_damping = None
    if internal_model == "rigid_motion_residual":
        from cotton_deformation_damping import install_cotton_deformation_damping
        deformation_damping = install_cotton_deformation_damping(
            scene, cloth, rate_per_s=p["internal_damping_per_s"])

    def tensor(values, integer=False):
        return torch.as_tensor(np.ascontiguousarray(values), device=gs.device,
                               dtype=gs.tc_int if integer else gs.tc_float)

    @qd.data_oriented
    class CottonMaterial:
        def __init__(self):
            self.solver = solver
            self.membrane, self.bending = membrane, bending
            self.velocity_update = velocity_update
            self.bending_damping = bending_damping
            self.deformation_damping = deformation_damping
            self.native_store_initial = solver._kernel_store_initial_pos
            self.cloth_start, self.n_cloth = int(cloth.particle_start), int(cloth.n_particles)
            self.edges = tensor(data["edges"] + self.cloth_start, integer=True)
            self.rates = tensor(data["edge_rates"])
            self.air = tensor(data["air_coefficients"])
            self.color_bounds = data["color_bounds"]
            self.n_colors = len(self.color_bounds) - 1
            self.linear_drag_rate = p["linear_drag_per_s"]
            self.configured = False
            self.description = {
                "model": "PDF-referenced cotton membrane, dihedral bending and local damping",
                "experimentally_calibrated": False,
                "configured": False,
                "physical_mesh_particles": self.n_cloth,
                "physical_mesh_faces": int(len(data["faces"])),
                "physical_mesh_edges": int(len(data["edges"])),
                "physical_mesh_rest_area_m2": float(data["particle_area"].sum()),
                "effective_parameters": dict(p),
                "membrane": membrane.description,
                "bending": bending.description,
                "native_stretch": {
                    "retained": True,
                    "stretch_compliance": float(cloth.material.stretch_compliance),
                    "stretch_relaxation": float(cloth.material.stretch_relaxation),
                    "solver_iterations": int(solver._max_stretch_solver_iterations),
                    "native_bending_replaced": True,
                },
                "mass": {
                    "model": "native uniform dry masses from areal density, unchanged by this adapter",
                    "areal_density_kg_m2": float(cloth.material.rho),
                    "native_mass_read_each_impulse": True,
                    "wet_mass_supported": True,
                },
                "internal_damping": {
                    "model": "degree-normalized axial relative-velocity impulses on disjoint edge colors",
                    "rate_per_s": p["internal_damping_per_s"],
                    "maximum_mesh_degree": data["maximum_degree"],
                    "per_edge_rate_per_s": _range(data["edge_rates"]),
                    "colors": self.n_colors,
                    "scope": "free endpoints of this cloth's physical mesh edges",
                    "rigid_motion_unchanged_by_internal_damping": True,
                    "fixed_and_link_attached_velocities_unchanged": True,
                    "current_native_mass_and_free_read_each_substep": True,
                },
                "air_drag": {
                    "model": "native isotropic quadratic F_i = -0.5*rho_air*Cd*A_i*|v_i|*v_i",
                    "density_kg_m3": p["air_density_kg_m3"],
                    "drag_coefficient": p["air_drag_coefficient"],
                    "area_model": "one third of each incident physical rest triangle area",
                    "particle_rest_area_m2": _range(data["particle_area"]),
                    "particle_coefficient_kg_m": _range(data["air_coefficients"]),
                    "coefficient_sum_kg_m": float(data["air_coefficients"].sum()),
                    "scope": "free cotton particles only; native force divided by current native mass",
                    "orientation_resolved": False,
                },
                "linear_drag": {
                    "rate_per_s": self.linear_drag_rate,
                    "integration": "v *= exp(-rate*substep_dt), after internal damping and before native forces",
                    "scope": "free cotton particles only; SPH and rigid bodies unchanged",
                    "difference_from_pdf": "PDF full-scene drag is intentionally restricted to cotton",
                },
                "substep_lifecycle": {
                    "configure_at_first_build_warmup": True,
                    "bending_multipliers_reset_each_substep": True,
                    "new_state_persisted_between_substeps": False,
                },
            }
            if velocity_update is not None:
                self.description["velocity_update"] = velocity_update.description
            if bending_damping is not None:
                self.description["bending_damping"] = bending_damping.description
            if deformation_damping is not None:
                self.description["internal_damping"] = deformation_damping.description

        def configure(self):
            if not scene.is_built:
                raise RuntimeError("Configure cotton material after native fields are built.")
            if solver._B != 1:
                raise ValueError("Cotton material response supports one environment.")
            if self.configured:
                return self.description
            start, end = self.cloth_start, self.cloth_start + self.n_cloth
            masses = solver.particles_info.mass.to_numpy()[start:end]
            if not np.isfinite(masses).all() or np.any(masses <= 0):
                raise ValueError("Cotton native particle masses must be finite and positive.")
            self.configure_properties(self.air)
            edge_start = int(cloth._edge_start)
            edge_end = edge_start + int(cloth.n_edges)
            native = self.description["native_stretch"]
            native["effective_compliance_range"] = _range(
                solver.edges_info.stretch_compliance.to_numpy()[edge_start:edge_end])
            native["effective_relaxation_range"] = _range(
                solver.edges_info.stretch_relaxation.to_numpy()[edge_start:edge_end])
            self.description["mass"]["at_configuration_kg"] = _range(masses)
            self.description["mass"]["at_configuration_total_kg"] = float(masses.sum(dtype=np.float64))
            self.description["air_drag"]["native_effective_coefficient_kg_m"] = _range(
                solver.particles_info.air_resistance.to_numpy()[start:end])
            self.description["substep_dt_s"] = float(solver._substep_dt)
            self.description["configured"] = self.configured = True
            return self.description

        def store_initial(self, f):
            # Genesis marks the scene built before executing its warmup step.
            # Fields must be configured here, not only after build() returns.
            if not self.configured:
                self.configure()
            self.native_store_initial(f)
            self.bending.reset()
            if self.deformation_damping is None:
                self.damp_edges(self.edges, self.rates)
            else:
                self.deformation_damping.apply()
            if self.bending_damping is not None:
                self.bending_damping.apply()
            self.apply_linear_drag()

        @qd.kernel
        def configure_properties(self, air: qd.types.ndarray()):
            for local in range(self.n_cloth):
                self.solver.particles_info[self.cloth_start + local].air_resistance = air[local]

        @qd.kernel
        def damp_edges(self, pairs: qd.types.ndarray(), rates: qd.types.ndarray()):
            for color in qd.static(range(self.n_colors)):
                for edge in range(self.color_bounds[color], self.color_bounds[color + 1]):
                    a, b = pairs[edge, 0], pairs[edge, 1]
                    wa = self.solver.particles[a, 0].free / self.solver.particles_info[a].mass
                    wb = self.solver.particles[b, 0].free / self.solver.particles_info[b].mass
                    direction = self.solver.particles[a, 0].pos - self.solver.particles[b, 0].pos
                    length = direction.norm()
                    if wa + wb > 0.0 and length > 1e-12:
                        direction /= length
                        relative_speed = (self.solver.particles[a, 0].vel - self.solver.particles[b, 0].vel).dot(direction)
                        fraction = 1.0 - qd.exp(-rates[edge] * self.solver._substep_dt)
                        impulse = -fraction * relative_speed / (wa + wb) * direction
                        self.solver.particles[a, 0].vel += wa * impulse
                        self.solver.particles[b, 0].vel -= wb * impulse

        @qd.kernel
        def apply_linear_drag(self):
            for local in range(self.n_cloth):
                i = self.cloth_start + local
                if self.solver.particles[i, 0].free:
                    self.solver.particles[i, 0].vel *= qd.exp(-self.linear_drag_rate * self.solver._substep_dt)

    adapter = CottonMaterial()
    solver._kernel_store_initial_pos = adapter.store_initial
    solver._cotton_material_adapter = adapter
    return adapter
