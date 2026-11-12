"""Local, conservative mesoscopic absorption for the Genesis soy/cloth scene.

This is an operator-split porous-sheet approximation, not a resolved cotton-fibre
or capillary SPH solver. Call ``step(dt)`` once after each native physics step.
A finite local mass reservoir lives on the *physics* triangles of the deforming
cloth. Transferred SPH parcels are inactive, retained in a mass ledger and
advected by triangle barycentric coordinates. Their mass is lumped onto the
three PBD vertices, so gravity, inertia and constraint inverse masses include it.

Contacts use exact closest points after a conservative cKDTree broad phase.
Contact drag/normal impulses and absorption conserve linear momentum for free
cloth vertices; prescribed vertices exchange a separately reported impulse with
the fixture. Geometric nonpenetration projection is split from velocity impulses.
Optional local in-plane saturation transport is an uncalibrated porous-sheet
approximation. No desorption, evaporation or resolved cotton fibres are claimed.
Single unbatched environments only. No Genesis source modification.
"""
from __future__ import annotations

import copy
import math
import numpy as np
from scipy.spatial import cKDTree
from numba import njit


def _numpy(value, dtype=None):
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=dtype)


def closest_points_on_triangles(points, triangles):
    """Vectorized exact point/triangle closest points and barycentric weights.

    Degenerate triangles reduce to their line segments. Inputs are corresponding
    (N,3) points and (N,3,3) triangles, not a dense Cartesian product.
    """
    points = np.asarray(points, dtype=np.float64)
    triangles = np.asarray(triangles, dtype=np.float64)
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    ab, ac, ap = b-a, c-a, points-a
    d00 = np.einsum("ij,ij->i", ab, ab)
    d01 = np.einsum("ij,ij->i", ab, ac)
    d11 = np.einsum("ij,ij->i", ac, ac)
    d20 = np.einsum("ij,ij->i", ap, ab)
    d21 = np.einsum("ij,ij->i", ap, ac)
    denom = d00*d11-d01*d01
    valid = denom > 1e-24
    safe = np.where(valid, denom, 1.0)
    w1, w2 = (d11*d20-d01*d21)/safe, (d00*d21-d01*d20)/safe
    weights = np.column_stack((1.0-w1-w2, w1, w2))
    inside = valid & (weights.min(axis=1) >= 0.0)
    projected = np.einsum("ni,nij->nj", weights, triangles)
    best = np.where(inside[:, None], projected, a)
    best_weights = np.zeros_like(weights)
    best_weights[:, 0] = 1.0
    best_weights[inside] = weights[inside]
    best_dist2 = np.where(inside, np.sum((points-best)**2, axis=1), np.inf)
    for i, j in ((0, 1), (1, 2), (2, 0)):
        start, edge = triangles[:, i], triangles[:, j]-triangles[:, i]
        length2 = np.sum(edge*edge, axis=1)
        t = np.clip(np.sum((points-start)*edge, axis=1)/np.maximum(length2, 1e-30), 0.0, 1.0)
        candidate = start+t[:, None]*edge
        distance2 = np.sum((points-candidate)**2, axis=1)
        replace = distance2 < best_dist2
        best[replace] = candidate[replace]
        best_dist2[replace] = distance2[replace]
        best_weights[replace] = 0.0
        best_weights[replace, i] = 1.0-t[replace]
        best_weights[replace, j] = t[replace]
    return best, best_weights, np.sqrt(np.maximum(best_dist2, 0.0))


@njit(cache=True)
def _contact_projection_kernel(ids, face_ids, weights, faces, cp, cv, lp, lv,
                               previous_cp, previous_lp, free, mass, liquid_mass,
                               drag_fraction, collision_distance, liquid_floor, cloth_floor):
    """Sequential impulses in compiled CPU code; no parallel race at shared nodes."""
    support = np.zeros(3)
    projections = 0
    for index in range(len(ids)):
        pid, fid = ids[index], face_ids[index]
        vertices, w = faces[fid], weights[index]
        triangle = cp[vertices]
        q = np.zeros(3)
        for k in range(3):
            q += w[k]*triangle[k]
        offset = lp[pid]-q
        distance = np.sqrt(np.sum(offset*offset))
        normal = np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0])
        norm = np.sqrt(np.sum(normal*normal))
        if norm <= 1e-14:
            continue
        normal /= norm
        if distance > 1e-12:
            normal = offset/distance
        else:
            previous_offset = previous_lp[pid].copy()
            for k in range(3):
                previous_offset -= w[k]*previous_cp[vertices[k]]
            if np.dot(previous_offset, normal) < 0:
                normal = -normal
        inv_cloth = np.zeros(3)
        inv_liquid = 1.0/liquid_mass[pid]
        inverse_effective_mass = inv_liquid
        sheet_velocity = np.zeros(3)
        for k in range(3):
            vertex = vertices[k]
            inv_cloth[k] = (1.0/mass[vertex]) if free[vertex] else 0.0
            inverse_effective_mass += w[k]*w[k]*inv_cloth[k]
            sheet_velocity += w[k]*cv[vertex]
        relative = lv[pid]-sheet_velocity
        vn = np.dot(relative, normal)
        impulse = -drag_fraction*(relative-vn*normal)
        if distance <= collision_distance+1e-7 and vn < 0:
            impulse -= vn*normal
        impulse /= inverse_effective_mass
        lv[pid] += impulse*inv_liquid
        for k in range(3):
            vertex = vertices[k]
            cv[vertex] -= w[k]*inv_cloth[k]*impulse
            if not free[vertex]:
                support += w[k]*impulse
        if distance < collision_distance:
            correction = (collision_distance-distance)/inverse_effective_mass*normal
            lp[pid] += inv_liquid*correction
            lp[pid, 2] = max(lp[pid, 2], liquid_floor)
            for k in range(3):
                vertex = vertices[k]
                cp[vertex] -= w[k]*inv_cloth[k]*correction
                if free[vertex]:
                    cp[vertex, 2] = max(cp[vertex, 2], cloth_floor)
            projections += 1
    return support, projections


@njit(cache=True)
def _absorb_kernel(ids, face_ids, weights, distances, faces, cv, lv, free, mass,
                   liquid_mass, dwell, minimum_dwell, gap, flux_credit, capacity,
                   absorbed_mass, bound_face, bound_weights, active):
    support = np.zeros(3)
    count = 0
    for index in range(len(ids)):
        pid, fid = ids[index], face_ids[index]
        if dwell[pid]+1e-12 < minimum_dwell or distances[index] > gap+1e-12:
            continue
        vertices, w = faces[fid], weights[index]
        permitted = True
        for k in range(3):
            vertex = vertices[k]
            addition = liquid_mass[pid]*w[k]
            if addition > flux_credit[vertex]+1e-15 or addition > capacity[vertex]-absorbed_mass[vertex]+1e-15:
                permitted = False
        if not permitted:
            continue
        for k in range(3):
            vertex = vertices[k]
            addition = liquid_mass[pid]*w[k]
            new_mass = mass[vertex]+addition
            if free[vertex]:
                cv[vertex] = (mass[vertex]*cv[vertex]+addition*lv[pid])/new_mass
            else:
                support += addition*(cv[vertex]-lv[pid])
            mass[vertex] = new_mass
            absorbed_mass[vertex] += addition
            flux_credit[vertex] = max(flux_credit[vertex]-addition, 0.0)
        bound_face[pid] = fid
        bound_weights[pid] = w
        active[pid] = False
        count += 1
    return support, count


class ClothLiquidCoupling:
    """Finite-capacity local wet cloth coupled to native PBD and SPH entities.

    ``config`` may be the full scene configuration or the coupling subsection.
    Public ``saturation``/``vertex_saturation`` and ``vertex_absorbed_mass`` arrays
    use the physics particle indexing. ``bound_positions`` is a computed array of
    liquid parcel positions on the current deforming cloth. Capture/restore this
    Python state alongside the native scene state when checkpointing wet scenes.
    """

    DEFAULTS = {
        "capacity_kg_m2": 1.2,
        "absorption_flux_kg_m2_s": 1.0,
        "minimum_contact_time_s": 0.035,
        "absorption_gap_m": 0.0035,
        "collision_distance_m": 0.0015,
        "drag_rate_s": 25.0,
    }

    def __init__(self, scene, cloth, soy, config):
        if getattr(scene, "n_envs", 0) != 0:
            raise ValueError("ClothLiquidCoupling supports one unbatched environment (n_envs=0).")
        self.scene, self.cloth, self.soy = scene, cloth, soy
        self.config = {**self.DEFAULTS, **config.get("coupling", config)}
        for key in self.DEFAULTS:
            value = float(self.config[key])
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"coupling.{key} must be finite and nonnegative")
            self.config[key] = value
        if self.config["absorption_gap_m"] <= 0:
            raise ValueError("absorption_gap_m must be positive")
        if self.config["collision_distance_m"] > self.config["absorption_gap_m"]:
            raise ValueError("collision_distance_m must not exceed absorption_gap_m")
        self.n_cloth, self.n_liquid = int(cloth.n_particles), int(soy.n_particles)
        self.faces = np.asarray(cloth.mesh.faces, dtype=np.int64).copy()
        if self.faces.ndim != 2 or self.faces.shape[1] != 3 or not len(self.faces):
            raise ValueError("The cloth physics mesh must contain triangular faces")
        if self.faces.min() < 0 or self.faces.max() >= self.n_cloth:
            raise ValueError("Cloth faces do not index native PBD particles")
        self._cloth_slice = slice(cloth.particle_start, cloth.particle_start+self.n_cloth)
        self._soy_slice = slice(soy.particle_start, soy.particle_start+self.n_liquid)
        self.dry_vertex_mass = self._read_mass(cloth, self._cloth_slice)
        self.liquid_particle_mass = self._read_mass(soy, self._soy_slice)
        if (self.dry_vertex_mass <= 0).any() or (self.liquid_particle_mass <= 0).any():
            raise ValueError("Native cloth and SPH masses must be positive")
        self._initial_cloth_pos = _numpy(cloth.get_particles_pos(), np.float64).copy()
        self._initial_soy_pos = _numpy(soy.get_particles_pos(), np.float64).copy()
        self._initial_soy_vel = _numpy(soy.get_particles_vel(), np.float64).copy()
        self._initial_active = _numpy(soy.get_particles_active(), bool).copy()
        self.initial_liquid_mass = float(self.liquid_particle_mass[self._initial_active].sum())
        tri = self._initial_cloth_pos[self.faces]
        areas = np.linalg.norm(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]), axis=1)/2.0
        self.vertex_reference_area = np.zeros(self.n_cloth)
        np.add.at(self.vertex_reference_area, self.faces.ravel(), np.repeat(areas/3.0, 3))
        if (self.vertex_reference_area <= 0).any():
            raise ValueError("Each cloth particle must belong to a nondegenerate surface triangle")
        self.vertex_capacity = self.vertex_reference_area*self.config["capacity_kg_m2"]
        self.wicking = None
        if self.config.get("wicking", {}).get("enabled", False):
            from cotton_wicking import CottonWicking
            self.wicking = CottonWicking(self._initial_cloth_pos, self.faces,
                self.liquid_particle_mass, self.vertex_capacity, self.config["wicking"])
        # Optional tabletop constraints make the geometric projection consistent
        # with native rigid contacts. No position is sent through the tabletop.
        self.table_height = config.get("table", {}).get("height")
        self._liquid_floor = None if self.table_height is None else float(self.table_height)+float(soy.particle_size)/2.0
        cloth_radius = float(getattr(cloth, "collision_radius_m", cloth.particle_size/2.0))
        self._cloth_floor = None if self.table_height is None else float(self.table_height)+cloth_radius
        self._initialize_ledger()

    @staticmethod
    def _read_mass(entity, particle_slice):
        # Installed Genesis SPH get_mass() references a nonexistent state member.
        # Read the authoritative static mass field, including its empirical SPH
        # volume factor; never infer mass from nominal spacing or entity totals.
        values = entity.solver.particles_info.mass.to_numpy()
        return np.asarray(values[particle_slice], dtype=np.float64).copy()

    def _write_cloth_mass(self):
        # Genesis has no public per-particle mass setter. This isolated field
        # write preserves all other PBD entities. Reordering copies it next step;
        # PBD computes inverse mass as free/mass, so no inverse-mass cache exists.
        field = self.cloth.solver.particles_info.mass
        all_mass = field.to_numpy()
        all_mass[self._cloth_slice] = self.dry_vertex_mass+self.vertex_absorbed_mass
        field.from_numpy(np.ascontiguousarray(all_mass))

    def _free_vertices(self):
        free = self.cloth.solver.particles.free.to_numpy()[self._cloth_slice]
        return np.asarray(free, dtype=bool).reshape(self.n_cloth, -1)[:, 0]

    def _initialize_ledger(self):
        if self.wicking is not None:
            self.wicking.reset()
        self.vertex_absorbed_mass = np.zeros(self.n_cloth)
        self._flux_credit = np.zeros(self.n_cloth)
        self._dwell = np.zeros(self.n_liquid)
        self._bound_face = np.full(self.n_liquid, -1, dtype=np.int64)
        self._bound_weights = np.zeros((self.n_liquid, 3))
        self._previous_cloth_pos = self._initial_cloth_pos.copy()
        self._previous_soy_pos = self._initial_soy_pos.copy()
        self._support_impulse = np.zeros(3)
        self._last_support_impulse = np.zeros(3)
        self._last_momentum_residual = np.zeros(3)
        self._max_momentum_residual = 0.0
        self._last_contact_count = 0
        self._last_absorbed_count = 0
        self._elapsed = 0.0
        self._steps = 0
        self._projection_count = 0
        self._last_wicking_report = None
        self._wicking_moves = 0
        self._wicking_crossings = 0
        self._wicking_neighbor_hops = 0
        self._maximum_wicking_step = 0.0

    @property
    def vertex_saturation(self):
        return np.divide(self.vertex_absorbed_mass, self.vertex_capacity,
                         out=np.zeros(self.n_cloth), where=self.vertex_capacity > 0)

    @property
    def saturation(self):
        return self.vertex_saturation

    @property
    def bound_particle_indices(self):
        return np.flatnonzero(self._bound_face >= 0)

    @property
    def bound_positions(self):
        ids = self.bound_particle_indices
        pos = _numpy(self.cloth.get_particles_pos(), np.float64)
        return np.einsum("ni,nij->nj", self._bound_weights[ids], pos[self.faces[self._bound_face[ids]]])

    def _contacts(self, liquid_pos, cloth_pos, active_ids):
        """Return ids, exact nearest face, weights and distance within the gap."""
        radius = self.config["absorption_gap_m"]
        if not len(active_ids):
            return active_ids, np.empty(0, np.int64), np.empty((0, 3)), np.empty(0)
        points = liquid_pos[active_ids]
        lo, hi = cloth_pos.min(axis=0)-radius, cloth_pos.max(axis=0)+radius
        active_ids = active_ids[((points >= lo) & (points <= hi)).all(axis=1)]
        if not len(active_ids):
            return active_ids, np.empty(0, np.int64), np.empty((0, 3)), np.empty(0)
        triangles = cloth_pos[self.faces]
        centers = triangles.mean(axis=1)
        bounding_radius = np.linalg.norm(triangles-centers[:, None], axis=2).max()
        neighborhoods = cKDTree(centers).query_ball_point(liquid_pos[active_ids], radius+bounding_radius)
        counts = np.fromiter((len(row) for row in neighborhoods), dtype=np.int64, count=len(active_ids))
        nonempty = counts > 0
        if not nonempty.any():
            return np.empty(0, np.int64), np.empty(0, np.int64), np.empty((0, 3)), np.empty(0)
        ids = active_ids[nonempty]
        counts = counts[nonempty]
        face_ids = np.concatenate([np.asarray(row, np.int64) for row in neighborhoods if len(row)])
        repeated_ids = np.repeat(ids, counts)
        _, weights, distances = closest_points_on_triangles(liquid_pos[repeated_ids], triangles[face_ids])
        starts = np.r_[0, np.cumsum(counts)[:-1]]
        minima = np.minimum.reduceat(distances, starts)
        # Choose exactly one deterministic nearest face, including edge ties.
        index = np.arange(len(distances))
        possible = np.where(distances <= np.repeat(minima, counts)+1e-14, index, len(index))
        selected = np.minimum.reduceat(possible, starts)
        keep = minima <= radius
        return ids[keep], face_ids[selected[keep]], weights[selected[keep]], minima[keep]

    def step(self, dt):
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("Coupling dt must be finite and positive")
        cp = _numpy(self.cloth.get_particles_pos(), np.float64).copy()
        cv = _numpy(self.cloth.get_particles_vel(), np.float64).copy()
        lp = _numpy(self.soy.get_particles_pos(), np.float64).copy()
        lv = _numpy(self.soy.get_particles_vel(), np.float64).copy()
        active = _numpy(self.soy.get_particles_active(), bool).copy()
        if not all(np.isfinite(a).all() for a in (cp, cv, lp, lv)):
            raise FloatingPointError("Nonfinite native particle state before cloth/liquid coupling")
        if np.any(active & (self._bound_face >= 0)):
            raise RuntimeError("An absorbed parcel was externally reactivated; restore coupling state with scene state")
        free = self._free_vertices()
        mass = self.dry_vertex_mass+self.vertex_absorbed_mass
        momentum_before = (mass[:, None]*cv).sum(axis=0)+(self.liquid_particle_mass[active, None]*lv[active]).sum(axis=0)
        ids, face_ids, weights, distances = self._contacts(lp, cp, np.flatnonzero(active))
        contact = np.zeros(self.n_liquid, dtype=bool)
        contact[ids] = True
        self._dwell[~contact] = 0.0
        self._dwell[ids] += dt
        self._last_contact_count, self._last_absorbed_count = len(ids), 0
        support = np.zeros(3)
        drag_fraction = -math.expm1(-self.config["drag_rate_s"]*dt)
        collision_distance = self.config["collision_distance_m"]
        # Sequential local impulse solves avoid unbounded summed drag at a vertex
        # touched by many liquid parcels. Each update conserves momentum exactly.
        support, projection_count = _contact_projection_kernel(
            ids, face_ids, weights, self.faces, cp, cv, lp, lv,
            self._previous_cloth_pos, self._previous_soy_pos, free, mass,
            self.liquid_particle_mass, drag_fraction, collision_distance,
            self._liquid_floor if self._liquid_floor is not None else -np.inf,
            self._cloth_floor if self._cloth_floor is not None else -np.inf)
        self._projection_count += projection_count
        changed_positions = projection_count > 0
        # Credit is local and earned only during present contact. It is bounded
        # by both remaining capacity and one parcel, so no dry-time flux banking.
        wet_vertices = np.zeros(self.n_cloth, dtype=bool)
        if len(ids):
            wet_vertices[np.unique(self.faces[face_ids])] = True
        self._flux_credit[~wet_vertices] = 0.0
        remaining = np.maximum(self.vertex_capacity-self.vertex_absorbed_mass, 0.0)
        rate = self.config["absorption_flux_kg_m2_s"]*self.vertex_reference_area
        rate *= np.maximum(1.0-self.vertex_saturation, 0.0)
        self._flux_credit[wet_vertices] += rate[wet_vertices]*dt
        self._flux_credit = np.minimum(self._flux_credit, np.minimum(remaining, self.liquid_particle_mass.max()))
        # Recheck exact local geometry in one vectorized pass after projection.
        # Absorption only changes velocities/mass, so these weights remain valid
        # throughout the following sequential conservative transfer solve.
        if len(ids):
            _, capture_weights, capture_distances = closest_points_on_triangles(
                lp[ids], cp[self.faces[face_ids]])
            capture_support, self._last_absorbed_count = _absorb_kernel(
                ids, face_ids, capture_weights, capture_distances, self.faces,
                cv, lv, free, mass, self.liquid_particle_mass, self._dwell,
                self.config["minimum_contact_time_s"], self.config["absorption_gap_m"],
                self._flux_credit, self.vertex_capacity, self.vertex_absorbed_mass,
                self._bound_face, self._bound_weights, active)
            support += capture_support
        # Pore liquid migrates only through neighboring material triangles.
        # This redistributes existing wet mass; it neither absorbs remote free
        # liquid nor increases local capacity. Keep native inertia and the
        # particle/vertex ledgers synchronized before the next physics step.
        wicking_changed = False
        if self.wicking is not None:
            self._last_wicking_report = self.wicking.step(dt, self._bound_face,
                self._bound_weights, self.vertex_absorbed_mass,
                self.dry_vertex_mass, cv, free)
            wr = self._last_wicking_report
            support += np.asarray(wr["support_impulse_on_system_kg_m_s"])
            self._wicking_moves += wr["moved_parcels"]
            self._wicking_crossings += wr["crossed_faces"]
            self._wicking_neighbor_hops += wr["neighbor_hops"]
            self._maximum_wicking_step = max(self._maximum_wicking_step,
                                             wr["maximum_material_distance_m"])
            wicking_changed = wr["moved_parcels"] > 0
            mass[:] = self.dry_vertex_mass+self.vertex_absorbed_mass
        bound = self.bound_particle_indices
        if len(bound):
            triangles = self.faces[self._bound_face[bound]]
            lp[bound] = np.einsum("ni,nij->nj", self._bound_weights[bound], cp[triangles])
            lv[bound] = np.einsum("ni,nij->nj", self._bound_weights[bound], cv[triangles])
        # Public position setters zero velocities in Genesis, hence velocity
        # setters must always follow them. Inactive parcels remain spatially
        # attached, but their dynamical mass exists only in the wet PBD vertices.
        # Public setters convert these float64 work arrays to gs.tc_float. Do
        # not quantize through float32 first when the native scene is float64.
        if changed_positions:
            self.cloth.set_particles_pos(cp)
        if changed_positions or len(bound):
            self.soy.set_particles_pos(lp)
        if len(ids) or len(bound):
            self.cloth.set_particles_vel(cv)
            self.soy.set_particles_vel(lv)
        if self._last_absorbed_count:
            self.soy.set_particles_active(active)
        if self._last_absorbed_count or wicking_changed:
            self._write_cloth_mass()
        momentum_after = (mass[:, None]*cv).sum(axis=0)+(self.liquid_particle_mass[active, None]*lv[active]).sum(axis=0)
        self._last_support_impulse = support
        self._support_impulse += support
        self._last_momentum_residual = momentum_after-momentum_before-support
        self._max_momentum_residual = max(self._max_momentum_residual, float(np.linalg.norm(self._last_momentum_residual)))
        self._previous_cloth_pos, self._previous_soy_pos = cp.copy(), lp.copy()
        self._elapsed += dt
        self._steps += 1

    def reset(self):
        """Call after native scene.reset(): restore dry mass and the full ledger."""
        self._initialize_ledger()
        self._write_cloth_mass()
        self.soy.set_particles_pos(self._initial_soy_pos)
        self.soy.set_particles_vel(self._initial_soy_vel)
        self.soy.set_particles_active(self._initial_active)

    def capture_state(self):
        names = ("vertex_absorbed_mass", "_flux_credit", "_dwell", "_bound_face", "_bound_weights",
                 "_previous_cloth_pos", "_previous_soy_pos", "_support_impulse", "_last_support_impulse",
                 "_last_momentum_residual", "_max_momentum_residual", "_last_contact_count",
                 "_last_absorbed_count", "_elapsed", "_steps", "_projection_count",
                 "_last_wicking_report", "_wicking_moves", "_wicking_crossings",
                 "_wicking_neighbor_hops", "_maximum_wicking_step")
        state = {name: copy.deepcopy(getattr(self, name)) for name in names}
        state["wicking_state"] = None if self.wicking is None else self.wicking.capture_state()
        return state

    def restore_state(self, state):
        """Restore after restoring the matching native scene state."""
        expected = self.capture_state()
        if set(state) != set(expected):
            raise ValueError("Coupling checkpoint keys do not match this version")
        for name, value in state.items():
            if isinstance(expected[name], np.ndarray) and np.shape(value) != expected[name].shape:
                raise ValueError(f"Coupling checkpoint shape mismatch for {name}")
        for name, value in state.items():
            if name == "wicking_state":
                continue
            setattr(self, name, copy.deepcopy(value))
        if self.wicking is not None:
            self.wicking.restore_state(state["wicking_state"])
        self._write_cloth_mass()
        active = self._initial_active.copy()
        active[self._bound_face >= 0] = False
        self.soy.set_particles_active(active)

    def metrics(self):
        active = _numpy(self.soy.get_particles_active(), bool)
        bound = self.bound_particle_indices
        active_mass = float(self.liquid_particle_mass[active].sum())
        reservoir_mass = float(self.liquid_particle_mass[bound].sum())
        native_mass = self._read_mass(self.cloth, self._cloth_slice)
        return {
            "model": "local finite-capacity porous-sheet reservoir / split SPH-PBD coupling",
            "steps": self._steps, "simulated_seconds": self._elapsed,
            "initial_liquid_mass_kg": self.initial_liquid_mass,
            "active_liquid_mass_kg": active_mass,
            "absorbed_mass_kg": reservoir_mass,
            "total_liquid_mass_kg": active_mass+reservoir_mass,
            "mass_balance_error_kg": active_mass+reservoir_mass-self.initial_liquid_mass,
            "active_liquid_count": int(active.sum()), "absorbed_particle_count": len(bound),
            "last_contact_count": self._last_contact_count,
            "last_absorbed_count": self._last_absorbed_count,
            "capacity_kg": float(self.vertex_capacity.sum()),
            "maximum_saturation": float(self.vertex_saturation.max(initial=0.0)),
            "wet_vertex_count": int(np.count_nonzero(self.vertex_absorbed_mass)),
            "dry_cloth_mass_kg": float(self.dry_vertex_mass.sum()),
            "native_wet_cloth_mass_kg": float(native_mass.sum()),
            "cloth_loading_error_kg": float(native_mass.sum()-self.dry_vertex_mass.sum()-reservoir_mass),
            "vertex_reservoir_error_kg": float(self.vertex_absorbed_mass.sum()-reservoir_mass),
            "support_impulse_on_system_kg_m_s": self._support_impulse.tolist(),
            "last_support_impulse_on_system_kg_m_s": self._last_support_impulse.tolist(),
            "last_operator_momentum_residual_kg_m_s": self._last_momentum_residual.tolist(),
            "maximum_operator_momentum_residual_norm_kg_m_s": self._max_momentum_residual,
            "geometric_projection_count": self._projection_count,
            "wicking_enabled": self.wicking is not None,
            "wicking_parcel_moves": self._wicking_moves,
            "wicking_neighbor_face_crossings": self._wicking_crossings,
            "wicking_neighbor_hops": self._wicking_neighbor_hops,
            "maximum_wicking_material_step_m": self._maximum_wicking_step,
            "last_wicking_report": self._last_wicking_report,
            "bound_height_range_m": ([float(self.bound_positions[:, 2].min()), float(self.bound_positions[:, 2].max())] if len(bound) else None),
        }
