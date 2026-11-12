"""Optional conservative, local in-plane transport of absorbed cloth parcels.

This is an overdamped porous-sheet engineering model, not a calibrated cotton
capillary-pressure law. It approximates saturation diffusion with parcel drift
v_material = -D grad(saturation) / max(local_saturation, saturation_floor).
The effective D has units m^2/s. A local quadratic saturation-energy check,
finite capacities, a material-distance cap and at most ONE adjacent-face
crossing per call limit each accepted transfer. There is no remote reservoir,
new absorption, liquid deletion, desorption or increase of cloth capacity.

Exactly saturated cells cannot move one parcel continuously without temporarily
overfilling a vertex. A bounded adjacent-face hop therefore supplements drift:
shared-edge weights stay unchanged and the opposite-vertex weight moves to the
neighbor's dry opposite vertex. A local D*dt/distance^2 saturation-driven credit
delays this discrete event. This is finite-parcel diffusion, not a resolved
continuous capillary trajectory. Credit is cleared if its local path changes.

Each parcel retains its original mass and its unique face/barycentric record.
Its donor momentum is mixed into receiving vertices; fixed-vertex reactions
are reported separately. Relative capillary-flow inertia and mechanical energy
are not resolved. Geometry follows the original planar material coordinates,
not spatial proximity of folded but materially distant sheets.

The operator only mutates supplied CPU ledger/velocity arrays. The caller must
write changed wet masses and velocities to native PBD, and recompute absorbed
parcel positions/velocities from the updated face/weights before rendering.
Call after local absorption, before computing the coupling momentum residual.
Defaults to disabled; it never imports or modifies Genesis.
"""
from __future__ import annotations

import numpy as np
from numba import njit


@njit(cache=True)
def _barycentric(point, face, origins, gradients):
    result = np.empty(3)
    delta = point-origins[face]
    result[1] = np.dot(gradients[face, 1], delta)
    result[2] = np.dot(gradients[face, 2], delta)
    result[0] = 1.0-result[1]-result[2]
    return result


@njit(cache=True)
def _local_target(face, weights, displacement, material_vertices, faces,
                  origins, gradients, neighbors):
    """Trace in material space, stopping at boundaries or a second crossing."""
    point = np.zeros(2)
    for k in range(3):
        point += weights[k]*material_vertices[faces[face, k]]
    target = point+displacement
    dw = gradients[face] @ displacement
    end_weights = weights+dw
    result_face = face
    if end_weights.min() < -1e-12:
        fraction, edge = 1.0, -1
        for k in range(3):
            if dw[k] < -1e-15:
                crossing = max(0.0, -weights[k]/dw[k])
                if crossing < fraction:
                    fraction, edge = crossing, k
        boundary = point+fraction*displacement
        neighbor = neighbors[face, edge] if edge >= 0 else -1
        if neighbor < 0:
            end_weights = _barycentric(boundary, face, origins, gradients)
        else:
            result_face = neighbor
            start_weights = _barycentric(boundary, neighbor, origins, gradients)
            end_weights = _barycentric(target, neighbor, origins, gradients)
            # A vertex crossing may immediately leave the first neighbor.
            # Stop there rather than jumping over another material face.
            if end_weights.min() < -1e-12:
                change = end_weights-start_weights
                fraction2 = 1.0
                for k in range(3):
                    if change[k] < -1e-15:
                        fraction2 = min(fraction2, max(0.0, -start_weights[k]/change[k]))
                end_weights = start_weights+fraction2*change
    for k in range(3):
        end_weights[k] = max(0.0, end_weights[k])
    end_weights /= end_weights.sum()
    new_point = np.zeros(2)
    for k in range(3):
        new_point += end_weights[k]*material_vertices[faces[result_face, k]]
    return result_face, end_weights, np.linalg.norm(new_point-point)


@njit(cache=True)
def _redistribute(dt, diffusivity, saturation_floor, maximum_step, step_fraction,
                  max_backtracks, maximum_hop, hop_credit, hop_target,
                  material_vertices, faces, origins, gradients,
                  neighbors, face_altitudes, parcel_mass, capacity, dry_mass,
                  bound_face, bound_weights, wet_mass, velocities, free):
    support = np.zeros(3)
    moved, crossed, limited, hops = 0, 0, 0, 0
    max_distance, energy_change = 0.0, 0.0
    for pid in range(len(bound_face)):
        face = bound_face[pid]
        if face < 0:
            continue
        old_weights = bound_weights[pid].copy()
        old_vertices = faces[face]
        saturation = wet_mass[old_vertices]/capacity[old_vertices]
        gradient = np.zeros(2)
        for k in range(3):
            gradient += saturation[k]*gradients[face, k]
        local_saturation = np.dot(old_weights, saturation)
        displacement = -dt*diffusivity*gradient/max(local_saturation, saturation_floor)
        distance = np.linalg.norm(displacement)
        limit = min(maximum_step, step_fraction*face_altitudes[face])
        if distance > limit:
            displacement *= limit/distance
            limited += 1
        # Each trial touches <=6 slots (usually four unique material vertices).
        # Energy and capacity checks use the complete donor+receiver delta, so
        # shared-edge vertices cannot be accidentally double-counted.
        accepted = False
        target_face, target_weights, actual_distance = face, old_weights, 0.0
        indices, delta = np.empty(6, np.int64), np.zeros(6)
        n_unique, delta_energy = 0, 0.0
        for attempt in range(max_backtracks+1):
            target_face, target_weights, actual_distance = _local_target(
                face, old_weights, displacement, material_vertices, faces,
                origins, gradients, neighbors)
            if actual_distance <= 1e-15:
                break
            indices[:] = -1
            delta[:] = 0.0
            n_unique = 0
            for side in range(2):
                vv = old_vertices if side == 0 else faces[target_face]
                ww = old_weights if side == 0 else target_weights
                sign = -1.0 if side == 0 else 1.0
                for k in range(3):
                    vertex = vv[k]
                    slot = -1
                    for j in range(n_unique):
                        if indices[j] == vertex:
                            slot = j
                    if slot < 0:
                        slot = n_unique
                        indices[slot] = vertex
                        n_unique += 1
                    delta[slot] += sign*parcel_mass[pid]*ww[k]
            permitted = True
            delta_energy = 0.0
            for j in range(n_unique):
                vertex, dm = indices[j], delta[j]
                new_wet = wet_mass[vertex]+dm
                if new_wet < -1e-15 or new_wet > capacity[vertex]+1e-15:
                    permitted = False
                delta_energy += (wet_mass[vertex]*dm+0.5*dm*dm)/capacity[vertex]
            # Non-increasing saturation energy is the numerical diffusion
            # criterion. It also prevents large-step overshoot around dry cells.
            if permitted and delta_energy < -1e-20:
                accepted = True
                break
            displacement *= 0.5
        if not accepted:
            # A flat, saturated face needs an outward flux even when its own
            # gradient is zero. Only one topological neighbor may receive this
            # parcel. Shared vertices keep precisely the same parcel weights.
            best_rate, best_face, best_energy, best_distance = 0.0, -1, 0.0, 0.0
            best_weights = np.zeros(3)
            for opposite in range(3):
                neighbor = neighbors[face, opposite]
                fraction = old_weights[opposite]
                if neighbor < 0 or fraction <= 1e-12:
                    continue
                new_weights = np.zeros(3)
                receiver = -1
                for j in range(3):
                    vertex = faces[neighbor, j]
                    found = False
                    for k in range(3):
                        if vertex == old_vertices[k]:
                            new_weights[j] = old_weights[k]
                            found = True
                    if not found:
                        receiver = vertex
                        new_weights[j] = fraction
                donor = old_vertices[opposite]
                if receiver < 0:
                    continue
                old_sat = wet_mass[donor]/capacity[donor]
                new_sat = wet_mass[receiver]/capacity[receiver]
                transferred = parcel_mass[pid]*fraction
                hop_distance = fraction*np.linalg.norm(material_vertices[receiver]-material_vertices[donor])
                if (old_sat <= new_sat or hop_distance <= 1e-15 or hop_distance > maximum_hop
                        or wet_mass[receiver]+transferred > capacity[receiver]+1e-15):
                    continue
                hop_energy = transferred*(new_sat-old_sat)+0.5*transferred*transferred*(
                    1.0/capacity[donor]+1.0/capacity[receiver])
                if hop_energy >= -1e-20:
                    continue
                rate = diffusivity/max(hop_distance*hop_distance, 1e-12)*(
                    old_sat-new_sat)/max(old_sat, saturation_floor)
                if rate > best_rate:
                    best_rate, best_face = rate, neighbor
                    best_weights = new_weights
                    best_energy, best_distance = hop_energy, hop_distance
            if best_face < 0:
                hop_credit[pid], hop_target[pid] = 0.0, -1
                continue
            if hop_target[pid] != best_face:
                hop_credit[pid] = 0.0
                hop_target[pid] = best_face
            hop_credit[pid] = min(1.0, hop_credit[pid]+dt*best_rate)
            if hop_credit[pid] < 1.0-1e-12:
                continue
            target_face, target_weights = best_face, best_weights
            actual_distance, delta_energy = best_distance, best_energy
            hops += 1
        hop_credit[pid], hop_target[pid] = 0.0, -1
        carried_velocity = np.zeros(3)
        for k in range(3):
            carried_velocity += old_weights[k]*velocities[old_vertices[k]]
        # Remove mass at each donor's current velocity. Then deposit the SAME
        # parcel momentum at recipients, including vertices shared by both faces.
        for k in range(3):
            wet_mass[old_vertices[k]] -= parcel_mass[pid]*old_weights[k]
        for k in range(3):
            vertex = faces[target_face, k]
            dm = parcel_mass[pid]*target_weights[k]
            before = dry_mass[vertex]+wet_mass[vertex]
            after = before+dm
            if free[vertex]:
                velocities[vertex] = (before*velocities[vertex]+dm*carried_velocity)/after
            else:
                support += dm*(velocities[vertex]-carried_velocity)
            wet_mass[vertex] += dm
        bound_face[pid] = target_face
        bound_weights[pid] = target_weights
        moved += 1
        crossed += int(face != target_face)
        max_distance = max(max_distance, actual_distance)
        energy_change += delta_energy
    return support, moved, crossed, limited, max_distance, energy_change, hops


class CottonWicking:
    """Transport operator for one planar-reference cloth.

    ``step`` mutates only bound_face, bound_weights, vertex_absorbed_mass and
    cloth_velocities. ``capture_state/restore_state/reset`` preserve the local
    neighbor-hop credits in addition to the caller's parcel ledger checkpoint.
    Geometry, capacity and per-parcel mass are immutable copies.
    """
    DEFAULTS = {
        "enabled": False,
        "diffusivity_m2_s": 1e-6,
        "maximum_material_step_m": 0.0002,
        "maximum_neighbor_hop_m": 0.003,
        "maximum_face_altitude_fraction": 0.2,
        "saturation_floor": 0.05,
        "maximum_backtracks": 12,
    }

    def __init__(self, reference_positions, faces, liquid_particle_mass,
                 vertex_capacity, config=None):
        self.config = {**self.DEFAULTS, **(config or {})}
        unknown = set(self.config)-set(self.DEFAULTS)
        if unknown:
            raise ValueError(f"Unknown wicking options: {sorted(unknown)}")
        if not isinstance(self.config["enabled"], (bool, np.bool_)):
            raise ValueError("Wicking enabled must be boolean")
        for key in ("diffusivity_m2_s", "maximum_material_step_m",
                    "maximum_neighbor_hop_m", "maximum_face_altitude_fraction", "saturation_floor"):
            value = float(self.config[key])
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be finite and positive")
            self.config[key] = value
        if self.config["maximum_face_altitude_fraction"] > 0.25:
            raise ValueError("Material step is limited to at most one quarter of local face altitude")
        if self.config["saturation_floor"] > 1:
            raise ValueError("saturation_floor must not exceed 1")
        backtracks = self.config["maximum_backtracks"]
        if isinstance(backtracks, bool) or not isinstance(backtracks, (int, np.integer)) or not 0 <= backtracks <= 30:
            raise ValueError("maximum_backtracks must be an integer in [0,30]")
        reference = np.asarray(reference_positions, dtype=np.float64)
        self.faces = np.asarray(faces, dtype=np.int64).copy()
        self.parcel_mass = np.asarray(liquid_particle_mass, dtype=np.float64).copy()
        self.capacity = np.asarray(vertex_capacity, dtype=np.float64).copy()
        if reference.ndim != 2 or reference.shape[1] != 3 or not np.isfinite(reference).all():
            raise ValueError("reference_positions must be a finite (N,3) array")
        if self.faces.ndim != 2 or self.faces.shape[1] != 3 or not len(self.faces):
            raise ValueError("faces must contain triangles")
        if self.faces.min() < 0 or self.faces.max() >= len(reference):
            raise ValueError("Face index outside reference positions")
        if self.capacity.shape != (len(reference),) or np.any(self.capacity <= 0) or not np.isfinite(self.capacity).all():
            raise ValueError("vertex_capacity must be positive and match reference vertices")
        if self.parcel_mass.ndim != 1 or not len(self.parcel_mass) or np.any(self.parcel_mass <= 0) or not np.isfinite(self.parcel_mass).all():
            raise ValueError("liquid_particle_mass must be a positive vector")
        centered = reference-reference.mean(axis=0)
        _, _, basis = np.linalg.svd(centered, full_matrices=False)
        if np.max(np.abs(centered @ basis[2])) > 1e-7:
            raise ValueError("Wicking currently requires a planar initial material sheet")
        self.material_vertices = np.ascontiguousarray(centered @ basis[:2].T)
        triangles = self.material_vertices[self.faces]
        edge1, edge2 = triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0]
        matrices = np.stack((edge1, edge2), axis=2)
        determinants = np.linalg.det(matrices)
        if np.any(np.abs(determinants) < 1e-16):
            raise ValueError("Degenerate material triangle")
        inverses = np.linalg.inv(matrices)
        self.origins = np.ascontiguousarray(triangles[:, 0])
        self.gradients = np.ascontiguousarray(np.concatenate(
            (-inverses.sum(axis=1)[:, None, :], inverses), axis=1))
        max_edge = np.maximum.reduce((np.linalg.norm(edge1, axis=1), np.linalg.norm(edge2, axis=1),
                                      np.linalg.norm(edge1-edge2, axis=1)))
        self.face_altitudes = np.abs(determinants)/max_edge
        self.neighbors = np.full(self.faces.shape, -1, dtype=np.int64)
        edge_owners = {}
        for face, vertices in enumerate(self.faces):
            for opposite in range(3):
                key = tuple(sorted((int(vertices[(opposite+1) % 3]), int(vertices[(opposite+2) % 3]))))
                owners = edge_owners.setdefault(key, [])
                owners.append((face, opposite))
                if len(owners) > 2:
                    raise ValueError("Non-manifold material edge is unsupported")
        for owners in edge_owners.values():
            if len(owners) == 2:
                (a, ea), (b, eb) = owners
                self.neighbors[a, ea], self.neighbors[b, eb] = b, a
        self.reset()

    def reset(self):
        self._hop_credit = np.zeros(len(self.parcel_mass), dtype=np.float64)
        self._hop_target = np.full(len(self.parcel_mass), -1, dtype=np.int64)

    def capture_state(self):
        return {"hop_credit": self._hop_credit.copy(), "hop_target": self._hop_target.copy()}

    def restore_state(self, state):
        credit = np.asarray(state["hop_credit"], dtype=np.float64)
        target = np.asarray(state["hop_target"], dtype=np.int64)
        if (credit.shape != self._hop_credit.shape or target.shape != self._hop_target.shape
                or not np.isfinite(credit).all() or np.any(credit < 0) or np.any(credit > 1)
                or np.any(target < -1) or np.any(target >= len(self.faces))):
            raise ValueError("Invalid local wicking credit checkpoint")
        self._hop_credit[:] = credit
        self._hop_target[:] = target

    @classmethod
    def from_coupling(cls, coupling, config=None):
        return cls(coupling._initial_cloth_pos, coupling.faces,
                   coupling.liquid_particle_mass, coupling.vertex_capacity, config)

    def step(self, dt, bound_face, bound_weights, vertex_absorbed_mass,
             dry_vertex_mass, cloth_velocities, free_vertices):
        """Advance optional transport; return conservation and locality evidence.

        Writable arrays must retain their identity: faces are int64, weights,
        absorbed mass and velocities float64, free_vertices bool. Native calls
        are intentionally left to the caller's existing coupling step wrapper.
        """
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("Wicking dt must be finite and positive")
        report = {"enabled": bool(self.config["enabled"]), "moved_parcels": 0,
                  "crossed_faces": 0, "neighbor_hops": 0, "step_limited_parcels": 0,
                  "maximum_material_distance_m": 0.0,
                  "saturation_energy_change_kg": 0.0,
                  "support_impulse_on_system_kg_m_s": [0.0, 0.0, 0.0],
                  "momentum_residual_kg_m_s": [0.0, 0.0, 0.0],
                  "mass_change_kg": 0.0, "maximum_ledger_error_kg": 0.0}
        if not self.config["enabled"]:
            return report
        n, p = len(self.capacity), len(self.parcel_mass)
        arrays = ((bound_face, (p,), np.int64), (bound_weights, (p, 3), np.float64),
                  (vertex_absorbed_mass, (n,), np.float64), (cloth_velocities, (n, 3), np.float64))
        for value, shape, dtype in arrays:
            if not isinstance(value, np.ndarray) or value.shape != shape or value.dtype != dtype or not value.flags.writeable:
                raise ValueError(f"Wicking mutable arrays must be writable {dtype} with shape {shape}")
            if not np.isfinite(value).all():
                raise ValueError("Nonfinite wicking state")
        dry = np.asarray(dry_vertex_mass, dtype=np.float64)
        free = np.asarray(free_vertices, dtype=bool)
        if dry.shape != (n,) or np.any(dry <= 0) or not np.isfinite(dry).all() or free.shape != (n,):
            raise ValueError("Invalid dry mass or free-vertex mask")
        if bound_face.min(initial=-1) < -1 or bound_face.max(initial=-1) >= len(self.faces):
            raise ValueError("Invalid bound face index")
        bound = np.flatnonzero(bound_face >= 0)
        weights = bound_weights[bound]
        if np.any(weights < -1e-12) or np.any(np.abs(weights.sum(axis=1)-1) > 1e-10):
            raise ValueError("Bound barycentric weights must be nonnegative and sum to one")
        if np.any(vertex_absorbed_mass < -1e-14) or np.any(vertex_absorbed_mass > self.capacity+1e-14):
            raise ValueError("Initial wet mass violates finite capacity")
        def ledger():
            mass = np.zeros(n)
            np.add.at(mass, self.faces[bound_face[bound]].ravel(),
                      (self.parcel_mass[bound, None]*bound_weights[bound]).ravel())
            return mass
        if np.max(np.abs(ledger()-vertex_absorbed_mass), initial=0) > 1e-10:
            raise ValueError("Initial parcel and vertex wet-mass ledgers disagree")
        before_mass = float(vertex_absorbed_mass.sum())
        before_momentum = ((dry+vertex_absorbed_mass)[:, None]*cloth_velocities).sum(axis=0)
        support, moved, crossed, limited, max_distance, energy, hops = _redistribute(
            dt, self.config["diffusivity_m2_s"], self.config["saturation_floor"],
            self.config["maximum_material_step_m"], self.config["maximum_face_altitude_fraction"],
            self.config["maximum_backtracks"], self.config["maximum_neighbor_hop_m"],
            self._hop_credit, self._hop_target, self.material_vertices, self.faces,
            self.origins, self.gradients, self.neighbors, self.face_altitudes,
            self.parcel_mass, self.capacity, dry, bound_face, bound_weights,
            vertex_absorbed_mass, cloth_velocities, free)
        after_momentum = ((dry+vertex_absorbed_mass)[:, None]*cloth_velocities).sum(axis=0)
        report.update(moved_parcels=int(moved), crossed_faces=int(crossed), neighbor_hops=int(hops),
            step_limited_parcels=int(limited), maximum_material_distance_m=float(max_distance),
            saturation_energy_change_kg=float(energy),
            support_impulse_on_system_kg_m_s=support.tolist(),
            momentum_residual_kg_m_s=(after_momentum-before_momentum-support).tolist(),
            mass_change_kg=float(vertex_absorbed_mass.sum()-before_mass),
            maximum_ledger_error_kg=float(np.max(np.abs(ledger()-vertex_absorbed_mass), initial=0)))
        return report
