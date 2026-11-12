"""Scene-local, geometry-scaled XPBD bending for the existing cotton mesh.

Install before ``scene.build()``. The material controller must call ``reset()``
once at the beginning of *each simulation substep*, before any bending solve.
Multipliers persist across all iterations/calls within that substep. This module
only replaces ``solver._kernel_solve_bending``; membrane/stretch, contacts,
particle masses, mesh topology, and scripted motion are left to the scene.
Four-vertex-disjoint hinge colors are solved in sequence. This numerical
Gauss-Seidel scheduling differs from the source archive's summed Jacobi updates,
which amplify a fine-grid checkerboard mode; the hinge energy is unchanged.

For plate bending rigidity B [N m], each interior edge has energy
``0.5 * (3 B l_rest**2 / (A_left + A_right)) * (theta - theta_rest)**2``.
This uses the discrete-shell geometric weight, with the physical rigidity B
equal to twice the energy coefficient used in Grinspun et al., Discrete Shells:
https://multires.caltech.edu/pubs/ds.pdf
It is a cloth approximation, not a measured constitutive fit to this fabric.
"""

# Ported from reality_latest / runs/plush-cotton-complete-01/source-at-run.
# The material archive follows upstream baseline abe0008; no Genesis source edit.
from importlib.metadata import version

import numpy as np


def hinge_angle_gradient(positions):
    """Return signed dihedral and its four position gradients (NumPy reference).

    Vertices 0,1 form the hinge. Vertices 2,3 are opposite vertices of its
    adjacent triangles. The normals use (0,1,2) and (1,0,3), so coplanar
    consistently oriented triangles have angle zero, without an acos singularity.
    """
    p = np.asarray(positions, dtype=np.float64)
    if p.shape != (4, 3) or not np.isfinite(p).all():
        raise ValueError("A hinge needs four finite 3D points")
    edge = p[1] - p[0]
    edge2 = float(edge @ edge)
    n1 = np.cross(edge, p[2] - p[0])
    n2 = np.cross(p[3] - p[0], edge)
    n12, n22 = float(n1 @ n1), float(n2 @ n2)
    if edge2 <= 1e-24 or min(n12, n22) <= 1e-24:
        raise ValueError("Collapsed hinge edge or incident triangle")
    length = np.sqrt(edge2)
    a, b = n1 / np.sqrt(n12), n2 / np.sqrt(n22)
    angle = np.arctan2((edge / length) @ np.cross(a, b), a @ b)
    g3, g4 = -length * n1 / n12, -length * n2 / n22
    t3 = (p[2] - p[0]) @ edge / edge2
    t4 = (p[3] - p[0]) @ edge / edge2
    g2 = -t3 * g3 - t4 * g4
    g1 = -g2 - g3 - g4
    return float(angle), np.asarray([g1, g2, g3, g4])


def build_hinges(points, faces, rigidity):
    """Build interior-edge indices, rest angles and angular compliance [1/(N m)]."""
    points = np.asarray(points, dtype=np.float64)
    faces_input = np.asarray(faces)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("Cloth points must be finite N by 3 coordinates")
    if (faces_input.ndim != 2 or faces_input.shape[1] != 3
            or not np.issubdtype(faces_input.dtype, np.integer)):
        raise ValueError("Cloth faces must be an integer M by 3 array")
    faces = faces_input.astype(np.int32)
    if faces.size and (faces.min() < 0 or faces.max() >= len(points)):
        raise ValueError("Cloth triangle index outside the particle array")
    if not np.isfinite(rigidity) or rigidity <= 0:
        raise ValueError("Bending rigidity must be finite and positive, in N m")
    incident = {}
    for triangle in faces:
        if len(set(map(int, triangle))) != 3:
            raise ValueError("Cloth contains a triangle with repeated vertices")
        a, b, c = map(int, triangle)
        for u, v, opposite in ((a, b, c), (b, c, a), (c, a, b)):
            incident.setdefault(tuple(sorted((u, v))), []).append((u, v, opposite))
    hinges, angles, compliance = [], [], []
    for adjacent in incident.values():
        if len(adjacent) > 2:
            raise ValueError("Cotton bending requires a manifold triangle mesh")
        if len(adjacent) != 2:
            continue  # Free hems and the existing arm-hole boundaries.
        a, b, c = adjacent[0]
        u, v, d = adjacent[1]
        if u != b or v != a:
            raise ValueError("Adjacent cloth faces must have consistent winding")
        indices = [a, b, c, d]
        rest = points[indices]
        angle, _ = hinge_angle_gradient(rest)
        edge = rest[1] - rest[0]
        area = 0.5 * (np.linalg.norm(np.cross(edge, rest[2] - rest[0]))
                      + np.linalg.norm(np.cross(rest[3] - rest[0], edge)))
        stiffness = 3.0 * float(rigidity) * float(edge @ edge) / area
        hinges.append(indices)
        angles.append(angle)
        compliance.append(1.0 / stiffness)
    return (np.asarray(hinges, dtype=np.int32).reshape(-1, 4),
            np.asarray(angles), np.asarray(compliance))


def disjoint_hinge_colors(hinges, n_particles):
    """Return original-row order and boundaries of four-vertex-disjoint colors.

    Colors run sequentially, with simultaneous in-place updates safe within a
    color. Reordering must also be applied to rest angles and compliance so
    each multiplier retains its own constraint throughout a simulation substep.
    """
    ids = np.asarray(hinges)
    if (isinstance(n_particles, (bool, np.bool_))
            or not isinstance(n_particles, (int, np.integer)) or n_particles < 0):
        raise ValueError("The particle count must be a nonnegative integer")
    if ids.ndim != 2 or ids.shape[1] != 4 or not np.issubdtype(ids.dtype, np.integer):
        raise ValueError("Hinges must be an integer N by 4 array")
    if ids.size and (ids.min() < 0 or ids.max() >= n_particles):
        raise ValueError("Hinge index outside the particle array")
    if len(ids) and np.any(np.diff(np.sort(ids, axis=1), axis=1) == 0):
        raise ValueError("Every hinge needs four distinct particles")
    used, groups = [set() for _ in range(n_particles)], []
    for index, vertices in enumerate(ids):
        unavailable = set.union(*(used[v] for v in vertices))
        color = 0
        while color in unavailable:
            color += 1
        if color == len(groups):
            groups.append([])
        groups[color].append(index)
        for vertex in vertices:
            used[vertex].add(color)
    order = np.asarray([index for group in groups for index in group], dtype=np.int32)
    bounds = np.cumsum([0] + [len(group) for group in groups]).tolist()
    return order, bounds


def install_cotton_bending(scene, cloth, points, faces, rigidity=5e-5, relaxation=0.5):
    """Install before build; call returned ``reset()`` once at each substep start.

    ``rigidity`` is plate/fabric bending rigidity in N m, not an XPBD compliance.
    Relaxation under-relaxes both the multiplier and its position correction.
    The hook is intentionally restricted to the scene's sole bending entity.
    """
    import genesis as gs
    import quadrants as qd
    import torch

    if version("genesis-world") != "1.4.3":
        raise RuntimeError("Review the private PBD hook before another Genesis version")
    if scene.is_built:
        raise RuntimeError("Install cotton bending before scene.build()")
    if not np.isfinite(relaxation) or not 0 < relaxation <= 1:
        raise ValueError("Bending relaxation must be in (0, 1]")
    hinges, angles, compliance = build_hinges(points, faces, rigidity)
    if len(points) != cloth.n_particles:
        raise ValueError("Cotton particle count does not match the supplied mesh")
    if len(hinges) != cloth.n_inner_edges or cloth.solver.n_inner_edges != len(hinges):
        raise ValueError("Cotton must be the solver's sole bending entity with unchanged topology")
    order, bounds = disjoint_hinge_colors(hinges, len(points))
    hinges, angles, compliance = hinges[order], angles[order], compliance[order]

    @qd.data_oriented
    class CottonBending:
        def __init__(self):
            self.solver, self.cloth = cloth.solver, cloth
            self.previous_solve = self.solver._kernel_solve_bending
            self.relaxation = float(relaxation)
            self.color_bounds, self.colors = bounds, len(bounds) - 1
            self.hinges = torch.as_tensor(hinges + cloth.particle_start,
                                         dtype=gs.tc_int, device=gs.device)
            self.rest = torch.as_tensor(angles, dtype=gs.tc_float, device=gs.device)
            self.compliance = torch.as_tensor(compliance, dtype=gs.tc_float, device=gs.device)
            self.multipliers = None
            self.solver._kernel_solve_bending = self.solve
            self.description = {
                "model": "XPBD signed-dihedral discrete-shell bending",
                "bending_rigidity_N_m": float(rigidity),
                "angular_stiffness": "3 * B * rest_edge_length^2 / sum_adjacent_rest_areas",
                "hinge_energy": "0.5 * angular_stiffness * wrapped_angle_error^2",
                "rest_angles": "original cloth mesh",
                "interior_hinges": len(hinges),
                "free_boundary_edges": True,
                "relaxation": self.relaxation,
                "solver": "in-place inverse-mass-weighted Gauss-Seidel on disjoint four-node hinge colors",
                "colors": self.colors,
                "reference_numerical_difference": "Sequential disjoint hinge colors replace source-archive summed Jacobi to suppress fine-grid checkerboard amplification; energy, rigidity, relaxation and iteration count unchanged",
                "iterations_per_solve": int(self.solver._max_bending_solver_iterations),
                "multiplier_reset": "once per simulation substep by material_response",
                "flat_angle_gradient": "finite analytic gradient; atan2 signed angle",
                "stretch_modified": False,
                "measured_fabric_calibration": False,
                "reference": "https://multires.caltech.edu/pubs/ds.pdf",
            }

        def reset(self):
            """Zero XPBD history once per substep, including build-time warmup."""
            if (getattr(self.solver, "_B", 0) < 1
                    or not hasattr(self.solver, "particles")
                    or not hasattr(self.solver, "particles_info")):
                raise RuntimeError("Cotton bending reset requires initialized PBD particle fields")
            if self.multipliers is None:
                self.multipliers = torch.zeros((len(hinges), self.solver._B),
                                               dtype=gs.tc_float, device=gs.device)
            else:
                self.multipliers.zero_()

        def solve(self, f):
            if self.multipliers is None:
                raise RuntimeError("Call cotton bending reset() at each substep start")
            self.project(self.hinges, self.rest, self.compliance, self.multipliers)

        @qd.kernel
        def project(self, ids: qd.types.ndarray(), rest: qd.types.ndarray(),
                    compliance: qd.types.ndarray(), multipliers: qd.types.ndarray()):
            for _ in qd.static(range(self.solver._max_bending_solver_iterations)):
                for color in qd.static(range(self.colors)):
                    for k, batch in qd.ndrange(
                            (self.color_bounds[color], self.color_bounds[color + 1]), self.solver._B):
                        v1, v2, v3, v4 = ids[k, 0], ids[k, 1], ids[k, 2], ids[k, 3]
                        p1 = self.solver.particles[v1, batch].pos
                        p2 = self.solver.particles[v2, batch].pos
                        p3 = self.solver.particles[v3, batch].pos
                        p4 = self.solver.particles[v4, batch].pos
                        edge = p2 - p1
                        edge2 = edge.norm_sqr()
                        n1, n2 = edge.cross(p3 - p1), (p4 - p1).cross(edge)
                        n12, n22 = n1.norm_sqr(), n2.norm_sqr()
                        # A collapsed face has no defined hinge angle. Contact and
                        # membrane constraints handle it; never divide by zero here.
                        if edge2 > 1e-20 and n12 > 1e-24 and n22 > 1e-24:
                            length = qd.sqrt(edge2)
                            a, b = n1 / qd.sqrt(n12), n2 / qd.sqrt(n22)
                            theta = qd.atan2((edge / length).dot(a.cross(b)), a.dot(b))
                            error = theta - rest[k]
                            error = qd.atan2(qd.sin(error), qd.cos(error))
                            g3, g4 = -length * n1 / n12, -length * n2 / n22
                            g2 = -(p3 - p1).dot(edge) / edge2 * g3 - (p4 - p1).dot(edge) / edge2 * g4
                            g1 = -g2 - g3 - g4
                            gradient = qd.Matrix.rows([g1, g2, g3, g4])
                            inverse_mass = qd.Vector.zero(gs.qd_float, 4)
                            denominator = gs.qd_float(0.)
                            for j in qd.static(range(4)):
                                node = ids[k, j]
                                inverse_mass[j] = (self.solver.particles[node, batch].free
                                                   / self.solver.particles_info[node].mass)
                                denominator += inverse_mass[j] * gradient[j, :].norm_sqr()
                            if denominator > 0.:
                                alpha = compliance[k] / (self.solver._substep_dt ** 2)
                                delta_lambda = (self.relaxation * (-error - alpha * multipliers[k, batch])
                                                / (denominator + alpha))
                                multipliers[k, batch] += delta_lambda
                                # Every hinge in this color has disjoint vertices.
                                # Apply the same delta-lambda directly; never sum
                                # multiple stale corrections at a shared particle.
                                for j in qd.static(range(4)):
                                    self.solver.particles[ids[k, j], batch].pos += (
                                        inverse_mass[j] * delta_lambda * gradient[j, :])

        def close(self):
            """Restore the previous instance hook when this remains the active hook."""
            if self.solver._kernel_solve_bending == self.solve:
                self.solver._kernel_solve_bending = self.previous_solve

    return CottonBending()
