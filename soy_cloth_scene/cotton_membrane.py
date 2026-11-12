"""Finite yarn extension for the unchanged cotton mesh, local to this scene.

The native membrane solve remains active. This adapter adds tension-only upper
bounds on its existing edges, using the original horizontal cloth's X and Y as
warp/weft directions. Diagonal edges receive a looser bound. Disjoint edge colors
allow in-place, inverse-mass-weighted projections without Jacobi overshoot.

These are numerical finite-extension limits, not measured material constants.
Finite iterations and subsequent bending/contact projections can leave a residual
above a bound; ``measure()`` reports that residual rather than promising a hard
global strain guarantee. Compression, folds, and particle targets are untouched.
"""

# Ported from reality_latest / runs/plush-cotton-complete-01/source-at-run.
# The material archive follows upstream baseline abe0008; no Genesis source edit.
from importlib.metadata import version

import numpy as np


def edge_constraints(points, faces, warp_limit=.02, weft_limit=.02, bias_limit=.10):
    """Return color-ordered original edges, rest lengths, bounds and directions."""
    points = np.asarray(points, dtype=np.float64)
    faces = np.asarray(faces)
    limits = np.asarray([warp_limit, weft_limit, bias_limit], dtype=np.float64)
    if not np.isfinite(limits).all() or np.any(limits < 0) or np.any(limits >= 1):
        raise ValueError("Cotton extension limits must be finite fractions in [0, 1)")
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("Cotton points must be finite N by 3 coordinates")
    if (faces.ndim != 2 or faces.shape[1] != 3
            or not np.issubdtype(faces.dtype, np.integer)):
        raise ValueError("Cotton faces must be integer triangle indices")
    if faces.size == 0 or faces.min() < 0 or faces.max() >= len(points):
        raise ValueError("Cotton faces must reference existing particles")
    if np.ptp(points[:, 2]) > 1e-6 * max(1., float(np.ptp(points[:, :2], axis=0).max())):
        raise ValueError("Warp/weft identification expects the original horizontal XY cloth")
    edges = np.unique(np.sort(np.concatenate([
        faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1), axis=0).astype(np.int32)
    delta = points[edges[:, 1]] - points[edges[:, 0]]
    lengths = np.linalg.norm(delta, axis=1)
    if np.any(lengths <= 1e-12):
        raise ValueError("Cotton has collapsed mesh edges")
    directions = np.full(len(edges), 2, dtype=np.int32)  # Diagonal/bias.
    directions[np.abs(delta[:, 1]) <= 1e-6 * lengths] = 0
    directions[np.abs(delta[:, 0]) <= 1e-6 * lengths] = 1
    if not all(np.any(directions == direction) for direction in (0, 1)):
        raise ValueError("The cloth mesh must contain both X-warp and Y-weft edges")
    used, groups = [set() for _ in points], []
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
    color_bounds = np.cumsum([0] + [len(group) for group in groups]).tolist()
    return (edges[order], lengths[order], lengths[order] * (1. + limits[directions[order]]),
            directions[order], color_bounds)


def install_cotton_membrane(scene, cloth, points, faces, warp_limit=.02,
                            weft_limit=.02, bias_limit=.10, iterations=24):
    """Install before build; no per-step refresh/reset is needed.

    Each native stretch call is followed by ``iterations`` colored sweeps over
    the cotton's original edges. Only excessive extension is projected; fixed
    and link-attached particles have zero inverse mass and remain unchanged.
    """
    import genesis as gs
    import quadrants as qd
    import torch

    if version("genesis-world") != "1.4.3" or scene.is_built:
        raise RuntimeError("Install the Genesis 1.4.3 cotton membrane adapter before build")
    if isinstance(iterations, bool) or int(iterations) != iterations or iterations < 1:
        raise ValueError("Cotton membrane iterations must be a positive integer")
    if len(points) != cloth.n_particles:
        raise ValueError("Cotton particles must match the original mesh")
    edges, rest, maximum, directions, bounds = edge_constraints(
        points, faces, warp_limit, weft_limit, bias_limit)
    if len(edges) != len(cloth._edges):
        raise ValueError("Cotton membrane requires the original mesh edges")

    @qd.data_oriented
    class CottonMembrane:
        def __init__(self):
            self.solver, self.cloth = cloth.solver, cloth
            self.previous_solve = self.solver._kernel_solve_stretch
            self.edges = torch.as_tensor(np.ascontiguousarray(edges + cloth.particle_start),
                                         dtype=gs.tc_int, device=gs.device)
            self.maximum = torch.as_tensor(maximum, dtype=gs.tc_float, device=gs.device)
            self.iterations, self.color_bounds = int(iterations), bounds
            self.colors = len(bounds) - 1
            self.solver._kernel_solve_stretch = self.solve
            self.description = {
                "model": "tension-only finite extension of existing woven-cloth mesh edges",
                "warp_extension_limit_fraction": float(warp_limit),
                "weft_extension_limit_fraction": float(weft_limit),
                "bias_extension_limit_fraction": float(bias_limit),
                "material_axes": "original horizontal cloth X=warp, Y=weft; other edges=bias",
                "edge_counts": {name: int(np.sum(directions == k))
                                for k, name in enumerate(("warp", "weft", "bias"))},
                "solver": "inverse-mass-weighted in-place projections on disjoint edge colors",
                "colors": self.colors,
                "sweeps_per_native_stretch_call": self.iterations,
                "coupled_constraint_residual": "finite sweeps and later bending/contact may exceed bounds; inspect measure()",
                "native_stretch_retained": True,
                "compression_constrained": False,
                "fixed_and_link_attached_particles_unchanged": True,
                "each_edge_correction_equal_opposite_and_central": True,
                "measured_fabric_calibration": False,
                "mesh_and_particle_targets_unchanged": True,
            }

        def solve(self, f):
            self.previous_solve(f)
            self.enforce()

        def enforce(self):
            self.project(self.edges, self.maximum)

        @qd.kernel
        def project(self, pairs: qd.types.ndarray(), maximum_length: qd.types.ndarray()):
            for _ in qd.static(range(self.iterations)):
                for color in qd.static(range(self.colors)):
                    for edge, batch in qd.ndrange(
                            (self.color_bounds[color], self.color_bounds[color + 1]), self.solver._B):
                        a, b = pairs[edge, 0], pairs[edge, 1]
                        wa = self.solver.particles[a, batch].free / self.solver.particles_info[a].mass
                        wb = self.solver.particles[b, batch].free / self.solver.particles_info[b].mass
                        delta = self.solver.particles[b, batch].pos - self.solver.particles[a, batch].pos
                        length = delta.norm()
                        if wa + wb > 0. and length > maximum_length[edge]:
                            correction = (length - maximum_length[edge]) / ((wa + wb) * length) * delta
                            self.solver.particles[a, batch].pos += wa * correction
                            self.solver.particles[b, batch].pos -= wb * correction

        def measure(self, positions=None):
            if positions is None:
                positions = self.cloth.get_particles_pos().detach().cpu().numpy()
            positions = np.asarray(positions)
            if positions.shape != (cloth.n_particles, 3) or not np.isfinite(positions).all():
                raise ValueError("Membrane diagnostics require finite local cotton positions")
            lengths = np.linalg.norm(positions[edges[:, 1]] - positions[edges[:, 0]], axis=1)
            strain = lengths / rest - 1.
            excess = np.maximum((lengths - maximum) / rest, 0.)
            return {
                "maximum_extension_fraction": float(strain.max()),
                "p95_absolute_edge_strain": float(np.percentile(np.abs(strain), 95)),
                "maximum_bound_excess_fraction": float(excess.max()),
                "directions": {name: {
                    "maximum_extension_fraction": float(strain[directions == k].max()),
                    "p95_extension_fraction": float(np.percentile(strain[directions == k], 95)),
                    "maximum_bound_excess_fraction": float(excess[directions == k].max()),
                } for k, name in enumerate(("warp", "weft", "bias")) if np.any(directions == k)},
            }

        def close(self):
            if self.solver._kernel_solve_stretch == self.solve:
                self.solver._kernel_solve_stretch = self.previous_solve

    return CottonMembrane()
