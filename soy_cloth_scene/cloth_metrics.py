"""Material-coordinate strain and projected coverage, independent of rendering.

Edge strain is extension divided by rest length, not vertical displacement.
Projected coverage is the union of triangle projections sampled on a fixed
1 mm world grid: folded/overlapping material is counted only once.
"""
import numpy as np


def _direction_limits(value):
    if value is None:
        return None
    if not isinstance(value, dict) or set(value) != {"warp", "weft", "bias"}:
        raise ValueError("direction_limits must contain exactly warp, weft and bias")
    limits = {key: float(value[key]) for key in ("warp", "weft", "bias")}
    if any(not np.isfinite(v) or v < 0 for v in limits.values()):
        raise ValueError("Directional extension limits must be finite and nonnegative")
    return limits


class ClothMetrics:
    def __init__(self, positions, faces, direction_limits=None, *, residual_tolerance=.002):
        """Measure edges in fixed reference material X/Y directions.

        Optional directional limits apply to an initially planar XY mesh:
        X-aligned edges are warp, Y-aligned edges weft, others bias. They are
        classified once from reference coordinates, never from the moving
        world orientation. residual_tolerance is a numerical acceptance margin
        in absolute strain (0.002 = 0.2 percentage points), not a material limit.
        """
        self.reference = np.asarray(positions, dtype=float).copy()
        self.faces = np.asarray(faces, dtype=int).copy()
        self.direction_limits = _direction_limits(direction_limits)
        self.residual_tolerance = float(residual_tolerance)
        if not np.isfinite(self.residual_tolerance) or self.residual_tolerance < 0:
            raise ValueError("residual_tolerance must be finite and nonnegative")
        self.edges = np.unique(np.sort(self.faces[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1), axis=0)
        self.rest_lengths = np.linalg.norm(self.reference[self.edges[:, 1]]-self.reference[self.edges[:, 0]], axis=1)
        if np.any(self.rest_lengths <= 1e-12):
            raise ValueError("Cannot measure strain of a zero-length reference edge")
        self.edge_directions = None
        if self.direction_limits is not None:
            delta = np.abs(self.reference[self.edges[:, 1]]-self.reference[self.edges[:, 0]])
            axis_tolerance = np.maximum(1e-10, self.rest_lengths*1e-5)
            if np.ptp(self.reference[:, 2]) > 1e-7:
                raise ValueError("Directional metrics require the initial planar XY material reference")
            self.edge_directions = np.full(len(self.edges), "bias", dtype="U4")
            self.edge_directions[(delta[:, 1] <= axis_tolerance) & (delta[:, 2] <= axis_tolerance)] = "warp"
            self.edge_directions[(delta[:, 0] <= axis_tolerance) & (delta[:, 2] <= axis_tolerance)] = "weft"
        tri = self.reference[self.faces]
        self.rest_area = float(np.linalg.norm(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]), axis=1).sum()/2)
        self.reference_footprint = self.footprint(self.reference)

    def strain(self, positions):
        p = np.asarray(positions)
        extension = np.linalg.norm(p[self.edges[:, 1]]-p[self.edges[:, 0]], axis=1)/self.rest_lengths-1
        result = {"edge_extension_p95": float(np.percentile(extension, 95)),
                "edge_extension_p99": float(np.percentile(extension, 99)),
                "edge_extension_max": float(extension.max()),
                "edges_extended_over_5pct": int(np.sum(extension > .05))}
        if self.direction_limits is not None:
            result["directional_residual_tolerance"] = self.residual_tolerance
            for direction, limit in self.direction_limits.items():
                values = extension[self.edge_directions == direction]
                result.update({
                    f"{direction}_edge_count": int(len(values)),
                    f"{direction}_extension_limit": limit,
                    f"{direction}_edge_extension_max": float(values.max()) if len(values) else None,
                    f"{direction}_edge_extension_p99": float(np.percentile(values, 99)) if len(values) else None,
                    f"{direction}_bound_excess_max": float(np.maximum(values-limit, 0).max()) if len(values) else None,
                    f"{direction}_edges_over_limit": int(np.sum(values > limit)),
                    f"{direction}_edges_over_limit_plus_residual": int(np.sum(values > limit+self.residual_tolerance)),
                })
        return result

    def footprint(self, positions, max_height=None, resolution=.001):
        # Reference geometry is float64; live particle snapshots are usually
        # float32. Use the same arithmetic for both so barycentric boundary
        # rounding cannot change coverage after a purely vertical translation.
        p = np.asarray(positions, dtype=np.float64)
        occupied = set()
        for tri in p[self.faces]:
            if max_height is not None and tri[:, 2].max() > max_height:
                continue
            xy = tri[:, :2]
            low = np.ceil(xy.min(axis=0)/resolution).astype(int)
            high = np.floor(xy.max(axis=0)/resolution).astype(int)
            if (high < low).any():
                continue
            a, b = xy[1]-xy[0], xy[2]-xy[0]
            determinant = a[0]*b[1]-a[1]*b[0]
            if abs(determinant) < 1e-12:
                continue
            grid = np.stack(np.meshgrid(np.arange(low[0], high[0]+1),
                                       np.arange(low[1], high[1]+1), indexing="ij"), -1).reshape(-1, 2)
            d = grid*resolution-xy[0]
            u = (d[:, 0]*b[1]-d[:, 1]*b[0])/determinant
            v = (a[0]*d[:, 1]-a[1]*d[:, 0])/determinant
            keep = (u >= -1e-7) & (v >= -1e-7) & (u+v <= 1+1e-7)
            occupied.update(map(tuple, grid[keep]))
        return len(occupied)*resolution**2

    def measure(self, positions, table_height):
        projected = self.footprint(positions)
        flat = self.footprint(positions, max_height=table_height+.012)
        return {**self.strain(positions), "rest_area_m2": self.rest_area,
                "projection_union_m2": projected,
                "projection_fraction_of_initial": projected/self.reference_footprint,
                "within_12mm_of_table_union_m2": flat,
                "near_table_fraction_of_initial": flat/self.reference_footprint,
                "area_measurement_grid_m": .001}


def acceptance(stages, direction_limits=None, *, residual_tolerance=.002):
    """End-stage engineering checks; unchanged legacy checks without limits.

    Directional mode checks each actual maximum against its separately reported
    material limit plus the explicit numerical margin. It does not substitute
    the old whole-mesh 5% cap for a 10% bias material target. Missing directional
    evidence, a different target, or a different margin fails the relevant
    check rather than accepting a stale legacy-stage record.
    """
    lifted = [stages.get(k) for k in ("04_cloth_lifted", "12_wet_cloth_lifted")]
    laid = stages.get("08_after_pressing", {})
    limits = _direction_limits(direction_limits)
    if not np.isfinite(residual_tolerance) or residual_tolerance < 0:
        raise ValueError("residual_tolerance must be finite and nonnegative")
    if limits is None:
        checks = {
        "dry_and_wet_lift_p99_extension_below_2pct": all(
            m is not None and m["edge_extension_p99"] <= .02 for m in lifted),
        "dry_and_wet_lift_peak_extension_below_5pct": all(
            m is not None and m["edge_extension_max"] <= .05 for m in lifted),
        }
    else:
        checks = {}
        for direction, limit in limits.items():
            checks[f"dry_and_wet_lift_{direction}_extension_within_limit_and_residual"] = all(
                m is not None and m.get(f"{direction}_edge_count", 0) > 0
                and m.get(f"{direction}_extension_limit") == limit
                and m.get("directional_residual_tolerance") == residual_tolerance
                and m.get(f"{direction}_edge_extension_max") is not None
                and np.isfinite(m[f"{direction}_edge_extension_max"])
                and m[f"{direction}_edge_extension_max"] <= limit+residual_tolerance+1e-12
                for m in lifted)
    checks.update({
        "laid_projected_area_at_least_85pct": laid.get("projection_fraction_of_initial", 0) >= .85,
        "laid_near_table_area_at_least_85pct": laid.get("near_table_fraction_of_initial", 0) >= .85,
    })
    return checks
